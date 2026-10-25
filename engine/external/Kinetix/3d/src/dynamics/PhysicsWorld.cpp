#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/PhysicsWorld.h"

#include <kinetix/core/DebugDraw3D.h>
#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "dynamics/Joint.h"

#include <kinetix/core/Algorithm.h>

#include <assert.h>
#include <math.h>

namespace kinetix::Physics
{

namespace
{
// How close a new contact point must be to a cached one to count as the same
// point: too tight discards the warm-start impulse, too loose lets a point that
// slid across a face inherit an impulse meant for elsewhere.
constexpr f32 kMatchDistance = 0.02f;

bool finiteVector(const Math::vec3& value)
{
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

f32 radialWeight(const Math::vec3& centre, const Math::vec3& position, f32 radius,
                 Math::vec3* direction = nullptr)
{
    // Linear attenuation; applies a body-level impulse/force, not one scaled by
    // exposed or projected area.
    const Math::vec3 offset = position - centre;
    const f32 distanceSquared = Math::dot(offset, offset);
    if (distanceSquared >= radius * radius)
        return 0.0f;

    const f32 distance = sqrtf(distanceSquared);
    if (direction)
        *direction = distance > 1.0e-6f ? offset / distance : Math::vec3(0.0f, 1.0f, 0.0f);
    return 1.0f - distance / radius;
}

// A kinematic body that is actually going somewhere. What rests on or hangs
// from it must go too, and nothing else wakes it: the contact is not new and
// kinematic bodies are kept out of the sleep islands.
bool movingKinematic(const RigidBody* body)
{
    return body && body->bodyType() == BodyType::Kinematic &&
           (Math::dot(body->velocity(), body->velocity()) > 0.0f ||
            Math::dot(body->angularVelocity(), body->angularVelocity()) > 0.0f);
}

bool resting(const RigidBody& body)
{
    return body.bodyType() == BodyType::Static ||
           (body.bodyType() == BodyType::Dynamic && !body.awake());
}
} // namespace

PhysicsWorld::PhysicsWorld()
{
}

void PhysicsWorld::addJoint(Joint* joint)
{
    if (!joint || !joint->bodyA() || !joint->bodyB())
        return;
    if (joint->bodyA() == joint->bodyB() && !joint->singleBody())
        return;
    if (kinetix::Find(mJoints.begin(), mJoints.end(), joint) != mJoints.end())
        return;
    mJoints.push_back(joint);
    joint->bodyA()->setAwake(true);
    joint->bodyB()->setAwake(true);
}

void PhysicsWorld::removeJoint(Joint* joint)
{
    const auto found = kinetix::Find(mJoints.begin(), mJoints.end(), joint);
    if (!joint || found == mJoints.end())
        return;
    *found = mJoints.back();
    mJoints.pop_back();
    // A body held up by this joint alone may be asleep hanging from it.
    wakeBody(joint->bodyA());
    wakeBody(joint->bodyB());
}

void PhysicsWorld::wakeBody(RigidBody* body)
{
    // Even when already awake: setAwake() also reseeds the motion average, so a
    // body about to fall asleep does not do so the step its support disappears.
    if (body && body->isDynamic())
        body->setAwake(true);
}

u64 PhysicsWorld::pairKey(u32 a, u32 b)
{
    const u32 low = a < b ? a : b;
    const u32 high = a < b ? b : a;
    return (static_cast<u64>(low) << 32) | high;
}

void PhysicsWorld::setPairIgnored(u32 idA, u32 idB, bool ignored)
{
    if (idA == idB)
        return;
    if (ignored)
        mIgnoredPairs[pairKey(idA, idB)] = 1;
    else
        mIgnoredPairs.erase(pairKey(idA, idB));
}

bool PhysicsWorld::pairIgnored(u32 idA, u32 idB) const
{
    return mIgnoredPairs.find(pairKey(idA, idB)) != nullptr;
}

u32 PhysicsWorld::addBody(const BodyEntry& entry)
{
    if (!entry.body || !entry.shape)
        return InvalidIndex;
    if (mClearPending)
    {
        Log::error("PhysicsWorld: cannot add a body after clear() was requested during a step");
        return InvalidIndex;
    }
    if (entry.body->isDynamic() &&
        (entry.shape->type() == ShapeType::Trimesh || entry.shape->type() == ShapeType::Plane))
    {
        Log::error("PhysicsWorld: a dynamic body cannot use a static collision shape");
        return InvalidIndex;
    }
    // Sleep is the world's decision, not the body's (see propagateSleep()).
    entry.body->setSleepDeferred(true);
    const u32 id = allocateId();
    if (mStepping)
    {
        mPendingAdds.push_back({id, entry});
        return id;
    }
    insertBody(id, entry);
    return id;
}

u32 PhysicsWorld::allocateId()
{
    if (!mFreeIds.empty())
    {
        const u32 id = mFreeIds.back();
        mFreeIds.pop_back();
        mBodyGenerations[id] = mNextBodyGeneration++;
        return id;
    }

    const u32 id = static_cast<u32>(mIdToSlot.size());
    mIdToSlot.push_back(InvalidIndex);
    mBodyGenerations.push_back(mNextBodyGeneration++);
    return id;
}

u32 PhysicsWorld::slotForId(u32 id) const
{
    return id < mIdToSlot.size() ? mIdToSlot[id] : InvalidIndex;
}

void PhysicsWorld::insertBody(u32 id, const BodyEntry& entry)
{
    assert(id < mIdToSlot.size());
    assert(mIdToSlot[id] == InvalidIndex);
    mIdToSlot[id] = static_cast<u32>(mBodies.size());
    mBodySlots[entry.body] = static_cast<u32>(mBodies.size());
    mBodies.push_back(entry);
    mBodyIds.push_back(id);
    if (entry.body->bodyType() == BodyType::Static)
        mStaticBroadphaseDirty = true;
}

void PhysicsWorld::removeBody(u32 id)
{
    if (id >= mBodyGenerations.size() || mBodyGenerations[id] == 0)
        return;

    if (mStepping)
    {
        // An add requested earlier in this callback has no slot yet and can be
        // cancelled without ever reaching the simulation.
        for (auto it = mPendingAdds.begin(); it != mPendingAdds.end(); ++it)
            if (it->id == id)
            {
                it->entry.body->setSleepDeferred(false);
                mPendingAdds.erase(it);
                mBodyGenerations[id] = 0;
                mFreeIds.push_back(id);
                return;
            }
        // The slot is released only once the step is over, but the body is let go
        // now: the owner may destroy it as soon as this returns, and the rest of
        // the step (sleep islands, the flush) must not reach it. Only the step
        // callback runs while stepping, after the solver is done with the contacts
        // and joints this drops.
        const u32 slot = slotForId(id);
        if (slot != InvalidIndex)
            detachBody(slot, id);
        mBodyGenerations[id] = 0;
        if (kinetix::Find(mPendingRemovals.begin(), mPendingRemovals.end(), id) ==
            mPendingRemovals.end())
            mPendingRemovals.push_back(id);
        return;
    }
    removeBodyImmediate(id);
}

void PhysicsWorld::detachBody(u32 slot, u32 id)
{
    RigidBody* removedBody = mBodies[slot].body;
    if (!removedBody)
        return; // already detached earlier in this step

    if (removedBody->bodyType() == BodyType::Static)
        mStaticBroadphaseDirty = true;

    // Whatever it touched or was jointed to wakes up: a box asleep on top of
    // this one has no new contact to tell it that it now stands on air.
    for (const Contact& contact : mContacts)
    {
        if (contact.idA == id)
            wakeBody(contact.b);
        else if (contact.idB == id)
            wakeBody(contact.a);
    }
    // A pair of sleepers has no contact this step, only its cache entry.
    for (const auto& entry : mCache)
    {
        const u32 idA = static_cast<u32>(entry.key >> 32);
        const u32 idB = static_cast<u32>(entry.key & 0xFFFFFFFFull);
        if (!entry.value.touching || (idA != id && idB != id))
            continue;
        const u32 other = slotForId(idA == id ? idB : idA);
        if (other != InvalidIndex && mBodies[other].body)
            wakeBody(mBodies[other].body);
    }
    mContacts.erase(kinetix::RemoveIf(mContacts.begin(), mContacts.end(),
                                   [id](const Contact& contact)
                                   { return contact.idA == id || contact.idB == id; }),
                    mContacts.end());
    for (Joint* joint : mJoints)
    {
        if (!joint)
            continue;
        if (joint->bodyA() == removedBody)
            wakeBody(joint->bodyB());
        else if (joint->bodyB() == removedBody)
            wakeBody(joint->bodyA());
    }
    mJoints.erase(kinetix::RemoveIf(mJoints.begin(), mJoints.end(),
                                 [removedBody](Joint* joint)
                                 {
                                     return !joint || joint->bodyA() == removedBody ||
                                            joint->bodyB() == removedBody;
                                 }),
                  mJoints.end());

    removedBody->setSleepDeferred(false);
    const u32* mapped = mBodySlots.find(removedBody);
    if (mapped && *mapped == slot)
        mBodySlots.erase(removedBody);
    mBodies[slot].body = nullptr;
}

void PhysicsWorld::removeBodyImmediate(u32 id)
{
    const u32 slot = slotForId(id);
    if (slot == InvalidIndex)
        return;

    detachBody(slot, id);
    // Events still to be delivered may name this id (queued by the step whose
    // callback removed it, or the batch being dispatched), so it is not reused
    // until they are.
    if (mDispatchingEvents || !mEventQueue.empty())
        mRetiredIds.push_back(id);
    else
        mFreeIds.push_back(id);
    mBodyGenerations[id] = 0;
    mIdToSlot[id] = InvalidIndex;

    const u32 lastSlot = static_cast<u32>(mBodies.size() - 1);
    if (slot != lastSlot)
    {
        mBodies[slot] = mBodies[lastSlot];
        mBodyIds[slot] = mBodyIds[lastSlot];
        mIdToSlot[mBodyIds[slot]] = slot;
        u32* moved = mBodySlots.find(mBodies[slot].body);
        if (moved && *moved == lastSlot)
            *moved = slot;
    }
    mBodies.pop_back();
    mBodyIds.pop_back();

    // Every cached pair naming it goes too, or the next body with this id
    // inherits impulses of a body that no longer exists.
    ct::Vector<u64> staleKeys;
    for (const auto& entry : mCache)
    {
        const u32 a = static_cast<u32>(entry.key >> 32);
        const u32 b = static_cast<u32>(entry.key & 0xFFFFFFFFull);
        if (a == id || b == id)
            staleKeys.push_back(entry.key);
    }
    for (u64 key : staleKeys)
        mCache.erase(key);
    if (!mIgnoredPairs.empty())
    {
        staleKeys.clear();
        for (const auto& entry : mIgnoredPairs)
        {
            const u32 a = static_cast<u32>(entry.key >> 32);
            const u32 b = static_cast<u32>(entry.key & 0xFFFFFFFFull);
            if (a == id || b == id)
                staleKeys.push_back(entry.key);
        }
        for (u64 key : staleKeys)
            mIgnoredPairs.erase(key);
    }
}

void PhysicsWorld::clear()
{
    if (mStepping)
    {
        // As in removeBody(): the storage goes at the end of the step, bodies and
        // joints are let go now.
        for (BodyEntry& entry : mBodies)
            if (entry.body)
            {
                entry.body->setSleepDeferred(false);
                entry.body = nullptr;
            }
        for (PendingAdd& pending : mPendingAdds)
            if (pending.entry.body)
            {
                pending.entry.body->setSleepDeferred(false);
                pending.entry.body = nullptr;
            }
        for (u32& generation : mBodyGenerations)
            generation = 0;
        mContacts.clear();
        mJoints.clear();
        mBodySlots.clear();
        mClearPending = true;
        return;
    }
    clearImmediate();
}

void PhysicsWorld::clearImmediate()
{
    for (BodyEntry& entry : mBodies)
        if (entry.body)
            entry.body->setSleepDeferred(false);
    for (PendingAdd& pending : mPendingAdds)
        if (pending.entry.body)
            pending.entry.body->setSleepDeferred(false);
    mBodies.clear();
    mBodyIds.clear();
    mBodyGenerations.clear();
    mIdToSlot.clear();
    mBodySlots.clear();
    mFreeIds.clear();
    mRetiredIds.clear();
    mPendingAdds.clear();
    mPendingRemovals.clear();
    mCache.clear();
    mIgnoredPairs.clear();
    mContacts.clear();
    mJoints.clear();
    mStaticBroadphase.clear();
    mStaticBounds.clear();
    mStaticIds.clear();
    mStaticBroadphaseDirty = true;
    mEventQueue.clear();
    mPairs.clear();
    mAccumulator = 0.0f;
    mStepIndex = 0;
    mClearPending = false;
}

void PhysicsWorld::flushPendingMutations()
{
    if (mClearPending)
    {
        clearImmediate();
        return;
    }
    for (const PendingAdd& pending : mPendingAdds)
        insertBody(pending.id, pending.entry);
    mPendingAdds.clear();
    for (u32 id : mPendingRemovals)
        removeBodyImmediate(id);
    mPendingRemovals.clear();
}

BodyEntry* PhysicsWorld::body(u32 id)
{
    // A body removed from the step callback keeps its slot until the step ends
    // but is gone for anyone asking.
    if (id >= mBodyGenerations.size() || mBodyGenerations[id] == 0)
        return nullptr;
    const u32 slot = slotForId(id);
    return slot != InvalidIndex ? &mBodies[slot] : nullptr;
}

const BodyEntry* PhysicsWorld::body(u32 id) const
{
    if (id >= mBodyGenerations.size() || mBodyGenerations[id] == 0)
        return nullptr;
    const u32 slot = slotForId(id);
    return slot != InvalidIndex ? &mBodies[slot] : nullptr;
}

BodyHandle PhysicsWorld::bodyHandle(u32 id) const
{
    if (id >= mBodyGenerations.size() || mBodyGenerations[id] == 0)
        return {};
    return {id, mBodyGenerations[id]};
}

BodyEntry* PhysicsWorld::body(BodyHandle handle)
{
    if (!handle.valid() || handle.id >= mBodyGenerations.size() ||
        mBodyGenerations[handle.id] != handle.generation)
        return nullptr;
    return body(handle.id);
}

const BodyEntry* PhysicsWorld::body(BodyHandle handle) const
{
    if (!handle.valid() || handle.id >= mBodyGenerations.size() ||
        mBodyGenerations[handle.id] != handle.generation)
        return nullptr;
    return body(handle.id);
}

void PhysicsWorld::setGravity(const Math::vec3& gravity)
{
    mGravity = gravity;
}

void PhysicsWorld::setFixedStep(f32 seconds)
{
    if (seconds > 0.0f && isfinite(seconds))
        mFixedStep = seconds;
}

void PhysicsWorld::setEventCallback(EventCallback callback, void* userData)
{
    mEventCallback = callback;
    mEventUserData = userData;
}

void PhysicsWorld::setStepCallback(StepCallback callback, void* userData)
{
    mStepCallback = callback;
    mStepUserData = userData;
}

void PhysicsWorld::setContactPersistence(u32 steps)
{
    mContactPersistence = Math::max(steps, 1u);
}

void PhysicsWorld::setContactMargin(f32 margin)
{
    mContactMargin = Math::max(margin, 0.0f);
    mStaticBroadphaseDirty = true;
}

void PhysicsWorld::markStaticBroadphaseDirty()
{
    mStaticBroadphaseDirty = true;
}

void PhysicsWorld::rebuildStaticBroadphase() const
{
    mStaticBounds.clear();
    mStaticIds.clear();
    mStaticBounds.reserve(mBodies.size());
    mStaticIds.reserve(mBodies.size());
    for (u32 index = 0; index < mBodies.size(); ++index)
    {
        const BodyEntry& entry = mBodies[index];
        if (!entry.body || !entry.shape || !entry.enabled ||
            entry.body->bodyType() != BodyType::Static)
            continue;
        AABB bounds = entry.shape->bounds(entry.body->transform());
        bounds.min -= Math::vec3(mContactMargin);
        bounds.max += Math::vec3(mContactMargin);
        mStaticBounds.push_back(bounds);
        mStaticIds.push_back(mBodyIds[index]);
    }
    mStaticBroadphase.build(mStaticBounds.data(), static_cast<u32>(mStaticBounds.size()));
    mStaticBroadphaseDirty = false;
}

void PhysicsWorld::warmStartFromCache(u32 a, u32 b, ContactManifold& manifold)
{
    const CachedPair* found = mCache.find(pairKey(a, b));
    if (!found)
        return;
    const CachedPair& cached = *found;
    for (u32 i = 0; i < manifold.count; ++i)
    {
        ContactPoint& point = manifold.points[i];
        // Matched by position, not index: the narrowphase can emit the same patch
        // in a different order from step to step.
        f32 best = kMatchDistance;
        const CachedPoint* match = nullptr;
        for (u32 j = 0; j < cached.count; ++j)
        {
            const f32 distance = Math::length(point.position - cached.points[j].position);
            if (distance < best)
            {
                best = distance;
                match = &cached.points[j];
            }
        }
        if (!match)
            continue;
        point.normalImpulse = match->normalImpulse;
        point.tangentImpulse[0] = match->tangentImpulse[0];
        point.tangentImpulse[1] = match->tangentImpulse[1];
    }
}

void PhysicsWorld::storeInCache(u32 a, u32 b, const ContactManifold& manifold)
{
    CachedPair& cached = mCache[pairKey(a, b)];
    // The first manifold of this step starts the list; the pair's other
    // manifolds append to it (replacing would keep only the last triangle's impulses).
    if (cached.storedStep != mStepIndex)
    {
        cached.count = 0;
        cached.storedStep = mStepIndex;
    }
    for (u32 i = 0; i < manifold.count && cached.count < CachedPair::MaxPoints; ++i)
    {
        CachedPoint& point = cached.points[cached.count++];
        point.position = manifold.points[i].position;
        point.normalImpulse = manifold.points[i].normalImpulse;
        point.tangentImpulse[0] = manifold.points[i].tangentImpulse[0];
        point.tangentImpulse[1] = manifold.points[i].tangentImpulse[1];
    }
}

void PhysicsWorld::emitExits()
{
    mStaleKeys.clear();
    for (const auto& entry : mCache)
    {
        if (mStepIndex - entry.value.lastStep < mContactPersistence)
            continue;
        const u32 idA = static_cast<u32>(entry.key >> 32);
        const u32 idB = static_cast<u32>(entry.key & 0xFFFFFFFFull);
        const u32 slotA = slotForId(idA);
        const u32 slotB = slotForId(idB);
        if (!entry.value.touching)
        {
            mStaleKeys.push_back(entry.key);
            continue;
        }
        // Two bodies at rest make no pair; theirs stays until one of them moves.
        if (slotA != InvalidIndex && slotB != InvalidIndex && mBodies[slotA].body &&
            mBodies[slotB].body && resting(*mBodies[slotA].body) && resting(*mBodies[slotB].body))
            continue;
        mStaleKeys.push_back(entry.key);

        // Not touched this step, so the pair stopped touching. Reported once, then
        // dropped (which also stops the cache growing with every pair that ever met).
        if (entry.value.reported && mEventCallback)
        {
            ContactEventInfo info;
            info.bodyA = idA;
            info.bodyB = idB;
            info.event = ContactEvent::Exit;
            mEventQueue.push_back(info);
        }

        // Losing a contact wakes both sides, as gaining one does: a body asleep on
        // something moved out from under it (teleported, or a kinematic that
        // outran the margin) has no new contact and would sleep on in mid-air.
        if (slotA != InvalidIndex)
            wakeBody(mBodies[slotA].body);
        if (slotB != InvalidIndex)
            wakeBody(mBodies[slotB].body);
    }
    for (u64 key : mStaleKeys)
        mCache.erase(key);
}

u32 PhysicsWorld::islandRoot(u32 index)
{
    while (mIslandParent[index] != index)
    {
        // Path halving: a long stack does not pay for its depth on every query.
        mIslandParent[index] = mIslandParent[mIslandParent[index]];
        index = mIslandParent[index];
    }
    return index;
}

void PhysicsWorld::propagateSleep()
{
    const usize count = mBodies.size();
    mIslandParent.resize(count);
    mIslandAwake.assign(count, 0);
    for (usize i = 0; i < count; ++i)
        mIslandParent[i] = static_cast<u32>(i);

    // Static and kinematic bodies are left out: they never sleep or wake, and
    // joining them would put every body resting on the same floor into one island.
    for (const Contact& contact : mContacts)
    {
        if (!contact.a->isDynamic() || !contact.b->isDynamic())
            continue;
        const u32 slotA = slotForId(contact.idA);
        const u32 slotB = slotForId(contact.idB);
        if (slotA == InvalidIndex || slotB == InvalidIndex)
            continue;
        const u32 rootA = islandRoot(slotA);
        const u32 rootB = islandRoot(slotB);
        if (rootA != rootB)
            mIslandParent[rootB] = rootA;
    }

    // Joints link an island like contacts. Otherwise a limb held in the air by
    // its joint alone was an island of its own: it slept while the body still
    // moved, the joint fed it impulses that were never integrated, and on waking
    // it snapped to its anchor.
    for (const Joint* joint : mJoints)
    {
        if (!joint || !joint->enabled() || joint->singleBody())
            continue;
        RigidBody* a = joint->bodyA();
        RigidBody* b = joint->bodyB();
        if (!a || !b || !a->isDynamic() || !b->isDynamic())
            continue;
        const u32* slotA = mBodySlots.find(a);
        const u32* slotB = mBodySlots.find(b);
        if (!slotA || !slotB)
            continue;
        const u32 rootA = islandRoot(*slotA);
        const u32 rootB = islandRoot(*slotB);
        if (rootA != rootB)
            mIslandParent[rootB] = rootA;
    }

    // An island sleeps only when EVERY body in it is quiet, so a box does not
    // drop out of a stack that has not finished settling.
    for (usize i = 0; i < count; ++i)
    {
        const BodyEntry& entry = mBodies[i];
        if (!entry.body || !entry.enabled || !entry.body->isDynamic())
            continue;
        if (!entry.body->canSleep() || entry.body->motion() >= entry.body->sleepEpsilon())
            mIslandAwake[islandRoot(static_cast<u32>(i))] = 1;
    }

    // Kinematic bodies are in no island, but a moving one keeps awake every
    // island it touches or holds by a joint (a box asleep on a lift would hang
    // in place or be driven through).
    for (const Contact& contact : mContacts)
    {
        u32 carriedId = InvalidIndex;
        if (movingKinematic(contact.a) && contact.b->isDynamic())
            carriedId = contact.idB;
        else if (movingKinematic(contact.b) && contact.a->isDynamic())
            carriedId = contact.idA;
        const u32 slot = carriedId != InvalidIndex ? slotForId(carriedId) : InvalidIndex;
        if (slot != InvalidIndex)
            mIslandAwake[islandRoot(slot)] = 1;
    }
    for (const Joint* joint : mJoints)
    {
        if (!joint || !joint->enabled() || joint->singleBody())
            continue;
        const RigidBody* carried = nullptr;
        if (movingKinematic(joint->bodyA()) && joint->bodyB() && joint->bodyB()->isDynamic())
            carried = joint->bodyB();
        else if (movingKinematic(joint->bodyB()) && joint->bodyA() && joint->bodyA()->isDynamic())
            carried = joint->bodyA();
        const u32* slot = carried ? mBodySlots.find(carried) : nullptr;
        if (slot)
            mIslandAwake[islandRoot(*slot)] = 1;
    }

    for (usize i = 0; i < count; ++i)
    {
        BodyEntry& entry = mBodies[i];
        if (!entry.body || !entry.enabled || !entry.body->isDynamic())
            continue;
        const bool shouldBeAwake = mIslandAwake[islandRoot(static_cast<u32>(i))] != 0;
        if (shouldBeAwake != entry.body->awake())
            entry.body->setAwake(shouldBeAwake);
    }
}

void PhysicsWorld::step(f32 duration)
{
    if (duration <= 0.0f || !isfinite(duration))
        return;
    if (mStepping || mDispatchingEvents)
    {
        Log::error("PhysicsWorld: recursive step() is not supported");
        return;
    }
    mStepping = true;
    ++mStepIndex;

    // Gravity is set on every dynamic body rather than added as a force, so it
    // reaches them whatever their mass; a body's own acceleration is overwritten.
    for (BodyEntry& entry : mBodies)
        if (entry.body && entry.enabled && entry.body->isDynamic())
        {
            entry.body->setAcceleration(mGravity);
            entry.body->integrateForces(duration);
        }

    if (mStaticBroadphaseDirty)
        rebuildStaticBroadphase();
    mDynamicBroadphase.clear();
    mDynamicBroadphase.reserve(mBodies.size());
    mDynamicProxies.clear();
    mDynamicProxies.reserve(mBodies.size());
    mWorldBounds.resize(mBodies.size());
    mTravel.assign(mBodies.size(), 0.0f);
    for (u32 i = 0; i < mBodies.size(); ++i)
    {
        const BodyEntry& entry = mBodies[i];
        if (!entry.body || !entry.shape || !entry.enabled ||
            entry.body->bodyType() == BodyType::Static)
            continue;
        BroadphaseProxy proxy;
        proxy.id = mBodyIds[i];
        proxy.filter = entry.filter;
        proxy.movable = !resting(*entry.body);
        proxy.bounds = entry.shape->bounds(entry.body->transform());
        mWorldBounds[i] = proxy.bounds;
        if (proxy.movable)
        {
            const f32 radius = 0.5f * Math::length(proxy.bounds.max - proxy.bounds.min);
            const f32 travel = (Math::length(entry.body->velocity()) +
                                Math::length(entry.body->angularVelocity()) * radius) *
                               duration;
            if (isfinite(travel) && travel > mContactMargin)
                mTravel[i] = travel - mContactMargin;
        }
        // Grown by the contact margin, or the broadphase discards the pairs the
        // margin exists to keep (a resting body's AABB ends where the other starts).
        proxy.bounds.min -= Math::vec3(mContactMargin + mTravel[i]);
        proxy.bounds.max += Math::vec3(mContactMargin + mTravel[i]);
        mDynamicBroadphase.add(proxy);
        mDynamicProxies.push_back(proxy);
    }
    mPairs.clear();
    for (const BroadphaseProxy& dynamic : mDynamicProxies)
    {
        if (!dynamic.movable)
            continue;
        mStaticBroadphase.queryCandidates(dynamic.bounds, mStaticCandidates);
        for (u32 candidate : mStaticCandidates)
        {
            const AABB& staticBounds = mStaticBroadphase.itemBounds(candidate);
            const u32 staticId = mStaticIds[candidate];
            const u32 staticSlot = slotForId(staticId);
            if (staticSlot == InvalidIndex)
                continue;
            const BodyEntry& staticEntry = mBodies[staticSlot];
            if (!shouldCollide(dynamic.filter, staticEntry.filter) ||
                !Broadphase::overlaps(dynamic.bounds, staticBounds))
                continue;
            mPairs.push_back({Math::min(dynamic.id, staticId), Math::max(dynamic.id, staticId)});
        }
    }
    mDynamicBroadphase.findPairs(mDynamicPairs);
    for (const BroadphasePair& pair : mDynamicPairs)
        mPairs.push_back(pair);

    mContacts.clear();
    mContacts.reserve(mPairs.size());
    for (const BroadphasePair& pair : mPairs)
    {
        const u32 slotA = slotForId(pair.a);
        const u32 slotB = slotForId(pair.b);
        if (slotA == InvalidIndex || slotB == InvalidIndex)
            continue;
        if (!mIgnoredPairs.empty() && pairIgnored(pair.a, pair.b))
            continue;
        BodyEntry& a = mBodies[slotA];
        BodyEntry& b = mBodies[slotB];

        // Kinematic against kinematic or static: nothing the solver can move, and
        // without a listener nothing to report.
        if (!a.body->isDynamic() && !b.body->isDynamic() && !mEventCallback)
            continue;

        const f32 margin = mContactMargin + Math::max(mTravel[slotA], mTravel[slotB]);
        const u64 key = pairKey(pair.a, pair.b);
        const CachedPair* existing = mCache.find(key);
        const bool isNew = !(existing && existing->touching);
        Narrowphase::SeparatingAxis separation;
        if (existing && existing->axisKnown)
        {
            separation.axis = existing->axis;
            separation.known = true;
        }

        // A trimesh answers with one manifold per triangle touched, so the
        // single-manifold path cannot serve it; both go through the same vector.
        mManifolds.clear();
        const bool aIsMesh = a.shape->type() == ShapeType::Trimesh;
        const bool bIsMesh = b.shape->type() == ShapeType::Trimesh;
        if (aIsMesh && bIsMesh)
            continue; // two static meshes never move
        if (aIsMesh || bIsMesh)
        {
            const BodyEntry& convex = aIsMesh ? b : a;
            const BodyEntry& mesh = aIsMesh ? a : b;
            const u32 convexSlot = aIsMesh ? slotB : slotA;
            const AABB* convexBounds = convex.body->bodyType() != BodyType::Static
                                           ? &mWorldBounds[convexSlot]
                                           : nullptr;
            if (!Narrowphase::convexTrimesh(*convex.shape, convex.body->transform(),
                                            static_cast<const TrimeshShape&>(*mesh.shape),
                                            mesh.body->transform(), mManifolds, margin,
                                            convexBounds))
                continue;
            // convexTrimesh() reports convex-to-mesh; with the mesh as body A the
            // normal must still run A to B.
            if (aIsMesh)
                for (ContactManifold& flipped : mManifolds)
                {
                    flipped.normal = -flipped.normal;
                    flipped.buildTangents();
                }
        }
        else
        {
            ContactManifold manifold;
            if (!Narrowphase::collide(*a.shape, a.body->transform(), *b.shape, b.body->transform(),
                                      manifold, margin, &separation))
            {
                if (separation.known)
                {
                    CachedPair& apart = mCache[key];
                    apart.axis = separation.axis;
                    apart.axisKnown = true;
                    apart.lastStep = mStepIndex;
                }
                continue;
            }
            mManifolds.push_back(manifold);
        }

        ContactManifold& manifold = mManifolds[0];
        for (ContactManifold& current : mManifolds)
        {
            warmStartFromCache(pair.a, pair.b, current);

            Contact contact;
            contact.a = a.body;
            contact.b = b.body;
            contact.idA = pair.a;
            contact.idB = pair.b;
            contact.manifold = current;
            // Geometric mean for friction, the larger for restitution.
            contact.friction = sqrtf(a.friction * b.friction);
            contact.restitution = Math::max(a.restitution, b.restitution);
            mContacts.push_back(contact);
        }

        if (isNew)
        {
            // A new contact is the one moment a sleeping body has to wake; the
            // solver does not, since a resting stack has contacts every step.
            a.body->setAwake(true);
            b.body->setAwake(true);
        }
        if (mEventCallback)
        {
            ContactEventInfo info;
            info.bodyA = pair.a;
            info.bodyB = pair.b;
            info.event = isNew ? ContactEvent::Enter : ContactEvent::Stay;
            info.normal = manifold.normal;
            info.point = manifold.points[0].position;
            info.penetration = manifold.points[0].penetration;
            mEventQueue.push_back(info);
        }
        CachedPair& cached = mCache[key];
        cached.lastStep = mStepIndex;
        cached.reported = true;
        cached.touching = true;
        cached.axisKnown = false;
    }

    mSolver.solve(mContacts.data(), static_cast<u32>(mContacts.size()), mJoints.data(),
                  static_cast<u32>(mJoints.size()), duration);

    // Stored after solving, so the next step carries the settled impulse.
    for (const Contact& contact : mContacts)
        storeInCache(contact.idA, contact.idB, contact.manifold);

    emitExits();

    for (BodyEntry& entry : mBodies)
        if (entry.body && entry.enabled)
            entry.body->integrateVelocity(duration);

    // Position correction after the positions moved, where this step's overlap
    // is measurable (contact points ride on the bodies).
    mSolver.solvePositions(mContacts.data(), static_cast<u32>(mContacts.size()), mJoints.data(),
                           static_cast<u32>(mJoints.size()));

    // Actions (a vehicle, anything reaching into bodies each step) run once
    // positions are final and before sleep state is decided.
    if (mStepCallback)
        mStepCallback(duration, mStepUserData);

    // After integrating: the motion average this reads is what integrate() just
    // updated. Bodies in this world have their own sleep deferred.
    propagateSleep();
    mStepping = false;
    flushPendingMutations();
    dispatchEvents();
}

void PhysicsWorld::dispatchEvents()
{
    if (!mEventCallback || mEventQueue.empty())
    {
        mEventQueue.clear();
        for (u32 id : mRetiredIds)
            mFreeIds.push_back(id);
        mRetiredIds.clear();
        return;
    }

    // Callbacks run outside the simulation lock. The world is non-owning, so
    // removeBody() must return only after the solver has released every pointer
    // to a body the owner may destroy immediately.
    mDispatching.swap(mEventQueue);
    mDispatchingEvents = true;
    for (const ContactEventInfo& info : mDispatching)
        if (mEventCallback)
            mEventCallback(info, mEventUserData);
    mDispatchingEvents = false;
    mDispatching.clear();
    for (u32 id : mRetiredIds)
        mFreeIds.push_back(id);
    mRetiredIds.clear();
}

void PhysicsWorld::update(f32 deltaTime)
{
    if (deltaTime <= 0.0f || !isfinite(deltaTime))
        return;
    mAccumulator += deltaTime;
    const f32 budget = mFixedStep * static_cast<f32>(mMaxStepsPerUpdate);
    if (mAccumulator > budget)
        mAccumulator = budget;
    while (mAccumulator >= mFixedStep)
    {
        step(mFixedStep);
        mAccumulator -= mFixedStep;
    }
}

void PhysicsWorld::debugDraw() const
{
    for (const BodyEntry& entry : mBodies)
    {
        if (!entry.body || !entry.shape || !entry.enabled)
            continue;
        // Hue says which shape it is, brightness what the simulation is doing
        // with it (a body that never dims never settled).
        Color color = Color(60, 220, 60, 255);
        if (entry.body->bodyType() == BodyType::Kinematic)
            color = Color(230, 200, 60, 255);
        else if (entry.body->isDynamic())
        {
            switch (entry.shape->type())
            {
            case ShapeType::Sphere:  color = Color(80, 220, 200, 255); break;
            case ShapeType::Box:     color = Color(110, 220, 90, 255); break;
            case ShapeType::Capsule: color = Color(220, 110, 220, 255); break;
            case ShapeType::ConvexHull: color = Color(230, 150, 60, 255); break;
            case ShapeType::Cylinder: color = Color(120, 170, 240, 255); break;
            default:                 color = Color::White; break;
            }
            // Asleep is a third of the brightness.
            if (!entry.body->awake())
                color = Color(static_cast<u8>(color.r() / 3), static_cast<u8>(color.g() / 3),
                              static_cast<u8>(color.b() / 3), 255);
        }
        entry.shape->debugDraw(entry.body->transform(), color);
    }

    for (const Contact& contact : mContacts)
        for (u32 i = 0; i < contact.manifold.count; ++i)
        {
            const ContactPoint& cp = contact.manifold.points[i];
            // Red where the shapes overlap, yellow for a speculative point that
            // is still separated; the cross keeps a point visible when it
            // carries no impulse, the normal is scaled by the impulse.
            const Color color = cp.penetration > 0.0f ? Color(255, 40, 40, 255) : Color(255, 220, 40, 255);
            DebugDraw().cross(cp.position, 0.12f, color);
            const f32 scale = 0.15f + cp.normalImpulse * 0.02f;
            DebugDraw().line(cp.position, cp.position + contact.manifold.normal * scale, color);
        }
}

template <typename Test>
void PhysicsWorld::testCandidatesThenMovers(const Test& test) const
{
    for (u32 candidate : mQueryCandidates)
    {
        const u32 slot = slotForId(mStaticIds[candidate]);
        if (slot != InvalidIndex)
            test(slot);
    }
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
        if (mBodies[slot].body && mBodies[slot].body->bodyType() != BodyType::Static)
            test(slot);
}

bool PhysicsWorld::raycast(const Ray& ray, f32 maxDistance, const QueryFilter& filter,
                           WorldRayHit& hit) const
{
    bool found = false;
    f32 nearest = maxDistance;
    auto test = [&](u32 slot)
    {
        const BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.body || !entry.shape || !entry.enabled)
            return;
        if (!filter.accepts(id, entry.filter))
            return;

        ShapeRayHit shapeHit;
        if (!Narrowphase::raycast(*entry.shape, entry.body->transform(), ray, nearest, shapeHit))
            return;

        nearest = shapeHit.distance;
        hit.body = id;
        hit.point = shapeHit.point;
        hit.normal = shapeHit.normal;
        hit.distance = shapeHit.distance;
        hit.triangle = shapeHit.triangle;
        found = true;
    };

    if (mStaticBroadphaseDirty)
        rebuildStaticBroadphase();
    mStaticBroadphase.queryCandidates(ray, maxDistance, mQueryCandidates);
    testCandidatesThenMovers(test);
    return found;
}

void PhysicsWorld::overlapSphere(const Math::vec3& centre, f32 radius, const QueryFilter& filter,
                                 ct::Vector<u32>& out) const
{
    out.clear();
    auto test = [&](u32 slot)
    {
        const BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.body || !entry.shape || !entry.enabled)
            return;
        if (!filter.accepts(id, entry.filter))
            return;
        if (Narrowphase::overlapSphere(*entry.shape, entry.body->transform(), centre, radius))
            out.push_back(id);
    };

    if (mStaticBroadphaseDirty)
        rebuildStaticBroadphase();
    Sphere sphere;
    sphere.center = centre;
    sphere.radius = radius;
    mStaticBroadphase.queryCandidates(sphere, mQueryCandidates);
    testCandidatesThenMovers(test);
}

void PhysicsWorld::queryAABB(const AABB& bounds, const QueryFilter& filter,
                             ct::Vector<u32>& out) const
{
    out.clear();
    auto test = [&](u32 slot)
    {
        const BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.body || !entry.shape || !entry.enabled || !filter.accepts(id, entry.filter))
            return;
        if (Broadphase::overlaps(bounds, entry.shape->bounds(entry.body->transform())))
            out.push_back(id);
    };

    if (mStaticBroadphaseDirty)
        rebuildStaticBroadphase();
    mStaticBroadphase.queryCandidates(bounds, mQueryCandidates);
    testCandidatesThenMovers(test);
}

u32 PhysicsWorld::applyRadialImpulse(const Math::vec3& centre, f32 radius, f32 strength,
                                     const QueryFilter& filter)
{
    if (!finiteVector(centre) || !(radius > 0.0f) || !isfinite(radius) ||
        !isfinite(strength) || strength == 0.0f)
        return 0;

    u32 affected = 0;
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.enabled || !entry.body || !entry.body->isDynamic() ||
            !filter.accepts(id, entry.filter))
            continue;

        Math::vec3 direction;
        const f32 weight = radialWeight(centre, entry.body->position(), radius, &direction);
        if (weight <= 0.0f)
            continue;
        entry.body->applyLinearImpulse(direction * (strength * weight));
        ++affected;
    }
    return affected;
}

u32 PhysicsWorld::addRadialForce(const Math::vec3& centre, f32 radius, f32 strength,
                                 const QueryFilter& filter)
{
    if (!finiteVector(centre) || !(radius > 0.0f) || !isfinite(radius) ||
        !isfinite(strength) || strength == 0.0f)
        return 0;

    u32 affected = 0;
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.enabled || !entry.body || !entry.body->isDynamic() ||
            !filter.accepts(id, entry.filter))
            continue;

        Math::vec3 direction;
        const f32 weight = radialWeight(centre, entry.body->position(), radius, &direction);
        if (weight <= 0.0f)
            continue;
        entry.body->addForce(direction * (strength * weight));
        ++affected;
    }
    return affected;
}

u32 PhysicsWorld::addDirectionalForce(const Math::vec3& centre, f32 radius, const Math::vec3& force,
                                      const QueryFilter& filter)
{
    if (!finiteVector(centre) || !finiteVector(force) || !(radius > 0.0f) ||
        !isfinite(radius) || Math::dot(force, force) == 0.0f)
        return 0;

    u32 affected = 0;
    for (u32 slot = 0; slot < mBodies.size(); ++slot)
    {
        BodyEntry& entry = mBodies[slot];
        const u32 id = mBodyIds[slot];
        if (!entry.enabled || !entry.body || !entry.body->isDynamic() ||
            !filter.accepts(id, entry.filter))
            continue;

        const f32 weight = radialWeight(centre, entry.body->position(), radius);
        if (weight <= 0.0f)
            continue;
        entry.body->addForce(force * weight);
        ++affected;
    }
    return affected;
}

} // namespace kinetix::Physics
