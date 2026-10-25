#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/Hook.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "dynamics/FixedJoint.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RigidBody.h"

#include <ct/vector.hpp>

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr u32 kNone = 0xFFFFFFFFu;
}

Hook::Hook(PhysicsWorld& world, u32 hookBodyId, const Math::vec3& worldHookPoint)
    : mWorld(&world), mHook(world.bodyHandle(hookBodyId))
{
    const BodyEntry* entry = world.body(mHook);
    mLocalPoint = entry && entry->body ? entry->body->pointToLocal(worldHookPoint) : worldHookPoint;
}

Hook::~Hook()
{
    detach();
}

RigidBody* Hook::hookBody() const
{
    const BodyEntry* entry = mWorld->body(mHook);
    return entry ? entry->body : nullptr;
}

Math::vec3 Hook::pointWorld() const
{
    const RigidBody* body = hookBody();
    return body ? body->pointToWorld(mLocalPoint) : mLocalPoint;
}

bool Hook::attached() const
{
    // The world drops a joint whose body it lets go of.
    return mJoint && mWorld->body(mLoad) && mWorld->body(mHook);
}

u32 Hook::loadId() const
{
    return attached() ? mLoad.id : kNone;
}

RigidBody* Hook::load() const
{
    if (!attached())
        return nullptr;
    const BodyEntry* entry = mWorld->body(mLoad);
    return entry ? entry->body : nullptr;
}

Math::vec3 Hook::loadForce() const
{
    // The joint reports the force on the load (its body B); the hook gets the opposite.
    return attached() ? -mJoint->reactionForce() : Math::vec3(0.0f);
}

bool Hook::attach(u32 loadBodyId, HookMode mode, bool ignoreCollision)
{
    if (attached())
        return false;
    detach();

    BodyEntry* hookEntry = mWorld->body(mHook);
    const BodyHandle loadHandle = mWorld->bodyHandle(loadBodyId);
    BodyEntry* loadEntry = mWorld->body(loadHandle);
    if (!hookEntry || !hookEntry->body || !loadEntry || !loadEntry->body ||
        loadEntry->body == hookEntry->body || !loadEntry->body->isDynamic())
        return false;
    RigidBody& hook = *hookEntry->body;
    RigidBody& load = *loadEntry->body;

    // The joints are born in the bodies' current pose, with no error to correct.
    const Math::vec3 point = hook.pointToWorld(mLocalPoint);
    if (mode == HookMode::Fixed)
    {
        mFixedJoint = new FixedJoint(hook, load, point);
        mJoint = mFixedJoint;
        mJoint->setArticulated(mArticulated);
    }
    else
    {
        mPointJoint = new PointJoint(hook, load, point);
        mJoint = mPointJoint;
        mJoint->setArticulated(mArticulated);
    }
    mLoad = loadHandle;
    mMode = mode;
    mWorld->addJoint(mJoint);
    mIgnoringCollision = false;
    if (ignoreCollision && !mWorld->pairIgnored(mHook.id, loadHandle.id))
    {
        mWorld->setPairIgnored(mHook.id, loadHandle.id, true);
        mIgnoringCollision = true;
    }
    return true;
}

u32 Hook::findNearest(f32 captureRadius, const QueryFilter& filter) const
{
    const RigidBody* hook = hookBody();
    if (!hook || !(captureRadius > 0.0f) || !isfinite(captureRadius))
        return kNone;
    const Math::vec3 point = hook->pointToWorld(mLocalPoint);
    QueryFilter query = filter;
    query.ignoredBody = mHook.id;
    ct::Vector<u32> candidates;
    mWorld->overlapSphere(point, captureRadius, query, candidates);

    // Ranked by the surface distance: the deepest contact of a capture-radius
    // sphere at the hook point, so a long body with a far centre can still win.
    const SphereShape probe(captureRadius);
    const Math::mat4 probeTransform = Math::translate(Math::mat4(1.0f), point);
    u32 best = kNone;
    f32 bestSurface = M_INFINITY;
    f32 bestCentre = M_INFINITY;
    for (u32 id : candidates)
    {
        const BodyEntry* entry = mWorld->body(id);
        if (!entry || !entry->body || !entry->shape || !entry->body->isDynamic() ||
            entry->body == hook)
            continue;
        f32 surface = Math::length(entry->body->position() - point);
        ContactManifold manifold;
        if (Narrowphase::collide(probe, probeTransform, *entry->shape, entry->body->transform(),
                                 manifold, 0.0f) &&
            manifold.count > 0)
        {
            f32 deepest = manifold.points[0].penetration;
            for (u32 i = 1; i < manifold.count; ++i)
                deepest = Math::max(deepest, manifold.points[i].penetration);
            surface = Math::max(captureRadius - deepest, 0.0f);
        }
        const f32 centre = Math::length(entry->body->position() - point);
        if (surface < bestSurface - 1.0e-6f ||
            (fabsf(surface - bestSurface) <= 1.0e-6f && centre < bestCentre))
        {
            best = id;
            bestSurface = surface;
            bestCentre = centre;
        }
    }
    return best;
}

u32 Hook::attachNearest(f32 captureRadius, HookMode mode, const QueryFilter& filter,
                        bool ignoreCollision)
{
    if (attached())
        return kNone;
    const u32 nearest = findNearest(captureRadius, filter);
    if (nearest == kNone || !attach(nearest, mode, ignoreCollision))
        return kNone;
    return nearest;
}

void Hook::detach()
{
    if (!mJoint)
        return;
    // No-op when the world already dropped the joint with its body; the world
    // wakes both bodies.
    mWorld->removeJoint(mJoint);
    if (mIgnoringCollision && mWorld->body(mLoad) && mWorld->body(mHook))
        mWorld->setPairIgnored(mHook.id, mLoad.id, false);
    delete mJoint;
    mJoint = nullptr;
    mPointJoint = nullptr;
    mFixedJoint = nullptr;
    mLoad = BodyHandle();
    mIgnoringCollision = false;
}

} // namespace kinetix::Physics
