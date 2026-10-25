#ifndef KINETIX_PHYSICS_DYNAMICS_PHYSICSWORLD_H
#define KINETIX_PHYSICS_DYNAMICS_PHYSICSWORLD_H

#include <kinetix/core/Containers.h>
#include <kinetix/core/BoundsTree.h>
#include "collision/Broadphase.h"
#include "collision/CollisionFilter.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/RigidBody.h"

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class CollisionShape;
class Joint;

struct BodyHandle
{
    u32 id = 0xFFFFFFFFu;
    u32 generation = 0;

    bool valid() const
    {
        return id != 0xFFFFFFFFu && generation != 0;
    }
};

// What happened to a pair this step, derived from the warm-starting contact cache.
enum class ContactEvent : u8
{
    Enter,
    Stay,
    Exit
};

struct ContactEventInfo
{
    u32 bodyA = 0;
    u32 bodyB = 0;
    ContactEvent event = ContactEvent::Enter;
    // Empty for Exit.
    Math::vec3 normal{0.0f};
    Math::vec3 point{0.0f};
    f32 penetration = 0.0f;
};

struct WorldRayHit
{
    u32 body = 0xFFFFFFFFu;
    Math::vec3 point{0.0f};
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    f32 distance = 0.0f;
    // Index of the triangle hit when the body's shape is a trimesh, -1 otherwise.
    s32 triangle = -1;
};

// A body and the shape it collides with. The world owns neither.
struct BodyEntry
{
    RigidBody* body = nullptr;
    CollisionShape* shape = nullptr;
    CollisionFilter filter;
    f32 friction = 0.5f;
    f32 restitution = 0.0f;
    bool enabled = true;
};

// Drives one simulation step end to end: broadphase, narrowphase, contact
// cache, solver, integrate. Free of scene knowledge.
class PhysicsWorld
{
public:
    using EventCallback = void (*)(const ContactEventInfo& info, void* userData);
    // Called once per fixed step, after velocities are integrated into position.
    using StepCallback = void (*)(f32 step, void* userData);

    PhysicsWorld();

    // Returns the stable id used by pairs and events; storage is dense and
    // removal swap-pops an internal slot, the id keeps naming the same body.
    u32 addBody(const BodyEntry& entry);
    // The world never touches the body again once this returns, so the caller
    // may destroy it at once, also from the step callback (the slot is released
    // at the end of the step). Bodies it touched or was jointed to are woken.
    void removeBody(u32 id);
    // Two bodies that never collide whatever their filters say (e.g. jointed
    // bodies whose shapes overlap at the pivot). Forgotten when either is removed.
    void setPairIgnored(u32 idA, u32 idB, bool ignored);
    bool pairIgnored(u32 idA, u32 idB) const;
    void clear();
    // Call after changing an added static body's transform, shape, filter,
    // enabled state or type. Dynamic and kinematic bodies rebuild every step.
    void markStaticBroadphaseDirty();

    void addJoint(Joint* joint);
    // Wakes both bodies, so what the joint held up can fall.
    void removeJoint(Joint* joint);
    usize jointCount() const
    {
        return mJoints.size();
    }

    BodyEntry* body(u32 id);
    const BodyEntry* body(u32 id) const;
    BodyHandle bodyHandle(u32 id) const;
    BodyEntry* body(BodyHandle handle);
    const BodyEntry* body(BodyHandle handle) const;
    usize bodyCount() const
    {
        return mBodies.size();
    }

    void setGravity(const Math::vec3& gravity);
    const Math::vec3& gravity() const
    {
        return mGravity;
    }

    // Seconds per simulation step. Physics runs at this rate whatever the frame
    // rate: a solver tuned at one step size behaves differently at another.
    void setFixedStep(f32 seconds);
    f32 fixedStep() const
    {
        return mFixedStep;
    }

    void setSolverSettings(const ContactSolverSettings& settings)
    {
        mSolver.setSettings(settings);
    }
    const ContactSolverSettings& solverSettings() const
    {
        return mSolver.settings();
    }
    // The articulated joints' direct solver (Joint::setArticulated), for diagnostics.
    const ArticulationSolver& articulation() const
    {
        return mSolver.articulation();
    }

    void setEventCallback(EventCallback callback, void* userData);
    void setStepCallback(StepCallback callback, void* userData);

    // Steps a pair may go without a manifold before it counts as separated. One
    // is not enough: on landing the position pass lifts the body clear for a
    // single step, which would report Exit/Enter and lose the accumulated impulses.
    void setContactPersistence(u32 steps);
    u32 contactPersistence() const
    {
        return mContactPersistence;
    }

    // Distance within which two shapes still count as in contact (negative
    // penetration), so a pair does not die whenever the position pass lifts a
    // body clear. See Narrowphase::collide.
    void setContactMargin(f32 margin);
    f32 contactMargin() const
    {
        return mContactMargin;
    }

    // Consumes `deltaTime` in fixed steps, keeping the remainder. Backlog beyond
    // `maxStepsPerUpdate` steps is dropped, otherwise a stall is paid off with a
    // burst of steps that stalls again.
    void update(f32 deltaTime);
    void step(f32 duration);

    void debugDraw() const;

    bool raycast(const Ray& ray, f32 maxDistance, const QueryFilter& filter,
                 WorldRayHit& hit) const;
    void overlapSphere(const Math::vec3& centre, f32 radius, const QueryFilter& filter,
                       ct::Vector<u32>& out) const;
    void queryAABB(const AABB& bounds, const QueryFilter& filter, ct::Vector<u32>& out) const;

    // Radial strength is signed: positive pushes away, negative pulls inward;
    // effects decay linearly to zero at radius and affect only dynamic bodies
    // accepted by the filter. An impulse is instantaneous; forces must be added
    // before step() and are integrated during it.
    u32 applyRadialImpulse(const Math::vec3& centre, f32 radius, f32 strength,
                           const QueryFilter& filter = QueryFilter());
    u32 addRadialForce(const Math::vec3& centre, f32 radius, f32 strength,
                       const QueryFilter& filter = QueryFilter());
    u32 addDirectionalForce(const Math::vec3& centre, f32 radius, const Math::vec3& force,
                            const QueryFilter& filter = QueryFilter());

    usize contactCount() const
    {
        return mContacts.size();
    }
    const ct::Vector<Contact>& contacts() const
    {
        return mContacts;
    }

private:
    static constexpr u32 InvalidIndex = 0xFFFFFFFFu;

    struct PendingAdd
    {
        u32 id = InvalidIndex;
        BodyEntry entry;
    };

    struct CachedPoint
    {
        Math::vec3 position{0.0f};
        f32 normalImpulse = 0.0f;
        f32 tangentImpulse[2] = {0.0f, 0.0f};
    };
    struct CachedPair
    {
        // Several manifolds per pair (a convex against a trimesh makes one per
        // triangle). warmStartFromCache() matches points by position, so the
        // shared pool needs no per-manifold bookkeeping.
        static constexpr u32 MaxPoints = ContactManifold::MaxPoints * 4;
        CachedPoint points[MaxPoints];
        u32 count = 0;
        // lastStep: last step the pair had a manifold (decides Exit and pruning),
        // written during contact generation. storedStep: last step the solved
        // impulses were written into points[], after the solve. They must stay
        // separate, or the list is never restarted and warm starting replays old
        // impulses.
        u32 lastStep = 0;
        u32 storedStep = 0;
        bool reported = false;
        // Whether the pair touched; otherwise the entry only remembers the axis
        // the shapes were last found apart along, tested first the next step.
        bool touching = false;
        bool axisKnown = false;
        Math::vec3 axis{0.0f};
    };

    static u64 pairKey(u32 a, u32 b);
    // Bodies joined by contacts must sleep as one, otherwise the top of a stack
    // sleeps first and hangs in the air over bodies still settling.
    void propagateSleep();
    u32 islandRoot(u32 index);
    void warmStartFromCache(u32 a, u32 b, ContactManifold& manifold);
    void storeInCache(u32 a, u32 b, const ContactManifold& manifold);
    void emitExits();
    u32 allocateId();
    u32 slotForId(u32 id) const;
    void insertBody(u32 id, const BodyEntry& entry);
    void removeBodyImmediate(u32 id);
    void detachBody(u32 slot, u32 id);
    void wakeBody(RigidBody* body);
    void clearImmediate();
    void flushPendingMutations();
    void dispatchEvents();
    void rebuildStaticBroadphase() const;
    template <typename Test>
    void testCandidatesThenMovers(const Test& test) const;

    // Dense parallel arrays; public ids never move, mIdToSlot follows the swap-pop.
    ct::Vector<BodyEntry> mBodies;
    ct::Vector<u32> mBodyIds;
    ct::Vector<u32> mBodyGenerations;
    ct::Vector<u32> mIdToSlot;
    ct::Vector<u32> mFreeIds;
    // Ids removed by a collision callback are not reused until the event batch
    // ends; later events may still mention them.
    ct::Vector<u32> mRetiredIds;
    u32 mNextBodyGeneration = 1;
    // Body pointer to slot, for the joints (they name bodies by pointer).
    HashMap<const RigidBody*, u32> mBodySlots;

    // Mutations requested while step() holds temporary references are queued
    // here. Collision events are dispatched after the simulation unlocks, so
    // their callbacks mutate immediately.
    ct::Vector<PendingAdd> mPendingAdds;
    ct::Vector<u32> mPendingRemovals;
    bool mStepping = false;
    bool mClearPending = false;
    bool mDispatchingEvents = false;

    Broadphase mDynamicBroadphase;
    mutable BoundsTree mStaticBroadphase;
    mutable ct::Vector<AABB> mStaticBounds;
    mutable ct::Vector<u32> mStaticIds;
    mutable bool mStaticBroadphaseDirty = true;
    mutable ct::Vector<u32> mQueryCandidates;
    ct::Vector<BroadphaseProxy> mDynamicProxies;
    ct::Vector<AABB> mWorldBounds;
    // How far each body can move this step; the contact margin grows by it so a
    // fast body meets what it would otherwise pass through.
    ct::Vector<f32> mTravel;
    ct::Vector<BroadphasePair> mDynamicPairs;
    ct::Vector<u32> mStaticCandidates;
    ct::Vector<BroadphasePair> mPairs;
    // Scratch for one pair's manifolds (one per triangle for a trimesh).
    ct::Vector<ContactManifold> mManifolds;
    ct::Vector<Contact> mContacts;
    ct::Vector<Joint*> mJoints;
    ct::Vector<ContactEventInfo> mEventQueue;
    ct::Vector<ContactEventInfo> mDispatching;
    ct::Vector<u64> mStaleKeys;
    HashMap<u64, CachedPair> mCache;
    HashMap<u64, u8> mIgnoredPairs;
    ct::Vector<u32> mIslandParent;
    ct::Vector<u8> mIslandAwake;

    ContactSolver mSolver;
    Math::vec3 mGravity{0.0f, -9.81f, 0.0f};
    f32 mFixedStep = 1.0f / 120.0f;
    f32 mAccumulator = 0.0f;
    u32 mMaxStepsPerUpdate = 8;
    u32 mStepIndex = 0;
    u32 mContactPersistence = 2;
    f32 mContactMargin = 0.04f;

    EventCallback mEventCallback = nullptr;
    void* mEventUserData = nullptr;
    StepCallback mStepCallback = nullptr;
    void* mStepUserData = nullptr;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_PHYSICSWORLD_H
