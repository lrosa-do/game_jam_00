#ifndef KINETIX_PHYSICS_ACTUATORS_HOOK_H
#define KINETIX_PHYSICS_ACTUATORS_HOOK_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "collision/CollisionFilter.h"
#include "dynamics/PhysicsWorld.h"

namespace kinetix::Physics
{

class FixedJoint;
class Joint;
class PointJoint;
class RigidBody;

enum class HookMode : u8
{
    // The load swings freely about the hook point.
    Point,
    // Locked to the hook body in the pose it was caught in.
    Fixed
};

// A joint between a hook body and a load body, made in their relative pose
// at attach time so it starts with no error.
//
// attach() and detach() may be called at any time outside the solver -
// between steps, from the world's step callback, from a contact event - and
// take effect from the next step: the world adds and removes joints
// immediately, and none of those places is inside a pass over them. Both
// wake the bodies. A load body removed from the world while hooked takes the
// joint out of the world with it; the hook notices (attached() turns false)
// and the next detach() or attach() frees what is left.
class Hook
{
public:
    Hook(PhysicsWorld& world, u32 hookBodyId, const Math::vec3& worldHookPoint);
    ~Hook();
    Hook(const Hook&) = delete;
    Hook& operator=(const Hook&) = delete;

    // Hooks the load with this world id. False if the hook is already
    // holding something, the load is not a body of this world, is the hook
    // itself, or is not dynamic. With `ignoreCollision`, the two bodies stop
    // colliding with each other while attached.
    bool attach(u32 loadBodyId, HookMode mode = HookMode::Point, bool ignoreCollision = true);
    // Hooks the nearest dynamic body (by surface distance, then by centre
    // distance) within `captureRadius` of the hook point that `filter`
    // accepts, through the world's sphere query. Returns its id, or
    // 0xFFFFFFFF if there was none (or the hook was already holding one).
    u32 attachNearest(f32 captureRadius, HookMode mode = HookMode::Point,
                      const QueryFilter& filter = QueryFilter(), bool ignoreCollision = true);
    u32 findNearest(f32 captureRadius, const QueryFilter& filter = QueryFilter()) const;
    void detach();
    // Joints made from now on are articulated (Joint::setArticulated).
    void setArticulated(bool articulated)
    {
        mArticulated = articulated;
    }

    bool attached() const;
    u32 loadId() const;
    RigidBody* load() const;
    HookMode mode() const
    {
        return mMode;
    }
    Joint* joint() const
    {
        return mJoint;
    }
    RigidBody* hookBody() const;
    Math::vec3 pointWorld() const;
    // The force the load pulls the hook with, world, N (zero when free).
    Math::vec3 loadForce() const;

private:
    PhysicsWorld* mWorld;
    BodyHandle mHook;
    Math::vec3 mLocalPoint;

    Joint* mJoint = nullptr;
    PointJoint* mPointJoint = nullptr;
    FixedJoint* mFixedJoint = nullptr;
    BodyHandle mLoad;
    HookMode mMode = HookMode::Point;
    bool mIgnoringCollision = false;
    bool mArticulated = false;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ACTUATORS_HOOK_H
