#ifndef KINETIX_PHYSICS_DYNAMICS_RIGIDBODY_H
#define KINETIX_PHYSICS_DYNAMICS_RIGIDBODY_H

#include "geometry/ConvexHullComputer.h"
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

enum class BodyType : u8
{
    // Never moves, never responds to a force; infinite mass and inertia, so a
    // contact against it pushes only the other side.
    Static,

    Dynamic,

    // Moved by its owner (animation, lift, platform), unmoved by contacts. It
    // carries a velocity, so a dynamic body resting on it is carried along.
    Kinematic
};

// Moments of inertia about the centre of mass for uniform density (standard
// closed forms), kept here because a body needs one to rotate whether or not
// it collides.
struct Inertia
{
    // `halfExtents` are half the box's size on each axis.
    static Math::mat3 box(f32 mass, const Math::vec3& halfExtents);
    static Math::mat3 solidSphere(f32 mass, f32 radius);
    static Math::mat3 hollowSphere(f32 mass, f32 radius);
    // Along +Y.
    static Math::mat3 cylinderY(f32 mass, f32 radius, f32 height);
    static Math::mat3 capsuleY(f32 mass, f32 radius, f32 cylinderHeight);
    // Any convex polyhedron from its half-edge mesh; measures the tensor off the
    // actual geometry. `faces` holds one edge index per face, walked with
    // getNextEdgeOfFace() as ConvexHullComputer expects.
    static Math::mat3 convexHull(f32 mass, const Math::vec3* vertices, u32 vertexCount,
                                const kinetix::Geometry::ConvexHullComputer::Edge* edges,
                                u32 edgeCount, const int* faces, u32 faceCount);
};

// One rigid body: pose, velocities and the force and torque accumulators. It
// knows nothing about collision or the scene; contacts and constraints reach it
// through the impulse and force calls. The inertia tensor is kept in body space
// and rotated into world space once per step.
class RigidBody
{
public:
    RigidBody();

    void setBodyType(BodyType type);
    BodyType bodyType() const
    {
        return mBodyType;
    }
    // False for Static and Kinematic (infinite mass).
    bool isDynamic() const
    {
        return mBodyType == BodyType::Dynamic;
    }

    // A mass of zero is refused; use setBodyType(Static) for immovable.
    void setMass(f32 mass);
    f32 mass() const;
    void setInverseMass(f32 inverseMass);
    f32 inverseMass() const
    {
        return mInverseMass;
    }
    bool hasFiniteMass() const
    {
        return mInverseMass > 0.0f;
    }

    // In BODY space; calculateDerivedData() rotates it into world space.
    void setInertiaTensor(const Math::mat3& inertiaTensor);
    Math::mat3 inertiaTensor() const;
    void setInverseInertiaTensor(const Math::mat3& inverseInertiaTensor);
    const Math::mat3& inverseInertiaTensor() const
    {
        return mInverseInertiaTensor;
    }
    const Math::mat3& inverseInertiaTensorWorld() const
    {
        return mInverseInertiaTensorWorld;
    }

    // Per-second retention, applied as pow(damping, dt) so it is independent of
    // the step size. 1 is no damping, which lets integration error add energy
    // unchecked and a stack slowly explode.
    void setDamping(f32 linear, f32 angular);
    f32 linearDamping() const
    {
        return mLinearDamping;
    }
    f32 angularDamping() const
    {
        return mAngularDamping;
    }

    void setPosition(const Math::vec3& position);
    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setOrientation(const Math::quat& orientation);
    const Math::quat& orientation() const
    {
        return mOrientation;
    }

    void setVelocity(const Math::vec3& velocity);
    const Math::vec3& velocity() const
    {
        return mVelocity;
    }
    void setAngularVelocity(const Math::vec3& angularVelocity);
    const Math::vec3& angularVelocity() const
    {
        return mAngularVelocity;
    }

    // Constant acceleration that ignores mass (gravity); apart from the force
    // accumulator, which is scaled by inverse mass.
    void setAcceleration(const Math::vec3& acceleration);
    const Math::vec3& acceleration() const
    {
        return mAcceleration;
    }
    // Acceleration the last step ran with, gravity included; contact resolution
    // needs it to tell a resting body from a colliding one.
    const Math::vec3& lastFrameAcceleration() const
    {
        return mLastFrameAcceleration;
    }

    const Math::mat4& transform() const
    {
        return mTransform;
    }
    Math::vec3 pointToWorld(const Math::vec3& local) const;
    Math::vec3 pointToLocal(const Math::vec3& world) const;
    Math::vec3 directionToWorld(const Math::vec3& local) const;
    Math::vec3 directionToLocal(const Math::vec3& world) const;
    // World-space velocity of a point attached to the body, v + w x r.
    Math::vec3 velocityAtPoint(const Math::vec3& worldPoint) const;

    // Forces accumulate over a step and are cleared by integrate(). The
    // accessors read what has been added so far this step.
    const Math::vec3& forceAccumulator() const
    {
        return mForceAccumulator;
    }
    const Math::vec3& torqueAccumulator() const
    {
        return mTorqueAccumulator;
    }
    void addForce(const Math::vec3& force);
    void addForceAtPoint(const Math::vec3& force, const Math::vec3& worldPoint);
    void addForceAtBodyPoint(const Math::vec3& force, const Math::vec3& localPoint);
    void addTorque(const Math::vec3& torque);
    void clearAccumulators();

    // Impulses change velocity immediately; contacts and constraints use them
    // because the correction must hold at the end of the step.
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);
    void applyImpulseAtPoint(const Math::vec3& impulse, const Math::vec3& worldPoint);
    // Split-impulse position correction: changes pose without adding kinetic energy.
    void applyPositionImpulseAtPoint(const Math::vec3& impulse, const Math::vec3& worldPoint);

    // A sleeping body is skipped by integrate(). Any force or impulse wakes it;
    // falling asleep is decided by integrate() from a running average of motion.
    void setAwake(bool awake);
    bool awake() const
    {
        return mAwake;
    }
    void setCanSleep(bool canSleep);
    bool canSleep() const
    {
        return mCanSleep;
    }
    void setSleepEpsilon(f32 epsilon);
    void setMaxSpeeds(f32 linear, f32 angular);
    f32 sleepEpsilon() const
    {
        return mSleepEpsilon;
    }
    f32 motion() const
    {
        return mMotion;
    }

    // integrate() still updates the motion average but never sleeps on its own.
    // PhysicsWorld sets it, since sleep belongs to a whole contact island: a box
    // that stops first would otherwise hang in the air over a settling stack.
    void setSleepDeferred(bool deferred);
    bool sleepDeferred() const
    {
        return mSleepDeferred;
    }

    // Rebuilds the transform and world-space inverse inertia tensor from
    // position and orientation. integrate() ends with it; call it after setting
    // state directly.
    void calculateDerivedData();

    // Split integration: forces are applied before solving contacts and joints,
    // then poses advance with the corrected velocities.
    void integrateForces(f32 duration);
    void integrateVelocity(f32 duration);

    void integrate(f32 duration);

private:
    void applyBodyTypeMass();

    BodyType mBodyType = BodyType::Dynamic;

    f32 mInverseMass = 1.0f;
    Math::mat3 mInverseInertiaTensor{1.0f};
    Math::mat3 mInverseInertiaTensorWorld{1.0f};

    f32 mLinearDamping = 0.99f;
    f32 mAngularDamping = 0.99f;

    Math::vec3 mPosition{0.0f};
    Math::quat mOrientation{1.0f, 0.0f, 0.0f, 0.0f};
    Math::vec3 mVelocity{0.0f};
    Math::vec3 mAngularVelocity{0.0f};

    Math::vec3 mAcceleration{0.0f};
    Math::vec3 mLastFrameAcceleration{0.0f};

    Math::vec3 mForceAccumulator{0.0f};
    Math::vec3 mTorqueAccumulator{0.0f};

    Math::mat4 mTransform{1.0f};

    // Mass and inertia as set by the caller, so switching back to Dynamic after
    // Static or Kinematic restores them.
    f32 mDynamicInverseMass = 1.0f;
    Math::mat3 mDynamicInverseInertiaTensor{1.0f};

    bool mAwake = true;
    bool mCanSleep = true;
    bool mSleepDeferred = false;
    f32 mMotion = 0.0f;
    f32 mSleepEpsilon = 0.3f;
    // Speeds are clamped here each step: a sliver with a tiny inertia takes an
    // ordinary impulse and spins at thousands of radians a second otherwise.
    f32 mMaxLinearSpeed = 500.0f;
    f32 mMaxAngularSpeed = 47.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_RIGIDBODY_H
