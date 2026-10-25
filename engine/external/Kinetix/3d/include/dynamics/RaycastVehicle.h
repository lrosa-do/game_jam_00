#ifndef KINETIX_PHYSICS_DYNAMICS_RAYCASTVEHICLE_H
#define KINETIX_PHYSICS_DYNAMICS_RAYCASTVEHICLE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class RigidBody;
class PhysicsWorld;

// A rigid body turned into a car by four or more raycast-probed wheels: no
// collision shape per wheel, a ray from the chassis along the suspension
// travel, a spring-damper on how far it reached, and a forward/sideways
// friction pair applied as impulses at the contact point. The wheels never
// enter the broadphase.
class RaycastVehicle
{
public:
    struct Tuning
    {
        f32 suspensionStiffness = 5.88f;
        f32 suspensionCompression = 0.83f;
        f32 suspensionDamping = 0.88f;
        f32 maxSuspensionTravelCm = 500.0f;
        f32 frictionSlip = 10.5f;
        f32 maxSuspensionForce = 6000.0f;
    };

    struct Wheel
    {
        Math::vec3 chassisConnectionLocal{0.0f};
        Math::vec3 directionLocal{0.0f, -1.0f, 0.0f};
        Math::vec3 axleLocal{-1.0f, 0.0f, 0.0f};
        f32 restLength = 0.3f;
        f32 radius = 0.4f;

        f32 stiffness = 5.88f;
        f32 dampingCompression = 0.83f;
        f32 dampingRelaxation = 0.88f;
        f32 frictionSlip = 10.5f;
        f32 rollInfluence = 0.1f;
        f32 maxSuspensionTravelCm = 500.0f;
        f32 maxSuspensionForce = 6000.0f;
        bool isFrontWheel = false;

        f32 engineForce = 0.0f;
        f32 brake = 0.0f;
        f32 steering = 0.0f;

        f32 rotation = 0.0f;
        f32 deltaRotation = 0.0f;
        f32 skidInfo = 1.0f;

        bool inContact = false;
        u32 groundBody = 0xFFFFFFFFu;
        Math::vec3 contactPoint{0.0f};
        Math::vec3 contactNormal{0.0f, 1.0f, 0.0f};
        Math::vec3 hardPointWorld{0.0f};
        Math::vec3 directionWorld{0.0f, -1.0f, 0.0f};
        Math::vec3 axleWorld{-1.0f, 0.0f, 0.0f};
        f32 suspensionLength = 0.0f;
        f32 suspensionRelativeVelocity = 0.0f;
        f32 clippedInvContactDotSuspension = 1.0f;
        f32 suspensionForce = 0.0f;
        // What the last update applied (a lean controller weighs its target by these).
        f32 appliedSuspensionImpulse = 0.0f;
        f32 appliedSideImpulse = 0.0f;
        Math::vec3 lateralWorld{1.0f, 0.0f, 0.0f};

        Math::mat4 worldTransform{1.0f};
    };

    RaycastVehicle(RigidBody& chassis, const PhysicsWorld* world, u32 chassisBodyId);

    u32 addWheel(const Math::vec3& connectionPointLocal, const Math::vec3& directionLocal,
                const Math::vec3& axleLocal, f32 suspensionRestLength, f32 wheelRadius,
                const Tuning& tuning, bool isFrontWheel);

    void update(f32 step);
    void resetSuspension();

    // An index past wheelCount() is reported and ignored.
    void setSteering(f32 steering, u32 wheelIndex);
    void setEngineForce(f32 force, u32 wheelIndex);
    void setBrake(f32 brake, u32 wheelIndex);

    u32 wheelCount() const
    {
        return static_cast<u32>(mWheels.size());
    }
    const Wheel& wheel(u32 index) const
    {
        return mWheels[index];
    }
    Wheel& wheel(u32 index)
    {
        return mWheels[index];
    }

    f32 currentSpeedKmHour() const
    {
        return mCurrentSpeedKmHour;
    }

    RigidBody& chassis()
    {
        return mChassis;
    }
    const RigidBody& chassis() const
    {
        return mChassis;
    }

private:
    void updateWheelTransformWS(Wheel& wheel);
    void updateWheelTransform(Wheel& wheel);
    f32 rayCast(Wheel& wheel);
    void updateSuspension();
    void updateFriction(f32 step);

    RigidBody& mChassis;
    const PhysicsWorld* mWorld;
    u32 mChassisBodyId;

    ct::Vector<Wheel> mWheels;
    ct::Vector<Math::vec3> mForwardWS;
    ct::Vector<Math::vec3> mAxleWS;
    ct::Vector<f32> mForwardImpulse;
    ct::Vector<f32> mSideImpulse;

    f32 mCurrentSpeedKmHour = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_RAYCASTVEHICLE_H
