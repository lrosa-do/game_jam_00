#ifndef KINETIX_PHYSICS_DYNAMICS_SLIDERJOINT_H
#define KINETIX_PHYSICS_DYNAMICS_SLIDERJOINT_H

#include <kinetix/core/Math.h>
#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"

namespace kinetix::Physics
{

class SliderJoint final : public Joint
{
public:
    // A zero slider axis falls back to X and a zero or parallel normal axis to
    // one perpendicular to it; B's missing axes fall back to A's (JointAxis.h).
    SliderJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                const Math::vec3& worldSliderAxis);
    SliderJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localSliderAxisA,
                const Math::vec3& localNormalAxisA, RigidBody& b, const Math::vec3& localAnchorB,
                const Math::vec3& localSliderAxisB, const Math::vec3& localNormalAxisB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    bool supportsArticulation() const override
    {
        return true;
    }
    u32 articulationLocks(JointRow* rows, f32* errors) override;
    void addArticulationLockImpulses(const f32* impulses) override;
    u32 articulationMotors(ArticulationMotor* motors) override;
    void setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass) override;
    f32 solveArticulationMotor(u32 id, f32 velocity) override;
    u32 articulationPositionRows(JointRow* rows, f32* errors) override;

    void setLimits(f32 minDistance, f32 maxDistance);
    // B's anchor relative to A's along the slider axis, m (0 at the rest
    // pose), and its rate, m/s.
    f32 currentPosition() const;
    f32 currentVelocity() const;

    // Velocity motor or position servo on currentPosition(), never both: each
    // setter replaces the other, either disable turns the drive off. The servo
    // target is clamped into the limits.
    void setMotor(f32 targetVelocity, f32 maxForce);
    void disableMotor();
    void setServo(f32 targetPosition, const ServoSettings& settings);
    void setServoTarget(f32 targetPosition, f32 targetVelocity = 0.0f);
    void disableServo();
    JointDrive& drive()
    {
        return mDrive;
    }
    const JointDrive& drive() const
    {
        return mDrive;
    }

    // Sensors from the last solved step (Joint::reactionForce); axial forces are
    // positive in the direction of increasing currentPosition().
    f32 motorForce() const;
    f32 limitForce() const;
    // The two lateral rows (world, N) and the rotation lock (world, N*m).
    Math::vec3 lockForce() const;
    Math::vec3 lockTorque() const;
    Math::vec3 reactionForce() const override;
    Math::vec3 reactionTorque() const override;
    Math::vec3 axisWorld() const;

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculateRotationProperties();
    void calculateSlideAxisAndPosition();
    f32 calculateAxialEffectiveMass() const;
    void calculateLimitProperties();
    void calculateDriveProperties(f32 duration);
    void solveDrivePosition(f32 baumgarte);
    f32 axialVelocity() const;
    void applyVelocityImpulse(const Math::vec3& impulse);
    void applyAngularVelocityImpulse(const Math::vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::vec3 mLocalSliderAxisA;
    Math::vec3 mLocalNormalAxisA;
    Math::vec3 mLocalNormalAxisA2;
    Math::quat mInverseInitialOrientation;

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::vec3 mOffset{0.0f};

    Math::vec3 mN1{1.0f, 0.0f, 0.0f};
    Math::vec3 mN2{0.0f, 0.0f, 1.0f};
    Math::mat2 mPositionLockEffectiveMass{0.0f};
    Math::vec2 mTotalPositionLockImpulse{0.0f};

    Math::mat3 mRotationEffectiveMass{0.0f};
    Math::vec3 mTotalRotationImpulse{0.0f};

    Math::vec3 mWorldSliderAxis{1.0f, 0.0f, 0.0f};
    f32 mSlidePosition = 0.0f;

    f32 mLimitsMin = -M_INFINITY;
    f32 mLimitsMax = M_INFINITY;
    bool mHasLimits = false;
    bool mLimitActive = false;
    f32 mLimitEffectiveMass = 0.0f;
    f32 mTotalLimitImpulse = 0.0f;

    JointDrive mDrive;
    // Lock impulses carried by each unit of drive impulse: two lateral rows,
    // then three rotation rows.
    f32 mDriveCompensation[5] = {};

    f32 mPreviousDuration = 0.0f;
};

}

#endif
