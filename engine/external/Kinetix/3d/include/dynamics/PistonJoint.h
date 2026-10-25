#ifndef KINETIX_PHYSICS_DYNAMICS_PISTONJOINT_H
#define KINETIX_PHYSICS_DYNAMICS_PISTONJOINT_H

#include <kinetix/core/Math.h>
#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"

namespace kinetix::Physics
{

// Prismatic and revolute freedom along the same axis (shock absorber, strut):
// 2 translation + 2 rotation locked, 1 translation + 1 rotation free, each
// limited and motored independently.
class PistonJoint final : public Joint
{
public:
    PistonJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                const Math::vec3& worldAxis);

    // Retargets an existing joint (world space) so it can be reused.
    void configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                   const Math::vec3& worldAxis);
    PistonJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localAxisA,
                const Math::vec3& localNormalAxisA, RigidBody& b, const Math::vec3& localAnchorB,
                const Math::vec3& localAxisB, const Math::vec3& localNormalAxisB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    void setLinearLimits(f32 minDistance, f32 maxDistance);
    // Angles about the axis from the rest pose, any range inside [-pi, pi] (as
    // HingeJoint::setLimits()).
    void setAngularLimits(f32 minAngle, f32 maxAngle);
    f32 currentPosition() const;
    f32 currentAngle() const;
    // m/s along the axis, rad/s about it (B relative to A).
    f32 currentVelocity() const;
    f32 currentAngularVelocity() const;

    // Two independent drives (JointDrive.h), one per free axis: a velocity motor
    // or a position servo, never both; each setter replaces the other on its
    // axis and either disable turns that axis off.
    void setLinearMotor(f32 targetVelocity, f32 maxForce);
    void disableLinearMotor();
    void setAngularMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableAngularMotor();
    void setLinearServo(f32 targetPosition, const ServoSettings& settings);
    void setLinearServoTarget(f32 targetPosition, f32 targetVelocity = 0.0f);
    void disableLinearServo();
    // The angle wraps the short way round unless the limits forbid it.
    void setAngularServo(f32 targetAngle, const ServoSettings& settings);
    void setAngularServoTarget(f32 targetAngle, f32 targetAngularVelocity = 0.0f);
    void disableAngularServo();
    JointDrive& linearDrive()
    {
        return mLinearDrive;
    }
    const JointDrive& linearDrive() const
    {
        return mLinearDrive;
    }
    JointDrive& angularDrive()
    {
        return mAngularDrive;
    }
    const JointDrive& angularDrive() const
    {
        return mAngularDrive;
    }

    // Sensors from the last solved step (Joint::reactionForce): positive along /
    // about the axis in the direction of the increasing coordinate.
    f32 linearMotorForce() const;
    f32 angularMotorTorque() const;
    f32 linearLimitForce() const;
    f32 angularLimitTorque() const;
    // Two lateral rows (world, N) and the two axis-alignment rows (world, N*m).
    Math::vec3 lockForce() const;
    Math::vec3 lockTorque() const;
    Math::vec3 reactionForce() const override;
    Math::vec3 reactionTorque() const override;
    Math::vec3 axisWorld() const;

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculateRotationLockProperties();
    void calculateAxisAndPosition();
    f32 calculateLinearEffectiveMass() const;
    void calculateLinearLimitProperties();
    void calculateAngularLimitProperties(f32 duration);
    void calculateDriveProperties(f32 duration);
    void solveDrivePosition(f32 baumgarte);
    f32 axialVelocity() const;
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);
    void applyLockCompensation(const f32* compensation, f32 impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::vec3 mLocalAxisA;
    Math::vec3 mLocalAxisB;
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

    Math::vec3 mA1{0.0f, 1.0f, 0.0f};
    Math::vec3 mB2{1.0f, 0.0f, 0.0f};
    Math::vec3 mC2{0.0f, 0.0f, 1.0f};
    Math::vec3 mB2xA1{0.0f};
    Math::vec3 mC2xA1{0.0f};
    Math::mat2 mRotationLockEffectiveMass{0.0f};
    Math::vec2 mTotalRotationLockImpulse{0.0f};

    f32 mSlidePosition = 0.0f;
    f32 mTheta = 0.0f;

    f32 mLinearLimitsMin = -M_INFINITY;
    f32 mLinearLimitsMax = M_INFINITY;
    bool mHasLinearLimits = false;
    bool mLinearLimitActive = false;
    f32 mLinearLimitEffectiveMass = 0.0f;
    f32 mTotalLinearLimitImpulse = 0.0f;

    f32 mAngularLimitsMin = -Math::pi<f32>();
    f32 mAngularLimitsMax = Math::pi<f32>();
    bool mHasAngularLimits = false;
    bool mAngularLimitActive = false;
    f32 mAngularLimitEffectiveMass = 0.0f;
    f32 mTotalAngularLimitImpulse = 0.0f;

    JointDrive mLinearDrive;
    JointDrive mAngularDrive;
    // Lock impulses carried by each unit of drive impulse: two lateral rows, then
    // two rotation rows.
    f32 mLinearDriveCompensation[4] = {};
    f32 mAngularDriveCompensation[4] = {};

    f32 mPreviousDuration = 0.0f;
};

}

#endif
