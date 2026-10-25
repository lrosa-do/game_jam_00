#ifndef KINETIX_PHYSICS_DYNAMICS_WHEELJOINT_H
#define KINETIX_PHYSICS_DYNAMICS_WHEELJOINT_H

#include <kinetix/core/Math.h>
#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"
#include "dynamics/SoftSpring.h"

namespace kinetix::Physics
{

class WheelJoint final : public Joint
{
public:
    // `worldSuspensionAxis` points from the chassis mount towards the ground (the
    // strut's travel direction); `worldSpinAxis` is the wheel's rolling axis. They
    // need only be linearly independent: the perpendicularity constraint squares
    // the spin axis to the suspension axis once solving starts.
    WheelJoint(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
              const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis);

    void configure(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
                   const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    f32 suspensionRestLength() const
    {
        return mSuspensionRestLength;
    }
    f32 suspensionStiffness() const
    {
        return mSuspensionStiffness;
    }
    f32 suspensionDamping() const
    {
        return mSuspensionDamping;
    }
    // Steer to an angle. The steering drive (JointDrive.h) is either this servo
    // or the steering motor below, never both. This form is the rigid servo: the
    // remaining error becomes steering speed, capped by the rack's rated speed;
    // the second form takes any ServoSettings gains.
    void setSteeringServo(f32 targetAngle, f32 maxTorque, f32 maxAngularVelocity = 0.0f);
    void setSteeringServo(f32 targetAngle, const ServoSettings& settings);
    void disableSteeringServo();
    f32 steeringServoTargetAngle() const
    {
        return mSteeringDrive.targetPosition();
    }
    f32 steeringServoMaxAngularVelocity() const
    {
        return mSteeringDrive.servoSettings().maxVelocity;
    }
    bool steeringServoEnabled() const
    {
        return mSteeringDrive.mode() == JointDrive::Mode::Position;
    }

    // The motor's target, or while the servo is on, the speed it last asked for.
    f32 steeringMotorTargetVelocity() const
    {
        return mSteeringDrive.mode() == JointDrive::Mode::Velocity ? mSteeringDrive.targetVelocity()
                                                                   : mSteeringDrive.commandedVelocity();
    }
    f32 steeringMotorMaxTorque() const
    {
        return mSteeringDrive.maxForce();
    }
    f32 spinMotorTargetVelocity() const
    {
        return mSpinDrive.targetVelocity();
    }
    f32 spinMotorMaxTorque() const
    {
        return mSpinDrive.maxForce();
    }
    JointDrive& steeringDrive()
    {
        return mSteeringDrive;
    }
    const JointDrive& steeringDrive() const
    {
        return mSteeringDrive;
    }
    JointDrive& spinDrive()
    {
        return mSpinDrive;
    }
    const JointDrive& spinDrive() const
    {
        return mSpinDrive;
    }

    // Sensors from the last solved step (Joint::reactionForce). Steering torque
    // about the suspension axis is positive towards a larger steeringAngle();
    // spin torque about the spin axis towards a larger spinAngularVelocity();
    // suspension force on the wheel along its axis, positive pushing it away
    // from the chassis.
    f32 steeringTorque() const;
    f32 spinTorque() const;
    f32 steeringLimitTorque() const;
    f32 suspensionForce() const;
    Math::vec3 reactionForce() const override;
    Math::vec3 reactionTorque() const override;
    f32 minSteeringAngle() const
    {
        return mSteeringLimitsMin;
    }
    f32 maxSteeringAngle() const
    {
        return mSteeringLimitsMax;
    }

    Math::vec3 anchorWorldA() const;
    Math::vec3 anchorWorldB() const;
    Math::vec3 axisWorld() const;

    // Spring-damper along the suspension axis. `restLength` is the anchor
    // separation (along the suspension direction) with zero spring force;
    // `stiffness` and `damping` are the F = -k*x - c*v terms per metre and per
    // metre-per-second.
    //
    // Solved as a soft constraint row (SoftSpring), not an external force: a
    // car's springs are stiff (1500 kg sitting 20 cm into its travel needs about
    // 75 kN/m) and an explicit spring that stiff blows up.
    void setSuspension(f32 restLength, f32 stiffness, f32 damping)
    {
        mSuspensionRestLength = restLength;
        mSuspensionStiffness = Math::max(stiffness, 0.0f);
        mSuspensionDamping = Math::max(damping, 0.0f);
    }
    f32 suspensionTravel() const
    {
        return mSlidePosition;
    }

    // Steering angle from the rest pose, any range inside [-pi, pi] (as
    // HingeJoint::setLimits()); the steering servo's target is clamped into it.
    void setSteeringLimits(f32 minAngle, f32 maxAngle);
    void setSteeringMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableSteeringMotor();
    f32 steeringAngle() const;

    void setSpinMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableSpinMotor();
    f32 spinAngle() const;
    f32 spinAngularVelocity() const;

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculatePerpendicularityProperties();
    void calculateAngles();
    void calculateSteeringLimitProperties();
    void calculateDriveProperties(f32 duration);
    void solveDrivePosition(f32 baumgarte);
    void calculateSuspensionProperties(f32 duration);
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);

    RigidBody* mChassis = nullptr;
    RigidBody* mWheel = nullptr;
    Math::vec3 mLocalAnchorChassis{0.0f};
    Math::vec3 mLocalAnchorWheel{0.0f};
    Math::vec3 mLocalSuspensionAxis{0.0f, -1.0f, 0.0f};
    Math::vec3 mLocalSpinAxis{1.0f, 0.0f, 0.0f};
    Math::vec3 mLocalNormalAxis{1.0f, 0.0f, 0.0f};
    Math::quat mInverseInitialOrientation{1.0f, 0.0f, 0.0f, 0.0f};
    // The wheel's spin axis at construction, squared to the suspension axis, in
    // the chassis frame: the steering angle is measured from it, so spin does not
    // disturb that angle.
    Math::vec3 mLocalNeutralSpinAxis{1.0f, 0.0f, 0.0f};

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::vec3 mOffset{0.0f};

    // Suspension axis in world space and the two lateral directions locked against it.
    Math::vec3 mAxisA{0.0f, -1.0f, 0.0f};
    Math::vec3 mN1{1.0f, 0.0f, 0.0f};
    Math::vec3 mN2{0.0f, 0.0f, 1.0f};
    Math::mat2 mPositionLockEffectiveMass{0.0f};
    Math::vec2 mTotalPositionLockImpulse{0.0f};

    // Spin axis in world space and the row keeping it square to the suspension axis.
    Math::vec3 mAxisB{1.0f, 0.0f, 0.0f};
    Math::vec3 mPerpendicularAxis{0.0f, 0.0f, 1.0f};
    f32 mPerpendicularity = 0.0f;
    f32 mPerpendicularEffectiveMass = 0.0f;
    f32 mTotalPerpendicularImpulse = 0.0f;

    f32 mSlidePosition = 0.0f;
    f32 mSteeringAngle = 0.0f;
    f32 mSpinAngleValue = 0.0f;

    f32 mSuspensionRestLength = 0.0f;
    f32 mSuspensionStiffness = 0.0f;
    f32 mSuspensionDamping = 0.0f;
    SoftSpring mSuspensionSpring;
    f32 mSuspensionEffectiveMass = 0.0f;
    f32 mTotalSuspensionImpulse = 0.0f;

    f32 mSteeringLimitsMin = -Math::pi<f32>();
    f32 mSteeringLimitsMax = Math::pi<f32>();
    bool mHasSteeringLimits = false;
    bool mSteeringLimitActive = false;
    f32 mSteeringLimitEffectiveMass = 0.0f;
    f32 mTotalSteeringLimitImpulse = 0.0f;

    JointDrive mSteeringDrive;
    JointDrive mSpinDrive;

    f32 mPreviousDuration = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_WHEELJOINT_H
