#ifndef KINETIX_PHYSICS_DYNAMICS_UNIVERSALJOINT_H
#define KINETIX_PHYSICS_DYNAMICS_UNIVERSALJOINT_H

#include <kinetix/core/Math.h>
#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"

namespace kinetix::Physics
{

// A ball-and-socket point plus two hinge axes held perpendicular to each other
// (CV joint, wrist gimbal): 3 point rows + 1 perpendicularity row, 2 free
// rotations, each limited and motored independently. Axis angles come from a
// relative orientation captured at construction (as FixedJoint does), each
// axis's twist read by the swing-twist projection HingeJoint uses.
class UniversalJoint final : public Joint
{
public:
    // A zero axis falls back to one perpendicular to the other (see JointAxis.h).
    UniversalJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                   const Math::vec3& worldAxisA, const Math::vec3& worldAxisB);
    UniversalJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localAxisA,
                   RigidBody& b, const Math::vec3& localAnchorB, const Math::vec3& localAxisB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    // Angles about each axis from the rest pose, any range inside [-pi, pi] (as
    // HingeJoint::setLimits()).
    void setLimitsA(f32 minAngle, f32 maxAngle);
    void setLimitsB(f32 minAngle, f32 maxAngle);
    f32 currentAngleA() const;
    f32 currentAngleB() const;
    // B's angular velocity relative to A about each axis, rad/s.
    f32 currentAngularVelocityA() const;
    f32 currentAngularVelocityB() const;

    // One drive per axis (JointDrive.h): a velocity motor or a position servo,
    // never both; each setter replaces the other on its axis and either disable
    // turns that axis off. Angles wrap the short way unless the limits forbid it.
    void setMotorA(f32 targetAngularVelocity, f32 maxTorque);
    void setMotorB(f32 targetAngularVelocity, f32 maxTorque);
    void disableMotorA();
    void disableMotorB();
    void setServoA(f32 targetAngle, const ServoSettings& settings);
    void setServoB(f32 targetAngle, const ServoSettings& settings);
    void setServoTargetA(f32 targetAngle, f32 targetAngularVelocity = 0.0f);
    void setServoTargetB(f32 targetAngle, f32 targetAngularVelocity = 0.0f);
    void disableServoA();
    void disableServoB();
    JointDrive& driveA()
    {
        return mDriveA;
    }
    JointDrive& driveB()
    {
        return mDriveB;
    }
    const JointDrive& driveA() const
    {
        return mDriveA;
    }
    const JointDrive& driveB() const
    {
        return mDriveB;
    }

    // Sensors from the last solved step (Joint::reactionForce): torques about
    // each axis, positive in the direction of its increasing angle.
    f32 motorTorqueA() const;
    f32 motorTorqueB() const;
    f32 limitTorqueA() const;
    f32 limitTorqueB() const;
    // Point lock (world, N) and the row keeping the axes square (world, N*m).
    Math::vec3 lockForce() const;
    Math::vec3 lockTorque() const;
    Math::vec3 reactionForce() const override;
    Math::vec3 reactionTorque() const override;

private:
    void calculatePositionProperties();
    void calculatePerpendicularityProperties();
    void calculateAngles();
    void calculateLimitProperties(bool hasLimits, f32 theta, f32 minAngle, f32 maxAngle,
                                  const Math::vec3& axis, bool& active, f32& effectiveMass);
    void calculateDriveProperties(f32 duration);
    void solveDrivePosition(f32 baumgarte);
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);
    void applyLockCompensation(const f32* compensation, f32 impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::vec3 mLocalAxisA;
    Math::vec3 mLocalAxisB;
    Math::quat mInverseInitialOrientation;

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::mat3 mPositionEffectiveMass{0.0f};
    Math::vec3 mTotalPositionImpulse{0.0f};

    Math::vec3 mAxisA{1.0f, 0.0f, 0.0f};
    Math::vec3 mAxisB{0.0f, 1.0f, 0.0f};
    Math::vec3 mPerpendicularAxis{0.0f, 0.0f, 1.0f};
    f32 mPerpendicularity = 0.0f;
    f32 mPerpendicularEffectiveMass = 0.0f;
    f32 mTotalPerpendicularImpulse = 0.0f;

    f32 mThetaA = 0.0f;
    f32 mThetaB = 0.0f;

    f32 mLimitsMinA = -Math::pi<f32>();
    f32 mLimitsMaxA = Math::pi<f32>();
    bool mHasLimitsA = false;
    bool mLimitActiveA = false;
    f32 mLimitEffectiveMassA = 0.0f;
    f32 mTotalLimitImpulseA = 0.0f;

    f32 mLimitsMinB = -Math::pi<f32>();
    f32 mLimitsMaxB = Math::pi<f32>();
    bool mHasLimitsB = false;
    bool mLimitActiveB = false;
    f32 mLimitEffectiveMassB = 0.0f;
    f32 mTotalLimitImpulseB = 0.0f;

    JointDrive mDriveA;
    JointDrive mDriveB;
    // Lock impulses carried by each unit of drive impulse: point lock's three,
    // then the perpendicularity row.
    f32 mDriveCompensationA[4] = {};
    f32 mDriveCompensationB[4] = {};

    f32 mPreviousDuration = 0.0f;
};

}

#endif
