#ifndef KINETIX_PHYSICS_DYNAMICS_HINGEJOINT_H
#define KINETIX_PHYSICS_DYNAMICS_HINGEJOINT_H

#include <kinetix/core/Math.h>
#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"

namespace kinetix::Physics
{

class HingeJoint final : public Joint
{
public:
    HingeJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
               const Math::vec3& worldHingeAxis);

    // Retargets an existing joint (world space) so it can be reused.
    void configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                   const Math::vec3& worldHingeAxis);
    HingeJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localHingeAxisA,
               const Math::vec3& localNormalAxisA, RigidBody& b, const Math::vec3& localAnchorB,
               const Math::vec3& localHingeAxisB, const Math::vec3& localNormalAxisB);

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

    // Limits as angles about the hinge axis from the rest pose (currentAngle()
    // == 0; the pose the joint was built in). Any range inside [-pi, pi],
    // including one excluding the rest pose: a body outside is pulled to the
    // nearer end around the circle. Values beyond +-pi are clamped, and a
    // maximum below the minimum is raised to it (locks the hinge).
    void setLimits(f32 minAngle, f32 maxAngle);
    f32 currentAngle() const;
    // B's angular velocity relative to A about the hinge axis, rad/s.
    f32 currentAngularVelocity() const;

    // Velocity motor or position servo on the angle, never both: setMotor() and
    // setServo() replace each other; the disable calls turn the drive off.
    void setMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableMotor();
    // Turns the hinge to `targetAngle` (rad, frame of currentAngle()), the short
    // way round unless the limits forbid it, clamped into the limits.
    void setServo(f32 targetAngle, const ServoSettings& settings);
    void setServoTarget(f32 targetAngle, f32 targetAngularVelocity = 0.0f);
    void disableServo();
    JointDrive& drive()
    {
        return mDrive;
    }
    const JointDrive& drive() const
    {
        return mDrive;
    }

    // Sensors from the last solved step (see Joint::reactionForce). Torques
    // about the hinge axis are positive in the direction of increasing currentAngle().
    f32 motorTorque() const;
    f32 limitTorque() const;
    // Point lock's force on B (world, N) and the torque of the two rows holding
    // the axes aligned (world, N*m, perpendicular to the axis).
    Math::vec3 lockForce() const;
    Math::vec3 lockTorque() const;
    Math::vec3 reactionForce() const override;
    Math::vec3 reactionTorque() const override;
    Math::vec3 axisWorld() const;
    // A teleport must not reuse impulses accumulated in the previous frame.
    void resetWarmStart();

private:
    void calculatePositionProperties();
    void calculateHingeRotationProperties();
    void calculateAxisAndAngle();
    void calculateLimitProperties(f32 duration);
    void calculateDriveProperties(f32 duration);
    void solveDrivePosition(f32 baumgarte);
    f32 smallestAngleToLimit() const;
    bool minLimitClosest() const;
    void applyVelocityImpulse(const Math::vec3& impulse);
    void applyAngularVelocityImpulse(const Math::vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::vec3 mLocalHingeAxisA;
    Math::vec3 mLocalHingeAxisB;
    Math::vec3 mLocalNormalAxisA;
    Math::vec3 mLocalNormalAxisB;
    Math::quat mInverseInitialOrientation;

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::mat3 mPositionEffectiveMass{0.0f};
    Math::vec3 mTotalPositionImpulse{0.0f};

    Math::vec3 mA1{0.0f, 1.0f, 0.0f};
    Math::vec3 mB2{1.0f, 0.0f, 0.0f};
    Math::vec3 mC2{0.0f, 0.0f, 1.0f};
    Math::vec3 mB2xA1{0.0f};
    Math::vec3 mC2xA1{0.0f};
    Math::mat2 mHingeRotationEffectiveMass{0.0f};
    Math::vec2 mTotalHingeRotationImpulse{0.0f};

    f32 mTheta = 0.0f;
    f32 mLimitsMin = -Math::pi<f32>();
    f32 mLimitsMax = Math::pi<f32>();
    bool mHasLimits = false;
    f32 mLimitEffectiveMass = 0.0f;
    f32 mTotalLimitImpulse = 0.0f;
    bool mLimitActive = false;

    JointDrive mDrive;
    // Lock impulses carried by each unit of drive impulse: point lock's three,
    // then the two rotation rows.
    f32 mDriveCompensation[5] = {};

    f32 mPreviousDuration = 0.0f;
};

}

#endif
