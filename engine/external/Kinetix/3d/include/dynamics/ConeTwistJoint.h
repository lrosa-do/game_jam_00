#ifndef KINETIX_PHYSICS_DYNAMICS_CONETWISTJOINT_H
#define KINETIX_PHYSICS_DYNAMICS_CONETWISTJOINT_H

#include "dynamics/Joint.h"
#include "dynamics/JointDrive.h"
#include <kinetix/core/Math.h>

namespace kinetix::Physics
{

class ConeTwistJoint final : public Joint
{
public:
    ConeTwistJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
            const Math::vec3& worldTwistAxis, const Math::vec3& worldPlaneAxis);
    ConeTwistJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localTwistAxisA,
            const Math::vec3& localPlaneAxisA, RigidBody& b, const Math::vec3& localAnchorB,
            const Math::vec3& localTwistAxisB, const Math::vec3& localPlaneAxisB);

    void configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
            const Math::vec3& worldTwistAxis, const Math::vec3& worldPlaneAxis);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    Math::vec3 anchorWorldA() const;
    Math::vec3 anchorWorldB() const;
    Math::vec3 axisWorld() const;

    // Half-angles of the swing cone, each in [0, pi]. Zero locks that swing
    // direction; both zero lock the swing.
    void setSwingLimits(f32 halfAngleY, f32 halfAngleZ);
    f32 swingHalfAngleY() const
    {
        return mSwingHalfAngleY;
    }
    f32 swingHalfAngleZ() const
    {
        return mSwingHalfAngleZ;
    }

    // Twist from the rest pose, any range inside [-pi, pi] (as
    // HingeJoint::setLimits()). Narrower than a degree is a lock at its middle.
    void setTwistLimits(f32 minAngle, f32 maxAngle);
    f32 twistMinAngle() const
    {
        return mTwistMinAngle;
    }
    f32 twistMaxAngle() const
    {
        return mTwistMaxAngle;
    }

    void resetWarmStart();

    // B's constraint frame relative to A's: full rotation, twist about the twist
    // axis (rad, -pi..pi) and swing as a rotation vector in the frame's Y and Z (rad).
    Math::quat relativeOrientation() const;
    f32 twistAngle() const;
    Math::vec2 swingAngles() const;
    // Swing (rotation of |swing| about (0, swing.y, swing.z) of the constraint
    // frame) applied after the twist, the split relativeOrientation() reads.
    static Math::quat orientationFromTwistSwing(f32 twist, f32 swingY, f32 swingZ);

    // Orientation servo: turns B's constraint frame towards `target` relative
    // to A's along the shortest rotation, as one three-axis spring-damper with
    // the gains of `settings` (maxForce is the torque limit on the whole vector,
    // N*m; maxVelocity the slew rate, rad/s). Limits still apply. Off until set.
    void setServo(const Math::quat& target, const ServoSettings& settings);
    void setServoTarget(const Math::quat& target);
    void disableServo();
    bool servoEnabled() const
    {
        return mServoEnabled;
    }
    const Math::quat& servoTarget() const
    {
        return mServoTarget;
    }
    const Math::vec3& servoError() const
    {
        return mServoError;
    }

    Math::vec3 servoTorque() const;
    Math::vec3 swingLimitTorque() const;
    Math::vec3 twistLimitTorque() const;
    Math::vec3 lockForce() const;
    Math::vec3 reactionForce() const override;
    Math::vec3 reactionTorque() const override;

private:
    void calculateServoProperties(f32 duration);
    void calculatePositionProperties();
    void calculateConstraintFrame();
    void calculateSwingTwistProperties();
    void calculateLimitEffectiveMass(const Math::vec3& axis, f32& effectiveMass) const;
    f32 twistLockAngle() const;
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);
    void rotateBody(RigidBody& body, const Math::vec3& step) const;

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::vec3 mLocalTwistAxisA;
    Math::vec3 mLocalPlaneAxisA;
    Math::vec3 mLocalTwistAxisB;
    Math::vec3 mLocalPlaneAxisB;

    Math::vec3 mArmA{ 0.0f };
    Math::vec3 mArmB{ 0.0f };
    Math::mat3 mPositionEffectiveMass{ 0.0f };
    Math::vec3 mTotalPositionImpulse{ 0.0f };

    Math::vec3 mWorldTwistAxis{ 1.0f, 0.0f, 0.0f };
    Math::vec3 mWorldPlaneAxis{ 0.0f, 0.0f, 1.0f };
    Math::vec3 mWorldNormalAxis{ 0.0f, 1.0f, 0.0f };
    Math::quat mConstraintToWorld{ 1.0f, 0.0f, 0.0f, 0.0f };

    Math::quat mSwing{ 1.0f, 0.0f, 0.0f, 0.0f };
    Math::quat mTwist{ 1.0f, 0.0f, 0.0f, 0.0f };

    f32 mSwingHalfAngleY = Math::pi<f32>();
    f32 mSwingHalfAngleZ = Math::pi<f32>();
    f32 mTwistMinAngle = -Math::pi<f32>();
    f32 mTwistMaxAngle = Math::pi<f32>();

    bool mSwingActive = false;
    Math::vec3 mWorldSwingLimitAxis{ 0.0f, 1.0f, 0.0f };
    f32 mSwingLimitEffectiveMass = 0.0f;
    f32 mTotalSwingLimitImpulse = 0.0f;

    bool mTwistActive = false;
    bool mTwistLocked = false;
    bool mTwistMinClosest = true;
    Math::vec3 mWorldTwistLimitAxis{ 1.0f, 0.0f, 0.0f };
    f32 mTwistLimitEffectiveMass = 0.0f;
    f32 mTotalTwistLimitImpulse = 0.0f;

    bool mServoEnabled = false;
    bool mServoTargetChanged = false;
    bool mServoActive = false;
    bool mServoPositionCorrectable = false;
    Math::quat mServoTarget{ 1.0f, 0.0f, 0.0f, 0.0f };
    ServoSettings mServoSettings;
    Math::vec3 mServoError{ 0.0f };
    Math::vec3 mServoDesiredVelocity{ 0.0f };
    Math::mat3 mServoEffectiveMass{ 0.0f };
    Math::mat3 mServoSoftness{ 0.0f };
    Math::mat3 mServoCompensation{ 0.0f };
    f32 mServoMaxImpulse = 0.0f;
    Math::vec3 mTotalServoImpulse{ 0.0f };

    f32 mPreviousDuration = 0.0f;
};

} // namespace kinetix::Physics

#endif
