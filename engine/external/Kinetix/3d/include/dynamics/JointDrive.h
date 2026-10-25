#ifndef KINETIX_PHYSICS_DYNAMICS_JOINTDRIVE_H
#define KINETIX_PHYSICS_DYNAMICS_JOINTDRIVE_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Gains of a position servo, in one of three forms:
//  - frequency > 0: natural frequency (Hz) and damping ratio, scaled every step
//    by the inertia the axis drives (the bodies as the joint's other rows hold
//    them), so the response is independent of what hangs off the joint. Further
//    links of a chain add inertia this joint cannot see (lower real frequency).
//    dampingRatio 1 is critically damped.
//  - stiffness > 0 (frequency == 0): absolute PD gains, F = -kp*e - kd*v in
//    N/m and N*s/m (linear) or N*m/rad and N*m*s/rad (angular); load dependent.
//  - both zero: rigid, a hard position constraint like the joint's locks; the
//    velocity pass closes a fifth of the error per step and the position pass
//    the rest (about 60% together), limited by maxForce and maxVelocity.
//
// The spring is solved implicitly in the constraint row (see SoftSpring.h), so
// no gain is too high for the step: it degrades towards rigid instead of exploding.
//
// maxForce is the force or torque limit (N or N*m); the servo is off unless
// positive. maxVelocity > 0 limits the slew speed towards a distant target
// (m/s or rad/s): beyond it the row is a velocity motor at that speed, still
// within maxForce.
struct ServoSettings
{
    f32 frequency = 0.0f;
    f32 dampingRatio = 1.0f;
    f32 stiffness = 0.0f;
    f32 damping = 0.0f;
    f32 maxForce = 0.0f;
    f32 maxVelocity = 0.0f;
};

// One driven degree of freedom of a joint: the motor/servo row the joints
// share. The joint measures its coordinate and effective mass every setup()
// and hands them to prepare().
//
// Two mutually exclusive modes per axis:
//  - Velocity: a motor holding a target speed within a force limit, optionally
//    with a viscous gain: force = damping * (target - speed), still clamped
//    (electric-motor torque-speed line), solved implicitly.
//  - Position: the ServoSettings servo towards a target coordinate, with an
//    optional velocity feed-forward.
//
// The coordinate is the joint's own (currentAngle()/currentPosition(), from the
// rest pose); force (N or N*m) is applied to body B in the direction that
// increases it, A gets the opposite. Angles wrap: a target across +-pi is
// reached the short way unless the limits leave only the long one.
class JointDrive
{
public:
    enum class Mode : u8
    {
        Off,
        Velocity,
        Position
    };

    // Velocity motor. A maxForce of zero or below turns the drive off;
    // non-finite arguments are ignored.
    void setVelocity(f32 targetVelocity, f32 maxForce, f32 damping = 0.0f);
    void setServo(f32 targetPosition, const ServoSettings& settings);
    // Moves the servo's target (and feed-forward speed) keeping its gains; for
    // per-step controllers. Ignored unless in Position mode.
    void setServoTarget(f32 targetPosition, f32 targetVelocity = 0.0f);
    void disable();

    // Optional asymmetric force range on top of maxForce (e.g. a hydraulic
    // cylinder: cap area pushes, annulus pulls). Needs minForce <= 0 <= maxForce.
    void setForceRange(f32 minForce, f32 maxForce);
    void clearForceRange();
    // Optional torque-speed envelope, joint side: force at speed v is
    // stallForce * (1 - v / noLoadSpeed) along the motion and
    // stallForce * (1 + v / noLoadSpeed) against it, capped at stallForce.
    // noLoadSpeed <= 0 removes it.
    void setTorqueSpeedLine(f32 stallForce, f32 noLoadSpeed);

    Mode mode() const
    {
        return mMode;
    }
    bool enabled() const
    {
        return mMode != Mode::Off;
    }
    f32 targetVelocity() const
    {
        return mTargetVelocity;
    }
    f32 targetPosition() const
    {
        return mTargetPosition;
    }
    f32 maxForce() const
    {
        return mMaxForce;
    }
    f32 velocityDamping() const
    {
        return mVelocityDamping;
    }
    const ServoSettings& servoSettings() const
    {
        return mServo;
    }

    // Readouts from the last solved step. appliedForce is impulse / step, N or
    // N*m, signed as above. position/velocity are as setup() measured them (the
    // rate as the step ended, if solved).
    f32 appliedForce() const;
    f32 position() const
    {
        return mPosition;
    }
    f32 velocity() const
    {
        return mVelocity;
    }
    // Servo error (position - target, wrapped for angles), 0 otherwise.
    f32 positionError() const
    {
        return mError;
    }
    // The speed the row drove towards: the motor's target, or the servo's speed
    // after the maxVelocity clamp.
    f32 commandedVelocity() const
    {
        return mDesiredVelocity;
    }
    // Whether the accumulated force sits on a limit.
    bool saturated() const;

    struct Coordinate
    {
        f32 position = 0.0f;
        f32 velocity = 0.0f;
        bool angular = false;
        bool hasLimits = false;
        f32 minPosition = 0.0f;
        f32 maxPosition = 0.0f;
        // Inverse inertia the frequency gains are scaled by, when different from
        // the row's (detail::coupleDriveToLocks). Zero uses the row's.
        f32 gainInverseMass = 0.0f;
    };

    // Works out the row for this step from the joint's measurement and the
    // inverse effective mass it is solved with (see detail::coupleDriveToLocks).
    // Returns whether the row is solved this step. Carries the warm-start
    // impulse over from the previous step, scaled to this step's duration.
    bool prepare(f32 duration, f32 inverseEffectiveMass, const Coordinate& coordinate);
    // Re-derives the row for an inverse effective mass found after prepare()
    // (the articulation solver's): the mass the row is solved with, and the one
    // frequency gains are scaled by (zero: the first). False, changing nothing,
    // if the row is not active this step.
    bool setInverseMass(f32 inverseEffectiveMass, f32 gainInverseMass);
    // The impulse to add given the row's velocity; the caller applies it.
    f32 solve(f32 velocity);
    bool active() const
    {
        return mActive;
    }
    // For the joint's position pass: what is left of a rigid servo's error after
    // the velocity pass, measured from `coordinate` and the target at the end of
    // the step. A rigid servo is one more hard row; closing it only at the
    // velocity level fights the locks' position pass in a chain. False when
    // there is nothing to correct (no rigid servo, slewing at maxVelocity, or
    // saturated at maxForce).
    bool positionError(const Coordinate& coordinate, f32& error) const;
    f32 totalImpulse() const
    {
        return mTotalImpulse;
    }
    // Whether setup() found work to do (moving target, error not yet held against
    // a load); the joint then wakes its bodies.
    bool wantsWake() const
    {
        return mWantsWake;
    }
    void resetWarmStart();

private:
    bool atBound() const;
    void configure(f32 inverseEffectiveMass, f32 gainInverseMass);

    Mode mMode = Mode::Off;
    f32 mTargetVelocity = 0.0f;
    f32 mTargetPosition = 0.0f;
    f32 mMaxForce = 0.0f;
    f32 mVelocityDamping = 0.0f;
    ServoSettings mServo;

    bool mHasForceRange = false;
    f32 mRangeMin = 0.0f;
    f32 mRangeMax = 0.0f;
    f32 mStallForce = 0.0f;
    f32 mNoLoadSpeed = 0.0f;

    // A new target or mode switch on a sleeping joint is never stepped; the next
    // prepare() turns this into a wake.
    bool mTargetChanged = false;
    bool mPreparedTargetChanged = false;
    Coordinate mCoordinate;

    bool mActive = false;
    bool mWantsWake = false;
    f32 mPosition = 0.0f;
    f32 mVelocity = 0.0f;
    f32 mError = 0.0f;
    f32 mDesiredVelocity = 0.0f;
    bool mPositionCorrectable = false;
    f32 mEndTarget = 0.0f;
    f32 mStiffness = 0.0f;
    f32 mEffectiveMass = 0.0f;
    f32 mInverseEffectiveMass = 0.0f;
    f32 mSoftness = 0.0f;
    f32 mMinImpulse = 0.0f;
    f32 mMaxImpulse = 0.0f;
    f32 mTotalImpulse = 0.0f;
    f32 mDuration = 0.0f;
    f32 mPreviousDuration = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_JOINTDRIVE_H
