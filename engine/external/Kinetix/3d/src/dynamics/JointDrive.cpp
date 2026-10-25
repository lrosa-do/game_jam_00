#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/JointDrive.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

// How far from equilibrium a servo may be left asleep: a millimetre on a linear
// axis, a milliradian on an angular one.
constexpr f32 kWakeTolerance = 1.0e-3f;

// Fraction of a rigid servo's error the velocity pass closes per step; the
// position pass (positionError) closes the rest with the joint's other rows.
// All of it (deadbeat) is exact for one joint but in a chain each joint
// over-corrects for its neighbours and the torques flip sign every step; a
// fifth, the position pass's own fraction, settles a chain exactly.
constexpr f32 kRigidVelocityFraction = 0.2f;

f32 wrapAngle(f32 angle)
{
    while (angle < -Math::pi<f32>())
        angle += Math::two_pi<f32>();
    while (angle > Math::pi<f32>())
        angle -= Math::two_pi<f32>();
    return angle;
}

// position - target, the short way round for an angle unless the limits leave
// only the long way (the direct difference while inside them).
f32 coordinateError(const JointDrive::Coordinate& coordinate, f32 target)
{
    const f32 error = coordinate.position - target;
    if (!coordinate.angular)
        return error;
    const bool inside = !coordinate.hasLimits || (coordinate.position >= coordinate.minPosition &&
                                                  coordinate.position <= coordinate.maxPosition);
    if (!coordinate.hasLimits || !inside)
        return wrapAngle(error);
    return error;
}

bool finite(f32 value)
{
    return isfinite(value) != 0;
}

}

void JointDrive::setVelocity(f32 targetVelocity, f32 maxForce, f32 damping)
{
    if (!finite(targetVelocity) || !finite(maxForce) || !finite(damping))
        return;
    if (mMode != Mode::Velocity || targetVelocity != mTargetVelocity)
        mTargetChanged = true;
    mTargetVelocity = targetVelocity;
    mMaxForce = Math::max(maxForce, 0.0f);
    mVelocityDamping = Math::max(damping, 0.0f);
    mMode = mMaxForce > 0.0f ? Mode::Velocity : Mode::Off;
    if (mMode == Mode::Off)
        disable();
}

void JointDrive::setServo(f32 targetPosition, const ServoSettings& settings)
{
    if (!finite(targetPosition) || !finite(settings.frequency) || !finite(settings.dampingRatio) ||
        !finite(settings.stiffness) || !finite(settings.damping) || !finite(settings.maxForce) ||
        !finite(settings.maxVelocity))
        return;
    if (mMode != Mode::Position || targetPosition != mTargetPosition)
        mTargetChanged = true;
    mServo.frequency = Math::max(settings.frequency, 0.0f);
    mServo.dampingRatio = Math::max(settings.dampingRatio, 0.0f);
    mServo.stiffness = Math::max(settings.stiffness, 0.0f);
    mServo.damping = Math::max(settings.damping, 0.0f);
    mServo.maxForce = Math::max(settings.maxForce, 0.0f);
    mServo.maxVelocity = Math::max(settings.maxVelocity, 0.0f);
    mTargetPosition = targetPosition;
    mTargetVelocity = 0.0f;
    mMaxForce = mServo.maxForce;
    mMode = mMaxForce > 0.0f ? Mode::Position : Mode::Off;
    if (mMode == Mode::Off)
        disable();
}

void JointDrive::setServoTarget(f32 targetPosition, f32 targetVelocity)
{
    if (mMode != Mode::Position || !finite(targetPosition) || !finite(targetVelocity))
        return;
    if (targetPosition != mTargetPosition || targetVelocity != mTargetVelocity)
        mTargetChanged = true;
    mTargetPosition = targetPosition;
    mTargetVelocity = targetVelocity;
}

void JointDrive::disable()
{
    mMode = Mode::Off;
    mActive = false;
    mTotalImpulse = 0.0f;
    mDesiredVelocity = 0.0f;
    mError = 0.0f;
}

void JointDrive::setForceRange(f32 minForce, f32 maxForce)
{
    if (!finite(minForce) || !finite(maxForce))
        return;
    mRangeMin = Math::min(minForce, 0.0f);
    mRangeMax = Math::max(maxForce, 0.0f);
    mHasForceRange = true;
}

void JointDrive::clearForceRange()
{
    mHasForceRange = false;
}

void JointDrive::setTorqueSpeedLine(f32 stallForce, f32 noLoadSpeed)
{
    if (!finite(stallForce) || !finite(noLoadSpeed))
        return;
    mStallForce = Math::max(stallForce, 0.0f);
    mNoLoadSpeed = noLoadSpeed > 0.0f ? noLoadSpeed : 0.0f;
}

f32 JointDrive::appliedForce() const
{
    if (mMode == Mode::Off || !(mDuration > 0.0f))
        return 0.0f;
    return mTotalImpulse / mDuration;
}

bool JointDrive::saturated() const
{
    return mActive && atBound();
}

bool JointDrive::atBound() const
{
    if (mMaxImpulse == 0.0f && mMinImpulse == 0.0f)
        return false;
    const f32 tolerance = 1.0e-5f * Math::max(fabsf(mMinImpulse), fabsf(mMaxImpulse));
    return mTotalImpulse >= mMaxImpulse - tolerance || mTotalImpulse <= mMinImpulse + tolerance;
}

void JointDrive::resetWarmStart()
{
    mTotalImpulse = 0.0f;
    mPreviousDuration = 0.0f;
}

bool JointDrive::prepare(f32 duration, f32 inverseEffectiveMass, const Coordinate& coordinate)
{
    mPosition = coordinate.position;
    mVelocity = coordinate.velocity;
    mWantsWake = false;
    mError = 0.0f;
    mPositionCorrectable = false;
    if (mMode == Mode::Off || !(duration > 0.0f) || !finite(duration) ||
        !(inverseEffectiveMass > 1.0e-9f) || !finite(inverseEffectiveMass))
    {
        mActive = false;
        mTotalImpulse = 0.0f;
        mDesiredVelocity = 0.0f;
        mPreviousDuration = 0.0f;
        mDuration = duration;
        mWantsWake = mMode != Mode::Off && mTargetChanged;
        mTargetChanged = false;
        return false;
    }

    // Last step's answer rescaled for a different step length (the impulse that
    // held a load over 1/60 s holds it over 1/120 s at half).
    if (mPreviousDuration > 0.0f)
        mTotalImpulse *= duration / mPreviousDuration;
    else
        mTotalImpulse = 0.0f;
    mPreviousDuration = duration;
    mDuration = duration;
    mCoordinate = coordinate;
    mPreparedTargetChanged = mTargetChanged;
    mTargetChanged = false;
    configure(inverseEffectiveMass, coordinate.gainInverseMass);
    return true;
}

bool JointDrive::setInverseMass(f32 inverseEffectiveMass, f32 gainInverseMass)
{
    if (!mActive || !(inverseEffectiveMass > 1.0e-9f) || !finite(inverseEffectiveMass))
        return false;
    configure(inverseEffectiveMass, gainInverseMass);
    return true;
}

// Everything about the row that depends on the inertia it moves: force range
// in impulses, gains, softness and effective mass. Run by prepare() and again
// by setInverseMass().
void JointDrive::configure(f32 inverseEffectiveMass, f32 gainInverseMass)
{
    const Coordinate& coordinate = mCoordinate;
    const f32 duration = mDuration;
    mWantsWake = false;
    mError = 0.0f;
    mPositionCorrectable = false;
    const f32 mass = 1.0f / inverseEffectiveMass;

    // The force range: the rated limit, narrowed by the actuator's model.
    f32 low = -mMaxForce;
    f32 high = mMaxForce;
    if (mHasForceRange)
    {
        low = Math::max(low, mRangeMin);
        high = Math::min(high, mRangeMax);
    }
    if (mNoLoadSpeed > 0.0f && mStallForce > 0.0f)
    {
        // DC motor at full voltage either way: T = Ts * (u - w / w0), u = +-1,
        // capped by the stall limit, read from this step's speed (the row then
        // decides how much of the envelope it needs).
        const f32 ratio = coordinate.velocity / mNoLoadSpeed;
        const f32 lineHigh = Math::min(mStallForce * (1.0f - ratio), mStallForce);
        const f32 lineLow = Math::max(-mStallForce * (1.0f + ratio), -mStallForce);
        high = Math::min(high, lineHigh);
        low = Math::max(low, lineLow);
    }
    if (high < low)
    {
        // Overspeed beyond the envelope: the motor can only brake at its
        // current limit, the bound nearer zero.
        if (coordinate.velocity > 0.0f)
            high = low;
        else
            low = high;
    }
    mMinImpulse = low * duration;
    mMaxImpulse = high * duration;
    mTotalImpulse = Math::clamp(mTotalImpulse, mMinImpulse, mMaxImpulse);

    mSoftness = 0.0f;
    mStiffness = 0.0f;
    mPositionCorrectable = false;
    if (mMode == Mode::Velocity)
    {
        mDesiredVelocity = mTargetVelocity;
        if (mVelocityDamping > 0.0f)
        {
            // F = c * (target - v_end), the damper solved at the end of the step:
            // v_end - target + lambda / (c * h) = 0, so softness = 1 / (c * h).
            mSoftness = 1.0f / (mVelocityDamping * duration);
        }
        mWantsWake = mPreparedTargetChanged || mTargetVelocity != 0.0f;
    }
    else
    {
        f32 target = mTargetPosition;
        if (coordinate.hasLimits)
            target = Math::clamp(target, coordinate.minPosition, coordinate.maxPosition);
        // An angle wraps: the error goes the short way round. With limits the way
        // inside them is the only one, unless already out, where the nearest way
        // back is what the limit row takes.
        const f32 error = coordinateError(coordinate, target);
        mError = error;
        mEndTarget = target + mTargetVelocity * duration;
        if (coordinate.hasLimits)
            mEndTarget = Math::clamp(mEndTarget, coordinate.minPosition, coordinate.maxPosition);

        f32 stiffness = 0.0f;
        f32 damping = 0.0f;
        if (mServo.frequency > 0.0f)
        {
            const f32 gainMass = gainInverseMass > 0.0f && finite(gainInverseMass)
                                     ? 1.0f / gainInverseMass
                                     : mass;
            const f32 omega = Math::two_pi<f32>() * mServo.frequency;
            stiffness = gainMass * omega * omega;
            damping = 2.0f * gainMass * mServo.dampingRatio * omega;
        }
        else if (mServo.stiffness > 0.0f)
        {
            stiffness = mServo.stiffness;
            damping = mServo.damping;
        }

        f32 biasVelocity = kRigidVelocityFraction * error / duration;
        if (stiffness > 0.0f)
        {
            // The implicit spring-damper (SoftSpring.h): F = -k*x_end - c*v_end
            // gives v_end + k*x/(c + h*k) + gamma*lambda = 0 with
            // gamma = 1 / (h*(c + h*k)).
            const f32 denominator = damping + duration * stiffness;
            mSoftness = 1.0f / (duration * denominator);
            biasVelocity = stiffness * error / denominator;
            mStiffness = stiffness;
        }
        mDesiredVelocity = mTargetVelocity - biasVelocity;
        mPositionCorrectable = !(stiffness > 0.0f);
        // The speed the soft row would end the step at holding last step's force:
        // v = desired - gamma * lambda. The softness term lets a lowered load run
        // ahead of the spring, so it counts.
        const f32 predicted = mDesiredVelocity - mSoftness * mTotalImpulse;
        if (mServo.maxVelocity > 0.0f && fabsf(predicted) > mServo.maxVelocity)
        {
            // Too far to close at the rated speed: slew at it as a stiff velocity
            // motor until the spring asks for less.
            mDesiredVelocity = Math::clamp(predicted, -mServo.maxVelocity, mServo.maxVelocity);
            mSoftness = 0.0f;
            mPositionCorrectable = false;
        }

        // Asleep is fine where the servo is in equilibrium with what it holds:
        // e + F/k is the distance to it (the whole error for a rigid servo). A
        // stalled, saturated servo is in equilibrium too.
        const f32 heldForce = mTotalImpulse / duration;
        const f32 toEquilibrium = mStiffness > 0.0f ? error + heldForce / mStiffness : error;
        mWantsWake = mPreparedTargetChanged || mTargetVelocity != 0.0f ||
                     (fabsf(toEquilibrium) > kWakeTolerance && !atBound());
    }

    mEffectiveMass = 1.0f / (inverseEffectiveMass + mSoftness);
    mInverseEffectiveMass = inverseEffectiveMass;
    mActive = true;
}

bool JointDrive::positionError(const Coordinate& coordinate, f32& error) const
{
    if (!mActive || !mPositionCorrectable || atBound())
        return false;
    error = coordinateError(coordinate, mEndTarget);
    return error != 0.0f && isfinite(error);
}

f32 JointDrive::solve(f32 velocity)
{
    if (!mActive)
        return 0.0f;
    const f32 impulse =
        -mEffectiveMass * (velocity - mDesiredVelocity + mSoftness * mTotalImpulse);
    const f32 previous = mTotalImpulse;
    mTotalImpulse = Math::clamp(previous + impulse, mMinImpulse, mMaxImpulse);
    const f32 applied = mTotalImpulse - previous;
    // The row's speed once this impulse is in (after the last iteration, the
    // speed the step ends with; setup() measured it before the step's impulses).
    mVelocity = velocity + applied * mInverseEffectiveMass;
    return applied;
}

} // namespace kinetix::Physics
