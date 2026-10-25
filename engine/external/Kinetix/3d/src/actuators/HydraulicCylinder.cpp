#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/HydraulicCylinder.h"

#include "../dynamics/JointAxis.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kLitresPerMinute = 1.0f / 60000.0f; // m^3/s per L/min

f32 circleArea(f32 diameter)
{
    return 0.25f * Math::pi<f32>() * diameter * diameter;
}

HydraulicCylinderParameters sanitized(const HydraulicCylinderParameters& in)
{
    HydraulicCylinderParameters p = in;
    p.boreDiameter = isfinite(p.boreDiameter) ? Math::max(p.boreDiameter, 1.0e-4f) : 0.1f;
    p.rodDiameter = isfinite(p.rodDiameter) ? Math::clamp(p.rodDiameter, 0.0f, 0.99f * p.boreDiameter)
                                            : 0.5f * p.boreDiameter;
    p.stroke = isfinite(p.stroke) ? Math::max(p.stroke, 1.0e-3f) : 1.0f;
    p.initialStroke = isfinite(p.initialStroke) ? Math::clamp(p.initialStroke, 0.0f, p.stroke) : 0.0f;
    p.count = Math::max(p.count, 1u);
    p.pumpFlow = isfinite(p.pumpFlow) ? Math::max(p.pumpFlow, 0.0f) : 0.0f;
    p.reliefPressure = isfinite(p.reliefPressure) ? Math::max(p.reliefPressure, 0.0f) : 0.0f;
    p.cushionLength = isfinite(p.cushionLength) ? Math::clamp(p.cushionLength, 0.0f, 0.5f * p.stroke) : 0.0f;
    p.cushionSpeedFraction =
        isfinite(p.cushionSpeedFraction) ? Math::clamp(p.cushionSpeedFraction, 0.0f, 1.0f) : 1.0f;
    p.valveDeadband = isfinite(p.valveDeadband) ? Math::clamp(p.valveDeadband, 0.0f, 0.5f) : 0.0f;
    p.valveResponseTime = isfinite(p.valveResponseTime) ? Math::max(p.valveResponseTime, 0.0f) : 0.0f;
    p.bulkModulus = isfinite(p.bulkModulus) ? Math::max(p.bulkModulus, 0.0f) : 0.0f;
    p.deadVolume = isfinite(p.deadVolume) ? Math::max(p.deadVolume, 1.0e-7f) : 5.0e-4f;
    p.compressibilityDamping =
        isfinite(p.compressibilityDamping) ? Math::max(p.compressibilityDamping, 0.0f) : 0.7f;
    return p;
}

}

HydraulicCylinder::HydraulicCylinder(RigidBody& barrelBody, const Math::vec3& worldBarrelPin,
                                     RigidBody& rodBody, const Math::vec3& worldRodPin,
                                     const HydraulicCylinderParameters& parameters)
    : mBarrel(&barrelBody), mRod(&rodBody), mLocalBarrelPin(barrelBody.pointToLocal(worldBarrelPin)),
      mLocalRodPin(rodBody.pointToLocal(worldRodPin)), mParameters(sanitized(parameters))
{
    const f32 count = static_cast<f32>(mParameters.count);
    mCapArea = count * circleArea(mParameters.boreDiameter);
    mAnnulusArea = mCapArea - count * circleArea(mParameters.rodDiameter);
    const f32 distance = Math::length(worldRodPin - worldBarrelPin);
    if (!(mParameters.closedLength > 0.0f) || !isfinite(mParameters.closedLength))
        mParameters.closedLength = Math::max(distance - mParameters.initialStroke, 1.0e-3f);
    mOilStroke = Math::clamp(distance - mParameters.closedLength, 0.0f, mParameters.stroke);
    calculateGeometry();
}

RigidBody* HydraulicCylinder::bodyA() const
{
    return mBarrel;
}

RigidBody* HydraulicCylinder::bodyB() const
{
    return mRod;
}

void HydraulicCylinder::setValve(f32 command)
{
    if (!isfinite(command))
        return;
    mValve = Math::clamp(command, -1.0f, 1.0f);
    mFlowControlled = false;
}

void HydraulicCylinder::setFlow(f32 litresPerMinute)
{
    if (!isfinite(litresPerMinute))
        return;
    mFlowCommand = litresPerMinute;
    mFlowControlled = true;
}

void HydraulicCylinder::setPressureLimit(f32 pascals)
{
    mPressureLimit = isfinite(pascals) && pascals > 0.0f ? pascals : 0.0f;
}

f32 HydraulicCylinder::effectivePressureLimit() const
{
    return mPressureLimit > 0.0f ? Math::min(mPressureLimit, mParameters.reliefPressure)
                                 : mParameters.reliefPressure;
}

f32 HydraulicCylinder::maxExtendSpeed() const
{
    return mParameters.pumpFlow * kLitresPerMinute / mCapArea;
}

f32 HydraulicCylinder::maxRetractSpeed() const
{
    return mParameters.pumpFlow * kLitresPerMinute / mAnnulusArea;
}

f32 HydraulicCylinder::maxPushForce() const
{
    return mParameters.reliefPressure * mCapArea;
}

f32 HydraulicCylinder::maxPullForce() const
{
    return mParameters.reliefPressure * mAnnulusArea;
}

Math::vec3 HydraulicCylinder::barrelPinWorld() const
{
    return mBarrel->pointToWorld(mLocalBarrelPin);
}

Math::vec3 HydraulicCylinder::rodPinWorld() const
{
    return mRod->pointToWorld(mLocalRodPin);
}

f32 HydraulicCylinder::length() const
{
    return Math::length(mRod->pointToWorld(mLocalRodPin) - mBarrel->pointToWorld(mLocalBarrelPin));
}

f32 HydraulicCylinder::stroke() const
{
    return length() - mParameters.closedLength;
}

f32 HydraulicCylinder::strokeVelocity() const
{
    const Math::vec3 pinA = mBarrel->pointToWorld(mLocalBarrelPin);
    const Math::vec3 pinB = mRod->pointToWorld(mLocalRodPin);
    const Math::vec3 delta = pinB - pinA;
    const f32 distance = Math::length(delta);
    if (!(distance > 1.0e-6f))
        return 0.0f;
    return Math::dot(delta / distance, mRod->velocityAtPoint(pinB) - mBarrel->velocityAtPoint(pinA));
}

f32 HydraulicCylinder::force() const
{
    return mDrive.appliedForce();
}

f32 HydraulicCylinder::capPressure() const
{
    const f32 f = force();
    return f > 0.0f ? f / mCapArea : 0.0f;
}

f32 HydraulicCylinder::rodPressure() const
{
    const f32 f = force();
    return f < 0.0f ? -f / mAnnulusArea : 0.0f;
}

f32 HydraulicCylinder::flow() const
{
    const f32 v = strokeVelocity();
    return (v > 0.0f ? mCapArea * v : -mAnnulusArea * v) / kLitresPerMinute;
}

bool HydraulicCylinder::reliefOpen() const
{
    return mDrive.saturated();
}

f32 HydraulicCylinder::endStopForce() const
{
    return mPreviousDuration > 0.0f ? mTotalStopImpulse / mPreviousDuration : 0.0f;
}

bool HydraulicCylinder::atStrokeEnd(f32 direction) const
{
    // 1 mm tolerance: the piston is within the position pass's reach.
    if (direction > 0.0f)
        return mOilStroke >= mParameters.stroke - 1.0e-3f;
    if (direction < 0.0f)
        return mOilStroke <= 1.0e-3f;
    return false;
}

Math::vec3 HydraulicCylinder::reactionForce() const
{
    return mAxis * (force() + endStopForce());
}

void HydraulicCylinder::calculateGeometry()
{
    const Math::vec3 pinA = mBarrel->pointToWorld(mLocalBarrelPin);
    const Math::vec3 pinB = mRod->pointToWorld(mLocalRodPin);
    mArmA = pinA - mBarrel->position();
    mArmB = pinB - mRod->position();
    const Math::vec3 delta = pinB - pinA;
    mLength = Math::length(delta);
    // Two pins on top of each other have no line between them; the last one
    // known stands in until they part.
    if (mLength > 1.0e-6f && isfinite(mLength))
        mAxis = delta / mLength;
    const detail::JacobianRow row = detail::linearRow(mArmA, mArmB, mAxis);
    mInverseEffectiveMass = detail::rowProduct(*mBarrel, *mRod, row, row);
}

f32 HydraulicCylinder::rowVelocity() const
{
    const Math::vec3 velocityA = mBarrel->velocity() + Math::cross(mBarrel->angularVelocity(), mArmA);
    const Math::vec3 velocityB = mRod->velocity() + Math::cross(mRod->angularVelocity(), mArmB);
    return Math::dot(mAxis, velocityB - velocityA);
}

void HydraulicCylinder::applyImpulse(f32 impulse)
{
    const Math::vec3 linear = mAxis * impulse;
    if (mBarrel->isDynamic())
    {
        mBarrel->setVelocity(mBarrel->velocity() - linear * mBarrel->inverseMass());
        mBarrel->setAngularVelocity(mBarrel->angularVelocity() -
                                    mBarrel->inverseInertiaTensorWorld() * Math::cross(mArmA, linear));
    }
    if (mRod->isDynamic())
    {
        mRod->setVelocity(mRod->velocity() + linear * mRod->inverseMass());
        mRod->setAngularVelocity(mRod->angularVelocity() +
                                 mRod->inverseInertiaTensorWorld() * Math::cross(mArmB, linear));
    }
}

void HydraulicCylinder::setup(f32 duration)
{
    calculateGeometry();
    const f32 stroke = mLength - mParameters.closedLength;
    const f32 fullStroke = mParameters.stroke;

    // The relief was open last step: the column now holds the piston where
    // the load pushed it.
    if (mDrive.saturated())
        mOilStroke = Math::clamp(stroke, 0.0f, fullStroke);

    f32 flow = 0.0f;
    if (mFlowControlled)
    {
        // A shared circuit's section owns the spool, lag and deadband.
        flow = mFlowCommand * kLitresPerMinute;
        mSpool = mParameters.pumpFlow > 0.0f ? Math::clamp(mFlowCommand / mParameters.pumpFlow, -1.0f, 1.0f)
                                              : 0.0f;
    }
    else
    {
        if (mParameters.valveResponseTime > 0.0f)
            mSpool += (mValve - mSpool) * (1.0f - expf(-duration / mParameters.valveResponseTime));
        else
            mSpool = mValve;

        f32 command = 0.0f;
        const f32 deadband = mParameters.valveDeadband;
        if (fabsf(mSpool) > deadband)
            command = (mSpool > 0.0f ? 1.0f : -1.0f) * (fabsf(mSpool) - deadband) / (1.0f - deadband);
        flow = mParameters.pumpFlow * kLitresPerMinute * command;
    }
    f32 speed = flow > 0.0f ? flow / mCapArea : flow / mAnnulusArea;

    if (mParameters.cushionLength > 0.0f && speed != 0.0f)
    {
        const f32 remaining = speed > 0.0f ? fullStroke - stroke : stroke;
        const f32 fraction = Math::clamp(remaining / mParameters.cushionLength,
                                         mParameters.cushionSpeedFraction, 1.0f);
        speed *= fraction;
    }
    if ((speed > 0.0f && mOilStroke >= fullStroke) || (speed < 0.0f && mOilStroke <= 0.0f))
        speed = 0.0f;
    mCommandedSpeed = speed;

    const f32 pressure = effectivePressureLimit();
    const f32 pushLimit = pressure * mCapArea;
    const f32 pullLimit = pressure * mAnnulusArea;
    const f32 maxForce = Math::max(pushLimit, pullLimit);
    JointDrive::Coordinate coordinate;
    coordinate.position = stroke;
    coordinate.velocity = rowVelocity();
    coordinate.hasLimits = true;
    coordinate.minPosition = 0.0f;
    coordinate.maxPosition = fullStroke;
    mCompressible = mParameters.bulkModulus > 0.0f && mInverseEffectiveMass > 0.0f;
    if (mCompressible)
    {
        // The oil spring k = beta * A^2 / V of each chamber, in parallel, with
        // the hoses' dead volume on either side.
        const f32 capVolume = mCapArea * Math::clamp(stroke, 0.0f, fullStroke) + mParameters.deadVolume;
        const f32 annulusVolume =
            mAnnulusArea * Math::clamp(fullStroke - stroke, 0.0f, fullStroke) + mParameters.deadVolume;
        const f32 stiffness = mParameters.bulkModulus * (mCapArea * mCapArea / capVolume +
                                                         mAnnulusArea * mAnnulusArea / annulusVolume);
        ServoSettings settings;
        settings.maxForce = maxForce;
        settings.stiffness = stiffness;
        settings.damping =
            2.0f * mParameters.compressibilityDamping * sqrtf(stiffness / mInverseEffectiveMass);
        mDrive.setServo(mOilStroke, settings);
        mDrive.setServoTarget(mOilStroke, speed);
    }
    else
    {
        // A hard row: the position pass closes the error like the hinges it
        // works with; correcting it at velocity level in one step fights their
        // position pass in a chain of cylinders and chatters between the reliefs.
        mDrive.setVelocity(speed, maxForce);
    }
    mDrive.setForceRange(-pullLimit, pushLimit);
    mDrive.prepare(duration, mInverseEffectiveMass, coordinate);
    if (mDrive.wantsWake() || (!mCompressible && fabsf(stroke - mOilStroke) > 1.0e-3f))
        wakeBodies();

    mOilStroke = Math::clamp(mOilStroke + speed * duration, 0.0f, fullStroke);

    mStopActive = (stroke <= 0.0f || stroke >= fullStroke) && mInverseEffectiveMass > 1.0e-9f;
    mStopAtClosed = stroke <= 0.5f * fullStroke;
    mStopEffectiveMass = mStopActive ? 1.0f / mInverseEffectiveMass : 0.0f;
    if (mStopActive && mPreviousDuration > 0.0f)
        mTotalStopImpulse *= duration / mPreviousDuration;
    else
        mTotalStopImpulse = 0.0f;
    mPreviousDuration = duration;
}

void HydraulicCylinder::warmStart()
{
    if (mDrive.active())
        applyImpulse(mDrive.totalImpulse());
    if (mStopActive)
        applyImpulse(mTotalStopImpulse);
}

void HydraulicCylinder::solveVelocity()
{
    if (mDrive.active())
        applyImpulse(mDrive.solve(rowVelocity()));

    if (mStopActive)
    {
        // At the closed stop the stop can only push the pins apart; at the
        // open one only hold them together.
        const f32 impulse = -mStopEffectiveMass * rowVelocity();
        const f32 previous = mTotalStopImpulse;
        mTotalStopImpulse = mStopAtClosed ? Math::max(previous + impulse, 0.0f)
                                          : Math::min(previous + impulse, 0.0f);
        applyImpulse(mTotalStopImpulse - previous);
    }
}

void HydraulicCylinder::solvePosition(f32 baumgarte)
{
    calculateGeometry();
    const f32 stroke = mLength - mParameters.closedLength;
    f32 error = 0.0f;
    if (stroke < 0.0f)
        error = stroke;
    else if (stroke > mParameters.stroke)
        error = stroke - mParameters.stroke;
    else if (!mCompressible && mDrive.active() && !mDrive.saturated())
        // The rigid column, unless the relief is open - then the oil follows
        // the piston instead (see setup()).
        error = stroke - mOilStroke;
    if (error == 0.0f || !(mInverseEffectiveMass > 1.0e-9f))
        return;
    const Math::vec3 impulse = mAxis * (-baumgarte * error / mInverseEffectiveMass);
    mBarrel->applyPositionImpulseAtPoint(-impulse, mBarrel->position() + mArmA);
    mRod->applyPositionImpulseAtPoint(impulse, mRod->position() + mArmB);
}

u32 HydraulicCylinder::articulationMotors(ArticulationMotor* motors)
{
    u32 count = 0;
    const detail::JacobianRow row = detail::linearRow(mArmA, mArmB, mAxis);
    if (mDrive.active())
    {
        motors[count].row = row;
        motors[count].id = 0;
        motors[count].wantsGain =
            mDrive.mode() == JointDrive::Mode::Position && mDrive.servoSettings().frequency > 0.0f;
        ++count;
    }
    if (mStopActive)
    {
        motors[count].row = row;
        motors[count].id = 1;
        ++count;
    }
    return count;
}

void HydraulicCylinder::setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass)
{
    if (id == 0)
        mDrive.setInverseMass(inverseMass, gainInverseMass);
    else if (inverseMass > 1.0e-9f)
        mStopEffectiveMass = 1.0f / inverseMass;
}

f32 HydraulicCylinder::solveArticulationMotor(u32 id, f32 velocity)
{
    if (id == 0)
        return mDrive.solve(velocity);
    const f32 previous = mTotalStopImpulse;
    const f32 impulse = -mStopEffectiveMass * velocity;
    mTotalStopImpulse = mStopAtClosed ? Math::max(previous + impulse, 0.0f) : Math::min(previous + impulse, 0.0f);
    return mTotalStopImpulse - previous;
}

// The end stops when the stroke is beyond them, and otherwise the rigid oil
// column (unless the relief is open) - what solvePosition() corrects.
u32 HydraulicCylinder::articulationPositionRows(JointRow* rows, f32* errors)
{
    calculateGeometry();
    const f32 stroke = mLength - mParameters.closedLength;
    f32 error = 0.0f;
    if (stroke < 0.0f)
        error = stroke;
    else if (stroke > mParameters.stroke)
        error = stroke - mParameters.stroke;
    else if (!mCompressible && mDrive.active() && !mDrive.saturated())
        error = stroke - mOilStroke;
    if (error == 0.0f || !(mInverseEffectiveMass > 1.0e-9f))
        return 0;
    rows[0] = detail::linearRow(mArmA, mArmB, mAxis);
    errors[0] = error;
    return 1;
}

} // namespace kinetix::Physics
