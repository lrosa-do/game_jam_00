#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/WheelJoint.h"

#include "JointAxis.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

Math::vec3 normalizedPerpendicular(const Math::vec3& v)
{
    if (fabsf(v.x) > fabsf(v.y))
    {
        const f32 length = sqrtf(v.x * v.x + v.z * v.z);
        return Math::vec3(v.z, 0.0f, -v.x) / length;
    }
    const f32 length = sqrtf(v.y * v.y + v.z * v.z);
    return Math::vec3(0.0f, v.z, -v.y) / length;
}

f32 rotationAngleAroundAxis(const Math::quat& q, const Math::vec3& axis)
{
    if (q.w == 0.0f)
        return Math::pi<f32>();
    return 2.0f * atanf(Math::dot(Math::vec3(q.x, q.y, q.z), axis) / q.w);
}

f32 centerAngleAroundZero(f32 angle)
{
    while (angle < -Math::pi<f32>())
        angle += Math::two_pi<f32>();
    while (angle > Math::pi<f32>())
        angle -= Math::two_pi<f32>();
    return angle;
}

} // namespace

WheelJoint::WheelJoint(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
                       const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis)
{
    configure(chassis, wheel, worldAnchor, worldSuspensionAxis, worldSpinAxis);
}

void WheelJoint::configure(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
                           const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis)
{
    const Math::vec3 suspension =
        detail::normalizedAxisOr(worldSuspensionAxis, Math::vec3(0.0f, -1.0f, 0.0f));
    mChassis = &chassis;
    mWheel = &wheel;
    mLocalAnchorChassis = chassis.pointToLocal(worldAnchor);
    mLocalAnchorWheel = wheel.pointToLocal(worldAnchor);
    mLocalSuspensionAxis = chassis.directionToLocal(suspension);
    mLocalSpinAxis = wheel.directionToLocal(
        detail::normalizedAxisOr(worldSpinAxis, Math::vec3(1.0f, 0.0f, 0.0f)));
    mLocalNormalAxis = chassis.directionToLocal(normalizedPerpendicular(suspension));
    mInverseInitialOrientation = Math::conjugate(wheel.orientation()) * chassis.orientation();
    const Math::vec3 spin =
        detail::normalizedAxisOr(worldSpinAxis, Math::vec3(1.0f, 0.0f, 0.0f));
    mLocalNeutralSpinAxis = chassis.directionToLocal(
        detail::normalizedAxisOr(spin - Math::dot(spin, suspension) * suspension,
                                 normalizedPerpendicular(suspension)));
}

RigidBody* WheelJoint::bodyA() const
{
    return mChassis;
}

RigidBody* WheelJoint::bodyB() const
{
    return mWheel;
}

Math::vec3 WheelJoint::anchorWorldA() const
{
    return mChassis->pointToWorld(mLocalAnchorChassis);
}

Math::vec3 WheelJoint::anchorWorldB() const
{
    return mWheel->pointToWorld(mLocalAnchorWheel);
}

Math::vec3 WheelJoint::axisWorld() const
{
    return mChassis->directionToWorld(mLocalSuspensionAxis);
}

void WheelJoint::setSteeringLimits(f32 minAngle, f32 maxAngle)
{
    mSteeringLimitsMin = Math::clamp(minAngle, -Math::pi<f32>(), Math::pi<f32>());
    mSteeringLimitsMax =
        Math::max(Math::clamp(maxAngle, -Math::pi<f32>(), Math::pi<f32>()), mSteeringLimitsMin);
    mHasSteeringLimits = mSteeringLimitsMin > -Math::pi<f32>() || mSteeringLimitsMax < Math::pi<f32>();
}

void WheelJoint::setSteeringMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    mSteeringDrive.setVelocity(targetAngularVelocity, maxTorque);
}

void WheelJoint::disableSteeringMotor()
{
    mSteeringDrive.disable();
}

void WheelJoint::setSteeringServo(f32 targetAngle, f32 maxTorque, f32 maxAngularVelocity)
{
    ServoSettings settings;
    settings.maxForce = maxTorque;
    settings.maxVelocity = maxAngularVelocity;
    mSteeringDrive.setServo(targetAngle, settings);
}

void WheelJoint::setSteeringServo(f32 targetAngle, const ServoSettings& settings)
{
    mSteeringDrive.setServo(targetAngle, settings);
}

void WheelJoint::disableSteeringServo()
{
    mSteeringDrive.disable();
}

f32 WheelJoint::steeringTorque() const
{
    return mSteeringDrive.appliedForce();
}

f32 WheelJoint::spinTorque() const
{
    return mSpinDrive.appliedForce();
}

f32 WheelJoint::steeringLimitTorque() const
{
    return mPreviousDuration > 0.0f ? mTotalSteeringLimitImpulse / mPreviousDuration : 0.0f;
}

f32 WheelJoint::suspensionForce() const
{
    return mPreviousDuration > 0.0f && mSuspensionEffectiveMass > 0.0f
               ? mTotalSuspensionImpulse / mPreviousDuration
               : 0.0f;
}

Math::vec3 WheelJoint::reactionForce() const
{
    if (!(mPreviousDuration > 0.0f))
        return Math::vec3(0.0f);
    return (mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y) / mPreviousDuration +
           mAxisA * suspensionForce();
}

Math::vec3 WheelJoint::reactionTorque() const
{
    if (!(mPreviousDuration > 0.0f))
        return Math::vec3(0.0f);
    return mPerpendicularAxis * (mTotalPerpendicularImpulse / mPreviousDuration) +
           mAxisA * (steeringTorque() + steeringLimitTorque()) + mAxisB * spinTorque();
}

f32 WheelJoint::steeringAngle() const
{
    return mSteeringAngle;
}

void WheelJoint::setSpinMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    mSpinDrive.setVelocity(targetAngularVelocity, maxTorque);
}

void WheelJoint::disableSpinMotor()
{
    mSpinDrive.disable();
}

f32 WheelJoint::spinAngle() const
{
    return mSpinAngleValue;
}

f32 WheelJoint::spinAngularVelocity() const
{
    return Math::dot(mAxisB, mWheel->angularVelocity() - mChassis->angularVelocity());
}

void WheelJoint::calculateArmsAndOffset()
{
    mArmA = mChassis->directionToWorld(mLocalAnchorChassis);
    mArmB = mWheel->directionToWorld(mLocalAnchorWheel);
    mOffset = (mWheel->position() - mChassis->position()) + mArmB - mArmA;
}

void WheelJoint::calculatePositionLockProperties()
{
    mAxisA = mChassis->directionToWorld(mLocalSuspensionAxis);
    mN1 = mChassis->directionToWorld(mLocalNormalAxis);
    mN2 = Math::cross(mAxisA, mN1);

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1x1 = Math::cross(armAPlusOffset, mN1);
    const Math::vec3 r1x2 = Math::cross(armAPlusOffset, mN2);
    const Math::vec3 r2x1 = Math::cross(mArmB, mN1);
    const Math::vec3 r2x2 = Math::cross(mArmB, mN2);

    const f32 inverseMassSum = mChassis->inverseMass() + mWheel->inverseMass();
    Math::mat2 inverseEffectiveMass(0.0f);
    inverseEffectiveMass[0][0] = inverseMassSum + Math::dot(r1x1, mChassis->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x1, mWheel->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[0][1] = Math::dot(r1x1, mChassis->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x1, mWheel->inverseInertiaTensorWorld() * r2x2);
    inverseEffectiveMass[1][0] = Math::dot(r1x2, mChassis->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x2, mWheel->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[1][1] = inverseMassSum + Math::dot(r1x2, mChassis->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x2, mWheel->inverseInertiaTensorWorld() * r2x2);

    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (detail::invertible(inverseEffectiveMass, determinant))
        mPositionLockEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mPositionLockEffectiveMass = Math::mat2(0.0f);
        mTotalPositionLockImpulse = Math::vec2(0.0f);
    }
}

void WheelJoint::calculatePerpendicularityProperties()
{
    mAxisA = Math::normalize(mChassis->directionToWorld(mLocalSuspensionAxis));
    mAxisB = Math::normalize(mWheel->directionToWorld(mLocalSpinAxis));

    const f32 k = Math::dot(mAxisA, mAxisB);
    const Math::vec3 axisBPerpendicular = mAxisB - k * mAxisA;
    const f32 length = Math::length(axisBPerpendicular);
    mPerpendicularAxis = length > 1.0e-6f ? Math::normalize(Math::cross(mAxisA, axisBPerpendicular))
                                          : normalizedPerpendicular(mAxisA);
    mPerpendicularity = k;

    const f32 inverseEffectiveMass = Math::dot(
        mPerpendicularAxis, mChassis->inverseInertiaTensorWorld() * mPerpendicularAxis +
                                mWheel->inverseInertiaTensorWorld() * mPerpendicularAxis);
    mPerpendicularEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mPerpendicularEffectiveMass == 0.0f)
        mTotalPerpendicularImpulse = 0.0f;
}

void WheelJoint::calculateAngles()
{
    mSlidePosition = Math::dot(mOffset, mAxisA);
    const Math::quat diff = mWheel->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mChassis->orientation());
    // The steering angle is where the axle points, not the wheel's whole
    // rotation: that mixes in the spin, and a quarter or half turn of spin makes
    // the swing-twist reading jump.
    const Math::vec3 neutral = Math::normalize(mChassis->directionToWorld(mLocalNeutralSpinAxis));
    const Math::vec3 projected = mAxisB - Math::dot(mAxisB, mAxisA) * mAxisA;
    if (Math::length(projected) > 1.0e-6f)
    {
        const Math::vec3 axle = Math::normalize(projected);
        mSteeringAngle = atan2f(Math::dot(Math::cross(neutral, axle), mAxisA), Math::dot(neutral, axle));
    }
    mSpinAngleValue = rotationAngleAroundAxis(diff, mAxisB);
}

void WheelJoint::calculateSteeringLimitProperties()
{
    mSteeringLimitActive =
        mHasSteeringLimits && (mSteeringAngle <= mSteeringLimitsMin || mSteeringAngle >= mSteeringLimitsMax);
    if (!mSteeringLimitActive)
    {
        mSteeringLimitEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        mAxisA, mChassis->inverseInertiaTensorWorld() * mAxisA + mWheel->inverseInertiaTensorWorld() * mAxisA);
    mSteeringLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mSteeringLimitEffectiveMass == 0.0f)
        mSteeringLimitActive = false;
}

void WheelJoint::calculateDriveProperties(f32 duration)
{
    // Steering, after calculateAngles() (which refreshes the angle the servo
    // measures its error from); clamped into the steering limits.
    if (mSteeringDrive.enabled())
    {
        JointDrive::Coordinate coordinate;
        coordinate.position = mSteeringAngle;
        coordinate.velocity = Math::dot(mAxisA, mWheel->angularVelocity() - mChassis->angularVelocity());
        coordinate.angular = true;
        coordinate.hasLimits = mHasSteeringLimits;
        coordinate.minPosition = mSteeringLimitsMin;
        coordinate.maxPosition = mSteeringLimitsMax;
        const f32 inverseEffectiveMass =
            Math::dot(mAxisA, mChassis->inverseInertiaTensorWorld() * mAxisA +
                                  mWheel->inverseInertiaTensorWorld() * mAxisA);
        mSteeringDrive.prepare(duration, inverseEffectiveMass, coordinate);
    }
    else
        mSteeringDrive.prepare(duration, 0.0f, JointDrive::Coordinate());

    if (mSpinDrive.enabled())
    {
        JointDrive::Coordinate coordinate;
        coordinate.position = mSpinAngleValue;
        coordinate.velocity = Math::dot(mAxisB, mWheel->angularVelocity() - mChassis->angularVelocity());
        coordinate.angular = true;
        const f32 inverseEffectiveMass =
            Math::dot(mAxisB, mChassis->inverseInertiaTensorWorld() * mAxisB +
                                  mWheel->inverseInertiaTensorWorld() * mAxisB);
        mSpinDrive.prepare(duration, inverseEffectiveMass, coordinate);
    }
    else
        mSpinDrive.prepare(duration, 0.0f, JointDrive::Coordinate());

    if (mSteeringDrive.wantsWake() || mSpinDrive.wantsWake())
        wakeBodies();
}

// A rigid steering servo's position error, closed with the joint's other rows
// (JointDrive::positionError).
void WheelJoint::solveDrivePosition(f32 baumgarte)
{
    if (!mSteeringDrive.active())
        return;
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculatePerpendicularityProperties();
    calculateAngles();
    JointDrive::Coordinate coordinate;
    coordinate.position = mSteeringAngle;
    coordinate.angular = true;
    coordinate.hasLimits = mHasSteeringLimits;
    coordinate.minPosition = mSteeringLimitsMin;
    coordinate.maxPosition = mSteeringLimitsMax;
    f32 error = 0.0f;
    if (!mSteeringDrive.positionError(coordinate, error))
        return;
    const f32 inverseMass = Math::dot(mAxisA, mChassis->inverseInertiaTensorWorld() * mAxisA +
                                                  mWheel->inverseInertiaTensorWorld() * mAxisA);
    if (inverseMass > 1.0e-9f)
        detail::applyAngularPositionImpulse(*mChassis, *mWheel, mAxisA * (-baumgarte * error / inverseMass));
}

void WheelJoint::applyLinearImpulse(const Math::vec3& impulse)
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    if (mChassis->isDynamic())
    {
        mChassis->setVelocity(mChassis->velocity() - impulse * mChassis->inverseMass());
        mChassis->setAngularVelocity(
            mChassis->angularVelocity() -
            mChassis->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, impulse));
    }
    if (mWheel->isDynamic())
    {
        mWheel->setVelocity(mWheel->velocity() + impulse * mWheel->inverseMass());
        mWheel->setAngularVelocity(
            mWheel->angularVelocity() +
            mWheel->inverseInertiaTensorWorld() * Math::cross(mArmB, impulse));
    }
}

void WheelJoint::applyAngularImpulse(const Math::vec3& impulse)
{
    if (mChassis->isDynamic())
        mChassis->setAngularVelocity(mChassis->angularVelocity() -
                                     mChassis->inverseInertiaTensorWorld() * impulse);
    if (mWheel->isDynamic())
        mWheel->setAngularVelocity(mWheel->angularVelocity() +
                                   mWheel->inverseInertiaTensorWorld() * impulse);
}

// F = -k*x - c*v, applied once per step like a real spring: not clamped or warm
// started, recomputed from the current travel and closing speed every setup().
void WheelJoint::calculateSuspensionProperties(f32 duration)
{
    if (mSuspensionStiffness <= 0.0f && mSuspensionDamping <= 0.0f)
    {
        mSuspensionEffectiveMass = 0.0f;
        mTotalSuspensionImpulse = 0.0f;
        return;
    }

    // The row is the suspension axis itself, with the arms the two perpendicular
    // rows use (calculatePositionLockProperties()).
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1 = Math::cross(armAPlusOffset, mAxisA);
    const Math::vec3 r2 = Math::cross(mArmB, mAxisA);
    const f32 inverseEffectiveMass = mChassis->inverseMass() + mWheel->inverseMass() +
                                     Math::dot(r1, mChassis->inverseInertiaTensorWorld() * r1) +
                                     Math::dot(r2, mWheel->inverseInertiaTensorWorld() * r2);
    if (inverseEffectiveMass <= 1.0e-9f)
    {
        mSuspensionEffectiveMass = 0.0f;
        mTotalSuspensionImpulse = 0.0f;
        return;
    }

    // C is how far the strut sits from where the spring wants it. Compressed
    // (slide below rest) is negative and solveVelocity subtracts the bias, so a
    // compressed strut pushes the wheel along mAxisA, chassis towards ground.
    const f32 positionError = mSlidePosition - mSuspensionRestLength;
    mSuspensionSpring.calculate(duration, inverseEffectiveMass, 0.0f, positionError,
                                mSuspensionStiffness, mSuspensionDamping, mSuspensionEffectiveMass);
}

void WheelJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculatePerpendicularityProperties();
    calculateAngles();
    calculateSteeringLimitProperties();
    calculateDriveProperties(duration);
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionLockImpulse *= ratio;
        mTotalPerpendicularImpulse *= ratio;
        mTotalSteeringLimitImpulse *= ratio;
    }
    else
    {
        mTotalPositionLockImpulse = Math::vec2(0.0f);
        mTotalPerpendicularImpulse = 0.0f;
        mTotalSteeringLimitImpulse = 0.0f;
    }
    // A steering stop not in contact this step starts from nothing next time
    // instead of warm starting its last impulse. Decided here only: the position
    // pass re-measures the stop too, and an in-range reading does not mean the
    // contact is gone.
    if (!mSteeringLimitActive)
        mTotalSteeringLimitImpulse = 0.0f;
    mPreviousDuration = duration;

    calculateSuspensionProperties(duration);
}

void WheelJoint::warmStart()
{
    if (mSteeringDrive.active())
        applyAngularImpulse(mAxisA * mSteeringDrive.totalImpulse());
    if (mSpinDrive.active())
        applyAngularImpulse(mAxisB * mSpinDrive.totalImpulse());
    applyLinearImpulse(mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y);
    if (mSuspensionEffectiveMass > 0.0f)
        applyLinearImpulse(mAxisA * mTotalSuspensionImpulse);
    applyAngularImpulse(mPerpendicularAxis * mTotalPerpendicularImpulse);
    if (mSteeringLimitActive)
        applyAngularImpulse(mAxisA * mTotalSteeringLimitImpulse);
}

void WheelJoint::solveVelocity()
{
    if (mSteeringDrive.active())
    {
        const f32 velocity = Math::dot(mAxisA, mWheel->angularVelocity() - mChassis->angularVelocity());
        applyAngularImpulse(mAxisA * mSteeringDrive.solve(velocity));
    }

    if (mSpinDrive.active())
    {
        const f32 velocity = Math::dot(mAxisB, mWheel->angularVelocity() - mChassis->angularVelocity());
        applyAngularImpulse(mAxisB * mSpinDrive.solve(velocity));
    }

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 deltaLinear = mChassis->velocity() - mWheel->velocity();
    Math::vec2 positionJv;
    positionJv.x = Math::dot(mN1, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN1), mChassis->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN1), mWheel->angularVelocity());
    positionJv.y = Math::dot(mN2, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN2), mChassis->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN2), mWheel->angularVelocity());
    const Math::vec2 positionImpulse = mPositionLockEffectiveMass * positionJv;
    mTotalPositionLockImpulse += positionImpulse;
    applyLinearImpulse(mN1 * positionImpulse.x + mN2 * positionImpulse.y);

    // The suspension is the third row of the same position constraint, along the
    // axis, left soft instead of locked.
    if (mSuspensionEffectiveMass > 0.0f)
    {
        const f32 suspensionJv =
            Math::dot(mAxisA, deltaLinear) +
            Math::dot(Math::cross(armAPlusOffset, mAxisA), mChassis->angularVelocity()) -
            Math::dot(Math::cross(mArmB, mAxisA), mWheel->angularVelocity());
        // The rows above solve lambda = +mass * Jv with Jv measured chassis -
        // wheel; the spring's bias is written for lambda = -mass * (Jv + bias),
        // so here it subtracts.
        const f32 impulse = mSuspensionEffectiveMass *
                            (suspensionJv - mSuspensionSpring.bias(mTotalSuspensionImpulse));
        mTotalSuspensionImpulse += impulse;
        applyLinearImpulse(mAxisA * impulse);
    }

    const f32 perpJv = Math::dot(mPerpendicularAxis, mChassis->angularVelocity() - mWheel->angularVelocity());
    const f32 perpImpulse = mPerpendicularEffectiveMass * perpJv;
    mTotalPerpendicularImpulse += perpImpulse;
    applyAngularImpulse(mPerpendicularAxis * perpImpulse);

    if (mSteeringLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mSteeringLimitsMin != mSteeringLimitsMax)
        {
            const f32 distanceToMin = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMax);
            if (fabsf(distanceToMin) < fabsf(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mAxisA, mChassis->angularVelocity() - mWheel->angularVelocity());
        const f32 impulse = mSteeringLimitEffectiveMass * relative;
        const f32 previous = mTotalSteeringLimitImpulse;
        mTotalSteeringLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisA * (mTotalSteeringLimitImpulse - previous));
    }
}

void WheelJoint::solvePosition(f32 baumgarte)
{
    solveDrivePosition(baumgarte);

    calculateArmsAndOffset();
    calculatePositionLockProperties();
    const Math::vec2 c(Math::dot(mOffset, mN1), Math::dot(mOffset, mN2));
    if (c != Math::vec2(0.0f))
    {
        const Math::vec2 lambda = -baumgarte * (mPositionLockEffectiveMass * c);
        const Math::vec3 impulse = mN1 * lambda.x + mN2 * lambda.y;
        const Math::vec3 armAPlusOffset = mArmA + mOffset;
        mChassis->applyPositionImpulseAtPoint(-impulse, mChassis->position() + armAPlusOffset);
        mWheel->applyPositionImpulseAtPoint(impulse, mWheel->position() + mArmB);
    }

    calculatePerpendicularityProperties();
    if (mPerpendicularity != 0.0f)
    {
        // Rotating the wheel by +d about mPerpendicularAxis lowers the dot of the
        // two axes by d, so the correction must be +k: with -k the error grows by
        // (1 + baumgarte) per iteration and a wheel not turning about a world axis
        // ends up with its spin axis along the suspension.
        const f32 lambda = mPerpendicularEffectiveMass * baumgarte * mPerpendicularity;
        if (mChassis->isDynamic())
        {
            const Math::vec3 step = mChassis->inverseInertiaTensorWorld() * mPerpendicularAxis * -lambda;
            const Math::quat spin(0.0f, step);
            mChassis->setOrientation(mChassis->orientation() + 0.5f * spin * mChassis->orientation());
        }
        if (mWheel->isDynamic())
        {
            const Math::vec3 step = mWheel->inverseInertiaTensorWorld() * mPerpendicularAxis * lambda;
            const Math::quat spin(0.0f, step);
            mWheel->setOrientation(mWheel->orientation() + 0.5f * spin * mWheel->orientation());
        }
    }

    if (mHasSteeringLimits)
    {
        calculateArmsAndOffset();
        calculateAngles();
        calculateSteeringLimitProperties();
        if (mSteeringLimitActive)
        {
            const f32 distanceToMin = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMax);
            const f32 error = fabsf(distanceToMin) < fabsf(distanceToMax) ? distanceToMin : distanceToMax;
            const f32 lambda = -mSteeringLimitEffectiveMass * baumgarte * error;
            if (mChassis->isDynamic())
            {
                const Math::vec3 step = mChassis->inverseInertiaTensorWorld() * mAxisA * -lambda;
                const Math::quat spin(0.0f, step);
                mChassis->setOrientation(mChassis->orientation() + 0.5f * spin * mChassis->orientation());
            }
            if (mWheel->isDynamic())
            {
                const Math::vec3 step = mWheel->inverseInertiaTensorWorld() * mAxisA * lambda;
                const Math::quat spin(0.0f, step);
                mWheel->setOrientation(mWheel->orientation() + 0.5f * spin * mWheel->orientation());
            }
        }
    }
}

} // namespace kinetix::Physics
