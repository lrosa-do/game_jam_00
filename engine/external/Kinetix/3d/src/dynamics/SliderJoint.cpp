#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/SliderJoint.h"

#include "JointAxis.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

Math::quat invInitialOrientationXY(const Math::vec3& xAxisA, const Math::vec3& yAxisA,
                                  const Math::vec3& xAxisB, const Math::vec3& yAxisB)
{
    if (xAxisA == xAxisB && yAxisA == yAxisB)
        return Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const Math::mat3 basisA(xAxisA, yAxisA, Math::cross(xAxisA, yAxisA));
    const Math::mat3 basisB(xAxisB, yAxisB, Math::cross(xAxisB, yAxisB));
    return Math::quat_cast(basisB) * Math::conjugate(Math::quat_cast(basisA));
}

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

Math::vec3 sliderAxisOrDefault(const Math::vec3& axis)
{
    return detail::normalizedAxisOr(axis, Math::vec3(1.0f, 0.0f, 0.0f));
}

// The normal axis must be a unit vector perpendicular to the (unit) slider axis;
// it and its cross product with the slider axis are the two directions the
// position lock holds. A zero or parallel one gets the fallback, a slightly off
// one is straightened.
Math::vec3 normalAxisOr(const Math::vec3& normal, const Math::vec3& sliderAxis,
                        const Math::vec3& fallback)
{
    const Math::vec3 unit = detail::normalizedAxisOr(normal, fallback);
    const Math::vec3 perpendicular = unit - Math::dot(unit, sliderAxis) * sliderAxis;
    return detail::normalizedAxisOr(perpendicular, normalizedPerpendicular(sliderAxis));
}

// B's frame only feeds the rest orientation. A missing axis there falls back to
// A's, so the pose at construction is the rest pose.
Math::quat invInitialOrientation(const RigidBody& a, const RigidBody& b,
                                 const Math::vec3& sliderAxisA, const Math::vec3& normalAxisA,
                                 const Math::vec3& localSliderAxisB,
                                 const Math::vec3& localNormalAxisB)
{
    const Math::vec3 sliderAxisB = detail::normalizedAxisOr(
        localSliderAxisB, b.directionToLocal(a.directionToWorld(sliderAxisA)));
    const Math::vec3 normalAxisB = normalAxisOr(
        localNormalAxisB, sliderAxisB, b.directionToLocal(a.directionToWorld(normalAxisA)));
    return invInitialOrientationXY(sliderAxisA, normalAxisA, sliderAxisB, normalAxisB);
}

}

SliderJoint::SliderJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                         const Math::vec3& worldSliderAxis)
    : SliderJoint(a, a.pointToLocal(worldAnchor),
                 a.directionToLocal(sliderAxisOrDefault(worldSliderAxis)),
                 a.directionToLocal(normalizedPerpendicular(sliderAxisOrDefault(worldSliderAxis))),
                 b, b.pointToLocal(worldAnchor),
                 b.directionToLocal(sliderAxisOrDefault(worldSliderAxis)),
                 b.directionToLocal(normalizedPerpendicular(sliderAxisOrDefault(worldSliderAxis))))
{
}

SliderJoint::SliderJoint(RigidBody& a, const Math::vec3& localAnchorA,
                         const Math::vec3& localSliderAxisA, const Math::vec3& localNormalAxisA,
                         RigidBody& b, const Math::vec3& localAnchorB,
                         const Math::vec3& localSliderAxisB, const Math::vec3& localNormalAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalSliderAxisA(sliderAxisOrDefault(localSliderAxisA)),
      mLocalNormalAxisA(normalAxisOr(localNormalAxisA, mLocalSliderAxisA,
                                     normalizedPerpendicular(mLocalSliderAxisA))),
      mLocalNormalAxisA2(Math::cross(mLocalSliderAxisA, mLocalNormalAxisA)),
      mInverseInitialOrientation(invInitialOrientation(a, b, mLocalSliderAxisA, mLocalNormalAxisA,
                                                       localSliderAxisB, localNormalAxisB))
{
}

RigidBody* SliderJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* SliderJoint::bodyB() const
{
    return mBodyB;
}

void SliderJoint::setLimits(f32 minDistance, f32 maxDistance)
{
    mLimitsMin = minDistance;
    mLimitsMax = maxDistance;
    mHasLimits = mLimitsMin != -M_INFINITY || mLimitsMax != M_INFINITY;
}

f32 SliderJoint::currentPosition() const
{
    const Math::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const Math::vec3 offset = (mBodyB->position() - mBodyA->position()) + armB - armA;
    return Math::dot(offset, mBodyA->directionToWorld(mLocalSliderAxisA));
}

f32 SliderJoint::currentVelocity() const
{
    const Math::vec3 axis = mBodyA->directionToWorld(mLocalSliderAxisA);
    const Math::vec3 pointB = mBodyB->pointToWorld(mLocalAnchorB);
    return Math::dot(axis, mBodyB->velocityAtPoint(pointB) - mBodyA->velocityAtPoint(pointB));
}

void SliderJoint::setMotor(f32 targetVelocity, f32 maxForce)
{
    mDrive.setVelocity(targetVelocity, maxForce);
}

void SliderJoint::disableMotor()
{
    mDrive.disable();
}

void SliderJoint::setServo(f32 targetPosition, const ServoSettings& settings)
{
    mDrive.setServo(targetPosition, settings);
}

void SliderJoint::setServoTarget(f32 targetPosition, f32 targetVelocity)
{
    mDrive.setServoTarget(targetPosition, targetVelocity);
}

void SliderJoint::disableServo()
{
    mDrive.disable();
}

f32 SliderJoint::motorForce() const
{
    return mDrive.appliedForce();
}

f32 SliderJoint::limitForce() const
{
    return mPreviousDuration > 0.0f ? mTotalLimitImpulse / mPreviousDuration : 0.0f;
}

Math::vec3 SliderJoint::lockForce() const
{
    if (!(mPreviousDuration > 0.0f))
        return Math::vec3(0.0f);
    return (mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y) / mPreviousDuration;
}

Math::vec3 SliderJoint::lockTorque() const
{
    return mPreviousDuration > 0.0f ? mTotalRotationImpulse / mPreviousDuration : Math::vec3(0.0f);
}

Math::vec3 SliderJoint::reactionForce() const
{
    return lockForce() + mWorldSliderAxis * (motorForce() + limitForce());
}

Math::vec3 SliderJoint::reactionTorque() const
{
    return lockTorque();
}

Math::vec3 SliderJoint::axisWorld() const
{
    return Math::normalize(mBodyA->directionToWorld(mLocalSliderAxisA));
}

void SliderJoint::calculateArmsAndOffset()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    mOffset = (mBodyB->position() - mBodyA->position()) + mArmB - mArmA;
}

void SliderJoint::calculatePositionLockProperties()
{
    mN1 = mBodyA->directionToWorld(mLocalNormalAxisA);
    mN2 = mBodyA->directionToWorld(mLocalNormalAxisA2);

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1x1 = Math::cross(armAPlusOffset, mN1);
    const Math::vec3 r1x2 = Math::cross(armAPlusOffset, mN2);
    const Math::vec3 r2x1 = Math::cross(mArmB, mN1);
    const Math::vec3 r2x2 = Math::cross(mArmB, mN2);

    Math::mat2 inverseEffectiveMass(0.0f);
    const f32 inverseMassSum = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass[0][0] = inverseMassSum + Math::dot(r1x1, mBodyA->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x1, mBodyB->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[0][1] = Math::dot(r1x1, mBodyA->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x1, mBodyB->inverseInertiaTensorWorld() * r2x2);
    inverseEffectiveMass[1][0] = Math::dot(r1x2, mBodyA->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x2, mBodyB->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[1][1] = inverseMassSum + Math::dot(r1x2, mBodyA->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x2, mBodyB->inverseInertiaTensorWorld() * r2x2);

    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (detail::invertible(inverseEffectiveMass, determinant))
        mPositionLockEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mPositionLockEffectiveMass = Math::mat2(0.0f);
        mTotalPositionLockImpulse = Math::vec2(0.0f);
    }
}

void SliderJoint::calculateRotationProperties()
{
    const Math::mat3 inverseInertiaSum =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    const f32 determinant = Math::determinant(inverseInertiaSum);
    if (detail::invertible(inverseInertiaSum, determinant))
        mRotationEffectiveMass = Math::inverse(inverseInertiaSum);
    else
    {
        mRotationEffectiveMass = Math::mat3(0.0f);
        mTotalRotationImpulse = Math::vec3(0.0f);
    }
}

void SliderJoint::calculateSlideAxisAndPosition()
{
    mWorldSliderAxis = Math::normalize(mBodyA->directionToWorld(mLocalSliderAxisA));
    mSlidePosition = Math::dot(mOffset, mWorldSliderAxis);
}

// The limit and the motor are the same row, along the slider axis:
// J = [-axis, -((armA + offset) x axis), axis, armB x axis], angular part of
// J M^-1 J^T (r x axis) . I^-1 (r x axis) per body.
f32 SliderJoint::calculateAxialEffectiveMass() const
{
    const Math::vec3 armAPlusOffsetCrossAxis = Math::cross(mArmA + mOffset, mWorldSliderAxis);
    const Math::vec3 armBCrossAxis = Math::cross(mArmB, mWorldSliderAxis);
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass += Math::dot(
        armAPlusOffsetCrossAxis, mBodyA->inverseInertiaTensorWorld() * armAPlusOffsetCrossAxis);
    inverseEffectiveMass +=
        Math::dot(armBCrossAxis, mBodyB->inverseInertiaTensorWorld() * armBCrossAxis);
    return inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

// Measures only: the accumulated impulse belongs to the velocity pass, and
// solvePosition() comes through here too (see setup()).
void SliderJoint::calculateLimitProperties()
{
    mLimitActive = mHasLimits && (mSlidePosition <= mLimitsMin || mSlidePosition >= mLimitsMax);
    if (!mLimitActive)
    {
        mLimitEffectiveMass = 0.0f;
        return;
    }
    mLimitEffectiveMass = calculateAxialEffectiveMass();
    if (mLimitEffectiveMass == 0.0f)
        mLimitActive = false;
}

// Rate of the slide position: B's anchor relative to the same point carried by
// A, along the axis; the negative of the J*v the limit row is written with.
f32 SliderJoint::axialVelocity() const
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    return Math::dot(mWorldSliderAxis, mBodyB->velocity() - mBodyA->velocity()) -
           Math::dot(Math::cross(armAPlusOffset, mWorldSliderAxis), mBodyA->angularVelocity()) +
           Math::dot(Math::cross(mArmB, mWorldSliderAxis), mBodyB->angularVelocity());
}

void SliderJoint::calculateDriveProperties(f32 duration)
{
    if (!mDrive.enabled())
    {
        mDrive.prepare(duration, 0.0f, JointDrive::Coordinate());
        return;
    }
    JointDrive::Coordinate coordinate;
    coordinate.position = mSlidePosition;
    coordinate.velocity = axialVelocity();
    coordinate.hasLimits = mHasLimits;
    coordinate.minPosition = mLimitsMin;
    coordinate.maxPosition = mLimitsMax;
    const f32 effectiveMass = calculateAxialEffectiveMass();
    const f32 inverseEffectiveMass = effectiveMass > 0.0f ? 1.0f / effectiveMass : 0.0f;
    // With the lateral and rotation locks (detail::coupleDriveToLocks); the slide
    // carries the bodies' whole mass whatever the anchor's offset.
    const Math::vec3 armA = mArmA + mOffset;
    detail::JacobianRow locks[5];
    locks[0] = detail::linearRow(armA, mArmB, mN1);
    locks[1] = detail::linearRow(armA, mArmB, mN2);
    locks[2] = detail::angularRow(Math::vec3(1.0f, 0.0f, 0.0f));
    locks[3] = detail::angularRow(Math::vec3(0.0f, 1.0f, 0.0f));
    locks[4] = detail::angularRow(Math::vec3(0.0f, 0.0f, 1.0f));
    const detail::DriveCoupling coupling = detail::coupleDriveToLocks(
        *mBodyA, *mBodyB, detail::linearRow(armA, mArmB, mWorldSliderAxis), locks, 5,
        mDriveCompensation);
    coordinate.gainInverseMass = coupling.gainInverseMass;
    mDrive.prepare(duration, inverseEffectiveMass > 0.0f ? coupling.inverseMass : 0.0f, coordinate);
    // A sleeping body is skipped by the solver - see HingeJoint's motor.
    if (mDrive.wantsWake())
        wakeBodies();
}

void SliderJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculateRotationProperties();
    calculateSlideAxisAndPosition();
    calculateLimitProperties();
    calculateDriveProperties(duration);
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionLockImpulse *= ratio;
        mTotalRotationImpulse *= ratio;
        mTotalLimitImpulse *= ratio;
    }
    else
    {
        mTotalPositionLockImpulse = Math::vec2(0.0f);
        mTotalRotationImpulse = Math::vec3(0.0f);
        mTotalLimitImpulse = 0.0f;
    }
    // Whether the limit's impulse carries into this step is decided here only:
    // the position pass re-measures after the bodies moved, and an in-range
    // reading there does not mean the contact is gone.
    if (!mLimitActive)
        mTotalLimitImpulse = 0.0f;
    mPreviousDuration = duration;
}

// A rigid servo's position error, closed with the joint's other rows
// (JointDrive::positionError).
void SliderJoint::solveDrivePosition(f32 baumgarte)
{
    if (!mDrive.active())
        return;
    calculateArmsAndOffset();
    calculateSlideAxisAndPosition();
    JointDrive::Coordinate coordinate;
    coordinate.position = mSlidePosition;
    coordinate.hasLimits = mHasLimits;
    coordinate.minPosition = mLimitsMin;
    coordinate.maxPosition = mLimitsMax;
    f32 error = 0.0f;
    if (!mDrive.positionError(coordinate, error))
        return;
    const f32 effectiveMass = calculateAxialEffectiveMass();
    const f32 lambda = -effectiveMass * baumgarte * error;
    mBodyA->applyPositionImpulseAtPoint(-(lambda * mWorldSliderAxis), mBodyA->position() + mArmA + mOffset);
    mBodyB->applyPositionImpulseAtPoint(lambda * mWorldSliderAxis, mBodyB->position() + mArmB);
}

void SliderJoint::applyVelocityImpulse(const Math::vec3& impulse)
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass());
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            mBodyA->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, impulse));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass());
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, impulse));
    }
}

void SliderJoint::applyAngularVelocityImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void SliderJoint::warmStart()
{
    if (mDrive.active())
        applyVelocityImpulse(mWorldSliderAxis * mDrive.totalImpulse());
    applyVelocityImpulse(mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y);
    applyAngularVelocityImpulse(mTotalRotationImpulse);
    if (mLimitActive)
        applyVelocityImpulse(mWorldSliderAxis * mTotalLimitImpulse);
}

void SliderJoint::solveVelocity()
{
    if (mDrive.active())
    {
        const f32 impulse = mDrive.solve(axialVelocity());
        applyVelocityImpulse(mWorldSliderAxis * impulse);
        const Math::vec2 lateral = Math::vec2(mDriveCompensation[0], mDriveCompensation[1]) * impulse;
        const Math::vec3 rotation =
            Math::vec3(mDriveCompensation[2], mDriveCompensation[3], mDriveCompensation[4]) * impulse;
        mTotalPositionLockImpulse += lateral;
        mTotalRotationImpulse += rotation;
        applyVelocityImpulse(mN1 * lateral.x + mN2 * lateral.y);
        applyAngularVelocityImpulse(rotation);
    }

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 deltaLinear = mBodyA->velocity() - mBodyB->velocity();
    Math::vec2 jv;
    jv.x = Math::dot(mN1, deltaLinear) + Math::dot(Math::cross(armAPlusOffset, mN1), mBodyA->angularVelocity()) -
          Math::dot(Math::cross(mArmB, mN1), mBodyB->angularVelocity());
    jv.y = Math::dot(mN2, deltaLinear) + Math::dot(Math::cross(armAPlusOffset, mN2), mBodyA->angularVelocity()) -
          Math::dot(Math::cross(mArmB, mN2), mBodyB->angularVelocity());
    const Math::vec2 lockImpulse = mPositionLockEffectiveMass * jv;
    mTotalPositionLockImpulse += lockImpulse;
    applyVelocityImpulse(mN1 * lockImpulse.x + mN2 * lockImpulse.y);

    const Math::vec3 rotationImpulse =
        mRotationEffectiveMass * (mBodyA->angularVelocity() - mBodyB->angularVelocity());
    mTotalRotationImpulse += rotationImpulse;
    applyAngularVelocityImpulse(rotationImpulse);

    if (mLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMin != mLimitsMax)
        {
            if (mSlidePosition <= mLimitsMin)
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mWorldSliderAxis, mBodyA->velocity() - mBodyB->velocity()) +
                             Math::dot(Math::cross(armAPlusOffset, mWorldSliderAxis),
                                      mBodyA->angularVelocity()) -
                             Math::dot(Math::cross(mArmB, mWorldSliderAxis), mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMass * relative;
        const f32 previous = mTotalLimitImpulse;
        mTotalLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyVelocityImpulse(mWorldSliderAxis * (mTotalLimitImpulse - previous));
    }
}

void SliderJoint::solvePosition(f32 baumgarte)
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
        mBodyA->applyPositionImpulseAtPoint(-impulse, mBodyA->position() + armAPlusOffset);
        mBodyB->applyPositionImpulseAtPoint(impulse, mBodyB->position() + mArmB);
    }

    calculateRotationProperties();
    Math::quat diff =
        mBodyB->orientation() * mInverseInitialOrientation * Math::conjugate(mBodyA->orientation());
    if (diff.w < 0.0f)
        diff = -diff;
    const Math::vec3 rotationError(2.0f * diff.x, 2.0f * diff.y, 2.0f * diff.z);
    if (rotationError != Math::vec3(0.0f))
    {
        const Math::vec3 lambda = -baumgarte * (mRotationEffectiveMass * rotationError);
        if (mBodyA->isDynamic())
        {
            const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * -lambda;
            const Math::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * lambda;
            const Math::quat spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    if (mHasLimits)
    {
        calculateArmsAndOffset();
        calculateSlideAxisAndPosition();
        calculateLimitProperties();
        if (mLimitActive)
        {
            f32 error = 0.0f;
            if (mSlidePosition <= mLimitsMin)
                error = mSlidePosition - mLimitsMin;
            else
                error = mSlidePosition - mLimitsMax;
            const f32 lambda = -mLimitEffectiveMass * baumgarte * error;
            const Math::vec3 armAPlusOffset = mArmA + mOffset;
            mBodyA->applyPositionImpulseAtPoint(-(lambda * mWorldSliderAxis),
                                                mBodyA->position() + armAPlusOffset);
            mBodyB->applyPositionImpulseAtPoint(lambda * mWorldSliderAxis,
                                                mBodyB->position() + mArmB);
        }
    }
}

// The two lateral rows (A's normal axes, at B's anchor) and the three rotation
// rows, as setup() and solvePosition() build them.
u32 SliderJoint::articulationLocks(JointRow* rows, f32* errors)
{
    const Math::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const Math::vec3 offset = (mBodyB->position() - mBodyA->position()) + armB - armA;
    const Math::vec3 n1 = mBodyA->directionToWorld(mLocalNormalAxisA);
    const Math::vec3 n2 = mBodyA->directionToWorld(mLocalNormalAxisA2);
    rows[0] = detail::linearRow(armA + offset, armB, n1);
    rows[1] = detail::linearRow(armA + offset, armB, n2);
    rows[2] = detail::angularRow(Math::vec3(1.0f, 0.0f, 0.0f));
    rows[3] = detail::angularRow(Math::vec3(0.0f, 1.0f, 0.0f));
    rows[4] = detail::angularRow(Math::vec3(0.0f, 0.0f, 1.0f));
    if (errors)
    {
        Math::quat diff =
            mBodyB->orientation() * mInverseInitialOrientation * Math::conjugate(mBodyA->orientation());
        if (diff.w < 0.0f)
            diff = -diff;
        errors[0] = Math::dot(offset, n1);
        errors[1] = Math::dot(offset, n2);
        errors[2] = 2.0f * diff.x;
        errors[3] = 2.0f * diff.y;
        errors[4] = 2.0f * diff.z;
    }
    return 5;
}

void SliderJoint::addArticulationLockImpulses(const f32* impulses)
{
    mTotalPositionLockImpulse += Math::vec2(impulses[0], impulses[1]);
    mTotalRotationImpulse += Math::vec3(impulses[2], impulses[3], impulses[4]);
}

u32 SliderJoint::articulationMotors(ArticulationMotor* motors)
{
    u32 count = 0;
    const detail::JacobianRow axial = detail::linearRow(mArmA + mOffset, mArmB, mWorldSliderAxis);
    if (mDrive.active())
    {
        motors[count].row = axial;
        motors[count].id = 0;
        motors[count].wantsGain =
            mDrive.mode() == JointDrive::Mode::Position && mDrive.servoSettings().frequency > 0.0f;
        ++count;
    }
    if (mLimitActive)
    {
        motors[count].row = axial;
        motors[count].id = 1;
        ++count;
    }
    return count;
}

void SliderJoint::setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass)
{
    if (id == 0)
        mDrive.setInverseMass(inverseMass, gainInverseMass);
    else if (inverseMass > 1.0e-9f)
        mLimitEffectiveMass = 1.0f / inverseMass;
}

f32 SliderJoint::solveArticulationMotor(u32 id, f32 velocity)
{
    if (id == 0)
        return mDrive.solve(velocity);
    f32 minImpulse = -M_INFINITY;
    f32 maxImpulse = M_INFINITY;
    if (mLimitsMin != mLimitsMax)
    {
        if (mSlidePosition <= mLimitsMin)
            minImpulse = 0.0f;
        else
            maxImpulse = 0.0f;
    }
    const f32 previous = mTotalLimitImpulse;
    mTotalLimitImpulse = Math::clamp(previous - mLimitEffectiveMass * velocity, minImpulse, maxImpulse);
    return mTotalLimitImpulse - previous;
}

u32 SliderJoint::articulationPositionRows(JointRow* rows, f32* errors)
{
    if (!mDrive.active() && !mHasLimits)
        return 0;
    calculateArmsAndOffset();
    calculateSlideAxisAndPosition();
    const detail::JacobianRow axial = detail::linearRow(mArmA + mOffset, mArmB, mWorldSliderAxis);
    u32 count = 0;
    if (mDrive.active())
    {
        JointDrive::Coordinate coordinate;
        coordinate.position = mSlidePosition;
        coordinate.hasLimits = mHasLimits;
        coordinate.minPosition = mLimitsMin;
        coordinate.maxPosition = mLimitsMax;
        f32 error = 0.0f;
        if (mDrive.positionError(coordinate, error))
        {
            rows[count] = axial;
            errors[count] = error;
            ++count;
        }
    }
    if (mHasLimits && (mSlidePosition < mLimitsMin || mSlidePosition > mLimitsMax))
    {
        rows[count] = axial;
        errors[count] = mSlidePosition < mLimitsMin ? mSlidePosition - mLimitsMin : mSlidePosition - mLimitsMax;
        ++count;
    }
    return count;
}

}
