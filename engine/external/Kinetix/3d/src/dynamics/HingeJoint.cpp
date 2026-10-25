#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/HingeJoint.h"

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

Math::quat invInitialOrientationXZ(const Math::vec3& xAxisA, const Math::vec3& zAxisA,
                                  const Math::vec3& xAxisB, const Math::vec3& zAxisB)
{
    if (xAxisA == xAxisB && zAxisA == zAxisB)
        return Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const Math::mat3 basisA(xAxisA, Math::cross(zAxisA, xAxisA), zAxisA);
    const Math::mat3 basisB(xAxisB, Math::cross(zAxisB, xAxisB), zAxisB);
    return Math::quat_cast(basisB) * Math::conjugate(Math::quat_cast(basisA));
}

}

HingeJoint::HingeJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                       const Math::vec3& worldHingeAxis)
    : HingeJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(Math::normalize(worldHingeAxis)),
                a.directionToLocal(normalizedPerpendicular(Math::normalize(worldHingeAxis))), b,
                b.pointToLocal(worldAnchor), b.directionToLocal(Math::normalize(worldHingeAxis)),
                b.directionToLocal(normalizedPerpendicular(Math::normalize(worldHingeAxis))))
{
}

HingeJoint::HingeJoint(RigidBody& a, const Math::vec3& localAnchorA,
                       const Math::vec3& localHingeAxisA, const Math::vec3& localNormalAxisA,
                       RigidBody& b, const Math::vec3& localAnchorB,
                       const Math::vec3& localHingeAxisB, const Math::vec3& localNormalAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalHingeAxisA(detail::normalizedAxisOr(localHingeAxisA, Math::vec3(0.0f, 1.0f, 0.0f))),
      mLocalHingeAxisB(detail::normalizedAxisOr(localHingeAxisB, Math::vec3(0.0f, 1.0f, 0.0f))),
      mLocalNormalAxisA(detail::normalizedAxisOr(localNormalAxisA, Math::vec3(1.0f, 0.0f, 0.0f))),
      mLocalNormalAxisB(detail::normalizedAxisOr(localNormalAxisB, Math::vec3(1.0f, 0.0f, 0.0f))),
      mInverseInitialOrientation(invInitialOrientationXZ(mLocalNormalAxisA, mLocalHingeAxisA,
                                                          mLocalNormalAxisB, mLocalHingeAxisB))
{
}

void HingeJoint::configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                           const Math::vec3& worldHingeAxis)
{
    mBodyA = &a;
    mBodyB = &b;
    mLocalAnchorA = a.pointToLocal(worldAnchor);
    mLocalAnchorB = b.pointToLocal(worldAnchor);
    const Math::vec3 axis = detail::normalizedAxisOr(worldHingeAxis, Math::vec3(0.0f, 1.0f, 0.0f));
    mLocalHingeAxisA = Math::normalize(a.directionToLocal(axis));
    mLocalHingeAxisB = Math::normalize(b.directionToLocal(axis));
    mLocalNormalAxisA = Math::normalize(a.directionToLocal(normalizedPerpendicular(axis)));
    mLocalNormalAxisB = Math::normalize(b.directionToLocal(normalizedPerpendicular(axis)));
    mInverseInitialOrientation = invInitialOrientationXZ(mLocalNormalAxisA, mLocalHingeAxisA,
                                                         mLocalNormalAxisB, mLocalHingeAxisB);
}

RigidBody* HingeJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* HingeJoint::bodyB() const
{
    return mBodyB;
}

void HingeJoint::setLimits(f32 minAngle, f32 maxAngle)
{
    mLimitsMin = Math::clamp(minAngle, -Math::pi<f32>(), Math::pi<f32>());
    mLimitsMax = Math::max(Math::clamp(maxAngle, -Math::pi<f32>(), Math::pi<f32>()), mLimitsMin);
    mHasLimits = mLimitsMin > -Math::pi<f32>() || mLimitsMax < Math::pi<f32>();
}

void HingeJoint::resetWarmStart()
{
    mTotalPositionImpulse = Math::vec3(0.0f);
    mTotalHingeRotationImpulse = Math::vec2(0.0f);
    mTotalLimitImpulse = 0.0f;
    mDrive.resetWarmStart();
    mPreviousDuration = 0.0f;
}

f32 HingeJoint::currentAngle() const
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalHingeAxisA));
}

f32 HingeJoint::currentAngularVelocity() const
{
    return Math::dot(mBodyA->directionToWorld(mLocalHingeAxisA),
                     mBodyB->angularVelocity() - mBodyA->angularVelocity());
}

void HingeJoint::setMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    mDrive.setVelocity(targetAngularVelocity, maxTorque);
}

void HingeJoint::disableMotor()
{
    mDrive.disable();
}

void HingeJoint::setServo(f32 targetAngle, const ServoSettings& settings)
{
    mDrive.setServo(targetAngle, settings);
}

void HingeJoint::setServoTarget(f32 targetAngle, f32 targetAngularVelocity)
{
    mDrive.setServoTarget(targetAngle, targetAngularVelocity);
}

void HingeJoint::disableServo()
{
    mDrive.disable();
}

f32 HingeJoint::motorTorque() const
{
    return mDrive.appliedForce();
}

f32 HingeJoint::limitTorque() const
{
    // Not gated on mLimitActive: the position pass re-measures that after the
    // step, while setup() already zeroed the impulse of a limit not in contact.
    return mPreviousDuration > 0.0f ? mTotalLimitImpulse / mPreviousDuration : 0.0f;
}

Math::vec3 HingeJoint::lockForce() const
{
    return mPreviousDuration > 0.0f ? mTotalPositionImpulse / mPreviousDuration : Math::vec3(0.0f);
}

Math::vec3 HingeJoint::lockTorque() const
{
    if (!(mPreviousDuration > 0.0f))
        return Math::vec3(0.0f);
    return (mB2xA1 * mTotalHingeRotationImpulse.x + mC2xA1 * mTotalHingeRotationImpulse.y) /
           mPreviousDuration;
}

Math::vec3 HingeJoint::reactionForce() const
{
    return lockForce();
}

Math::vec3 HingeJoint::reactionTorque() const
{
    return lockTorque() + mA1 * (motorTorque() + limitTorque());
}

Math::vec3 HingeJoint::axisWorld() const
{
    return Math::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
}

void HingeJoint::calculatePositionProperties()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    Math::mat3 inverseEffectiveMass(0.0f);
    const Math::vec3 axes[] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                              Math::vec3(0.0f, 0.0f, 1.0f)};
    for (u32 axis = 0; axis < 3; ++axis)
    {
        Math::vec3 response = axes[axis] * (mBodyA->inverseMass() + mBodyB->inverseMass());
        response += Math::cross(
            mBodyA->inverseInertiaTensorWorld() * Math::cross(mArmA, axes[axis]), mArmA);
        response += Math::cross(
            mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, axes[axis]), mArmB);
        inverseEffectiveMass[axis] = response;
    }
    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (detail::invertible(inverseEffectiveMass, determinant))
        mPositionEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mPositionEffectiveMass = Math::mat3(0.0f);
        mTotalPositionImpulse = Math::vec3(0.0f);
    }
}

void HingeJoint::calculateHingeRotationProperties()
{
    mA1 = Math::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    Math::vec3 a2 = Math::normalize(mBodyB->directionToWorld(mLocalHingeAxisB));

    const f32 dot = Math::dot(mA1, a2);
    if (dot <= 1.0e-3f)
    {
        Math::vec3 perpendicular = a2 - dot * mA1;
        if (Math::dot(perpendicular, perpendicular) < 1.0e-6f)
            perpendicular = normalizedPerpendicular(mA1);
        else
            perpendicular = Math::normalize(perpendicular);
        a2 = Math::normalize(0.99f * perpendicular + 0.01f * mA1);
    }

    mB2 = normalizedPerpendicular(a2);
    mC2 = Math::cross(a2, mB2);
    mB2xA1 = Math::cross(mB2, mA1);
    mC2xA1 = Math::cross(mC2, mA1);

    const Math::mat3 inverseInertiaSum =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    Math::mat2 inverseEffectiveMass;
    inverseEffectiveMass[0][0] = Math::dot(mB2xA1, inverseInertiaSum * mB2xA1);
    inverseEffectiveMass[0][1] = Math::dot(mB2xA1, inverseInertiaSum * mC2xA1);
    inverseEffectiveMass[1][0] = Math::dot(mC2xA1, inverseInertiaSum * mB2xA1);
    inverseEffectiveMass[1][1] = Math::dot(mC2xA1, inverseInertiaSum * mC2xA1);

    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (detail::invertible(inverseEffectiveMass, determinant))
        mHingeRotationEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mHingeRotationEffectiveMass = Math::mat2(0.0f);
        mTotalHingeRotationImpulse = Math::vec2(0.0f);
    }
}

void HingeJoint::calculateAxisAndAngle()
{
    mA1 = Math::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    mTheta = rotationAngleAroundAxis(diff, mA1);
}

f32 HingeJoint::smallestAngleToLimit() const
{
    const f32 distanceToMin = centerAngleAroundZero(mTheta - mLimitsMin);
    const f32 distanceToMax = centerAngleAroundZero(mTheta - mLimitsMax);
    return fabsf(distanceToMin) < fabsf(distanceToMax) ? distanceToMin : distanceToMax;
}

bool HingeJoint::minLimitClosest() const
{
    const f32 distanceToMin = centerAngleAroundZero(mTheta - mLimitsMin);
    const f32 distanceToMax = centerAngleAroundZero(mTheta - mLimitsMax);
    return fabsf(distanceToMin) < fabsf(distanceToMax);
}

// Measures only: the accumulated impulse belongs to the velocity pass, and
// solvePosition() comes through here too (see setup()).
void HingeJoint::calculateLimitProperties(f32 duration)
{
    (void)duration;
    mLimitActive = mHasLimits && (mTheta <= mLimitsMin || mTheta >= mLimitsMax);
    if (!mLimitActive)
    {
        mLimitEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
    mLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mLimitEffectiveMass == 0.0f)
        mLimitActive = false;
}

void HingeJoint::calculateDriveProperties(f32 duration)
{
    if (!mDrive.enabled())
    {
        mDrive.prepare(duration, 0.0f, JointDrive::Coordinate());
        return;
    }
    JointDrive::Coordinate coordinate;
    coordinate.position = mTheta;
    coordinate.velocity = Math::dot(mA1, mBodyB->angularVelocity() - mBodyA->angularVelocity());
    coordinate.angular = true;
    coordinate.hasLimits = mHasLimits;
    coordinate.minPosition = mLimitsMin;
    coordinate.maxPosition = mLimitsMax;
    detail::JacobianRow locks[5];
    detail::pointLockRows(mArmA, mArmB, locks);
    locks[3] = detail::angularRow(mB2xA1);
    locks[4] = detail::angularRow(mC2xA1);
    const detail::DriveCoupling coupling = detail::coupleDriveToLocks(
        *mBodyA, *mBodyB, detail::angularRow(mA1), locks, 5, mDriveCompensation);
    coordinate.gainInverseMass = coupling.gainInverseMass;
    mDrive.prepare(duration, coupling.inverseMass, coordinate);
    // Wake for a drive turned on after its bodies settled: a sleeping body is
    // skipped by the solver, so the motor would read enabled and never move.
    if (mDrive.wantsWake())
        wakeBodies();
}

void HingeJoint::setup(f32 duration)
{
    calculatePositionProperties();
    calculateHingeRotationProperties();
    calculateAxisAndAngle();
    calculateLimitProperties(duration);
    calculateDriveProperties(duration);
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionImpulse *= ratio;
        mTotalHingeRotationImpulse *= ratio;
        mTotalLimitImpulse *= ratio;
    }
    else
    {
        mTotalPositionImpulse = Math::vec3(0.0f);
        mTotalHingeRotationImpulse = Math::vec2(0.0f);
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
void HingeJoint::solveDrivePosition(f32 baumgarte)
{
    if (!mDrive.active())
        return;
    calculateAxisAndAngle();
    JointDrive::Coordinate coordinate;
    coordinate.position = mTheta;
    coordinate.angular = true;
    coordinate.hasLimits = mHasLimits;
    coordinate.minPosition = mLimitsMin;
    coordinate.maxPosition = mLimitsMax;
    f32 error = 0.0f;
    if (!mDrive.positionError(coordinate, error))
        return;
    const f32 inverseMass = Math::dot(
        mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
    if (!(inverseMass > 1.0e-9f))
        return;
    detail::applyAngularPositionImpulse(*mBodyA, *mBodyB, mA1 * (-baumgarte * error / inverseMass));
}

void HingeJoint::applyVelocityImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass());
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            mBodyA->inverseInertiaTensorWorld() * Math::cross(mArmA, impulse));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass());
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, impulse));
    }
}

void HingeJoint::applyAngularVelocityImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void HingeJoint::warmStart()
{
    if (mDrive.active())
        applyAngularVelocityImpulse(mA1 * mDrive.totalImpulse());
    applyVelocityImpulse(mTotalPositionImpulse);
    applyAngularVelocityImpulse(mB2xA1 * mTotalHingeRotationImpulse.x +
                                mC2xA1 * mTotalHingeRotationImpulse.y);
    if (mLimitActive)
        applyAngularVelocityImpulse(mA1 * mTotalLimitImpulse);
}

void HingeJoint::solveVelocity()
{
    if (mDrive.active())
    {
        const f32 angularVelocity =
            Math::dot(mA1, mBodyB->angularVelocity() - mBodyA->angularVelocity());
        const f32 impulse = mDrive.solve(angularVelocity);
        applyAngularVelocityImpulse(mA1 * impulse);
        const Math::vec3 point =
            Math::vec3(mDriveCompensation[0], mDriveCompensation[1], mDriveCompensation[2]) * impulse;
        const Math::vec2 rotation = Math::vec2(mDriveCompensation[3], mDriveCompensation[4]) * impulse;
        mTotalPositionImpulse += point;
        mTotalHingeRotationImpulse += rotation;
        applyVelocityImpulse(point);
        applyAngularVelocityImpulse(mB2xA1 * rotation.x + mC2xA1 * rotation.y);
    }

    const Math::vec3 relativeVelocity =
        mBodyB->velocity() + Math::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - Math::cross(mBodyA->angularVelocity(), mArmA);
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyVelocityImpulse(positionImpulse);

    const Math::vec3 deltaAngular = mBodyA->angularVelocity() - mBodyB->angularVelocity();
    const Math::vec2 jv(Math::dot(mB2xA1, deltaAngular), Math::dot(mC2xA1, deltaAngular));
    const Math::vec2 hingeImpulse = mHingeRotationEffectiveMass * jv;
    mTotalHingeRotationImpulse += hingeImpulse;
    applyAngularVelocityImpulse(mB2xA1 * hingeImpulse.x + mC2xA1 * hingeImpulse.y);

    if (mLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMin != mLimitsMax)
        {
            if (minLimitClosest())
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMass * relative;
        const f32 previous = mTotalLimitImpulse;
        mTotalLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularVelocityImpulse(mA1 * (mTotalLimitImpulse - previous));
    }
}

void HingeJoint::solvePosition(f32 baumgarte)
{
    solveDrivePosition(baumgarte);

    calculatePositionProperties();
    const Math::vec3 pointA = mBodyA->position() + mArmA;
    const Math::vec3 pointB = mBodyB->position() + mArmB;
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculateHingeRotationProperties();
    const Math::vec2 c(Math::dot(mA1, mB2), Math::dot(mA1, mC2));
    if (c != Math::vec2(0.0f))
    {
        const Math::vec2 lambda = -baumgarte * (mHingeRotationEffectiveMass * c);
        const Math::vec3 impulse = mB2xA1 * lambda.x + mC2xA1 * lambda.y;
        if (mBodyA->isDynamic())
        {
            const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * -impulse;
            const Math::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * impulse;
            const Math::quat spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    if (mHasLimits)
    {
        calculateAxisAndAngle();
        calculateLimitProperties(0.0f);
        if (mLimitActive)
        {
            const f32 error = smallestAngleToLimit();
            const f32 lambda = -mLimitEffectiveMass * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mA1 * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() +
                                       0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mA1 * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() +
                                       0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

// The point lock on the three world axes and the two rows keeping B's hinge
// axis on A's: the same rows and frames as setup() and solvePosition(), so the
// impulses handed back are the ones warmStart() replays.
u32 HingeJoint::articulationLocks(JointRow* rows, f32* errors)
{
    const Math::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const Math::vec3 separation = (mBodyB->position() + armB) - (mBodyA->position() + armA);
    detail::pointLockRows(armA, armB, rows);

    const Math::vec3 a1 = Math::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    Math::vec3 a2 = Math::normalize(mBodyB->directionToWorld(mLocalHingeAxisB));
    const f32 dot = Math::dot(a1, a2);
    if (dot <= 1.0e-3f)
    {
        Math::vec3 perpendicular = a2 - dot * a1;
        if (Math::dot(perpendicular, perpendicular) < 1.0e-6f)
            perpendicular = normalizedPerpendicular(a1);
        else
            perpendicular = Math::normalize(perpendicular);
        a2 = Math::normalize(0.99f * perpendicular + 0.01f * a1);
    }
    const Math::vec3 b2 = normalizedPerpendicular(a2);
    const Math::vec3 c2 = Math::cross(a2, b2);
    rows[3] = detail::angularRow(Math::cross(b2, a1));
    rows[4] = detail::angularRow(Math::cross(c2, a1));
    if (errors)
    {
        errors[0] = separation.x;
        errors[1] = separation.y;
        errors[2] = separation.z;
        errors[3] = Math::dot(a1, b2);
        errors[4] = Math::dot(a1, c2);
    }
    return 5;
}

void HingeJoint::addArticulationLockImpulses(const f32* impulses)
{
    mTotalPositionImpulse += Math::vec3(impulses[0], impulses[1], impulses[2]);
    mTotalHingeRotationImpulse += Math::vec2(impulses[3], impulses[4]);
}

u32 HingeJoint::articulationMotors(ArticulationMotor* motors)
{
    u32 count = 0;
    if (mDrive.active())
    {
        motors[count].row = detail::angularRow(mA1);
        motors[count].id = 0;
        motors[count].wantsGain =
            mDrive.mode() == JointDrive::Mode::Position && mDrive.servoSettings().frequency > 0.0f;
        ++count;
    }
    if (mLimitActive)
    {
        motors[count].row = detail::angularRow(mA1);
        motors[count].id = 1;
        ++count;
    }
    return count;
}

void HingeJoint::setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass)
{
    if (id == 0)
        mDrive.setInverseMass(inverseMass, gainInverseMass);
    else if (inverseMass > 1.0e-9f)
        mLimitEffectiveMass = 1.0f / inverseMass;
}

f32 HingeJoint::solveArticulationMotor(u32 id, f32 velocity)
{
    if (id == 0)
        return mDrive.solve(velocity);
    f32 minImpulse = -M_INFINITY;
    f32 maxImpulse = M_INFINITY;
    if (mLimitsMin != mLimitsMax)
    {
        if (minLimitClosest())
            minImpulse = 0.0f;
        else
            maxImpulse = 0.0f;
    }
    const f32 previous = mTotalLimitImpulse;
    mTotalLimitImpulse = Math::clamp(previous - mLimitEffectiveMass * velocity, minImpulse, maxImpulse);
    return mTotalLimitImpulse - previous;
}

// A rigid servo's remaining error and a limit the hinge is beyond, both on the
// angle about A's axis.
u32 HingeJoint::articulationPositionRows(JointRow* rows, f32* errors)
{
    if (!mDrive.active() && !mHasLimits)
        return 0;
    calculateAxisAndAngle();
    u32 count = 0;
    if (mDrive.active())
    {
        JointDrive::Coordinate coordinate;
        coordinate.position = mTheta;
        coordinate.angular = true;
        coordinate.hasLimits = mHasLimits;
        coordinate.minPosition = mLimitsMin;
        coordinate.maxPosition = mLimitsMax;
        f32 error = 0.0f;
        if (mDrive.positionError(coordinate, error))
        {
            rows[count] = detail::angularRow(mA1);
            errors[count] = error;
            ++count;
        }
    }
    if (mHasLimits && (mTheta < mLimitsMin || mTheta > mLimitsMax))
    {
        rows[count] = detail::angularRow(mA1);
        errors[count] = smallestAngleToLimit();
        ++count;
    }
    return count;
}

}
