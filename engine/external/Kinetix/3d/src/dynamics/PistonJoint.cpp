#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/PistonJoint.h"

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

PistonJoint::PistonJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                         const Math::vec3& worldAxis)
    : PistonJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(Math::normalize(worldAxis)),
                 a.directionToLocal(normalizedPerpendicular(Math::normalize(worldAxis))), b,
                 b.pointToLocal(worldAnchor), b.directionToLocal(Math::normalize(worldAxis)),
                 b.directionToLocal(normalizedPerpendicular(Math::normalize(worldAxis))))
{
}

PistonJoint::PistonJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localAxisA,
                         const Math::vec3& localNormalAxisA, RigidBody& b,
                         const Math::vec3& localAnchorB, const Math::vec3& localAxisB,
                         const Math::vec3& localNormalAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalAxisA(detail::normalizedAxisOr(localAxisA, Math::vec3(1.0f, 0.0f, 0.0f))),
      mLocalAxisB(detail::normalizedAxisOr(localAxisB, Math::vec3(1.0f, 0.0f, 0.0f))),
      mLocalNormalAxisA(detail::normalizedAxisOr(localNormalAxisA, Math::vec3(0.0f, 1.0f, 0.0f))),
      mLocalNormalAxisA2(Math::cross(mLocalAxisA, mLocalNormalAxisA)),
      mInverseInitialOrientation(invInitialOrientationXZ(
          mLocalNormalAxisA, mLocalAxisA,
          detail::normalizedAxisOr(localNormalAxisB, Math::vec3(0.0f, 1.0f, 0.0f)), mLocalAxisB))
{
}

void PistonJoint::configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                            const Math::vec3& worldAxis)
{
    mBodyA = &a;
    mBodyB = &b;
    mLocalAnchorA = a.pointToLocal(worldAnchor);
    mLocalAnchorB = b.pointToLocal(worldAnchor);
    // Guarded once; the four frames below derive from it.
    const Math::vec3 axis = detail::normalizedAxisOr(worldAxis, Math::vec3(1.0f, 0.0f, 0.0f));
    mLocalAxisA = Math::normalize(a.directionToLocal(axis));
    mLocalAxisB = Math::normalize(b.directionToLocal(axis));
    mLocalNormalAxisA = Math::normalize(a.directionToLocal(normalizedPerpendicular(axis)));
    mLocalNormalAxisA2 = Math::cross(mLocalAxisA, mLocalNormalAxisA);
    const Math::vec3 localNormalAxisB = b.directionToLocal(normalizedPerpendicular(axis));
    mInverseInitialOrientation = invInitialOrientationXZ(mLocalNormalAxisA, mLocalAxisA,
                                                         Math::normalize(localNormalAxisB),
                                                         mLocalAxisB);
}

RigidBody* PistonJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* PistonJoint::bodyB() const
{
    return mBodyB;
}

void PistonJoint::setLinearLimits(f32 minDistance, f32 maxDistance)
{
    mLinearLimitsMin = minDistance;
    mLinearLimitsMax = maxDistance;
    mHasLinearLimits = mLinearLimitsMin != -M_INFINITY || mLinearLimitsMax != M_INFINITY;
}

void PistonJoint::setAngularLimits(f32 minAngle, f32 maxAngle)
{
    mAngularLimitsMin = Math::clamp(minAngle, -Math::pi<f32>(), Math::pi<f32>());
    mAngularLimitsMax =
        Math::max(Math::clamp(maxAngle, -Math::pi<f32>(), Math::pi<f32>()), mAngularLimitsMin);
    mHasAngularLimits = mAngularLimitsMin > -Math::pi<f32>() || mAngularLimitsMax < Math::pi<f32>();
}

f32 PistonJoint::currentPosition() const
{
    const Math::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const Math::vec3 offset = (mBodyB->position() - mBodyA->position()) + armB - armA;
    return Math::dot(offset, mBodyA->directionToWorld(mLocalAxisA));
}

f32 PistonJoint::currentAngle() const
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalAxisA));
}

f32 PistonJoint::currentVelocity() const
{
    const Math::vec3 axis = mBodyA->directionToWorld(mLocalAxisA);
    const Math::vec3 pointB = mBodyB->pointToWorld(mLocalAnchorB);
    return Math::dot(axis, mBodyB->velocityAtPoint(pointB) - mBodyA->velocityAtPoint(pointB));
}

f32 PistonJoint::currentAngularVelocity() const
{
    return Math::dot(mBodyA->directionToWorld(mLocalAxisA),
                     mBodyB->angularVelocity() - mBodyA->angularVelocity());
}

void PistonJoint::setLinearMotor(f32 targetVelocity, f32 maxForce)
{
    mLinearDrive.setVelocity(targetVelocity, maxForce);
}

void PistonJoint::disableLinearMotor()
{
    mLinearDrive.disable();
}

void PistonJoint::setAngularMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    mAngularDrive.setVelocity(targetAngularVelocity, maxTorque);
}

void PistonJoint::disableAngularMotor()
{
    mAngularDrive.disable();
}

void PistonJoint::setLinearServo(f32 targetPosition, const ServoSettings& settings)
{
    mLinearDrive.setServo(targetPosition, settings);
}

void PistonJoint::setLinearServoTarget(f32 targetPosition, f32 targetVelocity)
{
    mLinearDrive.setServoTarget(targetPosition, targetVelocity);
}

void PistonJoint::disableLinearServo()
{
    mLinearDrive.disable();
}

void PistonJoint::setAngularServo(f32 targetAngle, const ServoSettings& settings)
{
    mAngularDrive.setServo(targetAngle, settings);
}

void PistonJoint::setAngularServoTarget(f32 targetAngle, f32 targetAngularVelocity)
{
    mAngularDrive.setServoTarget(targetAngle, targetAngularVelocity);
}

void PistonJoint::disableAngularServo()
{
    mAngularDrive.disable();
}

f32 PistonJoint::linearMotorForce() const
{
    return mLinearDrive.appliedForce();
}

f32 PistonJoint::angularMotorTorque() const
{
    return mAngularDrive.appliedForce();
}

f32 PistonJoint::linearLimitForce() const
{
    return mPreviousDuration > 0.0f ? mTotalLinearLimitImpulse / mPreviousDuration : 0.0f;
}

f32 PistonJoint::angularLimitTorque() const
{
    return mPreviousDuration > 0.0f ? mTotalAngularLimitImpulse / mPreviousDuration : 0.0f;
}

Math::vec3 PistonJoint::lockForce() const
{
    if (!(mPreviousDuration > 0.0f))
        return Math::vec3(0.0f);
    return (mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y) / mPreviousDuration;
}

Math::vec3 PistonJoint::lockTorque() const
{
    if (!(mPreviousDuration > 0.0f))
        return Math::vec3(0.0f);
    return (mB2xA1 * mTotalRotationLockImpulse.x + mC2xA1 * mTotalRotationLockImpulse.y) /
           mPreviousDuration;
}

Math::vec3 PistonJoint::reactionForce() const
{
    return lockForce() + mA1 * (linearMotorForce() + linearLimitForce());
}

Math::vec3 PistonJoint::reactionTorque() const
{
    return lockTorque() + mA1 * (angularMotorTorque() + angularLimitTorque());
}

Math::vec3 PistonJoint::axisWorld() const
{
    return Math::normalize(mBodyA->directionToWorld(mLocalAxisA));
}

void PistonJoint::calculateArmsAndOffset()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    mOffset = (mBodyB->position() - mBodyA->position()) + mArmB - mArmA;
}

void PistonJoint::calculatePositionLockProperties()
{
    mN1 = mBodyA->directionToWorld(mLocalNormalAxisA);
    mN2 = mBodyA->directionToWorld(mLocalNormalAxisA2);

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1x1 = Math::cross(armAPlusOffset, mN1);
    const Math::vec3 r1x2 = Math::cross(armAPlusOffset, mN2);
    const Math::vec3 r2x1 = Math::cross(mArmB, mN1);
    const Math::vec3 r2x2 = Math::cross(mArmB, mN2);

    const f32 inverseMassSum = mBodyA->inverseMass() + mBodyB->inverseMass();
    Math::mat2 inverseEffectiveMass(0.0f);
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

void PistonJoint::calculateRotationLockProperties()
{
    mA1 = Math::normalize(mBodyA->directionToWorld(mLocalAxisA));
    Math::vec3 a2 = Math::normalize(mBodyB->directionToWorld(mLocalAxisB));

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
        mRotationLockEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mRotationLockEffectiveMass = Math::mat2(0.0f);
        mTotalRotationLockImpulse = Math::vec2(0.0f);
    }
}

void PistonJoint::calculateAxisAndPosition()
{
    mSlidePosition = Math::dot(mOffset, mA1);
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    mTheta = rotationAngleAroundAxis(diff, mA1);
}

// The linear limit and the linear motor are the same row, along the axis:
// J = [-a1, -((armA + offset) x a1), a1, armB x a1], whose angular part of
// J M^-1 J^T is (r x a1) . I^-1 (r x a1) for each body.
f32 PistonJoint::calculateLinearEffectiveMass() const
{
    const Math::vec3 armAPlusOffsetCrossAxis = Math::cross(mArmA + mOffset, mA1);
    const Math::vec3 armBCrossAxis = Math::cross(mArmB, mA1);
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass += Math::dot(
        armAPlusOffsetCrossAxis, mBodyA->inverseInertiaTensorWorld() * armAPlusOffsetCrossAxis);
    inverseEffectiveMass +=
        Math::dot(armBCrossAxis, mBodyB->inverseInertiaTensorWorld() * armBCrossAxis);
    return inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

// The two limit calculations measure only: the accumulated impulses belong to
// the velocity pass, and solvePosition() comes through here too (see setup()).
void PistonJoint::calculateLinearLimitProperties()
{
    mLinearLimitActive =
        mHasLinearLimits && (mSlidePosition <= mLinearLimitsMin || mSlidePosition >= mLinearLimitsMax);
    if (!mLinearLimitActive)
    {
        mLinearLimitEffectiveMass = 0.0f;
        return;
    }
    mLinearLimitEffectiveMass = calculateLinearEffectiveMass();
    if (mLinearLimitEffectiveMass == 0.0f)
        mLinearLimitActive = false;
}

void PistonJoint::calculateAngularLimitProperties(f32 duration)
{
    (void)duration;
    mAngularLimitActive =
        mHasAngularLimits && (mTheta <= mAngularLimitsMin || mTheta >= mAngularLimitsMax);
    if (!mAngularLimitActive)
    {
        mAngularLimitEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
    mAngularLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mAngularLimitEffectiveMass == 0.0f)
        mAngularLimitActive = false;
}

// Rate of the slide position, the negative of the J*v the linear limit row is
// written with.
f32 PistonJoint::axialVelocity() const
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    return Math::dot(mA1, mBodyB->velocity() - mBodyA->velocity()) -
           Math::dot(Math::cross(armAPlusOffset, mA1), mBodyA->angularVelocity()) +
           Math::dot(Math::cross(mArmB, mA1), mBodyB->angularVelocity());
}

void PistonJoint::calculateDriveProperties(f32 duration)
{
    if (mLinearDrive.enabled())
    {
        JointDrive::Coordinate coordinate;
        coordinate.position = mSlidePosition;
        coordinate.velocity = axialVelocity();
        coordinate.hasLimits = mHasLinearLimits;
        coordinate.minPosition = mLinearLimitsMin;
        coordinate.maxPosition = mLinearLimitsMax;
        const f32 effectiveMass = calculateLinearEffectiveMass();
        const f32 inverseEffectiveMass = effectiveMass > 0.0f ? 1.0f / effectiveMass : 0.0f;
        // One block with the locks, as in SliderJoint.
        const Math::vec3 armA = mArmA + mOffset;
        detail::JacobianRow locks[4];
        locks[0] = detail::linearRow(armA, mArmB, mN1);
        locks[1] = detail::linearRow(armA, mArmB, mN2);
        locks[2] = detail::angularRow(mB2xA1);
        locks[3] = detail::angularRow(mC2xA1);
        const detail::DriveCoupling coupling = detail::coupleDriveToLocks(
            *mBodyA, *mBodyB, detail::linearRow(armA, mArmB, mA1), locks, 4, mLinearDriveCompensation);
        coordinate.gainInverseMass = coupling.gainInverseMass;
        mLinearDrive.prepare(duration, inverseEffectiveMass > 0.0f ? coupling.inverseMass : 0.0f,
                             coordinate);
    }
    else
        mLinearDrive.prepare(duration, 0.0f, JointDrive::Coordinate());

    if (mAngularDrive.enabled())
    {
        JointDrive::Coordinate coordinate;
        coordinate.position = mTheta;
        coordinate.velocity = Math::dot(mA1, mBodyB->angularVelocity() - mBodyA->angularVelocity());
        coordinate.angular = true;
        coordinate.hasLimits = mHasAngularLimits;
        coordinate.minPosition = mAngularLimitsMin;
        coordinate.maxPosition = mAngularLimitsMax;
        const f32 inverseEffectiveMass = Math::dot(
            mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
        // One block with the locks, as in HingeJoint.
        const Math::vec3 armA = mArmA + mOffset;
        detail::JacobianRow locks[4];
        locks[0] = detail::linearRow(armA, mArmB, mN1);
        locks[1] = detail::linearRow(armA, mArmB, mN2);
        locks[2] = detail::angularRow(mB2xA1);
        locks[3] = detail::angularRow(mC2xA1);
        const detail::DriveCoupling coupling = detail::coupleDriveToLocks(
            *mBodyA, *mBodyB, detail::angularRow(mA1), locks, 4, mAngularDriveCompensation);
        coordinate.gainInverseMass = coupling.gainInverseMass;
        mAngularDrive.prepare(duration, inverseEffectiveMass > 1.0e-9f ? coupling.inverseMass : 0.0f,
                              coordinate);
    }
    else
        mAngularDrive.prepare(duration, 0.0f, JointDrive::Coordinate());

    // A sleeping body is skipped by the solver - see HingeJoint's motor.
    if (mLinearDrive.wantsWake() || mAngularDrive.wantsWake())
        wakeBodies();
}

void PistonJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculateRotationLockProperties();
    calculateAxisAndPosition();
    calculateLinearLimitProperties();
    calculateAngularLimitProperties(duration);
    calculateDriveProperties(duration);
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionLockImpulse *= ratio;
        mTotalRotationLockImpulse *= ratio;
        mTotalLinearLimitImpulse *= ratio;
        mTotalAngularLimitImpulse *= ratio;
    }
    else
    {
        mTotalPositionLockImpulse = Math::vec2(0.0f);
        mTotalRotationLockImpulse = Math::vec2(0.0f);
        mTotalLinearLimitImpulse = 0.0f;
        mTotalAngularLimitImpulse = 0.0f;
    }
    // Whether a limit's impulse carries into this step is decided here only: the
    // position pass re-measures after the bodies moved, and an in-range reading
    // there does not mean the contact is gone.
    if (!mLinearLimitActive)
        mTotalLinearLimitImpulse = 0.0f;
    if (!mAngularLimitActive)
        mTotalAngularLimitImpulse = 0.0f;
    mPreviousDuration = duration;
}

void PistonJoint::applyLinearImpulse(const Math::vec3& impulse)
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

void PistonJoint::applyAngularImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

// Rigid servos' position errors, closed with the joint's other rows
// (JointDrive::positionError).
void PistonJoint::solveDrivePosition(f32 baumgarte)
{
    if (!mLinearDrive.active() && !mAngularDrive.active())
        return;
    calculateArmsAndOffset();
    calculateRotationLockProperties();
    calculateAxisAndPosition();
    f32 error = 0.0f;
    JointDrive::Coordinate linear;
    linear.position = mSlidePosition;
    linear.hasLimits = mHasLinearLimits;
    linear.minPosition = mLinearLimitsMin;
    linear.maxPosition = mLinearLimitsMax;
    if (mLinearDrive.active() && mLinearDrive.positionError(linear, error))
    {
        const f32 lambda = -calculateLinearEffectiveMass() * baumgarte * error;
        mBodyA->applyPositionImpulseAtPoint(-(lambda * mA1), mBodyA->position() + mArmA + mOffset);
        mBodyB->applyPositionImpulseAtPoint(lambda * mA1, mBodyB->position() + mArmB);
        calculateArmsAndOffset();
        calculateRotationLockProperties();
        calculateAxisAndPosition();
    }
    JointDrive::Coordinate angular;
    angular.position = mTheta;
    angular.angular = true;
    angular.hasLimits = mHasAngularLimits;
    angular.minPosition = mAngularLimitsMin;
    angular.maxPosition = mAngularLimitsMax;
    if (mAngularDrive.active() && mAngularDrive.positionError(angular, error))
    {
        const f32 inverseMass = Math::dot(
            mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
        if (inverseMass > 1.0e-9f)
            detail::applyAngularPositionImpulse(*mBodyA, *mBodyB, mA1 * (-baumgarte * error / inverseMass));
    }
}

// Lock impulses that go with a drive impulse (detail::coupleDriveToLocks): two
// lateral rows, then two rotation rows.
void PistonJoint::applyLockCompensation(const f32* compensation, f32 impulse)
{
    const Math::vec2 lateral = Math::vec2(compensation[0], compensation[1]) * impulse;
    const Math::vec2 rotation = Math::vec2(compensation[2], compensation[3]) * impulse;
    mTotalPositionLockImpulse += lateral;
    mTotalRotationLockImpulse += rotation;
    applyLinearImpulse(mN1 * lateral.x + mN2 * lateral.y);
    applyAngularImpulse(mB2xA1 * rotation.x + mC2xA1 * rotation.y);
}

void PistonJoint::warmStart()
{
    if (mLinearDrive.active())
        applyLinearImpulse(mA1 * mLinearDrive.totalImpulse());
    if (mAngularDrive.active())
        applyAngularImpulse(mA1 * mAngularDrive.totalImpulse());
    applyLinearImpulse(mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y);
    applyAngularImpulse(mB2xA1 * mTotalRotationLockImpulse.x + mC2xA1 * mTotalRotationLockImpulse.y);
    if (mLinearLimitActive)
        applyLinearImpulse(mA1 * mTotalLinearLimitImpulse);
    if (mAngularLimitActive)
        applyAngularImpulse(mA1 * mTotalAngularLimitImpulse);
}

void PistonJoint::solveVelocity()
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;

    if (mLinearDrive.active())
    {
        const f32 impulse = mLinearDrive.solve(axialVelocity());
        applyLinearImpulse(mA1 * impulse);
        applyLockCompensation(mLinearDriveCompensation, impulse);
    }

    if (mAngularDrive.active())
    {
        const f32 angularVelocity =
            Math::dot(mA1, mBodyB->angularVelocity() - mBodyA->angularVelocity());
        const f32 impulse = mAngularDrive.solve(angularVelocity);
        applyAngularImpulse(mA1 * impulse);
        applyLockCompensation(mAngularDriveCompensation, impulse);
    }

    const Math::vec3 deltaLinear = mBodyA->velocity() - mBodyB->velocity();
    Math::vec2 positionJv;
    positionJv.x = Math::dot(mN1, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN1), mBodyA->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN1), mBodyB->angularVelocity());
    positionJv.y = Math::dot(mN2, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN2), mBodyA->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN2), mBodyB->angularVelocity());
    const Math::vec2 positionImpulse = mPositionLockEffectiveMass * positionJv;
    mTotalPositionLockImpulse += positionImpulse;
    applyLinearImpulse(mN1 * positionImpulse.x + mN2 * positionImpulse.y);

    const Math::vec3 deltaAngular = mBodyA->angularVelocity() - mBodyB->angularVelocity();
    const Math::vec2 rotationJv(Math::dot(mB2xA1, deltaAngular), Math::dot(mC2xA1, deltaAngular));
    const Math::vec2 rotationImpulse = mRotationLockEffectiveMass * rotationJv;
    mTotalRotationLockImpulse += rotationImpulse;
    applyAngularImpulse(mB2xA1 * rotationImpulse.x + mC2xA1 * rotationImpulse.y);

    if (mLinearLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLinearLimitsMin != mLinearLimitsMax)
        {
            if (mSlidePosition <= mLinearLimitsMin)
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mA1, mBodyA->velocity() - mBodyB->velocity()) +
                             Math::dot(Math::cross(armAPlusOffset, mA1), mBodyA->angularVelocity()) -
                             Math::dot(Math::cross(mArmB, mA1), mBodyB->angularVelocity());
        const f32 impulse = mLinearLimitEffectiveMass * relative;
        const f32 previous = mTotalLinearLimitImpulse;
        mTotalLinearLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyLinearImpulse(mA1 * (mTotalLinearLimitImpulse - previous));
    }

    if (mAngularLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mAngularLimitsMin != mAngularLimitsMax)
        {
            const f32 distanceToMin = centerAngleAroundZero(mTheta - mAngularLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mTheta - mAngularLimitsMax);
            if (fabsf(distanceToMin) < fabsf(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mAngularLimitEffectiveMass * relative;
        const f32 previous = mTotalAngularLimitImpulse;
        mTotalAngularLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mA1 * (mTotalAngularLimitImpulse - previous));
    }
}

void PistonJoint::solvePosition(f32 baumgarte)
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

    calculateRotationLockProperties();
    const Math::vec2 rc(Math::dot(mA1, mB2), Math::dot(mA1, mC2));
    if (rc != Math::vec2(0.0f))
    {
        const Math::vec2 lambda = -baumgarte * (mRotationLockEffectiveMass * rc);
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

    if (mHasLinearLimits)
    {
        calculateArmsAndOffset();
        calculateAxisAndPosition();
        calculateLinearLimitProperties();
        if (mLinearLimitActive)
        {
            f32 error = mSlidePosition <= mLinearLimitsMin ? mSlidePosition - mLinearLimitsMin
                                                            : mSlidePosition - mLinearLimitsMax;
            const f32 lambda = -mLinearLimitEffectiveMass * baumgarte * error;
            const Math::vec3 armAPlusOffset = mArmA + mOffset;
            mBodyA->applyPositionImpulseAtPoint(-(lambda * mA1), mBodyA->position() + armAPlusOffset);
            mBodyB->applyPositionImpulseAtPoint(lambda * mA1, mBodyB->position() + mArmB);
        }
    }

    if (mHasAngularLimits)
    {
        calculateAxisAndPosition();
        calculateAngularLimitProperties(0.0f);
        if (mAngularLimitActive)
        {
            const f32 distanceToMin = centerAngleAroundZero(mTheta - mAngularLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mTheta - mAngularLimitsMax);
            const f32 error = fabsf(distanceToMin) < fabsf(distanceToMax) ? distanceToMin
                                                                                 : distanceToMax;
            const f32 lambda = -mAngularLimitEffectiveMass * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mA1 * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mA1 * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
