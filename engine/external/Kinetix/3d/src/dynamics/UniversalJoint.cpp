#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/UniversalJoint.h"

#include "JointAxis.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

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

// A missing axis falls back to one perpendicular to the other at the pose the
// joint is built in: a fixed fallback could land parallel to the given axis (a
// cross folded flat).
Math::vec3 localAxisAOr(const RigidBody& a, const RigidBody& b, const Math::vec3& localAxisA,
                        const Math::vec3& localAxisB)
{
    const Math::vec3 worldAxisB =
        detail::normalizedAxisOr(b.directionToWorld(localAxisB), Math::vec3(0.0f, 0.0f, 1.0f));
    return detail::normalizedAxisOr(localAxisA,
                                    a.directionToLocal(normalizedPerpendicular(worldAxisB)));
}

Math::vec3 localAxisBOr(const RigidBody& a, const RigidBody& b, const Math::vec3& unitLocalAxisA,
                        const Math::vec3& localAxisB)
{
    const Math::vec3 worldAxisA = a.directionToWorld(unitLocalAxisA);
    return detail::normalizedAxisOr(localAxisB,
                                    b.directionToLocal(normalizedPerpendicular(worldAxisA)));
}

}

UniversalJoint::UniversalJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                               const Math::vec3& worldAxisA, const Math::vec3& worldAxisB)
    : UniversalJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(worldAxisA), b,
                     b.pointToLocal(worldAnchor), b.directionToLocal(worldAxisB))
{
}

UniversalJoint::UniversalJoint(RigidBody& a, const Math::vec3& localAnchorA,
                               const Math::vec3& localAxisA, RigidBody& b,
                               const Math::vec3& localAnchorB, const Math::vec3& localAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalAxisA(localAxisAOr(a, b, localAxisA, localAxisB)),
      mLocalAxisB(localAxisBOr(a, b, mLocalAxisA, localAxisB)),
      mInverseInitialOrientation(Math::conjugate(b.orientation()) * a.orientation())
{
}

RigidBody* UniversalJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* UniversalJoint::bodyB() const
{
    return mBodyB;
}

void UniversalJoint::setLimitsA(f32 minAngle, f32 maxAngle)
{
    mLimitsMinA = Math::clamp(minAngle, -Math::pi<f32>(), Math::pi<f32>());
    mLimitsMaxA = Math::max(Math::clamp(maxAngle, -Math::pi<f32>(), Math::pi<f32>()), mLimitsMinA);
    mHasLimitsA = mLimitsMinA > -Math::pi<f32>() || mLimitsMaxA < Math::pi<f32>();
}

void UniversalJoint::setLimitsB(f32 minAngle, f32 maxAngle)
{
    mLimitsMinB = Math::clamp(minAngle, -Math::pi<f32>(), Math::pi<f32>());
    mLimitsMaxB = Math::max(Math::clamp(maxAngle, -Math::pi<f32>(), Math::pi<f32>()), mLimitsMinB);
    mHasLimitsB = mLimitsMinB > -Math::pi<f32>() || mLimitsMaxB < Math::pi<f32>();
}

f32 UniversalJoint::currentAngleA() const
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalAxisA));
}

f32 UniversalJoint::currentAngleB() const
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyB->directionToWorld(mLocalAxisB));
}

f32 UniversalJoint::currentAngularVelocityA() const
{
    return Math::dot(Math::normalize(mBodyA->directionToWorld(mLocalAxisA)),
                     mBodyB->angularVelocity() - mBodyA->angularVelocity());
}

f32 UniversalJoint::currentAngularVelocityB() const
{
    return Math::dot(Math::normalize(mBodyB->directionToWorld(mLocalAxisB)),
                     mBodyB->angularVelocity() - mBodyA->angularVelocity());
}

void UniversalJoint::setMotorA(f32 targetAngularVelocity, f32 maxTorque)
{
    mDriveA.setVelocity(targetAngularVelocity, maxTorque);
}

void UniversalJoint::setMotorB(f32 targetAngularVelocity, f32 maxTorque)
{
    mDriveB.setVelocity(targetAngularVelocity, maxTorque);
}

void UniversalJoint::disableMotorA()
{
    mDriveA.disable();
}

void UniversalJoint::disableMotorB()
{
    mDriveB.disable();
}

void UniversalJoint::setServoA(f32 targetAngle, const ServoSettings& settings)
{
    mDriveA.setServo(targetAngle, settings);
}

void UniversalJoint::setServoB(f32 targetAngle, const ServoSettings& settings)
{
    mDriveB.setServo(targetAngle, settings);
}

void UniversalJoint::setServoTargetA(f32 targetAngle, f32 targetAngularVelocity)
{
    mDriveA.setServoTarget(targetAngle, targetAngularVelocity);
}

void UniversalJoint::setServoTargetB(f32 targetAngle, f32 targetAngularVelocity)
{
    mDriveB.setServoTarget(targetAngle, targetAngularVelocity);
}

void UniversalJoint::disableServoA()
{
    mDriveA.disable();
}

void UniversalJoint::disableServoB()
{
    mDriveB.disable();
}

f32 UniversalJoint::motorTorqueA() const
{
    return mDriveA.appliedForce();
}

f32 UniversalJoint::motorTorqueB() const
{
    return mDriveB.appliedForce();
}

f32 UniversalJoint::limitTorqueA() const
{
    return mPreviousDuration > 0.0f ? mTotalLimitImpulseA / mPreviousDuration : 0.0f;
}

f32 UniversalJoint::limitTorqueB() const
{
    return mPreviousDuration > 0.0f ? mTotalLimitImpulseB / mPreviousDuration : 0.0f;
}

Math::vec3 UniversalJoint::lockForce() const
{
    return mPreviousDuration > 0.0f ? mTotalPositionImpulse / mPreviousDuration : Math::vec3(0.0f);
}

Math::vec3 UniversalJoint::lockTorque() const
{
    if (!(mPreviousDuration > 0.0f))
        return Math::vec3(0.0f);
    return mPerpendicularAxis * (mTotalPerpendicularImpulse / mPreviousDuration);
}

Math::vec3 UniversalJoint::reactionForce() const
{
    return lockForce();
}

Math::vec3 UniversalJoint::reactionTorque() const
{
    return lockTorque() + mAxisA * (motorTorqueA() + limitTorqueA()) +
           mAxisB * (motorTorqueB() + limitTorqueB());
}

void UniversalJoint::calculatePositionProperties()
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

void UniversalJoint::calculatePerpendicularityProperties()
{
    mAxisA = Math::normalize(mBodyA->directionToWorld(mLocalAxisA));
    mAxisB = Math::normalize(mBodyB->directionToWorld(mLocalAxisB));

    const f32 k = Math::dot(mAxisA, mAxisB);
    const Math::vec3 axisBPerpendicular = mAxisB - k * mAxisA;
    const f32 length = Math::length(axisBPerpendicular);
    mPerpendicularAxis = length > 1.0e-6f ? Math::normalize(Math::cross(mAxisA, axisBPerpendicular))
                                          : normalizedPerpendicular(mAxisA);
    mPerpendicularity = k;

    const f32 inverseEffectiveMass = Math::dot(
        mPerpendicularAxis, mBodyA->inverseInertiaTensorWorld() * mPerpendicularAxis +
                                mBodyB->inverseInertiaTensorWorld() * mPerpendicularAxis);
    mPerpendicularEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mPerpendicularEffectiveMass == 0.0f)
        mTotalPerpendicularImpulse = 0.0f;
}

void UniversalJoint::calculateAngles()
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    mThetaA = rotationAngleAroundAxis(diff, mAxisA);
    mThetaB = rotationAngleAroundAxis(diff, mAxisB);
}

void UniversalJoint::calculateLimitProperties(bool hasLimits, f32 theta, f32 minAngle, f32 maxAngle,
                                              const Math::vec3& axis, bool& active,
                                              f32& effectiveMass)
{
    active = hasLimits && (theta <= minAngle || theta >= maxAngle);
    if (!active)
    {
        effectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        axis, mBodyA->inverseInertiaTensorWorld() * axis + mBodyB->inverseInertiaTensorWorld() * axis);
    effectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (effectiveMass == 0.0f)
        active = false;
}

void UniversalJoint::calculateDriveProperties(f32 duration)
{
    JointDrive* drives[2] = {&mDriveA, &mDriveB};
    f32* compensations[2] = {mDriveCompensationA, mDriveCompensationB};
    const Math::vec3 axes[2] = {mAxisA, mAxisB};
    const f32 angles[2] = {mThetaA, mThetaB};
    const bool hasLimits[2] = {mHasLimitsA, mHasLimitsB};
    const f32 minimum[2] = {mLimitsMinA, mLimitsMinB};
    const f32 maximum[2] = {mLimitsMaxA, mLimitsMaxB};
    bool wake = false;
    for (u32 i = 0; i < 2; ++i)
    {
        JointDrive& drive = *drives[i];
        if (!drive.enabled())
        {
            drive.prepare(duration, 0.0f, JointDrive::Coordinate());
            continue;
        }
        const Math::vec3& axis = axes[i];
        JointDrive::Coordinate coordinate;
        coordinate.position = angles[i];
        coordinate.velocity = Math::dot(axis, mBodyB->angularVelocity() - mBodyA->angularVelocity());
        coordinate.angular = true;
        coordinate.hasLimits = hasLimits[i];
        coordinate.minPosition = minimum[i];
        coordinate.maxPosition = maximum[i];
        const f32 inverseEffectiveMass = Math::dot(
            axis, mBodyA->inverseInertiaTensorWorld() * axis + mBodyB->inverseInertiaTensorWorld() * axis);
        // With the point lock and the perpendicularity row, as in HingeJoint. The
        // second matters: a servo on one axis of a shaft tilted about the other
        // pushes it about its own long, light axis, which only that row stops.
        detail::JacobianRow locks[4];
        detail::pointLockRows(mArmA, mArmB, locks);
        locks[3] = detail::angularRow(mPerpendicularAxis);
        const detail::DriveCoupling coupling = detail::coupleDriveToLocks(
            *mBodyA, *mBodyB, detail::angularRow(axis), locks, 4, compensations[i]);
        coordinate.gainInverseMass = coupling.gainInverseMass;
        drive.prepare(duration, inverseEffectiveMass > 1.0e-9f ? coupling.inverseMass : 0.0f,
                      coordinate);
        wake = wake || drive.wantsWake();
    }
    if (wake)
        wakeBodies();
}

void UniversalJoint::setup(f32 duration)
{
    calculatePositionProperties();
    calculatePerpendicularityProperties();
    calculateAngles();
    calculateLimitProperties(mHasLimitsA, mThetaA, mLimitsMinA, mLimitsMaxA, mAxisA, mLimitActiveA,
                             mLimitEffectiveMassA);
    calculateLimitProperties(mHasLimitsB, mThetaB, mLimitsMinB, mLimitsMaxB, mAxisB, mLimitActiveB,
                             mLimitEffectiveMassB);
    calculateDriveProperties(duration);
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionImpulse *= ratio;
        mTotalPerpendicularImpulse *= ratio;
        mTotalLimitImpulseA *= ratio;
        mTotalLimitImpulseB *= ratio;
    }
    else
    {
        mTotalPositionImpulse = Math::vec3(0.0f);
        mTotalPerpendicularImpulse = 0.0f;
        mTotalLimitImpulseA = 0.0f;
        mTotalLimitImpulseB = 0.0f;
    }
    // A limit not in contact this step starts from nothing next time instead of
    // warm starting its last impulse. Decided here only: the position pass
    // re-measures the limits too, and an in-range reading does not mean the
    // contact is gone.
    if (!mLimitActiveA)
        mTotalLimitImpulseA = 0.0f;
    if (!mLimitActiveB)
        mTotalLimitImpulseB = 0.0f;
    mPreviousDuration = duration;
}

void UniversalJoint::applyLinearImpulse(const Math::vec3& impulse)
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

void UniversalJoint::applyAngularImpulse(const Math::vec3& impulse)
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
void UniversalJoint::solveDrivePosition(f32 baumgarte)
{
    JointDrive* drives[2] = {&mDriveA, &mDriveB};
    for (u32 i = 0; i < 2; ++i)
    {
        if (!drives[i]->active())
            continue;
        calculatePerpendicularityProperties();
        calculateAngles();
        JointDrive::Coordinate coordinate;
        coordinate.position = i == 0 ? mThetaA : mThetaB;
        coordinate.angular = true;
        coordinate.hasLimits = i == 0 ? mHasLimitsA : mHasLimitsB;
        coordinate.minPosition = i == 0 ? mLimitsMinA : mLimitsMinB;
        coordinate.maxPosition = i == 0 ? mLimitsMaxA : mLimitsMaxB;
        f32 error = 0.0f;
        if (!drives[i]->positionError(coordinate, error))
            continue;
        const Math::vec3 axis = i == 0 ? mAxisA : mAxisB;
        const f32 inverseMass = Math::dot(
            axis, mBodyA->inverseInertiaTensorWorld() * axis + mBodyB->inverseInertiaTensorWorld() * axis);
        if (inverseMass > 1.0e-9f)
            detail::applyAngularPositionImpulse(*mBodyA, *mBodyB, axis * (-baumgarte * error / inverseMass));
    }
}

// Lock impulses that go with a drive impulse (detail::coupleDriveToLocks): the
// point lock's three, then the perpendicularity row.
void UniversalJoint::applyLockCompensation(const f32* compensation, f32 impulse)
{
    const Math::vec3 point = Math::vec3(compensation[0], compensation[1], compensation[2]) * impulse;
    const f32 perpendicular = compensation[3] * impulse;
    mTotalPositionImpulse += point;
    mTotalPerpendicularImpulse += perpendicular;
    applyLinearImpulse(point);
    applyAngularImpulse(mPerpendicularAxis * perpendicular);
}

void UniversalJoint::warmStart()
{
    if (mDriveA.active())
        applyAngularImpulse(mAxisA * mDriveA.totalImpulse());
    if (mDriveB.active())
        applyAngularImpulse(mAxisB * mDriveB.totalImpulse());
    applyLinearImpulse(mTotalPositionImpulse);
    applyAngularImpulse(mPerpendicularAxis * mTotalPerpendicularImpulse);
    if (mLimitActiveA)
        applyAngularImpulse(mAxisA * mTotalLimitImpulseA);
    if (mLimitActiveB)
        applyAngularImpulse(mAxisB * mTotalLimitImpulseB);
}

void UniversalJoint::solveVelocity()
{
    if (mDriveA.active())
    {
        const f32 velocity = Math::dot(mAxisA, mBodyB->angularVelocity() - mBodyA->angularVelocity());
        const f32 impulse = mDriveA.solve(velocity);
        applyAngularImpulse(mAxisA * impulse);
        applyLockCompensation(mDriveCompensationA, impulse);
    }
    if (mDriveB.active())
    {
        const f32 velocity = Math::dot(mAxisB, mBodyB->angularVelocity() - mBodyA->angularVelocity());
        const f32 impulse = mDriveB.solve(velocity);
        applyAngularImpulse(mAxisB * impulse);
        applyLockCompensation(mDriveCompensationB, impulse);
    }

    const Math::vec3 relativeVelocity =
        mBodyB->velocity() + Math::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - Math::cross(mBodyA->angularVelocity(), mArmA);
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyLinearImpulse(positionImpulse);

    const f32 perpJv =
        Math::dot(mPerpendicularAxis, mBodyA->angularVelocity() - mBodyB->angularVelocity());
    const f32 perpImpulse = mPerpendicularEffectiveMass * perpJv;
    mTotalPerpendicularImpulse += perpImpulse;
    applyAngularImpulse(mPerpendicularAxis * perpImpulse);

    if (mLimitActiveA)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMinA != mLimitsMaxA)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaA - mLimitsMinA);
            const f32 distanceToMax = centerAngleAroundZero(mThetaA - mLimitsMaxA);
            if (fabsf(distanceToMin) < fabsf(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mAxisA, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMassA * relative;
        const f32 previous = mTotalLimitImpulseA;
        mTotalLimitImpulseA = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisA * (mTotalLimitImpulseA - previous));
    }

    if (mLimitActiveB)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMinB != mLimitsMaxB)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaB - mLimitsMinB);
            const f32 distanceToMax = centerAngleAroundZero(mThetaB - mLimitsMaxB);
            if (fabsf(distanceToMin) < fabsf(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mAxisB, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMassB * relative;
        const f32 previous = mTotalLimitImpulseB;
        mTotalLimitImpulseB = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisB * (mTotalLimitImpulseB - previous));
    }
}

void UniversalJoint::solvePosition(f32 baumgarte)
{
    solveDrivePosition(baumgarte);

    calculatePositionProperties();
    const Math::vec3 pointA = mBodyA->position() + mArmA;
    const Math::vec3 pointB = mBodyB->position() + mArmB;
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculatePerpendicularityProperties();
    if (mPerpendicularity != 0.0f)
    {
        // Turning B by +d about mPerpendicularAxis (A by -d) lowers the dot of the
        // two axes by d, so the correction is +k: with -k the error grows by
        // (1 + baumgarte) every iteration and the cross folds flat. Same row as
        // WheelJoint's.
        const f32 lambda = mPerpendicularEffectiveMass * baumgarte * mPerpendicularity;
        if (mBodyA->isDynamic())
        {
            const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mPerpendicularAxis * -lambda;
            const Math::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mPerpendicularAxis * lambda;
            const Math::quat spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    if (mHasLimitsA || mHasLimitsB)
    {
        calculatePerpendicularityProperties();
        calculateAngles();
    }

    if (mHasLimitsA)
    {
        calculateLimitProperties(mHasLimitsA, mThetaA, mLimitsMinA, mLimitsMaxA, mAxisA,
                                 mLimitActiveA, mLimitEffectiveMassA);
        if (mLimitActiveA)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaA - mLimitsMinA);
            const f32 distanceToMax = centerAngleAroundZero(mThetaA - mLimitsMaxA);
            const f32 error = fabsf(distanceToMin) < fabsf(distanceToMax) ? distanceToMin
                                                                                 : distanceToMax;
            const f32 lambda = -mLimitEffectiveMassA * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mAxisA * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mAxisA * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }

    if (mHasLimitsB)
    {
        calculateLimitProperties(mHasLimitsB, mThetaB, mLimitsMinB, mLimitsMaxB, mAxisB,
                                 mLimitActiveB, mLimitEffectiveMassB);
        if (mLimitActiveB)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaB - mLimitsMinB);
            const f32 distanceToMax = centerAngleAroundZero(mThetaB - mLimitsMaxB);
            const f32 error = fabsf(distanceToMin) < fabsf(distanceToMax) ? distanceToMin
                                                                                 : distanceToMax;
            const f32 lambda = -mLimitEffectiveMassB * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mAxisB * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mAxisB * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
