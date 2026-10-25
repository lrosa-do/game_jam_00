#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/DistanceJoint.h"

#include "JointAxis.h"

#include "dynamics/RigidBody.h"

namespace kinetix::Physics
{

DistanceJoint::DistanceJoint(RigidBody& a, const Math::vec3& worldAnchorA, RigidBody& b,
                             const Math::vec3& worldAnchorB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(a.pointToLocal(worldAnchorA)),
      mLocalAnchorB(b.pointToLocal(worldAnchorB))
{
    const f32 distance = Math::length(worldAnchorB - worldAnchorA);
    mMinDistance = distance;
    mMaxDistance = distance;
}

DistanceJoint::DistanceJoint(RigidBody& a, const Math::vec3& localAnchorA, RigidBody& b,
                             const Math::vec3& localAnchorB, f32 minDistance, f32 maxDistance)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB)
{
    setDistance(minDistance, maxDistance);
}

RigidBody* DistanceJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* DistanceJoint::bodyB() const
{
    return mBodyB;
}

const Math::vec3& DistanceJoint::localAnchorA() const
{
    return mLocalAnchorA;
}

const Math::vec3& DistanceJoint::localAnchorB() const
{
    return mLocalAnchorB;
}

Math::vec3 DistanceJoint::worldAnchorA() const
{
    return mBodyA->pointToWorld(mLocalAnchorA);
}

Math::vec3 DistanceJoint::worldAnchorB() const
{
    return mBodyB->pointToWorld(mLocalAnchorB);
}

void DistanceJoint::setDistance(f32 minDistance, f32 maxDistance)
{
    mMinDistance = Math::max(minDistance, 0.0f);
    mMaxDistance = Math::max(maxDistance, mMinDistance);
}

f32 DistanceJoint::minDistance() const
{
    return mMinDistance;
}

f32 DistanceJoint::maxDistance() const
{
    return mMaxDistance;
}

void DistanceJoint::calculateProperties()
{
    const Math::vec3 pointA = worldAnchorA();
    const Math::vec3 pointB = worldAnchorB();
    const Math::vec3 delta = pointB - pointA;
    const f32 length = Math::length(delta);
    if (length > 1.0e-6f)
        mWorldNormal = delta / length;

    mArmA = pointB - mBodyA->position();
    mArmB = pointB - mBodyB->position();

    if (mMinDistance == mMaxDistance)
    {
        mMinImpulse = -M_INFINITY;
        mMaxImpulse = M_INFINITY;
        mActive = true;
    }
    else if (length <= mMinDistance)
    {
        mMinImpulse = 0.0f;
        mMaxImpulse = M_INFINITY;
        mActive = true;
    }
    else if (length >= mMaxDistance)
    {
        mMinImpulse = -M_INFINITY;
        mMaxImpulse = 0.0f;
        mActive = true;
    }
    else
    {
        mActive = false;
    }

    if (!mActive)
    {
        mEffectiveMass = 0.0f;
        return;
    }

    // J = [-n, -(rA x n), n, rB x n], so the angular part of J M^-1 J^T is
    // (r x n) . I^-1 (r x n) per body, not n . I^-1 (r x n) (near zero, which
    // leaves the row far too stiff with the anchor off the centre of mass).
    const Math::vec3 armACrossNormal = Math::cross(mArmA, mWorldNormal);
    const Math::vec3 armBCrossNormal = Math::cross(mArmB, mWorldNormal);
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass +=
        Math::dot(armACrossNormal, mBodyA->inverseInertiaTensorWorld() * armACrossNormal);
    inverseEffectiveMass +=
        Math::dot(armBCrossNormal, mBodyB->inverseInertiaTensorWorld() * armBCrossNormal);
    mEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mEffectiveMass == 0.0f)
        mActive = false;
}

void DistanceJoint::setup(f32 duration)
{
    calculateProperties();
    // A rope that went slack starts from nothing when it pulls tight again.
    if (mPreviousDuration > 0.0f && mActive)
        mTotalImpulse *= duration / mPreviousDuration;
    else
        mTotalImpulse = 0.0f;
    mTotalImpulse = Math::clamp(mTotalImpulse, mMinImpulse, mMaxImpulse);
    mPreviousDuration = duration;
}

void DistanceJoint::applyVelocityImpulse(f32 impulse)
{
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass() * mWorldNormal);
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            impulse * (mBodyA->inverseInertiaTensorWorld() * Math::cross(mArmA, mWorldNormal)));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass() * mWorldNormal);
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            impulse * (mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, mWorldNormal)));
    }
}

void DistanceJoint::warmStart()
{
    if (mActive)
        applyVelocityImpulse(mTotalImpulse);
}

void DistanceJoint::solveVelocity()
{
    if (!mActive)
        return;
    const f32 relativeVelocity =
        Math::dot(mWorldNormal, mBodyA->velocity() - mBodyB->velocity()) +
        Math::dot(Math::cross(mArmA, mWorldNormal), mBodyA->angularVelocity()) -
        Math::dot(Math::cross(mArmB, mWorldNormal), mBodyB->angularVelocity());
    const f32 lambda = mEffectiveMass * relativeVelocity;
    const f32 previous = mTotalImpulse;
    mTotalImpulse = Math::clamp(previous + lambda, mMinImpulse, mMaxImpulse);
    applyVelocityImpulse(mTotalImpulse - previous);
}

void DistanceJoint::solvePosition(f32 baumgarte)
{
    calculateProperties();
    if (!mActive)
        return;
    const Math::vec3 pointA = worldAnchorA();
    const Math::vec3 pointB = worldAnchorB();
    const f32 distance = Math::dot(pointB - pointA, mWorldNormal);
    f32 error = 0.0f;
    if (distance < mMinDistance)
        error = distance - mMinDistance;
    else if (distance > mMaxDistance)
        error = distance - mMaxDistance;
    if (error == 0.0f)
        return;
    const f32 lambda = -mEffectiveMass * baumgarte * error;
    mBodyA->applyPositionImpulseAtPoint(-(lambda * mWorldNormal), pointB);
    mBodyB->applyPositionImpulseAtPoint(lambda * mWorldNormal, pointB);
}

f32 DistanceJoint::axialForce() const
{
    return mPreviousDuration > 0.0f ? mTotalImpulse / mPreviousDuration : 0.0f;
}

Math::vec3 DistanceJoint::reactionForce() const
{
    return mWorldNormal * axialForce();
}

// A rigid link (minimum and maximum equal) is a lock; a range is a bounded
// row while the joint is on one of its ends.
u32 DistanceJoint::articulationLocks(JointRow* rows, f32* errors)
{
    if (mMinDistance != mMaxDistance)
        return 0;
    const Math::vec3 pointA = worldAnchorA();
    const Math::vec3 pointB = worldAnchorB();
    const Math::vec3 delta = pointB - pointA;
    const f32 length = Math::length(delta);
    const Math::vec3 normal = length > 1.0e-6f ? delta / length : mWorldNormal;
    rows[0] = detail::linearRow(pointB - mBodyA->position(), pointB - mBodyB->position(), normal);
    if (errors)
        errors[0] = length - mMinDistance;
    return 1;
}

void DistanceJoint::addArticulationLockImpulses(const f32* impulses)
{
    mTotalImpulse += impulses[0];
}

u32 DistanceJoint::articulationMotors(ArticulationMotor* motors)
{
    if (mMinDistance == mMaxDistance || !mActive)
        return 0;
    motors[0].row = detail::linearRow(mArmA, mArmB, mWorldNormal);
    motors[0].id = 0;
    return 1;
}

void DistanceJoint::setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass)
{
    (void)id;
    (void)gainInverseMass;
    if (inverseMass > 1.0e-9f)
        mEffectiveMass = 1.0f / inverseMass;
}

f32 DistanceJoint::solveArticulationMotor(u32 id, f32 velocity)
{
    (void)id;
    const f32 previous = mTotalImpulse;
    mTotalImpulse = Math::clamp(previous - mEffectiveMass * velocity, mMinImpulse, mMaxImpulse);
    return mTotalImpulse - previous;
}

u32 DistanceJoint::articulationPositionRows(JointRow* rows, f32* errors)
{
    if (mMinDistance == mMaxDistance)
        return 0;
    const Math::vec3 pointA = worldAnchorA();
    const Math::vec3 pointB = worldAnchorB();
    const Math::vec3 delta = pointB - pointA;
    const f32 length = Math::length(delta);
    f32 error = 0.0f;
    if (length < mMinDistance)
        error = length - mMinDistance;
    else if (length > mMaxDistance)
        error = length - mMaxDistance;
    if (error == 0.0f)
        return 0;
    const Math::vec3 normal = length > 1.0e-6f ? delta / length : mWorldNormal;
    rows[0] = detail::linearRow(pointB - mBodyA->position(), pointB - mBodyB->position(), normal);
    errors[0] = error;
    return 1;
}

}
