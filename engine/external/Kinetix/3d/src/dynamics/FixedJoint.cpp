#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/FixedJoint.h"

#include "JointAxis.h"

#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

FixedJoint::FixedJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor)
    : FixedJoint(a, a.pointToLocal(worldAnchor), b, b.pointToLocal(worldAnchor))
{
}

FixedJoint::FixedJoint(RigidBody& a, const Math::vec3& localAnchorA, RigidBody& b,
                       const Math::vec3& localAnchorB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mInverseInitialOrientation(Math::conjugate(b.orientation()) * a.orientation())
{
}

RigidBody* FixedJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* FixedJoint::bodyB() const
{
    return mBodyB;
}

void FixedJoint::calculatePositionProperties()
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

void FixedJoint::calculateRotationProperties()
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

void FixedJoint::setup(f32 duration)
{
    calculatePositionProperties();
    calculateRotationProperties();
    if (mPreviousDuration > 0.0f)
    {
        mTotalPositionImpulse *= duration / mPreviousDuration;
        mTotalRotationImpulse *= duration / mPreviousDuration;
    }
    else
    {
        mTotalPositionImpulse = Math::vec3(0.0f);
        mTotalRotationImpulse = Math::vec3(0.0f);
    }
    mPreviousDuration = duration;
}

void FixedJoint::applyVelocityImpulse(const Math::vec3& impulse)
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

void FixedJoint::applyAngularVelocityImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void FixedJoint::warmStart()
{
    applyAngularVelocityImpulse(mTotalRotationImpulse);
    applyVelocityImpulse(mTotalPositionImpulse);
}

void FixedJoint::solveVelocity()
{
    const Math::vec3 rotationImpulse =
        mRotationEffectiveMass * (mBodyA->angularVelocity() - mBodyB->angularVelocity());
    mTotalRotationImpulse += rotationImpulse;
    applyAngularVelocityImpulse(rotationImpulse);

    const Math::vec3 relativeVelocity =
        mBodyB->velocity() + Math::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - Math::cross(mBodyA->angularVelocity(), mArmA);
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyVelocityImpulse(positionImpulse);
}

void FixedJoint::solvePosition(f32 baumgarte)
{
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

    calculatePositionProperties();
    const Math::vec3 pointA = mBodyA->position() + mArmA;
    const Math::vec3 pointB = mBodyB->position() + mArmB;
    const Math::vec3 impulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-impulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(impulse, pointB);
}

Math::vec3 FixedJoint::reactionForce() const
{
    return mPreviousDuration > 0.0f ? mTotalPositionImpulse / mPreviousDuration : Math::vec3(0.0f);
}

Math::vec3 FixedJoint::reactionTorque() const
{
    return mPreviousDuration > 0.0f ? mTotalRotationImpulse / mPreviousDuration : Math::vec3(0.0f);
}

u32 FixedJoint::articulationLocks(JointRow* rows, f32* errors)
{
    const Math::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    detail::pointLockRows(armA, armB, rows);
    rows[3] = detail::angularRow(Math::vec3(1.0f, 0.0f, 0.0f));
    rows[4] = detail::angularRow(Math::vec3(0.0f, 1.0f, 0.0f));
    rows[5] = detail::angularRow(Math::vec3(0.0f, 0.0f, 1.0f));
    if (errors)
    {
        const Math::vec3 separation = (mBodyB->position() + armB) - (mBodyA->position() + armA);
        Math::quat diff =
            mBodyB->orientation() * mInverseInitialOrientation * Math::conjugate(mBodyA->orientation());
        if (diff.w < 0.0f)
            diff = -diff;
        errors[0] = separation.x;
        errors[1] = separation.y;
        errors[2] = separation.z;
        errors[3] = 2.0f * diff.x;
        errors[4] = 2.0f * diff.y;
        errors[5] = 2.0f * diff.z;
    }
    return 6;
}

void FixedJoint::addArticulationLockImpulses(const f32* impulses)
{
    mTotalPositionImpulse += Math::vec3(impulses[0], impulses[1], impulses[2]);
    mTotalRotationImpulse += Math::vec3(impulses[3], impulses[4], impulses[5]);
}

u32 FixedJoint::articulationMotors(ArticulationMotor* motors)
{
    (void)motors;
    return 0;
}

void FixedJoint::setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass)
{
    (void)id;
    (void)inverseMass;
    (void)gainInverseMass;
}

f32 FixedJoint::solveArticulationMotor(u32 id, f32 velocity)
{
    (void)id;
    (void)velocity;
    return 0.0f;
}

u32 FixedJoint::articulationPositionRows(JointRow* rows, f32* errors)
{
    (void)rows;
    (void)errors;
    return 0;
}

}
