#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/ConeTwistJoint.h"

#include "JointAxis.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

void swingTwistDecompose(const Math::quat& q, Math::quat& outSwing, Math::quat& outTwist)
{
    const f32 x = q.x, y = q.y, z = q.z, w = q.w;
    const f32 s = sqrtf(w * w + x * x);
    if (s != 0.0f)
    {
        outTwist = Math::quat(w / s, x / s, 0.0f, 0.0f);
        outSwing = Math::quat(s, 0.0f, (w * y - x * z) / s, (w * z + x * y) / s);
    }
    else
    {
        outTwist = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
        outSwing = q;
    }
}

// Below this the sine of a half swing angle is taken as zero: the cone has
// collapsed onto a line or point and the ellipse search would divide by next
// to nothing. 1e-4 is about 0.01 degrees.
const f32 kMinSinHalfSwing = 1.0e-4f;

// Newton on g(t) = (a x / (t + a^2))^2 + (b y / (t + b^2))^2 - 1 for a point
// outside the ellipse. g falls and is convex for t >= 0, so Newton started left
// of the root climbs to it without overshooting. a|x| - a^2 and b|y| - b^2 each
// make their own term exactly 1, so g >= 0 at the larger of them; starting
// there skips the slow climb a needle-thin ellipse would need.
Math::vec2 ellipseClosestPoint(const Math::vec2& point, f32 a, f32 b)
{
    const f32 aSq = a * a;
    const f32 bSq = b * b;
    f32 t = Math::max(0.0f, Math::max(a * fabsf(point.x) - aSq, b * fabsf(point.y) - bSq));
    for (u32 iteration = 0; iteration < 32; ++iteration)
    {
        const f32 tPlusASq = t + aSq;
        const f32 tPlusBSq = t + bSq;
        const f32 gt = ((a * point.x) / tPlusASq) * ((a * point.x) / tPlusASq) +
                       ((b * point.y) / tPlusBSq) * ((b * point.y) / tPlusBSq) - 1.0f;
        if (fabsf(gt) < 1.0e-6f)
            return Math::vec2(aSq * point.x / tPlusASq, bSq * point.y / tPlusBSq);
        const f32 gtAccent =
                -2.0f * (aSq * point.x * point.x / (tPlusASq * tPlusASq * tPlusASq) +
                                bSq * point.y * point.y / (tPlusBSq * tPlusBSq * tPlusBSq));
        t = t - gt / gtAccent;
    }
    return Math::vec2(aSq * point.x / (t + aSq), bSq * point.y / (t + bSq));
}

bool clampSwingCone(Math::quat& ioSwing, f32 halfAngleY, f32 halfAngleZ)
{
    const f32 sinHalfY = sinf(0.5f * halfAngleY);
    const f32 sinHalfZ = sinf(0.5f * halfAngleZ);
    const Math::vec2 point(ioSwing.y, ioSwing.z);
    Math::vec2 closest;
    if (sinHalfY > kMinSinHalfSwing && sinHalfZ > kMinSinHalfSwing)
    {
        if ((point.x / sinHalfY) * (point.x / sinHalfY) +
                        (point.y / sinHalfZ) * (point.y / sinHalfZ) <=
                1.0f)
            return false;
        closest = ellipseClosestPoint(point, sinHalfY, sinHalfZ);
    }
    else
    {
        // A zero half-angle locks that swing direction: the ellipse is a segment
        // (or a point), whose closest point is a clamp. The ellipse search would
        // divide by the zero sine and its NaN axis read as "no limit".
        closest = Math::vec2(Math::clamp(point.x, -sinHalfY, sinHalfY),
                Math::clamp(point.y, -sinHalfZ, sinHalfZ));
        if (closest.x == point.x && closest.y == point.y) return false;
    }
    const f32 w = sqrtf(Math::max(0.0f, 1.0f - closest.x * closest.x - closest.y * closest.y));
    ioSwing = Math::quat(w, 0.0f, closest.x, closest.y);
    return true;
}

bool clampTwist(Math::quat& ioTwist, f32 sinHalfMin, f32 sinHalfMax, f32 cosHalfMin, f32 cosHalfMax,
        bool& outMinClosest)
{
    f32 deltaMin = sinHalfMin - ioTwist.x;
    f32 deltaMax = ioTwist.x - sinHalfMax;
    if (deltaMin <= 0.0f && deltaMax <= 0.0f) return false;
    deltaMin = fabsf(deltaMin);
    if (deltaMin > 1.0f) deltaMin = 2.0f - deltaMin;
    deltaMax = fabsf(deltaMax);
    if (deltaMax > 1.0f) deltaMax = 2.0f - deltaMax;
    outMinClosest = deltaMin < deltaMax;
    ioTwist = outMinClosest ? Math::quat(cosHalfMin, sinHalfMin, 0.0f, 0.0f)
                            : Math::quat(cosHalfMax, sinHalfMax, 0.0f, 0.0f);
    return true;
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

Math::quat orientationXZ(const Math::vec3& xAxis, const Math::vec3& zAxis)
{
    return Math::normalize(Math::quat_cast(Math::mat3(xAxis, Math::cross(zAxis, xAxis), zAxis)));
}

// Rotation vector (axis times angle, angle in [0, pi]) of a unit quaternion.
Math::vec3 rotationVector(Math::quat q)
{
    if (q.w < 0.0f)
        q = Math::quat(-q.w, -q.x, -q.y, -q.z);
    const Math::vec3 v(q.x, q.y, q.z);
    const f32 sine = Math::length(v);
    if (sine < 1.0e-7f)
        return v * 2.0f;
    const f32 angle = 2.0f * atan2f(sine, q.w);
    return v * (angle / sine);
}

Math::vec3 clampLength(const Math::vec3& v, f32 maxLength)
{
    const f32 length = Math::length(v);
    if (length > maxLength && length > 0.0f)
        return v * (maxLength / length);
    return v;
}

} // namespace

ConeTwistJoint::ConeTwistJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
        const Math::vec3& worldTwistAxis, const Math::vec3& worldPlaneAxis)
        : mBodyA(&a),
          mBodyB(&b)
{
    configure(a, b, worldAnchor, worldTwistAxis, worldPlaneAxis);
}

ConeTwistJoint::ConeTwistJoint(RigidBody& a, const Math::vec3& localAnchorA,
        const Math::vec3& localTwistAxisA, const Math::vec3& localPlaneAxisA, RigidBody& b,
        const Math::vec3& localAnchorB, const Math::vec3& localTwistAxisB,
        const Math::vec3& localPlaneAxisB)
        : mBodyA(&a),
          mBodyB(&b),
          mLocalAnchorA(localAnchorA),
          mLocalAnchorB(localAnchorB),
          mLocalTwistAxisA(detail::normalizedAxisOr(localTwistAxisA, Math::vec3(1.0f, 0.0f, 0.0f))),
          mLocalPlaneAxisA(detail::normalizedAxisOr(localPlaneAxisA, Math::vec3(0.0f, 0.0f, 1.0f))),
          mLocalTwistAxisB(detail::normalizedAxisOr(localTwistAxisB, Math::vec3(1.0f, 0.0f, 0.0f))),
          mLocalPlaneAxisB(detail::normalizedAxisOr(localPlaneAxisB, Math::vec3(0.0f, 0.0f, 1.0f)))
{
}

void ConeTwistJoint::configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
        const Math::vec3& worldTwistAxis, const Math::vec3& worldPlaneAxis)
{
    mBodyA = &a;
    mBodyB = &b;
    mLocalAnchorA = a.pointToLocal(worldAnchor);
    mLocalAnchorB = b.pointToLocal(worldAnchor);
    const Math::vec3 twistAxis =
            detail::normalizedAxisOr(worldTwistAxis, Math::vec3(1.0f, 0.0f, 0.0f));
    Math::vec3 planeAxis = detail::normalizedAxisOr(worldPlaneAxis, Math::vec3(0.0f, 0.0f, 1.0f));
    const f32 dot = Math::dot(twistAxis, planeAxis);
    if (fabsf(dot) > 0.999f) planeAxis = normalizedPerpendicular(twistAxis);
    else
        planeAxis = Math::normalize(planeAxis - dot * twistAxis);
    mLocalTwistAxisA = Math::normalize(a.directionToLocal(twistAxis));
    mLocalPlaneAxisA = Math::normalize(a.directionToLocal(planeAxis));
    mLocalTwistAxisB = Math::normalize(b.directionToLocal(twistAxis));
    mLocalPlaneAxisB = Math::normalize(b.directionToLocal(planeAxis));
}

RigidBody* ConeTwistJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* ConeTwistJoint::bodyB() const
{
    return mBodyB;
}

Math::vec3 ConeTwistJoint::anchorWorldA() const
{
    return mBodyA->pointToWorld(mLocalAnchorA);
}

Math::vec3 ConeTwistJoint::anchorWorldB() const
{
    return mBodyB->pointToWorld(mLocalAnchorB);
}

Math::vec3 ConeTwistJoint::axisWorld() const
{
    return Math::normalize(mBodyA->directionToWorld(mLocalTwistAxisA));
}

void ConeTwistJoint::setSwingLimits(f32 halfAngleY, f32 halfAngleZ)
{
    mSwingHalfAngleY = Math::clamp(halfAngleY, 0.0f, Math::pi<f32>());
    mSwingHalfAngleZ = Math::clamp(halfAngleZ, 0.0f, Math::pi<f32>());
}

void ConeTwistJoint::setTwistLimits(f32 minAngle, f32 maxAngle)
{
    mTwistMinAngle = Math::clamp(minAngle, -Math::pi<f32>(), Math::pi<f32>());
    mTwistMaxAngle =
            Math::max(Math::clamp(maxAngle, -Math::pi<f32>(), Math::pi<f32>()), mTwistMinAngle);
}

void ConeTwistJoint::resetWarmStart()
{
    mTotalPositionImpulse = Math::vec3(0.0f);
    mTotalSwingLimitImpulse = 0.0f;
    mTotalTwistLimitImpulse = 0.0f;
    mTotalServoImpulse = Math::vec3(0.0f);
    mPreviousDuration = 0.0f;
}

Math::quat ConeTwistJoint::relativeOrientation() const
{
    const Math::vec3 twistAxis = Math::normalize(mBodyA->directionToWorld(mLocalTwistAxisA));
    const Math::vec3 planeAxis = Math::normalize(mBodyA->directionToWorld(mLocalPlaneAxisA));
    const Math::quat constraintA = orientationXZ(twistAxis, planeAxis);
    const Math::quat constraintB =
        Math::normalize(mBodyB->orientation() * orientationXZ(mLocalTwistAxisB, mLocalPlaneAxisB));
    Math::quat q = Math::normalize(Math::conjugate(constraintA) * constraintB);
    if (q.w < 0.0f)
        q = Math::quat(-q.w, -q.x, -q.y, -q.z);
    return q;
}

f32 ConeTwistJoint::twistAngle() const
{
    Math::quat swing, twist;
    swingTwistDecompose(relativeOrientation(), swing, twist);
    if (twist.w < 0.0f)
        twist = Math::quat(-twist.w, -twist.x, -twist.y, -twist.z);
    return 2.0f * atan2f(twist.x, twist.w);
}

Math::vec2 ConeTwistJoint::swingAngles() const
{
    Math::quat swing, twist;
    swingTwistDecompose(relativeOrientation(), swing, twist);
    const Math::vec3 vector = rotationVector(swing);
    return Math::vec2(vector.y, vector.z);
}

Math::quat ConeTwistJoint::orientationFromTwistSwing(f32 twist, f32 swingY, f32 swingZ)
{
    const Math::quat twistRotation(cosf(0.5f * twist), sinf(0.5f * twist), 0.0f, 0.0f);
    const f32 swingAngle = sqrtf(swingY * swingY + swingZ * swingZ);
    Math::quat swingRotation(1.0f, 0.0f, 0.0f, 0.0f);
    if (swingAngle > 1.0e-9f)
    {
        const f32 scale = sinf(0.5f * swingAngle) / swingAngle;
        swingRotation = Math::quat(cosf(0.5f * swingAngle), 0.0f, swingY * scale, swingZ * scale);
    }
    return Math::normalize(swingRotation * twistRotation);
}

void ConeTwistJoint::setServo(const Math::quat& target, const ServoSettings& settings)
{
    const f32 values[] = {target.w, target.x, target.y, target.z, settings.frequency,
                          settings.dampingRatio, settings.stiffness, settings.damping,
                          settings.maxForce, settings.maxVelocity};
    for (f32 value : values)
        if (!isfinite(value))
            return;
    if (Math::length(target) < 1.0e-6f)
        return;
    mServoSettings.frequency = Math::max(settings.frequency, 0.0f);
    mServoSettings.dampingRatio = Math::max(settings.dampingRatio, 0.0f);
    mServoSettings.stiffness = Math::max(settings.stiffness, 0.0f);
    mServoSettings.damping = Math::max(settings.damping, 0.0f);
    mServoSettings.maxForce = Math::max(settings.maxForce, 0.0f);
    mServoSettings.maxVelocity = Math::max(settings.maxVelocity, 0.0f);
    const bool enabled = mServoSettings.maxForce > 0.0f;
    if (!enabled)
    {
        disableServo();
        return;
    }
    const Math::quat normalized = Math::normalize(target);
    if (!mServoEnabled || normalized.w != mServoTarget.w || normalized.x != mServoTarget.x ||
        normalized.y != mServoTarget.y || normalized.z != mServoTarget.z)
        mServoTargetChanged = true;
    mServoTarget = normalized;
    mServoEnabled = true;
}

void ConeTwistJoint::setServoTarget(const Math::quat& target)
{
    if (!mServoEnabled || !isfinite(target.w) || !isfinite(target.x) || !isfinite(target.y) ||
        !isfinite(target.z) || Math::length(target) < 1.0e-6f)
        return;
    const Math::quat normalized = Math::normalize(target);
    if (normalized.w != mServoTarget.w || normalized.x != mServoTarget.x ||
        normalized.y != mServoTarget.y || normalized.z != mServoTarget.z)
        mServoTargetChanged = true;
    mServoTarget = normalized;
}

void ConeTwistJoint::disableServo()
{
    mServoEnabled = false;
    mServoActive = false;
    mTotalServoImpulse = Math::vec3(0.0f);
    mServoError = Math::vec3(0.0f);
}

Math::vec3 ConeTwistJoint::servoTorque() const
{
    return mPreviousDuration > 0.0f && mServoEnabled ? mTotalServoImpulse / mPreviousDuration
                                                     : Math::vec3(0.0f);
}

Math::vec3 ConeTwistJoint::swingLimitTorque() const
{
    return mPreviousDuration > 0.0f
               ? mWorldSwingLimitAxis * (mTotalSwingLimitImpulse / mPreviousDuration)
               : Math::vec3(0.0f);
}

Math::vec3 ConeTwistJoint::twistLimitTorque() const
{
    return mPreviousDuration > 0.0f
               ? mWorldTwistLimitAxis * (mTotalTwistLimitImpulse / mPreviousDuration)
               : Math::vec3(0.0f);
}

Math::vec3 ConeTwistJoint::lockForce() const
{
    return mPreviousDuration > 0.0f ? mTotalPositionImpulse / mPreviousDuration : Math::vec3(0.0f);
}

Math::vec3 ConeTwistJoint::reactionForce() const
{
    return lockForce();
}

Math::vec3 ConeTwistJoint::reactionTorque() const
{
    return servoTorque() + swingLimitTorque() + twistLimitTorque();
}

// The servo is one three-row block on the relative angular velocity: the error
// is the rotation vector taking B's constraint frame from the target to where it
// is, so driving it to zero with wB - wA = -bias turns B the shortest way. It
// is JointDrive's implicit spring-damper with a 3x3 effective mass.
void ConeTwistJoint::calculateServoProperties(f32 duration)
{
    mServoActive = false;
    if (!mServoEnabled || !(duration > 0.0f))
    {
        mTotalServoImpulse = Math::vec3(0.0f);
        mServoTargetChanged = false;
        return;
    }
    const Math::quat constraintB =
        Math::normalize(mBodyB->orientation() * orientationXZ(mLocalTwistAxisB, mLocalPlaneAxisB));
    const Math::quat desiredB = mConstraintToWorld * mServoTarget;
    const Math::vec3 error = rotationVector(constraintB * Math::conjugate(desiredB));
    mServoError = error;

    // As HingeJoint's drive (detail::coupleDriveToLocks) for three rows at once:
    // the axes drive the Schur complement M^-1 - C^T P C, column j of C being how
    // a unit angular impulse about axis j moves the anchors apart (a limb pinned
    // at its socket turns with its inertia about the socket). Gains are always
    // scaled by it; the block is solved as one (servo impulse carrying the lock
    // impulse that keeps the socket still) only against an immovable body.
    const Math::mat3 rowInverseMass =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    if (!detail::invertible(rowInverseMass, Math::determinant(rowInverseMass)))
    {
        mTotalServoImpulse = Math::vec3(0.0f);
        mServoTargetChanged = false;
        return;
    }
    Math::mat3 coupling(0.0f);
    const Math::vec3 unitAxes[] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                                   Math::vec3(0.0f, 0.0f, 1.0f)};
    for (u32 j = 0; j < 3; ++j)
        coupling[j] = detail::pointCoupling(mBodyA->inverseInertiaTensorWorld(),
                                            mBodyB->inverseInertiaTensorWorld(), mArmA, mArmB,
                                            unitAxes[j]);
    Math::mat3 gainInverseMass =
        rowInverseMass - Math::transpose(coupling) * mPositionEffectiveMass * coupling;
    if (!detail::invertible(gainInverseMass, Math::determinant(gainInverseMass)))
        gainInverseMass = rowInverseMass;
    const bool block = !mBodyA->isDynamic() || !mBodyB->isDynamic();
    const Math::mat3 solveInverseMass = block ? gainInverseMass : rowInverseMass;
    mServoCompensation = block ? (mPositionEffectiveMass * coupling) * -1.0f : Math::mat3(0.0f);
    const Math::mat3 mass = Math::inverse(solveInverseMass);

    if (mPreviousDuration > 0.0f)
        mTotalServoImpulse *= duration / mPreviousDuration;
    else
        mTotalServoImpulse = Math::vec3(0.0f);
    mServoMaxImpulse = mServoSettings.maxForce * duration;
    mTotalServoImpulse = clampLength(mTotalServoImpulse, mServoMaxImpulse);

    Math::mat3 softness(0.0f);
    // Rigid: a fifth of the error at the velocity level, the rest in the position pass.
    Math::vec3 bias = error * (0.2f / duration);
    Math::mat3 effectiveMass = mass;
    // Spring compliance, to read the distance from the held equilibrium: e + K^-1 * F.
    Math::mat3 compliance(0.0f);
    mServoPositionCorrectable = !(mServoSettings.frequency > 0.0f) && !(mServoSettings.stiffness > 0.0f);
    if (mServoSettings.frequency > 0.0f)
    {
        // K = M w^2 and C = 2 zeta w M, M the gain mass: gamma = g * M^-1 with
        // g = 1 / (h w (2 zeta + h w)) and bias h w^2 g e, the same response
        // whatever the load.
        const f32 omega = Math::two_pi<f32>() * mServoSettings.frequency;
        const f32 g =
            1.0f / (duration * omega * (2.0f * mServoSettings.dampingRatio + duration * omega));
        softness = gainInverseMass * g;
        bias = error * (duration * omega * omega * g);
        compliance = gainInverseMass * (1.0f / (omega * omega));
    }
    else if (mServoSettings.stiffness > 0.0f)
    {
        const f32 k = mServoSettings.stiffness;
        const f32 denominator = mServoSettings.damping + duration * k;
        softness = Math::mat3(1.0f / (duration * denominator));
        bias = error * (k / denominator);
        compliance = Math::mat3(1.0f / k);
    }
    if (mServoSettings.frequency > 0.0f || mServoSettings.stiffness > 0.0f)
    {
        const Math::mat3 softInverse = solveInverseMass + softness;
        effectiveMass = detail::invertible(softInverse, Math::determinant(softInverse))
                            ? Math::inverse(softInverse)
                            : Math::mat3(0.0f);
    }
    mServoDesiredVelocity = -bias;
    // The speed the soft block would end at holding last step's torque; if too
    // fast, it slews at the rated speed as a stiff velocity block.
    const Math::vec3 predicted = mServoDesiredVelocity - softness * mTotalServoImpulse;
    const f32 speed = Math::length(predicted);
    if (mServoSettings.maxVelocity > 0.0f && speed > mServoSettings.maxVelocity)
    {
        mServoPositionCorrectable = false;
        mServoDesiredVelocity = predicted * (mServoSettings.maxVelocity / speed);
        softness = Math::mat3(0.0f);
        effectiveMass = mass;
    }
    mServoSoftness = softness;
    mServoEffectiveMass = effectiveMass;

    // Awake while there is work left (see JointDrive): a new target or a distance
    // to the held equilibrium, unless saturated.
    const Math::vec3 held = mTotalServoImpulse / duration;
    const Math::vec3 toEquilibrium = error + compliance * held;
    const bool saturated = Math::length(mTotalServoImpulse) >= mServoMaxImpulse * (1.0f - 1.0e-5f);
    if (mServoTargetChanged || (Math::length(toEquilibrium) > 1.0e-3f && !saturated))
        wakeBodies();
    mServoTargetChanged = false;
    mServoActive = true;
}

void ConeTwistJoint::calculatePositionProperties()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    Math::mat3 inverseEffectiveMass(0.0f);
    const Math::vec3 axes[] = { Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
        Math::vec3(0.0f, 0.0f, 1.0f) };
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

void ConeTwistJoint::calculateConstraintFrame()
{
    mWorldTwistAxis = Math::normalize(mBodyA->directionToWorld(mLocalTwistAxisA));
    mWorldPlaneAxis = Math::normalize(mBodyA->directionToWorld(mLocalPlaneAxisA));
    mWorldNormalAxis = Math::normalize(Math::cross(mWorldPlaneAxis, mWorldTwistAxis));
    mConstraintToWorld = orientationXZ(mWorldTwistAxis, mWorldPlaneAxis);
}

void ConeTwistJoint::calculateLimitEffectiveMass(const Math::vec3& axis, f32& effectiveMass) const
{
    const f32 inverseEffectiveMass =
            Math::dot(axis, mBodyA->inverseInertiaTensorWorld() * axis +
                                    mBodyB->inverseInertiaTensorWorld() * axis);
    effectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void ConeTwistJoint::calculateSwingTwistProperties()
{
    calculateConstraintFrame();

    const Math::quat constraintBToWorld = Math::normalize(
            mBodyB->orientation() * orientationXZ(mLocalTwistAxisB, mLocalPlaneAxisB));
    const Math::quat q = Math::normalize(Math::conjugate(mConstraintToWorld) * constraintBToWorld);

    Math::quat swing, twist;
    swingTwistDecompose(q, swing, twist);

    if (swing.w < 0.0f) swing = Math::quat(-swing.w, -swing.x, -swing.y, -swing.z);
    if (twist.w < 0.0f) twist = Math::quat(-twist.w, -twist.x, -twist.y, -twist.z);

    mSwing = swing;
    mTwist = twist;

    const Math::quat originalSwing = swing;
    const bool swingClamped = mSwingHalfAngleY < Math::pi<f32>() * 0.999f ||
                                              mSwingHalfAngleZ < Math::pi<f32>() * 0.999f
                                      ? clampSwingCone(swing, mSwingHalfAngleY, mSwingHalfAngleZ)
                                      : false;

    mSwingActive = swingClamped;
    if (swingClamped)
    {
        const Math::quat twistToWorldCurrent = mConstraintToWorld * originalSwing;
        const Math::quat twistToWorldDesired = mConstraintToWorld * swing;
        const Math::vec3 currentAxis = twistToWorldCurrent * Math::vec3(1.0f, 0.0f, 0.0f);
        const Math::vec3 desiredAxis = twistToWorldDesired * Math::vec3(1.0f, 0.0f, 0.0f);
        const Math::vec3 limitAxis = Math::cross(desiredAxis, currentAxis);
        const f32 len = Math::length(limitAxis);
        if (len > 1.0e-9f)
        {
            mWorldSwingLimitAxis = limitAxis / len;
            calculateLimitEffectiveMass(mWorldSwingLimitAxis, mSwingLimitEffectiveMass);
            if (mSwingLimitEffectiveMass == 0.0f) mSwingActive = false;
        }
        else
            mSwingActive = false;
    }
    if (!mSwingActive) mSwingLimitEffectiveMass = 0.0f;

    // A range under a degree is a lock held at its middle from both sides: one
    // row pushing either way (see solveVelocity()), not two one-sided stops
    // that would chatter.
    const bool twistLocked = mTwistMaxAngle - mTwistMinAngle < Math::radians(1.0f);
    const bool twistFree =
            mTwistMinAngle <= -Math::radians(179.5f) && mTwistMaxAngle >= Math::radians(179.5f);
    bool twistClamped = false;
    bool minClosest = true;
    if (twistLocked)
    {
        twistClamped = twist.x != sinf(0.5f * twistLockAngle());
        minClosest = false;
    }
    else if (!twistFree)
    {
        const f32 sinHalfMin = sinf(0.5f * mTwistMinAngle);
        const f32 sinHalfMax = sinf(0.5f * mTwistMaxAngle);
        const f32 cosHalfMin = cosf(0.5f * mTwistMinAngle);
        const f32 cosHalfMax = cosf(0.5f * mTwistMaxAngle);
        twistClamped =
                clampTwist(twist, sinHalfMin, sinHalfMax, cosHalfMin, cosHalfMax, minClosest);
    }

    mTwistActive = twistClamped;
    mTwistMinClosest = minClosest;
    if (twistClamped)
    {
        const Math::quat twistToWorld = mConstraintToWorld * originalSwing;
        Math::vec3 limitAxis = twistToWorld * Math::vec3(1.0f, 0.0f, 0.0f);
        if (minClosest) limitAxis = -limitAxis;
        mWorldTwistLimitAxis = limitAxis;
        calculateLimitEffectiveMass(mWorldTwistLimitAxis, mTwistLimitEffectiveMass);
        if (mTwistLimitEffectiveMass == 0.0f) mTwistActive = false;
    }
    if (!mTwistActive) mTwistLimitEffectiveMass = 0.0f;
    mTwistLocked = twistLocked;
}

f32 ConeTwistJoint::twistLockAngle() const
{
    return 0.5f * (mTwistMinAngle + mTwistMaxAngle);
}

void ConeTwistJoint::setup(f32 duration)
{
    calculatePositionProperties();
    calculateSwingTwistProperties();
    calculateServoProperties(duration);
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionImpulse *= ratio;
        mTotalSwingLimitImpulse *= ratio;
        mTotalTwistLimitImpulse *= ratio;
    }
    else
    {
        mTotalPositionImpulse = Math::vec3(0.0f);
        mTotalSwingLimitImpulse = 0.0f;
        mTotalTwistLimitImpulse = 0.0f;
    }
    // Whether a limit's impulse carries into this step is decided here only:
    // solvePosition() re-measures after the bodies moved, and an in-range reading
    // there does not mean the contact is gone.
    if (!mSwingActive) mTotalSwingLimitImpulse = 0.0f;
    if (!mTwistActive) mTotalTwistLimitImpulse = 0.0f;
    mPreviousDuration = duration;
}

void ConeTwistJoint::applyLinearImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass());
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() - mBodyA->inverseInertiaTensorWorld() *
                                                                       Math::cross(mArmA, impulse));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass());
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() + mBodyB->inverseInertiaTensorWorld() *
                                                                       Math::cross(mArmB, impulse));
    }
}

void ConeTwistJoint::applyAngularImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(
                mBodyA->angularVelocity() - mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(
                mBodyB->angularVelocity() + mBodyB->inverseInertiaTensorWorld() * impulse);
}

void ConeTwistJoint::warmStart()
{
    if (mServoActive)
        applyAngularImpulse(mTotalServoImpulse);
    applyLinearImpulse(mTotalPositionImpulse);
    if (mSwingActive) applyAngularImpulse(mWorldSwingLimitAxis * mTotalSwingLimitImpulse);
    if (mTwistActive) applyAngularImpulse(mWorldTwistLimitAxis * mTotalTwistLimitImpulse);
}

void ConeTwistJoint::solveVelocity()
{
    if (mServoActive)
    {
        const Math::vec3 velocity = mBodyB->angularVelocity() - mBodyA->angularVelocity();
        const Math::vec3 impulse =
            -(mServoEffectiveMass *
              (velocity - mServoDesiredVelocity + mServoSoftness * mTotalServoImpulse));
        const Math::vec3 previous = mTotalServoImpulse;
        mTotalServoImpulse = clampLength(previous + impulse, mServoMaxImpulse);
        const Math::vec3 applied = mTotalServoImpulse - previous;
        applyAngularImpulse(applied);
        const Math::vec3 lock = mServoCompensation * applied;
        mTotalPositionImpulse += lock;
        applyLinearImpulse(lock);
    }

    const Math::vec3 relativeVelocity =
            mBodyB->velocity() + Math::cross(mBodyB->angularVelocity(), mArmB) -
            mBodyA->velocity() - Math::cross(mBodyA->angularVelocity(), mArmA);
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyLinearImpulse(positionImpulse);

    if (mSwingActive)
    {
        const f32 relative = Math::dot(mWorldSwingLimitAxis,
                mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mSwingLimitEffectiveMass * relative;
        const f32 previous = mTotalSwingLimitImpulse;
        mTotalSwingLimitImpulse = Math::min(previous + impulse, 0.0f);
        applyAngularImpulse(mWorldSwingLimitAxis * (mTotalSwingLimitImpulse - previous));
    }

    if (mTwistActive)
    {
        const f32 relative = Math::dot(mWorldTwistLimitAxis,
                mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mTwistLimitEffectiveMass * relative;
        const f32 previous = mTotalTwistLimitImpulse;
        mTotalTwistLimitImpulse =
                mTwistLocked ? previous + impulse : Math::min(previous + impulse, 0.0f);
        applyAngularImpulse(mWorldTwistLimitAxis * (mTotalTwistLimitImpulse - previous));
    }
}

void ConeTwistJoint::rotateBody(RigidBody& body, const Math::vec3& step) const
{
    const Math::quat spin(0.0f, step.x, step.y, step.z);
    body.setOrientation(body.orientation() + 0.5f * spin * body.orientation());
}

void ConeTwistJoint::solvePosition(f32 baumgarte)
{
    // A rigid servo's error, closed with the joint's other rows (see
    // JointDrive::positionError), unless saturated.
    if (mServoActive && mServoPositionCorrectable &&
        Math::length(mTotalServoImpulse) < mServoMaxImpulse * (1.0f - 1.0e-5f))
    {
        calculateConstraintFrame();
        const Math::quat constraintB =
            Math::normalize(mBodyB->orientation() * orientationXZ(mLocalTwistAxisB, mLocalPlaneAxisB));
        const Math::vec3 error =
            rotationVector(constraintB * Math::conjugate(mConstraintToWorld * mServoTarget));
        const Math::mat3 inverseMass =
            mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
        if (detail::invertible(inverseMass, Math::determinant(inverseMass)))
            detail::applyAngularPositionImpulse(*mBodyA, *mBodyB,
                                                Math::inverse(inverseMass) * (error * -baumgarte));
    }

    calculatePositionProperties();
    const Math::vec3 pointA = mBodyA->position() + mArmA;
    const Math::vec3 pointB = mBodyB->position() + mArmB;
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculateSwingTwistProperties();

    if (mSwingActive)
    {
        const Math::quat originalSwing = mSwing;
        Math::quat clampedSwing = originalSwing;
        clampSwingCone(clampedSwing, mSwingHalfAngleY, mSwingHalfAngleZ);
        const Math::vec3 currentAxis =
                (mConstraintToWorld * originalSwing) * Math::vec3(1.0f, 0.0f, 0.0f);
        const Math::vec3 desiredAxis =
                (mConstraintToWorld * clampedSwing) * Math::vec3(1.0f, 0.0f, 0.0f);
        const f32 error = acosf(Math::clamp(Math::dot(currentAxis, desiredAxis), -1.0f, 1.0f));
        if (error > 1.0e-6f && mSwingLimitEffectiveMass > 0.0f)
        {
            const f32 lambda = -mSwingLimitEffectiveMass * baumgarte * error;
            if (mBodyA->isDynamic())
                rotateBody(*mBodyA,
                        mBodyA->inverseInertiaTensorWorld() * mWorldSwingLimitAxis * -lambda);
            if (mBodyB->isDynamic())
                rotateBody(*mBodyB,
                        mBodyB->inverseInertiaTensorWorld() * mWorldSwingLimitAxis * lambda);
        }
    }

    if (mTwistActive)
    {
        // The axis is flipped for the minimum side so the velocity pass can clamp
        // its impulse to one sign. The position error is signed on its own, so it
        // goes about the unflipped axis, or the minimum side is pushed further
        // past its limit.
        const Math::vec3 twistAxis =
                mTwistMinClosest ? -mWorldTwistLimitAxis : mWorldTwistLimitAxis;
        f32 error = 2.0f * atanf(mTwist.x / Math::max(mTwist.w, 1.0e-6f));
        if (mTwistLocked) error -= twistLockAngle();
        else
            error -= mTwistMinClosest ? mTwistMinAngle : mTwistMaxAngle;
        const f32 lambda = -mTwistLimitEffectiveMass * baumgarte * error;
        if (mBodyA->isDynamic())
            rotateBody(*mBodyA, mBodyA->inverseInertiaTensorWorld() * twistAxis * -lambda);
        if (mBodyB->isDynamic())
            rotateBody(*mBodyB, mBodyB->inverseInertiaTensorWorld() * twistAxis * lambda);
    }
}

} // namespace kinetix::Physics
