#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/Rope.h"

#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

// A rope shorter than its rest length by more than this is slack: whatever
// it carried last step is not carried into this one.
constexpr f32 kSlackTolerance = 1.0e-3f;

}

Winch::Winch(const WinchParameters& parameters) : mParameters(parameters)
{
    WinchParameters& p = mParameters;
    p.drumRadius = isfinite(p.drumRadius) && p.drumRadius > 1.0e-3f ? p.drumRadius : 0.25f;
    p.maxLinePull = isfinite(p.maxLinePull) ? Math::max(p.maxLinePull, 0.0f) : 0.0f;
    p.maxLineSpeed = isfinite(p.maxLineSpeed) ? Math::max(p.maxLineSpeed, 0.0f) : 0.0f;
    if (!isfinite(p.brakeLinePull) || p.brakeLinePull <= 0.0f)
        p.brakeLinePull = 1.5f * p.maxLinePull;
    // The drum as a mass on the line: J / r^2.
    if (isfinite(p.drumInertia) && p.drumInertia > 0.0f)
        mEquivalentMass = p.drumInertia / (p.drumRadius * p.drumRadius);
    else
    {
        mEquivalentMass = 50.0f;
        p.drumInertia = mEquivalentMass * p.drumRadius * p.drumRadius;
    }
}

void Winch::setCommand(f32 command)
{
    if (!isfinite(command))
        return;
    mCommand = Math::clamp(command, -1.0f, 1.0f);
}

void Winch::setBrake(bool engaged)
{
    mManualBrake = engaged;
}

void Winch::setFreeSpool(bool freeSpool)
{
    mFreeSpool = freeSpool;
}

bool Winch::braking() const
{
    return mManualBrake || (!mFreeSpool && mParameters.autoBrake && mCommand == 0.0f);
}

Rope::Rope(RigidBody& a, const Math::vec3& worldPointA, RigidBody& b, const Math::vec3& worldPointB,
           f32 restLength)
{
    mPoints[0].body = &a;
    mPoints[0].local = a.pointToLocal(worldPointA);
    mPoints[1].body = &b;
    mPoints[1].local = b.pointToLocal(worldPointB);
    mCount = 2;
    for (u32 i = 0; i < MaxPoints; ++i)
        mPlane[i] = Math::vec3(0.0f, 0.0f, 1.0f);
    // Negative until the path is known: taken from it at the first setup.
    mRestLength = isfinite(restLength) && restLength > 0.0f ? restLength : -1.0f;
    calculateJacobian(false);
}

RigidBody* Rope::bodyA() const
{
    return mPoints[0].body;
}

RigidBody* Rope::bodyB() const
{
    return mPoints[mCount - 1].body;
}

bool Rope::addSheave(RigidBody& body, const Math::vec3& worldPoint, f32 radius, f32 efficiency)
{
    if (mCount >= MaxPoints)
        return false;
    mPoints[mCount] = mPoints[mCount - 1];
    Point& sheave = mPoints[mCount - 1];
    sheave.body = &body;
    sheave.local = body.pointToLocal(worldPoint);
    sheave.radius = isfinite(radius) ? Math::max(radius, 0.0f) : 0.0f;
    sheave.efficiency = isfinite(efficiency) ? Math::clamp(efficiency, 0.05f, 1.0f) : 1.0f;
    if (sheave.efficiency < 1.0f)
        mHasFriction = true;
    ++mCount;
    return true;
}

Math::vec3 Rope::pointWorld(u32 index) const
{
    if (index >= mCount)
        return Math::vec3(0.0f);
    return mPoints[index].body->pointToWorld(mPoints[index].local);
}

RigidBody* Rope::pointBody(u32 index) const
{
    return index < mCount ? mPoints[index].body : nullptr;
}

void Rope::setFalls(u32 falls)
{
    mFalls = falls > 0 ? falls : 1;
}

void Rope::setElasticity(f32 axialStiffness, f32 dampingRatio)
{
    if (!isfinite(axialStiffness) || !isfinite(dampingRatio))
        return;
    mAxialStiffness = Math::max(axialStiffness, 0.0f);
    mDampingRatio = Math::max(dampingRatio, 0.0f);
}

void Rope::setRestLength(f32 length)
{
    if (!isfinite(length))
        return;
    mRestLength = Math::max(length, 0.0f);
    for (u32 i = 0; i < mCount; ++i)
        if (mPoints[i].body && mPoints[i].body->isDynamic())
            mPoints[i].body->setAwake(true);
}

void Rope::setLengthLimits(f32 minLength, f32 maxLength)
{
    if (!isfinite(minLength) || !isfinite(maxLength))
        return;
    mMinLength = Math::max(minLength, 0.0f);
    mMaxLength = Math::max(maxLength, mMinLength);
}

void Rope::setWinch(Winch* winch)
{
    mWinch = winch;
    mTotalDrumImpulse = 0.0f;
    mDrumAdvanced = true;
}

f32 Rope::length() const
{
    return pathLength(nullptr);
}

f32 Rope::tension() const
{
    return mPreviousDuration > 0.0f ? mTotalImpulse / mPreviousDuration : 0.0f;
}

f32 Rope::segmentTension(u32 index) const
{
    return index + 1 < mCount ? tension() * mTensionFactor[index] : 0.0f;
}

Math::vec3 Rope::reactionForce() const
{
    return -mLastDirection * hookLoad();
}

// The route, from the points' centres. A sheave with a radius is wrapped on
// its outside: in the plane through it and its two neighbours (each taken as
// a point), the rope leaves along the tangent towards each neighbour and
// wraps the arc between the two tangent points,
//   phi = 2 pi - beta - acos(r / d_in) - acos(r / d_out),
// beta the angle the neighbours make at the centre. The straight segments
// run between tangent points (or the points themselves).
void Rope::computePath(Path& path) const
{
    Math::vec3 centres[MaxPoints];
    Math::vec3 entry[MaxPoints];
    Math::vec3 exit[MaxPoints];
    for (u32 i = 0; i < mCount; ++i)
    {
        centres[i] = mPoints[i].body->pointToWorld(mPoints[i].local);
        entry[i] = centres[i];
        exit[i] = centres[i];
    }
    f32 wrapped = 0.0f;
    for (u32 i = 1; i + 1 < mCount; ++i)
    {
        const f32 r = mPoints[i].radius;
        if (!(r > 0.0f))
            continue;
        // The sheave the falls hang from stays a point: the falls count its
        // outgoing segment N times but a wrap once, so length and gradient
        // would disagree.
        if (i + 2 == mCount && mFalls > 1)
            continue;
        const Math::vec3 toPrevious = centres[i - 1] - centres[i];
        const Math::vec3 toNext = centres[i + 1] - centres[i];
        const f32 d1 = Math::length(toPrevious);
        const f32 d2 = Math::length(toNext);
        // A neighbour inside the sheave has no tangent: a point there.
        if (!(d1 > 1.001f * r) || !(d2 > 1.001f * r))
            continue;
        const Math::vec3 e1 = toPrevious / d1;
        const Math::vec3 e2 = toNext / d2;
        Math::vec3 normal = Math::cross(e1, e2);
        const f32 normalLength = Math::length(normal);
        if (normalLength > 1.0e-5f)
        {
            normal = normal / normalLength;
            mPlane[i] = normal;
        }
        else
        {
            // Straight through or straight back: the last plane it had.
            normal = mPlane[i] - e1 * Math::dot(mPlane[i], e1);
            const f32 length = Math::length(normal);
            normal = length > 1.0e-5f ? normal / length : Math::vec3(0.0f, 0.0f, 1.0f);
        }
        // In the plane, square to each neighbour's direction and away from
        // the other one: the outside of the bend.
        Math::vec3 away1 = Math::cross(normal, e1);
        if (Math::dot(away1, e2) > 0.0f)
            away1 = -away1;
        Math::vec3 away2 = Math::cross(normal, e2);
        if (Math::dot(away2, e1) > 0.0f)
            away2 = -away2;
        const f32 c1 = r / d1;
        const f32 c2 = r / d2;
        entry[i] = centres[i] + (e1 * c1 + away1 * sqrtf(1.0f - c1 * c1)) * r;
        exit[i] = centres[i] + (e2 * c2 + away2 * sqrtf(1.0f - c2 * c2)) * r;
        const f32 beta = acosf(Math::clamp(Math::dot(e1, e2), -1.0f, 1.0f));
        const f32 phi = 2.0f * Math::pi<f32>() - beta - acosf(c1) - acosf(c2);
        wrapped += r * Math::max(phi, 0.0f);
    }
    f32 total = wrapped;
    for (u32 i = 0; i + 1 < mCount; ++i)
    {
        const Math::vec3 delta = entry[i + 1] - exit[i];
        const f32 segment = Math::length(delta);
        const f32 weight = i + 2 == mCount ? static_cast<f32>(mFalls) : 1.0f;
        total += weight * segment;
        path.segment[i] = segment;
        path.direction[i] = segment > 1.0e-6f ? delta / segment : Math::vec3(0.0f, -1.0f, 0.0f);
    }
    path.length = total;
}

// The length of the path, the last segment counted once per fall. Fills each
// segment's unit direction into `directions` when given.
f32 Rope::pathLength(Math::vec3* directions) const
{
    Path path;
    computePath(path);
    if (directions)
        for (u32 i = 0; i + 1 < mCount; ++i)
            directions[i] = path.direction[i];
    return path.length;
}

// Sheave friction: the tension rises by 1/eta across a sheave in the
// direction the rope runs through it. The rope's speed through sheave i is
// what the drum pays out less what the segments before it take up; the
// direction blends smoothly through standstill (a couple of cm/s) so a
// stopping rope does not chatter.
void Rope::updateFriction(const Path& path)
{
    for (u32 i = 0; i < MaxPoints; ++i)
        mTensionFactor[i] = 1.0f;
    if (!mHasFriction)
        return;
    f32 speed = mWinch ? mWinch->mLineSpeed : 0.0f;
    f32 factor = 1.0f;
    for (u32 i = 1; i + 1 < mCount; ++i)
    {
        // Segment i - 1 lengthening takes rope from the drum side.
        const Math::vec3 before = mPoints[i - 1].body->velocityAtPoint(mPoints[i - 1].body->pointToWorld(mPoints[i - 1].local));
        const Math::vec3 here = mPoints[i].body->velocityAtPoint(mPoints[i].body->pointToWorld(mPoints[i].local));
        speed -= Math::dot(path.direction[i - 1], here - before);
        const f32 eta = mPoints[i].efficiency;
        if (eta < 1.0f)
            factor *= powf(eta, -tanhf(speed / 0.02f));
        mTensionFactor[i] = factor;
    }
}

// dL/dt = sum over points of g_i . v_i, with g_i = w_(i-1) u_(i-1) - w_i u_i
// for the segments either side (u from each point towards the next, w the
// falls on the last). Points on the same body are folded into one linear and
// one angular part. The force an impulse applies is the same with each
// segment's tension factor in its weight.
void Rope::calculateJacobian(bool friction)
{
    Path path;
    computePath(path);
    if (friction)
        updateFriction(path);
    const Math::vec3* directions = path.direction;
    mLength = path.length;
    mPartCount = 0;
    mInverseMass = 0.0f;
    mGeometricInverseMass = 0.0f;
    for (u32 i = 0; i < mCount; ++i)
    {
        Math::vec3 gradient(0.0f);
        Math::vec3 force(0.0f);
        if (i > 0)
        {
            const Math::vec3 in = directions[i - 1] * (i + 1 == mCount ? static_cast<f32>(mFalls) : 1.0f);
            gradient += in;
            force += in * mTensionFactor[i - 1];
        }
        if (i + 1 < mCount)
        {
            const Math::vec3 out = directions[i] * (i + 2 == mCount ? static_cast<f32>(mFalls) : 1.0f);
            gradient -= out;
            force -= out * mTensionFactor[i];
        }
        RigidBody* body = mPoints[i].body;
        const Math::vec3 arm = body->pointToWorld(mPoints[i].local) - body->position();
        u32 part = 0;
        while (part < mPartCount && mParts[part].body != body)
            ++part;
        if (part == mPartCount)
        {
            mParts[part].body = body;
            mParts[part].linear = Math::vec3(0.0f);
            mParts[part].angular = Math::vec3(0.0f);
            mParts[part].forceLinear = Math::vec3(0.0f);
            mParts[part].forceAngular = Math::vec3(0.0f);
            ++mPartCount;
        }
        mParts[part].linear += gradient;
        mParts[part].angular += Math::cross(arm, gradient);
        mParts[part].forceLinear += force;
        mParts[part].forceAngular += Math::cross(arm, force);
    }
    for (u32 part = 0; part < mPartCount; ++part)
    {
        const RigidBody& body = *mParts[part].body;
        const BodyPart& p = mParts[part];
        mInverseMass += body.inverseMass() * Math::dot(p.linear, p.forceLinear) +
                        Math::dot(p.angular, body.inverseInertiaTensorWorld() * p.forceAngular);
        mGeometricInverseMass += body.inverseMass() * Math::dot(p.linear, p.linear) +
                                 Math::dot(p.angular, body.inverseInertiaTensorWorld() * p.angular);
    }
    if (mCount >= 2)
        mLastDirection = directions[mCount - 2];
}

f32 Rope::bodyVelocity() const
{
    f32 rate = 0.0f;
    for (u32 part = 0; part < mPartCount; ++part)
    {
        const RigidBody& body = *mParts[part].body;
        rate += Math::dot(mParts[part].linear, body.velocity()) +
                Math::dot(mParts[part].angular, body.angularVelocity());
    }
    return rate;
}

// The tension pulls every point along -g_i: the ends towards each other, a
// sheave into the bight of the rope.
void Rope::applyBodyImpulse(f32 impulse)
{
    for (u32 part = 0; part < mPartCount; ++part)
    {
        RigidBody& body = *mParts[part].body;
        if (!body.isDynamic())
            continue;
        body.setVelocity(body.velocity() - mParts[part].forceLinear * (impulse * body.inverseMass()));
        body.setAngularVelocity(body.angularVelocity() -
                                body.inverseInertiaTensorWorld() * mParts[part].forceAngular * impulse);
    }
}

void Rope::wakeRoute()
{
    for (u32 i = 0; i < mCount; ++i)
        if (mPoints[i].body && mPoints[i].body->isDynamic())
            mPoints[i].body->setAwake(true);
}

// The drum's line speed, integrated into the rest length once per step,
// after the velocities are final and before the position pass measures the
// stretch.
void Rope::advanceDrum()
{
    mDrumAdvanced = true;
    if (!mWinch || !(mDuration > 0.0f))
        return;
    Winch& winch = *mWinch;
    f32 length = mRestLength + winch.mLineSpeed * mDuration;
    if (length < mMinLength || length > mMaxLength)
    {
        length = Math::clamp(length, mMinLength, mMaxLength);
        winch.mLineSpeed = 0.0f;
    }
    winch.mDrumAngle += (length - mRestLength) / winch.mParameters.drumRadius;
    mRestLength = length;
    const f32 torque = mTotalDrumImpulse / mDuration * winch.mParameters.drumRadius;
    winch.mMotorTorque = mDrumBraking ? 0.0f : -torque;
    winch.mBrakeTorque = mDrumBraking ? fabsf(torque) : 0.0f;
}

void Rope::setup(f32 duration)
{
    if (!mDrumAdvanced)
        advanceDrum();

    calculateJacobian(true);
    if (mRestLength < 0.0f)
        mRestLength = mLength;
    const f32 stretch = mLength - mRestLength;

    mRigid = !(mAxialStiffness > 0.0f);
    mSoftness = 0.0f;
    if (stretch < 0.0f)
        // Slack: the rope may close the gap this step, no more - the
        // speculative target of a contact.
        mBias = stretch / duration;
    else if (mRigid || !(mInverseMass > 0.0f))
        mBias = 0.0f;
    else
    {
        // The elastic rope: a spring EA / L on the stretch, solved implicitly.
        const f32 stiffness = mAxialStiffness / Math::max(mRestLength, 1.0e-3f);
        const f32 damping = 2.0f * mDampingRatio * sqrtf(stiffness / mInverseMass);
        const f32 denominator = damping + duration * stiffness;
        mSoftness = 1.0f / (duration * denominator);
        mBias = stiffness * stretch / denominator;
    }

    if (mPreviousDuration > 0.0f && stretch > -kSlackTolerance)
        mTotalImpulse *= duration / mPreviousDuration;
    else
        mTotalImpulse = 0.0f;
    if (!(mInverseMass > 0.0f))
        mTotalImpulse = 0.0f;

    mDrumActive = mWinch != nullptr;
    if (mWinch)
    {
        const Winch& winch = *mWinch;
        const WinchParameters& p = winch.mParameters;
        f32 target = -winch.mCommand * p.maxLineSpeed;
        if ((target < 0.0f && mRestLength <= mMinLength) || (target > 0.0f && mRestLength >= mMaxLength))
            target = 0.0f;
        mDrumBraking = winch.braking();
        const f32 limit = mDrumBraking ? p.brakeLinePull : (winch.mFreeSpool ? 0.0f : p.maxLinePull);
        mDrumTarget = mDrumBraking ? 0.0f : target;
        mDrumMinImpulse = -limit * duration;
        mDrumMaxImpulse = limit * duration;
        if (mPreviousDuration > 0.0f)
            mTotalDrumImpulse *= duration / mPreviousDuration;
        else
            mTotalDrumImpulse = 0.0f;
        mTotalDrumImpulse = Math::clamp(mTotalDrumImpulse, mDrumMinImpulse, mDrumMaxImpulse);
        if (mDrumTarget != 0.0f || fabsf(winch.mLineSpeed) > 1.0e-4f)
            wakeRoute();
    }

    mDuration = duration;
    mDrumAdvanced = false;
    mPreviousDuration = duration;
}

void Rope::warmStart()
{
    if (mInverseMass > 0.0f)
        applyBodyImpulse(mTotalImpulse);
    if (mDrumActive)
        mWinch->mLineSpeed += (mTotalImpulse + mTotalDrumImpulse) / mWinch->mEquivalentMass;
}

// The rope row and the drum row, solved together. With q = -dC/dt the rate
// the rope shortens at, the rope wants q' = bias - softness * lambda' with
// lambda' >= 0, and the motor wants the line speed psi' at its target with
// its impulse inside the motor's (or the brake's) limit. Tension pulls line
// off the drum: psi' = psi + (lambda_rope + lambda_motor) / m_drum.
void Rope::solveVelocity()
{
    const f32 rope = solveRow(bodyVelocity(), mInverseMass);
    if (rope != 0.0f)
        applyBodyImpulse(rope);
}

// One iteration of the rope row and the drum row, given the path's
// lengthening rate and the inverse mass along the row; returns the rope
// impulse for the bodies (the drum's is applied here).
f32 Rope::solveRow(f32 rate, f32 bodies)
{
    if (!mDrumActive)
    {
        if (!(bodies > 0.0f))
            return 0.0f;
        const f32 impulse = (rate + mBias - mSoftness * mTotalImpulse) / (bodies + mSoftness);
        const f32 previous = mTotalImpulse;
        mTotalImpulse = Math::max(previous + impulse, 0.0f);
        return mTotalImpulse - previous;
    }

    Winch& winch = *mWinch;
    const f32 drum = 1.0f / winch.mEquivalentMass;
    const f32 psi = winch.mLineSpeed;
    f32 rope = 0.0f;
    f32 motor = 0.0f;
    if (bodies > 0.0f)
    {
        // Both rows free: the motor holds the drum at its target.
        rope = (rate - mDrumTarget + mBias - mSoftness * mTotalImpulse) / (bodies + mSoftness);
        if (mTotalImpulse + rope < 0.0f)
            rope = -mTotalImpulse;
    }
    motor = (mDrumTarget - psi) / drum - rope;
    const f32 clamped = Math::clamp(mTotalDrumImpulse + motor, mDrumMinImpulse, mDrumMaxImpulse);
    if (clamped != mTotalDrumImpulse + motor)
    {
        // The motor is on its limit: the rope takes the drum with it.
        motor = clamped - mTotalDrumImpulse;
        if (bodies > 0.0f)
        {
            rope = (rate - psi + mBias - mSoftness * mTotalImpulse - drum * motor) /
                   (bodies + drum + mSoftness);
            if (mTotalImpulse + rope < 0.0f)
                rope = -mTotalImpulse;
        }
    }
    mTotalImpulse += rope;
    mTotalDrumImpulse += motor;
    winch.mLineSpeed = psi + (rope + motor) * drum;
    return rope;
}

bool Rope::supportsArticulation() const
{
    u32 distinct = 0;
    for (u32 i = 0; i < mCount; ++i)
    {
        bool seen = false;
        for (u32 k = 0; k < i; ++k)
            seen = seen || mPoints[k].body == mPoints[i].body;
        if (!seen)
            ++distinct;
    }
    return distinct <= Joint::MaxArticulationBodies;
}

u32 Rope::articulationBodies(RigidBody** bodies)
{
    u32 count = 0;
    for (u32 i = 0; i < mCount; ++i)
    {
        RigidBody* body = mPoints[i].body;
        bool seen = false;
        for (u32 k = 0; k < count; ++k)
            seen = seen || bodies[k] == body;
        if (!seen && count < Joint::MaxArticulationBodies)
            bodies[count++] = body;
    }
    return count;
}

// The row as the articulation counts it: its rate is how fast the path
// shortens (the rope's impulse, its tension, shortens it), so -dL/dt.
u32 Rope::articulationWideMotors(ArticulationRow* rows)
{
    ArticulationRow& row = rows[0];
    row.parts = mPartCount;
    row.asymmetric = mHasFriction;
    for (u32 part = 0; part < mPartCount; ++part)
    {
        row.body[part] = mParts[part].body;
        row.linear[part] = -mParts[part].linear;
        row.angular[part] = -mParts[part].angular;
        row.forceLinear[part] = -mParts[part].forceLinear;
        row.forceAngular[part] = -mParts[part].forceAngular;
    }
    row.id = 0;
    return 1;
}

void Rope::setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass)
{
    (void)id;
    (void)gainInverseMass;
    mArticulatedInverseMass = inverseMass;
}

f32 Rope::solveArticulationMotor(u32 id, f32 velocity)
{
    (void)id;
    return solveRow(-velocity, mArticulatedInverseMass);
}

// Stretched past its length, a rigid rope is held with the articulation's
// locks: C = rest length - length, dC/dt the shortening rate. The drum's
// line speed goes into the rest length first, once a step (as the position
// pass does).
u32 Rope::articulationWidePositionRows(ArticulationRow* rows, f32* errors)
{
    if (!mDrumAdvanced)
        advanceDrum();
    if (!mRigid)
        return 0;
    calculateJacobian(false);
    const f32 stretch = mLength - mRestLength;
    if (stretch <= 0.0f)
        return 0;
    ArticulationRow& row = rows[0];
    row.parts = mPartCount;
    row.asymmetric = false;
    for (u32 part = 0; part < mPartCount; ++part)
    {
        row.body[part] = mParts[part].body;
        row.linear[part] = -mParts[part].linear;
        row.angular[part] = -mParts[part].angular;
    }
    row.id = 0;
    errors[0] = -stretch;
    return 1;
}

void Rope::solvePosition(f32 baumgarte)
{
    if (!mDrumAdvanced)
        advanceDrum();
    if (!mRigid)
        return;
    calculateJacobian(false);
    const f32 stretch = mLength - mRestLength;
    if (stretch <= 0.0f || !(mGeometricInverseMass > 0.0f))
        return;
    // Stretched past its length: the bodies are moved back along the
    // gradient (the drum keeps its place - the brake or the motor has it).
    const f32 lambda = baumgarte * stretch / mGeometricInverseMass;
    Math::vec3 directions[MaxPoints];
    pathLength(directions);
    for (u32 i = 0; i < mCount; ++i)
    {
        RigidBody& body = *mPoints[i].body;
        if (!body.isDynamic())
            continue;
        Math::vec3 gradient(0.0f);
        if (i > 0)
            gradient += directions[i - 1] * (i + 1 == mCount ? static_cast<f32>(mFalls) : 1.0f);
        if (i + 1 < mCount)
            gradient -= directions[i] * (i + 2 == mCount ? static_cast<f32>(mFalls) : 1.0f);
        body.applyPositionImpulseAtPoint(-gradient * lambda, body.pointToWorld(mPoints[i].local));
    }
}

} // namespace kinetix::Physics
