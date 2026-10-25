#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/SoilCutting.h"

#include "../dynamics/JointAxis.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kGravity = 9.81f;

}

f32 SoilBed::depth(const Math::vec3& point) const
{
    if (point.x < minX || point.x > maxX || point.z < minZ || point.z > maxZ)
        return 0.0f;
    return Math::max(surfaceHeight - point.y, 0.0f);
}

SoilCuttingTool::SoilCuttingTool(RigidBody& tool, const Math::vec3& worldEdgeA, const Math::vec3& worldEdgeB)
    : mTool(&tool), mLocalA(tool.pointToLocal(worldEdgeA)), mLocalB(tool.pointToLocal(worldEdgeB))
{
}

RigidBody* SoilCuttingTool::bodyA() const
{
    return mTool;
}

RigidBody* SoilCuttingTool::bodyB() const
{
    return mTool;
}

void SoilCuttingTool::setSoilBed(const SoilBed& bed)
{
    mBed = bed;
}

void SoilCuttingTool::setResistanceScale(f32 scale)
{
    if (isfinite(scale))
        mScale = Math::max(scale, 0.0f);
}

Math::vec3 SoilCuttingTool::edgeMiddle() const
{
    return (mTool->pointToWorld(mLocalA) + mTool->pointToWorld(mLocalB)) * 0.5f;
}

Math::vec3 SoilCuttingTool::force() const
{
    return mPreviousDuration > 0.0f && mActive ? mTotalImpulse / mPreviousDuration : Math::vec3(0.0f);
}

void SoilCuttingTool::setRakeAngle(f32 radians)
{
    if (isfinite(radians))
        mRake = Math::clamp(radians, 0.1f, 1.5707963f);
}

f32 SoilCuttingTool::cuttingResistance(const SoilParameters& soil, f32 width, f32 depth, f32 speed, f32 rakeAngle)
{
    if (!(depth > 0.0f) || !(width > 0.0f))
        return 0.0f;
    const f32 alpha = Math::clamp(rakeAngle, 0.1f, 1.5707963f);
    const f32 delta = Math::max(soil.toolFrictionAngle, 0.0f);
    const f32 phi = Math::max(soil.frictionAngle, 0.0f);
    const f32 gamma = soil.density * kGravity;
    const f32 cotAlpha = cosf(alpha) / sinf(alpha);
    // F(beta) over the failure planes that close (beta + phi < 90 deg, positive denominator); the
    // least is the one the soil takes. Coarse then fine.
    auto force = [&](f32 beta) -> f32
    {
        const f32 sum = beta + phi;
        if (!(sum < 1.5707f) || !(beta > 0.0f))
            return 1.0e30f;
        const f32 cotBetaPhi = cosf(sum) / sinf(sum);
        const f32 cotBeta = cosf(beta) / sinf(beta);
        const f32 denominator = cosf(alpha + delta) + sinf(alpha + delta) * cotBetaPhi;
        if (!(denominator > 1.0e-4f))
            return 1.0e30f;
        const f32 nGamma = (cotAlpha + cotBeta) / (2.0f * denominator);
        const f32 nC = (1.0f + cotBeta * cotBetaPhi) / denominator;
        return gamma * depth * depth * nGamma + soil.cohesion * depth * nC;
    };
    f32 bestBeta = 0.5f;
    f32 best = 1.0e30f;
    for (u32 i = 1; i < 90; ++i)
    {
        const f32 beta = static_cast<f32>(i) * 0.0174533f;
        const f32 f = force(beta);
        if (f < best)
        {
            best = f;
            bestBeta = beta;
        }
    }
    for (u32 i = 0; i <= 40; ++i)
    {
        const f32 beta = bestBeta + (static_cast<f32>(i) - 20.0f) * 0.0174533f / 20.0f;
        best = Math::min(best, force(beta));
    }
    if (!(best < 1.0e29f))
        return 0.0f;
    return width * (best + soil.density * speed * speed * depth);
}

void SoilCuttingTool::setup(f32 duration)
{
    const Math::vec3 a = mTool->pointToWorld(mLocalA);
    const Math::vec3 b = mTool->pointToWorld(mLocalB);
    const Math::vec3 middle = (a + b) * 0.5f;
    mDepth = (mBed.depth(a) + mBed.depth(b) + 2.0f * mBed.depth(middle)) * 0.25f;
    const f32 width = Math::length(b - a);
    const f32 speed = Math::length(mTool->velocityAtPoint(middle));
    mResistance = mScale * cuttingResistance(mBed.soil, width, mDepth, speed, mRake);
    const bool wasActive = mActive;
    mActive = mResistance > 0.0f && mTool->isDynamic();
    if (!mActive)
    {
        mTotalImpulse = Math::vec3(0.0f);
        mPreviousDuration = duration;
        return;
    }
    mArm = middle - mTool->position();
    Math::mat3 inverseEffectiveMass(0.0f);
    const Math::vec3 axes[] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f)};
    for (u32 axis = 0; axis < 3; ++axis)
        inverseEffectiveMass[axis] =
            axes[axis] * mTool->inverseMass() +
            Math::cross(mTool->inverseInertiaTensorWorld() * Math::cross(mArm, axes[axis]), mArm);
    const f32 determinant = Math::determinant(inverseEffectiveMass);
    mEffectiveMass = detail::invertible(inverseEffectiveMass, determinant) ? Math::inverse(inverseEffectiveMass)
                                                                          : Math::mat3(0.0f);
    mCap = mResistance * duration;
    if (wasActive && mPreviousDuration > 0.0f)
        mTotalImpulse *= duration / mPreviousDuration;
    else
        mTotalImpulse = Math::vec3(0.0f);
    const f32 magnitude = Math::length(mTotalImpulse);
    if (magnitude > mCap)
        mTotalImpulse *= mCap / magnitude;
    mPreviousDuration = duration;
}

void SoilCuttingTool::warmStart()
{
    if (mActive)
        mTool->applyImpulseAtPoint(mTotalImpulse, mTool->position() + mArm);
}

void SoilCuttingTool::solveVelocity()
{
    if (!mActive)
        return;
    const Math::vec3 velocity = mTool->velocity() + Math::cross(mTool->angularVelocity(), mArm);
    Math::vec3 total = mTotalImpulse - mEffectiveMass * velocity;
    const f32 magnitude = Math::length(total);
    if (magnitude > mCap && magnitude > 0.0f)
        total *= mCap / magnitude;
    const Math::vec3 applied = total - mTotalImpulse;
    mTotalImpulse = total;
    // Applied without waking: the tool is awake whenever this is solved.
    mTool->setVelocity(mTool->velocity() + applied * mTool->inverseMass());
    mTool->setAngularVelocity(mTool->angularVelocity() +
                              mTool->inverseInertiaTensorWorld() * Math::cross(mArm, applied));
}

void SoilCuttingTool::solvePosition(f32 baumgarte)
{
    (void)baumgarte;
}

} // namespace kinetix::Physics
