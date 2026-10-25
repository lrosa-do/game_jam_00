#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/TrackedUndercarriage.h"

#include "dynamics/PhysicsWorld.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr u32 kNoBody = 0xFFFFFFFFu;
// A sample this far above the surface still gets a speculative row (it may land this step).
constexpr f32 kContactMargin = 0.02f;
// Shear displacement is held within this many K: the stress is saturated (1 - e^-8) there, and a
// reversal need not unwind an unbounded slide.
constexpr f32 kShearDisplacementLimit = 8.0f;
// Soil damping on the shear, faded out between these belt speeds: it is for a machine standing on
// its shear springs (slope, braked); a running belt refreshes the soil, which damps by itself,
// and a viscous term would add to the Janosi-Hanamoto stress.
constexpr f32 kShearDampingFull = 0.05f;
constexpr f32 kShearDampingNone = 0.15f;

f32 positiveOr(f32 value, f32 minimum, f32 fallback)
{
    return isfinite(value) ? Math::max(value, minimum) : fallback;
}

bool finiteVector(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

}

TrackedUndercarriage::TrackedUndercarriage(PhysicsWorld& world, u32 hullBodyId, const TrackParameters& parameters)
    : mWorld(&world), mHullId(hullBodyId), mHull(nullptr), mParameters(parameters)
{
    TrackParameters& p = mParameters;
    p.gauge = positiveOr(p.gauge, 0.1f, 2.0f);
    p.shoeWidth = positiveOr(p.shoeWidth, 0.01f, 0.6f);
    p.contactLength = positiveOr(p.contactLength, 0.05f, 3.6f);
    p.contactCentre = isfinite(p.contactCentre) ? p.contactCentre : 0.0f;
    p.bottomHeight = isfinite(p.bottomHeight) ? p.bottomHeight : -0.8f;
    p.sprocketRadius = positiveOr(p.sprocketRadius, 0.01f, 0.35f);
    p.idlerRadius = positiveOr(p.idlerRadius, 0.01f, 0.32f);
    p.roadWheelRadius = positiveOr(p.roadWheelRadius, 0.01f, 0.1f);
    p.samples = Math::clamp(p.samples, 2u, MaxSamples);
    p.beltMass = positiveOr(p.beltMass, 1.0f, 1200.0f);
    p.internalResistance = positiveOr(p.internalResistance, 0.0f, 0.0f);
    p.soilDampingRatio = positiveOr(p.soilDampingRatio, 0.0f, 0.8f);
    p.supportedMass = positiveOr(p.supportedMass, 0.0f, 0.0f);
    p.probeHeight = positiveOr(p.probeHeight, 0.01f, 0.6f);
    p.probeDepth = positiveOr(p.probeDepth, 0.01f, 0.8f);
    mSampleCount = p.samples;
    mSampleLength = p.contactLength / static_cast<f32>(mSampleCount);
    mRayFilter.ignoredBody = hullBodyId;
    if (BodyEntry* entry = world.body(hullBodyId))
        mHull = entry->body;
    if (!mHull)
        Log::error("TrackedUndercarriage: no hull body %u", hullBodyId);
}

RigidBody* TrackedUndercarriage::bodyA() const
{
    return mHull;
}

RigidBody* TrackedUndercarriage::bodyB() const
{
    return mHull;
}

void TrackedUndercarriage::setSoil(const SoilParameters& soil)
{
    mSoil = soil;
}

bool TrackedUndercarriage::setSoil(u32 groundBodyId, const SoilParameters& soil)
{
    for (u32 i = 0; i < mOverrideCount; ++i)
        if (mOverrideBodies[i] == groundBodyId)
        {
            mOverrideSoils[i] = soil;
            return true;
        }
    if (mOverrideCount >= MaxSoilOverrides)
        return false;
    mOverrideBodies[mOverrideCount] = groundBodyId;
    mOverrideSoils[mOverrideCount] = soil;
    ++mOverrideCount;
    return true;
}

void TrackedUndercarriage::setRayFilter(const QueryFilter& filter)
{
    mRayFilter = filter;
    mRayFilter.ignoredBody = mHullId;
}

const SoilParameters& TrackedUndercarriage::soilFor(u32 bodyId) const
{
    for (u32 i = 0; i < mOverrideCount; ++i)
        if (mOverrideBodies[i] == bodyId)
            return mOverrideSoils[i];
    return mSoil;
}

Math::vec3 TrackedUndercarriage::sampleLocal(u32 side, u32 index) const
{
    const TrackParameters& p = mParameters;
    const f32 x = side == Left ? 0.5f * p.gauge : -0.5f * p.gauge;
    const f32 z = p.contactCentre + 0.5f * p.contactLength - (static_cast<f32>(index) + 0.5f) * mSampleLength;
    return Math::vec3(x, p.bottomHeight, z);
}

void TrackedUndercarriage::buildRow(Row& row, const Math::vec3& point, const Math::vec3& direction, f32 belt) const
{
    row.linear = direction;
    row.angular = Math::cross(point - mHull->position(), direction);
    row.belt = belt;
    row.inverseMass = mHull->inverseMass() * Math::dot(direction, direction) +
                      Math::dot(row.angular, mHull->inverseInertiaTensorWorld() * row.angular) +
                      belt * belt / mParameters.beltMass;
}

f32 TrackedUndercarriage::rowVelocity(const Row& row, f32 beltSpeed) const
{
    return Math::dot(row.linear, mHull->velocity()) + Math::dot(row.angular, mHull->angularVelocity()) +
           row.belt * beltSpeed;
}

void TrackedUndercarriage::applyRow(Row& row, Side& side, f32 impulse)
{
    if (impulse == 0.0f)
        return;
    if (mHull->isDynamic())
    {
        mHull->setVelocity(mHull->velocity() + row.linear * (impulse * mHull->inverseMass()));
        mHull->setAngularVelocity(mHull->angularVelocity() + mHull->inverseInertiaTensorWorld() * row.angular * impulse);
    }
    side.beltSpeed += row.belt * impulse / mParameters.beltMass;
}

// lambda = A - B u at the end of the step, within [lo, hi].
void TrackedUndercarriage::solveSoft(Row& row, Side& side)
{
    if (!(row.inverseMass > 0.0f))
        return;
    const f32 u = rowVelocity(row, side.beltSpeed);
    const f32 delta = (row.softA - row.softB * u - row.impulse) / (1.0f + row.softB * row.inverseMass);
    const f32 total = Math::clamp(row.impulse + delta, row.lo, row.hi);
    const f32 applied = total - row.impulse;
    row.impulse = total;
    applyRow(row, side, applied);
}

// u -> 0 within [lo, hi]: a Coulomb element.
void TrackedUndercarriage::solveBounded(Row& row, Side& side)
{
    if (!(row.inverseMass > 0.0f) || (row.lo == 0.0f && row.hi == 0.0f))
        return;
    const f32 u = rowVelocity(row, side.beltSpeed);
    const f32 total = Math::clamp(row.impulse - u / row.inverseMass, row.lo, row.hi);
    const f32 applied = total - row.impulse;
    row.impulse = total;
    applyRow(row, side, applied);
}

// Each sample's ground: a probe down the hull's -Y from above the ground face; the sinkage of the
// face below the surface along its normal.
void TrackedUndercarriage::probe(Side& side, u32 sideIndex, f32 duration)
{
    const TrackParameters& p = mParameters;
    const Math::vec3 up = mHull->directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    const Math::vec3 forwardHull = mHull->directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f));
    const f32 area = p.shoeWidth * mSampleLength;
    u32 contacts = 0;
    for (u32 k = 0; k < mSampleCount; ++k)
    {
        Sample& s = side.samples[k];
        const bool wasActive = s.active;
        const Math::vec3 point = mHull->pointToWorld(sampleLocal(sideIndex, k));
        s.state.point = point;
        s.active = false;
        s.state.contact = false;
        Ray ray;
        ray.origin = point + up * p.probeHeight;
        ray.direction = -up;
        WorldRayHit hit;
        if (mWorld->raycast(ray, p.probeHeight + p.probeDepth, mRayFilter, hit) && finiteVector(hit.normal))
        {
            const Math::vec3 normal = Math::normalize(hit.normal);
            const f32 z = Math::dot(hit.point - point, normal);
            if (z > -kContactMargin)
            {
                s.active = true;
                s.groundBody = hit.body;
                s.state.normal = normal;
                s.state.sinkage = z;
                s.soil = &soilFor(hit.body);
                ++contacts;
            }
        }
        if (!s.active)
        {
            s.state.sinkage = 0.0f;
            s.state.pressure = 0.0f;
            s.state.shearDisplacement = Math::vec2(0.0f);
            s.state.shearStress = Math::vec2(0.0f);
            s.normal.impulse = 0.0f;
            s.along.impulse = 0.0f;
            s.across.impulse = 0.0f;
            continue;
        }
        if (!wasActive)
        {
            s.lastSlip = Math::vec2(0.0f);
            s.normal.impulse = 0.0f;
            s.along.impulse = 0.0f;
            s.across.impulse = 0.0f;
        }
        Math::vec3 forward = forwardHull - s.state.normal * Math::dot(forwardHull, s.state.normal);
        const f32 length = Math::length(forward);
        forward = length > 1.0e-5f ? forward / length : Math::vec3(0.0f, 0.0f, 1.0f);
        s.forward = forward;
        s.lateral = Math::cross(s.state.normal, forward);
    }
    (void)area;
    (void)duration;
    side.state.samplesInContact = contacts;
}

// Semi-Lagrangian advection of the shear displacement: shoes run from the front of the contact to
// the rear at the belt speed (rear forward when it reverses), so the displacement now under
// sample k is the one |psi| h upstream - zero (fresh soil) from the entrance - plus this step's
// slip over the time that soil has been under the track (slipTime).
void TrackedUndercarriage::advance(Side& side, f32 duration)
{
    const u32 n = mSampleCount;
    const f32 dl = mSampleLength;
    const f32 speed = side.beltSpeed;
    const f32 travel = fabsf(speed) * duration;
    Math::vec2 previous[MaxSamples];
    for (u32 k = 0; k < n; ++k)
        previous[k] = side.samples[k].active ? side.samples[k].state.shearDisplacement : Math::vec2(0.0f);
    for (u32 k = 0; k < n; ++k)
    {
        Sample& s = side.samples[k];
        if (!s.active)
            continue;
        const u32 order = speed >= 0.0f ? k : n - 1 - k;
        const f32 x = (static_cast<f32>(order) + 0.5f) * dl;
        const f32 upstream = x - travel;
        if (upstream <= 0.0f)
        {
            s.advected = Math::vec2(0.0f);
            s.slipTime = speed != 0.0f ? x / fabsf(speed) : duration;
            continue;
        }
        s.slipTime = duration;
        const f32 cell = upstream / dl - 0.5f;
        Math::vec2 value(0.0f);
        if (cell <= 0.0f)
        {
            const u32 first = speed >= 0.0f ? 0 : n - 1;
            value = previous[first] * (upstream / (0.5f * dl));
        }
        else
        {
            const u32 low = Math::min(static_cast<u32>(cell), n - 1);
            const u32 high = Math::min(low + 1, n - 1);
            const f32 t = Math::clamp(cell - static_cast<f32>(low), 0.0f, 1.0f);
            const u32 a = speed >= 0.0f ? low : n - 1 - low;
            const u32 b = speed >= 0.0f ? high : n - 1 - high;
            value = previous[a] * (1.0f - t) + previous[b] * t;
        }
        s.advected = value;
    }
}

void TrackedUndercarriage::setup(f32 duration)
{
    mStepped = false;
    if (!mHull || !(duration > 0.0f))
        return;
    const TrackParameters& p = mParameters;
    const f32 ratio = mPreviousDuration > 0.0f ? duration / mPreviousDuration : 0.0f;
    mDuration = duration;
    const f32 area = p.shoeWidth * mSampleLength;
    const f32 supported = p.supportedMass > 0.0f ? p.supportedMass
                                                 : (mHull->inverseMass() > 0.0f ? 1.0f / mHull->inverseMass() : 0.0f);
    const Math::vec3 forwardHull = mHull->directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f));
    u32 totalContacts = 0;
    for (u32 sideIndex = 0; sideIndex < 2; ++sideIndex)
    {
        probe(mSides[sideIndex], sideIndex, duration);
        totalContacts += mSides[sideIndex].state.samplesInContact;
    }
    const f32 massShare = totalContacts > 0 ? supported / static_cast<f32>(totalContacts) : 0.0f;

    for (u32 sideIndex = 0; sideIndex < 2; ++sideIndex)
    {
        Side& side = mSides[sideIndex];
        advance(side, duration);
        const f32 beltFactor = Math::clamp((kShearDampingNone - fabsf(side.beltSpeed)) /
                                               (kShearDampingNone - kShearDampingFull),
                                           0.0f, 1.0f);
        f32 sinkageSum = 0.0f;
        f32 widthModulusSinkage = 0.0f;
        Math::vec3 centre(0.0f);
        const SoilParameters* sideSoil = nullptr;
        for (u32 k = 0; k < mSampleCount; ++k)
        {
            Sample& s = side.samples[k];
            if (!s.active)
                continue;
            const SoilParameters& soil = *s.soil;
            sideSoil = &soil;
            const f32 z = s.state.sinkage;
            const Math::vec3& point = s.state.point;
            centre += point;

            buildRow(s.normal, point, s.state.normal, 0.0f);
            const f32 stiffness = Terramechanics::pressureSlope(soil, z, p.shoeWidth) * area;
            const f32 damping = 2.0f * p.soilDampingRatio * sqrtf(stiffness * massShare);
            if (z > 0.0f)
            {
                const f32 force = Terramechanics::pressure(soil, z, p.shoeWidth) * area;
                s.normal.softA = duration * force;
                s.normal.softB = duration * (stiffness * duration + damping);
                sinkageSum += z;
            }
            else
            {
                // Above the surface, inside the margin: may land this step, no harder than the spring from zero.
                s.normal.softA = duration * stiffness * z;
                s.normal.softB = duration * stiffness * duration;
            }
            s.normal.lo = 0.0f;
            s.normal.hi = 1.0e30f;
            s.normal.impulse = Math::max(s.normal.impulse * ratio, 0.0f);

            // Shear: Janosi-Hanamoto on the advected displacement, with its tangent stiffness for this step's slip.
            const f32 lastPressure = s.state.pressure;
            const f32 sigma = lastPressure > 0.0f ? lastPressure : Terramechanics::pressure(soil, z, p.shoeWidth);
            const f32 strength = Terramechanics::shearStrength(soil, sigma);
            const f32 K = Math::max(soil.shearModulus, 1.0e-5f);
            const Math::vec2 j = s.advected;
            const f32 magnitude = Math::length(j);
            const Math::vec2 direction = magnitude > 1.0e-7f ? j / magnitude : Math::vec2(0.0f);
            const f32 stress = strength * (1.0f - expf(-magnitude / K));
            // Stiffness per direction over this step: the secant from where the displacement
            // starts to where last step's slip would take it (exact if the slip holds), the
            // tangent when it would hardly move. The law is isotropic, so from zero both
            // directions start at tau_max / K.
            const Math::vec2 predicted = j + s.lastSlip * s.slipTime;
            const f32 predictedMagnitude = Math::length(predicted);
            const Math::vec2 predictedStress =
                predictedMagnitude > 1.0e-7f
                    ? predicted * (strength * (1.0f - expf(-predictedMagnitude / K)) / predictedMagnitude)
                    : Math::vec2(0.0f);
            const Math::vec2 startStress = direction * stress;
            const f32 tangent = strength / K * expf(-magnitude / K);
            const f32 secant = magnitude > 1.0e-7f ? stress / magnitude : strength / K;
            f32 stiffAlong = magnitude > 1.0e-7f
                                 ? tangent * direction.x * direction.x + secant * direction.y * direction.y
                                 : strength / K;
            f32 stiffAcross = magnitude > 1.0e-7f
                                  ? tangent * direction.y * direction.y + secant * direction.x * direction.x
                                  : strength / K;
            const f32 reach = 0.05f * K;
            if (fabsf(predicted.x - j.x) > reach)
                stiffAlong = Math::max((predictedStress.x - startStress.x) / (predicted.x - j.x), 0.0f);
            if (fabsf(predicted.y - j.y) > reach)
                stiffAcross = Math::max((predictedStress.y - startStress.y) / (predicted.y - j.y), 0.0f);
            const f32 shearDamping = beltFactor * 2.0f * p.soilDampingRatio * sqrtf(strength / K * area * massShare);
            s.shearCap = duration * strength * area;
            buildRow(s.along, point, s.forward, -Math::dot(forwardHull, s.forward));
            buildRow(s.across, point, s.lateral, -Math::dot(forwardHull, s.lateral));
            s.along.softA = -duration * area * stress * direction.x;
            s.along.softB = duration * (s.slipTime * area * stiffAlong + shearDamping);
            s.across.softA = -duration * area * stress * direction.y;
            s.across.softB = duration * (s.slipTime * area * stiffAcross + shearDamping);
            s.along.lo = -s.shearCap;
            s.along.hi = s.shearCap;
            s.across.lo = -s.shearCap;
            s.across.hi = s.shearCap;
            s.along.impulse = Math::clamp(s.along.impulse * ratio, -s.shearCap, s.shearCap);
            s.across.impulse = Math::clamp(s.across.impulse * ratio, -s.shearCap, s.shearCap);
            widthModulusSinkage = Math::max(widthModulusSinkage, z);
        }

        const u32 contacts = side.state.samplesInContact;
        const f32 meanSinkage = contacts > 0 ? sinkageSum / static_cast<f32>(contacts) : 0.0f;
        const f32 compaction = sideSoil ? Terramechanics::compactionResistance(*sideSoil, p.shoeWidth, meanSinkage)
                                        : 0.0f;
        side.compaction.lo = 0.0f;
        side.compaction.hi = 0.0f;
        if (contacts > 0 && compaction > 0.0f)
        {
            centre = centre / static_cast<f32>(contacts);
            Math::vec3 along = forwardHull;
            buildRow(side.compaction, centre, along, 0.0f);
            side.compaction.lo = -duration * compaction;
            side.compaction.hi = duration * compaction;
            side.compaction.impulse = Math::clamp(side.compaction.impulse * ratio, side.compaction.lo, side.compaction.hi);
        }
        else
        {
            side.compaction.impulse = 0.0f;
        }
        side.state.compactionResistance = compaction;
        const f32 internal = p.internalResistance * side.state.normalLoad;
        side.internal.linear = Math::vec3(0.0f);
        side.internal.angular = Math::vec3(0.0f);
        side.internal.belt = 1.0f;
        side.internal.inverseMass = 1.0f / p.beltMass;
        side.internal.lo = -duration * internal;
        side.internal.hi = duration * internal;
        side.internal.impulse = Math::clamp(side.internal.impulse * ratio, side.internal.lo, side.internal.hi);
        (void)widthModulusSinkage;

        JointDrive::Coordinate coordinate;
        coordinate.position = side.beltTravel;
        coordinate.velocity = side.beltSpeed;
        side.driveActive = side.drive.prepare(duration, 1.0f / p.beltMass, coordinate);
        if (side.drive.wantsWake() || fabsf(side.beltSpeed) > 1.0e-3f)
            wakeBodies();
    }
    mPreviousDuration = duration;
}

void TrackedUndercarriage::warmStart()
{
    if (!mHull)
        return;
    for (Side& side : mSides)
    {
        for (u32 k = 0; k < mSampleCount; ++k)
        {
            Sample& s = side.samples[k];
            if (!s.active)
                continue;
            applyRow(s.normal, side, s.normal.impulse);
            applyRow(s.along, side, s.along.impulse);
            applyRow(s.across, side, s.across.impulse);
        }
        if (side.compaction.hi > 0.0f)
            applyRow(side.compaction, side, side.compaction.impulse);
        applyRow(side.internal, side, side.internal.impulse);
        if (side.driveActive)
            side.beltSpeed += side.drive.totalImpulse() / mParameters.beltMass;
    }
}

void TrackedUndercarriage::solveVelocity()
{
    if (!mHull)
        return;
    for (Side& side : mSides)
    {
        for (u32 k = 0; k < mSampleCount; ++k)
        {
            Sample& s = side.samples[k];
            if (!s.active)
                continue;
            solveSoft(s.normal, side);
            // The shear pair on its disk, c + sigma tan phi with the normal impulse this iteration found.
            const f32 sigma = s.normal.impulse / (mDuration * mParameters.shoeWidth * mSampleLength);
            const f32 cap = mDuration * mParameters.shoeWidth * mSampleLength *
                            Terramechanics::shearStrength(*s.soil, sigma);
            s.along.lo = -cap;
            s.along.hi = cap;
            s.across.lo = -cap;
            s.across.hi = cap;
            solveSoft(s.along, side);
            solveSoft(s.across, side);
            const f32 magnitude = sqrtf(s.along.impulse * s.along.impulse + s.across.impulse * s.across.impulse);
            if (magnitude > cap && magnitude > 0.0f)
            {
                const f32 scale = cap / magnitude;
                const f32 along = s.along.impulse * scale;
                const f32 across = s.across.impulse * scale;
                applyRow(s.along, side, along - s.along.impulse);
                applyRow(s.across, side, across - s.across.impulse);
                s.along.impulse = along;
                s.across.impulse = across;
            }
        }
        if (side.compaction.hi > 0.0f)
            solveBounded(side.compaction, side);
        solveBounded(side.internal, side);
        // Drive last: the belt leaves each iteration at the speed it asks for, unless on its force limit.
        if (side.driveActive)
            side.beltSpeed += side.drive.solve(side.beltSpeed) / mParameters.beltMass;
    }
}

// No position correction (every row is soft), but the first call of a step is where the step
// ends for the tracks: the velocities are final.
void TrackedUndercarriage::solvePosition(f32 baumgarte)
{
    (void)baumgarte;
    if (mStepped || !mHull)
        return;
    mStepped = true;
    finishStep();
}

// Shear displacements advance with the slip the belts really had this step; telemetry; the reaction on a dynamic ground.
void TrackedUndercarriage::finishStep()
{
    const TrackParameters& p = mParameters;
    const f32 h = mDuration;
    const f32 inverseStep = h > 0.0f ? 1.0f / h : 0.0f;
    const f32 area = p.shoeWidth * mSampleLength;
    const Math::vec3 forwardHull = mHull->directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f));
    mGroundForce = Math::vec3(0.0f);
    mGroundTorque = Math::vec3(0.0f);
    for (u32 sideIndex = 0; sideIndex < 2; ++sideIndex)
    {
        Side& side = mSides[sideIndex];
        SideState& state = side.state;
        side.beltTravel += side.beltSpeed * h;
        state.beltSpeed = side.beltSpeed;
        state.beltTravel = side.beltTravel;
        state.thrust = 0.0f;
        state.lateralForce = 0.0f;
        state.normalLoad = 0.0f;
        state.meanSinkage = 0.0f;
        state.maxSinkage = 0.0f;
        Math::vec3 centre(0.0f);
        u32 contacts = 0;
        for (u32 k = 0; k < mSampleCount; ++k)
        {
            Sample& s = side.samples[k];
            if (!s.active)
                continue;
            const f32 uAlong = rowVelocity(s.along, side.beltSpeed);
            const f32 uAcross = rowVelocity(s.across, side.beltSpeed);
            Math::vec2 j = s.advected + Math::vec2(uAlong, uAcross) * s.slipTime;
            const f32 limit = kShearDisplacementLimit * Math::max(s.soil->shearModulus, 1.0e-5f);
            const f32 length = Math::length(j);
            if (length > limit)
                j = j * (limit / length);
            s.state.shearDisplacement = j;
            s.lastSlip = Math::vec2(uAlong, uAcross);
            s.state.pressure = Math::max(s.normal.impulse, 0.0f) * inverseStep / area;
            s.state.shearStress = Math::vec2(s.along.impulse, s.across.impulse) * (inverseStep / area);
            s.state.contact = s.normal.impulse > 0.0f;
            const Math::vec3 force =
                (s.state.normal * s.normal.impulse + s.forward * s.along.impulse + s.lateral * s.across.impulse) *
                inverseStep;
            mGroundForce += force;
            mGroundTorque += Math::cross(s.state.point - mHull->position(), force);
            state.thrust += Math::dot(force, forwardHull);
            state.lateralForce += s.across.impulse * inverseStep;
            state.normalLoad += s.normal.impulse * inverseStep;
            if (s.state.contact)
            {
                state.meanSinkage += s.state.sinkage;
                state.maxSinkage = Math::max(state.maxSinkage, s.state.sinkage);
                centre += s.state.point;
                ++contacts;
            }
            // A dynamic ground feels the tracks, a step late.
            if (const BodyEntry* entry = mWorld->body(s.groundBody))
                if (entry->body && entry->body->isDynamic())
                    entry->body->addForceAtPoint(-force, s.state.point);
        }
        if (contacts > 0)
        {
            state.meanSinkage /= static_cast<f32>(contacts);
            centre = centre / static_cast<f32>(contacts);
            state.groundSpeed = Math::dot(mHull->velocityAtPoint(centre), forwardHull);
        }
        else
        {
            state.groundSpeed = Math::dot(mHull->velocity(), forwardHull);
        }
        state.slip = fabsf(side.beltSpeed) > 1.0e-3f ? 1.0f - state.groundSpeed / side.beltSpeed : 0.0f;
        state.internalResistance = fabsf(side.internal.impulse) * inverseStep;
        if (side.compaction.hi > 0.0f)
        {
            const f32 resistance = side.compaction.impulse * inverseStep;
            mGroundForce += side.compaction.linear * resistance;
            mGroundTorque += side.compaction.angular * resistance;
        }
        state.driveForce = side.driveActive ? side.drive.appliedForce() : 0.0f;
    }
}

} // namespace kinetix::Physics
