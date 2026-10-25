#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/GroundVehicle.h"

#include "VehicleSolver.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kRpmToRadians = 2.0f * kPi / 60.0f;
constexpr f32 kGravity = 9.81f;
// Preload fades in over this much compression: touchdown at full droop meets a ramp, not a step.
constexpr f32 kPreloadRamp = 0.01f;
// Fraction of a bottomed wheel's penetration pushed out per step.
constexpr f32 kBottomingBias = 0.2f;
// Floor on |vx| for the kinematic slips, m/s.
constexpr f32 kKinematicSlipSpeed = 1.0f;

f32 positiveOr(f32 value, f32 minimum, f32 fallback)
{
    return isfinite(value) ? Max(value, minimum) : fallback;
}

f32 unit(f32 value)
{
    return isfinite(value) ? Clamp(value, 0.0f, 1.0f) : 0.0f;
}

f32 smoothstep(f32 t)
{
    t = Clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

enum RowCategory : u32
{
    kRowNormal = 1,
    kRowAxlePivot,
    kRowWheelFriction,
    kRowTyreX,
    kRowTyreY,
    kRowEngineFriction,
    kRowClutch,
    kRowGear,
    kRowGearboxLoss,
    kRowDifferential,
    kRowLock
};

u32 rowKey(RowCategory category, u32 index)
{
    return (static_cast<u32>(category) << 24) | (index & 0xFFFFFFu);
}

// atan(t) / t: slip-angle secant correction for using tan(alpha) as the deflection measure.
f32 atanOverArgument(f32 t)
{
    if (fabsf(t) < 1.0e-4f)
        return 1.0f;
    return atanf(t) / t;
}
} // namespace

SuspensionParameters SuspensionParameters::forLoad(f32 staticLoad, f32 frequency, f32 dampingRatio,
                                                   f32 travel, f32 sag)
{
    SuspensionParameters s;
    staticLoad = positiveOr(staticLoad, 1.0f, 3000.0f);
    frequency = positiveOr(frequency, 0.1f, 1.5f);
    dampingRatio = positiveOr(dampingRatio, 0.0f, 0.3f);
    s.travel = positiveOr(travel, 0.01f, 0.2f);
    sag = Clamp(isfinite(sag) ? sag : 0.6f, 0.05f, 0.95f);
    const f32 mass = staticLoad / kGravity;
    const f32 omega = 2.0f * kPi * frequency;
    s.stiffness = mass * omega * omega;
    const f32 target = sag * s.travel;
    if (staticLoad / s.stiffness > target)
    {
        s.stiffness = staticLoad / target;
        s.preload = 0.0f;
    }
    else
    {
        s.preload = staticLoad - s.stiffness * target;
    }
    const f32 critical = 2.0f * sqrtf(s.stiffness * mass);
    s.bumpDamping = 0.7f * dampingRatio * critical;
    s.reboundDamping = 1.3f * dampingRatio * critical;
    s.bumpStopRange = 0.15f * s.travel;
    s.bumpStopStiffness = 10.0f * s.stiffness;
    return s;
}

f32 SuspensionParameters::staticCompression(f32 load) const
{
    if (!(load > 0.0f))
        return 0.0f;
    // Within the preload ramp the spring rises at preload / ramp + k.
    const f32 rampEnd = preload + stiffness * kPreloadRamp;
    f32 x;
    if (load <= rampEnd)
        x = load / (preload / kPreloadRamp + stiffness);
    else
        x = (load - preload) / Max(stiffness, 1.0e-3f);
    return Clamp(x, 0.0f, travel);
}

GroundVehicle::GroundVehicle()
    : mSolver(new VehicleSolver())
{
}

GroundVehicle::~GroundVehicle()
{
    delete mSolver;
}

void GroundVehicle::setBody(RigidBody* body, u32 bodyId)
{
    mBody = body;
    mBodyId = bodyId;
}

void GroundVehicle::setWorld(PhysicsWorld* world)
{
    mWorld = world;
}

void GroundVehicle::setRayFilter(const QueryFilter& filter)
{
    mRayFilter = filter;
}

void GroundVehicle::setSurfaceCallback(SurfaceCallback callback, void* userData)
{
    mSurfaceCallback = callback;
    mSurfaceUserData = userData;
}

u32 GroundVehicle::addAxle(const AxleParameters& axle)
{
    AxleParameters a = axle;
    a.halfTrack = positiveOr(a.halfTrack, 0.01f, 0.75f);
    a.wheel.radius = positiveOr(a.wheel.radius, 0.01f, 0.3f);
    a.wheel.inertia = positiveOr(a.wheel.inertia, 1.0e-3f, 1.0f);
    a.wheel.suspension.travel = positiveOr(a.wheel.suspension.travel, 0.005f, 0.2f);
    a.antiRollStiffness = positiveOr(a.antiRollStiffness, 0.0f, 0.0f);
    a.oscillationLimit = Clamp(isfinite(a.oscillationLimit) ? a.oscillationLimit : 0.17f, 0.0f, 0.6f);
    mAxles.push_back(a);
    mTyres.push_back(Tyre(a.wheel.tyre));
    const u32 index = static_cast<u32>(mAxles.size() - 1);
    for (u32 side = 0; side < 2; ++side)
    {
        WheelState w;
        w.axle = index;
        w.side = side == 0 ? 1.0f : -1.0f;
        mWheels.push_back(w);
    }
    mDrivetrainChecked = false;
    return index;
}

void GroundVehicle::setWheelAngularVelocity(u32 index, f32 omega)
{
    if (index < mWheels.size() && isfinite(omega))
        mWheels[index].omega = omega;
}

void GroundVehicle::stopWheels()
{
    for (WheelState& wheel : mWheels)
        wheel.omega = 0.0f;
    mRoadSpeed = 0.0f;
    if (mGearbox.targetGear() > 0)
        mGearbox.engage(1);
}

void GroundVehicle::setWheelRadiusScale(u32 index, f32 scale)
{
    if (index < mWheels.size() && isfinite(scale) && scale > 0.05f) mWheels[index].radiusScale = scale;
}

void GroundVehicle::setWheelDetached(u32 index, bool detached)
{
    if (index < mWheels.size()) mWheels[index].detached = detached;
}

void GroundVehicle::setSteering(const SteeringParameters& steering)
{
    mSteering.maxAngle = Clamp(positiveOr(steering.maxAngle, 0.0f, 0.6f), 0.0f, 1.2f);
    mSteering.rate = positiveOr(steering.rate, 0.0f, 1.5f);
    mSteering.ackermann = Clamp(isfinite(steering.ackermann) ? steering.ackermann : 1.0f, 0.0f, 1.0f);
}

f32 GroundVehicle::referenceAxlePosition() const
{
    f32 sum = 0.0f;
    u32 count = 0;
    f32 rearmost = 1.0e30f;
    for (const AxleParameters& a : mAxles)
    {
        rearmost = Min(rearmost, a.position);
        if (!a.steered)
        {
            sum += a.position;
            ++count;
        }
    }
    if (count > 0)
        return sum / static_cast<f32>(count);
    return mAxles.empty() ? 0.0f : rearmost;
}

f32 GroundVehicle::wheelbase() const
{
    const f32 reference = referenceAxlePosition();
    f32 best = 0.0f;
    for (const AxleParameters& a : mAxles)
        if (a.steered && fabsf(a.position - reference) > fabsf(best))
            best = a.position - reference;
    return best;
}

f32 GroundVehicle::wheelSteerAngle(u32 index, f32 referenceAngle) const
{
    if (index >= mWheels.size() || !isfinite(referenceAngle))
        return 0.0f;
    const AxleParameters& a = mAxles[mWheels[index].axle];
    if (!a.steered)
        return 0.0f;
    const f32 primary = wheelbase();
    if (fabsf(primary) < 1.0e-3f)
        return referenceAngle;
    const f32 d = a.position - referenceAxlePosition();
    const f32 t = tanf(Clamp(referenceAngle, -1.2f, 1.2f));
    // Every wheel aimed at one centre on the reference line: with signed radius R = primary / t
    // (centre at x = -R for t > 0), a wheel at lateral x needs tan(delta) = d / (x + R).
    const f32 parallel = atanf(d * t / primary);
    const f32 x = mWheels[index].side * a.halfTrack;
    f32 denominator = primary + x * t;
    // A centre between the wheels (absurd locks) would flip the inner wheel; hold it at a right angle.
    if (primary > 0.0f)
        denominator = Max(denominator, 0.05f * primary);
    else
        denominator = Min(denominator, 0.05f * primary);
    const f32 ackermann = atanf(d * t / denominator);
    return parallel + (ackermann - parallel) * mSteering.ackermann;
}

u32 GroundVehicle::addDifferential(const DifferentialParameters& parameters, DriveOutput a, DriveOutput b)
{
    mDifferentials.push_back(Differential(parameters, a, b));
    mDrivetrainChecked = false;
    return static_cast<u32>(mDifferentials.size() - 1);
}

void GroundVehicle::setDriveRoot(s32 differential)
{
    mRoot = differential;
    mDrivetrainChecked = false;
}

bool GroundVehicle::hasDrivetrain() const
{
    return mRoot >= 0 && static_cast<u32>(mRoot) < mDifferentials.size();
}

void GroundVehicle::validateDrivetrain()
{
    mDrivetrainChecked = true;
    mDrivetrainValid = false;
    if (!hasDrivetrain())
        return;
    // Every output must exist and be driven by exactly one differential, and the outputs from
    // the root must reach every differential once (a tree).
    const u32 diffs = static_cast<u32>(mDifferentials.size());
    ct::Vector<u32> wheelUse(mWheels.size(), 0u);
    ct::Vector<u32> diffUse(diffs, 0u);
    for (u32 d = 0; d < diffs; ++d)
    {
        const DriveOutput outputs[2] = {mDifferentials[d].outputA(), mDifferentials[d].outputB()};
        for (const DriveOutput& o : outputs)
        {
            if (o.kind == DriveOutput::Kind::Wheel)
            {
                if (o.index >= mWheels.size())
                {
                    Log::error("GroundVehicle: differential %u drives wheel %u, which does not exist", d, o.index);
                    return;
                }
                ++wheelUse[o.index];
            }
            else if (o.kind == DriveOutput::Kind::Differential)
            {
                if (o.index >= diffs || o.index == d)
                {
                    Log::error("GroundVehicle: differential %u drives differential %u, which is invalid", d,
                               o.index);
                    return;
                }
                ++diffUse[o.index];
            }
            else
            {
                Log::error("GroundVehicle: differential %u has an unconnected output", d);
                return;
            }
        }
    }
    for (u32 w = 0; w < mWheels.size(); ++w)
    {
        if (wheelUse[w] > 1)
        {
            Log::error("GroundVehicle: wheel %u is driven by more than one differential", w);
            return;
        }
    }
    for (u32 d = 0; d < diffs; ++d)
    {
        const u32 expected = static_cast<s32>(d) == mRoot ? 0u : 1u;
        if (diffUse[d] != expected)
        {
            Log::error("GroundVehicle: differential %u is driven %u times (expected %u)", d, diffUse[d], expected);
            return;
        }
    }
    mDrivetrainValid = true;
}

void GroundVehicle::setAutoClutchSpeeds(f32 engageRpm, f32 fullRpm, f32 launchRpm, f32 engageTime)
{
    mAutoEngageRpm = isfinite(engageRpm) && engageRpm > 0.0f ? engageRpm : -1.0f;
    mAutoFullRpm = isfinite(fullRpm) && fullRpm > 0.0f ? fullRpm : -1.0f;
    mAutoLaunchRpm = isfinite(launchRpm) && launchRpm > 0.0f ? launchRpm : -1.0f;
    mAutoEngageTime = positiveOr(engageTime, 0.0f, 0.35f);
}

void GroundVehicle::setAntiLockBrakes(bool enabled, f32 absSlip)
{
    mAbs = enabled;
    mAbsSlip = Clamp(positiveOr(absSlip, 0.01f, 0.12f), 0.01f, 0.9f);
}

void GroundVehicle::setAerodynamics(f32 dragArea, const Math::vec3& centreOfPressure, f32 airDensity)
{
    mDragArea = positiveOr(dragArea, 0.0f, 0.0f);
    mCentreOfPressure = finiteVec(centreOfPressure) ? centreOfPressure : Math::vec3(0.0f);
    mAirDensity = positiveOr(airDensity, 0.0f, 1.225f);
}

void GroundVehicle::setControls(const VehicleControls& controls)
{
    mControls.throttle = unit(controls.throttle);
    mControls.brake = unit(controls.brake);
    mControls.handbrake = unit(controls.handbrake);
    mControls.clutch = unit(controls.clutch);
    mControls.steering = isfinite(controls.steering) ? Clamp(controls.steering, -1.0f, 1.0f) : 0.0f;
    mControls.starter = controls.starter;
}

void GroundVehicle::shiftUp()
{
    mGearbox.shiftUp();
}

void GroundVehicle::shiftDown()
{
    mGearbox.shiftDown();
}

void GroundVehicle::selectGear(s32 gear)
{
    mGearbox.selectGear(gear);
}

void GroundVehicle::setDifferentialLock(u32 differential, bool locked)
{
    if (differential >= mDifferentials.size())
    {
        Log::error("GroundVehicle: no differential %u to lock", differential);
        return;
    }
    mDifferentials[differential].setLocked(locked);
}

void GroundVehicle::toggleDifferentialLock(u32 differential)
{
    if (differential < mDifferentials.size())
        setDifferentialLock(differential, !mDifferentials[differential].locked());
}

void GroundVehicle::stepCallback(f32 step, void* userData)
{
    if (userData)
        static_cast<GroundVehicle*>(userData)->applyForces(step);
}

void GroundVehicle::updateSteering(f32 step)
{
    const f32 target = mControls.steering * mSteering.maxAngle;
    const f32 limit = mSteering.rate * step;
    mSteerAngle += Clamp(target - mSteerAngle, -limit, limit);
}

void GroundVehicle::probeWheels(f32 step)
{
    (void)step;
    const RigidBody& body = *mBody;
    const Math::vec3 down = body.directionToWorld(Math::vec3(0.0f, -1.0f, 0.0f));
    const Math::vec3 up = -down;
    QueryFilter filter = mRayFilter;
    filter.ignoredBody = mBodyId;

    for (u32 i = 0; i < mWheels.size(); ++i)
    {
        WheelState& w = mWheels[i];
        const AxleParameters& a = mAxles[w.axle];
        const WheelParameters& wp = a.wheel;
        const f32 r = wp.radius * w.radiusScale;
        const f32 travel = wp.suspension.travel;
        w.grounded = false;
        w.bottomed = false;
        w.telemetry.groundBody = 0xFFFFFFFFu;
        w.down = down;
        w.hardPoint = body.pointToWorld(Math::vec3(w.side * a.halfTrack, a.height, a.position));
        const f32 delta = wheelSteerAngle(i, mSteerAngle);
        w.telemetry.steerAngle = delta;
        w.heading = body.directionToWorld(Math::vec3(-sinf(delta), 0.0f, cosf(delta)));
        if (w.detached)
        {
            w.length = 1.0e6f;
            continue;
        }

        // Top of the wheel at full bump down past full droop; a rolling beam axle reaches further.
        const f32 extra = a.oscillating ? a.halfTrack * tanf(a.oscillationLimit) : 0.0f;
        Ray ray;
        ray.origin = w.hardPoint - down * r;
        ray.direction = down;
        WorldRayHit hit;
        const bool found = mWorld->raycast(ray, 2.0f * r + travel + extra, filter, hit);
        w.length = 1.0e6f;
        if (!found)
            continue;
        Math::vec3 normal = hit.normal;
        if (Math::dot(normal, up) < 0.0f)
            normal = -normal;
        // A wall, not a road.
        if (Math::dot(normal, up) < 0.15f)
            continue;
        w.length = hit.distance - 2.0f * r;
        w.normal = normal;
        w.point = hit.point;
        w.cosine = Max(Math::dot(normal, up), 0.15f);
        w.groundVelocity = Math::vec3(0.0f);
        w.telemetry.groundBody = hit.body;
        if (const BodyEntry* entry = mWorld->body(hit.body))
        {
            if (entry->body && entry->body->bodyType() != BodyType::Static)
                w.groundVelocity = entry->body->velocityAtPoint(hit.point);
        }
        w.telemetry.surface = mSurfaceCallback ? mSurfaceCallback(hit.body, hit.point, mSurfaceUserData)
                                               : mDefaultSurface;
        // Non-oscillating wheels decide contact here, beam axles in shareOscillatingAxles().
        if (!a.oscillating)
            w.grounded = w.length <= travel;
    }
}

void GroundVehicle::shareOscillatingAxles()
{
    mPairRows.clear();
    u32 oscillatingIndex = 0;
    for (u32 ai = 0; ai < mAxles.size(); ++ai)
    {
        const AxleParameters& a = mAxles[ai];
        if (!a.oscillating)
            continue;
        WheelState& left = mWheels[2 * ai];
        WheelState& right = mWheels[2 * ai + 1];
        const f32 travel = a.wheel.suspension.travel;
        // Compressions with the beam parallel to the body (a wheel with no ground reads hugely
        // negative), then the beam rolls by e to equalise them, up to the stop.
        const f32 xl = travel - left.length;
        const f32 xr = travel - right.length;
        const f32 limit = a.halfTrack * tanf(a.oscillationLimit);
        const f32 e = Clamp(0.5f * (xl - xr), -limit, limit);
        left.compression = xl - e;
        right.compression = xr + e;
        left.length = travel - left.compression;
        right.length = travel - right.compression;
        left.grounded = left.compression >= 0.0f && left.length < 1.0e5f;
        right.grounded = right.compression >= 0.0f && right.length < 1.0e5f;
        const bool shared = left.grounded && right.grounded && fabsf(0.5f * (xl - xr)) < limit;
        mPairRows.push_back(shared ? 1 : 0);
        if (oscillatingIndex < 4)
            mTelemetry.axleRoll[oscillatingIndex] = atanf(e / a.halfTrack);
        ++oscillatingIndex;
    }
}

void GroundVehicle::computeStruts()
{
    const RigidBody& body = *mBody;
    for (u32 i = 0; i < mWheels.size(); ++i)
    {
        WheelState& w = mWheels[i];
        const AxleParameters& a = mAxles[w.axle];
        const SuspensionParameters& s = a.wheel.suspension;
        if (!a.oscillating)
            w.compression = s.travel - w.length;
        w.strutForce = 0.0f;
        w.strutStiffness = 0.0f;
        w.strutDamping = 0.0f;
        w.loadEstimate = 0.0f;
        if (!w.grounded)
            continue;
        w.bottomed = w.length < 0.0f;
        const f32 x = Clamp(w.compression, 0.0f, s.travel);
        f32 force = s.stiffness * x;
        f32 stiffness = s.stiffness;
        if (x < kPreloadRamp)
        {
            force += s.preload * x / kPreloadRamp;
            stiffness += s.preload / kPreloadRamp;
        }
        else
        {
            force += s.preload;
        }
        const f32 length = Clamp(w.length, 0.0f, s.travel);
        if (s.bumpStopRange > 0.0f && length < s.bumpStopRange)
        {
            const f32 depth = s.bumpStopRange - length;
            force += 0.5f * s.bumpStopStiffness * depth * depth / s.bumpStopRange;
            stiffness += s.bumpStopStiffness * depth / s.bumpStopRange;
        }
        // Anti-roll bar: the other wheel of the axle counts as full droop if airborne.
        if (!a.oscillating && a.antiRollStiffness > 0.0f)
        {
            const WheelState& other = mWheels[i ^ 1u];
            const f32 otherX = other.grounded ? Clamp(other.compression, 0.0f, s.travel) : 0.0f;
            force += a.antiRollStiffness * (x - otherX);
            stiffness += a.antiRollStiffness;
        }
        const Math::vec3 velocity = body.velocityAtPoint(w.point) - w.groundVelocity;
        const f32 rate = -Math::dot(velocity, w.normal) / w.cosine;
        w.strutForce = force;
        w.strutStiffness = stiffness;
        w.strutDamping = rate > 0.0f ? s.bumpDamping : s.reboundDamping;
        w.loadEstimate = Max(force + w.strutDamping * rate, 0.0f) / w.cosine;
        if (w.bottomed)
            w.loadEstimate = Max(w.loadEstimate, w.lastLoad);
    }
}

f32 GroundVehicle::noSlipVelocity(DriveOutput output, u32 depth) const
{
    // Shaft speed if no driven tyre slipped: grounded wheels at ground speed over radius,
    // differentials by kinematics, airborne wheels as turning.
    if (depth > mDifferentials.size())
        return 0.0f;
    if (output.kind == DriveOutput::Kind::Wheel && output.index < mWheels.size())
    {
        const WheelState& w = mWheels[output.index];
        const f32 radius = mAxles[w.axle].wheel.radius * w.radiusScale;
        if (w.grounded)
            return w.vx / radius;
        if (!mBody)
            return w.omega;
        return Math::dot(mBody->velocity(), mBody->directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f))) / radius;
    }
    if (output.kind == DriveOutput::Kind::Differential && output.index < mDifferentials.size())
    {
        const Differential& d = mDifferentials[output.index];
        const DifferentialParameters& p = d.parameters();
        return p.ratio * (p.split * noSlipVelocity(d.outputA(), depth + 1) +
                          (1.0f - p.split) * noSlipVelocity(d.outputB(), depth + 1));
    }
    return 0.0f;
}

bool GroundVehicle::drivenSlipping(DriveOutput output, u32 depth) const
{
    if (depth > mDifferentials.size())
        return false;
    if (output.kind == DriveOutput::Kind::Wheel && output.index < mWheels.size())
    {
        const WheelState& w = mWheels[output.index];
        return !w.grounded || fabsf(w.telemetry.slipRatio) > mAxles[w.axle].wheel.tyre.peakSlipRatio;
    }
    if (output.kind == DriveOutput::Kind::Differential && output.index < mDifferentials.size())
    {
        const Differential& d = mDifferentials[output.index];
        return drivenSlipping(d.outputA(), depth + 1) || drivenSlipping(d.outputB(), depth + 1);
    }
    return false;
}

bool GroundVehicle::drivenGrounded(DriveOutput output, u32 depth) const
{
    if (depth > mDifferentials.size())
        return false;
    if (output.kind == DriveOutput::Kind::Wheel && output.index < mWheels.size())
        return mWheels[output.index].grounded;
    if (output.kind == DriveOutput::Kind::Differential && output.index < mDifferentials.size())
    {
        const Differential& d = mDifferentials[output.index];
        return drivenGrounded(d.outputA(), depth + 1) || drivenGrounded(d.outputB(), depth + 1);
    }
    return false;
}

void GroundVehicle::updateClutchAndGearbox(f32 step, f32 throttle)
{
    // Shift on road speed, as a controller reading wheel sensors: a spinning driven wheel must
    // not read as reaching the shift point. With every driven wheel off the ground there is no
    // road speed to read, so the box keeps the one it had and does not climb the gears in a jump.
    // Off the ground a wheel reads the chassis speed, a speedometer, so a box
    // that reads a stale value after a crash does not pick a gear for a speed
    // the car no longer has.
    const DriveOutput root = DriveOutput::differential(static_cast<u32>(mRoot));
    mRoadSpeed = noSlipVelocity(root, 0);
    // No upshift while a driven wheel slips or is in the air, or the engine
    // is slowing: a wheel spinning free, or a jump, reads as a shift point.
    // At the limiter the engine cannot read any higher, so wheelspin no longer
    // disguises the shift point: the box shifts, or the car sits on the cut.
    const f32 rpm = mEngine.rpm();
    const EngineParameters& e = mEngine.parameters();
    const bool atLimiter = rpm >= e.limiterRpm - e.limiterBand;
    const bool canShiftUp = (atLimiter || !drivenSlipping(root, 0)) && rpm >= mLastEngineRpm;
    mLastEngineRpm = rpm;
    mGearbox.update(step, mRoadSpeed, throttle, canShiftUp);

    const f32 pedal = 1.0f - mControls.clutch;
    f32 engagement = pedal;
    if (mAutoClutch || mGearbox.mode() == GearboxMode::Automatic)
    {
        f32 automatic = 0.0f;
        if (!mGearbox.shifting() && mGearbox.gear() != 0)
        {
            const EngineParameters& e = mEngine.parameters();
            const f32 engage = mAutoEngageRpm > 0.0f ? mAutoEngageRpm : 1.15f * e.idleRpm;
            const f32 light = mAutoFullRpm > 0.0f ? mAutoFullRpm : e.idleRpm + 0.15f * (e.limiterRpm - e.idleRpm);
            const f32 launch =
                mAutoLaunchRpm > 0.0f ? mAutoLaunchRpm : e.idleRpm + 0.45f * (e.limiterRpm - e.idleRpm);
            // Band the engine is held in while the clutch slips: engage..light at closed throttle up to
            // the upper half of engage..launch at full throttle.
            const f32 full = Max(light + (launch - light) * throttle, engage + 10.0f);
            const f32 start = engage + (full - engage) * 0.5f * throttle;
            // Closing cannot stall the engine when either side already turns fast enough.
            const f32 inputRpm = fabsf(mGearbox.inputAngularVelocity()) / kRpmToRadians;
            const f32 speed = Max(mEngine.rpm(), inputRpm);
            const f32 bySpeed = smoothstep((speed - start) / (full - start));
            // Once the gearbox input alone keeps the engine above engage speed, let the clutch out the
            // rest of the way over about a second.
            if (inputRpm > engage)
                mAutoClosing = Min(mAutoClosing + step, 1.0f);
            else if (inputRpm < 0.9f * engage)
                mAutoClosing = 0.0f;
            const f32 ramp =
                mAutoEngageTime > 0.0f ? Clamp(mGearbox.timeSinceShift() / mAutoEngageTime, 0.0f, 1.0f) : 1.0f;
            automatic = Max(bySpeed, smoothstep(mAutoClosing)) * ramp;
        }
        else
        {
            mAutoClosing = 0.0f;
        }
        engagement = Min(pedal, automatic);
    }
    mClutch.setEngagement(engagement);
}

void GroundVehicle::buildRows(f32 step, bool firstPass)
{
    (void)firstPass;
    VehicleSolver& solver = *mSolver;
    const RigidBody& body = *mBody;
    const Math::vec3 com = body.position();
    const Math::vec3 bodyUp = body.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    const bool powered = mDrivetrainValid;
    const u32 diffs = static_cast<u32>(mDifferentials.size());

    // DOF layout: engine, gearbox input, differential inputs, then wheels.
    mEngineDof = 0;
    mInputDof = 1;
    mDifferentialDof = 2;
    const u32 wheelDof = powered ? 2 + diffs : 0;
    const u32 dofCount = wheelDof + static_cast<u32>(mWheels.size());
    solver.reset(dofCount);

    Math::vec3 gravity(0.0f, -kGravity, 0.0f);
    if (mWorld)
        gravity = mWorld->gravity();
    Math::vec3 aeroForce(0.0f);
    Math::vec3 aeroTorque(0.0f);
    if (mDragArea > 0.0f)
    {
        const Math::vec3 point = body.pointToWorld(mCentreOfPressure);
        const Math::vec3 v = body.velocityAtPoint(point);
        aeroForce = v * (-0.5f * mAirDensity * mDragArea * Math::length(v));
        aeroTorque = Math::cross(point - com, aeroForce);
    }
    mAeroForce = aeroForce;
    mAeroTorque = aeroTorque;
    // Forces already on the body this step (user force, an aircraft's wings) are in its
    // accumulators and predicted along with gravity.
    const f32 inverseMass = body.inverseMass();
    const Math::mat3& inverseInertia = body.inverseInertiaTensorWorld();
    const Math::vec3 external = body.forceAccumulator() + aeroForce;
    const Math::vec3 externalTorque = body.torqueAccumulator() + aeroTorque;
    solver.setChassis(inverseMass, inverseInertia, body.velocity() + (gravity + external * inverseMass) * step,
                      body.angularVelocity() + (inverseInertia * externalTorque) * step);

    if (powered)
    {
        solver.setDof(mEngineDof, 1.0f / mEngine.parameters().inertia,
                      mEngine.angularVelocity() + mCombustion / mEngine.parameters().inertia * step);
        solver.setDof(mInputDof, 1.0f / mGearbox.parameters().inputInertia, mGearbox.inputAngularVelocity());
        for (u32 d = 0; d < diffs; ++d)
            solver.setDof(mDifferentialDof + d, 1.0f / mDifferentials[d].parameters().inputInertia,
                          mDifferentials[d].inputAngularVelocity());
    }
    for (u32 i = 0; i < mWheels.size(); ++i)
    {
        WheelState& w = mWheels[i];
        w.dof = wheelDof + i;
        solver.setDof(w.dof, 1.0f / mAxles[w.axle].wheel.inertia, w.omega);
    }

    u32 oscillatingIndex = 0;
    for (u32 ai = 0; ai < mAxles.size(); ++ai)
    {
        const AxleParameters& a = mAxles[ai];
        bool shared = false;
        if (a.oscillating)
        {
            shared = mPairRows[oscillatingIndex] != 0;
            ++oscillatingIndex;
        }
        WheelState& left = mWheels[2 * ai];
        WheelState& right = mWheels[2 * ai + 1];
        left.normalRow = -1;
        right.normalRow = -1;
        if (shared && !left.bottomed && !right.bottomed)
        {
            // Beam on its pivot: one strut carrying both wheels' springs, load split equally.
            SolverRow row;
            row.kind = SolverRow::Kind::Soft;
            row.linear = (left.normal + right.normal) * 0.5f;
            row.angular = (Math::cross(left.point - com, left.normal) + Math::cross(right.point - com, right.normal)) *
                          0.5f;
            row.offset = 0.5f * (Math::dot(left.normal, left.groundVelocity) +
                                 Math::dot(right.normal, right.groundVelocity));
            const f32 cosine = 0.5f * (left.cosine + right.cosine);
            const f32 force = left.strutForce + right.strutForce;
            const f32 stiffness = left.strutStiffness + right.strutStiffness;
            const f32 damping = left.strutDamping + right.strutDamping;
            row.key = rowKey(kRowAxlePivot, ai);
            row.softA = step * force / cosine;
            row.softB = (stiffness * step + damping) * step / (cosine * cosine);
            row.lo = 0.0f;
            const s32 index = static_cast<s32>(solver.addRow(row));
            left.normalRow = index;
            right.normalRow = index;
            const f32 load = 0.5f * (left.loadEstimate + right.loadEstimate);
            left.loadEstimate = load;
            right.loadEstimate = load;
            continue;
        }
        for (u32 side = 0; side < 2; ++side)
        {
            WheelState& w = mWheels[2 * ai + side];
            if (!w.grounded)
                continue;
            SolverRow row;
            row.key = rowKey(kRowNormal, 2 * ai + side);
            row.linear = w.normal;
            row.angular = Math::cross(w.point - com, w.normal);
            row.offset = Math::dot(w.normal, w.groundVelocity);
            row.lo = 0.0f;
            if (w.bottomed)
            {
                // Past full bump: hard contact pushing out the penetration, never with less than the spring
                // and bump stop force at full bump, or a bottomed body would sit on its stops.
                row.kind = SolverRow::Kind::Rigid;
                row.target = Min(kBottomingBias * (-w.length) * w.cosine / step, 2.0f);
                row.lo = Max(step * w.strutForce / w.cosine, 0.0f);
            }
            else
            {
                row.kind = SolverRow::Kind::Soft;
                row.softA = step * w.strutForce / w.cosine;
                row.softB = (w.strutStiffness * step + w.strutDamping) * step / (w.cosine * w.cosine);
            }
            w.normalRow = static_cast<s32>(solver.addRow(row));
        }
    }

    for (u32 i = 0; i < mWheels.size(); ++i)
    {
        WheelState& w = mWheels[i];
        w.rowX = -1;
        w.rowY = -1;
        SolverRow friction;
        friction.key = rowKey(kRowWheelFriction, i);
        friction.kind = SolverRow::Kind::Rigid;
        friction.addDof(w.dof, 1.0f);
        w.frictionRow = static_cast<s32>(solver.addRow(friction));
        if (!w.grounded)
            continue;
        const f32 r = mAxles[w.axle].wheel.radius * w.radiusScale;
        const Math::vec3 arm = w.point - com;
        SolverRow x;
        x.key = rowKey(kRowTyreX, i);
        x.kind = SolverRow::Kind::Soft;
        x.linear = w.forward;
        x.angular = Math::cross(arm, w.forward);
        x.addDof(w.dof, -r);
        x.offset = Math::dot(w.forward, w.groundVelocity);
        const Math::vec3 lateralArm =
            arm - bodyUp * (Math::dot(bodyUp, arm) * (1.0f - mAxles[w.axle].rollInfluence));
        SolverRow y;
        y.key = rowKey(kRowTyreY, i);
        y.kind = SolverRow::Kind::Soft;
        y.linear = w.lateral;
        y.angular = Math::cross(lateralArm, w.lateral);
        y.offset = Math::dot(w.lateral, w.groundVelocity);
        w.rowX = static_cast<s32>(solver.addRow(x));
        w.rowY = static_cast<s32>(solver.addRow(y));
        solver.row(static_cast<u32>(w.rowX)).partner = w.rowY;
        solver.row(static_cast<u32>(w.rowY)).partner = w.rowX;
    }

    mEngineFrictionRow = -1;
    mClutchRow = -1;
    mGearRow = -1;
    mGearboxLossRow = -1;
    mDifferentialRows.clear();
    mLockRows.clear();
    if (!powered)
        return;
    {
        SolverRow row;
        row.key = rowKey(kRowEngineFriction, 0);
        row.addDof(mEngineDof, 1.0f);
        const f32 cap = mEngine.frictionTorque(mEngine.rpm()) * step;
        row.lo = -cap;
        row.hi = cap;
        mEngineFrictionRow = static_cast<s32>(solver.addRow(row));
    }
    if (mClutch.capacity() > 0.0f)
    {
        SolverRow row;
        row.key = rowKey(kRowClutch, 0);
        row.addDof(mEngineDof, 1.0f);
        row.addDof(mInputDof, -1.0f);
        const f32 cap = mClutch.capacity() * step;
        row.lo = -cap;
        row.hi = cap;
        mClutchRow = static_cast<s32>(solver.addRow(row));
    }
    const f32 ratio = mGearbox.ratio();
    if (ratio != 0.0f)
    {
        SolverRow row;
        row.key = rowKey(kRowGear, static_cast<u32>(mGearbox.gear() + 64));
        row.addDof(mInputDof, 1.0f);
        row.addDof(mDifferentialDof + static_cast<u32>(mRoot), -ratio);
        mGearRow = static_cast<s32>(solver.addRow(row));
    }
    {
        const GearboxParameters& g = mGearbox.parameters();
        const f32 loss = ratio != 0.0f ? (1.0f - g.efficiency) * fabsf(mGearbox.torque()) : 0.0f;
        SolverRow row;
        row.key = rowKey(kRowGearboxLoss, 0);
        row.addDof(mInputDof, 1.0f);
        const f32 cap = (loss + g.dragTorque) * step;
        row.lo = -cap;
        row.hi = cap;
        mGearboxLossRow = static_cast<s32>(solver.addRow(row));
    }
    for (u32 d = 0; d < diffs; ++d)
    {
        const Differential& diff = mDifferentials[d];
        const DifferentialParameters& p = diff.parameters();
        auto dofOf = [&](DriveOutput o) -> u32
        {
            return o.kind == DriveOutput::Kind::Wheel ? wheelDof + o.index : mDifferentialDof + o.index;
        };
        const u32 a = dofOf(diff.outputA());
        const u32 b = dofOf(diff.outputB());
        SolverRow kinematic;
        kinematic.key = rowKey(kRowDifferential, d);
        kinematic.addDof(mDifferentialDof + d, 1.0f / p.ratio);
        kinematic.addDof(a, -p.split);
        kinematic.addDof(b, -(1.0f - p.split));
        mDifferentialRows.push_back(static_cast<s32>(solver.addRow(kinematic)));

        s32 lockRow = -1;
        SolverRow lock;
        lock.key = rowKey(kRowLock, d);
        lock.addDof(a, 1.0f);
        lock.addDof(b, -1.0f);
        if (diff.rigid())
        {
            lockRow = static_cast<s32>(solver.addRow(lock));
        }
        else if (p.type == DifferentialType::ClutchLimitedSlip)
        {
            const f32 carrier = fabsf(diff.inputTorque()) * p.ratio;
            const f32 cap = (p.preloadTorque + p.torqueSensing * carrier) * step;
            if (cap > 0.0f)
            {
                lock.lo = -cap;
                lock.hi = cap;
                lockRow = static_cast<s32>(solver.addRow(lock));
            }
        }
        else if (p.type == DifferentialType::ViscousLimitedSlip && p.viscousCoefficient > 0.0f)
        {
            lock.kind = SolverRow::Kind::Soft;
            lock.softA = 0.0f;
            lock.softB = p.viscousCoefficient * step;
            lockRow = static_cast<s32>(solver.addRow(lock));
        }
        mLockRows.push_back(lockRow);
    }
}

void GroundVehicle::updateTyreRows(f32 step, bool firstPass)
{
    VehicleSolver& solver = *mSolver;
    for (u32 i = 0; i < mWheels.size(); ++i)
    {
        WheelState& w = mWheels[i];
        const AxleParameters& a = mAxles[w.axle];
        const WheelParameters& wp = a.wheel;
        const Tyre& tyre = mTyres[w.axle];
        const TyreParameters& tp = tyre.parameters();
        f32 load = 0.0f;
        if (w.grounded)
        {
            if (firstPass || w.normalRow < 0)
            {
                // Last step's solved load while the wheel stays down, else the spring's estimate.
                load = w.wasGrounded && w.lastLoad > 0.0f ? w.lastLoad : w.loadEstimate;
            }
            else
            {
                const SolverRow& n = solver.row(static_cast<u32>(w.normalRow));
                const bool shared = (i % 2 == 0) ? mWheels[i + 1].normalRow == w.normalRow
                                                 : mWheels[i - 1].normalRow == w.normalRow;
                load = Max(n.impulse, 0.0f) / step * (shared ? 0.5f : 1.0f);
            }
        }
        w.telemetry.load = load;

        const f32 rolling = w.grounded ? tyre.rollingResistanceCoefficient(w.telemetry.surface) * load * wp.radius * w.radiusScale
                                       : 0.0f;
        const f32 frictionTorque = w.brakeTorque + rolling + wp.bearingFriction;
        SolverRow& friction = solver.row(static_cast<u32>(w.frictionRow));
        friction.lo = -frictionTorque * step;
        friction.hi = frictionTorque * step;
        if (!w.grounded || w.rowX < 0)
            continue;

        const f32 sigmaX = tp.longitudinalRelaxation;
        const f32 sigmaY = tp.lateralRelaxation;
        const f32 slipRatio = w.deflectionX / sigmaX;
        const f32 tanAlpha = w.deflectionY / sigmaY;
        const f32 slipAngle = atanf(tanAlpha);
        f32 kx = 0.0f;
        f32 ky = 0.0f;
        tyre.secantStiffness(load, slipRatio, slipAngle, w.telemetry.surface, w.camber, kx, ky);
        const f32 stiffnessX = kx / sigmaX;
        const f32 stiffnessY = ky * atanOverArgument(tanAlpha) / sigmaY;
        const f32 speed = fabsf(w.vx);
        const f32 damping = tp.lowSpeedDamping * load * Max(1.0f - speed / tp.lowSpeedDampingFade, 0.0f);
        const f32 relaxX = 1.0f / (1.0f + step * speed / sigmaX);
        const f32 relaxY = 1.0f / (1.0f + step * speed / sigmaY);
        const f32 camberForce = ky * tp.camberRatio * w.camber;

        const f32 mu = tyre.frictionCoefficient(load, w.telemetry.surface);
        SolverRow& x = solver.row(static_cast<u32>(w.rowX));
        const f32 gainX = stiffnessX * step + damping;
        x.softA = (gainX * relaxX - damping) * w.deflectionX;
        x.softB = gainX * relaxX * step;
        x.ellipseCap = mu * load * step;
        SolverRow& y = solver.row(static_cast<u32>(w.rowY));
        const f32 gainY = stiffnessY * step + damping;
        y.softA = (gainY * relaxY - damping) * w.deflectionY + camberForce * step;
        y.softB = gainY * relaxY * step;
        y.ellipseCap = mu * tp.lateralFrictionRatio * load * step;
    }
}

void GroundVehicle::applyForces(f32 step)
{
    if (!mBody || !mWorld || !(step > 0.0f) || !isfinite(step) || mAxles.empty())
        return;
    if (!mBody->isDynamic())
        return;
    if (!mDrivetrainChecked)
        validateDrivetrain();
    for (u32 ai = 0; ai < mAxles.size(); ++ai)
        mTyres[ai].setParameters(mAxles[ai].wheel.tyre);

    mAppliedBrake = mBrakeSource ? mBrakeSource->controls().brake : mControls.brake;
    updateSteering(step);
    probeWheels(step);
    shareOscillatingAxles();
    computeStruts();

    const RigidBody& body = *mBody;
    const Math::vec3 up = body.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    for (u32 i = 0; i < mWheels.size(); ++i)
    {
        WheelState& w = mWheels[i];
        const AxleParameters& a = mAxles[w.axle];
        if (w.grounded)
        {
            Math::vec3 forward = w.heading - w.normal * Math::dot(w.normal, w.heading);
            const f32 length = Math::length(forward);
            if (length > 1.0e-6f)
                forward = forward / length;
            else
                forward = Math::normalize(Math::cross(body.directionToWorld(Math::vec3(1.0f, 0.0f, 0.0f)), w.normal));
            w.forward = forward;
            w.lateral = Math::cross(w.normal, forward);
            const Math::vec3 velocity = body.velocityAtPoint(w.point) - w.groundVelocity;
            w.vx = Math::dot(velocity, w.forward);
            w.vy = Math::dot(velocity, w.lateral);
            // Static camber (outward positive, so flipped on the right) plus the body's lean.
            w.camber = w.side * a.wheel.camber + asinf(Clamp(Math::dot(up, w.lateral), -1.0f, 1.0f));
        }
        else
        {
            w.vx = 0.0f;
            w.vy = 0.0f;
            w.camber = 0.0f;
        }
        const f32 reference = Max(fabsf(w.vx), kKinematicSlipSpeed);
        w.telemetry.kinematicSlipRatio = w.grounded ? (w.omega * (a.wheel.radius * w.radiusScale) - w.vx) / reference : 0.0f;
        w.telemetry.kinematicSlipAngle = w.grounded ? atanf(-w.vy / reference) : 0.0f;

        // Carcass deflection over the last step from what the patch really did: velocities now are
        // the world's end-of-step ones, joints and contacts included.
        if (w.grounded && w.wasGrounded && w.lastStep > 0.0f)
        {
            const TyreParameters& tp = mTyres[w.axle].parameters();
            const f32 limitX = tp.maxSlipRatio * tp.longitudinalRelaxation;
            const f32 limitY = tanf(tp.maxSlipAngle) * tp.lateralRelaxation;
            const f32 slipX = w.vx - w.omega * (a.wheel.radius * w.radiusScale);
            w.deflectionX = Clamp(w.lastRelaxX * (w.deflectionX - w.lastStep * slipX), -limitX, limitX);
            w.deflectionY = Clamp(w.lastRelaxY * (w.deflectionY - w.lastStep * w.vy), -limitY, limitY);
        }
        else if (!w.grounded)
        {
            w.deflectionX = 0.0f;
            w.deflectionY = 0.0f;
        }

        if (mAbs && w.grounded && mAppliedBrake > 0.0f && fabsf(w.vx) > 2.0f)
        {
            const f32 slip = w.telemetry.kinematicSlipRatio * (w.vx >= 0.0f ? 1.0f : -1.0f);
            if (slip < -mAbsSlip)
                w.absPressure = Max(w.absPressure - step / 0.04f, 0.0f);
            else if (slip > -0.5f * mAbsSlip)
                w.absPressure = Min(w.absPressure + step / 0.12f, 1.0f);
        }
        else
        {
            w.absPressure = 1.0f;
        }
        w.brakeTorque = mAppliedBrake * a.wheel.maxBrakeTorque * w.absPressure +
                        mControls.handbrake * a.wheel.maxHandbrakeTorque;
    }

    mCombustion = 0.0f;
    if (mDrivetrainValid)
    {
        const bool automated = mAutoClutch || mGearbox.mode() == GearboxMode::Automatic;
        updateClutchAndGearbox(step, mControls.throttle);
        // An automated shift lifts the throttle and brings it back in with the clutch.
        f32 throttle = mControls.throttle;
        if (automated && mGearbox.shifting())
            throttle = 0.0f;
        else if (automated && mAutoEngageTime > 0.0f)
            throttle *= Clamp(mGearbox.timeSinceShift() / mAutoEngageTime, 0.0f, 1.0f);
        mCombustion = mEngine.update(throttle, mControls.starter);
    }

    // Pass one builds the tyre limits and rolling resistance on last step's loads, starting
    // from last step's impulses when the rows are the same ones; a second pass follows if the
    // loads were off.
    buildRows(step, true);
    updateTyreRows(step, true);
    VehicleSolver& solver = *mSolver;
    const u32 rows = solver.rowCount();
    bool same = rows == mLastKeys.size() && mLastStep > 0.0f;
    for (u32 i = 0; same && i < rows; ++i)
        same = solver.row(i).key == mLastKeys[i];
    if (same)
    {
        const f32 scale = step / mLastStep;
        for (u32 i = 0; i < rows; ++i)
            solver.row(i).impulse = mLastImpulses[i] * scale;
    }
    solver.solve(same);
    bool again = false;
    for (u32 i = 0; i < mWheels.size() && !again; ++i)
    {
        const WheelState& w = mWheels[i];
        if (!w.grounded || w.normalRow < 0)
            continue;
        const bool shared = (i % 2 == 0) ? mWheels[i + 1].normalRow == w.normalRow
                                         : mWheels[i - 1].normalRow == w.normalRow;
        const f32 solved = Max(solver.row(static_cast<u32>(w.normalRow)).impulse, 0.0f) / step * (shared ? 0.5f : 1.0f);
        const f32 assumed = w.telemetry.load;
        again = fabsf(solved - assumed) > 0.05f * Max(solved, assumed) + 100.0f;
    }
    if (again)
    {
        updateTyreRows(step, false);
        solver.solve(true, true);
    }
    mLastKeys.resize(rows);
    mLastImpulses.resize(rows);
    for (u32 i = 0; i < rows; ++i)
    {
        mLastKeys[i] = solver.row(i).key;
        mLastImpulses[i] = solver.row(i).impulse;
    }
    mLastStep = step;

    finishStep(step);
}

void GroundVehicle::finishStep(f32 step)
{
    VehicleSolver& solver = *mSolver;
    RigidBody& body = *mBody;
    const Math::vec3 com = body.position();
    const f32 inverseStep = 1.0f / step;

    if (mDrivetrainValid)
    {
        mEngine.setAngularVelocity(solver.dofVelocity(mEngineDof));
        mGearbox.setInputAngularVelocity(solver.dofVelocity(mInputDof));
        for (u32 d = 0; d < mDifferentials.size(); ++d)
        {
            Differential& diff = mDifferentials[d];
            diff.setInputAngularVelocity(solver.dofVelocity(mDifferentialDof + d));
            const SolverRow& kinematic = solver.row(static_cast<u32>(mDifferentialRows[d]));
            const f32 inputTorque = kinematic.impulse / diff.parameters().ratio * inverseStep;
            const f32 locking =
                mLockRows[d] >= 0 ? solver.row(static_cast<u32>(mLockRows[d])).impulse * inverseStep : 0.0f;
            diff.setSolved(inputTorque, locking);
        }
        if (mClutchRow >= 0)
        {
            const SolverRow& row = solver.row(static_cast<u32>(mClutchRow));
            const f32 cap = row.hi;
            const bool locked = fabsf(row.impulse) < cap * (1.0f - 1.0e-4f);
            mClutch.setSolved(-row.impulse * inverseStep,
                              mEngine.angularVelocity() - mGearbox.inputAngularVelocity(), locked);
        }
        else
        {
            mClutch.setSolved(0.0f, mEngine.angularVelocity() - mGearbox.inputAngularVelocity(), false);
        }
        mGearbox.setSolvedTorque(mGearRow >= 0 ? solver.row(static_cast<u32>(mGearRow)).impulse * inverseStep : 0.0f);
    }

    Math::vec3 force = solver.chassisLinearImpulse() * inverseStep;
    Math::vec3 torque = solver.chassisAngularImpulse() * inverseStep;
    force += mAeroForce;
    torque += mAeroTorque;

    for (u32 i = 0; i < mWheels.size(); ++i)
    {
        WheelState& w = mWheels[i];
        const AxleParameters& a = mAxles[w.axle];
        const WheelParameters& wp = a.wheel;
        const TyreParameters& tp = mTyres[w.axle].parameters();
        const f32 previousOmega = w.omega;
        w.omega = solver.dofVelocity(w.dof);
        w.rotation += w.omega * step;
        if (w.rotation > 2.0f * kPi || w.rotation < -2.0f * kPi)
            w.rotation = fmodf(w.rotation, 2.0f * kPi);
        WheelTelemetry& t = w.telemetry;
        t.inContact = w.grounded;
        t.angularVelocity = w.omega;
        t.rotation = w.rotation;
        t.brakeTorque = w.brakeTorque;
        t.compression = w.grounded ? Clamp(w.compression, 0.0f, wp.suspension.travel) : 0.0f;
        t.suspensionForce = w.grounded ? w.strutForce : 0.0f;
        const SolverRow& friction = solver.row(static_cast<u32>(w.frictionRow));
        t.locked = w.brakeTorque > 0.0f && fabsf(w.omega) < 1.0e-3f &&
                   fabsf(friction.impulse) < friction.hi * (1.0f - 1.0e-4f);

        if (!w.grounded || w.rowX < 0)
        {
            w.deflectionX = 0.0f;
            w.deflectionY = 0.0f;
            w.wasGrounded = false;
            w.lastLoad = 0.0f;
            t.load = 0.0f;
            t.longitudinalForce = 0.0f;
            t.lateralForce = 0.0f;
            t.aligningMoment = 0.0f;
            t.slipRatio = 0.0f;
            t.slipAngle = 0.0f;
            t.contactPoint = w.hardPoint + w.down * (wp.suspension.travel + wp.radius * w.radiusScale);
            t.contactNormal = -w.down;
        }
        else
        {
            const SolverRow& x = solver.row(static_cast<u32>(w.rowX));
            const SolverRow& y = solver.row(static_cast<u32>(w.rowY));
            // Relaxation used this step (advances the deflection next step) and the end-of-step
            // deflection the solve expects, for telemetry.
            const f32 speed = fabsf(w.vx);
            const f32 relaxX = 1.0f / (1.0f + step * speed / tp.longitudinalRelaxation);
            const f32 relaxY = 1.0f / (1.0f + step * speed / tp.lateralRelaxation);
            w.lastRelaxX = relaxX;
            w.lastRelaxY = relaxY;
            w.lastStep = step;
            w.wasGrounded = true;
            const f32 limitX = tp.maxSlipRatio * tp.longitudinalRelaxation;
            const f32 limitY = tanf(tp.maxSlipAngle) * tp.lateralRelaxation;
            const f32 endX = Clamp(relaxX * (w.deflectionX - step * x.velocity), -limitX, limitX);
            const f32 endY = Clamp(relaxY * (w.deflectionY - step * y.velocity), -limitY, limitY);
            t.longitudinalForce = x.impulse * inverseStep;
            t.lateralForce = y.impulse * inverseStep;
            t.slipRatio = endX / tp.longitudinalRelaxation;
            t.slipAngle = atanf(endY / tp.lateralRelaxation);
            // Solved load (rows used an earlier guess) for telemetry and next step's rows.
            if (w.normalRow >= 0)
            {
                const bool shared = (i % 2 == 0) ? mWheels[i + 1].normalRow == w.normalRow
                                                 : mWheels[i - 1].normalRow == w.normalRow;
                t.load = Max(solver.row(static_cast<u32>(w.normalRow)).impulse, 0.0f) * inverseStep *
                         (shared ? 0.5f : 1.0f);
            }
            w.lastLoad = t.load;
            t.aligningMoment = -mTyres[w.axle].trail(t.slipRatio, t.slipAngle, w.camber) * t.lateralForce;
            torque += w.normal * t.aligningMoment;
            t.contactPoint = w.point;
            t.contactNormal = w.normal;

            // The body the wheel stands on feels the opposite.
            if (const BodyEntry* entry = mWorld->body(t.groundBody))
            {
                if (entry->body && entry->body->isDynamic())
                {
                    f32 normalImpulse = 0.0f;
                    if (w.normalRow >= 0)
                    {
                        const bool shared = (i % 2 == 0) ? mWheels[i + 1].normalRow == w.normalRow
                                                         : mWheels[i - 1].normalRow == w.normalRow;
                        normalImpulse = solver.row(static_cast<u32>(w.normalRow)).impulse * (shared ? 0.5f : 1.0f);
                    }
                    const Math::vec3 impulse = w.normal * normalImpulse + w.forward * x.impulse + w.lateral * y.impulse;
                    entry->body->addForceAtPoint(-impulse * inverseStep, w.point);
                }
            }
        }
        // What spun the wheel up beyond the tyre and brake.
        t.driveTorque = wp.inertia * (w.omega - previousOmega) * inverseStep - friction.impulse * inverseStep +
                        (w.rowX >= 0 ? solver.row(static_cast<u32>(w.rowX)).impulse * inverseStep * wp.radius * w.radiusScale : 0.0f);

        const f32 length = w.grounded ? Clamp(w.length, 0.0f, wp.suspension.travel) : wp.suspension.travel;
        t.centre = w.hardPoint + w.down * length;
        const Math::vec3 wheelUp = -w.down;
        const Math::vec3 heading = Math::normalize(w.heading - wheelUp * Math::dot(wheelUp, w.heading));
        const Math::vec3 axleLeft = Math::cross(wheelUp, heading);
        const Math::quat spin = Math::angleAxis(w.rotation, axleLeft);
        const Math::mat3 basis = Math::mat3_cast(spin) * Math::mat3(axleLeft, wheelUp, heading);
        t.worldTransform = Math::mat4(basis);
        t.worldTransform[3] = Math::vec4(t.centre, 1.0f);
    }

    if (finiteVec(force) && finiteVec(torque))
    {
        body.addForce(force);
        body.addTorque(torque);
    }
    else
    {
        Log::error("GroundVehicle: non-finite force - nothing applied this step");
    }
    (void)com;
    mTelemetry.solverRows = solver.rowCount();
    mTelemetry.solverIterations = solver.iterations();
    updateTelemetry(force, torque);
}

void GroundVehicle::updateTelemetry(const Math::vec3& force, const Math::vec3& torque)
{
    const RigidBody& body = *mBody;
    VehicleTelemetry& t = mTelemetry;
    t.force = force;
    t.torque = torque;
    const Math::vec3 forward = body.directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f));
    const Math::vec3 up = body.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    const Math::vec3 right = body.directionToWorld(Math::vec3(-1.0f, 0.0f, 0.0f));
    const Math::vec3 v = body.velocity();
    t.forwardSpeed = Math::dot(v, forward);
    t.lateralSpeed = Math::dot(v, right);
    t.groundSpeed = Math::length(Math::vec3(v.x, 0.0f, v.z));
    t.heading = atan2f(-forward.x, forward.z);
    t.roll = atan2f(-right.y, up.y);
    t.pitch = asinf(Clamp(forward.y, -1.0f, 1.0f));
    t.yawRate = -Math::dot(body.angularVelocity(), up);
    t.lateralAcceleration = t.yawRate * t.forwardSpeed;
    t.steeringAngle = mSteerAngle;
    t.engineRpm = mDrivetrainValid ? mEngine.rpm() : 0.0f;
    t.engineTorque = mDrivetrainValid ? mEngine.combustionTorque() : 0.0f;
    t.engineRunning = mDrivetrainValid && mEngine.running();
    t.gearboxInputRpm = mDrivetrainValid ? mGearbox.inputAngularVelocity() / kRpmToRadians : 0.0f;
    t.gear = mDrivetrainValid ? mGearbox.gear() : 0;
    t.shifting = mDrivetrainValid && mGearbox.shifting();
    t.clutchEngagement = mDrivetrainValid ? mClutch.engagement() : 0.0f;
    t.clutchTorque = mDrivetrainValid ? mClutch.torque() : 0.0f;
    t.clutchSlip = mDrivetrainValid ? mClutch.slip() : 0.0f;
    t.clutchLocked = mDrivetrainValid && mClutch.locked();
}

} // namespace kinetix::Physics
