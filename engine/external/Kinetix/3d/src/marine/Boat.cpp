#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "marine/Boat.h"

#include "dynamics/RigidBody.h"
#include "marine/WaterSurface.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// Volume of a hull below the plane y = 0, origin at `height`, given orientation: the
// divergence-theorem sum of BuoyancyBody on the exact clipped triangles.
f64 volumeBelow(const HullMesh& hull, const Math::quat& orientation, f32 height)
{
    const ct::Vector<Math::vec3>& vertices = hull.vertices();
    const ct::Vector<u32>& indices = hull.indices();
    f64 volume = 0.0;
    for (u32 t = 0; t < hull.triangleCount(); ++t)
    {
        WetVertex corners[3];
        for (u32 c = 0; c < 3; ++c)
        {
            corners[c].position = orientation * vertices[indices[3 * t + c]];
            corners[c].depth = -(height + corners[c].position.y);
        }
        WetTriangle pieces[2];
        const u32 count = clipTriangleToWater(corners, pieces);
        const f32 ny = (orientation * hull.triangleNormal(t)).y;
        for (u32 p = 0; p < count; ++p)
        {
            const WetVertex* v = pieces[p].vertices;
            const f32 area =
                0.5f * Math::length(Math::cross(v[1].position - v[0].position, v[2].position - v[0].position));
            volume -= static_cast<f64>(ny) * area * (v[0].depth + v[1].depth + v[2].depth) / 3.0;
        }
    }
    return volume;
}
} // namespace

f32 floatingHeight(const HullMesh& hull, f32 mass, f32 density, const Math::quat& orientation)
{
    if (hull.vertexCount() == 0 || !(density > 0.0f) || !(mass > 0.0f))
        return 0.0f;
    const Math::quat q = Math::normalize(orientation);
    f32 lowest = 1.0e30f;
    f32 highest = -1.0e30f;
    for (const Math::vec3& v : hull.vertices())
    {
        const f32 y = (q * v).y;
        lowest = Min(lowest, y);
        highest = Max(highest, y);
    }
    f32 lo = -highest;
    f32 hi = -lowest;
    const f64 target = static_cast<f64>(mass) / density;
    if (volumeBelow(hull, q, lo) <= target)
        return lo;
    for (u32 iteration = 0; iteration < 60; ++iteration)
    {
        const f32 mid = 0.5f * (lo + hi);
        if (volumeBelow(hull, q, mid) > target)
            lo = mid;
        else
            hi = mid;
        if (hi - lo < 1.0e-6f)
            break;
    }
    return 0.5f * (lo + hi);
}

Boat::Boat()
{
    mHull.setForwardAxis(mForward);
}

void Boat::setBody(RigidBody* body)
{
    mBody = body;
    mHull.setBody(body);
    mPrismHull.setBody(body);
    for (ScrewPropeller& propeller : mScrewPropellers)
        propeller.setBody(body);
}

void Boat::setWater(const WaterSurface* water)
{
    mWater = water;
    mHull.setWater(water);
    mPrismHull.setWater(water);
    for (ScrewPropeller& propeller : mScrewPropellers)
        propeller.setWater(water);
}

void Boat::setAxes(const Math::vec3& forward, const Math::vec3& up)
{
    if (!finiteVec(forward) || !finiteVec(up) || Math::dot(forward, forward) < 1.0e-12f)
    {
        Log::error("Boat: forward axis must be finite and non-zero");
        return;
    }
    const Math::vec3 f = Math::normalize(forward);
    const Math::vec3 u = up - f * Math::dot(up, f);
    if (Math::dot(u, u) < 1.0e-12f)
    {
        Log::error("Boat: up axis must not be parallel to forward");
        return;
    }
    mForward = f;
    mUp = Math::normalize(u);
    mHull.setForwardAxis(mForward);
}

void Boat::setHull(const HullMesh& hull)
{
    mHull.setHull(hull);
    mPrismHull.setHull(hull);
    mHull.setForwardAxis(mForward);
}

u32 Boat::addHydrofoil(const Hydrofoil& foil, f32 rudderMix)
{
    FoilEntry entry;
    entry.foil = foil;
    entry.rudderMix = isfinite(rudderMix) ? rudderMix : 0.0f;
    mFoils.push_back(entry);
    return static_cast<u32>(mFoils.size() - 1);
}

u32 Boat::addEngine(const PistonEngine& engine)
{
    mEngines.push_back(engine);
    return static_cast<u32>(mEngines.size() - 1);
}

u32 Boat::addPropeller(const MarinePropeller& propeller, s32 engine, f32 steeringMix)
{
    PropellerEntry entry;
    entry.propeller = propeller;
    entry.engine = engine;
    entry.steeringMix = isfinite(steeringMix) ? steeringMix : 0.0f;
    if (engine >= 0 && static_cast<u32>(engine) >= mEngines.size())
    {
        Log::error("Boat: engine %d does not exist - propeller left undriven", engine);
        entry.engine = -1;
    }
    mPropellers.push_back(entry);
    return static_cast<u32>(mPropellers.size() - 1);
}

u32 Boat::addWindage(const DragElement& element)
{
    DragElement stored = element;
    stored.dragAreas = Math::max(element.dragAreas, Math::vec3(0.0f));
    mWindage.push_back(stored);
    return static_cast<u32>(mWindage.size() - 1);
}

void Boat::setHydrodynamics(BoatHydrodynamics model)
{
    mHydrodynamics = model;
}

u32 Boat::addScrewPropeller(const ScrewPropeller& propeller)
{
    ScrewPropeller stored = propeller;
    stored.setBody(mBody);
    stored.setWater(mWater);
    stored.setEffects(mEffects);
    mScrewPropellers.push_back(stored);
    return static_cast<u32>(mScrewPropellers.size() - 1);
}

void Boat::setEffects(WaterEffects* effects)
{
    mEffects = effects;
    mPrismHull.setEffects(effects);
    for (ScrewPropeller& propeller : mScrewPropellers)
        propeller.setEffects(effects);
}

void Boat::setControls(const BoatControls& controls)
{
    auto unit = [](f32 value)
    {
        return isfinite(value) ? Clamp(value, -1.0f, 1.0f) : 0.0f;
    };
    mControls.throttle = unit(controls.throttle);
    mControls.rudder = unit(controls.rudder);
}

void Boat::setThrottleDeadband(f32 deadband)
{
    mDeadband = isfinite(deadband) ? Clamp(deadband, 0.0f, 0.5f) : 0.05f;
}

void Boat::updateHelm()
{
    for (FoilEntry& entry : mFoils)
        entry.foil.setDeflection(mControls.rudder * entry.rudderMix * entry.foil.maxDeflection());

    // A band around zero is neutral, beyond it the gear engages and the rest of the travel
    // opens the throttle.
    const f32 lever = mControls.throttle;
    MarinePropeller::Gear gear = MarinePropeller::Gear::Neutral;
    f32 opening = 0.0f;
    if (fabsf(lever) > mDeadband)
    {
        gear = lever > 0.0f ? MarinePropeller::Gear::Forward : MarinePropeller::Gear::Reverse;
        opening = (fabsf(lever) - mDeadband) / Max(1.0f - mDeadband, 1.0e-6f);
    }
    for (PistonEngine& engine : mEngines)
        engine.setThrottle(opening);
    for (PropellerEntry& entry : mPropellers)
    {
        MarinePropeller& propeller = entry.propeller;
        propeller.setSteering(mControls.rudder * entry.steeringMix * propeller.description().maxSteeringAngle);
        if (entry.engine < 0)
            continue;
        propeller.setGear(gear);
        // Air at the engine's intake: sea-level value unless the atmosphere says otherwise.
        const f32 air = atmosphere().sampleAt(mBody->position()).density;
        const PistonEngine& engine = mEngines[static_cast<u32>(entry.engine)];
        propeller.setEngineTorque(engine.shaftTorque(propeller.engineAngularSpeed(), air));
    }
}

void Boat::accumulate(bool apply, f32 step, Math::vec3& force, Math::vec3& torque)
{
    force = Math::vec3(0.0f);
    torque = Math::vec3(0.0f);
    RigidBody& body = *mBody;
    const WaterSurface& water = *mWater;

    if (mHydrodynamics == BoatHydrodynamics::Prism)
    {
        accumulatePrism(apply, force, torque);
        return;
    }
    updateHelm();

    Math::vec3 f;
    Math::vec3 t;
    if (apply)
        mHull.applyForces(step);
    else
        mHull.computeForces(f, t);
    force += mHull.state().force;
    torque += mHull.state().torque;

    for (FoilEntry& entry : mFoils)
    {
        const WingState& state = apply ? entry.foil.apply(body, water) : entry.foil.computeForces(body, water);
        force += state.force;
        torque += state.torque;
    }

    mThrust = 0.0f;
    mFirstImmersion = 0.0f;
    for (u32 i = 0; i < mPropellers.size(); ++i)
    {
        MarinePropeller& propeller = mPropellers[i].propeller;
        const PropellerLoads loads = apply ? propeller.apply(body, water, step, &f, &t)
                                           : propeller.computeForces(body, water, f, t);
        mThrust += loads.thrust;
        if (i == 0)
            mFirstImmersion = apply ? propeller.lastImmersion() : propeller.immersion(body, water);
        force += f;
        torque += t;
    }

    const Atmosphere& air = atmosphere();
    const f32 density = air.sampleAt(body.position()).density;
    for (const DragElement& element : mWindage)
    {
        const Math::vec3 point = body.pointToWorld(element.position);
        const Math::vec3 flow = body.directionToLocal(air.windAt(point) - body.velocityAtPoint(point));
        const f32 speed = Math::length(flow);
        const Math::vec3 local = element.dragAreas * flow * (0.5f * density * speed);
        const Math::vec3 drag = body.directionToWorld(local);
        const Math::vec3 moment = Math::cross(point - body.position(), drag);
        if (apply)
        {
            body.addForce(drag);
            body.addTorque(moment);
        }
        force += drag;
        torque += moment;
    }

    updateTelemetry(force, torque);
}

// The prism model: the hull's pressure and drag and the screw propellers.
// It has no pass that only computes, so without `apply` this answers with
// the last step's totals.
void Boat::accumulatePrism(bool apply, Math::vec3& force, Math::vec3& torque)
{
    if (!apply)
    {
        force = mTelemetry.force;
        torque = mTelemetry.torque;
        return;
    }
    mPrismHull.setEmitEffects(mEmitEffects);
    mPrismHull.applyForces();
    force += mPrismHull.force();
    torque += mPrismHull.torque();
    // The appendages act in this model too: the lower unit gives the stern
    // its grip and damping, with the helm.
    for (FoilEntry& entry : mFoils)
    {
        entry.foil.setDeflection(mControls.rudder * entry.rudderMix * entry.foil.maxDeflection());
        const WingState& state = entry.foil.apply(*mBody, *mWater);
        force += state.force;
        torque += state.torque;
    }
    mThrust = 0.0f;
    mFirstImmersion = 0.0f;
    for (u32 i = 0; i < mScrewPropellers.size(); ++i)
    {
        ScrewPropeller& propeller = mScrewPropellers[i];
        // The screw's rudder turns its thrust about the line down through
        // the hub, so a positive rudder swings the thrust the other way from
        // MarinePropeller's steering about its up axis; the helm keeps the
        // module's convention, positive towards the side the heading grows.
        propeller.setThrottle(mControls.throttle);
        propeller.setRudder(-mControls.rudder);
        propeller.updateForces(mEmitEffects);
        mThrust += propeller.thrust();
        if (i == 0)
            mFirstImmersion = propeller.inWater() ? 1.0f : 0.0f;
        force += propeller.force();
        torque += Math::cross(mBody->pointToWorld(propeller.hub()) - mBody->position(), propeller.force());
    }
    mEmitEffects = false;
    updateTelemetry(force, torque);
}

void Boat::applyForces(f32 step)
{
    if (!mBody || !mWater)
        return;
    Math::vec3 force;
    Math::vec3 torque;
    accumulate(true, step, force, torque);
}

void Boat::computeForces(Math::vec3& force, Math::vec3& torque)
{
    force = Math::vec3(0.0f);
    torque = Math::vec3(0.0f);
    if (!mBody || !mWater)
        return;
    accumulate(false, 0.0f, force, torque);
}

void Boat::stepCallback(f32 step, void* userData)
{
    if (userData)
        static_cast<Boat*>(userData)->applyForces(step);
}

void Boat::updateTelemetry(const Math::vec3& force, const Math::vec3& torque)
{
    const RigidBody& body = *mBody;
    const BuoyancyState& hull = mHull.state();
    BoatTelemetry t;
    t.force = force;
    t.torque = torque;

    const Math::vec3 forwardWorld = body.directionToWorld(mForward);
    const Math::vec3 upWorld = body.directionToWorld(mUp);
    const Math::vec3 rightWorld = body.directionToWorld(rightAxis());
    const Math::vec3 waterVelocity = mHydrodynamics == BoatHydrodynamics::Prism
                                         ? mWater->flowVelocity(body.position())
                                         : hull.waterVelocity;
    const Math::vec3 throughWater = body.velocity() - waterVelocity;
    t.speedThroughWater = Math::length(throughWater);
    t.forwardSpeed = Math::dot(throughWater, forwardWorld);
    t.lateralSpeed = Math::dot(throughWater, rightWorld);
    t.groundSpeed = Math::length(Math::vec3(body.velocity().x, 0.0f, body.velocity().z));
    t.heading = atan2f(-forwardWorld.x, forwardWorld.z);
    t.heel = atan2f(-rightWorld.y, upWorld.y);
    t.trim = asinf(Clamp(forwardWorld.y, -1.0f, 1.0f));
    t.yawRate = -Math::dot(body.angularVelocity(), upWorld);
    t.draft = hull.draft;
    t.submergedVolume = hull.submergedVolume;
    t.waterplaneArea = hull.waterplaneArea;
    t.centreOfBuoyancy = hull.centreOfBuoyancy;
    t.hydrostaticForce = hull.hydrostaticForce;
    t.thrust = mThrust;
    t.immersion = mFirstImmersion;
    if (!mPropellers.empty())
    {
        t.engineRpm = mPropellers[0].propeller.engineRpm();
        t.propellerRpm = mPropellers[0].propeller.rpm();
    }
    if (mHydrodynamics == BoatHydrodynamics::Prism)
    {
        // The prism model has no hydrostatics to report beyond the force,
        // and its engine reads as a percentage of power.
        t.draft = mPrismHull.draft();
        t.submergedVolume = 0.0f;
        t.waterplaneArea = 0.0f;
        t.centreOfBuoyancy = Math::vec3(0.0f);
        t.hydrostaticForce = mPrismHull.force();
        if (!mScrewPropellers.empty())
        {
            t.engineRpm = mScrewPropellers[0].powerPercent();
            t.propellerRpm = mScrewPropellers[0].powerPercent();
        }
    }
    mTelemetry = t;
}

} // namespace kinetix::Physics
