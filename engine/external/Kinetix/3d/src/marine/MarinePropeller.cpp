#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "marine/MarinePropeller.h"

#include "dynamics/RigidBody.h"
#include "marine/WaterSurface.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kRpmToRadians = 2.0f * kPi / 60.0f;

bool usable(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z) && Math::dot(v, v) > 1.0e-12f;
}
} // namespace

MarinePropeller::MarinePropeller()
{
    setDescription(Description());
}

MarinePropeller::MarinePropeller(const Description& description, const Airfoil& airfoil)
{
    mBlades.setAirfoil(airfoil);
    setDescription(description);
}

void MarinePropeller::setDescription(const Description& description)
{
    mDescription = description;
    Description& d = mDescription;
    d.gearRatio = isfinite(description.gearRatio) ? Max(description.gearRatio, 0.05f) : 1.0f;
    d.gearEfficiency = isfinite(description.gearEfficiency) ? Clamp(description.gearEfficiency, 0.05f, 1.0f) : 1.0f;
    d.reverseEfficiency =
        isfinite(description.reverseEfficiency) ? Clamp(description.reverseEfficiency, 0.0f, 1.0f) : 0.6f;
    d.wakeFraction = isfinite(description.wakeFraction) ? Clamp(description.wakeFraction, 0.0f, 0.9f) : 0.0f;
    d.thrustDeduction =
        isfinite(description.thrustDeduction) ? Clamp(description.thrustDeduction, 0.0f, 0.9f) : 0.0f;
    d.engineInertia = isfinite(description.engineInertia) ? Max(description.engineInertia, 1.0e-4f) : 0.1f;
    d.maxSteeringAngle =
        isfinite(description.maxSteeringAngle) ? Clamp(fabsf(description.maxSteeringAngle), 0.0f, 1.5f) : 0.0f;
    if (!usable(d.steeringAxis))
    {
        Log::error("MarinePropeller: the steering axis must be a finite, non-zero direction");
        d.steeringAxis = Math::vec3(0.0f, 1.0f, 0.0f);
    }
    d.steeringAxis = Math::normalize(d.steeringAxis);
    mBlades.setDescription(d.blades);
    d.blades = mBlades.description();
    setSteering(mSteering);
}

void MarinePropeller::setGear(Gear gear)
{
    mGear = gear;
}

void MarinePropeller::setPropellerRpm(f32 rpm)
{
    mHeldRpm = true;
    mHeldOmega = isfinite(rpm) ? rpm * kRpmToRadians : 0.0f;
    mEngineOmega = fabsf(mHeldOmega) * mDescription.gearRatio;
}

void MarinePropeller::setEngineAngularSpeed(f32 omega)
{
    mEngineOmega = isfinite(omega) ? Max(omega, 0.0f) : 0.0f;
}

f32 MarinePropeller::engineRpm() const
{
    return mEngineOmega / kRpmToRadians;
}

f32 MarinePropeller::angularSpeed() const
{
    if (mHeldRpm)
        return mHeldOmega;
    return gearSign() * mEngineOmega / mDescription.gearRatio;
}

f32 MarinePropeller::rpm() const
{
    return angularSpeed() / kRpmToRadians;
}

void MarinePropeller::setSteering(f32 angle)
{
    mSteering = isfinite(angle) ? Clamp(angle, -mDescription.maxSteeringAngle, mDescription.maxSteeringAngle) : 0.0f;
}

PropellerLoads MarinePropeller::evaluate(f32 omega, f32 advanceSpeed, f32 density, f32 immersion) const
{
    PropellerLoads loads;
    if (!(immersion > 0.0f) || !isfinite(omega) || !isfinite(advanceSpeed))
        return loads;
    const f32 share = Min(immersion, 1.0f);
    if (omega >= 0.0f)
    {
        loads = mBlades.evaluate(omega, advanceSpeed, density);
    }
    else
    {
        const PropellerLoads ahead = mBlades.evaluate(-omega, -advanceSpeed, density);
        loads.thrust = -ahead.thrust * mDescription.reverseEfficiency;
        loads.torque = -ahead.torque;
    }
    loads.thrust *= share;
    loads.torque *= share;
    loads.power = loads.torque * omega;
    return loads;
}

Math::vec3 MarinePropeller::thrustAxis(const RigidBody& body) const
{
    Math::vec3 axis = mDescription.blades.axis;
    if (mSteering != 0.0f)
        axis = Math::angleAxis(mSteering, mDescription.steeringAxis) * axis;
    return body.directionToWorld(axis);
}

Math::vec3 MarinePropeller::hubPosition(const RigidBody& body) const
{
    return body.pointToWorld(mDescription.blades.position);
}

f32 MarinePropeller::immersion(const RigidBody& body, const WaterSurface& water) const
{
    const Math::vec3 hub = hubPosition(body);
    const Math::vec3 axis = thrustAxis(body);
    const f32 depth = water.depth(hub);
    // Disc half-height: radius times the sine of the axis' inclination to the vertical.
    const f32 reach = mDescription.blades.radius * sqrtf(Max(1.0f - axis.y * axis.y, 0.0f));
    if (reach < 1.0e-4f)
        return depth > 0.0f ? 1.0f : 0.0f;
    // Area of the disc below a chord at height `depth` above its centre.
    const f32 u = Clamp(depth / reach, -1.0f, 1.0f);
    return Clamp((kPi - acosf(u) + u * sqrtf(Max(1.0f - u * u, 0.0f))) / kPi, 0.0f, 1.0f);
}

f32 MarinePropeller::advanceSpeed(const RigidBody& body, const WaterSurface& water) const
{
    const Math::vec3 hub = hubPosition(body);
    const Math::vec3 axis = thrustAxis(body);
    const Math::vec3 relative = body.velocityAtPoint(hub) - water.flowVelocity(hub);
    return Math::dot(relative, axis) * (1.0f - mDescription.wakeFraction);
}

PropellerLoads MarinePropeller::computeForces(const RigidBody& body, const WaterSurface& water,
                                              Math::vec3& force, Math::vec3& torque) const
{
    return loadsAt(body, water, immersion(body, water), force, torque);
}

PropellerLoads MarinePropeller::loadsAt(const RigidBody& body, const WaterSurface& water, f32 share,
                                        Math::vec3& force, Math::vec3& torque) const
{
    const Math::vec3 hub = hubPosition(body);
    const Math::vec3 axis = thrustAxis(body);
    const PropellerLoads loads = evaluate(angularSpeed(), advanceSpeed(body, water), water.density(), share);
    force = axis * (loads.thrust * (1.0f - mDescription.thrustDeduction));
    // The water's torque on the blades reaches the hull through shaft and engine mounts, against
    // the rotation whichever way it turns.
    const Math::vec3 spin = axis * mDescription.blades.rotation;
    torque = Math::cross(hub - body.position(), force) - spin * loads.torque;
    return loads;
}

const PropellerLoads& MarinePropeller::apply(RigidBody& body, const WaterSurface& water, f32 step,
                                             Math::vec3* appliedForce, Math::vec3* appliedTorque)
{
    Math::vec3 force;
    Math::vec3 torque;
    mImmersion = immersion(body, water);
    mLoads = loadsAt(body, water, mImmersion, force, torque);
    body.addForce(force);
    body.addTorque(torque);
    if (appliedForce)
        *appliedForce = force;
    if (appliedTorque)
        *appliedTorque = torque;

    if (mHeldRpm || !(step > 0.0f) || !isfinite(step))
        return mLoads;

    const Description& d = mDescription;
    const f32 sign = gearSign();
    if (sign == 0.0f)
    {
        mEngineOmega += step * mEngineTorque / d.engineInertia;
    }
    else
    {
        // In gear: one shaft. The propeller's torque reaches the crank divided by ratio and gearbox
        // efficiency, its inertia by ratio squared. Linearly implicit in that load as for the
        // aircraft propeller (slope 2 Q / omega).
        const f32 load = mLoads.torque * sign / (d.gearRatio * d.gearEfficiency);
        const f32 inertia = d.engineInertia + d.blades.inertia / (d.gearRatio * d.gearRatio);
        const f32 slope = mEngineOmega > 1.0f ? 2.0f * load / mEngineOmega : 0.0f;
        mEngineOmega += step * (mEngineTorque - load) / (inertia + step * Max(slope, 0.0f));
    }
    if (!isfinite(mEngineOmega) || mEngineOmega < 0.0f)
        mEngineOmega = 0.0f;
    return mLoads;
}

} // namespace kinetix::Physics
