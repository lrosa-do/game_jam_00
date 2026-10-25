#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "rotorcraft/RotorTransmission.h"

#include "rotorcraft/TurboshaftEngine.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
f32 finiteOr(f32 value, f32 fallback)
{
    return isfinite(value) ? value : fallback;
}
} // namespace

RotorTransmission::RotorTransmission(const Description& description)
{
    setDescription(description);
}

void RotorTransmission::setDescription(const Description& description)
{
    mDescription = description;
    Description& d = mDescription;
    d.mainRotorInertia = Max(finiteOr(description.mainRotorInertia, 1.0f), 1.0e-3f);
    d.tailRotorInertia = Max(finiteOr(description.tailRotorInertia, 0.0f), 0.0f);
    d.tailRotorRatio = Max(finiteOr(description.tailRotorRatio, 0.0f), 0.0f);
    d.engineRatio = Max(finiteOr(description.engineRatio, 1.0f), 1.0e-3f);
    d.efficiency = Clamp(finiteOr(description.efficiency, 1.0f), 0.1f, 1.0f);
    d.frictionTorque = Max(finiteOr(description.frictionTorque, 0.0f), 0.0f);
    d.frictionPerSpeed = Max(finiteOr(description.frictionPerSpeed, 0.0f), 0.0f);
}

void RotorTransmission::setRotorSpeed(f32 omega)
{
    mRotorSpeed = Max(finiteOr(omega, 0.0f), 0.0f);
    mTurbineSpeed = mRotorSpeed * mDescription.engineRatio;
    mEngaged = true;
    mState = TransmissionState();
    mState.rotorSpeed = mRotorSpeed;
    mState.tailRotorSpeed = tailRotorSpeed();
    mState.turbineSpeed = mTurbineSpeed;
    mState.engaged = true;
}

void RotorTransmission::setTurbineSpeed(f32 omega)
{
    mTurbineSpeed = Max(finiteOr(omega, 0.0f), 0.0f);
    mEngaged = mTurbineSpeed >= mRotorSpeed * mDescription.engineRatio;
    if (mEngaged)
        mTurbineSpeed = mRotorSpeed * mDescription.engineRatio;
}

const TransmissionState& RotorTransmission::step(f32 step, const TurboshaftEngine* engine,
                                                 f32 density, f32 mainRotorTorque,
                                                 f32 tailRotorTorque)
{
    const Description& d = mDescription;
    if (!(step > 0.0f) || !isfinite(step))
        return mState;
    TransmissionState s;
    mainRotorTorque = finiteOr(mainRotorTorque, 0.0f);
    tailRotorTorque = finiteOr(tailRotorTorque, 0.0f);

    const f32 ratio = d.engineRatio;
    const f32 tail = d.tailRotorRatio;
    const f32 rotorInertia = d.mainRotorInertia + tail * tail * d.tailRotorInertia;
    const f32 turbineInertia = engine ? engine->description().turbineInertia : 1.0e-3f;

    // Rotor-side load at the main shaft; friction opposes the rotation and vanishes with it.
    const f32 friction = mRotorSpeed > 1.0e-3f ? d.frictionTorque + d.frictionPerSpeed * mRotorSpeed : 0.0f;
    const f32 load = mainRotorTorque + tail * tailRotorTorque + friction;
    // Load slope for the implicit step: aerodynamic torque goes roughly as omega^2 at fixed
    // collective, so 2 Q / omega; a rotor the air drives (Q < 0) gets no stiffening.
    const f32 aeroTorque = mainRotorTorque + tail * tailRotorTorque;
    f32 loadSlope = d.frictionPerSpeed;
    if (mRotorSpeed > 1.0f && aeroTorque > 0.0f)
        loadSlope += 2.0f * aeroTorque / mRotorSpeed;

    f32 turbineTorque = 0.0f;
    f32 turbineSlope = 0.0f;
    if (engine)
    {
        turbineTorque = engine->turbineTorque(mTurbineSpeed, density);
        turbineSlope = engine->turbineTorqueSlope(mTurbineSpeed, density);
    }

    bool engaged = mEngaged || !d.freewheel;
    f32 rotorAcceleration = 0.0f;
    f32 turbineAcceleration = 0.0f;
    f32 sprag = 0.0f;
    if (engaged)
    {
        // Both sides at the main shaft. The engine's slope is negative (torque falls with speed), adding damping.
        const f32 inertia = rotorInertia + ratio * ratio * turbineInertia;
        const f32 drive = turbineTorque > 0.0f ? turbineTorque * d.efficiency : turbineTorque;
        const f32 net = ratio * drive - load;
        const f32 slope = loadSlope - ratio * ratio * Min(turbineSlope, 0.0f);
        rotorAcceleration = net / (inertia + step * Max(slope, 0.0f));
        turbineAcceleration = ratio * rotorAcceleration;
        // The sprag passes on the turbine's torque less what accelerating the turbine itself takes.
        sprag = turbineTorque - turbineInertia * turbineAcceleration;
        if (d.freewheel && sprag < 0.0f)
            engaged = false;
    }
    if (!engaged)
    {
        rotorAcceleration = -load / (rotorInertia + step * Max(loadSlope, 0.0f));
        turbineAcceleration = turbineTorque / (turbineInertia + step * Max(-turbineSlope, 0.0f));
        sprag = 0.0f;
    }

    f32 rotorSpeed = Max(mRotorSpeed + step * rotorAcceleration, 0.0f);
    f32 turbineSpeed = Max(mTurbineSpeed + step * turbineAcceleration, 0.0f);
    if (engaged)
    {
        turbineSpeed = rotorSpeed * ratio;
    }
    else if (turbineSpeed >= rotorSpeed * ratio)
    {
        // The turbine has caught up: the sprag locks and the two inertias share their angular
        // momentum (referred to the main shaft).
        const f32 inertia = rotorInertia + ratio * ratio * turbineInertia;
        const f32 momentum = rotorInertia * rotorSpeed + ratio * turbineInertia * turbineSpeed;
        rotorSpeed = momentum / inertia;
        turbineSpeed = rotorSpeed * ratio;
        engaged = true;
    }
    if (!isfinite(rotorSpeed) || !isfinite(turbineSpeed))
    {
        rotorSpeed = 0.0f;
        turbineSpeed = 0.0f;
    }
    rotorAcceleration = (rotorSpeed - mRotorSpeed) / step;
    turbineAcceleration = (turbineSpeed - mTurbineSpeed) / step;

    mRotorSpeed = rotorSpeed;
    mTurbineSpeed = turbineSpeed;
    mEngaged = engaged;

    s.rotorSpeed = mRotorSpeed;
    s.tailRotorSpeed = mRotorSpeed * tail;
    s.turbineSpeed = mTurbineSpeed;
    s.rotorAcceleration = rotorAcceleration;
    s.turbineAcceleration = turbineAcceleration;
    s.engineTorque = sprag;
    s.enginePower = sprag * mTurbineSpeed;
    s.mainShaftTorque = mainRotorTorque + d.mainRotorInertia * rotorAcceleration;
    s.tailShaftTorque = tailRotorTorque + d.tailRotorInertia * tail * rotorAcceleration;
    s.engaged = mEngaged;
    mState = s;
    return mState;
}

} // namespace kinetix::Physics
