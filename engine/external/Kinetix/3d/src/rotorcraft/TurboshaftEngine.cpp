#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "rotorcraft/TurboshaftEngine.h"

#include "dynamics/Atmosphere.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kRpmToRadians = 2.0f * 3.14159265358979f / 60.0f;
// Below this the gas generator is a windmilling compressor, not an engine; also where the lag's time constant stops growing.
constexpr f32 kMinSpoolSpeed = 0.3f;

f32 finiteOr(f32 value, f32 fallback)
{
    return isfinite(value) ? value : fallback;
}
} // namespace

TurboshaftEngine::TurboshaftEngine(const Description& description)
{
    setDescription(description);
}

void TurboshaftEngine::setDescription(const Description& description)
{
    mDescription = description;
    Description& d = mDescription;
    d.maxPower = Max(finiteOr(description.maxPower, 0.0f), 0.0f);
    d.ratedTurbineRpm = Max(finiteOr(description.ratedTurbineRpm, 6000.0f), 1.0f);
    d.zeroPowerGasGenerator = Clamp(finiteOr(description.zeroPowerGasGenerator, 0.5f), 0.0f, 0.95f);
    d.idleGasGenerator = Clamp(finiteOr(description.idleGasGenerator, 0.6f), d.zeroPowerGasGenerator, 1.0f);
    d.maxGasGenerator = Max(finiteOr(description.maxGasGenerator, 1.0f), d.idleGasGenerator + 0.01f);
    d.spoolTimeConstant = Max(finiteOr(description.spoolTimeConstant, 0.5f), 1.0e-3f);
    d.spoolDownTimeConstant = Max(finiteOr(description.spoolDownTimeConstant, 3.0f), 1.0e-3f);
    d.turbineInertia = Max(finiteOr(description.turbineInertia, 0.1f), 1.0e-4f);
    d.windage = Max(finiteOr(description.windage, 0.0f), 0.0f);
    d.governorGain = Max(finiteOr(description.governorGain, 0.0f), 0.0f);
    d.governorIntegral = Max(finiteOr(description.governorIntegral, 0.0f), 0.0f);
    d.anticipation = finiteOr(description.anticipation, 0.0f);
    d.anticipationBias = finiteOr(description.anticipationBias, 0.0f);
    d.specificFuelConsumption = Max(finiteOr(description.specificFuelConsumption, 0.0f), 0.0f);
    d.idleFuelFlow = Max(finiteOr(description.idleFuelFlow, 0.0f), 0.0f);
}

void TurboshaftEngine::setThrottle(f32 throttle)
{
    mThrottle = Clamp(finiteOr(throttle, 0.0f), 0.0f, 1.0f);
}

void TurboshaftEngine::setCollective(f32 collective)
{
    mCollective = Clamp(finiteOr(collective, 0.0f), 0.0f, 1.0f);
}

void TurboshaftEngine::setGovernedSpeed(f32 fraction)
{
    mGovernedSpeed = Clamp(finiteOr(fraction, 1.0f), 0.5f, 1.2f);
}

f32 TurboshaftEngine::ratedTurbineSpeed() const
{
    return mDescription.ratedTurbineRpm * kRpmToRadians;
}

void TurboshaftEngine::setGasGenerator(f32 fraction)
{
    const Description& d = mDescription;
    mGasGenerator = Clamp(finiteOr(fraction, 0.0f), 0.0f, d.maxGasGenerator);
    mRunning = mGasGenerator >= d.idleGasGenerator;
    // The demand that holds this Ng, all in the integrator so the governor starts in balance at zero error.
    mDemand = Clamp((mGasGenerator - d.idleGasGenerator) / (d.maxGasGenerator - d.idleGasGenerator), 0.0f, 1.0f);
    mIntegral = mDemand - d.anticipationBias - d.anticipation * mCollective;
}

f32 TurboshaftEngine::gasPower(f32 density) const
{
    const Description& d = mDescription;
    const f32 x = (mGasGenerator - d.zeroPowerGasGenerator) / (1.0f - d.zeroPowerGasGenerator);
    if (x <= 0.0f)
        return 0.0f;
    const f32 sigma = Max(finiteOr(density, Atmosphere::SeaLevelDensity), 0.0f) / Atmosphere::SeaLevelDensity;
    return d.maxPower * powf(sigma, 0.8f) * x * x;
}

f32 TurboshaftEngine::turbineTorque(f32 omega, f32 density) const
{
    const f32 rated = ratedTurbineSpeed();
    const f32 x = Max(finiteOr(omega, 0.0f), 0.0f) / rated;
    // Free-turbine line: torque falls linearly with turbine speed at a fixed gas generator state.
    const f32 drive = gasPower(density) / rated * Max(2.0f - x, 0.0f);
    const f32 windage = mDescription.windage * (mDescription.maxPower / rated) * x;
    return drive - windage;
}

f32 TurboshaftEngine::turbineTorqueSlope(f32 omega, f32 density) const
{
    const f32 rated = ratedTurbineSpeed();
    const f32 x = Max(finiteOr(omega, 0.0f), 0.0f) / rated;
    const f32 drive = x < 2.0f ? -gasPower(density) / (rated * rated) : 0.0f;
    return drive - mDescription.windage * mDescription.maxPower / (rated * rated);
}

f32 TurboshaftEngine::fuelFlow(f32 density) const
{
    if (!mRunning || mGasGenerator < mDescription.zeroPowerGasGenerator)
        return 0.0f;
    return mDescription.idleFuelFlow + mDescription.specificFuelConsumption * gasPower(density);
}

void TurboshaftEngine::step(f32 step, f32 turbineSpeed, f32 density)
{
    (void)density;
    if (!(step > 0.0f) || !isfinite(step))
        return;
    const Description& d = mDescription;

    f32 target = 0.0f;
    f32 timeConstant = d.spoolDownTimeConstant;
    if (mRunning)
    {
        const f32 setpoint = mGovernedSpeed * ratedTurbineSpeed();
        const f32 error = (setpoint - Max(finiteOr(turbineSpeed, 0.0f), 0.0f)) / setpoint;
        const f32 feedForward = d.anticipationBias + d.anticipation * mCollective;
        const f32 unclamped = feedForward + d.governorGain * error + mIntegral;
        // Anti-windup: integrate only while the demand is inside its range, or the error pulls it back in.
        const bool high = unclamped >= mThrottle && error > 0.0f;
        const bool low = unclamped <= 0.0f && error < 0.0f;
        if (!high && !low)
            mIntegral += d.governorIntegral * error * step;
        mIntegral = Clamp(mIntegral, -1.5f, 1.5f);
        mDemand = Clamp(feedForward + d.governorGain * error + mIntegral, 0.0f, mThrottle);
        target = d.idleGasGenerator + mDemand * (d.maxGasGenerator - d.idleGasGenerator);
        // A spool well below idle is being started; it accelerates no faster than the lag at idle speed allows.
        timeConstant = d.spoolTimeConstant / Clamp(mGasGenerator, Max(kMinSpoolSpeed, 0.5f * d.idleGasGenerator), 1.2f);
    }
    else
    {
        mDemand = 0.0f;
        mIntegral = 0.0f;
    }
    mGasGenerator = target + (mGasGenerator - target) * expf(-step / timeConstant);
    if (!isfinite(mGasGenerator))
        mGasGenerator = 0.0f;
}

} // namespace kinetix::Physics
