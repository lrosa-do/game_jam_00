#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/PistonEngine.h"

#include "dynamics/Atmosphere.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kRpmToRadians = 2.0f * 3.14159265358979f / 60.0f;

// How far past maxRpm the torque takes to fall to nothing.
constexpr f32 kLimiterBand = 100.0f;
} // namespace

PistonEngine::PistonEngine(const Description& description)
{
    setDescription(description);
}

void PistonEngine::setDescription(const Description& description)
{
    mDescription = description;
    mDescription.maxPower = Max(description.maxPower, 0.0f);
    mDescription.ratedRpm = Max(description.ratedRpm, 1.0f);
    mDescription.maxRpm = Max(description.maxRpm, mDescription.ratedRpm);
    mDescription.idleThrottle = Clamp(description.idleThrottle, 0.0f, 1.0f);
    mDescription.frictionStatic = Max(description.frictionStatic, 0.0f);
    mDescription.frictionPerRpm = Max(description.frictionPerRpm, 0.0f);
}

void PistonEngine::setThrottle(f32 throttle)
{
    mThrottle = isfinite(throttle) ? Clamp(throttle, 0.0f, 1.0f) : 0.0f;
}

f32 PistonEngine::ratedTorque() const
{
    return mDescription.maxPower / (mDescription.ratedRpm * kRpmToRadians);
}

f32 PistonEngine::fullThrottleTorque(f32 omega) const
{
    // P / P_max = x + x^2 - x^3, so torque / rated torque = 1 + x - x^2: finite
    // at rest, a gentle peak at half the rated speed, rated again at it.
    const f32 x = Max(omega, 0.0f) / (mDescription.ratedRpm * kRpmToRadians);
    const f32 shape = Max(1.0f + x - x * x, 0.0f);
    return ratedTorque() * shape;
}

f32 PistonEngine::shaftTorque(f32 omega, f32 density) const
{
    if (!isfinite(omega))
        return 0.0f;
    const f32 speed = Max(omega, 0.0f);
    const f32 rpm = speed / kRpmToRadians;

    const f32 sigma = Clamp(density / Atmosphere::SeaLevelDensity, 0.0f, 1.5f);
    const f32 altitudeFactor = Max(sigma - (1.0f - sigma) / 7.55f, 0.0f);

    const f32 throttle =
        mDescription.idleThrottle + (1.0f - mDescription.idleThrottle) * mThrottle;
    f32 combustion = throttle * altitudeFactor * fullThrottleTorque(speed);
    if (rpm > mDescription.maxRpm)
        combustion *= Max(1.0f - (rpm - mDescription.maxRpm) / kLimiterBand, 0.0f);

    const f32 x = rpm / mDescription.ratedRpm;
    const f32 friction =
        ratedTorque() * (mDescription.frictionStatic + mDescription.frictionPerRpm * x);
    // Friction never drives the shaft backwards: at a standstill it only cancels
    // what combustion makes.
    if (speed <= 0.0f)
        return Max(combustion - friction, 0.0f);
    return combustion - friction;
}

} // namespace kinetix::Physics
