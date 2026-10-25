#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/Atmosphere.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
// The ISA layers, by geopotential altitude: linear in temperature (lapse rate
// L, K/m) or isothermal. Each layer's base pressure is carried up from the one
// below by the same hydrostatic integral, so boundaries agree.
constexpr f64 kTroposphereTop = 11000.0;
constexpr f64 kIsothermalTop = 20000.0;
constexpr f64 kInversionTop = 32000.0;
constexpr f64 kBottom = -2000.0;
constexpr f64 kTroposphereLapse = 0.0065;
constexpr f64 kInversionLapse = -0.001;

// Hydrostatic balance with the ideal gas law, dp/dH = -g0 p / (R T): a power
// law over a linear-temperature layer, an exponential over an isothermal one.
f64 linearLayerPressure(f64 baseTemperature, f64 basePressure, f64 lapse, f64 height)
{
    const f64 temperature = baseTemperature - lapse * height;
    const f64 exponent = Atmosphere::StandardGravity / (Atmosphere::GasConstant * lapse);
    return basePressure * pow(temperature / baseTemperature, exponent);
}

f64 isothermalLayerPressure(f64 temperature, f64 basePressure, f64 height)
{
    return basePressure *
           exp(-Atmosphere::StandardGravity * height / (Atmosphere::GasConstant * temperature));
}

AtmosphereState fromTemperaturePressure(f64 temperature, f64 pressure)
{
    AtmosphereState state;
    state.temperature = static_cast<f32>(temperature);
    state.pressure = static_cast<f32>(pressure);
    state.density = static_cast<f32>(pressure / (Atmosphere::GasConstant * temperature));
    state.speedOfSound =
        static_cast<f32>(sqrt(Atmosphere::HeatCapacityRatio * Atmosphere::GasConstant * temperature));
    return state;
}
} // namespace

AtmosphereState Atmosphere::standard(f32 altitude)
{
    // Geometric to geopotential (the tables are tabulated against the latter).
    f64 h = static_cast<f64>(altitude);
    if (!isfinite(h))
        h = 0.0;
    f64 geopotential = EarthRadius * h / (EarthRadius + h);
    if (geopotential < kBottom)
        geopotential = kBottom;
    if (geopotential > kInversionTop)
        geopotential = kInversionTop;

    const f64 t0 = SeaLevelTemperature;
    const f64 p0 = SeaLevelPressure;
    if (geopotential <= kTroposphereTop)
    {
        const f64 temperature = t0 - kTroposphereLapse * geopotential;
        return fromTemperaturePressure(
            temperature, linearLayerPressure(t0, p0, kTroposphereLapse, geopotential));
    }

    const f64 t1 = t0 - kTroposphereLapse * kTroposphereTop;
    const f64 p1 = linearLayerPressure(t0, p0, kTroposphereLapse, kTroposphereTop);
    if (geopotential <= kIsothermalTop)
        return fromTemperaturePressure(
            t1, isothermalLayerPressure(t1, p1, geopotential - kTroposphereTop));

    const f64 p2 = isothermalLayerPressure(t1, p1, kIsothermalTop - kTroposphereTop);
    const f64 height = geopotential - kIsothermalTop;
    return fromTemperaturePressure(t1 - kInversionLapse * height,
                                   linearLayerPressure(t1, p2, kInversionLapse, height));
}

void Atmosphere::setConstantDensity(f32 density)
{
    if (!(density > 0.0f) || !isfinite(density))
    {
        Log::error("Atmosphere: constant density must be finite and above zero");
        return;
    }
    mConstantDensity = density;
    mModel = Model::ConstantDensity;
}

AtmosphereState Atmosphere::sample(f32 altitude) const
{
    if (mModel == Model::Standard)
        return standard(altitude);

    // Sea-level temperature, and the pressure it and this density imply.
    AtmosphereState state;
    state.density = mConstantDensity;
    state.temperature = SeaLevelTemperature;
    state.pressure = mConstantDensity * GasConstant * SeaLevelTemperature;
    state.speedOfSound = sqrtf(HeatCapacityRatio * GasConstant * SeaLevelTemperature);
    return state;
}

Math::vec3 Atmosphere::windAt(const Math::vec3& worldPosition) const
{
    Math::vec3 wind = mWind;
    if (mWindCallback)
    {
        const Math::vec3 gust = mWindCallback(worldPosition, mWindUserData);
        // Garbage from the callback must not poison every downstream force.
        if (isfinite(gust.x) && isfinite(gust.y) && isfinite(gust.z))
            wind += gust;
    }
    return wind;
}

} // namespace kinetix::Physics
