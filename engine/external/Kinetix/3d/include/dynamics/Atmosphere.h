#ifndef KINETIX_PHYSICS_DYNAMICS_ATMOSPHERE_H
#define KINETIX_PHYSICS_DYNAMICS_ATMOSPHERE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// The air at one altitude (for dynamic pressure and Mach number).
struct AtmosphereState
{
    f32 density = 1.225f;        // kg/m^3
    f32 pressure = 101325.0f;    // Pa
    f32 temperature = 288.15f;   // K
    f32 speedOfSound = 340.294f; // m/s
};

// The air the aerodynamics flies through: state as a function of altitude, and wind.
//
// Standard model: ISA 1976 (6.5 K/km lapse to 11 km, isothermal to 20 km,
// +1 K/km to 32 km), hydrostatic with the ideal gas law. Altitudes are
// geometric, converted to geopotential internally. The 32 km and -2 km states
// are held outside that range.
//
// Constant model: one density everywhere, sea-level temperature, pressure from
// the ideal gas law.
//
// Altitude is along world +Y plus originAltitude (y = 0 on a runway at 1500 m
// sets 1500).
class Atmosphere
{
public:
    enum class Model : u8
    {
        Standard,
        ConstantDensity
    };

    // Extra wind at a world position on top of the steady wind (gusts,
    // turbulence, thermals). State lives behind userData; the atmosphere has no clock.
    using WindCallback = Math::vec3 (*)(const Math::vec3& worldPosition, void* userData);

    static constexpr f32 SeaLevelTemperature = 288.15f;
    static constexpr f32 SeaLevelPressure = 101325.0f;
    static constexpr f32 SeaLevelDensity = 1.225f;
    static constexpr f32 GasConstant = 287.05287f;      // J/(kg K), dry air
    static constexpr f32 HeatCapacityRatio = 1.4f;
    static constexpr f32 StandardGravity = 9.80665f;    // m/s^2
    static constexpr f32 EarthRadius = 6356766.0f;      // m, the ISA reference radius

    Atmosphere() = default;

    void setModel(Model model)
    {
        mModel = model;
    }
    Model model() const
    {
        return mModel;
    }
    // Switches to the constant model too. A density of zero or below is refused.
    void setConstantDensity(f32 density);
    f32 constantDensity() const
    {
        return mConstantDensity;
    }

    void setOriginAltitude(f32 altitude)
    {
        mOriginAltitude = altitude;
    }
    f32 originAltitude() const
    {
        return mOriginAltitude;
    }
    f32 altitudeAt(const Math::vec3& worldPosition) const
    {
        return worldPosition.y + mOriginAltitude;
    }

    AtmosphereState sample(f32 altitude) const;
    AtmosphereState sampleAt(const Math::vec3& worldPosition) const
    {
        return sample(altitudeAt(worldPosition));
    }

    // The velocity of the AIR in world space, m/s: the direction it blows
    // towards. A 10 m/s headwind for something flying along +Z is (0, 0, -10).
    void setWind(const Math::vec3& wind)
    {
        mWind = wind;
    }
    const Math::vec3& wind() const
    {
        return mWind;
    }
    void setWindCallback(WindCallback callback, void* userData)
    {
        mWindCallback = callback;
        mWindUserData = userData;
    }
    Math::vec3 windAt(const Math::vec3& worldPosition) const;

    static AtmosphereState standard(f32 altitude);

private:
    Model mModel = Model::Standard;
    f32 mConstantDensity = SeaLevelDensity;
    f32 mOriginAltitude = 0.0f;
    Math::vec3 mWind{0.0f};
    WindCallback mWindCallback = nullptr;
    void* mWindUserData = nullptr;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_ATMOSPHERE_H
