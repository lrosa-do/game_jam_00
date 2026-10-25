#include "check.h"

#include <dynamics/Atmosphere.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
namespace KM = kinetix::Math;

namespace
{
// |actual / expected - 1| within `fraction`.
bool within(f32 actual, f32 expected, f32 fraction)
{
    return fabsf(actual / expected - 1.0f) <= fraction;
}

void checkTableRow(f32 altitude, f32 temperature, f32 pressure, f32 density, f32 speedOfSound)
{
    const AtmosphereState s = Atmosphere::standard(altitude);
    if (!within(s.temperature, temperature, 0.005f) || !within(s.pressure, pressure, 0.005f) ||
        !within(s.density, density, 0.005f) || !within(s.speedOfSound, speedOfSound, 0.005f))
    {
        fprintf(stderr, "ISA %.0f m: T %.3f P %.1f rho %.5f a %.3f\n", double(altitude),
                double(s.temperature), double(s.pressure), double(s.density),
                double(s.speedOfSound));
    }
    CHECK(within(s.temperature, temperature, 0.005f));
    CHECK(within(s.pressure, pressure, 0.005f));
    CHECK(within(s.density, density, 0.005f));
    CHECK(within(s.speedOfSound, speedOfSound, 0.005f));
}

// Tabulated values (geometric altitude) from the 1976 standard.
void standardTables()
{
    checkTableRow(0.0f, 288.150f, 101325.0f, 1.22500f, 340.294f);
    checkTableRow(5000.0f, 255.676f, 54048.3f, 0.736429f, 320.529f);
    checkTableRow(11000.0f, 216.774f, 22699.9f, 0.364801f, 295.154f);
    checkTableRow(15000.0f, 216.650f, 12111.8f, 0.194755f, 295.069f);
    checkTableRow(25000.0f, 221.552f, 2549.22f, 0.0400837f, 298.389f);

    // Sea level is exact, not just within half a percent: everything else is
    // scaled from it.
    const AtmosphereState sea = Atmosphere::standard(0.0f);
    CHECK_NEAR(sea.density, 1.225, 1e-4);
    CHECK_NEAR(sea.pressure, 101325.0, 0.5);
}

void monotonicAndContinuous()
{
    AtmosphereState previous = Atmosphere::standard(-1000.0f);
    bool monotonic = true;
    f32 largestTemperatureStep = 0.0f;
    for (f32 h = -990.0f; h <= 30000.0f; h += 10.0f)
    {
        const AtmosphereState s = Atmosphere::standard(h);
        if (!(s.density < previous.density) || !(s.pressure < previous.pressure))
            monotonic = false;
        largestTemperatureStep =
            fmaxf(largestTemperatureStep, fabsf(s.temperature - previous.temperature));
        previous = s;
    }
    CHECK(monotonic);
    // 10 m at 6.5 K/km is 0.065 K.
    CHECK(largestTemperatureStep < 0.07f);

    const AtmosphereState high = Atmosphere::standard(1.0e7f);
    const AtmosphereState nan = Atmosphere::standard(NAN);
    CHECK(isfinite(high.density) && high.density > 0.0f);
    CHECK(isfinite(nan.density) && nan.density > 0.0f);
}

void constantDensity()
{
    Atmosphere air;
    CHECK(air.model() == Atmosphere::Model::Standard);
    air.setConstantDensity(0.9f);
    CHECK(air.model() == Atmosphere::Model::ConstantDensity);
    CHECK_NEAR(air.sample(0.0f).density, 0.9, 1e-6);
    CHECK_NEAR(air.sample(8000.0f).density, 0.9, 1e-6);
    CHECK_NEAR(air.sample(8000.0f).speedOfSound, 340.294, 0.01);

    air.setConstantDensity(-1.0f);
    CHECK_NEAR(air.constantDensity(), 0.9, 1e-6);
    air.setConstantDensity(0.0f);
    CHECK_NEAR(air.constantDensity(), 0.9, 1e-6);
}

void originAltitude()
{
    Atmosphere air;
    air.setOriginAltitude(1500.0f);
    CHECK_NEAR(air.altitudeAt(KM::vec3(3.0f, 500.0f, -7.0f)), 2000.0, 1e-3);
    CHECK_NEAR(air.sampleAt(KM::vec3(0.0f, 500.0f, 0.0f)).density,
               Atmosphere::standard(2000.0f).density, 1e-6);
}

KM::vec3 gust(const KM::vec3& position, void* userData)
{
    const f32 strength = *static_cast<const f32*>(userData);
    return KM::vec3(0.0f, strength * position.x, 0.0f);
}

KM::vec3 brokenGust(const KM::vec3&, void*)
{
    return KM::vec3(NAN, 0.0f, 0.0f);
}

void windAndGusts()
{
    Atmosphere air;
    air.setWind(KM::vec3(0.0f, 0.0f, -10.0f));
    KM::vec3 w = air.windAt(KM::vec3(2.0f, 0.0f, 0.0f));
    CHECK_NEAR(w.z, -10.0, 1e-6);
    CHECK_NEAR(w.y, 0.0, 1e-6);

    f32 strength = 1.5f;
    air.setWindCallback(&gust, &strength);
    w = air.windAt(KM::vec3(2.0f, 0.0f, 0.0f));
    CHECK_NEAR(w.y, 3.0, 1e-6);
    CHECK_NEAR(w.z, -10.0, 1e-6);

    air.setWindCallback(&brokenGust, nullptr);
    w = air.windAt(KM::vec3(2.0f, 0.0f, 0.0f));
    CHECK(isfinite(w.x));
    CHECK_NEAR(w.z, -10.0, 1e-6);
}
}

int main()
{
    standardTables();
    monotonicAndContinuous();
    constantDensity();
    originAltitude();
    windAndGusts();
    return TEST_RESULT();
}
