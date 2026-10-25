#include "check.h"

#include <marine/WaterSurface.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;

bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// A moderate three-component sea: total steepness sum(s k A) about 0.24.
GerstnerWaves sea()
{
    GerstnerWaves waves(0.3f);
    GerstnerWaves::Wave a;
    a.amplitude = 0.6f;
    a.wavelength = 40.0f;
    a.direction = KM::vec2(1.0f, 0.3f);
    a.phase = 0.4f;
    a.steepness = 1.0f;
    waves.addWave(a);
    GerstnerWaves::Wave b;
    b.amplitude = 0.25f;
    b.wavelength = 13.0f;
    b.direction = KM::vec2(-0.4f, 1.0f);
    b.phase = 2.1f;
    b.steepness = 0.8f;
    waves.addWave(b);
    GerstnerWaves::Wave c;
    c.amplitude = 0.08f;
    c.wavelength = 5.0f;
    c.direction = KM::vec2(0.7f, -0.7f);
    c.phase = -1.0f;
    c.steepness = 0.6f;
    waves.addWave(c);
    return waves;
}

void flatWater()
{
    FlatWater water(1.5f);
    CHECK(water.height(3.0f, -7.0f, 12.0f) == 1.5f);
    const KM::vec3 n = water.normal(1.0f, 2.0f, 0.0f);
    CHECK(n.x == 0.0f && n.y == 1.0f && n.z == 0.0f);
    CHECK(KM::length(water.orbitalVelocity(KM::vec3(0.0f, -3.0f, 0.0f), 1.0f)) == 0.0f);
    water.setCurrent(KM::vec3(0.5f, 0.0f, -1.0f));
    const WaterSample s = water.sample(KM::vec3(4.0f, 0.0f, 4.0f));
    CHECK(s.height == 1.5f);
    CHECK(s.velocity.x == 0.5f && s.velocity.z == -1.0f);
    CHECK_NEAR(water.depth(KM::vec3(0.0f, -0.5f, 0.0f)), 2.0, 1e-6);

    CHECK_NEAR(water.density(), 1025.9, 1e-3);
    water.setProperties(WaterProperties::fresh());
    CHECK_NEAR(water.density(), 999.1, 1e-3);
    CHECK_NEAR(water.kinematicViscosity(), 1.1386e-6, 1e-10);
    WaterProperties bad;
    bad.density = -1.0f;
    water.setProperties(bad);
    CHECK_NEAR(water.density(), 999.1, 1e-3);
    water.setGravity(0.0f);
    CHECK_NEAR(water.gravity(), 9.81, 1e-6);
}

void emptyGerstnerIsFlat()
{
    GerstnerWaves waves(0.75f);
    FlatWater flat(0.75f);
    waves.setCurrent(KM::vec3(0.2f, 0.0f, 0.1f));
    flat.setCurrent(KM::vec3(0.2f, 0.0f, 0.1f));
    for (int i = 0; i < 20; ++i)
    {
        const KM::vec3 p(static_cast<f32>(i) * 1.7f - 15.0f, -0.3f * static_cast<f32>(i),
                         static_cast<f32>(i * i) * 0.13f);
        const f32 t = static_cast<f32>(i) * 0.37f;
        CHECK(waves.height(p.x, p.z, t) == flat.height(p.x, p.z, t));
        const WaterSample a = waves.sample(p, t);
        const WaterSample b = flat.sample(p, t);
        CHECK(a.height == b.height);
        CHECK(a.normal.x == b.normal.x && a.normal.y == b.normal.y && a.normal.z == b.normal.z);
        CHECK(a.velocity.x == b.velocity.x && a.velocity.y == b.velocity.y &&
              a.velocity.z == b.velocity.z);
    }
}

void dispersion()
{
    GerstnerWaves waves;
    GerstnerWaves::Wave w;
    w.wavelength = 50.0f;
    w.direction = KM::vec2(1.0f, 0.0f);
    waves.addWave(w);
    // omega^2 = g k, so c = sqrt(g lambda / 2 pi) and T = sqrt(2 pi lambda / g).
    CHECK_NEAR(waves.wavenumber(0), 2.0 * kPi / 50.0, 1e-6);
    CHECK_NEAR(waves.phaseSpeed(0), sqrt(9.81 * 50.0 / (2.0 * kPi)), 1e-4);
    CHECK_NEAR(waves.period(0), sqrt(2.0 * kPi * 50.0 / 9.81), 1e-4);
    const f32 c = waves.phaseSpeed(0);
    const f32 t = 3.0f;
    CHECK_NEAR(waves.height(c * t, 0.0f, t), waves.height(0.0f, 0.0f, 0.0f), 1e-4);
    CHECK_NEAR(waves.height(0.0f, 0.0f, 0.0f), 0.5, 1e-6);

    waves.setGravity(9.81f * 4.0f);
    CHECK_NEAR(waves.phaseSpeed(0), 2.0 * sqrt(9.81 * 50.0 / (2.0 * kPi)), 1e-3);
}

void heightMatchesDisplacedSurface()
{
    const GerstnerWaves waves = sea();
    printf("sea: total steepness %.3f\n", double(waves.totalSteepness()));
    CHECK(waves.totalSteepness() < GerstnerWaves::MaxTotalSteepness);
    f32 worst = 0.0f;
    f32 worstRest = 0.0f;
    for (int i = 0; i < 400; ++i)
    {
        const f32 x0 = -40.0f + 0.2f * static_cast<f32>(i);
        const f32 z0 = 7.0f - 0.13f * static_cast<f32>(i);
        const f32 t = 0.05f * static_cast<f32>(i);
        const KM::vec3 p = waves.surfacePoint(x0, z0, t);
        worst = fmaxf(worst, fabsf(waves.height(p.x, p.z, t) - p.y));
        const KM::vec2 rest = waves.restPosition(p.x, p.z, t);
        worstRest = fmaxf(worstRest, fabsf(rest.x - x0) + fabsf(rest.y - z0));
    }
    printf("height query vs displaced surface: worst %.2e m (rest point %.2e m)\n", double(worst),
           double(worstRest));
    // Amplitudes sum to 0.93 m; the error is the fixed point's tolerance
    // times the slope.
    CHECK(worst < 1.0e-4f);
    CHECK(worstRest < 5.0e-4f);

    // Fewer iterations: worse, and within the documented bound L^n sum(s A).
    GerstnerWaves coarse = sea();
    coarse.setSolverIterations(2);
    f32 coarseWorst = 0.0f;
    for (int i = 0; i < 200; ++i)
    {
        const KM::vec3 p = coarse.surfacePoint(0.37f * static_cast<f32>(i), 1.0f, 0.0f);
        coarseWorst = fmaxf(coarseWorst, fabsf(coarse.height(p.x, p.z, 0.0f) - p.y));
    }
    f32 sumSA = 0.0f;
    f32 sumKA = 0.0f;
    for (u32 i = 0; i < coarse.waveCount(); ++i)
    {
        sumSA += coarse.effectiveSteepness(i) * coarse.wave(i).amplitude;
        sumKA += coarse.wavenumber(i) * coarse.wave(i).amplitude;
    }
    const f32 L = coarse.totalSteepness();
    const f32 bound = L * L * sumSA * sumKA;
    printf("two iterations: worst %.2e m, bound %.2e m\n", double(coarseWorst), double(bound));
    CHECK(coarseWorst <= bound * 1.01f + 1e-6f);
    CHECK(coarseWorst > worst);
}

void normalMatchesSlope()
{
    const GerstnerWaves waves = sea();
    const f32 h = 1.0e-2f;
    f32 worst = 0.0f;
    for (int i = 0; i < 100; ++i)
    {
        const f32 x = -20.0f + 0.41f * static_cast<f32>(i);
        const f32 z = 3.0f + 0.29f * static_cast<f32>(i);
        const f32 t = 0.11f * static_cast<f32>(i);
        const f32 dhdx = (waves.height(x + h, z, t) - waves.height(x - h, z, t)) / (2.0f * h);
        const f32 dhdz = (waves.height(x, z + h, t) - waves.height(x, z - h, t)) / (2.0f * h);
        const KM::vec3 expected = KM::normalize(KM::vec3(-dhdx, 1.0f, -dhdz));
        const KM::vec3 n = waves.normal(x, z, t);
        CHECK_NEAR(KM::length(n), 1.0, 1e-5);
        worst = fmaxf(worst, KM::length(n - expected));
    }
    printf("normal vs finite-difference slope: worst %.2e\n", double(worst));
    CHECK(worst < 2.0e-3f);
}

void orbitalVelocityIsParticleVelocity()
{
    GerstnerWaves waves;
    GerstnerWaves::Wave a;
    a.amplitude = 0.4f;
    a.wavelength = 30.0f;
    a.direction = KM::vec2(1.0f, 0.5f);
    a.steepness = 1.0f;
    waves.addWave(a);
    GerstnerWaves::Wave b;
    b.amplitude = 0.15f;
    b.wavelength = 11.0f;
    b.direction = KM::vec2(-0.2f, 1.0f);
    b.phase = 1.3f;
    b.steepness = 1.0f;
    waves.addWave(b);
    CHECK_NEAR(waves.effectiveSteepness(0), 1.0, 1e-6);

    // Central differences over 10 ms near the origin, where a float
    // position is good to a few tenths of a micrometre.
    f32 worst = 0.0f;
    const f32 dt = 1.0e-2f;
    for (int i = 0; i < 60; ++i)
    {
        const f32 x0 = -3.0f + 0.1f * static_cast<f32>(i);
        const f32 z0 = 2.0f - 0.07f * static_cast<f32>(i);
        const f32 t = 0.2f * static_cast<f32>(i);
        const KM::vec3 p = waves.surfacePoint(x0, z0, t);
        const KM::vec3 expected =
            (waves.surfacePoint(x0, z0, t + dt) - waves.surfacePoint(x0, z0, t - dt)) / (2.0f * dt);
        const KM::vec3 u = waves.orbitalVelocity(p - KM::vec3(0.0f, 1.0e-6f, 0.0f), t);
        worst = fmaxf(worst, KM::length(u - expected));
    }
    printf("orbital velocity vs particle velocity: worst %.2e m/s\n", double(worst));
    CHECK(worst < 5.0e-4f);

    // Decays as exp(-k h) with depth below the local surface.
    GerstnerWaves single;
    single.addWave(a);
    const f32 k = single.wavenumber(0);
    const f32 omega = single.angularFrequency(0);
    const f32 surface = single.height(0.0f, 0.0f, 0.0f);
    const KM::vec3 deep(0.0f, surface - 5.0f, 0.0f);
    CHECK_NEAR(KM::length(single.orbitalVelocity(deep, 0.0f)), a.amplitude * omega * expf(-k * 5.0f), 1e-4);
    CHECK_NEAR(KM::length(single.orbitalVelocity(KM::vec3(0.0f, surface + 3.0f, 0.0f), 0.0f)),
               a.amplitude * omega, 1e-4);
    single.setCurrent(KM::vec3(1.0f, 0.0f, 0.0f));
    const KM::vec3 flow = single.flowVelocity(deep, 0.0f);
    CHECK_NEAR(flow.x - single.orbitalVelocity(deep, 0.0f).x, 1.0, 1e-6);
}

void steepnessIsClamped()
{
    GerstnerWaves waves;
    for (int i = 0; i < 4; ++i)
    {
        GerstnerWaves::Wave w;
        w.amplitude = 1.2f;
        w.wavelength = 12.0f + 2.0f * static_cast<f32>(i);
        w.direction = KM::vec2(cosf(0.3f * static_cast<f32>(i)), sinf(0.3f * static_cast<f32>(i)));
        w.steepness = 1.0f;
        waves.addWave(w);
    }
    printf("steep sea: clamped total %.3f, first wave s = %.3f\n", double(waves.totalSteepness()),
           double(waves.effectiveSteepness(0)));
    CHECK_NEAR(waves.totalSteepness(), GerstnerWaves::MaxTotalSteepness, 1e-5);
    CHECK(waves.effectiveSteepness(0) < 1.0f);
    CHECK_NEAR(waves.wave(0).amplitude, 1.2, 1e-6);

    f32 previous = -1.0e9f;
    bool monotonic = true;
    bool finite = true;
    for (int i = 0; i < 2000; ++i)
    {
        const f32 x0 = -30.0f + 0.03f * static_cast<f32>(i);
        const KM::vec3 p = waves.surfacePoint(x0, 0.0f, 0.7f);
        if (p.x <= previous)
            monotonic = false;
        previous = p.x;
        const KM::vec3 n = waves.normal(p.x, p.z, 0.7f);
        finite = finite && isfinite(waves.height(p.x, p.z, 0.7f)) && finiteVec(n) && n.y > 0.0f;
    }
    CHECK(monotonic);
    CHECK(finite);

    GerstnerWaves::Wave bad;
    bad.wavelength = 0.0f;
    CHECK(waves.addWave(bad) == GerstnerWaves::MaxWaves);
    bad.wavelength = 10.0f;
    bad.direction = KM::vec2(0.0f, 0.0f);
    CHECK(waves.addWave(bad) == GerstnerWaves::MaxWaves);
    CHECK(waves.waveCount() == 4);
}

void waterClock()
{
    GerstnerWaves waves = sea();
    CHECK(waves.time() == 0.0f);
    for (int i = 0; i < 60; ++i)
        waves.advance(1.0f / 60.0f);
    CHECK_NEAR(waves.time(), 1.0, 1e-5);
    CHECK(waves.height(2.0f, 3.0f) == waves.height(2.0f, 3.0f, waves.time()));
    waves.setTime(5.0f);
    const WaterSample s = waves.sample(KM::vec3(1.0f, -1.0f, 1.0f));
    const WaterSample e = waves.sample(KM::vec3(1.0f, -1.0f, 1.0f), 5.0f);
    CHECK(s.height == e.height && s.velocity.y == e.velocity.y);
}
}

int main()
{
    flatWater();
    emptyGerstnerIsFlat();
    dispersion();
    heightMatchesDisplacedSurface();
    normalMatchesSlope();
    orbitalVelocityIsParticleVelocity();
    steepnessIsClamped();
    waterClock();
    return TEST_RESULT();
}
