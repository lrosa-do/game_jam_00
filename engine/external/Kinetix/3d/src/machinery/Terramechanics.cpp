#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/Terramechanics.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kDegrees = 3.14159265358979f / 180.0f;

// Below this sinkage (1 mm) the power law's slope (unbounded at zero for n < 1) is read at it.
constexpr f32 kMinimumSlopeSinkage = 1.0e-3f;

f32 modulus(const SoilParameters& soil, f32 width)
{
    return soil.kc / Math::max(width, 1.0e-3f) + soil.kphi;
}

}

SoilParameters SoilParameters::drySand()
{
    SoilParameters s;
    s.n = 1.1f;
    s.kc = 0.99e3f;
    s.kphi = 1528.43e3f;
    s.cohesion = 1.04e3f;
    s.frictionAngle = 28.0f * kDegrees;
    s.shearModulus = 0.01f;
    s.density = 1600.0f;
    s.toolFrictionAngle = 20.0f * kDegrees;
    return s;
}

SoilParameters SoilParameters::loam()
{
    SoilParameters s;
    s.n = 0.7f;
    s.kc = 5.27e3f;
    s.kphi = 1515.04e3f;
    s.cohesion = 1.72e3f;
    s.frictionAngle = 29.0f * kDegrees;
    s.shearModulus = 0.025f;
    s.density = 1700.0f;
    s.toolFrictionAngle = 22.0f * kDegrees;
    return s;
}

SoilParameters SoilParameters::clay()
{
    SoilParameters s;
    s.n = 0.5f;
    s.kc = 13.19e3f;
    s.kphi = 692.15e3f;
    s.cohesion = 4.14e3f;
    s.frictionAngle = 13.0f * kDegrees;
    s.shearModulus = 0.025f;
    s.density = 1900.0f;
    s.toolFrictionAngle = 10.0f * kDegrees;
    return s;
}

SoilParameters SoilParameters::heavyClay()
{
    SoilParameters s;
    s.n = 0.13f;
    s.kc = 12.70e3f;
    s.kphi = 1555.95e3f;
    s.cohesion = 68.95e3f;
    s.frictionAngle = 34.0f * kDegrees;
    s.shearModulus = 0.02f;
    s.density = 1950.0f;
    s.toolFrictionAngle = 20.0f * kDegrees;
    return s;
}

SoilParameters SoilParameters::snow()
{
    SoilParameters s;
    s.n = 1.6f;
    s.kc = 4.37e3f;
    s.kphi = 196.72e3f;
    s.cohesion = 1.03e3f;
    s.frictionAngle = 19.7f * kDegrees;
    s.shearModulus = 0.04f;
    s.density = 300.0f;
    s.toolFrictionAngle = 10.0f * kDegrees;
    return s;
}

SoilParameters SoilParameters::hardSurface(f32 mu)
{
    SoilParameters s;
    s.hard = true;
    s.friction = isfinite(mu) ? Math::max(mu, 0.0f) : 0.7f;
    s.cohesion = 0.0f;
    s.frictionAngle = atanf(s.friction);
    s.shearModulus = 0.005f;
    s.kc = 0.0f;
    s.kphi = 0.0f;
    s.n = 1.0f;
    s.density = 2400.0f;
    s.toolFrictionAngle = atanf(s.friction);
    return s;
}

namespace Terramechanics
{

f32 pressure(const SoilParameters& soil, f32 sinkage, f32 width)
{
    if (!(sinkage > 0.0f))
        return 0.0f;
    if (soil.hard)
        return soil.hardStiffness * sinkage;
    return modulus(soil, width) * powf(sinkage, soil.n);
}

f32 pressureSlope(const SoilParameters& soil, f32 sinkage, f32 width)
{
    if (soil.hard)
        return soil.hardStiffness;
    const f32 z = Math::max(sinkage, kMinimumSlopeSinkage);
    return soil.n * modulus(soil, width) * powf(z, soil.n - 1.0f);
}

f32 sinkage(const SoilParameters& soil, f32 p, f32 width)
{
    if (!(p > 0.0f))
        return 0.0f;
    if (soil.hard)
        return soil.hardStiffness > 0.0f ? p / soil.hardStiffness : 0.0f;
    return powf(p / modulus(soil, width), 1.0f / soil.n);
}

f32 shearStrength(const SoilParameters& soil, f32 normalPressure)
{
    const f32 sigma = Math::max(normalPressure, 0.0f);
    if (soil.hard)
        return soil.friction * sigma;
    return soil.cohesion + sigma * tanf(soil.frictionAngle);
}

f32 shearStress(const SoilParameters& soil, f32 normalPressure, f32 shearDisplacement)
{
    const f32 j = Math::max(shearDisplacement, 0.0f);
    return shearStrength(soil, normalPressure) * (1.0f - expf(-j / Math::max(soil.shearModulus, 1.0e-5f)));
}

f32 maxTractiveEffort(const SoilParameters& soil, f32 load, f32 width, f32 length)
{
    const f32 area = width * length;
    if (soil.hard)
        return soil.friction * Math::max(load, 0.0f);
    return area * soil.cohesion + Math::max(load, 0.0f) * tanf(soil.frictionAngle);
}

f32 tractiveEffort(const SoilParameters& soil, f32 load, f32 width, f32 length, f32 slip)
{
    const f32 i = fabsf(slip);
    const f32 maximum = maxTractiveEffort(soil, load, width, length);
    const f32 K = Math::max(soil.shearModulus, 1.0e-5f);
    const f32 x = i * length / K;
    if (x < 1.0e-6f)
        return 0.0f;
    const f32 share = 1.0f - (1.0f - expf(-x)) / x;
    return (slip < 0.0f ? -1.0f : 1.0f) * maximum * share;
}

f32 compactionResistance(const SoilParameters& soil, f32 width, f32 z)
{
    if (soil.hard || !(z > 0.0f))
        return 0.0f;
    return width * modulus(soil, width) * powf(z, soil.n + 1.0f) / (soil.n + 1.0f);
}

} // namespace Terramechanics

} // namespace kinetix::Physics
