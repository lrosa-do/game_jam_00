#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "marine/WaterSurface.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kTwoPi = 6.28318530717959f;

// The fixed-point height solve stops once the rest point moves less than this, m.
constexpr f32 kRestTolerance = 1.0e-4f;
} // namespace

WaterSample WaterSurface::sample(const Math::vec3& point, f32 time) const
{
    WaterSample out;
    out.height = height(point.x, point.z, time);
    out.normal = normal(point.x, point.z, time);
    out.velocity = orbitalVelocity(point, time) + mCurrent;
    return out;
}

void WaterSurface::setCurrent(const Math::vec3& current)
{
    if (!isfinite(current.x) || !isfinite(current.y) || !isfinite(current.z))
    {
        Log::error("WaterSurface: the current must be finite");
        return;
    }
    mCurrent = current;
}

void WaterSurface::setProperties(const WaterProperties& properties)
{
    if (!(properties.density > 0.0f) || !isfinite(properties.density) ||
        !(properties.kinematicViscosity > 0.0f) || !isfinite(properties.kinematicViscosity))
    {
        Log::error("WaterSurface: density and viscosity must be positive and finite");
        return;
    }
    mProperties = properties;
}

void WaterSurface::setGravity(f32 gravity)
{
    if (!(gravity > 0.0f) || !isfinite(gravity))
    {
        Log::error("WaterSurface: gravity is a positive magnitude");
        return;
    }
    mGravity = gravity;
    gravityChanged();
}

void WaterSurface::setTime(f32 time)
{
    mTime = isfinite(time) ? time : 0.0f;
}

void WaterSurface::advance(f32 step)
{
    if (isfinite(step))
        mTime += step;
}

f32 FlatWater::height(f32 x, f32 z, f32 time) const
{
    (void)x;
    (void)z;
    (void)time;
    return mLevel;
}

Math::vec3 FlatWater::normal(f32 x, f32 z, f32 time) const
{
    (void)x;
    (void)z;
    (void)time;
    return Math::vec3(0.0f, 1.0f, 0.0f);
}

Math::vec3 FlatWater::orbitalVelocity(const Math::vec3& point, f32 time) const
{
    (void)point;
    (void)time;
    return Math::vec3(0.0f);
}

WaterSample FlatWater::sample(const Math::vec3& point, f32 time) const
{
    (void)point;
    (void)time;
    WaterSample out;
    out.height = mLevel;
    out.velocity = mCurrent;
    return out;
}

GerstnerWaves::GerstnerWaves(f32 level)
    : mLevel(level)
{
}

u32 GerstnerWaves::addWave(const Wave& wave)
{
    if (mWaves.size() >= MaxWaves)
    {
        Log::error("GerstnerWaves: at most %u waves", MaxWaves);
        return MaxWaves;
    }
    const f32 dirLength = sqrtf(wave.direction.x * wave.direction.x + wave.direction.y * wave.direction.y);
    if (!isfinite(wave.amplitude) || wave.amplitude < 0.0f || !(wave.wavelength > 0.0f) ||
        !isfinite(wave.wavelength) || !isfinite(wave.phase) || !(dirLength > 1.0e-6f) ||
        !isfinite(dirLength))
    {
        Log::error("GerstnerWaves: a wave needs a finite amplitude >= 0, a wavelength > 0 and a "
                   "non-zero direction");
        return MaxWaves;
    }
    Component component;
    component.wave = wave;
    component.wave.steepness = isfinite(wave.steepness) ? Clamp(wave.steepness, 0.0f, 1.0f) : 0.0f;
    component.direction = Math::vec2(wave.direction.x / dirLength, wave.direction.y / dirLength);
    component.wave.direction = component.direction;
    mWaves.push_back(component);
    rebuild();
    return static_cast<u32>(mWaves.size() - 1);
}

void GerstnerWaves::clearWaves()
{
    mWaves.clear();
    rebuild();
}

void GerstnerWaves::gravityChanged()
{
    rebuild();
}

void GerstnerWaves::rebuild()
{
    f32 total = 0.0f;
    for (Component& c : mWaves)
    {
        c.k = kTwoPi / c.wave.wavelength;
        // Deep water: omega^2 = g k.
        c.omega = sqrtf(mGravity * c.k);
        total += c.wave.steepness * c.k * c.wave.amplitude;
    }
    // Keeps the horizontal map p0 -> p0 + D(p0) one-to-one: its Jacobian is
    // at least 1 - sum s k A.
    const f32 scale = total > MaxTotalSteepness ? MaxTotalSteepness / total : 1.0f;
    mTotalSteepness = 0.0f;
    for (Component& c : mWaves)
    {
        c.horizontal = c.wave.steepness * scale;
        mTotalSteepness += c.horizontal * c.k * c.wave.amplitude;
    }
}

f32 GerstnerWaves::period(u32 index) const
{
    return kTwoPi / mWaves[index].omega;
}

f32 GerstnerWaves::phaseSpeed(u32 index) const
{
    return mWaves[index].omega / mWaves[index].k;
}

void GerstnerWaves::setSolverIterations(u32 iterations)
{
    mIterations = Clamp(iterations, 1u, 64u);
}

Math::vec3 GerstnerWaves::surfacePoint(f32 x0, f32 z0, f32 time) const
{
    Math::vec3 p(x0, mLevel, z0);
    for (const Component& c : mWaves)
    {
        const f32 theta = c.k * (c.direction.x * x0 + c.direction.y * z0) - c.omega * time + c.wave.phase;
        const f32 horizontal = c.horizontal * c.wave.amplitude * sinf(theta);
        p.x -= horizontal * c.direction.x;
        p.z -= horizontal * c.direction.y;
        p.y += c.wave.amplitude * cosf(theta);
    }
    return p;
}

Math::vec2 GerstnerWaves::restPosition(f32 x, f32 z, f32 time) const
{
    f32 x0 = x;
    f32 z0 = z;
    for (u32 iteration = 0; iteration < mIterations; ++iteration)
    {
        f32 dx = 0.0f;
        f32 dz = 0.0f;
        for (const Component& c : mWaves)
        {
            const f32 theta =
                c.k * (c.direction.x * x0 + c.direction.y * z0) - c.omega * time + c.wave.phase;
            const f32 horizontal = c.horizontal * c.wave.amplitude * sinf(theta);
            dx -= horizontal * c.direction.x;
            dz -= horizontal * c.direction.y;
        }
        const f32 nx = x - dx;
        const f32 nz = z - dz;
        const f32 moved = fabsf(nx - x0) + fabsf(nz - z0);
        x0 = nx;
        z0 = nz;
        if (moved < kRestTolerance)
            break;
    }
    return Math::vec2(x0, z0);
}

f32 GerstnerWaves::solve(f32 x, f32 z, f32 time, Math::vec3* normal, f32* phases) const
{
    const Math::vec2 rest = restPosition(x, z, time);
    f32 height = mLevel;
    // Derivatives of the displaced point with respect to the rest position, for the normal.
    f32 dxdx0 = 1.0f;
    f32 dxdz0 = 0.0f;
    f32 dzdx0 = 0.0f;
    f32 dzdz0 = 1.0f;
    f32 dydx0 = 0.0f;
    f32 dydz0 = 0.0f;
    u32 index = 0;
    for (const Component& c : mWaves)
    {
        const f32 theta =
            c.k * (c.direction.x * rest.x + c.direction.y * rest.y) - c.omega * time + c.wave.phase;
        const f32 sinTheta = sinf(theta);
        const f32 cosTheta = cosf(theta);
        height += c.wave.amplitude * cosTheta;
        if (phases)
            phases[index] = theta;
        ++index;
        if (normal)
        {
            const f32 kA = c.k * c.wave.amplitude;
            const f32 pinch = c.horizontal * kA * cosTheta;
            dxdx0 -= pinch * c.direction.x * c.direction.x;
            dxdz0 -= pinch * c.direction.x * c.direction.y;
            dzdx0 -= pinch * c.direction.y * c.direction.x;
            dzdz0 -= pinch * c.direction.y * c.direction.y;
            dydx0 -= kA * sinTheta * c.direction.x;
            dydz0 -= kA * sinTheta * c.direction.y;
        }
    }
    if (normal)
    {
        const Math::vec3 alongX(dxdx0, dydx0, dzdx0);
        const Math::vec3 alongZ(dxdz0, dydz0, dzdz0);
        Math::vec3 n = Math::cross(alongZ, alongX);
        const f32 length = Math::length(n);
        *normal = length > 1.0e-12f && isfinite(length) ? n / length : Math::vec3(0.0f, 1.0f, 0.0f);
    }
    return height;
}

f32 GerstnerWaves::height(f32 x, f32 z, f32 time) const
{
    return solve(x, z, time, nullptr, nullptr);
}

Math::vec3 GerstnerWaves::normal(f32 x, f32 z, f32 time) const
{
    Math::vec3 n(0.0f, 1.0f, 0.0f);
    solve(x, z, time, &n, nullptr);
    return n;
}

Math::vec3 GerstnerWaves::orbitalVelocity(const Math::vec3& point, f32 time) const
{
    return sample(point, time).velocity - mCurrent;
}

WaterSample GerstnerWaves::sample(const Math::vec3& point, f32 time) const
{
    f32 phases[MaxWaves];
    WaterSample out;
    out.height = solve(point.x, point.z, time, &out.normal, phases);
    // Wheeler stretching: depth below the actual surface, never negative.
    const f32 depth = Max(out.height - point.y, 0.0f);
    Math::vec3 velocity(0.0f);
    for (u32 i = 0; i < mWaves.size(); ++i)
    {
        const Component& c = mWaves[i];
        const f32 speed = c.wave.amplitude * c.omega * expf(-c.k * depth);
        const f32 horizontal = speed * cosf(phases[i]);
        velocity.x += horizontal * c.direction.x;
        velocity.z += horizontal * c.direction.y;
        velocity.y += speed * sinf(phases[i]);
    }
    out.velocity = velocity + mCurrent;
    return out;
}

} // namespace kinetix::Physics
