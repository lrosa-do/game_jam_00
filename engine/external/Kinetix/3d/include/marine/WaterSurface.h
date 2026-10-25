#ifndef KINETIX_PHYSICS_MARINE_WATERSURFACE_H
#define KINETIX_PHYSICS_MARINE_WATERSURFACE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

// ITTC reference values at 15 degrees C: sea water by default, fresh() for fresh water.
struct WaterProperties
{
    f32 density = 1025.9f;               // kg/m^3
    f32 kinematicViscosity = 1.1892e-6f; // m^2/s

    static WaterProperties sea()
    {
        return WaterProperties();
    }
    static WaterProperties fresh()
    {
        WaterProperties water;
        water.density = 999.1f;
        water.kinematicViscosity = 1.1386e-6f;
        return water;
    }
};

// One surface query. `velocity` is the water's own velocity at the point (wave orbital
// motion plus current), not the surface's.
struct WaterSample
{
    f32 height = 0.0f;
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    Math::vec3 velocity{0.0f};
};

// A free surface as a height field over the world XZ plane: height(x, z, t) is the world Y
// above (x, z), up is +Y. The water moves with the wave orbital velocity plus a uniform
// current and has one density and viscosity.
//
// gravity() serves both the hydrostatic pressure rho g h and the wave dispersion relation;
// it must match the PhysicsWorld's gravity for a floating body to balance its weight.
//
// The surface keeps its own clock so all bodies see the same waves; call advance(step) once
// per physics step. Timing as the forces' contract (see Boat): with PhysicsWorld::step()
// driven by hand, apply forces, step the world, then advance the water; from a step
// callback, advance the water first, then apply forces. The explicit-time queries ignore
// the clock.
class WaterSurface
{
public:
    static constexpr f32 StandardGravity = 9.81f;

    virtual ~WaterSurface() = default;

    // World Y of the free surface above (x, z) at `time`.
    virtual f32 height(f32 x, f32 z, f32 time) const = 0;
    // Unit normal of the surface above (x, z), pointing out of the water.
    virtual Math::vec3 normal(f32 x, f32 z, f32 time) const = 0;
    // Velocity due to the waves alone; decays with depth below the surface, equals the surface
    // particles' velocity at or above it.
    virtual Math::vec3 orbitalVelocity(const Math::vec3& point, f32 time) const = 0;
    // Height, normal and total water velocity (orbital plus current) for one point.
    virtual WaterSample sample(const Math::vec3& point, f32 time) const;

    f32 height(f32 x, f32 z) const
    {
        return height(x, z, mTime);
    }
    Math::vec3 normal(f32 x, f32 z) const
    {
        return normal(x, z, mTime);
    }
    WaterSample sample(const Math::vec3& point) const
    {
        return sample(point, mTime);
    }

    // Orbital velocity plus the current.
    Math::vec3 flowVelocity(const Math::vec3& point, f32 time) const
    {
        return orbitalVelocity(point, time) + mCurrent;
    }
    Math::vec3 flowVelocity(const Math::vec3& point) const
    {
        return flowVelocity(point, mTime);
    }
    // How far below the surface a point is; negative above it.
    f32 depth(const Math::vec3& point, f32 time) const
    {
        return height(point.x, point.z, time) - point.y;
    }
    f32 depth(const Math::vec3& point) const
    {
        return depth(point, mTime);
    }

    // Uniform current, world space, m/s: the direction the water flows.
    void setCurrent(const Math::vec3& current);
    const Math::vec3& current() const
    {
        return mCurrent;
    }

    void setProperties(const WaterProperties& properties);
    const WaterProperties& properties() const
    {
        return mProperties;
    }
    f32 density() const
    {
        return mProperties.density;
    }
    f32 kinematicViscosity() const
    {
        return mProperties.kinematicViscosity;
    }

    // Positive magnitude, m/s^2. Changing it changes the waves' frequencies.
    void setGravity(f32 gravity);
    f32 gravity() const
    {
        return mGravity;
    }

    f32 time() const
    {
        return mTime;
    }
    void setTime(f32 time);
    void advance(f32 step);

protected:
    virtual void gravityChanged()
    {
    }

    WaterProperties mProperties;
    Math::vec3 mCurrent{0.0f};
    f32 mGravity = StandardGravity;
    f32 mTime = 0.0f;
};

class FlatWater final : public WaterSurface
{
public:
    explicit FlatWater(f32 level = 0.0f)
        : mLevel(level)
    {
    }

    void setLevel(f32 level)
    {
        mLevel = level;
    }
    f32 level() const
    {
        return mLevel;
    }

    f32 height(f32 x, f32 z, f32 time) const override;
    Math::vec3 normal(f32 x, f32 z, f32 time) const override;
    Math::vec3 orbitalVelocity(const Math::vec3& point, f32 time) const override;
    WaterSample sample(const Math::vec3& point, f32 time) const override;

    using WaterSurface::height;
    using WaterSurface::normal;
    using WaterSurface::sample;

private:
    f32 mLevel = 0.0f;
};

// Sum of Gerstner (trochoidal) waves on deep water. Wave i has amplitude A, wavelength
// lambda (k = 2 pi / lambda), horizontal direction d, phase phi, steepness s in [0, 1] and
// omega^2 = g k. A particle with rest position p0 = (x0, z0) is displaced to
//
//   x = x0 - sum_i s_i A_i d_i.x sin(theta_i)
//   z = z0 - sum_i s_i A_i d_i.z sin(theta_i)
//   y = level + sum_i A_i cos(theta_i),   theta_i = k_i (d_i . p0) - omega_i t + phi_i
//
// s = 1 is the exact trochoid (circular orbits of radius A), s = 0 the linear (Airy)
// sinusoid.
//
// The surface folds over itself where the Jacobian of p0 -> (x, z) vanishes, impossible
// while sum_i s_i k_i A_i < 1; the horizontal factors are scaled down together to keep that
// sum at or below MaxTotalSteepness (heights kept). effectiveSteepness(i) is what was used.
//
// Height query: the height above (x, z) is that of the particle whose DISPLACED position is
// (x, z). The rest point comes from p0 <- (x, z) - D(p0), D the horizontal displacement,
// starting at (x, z): a contraction with L = sum s k A <= MaxTotalSteepness, so after n
// iterations the error is within L^n * sum(s A) in the rest point, sum(k A) times that in
// height. For L ~ 0.2-0.3 the default eight iterations give errors below 1e-5 of the
// amplitude; it stops early once a step moves the point by under 0.1 mm.
//
// Orbital velocity (linear theory per component, the time derivative of the trochoid at
// s = 1):
//   u_h = A omega cos(theta) e^(-k h) d,   u_y = A omega sin(theta) e^(-k h)
// h the depth below the local surface (Wheeler stretching, so the exponential stays <= 1
// inside a crest), at the rest point of the particle above the query.
//
// Limits: deep water only, no breaking, no interaction between components, no radiation or
// diffraction by floating bodies.
class GerstnerWaves final : public WaterSurface
{
public:
    static constexpr f32 MaxTotalSteepness = 0.8f;
    static constexpr u32 MaxWaves = 32;

    struct Wave
    {
        f32 amplitude = 0.5f;  // m, crest height above the mean level
        f32 wavelength = 20.0f; // m
        // Direction of travel, (x, z) in world space; normalised.
        Math::vec2 direction{0.0f, 1.0f};
        f32 phase = 0.0f;
        f32 steepness = 1.0f;
    };

    explicit GerstnerWaves(f32 level = 0.0f);

    void setLevel(f32 level)
    {
        mLevel = level;
    }
    f32 level() const
    {
        return mLevel;
    }

    // Returns the index, or MaxWaves (and logs) when full or invalid.
    u32 addWave(const Wave& wave);
    void clearWaves();
    u32 waveCount() const
    {
        return static_cast<u32>(mWaves.size());
    }
    const Wave& wave(u32 index) const
    {
        return mWaves[index].wave;
    }
    f32 wavenumber(u32 index) const
    {
        return mWaves[index].k;
    }
    f32 angularFrequency(u32 index) const
    {
        return mWaves[index].omega;
    }
    f32 period(u32 index) const;
    f32 phaseSpeed(u32 index) const;
    f32 effectiveSteepness(u32 index) const
    {
        return mWaves[index].horizontal;
    }
    // sum s k A after the clamp.
    f32 totalSteepness() const
    {
        return mTotalSteepness;
    }

    // Fixed-point iterations of the height query, 1..64.
    void setSolverIterations(u32 iterations);
    u32 solverIterations() const
    {
        return mIterations;
    }

    // Where the particle at rest position (x0, z0) is at `time`.
    Math::vec3 surfacePoint(f32 x0, f32 z0, f32 time) const;
    // The rest position whose displaced point lies above (x, z).
    Math::vec2 restPosition(f32 x, f32 z, f32 time) const;

    f32 height(f32 x, f32 z, f32 time) const override;
    Math::vec3 normal(f32 x, f32 z, f32 time) const override;
    Math::vec3 orbitalVelocity(const Math::vec3& point, f32 time) const override;
    WaterSample sample(const Math::vec3& point, f32 time) const override;

    using WaterSurface::height;
    using WaterSurface::normal;
    using WaterSurface::sample;

private:
    struct Component
    {
        Wave wave;
        Math::vec2 direction{0.0f, 1.0f};
        f32 k = 0.0f;
        f32 omega = 0.0f;
        f32 horizontal = 0.0f; // s after the clamp
    };

    void gravityChanged() override;
    void rebuild();
    // Height, normal and per-component phase at the rest point above (x, z); `phases` may be null.
    f32 solve(f32 x, f32 z, f32 time, Math::vec3* normal, f32* phases) const;

    ct::Vector<Component> mWaves;
    f32 mLevel = 0.0f;
    f32 mTotalSteepness = 0.0f;
    u32 mIterations = 8;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_WATERSURFACE_H
