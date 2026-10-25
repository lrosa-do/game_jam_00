#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/Tyre.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;

f32 finiteOr(f32 value, f32 fallback)
{
    return isfinite(value) ? value : fallback;
}

f32 positiveOr(f32 value, f32 minimum, f32 fallback)
{
    return isfinite(value) ? Max(value, minimum) : fallback;
}
} // namespace

const char* surfaceTypeName(SurfaceType type)
{
    switch (type)
    {
    case SurfaceType::Asphalt:
        return "asphalt";
    case SurfaceType::WetAsphalt:
        return "wet asphalt";
    case SurfaceType::Gravel:
        return "gravel";
    case SurfaceType::Grass:
        return "grass";
    case SurfaceType::Mud:
        return "mud";
    case SurfaceType::Ice:
        return "ice";
    case SurfaceType::Soil:
        return "soil";
    case SurfaceType::Snow:
        return "snow";
    default:
        return "unknown";
    }
}

TyreParameters TyreParameters::carRoad()
{
    TyreParameters p;
    p.friction = 1.0f;
    p.lateralFrictionRatio = 0.95f;
    p.nominalLoad = 3500.0f;
    p.loadSensitivity = -0.12f;
    p.peakSlipRatio = 0.10f;
    p.longitudinalShape = 1.5f;
    p.longitudinalCurvature = 0.3f;
    p.peakSlipAngle = 0.13f;
    p.lateralShape = 1.35f;
    p.lateralCurvature = -0.3f;
    p.camberRatio = 0.08f;
    p.pneumaticTrail = 0.03f;
    p.longitudinalRelaxation = 0.12f;
    p.lateralRelaxation = 0.35f;
    p.rollingResistance = 0.012f;
    const f32 friction[SurfaceTypeCount] = {1.0f, 0.7f, 0.6f, 0.45f, 0.3f, 0.08f, 0.55f, 0.25f};
    const f32 rolling[SurfaceTypeCount] = {1.0f, 1.1f, 2.5f, 4.0f, 8.0f, 1.0f, 4.0f, 3.0f};
    for (u32 i = 0; i < SurfaceTypeCount; ++i)
    {
        p.surfaceFriction[i] = friction[i];
        p.surfaceRollingResistance[i] = rolling[i];
    }
    return p;
}

TyreParameters TyreParameters::truck(bool dual)
{
    TyreParameters p;
    p.friction = 0.85f;
    p.lateralFrictionRatio = 0.9f;
    p.nominalLoad = dual ? 50000.0f : 25000.0f;
    p.loadSensitivity = -0.15f;
    p.peakSlipRatio = 0.12f;
    p.longitudinalShape = 1.5f;
    p.longitudinalCurvature = 0.3f;
    p.peakSlipAngle = 0.10f;
    p.lateralShape = 1.3f;
    p.lateralCurvature = -0.3f;
    p.camberRatio = 0.05f;
    p.pneumaticTrail = 0.06f;
    p.longitudinalRelaxation = 0.35f;
    p.lateralRelaxation = 0.8f;
    p.rollingResistance = 0.006f;
    const f32 friction[SurfaceTypeCount] = {1.0f, 0.7f, 0.6f, 0.45f, 0.3f, 0.08f, 0.55f, 0.25f};
    const f32 rolling[SurfaceTypeCount] = {1.0f, 1.1f, 3.0f, 6.0f, 12.0f, 1.0f, 6.0f, 4.0f};
    for (u32 i = 0; i < SurfaceTypeCount; ++i)
    {
        p.surfaceFriction[i] = friction[i];
        p.surfaceRollingResistance[i] = rolling[i];
    }
    return p;
}

TyreParameters TyreParameters::tractorLug(f32 nominalLoad)
{
    TyreParameters p;
    p.friction = 1.0f;
    p.lateralFrictionRatio = 0.85f;
    p.nominalLoad = positiveOr(nominalLoad, 100.0f, 15000.0f);
    p.loadSensitivity = -0.08f;
    p.peakSlipRatio = 0.2f;
    p.longitudinalShape = 1.35f;
    p.longitudinalCurvature = 0.2f;
    p.peakSlipAngle = 0.17f;
    p.lateralShape = 1.3f;
    p.lateralCurvature = -0.2f;
    p.camberRatio = 0.05f;
    p.pneumaticTrail = 0.08f;
    p.longitudinalRelaxation = 0.5f;
    p.lateralRelaxation = 0.9f;
    p.rollingResistance = 0.02f;
    // Lower on hard roads, where only the lug faces touch and the rubber squirms.
    const f32 friction[SurfaceTypeCount] = {0.75f, 0.55f, 0.8f, 0.8f, 0.6f, 0.1f, 0.95f, 0.4f};
    const f32 rolling[SurfaceTypeCount] = {1.0f, 1.0f, 1.5f, 2.5f, 5.0f, 1.0f, 3.0f, 2.5f};
    for (u32 i = 0; i < SurfaceTypeCount; ++i)
    {
        p.surfaceFriction[i] = friction[i];
        p.surfaceRollingResistance[i] = rolling[i];
    }
    return p;
}

Tyre::Tyre()
{
    setParameters(TyreParameters());
}

Tyre::Tyre(const TyreParameters& parameters)
{
    setParameters(parameters);
}

void Tyre::setParameters(const TyreParameters& parameters)
{
    TyreParameters p = parameters;
    p.friction = positiveOr(p.friction, 0.0f, 1.0f);
    p.lateralFrictionRatio = positiveOr(p.lateralFrictionRatio, 0.0f, 1.0f);
    p.nominalLoad = positiveOr(p.nominalLoad, 1.0f, 4000.0f);
    p.loadSensitivity = Clamp(finiteOr(p.loadSensitivity, 0.0f), -1.0f, 1.0f);
    p.peakSlipRatio = positiveOr(p.peakSlipRatio, 1.0e-3f, 0.1f);
    p.peakSlipAngle = positiveOr(p.peakSlipAngle, 1.0e-3f, 0.12f);
    p.longitudinalShape = Clamp(finiteOr(p.longitudinalShape, 1.5f), 1.05f, 1.95f);
    p.lateralShape = Clamp(finiteOr(p.lateralShape, 1.35f), 1.05f, 1.95f);
    p.longitudinalCurvature = Clamp(finiteOr(p.longitudinalCurvature, 0.0f), -5.0f, 0.95f);
    p.lateralCurvature = Clamp(finiteOr(p.lateralCurvature, 0.0f), -5.0f, 0.95f);
    p.camberRatio = finiteOr(p.camberRatio, 0.0f);
    p.pneumaticTrail = positiveOr(p.pneumaticTrail, 0.0f, 0.0f);
    p.longitudinalRelaxation = positiveOr(p.longitudinalRelaxation, 0.01f, 0.15f);
    p.lateralRelaxation = positiveOr(p.lateralRelaxation, 0.01f, 0.4f);
    p.lowSpeedDamping = positiveOr(p.lowSpeedDamping, 0.0f, 0.0f);
    p.lowSpeedDampingFade = positiveOr(p.lowSpeedDampingFade, 0.01f, 3.0f);
    p.maxSlipRatio = positiveOr(p.maxSlipRatio, 0.05f, 1.5f);
    p.maxSlipAngle = Clamp(finiteOr(p.maxSlipAngle, 1.2f), 0.05f, 1.5f);
    p.rollingResistance = positiveOr(p.rollingResistance, 0.0f, 0.0f);
    for (u32 i = 0; i < SurfaceTypeCount; ++i)
    {
        p.surfaceFriction[i] = positiveOr(p.surfaceFriction[i], 0.0f, 1.0f);
        p.surfaceRollingResistance[i] = positiveOr(p.surfaceRollingResistance[i], 0.0f, 1.0f);
    }
    mParameters = p;
    mBx = solveStiffnessFactor(p.peakSlipRatio, p.longitudinalShape, p.longitudinalCurvature);
    mBy = solveStiffnessFactor(p.peakSlipAngle, p.lateralShape, p.lateralCurvature);
}

f32 Tyre::solveStiffnessFactor(f32 peakSlip, f32 c, f32 e)
{
    // The peak is where the sine's argument is pi/2: phi - E (phi - atan phi) = tan(pi / 2C),
    // phi = B x. The left side rises monotonically for E < 1, so Newton from the E = 0 answer
    // converges in a few steps.
    const f32 target = tanf(0.5f * kPi / c);
    f32 phi = target;
    for (u32 iteration = 0; iteration < 40; ++iteration)
    {
        const f32 f = phi - e * (phi - atanf(phi)) - target;
        const f32 slope = 1.0f - e * phi * phi / (1.0f + phi * phi);
        const f32 next = phi - f / slope;
        phi = Max(next, 1.0e-4f);
        if (fabsf(f) < 1.0e-7f * target)
            break;
    }
    return phi / peakSlip;
}

f32 Tyre::shape(f32 x, f32 b, f32 c, f32 e)
{
    const f32 bx = b * x;
    return sinf(c * atanf(bx - e * (bx - atanf(bx))));
}

f32 Tyre::shapeOverSlip(f32 x, f32 b, f32 c, f32 e)
{
    // sin(C atan(...)) ~ C B x near zero; below 1e-4 in B x that is exact and avoids 0 / 0.
    if (fabsf(b * x) < 1.0e-4f)
        return b * c;
    return shape(x, b, c, e) / x;
}

f32 Tyre::frictionCoefficient(f32 load, const SurfaceInfo& surface) const
{
    const TyreParameters& p = mParameters;
    const u32 type = Min(static_cast<u32>(surface.type), SurfaceTypeCount - 1u);
    const f32 sensitivity =
        Clamp(1.0f + p.loadSensitivity * (load - p.nominalLoad) / p.nominalLoad, 0.5f, 1.3f);
    const f32 scale = isfinite(surface.frictionScale) ? Max(surface.frictionScale, 0.0f) : 1.0f;
    return p.friction * p.surfaceFriction[type] * scale * sensitivity;
}

f32 Tyre::rollingResistanceCoefficient(const SurfaceInfo& surface) const
{
    const TyreParameters& p = mParameters;
    const u32 type = Min(static_cast<u32>(surface.type), SurfaceTypeCount - 1u);
    const f32 scale =
        isfinite(surface.rollingResistanceScale) ? Max(surface.rollingResistanceScale, 0.0f) : 1.0f;
    return p.rollingResistance * p.surfaceRollingResistance[type] * scale;
}

f32 Tyre::longitudinalStiffness(f32 load, const SurfaceInfo& surface) const
{
    if (!(load > 0.0f) || !isfinite(load))
        return 0.0f;
    return frictionCoefficient(load, surface) * load * mBx * mParameters.longitudinalShape;
}

f32 Tyre::corneringStiffness(f32 load, const SurfaceInfo& surface) const
{
    if (!(load > 0.0f) || !isfinite(load))
        return 0.0f;
    return frictionCoefficient(load, surface) * mParameters.lateralFrictionRatio * load * mBy *
           mParameters.lateralShape;
}

void Tyre::secantStiffness(f32 load, f32 slipRatio, f32 slipAngle, const SurfaceInfo& surface,
                           f32 camber, f32& kx, f32& ky) const
{
    kx = 0.0f;
    ky = 0.0f;
    if (!(load > 0.0f) || !isfinite(load) || !isfinite(slipRatio) || !isfinite(slipAngle))
        return;
    const TyreParameters& p = mParameters;
    const f32 mu = frictionCoefficient(load, surface);
    const f32 dx = mu * load;
    const f32 dy = mu * p.lateralFrictionRatio * load;
    const f32 alpha = slipAngle + (isfinite(camber) ? p.camberRatio * camber : 0.0f);
    const f32 sx = slipRatio / p.peakSlipRatio;
    const f32 sy = alpha / p.peakSlipAngle;
    const f32 s = sqrtf(sx * sx + sy * sy);
    kx = dx * shapeOverSlip(s * p.peakSlipRatio, mBx, p.longitudinalShape, p.longitudinalCurvature);
    ky = dy * shapeOverSlip(s * p.peakSlipAngle, mBy, p.lateralShape, p.lateralCurvature);
    // Force direction: near the peak the normalised slip's (sx, sy) / s (keeps each pure-slip
    // stiffness); far past it the patch slides and the force must oppose the sliding velocity
    // along (kappa, tan alpha). Blended between s = 1 and s = 3 so a locked, steered wheel does
    // not push sideways as if it rolled. Both are proportional to their own slip component, so
    // the secants stay finite and non-negative; a unit direction keeps the force in the ellipse.
    if (s > 1.0f)
    {
        const f32 tanAlpha = tanf(Clamp(alpha, -1.5f, 1.5f));
        const f32 m = sqrtf(slipRatio * slipRatio + tanAlpha * tanAlpha);
        const f32 w = Clamp(0.5f * (s - 1.0f), 0.0f, 1.0f);
        if (m > 1.0e-12f && w > 0.0f)
        {
            const f32 perX = (1.0f - w) / (p.peakSlipRatio * s) + w / m;
            const f32 tanOverAlpha = fabsf(alpha) > 1.0e-6f ? tanAlpha / alpha : 1.0f;
            const f32 perY = (1.0f - w) / (p.peakSlipAngle * s) + w * tanOverAlpha / m;
            const f32 bx = perX * slipRatio;
            const f32 by = perY * alpha;
            const f32 length = sqrtf(bx * bx + by * by);
            if (length > 1.0e-12f)
            {
                // kx was Fx0 / (s kappa_peak) with direction (sx / s); swap in the blended direction.
                const f32 fx0 = dx * shape(s * p.peakSlipRatio, mBx, p.longitudinalShape, p.longitudinalCurvature);
                const f32 fy0 = dy * shape(s * p.peakSlipAngle, mBy, p.lateralShape, p.lateralCurvature);
                kx = fx0 * perX / length;
                ky = fy0 * perY / length;
            }
        }
    }
    // A secant must not push the wrong way where the sine would turn negative.
    kx = Max(kx, 0.0f);
    ky = Max(ky, 0.0f);
}

f32 Tyre::trail(f32 slipRatio, f32 slipAngle, f32 camber) const
{
    const TyreParameters& p = mParameters;
    if (!isfinite(slipRatio) || !isfinite(slipAngle))
        return 0.0f;
    const f32 alpha = slipAngle + (isfinite(camber) ? p.camberRatio * camber : 0.0f);
    const f32 sx = slipRatio / p.peakSlipRatio;
    const f32 sy = alpha / p.peakSlipAngle;
    const f32 s = sqrtf(sx * sx + sy * sy);
    return p.pneumaticTrail * Max(1.0f - 0.5f * s, 0.0f);
}

TyreForces Tyre::evaluate(f32 load, f32 slipRatio, f32 slipAngle, const SurfaceInfo& surface,
                          f32 camber) const
{
    TyreForces out;
    if (!(load > 0.0f) || !isfinite(load) || !isfinite(slipRatio) || !isfinite(slipAngle))
        return out;
    const TyreParameters& p = mParameters;
    f32 kx = 0.0f;
    f32 ky = 0.0f;
    secantStiffness(load, slipRatio, slipAngle, surface, camber, kx, ky);
    const f32 alpha = slipAngle + (isfinite(camber) ? p.camberRatio * camber : 0.0f);
    const f32 mu = frictionCoefficient(load, surface);
    out.peakFx = mu * load;
    out.peakFy = mu * p.lateralFrictionRatio * load;
    out.fx = Clamp(kx * slipRatio, -out.peakFx, out.peakFx);
    out.fy = Clamp(ky * alpha, -out.peakFy, out.peakFy);
    out.mz = -trail(slipRatio, slipAngle, camber) * out.fy;
    out.rollingResistance = rollingResistanceCoefficient(surface) * load;
    return out;
}

} // namespace kinetix::Physics
