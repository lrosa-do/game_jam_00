#ifndef KINETIX_PHYSICS_GROUND_TYRE_H
#define KINETIX_PHYSICS_GROUND_TYRE_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// What a wheel is standing on; the tyre has its own friction and rolling resistance
// multiplier for each.
enum class SurfaceType : u8
{
    Asphalt,
    WetAsphalt,
    Gravel,
    Grass,
    Mud,
    Ice,
    Soil, // dry dirt, a ploughed or packed field
    Snow,
    Count
};

constexpr u32 SurfaceTypeCount = static_cast<u32>(SurfaceType::Count);

const char* surfaceTypeName(SurfaceType type);

// The surface under one contact (from the vehicle's surface callback): a type plus two
// scales on top of the tyre's table for what the type does not capture (puddle, black ice).
struct SurfaceInfo
{
    SurfaceType type = SurfaceType::Asphalt;
    f32 frictionScale = 1.0f;
    f32 rollingResistanceScale = 1.0f;
};

// Pure-slip "magic formula" in each direction,
//     F0(x) = D sin(C atan(B x - E (B x - atan(B x)))),
// D = mu(Fz) Fz the peak, C the shape factor (> 1), E the curvature near the peak, B solved
// from the peak slip so the curve peaks exactly there. Slip ratio drives Fx, slip angle
// (rad) Fy.
// Load sensitivity: mu(Fz) = mu0 * (1 + loadSensitivity (Fz - Fz0) / Fz0), clamped to
// [0.5, 1.3] mu0.
// Relaxation lengths, low-speed damping and slip limits are not used by the steady-state
// evaluate(); they serve GroundVehicle's transient (carcass deflection) formulation.
struct TyreParameters
{
    // Peak friction coefficient at the nominal load on asphalt.
    f32 friction = 1.0f;
    f32 lateralFrictionRatio = 0.95f;
    f32 nominalLoad = 4000.0f; // N
    f32 loadSensitivity = -0.1f;

    f32 peakSlipRatio = 0.1f;
    f32 longitudinalShape = 1.65f;
    f32 longitudinalCurvature = 0.1f;
    f32 peakSlipAngle = 0.12f; // rad
    f32 lateralShape = 1.35f;
    f32 lateralCurvature = -0.2f;

    // Camber thrust as an equivalent slip angle: alpha + ratio * camber.
    // Positive camber (top of the wheel leaning to the tyre's left) pushes
    // left.
    f32 camberRatio = 0.08f;
    // Pneumatic trail at small slips; it shrinks to nothing at twice the
    // peak (combined) slip. Mz = -t Fy.
    f32 pneumaticTrail = 0.03f; // m

    // Distance the tyre rolls to build up ~63% of a step in slip.
    f32 longitudinalRelaxation = 0.15f; // m
    f32 lateralRelaxation = 0.4f;       // m
    // Carcass damping at walking pace and below, per newton of load, fading out by fadeSpeed;
    // stops a parked tyre ringing on its own stiffness. N s/m per N.
    f32 lowSpeedDamping = 1.5f;
    f32 lowSpeedDampingFade = 3.0f; // m/s
    // Transient slips are limited to these so a wheel spinning at a standstill saturates
    // instead of winding up its carcass.
    f32 maxSlipRatio = 1.5f;
    f32 maxSlipAngle = 1.2f; // rad

    // Rolling resistance coefficient on asphalt; the force is C_rr Fz,
    // applied as a moment C_rr Fz r against the wheel's spin.
    f32 rollingResistance = 0.012f;

    f32 surfaceFriction[SurfaceTypeCount] = {1.0f, 0.75f, 0.6f, 0.45f, 0.35f, 0.1f, 0.6f, 0.3f};
    f32 surfaceRollingResistance[SurfaceTypeCount] = {1.0f, 1.1f, 2.0f, 4.0f, 8.0f, 1.0f, 3.0f, 3.0f};

    // Passenger car summer road tyre (205/55 R16 class, ~3.5 kN nominal).
    static TyreParameters carRoad();
    // Heavy truck steer or drive tyre (315/80 R22.5 class, ~25 kN); `dual` doubles the nominal
    // load and stiffness for a twin-tyre wheel end.
    static TyreParameters truck(bool dual = false);
    // Lugged agricultural tyre: high grip and peak slip on soil, less grip on asphalt.
    static TyreParameters tractorLug(f32 nominalLoad = 15000.0f);
};

// What one evaluation gives. Forces in the contact frame: Fx along the wheel
// heading, Fy to its left, Mz about the contact normal (positive turning the
// wheel's heading to the left).
struct TyreForces
{
    f32 fx = 0.0f;
    f32 fy = 0.0f;
    f32 mz = 0.0f;
    f32 rollingResistance = 0.0f;
    f32 peakFx = 0.0f;
    f32 peakFy = 0.0f;
};

// The tyre model alone: no wheel, vehicle or state. Contact frame signs: slip ratio kappa =
// (omega r - vx) / |vx|, positive driving; slip angle alpha = atan(-vy / |vx|), positive
// when the patch slides right, so Fy has the sign of alpha.
//
// Combined slip: sx = kappa / kappa_peak, sy = alpha / alpha_peak, s = |(sx, sy)|,
//     Fx = (sx / s) Fx0(s kappa_peak),  Fy = (sy / s) Fy0(s alpha_peak),
// so (Fx/Dx)^2 + (Fy/Dy)^2 <= 1 always (friction ellipse built in); near zero slip each
// direction keeps its pure-slip stiffness. For s from 1 to 3 the direction (sx, sy) / s is
// blended into the sliding direction (kappa, tan alpha) / |.|, so a locked wheel's force
// opposes its sliding velocity.
class Tyre
{
public:
    Tyre();
    explicit Tyre(const TyreParameters& parameters);

    // Sanitised: shape factors into (1, 2), curvature below 1, positive
    // peaks, lengths and loads. B is solved here from the peak slips.
    void setParameters(const TyreParameters& parameters);
    const TyreParameters& parameters() const
    {
        return mParameters;
    }

    // Steady-state forces at a given load (N), slip ratio, slip angle (rad)
    // and camber (rad). Zero load, or a non-finite input, gives zero force.
    TyreForces evaluate(f32 load, f32 slipRatio, f32 slipAngle, const SurfaceInfo& surface,
                        f32 camber = 0.0f) const;

    // Peak friction coefficient (longitudinal) at this load on this surface.
    f32 frictionCoefficient(f32 load, const SurfaceInfo& surface) const;
    f32 rollingResistanceCoefficient(const SurfaceInfo& surface) const;

    // Secant stiffnesses of the combined model, Fx = kx * kappa and Fy = ky * alpha (N per unit
    // slip, N per rad), >= 0 and finite at zero slip, where they equal the pure-slip slopes
    // D B C. Used by GroundVehicle's implicit tyre rows.
    void secantStiffness(f32 load, f32 slipRatio, f32 slipAngle, const SurfaceInfo& surface,
                         f32 camber, f32& kx, f32& ky) const;

    // Pneumatic trail, m: shrinks linearly to nothing at a combined slip of twice the peak.
    // Mz = -trail * Fy.
    f32 trail(f32 slipRatio, f32 slipAngle, f32 camber = 0.0f) const;

    // Slopes at zero slip (N per unit slip ratio, N per rad).
    f32 longitudinalStiffness(f32 load, const SurfaceInfo& surface) const;
    f32 corneringStiffness(f32 load, const SurfaceInfo& surface) const;

private:
    // sin(C atan(B x - E (B x - atan(B x)))) / x, the pure-slip curve over
    // its argument; finite at x = 0.
    static f32 shapeOverSlip(f32 x, f32 b, f32 c, f32 e);
    static f32 shape(f32 x, f32 b, f32 c, f32 e);
    static f32 solveStiffnessFactor(f32 peakSlip, f32 c, f32 e);

    TyreParameters mParameters;
    f32 mBx = 10.0f;
    f32 mBy = 10.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_TYRE_H
