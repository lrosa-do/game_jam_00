#ifndef KINETIX_PHYSICS_ROTORCRAFT_ROTOR_H
#define KINETIX_PHYSICS_ROTORCRAFT_ROTOR_H

#include "dynamics/Airfoil.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Rotor operating point, everything in the airframe's body space.
struct RotorOperatingPoint
{
    // Rotor speed relative to the airframe, rad/s, >= 0.
    f32 angularSpeed = 0.0f;
    // Hub velocity through the air (hub velocity minus wind), body space.
    Math::vec3 velocity{0.0f};
    // Airframe angular velocity, body space: the source of pitch and roll damping.
    Math::vec3 angularVelocity{0.0f};
    f32 density = 1.225f;
    // Hub height above the ground, m. <= 0: no ground in reach (out of ground effect).
    f32 groundDistance = -1.0f;
};

// Rotor loads on the airframe for one operating point. Forces in N, moments in N m, angles in rad.
struct RotorLoads
{
    // Resultant aerodynamic force, body space, at the hub; tilts with the tip-path plane (cyclic acts through it).
    Math::vec3 force{0.0f};
    // Hub moment, body space: what the hinge offset and flap spring pass to the shaft when the
    // tip-path plane tilts. Shaft torque is separate (`torque`).
    Math::vec3 moment{0.0f};
    // Components of `force`: along the shaft (thrust), in the disc plane aft (H force) and to the
    // body's right (Y force).
    f32 thrust = 0.0f;
    f32 hForce = 0.0f;
    f32 yForce = 0.0f;
    // About the shaft, positive resisting rotation; negative when the air drives the rotor (autorotation).
    f32 torque = 0.0f;
    f32 power = 0.0f;
    // Disc-area mean induced velocity, m/s, positive down through the disc (ground effect included).
    f32 inducedVelocity = 0.0f;
    // lambda = (climb component + induced) / (omega R), positive down; mu = in-plane airspeed / (omega R).
    f32 inflowRatio = 0.0f;
    f32 advanceRatio = 0.0f;
    f32 thrustCoefficient = 0.0f;
    // Flapping: beta(psi) = a0 - a1s cos(psi) - b1s sin(psi), psi from aft in the direction of
    // rotation. a1s > 0: tip-path plane tilted aft (blowback); b1s > 0: towards the advancing side (psi = 90 deg).
    f32 coning = 0.0f;
    f32 longitudinalFlapping = 0.0f;
    f32 lateralFlapping = 0.0f;
    f32 discTiltForward = 0.0f;
    f32 discTiltRight = 0.0f;
    f32 wakeSkew = 0.0f;
    // Cheeseman-Bennett thrust ratio at constant power, 1 out of ground effect.
    f32 groundEffectFactor = 1.0f;
    // Descent rate along the shaft over the hover induced velocity sqrt(T / (2 rho A)) at this thrust.
    f32 descentRatio = 0.0f;
    // Inside the vortex ring state boundary (see Rotor).
    bool vortexRing = false;
    f32 stalledFraction = 0.0f;
};

// Helicopter rotor by blade element theory on a polar grid, with momentum-theory inflow and
// quasi-steady flapping.
//
// Grid: radialStations annuli from the root cutout to the tip, azimuthStations azimuths. Azimuth
// psi is from aft (projection of -forward on the disc) in the direction of rotation, so psi = 90
// deg is always the advancing side (right for rotation = +1, anticlockwise seen from above). All
// formulas use that frame, so both rotation directions are the same code.
//
// Blade element at (r, psi), flap angle beta(psi): tangential speed U_T = (Omega + yaw rate) r +
// V.e_t and speed through the disc
//     U_P = V.n + (w x r e_r).n + v_i cos(beta) + (r - e) dbeta/dt,
// n the flapped blade's normal, V the hub's velocity through the air, w the airframe rate (the
// source of pitch and roll damping). Blade pitch theta0 + twist (r / R - 0.75) - A1 cos(psi) -
// B1 sin(psi); the airfoil lift and drag at theta - atan2(U_P, U_T) sum to the hub force, shaft
// torque and flap moment about the hinge. Reverse flow on the retreating side is an angle near
// 180 deg (the airfoil covers the full circle).
//
// Inflow: per-annulus induced velocity v(r), uniform in azimuth, from Glauert's momentum balance
// with Prandtl tip loss F:
//     dT = 4 pi rho F r v sqrt(V_xy^2 + (V_z + v)^2) dr
// (propeller BEMT at zero airspeed, Glauert's high-speed rotor fast). On top, Pitt-Peters
// first-harmonic skew v0 kx (r / R) cos(psi - psi_wake), kx = (15 pi / 32) tan(chi / 2), chi the
// wake skew angle. In steep descent momentum theory has no physical solution (vortex ring,
// turbulent wake); there the mass flow gets an empirical extra term (0.37 v)^2 blended in with
// descent rate and faded with forward speed, keeping the solution unique and continuous into the
// windmill brake state (ideal vertical autorotation at about 1.65 v_h vs 1.7 to 1.8 v_h
// measured). The vortex ring flag is raised for descent 0.5 to 2 v_h with < 0.9 v_h in-plane.
//
// Ground effect: Cheeseman-Bennett T / T_inf = 1 / (1 - x) at constant power, x = (R / 4z)^2 /
// (1 + (V_xy / v_i)^2), reproduced by scaling the induced velocity at a given thrust by
// (1 - x)^(3/2) (exact for an ideal rotor, P ~ T^1.5). z is clamped at 0.4 R, the tested range.
//
// Flapping: rigid blade hinged at offset e with flap spring K_beta; I_beta and S_beta its second
// and first mass moments about the hinge (uniform blade of bladeMass):
//     I_beta beta'' + ((I_beta + e S_beta) Omega^2 + K_beta) beta =
//         M_aero - 2 Omega (I_beta + e S_beta) (w x e_t).z
// (gyroscopic forcing from airframe pitch and roll rate included), solved for the steady first
// harmonic beta = a0 + b1c cos + b1s sin by harmonic balance against the numerically integrated
// aerodynamic moment: Newton iterations with the classical hover Jacobian (Lock-number flap
// damping couples the two cyclic harmonics). Hub moment (N_b / 2) (e S_beta Omega^2 + K_beta)
// times the tip-path-plane tilt, plus the moment of the tilted thrust about the airframe. Under
// advance() the flapping approaches the quasi-steady solution with time constant 16 / (gamma
// Omega) (regressing flap mode, gamma the Lock number), updated exactly so it is stable at any
// step. Per-blade dynamic flapping would need a step well under a tenth of a revolution.
//
// Not modelled: lead-lag, blade elasticity, pitch-flap coupling (delta-3), compressibility,
// dynamic stall, dynamic inflow lag, blade weight against coning, angular acceleration terms of
// the flap equation. The gyroscopic moment of the spinning disc is not applied to the airframe:
// a flapping rotor transmits only the hub moment above.
class Rotor
{
public:
    struct Description
    {
        f32 radius = 5.0f;
        // Where the aerodynamic blade starts; at or outside the hinge.
        f32 rootCutout = 0.75f;
        u32 bladeCount = 3;
        // At the root cutout and at the tip, linear between.
        f32 rootChord = 0.24f;
        f32 tipChord = 0.24f;
        // Linear twist from root to tip, rad: negative is washout. Collective is the pitch at 0.75 R.
        f32 twist = -0.14f;
        // Flapping hinge offset from the shaft, m, and flap spring per blade at the hinge, N m / rad.
        // A teetering or gimballed rotor is offset 0, spring 0.
        f32 hingeOffset = 0.2f;
        f32 flapSpring = 0.0f;
        // Mass of one blade, uniform from hinge to tip; sets the flap inertia (Lock number) and polar inertia.
        f32 bladeMass = 25.0f;
        // Hub, shaft and whatever else turns with the rotor, kg m^2.
        f32 hubInertia = 15.0f;
        // Hub position, body space; shaft axis (direction of positive thrust); airframe forward,
        // which sets azimuth zero and the meaning of longitudinal and lateral cyclic.
        Math::vec3 position{0.0f, 1.5f, 0.0f};
        Math::vec3 shaftAxis{0.0f, 1.0f, 0.0f};
        Math::vec3 forward{0.0f, 0.0f, 1.0f};
        // +1 turns right-handed about the shaft axis (anticlockwise from above for a main rotor), -1 the other way.
        f32 rotation = 1.0f;
        // Swashplate phase, rad: cyclic is applied this much earlier in azimuth than its nominal
        // 90 degree lead, cancelling the phase shift a hinge offset or spring introduces.
        f32 cyclicPhase = 0.0f;
        u32 radialStations = 10;  // 2..24
        u32 azimuthStations = 12; // 4..32
        bool tipLoss = true;
        bool groundEffect = true;
        // First-order tip-path-plane lag under advance(); off makes advance() quasi-steady like solve().
        bool flapLag = true;
    };

    static constexpr u32 MaxRadialStations = 24;
    static constexpr u32 MaxAzimuthStations = 32;

    Rotor();
    Rotor(const Description& description, const Airfoil& airfoil);

    void setDescription(const Description& description);
    const Description& description() const
    {
        return mDescription;
    }
    void setAirfoil(const Airfoil& airfoil)
    {
        mAirfoil = airfoil;
    }
    const Airfoil& airfoil() const
    {
        return mAirfoil;
    }

    // Swashplate, rad: collective at 0.75 R; positive longitudinal cyclic tilts the disc forward,
    // positive lateral cyclic towards the body's right, whichever way the rotor turns.
    void setControls(f32 collective, f32 longitudinalCyclic, f32 lateralCyclic);
    f32 collective() const
    {
        return mCollective;
    }
    f32 longitudinalCyclic() const
    {
        return mLongitudinalCyclic;
    }
    f32 lateralCyclic() const
    {
        return mLateralCyclic;
    }

    // Converges inflow and flapping to the steady state at this operating point (quasi-steady, no
    // lag) and keeps it as the rotor's state. For tests, trim and performance.
    const RotorLoads& solve(const RotorOperatingPoint& point);
    // One simulation step: a couple of Newton sweeps on the inflow and the lagged flapping from
    // the previous state, then the loads at the new state.
    const RotorLoads& advance(const RotorOperatingPoint& point, f32 step);
    // The loads at the current state, nothing updated.
    RotorLoads evaluate(const RotorOperatingPoint& point) const;
    // Forgets the inflow and flapping (back to a still disc).
    void reset();

    const RotorLoads& loads() const
    {
        return mLoads;
    }

    f32 discArea() const
    {
        return 3.14159265358979f * mDescription.radius * mDescription.radius;
    }
    // Blade area over disc area, with the mean chord over the blade span.
    f32 solidity() const;
    // gamma = rho a c R^4 / I_beta, c the chord at 0.75 R, a the airfoil's lift slope.
    f32 lockNumber(f32 density) const;
    f32 flapFrequencyRatio(f32 angularSpeed) const;
    f32 polarInertia() const;
    f32 flapInertia() const
    {
        return mFlapInertia;
    }
    // Body-space hub frame: shaft, aft (azimuth 0) and psi = 90 deg.
    const Math::vec3& shaftAxis() const
    {
        return mShaft;
    }
    const Math::vec3& aftAxis() const
    {
        return mAft;
    }
    const Math::vec3& advancingAxis() const
    {
        return mAdvancing;
    }

private:
    struct RadialStation
    {
        f32 radius;
        f32 width;
        f32 chord;
        f32 twist;
    };
    struct AzimuthStation
    {
        f32 cosine;
        f32 sine;
        Math::vec3 radial;     // e_r, body space
        Math::vec3 tangential; // e_t, the blade's direction of motion
    };
    struct FlapState
    {
        f32 coning = 0.0f;
        f32 cosine = 0.0f; // beta_1c
        f32 sine = 0.0f;   // beta_1s
    };
    struct Context;
    struct Sweep;

    void rebuild();
    bool prepare(const RotorOperatingPoint& point, const f32* induced, f32 thrustEstimate,
                 Context& context) const;
    void sweep(const Context& context, const f32* induced, const FlapState& flap, Sweep& out) const;
    f32 updateInflow(const Context& context, const Sweep& sweep, f32* induced) const;
    f32 updateFlapping(const Context& context, const Sweep& sweep, f32 relaxation,
                       FlapState& flap) const;
    RotorLoads finish(const Context& context, const Sweep& sweep, const f32* induced,
                      const FlapState& flap) const;

    Description mDescription;
    Airfoil mAirfoil;
    RadialStation mRadial[MaxRadialStations];
    AzimuthStation mAzimuth[MaxAzimuthStations];
    u32 mRadialCount = 0;
    u32 mAzimuthCount = 0;
    Math::vec3 mShaft{0.0f, 1.0f, 0.0f};
    Math::vec3 mAft{0.0f, 0.0f, -1.0f};
    Math::vec3 mAdvancing{-1.0f, 0.0f, 0.0f};
    f32 mFlapInertia = 1.0f;     // I_beta about the hinge
    f32 mFlapFirstMoment = 1.0f; // S_beta about the hinge
    f32 mFlapDampingIntegral = 0.0f; // sum of c r (r - e)^2 dr
    f32 mRootChordAt75 = 0.24f;

    f32 mCollective = 0.0f;
    f32 mLongitudinalCyclic = 0.0f;
    f32 mLateralCyclic = 0.0f;

    f32 mInduced[MaxRadialStations];
    FlapState mFlap;
    RotorLoads mLoads;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ROTORCRAFT_ROTOR_H
