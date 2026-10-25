#ifndef KINETIX_PHYSICS_DYNAMICS_AIRFOIL_H
#define KINETIX_PHYSICS_DYNAMICS_AIRFOIL_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

// Section coefficients at one angle of attack: lift and drag per unit
// dynamic pressure and chord, and the pitching moment about the quarter
// chord, positive nose up.
struct AirfoilCoefficients
{
    f32 lift = 0.0f;
    f32 drag = 0.0f;
    f32 moment = 0.0f;
    // Separation: 0 fully attached, 1 fully flat-plate; above 0.5 counts as stalled.
    f32 separation = 0.0f;
};

// Thin-airfoil flap theory for a plain flap hinged at (1 - chordFraction) of
// the chord: fraction of a deflection that acts as angle of attack,
// tau = 1 - (theta_f - sin(theta_f)) / pi, cos(theta_f) = 2 * chordFraction - 1
// (0.61 for a quarter-chord flap).
f32 flapEffectiveness(f32 chordFraction);

// Change in quarter-chord moment per radian of flap,
// -sin(theta_f) * (1 - cos(theta_f)) / 2 (flap down pitches nose down).
f32 flapMomentDerivative(f32 chordFraction);

// Empirical factor on tau for flow separating over a deflected flap: 1 up to
// 10 degrees, falling linearly to 0.4 at 60 degrees and held there.
f32 flapDeflectionEfficiency(f32 deflection);

// One airfoil section's lift, drag and moment over the full -180..+180 degrees
// of angle of attack, continuous, so a tumbling surface still gives sane forces.
//  - attached: Cl = a0 * (alpha - alpha0) (a0 = 2 pi per radian by default),
//    rounded off just below maxLift / minLift; Cd = Cd0 + k Cl^2; constant
//    quarter-chord moment.
//  - separated: flat plate, Cl = Cd_max sin(alpha) cos(alpha),
//    Cd = Cd0 + Cd_max sin^2(alpha), centre of pressure moving from the quarter
//    chord through mid-chord at 90 degrees to three-quarter chord for reversed flow.
//  - stall: smoothstep blend over stallWidth from where the attached curve
//    reaches maxLift (or minLift).
//
// A table (alpha, Cl, Cd, Cm) replaces the attached part: linear inside its
// range, blended into the flat plate over tableBlendWidth past either end.
//
// Flap deflection (positive trailing edge down) shifts the curve by
// tau * eta * delta, adds the thin-airfoil moment and an empirical profile drag
// 0.9 * chordFraction * sin^2(delta). The stall limits rise by only
// flapStallFraction of the lift the flap adds, so a flapped section stalls earlier.
class Airfoil
{
public:
    struct Parameters
    {
        f32 liftSlope = 2.0f * 3.14159265f;  // per radian
        f32 zeroLiftAngle = 0.0f;            // radians
        f32 maxLift = 1.5f;
        f32 minLift = -1.5f;
        f32 stallWidth = 0.14f;              // radians, ~8 degrees
        f32 minDrag = 0.008f;
        f32 liftDragFactor = 0.006f;         // k in Cd0 + k Cl^2
        f32 maxDrag = 1.8f;                  // flat-plate normal force at 90 degrees
        f32 moment = 0.0f;                   // attached-flow Cm about c/4
        f32 flapStallFraction = 0.7f;
    };

    struct TablePoint
    {
        f32 alpha = 0.0f; // radians, strictly increasing through the table
        f32 lift = 0.0f;
        f32 drag = 0.0f;
        f32 moment = 0.0f;
    };

    Airfoil() = default;
    explicit Airfoil(const Parameters& parameters);

    // Symmetric (thin airfoil defaults) and a typical cambered light-aircraft
    // section: zero lift at -2.5 degrees, Cl_max 1.55, Cm -0.05.
    static Airfoil symmetric();
    static Airfoil cambered();
    // Cambered section with the stall spread over 20 degrees instead of 8: inboard
    // blade stations of a static fixed-pitch propeller stall gradually, and a 2D
    // post-stall dip would make thrust ripple with airspeed.
    static Airfoil propeller();

    void setParameters(const Parameters& parameters);
    const Parameters& parameters() const
    {
        return mParameters;
    }

    // Refused, leaving the table unchanged, unless there are at least two points,
    // all values are finite and alpha strictly increases.
    bool setTable(const TablePoint* points, u32 count, f32 blendWidth = 0.17f);
    void clearTable();
    bool hasTable() const
    {
        return !mTable.empty();
    }

    // Any finite alpha; it is wrapped into (-pi, pi]. flapChordFraction is
    // the control surface's share of the chord (0 for none).
    AirfoilCoefficients evaluate(f32 alpha, f32 flapDeflection = 0.0f,
                                 f32 flapChordFraction = 0.0f) const;

    // Where the clean attached curve reaches maxLift / minLift.
    f32 stallAngle() const;
    f32 negativeStallAngle() const;

private:
    AirfoilCoefficients evaluateParametric(f32 alpha, f32 flapShift, f32 liftShift) const;
    AirfoilCoefficients evaluateTable(f32 alpha, f32 flapShift) const;
    AirfoilCoefficients flatPlate(f32 alpha) const;

    Parameters mParameters;
    ct::Vector<TablePoint> mTable;
    f32 mTableBlend = 0.17f;
    f32 mTableStall = 0.0f;
    f32 mTableNegativeStall = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_DYNAMICS_AIRFOIL_H
