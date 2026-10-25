#include "check.h"

#include <dynamics/Airfoil.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;

void linearRange()
{
    const Airfoil symmetric = Airfoil::symmetric();
    // Central difference across the linear range: 2 pi per radian, the thin
    // airfoil value, to well under a percent.
    const f32 slope =
        (symmetric.evaluate(4.0f * kDeg).lift - symmetric.evaluate(-4.0f * kDeg).lift) /
        (8.0f * kDeg);
    CHECK_NEAR(slope, 2.0 * kPi, 0.01);
    CHECK_NEAR(symmetric.evaluate(0.0f).lift, 0.0, 1e-6);
    CHECK_NEAR(symmetric.evaluate(6.0f * kDeg).lift, -symmetric.evaluate(-6.0f * kDeg).lift, 1e-5);
    CHECK_NEAR(symmetric.evaluate(6.0f * kDeg).lift, 2.0 * kPi * 6.0 * kDeg, 1e-3);
    CHECK_NEAR(symmetric.evaluate(0.0f).moment, 0.0, 1e-6);

    Airfoil::Parameters p;
    p.liftSlope = 5.5f;
    p.zeroLiftAngle = -3.0f * kDeg;
    p.moment = -0.07f;
    const Airfoil custom(p);
    CHECK_NEAR(custom.evaluate(-3.0f * kDeg).lift, 0.0, 1e-5);
    CHECK_NEAR(custom.evaluate(2.0f * kDeg).lift, 5.5 * 5.0 * kDeg, 1e-4);
    CHECK_NEAR(custom.evaluate(2.0f * kDeg).moment, -0.07, 1e-5);

    const Airfoil cambered = Airfoil::cambered();
    CHECK_NEAR(cambered.evaluate(-2.5f * kDeg).lift, 0.0, 1e-5);
    CHECK(cambered.evaluate(0.0f).lift > 0.2f);

    // Drag: minimum at zero lift, growing as Cd0 + k Cl^2.
    const AirfoilCoefficients zero = symmetric.evaluate(0.0f);
    const AirfoilCoefficients five = symmetric.evaluate(5.0f * kDeg);
    CHECK_NEAR(zero.drag, symmetric.parameters().minDrag, 1e-6);
    CHECK_NEAR(five.drag,
               symmetric.parameters().minDrag +
                   symmetric.parameters().liftDragFactor * five.lift * five.lift,
               1e-5);
    CHECK_NEAR(zero.separation, 0.0, 1e-6);
}

void stall()
{
    const Airfoil foil = Airfoil::cambered();
    const f32 maxLift = foil.parameters().maxLift;
    f32 peak = -1.0f;
    f32 peakAlpha = 0.0f;
    for (f32 a = 0.0f; a <= 40.0f; a += 0.1f)
    {
        const f32 cl = foil.evaluate(a * kDeg).lift;
        if (cl > peak)
        {
            peak = cl;
            peakAlpha = a;
        }
    }
    // Peaks at Cl_max (rounded just below it), at the angle where the linear
    // curve reaches it.
    CHECK(peak <= maxLift + 1e-4f);
    CHECK(peak > maxLift - 0.06f);
    CHECK_NEAR(peakAlpha * kDeg, foil.stallAngle(), 2.0 * kDeg);
    const AirfoilCoefficients after = foil.evaluate(foil.stallAngle() + 8.0f * kDeg);
    const AirfoilCoefficients before = foil.evaluate(foil.stallAngle() - 3.0f * kDeg);
    CHECK(after.lift < 0.7f * peak);
    CHECK(after.drag > 5.0f * before.drag);
    CHECK(after.separation > 0.9f);
    CHECK(before.separation < 0.01f);

    const Airfoil symmetric = Airfoil::symmetric();
    f32 trough = 1.0f;
    for (f32 a = 0.0f; a >= -40.0f; a -= 0.1f)
        trough = fminf(trough, symmetric.evaluate(a * kDeg).lift);
    CHECK(trough >= symmetric.parameters().minLift - 1e-4f);
    CHECK(trough < symmetric.parameters().minLift + 0.06f);
}

// The propeller section stalls gradually: no dip below the flat-plate curve after the peak.
void propellerSection()
{
    const Airfoil foil = Airfoil::propeller();
    CHECK_NEAR(foil.evaluate(4.0f * kDeg).lift, Airfoil::cambered().evaluate(4.0f * kDeg).lift,
               1e-6);
    f32 lowest = 10.0f;
    for (f32 a = foil.stallAngle(); a < 45.0f * kDeg; a += 0.1f * kDeg)
        lowest = fminf(lowest, foil.evaluate(a).lift);
    const f32 plateMinimum = foil.evaluate(45.0f * kDeg).lift * 0.85f;
    CHECK(lowest > plateMinimum);
    const Airfoil sharp = Airfoil::cambered();
    f32 sharpLowest = 10.0f;
    for (f32 a = sharp.stallAngle(); a < 45.0f * kDeg; a += 0.1f * kDeg)
        sharpLowest = fminf(sharpLowest, sharp.evaluate(a).lift);
    CHECK(sharpLowest < 0.7f * sharp.evaluate(45.0f * kDeg).lift);
}

void flatPlate()
{
    const Airfoil foil = Airfoil::cambered();
    const f32 maxDrag = foil.parameters().maxDrag;
    const f32 minDrag = foil.parameters().minDrag;
    // Broadside: no lift, all drag, and the centre of pressure at mid chord
    // pitches it nose down by a quarter of the normal force.
    const AirfoilCoefficients broadside = foil.evaluate(90.0f * kDeg);
    CHECK_NEAR(broadside.lift, 0.0, 1e-4);
    CHECK_NEAR(broadside.drag, maxDrag + minDrag, 1e-4);
    CHECK_NEAR(broadside.moment, -0.25 * maxDrag, 1e-4);
    // 45 degrees: Cl = Cd_max sin cos, Cd = Cd0 + Cd_max sin^2.
    const AirfoilCoefficients diagonal = foil.evaluate(45.0f * kDeg);
    CHECK_NEAR(diagonal.lift, 0.5 * maxDrag, 1e-3);
    CHECK_NEAR(diagonal.drag, minDrag + 0.5 * maxDrag, 1e-3);
    const AirfoilCoefficients reversed = foil.evaluate(kPi);
    CHECK_NEAR(reversed.lift, 0.0, 1e-4);
    CHECK_NEAR(reversed.drag, minDrag, 1e-4);
    // Weathervane: positive alpha pitches nose down, negative nose up.
    CHECK(foil.evaluate(60.0f * kDeg).moment < 0.0f);
    CHECK(foil.evaluate(-60.0f * kDeg).moment > 0.0f);
}

struct Steps
{
    f32 lift = 0.0f;
    f32 drag = 0.0f;
    f32 moment = 0.0f;
    bool finite = true;
    bool positiveDrag = true;
};

Steps largestSteps(const Airfoil& foil, f32 deflection, f32 step)
{
    Steps steps;
    AirfoilCoefficients previous = foil.evaluate(-kPi, deflection, 0.3f);
    for (f32 a = -kPi + step; a <= kPi + 1e-6f; a += step)
    {
        const AirfoilCoefficients c = foil.evaluate(a, deflection, 0.3f);
        steps.finite = steps.finite && isfinite(c.lift) && isfinite(c.drag) && isfinite(c.moment);
        steps.positiveDrag = steps.positiveDrag && c.drag > 0.0f;
        steps.lift = fmaxf(steps.lift, fabsf(c.lift - previous.lift));
        steps.drag = fmaxf(steps.drag, fabsf(c.drag - previous.drag));
        steps.moment = fmaxf(steps.moment, fabsf(c.moment - previous.moment));
        previous = c;
    }
    return steps;
}

// No jumps: halving the sample spacing halves the largest difference, which a jump would not do.
void continuity()
{
    const Airfoil foils[4] = {Airfoil::symmetric(), Airfoil::cambered(), Airfoil::cambered(),
                              Airfoil::propeller()};
    const f32 deflections[4] = {0.0f, 0.0f, 25.0f * kDeg, 0.0f};
    for (u32 f = 0; f < 4; ++f)
    {
        const Airfoil& foil = foils[f];
        const Steps coarse = largestSteps(foil, deflections[f], 0.05f * kDeg);
        const Steps fine = largestSteps(foil, deflections[f], 0.025f * kDeg);
        CHECK(coarse.finite && fine.finite);
        CHECK(coarse.positiveDrag);
        // 2 pi per radian is 0.0055 per step; a flapped section's stall drop
        // is the steepest part, at about four times that.
        CHECK(coarse.lift < 0.03f);
        CHECK(coarse.drag < 0.03f);
        CHECK(coarse.moment < 0.03f);
        CHECK(fine.lift < 0.6f * coarse.lift);
        CHECK(fine.drag < 0.6f * coarse.drag);
        CHECK(fine.moment < 0.6f * coarse.moment);

        const AirfoilCoefficients plus = foil.evaluate(kPi - 1e-4f, deflections[f], 0.3f);
        const AirfoilCoefficients minus = foil.evaluate(-kPi + 1e-4f, deflections[f], 0.3f);
        CHECK_NEAR(plus.lift, minus.lift, 2e-3);
        CHECK_NEAR(plus.drag, minus.drag, 2e-3);
        CHECK_NEAR(plus.moment, minus.moment, 2e-3);
        CHECK_NEAR(foil.evaluate(0.3f + 2.0f * kPi, deflections[f], 0.3f).lift,
                   foil.evaluate(0.3f, deflections[f], 0.3f).lift, 1e-4);
    }

    const AirfoilCoefficients nan = Airfoil::cambered().evaluate(NAN);
    CHECK(nan.lift == 0.0f && nan.drag == 0.0f && nan.moment == 0.0f);
}

void flaps()
{
    // tau(E) = 1 - (theta - sin theta) / pi with cos theta = 2E - 1.
    CHECK_NEAR(flapEffectiveness(0.0f), 0.0, 1e-5);
    CHECK_NEAR(flapEffectiveness(1.0f), 1.0, 1e-5);
    CHECK_NEAR(flapEffectiveness(0.25f), 0.609, 0.002);
    CHECK_NEAR(flapEffectiveness(0.5f), 0.818, 0.002);
    CHECK(flapMomentDerivative(0.25f) < 0.0f);
    CHECK_NEAR(flapMomentDerivative(0.25f), -0.5 * sin(2.0944) * 1.5, 2e-3);
    CHECK_NEAR(flapDeflectionEfficiency(5.0f * kDeg), 1.0, 1e-6);
    CHECK_NEAR(flapDeflectionEfficiency(-35.0f * kDeg), 0.7, 1e-4);
    CHECK_NEAR(flapDeflectionEfficiency(80.0f * kDeg), 0.4, 1e-6);

    const Airfoil foil = Airfoil::symmetric();
    const f32 alpha = 2.0f * kDeg;
    const f32 clean = foil.evaluate(alpha).lift;
    const f32 flapped = foil.evaluate(alpha, 10.0f * kDeg, 0.25f).lift;
    // Small deflection: exactly the thin-airfoil increment a0 tau delta.
    CHECK(flapped > clean);
    CHECK_NEAR(flapped - clean, 2.0 * kPi * flapEffectiveness(0.25f) * 10.0 * kDeg, 1e-3);
    CHECK(foil.evaluate(alpha, -10.0f * kDeg, 0.25f).lift < clean);
    const f32 per10 = (flapped - clean) / 10.0f;
    const f32 per40 = (foil.evaluate(-8.0f * kDeg, 40.0f * kDeg, 0.25f).lift -
                       foil.evaluate(-8.0f * kDeg).lift) /
                      40.0f;
    CHECK(per40 < 0.8f * per10);
    CHECK(foil.evaluate(alpha, 20.0f * kDeg, 0.25f).moment < foil.evaluate(alpha).moment);
    CHECK(foil.evaluate(alpha, 20.0f * kDeg, 0.25f).drag > foil.evaluate(alpha).drag);
    CHECK_NEAR(foil.evaluate(alpha, 20.0f * kDeg, 0.0f).lift, clean, 1e-6);

    f32 cleanPeak = -1.0f;
    f32 cleanPeakAlpha = 0.0f;
    f32 flapPeak = -1.0f;
    f32 flapPeakAlpha = 0.0f;
    for (f32 a = 0.0f; a < 30.0f; a += 0.1f)
    {
        const f32 c0 = foil.evaluate(a * kDeg).lift;
        const f32 c1 = foil.evaluate(a * kDeg, 20.0f * kDeg, 0.25f).lift;
        if (c0 > cleanPeak)
        {
            cleanPeak = c0;
            cleanPeakAlpha = a;
        }
        if (c1 > flapPeak)
        {
            flapPeak = c1;
            flapPeakAlpha = a;
        }
    }
    CHECK(flapPeak > cleanPeak + 0.3f);
    CHECK(flapPeakAlpha < cleanPeakAlpha - 1.0f);
}

void table()
{
    Airfoil::TablePoint points[5];
    const f32 alphas[5] = {-16.0f, -8.0f, 0.0f, 8.0f, 16.0f};
    const f32 lifts[5] = {-1.0f, -0.85f, 0.0f, 0.85f, 1.0f};
    for (u32 i = 0; i < 5; ++i)
    {
        points[i].alpha = alphas[i] * kDeg;
        points[i].lift = lifts[i];
        points[i].drag = 0.01f + 0.01f * fabsf(lifts[i]);
        points[i].moment = -0.01f * lifts[i];
    }
    Airfoil foil = Airfoil::symmetric();
    CHECK(foil.setTable(points, 5, 10.0f * kDeg));
    CHECK(foil.hasTable());

    CHECK_NEAR(foil.evaluate(8.0f * kDeg).lift, 0.85, 1e-5);
    CHECK_NEAR(foil.evaluate(4.0f * kDeg).lift, 0.425, 1e-4);
    CHECK_NEAR(foil.evaluate(12.0f * kDeg).drag, 0.01 + 0.01 * 0.925, 1e-4);
    CHECK_NEAR(foil.evaluate(-4.0f * kDeg).moment, 0.01 * 0.425, 1e-5);
    CHECK_NEAR(foil.stallAngle(), 16.0 * kDeg, 1e-5);

    CHECK_NEAR(foil.evaluate(16.0f * kDeg + 1e-4f).lift, 1.0, 2e-3);
    const f32 maxDrag = foil.parameters().maxDrag;
    CHECK_NEAR(foil.evaluate(60.0f * kDeg).lift, maxDrag * sin(60.0 * kDeg) * cos(60.0 * kDeg),
               1e-3);
    f32 worst = 0.0f;
    f32 previous = foil.evaluate(-kPi).lift;
    for (f32 a = -kPi; a <= kPi; a += 0.05f * kDeg)
    {
        const f32 cl = foil.evaluate(a).lift;
        worst = fmaxf(worst, fabsf(cl - previous));
        previous = cl;
    }
    CHECK(worst < 0.02f);

    const f32 shift = flapEffectiveness(0.25f) * 6.0f * kDeg;
    CHECK_NEAR(foil.evaluate(2.0f * kDeg, 6.0f * kDeg, 0.25f).lift,
               foil.evaluate(2.0f * kDeg + shift).lift, 1e-4);

    Airfoil::TablePoint unordered[2] = {points[3], points[1]};
    CHECK(!foil.setTable(unordered, 2));
    CHECK(!foil.setTable(points, 1));
    Airfoil::TablePoint broken[2] = {points[0], points[1]};
    broken[1].lift = NAN;
    CHECK(!foil.setTable(broken, 2));
    CHECK_NEAR(foil.evaluate(8.0f * kDeg).lift, 0.85, 1e-5);

    foil.clearTable();
    CHECK(!foil.hasTable());
    CHECK_NEAR(foil.evaluate(4.0f * kDeg).lift, 2.0 * kPi * 4.0 * kDeg, 1e-3);
}
}

int main()
{
    linearRange();
    stall();
    propellerSection();
    flatPlate();
    continuity();
    flaps();
    table();
    return TEST_RESULT();
}
