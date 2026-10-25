#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/Airfoil.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kTwoPi = 6.28318530717959f;

// Rounding of the attached curve into its stall limit, in lift-coefficient
// units: the corner is smoothed over +-kStallRounding / a0 of angle and the
// peak sits kStallRounding / 4 below the limit.
constexpr f32 kStallRounding = 0.2f;

// Empirical profile drag of a deflected plain flap, per unit chord fraction.
constexpr f32 kFlapDragFactor = 0.9f;

f32 wrapAngle(f32 alpha)
{
    f32 wrapped = fmodf(alpha + kPi, kTwoPi);
    if (wrapped < 0.0f)
        wrapped += kTwoPi;
    return wrapped - kPi;
}

// Quadratic smooth minimum: exactly min(a, b) once more than k apart, a
// parabola in between, so the linear range keeps the textbook lift slope.
f32 smoothMin(f32 a, f32 b, f32 k)
{
    const f32 h = Max(k - fabsf(a - b), 0.0f) / k;
    return Min(a, b) - h * h * k * 0.25f;
}

f32 smoothMax(f32 a, f32 b, f32 k)
{
    return -smoothMin(-a, -b, k);
}

f32 flapAngle(f32 chordFraction)
{
    const f32 fraction = Clamp(chordFraction, 0.0f, 1.0f);
    return acosf(Clamp(2.0f * fraction - 1.0f, -1.0f, 1.0f));
}

bool finite(f32 value)
{
    return isfinite(value);
}
} // namespace

f32 flapEffectiveness(f32 chordFraction)
{
    const f32 theta = flapAngle(chordFraction);
    return 1.0f - (theta - sinf(theta)) / kPi;
}

f32 flapMomentDerivative(f32 chordFraction)
{
    const f32 theta = flapAngle(chordFraction);
    return -0.5f * sinf(theta) * (1.0f - cosf(theta));
}

f32 flapDeflectionEfficiency(f32 deflection)
{
    const f32 degrees = fabsf(deflection) * (180.0f / kPi);
    if (degrees <= 10.0f)
        return 1.0f;
    return Max(1.0f - 0.6f * (degrees - 10.0f) / 50.0f, 0.4f);
}

Airfoil::Airfoil(const Parameters& parameters)
{
    setParameters(parameters);
}

Airfoil Airfoil::symmetric()
{
    return Airfoil(Parameters());
}

Airfoil Airfoil::cambered()
{
    Parameters parameters;
    parameters.zeroLiftAngle = -2.5f * (kPi / 180.0f);
    parameters.maxLift = 1.55f;
    parameters.minLift = -1.1f;
    parameters.minDrag = 0.0065f;
    parameters.liftDragFactor = 0.005f;
    parameters.moment = -0.05f;
    return Airfoil(parameters);
}

Airfoil Airfoil::propeller()
{
    Parameters parameters = cambered().parameters();
    parameters.stallWidth = 0.35f;
    return Airfoil(parameters);
}

void Airfoil::setParameters(const Parameters& parameters)
{
    // Clamped rather than refused: a section that silently keeps its old values
    // is harder to debug than one with the closest legal ones.
    mParameters = parameters;
    mParameters.liftSlope = Max(parameters.liftSlope, 0.1f);
    mParameters.maxLift = Max(parameters.maxLift, 0.1f);
    mParameters.minLift = Min(parameters.minLift, -0.1f);
    mParameters.stallWidth = Clamp(parameters.stallWidth, 0.01f, 1.0f);
    mParameters.minDrag = Max(parameters.minDrag, 0.0f);
    mParameters.liftDragFactor = Max(parameters.liftDragFactor, 0.0f);
    mParameters.maxDrag = Max(parameters.maxDrag, 0.0f);
    mParameters.flapStallFraction = Clamp(parameters.flapStallFraction, 0.0f, 1.0f);
}

bool Airfoil::setTable(const TablePoint* points, u32 count, f32 blendWidth)
{
    if (!points || count < 2)
    {
        Log::error("Airfoil: a table needs at least two points");
        return false;
    }
    for (u32 i = 0; i < count; ++i)
    {
        const TablePoint& point = points[i];
        if (!finite(point.alpha) || !finite(point.lift) || !finite(point.drag) ||
            !finite(point.moment))
        {
            Log::error("Airfoil: table point %u is not finite", i);
            return false;
        }
        if (i > 0 && !(point.alpha > points[i - 1].alpha))
        {
            Log::error("Airfoil: table alpha must strictly increase (point %u)", i);
            return false;
        }
    }
    mTable.clear();
    mTable.reserve(count);
    for (u32 i = 0; i < count; ++i)
        mTable.push_back(points[i]);
    mTableBlend = Clamp(blendWidth, 0.01f, kPi);

    usize highest = 0;
    usize lowest = 0;
    for (usize i = 1; i < mTable.size(); ++i)
    {
        if (mTable[i].lift > mTable[highest].lift)
            highest = i;
        if (mTable[i].lift < mTable[lowest].lift)
            lowest = i;
    }
    mTableStall = mTable[highest].alpha;
    mTableNegativeStall = mTable[lowest].alpha;
    return true;
}

void Airfoil::clearTable()
{
    mTable.clear();
}

f32 Airfoil::stallAngle() const
{
    if (!mTable.empty())
        return mTableStall;
    return mParameters.zeroLiftAngle + mParameters.maxLift / mParameters.liftSlope;
}

f32 Airfoil::negativeStallAngle() const
{
    if (!mTable.empty())
        return mTableNegativeStall;
    return mParameters.zeroLiftAngle + mParameters.minLift / mParameters.liftSlope;
}

AirfoilCoefficients Airfoil::flatPlate(f32 alpha) const
{
    // A plate in separated flow carries a force normal to itself only. Its centre
    // of pressure walks aft: quarter chord at small angles, mid-chord broadside,
    // quarter chord from the trailing edge once the flow arrives from behind.
    const f32 normal = mParameters.maxDrag * sinf(alpha);
    const f32 centreOfPressure = 0.25f + 0.5f * fabsf(alpha) / kPi;

    AirfoilCoefficients plate;
    plate.lift = normal * cosf(alpha);
    plate.drag = mParameters.minDrag + normal * sinf(alpha);
    plate.moment = -normal * (centreOfPressure - 0.25f);
    plate.separation = 1.0f;
    return plate;
}

AirfoilCoefficients Airfoil::evaluateParametric(f32 alpha, f32 flapShift, f32 liftShift) const
{
    const Parameters& p = mParameters;
    const f32 zeroLift = p.zeroLiftAngle - flapShift;

    f32 maxLift = p.maxLift + p.flapStallFraction * liftShift;
    f32 minLift = p.minLift + p.flapStallFraction * liftShift;
    // A large flap deflection must not push one stall limit past the other:
    // the section would have no attached range left at all.
    maxLift = Max(maxLift, 0.1f);
    minLift = Min(minLift, -0.1f);

    const f32 linear = p.liftSlope * (alpha - zeroLift);
    const f32 attached =
        smoothMax(smoothMin(linear, maxLift, kStallRounding), minLift, kStallRounding);

    const f32 stall = zeroLift + maxLift / p.liftSlope;
    const f32 negativeStall = zeroLift + minLift / p.liftSlope;
    f32 separation = Max(Math::smoothstep(stall, stall + p.stallWidth, alpha),
                         Math::smoothstep(negativeStall, negativeStall - p.stallWidth, alpha));
    // Flow from behind is separated whatever the parameters say, so both ends of
    // the circle agree at +-180 degrees even for a stall angle pushed far by a flap.
    separation = Max(separation, Math::smoothstep(0.5f * kPi, 0.6667f * kPi, fabsf(alpha)));

    AirfoilCoefficients result;
    result.lift = attached;
    result.drag = p.minDrag + p.liftDragFactor * attached * attached;
    result.moment = p.moment;
    result.separation = separation;
    if (separation > 0.0f)
    {
        const AirfoilCoefficients plate = flatPlate(alpha);
        result.lift = Math::mix(result.lift, plate.lift, separation);
        result.drag = Math::mix(result.drag, plate.drag, separation);
        result.moment = Math::mix(result.moment, plate.moment, separation);
    }
    return result;
}

AirfoilCoefficients Airfoil::evaluateTable(f32 alpha, f32 flapShift) const
{
    const f32 lookup = wrapAngle(alpha + flapShift);
    const TablePoint& first = mTable[0];
    const TablePoint& last = mTable.back();

    AirfoilCoefficients edge;
    f32 blend = 0.0f;
    if (lookup <= first.alpha || lookup >= last.alpha)
    {
        const TablePoint& end = lookup <= first.alpha ? first : last;
        edge.lift = end.lift;
        edge.drag = end.drag;
        edge.moment = end.moment;
        blend = lookup <= first.alpha
                    ? Math::smoothstep(first.alpha, first.alpha - mTableBlend, lookup)
                    : Math::smoothstep(last.alpha, last.alpha + mTableBlend, lookup);
    }
    else
    {
        usize lo = 0;
        usize hi = mTable.size() - 1;
        while (hi - lo > 1)
        {
            const usize mid = (lo + hi) / 2;
            if (mTable[mid].alpha <= lookup)
                lo = mid;
            else
                hi = mid;
        }
        const TablePoint& a = mTable[lo];
        const TablePoint& b = mTable[hi];
        const f32 t = (lookup - a.alpha) / (b.alpha - a.alpha);
        edge.lift = Math::mix(a.lift, b.lift, t);
        edge.drag = Math::mix(a.drag, b.drag, t);
        edge.moment = Math::mix(a.moment, b.moment, t);
    }

    // Inside the table, "stalled" means past either lift peak.
    edge.separation = (lookup > mTableStall || lookup < mTableNegativeStall) ? 1.0f : 0.0f;
    if (blend <= 0.0f)
        return edge;

    const AirfoilCoefficients plate = flatPlate(lookup);
    AirfoilCoefficients result;
    result.lift = Math::mix(edge.lift, plate.lift, blend);
    result.drag = Math::mix(edge.drag, plate.drag, blend);
    result.moment = Math::mix(edge.moment, plate.moment, blend);
    result.separation = Max(edge.separation, blend);
    return result;
}

AirfoilCoefficients Airfoil::evaluate(f32 alpha, f32 flapDeflection, f32 flapChordFraction) const
{
    if (!finite(alpha))
        return AirfoilCoefficients();
    const f32 wrapped = wrapAngle(alpha);

    f32 deflection = finite(flapDeflection) ? flapDeflection : 0.0f;
    const f32 chordFraction = Clamp(flapChordFraction, 0.0f, 1.0f);
    if (chordFraction <= 0.0f)
        deflection = 0.0f;
    if (deflection == 0.0f)
        return mTable.empty() ? evaluateParametric(wrapped, 0.0f, 0.0f) : evaluateTable(wrapped, 0.0f);

    // The flap acts on attached flow as an equivalent shift in angle of attack.
    const f32 efficiency = flapDeflectionEfficiency(deflection);
    const f32 shift = flapEffectiveness(chordFraction) * efficiency * deflection;

    AirfoilCoefficients result = mTable.empty()
                                     ? evaluateParametric(wrapped, shift, mParameters.liftSlope * shift)
                                     : evaluateTable(wrapped, shift);
    const f32 sine = sinf(deflection);
    result.moment += flapMomentDerivative(chordFraction) * efficiency * deflection;
    result.drag += kFlapDragFactor * chordFraction * sine * sine;
    return result;
}

} // namespace kinetix::Physics
