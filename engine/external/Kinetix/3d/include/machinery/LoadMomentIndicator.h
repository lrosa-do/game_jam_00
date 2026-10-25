#ifndef KINETIX_PHYSICS_MACHINERY_LOADMOMENTINDICATOR_H
#define KINETIX_PHYSICS_MACHINERY_LOADMOMENTINDICATOR_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// A crane's load chart: rated load (kg, hook block included) for a boom length and a working
// radius (m, slewing axis to hook), bilinearly interpolated. Nothing is rated beyond a length's
// last radius; between lengths the lower rating of the neighbours is taken.
class LoadChart
{
public:
    static constexpr u32 MaxLengths = 8;
    static constexpr u32 MaxRadii = 24;

    // `ratings` holds lengthCount rows of radiusCount values, kg. Lengths and radii must ascend;
    // false (chart unchanged) otherwise.
    bool set(const f32* lengths, u32 lengthCount, const f32* radii, u32 radiusCount, const f32* ratings);
    // kg; 0 outside the chart.
    f32 rated(f32 boomLength, f32 radius) const;

    u32 lengthCount() const
    {
        return mLengthCount;
    }
    u32 radiusCount() const
    {
        return mRadiusCount;
    }
    f32 length(u32 index) const
    {
        return mLengths[index];
    }
    f32 radius(u32 index) const
    {
        return mRadii[index];
    }
    f32 rating(u32 lengthIndex, u32 radiusIndex) const
    {
        return mRatings[lengthIndex][radiusIndex];
    }

private:
    f32 ratedAt(u32 lengthIndex, f32 radius) const;

    f32 mLengths[MaxLengths] = {};
    f32 mRadii[MaxRadii] = {};
    f32 mRatings[MaxLengths][MaxRadii] = {};
    u32 mLengthCount = 0;
    u32 mRadiusCount = 0;
};

// Load moment indicator (rated capacity limiter): the share of the chart's rating the crane is
// lifting, from the rope-measured hook load, radius and boom length. At `warning` (0.9) it warns;
// at 1.0 it cuts out the motions that raise the load moment (hoisting up, luffing out,
// telescoping out) and leaves the ones that lower it, so the operator can get back inside the chart.
// The load signal is filtered (first order, `filterTime`) so a swinging load reads its weight, and
// the cut-out latches until utilisation is below 1 - hysteresis.
class LoadMomentIndicator
{
public:
    void setChart(const LoadChart& chart)
    {
        mChart = chart;
    }
    const LoadChart& chart() const
    {
        return mChart;
    }
    void setEnabled(bool enabled)
    {
        mEnabled = enabled;
    }
    bool enabled() const
    {
        return mEnabled;
    }
    void setWarning(f32 fraction);
    void setFilterTime(f32 seconds);
    void setHysteresis(f32 fraction);

    // Hook load in kg, radius and boom length in m, step s (the filter's).
    void update(f32 hookLoad, f32 radius, f32 boomLength, f32 step);

    f32 load() const
    {
        return mLoad;
    }
    f32 radius() const
    {
        return mRadius;
    }
    f32 ratedLoad() const
    {
        return mRated;
    }
    // load / rated (> 1 overloaded; large with nothing rated).
    f32 utilisation() const
    {
        return mUtilisation;
    }
    // Load moment load * g * radius, N*m, and the chart's at this radius.
    f32 loadMoment() const;
    f32 ratedMoment() const;
    bool warning() const
    {
        return mEnabled && mUtilisation >= mWarning;
    }
    bool cutOut() const
    {
        return mEnabled && mCutOut;
    }

private:
    LoadChart mChart;
    bool mEnabled = true;
    f32 mWarning = 0.9f;
    f32 mFilterTime = 1.0f;
    f32 mHysteresis = 0.03f;
    bool mCutOut = false;
    bool mPrimed = false;
    f32 mLoad = 0.0f;
    f32 mRadius = 0.0f;
    f32 mRated = 0.0f;
    f32 mUtilisation = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_LOADMOMENTINDICATOR_H
