#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/LoadMomentIndicator.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kGravity = 9.81f;
}

bool LoadChart::set(const f32* lengths, u32 lengthCount, const f32* radii, u32 radiusCount, const f32* ratings)
{
    if (!lengths || !radii || !ratings || lengthCount == 0 || radiusCount == 0 || lengthCount > MaxLengths ||
        radiusCount > MaxRadii)
    {
        Log::error("LoadChart: %u lengths x %u radii does not fit (at most %u x %u)", lengthCount, radiusCount,
                   MaxLengths, MaxRadii);
        return false;
    }
    for (u32 i = 1; i < lengthCount; ++i)
        if (!(lengths[i] > lengths[i - 1]))
            return false;
    for (u32 i = 1; i < radiusCount; ++i)
        if (!(radii[i] > radii[i - 1]))
            return false;
    mLengthCount = lengthCount;
    mRadiusCount = radiusCount;
    for (u32 i = 0; i < lengthCount; ++i)
        mLengths[i] = lengths[i];
    for (u32 j = 0; j < radiusCount; ++j)
        mRadii[j] = radii[j];
    for (u32 i = 0; i < lengthCount; ++i)
        for (u32 j = 0; j < radiusCount; ++j)
            mRatings[i][j] = isfinite(ratings[i * radiusCount + j]) ? Math::max(ratings[i * radiusCount + j], 0.0f) : 0.0f;
    return true;
}

f32 LoadChart::ratedAt(u32 lengthIndex, f32 radius) const
{
    if (mRadiusCount == 0 || !isfinite(radius) || radius > mRadii[mRadiusCount - 1])
        return 0.0f;
    if (radius <= mRadii[0])
        return mRatings[lengthIndex][0];
    for (u32 j = 1; j < mRadiusCount; ++j)
        if (radius <= mRadii[j])
        {
            const f32 t = (radius - mRadii[j - 1]) / (mRadii[j] - mRadii[j - 1]);
            return mRatings[lengthIndex][j - 1] + (mRatings[lengthIndex][j] - mRatings[lengthIndex][j - 1]) * t;
        }
    return 0.0f;
}

f32 LoadChart::rated(f32 boomLength, f32 radius) const
{
    if (mLengthCount == 0 || !isfinite(boomLength))
        return 0.0f;
    if (boomLength <= mLengths[0])
        return ratedAt(0, radius);
    if (boomLength >= mLengths[mLengthCount - 1])
        return ratedAt(mLengthCount - 1, radius);
    for (u32 i = 1; i < mLengthCount; ++i)
        if (boomLength <= mLengths[i])
        {
            const f32 t = (boomLength - mLengths[i - 1]) / (mLengths[i] - mLengths[i - 1]);
            const f32 shorter = ratedAt(i - 1, radius);
            const f32 longer = ratedAt(i, radius);
            // Never above the shorter boom's rating where the longer one reaches further.
            return Math::min(shorter + (longer - shorter) * t, Math::max(shorter, longer));
        }
    return 0.0f;
}

void LoadMomentIndicator::setWarning(f32 fraction)
{
    if (isfinite(fraction))
        mWarning = Math::clamp(fraction, 0.0f, 1.0f);
}

void LoadMomentIndicator::setFilterTime(f32 seconds)
{
    if (isfinite(seconds))
        mFilterTime = Math::max(seconds, 0.0f);
}

void LoadMomentIndicator::setHysteresis(f32 fraction)
{
    if (isfinite(fraction))
        mHysteresis = Math::clamp(fraction, 0.0f, 0.5f);
}

void LoadMomentIndicator::update(f32 hookLoad, f32 radius, f32 boomLength, f32 step)
{
    const f32 load = isfinite(hookLoad) ? Math::max(hookLoad, 0.0f) : 0.0f;
    if (!mPrimed || !(mFilterTime > 0.0f) || !(step > 0.0f))
        mLoad = load;
    else
        mLoad += (load - mLoad) * (1.0f - expf(-step / mFilterTime));
    mPrimed = true;
    mRadius = isfinite(radius) ? Math::max(radius, 0.0f) : 0.0f;
    mRated = mChart.rated(boomLength, mRadius);
    mUtilisation = mRated > 0.0f ? mLoad / mRated : (mLoad > 0.0f ? 1.0e6f : 0.0f);
    if (mUtilisation >= 1.0f)
        mCutOut = true;
    else if (mUtilisation < 1.0f - mHysteresis)
        mCutOut = false;
}

f32 LoadMomentIndicator::loadMoment() const
{
    return mLoad * kGravity * mRadius;
}

f32 LoadMomentIndicator::ratedMoment() const
{
    return mRated * kGravity * mRadius;
}

} // namespace kinetix::Physics
