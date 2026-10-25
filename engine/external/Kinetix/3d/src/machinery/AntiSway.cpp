#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/AntiSway.h"

#include <math.h>

namespace kinetix::Physics
{

void InputShaper::setPeriod(f32 period)
{
    mPeriod = isfinite(period) ? Math::max(period, 0.0f) : 0.0f;
}

f32 InputShaper::pendulumPeriod(f32 ropeLength, f32 gravity)
{
    if (!(ropeLength > 0.0f) || !(gravity > 0.0f))
        return 0.0f;
    return 2.0f * Math::pi<f32>() * sqrtf(ropeLength / gravity);
}

void InputShaper::reset()
{
    mHistory.clear();
    mTime = 0.0f;
}

f32 InputShaper::shape(f32 command, f32 step)
{
    if (!isfinite(command))
        command = 0.0f;
    if (!(step > 0.0f) || !isfinite(step))
        step = 0.0f;
    mTime += step;
    Sample sample;
    sample.time = mTime;
    sample.value = command;
    mHistory.push_back(sample);
    if (!(mPeriod > 0.0f))
    {
        mHistory.clear();
        return command;
    }
    const f32 delay = 0.5f * mPeriod;
    const f32 then = mTime - delay;
    f32 delayed = 0.0f;
    u32 keep = 0;
    for (u32 i = 0; i < mHistory.size(); ++i)
        if (mHistory[i].time <= then + 1.0e-6f)
        {
            delayed = mHistory[i].value;
            keep = i;
        }
    if (keep > 0)
    {
        for (u32 i = keep; i < mHistory.size(); ++i)
            mHistory[i - keep] = mHistory[i];
        mHistory.resize(mHistory.size() - keep);
    }
    return 0.5f * command + 0.5f * delayed;
}

} // namespace kinetix::Physics
