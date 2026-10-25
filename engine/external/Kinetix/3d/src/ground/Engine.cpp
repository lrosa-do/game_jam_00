#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/Engine.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kRpmToRadians = 2.0f * kPi / 60.0f;

f32 positiveOr(f32 value, f32 minimum, f32 fallback)
{
    return isfinite(value) ? Max(value, minimum) : fallback;
}

f32 easeInOut(f32 t)
{
    t = Clamp(t, 0.0f, 1.0f);
    return 0.5f - 0.5f * cosf(kPi * t);
}
} // namespace

void EngineParameters::setTorqueCurve(const f32* rpm, const f32* torque, u32 count)
{
    curvePoints = 0;
    if (!rpm || !torque)
        return;
    if (count > MaxCurvePoints)
    {
        Log::error("EngineParameters: %u curve points, keeping the first %u", count, MaxCurvePoints);
        count = MaxCurvePoints;
    }
    for (u32 i = 0; i < count; ++i)
    {
        if (!isfinite(rpm[i]) || !isfinite(torque[i]))
            continue;
        if (curvePoints > 0 && rpm[i] <= curveRpm[curvePoints - 1])
        {
            Log::error("EngineParameters: torque curve rpm must ascend - point %u dropped", i);
            continue;
        }
        curveRpm[curvePoints] = rpm[i];
        curveTorque[curvePoints] = Max(torque[i], 0.0f);
        ++curvePoints;
    }
}

EngineParameters EngineParameters::gasoline(f32 maxPower, f32 powerRpm, f32 maxTorque, f32 torqueRpm,
                                            f32 idleRpm, f32 limiterRpm)
{
    EngineParameters p;
    p.idleRpm = positiveOr(idleRpm, 100.0f, 800.0f);
    p.stallRpm = 0.5f * p.idleRpm;
    torqueRpm = Max(positiveOr(torqueRpm, 1.0f, 4000.0f), p.idleRpm + 100.0f);
    powerRpm = Max(positiveOr(powerRpm, 1.0f, 6000.0f), torqueRpm + 100.0f);
    p.limiterRpm = Max(positiveOr(limiterRpm, 1.0f, powerRpm), powerRpm);
    const f32 torqueAtPower = Min(positiveOr(maxPower, 0.0f, 0.0f) / (powerRpm * kRpmToRadians),
                                  positiveOr(maxTorque, 0.0f, 0.0f));
    f32 rpm[MaxCurvePoints];
    f32 torque[MaxCurvePoints];
    u32 n = 0;
    for (u32 i = 0; i <= 6; ++i)
    {
        const f32 t = static_cast<f32>(i) / 6.0f;
        rpm[n] = p.idleRpm + (torqueRpm - p.idleRpm) * t;
        torque[n] = maxTorque * (0.65f + 0.35f * easeInOut(t));
        ++n;
    }
    for (u32 i = 1; i <= 5; ++i)
    {
        const f32 t = static_cast<f32>(i) / 5.0f;
        rpm[n] = torqueRpm + (powerRpm - torqueRpm) * t;
        torque[n] = maxTorque + (torqueAtPower - maxTorque) * easeInOut(t);
        ++n;
    }
    const f32 slope = (torqueAtPower - torque[n - 2]) / Max(rpm[n - 1] - rpm[n - 2], 1.0f);
    const f32 end = Max(p.limiterRpm + 500.0f, powerRpm + 500.0f);
    rpm[n] = end;
    torque[n] = Max(torqueAtPower + slope * (end - powerRpm), 0.3f * torqueAtPower);
    ++n;
    rpm[n] = end + 300.0f;
    torque[n] = 0.0f;
    ++n;
    p.setTorqueCurve(rpm, torque, n);
    return p;
}

EngineParameters EngineParameters::diesel(f32 maxPower, f32 powerRpm, f32 maxTorque, f32 torqueRpm,
                                          f32 plateauEndRpm, f32 idleRpm, f32 highIdleRpm)
{
    EngineParameters p;
    p.idleRpm = positiveOr(idleRpm, 100.0f, 600.0f);
    p.stallRpm = 0.55f * p.idleRpm;
    torqueRpm = Max(positiveOr(torqueRpm, 1.0f, 1200.0f), p.idleRpm + 50.0f);
    plateauEndRpm = Max(positiveOr(plateauEndRpm, 1.0f, torqueRpm), torqueRpm);
    powerRpm = Max(positiveOr(powerRpm, 1.0f, 1900.0f), plateauEndRpm + 50.0f);
    highIdleRpm = Max(positiveOr(highIdleRpm, 1.0f, powerRpm + 200.0f), powerRpm + 50.0f);
    // Just short of high idle: the governor's droop alone would make the top speed in each gear
    // load dependent; this cuts fuel at a definite speed while torque remains to reach it.
    p.limiterRpm = highIdleRpm - 0.25f * (highIdleRpm - powerRpm);
    p.limiterBand = 30.0f;
    const f32 torqueAtPower = Min(positiveOr(maxPower, 0.0f, 0.0f) / (powerRpm * kRpmToRadians),
                                  positiveOr(maxTorque, 0.0f, 0.0f));
    f32 rpm[MaxCurvePoints];
    f32 torque[MaxCurvePoints];
    u32 n = 0;
    for (u32 i = 0; i <= 4; ++i)
    {
        const f32 t = static_cast<f32>(i) / 4.0f;
        rpm[n] = p.idleRpm + (torqueRpm - p.idleRpm) * t;
        torque[n] = maxTorque * (0.6f + 0.4f * easeInOut(t));
        ++n;
    }
    if (plateauEndRpm > torqueRpm)
    {
        rpm[n] = plateauEndRpm;
        torque[n] = maxTorque;
        ++n;
    }
    for (u32 i = 1; i <= 4; ++i)
    {
        const f32 t = static_cast<f32>(i) / 4.0f;
        rpm[n] = plateauEndRpm + (powerRpm - plateauEndRpm) * t;
        torque[n] = maxTorque + (torqueAtPower - maxTorque) * easeInOut(t);
        ++n;
    }
    rpm[n] = highIdleRpm;
    torque[n] = 0.0f;
    ++n;
    p.setTorqueCurve(rpm, torque, n);
    return p;
}

Engine::Engine()
{
    setParameters(EngineParameters::gasoline(100000.0f, 6000.0f, 180.0f, 4000.0f, 800.0f, 6500.0f));
}

Engine::Engine(const EngineParameters& parameters)
{
    setParameters(parameters);
}

void Engine::setParameters(const EngineParameters& parameters)
{
    EngineParameters p = parameters;
    p.idleRpm = positiveOr(p.idleRpm, 50.0f, 800.0f);
    p.stallRpm = Clamp(positiveOr(p.stallRpm, 0.0f, 0.5f * p.idleRpm), 0.0f, p.idleRpm);
    p.limiterRpm = Max(positiveOr(p.limiterRpm, 0.0f, 6500.0f), p.idleRpm + 1.0f);
    p.limiterBand = Clamp(positiveOr(p.limiterBand, 0.0f, 100.0f), 0.0f, p.limiterRpm - p.idleRpm);
    p.inertia = positiveOr(p.inertia, 1.0e-3f, 0.15f);
    p.frictionTorque = positiveOr(p.frictionTorque, 0.0f, 0.0f);
    p.frictionPerRpm = positiveOr(p.frictionPerRpm, 0.0f, 0.0f);
    p.frictionPerRpm2 = positiveOr(p.frictionPerRpm2, 0.0f, 0.0f);
    p.idleGain = positiveOr(p.idleGain, 0.0f, 0.0f);
    p.maxIdleThrottle = Clamp(positiveOr(p.maxIdleThrottle, 0.0f, 0.3f), 0.0f, 1.0f);
    p.starterTorque = positiveOr(p.starterTorque, 0.0f, 0.0f);
    p.starterCatchRpm = positiveOr(p.starterCatchRpm, 0.0f, 300.0f);
    if (p.curvePoints == 0)
    {
        Log::error("Engine: no torque curve - the engine makes no torque");
    }
    mParameters = p;
    if (mRunning && mOmega < p.idleRpm * kRpmToRadians)
        mOmega = p.idleRpm * kRpmToRadians;
}

f32 Engine::fullLoadTorque(f32 rpm) const
{
    const EngineParameters& p = mParameters;
    if (p.curvePoints == 0 || !isfinite(rpm))
        return 0.0f;
    if (rpm <= p.curveRpm[0])
        return p.curveTorque[0];
    for (u32 i = 1; i < p.curvePoints; ++i)
    {
        if (rpm <= p.curveRpm[i])
        {
            const f32 t = (rpm - p.curveRpm[i - 1]) / (p.curveRpm[i] - p.curveRpm[i - 1]);
            return p.curveTorque[i - 1] + (p.curveTorque[i] - p.curveTorque[i - 1]) * t;
        }
    }
    return 0.0f;
}

f32 Engine::frictionTorque(f32 rpm) const
{
    const EngineParameters& p = mParameters;
    const f32 speed = isfinite(rpm) ? fabsf(rpm) : 0.0f;
    return p.frictionTorque + p.frictionPerRpm * speed + p.frictionPerRpm2 * speed * speed;
}

f32 Engine::peakPower(f32* atRpm) const
{
    const EngineParameters& p = mParameters;
    f32 best = 0.0f;
    f32 bestRpm = 0.0f;
    if (p.curvePoints > 0)
    {
        const f32 last = p.curveRpm[p.curvePoints - 1];
        for (u32 i = 0; i <= 400; ++i)
        {
            const f32 rpm = last * static_cast<f32>(i) / 400.0f;
            const f32 power = fullLoadTorque(rpm) * rpm * kRpmToRadians;
            if (power > best)
            {
                best = power;
                bestRpm = rpm;
            }
        }
    }
    if (atRpm)
        *atRpm = bestRpm;
    return best;
}

void Engine::setAngularVelocity(f32 omega)
{
    mOmega = isfinite(omega) ? Max(omega, 0.0f) : 0.0f;
}

f32 Engine::rpm() const
{
    return mOmega / kRpmToRadians;
}

void Engine::start()
{
    mRunning = true;
    mLimiterCut = false;
    mOmega = Max(mOmega, mParameters.idleRpm * kRpmToRadians);
}

void Engine::stop()
{
    mRunning = false;
}

f32 Engine::update(f32 throttle, bool starter)
{
    const EngineParameters& p = mParameters;
    throttle = isfinite(throttle) ? Clamp(throttle, 0.0f, 1.0f) : 0.0f;
    const f32 rpmNow = rpm();
    mEffectiveThrottle = 0.0f;
    mCombustionTorque = 0.0f;

    if (mRunning && rpmNow < p.stallRpm)
        mRunning = false;

    f32 starterTorque = 0.0f;
    if (!mRunning)
    {
        if (!starter)
            return 0.0f;
        if (rpmNow >= p.starterCatchRpm && rpmNow >= p.stallRpm)
            mRunning = true;
        else
            return p.starterTorque;
    }
    else if (starter && rpmNow < p.starterCatchRpm)
    {
        starterTorque = p.starterTorque;
    }

    // Soft cut: fuel fades out over the band below the limit, so the engine settles where what
    // is left matches the load instead of hunting.
    f32 limiter = rpmNow < p.limiterRpm ? 1.0f : 0.0f;
    if (p.limiterBand > 0.0f)
        limiter = Clamp((p.limiterRpm - rpmNow) / p.limiterBand, 0.0f, 1.0f);
    mLimiterCut = limiter < 1.0f;

    // At idle the base throttle exactly balances friction; below idle it adds throttle
    // proportionally, above it fades the base out over a quarter of idle so coasting is not fuelled.
    f32 idleThrottle = 0.0f;
    const f32 idleTorque = fullLoadTorque(p.idleRpm) + frictionTorque(p.idleRpm);
    if (idleTorque > 0.0f)
    {
        const f32 base = Min(frictionTorque(p.idleRpm) / idleTorque, p.maxIdleThrottle);
        if (rpmNow <= p.idleRpm)
            idleThrottle = Min(base + p.idleGain * (p.idleRpm - rpmNow) / p.idleRpm, p.maxIdleThrottle);
        else
            idleThrottle = base * Max(1.0f - (rpmNow - p.idleRpm) / (0.25f * p.idleRpm), 0.0f);
    }

    mEffectiveThrottle = Max(throttle, idleThrottle) * limiter;
    const f32 fullLoad = fullLoadTorque(rpmNow);
    mCombustionTorque = fullLoad > 0.0f ? mEffectiveThrottle * (fullLoad + frictionTorque(rpmNow)) : 0.0f;
    return mCombustionTorque + starterTorque;
}

} // namespace kinetix::Physics
