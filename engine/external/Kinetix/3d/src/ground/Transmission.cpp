#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "ground/Transmission.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kRadiansToRpm = 60.0f / (2.0f * 3.14159265358979f);

f32 positiveOr(f32 value, f32 minimum, f32 fallback)
{
    return isfinite(value) ? Max(value, minimum) : fallback;
}

f32 finiteOr(f32 value, f32 fallback)
{
    return isfinite(value) ? value : fallback;
}
} // namespace

Clutch::Clutch(const ClutchParameters& parameters)
{
    setParameters(parameters);
}

void Clutch::setParameters(const ClutchParameters& parameters)
{
    mParameters.maxTorque = positiveOr(parameters.maxTorque, 0.0f, 300.0f);
}

void Clutch::setEngagement(f32 engagement)
{
    mEngagement = isfinite(engagement) ? Clamp(engagement, 0.0f, 1.0f) : 0.0f;
}

void Clutch::setSolved(f32 torque, f32 slip, bool locked)
{
    mTorque = finiteOr(torque, 0.0f);
    mSlip = finiteOr(slip, 0.0f);
    mLocked = locked;
}

void GearboxParameters::setRatios(const f32* forward, u32 count)
{
    if (!forward)
        return;
    if (count > MaxGears)
    {
        Log::error("GearboxParameters: %u gears, keeping the first %u", count, MaxGears);
        count = MaxGears;
    }
    gearCount = count;
    for (u32 i = 0; i < count; ++i)
        ratios[i] = forward[i];
}

Gearbox::Gearbox(const GearboxParameters& parameters)
{
    setParameters(parameters);
}

void Gearbox::setParameters(const GearboxParameters& parameters)
{
    GearboxParameters p = parameters;
    p.gearCount = Min(p.gearCount, GearboxParameters::MaxGears);
    for (u32 i = 0; i < p.gearCount; ++i)
    {
        if (!(p.ratios[i] > 0.0f) || !isfinite(p.ratios[i]))
        {
            Log::error("Gearbox: gear %u ratio must be positive - set to 1", i + 1);
            p.ratios[i] = 1.0f;
        }
    }
    p.reverseRatio = positiveOr(fabsf(finiteOr(p.reverseRatio, 3.5f)), 1.0e-3f, 3.5f);
    p.finalDrive = positiveOr(p.finalDrive, 1.0e-3f, 1.0f);
    p.efficiency = Clamp(finiteOr(p.efficiency, 1.0f), 0.0f, 1.0f);
    p.dragTorque = positiveOr(p.dragTorque, 0.0f, 0.0f);
    p.inputInertia = positiveOr(p.inputInertia, 1.0e-4f, 0.02f);
    p.shiftTime = positiveOr(p.shiftTime, 0.0f, 0.25f);
    p.minShiftInterval = positiveOr(p.minShiftInterval, 0.0f, 1.0f);
    mParameters = p;
    if (mGear > static_cast<s32>(p.gearCount))
        mGear = static_cast<s32>(p.gearCount);
    if (mTarget > static_cast<s32>(p.gearCount))
        mTarget = static_cast<s32>(p.gearCount);
}

void Gearbox::setMode(GearboxMode mode)
{
    mMode = mode;
}

f32 Gearbox::ratioOf(s32 gear) const
{
    if (gear < 0)
        return -mParameters.reverseRatio * mParameters.finalDrive;
    if (gear == 0 || gear > static_cast<s32>(mParameters.gearCount))
        return 0.0f;
    return mParameters.ratios[gear - 1] * mParameters.finalDrive;
}

void Gearbox::beginShift(s32 target)
{
    mTarget = target;
    if (target == mGear && !shifting())
        return;
    if (mParameters.shiftTime <= 0.0f)
    {
        mGear = target;
        mShiftTimer = 0.0f;
        mSinceShift = 0.0f;
        ++mShiftCount;
        return;
    }
    mGear = 0;
    mShiftTimer = mParameters.shiftTime;
}

s32 Gearbox::automaticGearFor(f32 outputSpeed, f32 throttle) const
{
    const GearboxParameters& p = mParameters;
    const f32 up = p.upshiftRpmLight + (p.upshiftRpmFull - p.upshiftRpmLight) * throttle;
    for (u32 g = 1; g <= p.gearCount; ++g)
    {
        const f32 rpm = fabsf(outputSpeed) * fabsf(ratioOf(static_cast<s32>(g))) * kRadiansToRpm;
        if (rpm <= 0.9f * up)
            return static_cast<s32>(g);
    }
    return static_cast<s32>(p.gearCount);
}

void Gearbox::selectGear(s32 gear)
{
    const s32 top = static_cast<s32>(mParameters.gearCount);
    gear = Clamp(gear, -1, top);
    if (mMode == GearboxMode::Automatic && gear > 0)
    {
        if (mTarget > 0)
            return;
        gear = automaticGearFor(mOutputOmega, 0.0f);
    }
    if (gear == mTarget)
        return;
    beginShift(gear);
}

void Gearbox::engage(s32 gear)
{
    gear = Clamp(gear, -1, static_cast<s32>(mParameters.gearCount));
    mGear = gear;
    mTarget = gear;
    mShiftTimer = 0.0f;
    mSinceShift = 0.0f;
}

void Gearbox::shiftUp()
{
    if (mMode == GearboxMode::Automatic)
        return;
    const s32 top = static_cast<s32>(mParameters.gearCount);
    if (mTarget < top)
        beginShift(mTarget + 1);
}

void Gearbox::shiftDown()
{
    if (mMode == GearboxMode::Automatic)
        return;
    if (mTarget > -1)
        beginShift(mTarget - 1);
}

void Gearbox::update(f32 step, f32 outputSpeed, f32 throttle, bool canShiftUp)
{
    if (!(step > 0.0f) || !isfinite(step))
        return;
    outputSpeed = finiteOr(outputSpeed, 0.0f);
    mOutputOmega = outputSpeed;
    throttle = Clamp(finiteOr(throttle, 0.0f), 0.0f, 1.0f);
    if (shifting())
    {
        mShiftTimer -= step;
        if (mShiftTimer <= 0.0f)
        {
            // Never engage a forward gear the road speed would overspeed ("drive" selected on the move,
            // or speed gained during the shift): take the gear that fits.
            if (mMode == GearboxMode::Automatic && mTarget > 0)
            {
                const f32 rpm = fabsf(outputSpeed) * fabsf(ratioOf(mTarget)) * kRadiansToRpm;
                if (rpm > mParameters.upshiftRpmFull)
                    mTarget = Max(mTarget, automaticGearFor(outputSpeed, throttle));
            }
            mShiftTimer = 0.0f;
            mGear = mTarget;
            mSinceShift = 0.0f;
            ++mShiftCount;
        }
        return;
    }
    mSinceShift += step;

    if (mMode != GearboxMode::Automatic || mGear < 1)
        return;
    const GearboxParameters& p = mParameters;
    if (mSinceShift < p.minShiftInterval)
        return;
    const f32 up = p.upshiftRpmLight + (p.upshiftRpmFull - p.upshiftRpmLight) * throttle;
    const f32 down = p.downshiftRpmLight + (p.downshiftRpmFull - p.downshiftRpmLight) * throttle;
    const f32 current = fabsf(ratioOf(mGear));
    const f32 rpm = fabsf(outputSpeed) * current * kRadiansToRpm;
    const s32 top = static_cast<s32>(p.gearCount);
    if (mGear < top && rpm > up && canShiftUp)
    {
        // Only if the next gear does not land below the downshift line, or the box would hunt.
        // The line at full throttle: on a climb the next gear has to pull at the speed it is
        // engaged at, whatever the pedal, or it is back down a gear after the shift.
        const f32 next = rpm * fabsf(ratioOf(mGear + 1)) / current;
        if (next > p.downshiftRpmFull)
            beginShift(mGear + 1);
    }
    else if (mGear > 1 && rpm < down)
    {
        const f32 next = rpm * fabsf(ratioOf(mGear - 1)) / current;
        if (next < up)
            beginShift(mGear - 1);
    }
}

void Gearbox::setInputAngularVelocity(f32 omega)
{
    mInputOmega = finiteOr(omega, 0.0f);
}

void Gearbox::setSolvedTorque(f32 torque)
{
    mTorque = finiteOr(torque, 0.0f);
}

Differential::Differential(const DifferentialParameters& parameters, DriveOutput a, DriveOutput b)
    : mA(a), mB(b)
{
    setParameters(parameters);
}

void Differential::setParameters(const DifferentialParameters& parameters)
{
    DifferentialParameters p = parameters;
    p.ratio = positiveOr(p.ratio, 1.0e-3f, 1.0f);
    p.split = Clamp(finiteOr(p.split, 0.5f), 0.0f, 1.0f);
    p.preloadTorque = positiveOr(p.preloadTorque, 0.0f, 0.0f);
    p.torqueSensing = positiveOr(p.torqueSensing, 0.0f, 0.0f);
    p.viscousCoefficient = positiveOr(p.viscousCoefficient, 0.0f, 0.0f);
    p.inputInertia = positiveOr(p.inputInertia, 1.0e-4f, 0.05f);
    mParameters = p;
}

void Differential::setLocked(bool locked)
{
    mLocked = locked;
}

void Differential::setInputAngularVelocity(f32 omega)
{
    mInputOmega = finiteOr(omega, 0.0f);
}

void Differential::setSolved(f32 inputTorque, f32 lockingTorque)
{
    mInputTorque = finiteOr(inputTorque, 0.0f);
    mLockingTorque = finiteOr(lockingTorque, 0.0f);
}

} // namespace kinetix::Physics
