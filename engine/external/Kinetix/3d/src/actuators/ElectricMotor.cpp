#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/ElectricMotor.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

f32 positiveOr(f32 value, f32 fallback)
{
    return isfinite(value) && value > 0.0f ? value : fallback;
}

}

ElectricMotor::ElectricMotor(JointDrive& drive, const ElectricMotorParameters& parameters)
    : mDrive(&drive), mParameters(parameters)
{
    ElectricMotorParameters& p = mParameters;
    p.stallTorque = positiveOr(p.stallTorque, 1.0f);
    p.noLoadSpeed = positiveOr(p.noLoadSpeed, 300.0f);
    p.stallCurrent = positiveOr(p.stallCurrent, 1.0f);
    p.noLoadCurrent = isfinite(p.noLoadCurrent) ? Math::clamp(p.noLoadCurrent, 0.0f, p.stallCurrent) : 0.0f;
    p.supplyVoltage = positiveOr(p.supplyVoltage, 24.0f);
    p.gearRatio = positiveOr(p.gearRatio, 1.0f);
    p.efficiency = isfinite(p.efficiency) ? Math::clamp(p.efficiency, 0.01f, 1.0f) : 1.0f;
}

f32 ElectricMotor::outputStallTorque() const
{
    return mParameters.stallTorque * mParameters.gearRatio * mParameters.efficiency;
}

f32 ElectricMotor::outputNoLoadSpeed() const
{
    return mParameters.noLoadSpeed / mParameters.gearRatio;
}

void ElectricMotor::setVoltage(f32 command)
{
    if (!isfinite(command))
        return;
    mVoltage = Math::clamp(command, -1.0f, 1.0f);
    const f32 stall = outputStallTorque();
    const f32 noLoad = outputNoLoadSpeed();
    // The torque-speed line is the damper of the velocity row: the slope is
    // Ts / w0, the zero-torque point u * w0, the current limit +-Ts.
    mDrive->clearForceRange();
    mDrive->setTorqueSpeedLine(0.0f, 0.0f);
    mDrive->setVelocity(mVoltage * noLoad, stall, stall / noLoad);
}

void ElectricMotor::setPositionTarget(f32 target, const ServoSettings& gains)
{
    ServoSettings settings = gains;
    const f32 stall = outputStallTorque();
    settings.maxForce = settings.maxForce > 0.0f ? Math::min(settings.maxForce, stall) : stall;
    mDrive->clearForceRange();
    mDrive->setTorqueSpeedLine(stall, outputNoLoadSpeed());
    mDrive->setServo(target, settings);
}

void ElectricMotor::setPositionTarget(f32 target, f32 targetVelocity)
{
    mDrive->setServoTarget(target, targetVelocity);
}

void ElectricMotor::stop()
{
    mVoltage = 0.0f;
    mDrive->setTorqueSpeedLine(0.0f, 0.0f);
    mDrive->disable();
}

f32 ElectricMotor::outputTorque() const
{
    return mDrive->appliedForce();
}

f32 ElectricMotor::outputSpeed() const
{
    return mDrive->velocity();
}

f32 ElectricMotor::motorSpeed() const
{
    return outputSpeed() * mParameters.gearRatio;
}

// The gearbox loses the same fraction whichever way the power goes: driving,
// the motor makes T / (ratio * eff) for the output's T; driven back, it sees
// only T * eff / ratio.
f32 ElectricMotor::motorTorque() const
{
    const f32 torque = outputTorque();
    const f32 ratio = mParameters.gearRatio;
    const f32 efficiency = mParameters.efficiency;
    const bool driving = torque * outputSpeed() >= 0.0f;
    return driving ? torque / (ratio * efficiency) : torque * efficiency / ratio;
}

f32 ElectricMotor::current() const
{
    const f32 torque = motorTorque();
    const ElectricMotorParameters& p = mParameters;
    const f32 perTorque = (p.stallCurrent - p.noLoadCurrent) / p.stallTorque;
    f32 sign = torque > 0.0f ? 1.0f : (torque < 0.0f ? -1.0f : 0.0f);
    if (sign == 0.0f)
        sign = motorSpeed() > 0.0f ? 1.0f : (motorSpeed() < 0.0f ? -1.0f : 0.0f);
    return p.noLoadCurrent * sign + torque * perTorque;
}

// The voltage the drive is applying is the one that puts this torque and
// speed on the line: u = T / Ts + w / w0.
f32 ElectricMotor::electricalPower() const
{
    const f32 command = Math::clamp(outputTorque() / outputStallTorque() + outputSpeed() / outputNoLoadSpeed(),
                                    -1.0f, 1.0f);
    return mParameters.supplyVoltage * command * current();
}

f32 ElectricMotor::mechanicalPower() const
{
    return outputTorque() * outputSpeed();
}

} // namespace kinetix::Physics
