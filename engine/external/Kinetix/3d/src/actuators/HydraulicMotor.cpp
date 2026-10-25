#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "actuators/HydraulicMotor.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kLitresPerMinute = 1.0f / 60000.0f; // m^3/s per L/min
constexpr f32 kCubicCentimetre = 1.0e-6f;

f32 positiveOr(f32 value, f32 minimum, f32 fallback)
{
    return isfinite(value) ? Math::max(value, minimum) : fallback;
}

}

HydraulicMotor::HydraulicMotor(JointDrive& drive, const HydraulicMotorParameters& parameters)
    : mDrive(&drive), mParameters(parameters)
{
    HydraulicMotorParameters& p = mParameters;
    p.displacement = positiveOr(p.displacement, 1.0e-3f, 100.0f);
    p.volumetricEfficiency = isfinite(p.volumetricEfficiency) ? Math::clamp(p.volumetricEfficiency, 0.01f, 1.0f) : 1.0f;
    p.mechanicalEfficiency = isfinite(p.mechanicalEfficiency) ? Math::clamp(p.mechanicalEfficiency, 0.01f, 1.0f) : 1.0f;
    p.gearRatio = positiveOr(p.gearRatio, 1.0e-3f, 1.0f);
    p.outputRadius = positiveOr(p.outputRadius, 0.0f, 0.0f);
    p.reliefPressure = positiveOr(p.reliefPressure, 0.0f, 0.0f);
    p.maxFlow = positiveOr(p.maxFlow, 0.0f, 0.0f);
    p.holdingBrake = positiveOr(p.holdingBrake, 0.0f, 0.0f);
}

void HydraulicMotor::setValve(f32 command)
{
    if (!isfinite(command))
        return;
    mValve = Math::clamp(command, -1.0f, 1.0f);
    mFlowControlled = false;
}

void HydraulicMotor::setFlow(f32 litresPerMinute)
{
    if (!isfinite(litresPerMinute))
        return;
    mFlow = litresPerMinute;
    mFlowControlled = true;
}

void HydraulicMotor::setPressureLimit(f32 pascals)
{
    mPressureLimit = isfinite(pascals) && pascals > 0.0f ? pascals : 0.0f;
}

void HydraulicMotor::setBrakeReleased(bool released)
{
    mBrakeReleased = released;
}

// Output speed per L/min: Q eta_v / (D i), D in m^3 per radian of the motor.
f32 HydraulicMotor::speedPerFlow() const
{
    const HydraulicMotorParameters& p = mParameters;
    const f32 displacementPerRadian = p.displacement * kCubicCentimetre / Math::two_pi<f32>();
    const f32 angular = kLitresPerMinute * p.volumetricEfficiency / (displacementPerRadian * p.gearRatio);
    return p.outputRadius > 0.0f ? angular * p.outputRadius : angular;
}

// Output torque per Pa: D i eta_m, D in m^3 per radian.
f32 HydraulicMotor::torquePerPressure() const
{
    const HydraulicMotorParameters& p = mParameters;
    const f32 displacementPerRadian = p.displacement * kCubicCentimetre / Math::two_pi<f32>();
    const f32 torque = displacementPerRadian * p.gearRatio * p.mechanicalEfficiency;
    return p.outputRadius > 0.0f ? torque / p.outputRadius : torque;
}

void HydraulicMotor::update(f32 step)
{
    (void)step;
    const HydraulicMotorParameters& p = mParameters;
    const f32 flow = mFlowControlled ? mFlow : mValve * p.maxFlow;
    mFlow = flow;
    const f32 pressure = mPressureLimit > 0.0f ? Math::min(mPressureLimit, p.reliefPressure) : p.reliefPressure;
    mCommandedSpeed = flow * speedPerFlow();
    // A closed valve traps the oil: the motor holds at the relief (a
    // cross-port relief lets a load through above it), plus the brake.
    mBraking = flow == 0.0f && p.holdingBrake > 0.0f && !mBrakeReleased;
    const f32 relief = p.reliefPressure * torquePerPressure();
    mTorqueLimit = flow == 0.0f ? relief + (mBraking ? p.holdingBrake : 0.0f) : pressure * torquePerPressure();
    // Overrunning loads are braked at the relief, whatever the supply
    // pressure: the force range is asymmetric when the supply is lower.
    mDrive->setVelocity(mCommandedSpeed, Math::max(mTorqueLimit, relief));
    if (flow > 0.0f)
        mDrive->setForceRange(-relief, mTorqueLimit);
    else if (flow < 0.0f)
        mDrive->setForceRange(-mTorqueLimit, relief);
    else
        mDrive->setForceRange(-mTorqueLimit, mTorqueLimit);
}

f32 HydraulicMotor::outputSpeed() const
{
    return mDrive->velocity();
}

f32 HydraulicMotor::outputTorque() const
{
    return mDrive->appliedForce();
}

f32 HydraulicMotor::pressure() const
{
    const f32 perPressure = torquePerPressure();
    return perPressure > 0.0f ? outputTorque() / perPressure : 0.0f;
}

f32 HydraulicMotor::consumedFlow() const
{
    const f32 perFlow = speedPerFlow();
    return perFlow > 0.0f ? outputSpeed() / perFlow : 0.0f;
}

f32 HydraulicMotor::loadPressure() const
{
    // Pressure on the inflow side; an overrunning load needs none.
    const f32 direction = mFlow > 0.0f ? 1.0f : (mFlow < 0.0f ? -1.0f : 0.0f);
    return Math::max(pressure() * direction, 0.0f);
}

bool HydraulicMotor::reliefOpen() const
{
    return mDrive->saturated();
}

} // namespace kinetix::Physics
