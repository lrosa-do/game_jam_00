#ifndef KINETIX_PHYSICS_ACTUATORS_ELECTRICMOTOR_H
#define KINETIX_PHYSICS_ACTUATORS_ELECTRICMOTOR_H

#include <kinetix/core/Types.h>

#include "dynamics/JointDrive.h"

namespace kinetix::Physics
{

struct ElectricMotorParameters
{
    // Torque at zero speed and full voltage, N*m, and speed at no load, rad/s:
    // the two ends of the straight torque-speed line.
    f32 stallTorque = 1.0f;
    f32 noLoadSpeed = 300.0f;
    // Current at stall and at no load, A.
    f32 stallCurrent = 20.0f;
    f32 noLoadCurrent = 0.5f;
    f32 supplyVoltage = 24.0f;
    // Motor turns per output turn: the output gets ratio x the torque and
    // 1/ratio of the speed. Efficiency (0..1) of the gearbox, taken off the
    // torque the way the power flows.
    f32 gearRatio = 1.0f;
    f32 efficiency = 1.0f;
};

// Drives a joint's motor/servo row as an electric motor.
//
// T = Ts * (u - w / w0) at the output, with Ts = stallTorque * ratio *
// efficiency and w0 = noLoadSpeed / ratio: the voltage command u (-1..1)
// shifts the line, the stall current caps it at +-Ts. Solved implicitly as a
// velocity row with a viscous gain, so a light rotor cannot destabilise it.
// Position mode is a servo whose PD output is held inside that envelope.
//
// Not modelled: rotor inertia (ratio^2 * J_rotor through the gearbox),
// winding inductance, heating.
class ElectricMotor
{
public:
    // The drive belongs to the joint and has to outlive the motor.
    ElectricMotor(JointDrive& drive, const ElectricMotorParameters& parameters);

    // Voltage command, -1..1 of the supply: runs on the torque-speed line.
    // Clamped; non-finite values are ignored.
    void setVoltage(f32 command);
    // Servo to a joint position (rad or m, the joint's own coordinate) with
    // these gains; `gains.maxForce` is taken from the motor (the output
    // stall torque) unless it is set lower.
    void setPositionTarget(f32 target, const ServoSettings& gains);
    void setPositionTarget(f32 target, f32 targetVelocity = 0.0f);
    void stop();

    const ElectricMotorParameters& parameters() const
    {
        return mParameters;
    }
    f32 outputStallTorque() const;
    f32 outputNoLoadSpeed() const;

    f32 outputTorque() const;
    f32 outputSpeed() const;
    f32 motorTorque() const;
    f32 motorSpeed() const;
    // Winding current, A (signed with the motor torque), from the torque
    // constant the datasheet implies: I = I0 + (Is - I0) * T / Ts.
    f32 current() const;
    // Power drawn from the supply (W, negative regenerating) at the voltage
    // that puts the motor where it is on its line, and delivered at the
    // output (W).
    f32 electricalPower() const;
    f32 mechanicalPower() const;

private:
    JointDrive* mDrive;
    ElectricMotorParameters mParameters;
    f32 mVoltage = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ACTUATORS_ELECTRICMOTOR_H
