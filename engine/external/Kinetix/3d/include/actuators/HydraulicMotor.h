#ifndef KINETIX_PHYSICS_ACTUATORS_HYDRAULICMOTOR_H
#define KINETIX_PHYSICS_ACTUATORS_HYDRAULICMOTOR_H

#include <kinetix/core/Types.h>

#include "dynamics/JointDrive.h"

namespace kinetix::Physics
{

struct HydraulicMotorParameters
{
    f32 displacement = 100.0f;
    // Volumetric efficiency (the share of the flow that turns the motor;
    // the rest leaks to the case drain) and mechanical (torque) efficiency.
    f32 volumetricEfficiency = 0.95f;
    f32 mechanicalEfficiency = 0.92f;
    // Motor revolutions per output revolution.
    f32 gearRatio = 1.0f;
    // 0: the drive's coordinate is the output's angle (rad, N*m). > 0: it is
    // linear (m, N) at this radius - a sprocket driving a track, a drum
    // paying out rope.
    f32 outputRadius = 0.0f;
    // Cross-port relief, Pa: the most the motor drives or brakes with. A
    // load that overruns the metered speed is braked at this pressure.
    f32 reliefPressure = 30.0e6f;
    // The flow its own valve gives it fully open, L/min (setValve).
    f32 maxFlow = 200.0f;
    // A spring-applied holding brake at the output, N*m (or N with an
    // output radius), engaged whenever the command is zero. 0: none - the
    // closed valve holds it hydraulically, at the relief.
    f32 holdingBrake = 0.0f;
};

// Drives a joint's motor row as a hydraulic motor fed with a flow:
//
//   output speed  = Q * eta_v / (D * i)              (rad/s; times r if linear)
//   output torque = dp * D * i * eta_m / (2 pi)      (N*m; over r if linear)
//
// with D the displacement per revolution. Per step the drive becomes a
// velocity motor at the metered speed, bounded by the torque of the
// available pressure (the relief, or the pump's if lower): it stalls at that
// torque, and a load running ahead of the flow is braked at the relief.
//
// Call update() once per step, before the world steps.
class HydraulicMotor
{
public:
    // The drive belongs to the joint and has to outlive the motor.
    HydraulicMotor(JointDrive& drive, const HydraulicMotorParameters& parameters);

    // Its own valve, -1..1 of maxFlow. Clamped; non-finite ignored.
    void setValve(f32 command);
    // The flow a shared circuit meters into it this step, L/min, signed
    // with the direction (replaces the valve until setValve()).
    void setFlow(f32 litresPerMinute);
    // The pressure the supply can reach this step, Pa (<= 0: the relief).
    void setPressureLimit(f32 pascals);
    // Releases the holding brake by hand, or lets it work automatically again.
    void setBrakeReleased(bool released);

    void update(f32 step);

    const HydraulicMotorParameters& parameters() const
    {
        return mParameters;
    }
    // Ratings at the output: the speed of a flow (per L/min) and the torque
    // (or force) of a pressure (per Pa).
    f32 speedPerFlow() const;
    f32 torquePerPressure() const;

    // Output speed (rad/s or m/s) and torque (N*m or N), the drive's signs.
    f32 outputSpeed() const;
    f32 outputTorque() const;
    // The pressure difference across the motor that torque means, Pa
    // (signed with the torque).
    f32 pressure() const;
    // The flow the motor actually takes at its speed, L/min (signed).
    f32 consumedFlow() const;
    // The pressure it needs to keep doing what it is asked (>= 0, Pa):
    // what a load-sensing circuit reads off it.
    f32 loadPressure() const;
    f32 commandedSpeed() const
    {
        return mCommandedSpeed;
    }
    f32 flowCommand() const
    {
        return mFlow;
    }
    bool braking() const
    {
        return mBraking;
    }
    bool reliefOpen() const;

private:
    JointDrive* mDrive;
    HydraulicMotorParameters mParameters;
    f32 mValve = 0.0f;
    f32 mFlow = 0.0f;
    bool mFlowControlled = false;
    f32 mPressureLimit = 0.0f;
    bool mBrakeReleased = false;
    bool mBraking = false;
    f32 mCommandedSpeed = 0.0f;
    f32 mTorqueLimit = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ACTUATORS_HYDRAULICMOTOR_H
