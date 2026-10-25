#ifndef KINETIX_PHYSICS_ROTORCRAFT_ROTORTRANSMISSION_H
#define KINETIX_PHYSICS_ROTORCRAFT_ROTORTRANSMISSION_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class TurboshaftEngine;

// What the drive did on its last step. Speeds in rad/s, torques in N m.
struct TransmissionState
{
    f32 rotorSpeed = 0.0f;
    f32 tailRotorSpeed = 0.0f;
    f32 turbineSpeed = 0.0f;
    // d omega / dt of the main rotor (and the tail rotor geared to it) and of the power turbine.
    f32 rotorAcceleration = 0.0f;
    f32 turbineAcceleration = 0.0f;
    // Through the freewheel unit, at the turbine shaft; zero when it overruns.
    f32 engineTorque = 0.0f;
    // What the shafts deliver to their rotors (aerodynamic torque plus spin-up); the airframe
    // feels the opposite.
    f32 mainShaftTorque = 0.0f;
    f32 tailShaftTorque = 0.0f;
    // Shaft power into the main rotor gearbox from the engine, W.
    f32 enginePower = 0.0f;
    bool engaged = false;
};

// The helicopter's drive train: power turbine, freewheel (sprag) unit, main gearbox, and a tail
// rotor drive geared to it.
//
// The rotors are one rigid system, omega_tail = tailRotorRatio * omega_main: inertia on the main
// shaft I_main + ratio^2 I_tail, load the main rotor's aerodynamic torque plus ratio times the
// tail rotor's, plus friction. The engine joins through the freewheel at engineRatio = Np / Nr:
//  - engaged, both sides turn together; the acceleration comes from the engine torque (times
//    efficiency) against the load over the summed inertia, and the sprag torque is what the
//    turbine has left after accelerating its own inertia. If that comes out negative the unit
//    overruns;
//  - overrunning, each side spins on its own torques; it re-engages when the turbine catches up,
//    speeds equalised by an inelastic (momentum-conserving) lock-up.
// The engine can drive the rotor but never be driven by it: with the engine stopped the rotor keeps
// the speed the air through the disc gives it (autorotation).
//
// Each side is integrated linearly implicitly in the slope of its torques (as the propeller is),
// which only sets how a speed approaches its balance, so a light turbine on a long step does not
// overshoot.
class RotorTransmission
{
public:
    struct Description
    {
        f32 mainRotorInertia = 680.0f; // kg m^2, blades and hub
        f32 tailRotorInertia = 0.8f;
        f32 tailRotorRatio = 6.25f;    // omega_tail / omega_main
        f32 engineRatio = 15.0f;       // Np / Nr
        f32 efficiency = 0.97f;        // gearbox, engine to rotors
        // Friction and accessory drag on the rotor side, as a torque at the main shaft: constant
        // plus proportional to rotor speed.
        f32 frictionTorque = 20.0f;
        f32 frictionPerSpeed = 1.0f;   // N m per rad/s
        bool freewheel = true;
    };

    RotorTransmission() = default;
    explicit RotorTransmission(const Description& description);

    void setDescription(const Description& description);
    const Description& description() const
    {
        return mDescription;
    }

    // Both sides to this main rotor speed, engaged.
    void setRotorSpeed(f32 omega);
    // Only the turbine (e.g. 0 for a cold engine), unengaged until it catches up.
    void setTurbineSpeed(f32 omega);
    f32 rotorSpeed() const
    {
        return mRotorSpeed;
    }
    f32 tailRotorSpeed() const
    {
        return mRotorSpeed * mDescription.tailRotorRatio;
    }
    f32 turbineSpeed() const
    {
        return mTurbineSpeed;
    }
    bool engaged() const
    {
        return mEngaged;
    }

    // The rotors' aerodynamic torques (positive resisting rotation) at the current speeds;
    // `engine` may be null for an unpowered rotor. The engine's fuel control is not stepped here.
    const TransmissionState& step(f32 step, const TurboshaftEngine* engine, f32 density,
                                  f32 mainRotorTorque, f32 tailRotorTorque);
    const TransmissionState& state() const
    {
        return mState;
    }

private:
    Description mDescription;
    f32 mRotorSpeed = 0.0f;
    f32 mTurbineSpeed = 0.0f;
    bool mEngaged = true;
    TransmissionState mState;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ROTORCRAFT_ROTORTRANSMISSION_H
