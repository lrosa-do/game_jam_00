#ifndef KINETIX_PHYSICS_ROTORCRAFT_TURBOSHAFTENGINE_H
#define KINETIX_PHYSICS_ROTORCRAFT_TURBOSHAFTENGINE_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Free-turbine turboshaft with fuel control and rotor speed governor.
//
// Two spools sharing no shaft. The gas generator runs at Ng: fuel flow sets the speed it settles
// at, with a first-order lag whose time constant grows as the spool slows (spoolTimeConstant at
// 100%, divided by Ng). Its exhaust drives the free power turbine (output shaft at Np,
// mechanically independent of Ng, so the rotor keeps its speed while power changes).
//
// Gas power P_gas = maxPower * sigma^0.8 * ((Ng - Ng0) / (1 - Ng0))^2, zero below Ng0
// (zeroPowerGasGenerator), sigma the density ratio (empirical altitude lapse). The power turbine
// follows the free-turbine line, linear in its own speed:
//     Q_pt = (P_gas / omega_0) (2 - Np / Np_0),
// twice rated torque stalled, full gas power at rated speed, nothing at twice it. A small windage
// drag acts on the power turbine regardless.
//
// Fuel control: the governor holds Np at governedSpeed (fraction of rated) with a PI loop on the
// speed error plus collective anticipation (the lever feeds the fuel demand directly, so a pull
// accelerates the gas generator before the rotor slows). The demand (0..1) maps onto Ng between
// idle and maxGasGenerator; the throttle (twist grip) caps it: 0 holds idle, 1 gives the governor
// full authority. Fuel flow = idleFuelFlow + specificFuelConsumption * P_gas.
//
// With running set the gas generator spools to idle (starter and light-off folded into the lag);
// cleared, fuel is cut and Ng runs down with spoolDownTimeConstant, leaving the power turbine only
// windage, which lets the freewheel unit separate it from the rotor.
class TurboshaftEngine
{
public:
    struct Description
    {
        f32 maxPower = 300000.0f;        // W, Ng 100%, sea level
        f32 ratedTurbineRpm = 6000.0f;   // power turbine at 100%
        f32 idleGasGenerator = 0.62f;    // Ng at idle
        f32 maxGasGenerator = 1.02f;     // topping
        f32 zeroPowerGasGenerator = 0.5f;
        f32 spoolTimeConstant = 0.45f;   // s, at Ng = 100%
        f32 spoolDownTimeConstant = 3.0f; // s, fuel off
        f32 turbineInertia = 0.1f;       // kg m^2, power turbine and output shaft
        f32 windage = 0.01f;             // share of rated torque at rated speed
        // Governor: fuel demand per unit (fractional) speed error, per unit error per second, and
        // per unit collective (plus a bias).
        f32 governorGain = 5.0f;
        f32 governorIntegral = 2.5f;
        f32 anticipation = 0.55f;
        f32 anticipationBias = 0.15f;
        f32 specificFuelConsumption = 1.0e-7f; // kg/J (0.36 kg/kWh)
        f32 idleFuelFlow = 0.004f;             // kg/s
    };

    TurboshaftEngine() = default;
    explicit TurboshaftEngine(const Description& description);

    void setDescription(const Description& description);
    const Description& description() const
    {
        return mDescription;
    }

    void setRunning(bool running)
    {
        mRunning = running;
    }
    bool running() const
    {
        return mRunning;
    }
    void setThrottle(f32 throttle);
    f32 throttle() const
    {
        return mThrottle;
    }
    // Collective lever 0..1, for the anticipator.
    void setCollective(f32 collective);
    // Np the governor holds, as a fraction of rated (1 = 100%).
    void setGovernedSpeed(f32 fraction);
    f32 governedSpeed() const
    {
        return mGovernedSpeed;
    }

    // Starts the engine running at this gas generator speed with the governor settled (integrator
    // set for a steady demand): an initial condition, not a start sequence.
    void setGasGenerator(f32 fraction);
    f32 gasGenerator() const
    {
        return mGasGenerator;
    }

    // Advances the fuel control and gas generator; reads the power turbine speed (rad/s) the governor regulates.
    void step(f32 step, f32 turbineSpeed, f32 density);

    // Power turbine output torque, N m, at turbine speed omega (rad/s) and the current gas
    // generator state; and its slope d torque / d omega.
    f32 turbineTorque(f32 omega, f32 density) const;
    f32 turbineTorqueSlope(f32 omega, f32 density) const;
    f32 gasPower(f32 density) const;
    f32 fuelFlow(f32 density) const;
    f32 fuelDemand() const
    {
        return mDemand;
    }
    f32 ratedTurbineSpeed() const; // rad/s

private:
    Description mDescription;
    bool mRunning = false;
    f32 mThrottle = 1.0f;
    f32 mCollective = 0.0f;
    f32 mGovernedSpeed = 1.0f;
    f32 mGasGenerator = 0.0f;
    f32 mIntegral = 0.0f;
    f32 mDemand = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_ROTORCRAFT_TURBOSHAFTENGINE_H
