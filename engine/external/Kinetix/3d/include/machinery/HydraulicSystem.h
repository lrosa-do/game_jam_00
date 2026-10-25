#ifndef KINETIX_PHYSICS_MACHINERY_HYDRAULICSYSTEM_H
#define KINETIX_PHYSICS_MACHINERY_HYDRAULICSYSTEM_H

#include <kinetix/core/Types.h>

#include "ground/Engine.h"

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class HydraulicCylinder;
class HydraulicMotor;

enum class HydraulicCircuit : u8
{
    // Fixed-displacement pump, open-centre spools: in neutral the whole flow returns to tank through
    // the centre gallery; opening a spool chokes it, so flow divides by the orifices' pressure drops
    // (lightest load first, speed depends on load). Q = K u sqrt(p_pump - p_load).
    OpenCentre,
    // Variable pump holding a margin above the highest load pressure, post-compensated flow-sharing
    // sections: each gets the flow its spool asks for whatever its load; when the pump cannot
    // deliver all (displacement, power or speed) every section is cut back in the same proportion.
    // Priority sections are served first.
    LoadSensing
};

struct HydraulicPumpParameters
{
    HydraulicCircuit circuit = HydraulicCircuit::LoadSensing;
    // Full displacement, cm^3 per pump revolution; pump revolutions per engine revolution.
    f32 displacement = 120.0f;
    f32 driveRatio = 1.0f;
    f32 volumetricEfficiency = 0.95f;
    f32 mechanicalEfficiency = 0.92f;
    // Main relief / pressure cut-off, Pa.
    f32 maxPressure = 32.0e6f;
    // Load sensing: margin above the highest load pressure; stand-by pressure with nothing working, Pa.
    f32 marginPressure = 2.0e6f;
    f32 standbyPressure = 2.5e6f;
    // Open centre: pressure drop of the full flow through the open centre gallery in neutral, and
    // through one fully open metering orifice at its section's rated flow, Pa.
    f32 neutralPressure = 1.0e6f;
    f32 meteringPressure = 1.5e6f;
    // Regulator power limit (W of hydraulic power, p * Q): above it the pump destrokes. 0: no
    // limit - an overload lugs the engine down or stalls it.
    f32 powerLimit = 0.0f;
    // Anti-stall (load sensing with a power limit): with the engine pulled more than this far
    // (rpm) below its governor setting the power limit falls in proportion, to nothing at twice as
    // far, since a lugged engine has less torque than the limit (set at rated speed) asks.
    f32 antiStallDroop = 150.0f;
    // Time constant of the swash plate following the regulator (load sensing), s; also keeps a
    // load held back by the power limit from hunting between two flows.
    f32 responseTime = 0.08f;
};

struct ValveSectionParameters
{
    // Flow at full spool, L/min (load sensing: compensated flow; open centre: at the rated metering drop).
    f32 ratedFlow = 150.0f;
    // Port relief, Pa: what a closed spool holds against.
    f32 portRelief = 35.0e6f;
    // Spool overlap (fraction of the -1..1 range with no flow); spool lag time constant, s.
    f32 deadband = 0.05f;
    f32 responseTime = 0.06f;
    // Load sensing: higher priority sections are served before the rest share what is left. 0 shares.
    u32 priority = 0;
};

// The hydraulic system of a machine: a diesel engine with an all-speed governor, pumps on it, and
// valve sections feeding cylinders and motors.
//
// update() per step (before the world steps), from what the consumers did last step:
//  1. Each section's load pressure: consumer force (or torque) over area (or displacement), on
//     the side the spool sends the oil to.
//  2. Each pump's flow and pressure by its circuit, Q = D n eta_v, capped by the power limit;
//     pressure capped at relief (load-sensing cut-off: a section blocked at it gets nothing).
//  3. Section flows go to the consumers (setFlow) with the pump pressure as their limit
//     (setPressureLimit): a cylinder moves at its allocated flow until the load needs more pressure.
//  4. Shaft torque D_eff p / (2 pi eta_m) loads the engine, whose speed is integrated from
//     combustion torque (governor throttle), friction and that load; overload lugs it down the
//     governor's droop and below stall speed it stops, and with it the flow.
//
// Flow is conserved: a pump's flow is the sum of its sections' flows, the relief flow and (open
// centre) the centre bypass.
class HydraulicSystem
{
public:
    static constexpr u32 MaxConsumersPerSection = 4;

    explicit HydraulicSystem(const EngineParameters& engine);

    u32 addPump(const HydraulicPumpParameters& parameters);
    u32 addSection(u32 pump, const ValveSectionParameters& parameters);
    // Consumers of a section are fed in parallel (a boom with two cylinders): + spool extends a
    // cylinder, turns a motor positive; negative `sign` reverses one. Cylinders share the flow by
    // their areas. The consumer is not owned.
    bool addConsumer(u32 section, HydraulicCylinder& cylinder, f32 sign = 1.0f);
    bool addConsumer(u32 section, HydraulicMotor& motor, f32 sign = 1.0f);

    // A section's spool, -1..1 (clamped, non-finite ignored).
    void setSpool(u32 section, f32 command);
    f32 spool(u32 section) const;
    // Engine speed dial (rpm the governor holds at no load), clamped to idle..high idle; starter.
    void setEngineSpeed(f32 rpm);
    void setStarter(bool cranking)
    {
        mStarter = cranking;
    }

    // Allocates the flows, feeds the consumers, loads and turns the engine. Call before the world's
    // step and before HydraulicMotor's update() (the system calls it for its motors).
    void update(f32 step);

    Engine& engine()
    {
        return mEngine;
    }
    const Engine& engine() const
    {
        return mEngine;
    }
    u32 pumpCount() const
    {
        return static_cast<u32>(mPumps.size());
    }
    u32 sectionCount() const
    {
        return static_cast<u32>(mSections.size());
    }
    const HydraulicPumpParameters& pumpParameters(u32 pump) const
    {
        return mPumps[pump].parameters;
    }
    const ValveSectionParameters& sectionParameters(u32 section) const
    {
        return mSections[section].parameters;
    }

    struct PumpState
    {
        // Flow delivered, L/min, at this pressure, Pa.
        f32 flow = 0.0f;
        f32 pressure = 0.0f;
        // Most it could deliver at this engine speed, and what the power regulator left, L/min.
        f32 maxFlow = 0.0f;
        f32 availableFlow = 0.0f;
        // Over the main relief and (open centre) through the centre gallery, L/min. Load sensing has
        // no main relief flow; see SectionState::reliefFlow.
        f32 reliefFlow = 0.0f;
        f32 bypassFlow = 0.0f;
        // Displacement in use (0..1 of full), shaft torque (N*m, pump side), hydraulic power p Q (W).
        f32 displacementFraction = 0.0f;
        f32 shaftTorque = 0.0f;
        f32 hydraulicPower = 0.0f;
        // The regulator cut the flow back (load sensing: flow sharing was also needed).
        bool powerLimited = false;
        bool saturated = false;
    };
    struct SectionState
    {
        // Spool after its lag (-1..1), flow asked and got (L/min, signed with the spool), load pressure (Pa).
        f32 spool = 0.0f;
        f32 demandedFlow = 0.0f;
        f32 flow = 0.0f;
        f32 loadPressure = 0.0f;
        // L/min consumers took last step, and what they could not take that went over the section's
        // relief.
        f32 consumedFlow = 0.0f;
        f32 reliefFlow = 0.0f;
    };
    const PumpState& pump(u32 index) const
    {
        return mPumps[index].state;
    }
    const SectionState& section(u32 index) const
    {
        return mSections[index].state;
    }
    // Torque all pumps take from the crankshaft, N*m.
    f32 engineLoadTorque() const
    {
        return mEngineLoad;
    }
    f32 engineThrottle() const
    {
        return mThrottle;
    }
    f32 engineSpeedSetting() const
    {
        return mSpeedSetting;
    }
    bool engineRunning() const
    {
        return mEngine.running();
    }

private:
    struct Consumer
    {
        HydraulicCylinder* cylinder = nullptr;
        HydraulicMotor* motor = nullptr;
        f32 sign = 1.0f;
    };
    struct Section
    {
        ValveSectionParameters parameters;
        u32 pump = 0;
        Consumer consumers[MaxConsumersPerSection];
        u32 consumerCount = 0;
        f32 command = 0.0f;
        SectionState state;
    };
    struct Pump
    {
        HydraulicPumpParameters parameters;
        PumpState state;
        // m^3/s, after the swash plate lag.
        f32 regulatedFlow = 0.0f;
        bool regulated = false;
    };

    f32 meteredCommand(const Section& section) const;
    f32 loadPressure(const Section& section, f32 direction) const;
    f32 consumedFlow(const Section& section) const;
    void allocateOpenCentre(Pump& pump, u32 index, f32 theoretical);
    void allocateLoadSensing(Pump& pump, u32 index, f32 theoretical, f32 step);
    void feed(Section& section, f32 pressureLimit, f32 step);

    Engine mEngine;
    ct::Vector<Pump> mPumps;
    ct::Vector<Section> mSections;
    f32 mSpeedSetting = 0.0f;
    bool mStarter = false;
    f32 mThrottle = 0.0f;
    f32 mEngineLoad = 0.0f;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_HYDRAULICSYSTEM_H
