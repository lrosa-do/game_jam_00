#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/HydraulicSystem.h"

#include "actuators/HydraulicCylinder.h"
#include "actuators/HydraulicMotor.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kLitresPerMinute = 1.0f / 60000.0f; // m^3/s per L/min
constexpr f32 kCubicCentimetre = 1.0e-6f;
constexpr f32 kRpmToRadians = 2.0f * 3.14159265358979f / 60.0f;
// All-speed governor droop: full fuel at the dial's speed, none this far above it.
constexpr f32 kGovernorBand = 80.0f;
// A section whose consumer is at its end stop: more than any pump makes.
constexpr f32 kBlocked = 1.0e12f;
// Engine speed is integrated in this many substeps (the governor and a speed-falling pump torque are stiff).
constexpr u32 kEngineSubsteps = 4;

f32 positiveOr(f32 value, f32 minimum, f32 fallback)
{
    return isfinite(value) ? Math::max(value, minimum) : fallback;
}

f32 signOf(f32 value)
{
    return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f);
}

}

HydraulicSystem::HydraulicSystem(const EngineParameters& engine) : mEngine(engine)
{
    mSpeedSetting = mEngine.parameters().limiterRpm;
    mEngine.start();
}

u32 HydraulicSystem::addPump(const HydraulicPumpParameters& parameters)
{
    Pump pump;
    HydraulicPumpParameters& p = pump.parameters;
    p = parameters;
    p.displacement = positiveOr(p.displacement, 0.0f, 0.0f);
    p.driveRatio = positiveOr(p.driveRatio, 1.0e-3f, 1.0f);
    p.volumetricEfficiency = isfinite(p.volumetricEfficiency) ? Math::clamp(p.volumetricEfficiency, 0.01f, 1.0f) : 1.0f;
    p.mechanicalEfficiency = isfinite(p.mechanicalEfficiency) ? Math::clamp(p.mechanicalEfficiency, 0.01f, 1.0f) : 1.0f;
    p.maxPressure = positiveOr(p.maxPressure, 1.0e5f, 32.0e6f);
    p.marginPressure = positiveOr(p.marginPressure, 0.0f, 2.0e6f);
    p.standbyPressure = Math::min(positiveOr(p.standbyPressure, 0.0f, 2.5e6f), p.maxPressure);
    p.neutralPressure = positiveOr(p.neutralPressure, 1.0e3f, 1.0e6f);
    p.meteringPressure = positiveOr(p.meteringPressure, 1.0e3f, 1.5e6f);
    p.powerLimit = positiveOr(p.powerLimit, 0.0f, 0.0f);
    p.antiStallDroop = positiveOr(p.antiStallDroop, 0.0f, 0.0f);
    p.responseTime = positiveOr(p.responseTime, 0.0f, 0.0f);
    mPumps.push_back(pump);
    return static_cast<u32>(mPumps.size() - 1);
}

u32 HydraulicSystem::addSection(u32 pump, const ValveSectionParameters& parameters)
{
    if (pump >= mPumps.size())
    {
        Log::error("HydraulicSystem: no pump %u for a valve section", pump);
        return 0xFFFFFFFFu;
    }
    Section section;
    section.pump = pump;
    ValveSectionParameters& p = section.parameters;
    p = parameters;
    p.ratedFlow = positiveOr(p.ratedFlow, 0.0f, 0.0f);
    p.portRelief = positiveOr(p.portRelief, 0.0f, 35.0e6f);
    p.deadband = isfinite(p.deadband) ? Math::clamp(p.deadband, 0.0f, 0.5f) : 0.0f;
    p.responseTime = positiveOr(p.responseTime, 0.0f, 0.0f);
    mSections.push_back(section);
    return static_cast<u32>(mSections.size() - 1);
}

bool HydraulicSystem::addConsumer(u32 section, HydraulicCylinder& cylinder, f32 sign)
{
    if (section >= mSections.size() || mSections[section].consumerCount >= MaxConsumersPerSection)
        return false;
    Section& s = mSections[section];
    s.consumers[s.consumerCount].cylinder = &cylinder;
    s.consumers[s.consumerCount].sign = sign < 0.0f ? -1.0f : 1.0f;
    ++s.consumerCount;
    return true;
}

bool HydraulicSystem::addConsumer(u32 section, HydraulicMotor& motor, f32 sign)
{
    if (section >= mSections.size() || mSections[section].consumerCount >= MaxConsumersPerSection)
        return false;
    Section& s = mSections[section];
    s.consumers[s.consumerCount].motor = &motor;
    s.consumers[s.consumerCount].sign = sign < 0.0f ? -1.0f : 1.0f;
    ++s.consumerCount;
    return true;
}

void HydraulicSystem::setSpool(u32 section, f32 command)
{
    if (section >= mSections.size() || !isfinite(command))
        return;
    mSections[section].command = Math::clamp(command, -1.0f, 1.0f);
}

f32 HydraulicSystem::spool(u32 section) const
{
    return section < mSections.size() ? mSections[section].state.spool : 0.0f;
}

void HydraulicSystem::setEngineSpeed(f32 rpm)
{
    if (!isfinite(rpm))
        return;
    const EngineParameters& p = mEngine.parameters();
    const f32 top = p.curvePoints > 0 ? p.curveRpm[p.curvePoints - 1] : p.limiterRpm;
    mSpeedSetting = Math::clamp(rpm, p.idleRpm, top);
}

f32 HydraulicSystem::meteredCommand(const Section& section) const
{
    const f32 spool = section.state.spool;
    const f32 deadband = section.parameters.deadband;
    if (fabsf(spool) <= deadband)
        return 0.0f;
    return signOf(spool) * (fabsf(spool) - deadband) / (1.0f - deadband);
}

// Pressure the section's consumers need to move the way the oil is sent (direction +-1), from
// last step's push. Overrunning loads need none (counterbalance valves meter them out).
f32 HydraulicSystem::loadPressure(const Section& section, f32 direction) const
{
    f32 pressure = 0.0f;
    for (u32 c = 0; c < section.consumerCount; ++c)
    {
        const Consumer& consumer = section.consumers[c];
        const f32 way = direction * consumer.sign;
        if (consumer.cylinder)
        {
            const HydraulicCylinder& cylinder = *consumer.cylinder;
            if (cylinder.atStrokeEnd(way))
                return kBlocked;
            const f32 force = cylinder.force();
            const f32 need = way > 0.0f ? Math::max(force, 0.0f) / cylinder.capArea()
                                        : Math::max(-force, 0.0f) / cylinder.annulusArea();
            pressure = Math::max(pressure, need);
        }
        else if (consumer.motor)
        {
            pressure = Math::max(pressure, Math::max(consumer.motor->pressure() * way, 0.0f));
        }
    }
    return pressure;
}

f32 HydraulicSystem::consumedFlow(const Section& section) const
{
    f32 flow = 0.0f;
    for (u32 c = 0; c < section.consumerCount; ++c)
    {
        const Consumer& consumer = section.consumers[c];
        if (consumer.cylinder)
            flow += consumer.sign * signOf(consumer.cylinder->strokeVelocity()) * consumer.cylinder->flow();
        else if (consumer.motor)
            flow += consumer.sign * consumer.motor->consumedFlow();
    }
    return flow;
}

// Open centre: pump pressure at which the centre gallery and metering orifices together take the
// pump's flow, by bisection (monotonic in pressure); past the main relief the rest goes over it.
void HydraulicSystem::allocateOpenCentre(Pump& pump, u32 index, f32 theoretical)
{
    const HydraulicPumpParameters& p = pump.parameters;
    PumpState& state = pump.state;
    const EngineParameters& engine = mEngine.parameters();
    // Gallery sized for the pump's flow at the engine's top working speed, where it drops neutralPressure.
    const f32 nominalFlow = p.displacement * kCubicCentimetre * p.driveRatio * engine.limiterRpm / 60.0f *
                            p.volumetricEfficiency;
    f32 bypassOpening = 1.0f;
    for (Section& section : mSections)
    {
        if (section.pump != index)
            continue;
        const f32 command = meteredCommand(section);
        bypassOpening *= 1.0f - fabsf(command);
        section.state.loadPressure = command != 0.0f ? loadPressure(section, signOf(command)) : 0.0f;
    }
    const f32 bypassGain = nominalFlow / sqrtf(p.neutralPressure) * bypassOpening;
    auto delivered = [&](f32 pressure, bool write) -> f32
    {
        f32 total = bypassGain * sqrtf(Math::max(pressure, 0.0f));
        if (write)
            state.bypassFlow = total / kLitresPerMinute;
        for (Section& section : mSections)
        {
            if (section.pump != index)
                continue;
            const f32 command = meteredCommand(section);
            f32 flow = 0.0f;
            if (command != 0.0f)
            {
                const f32 gain = section.parameters.ratedFlow * kLitresPerMinute / sqrtf(p.meteringPressure);
                flow = gain * fabsf(command) * sqrtf(Math::max(pressure - section.state.loadPressure, 0.0f));
            }
            total += flow;
            if (write)
            {
                section.state.demandedFlow = command * section.parameters.ratedFlow;
                section.state.flow = signOf(command) * flow / kLitresPerMinute;
            }
        }
        return total;
    };
    f32 pressure = p.maxPressure;
    if (delivered(p.maxPressure, false) >= theoretical)
    {
        f32 low = 0.0f;
        f32 high = p.maxPressure;
        for (u32 i = 0; i < 48; ++i)
        {
            const f32 middle = 0.5f * (low + high);
            if (delivered(middle, false) < theoretical)
                low = middle;
            else
                high = middle;
        }
        pressure = 0.5f * (low + high);
    }
    const f32 taken = delivered(pressure, true);
    state.pressure = pressure;
    state.flow = theoretical / kLitresPerMinute;
    state.maxFlow = state.flow;
    state.availableFlow = state.flow;
    // Flow conservation: whatever the gallery and orifices do not take at relief pressure goes over it.
    state.reliefFlow = Math::max(theoretical - taken, 0.0f) / kLitresPerMinute;
    if (pressure < p.maxPressure)
    {
        state.bypassFlow += (theoretical - taken) / kLitresPerMinute;
        state.reliefFlow = 0.0f;
    }
    state.displacementFraction = theoretical > 0.0f ? 1.0f : 0.0f;
    state.powerLimited = false;
    state.saturated = state.reliefFlow > 0.0f;
}

// Load sensing with flow sharing: the pump holds the margin above the highest load; sections get
// their spool's flow, priorities first, all cut back by one factor when the pump cannot give them all.
void HydraulicSystem::allocateLoadSensing(Pump& pump, u32 index, f32 theoretical, f32 step)
{
    const HydraulicPumpParameters& p = pump.parameters;
    PumpState& state = pump.state;
    f32 highest = 0.0f;
    bool working = false;
    u32 topPriority = 0;
    for (Section& section : mSections)
    {
        if (section.pump != index)
            continue;
        const f32 command = meteredCommand(section);
        section.state.demandedFlow = command * section.parameters.ratedFlow;
        section.state.flow = 0.0f;
        section.state.loadPressure = command != 0.0f ? loadPressure(section, signOf(command)) : 0.0f;
        if (command != 0.0f)
        {
            working = true;
            highest = Math::max(highest, section.state.loadPressure);
            topPriority = Math::max(topPriority, section.parameters.priority);
        }
    }
    const f32 pressure = working ? Math::min(Math::max(highest + p.marginPressure, p.standbyPressure), p.maxPressure)
                                 : p.standbyPressure;
    f32 available = theoretical;
    state.powerLimited = false;
    f32 powerLimit = p.powerLimit;
    if (powerLimit > 0.0f && p.antiStallDroop > 0.0f)
    {
        const f32 droop = mSpeedSetting - mEngine.rpm();
        powerLimit *= Math::clamp(2.0f - droop / p.antiStallDroop, 0.0f, 1.0f);
    }
    if (p.powerLimit > 0.0f && pressure > 0.0f && powerLimit / pressure < available)
    {
        available = powerLimit / pressure;
        state.powerLimited = true;
    }
    // The swash plate follows the regulator with its lag, never past what the speed allows.
    if (p.responseTime > 0.0f && pump.regulated)
        pump.regulatedFlow += (available - pump.regulatedFlow) * (1.0f - expf(-step / p.responseTime));
    else
        pump.regulatedFlow = available;
    pump.regulated = true;
    available = Math::clamp(pump.regulatedFlow, 0.0f, theoretical);
    state.maxFlow = theoretical / kLitresPerMinute;
    state.availableFlow = available / kLitresPerMinute;

    // Priority levels from the top down; each shares what is left in proportion to its demand.
    f32 remaining = available / kLitresPerMinute;
    f32 delivered = 0.0f;
    f32 demandedTotal = 0.0f;
    bool saturated = false;
    for (s32 level = static_cast<s32>(topPriority); level >= 0; --level)
    {
        f32 demand = 0.0f;
        for (const Section& section : mSections)
            if (section.pump == index && section.parameters.priority == static_cast<u32>(level))
            {
                // A dead-headed section (consumer at its end stop) takes nothing: the pressure
                // cut-off. One merely on its relief (accelerating or stalled under a load) keeps
                // its flow and dumps what its consumer cannot take over the relief.
                const f32 headroom = section.state.loadPressure >= kBlocked ? 0.0f : 1.0f;
                demand += fabsf(section.state.demandedFlow) * headroom;
            }
        demandedTotal += demand;
        if (demand <= 0.0f)
            continue;
        const f32 share = demand > remaining ? remaining / demand : 1.0f;
        saturated = saturated || demand > remaining;
        for (Section& section : mSections)
            if (section.pump == index && section.parameters.priority == static_cast<u32>(level))
            {
                const f32 headroom = section.state.loadPressure >= kBlocked ? 0.0f : 1.0f;
                section.state.flow = section.state.demandedFlow * headroom * share;
                delivered += fabsf(section.state.flow);
            }
        remaining = Math::max(remaining - demand * share, 0.0f);
    }
    state.saturated = saturated;
    state.powerLimited = state.powerLimited && saturated;
    state.flow = delivered;
    state.pressure = pressure;
    state.reliefFlow = 0.0f;
    state.bypassFlow = 0.0f;
    state.displacementFraction = theoretical > 0.0f ? Math::clamp(delivered * kLitresPerMinute / theoretical, 0.0f, 1.0f)
                                                    : 0.0f;
    (void)demandedTotal;
}

void HydraulicSystem::feed(Section& section, f32 pressureLimit, f32 step)
{
    const bool open = meteredCommand(section) != 0.0f;
    const f32 flow = section.state.flow;
    // Cylinders share by the area they fill (identical ones move as one); anything else equally.
    f32 total = 0.0f;
    bool allCylinders = true;
    for (u32 c = 0; c < section.consumerCount; ++c)
    {
        const Consumer& consumer = section.consumers[c];
        if (consumer.cylinder)
            total += flow * consumer.sign >= 0.0f ? consumer.cylinder->capArea() : consumer.cylinder->annulusArea();
        else
            allCylinders = false;
    }
    for (u32 c = 0; c < section.consumerCount; ++c)
    {
        const Consumer& consumer = section.consumers[c];
        f32 share = 1.0f / static_cast<f32>(section.consumerCount);
        if (allCylinders && total > 0.0f)
            share = (flow * consumer.sign >= 0.0f ? consumer.cylinder->capArea() : consumer.cylinder->annulusArea()) /
                    total;
        const f32 limit = open ? Math::min(pressureLimit, section.parameters.portRelief) : 0.0f;
        if (consumer.cylinder)
        {
            consumer.cylinder->setFlow(flow * share * consumer.sign);
            consumer.cylinder->setPressureLimit(limit);
        }
        else if (consumer.motor)
        {
            consumer.motor->setFlow(flow * share * consumer.sign);
            consumer.motor->setPressureLimit(limit);
            consumer.motor->update(step);
        }
    }
}

void HydraulicSystem::update(f32 step)
{
    if (!(step > 0.0f) || !isfinite(step))
        return;

    for (Section& section : mSections)
    {
        const f32 tau = section.parameters.responseTime;
        if (tau > 0.0f)
            section.state.spool += (section.command - section.state.spool) * (1.0f - expf(-step / tau));
        else
            section.state.spool = section.command;
        section.state.consumedFlow = consumedFlow(section);
        section.state.reliefFlow = Math::max(fabsf(section.state.flow) - fabsf(section.state.consumedFlow), 0.0f);
        if (section.state.reliefFlow < 1.0e-3f * fabsf(section.state.flow) + 1.0e-3f)
            section.state.reliefFlow = 0.0f;
    }

    const f32 omega = mEngine.running() ? mEngine.angularVelocity() : 0.0f;
    mEngineLoad = 0.0f;
    for (u32 i = 0; i < mPumps.size(); ++i)
    {
        Pump& pump = mPumps[i];
        const HydraulicPumpParameters& p = pump.parameters;
        const f32 revolutions = omega * p.driveRatio / Math::two_pi<f32>();
        const f32 theoretical = p.displacement * kCubicCentimetre * revolutions * p.volumetricEfficiency;
        if (p.circuit == HydraulicCircuit::OpenCentre)
            allocateOpenCentre(pump, i, theoretical);
        else
            allocateLoadSensing(pump, i, theoretical, step);
        PumpState& state = pump.state;
        state.hydraulicPower = state.pressure * state.flow * kLitresPerMinute;
        // Shaft torque of the displacement in use: D_eff p / (2 pi eta_m).
        state.shaftTorque = state.displacementFraction * p.displacement * kCubicCentimetre * state.pressure /
                            (Math::two_pi<f32>() * p.mechanicalEfficiency);
        mEngineLoad += state.shaftTorque * p.driveRatio;
    }

    for (Section& section : mSections)
        feed(section, mPumps[section.pump].parameters.maxPressure, step);

    // The engine: governor, combustion, friction and the pumps' load (held over the step; the
    // governor and friction are integrated in substeps).
    const EngineParameters& engine = mEngine.parameters();
    const f32 inertia = engine.inertia;
    const f32 h = step / static_cast<f32>(kEngineSubsteps);
    for (u32 sub = 0; sub < kEngineSubsteps; ++sub)
    {
        const f32 rpm = mEngine.rpm();
        mThrottle = Math::clamp((mSpeedSetting + kGovernorBand - rpm) / kGovernorBand, 0.0f, 1.0f);
        const f32 combustion = mEngine.update(mThrottle, mStarter);
        const f32 w = mEngine.angularVelocity();
        // Friction and the pumps only resist: never turn the crank backwards within a substep.
        const f32 resisting = mEngine.frictionTorque(rpm) + mEngineLoad;
        f32 next = w + h * combustion / inertia;
        const f32 brake = h * resisting / inertia;
        next = next > brake ? next - brake : 0.0f;
        mEngine.setAngularVelocity(next);
    }
}

} // namespace kinetix::Physics
