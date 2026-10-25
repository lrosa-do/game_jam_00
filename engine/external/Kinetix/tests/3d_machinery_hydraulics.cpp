#include "check.h"
#include "actuator_rig.h"

#include <actuators/HydraulicCylinder.h>
#include <actuators/HydraulicMotor.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/SliderJoint.h>
#include <machinery/HydraulicSystem.h>

using namespace actuator_test;

namespace
{

constexpr f32 kLpm = 1.0f / 60000.0f;

EngineParameters machineDiesel()
{
    // A 110 kW machine diesel: 650 N m from 1400 rpm, rated at 2000 rpm,
    // high idle 2200.
    EngineParameters e = EngineParameters::diesel(110.0e3f, 2000.0f, 650.0f, 1400.0f, 1600.0f, 900.0f, 2200.0f);
    e.inertia = 2.5f;
    return e;
}

struct Lift
{
    RigidBody* mass = nullptr;
    SliderJoint* guide = nullptr;
    HydraulicCylinder* cylinder = nullptr;
};

struct Bench
{
    Rig rig;
    RigidBody* base = nullptr;
    Lift lifts[4];
    u32 liftCount = 0;

    Bench() : rig(60.0f, true)
    {
        base = &rig.base(KM::vec3(0.0f, -0.5f, 0.0f));
    }
    ~Bench()
    {
        for (u32 i = 0; i < liftCount; ++i)
        {
            rig.world.removeJoint(lifts[i].cylinder);
            rig.world.removeJoint(lifts[i].guide);
            delete lifts[i].cylinder;
            delete lifts[i].guide;
        }
    }
    Lift& lift(f32 x, f32 kilograms, f32 bore, f32 rod, f32 stroke = 2.0f)
    {
        Lift& l = lifts[liftCount++];
        const KM::vec3 top(x, 1.0f, 0.0f);
        l.mass = &rig.box(top + KM::vec3(0.0f, 0.2f, 0.0f), KM::vec3(0.2f), kilograms);
        l.guide = new SliderJoint(*base, *l.mass, top, KM::vec3(0.0f, 1.0f, 0.0f));
        rig.world.addJoint(l.guide);
        HydraulicCylinderParameters p;
        p.boreDiameter = bore;
        p.rodDiameter = rod;
        p.stroke = stroke;
        p.initialStroke = 0.2f;
        p.cushionLength = 0.0f;
        p.reliefPressure = 35.0e6f;
        l.cylinder = new HydraulicCylinder(*base, KM::vec3(x, -0.4f, 0.0f), *l.mass, top, p);
        rig.world.addJoint(l.cylinder);
        return l;
    }
    void run(HydraulicSystem& system, f32 seconds)
    {
        const int steps = static_cast<int>(seconds / rig.step + 0.5f);
        for (int i = 0; i < steps; ++i)
        {
            system.update(rig.step);
            rig.world.step(rig.step);
        }
    }
};

f32 strokeRate(Bench& bench, HydraulicSystem& system, const Lift& lift, f32 seconds)
{
    const f32 before = lift.cylinder->stroke();
    bench.run(system, seconds);
    return (lift.cylinder->stroke() - before) / seconds;
}

void loadSensingMetersFlow()
{
    Bench bench;
    Lift& light = bench.lift(0.0f, 500.0f, 0.10f, 0.05f);
    Lift& heavy = bench.lift(2.0f, 8000.0f, 0.12f, 0.06f);
    HydraulicSystem system(machineDiesel());
    HydraulicPumpParameters pump;
    pump.displacement = 140.0f;
    const u32 p = system.addPump(pump);
    ValveSectionParameters section;
    section.ratedFlow = 100.0f;
    section.deadband = 0.0f;
    section.responseTime = 0.0f;
    const u32 a = system.addSection(p, section);
    const u32 b = system.addSection(p, section);
    system.addConsumer(a, *light.cylinder);
    system.addConsumer(b, *heavy.cylinder);
    system.setEngineSpeed(2000.0f);
    bench.run(system, 1.0f);
    system.setSpool(a, 0.6f);
    system.setSpool(b, 0.6f);
    bench.run(system, 0.3f);
    const f32 lightRate = strokeRate(bench, system, light, 0.5f);
    const f32 heavyRate = strokeRate(bench, system, heavy, 0.5f);
    const HydraulicSystem::PumpState& state = system.pump(p);
    const f32 sum = system.section(a).flow + system.section(b).flow;
    printf("load sensing: sections %.2f / %.2f L/min (asked 60), pump %.2f L/min at %.2f MPa (loads %.2f / %.2f "
           "MPa); cylinders %.5f / %.5f m/s vs Q/A %.5f / %.5f; engine %.0f rpm\n",
           double(system.section(a).flow), double(system.section(b).flow), double(state.flow),
           double(state.pressure * 1.0e-6f), double(system.section(a).loadPressure * 1.0e-6f),
           double(system.section(b).loadPressure * 1.0e-6f), double(lightRate), double(heavyRate),
           double(60.0f * kLpm / light.cylinder->capArea()), double(60.0f * kLpm / heavy.cylinder->capArea()),
           double(system.engine().rpm()));
    CHECK_NEAR(system.section(a).flow, 60.0f, 1.0e-3f);
    CHECK_NEAR(system.section(b).flow, 60.0f, 1.0e-3f);
    CHECK_NEAR(state.flow, sum, 1.0e-4f);
    CHECK_NEAR(state.flow, state.reliefFlow + state.bypassFlow + sum, 1.0e-4f);
    CHECK_NEAR(lightRate, 60.0f * kLpm / light.cylinder->capArea(), 1.0e-3f * lightRate);
    CHECK_NEAR(heavyRate, 60.0f * kLpm / heavy.cylinder->capArea(), 1.0e-3f * heavyRate);
    // The pump holds its margin over the heavier load: 8 t on a 120 mm bore.
    const f32 heavyLoad = 8000.0f * kG / heavy.cylinder->capArea();
    CHECK_NEAR(system.section(b).loadPressure, heavyLoad, 0.01f * heavyLoad);
    CHECK_NEAR(state.pressure, heavyLoad + pump.marginPressure, 0.01f * heavyLoad);
    // Retracting meters by the annulus.
    system.setSpool(a, -0.6f);
    system.setSpool(b, 0.0f);
    bench.run(system, 0.3f);
    const f32 down = strokeRate(bench, system, light, 0.3f);
    CHECK_NEAR(down, -60.0f * kLpm / light.cylinder->annulusArea(), 1.0e-3f * fabsf(down));
}

void flowSharingUnderSaturation()
{
    for (int priority = 0; priority < 2; ++priority)
    {
        Bench bench;
        Lift& l0 = bench.lift(0.0f, 300.0f, 0.10f, 0.05f);
        Lift& l1 = bench.lift(2.0f, 3000.0f, 0.10f, 0.05f);
        Lift& l2 = bench.lift(4.0f, 1000.0f, 0.10f, 0.05f);
        HydraulicSystem system(machineDiesel());
        HydraulicPumpParameters pump;
        pump.displacement = 105.0f;
        const u32 p = system.addPump(pump);
        ValveSectionParameters section;
        section.ratedFlow = 100.0f;
        section.deadband = 0.0f;
        section.responseTime = 0.0f;
        const u32 s0 = system.addSection(p, section);
        section.priority = priority ? 1u : 0u;
        const u32 s1 = system.addSection(p, section);
        section.priority = 0;
        const u32 s2 = system.addSection(p, section);
        system.addConsumer(s0, *l0.cylinder);
        system.addConsumer(s1, *l1.cylinder);
        system.addConsumer(s2, *l2.cylinder);
        system.setEngineSpeed(2000.0f);
        bench.run(system, 1.0f);
        system.setSpool(s0, 1.0f);
        system.setSpool(s1, 1.0f);
        system.setSpool(s2, 0.5f);
        bench.run(system, 0.5f);
        const HydraulicSystem::PumpState& state = system.pump(p);
        const f32 f0 = system.section(s0).flow;
        const f32 f1 = system.section(s1).flow;
        const f32 f2 = system.section(s2).flow;
        const f32 rate = strokeRate(bench, system, l1, 0.2f);
        printf("flow sharing%s: pump %.2f of %.2f L/min available; sections %.2f / %.2f / %.2f (asked 100 / 100 / "
               "50); section 1 moves at %.5f m/s = %.2f L/min\n",
               priority ? " (section 1 priority)" : "", double(state.flow), double(state.availableFlow), double(f0),
               double(f1), double(f2), double(rate), double(rate * l1.cylinder->capArea() / kLpm));
        CHECK(state.saturated);
        CHECK_NEAR(state.flow, state.availableFlow, 1.0e-3f);
        CHECK_NEAR(f0 + f1 + f2, state.flow, 1.0e-3f);
        if (!priority)
        {
            CHECK_NEAR(f0 / 100.0f, f1 / 100.0f, 1.0e-5f);
            CHECK_NEAR(f2 / 50.0f, f0 / 100.0f, 1.0e-5f);
        }
        else
        {
            CHECK_NEAR(f1, 100.0f, 1.0e-3f);
            CHECK_NEAR(f0 / 100.0f, f2 / 50.0f, 1.0e-5f);
        }
        CHECK_NEAR(rate * l1.cylinder->capArea() / kLpm, f1, 0.01f * f1);
    }
}

void openCentreFavoursLightLoad()
{
    Bench bench;
    Lift& light = bench.lift(0.0f, 200.0f, 0.10f, 0.05f);
    Lift& heavy = bench.lift(2.0f, 12000.0f, 0.10f, 0.05f);
    HydraulicSystem system(machineDiesel());
    HydraulicPumpParameters pump;
    pump.circuit = HydraulicCircuit::OpenCentre;
    pump.displacement = 60.0f;
    const u32 p = system.addPump(pump);
    ValveSectionParameters section;
    section.ratedFlow = 120.0f;
    section.deadband = 0.0f;
    section.responseTime = 0.0f;
    const u32 a = system.addSection(p, section);
    const u32 b = system.addSection(p, section);
    system.addConsumer(a, *light.cylinder);
    system.addConsumer(b, *heavy.cylinder);
    system.setEngineSpeed(2000.0f);
    bench.run(system, 1.0f);
    const HydraulicSystem::PumpState& state = system.pump(p);
    printf("open centre, neutral: %.2f L/min through the gallery at %.3f MPa\n", double(state.bypassFlow),
           double(state.pressure * 1.0e-6f));
    CHECK_NEAR(state.bypassFlow, state.flow, 1.0e-3f);
    system.setSpool(a, 1.0f);
    system.setSpool(b, 1.0f);
    bench.run(system, 0.5f);
    const f32 fa = system.section(a).flow;
    const f32 fb = system.section(b).flow;
    printf("open centre, both open: pump %.2f L/min at %.2f MPa; light %.2f, heavy %.2f (load %.2f MPa), gallery "
           "%.2f, relief %.2f\n",
           double(state.flow), double(state.pressure * 1.0e-6f), double(fa), double(fb),
           double(system.section(b).loadPressure * 1.0e-6f), double(state.bypassFlow), double(state.reliefFlow));
    CHECK_NEAR(state.flow, fa + fb + state.bypassFlow + state.reliefFlow, 1.0e-3f * state.flow);
    CHECK(fa > 0.9f * state.flow);
    CHECK(fb < 0.05f * state.flow);
    const f32 rate = strokeRate(bench, system, light, 0.2f);
    CHECK_NEAR(rate * light.cylinder->capArea() / kLpm, fa, 0.02f * fa);
}

void reliefCapsPressure()
{
    for (int circuit = 0; circuit < 2; ++circuit)
    {
        Bench bench;
        Lift& lift = bench.lift(0.0f, 1000.0f, 0.10f, 0.05f, 0.4f);
        HydraulicSystem system(machineDiesel());
        HydraulicPumpParameters pump;
        pump.circuit = circuit ? HydraulicCircuit::OpenCentre : HydraulicCircuit::LoadSensing;
        pump.displacement = 50.0f;
        pump.maxPressure = 20.0e6f;
        const u32 p = system.addPump(pump);
        ValveSectionParameters section;
        section.ratedFlow = 100.0f;
        section.responseTime = 0.0f;
        const u32 s = system.addSection(p, section);
        system.addConsumer(s, *lift.cylinder);
        system.setEngineSpeed(2000.0f);
        system.setSpool(s, 1.0f);
        bench.run(system, 2.0f);
        const HydraulicSystem::PumpState& state = system.pump(p);
        // At the stop the cylinder pushes against it through the column;
        // the force the row may push with is the relief's.
        const f32 limit = lift.cylinder->effectivePressureLimit();
        printf("%s at the stop: pump %.3f MPa, cylinder limit %.3f MPa, stroke %.4f of %.4f; relief %.2f of %.2f "
               "L/min, displacement %.3f\n",
               circuit ? "open centre" : "load sensing", double(state.pressure * 1.0e-6f), double(limit * 1.0e-6f),
               double(lift.cylinder->stroke()), double(lift.cylinder->parameters().stroke), double(state.reliefFlow),
               double(state.flow), double(state.displacementFraction));
        CHECK_NEAR(state.pressure, pump.maxPressure, 1.0f);
        CHECK_NEAR(limit, pump.maxPressure, 1.0f);
        CHECK_NEAR(lift.cylinder->stroke(), 0.4f, 2.0e-3f);
        CHECK_NEAR(system.section(s).flow, 0.0f, 1.0e-3f);
        if (circuit)
        {
            CHECK_NEAR(state.reliefFlow, state.flow, 1.0e-3f);
        }
        else
        {
            CHECK(state.displacementFraction < 1.0e-3f);
        }
        // Against a load beyond the relief: 50 t on the 100 mm bore needs
        // 62 MPa; the pump gives 20 and the load comes down through the
        // relief, the cylinder holding exactly p_max * A.
        bench.lift(3.0f, 50000.0f, 0.10f, 0.05f);
        Lift& big = bench.lifts[bench.liftCount - 1];
        const u32 s2 = system.addSection(p, section);
        system.addConsumer(s2, *big.cylinder);
        system.setSpool(s, 0.0f);
        system.setSpool(s2, 1.0f);
        bench.run(system, 1.0f);
        printf("   50 t load: cylinder pushes %.0f N (p_max A = %.0f), sank from 0.2 to %.4f m\n",
               double(big.cylinder->force()), double(pump.maxPressure * big.cylinder->capArea()),
               double(big.cylinder->stroke()));
        CHECK_NEAR(big.cylinder->force(), pump.maxPressure * big.cylinder->capArea(),
                   1.0e-3f * pump.maxPressure * big.cylinder->capArea());
        CHECK(big.cylinder->stroke() < 0.01f);
    }
}

// Relief torque D p / (2 pi eta_m) between rated and peak torque lugs the engine down its droop to where
// the curve meets the load; beyond the peak it stalls.
void engineLugsAndStalls()
{
    const EngineParameters diesel = machineDiesel();
    const Engine reference(diesel);
    for (int variant = 0; variant < 3; ++variant)
    {
        Bench bench;
        // Open centre against the end stop (the pump at its relief); load
        // sensing lifting 20 t (25 MPa) while it moves - at the stop its
        // pump would destroke.
        Lift& lift = variant < 2 ? bench.lift(0.0f, 1000.0f, 0.10f, 0.05f, 0.3f)
                                 : bench.lift(0.0f, 20000.0f, 0.10f, 0.05f, 2.0f);
        HydraulicSystem system(diesel);
        HydraulicPumpParameters pump;
        pump.maxPressure = 30.0e6f;
        pump.mechanicalEfficiency = 0.9f;
        f32 displacement = 0.0f;
        if (variant == 0)
        {
            // 560 N m at the relief: above the 525 N m rated torque, below
            // the 650 N m peak.
            pump.circuit = HydraulicCircuit::OpenCentre;
            displacement = 560.0f * (2.0f * kPi) * 0.9f / 30.0e6f * 1.0e6f;
        }
        else if (variant == 1)
        {
            // 780 N m: beyond anything the engine makes.
            pump.circuit = HydraulicCircuit::OpenCentre;
            displacement = 780.0f * (2.0f * kPi) * 0.9f / 30.0e6f * 1.0e6f;
        }
        else
        {
            // Load sensing, 160 L/min asked at 27 MPa: 72 kW of oil; the
            // regulator holds it to 60 kW.
            pump.circuit = HydraulicCircuit::LoadSensing;
            pump.powerLimit = 60.0e3f;
            displacement = 140.0f;
        }
        pump.displacement = displacement;
        const u32 p = system.addPump(pump);
        ValveSectionParameters section;
        section.ratedFlow = 160.0f;
        section.responseTime = 0.0f;
        section.portRelief = 40.0e6f;
        const u32 s = system.addSection(p, section);
        system.addConsumer(s, *lift.cylinder);
        system.setEngineSpeed(2000.0f);
        bench.run(system, 1.0f);
        const f32 freeRpm = system.engine().rpm();
        system.setSpool(s, 1.0f);
        f32 lowest = freeRpm;
        const int steps = variant < 2 ? 6 * 60 : 3 * 60;
        f32 flowLow = 1.0e9f;
        f32 flowHigh = 0.0f;
        for (int i = 0; i < steps; ++i)
        {
            bench.run(system, bench.rig.step);
            lowest = fminf(lowest, system.engine().rpm());
            if (i > steps / 2)
            {
                flowLow = fminf(flowLow, system.pump(p).flow);
                flowHigh = fmaxf(flowHigh, system.pump(p).flow);
            }
        }
        const HydraulicSystem::PumpState& state = system.pump(p);
        const f32 loadTorque = system.engineLoadTorque();
        printf("%s: %.0f rpm free, %.0f rpm loaded (lowest %.0f), running %d; pump %.1f MPa x %.1f L/min = %.1f kW, "
               "%.1f N m (curve there %.1f)\n",
               variant == 0 ? "open centre, 560 N m" : (variant == 1 ? "open centre, 780 N m" : "load sensing, 60 kW limit"),
               double(freeRpm), double(system.engine().rpm()), double(lowest), int(system.engineRunning()),
               double(state.pressure * 1.0e-6f), double(state.flow), double(state.hydraulicPower * 1.0e-3f),
               double(loadTorque), double(reference.fullLoadTorque(system.engine().rpm())));
        CHECK(freeRpm > 2050.0f);
        if (variant == 0)
        {
            CHECK(system.engineRunning());
            CHECK(system.engine().rpm() < 1950.0f);
            CHECK(system.engine().rpm() > diesel.idleRpm);
            CHECK_NEAR(reference.fullLoadTorque(system.engine().rpm()), loadTorque, 0.03f * loadTorque);
        }
        else if (variant == 1)
        {
            CHECK(!system.engineRunning());
            CHECK(state.flow < 1.0f);
        }
        else
        {
            CHECK(system.engineRunning());
            CHECK(state.powerLimited);
            CHECK(state.hydraulicPower <= pump.powerLimit * 1.001f);
            CHECK(system.engine().rpm() > 1950.0f);
            printf("   regulated flow over the last half %.2f..%.2f L/min\n", double(flowLow), double(flowHigh));
            CHECK(flowHigh - flowLow < 0.01f * flowHigh);
            CHECK_NEAR(state.pressure, 20000.0f * kG / lift.cylinder->capArea() + pump.marginPressure, 0.2e6f);
        }
    }
}

// Two load-sensing pumps, 45 kW each (90 of the engine's 110 kW at rated speed), lifting 20 t from idle,
// where the engine makes less torque than the limits ask (45 kW at 900 rpm is 480 N m a pump).
void antiStallSavesTheEngine()
{
    for (int variant = 0; variant < 2; ++variant)
    {
        Bench bench;
        Lift& a = bench.lift(-1.0f, 20000.0f, 0.10f, 0.05f, 2.0f);
        Lift& b = bench.lift(1.0f, 20000.0f, 0.10f, 0.05f, 2.0f);
        HydraulicSystem system(machineDiesel());
        HydraulicPumpParameters pump;
        pump.circuit = HydraulicCircuit::LoadSensing;
        pump.displacement = 120.0f;
        pump.maxPressure = 30.0e6f;
        pump.powerLimit = 45.0e3f;
        pump.antiStallDroop = variant == 0 ? 0.0f : 150.0f;
        ValveSectionParameters section;
        section.ratedFlow = 200.0f;
        section.responseTime = 0.0f;
        section.portRelief = 40.0e6f;
        const u32 pa = system.addPump(pump);
        const u32 pb = system.addPump(pump);
        const u32 sa = system.addSection(pa, section);
        const u32 sb = system.addSection(pb, section);
        system.addConsumer(sa, *a.cylinder);
        system.addConsumer(sb, *b.cylinder);
        system.setEngineSpeed(2000.0f);
        const f32 start = system.engine().rpm();
        system.setSpool(sa, 1.0f);
        system.setSpool(sb, 1.0f);
        f32 lowest = start;
        for (int i = 0; i < 3 * 60; ++i)
        {
            bench.run(system, bench.rig.step);
            lowest = fminf(lowest, system.engine().rpm());
        }
        const f32 power = system.pump(pa).hydraulicPower + system.pump(pb).hydraulicPower;
        printf("anti-stall %s: from %.0f rpm, lowest %.0f, now %.0f rpm, running %d; pumps %.1f kW\n",
               variant == 0 ? "off" : "on ", double(start), double(lowest), double(system.engine().rpm()),
               int(system.engineRunning()), double(power * 1.0e-3f));
        if (variant == 0)
            CHECK(!system.engineRunning());
        else
        {
            CHECK(system.engineRunning());
            CHECK(system.engine().rpm() > 1950.0f);
            CHECK_NEAR(power, 2.0f * pump.powerLimit, 0.01f * 2.0f * pump.powerLimit);
        }
    }
}

// A hydraulic motor on a hinge: speed = Q eta_v / (D i); against a load it
// cannot lift it stalls at dp D i eta_m / (2 pi); with the valve closed the
// brake holds.
void hydraulicMotorOnAHinge()
{
    Rig rig(60.0f, false);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& turntable = rig.box(KM::vec3(0.0f, 1.0f, 0.0f), KM::vec3(1.5f, 0.3f, 1.5f), 4000.0f);
    HingeJoint hinge(base, turntable, KM::vec3(0.0f, 0.7f, 0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
    rig.world.addJoint(&hinge);
    HydraulicMotorParameters p;
    p.displacement = 100.0f;
    p.volumetricEfficiency = 0.95f;
    p.mechanicalEfficiency = 0.9f;
    p.gearRatio = 120.0f;
    p.reliefPressure = 25.0e6f;
    p.maxFlow = 120.0f;
    p.holdingBrake = 5.0e4f;
    HydraulicMotor motor(hinge.drive(), p);
    motor.setValve(0.5f);
    for (int i = 0; i < 4 * 60; ++i)
    {
        motor.update(rig.step);
        rig.world.step(rig.step);
    }
    const f32 expected = 60.0f * kLpm * 0.95f / (100.0e-6f / (2.0f * kPi) * 120.0f);
    printf("hydraulic motor: %.5f rad/s at 60 L/min (Q eta_v / (D i) = %.5f), consuming %.3f L/min\n",
           double(hinge.currentAngularVelocity()), double(expected), double(motor.consumedFlow()));
    CHECK_NEAR(hinge.currentAngularVelocity(), expected, 1.0e-4f * expected);
    CHECK_NEAR(motor.consumedFlow(), 60.0f, 0.01f);
    const f32 stall = 25.0e6f * 100.0e-6f / (2.0f * kPi) * 120.0f * 0.9f;
    for (int i = 0; i < 2 * 60; ++i)
    {
        turntable.addTorque(KM::vec3(0.0f, -1.5f * stall, 0.0f));
        motor.update(rig.step);
        rig.world.step(rig.step);
    }
    printf("   against %.0f N m it gives %.0f N m (dp D i eta_m / 2pi = %.0f), turning %.4f rad/s\n",
           double(1.5f * stall), double(motor.outputTorque()), double(stall), double(hinge.currentAngularVelocity()));
    CHECK_NEAR(motor.outputTorque(), stall, 1.0e-3f * stall);
    CHECK(motor.reliefOpen());
    motor.setValve(0.0f);
    for (int i = 0; i < 60; ++i)
    {
        motor.update(rig.step);
        rig.world.step(rig.step);
    }
    const f32 held = hinge.currentAngle();
    for (int i = 0; i < 2 * 60; ++i)
    {
        turntable.addTorque(KM::vec3(0.0f, 0.9f * (stall + p.holdingBrake), 0.0f));
        motor.update(rig.step);
        rig.world.step(rig.step);
    }
    CHECK(motor.braking());
    CHECK_NEAR(hinge.currentAngle(), held, 1.0e-4f);
    rig.world.removeJoint(&hinge);
}

}

int main()
{
    loadSensingMetersFlow();
    flowSharingUnderSaturation();
    openCentreFavoursLightLoad();
    reliefCapsPressure();
    engineLugsAndStalls();
    antiStallSavesTheEngine();
    hydraulicMotorOnAHinge();
    return TEST_RESULT();
}
