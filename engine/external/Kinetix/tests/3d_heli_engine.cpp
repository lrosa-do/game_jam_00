#include "check.h"

#include <dynamics/Atmosphere.h>
#include <rotorcraft/RotorTransmission.h>
#include <rotorcraft/TurboshaftEngine.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;

namespace
{
constexpr f32 kRho = 1.225f;
constexpr f32 kRotorSpeed = 200.0f / 4.6f;

TurboshaftEngine::Description engineDescription()
{
    TurboshaftEngine::Description e;
    e.maxPower = 300000.0f;
    e.ratedTurbineRpm = 6000.0f;
    e.anticipation = 1.2f;
    e.anticipationBias = 0.0f;
    e.governorGain = 8.0f;
    e.governorIntegral = 6.0f;
    return e;
}

RotorTransmission::Description driveDescription(const TurboshaftEngine& engine)
{
    RotorTransmission::Description d;
    d.mainRotorInertia = 500.0f;
    d.tailRotorInertia = 0.9f;
    d.tailRotorRatio = 5.75f;
    d.engineRatio = engine.ratedTurbineSpeed() / kRotorSpeed;
    d.efficiency = 0.97f;
    return d;
}

// A rotor-like load: torque ~ omega^2 at a fixed collective, functions of
// the main rotor speed; the tail rotor's, at its own shaft, takes a tenth of
// the power.
struct Load
{
    f32 coefficient = 0.0f;
    f32 main(f32 omega) const
    {
        return coefficient * omega * omega;
    }
    f32 tail(f32 omega) const
    {
        return 0.1f * coefficient * omega * omega / 5.75f;
    }
};

void gasPower()
{
    TurboshaftEngine engine(engineDescription());
    engine.setGasGenerator(1.0f);
    CHECK_NEAR(engine.gasPower(kRho), 300000.0f, 1.0f);
    engine.setGasGenerator(0.45f);
    CHECK(engine.gasPower(kRho) == 0.0f);
    f32 previous = -1.0f;
    for (f32 ng = 0.5f; ng <= 1.02f; ng += 0.04f)
    {
        engine.setGasGenerator(ng);
        const f32 p = engine.gasPower(kRho);
        CHECK(p > previous);
        previous = p;
    }
    // sigma^0.8 lapse: at 3000 m ISA the density ratio is 0.742.
    engine.setGasGenerator(1.0f);
    const f32 high = Atmosphere::standard(3000.0f).density;
    CHECK_NEAR(engine.gasPower(high) / engine.gasPower(kRho), powf(high / kRho, 0.8f), 1e-4);
    printf("gas power: 300 kW at sea level, %.0f kW at 3000 m\n", double(engine.gasPower(high) / 1000.0f));
}

void freeTurbineLine()
{
    TurboshaftEngine engine(engineDescription());
    engine.setGasGenerator(0.9f);
    const f32 rated = engine.ratedTurbineSpeed();
    const f32 power = engine.gasPower(kRho);
    const f32 windage = 0.01f * 300000.0f / rated;
    // Full gas power at the rated speed, twice the torque stalled, nothing
    // at twice the speed - less the windage.
    CHECK_NEAR(engine.turbineTorque(rated, kRho) * rated, power - windage * rated, 1.0f);
    CHECK_NEAR(engine.turbineTorque(0.0f, kRho), 2.0f * power / rated, 1e-3);
    CHECK_NEAR(engine.turbineTorque(2.0f * rated, kRho), -2.0f * windage, 1e-3);
    const f32 h = 1.0f;
    const f32 numeric = (engine.turbineTorque(rated + h, kRho) - engine.turbineTorque(rated - h, kRho)) / (2.0f * h);
    CHECK_NEAR(engine.turbineTorqueSlope(rated, kRho), numeric, 1e-3);
    engine.setGasGenerator(0.0f);
    CHECK(engine.turbineTorque(rated, kRho) < 0.0f);
}

void spoolLag()
{
    TurboshaftEngine engine(engineDescription());
    engine.setGasGenerator(engine.description().idleGasGenerator);
    engine.setThrottle(1.0f);
    const f32 step = 1.0f / 60.0f;
    f32 time = 0.0f;
    f32 previous = engine.gasGenerator();
    f32 reached95 = -1.0f;
    const f32 idle = engine.description().idleGasGenerator;
    const f32 top = engine.description().maxGasGenerator;
    while (time < 5.0f)
    {
        engine.step(step, 0.5f * engine.ratedTurbineSpeed(), kRho);
        time += step;
        CHECK(engine.gasGenerator() >= previous - 1e-6f);
        previous = engine.gasGenerator();
        if (reached95 < 0.0f && previous > idle + 0.95f * (top - idle))
            reached95 = time;
    }
    printf("spool-up idle to 95%% of topping: %.2f s\n", double(reached95));
    // A few time constants of 0.45 s, slower at the low end.
    CHECK(reached95 > 0.8f && reached95 < 2.5f);
    CHECK(engine.gasGenerator() <= top + 1e-5f);

    engine.setRunning(false);
    for (int i = 0; i < 180; ++i)
        engine.step(step, engine.ratedTurbineSpeed(), kRho);
    CHECK_NEAR(engine.gasGenerator(), top * expf(-3.0f / 3.0f), 0.01);
    for (int i = 0; i < 600; ++i)
        engine.step(step, engine.ratedTurbineSpeed(), kRho);
    CHECK(engine.gasPower(kRho) == 0.0f);
    CHECK(engine.fuelFlow(kRho) == 0.0f);
}

void fuelFlow()
{
    TurboshaftEngine engine(engineDescription());
    engine.setGasGenerator(0.7f);
    const f32 low = engine.fuelFlow(kRho);
    engine.setGasGenerator(1.0f);
    const f32 high = engine.fuelFlow(kRho);
    CHECK(high > low && low > 0.0f);
    // 0.36 kg/kWh at full power plus the idle flow: about 110 kg/h.
    CHECK_NEAR(high * 3600.0f, 0.004f * 3600.0f + 0.36f * 300.0f, 0.5f);
}

struct Drive
{
    TurboshaftEngine engine{engineDescription()};
    RotorTransmission drive;
    Load load;

    Drive()
    {
        drive.setDescription(driveDescription(engine));
        drive.setRotorSpeed(kRotorSpeed);
    }

    void settle(f32 collective)
    {
        engine.setCollective(collective);
        engine.setGasGenerator(0.85f);
        for (int i = 0; i < 60 * 30; ++i)
            step(1.0f / 60.0f);
    }

    const TransmissionState& step(f32 dt)
    {
        const f32 omega = drive.rotorSpeed();
        const TransmissionState& s = drive.step(dt, &engine, kRho, load.main(omega), load.tail(omega));
        engine.step(dt, s.turbineSpeed, kRho);
        return s;
    }
};

void governorHoldsRotorSpeed()
{
    Drive d;
    // About 150 kW at 100%.
    d.load.coefficient = 150000.0f / (kRotorSpeed * kRotorSpeed * kRotorSpeed);
    d.settle(0.6f);
    CHECK_NEAR(d.drive.rotorSpeed(), kRotorSpeed, 0.002 * kRotorSpeed);
    CHECK(d.drive.engaged());

    d.load.coefficient *= 1.4f;
    d.engine.setCollective(0.75f);
    f32 lowest = d.drive.rotorSpeed();
    f32 highest = lowest;
    for (int i = 0; i < 60 * 10; ++i)
    {
        d.step(1.0f / 60.0f);
        lowest = fminf(lowest, d.drive.rotorSpeed());
        highest = fmaxf(highest, d.drive.rotorSpeed());
    }
    printf("governor, +40%% load step: rotor speed between %.2f%% and %.2f%%, %.2f%% after 10 s\n",
           double(100.0f * lowest / kRotorSpeed), double(100.0f * highest / kRotorSpeed),
           double(100.0f * d.drive.rotorSpeed() / kRotorSpeed));
    CHECK(lowest > 0.98f * kRotorSpeed);
    CHECK(highest < 1.02f * kRotorSpeed);
    CHECK_NEAR(d.drive.rotorSpeed(), kRotorSpeed, 0.003 * kRotorSpeed);

    d.load.coefficient /= 1.4f;
    d.engine.setCollective(0.6f);
    highest = d.drive.rotorSpeed();
    for (int i = 0; i < 60 * 10; ++i)
    {
        d.step(1.0f / 60.0f);
        highest = fmaxf(highest, d.drive.rotorSpeed());
    }
    CHECK(highest < 1.02f * kRotorSpeed);

    Drive plain;
    plain.load.coefficient = 150000.0f / (kRotorSpeed * kRotorSpeed * kRotorSpeed);
    TurboshaftEngine::Description e = engineDescription();
    e.anticipation = 0.0f;
    e.anticipationBias = 0.72f;
    plain.engine.setDescription(e);
    plain.settle(0.6f);
    plain.load.coefficient *= 1.4f;
    plain.engine.setCollective(0.75f);
    f32 plainLowest = plain.drive.rotorSpeed();
    for (int i = 0; i < 60 * 10; ++i)
    {
        plain.step(1.0f / 60.0f);
        plainLowest = fminf(plainLowest, plain.drive.rotorSpeed());
    }
    CHECK(plainLowest < lowest);
}

void freewheel()
{
    Drive d;
    d.load.coefficient = 150000.0f / (kRotorSpeed * kRotorSpeed * kRotorSpeed);
    d.settle(0.6f);

    d.engine.setRunning(false);
    d.load.coefficient = 0.0f;
    const f32 driving = -150.0f;
    f32 rotorAt10 = 0.0f;
    for (int i = 0; i < 60 * 20; ++i)
    {
        const TransmissionState& s = d.drive.step(1.0f / 60.0f, &d.engine, kRho, driving, 0.0f);
        d.engine.step(1.0f / 60.0f, s.turbineSpeed, kRho);
        CHECK(s.engineTorque >= 0.0f);
        if (i == 600)
            rotorAt10 = d.drive.rotorSpeed();
    }
    const f32 ratio = d.drive.description().engineRatio;
    printf("freewheel: rotor %.1f%%, turbine %.1f%% of the rotor's speed after 20 s engine off\n",
           double(100.0f * d.drive.rotorSpeed() / kRotorSpeed),
           double(100.0f * d.drive.turbineSpeed() / (ratio * d.drive.rotorSpeed())));
    CHECK(!d.drive.engaged());
    CHECK(d.drive.turbineSpeed() < 0.7f * ratio * d.drive.rotorSpeed());
    // The rotor accelerates on the air's torque less friction; the engine
    // side takes none of it.
    CHECK(d.drive.rotorSpeed() > rotorAt10);

    // Locked together (no freewheel unit) the rotor would have to drag the
    // turbine along: same scenario, turbine still at the rotor's speed.
    Drive locked;
    RotorTransmission::Description noFreewheel = locked.drive.description();
    noFreewheel.freewheel = false;
    locked.drive.setDescription(noFreewheel);
    locked.load.coefficient = 150000.0f / (kRotorSpeed * kRotorSpeed * kRotorSpeed);
    locked.settle(0.6f);
    locked.engine.setRunning(false);
    for (int i = 0; i < 60 * 20; ++i)
    {
        const TransmissionState& s = locked.drive.step(1.0f / 60.0f, &locked.engine, kRho, driving, 0.0f);
        locked.engine.step(1.0f / 60.0f, s.turbineSpeed, kRho);
    }
    CHECK_NEAR(locked.drive.turbineSpeed(), ratio * locked.drive.rotorSpeed(), 1e-3f * locked.drive.turbineSpeed());
    CHECK(locked.drive.rotorSpeed() < d.drive.rotorSpeed());

    // Relight: the turbine spools up and engages when it catches the rotor,
    // and the lock-up conserves angular momentum.
    d.engine.setRunning(true);
    d.engine.setCollective(0.3f);
    bool engagedAgain = false;
    for (int i = 0; i < 60 * 30 && !engagedAgain; ++i)
    {
        const RotorTransmission::Description& desc = d.drive.description();
        const f32 rotorInertia = desc.mainRotorInertia + desc.tailRotorRatio * desc.tailRotorRatio * desc.tailRotorInertia;
        const f32 turbineInertia = d.engine.description().turbineInertia;
        const f32 before = rotorInertia * d.drive.rotorSpeed() + ratio * turbineInertia * d.drive.turbineSpeed();
        const TransmissionState& s = d.drive.step(1.0f / 60.0f, &d.engine, kRho, 0.0f, 0.0f);
        d.engine.step(1.0f / 60.0f, s.turbineSpeed, kRho);
        if (s.engaged)
        {
            engagedAgain = true;
            const f32 after = (rotorInertia + ratio * ratio * turbineInertia) * d.drive.rotorSpeed();
            // Within the torques acting over the one step.
            CHECK_NEAR(after, before, 0.002f * before);
        }
    }
    CHECK(engagedAgain);
}

void degenerate()
{
    TurboshaftEngine engine(engineDescription());
    engine.setThrottle(NAN);
    engine.setCollective(NAN);
    engine.setGasGenerator(NAN);
    engine.step(NAN, NAN, NAN);
    engine.step(1.0f / 60.0f, NAN, NAN);
    CHECK(isfinite(engine.gasGenerator()));
    CHECK(isfinite(engine.turbineTorque(NAN, kRho)));
    RotorTransmission drive(driveDescription(engine));
    drive.setRotorSpeed(NAN);
    CHECK(drive.rotorSpeed() == 0.0f);
    drive.setRotorSpeed(kRotorSpeed);
    drive.step(1.0f / 60.0f, &engine, kRho, NAN, NAN);
    CHECK(isfinite(drive.rotorSpeed()) && isfinite(drive.turbineSpeed()));
    for (int i = 0; i < 60 * 600; ++i)
        drive.step(1.0f / 60.0f, nullptr, kRho, 500.0f, 0.0f);
    CHECK(drive.rotorSpeed() == 0.0f);
}
}

int main()
{
    gasPower();
    freeTurbineLine();
    spoolLag();
    fuelFlow();
    governorHoldsRotorSpeed();
    freewheel();
    degenerate();
    return TEST_RESULT();
}
