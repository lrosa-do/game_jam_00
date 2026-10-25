#include "check.h"

#include <dynamics/RigidBody.h>
#include <rotorcraft/Helicopter.h>
#include <rotorcraft/HelicopterAutopilot.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kDeg = 3.14159265358979f / 180.0f;
constexpr f32 kStep = 1.0f / 60.0f;

struct Flight
{
    RigidBody body;
    Helicopter helicopter;
    LightHelicopterLayout layout;
    HelicopterAutopilot autopilot;

    Flight(f32 height, f32 airspeed)
    {
        layout = buildLightHelicopter(helicopter, body);
        body.setPosition(KM::vec3(0.0f, height, 0.0f));
        body.setAcceleration(KM::vec3(0.0f, -9.81f, 0.0f));
        HelicopterTrimTarget target;
        target.airspeed = airspeed;
        CHECK(helicopter.trim(target).converged);
        autopilot.engage(helicopter);
        autopilot.holdVelocity(body.velocity(), 0.0f);
        autopilot.holdAltitude(height);
    }

    void fly(const HelicopterControls& controls)
    {
        helicopter.setControls(controls);
        helicopter.applyForces(kStep);
        body.integrate(kStep);
    }
};

void governorOnCollectiveSteps(f32 airspeed)
{
    for (f32 step : {0.1f, -0.1f})
    {
        Flight f(500.0f, airspeed);
        const f32 trimmed = f.helicopter.controls().collective;
        f32 lowest = 1.0e9f;
        f32 highest = 0.0f;
        for (int i = 0; i < 15 * 60; ++i)
        {
            HelicopterControls c = f.autopilot.update(f.helicopter, kStep);
            c.collective = i >= 60 ? trimmed + step : trimmed;
            f.fly(c);
            const f32 nr = f.helicopter.telemetry().rotorSpeedPercent;
            lowest = fminf(lowest, nr);
            highest = fmaxf(highest, nr);
        }
        const HelicopterTelemetry& t = f.helicopter.telemetry();
        printf("governor at %.0f m/s, collective %+.2f: rotor %.2f%% to %.2f%%, %.2f%% at 15 s, %.0f kW, "
               "Ng %.1f%%, climbing %.1f m/s\n",
               double(airspeed), double(step), double(lowest), double(highest), double(t.rotorSpeedPercent),
               double(t.powerRequired / 1000.0f), double(t.gasGeneratorPercent), double(t.verticalSpeed));
        CHECK(lowest > 98.0f && highest < 102.0f);
        CHECK_NEAR(t.rotorSpeedPercent, 100.0f, 0.5f);
        if (step > 0.0f)
            CHECK(t.verticalSpeed > 1.0f);
        else
            CHECK(t.verticalSpeed < -1.0f);
    }
}

struct Autorotation
{
    f32 rotorLow = 1.0e9f;
    f32 rotorHigh = 0.0f;
    f32 descent = 0.0f; // mean over the last ten seconds
    f32 airspeed = 0.0f;
    f32 turbineShare = 0.0f; // power turbine speed over the rotor's, at the end
    bool engaged = true;
    f32 engineTorque = 0.0f;
    f32 enginePower = 0.0f;
    f32 flareDescent = 0.0f;  // least descent rate in the flare
    f32 flareRotor = 0.0f;    // highest rotor speed in it
};

Autorotation engineFailure(f32 fixedCollective)
{
    Flight f(1200.0f, 30.0f);
    Autorotation result;
    f32 collective = f.helicopter.controls().collective;
    f32 sum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 30 * 60; ++i)
    {
        HelicopterControls c = f.autopilot.update(f.helicopter, kStep);
        const HelicopterTelemetry& t = f.helicopter.telemetry();
        if (i >= 60)
        {
            c.engineOn = false;
            if (fixedCollective >= 0.0f)
            {
                c.collective = fixedCollective;
            }
            else
            {
                // Down to 0.2 in a second, then rotor speed on the
                // collective: more collective slows it.
                const f32 lowered = fmaxf(0.2f, collective - kStep * 0.5f);
                collective = KM::clamp(lowered + (i > 180 ? 0.04f * kStep * (t.rotorSpeedPercent - 100.0f) : 0.0f), 0.0f, 1.0f);
                c.collective = KM::clamp(collective + 0.01f * (t.rotorSpeedPercent - 100.0f), 0.0f, 1.0f);
            }
        }
        f.fly(c);
        if (i >= 10 * 60)
        {
            result.rotorLow = fminf(result.rotorLow, t.rotorSpeedPercent);
            result.rotorHigh = fmaxf(result.rotorHigh, t.rotorSpeedPercent);
        }
        if (i >= 20 * 60)
        {
            sum += -f.body.velocity().y;
            ++samples;
        }
    }
    const HelicopterTelemetry& t = f.helicopter.telemetry();
    const TransmissionState& drive = f.helicopter.transmission().state();
    result.descent = sum / static_cast<f32>(samples);
    result.airspeed = t.trueAirspeed;
    result.turbineShare = f.helicopter.transmission().turbineSpeed() /
                          (f.helicopter.transmission().description().engineRatio * f.helicopter.transmission().rotorSpeed());
    result.engaged = drive.engaged;
    result.engineTorque = drive.engineTorque;
    result.enginePower = t.enginePower;

    HelicopterAutopilotGains gains;
    const f32 held = f.helicopter.controls().collective;
    result.flareDescent = result.descent;
    result.flareRotor = t.rotorSpeedPercent;
    for (int i = 0; i < 4 * 60; ++i)
    {
        HelicopterControls c = f.helicopter.controls();
        const f32 pitchError = 20.0f * kDeg - t.pitch;
        c.cyclicLongitudinal = KM::clamp(-gains.attitude * pitchError + gains.pitchRate * t.bodyRates.y, -1.0f, 1.0f);
        c.cyclicLateral = KM::clamp(-gains.attitude * t.bank - gains.rollRate * t.bodyRates.x, -1.0f, 1.0f);
        c.collective = held;
        c.engineOn = false;
        f.fly(c);
        result.flareDescent = fminf(result.flareDescent, -f.body.velocity().y);
        result.flareRotor = fmaxf(result.flareRotor, t.rotorSpeedPercent);
    }
    return result;
}

void autorotation()
{
    const Autorotation managed = engineFailure(-1.0f);
    printf("autorotation, rotor on the collective: rotor %.1f%% to %.1f%%, descent %.2f m/s at %.1f m/s, "
           "power turbine at %.0f%% of the rotor's speed, freewheel %s; flare: descent down to %.2f m/s, "
           "rotor up to %.1f%%\n",
           double(managed.rotorLow), double(managed.rotorHigh), double(managed.descent), double(managed.airspeed),
           double(100.0f * managed.turbineShare), managed.engaged ? "engaged" : "overrunning",
           double(managed.flareDescent), double(managed.flareRotor));
    CHECK(managed.rotorLow > 95.0f && managed.rotorHigh < 110.0f);
    // A light helicopter comes down at 1500 to 2000 ft/min at its best
    // autorotation speed.
    CHECK(managed.descent > 8.0f && managed.descent < 15.0f);
    CHECK(managed.airspeed > 25.0f && managed.airspeed < 35.0f);
    CHECK(!managed.engaged);
    CHECK(managed.turbineShare < 0.5f);
    CHECK(managed.engineTorque == 0.0f);
    CHECK(managed.enginePower == 0.0f);
    CHECK(managed.flareDescent < 0.5f * managed.descent);
    CHECK(managed.flareRotor > managed.rotorHigh);

    const Autorotation fixed = engineFailure(0.12f);
    printf("autorotation, collective at 0.12: rotor %.1f%% to %.1f%%, descent %.2f m/s\n", double(fixed.rotorLow),
           double(fixed.rotorHigh), double(fixed.descent));
    CHECK(fixed.rotorLow > 100.0f && fixed.rotorHigh < 118.0f);
    CHECK(fixed.rotorHigh - fixed.rotorLow < 5.0f);
    CHECK(fixed.descent > managed.descent && fixed.descent < 15.0f);
}

void relight()
{
    Flight f(1200.0f, 30.0f);
    bool engagedAgain = false;
    for (int i = 0; i < 40 * 60; ++i)
    {
        HelicopterControls c = f.autopilot.update(f.helicopter, kStep);
        const bool off = i >= 60 && i < 15 * 60;
        if (i >= 60)
            c.engineOn = !off;
        if (off)
            c.collective = 0.2f;
        f.fly(c);
        if (!off && i > 15 * 60 && f.helicopter.transmission().engaged())
            engagedAgain = true;
    }
    const HelicopterTelemetry& t = f.helicopter.telemetry();
    printf("relight: freewheel %s, rotor %.2f%%, Ng %.1f%%, climbing %.2f m/s back to the hold height\n",
           engagedAgain ? "engaged again" : "still overrunning", double(t.rotorSpeedPercent),
           double(t.gasGeneratorPercent), double(t.verticalSpeed));
    CHECK(engagedAgain);
    CHECK_NEAR(t.rotorSpeedPercent, 100.0f, 1.0f);
    CHECK(t.enginePower > 50000.0f);
}

void startOnThePad()
{
    RigidBody body;
    Helicopter helicopter;
    const LightHelicopterLayout layout = buildLightHelicopter(helicopter, body);
    body.setPosition(KM::vec3(0.0f, layout.restHeight, 0.0f));
    body.setAcceleration(KM::vec3(0.0f, -9.81f, 0.0f));
    helicopter.shutDown();
    HelicopterControls c;
    c.collective = 0.0f;
    c.throttle = 0.0f;
    c.engineOn = true;
    f32 idleRotor = 0.0f;
    f32 time100 = -1.0f;
    for (int i = 0; i < 90 * 60; ++i)
    {
        if (i == 40 * 60)
        {
            idleRotor = helicopter.telemetry().rotorSpeedPercent;
            c.throttle = 1.0f;
        }
        helicopter.setControls(c);
        helicopter.applyForces(kStep);
        body.integrate(kStep);
        if (time100 < 0.0f && helicopter.telemetry().rotorSpeedPercent > 99.0f)
            time100 = static_cast<f32>(i) * kStep;
    }
    const HelicopterTelemetry& t = helicopter.telemetry();
    printf("start: rotor %.1f%% at idle after 40 s, 99%% at %.1f s with the throttle open, Ng %.1f%%\n",
           double(idleRotor), double(time100), double(t.gasGeneratorPercent));
    CHECK(idleRotor > 30.0f && idleRotor < 90.0f);
    CHECK(time100 > 40.0f);
    CHECK_NEAR(t.rotorSpeedPercent, 100.0f, 1.0f);
    CHECK(t.onGround);
    CHECK(KM::length(body.velocity()) < 0.05f);
}
}

int main()
{
    governorOnCollectiveSteps(0.0f);
    governorOnCollectiveSteps(40.0f);
    autorotation();
    relight();
    startOnThePad();
    return TEST_RESULT();
}
