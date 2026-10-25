#ifndef KINETIX_TESTS_HELI_STABILITY_H
#define KINETIX_TESTS_HELI_STABILITY_H

#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <rotorcraft/Helicopter.h>
#include <rotorcraft/HelicopterAutopilot.h>

#include <math.h>
#include <stdio.h>

namespace heli_test
{

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

constexpr f32 kPi = 3.14159265358979f;

inline f32 stroke(f32 time, f32 period, f32 phase = 0.0f)
{
    const f32 s = sinf(2.0f * kPi * (time / period + phase));
    return KM::clamp(1.6f * s, -1.0f, 1.0f);
}

struct StabilityResult
{
    bool finite = true;
    f32 worstSpeed = 0.0f;
    f32 worstRate = 0.0f;
    f32 worstRotor = 0.0f; // percent
    f32 lowestRotor = 1.0e9f;
    f32 worstTilt = 0.0f;
    f32 worstFlap = 0.0f;
};

inline bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

inline void track(StabilityResult& r, const RigidBody& body, const Helicopter& helicopter)
{
    const HelicopterTelemetry& t = helicopter.telemetry();
    const bool finite = finiteVec(body.position()) && finiteVec(body.velocity()) &&
                        finiteVec(body.angularVelocity()) && finiteVec(t.force) && finiteVec(t.torque) &&
                        isfinite(t.rotorSpeedPercent) && isfinite(t.powerRequired) && isfinite(t.thrust) &&
                        isfinite(t.inducedVelocity) && isfinite(t.gasGeneratorPercent);
    r.finite = r.finite && finite;
    if (!finite)
        return;
    r.worstSpeed = fmaxf(r.worstSpeed, KM::length(body.velocity()));
    r.worstRate = fmaxf(r.worstRate, KM::length(body.angularVelocity()));
    r.worstRotor = fmaxf(r.worstRotor, t.rotorSpeedPercent);
    r.lowestRotor = fminf(r.lowestRotor, t.rotorSpeedPercent);
    r.worstFlap = fmaxf(r.worstFlap, fmaxf(fabsf(t.longitudinalFlapping), fabsf(t.lateralFlapping)));
}

struct Rig
{
    PhysicsWorld world;
    RigidBody body;
    BoxShape hull{KM::vec3(0.5f)};
    Helicopter helicopter;
    LightHelicopterLayout layout;

    Rig()
    {
        world.setGravity(KM::vec3(0.0f, -9.81f, 0.0f));
        layout = buildLightHelicopter(helicopter, body);
        hull = BoxShape(layout.hullHalfExtents);
        body.setPosition(KM::vec3(0.0f, layout.restHeight, 0.0f));
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &hull;
        world.addBody(entry);
        HelicopterControls idle;
        idle.collective = 0.0f;
        helicopter.setControls(idle);
        helicopter.startRunning();
    }
};

// Full pedal on the ground is about as much yaw torque as skid friction can hold (tail rotor ~1.9 kN x 5.7 m
// against 0.6 x weight x 1.6 m), so at full stroke it may turn on the spot; it must not tip over.
inline void groundStirring(f32 rate, f32 amplitude, StabilityResult& r, f32& moved, f32& turned)
{
    Rig rig;
    const f32 step = 1.0f / rate;
    const int steps = static_cast<int>(30.0f * rate);
    for (int i = 0; i < steps; ++i)
    {
        const f32 time = static_cast<f32>(i) * step;
        HelicopterControls c;
        c.collective = 0.0f;
        c.cyclicLongitudinal = amplitude * stroke(time, 3.1f);
        c.cyclicLateral = amplitude * stroke(time, 2.3f, 0.25f);
        c.pedals = amplitude * stroke(time, 1.7f, 0.5f);
        rig.helicopter.setControls(c);
        rig.helicopter.applyForces(step);
        rig.world.step(step);
        track(r, rig.body, rig.helicopter);
        const HelicopterTelemetry& t = rig.helicopter.telemetry();
        r.worstTilt = fmaxf(r.worstTilt, fmaxf(fabsf(t.pitch), fabsf(t.bank)));
        CHECK(t.onGround);
    }
    const KM::vec3 p = rig.body.position();
    moved = KM::length(KM::vec3(p.x, 0.0f, p.z));
    turned = fabsf(rig.helicopter.telemetry().heading);
    printf("%.0f Hz on the skids, 30 s of %.0f%% cyclic and pedal strokes: tilt %.2f deg, moved %.3f m, turned "
           "%.1f deg, rotor %.1f%%\n",
           double(rate), double(100.0f * amplitude), double(r.worstTilt * 180.0f / kPi), double(moved),
           double(turned * 180.0f / kPi), double(rig.helicopter.telemetry().rotorSpeedPercent));
}

// Sea-level density; the dive must not reach the ground in ninety seconds.
inline void airborneThrashing(f32 rate, StabilityResult& r)
{
    Rig rig;
    rig.helicopter.ownAtmosphere().setConstantDensity(1.225f);
    rig.body.setPosition(KM::vec3(0.0f, 12000.0f, 0.0f));
    HelicopterTrimTarget hover;
    CHECK(rig.helicopter.trim(hover).converged);
    const f32 step = 1.0f / rate;
    const int steps = static_cast<int>(60.0f * rate);
    for (int i = 0; i < steps; ++i)
    {
        const f32 time = static_cast<f32>(i) * step;
        HelicopterControls c;
        c.collective = 0.5f + 0.5f * stroke(time, 7.3f);
        c.cyclicLongitudinal = stroke(time, 4.1f, 0.1f);
        c.cyclicLateral = stroke(time, 2.9f, 0.3f);
        c.pedals = stroke(time, 3.7f, 0.6f);
        c.throttle = (time > 10.0f && time < 15.0f) ? 0.0f : 1.0f;
        c.engineOn = !(time > 30.0f && time < 40.0f);
        rig.helicopter.setControls(c);
        rig.helicopter.applyForces(step);
        rig.world.step(step);
        track(r, rig.body, rig.helicopter);
    }
    printf("%.0f Hz, 60 s open loop at full stroke: fastest %.1f m/s, %.2f rad/s, rotor %.1f%% to %.1f%%, "
           "flapping up to %.1f deg\n",
           double(rate), double(r.worstSpeed), double(r.worstRate), double(r.lowestRotor), double(r.worstRotor),
           double(r.worstFlap * 180.0f / kPi));
}

inline void manoeuvring(f32 rate, StabilityResult& r, f32& worstAttitude)
{
    Rig rig;
    rig.body.setPosition(KM::vec3(0.0f, 500.0f, 0.0f));
    HelicopterTrimTarget hover;
    CHECK(rig.helicopter.trim(hover).converged);
    HelicopterAutopilot autopilot;
    autopilot.engage(rig.helicopter);
    const f32 step = 1.0f / rate;
    const int steps = static_cast<int>(60.0f * rate);
    worstAttitude = 0.0f;
    for (int i = 0; i < steps; ++i)
    {
        const f32 time = static_cast<f32>(i) * step;
        const int phase = static_cast<int>(time / 6.0f) % 6;
        const KM::vec3 velocities[6] = {KM::vec3(0.0f, 0.0f, 25.0f), KM::vec3(0.0f, 0.0f, -15.0f),
                                        KM::vec3(15.0f, 0.0f, 0.0f), KM::vec3(-15.0f, 0.0f, 10.0f),
                                        KM::vec3(0.0f, 0.0f, 25.0f), KM::vec3(0.0f)};
        const f32 headings[6] = {0.0f, 0.5f * kPi, -0.5f * kPi, 0.0f, 0.25f * kPi, 0.0f};
        autopilot.holdVelocity(velocities[phase], headings[phase]);
        autopilot.holdAltitude(phase % 2 == 0 ? 520.0f : 480.0f);
        rig.helicopter.setControls(autopilot.update(rig.helicopter, step));
        rig.helicopter.applyForces(step);
        rig.world.step(step);
        track(r, rig.body, rig.helicopter);
        const HelicopterTelemetry& t = rig.helicopter.telemetry();
        worstAttitude = fmaxf(worstAttitude, fmaxf(fabsf(t.pitch), fabsf(t.bank)));
    }
    printf("%.0f Hz, 60 s of hard manoeuvring under the autopilot: attitude up to %.1f deg, %.2f rad/s, rotor "
           "%.1f%% to %.1f%%, fastest %.1f m/s\n",
           double(rate), double(worstAttitude * 180.0f / kPi), double(r.worstRate), double(r.lowestRotor),
           double(r.worstRotor), double(r.worstSpeed));
}

inline void helicopterStability(f32 rate)
{
    f32 moved = 0.0f;
    f32 turned = 0.0f;
    StabilityResult ground;
    groundStirring(rate, 1.0f, ground, moved, turned);
    CHECK(ground.finite);
    CHECK(ground.worstTilt < 10.0f * kPi / 180.0f);
    CHECK(ground.worstRotor < 105.0f && ground.lowestRotor > 95.0f);
    StabilityResult gentle;
    groundStirring(rate, 0.3f, gentle, moved, turned);
    CHECK(gentle.finite);
    CHECK(gentle.worstTilt < 2.0f * kPi / 180.0f);
    CHECK(moved < 0.05f);
    CHECK(turned < 1.0f * kPi / 180.0f);

    StabilityResult air;
    airborneThrashing(rate, air);
    CHECK(air.finite);
    CHECK(air.worstSpeed < 150.0f);
    CHECK(air.worstRate < 10.0f);
    CHECK(air.worstRotor < 250.0f && air.lowestRotor >= 0.0f);
    CHECK(air.worstFlap <= 0.351f);

    StabilityResult flown;
    f32 attitude = 0.0f;
    manoeuvring(rate, flown, attitude);
    CHECK(flown.finite);
    CHECK(attitude < 35.0f * kPi / 180.0f);
    CHECK(flown.worstRate < 1.5f);
    // Brief dips when a manoeuvre asks for more than the engine's 300 kW,
    // caught by the autopilot lowering the collective.
    CHECK(flown.lowestRotor > 88.0f && flown.worstRotor < 106.0f);
    CHECK(flown.worstSpeed < 40.0f);
}

}

#endif // KINETIX_TESTS_HELI_STABILITY_H
