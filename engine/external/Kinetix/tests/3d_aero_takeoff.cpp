#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/Aircraft.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RaycastVehicle.h>
#include <dynamics/RigidBody.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kDeg = 3.14159265358979f / 180.0f;

struct Runway
{
    PhysicsWorld world;
    RigidBody ground;
    BoxShape groundShape{KM::vec3(60.0f, 1.0f, 2000.0f)};
    RigidBody body;
    BoxShape fuselage{KM::vec3(0.5f)};
    Aircraft aircraft;
    LightAircraftLayout layout;
    RaycastVehicle* gear = nullptr;

    explicit Runway(f32 headwind)
    {
        ground.setBodyType(BodyType::Static);
        ground.setPosition(KM::vec3(0.0f, -1.0f, 1500.0f));
        BodyEntry groundEntry;
        groundEntry.body = &ground;
        groundEntry.shape = &groundShape;
        world.addBody(groundEntry);

        layout = buildLightAircraft(aircraft, body);
        aircraft.ownAtmosphere().setWind(KM::vec3(0.0f, 0.0f, -headwind));
        body.setPosition(KM::vec3(0.0f, layout.restHeight, 0.0f));
        fuselage = BoxShape(layout.fuselageHalfExtents);
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &fuselage;
        const u32 id = world.addBody(entry);
        gear = new RaycastVehicle(body, &world, id);
        addLightAircraftGear(*gear, layout);
    }

    ~Runway()
    {
        delete gear;
    }

    bool onGround() const
    {
        for (u32 i = 0; i < gear->wheelCount(); ++i)
            if (gear->wheel(i).inContact)
                return true;
        return false;
    }

    void fly(f32 step, f32 rotateSpeed, f32 climbPitch, f32 throttle)
    {
        const AircraftTelemetry& t = aircraft.telemetry();
        ControlInputs c;
        c.throttle = throttle;
        c.flaps = 0.5f;
        if (t.equivalentAirspeed > rotateSpeed || !onGround())
            c.elevator = KM::clamp((climbPitch - t.pitch) * 3.0f - t.bodyRates.y, -1.0f, 1.0f);
        c.aileron = KM::clamp(-t.bank * 2.0f - t.bodyRates.x * 0.5f, -1.0f, 1.0f);
        const f32 lateral = body.position().x;
        c.rudder = KM::clamp(-t.heading * 3.0f - t.bodyRates.z - lateral * 0.05f, -1.0f, 1.0f);
        aircraft.setControls(c);
        gear->setSteering(-c.rudder * 0.3f, 0);

        aircraft.applyForces(step);
        world.step(step);
        gear->update(step);
    }
};

struct TakeoffResult
{
    bool airborne = false;
    f32 groundRoll = 0.0f;
    f32 liftoffTime = 0.0f;
    f32 liftoffSpeed = 0.0f;
    f32 speedAt5s = 0.0f;
    f32 heightAt25s = 0.0f;
    f32 climbRateAt25s = 0.0f;
    f32 worstLateral = 0.0f;
    bool finite = true;
};

TakeoffResult takeoff(f32 rate, f32 headwind)
{
    Runway runway(headwind);
    TakeoffResult result;
    const f32 step = 1.0f / rate;
    int airborneSteps = 0;
    for (int i = 0; i < static_cast<int>(25.0f * rate); ++i)
    {
        runway.fly(step, 24.0f, 9.0f * kDeg, 1.0f);
        const f32 t = static_cast<f32>(i + 1) * step;
        const KM::vec3 p = runway.body.position();
        if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z))
            result.finite = false;
        if (!result.airborne)
            result.worstLateral = fmaxf(result.worstLateral, fabsf(p.x));
        if (fabsf(t - 5.0f) < 0.5f * step)
            result.speedAt5s = KM::length(runway.body.velocity());
        airborneSteps = runway.onGround() ? 0 : airborneSteps + 1;
        if (!result.airborne && airborneSteps > static_cast<int>(0.5f * rate))
        {
            result.airborne = true;
            result.groundRoll = p.z;
            result.liftoffTime = t - 0.5f;
            result.liftoffSpeed = runway.aircraft.telemetry().trueAirspeed;
        }
    }
    result.heightAt25s = runway.body.position().y - runway.layout.restHeight;
    result.climbRateAt25s = runway.body.velocity().y;
    printf("%.0f Hz, %.0f m/s headwind: airborne %d after %.0f m and %.1f s at %.1f m/s TAS; "
           "%.0f m up at 25 s climbing %.1f m/s\n",
           double(rate), double(headwind), int(result.airborne), double(result.groundRoll),
           double(result.liftoffTime), double(result.liftoffSpeed), double(result.heightAt25s),
           double(result.climbRateAt25s));
    return result;
}

void sitsStillAtIdle()
{
    Runway runway(0.0f);
    f32 lowest = 1.0e9f;
    f32 highest = -1.0e9f;
    for (int i = 0; i < 5 * 60; ++i)
    {
        ControlInputs idle;
        runway.aircraft.setControls(idle);
        runway.aircraft.applyForces(1.0f / 60.0f);
        runway.world.step(1.0f / 60.0f);
        runway.gear->update(1.0f / 60.0f);
        if (i > 60)
        {
            lowest = fminf(lowest, runway.body.position().y);
            highest = fmaxf(highest, runway.body.position().y);
        }
    }
    CHECK(runway.gear->wheel(0).inContact && runway.gear->wheel(1).inContact &&
          runway.gear->wheel(2).inContact);
    CHECK_NEAR(lowest, runway.layout.restHeight, 0.08);
    CHECK(highest - lowest < 0.05f);
    CHECK(fabsf(runway.aircraft.telemetry().pitch) < 3.0f * kDeg);
    CHECK(fabsf(runway.aircraft.telemetry().bank) < 1.0f * kDeg);
    CHECK(KM::length(runway.body.velocity()) < 5.0f);
}

void takeoffRuns()
{
    const TakeoffResult still60 = takeoff(60.0f, 0.0f);
    const TakeoffResult still120 = takeoff(120.0f, 0.0f);
    const TakeoffResult windy = takeoff(60.0f, 10.0f);

    for (const TakeoffResult* r : {&still60, &still120})
    {
        CHECK(r->finite);
        // Accelerating: about 1.5 to 3 m/s^2 for 1000 kg on 3 kN of thrust.
        CHECK(r->speedAt5s > 7.0f && r->speedAt5s < 16.0f);
        CHECK(r->airborne);
        // A four-seat light aircraft needs 150 to 350 m of ground roll at sea
        // level, lifting off just above its flapped stall speed.
        CHECK(r->groundRoll > 120.0f && r->groundRoll < 400.0f);
        CHECK(r->liftoffSpeed > 22.0f && r->liftoffSpeed < 36.0f);
        CHECK(r->heightAt25s > 25.0f);
        CHECK(r->climbRateAt25s > 2.0f && r->climbRateAt25s < 8.0f);
        CHECK(r->worstLateral < 3.0f);
    }
    CHECK_NEAR(still120.groundRoll, still60.groundRoll, 0.1f * still60.groundRoll);

    // Into a 10 m/s headwind the same airspeed comes at a lower ground
    // speed: the roll scales with (V - Vw)^2 / V^2, about half here.
    CHECK(windy.airborne);
    CHECK(windy.groundRoll < 0.65f * still60.groundRoll);
    CHECK(windy.groundRoll > 0.3f * still60.groundRoll);
    CHECK_NEAR(windy.liftoffSpeed, still60.liftoffSpeed, 3.0);
}

// Without back stick it stays low: the rotated run above is airborne by about 190 m.
void noRotationNoLiftoff()
{
    Runway runway(0.0f);
    f32 highest = 0.0f;
    bool passed200m = false;
    bool groundedAt200m = false;
    for (int i = 0; i < 15 * 60; ++i)
    {
        ControlInputs c;
        c.throttle = 1.0f;
        c.flaps = 0.5f;
        const AircraftTelemetry& t = runway.aircraft.telemetry();
        c.aileron = KM::clamp(-t.bank * 2.0f, -1.0f, 1.0f);
        c.rudder = KM::clamp(-t.heading * 3.0f - runway.body.position().x * 0.05f, -1.0f, 1.0f);
        runway.aircraft.setControls(c);
        runway.gear->setSteering(-c.rudder * 0.3f, 0);
        runway.aircraft.applyForces(1.0f / 60.0f);
        runway.world.step(1.0f / 60.0f);
        runway.gear->update(1.0f / 60.0f);
        highest = fmaxf(highest, runway.body.position().y - runway.layout.restHeight);
        if (!passed200m && runway.body.position().z > 200.0f)
        {
            passed200m = true;
            groundedAt200m = runway.onGround();
        }
    }
    CHECK(passed200m);
    CHECK(groundedAt200m);
    CHECK(highest < 10.0f);
}
}

int main()
{
    sitsStillAtIdle();
    takeoffRuns();
    noRotationNoLiftoff();
    return TEST_RESULT();
}
