#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
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

struct Flight
{
    PhysicsWorld world;
    RigidBody body;
    BoxShape hull{KM::vec3(0.5f)};
    Helicopter helicopter;
    LightHelicopterLayout layout;
    HelicopterAutopilot autopilot;
    f32 step = 1.0f / 60.0f;

    explicit Flight(f32 rate = 60.0f)
    {
        step = 1.0f / rate;
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

    void fly()
    {
        helicopter.applyForces(step);
        world.step(step);
    }

    void autopilotStep()
    {
        helicopter.setControls(autopilot.update(helicopter, step));
        fly();
    }

    bool allSkidsDown() const
    {
        for (u32 i = 0; i < helicopter.skidContactCount(); ++i)
            if (!helicopter.skidContactState(i).inContact)
                return false;
        return true;
    }

    bool finite() const
    {
        const KM::vec3 p = body.position();
        const KM::vec3 w = body.angularVelocity();
        return isfinite(p.x) && isfinite(p.y) && isfinite(p.z) && isfinite(w.x) && isfinite(w.y) &&
               isfinite(w.z) && isfinite(helicopter.transmission().rotorSpeed());
    }
};

void sitsOnSkids()
{
    Flight f;
    f32 lowest = 1.0e9f;
    f32 highest = -1.0e9f;
    for (int i = 0; i < 8 * 60; ++i)
    {
        f.fly();
        if (i > 120)
        {
            lowest = fminf(lowest, f.body.position().y);
            highest = fmaxf(highest, f.body.position().y);
        }
    }
    const HelicopterTelemetry& t = f.helicopter.telemetry();
    printf("on the skids at flat pitch: height %.3f m (rest %.3f), pitch %.2f deg, bank %.2f deg, "
           "drift %.3f m, rotor %.1f%%, %.1f kW, gear load %.0f N\n",
           double(f.body.position().y), double(f.layout.restHeight), double(t.pitch / kDeg),
           double(t.bank / kDeg), double(KM::length(KM::vec3(f.body.position().x, 0.0f, f.body.position().z))),
           double(t.rotorSpeedPercent), double(t.powerRequired / 1000.0f), double(t.gearLoad));
    CHECK(f.allSkidsDown());
    CHECK(t.onGround);
    CHECK_NEAR(f.body.position().y, f.layout.restHeight, 0.02);
    CHECK(highest - lowest < 0.005f);
    CHECK(fabsf(t.pitch) < 1.0f * kDeg && fabsf(t.bank) < 1.0f * kDeg);
    // Stiction holds it against the main rotor's torque reaction and the
    // tail rotor's push.
    CHECK(KM::length(KM::vec3(f.body.position().x, 0.0f, f.body.position().z)) < 0.02f);
    CHECK(fabsf(t.heading) < 0.5f * kDeg);
    CHECK(KM::length(f.body.velocity()) < 0.01f);
    CHECK_NEAR(t.rotorSpeedPercent, 100.0f, 1.0f);
    CHECK(t.heightAboveGround < 0.01f);
    CHECK(t.gearLoad > 0.8f * f.body.mass() * 9.81f);
}

struct HoverResult
{
    f32 worstError = 0.0f;
    f32 worstHeading = 0.0f;
    bool finite = true;
};

HoverResult hoverHold(f32 rate, const KM::vec3& wind, f32 height)
{
    Flight f(rate);
    f.helicopter.ownAtmosphere().setWind(wind);
    const KM::vec3 target(0.0f, f.layout.restHeight + height, 0.0f);
    f.autopilot.engage(f.helicopter);
    f.autopilot.holdPosition(target, 0.0f);
    HoverResult result;
    const int settle = static_cast<int>(15.0f * rate);
    const int measure = static_cast<int>(30.0f * rate);
    for (int i = 0; i < settle + measure; ++i)
    {
        f.autopilotStep();
        result.finite = result.finite && f.finite();
        if (i >= settle)
        {
            result.worstError = fmaxf(result.worstError, KM::length(f.body.position() - target));
            result.worstHeading = fmaxf(result.worstHeading, fabsf(f.helicopter.telemetry().heading));
        }
    }
    const HelicopterTelemetry& t = f.helicopter.telemetry();
    printf("hover hold %.0f Hz, wind (%.0f %.0f %.0f), %.0f m: worst %.3f m and %.2f deg over 30 s; "
           "pitch %.1f bank %.1f deg, collective %.3f, cyclic %.3f / %.3f, pedals %.3f, %.0f kW, "
           "ground effect %.3f\n",
           double(rate), double(wind.x), double(wind.y), double(wind.z), double(height),
           double(result.worstError), double(result.worstHeading / kDeg), double(t.pitch / kDeg),
           double(t.bank / kDeg), double(f.helicopter.controls().collective),
           double(f.helicopter.controls().cyclicLongitudinal), double(f.helicopter.controls().cyclicLateral),
           double(f.helicopter.controls().pedals), double(t.powerRequired / 1000.0f),
           double(t.groundEffectFactor));
    return result;
}

void hoverHoldRuns()
{
    const HoverResult calm = hoverHold(60.0f, KM::vec3(0.0f), 6.0f);
    const HoverResult high = hoverHold(60.0f, KM::vec3(0.0f), 30.0f);
    const HoverResult windy = hoverHold(60.0f, KM::vec3(5.0f, 0.0f, 0.0f), 6.0f);
    const HoverResult fine = hoverHold(120.0f, KM::vec3(0.0f), 6.0f);
    for (const HoverResult* r : {&calm, &high, &windy, &fine})
    {
        CHECK(r->finite);
        CHECK(r->worstError < 1.0f);
        CHECK(r->worstHeading < 3.0f * kDeg);
    }
}

void takeoffAndLanding()
{
    Flight f;
    const f32 hoverHeight = f.layout.restHeight + 5.0f;
    f.autopilot.engage(f.helicopter);
    f.autopilot.holdPosition(KM::vec3(0.0f, hoverHeight, 0.0f), 0.0f);
    bool liftedOff = false;
    f32 liftoffTime = 0.0f;
    f32 highest = 0.0f;
    for (int i = 0; i < 20 * 60; ++i)
    {
        f.autopilotStep();
        if (!liftedOff && !f.helicopter.telemetry().onGround)
        {
            liftedOff = true;
            liftoffTime = static_cast<f32>(i) / 60.0f;
        }
        highest = fmaxf(highest, f.body.position().y);
    }
    CHECK(liftedOff);
    CHECK(fabsf(f.body.position().y - hoverHeight) < 0.3f);
    CHECK(highest < hoverHeight + 1.5f);

    // Down at half a metre per second, then the collective goes on down by
    // itself (the vertical loop still asks for descent once on the skids).
    f.autopilot.holdVerticalSpeed(-0.5f);
    f32 touchdownSpeed = 0.0f;
    bool touched = false;
    for (int i = 0; i < 25 * 60; ++i)
    {
        const f32 vy = f.body.velocity().y;
        f.autopilotStep();
        if (!touched && f.helicopter.telemetry().onGround)
        {
            touched = true;
            touchdownSpeed = -vy;
        }
        if (touched && f.allSkidsDown())
            f.autopilot.holdVerticalSpeed(-2.0f);
    }
    HelicopterControls idle;
    idle.collective = 0.0f;
    f.helicopter.setControls(idle);
    for (int i = 0; i < 5 * 60; ++i)
        f.fly();
    const HelicopterTelemetry& t = f.helicopter.telemetry();
    const f32 offset = KM::length(KM::vec3(f.body.position().x, 0.0f, f.body.position().z));
    printf("takeoff at %.1f s; landed at %.2f m/s, %.2f m from the start, pitch %.2f bank %.2f deg, "
           "heading %.2f deg, height %.3f (rest %.3f)\n",
           double(liftoffTime), double(touchdownSpeed), double(offset), double(t.pitch / kDeg),
           double(t.bank / kDeg), double(t.heading / kDeg), double(f.body.position().y), double(f.layout.restHeight));
    CHECK(touched);
    CHECK(touchdownSpeed < 1.0f);
    CHECK(f.allSkidsDown());
    CHECK(fabsf(t.pitch) < 2.0f * kDeg && fabsf(t.bank) < 2.0f * kDeg);
    CHECK_NEAR(f.body.position().y, f.layout.restHeight, 0.03);
    CHECK(KM::length(f.body.velocity()) < 0.02f);
    CHECK(KM::length(f.body.angularVelocity()) < 0.01f);
    CHECK(offset < 1.0f);
    CHECK(fabsf(t.heading) < 3.0f * kDeg);
    CHECK(f.finite());
}

void controlResponse()
{
    struct Case
    {
        const char* name;
        f32 longitudinal;
        f32 lateral;
        f32 pedals;
        f32 collective;
    };
    const Case cases[] = {
        {"forward cyclic", 0.15f, 0.0f, 0.0f, 0.0f},
        {"aft cyclic", -0.15f, 0.0f, 0.0f, 0.0f},
        {"right cyclic", 0.0f, 0.15f, 0.0f, 0.0f},
        {"left cyclic", 0.0f, -0.15f, 0.0f, 0.0f},
        {"right pedal", 0.0f, 0.0f, 0.2f, 0.0f},
        {"left pedal", 0.0f, 0.0f, -0.2f, 0.0f},
        {"collective up", 0.0f, 0.0f, 0.0f, 0.08f},
    };
    for (const Case& c : cases)
    {
        Flight f;
        f.body.setPosition(KM::vec3(0.0f, 300.0f, 0.0f));
        HelicopterTrimTarget hover;
        CHECK(f.helicopter.trim(hover).converged);
        HelicopterControls controls = f.helicopter.controls();
        controls.cyclicLongitudinal += c.longitudinal;
        controls.cyclicLateral += c.lateral;
        controls.pedals += c.pedals;
        controls.collective += c.collective;
        f.helicopter.setControls(controls);
        KM::vec3 peakRates(0.0f);
        for (int i = 0; i < 30; ++i)
        {
            f.fly();
            const KM::vec3 r = f.helicopter.telemetry().bodyRates;
            if (fabsf(r.x) > fabsf(peakRates.x))
                peakRates.x = r.x;
            if (fabsf(r.y) > fabsf(peakRates.y))
                peakRates.y = r.y;
            if (fabsf(r.z) > fabsf(peakRates.z))
                peakRates.z = r.z;
        }
        const KM::vec3 v = f.body.velocity();
        printf("%-15s half a second: rates p %+.3f q %+.3f r %+.3f rad/s, velocity (%+.2f %+.2f %+.2f)\n", c.name,
               double(peakRates.x), double(peakRates.y), double(peakRates.z), double(v.x), double(v.y), double(v.z));
        if (c.longitudinal > 0.0f)
            CHECK(peakRates.y < -0.05f); // nose down
        if (c.longitudinal < 0.0f)
            CHECK(peakRates.y > 0.05f);
        if (c.lateral > 0.0f)
            CHECK(peakRates.x > 0.05f); // roll right
        if (c.lateral < 0.0f)
            CHECK(peakRates.x < -0.05f);
        if (c.pedals > 0.0f)
            CHECK(peakRates.z > 0.05f); // nose right
        if (c.pedals < 0.0f)
            CHECK(peakRates.z < -0.05f);
        if (c.collective > 0.0f)
            CHECK(v.y > 0.5f);
        if (c.longitudinal != 0.0f)
            CHECK(fabsf(peakRates.y) > fabsf(peakRates.x) && fabsf(peakRates.y) > fabsf(peakRates.z));
        if (c.lateral != 0.0f)
            CHECK(fabsf(peakRates.x) > fabsf(peakRates.y) && fabsf(peakRates.x) > fabsf(peakRates.z));

        // A second and a half later the disc has dragged the helicopter the
        // same way: forward cyclic flies forward (+Z), right cyclic right
        // (-X).
        for (int i = 0; i < 90; ++i)
            f.fly();
        const KM::vec3 later = f.body.velocity();
        if (c.longitudinal > 0.0f)
            CHECK(later.z > 0.5f);
        if (c.longitudinal < 0.0f)
            CHECK(later.z < -0.5f);
        if (c.lateral > 0.0f)
            CHECK(later.x < -0.5f);
        if (c.lateral < 0.0f)
            CHECK(later.x > 0.5f);
    }
}

// A helicopter is not statically stable in speed, but over ten seconds it stays near the trim.
void cruiseHandsOff()
{
    Flight f;
    f.body.setPosition(KM::vec3(0.0f, 500.0f, 0.0f));
    HelicopterTrimTarget cruise;
    cruise.airspeed = 50.0f;
    const HelicopterTrimResult r = f.helicopter.trim(cruise);
    CHECK(r.converged);
    StabilityAugmentation sas;
    sas.enabled = true;
    f.helicopter.setStabilityAugmentation(sas);
    f32 worstSpeed = 0.0f;
    f32 worstHeight = 0.0f;
    f32 worstAttitude = 0.0f;
    const HelicopterControls trimmed = f.helicopter.controls();
    for (int i = 0; i < 10 * 60; ++i)
    {
        HelicopterControls c = trimmed;
        if (i >= 30 && i < 45)
        {
            c.cyclicLongitudinal += 0.15f;
            c.cyclicLateral += 0.15f;
        }
        else if (i >= 45 && i < 60)
        {
            c.cyclicLongitudinal -= 0.15f;
            c.cyclicLateral -= 0.15f;
        }
        f.helicopter.setControls(c);
        f.fly();
        const HelicopterTelemetry& t = f.helicopter.telemetry();
        worstSpeed = fmaxf(worstSpeed, fabsf(t.trueAirspeed - 50.0f));
        worstHeight = fmaxf(worstHeight, fabsf(f.body.position().y - 500.0f));
        worstAttitude = fmaxf(worstAttitude, fmaxf(fabsf(t.pitch - r.pitch), fabsf(t.bank - r.bank)));
    }
    printf("cruise hands off, 10 s with rate damping: speed within %.2f m/s, height %.2f m, attitude %.2f deg\n",
           double(worstSpeed), double(worstHeight), double(worstAttitude / kDeg));
    CHECK(worstAttitude > 0.5f * kDeg);
    CHECK(worstSpeed < 3.0f);
    CHECK(worstHeight < 15.0f);
    CHECK(worstAttitude < 10.0f * kDeg);
    CHECK(KM::length(f.helicopter.telemetry().bodyRates) < 0.05f);
    CHECK_NEAR(f.helicopter.telemetry().rotorSpeedPercent, 100.0f, 1.0f);
}

bool raisedGround(const KM::vec3& origin, const KM::vec3& direction, f32 maxDistance, GroundHit& hit, void* userData)
{
    const f32 height = *static_cast<const f32*>(userData);
    (void)direction;
    const f32 distance = origin.y - height;
    if (distance > maxDistance)
        return false;
    hit.distance = distance;
    hit.point = KM::vec3(origin.x, height, origin.z);
    hit.normal = KM::vec3(0.0f, 1.0f, 0.0f);
    return true;
}

void groundSources()
{
    auto settle = [](Flight& f)
    {
        for (int i = 0; i < 4 * 60; ++i)
            f.fly();
    };
    {
        Flight f;
        f32 height = 3.0f;
        f.helicopter.setGroundQuery(&raisedGround, &height);
        f.body.setPosition(KM::vec3(0.0f, height + f.layout.restHeight + 0.2f, 0.0f));
        settle(f);
        CHECK(f.allSkidsDown());
        CHECK_NEAR(f.body.position().y, height + f.layout.restHeight, 0.02);
        CHECK(f.helicopter.telemetry().heightAboveGround < 0.01f);
        f.helicopter.setGroundQuery(nullptr, nullptr);
        f.helicopter.setGroundPlane(0.0f);
        settle(f);
        CHECK_NEAR(f.body.position().y, f.layout.restHeight, 0.02);
    }
    {
        Flight f;
        RigidBody slab;
        BoxShape slabShape(KM::vec3(20.0f, 1.0f, 20.0f));
        slab.setBodyType(BodyType::Static);
        slab.setPosition(KM::vec3(0.0f, 1.0f, 0.0f)); // top at y = 2
        BodyEntry entry;
        entry.body = &slab;
        entry.shape = &slabShape;
        f.world.addBody(entry);
        f.world.markStaticBroadphaseDirty();
        f.helicopter.setGround(&f.world, 0);
        f.body.setPosition(KM::vec3(0.0f, 2.0f + f.layout.restHeight + 0.2f, 0.0f));
        settle(f);
        CHECK(f.allSkidsDown());
        CHECK_NEAR(f.body.position().y, 2.0f + f.layout.restHeight, 0.02);
        // In ground effect over the slab (hub at z/R 0.55: Cheeseman-Bennett
        // 1.26), not over the plane 2 m further down (1.07).
        HelicopterControls c = f.helicopter.controls();
        c.collective = 0.5f;
        f.helicopter.setControls(c);
        f.fly();
        CHECK(f.helicopter.telemetry().groundEffectFactor > 1.2f);
    }
}

void rateDamping()
{
    f32 peak[2] = {0.0f, 0.0f};
    f32 residual[2] = {0.0f, 0.0f};
    for (int withSas = 0; withSas < 2; ++withSas)
    {
        Flight f;
        f.body.setPosition(KM::vec3(0.0f, 300.0f, 0.0f));
        HelicopterTrimTarget hover;
        CHECK(f.helicopter.trim(hover).converged);
        StabilityAugmentation sas;
        sas.enabled = withSas == 1;
        f.helicopter.setStabilityAugmentation(sas);
        const HelicopterControls trimmed = f.helicopter.controls();
        for (int i = 0; i < 3 * 60; ++i)
        {
            HelicopterControls c = trimmed;
            if (i < 15)
                c.cyclicLateral += 0.3f;
            f.helicopter.setControls(c);
            f.fly();
            const f32 p = fabsf(f.helicopter.telemetry().bodyRates.x);
            peak[withSas] = fmaxf(peak[withSas], p);
            if (i == 3 * 60 - 1)
                residual[withSas] = p;
        }
    }
    printf("lateral pulse: peak roll rate %.3f rad/s bare, %.3f with rate damping; %.3f and %.3f after 3 s\n",
           double(peak[0]), double(peak[1]), double(residual[0]), double(residual[1]));
    CHECK(peak[1] < 0.95f * peak[0]);
    CHECK(residual[1] < 0.6f * residual[0]);
}

void stepCallbackTiming()
{
    Flight manual;
    Flight callback;
    for (Flight* f : {&manual, &callback})
    {
        f->body.setPosition(KM::vec3(0.0f, 100.0f, 0.0f));
        HelicopterTrimTarget hover;
        f->helicopter.trim(hover);
        HelicopterControls c = f->helicopter.controls();
        c.cyclicLongitudinal += 0.1f;
        c.pedals += 0.1f;
        f->helicopter.setControls(c);
    }
    callback.world.setFixedStep(1.0f / 60.0f);
    callback.world.setStepCallback(&Helicopter::stepCallback, &callback.helicopter);
    callback.helicopter.applyForces(1.0f / 60.0f);
    for (int frame = 0; frame < 180; ++frame)
    {
        callback.world.update(1.0f / 30.0f);
        manual.fly();
        manual.fly();
    }
    const KM::vec3 a = manual.body.position();
    const KM::vec3 b = callback.body.position();
    printf("callback vs manual after 6 s: %.6f m apart, moved %.1f m\n", double(KM::length(a - b)),
           double(KM::length(a - KM::vec3(0.0f, 100.0f, 0.0f))));
    CHECK(KM::length(a - b) < 1e-4f);
    CHECK(KM::length(a - KM::vec3(0.0f, 100.0f, 0.0f)) > 5.0f);
    manual.helicopter.applyForces(1.0f / 60.0f);
    CHECK_NEAR(manual.helicopter.transmission().rotorSpeed(), callback.helicopter.transmission().rotorSpeed(), 1e-5);
}
}

int main()
{
    sitsOnSkids();
    hoverHoldRuns();
    takeoffAndLanding();
    controlResponse();
    cruiseHandsOff();
    groundSources();
    rateDamping();
    stepCallbackTiming();
    return TEST_RESULT();
}
