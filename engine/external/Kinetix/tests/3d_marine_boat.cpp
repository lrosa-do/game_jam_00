#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <marine/Boat.h>
#include <marine/WaterSurface.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;

bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

struct Harbour
{
    PhysicsWorld world;
    RigidBody body;
    Boat boat;
    SmallMotorboatLayout layout;
    BoxShape shape{KM::vec3(1.0f)};
    WaterSurface* water;

    explicit Harbour(WaterSurface* surface)
        : water(surface)
    {
        world.setGravity(KM::vec3(0.0f, -9.81f, 0.0f));
        layout = buildSmallMotorboat(boat, body);
        boat.setWater(water);
        shape = BoxShape(layout.collisionHalfExtents);
        body.setPosition(KM::vec3(0.0f, water->height(0.0f, 0.0f) + layout.restHeight, 0.0f));
        body.calculateDerivedData();
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &shape;
        world.addBody(entry);
    }

    void step(f32 dt)
    {
        boat.applyForces(dt);
        world.step(dt);
        water->advance(dt);
    }

    void run(f32 seconds, const BoatControls& controls, f32 rate = 60.0f)
    {
        boat.setControls(controls);
        const int steps = static_cast<int>(seconds * rate + 0.5f);
        for (int i = 0; i < steps; ++i)
            step(1.0f / rate);
    }
};

void floatsAtRest()
{
    FlatWater water;
    Harbour h(&water);
    h.run(10.0f, BoatControls());
    const BoatTelemetry& t = h.boat.telemetry();
    printf("at rest: draft %.3f m, trim %.2f deg, heel %.2f deg, displacement %.3f m^3 (%.0f kg), "
           "waterplane %.2f m^2, engine idling at %.0f rpm\n",
           double(t.draft), double(t.trim / kDeg), double(t.heel / kDeg), double(t.submergedVolume),
           double(t.submergedVolume * water.density()), double(t.waterplaneArea), double(t.engineRpm));
    CHECK_NEAR(t.submergedVolume * water.density(), h.layout.mass, 0.005 * h.layout.mass);
    CHECK_NEAR(h.body.position().y, h.layout.restHeight, 0.03);
    CHECK(t.draft > 0.25f && t.draft < 0.5f);
    CHECK(fabsf(t.trim) < 2.0f * kDeg);
    CHECK(fabsf(t.heel) < 0.1f * kDeg);
    CHECK(t.speedThroughWater < 0.01f);
    CHECK(t.thrust == 0.0f);
    CHECK(t.engineRpm > 500.0f && t.engineRpm < 900.0f);
}

void topSpeed()
{
    FlatWater water;
    Harbour h(&water);
    BoatControls full;
    full.throttle = 1.0f;
    h.run(5.0f, full);
    const f32 at5 = h.boat.telemetry().forwardSpeed;
    h.run(35.0f, full);
    const f32 before = h.boat.telemetry().forwardSpeed;
    h.run(5.0f, full);
    const BoatTelemetry& t = h.boat.telemetry();
    printf("full throttle: %.1f m/s after 5 s; top speed %.2f m/s (%.1f kn), engine %.0f rpm, trim %.2f deg, "
           "heel %.2f deg, draft %.3f m, thrust %.0f N\n",
           double(at5), double(t.forwardSpeed), double(t.forwardSpeed * 1.94384f), double(t.engineRpm),
           double(t.trim / kDeg), double(t.heel / kDeg), double(t.draft), double(t.thrust));
    // A 6 m, 1250 kg boat with 75 kW: 30 to 45 knots planing.
    CHECK(t.forwardSpeed > 15.0f && t.forwardSpeed < 23.0f);
    CHECK_NEAR(t.forwardSpeed, before, 0.01 * before);
    CHECK(at5 > 4.0f);
    CHECK(t.draft < 0.25f);
    CHECK(t.trim > 0.0f && t.trim < 6.0f * kDeg);
    CHECK(t.engineRpm > 5000.0f && t.engineRpm < 5800.0f);
    CHECK(fabsf(t.heel) < 5.0f * kDeg);
    CHECK(fabsf(t.heading) < 3.0f * kDeg);
}

void turningCircle(f32 rudder, f32* diameter, f32* yawRate, f32* heel, f32* spread)
{
    FlatWater water;
    Harbour h(&water);
    BoatControls c;
    c.throttle = 0.6f;
    h.run(15.0f, c);
    c.rudder = rudder;
    h.run(20.0f, c);
    const f32 period = 2.0f * kPi / fabsf(h.boat.telemetry().yawRate);
    const int samples = static_cast<int>(2.0f * period * 60.0f);
    KM::vec3 centre(0.0f);
    ct::Vector<KM::vec3> track;
    for (int i = 0; i < samples; ++i)
    {
        h.step(1.0f / 60.0f);
        const KM::vec3 p(h.body.position().x, 0.0f, h.body.position().z);
        track.push_back(p);
        centre += p;
    }
    centre /= static_cast<f32>(samples);
    f32 mean = 0.0f;
    for (const KM::vec3& p : track)
        mean += KM::length(p - centre);
    mean /= static_cast<f32>(samples);
    f32 worst = 0.0f;
    for (const KM::vec3& p : track)
        worst = fmaxf(worst, fabsf(KM::length(p - centre) - mean));
    *diameter = 2.0f * mean;
    *yawRate = h.boat.telemetry().yawRate;
    *heel = h.boat.telemetry().heel;
    *spread = worst / mean;
    printf("rudder %+.1f at 60%% throttle: turning circle %.1f m across (%.1f boat lengths), yaw rate %.3f "
           "rad/s, %.2f m/s, heel %.1f deg, radius spread %.1f%%\n",
           double(rudder), double(*diameter), double(*diameter / 6.0f), double(*yawRate),
           double(h.boat.telemetry().speedThroughWater), double(*heel / kDeg), double(100.0f * *spread));
    CHECK(finiteVec(h.body.position()));
}

void turns()
{
    f32 diameter = 0.0f;
    f32 rate = 0.0f;
    f32 heel = 0.0f;
    f32 spread = 0.0f;
    turningCircle(1.0f, &diameter, &rate, &heel, &spread);
    // Rudder +1: to starboard, heading increasing.
    CHECK(rate > 0.0f);
    CHECK(diameter > 6.0f && diameter < 60.0f);
    CHECK(spread < 0.05f);
    CHECK(heel > 0.0f && heel < 20.0f * kDeg);

    f32 diameterLeft = 0.0f;
    turningCircle(-1.0f, &diameterLeft, &rate, &heel, &spread);
    CHECK(rate < 0.0f);
    CHECK(heel < 0.0f);
    CHECK(spread < 0.05f);
    // Nearly symmetric: only the propeller's torque tells the sides apart.
    CHECK_NEAR(diameterLeft, diameter, 0.15 * diameter);

    f32 wide = 0.0f;
    turningCircle(0.4f, &wide, &rate, &heel, &spread);
    CHECK(wide > 1.2f * diameter);
}

void goesAstern()
{
    FlatWater water;
    Harbour h(&water);
    BoatControls c;
    c.throttle = -1.0f;
    h.run(15.0f, c);
    const BoatTelemetry& t = h.boat.telemetry();
    printf("full astern: %.2f m/s, engine %.0f rpm, propeller %.0f rpm, thrust %.0f N, trim %.2f deg\n",
           double(t.forwardSpeed), double(t.engineRpm), double(t.propellerRpm), double(t.thrust),
           double(t.trim / kDeg));
    CHECK(t.forwardSpeed < -2.0f);
    CHECK(t.propellerRpm < 0.0f);
    CHECK(t.thrust < 0.0f);
    CHECK(h.body.position().z < -15.0f);
    CHECK(fabsf(t.heading) < 30.0f * kDeg);

    c.throttle = 0.03f;
    h.run(30.0f, c);
    CHECK(h.boat.telemetry().thrust == 0.0f);
    CHECK(fabsf(h.boat.telemetry().forwardSpeed) < 0.2f);
}

void propellerOutOfWater()
{
    FlatWater water;
    Harbour h(&water);
    h.body.setBodyType(BodyType::Kinematic);
    h.body.setPosition(KM::vec3(0.0f, 3.0f, 0.0f));
    h.body.calculateDerivedData();
    BoatControls c;
    c.throttle = 1.0f;
    h.boat.setControls(c);
    f32 worst = 0.0f;
    for (int i = 0; i < 60 * 5; ++i)
    {
        h.boat.applyForces(1.0f / 60.0f);
        worst = fmaxf(worst, fabsf(h.boat.telemetry().thrust));
        h.body.clearAccumulators();
    }
    const BoatTelemetry& t = h.boat.telemetry();
    printf("lifted clear: thrust %.1f N, immersion %.2f, engine %.0f rpm\n", double(worst), double(t.immersion),
           double(t.engineRpm));
    CHECK(worst == 0.0f);
    CHECK(t.immersion == 0.0f);
    CHECK(t.submergedVolume == 0.0f);
    CHECK(t.engineRpm > 5900.0f && t.engineRpm < 6150.0f);
    CHECK(KM::length(t.force - KM::vec3(0.0f)) < 1.0f);

    h.body.setPosition(KM::vec3(0.0f, h.layout.restHeight, 0.0f));
    h.body.calculateDerivedData();
    h.boat.applyForces(1.0f / 60.0f);
    CHECK(h.boat.telemetry().thrust > 1000.0f);
}

void restsOnWaves(f32 rate)
{
    GerstnerWaves waves;
    GerstnerWaves::Wave a;
    a.amplitude = 0.45f;
    a.wavelength = 30.0f;
    a.direction = KM::vec2(1.0f, 0.0f);
    a.steepness = 0.8f;
    waves.addWave(a);
    GerstnerWaves::Wave b;
    b.amplitude = 0.2f;
    b.wavelength = 12.0f;
    b.direction = KM::vec2(0.6f, 0.8f);
    b.phase = 1.0f;
    b.steepness = 0.8f;
    waves.addWave(b);
    GerstnerWaves::Wave c;
    c.amplitude = 0.07f;
    c.wavelength = 5.0f;
    c.direction = KM::vec2(-0.3f, 1.0f);
    c.phase = 2.0f;
    c.steepness = 0.6f;
    waves.addWave(c);
    Harbour h(&waves);
    f32 worstHeel = 0.0f;
    f32 worstTrim = 0.0f;
    f32 lowestUp = 1.0f;
    f32 worstDepth = -1.0f;
    f32 fastest = 0.0f;
    bool finite = true;
    const int steps = static_cast<int>(120.0f * rate);
    for (int i = 0; i < steps; ++i)
    {
        h.step(1.0f / rate);
        const BoatTelemetry& t = h.boat.telemetry();
        finite = finite && finiteVec(h.body.position()) && finiteVec(h.body.velocity()) &&
                 finiteVec(h.body.angularVelocity()) && isfinite(t.engineRpm);
        worstHeel = fmaxf(worstHeel, fabsf(t.heel));
        worstTrim = fmaxf(worstTrim, fabsf(t.trim));
        lowestUp = fminf(lowestUp, h.body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f)).y);
        worstDepth = fmaxf(worstDepth, waves.depth(h.body.position()));
        fastest = fmaxf(fastest, KM::length(h.body.velocity()));
    }
    const KM::vec3 p = h.body.position();
    printf("adrift on waves at %.0f Hz for 120 s: worst heel %.1f deg, worst trim %.1f deg, lowest up %.3f, "
           "centre of mass never more than %.2f m under the surface, fastest %.2f m/s, drifted to (%.1f, %.1f)\n",
           double(rate), double(worstHeel / kDeg), double(worstTrim / kDeg), double(lowestUp),
           double(worstDepth), double(fastest), double(p.x), double(p.z));
    CHECK(finite);
    CHECK(worstHeel < 30.0f * kDeg);
    CHECK(worstTrim < 20.0f * kDeg);
    CHECK(lowestUp > 0.85f);
    CHECK(worstDepth < 0.3f);
    CHECK(fastest < 3.0f);
    CHECK(fabsf(p.x) < 30.0f && fabsf(p.z) < 30.0f);
}

void drivesThroughWaves()
{
    GerstnerWaves waves;
    GerstnerWaves::Wave a;
    a.amplitude = 0.35f;
    a.wavelength = 25.0f;
    a.direction = KM::vec2(0.0f, -1.0f);
    waves.addWave(a);
    Harbour h(&waves);
    // 60% is about 25 knots into a 0.7 m sea - what a skipper would hold.
    // Much faster and the boat leaps off the crests and lands on a chine,
    // which with no one easing the throttle builds into large rolls.
    BoatControls c;
    c.throttle = 0.6f;
    c.rudder = 0.2f;
    bool finite = true;
    f32 lowestUp = 1.0f;
    f32 peakSlam = 0.0f;
    for (int i = 0; i < 60 * 40; ++i)
    {
        h.boat.setControls(c);
        h.step(1.0f / 60.0f);
        finite = finite && finiteVec(h.body.position()) && finiteVec(h.body.velocity());
        lowestUp = fminf(lowestUp, h.body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f)).y);
        peakSlam = fmaxf(peakSlam, KM::length(h.boat.hull().state().slamForce));
    }
    printf("driving into 25 m waves at 60%%: %.1f m/s, lowest up %.3f, peak slam %.0f N\n",
           double(h.boat.telemetry().speedThroughWater), double(lowestUp), double(peakSlam));
    CHECK(finite);
    CHECK(lowestUp > 0.95f);
    CHECK(h.boat.telemetry().speedThroughWater > 10.0f);
}

// Aircraft's timing contract: forces applied before each step(), or from
// the step callback after one call by hand, give identical trajectories.
void stepCallbackTiming()
{
    FlatWater water;
    Harbour manual(&water);
    Harbour callback(&water);
    BoatControls c;
    c.throttle = 0.7f;
    c.rudder = 0.3f;
    manual.boat.setControls(c);
    callback.boat.setControls(c);
    const f32 dt = 1.0f / 60.0f;
    callback.world.setFixedStep(dt);
    callback.world.setStepCallback(&Boat::stepCallback, &callback.boat);
    callback.boat.applyForces(dt);
    for (int i = 0; i < 300; ++i)
    {
        manual.boat.applyForces(dt);
        manual.world.step(dt);
        callback.world.update(dt);
    }
    const KM::vec3 a = manual.body.position();
    const KM::vec3 b = callback.body.position();
    printf("step callback timing: manual (%.4f, %.4f, %.4f), callback (%.4f, %.4f, %.4f)\n", double(a.x),
           double(a.y), double(a.z), double(b.x), double(b.y), double(b.z));
    CHECK(a.x == b.x && a.y == b.y && a.z == b.z);
    CHECK(a.z > 5.0f);
}
}

int main()
{
    floatsAtRest();
    topSpeed();
    turns();
    goesAstern();
    propellerOutOfWater();
    restsOnWaves(60.0f);
    restsOnWaves(120.0f);
    drivesThroughWaves();
    stepCallbackTiming();
    return TEST_RESULT();
}
