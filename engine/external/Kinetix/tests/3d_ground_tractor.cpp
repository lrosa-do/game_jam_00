#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

namespace
{
constexpr f32 kStep = 1.0f / 60.0f;

SurfaceInfo soil()
{
    SurfaceInfo s;
    s.type = SurfaceType::Soil;
    return s;
}

void run(Scene& scene, Rig& rig, f32 seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5f);
    for (int i = 0; i < steps; ++i)
        step(scene, rig, kStep);
}

// The left wheel track slick wet clay: mud at 0.4 of the lug tyre's mud
// grip, mu ~ 0.25.
SurfaceInfo splitSoil(u32, const KM::vec3& point, void*)
{
    SurfaceInfo s;
    s.type = SurfaceType::Soil;
    if (point.x > 0.0f)
    {
        s.type = SurfaceType::Mud;
        s.frictionScale = 0.4f;
    }
    return s;
}

void climbsThirtyPercent(bool locked, bool split, f32* climbed, f32* slipOut)
{
    Scene scene(false);
    const f32 angle = atanf(0.3f);
    const KM::quat slope = KM::angleAxis(-angle, KM::vec3(1.0f, 0.0f, 0.0f));
    const KM::vec3 normal = slope * KM::vec3(0.0f, 1.0f, 0.0f);
    const KM::vec3 upSlope = slope * KM::vec3(0.0f, 0.0f, 1.0f);
    scene.addStaticBox(-normal * 1.0f, KM::vec3(30.0f, 1.0f, 100.0f), slope);
    Rig tractor;
    tractor.layout = buildFarmTractor(tractor.vehicle, tractor.body);
    tractor.vehicle.setDefaultSurface(soil());
    if (split)
        tractor.vehicle.setSurfaceCallback(splitSoil, nullptr);
    tractor.body.setPosition(normal * tractor.layout.restHeight - upSlope * 20.0f);
    tractor.body.setOrientation(slope);
    tractor.body.calculateDerivedData();
    tractor.addToWorld(scene);
    if (locked)
    {
        tractor.vehicle.setDifferentialLock(static_cast<u32>(tractor.layout.rearDifferential), true);
        tractor.vehicle.setDifferentialLock(static_cast<u32>(tractor.layout.frontDifferential), true);
    }
    VehicleControls c;
    c.handbrake = 1.0f;
    tractor.vehicle.setControls(c);
    run(scene, tractor, 1.0f);
    tractor.vehicle.selectGear(1);
    c.handbrake = 0.0f;
    c.throttle = 1.0f;
    tractor.vehicle.setControls(c);
    const KM::vec3 start = tractor.body.position();
    f32 lowest = 1.0e9f;
    f32 slip = 0.0f;
    int samples = 0;
    for (int i = 0; i < static_cast<int>(16.0f / kStep); ++i)
    {
        // The driver holds it straight up the slope: on split friction a
        // locked axle pushes harder on the grippy side and yaws the tractor.
        c.steering = kinetix::Clamp(-3.0f * tractor.vehicle.telemetry().heading, -1.0f, 1.0f);
        tractor.vehicle.setControls(c);
        step(scene, tractor, kStep);
        const f32 along = KM::dot(tractor.body.velocity(), upSlope);
        if (i > 60)
        {
            lowest = fminf(lowest, along);
            const f32 wheel = tractor.vehicle.wheel(0).angularVelocity * tractor.vehicle.axle(0).wheel.radius;
            slip += (wheel - along) / fmaxf(wheel, 0.1f);
            ++samples;
        }
    }
    *climbed = KM::dot(tractor.body.position() - start, upSlope);
    *slipOut = slip / samples;
    const f32 theory = tractor.vehicle.engine().parameters().limiterRpm * 2.0f * kPi / 60.0f /
                       tractor.vehicle.gearbox().ratioOf(1) * tractor.vehicle.axle(0).wheel.radius;
    printf("30%% %s slope, differentials %s: climbed %.2f m in 16 s (%.2f m/s, first gear at the limiter %.2f), "
           "mean rear slip %.1f%%, slowest %.3f m/s, %.0f rpm, clutch locked %d\n",
           split ? "split soil/clay" : "soil", locked ? "locked" : "open", double(*climbed), double(*climbed / 16.0f),
           double(theory),
           double(100.0f * *slipOut), double(lowest), double(tractor.vehicle.telemetry().engineRpm),
           tractor.vehicle.telemetry().clutchLocked);
    if (!split || locked)
        CHECK(lowest > 0.0f);
    CHECK(tractor.vehicle.telemetry().engineRunning);
    CHECK(finiteVec(tractor.body.position()));
    CHECK(tractor.lowestUp() > 0.85f);
}

void slope()
{
    f32 climb, slip, splitLocked, splitLockedSlip, splitOpen, splitOpenSlip;
    climbsThirtyPercent(true, false, &climb, &slip);
    climbsThirtyPercent(true, true, &splitLocked, &splitLockedSlip);
    climbsThirtyPercent(false, true, &splitOpen, &splitOpenSlip);
    CHECK(climb > 9.0f);
    CHECK(slip < 0.2f);
    // With the left track slick the locks are what get it up at pace; open, the clay-side wheels spin.
    // On a slicker track the locked tractor's grippy side yaws it into the slick one faster than the front wheels can steer it back.
    CHECK(splitLocked > 9.0f);
    CHECK(splitLockedSlip < 0.1f);
    CHECK(splitOpen < 0.7f * splitLocked);
    CHECK(splitOpenSlip > 0.3f);
}

// A cosine hump 0.3 m high and 3 m long under the left wheel track only
// (about x = 0.94, flat across the tyre and falling off outside it, clear of
// the bodywork), on a flat soil field.
f32 bumpHeight(f32 x, f32 z)
{
    const f32 centre = 3.0f;
    const f32 half = 1.5f;
    if (fabsf(z - centre) > half)
        return 0.0f;
    const f32 along = 0.5f * (1.0f + cosf(kPi * (z - centre) / half));
    const f32 d = fabsf(x - 0.94f);
    f32 across = 0.0f;
    if (d < 0.25f)
        across = 1.0f;
    else if (d < 0.45f)
        across = 0.5f * (1.0f + cosf(kPi * (d - 0.25f) / 0.2f));
    return 0.3f * along * across;
}

struct BumpResult
{
    f32 lightest[4] = {1.0e9f, 1.0e9f, 1.0e9f, 1.0e9f};
    f32 maxRoll = 0.0f;
    f32 maxBodyRoll = 0.0f;
    bool allContact = true;
};

BumpResult overTheBump(bool oscillating, f32 bumpScale)
{
    Scene scene(false);
    auto height = [bumpScale](f32 x, f32 z)
    {
        return bumpScale * bumpHeight(x, z);
    };
    scene.addStatic(makeHeightfield(150, 0.2f, height), KM::vec3(0.0f), KM::quat());
    Rig tractor;
    tractor.layout = buildFarmTractor(tractor.vehicle, tractor.body);
    if (!oscillating)
        tractor.vehicle.axle(1).oscillating = false;
    tractor.vehicle.setDefaultSurface(soil());
    tractor.place(scene, KM::vec3(0.0f, 0.0f, -8.0f));
    run(scene, tractor, 1.0f);
    tractor.vehicle.selectGear(3);
    VehicleControls c;
    c.throttle = 1.0f;
    tractor.vehicle.setControls(c);
    BumpResult r;
    for (int i = 0; i < static_cast<int>(16.0f / kStep); ++i)
    {
        step(scene, tractor, kStep);
        if (i < 60)
            continue;
        for (u32 w = 0; w < 4; ++w)
        {
            r.lightest[w] = fminf(r.lightest[w], tractor.vehicle.wheel(w).load);
            r.allContact = r.allContact && tractor.vehicle.wheel(w).inContact;
        }
        r.maxRoll = fmaxf(r.maxRoll, fabsf(tractor.vehicle.telemetry().axleRoll[0]));
        r.maxBodyRoll = fmaxf(r.maxBodyRoll, fabsf(tractor.vehicle.telemetry().roll));
    }
    CHECK(tractor.body.position().z > 6.0f && tractor.body.position().z < 14.0f);
    CHECK(finiteVec(tractor.body.position()));
    return r;
}

void bump()
{
    const BumpResult osc = overTheBump(true, 1.0f);
    const BumpResult rigid = overTheBump(false, 1.0f);
    const BumpResult big = overTheBump(true, 1.8f);
    printf("0.3 m bump under the left side, oscillating front axle: lightest wheel loads %.1f %.1f %.1f %.1f kN, "
           "axle roll up to %.2f deg, body roll up to %.2f deg\n",
           double(osc.lightest[0] / 1000.0f), double(osc.lightest[1] / 1000.0f), double(osc.lightest[2] / 1000.0f),
           double(osc.lightest[3] / 1000.0f), double(osc.maxRoll / kDeg), double(osc.maxBodyRoll / kDeg));
    printf("same bump, front axle held rigid: lightest %.1f %.1f %.1f %.1f kN\n", double(rigid.lightest[0] / 1000.0f),
           double(rigid.lightest[1] / 1000.0f), double(rigid.lightest[2] / 1000.0f), double(rigid.lightest[3] / 1000.0f));
    printf("0.54 m bump, oscillating: axle roll up to %.2f deg (stop at 10), lightest %.1f %.1f %.1f %.1f kN\n",
           double(big.maxRoll / kDeg), double(big.lightest[0] / 1000.0f), double(big.lightest[1] / 1000.0f),
           double(big.lightest[2] / 1000.0f), double(big.lightest[3] / 1000.0f));
    for (u32 w = 0; w < 4; ++w)
        CHECK(osc.lightest[w] > 1000.0f);
    CHECK(osc.allContact);
    // The axle rolls as far as the bump needs - about atan(0.3 / track)
    // plus what the tyres give - and never past its stop.
    CHECK(osc.maxRoll > 7.0f * kDeg && osc.maxRoll <= 10.0f * kDeg + 1.0e-4f);
    f32 rigidFront = fminf(rigid.lightest[2], rigid.lightest[3]);
    CHECK(rigidFront < 100.0f);
    CHECK(big.maxRoll <= 10.0f * kDeg + 1.0e-4f);
    CHECK(big.maxRoll > 9.9f * kDeg);
}

void pullsLoadedTrailer()
{
    Scene scene;
    Rig tractor;
    Rig trailer;
    tractor.layout = buildFarmTractor(tractor.vehicle, tractor.body);
    trailer.layout = buildFarmTrailer(trailer.vehicle, trailer.body, 6000.0f, 0.5f);
    tractor.vehicle.setDefaultSurface(soil());
    trailer.vehicle.setDefaultSurface(soil());
    tractor.place(scene, KM::vec3(0.0f));
    CouplingParameters cp;
    cp.type = CouplingType::Ball;
    cp.towPoint = tractor.layout.towPoint;
    cp.trailerPoint = trailer.layout.couplingPoint;
    TrailerCoupling::placeTrailer(tractor.body, trailer.body, cp);
    trailer.addToWorld(scene);
    TrailerCoupling coupling;
    CHECK(coupling.attach(scene.world, tractor.vehicle, trailer.vehicle, cp));
    Rig* rigs[2] = {&tractor, &trailer};
    for (int i = 0; i < 60; ++i)
        stepAll(scene, rigs, 2, kStep);
    f32 trailerLoad = 0.0f;
    for (u32 w = 0; w < trailer.vehicle.wheelCount(); ++w)
        trailerLoad += trailer.vehicle.wheel(w).load;
    const f32 drawbar = trailer.layout.mass * kG - trailerLoad;
    tractor.vehicle.selectGear(5);
    VehicleControls c;
    c.throttle = 1.0f;
    tractor.vehicle.setControls(c);
    f32 worst = 0.0f;
    for (int i = 0; i < static_cast<int>(12.0f / kStep); ++i)
    {
        stepAll(scene, rigs, 2, kStep);
        worst = fmaxf(worst, coupling.separation());
    }
    const f32 v = tractor.vehicle.telemetry().forwardSpeed;
    const f32 theory = tractor.vehicle.engine().parameters().limiterRpm * 2.0f * kPi / 60.0f /
                       tractor.vehicle.gearbox().ratioOf(5) * tractor.vehicle.axle(0).wheel.radius;
    printf("pulling a %.1f t trailer on soil in fifth: %.2f m/s (%.2f at the limiter), %.0f rpm, drawbar load %.1f kN "
           "(%.0f%%), worst hitch separation %.2f mm, trailer %.2f m/s, articulation %.2f deg\n",
           double(trailer.layout.mass / 1000.0f), double(v), double(theory), double(tractor.vehicle.telemetry().engineRpm),
           double(drawbar / 1000.0f), double(100.0f * drawbar / (trailer.layout.mass * kG)), double(1000.0f * worst),
           double(trailer.vehicle.telemetry().forwardSpeed), double(coupling.articulationAngle() / kDeg));
    CHECK(v > 0.9f * theory);
    CHECK(worst < 0.005f);
    CHECK_NEAR(trailer.vehicle.telemetry().forwardSpeed, v, 0.02 * v);
    CHECK(fabsf(coupling.articulationAngle()) < 1.0f * kDeg);
    CHECK(drawbar > 0.04f * trailer.layout.mass * kG && drawbar < 0.12f * trailer.layout.mass * kG);
    c.throttle = 0.0f;
    c.brake = 1.0f;
    tractor.vehicle.setControls(c);
    for (int i = 0; i < static_cast<int>(4.0f / kStep); ++i)
    {
        stepAll(scene, rigs, 2, kStep);
        worst = fmaxf(worst, coupling.separation());
    }
    CHECK(trailer.vehicle.appliedBrake() == 1.0f);
    CHECK(fabsf(tractor.vehicle.telemetry().forwardSpeed) < 0.05f);
    CHECK(worst < 0.005f);
}
}

int main()
{
    slope();
    bump();
    pullsLoadedTrailer();
    return TEST_RESULT();
}
