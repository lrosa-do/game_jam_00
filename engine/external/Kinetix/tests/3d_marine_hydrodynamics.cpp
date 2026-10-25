#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <marine/Boat.h>
#include <marine/BuoyancyBody.h>
#include <marine/HullMesh.h>
#include <marine/WaterSurface.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kRho = 1000.0f;
constexpr f32 kG = 9.81f;
constexpr f32 kNu = 1.0e-6f;

bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

FlatWater testWater()
{
    FlatWater water(0.0f);
    WaterProperties properties;
    properties.density = kRho;
    properties.kinematicViscosity = kNu;
    water.setProperties(properties);
    return water;
}

HydrodynamicCoefficients none()
{
    HydrodynamicCoefficients c;
    c.pressureLinear = 0.0f;
    c.pressureQuadratic = 0.0f;
    c.suctionLinear = 0.0f;
    c.suctionQuadratic = 0.0f;
    c.skinFriction = false;
    c.slamCoefficient = 0.0f;
    return c;
}

struct Floating
{
    PhysicsWorld world;
    RigidBody body;
    BoxShape shape;
    FlatWater water = testWater();
    BuoyancyBody buoyancy;

    Floating(const HullMesh& hull, const KM::vec3& halfExtents, f32 mass)
        : shape(halfExtents)
    {
        world.setGravity(KM::vec3(0.0f, -kG, 0.0f));
        body.setMass(mass);
        body.setInertiaTensor(Inertia::box(mass, halfExtents));
        body.setDamping(1.0f, 1.0f);
        body.setCanSleep(false);
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &shape;
        world.addBody(entry);
        buoyancy = BuoyancyBody(&body, hull, &water);
    }

    void step(f32 dt)
    {
        buoyancy.applyForces(dt);
        world.step(dt);
        water.advance(dt);
    }
};

void ittcLine()
{
    // 0.075 / (log10 Re - 2)^2.
    CHECK_NEAR(ittc57FrictionCoefficient(1.0e7f), 0.075 / 25.0, 1e-7);
    CHECK_NEAR(ittc57FrictionCoefficient(1.0e9f), 0.075 / 49.0, 1e-7);
    CHECK_NEAR(ittc57FrictionCoefficient(3.0e6f), 0.075 / pow(log10(3.0e6) - 2.0, 2.0), 1e-7);
    // Floored at 1e5, never infinite, never NaN.
    CHECK_NEAR(ittc57FrictionCoefficient(10.0f), 0.075 / 9.0, 1e-7);
    CHECK_NEAR(ittc57FrictionCoefficient(0.0f), 0.075 / 9.0, 1e-7);
    CHECK(isfinite(ittc57FrictionCoefficient(NAN)));
    f32 previous = 1.0f;
    for (f32 re = 1.0e5f; re < 1.0e10f; re *= 3.0f)
    {
        const f32 cf = ittc57FrictionCoefficient(re);
        CHECK(cf < previous);
        previous = cf;
    }
}

// A thin plate deep down, moving edgewise: all of it is skin friction,
// 1/2 rho Cf (1 + k) V^2 S over the faces the flow runs along.
void plateFriction()
{
    FlatWater water = testWater();
    const KM::vec3 h(0.005f, 0.5f, 1.0f);
    RigidBody body;
    body.setMass(kRho * 8.0f * h.x * h.y * h.z);
    body.setPosition(KM::vec3(0.0f, -10.0f, 0.0f));
    const f32 speed = 2.0f;
    body.setVelocity(KM::vec3(0.0f, 0.0f, speed));
    body.calculateDerivedData();
    BuoyancyBody buoyancy(&body, HullMesh::box(h, 2), &water);
    HydrodynamicCoefficients c = none();
    c.skinFriction = true;
    c.formFactor = 0.1f;
    buoyancy.setCoefficients(c);
    KM::vec3 force;
    KM::vec3 torque;
    buoyancy.computeForces(force, torque);
    const BuoyancyState& s = buoyancy.state();
    // Reference length: the hull along +Z, 2 m.
    const f32 re = speed * 2.0f / kNu;
    const f32 cf = 0.075f / ((log10f(re) - 2.0f) * (log10f(re) - 2.0f));
    // Sides (2 x 1 m each) plus top and bottom (0.01 x 2 m each).
    const f32 wetted = 2.0f * (2.0f * h.y * 2.0f * h.z) + 2.0f * (2.0f * h.x * 2.0f * h.z);
    const f32 expected = 0.5f * kRho * cf * 1.1f * speed * speed * wetted;
    printf("plate: Re %.3g, Cf %.5f, friction %.2f N, expected %.2f N\n", double(s.reynolds),
           double(s.frictionCoefficient), double(-s.frictionForce.z), double(expected));
    CHECK_NEAR(s.reynolds, re, 1e-3 * re);
    CHECK_NEAR(s.frictionCoefficient, cf, 1e-6);
    CHECK_NEAR(-s.frictionForce.z, expected, 1e-3 * expected);
    CHECK_NEAR(s.frictionForce.x, 0.0, 1e-3);
    CHECK_NEAR(s.frictionForce.y, 0.0, 1e-3);
    CHECK_NEAR(s.speedThroughWater, speed, 1e-5);
    CHECK_NEAR(s.hydrostaticForce.y, kRho * kG * 8.0f * h.x * h.y * h.z, 1e-2);
}

void cubePressureDrag()
{
    FlatWater water = testWater();
    RigidBody body;
    body.setMass(1000.0f);
    BuoyancyBody buoyancy(&body, HullMesh::box(KM::vec3(0.5f), 3), &water);
    HydrodynamicCoefficients c = none();
    c.pressureLinear = 0.4f;
    c.pressureQuadratic = 1.1f;
    c.suctionLinear = 0.3f;
    c.suctionQuadratic = 0.5f;
    buoyancy.setCoefficients(c);
    const f32 speed = 3.0f;
    body.setPosition(KM::vec3(0.0f, -20.0f, 0.0f));
    body.setVelocity(KM::vec3(speed, 0.0f, 0.0f));
    body.calculateDerivedData();
    KM::vec3 force;
    KM::vec3 torque;
    buoyancy.computeForces(force, torque);
    const f32 front = kRho * (0.4f * speed + 0.5f * 1.1f * speed * speed);
    const f32 back = kRho * (0.3f * speed + 0.5f * 0.5f * speed * speed);
    const KM::vec3 drag = buoyancy.state().pressureDragForce;
    printf("cube face-on at 3 m/s: drag %.1f N, expected %.1f N\n", double(-drag.x), double(front + back));
    CHECK_NEAR(-drag.x, front + back, 1e-4 * (front + back));
    CHECK_NEAR(drag.y, 0.0, 1e-2);
    CHECK_NEAR(drag.z, 0.0, 1e-2);
    CHECK_NEAR(KM::length(buoyancy.state().dynamicTorque), 0.0, 1e-2);

    // Linear in the coefficients' own sense: pressure ~ cos^f on a face met
    // obliquely. At 30 degrees in the xz plane two faces lead and two
    // trail; with f = 2 each sees cos^2 of its own angle.
    c.pressureFalloff = 2.0f;
    c.suctionFalloff = 2.0f;
    buoyancy.setCoefficients(c);
    const f32 angle = 30.0f * kPi / 180.0f;
    body.setVelocity(KM::vec3(cosf(angle), 0.0f, sinf(angle)) * speed);
    buoyancy.computeForces(force, torque);
    const f32 q = kRho * (0.4f * speed + 0.5f * 1.1f * speed * speed);
    const f32 s = kRho * (0.3f * speed + 0.5f * 0.5f * speed * speed);
    const f32 cx = cosf(angle);
    const f32 cz = sinf(angle);
    const KM::vec3 expected(-(q + s) * cx * cx, 0.0f, -(q + s) * cz * cz);
    const KM::vec3 got = buoyancy.state().pressureDragForce;
    CHECK_NEAR(got.x, expected.x, 1e-3 * fabsf(expected.x));
    CHECK_NEAR(got.z, expected.z, 1e-3 * fabsf(expected.z));
}

void ventilation()
{
    FlatWater water = testWater();
    RigidBody body;
    body.setMass(1000.0f);
    BuoyancyBody buoyancy(&body, HullMesh::box(KM::vec3(0.5f), 3), &water);
    HydrodynamicCoefficients c = none();
    c.suctionQuadratic = 0.5f;
    c.ventilation = true;
    buoyancy.setCoefficients(c);
    // Top face 5 cm under: the back face spans depths 0.05 .. 1.05 m.
    body.setPosition(KM::vec3(0.0f, -0.55f, 0.0f));
    body.setVelocity(KM::vec3(10.0f, 0.0f, 0.0f));
    body.calculateDerivedData();
    KM::vec3 force;
    KM::vec3 torque;
    buoyancy.computeForces(force, torque);
    const f32 limited = -buoyancy.state().pressureDragForce.x;
    c.ventilation = false;
    buoyancy.setCoefficients(c);
    buoyancy.computeForces(force, torque);
    const f32 unlimited = -buoyancy.state().pressureDragForce.x;
    // Unlimited: 1/2 rho 0.5 V^2 over the square metre. Limited: never more
    // than rho g h over each piece of it - the mean depth there is 0.55 m.
    printf("ventilation: suction %.0f N limited, %.0f N without\n", double(limited), double(unlimited));
    CHECK_NEAR(unlimited, 0.5 * kRho * 0.5 * 100.0, 1.0);
    CHECK(limited <= kRho * kG * 0.55f * 1.0f * 1.001f);
    CHECK(limited > 0.8f * kRho * kG * 0.55f);
}

void coastsToRest()
{
    const KM::vec3 h(1.0f, 0.5f, 2.0f);
    const f32 mass = 2000.0f;
    Floating box(HullMesh::box(h, 4), h, mass);
    box.body.setPosition(KM::vec3(0.0f, floatingHeight(box.buoyancy.hull(), mass, kRho), 0.0f));
    box.body.setVelocity(KM::vec3(0.0f, 0.0f, 3.0f));
    f32 previous = 3.0f;
    bool monotonic = true;
    bool forward = true;
    f32 at5 = 0.0f;
    for (int i = 0; i < 60 * 60; ++i)
    {
        box.step(1.0f / 60.0f);
        const f32 v = box.body.velocity().z;
        if (v > previous + 1.0e-6f)
            monotonic = false;
        if (v < -1.0e-4f)
            forward = false;
        previous = v;
        if (i == 60 * 5)
            at5 = v;
    }
    printf("coasting box: %.3f m/s after 5 s, %.4f m/s after 60 s, %.1f m travelled\n", double(at5),
           double(previous), double(box.body.position().z));
    CHECK(monotonic);
    CHECK(forward);
    CHECK(at5 < 1.5f);
    CHECK(previous < 0.05f);
    CHECK(fabsf(box.body.velocity().x) < 1.0e-3f);
}

// A neutrally buoyant board, broadside to its motion, light enough that one
// step of its drag at 30 Hz would throw it backwards: the limit stops it
// instead.
void dissipationLimit()
{
    const KM::vec3 h(0.01f, 0.5f, 0.5f);
    const f32 volume = 8.0f * h.x * h.y * h.z;
    for (bool limit : {true, false})
    {
        Floating board(HullMesh::box(h, 1), h, kRho * volume);
        HydrodynamicCoefficients c;
        c.skinFriction = false;
        c.limitDissipation = limit;
        board.buoyancy.setCoefficients(c);
        board.body.setPosition(KM::vec3(0.0f, -5.0f, 0.0f));
        board.body.setVelocity(KM::vec3(2.0f, 0.0f, 0.0f));
        f32 lowest = 2.0f;
        f32 fastest = 0.0f;
        f32 smallestLimit = 1.0f;
        for (int i = 0; i < 30; ++i)
        {
            board.step(1.0f / 30.0f);
            lowest = fminf(lowest, board.body.velocity().x);
            fastest = fmaxf(fastest, fabsf(board.body.velocity().x));
            smallestLimit = fminf(smallestLimit, board.buoyancy.state().linearLimit);
        }
        printf("board at 30 Hz, limit %s: lowest velocity %.3g m/s, largest after a step %.3g m/s, "
               "limit factor %.3f\n",
               limit ? "on" : "off", double(lowest), double(fastest), double(smallestLimit));
        if (limit)
        {
            CHECK(lowest >= -1.0e-4f);
            CHECK(fastest <= 2.0f);
            CHECK(smallestLimit < 1.0f);
            CHECK(finiteVec(board.body.position()));
        }
        else
        {
            CHECK(lowest < -0.1f);
            CHECK(smallestLimit == 1.0f);
        }
    }
}

void slamming()
{
    const KM::vec3 h(0.5f, 0.5f, 1.0f);
    const f32 mass = 400.0f;
    f32 peakSlam[2] = {0.0f, 0.0f};
    f32 peakDeceleration[2] = {0.0f, 0.0f};
    for (int run = 0; run < 2; ++run)
    {
        Floating wedge(HullMesh::box(h, 4), h, mass);
        HydrodynamicCoefficients c;
        if (run == 1)
            c.slamCoefficient = 0.0f;
        c.maxSlamPressure = 5.0e4f;
        wedge.buoyancy.setCoefficients(c);
        wedge.body.setOrientation(KM::angleAxis(0.25f * kPi, KM::vec3(0.0f, 0.0f, 1.0f)));
        wedge.body.setPosition(KM::vec3(0.0f, 0.75f, 0.0f));
        wedge.body.setVelocity(KM::vec3(0.0f, -4.0f, 0.0f));
        f32 previous = -4.0f;
        bool bounded = true;
        for (int i = 0; i < 60 * 20; ++i)
        {
            wedge.step(1.0f / 60.0f);
            const BuoyancyState& s = wedge.buoyancy.state();
            peakSlam[run] = fmaxf(peakSlam[run], s.slamForce.y);
            if (KM::length(s.slamForce) > c.maxSlamPressure * s.wettedArea * 1.001f)
                bounded = false;
            const f32 v = wedge.body.velocity().y;
            peakDeceleration[run] = fmaxf(peakDeceleration[run], (v - previous) * 60.0f);
            previous = v;
        }
        CHECK(bounded);
        CHECK(finiteVec(wedge.body.position()));
        CHECK(KM::length(wedge.body.velocity()) < 0.01f);
        if (run == 1)
            CHECK(peakSlam[run] == 0.0f);
    }
    printf("wedge entry at 4 m/s: peak slam force %.0f N, peak deceleration %.1f m/s^2 (%.1f without "
           "slamming)\n",
           double(peakSlam[0]), double(peakDeceleration[0]), double(peakDeceleration[1]));
    CHECK(peakSlam[0] > 100.0f);
    CHECK(peakDeceleration[0] > peakDeceleration[1]);
    CHECK(peakDeceleration[0] < 10.0f * kG);

    FlatWater water = testWater();
    RigidBody body;
    body.setMass(mass);
    body.setOrientation(KM::angleAxis(0.25f * kPi, KM::vec3(0.0f, 0.0f, 1.0f)));
    body.setVelocity(KM::vec3(0.0f, -4.0f, 0.0f));
    body.calculateDerivedData();
    BuoyancyBody buoyancy(&body, HullMesh::box(h, 4), &water);
    KM::vec3 force;
    KM::vec3 torque;
    buoyancy.computeForces(force, torque);
    CHECK(KM::length(buoyancy.state().slamForce) == 0.0f);
}

void carriedByCurrent()
{
    const KM::vec3 h(1.0f, 0.5f, 1.0f);
    const f32 mass = 1000.0f;
    Floating box(HullMesh::box(h, 3), h, mass);
    box.water.setCurrent(KM::vec3(0.8f, 0.0f, -0.3f));
    box.body.setPosition(KM::vec3(0.0f, floatingHeight(box.buoyancy.hull(), mass, kRho), 0.0f));
    for (int i = 0; i < 60 * 120; ++i)
        box.step(1.0f / 60.0f);
    const KM::vec3 v = box.body.velocity();
    printf("in a 0.85 m/s current: drifting at (%.3f, %.3f, %.3f) m/s, %.3f m/s through the water\n",
           double(v.x), double(v.y), double(v.z), double(box.buoyancy.state().speedThroughWater));
    CHECK_NEAR(v.x, 0.8, 0.02);
    CHECK_NEAR(v.z, -0.3, 0.02);
    CHECK(box.buoyancy.state().speedThroughWater < 0.02f);
}
}

int main()
{
    ittcLine();
    plateFriction();
    cubePressureDrag();
    ventilation();
    coastsToRest();
    dissipationLimit();
    slamming();
    carriedByCurrent();
    return TEST_RESULT();
}
