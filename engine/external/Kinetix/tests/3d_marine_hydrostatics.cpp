#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <geometry/ConvexHullComputer.h>
#include <marine/Boat.h>
#include <marine/BuoyancyBody.h>
#include <marine/HullMesh.h>
#include <marine/WaterSurface.h>

#include <math.h>
#include <stdlib.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;
constexpr f32 kRho = 1000.0f;
constexpr f32 kG = 9.81f;

f32 random(f32 lo, f32 hi)
{
    return lo + (hi - lo) * static_cast<f32>(rand()) / static_cast<f32>(RAND_MAX);
}

FlatWater testWater()
{
    FlatWater water(0.0f);
    WaterProperties properties;
    properties.density = kRho;
    properties.kinematicViscosity = 1.0e-6f;
    water.setProperties(properties);
    return water;
}

HydrodynamicCoefficients hydrostaticsOnly()
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

f32 heelOf(const RigidBody& body)
{
    // Starboard (body -X) down is positive, as in BoatTelemetry.
    const KM::vec3 right = body.directionToWorld(KM::vec3(-1.0f, 0.0f, 0.0f));
    const KM::vec3 up = body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f));
    return atan2f(-right.y, up.y);
}

struct Floating
{
    PhysicsWorld world;
    RigidBody body;
    BoxShape shape;
    FlatWater water = testWater();
    BuoyancyBody buoyancy;

    Floating(const HullMesh& hull, const KM::vec3& halfExtents, f32 mass, const KM::mat3& inertia)
        : shape(halfExtents)
    {
        world.setGravity(KM::vec3(0.0f, -kG, 0.0f));
        body.setMass(mass);
        body.setInertiaTensor(inertia);
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

void checkArchimedes(const char* name, const HullMesh& hull, f32 size)
{
    FlatWater water = testWater();
    RigidBody body;
    body.setMass(10.0f);
    BuoyancyBody buoyancy(&body, hull, &water);
    f32 worstForce = 0.0f;
    f32 worstTorque = 0.0f;
    f32 worstCentre = 0.0f;
    for (int trial = 0; trial < 20; ++trial)
    {
        const KM::quat q = KM::normalize(KM::quat(random(-1, 1), random(-1, 1), random(-1, 1), random(-1, 1)));
        body.setOrientation(q);
        body.setPosition(KM::vec3(random(-50, 50), -3.0f * size - random(0, 20), random(-50, 50)));
        body.calculateDerivedData();
        KM::vec3 force;
        KM::vec3 torque;
        buoyancy.computeForces(force, torque);
        const BuoyancyState& s = buoyancy.state();
        const f32 weight = kRho * kG * hull.volume();
        worstForce = fmaxf(worstForce, KM::length(force - KM::vec3(0.0f, weight, 0.0f)) / weight);
        // About the centre of buoyancy the buoyant force has no moment.
        const KM::vec3 aboutCentre = torque - KM::cross(s.centreOfBuoyancy - body.position(), force);
        worstTorque = fmaxf(worstTorque, KM::length(aboutCentre) / (weight * size));
        const KM::vec3 centroid = body.pointToWorld(hull.centroid());
        worstCentre = fmaxf(worstCentre, KM::length(s.centreOfBuoyancy - centroid) / size);
        CHECK_NEAR(s.submergedVolume, hull.volume(), 1e-4 * hull.volume());
        CHECK_NEAR(s.submergedFraction, 1.0, 1e-4);
        CHECK_NEAR(s.wettedArea, hull.surfaceArea(), 1e-4 * hull.surfaceArea());
        CHECK(s.waterplaneArea == 0.0f);
    }
    printf("%s fully submerged: buoyancy error %.2e, torque about CoB %.2e (weight x size), "
           "CoB off centroid %.2e (size)\n",
           name, double(worstForce), double(worstTorque), double(worstCentre));
    CHECK(worstForce < 0.005f);
    CHECK(worstTorque < 1.0e-4f);
    CHECK(worstCentre < 1.0e-4f);
}

void archimedes()
{
    srand(3);
    checkArchimedes("cube", HullMesh::box(KM::vec3(0.5f), 2), 1.0f);
    checkArchimedes("sphere", HullMesh::sphere(0.8f, 16, 32), 1.6f);
    kinetix::Geometry::ConvexHullComputer computer;
    f32 points[3 * 30];
    for (int i = 0; i < 30; ++i)
    {
        points[3 * i] = random(-1.5f, 0.8f);
        points[3 * i + 1] = random(-0.4f, 0.9f);
        points[3 * i + 2] = random(-0.7f, 2.0f);
    }
    computer.compute(points, 3 * sizeof(f32), 30, 0.0f, 0.0f);
    HullMesh hull = HullMesh::fromConvexHull(computer);
    CHECK(hull.isClosed());
    checkArchimedes("convex hull", hull, 2.0f);
    checkArchimedes("boat hull", HullMesh::boat(BoatHullParameters()), 6.0f);
}

void halfVolumes()
{
    srand(5);
    FlatWater water = testWater();
    RigidBody body;
    const HullMesh shapes[2] = {HullMesh::box(KM::vec3(0.6f, 0.3f, 1.1f), 3), HullMesh::sphere(1.0f, 16, 32)};
    for (const HullMesh& hull : shapes)
    {
        BuoyancyBody buoyancy(&body, hull, &water);
        for (int trial = 0; trial < 20; ++trial)
        {
            body.setOrientation(KM::normalize(KM::quat(random(-1, 1), random(-1, 1), random(-1, 1), random(-1, 1))));
            body.setPosition(KM::vec3(random(-5, 5), 0.0f, random(-5, 5)));
            body.calculateDerivedData();
            KM::vec3 force;
            KM::vec3 torque;
            buoyancy.computeForces(force, torque);
            CHECK_NEAR(buoyancy.state().submergedVolume, 0.5 * hull.volume(), 1e-5 * hull.volume());
            CHECK_NEAR(force.y, 0.5 * kRho * kG * hull.volume(), 1e-4 * kRho * kG * hull.volume());
            CHECK_NEAR(KM::length(KM::vec3(force.x, 0.0f, force.z)), 0.0, 1e-4 * force.y);
        }
    }

    // Level box half under: centre of buoyancy at -hy/2, waterplane 4 hx hz.
    const KM::vec3 h(0.6f, 0.3f, 1.1f);
    BuoyancyBody buoyancy(&body, HullMesh::box(h, 2), &water);
    body.setOrientation(KM::quat());
    body.setPosition(KM::vec3(2.0f, 0.0f, -1.0f));
    body.calculateDerivedData();
    KM::vec3 force;
    KM::vec3 torque;
    buoyancy.computeForces(force, torque);
    const BuoyancyState& s = buoyancy.state();
    CHECK_NEAR(s.centreOfBuoyancy.x, 2.0, 1e-5);
    CHECK_NEAR(s.centreOfBuoyancy.y, -0.5 * h.y, 1e-5);
    CHECK_NEAR(s.centreOfBuoyancy.z, -1.0, 1e-5);
    CHECK_NEAR(s.waterplaneArea, 4.0 * h.x * h.z, 1e-5);
    CHECK_NEAR(s.draft, h.y, 1e-6);
    CHECK_NEAR(KM::length(torque), 0.0, 1e-3);
    for (const KM::vec3& p : buoyancy.waterline())
        CHECK_NEAR(p.y, 0.0, 1e-6);
    CHECK(buoyancy.waterline().size() > 0);

    body.setPosition(KM::vec3(0.0f, 5.0f, 0.0f));
    body.calculateDerivedData();
    buoyancy.computeForces(force, torque);
    CHECK(KM::length(force) == 0.0f);
    CHECK(buoyancy.state().submergedVolume == 0.0f);
    CHECK(buoyancy.state().draft < 0.0f);
}

// L = 4 (z), B = 2 (x), H = 1: draft m / (rho L B).
void floatingBoxDraft()
{
    const KM::vec3 h(1.0f, 0.5f, 2.0f);
    const f32 mass = 2000.0f;
    const f32 draft = mass / (kRho * 4.0f * 2.0f);
    const HullMesh hull = HullMesh::box(h, 4);
    const f32 height = floatingHeight(hull, mass, kRho);
    CHECK_NEAR(h.y - height, draft, 1e-5);

    Floating box(hull, h, mass, Inertia::box(mass, h));
    box.body.setPosition(KM::vec3(0.0f, height + 0.2f, 0.0f));
    f32 window = 0.0f;
    f32 lastWindow = 1.0e9f;
    bool decaying = true;
    for (int i = 0; i < 60 * 30; ++i)
    {
        box.step(1.0f / 60.0f);
        window = fmaxf(window, fabsf(box.body.position().y - height));
        if ((i + 1) % 60 == 0)
        {
            if (window > lastWindow && window > 1.0e-4f)
                decaying = false;
            lastWindow = window;
            window = 0.0f;
        }
    }
    const f32 settled = h.y - box.body.position().y;
    printf("floating box: draft %.5f m, expected %.5f (%.3f%%), residual speed %.2e m/s\n", double(settled),
           double(draft), double(100.0f * (settled - draft) / draft), double(KM::length(box.body.velocity())));
    CHECK_NEAR(settled, draft, 0.01 * draft);
    CHECK(decaying);
    CHECK(KM::length(box.body.velocity()) < 1.0e-3f);
    CHECK(KM::length(box.body.angularVelocity()) < 1.0e-3f);
    CHECK_NEAR(box.buoyancy.state().draft, draft, 0.01 * draft);
}

// Undamped heave: T = 2 pi sqrt(m / (rho g A_wp)). There is no added mass in
// the model, so the simulation should match the formula itself - a real box
// of these proportions (B / T = 8) heaves some 20-40% slower because the
// water it moves adds that share to its inertia.
void heavePeriod()
{
    const KM::vec3 h(1.0f, 0.5f, 2.0f);
    const f32 mass = 2000.0f;
    const HullMesh hull = HullMesh::box(h, 2);
    const f32 expected = 2.0f * kPi * sqrtf(mass / (kRho * kG * 8.0f));
    for (f32 rate : {60.0f, 120.0f})
    {
        Floating box(hull, h, mass, Inertia::box(mass, h));
        box.buoyancy.setCoefficients(hydrostaticsOnly());
        const f32 height = floatingHeight(hull, mass, kRho);
        box.body.setPosition(KM::vec3(0.0f, height + 0.05f, 0.0f));
        const f32 dt = 1.0f / rate;
        f32 previous = 0.05f;
        f32 first = -1.0f;
        f32 last = -1.0f;
        int crossings = 0;
        f32 highest = 0.0f;
        for (int i = 0; i < static_cast<int>(12.0f * rate); ++i)
        {
            box.step(dt);
            const f32 offset = box.body.position().y - height;
            if (previous > 0.0f && offset <= 0.0f)
            {
                const f32 t = (static_cast<f32>(i) + previous / (previous - offset)) * dt;
                if (first < 0.0f)
                    first = t;
                else
                    ++crossings;
                last = t;
            }
            if (i > static_cast<int>(rate))
                highest = fmaxf(highest, offset);
            previous = offset;
        }
        const f32 period = (last - first) / static_cast<f32>(crossings);
        printf("heave at %.0f Hz: period %.4f s, expected %.4f s (%.3f%%), amplitude kept %.1f%%\n",
               double(rate), double(period), double(expected), double(100.0f * (period - expected) / expected),
               double(100.0f * highest / 0.05f));
        CHECK(crossings >= 10);
        CHECK_NEAR(period, expected, 0.01 * expected);
        // No damping and a symplectic integrator: the swing neither grows
        // nor dies.
        CHECK_NEAR(highest, 0.05, 0.002);
    }
}

// A wall-sided box: L = 4, B = 2, H = 1, draft 0.5, centre of mass 0.3 above
// the keel. KB = T / 2, BM = I_wp / V = B^2 / (12 T), GM = KB + BM - KG, and
// heeled by phi the righting arm is GZ = sin(phi) (GM + BM tan^2(phi) / 2)
// exactly while neither the deck edge nor the bilge leaves its side.
struct StabilityBox
{
    KM::vec3 halfExtents{1.0f, 0.5f, 2.0f};
    f32 draft = 0.5f;
    f32 kg = 0.3f;
    f32 mass = kRho * 4.0f * 2.0f * 0.5f;
    f32 bm = 2.0f * 2.0f / (12.0f * 0.5f);
    f32 gm = 0.5f * 0.5f + 2.0f * 2.0f / (12.0f * 0.5f) - 0.3f;

    HullMesh hull() const
    {
        HullMesh mesh = HullMesh::box(halfExtents, 4);
        // Keel at y = -kg: the body origin is the centre of mass.
        mesh.translate(KM::vec3(0.0f, halfExtents.y - kg, 0.0f));
        return mesh;
    }
    void pose(RigidBody& body, f32 heel) const
    {
        const KM::quat q = KM::angleAxis(heel, KM::vec3(0.0f, 0.0f, 1.0f));
        body.setOrientation(q);
        body.setPosition(-(q * KM::vec3(0.0f, draft - kg, 0.0f)));
        body.calculateDerivedData();
    }
};

void rightingMoment()
{
    const StabilityBox box;
    FlatWater water = testWater();
    RigidBody body;
    body.setMass(box.mass);
    BuoyancyBody buoyancy(&body, box.hull(), &water);
    const f32 displacement = box.mass * kG;
    printf("stability box: GM %.4f m, BM %.4f m\n", double(box.gm), double(box.bm));
    for (f32 degrees : {1.0f, 2.0f, 5.0f, 10.0f, 20.0f})
    {
        const f32 phi = degrees * kDeg;
        box.pose(body, phi);
        KM::vec3 force;
        KM::vec3 torque;
        buoyancy.computeForces(force, torque);
        const f32 gz = sinf(phi) * (box.gm + 0.5f * box.bm * tanf(phi) * tanf(phi));
        const f32 small = displacement * box.gm * sinf(phi);
        // Positive rotation about +Z lifts body +X: the righting moment is
        // about -Z.
        const f32 righting = -torque.z;
        printf("  heel %4.1f deg: righting %.1f N m, wall-sided %.1f (%.3f%%), small-angle rho g V GM sin %.1f\n",
               double(degrees), double(righting), double(displacement * gz),
               double(100.0f * (righting - displacement * gz) / (displacement * gz)), double(small));
        CHECK_NEAR(righting, displacement * gz, 0.005 * displacement * gz);
        if (degrees <= 5.0f)
            CHECK_NEAR(righting, small, 0.03 * small);
        CHECK_NEAR(force.y, displacement, 1e-4 * displacement);
        CHECK_NEAR(buoyancy.state().submergedVolume, box.mass / kRho, 1e-4 * box.mass / kRho);
    }
}

// Released at 10 degrees it rolls back upright: undamped at the natural
// period 2 pi sqrt(I / (rho g V GM)), damped it settles.
void returnsUpright()
{
    const StabilityBox box;
    const KM::mat3 inertia = Inertia::box(box.mass, box.halfExtents);
    const f32 rollInertia = box.mass * (4.0f * box.halfExtents.x * box.halfExtents.x +
                                        4.0f * box.halfExtents.y * box.halfExtents.y) / 12.0f;
    const f32 expected = 2.0f * kPi * sqrtf(rollInertia / (box.mass * kG * box.gm));
    {
        Floating f(box.hull(), box.halfExtents, box.mass, inertia);
        f.buoyancy.setCoefficients(hydrostaticsOnly());
        box.pose(f.body, 2.0f * kDeg);
        f32 previous = 2.0f * kDeg;
        f32 first = -1.0f;
        f32 last = 0.0f;
        int crossings = 0;
        const f32 dt = 1.0f / 120.0f;
        for (int i = 0; i < 120 * 15; ++i)
        {
            f.step(dt);
            const f32 heel = heelOf(f.body);
            // Heeled to port (+X down) is negative in the telemetry sign.
            if (previous < 0.0f && heel >= 0.0f)
            {
                const f32 t = (static_cast<f32>(i) + previous / (previous - heel)) * dt;
                if (first < 0.0f)
                    first = t;
                else
                    ++crossings;
                last = t;
            }
            previous = heel;
        }
        const f32 period = (last - first) / static_cast<f32>(crossings);
        printf("roll: period %.3f s, expected %.3f s (%.2f%%)\n", double(period), double(expected),
               double(100.0f * (period - expected) / expected));
        CHECK(crossings >= 4);
        CHECK_NEAR(period, expected, 0.03 * expected);
    }
    {
        Floating f(box.hull(), box.halfExtents, box.mass, inertia);
        box.pose(f.body, 10.0f * kDeg);
        f32 worst = 0.0f;
        for (int i = 0; i < 60 * 40; ++i)
        {
            f.step(1.0f / 60.0f);
            worst = fmaxf(worst, fabsf(heelOf(f.body)));
        }
        printf("damped roll from 10 deg: %.3f deg after 40 s (never past %.2f deg)\n",
               double(fabsf(heelOf(f.body)) / kDeg), double(worst / kDeg));
        CHECK(fabsf(heelOf(f.body)) < 0.2f * kDeg);
        CHECK(worst < 10.5f * kDeg);
    }
}

// Tall and narrow at half density: upright GM = T/2 + B^2/(12 T) - H/2 < 0.
// On its side GM is positive, and that is where it ends up.
void negativeStabilityCapsizes()
{
    const KM::vec3 h(0.5f, 1.5f, 2.0f);
    const f32 mass = 0.5f * kRho * 8.0f * h.x * h.y * h.z;
    const f32 draft = 1.5f;
    const f32 uprightGm = 0.5f * draft + 1.0f / (12.0f * draft) - h.y;
    const f32 sideGm = 0.25f + 9.0f / (12.0f * 0.5f) - 0.5f;
    printf("capsize box: GM upright %.3f m, on its side %.3f m\n", double(uprightGm), double(sideGm));
    CHECK(uprightGm < 0.0f);
    Floating f(HullMesh::box(h, 3), h, mass, Inertia::box(mass, h));
    f.body.setOrientation(KM::angleAxis(1.0f * kDeg, KM::vec3(0.0f, 0.0f, 1.0f)));
    f.body.setPosition(KM::vec3(0.0f, h.y - draft, 0.0f));
    f32 settledWorst = 0.0f;
    for (int i = 0; i < 60 * 60; ++i)
    {
        f.step(1.0f / 60.0f);
        if (i > 60 * 50)
            settledWorst = fmaxf(settledWorst, fabsf(fabsf(heelOf(f.body)) - 90.0f * kDeg));
    }
    const f32 heel = heelOf(f.body);
    printf("capsize box: heel %.2f deg after 60 s, worst over the last 10 s %.2f deg off 90\n",
           double(heel / kDeg), double(settledWorst / kDeg));
    CHECK_NEAR(fabsf(heel), 90.0f * kDeg, 2.0f * kDeg);
    CHECK(settledWorst < 2.0f * kDeg);
    CHECK(KM::length(f.body.angularVelocity()) < 0.01f);
    CHECK_NEAR(f.buoyancy.state().draft, 0.5, 0.01);
}
}

int main()
{
    archimedes();
    halfVolumes();
    floatingBoxDraft();
    heavePeriod();
    rightingMoment();
    returnsUpright();
    negativeStabilityCapsizes();
    return TEST_RESULT();
}
