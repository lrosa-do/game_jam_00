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
constexpr f32 kRho = 1000.0f;
constexpr f32 kG = 9.81f;

bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

WaterProperties testProperties()
{
    WaterProperties properties;
    properties.density = kRho;
    properties.kinematicViscosity = 1.0e-6f;
    return properties;
}

struct Floating
{
    PhysicsWorld world;
    RigidBody body;
    BoxShape shape;
    BuoyancyBody buoyancy;
    WaterSurface* water;

    Floating(WaterSurface* surface, const KM::vec3& halfExtents, f32 mass, u32 subdivisions)
        : shape(halfExtents)
        , water(surface)
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
        const HullMesh hull = HullMesh::box(halfExtents, subdivisions);
        buoyancy = BuoyancyBody(&body, hull, water);
        body.setPosition(KM::vec3(0.0f, floatingHeight(hull, mass, kRho), 0.0f));
    }

    void step(f32 dt)
    {
        buoyancy.applyForces(dt);
        world.step(dt);
        water->advance(dt);
    }
};

// Heave amplitude of a 2 m box over the last two periods of 40 s.
f32 heaveAmplitude(f32 wavelength, f32 amplitude, f32* surgeAmplitude, f32* worstTilt)
{
    GerstnerWaves waves;
    waves.setProperties(testProperties());
    GerstnerWaves::Wave w;
    w.amplitude = amplitude;
    w.wavelength = wavelength;
    w.direction = KM::vec2(0.0f, 1.0f);
    waves.addWave(w);
    const KM::vec3 h(1.0f, 0.5f, 1.0f);
    Floating box(&waves, h, 0.5f * kRho * 8.0f * h.x * h.y * h.z, 6);
    const f32 rest = box.body.position().y;
    f32 lo = 1.0e9f;
    f32 hi = -1.0e9f;
    f32 zlo = 1.0e9f;
    f32 zhi = -1.0e9f;
    *worstTilt = 0.0f;
    const f32 period = waves.period(0);
    const int steps = static_cast<int>(40.0f * 60.0f);
    for (int i = 0; i < steps; ++i)
    {
        box.step(1.0f / 60.0f);
        if (static_cast<f32>(i) / 60.0f > 40.0f - 2.0f * period)
        {
            const KM::vec3 p = box.body.position();
            lo = fminf(lo, p.y - rest);
            hi = fmaxf(hi, p.y - rest);
            zlo = fminf(zlo, p.z);
            zhi = fmaxf(zhi, p.z);
        }
        const KM::vec3 up = box.body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f));
        *worstTilt = fmaxf(*worstTilt, acosf(fminf(up.y, 1.0f)));
    }
    CHECK(finiteVec(box.body.position()));
    *surgeAmplitude = 0.5f * (zhi - zlo);
    return 0.5f * (hi - lo);
}

void followsLongWaves()
{
    f32 surge = 0.0f;
    f32 tilt = 0.0f;
    // 80 m waves: 40 box lengths, T = 7.2 s. The box is a particle on them.
    const f32 heave = heaveAmplitude(80.0f, 0.3f, &surge, &tilt);
    printf("80 m waves, A = 0.3 m: heave amplitude %.3f m, surge amplitude %.3f m, worst tilt %.2f deg\n",
           double(heave), double(surge), double(tilt * 57.2958f));
    CHECK_NEAR(heave, 0.3, 0.03);
    CHECK_NEAR(surge, 0.3, 0.06);
    // The slope is k A = 0.024 rad: it pitches with the surface, gently.
    CHECK(tilt < 3.0f * 0.024f);

    // Waves as long as the box: a crest and a trough over its length at any
    // time, so the pressure averages out and it hardly heaves at all.
    const f32 shortHeave = heaveAmplitude(2.0f, 0.1f, &surge, &tilt);
    printf("2 m waves, A = 0.1 m: heave amplitude %.4f m\n", double(shortHeave));
    CHECK(shortHeave < 0.25f * 0.1f);
}

void flatGerstnerMatchesFlat()
{
    FlatWater flat(0.0f);
    flat.setProperties(testProperties());
    GerstnerWaves still(0.0f);
    still.setProperties(testProperties());
    const KM::vec3 h(0.8f, 0.4f, 1.5f);
    Floating a(&flat, h, 900.0f, 3);
    Floating b(&still, h, 900.0f, 3);
    for (Floating* f : {&a, &b})
    {
        f->body.setOrientation(KM::angleAxis(0.3f, KM::normalize(KM::vec3(1.0f, 0.2f, 0.5f))));
        f->body.setVelocity(KM::vec3(1.0f, 0.5f, -0.4f));
        f->body.setAngularVelocity(KM::vec3(0.2f, -0.3f, 0.1f));
    }
    bool identical = true;
    for (int i = 0; i < 600; ++i)
    {
        a.step(1.0f / 60.0f);
        b.step(1.0f / 60.0f);
        const KM::vec3 pa = a.body.position();
        const KM::vec3 pb = b.body.position();
        identical = identical && pa.x == pb.x && pa.y == pb.y && pa.z == pb.z &&
                    a.body.orientation().w == b.body.orientation().w;
    }
    CHECK(identical);
}

void floatsAgainstDock()
{
    FlatWater water(0.0f);
    water.setProperties(testProperties());
    const KM::vec3 h(1.0f, 0.5f, 1.0f);
    const f32 mass = 1000.0f;
    Floating box(&water, h, mass, 3);
    RigidBody dock;
    dock.setBodyType(BodyType::Static);
    const f32 dockCentre = h.z + 0.3f + 2.0f;
    dock.setPosition(KM::vec3(0.0f, 0.0f, dockCentre));
    dock.calculateDerivedData();
    BoxShape dockShape(KM::vec3(4.0f, 1.5f, 2.0f));
    BodyEntry entry;
    entry.body = &dock;
    entry.shape = &dockShape;
    box.world.addBody(entry);
    box.world.markStaticBroadphaseDirty();
    box.body.setVelocity(KM::vec3(0.0f, 0.0f, 1.5f));

    const f32 face = dockCentre - 2.0f - h.z;
    f32 deepest = 0.0f;
    f32 fastest = 0.0f;
    bool touched = false;
    bool finite = true;
    for (int i = 0; i < 60 * 20; ++i)
    {
        box.step(1.0f / 60.0f);
        const KM::vec3 p = box.body.position();
        finite = finite && finiteVec(p) && finiteVec(box.body.velocity());
        deepest = fmaxf(deepest, p.z - face);
        if (i > 5)
            fastest = fmaxf(fastest, KM::length(box.body.velocity()));
        if (box.world.contactCount() > 0)
            touched = true;
    }
    const KM::vec3 p = box.body.position();
    printf("dock: touched %d, deepest penetration %.4f m, fastest %.2f m/s, rests at z %.3f "
           "(%.3f m off the face), heel %.2f deg\n",
           int(touched), double(deepest), double(fastest), double(p.z), double(face - p.z),
           double(acosf(fminf(box.body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f)).y, 1.0f)) * 57.3f));
    CHECK(finite);
    CHECK(touched);
    CHECK(deepest < 0.05f);
    CHECK(fastest <= 1.5f + 1.0e-3f);
    CHECK(face - p.z < 0.5f);
    CHECK(KM::length(box.body.velocity()) < 0.05f);
    CHECK(box.body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f)).y > 0.99f);
    CHECK_NEAR(box.buoyancy.state().submergedVolume, mass / kRho, 0.02 * mass / kRho);
}
}

int main()
{
    followsLongWaves();
    flatGerstnerMatchesFlat();
    floatsAgainstDock();
    return TEST_RESULT();
}
