#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
RigidBody* makeBox(RigidBody& body, const KM::vec3& position, const KM::vec3& halfExtents,
                   bool isStatic, f32 mass = 1.0f)
{
    body.setPosition(position);
    body.setBodyType(isStatic ? BodyType::Static : BodyType::Dynamic);
    if (!isStatic)
    {
        body.setMass(mass);
        body.setInertiaTensor(Inertia::box(mass, halfExtents));
    }
    return &body;
}

void stackOfFive()
{
    constexpr int count = 5;
    PhysicsWorld world;
    RigidBody ground;
    BoxShape groundShape(KM::vec3(20.0f, 1.0f, 20.0f));
    makeBox(ground, KM::vec3(0.0f, -1.0f, 0.0f), KM::vec3(20.0f, 1.0f, 20.0f), true);
    BodyEntry groundEntry;
    groundEntry.body = &ground;
    groundEntry.shape = &groundShape;
    groundEntry.friction = 0.55f;
    world.addBody(groundEntry);

    RigidBody boxes[count];
    BoxShape boxShape(KM::vec3(0.5f));
    for (int i = 0; i < count; ++i)
    {
        makeBox(boxes[i], KM::vec3(0.0f, 0.5f + float(i) * 1.05f, 0.0f), KM::vec3(0.5f), false);
        BodyEntry entry;
        entry.body = &boxes[i];
        entry.shape = &boxShape;
        entry.friction = 0.55f;
        world.addBody(entry);
    }

    for (int i = 0; i < 600; ++i)
        world.step(1.0f / 60.0f);

    f32 drift = 0.0f;
    for (int i = 0; i < count; ++i)
    {
        const KM::vec3 p = boxes[i].position();
        drift = fmaxf(drift, sqrtf(p.x * p.x + p.z * p.z));
        if (i > 0)
            CHECK(boxes[i].position().y > boxes[i - 1].position().y);
    }
    // Resting heights are 0.5, 1.5 ... 4.5; a collapsed stack ends near 0.5.
    CHECK_NEAR(boxes[count - 1].position().y, 4.5, 0.15);
    CHECK(drift < 1.0f);
}

void warmStartFollowsLoad()
{
    PhysicsWorld world;
    RigidBody ground;
    BoxShape groundShape(KM::vec3(10.0f, 0.5f, 10.0f));
    makeBox(ground, KM::vec3(0.0f, -0.5f, 0.0f), KM::vec3(10.0f, 0.5f, 10.0f), true);
    world.addBody({&ground, &groundShape});

    RigidBody box;
    BoxShape boxShape(KM::vec3(0.5f));
    makeBox(box, KM::vec3(0.0f, 0.5f, 0.0f), KM::vec3(0.5f), false);
    box.setCanSleep(false);
    world.addBody({&box, &boxShape});

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 120; ++i)
        world.step(dt);

    const f32 heavy = 20.0f;
    box.setMass(heavy);
    box.setInertiaTensor(Inertia::box(heavy, KM::vec3(0.5f)));
    for (int i = 0; i < 240; ++i)
        world.step(dt);

    // Step once more with the warm start alone: no iterations, so the
    // impulse the step ends with is exactly what the cache handed it.
    ContactSolverSettings settings;
    settings.velocityIterations = 0;
    world.setSolverSettings(settings);
    world.step(dt);

    f32 total = 0.0f;
    for (const Contact& contact : world.contacts())
        for (u32 i = 0; i < contact.manifold.count; ++i)
            total += contact.manifold.points[i].normalImpulse;
    // Holding the heavy box up takes m * g * dt per step.
    const f32 expected = heavy * 9.81f * dt;
    CHECK_NEAR(total, expected, expected * 0.1f);
}
}

int main()
{
    stackOfFive();
    warmStartFollowsLoad();
    return TEST_RESULT();
}
