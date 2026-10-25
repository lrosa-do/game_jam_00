#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/PointJoint.h>
#include <dynamics/RigidBody.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kStep = 1.0f / 120.0f;

void makeDynamicBox(RigidBody& body, const KM::vec3& position, f32 halfExtent)
{
    body.setBodyType(BodyType::Dynamic);
    body.setMass(1.0f);
    body.setInertiaTensor(Inertia::box(1.0f, KM::vec3(halfExtent)));
    body.setPosition(position);
}

void makeFixedBox(RigidBody& body, const KM::vec3& position, BodyType type)
{
    body.setBodyType(type);
    body.setPosition(position);
}

bool settle(PhysicsWorld& world, const RigidBody& body)
{
    for (int i = 0; i < 3000; ++i)
    {
        world.step(kStep);
        if (!body.awake())
            return true;
    }
    return false;
}

void stepFor(PhysicsWorld& world, f32 seconds)
{
    const int steps = static_cast<int>(seconds / kStep);
    for (int i = 0; i < steps; ++i)
        world.step(kStep);
}

void removedSupportWakesBodyOnTop()
{
    PhysicsWorld world;
    RigidBody ground;
    BoxShape groundShape(KM::vec3(10.0f, 0.5f, 10.0f));
    makeFixedBox(ground, KM::vec3(0.0f, -0.5f, 0.0f), BodyType::Static);
    world.addBody({&ground, &groundShape});

    RigidBody bottom;
    RigidBody top;
    BoxShape shape(KM::vec3(0.5f));
    makeDynamicBox(bottom, KM::vec3(0.0f, 0.5f, 0.0f), 0.5f);
    makeDynamicBox(top, KM::vec3(0.0f, 1.5f, 0.0f), 0.5f);
    const u32 bottomId = world.addBody({&bottom, &shape});
    world.addBody({&top, &shape});
    CHECK(settle(world, top));

    world.removeBody(bottomId);
    stepFor(world, 2.0f);
    CHECK_NEAR(top.position().y, 0.5, 0.1);
}

void removedStaticShelfWakesBody()
{
    PhysicsWorld world;
    RigidBody ground;
    BoxShape groundShape(KM::vec3(10.0f, 0.5f, 10.0f));
    makeFixedBox(ground, KM::vec3(0.0f, -0.5f, 0.0f), BodyType::Static);
    world.addBody({&ground, &groundShape});

    RigidBody shelf;
    BoxShape shelfShape(KM::vec3(1.0f, 0.1f, 1.0f));
    makeFixedBox(shelf, KM::vec3(0.0f, 3.0f, 0.0f), BodyType::Static);
    const u32 shelfId = world.addBody({&shelf, &shelfShape});

    RigidBody box;
    BoxShape shape(KM::vec3(0.5f));
    makeDynamicBox(box, KM::vec3(0.0f, 3.6f, 0.0f), 0.5f);
    world.addBody({&box, &shape});
    CHECK(settle(world, box));

    world.removeBody(shelfId);
    stepFor(world, 2.0f);
    CHECK_NEAR(box.position().y, 0.5, 0.1);
}

void removedJointWakesBody()
{
    PhysicsWorld world;
    RigidBody anchor;
    BoxShape anchorShape(KM::vec3(0.1f));
    makeFixedBox(anchor, KM::vec3(0.0f), BodyType::Static);
    world.addBody({&anchor, &anchorShape});

    RigidBody bob;
    BoxShape bobShape(KM::vec3(0.2f));
    makeDynamicBox(bob, KM::vec3(0.0f, -1.0f, 0.0f), 0.2f);
    BodyEntry entry{&bob, &bobShape};
    entry.filter.mask = 0;
    world.addBody(entry);
    PointJoint joint(anchor, bob, KM::vec3(0.0f));
    world.addJoint(&joint);
    CHECK(settle(world, bob));

    world.removeJoint(&joint);
    stepFor(world, 0.5f);
    CHECK(bob.awake());
    CHECK(bob.position().y < -1.5f);
}

void movingKinematicPlatformWakesBody()
{
    PhysicsWorld world;
    RigidBody platform;
    BoxShape platformShape(KM::vec3(2.0f, 0.1f, 2.0f));
    makeFixedBox(platform, KM::vec3(0.0f, 5.0f, 0.0f), BodyType::Kinematic);
    world.addBody({&platform, &platformShape});

    RigidBody box;
    BoxShape shape(KM::vec3(0.3f));
    makeDynamicBox(box, KM::vec3(0.0f, 5.4f, 0.0f), 0.3f);
    world.addBody({&box, &shape});
    CHECK(settle(world, box));

    platform.setVelocity(KM::vec3(0.0f, -1.0f, 0.0f));
    stepFor(world, 2.0f);
    // Platform top is at y - 0.1; the box centre rests 0.3 above that.
    CHECK_NEAR(box.position().y, platform.position().y + 0.4, 0.05);

    platform.setVelocity(KM::vec3(0.0f, 0.5f, 0.0f));
    stepFor(world, 2.0f);
    CHECK_NEAR(box.position().y, platform.position().y + 0.4, 0.05);

    platform.setVelocity(KM::vec3(0.0f));
    CHECK(settle(world, box));
}

void lostContactWakesBody()
{
    PhysicsWorld world;
    RigidBody platform;
    BoxShape platformShape(KM::vec3(2.0f, 0.1f, 2.0f));
    makeFixedBox(platform, KM::vec3(0.0f, 5.0f, 0.0f), BodyType::Kinematic);
    world.addBody({&platform, &platformShape});

    RigidBody box;
    BoxShape shape(KM::vec3(0.3f));
    makeDynamicBox(box, KM::vec3(0.0f, 5.4f, 0.0f), 0.3f);
    world.addBody({&box, &shape});
    CHECK(settle(world, box));

    platform.setPosition(KM::vec3(0.0f, 2.0f, 0.0f));
    stepFor(world, 1.5f);
    CHECK_NEAR(box.position().y, 2.4, 0.05);
}

void movingKinematicJointWakesBody()
{
    PhysicsWorld world;
    RigidBody carrier;
    BoxShape carrierShape(KM::vec3(0.1f));
    makeFixedBox(carrier, KM::vec3(0.0f), BodyType::Kinematic);
    world.addBody({&carrier, &carrierShape});

    RigidBody bob;
    BoxShape bobShape(KM::vec3(0.2f));
    makeDynamicBox(bob, KM::vec3(0.0f, -1.0f, 0.0f), 0.2f);
    BodyEntry entry{&bob, &bobShape};
    entry.filter.mask = 0;
    world.addBody(entry);
    PointJoint joint(carrier, bob, KM::vec3(0.0f));
    world.addJoint(&joint);
    CHECK(settle(world, bob));

    carrier.setVelocity(KM::vec3(1.0f, 0.0f, 0.0f));
    stepFor(world, 2.0f);
    CHECK(bob.awake());
    CHECK(bob.position().x > 1.0f);
    CHECK(KM::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.05f);
}

void sleepingBodyIsNotMovedBySolver()
{
    PhysicsWorld world;
    RigidBody ground;
    BoxShape groundShape(KM::vec3(10.0f, 0.5f, 10.0f));
    makeFixedBox(ground, KM::vec3(0.0f, -0.5f, 0.0f), BodyType::Static);
    world.addBody({&ground, &groundShape});

    RigidBody box;
    BoxShape shape(KM::vec3(0.5f));
    makeDynamicBox(box, KM::vec3(0.0f, 0.5f, 0.0f), 0.5f);
    world.addBody({&box, &shape});
    CHECK(settle(world, box));

    // Sunk a little further than the solver would leave it - setPosition()
    // does not wake - so a position pass that still ran would push it out.
    box.setPosition(box.position() - KM::vec3(0.0f, 0.03f, 0.0f));
    const KM::vec3 restingAt = box.position();
    bool stayedAsleep = true;
    bool stayedPut = true;
    bool stayedStill = true;
    for (int i = 0; i < 120; ++i)
    {
        world.step(kStep);
        stayedAsleep = stayedAsleep && !box.awake();
        stayedPut = stayedPut && box.position().y == restingAt.y;
        stayedStill = stayedStill && KM::length(box.velocity()) == 0.0f &&
                      KM::length(box.angularVelocity()) == 0.0f;
    }
    CHECK(stayedAsleep);
    CHECK(stayedPut);
    CHECK(stayedStill);

    box.setAwake(true);
    stepFor(world, 0.5f);
    CHECK(box.position().y > restingAt.y + 0.02f);
}

void jointIslandSurvivesSlotMoves()
{
    PhysicsWorld world;
    world.setGravity(KM::vec3(0.0f));
    RigidBody filler[3];
    BoxShape fillerShape(KM::vec3(0.1f));
    u32 fillerIds[3];
    for (int i = 0; i < 3; ++i)
    {
        makeFixedBox(filler[i], KM::vec3(20.0f + 5.0f * float(i), 0.0f, 0.0f), BodyType::Static);
        fillerIds[i] = world.addBody({&filler[i], &fillerShape});
    }

    RigidBody a;
    RigidBody b;
    BoxShape shape(KM::vec3(0.2f));
    makeDynamicBox(a, KM::vec3(0.0f), 0.2f);
    makeDynamicBox(b, KM::vec3(1.0f, 0.0f, 0.0f), 0.2f);
    BodyEntry entryA{&a, &shape};
    BodyEntry entryB{&b, &shape};
    entryA.filter.mask = 0;
    entryB.filter.mask = 0;
    world.addBody(entryA);
    world.addBody(entryB);
    PointJoint joint(a, b, KM::vec3(0.5f, 0.0f, 0.0f));
    world.addJoint(&joint);
    CHECK(settle(world, b));
    CHECK(!a.awake());

    // Swap-pops b and then a into the freed slots.
    world.removeBody(fillerIds[0]);
    world.removeBody(fillerIds[1]);

    // Only a is pushed. The joint is what wakes b and drags it along.
    a.applyLinearImpulse(KM::vec3(0.0f, 0.0f, 4.0f));
    stepFor(world, 0.5f);
    CHECK(b.awake());
    CHECK(b.position().z > 0.2f);
}
}

int main()
{
    removedSupportWakesBodyOnTop();
    removedStaticShelfWakesBody();
    removedJointWakesBody();
    movingKinematicPlatformWakesBody();
    lostContactWakesBody();
    movingKinematicJointWakesBody();
    sleepingBodyIsNotMovedBySolver();
    jointIslandSurvivesSlotMoves();
    return TEST_RESULT();
}
