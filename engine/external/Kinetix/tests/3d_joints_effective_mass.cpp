#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/DistanceJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/PistonJoint.h>
#include <dynamics/RigidBody.h>
#include <dynamics/SliderJoint.h>

// Effective mass of a scalar row:
// 1 / (mA^-1 + mB^-1 + (rA x n).IA^-1(rA x n) + (rB x n).IB^-1(rB x n)).
// One velocity iteration with it removes the row's velocity exactly.

using namespace kinetix;
using namespace kinetix::Physics;

namespace
{

const f32 kStep = 1.0f / 60.0f;

void makeDynamicBox(RigidBody& body, const kinetix::Math::vec3& halfExtents, f32 mass)
{
    body.setBodyType(BodyType::Dynamic);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
}

// Anchor far from the centre of mass: the angular term dominates.
void distanceRowIsSolvedInOneIteration()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b, kinetix::Math::vec3(1.0f, 0.1f, 0.1f), 1.0f);
    b.setPosition(kinetix::Math::vec3(0.0f, -2.0f, 0.0f));
    const kinetix::Math::vec3 anchorA(1.0f, 0.0f, 0.0f);
    const kinetix::Math::vec3 anchorB(1.0f, -2.0f, 0.0f);
    DistanceJoint joint(a, anchorA, b, anchorB);
    b.setVelocity(kinetix::Math::vec3(0.0f, -3.0f, 0.0f));

    joint.setup(kStep);
    joint.solveVelocity();

    const kinetix::Math::vec3 pointB = joint.worldAnchorB();
    const kinetix::Math::vec3 normal = kinetix::Math::normalize(pointB - anchorA);
    CHECK_NEAR(kinetix::Math::dot(b.velocityAtPoint(pointB), normal), 0.0f, 1.0e-4f);
}

// The limit row is solved last, so after one iteration B's anchor speed along the axis is what it left.
void sliderLimitRowIsSolvedInOneIteration()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b, kinetix::Math::vec3(1.0f, 0.1f, 0.1f), 1.0f);
    b.setPosition(kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    SliderJoint joint(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f, 1.0f, 0.0f));
    joint.setLimits(0.0f, 1.0f);
    b.setPosition(kinetix::Math::vec3(1.0f, -0.01f, 0.0f));
    b.setVelocity(kinetix::Math::vec3(0.0f, -3.0f, 0.0f));

    joint.setup(kStep);
    joint.solveVelocity();

    const kinetix::Math::vec3 anchorB = b.pointToWorld(kinetix::Math::vec3(-1.0f, 0.0f, 0.0f));
    CHECK_NEAR(b.velocityAtPoint(anchorB).y, 0.0f, 1.0e-4f);
}

void pistonLinearLimitRowIsSolvedInOneIteration()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b, kinetix::Math::vec3(1.0f, 0.1f, 0.1f), 1.0f);
    b.setPosition(kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    PistonJoint joint(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f, 1.0f, 0.0f));
    joint.setLinearLimits(0.0f, 1.0f);
    b.setPosition(kinetix::Math::vec3(1.0f, -0.01f, 0.0f));
    b.setVelocity(kinetix::Math::vec3(0.0f, -3.0f, 0.0f));

    joint.setup(kStep);
    joint.solveVelocity();

    const kinetix::Math::vec3 anchorB = b.pointToWorld(kinetix::Math::vec3(-1.0f, 0.0f, 0.0f));
    CHECK_NEAR(b.velocityAtPoint(anchorB).y, 0.0f, 1.0e-4f);
}

void offCentreRopeStaysBounded()
{
    PhysicsWorld world;
    world.setGravity(kinetix::Math::vec3(0.0f, -9.81f, 0.0f));
    RigidBody anchor;
    anchor.setBodyType(BodyType::Static);
    BoxShape anchorShape(kinetix::Math::vec3(0.1f));
    BodyEntry anchorEntry{&anchor, &anchorShape};
    anchorEntry.filter.mask = 0;
    world.addBody(anchorEntry);

    const kinetix::Math::vec3 halfExtents(1.0f, 0.1f, 0.1f);
    RigidBody bar;
    makeDynamicBox(bar, halfExtents, 1.0f);
    bar.setPosition(kinetix::Math::vec3(0.0f, -2.0f, 0.0f));
    bar.setCanSleep(false);
    BoxShape barShape(halfExtents);
    BodyEntry barEntry{&bar, &barShape};
    barEntry.filter.mask = 0;
    world.addBody(barEntry);

    DistanceJoint rope(anchor, kinetix::Math::vec3(1.0f, 0.0f, 0.0f), bar,
                       kinetix::Math::vec3(1.0f, -2.0f, 0.0f));
    world.addJoint(&rope);
    bar.setVelocity(kinetix::Math::vec3(0.0f, 0.0f, 3.0f));

    f32 worstStretch = 0.0f;
    f32 fastest = 0.0f;
    for (int i = 0; i < 600; ++i)
    {
        world.step(kStep);
        const f32 length = kinetix::Math::length(rope.worldAnchorB() - rope.worldAnchorA());
        worstStretch = fmaxf(worstStretch, fabsf(length - 2.0f));
        fastest = fmaxf(fastest, kinetix::Math::length(bar.velocity()));
    }
    CHECK(isfinite(worstStretch));
    CHECK(worstStretch < 0.05f);
    // Released at 3 m/s from rest height: energy alone caps it near 7 m/s.
    CHECK(fastest < 10.0f);
}

}

int main()
{
    distanceRowIsSolvedInOneIteration();
    sliderLimitRowIsSolvedInOneIteration();
    pistonLinearLimitRowIsSolvedInOneIteration();
    offCentreRopeStaysBounded();
    return TEST_RESULT();
}
