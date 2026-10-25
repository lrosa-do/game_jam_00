#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <dynamics/UniversalJoint.h>

using namespace kinetix;
using namespace kinetix::Physics;

namespace
{

void makeDynamicBox(RigidBody& body, const kinetix::Math::vec3& halfExtents, f32 mass)
{
    body.setBodyType(BodyType::Dynamic);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
}

f32 axesDot(const RigidBody& a, const RigidBody& b)
{
    return kinetix::Math::dot(a.directionToWorld(kinetix::Math::vec3(1.0f, 0.0f, 0.0f)),
                              b.directionToWorld(kinetix::Math::vec3(0.0f, 0.0f, 1.0f)));
}

void positionPassConverges()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b, kinetix::Math::vec3(0.5f), 1.0f);
    b.setPosition(kinetix::Math::vec3(0.0f, -1.0f, 0.0f));
    UniversalJoint joint(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(1.0f, 0.0f, 0.0f),
                         kinetix::Math::vec3(0.0f, 0.0f, 1.0f));
    // Tilting B about Y swings its Z axis towards A's X axis.
    b.setOrientation(kinetix::Math::angleAxis(0.05f, kinetix::Math::vec3(0.0f, 1.0f, 0.0f)));
    const f32 before = fabsf(axesDot(a, b));
    f32 previous = before;
    for (int pass = 0; pass < 20; ++pass)
    {
        joint.solvePosition(0.2f);
        const f32 now = fabsf(axesDot(a, b));
        CHECK(now < previous);
        previous = now;
    }
    CHECK(previous < 0.1f * before);
}

void spinningCrossStaysPerpendicular()
{
    PhysicsWorld world;
    world.setGravity(kinetix::Math::vec3(0.0f, -9.81f, 0.0f));
    const kinetix::Math::vec3 halfExtents(0.2f, 0.5f, 0.2f);
    RigidBody a;
    a.setBodyType(BodyType::Static);
    BoxShape anchorShape(kinetix::Math::vec3(0.1f));
    BodyEntry anchorEntry{&a, &anchorShape};
    anchorEntry.filter.mask = 0;
    world.addBody(anchorEntry);

    RigidBody b;
    makeDynamicBox(b, halfExtents, 1.0f);
    b.setPosition(kinetix::Math::vec3(0.0f, -0.5f, 0.0f));
    b.setCanSleep(false);
    BoxShape shape(halfExtents);
    BodyEntry entry{&b, &shape};
    entry.filter.mask = 0;
    world.addBody(entry);

    UniversalJoint joint(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(1.0f, 0.0f, 0.0f),
                         kinetix::Math::vec3(0.0f, 0.0f, 1.0f));
    world.addJoint(&joint);
    b.setAngularVelocity(kinetix::Math::vec3(2.0f, 1.5f, -3.0f));

    f32 worst = 0.0f;
    for (int i = 0; i < 600; ++i)
    {
        world.step(1.0f / 60.0f);
        worst = fmaxf(worst, fabsf(axesDot(a, b)));
    }
    CHECK(isfinite(worst));
    CHECK(worst < 0.05f);
}

}

int main()
{
    positionPassConverges();
    spinningCrossStaysPerpendicular();
    return TEST_RESULT();
}
