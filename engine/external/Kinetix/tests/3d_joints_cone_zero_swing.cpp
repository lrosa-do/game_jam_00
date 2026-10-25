#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/ConeTwistJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

using namespace kinetix;
using namespace kinetix::Physics;

namespace
{

struct SwingRange
{
    f32 aboutX = 0.0f;
    f32 aboutZ = 0.0f;
};

// B hangs below A along -Y, plane axis Z: half-angle "Y" governs swing about world X, "Z" about world Z.
SwingRange largestSwing(f32 halfAngleY, f32 halfAngleZ, const kinetix::Math::vec3& spin)
{
    PhysicsWorld world;
    world.setGravity(kinetix::Math::vec3(0.0f));
    RigidBody a;
    a.setBodyType(BodyType::Static);
    BoxShape anchorShape(kinetix::Math::vec3(0.1f));
    BodyEntry anchorEntry{&a, &anchorShape};
    anchorEntry.filter.mask = 0;
    world.addBody(anchorEntry);

    const kinetix::Math::vec3 halfExtents(0.1f, 0.5f, 0.3f);
    RigidBody b;
    b.setBodyType(BodyType::Dynamic);
    b.setMass(1.0f);
    b.setInertiaTensor(Inertia::box(1.0f, halfExtents));
    b.setPosition(kinetix::Math::vec3(0.0f, -0.5f, 0.0f));
    b.setCanSleep(false);
    BoxShape shape(halfExtents);
    BodyEntry entry{&b, &shape};
    entry.filter.mask = 0;
    world.addBody(entry);

    ConeTwistJoint joint(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f, -1.0f, 0.0f),
                         kinetix::Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setSwingLimits(halfAngleY, halfAngleZ);
    world.addJoint(&joint);
    b.setAngularVelocity(spin);

    SwingRange worst;
    for (int i = 0; i < 300; ++i)
    {
        world.step(1.0f / 120.0f);
        const kinetix::Math::vec3 axis = b.directionToWorld(kinetix::Math::vec3(0.0f, -1.0f, 0.0f));
        if (!isfinite(axis.x) || !isfinite(axis.y) || !isfinite(axis.z))
        {
            worst.aboutX = worst.aboutZ = 1.0e9f;
            break;
        }
        // Swinging about X tips the hanging axis into Z, about Z into X.
        worst.aboutX = fmaxf(worst.aboutX, fabsf(atan2f(axis.z, -axis.y)));
        worst.aboutZ = fmaxf(worst.aboutZ, fabsf(atan2f(axis.x, -axis.y)));
    }
    return worst;
}

}

int main()
{
    const kinetix::Math::vec3 spin(3.0f, 0.0f, 3.0f);

    const SwingRange halfLocked = largestSwing(0.0f, 0.5f, spin);
    CHECK(halfLocked.aboutX < 0.05f);
    CHECK(halfLocked.aboutZ > 0.4f);
    CHECK(halfLocked.aboutZ < 0.55f);

    const SwingRange otherLocked = largestSwing(0.5f, 0.0f, spin);
    CHECK(otherLocked.aboutZ < 0.05f);
    CHECK(otherLocked.aboutX > 0.4f);
    CHECK(otherLocked.aboutX < 0.55f);

    const SwingRange locked = largestSwing(0.0f, 0.0f, spin);
    CHECK(locked.aboutX < 0.05f);
    CHECK(locked.aboutZ < 0.05f);

    // Tiny but non-zero must not fall apart in the closest-point search on a needle-thin ellipse.
    const SwingRange needle = largestSwing(1.0e-4f, 0.5f, spin);
    CHECK(needle.aboutX < 0.05f);
    CHECK(needle.aboutZ > 0.4f);
    CHECK(needle.aboutZ < 0.55f);

    const SwingRange hair = largestSwing(1.0e-6f, 0.5f, spin);
    CHECK(hair.aboutX < 0.05f);
    CHECK(hair.aboutZ > 0.4f);
    CHECK(hair.aboutZ < 0.55f);

    const SwingRange tinyBoth = largestSwing(1.0e-6f, 1.0e-6f, spin);
    CHECK(tinyBoth.aboutX < 0.05f);
    CHECK(tinyBoth.aboutZ < 0.05f);

    const SwingRange cone = largestSwing(0.5f, 0.5f, spin);
    CHECK(cone.aboutX < 0.55f);
    CHECK(cone.aboutZ < 0.55f);
    CHECK(cone.aboutX > 0.2f);
    CHECK(cone.aboutZ > 0.2f);

    return TEST_RESULT();
}
