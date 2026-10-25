#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/ConeTwistJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/PistonJoint.h>
#include <dynamics/RigidBody.h>
#include <dynamics/UniversalJoint.h>
#include <dynamics/WheelJoint.h>

// Angular limits are any range inside [-pi, pi], not only around the build pose.

using namespace kinetix;
using namespace kinetix::Physics;

namespace
{

const f32 kStep = 1.0f / 120.0f;
const f32 kTolerance = 0.03f;

struct Rig
{
    PhysicsWorld world;
    RigidBody a;
    RigidBody b;
    BoxShape anchorShape{kinetix::Math::vec3(0.1f)};
    BoxShape shape;

    Rig(const kinetix::Math::vec3& halfExtents, const kinetix::Math::vec3& position)
        : shape(halfExtents)
    {
        world.setGravity(kinetix::Math::vec3(0.0f));
        a.setBodyType(BodyType::Static);
        BodyEntry anchorEntry{&a, &anchorShape};
        anchorEntry.filter.mask = 0;
        world.addBody(anchorEntry);

        b.setBodyType(BodyType::Dynamic);
        b.setMass(1.0f);
        b.setInertiaTensor(Inertia::box(1.0f, halfExtents));
        b.setPosition(position);
        b.setCanSleep(false);
        BodyEntry entry{&b, &shape};
        entry.filter.mask = 0;
        world.addBody(entry);
    }
};

struct Range
{
    f32 lowest = 1.0e9f;
    f32 highest = -1.0e9f;
};

// Angles seen in the second of two seconds, once the joint has pulled the body in.
template <typename AngleFn>
Range settledRange(Rig& rig, AngleFn angle)
{
    Range range;
    for (int i = 0; i < 240; ++i)
    {
        rig.world.step(kStep);
        if (i < 120)
            continue;
        const f32 value = angle();
        if (!isfinite(value))
            return Range{-1.0e9f, 1.0e9f};
        range.lowest = fminf(range.lowest, value);
        range.highest = fmaxf(range.highest, value);
    }
    return range;
}

void checkInside(const Range& range, f32 minAngle, f32 maxAngle, const char* what, f32 spin)
{
    const bool inside = range.lowest >= minAngle - kTolerance && range.highest <= maxAngle + kTolerance;
    if (!inside)
        fprintf(stderr, "%s [%g, %g] spin %+g: settled in [%g, %g]\n", what, minAngle, maxAngle,
                spin, range.lowest, range.highest);
    CHECK(inside);
}

void hinge(f32 minAngle, f32 maxAngle, f32 spin)
{
    Rig rig(kinetix::Math::vec3(0.5f, 0.1f, 0.1f), kinetix::Math::vec3(0.5f, 0.0f, 0.0f));
    HingeJoint joint(rig.a, rig.b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f, 1.0f, 0.0f));
    joint.setLimits(minAngle, maxAngle);
    rig.world.addJoint(&joint);
    rig.b.setAngularVelocity(kinetix::Math::vec3(0.0f, spin, 0.0f));
    checkInside(settledRange(rig, [&] { return joint.currentAngle(); }), minAngle, maxAngle,
                "hinge", spin);
}

void piston(f32 minAngle, f32 maxAngle, f32 spin)
{
    Rig rig(kinetix::Math::vec3(0.5f, 0.1f, 0.3f), kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    PistonJoint joint(rig.a, rig.b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    joint.setAngularLimits(minAngle, maxAngle);
    joint.setLinearLimits(1.0f, 1.0f);
    rig.world.addJoint(&joint);
    rig.b.setAngularVelocity(kinetix::Math::vec3(spin, 0.0f, 0.0f));
    checkInside(settledRange(rig, [&] { return joint.currentAngle(); }), minAngle, maxAngle,
                "piston", spin);
}

void universal(f32 minAngle, f32 maxAngle, f32 spin)
{
    Rig rig(kinetix::Math::vec3(0.1f, 0.5f, 0.1f), kinetix::Math::vec3(0.0f, -0.5f, 0.0f));
    UniversalJoint joint(rig.a, rig.b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(1.0f, 0.0f, 0.0f),
                         kinetix::Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setLimitsA(minAngle, maxAngle);
    joint.setLimitsB(-0.01f, 0.01f);
    rig.world.addJoint(&joint);
    rig.b.setAngularVelocity(kinetix::Math::vec3(spin, 0.0f, 0.0f));
    checkInside(settledRange(rig, [&] { return joint.currentAngleA(); }), minAngle, maxAngle,
                "universal", spin);
}

void coneTwist(f32 minAngle, f32 maxAngle, f32 spin)
{
    Rig rig(kinetix::Math::vec3(0.1f, 0.5f, 0.3f), kinetix::Math::vec3(0.0f, -0.5f, 0.0f));
    ConeTwistJoint joint(rig.a, rig.b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f, -1.0f, 0.0f),
                         kinetix::Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setTwistLimits(minAngle, maxAngle);
    joint.setSwingLimits(0.2f, 0.2f);
    rig.world.addJoint(&joint);
    // Twist is measured about the twist axis, -Y.
    rig.b.setAngularVelocity(kinetix::Math::vec3(0.0f, -spin, 0.0f));
    const auto twist = [&]
    {
        const kinetix::Math::quat q = rig.b.orientation();
        return 2.0f * atanf(-q.y / q.w);
    };
    checkInside(settledRange(rig, twist), minAngle, maxAngle, "cone twist", spin);
}

void wheelSteering(f32 minAngle, f32 maxAngle, f32 spin)
{
    Rig rig(kinetix::Math::vec3(0.1f, 0.3f, 0.3f), kinetix::Math::vec3(0.0f, -1.0f, 0.0f));
    WheelJoint joint(rig.a, rig.b, kinetix::Math::vec3(0.0f, -1.0f, 0.0f),
                     kinetix::Math::vec3(0.0f, -1.0f, 0.0f), kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    joint.setSteeringLimits(minAngle, maxAngle);
    rig.world.addJoint(&joint);
    // Steering turns about the suspension axis, -Y.
    rig.b.setAngularVelocity(kinetix::Math::vec3(0.0f, -spin, 0.0f));
    checkInside(settledRange(rig, [&] { return joint.steeringAngle(); }), minAngle, maxAngle,
                "wheel steering", spin);
}

template <typename Case>
void allSpins(Case run, f32 minAngle, f32 maxAngle)
{
    run(minAngle, maxAngle, 0.0f);
    run(minAngle, maxAngle, 5.0f);
    run(minAngle, maxAngle, -5.0f);
}

template <typename Case>
void allRanges(Case run)
{
    allSpins(run, -0.5f, 0.8f);
    allSpins(run, 0.3f, 0.8f);
    allSpins(run, -0.8f, -0.3f);
    allSpins(run, 0.4f, 0.4f);
}

}

int main()
{
    allRanges(hinge);
    allRanges(piston);
    allRanges(universal);
    allRanges(coneTwist);
    allRanges(wheelSteering);

    // Reversed bounds collapse onto the minimum instead of being reordered.
    {
        RigidBody a;
        a.setBodyType(BodyType::Static);
        RigidBody b;
        b.setBodyType(BodyType::Dynamic);
        ConeTwistJoint cone(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f, -1.0f, 0.0f),
                            kinetix::Math::vec3(0.0f, 0.0f, 1.0f));
        cone.setTwistLimits(0.6f, 0.2f);
        CHECK_NEAR(cone.twistMinAngle(), 0.6f, 1.0e-6f);
        CHECK_NEAR(cone.twistMaxAngle(), 0.6f, 1.0e-6f);
        cone.setTwistLimits(-5.0f, 5.0f);
        CHECK_NEAR(cone.twistMinAngle(), -kinetix::Math::pi<f32>(), 1.0e-6f);
        CHECK_NEAR(cone.twistMaxAngle(), kinetix::Math::pi<f32>(), 1.0e-6f);
    }
    return TEST_RESULT();
}
