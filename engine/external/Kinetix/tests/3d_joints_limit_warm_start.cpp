#include "check.h"

#include <dynamics/ConeTwistJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/PistonJoint.h>
#include <dynamics/RigidBody.h>
#include <dynamics/SliderJoint.h>
#include <dynamics/UniversalJoint.h>
#include <dynamics/WheelJoint.h>

// The position pass may find the limit momentarily inside the range (another constraint moved
// the body back); it must not discard the velocity pass's impulse - only the next setup() decides contact.

using namespace kinetix;
using namespace kinetix::Physics;

namespace
{

const f32 kStep = 1.0f / 60.0f;

struct Pose
{
    kinetix::Math::vec3 position{0.0f};
    kinetix::Math::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    kinetix::Math::vec3 velocity{0.0f};
    kinetix::Math::vec3 angularVelocity{0.0f};
};

void place(RigidBody& body, const Pose& pose)
{
    body.setPosition(pose.position);
    body.setOrientation(pose.orientation);
    body.setVelocity(pose.velocity);
    body.setAngularVelocity(pose.angularVelocity);
}

void makeDynamicBox(RigidBody& body, const kinetix::Math::vec3& halfExtents)
{
    body.setBodyType(BodyType::Dynamic);
    body.setMass(1.0f);
    body.setInertiaTensor(Inertia::box(1.0f, halfExtents));
}

kinetix::Math::quat turn(f32 angle, const kinetix::Math::vec3& axis)
{
    return kinetix::Math::angleAxis(angle, axis);
}

struct Kick
{
    kinetix::Math::vec3 linear{0.0f};
    kinetix::Math::vec3 angular{0.0f};
};

template <typename MakeJoint>
Kick nextWarmStart(MakeJoint makeJoint, const Pose& rest, const Pose& outside, const Pose& inside,
                   bool visitInside)
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b, kinetix::Math::vec3(0.5f, 0.2f, 0.3f));
    place(b, rest);
    auto joint = makeJoint(a, b);

    place(b, outside);
    joint->setup(kStep);
    joint->warmStart();
    for (int i = 0; i < 4; ++i)
        joint->solveVelocity();

    if (visitInside)
    {
        place(b, inside);
        joint->solvePosition(0.2f);
    }

    place(b, outside);
    joint->setup(kStep);
    joint->warmStart();
    Kick kick;
    kick.linear = b.velocity() - outside.velocity;
    kick.angular = b.angularVelocity() - outside.angularVelocity;
    delete joint;
    return kick;
}

template <typename MakeJoint>
void positionPassKeepsLimitImpulse(const char* what, MakeJoint makeJoint, const Pose& rest,
                                   const Pose& outside, const Pose& inside)
{
    const Kick control = nextWarmStart(makeJoint, rest, outside, inside, false);
    const Kick visited = nextWarmStart(makeJoint, rest, outside, inside, true);
    const f32 size = kinetix::Math::length(control.angular) + kinetix::Math::length(control.linear);
    const f32 difference = kinetix::Math::length(visited.angular - control.angular) +
                           kinetix::Math::length(visited.linear - control.linear);
    if (!(size > 1.0e-3f) || !(difference < 1.0e-4f * size + 1.0e-6f))
        fprintf(stderr, "%s: warm start %g, after an inside position pass off by %g\n", what, size,
                difference);
    CHECK(size > 1.0e-3f);
    CHECK(difference < 1.0e-4f * size + 1.0e-6f);
}

}

int main()
{
    const kinetix::Math::vec3 x(1.0f, 0.0f, 0.0f);
    const kinetix::Math::vec3 y(0.0f, 1.0f, 0.0f);
    const kinetix::Math::vec3 z(0.0f, 0.0f, 1.0f);

    // Hinge about Y through the origin, B out along +X, pushed past -0.5.
    {
        Pose rest;
        rest.position = kinetix::Math::vec3(0.5f, 0.0f, 0.0f);
        Pose outside = rest;
        outside.orientation = turn(-0.6f, y);
        outside.position = outside.orientation * rest.position;
        outside.angularVelocity = kinetix::Math::vec3(0.0f, -2.0f, 0.0f);
        Pose inside = rest;
        positionPassKeepsLimitImpulse("hinge", [&](RigidBody& a, RigidBody& b)
        {
            HingeJoint* joint = new HingeJoint(a, b, kinetix::Math::vec3(0.0f), y);
            joint->setLimits(-0.5f, 0.5f);
            return joint;
        }, rest, outside, inside);
    }

    {
        Pose rest;
        rest.position = kinetix::Math::vec3(1.0f, 0.0f, 0.0f);
        Pose outside = rest;
        outside.orientation = turn(-0.6f, x);
        outside.angularVelocity = kinetix::Math::vec3(-2.0f, 0.0f, 0.0f);
        positionPassKeepsLimitImpulse("piston angular", [&](RigidBody& a, RigidBody& b)
        {
            PistonJoint* joint = new PistonJoint(a, b, kinetix::Math::vec3(0.0f), x);
            joint->setAngularLimits(-0.5f, 0.5f);
            return joint;
        }, rest, outside, rest);

        Pose pushed = rest;
        pushed.position = kinetix::Math::vec3(0.9f, 0.0f, 0.0f);
        pushed.velocity = kinetix::Math::vec3(-2.0f, 0.0f, 0.0f);
        positionPassKeepsLimitImpulse("piston linear", [&](RigidBody& a, RigidBody& b)
        {
            PistonJoint* joint = new PistonJoint(a, b, kinetix::Math::vec3(0.0f), x);
            joint->setLinearLimits(-0.05f, 0.5f);
            return joint;
        }, rest, pushed, rest);
    }

    {
        Pose rest;
        rest.position = kinetix::Math::vec3(1.0f, 0.0f, 0.0f);
        Pose pushed = rest;
        pushed.position = kinetix::Math::vec3(0.9f, 0.0f, 0.0f);
        pushed.velocity = kinetix::Math::vec3(-2.0f, 0.0f, 0.0f);
        positionPassKeepsLimitImpulse("slider", [&](RigidBody& a, RigidBody& b)
        {
            SliderJoint* joint = new SliderJoint(a, b, kinetix::Math::vec3(0.0f), x);
            joint->setLimits(-0.05f, 0.5f);
            return joint;
        }, rest, pushed, rest);
    }

    {
        Pose rest;
        rest.position = kinetix::Math::vec3(0.0f, -0.5f, 0.0f);
        Pose twisted = rest;
        twisted.orientation = turn(0.6f, -y);
        twisted.angularVelocity = kinetix::Math::vec3(0.0f, -2.0f, 0.0f);
        positionPassKeepsLimitImpulse("cone twist", [&](RigidBody& a, RigidBody& b)
        {
            ConeTwistJoint* joint = new ConeTwistJoint(a, b, kinetix::Math::vec3(0.0f), -y, z);
            joint->setTwistLimits(-0.5f, 0.5f);
            return joint;
        }, rest, twisted, rest);

        Pose swung = rest;
        swung.orientation = turn(0.6f, x);
        swung.position = swung.orientation * rest.position;
        swung.angularVelocity = kinetix::Math::vec3(2.0f, 0.0f, 0.0f);
        positionPassKeepsLimitImpulse("cone swing", [&](RigidBody& a, RigidBody& b)
        {
            ConeTwistJoint* joint = new ConeTwistJoint(a, b, kinetix::Math::vec3(0.0f), -y, z);
            joint->setSwingLimits(0.5f, 0.5f);
            return joint;
        }, rest, swung, rest);
    }

    // Universal: A's axis X, B's axis Z, B hanging; limited about X.
    {
        Pose rest;
        rest.position = kinetix::Math::vec3(0.0f, -0.5f, 0.0f);
        Pose outside = rest;
        outside.orientation = turn(0.6f, x);
        outside.position = outside.orientation * rest.position;
        outside.angularVelocity = kinetix::Math::vec3(2.0f, 0.0f, 0.0f);
        positionPassKeepsLimitImpulse("universal", [&](RigidBody& a, RigidBody& b)
        {
            UniversalJoint* joint = new UniversalJoint(a, b, kinetix::Math::vec3(0.0f), x, z);
            joint->setLimitsA(-0.5f, 0.5f);
            return joint;
        }, rest, outside, rest);
    }

    {
        Pose rest;
        rest.position = kinetix::Math::vec3(0.0f, -1.0f, 0.0f);
        Pose steered = rest;
        steered.orientation = turn(0.6f, -y);
        steered.angularVelocity = kinetix::Math::vec3(0.0f, -2.0f, 0.0f);
        positionPassKeepsLimitImpulse("wheel steering", [&](RigidBody& a, RigidBody& b)
        {
            WheelJoint* joint = new WheelJoint(a, b, kinetix::Math::vec3(0.0f, -1.0f, 0.0f), -y, x);
            joint->setSteeringLimits(-0.5f, 0.5f);
            return joint;
        }, rest, steered, rest);
    }

    return TEST_RESULT();
}
