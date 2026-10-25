#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/ConeTwistJoint.h>
#include <dynamics/FixedJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/PistonJoint.h>
#include <dynamics/PointJoint.h>
#include <dynamics/RigidBody.h>
#include <dynamics/SliderJoint.h>
#include <dynamics/UniversalJoint.h>

#include <math.h>
#include <stdio.h>

// The determinant scales with the cube (square) of the inverse masses, so an absolute
// singularity threshold zeroes the joint for bodies over about a tonne; a 1 kg body is the control.

using namespace kinetix;
using namespace kinetix::Physics;
namespace KM = kinetix::Math;

namespace
{
const f32 kStep = 1.0f / 60.0f;

struct Hang
{
    PhysicsWorld world;
    RigidBody anchor;
    RigidBody bob;
    // The world only moves bodies that have a shape.
    BoxShape anchorShape{KM::vec3(0.1f)};
    BoxShape bobShape{KM::vec3(1.2f, 1.3f, 6.8f)};
    KM::vec3 worldAnchor{0.0f, 5.0f, 0.0f};
    KM::vec3 localOnBob{0.0f};

    explicit Hang(f32 mass)
    {
        world.setGravity(KM::vec3(0.0f, -9.81f, 0.0f));
        anchor.setBodyType(BodyType::Static);
        anchor.setPosition(worldAnchor);
        anchor.calculateDerivedData();
        const KM::vec3 half(1.2f, 1.3f, 6.8f);
        bob.setMass(mass);
        bob.setInertiaTensor(Inertia::box(mass, half));
        bob.setDamping(1.0f, 1.0f);
        bob.setCanSleep(false);
        bob.setPosition(worldAnchor - KM::vec3(0.0f, 0.0f, 4.6f));
        bob.calculateDerivedData();
        localOnBob = bob.pointToLocal(worldAnchor);
        BodyEntry a;
        a.body = &anchor;
        a.shape = &anchorShape;
        const u32 idA = world.addBody(a);
        BodyEntry b;
        b.body = &bob;
        b.shape = &bobShape;
        const u32 idB = world.addBody(b);
        world.setPairIgnored(idA, idB, true);
    }

    f32 separation() const
    {
        return KM::length(bob.pointToWorld(localOnBob) - worldAnchor);
    }

    // Worst anchor separation over `seconds`; `moved` is the centre of mass travel.
    f32 run(Joint& joint, f32 seconds, f32* moved = nullptr)
    {
        world.addJoint(&joint);
        const KM::vec3 start = bob.position();
        f32 worst = 0.0f;
        f32 farthest = 0.0f;
        const int steps = static_cast<int>(seconds / kStep + 0.5f);
        for (int i = 0; i < steps; ++i)
        {
            world.step(kStep);
            worst = fmaxf(worst, separation());
            farthest = fmaxf(farthest, KM::length(bob.position() - start));
        }
        world.removeJoint(&joint);
        if (moved)
            *moved = farthest;
        return worst;
    }
};

void check(const char* name, f32 heavy, f32 light)
{
    printf("%-12s worst anchor separation: %.2e m at 20 t, %.2e m at 1 kg\n", name, double(heavy), double(light));
    CHECK(heavy < 0.01f);
    CHECK(light < 0.01f);
}

// `swings`: the joint lets the bob swing down (all but the fixed joint),
// so it must really have hung from it.
template <typename Make>
void hangs(const char* name, Make make, bool swings = true)
{
    f32 result[2];
    const f32 masses[2] = {20000.0f, 1.0f};
    for (u32 k = 0; k < 2; ++k)
    {
        Hang h(masses[k]);
        Joint* joint = make(h);
        f32 moved = 0.0f;
        result[k] = h.run(*joint, 2.0f, &moved);
        CHECK(isfinite(h.bob.position().x) && isfinite(h.bob.position().y));
        if (swings)
            CHECK(moved > 1.0f);
        else
            CHECK(moved < 0.02f);
        delete joint;
    }
    check(name, result[0], result[1]);
}
}

int main()
{
    const KM::vec3 x(1.0f, 0.0f, 0.0f);
    const KM::vec3 z(0.0f, 0.0f, 1.0f);
    hangs("point",
          [](Hang& h) -> Joint*
          {
              return new PointJoint(h.anchor, h.bob, h.worldAnchor);
          });
    hangs("hinge",
          [&](Hang& h) -> Joint*
          {
              return new HingeJoint(h.anchor, h.bob, h.worldAnchor, x);
          });
    hangs("cone-twist",
          [&](Hang& h) -> Joint*
          {
              ConeTwistJoint* joint = new ConeTwistJoint(h.anchor, h.bob, h.worldAnchor, z, x);
              joint->setSwingLimits(1.4f, 1.4f);
              return joint;
          });
    hangs("universal",
          [&](Hang& h) -> Joint*
          {
              return new UniversalJoint(h.anchor, h.bob, h.worldAnchor, x, KM::vec3(0.0f, 1.0f, 0.0f));
          });
    hangs("fixed",
          [](Hang& h) -> Joint*
          {
              return new FixedJoint(h.anchor, h.bob, h.worldAnchor);
          },
          false);
    {
        f32 result[2];
        const f32 masses[2] = {20000.0f, 1.0f};
        for (u32 k = 0; k < 2; ++k)
        {
            Hang h(masses[k]);
            SliderJoint slider(h.anchor, h.bob, h.worldAnchor, z);
            slider.setLimits(-0.01f, 0.01f);
            result[k] = h.run(slider, 2.0f);
        }
        check("slider", result[0], result[1]);
        for (u32 k = 0; k < 2; ++k)
        {
            Hang h(masses[k]);
            PistonJoint piston(h.anchor, h.bob, h.worldAnchor, z);
            piston.setLinearLimits(-0.01f, 0.01f);
            result[k] = h.run(piston, 2.0f);
        }
        check("piston", result[0], result[1]);
    }
    return TEST_RESULT();
}
