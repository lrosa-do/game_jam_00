#include "check.h"

#include <kx/joints/mousejoint.h>
#include <kx/joints/revolutejoint.h>
#include <kx/world.h>

using namespace kx;

int main()
{
    const float dt = 1.0f / 60.0f;

    {
        World world(vec2(0.0f, 500.0f));
        world.CreateStaticBox(vec2(0.0f, 0.0f), 200.0f, 10.0f);
        Body* single = world.CreateBox(vec2(-100.0f, -21.0f), 10.0f, 10.0f, 1.0f);
        Body* bottom = world.CreateBox(vec2(100.0f, -21.0f), 10.0f, 10.0f, 1.0f);
        Body* top = world.CreateBox(vec2(100.0f, -42.0f), 10.0f, 10.0f, 1.0f);
        for (int i = 0; i < 600; ++i)
            world.Step(dt);
        CHECK(!single->IsAwake());
        CHECK(!bottom->IsAwake());
        CHECK(!top->IsAwake());

        top->ApplyLinearImpulseToCenter(vec2(0.0f, -1.0f));
        world.Step(dt);
        CHECK(top->IsAwake());
        CHECK(bottom->IsAwake());
        CHECK(!single->IsAwake());

        for (int i = 0; i < 600; ++i)
            world.Step(dt);
        CHECK(!bottom->IsAwake());
        CHECK(!top->IsAwake());

        Body* dropped = world.CreateBox(vec2(100.0f, -120.0f), 10.0f, 10.0f, 1.0f);
        bool bottomWoke = false;
        for (int i = 0; i < 120; ++i)
        {
            world.Step(dt);
            bottomWoke = bottomWoke || bottom->IsAwake();
        }
        CHECK(bottomWoke);
        CHECK(dropped->Position().y < -50.0f);
    }

    {
        World world(vec2(0.0f, 500.0f));
        Body* ground = world.CreateStaticBox(vec2(0.0f, 0.0f), 200.0f, 10.0f);
        Body* hanging = world.CreateBox(vec2(0.0f, 50.0f), 5.0f, 5.0f, 1.0f);
        world.AddJoint(new RevoluteJoint(ground, hanging, vec2(0.0f, 50.0f)));
        for (int i = 0; i < 1200; ++i)
            world.Step(dt);
        CHECK(!hanging->IsAwake());
    }

    {
        World world(vec2(0.0f, 500.0f));
        Body* anchor = world.CreateBody(BodyType::Static, vec2(0.0f, 0.0f));
        Body* link = world.CreateBox(vec2(0.0f, 40.0f), 4.0f, 4.0f, 1.0f);
        Body* bob = world.CreateBox(vec2(0.0f, 80.0f), 4.0f, 4.0f, 1.0f);
        world.AddJoint(new RevoluteJoint(anchor, link, vec2(0.0f, 0.0f)));
        world.AddJoint(new RevoluteJoint(link, bob, vec2(0.0f, 40.0f)));
        link->SetAngularDamping(5.0f);
        bob->SetAngularDamping(5.0f);
        link->SetLinearDamping(2.0f);
        bob->SetLinearDamping(2.0f);
        for (int i = 0; i < 1800; ++i)
            world.Step(dt);
        CHECK(!link->IsAwake());
        CHECK(!bob->IsAwake());
        CHECK_NEAR(bob->Position().x, 0.0f, 1.0f);
    }

    {
        World world(vec2(0.0f, 500.0f));
        Body* platform = world.CreateKinematicBox(vec2(0.0f, 0.0f), 100.0f, 10.0f);
        Body* box = world.CreateBox(vec2(0.0f, -21.0f), 10.0f, 10.0f, 1.0f);
        box->SetFriction(1.0f);
        platform->SetFriction(1.0f);
        for (int i = 0; i < 300; ++i)
            world.Step(dt);
        CHECK(!box->IsAwake());
        platform->SetVelocity(vec2(0.0f, -20.0f));
        for (int i = 0; i < 120; ++i)
        {
            world.Step(dt);
            CHECK(box->IsAwake());
        }
        CHECK(box->Position().y < -55.0f); // rode up 40 px with the platform
    }

    {
        World world(vec2(0.0f, 500.0f));
        world.CreateStaticBox(vec2(0.0f, 0.0f), 200.0f, 10.0f);
        Body* box = world.CreateBox(vec2(0.0f, -21.0f), 10.0f, 10.0f, 1.0f);
        for (int i = 0; i < 300; ++i)
            world.Step(dt);
        CHECK(!box->IsAwake());

        MouseJoint* mouse = new MouseJoint(box, box->Position(), 1000.0f * box->Mass());
        world.AddJoint(mouse);
        CHECK(box->IsAwake());
        for (int i = 0; i < 300; ++i)
            world.Step(dt);
        CHECK(!box->IsAwake());

        for (int i = 0; i < 120; ++i)
        {
            mouse->SetTarget(vec2(0.0f, -300.0f));
            world.Step(dt);
        }
        CHECK(box->Position().y < -200.0f);
    }

    return TEST_RESULT();
}
