#include "check.h"

#include <kx/joints/gearjoint.h>
#include <kx/joints/mousejoint.h>
#include <kx/joints/revolutejoint.h>
#include <kx/world.h>

using namespace kx;

namespace
{

struct Listener
{
    MouseJoint* mouse;
    int calls;
};

void OnJointDestroyed(Joint* joint, void* context)
{
    Listener* listener = static_cast<Listener*>(context);
    ++listener->calls;
    if (joint == listener->mouse)
        listener->mouse = nullptr;
}

}

int main()
{
    {
        World world(vec2(0.0f, 500.0f));
        Listener listener{nullptr, 0};
        world.SetJointDestroyedCallback(&OnJointDestroyed, &listener);
        Body* box = world.CreateBox(vec2(0.0f, 0.0f), 50.0f, 20.0f, 1.0f);
        listener.mouse = new MouseJoint(box, vec2(0.0f, 0.0f), 1000.0f * box->Mass());
        world.AddJoint(listener.mouse);
        Body* positive = nullptr;
        Body* negative = nullptr;
        CHECK(Slice(world, box, vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), &positive, &negative, 40.0f));
        CHECK(listener.mouse == nullptr);
        CHECK(listener.calls == 1);
        CHECK(world.Joints().size() == 0);
    }

    {
        World world(vec2(0.0f, 0.0f));
        Listener listener{nullptr, 0};
        world.SetJointDestroyedCallback(&OnJointDestroyed, &listener);
        Body* ground = world.CreateBody(BodyType::Static, vec2(0.0f, 0.0f));
        Body* wheelA = world.CreateCircle(vec2(-20.0f, 0.0f), 10.0f, 1.0f);
        Body* wheelB = world.CreateCircle(vec2(20.0f, 0.0f), 10.0f, 1.0f);
        RevoluteJoint* jointA = new RevoluteJoint(ground, wheelA, vec2(-20.0f, 0.0f));
        RevoluteJoint* jointB = new RevoluteJoint(ground, wheelB, vec2(20.0f, 0.0f));
        world.AddJoint(jointA);
        world.AddJoint(jointB);
        world.AddJoint(new GearJoint(jointA, jointB, 1.0f));
        world.Destroy(wheelA);
        CHECK(listener.calls == 2);
        CHECK(world.Joints().size() == 1);
        CHECK(world.Joints()[0] == jointB);

        world.DestroyJoint(jointB);
        CHECK(listener.calls == 2);
        CHECK(world.Joints().size() == 0);
    }

    {
        World world(vec2(0.0f, 0.0f));
        Body* a = world.CreateCircle(vec2(0.0f, 0.0f), 5.0f, 1.0f);
        Body* b = world.CreateCircle(vec2(20.0f, 0.0f), 5.0f, 1.0f);
        RevoluteJoint* joint = new RevoluteJoint(a, b, vec2(10.0f, 0.0f));
        world.AddJoint(joint);
        world.AddJoint(joint);
        CHECK(world.Joints().size() == 1);
    }

    return TEST_RESULT();
}
