#include "check.h"

#include <kx/joints/revolutejoint.h>
#include <kx/world.h>

using namespace kx;

namespace
{

struct SelfDestruct
{
    World* world;
    int beginCalls;
    int callsAfterDestroy;
    Body* destroyed;
};

void DestroySelfOnBegin(const ContactEvent& event, void* context)
{
    SelfDestruct* state = static_cast<SelfDestruct*>(context);
    if (state->destroyed == event.self)
        ++state->callsAfterDestroy;
    if (event.phase != ContactPhase::Begin)
        return;
    ++state->beginCalls;
    state->destroyed = event.self;
    state->world->Destroy(event.self);
}

bool ContactsReference(const World& world, const Body* body)
{
    for (size_t i = 0; i < world.Contacts().size(); ++i)
        if (world.Contacts()[i].a == body || world.Contacts()[i].b == body)
            return true;
    return false;
}

bool HasBody(const World& world, const Body* body)
{
    for (size_t i = 0; i < world.Bodies().size(); ++i)
        if (world.Bodies()[i] == body)
            return true;
    return false;
}

struct Counter
{
    int begin;
    int end;
};

void CountEvents(const ContactEvent& event, void* context)
{
    Counter* counter = static_cast<Counter*>(context);
    if (event.phase == ContactPhase::Begin)
        ++counter->begin;
    else if (event.phase == ContactPhase::End)
        ++counter->end;
}

struct SliceOnBegin
{
    World* world;
    bool done;
    bool sliced;
    Body* positive;
    Body* negative;
};

void SliceSelf(const ContactEvent& event, void* context)
{
    SliceOnBegin* state = static_cast<SliceOnBegin*>(context);
    if (event.phase != ContactPhase::Begin || state->done)
        return;
    state->done = true;
    state->sliced = Slice(*state->world, event.self, event.self->WorldCenter(), vec2(1.0f, 0.0f), &state->positive,
                          &state->negative);
    CHECK(event.self->IsDestroyPending());
    CHECK(!Slice(*state->world, event.self, event.self->WorldCenter(), vec2(0.0f, 1.0f), nullptr, nullptr));
    state->world->Destroy(event.self);
}

struct DestroyJointOnBegin
{
    World* world;
    Joint* joint;
};

void DestroyJointCallback(const ContactEvent& event, void* context)
{
    DestroyJointOnBegin* state = static_cast<DestroyJointOnBegin*>(context);
    if (event.phase != ContactPhase::Begin || !state->joint)
        return;
    state->world->DestroyJoint(state->joint);
    CHECK(state->world->Joints().size() == 1);
    state->joint = nullptr;
}

struct DestroyOnEnd
{
    World* world;
    int ends;
};

void DestroySelfOnEnd(const ContactEvent& event, void* context)
{
    DestroyOnEnd* state = static_cast<DestroyOnEnd*>(context);
    if (event.phase != ContactPhase::End)
        return;
    ++state->ends;
    state->world->Destroy(event.self);
    state->world->Step(1.0f / 60.0f);
}

}

int main()
{
    {
        World world(vec2(0.0f, 0.0f));
        world.CreateStaticBox(vec2(-10.0f, 0.0f), 10.0f, 10.0f);
        world.CreateStaticBox(vec2(10.0f, 0.0f), 10.0f, 10.0f);
        Body* projectile = world.CreateCircle(vec2(0.0f, 12.0f), 5.0f, 1.0f);
        SelfDestruct state{&world, 0, 0, nullptr};
        projectile->SetContactCallback(&DestroySelfOnBegin, &state);

        world.Step(1.0f / 60.0f);
        CHECK(state.beginCalls == 1);
        CHECK(state.callsAfterDestroy == 0);
        CHECK(world.BodyCount() == 2);
        CHECK(!HasBody(world, projectile));
        CHECK(!ContactsReference(world, projectile));
        world.Step(1.0f / 60.0f);
        CHECK(state.beginCalls == 1);
    }

    {
        World world(vec2(0.0f, 0.0f));
        Body* ground = world.CreateStaticBox(vec2(0.0f, 0.0f), 50.0f, 10.0f);
        Counter groundEvents{0, 0};
        ground->SetContactCallback(&CountEvents, &groundEvents);
        Body* projectile = world.CreateCircle(vec2(0.0f, 14.0f), 5.0f, 1.0f);
        projectile->SetVelocity(vec2(0.0f, -200.0f));
        SelfDestruct state{&world, 0, 0, nullptr};
        projectile->SetContactCallback(&DestroySelfOnBegin, &state);

        world.Step(1.0f / 60.0f);
        CHECK(state.beginCalls == 1);
        CHECK(groundEvents.begin == 1);
        CHECK(groundEvents.end == 1);
        CHECK(world.BodyCount() == 1);
    }

    {
        World world(vec2(0.0f, 500.0f));
        world.CreateStaticBox(vec2(0.0f, 0.0f), 100.0f, 10.0f);
        Body* box = world.CreateBox(vec2(0.0f, -20.0f), 10.0f, 10.0f, 1.0f);
        world.Step(1.0f / 60.0f);
        CHECK(ContactsReference(world, box));
        world.Destroy(box);
        CHECK(!ContactsReference(world, box));
        world.Destroy(box);
        CHECK(world.BodyCount() == 1);
    }

    {
        World world(vec2(0.0f, 0.0f));
        Body* a = world.CreateBox(vec2(0.0f, 0.0f), 10.0f, 10.0f, 1.0f);
        Body* b = world.CreateBox(vec2(0.0f, 19.0f), 10.0f, 10.0f, 1.0f);
        Body* c = world.CreateBox(vec2(0.0f, -19.0f), 10.0f, 10.0f, 1.0f);
        world.Step(1.0f / 60.0f);
        DestroyOnEnd stateB{&world, 0};
        DestroyOnEnd stateC{&world, 0};
        b->SetContactCallback(&DestroySelfOnEnd, &stateB);
        c->SetContactCallback(&DestroySelfOnEnd, &stateC);
        world.Destroy(a);
        CHECK(stateB.ends == 1);
        CHECK(stateC.ends == 1);
        CHECK(world.BodyCount() == 0);
        CHECK(world.ContactCount() == 0);
    }

    {
        World world(vec2(0.0f, 0.0f));
        world.CreateStaticBox(vec2(0.0f, 0.0f), 100.0f, 10.0f);
        Body* box = world.CreateBox(vec2(0.0f, -20.0f), 10.0f, 10.0f, 1.0f);
        SliceOnBegin state{&world, false, false, nullptr, nullptr};
        box->SetContactCallback(&SliceSelf, &state);
        world.Step(1.0f / 60.0f);
        CHECK(state.sliced);
        CHECK(!HasBody(world, box));
        CHECK(HasBody(world, state.positive));
        CHECK(HasBody(world, state.negative));
        CHECK(world.BodyCount() == 3);
    }

    {
        World world(vec2(0.0f, 0.0f));
        world.CreateStaticBox(vec2(0.0f, 0.0f), 100.0f, 10.0f);
        Body* box = world.CreateBox(vec2(0.0f, -20.0f), 10.0f, 10.0f, 1.0f);
        Body* anchor = world.CreateBody(BodyType::Static, vec2(0.0f, -100.0f));
        RevoluteJoint* joint = new RevoluteJoint(anchor, box, vec2(0.0f, -20.0f));
        world.AddJoint(joint);
        DestroyJointOnBegin state{&world, joint};
        box->SetContactCallback(&DestroyJointCallback, &state);
        world.Step(1.0f / 60.0f);
        CHECK(state.joint == nullptr);
        CHECK(world.Joints().size() == 0);
    }

    return TEST_RESULT();
}
