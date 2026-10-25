#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/PointJoint.h>
#include <dynamics/RigidBody.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kStep = 1.0f / 120.0f;

RigidBody* newDynamicBox(const KM::vec3& position)
{
    RigidBody* body = new RigidBody();
    body->setBodyType(BodyType::Dynamic);
    body->setMass(1.0f);
    body->setInertiaTensor(Inertia::box(1.0f, KM::vec3(0.5f)));
    body->setPosition(position);
    return body;
}

struct RemoveContext
{
    PhysicsWorld* world = nullptr;
    u32 id = 0;
    RigidBody* body = nullptr;
    Joint* joint = nullptr;
    bool clearAll = false;
    bool done = false;
    u32 removedId = 0xFFFFFFFFu;
    u32 eventsNamingRemoved = 0;
    u32 reusedIdDuringEvents = 0;
};

void removeAndDestroy(f32, void* userData)
{
    RemoveContext& context = *static_cast<RemoveContext*>(userData);
    if (context.done)
        return;
    context.done = true;
    if (context.clearAll)
        context.world->clear();
    else
        context.world->removeBody(context.id);
    CHECK(context.world->body(context.id) == nullptr);
    delete context.joint;
    context.joint = nullptr;
    delete context.body;
    context.body = nullptr;
}

// Events of the step whose callback removed a body still name it. Its id must
// not be handed to a body added while those events are delivered, or the
// rest of the batch would describe the wrong body.
void onEvent(const ContactEventInfo& info, void* userData)
{
    RemoveContext& context = *static_cast<RemoveContext*>(userData);
    if (info.bodyA != context.removedId && info.bodyB != context.removedId)
        return;
    ++context.eventsNamingRemoved;
    static RigidBody late;
    static SphereShape lateShape(0.1f);
    late.setBodyType(BodyType::Static);
    late.setPosition(KM::vec3(50.0f));
    const u32 lateId = context.world->addBody({&late, &lateShape});
    if (lateId == context.removedId)
        ++context.reusedIdDuringEvents;
    context.world->removeBody(lateId);
}

void removeBodyFromStepCallback(bool clearAll)
{
    PhysicsWorld world;
    RigidBody ground;
    BoxShape groundShape(KM::vec3(10.0f, 0.5f, 10.0f));
    ground.setBodyType(BodyType::Static);
    ground.setPosition(KM::vec3(0.0f, -0.5f, 0.0f));
    world.addBody({&ground, &groundShape});

    // A dynamic neighbour in contact and jointed to the doomed body, so the
    // island pass after the callback has every reason to reach it.
    BoxShape shape(KM::vec3(0.5f));
    RigidBody neighbour;
    neighbour.setBodyType(BodyType::Dynamic);
    neighbour.setMass(1.0f);
    neighbour.setInertiaTensor(Inertia::box(1.0f, KM::vec3(0.5f)));
    neighbour.setPosition(KM::vec3(0.0f, 1.5f, 0.0f));

    RemoveContext context;
    context.world = &world;
    context.clearAll = clearAll;
    context.body = newDynamicBox(KM::vec3(0.0f, 0.5f, 0.0f));
    context.id = world.addBody({context.body, &shape});
    world.addBody({&neighbour, &shape});
    context.joint = new PointJoint(*context.body, neighbour, KM::vec3(0.0f, 1.0f, 0.0f));
    world.addJoint(context.joint);
    world.step(kStep);

    context.removedId = context.id;
    world.setStepCallback(removeAndDestroy, &context);
    world.setEventCallback(onEvent, &context);
    world.step(kStep);
    CHECK(context.done);
    CHECK(context.body == nullptr);
    CHECK(world.body(context.id) == nullptr);
    CHECK(world.jointCount() == 0);
    if (!clearAll)
    {
        CHECK(context.eventsNamingRemoved > 0);
        CHECK(context.reusedIdDuringEvents == 0);
        CHECK(world.bodyCount() == 2);
        CHECK(neighbour.awake());
        for (int i = 0; i < 240; ++i)
            world.step(kStep);
        CHECK_NEAR(neighbour.position().y, 0.5, 0.1);
    }
    else
    {
        CHECK(world.bodyCount() == 0);
    }
}
}

int main()
{
    removeBodyFromStepCallback(false);
    removeBodyFromStepCallback(true);
    return TEST_RESULT();
}
