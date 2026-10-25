#include "check.h"

#include "../ai/src/AgentAvoidance.h" // internal: applyAgentAvoidance()
#include <Behavior.h>
#include <Entity.h>
#include <FormationBehavior.h>
#include <Group.h>
#include <NavMesh.h>
#include <NavMeshBehavior.h>
#include <PathfindBehavior.h>
#include <PointOfInterest.h>
#include <SquadEntity.h>
#include <World.h>

#include <math.h>

using namespace kinetix::AI;

namespace
{

namespace KM = kinetix::Math;

class Push final : public Behavior
{
public:
    explicit Push(const KM::vec3& push) : mPush(push)
    {
    }
    void iterate(float, Entity& entity) override
    {
        entity.setDesiredMove(entity.desiredMove() + mPush);
    }

private:
    KM::vec3 mPush;
};

bool finite(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

}

int main()
{
    // Avoidance into a neighbour with an even blend (turnRate 0.5): escape and desired direction
    // cancel, and normalize(0) must not give NaN.
    {
        World world;
        Group* group = new Group(world);
        world.add(*group);
        Entity* a = new Entity(world, Entity::Settings());
        Entity* b = new Entity(world, Entity::Settings());
        group->add(*a);
        group->add(*b);
        a->setPosition(KM::vec3(0.0f));
        b->setPosition(KM::vec3(0.5f, 0.0f, 0.0f));
        world.iterate(0.0f);
        a->setDesiredMove(KM::vec3(1.0f, 0.0f, 0.0f));
        detail::applyAgentAvoidance(*a, 1.0f, 0.5f);
        CHECK(finite(a->desiredMove()));
        CHECK_NEAR(KM::length(a->desiredMove()), 1.0f, 1e-4f);
    }

    {
        World world;
        Group* group = new Group(world);
        world.add(*group);
        Entity* entity = new Entity(world, Entity::Settings());
        group->add(*entity);
        const float nan = KM::vec3(0.0f).x / KM::vec3(0.0f).x;
        entity->setVelocity(KM::vec3(1.0f, 0.0f, 0.0f));
        entity->setVelocity(KM::vec3(nan, 0.0f, 0.0f));
        CHECK(finite(entity->velocity()));
        Push poison(KM::vec3(nan, 0.0f, nan));
        entity->addBehavior(poison);
        for (int i = 0; i < 4; ++i)
            world.iterate(1.0f / 60.0f);
        CHECK(finite(entity->velocity()));
        CHECK(finite(entity->position()));
        entity->removeBehavior(poison);
    }

    {
        World world;
        Group* group = new Group(world);
        world.add(*group);
        Entity* gentle = new Entity(world, Entity::Settings());
        Entity::Settings slowSettings;
        slowSettings.maxSpeed = 0.05f;
        Entity* slow = new Entity(world, slowSettings);
        slow->setPosition(KM::vec3(0.0f, 0.0f, 50.0f));
        group->add(*gentle);
        group->add(*slow);
        Push gentlePush(KM::vec3(0.09f, 0.0f, 0.0f));
        Push slowPush(KM::vec3(0.02f, 0.0f, 0.0f));
        gentle->addBehavior(gentlePush);
        slow->addBehavior(slowPush);
        for (int i = 0; i < 60; ++i)
            world.iterate(1.0f / 60.0f);
        CHECK(gentle->position().x > 1.0f);
        CHECK(slow->position().x > 0.02f);
        CHECK(slow->speed() <= 0.05f + 1e-5f);

        gentle->removeBehavior(gentlePush);
        gentle->setVelocity(KM::vec3(0.05f, 0.0f, 0.0f));
        Push brake(KM::vec3(-0.01f, 0.0f, 0.0f));
        gentle->addBehavior(brake);
        world.iterate(1.0f / 60.0f);
        CHECK(gentle->speed() == 0.0f);
        gentle->removeBehavior(brake);
        slow->removeBehavior(slowPush);
    }

    {
        World world;
        Group* group = new Group(world);
        world.add(*group);
        Entity* entity = new Entity(world, Entity::Settings());
        group->add(*entity);
        FormationBehavior formation(1.0f, 1.0f);
        PathfindBehavior pathfind{PathfindBehavior::Settings()};
        entity->addBehavior(formation);
        entity->addBehavior(pathfind);
        world.iterate(1.0f / 60.0f);
        entity->removeBehavior(formation);
        entity->removeBehavior(pathfind);
    }

    {
        const float verts[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f, 0.0f};
        const int indices[] = {0, 1, 2, 0, 2, 3};
        NavMesh mesh;
        CHECK(mesh.build(verts, 4, indices, 2, NavMeshConfig()));
        NavMeshBehavior behavior(mesh, NavMeshBehavior::Settings());

        World world;
        Group* group = new Group(world);
        world.add(*group);
        SquadMemberEntity* first = new SquadMemberEntity(world, Entity::Settings());
        SquadMemberEntity* second = new SquadMemberEntity(world, Entity::Settings());
        first->setPosition(KM::vec3(1.0f, 0.0f, 1.0f));
        second->setPosition(KM::vec3(2.0f, 0.0f, 1.0f));
        first->setGoal(KM::vec3(9.0f, 0.0f, 9.0f));
        second->setGoal(KM::vec3(9.0f, 0.0f, 9.0f));
        group->add(*first);
        group->add(*second);
        first->addBehavior(behavior);
        second->addBehavior(behavior);
        world.iterate(1.0f / 60.0f);
        CHECK(behavior.routeCount() == 2);

        delete first;
        CHECK(behavior.routeCount() == 1);
        CHECK(group->entities().size() == 1);
        world.iterate(1.0f / 60.0f);

        second->removeBehavior(behavior);
        CHECK(behavior.routeCount() == 0);
    }

    {
        World world;
        Group* group = new Group(world);
        world.add(*group);
        Entity* entity = new Entity(world, Entity::Settings());
        group->add(*entity);
        {
            Push shortLived(KM::vec3(0.1f, 0.0f, 0.0f));
            entity->addBehavior(shortLived);
        }
        world.iterate(1.0f / 60.0f);
    }

    {
        World world;
        Group* group = new Group(world);
        world.add(*group);
        SquadLeaderEntity* leader = new SquadLeaderEntity(world, Entity::Settings());
        SquadMemberEntity* a = new SquadMemberEntity(world, Entity::Settings());
        SquadMemberEntity* b = new SquadMemberEntity(world, Entity::Settings());
        group->add(*leader);
        group->add(*a);
        group->add(*b);
        leader->addSquadMember(a);
        leader->addSquadMember(b);
        leader->addSquadMember(b);
        CHECK(leader->squadMembers().size() == 2);

        leader->sendSquadToRandomWaypoint();

        PointsOfInterest pois;
        PointOfInterest* poi = new PointOfInterest(KM::vec3(5.0f, 0.0f, 5.0f), 1.0f);
        pois.add(poi);
        leader->setPointsOfInterest(&pois);
        leader->setSelectedPointOfInterest(poi);
        CHECK(leader->selectedPointOfInterest() == poi);
        pois.remove(poi->id());
        CHECK(leader->selectedPointOfInterest() == nullptr);
        leader->setCommand(SquadCommand::AttackTarget);

        delete a;
        CHECK(leader->squadMembers().size() == 1);

        leader->clearSquadMembers();
        CHECK(b->squadLeader() == nullptr);

        SquadLeaderEntity* other = new SquadLeaderEntity(world, Entity::Settings());
        group->add(*other);
        leader->addSquadMember(b);
        other->addSquadMember(b);
        CHECK(leader->squadMembers().empty());
        CHECK(b->squadLeader() == other);

        FormationBehavior formation(1.0f, 1.0f);
        b->setSquadId(2);
        b->addBehavior(formation);
        delete other;
        CHECK(b->squadLeader() == nullptr);
        world.iterate(1.0f / 60.0f);
        b->removeBehavior(formation);
    }

    return TEST_RESULT();
}
