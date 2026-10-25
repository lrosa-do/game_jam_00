#include "check.h"

#include "collision/CollisionShape.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/RigidBody.h"
#include "effects/ParticleSystem.h"

#include <kinetix/core/Math.h>

using kinetix::u32;
using kinetix::Physics::ParticleHit;
using kinetix::Physics::ParticleResponse;
using kinetix::Physics::ParticleSpawn;
using kinetix::Physics::ParticleSystem;

namespace
{

struct Spawner
{
    ParticleSystem* system = nullptr;
    u32 hits = 0;
    u32 perHit = 0;
    bool clear = false;
};

// Emits enough from inside the callback to make the pool reallocate while
// step() is still working on the particle that hit.
void onHit(const ParticleHit& hit, void* userData)
{
    Spawner& spawner = *static_cast<Spawner*>(userData);
    ++spawner.hits;
    if (spawner.clear)
        spawner.system->clear();
    for (u32 i = 0; i < spawner.perHit; ++i)
    {
        ParticleSpawn spawn;
        spawn.position = hit.point + kinetix::Math::vec3(0.0f, 1.0f, 0.0f);
        spawn.life = 5.0f;
        spawner.system->emit(spawn);
    }
    spawner.system->explode(hit.point + kinetix::Math::vec3(0.0f, 2.0f, 0.0f), spawner.perHit, 0.1f, 0.2f,
                            0.01f, 0.02f, 5.0f);
}

}

int main()
{
    kinetix::Physics::PhysicsWorld world;
    kinetix::Physics::RigidBody ground;
    ground.setBodyType(kinetix::Physics::BodyType::Static);
    ground.setPosition(kinetix::Math::vec3(0.0f, -0.5f, 0.0f));
    ground.calculateDerivedData();
    kinetix::Physics::BoxShape box(kinetix::Math::vec3(50.0f, 0.5f, 50.0f));
    kinetix::Physics::BodyEntry entry;
    entry.body = &ground;
    entry.shape = &box;
    world.addBody(entry);
    world.step(1.0f / 60.0f);

    {
        ParticleSystem system(world);
        Spawner spawner;
        spawner.system = &system;
        spawner.perHit = 64;
        system.setHitCallback(onHit, &spawner);

        ParticleSpawn spawn;
        spawn.position = kinetix::Math::vec3(0.0f, 0.05f, 0.0f);
        spawn.velocity = kinetix::Math::vec3(0.0f, -5.0f, 0.0f);
        spawn.response = ParticleResponse::Bounce;
        spawn.restitution = 1.0f;
        spawn.drag = 0.0f;
        system.emit(spawn);
        system.step(1.0f / 60.0f);

        CHECK(spawner.hits == 1);
        CHECK(system.count() == 1 + 2 * 64);
        CHECK_NEAR(system.particle(0).position.y, 0.02f, 1.0e-4f);
        CHECK_NEAR(system.particle(1).position.y, 1.0f, 1.0e-4f);
        CHECK_NEAR(system.particle(1 + 64).position.y, 2.0f, 1.0e-4f);
    }

    {
        ParticleSystem system(world);
        Spawner spawner;
        spawner.system = &system;
        spawner.perHit = 4;
        spawner.clear = true;
        system.setHitCallback(onHit, &spawner);
        for (int i = 0; i < 3; ++i)
        {
            ParticleSpawn spawn;
            spawn.position = kinetix::Math::vec3(static_cast<float>(i), 0.05f, 0.0f);
            spawn.velocity = kinetix::Math::vec3(0.0f, -5.0f, 0.0f);
            system.emit(spawn);
        }
        system.step(1.0f / 60.0f);
        CHECK(spawner.hits == 1);
        CHECK(system.count() == 8);
    }

    return TEST_RESULT();
}
