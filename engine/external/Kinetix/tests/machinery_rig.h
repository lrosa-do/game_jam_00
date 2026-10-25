#ifndef KINETIX_TESTS_MACHINERY_RIG_H
#define KINETIX_TESTS_MACHINERY_RIG_H

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

#include <ct/vector.hpp>

#include <math.h>

namespace machinery_test
{

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;
constexpr f32 kG = 9.81f;
// Machine bodies collide with the world but not with each other; ground
// probes skip them.
constexpr u32 kMachineGroup = 1u << 6;
constexpr u32 kGroundGroup = 1u << 0;

inline bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

struct Scene
{
    PhysicsWorld world;
    ct::Vector<RigidBody*> bodies;
    ct::Vector<CollisionShape*> shapes;
    ct::Vector<u32> ids;
    u32 groundId = 0xFFFFFFFFu;
    f32 step = 1.0f / 60.0f;

    explicit Scene(f32 hz = 60.0f, bool floor = true)
    {
        step = 1.0f / hz;
        world.setFixedStep(step);
        world.setGravity(KM::vec3(0.0f, -kG, 0.0f));
        if (floor)
            groundId = addStatic(new BoxShape(KM::vec3(500.0f, 1.0f, 500.0f)), KM::vec3(0.0f, -1.0f, 0.0f), KM::quat());
    }
    ~Scene()
    {
        world.clear();
        for (RigidBody* b : bodies)
            delete b;
        for (CollisionShape* s : shapes)
            delete s;
    }
    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

    u32 addStatic(CollisionShape* shape, const KM::vec3& position, const KM::quat& orientation, f32 friction = 0.8f)
    {
        RigidBody* body = new RigidBody();
        body->setBodyType(BodyType::Static);
        body->setPosition(position);
        body->setOrientation(orientation);
        body->calculateDerivedData();
        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = friction;
        entry.filter.group = kGroundGroup;
        const u32 id = world.addBody(entry);
        world.markStaticBroadphaseDirty();
        bodies.push_back(body);
        shapes.push_back(shape);
        ids.push_back(id);
        return id;
    }

    // A dynamic box; `collide` puts it in the ground's group (it collides
    // with everything), otherwise it collides with nothing.
    RigidBody& box(const KM::vec3& position, const KM::vec3& halfExtents, f32 mass, bool collide = false,
                   u32* idOut = nullptr, f32 friction = 0.6f)
    {
        RigidBody* body = new RigidBody();
        body->setBodyType(BodyType::Dynamic);
        body->setMass(mass);
        body->setInertiaTensor(Inertia::box(mass, halfExtents));
        body->setDamping(1.0f, 1.0f);
        body->setPosition(position);
        body->calculateDerivedData();
        BoxShape* shape = new BoxShape(halfExtents);
        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = friction;
        if (!collide)
        {
            entry.filter.group = 1u << 30;
            entry.filter.mask = 0u;
        }
        const u32 id = world.addBody(entry);
        bodies.push_back(body);
        shapes.push_back(shape);
        ids.push_back(id);
        if (idOut)
            *idOut = id;
        return *body;
    }

    void run(f32 seconds)
    {
        const int steps = static_cast<int>(seconds / step + 0.5f);
        for (int i = 0; i < steps; ++i)
            world.step(step);
    }
};

}

#endif
