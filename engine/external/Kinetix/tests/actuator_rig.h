#ifndef KINETIX_TESTS_ACTUATOR_RIG_H
#define KINETIX_TESTS_ACTUATOR_RIG_H

// Every body has a box shape (the world wants one) but sits in a group nothing collides with or queries
// unless a test asks, so only the joints act on it.

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

#include <math.h>

namespace actuator_test
{

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kG = 9.81f;

inline bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

struct Rig
{
    static constexpr u32 MaxBodies = 16;

    PhysicsWorld world;
    RigidBody bodies[MaxBodies];
    BoxShape* shapes[MaxBodies] = {};
    u32 ids[MaxBodies] = {};
    u32 count = 0;
    f32 step = 1.0f / 60.0f;

    explicit Rig(f32 hz = 60.0f, bool gravity = true)
    {
        step = 1.0f / hz;
        world.setFixedStep(step);
        world.setGravity(gravity ? KM::vec3(0.0f, -kG, 0.0f) : KM::vec3(0.0f));
    }

    ~Rig()
    {
        for (u32 i = 0; i < count; ++i)
            delete shapes[i];
    }

    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;

    RigidBody& base(const KM::vec3& position, bool collide = false,
                    const KM::vec3& halfExtents = KM::vec3(0.5f))
    {
        RigidBody& body = bodies[count];
        body.setBodyType(BodyType::Static);
        body.setPosition(position);
        body.calculateDerivedData();
        add(body, halfExtents, collide);
        return body;
    }

    // A dynamic box with the inertia of its own dimensions and no damping.
    RigidBody& box(const KM::vec3& position, const KM::vec3& halfExtents, f32 mass,
                   bool collide = false)
    {
        RigidBody& body = bodies[count];
        body.setBodyType(BodyType::Dynamic);
        body.setMass(mass);
        body.setInertiaTensor(Inertia::box(mass, halfExtents));
        body.setDamping(1.0f, 1.0f);
        body.setPosition(position);
        body.calculateDerivedData();
        add(body, halfExtents, collide);
        return body;
    }

    u32 idOf(const RigidBody& body) const
    {
        for (u32 i = 0; i < count; ++i)
            if (&bodies[i] == &body)
                return ids[i];
        return 0xFFFFFFFFu;
    }

    void run(f32 seconds)
    {
        const int steps = static_cast<int>(seconds / step + 0.5f);
        for (int i = 0; i < steps; ++i)
            world.step(step);
    }

private:
    void add(RigidBody& body, const KM::vec3& halfExtents, bool collide)
    {
        BodyEntry entry;
        entry.body = &body;
        shapes[count] = new BoxShape(halfExtents);
        entry.shape = shapes[count];
        if (!collide)
        {
            entry.filter.group = 1u << 30;
            entry.filter.mask = 0u;
        }
        ids[count] = world.addBody(entry);
        ++count;
    }
};

}

#endif
