#ifndef KINETIX_PHYSICS_EFFECTS_PARTICLESYSTEM_H
#define KINETIX_PHYSICS_EFFECTS_PARTICLESYSTEM_H

// Cheap point/sphere projectiles for VFX (debris, sparks, blood drops), not
// RigidBody: no inertia, joints or sleeping. Integrated in bulk and swept
// against the world with PhysicsWorld::raycast(); a hit callback reports where
// and with what normal/velocity.

#include <kinetix/core/Types.h>
#include "collision/CollisionFilter.h"

#include <kinetix/core/Math.h>
#include <ct/vector.hpp>

namespace kinetix::Physics
{

class PhysicsWorld;
struct WorldRayHit;

enum class ParticleResponse : u8
{
    Kill,   // removed immediately after the callback runs (sparks, blood mist)
    Bounce, // reflects with its own restitution (debris, gravel)
    Stick   // zero velocity, stays at the hit point until life runs out (blood drops, mud)
};

struct ParticleHit
{
    u32 particleId = 0xFFFFFFFFu;
    u32 userTag = 0;
    Math::vec3 point{0.0f};
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    Math::vec3 incomingVelocity{0.0f};
};

struct ParticleSpawn
{
    Math::vec3 position{0.0f};
    Math::vec3 velocity{0.0f};
    f32 radius = 0.02f;
    f32 life = 1.0f;
    f32 restitution = 0.3f;
    f32 drag = 0.1f; // fraction of velocity lost per second, independent of gravity
    u32 userTag = 0;
    ParticleResponse response = ParticleResponse::Kill;
};

// Pooled, index-stable-per-frame (dead particles are swap-removed at the end
// of step(), so an id handed to the hit callback is only valid during that
// same step - store userTag/position out of the callback if you need more).
class ParticleSystem
{
public:
    using HitCallback = void (*)(const ParticleHit& hit, void* userData);

    explicit ParticleSystem(PhysicsWorld& world);

    void setGravity(const Math::vec3& gravity)
    {
        mGravity = gravity;
    }
    const Math::vec3& gravity() const
    {
        return mGravity;
    }

    void setHitCallback(HitCallback callback, void* userData)
    {
        mHitCallback = callback;
        mHitUserData = userData;
    }

    void setQueryFilter(const QueryFilter& filter)
    {
        mFilter = filter;
    }

    u32 emit(const ParticleSpawn& spawn);

    // Burst of `count` particles from `center`, speed and radius uniform in
    // the given ranges, direction uniform over the sphere.
    void explode(const Math::vec3& center, u32 count, f32 speedMin, f32 speedMax, f32 radiusMin,
                f32 radiusMax, f32 life, u32 userTag = 0, ParticleResponse response = ParticleResponse::Bounce);

    // Integrates every live particle by `dt`, sweeps each against the world
    // and fires the hit callback (if set) on impact. Dead particles (life
    // expired or ParticleResponse::Kill) are removed at the end.
    //
    // The callback may emit(), explode() or clear(): particles it emits first
    // move on the next step, and a clear() ends this one.
    void step(f32 dt);

    void clear()
    {
        mParticles.clear();
        ++mClearCount;
    }

    usize count() const
    {
        return mParticles.size();
    }

    // Where the live particles are, for whoever draws them. Valid until the
    // next step(): dead particles are swap-removed, so an index does not name
    // the same particle across steps.
    struct View
    {
        Math::vec3 position;
        f32 radius;
        u32 userTag;
    };
    View particle(usize index) const
    {
        const Particle& p = mParticles[index];
        return View{p.position, p.radius, p.userTag};
    }

private:
    struct Particle
    {
        Math::vec3 position{0.0f};
        Math::vec3 velocity{0.0f};
        f32 radius = 0.02f;
        f32 life = 1.0f;
        f32 restitution = 0.3f;
        f32 drag = 0.1f;
        u32 userTag = 0;
        u32 id = 0;
        ParticleResponse response = ParticleResponse::Kill;
        bool settled = false; // Stick response after it has come to rest: skip integration
    };

    void respond(Particle& particle, const WorldRayHit& hit);

    PhysicsWorld& mWorld;
    ct::Vector<Particle> mParticles;
    Math::vec3 mGravity{0.0f, -9.81f, 0.0f};
    QueryFilter mFilter;
    HitCallback mHitCallback = nullptr;
    void* mHitUserData = nullptr;
    u32 mNextId = 1;
    // Bumped by clear(), so step() can tell its callback emptied the pool.
    u32 mClearCount = 0;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_EFFECTS_PARTICLESYSTEM_H
