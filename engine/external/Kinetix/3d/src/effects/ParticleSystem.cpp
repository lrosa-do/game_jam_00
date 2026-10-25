#include "effects/ParticleSystem.h"

#include "dynamics/PhysicsWorld.h"

#include <math.h>
#include <stdlib.h>

namespace kinetix::Physics
{

namespace
{
f32 randomUnit()
{
    return static_cast<f32>(rand()) / static_cast<f32>(RAND_MAX);
}

f32 randomRange(f32 lo, f32 hi)
{
    return lo + (hi - lo) * randomUnit();
}

Math::vec3 randomDirection()
{
    // Uniform point on the unit sphere (Marsaglia): reject outside the disk,
    // then the standard 2D-to-3D lift - no trig, no clustering at the poles.
    f32 x1, x2, s;
    do
    {
        x1 = randomRange(-1.0f, 1.0f);
        x2 = randomRange(-1.0f, 1.0f);
        s = x1 * x1 + x2 * x2;
    } while (s >= 1.0f);
    const f32 factor = 2.0f * sqrtf(1.0f - s);
    return Math::vec3(x1 * factor, x2 * factor, 1.0f - 2.0f * s);
}
} // namespace

ParticleSystem::ParticleSystem(PhysicsWorld& world) : mWorld(world)
{
}

u32 ParticleSystem::emit(const ParticleSpawn& spawn)
{
    Particle particle;
    particle.position = spawn.position;
    particle.velocity = spawn.velocity;
    particle.radius = spawn.radius;
    particle.life = spawn.life;
    particle.restitution = spawn.restitution;
    particle.drag = spawn.drag;
    particle.userTag = spawn.userTag;
    particle.response = spawn.response;
    particle.id = mNextId++;
    mParticles.push_back(particle);
    return particle.id;
}

void ParticleSystem::explode(const Math::vec3& center, u32 count, f32 speedMin, f32 speedMax,
                             f32 radiusMin, f32 radiusMax, f32 life, u32 userTag,
                             ParticleResponse response)
{
    for (u32 i = 0; i < count; ++i)
    {
        ParticleSpawn spawn;
        spawn.position = center;
        spawn.velocity = randomDirection() * randomRange(speedMin, speedMax);
        spawn.radius = randomRange(radiusMin, radiusMax);
        spawn.life = life;
        spawn.userTag = userTag;
        spawn.response = response;
        emit(spawn);
    }
}

void ParticleSystem::respond(Particle& particle, const WorldRayHit& hit)
{
    switch (particle.response)
    {
    case ParticleResponse::Kill:
        particle.life = 0.0f;
        break;
    case ParticleResponse::Bounce:
        particle.position = hit.point + hit.normal * particle.radius;
        particle.velocity = particle.velocity -
                            (1.0f + particle.restitution) * Math::dot(particle.velocity, hit.normal) *
                                hit.normal;
        break;
    case ParticleResponse::Stick:
        particle.position = hit.point + hit.normal * particle.radius;
        particle.velocity = Math::vec3(0.0f);
        particle.settled = true;
        break;
    }
}

void ParticleSystem::step(f32 dt)
{
    if (dt <= 0.0f)
        return;

    // Only the particles alive when the step began. The hit callback is free
    // to emit() or explode(), which appends to mParticles and may reallocate
    // it: those new particles start moving on the next step, and nothing
    // here may hold a reference into the array across the callback.
    const usize count = mParticles.size();
    const u32 clears = mClearCount;
    for (usize i = 0; i < count; ++i)
    {
        Particle& particle = mParticles[i];
        particle.life -= dt;
        if (particle.settled)
            continue;

        particle.velocity += mGravity * dt;
        particle.velocity *= Math::max(0.0f, 1.0f - particle.drag * dt);

        const Math::vec3 displacement = particle.velocity * dt;
        const f32 travel = Math::length(displacement);
        if (travel <= 1e-6f)
            continue;

        Ray ray;
        ray.origin = particle.position;
        ray.direction = displacement / travel;

        WorldRayHit hit;
        if (mWorld.raycast(ray, travel + particle.radius, mFilter, hit))
        {
            if (mHitCallback)
            {
                ParticleHit info;
                info.particleId = particle.id;
                info.userTag = particle.userTag;
                info.point = hit.point;
                info.normal = hit.normal;
                info.incomingVelocity = particle.velocity;
                mHitCallback(info, mHitUserData);

                // A clear() from inside the callback dropped every particle
                // this loop was walking; whatever is there now is new.
                if (mClearCount != clears)
                    break;
            }

            respond(mParticles[i], hit);
        }
        else
        {
            particle.position += displacement;
        }
    }

    // Swap-remove anything past its lifetime; order does not matter for VFX particles.
    for (usize i = 0; i < mParticles.size();)
    {
        if (mParticles[i].life <= 0.0f)
        {
            mParticles[i] = mParticles[mParticles.size() - 1];
            mParticles.pop_back();
        }
        else
        {
            ++i;
        }
    }
}

} // namespace kinetix::Physics
