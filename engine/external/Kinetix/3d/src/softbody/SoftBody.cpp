#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "softbody/SoftBody.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "dynamics/PhysicsWorld.h"

#include <ct/sort.hpp>
#include <float.h>
#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kEpsilon = 1.0e-12f;

u64 edgeKey(u32 a, u32 b)
{
    const u64 low = a < b ? a : b;
    const u64 high = a < b ? b : a;
    return (low << 32) | high;
}

// Signed angle between the normals of the two triangles sharing edge x0-x1,
// with x2 and x3 the opposite vertices. The same expression the projection
// uses, so the rest angle measured at build time cancels exactly at rest.
f32 dihedralAngle(const Math::vec3& x0, const Math::vec3& x1, const Math::vec3& x2,
                  const Math::vec3& x3)
{
    const Math::vec3 e = x1 - x0;
    const Math::vec3 n1 = Math::cross(x2 - x0, x2 - x1);
    const Math::vec3 n2 = Math::cross(x3 - x1, x3 - x0);
    const f32 lengthsSquared = Math::dot(n1, n1) * Math::dot(n2, n2);
    if (lengthsSquared < 1.0e-24f)
        return 0.0f;
    const f32 sign = Math::dot(Math::cross(n2, n1), e) < 0.0f ? -1.0f : 1.0f;
    const f32 d = Math::clamp(Math::dot(n1, n2) / sqrtf(lengthsSquared), -1.0f, 1.0f);
    return sign * acosf(d);
}

// How far this particle can move before determineContactPlanes() runs again:
// its own speed over the step plus what predict() adds inside the step. A
// search below this lets a particle cross a collider and pop back out in one
// substep; above it only widens the narrowphase. Not mMaxLinearVelocity,
// which is the 500 m/s safety clamp - metres of reach per frame.
f32 particleTravel(const SoftBody::Particle& particle, const Math::vec3& gravity,
                   const Math::vec3& windPerParticle, f32 dt)
{
    const Math::vec3 acceleration = gravity + windPerParticle * particle.invMass;
    return (Math::length(particle.velocity) + Math::length(acceleration) * dt) * dt;
}
} // namespace

void SoftBody::clear()
{
    mFaces.clear();
    mParticles.clear();
    mConstraints.clear();
    mBendConstraints.clear();
    mAttachments.clear();
}

void SoftBody::setParticles(const Math::vec3* positions, u32 count, f32 totalMass)
{
    mParticles.clear();
    mFaces.clear();
    mConstraints.clear();
    mBendConstraints.clear();
    mAttachments.clear();
    if (!positions || count == 0)
        return;

    const f32 perParticle = totalMass > 0.0f ? totalMass / static_cast<f32>(count) : 0.0f;
    const f32 invMass = perParticle > 0.0f ? 1.0f / perParticle : 0.0f;
    mParticles.resize(count);
    for (u32 i = 0; i < count; ++i)
    {
        Particle& particle = mParticles[i];
        particle.position = positions[i];
        particle.previousPosition = positions[i];
        particle.velocity = Math::vec3(0.0f);
        particle.invMass = invMass;
    }
}

void SoftBody::setTotalMass(f32 totalMass)
{
    if (mParticles.empty())
        return;
    const f32 perParticle = totalMass > 0.0f
                                ? totalMass / static_cast<f32>(mParticles.size())
                                : 0.0f;
    const f32 invMass = perParticle > 0.0f ? 1.0f / perParticle : 0.0f;
    for (Particle& particle : mParticles)
        if (particle.invMass != 0.0f)
            particle.invMass = invMass;
}

void SoftBody::addDistanceConstraint(u32 a, u32 b, f32 compliance)
{
    if (a >= mParticles.size() || b >= mParticles.size() || a == b)
        return;
    DistanceConstraint constraint;
    constraint.a = a;
    constraint.b = b;
    constraint.restLength = Math::length(mParticles[b].position - mParticles[a].position);
    constraint.compliance = Math::max(compliance, 0.0f);
    mConstraints.push_back(constraint);
}

void SoftBody::buildFromMesh(const u32* indices, u32 indexCount, f32 structuralCompliance,
                             f32 bendCompliance, BendType bendType)
{
    mConstraints.clear();
    mBendConstraints.clear();
    mFaces.clear();
    if (!indices || indexCount < 3 || mParticles.empty())
        return;

    // Only triangles naming three distinct, existing particles: pressure and
    // the edge and bend constraints index mParticles straight from these.
    const u32 particleCount = static_cast<u32>(mParticles.size());
    const u32 triangleCount = indexCount / 3;
    u32 rejected = 0;
    mFaces.reserve(static_cast<usize>(triangleCount) * 3);
    for (u32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        const u32 a = indices[triangle * 3];
        const u32 b = indices[triangle * 3 + 1];
        const u32 c = indices[triangle * 3 + 2];
        if (a >= particleCount || b >= particleCount || c >= particleCount || a == b || b == c ||
            a == c)
        {
            ++rejected;
            continue;
        }
        mFaces.push_back(a);
        mFaces.push_back(b);
        mFaces.push_back(c);
    }
    if (rejected > 0)
        Log::warning("SoftBody: buildFromMesh skipped %u of %u triangles with an index out of "
                     "range (%u particles) or repeated",
                     rejected, triangleCount, particleCount);

    struct EdgeRecord
    {
        u64 key;
        u32 triangle;
        u32 opposite;
    };
    ct::Vector<EdgeRecord> records;
    records.reserve(mFaces.size());

    for (u32 triangle = 0; triangle < mFaces.size() / 3; ++triangle)
    {
        const u32 v[3] = {mFaces[triangle * 3], mFaces[triangle * 3 + 1],
                          mFaces[triangle * 3 + 2]};
        for (u32 e = 0; e < 3; ++e)
            records.push_back({edgeKey(v[e], v[(e + 1) % 3]), triangle, v[(e + 2) % 3]});
    }

    ct::sort(records.begin(), records.end(),
              [](const EdgeRecord& a, const EdgeRecord& b) { return a.key < b.key; });

    for (usize i = 0; i < records.size();)
    {
        usize end = i + 1;
        while (end < records.size() && records[end].key == records[i].key)
            ++end;

        const u32 a = static_cast<u32>(records[i].key >> 32);
        const u32 b = static_cast<u32>(records[i].key & 0xFFFFFFFFull);
        addDistanceConstraint(a, b, structuralCompliance);

        if (end - i == 2 && bendCompliance >= 0.0f && bendType != BendType::None)
        {
            if (bendType == BendType::Distance)
            {
                addDistanceConstraint(records[i].opposite, records[i + 1].opposite,
                                      bendCompliance);
            }
            else
            {
                DihedralBendConstraint bend;
                bend.a = a;
                bend.b = b;
                bend.c = records[i].opposite;
                bend.d = records[i + 1].opposite;
                bend.compliance = bendCompliance;
                bend.initialAngle =
                    dihedralAngle(mParticles[bend.a].position, mParticles[bend.b].position,
                                  mParticles[bend.c].position, mParticles[bend.d].position);
                mBendConstraints.push_back(bend);
            }
        }

        i = end;
    }
}

void SoftBody::setPinned(u32 index, bool pinnedState)
{
    if (index >= mParticles.size())
        return;
    if (pinnedState)
    {
        mParticles[index].invMass = 0.0f;
        return;
    }
    if (mParticles[index].invMass == 0.0f)
        mParticles[index].invMass = 1.0f;
}

bool SoftBody::pinned(u32 index) const
{
    return index < mParticles.size() && mParticles[index].invMass == 0.0f;
}

void SoftBody::buildAttachments(f32 maxDistanceMultiplier)
{
    mAttachments.clear();

    ct::Vector<u32> anchors;
    for (u32 i = 0; i < mParticles.size(); ++i)
        if (mParticles[i].invMass == 0.0f)
            anchors.push_back(i);
    if (anchors.empty())
        return;

    const f32 multiplier = Math::max(maxDistanceMultiplier, 1.0f);
    for (u32 i = 0; i < mParticles.size(); ++i)
    {
        if (mParticles[i].invMass == 0.0f)
            continue;

        u32 nearest = anchors[0];
        f32 nearestSquared = FLT_MAX;
        for (u32 anchor : anchors)
        {
            const Math::vec3 delta = mParticles[i].position - mParticles[anchor].position;
            const f32 squared = Math::dot(delta, delta);
            if (squared < nearestSquared)
            {
                nearestSquared = squared;
                nearest = anchor;
            }
        }

        LongRangeAttachment attachment;
        attachment.anchor = nearest;
        attachment.vertex = i;
        attachment.maxDistance = sqrtf(nearestSquared) * multiplier;
        mAttachments.push_back(attachment);
    }
}

f32 SoftBody::volumeTimesSix() const
{
    f32 sixVolume = 0.0f;
    for (usize i = 0; i + 2 < mFaces.size(); i += 3)
    {
        const Math::vec3& x1 = mParticles[mFaces[i]].position;
        const Math::vec3& x2 = mParticles[mFaces[i + 1]].position;
        const Math::vec3& x3 = mParticles[mFaces[i + 2]].position;
        sixVolume += Math::dot(Math::cross(x1, x2), x3);
    }
    return sixVolume;
}

// p = F / A = n R T / V, with the coefficient standing for n R T: the impulse
// on a triangle over one substep is coefficient / V * A * dt along its normal,
// a third of it to each corner.
void SoftBody::applyPressure(f32 dt)
{
    if (!(mPressure > 0.0f) || mFaces.size() < 3)
        return;
    const f32 sixVolume = volumeTimesSix();
    if (!(sixVolume > 0.0f))
        return;

    const f32 coefficient = mPressure * dt / sixVolume;
    for (usize i = 0; i + 2 < mFaces.size(); i += 3)
    {
        const u32 indices[3] = {mFaces[i], mFaces[i + 1], mFaces[i + 2]};
        const Math::vec3& x1 = mParticles[indices[0]].position;
        const Math::vec3& x2 = mParticles[indices[1]].position;
        const Math::vec3& x3 = mParticles[indices[2]].position;
        const Math::vec3 impulse = coefficient * Math::cross(x2 - x1, x3 - x1);
        for (u32 index : indices)
        {
            Particle& particle = mParticles[index];
            particle.velocity += impulse * particle.invMass;
        }
    }
}

void SoftBody::predict(f32 dt)
{
    const f32 damping = mDamping < 1.0f ? powf(mDamping, dt) : 1.0f;
    // The body force is distributed over all vertices, else refining a cloth
    // mesh multiplies its wind acceleration by its vertex count.
    const Math::vec3 windPerParticle =
        mParticles.empty() ? Math::vec3(0.0f)
                           : mWind / static_cast<f32>(mParticles.size());
    for (Particle& particle : mParticles)
    {
        particle.previousPosition = particle.position;
        if (particle.invMass == 0.0f)
            continue;
        particle.velocity += (mGravity + windPerParticle * particle.invMass) * dt;
        particle.velocity *= damping;
        particle.position += particle.velocity * dt;
    }
}

void SoftBody::projectDistanceConstraints(f32 dt)
{
    const f32 inverseStepSquared = dt > 0.0f ? 1.0f / (dt * dt) : 0.0f;

    for (const DistanceConstraint& constraint : mConstraints)
    {
        Particle& a = mParticles[constraint.a];
        Particle& b = mParticles[constraint.b];

        const Math::vec3 delta = b.position - a.position;
        const f32 length = Math::length(delta);
        if (length < kEpsilon)
            continue;

        const f32 denominator =
            length * (a.invMass + b.invMass + constraint.compliance * inverseStepSquared);
        if (denominator < kEpsilon)
            continue;

        const Math::vec3 correction = delta * ((length - constraint.restLength) / denominator);
        a.position += correction * a.invMass;
        b.position -= correction * b.invMass;
    }
}

void SoftBody::projectDihedralBendConstraints(f32 dt)
{
    const f32 inverseStepSquared = dt > 0.0f ? 1.0f / (dt * dt) : 0.0f;

    for (const DihedralBendConstraint& bend : mBendConstraints)
    {
        Particle& p0 = mParticles[bend.a];
        Particle& p1 = mParticles[bend.b];
        Particle& p2 = mParticles[bend.c];
        Particle& p3 = mParticles[bend.d];

        const Math::vec3 x0 = p0.position;
        const Math::vec3 x1 = p1.position;
        const Math::vec3 x2 = p2.position;
        const Math::vec3 x3 = p3.position;

        const Math::vec3 e = x1 - x0;
        const f32 edgeLength = Math::length(e);
        if (edgeLength < 1.0e-6f)
            continue;

        const Math::vec3 x1x2 = x2 - x1;
        const Math::vec3 x1x3 = x3 - x1;
        Math::vec3 n1 = Math::cross(x2 - x0, x1x2);
        Math::vec3 n2 = Math::cross(x1x3, x3 - x0);
        const f32 n1LengthSquared = Math::dot(n1, n1);
        const f32 n2LengthSquared = Math::dot(n2, n2);
        const f32 lengthsSquared = n1LengthSquared * n2LengthSquared;
        if (lengthsSquared < 1.0e-24f)
            continue;

        const f32 sign = Math::dot(Math::cross(n2, n1), e) < 0.0f ? -1.0f : 1.0f;
        const f32 d = Math::clamp(Math::dot(n1, n2) / sqrtf(lengthsSquared), -1.0f, 1.0f);
        f32 c = sign * acosf(d) - bend.initialAngle;
        if (c > Math::pi<f32>())
            c -= Math::two_pi<f32>();
        else if (c < -Math::pi<f32>())
            c += Math::two_pi<f32>();

        n1 /= n1LengthSquared;
        n2 /= n2LengthSquared;
        const Math::vec3 d0c = (Math::dot(x1x2, e) * n1 + Math::dot(x1x3, e) * n2) / edgeLength;
        const Math::vec3 d2c = edgeLength * n1;
        const Math::vec3 d3c = edgeLength * n2;
        const Math::vec3 d1c = -d0c - d2c - d3c;

        const f32 denominator = p0.invMass * Math::dot(d0c, d0c) +
                                p1.invMass * Math::dot(d1c, d1c) +
                                p2.invMass * Math::dot(d2c, d2c) +
                                p3.invMass * Math::dot(d3c, d3c) +
                                bend.compliance * inverseStepSquared;
        if (denominator < 1.0e-12f)
            continue;
        const f32 minusLambda = c / denominator;

        p0.position = x0 - minusLambda * p0.invMass * d0c;
        p1.position = x1 - minusLambda * p1.invMass * d1c;
        p2.position = x2 - minusLambda * p2.invMass * d2c;
        p3.position = x3 - minusLambda * p3.invMass * d3c;
    }
}

void SoftBody::projectAttachments()
{
    for (const LongRangeAttachment& attachment : mAttachments)
    {
        const Math::vec3 anchor = mParticles[attachment.anchor].position;
        Particle& particle = mParticles[attachment.vertex];
        const Math::vec3 delta = particle.position - anchor;
        const f32 squared = Math::dot(delta, delta);
        if (squared > attachment.maxDistance * attachment.maxDistance && squared > kEpsilon)
            particle.position = anchor + delta * (attachment.maxDistance / sqrtf(squared));
    }
}

// Once per step, not per substep: each particle gets the plane of the
// closest collider surface it could reach, and every substep projects against
// that same plane. A fresh narrowphase normal per substep turns a resting
// pile into a feedback loop between the constraints and the push-out.
void SoftBody::determineContactPlanes(f32 dt)
{
    mContactPlanes.assign(mParticles.size(), ContactPlane{});
    if (!mCollisionWorld || dt <= 0.0f || mParticles.empty())
        return;

    const Math::vec3 windPerParticle = mWind / static_cast<f32>(mParticles.size());
    AABB bounds;
    bounds.min = Math::vec3(FLT_MAX);
    bounds.max = Math::vec3(-FLT_MAX);
    for (const Particle& particle : mParticles)
    {
        const f32 reach =
            mCollisionMargin + particleTravel(particle, mGravity, windPerParticle, dt);
        bounds.min = Math::min(bounds.min, particle.position - Math::vec3(reach));
        bounds.max = Math::max(bounds.max, particle.position + Math::vec3(reach));
    }
    mCollisionWorld->queryAABB(bounds, mCollisionQuery, mCollisionCandidates);
    if (mCollisionCandidates.empty())
        return;

    const SphereShape particleShape(mCollisionMargin);
    ct::Vector<ContactManifold>& manifolds = mCollisionManifolds;
    if (manifolds.capacity() < 4)
        manifolds.reserve(4);

    for (u32 index = 0; index < mParticles.size(); ++index)
    {
        Particle& particle = mParticles[index];
        if (particle.invMass == 0.0f)
            continue;

        const f32 search = particleTravel(particle, mGravity, windPerParticle, dt);
        Math::mat4 particleTransform(1.0f);
        particleTransform[3] = Math::vec4(particle.position, 1.0f);

        f32 deepest = -FLT_MAX;
        for (u32 id : mCollisionCandidates)
        {
            const BodyEntry* entry = mCollisionWorld->body(id);
            if (!entry || !entry->body || !entry->shape || !entry->enabled)
                continue;

            manifolds.clear();
            if (entry->shape->type() == ShapeType::Trimesh)
            {
                Narrowphase::convexTrimesh(
                    particleShape, particleTransform,
                    static_cast<const TrimeshShape&>(*entry->shape), entry->body->transform(),
                    manifolds, search);
            }
            else if (entry->shape->type() == ShapeType::Triangle)
            {
                ContactManifold manifold;
                if (Narrowphase::sphereTriangle(
                        particleShape, particleTransform,
                        static_cast<const TriangleShape&>(*entry->shape), entry->body->transform(),
                        manifold, search))
                    manifolds.push_back(manifold);
            }
            else
            {
                ContactManifold manifold;
                if (Narrowphase::collide(particleShape, particleTransform, *entry->shape,
                                         entry->body->transform(), manifold, search))
                    manifolds.push_back(manifold);
            }

            for (const ContactManifold& manifold : manifolds)
            {
                for (u32 i = 0; i < manifold.count; ++i)
                {
                    if (manifold.points[i].penetration <= deepest)
                        continue;
                    deepest = manifold.points[i].penetration;

                    ContactPlane& plane = mContactPlanes[index];
                    // Narrowphase normal runs from its first shape (the
                    // particle) towards the collider; the plane wants the
                    // opposite direction, out of the collider.
                    plane.normal = -manifold.normal;
                    // The particle centre sits margin - penetration from the
                    // surface along the normal; anchor the plane there so a
                    // later position measures its true clearance against it.
                    plane.offset = Math::dot(plane.normal, particle.position) -
                                   (mCollisionMargin - deepest);
                    plane.bodyId = id;
                    plane.friction = entry->friction;
                    plane.restitution = entry->restitution;
                    plane.active = true;
                }
            }
        }
    }
}

void SoftBody::applyContactPlanes(f32 dt)
{
    if (!mCollisionWorld || dt <= 0.0f || mContactPlanes.size() != mParticles.size())
        return;

    const f32 restitutionThreshold = -2.0f * Math::length(mGravity) * dt;
    for (u32 index = 0; index < mParticles.size(); ++index)
    {
        const ContactPlane& plane = mContactPlanes[index];
        Particle& particle = mParticles[index];
        if (!plane.active || particle.invMass == 0.0f)
            continue;

        const f32 projected = plane.offset - Math::dot(plane.normal, particle.position) +
                              mCollisionMargin;
        if (projected <= 0.0f)
            continue;

        // Push-out after updateVelocities() has run, so it changes pose
        // without injecting velocity.
        particle.position += plane.normal * projected;

        // Contact velocity is read from the collider so a kinematic body drags
        // and carries the cloth.
        Math::vec3 colliderVelocity(0.0f);
        if (const BodyEntry* entry = mCollisionWorld->body(plane.bodyId))
            if (entry->body)
                colliderVelocity = entry->body->velocityAtPoint(particle.position);

        const Math::vec3 relativeVelocity = particle.velocity - colliderVelocity;
        const f32 normalVelocity = Math::dot(relativeVelocity, plane.normal);
        const Math::vec3 tangent = relativeVelocity - plane.normal * normalVelocity;
        const f32 tangentSpeed = Math::length(tangent);
        if (tangentSpeed > kEpsilon && plane.friction > 0.0f)
            particle.velocity -=
                tangent * Math::min(plane.friction * projected / (tangentSpeed * dt), 1.0f);

        // The whole normal component goes, separating or not: a resting
        // pile's constraint corrections keep generating small outward
        // velocities, which make a sheet jitter and glide. Bounce comes back
        // only through restitution, on the velocity before this substep's
        // update.
        particle.velocity -= plane.normal * normalVelocity;
        const f32 previousNormalVelocity =
            Math::dot(particle.previousVelocity - colliderVelocity, plane.normal);
        if (previousNormalVelocity < restitutionThreshold)
            particle.velocity -= plane.restitution * previousNormalVelocity * plane.normal;
    }
}

void SoftBody::updateVelocities(f32 dt)
{
    if (dt <= 0.0f)
        return;
    const f32 inverseStep = 1.0f / dt;
    for (Particle& particle : mParticles)
    {
        particle.previousVelocity = particle.velocity;
        if (particle.invMass == 0.0f)
        {
            particle.velocity = Math::vec3(0.0f);
            continue;
        }
        particle.velocity = (particle.position - particle.previousPosition) * inverseStep;
    }
}

void SoftBody::step(f32 dt, u32 substeps)
{
    if (mParticles.empty() || !(dt > 0.0f))
        return;

    const u32 count = Math::max(substeps, 1u);
    const f32 h = dt / static_cast<f32>(count);
    determineContactPlanes(dt);
    for (u32 i = 0; i < count; ++i)
    {
        applyPressure(h);
        predict(h);
        projectDihedralBendConstraints(h);
        projectDistanceConstraints(h);
        projectAttachments();
        updateVelocities(h);
        applyContactPlanes(h);
    }

    // Clamps vertex velocity: a dense constraint graph can turn one large
    // positional correction into an extreme velocity.
    const f32 maximumSquared = mMaxLinearVelocity * mMaxLinearVelocity;
    for (Particle& particle : mParticles)
    {
        const f32 speedSquared = Math::dot(particle.velocity, particle.velocity);
        if (speedSquared > maximumSquared && speedSquared > kEpsilon)
            particle.velocity *= mMaxLinearVelocity / sqrtf(speedSquared);
    }
}

f32 SoftBody::worstStretch() const
{
    f32 worst = 1.0f;
    for (const DistanceConstraint& constraint : mConstraints)
    {
        if (constraint.restLength < kEpsilon)
            continue;
        const f32 length = Math::length(mParticles[constraint.b].position -
                                       mParticles[constraint.a].position);
        worst = Math::max(worst, length / constraint.restLength);
    }
    return worst;
}

SoftBody::Contact SoftBody::contact(u32 index) const
{
    Contact result;
    if (index >= mContactPlanes.size())
        return result;
    const ContactPlane& plane = mContactPlanes[index];
    result.active = plane.active;
    result.normal = plane.normal;
    result.bodyId = plane.bodyId;
    return result;
}

} // namespace kinetix::Physics
