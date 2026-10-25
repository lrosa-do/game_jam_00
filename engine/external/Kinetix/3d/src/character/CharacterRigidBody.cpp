#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "character/CharacterRigidBody.h"

#include "collision/Narrowphase.h"
#include "dynamics/PhysicsWorld.h"

#include <float.h>
#include <math.h>

namespace kinetix::Physics
{

CharacterRigidBody::CharacterRigidBody()
{
}

CharacterRigidBody::~CharacterRigidBody()
{
    removeFromWorld();
}

void CharacterRigidBody::setShape(f32 radius, f32 height)
{
    mRadius = Math::max(radius, 0.01f);
    mHeight = Math::max(height, 0.0f);
    mShape = CapsuleShape(mRadius, mHeight * 0.5f);
}

void CharacterRigidBody::setMass(f32 mass)
{
    mMass = Math::max(mass, 0.001f);
    if (mBodyId != kInvalidBodyId)
        body().setMass(mMass);
}

void CharacterRigidBody::setFriction(f32 friction)
{
    mFriction = Math::max(friction, 0.0f);
    if (mWorld && mBodyId != kInvalidBodyId)
        if (BodyEntry* entry = mWorld->body(mBodyId))
            entry->friction = mFriction;
}

void CharacterRigidBody::setMaxSlopeAngle(f32 degrees)
{
    mMaxSlopeAngleDegrees = Math::clamp(degrees, 0.0f, 89.0f);
    mMaxSlopeAngleCosine = cosf(Math::radians(mMaxSlopeAngleDegrees));
}

void CharacterRigidBody::setLayer(u32 layer)
{
    mLayer = layer;
    if (mWorld && mBodyId != kInvalidBodyId)
        if (BodyEntry* entry = mWorld->body(mBodyId))
            entry->filter.group = mLayer;
}

void CharacterRigidBody::setMask(u32 mask)
{
    mMask = mask;
    if (mWorld && mBodyId != kInvalidBodyId)
        if (BodyEntry* entry = mWorld->body(mBodyId))
            entry->filter.mask = mMask;
}

void CharacterRigidBody::addToWorld(PhysicsWorld& world, const Math::vec3& position)
{
    // Leaves a previous addToWorld() or bindToBody() before registering mBody
    // again, so no entry is left untracked.
    removeFromWorld();

    mBody.setBodyType(BodyType::Dynamic);
    mBody.setMass(mMass);
    mBody.setInverseInertiaTensor(Math::mat3(0.0f));
    mBody.setPosition(position);

    BodyEntry entry;
    entry.body = &mBody;
    entry.shape = &mShape;
    entry.filter.group = mLayer;
    entry.filter.mask = mMask;
    entry.friction = mFriction;
    entry.restitution = 0.0f;

    mWorld = &world;
    mBodyId = world.addBody(entry);
}

bool CharacterRigidBody::bindToBody(PhysicsWorld& world, u32 bodyId)
{
    BodyEntry* entry = world.body(bodyId);
    if (!entry || !entry->body || !entry->shape)
        return false;

    removeFromWorld();
    mWorld = &world;
    mBodyId = bodyId;
    mBodyRef = entry->body;
    mShapeRef = entry->shape;
    mBound = true;
    mLayer = entry->filter.group;
    mMask = entry->filter.mask;
    return true;
}

void CharacterRigidBody::removeFromWorld()
{
    if (mWorld && mBodyId != kInvalidBodyId && !mBound)
        mWorld->removeBody(mBodyId);
    mWorld = nullptr;
    mBodyId = kInvalidBodyId;
    mBodyRef = &mBody;
    mShapeRef = &mShape;
    mBound = false;
}

void CharacterRigidBody::setLinearVelocity(const Math::vec3& velocity)
{
    body().setVelocity(velocity);
    if (Math::dot(velocity, velocity) > 1.0e-12f && !body().awake())
        body().setAwake(true);
}

const Math::vec3& CharacterRigidBody::linearVelocity() const
{
    return body().velocity();
}

void CharacterRigidBody::addLinearVelocity(const Math::vec3& velocity)
{
    setLinearVelocity(body().velocity() + velocity);
}

void CharacterRigidBody::addImpulse(const Math::vec3& impulse)
{
    body().applyLinearImpulse(impulse);
}

const Math::vec3& CharacterRigidBody::position() const
{
    return body().position();
}

Math::mat4 CharacterRigidBody::transform() const
{
    return body().transform();
}

void CharacterRigidBody::postSimulation(f32 maxSeparationDistance)
{
    if (!mWorld || mBodyId == kInvalidBodyId)
        return;

    const Math::vec3 characterPosition = body().position();
    const Math::mat4 characterTransform = body().transform();

    u32 groundBodyId = kInvalidBodyId;
    Math::vec3 groundNormal(0.0f);
    Math::vec3 groundPosition(0.0f);
    f32 bestDot = -FLT_MAX;

    AABB candidateBounds = mShapeRef->bounds(characterTransform);
    candidateBounds.min -= Math::vec3(maxSeparationDistance);
    candidateBounds.max += Math::vec3(maxSeparationDistance);
    QueryFilter query;
    query.collision = {mLayer, mMask};
    query.ignoredBody = mBodyId;
    mWorld->queryAABB(candidateBounds, query, mCandidates);

    for (u32 id : mCandidates)
    {
        BodyEntry* entry = mWorld->body(id);
        if (!entry || !entry->body || !entry->shape || !entry->enabled)
            continue;

        mManifolds.clear();
        if (entry->shape->type() == ShapeType::Trimesh)
        {
            const TrimeshShape& mesh = static_cast<const TrimeshShape&>(*entry->shape);
            Narrowphase::convexTrimesh(*mShapeRef, characterTransform, mesh, entry->body->transform(),
                                       mManifolds, maxSeparationDistance);
        }
        else
        {
            ContactManifold manifold;
            if (Narrowphase::collide(*mShapeRef, characterTransform, *entry->shape,
                                     entry->body->transform(), manifold, maxSeparationDistance))
                mManifolds.push_back(manifold);
        }

        for (const ContactManifold& manifold : mManifolds)
        {
            if (manifold.count == 0)
                continue;
            const Math::vec3 normal = -manifold.normal;
            const f32 dot = Math::dot(normal, mUp);
            if (dot > bestDot)
            {
                bestDot = dot;
                groundNormal = normal;
                groundPosition = manifold.points[0].position;
                groundBodyId = id;
            }
        }
    }

    mGroundBodyId = groundBodyId;
    if (groundBodyId == kInvalidBodyId)
    {
        mGroundState = GroundState::InAir;
        mGroundNormal = Math::vec3(0.0f);
        mGroundPosition = Math::vec3(0.0f);
        mGroundVelocity = Math::vec3(0.0f);
        return;
    }

    mGroundNormal = groundNormal;
    mGroundPosition = groundPosition;

    const Math::vec3 localGroundPosition = groundPosition - characterPosition;
    if (mSupportingVolume.distance(localGroundPosition) > 0.0f)
        mGroundState = GroundState::NotSupported;
    else if (Math::dot(groundNormal, mUp) < mMaxSlopeAngleCosine)
        mGroundState = GroundState::OnSteepGround;
    else
        mGroundState = GroundState::OnGround;

    const BodyEntry* groundEntry = mWorld->body(groundBodyId);
    mGroundVelocity = (groundEntry && groundEntry->body) ? groundEntry->body->velocityAtPoint(groundPosition)
                                                          : Math::vec3(0.0f);
}

} // namespace kinetix::Physics
