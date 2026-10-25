#include "prisma3d/scene/CharacterBody3D.h"

#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/NodeClassDB.h"

#include <math.h>

namespace prisma3d
{

CharacterBody3D::CharacterBody3D()
        : PhysicsBody3D(PhysicsBodyMode::Kinematic),
          mVelocity(0.0f, 0.0f, 0.0f),
          mPushForce(0.0f),
          mHasShape(false)
{
}

CharacterBody3D::~CharacterBody3D()
{
    release();
}

void CharacterBody3D::release()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mCharacter.valid()) server->characterFree(mCharacter);
    mCharacter = PhysicsCharacterId();
}

void CharacterBody3D::gatherShape()
{
    mHasShape = false;
    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        const CollisionShape3D* shape = getChild(i)->castTo<CollisionShape3D>();
        if (!shape || shape->isDisabled() || shape->getKind() != CollisionShapeKind::Capsule)
            continue;
        const float radius = shape->getCapsuleRadius();
        const float total = shape->getCapsuleHeight();
        mParameters.radius = radius;
        mParameters.height = total > radius * 2.0f ? total - radius * 2.0f : 0.0f;
        mHasShape = true;
        return;
    }
}

void CharacterBody3D::setVelocity(const Math::Vec3& velocity)
{
    mVelocity = velocity;
}

const Math::Vec3& CharacterBody3D::getVelocity() const
{
    return mVelocity;
}

bool CharacterBody3D::moveAndSlide(float deltaTime)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !isInsideTree()) return false;
    if (!(deltaTime > 0.0f))
    {
        deltaTime = static_cast<float>(getPhysicsProcessDeltaTime());
        if (!(deltaTime > 0.0f)) deltaTime = 1.0f / 60.0f;
    }

    gatherShape();
    mParameters.collisionLayer = getCollisionLayer();
    mParameters.collisionMask = getCollisionMask();
    if (!server->characterValid(mCharacter))
        mCharacter = server->characterCreate(mParameters, Math::Vec3(0.0f, 0.0f, 0.0f));
    else
        server->characterSetParameters(mCharacter, mParameters);
    server->characterSetIgnoredBody(mCharacter, getBody());

    Math::Mat4 frame = getGlobalTransform();
    server->characterSetPosition(mCharacter, Math::Vec3(frame.col3.x, frame.col3.y, frame.col3.z));
    server->characterSetVelocity(mCharacter, mVelocity);
    mState = server->characterMoveAndSlide(mCharacter, deltaTime);
    mVelocity = mState.velocity;

    frame.col3 = Math::Vec4(mState.position.x, mState.position.y, mState.position.z, 1.0f);
    setGlobalTransform(frame);

    if (mPushForce > 0.0f)
    {
        const Math::Vec3& up = mParameters.up;
        const std::size_t hits = server->characterGetHitCount(mCharacter);
        for (std::size_t i = 0; i < hits; ++i)
        {
            const PhysicsCharacterHit hit = server->characterGetHit(mCharacter, i);
            if (fabsf(Math::Vec3::Dot(hit.normal, up)) > 0.7f) continue;
            RigidBody3D* rigid = nullptr;
            if (PhysicsBody3D* other = PhysicsBody3D::fromBody(hit.body))
                rigid = other->castTo<RigidBody3D>();
            if (!rigid) continue;
            Math::Vec3 push = hit.normal * -1.0f;
            push = push - up * Math::Vec3::Dot(push, up);
            if (push.Length() < 1e-4f) continue;
            rigid->applyCentralImpulse(push.Normalized() * (mPushForce * deltaTime));
        }
    }
    return true;
}

bool CharacterBody3D::isOnFloor() const
{
    return mState.onFloor;
}

bool CharacterBody3D::isOnWall() const
{
    return mState.onWall;
}

bool CharacterBody3D::isOnCeiling() const
{
    return mState.onCeiling;
}

const Math::Vec3& CharacterBody3D::getFloorNormal() const
{
    return mState.floorNormal;
}

const Math::Vec3& CharacterBody3D::getWallNormal() const
{
    return mState.wallNormal;
}

const Math::Vec3& CharacterBody3D::getCeilingNormal() const
{
    return mState.ceilingNormal;
}

float CharacterBody3D::getFloorAngle() const
{
    return mState.slopeAngle;
}

bool CharacterBody3D::isNearGround(float distance) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server && server->characterValid(mCharacter) &&
           server->characterIsNearGround(mCharacter, distance);
}

void CharacterBody3D::setCapsule(float radius, float height)
{
    mParameters.radius = radius;
    mParameters.height = height;
}

float CharacterBody3D::getRadius() const
{
    return mParameters.radius;
}

float CharacterBody3D::getHeight() const
{
    return mParameters.height;
}

void CharacterBody3D::setSlopeLimit(float degrees)
{
    mParameters.slopeLimit = degrees;
}

float CharacterBody3D::getSlopeLimit() const
{
    return mParameters.slopeLimit;
}

void CharacterBody3D::setStepOffset(float offset)
{
    mParameters.stepOffset = offset;
}

float CharacterBody3D::getStepOffset() const
{
    return mParameters.stepOffset;
}

void CharacterBody3D::setGroundSnap(float distance)
{
    mParameters.groundSnap = distance;
}

float CharacterBody3D::getGroundSnap() const
{
    return mParameters.groundSnap;
}

void CharacterBody3D::setSkinWidth(float width)
{
    mParameters.skinWidth = width;
}

float CharacterBody3D::getSkinWidth() const
{
    return mParameters.skinWidth;
}

void CharacterBody3D::setWallMinSlideAngle(float degrees)
{
    mParameters.wallMinSlideAngle = degrees;
}

float CharacterBody3D::getWallMinSlideAngle() const
{
    return mParameters.wallMinSlideAngle;
}

void CharacterBody3D::setUpDirection(const Math::Vec3& up)
{
    mParameters.up = up;
}

const Math::Vec3& CharacterBody3D::getUpDirection() const
{
    return mParameters.up;
}

void CharacterBody3D::setFloorStopOnSlope(bool enabled)
{
    mParameters.floorStopOnSlope = enabled;
}

bool CharacterBody3D::isFloorStopOnSlope() const
{
    return mParameters.floorStopOnSlope;
}

void CharacterBody3D::setSlideOnCeiling(bool enabled)
{
    mParameters.slideOnCeiling = enabled;
}

bool CharacterBody3D::isSlideOnCeiling() const
{
    return mParameters.slideOnCeiling;
}

void CharacterBody3D::setPushForce(float force)
{
    mPushForce = force;
}

float CharacterBody3D::getPushForce() const
{
    return mPushForce;
}

void CharacterBody3D::setMaxSlides(int slides)
{
    mParameters.maxIterations = slides < 1 ? 1 : slides;
}

int CharacterBody3D::getMaxSlides() const
{
    return mParameters.maxIterations;
}

std::size_t CharacterBody3D::getSlideCollisionCount() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->characterGetHitCount(mCharacter) : 0;
}

PhysicsBody3D* CharacterBody3D::getSlideCollider(std::size_t index) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return nullptr;
    return PhysicsBody3D::fromBody(server->characterGetHit(mCharacter, index).body);
}

Math::Vec3 CharacterBody3D::getSlideNormal(std::size_t index) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return Math::Vec3(0.0f, 1.0f, 0.0f);
    return server->characterGetHit(mCharacter, index).normal;
}

PhysicsCharacterId CharacterBody3D::getCharacter() const
{
    return mCharacter;
}

void CharacterBody3D::notificationHandler(int what)
{
    PhysicsBody3D::notificationHandler(what);
    if (what == kNotificationExitTree) release();
}

void CharacterBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = CharacterBody3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "velocity"), &P::setVelocity,
            &P::getVelocity);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "up_direction"), &P::setUpDirection,
            &P::getUpDirection);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "max_slides", 1.0, 256.0, 1.0, true),
            &P::setMaxSlides, &P::getMaxSlides);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "radius", 0.001, 100.0, 0.001, true),
            [](P* body, const PropertyValue& value)
            { body->setCapsule(static_cast<float>(value.asFloat()), body->getHeight()); },
            &P::getRadius);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "height", 0.0, 100.0, 0.001, true),
            [](P* body, const PropertyValue& value)
            { body->setCapsule(body->getRadius(), static_cast<float>(value.asFloat())); },
            &P::getHeight);

    DB::addPropertyGroup<P>("Floor", "floor_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "floor_stop_on_slope"),
            &P::setFloorStopOnSlope, &P::isFloorStopOnSlope);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "slope_limit", 0.0, 90.0, 0.1),
            &P::setSlopeLimit, &P::getSlopeLimit);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "step_offset", 0.0, 10.0, 0.001, true),
            &P::setStepOffset, &P::getStepOffset);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "ground_snap", 0.0, 10.0, 0.001, true),
            &P::setGroundSnap, &P::getGroundSnap);

    DB::addPropertyGroup<P>("Wall", "wall_");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "wall_min_slide_angle", 0.0, 180.0, 0.1),
            &P::setWallMinSlideAngle, &P::getWallMinSlideAngle);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "slide_on_ceiling"), &P::setSlideOnCeiling,
            &P::isSlideOnCeiling);

    DB::addPropertyGroup<P>("Collision", "");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "skin_width", 0.0001, 1.0, 0.0001, true),
            &P::setSkinWidth, &P::getSkinWidth);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "push_force", 0.0, 10000.0, 0.01, true),
            &P::setPushForce, &P::getPushForce);
}

} // namespace prisma3d
