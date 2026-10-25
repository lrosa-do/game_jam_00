#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "Entity.h"

#include "Behavior.h"
#include "Group.h"
#include "World.h"

#include <kinetix/core/Algorithm.h>

#include <ct/sort.hpp>

#include <math.h>

namespace kinetix::AI
{

namespace
{
// Appends; the caller sorts once when the pass is done (inserting in place shifted the tail per
// neighbour).
void appendDist(ct::Vector<EntityDist>& list, Entity* entity, float distance)
{
    list.push_back(EntityDist{distance, entity});
}

bool finite(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

void sortByDistance(ct::Vector<EntityDist>& list)
{
    ct::sort(list.begin(), list.end(),
             [](const EntityDist& a, const EntityDist& b) { return a.distance < b.distance; });
}
}

Entity::Entity(World& world, const Settings& settings)
    : mWorld(world), mFriendMask(settings.type), mEnemyMask(~settings.type),
      mEntityType(settings.type), mSenseRange(settings.senseRange),
      mMaxVelocityChange(settings.maxVelocityChange), mMaxSpeed(settings.maxSpeed),
      mDesiredSpeed(settings.desiredSpeed), mRadius(settings.radius),
      mMoveXScalar(settings.moveXScalar), mMoveYScalar(settings.moveYScalar),
      mMoveZScalar(settings.moveZScalar)
{
}

Entity::~Entity()
{
    // Swapped out first: detach() runs user code (onDetach) and must not see
    // a half-erased list.
    ct::Vector<Behavior*> behaviors = kinetix::Move(mBehaviors);
    mBehaviors.clear();
    for (Behavior* behavior : behaviors)
        behavior->detach(*this);

    if (mCurrentGroup)
        mCurrentGroup->remove(*this);
}

void Entity::setVelocity(const Math::vec3& velocity)
{
    if (finite(velocity))
        mVelocity = velocity;
}

void Entity::setSpeed(float newSpeed)
{
    float spd = Math::length(mVelocity);
    if (spd > 0.0f)
        mVelocity *= newSpeed / spd;
    else
        mVelocity = forward() * newSpeed;
}

void Entity::alignWithVelocity()
{
    float spd = Math::length(mVelocity);
    // A direction from a near-zero velocity is noise; updating the orientation from it makes
    // formation goals rotate while the squad is at rest.
    if (spd <= 0.1f)
        return;

    // Right-handed orthonormal regeneration: forward = velocity, side = normalize(cross(up,
    // forward)), up = cross(forward, side).
    Math::vec3 newForward = mVelocity / spd;
    Math::vec3 oldUp = up();
    Math::vec3 sideReference = oldUp;
    // A forward vector parallel to up has no valid cross product. Keep the previous side (projected
    // onto the plane perpendicular to forward) so a near-vertical velocity cannot produce NaNs.
    Math::vec3 newSide = Math::cross(sideReference, newForward);
    if (Math::dot(newSide, newSide) <= 1e-8f)
        newSide = side();
    newSide -= newForward * Math::dot(newSide, newForward);
    if (Math::dot(newSide, newSide) <= 1e-8f)
        newSide = Math::cross(Math::vec3(0.0f, 1.0f, 0.0f), newForward);
    newSide = Math::normalize(newSide);
    Math::vec3 newUp = Math::cross(newForward, newSide);

    mOrientation = Math::quat_cast(Math::mat3(newSide, newUp, newForward));
}

Math::vec3 Entity::localizeDirection(const Math::vec3& globalDirection) const
{
    const Math::mat3 basis(side(), up(), forward());
    return Math::transpose(basis) * globalDirection;
}

Math::vec3 Entity::localizePosition(const Math::vec3& globalPosition) const
{
    const Math::mat3 basis(side(), up(), forward());
    return Math::transpose(basis) * (globalPosition - mPosition);
}

Math::vec3 Entity::globalizePosition(const Math::vec3& localPosition) const
{
    const Math::mat3 basis(side(), up(), forward());
    return mPosition + (basis * localPosition);
}

Math::vec3 Entity::globalizeDirection(const Math::vec3& localDirection) const
{
    const Math::mat3 basis(side(), up(), forward());
    return basis * localDirection;
}

void Entity::iterate(float timeDelta)
{
    // A dead entity is frozen in place instead of removed from its Group: removal would invalidate
    // Group::entities() iterators mid-World::iterate() (death is discovered inside another entity's
    // CombatBehavior pass).
    if (!alive())
    {
        mVelocity = Math::vec3(0.0f);
        mDesiredMoveVector = Math::vec3(0.0f);
        return;
    }

    mPosition += mVelocity * timeDelta;

    // Cleared here so a shot fired this pass stays visible for exactly one frame.
    mFiredThisFrame = false;
    mLastFireTarget = nullptr;

    // Per-frame accumulator; keeping the previous value would make acceleration compound forever.
    mDesiredMoveVector = Math::vec3(0.0f);

    mVisibleGroupMembers.clear();
    mVisibleEnemies.clear();
    updateGroupVisibility();
    updateEnemyVisibility();

    for (Behavior* behavior : mBehaviors)
        behavior->iterate(timeDelta, *this);

    // A behavior that divided by zero must not poison the entity for good.
    if (!finite(mDesiredMoveVector))
        mDesiredMoveVector = Math::vec3(0.0f);

    const float speedBefore = Math::length(mVelocity);
    float velChange = Math::length(mDesiredMoveVector);
    if (velChange > mMaxVelocityChange && velChange > 0.0f)
        mDesiredMoveVector = Math::normalize(mDesiredMoveVector) * mMaxVelocityChange;

    mVelocity += mDesiredMoveVector;

    // Per-axis scaling; > 1.0f destabilises the system.
    mVelocity.x *= mMoveXScalar;
    mVelocity.y *= mMoveYScalar;
    mVelocity.z *= mMoveZScalar;

    float speed = Math::length(mVelocity);
    if (speed > mMaxSpeed && speed > 0.0f)
        mVelocity = Math::normalize(mVelocity) * mMaxSpeed;

    // Dead zone: snap tiny residual velocities to rest, only while the steering is not speeding the
    // agent up and never above half its top speed (an absolute snap would swallow every
    // acceleration smaller than itself, so a gently pushed agent could never get going).
    const float deadZone = kinetix::Min(0.1f, 0.5f * mMaxSpeed);
    speed = Math::length(mVelocity);
    if (speed < deadZone && speed <= speedBefore)
        mVelocity = Math::vec3(0.0f);

    if (!finite(mVelocity))
        mVelocity = Math::vec3(0.0f);
}

void Entity::updateGroupVisibility()
{
    // One query answers both sense passes (same centre and radius); walking every group made a
    // crowd quadratic.
    ct::Vector<Entity*>& nearby = mSenseScratch;
    mWorld.proximityGrid().query(mPosition, mSenseRange, nearby);

    for (Entity* other : nearby)
    {
        if (other == this || !other->alive())
            continue;

        float dist = 0.0f;
        if (!visibilityTest(*other, dist))
            continue;

        if (mCurrentGroup && other->mCurrentGroup == mCurrentGroup)
            appendDist(mVisibleGroupMembers, other, dist);

        if ((other->type() & enemyMask()) != 0)
            appendDist(mVisibleEnemies, other, dist);
    }

    sortByDistance(mVisibleGroupMembers);
    sortByDistance(mVisibleEnemies);
}

// Both lists are filled by updateGroupVisibility() from a single query.
void Entity::updateEnemyVisibility()
{
}

bool Entity::visibilityTest(const Entity& other, float& dist) const
{
    Math::vec3 distVec = other.position() - position();
    dist = Math::length(distVec);
    return dist < mSenseRange;
}

void Entity::addBehavior(Behavior& behavior)
{
    mBehaviors.push_back(&behavior);
    behavior.attach(*this);
}

void Entity::removeBehavior(Behavior& behavior)
{
    auto it = kinetix::Find(mBehaviors.begin(), mBehaviors.end(), &behavior);
    if (it == mBehaviors.end())
        return;
    mBehaviors.erase(it);
    behavior.detach(*this);
}

}
