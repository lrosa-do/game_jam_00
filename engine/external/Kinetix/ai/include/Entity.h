#ifndef KINETIX_AI_ENTITY_H
#define KINETIX_AI_ENTITY_H

#include <kinetix/core/Types.h>

#include <kinetix/core/Math.h>
#include <ct/vector.hpp>

namespace kinetix::AI
{

class Behavior;
class Entity;
class Group;
class World;

using EntityType = kinetix::u32;

// What class an entity is, for code that needs the derived class without RTTI.
enum class EntityKind : kinetix::u8
{
    Plain,
    Squad
};

struct EntityDist
{
    float distance = 0.0f;
    Entity* entity = nullptr;
};

class Entity
{
public:
    struct Settings
    {
        EntityType type = 0;
        float senseRange = 4.0f;
        float maxVelocityChange = 1.0f; // max acceleration per iterate()
        float maxSpeed = 5.0f;
        float desiredSpeed = 2.0f;
        float radius = 0.5f;
        float moveXScalar = 1.0f; // per-axis velocity scalars (0 = locked)
        float moveYScalar = 0.0f;
        float moveZScalar = 1.0f;
    };

    Entity(World& world, const Settings& settings);
    // Unlinks from everything holding a raw pointer back to it: its Group
    // (which would otherwise delete it a second time) and its behaviors.
    virtual ~Entity();

    virtual void iterate(float timeDelta);

    void setFriendMask(EntityType mask)
    {
        mFriendMask = mask;
        mEnemyMask = ~mask;
    }
    EntityType friendMask() const
    {
        return mFriendMask;
    }
    EntityType enemyMask() const
    {
        return mEnemyMask;
    }
    EntityType type() const
    {
        return mEntityType;
    }

    // Sense data refreshed each iterate(), sorted ascending by distance; only living entities.
    const ct::Vector<EntityDist>& visibleGroupMembers() const
    {
        return mVisibleGroupMembers;
    }
    const ct::Vector<EntityDist>& visibleEnemies() const
    {
        return mVisibleEnemies;
    }

    // Plain hit points, no regen. iterate() freezes a dead entity in place (no sensing, behaviors
    // or movement) instead of removing it from its Group.
    float health() const
    {
        return mHealth;
    }
    void setHealth(float health)
    {
        mHealth = Math::max(health, 0.0f);
    }
    bool alive() const
    {
        return mHealth > 0.0f;
    }
    void applyDamage(float amount)
    {
        mHealth = Math::max(mHealth - amount, 0.0f);
    }

    // Per-entity (not in the shared Behavior) so Behavior instances stay stateless.
    float attackCooldown() const
    {
        return mAttackCooldown;
    }
    void setAttackCooldown(float seconds)
    {
        mAttackCooldown = Math::max(seconds, 0.0f);
    }
    void tickAttackCooldown(float timeDelta)
    {
        mAttackCooldown = Math::max(mAttackCooldown - timeDelta, 0.0f);
    }

    // Valid for the frame CombatBehavior fires; cleared at the top of the next iterate(). Poll
    // after World::iterate().
    bool firedThisFrame() const
    {
        return mFiredThisFrame;
    }
    Entity* lastFireTarget() const
    {
        return mLastFireTarget;
    }
    void markFired(Entity* target)
    {
        mFiredThisFrame = true;
        mLastFireTarget = target;
    }

    // Behaviors are non-owning; do not delete them here. The link is two-way
    // (see Behavior), so either side may be destroyed first.
    void addBehavior(Behavior& behavior);
    void removeBehavior(Behavior& behavior);

    void setCurrentGroup(Group* group)
    {
        mCurrentGroup = group;
    }
    Group* currentGroup() const
    {
        return mCurrentGroup;
    }

    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    const Math::vec3& velocity() const
    {
        return mVelocity;
    }
    // A non-finite velocity is ignored (the old one is kept): one NaN would
    // otherwise spread into the position and every neighbour's sensing.
    void setVelocity(const Math::vec3& velocity);
    const Math::quat& orientation() const
    {
        return mOrientation;
    }
    void setOrientation(const Math::quat& orientation)
    {
        mOrientation = orientation;
    }
    const Math::vec3& desiredMove() const
    {
        return mDesiredMoveVector;
    }
    void setDesiredMove(const Math::vec3& move)
    {
        mDesiredMoveVector = move;
    }

    float maxSpeed() const
    {
        return mMaxSpeed;
    }
    // Read every iterate(), so safe to change on a live entity.
    void setMaxSpeed(float speed)
    {
        mMaxSpeed = Math::max(speed, 0.0f);
    }
    float desiredSpeed() const
    {
        return mDesiredSpeed;
    }
    void setDesiredSpeed(float speed)
    {
        mDesiredSpeed = Math::max(speed, 0.0f);
    }
    float senseRange() const
    {
        return mSenseRange;
    }

    // Local frame: right = +X, up = +Y, forward (look) = +Z, from the orientation quaternion.
    // Differs from runtime/scene (GameObject::forward() = -Z): copying mOrientation onto a
    // GameObject faces it 180 degrees off, so turn 180 degrees around up() first.
    Math::vec3 forward() const
    {
        return Math::mat3_cast(mOrientation)[2];
    }
    Math::vec3 side() const
    {
        return Math::mat3_cast(mOrientation)[0];
    }
    Math::vec3 up() const
    {
        return Math::mat3_cast(mOrientation)[1];
    }

    float speed() const
    {
        return Math::length(mVelocity);
    }
    void setSpeed(float newSpeed);

    float radius() const
    {
        return mRadius;
    }
    void setRadius(float radius)
    {
        mRadius = radius;
    }

    // Maps to maxVelocityChange, the acceleration limit iterate() clamps against.
    float maxForce() const
    {
        return mMaxVelocityChange;
    }
    void setMaxForce(float force)
    {
        mMaxVelocityChange = force;
    }

    Math::vec3 predictFuturePosition(float predictionTime) const
    {
        return mPosition + (mVelocity * predictionTime);
    }

    Math::vec3 localizeDirection(const Math::vec3& globalDirection) const;
    Math::vec3 localizePosition(const Math::vec3& globalPosition) const;
    Math::vec3 globalizePosition(const Math::vec3& localPosition) const;
    Math::vec3 globalizeDirection(const Math::vec3& localDirection) const;

    void alignWithVelocity();

    World& world() const
    {
        return mWorld;
    }

    EntityKind kind() const
    {
        return mKind;
    }

protected:
    friend class Behavior;

    void updateGroupVisibility();
    void updateEnemyVisibility();
    bool visibilityTest(const Entity& other, float& dist) const;

    Group* mCurrentGroup = nullptr;
    ct::Vector<Behavior*> mBehaviors;
    World& mWorld;
    EntityType mFriendMask = 0;
    EntityType mEnemyMask = ~EntityType(0);
    EntityType mEntityType = 0;
    EntityKind mKind = EntityKind::Plain;

    Math::vec3 mPosition = Math::vec3(0.0f);
    Math::vec3 mVelocity = Math::vec3(0.0f);
    Math::quat mOrientation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 mDesiredMoveVector = Math::vec3(0.0f);

    float mSenseRange = 4.0f;
    float mMaxVelocityChange = 1.0f;
    float mMaxSpeed = 5.0f;
    float mDesiredSpeed = 2.0f;
    float mRadius = 0.5f;
    float mMoveXScalar = 1.0f;
    float mMoveYScalar = 0.0f;
    float mMoveZScalar = 1.0f;

    ct::Vector<EntityDist> mVisibleGroupMembers;
    ct::Vector<EntityDist> mVisibleEnemies;

    float mHealth = 100.0f;
    // Reused by the two sensing passes so a frame allocates nothing.
    ct::Vector<Entity*> mSenseScratch;
    float mAttackCooldown = 0.0f;
    bool mFiredThisFrame = false;
    Entity* mLastFireTarget = nullptr;
};

}

#endif
