#ifndef KINETIX_AI_BEHAVIOR_H
#define KINETIX_AI_BEHAVIOR_H

#include <kinetix/core/Math.h>

#include <ct/vector.hpp>

namespace kinetix::AI
{

class Entity;

// A Behavior is shared, non-owning, between any number of entities, and the
// link is kept in both directions: an entity that dies, or drops the
// behavior, tells it through onDetach() (so per-entity state keyed by the
// entity can go), and a behavior that dies first takes itself out of every
// entity still holding it.
class Behavior
{
public:
    Behavior() = default;
    // Copies the settings, never the attachments: a copy starts unattached.
    Behavior(const Behavior& other) : mGain(other.mGain)
    {
    }
    Behavior& operator=(const Behavior& other)
    {
        mGain = other.mGain;
        return *this;
    }
    virtual ~Behavior();

    virtual void iterate(float timeDelta, Entity& entity) = 0;

    float gain() const
    {
        return mGain;
    }
    void setGain(float gain)
    {
        mGain = gain;
    }

    virtual const char* name() const
    {
        return "Base Behavior";
    }

protected:
    // `entity` has dropped its last attachment to this behavior, or is being
    // destroyed. Called before the entity's memory goes away; the entity
    // must not be iterated or stored from here.
    virtual void onDetach(Entity& entity)
    {
        (void)entity;
    }

private:
    friend class Entity;
    void attach(Entity& entity);
    void detach(Entity& entity);

    float mGain = 1.0f;
    ct::Vector<Entity*> mAttached;
};

// Push away from the closest visible group member when inside the separation
// distance; pull toward it when it drifts beyond it.
class SeparationBehavior final : public Behavior
{
public:
    SeparationBehavior(float separationDistance, float minPercent, float maxPercent);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Separation Behavior";
    }

    float separationDistance() const
    {
        return mSeparationDistance;
    }
    void setSeparationDistance(float dist)
    {
        mSeparationDistance = dist;
    }
    float minSeparationPercentage() const
    {
        return mMinSeparationPercentage;
    }
    void setMinSeparationPercentage(float pct)
    {
        mMinSeparationPercentage = pct;
    }
    float maxSeparationPercentage() const
    {
        return mMaxSeparationPercentage;
    }
    void setMaxSeparationPercentage(float pct)
    {
        mMaxSeparationPercentage = pct;
    }

private:
    float mSeparationDistance;
    float mMinSeparationPercentage;
    float mMaxSeparationPercentage;
};

class AlignmentBehavior final : public Behavior
{
public:
    explicit AlignmentBehavior(float turnRate);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Alignment Behavior";
    }

    float turnRate() const
    {
        return mTurnRate;
    }
    void setTurnRate(float rate)
    {
        mTurnRate = rate;
    }

private:
    float mTurnRate;
};

class CohesionBehavior final : public Behavior
{
public:
    explicit CohesionBehavior(float turnRate);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Cohesion Behavior";
    }

    float turnRate() const
    {
        return mTurnRate;
    }
    void setTurnRate(float rate)
    {
        mTurnRate = rate;
    }

private:
    float mTurnRate;
};

// Flee from the closest visible enemy inside the avoidance distance.
class AvoidanceBehavior final : public Behavior
{
public:
    AvoidanceBehavior(float avoidanceDistance, float avoidanceSpeed);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Avoidance Behavior";
    }

    float avoidanceDistance() const
    {
        return mAvoidanceDistance;
    }
    void setAvoidanceDistance(float dist)
    {
        mAvoidanceDistance = dist;
    }
    float avoidanceSpeed() const
    {
        return mAvoidanceSpeed;
    }
    void setAvoidanceSpeed(float speed)
    {
        mAvoidanceSpeed = speed;
    }

private:
    float mAvoidanceDistance;
    float mAvoidanceSpeed;
};

// Wander: nudge the desired move toward/away from the desired speed and throw
// in occasional random per-axis movement.
class CruisingBehavior final : public Behavior
{
public:
    CruisingBehavior(float randMoveXChance, float randMoveYChance, float randMoveZChance,
                     float minRandomMove, float maxRateChange, float minRateChange);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Cruising Behavior";
    }

    float randMoveXChance() const
    {
        return mRandMoveXChance;
    }
    void setRandMoveXChance(float chance)
    {
        mRandMoveXChance = chance;
    }
    float randMoveYChance() const
    {
        return mRandMoveYChance;
    }
    void setRandMoveYChance(float chance)
    {
        mRandMoveYChance = chance;
    }
    float randMoveZChance() const
    {
        return mRandMoveZChance;
    }
    void setRandMoveZChance(float chance)
    {
        mRandMoveZChance = chance;
    }
    float minRandomMove() const
    {
        return mMinRandomMove;
    }
    void setMinRandomMove(float move)
    {
        mMinRandomMove = move;
    }
    float maxRateChange() const
    {
        return mMaxRateChange;
    }
    void setMaxRateChange(float rateChange)
    {
        mMaxRateChange = rateChange;
    }
    float minRateChange() const
    {
        return mMinRateChange;
    }
    void setMinRateChange(float rateChange)
    {
        mMinRateChange = rateChange;
    }

private:
    float mRandMoveXChance;
    float mRandMoveYChance;
    float mRandMoveZChance;
    float mMinRandomMove;
    float mMaxRateChange;
    float mMinRateChange;
};

class StayWithinSphereBehavior final : public Behavior
{
public:
    StayWithinSphereBehavior(const Math::vec3& center, float radius);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Stay Within Sphere Behavior";
    }

    const Math::vec3& sphereCenter() const
    {
        return mCenter;
    }
    void setSphereCenter(const Math::vec3& center)
    {
        mCenter = center;
    }
    float sphereRadius() const
    {
        return mRadius;
    }
    void setSphereRadius(float radius)
    {
        mRadius = radius;
    }

private:
    Math::vec3 mCenter;
    float mRadius;
};

// Reactive only: damages the nearest visible enemy within fire range once Entity::attackCooldown()
// (ticked every frame regardless of range) has elapsed; never moves the entity.
class CombatBehavior final : public Behavior
{
public:
    CombatBehavior(float fireRange, float damagePerHit, float fireInterval);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Combat Behavior";
    }

    float fireRange() const
    {
        return mFireRange;
    }
    void setFireRange(float range)
    {
        mFireRange = range;
    }
    float damagePerHit() const
    {
        return mDamagePerHit;
    }
    void setDamagePerHit(float damage)
    {
        mDamagePerHit = damage;
    }
    float fireInterval() const
    {
        return mFireInterval;
    }
    void setFireInterval(float interval)
    {
        mFireInterval = interval;
    }

private:
    float mFireRange;
    float mDamagePerHit;
    float mFireInterval;
};

}

#endif
