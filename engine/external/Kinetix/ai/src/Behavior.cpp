#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "Behavior.h"

#include "AIInternal.h"
#include "Entity.h"

#include <math.h>
#include <stdlib.h>

#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

using detail::safeNormalize;

Behavior::~Behavior()
{
    // Dying before the entities that use it: take this behavior out of each
    // of them, or their next iterate() calls into freed memory.
    for (Entity* entity : mAttached)
    {
        ct::Vector<Behavior*>& behaviors = entity->mBehaviors;
        for (usize i = behaviors.size(); i > 0; --i)
        {
            if (behaviors[i - 1] == this)
                behaviors.erase(behaviors.begin() + (i - 1));
        }
    }
    mAttached.clear();
}

void Behavior::attach(Entity& entity)
{
    mAttached.push_back(&entity);
}

void Behavior::detach(Entity& entity)
{
    auto it = kinetix::Find(mAttached.begin(), mAttached.end(), &entity);
    if (it == mAttached.end())
        return;
    mAttached.erase(it);
    if (kinetix::Find(mAttached.begin(), mAttached.end(), &entity) == mAttached.end())
        onDetach(entity);
}

SeparationBehavior::SeparationBehavior(float separationDistance, float minPercent, float maxPercent)
    : mSeparationDistance(separationDistance), mMinSeparationPercentage(minPercent),
      mMaxSeparationPercentage(maxPercent)
{
}

void SeparationBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    const ct::Vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    // Repel from EVERY neighbour inside the separation distance (summed, with a falloff): reacting
    // to one lets a school collapse into a dense ball at the centre of mass where separation and
    // cohesion fight every frame.
    Math::vec3 separationPush(0.0f);
    for (const EntityDist& member : groupMembers)
    {
        const float d = member.distance;
        if (d >= mSeparationDistance)
            continue;

        // 0.0 at the separation edge, growing to (1 - minPct) as the
        // neighbour fully overlaps us.
        const float pct =
            kinetix::Clamp(d / mSeparationDistance, mMinSeparationPercentage, mMaxSeparationPercentage);

        Math::vec3 away = member.entity->position() - entity.position();
        if (Math::dot(away, away) < 1e-8f)
            away = Math::vec3(1.0f, 0.0f, 0.0f); // coincident - pick a stable direction
        else
            away = safeNormalize(away);

        separationPush -= away * (1.0f - pct);
    }

    if (Math::dot(separationPush, separationPush) < 1e-8f)
        return;

    Math::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += safeNormalize(separationPush) * gain();
    entity.setDesiredMove(currentDesiredMove);
}

AlignmentBehavior::AlignmentBehavior(float turnRate) : mTurnRate(turnRate)
{
}

void AlignmentBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    const ct::Vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    const Entity& nearestGroupMember = *groupMembers.front().entity;

    Math::vec3 desiredMoveAdj = safeNormalize(nearestGroupMember.velocity()) * mTurnRate;
    Math::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

CohesionBehavior::CohesionBehavior(float turnRate) : mTurnRate(turnRate)
{
}

void CohesionBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    const ct::Vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    Math::vec3 groupCenterOfMass(0.0f);
    for (const EntityDist& member : groupMembers)
        groupCenterOfMass += member.entity->position();
    groupCenterOfMass /= static_cast<float>(groupMembers.size());

    // Dead zone: ON the centre of mass the direction is floating-point noise and the force would
    // fight separation every frame. Skip it until we drift away.
    const Math::vec3 toCenterOfMass = groupCenterOfMass - entity.position();
    if (Math::dot(toCenterOfMass, toCenterOfMass) < 0.0625f) // < 0.25 units
        return;

    Math::vec3 desiredMoveAdj = safeNormalize(toCenterOfMass) * mTurnRate;
    Math::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

AvoidanceBehavior::AvoidanceBehavior(float avoidanceDistance, float avoidanceSpeed)
    : mAvoidanceDistance(avoidanceDistance), mAvoidanceSpeed(avoidanceSpeed)
{
}

void AvoidanceBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    const ct::Vector<EntityDist>& enemies = entity.visibleEnemies();
    if (enemies.empty())
        return;

    const Entity& nearestEnemy = *enemies.front().entity;
    float nearestEnemyDist = enemies.front().distance;

    if (nearestEnemyDist < mAvoidanceDistance)
    {
        Math::vec3 desiredMoveAdj =
            safeNormalize(entity.position() - nearestEnemy.position()) * mAvoidanceSpeed;
        Math::vec3 currentDesiredMove = entity.desiredMove();
        currentDesiredMove += desiredMoveAdj * gain();
        entity.setDesiredMove(currentDesiredMove);
    }
}

CruisingBehavior::CruisingBehavior(float randMoveXChance, float randMoveYChance,
                                   float randMoveZChance, float minRandomMove, float maxRateChange,
                                   float minRateChange)
    : mRandMoveXChance(randMoveXChance), mRandMoveYChance(randMoveYChance),
      mRandMoveZChance(randMoveZChance), mMinRandomMove(minRandomMove),
      mMaxRateChange(maxRateChange), mMinRateChange(minRateChange)
{
}

void CruisingBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    float currentSpeed = Math::length(entity.velocity());
    float percentDesiredSpeed =
        fabsf((currentSpeed - entity.desiredSpeed()) / entity.maxSpeed());
    float signum = (currentSpeed - entity.desiredSpeed()) > 0.0f ? -1.0f : 1.0f;

    percentDesiredSpeed = kinetix::Clamp(percentDesiredSpeed, mMinRateChange, mMaxRateChange);

    // The chances are per-axis probabilities, so the roll is tested against cumulative bands;
    // comparing each against the raw roll would make an axis unreachable whenever its chance is
    // smaller than the previous one.
    Math::vec3 desiredMoveAdj(0.0f);
    float randmove = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    if (randmove < mRandMoveXChance)
        desiredMoveAdj.x += mMinRandomMove * signum;
    else if (randmove < mRandMoveXChance + mRandMoveYChance)
        desiredMoveAdj.y += mMinRandomMove * signum;
    else if (randmove < mRandMoveXChance + mRandMoveYChance + mRandMoveZChance)
        desiredMoveAdj.z += mMinRandomMove * signum;

    Math::vec3 currentDesiredMove = entity.desiredMove();
    desiredMoveAdj = safeNormalize(desiredMoveAdj) * (percentDesiredSpeed * signum);
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

StayWithinSphereBehavior::StayWithinSphereBehavior(const Math::vec3& center, float radius)
    : mCenter(center), mRadius(radius)
{
}

void StayWithinSphereBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    Math::vec3 toCenter = mCenter - entity.position();
    float dist = Math::length(toCenter);
    if (dist > mRadius)
    {
        Math::vec3 desiredMoveAdj = safeNormalize(toCenter) * entity.maxSpeed();
        Math::vec3 currentDesiredMove = entity.desiredMove();
        currentDesiredMove += desiredMoveAdj * gain();
        entity.setDesiredMove(currentDesiredMove);
    }
}

CombatBehavior::CombatBehavior(float fireRange, float damagePerHit, float fireInterval)
    : mFireRange(fireRange), mDamagePerHit(damagePerHit), mFireInterval(fireInterval)
{
}

void CombatBehavior::iterate(float timeDelta, Entity& entity)
{
    entity.tickAttackCooldown(timeDelta);

    const ct::Vector<EntityDist>& enemies = entity.visibleEnemies();
    if (enemies.empty() || entity.attackCooldown() > 0.0f)
        return;

    const EntityDist& nearest = enemies.front();
    if (nearest.distance > mFireRange)
        return;

    nearest.entity->applyDamage(mDamagePerHit);
    entity.setAttackCooldown(mFireInterval);
    entity.markFired(nearest.entity);
}

}
