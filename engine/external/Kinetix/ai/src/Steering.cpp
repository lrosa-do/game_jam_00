

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "Steering.h"

#include "AIInternal.h"

#include <float.h>
#include <math.h>

namespace kinetix::AI
{

using detail::clip;
using detail::intervalComparison;
using detail::perpendicularComponent;
using detail::safeNormalize;
using detail::scalarRandomWalk;

Math::vec3 SteerLibrary::seek(const Math::vec3& target) const
{
    const Math::vec3 desiredVelocity = target - vehicle().position();
    return desiredVelocity - vehicle().velocity();
}

Math::vec3 SteerLibrary::flee(const Math::vec3& target) const
{
    const Math::vec3 desiredVelocity = vehicle().position() - target;
    return desiredVelocity - vehicle().velocity();
}

Math::vec3 SteerLibrary::wander(float dt)
{
    const float speed = 12.0f * dt;
    wanderSide = scalarRandomWalk(wanderSide, speed, -1.0f, +1.0f);
    wanderUp = scalarRandomWalk(wanderUp, speed, -1.0f, +1.0f);

    return (vehicle().side() * wanderSide) + (vehicle().up() * wanderUp);
}

Math::vec3 SteerLibrary::pursuit(const Entity& quarry) const
{
    return pursuit(quarry, FLT_MAX);
}

Math::vec3 SteerLibrary::pursuit(const Entity& quarry, float maxPredictionTime) const
{
    const Math::vec3 offset = quarry.position() - vehicle().position();
    const float distance = Math::length(offset);
    const Math::vec3 unitOffset = distance > 0.0f ? offset / distance : vehicle().forward();

    const float parallelness = Math::dot(vehicle().forward(), quarry.forward());

    const float forwardness = Math::dot(vehicle().forward(), unitOffset);

    // Estimated time to intercept at current speed (0 if not moving yet, so a
    // parked quarry doesn't produce NaN).
    const float directTravelTime = vehicle().speed() > 0.0f ? distance / vehicle().speed() : 0.0f;
    const int f = intervalComparison(forwardness, -0.707f, 0.707f);
    const int p = intervalComparison(parallelness, -0.707f, 0.707f);

    float timeFactor = 0.0f;
    switch (f)
    {
    case +1: // ahead
        switch (p)
        {
        case +1:
            timeFactor = 4.0f;
            break; // ahead, parallel
        case 0:
            timeFactor = 1.8f;
            break; // ahead, perpendicular
        case -1:
            timeFactor = 0.85f;
            break; // ahead, anti-parallel
        }
        break;
    case 0: // aside
        switch (p)
        {
        case +1:
            timeFactor = 1.0f;
            break; // aside, parallel
        case 0:
            timeFactor = 0.8f;
            break; // aside, perpendicular
        case -1:
            timeFactor = 4.0f;
            break; // aside, anti-parallel
        }
        break;
    case -1: // behind
        switch (p)
        {
        case +1:
            timeFactor = 0.5f;
            break; // behind, parallel
        case 0:
            timeFactor = 2.0f;
            break; // behind, perpendicular
        case -1:
            timeFactor = 2.0f;
            break; // behind, anti-parallel
        }
        break;
    }

    const float et = directTravelTime * timeFactor;
    const float etl = (et > maxPredictionTime) ? maxPredictionTime : et;

    const Math::vec3 target = quarry.predictFuturePosition(etl);

    return seek(target);
}

Math::vec3 SteerLibrary::evasion(const Entity& menace, float maxPredictionTime) const
{
    const Math::vec3 offset = menace.position() - vehicle().position();
    const float distance = Math::length(offset);

    // Predicted intercept time, capped at maxPredictionTime (a stationary
    // menace is predicted at the cap rather than dividing by zero).
    const float roughTime = menace.speed() > 0.0f ? distance / menace.speed() : maxPredictionTime;
    const float predictionTime = (roughTime > maxPredictionTime) ? maxPredictionTime : roughTime;

    const Math::vec3 target = menace.predictFuturePosition(predictionTime);

    return flee(target);
}

bool SteerLibrary::inBoidNeighborhood(const Entity& other, float minDistance, float maxDistance,
                                      float cosMaxAngle) const
{
    if (&other == &vehicle())
        return false;

    const Math::vec3 offset = other.position() - vehicle().position();
    const float distanceSquared = Math::dot(offset, offset);

    if (distanceSquared < (minDistance * minDistance))
        return true;

    if (distanceSquared > (maxDistance * maxDistance))
        return false;

    const Math::vec3 unitOffset = offset / sqrtf(distanceSquared);
    const float forwardness = Math::dot(vehicle().forward(), unitOffset);
    return forwardness > cosMaxAngle;
}

Math::vec3 SteerLibrary::separation(float maxDistance, float cosMaxAngle,
                                   const ct::Vector<EntityDist>& flock) const
{
    Math::vec3 steering(0.0f);

    for (const EntityDist& member : flock)
    {
        const Entity& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            // Opposite of the offset direction, divided once to normalize and again for a 1/d
            // falloff.
            const Math::vec3 offset = other.position() - vehicle().position();
            const float distanceSquared = Math::dot(offset, offset);
            if (distanceSquared > 0.0f)
                steering += (offset / -distanceSquared);
        }
    }

    return safeNormalize(steering);
}

Math::vec3 SteerLibrary::alignment(float maxDistance, float cosMaxAngle,
                                  const ct::Vector<EntityDist>& flock) const
{
    Math::vec3 steering(0.0f);
    int neighbors = 0;

    for (const EntityDist& member : flock)
    {
        const Entity& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            steering += other.forward();
            ++neighbors;
        }
    }

    if (neighbors > 0)
        steering = safeNormalize((steering / static_cast<float>(neighbors)) - vehicle().forward());

    return steering;
}

Math::vec3 SteerLibrary::cohesion(float maxDistance, float cosMaxAngle,
                                 const ct::Vector<EntityDist>& flock) const
{
    Math::vec3 steering(0.0f);
    int neighbors = 0;

    for (const EntityDist& member : flock)
    {
        const Entity& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            steering += other.position();
            ++neighbors;
        }
    }

    if (neighbors > 0)
        steering = safeNormalize((steering / static_cast<float>(neighbors)) - vehicle().position());

    return steering;
}

Math::vec3 SteerLibrary::avoidObstacles(float minTimeToCollision,
                                       const ObstacleGroup& obstacles) const
{
    return Obstacle::steerToAvoidObstacles(vehicle(), minTimeToCollision, obstacles);
}

Math::vec3 SteerLibrary::avoidCloseNeighbors(float minSeparationDistance,
                                            const ct::Vector<EntityDist>& others) const
{
    for (const EntityDist& entry : others)
    {
        const Entity& other = *entry.entity;
        if (&other == &vehicle())
            continue;

        const float sumOfRadii = vehicle().radius() + other.radius();
        const float minCenterToCenter = minSeparationDistance + sumOfRadii;
        const Math::vec3 offset = other.position() - vehicle().position();
        const float currentDistance = Math::length(offset);

        if (currentDistance < minCenterToCenter)
        {
            // Remove the component along the heading; a head-on overlap makes that projection zero,
            // so fall back to the vehicle's side as a deterministic escape direction.
            const Math::vec3 lateral = perpendicularComponent(-offset, vehicle().forward());
            if (Math::dot(lateral, lateral) > 1e-8f)
                return safeNormalize(lateral);
            return vehicle().side();
        }
    }

    return Math::vec3(0.0f);
}

float SteerLibrary::predictNearestApproachTime(const Entity& other) const
{
    // Imagine we are at the origin with no velocity; compute the other entity's relative velocity.
    const Math::vec3 relVelocity = other.velocity() - vehicle().velocity();
    const float relSpeed = Math::length(relVelocity);

    // For parallel paths the distance never changes, so return 0 (now).
    if (relSpeed == 0.0f)
        return 0.0f;

    // In this relative space the other vehicle's path is a line; the distance from the origin (us)
    // to that line is the nearest approach.
    const Math::vec3 relTangent = relVelocity / relSpeed;
    const Math::vec3 relPosition = vehicle().position() - other.position();
    const float projection = Math::dot(relTangent, relPosition);

    return projection / relSpeed;
}

float SteerLibrary::computeNearestApproachPositions(const Entity& other, float time)
{
    const Math::vec3 myTravel = vehicle().velocity() * time;
    const Math::vec3 otherTravel = other.velocity() * time;

    const Math::vec3 myFinal = vehicle().position() + myTravel;
    const Math::vec3 otherFinal = other.position() + otherTravel;

    ourPositionAtNearestApproach = myFinal;
    hisPositionAtNearestApproach = otherFinal;

    return Math::length(myFinal - otherFinal);
}

Math::vec3 SteerLibrary::avoidNeighbors(float minTimeToCollision,
                                       const ct::Vector<EntityDist>& others)
{
    const Math::vec3 separation = avoidCloseNeighbors(0.0f, others);
    if (Math::length(separation) > 0.0f)
        return separation;

    float steer = 0.0f;
    const Entity* threat = nullptr;

    // Time until the most immediate collision threat found so far; starts as the look-ahead limit.
    float minTime = minTimeToCollision;

    Math::vec3 threatPositionAtNearestApproach(0.0f);
    Math::vec3 ourPositionAtNearestApproachTmp(0.0f);

    for (const EntityDist& entry : others)
    {
        const Entity& other = *entry.entity;
        if (&other == &vehicle())
            continue;

        const float collisionDangerThreshold = vehicle().radius() * 2.0f;

        const float time = predictNearestApproachTime(other);

        if ((time >= 0.0f) && (time < minTime))
        {
            if (computeNearestApproachPositions(other, time) < collisionDangerThreshold)
            {
                minTime = time;
                threat = &other;
                threatPositionAtNearestApproach = hisPositionAtNearestApproach;
                ourPositionAtNearestApproachTmp = ourPositionAtNearestApproach;
            }
        }
    }

    if (threat != nullptr)
    {
        // Parallel: +1, perpendicular: 0, anti-parallel: -1.
        const float parallelness = Math::dot(vehicle().forward(), threat->forward());
        const float angle = 0.707f;

        if (parallelness < -angle)
        {
            // Anti-parallel head-on paths: steer away from the future threat position.
            const Math::vec3 offset = threatPositionAtNearestApproach - vehicle().position();
            const float sideDot = Math::dot(offset, vehicle().side());
            steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
        }
        else if (parallelness > angle)
        {
            const Math::vec3 offset = threat->position() - vehicle().position();
            const float sideDot = Math::dot(offset, vehicle().side());
            steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
        }
        else
        {
            // Perpendicular paths: steer behind the threat (only the slower of the two does this).
            if (threat->speed() <= vehicle().speed())
            {
                const float sideDot = Math::dot(vehicle().side(), threat->velocity());
                steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
            }
        }
    }

    return vehicle().side() * steer;
}

Math::vec3 SteerLibrary::targetSpeed(float targetSpeed) const
{
    const float mf = vehicle().maxForce();
    const float speedError = targetSpeed - vehicle().speed();
    return vehicle().forward() * clip(speedError, -mf, +mf);
}

bool SteerLibrary::isAhead(const Math::vec3& target, float cosThreshold) const
{
    const Math::vec3 offset = target - vehicle().position();
    if (Math::dot(offset, offset) <= 1e-8f)
        return false;
    const Math::vec3 targetDirection = safeNormalize(offset);
    return Math::dot(vehicle().forward(), targetDirection) > cosThreshold;
}

bool SteerLibrary::isAside(const Math::vec3& target, float cosThreshold) const
{
    const Math::vec3 offset = target - vehicle().position();
    if (Math::dot(offset, offset) <= 1e-8f)
        return false;
    const Math::vec3 targetDirection = safeNormalize(offset);
    const float dp = Math::dot(vehicle().forward(), targetDirection);
    return (dp < cosThreshold) && (dp > -cosThreshold);
}

bool SteerLibrary::isBehind(const Math::vec3& target, float cosThreshold) const
{
    const Math::vec3 offset = target - vehicle().position();
    if (Math::dot(offset, offset) <= 1e-8f)
        return false;
    const Math::vec3 targetDirection = safeNormalize(offset);
    return Math::dot(vehicle().forward(), targetDirection) < cosThreshold;
}

void SeekBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;
    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() + (mSteer.seek(mTarget) * gain()));
}

void FleeBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;
    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() + (mSteer.flee(mThreat) * gain()));
}

void WanderBehavior::iterate(float timeDelta, Entity& entity)
{
    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() + (mSteer.wander(timeDelta) * gain()));
}

ObstacleAvoidanceBehavior::ObstacleAvoidanceBehavior(float minTimeToCollision,
                                                     const ObstacleGroup& obstacles)
    : mMinTimeToCollision(minTimeToCollision), mObstacles(&obstacles)
{
}

void ObstacleAvoidanceBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;
    if (!mObstacles)
        return;
    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() +
                          (mSteer.avoidObstacles(mMinTimeToCollision, *mObstacles) * gain()));
}

void SteerBehavior::iterate(float timeDelta, Entity& entity)
{
    mSteer.setVehicle(entity);
    if (mFunc)
        entity.setDesiredMove(entity.desiredMove() + (mFunc(mSteer, timeDelta) * gain()));
}

}
