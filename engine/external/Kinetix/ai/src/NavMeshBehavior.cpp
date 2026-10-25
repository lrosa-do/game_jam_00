#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "NavMeshBehavior.h"

#include "AIInternal.h"
#include "AgentAvoidance.h"
#include "Group.h"
#include "NavMesh.h"
#include "SquadEntity.h"
#include "World.h"

#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

using detail::safeNormalize;

NavMeshBehavior::NavMeshBehavior(const NavMesh& navMesh, const Settings& settings)
    : mNavMesh(navMesh), mSettings(settings)
{
}

void NavMeshBehavior::iterate(float timeDelta, Entity& entity)
{
    SquadEntity* squadmate = SquadEntity::from(entity);
    if (!squadmate || !mNavMesh.valid())
        return;

    Route& route = mRoutes[&entity];
    route.sinceRepath += timeDelta;

    // Entity::iterate() advances the position by the velocity before any behavior runs, so it is a
    // freely integrated guess that may have crossed a wall. Slide it back onto the walkable
    // surface: a route only suggests a direction, so corner cutting or avoidance pushes would
    // otherwise leave the floor.
    constrainToSurface(entity, route);

    const Math::vec3 position = entity.position();
    const Math::vec3 goal = squadmate->goal();

    Math::vec3 flatToGoal = goal - position;
    flatToGoal.y = 0.0f;
    // Hysteresis: once inside goalRadius an agent stays "arrived" until pushed well back out; with
    // a single threshold avoidance shoves a crowd past the edge and they jitter around the rim
    // forever.
    const float distanceToGoal = Math::length(flatToGoal);
    const float releaseRadius = mSettings.goalRadius * 2.0f;
    if (route.arrived ? distanceToGoal < releaseRadius : distanceToGoal < mSettings.goalRadius)
    {
        route.arrived = true;

        // Brake only as hard as the remaining speed; asking for -velocity every frame while
        // avoidance pushes back keeps agents moving.
        const Math::vec3 velocity = entity.velocity();
        const float speed = Math::length(velocity);
        if (speed > 0.05f)
            entity.setDesiredMove(-velocity);

        // Avoidance still runs so a pile on the goal spreads out; the release radius stops it
        // re-triggering the chase.
        applyAvoidance(entity);
        return;
    }
    route.arrived = false;

    // Ramp turnRate down between arrivalRadius and goalRadius so a crowd converging on one goal
    // does not hit the hard brake all at once and get shoved back out by avoidance.
    float arrivalScale = 1.0f;
    if (mSettings.arrivalRadius > mSettings.goalRadius)
    {
        const float span = mSettings.arrivalRadius - mSettings.goalRadius;
        arrivalScale = (distanceToGoal - mSettings.goalRadius) / span;
        arrivalScale = arrivalScale < 0.0f ? 0.0f : (arrivalScale > 1.0f ? 1.0f : arrivalScale);
    }

    // Two gates: the interval limits the rate, and the goal having moved is what says there is
    // anything new to find.
    const bool goalMoved =
        !route.hasRoute ||
        Math::length(goal - route.goalWhenFound) > mSettings.goalMoveThreshold;
    const bool outOfCorners = route.next >= route.corners.size();
    const bool wantsRoute =
        (route.sinceRepath >= mSettings.repathInterval && goalMoved) || outOfCorners;

    if (wantsRoute && !route.searching)
    {
        route.sinceRepath = 0.0f;
        route.query.setMaxNodes(mSettings.searchMaxNodes);
        route.goalWhenSearchStarted = goal;
        route.searching = route.query.start(mNavMesh, position, goal, mSettings.searchExtents);
        if (!route.searching)
            clearRoute(route);
    }

    if (route.searching)
    {
        // Spread over frames so a crowd starting together does not spike; the old corners stay in
        // use meanwhile.
        const PathQueryState state =
            mSettings.searchNodesPerFrame == 0
                ? runToCompletion(route.query)
                : route.query.step(mSettings.searchNodesPerFrame);

        if (state != PathQueryState::Searching)
        {
            route.searching = false;
            ct::Vector<Math::vec3> fresh;
            if (state == PathQueryState::Ready && route.query.result(fresh) && fresh.size() > 1)
            {
                route.corners = kinetix::Move(fresh);
                route.next = 1; // [0] is where the agent already stands
                route.goalWhenFound = route.goalWhenSearchStarted;
                route.hasRoute = true;
            }
            else if (route.next >= route.corners.size())
            {
                // Off the mesh, or nothing reachable, and the old route is spent: head straight at
                // the goal rather than freezing. A route with corners left is kept.
                clearRoute(route);
            }
        }
    }

    Math::vec3 towards = flatToGoal;
    if (route.next < route.corners.size())
    {
        Math::vec3 toCorner = route.corners[route.next] - position;
        toCorner.y = 0.0f;
        if (Math::length(toCorner) < mSettings.cornerRadius)
        {
            ++route.next;
            if (route.next < route.corners.size())
            {
                toCorner = route.corners[route.next] - position;
                toCorner.y = 0.0f;
            }
        }
        if (route.next < route.corners.size())
            towards = toCorner;
    }

    Math::vec3 desired = entity.desiredMove();
    desired += safeNormalize(towards) * (mSettings.turnRate * arrivalScale) * gain();
    entity.setDesiredMove(desired);

    applyAvoidance(entity);
}

void NavMeshBehavior::onDetach(Entity& entity)
{
    mRoutes.erase(&entity);
}

void NavMeshBehavior::clearRoute(Route& route)
{
    route.corners.clear();
    route.next = 0;
    route.hasRoute = false;
}

PathQueryState NavMeshBehavior::runToCompletion(NavMeshQuery& query)
{
    // Big enough that a normal search ends in one call; the query's own
    // maxNodes is what actually bounds it.
    constexpr u32 kUnbudgetedStep = 1u << 20;

    PathQueryState state = query.state();
    while (state == PathQueryState::Searching)
        state = query.step(kUnbudgetedStep);
    return state;
}

void NavMeshBehavior::constrainToSurface(Entity& entity, Route& route)
{
    const Math::vec3 wanted = entity.position();

    // First pass for this agent: it has no known-good position behind it, so
    // snap onto the surface instead of sliding across it.
    if (!route.onSurface)
    {
        Math::vec3 snapped;
        if (!mNavMesh.nearestPoint(wanted, snapped, mSettings.searchExtents))
            return;
        route.surfacePosition = snapped;
        route.onSurface = true;
        route.surfaceTri = -1;
        entity.setPosition(Math::vec3(snapped.x, wanted.y, snapped.z));
        return;
    }

    Math::vec3 slid;
    if (!mNavMesh.moveAlongSurface(route.surfacePosition, wanted, slid, route.surfaceTri,
                                   mSettings.searchExtents))
    {
        // The last good position stopped being on the mesh - the surface was
        // rebuilt, or the agent was teleported. Re-acquire next pass.
        route.onSurface = false;
        route.surfaceTri = -1;
        return;
    }

    route.surfacePosition = slid;

    // Height stays the caller's business: overwriting y here would fight its own ground offset.
    entity.setPosition(Math::vec3(slid.x, wanted.y, slid.z));
}

void NavMeshBehavior::applyAvoidance(Entity& entity)
{
    detail::applyAgentAvoidance(entity, mSettings.avoidDistance, mSettings.turnRate);
}

}
