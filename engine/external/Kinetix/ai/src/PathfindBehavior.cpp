#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "PathfindBehavior.h"

#include "AgentAvoidance.h"
#include "Entity.h"
#include "Group.h"
#include "SquadEntity.h"
#include "World.h"

#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

namespace
{
// normalize * turnRate without producing NaNs on a zero vector.
Math::vec3 normalizedScaled(const Math::vec3& v, float scale)
{
    float len = Math::length(v);
    if (len <= 0.0f)
        return Math::vec3(0.0f);
    return v * (scale / len);
}
}

PathfindBehavior::PathfindBehavior(const Settings& settings) : mSettings(settings)
{
}

void PathfindBehavior::iterate(float timeDelta, Entity& entity)
{
    SquadEntity* squadmatePtr = SquadEntity::from(entity);
    WaypointNetwork* network = mSettings.waypointNetwork;
    if (!squadmatePtr || !network)
        return;
    SquadEntity& squadmate = *squadmatePtr;

    Path& path = squadmate.path();
    WaypointID wpID = squadmate.nextWaypoint();
    Math::vec3 entityPos = entity.position();
    Math::vec3 desiredMoveAdj(0.0f);

    WaypointVisibility defaultVisibility;
    const WaypointVisibility& visibility =
        mSettings.visibility ? *mSettings.visibility : defaultVisibility;

    squadmate.incrementTimeSinceLOSTest(timeDelta);
    if (squadmate.timeSinceLOSTest() > mSettings.maxTimeBeforeLineOfSight)
    {
        squadmate.setLOSStatus(visibility.isVisible(squadmate.position(), squadmate.goal()));
        if (squadmate.losStatus())
        {
            path.clear();
            squadmate.setNextWaypoint(0);
        }
        squadmate.resetTimeSinceLOSTest();
    }

    if (wpID == 0 && !path.empty())
    {
        wpID = path.front();
        path.pop_front();
        squadmate.setNextWaypoint(wpID);
        squadmate.resetTimeSinceWaypointReached();
    }
    else if (wpID == 0 || path.empty())
    {
        desiredMoveAdj = squadmate.goal() - entityPos;
        desiredMoveAdj.y = 0.0f;
        if (Math::length(desiredMoveAdj) < squadmate.goalRadius())
        {
            entity.setDesiredMove(-entity.velocity());
            applyAvoidance(entity);
            squadmate.resetTimeSinceWaypointReached();
            return;
        }
        else if (!squadmate.losStatus())
        {
            network->findPath(entityPos, squadmate.goal(), visibility, path);
            squadmate.setPath(path);
            wpID = squadmate.nextWaypoint();
        }
    }

    Waypoint* wp = network->findWaypoint(wpID);
    if (wp)
    {
        Math::vec3 wppos = wp->position();
        desiredMoveAdj = wppos - entityPos;
        desiredMoveAdj.y = 0.0f;
        if (Math::length(desiredMoveAdj) < wp->radius())
        {
            squadmate.setCurrentWaypoint(wp->id());
            if (!path.empty())
            {
                wpID = path.front();
                path.pop_front();
                squadmate.setNextWaypoint(wpID);
                wp = network->findWaypoint(wpID);
                if (!wp) // the reference asserted here; guard instead
                    return;
                wppos = wp->position();
                desiredMoveAdj = wppos - entityPos;
                squadmate.resetTimeSinceWaypointReached();
            }
            else
            {
                desiredMoveAdj = squadmate.goal() - entityPos;
                squadmate.setNextWaypoint(0);
                if (Math::length(desiredMoveAdj) < mSettings.goalRadius)
                {
                    entity.setDesiredMove(-entity.velocity());
                    applyAvoidance(entity);
                    squadmate.resetTimeSinceWaypointReached();
                    return;
                }
            }
        }
    }

    squadmate.incrementTimeSinceWaypointReached(timeDelta);
    Math::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += normalizedScaled(desiredMoveAdj, mSettings.turnRate) * gain();

    if (squadmate.timeSinceWaypointReached() > mSettings.maxTimeBeforeAgitation)
    {
        currentDesiredMove = Math::cross(currentDesiredMove, mSettings.upVector);
        squadmate.resetTimeSinceWaypointReached();
        squadmate.resetTimeSinceGoalReached();
    }

    entity.setDesiredMove(currentDesiredMove);

    applyAvoidance(entity);
}

void PathfindBehavior::applyAvoidance(Entity& entity)
{
    detail::applyAgentAvoidance(entity, mSettings.avoidDistance, mSettings.turnRate);
}

}
