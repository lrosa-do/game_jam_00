#ifndef KINETIX_AI_PATHFINDBEHAVIOR_H
#define KINETIX_AI_PATHFINDBEHAVIOR_H

// Pops waypoints as they are reached, periodically tests line of sight to the goal
// (short-circuiting the path when visible), and nudges the agent with a perpendicular agitation
// vector if it stalls. Acts only on a SquadEntity.

#include "Behavior.h"
#include "WaypointNetwork.h"

#include <kinetix/core/Math.h>

namespace kinetix::AI
{

class WaypointNetwork;

class PathfindBehavior final : public Behavior
{
public:
    struct Settings
    {
        float turnRate = 0.2f;                 // desired-move strength toward the next node
        float goalRadius = 50.0f;
        float avoidDistance = 0.0f;            // <= 0 disables avoidance
        float maxTimeBeforeAgitation = 25.0f;  // seconds stuck before the agitation nudge
        float maxTimeBeforeLineOfSight = 0.5f; // seconds between goal line-of-sight tests
        Math::vec3 upVector = Math::vec3(0.0f, 1.0f, 0.0f);
        WaypointNetwork* waypointNetwork = nullptr; // non-owning
        // Line-of-sight functor used for the goal short-circuit. Supply one to
        // feed real LOS results, or nullptr for "always visible".
        const WaypointVisibility* visibility = nullptr;
    };

    explicit PathfindBehavior(const Settings& settings);

    void iterate(float timeDelta, Entity& entity) override;
    void applyAvoidance(Entity& entity);
    const char* name() const override
    {
        return "Pathfind Behavior";
    }

    Settings& settings()
    {
        return mSettings;
    }
    const Settings& settings() const
    {
        return mSettings;
    }

private:
    Settings mSettings;
};

}

#endif
