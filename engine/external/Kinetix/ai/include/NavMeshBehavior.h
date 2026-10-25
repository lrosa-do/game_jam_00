#ifndef KINETIX_AI_NAVMESHBEHAVIOR_H
#define KINETIX_AI_NAVMESHBEHAVIOR_H

// Counterpart to PathfindBehavior: routes over a NavMesh instead of a WaypointNetwork. Only steers;
// agent avoidance is applied here too.

#include "Behavior.h"
#include "NavMeshQuery.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

namespace kinetix::AI
{

class NavMesh;

class NavMeshBehavior final : public Behavior
{
public:
    struct Settings
    {
        float turnRate = 0.35f;    // desired-move strength toward the next corner
        float goalRadius = 1.0f;
        float cornerRadius = 0.6f;
        float avoidDistance = 0.0f; // <= 0 disables agent avoidance
        // Distance at which the agent starts slowing for the goal, so a crowd arriving together
        // does not hit the goal at full speed. <= goalRadius disables the ramp (seek, then hard
        // brake).
        float arrivalRadius = 0.0f;
        // Seconds between route queries (findPath() is a real A* over the surface).
        float repathInterval = 0.35f;
        // A route is only recomputed once the goal has moved this far from where it was when the
        // current one was found (or the agent ran out of corners).
        float goalMoveThreshold = 1.0f;
        // How far off the mesh a point may sit and still snap onto it.
        Math::vec3 searchExtents = Math::vec3(2.0f, 6.0f, 2.0f);
        // Nodes each searching agent may expand per frame; routes are found over several frames
        // while the agent walks its previous route. 0 searches to completion in one frame.
        u32 searchNodesPerFrame = 64;
        // Nodes one route may cost in total before the search is abandoned.
        u32 searchMaxNodes = 10000;
    };

    NavMeshBehavior(const NavMesh& navMesh, const Settings& settings);

    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "NavMesh Behavior";
    }

    Settings& settings()
    {
        return mSettings;
    }
    const Settings& settings() const
    {
        return mSettings;
    }

    usize routeCount() const
    {
        return mRoutes.size();
    }

protected:
    // Drops the agent's route, so a dead agent's state neither leaks nor is
    // inherited by a new one that happens to reuse its address.
    void onDetach(Entity& entity) override;

private:
    // The behavior is shared, so nothing about one agent's path can live in its fields.
    struct Route
    {
        ct::Vector<Math::vec3> corners;
        usize next = 0;
        float sinceRepath = 0.0f;
        Math::vec3 goalWhenFound = Math::vec3(0.0f);
        bool hasRoute = false;
        // Last position known to be on the walkable surface; moves are slid from it.
        Math::vec3 surfacePosition = Math::vec3(0.0f);
        bool onSurface = false;
        // Triangle the agent was last on, fed back as the hint to avoid a whole-mesh search per
        // frame.
        s32 surfaceTri = -1;
        // The search in flight, if any; the corners above stay in use until it produces better
        // ones.
        NavMeshQuery query;
        Math::vec3 goalWhenSearchStarted = Math::vec3(0.0f);
        bool searching = false;
        // Sticky once inside goalRadius; see the release radius in iterate().
        bool arrived = false;
    };

    void constrainToSurface(Entity& entity, Route& route);
    void applyAvoidance(Entity& entity);
    static void clearRoute(Route& route);
    static PathQueryState runToCompletion(NavMeshQuery& query);

    const NavMesh& mNavMesh;
    Settings mSettings;
    ct::HashMap<const Entity*, Route> mRoutes;
};

}

#endif
