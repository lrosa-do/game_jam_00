#ifndef KINETIX_AI_WAYPOINTNETWORK_H
#define KINETIX_AI_WAYPOINTNETWORK_H

// A path is a deque of WaypointIDs (O(1) pop_front).

#include "Waypoint.h"

#include <ct/deque.hpp>
#include <kinetix/core/Math.h>
#include <ct/hashmap.hpp>

namespace kinetix::AI
{

using Path = ct::Deque<WaypointID>;

// Decides whether a waypoint is reachable/visible from a position; the base accepts everything.
// Override isVisible() to feed line-of-sight results from your own world.
class WaypointVisibility
{
public:
    virtual ~WaypointVisibility() = default;
    virtual bool isVisible(const Math::vec3& origin, const Math::vec3& destination) const
    {
        (void)origin;
        (void)destination;
        return true;
    }
};

class WaypointNetwork
{
public:
    using WaypointMap = ct::HashMap<WaypointID, Waypoint*>;

    WaypointNetwork() = default;
    ~WaypointNetwork();

    // Ownership: the network takes ownership of waypoint and deletes it on
    // remove/clear/destruction.
    bool addWaypoint(Waypoint* waypoint);
    bool removeWaypoint(WaypointID waypointID);
    void clearWaypoints();
    Waypoint* findWaypoint(WaypointID waypointID) const;

    // Returns true and fills outPath (start..goal) if a route exists. The
    // heuristic is the straight-line distance scaled by the smallest cost
    // modifier of any open edge, so it never overestimates and the route is
    // the cheapest one even with modifiers below 1. Negative modifiers count
    // as 0 (a free edge, never a profit) and NaN ones as impassable.
    bool findPath(WaypointID fromWaypoint, WaypointID toWaypoint, Path& outPath) const;

    // A* between arbitrary positions: snaps both endpoints to the closest
    // visible waypoint, then searches. If both snap to the same waypoint the
    // path is left empty and true is returned (walk straight to the goal).
    bool findPath(const Math::vec3& origin, const Math::vec3& destination,
                  const WaypointVisibility& visibility, Path& outPath) const;

    // False, with minimum/maximum left untouched, when the network is empty.
    bool extents(Math::vec3& minimum, Math::vec3& maximum) const;

    const WaypointMap& waypoints() const
    {
        return mWaypoints;
    }
    WaypointMap& waypoints()
    {
        return mWaypoints;
    }

private:
    bool findClosestValidWaypoint(const Math::vec3& origin, const WaypointVisibility& visibility,
                                  WaypointID& outWaypointID) const;
    float goalEstimate(WaypointID fromWaypoint, WaypointID toWaypoint) const;
    // Smallest per-distance cost any open edge can charge - what the
    // straight-line heuristic must be scaled by to stay admissible.
    float minimumCostModifier() const;

    WaypointMap mWaypoints;
};

}

#endif
