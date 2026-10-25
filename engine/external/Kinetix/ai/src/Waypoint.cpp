#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "Waypoint.h"

#include "WaypointNetwork.h"

#include <cfloat>

namespace kinetix::AI
{

namespace
{
WaypointID nextWaypointId()
{
    static WaypointID counter = 1; // 0 is reserved as the null/invalid id
    return counter++;
}
}

Waypoint::Waypoint(const Math::vec3& position, const Math::quat& orientation, float radius)
    : mId(nextWaypointId()), mPosition(position), mOrientation(orientation), mRadius(radius)
{
}

bool Waypoint::addEdge(const NetworkEdge& edge)
{
    for (const NetworkEdge& e : mOutgoingEdges)
    {
        if (e.destination == edge.destination && e.costModifier == edge.costModifier &&
            e.open == edge.open)
            return false;
    }
    mOutgoingEdges.push_back(edge);
    return true;
}

bool Waypoint::removeEdge(const NetworkEdge& edge)
{
    for (auto it = mOutgoingEdges.begin(); it != mOutgoingEdges.end(); ++it)
    {
        const NetworkEdge& e = *it;
        if (e.destination == edge.destination && e.costModifier == edge.costModifier &&
            e.open == edge.open)
        {
            mOutgoingEdges.erase(it);
            return true;
        }
    }
    return false;
}

void Waypoint::clearEdges()
{
    mOutgoingEdges.clear();
}

float Waypoint::costForEdge(const NetworkEdge& edge, const WaypointNetwork& network) const
{
    const Waypoint* dest = network.findWaypoint(edge.destination);
    // WaypointNetwork::findPath() skips edges whose destination does not resolve, but other callers
    // can reach this: a missing destination must read as impassable, not free.
    if (!dest)
        return FLT_MAX;
    if (edge.costModifier != edge.costModifier)
        return FLT_MAX; // NaN: no meaningful cost, so no passage
    const float modifier = edge.costModifier > 0.0f ? edge.costModifier : 0.0f;
    return Math::length(mPosition - dest->position()) * modifier;
}

}
