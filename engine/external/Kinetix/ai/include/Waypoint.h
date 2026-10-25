#ifndef KINETIX_AI_WAYPOINT_H
#define KINETIX_AI_WAYPOINT_H

// WaypointID is a u32 auto-incremented from 1 (0 = invalid). Waypoints are owned by the
// WaypointNetwork they are added to.

#include <kinetix/core/Types.h>

#include <kinetix/core/Math.h>
#include <ct/vector.hpp>

namespace kinetix::AI
{

using WaypointID = kinetix::u32;

// An edge from one waypoint to another. costModifier scales the distance-based
// edge cost; open == false makes the edge impassable to the A* search. A
// negative modifier is treated as 0 and a NaN one as impassable: a negative
// edge cost would let the search loop forever.
struct NetworkEdge
{
    WaypointID destination = 0;
    float costModifier = 1.0f;
    bool open = true;
};

class WaypointNetwork;

class Waypoint
{
public:
    explicit Waypoint(const Math::vec3& position = Math::vec3(0.0f),
                      const Math::quat& orientation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f),
                      float radius = 1.0f);

    WaypointID id() const
    {
        return mId;
    }

    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    const Math::quat& orientation() const
    {
        return mOrientation;
    }
    void setOrientation(const Math::quat& orientation)
    {
        mOrientation = orientation;
    }
    float radius() const
    {
        return mRadius;
    }
    void setRadius(float radius)
    {
        mRadius = radius;
    }

    // addEdge() returns false if an equal edge exists.
    bool addEdge(const NetworkEdge& edge);
    bool removeEdge(const NetworkEdge& edge);
    void clearEdges();

    // Cost of travelling along this edge: distance to the destination
    // waypoint scaled by the edge's cost modifier.
    float costForEdge(const NetworkEdge& edge, const WaypointNetwork& network) const;

    ct::Vector<NetworkEdge>& edges()
    {
        return mOutgoingEdges;
    }
    const ct::Vector<NetworkEdge>& edges() const
    {
        return mOutgoingEdges;
    }

private:
    WaypointID mId;
    ct::Vector<NetworkEdge> mOutgoingEdges;
    Math::vec3 mPosition;
    Math::quat mOrientation;
    float mRadius;
};

}

#endif
