#include "check.h"

#include <GridMap.h>
#include <GridPathfinder.h>
#include <Waypoint.h>
#include <WaypointNetwork.h>

using namespace kinetix::AI;

namespace
{

Waypoint* addWaypoint(WaypointNetwork& network, float x, float z)
{
    Waypoint* waypoint = new Waypoint(kinetix::Math::vec3(x, 0.0f, z));
    network.addWaypoint(waypoint);
    return waypoint;
}

void link(Waypoint* from, Waypoint* to, float modifier)
{
    NetworkEdge edge;
    edge.destination = to->id();
    edge.costModifier = modifier;
    from->addEdge(edge);
}

}

int main()
{
    // A cheap detour away from the goal (modifier 0.01) beats the direct
    // edge. A distance heuristic that ignores the modifiers overestimates
    // and pops the goal through the direct edge first.
    {
        WaypointNetwork network;
        Waypoint* a = addWaypoint(network, 0.0f, 0.0f);
        Waypoint* b = addWaypoint(network, 10.0f, 0.0f);
        Waypoint* c = addWaypoint(network, -20.0f, 0.0f);
        link(a, b, 1.0f);
        link(a, c, 0.01f);
        link(c, b, 0.01f);

        Path path;
        CHECK(network.findPath(a->id(), b->id(), path));
        CHECK(path.size() == 3);
        if (path.size() == 3)
        {
            CHECK(path[0] == a->id());
            CHECK(path[1] == c->id());
            CHECK(path[2] == b->id());
        }
    }

    {
        WaypointNetwork network;
        Waypoint* a = addWaypoint(network, 0.0f, 0.0f);
        Waypoint* b = addWaypoint(network, 10.0f, 0.0f);
        Waypoint* c = addWaypoint(network, 0.0f, 5.0f);
        link(a, c, -1.0f);
        link(c, a, -1.0f);
        link(a, b, 1.0f);

        Path path;
        CHECK(network.findPath(a->id(), b->id(), path));
        CHECK(!path.empty() && path[path.size() - 1] == b->id());
        CHECK(a->costForEdge(a->edges()[0], network) >= 0.0f);
    }

    {
        GridMap grid(8);
        grid.setCost(2, 2, -5);
        CHECK(grid.cost(2, 2) == GridMap::DefaultCost);
        grid.setCost(2, 2, 7);
        CHECK(grid.cost(2, 2) == 7);

        for (int x = 0; x < 8; ++x)
            grid.setCost(x, 4, -100);
        GridPathfinder pathfinder(&grid);
        ct::Vector<GridCellCoord> path;
        CHECK(pathfinder.findPath(0, 0, 7, 7, path));
    }

    return TEST_RESULT();
}
