#ifndef KINETIX_AI_GRIDPATHFINDER_H
#define KINETIX_AI_GRIDPATHFINDER_H

// 8-directional neighbours; diagonals cost the same as orthogonal moves.

#include "GridMap.h"

#include <ct/vector.hpp>

namespace kinetix::AI
{

// With diagonals costing as much as orthogonal moves, the true remaining cost is max(|dx|, |dy|),
// so MaxDxDy (default) is the only heuristic that never overestimates and keeps A* optimal.
// Manhattan and Euclidean overestimate, trading optimality for fewer expansions.
enum class GridHeuristic
{
    Manhattan, // |dx| + |dy|
    Euclidean, // floor(sqrt(dx^2 + dy^2))
    MaxDxDy    // max(|dx|, |dy|)
};

enum class GridSearchAlgorithm
{
    AStar,       // f = g + h (weighted, uses the heuristic)
    Dijkstra,    // f = g (weighted, ignores the heuristic)
    BestFirst,   // f = h (greedy best-first, uses the heuristic)
    BreadthFirst // unweighted FIFO
};

struct GridCellCoord
{
    int x = 0;
    int y = 0;
};

struct GridPathSettings
{
    GridSearchAlgorithm algorithm = GridSearchAlgorithm::AStar;
    GridHeuristic heuristic = GridHeuristic::MaxDxDy;
    float heuristicWeight = 1.0f;
};

class GridPathfinder
{
public:
    explicit GridPathfinder(const GridMap* grid) : mGrid(grid)
    {
    }

    // On success fills outPath with the cells from start to end inclusive and returns true; false
    // if the goal is unreachable.
    bool findPath(int startX, int startY, int endX, int endY,
                  ct::Vector<GridCellCoord>& outPath) const;

    GridPathSettings& settings()
    {
        return mSettings;
    }
    const GridPathSettings& settings() const
    {
        return mSettings;
    }

private:
    int goalEstimate(int x, int y, int endX, int endY) const;

    bool searchInformed(int startX, int startY, int endX, int endY, bool useHeuristic,
                        ct::Vector<GridCellCoord>& outPath) const;
    bool searchBestFirst(int startX, int startY, int endX, int endY,
                         ct::Vector<GridCellCoord>& outPath) const;
    bool searchBreadthFirst(int startX, int startY, int endX, int endY,
                            ct::Vector<GridCellCoord>& outPath) const;

    const GridMap* mGrid; // non-owning
    GridPathSettings mSettings;
};

}

#endif
