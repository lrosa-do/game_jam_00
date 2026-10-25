#ifndef KINETIX_AI_GRIDMAP_H
#define KINETIX_AI_GRIDMAP_H

// Default cost is 1; any cost >= Blocked (3000) is impassable.

#include <ct/vector.hpp>

namespace kinetix::AI
{

class GridMap
{
public:
    explicit GridMap(int size);

    int size() const
    {
        return mSize;
    }

    int cost(int x, int y) const;
    // Negative costs are rejected (the cell keeps its cost): the searches
    // assume walking a cell never pays.
    void setCost(int x, int y, int cost);

    bool inBounds(int x, int y) const
    {
        return x >= 0 && y >= 0 && x < mSize && y < mSize;
    }
    bool isBlocked(int x, int y) const
    {
        return cost(x, y) >= Blocked;
    }
    void setBlocked(int x, int y)
    {
        setCost(x, y, Blocked);
    }

    static constexpr int DefaultCost = 1;
    static constexpr int Blocked = 3000;

private:
    int mSize;
    ct::Vector<int> mCosts; // row-major, index = y * mSize + x
};

}

#endif
