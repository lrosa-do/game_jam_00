#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "GridMap.h"

#include <stddef.h>

namespace kinetix::AI
{

GridMap::GridMap(int size) : mSize(size), mCosts(static_cast<size_t>(size) * size, DefaultCost)
{
}

int GridMap::cost(int x, int y) const
{
    if (!inBounds(x, y))
        return Blocked;
    return mCosts[static_cast<size_t>(y) * mSize + x];
}

void GridMap::setCost(int x, int y, int cost)
{
    if (!inBounds(x, y))
        return;
    // A negative cost would let a search profit from walking back and forth and never finish.
    if (cost < 0)
    {
        Log::warning("GridMap: setCost() called with a negative cost; ignored");
        return;
    }
    mCosts[static_cast<size_t>(y) * mSize + x] = cost;
}

}
