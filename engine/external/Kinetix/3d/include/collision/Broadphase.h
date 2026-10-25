#ifndef KINETIX_PHYSICS_COLLISION_BROADPHASE_H
#define KINETIX_PHYSICS_COLLISION_BROADPHASE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include "collision/CollisionFilter.h"

#include <ct/vector.hpp>

namespace kinetix::Physics
{

struct BroadphaseProxy
{
    AABB bounds;
    u32 id = 0;
    CollisionFilter filter;
    // Static against static: neither can move, so the answer cannot change.
    bool movable = true;
};

struct BroadphasePair
{
    u32 a = 0;
    u32 b = 0;
};

// Sweep and prune on one axis. Proxies are sorted by their lower bound and
// each is only tested against those that start before its own upper bound;
// degrades to quadratic when all bodies are stacked in one place.
class Broadphase
{
public:
    void clear();
    void add(const BroadphaseProxy& proxy);
    void reserve(usize count);

    // Overwrites `out` with every pair whose bounds overlap and whose layers
    // accept each other. Pairs come back with a < b.
    void findPairs(ct::Vector<BroadphasePair>& out);

    usize proxyCount() const
    {
        return mProxies.size();
    }

    // The axis the sweep runs along, the one the bodies are most spread over;
    // recomputed by findPairs().
    u32 sweepAxis() const
    {
        return mSweepAxis;
    }

    static bool overlaps(const AABB& a, const AABB& b);

private:
    ct::Vector<BroadphaseProxy> mProxies;
    ct::Vector<u32> mOrder;
    u32 mSweepAxis = 0;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_COLLISION_BROADPHASE_H
