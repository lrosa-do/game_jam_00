#ifndef KINETIX_PHYSICS_COLLISION_FILTER_H
#define KINETIX_PHYSICS_COLLISION_FILTER_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Category bits and acceptance bits are separate: a pair is accepted only
// when both objects opt in.
struct CollisionFilter
{
    u32 group = 1;
    u32 mask = 0xFFFFFFFFu;
};

inline bool shouldCollide(const CollisionFilter& a, const CollisionFilter& b)
{
    return (a.mask & b.group) != 0 && (b.mask & a.group) != 0;
}

struct QueryFilter
{
    CollisionFilter collision{0xFFFFFFFFu, 0xFFFFFFFFu};
    u32 ignoredBody = 0xFFFFFFFFu;

    bool accepts(u32 bodyId, const CollisionFilter& bodyFilter) const
    {
        return bodyId != ignoredBody && shouldCollide(collision, bodyFilter);
    }
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_COLLISION_FILTER_H
