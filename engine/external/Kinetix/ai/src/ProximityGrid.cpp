#include <ProximityGrid.h>

#include <Entity.h>

#include <kinetix/core/Algorithm.h>

#include <math.h>

namespace kinetix::AI
{

namespace
{
// Power of two, so the bucket index is a mask instead of a modulo. Grown to
// stay comfortably above the number of occupied cells.
constexpr usize kMinBuckets = 64;

usize nextPowerOfTwo(usize n)
{
    usize result = kMinBuckets;
    while (result < n)
        result *= 2;
    return result;
}

// World coordinate to cell coordinate, clamped in float space so a position
// far outside any sane level cannot reach an out-of-range float-to-int cast.
// Callers drop non-finite input before this.
s32 cellCoord(f32 value, f32 invCellSize)
{
    const f32 limit = 1073741824.0f; // 2^30
    const f32 cell = floorf(value * invCellSize);
    return static_cast<s32>(cell < -limit ? -limit : (cell > limit ? limit : cell));
}

bool finite(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

u32 hashCell(s32 x, s32 z)
{
    // Two odd primes: neighbouring cells land far apart, which is what keeps
    // a row of agents from piling into one bucket chain.
    const u32 hx = static_cast<u32>(x) * 73856093u;
    const u32 hz = static_cast<u32>(z) * 19349663u;
    return hx ^ hz;
}
}

ProximityGrid::ProximityGrid(f32 cellSize)
{
    setCellSize(cellSize);
}

void ProximityGrid::setCellSize(f32 cellSize)
{
    mCellSize = cellSize > 1e-4f ? cellSize : 1e-4f;
    mInvCellSize = 1.0f / mCellSize;
}

void ProximityGrid::clear()
{
    mEntities.clear();
    mNext.clear();
    for (Bucket& b : mBuckets)
        b.first = -1;
}

s32 ProximityGrid::cellIndex(s32 cellX, s32 cellZ) const
{
    if (mBuckets.empty())
        return -1;

    usize slot = static_cast<usize>(hashCell(cellX, cellZ)) & mBucketMask;
    // Linear probing. The table is kept at least twice the occupied-cell
    // count, so a probe run stays short.
    for (usize probe = 0; probe <= mBucketMask; ++probe)
    {
        const Bucket& bucket = mBuckets[slot];
        if (bucket.first < 0)
            return -1;
        if (bucket.cellX == cellX && bucket.cellZ == cellZ)
            return static_cast<s32>(slot);
        slot = (slot + 1) & mBucketMask;
    }
    return -1;
}

void ProximityGrid::rebuild(const ct::Vector<Entity*>& entities)
{
    mEntities.clear();
    mNext.clear();

    // Two buckets per agent is the worst case (every agent in its own cell),
    // and that is the case a crowd spread over a level is closest to.
    const usize wanted = nextPowerOfTwo(entities.size() * 2 + 1);
    if (mBuckets.size() != wanted)
    {
        mBuckets.clear();
        mBuckets.resize(wanted);
        mBucketMask = wanted - 1;
    }
    for (Bucket& b : mBuckets)
        b.first = -1;

    if (entities.empty())
        return;

    mEntities.reserve(entities.size());
    mNext.reserve(entities.size());

    for (Entity* entity : entities)
    {
        if (!entity)
            continue;

        const Math::vec3 position = entity->position();
        if (!finite(position))
            continue; // nowhere, so near nothing
        const s32 cellX = cellCoord(position.x, mInvCellSize);
        const s32 cellZ = cellCoord(position.z, mInvCellSize);

        usize slot = static_cast<usize>(hashCell(cellX, cellZ)) & mBucketMask;
        while (mBuckets[slot].first >= 0 &&
               (mBuckets[slot].cellX != cellX || mBuckets[slot].cellZ != cellZ))
            slot = (slot + 1) & mBucketMask;

        const s32 index = static_cast<s32>(mEntities.size());
        mEntities.push_back(entity);

        Bucket& bucket = mBuckets[slot];
        if (bucket.first < 0)
        {
            bucket.cellX = cellX;
            bucket.cellZ = cellZ;
            mNext.push_back(-1);
        }
        else
        {
            mNext.push_back(bucket.first);
        }
        bucket.first = index;
    }
}

void ProximityGrid::query(const Math::vec3& center, f32 radius, ct::Vector<Entity*>& out) const
{
    out.clear();
    if (mEntities.empty() || !(radius > 0.0f) || !isfinite(radius) || !finite(center))
        return;

    const s32 minX = cellCoord(center.x - radius, mInvCellSize);
    const s32 maxX = cellCoord(center.x + radius, mInvCellSize);
    const s32 minZ = cellCoord(center.z - radius, mInvCellSize);
    const s32 maxZ = cellCoord(center.z + radius, mInvCellSize);

    const f32 radiusSq = radius * radius;

    for (s32 cellZ = minZ; cellZ <= maxZ; ++cellZ)
    {
        for (s32 cellX = minX; cellX <= maxX; ++cellX)
        {
            const s32 slot = cellIndex(cellX, cellZ);
            if (slot < 0)
                continue;

            for (s32 i = mBuckets[static_cast<usize>(slot)].first; i >= 0;
                 i = mNext[static_cast<usize>(i)])
            {
                Entity* entity = mEntities[static_cast<usize>(i)];
                const Math::vec3 delta = entity->position() - center;
                // XZ only: agents on a walkable surface are separated by where
                // they stand, not by how high the floor is under them.
                if (delta.x * delta.x + delta.z * delta.z <= radiusSq)
                    out.push_back(entity);
            }
        }
    }
}

}
