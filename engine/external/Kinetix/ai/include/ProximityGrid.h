#ifndef KINETIX_AI_PROXIMITYGRID_H
#define KINETIX_AI_PROXIMITYGRID_H

// Uniform XZ grid so neighbour queries visit only nearby cells instead of testing every agent pair
// (O(n^2)). Cells hold indices into one flat array, so a rebuild is two linear passes and
// allocation-free once sized.

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::AI
{

class Entity;

class ProximityGrid
{
public:
    // Should be about the largest query radius in use; a hint only, any radius still returns the
    // right answer.
    explicit ProximityGrid(f32 cellSize = 4.0f);

    void setCellSize(f32 cellSize);
    f32 cellSize() const { return mCellSize; }

    // Discards the previous contents and files every entity again. Call once per frame, before any
    // query.
    void rebuild(const ct::Vector<Entity*>& entities);

    void clear();

    // Entities within `radius` of `center`, in XZ. The vector is cleared first.
    void query(const Math::vec3& center, f32 radius, ct::Vector<Entity*>& out) const;

    usize size() const { return mEntities.size(); }

private:
    struct Bucket
    {
        s32 cellX = 0;
        s32 cellZ = 0;
        s32 first = -1; // index into mEntities/mNext, -1 when empty
    };

    s32 cellIndex(s32 cellX, s32 cellZ) const;

    f32 mCellSize;
    f32 mInvCellSize;

    // Open-addressed table of occupied cells, so an agent far from the rest
    // costs one bucket instead of a row of empty ones.
    ct::Vector<Bucket> mBuckets;
    usize mBucketMask = 0;

    ct::Vector<Entity*> mEntities;
    ct::Vector<s32> mNext;
};

}

#endif
