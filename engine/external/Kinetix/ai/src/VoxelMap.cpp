#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "VoxelMap.h"

#include <math.h>

#include <ct/vector.hpp>
#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

namespace
{

u32 flatten3D(const Math::uvec3& coord, const Math::uvec3& dim)
{
    return coord.z * dim.x * dim.y + coord.y * dim.x + coord.x;
}

void voxelAddress(const Math::uvec3& coord, const Math::uvec3& resolutionDiv4,
                  u32& outIndex, u64& outMask)
{
    const Math::uvec3 macroCoord = coord / 4u;
    const Math::uvec3 subCoord = coord % 4u;
    outIndex = flatten3D(macroCoord, resolutionDiv4);
    const u32 bit = flatten3D(subCoord, Math::uvec3(4, 4, 4));
    outMask = 1ull << bit;
}

Math::vec3 worldToUvw(const Math::vec3& worldPos, const Math::vec3& center,
                     const Math::vec3& resolutionRcp, const Math::vec3& voxelSizeRcp)
{
    const Math::vec3 diff = (worldPos - center) * resolutionRcp * voxelSizeRcp;
    return diff * Math::vec3(0.5f, -0.5f, 0.5f) + Math::vec3(0.5f);
}

Math::vec3 uvwToWorld(const Math::vec3& uvw, const Math::vec3& center,
                     const Math::vec3& resolution, const Math::vec3& voxelSize)
{
    Math::vec3 pos = uvw * 2.0f - Math::vec3(1.0f);
    pos *= Math::vec3(1.0f, -1.0f, 1.0f);
    pos *= voxelSize;
    pos *= resolution;
    pos += center;
    return pos;
}

// Floors rather than truncates, so the slab just before the grid maps to -1 and not 0, and clamps
// in float space first so a far-away or NaN position never reaches an out-of-range float-to-int
// cast.
Math::ivec3 pixelToCoord(const Math::vec3& pixel)
{
    const f32 limit = 1073741824.0f; // 2^30, well inside s32
    Math::ivec3 coord;
    for (int axis = 0; axis < 3; ++axis)
    {
        const f32 value = floorf(pixel[axis]);
        if (!(value > -limit))
            coord[axis] = value != value ? -1 : -static_cast<s32>(limit);
        else if (value >= limit)
            coord[axis] = static_cast<s32>(limit);
        else
            coord[axis] = static_cast<s32>(value);
    }
    return coord;
}

// Two pixel-space corners (order not guaranteed: the coordinate flip in worldToUvw can swap min and
// max) turned into an inclusive-exclusive voxel range, clamped per axis in float space before any
// unsigned cast: a shape wholly outside the grid (or a NaN corner) yields no range, so the caller
// skips the loop instead of walking a wrapped-around one.
bool clampVoxelRange(const Math::vec3& p0, const Math::vec3& p1, const Math::vec3& resolution,
                     Math::uvec3& outMin, Math::uvec3& outMax)
{
    outMin = Math::uvec3(0u);
    outMax = Math::uvec3(0u);

    const Math::vec3 rangeMin = Math::floor(Math::min(p0, p1));
    const Math::vec3 rangeMax = Math::ceil(Math::max(p0, p1) + Math::vec3(0.0001f));

    Math::uvec3 mini(0u), maxi(0u);
    for (int axis = 0; axis < 3; ++axis)
    {
        if (p0[axis] != p0[axis] || p1[axis] != p1[axis])
            return false; // NaN corner: min/max above would quietly drop it
        const f32 lo = kinetix::Max(rangeMin[axis], 0.0f);
        const f32 hi = kinetix::Min(rangeMax[axis], resolution[axis]);
        if (!(lo < hi))
            return false;
        mini[axis] = static_cast<u32>(lo);
        maxi[axis] = static_cast<u32>(hi);
    }

    outMin = mini;
    outMax = maxi;
    return true;
}

// Triangle-vs-AABB separating axis test: 3 box axes, 1 triangle normal, and
// the 9 cross products of a box axis with a triangle edge.
bool triangleIntersectsAABB(const Math::vec3& boxCenter, const Math::vec3& boxHalfExtent,
                            const Math::vec3& v0, const Math::vec3& v1, const Math::vec3& v2)
{
    const Math::vec3 p0 = v0 - boxCenter;
    const Math::vec3 p1 = v1 - boxCenter;
    const Math::vec3 p2 = v2 - boxCenter;

    const Math::vec3 e0 = p1 - p0;
    const Math::vec3 e1 = p2 - p1;
    const Math::vec3 e2 = p0 - p2;

    const Math::vec3 edges[3] = {e0, e1, e2};
    const Math::vec3 boxAxes[3] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                                  Math::vec3(0.0f, 0.0f, 1.0f)};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            const Math::vec3 axis = Math::cross(boxAxes[i], edges[j]);
            const f32 r = boxHalfExtent.x * fabsf(axis.x) +
                         boxHalfExtent.y * fabsf(axis.y) +
                         boxHalfExtent.z * fabsf(axis.z);

            const f32 d0 = Math::dot(p0, axis);
            const f32 d1 = Math::dot(p1, axis);
            const f32 d2 = Math::dot(p2, axis);

            const f32 projMin = kinetix::Min(d0, kinetix::Min(d1, d2));
            const f32 projMax = kinetix::Max(d0, kinetix::Max(d1, d2));
            if (projMin > r || projMax < -r)
                return false;
        }
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        const f32 projMin = kinetix::Min(p0[axis], kinetix::Min(p1[axis], p2[axis]));
        const f32 projMax = kinetix::Max(p0[axis], kinetix::Max(p1[axis], p2[axis]));
        if (projMin > boxHalfExtent[axis] || projMax < -boxHalfExtent[axis])
            return false;
    }

    const Math::vec3 normal = Math::cross(e0, e1);
    const f32 r = boxHalfExtent.x * fabsf(normal.x) +
                 boxHalfExtent.y * fabsf(normal.y) +
                 boxHalfExtent.z * fabsf(normal.z);
    const f32 distance = Math::dot(normal, p0);
    if (fabsf(distance) > r)
        return false;

    return true;
}

Math::vec3 closestPointOnSegment(const Math::vec3& a, const Math::vec3& b, const Math::vec3& point)
{
    const Math::vec3 ab = b - a;
    const f32 lengthSq = Math::dot(ab, ab);
    if (lengthSq < Epsilon)
        return a;
    f32 t = Math::dot(point - a, ab) / lengthSq;
    t = Math::clamp(t, 0.0f, 1.0f);
    return a + ab * t;
}

// base and tip are the capsule's poles, not the centres of its end spheres: the segment shrinks by
// one radius at each end before the distance test. A degenerate axis leaves the segment as it is,
// since a zero vector cannot be normalised.
bool pointInCapsule(const Math::vec3& point, const Math::vec3& base, const Math::vec3& tip,
                    f32 radius)
{
    Math::vec3 a = base;
    Math::vec3 b = tip;
    const Math::vec3 axis = tip - base;
    if (Math::dot(axis, axis) >= Epsilon)
    {
        const Math::vec3 offset = Math::normalize(axis) * radius;
        a = base + offset;
        b = tip - offset;
    }

    const Math::vec3 closest = closestPointOnSegment(a, b, point);
    const Math::vec3 delta = point - closest;
    return Math::dot(delta, delta) <= radius * radius;
}

void aabbCorners(const Math::vec3& center, const Math::vec3& halfExtent, Math::vec3 outCorners[8])
{
    outCorners[0] = center + Math::vec3(-halfExtent.x, -halfExtent.y, -halfExtent.z);
    outCorners[1] = center + Math::vec3(-halfExtent.x, halfExtent.y, -halfExtent.z);
    outCorners[2] = center + Math::vec3(-halfExtent.x, halfExtent.y, halfExtent.z);
    outCorners[3] = center + Math::vec3(-halfExtent.x, -halfExtent.y, halfExtent.z);
    outCorners[4] = center + Math::vec3(halfExtent.x, -halfExtent.y, -halfExtent.z);
    outCorners[5] = center + Math::vec3(halfExtent.x, halfExtent.y, -halfExtent.z);
    outCorners[6] = center + Math::vec3(halfExtent.x, halfExtent.y, halfExtent.z);
    outCorners[7] = center + Math::vec3(halfExtent.x, -halfExtent.y, halfExtent.z);
}

}

void VoxelMap::create(u32 dimensionX, u32 dimensionY, u32 dimensionZ)
{
    mResolution.x = kinetix::Max(4u, dimensionX);
    mResolution.y = kinetix::Max(4u, dimensionY);
    mResolution.z = kinetix::Max(4u, dimensionZ);

    mResolutionDiv4.x = (mResolution.x + 3u) / 4u;
    mResolutionDiv4.y = (mResolution.y + 3u) / 4u;
    mResolutionDiv4.z = (mResolution.z + 3u) / 4u;

    mResolutionRcp.x = 1.0f / static_cast<f32>(mResolution.x);
    mResolutionRcp.y = 1.0f / static_cast<f32>(mResolution.y);
    mResolutionRcp.z = 1.0f / static_cast<f32>(mResolution.z);

    mVoxels.clear();
    mVoxels.resize(static_cast<usize>(mResolutionDiv4.x) * mResolutionDiv4.y * mResolutionDiv4.z);
}

void VoxelMap::clearVoxels()
{
    kinetix::Fill(mVoxels.begin(), mVoxels.end(), 0ull);
}

bool VoxelMap::valid() const
{
    return !mVoxels.empty();
}

const Math::uvec3& VoxelMap::resolution() const
{
    return mResolution;
}

const Math::vec3& VoxelMap::center() const
{
    return mCenter;
}

void VoxelMap::setCenter(const Math::vec3& center)
{
    mCenter = center;
}

const Math::vec3& VoxelMap::voxelSize() const
{
    return mVoxelSize;
}

void VoxelMap::setVoxelSize(f32 size)
{
    setVoxelSize(Math::vec3(size, size, size));
}

void VoxelMap::setVoxelSize(const Math::vec3& size)
{
    mVoxelSize = size;
    mVoxelSizeRcp.x = 1.0f / mVoxelSize.x;
    mVoxelSizeRcp.y = 1.0f / mVoxelSize.y;
    mVoxelSizeRcp.z = 1.0f / mVoxelSize.z;
}

usize VoxelMap::memorySize() const
{
    return mVoxels.size() * sizeof(u64);
}

AABB VoxelMap::bounds() const
{
    AABB box;
    const Math::vec3 halfWidth = Math::vec3(mResolution) * mVoxelSize;
    box.min = mCenter - halfWidth;
    box.max = mCenter + halfWidth;
    return box;
}

void VoxelMap::fromBounds(const AABB& box)
{
    mCenter = box.center();
    const Math::vec3 halfWidth = box.extents();
    setVoxelSize(Math::vec3(halfWidth.x / static_cast<f32>(mResolution.x),
                          halfWidth.y / static_cast<f32>(mResolution.y),
                          halfWidth.z / static_cast<f32>(mResolution.z)));
}

Math::uvec3 VoxelMap::worldToCoord(const Math::vec3& worldPosition) const
{
    // The signed coordinate reinterpreted: a position before the grid wraps to a huge (invalid)
    // coordinate instead of an undefined float-to-unsigned cast of a negative value.
    return Math::uvec3(worldToCoordSigned(worldPosition));
}

Math::ivec3 VoxelMap::worldToCoordSigned(const Math::vec3& worldPosition) const
{
    const Math::vec3 uvw = worldToUvw(worldPosition, mCenter, mResolutionRcp, mVoxelSizeRcp);
    return pixelToCoord(uvw * Math::vec3(mResolution));
}

Math::vec3 VoxelMap::coordToWorld(const Math::uvec3& coord) const
{
    const Math::vec3 uvw = (Math::vec3(coord) + Math::vec3(0.5f)) * mResolutionRcp;
    return uvwToWorld(uvw, mCenter, Math::vec3(mResolution), mVoxelSize);
}

Math::vec3 VoxelMap::coordToWorld(const Math::ivec3& coord) const
{
    const Math::vec3 uvw = (Math::vec3(coord) + Math::vec3(0.5f)) * mResolutionRcp;
    return uvwToWorld(uvw, mCenter, Math::vec3(mResolution), mVoxelSize);
}

bool VoxelMap::validCoord(const Math::uvec3& coord) const
{
    return coord.x < mResolution.x && coord.y < mResolution.y && coord.z < mResolution.z;
}

bool VoxelMap::validCoord(const Math::ivec3& coord) const
{
    return validCoord(Math::uvec3(static_cast<u32>(coord.x), static_cast<u32>(coord.y),
                                 static_cast<u32>(coord.z)));
}

bool VoxelMap::voxel(const Math::uvec3& coord) const
{
    if (!validCoord(coord))
        return false;

    const Math::uvec3 macroCoord = coord / 4u;
    const u32 idx = flatten3D(macroCoord, mResolutionDiv4);
    const u64 block = mVoxels[idx];
    if (block == 0)
        return false;

    const Math::uvec3 subCoord = coord % 4u;
    const u32 bit = flatten3D(subCoord, Math::uvec3(4, 4, 4));
    const u64 mask = 1ull << bit;
    return (block & mask) != 0ull;
}

bool VoxelMap::voxel(const Math::ivec3& coord) const
{
    return voxel(Math::uvec3(static_cast<u32>(coord.x), static_cast<u32>(coord.y),
                            static_cast<u32>(coord.z)));
}

bool VoxelMap::voxel(const Math::vec3& worldPosition) const
{
    return voxel(worldToCoord(worldPosition));
}

void VoxelMap::setVoxel(const Math::uvec3& coord, bool value)
{
    if (!validCoord(coord))
        return;

    u32 index;
    u64 mask;
    voxelAddress(coord, mResolutionDiv4, index, mask);
    if (value)
        mVoxels[index] |= mask;
    else
        mVoxels[index] &= ~mask;
}

void VoxelMap::setVoxel(const Math::ivec3& coord, bool value)
{
    setVoxel(Math::uvec3(static_cast<u32>(coord.x), static_cast<u32>(coord.y),
                        static_cast<u32>(coord.z)),
             value);
}

void VoxelMap::setVoxel(const Math::vec3& worldPosition, bool value)
{
    setVoxel(worldToCoord(worldPosition), value);
}

void VoxelMap::injectTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                              bool subtract)
{
    const Math::vec3 resolutionF = Math::vec3(mResolution);

    const Math::vec3 pa = worldToUvw(a, mCenter, mResolutionRcp, mVoxelSizeRcp) * resolutionF;
    const Math::vec3 pb = worldToUvw(b, mCenter, mResolutionRcp, mVoxelSizeRcp) * resolutionF;
    const Math::vec3 pc = worldToUvw(c, mCenter, mResolutionRcp, mVoxelSizeRcp) * resolutionF;

    const Math::vec3 normal = Math::cross(pb - pa, pc - pa);
    if (normal == Math::vec3(0.0f))
        return;

    Math::uvec3 mini, maxi;
    if (!clampVoxelRange(Math::min(pa, Math::min(pb, pc)), Math::max(pa, Math::max(pb, pc)),
                         resolutionF, mini, maxi))
        return;

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                const Math::vec3 voxelCenter(static_cast<f32>(x) + 0.5f, static_cast<f32>(y) + 0.5f,
                                            static_cast<f32>(z) + 0.5f);
                if (!triangleIntersectsAABB(voxelCenter, Math::vec3(0.5f), pa, pb, pc))
                    continue;

                u32 index;
                u64 mask;
                voxelAddress(Math::uvec3(x, y, z), mResolutionDiv4, index, mask);
                // Single-threaded here, so a plain OR/AND is enough.
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::injectBounds(const AABB& box, bool subtract)
{
    const Math::vec3 resolutionF = Math::vec3(mResolution);

    const Math::vec3 pixelMin = worldToUvw(box.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const Math::vec3 pixelMax = worldToUvw(box.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    Math::uvec3 mini, maxi;
    if (!clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi))
        return;

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                u32 index;
                u64 mask;
                voxelAddress(Math::uvec3(x, y, z), mResolutionDiv4, index, mask);
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::injectSphere(const Sphere& sphere, bool subtract)
{
    const Math::vec3 resolutionF = Math::vec3(mResolution);

    AABB box;
    box.min = sphere.center - Math::vec3(sphere.radius);
    box.max = sphere.center + Math::vec3(sphere.radius);

    const Math::vec3 pixelMin = worldToUvw(box.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const Math::vec3 pixelMax = worldToUvw(box.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    Math::uvec3 mini, maxi;
    if (!clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi))
        return;

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                const Math::uvec3 coord(x, y, z);
                const Math::vec3 voxelCenter = coordToWorld(coord);

                AABB voxelBox;
                voxelBox.min = voxelCenter - mVoxelSize;
                voxelBox.max = voxelCenter + mVoxelSize;
                if (!sphere.intersects(voxelBox))
                    continue;

                u32 index;
                u64 mask;
                voxelAddress(coord, mResolutionDiv4, index, mask);
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::injectCapsule(const Math::vec3& base, const Math::vec3& tip, f32 radius,
                             bool subtract)
{
    const Math::vec3 resolutionF = Math::vec3(mResolution);

    AABB box;
    box.expand(base);
    box.expand(tip);
    box.min -= Math::vec3(radius);
    box.max += Math::vec3(radius);

    const Math::vec3 pixelMin = worldToUvw(box.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const Math::vec3 pixelMax = worldToUvw(box.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    Math::uvec3 mini, maxi;
    if (!clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi))
        return;

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                const Math::uvec3 coord(x, y, z);
                const Math::vec3 voxelCenter = coordToWorld(coord);

                bool intersects = pointInCapsule(voxelCenter, base, tip, radius);
                if (!intersects)
                {
                    Math::vec3 corners[8];
                    aabbCorners(voxelCenter, mVoxelSize, corners);
                    for (int i = 0; i < 8 && !intersects; ++i)
                        intersects = pointInCapsule(corners[i], base, tip, radius);
                }
                if (!intersects)
                    continue;

                u32 index;
                u64 mask;
                voxelAddress(coord, mResolutionDiv4, index, mask);
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::add(const VoxelMap& other)
{
    if (mVoxels.size() != other.mVoxels.size())
        return;
    for (usize i = 0; i < mVoxels.size(); ++i)
        mVoxels[i] |= other.mVoxels[i];
}

void VoxelMap::subtract(const VoxelMap& other)
{
    if (mVoxels.size() != other.mVoxels.size())
        return;
    for (usize i = 0; i < mVoxels.size(); ++i)
        mVoxels[i] &= ~other.mVoxels[i];
}

void VoxelMap::floodFill()
{
    // One flood seeded from every empty voxel on the grid's surface marks all air connected to the
    // outside; any empty voxel it did not reach is enclosed and becomes solid. Linear in the voxel
    // count.
    VoxelMap outside;
    outside.create(mResolution.x, mResolution.y, mResolution.z);
    ct::Vector<Math::ivec3> stack;

    const Math::ivec3 last(static_cast<s32>(mResolution.x) - 1, static_cast<s32>(mResolution.y) - 1,
                           static_cast<s32>(mResolution.z) - 1);
    for (s32 z = 0; z <= last.z; ++z)
    {
        for (s32 y = 0; y <= last.y; ++y)
        {
            for (s32 x = 0; x <= last.x; ++x)
            {
                const bool surface = x == 0 || y == 0 || z == 0 || x == last.x || y == last.y ||
                                     z == last.z;
                const Math::ivec3 coord(x, y, z);
                if (!surface || voxel(coord))
                    continue;
                outside.setVoxel(coord, true);
                stack.push_back(coord);
            }
        }
    }

    while (!stack.empty())
    {
        const Math::ivec3 current = stack.back();
        stack.pop_back();

        const Math::ivec3 neighbors[6] = {
            Math::ivec3(current.x - 1, current.y, current.z),
            Math::ivec3(current.x + 1, current.y, current.z),
            Math::ivec3(current.x, current.y - 1, current.z),
            Math::ivec3(current.x, current.y + 1, current.z),
            Math::ivec3(current.x, current.y, current.z - 1),
            Math::ivec3(current.x, current.y, current.z + 1),
        };
        for (const Math::ivec3& neighbor : neighbors)
        {
            if (!validCoord(neighbor) || voxel(neighbor) || outside.voxel(neighbor))
                continue;
            outside.setVoxel(neighbor, true); // marked on push, so each voxel is pushed once
            stack.push_back(neighbor);
        }
    }

    for (s32 z = 0; z <= last.z; ++z)
    {
        for (s32 y = 0; y <= last.y; ++y)
        {
            for (s32 x = 0; x <= last.x; ++x)
            {
                const Math::ivec3 coord(x, y, z);
                if (!outside.voxel(coord))
                    setVoxel(coord, true); // solid already, or enclosed air
            }
        }
    }
}

bool VoxelMap::visible(const Math::uvec3& start, const Math::uvec3& goal) const
{
    // Signed view of both ends: a coordinate before the grid arrives wrapped
    // around from worldToCoord.
    const Math::ivec3 from(start);
    const Math::ivec3 to(goal);

    const s64 delta[3] = {static_cast<s64>(to.x) - from.x, static_cast<s64>(to.y) - from.y,
                          static_cast<s64>(to.z) - from.z};
    s64 step = 0;
    for (int axis = 0; axis < 3; ++axis)
        step = kinetix::Max(step, delta[axis] < 0 ? -delta[axis] : delta[axis]);
    if (step == 0)
        return true; // start and goal coincide, avoid dividing by zero below

    // Only the stretch of the line inside the grid can be blocked, so the steps are clipped to it:
    // an end far outside costs nothing extra and no rounded position outside the grid is formed.
    f64 increment[3];
    f64 first = 0.0;
    f64 last = static_cast<f64>(step - 1);
    for (int axis = 0; axis < 3; ++axis)
    {
        const f64 origin = static_cast<f64>(from[axis]);
        const f64 extent = static_cast<f64>(mResolution[axis]);
        increment[axis] = static_cast<f64>(delta[axis]) / static_cast<f64>(step);
        if (increment[axis] == 0.0)
        {
            if (origin < 0.0 || origin >= extent)
                return true; // the line runs beside the grid, never through it
            continue;
        }
        // round(origin + i * increment) lands in [0, extent) exactly while
        // the unrounded value is in [-0.5, extent - 0.5).
        f64 enter = (-0.5 - origin) / increment[axis];
        f64 leave = (extent - 0.5 - origin) / increment[axis];
        if (enter > leave)
        {
            const f64 swap = enter;
            enter = leave;
            leave = swap;
        }
        first = kinetix::Max(first, ceil(enter));
        last = kinetix::Min(last, floor(leave));
    }

    for (f64 i = first; i <= last; i += 1.0)
    {
        Math::ivec3 coord;
        for (int axis = 0; axis < 3; ++axis)
            coord[axis] = static_cast<s32>(round(static_cast<f64>(from[axis]) + i * increment[axis]));
        if (coord == to)
            return true;
        if (voxel(coord))
            return false;
    }
    return true;
}

bool VoxelMap::visible(const Math::vec3& observer, const Math::vec3& subject) const
{
    const Math::uvec3 start = worldToCoord(observer);
    const Math::uvec3 goal = worldToCoord(subject);
    return visible(start, goal);
}

bool VoxelMap::visible(const Math::vec3& observer, const AABB& subject) const
{
    const Math::uvec3 start = worldToCoord(observer);

    const Math::vec3 resolutionF = Math::vec3(mResolution);
    const Math::vec3 pixelMin = worldToUvw(subject.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const Math::vec3 pixelMax = worldToUvw(subject.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    Math::uvec3 mini, maxi;
    if (!clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi))
        return false;

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                if (visible(start, Math::uvec3(x, y, z)))
                    return true;
            }
        }
    }
    return false;
}

}
