#include "check.h"

#include <VoxelMap.h>

#include <ct/vector.hpp>

using kinetix::AI::VoxelMap;

namespace
{

// 8^3 grid of 1 m voxels spanning [-4, 4] on every axis.
void makeMap(VoxelMap& map, unsigned dimension = 8)
{
    map.create(dimension, dimension, dimension);
    map.setVoxelSize(0.5f);
    map.setCenter(kinetix::Math::vec3(0.0f));
}

int solidCount(const VoxelMap& map)
{
    int count = 0;
    const kinetix::Math::uvec3 res = map.resolution();
    for (unsigned z = 0; z < res.z; ++z)
        for (unsigned y = 0; y < res.y; ++y)
            for (unsigned x = 0; x < res.x; ++x)
                count += map.voxel(kinetix::Math::uvec3(x, y, z)) ? 1 : 0;
    return count;
}

kinetix::AABB box(const kinetix::Math::vec3& min, const kinetix::Math::vec3& max)
{
    kinetix::AABB b;
    b.min = min;
    b.max = max;
    return b;
}

// The original per-voxel flood: an empty voxel turns solid when no empty
// path from it reaches the grid's surface.
void referenceFloodFill(VoxelMap& map)
{
    const kinetix::Math::uvec3 res = map.resolution();
    const int n = static_cast<int>(res.x * res.y * res.z);
    ct::Vector<char> enclosed;
    enclosed.resize(n);
    for (int i = 0; i < n; ++i)
    {
        const kinetix::Math::ivec3 origin(i % res.x, (i / res.x) % res.y, i / (res.x * res.y));
        enclosed[i] = 0;
        if (map.voxel(origin))
            continue;
        ct::Vector<char> seen;
        seen.resize(n);
        for (int j = 0; j < n; ++j)
            seen[j] = 0;
        ct::Vector<kinetix::Math::ivec3> stack;
        stack.push_back(origin);
        seen[i] = 1;
        bool escaped = false;
        while (!stack.empty() && !escaped)
        {
            const kinetix::Math::ivec3 c = stack.back();
            stack.pop_back();
            for (int k = 0; k < 6 && !escaped; ++k)
            {
                kinetix::Math::ivec3 nb = c;
                nb[k / 2] += (k % 2) ? 1 : -1;
                if (!map.validCoord(nb))
                {
                    escaped = true;
                    break;
                }
                const int idx = nb.x + nb.y * res.x + nb.z * res.x * res.y;
                if (seen[idx] || map.voxel(nb))
                    continue;
                seen[idx] = 1;
                stack.push_back(nb);
            }
        }
        enclosed[i] = escaped ? 0 : 1;
    }
    for (int i = 0; i < n; ++i)
        if (enclosed[i])
            map.setVoxel(kinetix::Math::ivec3(i % res.x, (i / res.x) % res.y, i / (res.x * res.y)), true);
}

}

int main()
{
    namespace KM = kinetix::Math;

    // Shapes wholly outside the grid, on each side (Y is flipped internally,
    // so "above the top" is the negative pixel side): nothing is touched and
    // nothing loops over a wrapped-around range.
    {
        const KM::vec3 offsets[6] = {KM::vec3(-15, 0, 0), KM::vec3(15, 0, 0), KM::vec3(0, -15, 0),
                                     KM::vec3(0, 15, 0), KM::vec3(0, 0, -15), KM::vec3(0, 0, 15)};
        for (const KM::vec3& offset : offsets)
        {
            VoxelMap map;
            makeMap(map);
            map.injectBounds(box(offset - KM::vec3(5.0f), offset + KM::vec3(5.0f)));
            kinetix::Sphere sphere;
            sphere.center = offset;
            sphere.radius = 2.0f;
            map.injectSphere(sphere);
            map.injectCapsule(offset - KM::vec3(1.0f), offset + KM::vec3(1.0f), 1.0f);
            map.injectTriangle(offset + KM::vec3(-2, 0, -2), offset + KM::vec3(2, 0, -2),
                               offset + KM::vec3(0, 1, 2));
            CHECK(solidCount(map) == 0);
            CHECK(!map.visible(KM::vec3(0.5f), box(offset - KM::vec3(1.0f), offset + KM::vec3(1.0f))));
        }
    }

    {
        VoxelMap map;
        makeMap(map);
        map.injectBounds(box(KM::vec3(-10.0f), KM::vec3(-2.5f)));
        CHECK(solidCount(map) == 8); // 2x2x2 voxels of [-4,-2.5] (ceil'd)
    }

    {
        VoxelMap map;
        makeMap(map);
        const float nan = KM::vec3(0.0f).x / KM::vec3(0.0f).x;
        map.injectBounds(box(KM::vec3(nan), KM::vec3(1.0f)));
        CHECK(solidCount(map) == 0);
    }

    // Coordinates just before the grid are -1, not 0.
    {
        VoxelMap map;
        makeMap(map);
        const KM::ivec3 c = map.worldToCoordSigned(KM::vec3(-4.5f, 0.5f, 0.5f));
        CHECK(c.x == -1);
        CHECK(!map.validCoord(map.worldToCoord(KM::vec3(-4.5f, 0.5f, 0.5f))));
        CHECK(!map.validCoord(map.worldToCoord(KM::vec3(-1e30f, 0.5f, 0.5f))));
    }

    {
        VoxelMap map;
        makeMap(map);
        CHECK(map.visible(KM::vec3(-40.0f, 0.5f, 0.5f), KM::vec3(3.5f, 0.5f, 0.5f)));
        map.injectBounds(box(KM::vec3(-0.9f, -4.0f, -4.0f), KM::vec3(-0.1f, 4.0f, 4.0f)));
        CHECK(!map.visible(KM::vec3(-40.0f, 0.5f, 0.5f), KM::vec3(3.5f, 0.5f, 0.5f)));
        CHECK(!map.visible(KM::vec3(-3.5f, 0.5f, 0.5f), KM::vec3(3.5f, 0.5f, 0.5f)));
        CHECK(map.visible(KM::vec3(1.5f, 0.5f, 0.5f), KM::vec3(3.5f, 0.5f, 0.5f)));
        CHECK(map.visible(KM::vec3(-3.5f, 0.5f, 0.5f), box(KM::vec3(-3.0f), KM::vec3(-2.0f))));
    }

    {
        VoxelMap map;
        makeMap(map);
        map.injectBounds(box(KM::vec3(-3.0f), KM::vec3(1.0f)));
        map.injectBounds(box(KM::vec3(-2.0f), KM::vec3(0.0f)), true);
        const int shell = solidCount(map);
        map.floodFill();
        CHECK(solidCount(map) > shell);
        CHECK(map.voxel(KM::vec3(-1.5f)));

        unsigned seed = 12345u;
        const unsigned dims[3] = {6u, 8u, 9u};
        for (int round = 0; round < 12; ++round)
        {
            VoxelMap a;
            makeMap(a, dims[round % 3]);
            const KM::uvec3 res = a.resolution();
            for (unsigned z = 0; z < res.z; ++z)
                for (unsigned y = 0; y < res.y; ++y)
                    for (unsigned x = 0; x < res.x; ++x)
                    {
                        seed = seed * 1103515245u + 12345u;
                        if (((seed >> 16) % 100u) < 45u)
                            a.setVoxel(KM::uvec3(x, y, z), true);
                    }
            VoxelMap b = a;
            a.floodFill();
            referenceFloodFill(b);
            bool same = true;
            for (unsigned z = 0; z < res.z; ++z)
                for (unsigned y = 0; y < res.y; ++y)
                    for (unsigned x = 0; x < res.x; ++x)
                        same = same && a.voxel(KM::uvec3(x, y, z)) == b.voxel(KM::uvec3(x, y, z));
            CHECK(same);
        }
    }

    return TEST_RESULT();
}
