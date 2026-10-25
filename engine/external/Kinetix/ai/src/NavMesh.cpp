#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>
#include <kinetix/core/Containers.h>
#include <kinetix/core/Algorithm.h>

#include "NavMesh.h"

#include <math.h>

namespace kinetix::AI
{

namespace
{

constexpr float kFloatMax = 3.402823466e+38F;

// How far off a triangle boundary moveAlongSurface() steps when it starts exactly on one; far
// enough that the barycentric test lands strictly inside a triangle.
constexpr float kBoundaryNudge = 1e-3f;

// Triangles meeting at one vertex on sane floor geometry. Overflow just means
// a couple of candidates go untested, not a wrong answer.
constexpr usize kVertexRingMax = 32;

Math::vec3 closestPointOnTriangle(const Math::vec3& p, const Math::vec3& a, const Math::vec3& b,
                                  const Math::vec3& c)
{
    const Math::vec3 ab = b - a;
    const Math::vec3 ac = c - a;
    const Math::vec3 ap = p - a;

    const float d1 = Math::dot(ab, ap);
    const float d2 = Math::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return a;

    const Math::vec3 bp = p - b;
    const float d3 = Math::dot(ab, bp);
    const float d4 = Math::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
        return b;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        return a + ab * (d1 / (d1 - d3));

    const Math::vec3 cp = p - c;
    const float d5 = Math::dot(ab, cp);
    const float d6 = Math::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
        return c;

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        return a + ac * (d2 / (d2 - d6));

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

    const float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

// Barycentric weights of p against triangle (a,b,c) projected onto XZ, for sampling the mesh height
// under a point inside the triangle's footprint.
void barycentricXZ(const Math::vec3& p, const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                    float& u, float& v, float& w)
{
    const float denom = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
    if (fabsf(denom) < 1e-9f)
    {
        u = 1.0f;
        v = 0.0f;
        w = 0.0f;
        return;
    }
    u = ((b.z - c.z) * (p.x - c.x) + (c.x - b.x) * (p.z - c.z)) / denom;
    v = ((c.z - a.z) * (p.x - c.x) + (a.x - c.x) * (p.z - c.z)) / denom;
    w = 1.0f - u - v;
}

float heightOnTriangle(const Math::vec3& p, const Math::vec3& a, const Math::vec3& b, const Math::vec3& c)
{
    float u, v, w;
    barycentricXZ(p, a, b, c, u, v, w);
    return u * a.y + v * b.y + w * c.y;
}

bool nearlyEqual(const Math::vec3& a, const Math::vec3& b)
{
    const Math::vec3 d = a - b;
    return d.x * d.x + d.y * d.y + d.z * d.z < 1e-8f;
}

// Signed area (x2) of (a,b,c) projected onto XZ; the sign says which side of the a->b line c falls
// on. The funnel only compares it against zero.
float triArea2XZ(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c)
{
    const float abx = b.x - a.x;
    const float abz = b.z - a.z;
    const float acx = c.x - a.x;
    const float acz = c.z - a.z;
    return acx * abz - abx * acz;
}

u64 edgeKey(u32 a, u32 b)
{
    return (static_cast<u64>(a) << 32) | static_cast<u64>(b);
}

// Parameter range [lo, hi] of the line a + t * (b - a) (XZ) for which
// lo <= alpha + beta * t <= hi holds; false when no t does.
bool clipLinear(float alpha, float beta, float lo, float hi, float& t0, float& t1)
{
    if (fabsf(beta) < 1e-12f)
    {
        if (alpha < lo || alpha > hi)
            return false;
        t0 = -kFloatMax;
        t1 = kFloatMax;
        return true;
    }
    float s0 = (lo - alpha) / beta;
    float s1 = (hi - alpha) / beta;
    if (s0 > s1)
    {
        const float swap = s0;
        s0 = s1;
        s1 = swap;
    }
    t0 = s0;
    t1 = s1;
    return true;
}

// Parameter range of the line a + t * d (XZ) inside the disk (centre, r).
bool lineDisk(float ax, float az, float dx, float dz, float cx, float cz, float r, float& t0,
              float& t1)
{
    const float fx = ax - cx;
    const float fz = az - cz;
    const float qa = dx * dx + dz * dz;
    const float qb = 2.0f * (fx * dx + fz * dz);
    const float qc = fx * fx + fz * fz - r * r;
    const float disc = qb * qb - 4.0f * qa * qc;
    if (qa < 1e-12f || disc < 0.0f)
        return false;
    const float root = sqrtf(disc);
    t0 = (-qb - root) / (2.0f * qa);
    t1 = (-qb + root) / (2.0f * qa);
    return true;
}

// Parameter range of the portal line a + t * (b - a), in XZ, that comes closer than r to the wall
// segment (c, e): the line against the capsule around the segment. A capsule is convex, so the
// range is one interval, the hull of what the two end disks and the side slab each cut.
bool portalBlockedRange(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                        const Math::vec3& e, float r, float& outT0, float& outT1)
{
    const float dx = b.x - a.x;
    const float dz = b.z - a.z;
    float lo = kFloatMax;
    float hi = -kFloatMax;
    float t0, t1;

    if (lineDisk(a.x, a.z, dx, dz, c.x, c.z, r, t0, t1))
    {
        lo = Min(lo, t0);
        hi = Max(hi, t1);
    }
    if (lineDisk(a.x, a.z, dx, dz, e.x, e.z, r, t0, t1))
    {
        lo = Min(lo, t0);
        hi = Max(hi, t1);
    }

    const float sx = e.x - c.x;
    const float sz = e.z - c.z;
    const float length = sqrtf(sx * sx + sz * sz);
    if (length > 1e-6f)
    {
        const float ux = sx / length;
        const float uz = sz / length;
        // Along the wall: (p - c) . u in [0, length]; across it: |(p - c) . n| <= r.
        const float alongAlpha = (a.x - c.x) * ux + (a.z - c.z) * uz;
        const float alongBeta = dx * ux + dz * uz;
        const float acrossAlpha = -(a.x - c.x) * uz + (a.z - c.z) * ux;
        const float acrossBeta = -dx * uz + dz * ux;
        float s0, s1, q0, q1;
        if (clipLinear(alongAlpha, alongBeta, 0.0f, length, s0, s1) &&
            clipLinear(acrossAlpha, acrossBeta, -r, r, q0, q1))
        {
            t0 = Max(s0, q0);
            t1 = Min(s1, q1);
            if (t0 <= t1)
            {
                lo = Min(lo, t0);
                hi = Max(hi, t1);
            }
        }
    }

    if (lo > hi)
        return false;
    outT0 = lo;
    outT1 = hi;
    return true;
}

}

void NavMesh::release()
{
    mVerts.clear();
    mTris.clear();
    mPortals.clear();
    mAgentRadius = 0.0f;
    mDebugTriangles.clear();
    mCentroids.clear();
    mAreaPrefix.clear();
    mGridStart.clear();
    mGridTris.clear();
    mGridCols = 0;
    mGridRows = 0;
    mSearch = SearchScratch();
}

void NavMesh::buildSpatialGrid()
{
    mCentroids.clear();
    mCentroids.reserve(mTris.size());
    Math::vec3 low(kFloatMax);
    Math::vec3 high(-kFloatMax);
    for (usize t = 0; t < mTris.size(); ++t)
    {
        const Math::vec3& a = mVerts[mTris[t].v[0]];
        const Math::vec3& b = mVerts[mTris[t].v[1]];
        const Math::vec3& c = mVerts[mTris[t].v[2]];
        mCentroids.push_back((a + b + c) * (1.0f / 3.0f));
        low = Math::min(low, Math::min(a, Math::min(b, c)));
        high = Math::max(high, Math::max(a, Math::max(b, c)));
    }
    mGridMin = low;
    mGridCell = 2.0f;
    // Keep the table near the triangle count on huge, sparse meshes.
    for (;;)
    {
        mGridCols = static_cast<s32>(floorf((high.x - low.x) / mGridCell)) + 1;
        mGridRows = static_cast<s32>(floorf((high.z - low.z) / mGridCell)) + 1;
        if (static_cast<usize>(mGridCols) * static_cast<usize>(mGridRows) <= mTris.size() * 4 + 64)
            break;
        mGridCell *= 2.0f;
    }
    const usize cells = static_cast<usize>(mGridCols) * static_cast<usize>(mGridRows);
    const f32 inverse = 1.0f / mGridCell;
    auto range = [&](const NavTri& tri, s32& x0, s32& x1, s32& z0, s32& z1) {
        const Math::vec3& a = mVerts[tri.v[0]];
        const Math::vec3& b = mVerts[tri.v[1]];
        const Math::vec3& c = mVerts[tri.v[2]];
        x0 = static_cast<s32>(floorf((Math::min(a.x, Math::min(b.x, c.x)) - mGridMin.x) * inverse));
        x1 = static_cast<s32>(floorf((Math::max(a.x, Math::max(b.x, c.x)) - mGridMin.x) * inverse));
        z0 = static_cast<s32>(floorf((Math::min(a.z, Math::min(b.z, c.z)) - mGridMin.z) * inverse));
        z1 = static_cast<s32>(floorf((Math::max(a.z, Math::max(b.z, c.z)) - mGridMin.z) * inverse));
        x0 = Math::max(x0, 0);
        z0 = Math::max(z0, 0);
        x1 = Math::min(x1, mGridCols - 1);
        z1 = Math::min(z1, mGridRows - 1);
    };
    mGridStart.clear();
    mGridStart.resize(cells + 1, 0u);
    for (usize t = 0; t < mTris.size(); ++t)
    {
        s32 x0, x1, z0, z1;
        range(mTris[t], x0, x1, z0, z1);
        for (s32 z = z0; z <= z1; ++z)
            for (s32 x = x0; x <= x1; ++x)
                ++mGridStart[static_cast<usize>(z) * static_cast<usize>(mGridCols) + static_cast<usize>(x) + 1];
    }
    for (usize i = 0; i < cells; ++i)
        mGridStart[i + 1] += mGridStart[i];
    mGridTris.clear();
    mGridTris.resize(mGridStart[cells], -1);
    ct::Vector<u32> fill;
    fill.resize(cells);
    for (usize i = 0; i < cells; ++i)
        fill[i] = mGridStart[i];
    for (usize t = 0; t < mTris.size(); ++t)
    {
        s32 x0, x1, z0, z1;
        range(mTris[t], x0, x1, z0, z1);
        for (s32 z = z0; z <= z1; ++z)
            for (s32 x = x0; x <= x1; ++x)
                mGridTris[fill[static_cast<usize>(z) * static_cast<usize>(mGridCols) + static_cast<usize>(x)]++] =
                    static_cast<s32>(t);
    }
}

bool NavMesh::valid() const
{
    return !mTris.empty();
}

bool NavMesh::build(const f32* vertices, s32 vertexCount, const s32* indices, s32 triangleCount,
                    const NavMeshConfig& config, const Math::vec3* groundSeed)
{
    release();
    if (!vertices || !indices || vertexCount <= 0 || triangleCount <= 0)
        return false;

    mVerts.reserve(static_cast<usize>(vertexCount));
    for (s32 i = 0; i < vertexCount; ++i)
        mVerts.push_back(Math::vec3(vertices[i * 3 + 0], vertices[i * 3 + 1], vertices[i * 3 + 2]));

    const float slopeCos = cosf(Math::radians(config.agentMaxSlope));

    mTris.reserve(static_cast<usize>(triangleCount));
    for (s32 t = 0; t < triangleCount; ++t)
    {
        const s32 i0 = indices[t * 3 + 0];
        const s32 i1 = indices[t * 3 + 1];
        const s32 i2 = indices[t * 3 + 2];
        if (i0 < 0 || i0 >= vertexCount || i1 < 0 || i1 >= vertexCount || i2 < 0 || i2 >= vertexCount)
            continue;

        const Math::vec3& a = mVerts[static_cast<usize>(i0)];
        const Math::vec3& b = mVerts[static_cast<usize>(i1)];
        const Math::vec3& c = mVerts[static_cast<usize>(i2)];
        Math::vec3 normal = Math::cross(b - a, c - a);
        const float len = Math::length(normal);
        if (len < 1e-8f)
            continue;
        normal = normal * (1.0f / len);
        if (normal.y < slopeCos)
            continue;

        NavTri tri;
        tri.v[0] = static_cast<u32>(i0);
        tri.v[1] = static_cast<u32>(i1);
        tri.v[2] = static_cast<u32>(i2);
        tri.normal = normal;
        mTris.push_back(tri);
    }

    if (mTris.empty())
    {
        mVerts.clear();
        return false;
    }

    buildAdjacency(config.edgeWeldDistance, config.agentMaxClimb);
    buildSpatialGrid();

    if (groundSeed)
        pruneUnreachable(*groundSeed);

    // After pruneUnreachable(): the triangles it turns off are walls too, as
    // far as clearance is concerned.
    if (config.agentRadius > 0.0f)
        computePortalSpans(config.agentRadius, config.agentMaxClimb);

    // After pruneUnreachable(): only the triangles left on count.
    mAreaPrefix.clear();
    mAreaPrefix.reserve(mTris.size());
    f32 area = 0.0f;
    for (usize t = 0; t < mTris.size(); ++t)
    {
        if (mTris[t].active)
            area += fabsf(triArea2XZ(mVerts[mTris[t].v[0]], mVerts[mTris[t].v[1]], mVerts[mTris[t].v[2]])) * 0.5f;
        mAreaPrefix.push_back(area);
    }

    rebuildDebugTriangles();
    return true;
}

void NavMesh::buildAdjacency(float weldDistance, float maxClimb)
{
    // An edge links its two triangles only when it is manifold (exactly two triangles use it, one
    // in each direction). With three or more, any pairing is arbitrary and can come out one-way (A
    // links to B while B links to C), so such an edge is left open; the weld pass below may still
    // join two of its sides, in both directions.
    HashMap<u64, s32> edgeOwner;
    HashMap<u64, u32> edgeUses; // undirected key, min vertex first
    for (usize t = 0; t < mTris.size(); ++t)
        for (s32 e = 0; e < 3; ++e)
        {
            const u32 a = mTris[t].v[static_cast<usize>(e)];
            const u32 b = mTris[t].v[static_cast<usize>((e + 1) % 3)];
            edgeOwner[edgeKey(a, b)] = static_cast<s32>(t) * 3 + e;
            const u64 undirected = a < b ? edgeKey(a, b) : edgeKey(b, a);
            u32* uses = edgeUses.find(undirected);
            if (uses)
                ++*uses;
            else
                edgeUses[undirected] = 1;
        }

    for (usize t = 0; t < mTris.size(); ++t)
        for (s32 e = 0; e < 3; ++e)
        {
            const u32 a = mTris[t].v[static_cast<usize>(e)];
            const u32 b = mTris[t].v[static_cast<usize>((e + 1) % 3)];
            const u32* uses = edgeUses.find(a < b ? edgeKey(a, b) : edgeKey(b, a));
            if (!uses || *uses != 2)
                continue;
            const s32* found = edgeOwner.find(edgeKey(b, a));
            if (!found)
                continue;
            const s32 otherTri = *found / 3;
            const s32 otherEdge = *found % 3;
            if (static_cast<usize>(otherTri) == t)
                continue;
            mTris[t].neighbor[e] = otherTri;
            mTris[static_cast<usize>(otherTri)].neighbor[otherEdge] = static_cast<s32>(t);
        }

    struct BoundaryEdge
    {
        u32 tri;
        s32 edge;
        Math::vec3 a, b;
    };
    ct::Vector<BoundaryEdge> boundary;
    for (usize t = 0; t < mTris.size(); ++t)
        for (s32 e = 0; e < 3; ++e)
            if (mTris[t].neighbor[e] == -1)
            {
                BoundaryEdge be;
                be.tri = static_cast<u32>(t);
                be.edge = e;
                be.a = mVerts[mTris[t].v[static_cast<usize>(e)]];
                be.b = mVerts[mTris[t].v[static_cast<usize>((e + 1) % 3)]];
                boundary.push_back(be);
            }

    const float weld2 = weldDistance * weldDistance;

    // Grelha espacial em XZ sobre os extremos das arestas abertas. Duas arestas so soldam se as
    // pontas ficam a menos de weldDistance, portanto basta olhar as celulas vizinhas: O(B) na
    // pratica em vez de O(B^2).
    if (!boundary.empty())
    {
        const float cell = weldDistance > 1e-6f ? weldDistance : 1e-6f;
        const float invCell = 1.0f / cell;

        // Chave da celula a partir do ponto 'a' de cada aresta; os dois pontos de uma aresta
        // candidata caem na mesma celula ou numa adjacente (varrimento 3x3 abaixo).
        auto cellKey = [invCell](float x, float z) {
            const s64 cx = static_cast<s64>(floorf(x * invCell));
            const s64 cz = static_cast<s64>(floorf(z * invCell));
            return (static_cast<u64>(static_cast<u32>(static_cast<s32>(cx))) << 32) |
                   static_cast<u64>(static_cast<u32>(static_cast<s32>(cz)));
        };

        // Cada celula guarda a cabeca de uma lista ligada por indices, sem alocar um vetor por
        // celula.
        HashMap<u64, s32> head;
        ct::Vector<s32> next(boundary.size(), -1);
        for (usize i = 0; i < boundary.size(); ++i)
        {
            const u64 key = cellKey(boundary[i].a.x, boundary[i].a.z);
            s32* slot = head.find(key);
            if (slot)
            {
                next[i] = *slot;
                *slot = static_cast<s32>(i);
            }
            else
            {
                head[key] = static_cast<s32>(i);
            }
        }

        for (usize i = 0; i < boundary.size(); ++i)
        {
            NavTri& triI = mTris[boundary[i].tri];
            if (triI.neighbor[boundary[i].edge] != -1)
                continue;

            // A aresta parceira tem o seu 'a' perto do nosso 'b' (sentidos
            // opostos), por isso a procura centra-se em b.
            const s64 bx = static_cast<s64>(floorf(boundary[i].b.x * invCell));
            const s64 bz = static_cast<s64>(floorf(boundary[i].b.z * invCell));

            bool welded = false;
            for (s64 ox = -1; ox <= 1 && !welded; ++ox)
                for (s64 oz = -1; oz <= 1 && !welded; ++oz)
                {
                    const u64 key =
                        (static_cast<u64>(static_cast<u32>(static_cast<s32>(bx + ox))) << 32) |
                        static_cast<u64>(static_cast<u32>(static_cast<s32>(bz + oz)));
                    const s32* slot = head.find(key);
                    if (!slot)
                        continue;

                    for (s32 j = *slot; j != -1; j = next[static_cast<usize>(j)])
                    {
                        const usize ju = static_cast<usize>(j);
                        if (ju == i || boundary[i].tri == boundary[ju].tri)
                            continue;
                        NavTri& triJ = mTris[boundary[ju].tri];
                        if (triJ.neighbor[boundary[ju].edge] != -1)
                            continue;

                        const Math::vec3 dA = boundary[i].a - boundary[ju].b;
                        const Math::vec3 dB = boundary[i].b - boundary[ju].a;
                        if (dA.x * dA.x + dA.z * dA.z > weld2 || dB.x * dB.x + dB.z * dB.z > weld2)
                            continue;
                        if (fabsf(dA.y) > maxClimb || fabsf(dB.y) > maxClimb)
                            continue;

                        triI.neighbor[boundary[i].edge] = static_cast<s32>(boundary[ju].tri);
                        triJ.neighbor[boundary[ju].edge] = static_cast<s32>(boundary[i].tri);
                        welded = true;
                        break;
                    }
                }
        }
    }
}

void NavMesh::pruneUnreachable(const Math::vec3& seed)
{
    Math::vec3 closest;
    const s32 seedTri = findContainingTri(seed, Math::vec3(4.0f, 200.0f, 4.0f), &closest);
    if (seedTri < 0)
        return;

    ct::Vector<u8> reached(mTris.size(), 0);
    ct::Vector<s32> frontier;
    reached[static_cast<usize>(seedTri)] = 1;
    frontier.push_back(seedTri);

    while (!frontier.empty())
    {
        const s32 cur = frontier.back();
        frontier.pop_back();
        for (s32 e = 0; e < 3; ++e)
        {
            const s32 nb = mTris[static_cast<usize>(cur)].neighbor[e];
            if (nb < 0 || reached[static_cast<usize>(nb)])
                continue;
            reached[static_cast<usize>(nb)] = 1;
            frontier.push_back(nb);
        }
    }

    for (usize t = 0; t < mTris.size(); ++t)
        mTris[t].active = reached[t] != 0;
}

// Clearance on the unexpanded mesh: every triangle stays and each portal is cut down to the part at
// least agentRadius from every wall. Walls are the boundary edges (open mesh edges and edges
// against a triangle pruneUnreachable() turned off), bucketed on an XZ grid. A wall whose height
// range is more than a climbable step from the portal's belongs to another floor and is ignored. A
// wall excludes one interval of a portal line (the line against the capsule of radius agentRadius
// around the wall); the portal keeps the longest stretch no wall excludes, or closes. Both sides of
// a portal agree on whether it is open.
void NavMesh::computePortalSpans(float agentRadius, float maxClimb)
{
    const usize triCount = mTris.size();
    mPortals.clear();
    mPortals.resize(triCount * 3);
    mAgentRadius = agentRadius;

    struct Wall
    {
        Math::vec3 a, b;
    };
    ct::Vector<Wall> walls;
    for (usize t = 0; t < triCount; ++t)
    {
        if (!mTris[t].active)
            continue;
        for (s32 e = 0; e < 3; ++e)
        {
            const s32 next = mTris[t].neighbor[e];
            if (next >= 0 && mTris[static_cast<usize>(next)].active)
                continue;
            walls.push_back(Wall{mVerts[mTris[t].v[static_cast<usize>(e)]],
                                 mVerts[mTris[t].v[static_cast<usize>((e + 1) % 3)]]});
        }
    }

    // Each wall is filed under every cell its box, grown by the radius, overlaps, so a portal
    // looking through the cells under its own box meets every wall that can come within the radius
    // of it.
    const float cell = Max(agentRadius * 2.0f, 0.5f);
    const float invCell = 1.0f / cell;
    auto cellOf = [invCell](float value) {
        return static_cast<s32>(floorf(value * invCell));
    };
    auto cellKey = [](s32 x, s32 z) {
        return (static_cast<u64>(static_cast<u32>(x)) << 32) | static_cast<u64>(static_cast<u32>(z));
    };
    struct Filed
    {
        s32 wall;
        s32 next;
    };
    HashMap<u64, s32> head;
    ct::Vector<Filed> filed;
    for (usize w = 0; w < walls.size(); ++w)
    {
        const Wall& wall = walls[w];
        const s32 x0 = cellOf(Min(wall.a.x, wall.b.x) - agentRadius);
        const s32 x1 = cellOf(Max(wall.a.x, wall.b.x) + agentRadius);
        const s32 z0 = cellOf(Min(wall.a.z, wall.b.z) - agentRadius);
        const s32 z1 = cellOf(Max(wall.a.z, wall.b.z) + agentRadius);
        for (s32 x = x0; x <= x1; ++x)
            for (s32 z = z0; z <= z1; ++z)
            {
                const u64 key = cellKey(x, z);
                s32* slot = head.find(key);
                filed.push_back(Filed{static_cast<s32>(w), slot ? *slot : -1});
                if (slot)
                    *slot = static_cast<s32>(filed.size() - 1);
                else
                    head[key] = static_cast<s32>(filed.size() - 1);
            }
    }

    // Exactly-fitting gaps (width == 2 * radius) stay open rather than
    // flickering shut on rounding.
    const float exclusionRadius = Max(agentRadius - 1e-4f, 0.0f);

    struct Range
    {
        float t0, t1;
    };
    ct::Vector<Range> blocked;
    ct::Vector<u32> seenStamp(walls.size(), 0u);
    u32 stamp = 0;

    for (usize t = 0; t < triCount; ++t)
    {
        const NavTri& tri = mTris[t];
        for (s32 e = 0; e < 3; ++e)
        {
            PortalSpan& span = mPortals[t * 3 + static_cast<usize>(e)];
            const s32 next = tri.neighbor[e];
            if (!tri.active || next < 0 || !mTris[static_cast<usize>(next)].active)
            {
                span.lo = 1.0f;
                span.hi = 0.0f;
                continue;
            }

            const Math::vec3& a = mVerts[tri.v[static_cast<usize>(e)]];
            const Math::vec3& b = mVerts[tri.v[static_cast<usize>((e + 1) % 3)]];
            const float portalLow = Min(a.y, b.y);
            const float portalHigh = Max(a.y, b.y);

            ++stamp;
            blocked.clear();
            const s32 x0 = cellOf(Min(a.x, b.x));
            const s32 x1 = cellOf(Max(a.x, b.x));
            const s32 z0 = cellOf(Min(a.z, b.z));
            const s32 z1 = cellOf(Max(a.z, b.z));
            for (s32 x = x0; x <= x1; ++x)
                for (s32 z = z0; z <= z1; ++z)
                {
                    const s32* slot = head.find(cellKey(x, z));
                    for (s32 f = slot ? *slot : -1; f >= 0; f = filed[static_cast<usize>(f)].next)
                    {
                        const usize w = static_cast<usize>(filed[static_cast<usize>(f)].wall);
                        if (seenStamp[w] == stamp)
                            continue;
                        seenStamp[w] = stamp;

                        const Wall& wall = walls[w];
                        const float wallLow = Min(wall.a.y, wall.b.y);
                        const float wallHigh = Max(wall.a.y, wall.b.y);
                        const float gap = Max(Max(wallLow - portalHigh, portalLow - wallHigh), 0.0f);
                        if (gap > maxClimb)
                            continue;

                        float t0, t1;
                        if (!portalBlockedRange(a, b, wall.a, wall.b, exclusionRadius, t0, t1))
                            continue;
                        if (t1 <= 0.0f || t0 >= 1.0f)
                            continue;
                        blocked.push_back(Range{t0, t1});
                    }
                }

            // Longest stretch of [0, 1] none of the ranges covers.
            for (usize i = 1; i < blocked.size(); ++i)
            {
                const Range key = blocked[i];
                usize j = i;
                for (; j > 0 && blocked[j - 1].t0 > key.t0; --j)
                    blocked[j] = blocked[j - 1];
                blocked[j] = key;
            }
            float cursor = 0.0f;
            float bestLo = 1.0f;
            float bestHi = 0.0f;
            for (const Range& range : blocked)
            {
                if (range.t0 > cursor && range.t0 - cursor > bestHi - bestLo)
                {
                    bestLo = cursor;
                    bestHi = range.t0;
                }
                cursor = Max(cursor, range.t1);
            }
            if (cursor < 1.0f && 1.0f - cursor > bestHi - bestLo)
            {
                bestLo = cursor;
                bestHi = 1.0f;
            }
            span.lo = bestLo;
            span.hi = bestHi;
        }
    }

    // A welded pair is two different segments, each measured on its own;
    // what matters is that A* sees the crossing the same both ways.
    for (usize t = 0; t < triCount; ++t)
        for (s32 e = 0; e < 3; ++e)
        {
            const s32 next = mTris[t].neighbor[e];
            if (next < 0)
                continue;
            for (s32 back = 0; back < 3; ++back)
            {
                if (mTris[static_cast<usize>(next)].neighbor[back] != static_cast<s32>(t))
                    continue;
                PortalSpan& mine = mPortals[t * 3 + static_cast<usize>(e)];
                PortalSpan& theirs = mPortals[static_cast<usize>(next) * 3 + static_cast<usize>(back)];
                if (mine.lo > mine.hi || theirs.lo > theirs.hi)
                {
                    mine.lo = theirs.lo = 1.0f;
                    mine.hi = theirs.hi = 0.0f;
                }
            }
        }
}

bool NavMesh::portalOpen(s32 tri, s32 edge) const
{
    if (mPortals.empty())
        return true;
    const PortalSpan& span = mPortals[static_cast<usize>(tri) * 3 + static_cast<usize>(edge)];
    return span.lo <= span.hi;
}

void NavMesh::keepClearOfWalls(Math::vec3& point, s32& tri) const
{
    if (mAgentRadius <= 0.0f || tri < 0)
        return;

    // A few pushes, so a point in a corner clears both walls; a gap too
    // narrow for the agent has no clear point and is left where it ends up.
    const Math::vec3 extents(0.05f, 0.5f, 0.05f);
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        f32 distance = 0.0f;
        Math::vec3 normal;
        if (!distanceToWall(point, mAgentRadius, distance, normal, extents))
            return;
        if (distance >= mAgentRadius - 1e-3f || Math::length(normal) < 0.5f)
            return;

        const Math::vec3 target = point + normal * (mAgentRadius - distance + 1e-3f);
        Math::vec3 moved;
        s32 movedTri = tri;
        if (!moveAlongSurface(point, target, moved, movedTri, extents))
            return;
        point = moved;
        tri = movedTri;
    }
}

void NavMesh::rebuildDebugTriangles()
{
    mDebugTriangles.clear();
    mDebugTriangles.reserve(mTris.size() * 3);
    for (usize t = 0; t < mTris.size(); ++t)
    {
        if (!mTris[t].active)
            continue;
        mDebugTriangles.push_back(mVerts[mTris[t].v[0]]);
        mDebugTriangles.push_back(mVerts[mTris[t].v[1]]);
        mDebugTriangles.push_back(mVerts[mTris[t].v[2]]);
    }
}

s32 NavMesh::findContainingTri(const Math::vec3& point, const Math::vec3& searchExtents,
                               Math::vec3* outClosest) const
{
    s32 best = -1;
    float bestDist2 = kFloatMax;
    auto test = [&](usize t) {
        if (!mTris[t].active)
            return;
        const Math::vec3& a = mVerts[mTris[t].v[0]];
        const Math::vec3& b = mVerts[mTris[t].v[1]];
        const Math::vec3& c = mVerts[mTris[t].v[2]];
        const Math::vec3 cp = closestPointOnTriangle(point, a, b, c);
        const Math::vec3 delta = cp - point;
        if (fabsf(delta.x) > searchExtents.x || fabsf(delta.y) > searchExtents.y ||
            fabsf(delta.z) > searchExtents.z)
            return;
        const float d2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
        // Ties go to the lowest index, as the scan over the whole mesh did.
        if (d2 < bestDist2 || (d2 == bestDist2 && static_cast<s32>(t) < best))
        {
            bestDist2 = d2;
            best = static_cast<s32>(t);
            if (outClosest)
                *outClosest = cp;
        }
    };
    if (mGridCols <= 0 || mGridRows <= 0)
    {
        for (usize t = 0; t < mTris.size(); ++t)
            test(t);
        return best;
    }
    // A triangle whose box misses the search box cannot have its closest point inside it.
    const f32 inverse = 1.0f / mGridCell;
    const s32 x0 = Math::max(static_cast<s32>(floorf((point.x - searchExtents.x - mGridMin.x) * inverse)), 0);
    const s32 x1 = Math::min(static_cast<s32>(floorf((point.x + searchExtents.x - mGridMin.x) * inverse)), mGridCols - 1);
    const s32 z0 = Math::max(static_cast<s32>(floorf((point.z - searchExtents.z - mGridMin.z) * inverse)), 0);
    const s32 z1 = Math::min(static_cast<s32>(floorf((point.z + searchExtents.z - mGridMin.z) * inverse)), mGridRows - 1);
    for (s32 z = z0; z <= z1; ++z)
        for (s32 x = x0; x <= x1; ++x)
        {
            const usize cell = static_cast<usize>(z) * static_cast<usize>(mGridCols) + static_cast<usize>(x);
            for (u32 k = mGridStart[cell]; k < mGridStart[cell + 1]; ++k)
                test(static_cast<usize>(mGridTris[k]));
        }
    return best;
}

bool NavMesh::nearestPoint(const Math::vec3& point, Math::vec3& out, const Math::vec3& searchExtents) const
{
    Math::vec3 closest;
    if (findContainingTri(point, searchExtents, &closest) < 0)
        return false;
    out = closest;
    return true;
}

bool NavMesh::getPortalPoints(s32 triA, s32 triB, Math::vec3& left, Math::vec3& right) const
{
    const NavTri& t = mTris[static_cast<usize>(triA)];
    for (s32 e = 0; e < 3; ++e)
    {
        if (t.neighbor[e] == triB)
        {
            const Math::vec3& a = mVerts[t.v[static_cast<usize>(e)]];
            const Math::vec3& b = mVerts[t.v[static_cast<usize>((e + 1) % 3)]];
            if (mPortals.empty())
            {
                left = a;
                right = b;
                return true;
            }
            const PortalSpan& span = mPortals[static_cast<usize>(triA) * 3 + static_cast<usize>(e)];
            left = a + (b - a) * span.lo;
            right = a + (b - a) * span.hi;
            return true;
        }
    }
    return false;
}

Math::vec3 NavMesh::triCentroid(s32 tri) const
{
    const NavTri& t = mTris[static_cast<usize>(tri)];
    return (mVerts[t.v[0]] + mVerts[t.v[1]] + mVerts[t.v[2]]) * (1.0f / 3.0f);
}

void NavMesh::funnelPath(const ct::Vector<s32>& triPath, const Math::vec3& startPos,
                         const Math::vec3& endPos, ct::Vector<Math::vec3>& outPath) const
{
    outPath.push_back(startPos);

    if (triPath.size() < 2)
    {
        outPath.push_back(endPos);
        return;
    }

    ct::Vector<Math::vec3> lefts;
    ct::Vector<Math::vec3> rights;
    lefts.push_back(startPos);
    rights.push_back(startPos);
    for (usize i = 0; i + 1 < triPath.size(); ++i)
    {
        Math::vec3 l, r;
        if (getPortalPoints(triPath[i], triPath[i + 1], l, r))
        {
            lefts.push_back(l);
            rights.push_back(r);
        }
    }

    lefts.push_back(endPos);
    rights.push_back(endPos);


    Math::vec3 apex = startPos;
    usize apexIndex = 0;
    usize leftIndex = 0;
    usize rightIndex = 0;

    // Every restart below rewinds i to just past the new apex. That only terminates while the apex
    // keeps moving forward: a portal pair that leaves apexIndex where it was rewinds i to the same
    // place and the walk repeats forever. Two agents a fraction apart are enough to produce one.
    usize lastApexIndex = 0;

    for (usize i = 1; i < lefts.size();)
    {
        const Math::vec3& newRight = rights[i];
        const Math::vec3& newLeft = lefts[i];

        // A portal the apex itself lies on (the start sitting on the edge it leaves its triangle
        // through) constrains nothing: both ends are collinear with the apex, so taking them as the
        // funnel's sides opens it to a flat 180 degrees and the next portal emits one of them as a
        // corner, often behind the agent. An apex at one of the portal's ends is the ordinary
        // shared-vertex case handled below.
        if (i + 1 < lefts.size() && !nearlyEqual(apex, newLeft) && !nearlyEqual(apex, newRight))
        {
            const Math::vec3 portal = newRight - newLeft;
            const float lengthSq = portal.x * portal.x + portal.z * portal.z;
            float along = 0.0f;
            if (lengthSq > 1e-12f)
            {
                along = ((apex.x - newLeft.x) * portal.x + (apex.z - newLeft.z) * portal.z) / lengthSq;
                along = along < 0.0f ? 0.0f : (along > 1.0f ? 1.0f : along);
            }
            const float dx = apex.x - (newLeft.x + portal.x * along);
            const float dz = apex.z - (newLeft.z + portal.z * along);
            if (dx * dx + dz * dz < 1e-6f)
            {
                ++i;
                continue;
            }
        }

        if (triArea2XZ(apex, rights[rightIndex], newRight) <= 0.0f)
        {
            if (nearlyEqual(apex, rights[rightIndex]) ||
                triArea2XZ(apex, lefts[leftIndex], newRight) > 0.0f)
            {
                rightIndex = i;
            }
            else
            {
                if (outPath.empty() || !nearlyEqual(outPath.back(), lefts[leftIndex]))
                    outPath.push_back(lefts[leftIndex]);
                apex = lefts[leftIndex];
                apexIndex = leftIndex;
                // Both sides collapse back onto the new apex; keeping the old ones narrows the
                // funnel against itself and it stops emitting corners.
                lefts[apexIndex] = apex;
                rights[apexIndex] = apex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                if (apexIndex <= lastApexIndex)
                {
                    ++i;
                    continue;
                }
                lastApexIndex = apexIndex;
                // Restart AT the apex, not past it: the portal the apex sits
                // on still has to be tested against the new funnel.
                i = apexIndex;
                ++i;
                continue;
            }
        }

        if (triArea2XZ(apex, lefts[leftIndex], newLeft) >= 0.0f)
        {
            if (nearlyEqual(apex, lefts[leftIndex]) ||
                triArea2XZ(apex, rights[rightIndex], newLeft) < 0.0f)
            {
                leftIndex = i;
            }
            else
            {
                if (outPath.empty() || !nearlyEqual(outPath.back(), rights[rightIndex]))
                    outPath.push_back(rights[rightIndex]);
                apex = rights[rightIndex];
                apexIndex = rightIndex;
                lefts[apexIndex] = apex;
                rights[apexIndex] = apex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                if (apexIndex <= lastApexIndex)
                {
                    ++i;
                    continue;
                }
                lastApexIndex = apexIndex;
                i = apexIndex;
                ++i;
                continue;
            }
        }

        ++i;
    }

    if (outPath.empty() || !nearlyEqual(outPath.back(), endPos))
        outPath.push_back(endPos);
}

bool NavMesh::findPath(const Math::vec3& start, const Math::vec3& end, ct::Vector<Math::vec3>& outPath,
                       const Math::vec3& searchExtents, bool* outPartial) const
{
    outPath.clear();
    if (outPartial)
        *outPartial = false;
    if (mTris.empty())
        return false;

    Math::vec3 startClosest, endClosest;
    const s32 startTri = findContainingTri(start, searchExtents, &startClosest);
    s32 endTri = findContainingTri(end, searchExtents, &endClosest);
    if (startTri < 0 || endTri < 0)
        return false;
    keepClearOfWalls(endClosest, endTri);

    if (startTri == endTri)
    {
        outPath.push_back(startClosest);
        outPath.push_back(endClosest);
        return true;
    }

    const usize n = mTris.size();
    SearchScratch& search = mSearch;
    if (search.g.size() != n || search.generation == ~u32(0))
    {
        search.g.clear();
        search.parent.clear();
        search.seen.clear();
        search.closed.clear();
        search.g.resize(n, kFloatMax);
        search.parent.resize(n, -1);
        search.seen.resize(n, 0u);
        search.closed.resize(n, 0u);
        search.generation = 0;
    }
    const u32 generation = ++search.generation;
    const ct::Vector<Math::vec3>& centroid = mCentroids;
    const Math::vec3 goalCentroid = centroid[static_cast<usize>(endTri)];

    using OpenEntry = SearchEntry;
    ct::Vector<OpenEntry>& open = search.open;
    open.clear();

    // A slot holds this search's g and parent only once seen in this generation.
    auto gScoreOf = [&](usize tri) { return search.seen[tri] == generation ? search.g[tri] : kFloatMax; };
    auto isClosed = [&](usize tri) { return search.closed[tri] == generation; };

    auto heapPush = [&open](OpenEntry entry) {
        open.push_back(entry);
        usize child = open.size() - 1;
        while (child > 0)
        {
            const usize parentIdx = (child - 1) / 2;
            if (open[parentIdx].f <= open[child].f)
                break;
            const OpenEntry tmp = open[parentIdx];
            open[parentIdx] = open[child];
            open[child] = tmp;
            child = parentIdx;
        }
    };

    auto heapPop = [&open]() {
        const OpenEntry top = open[0];
        open[0] = open.back();
        open.pop_back();
        const usize size = open.size();
        usize node = 0;
        for (;;)
        {
            const usize left = node * 2 + 1;
            if (left >= size)
                break;
            const usize right = left + 1;
            usize smallest = (right < size && open[right].f < open[left].f) ? right : left;
            if (open[node].f <= open[smallest].f)
                break;
            const OpenEntry tmp = open[node];
            open[node] = open[smallest];
            open[smallest] = tmp;
            node = smallest;
        }
        return top;
    };

    search.g[static_cast<usize>(startTri)] = 0.0f;
    search.parent[static_cast<usize>(startTri)] = -1;
    search.seen[static_cast<usize>(startTri)] = generation;
    heapPush({startTri, Math::distance(centroid[static_cast<usize>(startTri)], goalCentroid)});

    s32 reachedTri = startTri;
    float bestHeuristic = kFloatMax;

    while (!open.empty())
    {
        const OpenEntry cur = heapPop();
        const usize curIdx = static_cast<usize>(cur.tri);

        if (isClosed(curIdx))
            continue;
        search.closed[curIdx] = generation;

        const Math::vec3& curCentroid = centroid[curIdx];
        const float h = Math::distance(curCentroid, goalCentroid);
        if (h < bestHeuristic)
        {
            bestHeuristic = h;
            reachedTri = cur.tri;
        }

        if (cur.tri == endTri)
            break;

        const NavTri& triData = mTris[curIdx];
        const float curG = search.g[curIdx];
        for (s32 e = 0; e < 3; ++e)
        {
            const s32 nb = triData.neighbor[e];
            if (nb < 0)
                continue;
            const usize nbIdx = static_cast<usize>(nb);
            if (!mTris[nbIdx].active || isClosed(nbIdx) || !portalOpen(cur.tri, e))
                continue;

            const float tentative = curG + Math::distance(curCentroid, centroid[nbIdx]);
            if (tentative < gScoreOf(nbIdx))
            {
                search.g[nbIdx] = tentative;
                search.parent[nbIdx] = cur.tri;
                search.seen[nbIdx] = generation;
                heapPush({nb, tentative + Math::distance(centroid[nbIdx], goalCentroid)});
            }
        }
    }

    ct::Vector<s32> triPath;
    s32 cur = reachedTri;
    while (cur != -1)
    {
        triPath.push_back(cur);
        cur = search.parent[static_cast<usize>(cur)];
    }
    for (usize i = 0, j = triPath.size() - 1; i < j; ++i, --j)
        kinetix::Swap(triPath[i], triPath[j]);

    Math::vec3 finalPoint = endClosest;
    if (reachedTri != endTri)
    {
        finalPoint = closestPointOnTri(end, mTris[static_cast<usize>(reachedTri)]);
        if (outPartial)
            *outPartial = true;
    }

    funnelPath(triPath, startClosest, finalPoint, outPath);
    return true;
}

Math::vec3 NavMesh::closestPointOnTri(const Math::vec3& point, const NavTri& tri) const
{
    return closestPointOnTriangle(point, mVerts[tri.v[0]], mVerts[tri.v[1]], mVerts[tri.v[2]]);
}

void NavMesh::collectVertexRing(s32 tri, s32* out, usize capacity, usize& count) const
{
    count = 0;
    if (tri < 0 || capacity == 0)
        return;

    // Breadth-first over edge neighbours, kept to triangles that still share one of the seed's
    // vertices: the one-ring, reached without touching a triangle outside it.
    const NavTri& seed = mTris[static_cast<usize>(tri)];
    const u32 v0 = seed.v[0], v1 = seed.v[1], v2 = seed.v[2];

    out[count++] = tri;
    for (usize head = 0; head < count && count < capacity; ++head)
    {
        const NavTri& current = mTris[static_cast<usize>(out[head])];
        for (s32 e = 0; e < 3 && count < capacity; ++e)
        {
            const s32 next = current.neighbor[e];
            if (next < 0 || !mTris[static_cast<usize>(next)].active)
                continue;

            const NavTri& candidate = mTris[static_cast<usize>(next)];
            bool shares = false;
            for (s32 i = 0; i < 3 && !shares; ++i)
                shares = candidate.v[static_cast<usize>(i)] == v0 ||
                         candidate.v[static_cast<usize>(i)] == v1 ||
                         candidate.v[static_cast<usize>(i)] == v2;
            if (!shares)
                continue;

            bool seen = false;
            for (usize i = 0; i < count && !seen; ++i)
                seen = out[i] == next;
            if (!seen)
                out[count++] = next;
        }
    }
}

bool NavMesh::moveAlongSurface(const Math::vec3& from, const Math::vec3& to, Math::vec3& out,
                               const Math::vec3& searchExtents) const
{
    s32 tri = -1;
    return moveAlongSurface(from, to, out, tri, searchExtents);
}

bool NavMesh::moveAlongSurface(const Math::vec3& from, const Math::vec3& to, Math::vec3& out,
                               s32& tri, const Math::vec3& searchExtents) const
{
    if (mTris.empty())
        return false;

    Math::vec3 fromClosest = from;

    // A hint is only worth taking if it still holds: the mesh may have been rebuilt, or the agent
    // teleported, since the caller last saw it.
    bool hintUsable = false;
    if (tri >= 0 && static_cast<usize>(tri) < mTris.size() && mTris[static_cast<usize>(tri)].active)
    {
        const NavTri& hinted = mTris[static_cast<usize>(tri)];
        float u, v, w;
        barycentricXZ(from, mVerts[hinted.v[0]], mVerts[hinted.v[1]], mVerts[hinted.v[2]], u, v, w);
        if (u >= -1e-3f && v >= -1e-3f && w >= -1e-3f)
        {
            fromClosest = closestPointOnTri(from, hinted);
            hintUsable = true;
        }
    }

    if (!hintUsable)
    {
        tri = findContainingTri(from, searchExtents, &fromClosest);
        if (tri < 0)
            return false;
    }

    Math::vec3 pos = fromClosest;
    Math::vec3 target = to;

    for (int iteration = 0; iteration < 16; ++iteration)
    {
        const NavTri& triData = mTris[static_cast<usize>(tri)];
        const Math::vec3& a = mVerts[triData.v[0]];
        const Math::vec3& b = mVerts[triData.v[1]];
        const Math::vec3& c = mVerts[triData.v[2]];

        float u, v, w;
        barycentricXZ(target, a, b, c, u, v, w);
        if (u >= -1e-4f && v >= -1e-4f && w >= -1e-4f)
        {
            out = Math::vec3(target.x, heightOnTriangle(target, a, b, c), target.z);
            return true;
        }

        const Math::vec3 verts[3] = {a, b, c};
        float bestT = kFloatMax;
        s32 bestEdge = -1;
        Math::vec3 bestPoint(0.0f);
        for (s32 e = 0; e < 3; ++e)
        {
            const Math::vec3& e0 = verts[e];
            const Math::vec3& e1 = verts[(e + 1) % 3];

            const float dx1 = target.x - pos.x, dz1 = target.z - pos.z;
            const float dx2 = e1.x - e0.x, dz2 = e1.z - e0.z;
            const float denom = dx1 * dz2 - dz1 * dx2;
            if (fabsf(denom) < 1e-9f)
                continue;

            const float ex = e0.x - pos.x, ez = e0.z - pos.z;
            const float t = (ex * dz2 - ez * dx2) / denom;
            const float s = (ex * dz1 - ez * dx1) / denom;
            if (t < 1e-5f || t > 1.0f + 1e-5f || s < -1e-5f || s > 1.0f + 1e-5f)
                continue;
            if (t < bestT)
            {
                bestT = t;
                bestEdge = e;
                bestPoint = Math::vec3(pos.x + dx1 * t, 0.0f, pos.z + dz1 * t);
            }
        }

        if (bestEdge < 0)
        {
            // No edge crossing found, yet the target is outside this triangle. Usually the start is
            // exactly on a vertex or edge, so the segment leaves through a boundary it sits on at t
            // == 0, which the crossing test discards; an agent placed on integer coordinates does
            // this routinely, and left alone it pins the agent there forever. Retry from a point
            // nudged towards the target: far enough off the boundary to be strictly inside some
            // triangle, small enough not to be a teleport. Needed mid-walk too (a crossing exactly
            // at an interior vertex); the iteration cap bounds the nudges.
            {
                Math::vec3 toTarget = target - pos;
                toTarget.y = 0.0f;
                const float length = Math::length(toTarget);
                if (length > 1e-6f)
                {
                    // Candidates are the triangles touching this one's vertices, not just its edge
                    // neighbours: standing on a shared vertex, the triangle the move heads into
                    // touches only at that point. Walking the one-ring by neighbour links stays
                    // local; findContainingTri() would rescan the whole mesh.
                    const float nudge = Min(kBoundaryNudge, length * 0.5f);
                    const Math::vec3 probe = pos + toTarget * (nudge / length);

                    s32 ring[kVertexRingMax];
                    usize ringCount = 0;
                    collectVertexRing(tri, ring, kVertexRingMax, ringCount);

                    bool moved = false;
                    for (usize r = 0; r < ringCount && !moved; ++r)
                    {
                        const s32 candidate = ring[r];
                        const NavTri& candidateTri = mTris[static_cast<usize>(candidate)];
                        const Math::vec3& ca = mVerts[candidateTri.v[0]];
                        const Math::vec3& cb = mVerts[candidateTri.v[1]];
                        const Math::vec3& cc = mVerts[candidateTri.v[2]];
                        float cu, cv, cw;
                        barycentricXZ(probe, ca, cb, cc, cu, cv, cw);
                        if (cu >= -1e-4f && cv >= -1e-4f && cw >= -1e-4f)
                        {
                            pos = Math::vec3(probe.x, heightOnTriangle(probe, ca, cb, cc), probe.z);
                            tri = candidate;
                            moved = true;
                        }
                    }
                    if (moved)
                        continue;
                }
            }

            out = Math::vec3(pos.x, heightOnTriangle(pos, a, b, c), pos.z);
            return true;
        }

        bestPoint.y = heightOnTriangle(bestPoint, a, b, c);
        const s32 next = triData.neighbor[bestEdge];
        if (next < 0 || !mTris[static_cast<usize>(next)].active)
        {
            out = bestPoint;
            return true;
        }

        pos = bestPoint;
        tri = next;
    }

    out = pos;
    return true;
}

bool NavMesh::raycast(const Math::vec3& from, const Math::vec3& to, f32& outHitFraction,
                      Math::vec3& outHitNormal, const Math::vec3& searchExtents) const
{
    outHitFraction = 1.0f;
    outHitNormal = Math::vec3(0.0f);
    if (mTris.empty())
        return false;

    Math::vec3 fromClosest;
    s32 tri = findContainingTri(from, searchExtents, &fromClosest);
    if (tri < 0)
        return false;

    Math::vec3 pos = fromClosest;

    // Walk triangle to triangle along the segment, bounded by the path length so a ray that loops
    // cannot hang the caller.
    for (usize step = 0; step < mTris.size(); ++step)
    {
        const NavTri& triData = mTris[static_cast<usize>(tri)];
        const Math::vec3 verts[3] = {mVerts[triData.v[0]], mVerts[triData.v[1]],
                                     mVerts[triData.v[2]]};

        float u, v, w;
        barycentricXZ(to, verts[0], verts[1], verts[2], u, v, w);
        if (u >= -1e-4f && v >= -1e-4f && w >= -1e-4f)
            return true;

        s32 crossedEdge = -1;
        float bestT = kFloatMax;
        Math::vec3 bestPoint(0.0f);

        for (s32 e = 0; e < 3; ++e)
        {
            const Math::vec3& e0 = verts[e];
            const Math::vec3& e1 = verts[(e + 1) % 3];

            const float dx1 = to.x - pos.x, dz1 = to.z - pos.z;
            const float dx2 = e1.x - e0.x, dz2 = e1.z - e0.z;
            const float denom = dx1 * dz2 - dz1 * dx2;
            if (fabsf(denom) < 1e-9f)
                continue;

            const float ex = e0.x - pos.x, ez = e0.z - pos.z;
            const float t = (ex * dz2 - ez * dx2) / denom;
            const float sEdge = (ex * dz1 - ez * dx1) / denom;
            if (t < 1e-5f || t > 1.0f + 1e-5f || sEdge < -1e-5f || sEdge > 1.0f + 1e-5f)
                continue;
            if (t < bestT)
            {
                bestT = t;
                crossedEdge = e;
                bestPoint = Math::vec3(pos.x + dx1 * t, 0.0f, pos.z + dz1 * t);
            }
        }

        if (crossedEdge < 0)
        {
            // No edge crossed, yet the target is outside: pos is exactly on a vertex, where every
            // exit is at t == 0 and the crossing test rejects all of them. Step a hair along the
            // segment and pick up whichever triangle of the one-ring holds that point; if none
            // does, the segment really has left the surface here.
            {
                Math::vec3 toTarget = to - pos;
                toTarget.y = 0.0f;
                const float length = Math::length(toTarget);
                if (length > 1e-6f)
                {
                    const float nudge = Min(kBoundaryNudge, length * 0.5f);
                    const Math::vec3 probe = pos + toTarget * (nudge / length);

                    s32 ring[kVertexRingMax];
                    usize ringCount = 0;
                    collectVertexRing(tri, ring, kVertexRingMax, ringCount);

                    s32 best = -1;
                    float bestScore = -1e-3f;
                    for (usize r = 0; r < ringCount; ++r)
                    {
                        const s32 candidate = ring[r];
                        if (candidate == tri)
                            continue;
                        const NavTri& candidateTri = mTris[static_cast<usize>(candidate)];
                        float cu, cv, cw;
                        barycentricXZ(probe, mVerts[candidateTri.v[0]], mVerts[candidateTri.v[1]],
                                      mVerts[candidateTri.v[2]], cu, cv, cw);
                        const float score = Min(Min(cu, cv), cw);
                        if (score > bestScore)
                        {
                            bestScore = score;
                            best = candidate;
                        }
                    }
                    if (best >= 0)
                    {
                        pos = probe;
                        tri = best;
                        continue;
                    }
                }
            }

            // The segment leaves the surface here. Reporting "clear" would let a straight line be
            // declared walkable across a hole, so stop where the walk got to.
            const Math::vec3 full = to - from;
            const float fullLength = sqrtf(full.x * full.x + full.z * full.z);
            const Math::vec3 delta = pos - from;
            const float reached = sqrtf(delta.x * delta.x + delta.z * delta.z);
            outHitFraction = fullLength > 1e-6f ? reached / fullLength : 0.0f;
            outHitNormal = Math::vec3(0.0f);
            return true;
        }

        const s32 next = triData.neighbor[crossedEdge];
        if (next < 0 || !mTris[static_cast<usize>(next)].active)
        {
            const Math::vec3& e0 = verts[crossedEdge];
            const Math::vec3& e1 = verts[(crossedEdge + 1) % 3];
            const Math::vec3 edge = e1 - e0;
            Math::vec3 normal(-edge.z, 0.0f, edge.x);
            const float length = Math::length(normal);
            if (length > 1e-6f)
                normal = normal / length;
            // Facing back into the walkable side (the side the ray came from) whatever the
            // triangle's winding, matching distanceToWall()'s wall-to-point normal.
            const Math::vec3 centre = triCentroid(tri);
            if ((centre.x - e0.x) * normal.x + (centre.z - e0.z) * normal.z < 0.0f)
                normal = -normal;

            const Math::vec3 full = to - from;
            const float fullLength = sqrtf(full.x * full.x + full.z * full.z);
            const Math::vec3 hitDelta = bestPoint - from;
            const float hitLength = sqrtf(hitDelta.x * hitDelta.x + hitDelta.z * hitDelta.z);

            outHitFraction = fullLength > 1e-6f ? hitLength / fullLength : 0.0f;
            outHitNormal = normal;
            return true;
        }

        pos = bestPoint;
        tri = next;
    }

    // Out of steps without reaching `to` or a wall: reporting that as clear would mislead
    // visibility and smoothing callers, so it counts as a hit where the walk got to.
    const Math::vec3 full = to - from;
    const float fullLength = sqrtf(full.x * full.x + full.z * full.z);
    const Math::vec3 delta = pos - from;
    const float reached = sqrtf(delta.x * delta.x + delta.z * delta.z);
    outHitFraction = fullLength > 1e-6f ? Min(reached / fullLength, 0.99f) : 0.0f;
    outHitNormal = Math::vec3(0.0f);
    return true;
}

bool NavMesh::distanceToWall(const Math::vec3& point, f32 maxRadius, f32& outDistance,
                             Math::vec3& outWallNormal, const Math::vec3& searchExtents) const
{
    outDistance = maxRadius;
    outWallNormal = Math::vec3(0.0f);
    if (mTris.empty() || maxRadius <= 0.0f)
        return false;

    Math::vec3 closest;
    const s32 startTri = findContainingTri(point, searchExtents, &closest);
    if (startTri < 0)
        return false;

    // Flood outward by adjacency, stopping at triangles already further away than the best wall
    // found so far, so a big open floor costs the same as a small one.
    ct::Vector<s32> queue;
    ct::Vector<u8> visited(mTris.size(), 0);
    queue.push_back(startTri);
    visited[static_cast<usize>(startTri)] = 1;

    f32 best = maxRadius;
    for (usize head = 0; head < queue.size(); ++head)
    {
        const s32 tri = queue[head];
        const NavTri& triData = mTris[static_cast<usize>(tri)];
        const Math::vec3 verts[3] = {mVerts[triData.v[0]], mVerts[triData.v[1]],
                                     mVerts[triData.v[2]]};

        for (s32 e = 0; e < 3; ++e)
        {
            const s32 next = triData.neighbor[e];
            const Math::vec3& e0 = verts[e];
            const Math::vec3& e1 = verts[(e + 1) % 3];

            if (next < 0 || !mTris[static_cast<usize>(next)].active)
            {
                Math::vec3 edge = e1 - e0;
                edge.y = 0.0f;
                const float lengthSq = edge.x * edge.x + edge.z * edge.z;
                float t = 0.0f;
                if (lengthSq > 1e-9f)
                {
                    t = ((point.x - e0.x) * edge.x + (point.z - e0.z) * edge.z) / lengthSq;
                    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
                }
                const Math::vec3 onEdge = e0 + edge * t;
                const float dx = point.x - onEdge.x;
                const float dz = point.z - onEdge.z;
                const float distance = sqrtf(dx * dx + dz * dz);
                if (distance < best)
                {
                    best = distance;
                    Math::vec3 normal(dx, 0.0f, dz);
                    const float normalLength = Math::length(normal);
                    if (normalLength > 1e-6f)
                        outWallNormal = normal / normalLength;
                    else
                    {
                        // On the wall itself: the edge's normal, turned to face the walkable side.
                        Math::vec3 inward(-edge.z, 0.0f, edge.x);
                        const Math::vec3 centre = triCentroid(tri);
                        if ((centre.x - e0.x) * inward.x + (centre.z - e0.z) * inward.z < 0.0f)
                            inward = -inward;
                        const float inwardLength = Math::length(inward);
                        outWallNormal = inwardLength > 1e-6f ? inward / inwardLength
                                                             : Math::vec3(0.0f);
                    }
                }
                continue;
            }

            if (visited[static_cast<usize>(next)])
                continue;

            // Only worth visiting if part of it could still be nearer than the wall already found.
            const Math::vec3 centre = triCentroid(next);
            const float dx = point.x - centre.x;
            const float dz = point.z - centre.z;
            if (sqrtf(dx * dx + dz * dz) > best + maxRadius)
                continue;

            visited[static_cast<usize>(next)] = 1;
            queue.push_back(next);
        }
    }

    outDistance = best;
    return true;
}

bool NavMesh::randomPoint(u32& seed, Math::vec3& out) const
{
    if (mTris.empty())
        return false;

    auto nextRandom = [&seed]() {
        seed = seed * 1103515245u + 12345u;
        return f32((seed >> 16) & 0x7fffu) / f32(0x7fffu);
    };

    // Area-weighted: the first triangle whose running area passes a random share of the total.
    if (mAreaPrefix.size() != mTris.size() || mAreaPrefix.back() <= 0.0f)
        return false;
    const f32 target = nextRandom() * mAreaPrefix.back();
    usize lo = 0;
    usize hi = mAreaPrefix.size() - 1;
    while (lo < hi)
    {
        const usize mid = (lo + hi) / 2;
        if (mAreaPrefix[mid] < target)
            lo = mid + 1;
        else
            hi = mid;
    }
    // Skip the zero-area and pruned triangles that share the running total of the one before.
    while (lo + 1 < mAreaPrefix.size() &&
           (!mTris[lo].active || (lo > 0 ? mAreaPrefix[lo] - mAreaPrefix[lo - 1] : mAreaPrefix[lo]) <= 0.0f))
        ++lo;
    const s32 chosen = static_cast<s32>(lo);
    if (!mTris[lo].active)
        return false;

    const NavTri& tri = mTris[static_cast<usize>(chosen)];
    const Math::vec3& a = mVerts[tri.v[0]];
    const Math::vec3& b = mVerts[tri.v[1]];
    const Math::vec3& c = mVerts[tri.v[2]];

    // Uniform inside a triangle: fold the square onto it.
    f32 r1 = nextRandom();
    f32 r2 = nextRandom();
    if (r1 + r2 > 1.0f)
    {
        r1 = 1.0f - r1;
        r2 = 1.0f - r2;
    }
    out = a + (b - a) * r1 + (c - a) * r2;
    return true;
}

void NavMesh::smoothPath(ct::Vector<Math::vec3>& path, const Math::vec3& searchExtents) const
{
    if (path.size() < 3)
        return;

    ct::Vector<Math::vec3> result;
    result.push_back(path[0]);

    usize current = 0;
    while (current + 1 < path.size())
    {
        usize furthest = current + 1;
        for (usize candidate = path.size() - 1; candidate > current + 1; --candidate)
        {
            f32 fraction = 0.0f;
            Math::vec3 normal;
            if (raycast(path[current], path[candidate], fraction, normal, searchExtents) &&
                fraction >= 0.999f)
            {
                furthest = candidate;
                break;
            }
        }
        result.push_back(path[furthest]);
        current = furthest;
    }

    path = kinetix::Move(result);
}

const ct::Vector<Math::vec3>& NavMesh::debugTriangles() const
{
    return mDebugTriangles;
}

}
