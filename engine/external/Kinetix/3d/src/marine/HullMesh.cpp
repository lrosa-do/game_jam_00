#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "marine/HullMesh.h"

#include "geometry/ConvexHullComputer.h"

#include <ct/sort.hpp>

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;

bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

u64 edgeKey(u32 from, u32 to)
{
    return (static_cast<u64>(from) << 32) | static_cast<u64>(to);
}

// Appends the two triangles of quad a-b-c-d (counter-clockwise from outside).
void addQuad(ct::Vector<u32>& indices, u32 a, u32 b, u32 c, u32 d)
{
    indices.push_back(a);
    indices.push_back(b);
    indices.push_back(c);
    indices.push_back(a);
    indices.push_back(c);
    indices.push_back(d);
}

// Appends triangle a-b-c wound so its normal points along `outward`.
void addOriented(ct::Vector<u32>& indices, const ct::Vector<Math::vec3>& vertices, u32 a, u32 b,
                 u32 c, const Math::vec3& outward)
{
    const Math::vec3 n = Math::cross(vertices[b] - vertices[a], vertices[c] - vertices[a]);
    indices.push_back(a);
    if (Math::dot(n, outward) >= 0.0f)
    {
        indices.push_back(b);
        indices.push_back(c);
    }
    else
    {
        indices.push_back(c);
        indices.push_back(b);
    }
}
} // namespace

HullMesh::HullMesh(const Math::vec3* vertices, u32 vertexCount, const u32* indices, u32 indexCount)
{
    set(vertices, vertexCount, indices, indexCount);
}

bool HullMesh::set(const Math::vec3* vertices, u32 vertexCount, const u32* indices, u32 indexCount)
{
    mVertices.clear();
    mIndices.clear();
    if (!vertices || !indices || vertexCount < 3 || indexCount < 3)
    {
        Log::error("HullMesh: needs at least three vertices and one triangle");
        rebuild();
        return false;
    }
    mVertices.reserve(vertexCount);
    for (u32 i = 0; i < vertexCount; ++i)
    {
        if (!finiteVec(vertices[i]))
        {
            Log::error("HullMesh: vertex %u is not finite", i);
            mVertices.clear();
            rebuild();
            return false;
        }
        mVertices.push_back(vertices[i]);
    }
    if (indexCount % 3 != 0)
        Log::error("HullMesh: %u indices is not a whole number of triangles - the rest is dropped",
                   indexCount);
    u32 dropped = 0;
    mIndices.reserve(indexCount - indexCount % 3);
    for (u32 t = 0; t + 2 < indexCount; t += 3)
    {
        const u32 a = indices[t];
        const u32 b = indices[t + 1];
        const u32 c = indices[t + 2];
        if (a >= vertexCount || b >= vertexCount || c >= vertexCount || a == b || b == c || a == c)
        {
            ++dropped;
            continue;
        }
        mIndices.push_back(a);
        mIndices.push_back(b);
        mIndices.push_back(c);
    }
    if (dropped > 0)
        Log::error("HullMesh: dropped %u triangle(s) with an index out of range or repeated", dropped);
    rebuild();
    if (mVolume < 0.0f)
    {
        for (usize t = 0; t + 2 < mIndices.size(); t += 3)
            Swap(mIndices[t + 1], mIndices[t + 2]);
        rebuild();
    }
    return !mIndices.empty();
}

void HullMesh::rebuild()
{
    const u32 count = triangleCount();
    mNormals.clear();
    mAreas.clear();
    mNormals.reserve(count);
    mAreas.reserve(count);
    // Accumulated in double: signed tetrahedra of a hull of thousands of triangles cancel to a
    // small fraction of their magnitude.
    f64 volume = 0.0;
    f64 cx = 0.0;
    f64 cy = 0.0;
    f64 cz = 0.0;
    f64 area = 0.0;
    for (u32 t = 0; t < count; ++t)
    {
        const Math::vec3& a = mVertices[mIndices[3 * t]];
        const Math::vec3& b = mVertices[mIndices[3 * t + 1]];
        const Math::vec3& c = mVertices[mIndices[3 * t + 2]];
        const Math::vec3 cross = Math::cross(b - a, c - a);
        const f32 length = Math::length(cross);
        mAreas.push_back(0.5f * length);
        mNormals.push_back(length > 1.0e-20f ? cross / length : Math::vec3(0.0f));
        area += 0.5 * static_cast<f64>(length);
        // Signed tetrahedron with the origin (divergence theorem: volume and first moment).
        const f64 tetra = static_cast<f64>(Math::dot(a, Math::cross(b, c))) / 6.0;
        volume += tetra;
        cx += tetra * static_cast<f64>(a.x + b.x + c.x) * 0.25;
        cy += tetra * static_cast<f64>(a.y + b.y + c.y) * 0.25;
        cz += tetra * static_cast<f64>(a.z + b.z + c.z) * 0.25;
    }
    mVolume = static_cast<f32>(volume);
    mSurfaceArea = static_cast<f32>(area);
    if (fabs(volume) > 1.0e-18)
        mCentroid = Math::vec3(static_cast<f32>(cx / volume), static_cast<f32>(cy / volume),
                               static_cast<f32>(cz / volume));
    else
        mCentroid = Math::vec3(0.0f);

    if (mVertices.empty())
    {
        mBoundsMin = Math::vec3(0.0f);
        mBoundsMax = Math::vec3(0.0f);
        return;
    }
    mBoundsMin = mVertices[0];
    mBoundsMax = mVertices[0];
    for (const Math::vec3& v : mVertices)
    {
        mBoundsMin = Math::min(mBoundsMin, v);
        mBoundsMax = Math::max(mBoundsMax, v);
    }
}

void HullMesh::weldVertices(f32 tolerance)
{
    if (mVertices.empty())
        return;
    const f32 cell = Max(tolerance, 1.0e-9f);
    struct Keyed
    {
        s64 x, y, z;
        u32 index;
    };
    ct::Vector<Keyed> keyed;
    keyed.reserve(mVertices.size());
    for (u32 i = 0; i < mVertices.size(); ++i)
    {
        const Math::vec3& v = mVertices[i];
        Keyed k;
        k.x = static_cast<s64>(floor(static_cast<f64>(v.x) / cell + 0.5));
        k.y = static_cast<s64>(floor(static_cast<f64>(v.y) / cell + 0.5));
        k.z = static_cast<s64>(floor(static_cast<f64>(v.z) / cell + 0.5));
        k.index = i;
        keyed.push_back(k);
    }
    ct::sort(keyed.begin(), keyed.end(), [](const Keyed& l, const Keyed& r)
    {
        if (l.x != r.x)
            return l.x < r.x;
        if (l.y != r.y)
            return l.y < r.y;
        if (l.z != r.z)
            return l.z < r.z;
        return l.index < r.index;
    });
    ct::Vector<u32> remap;
    remap.resize(mVertices.size());
    ct::Vector<Math::vec3> welded;
    for (usize i = 0; i < keyed.size(); ++i)
    {
        const bool same = i > 0 && keyed[i].x == keyed[i - 1].x && keyed[i].y == keyed[i - 1].y &&
                          keyed[i].z == keyed[i - 1].z;
        if (!same)
            welded.push_back(mVertices[keyed[i].index]);
        remap[keyed[i].index] = static_cast<u32>(welded.size() - 1);
    }
    ct::Vector<u32> indices;
    indices.reserve(mIndices.size());
    for (usize t = 0; t + 2 < mIndices.size(); t += 3)
    {
        const u32 a = remap[mIndices[t]];
        const u32 b = remap[mIndices[t + 1]];
        const u32 c = remap[mIndices[t + 2]];
        if (a == b || b == c || a == c)
            continue;
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }
    mVertices = welded;
    mIndices = indices;
    rebuild();
}

void HullMesh::translate(const Math::vec3& offset)
{
    for (Math::vec3& v : mVertices)
        v += offset;
    rebuild();
}

void HullMesh::rotate(const Math::quat& rotation)
{
    const Math::quat q = Math::normalize(rotation);
    for (Math::vec3& v : mVertices)
        v = q * v;
    rebuild();
}

f32 HullMesh::extent(const Math::vec3& axis) const
{
    if (mVertices.empty() || !(Math::dot(axis, axis) > 1.0e-12f))
        return 0.0f;
    const Math::vec3 unit = Math::normalize(axis);
    f32 lo = Math::dot(mVertices[0], unit);
    f32 hi = lo;
    for (const Math::vec3& v : mVertices)
    {
        const f32 d = Math::dot(v, unit);
        lo = Min(lo, d);
        hi = Max(hi, d);
    }
    return hi - lo;
}

u32 HullMesh::openEdgeCount() const
{
    ct::Vector<u64> edges;
    edges.reserve(mIndices.size());
    for (usize t = 0; t + 2 < mIndices.size(); t += 3)
    {
        edges.push_back(edgeKey(mIndices[t], mIndices[t + 1]));
        edges.push_back(edgeKey(mIndices[t + 1], mIndices[t + 2]));
        edges.push_back(edgeKey(mIndices[t + 2], mIndices[t]));
    }
    ct::sort(edges.begin(), edges.end());
    u32 open = 0;
    for (usize i = 0; i < edges.size(); ++i)
    {
        if (i > 0 && edges[i] == edges[i - 1])
        {
            ++open;
            continue;
        }
        const u32 from = static_cast<u32>(edges[i] >> 32);
        const u32 to = static_cast<u32>(edges[i] & 0xFFFFFFFFu);
        const u64 reverse = edgeKey(to, from);
        const u64* found = LowerBound(edges.begin(), edges.end(), reverse);
        if (found == edges.end() || *found != reverse)
            ++open;
    }
    return open;
}

bool HullMesh::isClosed() const
{
    return !mIndices.empty() && openEdgeCount() == 0;
}

HullMesh HullMesh::box(const Math::vec3& halfExtents, u32 subdivisions)
{
    const u32 n = Clamp(subdivisions, 1u, 32u);
    const Math::vec3 h = Math::max(Math::abs(halfExtents), Math::vec3(1.0e-4f));
    ct::Vector<Math::vec3> vertices;
    ct::Vector<u32> indices;

    // Each face a grid in its own (u, v) axes, u x v along the outward normal; shared edges are
    // welded afterwards. Grid coordinates come from the same expression on every face, so the
    // duplicates are bitwise equal.
    struct Face
    {
        int normal;
        f32 sign;
        int u;
        int v;
    };
    const Face faces[6] = {
        {0, 1.0f, 1, 2}, {0, -1.0f, 2, 1}, {1, 1.0f, 2, 0},
        {1, -1.0f, 0, 2}, {2, 1.0f, 0, 1}, {2, -1.0f, 1, 0},
    };
    const f32 half[3] = {h.x, h.y, h.z};
    for (const Face& face : faces)
    {
        const u32 base = static_cast<u32>(vertices.size());
        for (u32 j = 0; j <= n; ++j)
        {
            for (u32 i = 0; i <= n; ++i)
            {
                f32 p[3];
                p[face.normal] = face.sign * half[face.normal];
                p[face.u] = -half[face.u] + 2.0f * half[face.u] * static_cast<f32>(i) / static_cast<f32>(n);
                p[face.v] = -half[face.v] + 2.0f * half[face.v] * static_cast<f32>(j) / static_cast<f32>(n);
                vertices.push_back(Math::vec3(p[0], p[1], p[2]));
            }
        }
        for (u32 j = 0; j < n; ++j)
        {
            for (u32 i = 0; i < n; ++i)
            {
                const u32 a = base + j * (n + 1) + i;
                const u32 b = a + 1;
                const u32 c = a + (n + 1) + 1;
                const u32 d = a + (n + 1);
                addQuad(indices, a, b, c, d);
            }
        }
    }
    HullMesh mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                  static_cast<u32>(indices.size()));
    mesh.weldVertices(1.0e-6f * Min(h.x, Min(h.y, h.z)));
    return mesh;
}

HullMesh HullMesh::sphere(f32 radius, u32 rings, u32 segments)
{
    const f32 r = Max(fabsf(radius), 1.0e-4f);
    const u32 bands = Clamp(rings, 2u, 128u);
    const u32 around = Clamp(segments, 3u, 256u);
    ct::Vector<Math::vec3> vertices;
    ct::Vector<u32> indices;
    vertices.push_back(Math::vec3(0.0f, r, 0.0f));
    for (u32 ring = 1; ring < bands; ++ring)
    {
        const f32 polar = kPi * static_cast<f32>(ring) / static_cast<f32>(bands);
        for (u32 s = 0; s < around; ++s)
        {
            const f32 azimuth = 2.0f * kPi * static_cast<f32>(s) / static_cast<f32>(around);
            vertices.push_back(Math::vec3(r * sinf(polar) * cosf(azimuth), r * cosf(polar),
                                          r * sinf(polar) * sinf(azimuth)));
        }
    }
    const u32 south = static_cast<u32>(vertices.size());
    vertices.push_back(Math::vec3(0.0f, -r, 0.0f));
    auto ringVertex = [&](u32 ring, u32 s)
    {
        return 1u + (ring - 1u) * around + (s % around);
    };
    for (u32 s = 0; s < around; ++s)
    {
        addOriented(indices, vertices, 0u, ringVertex(1, s), ringVertex(1, s + 1),
                    vertices[ringVertex(1, s)]);
        addOriented(indices, vertices, south, ringVertex(bands - 1, s), ringVertex(bands - 1, s + 1),
                    vertices[ringVertex(bands - 1, s)]);
    }
    for (u32 ring = 1; ring + 1 < bands; ++ring)
    {
        for (u32 s = 0; s < around; ++s)
        {
            const u32 a = ringVertex(ring, s);
            const u32 b = ringVertex(ring, s + 1);
            const u32 c = ringVertex(ring + 1, s + 1);
            const u32 d = ringVertex(ring + 1, s);
            const Math::vec3 outward = vertices[a] + vertices[c];
            addOriented(indices, vertices, a, b, c, outward);
            addOriented(indices, vertices, a, c, d, outward);
        }
    }
    return HullMesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                    static_cast<u32>(indices.size()));
}

HullMesh HullMesh::fromConvexHull(const kinetix::Geometry::ConvexHullComputer& hull)
{
    using Edge = kinetix::Geometry::ConvexHullComputer::Edge;
    ct::Vector<Math::vec3> vertices;
    vertices.reserve(hull.vertices.size());
    Math::vec3 centre(0.0f);
    for (const Math::vec3& v : hull.vertices)
    {
        vertices.push_back(v);
        centre += v;
    }
    if (vertices.empty())
        return HullMesh();
    centre /= static_cast<f32>(vertices.size());

    ct::Vector<u32> indices;
    ct::Vector<u32> ring;
    for (int faceEdge : hull.faces)
    {
        ring.clear();
        const Edge* first = &hull.edges[static_cast<usize>(faceEdge)];
        const Edge* edge = first;
        // A face of a convex hull has few edges; the bound only stops a broken half-edge structure
        // from looping forever.
        for (usize guard = 0; guard < hull.edges.size(); ++guard)
        {
            ring.push_back(static_cast<u32>(edge->getSourceVertex()));
            edge = edge->getNextEdgeOfFace();
            if (edge == first)
                break;
        }
        if (ring.size() < 3)
            continue;
        Math::vec3 faceCentre(0.0f);
        for (u32 v : ring)
            faceCentre += vertices[v];
        faceCentre /= static_cast<f32>(ring.size());
        // Convex: every face looks away from any interior point.
        const Math::vec3 outward = faceCentre - centre;
        for (usize i = 1; i + 1 < ring.size(); ++i)
            addOriented(indices, vertices, ring[0], ring[i], ring[i + 1], outward);
    }
    return HullMesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                    static_cast<u32>(indices.size()));
}

HullMesh HullMesh::boat(const BoatHullParameters& parameters)
{
    BoatHullParameters p = parameters;
    p.length = Max(p.length, 0.1f);
    p.beam = Max(p.beam, 0.05f);
    p.depth = Max(p.depth, 0.05f);
    p.transomBeamFraction = Clamp(p.transomBeamFraction, 0.05f, 1.0f);
    p.chineBeamFraction = Clamp(p.chineBeamFraction, 0.05f, 1.0f);
    p.deadrise = Clamp(p.deadrise, 0.0f, 1.3f);
    p.bowDeadrise = Clamp(p.bowDeadrise, 0.0f, 1.3f);
    p.bilgeRadius = Max(p.bilgeRadius, 0.0f);
    p.forefootRise = Clamp(p.forefootRise, 0.0f, 0.9f);
    p.forefootStart = Clamp(p.forefootStart, 0.0f, 0.95f);
    p.sheerRise = Max(p.sheerRise, 0.0f);
    const u32 stations = Clamp(p.stations, 3u, 128u);
    const u32 m = Clamp(p.sectionPoints, 2u, 64u);

    ct::Vector<Math::vec3> vertices;
    ct::Vector<u32> indices;
    // Per station, the first vertex: keel, then port 1..m, then starboard
    // 1..m (the stem has no starboard copies: both sides meet there).
    ct::Vector<u32> base;
    base.reserve(stations);

    // One side's section as a polyline from the keel (x = 0) to the sheer,
    // resampled to m + 1 points. Port side (+X); starboard is its mirror.
    Math::vec2 section[65];
    for (u32 i = 0; i < stations; ++i)
    {
        const f32 s = static_cast<f32>(i) / static_cast<f32>(stations - 1);
        const f32 z = -0.5f * p.length + s * p.length;

        f32 plan;
        if (s <= 0.5f)
        {
            const f32 t = s / 0.5f;
            plan = p.transomBeamFraction + (1.0f - p.transomBeamFraction) * (1.0f - (1.0f - t) * (1.0f - t));
        }
        else
        {
            const f32 t = (s - 0.5f) / 0.5f;
            plan = sqrtf(Max(1.0f - t * t, 0.0f));
        }
        const bool stem = i == stations - 1;
        const f32 halfBeam = stem ? 0.0f : 0.5f * p.beam * plan;
        f32 keel = 0.0f;
        if (s > p.forefootStart)
        {
            const f32 t = (s - p.forefootStart) / (1.0f - p.forefootStart);
            keel = p.depth * p.forefootRise * t * t;
        }
        const f32 sheer = p.depth + p.sheerRise * s * s;
        const f32 beta = Math::mix(p.deadrise, p.bowDeadrise, s);
        const f32 chineHalf = p.chineBeamFraction * halfBeam;
        const f32 chine = Min(keel + chineHalf * tanf(beta), keel + 0.8f * (sheer - keel));

        const Math::vec2 k(0.0f, keel);
        const Math::vec2 c(chineHalf, chine);
        const Math::vec2 top(halfBeam, sheer);
        const f32 bottomLength = Math::length(c - k);
        const f32 sideLength = Math::length(top - c);
        if (p.bilgeRadius <= 0.0f || bottomLength < 1.0e-5f || sideLength < 1.0e-5f)
        {
            // Hard chine: the chine is a vertex, bottom and topside stay two flat panels.
            const u32 bottomPoints = Clamp(static_cast<u32>(floorf(0.6f * static_cast<f32>(m) + 0.5f)), 1u, m - 1);
            for (u32 j = 0; j <= m; ++j)
            {
                if (j <= bottomPoints)
                    section[j] = Math::mix(k, c, static_cast<f32>(j) / static_cast<f32>(bottomPoints));
                else
                    section[j] = Math::mix(c, top, static_cast<f32>(j - bottomPoints) /
                                                       static_cast<f32>(m - bottomPoints));
            }
        }
        else
        {
            // Round bilge: the chine corner replaced by an arc tangent to both panels, the curve then
            // resampled evenly by arc length.
            const Math::vec2 into = (c - k) / bottomLength;
            const Math::vec2 out = (top - c) / sideLength;
            const f32 turn = acosf(Clamp(into.x * out.x + into.y * out.y, -1.0f, 1.0f));
            f32 tangent = p.bilgeRadius * tanf(0.5f * turn);
            tangent = Min(tangent, 0.45f * Min(bottomLength, sideLength));
            const f32 radius = turn > 1.0e-4f ? tangent / tanf(0.5f * turn) : 0.0f;
            constexpr u32 kArcPoints = 12;
            Math::vec2 curve[kArcPoints + 3];
            u32 count = 0;
            curve[count++] = k;
            const Math::vec2 start = c - into * tangent;
            // Centre on the inside of the turn (left of the travel direction
            // when it turns left, as it does from the bottom to the side).
            Math::vec2 inward(-into.y, into.x);
            if (inward.x * out.x + inward.y * out.y < 0.0f)
                inward = Math::vec2(-inward.x, -inward.y);
            const Math::vec2 centre = start + inward * radius;
            const f32 a0 = atan2f(start.y - centre.y, start.x - centre.x);
            const Math::vec2 end = c + out * tangent;
            f32 a1 = atan2f(end.y - centre.y, end.x - centre.x);
            while (a1 - a0 > kPi)
                a1 -= 2.0f * kPi;
            while (a0 - a1 > kPi)
                a1 += 2.0f * kPi;
            for (u32 a = 0; a <= kArcPoints; ++a)
            {
                const f32 angle = Math::mix(a0, a1, static_cast<f32>(a) / static_cast<f32>(kArcPoints));
                curve[count++] = centre + Math::vec2(cosf(angle), sinf(angle)) * radius;
            }
            curve[count++] = top;
            f32 lengths[kArcPoints + 3];
            lengths[0] = 0.0f;
            for (u32 q = 1; q < count; ++q)
                lengths[q] = lengths[q - 1] + Math::length(curve[q] - curve[q - 1]);
            const f32 total = Max(lengths[count - 1], 1.0e-6f);
            u32 segment = 1;
            for (u32 j = 0; j <= m; ++j)
            {
                const f32 target = total * static_cast<f32>(j) / static_cast<f32>(m);
                while (segment + 1 < count && lengths[segment] < target)
                    ++segment;
                const f32 span = Max(lengths[segment] - lengths[segment - 1], 1.0e-9f);
                const f32 t = Clamp((target - lengths[segment - 1]) / span, 0.0f, 1.0f);
                section[j] = Math::mix(curve[segment - 1], curve[segment], t);
            }
            section[0] = k;
            section[m] = top;
        }

        base.push_back(static_cast<u32>(vertices.size()));
        vertices.push_back(Math::vec3(0.0f, section[0].y, z));
        for (u32 j = 1; j <= m; ++j)
            vertices.push_back(Math::vec3(section[j].x, section[j].y, z));
        if (!stem)
            for (u32 j = 1; j <= m; ++j)
                vertices.push_back(Math::vec3(-section[j].x, section[j].y, z));
    }

    auto port = [&](u32 station, u32 j)
    {
        return j == 0 ? base[station] : base[station] + j;
    };
    auto starboard = [&](u32 station, u32 j)
    {
        if (j == 0)
            return base[station];
        return station == stations - 1 ? base[station] + j : base[station] + m + j;
    };

    for (u32 i = 0; i + 1 < stations; ++i)
    {
        for (u32 j = 0; j < m; ++j)
        {
            // Port (+X): normal out along +X, so a-d-c / a-c-b with d above a.
            const u32 a = port(i, j);
            const u32 b = port(i + 1, j);
            const u32 c = port(i + 1, j + 1);
            const u32 d = port(i, j + 1);
            const u32 sa = starboard(i, j);
            const u32 sb = starboard(i + 1, j);
            const u32 sc = starboard(i + 1, j + 1);
            const u32 sd = starboard(i, j + 1);
            if (i + 2 < stations)
            {
                addQuad(indices, a, d, c, b);
                addQuad(indices, sa, sb, sc, sd);
            }
            else
            {
                // Into the stem, where both sides share their vertices: cut
                // along the other diagonal, so each side's triangles meet
                // the stem line b-c and never each other.
                addQuad(indices, d, c, b, a);
                addQuad(indices, sb, sc, sd, sa);
            }
        }
        const u32 pa = port(i, m);
        const u32 pb = port(i + 1, m);
        const u32 sa = starboard(i, m);
        const u32 sb = starboard(i + 1, m);
        if (pb != sb)
        {
            indices.push_back(pa);
            indices.push_back(sb);
            indices.push_back(pb);
        }
        indices.push_back(pa);
        indices.push_back(sa);
        indices.push_back(sb);
    }

    {
        Math::vec3 centre(0.0f);
        ct::Vector<u32> loop;
        for (u32 j = 0; j <= m; ++j)
            loop.push_back(port(0, j));
        for (u32 j = m; j >= 1; --j)
            loop.push_back(starboard(0, j));
        for (u32 v : loop)
            centre += vertices[v];
        centre /= static_cast<f32>(loop.size());
        const u32 hub = static_cast<u32>(vertices.size());
        vertices.push_back(centre);
        const Math::vec3 aft(0.0f, 0.0f, -1.0f);
        for (usize q = 0; q < loop.size(); ++q)
            addOriented(indices, vertices, hub, loop[q], loop[(q + 1) % loop.size()], aft);
    }

    HullMesh mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                  static_cast<u32>(indices.size()));
    return mesh;
}

} // namespace kinetix::Physics
