#include "prisma3d/render/MeshSlice.h"

#include <ct/hashmap.hpp>

#include <math.h>

namespace prisma3d
{

namespace
{

constexpr float kOnPlane = 0.00001f;

float polygonArea(const ct::Vector<PolygonPoint>& polygon)
{
    const int n = static_cast<int>(polygon.size());
    float area = 0.0f;
    for (int p = n - 1, q = 0; q < n; p = q++)
        area += polygon[static_cast<std::size_t>(p)].x * polygon[static_cast<std::size_t>(q)].y -
                polygon[static_cast<std::size_t>(p)].y * polygon[static_cast<std::size_t>(q)].x;
    return area * 0.5f;
}

bool insideTriangle(float ax, float ay, float bx, float by, float cx, float cy, float px, float py,
        bool includeEdges)
{
    const float ex = cx - bx;
    const float ey = cy - by;
    const float fx = ax - cx;
    const float fy = ay - cy;
    const float gx = bx - ax;
    const float gy = by - ay;
    const float apx = px - ax;
    const float apy = py - ay;
    const float bpx = px - bx;
    const float bpy = py - by;
    const float cpx = px - cx;
    const float cpy = py - cy;
    const float aCrossBp = ex * bpy - ey * bpx;
    const float cCrossAp = gx * apy - gy * apx;
    const float bCrossCp = fx * cpy - fy * cpx;
    if (includeEdges) return aCrossBp > 0.0f && bCrossCp > 0.0f && cCrossAp > 0.0f;
    return aCrossBp >= 0.0f && bCrossCp >= 0.0f && cCrossAp >= 0.0f;
}

bool snip(const ct::Vector<PolygonPoint>& polygon, int u, int v, int w, int n,
        const ct::Vector<int>& order, bool relaxed)
{
    const PolygonPoint& a = polygon[static_cast<std::size_t>(order[static_cast<std::size_t>(u)])];
    const PolygonPoint& b = polygon[static_cast<std::size_t>(order[static_cast<std::size_t>(v)])];
    const PolygonPoint& c = polygon[static_cast<std::size_t>(order[static_cast<std::size_t>(w)])];
    const float threshold = relaxed ? -0.00001f : 0.00001f;
    if (threshold > ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x))) return false;
    for (int p = 0; p < n; ++p)
    {
        if (p == u || p == v || p == w) continue;
        const PolygonPoint& q =
                polygon[static_cast<std::size_t>(order[static_cast<std::size_t>(p)])];
        if (insideTriangle(a.x, a.y, b.x, b.y, c.x, c.y, q.x, q.y, relaxed)) return false;
    }
    return true;
}

// A corner of the surface with every attribute the surface carries.
struct Corner
{
    Math::Vec3 position;
    Math::Vec3 normal;
    Math::Vec4 tangent;
    Math::Vec4 color;
    Math::Vec2 uv;
    Math::Vec2 uv2;
};

Corner cornerOf(const SurfaceData& in, std::uint32_t index)
{
    Corner c;
    c.position = in.positions[index];
    c.normal = in.normals.empty() ? Math::Vec3(0.0f, 1.0f, 0.0f) : in.normals[index];
    c.tangent = in.tangents.empty() ? Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f) : in.tangents[index];
    c.color = in.colors.empty() ? Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f) : in.colors[index];
    c.uv = in.uvs.empty() ? Math::Vec2(0.0f, 0.0f) : in.uvs[index];
    c.uv2 = in.uvs2.empty() ? Math::Vec2(0.0f, 0.0f) : in.uvs2[index];
    return c;
}

Corner lerpCorner(const Corner& a, const Corner& b, float t)
{
    Corner c;
    c.position = a.position + (b.position - a.position) * t;
    c.normal = a.normal + (b.normal - a.normal) * t;
    const float length = c.normal.Length();
    if (length > 1e-6f) c.normal = c.normal * (1.0f / length);
    c.tangent = a.tangent + (b.tangent - a.tangent) * t;
    c.color = a.color + (b.color - a.color) * t;
    c.uv = a.uv + (b.uv - a.uv) * t;
    c.uv2 = a.uv2 + (b.uv2 - a.uv2) * t;
    return c;
}

std::uint32_t addCorner(const SurfaceData& in, const Corner& c, SurfaceData& out)
{
    const std::uint32_t index = static_cast<std::uint32_t>(out.positions.size());
    out.positions.push_back(c.position);
    if (!in.normals.empty()) out.normals.push_back(c.normal);
    if (!in.tangents.empty()) out.tangents.push_back(c.tangent);
    if (!in.colors.empty()) out.colors.push_back(c.color);
    if (!in.uvs.empty()) out.uvs.push_back(c.uv);
    if (!in.uvs2.empty()) out.uvs2.push_back(c.uv2);
    return index;
}

std::uint64_t positionKey(const Math::Vec3& p)
{
    const std::int64_t x = static_cast<std::int64_t>(floorf(p.x * 20000.0f + 0.5f));
    const std::int64_t y = static_cast<std::int64_t>(floorf(p.y * 20000.0f + 0.5f));
    const std::int64_t z = static_cast<std::int64_t>(floorf(p.z * 20000.0f + 0.5f));
    return (static_cast<std::uint64_t>(x & 0x1FFFFF) << 42) |
           (static_cast<std::uint64_t>(y & 0x1FFFFF) << 21) |
           static_cast<std::uint64_t>(z & 0x1FFFFF);
}

struct CutSegment
{
    Math::Vec3 from;
    Math::Vec3 to;
};

} // namespace

bool triangulatePolygon(const ct::Vector<PolygonPoint>& polygon, ct::Vector<int>& triangles)
{
    const int n = static_cast<int>(polygon.size());
    if (n < 3) return false;
    ct::Vector<int> order;
    order.resize(static_cast<std::size_t>(n));
    if (0.0f < polygonArea(polygon))
        for (int v = 0; v < n; ++v) order[static_cast<std::size_t>(v)] = v;
    else
        for (int v = 0; v < n; ++v) order[static_cast<std::size_t>(v)] = (n - 1) - v;
    bool relaxed = false;
    int nv = n;
    int count = 2 * nv;
    for (int v = nv - 1; nv > 2;)
    {
        if (0 >= (count--))
        {
            if (relaxed) return false;
            count = 2 * nv;
            relaxed = true;
        }
        int u = v;
        if (nv <= u) u = 0;
        v = u + 1;
        if (nv <= v) v = 0;
        int w = v + 1;
        if (nv <= w) w = 0;
        if (snip(polygon, u, v, w, nv, order, relaxed))
        {
            triangles.push_back(order[static_cast<std::size_t>(u)]);
            triangles.push_back(order[static_cast<std::size_t>(v)]);
            triangles.push_back(order[static_cast<std::size_t>(w)]);
            for (int a = v, b = v + 1; b < nv; ++a, ++b)
                order[static_cast<std::size_t>(a)] = order[static_cast<std::size_t>(b)];
            --nv;
            count = 2 * nv;
        }
    }
    return true;
}

namespace
{

// Closes a ring lying on the plane: ear clipping between its own corners, facing along the normal.
void addCap(const SurfaceData& in, const ct::Vector<Math::Vec3>& ring, const Math::Vec3& normal,
        const Math::Vec3& axisU, const Math::Vec3& axisV, SurfaceData& out)
{
    if (ring.size() < 3) return;
    ct::Vector<PolygonPoint> points;
    for (std::size_t i = 0; i < ring.size(); ++i)
    {
        PolygonPoint point;
        point.x = ring[i].Dot(axisU);
        point.y = ring[i].Dot(axisV);
        points.push_back(point);
    }
    ct::Vector<int> triangles;
    if (!triangulatePolygon(points, triangles)) return;
    const std::uint32_t first = static_cast<std::uint32_t>(out.positions.size());
    for (std::size_t i = 0; i < ring.size(); ++i)
    {
        Corner c;
        c.position = ring[i];
        c.normal = normal;
        c.tangent = Math::Vec4(axisU.x, axisU.y, axisU.z, 1.0f);
        c.color = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
        c.uv = Math::Vec2(points[i].x, points[i].y);
        c.uv2 = c.uv;
        addCorner(in, c, out);
    }
    for (std::size_t i = 0; i + 2 < triangles.size(); i += 3)
    {
        out.indices.push_back(first + static_cast<std::uint32_t>(triangles[i]));
        out.indices.push_back(first + static_cast<std::uint32_t>(triangles[i + 1]));
        out.indices.push_back(first + static_cast<std::uint32_t>(triangles[i + 2]));
    }
}

} // namespace

bool sliceSurface(const SurfaceData& in, const Math::Vec3& normal, float offset, SurfaceData& out,
        bool cap)
{
    out = SurfaceData();
    out.topology = in.topology;
    if (in.topology != prisma::Topology::Triangles || in.positions.empty()) return false;

    const std::size_t count = in.indices.empty() ? in.positions.size() : in.indices.size();
    ct::HashMap<std::uint64_t, std::uint32_t> kept;
    ct::HashMap<std::uint64_t, std::uint32_t> crossings;
    ct::Vector<CutSegment> segments;

    for (std::size_t t = 0; t + 2 < count; t += 3)
    {
        std::uint32_t source[3];
        float distance[3];
        int inside = 0;
        int outside = 0;
        for (int c = 0; c < 3; ++c)
        {
            source[c] = in.indices.empty() ? static_cast<std::uint32_t>(t + c)
                                           : in.indices[t + static_cast<std::size_t>(c)];
            distance[c] = normal.Dot(in.positions[source[c]]) + offset;
            if (distance[c] < -kOnPlane) ++inside;
            else if (distance[c] > kOnPlane)
                ++outside;
        }
        if (inside == 0) continue;

        // The triangle clipped by the plane, its corners in winding order: an original corner
        // (index into the input) or a corner on a cut edge (the two input corners and where).
        std::uint32_t polygon[4];
        bool onPlane[4];
        int corners = 0;
        for (int c = 0; c < 3; ++c)
        {
            const int previous = (c + 2) % 3;
            const bool out0 = distance[previous] > kOnPlane;
            const bool in0 = distance[previous] < -kOnPlane;
            const bool out1 = distance[c] > kOnPlane;
            const bool in1 = distance[c] < -kOnPlane;
            if ((out1 && in0) || (in1 && out0))
            {
                const std::uint32_t a = source[previous];
                const std::uint32_t b = source[c];
                const std::uint64_t key = a < b ? (static_cast<std::uint64_t>(a) << 32) | b
                                                : (static_cast<std::uint64_t>(b) << 32) | a;
                const std::uint32_t* found = crossings.find(key);
                std::uint32_t index = 0;
                if (found) index = *found;
                else
                {
                    const float s = distance[previous] / (distance[previous] - distance[c]);
                    index = addCorner(in, lerpCorner(cornerOf(in, a), cornerOf(in, b), s), out);
                    crossings[key] = index;
                }
                polygon[corners] = index;
                onPlane[corners] = true;
                ++corners;
            }
            if (!out1)
            {
                const std::uint32_t* found = kept.find(source[c]);
                std::uint32_t index = 0;
                if (found) index = *found;
                else
                {
                    index = addCorner(in, cornerOf(in, source[c]), out);
                    kept[source[c]] = index;
                }
                polygon[corners] = index;
                onPlane[corners] = !in1;
                ++corners;
            }
        }
        for (int c = 1; c + 1 < corners; ++c)
        {
            out.indices.push_back(polygon[0]);
            out.indices.push_back(polygon[c]);
            out.indices.push_back(polygon[c + 1]);
        }
        // The edge the plane opened: two corners in a row on the plane, of a triangle it cut.
        if (outside == 0) continue;
        for (int c = 0; c < corners; ++c)
        {
            const int next = (c + 1) % corners;
            if (!onPlane[c] || !onPlane[next]) continue;
            CutSegment segment;
            segment.from = out.positions[polygon[c]];
            segment.to = out.positions[polygon[next]];
            segments.push_back(segment);
        }
    }
    if (out.indices.empty()) return false;
    if (!cap || segments.empty()) return true;

    // The rings the cut opened: segments chained end to start, each closed by a cap on the plane
    // (an exact plane, so the ring is flat).
    const Math::Vec3 helper =
            fabsf(normal.x) < 0.9f ? Math::Vec3(1.0f, 0.0f, 0.0f) : Math::Vec3(0.0f, 1.0f, 0.0f);
    Math::Vec3 axisU = Math::Vec3::Cross(helper, normal);
    axisU = axisU * (1.0f / axisU.Length());
    const Math::Vec3 axisV = Math::Vec3::Cross(normal, axisU);
    ct::HashMap<std::uint64_t, std::uint32_t> startsAt;
    for (std::size_t i = 0; i < segments.size(); ++i)
        startsAt[positionKey(segments[i].from)] = static_cast<std::uint32_t>(i);
    ct::HashMap<std::uint64_t, std::uint32_t> endsAt;
    for (std::size_t i = 0; i < segments.size(); ++i)
        endsAt[positionKey(segments[i].to)] = static_cast<std::uint32_t>(i);
    ct::Vector<int> used;
    used.resize(segments.size());
    for (std::size_t i = 0; i < segments.size(); ++i) used[i] = 0;
    // A ring left open (the surface has a gap there) is walked from its first segment, so that it
    // comes out whole; the closed rings follow from any segment.
    ct::Vector<std::uint32_t> seeds;
    for (std::size_t i = 0; i < segments.size(); ++i)
        if (!endsAt.find(positionKey(segments[i].from)))
            seeds.push_back(static_cast<std::uint32_t>(i));
    for (std::size_t i = 0; i < segments.size(); ++i)
        seeds.push_back(static_cast<std::uint32_t>(i));
    ct::Vector<Math::Vec3> ring;
    ct::Vector<ct::Vector<Math::Vec3>> pieces;
    for (std::size_t s = 0; s < seeds.size(); ++s)
    {
        const std::size_t seed = seeds[s];
        if (used[seed]) continue;
        ring.clear();
        std::size_t at = seed;
        bool closed = false;
        for (std::size_t guard = 0; guard < segments.size(); ++guard)
        {
            used[at] = 1;
            if (ring.empty() || positionKey(ring.back()) != positionKey(segments[at].from))
                ring.push_back(segments[at].from);
            const std::uint32_t* next = startsAt.find(positionKey(segments[at].to));
            if (!next || used[*next])
            {
                closed = next != nullptr;
                if (!closed) ring.push_back(segments[at].to);
                break;
            }
            at = *next;
        }
        while (ring.size() > 1 && positionKey(ring.back()) == positionKey(ring[0])) ring.pop_back();
        if (closed) addCap(in, ring, normal, axisU, axisV, out);
        else
            pieces.push_back(ring);
    }

    // The pieces of a ring the surface leaves open, put back in a row: each one goes on from the
    // end of the last to the nearest start.
    if (pieces.empty()) return true;
    ring = pieces[0];
    ct::Vector<int> joined;
    joined.resize(pieces.size());
    for (std::size_t i = 0; i < pieces.size(); ++i) joined[i] = i == 0 ? 1 : 0;
    for (std::size_t round = 1; round < pieces.size(); ++round)
    {
        std::size_t nearest = 0;
        float best = 3.4e38f;
        for (std::size_t i = 0; i < pieces.size(); ++i)
        {
            if (joined[i]) continue;
            const float d = (pieces[i][0] - ring.back()).Length();
            if (d < best)
            {
                best = d;
                nearest = i;
            }
        }
        joined[nearest] = 1;
        for (std::size_t i = 0; i < pieces[nearest].size(); ++i) ring.push_back(pieces[nearest][i]);
    }
    addCap(in, ring, normal, axisU, axisV, out);
    return true;
}

} // namespace prisma3d
