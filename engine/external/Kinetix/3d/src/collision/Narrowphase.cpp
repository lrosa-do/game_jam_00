#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "collision/Narrowphase.h"

#include <float.h>
#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kEpsilon = 1e-6f;

// Overlap of the two shadows on `axis`, and which way the axis has to point
// to push B off A. Negative means a gap, which ends the test.
bool axisOverlap(const CollisionShape& a, const Math::mat4& transformA, const CollisionShape& b,
                 const Math::mat4& transformB, const Math::vec3& axis, f32 margin,
                 f32& penetration, Math::vec3& normal)
{
    const f32 length = Math::length(axis);
    if (length < kEpsilon)
        return true; // degenerate axis carries no information; not a separation

    const Math::vec3 unit = axis / length;
    f32 minA, maxA, minB, maxB;
    a.project(transformA, unit, minA, maxA);
    b.project(transformB, unit, minB, maxB);

    // overlapA is what moving B along -unit costs, overlapB along +unit.
    // Either going negative past the margin is a gap, which ends the whole test.
    const f32 overlapA = maxB - minA;
    const f32 overlapB = maxA - minB;
    if (overlapA < -margin || overlapB < -margin)
        return false;

    if (overlapA < overlapB)
    {
        penetration = overlapA;
        normal = -unit;
    }
    else
    {
        penetration = overlapB;
        normal = unit;
    }
    return true;
}

// Sutherland-Hodgman against one plane, keeping what is behind it. A convex
// polygon comes out at most one vertex longer than it went in, so `output`
// needs room for count + 1; every point that fits is written.
u32 clipPolygon(const Math::vec3* input, u32 count, const Math::vec3& planeNormal, f32 planeOffset,
                Math::vec3* output, u32 capacity)
{
    u32 written = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const Math::vec3& current = input[i];
        const Math::vec3& next = input[(i + 1) % count];
        const f32 distanceCurrent = Math::dot(planeNormal, current) - planeOffset;
        const f32 distanceNext = Math::dot(planeNormal, next) - planeOffset;

        if (distanceCurrent <= 0.0f && written < capacity)
            output[written++] = current;
        if (distanceCurrent * distanceNext < 0.0f && written < capacity)
        {
            const f32 t = distanceCurrent / (distanceCurrent - distanceNext);
            output[written++] = current + (next - current) * t;
        }
    }
    return written;
}

u32 incidentFace(const Math::mat4& transform, const Math::vec3& normal)
{
    u32 best = 0;
    f32 bestDot = 1.0e30f;
    for (u32 face = 0; face < 6; ++face)
    {
        const f32 value = Math::dot(BoxShape::faceNormal(transform, face), normal);
        if (value < bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    return best;
}

u32 reducePoints(const Math::vec3* points, const f32* depths, u32 count, ContactManifold& out);

void writePoint(ContactPoint& point, const Math::vec3& position, f32 penetration)
{
    point.position = position;
    point.penetration = penetration;
    point.normalImpulse = 0.0f;
    point.tangentImpulse[0] = 0.0f;
    point.tangentImpulse[1] = 0.0f;
}

// Outward normal of hull face `face`, world space; zero for a face with no
// area, which every caller skips.
Math::vec3 hullFaceNormal(const ConvexHullShape& hull, const Math::mat4& transform, u32 face)
{
    return Math::mat3(transform) * hull.faceNormals()[face];
}

bool hullFaceHasArea(const ConvexHullShape& hull, u32 face)
{
    const Math::vec3& normal = hull.faceNormals()[face];
    return Math::dot(normal, normal) > 0.5f;
}

// Every face normal of the hull in world space, zero ones included so the
// indices still match the hull's own faces.
void hullFaceNormals(const ConvexHullShape& hull, const Math::mat4& transform,
                     ct::Vector<Math::vec3>& out)
{
    const Math::mat3 rotation(transform);
    const ct::Vector<Math::vec3>& normals = hull.faceNormals();
    out.resize(normals.size());
    for (usize i = 0; i < normals.size(); ++i)
        out[i] = rotation * normals[i];
}

// Hull face whose outward normal is most opposed to `normal` (the rule
// incidentFace() uses for a box), or faceCount() when none has any area.
u32 hullIncidentFace(const ConvexHullShape& hull, const Math::mat4& transform,
                     const Math::vec3& normal)
{
    u32 best = hull.faceCount();
    f32 bestDot = 1.0e30f;
    const u32 faceCount = hull.faceCount();
    for (u32 face = 0; face < faceCount; ++face)
    {
        if (!hullFaceHasArea(hull, face))
            continue;
        const f32 value = Math::dot(hullFaceNormal(hull, transform, face), normal);
        if (value < bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    return best;
}

// Hull's own face whose outward normal agrees most with `normal`, or
// faceCount() when no face has any area.
u32 hullReferenceFace(const ConvexHullShape& hull, const Math::mat4& transform,
                      const Math::vec3& normal)
{
    u32 best = hull.faceCount();
    f32 bestDot = -1.0e30f;
    const u32 faceCount = hull.faceCount();
    for (u32 face = 0; face < faceCount; ++face)
    {
        if (!hullFaceHasArea(hull, face))
            continue;
        const f32 value = Math::dot(hullFaceNormal(hull, transform, face), normal);
        if (value > bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    return best;
}

// Face `face`'s corners in world space, in order: the polygon clipPolygon()
// clips and the source of its side planes. Empty for a face with no area or
// past the last face.
u32 hullFacePolygon(const ConvexHullShape& hull, const Math::mat4& transform, u32 face,
                    ct::Vector<Math::vec3>& out)
{
    out.clear();
    if (face >= hull.faceCount() || !hullFaceHasArea(hull, face))
        return 0;
    const u32 first = hull.faceStart()[face];
    const u32 last = hull.faceStart()[face + 1];
    const ct::Vector<u32>& corners = hull.faceVertices();
    // ConvexHullComputer's face loop winds the OPPOSITE way round from the
    // face's outward normal (checked against an interior point); walked
    // backwards it agrees with BoxShape's kFaces winding.
    for (u32 i = last; i > first; --i)
        out.push_back(Math::vec3(transform * Math::vec4(hull.vertices()[corners[i - 1]], 1.0f)));
    return static_cast<u32>(out.size());
}

// One world direction per set of parallel hull edges, unit length: stands in
// for a box's three edge directions when SAT needs a cross-product axis.
void hullEdgeDirections(const ConvexHullShape& hull, const Math::mat4& transform,
                        ct::Vector<Math::vec3>& out)
{
    const Math::mat3 rotation(transform);
    const ct::Vector<Math::vec3>& directions = hull.edgeDirections();
    out.resize(directions.size());
    for (usize i = 0; i < directions.size(); ++i)
        out[i] = rotation * directions[i];
}

Math::vec3 hullEdgeDirection(const ConvexHullShape& hull, const Math::mat4& transform, u32 edge)
{
    const ct::Vector<u32>& ends = hull.edgeVertices();
    return Math::mat3(transform) *
           (hull.vertices()[ends[2 * edge + 1]] - hull.vertices()[ends[2 * edge]]);
}

// Single contact at the midpoint of the two support points along `normal`:
// the fallback for a grazing face pair clipPolygon() clipped to nothing. NOT
// for an edge-edge axis, whose support corners can sit at opposite ends of
// the edges; edgeEdgeContact() is for that.
void supportPointContact(const CollisionShape& a, const Math::mat4& transformA,
                         const CollisionShape& b, const Math::mat4& transformB, f32 penetration,
                         ContactManifold& out)
{
    out.count = 1;
    const Math::vec3 pointA = a.support(transformA, out.normal);
    const Math::vec3 pointB = b.support(transformB, -out.normal);
    writePoint(out.points[0], (pointA + pointB) * 0.5f, penetration);
}

void edgeEdgeContact(const Math::vec3& startA, const Math::vec3& endA, const Math::vec3& startB,
                     const Math::vec3& endB, f32 penetration, ContactManifold& out)
{
    Math::vec3 closestA, closestB;
    closestPointsBetweenSegments(startA, endA, startB, endB, closestA, closestB);
    out.count = 1;
    writePoint(out.points[0], (closestA + closestB) * 0.5f, penetration);
}

// The box's edge along its local axis `axis` that is furthest along `normal`
// - the edge an edge-edge SAT axis was built from, found again. Corner i and
// corner i | (1 << axis) are its two ends.
void boxSupportingEdge(const Math::vec3 corners[8], u32 axis, const Math::vec3& normal,
                       Math::vec3& start, Math::vec3& end)
{
    f32 best = -FLT_MAX;
    for (u32 i = 0; i < 8; ++i)
    {
        if (i & (1u << axis))
            continue;
        const u32 j = i | (1u << axis);
        const f32 value = Math::dot(normal, corners[i] + corners[j]);
        if (value > best)
        {
            best = value;
            start = corners[i];
            end = corners[j];
        }
    }
}

// Of the hull's edges parallel to `direction`, the one furthest along
// `normal` (both world space): the edge an edge-edge SAT axis was built from.
bool hullSupportingEdge(const ConvexHullShape& hull, const Math::mat4& transform,
                        const Math::vec3& direction, const Math::vec3& normal, Math::vec3& start,
                        Math::vec3& end)
{
    const Math::mat3 inverse = Math::transpose(Math::mat3(transform));
    const Math::vec3 localDirection = Math::normalize(inverse * direction);
    const Math::vec3 localNormal = inverse * normal;
    const ct::Vector<u32>& edges = hull.edgeVertices();
    const ct::Vector<Math::vec3>& vertices = hull.vertices();
    f32 best = -FLT_MAX;
    bool found = false;
    for (usize i = 0; i + 1 < edges.size(); i += 2)
    {
        const Math::vec3& from = vertices[edges[i]];
        const Math::vec3& to = vertices[edges[i + 1]];
        const Math::vec3 delta = to - from;
        const f32 length = Math::length(delta);
        if (!(length > 1e-12f))
            continue;
        const Math::vec3 sine = Math::cross(delta / length, localDirection);
        if (Math::dot(sine, sine) > 1e-6f)
            continue;
        const f32 value = Math::dot(localNormal, from + to);
        if (value > best)
        {
            best = value;
            start = Math::vec3(transform * Math::vec4(from, 1.0f));
            end = Math::vec3(transform * Math::vec4(to, 1.0f));
            found = true;
        }
    }
    return found;
}

// The winner of a SAT run: the axis with the least overlap, the kind of
// feature pair that built it, and which ones.
struct SatAxis
{
    f32 penetration = FLT_MAX;
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    u32 kind = 0;
    u32 first = 0;
    u32 second = 0;
};

// How much shallower an edge or corner axis has to be to beat a face. An
// edge crossed with an edge parallel to a face IS that face's normal, give or
// take rounding, and winning on the rounding trades a clipped face patch for
// a single point - a box or a wheel lying flat then rocks on it.
constexpr f32 kFeatureAxisBias = 1e-5f;
// How far outside a reference face's side plane a clipped vertex may lie: a vertex on the
// plane is kept, not left to rounding.
constexpr f32 kClipTolerance = 1e-4f;

// Tests one axis and keeps it when it is the shallowest so far, by more than
// `bias`. False means the axis separates the shapes, which ends the whole
// test.
bool testAxis(const CollisionShape& a, const Math::mat4& transformA, const CollisionShape& b,
              const Math::mat4& transformB, const Math::vec3& axis, f32 margin, u32 kind, u32 first,
              u32 second, SatAxis& best, f32 bias = 0.0f)
{
    // A parallel edge pair or a face with no area gives a zero axis, which
    // carries no information and must not win with a stale zero.
    if (Math::length(axis) < kEpsilon)
        return true;
    f32 penetration = 0.0f;
    Math::vec3 normal(0.0f);
    if (!axisOverlap(a, transformA, b, transformB, axis, margin, penetration, normal))
        return false;
    if (penetration + bias < best.penetration)
    {
        best.penetration = penetration;
        best.normal = normal;
        best.kind = kind;
        best.first = first;
        best.second = second;
    }
    return true;
}

// Clips the incident polygon against the reference face's side planes and
// keeps what ends up behind the reference plane (the shape-agnostic half of
// boxBox()'s face case). With `windingKnown` the side planes come from the
// outward-CCW winding of kFaces and hullFacePolygon(); without it every
// plane is turned to put the polygon's own centre behind it, so a mesh
// triangle clips the same whichever way it was wound.
bool clipIncidentPolygon(const Math::vec3* referencePolygon, u32 referenceCount,
                         const Math::vec3& referenceNormal, const Math::vec3* incidentPolygon,
                         u32 incidentCount, f32 margin, bool windingKnown, ContactManifold& out)
{
    if (referenceCount < 3 || incidentCount < 3)
        return false;

    Math::vec3 interior(0.0f);
    for (u32 i = 0; i < referenceCount; ++i)
        interior += referencePolygon[i];
    interior /= static_cast<f32>(referenceCount);

    static thread_local ct::Vector<Math::vec3> polygon;
    static thread_local ct::Vector<Math::vec3> scratch;
    const u32 capacity = incidentCount + referenceCount;
    polygon.resize(capacity);
    scratch.resize(capacity);
    u32 count = incidentCount;
    for (u32 i = 0; i < count; ++i)
        polygon[i] = incidentPolygon[i];

    const f32 referenceOffset = Math::dot(referenceNormal, referencePolygon[0]);
    for (u32 i = 0; i < referenceCount && count > 0; ++i)
    {
        const Math::vec3& edgeStart = referencePolygon[i];
        const Math::vec3& edgeEnd = referencePolygon[(i + 1) % referenceCount];
        // cross(normal, edge), not cross(edge, normal): the other order
        // points the side plane into the face.
        const Math::vec3 planeNormal = Math::cross(referenceNormal, edgeEnd - edgeStart);
        const f32 planeLength = Math::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        Math::vec3 unit = planeNormal / planeLength;
        f32 offset = Math::dot(unit, edgeStart);
        if (!windingKnown && Math::dot(unit, interior) > offset)
        {
            unit = -unit;
            offset = -offset;
        }
        count = clipPolygon(polygon.data(), count, unit, offset + kClipTolerance, scratch.data(),
                            capacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    static thread_local ct::Vector<Math::vec3> kept;
    static thread_local ct::Vector<f32> depths;
    kept.clear();
    depths.clear();
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - Math::dot(referenceNormal, polygon[i]);
        if (depth < -margin)
            continue;
        kept.push_back(polygon[i] + referenceNormal * depth);
        depths.push_back(depth);
    }
    if (kept.empty())
        return false;

    out.count = reducePoints(kept.data(), depths.data(), static_cast<u32>(kept.size()), out);
    for (u32 i = 0; i < out.count; ++i)
    {
        out.points[i].normalImpulse = 0.0f;
        out.points[i].tangentImpulse[0] = 0.0f;
        out.points[i].tangentImpulse[1] = 0.0f;
    }
    return out.count > 0;
}

bool clipFaceAgainstFace(const Math::vec3* referencePolygon, u32 referenceCount,
                         const Math::vec3& referenceNormal, const Math::vec3* incidentPolygon,
                         u32 incidentCount, f32 margin, ContactManifold& out)
{
    return clipIncidentPolygon(referencePolygon, referenceCount, referenceNormal, incidentPolygon,
                               incidentCount, margin, true, out);
}

bool clipFaceAgainstPolygon(const Math::vec3* referencePolygon, u32 referenceCount,
                            const Math::vec3& referenceNormal, const Math::vec3* incidentPolygon,
                            u32 incidentCount, f32 margin, ContactManifold& out)
{
    return clipIncidentPolygon(referencePolygon, referenceCount, referenceNormal, incidentPolygon,
                               incidentCount, margin, false, out);
}

// The part of the segment [start, end] over a convex face: behind every side
// plane of `polygon`, each turned to keep the polygon's own centre so the
// face's winding does not matter. False when none of it is.
bool clipSegmentToFace(const Math::vec3* polygon, u32 count, const Math::vec3& normal,
                       Math::vec3& start, Math::vec3& end)
{
    if (count < 3)
        return false;
    Math::vec3 interior(0.0f);
    for (u32 i = 0; i < count; ++i)
        interior += polygon[i];
    interior /= static_cast<f32>(count);

    for (u32 i = 0; i < count; ++i)
    {
        const Math::vec3& edgeStart = polygon[i];
        const Math::vec3& edgeEnd = polygon[(i + 1) % count];
        const Math::vec3 planeNormal = Math::cross(normal, edgeEnd - edgeStart);
        const f32 planeLength = Math::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        Math::vec3 unit = planeNormal / planeLength;
        f32 offset = Math::dot(unit, edgeStart);
        if (Math::dot(unit, interior) > offset)
        {
            unit = -unit;
            offset = -offset;
        }
        const f32 distanceStart = Math::dot(unit, start) - offset;
        const f32 distanceEnd = Math::dot(unit, end) - offset;
        if (distanceStart > 0.0f && distanceEnd > 0.0f)
            return false;
        if (distanceStart > 0.0f)
            start = start + (end - start) * (distanceStart / (distanceStart - distanceEnd));
        else if (distanceEnd > 0.0f)
            end = end + (start - end) * (distanceEnd / (distanceEnd - distanceStart));
    }
    return true;
}

// A segment with `radius` round it lying along a face (a capsule on its
// side, or a cylinder's side line with radius zero). One point a radius below
// each end of the part over the face, so the pair sits on the surface;
// `faceNormal` points out of the face towards the segment. Returns the number
// of points written, 0 to 2.
u32 lyingContacts(const Math::vec3* polygon, u32 count, const Math::vec3& faceNormal,
                  const Math::vec3& segmentStart, const Math::vec3& segmentEnd, f32 radius,
                  f32 margin, ContactPoint out[2])
{
    Math::vec3 ends[2] = {segmentStart, segmentEnd};
    if (!clipSegmentToFace(polygon, count, faceNormal, ends[0], ends[1]))
        return 0;
    const u32 endCount = Math::length(ends[1] - ends[0]) > 1e-4f ? 2u : 1u;
    const f32 faceOffset = Math::dot(faceNormal, polygon[0]);
    u32 written = 0;
    for (u32 i = 0; i < endCount; ++i)
    {
        const Math::vec3 surface = ends[i] - faceNormal * radius;
        const f32 depth = faceOffset - Math::dot(faceNormal, surface);
        if (depth < -margin)
            continue;
        writePoint(out[written++], surface + faceNormal * (depth * 0.5f), depth);
    }
    // Deepest first, as reducePoints() orders every other manifold.
    if (written == 2 && out[1].penetration > out[0].penetration)
    {
        const ContactPoint deeper = out[1];
        out[1] = out[0];
        out[0] = deeper;
    }
    return written;
}

// A convex polytope's surface in its own space, for closest-point queries:
// corners, every face as a loop of corner indices with its outward normal
// (zero for no area), and edges for a polytope too thin to have a face with
// area. `solid` says the faces bound a volume.
struct PolytopeSurface
{
    const Math::vec3* vertices = nullptr;
    u32 vertexCount = 0;
    const u32* faceStart = nullptr;
    const u32* faceVertices = nullptr;
    const Math::vec3* faceNormals = nullptr;
    u32 faceCount = 0;
    const u32* edgeVertices = nullptr;
    u32 edgeCount = 0;
    bool solid = false;
};

PolytopeSurface hullSurface(const ConvexHullShape& hull)
{
    PolytopeSurface surface;
    surface.vertices = hull.vertices().data();
    surface.vertexCount = static_cast<u32>(hull.vertices().size());
    surface.faceStart = hull.faceStart().data();
    surface.faceVertices = hull.faceVertices().data();
    surface.faceNormals = hull.faceNormals().data();
    surface.faceCount = hull.faceCount();
    surface.edgeVertices = hull.edgeVertices().data();
    surface.edgeCount = static_cast<u32>(hull.edgeVertices().size() / 2);
    surface.solid = hull.solid();
    return surface;
}

struct BoxSurface
{
    Math::vec3 vertices[8];
    u32 faceStart[7];
    u32 faceVertices[24];
    Math::vec3 faceNormals[6];

    PolytopeSurface view() const
    {
        PolytopeSurface surface;
        surface.vertices = vertices;
        surface.vertexCount = 8;
        surface.faceStart = faceStart;
        surface.faceVertices = faceVertices;
        surface.faceNormals = faceNormals;
        surface.faceCount = 6;
        surface.solid = true;
        return surface;
    }
};

void buildBoxSurface(const BoxShape& box, BoxSurface& surface)
{
    const Math::vec3& half = box.halfExtents();
    for (u32 i = 0; i < 8; ++i)
        surface.vertices[i] = Math::vec3((i & 1) ? half.x : -half.x, (i & 2) ? half.y : -half.y,
                                         (i & 4) ? half.z : -half.z);
    for (u32 face = 0; face < 6; ++face)
    {
        surface.faceStart[face] = face * 4;
        const u8* corners = BoxShape::faceCorners(face);
        for (u32 i = 0; i < 4; ++i)
            surface.faceVertices[face * 4 + i] = corners[i];
        Math::vec3 normal(0.0f);
        normal[face / 2] = (face % 2) ? 1.0f : -1.0f;
        surface.faceNormals[face] = normal;
    }
    surface.faceStart[6] = 24;
}

bool hasArea(const Math::vec3& normal)
{
    return Math::dot(normal, normal) > 0.5f;
}

bool insidePolytope(const PolytopeSurface& surface, const Math::vec3& point)
{
    if (!surface.solid)
        return false;
    for (u32 face = 0; face < surface.faceCount; ++face)
    {
        const Math::vec3& normal = surface.faceNormals[face];
        if (!hasArea(normal))
            continue;
        const Math::vec3& corner = surface.vertices[surface.faceVertices[surface.faceStart[face]]];
        if (Math::dot(normal, point - corner) > 0.0f)
            return false;
    }
    return true;
}

// Closest point of the polytope to `point`, both in its own space, and the
// squared distance between them. Inside a solid polytope that is the point
// itself at distance zero, with `inside` set. Otherwise the closest point is
// on the surface: on a face's triangle fan when it has faces, on an edge or
// corner when it is only a segment or a point. FLT_MAX for an empty one.
f32 closestPointOnPolytope(const PolytopeSurface& surface, const Math::vec3& point,
                           Math::vec3& closest, bool& inside)
{
    inside = insidePolytope(surface, point);
    if (inside)
    {
        closest = point;
        return 0.0f;
    }

    f32 best = FLT_MAX;
    bool anyFace = false;
    for (u32 face = 0; face < surface.faceCount; ++face)
    {
        if (!hasArea(surface.faceNormals[face]))
            continue;
        anyFace = true;
        const u32 first = surface.faceStart[face];
        const u32 last = surface.faceStart[face + 1];
        const Math::vec3& apex = surface.vertices[surface.faceVertices[first]];
        for (u32 i = first + 1; i + 1 < last; ++i)
        {
            const Math::vec3 candidate =
                closestPointOnTriangle(apex, surface.vertices[surface.faceVertices[i]],
                                       surface.vertices[surface.faceVertices[i + 1]], point);
            const f32 squared = Math::dot(candidate - point, candidate - point);
            if (squared < best)
            {
                best = squared;
                closest = candidate;
            }
        }
    }
    if (anyFace)
        return best;

    for (u32 edge = 0; edge < surface.edgeCount; ++edge)
    {
        const Math::vec3 candidate =
            closestPointOnSegment(surface.vertices[surface.edgeVertices[edge * 2]],
                                  surface.vertices[surface.edgeVertices[edge * 2 + 1]], point);
        const f32 squared = Math::dot(candidate - point, candidate - point);
        if (squared < best)
        {
            best = squared;
            closest = candidate;
        }
    }
    if (surface.edgeCount > 0)
        return best;

    for (u32 i = 0; i < surface.vertexCount; ++i)
    {
        const f32 squared = Math::dot(surface.vertices[i] - point, surface.vertices[i] - point);
        if (squared < best)
        {
            best = squared;
            closest = surface.vertices[i];
        }
    }
    return best;
}

// closestPointOnPolytope() for the segment [start, end]. `inside` is set when
// the segment reaches into the solid; there is then no closest pair and the
// caller has to measure the overlap.
f32 closestPointsSegmentPolytope(const PolytopeSurface& surface, const Math::vec3& start,
                                 const Math::vec3& end, Math::vec3& onSegment,
                                 Math::vec3& onPolytope, bool& inside)
{
    inside = insidePolytope(surface, start);
    if (inside)
    {
        onSegment = start;
        onPolytope = start;
        return 0.0f;
    }

    f32 best = FLT_MAX;
    bool anyFace = false;
    for (u32 face = 0; face < surface.faceCount; ++face)
    {
        if (!hasArea(surface.faceNormals[face]))
            continue;
        anyFace = true;
        const u32 first = surface.faceStart[face];
        const u32 last = surface.faceStart[face + 1];
        const Math::vec3& apex = surface.vertices[surface.faceVertices[first]];
        for (u32 i = first + 1; i + 1 < last; ++i)
        {
            Math::vec3 segmentPoint, facePoint;
            const f32 squared = closestPointsSegmentTriangle(
                start, end, apex, surface.vertices[surface.faceVertices[i]],
                surface.vertices[surface.faceVertices[i + 1]], segmentPoint, facePoint);
            if (squared < best)
            {
                best = squared;
                onSegment = segmentPoint;
                onPolytope = facePoint;
            }
        }
    }
    if (!anyFace)
    {
        for (u32 edge = 0; edge < surface.edgeCount; ++edge)
        {
            Math::vec3 segmentPoint, edgePoint;
            closestPointsBetweenSegments(start, end,
                                         surface.vertices[surface.edgeVertices[edge * 2]],
                                         surface.vertices[surface.edgeVertices[edge * 2 + 1]],
                                         segmentPoint, edgePoint);
            const f32 squared = Math::dot(edgePoint - segmentPoint, edgePoint - segmentPoint);
            if (squared < best)
            {
                best = squared;
                onSegment = segmentPoint;
                onPolytope = edgePoint;
            }
        }
        if (surface.edgeCount == 0)
        {
            for (u32 i = 0; i < surface.vertexCount; ++i)
            {
                const Math::vec3 segmentPoint =
                    closestPointOnSegment(start, end, surface.vertices[i]);
                const f32 squared = Math::dot(surface.vertices[i] - segmentPoint,
                                              surface.vertices[i] - segmentPoint);
                if (squared < best)
                {
                    best = squared;
                    onSegment = segmentPoint;
                    onPolytope = surface.vertices[i];
                }
            }
        }
    }
    inside = best <= kEpsilon * kEpsilon;
    return best;
}

// Keeps the deepest point, then the three that are furthest from it and from
// each other, so the patch spans the real contact area instead of clustering
// in one corner. A manifold of four points all but touching is a manifold of
// one, and a box balanced on it wobbles.
u32 reducePoints(const Math::vec3* points, const f32* depths, u32 count, ContactManifold& out)
{
    if (count == 0)
        return 0;
    u32 deepest = 0;
    for (u32 i = 1; i < count; ++i)
        if (depths[i] > depths[deepest])
            deepest = i;

    if (count <= ContactManifold::MaxPoints)
    {
        // The deepest point goes first even when nothing is dropped, so
        // points[0] is always the deepest.
        out.points[0].position = points[deepest];
        out.points[0].penetration = depths[deepest];
        u32 written = 1;
        for (u32 i = 0; i < count; ++i)
        {
            if (i == deepest)
                continue;
            out.points[written].position = points[i];
            out.points[written].penetration = depths[i];
            ++written;
        }
        return written;
    }

    u32 chosen[ContactManifold::MaxPoints] = {0, 0, 0, 0};
    u32 chosenCount = 0;
    chosen[chosenCount++] = deepest;

    while (chosenCount < ContactManifold::MaxPoints)
    {
        u32 best = 0;
        f32 bestDistance = -1.0f;
        for (u32 i = 0; i < count; ++i)
        {
            bool already = false;
            for (u32 c = 0; c < chosenCount; ++c)
                if (chosen[c] == i)
                    already = true;
            if (already)
                continue;
            f32 nearest = 1.0e30f;
            for (u32 c = 0; c < chosenCount; ++c)
                nearest = Math::min(nearest, Math::length(points[i] - points[chosen[c]]));
            if (nearest > bestDistance)
            {
                bestDistance = nearest;
                best = i;
            }
        }
        if (bestDistance < 0.0f)
            break;
        chosen[chosenCount++] = best;
    }

    for (u32 i = 0; i < chosenCount; ++i)
    {
        out.points[i].position = points[chosen[i]];
        out.points[i].penetration = depths[chosen[i]];
    }
    return chosenCount;
}
} // namespace

void ContactManifold::buildTangents()
{
    const Math::vec3 reference = fabsf(normal.x) < 0.57735f ? Math::vec3(1.0f, 0.0f, 0.0f)
                                                              : Math::vec3(0.0f, 1.0f, 0.0f);
    tangent[0] = Math::normalize(Math::cross(reference, normal));
    tangent[1] = Math::cross(normal, tangent[0]);
}

bool Narrowphase::sphereSphere(const SphereShape& a, const Math::mat4& transformA,
                               const SphereShape& b, const Math::mat4& transformB,
                               ContactManifold& out, f32 margin)
{
    const Math::vec3 centerA(transformA[3]);
    const Math::vec3 centerB(transformB[3]);
    const Math::vec3 delta = centerB - centerA;
    const f32 distance = Math::length(delta);
    const f32 total = a.radius() + b.radius();
    if (distance >= total + margin)
        return false;

    out.normal = distance > kEpsilon ? delta / distance : Math::vec3(0.0f, 1.0f, 0.0f);
    out.buildTangents();
    out.count = 1;
    out.points[0].penetration = total - distance;
    out.points[0].position =
        centerA + out.normal * (a.radius() - out.points[0].penetration * 0.5f);
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::sphereBox(const SphereShape& a, const Math::mat4& transformA, const BoxShape& b,
                            const Math::mat4& transformB, ContactManifold& out, f32 margin)
{
    const Math::vec3 center(transformA[3]);
    const Math::mat3 rotation(transformB);
    const Math::vec3 boxCenter(transformB[3]);
    const Math::vec3 local = Math::transpose(rotation) * (center - boxCenter);
    const Math::vec3& half = b.halfExtents();

    const Math::vec3 closestLocal = Math::clamp(local, -half, half);
    const Math::vec3 offset = local - closestLocal;
    const f32 distanceSquared = Math::dot(offset, offset);
    const f32 reach = a.radius() + margin;
    if (distanceSquared > reach * reach)
        return false;

    Math::vec3 normalLocal;
    f32 penetration = 0.0f;
    if (distanceSquared > kEpsilon * kEpsilon)
    {
        const f32 distance = sqrtf(distanceSquared);
        normalLocal = offset / distance;
        penetration = a.radius() - distance;
    }
    else
    {
        // Centre inside the box: the closest face is the one it is least far
        // from, and the sphere has to come out through that one.
        const Math::vec3 depth = half - Math::abs(local);
        if (depth.x <= depth.y && depth.x <= depth.z)
        {
            normalLocal = Math::vec3(local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
            penetration = a.radius() + depth.x;
        }
        else if (depth.y <= depth.z)
        {
            normalLocal = Math::vec3(0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
            penetration = a.radius() + depth.y;
        }
        else
        {
            normalLocal = Math::vec3(0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f);
            penetration = a.radius() + depth.z;
        }
    }

    out.normal = -Math::normalize(rotation * normalLocal);
    out.buildTangents();
    out.count = 1;
    out.points[0].penetration = penetration;
    out.points[0].position = boxCenter + rotation * closestLocal;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::boxBox(const BoxShape& a, const Math::mat4& transformA, const BoxShape& b,
                         const Math::mat4& transformB, ContactManifold& out, f32 margin)
{
    // Fifteen axes: three face normals each, and the nine cross products of
    // their edge directions. Without the nine, two boxes meeting edge to edge
    // get a face normal that is not the real separating direction.
    const Math::mat3 rotationA(transformA);
    const Math::mat3 rotationB(transformB);

    Math::vec3 axes[15];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationA[i]));
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationB[i]));
    for (u32 i = 0; i < 3; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = Math::cross(Math::vec3(rotationA[i]), Math::vec3(rotationB[j]));

    f32 bestPenetration = 1.0e30f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        // A parallel edge pair gives a zero cross product, which axisOverlap()
        // reports as "no information".
        if (Math::length(axes[i]) < kEpsilon)
            continue;
        // B's faces and the edges must beat A's faces by more than rounding.
        if (penetration + (i < 3 ? 0.0f : kFeatureAxisBias) < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
            bestAxis = i;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    // An edge-edge separation has no face to clip: the contact is the single
    // point where the two edges are closest. The axis says which edge
    // direction of each box it came from; the one furthest towards the other
    // box touches it.
    if (bestAxis >= 6)
    {
        Math::vec3 cornersA[8];
        Math::vec3 cornersB[8];
        a.corners(transformA, cornersA);
        b.corners(transformB, cornersB);
        Math::vec3 startA, endA, startB, endB;
        boxSupportingEdge(cornersA, (bestAxis - 6) / 3, out.normal, startA, endA);
        boxSupportingEdge(cornersB, (bestAxis - 6) % 3, -out.normal, startB, endB);
        edgeEdgeContact(startA, endA, startB, endB, bestPenetration, out);
        return true;
    }

    const bool referenceIsA = bestAxis < 3;
    const BoxShape& reference = referenceIsA ? a : b;
    const BoxShape& incident = referenceIsA ? b : a;
    const Math::mat4& referenceTransform = referenceIsA ? transformA : transformB;
    const Math::mat4& incidentTransform = referenceIsA ? transformB : transformA;
    const Math::vec3 referenceDirection = referenceIsA ? out.normal : -out.normal;

    u32 referenceFace = 0;
    f32 bestDot = -1.0e30f;
    for (u32 face = 0; face < 6; ++face)
    {
        const f32 value = Math::dot(BoxShape::faceNormal(referenceTransform, face),
                                   referenceDirection);
        if (value > bestDot)
        {
            bestDot = value;
            referenceFace = face;
        }
    }

    const Math::vec3 referenceNormal = BoxShape::faceNormal(referenceTransform, referenceFace);
    Math::vec3 referenceCorner[8];
    reference.corners(referenceTransform, referenceCorner);
    const u8* referenceIndices = BoxShape::faceCorners(referenceFace);
    const f32 referenceOffset = Math::dot(referenceNormal, referenceCorner[referenceIndices[0]]);

    const u32 incidentFaceIndex = incidentFace(incidentTransform, referenceNormal);
    Math::vec3 incidentCorner[8];
    incident.corners(incidentTransform, incidentCorner);
    const u8* incidentIndices = BoxShape::faceCorners(incidentFaceIndex);

    constexpr u32 capacity = 16;
    Math::vec3 polygon[capacity];
    Math::vec3 scratch[capacity];
    u32 count = 4;
    for (u32 i = 0; i < 4; ++i)
        polygon[i] = incidentCorner[incidentIndices[i]];

    for (u32 i = 0; i < 4 && count > 0; ++i)
    {
        const Math::vec3& edgeStart = referenceCorner[referenceIndices[i]];
        const Math::vec3& edgeEnd = referenceCorner[referenceIndices[(i + 1) % 4]];
        const Math::vec3 edge = edgeEnd - edgeStart;
        // cross(normal, edge), not cross(edge, normal): with the winding in
        // kFaces the latter points INTO the face and clips the whole incident
        // polygon away.
        const Math::vec3 planeNormal = Math::cross(referenceNormal, edge);
        const f32 planeLength = Math::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        const Math::vec3 unit = planeNormal / planeLength;
        count = clipPolygon(polygon, count, unit, Math::dot(unit, edgeStart) + kClipTolerance, scratch,
                            capacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    Math::vec3 kept[capacity];
    f32 depths[capacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - Math::dot(referenceNormal, polygon[i]);
        // Negative depth is a point in front of the reference face. Within
        // the margin it is kept as a speculative contact; past it, dropped.
        if (depth < -margin)
            continue;
        // Reported on the reference face, the surface the solver pushes along.
        kept[keptCount] = polygon[i] + referenceNormal * depth;
        depths[keptCount] = depth;
        ++keptCount;
    }

    if (keptCount == 0)
    {
        // Clipping produced nothing (a grazing face pair): the axis test proved
        // they overlap, so fall back to the support points.
        supportPointContact(a, transformA, b, transformB, bestPenetration, out);
        return true;
    }

    out.count = reducePoints(kept, depths, keptCount, out);
    for (u32 i = 0; i < out.count; ++i)
    {
        out.points[i].normalImpulse = 0.0f;
        out.points[i].tangentImpulse[0] = 0.0f;
        out.points[i].tangentImpulse[1] = 0.0f;
    }
    return true;
}

namespace
{
// Two spheres of the given radii at the given centres, written into the
// manifold; capsule contacts reduce to this.
bool sphereContact(const Math::vec3& centerA, f32 radiusA, const Math::vec3& centerB, f32 radiusB,
                   f32 margin, ContactManifold& out)
{
    const Math::vec3 delta = centerB - centerA;
    const f32 distance = Math::length(delta);
    const f32 total = radiusA + radiusB;
    if (distance >= total + margin)
        return false;

    out.normal = distance > kEpsilon ? delta / distance : Math::vec3(0.0f, 1.0f, 0.0f);
    out.buildTangents();
    out.count = 1;
    out.points[0].penetration = total - distance;
    out.points[0].position = centerA + out.normal * (radiusA - out.points[0].penetration * 0.5f);
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}
} // namespace

bool Narrowphase::capsuleSphere(const CapsuleShape& a, const Math::mat4& transformA,
                                const SphereShape& b, const Math::mat4& transformB,
                                ContactManifold& out, f32 margin)
{
    Math::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const Math::vec3 center(transformB[3]);
    const Math::vec3 closest = closestPointOnSegment(lower, upper, center);
    return sphereContact(closest, a.radius(), center, b.radius(), margin, out);
}

bool Narrowphase::capsuleCapsule(const CapsuleShape& a, const Math::mat4& transformA,
                                 const CapsuleShape& b, const Math::mat4& transformB,
                                 ContactManifold& out, f32 margin)
{
    Math::vec3 lowerA, upperA, lowerB, upperB;
    a.segment(transformA, lowerA, upperA);
    b.segment(transformB, lowerB, upperB);
    Math::vec3 closestA, closestB;
    closestPointsBetweenSegments(lowerA, upperA, lowerB, upperB, closestA, closestB);
    return sphereContact(closestA, a.radius(), closestB, b.radius(), margin, out);
}

namespace
{
// A capsule against a convex polytope, the normal running from the polytope
// to the capsule. While the capsule's segment stays OUTSIDE the polytope the
// exact answer is the closest pair between the two: its direction is the
// normal and the radius minus its length the depth, whatever feature it lands
// on. SAT over face normals and edge crosses cannot see an end cap meeting a
// corner or an edge.
//
// Once the segment reaches the polytope there is no closest pair, and
// `penetrating` tells the caller to measure the overlap with SAT instead,
// exact over face normals and segment-edge crosses for a segment inside.
struct CapsulePolytopeDistance
{
    bool penetrating = false;
    bool separated = false;
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    f32 penetration = 0.0f;
    Math::vec3 position{0.0f};
};

CapsulePolytopeDistance capsulePolytopeDistance(const PolytopeSurface& surface,
                                                const Math::mat4& polytopeTransform,
                                                const Math::vec3& lower, const Math::vec3& upper,
                                                f32 radius, f32 margin)
{
    CapsulePolytopeDistance result;
    const Math::mat3 rotation(polytopeTransform);
    const Math::mat3 inverse = Math::transpose(rotation);
    const Math::vec3 centre(polytopeTransform[3]);
    Math::vec3 onSegment, onPolytope;
    bool inside = false;
    const f32 squared = closestPointsSegmentPolytope(surface, inverse * (lower - centre),
                                                     inverse * (upper - centre), onSegment,
                                                     onPolytope, inside);
    if (squared == FLT_MAX)
    {
        result.separated = true;
        return result;
    }
    if (inside)
    {
        result.penetrating = true;
        return result;
    }
    const f32 distance = sqrtf(squared);
    if (distance > radius + margin)
    {
        result.separated = true;
        return result;
    }
    result.normal = rotation * ((onSegment - onPolytope) / distance);
    result.penetration = radius - distance;
    // Midway between the polytope's surface and the capsule's (its segment
    // pushed a radius back towards the polytope).
    const Math::vec3 polytopePoint = centre + rotation * onPolytope;
    const Math::vec3 capsulePoint = centre + rotation * onSegment - result.normal * radius;
    result.position = (polytopePoint + capsulePoint) * 0.5f;
    return result;
}

// The face of `surface` whose world normal agrees with `normal` (polytope to
// capsule) to within a few degrees, its corners in world space, or false
// when the normal is not a face's - an edge or a corner has no face for a
// capsule to lie along.
bool alignedFace(const PolytopeSurface& surface, const Math::mat4& transform,
                 const Math::vec3& normal, ct::Vector<Math::vec3>& polygon, Math::vec3& faceNormal)
{
    const Math::mat3 rotation(transform);
    u32 best = surface.faceCount;
    f32 bestDot = 0.995f;
    for (u32 face = 0; face < surface.faceCount; ++face)
    {
        if (!hasArea(surface.faceNormals[face]))
            continue;
        const f32 value = Math::dot(rotation * surface.faceNormals[face], normal);
        if (value > bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    if (best == surface.faceCount)
        return false;
    faceNormal = rotation * surface.faceNormals[best];
    polygon.clear();
    for (u32 i = surface.faceStart[best]; i < surface.faceStart[best + 1]; ++i)
        polygon.push_back(
            Math::vec3(transform * Math::vec4(surface.vertices[surface.faceVertices[i]], 1.0f)));
    return true;
}

// How far the deepest of a lying pair may fall short of the pair's own
// depth before the pair is not describing the real contact.
constexpr f32 kLyingDepthTolerance = 1e-3f;

// Lying along a face: both segment ends are in contact, and one point would
// let the capsule pivot. The part of the segment over the face, clipped to
// it, gives the pair; false when it does not give two, or when the deeper of
// them is not as deep as `penetration` (the capsule is tipped over the
// face's edge and the edge is the contact).
bool capsuleLyingOnFace(const PolytopeSurface& surface, const Math::mat4& transform,
                        const Math::vec3& lower, const Math::vec3& upper, f32 radius,
                        const Math::vec3& normal, f32 penetration, f32 margin, ContactPoint out[2])
{
    const Math::vec3 axis = upper - lower;
    const f32 length = Math::length(axis);
    if (length < kEpsilon || fabsf(Math::dot(axis / length, normal)) >= 0.05f)
        return false;
    static thread_local ct::Vector<Math::vec3> polygon;
    Math::vec3 faceNormal;
    if (!alignedFace(surface, transform, normal, polygon, faceNormal))
        return false;
    return lyingContacts(polygon.data(), static_cast<u32>(polygon.size()), faceNormal, lower,
                         upper, radius, margin, out) == 2 &&
           out[0].penetration >= penetration - kLyingDepthTolerance;
}
} // namespace

bool Narrowphase::capsuleBox(const CapsuleShape& a, const Math::mat4& transformA, const BoxShape& b,
                             const Math::mat4& transformB, ContactManifold& out, f32 margin)
{
    Math::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const Math::mat3 rotationB(transformB);
    const Math::vec3 capsuleAxis = Math::normalize(Math::vec3(transformA[1]));

    BoxSurface box;
    buildBoxSurface(b, box);
    const PolytopeSurface surface = box.view();
    const CapsulePolytopeDistance distance =
        capsulePolytopeDistance(surface, transformB, lower, upper, a.radius(), margin);
    if (distance.separated)
        return false;

    // The manifold's normal runs from A, the capsule, to the box.
    f32 bestPenetration = distance.penetration;
    Math::vec3 bestNormal = -distance.normal;
    if (distance.penetrating)
    {
        // The box's three faces, the capsule's axis, and the three cross
        // products between them; the last catch a capsule lying diagonally
        // across an edge.
        Math::vec3 axes[7];
        u32 axisCount = 0;
        for (u32 i = 0; i < 3; ++i)
            axes[axisCount++] = Math::normalize(Math::vec3(rotationB[i]));
        axes[axisCount++] = capsuleAxis;
        for (u32 i = 0; i < 3; ++i)
            axes[axisCount++] = Math::cross(capsuleAxis, Math::vec3(rotationB[i]));

        SatAxis best;
        for (u32 i = 0; i < axisCount; ++i)
            if (!testAxis(a, transformA, b, transformB, axes[i], margin, 0, i, 0, best))
                return false;
        bestPenetration = best.penetration;
        bestNormal = best.normal;
    }

    out.normal = bestNormal;
    out.buildTangents();

    ContactPoint lying[2];
    if (capsuleLyingOnFace(surface, transformB, lower, upper, a.radius(), -out.normal,
                           bestPenetration, margin, lying))
    {
        out.count = 2;
        out.points[0] = lying[0];
        out.points[1] = lying[1];
        return true;
    }

    out.count = 1;
    if (!distance.penetrating)
    {
        writePoint(out.points[0], distance.position, bestPenetration);
        return true;
    }

    // Overlapping: the closest point on the segment to the box, pushed out
    // by the radius.
    const Math::vec3 boxCenter(transformB[3]);
    const Math::vec3 nearSegment = closestPointOnSegment(lower, upper, boxCenter);
    const Math::vec3 localNear = Math::transpose(rotationB) * (nearSegment - boxCenter);
    const Math::vec3 onBox =
        boxCenter + rotationB * Math::clamp(localNear, -b.halfExtents(), b.halfExtents());
    const Math::vec3 refined = closestPointOnSegment(lower, upper, onBox);
    // Midway between the box's surface and the capsule's (its segment pushed
    // a radius along the normal, towards the box).
    writePoint(out.points[0], (onBox + (refined + out.normal * a.radius())) * 0.5f,
               bestPenetration);
    return true;
}

bool Narrowphase::convexHullSphere(const ConvexHullShape& a, const Math::mat4& transformA,
                                   const SphereShape& b, const Math::mat4& transformB,
                                   ContactManifold& out, f32 margin)
{
    // sphereBox()'s rule on the hull's faces. Centre outside: the closest
    // point of the hull decides - the face normal over a face, the direction
    // to the edge or corner past one. Centre inside: out through the face it
    // is least far behind.
    const Math::mat3 rotation(transformA);
    const Math::vec3 hullCentre(transformA[3]);
    const Math::vec3 sphereCentre(transformB[3]);
    const Math::vec3 local = Math::transpose(rotation) * (sphereCentre - hullCentre);
    const PolytopeSurface surface = hullSurface(a);

    Math::vec3 closest;
    bool inside = false;
    const f32 squared = closestPointOnPolytope(surface, local, closest, inside);
    if (squared == FLT_MAX)
        return false;

    const f32 reach = b.radius() + margin;
    if (squared > reach * reach)
        return false;

    Math::vec3 normalLocal(0.0f, 1.0f, 0.0f);
    f32 penetration = 0.0f;
    if (squared > kEpsilon * kEpsilon)
    {
        const f32 distance = sqrtf(squared);
        normalLocal = (local - closest) / distance;
        penetration = b.radius() - distance;
    }
    else
    {
        // Inside, or on a surface with no outside of its own (a flat or
        // thinner hull): the face the centre is least far behind. A hull with
        // no face at all keeps the default.
        f32 shallowest = -FLT_MAX;
        for (u32 face = 0; face < a.faceCount(); ++face)
        {
            const Math::vec3& normal = a.faceNormals()[face];
            if (!hasArea(normal))
                continue;
            const Math::vec3& corner =
                a.vertices()[a.faceVertices()[a.faceStart()[face]]];
            const f32 height = Math::dot(normal, local - corner);
            if (height > shallowest)
            {
                shallowest = height;
                normalLocal = normal;
            }
        }
        penetration = b.radius() - Math::min(shallowest, 0.0f);
        if (shallowest == -FLT_MAX)
            penetration = b.radius();
    }

    out.normal = Math::normalize(rotation * normalLocal);
    out.buildTangents();

    // The sphere's deepest point, then half the overlap along the normal: on
    // the line through the centre, so a sphere landed square on a face is not
    // spun by a contact at one of its corners.
    const Math::vec3 deepest = sphereCentre - out.normal * b.radius();
    out.count = 1;
    writePoint(out.points[0], deepest + out.normal * (penetration * 0.5f), penetration);
    return true;
}

bool Narrowphase::convexHullCapsule(const ConvexHullShape& a, const Math::mat4& transformA,
                                    const CapsuleShape& b, const Math::mat4& transformB,
                                    ContactManifold& out, f32 margin)
{
    if (a.vertices().empty())
        return false;
    Math::vec3 lower, upper;
    b.segment(transformB, lower, upper);
    const Math::vec3 capsuleAxis = Math::normalize(Math::vec3(transformB[1]));

    const PolytopeSurface surface = hullSurface(a);
    const CapsulePolytopeDistance distance =
        capsulePolytopeDistance(surface, transformA, lower, upper, b.radius(), margin);
    if (distance.separated)
        return false;

    f32 bestPenetration = distance.penetration;
    Math::vec3 bestNormal = distance.normal;
    if (distance.penetrating)
    {
        // The hull's face normals, the capsule's axis, and that axis crossed
        // with every hull edge direction (capsuleBox() generalized).
        static thread_local ct::Vector<Math::vec3> hullNormals;
        static thread_local ct::Vector<Math::vec3> hullEdges;
        hullFaceNormals(a, transformA, hullNormals);
        hullEdgeDirections(a, transformA, hullEdges);

        SatAxis best;
        for (u32 face = 0; face < hullNormals.size(); ++face)
            if (!testAxis(a, transformA, b, transformB, hullNormals[face], margin, 0, face, 0,
                          best))
                return false;
        if (!testAxis(a, transformA, b, transformB, capsuleAxis, margin, 1, 0, 0, best))
            return false;
        for (u32 i = 0; i < hullEdges.size(); ++i)
            if (!testAxis(a, transformA, b, transformB, Math::cross(capsuleAxis, hullEdges[i]),
                          margin, 2, i, 0, best))
                return false;
        bestPenetration = best.penetration;
        bestNormal = best.normal;
    }

    out.normal = bestNormal;
    out.buildTangents();

    ContactPoint lying[2];
    if (capsuleLyingOnFace(surface, transformA, lower, upper, b.radius(), out.normal,
                           bestPenetration, margin, lying))
    {
        out.count = 2;
        out.points[0] = lying[0];
        out.points[1] = lying[1];
        return true;
    }

    if (!distance.penetrating)
    {
        out.count = 1;
        writePoint(out.points[0], distance.position, bestPenetration);
        return true;
    }

    // Overlapping and not lying flat: the support points along the
    // separating normal, the fallback boxBox() takes with no face to clip.
    supportPointContact(a, transformA, b, transformB, bestPenetration, out);
    return true;
}

bool Narrowphase::convexHullBox(const ConvexHullShape& a, const Math::mat4& transformA,
                                const BoxShape& b, const Math::mat4& transformB,
                                ContactManifold& out, f32 margin)
{
    if (a.vertices().empty())
        return false;
    const Math::mat3 rotationB(transformB);

    static thread_local ct::Vector<Math::vec3> hullNormals;
    static thread_local ct::Vector<Math::vec3> hullEdges;
    hullFaceNormals(a, transformA, hullNormals);
    hullEdgeDirections(a, transformA, hullEdges);
    const Math::vec3 boxAxes[3] = {Math::normalize(Math::vec3(rotationB[0])),
                                   Math::normalize(Math::vec3(rotationB[1])),
                                   Math::normalize(Math::vec3(rotationB[2]))};

    // The hull's face normals stand in for boxBox()'s first three axes, the
    // box's three faces are the second three, and the cross products run over
    // the hull's actual edge directions against the box's three.
    enum : u32
    {
        HullFace,
        BoxFace,
        EdgeEdge
    };
    SatAxis best;
    for (u32 face = 0; face < hullNormals.size(); ++face)
        if (!testAxis(a, transformA, b, transformB, hullNormals[face], margin, HullFace, face, 0,
                      best))
            return false;
    for (u32 i = 0; i < 3; ++i)
        if (!testAxis(a, transformA, b, transformB, boxAxes[i], margin, BoxFace, i, 0, best))
            return false;
    for (u32 i = 0; i < hullEdges.size(); ++i)
        for (u32 j = 0; j < 3; ++j)
            if (!testAxis(a, transformA, b, transformB, Math::cross(hullEdges[i], boxAxes[j]),
                          margin, EdgeEdge, i, j, best, kFeatureAxisBias))
                return false;

    out.normal = best.normal;
    out.buildTangents();

    if (best.kind == EdgeEdge)
    {
        Math::vec3 startA, endA, startB, endB;
        Math::vec3 boxCorners[8];
        b.corners(transformB, boxCorners);
        boxSupportingEdge(boxCorners, best.second, -out.normal, startB, endB);
        if (hullSupportingEdge(a, transformA, hullEdges[best.first], out.normal, startA, endA))
            edgeEdgeContact(startA, endA, startB, endB, best.penetration, out);
        else
            supportPointContact(a, transformA, b, transformB, best.penetration, out);
        return true;
    }

    static thread_local ct::Vector<Math::vec3> referencePolygon;
    static thread_local ct::Vector<Math::vec3> incidentPolygon;
    referencePolygon.clear();
    incidentPolygon.clear();
    Math::vec3 referenceNormal(0.0f);

    if (best.kind == HullFace)
    {
        const u32 referenceFace = hullReferenceFace(a, transformA, out.normal);
        referenceNormal = hullFaceNormal(a, transformA, referenceFace);
        hullFacePolygon(a, transformA, referenceFace, referencePolygon);

        const u32 incidentFaceIndex = incidentFace(transformB, referenceNormal);
        Math::vec3 boxCorners[8];
        b.corners(transformB, boxCorners);
        const u8* indices = BoxShape::faceCorners(incidentFaceIndex);
        for (u32 i = 0; i < 4; ++i)
            incidentPolygon.push_back(boxCorners[indices[i]]);
    }
    else
    {
        // The box is the reference shape, pointing at the hull.
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < 6; ++face)
        {
            const f32 value = Math::dot(BoxShape::faceNormal(transformB, face), -out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = BoxShape::faceNormal(transformB, referenceFace);
        Math::vec3 boxCorners[8];
        b.corners(transformB, boxCorners);
        const u8* indices = BoxShape::faceCorners(referenceFace);
        for (u32 i = 0; i < 4; ++i)
            referencePolygon.push_back(boxCorners[indices[i]]);

        hullFacePolygon(a, transformA, hullIncidentFace(a, transformA, referenceNormal),
                        incidentPolygon);
    }

    // A hull with no face of any area - a point or a segment - has no
    // incident polygon to clip and ends up here too.
    if (!clipFaceAgainstFace(referencePolygon.data(), static_cast<u32>(referencePolygon.size()),
                             referenceNormal, incidentPolygon.data(),
                             static_cast<u32>(incidentPolygon.size()), margin, out))
        supportPointContact(a, transformA, b, transformB, best.penetration, out);
    return true;
}

bool Narrowphase::convexHullConvexHull(const ConvexHullShape& a, const Math::mat4& transformA,
                                       const ConvexHullShape& b, const Math::mat4& transformB,
                                       ContactManifold& out, f32 margin, Math::vec3* separatingAxis)
{
    if (a.vertices().empty() || b.vertices().empty())
        return false;
    auto apart = [separatingAxis](const Math::vec3& axis)
    {
        if (separatingAxis)
            *separatingAxis = axis;
        return false;
    };

    static thread_local ct::Vector<Math::vec3> normalsA;
    static thread_local ct::Vector<Math::vec3> normalsB;
    static thread_local ct::Vector<Math::vec3> edgesA;
    static thread_local ct::Vector<Math::vec3> edgesB;
    hullFaceNormals(a, transformA, normalsA);
    hullFaceNormals(b, transformB, normalsB);
    hullEdgeDirections(a, transformA, edgesA);
    hullEdgeDirections(b, transformB, edgesB);

    // A's face normals, B's face normals, then the cross product of every
    // distinct edge direction of A against every one of B.
    enum : u32
    {
        FaceA,
        FaceB,
        EdgeEdge
    };
    SatAxis best;
    for (u32 face = 0; face < normalsA.size(); ++face)
        if (!testAxis(a, transformA, b, transformB, normalsA[face], margin, FaceA, face, 0, best))
            return apart(normalsA[face]);
    for (u32 face = 0; face < normalsB.size(); ++face)
        if (!testAxis(a, transformA, b, transformB, normalsB[face], margin, FaceB, face, 0, best,
                      kFeatureAxisBias))
            return apart(normalsB[face]);
    // Only an edge pair whose arcs cross on the Gauss map can carry the axis.
    const bool arcs = a.edgeArcsKnown() && b.edgeArcsKnown();
    if (arcs)
    {
        const ct::Vector<u32>& facesA = a.edgeFaces();
        const ct::Vector<u32>& facesB = b.edgeFaces();
        static thread_local ct::Vector<Math::vec3> alongB;
        alongB.resize(facesB.size() / 2);
        for (u32 j = 0; j < alongB.size(); ++j)
            alongB[j] = Math::cross(normalsB[facesB[2 * j]], normalsB[facesB[2 * j + 1]]);
        for (u32 i = 0; 2 * i + 1 < facesA.size(); ++i)
        {
            const Math::vec3& normalA0 = normalsA[facesA[2 * i]];
            const Math::vec3& normalA1 = normalsA[facesA[2 * i + 1]];
            const Math::vec3 alongA = Math::cross(normalA1, normalA0);
            if (Math::dot(alongA, alongA) < 1e-12f)
                continue;
            for (u32 j = 0; j < alongB.size(); ++j)
            {
                const Math::vec3& crossB = alongB[j];
                if (Math::dot(crossB, crossB) < 1e-12f)
                    continue;
                const Math::vec3& normalB0 = normalsB[facesB[2 * j]];
                const Math::vec3& normalB1 = normalsB[facesB[2 * j + 1]];
                // Each arc must cross the other's great circle, and at the same point:
                // the crossing lies on arc A and on arc B taken through the origin.
                if (!(Math::dot(normalB0, alongA) * Math::dot(normalB1, alongA) < 0.0f) ||
                    !(Math::dot(normalA0, crossB) * Math::dot(normalA1, crossB) < 0.0f))
                    continue;
                Math::vec3 crossing = Math::cross(alongA, crossB);
                if (Math::dot(crossing, normalA0 + normalA1) < 0.0f)
                    crossing = -crossing;
                if (!(Math::dot(crossing, normalB0 + normalB1) < 0.0f))
                    continue;
                if (!testAxis(a, transformA, b, transformB, crossing, margin, EdgeEdge, i, j, best,
                              kFeatureAxisBias))
                    return apart(crossing);
            }
        }
    }
    else
    {
        for (u32 i = 0; i < edgesA.size(); ++i)
            for (u32 j = 0; j < edgesB.size(); ++j)
                if (!testAxis(a, transformA, b, transformB, Math::cross(edgesA[i], edgesB[j]),
                              margin, EdgeEdge, i, j, best, kFeatureAxisBias))
                    return apart(Math::cross(edgesA[i], edgesB[j]));
    }

    // Two points, or a point and a segment: no axis at all, except their
    // supports along the line joining them.
    if (best.penetration == FLT_MAX)
    {
        const Math::vec3 axis = Math::vec3(transformB[3]) - Math::vec3(transformA[3]);
        if (!testAxis(a, transformA, b, transformB,
                      Math::length(axis) > kEpsilon ? axis : Math::vec3(0.0f, 1.0f, 0.0f), margin,
                      EdgeEdge, 0, 0, best))
            return false;
        out.normal = best.normal;
        out.buildTangents();
        supportPointContact(a, transformA, b, transformB, best.penetration, out);
        return true;
    }

    out.normal = best.normal;
    out.buildTangents();

    if (best.kind == EdgeEdge)
    {
        const Math::vec3 directionA =
            arcs ? hullEdgeDirection(a, transformA, best.first) : edgesA[best.first];
        const Math::vec3 directionB =
            arcs ? hullEdgeDirection(b, transformB, best.second) : edgesB[best.second];
        Math::vec3 startA, endA, startB, endB;
        if (hullSupportingEdge(a, transformA, directionA, out.normal, startA, endA) &&
            hullSupportingEdge(b, transformB, directionB, -out.normal, startB, endB))
            edgeEdgeContact(startA, endA, startB, endB, best.penetration, out);
        else
            supportPointContact(a, transformA, b, transformB, best.penetration, out);
        return true;
    }

    static thread_local ct::Vector<Math::vec3> referencePolygon;
    static thread_local ct::Vector<Math::vec3> incidentPolygon;
    Math::vec3 referenceNormal(0.0f);

    if (best.kind == FaceA)
    {
        const u32 referenceFace = hullReferenceFace(a, transformA, out.normal);
        referenceNormal = hullFaceNormal(a, transformA, referenceFace);
        hullFacePolygon(a, transformA, referenceFace, referencePolygon);
        hullFacePolygon(b, transformB, hullIncidentFace(b, transformB, referenceNormal),
                        incidentPolygon);
    }
    else
    {
        const u32 referenceFace = hullReferenceFace(b, transformB, -out.normal);
        referenceNormal = hullFaceNormal(b, transformB, referenceFace);
        hullFacePolygon(b, transformB, referenceFace, referencePolygon);
        hullFacePolygon(a, transformA, hullIncidentFace(a, transformA, referenceNormal),
                        incidentPolygon);
    }

    if (!clipFaceAgainstFace(referencePolygon.data(), static_cast<u32>(referencePolygon.size()),
                             referenceNormal, incidentPolygon.data(),
                             static_cast<u32>(incidentPolygon.size()), margin, out))
        supportPointContact(a, transformA, b, transformB, best.penetration, out);
    return true;
}

bool Narrowphase::convexPlane(const CollisionShape& a, const Math::mat4& transformA,
                              const PlaneShape& b, const Math::mat4& transformB,
                              ContactManifold& out, f32 margin)
{
    const Math::vec3 planeNormal = Math::normalize(Math::mat3(transformB) * b.normal());
    const Math::vec3 planeOrigin =
        Math::vec3(transformB * Math::vec4(b.normal() * b.constant(), 1.0f));
    const f32 planeConstant = Math::dot(planeNormal, planeOrigin);
    const Math::vec3 vertex = a.support(transformA, -planeNormal);
    const f32 distance = Math::dot(planeNormal, vertex) - planeConstant;
    if (distance > margin)
        return false;

    out.normal = -planeNormal;
    out.buildTangents();
    out.count = 1;
    out.points[0].position = vertex - distance * planeNormal;
    out.points[0].penetration = -distance;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

namespace
{

// A cylinder is collided as the inscribed prism it carries, whose contacts
// sit on its facets: a rolling cylinder would hop by the facet depth at each
// one and lose speed. This puts the contacts back on the true surface: the
// depth grows by how far the true support point along the normal is beyond
// the prism's, and every point moves to the true rim point straight under (or
// beside) the axis.
void correctCylinderManifold(const CylinderShape& cylinder, const Math::mat4& transform,
                             bool cylinderIsA, ContactManifold& manifold)
{
    if (manifold.count == 0)
        return;

    const Math::vec3 centre(transform[3]);
    const Math::vec3 axis = Math::normalize(Math::vec3(transform[1]));
    const Math::vec3 toOther = cylinderIsA ? manifold.normal : -manifold.normal;
    const f32 along = Math::dot(axis, toOther);
    // A cap against a face is exact in the prism already, and the radial
    // direction below is undefined there.
    if (fabsf(along) > 0.9995f)
        return;

    const Math::vec3 radial = Math::normalize(toOther - axis * along);
    const f32 end = along >= 0.0f ? cylinder.halfHeight() : -cylinder.halfHeight();
    const Math::vec3 trueSupport = centre + axis * end + radial * cylinder.radius();
    const Math::vec3 prismSupport = cylinder.hull().support(transform, toOther);
    const f32 delta = Math::max(0.0f, Math::dot(trueSupport - prismSupport, toOther));

    ContactPoint corrected[ContactManifold::MaxPoints];
    u32 count = 0;
    for (u32 i = 0; i < manifold.count; ++i)
    {
        ContactPoint point = manifold.points[i];
        point.penetration += delta;
        const f32 axial = Math::clamp(Math::dot(point.position - centre, axis),
                                      -cylinder.halfHeight(), cylinder.halfHeight());
        const Math::vec3 onSurface = centre + axis * axial + radial * cylinder.radius();
        point.position = onSurface - toOther * (point.penetration * 0.5f);

        bool duplicate = false;
        for (u32 k = 0; k < count && !duplicate; ++k)
        {
            const Math::vec3 gap = corrected[k].position - point.position;
            duplicate = Math::dot(gap, gap) < 1e-6f;
        }
        if (duplicate)
            continue;
        corrected[count++] = point;
    }
    for (u32 i = 0; i < count; ++i)
        manifold.points[i] = corrected[i];
    manifold.count = count;
}

// Two unit vectors square to `axis` and to each other.
void perpendicularBasis(const Math::vec3& axis, Math::vec3& u, Math::vec3& v)
{
    const Math::vec3 helper = fabsf(axis.y) < 0.9f ? Math::vec3(0.0f, 1.0f, 0.0f)
                                                   : Math::vec3(1.0f, 0.0f, 0.0f);
    u = Math::normalize(Math::cross(helper, axis));
    v = Math::cross(axis, u);
}

// The cylinder against a plane. Normal runs from the cylinder into the plane,
// like convexPlane(). A cap lying flat is four points on its rim, a cylinder
// on its side is the two ends of the line it touches, and anything between is
// the one rim point lowest into the plane - never a hull's facet corner.
bool cylinderPlane(const CylinderShape& cylinder, const Math::mat4& transform,
                   const PlaneShape& plane, const Math::mat4& planeTransform, ContactManifold& out,
                   f32 margin)
{
    const Math::vec3 planeNormal = Math::normalize(Math::mat3(planeTransform) * plane.normal());
    const Math::vec3 planeOrigin =
        Math::vec3(planeTransform * Math::vec4(plane.normal() * plane.constant(), 1.0f));
    const f32 planeConstant = Math::dot(planeNormal, planeOrigin);

    const Math::vec3 centre(transform[3]);
    const Math::vec3 axis = Math::normalize(Math::vec3(transform[1]));
    const f32 radius = cylinder.radius();
    const f32 half = cylinder.halfHeight();
    const Math::vec3 down = -planeNormal;
    const f32 along = Math::dot(axis, down);
    const f32 end = along >= 0.0f ? half : -half;

    Math::vec3 points[4];
    u32 count = 0;
    if (fabsf(along) > 0.9995f)
    {
        Math::vec3 u, v;
        perpendicularBasis(axis, u, v);
        const Math::vec3 capCentre = centre + axis * end;
        points[count++] = capCentre + u * radius;
        points[count++] = capCentre + v * radius;
        points[count++] = capCentre - u * radius;
        points[count++] = capCentre - v * radius;
    }
    else
    {
        const Math::vec3 radial = Math::normalize(down - axis * along);
        const Math::vec3 rim = centre + radial * radius;
        if (fabsf(along) < 0.05f)
        {
            points[count++] = rim + axis * half;
            points[count++] = rim - axis * half;
        }
        else
            points[count++] = rim + axis * end;
    }

    f32 deepest = 1.0e30f;
    for (u32 i = 0; i < count; ++i)
        deepest = Math::min(deepest, Math::dot(planeNormal, points[i]) - planeConstant);
    if (deepest > margin)
        return false;

    out.normal = down;
    out.buildTangents();
    out.count = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 distance = Math::dot(planeNormal, points[i]) - planeConstant;
        if (distance > margin)
            continue;
        writePoint(out.points[out.count++], points[i] - planeNormal * distance, -distance);
    }
    return out.count > 0;
}

// The cylinder against a sphere, normal from the cylinder to the sphere: the
// closest point of the cylinder to the centre is a clamp in its own space.
bool cylinderSphere(const CylinderShape& cylinder, const Math::mat4& transform,
                    const SphereShape& sphere, const Math::mat4& sphereTransform,
                    ContactManifold& out, f32 margin)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverse = Math::transpose(rotation);
    const Math::vec3 centre(transform[3]);
    const Math::vec3 local = inverse * (Math::vec3(sphereTransform[3]) - centre);
    const f32 radius = cylinder.radius();
    const f32 half = cylinder.halfHeight();
    const f32 sphereRadius = sphere.radius();

    const f32 radial = sqrtf(local.x * local.x + local.z * local.z);
    Math::vec3 closest = local;
    Math::vec3 localNormal(0.0f, 1.0f, 0.0f);
    f32 penetration = 0.0f;
    if (radial <= radius && fabsf(local.y) <= half)
    {
        const f32 toSide = radius - radial;
        const f32 toCap = half - fabsf(local.y);
        if (toSide < toCap)
        {
            const f32 inverseRadial = radial > kEpsilon ? 1.0f / radial : 0.0f;
            localNormal = radial > kEpsilon
                              ? Math::vec3(local.x * inverseRadial, 0.0f, local.z * inverseRadial)
                              : Math::vec3(1.0f, 0.0f, 0.0f);
            closest = localNormal * radius + Math::vec3(0.0f, local.y, 0.0f);
            penetration = toSide + sphereRadius;
        }
        else
        {
            localNormal = Math::vec3(0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
            closest = Math::vec3(local.x, localNormal.y * half, local.z);
            penetration = toCap + sphereRadius;
        }
    }
    else
    {
        const f32 clampedRadial = Math::min(radial, radius);
        const f32 scale = radial > kEpsilon ? clampedRadial / radial : 0.0f;
        closest = Math::vec3(local.x * scale, Math::clamp(local.y, -half, half), local.z * scale);
        const Math::vec3 delta = local - closest;
        const f32 distance = Math::length(delta);
        if (distance > sphereRadius + margin)
            return false;
        localNormal = distance > kEpsilon ? delta / distance : Math::vec3(0.0f, 1.0f, 0.0f);
        penetration = sphereRadius - distance;
    }

    const Math::vec3 normal = Math::normalize(rotation * localNormal);
    const Math::vec3 surfaceOfCylinder = centre + rotation * closest;
    const Math::vec3 surfaceOfSphere = Math::vec3(sphereTransform[3]) - normal * sphereRadius;
    out.normal = normal;
    out.buildTangents();
    out.count = 1;
    writePoint(out.points[0], (surfaceOfCylinder + surfaceOfSphere) * 0.5f, penetration);
    return true;
}

// A convex polytope against a cylinder, normal from the polytope to the
// cylinder. Separating axes: the polytope's face normals, the cylinder's
// axis, that axis crossed with every polytope edge, and the direction from
// the axis to every polytope vertex (finds a curb running parallel to a
// wheel). Contact points come from the cylinder's own surface (the cap rim
// clipped to the reference face, the side line, or the single rim point), so
// a wheel never sees the corners of a prism.
template <typename PolygonOf, typename ClosestToSegment>
bool polytopeCylinder(const CollisionShape& polytope, const Math::mat4& polytopeTransform,
                      const Math::vec3* faceNormals, u32 faceCount, const Math::vec3* edges,
                      u32 edgeCount, const Math::vec3* vertices, u32 vertexCount,
                      PolygonOf polygonOf, bool windingKnown, ClosestToSegment closestToSegment,
                      const CylinderShape& cylinder, const Math::mat4& cylinderTransform,
                      ContactManifold& out, f32 margin)
{
    const Math::vec3 centre(cylinderTransform[3]);
    const Math::vec3 axis = Math::normalize(Math::vec3(cylinderTransform[1]));
    const f32 radius = cylinder.radius();
    const f32 half = cylinder.halfHeight();

    SatAxis best;
    for (u32 i = 0; i < faceCount; ++i)
        if (!testAxis(polytope, polytopeTransform, cylinder, cylinderTransform, faceNormals[i],
                      margin, 0, i, 0, best))
            return false;
    if (!testAxis(polytope, polytopeTransform, cylinder, cylinderTransform, axis, margin, 1, 0, 0,
                  best))
        return false;
    for (u32 i = 0; i < edgeCount; ++i)
        if (!testAxis(polytope, polytopeTransform, cylinder, cylinderTransform,
                      Math::cross(axis, edges[i]), margin, 2, i, 0, best, kFeatureAxisBias))
            return false;
    for (u32 i = 0; i < vertexCount; ++i)
    {
        const Math::vec3 offset = vertices[i] - centre;
        if (!testAxis(polytope, polytopeTransform, cylinder, cylinderTransform,
                      offset - axis * Math::dot(offset, axis), margin, 3, i, 0, best,
                      kFeatureAxisBias))
            return false;
    }
    const f32 bestPenetration = best.penetration;

    out.normal = best.normal;
    out.buildTangents();
    out.count = 0;

    const f32 along = Math::dot(axis, out.normal);
    const Math::vec3 toPolytope = -out.normal;
    const f32 end = along >= 0.0f ? -half : half;
    const Math::vec3 radial = Math::normalize(toPolytope - axis * Math::dot(axis, toPolytope));
    const bool capFlat = fabsf(along) > 0.9995f;
    const bool sideOn = fabsf(along) < 0.05f;

    if (best.kind == 0)
    {
        u32 referenceFace = best.first;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < faceCount; ++face)
        {
            const f32 value = Math::dot(faceNormals[face], out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        static thread_local ct::Vector<Math::vec3> polygon;
        const u32 polygonCount = polygonOf(referenceFace, polygon);
        // Facing the cylinder: a triangle's normal comes from its winding,
        // which can point either way.
        Math::vec3 referenceNormal = faceNormals[referenceFace];
        if (Math::dot(referenceNormal, out.normal) < 0.0f)
            referenceNormal = -referenceNormal;

        if (polygonCount >= 3 && fabsf(along) > 0.98f)
        {
            Math::vec3 u, v;
            perpendicularBasis(axis, u, v);
            const Math::vec3 capCentre = centre + axis * end;
            constexpr u32 kRim = 8;
            Math::vec3 rim[kRim];
            for (u32 i = 0; i < kRim; ++i)
            {
                const f32 angle = 6.28318530718f * static_cast<f32>(i) / static_cast<f32>(kRim);
                rim[i] = capCentre + (u * cosf(angle) + v * sinf(angle)) * radius;
            }
            const bool clipped =
                windingKnown ? clipFaceAgainstFace(polygon.data(), polygonCount, referenceNormal,
                                                   rim, kRim, margin, out)
                             : clipFaceAgainstPolygon(polygon.data(), polygonCount,
                                                      referenceNormal, rim, kRim, margin, out);
            if (clipped)
                return true;
        }
        else if (polygonCount >= 3 && sideOn)
        {
            // The side line, clipped to the face: a cylinder lying half off
            // a ledge touches it only along the part over the ledge.
            const Math::vec3 rimLine = centre + radial * radius;
            ContactPoint lying[2];
            if (lyingContacts(polygon.data(), polygonCount, referenceNormal, rimLine + axis * half,
                              rimLine - axis * half, 0.0f, margin, lying) == 2 &&
                lying[0].penetration >= bestPenetration - kLyingDepthTolerance)
            {
                out.count = 2;
                out.points[0] = lying[0];
                out.points[1] = lying[1];
                return true;
            }
            out.count = 0;
        }
    }

    if (capFlat)
    {
        supportPointContact(polytope, polytopeTransform, cylinder, cylinderTransform,
                            bestPenetration, out);
        return true;
    }

    out.count = 1;
    if (sideOn)
    {
        // The deepest part of the cylinder is a whole line here, and which end
        // is "the" rim point is a coin toss on the sign of a near-zero
        // `along`. With the cylinder moved just clear along the normal, the
        // closest pair between its side line and the polytope is where they
        // meet - on a ledge, not at the end of the line hanging over nothing.
        const f32 clearance = Math::max(bestPenetration, 0.0f) + 0.01f * radius + 1e-4f;
        const Math::vec3 shift = out.normal * clearance;
        const Math::vec3 rimLine = centre + radial * radius + shift;
        Math::vec3 onLine, onPolytope;
        if (closestToSegment(rimLine - axis * half, rimLine + axis * half, onLine, onPolytope) !=
            FLT_MAX)
        {
            writePoint(out.points[0], ((onLine - shift) + onPolytope) * 0.5f, bestPenetration);
            return true;
        }
    }

    // Tilted: the cylinder's one deepest point along the normal is the rim
    // point at the lower end, and that is the contact.
    const Math::vec3 rimPoint = centre + axis * end + radial * radius;
    writePoint(out.points[0], rimPoint + out.normal * (bestPenetration * 0.5f), bestPenetration);
    return true;
}

// closestPointsSegmentPolytope() for a polytope placed by `transform`, in
// world space and without the inside flag - the side-line query above.
f32 closestPointsSegmentPolytopeWorld(const PolytopeSurface& surface, const Math::mat4& transform,
                                      const Math::vec3& start, const Math::vec3& end,
                                      Math::vec3& onSegment, Math::vec3& onPolytope)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverse = Math::transpose(rotation);
    const Math::vec3 origin(transform[3]);
    bool inside = false;
    const f32 squared = closestPointsSegmentPolytope(surface, inverse * (start - origin),
                                                     inverse * (end - origin), onSegment,
                                                     onPolytope, inside);
    onSegment = origin + rotation * onSegment;
    onPolytope = origin + rotation * onPolytope;
    return squared;
}

bool boxCylinder(const BoxShape& box, const Math::mat4& boxTransform, const CylinderShape& cylinder,
                 const Math::mat4& cylinderTransform, ContactManifold& out, f32 margin)
{
    Math::vec3 corners[8];
    box.corners(boxTransform, corners);
    Math::vec3 normals[6];
    for (u32 i = 0; i < 6; ++i)
        normals[i] = BoxShape::faceNormal(boxTransform, i);
    const Math::mat3 rotation(boxTransform);
    const Math::vec3 edges[3] = {Math::normalize(Math::vec3(rotation[0])),
                                 Math::normalize(Math::vec3(rotation[1])),
                                 Math::normalize(Math::vec3(rotation[2]))};
    auto polygonOf = [&](u32 face, ct::Vector<Math::vec3>& polygon)
    {
        polygon.clear();
        const u8* indices = BoxShape::faceCorners(face);
        for (u32 i = 0; i < 4; ++i)
            polygon.push_back(corners[indices[i]]);
        return 4u;
    };
    BoxSurface surface;
    buildBoxSurface(box, surface);
    auto closestToSegment = [&](const Math::vec3& start, const Math::vec3& end,
                                Math::vec3& onSegment, Math::vec3& onPolytope)
    {
        return closestPointsSegmentPolytopeWorld(surface.view(), boxTransform, start, end,
                                                 onSegment, onPolytope);
    };
    return polytopeCylinder(box, boxTransform, normals, 6, edges, 3, corners, 8, polygonOf, true,
                            closestToSegment, cylinder, cylinderTransform, out, margin);
}

bool hullCylinder(const ConvexHullShape& hull, const Math::mat4& hullTransform,
                  const CylinderShape& cylinder, const Math::mat4& cylinderTransform,
                  ContactManifold& out, f32 margin)
{
    if (hull.vertices().empty())
        return false;
    static thread_local ct::Vector<Math::vec3> normals;
    static thread_local ct::Vector<Math::vec3> edges;
    static thread_local ct::Vector<Math::vec3> vertices;
    hullFaceNormals(hull, hullTransform, normals);
    hullEdgeDirections(hull, hullTransform, edges);
    vertices.resize(hull.vertices().size());
    for (usize i = 0; i < vertices.size(); ++i)
        vertices[i] = Math::vec3(hullTransform * Math::vec4(hull.vertices()[i], 1.0f));
    auto polygonOf = [&](u32 face, ct::Vector<Math::vec3>& polygon)
    { return hullFacePolygon(hull, hullTransform, face, polygon); };
    const PolytopeSurface surface = hullSurface(hull);
    auto closestToSegment = [&](const Math::vec3& start, const Math::vec3& end,
                                Math::vec3& onSegment, Math::vec3& onPolytope)
    {
        return closestPointsSegmentPolytopeWorld(surface, hullTransform, start, end, onSegment,
                                                 onPolytope);
    };
    return polytopeCylinder(hull, hullTransform, normals.data(), static_cast<u32>(normals.size()),
                            edges.data(), static_cast<u32>(edges.size()), vertices.data(),
                            static_cast<u32>(vertices.size()), polygonOf, true, closestToSegment,
                            cylinder, cylinderTransform, out, margin);
}

bool triangleCylinder(const TriangleShape& triangle, const Math::mat4& triangleTransform,
                      const CylinderShape& cylinder, const Math::mat4& cylinderTransform,
                      ContactManifold& out, f32 margin)
{
    const Math::vec3 v0 = Math::vec3(triangleTransform * Math::vec4(triangle.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(triangleTransform * Math::vec4(triangle.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(triangleTransform * Math::vec4(triangle.vertex(2), 1.0f));
    const Math::vec3 raw = Math::cross(v1 - v0, v2 - v0);
    const f32 length = Math::length(raw);
    if (length < kEpsilon)
        return false;
    const Math::vec3 normal = raw / length;
    const Math::vec3 vertices[3] = {v0, v1, v2};
    const Math::vec3 edges[3] = {v1 - v0, v2 - v1, v0 - v2};
    auto polygonOf = [&](u32, ct::Vector<Math::vec3>& polygon)
    {
        polygon.clear();
        polygon.push_back(v0);
        polygon.push_back(v1);
        polygon.push_back(v2);
        return 3u;
    };
    auto closestToSegment = [&](const Math::vec3& start, const Math::vec3& end,
                                Math::vec3& onSegment, Math::vec3& onPolytope)
    { return closestPointsSegmentTriangle(start, end, v0, v1, v2, onSegment, onPolytope); };
    return polytopeCylinder(triangle, triangleTransform, &normal, 1, edges, 3, vertices, 3,
                            polygonOf, false, closestToSegment, cylinder, cylinderTransform, out,
                            margin);
}

// Every pair that has a cylinder in it. Plane, sphere, box, hull and triangle
// are solved on the true surface; capsule and cylinder-cylinder go through
// the prism, pulled back onto the surface afterwards.
bool collideCylinder(const CollisionShape& a, const Math::mat4& transformA, const CollisionShape& b,
                     const Math::mat4& transformB, ContactManifold& out, f32 margin)
{
    const bool cylinderFirst = a.type() == ShapeType::Cylinder;
    const CollisionShape& other = cylinderFirst ? b : a;
    const Math::mat4& otherTransform = cylinderFirst ? transformB : transformA;
    const CylinderShape& cylinder = static_cast<const CylinderShape&>(cylinderFirst ? a : b);
    const Math::mat4& cylinderTransform = cylinderFirst ? transformA : transformB;

    bool hit = false;
    bool normalFromCylinder = false;
    switch (other.type())
    {
    case ShapeType::Plane:
        hit = cylinderPlane(cylinder, cylinderTransform, static_cast<const PlaneShape&>(other),
                            otherTransform, out, margin);
        normalFromCylinder = true;
        break;
    case ShapeType::Sphere:
        hit = cylinderSphere(cylinder, cylinderTransform, static_cast<const SphereShape&>(other),
                             otherTransform, out, margin);
        normalFromCylinder = true;
        break;
    case ShapeType::Box:
        hit = boxCylinder(static_cast<const BoxShape&>(other), otherTransform, cylinder,
                          cylinderTransform, out, margin);
        break;
    case ShapeType::ConvexHull:
        hit = hullCylinder(static_cast<const ConvexHullShape&>(other), otherTransform, cylinder,
                           cylinderTransform, out, margin);
        break;
    case ShapeType::Triangle:
        hit = triangleCylinder(static_cast<const TriangleShape&>(other), otherTransform, cylinder,
                               cylinderTransform, out, margin);
        break;
    default:
        return false;
    }
    if (!hit)
        return false;

    // The routines above leave the normal running from the first shape they
    // name to the second; the pair asked for runs from a to b.
    const bool runsFromCylinder = normalFromCylinder;
    if (runsFromCylinder != cylinderFirst)
    {
        out.normal = -out.normal;
        out.buildTangents();
    }
    return true;
}

} // namespace

bool Narrowphase::collide(const CollisionShape& a, const Math::mat4& transformA,
                          const CollisionShape& b, const Math::mat4& transformB,
                          ContactManifold& out, f32 margin, SeparatingAxis* separation)
{
    f32 penetration = 0.0f;
    Math::vec3 normal(0.0f);
    if (separation && separation->known &&
        !axisOverlap(a, transformA, b, transformB, separation->axis, margin, penetration, normal))
        return false;
    Math::vec3 axis(0.0f);
    if (collideShapes(a, transformA, b, transformB, out, margin, separation ? &axis : nullptr))
    {
        if (separation)
            separation->known = false;
        return true;
    }
    if (separation)
    {
        // Pairs that do not name their axis get the centre line, when it separates.
        if (Math::length(axis) < kEpsilon)
            axis = Math::vec3(transformB[3]) - Math::vec3(transformA[3]);
        separation->known =
            Math::length(axis) > kEpsilon &&
            !axisOverlap(a, transformA, b, transformB, axis, margin, penetration, normal);
        if (separation->known)
            separation->axis = axis;
    }
    return false;
}

bool Narrowphase::collideShapes(const CollisionShape& a, const Math::mat4& transformA,
                                const CollisionShape& b, const Math::mat4& transformB,
                                ContactManifold& out, f32 margin, Math::vec3* separatingAxis)
{
    // A cylinder meets everything through the prism it carries inside: the
    // hull pairs below already do every case, in both orders.
    if ((a.type() == ShapeType::Cylinder) != (b.type() == ShapeType::Cylinder))
    {
        const ShapeType partner = a.type() == ShapeType::Cylinder ? b.type() : a.type();
        if (partner == ShapeType::Plane || partner == ShapeType::Sphere ||
            partner == ShapeType::Box || partner == ShapeType::ConvexHull ||
            partner == ShapeType::Triangle)
            return collideCylinder(a, transformA, b, transformB, out, margin);
    }
    if (a.type() == ShapeType::Cylinder)
    {
        const CylinderShape& cylinder = static_cast<const CylinderShape&>(a);
        if (!collideShapes(cylinder.hull(), transformA, b, transformB, out, margin, separatingAxis))
            return false;
        correctCylinderManifold(cylinder, transformA, true, out);
        return true;
    }
    if (b.type() == ShapeType::Cylinder)
    {
        const CylinderShape& cylinder = static_cast<const CylinderShape&>(b);
        if (!collideShapes(a, transformA, cylinder.hull(), transformB, out, margin, separatingAxis))
            return false;
        correctCylinderManifold(cylinder, transformB, false, out);
        return true;
    }

    if (b.type() == ShapeType::Plane &&
        (a.type() == ShapeType::Sphere || a.type() == ShapeType::Box ||
         a.type() == ShapeType::Capsule || a.type() == ShapeType::ConvexHull))
        return convexPlane(a, transformA, static_cast<const PlaneShape&>(b), transformB, out,
                           margin);

    if (a.type() == ShapeType::Plane &&
        (b.type() == ShapeType::Sphere || b.type() == ShapeType::Box ||
         b.type() == ShapeType::Capsule || b.type() == ShapeType::ConvexHull))
    {
        if (!convexPlane(b, transformB, static_cast<const PlaneShape&>(a), transformA, out,
                         margin))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    if (a.type() == ShapeType::Sphere && b.type() == ShapeType::Sphere)
        return sphereSphere(static_cast<const SphereShape&>(a), transformA,
                            static_cast<const SphereShape&>(b), transformB, out, margin);

    if (a.type() == ShapeType::Sphere && b.type() == ShapeType::Box)
        return sphereBox(static_cast<const SphereShape&>(a), transformA,
                         static_cast<const BoxShape&>(b), transformB, out, margin);

    if (a.type() == ShapeType::Box && b.type() == ShapeType::Sphere)
    {
        // Solved in the other order and flipped: one sphere-box routine.
        if (!sphereBox(static_cast<const SphereShape&>(b), transformB,
                       static_cast<const BoxShape&>(a), transformA, out, margin))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    if (a.type() == ShapeType::Box && b.type() == ShapeType::Box)
        return boxBox(static_cast<const BoxShape&>(a), transformA,
                      static_cast<const BoxShape&>(b), transformB, out, margin);

    // Every capsule pair is solved with the capsule first and flipped when
    // asked the other way round.
    if (a.type() == ShapeType::Capsule && b.type() == ShapeType::Sphere)
        return capsuleSphere(static_cast<const CapsuleShape&>(a), transformA,
                             static_cast<const SphereShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::Capsule && b.type() == ShapeType::Capsule)
        return capsuleCapsule(static_cast<const CapsuleShape&>(a), transformA,
                              static_cast<const CapsuleShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::Capsule && b.type() == ShapeType::Box)
        return capsuleBox(static_cast<const CapsuleShape&>(a), transformA,
                          static_cast<const BoxShape&>(b), transformB, out, margin);

    if (b.type() == ShapeType::Capsule &&
        (a.type() == ShapeType::Sphere || a.type() == ShapeType::Box))
    {
        if (!collideShapes(b, transformB, a, transformA, out, margin, separatingAxis))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    // Every hull pair is solved with the hull first and flipped when asked the
    // other way round.
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::ConvexHull)
        return convexHullConvexHull(static_cast<const ConvexHullShape&>(a), transformA,
                                    static_cast<const ConvexHullShape&>(b), transformB, out,
                                    margin, separatingAxis);
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::Sphere)
        return convexHullSphere(static_cast<const ConvexHullShape&>(a), transformA,
                                static_cast<const SphereShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::Box)
        return convexHullBox(static_cast<const ConvexHullShape&>(a), transformA,
                             static_cast<const BoxShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::Capsule)
        return convexHullCapsule(static_cast<const ConvexHullShape&>(a), transformA,
                                 static_cast<const CapsuleShape&>(b), transformB, out, margin);

    if (b.type() == ShapeType::ConvexHull &&
        (a.type() == ShapeType::Sphere || a.type() == ShapeType::Box ||
         a.type() == ShapeType::Capsule))
    {
        if (!collideShapes(b, transformB, a, transformA, out, margin, separatingAxis))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    return false;
}

namespace
{

// The normal a triangle contact should push along, A towards B. On the face
// it is the face normal; on a rim edge the direction to the closest point (so
// a body slides off a real edge); on an edge or corner SHARED with another
// triangle the face normal again - the seam is interior, and pushing along it
// stops a character dead walking from one triangle onto the next.
bool triangleContactNormal(const TriangleShape& triangle, const Math::mat4& transform,
                           TriangleFeature feature, const Math::vec3& offset, f32 distanceSquared,
                           Math::vec3& out)
{
    const bool degenerate = distanceSquared <= kEpsilon * kEpsilon;
    if (!degenerate && !triangle.featureIsInternal(feature))
    {
        out = offset / sqrtf(distanceSquared);
        return true;
    }

    const Math::vec3 raw = Math::mat3(transform) * triangle.rawNormal();
    const f32 length = Math::length(raw);
    if (length < kEpsilon)
        return false;
    Math::vec3 faceNormal = raw / length;
    // Oriented to agree with which side the convex is actually on, so a body
    // under a ceiling triangle is not pushed up through it.
    if (!degenerate && Math::dot(faceNormal, offset) < 0.0f)
        faceNormal = -faceNormal;
    out = faceNormal;
    return true;
}

} // namespace

bool Narrowphase::sphereTriangle(const SphereShape& a, const Math::mat4& transformA,
                                 const TriangleShape& b, const Math::mat4& transformB,
                                 ContactManifold& out, f32 margin)
{
    const Math::vec3 center(transformA[3]);
    const Math::vec3 v0 = Math::vec3(transformB * Math::vec4(b.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transformB * Math::vec4(b.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transformB * Math::vec4(b.vertex(2), 1.0f));

    TriangleFeature feature = TriangleFeature::Face;
    const Math::vec3 closest = closestPointOnTriangle(v0, v1, v2, center, &feature);
    const Math::vec3 offset = closest - center;
    const f32 distanceSquared = Math::dot(offset, offset);
    const f32 reach = a.radius() + margin;
    if (distanceSquared > reach * reach)
        return false;

    Math::vec3 normal;
    if (!triangleContactNormal(b, transformB, feature, offset, distanceSquared, normal))
        return false;

    out.normal = normal;
    out.buildTangents();
    out.count = 1;
    out.points[0].position = closest;
    out.points[0].penetration = a.radius() - sqrtf(distanceSquared);
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::boxTriangle(const BoxShape& a, const Math::mat4& transformA,
                              const TriangleShape& b, const Math::mat4& transformB,
                              ContactManifold& out, f32 margin)
{
    const Math::mat3 rotationA(transformA);
    const Math::vec3 v0 = Math::vec3(transformB * Math::vec4(b.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transformB * Math::vec4(b.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transformB * Math::vec4(b.vertex(2), 1.0f));

    const Math::vec3 faceNormalRaw = Math::cross(v1 - v0, v2 - v0);
    const f32 faceLength = Math::length(faceNormalRaw);
    if (faceLength < kEpsilon)
        return false;
    const Math::vec3 faceNormal = faceNormalRaw / faceLength;

    // Thirteen axes: the box's three face normals, the triangle's one, and the
    // nine cross products of the box's edges with the triangle's.
    const Math::vec3 triangleEdges[3] = {v1 - v0, v2 - v1, v0 - v2};
    Math::vec3 axes[13];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationA[i]));
    axes[axisCount++] = faceNormal;
    for (u32 i = 0; i < 3; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = Math::cross(Math::vec3(rotationA[i]), triangleEdges[j]);

    f32 bestPenetration = FLT_MAX;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        // A zero-length cross product carries no information (as in boxBox()).
        if (Math::length(axes[i]) < kEpsilon)
            continue;
        if (penetration + (i < 3 ? 0.0f : kFeatureAxisBias) < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
            bestAxis = i;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    // Edge against edge, no face to clip: one point where the two are
    // closest - the box's edge of that direction furthest towards the
    // triangle, against the triangle's own edge.
    if (bestAxis > 3)
    {
        const u32 boxAxis = (bestAxis - 4) / 3;
        const u32 triangleEdge = (bestAxis - 4) % 3;
        const Math::vec3 vertices[3] = {v0, v1, v2};
        Math::vec3 corners[8];
        a.corners(transformA, corners);
        Math::vec3 start, end;
        boxSupportingEdge(corners, boxAxis, out.normal, start, end);
        edgeEdgeContact(start, end, vertices[triangleEdge], vertices[(triangleEdge + 1) % 3],
                        bestPenetration, out);
        return true;
    }

    constexpr u32 capacity = 16;
    Math::vec3 polygon[capacity];
    Math::vec3 scratch[capacity];
    u32 count = 0;
    Math::vec3 referenceNormal(0.0f);
    f32 referenceOffset = 0.0f;
    Math::vec3 referenceInterior(0.0f);
    Math::vec3 edgeStart[4];
    Math::vec3 edgeEnd[4];
    u32 edgeCount = 0;

    Math::vec3 boxCorners[8];
    a.corners(transformA, boxCorners);

    if (bestAxis < 3)
    {
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < 6; ++face)
        {
            const f32 value = Math::dot(BoxShape::faceNormal(transformA, face), out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = BoxShape::faceNormal(transformA, referenceFace);
        const u8* indices = BoxShape::faceCorners(referenceFace);
        referenceOffset = Math::dot(referenceNormal, boxCorners[indices[0]]);
        for (u32 i = 0; i < 4; ++i)
        {
            edgeStart[i] = boxCorners[indices[i]];
            edgeEnd[i] = boxCorners[indices[(i + 1) % 4]];
            referenceInterior += edgeStart[i] * 0.25f;
        }
        edgeCount = 4;

        polygon[count++] = v0;
        polygon[count++] = v1;
        polygon[count++] = v2;
    }
    else
    {
        // Triangle is the reference; its outward normal must face the box, the
        // opposite of the A-to-B normal.
        referenceNormal = -out.normal;
        referenceOffset = Math::dot(referenceNormal, v0);
        referenceInterior = (v0 + v1 + v2) / 3.0f;
        const Math::vec3 vertices[3] = {v0, v1, v2};
        for (u32 i = 0; i < 3; ++i)
        {
            edgeStart[i] = vertices[i];
            edgeEnd[i] = vertices[(i + 1) % 3];
        }
        edgeCount = 3;

        const u32 face = incidentFace(transformA, referenceNormal);
        const u8* indices = BoxShape::faceCorners(face);
        for (u32 i = 0; i < 4; ++i)
            polygon[count++] = boxCorners[indices[i]];
    }

    for (u32 i = 0; i < edgeCount && count > 0; ++i)
    {
        const Math::vec3 edge = edgeEnd[i] - edgeStart[i];
        Math::vec3 planeNormal = Math::cross(referenceNormal, edge);
        const f32 planeLength = Math::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        Math::vec3 unit = planeNormal / planeLength;
        f32 offset = Math::dot(unit, edgeStart[i]);
        if (Math::dot(unit, referenceInterior) > offset)
        {
            unit = -unit;
            offset = -offset;
        }
        count = clipPolygon(polygon, count, unit, offset + kClipTolerance, scratch, capacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    Math::vec3 kept[capacity];
    f32 depths[capacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - Math::dot(referenceNormal, polygon[i]);
        if (depth < -margin)
            continue;
        kept[keptCount] = polygon[i] + referenceNormal * depth;
        depths[keptCount] = depth;
        ++keptCount;
    }
    if (keptCount == 0)
    {
        // Clipping produced nothing (a grazing pair): same fallback as boxBox().
        supportPointContact(a, transformA, b, transformB, bestPenetration, out);
        return true;
    }

    out.count = reducePoints(kept, depths, keptCount, out);
    for (u32 i = 0; i < out.count; ++i)
    {
        out.points[i].normalImpulse = 0.0f;
        out.points[i].tangentImpulse[0] = 0.0f;
        out.points[i].tangentImpulse[1] = 0.0f;
    }
    return out.count > 0;
}

bool Narrowphase::capsuleTriangle(const CapsuleShape& a, const Math::mat4& transformA,
                                  const TriangleShape& b, const Math::mat4& transformB,
                                  ContactManifold& out, f32 margin)
{
    Math::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const Math::vec3 v0 = Math::vec3(transformB * Math::vec4(b.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transformB * Math::vec4(b.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transformB * Math::vec4(b.vertex(2), 1.0f));

    // The closest pair between the segment and the triangle - ends, edges AND
    // the face the segment may pass through. Steadier than SAT here, since a
    // capsule has no faces to separate on.
    TriangleFeature feature = TriangleFeature::Face;
    Math::vec3 onSegment, onTriangle;
    const f32 squared =
        closestPointsSegmentTriangle(lower, upper, v0, v1, v2, onSegment, onTriangle, &feature);
    const f32 reach = a.radius() + margin;
    if (squared > reach * reach)
        return false;

    if (squared > kEpsilon * kEpsilon)
    {
        const Math::vec3 offset = onTriangle - onSegment;
        Math::vec3 normal;
        if (!triangleContactNormal(b, transformB, feature, offset, squared, normal))
            return false;
        out.normal = normal;
        out.buildTangents();
        out.count = 1;
        writePoint(out.points[0], onTriangle, a.radius() - sqrtf(squared));
        return true;
    }

    // The segment itself touches or passes through the face (a capsule
    // standing in the floor). The closest pair is one point with no
    // direction, and taking it from whichever end is nearer the plane flips
    // the normal and pushes a capsule mostly ABOVE the floor down through it.
    //
    // Out along the face normal, the shorter way: with the ends at heights d0
    // and d1 over the plane, leaving upwards takes r - min(d0, d1) and
    // downwards r + max(d0, d1); the first is shorter exactly when
    // d0 + d1 >= 0, i.e. the centre is on the upper side. The depth stays
    // continuous with the case above as an end crosses the plane.
    const Math::vec3 raw = Math::cross(v1 - v0, v2 - v0);
    const f32 rawLength = Math::length(raw);
    if (rawLength < kEpsilon)
        return false;
    const Math::vec3 faceNormal = raw / rawLength;
    const f32 heightLower = Math::dot(faceNormal, lower - v0);
    const f32 heightUpper = Math::dot(faceNormal, upper - v0);
    const f32 side = heightLower + heightUpper >= 0.0f ? 1.0f : -1.0f;
    const bool lowerDeeper = side * heightLower <= side * heightUpper;
    const f32 deepestHeight = side * (lowerDeeper ? heightLower : heightUpper);
    const Math::vec3 deepestEnd = lowerDeeper ? lower : upper;

    // Normal from the capsule into the triangle: against the way out.
    out.normal = -faceNormal * side;
    out.buildTangents();
    out.count = 1;
    writePoint(out.points[0], closestPointOnTriangle(v0, v1, v2, deepestEnd),
               a.radius() - deepestHeight);
    return true;
}

bool Narrowphase::convexHullTriangle(const ConvexHullShape& a, const Math::mat4& transformA,
                                     const TriangleShape& b, const Math::mat4& transformB,
                                     ContactManifold& out, f32 margin)
{
    if (a.vertices().empty())
        return false;
    const Math::vec3 v0 = Math::vec3(transformB * Math::vec4(b.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transformB * Math::vec4(b.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transformB * Math::vec4(b.vertex(2), 1.0f));

    const Math::vec3 faceNormalRaw = Math::cross(v1 - v0, v2 - v0);
    const f32 faceLength = Math::length(faceNormalRaw);
    if (faceLength < kEpsilon)
        return false;
    const Math::vec3 triangleNormal = faceNormalRaw / faceLength;

    static thread_local ct::Vector<Math::vec3> hullNormals;
    static thread_local ct::Vector<Math::vec3> hullEdges;
    hullFaceNormals(a, transformA, hullNormals);
    hullEdgeDirections(a, transformA, hullEdges);
    const Math::vec3 vertices[3] = {v0, v1, v2};
    const Math::vec3 triangleEdges[3] = {v1 - v0, v2 - v1, v0 - v2};

    enum : u32
    {
        HullFace,
        TriangleFace,
        EdgeEdge
    };
    SatAxis best;
    for (u32 face = 0; face < hullNormals.size(); ++face)
        if (!testAxis(a, transformA, b, transformB, hullNormals[face], margin, HullFace, face, 0,
                      best))
            return false;
    if (!testAxis(a, transformA, b, transformB, triangleNormal, margin, TriangleFace, 0, 0, best,
                  kFeatureAxisBias))
        return false;
    Math::vec3 along[3];
    for (u32 j = 0; j < 3; ++j)
        along[j] = triangleEdges[j] / Math::length(triangleEdges[j]);
    const bool arcs = a.edgeArcsKnown();
    if (arcs)
    {
        // The hull edge's arc must cross the triangle edge's half circle, the one through -outward.
        const ct::Vector<u32>& faces = a.edgeFaces();
        for (u32 i = 0; 2 * i + 1 < faces.size(); ++i)
        {
            const Math::vec3& normalA0 = hullNormals[faces[2 * i]];
            const Math::vec3& normalA1 = hullNormals[faces[2 * i + 1]];
            const Math::vec3 alongA = Math::cross(normalA1, normalA0);
            if (Math::dot(alongA, alongA) < 1e-12f)
                continue;
            for (u32 j = 0; j < 3; ++j)
            {
                if (!(Math::dot(normalA0, along[j]) * Math::dot(normalA1, along[j]) < 0.0f))
                    continue;
                Math::vec3 crossing = Math::cross(alongA, along[j]);
                if (Math::dot(crossing, normalA0 + normalA1) < 0.0f)
                    crossing = -crossing;
                if (!(Math::dot(crossing, Math::cross(along[j], triangleNormal)) < 0.0f))
                    continue;
                if (!testAxis(a, transformA, b, transformB, crossing, margin, EdgeEdge, i, j, best,
                              kFeatureAxisBias))
                    return false;
            }
        }
    }
    else
    {
        for (u32 i = 0; i < hullEdges.size(); ++i)
            for (u32 j = 0; j < 3; ++j)
                if (!testAxis(a, transformA, b, transformB, Math::cross(hullEdges[i], along[j]),
                              margin, EdgeEdge, i, j, best, kFeatureAxisBias))
                    return false;
    }

    out.normal = best.normal;
    out.buildTangents();

    if (best.kind == EdgeEdge)
    {
        Math::vec3 start, end;
        const Math::vec3 direction =
            arcs ? hullEdgeDirection(a, transformA, best.first) : hullEdges[best.first];
        if (hullSupportingEdge(a, transformA, direction, out.normal, start, end))
            edgeEdgeContact(start, end, vertices[best.second], vertices[(best.second + 1) % 3],
                            best.penetration, out);
        else
            supportPointContact(a, transformA, b, transformB, best.penetration, out);
        return true;
    }

    static thread_local ct::Vector<Math::vec3> referencePolygon;
    static thread_local ct::Vector<Math::vec3> incidentPolygon;
    referencePolygon.clear();
    incidentPolygon.clear();
    Math::vec3 referenceNormal(0.0f);

    if (best.kind == HullFace)
    {
        const u32 referenceFace = hullReferenceFace(a, transformA, out.normal);
        referenceNormal = hullFaceNormal(a, transformA, referenceFace);
        hullFacePolygon(a, transformA, referenceFace, referencePolygon);
        for (u32 i = 0; i < 3; ++i)
            incidentPolygon.push_back(vertices[i]);
    }
    else
    {
        referenceNormal = Math::dot(triangleNormal, -out.normal) >= 0.0f ? triangleNormal
                                                                         : -triangleNormal;
        for (u32 i = 0; i < 3; ++i)
            referencePolygon.push_back(vertices[i]);
        hullFacePolygon(a, transformA, hullIncidentFace(a, transformA, referenceNormal),
                        incidentPolygon);
    }

    if (!clipFaceAgainstPolygon(referencePolygon.data(), static_cast<u32>(referencePolygon.size()),
                                referenceNormal, incidentPolygon.data(),
                                static_cast<u32>(incidentPolygon.size()), margin, out))
        supportPointContact(a, transformA, b, transformB, best.penetration, out);
    return true;
}

bool Narrowphase::convexTrimesh(const CollisionShape& convex, const Math::mat4& convexTransform,
                                const TrimeshShape& mesh, const Math::mat4& meshTransform,
                                ct::Vector<ContactManifold>& out, f32 margin,
                                const AABB* convexBounds)
{
    // The convex's world bounds in the mesh's own space, where the tree was
    // built: cheaper than moving every triangle out.
    AABB worldBox = convexBounds ? *convexBounds : convex.bounds(convexTransform);
    worldBox.min -= Math::vec3(margin);
    worldBox.max += Math::vec3(margin);

    // A body transform is rotation and translation only, so the inverse is
    // the transpose, not a general 4x4 inverse (this runs per pair per
    // substep).
    const Math::mat3 rotation(meshTransform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    Math::mat4 inverseTransform(inverseRotation);
    inverseTransform[3] = Math::vec4(-(inverseRotation * Math::vec3(meshTransform[3])), 1.0f);
    const AABB localBox = transformAABB(worldBox, inverseTransform);

    // Reused rather than allocated per call.
    static thread_local ct::Vector<u32> candidates;
    mesh.query(localBox, candidates);
    if (candidates.empty())
        return false;

    const usize before = out.size();
    static thread_local ct::Vector<ContactManifold> raw;
    raw.clear();
    for (u32 index : candidates)
    {
        const TriangleShape triangle = mesh.triangle(index);
        ContactManifold manifold;
        bool hit = false;
        switch (convex.type())
        {
        case ShapeType::Sphere:
            hit = sphereTriangle(static_cast<const SphereShape&>(convex), convexTransform, triangle,
                                 meshTransform, manifold, margin);
            break;
        case ShapeType::Box:
            hit = boxTriangle(static_cast<const BoxShape&>(convex), convexTransform, triangle,
                              meshTransform, manifold, margin);
            break;
        case ShapeType::Capsule:
            hit = capsuleTriangle(static_cast<const CapsuleShape&>(convex), convexTransform,
                                  triangle, meshTransform, manifold, margin);
            break;
        case ShapeType::ConvexHull:
            hit = convexHullTriangle(static_cast<const ConvexHullShape&>(convex), convexTransform,
                                     triangle, meshTransform, manifold, margin);
            break;
        case ShapeType::Cylinder:
        {
            hit = collideCylinder(convex, convexTransform, triangle, meshTransform, manifold,
                                  margin);
            break;
        }
        default:
            break;
        }
        if (!hit)
            continue;
        if (mesh.oneSided())
        {
            const Math::vec3 front = Math::mat3(meshTransform) * triangle.rawNormal();
            if (Math::dot(manifold.normal, front) > 0.0f)
                continue;
        }
        raw.push_back(manifold);
    }

    // One manifold per normal: the triangles of a floor under one body make one
    // patch of at most four points, not a patch each.
    constexpr f32 kSameNormal = 0.985f;
    static thread_local ct::Vector<Math::vec3> positions;
    static thread_local ct::Vector<f32> depths;
    static thread_local ct::Vector<u8> used;
    used.assign(raw.size(), 0);
    for (usize i = 0; i < raw.size(); ++i)
    {
        if (used[i])
            continue;
        used[i] = 1;
        positions.clear();
        depths.clear();
        const Math::vec3 normal = raw[i].normal;
        for (usize j = i; j < raw.size(); ++j)
        {
            if (j != i && (used[j] || Math::dot(raw[j].normal, normal) < kSameNormal))
                continue;
            used[j] = 1;
            for (u32 k = 0; k < raw[j].count; ++k)
            {
                positions.push_back(raw[j].points[k].position);
                depths.push_back(raw[j].points[k].penetration);
            }
        }
        ContactManifold merged;
        merged.normal = normal;
        merged.buildTangents();
        merged.count = reducePoints(positions.data(), depths.data(),
                                    static_cast<u32>(positions.size()), merged);
        for (u32 k = 0; k < merged.count; ++k)
        {
            merged.points[k].normalImpulse = 0.0f;
            merged.points[k].tangentImpulse[0] = 0.0f;
            merged.points[k].tangentImpulse[1] = 0.0f;
        }
        out.push_back(merged);
    }
    return out.size() > before;
}

namespace
{

bool raycastPlane(const PlaneShape& shape, const Math::mat4& transform, const Ray& ray,
                  f32 maxDistance, ShapeRayHit& hit)
{
    const Math::vec3 normal = Math::normalize(Math::mat3(transform) * shape.normal());
    const Math::vec3 origin =
        Math::vec3(transform * Math::vec4(shape.normal() * shape.constant(), 1.0f));
    Plane plane;
    plane.normal = normal;
    plane.d = -Math::dot(normal, origin);
    f32 distance = 0.0f;
    if (!ray.intersects(plane, distance) || distance > maxDistance)
        return false;
    hit.distance = distance;
    hit.point = ray.at(distance);
    hit.normal = Math::dot(ray.direction, normal) < 0.0f ? normal : -normal;
    return true;
}

// The ray against a sphere, in the ray's own parameter (origin + direction *
// t, whatever the direction's length), the unit all the raycasts here report
// in. Ray::intersects(Sphere) assumes a unit direction. From inside, the way
// out.
bool raySphere(const Math::vec3& origin, const Math::vec3& direction, const Math::vec3& centre,
               f32 radius, f32& t)
{
    const f32 a = Math::dot(direction, direction);
    if (!(a > 1e-24f))
        return false;
    const Math::vec3 delta = origin - centre;
    const f32 b = Math::dot(delta, direction);
    const f32 c = Math::dot(delta, delta) - radius * radius;
    const f32 discriminant = b * b - a * c;
    if (discriminant < 0.0f)
        return false;
    const f32 root = sqrtf(discriminant);
    f32 hit = (-b - root) / a;
    if (hit < 0.0f)
        hit = (-b + root) / a;
    if (hit < 0.0f)
        return false;
    t = hit;
    return true;
}

bool raycastSphere(const SphereShape& sphere, const Math::mat4& transform, const Ray& ray,
                   f32 maxDistance, ShapeRayHit& hit)
{
    const Math::vec3 centre(transform[3]);
    f32 t = 0.0f;
    if (!raySphere(ray.origin, ray.direction, centre, sphere.radius(), t) || t > maxDistance)
        return false;

    hit.distance = t;
    hit.point = ray.at(t);
    const Math::vec3 offset = hit.point - centre;
    const f32 length = Math::length(offset);
    hit.normal = length > kEpsilon ? offset / length : Math::vec3(0.0f, 1.0f, 0.0f);
    return true;
}

bool raycastBox(const BoxShape& box, const Math::mat4& transform, const Ray& ray, f32 maxDistance,
                ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    const Math::vec3& half = box.halfExtents();
    AABB local;
    local.min = -half;
    local.max = half;

    f32 t = 0.0f;
    if (!localRay.intersects(local, t) || t < 0.0f || t > maxDistance)
        return false;

    const Math::vec3 localPoint = localRay.at(t);
    Math::vec3 localNormal(0.0f);
    f32 bestGap = FLT_MAX;
    for (u32 axis = 0; axis < 3; ++axis)
    {
        const f32 gap = half[axis] - fabsf(localPoint[axis]);
        if (gap < bestGap)
        {
            bestGap = gap;
            localNormal = Math::vec3(0.0f);
            localNormal[axis] = localPoint[axis] >= 0.0f ? 1.0f : -1.0f;
        }
    }

    hit.distance = t;
    hit.point = center + rotation * localPoint;
    hit.normal = rotation * localNormal;
    return true;
}

bool raycastCapsule(const CapsuleShape& capsule, const Math::mat4& transform, const Ray& ray,
                    f32 maxDistance, ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    const f32 radius = capsule.radius();
    const f32 halfHeight = capsule.halfHeight();
    const Math::vec3& o = localRay.origin;
    const Math::vec3& d = localRay.direction;

    bool found = false;
    f32 nearest = maxDistance;
    Math::vec3 localPoint(0.0f);
    Math::vec3 localNormal(0.0f, 1.0f, 0.0f);

    const f32 a = d.x * d.x + d.z * d.z;
    if (a > kEpsilon)
    {
        const f32 b = 2.0f * (o.x * d.x + o.z * d.z);
        const f32 c = o.x * o.x + o.z * o.z - radius * radius;
        const f32 discriminant = b * b - 4.0f * a * c;
        if (discriminant >= 0.0f)
        {
            const f32 root = sqrtf(discriminant);
            const f32 roots[2] = {(-b - root) / (2.0f * a), (-b + root) / (2.0f * a)};
            for (f32 t : roots)
            {
                if (t < 0.0f || t >= nearest)
                    continue;
                const f32 y = o.y + d.y * t;
                if (y < -halfHeight || y > halfHeight)
                    continue;
                nearest = t;
                localPoint = o + d * t;
                localNormal = Math::normalize(Math::vec3(localPoint.x, 0.0f, localPoint.z));
                found = true;
            }
        }
    }

    // The end caps in the same ray parameter as the side above.
    for (f32 capY : {-halfHeight, halfHeight})
    {
        const Math::vec3 capCentre(0.0f, capY, 0.0f);
        f32 t = 0.0f;
        if (raySphere(o, d, capCentre, radius, t) && t < nearest)
        {
            nearest = t;
            localPoint = localRay.at(t);
            localNormal = Math::normalize(localPoint - capCentre);
            found = true;
        }
    }

    if (!found)
        return false;

    hit.distance = nearest;
    hit.point = center + rotation * localPoint;
    hit.normal = rotation * localNormal;
    return true;
}

bool raycastCylinder(const CylinderShape& cylinder, const Math::mat4& transform, const Ray& ray,
                     f32 maxDistance, ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);
    const Math::vec3 o = inverseRotation * (ray.origin - center);
    const Math::vec3 d = inverseRotation * ray.direction;
    const f32 radius = cylinder.radius();
    const f32 halfHeight = cylinder.halfHeight();

    bool found = false;
    f32 nearest = maxDistance;
    Math::vec3 localNormal(0.0f, 1.0f, 0.0f);

    const f32 a = d.x * d.x + d.z * d.z;
    if (a > kEpsilon)
    {
        const f32 b = 2.0f * (o.x * d.x + o.z * d.z);
        const f32 c = o.x * o.x + o.z * o.z - radius * radius;
        const f32 discriminant = b * b - 4.0f * a * c;
        if (discriminant >= 0.0f)
        {
            const f32 root = sqrtf(discriminant);
            const f32 roots[2] = {(-b - root) / (2.0f * a), (-b + root) / (2.0f * a)};
            for (f32 t : roots)
            {
                if (t < 0.0f || t >= nearest)
                    continue;
                const f32 y = o.y + d.y * t;
                if (y < -halfHeight || y > halfHeight)
                    continue;
                nearest = t;
                const Math::vec3 point = o + d * t;
                localNormal = Math::normalize(Math::vec3(point.x, 0.0f, point.z));
                found = true;
            }
        }
    }

    if (fabsf(d.y) > kEpsilon)
    {
        for (f32 cap : {-halfHeight, halfHeight})
        {
            const f32 t = (cap - o.y) / d.y;
            if (t < 0.0f || t >= nearest)
                continue;
            const f32 x = o.x + d.x * t;
            const f32 z = o.z + d.z * t;
            if (x * x + z * z > radius * radius)
                continue;
            nearest = t;
            localNormal = Math::vec3(0.0f, cap > 0.0f ? 1.0f : -1.0f, 0.0f);
            found = true;
        }
    }

    if (!found)
        return false;

    hit.distance = nearest;
    hit.point = ray.origin + ray.direction * nearest;
    hit.normal = rotation * localNormal;
    return true;
}

bool raycastTriangle(const TriangleShape& triangle, const Math::mat4& transform, const Ray& ray,
                     f32 maxDistance, ShapeRayHit& hit)
{
    const Math::vec3 v0 = Math::vec3(transform * Math::vec4(triangle.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transform * Math::vec4(triangle.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transform * Math::vec4(triangle.vertex(2), 1.0f));

    f32 t = 0.0f;
    if (!ray.intersects(v0, v1, v2, t) || t > maxDistance)
        return false;

    hit.distance = t;
    hit.point = ray.at(t);
    const Math::vec3 raw = Math::cross(v1 - v0, v2 - v0);
    const f32 length = Math::length(raw);
    hit.normal = length > kEpsilon ? raw / length : Math::vec3(0.0f, 1.0f, 0.0f);
    if (Math::dot(hit.normal, ray.direction) > 0.0f)
        hit.normal = -hit.normal;
    return true;
}

bool raycastTrimesh(const TrimeshShape& mesh, const Math::mat4& transform, const Ray& ray,
                    f32 maxDistance, ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    TrimeshShape::RayHit localHit;
    if (!mesh.raycast(localRay, maxDistance, localHit))
        return false;

    hit.distance = localHit.distance;
    hit.point = center + rotation * localHit.point;
    hit.normal = rotation * localHit.normal;
    hit.triangle = static_cast<s32>(localHit.triangle);
    return true;
}

// Outward plane (xyz = normal, w = offset, inside when dot(n, p) + w <= 0) of
// every face of a convex hull. The vertex average sits inside, so it decides
// which way each face looks regardless of winding.
void convexHullFacePlanes(const ConvexHullShape& hull, ct::Vector<Math::vec4>& planes)
{
    planes.clear();
    const ct::Vector<Math::vec3>& vertices = hull.vertices();
    const ct::Vector<ConvexHullShape::Edge>& edges = hull.edges();
    const ct::Vector<int>& faces = hull.faces();
    if (vertices.empty())
        return;

    Math::vec3 average(0.0f);
    for (const Math::vec3& vertex : vertices)
        average += vertex;
    average /= static_cast<f32>(vertices.size());

    for (u32 face = 0; face < faces.size(); ++face)
    {
        const ConvexHullShape::Edge* start = &edges[static_cast<u32>(faces[face])];
        const ConvexHullShape::Edge* edge = start;
        Math::vec3 normal(0.0f);
        Math::vec3 first(0.0f);
        u32 count = 0;
        do
        {
            const Math::vec3& current = vertices[static_cast<u32>(edge->getSourceVertex())];
            const Math::vec3& next = vertices[static_cast<u32>(edge->getTargetVertex())];
            if (count == 0)
                first = current;
            normal.x += (current.y - next.y) * (current.z + next.z);
            normal.y += (current.z - next.z) * (current.x + next.x);
            normal.z += (current.x - next.x) * (current.y + next.y);
            edge = edge->getNextEdgeOfFace();
            ++count;
        } while (edge != start && edge != nullptr && count < 4096);

        const f32 length = Math::length(normal);
        if (count < 3 || length < 1e-12f)
            continue;
        normal /= length;
        f32 offset = -Math::dot(normal, first);
        if (Math::dot(normal, average) + offset > 0.0f)
        {
            normal = -normal;
            offset = -offset;
        }
        planes.push_back(Math::vec4(normal, offset));
    }
}

bool raycastConvexHull(const ConvexHullShape& hull, const Math::mat4& transform, const Ray& ray,
                       f32 maxDistance, ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);
    const Math::vec3 origin = inverseRotation * (ray.origin - center);
    const Math::vec3 direction = inverseRotation * ray.direction;

    static thread_local ct::Vector<Math::vec4> planes;
    convexHullFacePlanes(hull, planes);
    if (planes.empty())
        return false;

    f32 enter = 0.0f;
    f32 exit = maxDistance;
    Math::vec3 enterNormal(0.0f, 1.0f, 0.0f);
    bool entered = false;
    for (const Math::vec4& plane : planes)
    {
        const Math::vec3 normal(plane.x, plane.y, plane.z);
        const f32 denominator = Math::dot(normal, direction);
        const f32 distance = Math::dot(normal, origin) + plane.w;
        if (fabsf(denominator) < 1e-9f)
        {
            if (distance > 0.0f)
                return false;
            continue;
        }
        const f32 t = -distance / denominator;
        if (denominator < 0.0f)
        {
            if (t > enter)
            {
                enter = t;
                enterNormal = normal;
                entered = true;
            }
        }
        else if (t < exit)
        {
            exit = t;
        }
        if (enter > exit)
            return false;
    }
    if (!entered)
        return false;

    hit.distance = enter;
    hit.point = center + rotation * (origin + direction * enter);
    hit.normal = rotation * enterNormal;
    return true;
}

bool overlapSphereShape(const SphereShape& sphere, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::vec3 center(transform[3]);
    const f32 total = sphere.radius() + radius;
    return Math::dot(centre - center, centre - center) <= total * total;
}

bool overlapSphereShape(const BoxShape& box, const Math::mat4& transform, const Math::vec3& centre,
                        f32 radius)
{
    const Math::mat3 rotation(transform);
    const Math::vec3 boxCenter(transform[3]);
    const Math::vec3 local = Math::transpose(rotation) * (centre - boxCenter);
    const Math::vec3 closest = Math::clamp(local, -box.halfExtents(), box.halfExtents());
    const Math::vec3 offset = local - closest;
    return Math::dot(offset, offset) <= radius * radius;
}

bool overlapSphereShape(const CapsuleShape& capsule, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    Math::vec3 lower, upper;
    capsule.segment(transform, lower, upper);
    const Math::vec3 closest = closestPointOnSegment(lower, upper, centre);
    const f32 total = capsule.radius() + radius;
    return Math::dot(centre - closest, centre - closest) <= total * total;
}

bool overlapSphereShape(const CylinderShape& cylinder, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::mat3 inverseRotation = Math::transpose(Math::mat3(transform));
    const Math::vec3 local = inverseRotation * (centre - Math::vec3(transform[3]));
    const f32 radial = Math::max(0.0f, sqrtf(local.x * local.x + local.z * local.z) - cylinder.radius());
    const f32 axial = Math::max(0.0f, fabsf(local.y) - cylinder.halfHeight());
    return radial * radial + axial * axial <= radius * radius;
}

bool overlapSphereShape(const TriangleShape& triangle, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::vec3 v0 = Math::vec3(transform * Math::vec4(triangle.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transform * Math::vec4(triangle.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transform * Math::vec4(triangle.vertex(2), 1.0f));
    const Math::vec3 closest = closestPointOnTriangle(v0, v1, v2, centre);
    const Math::vec3 offset = closest - centre;
    return Math::dot(offset, offset) <= radius * radius;
}

bool overlapSphereShape(const TrimeshShape& mesh, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::mat3 rotation(transform);
    const Math::vec3 center(transform[3]);
    const Math::vec3 localCentre = Math::transpose(rotation) * (centre - center);

    static thread_local ct::Vector<u32> triangles;
    mesh.overlapSphere(localCentre, radius, triangles);
    return !triangles.empty();
}

bool overlapSphereShape(const ConvexHullShape& hull, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::mat3 rotation(transform);
    const Math::vec3 local = Math::transpose(rotation) * (centre - Math::vec3(transform[3]));
    Math::vec3 closest;
    bool inside = false;
    const f32 squared = closestPointOnPolytope(hullSurface(hull), local, closest, inside);
    return squared != FLT_MAX && squared <= radius * radius;
}

bool overlapSphereShape(const PlaneShape& plane, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::vec3 normal = Math::normalize(Math::mat3(transform) * plane.normal());
    const Math::vec3 origin =
        Math::vec3(transform * Math::vec4(plane.normal() * plane.constant(), 1.0f));
    return Math::dot(normal, centre - origin) <= radius;
}

} // namespace

bool Narrowphase::raycast(const CollisionShape& shape, const Math::mat4& transform, const Ray& ray,
                          f32 maxDistance, ShapeRayHit& hit)
{
    switch (shape.type())
    {
    case ShapeType::Sphere:
        return raycastSphere(static_cast<const SphereShape&>(shape), transform, ray, maxDistance,
                             hit);
    case ShapeType::Box:
        return raycastBox(static_cast<const BoxShape&>(shape), transform, ray, maxDistance, hit);
    case ShapeType::Capsule:
        return raycastCapsule(static_cast<const CapsuleShape&>(shape), transform, ray, maxDistance,
                              hit);
    case ShapeType::Plane:
        return raycastPlane(static_cast<const PlaneShape&>(shape), transform, ray, maxDistance, hit);
    case ShapeType::Triangle:
        return raycastTriangle(static_cast<const TriangleShape&>(shape), transform, ray,
                               maxDistance, hit);
    case ShapeType::Trimesh:
        return raycastTrimesh(static_cast<const TrimeshShape&>(shape), transform, ray, maxDistance,
                              hit);
    case ShapeType::ConvexHull:
        return raycastConvexHull(static_cast<const ConvexHullShape&>(shape), transform, ray,
                                 maxDistance, hit);
    case ShapeType::Cylinder:
        return raycastCylinder(static_cast<const CylinderShape&>(shape), transform, ray,
                               maxDistance, hit);
    default:
        return false;
    }
}

bool Narrowphase::overlapSphere(const CollisionShape& shape, const Math::mat4& transform,
                                const Math::vec3& centre, f32 radius)
{
    switch (shape.type())
    {
    case ShapeType::Sphere:
        return overlapSphereShape(static_cast<const SphereShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::Box:
        return overlapSphereShape(static_cast<const BoxShape&>(shape), transform, centre, radius);
    case ShapeType::Capsule:
        return overlapSphereShape(static_cast<const CapsuleShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::Plane:
        return overlapSphereShape(static_cast<const PlaneShape&>(shape), transform, centre, radius);
    case ShapeType::Triangle:
        return overlapSphereShape(static_cast<const TriangleShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::Trimesh:
        return overlapSphereShape(static_cast<const TrimeshShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::ConvexHull:
        return overlapSphereShape(static_cast<const ConvexHullShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::Cylinder:
        return overlapSphereShape(static_cast<const CylinderShape&>(shape), transform, centre,
                                  radius);
    default:
        return false;
    }
}

} // namespace kinetix::Physics
