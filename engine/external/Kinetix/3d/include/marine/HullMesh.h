#ifndef KINETIX_PHYSICS_MARINE_HULLMESH_H
#define KINETIX_PHYSICS_MARINE_HULLMESH_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Geometry
{
class ConvexHullComputer;
}

namespace kinetix::Physics
{

// Parametric planing / displacement hull of HullMesh::boat(). Boat body axes: +Z forward,
// +Y up, starboard (right) on -X; centred on the origin along Z, keel at midship at y = 0.
// Each section is a V bottom from the keel to the chine at `deadrise`, then the topside to
// the sheer; bilgeRadius > 0 rounds the chine. Half-beam blends from the transom's to the
// full beam by the middle and closes at the stem, the keel rises forward (forefoot), and
// deadrise grows from the transom's to the bow's (deep V forward).
struct BoatHullParameters
{
    f32 length = 6.0f;             // overall, transom to stem
    f32 beam = 2.2f;               // maximum, at the sheer
    f32 depth = 1.1f;              // keel to sheer at midship
    f32 transomBeamFraction = 0.85f;
    f32 chineBeamFraction = 0.9f;  // chine half-beam / sheer half-beam
    f32 deadrise = 0.31f;          // radians at the transom (~18 deg)
    f32 bowDeadrise = 0.7f;        // radians at the stem (~40 deg)
    f32 bilgeRadius = 0.0f;        // m; 0 = hard chine
    f32 forefootRise = 0.55f;      // keel height at the stem, share of depth
    f32 forefootStart = 0.55f;     // where the keel starts to rise, share of length from aft
    f32 sheerRise = 0.15f;         // extra sheer height at the stem, m
    u32 stations = 18;             // sections along the length, 3..128
    u32 sectionPoints = 6;         // points per side between keel and sheer, 2..64
};

// A closed triangle mesh in body space of the hull's outer surface, independent of any
// collision shape.
//
// Triangles are wound counter-clockwise seen from outside (right-hand normal points out,
// the direction pressure pushes against). set() fixes a mesh wound the other way as a
// whole (negative volume) but not individual triangles wound against their neighbours;
// openEdgeCount() counts the edges those leave.
//
// Volume, centroid and area use the divergence theorem: exact for a closed mesh,
// meaningless for an open one (isClosed(): every edge shared by exactly two triangles in
// opposite directions).
class HullMesh
{
public:
    HullMesh() = default;
    HullMesh(const Math::vec3* vertices, u32 vertexCount, const u32* indices, u32 indexCount);

    // Copies the arrays. Triangles with an out-of-range index or a repeated vertex are dropped
    // and reported. Returns false if nothing usable is left.
    bool set(const Math::vec3* vertices, u32 vertexCount, const u32* indices, u32 indexCount);

    // Merges vertices closer than `tolerance` (on a grid of that size) and drops collapsed
    // triangles; for renderer meshes, which split vertices along hard edges and never look closed.
    void weldVertices(f32 tolerance = 1.0e-5f);
    void translate(const Math::vec3& offset);
    void rotate(const Math::quat& rotation);

    u32 vertexCount() const
    {
        return static_cast<u32>(mVertices.size());
    }
    u32 triangleCount() const
    {
        return static_cast<u32>(mIndices.size() / 3);
    }
    const ct::Vector<Math::vec3>& vertices() const
    {
        return mVertices;
    }
    const ct::Vector<u32>& indices() const
    {
        return mIndices;
    }
    // Unit outward normal (zero for a degenerate triangle) and area of one triangle.
    const Math::vec3& triangleNormal(u32 triangle) const
    {
        return mNormals[triangle];
    }
    f32 triangleArea(u32 triangle) const
    {
        return mAreas[triangle];
    }

    f32 volume() const
    {
        return mVolume;
    }
    // Centre of the enclosed volume: the centre of buoyancy when fully submerged.
    const Math::vec3& centroid() const
    {
        return mCentroid;
    }
    f32 surfaceArea() const
    {
        return mSurfaceArea;
    }
    const Math::vec3& boundsMin() const
    {
        return mBoundsMin;
    }
    const Math::vec3& boundsMax() const
    {
        return mBoundsMax;
    }
    // Extent along a body direction (made unit).
    f32 extent(const Math::vec3& axis) const;

    bool isClosed() const;
    // Edges used by only one triangle, or twice in the same direction.
    u32 openEdgeCount() const;

    // Generators; every one returns a closed, outward-wound mesh.
    // Box centred on the origin, each face cut into `subdivisions` x `subdivisions` quads
    // (1..32); resolves a wave or drag distribution better.
    static HullMesh box(const Math::vec3& halfExtents, u32 subdivisions = 1);
    // UV sphere: `rings` (2..128) bands of latitude, `segments` (3..256) of longitude.
    // Inscribed, so volume() is slightly below 4/3 pi r^3.
    static HullMesh sphere(f32 radius, u32 rings = 16, u32 segments = 32);
    static HullMesh fromConvexHull(const kinetix::Geometry::ConvexHullComputer& hull);
    static HullMesh boat(const BoatHullParameters& parameters);

private:
    void rebuild();

    ct::Vector<Math::vec3> mVertices;
    ct::Vector<u32> mIndices;
    ct::Vector<Math::vec3> mNormals;
    ct::Vector<f32> mAreas;
    f32 mVolume = 0.0f;
    f32 mSurfaceArea = 0.0f;
    Math::vec3 mCentroid{0.0f};
    Math::vec3 mBoundsMin{0.0f};
    Math::vec3 mBoundsMax{0.0f};
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_HULLMESH_H
