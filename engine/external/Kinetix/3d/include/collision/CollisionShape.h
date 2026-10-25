#ifndef KINETIX_PHYSICS_COLLISION_SHAPE_H
#define KINETIX_PHYSICS_COLLISION_SHAPE_H

#include <kinetix/core/BoundsTree.h>
#include <kinetix/core/Color.h>
#include "geometry/ConvexHullComputer.h"
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Geometry
{
struct Shard;
}

namespace kinetix::Physics
{

enum class ShapeType : u8
{
    Sphere,
    Box,
    Capsule,
    Plane,
    Triangle,
    Trimesh,
    ConvexHull,
    Cylinder,
    Count
};

// Pure geometry in the body's own space. A shape holds no transform and no
// body pointer, so one shape serves any number of bodies and threads.
class CollisionShape
{
public:
    virtual ~CollisionShape() = default;

    virtual ShapeType type() const = 0;

    // Moment of inertia about the centre of mass, body space.
    virtual Math::mat3 inertia(f32 mass) const = 0;

    // World AABB under `transform`.
    virtual AABB bounds(const Math::mat4& transform) const = 0;

    // Furthest point of the shape along `direction` (world space, direction
    // need not be normalized). The one primitive SAT projection is built on.
    virtual Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const = 0;

    // Extent of the shape's shadow on `axis`, world space.
    virtual void project(const Math::mat4& transform, const Math::vec3& axis, f32& minimum,
                         f32& maximum) const;

    virtual void debugDraw(const Math::mat4& transform, Color color) const = 0;
};

class SphereShape final : public CollisionShape
{
public:
    explicit SphereShape(f32 radius);

    ShapeType type() const override
    {
        return ShapeType::Sphere;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    f32 radius() const
    {
        return mRadius;
    }

private:
    f32 mRadius;
};

class BoxShape final : public CollisionShape
{
public:
    explicit BoxShape(const Math::vec3& halfExtents);

    ShapeType type() const override
    {
        return ShapeType::Box;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const Math::vec3& halfExtents() const
    {
        return mHalfExtents;
    }
    // The eight corners in world space, in the order the face table indexes them.
    void corners(const Math::mat4& transform, Math::vec3 out[8]) const;
    // Outward normal and the four corner indices of face `index`, 0..5.
    static const u8* faceCorners(u32 index);
    static Math::vec3 faceNormal(const Math::mat4& transform, u32 index);

private:
    Math::vec3 mHalfExtents;
};

// A segment along the body's local Y with a radius around it. Everything a
// capsule does is the sphere case applied to the closest point on the segment.
class CapsuleShape final : public CollisionShape
{
public:
    // `halfHeight` is half the length of the SEGMENT, not of the whole
    // capsule: the total height is 2*(halfHeight + radius).
    CapsuleShape(f32 radius, f32 halfHeight);

    ShapeType type() const override
    {
        return ShapeType::Capsule;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    f32 radius() const
    {
        return mRadius;
    }
    f32 halfHeight() const
    {
        return mHalfHeight;
    }
    // The two ends of the inner segment, world space.
    void segment(const Math::mat4& transform, Math::vec3& lower, Math::vec3& upper) const;

private:
    f32 mRadius;
    f32 mHalfHeight;
};

class PlaneShape final : public CollisionShape
{
public:
    PlaneShape(const Math::vec3& normal, f32 constant = 0.0f);

    ShapeType type() const override
    {
        return ShapeType::Plane;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const Math::vec3& normal() const
    {
        return mNormal;
    }
    f32 constant() const
    {
        return mConstant;
    }

private:
    Math::vec3 mNormal;
    f32 mConstant;
};

// Which part of a triangle a closest-point query landed on. Edge i runs from
// vertex i to vertex i+1.
enum class TriangleFeature : u8
{
    Face,
    Edge0,
    Edge1,
    Edge2,
    Vertex0,
    Vertex1,
    Vertex2
};

// One triangle of a TrimeshShape, in that mesh's own space. Built and thrown
// away inside the trimesh narrowphase; it has no volume, mass or inertia.
class TriangleShape final : public CollisionShape
{
public:
    TriangleShape(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c, u8 sharedEdges = 0);

    ShapeType type() const override
    {
        return ShapeType::Triangle;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const Math::vec3& vertex(u32 index) const
    {
        return mVertices[index];
    }
    // Unnormalized when the triangle is degenerate - callers check the length
    // rather than trusting a normalize() of nothing.
    Math::vec3 rawNormal() const;

    // An edge another triangle also uses is inside the surface, not on its
    // rim. A contact there must be pushed out along the face, never along the
    // edge, or a character walking over a seam stops dead on flat ground.
    bool edgeIsShared(u32 edge) const
    {
        return (mSharedEdges & (1u << edge)) != 0;
    }
    // True when the normal for `feature` has to be taken from the face rather
    // than from the closest-point direction.
    bool featureIsInternal(TriangleFeature feature) const;

private:
    Math::vec3 mVertices[3];
    u8 mSharedEdges = 0;
};

// A static concave mesh. Dynamics has no meaning for one - a concave body has
// no single inertia tensor - so a body carrying this must be static or
// kinematic, which PhysicsWorld enforces.
class TrimeshShape final : public CollisionShape
{
public:
    // Copies both arrays: the tree indexes into this copy.
    //
    // A triangle with an index past the end of `vertices` is dropped here
    // with a logged error, so queries can index the vertex array unchecked.
    // Triangle numbers (triangle(), RayHit, SweepHit) count the triangles kept.
    TrimeshShape(const Math::vec3* vertices, u32 vertexCount, const u32* indices, u32 indexCount);

    ShapeType type() const override
    {
        return ShapeType::Trimesh;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    // One line per triangle, from its centre along the normal the WINDING
    // gives, which is the one collision uses; the mesh's stored (render)
    // normals can disagree.
    void debugDrawFaceNormals(const Math::mat4& transform, f32 length, Color color,
                              Color flippedColor) const;

    // Triangles wound against their neighbours: two triangles sharing an edge
    // should traverse it in opposite directions, else one faces backwards and
    // a one-sided sweep passes through it.
    void windingErrors(ct::Vector<u32>& out) const;

    // Collide with the winding's front only: a contact that would push a body
    // towards the back of a triangle is dropped. For meshes wound consistently.
    void setOneSided(bool oneSided)
    {
        mOneSided = oneSided;
    }
    bool oneSided() const
    {
        return mOneSided;
    }

    u32 triangleCount() const
    {
        return static_cast<u32>(mIndices.size() / 3);
    }
    TriangleShape triangle(u32 index) const;
    // Triangles whose bounds meet `box`, expressed in this mesh's own space.
    void query(const AABB& localBox, ct::Vector<u32>& out) const;

    // Geometry queries for everything that is not the solver (picking, line of
    // sight, prop placement), all in this mesh's own space.
    struct RayHit
    {
        u32 triangle = 0;
        f32 distance = 0.0f;
        Math::vec3 point{0.0f};
        // Face normal, unit length, as wound in the mesh.
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
    };

    struct SweepHit
    {
        u32 triangle = 0;
        // Fraction of `velocity` travelled before contact, in [0, 1]. Negative
        // means the shape already overlapped, and the value is the depth.
        f32 t = 0.0f;
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
    };

    // Moves an ellipsoid of `radii` from `centre` along `velocity` and reports
    // the first thing it meets: a sweep rather than a push-out, since a
    // character needs "how far can I go" and "what did I land on".
    bool sweepEllipsoid(const Math::vec3& localCentre, const Math::vec3& radii,
                        const Math::vec3& velocity, SweepHit& hit) const;
    bool sweepSphere(const Math::vec3& localCentre, f32 radius, const Math::vec3& velocity,
                     SweepHit& hit) const;

    // Third-person camera collision: sweeps a sphere of `radius` from `from`
    // (the camera anchor) to `to` (the desired position) and returns where the
    // camera can sit; a wall pulls it back to the first contact. `margin`
    // keeps the sphere clear of the surface.
    Math::vec3 slideCamera(const Math::vec3& from, const Math::vec3& to, f32 radius,
                          f32 margin = 0.02f) const;

    bool raycast(const Ray& localRay, f32 maxDistance, RayHit& hit) const;
    // Triangles actually within `radius` of `centre`, not merely whose boxes
    // are - the tree narrows it down, this confirms.
    void overlapSphere(const Math::vec3& localCentre, f32 radius, ct::Vector<u32>& out) const;

private:
    void buildAdjacency();

    ct::Vector<Math::vec3> mVertices;
    ct::Vector<u32> mIndices;
    // Three bits per triangle: edge i is shared with another triangle.
    ct::Vector<u8> mSharedEdges;
    BoundsTree mTree;
    AABB mBounds;
    bool mOneSided = false;
};

// An arbitrary convex polyhedron: the vertices/edges/faces of a
// ConvexHullComputer half-edge mesh, kept as flat arrays so the shape
// outlives whatever Shard or ConvexHullComputer it was built from. Meant for
// shatter debris: a handful of faces and tens of vertices.
class ConvexHullShape final : public CollisionShape
{
public:
    using Edge = kinetix::Geometry::ConvexHullComputer::Edge;

    // `faces` holds one edge index per face, matching ConvexHullComputer's
    // own convention - walking that edge with getNextEdgeOfFace() visits the
    // whole face in order.
    ConvexHullShape(const Math::vec3* vertices, u32 vertexCount, const Edge* edges, u32 edgeCount,
                    const int* faces, u32 faceCount);
    // Copies a shatter shard's hull; its vertices are already relative to its
    // centroid, the local space a CollisionShape is defined in.
    explicit ConvexHullShape(const kinetix::Geometry::Shard& shard);

    ShapeType type() const override
    {
        return ShapeType::ConvexHull;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void project(const Math::mat4& transform, const Math::vec3& axis, f32& minimum,
                 f32& maximum) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const ct::Vector<Math::vec3>& vertices() const
    {
        return mVertices;
    }
    const ct::Vector<Edge>& edges() const
    {
        return mEdges;
    }
    const ct::Vector<int>& faces() const
    {
        return mFaces;
    }
    u32 faceCount() const
    {
        return static_cast<u32>(mFaces.size());
    }

    // Everything below is derived once by the constructor from the three
    // arrays above, so the narrowphase never walks a half-edge loop.
    //
    // Face f's corners, as vertex indices in the half-edge loop's order:
    // faceVertices()[faceStart()[f]] up to faceVertices()[faceStart()[f + 1]].
    const ct::Vector<u32>& faceStart() const
    {
        return mFaceStart;
    }
    const ct::Vector<u32>& faceVertices() const
    {
        return mFaceVertices;
    }
    // Outward unit normal of each face, own space, from the whole loop rather
    // than its first three corners. ZERO for a face that encloses no area -
    // the one "face" ConvexHullComputer makes of a set of collinear points -
    // and every caller skips those.
    const ct::Vector<Math::vec3>& faceNormals() const
    {
        return mFaceNormals;
    }
    const ct::Vector<u32>& edgeVertices() const
    {
        return mEdgeVertices;
    }
    // One unit direction per set of parallel edges, own space. Parallel edges
    // give the same SAT axes, and a prism has as many of them as it has sides.
    const ct::Vector<Math::vec3>& edgeDirections() const
    {
        return mEdgeDirections;
    }
    // The face either side of each edge of edgeVertices(), two per edge.
    const ct::Vector<u32>& edgeFaces() const
    {
        return mEdgeFaces;
    }
    bool edgeArcsKnown() const
    {
        return mEdgeArcsKnown;
    }
    // Whether the hull encloses a volume (at least four faces with area). A
    // hull of one point, of collinear or of coplanar points does not; it still
    // collides, as that point, segment or flat polygon.
    bool solid() const
    {
        return mSolid;
    }

private:
    void buildFeatures();

    ct::Vector<Math::vec3> mVertices;
    ct::Vector<Edge> mEdges;
    ct::Vector<int> mFaces;

    ct::Vector<u32> mFaceStart;
    ct::Vector<u32> mFaceVertices;
    ct::Vector<Math::vec3> mFaceNormals;
    ct::Vector<u32> mEdgeVertices;
    ct::Vector<Math::vec3> mEdgeDirections;
    ct::Vector<u32> mEdgeFaces;
    bool mSolid = false;
    bool mEdgeArcsKnown = false;
};

// A cylinder along the body's local Y. Volume, inertia, bounds, support
// point, ray and sphere queries are the cylinder's own, exact, and so is its
// contact against planes, spheres, boxes, hulls and triangles. Against a
// capsule or another cylinder the narrowphase swaps in hull(), a prism of
// `segments` sides, and pulls the contacts back onto the true surface.
class CylinderShape final : public CollisionShape
{
public:
    // `halfHeight` is half the length along Y, caps included.
    CylinderShape(f32 radius, f32 halfHeight, u32 segments = 16);
    ~CylinderShape() override;
    CylinderShape(const CylinderShape&) = delete;
    CylinderShape& operator=(const CylinderShape&) = delete;

    ShapeType type() const override
    {
        return ShapeType::Cylinder;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    f32 radius() const
    {
        return mRadius;
    }
    f32 halfHeight() const
    {
        return mHalfHeight;
    }
    const ConvexHullShape& hull() const
    {
        return *mHull;
    }

private:
    f32 mRadius;
    f32 mHalfHeight;
    ConvexHullShape* mHull;
};

Math::vec3 closestPointOnSegment(const Math::vec3& a, const Math::vec3& b, const Math::vec3& point);

// Closest point on triangle [a,b,c] to `point`, by Voronoi regions rather
// than by projecting and clamping.
// The region it lands in is also which feature it lands on, so `feature` costs
// nothing to report and is what tells an edge contact from a face one.
Math::vec3 closestPointOnTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                                 const Math::vec3& point, TriangleFeature* feature = nullptr);

// Closest pair of points between two segments. Handles the parallel and
// degenerate cases, which is the whole difficulty - two capsules lying side
// by side are exactly the case a naive solve divides by zero on.
void closestPointsBetweenSegments(const Math::vec3& p1, const Math::vec3& q1, const Math::vec3& p2,
                                  const Math::vec3& q2, Math::vec3& c1, Math::vec3& c2);

// Closest pair of points between the segment [p,q] and the triangle [a,b,c],
// returning their squared distance; `feature` is where the triangle's point
// landed. A segment passing THROUGH the triangle is at distance zero, with
// both points where it passes and the feature Face - the case a test of the
// two ends and the three edges alone misses, and the one a capsule standing
// in a floor is in.
f32 closestPointsSegmentTriangle(const Math::vec3& p, const Math::vec3& q, const Math::vec3& a,
                                 const Math::vec3& b, const Math::vec3& c, Math::vec3& onSegment,
                                 Math::vec3& onTriangle, TriangleFeature* feature = nullptr);

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_COLLISION_SHAPE_H
