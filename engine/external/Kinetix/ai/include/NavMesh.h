#ifndef KINETIX_AI_NAVMESH_H
#define KINETIX_AI_NAVMESH_H

// A walkable triangle IS a navmesh polygon; two are neighbors when they share an edge, or when
// their open edges sit within edgeWeldDistance (XZ) and agentMaxClimb (Y), e.g. stair treads.
// Meant for hand-authored floor geometry; there is no voxelize/contour stage.
//
// agentRadius is applied by cutting each portal (edge between two walkable triangles) at build time
// to the stretch at least agentRadius from every wall (computePortalSpans()); A* only crosses
// portals with such a stretch left and the funnel threads the cut-down portals. The goal is pushed
// off a wall closer than agentRadius. Within one triangle a straight segment is not re-checked:
// next to a reflex corner it can cut the radius by a chord's depth.

#include <kinetix/core/Types.h>

#include <kinetix/core/Math.h>
#include <ct/vector.hpp>

namespace kinetix::AI
{

struct NavMeshConfig
{
    f32 agentMaxClimb = 0.90f; // step/stair height the agent can walk up
    f32 agentMaxSlope = 45.0f; // degrees
    f32 edgeWeldDistance = 0.10f; // XZ gap two open edges can still bridge
    // Keeps routes this far from every outer/unwalkable edge; gaps narrower than twice this are not
    // routed through. The surface itself is not shrunk. 0 disables.
    f32 agentRadius = 0.0f;
};

struct NavTri
{
    u32 v[3] = {0, 0, 0};
    Math::vec3 normal = Math::vec3(0.0f, 1.0f, 0.0f);
    s32 neighbor[3] = {-1, -1, -1};
    bool active = true;
};

// Queries are const but share scratch buffers (the path search and the spatial lookup), so one
// NavMesh answers one query at a time; give each thread its own copy.
class NavMesh
{
public:
    NavMesh() = default;

    // World-space triangles. Replaces whatever was built before; false leaves the previous mesh
    // released and valid() false.
    //
    // groundSeed, if given, prunes the surface to what a walking agent can reach from that point
    // (triangle-adjacency BFS), which excludes e.g. a flat roof with no stairs down. Omit it (or
    // pass a point off the mesh) to keep every walkable-slope surface.
    bool build(const f32* vertices, s32 vertexCount, const s32* indices, s32 triangleCount,
               const NavMeshConfig& config, const Math::vec3* groundSeed = nullptr);
    void release();
    bool valid() const;

    // Corner points from start to end, including both; outPath is replaced (empty on false). False
    // when either end is off the mesh (beyond `searchExtents`). A partial route (goal unreachable)
    // still returns true, ending at the closest reachable point; `outPartial`, if given, tells the
    // two apart.
    bool findPath(const Math::vec3& start, const Math::vec3& end, ct::Vector<Math::vec3>& outPath,
                  const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f),
                  bool* outPartial = nullptr) const;

    // Closest point on the walkable surface. False when nothing is within `searchExtents`.
    bool nearestPoint(const Math::vec3& point, Math::vec3& out,
                      const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // Slides `from` towards `to` across the surface and returns where it ends: `to` only if the
    // whole segment stays on the mesh, otherwise the boundary hit. `out.y` is placed on the
    // surface. `from` must already be on the mesh (the agent's last constrained position); false
    // otherwise, leaving `out` untouched.
    bool moveAlongSurface(const Math::vec3& from, const Math::vec3& to, Math::vec3& out,
                          const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // Same walk with a triangle hint: pass the triangle the previous call ended on in `tri` (or -1
    // the first time); it is updated. Without a hint the starting triangle is searched over the
    // whole mesh, so the cost grows with the level.
    bool moveAlongSurface(const Math::vec3& from, const Math::vec3& to, Math::vec3& out, s32& tri,
                          const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // Walks from `from` towards `to` and stops at the first boundary crossed (a navmesh
    // line-of-sight test); unlike moveAlongSurface() it reports the hit instead of sliding. False
    // when `from` is off the mesh. `outHitFraction` is 1 when the whole segment is walkable,
    // otherwise where along it the wall is. `outHitNormal` is the wall's XZ normal pointing back
    // into the walkable area (towards `from`), or zero when the stopping edge is unknown. A walk
    // that exhausts its step budget counts as a hit.
    bool raycast(const Math::vec3& from, const Math::vec3& to, f32& outHitFraction,
                 Math::vec3& outHitNormal,
                 const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // Distance from `point` to the nearest unwalkable edge, capped at `maxRadius`. `outWallNormal`
    // points from the wall into the walkable area (zero when no wall is within `maxRadius`).
    bool distanceToWall(const Math::vec3& point, f32 maxRadius, f32& outDistance,
                        Math::vec3& outWallNormal,
                        const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // Random point picked uniformly by area. `seed` is advanced, so repeated calls with the same
    // variable give a deterministic sequence.
    bool randomPoint(u32& seed, Math::vec3& out) const;

    // Drops corners that can be reached directly (findPath() already funnels, but a stitched or
    // partial route still zigzags).
    void smoothPath(ct::Vector<Math::vec3>& path,
                    const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // World space; rebuilt only by build(), so a caller may keep the reference.
    const ct::Vector<Math::vec3>& debugTriangles() const;

private:
    // Runs the same search across several frames, on the same internals.
    friend class NavMeshQuery;

    // Cut-down stretch of one triangle edge an agent's centre may cross, as
    // parameters along v[e] -> v[e + 1]. lo > hi means closed.
    struct PortalSpan
    {
        f32 lo = 0.0f;
        f32 hi = 1.0f;
    };

    void buildAdjacency(float weldDistance, float maxClimb);
    void pruneUnreachable(const Math::vec3& seed);
    void computePortalSpans(float agentRadius, float maxClimb);
    bool portalOpen(s32 tri, s32 edge) const;
    // Pushes a route end that sits closer than agentRadius to a wall back
    // out, staying on the surface; updates `tri` to where it lands.
    void keepClearOfWalls(Math::vec3& point, s32& tri) const;
    void rebuildDebugTriangles();
    s32 findContainingTri(const Math::vec3& point, const Math::vec3& searchExtents,
                          Math::vec3* outClosest) const;
    bool getPortalPoints(s32 triA, s32 triB, Math::vec3& left, Math::vec3& right) const;
    Math::vec3 closestPointOnTri(const Math::vec3& point, const NavTri& tri) const;
    Math::vec3 triCentroid(s32 tri) const;
    void collectVertexRing(s32 tri, s32* out, usize capacity, usize& count) const;
    void funnelPath(const ct::Vector<s32>& triPath, const Math::vec3& startPos,
                    const Math::vec3& endPos, ct::Vector<Math::vec3>& outPath) const;

    ct::Vector<Math::vec3> mVerts;
    ct::Vector<NavTri> mTris;
    ct::Vector<PortalSpan> mPortals; // 3 per triangle, empty with agentRadius 0
    f32 mAgentRadius = 0.0f;
    ct::Vector<Math::vec3> mDebugTriangles;

    // Triangles by XZ cell, so finding the triangle under a point looks at its neighbourhood
    // instead of the whole mesh.
    void buildSpatialGrid();
    ct::Vector<Math::vec3> mCentroids;
    ct::Vector<f32> mAreaPrefix; // walkable area up to and including each triangle, for randomPoint()
    ct::Vector<u32> mGridStart; // mGridCols * mGridRows + 1 offsets into mGridTris
    ct::Vector<s32> mGridTris;
    Math::vec3 mGridMin = Math::vec3(0.0f);
    f32 mGridCell = 2.0f;
    s32 mGridCols = 0;
    s32 mGridRows = 0;

    // A* state kept between searches; a slot is current only when its stamp is the generation of
    // this search, so nothing is cleared per query.
    struct SearchEntry
    {
        s32 tri;
        f32 f;
    };
    struct SearchScratch
    {
        ct::Vector<f32> g;
        ct::Vector<s32> parent;
        ct::Vector<u32> seen;
        ct::Vector<u32> closed;
        ct::Vector<SearchEntry> open;
        u32 generation = 0;
    };
    mutable SearchScratch mSearch;
};

}

#endif
