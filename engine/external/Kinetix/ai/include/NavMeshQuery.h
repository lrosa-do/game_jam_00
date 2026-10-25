#ifndef KINETIX_AI_NAVMESHQUERY_H
#define KINETIX_AI_NAVMESHQUERY_H

// The A* of NavMesh::findPath() spread over frames: step() advances a fixed number of nodes, so the
// caller bounds the per-frame cost (a crowd repathing at once would otherwise spike) and a search
// too expensive to finish is abandoned by budget. Yields the same path findPath() would.

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::AI
{

class NavMesh;

enum class PathQueryState : u8
{
    Idle,

    // Still searching. Call step() again next frame.
    Searching,

    Ready,

    // No path exists, an endpoint was off the mesh, or the node budget ran
    // out before the search could finish. result() returns false.
    Failed
};

class NavMeshQuery
{
public:
    NavMeshQuery() = default;

    // The mesh must outlive the query and not be rebuilt while one runs (build() invalidates the
    // triangle indices). Returns false (query Failed) when either endpoint is off the mesh.
    bool start(const NavMesh& mesh, const Math::vec3& start, const Math::vec3& end,
               const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f));

    // Expands at most `nodeBudget` nodes and returns the state afterwards. A no-op returning the
    // current state unless Searching, so a caller may step every frame.
    PathQueryState step(u32 nodeBudget = 64);

    // Total nodes expanded since start(). Once `maxNodes` are expanded with nodes left, the query
    // goes Failed (findPath() has no such cap).
    u32 nodesExpanded() const { return mNodesExpanded; }
    void setMaxNodes(u32 maxNodes) { mMaxNodes = maxNodes; }
    u32 maxNodes() const { return mMaxNodes; }

    PathQueryState state() const { return mState; }

    // Corner points once the query is Ready; `outPath` is replaced and left empty when this returns
    // false. A search that made progress but could not reach the goal is still Ready and ends at
    // the closest reachable point; partial() tells the two apart.
    bool result(ct::Vector<Math::vec3>& outPath) const;

    bool partial() const;

    // Drops the search state and returns to Idle; the per-triangle arrays are kept for the next
    // start() on a same-size mesh.
    void reset();

private:
    struct OpenEntry
    {
        s32 tri;
        f32 f;
    };

    void heapPush(OpenEntry entry);
    OpenEntry heapPop();
    void finish();

    const NavMesh* mMesh = nullptr;
    PathQueryState mState = PathQueryState::Idle;

    ct::Vector<f32> mGScore;
    ct::Vector<s32> mParent;
    ct::Vector<u8> mClosed;
    ct::Vector<Math::vec3> mCentroid;
    ct::Vector<OpenEntry> mOpen;

    Math::vec3 mStartClosest;
    Math::vec3 mEndClosest;
    Math::vec3 mEnd;
    Math::vec3 mGoalCentroid;

    s32 mStartTri = -1;
    s32 mEndTri = -1;
    s32 mReachedTri = -1;
    f32 mBestHeuristic = 0.0f;

    u32 mNodesExpanded = 0;
    u32 mMaxNodes = 10000;
};

}

#endif
