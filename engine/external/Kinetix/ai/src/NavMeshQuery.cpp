#include <NavMeshQuery.h>

#include <NavMesh.h>

#include <kinetix/core/Algorithm.h>

#include <float.h>

namespace kinetix::AI
{

namespace
{
constexpr f32 kFloatMax = FLT_MAX;
}

bool NavMeshQuery::start(const NavMesh& mesh, const Math::vec3& start, const Math::vec3& end,
                         const Math::vec3& searchExtents)
{
    reset();

    mMesh = &mesh;
    mEnd = end;

    if (mesh.mTris.empty())
    {
        mState = PathQueryState::Failed;
        return false;
    }

    mStartTri = mesh.findContainingTri(start, searchExtents, &mStartClosest);
    mEndTri = mesh.findContainingTri(end, searchExtents, &mEndClosest);
    if (mStartTri < 0 || mEndTri < 0)
    {
        mState = PathQueryState::Failed;
        return false;
    }
    mesh.keepClearOfWalls(mEndClosest, mEndTri);

    // Both ends on one triangle: there is nothing to search.
    if (mStartTri == mEndTri)
    {
        mReachedTri = mStartTri;
        mState = PathQueryState::Ready;
        return true;
    }

    const usize n = mesh.mTris.size();
    mGScore.clear();
    mGScore.resize(n, kFloatMax);
    mParent.clear();
    mParent.resize(n, -1);
    mClosed.clear();
    mClosed.resize(n, 0);

    // Centroids once for the whole mesh, as findPath() does.
    mCentroid.clear();
    mCentroid.reserve(n);
    for (usize i = 0; i < n; ++i)
    {
        const NavTri& tri = mesh.mTris[i];
        mCentroid.push_back(
            (mesh.mVerts[tri.v[0]] + mesh.mVerts[tri.v[1]] + mesh.mVerts[tri.v[2]]) * (1.0f / 3.0f));
    }
    mGoalCentroid = mCentroid[static_cast<usize>(mEndTri)];

    mOpen.clear();
    mGScore[static_cast<usize>(mStartTri)] = 0.0f;
    heapPush({mStartTri, Math::distance(mCentroid[static_cast<usize>(mStartTri)], mGoalCentroid)});

    mReachedTri = mStartTri;
    mBestHeuristic = kFloatMax;
    mNodesExpanded = 0;
    mState = PathQueryState::Searching;
    return true;
}

PathQueryState NavMeshQuery::step(u32 nodeBudget)
{
    if (mState != PathQueryState::Searching)
        return mState;

    const NavMesh& mesh = *mMesh;


    for (u32 expanded = 0; expanded < nodeBudget; ++expanded)
    {
        if (mOpen.empty())
        {
            // Nothing left to reach. Whatever got closest is the answer, the
            // same partial route findPath() returns.
            finish();
            return mState;
        }

        const OpenEntry cur = heapPop();
        const usize curIdx = static_cast<usize>(cur.tri);

        // Stale entry: this triangle was already expanded through a cheaper
        // route, so the heap still holds its earlier, worse copy.
        if (mClosed[curIdx])
            continue;

        // The budget is spent and a node is still left to expand: give up rather than return a
        // partial route. Checked here, not after an expansion, so a search needing exactly maxNodes
        // completes.
        if (mNodesExpanded >= mMaxNodes)
        {
            mState = PathQueryState::Failed;
            return mState;
        }

        mClosed[curIdx] = 1;
        ++mNodesExpanded;

        const Math::vec3& curCentroid = mCentroid[curIdx];
        const f32 h = Math::distance(curCentroid, mGoalCentroid);
        if (h < mBestHeuristic)
        {
            mBestHeuristic = h;
            mReachedTri = cur.tri;
        }

        if (cur.tri == mEndTri)
        {
            finish();
            return mState;
        }

        const NavTri& triData = mesh.mTris[curIdx];
        for (s32 e = 0; e < 3; ++e)
        {
            const s32 nb = triData.neighbor[e];
            if (nb < 0)
                continue;
            const usize nbIdx = static_cast<usize>(nb);
            if (!mesh.mTris[nbIdx].active || mClosed[nbIdx] || !mesh.portalOpen(cur.tri, e))
                continue;

            const f32 tentative = mGScore[curIdx] + Math::distance(curCentroid, mCentroid[nbIdx]);
            if (tentative < mGScore[nbIdx])
            {
                mGScore[nbIdx] = tentative;
                mParent[nbIdx] = cur.tri;
                heapPush({nb, tentative + Math::distance(mCentroid[nbIdx], mGoalCentroid)});
            }
        }
    }

    return mState;
}

bool NavMeshQuery::partial() const
{
    return mState == PathQueryState::Ready && mReachedTri != mEndTri;
}

void NavMeshQuery::finish()
{
    mState = PathQueryState::Ready;
}

bool NavMeshQuery::result(ct::Vector<Math::vec3>& outPath) const
{
    outPath.clear();
    if (mState != PathQueryState::Ready || !mMesh)
        return false;

    const NavMesh& mesh = *mMesh;

    if (mStartTri == mEndTri)
    {
        outPath.push_back(mStartClosest);
        outPath.push_back(mEndClosest);
        return true;
    }

    ct::Vector<s32> triPath;
    s32 cur = mReachedTri;
    while (cur != -1)
    {
        triPath.push_back(cur);
        cur = mParent[static_cast<usize>(cur)];
    }
    for (usize i = 0, j = triPath.size() - 1; i < j; ++i, --j)
        kinetix::Swap(triPath[i], triPath[j]);

    Math::vec3 finalPoint = mEndClosest;
    if (mReachedTri != mEndTri)
    {
        const NavTri& tri = mesh.mTris[static_cast<usize>(mReachedTri)];
        finalPoint = mesh.closestPointOnTri(mEnd, tri);
    }

    mesh.funnelPath(triPath, mStartClosest, finalPoint, outPath);
    return true;
}

void NavMeshQuery::reset()
{
    mMesh = nullptr;
    mState = PathQueryState::Idle;
    mOpen.clear();
    mStartTri = -1;
    mEndTri = -1;
    mReachedTri = -1;
    mNodesExpanded = 0;
    mBestHeuristic = 0.0f;
}

void NavMeshQuery::heapPush(OpenEntry entry)
{
    mOpen.push_back(entry);
    usize child = mOpen.size() - 1;
    while (child > 0)
    {
        const usize parentIdx = (child - 1) / 2;
        if (mOpen[parentIdx].f <= mOpen[child].f)
            break;
        const OpenEntry tmp = mOpen[parentIdx];
        mOpen[parentIdx] = mOpen[child];
        mOpen[child] = tmp;
        child = parentIdx;
    }
}

NavMeshQuery::OpenEntry NavMeshQuery::heapPop()
{
    const OpenEntry top = mOpen[0];
    mOpen[0] = mOpen.back();
    mOpen.pop_back();
    const usize size = mOpen.size();
    usize node = 0;
    for (;;)
    {
        const usize left = node * 2 + 1;
        if (left >= size)
            break;
        const usize right = left + 1;
        const usize smallest = (right < size && mOpen[right].f < mOpen[left].f) ? right : left;
        if (mOpen[node].f <= mOpen[smallest].f)
            break;
        const OpenEntry tmp = mOpen[node];
        mOpen[node] = mOpen[smallest];
        mOpen[smallest] = tmp;
        node = smallest;
    }
    return top;
}

}
