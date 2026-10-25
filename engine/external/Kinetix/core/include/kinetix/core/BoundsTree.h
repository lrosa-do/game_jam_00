#pragma once

#include "Math.h"
#include "Types.h"

#include <ct/vector.hpp>

namespace kinetix
{

class BoundsTree
{
public:
    struct Node
    {
        AABB bounds;
        u32 left = 0;
        u32 offset = 0;
        u32 count = 0;

        bool isLeaf() const { return count > 0; }
    };

    void build(const AABB* bounds, u32 count);
    bool refit(const AABB* bounds, u32 count);
    void clear();

    bool valid() const { return mNodeCount > 0; }
    u32 nodeCount() const { return mNodeCount; }
    u32 itemCount() const { return static_cast<u32>(mOrder.size()); }
    u32 depth() const { return mDepth; }
    const Node& node(u32 index) const { return mNodes[index]; }

    f32 quality() const;

    const AABB& itemBounds(u32 item) const { return mBounds[item]; }

    void queryCandidates(const AABB& box, ct::Vector<u32>& out) const;
    void queryCandidates(const Sphere& sphere, ct::Vector<u32>& out) const;
    void queryCandidates(const Frustum& frustum, ct::Vector<u32>& out) const;
    void queryCandidates(const Ray& ray, f32 maxDistance, ct::Vector<u32>& out) const;

    template <typename NodeTest>
    void queryCandidatesIf(const NodeTest& test, ct::Vector<u32>& out) const
    {
        out.clear();
        mStats = Stats();
        if (mNodeCount == 0)
            return;

        mStack.clear();
        mStack.push_back(0);
        mStack.push_back(0);
        while (!mStack.empty())
        {
            const u32 inside = mStack.back();
            mStack.pop_back();
            const u32 nodeIndex = mStack.back();
            mStack.pop_back();
            const Node& node = mNodes[nodeIndex];
            ++mStats.nodesVisited;

            u32 childInside = inside;
            if (!inside)
            {
                const Containment result = test(node.bounds);
                if (result == Containment::Outside)
                    continue;
                childInside = result == Containment::Inside ? 1u : 0u;
            }

            if (node.isLeaf())
            {
                ++mStats.leavesVisited;
                for (u32 i = 0; i < node.count; ++i)
                    out.push_back(mOrder[node.offset + i]);
                continue;
            }
            mStack.push_back(node.left);
            mStack.push_back(childInside);
            mStack.push_back(node.left + 1);
            mStack.push_back(childInside);
        }
        mStats.itemsReturned = static_cast<u32>(out.size());
    }

    struct Stats
    {
        u32 nodesVisited = 0;
        u32 leavesVisited = 0;
        u32 itemsReturned = 0;
    };
    const Stats& lastQueryStats() const { return mStats; }

    void setLeafCapacity(u32 capacity);
    u32 leafCapacity() const { return mLeafCapacity; }

private:
    void subdivide(u32 nodeIndex, const AABB* bounds, u32 depth);
    void updateNodeBounds(u32 nodeIndex, const AABB* bounds);

    ct::Vector<Node> mNodes;
    ct::Vector<AABB> mBounds;
    ct::Vector<u32> mOrder;
    u32 mNodeCount = 0;
    u32 mDepth = 0;
    u32 mLeafCapacity = 2;
    mutable Stats mStats;
    mutable ct::Vector<u32> mStack;
};

} // namespace kinetix
