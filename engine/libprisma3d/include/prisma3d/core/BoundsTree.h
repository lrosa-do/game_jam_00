#pragma once

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

enum class Containment : std::uint8_t
{
    Outside,
    Intersects,
    Inside
};

Containment classifyBox(const Math::Frustum& frustum, const Math::Box& box);
bool emptyBox(const Math::Box& box);
float boxSurfaceArea(const Math::Box& box);

class BoundsTree
{
public:
    static constexpr std::uint32_t kMaxDepth = 48;

    struct Node
    {
        Math::Box bounds;
        std::uint32_t left = 0;
        std::uint32_t offset = 0;
        std::uint32_t count = 0;

        bool isLeaf() const
        {
            return count > 0;
        }
    };

    struct Stats
    {
        std::uint32_t nodesVisited = 0;
        std::uint32_t leavesVisited = 0;
        std::uint32_t itemsReturned = 0;
    };

    void build(const Math::Box* bounds, std::uint32_t count);
    bool refit(const Math::Box* bounds, std::uint32_t count);
    void clear();

    bool valid() const;
    std::uint32_t nodeCount() const;
    std::uint32_t itemCount() const;
    std::uint32_t depth() const;
    const Node& node(std::uint32_t index) const;
    float quality() const;
    const Math::Box& itemBounds(std::uint32_t item) const;

    void queryCandidates(const Math::Box& box, ct::Vector<std::uint32_t>& out) const;
    void queryCandidates(const Math::Vec3& center, float radius,
            ct::Vector<std::uint32_t>& out) const;
    void queryCandidates(const Math::Frustum& frustum, ct::Vector<std::uint32_t>& out) const;
    void queryCandidates(const Math::Ray& ray, float maxDistance,
            ct::Vector<std::uint32_t>& out) const;

    const Stats& lastQueryStats() const;

    void setLeafCapacity(std::uint32_t capacity);
    std::uint32_t leafCapacity() const;

private:
    void subdivide(std::uint32_t nodeIndex, const Math::Box* bounds, std::uint32_t depth);
    void updateNodeBounds(std::uint32_t nodeIndex, const Math::Box* bounds);

    ct::Vector<Node> mNodes;
    ct::Vector<Math::Box> mBounds;
    ct::Vector<std::uint32_t> mOrder;
    std::uint32_t mNodeCount = 0;
    std::uint32_t mDepth = 0;
    std::uint32_t mLeafCapacity = 2;
    mutable Stats mStats;
    mutable ct::Vector<std::uint32_t> mStack;
};

} // namespace prisma3d
