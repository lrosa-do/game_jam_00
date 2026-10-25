#include "prisma3d/core/BoundsTree.h"

namespace prisma3d
{

namespace
{

float absolute(float value)
{
    return value < 0.0f ? -value : value;
}

float clampValue(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

bool sphereTouchesBox(const Math::Vec3& center, float radius, const Math::Box& box)
{
    if (emptyBox(box)) return false;
    const float dx = center.x - clampValue(center.x, box.min.x, box.max.x);
    const float dy = center.y - clampValue(center.y, box.min.y, box.max.y);
    const float dz = center.z - clampValue(center.z, box.min.z, box.max.z);
    return dx * dx + dy * dy + dz * dz <= radius * radius;
}

} // namespace

bool emptyBox(const Math::Box& box)
{
    return box.min.x > box.max.x || box.min.y > box.max.y || box.min.z > box.max.z;
}

float boxSurfaceArea(const Math::Box& box)
{
    if (emptyBox(box)) return 0.0f;
    const float x = box.max.x - box.min.x;
    const float y = box.max.y - box.min.y;
    const float z = box.max.z - box.min.z;
    return 2.0f * (x * y + y * z + z * x);
}

Containment classifyBox(const Math::Frustum& frustum, const Math::Box& box)
{
    if (emptyBox(box)) return Containment::Outside;

    const Math::Vec3 center = box.Center();
    const Math::Vec3 extents = box.Extents();
    bool intersecting = false;
    for (int i = 0; i < 6; ++i)
    {
        const Math::Plane& plane = frustum.planes[i];
        const float reach = extents.x * absolute(plane.normal.x) +
                            extents.y * absolute(plane.normal.y) +
                            extents.z * absolute(plane.normal.z);
        const float distance = plane.DistanceToPoint(center);
        if (distance < -reach) return Containment::Outside;
        if (distance < reach) intersecting = true;
    }
    return intersecting ? Containment::Intersects : Containment::Inside;
}

void BoundsTree::clear()
{
    mNodes.clear();
    mBounds.clear();
    mOrder.clear();
    mNodeCount = 0;
    mDepth = 0;
    mStats = Stats();
}

void BoundsTree::setLeafCapacity(std::uint32_t capacity)
{
    mLeafCapacity = capacity > 1u ? capacity : 1u;
}

std::uint32_t BoundsTree::leafCapacity() const
{
    return mLeafCapacity;
}

bool BoundsTree::valid() const
{
    return mNodeCount > 0;
}

std::uint32_t BoundsTree::nodeCount() const
{
    return mNodeCount;
}

std::uint32_t BoundsTree::itemCount() const
{
    return static_cast<std::uint32_t>(mOrder.size());
}

std::uint32_t BoundsTree::depth() const
{
    return mDepth;
}

const BoundsTree::Node& BoundsTree::node(std::uint32_t index) const
{
    return mNodes[index];
}

const Math::Box& BoundsTree::itemBounds(std::uint32_t item) const
{
    return mBounds[item];
}

const BoundsTree::Stats& BoundsTree::lastQueryStats() const
{
    return mStats;
}

void BoundsTree::build(const Math::Box* bounds, std::uint32_t count)
{
    clear();
    if (!bounds || count == 0) return;

    mNodes.resize(static_cast<std::size_t>(count) * 2u);
    mBounds.resize(count);
    mOrder.resize(count);
    for (std::uint32_t i = 0; i < count; ++i)
    {
        mBounds[i] = bounds[i];
        mOrder[i] = i;
    }

    mNodeCount = 1;
    Node& root = mNodes[0];
    root = Node();
    root.offset = 0;
    root.count = count;
    updateNodeBounds(0, bounds);

    mDepth = 1;
    subdivide(0, bounds, 1);
}

void BoundsTree::updateNodeBounds(std::uint32_t nodeIndex, const Math::Box* bounds)
{
    Node& node = mNodes[nodeIndex];
    node.bounds = Math::Box();
    for (std::uint32_t i = 0; i < node.count; ++i)
        node.bounds = node.bounds.Union(bounds[mOrder[node.offset + i]]);
}

void BoundsTree::subdivide(std::uint32_t nodeIndex, const Math::Box* bounds, std::uint32_t depth)
{
    if (depth > mDepth) mDepth = depth;
    if (depth >= kMaxDepth) return;

    const std::uint32_t count = mNodes[nodeIndex].count;
    const std::uint32_t offset = mNodes[nodeIndex].offset;
    if (count <= mLeafCapacity) return;

    const Math::Vec3 extent = mNodes[nodeIndex].bounds.max - mNodes[nodeIndex].bounds.min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    const float split = mNodes[nodeIndex].bounds.min[axis] + extent[axis] * 0.5f;

    std::uint32_t left = offset;
    std::uint32_t right = offset + count;
    while (left < right)
    {
        const Math::Box& box = bounds[mOrder[left]];
        const float center = (box.min[axis] + box.max[axis]) * 0.5f;
        if (center < split) ++left;
        else
        {
            --right;
            const std::uint32_t swapped = mOrder[left];
            mOrder[left] = mOrder[right];
            mOrder[right] = swapped;
        }
    }

    std::uint32_t leftCount = left - offset;
    if (leftCount == 0 || leftCount == count) leftCount = count / 2;

    const std::uint32_t leftChild = mNodeCount++;
    const std::uint32_t rightChild = mNodeCount++;
    mNodes[nodeIndex].left = leftChild;
    mNodes[nodeIndex].count = 0;

    mNodes[leftChild] = Node();
    mNodes[leftChild].offset = offset;
    mNodes[leftChild].count = leftCount;
    mNodes[rightChild] = Node();
    mNodes[rightChild].offset = offset + leftCount;
    mNodes[rightChild].count = count - leftCount;

    updateNodeBounds(leftChild, bounds);
    updateNodeBounds(rightChild, bounds);
    subdivide(leftChild, bounds, depth + 1);
    subdivide(rightChild, bounds, depth + 1);
}

bool BoundsTree::refit(const Math::Box* bounds, std::uint32_t count)
{
    if (mNodeCount == 0 || !bounds) return false;
    if (count != static_cast<std::uint32_t>(mOrder.size())) return false;
    for (std::uint32_t i = 0; i < count; ++i) mBounds[i] = bounds[i];

    for (std::uint32_t i = mNodeCount; i > 0; --i)
    {
        Node& node = mNodes[i - 1];
        node.bounds = Math::Box();
        if (node.isLeaf())
        {
            for (std::uint32_t j = 0; j < node.count; ++j)
                node.bounds = node.bounds.Union(bounds[mOrder[node.offset + j]]);
        }
        else
        {
            node.bounds = node.bounds.Union(mNodes[node.left].bounds);
            node.bounds = node.bounds.Union(mNodes[node.left + 1].bounds);
        }
    }
    return true;
}

float BoundsTree::quality() const
{
    if (mNodeCount == 0) return 0.0f;
    const float rootArea = boxSurfaceArea(mNodes[0].bounds);
    if (rootArea <= 0.0f) return 0.0f;
    float total = 0.0f;
    for (std::uint32_t i = 0; i < mNodeCount; ++i) total += boxSurfaceArea(mNodes[i].bounds);
    return total / rootArea;
}

void BoundsTree::queryCandidates(const Math::Box& box, ct::Vector<std::uint32_t>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0) return;

    mStack.clear();
    mStack.push_back(0);
    while (!mStack.empty())
    {
        const Node& node = mNodes[mStack.back()];
        mStack.pop_back();
        ++mStats.nodesVisited;
        if (!node.bounds.Intersects(box)) continue;
        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (std::uint32_t i = 0; i < node.count; ++i) out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(node.left + 1);
    }
    mStats.itemsReturned = static_cast<std::uint32_t>(out.size());
}

void BoundsTree::queryCandidates(const Math::Vec3& center, float radius,
        ct::Vector<std::uint32_t>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0) return;

    mStack.clear();
    mStack.push_back(0);
    while (!mStack.empty())
    {
        const Node& node = mNodes[mStack.back()];
        mStack.pop_back();
        ++mStats.nodesVisited;
        if (!sphereTouchesBox(center, radius, node.bounds)) continue;
        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (std::uint32_t i = 0; i < node.count; ++i) out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(node.left + 1);
    }
    mStats.itemsReturned = static_cast<std::uint32_t>(out.size());
}

void BoundsTree::queryCandidates(const Math::Frustum& frustum, ct::Vector<std::uint32_t>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0) return;

    mStack.clear();
    mStack.push_back(0);
    mStack.push_back(0);
    while (!mStack.empty())
    {
        const std::uint32_t inside = mStack.back();
        mStack.pop_back();
        const std::uint32_t nodeIndex = mStack.back();
        mStack.pop_back();
        const Node& node = mNodes[nodeIndex];
        ++mStats.nodesVisited;

        std::uint32_t childInside = inside;
        if (!inside)
        {
            const Containment result = classifyBox(frustum, node.bounds);
            if (result == Containment::Outside) continue;
            childInside = result == Containment::Inside ? 1u : 0u;
        }

        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (std::uint32_t i = 0; i < node.count; ++i) out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(childInside);
        mStack.push_back(node.left + 1);
        mStack.push_back(childInside);
    }
    mStats.itemsReturned = static_cast<std::uint32_t>(out.size());
}

void BoundsTree::queryCandidates(const Math::Ray& ray, float maxDistance,
        ct::Vector<std::uint32_t>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0) return;

    mStack.clear();
    mStack.push_back(0);
    while (!mStack.empty())
    {
        const Node& node = mNodes[mStack.back()];
        mStack.pop_back();
        ++mStats.nodesVisited;
        float tMin = 0.0f;
        float tMax = 0.0f;
        if (emptyBox(node.bounds) || !node.bounds.IntersectRay(ray, tMin, tMax)) continue;
        if (tMin > maxDistance) continue;
        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (std::uint32_t i = 0; i < node.count; ++i) out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(node.left + 1);
    }
    mStats.itemsReturned = static_cast<std::uint32_t>(out.size());
}

} // namespace prisma3d
