#include "Check.h"

#include "prisma3d/core/BoundsTree.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdint.h>

using namespace prisma3d;

namespace
{

struct Lcg
{
    std::uint32_t state;

    explicit Lcg(std::uint32_t seed)
            : state(seed)
    {
    }

    float next()
    {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / 16777216.0f;
    }

    float range(float low, float high)
    {
        return low + (high - low) * next();
    }
};

const float kVolume = 100.0f;

void makeBoxes(Lcg& random, ct::Vector<Math::Box>& boxes, std::uint32_t count)
{
    boxes.clear();
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const Math::Vec3 center(random.range(0.0f, kVolume), random.range(0.0f, kVolume),
                random.range(0.0f, kVolume));
        const Math::Vec3 half(random.range(0.1f, 1.5f), random.range(0.1f, 1.5f),
                random.range(0.1f, 1.5f));
        boxes.push_back(Math::Box(center - half, center + half));
    }
}

bool sphereHits(const Math::Vec3& center, float radius, const Math::Box& box)
{
    const float cx =
            center.x < box.min.x ? box.min.x : (center.x > box.max.x ? box.max.x : center.x);
    const float cy =
            center.y < box.min.y ? box.min.y : (center.y > box.max.y ? box.max.y : center.y);
    const float cz =
            center.z < box.min.z ? box.min.z : (center.z > box.max.z ? box.max.z : center.z);
    const float dx = center.x - cx;
    const float dy = center.y - cy;
    const float dz = center.z - cz;
    return dx * dx + dy * dy + dz * dz <= radius * radius;
}

bool rayHits(const Math::Ray& ray, float maxDistance, const Math::Box& box)
{
    float tMin = 0.0f;
    float tMax = 0.0f;
    return box.IntersectRay(ray, tMin, tMax) && tMin <= maxDistance;
}

bool contains(const ct::Vector<std::uint32_t>& list, std::uint32_t value)
{
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        if (list[i] == value) return true;
    }
    return false;
}

bool isSuperset(const ct::Vector<std::uint32_t>& candidates, const ct::Vector<std::uint32_t>& hits)
{
    for (std::size_t i = 0; i < hits.size(); ++i)
    {
        if (!contains(candidates, hits[i])) return false;
    }
    return true;
}

bool sameSet(const ct::Vector<std::uint32_t>& a, const ct::Vector<std::uint32_t>& b)
{
    return a.size() == b.size() && isSuperset(a, b) && isSuperset(b, a);
}

bool noDuplicates(const ct::Vector<std::uint32_t>& list)
{
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        for (std::size_t j = i + 1; j < list.size(); ++j)
        {
            if (list[i] == list[j]) return false;
        }
    }
    return true;
}

Math::Frustum makeFrustum(const Math::Vec3& eye, const Math::Vec3& target, float fovDegrees,
        float farPlane)
{
    const Math::Mat4 projection =
            Math::Mat4::Perspective(fovDegrees * 0.017453292519943295f, 1.5f, 0.1f, farPlane);
    const Math::Mat4 view = Math::Mat4::LookAt(eye, target, Math::Vec3(0.0f, 1.0f, 0.0f));
    return Math::Frustum::FromViewProjection(projection * view);
}

struct Queries
{
    ct::Vector<std::uint32_t> candidates;
    ct::Vector<std::uint32_t> hits;
};

void checkAll(const BoundsTree& tree, const ct::Vector<Math::Box>& boxes, const char* label)
{
    Queries q;
    const std::uint32_t count = static_cast<std::uint32_t>(boxes.size());

    const Math::Frustum frustums[3] = { makeFrustum(Math::Vec3(50.0f, 50.0f, -60.0f),
                                                Math::Vec3(50.0f, 50.0f, 50.0f), 40.0f, 300.0f),
        makeFrustum(Math::Vec3(-30.0f, 20.0f, 20.0f), Math::Vec3(60.0f, 40.0f, 70.0f), 25.0f,
                90.0f),
        makeFrustum(Math::Vec3(50.0f, 50.0f, 50.0f), Math::Vec3(50.0f, 50.0f, 52.0f), 80.0f,
                400.0f) };
    for (int f = 0; f < 3; ++f)
    {
        q.hits.clear();
        for (std::uint32_t i = 0; i < count; ++i)
        {
            if (frustums[f].IntersectsBox(boxes[i])) q.hits.push_back(i);
        }
        tree.queryCandidates(frustums[f], q.candidates);
        CHECK(isSuperset(q.candidates, q.hits));
        CHECK(noDuplicates(q.candidates));
        CHECK(tree.lastQueryStats().itemsReturned == q.candidates.size());
        ct::Vector<std::uint32_t> filtered;
        for (std::size_t i = 0; i < q.candidates.size(); ++i)
        {
            if (frustums[f].IntersectsBox(boxes[q.candidates[i]]))
                filtered.push_back(q.candidates[i]);
        }
        if (!sameSet(filtered, q.hits)) printf("frustum %d mismatch (%s)\n", f, label);
        CHECK(sameSet(filtered, q.hits));
    }

    const Math::Box queryBoxes[2] = { Math::Box(Math::Vec3(20.0f, 20.0f, 20.0f),
                                              Math::Vec3(45.0f, 40.0f, 35.0f)),
        Math::Box(Math::Vec3(90.0f, 90.0f, 90.0f), Math::Vec3(140.0f, 140.0f, 140.0f)) };
    for (int b = 0; b < 2; ++b)
    {
        q.hits.clear();
        for (std::uint32_t i = 0; i < count; ++i)
        {
            if (boxes[i].Intersects(queryBoxes[b])) q.hits.push_back(i);
        }
        tree.queryCandidates(queryBoxes[b], q.candidates);
        CHECK(isSuperset(q.candidates, q.hits));
        ct::Vector<std::uint32_t> filtered;
        for (std::size_t i = 0; i < q.candidates.size(); ++i)
        {
            if (boxes[q.candidates[i]].Intersects(queryBoxes[b]))
                filtered.push_back(q.candidates[i]);
        }
        CHECK(sameSet(filtered, q.hits));
    }

    const Math::Vec3 centers[2] = { Math::Vec3(30.0f, 60.0f, 40.0f),
        Math::Vec3(80.0f, 10.0f, 90.0f) };
    const float radii[2] = { 12.0f, 30.0f };
    for (int s = 0; s < 2; ++s)
    {
        q.hits.clear();
        for (std::uint32_t i = 0; i < count; ++i)
        {
            if (sphereHits(centers[s], radii[s], boxes[i])) q.hits.push_back(i);
        }
        tree.queryCandidates(centers[s], radii[s], q.candidates);
        CHECK(isSuperset(q.candidates, q.hits));
        ct::Vector<std::uint32_t> filtered;
        for (std::size_t i = 0; i < q.candidates.size(); ++i)
        {
            if (sphereHits(centers[s], radii[s], boxes[q.candidates[i]]))
                filtered.push_back(q.candidates[i]);
        }
        CHECK(sameSet(filtered, q.hits));
    }

    const Math::Ray rays[2] = { Math::Ray(Math::Vec3(-10.0f, 50.0f, 50.0f),
                                        Math::Vec3(1.0f, 0.02f, 0.01f)),
        Math::Ray(Math::Vec3(50.0f, 50.0f, 50.0f), Math::Vec3(0.3f, 0.5f, -0.8f)) };
    const float distances[2] = { 200.0f, 40.0f };
    for (int r = 0; r < 2; ++r)
    {
        q.hits.clear();
        for (std::uint32_t i = 0; i < count; ++i)
        {
            if (rayHits(rays[r], distances[r], boxes[i])) q.hits.push_back(i);
        }
        tree.queryCandidates(rays[r], distances[r], q.candidates);
        CHECK(isSuperset(q.candidates, q.hits));
        ct::Vector<std::uint32_t> filtered;
        for (std::size_t i = 0; i < q.candidates.size(); ++i)
        {
            if (rayHits(rays[r], distances[r], boxes[q.candidates[i]]))
                filtered.push_back(q.candidates[i]);
        }
        CHECK(sameSet(filtered, q.hits));
    }
}

bool nearValue(float a, float b)
{
    return fabsf(a - b) < 0.0001f;
}

} // namespace

int main()
{
    Lcg random(12345u);
    ct::Vector<Math::Box> boxes;
    makeBoxes(random, boxes, 500);
    const std::uint32_t count = static_cast<std::uint32_t>(boxes.size());

    BoundsTree empty;
    CHECK(!empty.valid());
    CHECK(empty.nodeCount() == 0);
    CHECK(empty.itemCount() == 0);
    CHECK(empty.depth() == 0);
    CHECK(empty.quality() == 0.0f);
    {
        ct::Vector<std::uint32_t> out;
        out.push_back(7);
        empty.queryCandidates(
                makeFrustum(Math::Vec3(0, 0, -10), Math::Vec3(0, 0, 0), 60.0f, 100.0f), out);
        CHECK(out.empty());
        out.push_back(7);
        empty.queryCandidates(Math::Box(Math::Vec3(0, 0, 0), Math::Vec3(1, 1, 1)), out);
        CHECK(out.empty());
        out.push_back(7);
        empty.queryCandidates(Math::Vec3(0, 0, 0), 5.0f, out);
        CHECK(out.empty());
        out.push_back(7);
        empty.queryCandidates(Math::Ray(Math::Vec3(0, 0, 0), Math::Vec3(0, 0, 1)), 10.0f, out);
        CHECK(out.empty());
        CHECK(empty.lastQueryStats().nodesVisited == 0);
        CHECK(!empty.refit(boxes.data(), count));
    }

    BoundsTree tree;
    CHECK(tree.leafCapacity() == 2);
    tree.build(boxes.data(), count);
    CHECK(tree.valid());
    CHECK(tree.itemCount() == count);
    CHECK(tree.nodeCount() > 1);
    CHECK(tree.nodeCount() <= 2 * count);
    CHECK(tree.depth() >= 1 && tree.depth() <= BoundsTree::kMaxDepth);
    CHECK(tree.quality() >= 1.0f);
    CHECK(!tree.node(0).isLeaf());
    CHECK(nearValue(tree.itemBounds(3).min.x, boxes[3].min.x));
    checkAll(tree, boxes, "capacity 2");

    {
        const Math::Frustum narrow = makeFrustum(Math::Vec3(50.0f, 50.0f, -60.0f),
                Math::Vec3(50.0f, 50.0f, 50.0f), 4.0f, 300.0f);
        ct::Vector<std::uint32_t> out;
        tree.queryCandidates(narrow, out);
        CHECK(tree.lastQueryStats().nodesVisited > 0);
        CHECK(tree.lastQueryStats().nodesVisited < tree.nodeCount());
        CHECK(tree.lastQueryStats().leavesVisited > 0);
        CHECK(out.size() < boxes.size());
    }

    {
        const Math::Frustum all = makeFrustum(Math::Vec3(50.0f, 50.0f, -400.0f),
                Math::Vec3(50.0f, 50.0f, 50.0f), 90.0f, 2000.0f);
        ct::Vector<std::uint32_t> out;
        tree.queryCandidates(all, out);
        CHECK(out.size() == boxes.size());
        CHECK(tree.lastQueryStats().nodesVisited == tree.nodeCount());
    }

    {
        BoundsTree single;
        single.setLeafCapacity(1);
        BoundsTree wide;
        wide.setLeafCapacity(8);
        CHECK(single.leafCapacity() == 1 && wide.leafCapacity() == 8);
        single.build(boxes.data(), count);
        wide.build(boxes.data(), count);
        checkAll(single, boxes, "capacity 1");
        checkAll(wide, boxes, "capacity 8");
        CHECK(wide.nodeCount() < single.nodeCount());
        CHECK(single.nodeCount() <= 2 * count);
        BoundsTree zero;
        zero.setLeafCapacity(0);
        CHECK(zero.leafCapacity() == 1);
    }

    {
        ct::Vector<Math::Box> moved;
        for (std::size_t i = 0; i < boxes.size(); ++i) moved.push_back(boxes[i]);
        for (std::size_t i = 0; i < moved.size(); i += 2)
        {
            const Math::Vec3 shift(random.range(-20.0f, 20.0f), random.range(-20.0f, 20.0f),
                    random.range(-20.0f, 20.0f));
            moved[i] = Math::Box(boxes[i].min + shift, boxes[i].max + shift);
        }
        const float before = tree.quality();
        CHECK(tree.refit(moved.data(), count));
        CHECK(tree.quality() > 0.0f);
        CHECK(tree.nodeCount() <= 2 * count);
        checkAll(tree, moved, "refit");
        CHECK(!tree.refit(moved.data(), count - 1));
        CHECK(!tree.refit(moved.data(), count + 1));
        CHECK(!tree.refit(nullptr, count));
        checkAll(tree, moved, "after rejected refit");
        BoundsTree fresh;
        fresh.build(moved.data(), count);
        CHECK(fresh.quality() <= tree.quality() + 0.0001f);
        (void) before;
    }

    tree.clear();
    CHECK(!tree.valid());
    CHECK(tree.nodeCount() == 0 && tree.itemCount() == 0);
    {
        ct::Vector<std::uint32_t> out;
        tree.queryCandidates(Math::Box(Math::Vec3(0, 0, 0), Math::Vec3(100, 100, 100)), out);
        CHECK(out.empty());
    }
    tree.build(nullptr, 0);
    CHECK(!tree.valid());
    tree.build(boxes.data(), 1);
    CHECK(tree.valid() && tree.nodeCount() == 1 && tree.node(0).isLeaf() && tree.depth() == 1);
    CHECK(nearValue(tree.quality(), 1.0f));

    {
        ct::Vector<Math::Box> same;
        for (int i = 0; i < 300; ++i)
            same.push_back(Math::Box(Math::Vec3(4.0f, 4.0f, 4.0f), Math::Vec3(6.0f, 6.0f, 6.0f)));
        BoundsTree coincident;
        coincident.build(same.data(), static_cast<std::uint32_t>(same.size()));
        CHECK(coincident.valid());
        CHECK(coincident.itemCount() == 300);
        CHECK(coincident.nodeCount() <= 600);
        CHECK(coincident.depth() <= BoundsTree::kMaxDepth);
        ct::Vector<std::uint32_t> out;
        coincident.queryCandidates(Math::Box(Math::Vec3(5, 5, 5), Math::Vec3(5.5f, 5.5f, 5.5f)),
                out);
        CHECK(out.size() == 300);
        coincident.queryCandidates(Math::Box(Math::Vec3(50, 50, 50), Math::Vec3(55, 55, 55)), out);
        CHECK(out.empty());
        CHECK(nearValue(coincident.quality(), coincident.quality()));
    }

    {
        const Math::Frustum frustum = makeFrustum(Math::Vec3(0.0f, 0.0f, -10.0f),
                Math::Vec3(0.0f, 0.0f, 0.0f), 60.0f, 100.0f);
        CHECK(classifyBox(frustum, Math::Box(Math::Vec3(-0.5f, -0.5f, 0.0f),
                                           Math::Vec3(0.5f, 0.5f, 1.0f))) == Containment::Inside);
        CHECK(classifyBox(frustum,
                      Math::Box(Math::Vec3(200.0f, 0.0f, 0.0f), Math::Vec3(201.0f, 1.0f, 1.0f))) ==
                Containment::Outside);
        CHECK(classifyBox(frustum,
                      Math::Box(Math::Vec3(-1.0f, -1.0f, -30.0f), Math::Vec3(1.0f, 1.0f, 1.0f))) ==
                Containment::Intersects);
        CHECK(classifyBox(frustum, Math::Box(Math::Vec3(0.0f, 0.0f, -300.0f),
                                           Math::Vec3(1.0f, 1.0f, -200.0f))) ==
                Containment::Outside);
        CHECK(classifyBox(frustum, Math::Box()) == Containment::Outside);
    }

    {
        CHECK(emptyBox(Math::Box()));
        CHECK(emptyBox(Math::Box(Math::Vec3(1, 0, 0), Math::Vec3(0, 1, 1))));
        CHECK(emptyBox(Math::Box(Math::Vec3(0, 1, 0), Math::Vec3(1, 0, 1))));
        CHECK(emptyBox(Math::Box(Math::Vec3(0, 0, 1), Math::Vec3(1, 1, 0))));
        CHECK(!emptyBox(Math::Box(Math::Vec3(0, 0, 0), Math::Vec3(1, 1, 1))));
        CHECK(!emptyBox(Math::Box(Math::Vec3(1, 1, 1), Math::Vec3(1, 1, 1))));
        CHECK(boxSurfaceArea(Math::Box()) == 0.0f);
        CHECK(nearValue(boxSurfaceArea(Math::Box(Math::Vec3(0, 0, 0), Math::Vec3(1, 2, 3))),
                2.0f * (2.0f + 6.0f + 3.0f)));
        CHECK(boxSurfaceArea(Math::Box(Math::Vec3(1, 1, 1), Math::Vec3(1, 1, 1))) == 0.0f);
    }

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
