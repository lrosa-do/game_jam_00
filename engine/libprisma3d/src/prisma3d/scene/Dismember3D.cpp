#include "prisma3d/scene/Dismember3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/hashmap.hpp>

#include <math.h>

namespace prisma3d
{

struct DismemberSegment
{
    Math::Vec3 from;
    Math::Vec3 to;
    int bone = 0;
    int group = 0;
};

namespace
{

constexpr std::size_t kParts = static_cast<std::size_t>(PhysicsRagdollPart::Count);

void collectSkinned(Node* node, const Skeleton3D* skeleton, ct::Vector<MeshInstance3D*>& out)
{
    MeshInstance3D* instance = node->castTo<MeshInstance3D>();
    if (instance && instance->getSkeleton() == skeleton && instance->getMesh().valid())
        out.push_back(instance);
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collectSkinned(node->getChild(i), skeleton, out);
}

float segmentDistance2(const Math::Vec3& point, const Math::Vec3& a, const Math::Vec3& b)
{
    const Math::Vec3 ab = b - a;
    const float length2 = ab.Dot(ab);
    float t = length2 > 1e-12f ? (point - a).Dot(ab) / length2 : 0.0f;
    t = t < 0.0f ? 0.0f : t > 1.0f ? 1.0f : t;
    const Math::Vec3 d = point - (a + ab * t);
    return d.Dot(d);
}

int dominantPart(const SurfaceData& in, const std::uint32_t* corners,
        const ct::Vector<int>& partOfBone)
{
    float score[kParts] = {};
    for (int c = 0; c < 3; ++c)
    {
        const std::uint32_t packed = in.joints[corners[c]];
        const Math::Vec4& weight = in.weights[corners[c]];
        const float weights[4] = { weight.x, weight.y, weight.z, weight.w };
        for (int j = 0; j < 4; ++j)
        {
            const std::uint32_t bone = (packed >> (8 * j)) & 0xFFu;
            if (weights[j] > 0.0f && bone < partOfBone.size() && partOfBone[bone] >= 0)
                score[static_cast<std::size_t>(partOfBone[bone])] += weights[j];
        }
    }
    int best = 0;
    for (std::size_t p = 1; p < kParts; ++p)
        if (score[p] > score[static_cast<std::size_t>(best)]) best = static_cast<int>(p);
    return best;
}

int nearestBoneOf(const Math::Vec3& point, int part, const ct::Vector<int>& groupOfBone,
        const ct::Vector<DismemberSegment>& segments, int fallback)
{
    float best = 3.4e38f;
    int bone = fallback;
    for (std::size_t k = 0; k < segments.size(); ++k)
    {
        const int owner = segments[k].bone;
        if (owner >= static_cast<int>(groupOfBone.size()) ||
                groupOfBone[static_cast<std::size_t>(owner)] != part)
            continue;
        const float d = segmentDistance2(point, segments[k].from, segments[k].to);
        if (d < best)
        {
            best = d;
            bone = owner;
        }
    }
    return bone;
}

void copyVertex(const SurfaceData& in, std::uint32_t vertex, int part,
        const ct::Vector<int>& partOfBone, int partBone,
        const ct::Vector<DismemberSegment>& segments, SurfaceData& out)
{
    out.positions.push_back(in.positions[vertex]);
    if (!in.normals.empty()) out.normals.push_back(in.normals[vertex]);
    if (!in.tangents.empty()) out.tangents.push_back(in.tangents[vertex]);
    if (!in.colors.empty()) out.colors.push_back(in.colors[vertex]);
    if (!in.uvs.empty()) out.uvs.push_back(in.uvs[vertex]);
    if (!in.uvs2.empty()) out.uvs2.push_back(in.uvs2[vertex]);

    const std::uint32_t packed = in.joints[vertex];
    const Math::Vec4& weight = in.weights[vertex];
    const float weights[4] = { weight.x, weight.y, weight.z, weight.w };
    std::uint32_t joints = 0;
    float kept[4] = {};
    float sum = 0.0f;
    for (int j = 0; j < 4; ++j)
    {
        const std::uint32_t bone = (packed >> (8 * j)) & 0xFFu;
        if (weights[j] <= 0.0f || bone >= partOfBone.size() || partOfBone[bone] != part) continue;
        joints |= bone << (8 * j);
        kept[j] = weights[j];
        sum += weights[j];
    }
    if (sum > 0.0f)
    {
        out.joints.push_back(joints);
        out.weights.push_back(
                Math::Vec4(kept[0] / sum, kept[1] / sum, kept[2] / sum, kept[3] / sum));
    }
    else if (partBone >= 0)
    {
        const int bone = segments.empty() ? partBone
                                          : nearestBoneOf(in.positions[vertex], part, partOfBone,
                                                    segments, partBone);
        out.joints.push_back(static_cast<std::uint32_t>(bone));
        out.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
    }
    else
    {
        out.joints.push_back(packed);
        out.weights.push_back(weight);
    }
}

std::uint64_t positionKey(const Math::Vec3& p)
{
    const std::int64_t x = static_cast<std::int64_t>(floorf(p.x * 20000.0f + 0.5f));
    const std::int64_t y = static_cast<std::int64_t>(floorf(p.y * 20000.0f + 0.5f));
    const std::int64_t z = static_cast<std::int64_t>(floorf(p.z * 20000.0f + 0.5f));
    return (static_cast<std::uint64_t>(x & 0x1FFFFF) << 42) |
           (static_cast<std::uint64_t>(y & 0x1FFFFF) << 21) |
           static_cast<std::uint64_t>(z & 0x1FFFFF);
}

struct ContourPoint
{
    float x;
    float y;
};

float contourArea(const ct::Vector<ContourPoint>& contour)
{
    const int n = static_cast<int>(contour.size());
    float area = 0.0f;
    for (int p = n - 1, q = 0; q < n; p = q++)
        area += contour[static_cast<std::size_t>(p)].x * contour[static_cast<std::size_t>(q)].y -
                contour[static_cast<std::size_t>(p)].y * contour[static_cast<std::size_t>(q)].x;
    return area * 0.5f;
}

bool insideTriangle(float ax, float ay, float bx, float by, float cx, float cy, float px, float py,
        bool includeEdges)
{
    const float ex = cx - bx;
    const float ey = cy - by;
    const float fx = ax - cx;
    const float fy = ay - cy;
    const float gx = bx - ax;
    const float gy = by - ay;
    const float apx = px - ax;
    const float apy = py - ay;
    const float bpx = px - bx;
    const float bpy = py - by;
    const float cpx = px - cx;
    const float cpy = py - cy;
    const float aCrossBp = ex * bpy - ey * bpx;
    const float cCrossAp = gx * apy - gy * apx;
    const float bCrossCp = fx * cpy - fy * cpx;
    if (includeEdges) return aCrossBp > 0.0f && bCrossCp > 0.0f && cCrossAp > 0.0f;
    return aCrossBp >= 0.0f && bCrossCp >= 0.0f && cCrossAp >= 0.0f;
}

bool snip(const ct::Vector<ContourPoint>& contour, int u, int v, int w, int n,
        const ct::Vector<int>& values, bool relaxed)
{
    const ContourPoint& a = contour[static_cast<std::size_t>(values[static_cast<std::size_t>(u)])];
    const ContourPoint& b = contour[static_cast<std::size_t>(values[static_cast<std::size_t>(v)])];
    const ContourPoint& c = contour[static_cast<std::size_t>(values[static_cast<std::size_t>(w)])];
    const float threshold = relaxed ? -0.00001f : 0.00001f;
    if (threshold > ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x))) return false;
    for (int p = 0; p < n; ++p)
    {
        if (p == u || p == v || p == w) continue;
        const ContourPoint& q =
                contour[static_cast<std::size_t>(values[static_cast<std::size_t>(p)])];
        if (insideTriangle(a.x, a.y, b.x, b.y, c.x, c.y, q.x, q.y, relaxed)) return false;
    }
    return true;
}

// Ear clipping: the polygon is cut into triangles between its own corners, counter-clockwise.
bool triangulate(const ct::Vector<ContourPoint>& contour, ct::Vector<int>& result)
{
    const int n = static_cast<int>(contour.size());
    if (n < 3) return false;
    ct::Vector<int> order;
    order.resize(static_cast<std::size_t>(n));
    if (0.0f < contourArea(contour))
        for (int v = 0; v < n; ++v) order[static_cast<std::size_t>(v)] = v;
    else
        for (int v = 0; v < n; ++v) order[static_cast<std::size_t>(v)] = (n - 1) - v;
    bool relaxed = false;
    int nv = n;
    int count = 2 * nv;
    for (int v = nv - 1; nv > 2;)
    {
        if (0 >= (count--))
        {
            if (relaxed) return false;
            count = 2 * nv;
            relaxed = true;
        }
        int u = v;
        if (nv <= u) u = 0;
        v = u + 1;
        if (nv <= v) v = 0;
        int w = v + 1;
        if (nv <= w) w = 0;
        if (snip(contour, u, v, w, nv, order, relaxed))
        {
            result.push_back(order[static_cast<std::size_t>(u)]);
            result.push_back(order[static_cast<std::size_t>(v)]);
            result.push_back(order[static_cast<std::size_t>(w)]);
            for (int a = v, b = v + 1; b < nv; ++a, ++b)
                order[static_cast<std::size_t>(a)] = order[static_cast<std::size_t>(b)];
            --nv;
            count = 2 * nv;
        }
    }
    return true;
}

struct CapEdge
{
    std::uint32_t from;
    std::uint32_t to;
    int group;
    int other;
};

// Closes the cuts: where a triangle of one group meets a triangle of another across an edge, the
// edge is an open boundary of the group; the boundary of each pair of groups is closed with a fan
// of triangles to its centre, facing out of the group, on vertices of its own that carry the
// group's weights (the cap normal, the texture of the cut).
void addCaps(const SurfaceData& in, const ct::Vector<int>& triGroup,
        const ct::Vector<int>& partOfBone, const int* partBones,
        const ct::Vector<DismemberSegment>& segments, SurfaceData& out)
{
    const std::size_t triangles = triGroup.size();
    ct::HashMap<std::uint64_t, std::uint32_t> ids;
    ct::Vector<std::uint32_t> weld;
    weld.resize(in.positions.size());
    for (std::size_t v = 0; v < in.positions.size(); ++v)
    {
        const std::uint64_t key = positionKey(in.positions[v]);
        const std::uint32_t* found = ids.find(key);
        if (found) weld[v] = *found;
        else
        {
            const std::uint32_t id = static_cast<std::uint32_t>(ids.size());
            ids[key] = id;
            weld[v] = id;
        }
    }

    ct::HashMap<std::uint64_t, std::uint32_t> seen;
    ct::Vector<CapEdge> edges;
    for (std::size_t t = 0; t < triangles; ++t)
    {
        for (std::size_t e = 0; e < 3; ++e)
        {
            const std::size_t i0 = t * 3 + e;
            const std::size_t i1 = t * 3 + (e + 1) % 3;
            const std::uint32_t a =
                    in.indices.empty() ? static_cast<std::uint32_t>(i0) : in.indices[i0];
            const std::uint32_t b =
                    in.indices.empty() ? static_cast<std::uint32_t>(i1) : in.indices[i1];
            const std::uint32_t wa = weld[a];
            const std::uint32_t wb = weld[b];
            if (wa == wb) continue;
            const std::uint64_t key = wa < wb ? (static_cast<std::uint64_t>(wa) << 32) | wb
                                              : (static_cast<std::uint64_t>(wb) << 32) | wa;
            const std::uint32_t* other = seen.find(key);
            if (!other)
            {
                seen[key] = static_cast<std::uint32_t>(i0);
                continue;
            }
            const std::size_t ot = *other / 3;
            if (triGroup[ot] == triGroup[t]) continue;
            const std::size_t oe = *other % 3;
            const std::size_t j0 = ot * 3 + oe;
            const std::size_t j1 = ot * 3 + (oe + 1) % 3;
            CapEdge mine;
            mine.from = a;
            mine.to = b;
            mine.group = triGroup[t];
            mine.other = triGroup[ot];
            edges.push_back(mine);
            CapEdge theirs;
            theirs.from = in.indices.empty() ? static_cast<std::uint32_t>(j0) : in.indices[j0];
            theirs.to = in.indices.empty() ? static_cast<std::uint32_t>(j1) : in.indices[j1];
            theirs.group = triGroup[ot];
            theirs.other = triGroup[t];
            edges.push_back(theirs);
        }
    }
    if (edges.empty()) return;

    constexpr int kGroups = static_cast<int>(kParts);
    for (int g = 0; g < kGroups; ++g)
    {
        for (int h = 0; h < kGroups; ++h)
        {
            if (g == h) continue;
            ct::Vector<std::size_t> pair;
            for (std::size_t k = 0; k < edges.size(); ++k)
                if (edges[k].group == g && edges[k].other == h) pair.push_back(k);
            if (pair.size() < 3) continue;

            // Each closed ring of the boundary gets a cap of its own (a sleeve and its lining are
            // two rings): the edges are joined through the welded corners they share.
            ct::Vector<std::size_t> ring;
            ring.resize(pair.size());
            for (std::size_t i = 0; i < pair.size(); ++i) ring[i] = i;
            ct::HashMap<std::uint64_t, std::uint32_t> cornerEdge;
            for (std::size_t i = 0; i < pair.size(); ++i)
            {
                const std::uint32_t ends[2] = { weld[edges[pair[i]].from],
                    weld[edges[pair[i]].to] };
                for (int c = 0; c < 2; ++c)
                {
                    const std::uint32_t* found = cornerEdge.find(ends[c]);
                    if (!found)
                    {
                        cornerEdge[ends[c]] = static_cast<std::uint32_t>(i);
                        continue;
                    }
                    std::size_t a = i;
                    while (ring[a] != a) a = ring[a];
                    std::size_t b = *found;
                    while (ring[b] != b) b = ring[b];
                    if (a != b) ring[a] = b;
                }
            }
            for (std::size_t i = 0; i < pair.size(); ++i)
            {
                std::size_t root = i;
                while (ring[root] != root) root = ring[root];
                ring[i] = root;
            }

            for (std::size_t r = 0; r < pair.size(); ++r)
            {
                if (ring[r] != r) continue;
                // The ring in order: each edge runs as its triangle winds, the next one starts
                // where it ends. An open chain (the rest of the ring is in another surface) is
                // closed from its last corner to its first.
                ct::HashMap<std::uint64_t, std::uint32_t> startsAt;
                for (std::size_t i = 0; i < pair.size(); ++i)
                    if (ring[i] == r)
                        startsAt[weld[edges[pair[i]].from]] = static_cast<std::uint32_t>(i);
                ct::Vector<int> used;
                used.resize(pair.size());
                for (std::size_t i = 0; i < pair.size(); ++i) used[i] = 0;
                for (std::size_t seed = 0; seed < pair.size(); ++seed)
                {
                    if (ring[seed] != r || used[seed]) continue;
                    // Walk back to the start of an open chain, so it is taken whole.
                    std::size_t head = seed;
                    {
                        ct::HashMap<std::uint64_t, std::uint32_t> endsAt;
                        for (std::size_t i = 0; i < pair.size(); ++i)
                            if (ring[i] == r && !used[i])
                                endsAt[weld[edges[pair[i]].to]] = static_cast<std::uint32_t>(i);
                        for (std::size_t guard = 0; guard < pair.size(); ++guard)
                        {
                            const std::uint32_t* before = endsAt.find(weld[edges[pair[head]].from]);
                            if (!before || *before == seed) break;
                            head = *before;
                        }
                    }
                    ct::Vector<std::uint32_t> loop;
                    std::size_t at = head;
                    for (std::size_t guard = 0; guard < pair.size() && !used[at]; ++guard)
                    {
                        used[at] = 1;
                        loop.push_back(edges[pair[at]].from);
                        const std::uint32_t* next = startsAt.find(weld[edges[pair[at]].to]);
                        if (!next || used[*next])
                        {
                            if (!next) loop.push_back(edges[pair[at]].to);
                            break;
                        }
                        at = *next;
                    }
                    if (loop.size() < 3) continue;
                    // The cap faces out of the group: against the direction its triangles run
                    // along the ring.
                    ct::Vector<std::uint32_t> polygon;
                    for (std::size_t i = loop.size(); i > 0; --i) polygon.push_back(loop[i - 1]);

                    Math::Vec3 normal(0.0f, 0.0f, 0.0f);
                    Math::Vec3 centre(0.0f, 0.0f, 0.0f);
                    for (std::size_t i = 0; i < polygon.size(); ++i)
                    {
                        const Math::Vec3& a = in.positions[polygon[i]];
                        const Math::Vec3& b = in.positions[polygon[(i + 1) % polygon.size()]];
                        normal = normal + Math::Vec3((a.y - b.y) * (a.z + b.z),
                                                  (a.z - b.z) * (a.x + b.x),
                                                  (a.x - b.x) * (a.y + b.y));
                        centre = centre + a;
                    }
                    const float length = normal.Length();
                    if (length < 1e-12f) continue;
                    normal = normal * (1.0f / length);
                    centre = centre * (1.0f / static_cast<float>(polygon.size()));
                    const Math::Vec3 helper = fabsf(normal.x) < 0.9f ? Math::Vec3(1.0f, 0.0f, 0.0f)
                                                                     : Math::Vec3(0.0f, 1.0f, 0.0f);
                    Math::Vec3 axisU = Math::Vec3::Cross(helper, normal);
                    axisU = axisU * (1.0f / axisU.Length());
                    const Math::Vec3 axisV = Math::Vec3::Cross(normal, axisU);
                    ct::Vector<ContourPoint> contour;
                    for (std::size_t i = 0; i < polygon.size(); ++i)
                    {
                        const Math::Vec3 d = in.positions[polygon[i]] - centre;
                        ContourPoint point;
                        point.x = d.Dot(axisU);
                        point.y = d.Dot(axisV);
                        contour.push_back(point);
                    }
                    ct::Vector<int> triangles;
                    if (!triangulate(contour, triangles)) continue;

                    // The cap's own corners: the group's weights, the cap normal, the tangent in
                    // the cap's plane.
                    ct::Vector<std::uint32_t> corner;
                    for (std::size_t i = 0; i < polygon.size(); ++i)
                    {
                        corner.push_back(static_cast<std::uint32_t>(out.positions.size()));
                        copyVertex(in, polygon[i], g, partOfBone, partBones[g], segments, out);
                        if (!out.normals.empty()) out.normals.back() = normal;
                        if (!out.tangents.empty())
                        {
                            Math::Vec4& tangent = out.tangents.back();
                            Math::Vec3 axis(tangent.x, tangent.y, tangent.z);
                            axis = axis - normal * normal.Dot(axis);
                            const float axisLength = axis.Length();
                            if (axisLength > 1e-6f) axis = axis * (1.0f / axisLength);
                            tangent = Math::Vec4(axis.x, axis.y, axis.z, tangent.w);
                        }
                    }
                    for (std::size_t i = 0; i + 2 < triangles.size(); i += 3)
                    {
                        out.indices.push_back(corner[static_cast<std::size_t>(triangles[i])]);
                        out.indices.push_back(corner[static_cast<std::size_t>(triangles[i + 1])]);
                        out.indices.push_back(corner[static_cast<std::size_t>(triangles[i + 2])]);
                    }
                }
            }
        }
    }
}

} // namespace

Dismember3D::Dismember3D()
        : Node3D(),
          mRagdoll(nullptr),
          mMeshCut(false),
          mRadius(3.0f),
          mForce(8.0f)
{
}

Dismember3D::~Dismember3D()
{
    destroyCuts();
}

void Dismember3D::setRagdoll(Ragdoll3D* ragdoll)
{
    mRagdollPath = ct::String();
    mRagdoll = ragdoll;
}

Ragdoll3D* Dismember3D::getRagdoll() const
{
    return mRagdoll;
}

void Dismember3D::setRagdollPath(const ct::String& path)
{
    mRagdollPath = path;
    resolveRagdoll();
}

ct::String Dismember3D::getRagdollPath() const
{
    if (mRagdoll) return getPathTo(mRagdoll);
    return mRagdollPath;
}

void Dismember3D::resolveRagdoll()
{
    Ragdoll3D* ragdoll = nullptr;
    if (!mRagdollPath.empty())
    {
        Node* node = getNode(mRagdollPath);
        ragdoll = node ? node->castTo<Ragdoll3D>() : nullptr;
    }
    const ct::String pending = mRagdollPath;
    setRagdoll(ragdoll);
    if (!ragdoll) mRagdollPath = pending;
}

void Dismember3D::setRadius(float radius)
{
    mRadius = radius < 0.0f ? 0.0f : radius;
}

float Dismember3D::getRadius() const
{
    return mRadius;
}

void Dismember3D::setForce(float force)
{
    mForce = force < 0.0f ? 0.0f : force;
}

float Dismember3D::getForce() const
{
    return mForce;
}

bool Dismember3D::ensureActive()
{
    if (!mRagdoll) return false;
    return mRagdoll->isActive() || mRagdoll->start();
}

bool Dismember3D::detach(PhysicsRagdollPart part)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || part == PhysicsRagdollPart::Pelvis || part >= PhysicsRagdollPart::Count ||
            !ensureActive())
        return false;
    const PhysicsRagdollId ragdoll = mRagdoll->getRagdoll();
    bool separated[kParts];
    for (std::size_t i = 0; i < kParts; ++i)
        separated[i] = server->ragdollIsDetached(ragdoll, static_cast<PhysicsRagdollPart>(i));
    separated[static_cast<std::size_t>(part)] = true;
    cutMeshes(separated);
    return server->ragdollDetach(ragdoll, part);
}

bool Dismember3D::isDetached(PhysicsRagdollPart part) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mRagdoll) return false;
    return server->ragdollIsDetached(mRagdoll->getRagdoll(), part);
}

int Dismember3D::getDetachedCount() const
{
    int count = 0;
    for (std::size_t i = 0; i < kParts; ++i)
        if (isDetached(static_cast<PhysicsRagdollPart>(i))) ++count;
    return count;
}

bool Dismember3D::explode(const Math::Vec3& origin)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !ensureActive()) return false;
    bool separated[kParts];
    for (std::size_t i = 0; i < kParts; ++i) separated[i] = true;
    cutMeshes(separated);
    const PhysicsRagdollId ragdoll = mRagdoll->getRagdoll();
    for (std::size_t i = 0; i < kParts; ++i)
        server->ragdollDetach(ragdoll, static_cast<PhysicsRagdollPart>(i));
    for (std::size_t i = 0; i < kParts; ++i)
    {
        const PhysicsRagdollPart part = static_cast<PhysicsRagdollPart>(i);
        const Math::Vec3 center = server->ragdollGetPartTransform(ragdoll, part).GetTranslation();
        Math::Vec3 direction = center - origin;
        const float distance = direction.Length();
        if (mRadius > 0.0f && distance >= mRadius) continue;
        const float falloff = mRadius > 0.0f ? 1.0f - distance / mRadius : 1.0f;
        direction = distance > 1e-3f ? direction / distance : Math::Vec3(0.0f, 1.0f, 0.0f);
        const float mass = server->ragdollGetPartMass(ragdoll, part);
        server->ragdollApplyImpulse(ragdoll, part, direction * (mForce * falloff * mass),
                Math::Vec3(0.0f, 0.0f, 0.0f));
    }
    return true;
}

bool Dismember3D::isMeshCut() const
{
    return mMeshCut;
}

MeshId Dismember3D::cutMesh(MeshId source, const ct::Vector<int>& partOfBone, const int* partBones,
        const ct::Vector<DismemberSegment>& segments) const
{
    MeshStore* store = MeshStore::singleton();
    const MeshInfo* info = store ? store->get(source) : nullptr;
    if (!info || info->surfaces.empty()) return MeshId();
    const std::size_t surfaceCount = info->surfaces.size();
    ct::Vector<MaterialId> materials;
    for (std::size_t s = 0; s < surfaceCount; ++s)
    {
        const SurfaceData* in = store->surfaceSource(source, static_cast<std::uint32_t>(s));
        if (!in || in->topology != prisma::Topology::Triangles || in->joints.empty() ||
                in->weights.size() != in->joints.size())
            return MeshId();
        materials.push_back(info->surfaces[s].material);
    }
    const ct::String name = info->name + " (cut)";

    const MeshId mesh = store->create(name.c_str());
    for (std::size_t s = 0; s < surfaceCount; ++s)
    {
        const SurfaceData& in = *store->surfaceSource(source, static_cast<std::uint32_t>(s));
        const std::size_t count = in.indices.empty() ? in.positions.size() : in.indices.size();
        SurfaceData out;
        out.topology = in.topology;
        ct::HashMap<std::uint64_t, std::uint32_t> remap;
        ct::Vector<int> triGroup;
        for (std::size_t t = 0; t + 2 < count; t += 3)
        {
            std::uint32_t corners[3];
            for (int c = 0; c < 3; ++c)
                corners[c] = in.indices.empty() ? static_cast<std::uint32_t>(t + c)
                                                : in.indices[t + static_cast<std::size_t>(c)];
            int part = 0;
            if (segments.empty()) part = dominantPart(in, corners, partOfBone);
            else
            {
                const Math::Vec3 centre = (in.positions[corners[0]] + in.positions[corners[1]] +
                                                  in.positions[corners[2]]) *
                                          (1.0f / 3.0f);
                float best = 3.4e38f;
                for (std::size_t k = 0; k < segments.size(); ++k)
                {
                    const float d = segmentDistance2(centre, segments[k].from, segments[k].to);
                    if (d < best)
                    {
                        best = d;
                        part = segments[k].group;
                    }
                }
            }
            triGroup.push_back(part);
            for (int c = 0; c < 3; ++c)
            {
                const std::uint64_t key = (static_cast<std::uint64_t>(corners[c]) << 8) |
                                          static_cast<std::uint64_t>(part);
                const std::uint32_t* found = remap.find(key);
                if (found)
                {
                    out.indices.push_back(*found);
                    continue;
                }
                const std::uint32_t index = static_cast<std::uint32_t>(out.positions.size());
                copyVertex(in, corners[c], part, partOfBone, partBones[part], segments, out);
                remap[key] = index;
                out.indices.push_back(index);
            }
        }
        if (!segments.empty()) addCaps(in, triGroup, partOfBone, partBones, segments, out);
        if (!store->addSurface(mesh, out, name.c_str()))
        {
            store->destroy(mesh);
            return MeshId();
        }
        store->setSurfaceMaterial(mesh, static_cast<std::uint32_t>(s), materials[s]);
    }
    return mesh;
}

void Dismember3D::cutMeshes(const bool* separated)
{
    if (!mRagdoll) return;
    Skeleton3D* skeleton = mRagdoll->getSkeleton();
    if (!skeleton) return;

    int partBones[kParts];
    for (std::size_t p = 0; p < kParts; ++p)
        partBones[p] = mRagdoll->getPartBone(static_cast<PhysicsRagdollPart>(p));
    const int boneCount = static_cast<int>(skeleton->getBoneCount());
    ct::Vector<int> partOfBone;
    partOfBone.resize(static_cast<std::size_t>(boneCount));
    for (int bone = 0; bone < boneCount; ++bone)
    {
        int part = 0;
        for (int walk = bone; walk >= 0 && part == 0; walk = skeleton->getBoneParent(walk))
            for (std::size_t p = 1; p < kParts; ++p)
                if (partBones[p] == walk)
                {
                    part = static_cast<int>(p);
                    break;
                }
        partOfBone[static_cast<std::size_t>(bone)] = part;
    }

    // The parts hang from each other as the physics ragdoll joins them (the arms from the spine,
    // as the head does), which is not always the hierarchy of the skeleton (a Bip01 holds the
    // clavicles under the neck).
    int parentPart[kParts];
    for (std::size_t p = 0; p < kParts; ++p)
    {
        const std::size_t parent = static_cast<std::size_t>(
                mRagdoll->getPartParent(static_cast<PhysicsRagdollPart>(p)));
        parentPart[p] = p == 0 || parent >= kParts ? 0 : static_cast<int>(parent);
    }
    int group[kParts];
    for (std::size_t p = 0; p < kParts; ++p)
    {
        int walk = static_cast<int>(p);
        for (std::size_t guard = 0; walk != 0 && !separated[walk] && guard < kParts; ++guard)
            walk = parentPart[walk];
        group[p] = separated[walk] ? walk : 0;
    }
    ct::Vector<int> groupOfBone;
    groupOfBone.resize(partOfBone.size());
    for (std::size_t bone = 0; bone < partOfBone.size(); ++bone)
    {
        // The bones above the pelvis (the root of a Bip01) are not moved by the ragdoll: no
        // vertex may stay bound to them.
        bool driven = partBones[0] < 0;
        for (int walk = static_cast<int>(bone); walk >= 0 && !driven;
                walk = skeleton->getBoneParent(walk))
            driven = walk == partBones[0];
        groupOfBone[bone] = driven ? group[static_cast<std::size_t>(partOfBone[bone])] : -1;
    }

    // Where a bone sits, as opposed to what moves it: the bones between a part and the first part
    // above it in the skeleton (the clavicles, between the neck and the arms) sit with the part
    // that carries them in the physics.
    ct::Vector<int> placeOfBone = partOfBone;
    for (std::size_t p = 1; p < kParts; ++p)
    {
        if (partBones[p] < 0) continue;
        ct::Vector<int> between;
        int walk = skeleton->getBoneParent(partBones[p]);
        for (; walk >= 0; walk = skeleton->getBoneParent(walk))
        {
            bool isPartBone = false;
            for (std::size_t q = 0; q < kParts; ++q)
                if (partBones[q] == walk) isPartBone = true;
            if (isPartBone) break;
            between.push_back(walk);
        }
        const int above = walk >= 0 ? partOfBone[static_cast<std::size_t>(walk)] : 0;
        if (above == parentPart[p]) continue;
        for (std::size_t i = 0; i < between.size(); ++i)
            placeOfBone[static_cast<std::size_t>(between[i])] = parentPart[p];
    }
    // ...and so do the bones hanging from them that no part carries (a Bip01 jacket's bones,
    // under the clavicles, run down the chest to the waist).
    {
        ct::Vector<int> moved;
        moved.resize(placeOfBone.size());
        for (std::size_t bone = 0; bone < placeOfBone.size(); ++bone)
            moved[bone] = placeOfBone[bone] != partOfBone[bone] ? placeOfBone[bone] : -1;
        for (int bone = 0; bone < boneCount; ++bone)
        {
            for (int walk = skeleton->getBoneParent(bone); walk >= 0;
                    walk = skeleton->getBoneParent(walk))
            {
                bool isPartBone = false;
                for (std::size_t q = 0; q < kParts; ++q)
                    if (partBones[q] == walk || partBones[q] == bone) isPartBone = true;
                if (isPartBone) break;
                if (moved[static_cast<std::size_t>(walk)] >= 0)
                {
                    placeOfBone[static_cast<std::size_t>(bone)] =
                            moved[static_cast<std::size_t>(walk)];
                    break;
                }
            }
        }
    }

    // Which group a triangle goes to is decided by where it is, not by the skin weights (which
    // on some models bind a head to the shoulders): the group of the bone whose segment, from its
    // joint to the joint of a child, is nearest in the bind pose. The cut falls on a plane through
    // each joint; the weights only smooth the joints inside a group.
    ct::Vector<DismemberSegment> segments;
    {
        const Skeleton& data = skeleton->getSkeleton();
        ct::Vector<Math::Vec3> joint;
        ct::Vector<int> children;
        joint.resize(static_cast<std::size_t>(boneCount));
        children.resize(static_cast<std::size_t>(boneCount));
        float reach = 0.0f;
        for (int bone = 0; bone < boneCount; ++bone)
        {
            joint[static_cast<std::size_t>(bone)] = data.bone(static_cast<std::uint32_t>(bone))
                                                            .inverseBind.InverseAffine()
                                                            .GetTranslation();
            children[static_cast<std::size_t>(bone)] = 0;
        }
        for (int bone = 0; bone < boneCount; ++bone)
        {
            const int parent = skeleton->getBoneParent(bone);
            if (parent < 0) continue;
            DismemberSegment segment;
            segment.from = joint[static_cast<std::size_t>(parent)];
            segment.to = joint[static_cast<std::size_t>(bone)];
            segment.bone = parent;
            bool childIsPart = false;
            for (std::size_t q = 0; q < kParts; ++q)
                if (partBones[q] == bone) childIsPart = true;
            const int owner = childIsPart ? parent : bone;
            segment.group =
                    group[static_cast<std::size_t>(placeOfBone[static_cast<std::size_t>(owner)])];
            // The ragdoll's head is the neck bone: the neck itself stays with the body, so the
            // head comes off at the base of the skull and not with the collar.
            if (parent == partBones[static_cast<std::size_t>(PhysicsRagdollPart::Head)])
                segment.group = group[static_cast<std::size_t>(
                        parentPart[static_cast<std::size_t>(PhysicsRagdollPart::Head)])];
            reach += (segment.to - segment.from).Length();
            segments.push_back(segment);
            ++children[static_cast<std::size_t>(parent)];
        }
        for (int bone = 0; bone < boneCount; ++bone)
        {
            if (children[static_cast<std::size_t>(bone)] != 0) continue;
            DismemberSegment segment;
            segment.from = joint[static_cast<std::size_t>(bone)];
            segment.to = segment.from;
            segment.bone = bone;
            segment.group =
                    group[static_cast<std::size_t>(placeOfBone[static_cast<std::size_t>(bone)])];
            segments.push_back(segment);
        }
        if (reach < 1e-4f) segments.clear();
    }

    MeshStore* store = MeshStore::singleton();
    if (mMeshCut)
    {
        for (std::size_t i = 0; i < mCuts.size(); ++i)
        {
            Node* node = getNode(mCuts[i].path);
            MeshInstance3D* instance = node ? node->castTo<MeshInstance3D>() : nullptr;
            if (!instance || instance->getMesh() != mCuts[i].cut) continue;
            const MeshId cut = cutMesh(mCuts[i].original, groupOfBone, partBones, segments);
            if (!cut.valid()) continue;
            instance->setMesh(cut);
            if (store) store->destroy(mCuts[i].cut);
            mCuts[i].cut = cut;
        }
        return;
    }
    mMeshCut = true;

    ct::Vector<MeshInstance3D*> instances;
    collectSkinned(skeleton, skeleton, instances);
    for (std::size_t i = 0; i < instances.size(); ++i)
    {
        MeshInstance3D* instance = instances[i];
        const MeshId original = instance->getMesh();
        const MeshId cut = cutMesh(original, groupOfBone, partBones, segments);
        if (!cut.valid()) continue;
        instance->setMesh(cut);
        Cut entry;
        entry.path = getPathTo(instance);
        entry.original = original;
        entry.cut = cut;
        mCuts.push_back(entry);
    }
}

void Dismember3D::restoreMeshes()
{
    for (std::size_t i = 0; i < mCuts.size(); ++i)
    {
        Node* node = getNode(mCuts[i].path);
        MeshInstance3D* instance = node ? node->castTo<MeshInstance3D>() : nullptr;
        if (instance && instance->getMesh() == mCuts[i].cut) instance->setMesh(mCuts[i].original);
    }
    destroyCuts();
    mMeshCut = false;
}

void Dismember3D::destroyCuts()
{
    MeshStore* store = MeshStore::singleton();
    if (store)
        for (std::size_t i = 0; i < mCuts.size(); ++i) store->destroy(mCuts[i].cut);
    mCuts.clear();
}

void Dismember3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    if (what == kNotificationEnterTree && !mRagdollPath.empty()) resolveRagdoll();
    if (what == kNotificationExitTree) restoreMeshes();
}

void Dismember3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Dismember3D;
    DB::addProperty<P>(PropertyInfo::nodePath("ragdoll", "Ragdoll3D"), &P::setRagdollPath,
            &P::getRagdollPath);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "radius", 0.0, 100.0, 0.01, true),
            &P::setRadius, &P::getRadius);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "force", 0.0, 1000.0, 0.01, true),
            &P::setForce, &P::getForce);
}

} // namespace prisma3d
