#include "Check.h"

#include "prisma3d/core/Random.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Tree3D.h"
#include "prisma3d/scene/TreeGenerator.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kEps = 1e-3f;

bool near(float a, float b, float eps = kEps)
{
    return fabsf(a - b) <= eps;
}

bool sameVec3(const ct::Vector<Math::Vec3>& a, const ct::Vector<Math::Vec3>& b)
{
    if (a.size() != b.size()) return false;
    return a.empty() || memcmp(a.data(), b.data(), a.size() * sizeof(Math::Vec3)) == 0;
}

bool sameVec2(const ct::Vector<Math::Vec2>& a, const ct::Vector<Math::Vec2>& b)
{
    if (a.size() != b.size()) return false;
    return a.empty() || memcmp(a.data(), b.data(), a.size() * sizeof(Math::Vec2)) == 0;
}

bool sameVec4(const ct::Vector<Math::Vec4>& a, const ct::Vector<Math::Vec4>& b)
{
    if (a.size() != b.size()) return false;
    return a.empty() || memcmp(a.data(), b.data(), a.size() * sizeof(Math::Vec4)) == 0;
}

bool sameIndices(const ct::Vector<std::uint32_t>& a, const ct::Vector<std::uint32_t>& b)
{
    if (a.size() != b.size()) return false;
    return a.empty() || memcmp(a.data(), b.data(), a.size() * sizeof(std::uint32_t)) == 0;
}

bool sameSurface(const SurfaceData& a, const SurfaceData& b)
{
    return sameVec3(a.positions, b.positions) && sameVec3(a.normals, b.normals) &&
           sameVec4(a.tangents, b.tangents) && sameVec2(a.uvs, b.uvs) &&
           sameIndices(a.indices, b.indices);
}

bool sameGeometry(const TreeGeometry& a, const TreeGeometry& b)
{
    return sameSurface(a.bark, b.bark) && sameSurface(a.leaves, b.leaves) &&
           sameIndices(a.trunkIndices, b.trunkIndices);
}

TreeGeometry build(const TreeParams& params)
{
    TreeGeometry geometry;
    TreeGenerator::generate(params, geometry);
    return geometry;
}

bool finite3(const Math::Vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

bool surfaceValid(const SurfaceData& s)
{
    const std::size_t vertices = s.positions.size();
    if (vertices == 0 || s.indices.empty()) return false;
    if (s.normals.size() != vertices || s.tangents.size() != vertices || s.uvs.size() != vertices)
        return false;
    if (!s.colors.empty() || !s.uvs2.empty()) return false;
    if (s.indices.size() % 3 != 0) return false;
    for (std::size_t i = 0; i < s.indices.size(); ++i)
        if (s.indices[i] >= vertices) return false;
    for (std::size_t i = 0; i < vertices; ++i)
    {
        if (!finite3(s.positions[i]) || !finite3(s.normals[i])) return false;
        if (!near(s.normals[i].Length(), 1.0f)) return false;
        const Math::Vec4& t = s.tangents[i];
        if (!isfinite(t.x) || !isfinite(t.y) || !isfinite(t.z) || !isfinite(t.w)) return false;
        if (!isfinite(s.uvs[i].x) || !isfinite(s.uvs[i].y)) return false;
    }
    return true;
}

bool geometryValid(const TreeGeometry& g)
{
    if (!surfaceValid(g.bark) || !surfaceValid(g.leaves)) return false;
    if (g.trunkIndices.empty() || g.trunkIndices.size() % 3 != 0) return false;
    for (std::size_t i = 0; i < g.trunkIndices.size(); ++i)
        if (g.trunkIndices[i] >= g.bark.positions.size()) return false;
    return true;
}

Math::Box boundsOf(const SurfaceData& s)
{
    return MeshStore::computeAabb(s.positions);
}

float baseRadius(const SurfaceData& s, int& count)
{
    float radius = 0.0f;
    count = 0;
    for (std::size_t i = 0; i < s.positions.size(); ++i)
    {
        const Math::Vec3& p = s.positions[i];
        if (fabsf(p.y) > 1e-5f) continue;
        ++count;
        const float r = sqrtf(p.x * p.x + p.z * p.z);
        if (r > radius) radius = r;
    }
    return radius;
}

TreeParams straightTrunk()
{
    TreeParams params;
    params.levels = 1;
    params.trunkSteps = 3;
    params.trunkKink = 0.0f;
    params.climbRate = 1.5f;
    params.trunkLength = 2.5f;
    params.maxRadius = 0.25f;
    return params;
}

void testDeterminism()
{
    for (std::uint32_t i = 0; i < TreeGenerator::presetCount(); ++i)
    {
        const TreeParams params = TreeGenerator::preset(i).params;
        CHECK(sameGeometry(build(params), build(params)));
    }

    TreeParams a = TreeGenerator::preset(0).params;
    TreeParams b = a;
    b.seed = a.seed + 1;
    const TreeGeometry ga = build(a);
    const TreeGeometry gb = build(b);
    CHECK(!sameGeometry(ga, gb));
    CHECK(!sameVec3(ga.bark.positions, gb.bark.positions));
    CHECK(!sameVec3(ga.leaves.positions, gb.leaves.positions));

    TreeParams defaults;
    TreeParams other;
    other.seed = 11;
    CHECK(!sameGeometry(build(defaults), build(other)));
}

void testStructure()
{
    const TreeGeometry defaults = build(TreeParams());
    CHECK(geometryValid(defaults));

    for (std::uint32_t i = 0; i < TreeGenerator::presetCount(); ++i)
    {
        const TreePreset& preset = TreeGenerator::preset(i);
        CHECK(preset.name != nullptr);
        CHECK(geometryValid(build(preset.params)));
    }

    const TreeGeometry g = build(TreeGenerator::preset(0).params);
    CHECK(g.leaves.positions.size() % 8 == 0);
    CHECK(g.leaves.indices.size() == g.leaves.positions.size() / 8 * 12);
    CHECK(g.trunkIndices.size() < g.bark.indices.size());

    bool uvsInBand = true;
    for (std::size_t i = 0; i < g.leaves.uvs.size(); ++i)
    {
        const Math::Vec2& uv = g.leaves.uvs[i];
        if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f) uvsInBand = false;
    }
    CHECK(uvsInBand);

    bool leafFlat = true;
    for (std::size_t i = 0; i + 7 < g.leaves.positions.size(); i += 8)
    {
        for (std::size_t k = 0; k < 4; ++k)
        {
            if (!near((g.leaves.positions[i + k] - g.leaves.positions[i + 4 + k]).Length(), 0.0f))
                leafFlat = false;
            if (!near((g.leaves.normals[i + k] + g.leaves.normals[i + 4 + k]).Length(), 0.0f))
                leafFlat = false;
            if (!near((g.leaves.normals[i + k] - g.leaves.normals[i]).Length(), 0.0f))
                leafFlat = false;
        }
        const std::size_t t = i / 8 * 12;
        for (std::size_t quad = 0; quad < 2; ++quad)
        {
            for (std::size_t tri = 0; tri < 2; ++tri)
            {
                const std::size_t at = t + quad * 6 + tri * 3;
                const Math::Vec3 a = g.leaves.positions[g.leaves.indices[at]];
                const Math::Vec3 b = g.leaves.positions[g.leaves.indices[at + 1]];
                const Math::Vec3 c = g.leaves.positions[g.leaves.indices[at + 2]];
                const Math::Vec3 face = (b - a).Cross(c - a);
                const Math::Vec3 normal = g.leaves.normals[g.leaves.indices[at]];
                if (face.Length() > 1e-6f && !(face.Dot(normal) > 0.0f)) leafFlat = false;
            }
        }
    }
    CHECK(leafFlat);

    int tangentsOrthogonal = 0;
    int tangentsMeasured = 0;
    for (std::size_t i = 0; i < g.bark.positions.size(); ++i)
    {
        const Math::Vec4& t = g.bark.tangents[i];
        const float length = t.xyz().Length();
        if (length < 1e-3f) continue;
        ++tangentsMeasured;
        if (fabsf(t.xyz().Dot(g.bark.normals[i])) / length < 0.2f) ++tangentsOrthogonal;
    }
    CHECK(tangentsMeasured > 0);
    CHECK(tangentsOrthogonal * 10 >= tangentsMeasured * 9);
}

void testBounds()
{
    const TreeParams params = straightTrunk();
    const TreeGeometry g = build(params);
    CHECK(geometryValid(g));

    const Math::Box bark = boundsOf(g.bark);
    CHECK(near(bark.min.y, 0.0f));

    int count = 0;
    const float radius = baseRadius(g.bark, count);
    CHECK(count == static_cast<int>(params.segments));
    CHECK(near(radius, params.maxRadius / params.radiusFalloffRate, 1e-4f));

    const float trunkTop =
            params.trunkLength + params.climbRate * static_cast<float>(params.trunkSteps);
    CHECK(bark.max.y >= trunkTop - params.maxRadius);
    CHECK(bark.max.y <= trunkTop + 2.0f * params.initialBranchLength + params.maxRadius);

    float trunkTopVertex = 0.0f;
    float trunkReach = 0.0f;
    for (std::size_t i = 0; i < g.trunkIndices.size(); ++i)
    {
        const Math::Vec3& p = g.bark.positions[g.trunkIndices[i]];
        if (p.y > trunkTopVertex) trunkTopVertex = p.y;
        const float r = sqrtf(p.x * p.x + p.z * p.z);
        if (r > trunkReach) trunkReach = r;
    }
    CHECK(trunkTopVertex >= trunkTop - params.maxRadius);
    CHECK(trunkTopVertex <= trunkTop + params.maxRadius);
    CHECK(trunkReach <= 2.0f * params.maxRadius / params.radiusFalloffRate);

    TreeParams shorter = params;
    shorter.trunkLength = 1.0f;
    CHECK(boundsOf(build(shorter).bark).max.y < bark.max.y - 1.0f);

    const Math::Box leaves = boundsOf(g.leaves);
    CHECK(leaves.max.y > leaves.min.y);
    CHECK(leaves.min.y > 0.0f);

    TreeParams bigLeaves = params;
    bigLeaves.twigScale = params.twigScale * 2.0f;
    const Math::Box wide = boundsOf(build(bigLeaves).leaves);
    CHECK((wide.max - wide.min).Length() > (leaves.max - leaves.min).Length());
}

void countOutward(const SurfaceData& surface, int& measured, int& agreeing)
{
    measured = 0;
    agreeing = 0;
    for (std::size_t i = 0; i + 2 < surface.indices.size(); i += 3)
    {
        const std::uint32_t ia = surface.indices[i];
        const std::uint32_t ib = surface.indices[i + 1];
        const std::uint32_t ic = surface.indices[i + 2];
        const Math::Vec3 face = (surface.positions[ib] - surface.positions[ia])
                                        .Cross(surface.positions[ic] - surface.positions[ia]);
        if (face.Length() < 1e-6f) continue;
        ++measured;
        const Math::Vec3 normal = surface.normals[ia] + surface.normals[ib] + surface.normals[ic];
        if (face.Dot(normal) > 0.0f) ++agreeing;
    }
}

void testWinding()
{
    const TreeParams params = straightTrunk();
    const TreeGeometry g = build(params);

    int measured = 0;
    int outward = 0;
    for (std::size_t i = 0; i + 2 < g.trunkIndices.size(); i += 3)
    {
        const Math::Vec3 a = g.bark.positions[g.trunkIndices[i]];
        const Math::Vec3 b = g.bark.positions[g.trunkIndices[i + 1]];
        const Math::Vec3 c = g.bark.positions[g.trunkIndices[i + 2]];
        const Math::Vec3 face = (b - a).Cross(c - a);
        if (face.Length() < 1e-6f) continue;
        const Math::Vec3 centroid = (a + b + c) / 3.0f;
        const Math::Vec3 radial(centroid.x, 0.0f, centroid.z);
        const Math::Vec3 sideways(face.x, 0.0f, face.z);
        if (radial.Length() < 1e-4f || sideways.Length() < 1e-6f) continue;
        ++measured;
        if (radial.Dot(sideways) > 0.0f) ++outward;
    }
    CHECK(measured > 0);
    CHECK(outward == measured);

    int barkMeasured = 0;
    int barkOutward = 0;
    countOutward(g.bark, barkMeasured, barkOutward);
    CHECK(barkMeasured > 0);
    CHECK(barkOutward * 100 >= barkMeasured * 98);

    for (std::uint32_t i = 0; i < TreeGenerator::presetCount(); ++i)
    {
        const TreeGeometry preset = build(TreeGenerator::preset(i).params);
        int measured = 0;
        int agreeing = 0;
        countOutward(preset.bark, measured, agreeing);
        CHECK(measured > 0);
        CHECK(agreeing * 100 >= measured * 95);
    }
}

void testParameters()
{
    TreeParams base;
    TreeParams deeper = base;
    deeper.levels = base.levels + 1;
    const TreeGeometry gb = build(base);
    const TreeGeometry gd = build(deeper);
    CHECK(gd.bark.positions.size() > gb.bark.positions.size());
    CHECK(gd.bark.indices.size() > gb.bark.indices.size());
    CHECK(gd.leaves.positions.size() > gb.leaves.positions.size());

    TreeParams longer = base;
    longer.trunkSteps = base.trunkSteps + 3;
    CHECK(build(longer).bark.positions.size() > gb.bark.positions.size());

    TreeParams round = base;
    round.segments = 12;
    CHECK(build(round).bark.positions.size() > gb.bark.positions.size());

    TreeParams thick = base;
    thick.maxRadius = base.maxRadius * 2.0f;
    int countA = 0;
    int countB = 0;
    const float radiusA = baseRadius(gb.bark, countA);
    const float radiusB = baseRadius(build(thick).bark, countB);
    CHECK(countA == countB);
    CHECK(radiusB > radiusA * 1.9f);
    CHECK(near(radiusB, thick.maxRadius / thick.radiusFalloffRate, 1e-4f));

    TreeParams odd = base;
    odd.segments = 7;
    TreeParams even = base;
    even.segments = 6;
    CHECK(sameGeometry(build(odd), build(even)));
    TreeParams few = base;
    few.segments = 2;
    TreeParams four = base;
    four.segments = 4;
    CHECK(sameGeometry(build(few), build(four)));

    const TreeParams clamped = TreeGenerator::sanitize(TreeParams());
    CHECK(clamped == TreeParams());
    TreeParams wild;
    wild.levels = 1000;
    wild.segments = 1000;
    wild.trunkSteps = 100000;
    const TreeParams limited = TreeGenerator::sanitize(wild);
    CHECK(limited.levels >= 1 && limited.levels <= TreeGenerator::kMaxLevels);
    CHECK(limited.segments == TreeGenerator::kMaxSegments);
    CHECK(limited.trunkSteps <= TreeGenerator::kMaxTrunkSteps);
    CHECK(static_cast<std::uint64_t>(limited.trunkSteps + 1u) * (1ull << limited.levels) *
                    limited.segments <=
            TreeGenerator::kMaxBudget);
}

void testDegenerate()
{
    TreeParams zero;
    memset(static_cast<void*>(&zero), 0, sizeof(zero));
    CHECK(geometryValid(build(zero)));

    TreeParams flat;
    flat.trunkLength = 0.0f;
    flat.initialBranchLength = 0.0f;
    flat.maxRadius = 0.0f;
    flat.twigScale = 0.0f;
    CHECK(geometryValid(build(flat)));

    TreeParams negative;
    negative.trunkLength = -3.0f;
    negative.maxRadius = -1.0f;
    negative.radiusFalloffRate = -2.0f;
    negative.taperRate = 0.0f;
    negative.initialBranchLength = -1.0f;
    negative.twigScale = -1.0f;
    negative.lengthFalloffFactor = -1.0f;
    CHECK(geometryValid(build(negative)));

    TreeParams backwards;
    backwards.clumpMin = 1.5f;
    backwards.clumpMax = -0.5f;
    backwards.lengthFalloffFactor = 1.5f;
    backwards.lengthFalloffPower = -2.0f;
    backwards.branchFactor = 50.0f;
    CHECK(geometryValid(build(backwards)));

    TreeParams nonFinite;
    nonFinite.trunkLength = NAN;
    nonFinite.climbRate = INFINITY;
    nonFinite.twistRate = -INFINITY;
    nonFinite.maxRadius = NAN;
    nonFinite.dropAmount = NAN;
    CHECK(geometryValid(build(nonFinite)));

    TreeParams huge;
    huge.levels = 100;
    huge.segments = 5000;
    huge.trunkSteps = 5000;
    huge.trunkLength = 1.0e20f;
    huge.maxRadius = 1.0e20f;
    CHECK(geometryValid(build(huge)));

    TreeParams noTrunk;
    noTrunk.trunkSteps = 0;
    noTrunk.levels = 1;
    CHECK(geometryValid(build(noTrunk)));

    TreeGeometry reused;
    TreeGenerator::generate(TreeGenerator::preset(0).params, reused);
    const std::size_t first = reused.bark.positions.size();
    TreeGenerator::generate(TreeParams(), reused);
    CHECK(reused.bark.positions.size() != first);
    CHECK(sameGeometry(reused, build(TreeParams())));
}

void testPresetsAndRandom()
{
    CHECK(TreeGenerator::presetCount() == 7);
    const TreePreset& beyond = TreeGenerator::preset(1000);
    CHECK(strcmp(beyond.name, "Beech") == 0);
    CHECK(strcmp(TreeGenerator::preset(0).name, "Oak") == 0);
    CHECK(TreeGenerator::preset(0).params.seed == 696);
    CHECK(TreeGenerator::preset(0).params.trunkSteps == 9);
    CHECK(near(TreeGenerator::preset(2).params.twigScale, 0.65f));

    Random first(1234);
    Random second(1234);
    Random third(99);
    const TreeParams a = TreeGenerator::randomParams(first);
    const TreeParams b = TreeGenerator::randomParams(second);
    const TreeParams c = TreeGenerator::randomParams(third);
    CHECK(a == b);
    CHECK(a != c);

    Random random(7);
    bool inBands = true;
    bool valid = true;
    for (int i = 0; i < 25; ++i)
    {
        const TreeParams p = TreeGenerator::randomParams(random);
        if (p.levels < 3 || p.levels > 5) inBands = false;
        if (p.segments < 6 || p.segments > 10 || p.segments % 2 != 0) inBands = false;
        if (p.trunkSteps < 1 || p.trunkSteps > 5) inBands = false;
        if (p.seed < 1 || p.seed > 99999) inBands = false;
        if (!(p.clumpMin < p.clumpMax)) inBands = false;
        if (p.maxRadius < 0.07f || p.maxRadius > 0.24f) inBands = false;
        if (!geometryValid(build(p))) valid = false;
    }
    CHECK(inBands);
    CHECK(valid);
}

void testNode()
{
    SceneTree tree;
    tree.initialize();
    Tree3D* node = tree.getRoot()->createChild<Tree3D>();
    CHECK(node != nullptr);
    CHECK(node->castTo<Tree3D>() == node);
    CHECK(node->castTo<VisualInstance3D>() == node);
    CHECK(node->castTo<Node3D>() == node);

    CHECK(node->isDirty());
    CHECK(node->getBuildCount() == 0);
    CHECK(node->getBarkGeometry().positions.empty());
    CHECK(node->getParams() == TreeParams());

    CHECK(node->rebuild());
    CHECK(node->getBuildCount() == 1);
    CHECK(!node->isDirty());
    CHECK(geometryValid(build(TreeParams())));
    CHECK(sameSurface(node->getBarkGeometry(), build(TreeParams()).bark));
    CHECK(sameSurface(node->getLeafGeometry(), build(TreeParams()).leaves));

    CHECK(!node->rebuild());
    CHECK(node->getBuildCount() == 1);

    node->setSeed(node->getSeed());
    CHECK(!node->isDirty());
    node->setParams(TreeParams());
    CHECK(!node->isDirty());

    node->setSeed(42);
    CHECK(node->isDirty());
    CHECK(node->getSeed() == 42);
    CHECK(node->getParams().seed == 42);
    CHECK(node->rebuild());
    CHECK(node->getBuildCount() == 2);
    TreeParams seeded;
    seeded.seed = 42;
    CHECK(sameSurface(node->getBarkGeometry(), build(seeded).bark));

    node->setSeed(43);
    node->setSeed(44);
    node->setParams(TreeGenerator::preset(0).params);
    CHECK(node->rebuild());
    CHECK(node->getBuildCount() == 3);
    CHECK(!node->rebuild());
    CHECK(node->getParams() == TreeGenerator::preset(0).params);
    CHECK(sameSurface(node->getBarkGeometry(), build(TreeGenerator::preset(0).params).bark));

    const Math::Box aabb = node->getAabb();
    const Math::Box bark = boundsOf(node->getBarkGeometry());
    const Math::Box leaves = boundsOf(node->getLeafGeometry());
    CHECK(aabb.min.y <= bark.min.y + 1e-6f);
    CHECK(aabb.max.y >= bark.max.y - 1e-6f);
    CHECK(aabb.max.x >= leaves.max.x - 1e-6f && aabb.min.x <= leaves.min.x + 1e-6f);
    CHECK(aabb.max.z >= leaves.max.z - 1e-6f && aabb.min.z <= leaves.min.z + 1e-6f);

    node->setBarkMaterial(MaterialId());
    node->setLeafMaterial(MaterialId());
    CHECK(!node->getBarkMaterial().valid());
    CHECK(!node->getLeafMaterial().valid());

    Tree3D* second = tree.getRoot()->createChild<Tree3D>();
    CHECK(second->isDirty());
    CHECK(second->getBuildCount() == 0);
    CHECK(second->rebuild());
    CHECK(sameSurface(second->getBarkGeometry(), build(TreeParams()).bark));

    tree.finalize();
}

void testCollision()
{
    SceneTree tree;
    tree.initialize();
    Tree3D* node = tree.getRoot()->createChild<Tree3D>();

    CollisionMesh empty;
    CHECK(!node->buildCollision(empty));
    CHECK(!empty.valid());

    node->rebuild();
    CollisionMesh collision;
    CHECK(node->buildCollision(collision));
    CHECK(collision.valid());
    CHECK(collision.triangleCount() > 0);

    SweepHit hit;
    const TreeParams params;
    const float expected = params.maxRadius / params.radiusFalloffRate;
    CHECK(collision.raycast(Math::Ray(Math::Vec3(-5.0f, 0.5f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f)),
            10.0f, hit));
    CHECK(hit.t > 5.0f - expected * 1.2f && hit.t < 5.0f);
    CHECK(collision.raycast(Math::Ray(Math::Vec3(0.0f, 1.0f, 5.0f), Math::Vec3(0.0f, 0.0f, -1.0f)),
            10.0f, hit));
    CHECK(hit.t > 5.0f - expected * 1.2f && hit.t < 5.0f);
    CHECK(!collision.raycast(Math::Ray(Math::Vec3(-5.0f, 0.5f, 3.0f), Math::Vec3(1.0f, 0.0f, 0.0f)),
            10.0f, hit));
    CHECK(!collision.raycast(
            Math::Ray(Math::Vec3(-5.0f, -1.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f)), 10.0f, hit));

    const Math::Box& bounds = collision.bounds();
    CHECK(near(bounds.min.y, 0.0f));
    CHECK(bounds.max.y > 1.0f);

    TreeGeometry geometry = build(TreeParams());
    CHECK(collision.triangleCount() == geometry.trunkIndices.size() / 3);

    tree.finalize();
}

} // namespace

void testShadowGeometry()
{
    TreeGeometry full;
    TreeGenerator::generate(TreeParams(), full);
    const std::size_t leafCount = full.leaves.positions.size() / 8;
    CHECK(leafCount > 6);

    TreeGeometry shadow;
    TreeGenerator::buildShadowGeometry(full, 6, shadow);
    const std::size_t expected = (leafCount + 5) / 6;
    CHECK(shadow.leaves.positions.size() == expected * 8);
    CHECK(shadow.leaves.indices.size() == expected * 12);
    CHECK(shadow.leaves.normals.size() == shadow.leaves.positions.size());
    CHECK(shadow.bark.positions.size() == full.bark.positions.size());
    CHECK(shadow.bark.indices.size() == full.trunkIndices.size());
    CHECK(shadow.bark.indices.size() < full.bark.indices.size());
    for (std::size_t i = 0; i < shadow.leaves.indices.size(); ++i)
        CHECK(shadow.leaves.indices[i] < shadow.leaves.positions.size());

    Math::Vec3 fullCentre(0.0f, 0.0f, 0.0f);
    Math::Vec3 shadowCentre(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 4; ++i)
    {
        fullCentre += full.leaves.positions[static_cast<std::size_t>(i)];
        shadowCentre += shadow.leaves.positions[static_cast<std::size_t>(i)];
    }
    CHECK((fullCentre - shadowCentre).Length() < 1e-3f);
    const float fullEdge = (full.leaves.positions[1] - full.leaves.positions[0]).Length();
    const float shadowEdge = (shadow.leaves.positions[1] - shadow.leaves.positions[0]).Length();
    CHECK(fabsf(shadowEdge - fullEdge * sqrtf(6.0f)) < 1e-3f);

    TreeGeometry same;
    TreeGenerator::buildShadowGeometry(full, 1, same);
    CHECK(same.leaves.positions.size() == full.leaves.positions.size());
    CHECK(fabsf((same.leaves.positions[1] - same.leaves.positions[0]).Length() - fullEdge) < 1e-4f);
    TreeGenerator::buildShadowGeometry(full, 0, same);
    CHECK(same.leaves.positions.size() == full.leaves.positions.size());

    const std::size_t fullTriangles = (full.bark.indices.size() + full.leaves.indices.size()) / 3;
    const std::size_t shadowTriangles =
            (shadow.bark.indices.size() + shadow.leaves.indices.size()) / 3;
    CHECK(shadowTriangles * 3 < fullTriangles);
    printf("tree shadow: %u triangles drawn per shadow pass, %u with the shadow mesh\n",
            static_cast<unsigned>(fullTriangles), static_cast<unsigned>(shadowTriangles));
    for (std::uint32_t i = 0; i < TreeGenerator::presetCount(); ++i)
    {
        TreeGeometry presetFull;
        TreeGeometry presetShadow;
        TreeGenerator::generate(TreeGenerator::preset(i).params, presetFull);
        TreeGenerator::buildShadowGeometry(presetFull, 6, presetShadow);
        printf("  %s: %u -> %u\n", TreeGenerator::preset(i).name,
                static_cast<unsigned>(
                        (presetFull.bark.indices.size() + presetFull.leaves.indices.size()) / 3),
                static_cast<unsigned>(
                        (presetShadow.bark.indices.size() + presetShadow.leaves.indices.size()) /
                        3));
    }

    SceneTree sceneTree;
    sceneTree.initialize();
    Tree3D* tree = sceneTree.getRoot()->createChild<Tree3D>();
    CHECK(tree->getShadowLeafStep() == 6);
    tree->rebuild();
    tree->setShadowLeafStep(0);
    CHECK(tree->getShadowLeafStep() == 1);
    CHECK(tree->isDirty());
    tree->setShadowLeafStep(1);
    tree->rebuild();
    CHECK(!tree->isDirty());
}

int main()
{
    testDeterminism();
    testStructure();
    testBounds();
    testWinding();
    testParameters();
    testDegenerate();
    testPresetsAndRandom();
    testNode();
    testCollision();
    testShadowGeometry();

    if (failures) printf("%d checks failed\n", failures);
    else
        printf("ok\n");
    return failures ? 1 : 0;
}
