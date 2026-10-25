#include "Check.h"

#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/Curve3D.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/Road3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/terrain/TerrainPath.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const float kPi = 3.14159265358979323846f;
const float kQuarter = 0.5522847498f;

bool near(float a, float b, float tolerance = kTolerance)
{
    return fabsf(a - b) <= tolerance;
}

bool isVec3(const Math::Vec3& v, float x, float y, float z, float tolerance = kTolerance)
{
    return near(v.x, x, tolerance) && near(v.y, y, tolerance) && near(v.z, z, tolerance);
}

bool isVec2(const Math::Vec2& v, float x, float y)
{
    return near(v.x, x) && near(v.y, y);
}

void addLine(Curve3D& curve, const Math::Vec3& from, const Math::Vec3& to)
{
    const Math::Vec3 third = (to - from) / 3.0f;
    curve.addPoint(from, Math::Vec3(0.0f, 0.0f, 0.0f), third);
    curve.addPoint(to, -third, Math::Vec3(0.0f, 0.0f, 0.0f));
}

void addStraight(Curve3D& curve, float length)
{
    addLine(curve, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -length));
}

void addBend(Curve3D& curve)
{
    const float k = 10.0f * kQuarter;
    curve.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -k));
    curve.addPoint(Math::Vec3(-10.0f, 0.0f, -10.0f), Math::Vec3(k, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f));
}

void addCircle(Curve3D& curve, float radius)
{
    const float k = radius * kQuarter;
    curve.addPoint(Math::Vec3(radius, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -k),
            Math::Vec3(0.0f, 0.0f, k));
    curve.addPoint(Math::Vec3(0.0f, 0.0f, radius), Math::Vec3(k, 0.0f, 0.0f),
            Math::Vec3(-k, 0.0f, 0.0f));
    curve.addPoint(Math::Vec3(-radius, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, k),
            Math::Vec3(0.0f, 0.0f, -k));
    curve.addPoint(Math::Vec3(0.0f, 0.0f, -radius), Math::Vec3(-k, 0.0f, 0.0f),
            Math::Vec3(k, 0.0f, 0.0f));
}

struct Scene
{
    SceneTree tree;
    Path3D* path;
    Road3D* road;

    Scene()
            : tree(),
              path(nullptr),
              road(nullptr)
    {
        tree.initialize();
        path = tree.getRoot()->createChild<Path3D>();
        road = path->createChild<Road3D>();
    }

    Curve3D& curve()
    {
        return path->getCurve();
    }

    const SurfaceData& build()
    {
        road->rebuild();
        return road->getGeometry();
    }
};

bool allTrianglesFaceUp(const SurfaceData& g)
{
    if (g.indices.empty()) return false;
    for (std::size_t i = 0; i + 2 < g.indices.size(); i += 3)
    {
        const Math::Vec3 a = g.positions[g.indices[i]];
        const Math::Vec3 b = g.positions[g.indices[i + 1]];
        const Math::Vec3 c = g.positions[g.indices[i + 2]];
        const Math::Vec3 face = (b - a).Cross(c - a);
        if (!(face.y > 1e-6f)) return false;
    }
    return true;
}

bool indicesValid(const SurfaceData& g)
{
    for (std::size_t i = 0; i < g.indices.size(); ++i)
        if (g.indices[i] >= g.positions.size()) return false;
    return g.indices.size() % 3 == 0 && g.normals.size() == g.positions.size() &&
           g.uvs.size() == g.positions.size() && g.tangents.size() == g.positions.size();
}

float radiusFrom(const Math::Vec3& centre, const Math::Vec3& point)
{
    const float dx = point.x - centre.x;
    const float dz = point.z - centre.z;
    return sqrtf(dx * dx + dz * dz);
}

void testStraight()
{
    Scene scene;
    addStraight(scene.curve(), 8.0f);
    scene.road->setWidth(4.0f);
    scene.road->setInterval(2.0f);
    scene.road->setTextureRepeat(4.0f);

    const SurfaceData& g = scene.build();
    CHECK(near(scene.curve().getBakedLength(), 8.0f));
    CHECK(g.topology == prisma::Topology::Triangles);
    CHECK(g.positions.size() == 10);
    CHECK(g.indices.size() == 24);
    CHECK(indicesValid(g));
    CHECK(allTrianglesFaceUp(g));

    for (int r = 0; r < 5; ++r)
    {
        const float z = -2.0f * static_cast<float>(r);
        const float v = 0.5f * static_cast<float>(r);
        const std::size_t left = static_cast<std::size_t>(r) * 2;
        CHECK(isVec3(g.positions[left], -2.0f, 0.0f, z));
        CHECK(isVec3(g.positions[left + 1], 2.0f, 0.0f, z));
        CHECK(isVec3(g.normals[left], 0.0f, 1.0f, 0.0f));
        CHECK(isVec3(g.normals[left + 1], 0.0f, 1.0f, 0.0f));
        CHECK(isVec2(g.uvs[left], 0.0f, v));
        CHECK(isVec2(g.uvs[left + 1], 1.0f, v));
    }

    CHECK(g.indices[0] == 0 && g.indices[1] == 3 && g.indices[2] == 2);
    CHECK(g.indices[3] == 0 && g.indices[4] == 1 && g.indices[5] == 3);
    CHECK(g.indices[6] == 2 && g.indices[7] == 5 && g.indices[8] == 4);

    for (std::size_t i = 0; i < g.tangents.size(); ++i)
    {
        const Math::Vec3 t(g.tangents[i].x, g.tangents[i].y, g.tangents[i].z);
        CHECK(near(t.Length(), 1.0f, 1e-3f));
        CHECK(near(t.Dot(g.normals[i]), 0.0f, 1e-3f));
    }

    const Math::Box bounds = scene.road->getAabb();
    CHECK(isVec3(bounds.min, -2.0f, 0.0f, -8.0f));
    CHECK(isVec3(bounds.max, 2.0f, 0.0f, 0.0f));

    scene.road->setTextureRepeat(0.0f);
    const SurfaceData& normalised = scene.build();
    CHECK(near(normalised.uvs[0].y, 0.0f));
    CHECK(near(normalised.uvs[2].y, 0.25f));
    CHECK(near(normalised.uvs[8].y, 1.0f));
}

void testIntervals()
{
    Scene scene;
    addStraight(scene.curve(), 8.0f);
    scene.road->setInterval(3.0f);
    const SurfaceData& distance = scene.build();
    CHECK(distance.positions.size() == 8);
    CHECK(near(distance.positions[2].z, -8.0f / 3.0f));
    CHECK(near(distance.positions[6].z, -8.0f));

    Scene subdivided;
    subdivided.curve().addPoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -4.0f / 3.0f));
    subdivided.curve().addPoint(Math::Vec3(0.0f, 0.0f, -4.0f), Math::Vec3(0.0f, 0.0f, 4.0f / 3.0f),
            Math::Vec3(0.0f, 0.0f, -4.0f / 3.0f));
    subdivided.curve().addPoint(Math::Vec3(0.0f, 0.0f, -8.0f), Math::Vec3(0.0f, 0.0f, 4.0f / 3.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f));
    subdivided.road->setIntervalType(Road3D::IntervalType::Subdivide);
    subdivided.road->setInterval(1.0f);
    const SurfaceData& g = subdivided.build();
    CHECK(g.positions.size() == 6);
    CHECK(near(g.positions[0].z, 0.0f));
    CHECK(near(g.positions[2].z, -4.0f));
    CHECK(near(g.positions[4].z, -8.0f));
    CHECK(subdivided.curve().getPointCount() == 3);

    Scene simplified;
    addStraight(simplified.curve(), 8.0f);
    simplified.road->setInterval(1.0f);
    simplified.road->setSimplifyAngle(5.0f);
    simplified.road->setTextureRepeat(4.0f);
    const SurfaceData& flat = simplified.build();
    CHECK(flat.positions.size() == 4);
    CHECK(flat.indices.size() == 6);
    CHECK(near(flat.positions[0].z, 0.0f));
    CHECK(near(flat.positions[2].z, -8.0f));
    CHECK(near(flat.uvs[2].y, 2.0f));
    CHECK(allTrianglesFaceUp(flat));
}

void testPointWidths()
{
    Scene scene;
    const float third = 4.0f / 3.0f;
    scene.curve().addPoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -third));
    scene.curve().addPoint(Math::Vec3(0.0f, 0.0f, -4.0f), Math::Vec3(0.0f, 0.0f, third),
            Math::Vec3(0.0f, 0.0f, -third));
    scene.curve().addPoint(Math::Vec3(0.0f, 0.0f, -8.0f), Math::Vec3(0.0f, 0.0f, third),
            Math::Vec3(0.0f, 0.0f, 0.0f));
    scene.road->setWidth(4.0f);
    scene.road->setInterval(2.0f);

    CHECK(near(scene.road->getPointWidth(0), 4.0f));
    scene.road->setPointWidth(0, 2.0f);
    scene.road->setPointWidth(2, 6.0f);
    CHECK(near(scene.road->getPointWidth(0), 2.0f));
    CHECK(near(scene.road->getPointWidth(1), 4.0f));
    CHECK(near(scene.road->getPointWidth(2), 6.0f));

    const SurfaceData& a = scene.build();
    CHECK(a.positions.size() == 10);
    const float halfA[5] = { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f };
    for (int r = 0; r < 5; ++r)
    {
        const std::size_t left = static_cast<std::size_t>(r) * 2;
        CHECK(isVec3(a.positions[left], -halfA[r], 0.0f, -2.0f * static_cast<float>(r)));
        CHECK(isVec3(a.positions[left + 1], halfA[r], 0.0f, -2.0f * static_cast<float>(r)));
    }

    scene.road->setPointWidth(1, 8.0f);
    const SurfaceData& b = scene.build();
    const float halfB[5] = { 1.0f, 2.5f, 4.0f, 3.5f, 3.0f };
    for (int r = 0; r < 5; ++r)
    {
        const std::size_t left = static_cast<std::size_t>(r) * 2;
        CHECK(near(b.positions[left].x, -halfB[r]));
        CHECK(near(b.positions[left + 1].x, halfB[r]));
        CHECK(near(b.uvs[left].x, 0.0f));
        CHECK(near(b.uvs[left + 1].x, 1.0f));
    }
    CHECK(allTrianglesFaceUp(b));

    scene.road->clearPointWidths();
    const SurfaceData& c = scene.build();
    CHECK(near(c.positions[0].x, -2.0f));
    CHECK(near(c.positions[9].x, 2.0f));
}

void testProfile()
{
    Scene scene;
    addStraight(scene.curve(), 2.0f);
    scene.road->setWidth(4.0f);
    scene.road->setInterval(2.0f);

    ct::Vector<Math::Vec2> profile;
    profile.push_back(Math::Vec2(-2.0f, 0.3f));
    profile.push_back(Math::Vec2(-1.0f, 0.0f));
    profile.push_back(Math::Vec2(1.0f, 0.0f));
    profile.push_back(Math::Vec2(2.0f, 0.3f));
    scene.road->setProfile(profile);
    CHECK(scene.road->getProfile().size() == 4);

    const SurfaceData& g = scene.build();
    CHECK(g.positions.size() == 12);
    CHECK(g.indices.size() == 18);
    CHECK(indicesValid(g));
    CHECK(allTrianglesFaceUp(g));

    CHECK(isVec3(g.positions[0], -2.0f, 0.3f, 0.0f));
    CHECK(isVec3(g.positions[1], -1.0f, 0.0f, 0.0f));
    CHECK(isVec3(g.positions[2], -1.0f, 0.0f, 0.0f));
    CHECK(isVec3(g.positions[3], 1.0f, 0.0f, 0.0f));
    CHECK(isVec3(g.positions[4], 1.0f, 0.0f, 0.0f));
    CHECK(isVec3(g.positions[5], 2.0f, 0.3f, 0.0f));
    CHECK(isVec3(g.positions[6], -2.0f, 0.3f, -2.0f));
    CHECK(isVec3(g.positions[11], 2.0f, 0.3f, -2.0f));

    CHECK(isVec3(g.normals[0], 0.2873479f, 0.9578263f, 0.0f));
    CHECK(isVec3(g.normals[1], 0.2873479f, 0.9578263f, 0.0f));
    CHECK(isVec3(g.normals[2], 0.0f, 1.0f, 0.0f));
    CHECK(isVec3(g.normals[4], -0.2873479f, 0.9578263f, 0.0f));
    CHECK(isVec3(g.normals[5], -0.2873479f, 0.9578263f, 0.0f));

    CHECK(near(g.uvs[0].x, 0.0f));
    CHECK(near(g.uvs[1].x, 0.2553853f));
    CHECK(near(g.uvs[2].x, 0.2553853f));
    CHECK(near(g.uvs[3].x, 0.7446147f));
    CHECK(near(g.uvs[4].x, 0.7446147f));
    CHECK(near(g.uvs[5].x, 1.0f));

    for (int s = 0; s < 3; ++s)
    {
        const std::size_t a = static_cast<std::size_t>(s) * 2;
        CHECK(g.indices[static_cast<std::size_t>(s) * 6 + 0] == a);
        CHECK(g.indices[static_cast<std::size_t>(s) * 6 + 1] == a + 7);
        CHECK(g.indices[static_cast<std::size_t>(s) * 6 + 2] == a + 6);
        CHECK(g.indices[static_cast<std::size_t>(s) * 6 + 3] == a);
        CHECK(g.indices[static_cast<std::size_t>(s) * 6 + 4] == a + 1);
        CHECK(g.indices[static_cast<std::size_t>(s) * 6 + 5] == a + 7);
    }

    scene.road->setPointWidth(0, 8.0f);
    scene.road->setPointWidth(1, 8.0f);
    const SurfaceData& wide = scene.build();
    CHECK(isVec3(wide.positions[0], -4.0f, 0.3f, 0.0f));
    CHECK(isVec3(wide.positions[1], -2.0f, 0.0f, 0.0f));
    CHECK(isVec3(wide.normals[0], 0.1483405f, 0.9889364f, 0.0f));
    CHECK(near(wide.uvs[1].x, 0.2553853f));

    scene.road->clearProfile();
    CHECK(scene.road->getProfile().empty());
    scene.road->clearPointWidths();
    const SurfaceData& flat = scene.build();
    CHECK(flat.positions.size() == 4);
}

void checkBend(const SurfaceData& g, std::size_t expectedVertices)
{
    const Math::Vec3 centre(-10.0f, 0.0f, 0.0f);
    CHECK(g.positions.size() == expectedVertices);
    CHECK(indicesValid(g));
    CHECK(allTrianglesFaceUp(g));
    bool onSide = true;
    for (std::size_t i = 0; i + 1 < g.positions.size(); i += 2)
    {
        if (!near(radiusFrom(centre, g.positions[i]), 9.0f, 0.02f)) onSide = false;
        if (!near(radiusFrom(centre, g.positions[i + 1]), 11.0f, 0.02f)) onSide = false;
        if (!near(g.positions[i].y, 0.0f) || !near(g.positions[i + 1].y, 0.0f)) onSide = false;
        if (!isVec3(g.normals[i], 0.0f, 1.0f, 0.0f, 1e-3f)) onSide = false;
    }
    CHECK(onSide);
    CHECK(isVec3(g.positions[0], -1.0f, 0.0f, 0.0f, 0.1f));
    CHECK(isVec3(g.positions[1], 1.0f, 0.0f, 0.0f, 0.1f));
    CHECK(isVec3(g.positions[g.positions.size() - 2], -10.0f, 0.0f, -9.0f, 0.1f));
    CHECK(isVec3(g.positions[g.positions.size() - 1], -10.0f, 0.0f, -11.0f, 0.1f));
}

void testBend()
{
    Scene scene;
    addBend(scene.curve());
    scene.road->setWidth(2.0f);
    scene.road->setInterval(1.0f);

    const float length = scene.curve().getBakedLength();
    CHECK(near(length, 10.0f * kPi * 0.5f, 0.02f));
    const int segments = static_cast<int>(ceilf(length));
    const SurfaceData& g = scene.build();
    checkBend(g, static_cast<std::size_t>(segments + 1) * 2);
    const std::size_t fullCount = g.positions.size();


    scene.road->setRotationAccurate(true);
    const SurfaceData& accurate = scene.build();
    checkBend(accurate, static_cast<std::size_t>(segments + 1) * 2);

    scene.road->setRotationMode(Road3D::RotationMode::Path);
    const SurfaceData& path = scene.build();
    checkBend(path, static_cast<std::size_t>(segments + 1) * 2);

    scene.road->setRotationAccurate(false);
    scene.road->setRotationMode(Road3D::RotationMode::Fixed);
    const SurfaceData& fixed = scene.build();
    CHECK(isVec3(fixed.positions[0], -1.0f, 0.0f, 0.0f));
    CHECK(isVec3(fixed.positions[1], 1.0f, 0.0f, 0.0f));
    CHECK(isVec3(fixed.positions[fixed.positions.size() - 2], -11.0f, 0.0f, -10.0f));
    CHECK(isVec3(fixed.positions[fixed.positions.size() - 1], -9.0f, 0.0f, -10.0f));

    scene.road->setRotationMode(Road3D::RotationMode::PathFollow);
    scene.road->setSimplifyAngle(10.0f);
    const SurfaceData& simplified = scene.build();
    CHECK(simplified.positions.size() > 4);
    CHECK(simplified.positions.size() < fullCount);
    CHECK(allTrianglesFaceUp(simplified));
}

void testClosedLoop()
{
    Scene scene;
    scene.curve().setClosed(true);
    addCircle(scene.curve(), 4.0f);
    scene.road->setWidth(2.0f);
    scene.road->setInterval(2.5f);
    scene.road->setTextureRepeat(4.0f);

    const float length = scene.curve().getBakedLength();
    CHECK(near(length, 2.0f * kPi * 4.0f, 0.03f));
    const int segments = static_cast<int>(ceilf(length / 2.5f));
    CHECK(segments == 11);

    const SurfaceData& g = scene.build();
    CHECK(g.positions.size() == static_cast<std::size_t>(segments + 1) * 2);
    CHECK(g.indices.size() == static_cast<std::size_t>(segments) * 6);
    CHECK(indicesValid(g));
    CHECK(allTrianglesFaceUp(g));

    const std::size_t last = g.positions.size() - 2;
    CHECK(isVec3(g.positions[last], g.positions[0].x, g.positions[0].y, g.positions[0].z));
    CHECK(isVec3(g.positions[last + 1], g.positions[1].x, g.positions[1].y, g.positions[1].z));
    CHECK(near(g.uvs[0].y, 0.0f));
    CHECK(near(g.uvs[last].y, length / 4.0f, 1e-3f));
    CHECK(near(g.uvs[last + 1].y, length / 4.0f, 1e-3f));
    CHECK(isVec3(g.positions[0], 5.0f, 0.0f, 0.0f, 0.3f));
    CHECK(isVec3(g.positions[1], 3.0f, 0.0f, 0.0f, 0.3f));

    bool ring = true;
    for (std::size_t i = 0; i + 1 < g.positions.size(); i += 2)
    {
        const float a = radiusFrom(Math::Vec3(0.0f, 0.0f, 0.0f), g.positions[i]);
        const float b = radiusFrom(Math::Vec3(0.0f, 0.0f, 0.0f), g.positions[i + 1]);
        const float low = a < b ? a : b;
        const float high = a < b ? b : a;
        if (!near(low, 3.0f, 0.08f) || !near(high, 5.0f, 0.08f)) ring = false;
    }
    CHECK(ring);

    float previousV = -1.0f;
    bool increasing = true;
    for (std::size_t i = 0; i < g.uvs.size(); i += 2)
    {
        if (!(g.uvs[i].y > previousV)) increasing = false;
        previousV = g.uvs[i].y;
    }
    CHECK(increasing);
}

void testTilt()
{
    Scene scene;
    addStraight(scene.curve(), 8.0f);
    scene.curve().setPointTilt(1, kPi / 6.0f);
    scene.road->setWidth(4.0f);
    scene.road->setInterval(2.0f);

    const float cos15 = cosf(kPi / 12.0f);
    const float sin15 = sinf(kPi / 12.0f);
    const float cos30 = cosf(kPi / 6.0f);
    const float sin30 = sinf(kPi / 6.0f);

    const SurfaceData& g = scene.build();
    CHECK(g.positions.size() == 10);
    CHECK(isVec3(g.positions[0], -2.0f, 0.0f, 0.0f));
    CHECK(isVec3(g.positions[1], 2.0f, 0.0f, 0.0f));
    CHECK(isVec3(g.positions[4], -2.0f * cos15, 2.0f * sin15, -4.0f, 2e-3f));
    CHECK(isVec3(g.positions[5], 2.0f * cos15, -2.0f * sin15, -4.0f, 2e-3f));
    CHECK(isVec3(g.positions[8], -2.0f * cos30, 2.0f * sin30, -8.0f, 2e-3f));
    CHECK(isVec3(g.positions[9], 2.0f * cos30, -2.0f * sin30, -8.0f, 2e-3f));
    CHECK(isVec3(g.normals[8], sin30, cos30, 0.0f, 2e-3f));
    CHECK(isVec3(g.normals[9], sin30, cos30, 0.0f, 2e-3f));
    CHECK(allTrianglesFaceUp(g));

    scene.road->setRotationMode(Road3D::RotationMode::Path);
    const SurfaceData& flat = scene.build();
    CHECK(isVec3(flat.positions[8], -2.0f, 0.0f, -8.0f));
    CHECK(isVec3(flat.positions[9], 2.0f, 0.0f, -8.0f));
}

void testDirtyTracking()
{
    Scene scene;
    CHECK(scene.road->castTo<Road3D>() == scene.road);
    CHECK(scene.road->castTo<VisualInstance3D>() == scene.road);
    CHECK(scene.road->castTo<Path3D>() == nullptr);
    CHECK(scene.road->isDirty());
    CHECK(scene.road->getBuildCount() == 0);

    CHECK(scene.road->flush());
    CHECK(scene.road->getBuildCount() == 1);
    CHECK(scene.road->getGeometry().positions.empty());
    CHECK(!scene.road->flush());
    CHECK(scene.road->getBuildCount() == 1);

    addStraight(scene.curve(), 8.0f);
    CHECK(scene.road->isDirty());
    CHECK(scene.road->flush());
    CHECK(scene.road->getBuildCount() == 2);
    const std::size_t count = scene.road->getGeometry().positions.size();
    CHECK(count == 18);
    CHECK(near(scene.road->getGeometry().positions[count - 2].z, -8.0f));

    CHECK(!scene.road->flush());
    scene.road->setWidth(scene.road->getWidth());
    scene.road->setInterval(scene.road->getInterval());
    scene.road->setTextureRepeat(scene.road->getTextureRepeat());
    scene.road->setSimplifyAngle(scene.road->getSimplifyAngle());
    scene.road->setRotationMode(scene.road->getRotationMode());
    scene.road->setRotationAccurate(scene.road->isRotationAccurate());
    scene.road->setIntervalType(scene.road->getIntervalType());
    CHECK(!scene.road->isDirty());
    CHECK(!scene.road->flush());
    scene.curve().getBakedLength();
    scene.curve().sampleBaked(1.0f);
    CHECK(!scene.road->flush());
    CHECK(scene.road->getBuildCount() == 2);

    scene.curve().setPointPosition(1, Math::Vec3(0.0f, 0.0f, -4.0f));
    CHECK(scene.road->isDirty());
    CHECK(scene.road->flush());
    CHECK(scene.road->getBuildCount() == 3);
    const SurfaceData& shorter = scene.road->getGeometry();
    CHECK(shorter.positions.size() < count);
    CHECK(near(shorter.positions[shorter.positions.size() - 2].z, -4.0f, 1e-3f));

    scene.road->setWidth(6.0f);
    CHECK(scene.road->isDirty());
    CHECK(scene.road->flush());
    CHECK(scene.road->getBuildCount() == 4);
    CHECK(near(scene.road->getGeometry().positions[1].x, 3.0f));

    scene.road->setPointWidth(0, 2.0f);
    CHECK(scene.road->flush());
    scene.road->setPointWidth(0, 2.0f);
    CHECK(!scene.road->isDirty());
    scene.road->setMaterial(MaterialId());
    CHECK(!scene.road->isDirty());
    CHECK(scene.road->getBuildCount() == 5);

    scene.road->rebuild();
    CHECK(scene.road->getBuildCount() == 6);
}

void testDegenerate()
{
    Scene scene;
    CHECK(scene.build().positions.empty());
    CHECK(scene.build().indices.empty());

    scene.curve().addPoint(Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(scene.build().positions.empty());

    scene.curve().addPoint(Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(scene.build().positions.empty());
    CHECK(scene.build().indices.empty());

    CollisionMesh collision;
    CHECK(!scene.road->buildCollision(collision));
    const Math::Box bounds = scene.road->getAabb();
    CHECK(bounds.min.x >= bounds.max.x);

    scene.curve().setPointPosition(1, Math::Vec3(1.0f, 2.0f, -5.0f));
    CHECK(scene.road->flush());
    CHECK(!scene.road->getGeometry().positions.empty());

    scene.curve().clearPoints();
    CHECK(scene.road->flush());
    CHECK(scene.road->getGeometry().positions.empty());

    SceneTree tree;
    tree.initialize();
    Road3D* orphan = tree.getRoot()->createChild<Road3D>();
    orphan->rebuild();
    CHECK(orphan->getGeometry().positions.empty());
    CHECK(orphan->getBuildCount() == 1);
}

void testCollision()
{
    Scene scene;
    addStraight(scene.curve(), 8.0f);
    scene.road->setWidth(4.0f);
    scene.road->setInterval(2.0f);
    scene.build();

    CollisionMesh collision;
    CHECK(scene.road->buildCollision(collision));
    CHECK(collision.valid());
    CHECK(collision.triangleCount() == 8);
    CHECK(isVec3(collision.bounds().min, -2.0f, 0.0f, -8.0f));
    CHECK(isVec3(collision.bounds().max, 2.0f, 0.0f, 0.0f));

    SweepHit hit;
    CHECK(collision.raycast(Math::Ray(Math::Vec3(0.5f, 3.0f, -3.0f), Math::Vec3(0.0f, -1.0f, 0.0f)),
            10.0f, hit));
    CHECK(near(hit.t, 3.0f));
    CHECK(isVec3(hit.normal, 0.0f, 1.0f, 0.0f));
    CHECK(!collision.raycast(
            Math::Ray(Math::Vec3(5.0f, 3.0f, -3.0f), Math::Vec3(0.0f, -1.0f, 0.0f)), 10.0f, hit));
    CHECK(!collision.raycast(Math::Ray(Math::Vec3(0.5f, 3.0f, 2.0f), Math::Vec3(0.0f, -1.0f, 0.0f)),
            10.0f, hit));
}

Math::Vec3 worldOf(const Scene& scene, const Math::Vec3& local)
{
    return scene.road->getGlobalTransform().TransformPoint(local);
}

void addStraightAt(Curve3D& curve, float x, float length)
{
    addLine(curve, Math::Vec3(x, 0.0f, 0.0f), Math::Vec3(x, 0.0f, -length));
}

int countBelow(const Scene& scene, const Road3D::GroundFunction& ground, float offset)
{
    int below = 0;
    const SurfaceData& g = scene.road->getGeometry();
    for (std::size_t i = 0; i < g.positions.size(); ++i)
    {
        const Math::Vec3 w = worldOf(scene, g.positions[i]);
        if (w.y < ground(w) + offset - 1e-5f) ++below;
    }
    return below;
}

float worstError(const Scene& scene, const Road3D::GroundFunction& ground, float offset)
{
    float worst = 0.0f;
    const SurfaceData& g = scene.road->getGeometry();
    for (std::size_t i = 0; i < g.positions.size(); ++i)
    {
        const Math::Vec3 w = worldOf(scene, g.positions[i]);
        const float error = fabsf(w.y - (ground(w) + offset));
        worst = error > worst ? error : worst;
    }
    return worst;
}

void testGround()
{
    const float offset = 0.06f;
    {
        Scene scene;
        addStraight(scene.curve(), 8.0f);
        scene.road->setWidth(4.0f);
        scene.road->setInterval(2.0f);
        CHECK(!scene.road->hasGround());
        const Road3D::GroundFunction plane([](const Math::Vec3&) { return 3.0f; });
        scene.road->setGround(plane);
        CHECK(scene.road->hasGround() && scene.road->isDirty());
        CHECK(near(scene.road->getGroundOffset(), offset) && scene.road->isGroundTilt());
        const SurfaceData& g = scene.build();
        CHECK(g.positions.size() == 10 && indicesValid(g) && allTrianglesFaceUp(g));
        CHECK(worstError(scene, plane, offset) < 1e-5f);
        for (std::size_t i = 0; i < g.normals.size(); ++i) CHECK(isVec3(g.normals[i], 0, 1, 0));
        CHECK(near(g.positions[0].x, -2.0f) && near(g.positions[1].x, 2.0f));
        scene.road->setGroundOffset(0.5f);
        scene.build();
        CHECK(worstError(scene, plane, 0.5f) < 1e-5f);
    }
    {
        Scene scene;
        addStraight(scene.curve(), 8.0f);
        scene.road->setWidth(4.0f);
        scene.road->setInterval(2.0f);
        const Road3D::GroundFunction ramp([](const Math::Vec3& p) { return 2.0f - 0.25f * p.z; });
        scene.road->setGround(ramp);
        const SurfaceData& g = scene.build();
        CHECK(worstError(scene, ramp, offset) < 1e-5f);
        CHECK(near(g.positions[0].y, 2.0f + offset) && near(g.positions[8].y, 4.0f + offset));
        const Math::Vec3 expected = Math::Vec3(0.0f, 1.0f, 0.25f).Normalized();
        for (std::size_t i = 0; i < g.normals.size(); ++i)
            CHECK(isVec3(g.normals[i], 0, expected.y, expected.z, 1e-4f));
        CHECK(allTrianglesFaceUp(g));
    }
    {
        Scene scene;
        addStraight(scene.curve(), 8.0f);
        scene.road->setWidth(4.0f);
        scene.road->setInterval(2.0f);
        const Road3D::GroundFunction side([](const Math::Vec3& p) { return 0.3f * p.x; });
        scene.road->setGround(side);
        const SurfaceData& g = scene.build();
        CHECK(worstError(scene, side, offset) < 1e-5f);
        const Math::Vec3 expected = Math::Vec3(-0.3f, 1.0f, 0.0f).Normalized();
        for (std::size_t i = 0; i < g.normals.size(); ++i)
            CHECK(isVec3(g.normals[i], expected.x, expected.y, expected.z, 1e-4f));
        scene.road->setGroundTilt(false);
        const SurfaceData& level = scene.build();
        CHECK(countBelow(scene, side, offset) == 0);
        for (std::size_t r = 0; r < 5; ++r)
        {
            CHECK(near(level.positions[r * 2].y, level.positions[r * 2 + 1].y));
            CHECK(near(level.positions[r * 2].y, 0.6f + offset));
            CHECK(isVec3(level.normals[r * 2], 0, 1, 0));
        }
    }
    {
        Scene scene;
        addStraightAt(scene.curve(), 1.0f, 8.0f);
        scene.road->setWidth(4.0f);
        scene.road->setInterval(2.0f);
        const Road3D::GroundFunction ridge(
                [](const Math::Vec3& p) { return 3.0f - 0.5f * fabsf(p.x); });
        scene.road->setGround(ridge);
        const SurfaceData& g = scene.build();
        CHECK(worstError(scene, ridge, offset) < 1e-5f);
        CHECK(near(g.positions[0].y, 2.5f + offset) && near(g.positions[1].y, 1.5f + offset));
        CHECK(countBelow(scene, ridge, offset) == 0);
        scene.road->setGroundTilt(false);
        const SurfaceData& level = scene.build();
        CHECK(countBelow(scene, ridge, offset) == 0);
        for (std::size_t i = 0; i < level.positions.size(); ++i)
            CHECK(near(level.positions[i].y, 2.5f + offset));
    }
    {
        Scene scene;
        scene.path->setPosition(Math::Vec3(10.0f, 5.0f, 20.0f));
        addStraight(scene.curve(), 8.0f);
        scene.road->setWidth(4.0f);
        scene.road->setInterval(2.0f);
        const Road3D::GroundFunction slope(
                [](const Math::Vec3& p) { return 0.1f * p.x + 0.05f * p.z; });
        scene.road->setGround(slope);
        scene.build();
        CHECK(worstError(scene, slope, offset) < 1e-4f);
        CHECK(allTrianglesFaceUp(scene.road->getGeometry()));
    }
}

void testGroundRebuild()
{
    Scene scene;
    addStraight(scene.curve(), 8.0f);
    scene.road->setWidth(4.0f);
    scene.road->setInterval(2.0f);
    const Road3D::GroundFunction plane([](const Math::Vec3&) { return 3.0f; });
    scene.road->onGroundChanged(Math::Box(Math::Vec3(-1, 0, -1), Math::Vec3(1, 1, 1)));
    CHECK(scene.road->flush());
    CHECK(!scene.road->flush());
    const int base = scene.road->getBuildCount();
    scene.road->onGroundChanged(Math::Box(Math::Vec3(-1, 0, -1), Math::Vec3(1, 1, 1)));
    CHECK(!scene.road->isDirty());

    scene.road->setGround(plane);
    CHECK(scene.road->flush() && scene.road->getBuildCount() == base + 1);
    scene.road->onGroundChanged(Math::Box(Math::Vec3(50, 0, 50), Math::Vec3(60, 9, 60)));
    scene.road->onGroundChanged(Math::Box(Math::Vec3(2.5f, 0, -3), Math::Vec3(9, 9, 1)));
    scene.road->onGroundChanged(Math::Box(Math::Vec3(-1, 0, 1.5f), Math::Vec3(1, 9, 9)));
    scene.road->onGroundChanged(Math::Box(Math::Vec3(-1, 0, -20), Math::Vec3(1, 9, -8.5f)));
    CHECK(!scene.road->flush() && scene.road->getBuildCount() == base + 1);
    scene.road->onGroundChanged(Math::Box(Math::Vec3(1.5f, 0, -3), Math::Vec3(9, 9, -2)));
    CHECK(scene.road->flush() && scene.road->getBuildCount() == base + 2);
    scene.road->onGroundChanged(Math::Box(Math::Vec3(-9, 0, -3), Math::Vec3(-1.9f, 9, -2)));
    CHECK(scene.road->flush() && scene.road->getBuildCount() == base + 3);
    scene.road->onGroundChanged(Math::Box(Math::Vec3(-1, 0, -0.5f), Math::Vec3(1, 9, 9)));
    CHECK(scene.road->flush() && scene.road->getBuildCount() == base + 4);

    scene.path->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
    scene.path->setPosition(Math::Vec3(0.0f, 2.0f, 0.0f));
    scene.road->setGroundTilt(true);
    CHECK(!scene.road->isDirty());
    scene.road->setGroundTilt(false);
    CHECK(scene.road->isDirty());
    scene.road->flush();

    scene.road->clearGround();
    CHECK(!scene.road->hasGround() && scene.road->flush());
    scene.road->onGroundChanged(Math::Box(Math::Vec3(-1, 0, -1), Math::Vec3(1, 1, 1)));
    CHECK(!scene.road->isDirty());
    CHECK(near(scene.road->getGeometry().positions[0].y, 0.0f));
}

void testFootprint()
{
    {
        Scene scene;
        addStraight(scene.curve(), 8.0f);
        scene.road->setWidth(4.0f);
        TerrainPath path;
        CHECK(scene.road->buildFootprint(path, 2.0f));
        CHECK(path.points.size() == 5 && !path.closed);
        for (std::size_t i = 0; i < 5; ++i)
        {
            CHECK(isVec3(path.points[i], 0.0f, 0.0f, -2.0f * static_cast<float>(i)));
            CHECK(near(path.halfWidths[i], 2.0f));
        }
        scene.road->setPointWidth(0, 2.0f);
        scene.road->setPointWidth(1, 6.0f);
        CHECK(scene.road->buildFootprint(path, 4.0f));
        CHECK(path.points.size() == 3);
        CHECK(near(path.halfWidths[0], 1.0f) && near(path.halfWidths[1], 2.0f) &&
                near(path.halfWidths[2], 3.0f));
        scene.road->clearPointWidths();
        ct::Vector<Math::Vec2> profile;
        profile.push_back(Math::Vec2(-3.0f, 0.0f));
        profile.push_back(Math::Vec2(-2.0f, 0.2f));
        profile.push_back(Math::Vec2(2.0f, 0.2f));
        profile.push_back(Math::Vec2(3.0f, 0.0f));
        scene.road->setProfile(profile);
        CHECK(scene.road->buildFootprint(path, 4.0f));
        CHECK(near(path.halfWidths[1], 3.0f));
        CHECK(!scene.road->buildFootprint(path, 0.0f));
    }
    {
        Scene scene;
        addCircle(scene.curve(), 10.0f);
        scene.curve().setClosed(true);
        scene.road->setWidth(6.0f);
        TerrainPath path;
        CHECK(scene.road->buildFootprint(path, 5.0f));
        CHECK(path.closed && path.points.size() >= 12 && path.points.size() <= 14);
        for (std::size_t i = 0; i < path.points.size(); ++i)
        {
            CHECK(near(radiusFrom(Math::Vec3(0, 0, 0), path.points[i]), 10.0f, 0.06f));
            CHECK(near(path.halfWidths[i], 3.0f));
        }
        CHECK(path.getSegmentCount() == path.points.size());
    }
    {
        Scene scene;
        TerrainPath path;
        CHECK(!scene.road->buildFootprint(path, 2.0f) && path.points.empty());
    }
}

} // namespace

int main()
{
    testStraight();
    testIntervals();
    testPointWidths();
    testProfile();
    testBend();
    testClosedLoop();
    testTilt();
    testDirtyTracking();
    testDegenerate();
    testCollision();
    testGround();
    testGroundRebuild();
    testFootprint();

    if (failures) printf("%d checks failed\n", failures);
    else
        printf("ok\n");
    return failures ? 1 : 0;
}
