#include "Check.h"

#include "prisma3d/scene/Curve3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const float kPi = 3.14159265358979323846f;
const float kSqrtHalf = 0.70710678118f;
const double kStep = 0.25;

bool near(float a, float b, float tolerance = kTolerance)
{
    return fabsf(a - b) <= tolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z, float tolerance = kTolerance)
{
    return near(a.x, x, tolerance) && near(a.y, y, tolerance) && near(a.z, z, tolerance);
}

void addLine(Curve3D& curve, const Math::Vec3& from, const Math::Vec3& to)
{
    const Math::Vec3 third = (to - from) / 3.0f;
    curve.addPoint(from, Math::Vec3(0.0f, 0.0f, 0.0f), third);
    curve.addPoint(to, -third, Math::Vec3(0.0f, 0.0f, 0.0f));
}

void addSquare(Curve3D& curve)
{
    curve.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f));
    curve.addPoint(Math::Vec3(4.0f, 0.0f, 0.0f));
    curve.addPoint(Math::Vec3(4.0f, 0.0f, 4.0f));
    curve.addPoint(Math::Vec3(0.0f, 0.0f, 4.0f));
}

void addCircle(Curve3D& curve, float height)
{
    const float k = 4.0f * 0.5522847498f;
    curve.addPoint(Math::Vec3(4.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -k),
            Math::Vec3(0.0f, 0.0f, k));
    curve.addPoint(Math::Vec3(0.0f, height, 4.0f), Math::Vec3(k, 0.0f, 0.0f),
            Math::Vec3(-k, 0.0f, 0.0f));
    curve.addPoint(Math::Vec3(-4.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, k),
            Math::Vec3(0.0f, 0.0f, -k));
    curve.addPoint(Math::Vec3(0.0f, -height, -4.0f), Math::Vec3(-k, 0.0f, 0.0f),
            Math::Vec3(k, 0.0f, 0.0f));
}

struct Scene
{
    SceneTree tree;
    Path3D* path;
    PathFollow3D* follower;

    Scene()
            : tree(),
              path(nullptr),
              follower(nullptr)
    {
        tree.initialize();
        path = tree.getRoot()->createChild<Path3D>();
        addLine(path->getCurve(), Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(10.0f, 0.0f, 0.0f));
        follower = path->createChild<PathFollow3D>();
    }
};

int gChanged = 0;

void countChanged()
{
    ++gChanged;
}

void testPoints()
{
    Curve3D curve;
    CHECK(curve.getPointCount() == 0);
    CHECK(!curve.isClosed());
    CHECK(near(curve.getBakeInterval(), 0.2f));
    CHECK(curve.isUpVectorEnabled());
    CHECK(near(curve.getBakedLength(), 0.0f));
    CHECK(curve.getBakedPoints().empty());

    curve.addPoint(Math::Vec3(1.0f, 2.0f, 3.0f), Math::Vec3(0.5f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.5f, 0.0f));
    curve.addPoint(Math::Vec3(4.0f, 5.0f, 6.0f));
    curve.addPoint(Math::Vec3(7.0f, 8.0f, 9.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f), 1);
    CHECK(curve.getPointCount() == 3);
    CHECK(nearVec3(curve.getPointPosition(0), 1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(curve.getPointPosition(1), 7.0f, 8.0f, 9.0f));
    CHECK(nearVec3(curve.getPointPosition(2), 4.0f, 5.0f, 6.0f));
    CHECK(nearVec3(curve.getPointIn(0), 0.5f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.getPointOut(0), 0.0f, 0.5f, 0.0f));

    curve.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f), 100);
    CHECK(curve.getPointCount() == 4);
    CHECK(nearVec3(curve.getPointPosition(3), 0.0f, 0.0f, 0.0f));

    curve.setPointPosition(2, Math::Vec3(-1.0f, -2.0f, -3.0f));
    curve.setPointIn(2, Math::Vec3(1.0f, 1.0f, 1.0f));
    curve.setPointOut(2, Math::Vec3(2.0f, 2.0f, 2.0f));
    curve.setPointTilt(2, 0.75f);
    CHECK(nearVec3(curve.getPointPosition(2), -1.0f, -2.0f, -3.0f));
    CHECK(nearVec3(curve.getPointIn(2), 1.0f, 1.0f, 1.0f));
    CHECK(nearVec3(curve.getPointOut(2), 2.0f, 2.0f, 2.0f));
    CHECK(near(curve.getPointTilt(2), 0.75f));

    const ct::Vector<Math::Vec3> data = curve.getPoints();
    CHECK(data.size() == 12);
    CHECK(nearVec3(data[0], 0.5f, 0.0f, 0.0f));
    CHECK(nearVec3(data[1], 0.0f, 0.5f, 0.0f));
    CHECK(nearVec3(data[2], 1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(data[6], 1.0f, 1.0f, 1.0f));
    CHECK(nearVec3(data[7], 2.0f, 2.0f, 2.0f));
    CHECK(nearVec3(data[8], -1.0f, -2.0f, -3.0f));

    curve.removePoint(0);
    CHECK(curve.getPointCount() == 3);
    CHECK(nearVec3(curve.getPointPosition(0), 7.0f, 8.0f, 9.0f));

    curve.setPointCount(5);
    CHECK(curve.getPointCount() == 5);
    CHECK(nearVec3(curve.getPointPosition(4), 0.0f, 0.0f, 0.0f));
    curve.setPointCount(2);
    CHECK(curve.getPointCount() == 2);

    curve.clearPoints();
    CHECK(curve.getPointCount() == 0);
    CHECK(curve.getBakedPoints().empty());
    CHECK(near(curve.getBakedLength(), 0.0f));
}

void testChanged()
{
    Curve3D curve;
    gChanged = 0;
    curve.changed = countChanged;

    curve.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(gChanged == 1);
    curve.addPoint(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(gChanged == 2);
    curve.setPointPosition(1, Math::Vec3(2.0f, 0.0f, 0.0f));
    CHECK(gChanged == 3);
    curve.setPointTilt(1, 1.0f);
    CHECK(gChanged == 4);
    curve.setPointIn(1, Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(gChanged == 5);
    curve.setPointOut(1, Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(gChanged == 6);
    curve.setBakeInterval(0.5f);
    CHECK(gChanged == 7);
    curve.setUpVectorEnabled(false);
    CHECK(gChanged == 8);
    curve.setClosed(true);
    CHECK(gChanged == 9);
    curve.setClosed(true);
    CHECK(gChanged == 9);

    curve.getBakedLength();
    curve.getBakedLength();
    curve.sampleBaked(0.5f);
    CHECK(gChanged == 9);

    curve.removePoint(1);
    CHECK(gChanged == 11);
    CHECK(!curve.isClosed());

    curve.clearPoints();
    CHECK(gChanged == 12);
    curve.clearPoints();
    CHECK(gChanged == 12);

    Curve3D other;
    addLine(other, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(3.0f, 0.0f, 0.0f));
    other.setBakeInterval(0.4f);
    other.setUpVectorEnabled(false);
    curve = other;
    CHECK(gChanged == 13);
    CHECK(curve.getPointCount() == 2);
    CHECK(near(curve.getBakeInterval(), 0.4f));
    CHECK(!curve.isUpVectorEnabled());
    CHECK(near(curve.getBakedLength(), 3.0f));

    Curve3D copy(curve);
    CHECK(copy.getPointCount() == 2);
    CHECK(near(copy.getBakedLength(), 3.0f));
    copy.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(gChanged == 13);
    CHECK(curve.getPointCount() == 2);
}

void testSample()
{
    Curve3D curve;
    curve.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 6.0f, 0.0f));
    curve.addPoint(Math::Vec3(12.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 6.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f));

    CHECK(nearVec3(curve.sample(0, 0.0f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sample(0, 1.0f), 12.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sample(0, 0.5f), 6.0f, 4.5f, 0.0f));
    CHECK(nearVec3(curve.sample(0, 0.25f), 1.875f, 3.375f, 0.0f));
    CHECK(nearVec3(curve.sample(1, 0.5f), 12.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sample(5, 0.5f), 12.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sample(-1, 0.5f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.samplef(0.5f), 6.0f, 4.5f, 0.0f));
    CHECK(nearVec3(curve.samplef(0.25f), 1.875f, 3.375f, 0.0f));
    CHECK(nearVec3(curve.samplef(-3.0f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.samplef(7.0f), 12.0f, 0.0f, 0.0f));

    curve.setClosed(true);
    CHECK(nearVec3(curve.sample(1, 0.0f), 12.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sample(1, 1.0f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sample(1, 0.5f), 6.0f, 0.0f, 0.0f));
}

void testBakedLine()
{
    Curve3D curve;
    addLine(curve, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(10.0f, 0.0f, 0.0f));

    CHECK(near(curve.getBakedLength(), 10.0f));
    const ct::Vector<Math::Vec3>& points = curve.getBakedPoints();
    CHECK(points.size() == 65);
    CHECK(nearVec3(points[0], 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(points[1], 0.15625f, 0.0f, 0.0f));
    CHECK(nearVec3(points[32], 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(points[64], 10.0f, 0.0f, 0.0f));

    const ct::Vector<float>& dists = curve.getBakedDistCache();
    CHECK(dists.size() == 65);
    CHECK(near(dists[0], 0.0f));
    CHECK(near(dists[16], 2.5f));
    CHECK(near(dists[64], 10.0f));

    CHECK(curve.getBakedTilts().size() == 65);
    CHECK(curve.getBakedUpVectors().size() == 65);

    CHECK(nearVec3(curve.sampleBaked(0.0f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(2.5f), 2.5f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(7.3f), 7.3f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(0.078125f), 0.078125f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(10.0f), 10.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(-4.0f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(40.0f), 10.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(5.078125f, true), 5.078125f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(7.3f, true), 7.3f, 0.0f, 0.0f));

    Curve3D smooth;
    smooth.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f));
    smooth.addPoint(Math::Vec3(10.0f, 0.0f, 0.0f));
    CHECK(near(smooth.getBakedLength(), 10.0f));
    CHECK(nearVec3(smooth.sampleBaked(5.0f), 5.0f, 0.0f, 0.0f));
    CHECK(smooth.getBakedPoints().size() > 3);
    const ct::Vector<float>& smoothDists = smooth.getBakedDistCache();
    for (std::size_t i = 1; i < smoothDists.size(); ++i)
        CHECK(smoothDists[i] >= smoothDists[i - 1]);
}

void testBakedBezier()
{
    Curve3D curve;
    curve.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 6.0f, 0.0f));
    curve.addPoint(Math::Vec3(12.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 6.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f));

    float reference = 0.0f;
    Math::Vec3 previous = curve.sample(0, 0.0f);
    for (int i = 1; i <= 20000; ++i)
    {
        const Math::Vec3 current = curve.sample(0, static_cast<float>(i) / 20000.0f);
        reference += (current - previous).Length();
        previous = current;
    }

    const float length = curve.getBakedLength();
    CHECK(length > 12.0f && length < 24.0f);
    CHECK(near(length, reference, 0.01f));
    CHECK(nearVec3(curve.sampleBaked(length * 0.5f), 6.0f, 4.5f, 0.0f, 1e-3f));
    CHECK(nearVec3(curve.sampleBaked(length * 0.5f, true), 6.0f, 4.5f, 0.0f, 1e-3f));
    CHECK(nearVec3(curve.sampleBaked(0.0f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(length), 12.0f, 0.0f, 0.0f));

    const ct::Vector<Math::Vec3>& points = curve.getBakedPoints();
    CHECK(points.size() > 60);
    const ct::Vector<float>& dists = curve.getBakedDistCache();
    for (std::size_t i = 1; i < dists.size(); ++i)
    {
        CHECK(dists[i] > dists[i - 1]);
        CHECK((points[i] - points[i - 1]).Length() < 0.2f + 1e-4f);
    }
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        CHECK(points[i].x >= -1e-4f && points[i].x <= 12.0001f);
        CHECK(points[i].y >= -1e-4f && points[i].y <= 4.5001f);
        CHECK(near(points[i].z, 0.0f));
    }

    const ct::Vector<Math::Vec3> coarse = curve.tessellate(5, 4.0f);
    CHECK(coarse.size() > 2);
    CHECK(nearVec3(coarse[0], 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(coarse[coarse.size() - 1], 12.0f, 0.0f, 0.0f));
    bool found = false;
    for (std::size_t i = 0; i < coarse.size(); ++i)
        if (nearVec3(coarse[i], 6.0f, 4.5f, 0.0f)) found = true;
    CHECK(found);
    for (std::size_t i = 1; i < coarse.size(); ++i) CHECK(coarse[i].x >= coarse[i - 1].x);

    const ct::Vector<Math::Vec3> fine = curve.tessellate(5, 0.5f);
    CHECK(fine.size() > coarse.size());
}

void testTessellate()
{
    Curve3D line;
    addLine(line, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(10.0f, 0.0f, 0.0f));

    const ct::Vector<Math::Vec3> flat = line.tessellate();
    CHECK(flat.size() == 2);
    CHECK(nearVec3(flat[0], 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(flat[1], 10.0f, 0.0f, 0.0f));

    const ct::Vector<Math::Vec3> even = line.tessellateEvenLength(5, 2.5f);
    CHECK(even.size() == 5);
    CHECK(nearVec3(even[0], 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(even[1], 2.5f, 0.0f, 0.0f));
    CHECK(nearVec3(even[2], 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(even[3], 7.5f, 0.0f, 0.0f));
    CHECK(nearVec3(even[4], 10.0f, 0.0f, 0.0f));

    const ct::Vector<Math::Vec3> limited = line.tessellateEvenLength(1, 0.1f);
    CHECK(limited.size() == 3);
    CHECK(nearVec3(limited[1], 5.0f, 0.0f, 0.0f));

    Curve3D closed;
    addSquare(closed);
    closed.setClosed(true);
    const ct::Vector<Math::Vec3> tess = closed.tessellate();
    CHECK(tess.size() >= 5);
    CHECK(nearVec3(tess[0], 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(tess[tess.size() - 1], 0.0f, 0.0f, 0.0f));

    Curve3D empty;
    CHECK(empty.tessellate().empty());
}

void testClosed()
{
    Curve3D curve;
    addSquare(curve);
    CHECK(near(curve.getBakedLength(), 12.0f));

    curve.setClosed(true);
    CHECK(curve.isClosed());
    CHECK(near(curve.getBakedLength(), 16.0f));
    const ct::Vector<Math::Vec3>& points = curve.getBakedPoints();
    CHECK(nearVec3(points[0], 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(points[points.size() - 1], 0.0f, 0.0f, 0.0f));

    CHECK(nearVec3(curve.sampleBaked(0.0f), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(2.0f), 2.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(4.0f), 4.0f, 0.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBaked(6.0f), 4.0f, 0.0f, 2.0f));
    CHECK(nearVec3(curve.sampleBaked(10.0f), 2.0f, 0.0f, 4.0f));
    CHECK(nearVec3(curve.sampleBaked(14.0f), 0.0f, 0.0f, 2.0f));
    CHECK(nearVec3(curve.sampleBaked(16.0f), 0.0f, 0.0f, 0.0f));

    CHECK(nearVec3(curve.getClosestPoint(Math::Vec3(5.0f, 0.0f, 2.0f)), 4.0f, 0.0f, 2.0f));
    CHECK(near(curve.getClosestOffset(Math::Vec3(5.0f, 0.0f, 2.0f)), 6.0f));
    CHECK(nearVec3(curve.getClosestPoint(Math::Vec3(-3.0f, 0.0f, 3.0f)), 0.0f, 0.0f, 3.0f));
    CHECK(near(curve.getClosestOffset(Math::Vec3(-3.0f, 0.0f, 3.0f)), 13.0f));

    const ct::Vector<Math::Vec3>& ups = curve.getBakedUpVectors();
    CHECK(ups.size() == points.size());
    for (std::size_t i = 0; i < ups.size(); ++i) CHECK(nearVec3(ups[i], 0.0f, 1.0f, 0.0f, 1e-3f));

    curve.removePoint(3);
    curve.removePoint(2);
    CHECK(curve.isClosed());
    curve.removePoint(1);
    CHECK(!curve.isClosed());
    CHECK(curve.getPointCount() == 1);

    Curve3D circle;
    addCircle(circle, 0.0f);
    circle.setClosed(true);
    const float length = circle.getBakedLength();
    CHECK(near(length, 2.0f * kPi * 4.0f, 0.02f));
    CHECK(nearVec3(circle.sampleBaked(length * 0.25f), 0.0f, 0.0f, 4.0f, 2e-3f));
    CHECK(nearVec3(circle.sampleBaked(length * 0.5f), -4.0f, 0.0f, 0.0f, 2e-3f));
    CHECK(near(circle.getClosestOffset(Math::Vec3(0.0f, 0.0f, 8.0f)), length * 0.25f, 2e-3f));
    CHECK(nearVec3(circle.getClosestPoint(Math::Vec3(0.0f, 0.0f, 8.0f)), 0.0f, 0.0f, 4.0f, 2e-3f));
    const ct::Vector<Math::Vec3>& circleUps = circle.getBakedUpVectors();
    for (std::size_t i = 0; i < circleUps.size(); ++i)
        CHECK(nearVec3(circleUps[i], 0.0f, 1.0f, 0.0f, 1e-3f));

    Curve3D wavy;
    addCircle(wavy, 1.0f);
    wavy.setClosed(true);
    const ct::Vector<Math::Vec3>& wavyUps = wavy.getBakedUpVectors();
    CHECK(wavyUps.size() > 100);
    for (std::size_t i = 0; i < wavyUps.size(); ++i) CHECK(near(wavyUps[i].Length(), 1.0f, 1e-3f));
    CHECK(wavyUps[0].Dot(wavyUps[wavyUps.size() - 1]) > 0.9999f);
}

void testTiltAndUp()
{
    Curve3D curve;
    addLine(curve, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(10.0f, 0.0f, 0.0f));

    const ct::Vector<Math::Vec3>& ups = curve.getBakedUpVectors();
    for (std::size_t i = 0; i < ups.size(); ++i) CHECK(nearVec3(ups[i], 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBakedUpVector(3.3f), 0.0f, 1.0f, 0.0f));

    const Math::Mat4 frame = curve.sampleBakedWithRotation(2.5f);
    CHECK(nearVec3(frame.GetTranslation(), 2.5f, 0.0f, 0.0f));
    CHECK(nearVec3(frame.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(frame.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(frame.col2.xyz(), -1.0f, 0.0f, 0.0f));

    curve.setPointTilt(1, kPi * 0.5f);
    CHECK(near(curve.getPointTilt(1), kPi * 0.5f));
    CHECK(near(curve.sampleBakedTilt(0.0f), 0.0f));
    CHECK(near(curve.sampleBakedTilt(2.5f), kPi / 8.0f));
    CHECK(near(curve.sampleBakedTilt(5.0f), kPi / 4.0f));
    CHECK(near(curve.sampleBakedTilt(10.0f), kPi / 2.0f));
    CHECK(near(curve.getBakedTilts()[64], kPi / 2.0f));

    CHECK(nearVec3(curve.sampleBakedUpVector(5.0f, false), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(curve.sampleBakedUpVector(5.0f, true), 0.0f, kSqrtHalf, kSqrtHalf));
    CHECK(nearVec3(curve.sampleBakedUpVector(10.0f, true), 0.0f, 0.0f, 1.0f));

    const Math::Mat4 tilted = curve.sampleBakedWithRotation(5.0f, false, true);
    CHECK(nearVec3(tilted.GetTranslation(), 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(tilted.col0.xyz(), 0.0f, -kSqrtHalf, kSqrtHalf));
    CHECK(nearVec3(tilted.col1.xyz(), 0.0f, kSqrtHalf, kSqrtHalf));
    CHECK(nearVec3(tilted.col2.xyz(), -1.0f, 0.0f, 0.0f));

    const Math::Mat4 untilted = curve.sampleBakedWithRotation(5.0f, false, false);
    CHECK(nearVec3(untilted.col1.xyz(), 0.0f, 1.0f, 0.0f));

    Curve3D diagonal;
    addLine(diagonal, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(10.0f, 10.0f, 0.0f));
    CHECK(near(diagonal.getBakedLength(), 10.0f * 1.41421356f));
    const Math::Mat4 slope = diagonal.sampleBakedWithRotation(diagonal.getBakedLength() * 0.5f);
    CHECK(nearVec3(slope.GetTranslation(), 5.0f, 5.0f, 0.0f));
    CHECK(nearVec3(slope.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(slope.col1.xyz(), -kSqrtHalf, kSqrtHalf, 0.0f));
    CHECK(nearVec3(slope.col2.xyz(), -kSqrtHalf, -kSqrtHalf, 0.0f));
    const ct::Vector<Math::Vec3>& slopeUps = diagonal.getBakedUpVectors();
    for (std::size_t i = 0; i < slopeUps.size(); ++i)
        CHECK(nearVec3(slopeUps[i], -kSqrtHalf, kSqrtHalf, 0.0f));

    Curve3D vertical;
    addLine(vertical, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 5.0f, 0.0f));
    CHECK(near(vertical.getBakedLength(), 5.0f));
    const Math::Mat4 rise = vertical.sampleBakedWithRotation(2.5f);
    CHECK(nearVec3(rise.col2.xyz(), 0.0f, -1.0f, 0.0f));
    CHECK(nearVec3(vertical.sampleBakedUpVector(1.0f), 1.0f, 0.0f, 0.0f));

    curve.setUpVectorEnabled(false);
    CHECK(!curve.isUpVectorEnabled());
    CHECK(curve.getBakedUpVectors().empty());
    const Math::Mat4 noUp = curve.sampleBakedWithRotation(2.5f);
    CHECK(nearVec3(noUp.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(noUp.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(noUp.col2.xyz(), -1.0f, 0.0f, 0.0f));
    CHECK(near(curve.sampleBakedTilt(5.0f), kPi / 4.0f));
}

void testSinglePoint()
{
    Curve3D curve;
    curve.addPoint(Math::Vec3(3.0f, 4.0f, 5.0f));
    curve.setPointTilt(0, 0.5f);
    CHECK(curve.getBakedPoints().size() == 1);
    CHECK(near(curve.getBakedLength(), 0.0f));
    CHECK(nearVec3(curve.sampleBaked(1.0f), 3.0f, 4.0f, 5.0f));
    CHECK(near(curve.sampleBakedTilt(1.0f), 0.5f));
    CHECK(nearVec3(curve.sampleBakedUpVector(1.0f), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(curve.getClosestPoint(Math::Vec3(0.0f, 0.0f, 0.0f)), 3.0f, 4.0f, 5.0f));
    CHECK(near(curve.getClosestOffset(Math::Vec3(0.0f, 0.0f, 0.0f)), 0.0f));
}

void testClosest()
{
    Curve3D curve;
    addLine(curve, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(10.0f, 0.0f, 0.0f));

    CHECK(nearVec3(curve.getClosestPoint(Math::Vec3(3.0f, 5.0f, 0.0f)), 3.0f, 0.0f, 0.0f));
    CHECK(near(curve.getClosestOffset(Math::Vec3(3.0f, 5.0f, 0.0f)), 3.0f));
    CHECK(nearVec3(curve.getClosestPoint(Math::Vec3(7.7f, 0.0f, -2.0f)), 7.7f, 0.0f, 0.0f));
    CHECK(near(curve.getClosestOffset(Math::Vec3(7.7f, 0.0f, -2.0f)), 7.7f));
    CHECK(nearVec3(curve.getClosestPoint(Math::Vec3(-5.0f, 2.0f, 0.0f)), 0.0f, 0.0f, 0.0f));
    CHECK(near(curve.getClosestOffset(Math::Vec3(-5.0f, 2.0f, 0.0f)), 0.0f));
    CHECK(nearVec3(curve.getClosestPoint(Math::Vec3(20.0f, 0.0f, 0.0f)), 10.0f, 0.0f, 0.0f));
    CHECK(near(curve.getClosestOffset(Math::Vec3(20.0f, 0.0f, 0.0f)), 10.0f));

    Curve3D empty;
    CHECK(near(empty.getClosestOffset(Math::Vec3(1.0f, 1.0f, 1.0f)), 0.0f));
}

void testFollowerPosition()
{
    Scene scene;
    PathFollow3D* follower = scene.follower;

    CHECK(follower->castTo<PathFollow3D>() == follower);
    CHECK(scene.path->castTo<Path3D>() == scene.path);
    CHECK(follower->castTo<Path3D>() == nullptr);
    CHECK(follower->getRotationMode() == PathFollow3D::RotationMode::XYZ);
    CHECK(follower->hasLoop());
    CHECK(follower->isTiltEnabled());
    CHECK(follower->isCubicInterpolationEnabled());
    CHECK(!follower->isUsingModelFront());
    CHECK(near(follower->getProgress(), 0.0f));
    CHECK(near(follower->getHOffset(), 0.0f));
    CHECK(near(follower->getVOffset(), 0.0f));

    CHECK(nearVec3(follower->getPosition(), 0.0f, 0.0f, 0.0f));

    follower->setProgress(2.5f);
    CHECK(near(follower->getProgress(), 2.5f));
    CHECK(nearVec3(follower->getPosition(), 2.5f, 0.0f, 0.0f));
    follower->setProgress(5.078125f);
    CHECK(nearVec3(follower->getPosition(), 5.078125f, 0.0f, 0.0f));
    follower->setProgress(7.5f);
    CHECK(nearVec3(follower->getPosition(), 7.5f, 0.0f, 0.0f));
    follower->setProgress(10.0f);
    CHECK(nearVec3(follower->getPosition(), 10.0f, 0.0f, 0.0f));

    follower->setProgress(5.0f);
    const Math::Mat4 transform = follower->getTransform();
    CHECK(nearVec3(transform.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(transform.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(transform.col2.xyz(), -1.0f, 0.0f, 0.0f));

    scene.path->setPosition(Math::Vec3(0.0f, 10.0f, 0.0f));
    CHECK(nearVec3(follower->getGlobalPosition(), 5.0f, 10.0f, 0.0f));

    follower->setCubicInterpolationEnabled(false);
    CHECK(!follower->isCubicInterpolationEnabled());
    follower->setProgress(1.0f);
    CHECK(nearVec3(follower->getPosition(), 1.0f, 0.0f, 0.0f));
    follower->setCubicInterpolationEnabled(true);
    follower->setProgress(1.1f);
    follower->setProgress(1.0f);
    CHECK(nearVec3(follower->getPosition(), 1.0f, 0.0f, 0.0f, 1e-3f));

    PathFollow3D* second = scene.path->createChild<PathFollow3D>();
    second->setProgress(8.0f);
    CHECK(nearVec3(second->getPosition(), 8.0f, 0.0f, 0.0f));
    CHECK(nearVec3(follower->getPosition(), 1.0f, 0.0f, 0.0f, 1e-3f));
}

void testFollowerLoop()
{
    Scene scene;
    PathFollow3D* follower = scene.follower;

    follower->setProgress(12.0f);
    CHECK(near(follower->getProgress(), 2.0f));
    follower->setProgress(-3.0f);
    CHECK(near(follower->getProgress(), 7.0f));
    follower->setProgress(10.0f);
    CHECK(near(follower->getProgress(), 10.0f));
    follower->setProgress(20.0f);
    CHECK(near(follower->getProgress(), 10.0f));
    CHECK(nearVec3(follower->getPosition(), 10.0f, 0.0f, 0.0f));
    follower->setProgress(0.0f);
    CHECK(near(follower->getProgress(), 0.0f));

    follower->setLoop(false);
    CHECK(!follower->hasLoop());
    follower->setProgress(12.0f);
    CHECK(near(follower->getProgress(), 10.0f));
    CHECK(nearVec3(follower->getPosition(), 10.0f, 0.0f, 0.0f));
    follower->setProgress(-3.0f);
    CHECK(near(follower->getProgress(), 0.0f));
    CHECK(nearVec3(follower->getPosition(), 0.0f, 0.0f, 0.0f));
    follower->setProgress(4.0f);
    CHECK(near(follower->getProgress(), 4.0f));

    follower->setLoop(true);
    follower->setProgress(NAN);
    CHECK(near(follower->getProgress(), 4.0f));
    follower->setProgress(INFINITY);
    CHECK(near(follower->getProgress(), 4.0f));

    Node3D* holder = scene.tree.getRoot()->createChild<Node3D>();
    PathFollow3D* loose = holder->createChild<PathFollow3D>();
    loose->setProgress(12.0f);
    CHECK(near(loose->getProgress(), 12.0f));
    CHECK(nearVec3(loose->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(near(loose->getProgressRatio(), 0.0f));
    loose->setProgressRatio(0.5f);
    CHECK(near(loose->getProgress(), 12.0f));
}

void testFollowerRatio()
{
    Scene scene;
    PathFollow3D* follower = scene.follower;

    follower->setProgressRatio(0.25f);
    CHECK(near(follower->getProgress(), 2.5f));
    CHECK(near(follower->getProgressRatio(), 0.25f));
    CHECK(nearVec3(follower->getPosition(), 2.5f, 0.0f, 0.0f));

    follower->setProgressRatio(1.0f);
    CHECK(near(follower->getProgress(), 10.0f));
    CHECK(near(follower->getProgressRatio(), 1.0f));

    follower->setProgressRatio(1.5f);
    CHECK(near(follower->getProgress(), 5.0f));
    CHECK(near(follower->getProgressRatio(), 0.5f));

    follower->setLoop(false);
    follower->setProgressRatio(1.5f);
    CHECK(near(follower->getProgress(), 10.0f));
    CHECK(near(follower->getProgressRatio(), 1.0f));

    scene.path->getCurve().clearPoints();
    CHECK(near(follower->getProgressRatio(), 0.0f));
    follower->setProgressRatio(0.5f);
    CHECK(near(follower->getProgress(), 10.0f));
}

void testFollowerOffsets()
{
    Scene scene;
    PathFollow3D* follower = scene.follower;

    follower->setProgress(5.0f);
    follower->setHOffset(2.0f);
    CHECK(near(follower->getHOffset(), 2.0f));
    CHECK(nearVec3(follower->getPosition(), 5.0f, 0.0f, 2.0f));
    follower->setVOffset(3.0f);
    CHECK(near(follower->getVOffset(), 3.0f));
    CHECK(nearVec3(follower->getPosition(), 5.0f, 3.0f, 2.0f));

    follower->setRotationMode(PathFollow3D::RotationMode::None);
    CHECK(nearVec3(follower->getPosition(), 7.0f, 3.0f, 0.0f));
    const Math::Mat4 none = follower->getTransform();
    CHECK(nearVec3(none.col0.xyz(), 1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(none.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(none.col2.xyz(), 0.0f, 0.0f, 1.0f));

    follower->setRotationMode(PathFollow3D::RotationMode::XYZ);
    follower->setHOffset(0.0f);
    follower->setVOffset(0.0f);
    CHECK(nearVec3(follower->getPosition(), 5.0f, 0.0f, 0.0f));

    follower->setScale(Math::Vec3(2.0f, 3.0f, 4.0f));
    follower->setProgress(6.0f);
    CHECK(nearVec3(follower->getScale(), 2.0f, 3.0f, 4.0f, 1e-3f));
    CHECK(nearVec3(follower->getPosition(), 6.0f, 0.0f, 0.0f));
    follower->setHOffset(1.0f);
    CHECK(nearVec3(follower->getPosition(), 6.0f, 0.0f, 1.0f));
    CHECK(nearVec3(follower->getScale(), 2.0f, 3.0f, 4.0f, 1e-3f));
}

void testRotationModes()
{
    const float yaw = 0.5f;
    const float pitch = 0.3f;
    const float roll = 0.2f;
    const Math::Mat3 input =
            Math::Mat3::RotationY(yaw) * Math::Mat3::RotationX(pitch) * Math::Mat3::RotationZ(roll);
    const Math::Mat4 transform(input, Math::Vec3(1.0f, 2.0f, 3.0f));

    const float sy = sinf(yaw), cy = cosf(yaw);
    const float sx = sinf(pitch), cx = cosf(pitch);

    Math::Mat4 result = PathFollow3D::correctPosture(transform, PathFollow3D::RotationMode::None);
    CHECK(nearVec3(result.col0.xyz(), 1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(result.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(result.col2.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(result.GetTranslation(), 1.0f, 2.0f, 3.0f));

    result = PathFollow3D::correctPosture(transform, PathFollow3D::RotationMode::Y);
    CHECK(nearVec3(result.col0.xyz(), cy, 0.0f, -sy));
    CHECK(nearVec3(result.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(result.col2.xyz(), sy, 0.0f, cy));
    CHECK(nearVec3(result.GetTranslation(), 1.0f, 2.0f, 3.0f));

    result = PathFollow3D::correctPosture(transform, PathFollow3D::RotationMode::XY);
    CHECK(nearVec3(result.col0.xyz(), cy, 0.0f, -sy));
    CHECK(nearVec3(result.col1.xyz(), sy * sx, cx, cy * sx));
    CHECK(nearVec3(result.col2.xyz(), sy * cx, -sx, cy * cx));

    result = PathFollow3D::correctPosture(transform, PathFollow3D::RotationMode::XYZ);
    CHECK(nearVec3(result.col0.xyz(), input.col0.x, input.col0.y, input.col0.z));
    CHECK(nearVec3(result.col1.xyz(), input.col1.x, input.col1.y, input.col1.z));
    CHECK(nearVec3(result.col2.xyz(), input.col2.x, input.col2.y, input.col2.z));

    result = PathFollow3D::correctPosture(transform, PathFollow3D::RotationMode::Oriented);
    CHECK(nearVec3(result.col2.xyz(), input.col2.x, input.col2.y, input.col2.z));
    CHECK(near(result.col0.y, 0.0f));
    CHECK(near(result.col0.xyz().Length(), 1.0f));
    CHECK(near(result.col1.xyz().Length(), 1.0f));
    CHECK(near(result.col0.xyz().Dot(result.col1.xyz()), 0.0f));
    CHECK(near(result.col0.xyz().Dot(result.col2.xyz()), 0.0f));
    CHECK(result.col1.y > 0.0f);
    const float horizontal = sqrtf(input.col2.x * input.col2.x + input.col2.z * input.col2.z);
    CHECK(nearVec3(result.col0.xyz(), input.col2.z / horizontal, 0.0f, -input.col2.x / horizontal));

    Scene scene;
    PathFollow3D* follower = scene.follower;
    scene.path->getCurve().clearPoints();
    addLine(scene.path->getCurve(), Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(10.0f, 10.0f, 0.0f));
    const float half = scene.path->getCurve().getBakedLength() * 0.5f;
    follower->setProgress(half);

    CHECK(nearVec3(follower->getPosition(), 5.0f, 5.0f, 0.0f));
    Math::Mat4 t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(t.col1.xyz(), -kSqrtHalf, kSqrtHalf, 0.0f));
    CHECK(nearVec3(t.col2.xyz(), -kSqrtHalf, -kSqrtHalf, 0.0f));

    follower->setRotationMode(PathFollow3D::RotationMode::XY);
    t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(t.col2.xyz(), -kSqrtHalf, -kSqrtHalf, 0.0f));

    follower->setRotationMode(PathFollow3D::RotationMode::Y);
    t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(t.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(t.col2.xyz(), -1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(follower->getPosition(), 5.0f, 5.0f, 0.0f));

    follower->setRotationMode(PathFollow3D::RotationMode::Oriented);
    t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(t.col1.xyz(), -kSqrtHalf, kSqrtHalf, 0.0f));
    CHECK(nearVec3(t.col2.xyz(), -kSqrtHalf, -kSqrtHalf, 0.0f));

    follower->setRotationMode(PathFollow3D::RotationMode::None);
    t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(t.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(t.col2.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(follower->getPosition(), 5.0f, 5.0f, 0.0f));

    follower->setRotationMode(PathFollow3D::RotationMode::XYZ);
    follower->setUseModelFront(true);
    CHECK(follower->isUsingModelFront());
    t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 0.0f, 0.0f, -1.0f));
    CHECK(nearVec3(t.col1.xyz(), -kSqrtHalf, kSqrtHalf, 0.0f));
    CHECK(nearVec3(t.col2.xyz(), kSqrtHalf, kSqrtHalf, 0.0f));
    follower->setUseModelFront(false);
}

void testFollowerTilt()
{
    Scene scene;
    PathFollow3D* follower = scene.follower;
    scene.path->getCurve().setPointTilt(1, kPi * 0.5f);
    follower->setProgress(5.0f);

    Math::Mat4 t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 0.0f, -kSqrtHalf, kSqrtHalf));
    CHECK(nearVec3(t.col1.xyz(), 0.0f, kSqrtHalf, kSqrtHalf));
    CHECK(nearVec3(t.col2.xyz(), -1.0f, 0.0f, 0.0f));

    follower->setTiltEnabled(false);
    CHECK(!follower->isTiltEnabled());
    t = follower->getTransform();
    CHECK(nearVec3(t.col0.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(t.col1.xyz(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(t.col2.xyz(), -1.0f, 0.0f, 0.0f));

    follower->setTiltEnabled(true);
    follower->setProgress(10.0f);
    t = follower->getTransform();
    CHECK(nearVec3(t.col1.xyz(), 0.0f, 0.0f, 1.0f));
    CHECK(nearVec3(t.col0.xyz(), 0.0f, -1.0f, 0.0f));
}

void testCurveChange()
{
    Scene scene;
    PathFollow3D* follower = scene.follower;
    gChanged = 0;
    scene.path->curveChanged = countChanged;

    follower->setProgress(5.0f);
    CHECK(nearVec3(follower->getPosition(), 5.0f, 0.0f, 0.0f));

    scene.path->getCurve().setPointPosition(0, Math::Vec3(0.0f, 3.0f, 0.0f));
    CHECK(gChanged == 1);
    scene.path->getCurve().setPointPosition(1, Math::Vec3(10.0f, 3.0f, 0.0f));
    CHECK(gChanged == 2);
    CHECK(nearVec3(follower->getPosition(), 5.0f, 3.0f, 0.0f));

    scene.path->getCurve().setPointPosition(0, Math::Vec3(0.0f, 0.0f, 3.0f));
    scene.path->getCurve().setPointPosition(1, Math::Vec3(10.0f, 0.0f, 3.0f));
    CHECK(nearVec3(follower->getPosition(), 5.0f, 0.0f, 3.0f));

    follower->setProgress(1.0f);
    CHECK(nearVec3(follower->getPosition(), 1.0f, 0.0f, 3.0f));

    Curve3D replacement;
    addLine(replacement, Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 20.0f));
    scene.path->getCurve() = replacement;
    CHECK(near(scene.path->getCurve().getBakedLength(), 20.0f));
    CHECK(near(follower->getProgress(), 1.0f));
    CHECK(nearVec3(follower->getPosition(), 0.0f, 0.0f, 1.0f));
    follower->setProgressRatio(0.5f);
    CHECK(nearVec3(follower->getPosition(), 0.0f, 0.0f, 10.0f));

    scene.path->getCurve().setBakeInterval(1.0f);
    CHECK(nearVec3(follower->getPosition(), 0.0f, 0.0f, 10.0f));

    Node3D* holder = scene.tree.getRoot()->createChild<Node3D>();
    Path3D* outside = holder->createChild<Path3D>();
    gChanged = 0;
    outside->curveChanged = countChanged;
    addLine(outside->getCurve(), Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(5.0f, 0.0f, 0.0f));
    CHECK(gChanged == 2);

    int updates = 0;
    scene.path->setUpdateCallback([&updates]() { ++updates; });
    scene.path->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    scene.tree.process(kStep);
    CHECK(updates >= 1);
    const int before = updates;
    scene.path->setUpdateCallback(ct::Function<void()>());
    scene.path->setPosition(Math::Vec3(4.0f, 5.0f, 6.0f));
    scene.tree.process(kStep);
    CHECK(updates == before);
}

void testNodes()
{
    Scene scene;
    PathFollow3D* follower = scene.follower;
    follower->setProgress(5.0f);
    CHECK(nearVec3(follower->getPosition(), 5.0f, 0.0f, 0.0f));

    follower->reparent(scene.tree.getRoot());
    CHECK(follower->getParent() == scene.tree.getRoot());
    scene.path->getCurve().setPointPosition(0, Math::Vec3(0.0f, 7.0f, 0.0f));
    scene.path->getCurve().setPointPosition(1, Math::Vec3(10.0f, 7.0f, 0.0f));
    CHECK(nearVec3(follower->getPosition(), 5.0f, 0.0f, 0.0f));
    follower->setProgress(2.0f);
    CHECK(near(follower->getProgress(), 2.0f));
    follower->setProgress(14.0f);
    CHECK(near(follower->getProgress(), 14.0f));

    follower->reparent(scene.path);
    CHECK(nearVec3(follower->getPosition(), 10.0f, 7.0f, 0.0f, 1e-3f));
    follower->setProgress(3.0f);
    CHECK(nearVec3(follower->getPosition(), 3.0f, 7.0f, 0.0f));

    PathFollow3D* extra = scene.path->createChild<PathFollow3D>();
    extra->setProgress(1.0f);
    extra->queueFree();
    scene.tree.process(kStep);
    CHECK(scene.path->getChildCount() == 1);
    scene.path->getCurve().setPointPosition(0, Math::Vec3(0.0f, 0.0f, 0.0f));
    scene.path->getCurve().setPointPosition(1, Math::Vec3(10.0f, 0.0f, 0.0f));
    CHECK(nearVec3(follower->getPosition(), 3.0f, 0.0f, 0.0f));

    Path3D* second = scene.tree.getRoot()->createChild<Path3D>();
    addLine(second->getCurve(), Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 4.0f, 0.0f));
    follower->reparent(second);
    CHECK(near(second->getCurve().getBakedLength(), 4.0f));
    CHECK(nearVec3(follower->getPosition(), 0.0f, 3.0f, 0.0f));
    follower->setProgress(5.0f);
    CHECK(near(follower->getProgress(), 1.0f));

    second->queueFree();
    scene.tree.process(kStep);
    CHECK(scene.tree.getRoot()->getChildCount() == 1);

    PathFollow3D* last = scene.path->createChild<PathFollow3D>();
    last->setProgress(4.0f);
    scene.path->queueFree();
    scene.tree.process(kStep);
    CHECK(scene.tree.getRoot()->getChildCount() == 0);

    Path3D* lone = scene.tree.getRoot()->createChild<Path3D>();
    PathFollow3D* mate = lone->createChild<PathFollow3D>();
    mate->setProgress(1.0f);
    CHECK(nearVec3(mate->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(near(mate->getProgressRatio(), 0.0f));
    addLine(lone->getCurve(), Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 6.0f));
    mate->setProgress(2.0f);
    CHECK(nearVec3(mate->getPosition(), 0.0f, 0.0f, 2.0f));
    lone->getCurve().clearPoints();
    CHECK(nearVec3(mate->getPosition(), 0.0f, 0.0f, 2.0f));
    mate->queueFree();
    lone->getCurve().addPoint(Math::Vec3(1.0f, 1.0f, 1.0f));
    scene.tree.process(kStep);
    CHECK(lone->getChildCount() == 0);
}

} // namespace

int main()
{
    testPoints();
    testChanged();
    testSample();
    testBakedLine();
    testBakedBezier();
    testTessellate();
    testClosed();
    testTiltAndUp();
    testSinglePoint();
    testClosest();
    testFollowerPosition();
    testFollowerLoop();
    testFollowerRatio();
    testFollowerOffsets();
    testRotationModes();
    testFollowerTilt();
    testCurveChange();
    testNodes();

    if (failures != 0)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("path: all checks passed\n");
    return 0;
}
