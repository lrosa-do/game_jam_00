#include "check.h"

#include <geometry/ConvexHullComputer.h>
#include <marine/BuoyancyBody.h>
#include <marine/HullMesh.h>

#include <math.h>
#include <stdlib.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;

f32 random(f32 lo, f32 hi)
{
    return lo + (hi - lo) * static_cast<f32>(rand()) / static_cast<f32>(RAND_MAX);
}

f32 triangleArea(const KM::vec3& a, const KM::vec3& b, const KM::vec3& c)
{
    return 0.5f * KM::length(KM::cross(b - a, c - a));
}

f32 piecesArea(const WetTriangle* pieces, u32 count)
{
    f32 area = 0.0f;
    for (u32 i = 0; i < count; ++i)
        area += triangleArea(pieces[i].vertices[0].position, pieces[i].vertices[1].position,
                             pieces[i].vertices[2].position);
    return area;
}

bool piecesFinite(const WetTriangle* pieces, u32 count)
{
    for (u32 i = 0; i < count; ++i)
        for (const WetVertex& v : pieces[i].vertices)
            if (!isfinite(v.position.x) || !isfinite(v.position.y) || !isfinite(v.position.z) ||
                !isfinite(v.depth) || !isfinite(v.flow.x))
                return false;
    return true;
}

// Water at y = level over the triangle: depth = level - y.
void setTriangle(WetVertex v[3], const KM::vec3& a, const KM::vec3& b, const KM::vec3& c, f32 level)
{
    v[0].position = a;
    v[1].position = b;
    v[2].position = c;
    for (int i = 0; i < 3; ++i)
    {
        v[i].depth = level - v[i].position.y;
        v[i].flow = v[i].position * 2.0f;
    }
}

void clippingAnalytic()
{
    // Right triangle with legs 2 along x and y: area 2. The part above
    // y = h is a similar triangle scaled by (2 - h) / 2.
    const KM::vec3 a(0.0f, 0.0f, 0.0f);
    const KM::vec3 b(2.0f, 0.0f, 0.0f);
    const KM::vec3 c(0.0f, 2.0f, 0.0f);
    WetVertex v[3];
    WetTriangle pieces[2];
    KM::vec3 cut[2];
    bool crosses = false;

    setTriangle(v, a, b, c, 1.0f);
    u32 count = clipTriangleToWater(v, pieces, cut, &crosses);
    CHECK(count == 2);
    CHECK(crosses);
    CHECK_NEAR(piecesArea(pieces, count), 2.0 - 0.5, 1e-6);
    CHECK_NEAR(cut[0].y, 1.0, 1e-6);
    CHECK_NEAR(cut[1].y, 1.0, 1e-6);
    CHECK_NEAR(KM::length(cut[1] - cut[0]), 1.0, 1e-6);
    for (u32 p = 0; p < count; ++p)
        for (const WetVertex& w : pieces[p].vertices)
        {
            CHECK_NEAR(KM::length(w.flow - w.position * 2.0f), 0.0, 1e-5);
            CHECK(w.depth >= 0.0f);
        }

    setTriangle(v, KM::vec3(0.0f, 2.0f, 0.0f), KM::vec3(2.0f, 2.0f, 0.0f), KM::vec3(0.0f, 0.0f, 0.0f), 1.5f);
    count = clipTriangleToWater(v, pieces, cut, &crosses);
    CHECK(count == 1);
    CHECK_NEAR(piecesArea(pieces, count), 2.0 * (1.5 * 1.5) / 4.0, 1e-6);

    setTriangle(v, a, b, c, 1.3f);
    count = clipTriangleToWater(v, pieces);
    const KM::vec3 normal = KM::cross(b - a, c - a);
    for (u32 p = 0; p < count; ++p)
    {
        const KM::vec3 n = KM::cross(pieces[p].vertices[1].position - pieces[p].vertices[0].position,
                                     pieces[p].vertices[2].position - pieces[p].vertices[0].position);
        CHECK(KM::dot(n, normal) > 0.0f);
    }

    setTriangle(v, a, b, c, 5.0f);
    CHECK(clipTriangleToWater(v, pieces, cut, &crosses) == 1);
    CHECK(!crosses);
    CHECK_NEAR(piecesArea(pieces, 1), 2.0, 1e-6);
    setTriangle(v, a, b, c, -1.0f);
    CHECK(clipTriangleToWater(v, pieces) == 0);
}

void verticesOnTheWaterline()
{
    const KM::vec3 a(0.0f, 0.0f, 0.0f);
    const KM::vec3 b(2.0f, 0.0f, 0.0f);
    const KM::vec3 c(0.0f, 2.0f, 0.0f);
    WetVertex v[3];
    WetTriangle pieces[2];
    KM::vec3 cut[2];
    bool crosses = false;

    setTriangle(v, a, b, c, 0.0f);
    CHECK(clipTriangleToWater(v, pieces, cut, &crosses) == 0);

    setTriangle(v, a, b, KM::vec3(1.0f, 0.0f, 1.0f), 0.0f);
    CHECK(clipTriangleToWater(v, pieces) == 0);

    setTriangle(v, a, b, KM::vec3(0.0f, -2.0f, 0.0f), 0.0f);
    u32 count = clipTriangleToWater(v, pieces, cut, &crosses);
    CHECK(piecesFinite(pieces, count));
    CHECK_NEAR(piecesArea(pieces, count), 2.0, 1e-6);

    setTriangle(v, KM::vec3(0.0f, 0.0f, 0.0f), KM::vec3(1.0f, 1.0f, 0.0f), KM::vec3(1.0f, -1.0f, 0.0f), 0.0f);
    count = clipTriangleToWater(v, pieces, cut, &crosses);
    CHECK(piecesFinite(pieces, count));
    CHECK_NEAR(piecesArea(pieces, count), 0.5, 1e-6);
    CHECK(crosses);
    CHECK_NEAR(KM::length(cut[0] - cut[1]), 1.0, 1e-6);

    // Depths that differ by a denormal: still finite.
    setTriangle(v, a, b, c, 0.0f);
    v[0].depth = 1.0e-38f;
    v[1].depth = -1.0e-38f;
    v[2].depth = -1.0f;
    count = clipTriangleToWater(v, pieces);
    CHECK(piecesFinite(pieces, count));
}

void clippingConservesArea()
{
    srand(7);
    f32 worst = 0.0f;
    for (int i = 0; i < 2000; ++i)
    {
        WetVertex under[3];
        WetVertex over[3];
        const KM::vec3 p[3] = {KM::vec3(random(-1, 1), random(-1, 1), random(-1, 1)),
                               KM::vec3(random(-1, 1), random(-1, 1), random(-1, 1)),
                               KM::vec3(random(-1, 1), random(-1, 1), random(-1, 1))};
        const KM::vec3 normal = KM::normalize(KM::vec3(random(-0.5f, 0.5f), 1.0f, random(-0.5f, 0.5f)));
        const f32 offset = random(-0.8f, 0.8f);
        for (int k = 0; k < 3; ++k)
        {
            under[k].position = p[k];
            under[k].depth = offset - KM::dot(normal, p[k]);
            over[k] = under[k];
            over[k].depth = -under[k].depth;
        }
        WetTriangle a[2];
        WetTriangle b[2];
        const u32 na = clipTriangleToWater(under, a);
        const u32 nb = clipTriangleToWater(over, b);
        const f32 total = triangleArea(p[0], p[1], p[2]);
        const f32 error = fabsf(piecesArea(a, na) + piecesArea(b, nb) - total);
        worst = fmaxf(worst, error / fmaxf(total, 1.0e-3f));
        for (u32 q = 0; q < na; ++q)
            for (const WetVertex& v : a[q].vertices)
                if (v.depth == 0.0f)
                    CHECK_NEAR(offset - KM::dot(normal, v.position), 0.0, 1e-5);
    }
    printf("clipping area conservation: worst relative error %.2e\n", double(worst));
    CHECK(worst < 1.0e-5f);
}

void boxMesh()
{
    for (u32 n = 1; n <= 4; ++n)
    {
        const HullMesh box = HullMesh::box(KM::vec3(1.0f, 0.5f, 2.0f), n);
        CHECK(box.isClosed());
        CHECK(box.triangleCount() == 12 * n * n);
        CHECK(box.vertexCount() == 6 * n * n + 2);
        CHECK_NEAR(box.volume(), 8.0 * 1.0 * 0.5 * 2.0, 1e-5);
        CHECK_NEAR(box.surfaceArea(), 2.0 * (2.0 * 1.0 + 2.0 * 4.0 + 1.0 * 4.0), 1e-4);
        CHECK_NEAR(KM::length(box.centroid()), 0.0, 1e-6);
        CHECK_NEAR(box.extent(KM::vec3(0.0f, 0.0f, 1.0f)), 4.0, 1e-6);
        for (u32 t = 0; t < box.triangleCount(); ++t)
        {
            const KM::vec3& a = box.vertices()[box.indices()[3 * t]];
            const KM::vec3& b = box.vertices()[box.indices()[3 * t + 1]];
            const KM::vec3& c = box.vertices()[box.indices()[3 * t + 2]];
            const KM::vec3 centre = (a + b + c) / 3.0f;
            CHECK(KM::dot(box.triangleNormal(t), centre) > 0.0f);
        }
    }
}

void sphereMesh()
{
    f32 previousError = 1.0f;
    for (u32 rings = 8; rings <= 32; rings *= 2)
    {
        const HullMesh sphere = HullMesh::sphere(2.0f, rings, 2 * rings);
        CHECK(sphere.isClosed());
        const f32 exact = 4.0f / 3.0f * kPi * 8.0f;
        const f32 error = (exact - sphere.volume()) / exact;
        // Inscribed: smaller, and converging with the square of the spacing.
        CHECK(error > 0.0f);
        CHECK(error < previousError * 0.3f);
        previousError = error;
        CHECK_NEAR(KM::length(sphere.centroid()), 0.0, 1e-4);
    }
    printf("sphere 32x64: volume error %.3f%%\n", double(previousError * 100.0f));
}

void convexHullMesh()
{
    kinetix::Geometry::ConvexHullComputer computer;
    const f32 corners[] = {-1, -2, -3, 1, -2, -3, -1, 2, -3, 1, 2, -3, -1, -2, 3, 1, -2, 3, -1, 2, 3, 1, 2, 3};
    computer.compute(corners, 3 * sizeof(f32), 8, 0.0f, 0.0f);
    const HullMesh box = HullMesh::fromConvexHull(computer);
    CHECK(box.isClosed());
    CHECK_NEAR(box.volume(), 2.0 * 4.0 * 6.0, 1e-4);

    srand(11);
    f32 points[3 * 40];
    for (f32& p : points)
        p = random(-1.0f, 1.0f);
    computer.compute(points, 3 * sizeof(f32), 40, 0.0f, 0.0f);
    const HullMesh hull = HullMesh::fromConvexHull(computer);
    CHECK(hull.isClosed());
    CHECK(hull.volume() > 0.5f && hull.volume() < 8.0f);
    bool behind = true;
    for (u32 t = 0; t < hull.triangleCount(); ++t)
    {
        const KM::vec3& a = hull.vertices()[hull.indices()[3 * t]];
        for (const KM::vec3& v : hull.vertices())
            if (KM::dot(v - a, hull.triangleNormal(t)) > 1.0e-4f)
                behind = false;
    }
    CHECK(behind);
}

void boatMesh()
{
    BoatHullParameters p;
    const HullMesh vee = HullMesh::boat(p);
    printf("V hull: %u triangles, %.3f m^3, %.2f m^2, centroid (%.3f, %.3f, %.3f)\n", vee.triangleCount(),
           double(vee.volume()), double(vee.surfaceArea()), double(vee.centroid().x),
           double(vee.centroid().y), double(vee.centroid().z));
    CHECK(vee.isClosed());
    CHECK(vee.volume() > 0.0f);
    const KM::vec3 size = vee.boundsMax() - vee.boundsMin();
    CHECK(vee.volume() < size.x * size.y * size.z);
    CHECK_NEAR(size.z, p.length, 1e-5);
    CHECK_NEAR(size.x, p.beam, 0.02);
    CHECK_NEAR(vee.boundsMin().y, 0.0, 1e-6);
    CHECK_NEAR(vee.centroid().x, 0.0, 1e-5);
    CHECK_NEAR(vee.boundsMax().x, -vee.boundsMin().x, 1e-5);
    // The bottom aft of the forefoot is a V: its triangles face down and
    // outboard at the deadrise, the transom faces aft.
    f32 bottomDeadrise = 0.0f;
    u32 bottom = 0;
    bool transomAft = true;
    for (u32 t = 0; t < vee.triangleCount(); ++t)
    {
        const KM::vec3& a = vee.vertices()[vee.indices()[3 * t]];
        const KM::vec3& b = vee.vertices()[vee.indices()[3 * t + 1]];
        const KM::vec3& c = vee.vertices()[vee.indices()[3 * t + 2]];
        const KM::vec3 n = vee.triangleNormal(t);
        if (a.z == -3.0f && b.z == -3.0f && c.z == -3.0f)
            transomAft = transomAft && n.z < -0.99f;
        const KM::vec3 centre = (a + b + c) / 3.0f;
        if (centre.z > -3.0f && centre.z < -2.5f && centre.y < 0.05f && fabsf(centre.x) > 0.1f)
        {
            bottomDeadrise += atan2f(fabsf(n.x), -n.y);
            ++bottom;
        }
    }
    CHECK(transomAft);
    CHECK(bottom > 0);
    if (bottom > 0)
    {
        // The panels twist as the deadrise grows towards the bow; within a
        // degree and a half of the transom's just forward of it.
        const f32 mean = bottomDeadrise / static_cast<f32>(bottom);
        printf("bottom deadrise near the transom: %.2f deg (%u triangles)\n", double(mean * 180.0f / kPi), bottom);
        CHECK_NEAR(mean, p.deadrise, 1.5f * kPi / 180.0f);
    }

    p.bilgeRadius = 0.35f;
    const HullMesh round = HullMesh::boat(p);
    CHECK(round.isClosed());
    CHECK(round.volume() < vee.volume() && round.volume() > 0.9f * vee.volume());

    p.stations = 3;
    p.sectionPoints = 2;
    CHECK(HullMesh::boat(p).isClosed());
    p.stations = 60;
    p.sectionPoints = 16;
    p.bilgeRadius = 0.0f;
    CHECK(HullMesh::boat(p).isClosed());
}

void meshRepair()
{
    // A cube with every face's corners separate, as a renderer would have it.
    const KM::vec3 c[8] = {KM::vec3(-1, -1, -1), KM::vec3(1, -1, -1), KM::vec3(1, 1, -1), KM::vec3(-1, 1, -1),
                           KM::vec3(-1, -1, 1),  KM::vec3(1, -1, 1),  KM::vec3(1, 1, 1),  KM::vec3(-1, 1, 1)};
    const u32 faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}, {0, 4, 7, 3}};
    KM::vec3 vertices[24];
    u32 indices[36];
    for (u32 f = 0; f < 6; ++f)
    {
        for (u32 k = 0; k < 4; ++k)
            vertices[4 * f + k] = c[faces[f][k]];
        const u32 quad[6] = {0, 1, 2, 0, 2, 3};
        for (u32 k = 0; k < 6; ++k)
            indices[6 * f + k] = 4 * f + quad[k];
    }
    HullMesh split(vertices, 24, indices, 36);
    CHECK(!split.isClosed());
    CHECK(split.openEdgeCount() == 24);
    CHECK_NEAR(split.volume(), 8.0, 1e-5);
    split.weldVertices();
    CHECK(split.vertexCount() == 8);
    CHECK(split.isClosed());

    u32 inverted[36];
    for (u32 t = 0; t < 12; ++t)
    {
        inverted[3 * t] = indices[3 * t];
        inverted[3 * t + 1] = indices[3 * t + 2];
        inverted[3 * t + 2] = indices[3 * t + 1];
    }
    HullMesh fixed(vertices, 24, inverted, 36);
    CHECK_NEAR(fixed.volume(), 8.0, 1e-5);

    u32 bad[9] = {0, 1, 2, 0, 1, 99, 3, 3, 1};
    HullMesh partial(vertices, 24, bad, 9);
    CHECK(partial.triangleCount() == 1);

    HullMesh box = HullMesh::box(KM::vec3(0.5f, 1.0f, 1.5f), 2);
    box.translate(KM::vec3(1.0f, -2.0f, 3.0f));
    CHECK_NEAR(box.centroid().x, 1.0, 1e-5);
    CHECK_NEAR(box.centroid().y, -2.0, 1e-5);
    CHECK_NEAR(box.centroid().z, 3.0, 1e-5);
    box.rotate(KM::angleAxis(0.7f, KM::normalize(KM::vec3(1.0f, 2.0f, 3.0f))));
    CHECK_NEAR(box.volume(), 6.0, 1e-4);
    CHECK(box.isClosed());
}
}

int main()
{
    clippingAnalytic();
    verticesOnTheWaterline();
    clippingConservesArea();
    boxMesh();
    sphereMesh();
    convexHullMesh();
    boatMesh();
    meshRepair();
    return TEST_RESULT();
}
