#include "check.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "geometry/ConvexHullComputer.h"

#include <kinetix/core/Math.h>

using namespace kinetix::Physics;
using kinetix::u32;
using kinetix::f32;
namespace KM = kinetix::Math;

namespace
{

const KM::mat4 kIdentity(::Math::Mat3::Identity(), KM::vec3(0.0f));

ConvexHullShape* cubeHull(f32 half)
{
    ct::Vector<KM::vec3> points;
    for (u32 i = 0; i < 8; ++i)
        points.push_back(KM::vec3((i & 1) ? half : -half, (i & 2) ? half : -half,
                                  (i & 4) ? half : -half));
    kinetix::Geometry::ConvexHullComputer computer;
    computer.compute(&points[0].x, sizeof(KM::vec3), static_cast<int>(points.size()), 0.0f, 0.0f);
    return new ConvexHullShape(computer.vertices.data(), static_cast<u32>(computer.vertices.size()),
                               computer.edges.data(), static_cast<u32>(computer.edges.size()),
                               computer.faces.data(), static_cast<u32>(computer.faces.size()));
}

void checkOnTopFace(const ContactManifold& manifold, u32 expectedCount)
{
    CHECK(manifold.count == expectedCount);
    for (u32 i = 0; i < manifold.count; ++i)
    {
        const KM::vec3& p = manifold.points[i].position;
        CHECK(p.x >= -0.5f - 1e-4f && p.x <= 0.5f + 1e-4f);
        CHECK(p.z >= -0.5f - 1e-4f && p.z <= 0.5f + 1e-4f);
        CHECK_NEAR(p.y, 0.495f, 1e-3f);
        CHECK_NEAR(manifold.points[i].penetration, 0.01f, 1e-4f);
    }
}

void lyingHalfOffABox()
{
    // Along x from -0.2 to 3.8, its underside 0.01 into the top of a unit
    // box that ends at x = 0.5.
    const BoxShape box(KM::vec3(0.5f));
    ConvexHullShape* cube = cubeHull(0.5f);
    const KM::mat4 lying(::Math::Mat3::RotationZ(1.5707963f), KM::vec3(1.8f, 0.79f, 0.0f));
    ContactManifold manifold;

    const CapsuleShape capsule(0.3f, 2.0f);
    CHECK(Narrowphase::collide(capsule, lying, box, kIdentity, manifold));
    checkOnTopFace(manifold, 2);
    CHECK(Narrowphase::collide(*cube, kIdentity, capsule, lying, manifold));
    checkOnTopFace(manifold, 2);

    const CylinderShape cylinder(0.3f, 2.0f);
    CHECK(Narrowphase::collide(cylinder, lying, box, kIdentity, manifold));
    checkOnTopFace(manifold, 2);
    CHECK(Narrowphase::collide(*cube, kIdentity, cylinder, lying, manifold));
    checkOnTopFace(manifold, 2);
    delete cube;
}

void cylinderAcrossAnEdge()
{
    const BoxShape box(KM::vec3(0.5f));
    const CylinderShape cylinder(0.2f, 1.0f);
    // Axis along z, out through the front face at z = 0.5, tipped down 0.02
    // rad towards +z; its underside line 0.005 below the top edge where it
    // passes over it, 0.3 along the axis from the centre.
    const f32 tip = 0.02f;
    const KM::mat3 rotation = ::Math::Mat3::RotationX(1.5707963f + tip);
    const KM::vec3 axis = rotation * KM::vec3(0.0f, 1.0f, 0.0f);
    const KM::vec3 down = rotation * KM::vec3(0.0f, 0.0f, 1.0f);
    CHECK(axis.z > 0.99f && axis.y < 0.0f);
    CHECK(down.y < -0.99f);
    const KM::vec3 overEdge(0.1f, 0.495f, 0.5f);
    const KM::mat4 pose(rotation, overEdge - down * 0.2f - axis * 0.3f);
    ContactManifold manifold;
    CHECK(Narrowphase::collide(box, kIdentity, cylinder, pose, manifold));
    CHECK(manifold.count == 1);
    CHECK_NEAR(manifold.points[0].position.x, 0.1f, 0.02f);
    CHECK_NEAR(manifold.points[0].position.y, 0.4975f, 0.01f);
    CHECK_NEAR(manifold.points[0].position.z, 0.5f, 0.02f);
    CHECK_NEAR(manifold.points[0].penetration, 0.005f, 1e-3f);
}

void edgeAcrossEdge()
{
    // Box A turned 45 degrees about z, its top edge running along z at
    // y = 0.707; box B turned 45 degrees about x, its bottom edge running
    // along x 0.05 below that. The edges cross at (0, 0.682, 0).
    const BoxShape box(KM::vec3(0.5f));
    const f32 e = 0.70710678f;
    const KM::mat4 poseA(::Math::Mat3::RotationZ(0.78539816f), KM::vec3(0.0f));
    const KM::mat4 poseB(::Math::Mat3::RotationX(0.78539816f), KM::vec3(0.0f, 2 * e - 0.05f, 0.0f));
    ContactManifold manifold;
    CHECK(Narrowphase::boxBox(box, poseA, box, poseB, manifold));
    CHECK(manifold.count == 1);
    CHECK_NEAR(manifold.points[0].position.x, 0.0f, 1e-4f);
    CHECK_NEAR(manifold.points[0].position.y, e - 0.025f, 1e-4f);
    CHECK_NEAR(manifold.points[0].position.z, 0.0f, 1e-4f);
    CHECK_NEAR(manifold.points[0].penetration, 0.05f, 1e-4f);

    const KM::mat4 shiftedA(::Math::Mat3::RotationZ(0.78539816f), KM::vec3(0.0f, 0.0f, 0.3f));
    CHECK(Narrowphase::boxBox(box, shiftedA, box, poseB, manifold));
    CHECK_NEAR(manifold.points[0].position.x, 0.0f, 1e-4f);
    CHECK_NEAR(manifold.points[0].position.z, 0.0f, 1e-4f);

    ConvexHullShape* cube = cubeHull(0.5f);
    CHECK(Narrowphase::collide(*cube, shiftedA, box, poseB, manifold));
    CHECK(manifold.count == 1);
    CHECK_NEAR(manifold.points[0].position.x, 0.0f, 1e-3f);
    CHECK_NEAR(manifold.points[0].position.z, 0.0f, 1e-3f);
    CHECK(Narrowphase::collide(*cube, shiftedA, *cube, poseB, manifold));
    CHECK(manifold.count == 1);
    CHECK_NEAR(manifold.points[0].position.x, 0.0f, 1e-3f);
    CHECK_NEAR(manifold.points[0].position.z, 0.0f, 1e-3f);

    // A box edge across a triangle's edge: the triangle hangs in the x = 0
    // plane below its top edge along z; the box, turned 45 degrees about x,
    // lies on it with its bottom edge along x, shifted 0.3 along it.
    const TriangleShape triangle(KM::vec3(0, 0, -1), KM::vec3(0, 0, 1), KM::vec3(0, -2, 0));
    const KM::mat4 boxPose(::Math::Mat3::RotationX(0.78539816f), KM::vec3(0.3f, e - 0.05f, 0.0f));
    CHECK(Narrowphase::boxTriangle(box, boxPose, triangle, kIdentity, manifold));
    CHECK(manifold.count == 1);
    CHECK_NEAR(manifold.points[0].position.x, 0.0f, 1e-4f);
    CHECK_NEAR(manifold.points[0].position.y, -0.025f, 1e-4f);
    CHECK_NEAR(manifold.points[0].position.z, 0.0f, 1e-4f);
    CHECK(Narrowphase::convexHullTriangle(*cube, boxPose, triangle, kIdentity, manifold));
    CHECK(manifold.count == 1);
    CHECK_NEAR(manifold.points[0].position.x, 0.0f, 1e-3f);
    CHECK_NEAR(manifold.points[0].position.z, 0.0f, 1e-3f);
    delete cube;
}

void grazingBoxTriangle()
{
    // A small tumbled box beside the triangle's corner, within the margin on all thirteen axes (speculative contact),
    // where clipping leaves no point at all. The pose was found by search.
    const BoxShape box(KM::vec3(0.1f));
    const TriangleShape triangle(KM::vec3(0, 0, 0), KM::vec3(1, 0, 0), KM::vec3(0, 0, 1));
    const KM::mat3 rotation(KM::vec3(0x1.c0858p-3f, -0x1.d4d3cep-1f, -0x1.5916e4p-2f),
                            KM::vec3(0.0f, -0x1.61acep-2f, 0x1.e07decp-1f),
                            KM::vec3(-0x1.f39212p-1f, -0x1.a4eb78p-3f, -0x1.35d37cp-4f));
    const KM::mat4 pose(rotation, KM::vec3(0x1.7c4782p-2f, -0x1.d676bp-4f, -0x1.ea0b9ep-3f));

    const KM::vec3 edges[3] = {KM::vec3(1, 0, 0), KM::vec3(-1, 0, 1), KM::vec3(0, 0, -1)};
    ct::Vector<KM::vec3> axes;
    for (u32 i = 0; i < 3; ++i)
        axes.push_back(KM::vec3(rotation[i]));
    axes.push_back(KM::vec3(0, 1, 0));
    for (u32 i = 0; i < 3; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes.push_back(KM::cross(KM::vec3(rotation[i]), edges[j]));
    for (const KM::vec3& raw : axes)
    {
        if (KM::length(raw) < 1e-6f)
            continue;
        const KM::vec3 axis = KM::normalize(raw);
        f32 minA, maxA, minB, maxB;
        box.project(pose, axis, minA, maxA);
        triangle.project(kIdentity, axis, minB, maxB);
        CHECK(maxB - minA >= -0.1f && maxA - minB >= -0.1f);
    }

    ContactManifold manifold;
    CHECK(Narrowphase::boxTriangle(box, pose, triangle, kIdentity, manifold, 0.1f));
    CHECK(manifold.count == 1);
}

}

int main()
{
    lyingHalfOffABox();
    cylinderAcrossAnEdge();
    edgeAcrossEdge();
    grazingBoxTriangle();
    return TEST_RESULT();
}
