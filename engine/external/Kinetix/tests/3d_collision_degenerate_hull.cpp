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

ConvexHullShape* hullOf(const ct::Vector<KM::vec3>& points)
{
    kinetix::Geometry::ConvexHullComputer computer;
    computer.compute(&points[0].x, sizeof(KM::vec3), static_cast<int>(points.size()), 0.0f, 0.0f);
    return new ConvexHullShape(computer.vertices.data(), static_cast<u32>(computer.vertices.size()),
                               computer.edges.data(), static_cast<u32>(computer.edges.size()),
                               computer.faces.data(), static_cast<u32>(computer.faces.size()));
}

KM::mat4 at(const KM::vec3& position)
{
    return KM::mat4(::Math::Mat3::Identity(), position);
}

void collideWithEverything(const ConvexHullShape& hull, const KM::vec3& inside,
                           const KM::vec3& outside)
{
    const SphereShape sphere(0.5f);
    const BoxShape box(KM::vec3(0.5f));
    const CapsuleShape capsule(0.3f, 0.5f);
    const CylinderShape cylinder(0.5f, 0.5f);
    const PlaneShape plane(KM::vec3(0.0f, 1.0f, 0.0f), 0.2f);
    const TriangleShape triangle(KM::vec3(-1, 0.05f, -1), KM::vec3(0, 0.05f, 1),
                                 KM::vec3(1, 0.05f, -1));
    ConvexHullShape* cube = hullOf({KM::vec3(-0.5f, -0.5f, -0.5f), KM::vec3(0.5f, -0.5f, -0.5f),
                                    KM::vec3(-0.5f, 0.5f, -0.5f), KM::vec3(0.5f, 0.5f, -0.5f),
                                    KM::vec3(-0.5f, -0.5f, 0.5f), KM::vec3(0.5f, -0.5f, 0.5f),
                                    KM::vec3(-0.5f, 0.5f, 0.5f), KM::vec3(0.5f, 0.5f, 0.5f)});
    const CollisionShape* others[] = {&sphere, &box, &capsule, &cylinder, cube};
    const KM::mat4 hullPose = at(KM::vec3(0.0f));
    ContactManifold manifold;
    for (const CollisionShape* other : others)
    {
        CHECK(Narrowphase::collide(hull, hullPose, *other, at(inside), manifold));
        CHECK(manifold.count > 0);
        CHECK(Narrowphase::collide(*other, at(inside), hull, hullPose, manifold));
        CHECK(!Narrowphase::collide(hull, hullPose, *other, at(outside), manifold));
        CHECK(!Narrowphase::collide(*other, at(outside), hull, hullPose, manifold));
    }
    CHECK(Narrowphase::collide(hull, hullPose, plane, at(KM::vec3(0.0f)), manifold));
    CHECK(!Narrowphase::collide(hull, hullPose, plane, at(KM::vec3(0.0f, -1.0f, 0.0f)), manifold));

    Narrowphase::convexHullTriangle(hull, hullPose, triangle, at(KM::vec3(0.0f)), manifold);
    const KM::vec3 vertices[3] = {triangle.vertex(0), triangle.vertex(1), triangle.vertex(2)};
    const u32 indices[3] = {0, 1, 2};
    const TrimeshShape mesh(vertices, 3, indices, 3);
    ct::Vector<ContactManifold> manifolds;
    Narrowphase::convexTrimesh(hull, hullPose, mesh, at(KM::vec3(0.0f)), manifolds, 0.1f);

    kinetix::Ray ray;
    ray.origin = KM::vec3(0.0f, 2.0f, 0.0f);
    ray.direction = KM::vec3(0.0f, -1.0f, 0.0f);
    ShapeRayHit hit;
    Narrowphase::raycast(hull, hullPose, ray, 10.0f, hit);
    CHECK(Narrowphase::overlapSphere(hull, hullPose, KM::vec3(0.0f, 0.3f, 0.0f), 0.4f));
    CHECK(!Narrowphase::overlapSphere(hull, hullPose, KM::vec3(0.0f, 3.0f, 0.0f), 0.4f));
    delete cube;
}

}

int main()
{
    {
        ConvexHullShape* point = hullOf({KM::vec3(0.0f)});
        CHECK(point->faceCount() == 0);
        collideWithEverything(*point, KM::vec3(0.0f, 0.4f, 0.0f), KM::vec3(0.0f, 3.0f, 0.0f));
        delete point;
        ConvexHullShape* copies =
            hullOf({KM::vec3(0.0f), KM::vec3(0.0f), KM::vec3(0.0f), KM::vec3(0.0f)});
        collideWithEverything(*copies, KM::vec3(0.0f, 0.4f, 0.0f), KM::vec3(3.0f, 0.0f, 0.0f));
        delete copies;
    }

    // Collinear: ConvexHullComputer gives it one "face" of two corners, which
    // must not turn into a face normal.
    {
        ConvexHullShape* segment =
            hullOf({KM::vec3(-1, 0, 0), KM::vec3(0, 0, 0), KM::vec3(1, 0, 0)});
        collideWithEverything(*segment, KM::vec3(0.9f, 0.4f, 0.0f), KM::vec3(0.0f, 0.0f, 3.0f));

        const SphereShape sphere(0.5f);
        ContactManifold manifold;
        CHECK(!Narrowphase::collide(*segment, at(KM::vec3(0.0f)), sphere,
                                    at(KM::vec3(1.51f, 0.0f, 0.0f)), manifold));
        CHECK(Narrowphase::collide(*segment, at(KM::vec3(0.0f)), sphere,
                                   at(KM::vec3(0.0f, 0.0f, 0.45f)), manifold));
        CHECK_NEAR(manifold.points[0].penetration, 0.05f, 1e-5f);
        CHECK_NEAR(manifold.normal.z, 1.0f, 1e-5f);
        delete segment;
    }

    {
        ConvexHullShape* square = hullOf({KM::vec3(-1, 0, -1), KM::vec3(1, 0, -1),
                                          KM::vec3(1, 0, 1), KM::vec3(-1, 0, 1)});
        collideWithEverything(*square, KM::vec3(0.5f, 0.4f, 0.0f), KM::vec3(3.0f, 0.0f, 0.0f));

        // A sphere level with the square but beside it is apart - the two
        // face planes alone would call that inside.
        const SphereShape sphere(0.2f);
        ContactManifold manifold;
        CHECK(!Narrowphase::collide(*square, at(KM::vec3(0.0f)), sphere,
                                    at(KM::vec3(1.5f, 0.0f, 0.0f)), manifold));
        CHECK(Narrowphase::collide(*square, at(KM::vec3(0.0f)), sphere,
                                   at(KM::vec3(0.0f, 0.1f, 0.0f)), manifold));
        CHECK_NEAR(manifold.points[0].penetration, 0.1f, 1e-5f);
        delete square;
    }

    {
        const ConvexHullShape empty(nullptr, 0, nullptr, 0, nullptr, 0);
        const BoxShape box(KM::vec3(0.5f));
        ContactManifold manifold;
        CHECK(!Narrowphase::collide(empty, at(KM::vec3(0.0f)), box, at(KM::vec3(0.0f)), manifold));
        CHECK(!Narrowphase::collide(box, at(KM::vec3(0.0f)), empty, at(KM::vec3(0.0f)), manifold));
    }
    return TEST_RESULT();
}
