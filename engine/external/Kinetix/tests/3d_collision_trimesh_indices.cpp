#include "check.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"

#include <kinetix/core/Math.h>

using namespace kinetix::Physics;
using kinetix::u32;
namespace KM = kinetix::Math;

int main()
{
    // A floor of two triangles, with a third naming vertex 7 of 4 and a
    // fourth naming a huge index, between and after them.
    const KM::vec3 vertices[4] = {KM::vec3(-1, 0, -1), KM::vec3(1, 0, -1), KM::vec3(1, 0, 1),
                                  KM::vec3(-1, 0, 1)};
    const u32 indices[12] = {0, 2, 1, 0, 7, 2, 0, 3, 2, 4000000000u, 1, 2};
    const TrimeshShape mesh(vertices, 4, indices, 12);
    CHECK(mesh.triangleCount() == 2);

    for (u32 i = 0; i < mesh.triangleCount(); ++i)
    {
        const TriangleShape triangle = mesh.triangle(i);
        CHECK(KM::length(triangle.rawNormal()) > 0.0f);
    }

    const TriangleShape first = mesh.triangle(0);
    const TriangleShape second = mesh.triangle(1);
    CHECK(first.edgeIsShared(0) || first.edgeIsShared(1) || first.edgeIsShared(2));
    CHECK(second.edgeIsShared(0) || second.edgeIsShared(1) || second.edgeIsShared(2));

    TrimeshShape::RayHit rayHit;
    kinetix::Ray ray;
    ray.origin = KM::vec3(0.2f, 1.0f, 0.3f);
    ray.direction = KM::vec3(0.0f, -1.0f, 0.0f);
    CHECK(mesh.raycast(ray, 10.0f, rayHit));
    CHECK_NEAR(rayHit.distance, 1.0f, 1e-5f);

    TrimeshShape::SweepHit sweepHit;
    CHECK(mesh.sweepSphere(KM::vec3(0.2f, 1.0f, 0.3f), 0.25f, KM::vec3(0.0f, -2.0f, 0.0f),
                           sweepHit));
    CHECK_NEAR(sweepHit.t, 0.375f, 1e-4f);

    ct::Vector<u32> touching;
    mesh.overlapSphere(KM::vec3(0.0f, 0.1f, 0.0f), 0.2f, touching);
    CHECK(touching.size() == 2);

    ct::Vector<u32> flipped;
    mesh.windingErrors(flipped);

    const SphereShape sphere(0.25f);
    const KM::mat4 identity(::Math::Mat3::Identity(), KM::vec3(0.0f));
    ct::Vector<ContactManifold> manifolds;
    CHECK(Narrowphase::convexTrimesh(sphere,
                                     KM::mat4(::Math::Mat3::Identity(), KM::vec3(0.5f, 0.2f, 0.5f)),
                                     mesh, identity, manifolds));

    const TrimeshShape empty(nullptr, 0, indices, 12);
    CHECK(empty.triangleCount() == 0);
    CHECK(!empty.raycast(ray, 10.0f, rayHit));
    return TEST_RESULT();
}
