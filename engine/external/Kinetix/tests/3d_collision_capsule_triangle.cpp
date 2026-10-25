#include "check.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"

#include <kinetix/core/Math.h>

#include <stdlib.h>

// Over a triangle much larger than the capsule the answer is the plane's: out along the face normal to the side
// the capsule's centre is on, as deep as the radius past its deepest end.

using namespace kinetix::Physics;
using kinetix::u32;
using kinetix::f32;
namespace KM = kinetix::Math;

namespace
{

f32 randomRange(f32 low, f32 high)
{
    return low + (high - low) * static_cast<f32>(rand()) / static_cast<f32>(RAND_MAX);
}

const KM::mat4 kIdentity(::Math::Mat3::Identity(), KM::vec3(0.0f));

void reviewCases()
{
    const TriangleShape floor(KM::vec3(-10, 0, -10), KM::vec3(0, 0, 10), KM::vec3(10, 0, -10));
    const CapsuleShape capsule(0.3f, 1.0f);
    ContactManifold manifold;

    // Nearly lying, centre 0.12 above the floor, lower end 0.05 under it.
    const KM::mat4 tilted(::Math::Mat3::RotationZ(1.4f), KM::vec3(0.0f, 0.12f, 0.0f));
    KM::vec3 lower, upper;
    capsule.segment(tilted, lower, upper);
    CHECK(Narrowphase::capsuleTriangle(capsule, tilted, floor, kIdentity, manifold));
    CHECK_NEAR(manifold.normal.y, -1.0f, 1e-5f);
    CHECK_NEAR(manifold.points[0].penetration, 0.3f - KM::min(lower.y, upper.y), 1e-5f);

    // Standing through it, centre 0.2 above: segment from -0.8 to 1.2.
    const KM::mat4 standing(::Math::Mat3::Identity(), KM::vec3(0.0f, 0.2f, 0.0f));
    CHECK(Narrowphase::capsuleTriangle(capsule, standing, floor, kIdentity, manifold));
    CHECK_NEAR(manifold.normal.y, -1.0f, 1e-5f);
    CHECK_NEAR(manifold.points[0].penetration, 1.1f, 1e-5f);

    const KM::mat4 under(::Math::Mat3::Identity(), KM::vec3(0.0f, -0.2f, 0.0f));
    CHECK(Narrowphase::capsuleTriangle(capsule, under, floor, kIdentity, manifold));
    CHECK_NEAR(manifold.normal.y, 1.0f, 1e-5f);
    CHECK_NEAR(manifold.points[0].penetration, 1.1f, 1e-5f);

    const KM::mat4 lying(::Math::Mat3::RotationZ(1.5707963f), KM::vec3(0.0f, 0.0f, 0.0f));
    CHECK(Narrowphase::capsuleTriangle(capsule, lying, floor, kIdentity, manifold));
    CHECK_NEAR(fabsf(manifold.normal.y), 1.0f, 1e-5f);
    CHECK_NEAR(manifold.points[0].penetration, 0.3f, 1e-4f);
}

// Random capsules near the middle of a huge triangle, in any orientation and
// either winding, against the plane answer. With the ends at heights h0, h1
// the capsule leaves to the side s = sign(h0 + h1), as deep as
// r - min(s h0, s h1) - one rule whether or not the segment crosses.
void randomAgainstPlane()
{
    srand(4242);
    u32 wrong = 0;
    u32 crossing = 0;
    u32 hits = 0;
    for (u32 trial = 0; trial < 5000; ++trial)
    {
        const KM::mat3 tilt = ::Math::Mat3::RotationX(randomRange(-0.6f, 0.6f)) *
                              ::Math::Mat3::RotationZ(randomRange(-0.6f, 0.6f));
        const KM::mat4 meshPose(tilt, KM::vec3(randomRange(-1, 1), randomRange(-1, 1),
                                               randomRange(-1, 1)));
        const bool flipped = (trial & 1) != 0;
        const KM::vec3 a(-50, 0, -50), b(0, 0, 50), c(50, 0, -50);
        const TriangleShape triangle(a, flipped ? c : b, flipped ? b : c);

        const CapsuleShape capsule(randomRange(0.05f, 0.5f), randomRange(0.0f, 1.5f));
        const KM::mat3 rotation = ::Math::Mat3::RotationX(randomRange(0.0f, 6.2831853f)) *
                                  ::Math::Mat3::RotationY(randomRange(0.0f, 6.2831853f)) *
                                  ::Math::Mat3::RotationZ(randomRange(0.0f, 6.2831853f));
        const KM::vec3 up = KM::mat3(meshPose) * KM::vec3(0.0f, 1.0f, 0.0f);
        const KM::vec3 centre = KM::vec3(meshPose[3]) + up * randomRange(-1.5f, 1.5f) +
                                KM::vec3(randomRange(-2, 2), 0.0f, randomRange(-2, 2));
        const KM::mat4 capsulePose(rotation, centre);

        KM::vec3 lower, upper;
        capsule.segment(capsulePose, lower, upper);
        const f32 heightLower = KM::dot(up, lower - KM::vec3(meshPose[3]));
        const f32 heightUpper = KM::dot(up, upper - KM::vec3(meshPose[3]));
        const f32 side = heightLower + heightUpper >= 0.0f ? 1.0f : -1.0f;
        const f32 depth =
            capsule.radius() - KM::min(side * heightLower, side * heightUpper);
        if (fabsf(depth) < 1e-4f || fabsf(heightLower + heightUpper) < 1e-4f)
            continue;
        if (heightLower * heightUpper < 0.0f)
            ++crossing;

        ContactManifold manifold;
        const bool hit =
            Narrowphase::capsuleTriangle(capsule, capsulePose, triangle, meshPose, manifold);
        if (hit != (depth > 0.0f))
        {
            ++wrong;
            continue;
        }
        if (!hit)
            continue;
        ++hits;
        // Normal from the capsule into the triangle: against the way out.
        if (KM::length(manifold.normal + up * side) > 1e-3f ||
            fabsf(manifold.points[0].penetration - depth) > 1e-3f)
            ++wrong;
    }
    CHECK(wrong == 0);
    CHECK(crossing > 500);
    CHECK(hits > 1000);
}

}

int main()
{
    reviewCases();
    randomAgainstPlane();
    return TEST_RESULT();
}
