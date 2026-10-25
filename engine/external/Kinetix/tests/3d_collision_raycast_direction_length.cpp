#include "check.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"

#include <kinetix/core/Math.h>

#include <stdlib.h>

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

}

int main()
{
    const KM::mat4 pose(::Math::Mat3::Identity(), KM::vec3(0.0f, 0.0f, -5.0f));
    kinetix::Ray ray;
    ray.origin = KM::vec3(0.0f);
    ray.direction = KM::vec3(0.0f, 0.0f, -2.0f);
    ShapeRayHit hit;

    // Sphere and box of radius 1 at z = -5: both surfaces at z = -4, t = 2.
    const SphereShape sphere(1.0f);
    const BoxShape box(KM::vec3(1.0f));
    CHECK(Narrowphase::raycast(box, pose, ray, 100.0f, hit));
    CHECK_NEAR(hit.distance, 2.0f, 1e-5f);
    CHECK(Narrowphase::raycast(sphere, pose, ray, 100.0f, hit));
    CHECK_NEAR(hit.distance, 2.0f, 1e-5f);
    CHECK_NEAR(hit.point.z, -4.0f, 1e-5f);
    CHECK_NEAR(hit.normal.z, 1.0f, 1e-5f);
    // maxDistance is in the same unit: 1.9 of these steps falls short.
    CHECK(!Narrowphase::raycast(sphere, pose, ray, 1.9f, hit));

    // A capsule lying along z, its end cap facing the ray: cap tip at z = -3.5.
    const CapsuleShape capsule(1.0f, 0.5f);
    const KM::mat4 lying(::Math::Mat3::RotationX(1.5707963f), KM::vec3(0.0f, 0.0f, -5.0f));
    CHECK(Narrowphase::raycast(capsule, lying, ray, 100.0f, hit));
    CHECK_NEAR(hit.distance, 1.75f, 1e-5f);
    CHECK_NEAR(hit.point.z, -3.5f, 1e-5f);

    // Random rays, each cast with its direction at unit length and scaled by
    // a random factor: same point, t divided by the factor - for every shape.
    srand(99);
    const CylinderShape cylinder(0.7f, 0.6f);
    const CollisionShape* shapes[] = {&sphere, &box, &capsule, &cylinder};
    u32 wrong = 0;
    u32 hits = 0;
    for (u32 trial = 0; trial < 2000; ++trial)
    {
        const CollisionShape& shape = *shapes[trial % 4];
        const KM::mat4 shapePose(::Math::Mat3::RotationX(randomRange(0, 6.28f)) *
                                     ::Math::Mat3::RotationY(randomRange(0, 6.28f)),
                                 KM::vec3(randomRange(-1, 1), randomRange(-1, 1),
                                          randomRange(-1, 1)));
        kinetix::Ray unit;
        unit.origin = KM::vec3(randomRange(-4, 4), randomRange(-4, 4), randomRange(-4, 4));
        unit.direction = KM::normalize(KM::vec3(shapePose[3]) - unit.origin +
                                       KM::vec3(randomRange(-1, 1), randomRange(-1, 1),
                                                randomRange(-1, 1)));
        const f32 scale = randomRange(0.1f, 10.0f);
        kinetix::Ray scaled = unit;
        scaled.direction = unit.direction * scale;

        ShapeRayHit unitHit;
        ShapeRayHit scaledHit;
        const bool hitUnit = Narrowphase::raycast(shape, shapePose, unit, 100.0f, unitHit);
        const bool hitScaled =
            Narrowphase::raycast(shape, shapePose, scaled, 100.0f / scale, scaledHit);
        if (hitUnit != hitScaled)
        {
            ++wrong;
            continue;
        }
        if (!hitUnit)
            continue;
        ++hits;
        if (fabsf(scaledHit.distance * scale - unitHit.distance) > 1e-3f ||
            KM::length(scaledHit.point - unitHit.point) > 1e-3f ||
            KM::length(scaledHit.normal - unitHit.normal) > 1e-3f)
            ++wrong;
    }
    CHECK(wrong == 0);
    CHECK(hits > 500);
    return TEST_RESULT();
}
