#include "check.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "geometry/ConvexHullComputer.h"

#include <kinetix/core/Math.h>

#include <stdlib.h>

// Checked against sphereBox() for the sphere and a direct minimisation of segment-to-box distance for the capsule.

using namespace kinetix::Physics;
using kinetix::u32;
using kinetix::f32;
namespace KM = kinetix::Math;

namespace
{

f32 random01()
{
    return static_cast<f32>(rand()) / static_cast<f32>(RAND_MAX);
}

f32 randomRange(f32 low, f32 high)
{
    return low + (high - low) * random01();
}

KM::mat3 randomRotation()
{
    return ::Math::Mat3::RotationX(randomRange(0.0f, 6.2831853f)) *
           ::Math::Mat3::RotationY(randomRange(0.0f, 6.2831853f)) *
           ::Math::Mat3::RotationZ(randomRange(0.0f, 6.2831853f));
}

KM::mat4 pose(const KM::mat3& rotation, const KM::vec3& position)
{
    return KM::mat4(rotation, position);
}

ConvexHullShape* cubeHull(const KM::vec3& half)
{
    ct::Vector<KM::vec3> points;
    for (u32 i = 0; i < 8; ++i)
        points.push_back(KM::vec3((i & 1) ? half.x : -half.x, (i & 2) ? half.y : -half.y,
                                  (i & 4) ? half.z : -half.z));
    kinetix::Geometry::ConvexHullComputer computer;
    computer.compute(&points[0].x, sizeof(KM::vec3), static_cast<int>(points.size()), 0.0f, 0.0f);
    return new ConvexHullShape(computer.vertices.data(), static_cast<u32>(computer.vertices.size()),
                               computer.edges.data(), static_cast<u32>(computer.edges.size()),
                               computer.faces.data(), static_cast<u32>(computer.faces.size()));
}

f32 pointBoxDistance(const KM::vec3& local, const KM::vec3& half)
{
    const KM::vec3 clamped = KM::clamp(local, -half, half);
    return KM::length(local - clamped);
}

// Exact distance from the segment to the box: the distance is convex along
// the segment, so a ternary search finds its minimum.
f32 segmentBoxDistance(const KM::vec3& start, const KM::vec3& end, const KM::vec3& half)
{
    f32 low = 0.0f;
    f32 high = 1.0f;
    for (u32 i = 0; i < 200; ++i)
    {
        const f32 a = low + (high - low) / 3.0f;
        const f32 b = high - (high - low) / 3.0f;
        if (pointBoxDistance(start + (end - start) * a, half) <
            pointBoxDistance(start + (end - start) * b, half))
            high = b;
        else
            low = a;
    }
    return pointBoxDistance(start + (end - start) * ((low + high) * 0.5f), half);
}

void cornerCases()
{
    ConvexHullShape* cube = cubeHull(KM::vec3(0.5f));
    const BoxShape box(KM::vec3(0.5f));
    const KM::mat4 origin = pose(::Math::Mat3::Identity(), KM::vec3(0.0f));
    ContactManifold manifold;

    // Sphere of radius 0.5 centred 0.606 from the cube's corner: apart.
    const SphereShape sphere(0.5f);
    const KM::mat4 offCorner = pose(::Math::Mat3::Identity(), KM::vec3(0.85f));
    CHECK(!Narrowphase::collide(sphere, offCorner, box, origin, manifold));
    CHECK(!Narrowphase::collide(*cube, origin, sphere, offCorner, manifold));
    CHECK(!Narrowphase::collide(sphere, offCorner, *cube, origin, manifold));

    // Within a margin it is a speculative contact, as deep as the true gap
    // and along the corner's diagonal.
    CHECK(Narrowphase::collide(*cube, origin, sphere, offCorner, manifold, 0.2f));
    CHECK_NEAR(manifold.points[0].penetration, 0.5f - sqrtf(3.0f) * 0.35f, 1e-4f);
    CHECK_NEAR(manifold.normal.x, 0.57735f, 1e-4f);
    CHECK_NEAR(manifold.normal.y, 0.57735f, 1e-4f);

    const CapsuleShape capsule(0.5f, 1.0f);
    const KM::mat4 aboveCorner = pose(::Math::Mat3::Identity(), KM::vec3(0.85f, 1.85f, 0.85f));
    CHECK(!Narrowphase::collide(capsule, aboveCorner, box, origin, manifold));
    CHECK(!Narrowphase::collide(*cube, origin, capsule, aboveCorner, manifold));

    // The same capsule beside an edge of the cube rather than a corner:
    // lying along z, 0.35 out on x and y from the edge, 0.495 away.
    const KM::mat4 besideEdge =
        pose(::Math::Mat3::RotationX(1.5707963f), KM::vec3(0.85f, 0.85f, 0.0f));
    CHECK(Narrowphase::collide(capsule, besideEdge, box, origin, manifold));
    CHECK_NEAR(manifold.points[0].penetration, 0.5f - sqrtf(2.0f) * 0.35f, 1e-4f);
    CHECK(Narrowphase::collide(*cube, origin, capsule, besideEdge, manifold));
    CHECK_NEAR(manifold.points[0].penetration, 0.5f - sqrtf(2.0f) * 0.35f, 1e-4f);
    delete cube;
}

void randomSphereAgainstBox()
{
    srand(12345);
    u32 mismatches = 0;
    u32 hits = 0;
    for (u32 trial = 0; trial < 4000; ++trial)
    {
        const KM::vec3 half(randomRange(0.2f, 1.0f), randomRange(0.2f, 1.0f),
                            randomRange(0.2f, 1.0f));
        ConvexHullShape* hull = cubeHull(half);
        const BoxShape box(half);
        const SphereShape sphere(randomRange(0.05f, 0.8f));
        const KM::mat4 boxPose = pose(randomRotation(), KM::vec3(randomRange(-1.0f, 1.0f),
                                                                 randomRange(-1.0f, 1.0f),
                                                                 randomRange(-1.0f, 1.0f)));
        const KM::vec3 offset(randomRange(-2.0f, 2.0f), randomRange(-2.0f, 2.0f),
                              randomRange(-2.0f, 2.0f));
        const KM::mat4 spherePose =
            pose(::Math::Mat3::Identity(), KM::vec3(boxPose[3]) + offset);
        const f32 margin = (trial % 3 == 0) ? 0.1f : 0.0f;

        ContactManifold exact;
        ContactManifold fromHull;
        const bool hitExact = Narrowphase::collide(sphere, spherePose, box, boxPose, exact, margin);
        const bool hitHull =
            Narrowphase::collide(sphere, spherePose, *hull, boxPose, fromHull, margin);
        delete hull;

        // A pair right on the boundary may fall either way by rounding.
        const KM::vec3 local =
            KM::transpose(KM::mat3(boxPose)) * (KM::vec3(spherePose[3]) - KM::vec3(boxPose[3]));
        const f32 gap = pointBoxDistance(local, half) - sphere.radius() - margin;
        if (fabsf(gap) < 1e-4f)
            continue;
        if (hitExact != hitHull)
        {
            ++mismatches;
            continue;
        }
        if (!hitExact)
            continue;
        ++hits;
        // ConvexHullComputer rounds the corners it is given onto its own
        // integer grid - by about 5e-5 here - and the direction to a corner
        // or an edge moves by that much over the distance to it.
        const f32 centreDistance = pointBoxDistance(local, half);
        const f32 normalTolerance = 1e-3f + 2e-4f / KM::max(centreDistance, 1e-4f);
        if (fabsf(exact.points[0].penetration - fromHull.points[0].penetration) > 1e-3f ||
            KM::length(exact.normal - fromHull.normal) > normalTolerance)
            ++mismatches;
    }
    CHECK(mismatches == 0);
    CHECK(hits > 500);
}

// A capsule whose segment stays outside the box touches it when the exact segment-to-box distance is under the radius,
// as deep as the difference.
void randomCapsuleAgainstBox()
{
    srand(777);
    u32 wrongVerdictBox = 0;
    u32 wrongVerdictHull = 0;
    u32 wrongDepth = 0;
    u32 checkedApart = 0;
    for (u32 trial = 0; trial < 3000; ++trial)
    {
        const KM::vec3 half(randomRange(0.2f, 1.0f), randomRange(0.2f, 1.0f),
                            randomRange(0.2f, 1.0f));
        ConvexHullShape* hull = cubeHull(half);
        const BoxShape box(half);
        const CapsuleShape capsule(randomRange(0.05f, 0.6f), randomRange(0.0f, 1.0f));
        const KM::mat4 boxPose = pose(randomRotation(), KM::vec3(randomRange(-1.0f, 1.0f),
                                                                 randomRange(-1.0f, 1.0f),
                                                                 randomRange(-1.0f, 1.0f)));
        const KM::vec3 offset(randomRange(-2.2f, 2.2f), randomRange(-2.2f, 2.2f),
                              randomRange(-2.2f, 2.2f));
        const KM::mat4 capsulePose = pose(randomRotation(), KM::vec3(boxPose[3]) + offset);

        KM::vec3 lower, upper;
        capsule.segment(capsulePose, lower, upper);
        const KM::mat3 inverse = KM::transpose(KM::mat3(boxPose));
        const KM::vec3 boxCentre(boxPose[3]);
        const f32 distance =
            segmentBoxDistance(inverse * (lower - boxCentre), inverse * (upper - boxCentre), half);
        const f32 gap = distance - capsule.radius();

        ContactManifold fromBox;
        ContactManifold fromHull;
        const bool hitBox = Narrowphase::collide(capsule, capsulePose, box, boxPose, fromBox);
        const bool hitHull = Narrowphase::collide(*hull, boxPose, capsule, capsulePose, fromHull);
        delete hull;

        if (fabsf(gap) < 1e-4f)
            continue;
        const bool expected = gap < 0.0f;
        if (hitBox != expected)
            ++wrongVerdictBox;
        if (hitHull != expected)
            ++wrongVerdictHull;
        if (expected && distance > 1e-3f)
        {
            ++checkedApart;
            if (hitBox && fabsf(fromBox.points[0].penetration + gap) > 1e-3f)
                ++wrongDepth;
            if (hitHull && fabsf(fromHull.points[0].penetration + gap) > 1e-3f)
                ++wrongDepth;
        }
    }
    CHECK(wrongVerdictBox == 0);
    CHECK(wrongVerdictHull == 0);
    CHECK(wrongDepth == 0);
    CHECK(checkedApart > 100);
}

}

int main()
{
    cornerCases();
    randomSphereAgainstBox();
    randomCapsuleAgainstBox();
    return TEST_RESULT();
}
