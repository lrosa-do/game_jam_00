#include "check.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "geometry/ConvexHullComputer.h"

#include <kinetix/core/Math.h>

#include <stdlib.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
using kinetix::usize;
namespace KM = kinetix::Math;

// The pruned SAT (Gauss map arcs, face bias) against every axis, brute force:
// the same verdict apart or touching, and the same axis when the minimum is
// clear. Random shards and poses, as debris on a level.
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

ConvexHullShape* randomHull()
{
    const f32 extent = randomRange(0.03f, 0.5f);
    const int count = 8 + rand() % 9;
    ct::Vector<KM::vec3> points;
    for (int i = 0; i < count; ++i)
        points.push_back(KM::vec3(randomRange(-extent, extent), randomRange(-extent, extent),
                                  randomRange(-extent, extent)));
    kinetix::Geometry::ConvexHullComputer computer;
    computer.compute(&points[0].x, sizeof(KM::vec3), static_cast<int>(points.size()), 0.0f, 0.0f);
    return new ConvexHullShape(computer.vertices.data(), static_cast<u32>(computer.vertices.size()),
                               computer.edges.data(), static_cast<u32>(computer.edges.size()),
                               computer.faces.data(), static_cast<u32>(computer.faces.size()));
}

KM::mat4 randomPose(f32 reach)
{
    const ::Math::Mat3 rotation = ::Math::Mat3::RotationX(randomRange(-3.2f, 3.2f)) *
                                  ::Math::Mat3::RotationY(randomRange(-3.2f, 3.2f)) *
                                  ::Math::Mat3::RotationZ(randomRange(-3.2f, 3.2f));
    return KM::mat4(rotation, KM::vec3(randomRange(-reach, reach), randomRange(-reach, reach),
                                       randomRange(-reach, reach)));
}

struct BruteForce
{
    f32 best = 1.0e30f;
    f32 second = 1.0e30f;
    KM::vec3 axis{0.0f, 1.0f, 0.0f};

    void test(const CollisionShape& a, const KM::mat4& transformA, const CollisionShape& b,
              const KM::mat4& transformB, const KM::vec3& raw)
    {
        const f32 length = KM::length(raw);
        if (length < 1e-6f)
            return;
        const KM::vec3 unit = raw / length;
        f32 minA, maxA, minB, maxB;
        a.project(transformA, unit, minA, maxA);
        b.project(transformB, unit, minB, maxB);
        const f32 overlap = KM::min(maxB - minA, maxA - minB);
        if (overlap < best)
        {
            second = best;
            best = overlap;
            axis = unit;
        }
        else if (overlap < second)
            second = overlap;
    }

    bool clear() const
    {
        return second - best > 1e-3f;
    }
};

void worldDirections(const ct::Vector<KM::vec3>& local, const KM::mat4& transform,
                     ct::Vector<KM::vec3>& out)
{
    const KM::mat3 rotation(transform);
    out.clear();
    for (usize i = 0; i < local.size(); ++i)
        if (KM::dot(local[i], local[i]) > 0.5f)
            out.push_back(rotation * local[i]);
}

int gReported = 0;

void report(const char* what, int trial, f32 margin, const BruteForce& brute, bool bruteApart,
            bool satApart, const KM::vec3& satNormal)
{
    if (gReported++ >= 8)
        return;
    fprintf(stderr,
            "%s trial %d margin %.2f: brute %s best %.5f second %.5f axis (%.3f %.3f %.3f); sat %s "
            "normal (%.3f %.3f %.3f)\n",
            what, trial, margin, bruteApart ? "apart" : "touch", brute.best, brute.second,
            brute.axis.x, brute.axis.y, brute.axis.z, satApart ? "apart" : "touch", satNormal.x,
            satNormal.y, satNormal.z);
}

void hullAgainstHull(int trials)
{
    int verdictMismatch = 0;
    int axisMismatch = 0;
    int speculative = 0;
    int touching = 0;
    ct::Vector<KM::vec3> normalsA, normalsB, edgesA, edgesB;
    for (int trial = 0; trial < trials; ++trial)
    {
        ConvexHullShape* a = randomHull();
        ConvexHullShape* b = randomHull();
        const KM::mat4 poseA = randomPose(0.2f);
        const KM::mat4 poseB = randomPose(0.6f);
        worldDirections(a->faceNormals(), poseA, normalsA);
        worldDirections(b->faceNormals(), poseB, normalsB);
        worldDirections(a->edgeDirections(), poseA, edgesA);
        worldDirections(b->edgeDirections(), poseB, edgesB);
        for (int m = 0; m < 2; ++m)
        {
            const f32 margin = m == 0 ? 0.0f : 0.04f;
            BruteForce brute;
            for (usize i = 0; i < normalsA.size(); ++i)
                brute.test(*a, poseA, *b, poseB, normalsA[i]);
            for (usize i = 0; i < normalsB.size(); ++i)
                brute.test(*a, poseA, *b, poseB, normalsB[i]);
            for (usize i = 0; i < edgesA.size(); ++i)
                for (usize j = 0; j < edgesB.size(); ++j)
                    brute.test(*a, poseA, *b, poseB, KM::cross(edgesA[i], edgesB[j]));
            const bool bruteApart = brute.best < -margin;

            ContactManifold manifold;
            const bool satTouch = Narrowphase::convexHullConvexHull(*a, poseA, *b, poseB, manifold, margin);
            if (satTouch == bruteApart)
            {
                if (m == 1) ++speculative; else ++verdictMismatch;
                report("hull-hull", trial, margin, brute, bruteApart, !satTouch, manifold.normal);
                continue;
            }
            if (!satTouch)
                continue;
            ++touching;
            if (brute.clear() && fabsf(KM::dot(brute.axis, manifold.normal)) < 0.999f)
            {
                if (m == 1) ++speculative; else ++axisMismatch;
                report("hull-hull axis", trial, margin, brute, bruteApart, false, manifold.normal);
            }
        }
        delete a;
        delete b;
    }
    printf("hull-hull: %d trials, %d touching, %d verdict mismatches, %d axis mismatches, %d within the "
           "speculative margin\n",
           trials, touching, verdictMismatch, axisMismatch, speculative);
    CHECK(verdictMismatch == 0);
    CHECK(axisMismatch == 0);
    // A pair a hair past the margin counted as speculative, or a speculative point along a
    // near-tied axis: harmless, and rare.
    CHECK(speculative <= 3);
    CHECK(touching > trials / 4);
}

void hullAgainstTriangle(int trials)
{
    int verdictMismatch = 0;
    int axisMismatch = 0;
    int speculative = 0;
    int touching = 0;
    ct::Vector<KM::vec3> normalsA, edgesA;
    const KM::mat4 identity(1.0f);
    for (int trial = 0; trial < trials; ++trial)
    {
        ConvexHullShape* a = randomHull();
        const KM::mat4 poseA = randomPose(0.3f);
        const f32 size = randomRange(0.1f, 2.0f);
        const KM::vec3 v0(randomRange(-size, size), randomRange(-size, size), randomRange(-size, size));
        const KM::vec3 v1(randomRange(-size, size), randomRange(-size, size), randomRange(-size, size));
        const KM::vec3 v2(randomRange(-size, size), randomRange(-size, size), randomRange(-size, size));
        const KM::vec3 faceNormal = KM::cross(v1 - v0, v2 - v0);
        if (KM::length(faceNormal) < 1e-4f)
        {
            delete a;
            continue;
        }
        const TriangleShape triangle(v0, v1, v2);
        worldDirections(a->faceNormals(), poseA, normalsA);
        worldDirections(a->edgeDirections(), poseA, edgesA);
        const KM::vec3 triangleEdges[3] = {v1 - v0, v2 - v1, v0 - v2};
        for (int m = 0; m < 2; ++m)
        {
            const f32 margin = m == 0 ? 0.0f : 0.04f;
            BruteForce brute;
            for (usize i = 0; i < normalsA.size(); ++i)
                brute.test(*a, poseA, triangle, identity, normalsA[i]);
            brute.test(*a, poseA, triangle, identity, faceNormal);
            for (usize i = 0; i < edgesA.size(); ++i)
                for (int j = 0; j < 3; ++j)
                    brute.test(*a, poseA, triangle, identity, KM::cross(edgesA[i], triangleEdges[j]));
            const bool bruteApart = brute.best < -margin;

            ContactManifold manifold;
            const bool satTouch =
                Narrowphase::convexHullTriangle(*a, poseA, triangle, identity, manifold, margin);
            if (satTouch == bruteApart)
            {
                if (m == 1) ++speculative; else ++verdictMismatch;
                report("hull-triangle", trial, margin, brute, bruteApart, !satTouch, manifold.normal);
                continue;
            }
            if (!satTouch)
                continue;
            ++touching;
            if (brute.clear() && fabsf(KM::dot(brute.axis, manifold.normal)) < 0.999f)
            {
                if (m == 1) ++speculative; else ++axisMismatch;
                report("hull-triangle axis", trial, margin, brute, bruteApart, false, manifold.normal);
            }
        }
        delete a;
    }
    printf("hull-triangle: %d trials, %d touching, %d verdict mismatches, %d axis mismatches, %d "
           "within the speculative margin\n",
           trials, touching, verdictMismatch, axisMismatch, speculative);
    CHECK(verdictMismatch == 0);
    CHECK(axisMismatch == 0);
    CHECK(speculative <= 3);
    CHECK(touching > trials / 4);
}

} // namespace

int main()
{
    srand(12345);
    hullAgainstHull(3000);
    hullAgainstTriangle(3000);
    return TEST_RESULT();
}
