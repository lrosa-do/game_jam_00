#include "check.h"

#include "collision/CollisionShape.h"
#include "collision/Narrowphase.h"
#include "geometry/ConvexHullComputer.h"

#include <kinetix/core/Math.h>

#include <stdlib.h>

using namespace kinetix::Physics;
using kinetix::u32;
using kinetix::f32;
using kinetix::usize;
namespace KM = kinetix::Math;

namespace
{

f32 random01()
{
    return static_cast<f32>(rand()) / static_cast<f32>(RAND_MAX);
}

ConvexHullShape* hullOf(const ct::Vector<KM::vec3>& points)
{
    kinetix::Geometry::ConvexHullComputer computer;
    computer.compute(&points[0].x, sizeof(KM::vec3), static_cast<int>(points.size()), 0.0f, 0.0f);
    return new ConvexHullShape(computer.vertices.data(), static_cast<u32>(computer.vertices.size()),
                               computer.edges.data(), static_cast<u32>(computer.edges.size()),
                               computer.faces.data(), static_cast<u32>(computer.faces.size()));
}

// Face `face`'s outward normal and one of its corners, own space, walked from
// the half-edge mesh here rather than taken from the shape.
void facePlane(const ConvexHullShape& hull, u32 face, KM::vec3& normal, KM::vec3& corner)
{
    const ConvexHullShape::Edge* start = &hull.edges()[static_cast<usize>(hull.faces()[face])];
    const ConvexHullShape::Edge* edge = start;
    normal = KM::vec3(0.0f);
    corner = hull.vertices()[static_cast<usize>(start->getSourceVertex())];
    do
    {
        const KM::vec3& a = hull.vertices()[static_cast<usize>(edge->getSourceVertex())];
        const KM::vec3& b = hull.vertices()[static_cast<usize>(edge->getTargetVertex())];
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
        edge = edge->getNextEdgeOfFace();
    } while (edge != start);
    normal = KM::normalize(normal);
    if (KM::dot(normal, corner) < 0.0f)
        normal = -normal;
}

void edgeVectors(const ConvexHullShape& hull, ct::Vector<KM::vec3>& out)
{
    out.clear();
    for (usize i = 0; i < hull.edges().size(); ++i)
    {
        const ConvexHullShape::Edge& edge = hull.edges()[i];
        if (static_cast<usize>(edge.getReverseEdge() - hull.edges().data()) <= i)
            continue;
        out.push_back(hull.vertices()[static_cast<usize>(edge.getTargetVertex())] -
                      hull.vertices()[static_cast<usize>(edge.getSourceVertex())]);
    }
}

void outsideEveryFace()
{
    ct::Vector<KM::vec3> points;
    srand(1);
    while (points.size() < 200)
    {
        const KM::vec3 v(random01() * 2 - 1, random01() * 2 - 1, random01() * 2 - 1);
        const f32 length = KM::length(v);
        if (length > 1e-3f)
            points.push_back(v / length);
    }
    ConvexHullShape* hull = hullOf(points);
    CHECK(hull->faceCount() > 100);

    const BoxShape box(KM::vec3(0.2f));
    const CylinderShape cylinder(0.15f, 0.2f);
    const SphereShape sphere(0.2f);
    const KM::mat4 identity(::Math::Mat3::Identity(), KM::vec3(0.0f));
    u32 falseBox = 0;
    u32 falseCylinder = 0;
    u32 falseHull = 0;
    u32 falseSphere = 0;
    u32 tested = 0;
    for (u32 face = 0; face < hull->faceCount(); ++face)
    {
        KM::vec3 normal, corner;
        facePlane(*hull, face, normal, corner);
        const f32 plane = KM::dot(normal, corner);
        const KM::mat3 rotation = ::Math::Mat3::RotationX(0.7f) *
                                  ::Math::Mat3::RotationY(0.4f + static_cast<f32>(face)) *
                                  ::Math::Mat3::RotationZ(0.3f);
        ContactManifold manifold;

        KM::mat4 pose(rotation, KM::vec3(0.0f));
        const f32 boxReach = -KM::dot(normal, box.support(pose, -normal));
        pose = KM::mat4(rotation, corner + normal * (0.01f + boxReach));
        CHECK(KM::dot(normal, box.support(pose, -normal)) > plane);
        if (Narrowphase::collide(*hull, identity, box, pose, manifold))
            ++falseBox;

        pose = KM::mat4(rotation, KM::vec3(0.0f));
        const f32 cylinderReach = -KM::dot(normal, cylinder.support(pose, -normal));
        pose = KM::mat4(rotation, corner + normal * (0.01f + cylinderReach));
        if (Narrowphase::collide(*hull, identity, cylinder, pose, manifold))
            ++falseCylinder;
        if (Narrowphase::collide(*hull, identity, cylinder.hull(), pose, manifold))
            ++falseHull;

        pose = KM::mat4(rotation, corner + normal * 0.21f);
        if (Narrowphase::collide(*hull, identity, sphere, pose, manifold))
            ++falseSphere;
        ++tested;
    }
    CHECK(tested == hull->faceCount());
    CHECK(falseBox == 0);
    CHECK(falseCylinder == 0);
    CHECK(falseHull == 0);
    CHECK(falseSphere == 0);
    delete hull;
}

// Two prisms of 48 sides - 50 faces, 144 edges each - in random poses,
// against a full SAT written out here over every face normal and every pair
// of edges: whenever that finds a separating axis, so must collide().
void prismAgainstFullSat()
{
    const CylinderShape cylinder(0.5f, 0.5f, 48);
    const ConvexHullShape& prism = cylinder.hull();
    srand(3);
    u32 separated = 0;
    u32 falsePositives = 0;
    for (u32 trial = 0; trial < 300; ++trial)
    {
        const KM::mat4 poseA(::Math::Mat3::RotationX(random01() * 6.28f) *
                                 ::Math::Mat3::RotationY(random01() * 6.28f),
                             KM::vec3(0.0f));
        const KM::mat4 poseB(::Math::Mat3::RotationX(random01() * 6.28f) *
                                 ::Math::Mat3::RotationZ(random01() * 6.28f),
                             KM::vec3(random01() * 2 - 1, random01() * 2 - 1, random01() * 2 - 1) *
                                 1.1f);
        ct::Vector<KM::vec3> axes;
        for (const KM::mat4* pose : {&poseA, &poseB})
            for (u32 face = 0; face < prism.faceCount(); ++face)
            {
                KM::vec3 normal, corner;
                facePlane(prism, face, normal, corner);
                axes.push_back(KM::mat3(*pose) * normal);
            }
        // Parallel edges give the same axes; one of each keeps the 144 x 144
        // grid down to the 17 x 17 that differ.
        ct::Vector<KM::vec3> all;
        edgeVectors(prism, all);
        CHECK(all.size() == 144);
        ct::Vector<KM::vec3> edges;
        for (const KM::vec3& edge : all)
        {
            bool parallel = false;
            for (const KM::vec3& kept : edges)
                parallel = parallel || KM::length(KM::cross(KM::normalize(edge),
                                                            KM::normalize(kept))) < 1e-5f;
            if (!parallel)
                edges.push_back(edge);
        }
        for (const KM::vec3& edgeA : edges)
            for (const KM::vec3& edgeB : edges)
                axes.push_back(KM::cross(KM::mat3(poseA) * edgeA, KM::mat3(poseB) * edgeB));
        bool apart = false;
        for (const KM::vec3& raw : axes)
        {
            const f32 length = KM::length(raw);
            if (length < 1e-5f)
                continue;
            const KM::vec3 axis = raw / length;
            f32 minA, maxA, minB, maxB;
            prism.project(poseA, axis, minA, maxA);
            prism.project(poseB, axis, minB, maxB);
            if (maxA < minB - 1e-4f || maxB < minA - 1e-4f)
            {
                apart = true;
                break;
            }
        }
        if (!apart)
            continue;
        ++separated;
        ContactManifold manifold;
        if (Narrowphase::collide(prism, poseA, prism, poseB, manifold))
            ++falsePositives;
    }
    CHECK(separated > 50);
    CHECK(falsePositives == 0);
}

// A 100-sided prism standing on a box: its bottom face has 100 corners, so the contact patch surrounds the middle of it.
void bigFaceClippedWhole()
{
    const CylinderShape cylinder(0.5f, 0.25f, 100);
    const ConvexHullShape& prism = cylinder.hull();
    const BoxShape ground(KM::vec3(2.0f, 0.5f, 2.0f));
    const KM::mat4 groundPose(::Math::Mat3::Identity(), KM::vec3(0.0f, -0.5f, 0.0f));
    for (f32 turn : {0.0f, 1.0f, 2.0f, 3.0f})
    {
        const KM::mat4 prismPose(::Math::Mat3::RotationY(turn), KM::vec3(0.0f, 0.24f, 0.0f));
        ContactManifold manifold;
        CHECK(Narrowphase::collide(prism, prismPose, ground, groundPose, manifold));
        CHECK(manifold.count == 4);
        KM::vec3 middle(0.0f);
        for (u32 i = 0; i < manifold.count; ++i)
        {
            middle += manifold.points[i].position;
            CHECK_NEAR(manifold.points[i].penetration, 0.01f, 1e-4f);
            const KM::vec3 flat(manifold.points[i].position.x, 0.0f,
                                manifold.points[i].position.z);
            CHECK(KM::length(flat) > 0.45f);
        }
        middle /= static_cast<f32>(manifold.count);
        CHECK(sqrtf(middle.x * middle.x + middle.z * middle.z) < 0.2f);

        CHECK(Narrowphase::collide(ground, groundPose, prism, prismPose, manifold));
        CHECK(manifold.count == 4);
    }
}

}

int main()
{
    outsideEveryFace();
    prismAgainstFullSat();
    bigFaceClippedWhole();
    return TEST_RESULT();
}
