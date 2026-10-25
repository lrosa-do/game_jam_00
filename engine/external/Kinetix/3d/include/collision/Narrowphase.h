#ifndef KINETIX_PHYSICS_COLLISION_NARROWPHASE_H
#define KINETIX_PHYSICS_COLLISION_NARROWPHASE_H

#include "collision/CollisionShape.h"

#include <ct/vector.hpp>

namespace kinetix::Physics
{

// One point where two shapes touch. The impulses persist between steps so the
// solver can warm start from what it needed last time.
struct ContactPoint
{
    Math::vec3 position{0.0f}; // world, midway between the two surfaces
    f32 penetration = 0.0f;   // positive when overlapping
    // The two surface points, each in its own body's frame. The position pass
    // re-measures the overlap from these every iteration, after the other
    // contacts and joints have moved the bodies, rather than trusting the
    // narrowphase's number (several constraints on one body over-correct).
    Math::vec3 localA{0.0f};
    Math::vec3 localB{0.0f};
    f32 normalImpulse = 0.0f;
    f32 tangentImpulse[2] = {0.0f, 0.0f};
    // Separation speed the solver aims for, from the approach speed measured
    // before any impulse. Not persistent: ContactSolver::solve() recomputes it
    // every step; zero for everything that is not bouncing.
    f32 velocityBias = 0.0f;
    // Set by ContactSolver::solve() for the velocity iterations: the arms and the
    // velocity a unit impulse gives along each direction do not change until the
    // bodies integrate.
    Math::vec3 armA{0.0f};
    Math::vec3 armB{0.0f};
    f32 normalK = 0.0f;
    f32 tangentK[2] = {0.0f, 0.0f};
};

// The at-most-four points of one contact patch, plus the shared normal. Four
// is what a box face against a box face needs.
struct ContactManifold
{
    static constexpr u32 MaxPoints = 4;

    // Points from A towards B: pushing B along it separates them.
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    // Perpendicular to the normal, for the two friction directions.
    Math::vec3 tangent[2] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f)};
    ContactPoint points[MaxPoints];
    u32 count = 0;

    void buildTangents();
};

struct ShapeRayHit
{
    f32 distance = 0.0f;
    Math::vec3 point{0.0f};
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    // Index of the triangle hit in a trimesh, -1 for every other shape.
    s32 triangle = -1;
};

// Separating Axis Test and contact generation, one static entry point per
// shape pair. Nothing here touches a RigidBody, so it can be tested without a
// world.
class Narrowphase
{
public:
    // `margin` reports shapes that are close but not yet touching, with a
    // NEGATIVE penetration saying how far apart they are. Zero is plain
    // overlap-only detection.
    //
    // The position pass pushes a body just clear of the surface it landed on;
    // with overlap-only detection the contact then vanishes until gravity
    // pulls it back, taking the pair's accumulated impulses and enter/stay/exit
    // state with it. The solver ignores a negative penetration for position
    // and applies no impulse while the bodies separate, so a speculative
    // contact keeps the pair alive across the gap.
    // The axis a pair was last found apart along. Given back, it is tested first:
    // still apart along it, and nothing else runs.
    struct SeparatingAxis
    {
        Math::vec3 axis{0.0f};
        bool known = false;
    };
    static bool collide(const CollisionShape& a, const Math::mat4& transformA,
                        const CollisionShape& b, const Math::mat4& transformB,
                        ContactManifold& out, f32 margin = 0.0f,
                        SeparatingAxis* separation = nullptr);
    static bool collideShapes(const CollisionShape& a, const Math::mat4& transformA,
                              const CollisionShape& b, const Math::mat4& transformB,
                              ContactManifold& out, f32 margin, Math::vec3* separatingAxis);

    static bool sphereSphere(const SphereShape& a, const Math::mat4& transformA,
                             const SphereShape& b, const Math::mat4& transformB,
                             ContactManifold& out, f32 margin = 0.0f);
    static bool sphereBox(const SphereShape& a, const Math::mat4& transformA, const BoxShape& b,
                          const Math::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);
    static bool boxBox(const BoxShape& a, const Math::mat4& transformA, const BoxShape& b,
                       const Math::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);

    // A capsule against a sphere is the sphere case at the closest point on
    // its segment; against another capsule, the same with both segments.
    // Exact, not approximations.
    static bool capsuleSphere(const CapsuleShape& a, const Math::mat4& transformA,
                              const SphereShape& b, const Math::mat4& transformB,
                              ContactManifold& out, f32 margin = 0.0f);
    static bool capsuleCapsule(const CapsuleShape& a, const Math::mat4& transformA,
                               const CapsuleShape& b, const Math::mat4& transformB,
                               ContactManifold& out, f32 margin = 0.0f);
    // While the capsule's segment is outside the box, the exact closest pair
    // between segment and box (an end cap against a corner or edge, which no
    // SAT axis sees). Once the segment reaches into the box, SAT over the
    // box's faces, the capsule's axis and their cross products. A capsule
    // lying along a face gets TWO points, its segment clipped to the face, so
    // it does not pivot on one.
    static bool capsuleBox(const CapsuleShape& a, const Math::mat4& transformA, const BoxShape& b,
                           const Math::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);
    static bool convexPlane(const CollisionShape& a, const Math::mat4& transformA,
                            const PlaneShape& b, const Math::mat4& transformB,
                            ContactManifold& out, f32 margin = 0.0f);

    // The face+edge SAT of boxBox() generalized to a ConvexHullShape: its
    // face normals stand in for the box's three, and every one of its edge
    // directions where a cross-product axis is needed. Against a sphere, and a
    // capsule whose segment is still outside the hull, the exact closest point
    // instead, as sphereBox() and capsuleBox() do. A hull with no volume
    // collides as a point, segment or flat polygon.
    static bool convexHullSphere(const ConvexHullShape& a, const Math::mat4& transformA,
                                 const SphereShape& b, const Math::mat4& transformB,
                                 ContactManifold& out, f32 margin = 0.0f);
    static bool convexHullBox(const ConvexHullShape& a, const Math::mat4& transformA,
                              const BoxShape& b, const Math::mat4& transformB, ContactManifold& out,
                              f32 margin = 0.0f);
    static bool convexHullCapsule(const ConvexHullShape& a, const Math::mat4& transformA,
                                  const CapsuleShape& b, const Math::mat4& transformB,
                                  ContactManifold& out, f32 margin = 0.0f);
    static bool convexHullConvexHull(const ConvexHullShape& a, const Math::mat4& transformA,
                                     const ConvexHullShape& b, const Math::mat4& transformB,
                                     ContactManifold& out, f32 margin = 0.0f,
                                     Math::vec3* separatingAxis = nullptr);

    // Triangle cases, normal pointing from the convex towards the triangle.
    static bool sphereTriangle(const SphereShape& a, const Math::mat4& transformA,
                               const TriangleShape& b, const Math::mat4& transformB,
                               ContactManifold& out, f32 margin = 0.0f);
    static bool boxTriangle(const BoxShape& a, const Math::mat4& transformA,
                            const TriangleShape& b, const Math::mat4& transformB,
                            ContactManifold& out, f32 margin = 0.0f);
    static bool capsuleTriangle(const CapsuleShape& a, const Math::mat4& transformA,
                                const TriangleShape& b, const Math::mat4& transformB,
                                ContactManifold& out, f32 margin = 0.0f);
    // The hull's face normals, the triangle's, and the cross products of the
    // hull's edge directions with the triangle's three; then the same
    // reference-face clipping as convexHullBox().
    static bool convexHullTriangle(const ConvexHullShape& a, const Math::mat4& transformA,
                                   const TriangleShape& b, const Math::mat4& transformB,
                                   ContactManifold& out, f32 margin = 0.0f);

    // One manifold PER TOUCHING TRIANGLE, appended to `out`: a manifold has a
    // single normal, and a box wedged into a corner needs one per face it
    // rests on, not an average belonging to neither.
    static bool convexTrimesh(const CollisionShape& convex, const Math::mat4& convexTransform,
                              const TrimeshShape& mesh, const Math::mat4& meshTransform,
                              ct::Vector<ContactManifold>& out, f32 margin = 0.0f,
                              const AABB* convexBounds = nullptr);

    static bool raycast(const CollisionShape& shape, const Math::mat4& transform, const Ray& ray,
                        f32 maxDistance, ShapeRayHit& hit);

    static bool overlapSphere(const CollisionShape& shape, const Math::mat4& transform,
                              const Math::vec3& centre, f32 radius);
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_COLLISION_NARROWPHASE_H
