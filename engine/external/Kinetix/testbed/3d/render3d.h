#pragma once

// render3d.h - solid shaded drawing for the 3D scenes, with the wireframe as an overlay.

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <geometry/ConvexHullComputer.h>

namespace testbed
{

struct Shade
{
    float r, g, b;
};

void setupLighting3D();

void drawSolidBox(const kinetix::Math::mat4& transform, const kinetix::Math::vec3& halfExtents,
                  Shade shade);
void drawSolidSphere(const kinetix::Math::vec3& centre, float radius, Shade shade);
// Voronoi debris and other ConvexHullShape bodies: each face's edge ring as a triangle fan;
// not meant for hulls with hundreds of faces.
void drawSolidConvexHull(const kinetix::Math::mat4& transform,
                         const kinetix::Physics::ConvexHullShape& hull, Shade shade);

void drawGround(float y, float extent, float spacing);

Shade shadeForBody(const kinetix::Physics::RigidBody& body);

// Every body, drawn solid; falls back to the shape's debugDraw() for anything but a box or sphere.
void drawWorldSolid(const kinetix::Physics::PhysicsWorld& world,
                    const ct::Vector<kinetix::Physics::RigidBody*>& bodies,
                    const ct::Vector<kinetix::Physics::CollisionShape*>& shapes);

} // namespace testbed
