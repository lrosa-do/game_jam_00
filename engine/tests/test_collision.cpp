#include "Check.h"

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const float kHalfPi = 1.57079632679f;
const float kSkin = 0.01f;
const double kFrame = 1.0 / 60.0;

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

bool nearVec(const Math::Vec3& a, const Math::Vec3& b, float tolerance = 1e-4f)
{
    return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) && near(a.z, b.z, tolerance);
}

MeshId makeFloor(MeshStore& store, float angle = 0.0f)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float xs[4] = { -10.0f, -10.0f, 10.0f, 10.0f };
    const float zs[4] = { -10.0f, 10.0f, 10.0f, -10.0f };
    Math::Vec3 vertices[4];
    for (int i = 0; i < 4; ++i) vertices[i] = Math::Vec3(xs[i], -zs[i] * s, zs[i] * c);
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    CollisionMesh* mesh = new CollisionMesh();
    CHECK(mesh->build(vertices, 4, indices, 6));
    const MeshId id = store.create();
    CHECK(store.setCollision(id, mesh));
    return id;
}

void testMesh()
{
    CollisionMesh floor;
    const Math::Vec3 vertices[4] = { Math::Vec3(-10.0f, 0.0f, -10.0f),
        Math::Vec3(-10.0f, 0.0f, 10.0f), Math::Vec3(10.0f, 0.0f, 10.0f),
        Math::Vec3(10.0f, 0.0f, -10.0f) };
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    CHECK(!floor.valid());
    CHECK(floor.build(vertices, 4, indices, 6));
    CHECK(floor.valid());
    CHECK(floor.triangleCount() == 2);
    CHECK(nearVec(floor.bounds().min, Math::Vec3(-10.0f, 0.0f, -10.0f)));
    CHECK(nearVec(floor.bounds().max, Math::Vec3(10.0f, 0.0f, 10.0f)));

    Math::Vec3 a, b, c;
    floor.triangle(1, a, b, c);
    CHECK(nearVec(a, vertices[0]) && nearVec(b, vertices[2]) && nearVec(c, vertices[3]));

    SweepHit hit;
    CHECK(floor.sweepSphere(Math::Vec3(0.0f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f),
            hit));
    CHECK(near(hit.t, 0.5f));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));

    CHECK(!floor.sweepSphere(Math::Vec3(0.0f, -2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, 3.0f, 0.0f),
            hit));
    CHECK(!floor.sweepSphere(Math::Vec3(0.0f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -1.0f, 0.0f),
            hit));
    CHECK(!floor.sweepSphere(Math::Vec3(30.0f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f),
            hit));

    CHECK(floor.sweepSphere(Math::Vec3(0.0f, 0.25f, 0.0f), 0.5f, Math::Vec3(0.0f, -0.1f, 0.0f),
            hit));
    CHECK(hit.t < 0.0f);
    CHECK(near(hit.t, -0.5f));

    CHECK(floor.sweepSphere(Math::Vec3(0.0f, 0.5f, 0.0f), 1.0f, Math::Vec3(0.0f, -0.1f, 0.0f),
            hit));
    CHECK(near(hit.t, -0.5f));

    CHECK(floor.sweepSphere(Math::Vec3(10.3f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f),
            hit));
    CHECK(near(hit.t, 1.6f / 3.0f, 1e-3f));
    CHECK(nearVec(hit.normal, Math::Vec3(0.6f, 0.8f, 0.0f), 1e-3f));

    CHECK(floor.sweepEllipsoid(Math::Vec3(0.0f, 5.0f, 0.0f), Math::Vec3(0.5f, 1.0f, 0.5f),
            Math::Vec3(0.0f, -6.0f, 0.0f), hit));
    CHECK(near(hit.t, 4.0f / 6.0f));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(!floor.sweepEllipsoid(Math::Vec3(0.0f, 5.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.5f),
            Math::Vec3(0.0f, -6.0f, 0.0f), hit));

    CHECK(floor.raycast(Math::Ray(Math::Vec3(1.0f, 3.0f, 1.0f), Math::Vec3(0.0f, -1.0f, 0.0f)),
            10.0f, hit));
    CHECK(near(hit.t, 3.0f));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(!floor.raycast(Math::Ray(Math::Vec3(1.0f, 3.0f, 1.0f), Math::Vec3(0.0f, -1.0f, 0.0f)),
            2.0f, hit));
    CHECK(!floor.raycast(Math::Ray(Math::Vec3(1.0f, 3.0f, 1.0f), Math::Vec3(0.0f, 1.0f, 0.0f)),
            10.0f, hit));

    CollisionMesh scaled;
    CHECK(scaled.build(vertices, 4, indices, 6, Math::Vec3(2.0f, 1.0f, 3.0f)));
    CHECK(nearVec(scaled.bounds().min, Math::Vec3(-20.0f, 0.0f, -30.0f)));
    CHECK(nearVec(scaled.bounds().max, Math::Vec3(20.0f, 0.0f, 30.0f)));

    CollisionMesh mirrored;
    CHECK(mirrored.build(vertices, 4, indices, 6, Math::Vec3(-1.0f, 1.0f, 1.0f)));
    CHECK(mirrored.sweepSphere(Math::Vec3(0.0f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f),
            hit));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));

    const std::uint32_t outOfRange[3] = { 0, 1, 7 };
    CollisionMesh invalid;
    CHECK(invalid.build(vertices, 4, indices, 6));
    CHECK(!invalid.build(vertices, 4, outOfRange, 3));
    CHECK(!invalid.valid());
    CHECK(invalid.triangleCount() == 0);
    CHECK(!invalid.build(vertices, 4, indices, 5));
    CHECK(!invalid.build(vertices, 4, indices, 0));
    CHECK(!invalid.build(nullptr, 4, indices, 6));
    CHECK(!invalid.sweepSphere(Math::Vec3(0.0f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f),
            hit));

    SurfaceData surface;
    surface.positions.push_back(vertices[0]);
    surface.positions.push_back(vertices[1]);
    surface.positions.push_back(vertices[2]);
    CollisionMesh fromSurface;
    CHECK(fromSurface.build(surface));
    CHECK(fromSurface.triangleCount() == 1);
    surface.positions.push_back(vertices[3]);
    CHECK(!fromSurface.build(surface));
    surface.indices.assign(indices, indices + 6);
    CHECK(fromSurface.build(surface));
    CHECK(fromSurface.triangleCount() == 2);

    CollisionMesh pieces;
    pieces.append(vertices, 4, indices, 6);
    CHECK(!pieces.valid());
    CHECK(!pieces.sweepSphere(Math::Vec3(0.0f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f),
            hit));
    CHECK(!pieces.raycast(Math::Ray(Math::Vec3(0.0f, 2.0f, 0.0f), Math::Vec3(0.0f, -1.0f, 0.0f)),
            10.0f, hit));
    CHECK(pieces.append(surface, Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(pieces.finish());
    CHECK(pieces.valid());
    CHECK(pieces.triangleCount() == 4);
    pieces.triangle(2, a, b, c);
    CHECK(nearVec(a, vertices[0]) && nearVec(b, vertices[1]) && nearVec(c, vertices[2]));
    CHECK(pieces.sweepSphere(Math::Vec3(0.0f, 2.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f),
            hit));
    SurfaceData lines;
    lines.topology = prisma::Topology::Lines;
    lines.positions.push_back(vertices[0]);
    lines.positions.push_back(vertices[1]);
    CHECK(!pieces.append(lines, Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(pieces.triangleCount() == 4);
    CHECK(pieces.valid());

    CollisionMesh box;
    CHECK(box.buildBox(Math::Vec3(-1.0f, -1.0f, -1.0f), Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(box.triangleCount() == 12);
    CHECK(nearVec(box.bounds().min, Math::Vec3(-1.0f, -1.0f, -1.0f)));
    CHECK(nearVec(box.bounds().max, Math::Vec3(1.0f, 1.0f, 1.0f)));

    const Math::Vec3 centre(0.0f, 0.0f, 0.0f);
    for (std::uint32_t i = 0; i < 12; ++i)
    {
        box.triangle(i, a, b, c);
        const Math::Vec3 normal = (b - a).Cross(c - a).Normalized();
        const Math::Vec3 middle = (a + b + c) * (1.0f / 3.0f);
        CHECK(normal.Dot(middle - centre) > 0.5f);
    }

    CHECK(box.sweepSphere(Math::Vec3(3.0f, 0.0f, 0.0f), 0.5f, Math::Vec3(-3.0f, 0.0f, 0.0f), hit));
    CHECK(near(hit.t, 0.5f));
    CHECK(nearVec(hit.normal, Math::Vec3(1.0f, 0.0f, 0.0f)));
    CHECK(box.sweepSphere(Math::Vec3(0.0f, 0.0f, -3.0f), 0.5f, Math::Vec3(0.0f, 0.0f, 3.0f), hit));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 0.0f, -1.0f)));
    CHECK(box.sweepSphere(Math::Vec3(0.0f, 3.0f, 0.0f), 0.5f, Math::Vec3(0.0f, -3.0f, 0.0f), hit));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(!box.sweepSphere(Math::Vec3(0.0f, 0.0f, 0.0f), 0.25f, Math::Vec3(1.0f, 0.0f, 0.0f), hit));
    CHECK(!box.buildBox(Math::Vec3(1.0f, 1.0f, 1.0f), Math::Vec3(1.0f, 2.0f, 2.0f)));
    CHECK(!box.valid());
}

struct Scene
{
    MeshStore store;
    SceneTree tree;
    Node3D holder;
    Node3D* floor = nullptr;
    Node3D* wall = nullptr;
    Node3D* player = nullptr;

    Scene()
            : store(nullptr)
    {
        tree.initialize();
    }

    void addFloor(MeshId mesh)
    {
        floor = holder.createChild<Node3D>();
        floor->setCollisionMesh(mesh);
        floor->setCollisionType(2);
        floor->reparent(tree.getRoot());
    }

    void addWall()
    {
        wall = holder.createChild<Node3D>();
        wall->setCollisionBox(Math::Vec3(-5.0f, 0.0f, -0.25f), Math::Vec3(5.0f, 4.0f, 0.25f));
        wall->setCollisionType(3);
        wall->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
        wall->rotateY(kHalfPi);
        wall->reparent(tree.getRoot());
    }

    void addPlayer(const Math::Vec3& position, float radius = 0.5f)
    {
        player = holder.createChild<Node3D>();
        player->setCollisionRadius(radius);
        player->setCollisionType(1);
        player->setPosition(position);
        player->reparent(tree.getRoot());
    }

    void step()
    {
        tree.process(kFrame);
    }
};

void testEllipsoidFloor()
{
    Scene scene;
    scene.addFloor(makeFloor(scene.store));
    scene.addPlayer(Math::Vec3(0.0f, 3.0f, 0.0f));
    scene.player->setCollisionRadius(0.5f, 1.0f);
    CollisionWorld& world = scene.tree.getCollisionWorld();
    world.setPair(1, 2, CollisionMethod::Polygon, CollisionResponse::Slide);

    scene.step();
    CHECK(scene.player->countCollisions() == 0);
    scene.player->setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));
    scene.step();
    CHECK(near(scene.player->getGlobalPosition().y, 1.0f + kSkin, 0.002f));
    CHECK(near(scene.player->getGlobalPosition().x, 0.0f));
    CHECK(scene.player->countCollisions() == 1);
    CHECK(scene.player->collidedWith(2) == scene.floor);
    CHECK(nearVec(scene.player->getCollision(0).normal, Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(scene.player->getCollision(0).point.y, 0.0f, 0.002f));
}

void testCapsuleFloor()
{
    Scene scene;
    scene.addFloor(makeFloor(scene.store));
    scene.addPlayer(Math::Vec3(0.0f, 3.0f, 0.0f));
    scene.player->setCollisionCapsule(0.5f, 2.0f);
    CHECK(nearVec(scene.player->getCollisionRadii(), Math::Vec3(0.5f, 1.5f, 0.5f)));
    CHECK(scene.player->getCollisionShape() == CollisionShapeType::Ellipsoid);
    scene.player->setCollisionCapsule(0.5f, 1.0f);
    CHECK(nearVec(scene.player->getCollisionRadii(), Math::Vec3(0.5f, 1.0f, 0.5f)));
    scene.player->setCollisionCapsule(0.5f, 2.0f);
    CollisionWorld& world = scene.tree.getCollisionWorld();
    world.setPair(1, 2, CollisionMethod::Polygon, CollisionResponse::Slide);

    scene.step();
    CHECK(scene.player->countCollisions() == 0);
    scene.player->setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));
    scene.step();
    CHECK(near(scene.player->getGlobalPosition().y, 1.5f + kSkin, 0.002f));
    CHECK(scene.player->countCollisions() == 1);
    CHECK(scene.player->collidedWith(2) == scene.floor);
}

void testSlideAlongWall()
{
    Scene scene;
    scene.addFloor(makeFloor(scene.store));
    scene.addWall();
    scene.addPlayer(Math::Vec3(0.0f, 1.0f, 0.0f));
    CollisionWorld& world = scene.tree.getCollisionWorld();
    world.setPair(1, 2, CollisionMethod::Polygon, CollisionResponse::Slide);
    world.setPair(1, 3, CollisionMethod::Box, CollisionResponse::Slide);

    scene.step();
    for (int i = 0; i < 12; ++i)
    {
        scene.player->translate(Math::Vec3(0.5f, 0.0f, 0.25f));
        scene.step();
    }

    const Math::Vec3 position = scene.player->getGlobalPosition();
    CHECK(near(position.x, 5.0f - 0.25f - 0.5f - kSkin, 0.02f));
    CHECK(near(position.z, 3.0f, 0.02f));
    CHECK(near(position.y, 1.0f, 0.02f));
    CHECK(scene.player->countCollisions() == 1);
    CHECK(scene.player->collidedWith(3) == scene.wall);
    CHECK(scene.player->collidedWith(2) == nullptr);
    CHECK(scene.player->collidedWith(9) == nullptr);

    const CollisionHit& hit = scene.player->getCollision(0);
    CHECK(hit.node == scene.wall);
    CHECK(hit.normal.x < -0.99f);
    CHECK(near(hit.point.x, 4.75f, 0.02f));
    CHECK(near(hit.point.y, 1.0f, 0.02f));
    CHECK(hit.time > 0.0f && hit.time < 1.0f);
    CHECK(scene.player->getCollision(7).node == nullptr);

    scene.step();
    CHECK(scene.player->countCollisions() == 0);
    CHECK(scene.player->collidedWith(3) == nullptr);
}

void testStop()
{
    Scene scene;
    scene.addFloor(makeFloor(scene.store));
    scene.addWall();
    scene.addPlayer(Math::Vec3(0.0f, 1.0f, 0.0f));
    CollisionWorld& world = scene.tree.getCollisionWorld();
    world.setPair(1, 2, CollisionMethod::Polygon, CollisionResponse::Stop);
    world.setPair(1, 3, CollisionMethod::Box, CollisionResponse::Stop);

    scene.step();
    float previousZ = 0.0f;
    for (int i = 0; i < 12; ++i)
    {
        scene.player->translate(Math::Vec3(0.5f, 0.0f, 0.25f));
        scene.step();
        if (i == 10) previousZ = scene.player->getGlobalPosition().z;
    }

    const Math::Vec3 position = scene.player->getGlobalPosition();
    CHECK(near(position.x, 4.24f, 0.02f));
    CHECK(near(position.z, 2.13f, 0.05f));
    CHECK(near(position.z, previousZ, 0.01f));
    CHECK(scene.player->countCollisions() == 1);
}

void testSlideNoSlope()
{
    for (int mode = 0; mode < 2; ++mode)
    {
        Scene scene;
        scene.addFloor(makeFloor(scene.store, 30.0f * 3.14159265f / 180.0f));
        scene.addPlayer(Math::Vec3(0.0f, 1.0f, 0.0f));
        CollisionWorld& world = scene.tree.getCollisionWorld();
        world.setPair(1, 2, CollisionMethod::Polygon,
                mode == 0 ? CollisionResponse::Slide : CollisionResponse::SlideNoSlope);

        scene.step();
        for (int i = 0; i < 20; ++i)
        {
            scene.player->translate(Math::Vec3(0.0f, -0.2f, 0.0f));
            scene.step();
        }

        const Math::Vec3 position = scene.player->getGlobalPosition();
        CHECK(near(position.x, 0.0f, 0.001f));
        if (mode == 0) CHECK(position.z > 0.5f);
        else
        {
            CHECK(near(position.z, 0.0f, 0.01f));
            CHECK(near(position.y,
                    0.5f / cosf(30.0f * 3.14159265f / 180.0f) +
                            kSkin / cosf(30.0f * 3.14159265f / 180.0f),
                    0.01f));
            CHECK(scene.player->countCollisions() == 1);
            CHECK(scene.player->getCollision(0).normal.y > 0.8f);
        }
    }
}

void testSphereMethod()
{
    Scene scene;
    scene.addPlayer(Math::Vec3(-5.0f, 0.0f, 0.0f), 0.5f);
    Node3D* other = scene.holder.createChild<Node3D>();
    other->setCollisionRadius(1.0f);
    other->setCollisionType(4);
    other->reparent(scene.tree.getRoot());
    scene.tree.getCollisionWorld().setPair(1, 4, CollisionMethod::Sphere, CollisionResponse::Stop);

    scene.step();
    scene.player->translate(Math::Vec3(4.0f, 0.0f, 0.0f));
    scene.step();

    const Math::Vec3 position = scene.player->getGlobalPosition();
    CHECK(near(position.x, -1.5f - kSkin, 0.005f));
    CHECK(near((position - other->getGlobalPosition()).Length(), 1.5f + kSkin, 0.005f));
    CHECK(scene.player->countCollisions() == 1);
    CHECK(scene.player->collidedWith(4) == other);
    CHECK(nearVec(scene.player->getCollision(0).normal, Math::Vec3(-1.0f, 0.0f, 0.0f)));
    CHECK(nearVec(scene.player->getCollision(0).point, Math::Vec3(-1.0f, 0.0f, 0.0f)));

    scene.player->setPosition(Math::Vec3(-5.0f, 0.0f, 0.0f));
    scene.player->resetCollision();
    scene.step();
    scene.player->translate(Math::Vec3(0.0f, 0.0f, 3.0f));
    scene.step();
    CHECK(scene.player->countCollisions() == 0);
    CHECK(near(scene.player->getGlobalPosition().z, 3.0f));
}

void testBookkeeping()
{
    Scene scene;
    CollisionWorld& world = scene.tree.getCollisionWorld();
    CHECK(CollisionWorld::singleton() == &world);
    CHECK(world.nodeCount() == 0);
    CHECK(world.pairCount() == 0);

    world.setPair(1, 2, CollisionMethod::Polygon, CollisionResponse::Slide);
    world.setPair(1, 3, CollisionMethod::Box, CollisionResponse::Slide);
    CHECK(world.pairCount() == 2);
    world.setPair(1, 2, CollisionMethod::Polygon, CollisionResponse::Stop);
    CHECK(world.pairCount() == 2);
    CHECK(world.pair(0).source == 1 && world.pair(0).destination == 2);
    CHECK(world.pair(0).response == CollisionResponse::Stop);
    CHECK(world.pair(1).method == CollisionMethod::Box);
    world.setPair(1, 2, CollisionMethod::Polygon, CollisionResponse::Slide);

    scene.addFloor(makeFloor(scene.store));
    scene.addWall();
    scene.addPlayer(Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(world.nodeCount() == 3);

    Node3D* twin = scene.holder.createChild<Node3D>();
    twin->setCollisionRadius(0.5f);
    twin->setCollisionType(1);
    twin->setPosition(Math::Vec3(0.0f, 1.0f, 2.0f));
    twin->reparent(scene.tree.getRoot());
    CHECK(world.nodeCount() == 4);

    scene.step();
    CHECK(world.sweepCount() == 0);

    scene.player->setPosition(Math::Vec3(8.0f, 1.0f, 0.0f));
    scene.player->resetCollision();
    twin->setPosition(Math::Vec3(8.0f, 1.0f, 2.0f));
    scene.step();
    CHECK(near(scene.player->getGlobalPosition().x, 8.0f));
    CHECK(scene.player->countCollisions() == 0);
    CHECK(near(twin->getGlobalPosition().x, 5.0f - 0.25f - 0.5f - kSkin, 0.02f));
    CHECK(twin->countCollisions() == 1);
    CHECK(twin->collidedWith(3) == scene.wall);
    CHECK(world.sweepCount() > 0);

    twin->resetCollision();
    CHECK(twin->countCollisions() == 0);

    scene.step();
    CHECK(world.sweepCount() == 0);
    CHECK(scene.player->countCollisions() == 0);

    twin->reparent(&scene.holder);
    CHECK(world.nodeCount() == 3);
    twin->setPosition(Math::Vec3(20.0f, 1.0f, 2.0f));
    scene.step();
    CHECK(nearVec(twin->getPosition(), Math::Vec3(20.0f, 1.0f, 2.0f)));
    twin->reparent(scene.tree.getRoot());
    CHECK(world.nodeCount() == 4);
    scene.step();
    CHECK(nearVec(twin->getPosition(), Math::Vec3(20.0f, 1.0f, 2.0f)));
    CHECK(twin->countCollisions() == 0);
    twin->reparent(&scene.holder);
    twin->queueFree();
    scene.step();
    CHECK(world.nodeCount() == 3);

    scene.player->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
    scene.player->resetCollision();
    scene.step();
    scene.player->translate(Math::Vec3(10.0f, 0.0f, 0.0f));
    scene.step();
    CHECK(scene.player->countCollisions() == 1);
    scene.wall->queueFree();
    scene.step();
    scene.wall = nullptr;
    CHECK(world.nodeCount() == 2);
    CHECK(scene.player->countCollisions() == 0);
    CHECK(scene.player->collidedWith(3) == nullptr);

    scene.floor->setCollisionType(0);
    CHECK(world.nodeCount() == 1);
    CHECK(scene.floor->getCollisionType() == 0);
    scene.floor->setCollisionType(2);
    CHECK(world.nodeCount() == 2);

    scene.player->setCollisionType(0);
    CHECK(world.nodeCount() == 1);
    scene.player->setCollisionType(1);
    CHECK(world.nodeCount() == 2);

    world.clearPairs();
    CHECK(world.pairCount() == 0);
    CHECK(near(world.skinWidth(), kSkin));
    CHECK(near(world.slopeLimit(), 45.0f));
    CHECK(nearVec(world.upDirection(), Math::Vec3(0.0f, 1.0f, 0.0f)));
}

void testNodeDefaults()
{
    MeshStore store(nullptr);
    Node3D node;
    CHECK(node.getCollisionType() == 0);
    CHECK(node.getCollisionShape() == CollisionShapeType::None);
    CHECK(node.countCollisions() == 0);
    CHECK(node.collidedWith(1) == nullptr);
    CHECK(!node.getCollisionMesh().valid());
    node.setCollisionType(5);
    CHECK(node.getCollisionType() == 5);
    node.setCollisionRadius(0.5f, 2.0f);
    CHECK(node.getCollisionShape() == CollisionShapeType::Ellipsoid);
    CHECK(nearVec(node.getCollisionRadii(), Math::Vec3(0.5f, 2.0f, 0.5f)));
    node.setCollisionBox(Math::Vec3(1.0f, 1.0f, 1.0f), Math::Vec3(-1.0f, -1.0f, -1.0f));
    CHECK(node.getCollisionShape() == CollisionShapeType::Box);
    const MeshId floorId = makeFloor(store);
    node.setCollisionMesh(floorId);
    CHECK(node.getCollisionShape() == CollisionShapeType::Mesh);
    CHECK(node.getCollisionMesh() == floorId);
    CHECK(store.collision(node.getCollisionMesh())->triangleCount() == 2);
    node.setCollisionMesh(MeshId());
    CHECK(node.getCollisionShape() == CollisionShapeType::None);
}

} // namespace

void testRaycast()
{
    Scene scene;
    CollisionWorld& world = scene.tree.getCollisionWorld();
    const Math::Ray forward(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -1.0f));
    CollisionHit hit;
    CHECK(!world.raycast(forward, 100.0f, hit));

    Node3D* box = scene.holder.createChild<Node3D>();
    box->setCollisionBox(Math::Vec3(-0.5f, -0.5f, -0.5f), Math::Vec3(0.5f, 0.5f, 0.5f));
    box->setCollisionType(3);
    box->setPosition(Math::Vec3(0.0f, 0.0f, -5.0f));
    box->reparent(scene.tree.getRoot());

    CHECK(world.raycast(forward, 100.0f, hit));
    CHECK(hit.node == box);
    CHECK(near(hit.time, 4.5f));
    CHECK(nearVec(hit.point, Math::Vec3(0.0f, 0.0f, -4.5f)));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 0.0f, 1.0f)));
    CHECK(!world.raycast(forward, 4.0f, hit));
    CHECK(!world.raycast(Math::Ray(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f)),
            100.0f, hit));
    CHECK(!world.raycast(Math::Ray(Math::Vec3(2.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -1.0f)),
            100.0f, hit));

    box->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(world.raycast(forward, 100.0f, hit));
    CHECK(near(hit.time, 4.0f));
    CHECK(nearVec(hit.point, Math::Vec3(0.0f, 0.0f, -4.0f)));
    CHECK(world.raycast(Math::Ray(Math::Vec3(0.9f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -1.0f)),
            100.0f, hit));
    box->setScale(Math::Vec3(1.0f, 1.0f, 1.0f));

    box->rotateY(kHalfPi * 0.5f);
    CHECK(world.raycast(forward, 100.0f, hit));
    CHECK(near(hit.time, 5.0f - 0.70710678f));
    CHECK(near(fabsf(hit.normal.x), 0.70710678f) && near(hit.normal.z, 0.70710678f));
    box->setRotation(Math::Vec3(0.0f, 0.0f, 0.0f));

    Node3D* ball = scene.holder.createChild<Node3D>();
    ball->setCollisionRadius(1.0f);
    ball->setCollisionType(1);
    ball->setPosition(Math::Vec3(0.0f, 0.0f, -2.0f));
    ball->reparent(scene.tree.getRoot());
    CHECK(world.raycast(forward, 100.0f, hit));
    CHECK(hit.node == ball);
    CHECK(near(hit.time, 1.0f));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 0.0f, 1.0f)));
    CHECK(world.raycast(forward, 100.0f, hit, 3));
    CHECK(hit.node == box && near(hit.time, 4.5f));
    CHECK(!world.raycast(forward, 100.0f, hit, 7));

    ball->setCollisionRadius(0.5f, 2.0f);
    const Math::Ray down(Math::Vec3(0.0f, 10.0f, -2.0f), Math::Vec3(0.0f, -1.0f, 0.0f));
    CHECK(world.raycast(down, 100.0f, hit));
    CHECK(hit.node == ball && near(hit.time, 8.0f));
    CHECK(nearVec(hit.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));
    const Math::Ray inside(Math::Vec3(0.0f, 0.0f, -2.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(world.raycast(inside, 100.0f, hit));
    CHECK(near(hit.time, 2.0f));

    scene.addFloor(makeFloor(scene.store));
    scene.floor->setPosition(Math::Vec3(0.0f, -3.0f, 0.0f));
    const Math::Ray toFloor(Math::Vec3(4.0f, 5.0f, 4.0f), Math::Vec3(0.0f, -1.0f, 0.0f));
    CHECK(world.raycast(toFloor, 100.0f, hit));
    CHECK(hit.node == scene.floor);
    CHECK(near(hit.time, 8.0f));
    CHECK(nearVec(hit.point, Math::Vec3(4.0f, -3.0f, 4.0f)));
    CHECK(near(fabsf(hit.normal.y), 1.0f));

    ball->queueFree();
    scene.tree.process(kFrame);
    CHECK(world.raycast(forward, 100.0f, hit));
    CHECK(hit.node == box);
}

int main()
{
    testMesh();
    testNodeDefaults();
    testRaycast();
    testEllipsoidFloor();
    testCapsuleFloor();
    testSlideAlongWall();
    testStop();
    testSlideNoSlope();
    testSphereMethod();
    testBookkeeping();
    CHECK(CollisionWorld::singleton() == nullptr);

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
