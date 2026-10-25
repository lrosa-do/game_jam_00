#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const double kStep = 1.0 / 60.0;

bool near(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

void run(SceneTree& tree, double seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i) tree.physicsProcess(kStep);
}

StaticBody3D* makeGround(Node* root)
{
    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* shape = ground->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(40.0f, 1.0f, 40.0f));
    ground->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
    return ground;
}

RigidBody3D* makeBall(Node* root, const Math::Vec3& position, float radius = 0.5f)
{
    RigidBody3D* ball = root->createChild<RigidBody3D>();
    CollisionShape3D* shape = ball->createChild<CollisionShape3D>();
    shape->setSphere(radius);
    ball->setPosition(position);
    return ball;
}

void testFallAndContacts()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    StaticBody3D* ground = makeGround(root);
    RigidBody3D* ball = makeBall(root, Math::Vec3(0.0f, 5.0f, 0.0f));
    ball->setContactMonitor(true, 4);

    int entered = 0;
    PhysicsBody3D* enteredWith = nullptr;
    int slept = 0;
    ball->bodyEntered = [&](PhysicsBody3D* other, const PhysicsContact&)
    {
        ++entered;
        enteredWith = other;
    };
    ball->sleepingChanged = [&](bool sleeping)
    {
        if (sleeping) ++slept;
    };

    CHECK(ground->isSimulated());
    CHECK(ball->isSimulated());
    CHECK(ball->getMode() == PhysicsBodyMode::Rigid);
    CHECK(ground->getMode() == PhysicsBodyMode::Static);
    CHECK(ball->isInterpolated());
    CHECK(near(ball->getGlobalPosition().y, 5.0f, 1e-4f));

    run(tree, 3.0);

    const Math::Vec3 position = ball->getGlobalPosition();
    printf("  fall: ball at %.3f %.3f %.3f, sleeping %d, entered %d\n", position.x, position.y,
            position.z, ball->isSleeping() ? 1 : 0, entered);
    CHECK(near(position.y, 0.5f, 0.03f));
    CHECK(near(position.x, 0.0f, 0.02f));
    CHECK(ball->isSleeping());
    CHECK(slept >= 1);
    CHECK(entered >= 1);
    CHECK(enteredWith == ground);
    CHECK(PhysicsBody3D::fromBody(ball->getBody()) == ball);

    tree.finalize();
}

void testTeleportAndForces()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);
    RigidBody3D* ball = makeBall(root, Math::Vec3(0.0f, 0.5f, 0.0f));
    run(tree, 1.0);
    CHECK(ball->isSleeping());

    ball->setGlobalPosition(Math::Vec3(3.0f, 4.0f, 0.0f));
    tree.physicsProcess(kStep);
    CHECK(near(server.bodyGetTransform(ball->getBody()).col3.x, 3.0f, 0.02f));
    CHECK(!ball->isSleeping());
    run(tree, 3.0);
    CHECK(near(ball->getGlobalPosition().x, 3.0f, 0.05f));
    CHECK(near(ball->getGlobalPosition().y, 0.5f, 0.03f));

    ball->setLinearVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
    ball->setMass(2.0f);
    CHECK(near(ball->getMass(), 2.0f, 1e-4f));
    ball->applyCentralImpulse(Math::Vec3(4.0f, 0.0f, 0.0f));
    CHECK(near(ball->getLinearVelocity().x, 2.0f, 1e-3f));
    ball->setLockRotation(true);
    CHECK(ball->isLockRotation());
    ball->setConstantForce(Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(near(ball->getConstantForce().y, 1.0f, 1e-4f));

    tree.finalize();
}

void testKinematicPlatform()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    AnimatableBody3D* platform = root->createChild<AnimatableBody3D>();
    CollisionShape3D* top = platform->createChild<CollisionShape3D>();
    top->setBox(Math::Vec3(6.0f, 0.5f, 6.0f));
    platform->setPosition(Math::Vec3(0.0f, 0.25f, 0.0f));

    RigidBody3D* crate = root->createChild<RigidBody3D>();
    CollisionShape3D* crateShape = crate->createChild<CollisionShape3D>();
    crateShape->setBox(Math::Vec3(1.0f, 1.0f, 1.0f));
    crate->setPosition(Math::Vec3(0.0f, 1.6f, 0.0f));
    crate->setFriction(1.0f);
    platform->setFriction(1.0f);

    run(tree, 0.5);
    float x = 0.0f;
    for (int i = 0; i < 120; ++i)
    {
        x += 0.02f;
        platform->setGlobalPosition(Math::Vec3(x, 0.25f, 0.0f));
        tree.physicsProcess(kStep);
    }
    printf("  kinematic: platform x %.2f y %.2f (server y %.2f), crate x %.2f y %.2f\n", x,
            platform->getGlobalPosition().y, server.bodyGetTransform(platform->getBody()).col3.y,
            crate->getGlobalPosition().x, crate->getGlobalPosition().y);
    CHECK(crate->getGlobalPosition().x > 1.5f);
    CHECK(near(crate->getGlobalPosition().y, 1.0f, 0.1f));

    tree.finalize();
}

void testShapeChanges()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    RigidBody3D* body = root->createChild<RigidBody3D>();
    CHECK(!body->isSimulated());
    CollisionShape3D* shape = body->createChild<CollisionShape3D>();
    CHECK(shape->getKind() == CollisionShapeKind::None);
    CHECK(!body->isSimulated());

    shape->setBox(Math::Vec3(2.0f, 4.0f, 6.0f));
    CHECK(shape->getKind() == CollisionShapeKind::Box);
    CHECK(near(shape->getBoxSize().y, 4.0f, 1e-4f));
    CHECK(body->isSimulated());

    shape->setDisabled(true);
    CHECK(!body->isSimulated());
    shape->setDisabled(false);
    CHECK(body->isSimulated());

    shape->setCapsule(0.4f, 1.8f);
    CHECK(shape->getKind() == CollisionShapeKind::Capsule);
    CHECK(near(shape->getCapsuleRadius(), 0.4f, 1e-4f));
    CHECK(near(shape->getCapsuleHeight(), 1.8f, 1e-4f));
    CHECK(body->isSimulated());

    shape->setSphere(0.75f);
    CHECK(shape->getKind() == CollisionShapeKind::Sphere);
    CHECK(near(shape->getSphereRadius(), 0.75f, 1e-4f));

    shape->queueFree();
    tree.physicsProcess(kStep);
    CHECK(!body->isSimulated());

    body->queueFree();
    const std::uint32_t before = server.getStats().bodies;
    tree.physicsProcess(kStep);
    tree.physicsProcess(kStep);
    CHECK(server.getStats().bodies + 1 == before || server.getStats().bodies < before);

    tree.finalize();
}

void testLayersAndMasks()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    StaticBody3D* ground = makeGround(root);
    ground->setCollisionLayer(1);
    RigidBody3D* ghost = makeBall(root, Math::Vec3(5.0f, 3.0f, 0.0f));
    ghost->setCollisionMask(2);
    RigidBody3D* solid = makeBall(root, Math::Vec3(-5.0f, 3.0f, 0.0f));
    CHECK(ghost->getCollisionMask() == 2);
    CHECK(ground->getCollisionLayer() == 1);

    run(tree, 2.0);
    CHECK(ghost->getGlobalPosition().y < -5.0f);
    CHECK(near(solid->getGlobalPosition().y, 0.5f, 0.05f));

    tree.finalize();
}

void testConvexAndConcave()
{
    PhysicsServer3D server;
    server.setDefaultLinearDamp(0.0f);
    server.setDefaultAngularDamp(0.0f);
    MeshStore meshes(nullptr);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    const Math::Vec3 vertices[4] = { Math::Vec3(-10.0f, 0.0f, -10.0f),
        Math::Vec3(-10.0f, 0.0f, 10.0f), Math::Vec3(10.0f, 0.0f, 10.0f),
        Math::Vec3(10.0f, 0.0f, -10.0f) };
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    CollisionMesh* floor = new CollisionMesh();
    CHECK(floor->build(vertices, 4, indices, 6));
    const MeshId floorMesh = meshes.create();
    CHECK(meshes.setCollision(floorMesh, floor));

    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* groundShape = ground->createChild<CollisionShape3D>();
    CHECK(groundShape->setConcaveFromMesh(floorMesh));
    CHECK(groundShape->getKind() == CollisionShapeKind::ConcaveMesh);
    CHECK(!groundShape->setConcaveFromMesh(MeshId()));

    RigidBody3D* cube = root->createChild<RigidBody3D>();
    CollisionShape3D* cubeShape = cube->createChild<CollisionShape3D>();
    ct::Vector<Math::Vec3> corners;
    for (int i = 0; i < 8; ++i)
        corners.push_back(
                Math::Vec3((i & 1) ? 0.5f : -0.5f, (i & 2) ? 0.5f : -0.5f, (i & 4) ? 0.5f : -0.5f));
    CHECK(cubeShape->setConvexFromPoints(corners.data(), corners.size()));
    CHECK(cubeShape->getKind() == CollisionShapeKind::ConvexHull);
    cube->setPosition(Math::Vec3(0.0f, 3.0f, 0.0f));

    run(tree, 3.0);
    printf("  convex on concave: cube y %.3f\n", cube->getGlobalPosition().y);
    CHECK(near(cube->getGlobalPosition().y, 0.5f, 0.05f));
    CHECK(cube->isSleeping());

    tree.finalize();
}

void testInterpolationFlag()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);
    RigidBody3D* ball = makeBall(root, Math::Vec3(0.0f, 2.0f, 0.0f));
    CHECK(ball->isInterpolated());
    ball->setInterpolated(false);
    CHECK(!ball->isInterpolated());
    for (int i = 0; i < 30; ++i)
    {
        tree.physicsProcess(kStep);
        tree.process(kStep);
    }
    CHECK(ball->getGlobalPosition().y < 2.0f);
    tree.finalize();
}

} // namespace

int main()
{
    testFallAndContacts();
    testTeleportAndForces();
    testKinematicPlatform();
    testShapeChanges();
    testLayersAndMasks();
    testConvexAndConcave();
    testInterpolationFlag();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
