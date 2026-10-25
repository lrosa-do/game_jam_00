#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/CharacterBody3D.h"
#include "prisma3d/scene/CharacterRigidBody3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const double kStep = 1.0 / 60.0;

bool near(float a, float b, float tolerance = 1e-3f)
{
    return fabsf(a - b) <= tolerance;
}

void run(SceneTree& tree, double seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i) tree.physicsProcess(kStep);
}

void drive(SceneTree& tree, CharacterBody3D* body, const Math::Vec3& input, double seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i)
    {
        Math::Vec3 velocity = body->getVelocity();
        velocity.x = input.x;
        velocity.z = input.z;
        velocity.y -= 9.8f * static_cast<float>(kStep);
        body->setVelocity(velocity);
        body->moveAndSlide();
        tree.physicsProcess(kStep);
    }
}

StaticBody3D* makeBox(Node* root, const Math::Vec3& size, const Math::Vec3& position)
{
    StaticBody3D* body = root->createChild<StaticBody3D>();
    CollisionShape3D* shape = body->createChild<CollisionShape3D>();
    shape->setBox(size);
    body->setPosition(position);
    return body;
}

CharacterBody3D* makePlayer(Node* root, const Math::Vec3& position)
{
    CharacterBody3D* player = root->createChild<CharacterBody3D>();
    CollisionShape3D* shape = player->createChild<CollisionShape3D>();
    shape->setCapsule(0.4f, 1.8f);
    player->setPosition(position);
    return player;
}

void testWalking()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeBox(root, Math::Vec3(100.0f, 1.0f, 100.0f), Math::Vec3(0.0f, -0.5f, 0.0f));
    CharacterBody3D* player = makePlayer(root, Math::Vec3(0.0f, 3.0f, 0.0f));

    drive(tree, player, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    printf("  character node: landed y %.3f on floor %d\n", player->getGlobalTransform().col3.y,
            player->isOnFloor() ? 1 : 0);
    CHECK(player->isOnFloor());
    CHECK(near(player->getGlobalTransform().col3.y, 0.92f, 0.05f));
    CHECK(near(player->getFloorAngle(), 0.0f, 0.5f));
    CHECK(player->isNearGround(0.3f));
    CHECK(near(player->getRadius(), 0.4f));
    CHECK(near(player->getHeight(), 1.0f));

    makeBox(root, Math::Vec3(0.4f, 4.0f, 10.0f), Math::Vec3(5.0f, 2.0f, 0.0f));
    drive(tree, player, Math::Vec3(4.0f, 0.0f, 0.0f), 2.0);
    const float x = player->getGlobalTransform().col3.x;
    printf("  character node: against wall x %.3f on wall %d\n", x, player->isOnWall() ? 1 : 0);
    CHECK(player->isOnWall());
    CHECK(x > 4.3f && x < 4.5f);
    CHECK(player->getSlideCollisionCount() >= 1);
    bool wallSeen = false;
    for (std::size_t i = 0; i < player->getSlideCollisionCount(); ++i)
        if (player->getSlideCollider(i) && player->getSlideCollider(i)->castTo<StaticBody3D>() &&
                player->getSlideNormal(i).x < -0.9f)
            wallSeen = true;
    CHECK(wallSeen);

    player->setPosition(Math::Vec3(-10.0f, 1.5f, 0.0f));
    player->setVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
    drive(tree, player, Math::Vec3(0.0f, 0.0f, 0.0f), 1.0);
    CHECK(near(player->getGlobalTransform().col3.x, -10.0f, 0.05f));
    CHECK(player->isOnFloor());
}

void testPushing()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeBox(root, Math::Vec3(100.0f, 1.0f, 100.0f), Math::Vec3(0.0f, -0.5f, 0.0f));
    CharacterBody3D* player = makePlayer(root, Math::Vec3(0.0f, 1.0f, 0.0f));
    player->setPushForce(300.0f);

    RigidBody3D* crate = root->createChild<RigidBody3D>();
    CollisionShape3D* shape = crate->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(0.6f, 0.6f, 0.6f));
    crate->setMass(5.0f);
    crate->setPosition(Math::Vec3(1.5f, 0.4f, 0.0f));

    drive(tree, player, Math::Vec3(1.5f, 0.0f, 0.0f), 3.0);
    const float crateX = crate->getGlobalTransform().col3.x;
    printf("  character node: crate pushed to x %.3f, player x %.3f\n", crateX,
            player->getGlobalTransform().col3.x);
    CHECK(crateX > 3.0f);
    CHECK(player->getGlobalTransform().col3.x > 2.0f);
}

void testCarriedWeight()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeBox(root, Math::Vec3(100.0f, 1.0f, 100.0f), Math::Vec3(0.0f, -0.5f, 0.0f));
    CharacterBody3D* player = makePlayer(root, Math::Vec3(0.0f, 1.0f, 0.0f));

    RigidBody3D* ball = root->createChild<RigidBody3D>();
    CollisionShape3D* shape = ball->createChild<CollisionShape3D>();
    shape->setSphere(0.3f);
    ball->setPosition(Math::Vec3(0.0f, 4.0f, 0.0f));

    drive(tree, player, Math::Vec3(0.0f, 0.0f, 0.0f), 3.0);
    const float ballY = ball->getGlobalTransform().col3.y;
    printf("  character node: ball rests at y %.3f\n", ballY);
    CHECK(ballY > 1.5f);
    CHECK(near(player->getGlobalTransform().col3.y, 0.92f, 0.05f));
}

void testFilter()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    StaticBody3D* ground =
            makeBox(root, Math::Vec3(100.0f, 1.0f, 100.0f), Math::Vec3(0.0f, -0.5f, 0.0f));
    ground->setCollisionLayer(2);
    CharacterBody3D* player = makePlayer(root, Math::Vec3(0.0f, 3.0f, 0.0f));
    player->setCollisionMask(1);
    drive(tree, player, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    CHECK(!player->isOnFloor());
    CHECK(player->getGlobalTransform().col3.y < -2.0f);

    player->setCollisionMask(2);
    player->setPosition(Math::Vec3(0.0f, 3.0f, 0.0f));
    player->setVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
    drive(tree, player, Math::Vec3(0.0f, 0.0f, 0.0f), 1.5);
    CHECK(player->isOnFloor());
}

void testLifetime()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeBox(root, Math::Vec3(100.0f, 1.0f, 100.0f), Math::Vec3(0.0f, -0.5f, 0.0f));
    CharacterBody3D* player = makePlayer(root, Math::Vec3(0.0f, 1.0f, 0.0f));
    drive(tree, player, Math::Vec3(0.0f, 0.0f, 0.0f), 0.2);
    const PhysicsCharacterId handle = player->getCharacter();
    CHECK(server.characterValid(handle));
    player->queueFree();
    run(tree, 0.1);
    CHECK(!server.characterValid(handle));

    CharacterBody3D* loose = root->createChild<CharacterBody3D>();
    CHECK(loose->moveAndSlide(1.0f / 60.0f));
    CHECK(!loose->isOnFloor());
}

CharacterRigidBody3D* makeRigidPlayer(Node* root, const Math::Vec3& position)
{
    CharacterRigidBody3D* player = root->createChild<CharacterRigidBody3D>();
    CollisionShape3D* shape = player->createChild<CollisionShape3D>();
    shape->setCapsule(0.4f, 1.8f);
    player->setPosition(position);
    return player;
}

void testRigidGround()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeBox(root, Math::Vec3(100.0f, 1.0f, 100.0f), Math::Vec3(0.0f, -0.5f, 0.0f));
    CharacterRigidBody3D* player = makeRigidPlayer(root, Math::Vec3(0.0f, 3.0f, 0.0f));

    run(tree, 0.1);
    printf("  rigid character: state in the air %d\n", static_cast<int>(player->getGroundState()));
    CHECK(player->getGroundState() == PhysicsGroundState::InAir);
    CHECK(!player->isSupported());

    run(tree, 2.0);
    printf("  rigid character: y %.3f, state %d, normal y %.3f\n",
            player->getGlobalTransform().col3.y, static_cast<int>(player->getGroundState()),
            player->getGroundNormal().y);
    CHECK(player->isOnGround());
    CHECK(player->isSupported());
    CHECK(player->getGroundNormal().y > 0.99f);
    CHECK(near(player->getGlobalTransform().col3.y, 0.9f, 0.08f));
    CHECK(player->getGroundBody() != nullptr);
    CHECK(player->getGroundBody()->castTo<StaticBody3D>() != nullptr);

    player->setLinearVelocity(Math::Vec3(0.0f, 6.0f, 0.0f));
    run(tree, 0.1);
    CHECK(player->getGroundState() == PhysicsGroundState::InAir);

    Math::Mat4 frame = player->getGlobalTransform();
    CHECK(fabsf(frame.col1.y - 1.0f) < 1e-3f);
}

void testRigidSlope()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    StaticBody3D* ramp = makeBox(root, Math::Vec3(12.0f, 0.4f, 8.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
    ramp->setRotation(Math::Vec3(0.0f, 0.0f, 3.14159265f / 3.0f));
    CharacterRigidBody3D* player = makeRigidPlayer(root, Math::Vec3(-1.5f, 3.0f, 0.0f));
    player->setMaxSlopeAngle(50.0f);
    bool sawSteep = false;
    for (int i = 0; i < 60; ++i)
    {
        tree.physicsProcess(kStep);
        if (player->getGroundState() == PhysicsGroundState::OnSteepGround) sawSteep = true;
    }
    printf("  rigid character: on a 60 degree ramp, steep ground seen %d\n", sawSteep ? 1 : 0);
    CHECK(sawSteep);
    CHECK(!player->isOnGround());

    CharacterRigidBody3D* second = makeRigidPlayer(root, Math::Vec3(30.0f, 3.0f, 0.0f));
    makeBox(root, Math::Vec3(20.0f, 1.0f, 20.0f), Math::Vec3(30.0f, -0.5f, 0.0f));
    run(tree, 2.0);
    CHECK(second->isOnGround());
}

void testRigidPlatform()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    AnimatableBody3D* platform = root->createChild<AnimatableBody3D>();
    CollisionShape3D* shape = platform->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(10.0f, 0.5f, 10.0f));
    platform->setPosition(Math::Vec3(0.0f, -0.25f, 0.0f));
    CharacterRigidBody3D* player = makeRigidPlayer(root, Math::Vec3(0.0f, 1.0f, 0.0f));

    for (int i = 0; i < 120; ++i)
    {
        Math::Mat4 frame = platform->getGlobalTransform();
        frame.col3.x += 2.0f * static_cast<float>(kStep);
        platform->setGlobalTransform(frame);
        tree.physicsProcess(kStep);
    }
    printf("  rigid character: ground velocity x %.3f\n", player->getGroundVelocity().x);
    CHECK(player->isOnGround());
    CHECK(near(player->getGroundVelocity().x, 2.0f, 0.3f));
    CHECK(player->getGroundBody() == platform);
}

void testRigidLifetime()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeBox(root, Math::Vec3(100.0f, 1.0f, 100.0f), Math::Vec3(0.0f, -0.5f, 0.0f));
    CharacterRigidBody3D* player = makeRigidPlayer(root, Math::Vec3(0.0f, 1.0f, 0.0f));
    run(tree, 0.5);
    player->queueFree();
    run(tree, 0.1);
    CHECK(root->getChildCount() >= 1);
}

} // namespace

int main()
{
    testWalking();
    testPushing();
    testCarriedWeight();
    testFilter();
    testLifetime();
    testRigidGround();
    testRigidSlope();
    testRigidPlatform();
    testRigidLifetime();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
