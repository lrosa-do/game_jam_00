#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SoftBody3D.h"

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

void makeGround(Node* root)
{
    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* shape = ground->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(50.0f, 1.0f, 50.0f));
    ground->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
}

void bounds(const SoftBody3D* body, Math::Vec3& low, Math::Vec3& high)
{
    const std::size_t count = body->getGeometry().positions.size();
    low = body->getParticlePosition(0);
    high = low;
    for (std::size_t i = 1; i < count; ++i)
    {
        const Math::Vec3 p = body->getParticlePosition(i);
        if (p.x < low.x) low.x = p.x;
        if (p.y < low.y) low.y = p.y;
        if (p.z < low.z) low.z = p.z;
        if (p.x > high.x) high.x = p.x;
        if (p.y > high.y) high.y = p.y;
        if (p.z > high.z) high.z = p.z;
    }
}

SoftBody3D* makeFlag(Node* root, const Math::Vec3& position)
{
    SoftBody3D* flag = root->createChild<SoftBody3D>();
    flag->makeCloth(2.0f, 2.0f, 12, 12);
    flag->setRotationDegrees(Math::Vec3(90.0f, 0.0f, 0.0f));
    flag->setPosition(position);
    flag->pinInBox(Math::Box(Math::Vec3(-2.0f, -1.0f, -1.01f), Math::Vec3(2.0f, 1.0f, -0.99f)));
    return flag;
}

void testHangingFlag()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    SoftBody3D* flag = makeFlag(root, Math::Vec3(0.0f, 3.0f, 0.0f));
    CHECK(!flag->isBuilt());
    run(tree, 0.1);
    CHECK(flag->isBuilt());
    CHECK(flag->getSoftBody().valid());
    CHECK(flag->getParticleCount() == flag->getGeometry().positions.size());

    std::size_t pinned = 0;
    for (std::size_t i = 0; i < flag->getGeometry().positions.size(); ++i)
        if (flag->isPinned(i)) ++pinned;
    CHECK(pinned == 13);

    run(tree, 3.0);
    Math::Vec3 low, high;
    bounds(flag, low, high);
    printf("  soft flag: y %.3f .. %.3f, stretch %.3f\n", low.y, high.y, flag->getWorstStretch());
    CHECK(near(high.y, 4.0f, 0.01f));
    CHECK(low.y < 2.4f);
    CHECK(low.y > 1.5f);
    CHECK(flag->getWorstStretch() < 1.3f);

    for (int i = 1; i <= 50; ++i)
    {
        flag->setPosition(Math::Vec3(0.1f * static_cast<float>(i), 3.0f, 0.0f));
        tree.physicsProcess(kStep);
    }
    run(tree, 0.1);
    float pinnedLow = 1e9f;
    float pinnedHigh = -1e9f;
    bool pinnedLevel = true;
    for (std::size_t i = 0; i < flag->getGeometry().positions.size(); ++i)
    {
        if (!flag->isPinned(i)) continue;
        const Math::Vec3 p = flag->getParticlePosition(i);
        if (p.x < pinnedLow) pinnedLow = p.x;
        if (p.x > pinnedHigh) pinnedHigh = p.x;
        if (!near(p.y, 4.0f, 0.01f)) pinnedLevel = false;
    }
    printf("  soft flag moved: pinned x %.3f .. %.3f\n", pinnedLow, pinnedHigh);
    CHECK(pinnedLevel);
    CHECK(near(pinnedLow, 4.0f, 0.01f));
    CHECK(near(pinnedHigh, 6.0f, 0.01f));
    printf("  soft flag stretch %.3f\n", flag->getWorstStretch());
    bounds(flag, low, high);
    CHECK(flag->getWorstStretch() < 1.5f);
}

void testPinsAtRuntime()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    SoftBody3D* flag = makeFlag(root, Math::Vec3(0.0f, 3.0f, 0.0f));
    run(tree, 1.0);
    Math::Vec3 low, high;
    bounds(flag, low, high);
    CHECK(near(high.y, 4.0f, 0.01f));

    flag->clearPins();
    for (std::size_t i = 0; i < flag->getGeometry().positions.size(); ++i)
        CHECK(!flag->isPinned(i));
    run(tree, 3.0);
    bounds(flag, low, high);
    printf("  soft flag released: y %.3f .. %.3f\n", low.y, high.y);
    CHECK(low.y > -0.02f);
    CHECK(high.y < 1.0f);

    flag->pinInSphere(Math::Vec3(0.0f, 0.0f, 0.0f), 0.3f);
    std::size_t pinned = 0;
    for (std::size_t i = 0; i < flag->getGeometry().positions.size(); ++i)
        if (flag->isPinned(i)) ++pinned;
    CHECK(pinned > 0);
    flag->setPinned(100000, true);
    CHECK(!flag->isPinned(100000));
}

void testBall()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    SoftBody3D* ball = root->createChild<SoftBody3D>();
    ball->makeBall(0.5f, 8, 12);
    PhysicsSoftBodyParameters parameters;
    parameters.mass = 2.0f;
    parameters.structuralCompliance = 0.0001f;
    parameters.bendCompliance = 0.001f;
    parameters.pressure = 40.0f;
    ball->setParameters(parameters);
    ball->setPosition(Math::Vec3(0.0f, 2.0f, 0.0f));

    run(tree, 0.05);
    const std::size_t vertices = ball->getGeometry().positions.size();
    printf("  soft ball: %zu vertices, %zu particles\n", vertices, ball->getParticleCount());
    CHECK(ball->getParticleCount() < vertices);
    CHECK(ball->getParticleCount() == 7 * 12 + 2);

    run(tree, 3.0);
    Math::Vec3 low, high;
    bounds(ball, low, high);
    printf("  soft ball settled: y %.3f .. %.3f\n", low.y, high.y);
    CHECK(low.y > -0.02f);
    CHECK(low.y < 0.1f);
    CHECK(high.y - low.y > 0.6f);
    CHECK(high.y - low.y < 1.2f);

    ball->addVelocity(Math::Vec3(0.0f, 4.0f, 0.0f));
    run(tree, 0.3);
    bounds(ball, low, high);
    CHECK(low.y > 0.3f);

    ball->reset();
    run(tree, 0.02);
    bounds(ball, low, high);
    CHECK(high.y > 2.2f);
}

void testFilter()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    SoftBody3D* ghost = root->createChild<SoftBody3D>();
    ghost->makeCloth(1.0f, 1.0f, 6, 6);
    PhysicsSoftBodyParameters parameters;
    parameters.collisionMask = 0;
    ghost->setParameters(parameters);
    ghost->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));

    SoftBody3D* solid = root->createChild<SoftBody3D>();
    solid->makeCloth(1.0f, 1.0f, 6, 6);
    solid->setPosition(Math::Vec3(3.0f, 1.0f, 0.0f));

    run(tree, 2.0);
    Math::Vec3 low, high;
    bounds(ghost, low, high);
    CHECK(high.y < -1.0f);
    bounds(solid, low, high);
    CHECK(low.y > -0.02f);

    solid->setSimulationEnabled(false);
    CHECK(!solid->isSimulationEnabled());
    const Math::Vec3 before = solid->getParticlePosition(0);
    run(tree, 0.5);
    const Math::Vec3 after = solid->getParticlePosition(0);
    CHECK(near(before.y, after.y, 1e-5f));
}

void testLifetime()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    SoftBody3D* body = root->createChild<SoftBody3D>();
    body->makeCloth(1.0f, 1.0f, 4, 4);
    run(tree, 0.1);
    const PhysicsSoftBodyId handle = body->getSoftBody();
    CHECK(server.softBodyValid(handle));

    body->queueFree();
    run(tree, 0.1);
    CHECK(!server.softBodyValid(handle));

    SoftBody3D* second = root->createChild<SoftBody3D>();
    second->makeCloth(1.0f, 1.0f, 4, 4);
    run(tree, 0.1);
    CHECK(second->isBuilt());
    second->makeCloth(2.0f, 2.0f, 8, 8);
    CHECK(!second->isBuilt());
    run(tree, 0.1);
    CHECK(second->isBuilt());
    CHECK(second->getParticleCount() == 81);

    SoftBody3D* empty = root->createChild<SoftBody3D>();
    run(tree, 0.1);
    CHECK(!empty->isBuilt());
    CHECK(empty->getParticleCount() == 0);
}

} // namespace

int main()
{
    testHangingFlag();
    testPinsAtRuntime();
    testBall();
    testFilter();
    testLifetime();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
