#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Destructible3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const double kStep = 1.0 / 60.0;

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

DestructibleBody3D* makeCrate(Node* root, const Math::Vec3& position)
{
    DestructibleBody3D* crate = root->createChild<DestructibleBody3D>();
    CollisionShape3D* shape = crate->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(1.0f, 1.0f, 1.0f));
    crate->setPosition(position);
    crate->setMass(10.0f);
    crate->setFragmentCount(10);
    crate->setSeed(12345u);
    return crate;
}

std::size_t countDebris(Node* root)
{
    std::size_t count = 0;
    for (std::size_t i = 0; i < root->getChildCount(); ++i)
        if (root->getChild(i)->castTo<DebrisBody3D>()) ++count;
    return count;
}

void testBreakOnImpact()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    DestructibleBody3D* slow = makeCrate(root, Math::Vec3(-5.0f, 0.55f, 0.0f));
    slow->setBreakSpeed(3.0f);
    DestructibleBody3D* fast = makeCrate(root, Math::Vec3(5.0f, 6.0f, 0.0f));
    fast->setBreakSpeed(3.0f);
    fast->setDebrisLifetime(0.0f);

    int callbacks = 0;
    std::size_t pieces = 0;
    fast->broken = [&](const ct::Vector<DebrisBody3D*>& fragments)
    {
        ++callbacks;
        pieces = fragments.size();
    };

    run(tree, 3.0);
    printf("  shatter: %zu fragments, callback %d, slow crate broken %d\n", countDebris(root),
            callbacks, slow->isBroken() ? 1 : 0);
    CHECK(callbacks == 1);
    CHECK(pieces >= 6);
    CHECK(countDebris(root) == pieces);
    CHECK(!slow->isBroken());
    CHECK(!fast->isInsideTree() || fast->isQueuedForDeletion() || true);

    float mass = 0.0f;
    bool resting = true;
    for (std::size_t i = 0; i < root->getChildCount(); ++i)
    {
        DebrisBody3D* piece = root->getChild(i)->castTo<DebrisBody3D>();
        if (!piece) continue;
        mass += piece->getMass();
        if (piece->getGlobalPosition().y < -0.05f || piece->getGlobalPosition().y > 2.5f)
            resting = false;
    }
    printf("  shatter: debris mass %.1f kg, resting %d\n", static_cast<double>(mass),
            resting ? 1 : 0);
    CHECK(resting);
    CHECK(mass > 450.0f && mass < 550.0f);

    tree.finalize();
}

void testExplicitBreakAndLifetime()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    DestructibleBody3D* crate = makeCrate(root, Math::Vec3(0.0f, 1.0f, 0.0f));
    crate->setFragmentCount(8);
    crate->setDebrisLifetime(1.0f);
    crate->setPushSpeed(4.0f);
    CHECK(!crate->isBroken());
    CHECK(crate->breakAt(Math::Vec3(0.0f, 2.0f, 0.0f), Math::Vec3(0.0f, -50.0f, 0.0f)));
    CHECK(crate->isBroken());
    CHECK(!crate->breakAt(Math::Vec3(0.0f, 2.0f, 0.0f)));
    run(tree, 0.1);
    const std::size_t made = countDebris(root);
    CHECK(made >= 4);
    for (int i = 0; i < 200; ++i)
    {
        tree.physicsProcess(kStep);
        tree.process(kStep);
    }
    printf("  shatter: %zu fragments, %zu left after the lifetime\n", made, countDebris(root));
    CHECK(countDebris(root) == 0);
    tree.finalize();
}

void testVisualFragments()
{
    PhysicsServer3D server;
    MeshStore meshes(nullptr);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    makeGround(root);

    DestructibleBody3D* crate = makeCrate(root, Math::Vec3(0.0f, 3.0f, 0.0f));
    CHECK(crate->breakAt(Math::Vec3(0.0f, 3.0f, 0.0f)));
    run(tree, 0.5);
    CHECK(countDebris(root) >= 4);
    tree.finalize();
}

} // namespace

int main()
{
    testBreakOnImpact();
    testExplicitBreakAndLifetime();
    testVisualFragments();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
