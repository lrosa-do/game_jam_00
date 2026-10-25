#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/Animation.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/Ragdoll3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const double kStep = 1.0 / 60.0;

Math::Mat4 translation(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z));
}

struct BoneSpec
{
    const char* name;
    int parent;
    float x;
    float y;
    float z;
};

template<int N>
Skeleton fromSpecs(const BoneSpec (&bones)[N])
{
    Skeleton skeleton;
    Math::Vec3 absolute[N];
    for (int i = 0; i < N; ++i)
    {
        absolute[i] = Math::Vec3(bones[i].x, bones[i].y, bones[i].z);
        if (bones[i].parent >= 0) absolute[i] = absolute[i] + absolute[bones[i].parent];
        skeleton.addBone(bones[i].name, bones[i].parent,
                translation(bones[i].x, bones[i].y, bones[i].z),
                translation(-absolute[i].x, -absolute[i].y, -absolute[i].z));
    }
    skeleton.finalize();
    return skeleton;
}

Skeleton humanoid()
{
    const BoneSpec bones[18] = { { "mixamorig:Hips", -1, 0.0f, 1.0f, 0.0f },
        { "mixamorig:Spine", 0, 0.0f, 0.2f, 0.0f }, { "mixamorig:Neck", 1, 0.0f, 0.3f, 0.0f },
        { "mixamorig:Head", 2, 0.0f, 0.1f, 0.0f }, { "mixamorig:LeftArm", 1, -0.2f, 0.25f, 0.0f },
        { "mixamorig:LeftForeArm", 4, -0.3f, 0.0f, 0.0f },
        { "mixamorig:LeftHand", 5, -0.25f, 0.0f, 0.0f },
        { "mixamorig:RightArm", 1, 0.2f, 0.25f, 0.0f },
        { "mixamorig:RightForeArm", 7, 0.3f, 0.0f, 0.0f },
        { "mixamorig:RightHand", 8, 0.25f, 0.0f, 0.0f },
        { "mixamorig:LeftUpLeg", 0, -0.1f, -0.05f, 0.0f },
        { "mixamorig:LeftLeg", 10, 0.0f, -0.45f, 0.0f },
        { "mixamorig:LeftFoot", 11, 0.0f, -0.45f, 0.0f },
        { "mixamorig:LeftToeBase", 12, 0.0f, -0.05f, 0.15f },
        { "mixamorig:RightUpLeg", 0, 0.1f, -0.05f, 0.0f },
        { "mixamorig:RightLeg", 14, 0.0f, -0.45f, 0.0f },
        { "mixamorig:RightFoot", 15, 0.0f, -0.45f, 0.0f },
        { "mixamorig:RightToeBase", 16, 0.0f, -0.05f, 0.15f } };
    return fromSpecs(bones);
}

Skeleton rootedHumanoid()
{
    const BoneSpec bones[19] = { { "Root", -1, 0.0f, 1.0f, 0.0f },
        { "Waist", 0, 0.0f, -0.1f, 0.0f }, { "Chest", 1, 0.0f, 0.3f, 0.0f },
        { "Neck", 2, 0.0f, 0.3f, 0.0f }, { "Head", 3, 0.0f, 0.1f, 0.0f },
        { "Humerus.L", 2, -0.25f, 0.2f, 0.0f }, { "Ulna.L", 5, -0.3f, 0.0f, 0.0f },
        { "Hand.L", 6, -0.25f, 0.0f, 0.0f }, { "Humerus.R", 2, 0.25f, 0.2f, 0.0f },
        { "Ulna.R", 8, 0.3f, 0.0f, 0.0f }, { "Hand.R", 9, 0.25f, 0.0f, 0.0f },
        { "Thigh.L", 0, -0.12f, -0.2f, 0.0f }, { "Calf.L", 11, 0.0f, -0.4f, 0.0f },
        { "Foot.L", 12, 0.0f, -0.4f, 0.0f }, { "Toe.L", 13, 0.0f, -0.05f, 0.15f },
        { "Thigh.R", 0, 0.12f, -0.2f, 0.0f }, { "Calf.R", 15, 0.0f, -0.4f, 0.0f },
        { "Foot.R", 16, 0.0f, -0.4f, 0.0f }, { "Toe.R", 17, 0.0f, -0.05f, 0.15f } };
    return fromSpecs(bones);
}

void run(SceneTree& tree, double seconds)
{
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    for (int i = 0; i < steps; ++i)
    {
        tree.physicsProcess(kStep);
        tree.process(kStep);
    }
}

Math::Vec3 origin(const Math::Mat4& frame)
{
    return Math::Vec3(frame.col3.x, frame.col3.y, frame.col3.z);
}

void testRagdollNode()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* groundShape = ground->createChild<CollisionShape3D>();
    groundShape->setBox(Math::Vec3(50.0f, 1.0f, 50.0f));
    ground->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    Skeleton3D* skeleton = root->createChild<Skeleton3D>();
    skeleton->setSkeleton(humanoid());
    skeleton->setPosition(Math::Vec3(0.0f, 2.0f, 0.0f));

    AnimationPlayer* player = root->createChild<AnimationPlayer>();
    Ragdoll3D* ragdoll = root->createChild<Ragdoll3D>();
    ragdoll->setSkeleton(skeleton);
    ragdoll->setAnimationPlayer(player);

    CHECK(!ragdoll->start());
    CHECK(!ragdoll->build());
    const int mapped = ragdoll->autoMapBones();
    printf("  ragdoll node: %d bones mapped\n", mapped);
    CHECK(mapped == 18);
    CHECK(ragdoll->getBone(RagdollBone::Hips) == 0);
    CHECK(ragdoll->getBone(RagdollBone::RightToes) == 17);
    CHECK(ragdoll->build());
    CHECK(ragdoll->isBuilt());
    CHECK(!ragdoll->isActive());

    const int head = skeleton->findBone("mixamorig:Head");
    const int neck = skeleton->findBone("mixamorig:Neck");
    CHECK(head == 3);
    CHECK(ragdoll->getPartBone(PhysicsRagdollPart::Head) == neck);
    CHECK(fabsf(ragdoll->getFacing() + 1.0f) < 1e-4f);
    const float standing = skeleton->getBoneGlobalPose(head).col3.y;

    run(tree, 0.5);
    const Math::Vec3 pelvisFollowing =
            origin(ragdoll->getPartTransform(PhysicsRagdollPart::Pelvis));
    printf("  ragdoll node: pelvis body follows the bone at y %.2f\n", pelvisFollowing.y);
    CHECK(fabsf(pelvisFollowing.y - 3.0f) < 1e-3f);
    CHECK(!ragdoll->isActive());

    const LocalPose headBefore = skeleton->getBonePose(head);
    const LocalPose neckBefore = skeleton->getBonePose(neck);
    CHECK(ragdoll->start());
    CHECK(ragdoll->isActive());
    CHECK(!player->isActive());
    ragdoll->applyImpulse(PhysicsRagdollPart::Pelvis, Math::Vec3(6.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.1f, 0.0f));
    run(tree, 4.0);

    const float lying = skeleton->getBoneGlobalPose(head).col3.y;
    const Math::Mat4 hipsPart = ragdoll->getPartTransform(PhysicsRagdollPart::Pelvis);
    printf("  ragdoll node: head in the skeleton frame %.2f -> %.2f, pelvis part y %.2f x %.2f\n",
            standing, lying, hipsPart.col3.y, hipsPart.col3.x);
    CHECK(standing > 1.5f);
    CHECK(lying < 0.8f);
    CHECK(hipsPart.col3.y < 0.6f && hipsPart.col3.y > 0.0f);
    CHECK(hipsPart.col3.x > 0.3f);
    const LocalPose neckLying = skeleton->getBonePose(neck);
    CHECK(fabsf(neckLying.rotation.w - neckBefore.rotation.w) > 1e-3f ||
            fabsf(neckLying.rotation.x - neckBefore.rotation.x) > 1e-3f);

    ragdoll->stop();
    CHECK(!ragdoll->isActive());
    CHECK(player->isActive());
    const LocalPose neckAfter = skeleton->getBonePose(neck);
    const LocalPose headAfter = skeleton->getBonePose(head);
    printf("  ragdoll node: neck rotation restored to w %.3f (was %.3f while lying)\n",
            neckAfter.rotation.w, neckLying.rotation.w);
    CHECK(fabsf(neckAfter.rotation.w - neckBefore.rotation.w) < 1e-5f);
    CHECK(fabsf(neckAfter.position.y - neckBefore.position.y) < 1e-5f);
    CHECK(fabsf(headAfter.rotation.w - headBefore.rotation.w) < 1e-5f);
    run(tree, 0.1);
    const Math::Vec3 pelvisBack = origin(ragdoll->getPartTransform(PhysicsRagdollPart::Pelvis));
    CHECK(fabsf(pelvisBack.y - 3.0f) < 1e-3f);

    ragdoll->setBlend(0.5f);
    CHECK(ragdoll->getBlend() > 0.49f && ragdoll->getBlend() < 0.51f);
    CHECK(ragdoll->start());
    CHECK(player->isActive());
    run(tree, 1.0);
    ragdoll->setBlend(1.0f);
    CHECK(!player->isActive());
    ragdoll->stop();

    ragdoll->queueFree();
    run(tree, 0.1);
    tree.finalize();
}

void testRootAboveHips()
{
    PhysicsServer3D server;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    StaticBody3D* ground = root->createChild<StaticBody3D>();
    CollisionShape3D* groundShape = ground->createChild<CollisionShape3D>();
    groundShape->setBox(Math::Vec3(50.0f, 1.0f, 50.0f));
    ground->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    Skeleton3D* skeleton = root->createChild<Skeleton3D>();
    skeleton->setSkeleton(rootedHumanoid());
    skeleton->setPosition(Math::Vec3(0.0f, 1.5f, 0.0f));

    Ragdoll3D* ragdoll = root->createChild<Ragdoll3D>();
    ragdoll->setSkeleton(skeleton);
    CHECK(ragdoll->autoMapBones() == 18);
    const int rootBone = skeleton->findBone("Root");
    const int waist = skeleton->findBone("Waist");
    printf("  rooted skeleton: hips mapped to bone %d, root is %d, waist is %d\n",
            ragdoll->getBone(RagdollBone::Hips), rootBone, waist);
    CHECK(ragdoll->getBone(RagdollBone::Hips) == rootBone);
    CHECK(ragdoll->getBone(RagdollBone::Spine) == skeleton->findBone("Chest"));
    CHECK(ragdoll->build());
    CHECK(fabsf(ragdoll->getFacing() + 1.0f) < 1e-4f);

    CHECK(ragdoll->start());
    ragdoll->applyImpulse(PhysicsRagdollPart::Pelvis, Math::Vec3(5.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.1f, 0.0f));
    run(tree, 3.0);

    const Math::Vec3 rootRest(0.0f, 1.0f, 0.0f);
    const Math::Vec3 rootPosition = origin(skeleton->getBoneGlobalPose(rootBone));
    const Math::Vec3 thigh = origin(skeleton->getBoneGlobalPose(skeleton->findBone("Thigh.L")));
    const Math::Vec3 calf = origin(skeleton->getBoneGlobalPose(skeleton->findBone("Calf.L")));
    printf("  rooted skeleton: root bone moved from y %.2f to y %.2f x %.2f, root-thigh %.3f, "
           "thigh-calf %.3f\n",
            rootRest.y, rootPosition.y, rootPosition.x, (thigh - rootPosition).Length(),
            (calf - thigh).Length());
    CHECK(rootPosition.y < 0.0f);
    CHECK(rootPosition.x > 0.2f);
    CHECK(fabsf((thigh - rootPosition).Length() - 0.2332f) < 0.005f);
    CHECK(fabsf((calf - thigh).Length() - 0.4f) < 0.005f);

    ragdoll->stop();
    ragdoll->queueFree();
    run(tree, 0.1);
    tree.finalize();
}

} // namespace

int main()
{
    testRagdollNode();
    testRootAboveHips();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
