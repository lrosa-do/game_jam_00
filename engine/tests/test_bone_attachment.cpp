#include "Check.h"

#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/scene/BoneAttachment3D.h"
#include "prisma3d/scene/ModelInstance.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const float kHalfPi = 1.57079632679f;
const double kStep = 0.25;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

Math::Mat4 translation(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z));
}

Skeleton chain()
{
    Skeleton skeleton;
    skeleton.addBone("root", -1, translation(0.0f, 0.0f, 0.0f), translation(0.0f, 0.0f, 0.0f));
    skeleton.addBone("spine", 0, translation(0.0f, 1.0f, 0.0f), translation(0.0f, -1.0f, 0.0f));
    skeleton.addBone("head", 1, translation(0.0f, 1.0f, 0.0f), translation(0.0f, -2.0f, 0.0f));
    skeleton.finalize();
    return skeleton;
}

Skeleton3D* addSkeleton(Node* parent)
{
    Skeleton3D* skeleton = parent->createChild<Skeleton3D>();
    skeleton->setName("Skeleton");
    skeleton->setSkeleton(chain());
    return skeleton;
}

void testFollow()
{
    SceneTree tree;
    tree.initialize();
    Skeleton3D* skeleton = addSkeleton(tree.getRoot());
    skeleton->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));

    BoneAttachment3D* hand = skeleton->createChild<BoneAttachment3D>();
    Node* asNode = hand;
    CHECK(asNode->castTo<BoneAttachment3D>() == hand);
    CHECK(asNode->castTo<Node3D>() == hand);
    CHECK(asNode->castTo<Skeleton3D>() == nullptr);
    CHECK(hand->getSkeleton() == skeleton);
    CHECK(hand->getBoneIdx() == -1);

    hand->setBoneName("head");
    CHECK(hand->getBoneIdx() == 2);
    CHECK(nearVec3(hand->getPosition(), 0.0f, 2.0f, 0.0f));

    Node3D* sword = hand->createChild<Node3D>();
    sword->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(sword->getGlobalPosition(), 11.0f, 2.0f, 0.0f));

    skeleton->setBonePosePosition(1, Math::Vec3(0.0f, 3.0f, 0.0f));
    CHECK(nearVec3(hand->getPosition(), 0.0f, 2.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(hand->getPosition(), 0.0f, 4.0f, 0.0f));
    CHECK(nearVec3(sword->getGlobalPosition(), 11.0f, 4.0f, 0.0f));

    skeleton->setBonePoseRotation(1,
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi));
    tree.process(kStep);
    CHECK(nearVec3(hand->getPosition(), -1.0f, 3.0f, 0.0f));
    CHECK(nearVec3(sword->getGlobalPosition(), 9.0f, 4.0f, 0.0f));

    hand->setBoneIdx(1);
    CHECK(strcmp(hand->getBoneName().c_str(), "spine") == 0);
    CHECK(nearVec3(hand->getPosition(), 0.0f, 3.0f, 0.0f));

    hand->setBoneName("nowhere");
    CHECK(hand->getBoneIdx() == -1);
    skeleton->setBonePosePosition(1, Math::Vec3(0.0f, 7.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(hand->getPosition(), 0.0f, 3.0f, 0.0f));

    hand->setBoneName("spine");
    CHECK(hand->getBoneIdx() == 1);
    CHECK(nearVec3(hand->getPosition(), 0.0f, 7.0f, 0.0f));
}

void testTreeEnterAndExit()
{
    SceneTree tree;
    tree.initialize();
    Node3D holder;
    Skeleton3D* skeleton = addSkeleton(&holder);
    BoneAttachment3D* hand = skeleton->createChild<BoneAttachment3D>();

    hand->setBoneName("head");
    CHECK(hand->getBoneIdx() == 2);
    CHECK(nearVec3(hand->getPosition(), 0.0f, 2.0f, 0.0f));

    skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 4.0f, 0.0f));
    skeleton->updateSkeleton();
    CHECK(nearVec3(hand->getPosition(), 0.0f, 2.0f, 0.0f));

    skeleton->reparent(tree.getRoot());
    CHECK(nearVec3(hand->getPosition(), 0.0f, 5.0f, 0.0f));
    skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 1.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(hand->getPosition(), 0.0f, 2.0f, 0.0f));

    skeleton->reparent(&holder);
    skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 6.0f, 0.0f));
    skeleton->updateSkeleton();
    CHECK(nearVec3(hand->getPosition(), 0.0f, 2.0f, 0.0f));
}

void testSeveralListeners()
{
    SceneTree tree;
    tree.initialize();
    Skeleton3D* skeleton = addSkeleton(tree.getRoot());
    int updates = 0;
    skeleton->poseUpdated = [&updates]() { ++updates; };

    BoneAttachment3D* spine = skeleton->createChild<BoneAttachment3D>();
    spine->setBoneName("spine");
    BoneAttachment3D* head = skeleton->createChild<BoneAttachment3D>();
    head->setBoneName("head");
    BoneAttachment3D* gone = skeleton->createChild<BoneAttachment3D>();
    gone->setBoneName("head");

    tree.process(kStep);
    CHECK(updates == 1);

    skeleton->setBonePosePosition(1, Math::Vec3(2.0f, 1.0f, 0.0f));
    gone->queueFree();
    tree.process(kStep);
    CHECK(updates == 2);
    CHECK(nearVec3(spine->getPosition(), 2.0f, 1.0f, 0.0f));
    CHECK(nearVec3(head->getPosition(), 2.0f, 2.0f, 0.0f));

    skeleton->setBonePosePosition(1, Math::Vec3(3.0f, 1.0f, 0.0f));
    tree.process(kStep);
    CHECK(updates == 3);
    CHECK(nearVec3(spine->getPosition(), 3.0f, 1.0f, 0.0f));
    CHECK(nearVec3(head->getPosition(), 3.0f, 2.0f, 0.0f));
}

void testExternalSkeleton()
{
    SceneTree tree;
    tree.initialize();
    Skeleton3D* skeleton = addSkeleton(tree.getRoot());
    skeleton->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    Node3D* other = tree.getRoot()->createChild<Node3D>();
    other->setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));

    BoneAttachment3D* hand = other->createChild<BoneAttachment3D>();
    hand->setBoneName("head");
    CHECK(hand->getSkeleton() == nullptr);
    CHECK(hand->getBoneIdx() == -1);
    hand->setExternalSkeleton("../../Skeleton");
    CHECK(hand->getSkeleton() == nullptr);
    hand->setUseExternalSkeleton(true);
    CHECK(hand->getSkeleton() == skeleton);
    CHECK(hand->getBoneIdx() == 2);
    CHECK(nearVec3(hand->getGlobalPosition(), 10.0f, 2.0f, 0.0f));
    CHECK(nearVec3(hand->getPosition(), 10.0f, 2.0f, -5.0f));

    BoneAttachment3D* nested = hand->createChild<BoneAttachment3D>();
    nested->setUseExternalSkeleton(true);
    CHECK(nested->getSkeleton() == skeleton);
    CHECK(strcmp(nested->getExternalSkeleton().c_str(), "../../../Skeleton") == 0);
    nested->setBoneName("spine");
    CHECK(nested->getBoneIdx() == 1);
    CHECK(nearVec3(nested->getGlobalPosition(), 10.0f, 1.0f, 0.0f));

    skeleton->setPosition(Math::Vec3(20.0f, 0.0f, 0.0f));
    skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 2.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(hand->getGlobalPosition(), 20.0f, 3.0f, 0.0f));
    CHECK(nearVec3(nested->getGlobalPosition(), 20.0f, 1.0f, 0.0f));

    hand->setExternalSkeleton("../../Nowhere");
    CHECK(hand->getSkeleton() == nullptr);
    skeleton->setBonePosePosition(2, Math::Vec3(0.0f, 9.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(hand->getGlobalPosition(), 20.0f, 3.0f, 0.0f));
    hand->setExternalSkeleton("/root/Skeleton");
    CHECK(hand->getSkeleton() == skeleton);
    CHECK(nearVec3(hand->getGlobalPosition(), 20.0f, 10.0f, 0.0f));

    skeleton->queueFree();
    tree.process(kStep);
    CHECK(hand->getSkeleton() == nullptr);
    CHECK(nested->getSkeleton() == nullptr);
    hand->setBoneIdx(1);
    hand->queueFree();
    tree.process(kStep);
}

void testOverridePose()
{
    SceneTree tree;
    tree.initialize();
    Skeleton3D* skeleton = addSkeleton(tree.getRoot());
    BoneAttachment3D* grip = skeleton->createChild<BoneAttachment3D>();
    grip->setBoneName("spine");
    CHECK(!grip->getOverridePose());

    grip->setOverridePose(true);
    CHECK(grip->getOverridePose());
    grip->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(skeleton->getBoneGlobalPose(1).GetTranslation(), 0.0f, 5.0f, 0.0f));
    CHECK(nearVec3(skeleton->getBoneGlobalPose(2).GetTranslation(), 0.0f, 6.0f, 0.0f));

    const std::uint64_t version = skeleton->getPoseVersion();
    skeleton->setBonePosePosition(1, Math::Vec3(0.0f, 1.0f, 0.0f));
    tree.process(kStep);
    CHECK(skeleton->getPoseVersion() == version + 1);
    CHECK(!skeleton->isDirty());
    CHECK(nearVec3(skeleton->getBoneGlobalPose(1).GetTranslation(), 0.0f, 5.0f, 0.0f));
    CHECK(nearVec3(skeleton->getBonePalette()[2].GetTranslation(), 0.0f, 4.0f, 0.0f));
    CHECK(nearVec3(grip->getPosition(), 0.0f, 5.0f, 0.0f));

    tree.process(kStep);
    CHECK(skeleton->getPoseVersion() == version + 1);

    grip->setOverridePose(false);
    CHECK(nearVec3(skeleton->getBonePosePosition(1), 0.0f, 1.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(grip->getPosition(), 0.0f, 1.0f, 0.0f));

    Node3D* other = tree.getRoot()->createChild<Node3D>();
    other->setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));
    skeleton->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    BoneAttachment3D* external = other->createChild<BoneAttachment3D>();
    external->setBoneName("spine");
    external->setExternalSkeleton("../../Skeleton");
    external->setUseExternalSkeleton(true);
    external->setOverridePose(true);
    external->setGlobalPosition(Math::Vec3(10.0f, 8.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(skeleton->getBoneGlobalPose(1).GetTranslation(), 0.0f, 8.0f, 0.0f));
}

int addNode(ModelData& document, const char* name, int parent, float x, float y, float z)
{
    const int index = static_cast<int>(document.nodes.size());
    ModelNodeData node;
    node.name = name;
    node.parent = parent;
    node.translation = Math::Vec3(x, y, z);
    document.nodes.push_back(node);
    if (parent >= 0) document.nodes[static_cast<std::size_t>(parent)].children.push_back(index);
    else
        document.roots.push_back(index);
    return index;
}

template<typename T>
T* childOf(Node* parent, const char* name)
{
    Node* child = parent ? parent->findChild(name) : nullptr;
    return child ? child->castTo<T>() : nullptr;
}

void testModelNodes()
{
    ModelData document;
    document.path = "rig.glb";
    document.skeletons.push_back(chain());
    document.meshes.push_back(ModelMeshData());
    const int rig = addNode(document, "Rig", -1, 5.0f, 0.0f, 0.0f);
    const int rootJoint = addNode(document, "root", rig, 0.0f, 0.0f, 0.0f);
    const int spineJoint = addNode(document, "spine", rootJoint, 0.0f, 1.0f, 0.0f);
    const int headJoint = addNode(document, "head", spineJoint, 0.0f, 1.0f, 0.0f);
    const int swordNode = addNode(document, "Sword", spineJoint, 1.0f, 0.0f, 0.0f);
    addNode(document, "Tip", swordNode, 0.0f, 0.0f, 1.0f);
    addNode(document, "Shield", spineJoint, -1.0f, 0.0f, 0.0f);
    addNode(document, "Hat", headJoint, 0.0f, 0.5f, 0.0f);
    addNode(document, "Prop", rig, 0.0f, 0.0f, 2.0f);
    document.nodes[static_cast<std::size_t>(headJoint)].mesh = 0;

    SceneTree tree;
    tree.initialize();
    const ct::Vector<MeshId> meshes;
    Node3D* model = instantiateModel(tree.getRoot(), document, meshes, 0);
    CHECK(model != nullptr);
    if (!model) return;

    Node3D* rigNode = childOf<Node3D>(model, "Rig");
    CHECK(rigNode != nullptr && rigNode->getChildCount() == 2);
    CHECK(childOf<Node3D>(rigNode, "Prop") != nullptr);
    Skeleton3D* skeleton = childOf<Skeleton3D>(rigNode, "Skeleton3D");
    CHECK(skeleton != nullptr);
    if (!skeleton) return;
    CHECK(skeleton->getChildCount() == 2);
    CHECK(skeleton->findChild("root") == nullptr);

    BoneAttachment3D* spine = childOf<BoneAttachment3D>(skeleton, "spine");
    BoneAttachment3D* head = childOf<BoneAttachment3D>(skeleton, "head");
    CHECK(spine != nullptr && head != nullptr);
    if (!spine || !head) return;
    CHECK(spine->getBoneIdx() == 1 && head->getBoneIdx() == 2);
    CHECK(spine->getSkeleton() == skeleton);
    CHECK(spine->getChildCount() == 2);

    Node3D* sword = childOf<Node3D>(spine, "Sword");
    Node3D* shield = childOf<Node3D>(spine, "Shield");
    Node3D* tip = childOf<Node3D>(sword, "Tip");
    CHECK(sword != nullptr && shield != nullptr && tip != nullptr);
    MeshInstance3D* headMesh = childOf<MeshInstance3D>(head, "head");
    CHECK(headMesh != nullptr);
    Node3D* hat = childOf<Node3D>(headMesh, "Hat");
    CHECK(hat != nullptr);
    if (!sword || !shield || !tip || !headMesh || !hat) return;

    CHECK(headMesh->getTransform() == Math::Mat4::Identity());
    CHECK(nearVec3(sword->getPosition(), 1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(sword->getGlobalPosition(), 6.0f, 1.0f, 0.0f));
    CHECK(nearVec3(shield->getGlobalPosition(), 4.0f, 1.0f, 0.0f));
    CHECK(nearVec3(tip->getGlobalPosition(), 6.0f, 1.0f, 1.0f));
    CHECK(nearVec3(headMesh->getGlobalPosition(), 5.0f, 2.0f, 0.0f));
    CHECK(nearVec3(hat->getGlobalPosition(), 5.0f, 2.5f, 0.0f));

    skeleton->setBonePosePosition(1, Math::Vec3(0.0f, 3.0f, 0.0f));
    skeleton->setBonePoseRotation(2,
            Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), kHalfPi));
    tree.process(kStep);
    CHECK(nearVec3(sword->getGlobalPosition(), 6.0f, 3.0f, 0.0f));
    CHECK(nearVec3(tip->getGlobalPosition(), 6.0f, 3.0f, 1.0f));
    CHECK(nearVec3(headMesh->getGlobalPosition(), 5.0f, 4.0f, 0.0f));
    CHECK(nearVec3(hat->getGlobalPosition(), 4.5f, 4.0f, 0.0f));
}

// A mesh hung under a bone may carry the bone's name (the bomb under the bone "Bomb" of the Serious
// Sam 2 kamikaze); it is not taken for the joint and keeps its own transform under the bone.
void testMeshNamedAsBone()
{
    ModelData document;
    document.path = "rig.glb";
    document.skeletons.push_back(chain());
    document.meshes.push_back(ModelMeshData());
    const int rig = addNode(document, "Rig", -1, 5.0f, 0.0f, 0.0f);
    const int rootJoint = addNode(document, "root", rig, 0.0f, 0.0f, 0.0f);
    const int spineJoint = addNode(document, "spine", rootJoint, 0.0f, 1.0f, 0.0f);
    const int headJoint = addNode(document, "head", spineJoint, 0.0f, 1.0f, 0.0f);
    const int bomb = addNode(document, "head", headJoint, 0.0f, 0.5f, 0.0f);
    document.nodes[static_cast<std::size_t>(bomb)].mesh = 0;

    SceneTree tree;
    tree.initialize();
    const ct::Vector<MeshId> meshes;
    Node3D* model = instantiateModel(tree.getRoot(), document, meshes, 0);
    CHECK(model != nullptr);
    if (!model) return;
    Node3D* rigNode = childOf<Node3D>(model, "Rig");
    Skeleton3D* skeleton = rigNode ? childOf<Skeleton3D>(rigNode, "Skeleton3D") : nullptr;
    CHECK(skeleton != nullptr);
    if (!skeleton) return;
    BoneAttachment3D* head = childOf<BoneAttachment3D>(skeleton, "head");
    CHECK(head != nullptr && head->getBoneIdx() == 2);
    MeshInstance3D* mesh = head ? childOf<MeshInstance3D>(head, "head") : nullptr;
    CHECK(mesh != nullptr);
    if (!mesh) return;
    CHECK(nearVec3(mesh->getPosition(), 0.0f, 0.5f, 0.0f));
    CHECK(nearVec3(mesh->getGlobalPosition(), 5.0f, 2.5f, 0.0f));
}

} // namespace

int main()
{
    testFollow();
    testTreeEnterAndExit();
    testSeveralListeners();
    testExternalSkeleton();
    testOverridePose();
    testModelNodes();
    testMeshNamedAsBone();

    if (failures) printf("%d checks failed\n", failures);
    else
        printf("ok\n");
    return failures ? 1 : 0;
}
