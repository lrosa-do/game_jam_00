#include "Check.h"

#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/AudioListener3D.h"
#include "prisma3d/scene/AudioStreamPlayer.h"
#include "prisma3d/scene/AudioStreamPlayer3D.h"
#include "prisma3d/scene/BoneAttachment3D.h"
#include "prisma3d/scene/BoostJet3D.h"
#include "prisma3d/scene/Button.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/CarDamage3D.h"
#include "prisma3d/scene/CharacterBody3D.h"
#include "prisma3d/scene/CharacterRigidBody3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/ColorRect.h"
#include "prisma3d/scene/Control.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Destructible3D.h"
#include "prisma3d/scene/Dismember3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/GroundVehicle3D.h"
#include "prisma3d/scene/Hair3D.h"
#include "prisma3d/scene/ImpactFx3D.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/Label.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/LineEdit.h"
#include "prisma3d/scene/ManualMesh3D.h"
#include "prisma3d/scene/MissingNode.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/NavigationRegion3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Ocean3D.h"
#include "prisma3d/scene/ParticleCollision3D.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/PlanarReflector3D.h"
#include "prisma3d/scene/Ragdoll3D.h"
#include "prisma3d/scene/ReflectionProbe3D.h"
#include "prisma3d/scene/RibbonTrail3D.h"
#include "prisma3d/scene/Road3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/SkeletonModifier3D.h"
#include "prisma3d/scene/SkidMarks3D.h"
#include "prisma3d/scene/SoftBody3D.h"
#include "prisma3d/scene/SpringBoneCollision3D.h"
#include "prisma3d/scene/SpringBoneSimulator3D.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/TextureRect.h"
#include "prisma3d/scene/Traffic3D.h"
#include "prisma3d/scene/Tree3D.h"
#include "prisma3d/scene/Vehicle3D.h"
#include "prisma3d/scene/VehicleDriver3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/Water3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <stddef.h>
#include <string.h>
#include <type_traits>

using namespace prisma3d;

namespace
{

template<typename... Ts>
struct TypeList
{
};

using AllClasses = TypeList<Node, MissingNode, Node3D, VisualInstance3D, MeshInstance3D, SoftBody3D,
        DebrisMesh3D, ManualMesh3D, MultiMeshInstance3D, ParticleSystem3D, RibbonTrail3D, Hair3D,
        Tree3D, Road3D, Ocean3D, Water3D, Camera3D, Light3D, DirectionalLight3D, OmniLight3D,
        SpotLight3D, Skeleton3D, BoneAttachment3D, SkeletonModifier3D, SpringBoneSimulator3D,
        SpringBoneCollision3D, SpringBoneCollisionSphere3D, SpringBoneCollisionCapsule3D,
        SpringBoneCollisionPlane3D, Ragdoll3D, Dismember3D, NavigationRegion3D, Decal3D,
        ReflectionProbe3D, LightmapGI, PlanarReflector3D, AudioListener3D, Control, Label,
        ColorRect, TextureRect, Button, LineEdit, AudioStreamPlayer3D, Path3D, PathFollow3D,
        Terrain3D, Grass3D, Forest3D, ParticleCollision3D, ParticleCollisionSphere3D,
        ParticleCollisionBox3D, CollisionShape3D, PhysicsBody3D, StaticBody3D, AnimatableBody3D,
        RigidBody3D, CharacterBody3D, CharacterRigidBody3D, VehicleBody3D, GroundVehicle3D,
        CarDamage3D, SkidMarks3D, ImpactFx3D, VehicleDriver3D, BoostJet3D, Traffic3D,
        DestructibleBody3D, DebrisBody3D, VehicleWheel3D, Joint3D, PinJoint3D, HingeJoint3D,
        SliderJoint3D, DistanceJoint3D, FixedJoint3D, PistonJoint3D, UniversalJoint3D, WheelJoint3D,
        ConeTwistJoint3D, MouseJoint3D, Viewport, Window, WorldEnvironment, AnimationPlayer,
        AudioStreamPlayer>;

size_t castChecks = 0;
size_t createdByName = 0;
size_t abstractCount = 0;

template<typename T>
const char* nameOf()
{
    return T::classNameStatic();
}

template<typename From, typename To>
void checkCast(Node* node)
{
    const bool expected = std::is_base_of<To, From>::value;
    To* cast = node->castTo<To>();
    CHECK((cast != nullptr) == expected);
    if (cast) CHECK(static_cast<Node*>(cast) == node);
    const Node* constNode = node;
    const To* constCast = constNode->castTo<To>();
    CHECK((constCast != nullptr) == expected);
    CHECK(node->isClass(nameOf<To>()) == expected);
    castChecks += 3;
}

template<typename From, typename... Tos>
void checkCastsTo(Node* node, TypeList<Tos...>)
{
    (checkCast<From, Tos>(node), ...);
}

template<typename From>
void checkClass(Node* root)
{
    const ct::String name = nameOf<From>();
    CHECK(NodeClassDB::classExists(name));
    CHECK(NodeClassDB::isParentClass(name, name));
    CHECK(NodeClassDB::isParentClass(name, "Node"));

    if constexpr (std::is_same<From, Node>::value)
    {
        CHECK(NodeClassDB::getParentClass(name) == "");
    }
    else
    {
        CHECK(NodeClassDB::getParentClass(name) == ct::String(From::SuperType::classNameStatic()));
        CHECK(NodeClassDB::isParentClass(name, From::SuperType::classNameStatic()));
        CHECK((std::is_base_of<typename From::SuperType, From>::value));
    }

    if (NodeClassDB::isAbstract(name))
    {
        ++abstractCount;
        CHECK(!NodeClassDB::canInstantiate(name));
        return;
    }

    CHECK(NodeClassDB::canInstantiate(name));
    const size_t before = root->getChildCount();
    Node* child = root->createChild(name.c_str());
    CHECK(child != nullptr);
    if (!child) return;
    ++createdByName;
    CHECK(root->getChildCount() == before + 1);
    CHECK(child->getParent() == root);
    CHECK(root->getChild(before) == child);
    CHECK(name == child->getClassName());
    CHECK(child->isClass(name.c_str()));
    CHECK(child->derivesFrom<From>());
    checkCastsTo<From>(child, AllClasses());
}

template<typename... Ts>
void checkAll(Node* root, TypeList<Ts...>)
{
    (checkClass<Ts>(root), ...);
}

template<typename... Ts>
size_t countOf(TypeList<Ts...>)
{
    return sizeof...(Ts);
}

bool contains(const ct::Vector<ct::String>& list, const char* name)
{
    for (size_t i = 0; i < list.size(); ++i)
    {
        if (list[i] == name) return true;
    }
    return false;
}

void testRegisteredClasses()
{
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    const size_t total = countOf(AllClasses());
    ct::Vector<ct::String> classes;
    NodeClassDB::getClassList(classes);
    CHECK(classes.size() == total);
    CHECK(classes.size() > 0 && classes[0] == "Node");

    checkAll(root, AllClasses());

    CHECK(abstractCount == 8);
    CHECK(createdByName == total - abstractCount);
    CHECK(NodeClassDB::isAbstract("Joint3D"));
    CHECK(NodeClassDB::isAbstract("Light3D"));
    CHECK(NodeClassDB::isAbstract("ParticleCollision3D"));
    CHECK(NodeClassDB::isAbstract("PhysicsBody3D"));
    CHECK(NodeClassDB::isAbstract("VisualInstance3D"));
    CHECK(NodeClassDB::isAbstract("SkeletonModifier3D"));
    CHECK(NodeClassDB::isAbstract("SpringBoneCollision3D"));
    CHECK(root->getChildCount() == createdByName);
    // Nodes such as ImpactFx3D build internal children (their particle systems) when created.
    size_t internal = 0;
    for (size_t i = 0; i < root->getChildCount(); ++i)
    {
        Node* child = root->getChild(i);
        for (size_t j = 0; j < child->getChildCount(); ++j)
        {
            CHECK(child->getChild(j)->isInternal());
            ++internal;
        }
    }
    CHECK(tree.getNodeCount() == createdByName + internal + 1);

    for (size_t i = 0; i < root->getChildCount(); ++i) root->getChild(i)->queueFree();
    tree.process(0.1);
    CHECK(root->getChildCount() == 0);
    CHECK(tree.getNodeCount() == 1);

    printf("classes %zu, abstract %zu, created by name %zu, castTo checks %zu\n", total,
            abstractCount, createdByName, castChecks);
}

void testCreateRefused()
{
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    CHECK(root->createChild("NoSuchClass") == nullptr);
    CHECK(root->createChild("") == nullptr);
    CHECK(root->createChild(nullptr) == nullptr);
    CHECK(root->createChild("Light3D") == nullptr);
    CHECK(root->createChild("PhysicsBody3D") == nullptr);
    CHECK(root->getChildCount() == 0);
    CHECK(tree.getNodeCount() == 1);

    CHECK(!NodeClassDB::classExists("NoSuchClass"));
    CHECK(!NodeClassDB::canInstantiate("NoSuchClass"));
    CHECK(!NodeClassDB::isAbstract("NoSuchClass"));
    CHECK(NodeClassDB::getParentClass("NoSuchClass") == "");
    CHECK(!NodeClassDB::isParentClass("NoSuchClass", "Node"));
    CHECK(!NodeClassDB::isParentClass("Node3D", "NoSuchClass"));
}

void testQueries()
{
    CHECK(NodeClassDB::isParentClass("SpotLight3D", "Node3D"));
    CHECK(NodeClassDB::isParentClass("SpotLight3D", "Light3D"));
    CHECK(NodeClassDB::isParentClass("SpotLight3D", "SpotLight3D"));
    CHECK(!NodeClassDB::isParentClass("Node3D", "SpotLight3D"));
    CHECK(!NodeClassDB::isParentClass("SpotLight3D", "OmniLight3D"));
    CHECK(!NodeClassDB::isParentClass("AnimationPlayer", "Node3D"));
    CHECK(NodeClassDB::getParentClass("MeshInstance3D") == "VisualInstance3D");
    CHECK(NodeClassDB::getParentClass("Window") == "Viewport");
    CHECK(NodeClassDB::getParentClass("Node") == "");

    ct::Vector<ct::String> inheriters;
    NodeClassDB::getInheritersFromClass("Node3D", inheriters);
    CHECK(contains(inheriters, "MeshInstance3D"));
    CHECK(contains(inheriters, "SpotLight3D"));
    CHECK(contains(inheriters, "VehicleBody3D"));
    CHECK(contains(inheriters, "Ocean3D"));
    CHECK(!contains(inheriters, "AnimationPlayer"));
    CHECK(!contains(inheriters, "Viewport"));
    CHECK(!contains(inheriters, "Node3D"));
    CHECK(!contains(inheriters, "Node"));

    ct::Vector<ct::String> everything;
    NodeClassDB::getInheritersFromClass("Node", everything);
    CHECK(everything.size() == countOf(AllClasses()) - 1);

    ct::Vector<ct::String> none;
    NodeClassDB::getInheritersFromClass("SpotLight3D", none);
    CHECK(none.size() == 0);

    ct::Vector<ct::String> lights;
    NodeClassDB::getDirectInheritersFromClass("Light3D", lights);
    CHECK(lights.size() == 3);
    CHECK(contains(lights, "DirectionalLight3D"));
    CHECK(contains(lights, "OmniLight3D"));
    CHECK(contains(lights, "SpotLight3D"));

    ct::Vector<ct::String> direct;
    NodeClassDB::getDirectInheritersFromClass("Node3D", direct);
    CHECK(contains(direct, "VisualInstance3D"));
    CHECK(contains(direct, "Light3D"));
    CHECK(!contains(direct, "MeshInstance3D"));
}

void testRegisterTwice()
{
    ct::Vector<ct::String> before;
    NodeClassDB::getClassList(before);
    registerSceneTypes();
    registerSceneTypes();
    NodeClassDB::registerClass<Node3D>();
    NodeClassDB::registerAbstractClass<Light3D>();
    ct::Vector<ct::String> after;
    NodeClassDB::getClassList(after);
    CHECK(before.size() == after.size());
    CHECK(before.size() == countOf(AllClasses()));
    for (size_t i = 0; i < before.size() && i < after.size(); ++i) CHECK(before[i] == after[i]);
    CHECK(NodeClassDB::isAbstract("Light3D"));
    CHECK(!NodeClassDB::isAbstract("Node3D"));
}

void testIsClassByName()
{
    SceneTree tree;
    tree.initialize();
    MeshInstance3D* mesh = tree.getRoot()->createChild<MeshInstance3D>();
    AnimationPlayer* player = tree.getRoot()->createChild<AnimationPlayer>();
    CHECK(mesh->isClass("MeshInstance3D"));
    CHECK(mesh->isClass("VisualInstance3D"));
    CHECK(mesh->isClass("Node3D"));
    CHECK(mesh->isClass("Node"));
    CHECK(!mesh->isClass("Light3D"));
    CHECK(!mesh->isClass("NoSuchClass"));
    CHECK(!mesh->isClass(nullptr));
    CHECK(player->isClass("Node"));
    CHECK(!player->isClass("Node3D"));
    CHECK(mesh->isClass(NodeClass::Node));
    CHECK(player->isClass(NodeClass::Node));
    CHECK(mesh->isClass(NodeClass::MeshInstance3D));
    CHECK(mesh->isClass(NodeClass::VisualInstance3D));
    CHECK(mesh->isClass(NodeClass::Node3D));
    CHECK(!mesh->isClass(NodeClass::PhysicsBody3D));
    CHECK(!player->isClass(NodeClass::Node3D));

    static_assert(Camera3D::kAncestralClass == Node3D::kAncestralClass, "no own bit");
    static_assert(Light3D::kAncestralClass == Node3D::kAncestralClass, "no own bit");
    static_assert(SoftBody3D::kAncestralClass == MeshInstance3D::kAncestralClass, "no own bit");
    static_assert(VehicleBody3D::kAncestralClass == PhysicsBody3D::kAncestralClass, "no own bit");
    static_assert(AnimationPlayer::kAncestralClass == NodeClass::Node, "no own bit");
    static_assert(MeshInstance3D::kAncestralClass != VisualInstance3D::kAncestralClass, "own bit");
}

void testWaterAndOcean()
{
    CHECK(NodeClassDB::classExists("Ocean3D"));
    CHECK(NodeClassDB::classExists("Water3D"));
    CHECK(NodeClassDB::getParentClass("Ocean3D") == "VisualInstance3D");
    CHECK(NodeClassDB::getParentClass("Water3D") == "VisualInstance3D");

    SceneTree tree;
    tree.initialize();
    Ocean3D* ocean = tree.getRoot()->createChild<Ocean3D>();
    Water3D* water = tree.getRoot()->createChild<Water3D>();
    Node* asOcean = ocean;
    Node* asWater = water;
    CHECK(asOcean->castTo<Ocean3D>() == ocean);
    CHECK(asOcean->castTo<VisualInstance3D>() == ocean);
    CHECK(asOcean->castTo<Node3D>() == ocean);
    CHECK(asOcean->castTo<Water3D>() == nullptr);
    CHECK(asWater->castTo<Water3D>() == water);
    CHECK(asWater->castTo<Ocean3D>() == nullptr);
    CHECK(asWater->castTo<VisualInstance3D>() == water);
    CHECK(ct::String(asOcean->getClassName()) == "Ocean3D");
    CHECK(ct::String(asWater->getClassName()) == "Water3D");
}

class CustomNode : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(CustomNode, Node3D);
    int value = 7;
};

class CustomChild : public CustomNode
{
public:
    PRISMA3D_NODE_CLASS(CustomChild, CustomNode);
};

void testCustomClasses()
{
    CHECK(!NodeClassDB::classExists("CustomChild"));
    CHECK(!NodeClassDB::classExists("CustomNode"));

    NodeClassDB::registerClass<CustomChild>();
    CHECK(NodeClassDB::classExists("CustomChild"));
    CHECK(NodeClassDB::classExists("CustomNode"));
    CHECK(NodeClassDB::getParentClass("CustomChild") == "CustomNode");
    CHECK(NodeClassDB::getParentClass("CustomNode") == "Node3D");
    CHECK(NodeClassDB::isAbstract("CustomNode"));
    CHECK(NodeClassDB::canInstantiate("CustomChild"));
    CHECK(NodeClassDB::isParentClass("CustomChild", "Node3D"));

    NodeClassDB::registerClass<CustomNode>();
    CHECK(!NodeClassDB::isAbstract("CustomNode"));

    SceneTree tree;
    tree.initialize();
    Node* made = tree.getRoot()->createChild("CustomChild");
    CHECK(made != nullptr);
    if (made)
    {
        CHECK(made->getParent() == tree.getRoot());
        CHECK(made->castTo<CustomChild>() != nullptr);
        CHECK(made->castTo<CustomNode>() != nullptr);
        CHECK(made->castTo<Node3D>() != nullptr);
        CHECK(made->castTo<Camera3D>() == nullptr);
        CHECK(made->isClass("CustomNode"));
        CHECK(ct::String(made->getClassName()) == "CustomChild");
    }
    Node* base = tree.getRoot()->createChild("CustomNode");
    CHECK(base != nullptr);
    if (base)
    {
        CHECK(base->castTo<CustomChild>() == nullptr);
        CHECK(base->castTo<CustomNode>() != nullptr);
    }
}

} // namespace

int main()
{
    testRegisteredClasses();
    testCreateRefused();
    testQueries();
    testRegisterTwice();
    testIsClassByName();
    testWaterAndOcean();
    testCustomClasses();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
