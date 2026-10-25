#include "Check.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <string.h>

using namespace prisma3d;

namespace
{

class Counter : public Node
{
public:
    int processed = 0;
    int physics = 0;
    int paused = 0;
    int unpaused = 0;
    int disabled = 0;
    int enabled = 0;

    Counter()
    {
        setProcess(true);
        setPhysicsProcess(true);
    }

protected:
    void notificationHandler(int what) override
    {
        switch (what)
        {
            case kNotificationProcess:
                ++processed;
                break;
            case kNotificationPhysicsProcess:
                ++physics;
                break;
            case kNotificationPaused:
                ++paused;
                break;
            case kNotificationUnpaused:
                ++unpaused;
                break;
            case kNotificationDisabled:
                ++disabled;
                break;
            case kNotificationEnabled:
                ++enabled;
                break;
            default:
                break;
        }
        Node::notificationHandler(what);
    }
};

void step(SceneTree& tree)
{
    tree.process(0.1);
    tree.physicsProcess(0.1);
}

void testProcessMode(SceneTree& tree)
{
    Node* root = tree.getRoot();
    CHECK(root->getProcessMode() == Node::ProcessMode::Pausable);

    Counter* a = root->createChild<Counter>();
    Counter* b = a->createChild<Counter>();
    Counter* c = a->createChild<Counter>();
    Counter* d = root->createChild<Counter>();

    CHECK(a->getProcessMode() == Node::ProcessMode::Inherit);
    CHECK(a->canProcess() && b->canProcess() && a->isEnabled() && b->isEnabled());
    step(tree);
    CHECK(a->processed == 1 && b->processed == 1 && a->physics == 1);

    c->setProcessMode(Node::ProcessMode::Always);
    a->setProcessMode(Node::ProcessMode::Disabled);
    CHECK(a->getProcessMode() == Node::ProcessMode::Disabled);
    CHECK(!a->canProcess() && !a->isEnabled());
    CHECK(!b->canProcess() && !b->isEnabled());
    CHECK(c->canProcess() && c->isEnabled());
    CHECK(d->canProcess() && d->isEnabled());
    CHECK(a->disabled == 1 && b->disabled == 1 && c->disabled == 0 && d->disabled == 0);
    CHECK(a->paused == 1 && b->paused == 1 && c->paused == 0);
    step(tree);
    CHECK(a->processed == 1 && b->processed == 1 && c->processed == 2 && d->processed == 2);
    CHECK(a->physics == 1 && b->physics == 1 && c->physics == 2);

    a->setProcessMode(Node::ProcessMode::Disabled);
    CHECK(a->disabled == 1);
    a->setProcessMode(Node::ProcessMode::Inherit);
    CHECK(a->canProcess() && b->canProcess() && a->isEnabled());
    CHECK(a->enabled == 1 && b->enabled == 1 && a->unpaused == 1 && b->unpaused == 1);
    step(tree);
    CHECK(a->processed == 2 && b->processed == 2);

    tree.setPaused(true);
    CHECK(a->paused == 2 && b->paused == 2 && d->paused == 1);
    CHECK(c->paused == 0);
    step(tree);
    CHECK(a->processed == 2 && d->processed == 3 && c->processed == 4);
    CHECK(!a->canProcess() && c->canProcess());

    Counter* whenPaused = root->createChild<Counter>();
    whenPaused->setProcessMode(Node::ProcessMode::WhenPaused);
    CHECK(whenPaused->canProcess());
    CHECK(whenPaused->unpaused == 1);
    step(tree);
    CHECK(whenPaused->processed == 1);
    tree.setPaused(false);
    CHECK(whenPaused->paused == 1 && a->unpaused == 2 && c->unpaused == 0);
    CHECK(!whenPaused->canProcess() && a->canProcess());
    step(tree);
    CHECK(whenPaused->processed == 1 && a->processed == 3);

    Node3D orphan;
    orphan.setName("orphan");
    CHECK(!orphan.canProcess() && !orphan.isEnabled());
    orphan.setProcessMode(Node::ProcessMode::Disabled);
    CHECK(orphan.getProcessMode() == Node::ProcessMode::Disabled);

    Counter* lateParent = root->createChild<Counter>();
    lateParent->setProcessMode(Node::ProcessMode::Disabled);
    Counter* late = lateParent->createChild<Counter>();
    CHECK(!late->canProcess() && !late->isEnabled());
    late->setProcessMode(Node::ProcessMode::Pausable);
    CHECK(late->canProcess() && late->isEnabled());
    CHECK(late->enabled == 1);

    a->queueFree();
    d->queueFree();
    whenPaused->queueFree();
    lateParent->queueFree();
    tree.process(0.1);
    tree.physicsProcess(0.1);
}

void testGroups(SceneTree& tree)
{
    Node* root = tree.getRoot();
    Node* a = root->createChild<Node>();
    a->setName("a");
    Node* b = root->createChild<Node>();
    b->setName("b");
    Node* nested = a->createChild<Node>();
    nested->setName("nested");

    CHECK(!tree.hasGroup("enemies"));
    b->addToGroup("enemies");
    nested->addToGroup("enemies", true);
    a->addToGroup("enemies", true);
    CHECK(a->isInGroup("enemies") && !a->isInGroup("allies"));
    CHECK(tree.hasGroup("enemies") && tree.getNodeCountInGroup("enemies") == 3);
    ct::Vector<Node*> found;
    tree.getNodesInGroup("enemies", found);
    CHECK(found.size() == 3 && found[0] == a && found[1] == nested && found[2] == b);
    CHECK(tree.getFirstNodeInGroup("enemies") == a);
    CHECK(tree.getFirstNodeInGroup("missing") == nullptr);

    a->addToGroup("enemies", false);
    ct::Vector<Node::GroupInfo> groups;
    a->getGroups(groups);
    CHECK(groups.size() == 1 && groups[0].name == "enemies" && groups[0].persistent);
    CHECK(a->getPersistentGroupCount() == 1 && b->getPersistentGroupCount() == 0);

    a->addToGroup("");
    a->addToGroup("boss", true);
    groups.clear();
    a->getGroups(groups);
    CHECK(groups.size() == 2 && groups[1].name == "boss");

    root->moveChild(b, 0);
    tree.getNodesInGroup("enemies", found);
    CHECK(found.size() == 3 && found[0] == b && found[1] == a && found[2] == nested);

    a->removeFromGroup("enemies");
    CHECK(!a->isInGroup("enemies") && tree.getNodeCountInGroup("enemies") == 2);
    a->removeFromGroup("unknown");

    a->reparent(b);
    CHECK(tree.getNodeCountInGroup("enemies") == 2);
    CHECK(tree.getNodeCountInGroup("boss") == 1);

    Node loose;
    loose.setName("loose");
    loose.addToGroup("late", true);
    CHECK(!tree.hasGroup("late"));
    CHECK(loose.isInGroup("late"));

    Node* spare = root->createChild<Node>();
    spare->addToGroup("late", true);
    CHECK(tree.hasGroup("late"));
    spare->reparent(a);
    CHECK(tree.hasGroup("late"));
    spare->queueFree();
    tree.process(0.1);
    CHECK(!tree.hasGroup("late"));

    b->queueFree();
    tree.process(0.1);
    tree.physicsProcess(0.1);
    CHECK(!tree.hasGroup("enemies") && !tree.hasGroup("boss"));
}

void testSaving(SceneTree& tree)
{
    Node* root = tree.getRoot();
    Node3D* scene = root->createChild<Node3D>();
    scene->setName("Scene");
    Node3D* child = scene->createChild<Node3D>();
    child->setName("Child");
    child->addToGroup("pickups", true);
    child->addToGroup("glow", true);
    child->addToGroup("runtime", false);
    child->setProcessMode(Node::ProcessMode::Disabled);
    Node3D* plain = scene->createChild<Node3D>();
    plain->setName("Plain");
    plain->setProcessMode(Node::ProcessMode::Always);

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(scene, text, &saved));
    CHECK(saved.success());
    CHECK(text.find("\"groups\"") != ct::String::npos);
    CHECK(text.find("pickups") != ct::String::npos && text.find("glow") != ct::String::npos);
    CHECK(text.find("runtime") == ct::String::npos);
    CHECK(text.find("\"process_mode\"") != ct::String::npos);

    SceneResult loaded;
    Node* holder = root->createChild<Node3D>();
    holder->setName("holder");
    Node* copy = SceneSerializer::load(text, holder, loaded);
    CHECK(copy != nullptr && loaded.success());
    if (copy)
    {
        Node* reChild = copy->findChild("Child");
        Node* rePlain = copy->findChild("Plain");
        CHECK(reChild && rePlain);
        if (reChild && rePlain)
        {
            CHECK(reChild->isInGroup("pickups") && reChild->isInGroup("glow"));
            CHECK(!reChild->isInGroup("runtime"));
            CHECK(reChild->getPersistentGroupCount() == 2);
            CHECK(reChild->getProcessMode() == Node::ProcessMode::Disabled);
            CHECK(rePlain->getProcessMode() == Node::ProcessMode::Always);
            CHECK(!reChild->canProcess() && rePlain->canProcess());
            CHECK(rePlain->getPersistentGroupCount() == 0);
        }
        CHECK(tree.getNodeCountInGroup("pickups") == 2);
    }

    ct::String again;
    CHECK(SceneSerializer::save(copy, again, nullptr));
    CHECK(again == text);

    const std::size_t at = text.find("\"pickups\"");
    CHECK(at != ct::String::npos);
    if (at != ct::String::npos)
    {
        const ct::String broken = text.substr(0, at) + "7" + text.substr(at + 9);
        SceneResult result;
        Node* odd = SceneSerializer::load(broken, holder, result);
        CHECK(odd != nullptr && result.success() && result.warningCount() == 1);
        if (odd)
        {
            Node* oddChild = odd->findChild("Child");
            CHECK(oddChild && !oddChild->isInGroup("pickups") && oddChild->isInGroup("glow"));
            odd->queueFree();
        }
    }

    scene->queueFree();
    holder->queueFree();
    tree.process(0.1);
    tree.physicsProcess(0.1);
}

void testBodies(SceneTree& tree)
{
    Node* root = tree.getRoot();
    StaticBody3D* wall = root->createChild<StaticBody3D>();
    wall->setName("Wall");
    CHECK(wall->isEnabled());
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    CHECK(server != nullptr);
    if (!server) return;

    wall->setProcessMode(Node::ProcessMode::Disabled);
    CHECK(!wall->isEnabled());
    wall->setProcessMode(Node::ProcessMode::Inherit);
    CHECK(wall->isEnabled());

    Node3D* group = root->createChild<Node3D>();
    group->setName("Group");
    StaticBody3D* inner = group->createChild<StaticBody3D>();
    inner->setName("Inner");
    group->setProcessMode(Node::ProcessMode::Disabled);
    CHECK(!inner->isEnabled());
    inner->setProcessMode(Node::ProcessMode::Always);
    CHECK(inner->isEnabled());
    group->setProcessMode(Node::ProcessMode::Inherit);

    wall->queueFree();
    group->queueFree();
    tree.process(0.1);
    tree.physicsProcess(0.1);
}

bool shown(const Node* node, const char* name)
{
    for (ct::String cls(node->getClassName()); !cls.empty(); cls = NodeClassDB::getParentClass(cls))
    {
        ct::Vector<PropertyInfo> list;
        NodeClassDB::getInstancePropertyList(node, cls, list, true);
        for (std::size_t i = 0; i < list.size(); ++i)
            if (list[i].name == name) return (list[i].usage & PropertyUsage::Editor) != 0;
    }
    return false;
}

void testConditionalProperties(SceneTree& tree)
{
    Node* root = tree.getRoot();

    OmniLight3D* omni = root->createChild<OmniLight3D>();
    CHECK(!omni->hasShadow());
    const std::uint32_t revision = omni->getPropertyListRevision();
    CHECK(shown(omni, "shadow_enabled") && shown(omni, "light_energy") &&
            shown(omni, "omni_range"));
    CHECK(!shown(omni, "shadow_bias") && !shown(omni, "shadow_normal_bias") &&
            !shown(omni, "shadow_opacity") && !shown(omni, "shadow_blur") &&
            !shown(omni, "omni_shadow_mode"));
    omni->setShadow(true);
    CHECK(omni->getPropertyListRevision() != revision);
    CHECK(shown(omni, "shadow_bias") && shown(omni, "shadow_blur") &&
            shown(omni, "omni_shadow_mode"));
    omni->setShadow(false);
    CHECK(!shown(omni, "shadow_bias"));

    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    CHECK(!shown(sun, "directional_shadow_mode") &&
            !shown(sun, "directional_shadow_max_distance") && !shown(sun, "shadow_bias"));
    sun->setShadow(true);
    CHECK(shown(sun, "directional_shadow_mode") && shown(sun, "directional_shadow_max_distance") &&
            shown(sun, "shadow_bias"));
    sun->setShadowMode(DirectionalShadowMode::Orthogonal);
    CHECK(!shown(sun, "directional_shadow_split_1"));
    sun->setShadow(false);
    CHECK(!shown(sun, "directional_shadow_mode"));

    SpotLight3D* spot = root->createChild<SpotLight3D>();
    CHECK(!shown(spot, "shadow_bias"));
    spot->setShadow(true);
    CHECK(shown(spot, "shadow_bias"));

    Decal3D* decal = root->createChild<Decal3D>();
    CHECK(!shown(decal, "distance_fade_begin") && !shown(decal, "distance_fade_length"));
    CHECK(shown(decal, "distance_fade_enabled") && shown(decal, "albedo_mix"));
    decal->setDistanceFadeEnabled(true);
    CHECK(shown(decal, "distance_fade_begin") && shown(decal, "distance_fade_length"));

    WorldEnvironment* environment = root->createChild<WorldEnvironment>();
    EnvironmentSettings settings = environment->getEnvironment();
    settings.fogEnabled = false;
    settings.glowEnabled = false;
    environment->setEnvironment(settings);
    CHECK(shown(environment, "fog_enabled") && shown(environment, "glow_enabled"));
    CHECK(!shown(environment, "fog_density") && !shown(environment, "fog_light_color") &&
            !shown(environment, "glow_intensity") && !shown(environment, "glow_bloom"));
    settings.fogEnabled = true;
    environment->setEnvironment(settings);
    CHECK(shown(environment, "fog_density") && shown(environment, "fog_light_color") &&
            !shown(environment, "glow_bloom"));
    settings.glowEnabled = true;
    environment->setEnvironment(settings);
    CHECK(shown(environment, "glow_bloom"));

    Node3D* body = root->createChild<Node3D>();
    CHECK(shown(body, "collision_enabled") && !shown(body, "collision_type"));
    body->setCollisionEnabled(true);
    CHECK(shown(body, "collision_type"));
    body->setCollisionEnabled(false);
    CHECK(!shown(body, "collision_type"));
    CHECK(!shown(body, "collision_shape") && !shown(body, "collision_radii") &&
            !shown(body, "collision_box_min") && !shown(body, "collision_mesh"));
    body->setCollisionType(2);
    CHECK(shown(body, "collision_shape"));
    CHECK(!shown(body, "collision_radii"));
    body->setCollisionShape(CollisionShapeType::Ellipsoid);
    CHECK(shown(body, "collision_radii") && !shown(body, "collision_box_min"));
    body->setCollisionType(0);
    CHECK(!shown(body, "collision_shape") && !shown(body, "collision_radii"));

    omni->queueFree();
    sun->queueFree();
    spot->queueFree();
    decal->queueFree();
    environment->queueFree();
    body->queueFree();
    tree.process(0.1);
    tree.physicsProcess(0.1);
}

} // namespace

int main()
{
    PhysicsServer3D physics;
    SceneTree tree;
    tree.initialize();
    registerSceneTypes();

    testProcessMode(tree);
    testGroups(tree);
    testSaving(tree);
    testBodies(tree);
    testConditionalProperties(tree);

    tree.finalize();

    if (failures) printf("%d failures\n", failures);
    return failures ? 1 : 0;
}
