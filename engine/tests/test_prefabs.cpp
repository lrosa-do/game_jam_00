#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/MissingNode.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"

#include <ct/json.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

ct::String gDir;

ct::String filePath(const char* name)
{
    return gDir + "/" + name;
}

bool hasDiagnostic(const SceneResult& result, SceneDiagnosticSeverity severity,
        const char* pathFragment, const char* messageFragment)
{
    for (size_t i = 0; i < result.diagnostics.size(); ++i)
    {
        const SceneDiagnostic& diagnostic = result.diagnostics[i];
        if (diagnostic.severity != severity) continue;
        if (pathFragment && !strstr(diagnostic.path.c_str(), pathFragment)) continue;
        if (!messageFragment || strstr(diagnostic.message.c_str(), messageFragment)) return true;
    }
    return false;
}

void expectClean(const SceneResult& result, const char* what)
{
    if (result.diagnostics.size() == 0) return;
    printf("%s:\n%s", what, result.describe().c_str());
    ++failures;
}

bool sameVec3(const Math::Vec3& a, float x, float y, float z)
{
    return a.x == x && a.y == y && a.z == z;
}

void makeMast(const ct::String& path, float energy, float poleDepth = -1.0f,
        Node* reloadRoot = nullptr)
{
    Node3D scratch;
    OmniLight3D* lamp = scratch.createChild<OmniLight3D>();
    lamp->setName("lamp");
    lamp->setEnergy(energy);
    lamp->setRange(8.0f);
    Node3D* pole = lamp->createChild<Node3D>();
    pole->setName("pole");
    pole->setPosition(Math::Vec3(0.0f, poleDepth, 0.0f));
    Node3D* base = lamp->createChild<Node3D>();
    base->setName("base");
    Node3D* foot = base->createChild<Node3D>();
    foot->setName("foot");
    foot->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));

    SceneResult result;
    CHECK(Prefab::saveFromNode(lamp, path, result, reloadRoot));
    expectClean(result, "make mast");
}

Node3D* place(const ct::String& path, Node* parent, const char* name, float x)
{
    SceneResult result;
    Node* node = Prefab::instantiate(path, parent, result);
    expectClean(result, "place");
    CHECK(node != nullptr);
    if (!node) return nullptr;
    node->setName(name);
    Node3D* spatial = node->castTo<Node3D>();
    CHECK(spatial != nullptr);
    if (spatial) spatial->setPosition(Math::Vec3(x, 0.0f, 0.0f));
    return spatial;
}

OmniLight3D* lampOf(Node* instance)
{
    return instance ? instance->castTo<OmniLight3D>() : nullptr;
}

Node3D* poleOf(Node* instance)
{
    Node* pole = instance ? instance->findChild("pole") : nullptr;
    return pole ? pole->castTo<Node3D>() : nullptr;
}

ct::Json readJson(const ct::String& text)
{
    return ct::Json::parse(text);
}

const ct::Json* childEntry(const ct::Json& entry, size_t index)
{
    const ct::Json* children = entry.find("children");
    if (!children || !children->is_array() || index >= children->size()) return nullptr;
    return &children->at(index);
}

bool hasKeys(const ct::Json& object, const char* a, const char* b = nullptr,
        const char* c = nullptr)
{
    if (!object.is_object()) return false;
    size_t expected = 1 + (b ? 1 : 0) + (c ? 1 : 0);
    if (object.size() != expected) return false;
    if (!object.find(a)) return false;
    if (b && !object.find(b)) return false;
    if (c && !object.find(c)) return false;
    return true;
}

void testSaveAndInstantiate()
{
    const ct::String mast = filePath("mast.prefab.json");
    Prefab::clearCache();
    makeMast(mast, 2.5f);
    CHECK(Prefab::isCached(mast));

    Node3D world;
    world.setName("world");
    Node3D* holder = world.createChild<Node3D>();
    holder->setName("holder");

    const size_t reads = Prefab::fileReadCount();
    Node3D* m1 = place(mast, holder, "m1", 1.0f);
    Node3D* m2 = place(mast, holder, "m2", 2.0f);
    Node3D* m3 = place(mast, holder, "m3", 3.0f);
    CHECK(Prefab::fileReadCount() == reads);
    CHECK(holder->getChildCount() == 3);
    if (!m1 || !m2 || !m3) return;

    for (Node3D* instance: { m1, m2, m3 })
    {
        CHECK(instance->isInstance());
        CHECK(instance->getSceneFilePath() == mast);
        CHECK(instance->getOwner() == nullptr);
        CHECK(strcmp(instance->getClassName(), "OmniLight3D") == 0);
        CHECK(instance->getChildCount() == 2);
        CHECK(lampOf(instance) && lampOf(instance)->getEnergy() == 2.5f);
        Node* foot = instance->getNode("base/foot");
        CHECK(foot != nullptr);
        if (!foot) continue;
        CHECK(foot->getOwner() == instance);
        CHECK(instance->findChild("base")->getOwner() == instance);
        CHECK(!foot->isInstance());
    }

    lampOf(m1)->setEnergy(5.0f);
    m1->setVisible(false);
    poleOf(m3)->setPosition(Math::Vec3(0.0f, -7.0f, 0.0f));

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(&world, text, &saved));
    expectClean(saved, "scene with instances");

    CHECK(strstr(text.c_str(), "pole") == nullptr);
    CHECK(strstr(text.c_str(), "foot") == nullptr);
    CHECK(strstr(text.c_str(), "OmniLight3D") == nullptr);
    CHECK(strstr(text.c_str(), "\"light_range\"") == nullptr);

    const ct::Json document = readJson(text);
    const ct::Json& worldEntry = document["root"];
    const ct::Json* holderEntry = childEntry(worldEntry, 0);
    CHECK(holderEntry && holderEntry->find("children") &&
            holderEntry->find("children")->size() == 3);
    if (!holderEntry || holderEntry->find("children")->size() != 3) return;

    const ct::Json* first = childEntry(*holderEntry, 0);
    const ct::Json* second = childEntry(*holderEntry, 1);
    const ct::Json* third = childEntry(*holderEntry, 2);
    CHECK(first && hasKeys(*first, "instance", "name", "properties"));
    CHECK(second && hasKeys(*second, "instance", "name", "properties"));
    CHECK(third && hasKeys(*third, "instance", "name", "properties"));
    if (!first || !second || !third) return;
    CHECK((*first)["instance"].str() == mast);
    CHECK((*first)["name"].str() == "m1");
    CHECK(first->find("class") == nullptr);
    CHECK(first->find("children") == nullptr);
    CHECK((*first)["properties"].size() == 3);
    CHECK((*first)["properties"].find("light_energy") != nullptr);
    CHECK((*first)["properties"].find("visible") != nullptr);
    CHECK((*first)["properties"].find("position") != nullptr);
    CHECK((*second)["properties"].size() == 1);
    CHECK((*second)["properties"].find("position") != nullptr);
    CHECK((*third)["properties"].size() == 1);

    Node3D loadedHost;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &loadedHost, loaded);
    expectClean(loaded, "read scene with instances");
    CHECK(copy != nullptr);
    if (!copy) return;
    Node* copyHolder = copy->findChild("holder");
    CHECK(copyHolder && copyHolder->getChildCount() == 3);
    if (!copyHolder || copyHolder->getChildCount() != 3) return;
    Node* c1 = copyHolder->getChild(0);
    Node* c2 = copyHolder->getChild(1);
    Node* c3 = copyHolder->getChild(2);
    CHECK(c1->getName() == "m1" && c2->getName() == "m2" && c3->getName() == "m3");
    CHECK(c1->isInstance() && c1->getSceneFilePath() == mast);
    CHECK(c1->castTo<Node3D>()->isVisible() == false);
    CHECK(c2->castTo<Node3D>()->isVisible() == true);
    CHECK(lampOf(c1)->getEnergy() == 5.0f);
    CHECK(lampOf(c2)->getEnergy() == 2.5f);
    CHECK(sameVec3(c2->castTo<Node3D>()->getPosition(), 2.0f, 0.0f, 0.0f));
    CHECK(poleOf(c1) && sameVec3(poleOf(c1)->getPosition(), 0.0f, -1.0f, 0.0f));
    CHECK(poleOf(c3) && sameVec3(poleOf(c3)->getPosition(), 0.0f, -1.0f, 0.0f));
    CHECK(c1->getNode("base/foot")->getOwner() == c1);

    ct::String direct;
    CHECK(SceneSerializer::save(&world, direct));
    ct::String reloaded;
    CHECK(SceneSerializer::save(copy, reloaded));
    CHECK(reloaded == direct);
}

void testAdditionsAndEditable()
{
    const ct::String mast = filePath("mast.prefab.json");
    Prefab::clearCache();
    makeMast(mast, 2.5f);

    Node3D world;
    world.setName("world");
    Node3D* m1 = place(mast, &world, "m1", 0.0f);
    if (!m1) return;

    Node3D* extra = m1->createChild<Node3D>();
    extra->setName("extra");
    extra->setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));
    Node3D* inner = extra->createChild<Node3D>();
    inner->setName("inner");
    m1->moveChild(extra, 1);
    CHECK(extra->getOwner() == nullptr);
    CHECK(m1->getChildCount() == 3);

    Node3D* gadget = poleOf(m1)->createChild<Node3D>();
    gadget->setName("gadget");
    lampOf(m1)->findChild("base")->castTo<Node3D>()->setPosition(Math::Vec3(0.0f, 4.0f, 0.0f));

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(&world, text, &saved));
    CHECK(hasDiagnostic(saved, SceneDiagnosticSeverity::Warning, "root.children[0]",
            "was added inside 'pole'"));
    CHECK(saved.errorCount() == 0);
    ct::Json document = readJson(text);
    const ct::Json& entry = *childEntry(document["root"], 0);
    CHECK(entry.find("overrides") == nullptr);
    CHECK(entry.find("editable") == nullptr);
    CHECK(entry.find("children") && entry.find("children")->size() == 1);
    CHECK(strstr(text.c_str(), "gadget") == nullptr);
    CHECK(strstr(text.c_str(), "\"base\"") == nullptr);

    world.setEditableInstance(m1, true);
    CHECK(world.isEditableInstance(m1));
    CHECK(!world.isEditableInstance(&world));
    CHECK(!m1->isEditableInstance(&world));

    ct::String editable;
    SceneResult editableSaved;
    CHECK(SceneSerializer::save(&world, editable, &editableSaved));
    expectClean(editableSaved, "editable instance");
    document = readJson(editable);
    const ct::Json& editableEntry = *childEntry(document["root"], 0);
    CHECK(editableEntry.find("editable") && editableEntry["editable"].as_bool());
    const ct::Json* overrides = editableEntry.find("overrides");
    CHECK(overrides && overrides->is_array() && overrides->size() == 2);
    if (!overrides || overrides->size() != 2) return;
    bool sawBase = false;
    bool sawPole = false;
    for (size_t i = 0; i < overrides->size(); ++i)
    {
        const ct::Json& item = overrides->at(i);
        if (item["path"].str() == "base")
        {
            sawBase = true;
            CHECK(item["properties"].find("position") != nullptr);
            CHECK(item.find("children") == nullptr);
        }
        if (item["path"].str() == "pole")
        {
            sawPole = true;
            CHECK(item.find("properties") == nullptr);
            CHECK(item["children"].size() == 1);
            CHECK(item["children"][0]["name"].str() == "gadget");
            CHECK(item["children"][0]["class"].str() == "Node3D");
        }
    }
    CHECK(sawBase && sawPole);
    const ct::Json* addedList = editableEntry.find("children");
    CHECK(addedList && addedList->size() == 1);
    if (!addedList || addedList->size() != 1) return;
    const ct::Json& added = addedList->at(0);
    CHECK(added["name"].str() == "extra");
    CHECK(added["index"].as_int() == 1);
    CHECK(added["children"].size() == 1);

    Node3D host;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(editable, &host, loaded);
    expectClean(loaded, "read editable instance");
    CHECK(copy != nullptr);
    if (!copy) return;
    Node* loadedInstance = copy->getChild(0);
    CHECK(loadedInstance && loadedInstance->getChildCount() == 3);
    if (!loadedInstance || loadedInstance->getChildCount() != 3) return;
    CHECK(loadedInstance->getChild(0)->getName() == "pole");
    CHECK(loadedInstance->getChild(1)->getName() == "extra");
    CHECK(loadedInstance->getChild(2)->getName() == "base");
    CHECK(loadedInstance->getChild(0)->getOwner() == loadedInstance);
    CHECK(loadedInstance->getChild(1)->getOwner() == nullptr);
    CHECK(loadedInstance->getChild(1)->getChild(0)->getName() == "inner");
    CHECK(copy->isEditableInstance(loadedInstance));
    Node* loadedGadget = loadedInstance->getNode("pole/gadget");
    CHECK(loadedGadget != nullptr && loadedGadget->getOwner() == nullptr);
    CHECK(sameVec3(loadedInstance->getNode("base")->castTo<Node3D>()->getPosition(), 0.0f, 4.0f,
            0.0f));
    CHECK(sameVec3(loadedInstance->getNode("extra")->castTo<Node3D>()->getPosition(), 0.0f, 0.0f,
            5.0f));

    ct::String direct;
    ct::String reloaded;
    CHECK(SceneSerializer::save(&world, direct));
    CHECK(SceneSerializer::save(copy, reloaded));
    CHECK(direct == reloaded);
}


Node3D* spatial(Node* node)
{
    return node ? node->castTo<Node3D>() : nullptr;
}

void testNested()
{
    const ct::String mast = filePath("nested_mast.prefab.json");
    const ct::String rack = filePath("rack.prefab.json");
    Prefab::clearCache();
    makeMast(mast, 2.5f);

    {
        Node3D scratch;
        Node3D* rackRoot = scratch.createChild<Node3D>();
        rackRoot->setName("rack");
        place(mast, rackRoot, "left", -1.0f);
        place(mast, rackRoot, "right", 1.0f);
        Node3D* beam = rackRoot->createChild<Node3D>();
        beam->setName("beam");
        SceneResult saved;
        CHECK(Prefab::saveFromNode(rackRoot, rack, saved));
        expectClean(saved, "save rack");
    }

    const ct::String rackText = FileSystem::getSingleton().readText(rack);
    CHECK(!rackText.empty());
    CHECK(strstr(rackText.c_str(), "\"pole\"") == nullptr);
    CHECK(strstr(rackText.c_str(), "\"foot\"") == nullptr);
    CHECK(strstr(rackText.c_str(), "nested_mast.prefab.json") != nullptr);

    ct::Vector<ct::String> chain;
    CHECK(Prefab::dependsOn(rack, mast, &chain));
    CHECK(chain.size() == 2 && chain[0] == rack && chain[1] == mast);
    CHECK(!Prefab::dependsOn(mast, rack));

    Node3D world;
    world.setName("world");
    Node3D* r1 = place(rack, &world, "r1", 10.0f);
    Node3D* r2 = place(rack, &world, "r2", 20.0f);
    if (!r1 || !r2) return;
    CHECK(r1->isInstance() && r1->getSceneFilePath() == rack);
    CHECK(r1->getChildCount() == 3);
    Node* left = r1->findChild("left");
    CHECK(left && left->isInstance() && left->getSceneFilePath() == mast);
    CHECK(left && left->getOwner() == r1);
    CHECK(left && left->getNode("base/foot") && left->getNode("base/foot")->getOwner() == left);
    CHECK(r1->findChild("beam") && r1->findChild("beam")->getOwner() == r1);
    CHECK(r1->getOwner() == nullptr);

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(&world, text, &saved));
    expectClean(saved, "scene with rack");
    CHECK(strstr(text.c_str(), "left") == nullptr);
    CHECK(strstr(text.c_str(), "pole") == nullptr);
    CHECK(strstr(text.c_str(), "beam") == nullptr);

    world.setEditableInstance(r1, true);
    lampOf(left)->setEnergy(9.0f);
    Node3D* hat = left->createChild<Node3D>();
    hat->setName("hat");
    ct::String editable;
    CHECK(SceneSerializer::save(&world, editable, &saved));
    expectClean(saved, "scene with edited rack");
    const ct::Json document = readJson(editable);
    const ct::Json& entry = *childEntry(document["root"], 0);
    const ct::Json* overrides = entry.find("overrides");
    CHECK(overrides && overrides->size() == 1);
    if (overrides && overrides->size() == 1)
    {
        const ct::Json& item = overrides->at(0);
        CHECK(item["path"].str() == "left");
        CHECK(item["properties"].find("light_energy") != nullptr);
        CHECK(item["children"].size() == 1 && item["children"][0]["name"].str() == "hat");
    }
    CHECK(childEntry(*childEntry(document["root"], 1), 0) == nullptr);

    Node3D host;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(editable, &host, loaded);
    expectClean(loaded, "read edited rack");
    CHECK(copy != nullptr && copy->getChildCount() == 2);
    if (!copy || copy->getChildCount() != 2) return;
    Node* loadedLeft = copy->getChild(0)->findChild("left");
    CHECK(loadedLeft && lampOf(loadedLeft)->getEnergy() == 9.0f);
    CHECK(loadedLeft && loadedLeft->findChild("hat") &&
            loadedLeft->findChild("hat")->getOwner() == nullptr);
    CHECK(lampOf(copy->getChild(1)->findChild("left"))->getEnergy() == 2.5f);
    ct::String again;
    CHECK(SceneSerializer::save(copy, again));
    CHECK(again == editable);

    world.setEditableInstance(r1, false);
    SceneResult applied;
    lampOf(r2->findChild("left"))->setEnergy(6.0f);
    Node* replaced = Prefab::apply(r2->findChild("left"), applied);
    CHECK(applied.errorCount() == 0);
    CHECK(applied.warningCount() == 1);
    CHECK(hasDiagnostic(applied, SceneDiagnosticSeverity::Warning, "r1",
            "node 'hat' was added inside 'left'"));
    CHECK(replaced != nullptr);
    CHECK(replaced && replaced->getParent() && replaced->getParent()->getName() == "r2");
    CHECK(replaced && lampOf(replaced)->getEnergy() == 6.0f);
    CHECK(replaced && sameVec3(spatial(replaced)->getPosition(), -1.0f, 0.0f, 0.0f));
    CHECK(lampOf(world.findChild("r1")->findChild("left"))->getEnergy() == 6.0f);
    CHECK(world.getChildCount() == 2);
    CHECK(world.findChild("r1")->getChildCount() == 3);
    CHECK(world.findChild("r1")->isInstance());
}

void testRootInstance()
{
    const ct::String mast = filePath("inherit_mast.prefab.json");
    const ct::String child = filePath("inherit_child.prefab.json");
    Prefab::clearCache();
    makeMast(mast, 2.5f);

    Node3D host;
    Node3D* j = place(mast, &host, "j", 0.0f);
    if (!j) return;
    lampOf(j)->setEnergy(3.5f);
    SceneResult saved;
    CHECK(Prefab::saveFromNode(j, child, saved));
    expectClean(saved, "save inherited prefab");
    const ct::String text = FileSystem::getSingleton().readText(child);
    CHECK(strstr(text.c_str(), "\"instance\"") != nullptr);
    CHECK(strstr(text.c_str(), "pole") == nullptr);
    CHECK(Prefab::dependsOn(child, mast));

    Node3D world;
    Node3D* k = place(child, &world, "k", 4.0f);
    if (!k) return;
    CHECK(k->isInstance() && k->getSceneFilePath() == child);
    CHECK(lampOf(k)->getEnergy() == 3.5f);
    CHECK(k->getChildCount() == 2 && k->findChild("base")->getOwner() == k);

    ct::String scene;
    SceneResult sceneResult;
    CHECK(SceneSerializer::save(&world, scene, &sceneResult));
    expectClean(sceneResult, "scene with inherited prefab");
    const ct::Json document = readJson(scene);
    const ct::Json& entry = *childEntry(document["root"], 0);
    CHECK(entry["instance"].str() == child);
    CHECK(hasKeys(entry, "instance", "name", "properties"));
    CHECK(entry["properties"].size() == 1);
}

void testApply()
{
    const ct::String mast = filePath("apply.prefab.json");
    Prefab::clearCache();
    makeMast(mast, 2.5f);

    SceneTree* tree = SceneTree::singleton();
    Node3D* world = tree->getRoot()->createChild<Node3D>();
    world->setName("applyWorld");
    Node3D* holder = world->createChild<Node3D>();
    holder->setName("holder");
    Node3D* before = holder->createChild<Node3D>();
    before->setName("before");
    Node3D* m1 = place(mast, holder, "m1", 1.0f);
    Node3D* m2 = place(mast, holder, "m2", 2.0f);
    Node3D* after = holder->createChild<Node3D>();
    after->setName("after");
    Node3D* m3 = place(mast, holder, "m3", 3.0f);
    (void) before;
    (void) after;
    if (!m1 || !m2 || !m3) return;
    CHECK(m1->isInsideTree() && m1->getTree() == tree);

    lampOf(m1)->setEnergy(5.0f);
    poleOf(m2)->setPosition(Math::Vec3(0.0f, -2.0f, 0.0f));
    lampOf(m2)->setEnergy(7.0f);
    Node3D* spare = m2->createChild<Node3D>();
    spare->setName("spare");
    holder->setEditableInstance(m3, true);
    m3->findChild("base")->findChild("foot")->castTo<Node3D>()->setPosition(
            Math::Vec3(5.0f, 0.0f, 0.0f));
    Node3D* mine = m3->createChild<Node3D>();
    mine->setName("mine");
    m3->moveChild(mine, 0);

    const size_t reads = Prefab::fileReadCount();
    SceneResult result;
    Node* applied = Prefab::apply(m2, result);
    expectClean(result, "apply");
    CHECK(Prefab::fileReadCount() == reads);
    CHECK(applied != nullptr);
    CHECK(holder->getChildCount() == 5);
    const char* order[] = { "before", "m1", "m2", "after", "m3" };
    for (size_t i = 0; i < 5 && i < holder->getChildCount(); ++i)
        CHECK(holder->getChild(i)->getName() == order[i]);
    CHECK(applied == holder->findChild("m2"));

    Node* n1 = holder->findChild("m1");
    Node* n2 = holder->findChild("m2");
    Node* n3 = holder->findChild("m3");
    CHECK(n1 && n2 && n3);
    if (!n1 || !n2 || !n3) return;
    for (Node* instance: { n1, n2, n3 })
    {
        CHECK(instance->isInstance() && instance->getSceneFilePath() == mast);
        CHECK(instance->isInsideTree() && instance->getTree() == tree);
        CHECK(poleOf(instance) && sameVec3(poleOf(instance)->getPosition(), 0.0f, -2.0f, 0.0f));
        CHECK(instance->findChild("spare") != nullptr);
        CHECK(instance->findChild("spare")->getOwner() == instance);
        CHECK(poleOf(instance)->getOwner() == instance);
        CHECK(instance->getOwner() == nullptr);
    }
    CHECK(lampOf(n1)->getEnergy() == 5.0f);
    CHECK(lampOf(n2)->getEnergy() == 7.0f);
    CHECK(lampOf(n3)->getEnergy() == 7.0f);
    CHECK(sameVec3(spatial(n1)->getPosition(), 1.0f, 0.0f, 0.0f));
    CHECK(sameVec3(spatial(n2)->getPosition(), 2.0f, 0.0f, 0.0f));
    CHECK(sameVec3(spatial(n3)->getPosition(), 3.0f, 0.0f, 0.0f));

    CHECK(holder->isEditableInstance(n3));
    CHECK(!holder->isEditableInstance(n1));
    CHECK(sameVec3(spatial(n3->getNode("base/foot"))->getPosition(), 5.0f, 0.0f, 0.0f));
    CHECK(sameVec3(spatial(n1->getNode("base/foot"))->getPosition(), 1.0f, 0.0f, 0.0f));
    CHECK(n3->getChildCount() == 4 && n3->getChild(0)->getName() == "mine");
    CHECK(n3->getChild(0)->getOwner() == nullptr);

    const ct::String onDisk = FileSystem::getSingleton().readText(mast);
    const ct::Json file = readJson(onDisk);
    const ct::Json& fileRoot = file["root"];
    CHECK(fileRoot["name"].str() == "lamp");
    CHECK(fileRoot["properties"]["light_energy"].as_double() == 7.0);
    CHECK(fileRoot["properties"].find("position") == nullptr);
    CHECK(fileRoot["children"].size() == 3);
    CHECK(Prefab::fileReadCount() == reads);

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(world, text, &saved));
    expectClean(saved, "scene after apply");
    const ct::Json scene = readJson(text);
    const ct::Json& entries = *childEntry(scene["root"], 0);
    const ct::Json* e1 = childEntry(entries, 1);
    const ct::Json* e2 = childEntry(entries, 2);
    const ct::Json* e3 = childEntry(entries, 4);
    CHECK(e1 && (*e1)["name"].str() == "m1" && (*e1)["properties"].size() == 2);
    CHECK(e1 && (*e1)["properties"].find("light_energy") != nullptr);
    CHECK(e2 && (*e2)["name"].str() == "m2" && (*e2)["properties"].size() == 1);
    CHECK(e3 && (*e3)["name"].str() == "m3" && (*e3)["properties"].size() == 1);
    CHECK(e3 && e3->find("editable") != nullptr);
    CHECK(e3 && e3->find("overrides") != nullptr && (*e3)["overrides"].size() == 1);
    CHECK(e3 && e3->find("children") != nullptr && (*e3)["children"].size() == 1);

    lampOf(n1)->setEnergy(9.0f);
    Node3D* scrap = n1->createChild<Node3D>();
    scrap->setName("scrap");
    n1->castTo<Node3D>()->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    SceneResult reverted;
    Node* back = Prefab::revert(n1, reverted);
    expectClean(reverted, "revert");
    CHECK(back != nullptr);
    CHECK(back && back == holder->getChild(1) && back->getName() == "m1");
    CHECK(back && lampOf(back)->getEnergy() == 7.0f);
    CHECK(back && back->findChild("scrap") == nullptr);
    CHECK(back && sameVec3(spatial(back)->getPosition(), 1.0f, 2.0f, 3.0f));
    CHECK(back && back->isInsideTree());

    SceneResult resetAll;
    Node* hard = Prefab::revert(back, resetAll, false);
    expectClean(resetAll, "revert with transform");
    CHECK(hard && sameVec3(spatial(hard)->getPosition(), 0.0f, 0.0f, 0.0f));
    CHECK(hard && hard->getName() == "m1" && hard == holder->getChild(1));

    spatial(hard)->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    lampOf(hard)->setEnergy(4.0f);

    makeMast(mast, 11.0f, -1.0f, world);
    CHECK(Prefab::fileReadCount() == reads);
    Node* r1 = holder->findChild("m1");
    Node* r2 = holder->findChild("m2");
    Node* r3 = holder->findChild("m3");
    CHECK(r1 && r2 && r3 && holder->getChildCount() == 5);
    if (!r1 || !r2 || !r3) return;
    CHECK(lampOf(r1)->getEnergy() == 4.0f);
    CHECK(lampOf(r2)->getEnergy() == 11.0f);
    CHECK(lampOf(r3)->getEnergy() == 11.0f);
    CHECK(sameVec3(poleOf(r2)->getPosition(), 0.0f, -1.0f, 0.0f));
    CHECK(r2->findChild("spare") == nullptr);
    CHECK(sameVec3(spatial(r1)->getPosition(), 1.0f, 0.0f, 0.0f));
    CHECK(sameVec3(spatial(r2)->getPosition(), 2.0f, 0.0f, 0.0f));
    CHECK(sameVec3(spatial(r3)->getPosition(), 3.0f, 0.0f, 0.0f));
    CHECK(sameVec3(spatial(r3->getNode("base/foot"))->getPosition(), 5.0f, 0.0f, 0.0f));
    CHECK(r3->getChild(0)->getName() == "mine");

    const ct::String otherPath = filePath("apply_other.prefab.json");
    makeMast(otherPath, 13.0f, -3.0f);
    const ct::String replacement = FileSystem::getSingleton().readText(otherPath);
    CHECK(FileSystem::writeText(mast, replacement));
    CHECK(Prefab::isCached(mast));
    SceneResult fromDisk;
    CHECK(Prefab::reloadInstances(mast, world, fromDisk) == 3);
    expectClean(fromDisk, "reload from disk");
    CHECK(Prefab::fileReadCount() == reads + 1);
    CHECK(lampOf(holder->findChild("m2"))->getEnergy() == 13.0f);
    CHECK(lampOf(holder->findChild("m1"))->getEnergy() == 4.0f);
    CHECK(sameVec3(poleOf(holder->findChild("m3"))->getPosition(), 0.0f, -3.0f, 0.0f));

    ct::Vector<Node*> found;
    Prefab::findInstances(world, mast, found);
    CHECK(found.size() == 3);

    Node scratch;
    world->reparent(&scratch);
}

void writePrefabFile(const ct::String& path, const char* name, const ct::String& instance)
{
    ct::Json root = ct::Json::object();
    root["class"] = ct::Json("Node3D");
    root["name"] = ct::Json(name);
    if (!instance.empty())
    {
        ct::Json entry = ct::Json::object();
        entry["instance"] = ct::Json(instance);
        ct::Json children = ct::Json::array();
        children.push_back(entry);
        root["children"] = children;
    }
    ct::Json document = ct::Json::object();
    document["format"] = ct::Json("prisma3d-scene");
    document["version"] = ct::Json(1);
    document["root"] = root;
    CHECK(FileSystem::writeText(path, document.dump(2)));
}

void testCycles()
{
    Prefab::clearCache();
    const ct::String a = filePath("cycle_a.prefab.json");
    const ct::String b = filePath("cycle_b.prefab.json");
    const ct::String self = filePath("cycle_self.prefab.json");
    writePrefabFile(a, "a", b);
    writePrefabFile(b, "b", a);
    writePrefabFile(self, "self", self);

    Node3D world;
    SceneResult result;
    Node* node = Prefab::instantiate(a, &world, result);
    CHECK(node != nullptr);
    CHECK(result.errorCount() == 1);
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, ".instance",
            "the prefab contains itself"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "cycle_b.prefab.json",
            "cycle_a.prefab.json"));
    CHECK(node && node->getName() == "a" && node->getChildCount() == 1);
    CHECK(node && node->getChild(0)->getName() == "b" && node->getChild(0)->getChildCount() == 1);
    MissingNode* closing = node && node->getChild(0)->getChildCount() == 1
                                   ? node->getChild(0)->getChild(0)->castTo<MissingNode>()
                                   : nullptr;
    CHECK(closing != nullptr && closing->getOriginalScene() == a);

    SceneResult selfResult;
    Node* selfNode = Prefab::instantiate(self, &world, selfResult);
    CHECK(selfNode != nullptr && selfNode->getChildCount() == 1);
    CHECK(selfNode && selfNode->getChild(0)->castTo<MissingNode>() != nullptr);
    CHECK(selfResult.errorCount() == 1);
    CHECK(hasDiagnostic(selfResult, SceneDiagnosticSeverity::Error, nullptr,
            "cycle_self.prefab.json -> "));

    SceneResult sceneResult;
    Node3D host;
    Node* scene = SceneSerializer::loadFile(a, &host, sceneResult);
    CHECK(scene != nullptr);
    CHECK(hasDiagnostic(sceneResult, SceneDiagnosticSeverity::Error, nullptr,
            "the prefab contains itself: "));
    CHECK(strstr(sceneResult.describe().c_str(), "cycle_a.prefab.json -> ") != nullptr);
    CHECK(Prefab::dependsOn(a, b));
    CHECK(Prefab::dependsOn(a, a));

    const ct::String leaf = filePath("cycle_leaf.prefab.json");
    const ct::String holder = filePath("cycle_holder.prefab.json");
    makeMast(leaf, 2.5f);
    {
        Node3D scratch;
        Node3D* root = scratch.createChild<Node3D>();
        root->setName("holder");
        place(leaf, root, "leaf", 0.0f);
        SceneResult saved;
        CHECK(Prefab::saveFromNode(root, holder, saved));
        expectClean(saved, "save holder");
    }
    const ct::String leafBefore = FileSystem::getSingleton().readText(leaf);

    Node3D scene2;
    Node3D* inside = place(holder, &scene2, "inside", 0.0f);
    SceneResult refused;
    CHECK(inside != nullptr);
    CHECK(!Prefab::saveFromNode(&scene2, leaf, refused));
    CHECK(refused.errorCount() == 1);
    CHECK(hasDiagnostic(refused, SceneDiagnosticSeverity::Error, "inside",
            "the prefab would contain itself"));
    CHECK(hasDiagnostic(refused, SceneDiagnosticSeverity::Error, "inside",
                  "cycle_leaf.prefab.json -> ") &&
            strstr(refused.describe().c_str(), "cycle_holder.prefab.json -> ") != nullptr);
    CHECK(FileSystem::getSingleton().readText(leaf) == leafBefore);

    Node3D* leafNode = place(leaf, &scene2, "leafNode", 0.0f);
    SceneResult selfRefused;
    CHECK(leafNode != nullptr && !Prefab::saveFromNode(leafNode, leaf, selfRefused));
    CHECK(selfRefused.errorCount() == 1);
    CHECK(FileSystem::getSingleton().readText(leaf) == leafBefore);

    Node3D* plain = scene2.createChild<Node3D>();
    plain->setName("plain");
    Node3D* host2 = place(leaf, &scene2, "host2", 0.0f);
    place(holder, host2, "bad", 0.0f);
    SceneResult applyRefused;
    CHECK(Prefab::apply(host2, applyRefused) == nullptr);
    CHECK(hasDiagnostic(applyRefused, SceneDiagnosticSeverity::Error, nullptr,
            "the prefab would contain itself"));
    CHECK(host2->getParent() == &scene2);
    CHECK(FileSystem::getSingleton().readText(leaf) == leafBefore);
    CHECK(host2->findChild("bad") != nullptr);

    SceneResult ok;
    Node3D* leafAgain = spatial(Prefab::instantiate(leaf, &scene2, ok));
    expectClean(ok, "leaf after refused saves");
    CHECK(leafAgain && lampOf(leafAgain)->getEnergy() == 2.5f);
}

void testMissingAndInvalid()
{
    Prefab::clearCache();
    const ct::String missing = filePath("not_there.prefab.json");
    const ct::String broken = filePath("broken.prefab.json");
    const ct::String wrong = filePath("wrong.prefab.json");
    const ct::String mast = filePath("missing_mast.prefab.json");
    makeMast(mast, 2.5f);
    CHECK(FileSystem::writeText(broken, "this is not json"));
    CHECK(FileSystem::writeText(wrong, "{\"format\": \"other\", \"version\": 1, \"root\": {}}"));

    ct::Json root = ct::Json::object();
    root["class"] = ct::Json("Node3D");
    root["name"] = ct::Json("scene");
    ct::Json children = ct::Json::array();
    auto plainChild = [&children](const char* name)
    {
        ct::Json entry = ct::Json::object();
        entry["class"] = ct::Json("Node3D");
        entry["name"] = ct::Json(name);
        children.push_back(entry);
    };
    auto instanceChild = [&children](const ct::String& path, const char* name)
    {
        ct::Json entry = ct::Json::object();
        entry["instance"] = ct::Json(path);
        entry["name"] = ct::Json(name);
        ct::Json properties = ct::Json::object();
        ct::Json position = ct::Json::array();
        position.push_back(ct::Json(1.0));
        position.push_back(ct::Json(2.0));
        position.push_back(ct::Json(3.0));
        properties["position"] = position;
        entry["properties"] = properties;
        children.push_back(entry);
    };
    plainChild("before");
    instanceChild(missing, "ghost");
    plainChild("middle");
    instanceChild(broken, "broken");
    instanceChild(wrong, "wrong");
    instanceChild(mast, "good");
    plainChild("after");
    root["children"] = children;
    ct::Json document = ct::Json::object();
    document["format"] = ct::Json("prisma3d-scene");
    document["version"] = ct::Json(1);
    document["root"] = root;

    Node3D host;
    SceneResult result;
    Node* scene = SceneSerializer::load(document.dump(2), &host, result);
    CHECK(scene != nullptr);
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning, "root.children[1].instance",
            "could not read the prefab"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.children[3].instance<",
            "not valid JSON"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.children[4].instance<",
            "prisma3d-scene"));
    CHECK(result.warningCount() == 1);
    CHECK(result.errorCount() == 2);
    if (!scene) return;
    CHECK(scene->getChildCount() == 7);
    const char* names[] = { "before", "ghost", "middle", "broken", "wrong", "good", "after" };
    for (size_t i = 0; i < 7 && i < scene->getChildCount(); ++i)
        CHECK(scene->getChild(i)->getName() == names[i]);
    CHECK(scene->findChild("good") && scene->findChild("good")->isInstance());
    CHECK(scene->findChild("good") &&
            sameVec3(spatial(scene->findChild("good"))->getPosition(), 1.0f, 2.0f, 3.0f));

    const size_t keptIndices[] = { 1, 3, 4 };
    const ct::String keptScenes[] = { FileSystem::simplifyPath(missing),
        FileSystem::simplifyPath(broken), FileSystem::simplifyPath(wrong) };
    for (size_t i = 0; i < 3; ++i)
    {
        MissingNode* kept = scene->getChild(keptIndices[i])->castTo<MissingNode>();
        CHECK(kept != nullptr);
        if (!kept) continue;
        CHECK(kept->getOriginalScene() == keptScenes[i]);
        CHECK(kept->getOriginalClass().empty());
        CHECK(!kept->isInstance());
        CHECK(kept->getEntry().dump(2) == children.at(keptIndices[i]).dump(2));
    }

    ct::String resaved;
    SceneResult resavedResult;
    CHECK(SceneSerializer::save(scene, resaved, &resavedResult));
    expectClean(resavedResult, "resave with missing prefabs");
    const ct::Json resavedDocument = readJson(resaved);
    const ct::Json& resavedRoot = resavedDocument["root"];
    for (size_t i = 0; i < 3; ++i)
    {
        const ct::Json* entry = childEntry(resavedRoot, keptIndices[i]);
        CHECK(entry != nullptr);
        if (entry) CHECK(entry->dump(2) == children.at(keptIndices[i]).dump(2));
    }
    const ct::Json* resavedGood = childEntry(resavedRoot, 5);
    CHECK(resavedGood && resavedGood->find("instance") && resavedGood->find("class") == nullptr);

    SceneResult direct;
    CHECK(Prefab::instantiate(missing, &host, direct) == nullptr);
    CHECK(hasDiagnostic(direct, SceneDiagnosticSeverity::Error, "instance",
            "could not read the prefab"));
    SceneResult empty;
    CHECK(Prefab::instantiate(ct::String(), &host, empty) == nullptr);
    CHECK(!empty.success());
    SceneResult orphan;
    CHECK(Prefab::instantiate(mast, nullptr, orphan) == nullptr);
    CHECK(hasDiagnostic(orphan, SceneDiagnosticSeverity::Error, nullptr, "no parent node"));
    SceneResult nothing;
    CHECK(!Prefab::saveFromNode(nullptr, mast, nothing));
    CHECK(Prefab::apply(nullptr, nothing) == nullptr);
    CHECK(Prefab::revert(&host, nothing) == nullptr);

    CHECK(Prefab::dependsOn(missing, mast) == false);

    makeMast(missing, 2.5f);
    Node3D secondHost;
    SceneResult restored;
    Node* back = SceneSerializer::load(resaved, &secondHost, restored);
    CHECK(back != nullptr && back->getChildCount() == 7);
    CHECK(restored.warningCount() == 0);
    Node* ghost = back ? back->findChild("ghost") : nullptr;
    CHECK(ghost && ghost->isInstance() && ghost->castTo<MissingNode>() == nullptr);
    CHECK(ghost && sameVec3(spatial(ghost)->getPosition(), 1.0f, 2.0f, 3.0f));
    CHECK(ghost && lampOf(ghost) && lampOf(ghost)->getEnergy() == 2.5f);
    CHECK(back && back->getChild(3)->castTo<MissingNode>() != nullptr);
}

ct::String documentOf(const ct::Json& root)
{
    ct::Json document = ct::Json::object();
    document["format"] = ct::Json("prisma3d-scene");
    document["version"] = ct::Json(1);
    document["root"] = root;
    return document.dump(2);
}

void testMissingInsideInstances()
{
    Prefab::clearCache();
    const ct::String mast = filePath("keep_mast.prefab.json");
    const ct::String nowhere = filePath("keep_nowhere.prefab.json");
    const ct::String widget = filePath("keep_widget.prefab.json");
    const ct::String holey = filePath("keep_holey.prefab.json");
    makeMast(mast, 2.5f);

    ct::Json widgetRoot = ct::Json::object();
    widgetRoot["class"] = ct::Json("Widget");
    widgetRoot["name"] = ct::Json("w");
    CHECK(FileSystem::writeText(widget, documentOf(widgetRoot)));

    ct::Json holeyRoot = ct::Json::object();
    holeyRoot["class"] = ct::Json("Node3D");
    holeyRoot["name"] = ct::Json("holey");
    ct::Json holeyChildren = ct::Json::array();
    ct::Json strange = ct::Json::object();
    strange["class"] = ct::Json("Strange");
    strange["name"] = ct::Json("odd");
    holeyChildren.push_back(strange);
    holeyRoot["children"] = holeyChildren;
    CHECK(FileSystem::writeText(holey, documentOf(holeyRoot)));

    ct::Json gizmo = ct::Json::object();
    gizmo["class"] = ct::Json("Gadget");
    gizmo["name"] = ct::Json("gizmo");
    ct::Json gizmoProperties = ct::Json::object();
    gizmoProperties["charge"] = ct::Json(0.75);
    gizmo["properties"] = gizmoProperties;
    gizmo["index"] = ct::Json(0);
    ct::Json ghost = ct::Json::object();
    ghost["instance"] = ct::Json(nowhere);
    ghost["name"] = ct::Json("ghost2");
    ct::Json ghostProperties = ct::Json::object();
    ct::Json ghostPosition = ct::Json::array();
    ghostPosition.push_back(ct::Json(5.0));
    ghostPosition.push_back(ct::Json(0.0));
    ghostPosition.push_back(ct::Json(0.0));
    ghostProperties["position"] = ghostPosition;
    ghost["properties"] = ghostProperties;
    ghost["index"] = ct::Json(1);
    ct::Json additions = ct::Json::array();
    additions.push_back(gizmo);
    additions.push_back(ghost);

    ct::Json holder = ct::Json::object();
    holder["instance"] = ct::Json(mast);
    holder["name"] = ct::Json("m");
    holder["children"] = additions;

    ct::Json brokenInstance = ct::Json::object();
    brokenInstance["instance"] = ct::Json(widget);
    brokenInstance["name"] = ct::Json("w1");
    ct::Json brokenProperties = ct::Json::object();
    ct::Json brokenPosition = ct::Json::array();
    brokenPosition.push_back(ct::Json(1.0));
    brokenPosition.push_back(ct::Json(2.0));
    brokenPosition.push_back(ct::Json(3.0));
    brokenProperties["position"] = brokenPosition;
    brokenInstance["properties"] = brokenProperties;

    ct::Json holeyInstance = ct::Json::object();
    holeyInstance["instance"] = ct::Json(holey);
    holeyInstance["name"] = ct::Json("h1");

    ct::Json root = ct::Json::object();
    root["class"] = ct::Json("Node3D");
    root["name"] = ct::Json("scene");
    ct::Json list = ct::Json::array();
    list.push_back(holder);
    list.push_back(brokenInstance);
    list.push_back(holeyInstance);
    root["children"] = list;

    Node3D host;
    SceneResult loaded;
    Node* scene = SceneSerializer::load(documentOf(root), &host, loaded);
    CHECK(scene != nullptr && scene->getChildCount() == 3);
    CHECK(hasDiagnostic(loaded, SceneDiagnosticSeverity::Warning, "root.children[0].children[1]",
            "could not read the prefab"));
    CHECK(hasDiagnostic(loaded, SceneDiagnosticSeverity::Error, "children[1]", "Widget"));
    CHECK(hasDiagnostic(loaded, SceneDiagnosticSeverity::Error, "children[2]", "Strange"));
    if (!scene || scene->getChildCount() != 3) return;

    Node* first = scene->getChild(0);
    CHECK(first->isInstance() && first->getChildCount() == 4);
    if (first->getChildCount() == 4)
    {
        MissingNode* gizmoNode = first->getChild(0)->castTo<MissingNode>();
        CHECK(gizmoNode && gizmoNode->getName() == "gizmo" &&
                gizmoNode->getOriginalClass() == "Gadget");
        MissingNode* ghostNode = first->getChild(1)->castTo<MissingNode>();
        CHECK(ghostNode && ghostNode->getName() == "ghost2" &&
                ghostNode->getOriginalScene() == FileSystem::simplifyPath(nowhere));
        CHECK(first->getChild(2)->getName() == "pole" && first->getChild(3)->getName() == "base");
        CHECK(first->getChild(0)->getOwner() == nullptr);
    }

    MissingNode* widgetNode = scene->getChild(1)->castTo<MissingNode>();
    CHECK(widgetNode && widgetNode->getName() == "w1" &&
            widgetNode->getOriginalScene() == FileSystem::simplifyPath(widget));
    CHECK(widgetNode && widgetNode->getChildCount() == 0 && !widgetNode->isInstance());

    Node* holeyNode = scene->getChild(2);
    CHECK(holeyNode->isInstance() && holeyNode->getChildCount() == 1);
    CHECK(holeyNode->getChildCount() == 1 && holeyNode->getChild(0)->castTo<MissingNode>());

    ct::String saved;
    SceneResult savedResult;
    CHECK(SceneSerializer::save(scene, saved, &savedResult));
    expectClean(savedResult, "save missing inside instances");
    const ct::Json savedDocument = readJson(saved);
    const ct::Json& savedRoot = savedDocument["root"];
    CHECK(savedRoot["children"].size() == 3);
    CHECK(savedRoot["children"][0].dump(2) == holder.dump(2));
    CHECK(savedRoot["children"][1].dump(2) == brokenInstance.dump(2));
    CHECK(savedRoot["children"][2].dump(2) == holeyInstance.dump(2));

    Node3D second;
    SceneResult direct;
    CHECK(Prefab::instantiate(widget, &second, direct) == nullptr);
    CHECK(second.getChildCount() == 0);
    CHECK(direct.errorCount() == 1);
    SceneResult rebuilt;
    CHECK(Prefab::revert(scene->getChild(0), rebuilt) != nullptr);
    CHECK(scene->getChild(0)->isInstance() && scene->getChild(0)->getChildCount() == 2);
}

void testEntryErrors()
{
    Prefab::clearCache();
    const ct::String mast = filePath("entry_mast.prefab.json");
    makeMast(mast, 2.5f);

    ct::Json entry = ct::Json::object();
    entry["instance"] = ct::Json(mast);
    entry["name"] = ct::Json("probe");
    entry["class"] = ct::Json("Camera3D");
    entry["data"] = ct::Json::object();
    entry["surprise"] = ct::Json(1);
    entry["editable"] = ct::Json("yes");
    ct::Json overrides = ct::Json::array();
    auto addOverride = [&overrides](const char* path)
    {
        ct::Json item = ct::Json::object();
        item["path"] = ct::Json(path);
        overrides.push_back(item);
    };
    addOverride("gone/leg");
    addOverride("../escape");
    addOverride("/base");
    addOverride("base/../pole");
    ct::Json valid = ct::Json::object();
    valid["path"] = ct::Json("base/foot");
    ct::Json footProperties = ct::Json::object();
    ct::Json footPosition = ct::Json::array();
    footPosition.push_back(ct::Json(0.0));
    footPosition.push_back(ct::Json(0.0));
    footPosition.push_back(ct::Json(9.0));
    footProperties["position"] = footPosition;
    valid["properties"] = footProperties;
    valid["junk"] = ct::Json(1);
    overrides.push_back(valid);
    entry["overrides"] = overrides;
    ct::Json children = ct::Json::array();
    ct::Json added = ct::Json::object();
    added["class"] = ct::Json("Node3D");
    added["name"] = ct::Json("added");
    added["index"] = ct::Json(-4);
    children.push_back(added);
    entry["children"] = children;

    ct::Json root = ct::Json::object();
    root["class"] = ct::Json("Node3D");
    root["name"] = ct::Json("scene");
    ct::Json list = ct::Json::array();
    list.push_back(entry);
    ct::Json noPath = ct::Json::object();
    noPath["name"] = ct::Json("x");
    noPath["instance"] = ct::Json(5);
    list.push_back(noPath);
    root["children"] = list;
    ct::Json document = ct::Json::object();
    document["format"] = ct::Json("prisma3d-scene");
    document["version"] = ct::Json(1);
    document["root"] = root;

    Node3D host;
    SceneResult result;
    Node* scene = SceneSerializer::load(document.dump(2), &host, result);
    CHECK(scene != nullptr && scene->getChildCount() == 2);
    CHECK(scene && scene->getChild(1)->castTo<MissingNode>() != nullptr);
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning, "root.children[0].class",
            "come from the prefab"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning, "root.children[0].data",
            "come from the prefab"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning, "root.children[0].surprise",
            "unknown field"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning, "root.children[0].editable",
            "not a boolean"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[0].overrides[0].path", "there is no node 'gone/leg'"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error,
            "root.children[0].overrides[1].path", "inside the instance"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error,
            "root.children[0].overrides[2].path", "inside the instance"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error,
            "root.children[0].overrides[3].path", "inside the instance"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[0].overrides[4].junk", "unknown field"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
            "root.children[0].children[0].index", "non-negative"));
    CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.children[1].instance",
            "missing or not a text"));
    if (!scene || scene->getChildCount() != 1) return;
    Node* probe = scene->getChild(0);
    CHECK(probe->getName() == "probe" && probe->isInstance());
    CHECK(probe->findChild("added") != nullptr);
    CHECK(sameVec3(spatial(probe->getNode("base/foot"))->getPosition(), 0.0f, 0.0f, 9.0f));
    CHECK(strcmp(probe->getClassName(), "OmniLight3D") == 0);
    CHECK(!scene->isEditableInstance(probe));
}

void testCache()
{
    const ct::String mast = filePath("cache.prefab.json");
    makeMast(mast, 2.5f);
    Prefab::clearCache();
    CHECK(!Prefab::isCached(mast));

    Node3D world;
    const size_t start = Prefab::fileReadCount();
    SceneResult first;
    Node* one = Prefab::instantiate(mast, &world, first);
    CHECK(one != nullptr);
    CHECK(Prefab::fileReadCount() == start + 1);
    CHECK(Prefab::isCached(mast));
    SceneResult second;
    Node* two = Prefab::instantiate(mast, &world, second);
    CHECK(two != nullptr && two != one);
    CHECK(Prefab::fileReadCount() == start + 1);

    SceneResult nested;
    Node* third = Prefab::instantiate(ct::String(mast) + "/../cache.prefab.json", &world, nested);
    CHECK(third != nullptr);
    CHECK(Prefab::fileReadCount() == start + 1);
    CHECK(third && third->getSceneFilePath() == mast);

    ct::String text;
    CHECK(SceneSerializer::save(&world, text));
    Node3D host;
    SceneResult loaded;
    CHECK(SceneSerializer::load(text, &host, loaded) != nullptr);
    expectClean(loaded, "cached scene");
    CHECK(Prefab::fileReadCount() == start + 1);

    Prefab::forget(mast);
    CHECK(!Prefab::isCached(mast));
    SceneResult fourth;
    CHECK(Prefab::instantiate(mast, &world, fourth) != nullptr);
    CHECK(Prefab::fileReadCount() == start + 2);
    Prefab::clearCache();
    SceneResult fifth;
    CHECK(Prefab::instantiate(mast, &world, fifth) != nullptr);
    CHECK(Prefab::fileReadCount() == start + 3);
}

void testOwners()
{
    const ct::String mast = filePath("owners.prefab.json");
    Prefab::clearCache();
    makeMast(mast, 2.5f);

    Node3D world;
    Node3D other;
    Node3D* m = place(mast, &world, "m", 0.0f);
    if (!m) return;
    Node* pole = m->findChild("pole");
    Node* base = m->findChild("base");
    Node* foot = m->getNode("base/foot");
    CHECK(pole && base && foot);
    if (!pole || !base || !foot) return;
    CHECK(pole->getOwner() == m && base->getOwner() == m && foot->getOwner() == m);

    other.setOwner(&world);
    CHECK(other.getOwner() == nullptr);
    pole->setOwner(&other);
    CHECK(pole->getOwner() == m);
    pole->setOwner(nullptr);
    CHECK(pole->getOwner() == nullptr);
    pole->setOwner(m);
    CHECK(pole->getOwner() == m);

    CHECK(pole->reparent(&other));
    CHECK(pole->getOwner() == nullptr);
    CHECK(base->reparent(&other));
    CHECK(base->getOwner() == nullptr && foot->getOwner() == nullptr);

    Node3D* second = place(mast, &world, "second", 0.0f);
    if (!second) return;
    Node* secondBase = second->findChild("base");
    Node* secondFoot = second->getNode("base/foot");
    CHECK(second->reparent(&other));
    CHECK(secondBase->getOwner() == second && secondFoot->getOwner() == second);
    CHECK(second->getOwner() == nullptr);
    CHECK(second->isInstance());

    Node3D third;
    Node3D* inner = place(mast, &third, "inner", 0.0f);
    Node3D* outer = third.createChild<Node3D>();
    outer->setName("outer");
    CHECK(inner && inner->reparent(outer));
    CHECK(inner->findChild("pole")->getOwner() == inner);
}

void testNodePaths()
{
    const ct::String smoke = filePath("smoke.prefab.json");
    Prefab::clearCache();
    {
        Node3D scratch;
        ParticleSystem3D* main = scratch.createChild<ParticleSystem3D>();
        main->setName("smoke");
        ParticleSystem3D* puff = main->createChild<ParticleSystem3D>();
        puff->setName("puff");
        main->setSubEmitter("puff");
        SceneResult saved;
        CHECK(Prefab::saveFromNode(main, smoke, saved));
        expectClean(saved, "save smoke");
    }

    Node3D world;
    ParticleSystem3D* other = world.createChild<ParticleSystem3D>();
    other->setName("other");
    SceneResult result;
    Node* first = Prefab::instantiate(smoke, &world, result);
    Node* second = Prefab::instantiate(smoke, &world, result);
    expectClean(result, "instantiate smoke");
    CHECK(first && second);
    if (!first || !second) return;
    first->setName("first");
    second->setName("second");
    CHECK(first->castTo<ParticleSystem3D>()->getSubEmitter() == "puff");
    second->castTo<ParticleSystem3D>()->setSubEmitter("../other");

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(&world, text, &saved));
    expectClean(saved, "scene with paths");
    const ct::Json document = readJson(text);
    const ct::Json* e1 = childEntry(document["root"], 1);
    const ct::Json* e2 = childEntry(document["root"], 2);
    CHECK(e1 && e1->find("properties") == nullptr);
    CHECK(e2 && e2->find("properties") && (*e2)["properties"].size() == 1);
    CHECK(e2 && (*e2)["properties"]["sub_emitter"].str() == "../other");

    Node3D host;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &host, loaded);
    expectClean(loaded, "read scene with paths");
    CHECK(copy && copy->getChildCount() == 3);
    if (!copy || copy->getChildCount() != 3) return;
    CHECK(copy->getChild(1)->castTo<ParticleSystem3D>()->getSubEmitter() == "puff");
    CHECK(copy->getChild(2)->castTo<ParticleSystem3D>()->getSubEmitter() == "../other");
}

void testInstanceBasics()
{
    Node plain;
    CHECK(!plain.isInstance());
    CHECK(plain.getSceneFilePath().empty());
    plain.setSceneFilePath("some/place.prefab.json");
    CHECK(plain.isInstance());
    CHECK(plain.getSceneFilePath() == "some/place.prefab.json");
    plain.setSceneFilePath(ct::String());
    CHECK(!plain.isInstance());
    CHECK(plain.getOwner() == nullptr);

    Node a;
    Node* b = a.createChild<Node>();
    Node* c = b->createChild<Node>();
    c->setOwner(&a);
    CHECK(c->getOwner() == &a);
    a.setEditableInstance(c, true);
    CHECK(a.isEditableInstance(c));
    CHECK(!a.isEditableInstance(&a));
    a.setEditableInstance(&a, true);
    CHECK(!a.isEditableInstance(&a));
    a.setEditableInstance(c, false);
    CHECK(!a.isEditableInstance(c));
}

} // namespace

int main()
{
    gDir = PRISMA3D_TEST_DIR;
    gDir += "/prefabs";
    removeTree(gDir);

    SceneTree tree;
    tree.initialize();
    registerSceneTypes();

    testInstanceBasics();
    testSaveAndInstantiate();
    testAdditionsAndEditable();
    testNested();
    testRootInstance();
    testApply();
    testCycles();
    testMissingAndInvalid();
    testMissingInsideInstances();
    testEntryErrors();
    testCache();
    testOwners();
    testNodePaths();

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("prefabs ok\n");
    return 0;
}
