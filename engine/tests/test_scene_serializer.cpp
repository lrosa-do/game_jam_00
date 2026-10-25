#include "Check.h"
#include "PropertyFixtures.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Animation.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Destructible3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/Hair3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/MissingNode.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/PlanarReflector3D.h"
#include "prisma3d/scene/PropertyValue.h"
#include "prisma3d/scene/Ragdoll3D.h"
#include "prisma3d/scene/RibbonTrail3D.h"
#include "prisma3d/scene/Road3D.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/SoftBody3D.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/json.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

bool near(double a, double b, double tolerance)
{
    const double scale = fmax(1.0, fmax(fabs(a), fabs(b)));
    return fabs(a - b) <= tolerance * scale;
}

bool valuesNear(const PropertyValue& a, const PropertyValue& b, double tolerance)
{
    if (a.type() != b.type()) return false;
    switch (a.type())
    {
        case PropertyType::Float:
            return near(a.asFloat(), b.asFloat(), tolerance);
        case PropertyType::Vec2:
            return near(a.asVec2().x, b.asVec2().x, tolerance) &&
                   near(a.asVec2().y, b.asVec2().y, tolerance);
        case PropertyType::Vec3:
            return near(a.asVec3().x, b.asVec3().x, tolerance) &&
                   near(a.asVec3().y, b.asVec3().y, tolerance) &&
                   near(a.asVec3().z, b.asVec3().z, tolerance);
        default:
            return a == b;
    }
}

bool hasDiagnostic(const SceneResult& result, SceneDiagnosticSeverity severity, const char* path,
        const char* fragment)
{
    for (size_t i = 0; i < result.diagnostics.size(); ++i)
    {
        const SceneDiagnostic& diagnostic = result.diagnostics[i];
        if (diagnostic.severity != severity) continue;
        if (strcmp(diagnostic.path.c_str(), path) != 0) continue;
        if (!fragment || strstr(diagnostic.message.c_str(), fragment)) return true;
    }
    return false;
}

void dump(const SceneResult& result)
{
    printf("%s", result.describe().c_str());
}

size_t countProperties = 0;

void printFirstDifference(const ct::String& a, const ct::String& b)
{
    size_t i = 0;
    while (i < a.size() && i < b.size() && a[i] == b[i]) ++i;
    size_t start = i;
    while (start > 0 && a[start - 1] != '\n') --start;
    size_t endA = i;
    while (endA < a.size() && a[endA] != '\n') ++endA;
    size_t endB = i;
    while (endB < b.size() && b[endB] != '\n') ++endB;
    printf("  first text:  %.*s\n", static_cast<int>(endA - start), a.c_str() + start);
    printf("  second text: %.*s\n", static_cast<int>(endB - start), b.c_str() + start);
}

void fillProperties(Node* node, const ct::String& className)
{
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList(className, list);
    for (size_t i = 0; i < list.size(); ++i)
    {
        const PropertyInfo& info = list[i];
        if (info.isGroup() || !(info.usage & PropertyUsage::Storage)) continue;
        PropertyValue defaultValue;
        if (!NodeClassDB::getDefaultPropertyValue(className, info.name, defaultValue)) continue;
        PropertyValue value;
        if (!makeValue(className, info, defaultValue, value)) continue;
        NodeClassDB::setProperty(node, info.name, value);
    }
}

void compareProperties(const Node* a, const Node* b, const ct::String& className, const char* label,
        const char* detail)
{
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList(className, list);
    for (size_t i = 0; i < list.size(); ++i)
    {
        const PropertyInfo& info = list[i];
        if (info.isGroup() || !(info.usage & PropertyUsage::Storage)) continue;
        PropertyValue expected;
        PropertyValue actual;
        const bool readableA = NodeClassDB::getProperty(a, info.name, expected);
        const bool readableB = NodeClassDB::getProperty(b, info.name, actual);
        CHECK(readableA == readableB);
        if (!readableA || !readableB) continue;
        ++countProperties;
        const bool same =
                approximate(info.name) ? valuesNear(actual, expected, 1e-4) : actual == expected;
        if (!same)
        {
            printf("FAIL %s %s.%s (%s): loaded value differs from the saved one\n", label,
                    className.c_str(), info.name.c_str(), detail);
            ++failures;
        }
    }
}

bool roundTrip(Node* original, const ct::String& className, const char* label, const char* detail)
{
    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(original, text, &saved));

    Node target;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &target, loaded);
    CHECK(copy != nullptr);
    if (!loaded.success())
    {
        printf("%s %s %s:\n", label, className.c_str(), detail);
        dump(loaded);
        ++failures;
    }
    if (!copy) return false;

    const int before = failures;
    compareProperties(original, copy, className, label, detail);

    ct::String again;
    CHECK(SceneSerializer::save(copy, again));
    if (again != text)
    {
        printf("FAIL %s %s %s: saving the loaded node gives another text\n", label,
                className.c_str(), detail);
        printFirstDifference(text, again);
        ++failures;
    }
    return failures == before;
}


template<typename T>
T* roundTripNode(T* source, Node* parent, SceneResult& result, ct::String* textOut = nullptr)
{
    ct::String text;
    CHECK(SceneSerializer::save(source, text, &result));
    if (textOut) *textOut = text;
    Node* loaded = SceneSerializer::load(text, parent, result);
    CHECK(loaded != nullptr);
    if (!loaded) return nullptr;
    T* typed = loaded->castTo<T>();
    CHECK(typed != nullptr);
    return typed;
}

void expectClean(const SceneResult& result, const char* what)
{
    if (result.diagnostics.size() == 0) return;
    printf("%s:\n%s", what, result.describe().c_str());
    ++failures;
}

bool sameVec3(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool sameVec4(const Math::Vec4& a, const Math::Vec4& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

bool sameMatrix(const Math::Mat4& a, const Math::Mat4& b)
{
    return memcmp(a.Data(), b.Data(), 16 * sizeof(float)) == 0;
}

bool sameJson(const ct::Json& a, const ct::Json& b)
{
    return a == b;
}

void testTree()
{
    Node3D top;
    top.setName("top");
    Node3D* base = top.createChild<Node3D>();
    base->setName("base");
    base->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    base->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));

    Camera3D* camera = base->createChild<Camera3D>();
    camera->setName("eye");
    camera->setPerspective(61.0f, 0.2f, 700.0f);
    OmniLight3D* lamp = base->createChild<OmniLight3D>();
    lamp->setName("lamp");
    lamp->setEnergy(3.5f);
    Node3D* arm = base->createChild<Node3D>();
    arm->setName("arm");
    arm->setRotationDegrees(Math::Vec3(0.0f, 45.0f, 0.0f));
    SpotLight3D* spot = arm->createChild<SpotLight3D>();
    spot->setName("spot");
    spot->setShadow(true);
    Node* plain = arm->createChild<Node>();
    plain->setName("plain");
    Node3D* tip = plain->createChild<Node3D>();
    tip->setName("tip");
    tip->setPosition(Math::Vec3(0.0f, 0.0f, -4.0f));
    Node3D* empty = base->createChild<Node3D>();
    empty->setName("");
    const ct::String generated = empty->getName();
    CHECK(strncmp(generated.c_str(), "@Node3D@", 8) == 0);
    Node3D* last = base->createChild<Node3D>();
    last->setName("last one");

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(&top, text, &saved));
    CHECK(saved.success());

    Node holder;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &holder, loaded);
    expectClean(loaded, "tree");
    CHECK(copy != nullptr && holder.getChildCount() == 1 && holder.getChild(0) == copy);
    if (!copy) return;
    CHECK(copy->getChildCount() == 1);
    Node* loadedBase = copy->getChild(0);
    CHECK(loadedBase && loadedBase->getName() == "base" && loadedBase->getChildCount() == 5);
    if (!loadedBase || loadedBase->getChildCount() != 5) return;
    const char* order[] = { "eye", "lamp", "arm", generated.c_str(), "last one" };
    const char* classes[] = { "Camera3D", "OmniLight3D", "Node3D", "Node3D", "Node3D" };
    for (size_t i = 0; i < 5; ++i)
    {
        CHECK(loadedBase->getChild(i)->getName() == order[i]);
        CHECK(strcmp(loadedBase->getChild(i)->getClassName(), classes[i]) == 0);
    }
    Node3D* loadedArm = loadedBase->getChild(2)->castTo<Node3D>();
    CHECK(loadedArm && loadedArm->getChildCount() == 2);
    if (loadedArm && loadedArm->getChildCount() == 2)
    {
        CHECK(strcmp(loadedArm->getChild(0)->getClassName(), "SpotLight3D") == 0);
        CHECK(strcmp(loadedArm->getChild(1)->getClassName(), "Node") == 0);
        Node* loadedPlain = loadedArm->getChild(1);
        CHECK(loadedPlain->getChildCount() == 1);
        Node3D* loadedTip = loadedPlain->getChild(0)->castTo<Node3D>();
        CHECK(loadedTip && loadedTip->getPosition().z == -4.0f);
        CHECK(loadedArm->getChild(0)->castTo<SpotLight3D>()->hasShadow());
    }
    Node3D* node3d = loadedBase->castTo<Node3D>();
    CHECK(node3d && node3d->getPosition().x == 1.0f && node3d->getScale().x == 2.0f);
    CHECK(loadedBase->getChild(0)->castTo<Camera3D>()->getFov() == 61.0f);
    CHECK(loadedBase->getChild(1)->castTo<OmniLight3D>()->getEnergy() == 3.5f);

    ct::String again;
    CHECK(SceneSerializer::save(copy, again));
    CHECK(again == text);

    ct::Json::Error error;
    const ct::Json document = ct::Json::parse(text, &error);
    CHECK(!error);
    CHECK(document["format"].str() == "prisma3d-scene");
    CHECK(document["version"].as_int() == 1);
    CHECK(document["root"]["class"].str() == "Node3D");
    CHECK(!document["root"].contains("properties"));
    const ct::Json& baseJson = document["root"]["children"][0];
    CHECK(baseJson["name"].str() == "base");
    CHECK(baseJson["properties"].contains("position"));
    CHECK(baseJson["properties"].contains("scale"));
    CHECK(!baseJson["properties"].contains("rotation"));
    CHECK(!baseJson["properties"].contains("visible"));
    CHECK(baseJson["properties"]["position"].size() == 3);
}

void testFloatText()
{
    Node3D holder;
    Node3D* node = holder.createChild<Node3D>();
    node->setPosition(Math::Vec3(0.1f, 1.0f / 3.0f, 16777216.0f));
    node->setScale(Math::Vec3(1e-7f, 123456.789f, -0.0f));
    ct::String text;
    CHECK(SceneSerializer::save(node, text));
    CHECK(strstr(text.c_str(), "0.1,") != nullptr);
    CHECK(strstr(text.c_str(), "0.33333334") != nullptr);
    CHECK(strstr(text.c_str(), "1e-07") != nullptr || strstr(text.c_str(), "1e-7") != nullptr);
    Node target;
    SceneResult result;
    Node3D* copy = static_cast<Node3D*>(SceneSerializer::load(text, &target, result));
    CHECK(copy != nullptr);
    if (copy)
    {
        CHECK(copy->getPosition().x == 0.1f && copy->getPosition().y == 1.0f / 3.0f);
        CHECK(copy->getPosition().z == 16777216.0f);
        CHECK(copy->getScale().x == 1e-7f && copy->getScale().y == 123456.789f);
        CHECK(signbit(copy->getScale().z));
    }

    PropertyValue wide = PropertyValue::fromFloat(0.1);
    ct::Json json = scenejson::fromValue(wide);
    PropertyValue back;
    ct::String error;
    CHECK(scenejson::toValue(json, PropertyType::Float, back, error));
    CHECK(back.asFloat() == 0.1);
    PropertyValue narrow = PropertyValue::fromFloat(static_cast<double>(0.1f));
    json = scenejson::fromValue(narrow);
    CHECK(scenejson::toValue(json, PropertyType::Float, back, error));
    CHECK(back.asFloat() == static_cast<double>(0.1f));
    json = scenejson::fromValue(PropertyValue::fromFloat(INFINITY));
    CHECK(scenejson::toValue(json, PropertyType::Float, back, error) && back.asFloat() == INFINITY);
    json = scenejson::fromValue(PropertyValue::fromInt(INT64_C(-9007199254740993)));
    CHECK(scenejson::toValue(json, PropertyType::Int, back, error));
    CHECK(back.asInt() == INT64_C(-9007199254740993));
    CHECK(!scenejson::toValue(ct::Json("text"), PropertyType::Int, back, error));
    CHECK(!scenejson::toValue(ct::Json(1.5), PropertyType::Int, back, error));
    CHECK(!scenejson::toValue(ct::Json::array(), PropertyType::Vec3, back, error));
    CHECK(error.size() > 0);

    ct::Vector<std::uint8_t> bytes;
    for (int length = 0; length < 9; ++length)
    {
        bytes.clear();
        for (int i = 0; i < length; ++i) bytes.push_back(static_cast<std::uint8_t>(i * 37 + 5));
        const ct::String encoded = scenejson::encodeBase64(bytes.data(), bytes.size());
        ct::Vector<std::uint8_t> decoded;
        CHECK(scenejson::decodeBase64(encoded, decoded));
        CHECK(decoded == bytes);
    }
    ct::Vector<std::uint8_t> sink;
    CHECK(!scenejson::decodeBase64("abc", sink));
    CHECK(!scenejson::decodeBase64("ab!d", sink));
    CHECK(!scenejson::decodeBase64("a===", sink));
}

void testNodePaths()
{
    Node3D top;
    top.setName("top");
    Node3D* ground = top.createChild<Node3D>();
    ground->setName("ground");
    Node3D* nest = top.createChild<Node3D>();
    nest->setName("nest");
    Forest3D* forest = nest->createChild<Forest3D>();
    forest->setName("forest");
    Terrain3D* terrain = top.createChild<Terrain3D>();
    terrain->setName("terrain");
    forest->setTerrain(terrain);

    Grass3D* grass = top.createChild<Grass3D>();
    grass->setName("grass");
    grass->setTerrain(terrain);
    Road3D* road = top.createChild<Road3D>();
    road->setName("road");
    road->setTerrain(terrain);
    PlanarReflector3D* mirror = top.createChild<PlanarReflector3D>();
    mirror->setName("mirror");
    MeshInstance3D* shiny = nest->createChild<MeshInstance3D>();
    shiny->setName("shiny");
    shiny->setReflector(mirror);
    Skeleton3D* skeleton = top.createChild<Skeleton3D>();
    skeleton->setName("skeleton");
    AnimationPlayer* player = top.createChild<AnimationPlayer>();
    player->setName("player");
    Ragdoll3D* ragdoll = top.createChild<Ragdoll3D>();
    ragdoll->setName("ragdoll");
    ragdoll->setSkeleton(skeleton);
    ragdoll->setAnimationPlayer(player);
    MeshInstance3D* skin = top.createChild<MeshInstance3D>();
    skin->setName("skin");
    skin->setSkeletonPath("../skeleton");
    player->setRootNode("../ground");

    CHECK(forest->getTerrainPath() == "../../terrain");
    CHECK(shiny->getReflectorPath() == "../../mirror");
    CHECK(ragdoll->getSkeletonPath() == "../skeleton");
    CHECK(ragdoll->getAnimationPlayerPath() == "../player");
    CHECK(terrain->getPathTo(terrain) == ".");
    CHECK(top.getPathTo(shiny) == "nest/shiny");
    CHECK(shiny->getPathTo(&top) == "../..");
    Node3D other;
    CHECK(top.getPathTo(&other).empty());
    Node3D twins;
    Node3D* firstTwin = twins.createChild<Node3D>();
    firstTwin->setName("twin");
    Node3D* secondTwin = twins.createChild<Node3D>();
    secondTwin->setName("twin");
    Node3D* nameless = twins.createChild<Node3D>();
    CHECK(twins.getPathTo(firstTwin) == "twin");
    CHECK(secondTwin->getName() == "twin2");
    CHECK(twins.getPathTo(secondTwin) == "twin2");
    CHECK(strncmp(nameless->getName().c_str(), "@Node3D@", 8) == 0);
    CHECK(twins.getPathTo(nameless) == nameless->getName());
    CHECK(secondTwin->getPathTo(firstTwin) == "../twin");

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(&top, text, &saved));
    expectClean(saved, "node paths save");
    ct::Json::Error error;
    ct::Json document = ct::Json::parse(text, &error);
    CHECK(!error);
    const ct::Json& children = document["root"]["children"];
    bool seenForest = false;
    for (size_t i = 0; i < children.size(); ++i)
    {
        const ct::Json& entry = children[i];
        if (entry["name"].str() == "nest")
        {
            CHECK(entry["children"][0]["properties"]["terrain"].str() == "../../terrain");
            CHECK(entry["children"][1]["properties"]["reflector"].str() == "../../mirror");
            seenForest = true;
        }
        if (entry["name"].str() == "ragdoll")
        {
            CHECK(entry["properties"]["skeleton"].str() == "../skeleton");
            CHECK(entry["properties"]["animation_player"].str() == "../player");
        }
    }
    CHECK(seenForest);

    Node holder;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &holder, loaded);
    expectClean(loaded, "node paths load");
    CHECK(copy != nullptr);
    if (!copy) return;
    Terrain3D* copiedTerrain = static_cast<Terrain3D*>(copy->findChild("terrain"));
    Node* copiedNest = copy->findChild("nest");
    CHECK(copiedTerrain && copiedNest);
    if (!copiedTerrain || !copiedNest) return;
    CHECK(static_cast<Forest3D*>(copiedNest->findChild("forest"))->getTerrain() == copiedTerrain);
    CHECK(static_cast<Grass3D*>(copy->findChild("grass"))->getTerrain() == copiedTerrain);
    CHECK(static_cast<Road3D*>(copy->findChild("road"))->getTerrain() == copiedTerrain);
    CHECK(static_cast<Node*>(
                  static_cast<MeshInstance3D*>(copiedNest->findChild("shiny"))->getReflector()) ==
            copy->findChild("mirror"));
    Ragdoll3D* copiedRagdoll = static_cast<Ragdoll3D*>(copy->findChild("ragdoll"));
    CHECK(copiedRagdoll->getSkeleton() == copy->findChild("skeleton"));
    CHECK(copiedRagdoll->getAnimationPlayer() == copy->findChild("player"));
    CHECK(static_cast<MeshInstance3D*>(copy->findChild("skin"))->getSkeleton() ==
            copy->findChild("skeleton"));
    CHECK(static_cast<AnimationPlayer*>(copy->findChild("player"))->getRootNode() == "../ground");

    ct::String again;
    CHECK(SceneSerializer::save(copy, again));
    CHECK(again == text);

    Node3D loose;
    Forest3D* lonely = loose.createChild<Forest3D>();
    lonely->setTerrainPath("../later");
    CHECK(lonely->getTerrain() == nullptr);
    CHECK(lonely->getTerrainPath() == "../later");
    Terrain3D* later = loose.createChild<Terrain3D>();
    later->setName("later");
    lonely->setTerrainPath("../later");
    CHECK(lonely->getTerrain() == later);
    CHECK(lonely->getTerrainPath() == "../later");
    lonely->setTerrainPath("");
    CHECK(lonely->getTerrain() == nullptr && lonely->getTerrainPath().empty());
    lonely->setTerrainPath("../missing");
    CHECK(lonely->getTerrain() == nullptr);
    Node3D bundle;
    Forest3D* waiting = bundle.createChild<Forest3D>();
    waiting->setTerrainPath("../late");
    Terrain3D* late = bundle.createChild<Terrain3D>();
    late->setName("late");
    CHECK(waiting->getTerrain() == nullptr);
    CHECK(waiting->getTerrainPath() == "../late");
    SceneTree* tree = SceneTree::singleton();
    CHECK(tree != nullptr);
    if (tree)
    {
        CHECK(bundle.reparent(tree->getRoot()));
        CHECK(waiting->getTerrain() == late);
        CHECK(waiting->getTerrainPath() == "../late");
    }
}


ct::String sceneText(const ct::Json& root)
{
    ct::Json document = ct::Json::object();
    document["format"] = ct::Json("prisma3d-scene");
    document["version"] = ct::Json(1);
    document["root"] = root;
    return document.dump(2);
}

ct::Json plainEntry(const char* className, const char* name)
{
    ct::Json entry = ct::Json::object();
    entry["class"] = ct::Json(className);
    entry["name"] = ct::Json(name);
    return entry;
}

void testUniqueNamesAndPaths()
{
    Node3D top;
    top.setName("top");
    Terrain3D* first = top.createChild<Terrain3D>();
    first->setName("terrain");
    Terrain3D* second = top.createChild<Terrain3D>();
    second->setName("terrain");
    Terrain3D* third = top.createChild<Terrain3D>();
    CHECK(first->getName() == "terrain" && second->getName() == "terrain2");
    CHECK(strncmp(third->getName().c_str(), "@Terrain3D@", 11) == 0);

    Forest3D* forestA = top.createChild<Forest3D>();
    forestA->setName("forest");
    forestA->setTerrain(second);
    Forest3D* forestB = top.createChild<Forest3D>();
    forestB->setName("forest");
    forestB->setTerrain(third);
    CHECK(forestB->getName() == "forest2");
    CHECK(forestA->getTerrainPath() == "../terrain2");
    CHECK(forestB->getTerrainPath() == ct::String("../") + third->getName());
    CHECK(top.getPathTo(second) == "terrain2");
    CHECK(top.getPathTo(third) == third->getName());

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(&top, text, &saved));
    expectClean(saved, "unique names save");

    const ct::Json document = ct::Json::parse(text);
    const ct::Json& children = document["root"]["children"];
    CHECK(children.size() == 5);
    CHECK(children[3]["properties"]["terrain"].str() == "../terrain2");
    CHECK(children[4]["properties"]["terrain"].str() == ct::String("../") + third->getName());

    Node holder;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &holder, loaded);
    expectClean(loaded, "unique names load");
    CHECK(copy != nullptr);
    if (!copy) return;
    CHECK(copy->getChildCount() == 5);
    Forest3D* copyA = static_cast<Forest3D*>(copy->getChild(3));
    Forest3D* copyB = static_cast<Forest3D*>(copy->getChild(4));
    CHECK(copyA->getTerrain() == copy->getChild(1));
    CHECK(copyB->getTerrain() == copy->getChild(2));
    CHECK(copy->getChild(2)->getName() == third->getName());
    ct::String again;
    CHECK(SceneSerializer::save(copy, again));
    CHECK(again == text);

    ct::Json root = plainEntry("Node3D", "scene");
    ct::Json list = ct::Json::array();
    list.push_back(plainEntry("Terrain3D", "terrain"));
    list.push_back(plainEntry("Terrain3D", "terrain"));
    ct::Json unnamed = ct::Json::object();
    unnamed["class"] = ct::Json("Terrain3D");
    list.push_back(unnamed);
    list.push_back(unnamed);
    ct::Json forest = plainEntry("Forest3D", "forest");
    ct::Json properties = ct::Json::object();
    properties["terrain"] = ct::Json("../terrain2");
    forest["properties"] = properties;
    list.push_back(forest);
    root["children"] = list;

    Node host;
    SceneResult repaired;
    Node* scene = SceneSerializer::load(sceneText(root), &host, repaired);
    CHECK(scene != nullptr && scene->getChildCount() == 5);
    CHECK(repaired.warningCount() == 1 && repaired.errorCount() == 0);
    CHECK(hasDiagnostic(repaired, SceneDiagnosticSeverity::Warning, "root.children[1].name",
            "renamed 'terrain2'"));
    if (!scene || scene->getChildCount() != 5) return;
    CHECK(scene->getChild(0)->getName() == "terrain");
    CHECK(scene->getChild(1)->getName() == "terrain2");
    CHECK(scene->getChild(2)->getName() == "Terrain3D");
    CHECK(scene->getChild(3)->getName() == "Terrain3D2");
    CHECK(static_cast<Forest3D*>(scene->getChild(4))->getTerrain() == scene->getChild(1));

    SceneResult twice;
    Node* sibling = SceneSerializer::load(sceneText(root), &host, twice);
    CHECK(sibling != nullptr && sibling->getName() == "scene2");
    CHECK(host.getChildCount() == 2 && host.findChild("scene") == scene);
}

void testMissingNodes()
{
    ct::Json unknown = plainEntry("Teleporter3D", "portal");
    ct::Json properties = ct::Json::object();
    properties["destination"] = ct::Json("level2");
    ct::Json offset = ct::Json::array();
    offset.push_back(ct::Json(0.5));
    offset.push_back(ct::Json(-2));
    offset.push_back(ct::Json(1.25));
    properties["offset"] = offset;
    ct::Json nested = ct::Json::object();
    nested["open"] = ct::Json(true);
    nested["tags"] = ct::Json::array();
    properties["state"] = nested;
    unknown["properties"] = properties;
    ct::Json data = ct::Json::object();
    data["version"] = ct::Json(3);
    data["blob"] = ct::Json("AAECAw==");
    unknown["data"] = data;
    ct::Json inside = ct::Json::array();
    ct::Json known = plainEntry("Node3D", "keeper");
    ct::Json knownProperties = ct::Json::object();
    ct::Json knownPosition = ct::Json::array();
    knownPosition.push_back(ct::Json(1.0));
    knownPosition.push_back(ct::Json(2.0));
    knownPosition.push_back(ct::Json(3.0));
    knownProperties["position"] = knownPosition;
    known["properties"] = knownProperties;
    inside.push_back(known);
    ct::Json deeper = plainEntry("Gadget", "inner");
    ct::Json deeperChildren = ct::Json::array();
    deeperChildren.push_back(plainEntry("Node3D", "leaf"));
    deeper["children"] = deeperChildren;
    inside.push_back(deeper);
    unknown["children"] = inside;

    ct::Json abstractEntry = plainEntry("VisualInstance3D", "shell");
    ct::Json abstractProperties = ct::Json::object();
    abstractProperties["layers"] = ct::Json(4);
    abstractEntry["properties"] = abstractProperties;
    ct::Json noClass = ct::Json::object();
    noClass["name"] = ct::Json("headless");
    noClass["extra"] = ct::Json(12);

    ct::Json root = plainEntry("Node3D", "scene");
    ct::Json list = ct::Json::array();
    list.push_back(plainEntry("Node3D", "before"));
    list.push_back(unknown);
    list.push_back(abstractEntry);
    list.push_back(noClass);
    list.push_back(plainEntry("Node3D", "after"));
    root["children"] = list;
    const ct::String text = sceneText(root) + "\n";

    Node host;
    SceneResult loaded;
    Node* scene = SceneSerializer::load(text, &host, loaded);
    CHECK(scene != nullptr && scene->getChildCount() == 5);
    CHECK(loaded.errorCount() == 4 && loaded.warningCount() == 0);
    CHECK(hasDiagnostic(loaded, SceneDiagnosticSeverity::Error, "root.children[1].class",
            "unknown class 'Teleporter3D'"));
    CHECK(hasDiagnostic(loaded, SceneDiagnosticSeverity::Error,
            "root.children[1].children[1].class", "unknown class 'Gadget'"));
    CHECK(hasDiagnostic(loaded, SceneDiagnosticSeverity::Error, "root.children[2].class",
            "abstract"));
    CHECK(hasDiagnostic(loaded, SceneDiagnosticSeverity::Error, "root.children[3].class",
            "missing"));
    if (!scene || scene->getChildCount() != 5) return;

    MissingNode* portal = scene->getChild(1)->castTo<MissingNode>();
    CHECK(portal != nullptr && portal->getName() == "portal");
    CHECK(portal && portal->getOriginalClass() == "Teleporter3D" &&
            portal->getOriginalScene().empty());
    CHECK(portal && portal->getChildCount() == 2);
    CHECK(portal && portal->get("original_class").asString() == "Teleporter3D");
    Node3D* keeper = portal && portal->getChildCount() == 2 ? portal->getChild(0)->castTo<Node3D>()
                                                            : nullptr;
    CHECK(keeper != nullptr && keeper->getName() == "keeper" && keeper->getPosition().z == 3.0f);
    MissingNode* gadget = portal && portal->getChildCount() == 2
                                  ? portal->getChild(1)->castTo<MissingNode>()
                                  : nullptr;
    CHECK(gadget != nullptr && gadget->getOriginalClass() == "Gadget" &&
            gadget->getChildCount() == 1 && gadget->getChild(0)->castTo<Node3D>() != nullptr);
    MissingNode* shell = scene->getChild(2)->castTo<MissingNode>();
    CHECK(shell && shell->getOriginalClass() == "VisualInstance3D");
    MissingNode* headless = scene->getChild(3)->castTo<MissingNode>();
    CHECK(headless && headless->getOriginalClass().empty() && headless->getName() == "headless");
    CHECK(!NodeClassDB::canInstantiate("MissingNode") && NodeClassDB::isAbstract("MissingNode"));
    CHECK(NodeClassDB::classExists("MissingNode"));
    CHECK(host.createChild("MissingNode") == nullptr);

    ct::String again;
    SceneResult resaved;
    CHECK(SceneSerializer::save(scene, again, &resaved));
    CHECK(resaved.diagnostics.size() == 0);
    const ct::Json savedDocument = ct::Json::parse(again);
    const ct::Json& savedChildren = savedDocument["root"]["children"];
    CHECK(savedChildren.size() == 5);
    for (size_t i = 0; i < 5 && i < savedChildren.size(); ++i)
        CHECK(savedChildren[i].dump(2) == list[i].dump(2));
    CHECK(again == text);

    keeper->setPosition(Math::Vec3(4.0f, 5.0f, 6.0f));
    portal->setName("gate");
    gadget->createChild<Node3D>()->setName("added");
    ct::String edited;
    CHECK(SceneSerializer::save(scene, edited));
    const ct::Json editedDocument = ct::Json::parse(edited);
    const ct::Json& gate = editedDocument["root"]["children"][1];
    CHECK(gate["class"].str() == "Teleporter3D" && gate["name"].str() == "gate");
    CHECK(gate["properties"].dump(2) == unknown["properties"].dump(2));
    CHECK(gate["data"].dump(2) == unknown["data"].dump(2));
    CHECK(gate["children"][0]["properties"]["position"][2].as_double() == 6.0);
    CHECK(gate["children"][1]["children"].size() == 2);
    CHECK(gate["children"][1]["children"][1]["name"].str() == "added");

    Node moved;
    CHECK(keeper->reparent(&moved));
    portal->createChild("Node3D", true);
    SceneResult trimmed;
    ct::String trimmedText;
    CHECK(SceneSerializer::save(scene, trimmedText, &trimmed));
    const ct::Json trimmedDocument = ct::Json::parse(trimmedText);
    CHECK(trimmedDocument["root"]["children"][1]["children"].size() == 2);
    CHECK(trimmedDocument["root"]["children"][1]["children"][0]["class"].str() == "Gadget");

    ct::Json lone = unknown;
    lone.erase("children");
    SceneResult loneResult;
    Node* loneScene = SceneSerializer::load(sceneText(lone), &host, loneResult);
    CHECK(loneScene != nullptr && loneScene->castTo<MissingNode>() != nullptr);
    CHECK(loneResult.errorCount() == 1);
    ct::String loneText;
    CHECK(SceneSerializer::save(loneScene, loneText));
    CHECK(ct::Json::parse(loneText)["root"].dump(2) == lone.dump(2));

    ct::Json badChildren = plainEntry("Teleporter3D", "badkids");
    badChildren["children"] = ct::Json(3);
    SceneResult badResult;
    Node* badScene = SceneSerializer::load(sceneText(badChildren), &host, badResult);
    CHECK(badScene != nullptr && badResult.errorCount() == 2);
    ct::String badText;
    CHECK(SceneSerializer::save(badScene, badText));
    CHECK(ct::Json::parse(badText)["root"].dump(2) == badChildren.dump(2));
}

void testPathAndRoad()
{
    Node holder;
    Path3D* path = holder.createChild<Path3D>();
    path->setName("route");
    Curve3D& curve = path->getCurve();
    curve.addPoint(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(2.0f, 0.0f, 1.0f));
    curve.addPoint(Math::Vec3(10.0f, 1.0f, 5.0f), Math::Vec3(-2.0f, 0.0f, -1.0f),
            Math::Vec3(3.0f, 1.0f, 0.5f));
    curve.addPoint(Math::Vec3(20.0f, 0.5f, -5.0f), Math::Vec3(-3.0f, -1.0f, 0.5f),
            Math::Vec3(0.0f, 0.0f, 0.0f));
    curve.setPointTilt(1, 0.25f);
    curve.setPointTilt(2, -0.5f);
    curve.setClosed(true);
    curve.setBakeInterval(0.37f);
    curve.setUpVectorEnabled(true);

    Road3D* road = path->createChild<Road3D>();
    road->setName("tarmac");
    ct::Vector<Math::Vec2> profile;
    profile.push_back(Math::Vec2(-3.0f, 0.0f));
    profile.push_back(Math::Vec2(-2.5f, 0.2f));
    profile.push_back(Math::Vec2(0.0f, 0.3f));
    profile.push_back(Math::Vec2(2.5f, 0.2f));
    profile.push_back(Math::Vec2(3.0f, 0.0f));
    road->setProfile(profile);
    road->setPointWidth(0, 4.5f);
    road->setPointWidth(2, 7.25f);

    Node target;
    SceneResult result;
    Path3D* copy = roundTripNode(path, &target, result);
    expectClean(result, "path and road");
    CHECK(copy != nullptr);
    if (!copy) return;
    const Curve3D& loaded = copy->getCurve();
    CHECK(loaded.getPointCount() == 3);
    for (int i = 0; i < 3 && i < loaded.getPointCount(); ++i)
    {
        CHECK(sameVec3(loaded.getPointPosition(i), curve.getPointPosition(i)));
        CHECK(sameVec3(loaded.getPointIn(i), curve.getPointIn(i)));
        CHECK(sameVec3(loaded.getPointOut(i), curve.getPointOut(i)));
        CHECK(loaded.getPointTilt(i) == curve.getPointTilt(i));
    }
    CHECK(loaded.isClosed() && loaded.isUpVectorEnabled() && loaded.getBakeInterval() == 0.37f);
    CHECK(loaded.getBakedLength() == curve.getBakedLength());

    Road3D* loadedRoad = copy->getChild(0)->castTo<Road3D>();
    CHECK(loadedRoad != nullptr);
    if (loadedRoad)
    {
        CHECK(loadedRoad->getProfile().size() == 5);
        for (size_t i = 0; i < 5 && i < loadedRoad->getProfile().size(); ++i)
        {
            CHECK(loadedRoad->getProfile()[i].x == profile[i].x);
            CHECK(loadedRoad->getProfile()[i].y == profile[i].y);
        }
        CHECK(loadedRoad->getPointWidth(0) == 4.5f && loadedRoad->getPointWidth(2) == 7.25f);
        CHECK(loadedRoad->getPointWidth(1) == road->getWidth());
    }

    Path3D* bare = holder.createChild<Path3D>();
    ct::String text;
    CHECK(SceneSerializer::save(bare, text));
    CHECK(strstr(text.c_str(), "\"data\"") == nullptr);
}

void testSkeletonAndPlayer()
{
    Node holder;
    Skeleton3D* skeleton = holder.createChild<Skeleton3D>();
    skeleton->setName("rig");
    Skeleton bones;
    CHECK(bones.addBone("hips", -1, Math::Mat4::Translation(Math::Vec3(0.0f, 1.0f, 0.0f)),
            Math::Mat4::Translation(Math::Vec3(0.0f, -1.0f, 0.0f))));
    CHECK(bones.addBone("spine", 0, Math::Mat4::Translation(Math::Vec3(0.0f, 0.5f, 0.0f)),
            Math::Mat4::Translation(Math::Vec3(0.0f, -1.5f, 0.0f))));
    CHECK(bones.addBone("head", 1, Math::Mat4::Translation(Math::Vec3(0.0f, 0.4f, 0.1f)),
            Math::Mat4::Translation(Math::Vec3(0.0f, -1.9f, -0.1f))));
    skeleton->setSkeleton(bones);
    LocalPose pose;
    pose.position = Math::Vec3(0.1f, 1.2f, -0.3f);
    pose.rotation = Math::Quaternion(0.0f, 0.38268343f, 0.0f, 0.92387953f);
    pose.scale = Math::Vec3(1.0f, 1.1f, 1.0f);
    skeleton->setBonePose(0, pose);
    pose.position = Math::Vec3(0.0f, 0.6f, 0.0f);
    skeleton->setBonePose(2, pose);

    AnimationPlayer* player = holder.createChild<AnimationPlayer>();
    player->setName("player");
    AnimationClip walk;
    walk.setName("walk");
    walk.setDuration(1.5f);
    walk.setLoopMode(LoopMode::PingPong);
    LocalPose key;
    key.position = Math::Vec3(0.0f, 1.0f, 0.0f);
    walk.setKeyframe(0, 0.0f, key);
    key.position = Math::Vec3(0.0f, 1.2f, 0.5f);
    key.rotation = Math::Quaternion(0.0f, 0.3826834f, 0.0f, 0.9238795f);
    walk.setKeyframe(0, 0.75f, key);
    walk.setKeyframe(1, 1.5f, key);
    const int nodeTrack = walk.addNodeTrack("../rig");
    walk.positionTrackInsertKey(nodeTrack, 0.0f, Math::Vec3(1.0f, 2.0f, 3.0f));
    walk.positionTrackInsertKey(nodeTrack, 1.0f, Math::Vec3(4.0f, 5.0f, 6.0f));
    walk.rotationTrackInsertKey(nodeTrack, 0.5f, Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
    walk.scaleTrackInsertKey(nodeTrack, 0.25f, Math::Vec3(2.0f, 2.0f, 2.0f));
    const int methodTrack = walk.addMethodTrack("..");
    walk.methodTrackInsertKey(methodTrack, 0.3f, "footstep");
    walk.methodTrackInsertKey(methodTrack, 0.9f, "footstep_right");
    const int valueTrack = walk.addValueTrack("../rig", "visible");
    walk.valueTrackInsertKey(valueTrack, 0.0f, 1.0f);
    walk.valueTrackInsertKey(valueTrack, 1.0f, 0.0f);
    const int animationTrack = walk.addAnimationTrack("../player");
    walk.animationTrackInsertKey(animationTrack, 1.2f, "idle");
    walk.addEvent(0.4f, "lift");
    walk.addEvent(1.1f, "land");
    AnimationClip idle;
    idle.setName("idle");
    idle.setDuration(2.0f);
    CHECK(player->addAnimation("walk", &walk));
    CHECK(player->addAnimation("idle", &idle));
    player->setBlendTime("walk", "idle", 0.35);
    player->setBlendTime("idle", "walk", 0.1);
    player->animationSetNext("walk", "idle");
    player->setSkeletonPath("../rig");
    player->setSpeedScale(1.5f);

    Node target;
    SceneResult result;
    ct::String text;
    Skeleton3D* skeletonCopy = roundTripNode(skeleton, &target, result, &text);
    AnimationPlayer* playerCopy = roundTripNode(player, &target, result);
    expectClean(result, "skeleton and player");
    CHECK(skeletonCopy && playerCopy);
    if (skeletonCopy)
    {
        CHECK(skeletonCopy->getBoneCount() == 3);
        for (int i = 0; i < 3; ++i)
        {
            CHECK(skeletonCopy->getBoneName(i) == skeleton->getBoneName(i));
            CHECK(skeletonCopy->getBoneParent(i) == skeleton->getBoneParent(i));
            CHECK(sameMatrix(skeletonCopy->getBoneRest(i), skeleton->getBoneRest(i)));
            const Bone& a = skeleton->getSkeleton().bone(static_cast<std::uint32_t>(i));
            const Bone& b = skeletonCopy->getSkeleton().bone(static_cast<std::uint32_t>(i));
            CHECK(sameMatrix(a.inverseBind, b.inverseBind));
            CHECK(sameVec3(skeletonCopy->getBonePose(i).position,
                    skeleton->getBonePose(i).position));
            CHECK(skeletonCopy->getBonePose(i).rotation.w == skeleton->getBonePose(i).rotation.w);
            CHECK(sameVec3(skeletonCopy->getBonePose(i).scale, skeleton->getBonePose(i).scale));
            CHECK(sameVec3(skeletonCopy->getBoneRestPose(i).position,
                    skeleton->getBoneRestPose(i).position));
        }
        CHECK(skeletonCopy->findBone("head") == 2);
    }
    if (playerCopy)
    {
        CHECK(playerCopy->getAnimationCount() == 2);
        const AnimationClip* copiedWalk = playerCopy->getAnimation("walk");
        CHECK(copiedWalk != nullptr && copiedWalk != &walk);
        if (copiedWalk)
        {
            CHECK(sameJson(scenejson::fromClip(*copiedWalk), scenejson::fromClip(walk)));
            CHECK(copiedWalk->duration() == 1.5f && copiedWalk->loopMode() == LoopMode::PingPong);
            CHECK(copiedWalk->tracks().size() == 2 && copiedWalk->tracks()[0].times.size() == 2);
            CHECK(copiedWalk->tracks()[0].positions[1].z == 0.5f);
            CHECK(copiedWalk->nodeTracks().size() == 1 &&
                    copiedWalk->nodeTracks()[0].positions.size() == 2 &&
                    copiedWalk->nodeTracks()[0].scales.size() == 1);
            CHECK(copiedWalk->methodTracks().size() == 1 &&
                    copiedWalk->methodTracks()[0].methods[1] == "footstep_right");
            CHECK(copiedWalk->valueTracks().size() == 1 &&
                    copiedWalk->valueTracks()[0].property == "visible" &&
                    copiedWalk->valueTracks()[0].values[1] == 0.0f);
            CHECK(copiedWalk->animationTracks().size() == 1 &&
                    copiedWalk->animationTracks()[0].animations[0] == "idle");
            CHECK(copiedWalk->events().size() == 2 && copiedWalk->events()[1].name == "land");
        }
        CHECK(playerCopy->getBlendTime("walk", "idle") == 0.35);
        CHECK(playerCopy->getBlendTime("idle", "walk") == 0.1);
        CHECK(playerCopy->animationGetNext("walk") == "idle");
        CHECK(playerCopy->getSpeedScale() == 1.5f);
        CHECK(playerCopy->getSkeletonPath() == "../rig");
    }
}

void testPhysicsData()
{
    MeshStore meshes(nullptr);
    Node holder;

    Node3D* body = holder.createChild<Node3D>();
    body->setName("body");
    CollisionShape3D* hull = body->createChild<CollisionShape3D>();
    hull->setName("hull");
    ct::Vector<Math::Vec3> corners;
    for (int i = 0; i < 8; ++i)
        corners.push_back(Math::Vec3((i & 1) ? 1.5f : -1.0f, (i & 2) ? 2.0f : -0.5f,
                (i & 4) ? 0.75f : -2.0f));
    CHECK(hull->setConvexFromPoints(corners.data(), corners.size()));

    const Math::Vec3 vertices[5] = { Math::Vec3(-10.0f, 0.0f, -10.0f),
        Math::Vec3(-10.0f, 0.5f, 10.0f), Math::Vec3(10.0f, 0.0f, 10.0f),
        Math::Vec3(10.0f, 1.0f, -10.0f), Math::Vec3(0.0f, 3.0f, 0.0f) };
    const std::uint32_t indices[9] = { 0, 1, 2, 0, 2, 3, 3, 2, 4 };
    CollisionMesh* floor = new CollisionMesh();
    CHECK(floor->build(vertices, 5, indices, 9));
    const MeshId floorMesh = meshes.create("floor");
    CHECK(meshes.setCollision(floorMesh, floor));
    CollisionShape3D* concave = body->createChild<CollisionShape3D>();
    concave->setName("concave");
    CHECK(concave->setConcaveFromMesh(floorMesh));
    CollisionShape3D* box = body->createChild<CollisionShape3D>();
    box->setName("box");
    box->setBox(Math::Vec3(2.0f, 3.0f, 4.0f));

    Node target;
    SceneResult result;
    Node3D* copy = roundTripNode(body, &target, result);
    expectClean(result, "collision shapes");
    CHECK(copy != nullptr && copy->getChildCount() == 3);
    if (copy && copy->getChildCount() == 3)
    {
        CollisionShape3D* copiedHull = copy->getChild(0)->castTo<CollisionShape3D>();
        CHECK(copiedHull->getKind() == CollisionShapeKind::ConvexHull);
        ct::Vector<Math::Vec3> points;
        CHECK(copiedHull->getHullPoints(points) && points.size() == corners.size());
        for (size_t i = 0; i < points.size() && i < corners.size(); ++i)
            CHECK(sameVec3(points[i], corners[i]));
        CHECK(copiedHull->getShape().valid());

        CollisionShape3D* copiedConcave = copy->getChild(1)->castTo<CollisionShape3D>();
        CHECK(copiedConcave->getKind() == CollisionShapeKind::ConcaveMesh);
        PhysicsServer3D* server = PhysicsServer3D::singleton();
        ct::Vector<Math::Vec3> expectedVertices;
        ct::Vector<std::uint32_t> expectedIndices;
        ct::Vector<Math::Vec3> actualVertices;
        ct::Vector<std::uint32_t> actualIndices;
        CHECK(server->shapeGetConcaveFaces(concave->getShape(), expectedVertices, expectedIndices));
        CHECK(server->shapeGetConcaveFaces(copiedConcave->getShape(), actualVertices,
                actualIndices));
        CHECK(actualVertices.size() == expectedVertices.size() && actualVertices.size() > 0);
        CHECK(actualIndices == expectedIndices);
        for (size_t i = 0; i < actualVertices.size() && i < expectedVertices.size(); ++i)
            CHECK(sameVec3(actualVertices[i], expectedVertices[i]));
        CHECK(server->shapeGetTriangleCount(copiedConcave->getShape()) == 3);

        CollisionShape3D* copiedBox = copy->getChild(2)->castTo<CollisionShape3D>();
        CHECK(copiedBox->getKind() == CollisionShapeKind::Box);
        CHECK(sameVec3(copiedBox->getBoxSize(), Math::Vec3(2.0f, 3.0f, 4.0f)));
    }
}

void testParticles()
{
    Node holder;
    ParticleSystem3D* particles = holder.createChild<ParticleSystem3D>();
    particles->setName("sparks");
    particles->addGravity(Math::Vec3(0.0f, -9.8f, 0.5f));
    particles->addDrag(0.3f);
    particles->addVortex(Math::Vec3(1.0f, 2.0f, 3.0f), 4.0f, 5.0f);
    particles->addAttractor(Math::Vec3(-1.0f, 0.0f, 2.0f), 6.0f, 7.0f, true);
    particles->addTurbulence(1.25f, 3.5f);
    particles->addColorOverLifetime(Math::Vec4(1.0f, 0.5f, 0.0f, 1.0f),
            Math::Vec4(0.2f, 0.0f, 0.0f, 0.0f));
    particles->addSizeOverLifetime(Math::Vec2(0.5f, 0.75f), Math::Vec2(2.0f, 3.0f));
    particles->getAffector(1)->enabled = false;
    particles->getAffector(4)->enabled = false;


    Node target;
    SceneResult result;
    ParticleSystem3D* copy = roundTripNode(particles, &target, result);
    expectClean(result, "particles");
    if (copy)
    {
        CHECK(copy->getAffectorCount() == 7);
        const ParticleAffectorType types[7] = { ParticleAffectorType::Gravity,
            ParticleAffectorType::Drag, ParticleAffectorType::Vortex,
            ParticleAffectorType::Attractor, ParticleAffectorType::Turbulence,
            ParticleAffectorType::ColorOverLifetime, ParticleAffectorType::SizeOverLifetime };
        for (size_t i = 0; i < 7 && i < copy->getAffectorCount(); ++i)
        {
            CHECK(copy->getAffector(i)->getType() == types[i]);
            CHECK(copy->getAffector(i)->enabled == particles->getAffector(i)->enabled);
        }
        if (copy->getAffectorCount() == 7)
        {
            CHECK(sameVec3(static_cast<GravityAffector*>(copy->getAffector(0))->gravity,
                    Math::Vec3(0.0f, -9.8f, 0.5f)));
            CHECK(static_cast<DragAffector*>(copy->getAffector(1))->drag == 0.3f);
            const VortexAffector* vortex = static_cast<VortexAffector*>(copy->getAffector(2));
            CHECK(sameVec3(vortex->center, Math::Vec3(1.0f, 2.0f, 3.0f)) &&
                    vortex->strength == 4.0f && vortex->radius == 5.0f);
            const AttractorAffector* attractor =
                    static_cast<AttractorAffector*>(copy->getAffector(3));
            CHECK(attractor->repulse && attractor->radius == 7.0f && attractor->strength == 6.0f);
            const TurbulenceAffector* turbulence =
                    static_cast<TurbulenceAffector*>(copy->getAffector(4));
            CHECK(turbulence->strength == 1.25f && turbulence->frequency == 3.5f);
            const ColorOverLifetimeAffector* color =
                    static_cast<ColorOverLifetimeAffector*>(copy->getAffector(5));
            CHECK(sameVec4(color->startColor, Math::Vec4(1.0f, 0.5f, 0.0f, 1.0f)));
            CHECK(sameVec4(color->endColor, Math::Vec4(0.2f, 0.0f, 0.0f, 0.0f)));
            const SizeOverLifetimeAffector* size =
                    static_cast<SizeOverLifetimeAffector*>(copy->getAffector(6));
            CHECK(size->startSize.x == 0.5f && size->endSize.y == 3.0f);
        }
    }
}

void testBodiesAndTrails()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    CHECK(server != nullptr);
    Node holder;

    SoftBody3D* cloth = holder.createChild<SoftBody3D>();
    cloth->setName("cloth");
    cloth->makeCloth(2.0f, 3.0f, 4, 5, 2.0f);
    cloth->pinInSphere(Math::Vec3(-1.0f, 0.0f, -1.5f), 0.4f);
    cloth->setPinned(7, true);
    cloth->setWind(Math::Vec3(0.5f, 0.0f, 1.0f));

    DestructibleBody3D* crate = holder.createChild<DestructibleBody3D>();
    crate->setName("crate");
    const Math::Vec3 sources[3] = { Math::Vec3(0.1f, 0.2f, 0.3f), Math::Vec3(-0.4f, 0.5f, 0.6f),
        Math::Vec3(0.7f, -0.8f, 0.9f) };
    crate->setSourcePoints(sources, 3);

    RibbonTrail3D* trail = holder.createChild<RibbonTrail3D>();
    trail->setName("trail");
    CHECK(trail->addChain("../left", Math::Vec4(1.0f, 0.5f, 0.25f, 1.0f),
                  Math::Vec4(0.0f, 0.0f, 1.0f, 0.0f), 0.8f, 0.1f, 0.6f) == 0);
    CHECK(trail->addBladeChain("../base", "../tip", Math::Vec4(1.0f, 1.0f, 0.0f, 1.0f),
                  Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f), 0.45f, 0.9f, 0.2f) == 1);

    Hair3D* hair = holder.createChild<Hair3D>();
    hair->setName("hair");
    SurfaceData scalp;
    scalp.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    scalp.positions.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
    scalp.positions.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    scalp.positions.push_back(Math::Vec3(1.0f, 0.2f, 1.0f));
    for (int i = 0; i < 4; ++i)
    {
        scalp.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
        scalp.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 0.25f * static_cast<float>(i + 1)));
    }
    const std::uint32_t scalpIndices[6] = { 0, 1, 2, 2, 1, 3 };
    for (std::uint32_t index: scalpIndices) scalp.indices.push_back(index);
    hair->setScalp(scalp);
    ct::Vector<HairAtlasRect> rects;
    HairAtlasRect rect;
    rect.texMulAdd = Math::Vec4(0.5f, 1.0f, 0.5f, 0.0f);
    rect.size = 1.25f;
    rects.push_back(rect);
    rect.texMulAdd = Math::Vec4(0.5f, 1.0f, 0.0f, 0.0f);
    rect.size = 0.75f;
    rects.push_back(rect);
    hair->setAtlasRects(rects);
    hair->addCollider(HairCollider::sphere(Math::Vec3(0.0f, 1.0f, 0.0f), 0.6f));
    hair->addCollider(HairCollider::capsule(Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 2.0f, 0.0f), 0.3f));
    hair->addCollider(
            HairCollider::plane(Math::Mat4::RotationY(0.5f), Math::Vec3(0.0f, 0.1f, 0.0f), 4.0f));
    HairForceField field;
    field.type = HairForceType::Plane;
    field.position = Math::Vec3(1.0f, 2.0f, 3.0f);
    field.direction = Math::Vec3(0.0f, 0.0f, 1.0f);
    field.gravity = -2.5f;
    field.range = 6.0f;
    hair->addForceField(field);

    Node target;
    SceneResult result;
    SoftBody3D* clothCopy = roundTripNode(cloth, &target, result);
    DestructibleBody3D* crateCopy = roundTripNode(crate, &target, result);
    RibbonTrail3D* trailCopy = roundTripNode(trail, &target, result);
    Hair3D* hairCopy = roundTripNode(hair, &target, result);
    expectClean(result, "bodies and trails");

    if (clothCopy)
    {
        const SurfaceData& a = cloth->getGeometry();
        const SurfaceData& b = clothCopy->getGeometry();
        CHECK(b.positions.size() == a.positions.size() && b.positions.size() == 30);
        CHECK(b.indices == a.indices);
        CHECK(b.uvs.size() == a.uvs.size() && b.normals.size() == a.normals.size());
        for (size_t i = 0; i < a.positions.size() && i < b.positions.size(); ++i)
        {
            CHECK(sameVec3(a.positions[i], b.positions[i]));
            CHECK(clothCopy->isPinned(i) == cloth->isPinned(i));
        }
        CHECK(clothCopy->isPinned(7));
        CHECK(sameVec3(clothCopy->getWind(), Math::Vec3(0.5f, 0.0f, 1.0f)));
    }
    if (crateCopy)
    {
        ct::Json a = ct::Json::object();
        ct::Json b = ct::Json::object();
        SceneResult scratch;
        CHECK(crate->saveData(a, scratch, "x") && crateCopy->saveData(b, scratch, "x"));
        CHECK(sameJson(a, b) && a["source_points"].size() == 3);
    }
    if (trailCopy)
    {
        CHECK(trailCopy->getChainCount() == 2);
        ct::Json a = ct::Json::object();
        ct::Json b = ct::Json::object();
        SceneResult scratch;
        CHECK(trail->saveData(a, scratch, "x") && trailCopy->saveData(b, scratch, "x"));
        CHECK(sameJson(a, b));
        CHECK(a["chains"][0]["emitter"].str() == "../left" && !a["chains"][0]["blade"].as_bool());
        CHECK(a["chains"][1]["tip"].str() == "../tip" && a["chains"][1]["blade"].as_bool());
        CHECK(static_cast<float>(a["chains"][1]["start_width"].as_double()) == 0.9f);
    }
    if (hairCopy)
    {
        ct::Json a = ct::Json::object();
        ct::Json b = ct::Json::object();
        SceneResult scratch;
        CHECK(hair->saveData(a, scratch, "x") && hairCopy->saveData(b, scratch, "x"));
        CHECK(sameJson(a, b));
        CHECK(hairCopy->getColliderCount() == 3);
        CHECK(hairCopy->getParams().atlasRects.size() == 2);
        CHECK(hairCopy->getParams().atlasRects[1].size == 0.75f);
        CHECK(hairCopy->getScalpTriangleCount() == hair->getScalpTriangleCount());
    }
}

void expectLoadError(const ct::String& text, Node* parent, SceneDiagnosticSeverity severity,
        const char* path, const char* fragment, bool expectNode, const char* label)
{
    SceneResult result;
    Node* node = SceneSerializer::load(text, parent, result);
    const bool found = hasDiagnostic(result, severity, path, fragment);
    if (!found)
    {
        printf("FAIL %s: expected %s at '%s' containing '%s', got:\n%s", label,
                severity == SceneDiagnosticSeverity::Error ? "an error" : "a warning", path,
                fragment, result.describe().c_str());
        ++failures;
    }
    if (severity == SceneDiagnosticSeverity::Error) CHECK(!result.success());
    CHECK((node != nullptr) == expectNode);
}

void testErrors()
{
    Node host;
    Node3D source;
    Node3D* child = source.createChild<Node3D>();
    child->setName("child");
    child->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    ct::String good;
    CHECK(SceneSerializer::save(&source, good));
    ct::Json::Error parseError;
    const ct::Json document = ct::Json::parse(good, &parseError);
    CHECK(!parseError);

    {
        SceneResult result;
        CHECK(SceneSerializer::load(good, nullptr, result) == nullptr);
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "", "no parent"));
    }

    expectLoadError("{\"format\": \"prisma3d-scene\", \"version\": 1, \"root\": {", &host,
            SceneDiagnosticSeverity::Error, "", "not valid JSON", false, "broken json");
    {
        SceneResult result;
        SceneSerializer::load("{\"format\": \"prisma3d-scene\", \"version\": 1,\n \"root\": {",
                &host, result);
        CHECK(strstr(result.describe().c_str(), "line") != nullptr);
    }
    expectLoadError("", &host, SceneDiagnosticSeverity::Error, "", "not valid JSON", false,
            "empty");
    expectLoadError("[1, 2]", &host, SceneDiagnosticSeverity::Error, "", "not a JSON object", false,
            "array document");

    {
        ct::Json other = document;
        other["format"] = ct::Json("radion-scene");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "format",
                "prisma3d-scene", false, "other format");
    }
    {
        ct::Json other = document;
        other.erase("format");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "format", "missing",
                false, "no format");
    }
    {
        ct::Json other = document;
        other["version"] = ct::Json(2);
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "version",
                "newer than this build", false, "newer version");
        other["version"] = ct::Json(0);
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "version", "positive",
                false, "version zero");
        other["version"] = ct::Json("one");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "version", "positive",
                false, "text version");
    }
    {
        ct::Json other = document;
        other.erase("root");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root", "missing",
                false, "no root");
        other["root"] = ct::Json(5);
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root",
                "expected an object", false, "root number");
    }
    {
        ct::Json other = document;
        other["extra"] = ct::Json(true);
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Warning, "extra",
                "unknown field", true, "unknown top level field");
    }
    {
        ct::Json other = document;
        other["root"]["class"] = ct::Json("Teleporter3D");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root.class",
                "unknown class 'Teleporter3D'", true, "unknown root class");
        other["root"]["class"] = ct::Json("VisualInstance3D");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root.class",
                "abstract", true, "abstract class");
        other["root"].erase("class");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root.class",
                "missing", true, "no class");
    }
    {
        ct::Json other = document;
        other["root"]["children"][0]["class"] = ct::Json("Nonsense");
        SceneResult result;
        Node* root = SceneSerializer::load(other.dump(), &host, result);
        CHECK(root != nullptr && root->getChildCount() == 1);
        MissingNode* missing = root ? root->getChild(0)->castTo<MissingNode>() : nullptr;
        CHECK(missing != nullptr && missing->getOriginalClass() == "Nonsense");
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.children[0].class",
                "unknown class 'Nonsense'"));
        if (root) root->queueFree();
    }
    {
        ct::Json other = document;
        ct::Json second = other["root"]["children"][0];
        second["name"] = ct::Json("second");
        second["properties"]["position"] = ct::Json("far away");
        second["properties"]["colour"] = ct::Json(1);
        second["properties"]["visible"] = ct::Json(false);
        second["unexpected"] = ct::Json::object();
        other["root"]["children"].push_back(second);
        ct::Json third = other["root"]["children"][0];
        third["name"] = ct::Json("third");
        third["class"] = ct::Json("Bogus");
        third["children"] = ct::Json::array();
        other["root"]["children"].push_back(third);
        ct::Json fourth = other["root"]["children"][0];
        fourth["name"] = ct::Json("fourth");
        other["root"]["children"].push_back(fourth);
        SceneResult result;
        Node* root = SceneSerializer::load(other.dump(), &host, result);
        CHECK(root != nullptr);
        CHECK(!result.success());
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error,
                "root.children[1].properties.position", "expected [x, y, z]"));
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning,
                "root.children[1].properties.colour", "unknown property for class 'Node3D'"));
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Warning, "root.children[1].unexpected",
                "unknown field"));
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.children[2].class",
                "unknown class 'Bogus'"));
        if (root)
        {
            CHECK(root->getChildCount() == 4);
            Node3D* kept = root->findChild("second") ? root->findChild("second")->castTo<Node3D>()
                                                     : nullptr;
            CHECK(kept != nullptr);
            if (kept) CHECK(!kept->isVisible());
            Node3D* first = root->getChild(0)->castTo<Node3D>();
            CHECK(first && first->getPosition().x == 1.0f);
            CHECK(root->findChild("fourth") != nullptr);
            MissingNode* third = root->findChild("third")
                                         ? root->findChild("third")->castTo<MissingNode>()
                                         : nullptr;
            CHECK(third != nullptr && third->getOriginalClass() == "Bogus");
            root->queueFree();
        }
    }
    {
        ct::Json other = document;
        other["root"]["children"][0]["properties"]["rotation"] = ct::Json::array();
        other["root"]["children"][0]["data"] = ct::Json("text");
        other["root"]["children"][0]["children"] = ct::Json(3);
        other["root"]["properties"] = ct::Json::array();
        SceneResult result;
        Node* root = SceneSerializer::load(other.dump(), &host, result);
        CHECK(root != nullptr);
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error,
                "root.children[0].properties.rotation", "expected [x, y, z]"));
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.children[0].data",
                "must be an object"));
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.children[0].children",
                "must be an array"));
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.properties",
                "must be an object"));
        if (root) root->queueFree();
    }
    {
        Node3D holder;
        Path3D* route = holder.createChild<Path3D>();
        route->getCurve().addPoint(Math::Vec3(1.0f, 2.0f, 3.0f));
        ct::String text;
        CHECK(SceneSerializer::save(route, text));
        ct::Json other = ct::Json::parse(text);
        other["root"]["data"]["version"] = ct::Json(9);
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root.data.version",
                "newer than this build", true, "newer data version");
        other["root"]["data"]["version"] = ct::Json(1);
        other["root"]["data"]["points"][0].erase("position");
        other["root"]["data"]["points"].push_back(ct::Json("not a point"));
        SceneResult result;
        Node* partial = SceneSerializer::load(other.dump(), &host, result);
        CHECK(partial != nullptr);
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.data.points[0]",
                "missing position"));
        CHECK(hasDiagnostic(result, SceneDiagnosticSeverity::Error, "root.data.points[1]",
                "expected an object"));
        if (partial) partial->queueFree();
    }
    {
        Node3D holder;
        Skeleton3D* rig = holder.createChild<Skeleton3D>();
        Skeleton bones;
        CHECK(bones.addBone("a", -1, Math::Mat4::Identity(), Math::Mat4::Identity()));
        CHECK(bones.addBone("b", 0, Math::Mat4::Identity(), Math::Mat4::Identity()));
        rig->setSkeleton(bones);
        ct::String text;
        CHECK(SceneSerializer::save(rig, text));
        ct::Json other = ct::Json::parse(text);
        other["root"]["data"]["bones"][1]["parent"] = ct::Json(1);
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root.data.bones",
                "cycle", true, "bone cycle");
        other = ct::Json::parse(text);
        other["root"]["data"]["bones"][0]["name"] = ct::Json("b");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error, "root.data.bones[1]",
                "repeated name", true, "repeated bone");
    }
    {
        Node3D holder;
        MeshInstance3D* instance = holder.createChild<MeshInstance3D>();
        instance->setSurfaceVisible(2, false);
        ct::String text;
        CHECK(SceneSerializer::save(instance, text));
        ct::Json other = ct::Json::parse(text);
        other["root"]["data"]["hidden_surfaces"][0] = ct::Json("two");
        expectLoadError(other.dump(), &host, SceneDiagnosticSeverity::Error,
                "root.data.hidden_surfaces[0]", "not a surface index", true, "bad surface");
    }
}

void testFiles()
{
    Node3D source;
    Camera3D* camera = source.createChild<Camera3D>();
    camera->setName("main camera");
    camera->setPerspective(48.0f, 0.3f, 900.0f);
    OmniLight3D* light = source.createChild<OmniLight3D>();
    light->setName("light");
    light->setEnergy(2.25f);

    ct::String path = PRISMA3D_TEST_DIR;
    path += "/scene_serializer/nested/scene.scene.json";
    SceneResult saved;
    CHECK(SceneSerializer::saveFile(&source, path, &saved));
    CHECK(saved.success());

    Node holder;
    SceneResult loaded;
    Node* copy = SceneSerializer::loadFile(path, &holder, loaded);
    expectClean(loaded, "file");
    CHECK(copy != nullptr && copy->getChildCount() == 2);
    if (copy && copy->getChildCount() == 2)
    {
        CHECK(copy->getChild(0)->castTo<Camera3D>()->getFov() == 48.0f);
        CHECK(copy->getChild(1)->castTo<OmniLight3D>()->getEnergy() == 2.25f);
    }

    SceneResult missing;
    ct::String nowhere = PRISMA3D_TEST_DIR;
    nowhere += "/scene_serializer/does_not_exist.scene.json";
    CHECK(SceneSerializer::loadFile(nowhere, &holder, missing) == nullptr);
    CHECK(hasDiagnostic(missing, SceneDiagnosticSeverity::Error, "", "could not read"));

    SceneResult unwritable;
    ct::String bad = PRISMA3D_TEST_DIR;
    bad += "/scene_serializer/nested/scene.scene.json/inside";
    CHECK(!SceneSerializer::saveFile(&source, bad, &unwritable));
    CHECK(hasDiagnostic(unwritable, SceneDiagnosticSeverity::Error, "", "could not write"));

    SceneResult none;
    ct::String text;
    CHECK(!SceneSerializer::save(nullptr, text, &none));
    CHECK(!none.success());
}

void testNoDataOnFreshNodes()
{
    ct::Vector<ct::String> classes;
    NodeClassDB::getClassList(classes);
    for (size_t c = 0; c < classes.size(); ++c)
    {
        if (!NodeClassDB::canInstantiate(classes[c])) continue;
        Node holder;
        Node* fresh = holder.createChild(classes[c].c_str());
        ct::String text;
        CHECK(SceneSerializer::save(fresh, text));
        if (strstr(text.c_str(), "\"data\"") != nullptr)
        {
            printf("FAIL %s: a fresh node writes data\n", classes[c].c_str());
            ++failures;
        }
    }
}

void testEveryClass()
{
    ct::Vector<ct::String> classes;
    NodeClassDB::getClassList(classes);
    size_t tested = 0;
    for (size_t c = 0; c < classes.size(); ++c)
    {
        const ct::String& className = classes[c];
        if (!NodeClassDB::canInstantiate(className)) continue;
        ++tested;

        Node holder;
        Node* original = holder.createChild(className.c_str());
        CHECK(original != nullptr);
        if (!original) continue;
        original->setName("subject");
        fillProperties(original, className);
        roundTrip(original, className, "all values", "");

        Node fresh;
        Node* untouched = fresh.createChild(className.c_str());
        ct::String text;
        CHECK(SceneSerializer::save(untouched, text));
        CHECK(strstr(text.c_str(), "\"properties\"") == nullptr);
    }
    printf("%zu classes, %zu property comparisons\n", tested, countProperties);
}

void testDependentStates()
{
    ct::Vector<ct::String> classes;
    NodeClassDB::getClassList(classes);
    size_t states = 0;
    for (size_t c = 0; c < classes.size(); ++c)
    {
        const ct::String& className = classes[c];
        if (!NodeClassDB::canInstantiate(className)) continue;

        ct::Vector<PropertyInfo> list;
        NodeClassDB::getPropertyList(className, list);
        ct::Vector<PropertyInfo> stored;
        ct::Vector<PropertyValue> defaults;
        ct::Vector<PropertyValue> changed;
        for (size_t i = 0; i < list.size(); ++i)
        {
            const PropertyInfo& info = list[i];
            if (info.isGroup() || !(info.usage & PropertyUsage::Storage)) continue;
            PropertyValue defaultValue;
            if (!NodeClassDB::getDefaultPropertyValue(className, info.name, defaultValue)) continue;
            PropertyValue value;
            if (!makeValue(className, info, defaultValue, value)) continue;
            stored.push_back(info);
            defaults.push_back(defaultValue);
            changed.push_back(value);
        }

        for (size_t p = 0; p < stored.size(); ++p)
        {
            {
                Node holder;
                Node* subject = holder.createChild(className.c_str());
                NodeClassDB::setProperty(subject, stored[p].name, changed[p]);
                for (size_t q = 0; q < stored.size(); ++q)
                    if (q != p) NodeClassDB::setProperty(subject, stored[q].name, defaults[q]);
                ct::String detail = ct::String("only ") + stored[p].name + " changed";
                roundTrip(subject, className, "dependent", detail.c_str());
                ++states;
            }
            {
                Node holder;
                Node* subject = holder.createChild(className.c_str());
                for (size_t q = 0; q < stored.size(); ++q)
                    NodeClassDB::setProperty(subject, stored[q].name, changed[q]);
                NodeClassDB::setProperty(subject, stored[p].name, defaults[p]);
                ct::String detail = ct::String("only ") + stored[p].name + " at the default";
                roundTrip(subject, className, "dependent", detail.c_str());
                ++states;
            }
        }
    }
    printf("%zu dependent states\n", states);
}
} // namespace

int main()
{
    PhysicsServer3D physics;
    SceneTree tree;
    tree.initialize();
    registerSceneTypes();

    testEveryClass();
    testDependentStates();
    testNoDataOnFreshNodes();
    testTree();
    testFloatText();
    testNodePaths();
    testUniqueNamesAndPaths();
    testMissingNodes();
    testPathAndRoad();
    testSkeletonAndPlayer();
    testPhysicsData();
    testParticles();
    testBodiesAndTrails();
    testErrors();
    testFiles();

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("scene serializer ok\n");
    return 0;
}
