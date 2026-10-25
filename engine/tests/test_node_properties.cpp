#include "Check.h"
#include "PropertyFixtures.h"

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/Animation.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/BoneAttachment3D.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/CharacterBody3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Destructible3D.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/PropertyValue.h"
#include "prisma3d/scene/Ragdoll3D.h"
#include "prisma3d/scene/ReflectionProbe3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

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

size_t propertyChecks = 0;
size_t skipped = 0;
size_t totalProperties = 0;
size_t totalAccessible = 0;
size_t totalDeclaredOnly = 0;

void failProperty(const ct::String& className, const ct::String& property, const char* message)
{
    printf("FAIL %s.%s: %s\n", className.c_str(), property.c_str(), message);
    ++failures;
}

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

void testPropertyValue()
{
    const PropertyValue nil;
    CHECK(nil.isNil());
    CHECK(nil.type() == PropertyType::Nil);
    CHECK(nil == PropertyValue());
    CHECK(!nil.asBool());
    CHECK(nil.asInt() == 0);
    CHECK(nil.asFloat() == 0.0);
    CHECK(nil.asString().size() == 0);

    const PropertyValue yes = PropertyValue::fromBool(true);
    const PropertyValue no = PropertyValue::fromBool(false);
    CHECK(yes.type() == PropertyType::Bool);
    CHECK(yes.asBool() && !no.asBool());
    CHECK(yes != no);
    CHECK(yes == PropertyValue::fromBool(true));
    CHECK(yes != nil);

    const PropertyValue big = PropertyValue::fromInt(INT64_C(-9007199254740993));
    CHECK(big.type() == PropertyType::Int);
    CHECK(big.asInt() == INT64_C(-9007199254740993));
    CHECK(big == PropertyValue::fromInt(INT64_C(-9007199254740993)));
    CHECK(big != PropertyValue::fromInt(INT64_C(-9007199254740992)));
    CHECK(PropertyValue::fromInt(4294967295LL).asInt() == 4294967295LL);

    const PropertyValue real = PropertyValue::fromFloat(0.1);
    CHECK(real.type() == PropertyType::Float);
    CHECK(real.asFloat() == 0.1);
    CHECK(real == PropertyValue::fromFloat(0.1));
    CHECK(real != PropertyValue::fromFloat(0.1000000001));
    CHECK(real != PropertyValue::fromFloat(static_cast<double>(0.1f)));
    CHECK(PropertyValue::fromFloat(static_cast<double>(0.1f)).asFloat() ==
            static_cast<double>(0.1f));
    CHECK(PropertyValue::fromFloat(0.0) == PropertyValue::fromFloat(-0.0));

    CHECK(PropertyValue::fromInt(3) != PropertyValue::fromFloat(3.0));
    CHECK(PropertyValue::fromInt(1) != PropertyValue::fromBool(true));

    const PropertyValue v2 = PropertyValue::fromVec2(Math::Vec2(1.0f, 2.0f));
    CHECK(v2.type() == PropertyType::Vec2);
    CHECK(v2.asVec2().x == 1.0f && v2.asVec2().y == 2.0f);
    CHECK(v2 == PropertyValue::fromVec2(Math::Vec2(1.0f, 2.0f)));
    CHECK(v2 != PropertyValue::fromVec2(Math::Vec2(1.0f, 2.0000002f)));
    CHECK(v2 != PropertyValue::fromVec2(Math::Vec2(2.0f, 1.0f)));
    CHECK(v2.asVec3() == Math::Vec3(0.0f, 0.0f, 0.0f));

    const PropertyValue v3 = PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(v3.type() == PropertyType::Vec3);
    CHECK(v3.asVec3().z == 3.0f);
    CHECK(v3 == PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(v3 != PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0000002f)));

    const PropertyValue v4 = PropertyValue::fromVec4(Math::Vec4(1.0f, 2.0f, 3.0f, 4.0f));
    CHECK(v4.type() == PropertyType::Vec4);
    CHECK(v4.asVec4().w == 4.0f);
    CHECK(v4 != PropertyValue::fromVec4(Math::Vec4(1.0f, 2.0f, 3.0f, 4.0000005f)));

    const PropertyValue quat = PropertyValue::fromQuat(Math::Quaternion(0.5f, 0.5f, 0.5f, 0.5f));
    CHECK(quat.type() == PropertyType::Quat);
    CHECK(quat.asQuat().x == 0.5f && quat.asQuat().w == 0.5f);
    CHECK(PropertyValue().asQuat().w == 1.0f);

    const PropertyValue color = PropertyValue::fromColor(Math::Vec4(1.0f, 2.0f, 3.0f, 4.0f));
    CHECK(color.type() == PropertyType::Color);
    CHECK(color.asColor().w == 4.0f);
    CHECK(color != v4);
    CHECK(color.asVec4().w == 0.0f);
    CHECK(v4.asColor().w == 1.0f);
    CHECK(quat != PropertyValue::fromVec4(Math::Vec4(0.5f, 0.5f, 0.5f, 0.5f)));

    const PropertyValue text = PropertyValue::fromString("a/b");
    const PropertyValue path = PropertyValue::fromNodePath("a/b");
    const PropertyValue resource = PropertyValue::fromResourcePath("a/b");
    CHECK(text.type() == PropertyType::String);
    CHECK(path.type() == PropertyType::NodePath);
    CHECK(resource.type() == PropertyType::ResourcePath);
    CHECK(text.asString() == "a/b" && path.asString() == "a/b" && resource.asString() == "a/b");
    CHECK(text != path && path != resource && text != resource);
    CHECK(text == PropertyValue::fromString("a/b"));
    CHECK(text != PropertyValue::fromString("a/c"));
    CHECK(PropertyValue::fromString("").asString().size() == 0);
    CHECK(PropertyValue::fromString("") == PropertyValue::fromString(""));
    CHECK(PropertyValue::fromString("") != nil);
    CHECK(v3.asString().size() == 0);

    CHECK(PropertyValue::fromInt(7).asFloat() == 7.0);
    CHECK(PropertyValue::fromFloat(7.9).asInt() == 7);
    CHECK(PropertyValue::fromFloat(-7.9).asInt() == -7);
    CHECK(PropertyValue::fromFloat(0.5).asBool());
    CHECK(!PropertyValue::fromFloat(0.0).asBool());
    CHECK(PropertyValue::fromInt(2).asBool());
    CHECK(!PropertyValue::fromInt(0).asBool());
    CHECK(yes.asInt() == 1 && no.asInt() == 0);
    CHECK(yes.asFloat() == 1.0 && no.asFloat() == 0.0);

    CHECK(PropertyValue::canConvert(PropertyType::Int, PropertyType::Float));
    CHECK(PropertyValue::canConvert(PropertyType::Float, PropertyType::Int));
    CHECK(PropertyValue::canConvert(PropertyType::Bool, PropertyType::Int));
    CHECK(PropertyValue::canConvert(PropertyType::Int, PropertyType::Bool));
    CHECK(PropertyValue::canConvert(PropertyType::Bool, PropertyType::Float));
    CHECK(PropertyValue::canConvert(PropertyType::Float, PropertyType::Bool));
    CHECK(PropertyValue::canConvert(PropertyType::String, PropertyType::NodePath));
    CHECK(PropertyValue::canConvert(PropertyType::NodePath, PropertyType::String));
    CHECK(PropertyValue::canConvert(PropertyType::String, PropertyType::ResourcePath));
    CHECK(PropertyValue::canConvert(PropertyType::ResourcePath, PropertyType::String));
    CHECK(!PropertyValue::canConvert(PropertyType::NodePath, PropertyType::ResourcePath));
    CHECK(!PropertyValue::canConvert(PropertyType::ResourcePath, PropertyType::NodePath));
    CHECK(!PropertyValue::canConvert(PropertyType::Vec3, PropertyType::Color));
    CHECK(!PropertyValue::canConvert(PropertyType::Vec4, PropertyType::Color));
    CHECK(!PropertyValue::canConvert(PropertyType::Color, PropertyType::Vec4));
    CHECK(!PropertyValue::canConvert(PropertyType::Quat, PropertyType::Vec4));
    CHECK(!PropertyValue::canConvert(PropertyType::Vec2, PropertyType::Vec3));
    CHECK(!PropertyValue::canConvert(PropertyType::String, PropertyType::Int));
    CHECK(!PropertyValue::canConvert(PropertyType::Int, PropertyType::String));
    CHECK(!PropertyValue::canConvert(PropertyType::Nil, PropertyType::Int));
    CHECK(PropertyValue::canConvert(PropertyType::Vec3, PropertyType::Nil));
    CHECK(PropertyValue::canConvert(PropertyType::Vec3, PropertyType::Vec3));

    PropertyValue converted;
    CHECK(PropertyValue::fromInt(5).convertTo(PropertyType::Float, converted));
    CHECK(converted.type() == PropertyType::Float && converted.asFloat() == 5.0);
    CHECK(PropertyValue::fromFloat(5.5).convertTo(PropertyType::Int, converted));
    CHECK(converted.type() == PropertyType::Int && converted.asInt() == 5);
    CHECK(PropertyValue::fromInt(3).convertTo(PropertyType::Bool, converted));
    CHECK(converted.type() == PropertyType::Bool && converted.asBool());
    CHECK(text.convertTo(PropertyType::NodePath, converted));
    CHECK(converted.type() == PropertyType::NodePath && converted.asString() == "a/b");
    CHECK(resource.convertTo(PropertyType::String, converted));
    CHECK(converted.type() == PropertyType::String && converted.asString() == "a/b");
    CHECK(v3.convertTo(PropertyType::Vec3, converted));
    CHECK(converted == v3);
    CHECK(!v3.convertTo(PropertyType::Color, converted));
    CHECK(!text.convertTo(PropertyType::Int, converted));

    PropertyValue copy = text;
    CHECK(copy == text);
    copy = v3;
    CHECK(copy == v3 && copy.asString().size() == 0);

    CHECK(strcmp(PropertyValue::typeName(PropertyType::Quat), "Quat") == 0);
    CHECK(strcmp(PropertyValue::typeName(PropertyType::ResourcePath), "ResourcePath") == 0);
}


struct ClassStats
{
    size_t own = 0;
    size_t total = 0;
    size_t accessible = 0;
    size_t declaredOnly = 0;
    size_t groups = 0;
};

void checkOrder(const ct::String& className)
{
    ct::Vector<PropertyInfo> all;
    NodeClassDB::getPropertyList(className, all);
    ct::Vector<PropertyInfo> own;
    NodeClassDB::getPropertyList(className, own, true);
    const ct::String parent = NodeClassDB::getParentClass(className);

    ct::Vector<PropertyInfo> inherited;
    if (parent.size() > 0) NodeClassDB::getPropertyList(parent, inherited);

    CHECK(all.size() == inherited.size() + own.size());
    if (all.size() != inherited.size() + own.size()) return;
    for (size_t i = 0; i < inherited.size(); ++i)
    {
        CHECK(all[i].name == inherited[i].name);
        CHECK(all[i].type == inherited[i].type);
        CHECK(all[i].usage == inherited[i].usage);
    }
    for (size_t i = 0; i < own.size(); ++i)
    {
        CHECK(all[inherited.size() + i].name == own[i].name);
        CHECK(all[inherited.size() + i].type == own[i].type);
    }

    for (size_t i = 0; i < all.size(); ++i)
    {
        if (all[i].isGroup())
        {
            CHECK(all[i].type == PropertyType::Nil);
            CHECK((all[i].usage & (PropertyUsage::Storage | PropertyUsage::Editor)) == 0);
            continue;
        }
        for (size_t j = i + 1; j < all.size(); ++j)
        {
            if (all[j].isGroup()) continue;
            if (all[i].name == all[j].name) failProperty(className, all[i].name, "duplicated name");
        }
    }
}

void checkClassProperties(const ct::String& className, ClassStats& stats)
{
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList(className, list);
    ct::Vector<PropertyInfo> own;
    NodeClassDB::getPropertyList(className, own, true);
    stats.total = 0;
    stats.own = 0;
    for (size_t i = 0; i < list.size(); ++i)
        if (!list[i].isGroup()) ++stats.total;
    for (size_t i = 0; i < own.size(); ++i)
    {
        if (own[i].isGroup()) ++stats.groups;
        else
            ++stats.own;
    }

    Node holder;
    Node* reference = holder.createChild(className.c_str());
    CHECK(reference != nullptr);
    if (!reference) return;

    for (size_t i = 0; i < list.size(); ++i)
    {
        const PropertyInfo& info = list[i];
        if (info.isGroup()) continue;
        ++propertyChecks;

        PropertyInfo found;
        CHECK(NodeClassDB::getPropertyInfo(className, info.name, found));
        CHECK(found.name == info.name && found.type == info.type && found.hint == info.hint);

        PropertyValue current;
        const bool readable = NodeClassDB::getProperty(reference, info.name, current);
        PropertyValue stored;
        const bool hasDefault = NodeClassDB::getDefaultPropertyValue(className, info.name, stored);
        const bool storable = (info.usage & (PropertyUsage::Storage | PropertyUsage::Editor)) != 0;

        if (hasDefault != (readable && storable))
            failProperty(className, info.name,
                    "default value presence does not match getter and usage");
        if (readable)
        {
            if (current.type() != info.type)
                failProperty(className, info.name, "getter type differs from the declared type");
            if (hasDefault && stored != current)
                failProperty(className, info.name, "default value differs from a new instance");
        }

        if (!readable)
        {
            ++stats.declaredOnly;
            PropertyValue probe;
            if (NodeClassDB::setProperty(reference, info.name, probe))
                failProperty(className, info.name, "setter without getter was accepted for Nil");
            continue;
        }

        if (info.type == PropertyType::ResourcePath)
        {
            ++stats.accessible;
            continue;
        }

        Node* subject = holder.createChild(className.c_str());
        PropertyValue value;
        if (!makeValue(className, info, current, value))
        {
            printf("skipped %s.%s: no distinct value to write\n", className.c_str(),
                    info.name.c_str());
            ++skipped;
            ++stats.accessible;
            continue;
        }

        if (!NodeClassDB::setProperty(subject, info.name, value))
        {
            if (!NodeClassDB::setProperty(subject, info.name, current))
            {
                ++stats.declaredOnly;
                continue;
            }
            failProperty(className, info.name, "setProperty refused a value of the declared type");
            continue;
        }
        ++stats.accessible;

        PropertyValue back;
        if (!NodeClassDB::getProperty(subject, info.name, back))
        {
            failProperty(className, info.name, "getProperty failed after setProperty");
            continue;
        }
        const bool same = approximate(info.name) ? valuesNear(back, value, 1e-4) : back == value;
        if (!same) failProperty(className, info.name, "read back differs from the value written");

        PropertyValue again;
        NodeClassDB::setProperty(subject, info.name, back);
        NodeClassDB::getProperty(subject, info.name, again);
        if (!valuesNear(again, back, 1e-6))
            failProperty(className, info.name, "writing the value read back changes it");
    }
}

void testRegisteredProperties(bool withServer)
{
    PhysicsServer3D* physics = withServer ? new PhysicsServer3D() : nullptr;
    {
        SceneTree tree;
        tree.initialize();

        ct::Vector<ct::String> classes;
        NodeClassDB::getClassList(classes);
        CHECK(classes.size() == 86);

        if (withServer)
            printf("%-26s %5s %5s %5s %5s %6s\n", "class", "own", "total", "set", "decl", "groups");
        for (size_t c = 0; c < classes.size(); ++c)
        {
            const ct::String& name = classes[c];
            checkOrder(name);

            ClassStats stats;
            if (NodeClassDB::canInstantiate(name))
            {
                checkClassProperties(name, stats);
            }
            else
            {
                ct::Vector<PropertyInfo> list;
                NodeClassDB::getPropertyList(name, list);
                ct::Vector<PropertyInfo> own;
                NodeClassDB::getPropertyList(name, own, true);
                for (size_t i = 0; i < list.size(); ++i)
                {
                    if (list[i].isGroup()) continue;
                    ++stats.total;
                    PropertyValue value;
                    CHECK(!NodeClassDB::getDefaultPropertyValue(name, list[i].name, value));
                }
                for (size_t i = 0; i < own.size(); ++i)
                {
                    if (own[i].isGroup()) ++stats.groups;
                    else
                        ++stats.own;
                }
            }
            if (withServer)
            {
                totalProperties += stats.own;
                totalAccessible += stats.accessible;
                totalDeclaredOnly += stats.declaredOnly;
                printf("%-26s %5zu %5zu %5zu %5zu %6zu\n", name.c_str(), stats.own, stats.total,
                        stats.accessible, stats.declaredOnly, stats.groups);
            }
        }
    }
    delete physics;
    if (withServer) printf("declared %zu own properties across the classes\n", totalProperties);
    printf("%s a physics server: %zu checks (instance level)\n", withServer ? "with" : "without",
            propertyChecks);
}

void testLookups()
{
    PhysicsServer3D physics;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    Camera3D* camera = root->createChild<Camera3D>();
    CHECK(!NodeClassDB::setProperty(camera, "no_such_property", PropertyValue::fromFloat(1.0)));
    PropertyValue value;
    CHECK(!NodeClassDB::getProperty(camera, "no_such_property", value));
    CHECK(!NodeClassDB::setProperty(nullptr, "fov", PropertyValue::fromFloat(1.0)));
    CHECK(!NodeClassDB::getProperty(nullptr, "fov", value));
    CHECK(!camera->set("no_such_property", PropertyValue::fromBool(true)));
    bool valid = true;
    camera->get("no_such_property", &valid);
    CHECK(!valid);
    CHECK(camera->get("no_such_property").isNil());

    CHECK(camera->set("fov", PropertyValue::fromFloat(60.0)));
    CHECK(camera->getFov() == 60.0f);
    CHECK(camera->get("fov", &valid) == PropertyValue::fromFloat(60.0));
    CHECK(valid);
    CHECK(camera->set("fov", PropertyValue::fromInt(90)));
    CHECK(camera->getFov() == 90.0f);
    CHECK(camera->get("fov") == PropertyValue::fromFloat(90.0));
    CHECK(!camera->set("fov", PropertyValue::fromString("90")));
    CHECK(!camera->set("fov", PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0f))));
    CHECK(!camera->set("fov", PropertyValue()));
    CHECK(camera->getFov() == 90.0f);

    CHECK(camera->set("current", PropertyValue::fromInt(1)));
    CHECK(camera->isCurrent());
    CHECK(camera->get("current") == PropertyValue::fromBool(true));
    CHECK(camera->set("projection", PropertyValue::fromInt(1)));
    CHECK(camera->getProjection() == CameraProjection::Orthogonal);
    CHECK(camera->getFov() == 90.0f);
    CHECK(camera->set("projection", PropertyValue::fromFloat(0.0)));
    CHECK(camera->getProjection() == CameraProjection::Perspective);
    CHECK(camera->getNear() == 0.05f && camera->getFar() == 4000.0f);

    CHECK(camera->set("name", PropertyValue::fromString("Eye")));
    CHECK(camera->getName() == "Eye");
    CHECK(camera->get("name") == PropertyValue::fromString("Eye"));
    CHECK(camera->set("name", PropertyValue::fromNodePath("Lens")));
    CHECK(camera->getName() == "Lens");

    CHECK(camera->set("position", PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0f))));
    CHECK(camera->getPosition() == Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(camera->get("position") == PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(camera->set("rotation", PropertyValue::fromVec3(Math::Vec3(0.5f, 0.25f, 0.125f))));
    CHECK(camera->get("rotation") == PropertyValue::fromVec3(Math::Vec3(0.5f, 0.25f, 0.125f)));
    CHECK(camera->set("quaternion",
            PropertyValue::fromQuat(Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f))));
    CHECK(camera->get("quaternion") ==
            PropertyValue::fromQuat(Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f)));
    CHECK(camera->set("visible", PropertyValue::fromBool(false)));
    CHECK(!camera->isVisible());

    MeshInstance3D* mesh = root->createChild<MeshInstance3D>();
    CHECK(mesh->set("mesh", PropertyValue::fromResourcePath("res/box.mesh")));
    CHECK(mesh->get("mesh") == PropertyValue::fromResourcePath(""));
    bool meshValid = false;
    mesh->get("mesh", &meshValid);
    CHECK(meshValid);
    PropertyInfo meshInfo;
    CHECK(NodeClassDB::getPropertyInfo("MeshInstance3D", "mesh", meshInfo));
    CHECK(meshInfo.type == PropertyType::ResourcePath);
    CHECK(meshInfo.hint == PropertyHint::ResourceType);
    CHECK(meshInfo.hintString == "Mesh");
    CHECK(mesh->set("color", PropertyValue::fromColor(Math::Vec4(0.5f, 0.25f, 0.125f, 0.5f))));
    CHECK(mesh->getColor() == Math::Vec4(0.5f, 0.25f, 0.125f, 0.5f));
    CHECK(!mesh->set("color", PropertyValue::fromVec4(Math::Vec4(0.5f, 0.25f, 0.125f, 0.5f))));
    CHECK(mesh->set("skeleton", PropertyValue::fromString("../Skel")));
    CHECK(mesh->getSkeletonPath() == "../Skel");
    CHECK(mesh->get("skeleton") == PropertyValue::fromNodePath("../Skel"));
    CHECK(mesh->set("layers", PropertyValue::fromInt(6)));
    CHECK(mesh->getLayerMask() == 6u);
    CHECK(mesh->set("cast_shadow", PropertyValue::fromBool(false)));
    CHECK(!mesh->getCastShadows());

    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    CHECK(sun->set("light_energy", PropertyValue::fromFloat(2.5)));
    CHECK(sun->getEnergy() == 2.5f);
    CHECK(sun->set("light_color", PropertyValue::fromColor(Math::Vec4(0.25f, 0.5f, 0.75f, 1.0f))));
    CHECK(sun->getColor() == Math::Vec3(0.25f, 0.5f, 0.75f));
    CHECK(sun->get("light_color") ==
            PropertyValue::fromColor(Math::Vec4(0.25f, 0.5f, 0.75f, 1.0f)));
    CHECK(sun->set("shadow_enabled", PropertyValue::fromBool(true)));
    CHECK(sun->hasShadow());
    CHECK(sun->set("directional_shadow_split_2", PropertyValue::fromFloat(0.5)));
    CHECK(sun->getShadowSplit(2) == 0.5f);
    CHECK(sun->set("directional_shadow_mode", PropertyValue::fromInt(0)));
    CHECK(sun->getShadowMode() == DirectionalShadowMode::Orthogonal);
    CHECK(sun->get("light_cull_mask") == PropertyValue::fromInt(0xffffffffLL));
}

void testGroups()
{
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList("Light3D", list);
    CHECK(list.size() > 0);
    CHECK(list[0].name == "name");
    CHECK(list[0].usage == PropertyUsage::None);

    size_t lightGroup = list.size();
    size_t shadowGroup = list.size();
    size_t transformGroup = list.size();
    size_t visibilityGroup = list.size();
    for (size_t i = 0; i < list.size(); ++i)
    {
        if (!(list[i].usage & PropertyUsage::Group)) continue;
        if (list[i].name == "Light") lightGroup = i;
        if (list[i].name == "Shadow") shadowGroup = i;
        if (list[i].name == "Transform") transformGroup = i;
        if (list[i].name == "Visibility") visibilityGroup = i;
    }
    CHECK(transformGroup < visibilityGroup);
    CHECK(visibilityGroup < lightGroup);
    CHECK(lightGroup < shadowGroup);
    if (lightGroup < list.size())
    {
        CHECK(list[lightGroup].hintString == "light_");
        CHECK(list[lightGroup].type == PropertyType::Nil);
        CHECK(list[lightGroup].usage == PropertyUsage::Group);
        CHECK(list[lightGroup + 1].name == "light_color");
        CHECK(list[lightGroup + 1].hint == PropertyHint::ColorNoAlpha);
        CHECK(list[lightGroup + 1].type == PropertyType::Color);
    }
    if (shadowGroup < list.size())
    {
        CHECK(list[shadowGroup].hintString == "shadow_");
        CHECK(list[shadowGroup + 1].name == "shadow_enabled");
    }
    if (transformGroup < list.size())
    {
        CHECK(list[transformGroup].hintString == "");
        CHECK(list[transformGroup + 1].name == "position");
    }

    ct::Vector<PropertyInfo> directional;
    NodeClassDB::getPropertyList("DirectionalLight3D", directional);
    bool shadowGroupSeen = false;
    bool directionalGroupSeen = false;
    for (size_t i = 0; i < directional.size(); ++i)
    {
        if (directional[i].name == "Shadow" && (directional[i].usage & PropertyUsage::Group))
            shadowGroupSeen = true;
        if (directional[i].name == "Directional Shadow")
        {
            CHECK(shadowGroupSeen);
            CHECK(directional[i].hintString == "directional_shadow_");
            directionalGroupSeen = true;
        }
    }
    CHECK(shadowGroupSeen && directionalGroupSeen);

    PropertyInfo energy;
    CHECK(NodeClassDB::getPropertyInfo("OmniLight3D", "light_energy", energy));
    CHECK(energy.hint == PropertyHint::Range);
    CHECK(energy.rangeMin == 0.0 && energy.rangeMax == 16.0 && energy.rangeStep == 0.001);
    CHECK(energy.rangeOrGreater && !energy.rangeOrLess);
    CHECK(!NodeClassDB::getPropertyInfo("OmniLight3D", "light_energy", energy, true));
    CHECK(NodeClassDB::getPropertyInfo("Light3D", "light_energy", energy, true));
    CHECK(!NodeClassDB::getPropertyInfo("OmniLight3D", "no_such_property", energy));
    CHECK(!NodeClassDB::getPropertyInfo("NoSuchClass", "light_energy", energy));

    PropertyInfo mode;
    CHECK(NodeClassDB::getPropertyInfo("Camera3D", "projection", mode));
    CHECK(mode.hint == PropertyHint::Enum);
    CHECK(mode.enumCount() == 2);
    CHECK(mode.enumName(0) == "Perspective");
    CHECK(mode.enumName(1) == "Orthogonal");
    CHECK(mode.enumName(2).size() == 0);

    PropertyInfo layers;
    CHECK(NodeClassDB::getPropertyInfo("RigidBody3D", "collision_layer", layers));
    CHECK(layers.hint == PropertyHint::Layers3DPhysics);
    CHECK(NodeClassDB::getPropertyInfo("Camera3D", "cull_mask", layers));
    CHECK(layers.hint == PropertyHint::Layers3DRender);

    ct::Vector<PropertyInfo> none;
    NodeClassDB::getPropertyList("NoSuchClass", none);
    CHECK(none.size() == 0);
}

void testDefaults()
{
    PhysicsServer3D physics;
    PropertyValue value;
    CHECK(!NodeClassDB::getDefaultPropertyValue("Light3D", "light_energy", value));
    CHECK(NodeClassDB::getDefaultPropertyValue("OmniLight3D", "light_energy", value));
    CHECK(value == PropertyValue::fromFloat(1.0));
    CHECK(NodeClassDB::getDefaultPropertyValue("OmniLight3D", "light_energy", value));
    CHECK(value == PropertyValue::fromFloat(1.0));
    CHECK(NodeClassDB::getDefaultPropertyValue("SpotLight3D", "spot_angle", value));
    CHECK(value == PropertyValue::fromFloat(45.0));
    CHECK(NodeClassDB::getDefaultPropertyValue("Camera3D", "fov", value));
    CHECK(value == PropertyValue::fromFloat(75.0));
    CHECK(NodeClassDB::getDefaultPropertyValue("Camera3D", "scale", value));
    CHECK(value == PropertyValue::fromVec3(Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(NodeClassDB::getDefaultPropertyValue("RigidBody3D", "collision_layer", value));
    CHECK(value.type() == PropertyType::Int);
    CHECK(!NodeClassDB::getDefaultPropertyValue("Camera3D", "name", value));
    CHECK(!NodeClassDB::getDefaultPropertyValue("Camera3D", "global_position", value));
    CHECK(!NodeClassDB::getDefaultPropertyValue("Camera3D", "no_such_property", value));
    CHECK(!NodeClassDB::getDefaultPropertyValue("NoSuchClass", "fov", value));
    CHECK(NodeClassDB::getDefaultPropertyValue("MeshInstance3D", "mesh", value));
    CHECK(value == PropertyValue::fromResourcePath(""));
    CHECK(NodeClassDB::getDefaultPropertyValue("MeshInstance3D", "layers", value));
    CHECK(value == PropertyValue::fromInt(1));
    CHECK(NodeClassDB::getDefaultPropertyValue("MeshInstance3D", "skeleton", value));
    CHECK(value == PropertyValue::fromNodePath(".."));
}

bool nearf(float a, float b)
{
    return fabsf(a - b) <= 1e-5f;
}

bool nearv(const Math::Vec3& a, const Math::Vec3& b)
{
    return nearf(a.x, b.x) && nearf(a.y, b.y) && nearf(a.z, b.z);
}

void writeBodyState(RigidBody3D* body)
{
    body->setMass(3.5f);
    body->setInertia(Math::Vec3(1.0f, 2.0f, 3.0f));
    body->setLockRotation(true);
    body->setLinearDamp(0.25f, PhysicsDampMode::Replace);
    body->setAngularDamp(0.5f, PhysicsDampMode::Replace);
    body->setCanSleep(false);
    body->setLinearVelocity(Math::Vec3(1.0f, -2.0f, 3.0f));
    body->setAngularVelocity(Math::Vec3(-1.0f, 0.5f, 2.0f));
    body->setConstantForce(Math::Vec3(4.0f, 5.0f, 6.0f));
    body->setConstantTorque(Math::Vec3(-4.0f, -5.0f, -6.0f));
    body->setInterpolated(false);
    body->setFriction(0.25f);
    body->setBounce(0.5f);
    body->setCollisionLayer(6u);
    body->setCollisionMask(10u);
    body->setContactMonitor(true, 5);
}

void checkBodyState(const RigidBody3D* body)
{
    CHECK(nearf(body->getMass(), 3.5f));
    CHECK(nearv(body->getInertia(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(body->isLockRotation());
    CHECK(nearf(body->getLinearDamp(), 0.25f));
    CHECK(body->getLinearDampMode() == PhysicsDampMode::Replace);
    CHECK(nearf(body->getAngularDamp(), 0.5f));
    CHECK(body->getAngularDampMode() == PhysicsDampMode::Replace);
    CHECK(!body->canSleep());
    CHECK(nearv(body->getLinearVelocity(), Math::Vec3(1.0f, -2.0f, 3.0f)));
    CHECK(nearv(body->getAngularVelocity(), Math::Vec3(-1.0f, 0.5f, 2.0f)));
    CHECK(nearv(body->getConstantForce(), Math::Vec3(4.0f, 5.0f, 6.0f)));
    CHECK(nearv(body->getConstantTorque(), Math::Vec3(-4.0f, -5.0f, -6.0f)));
    CHECK(!body->isInterpolated());
    CHECK(nearf(body->getFriction(), 0.25f));
    CHECK(nearf(body->getBounce(), 0.5f));
    CHECK(body->getCollisionLayer() == 6u);
    CHECK(body->getCollisionMask() == 10u);
    CHECK(body->isContactMonitor());
    CHECK(body->getMaxContacts() == 5);
}

void checkServerState(const PhysicsServer3D& server, PhysicsBodyId id)
{
    CHECK(server.bodyValid(id));
    CHECK(nearf(server.bodyGetMass(id), 3.5f));
    CHECK(nearv(server.bodyGetInertia(id), Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(server.bodyIsLockRotation(id));
    CHECK(!server.bodyCanSleep(id));
    CHECK(nearv(server.bodyGetLinearVelocity(id), Math::Vec3(1.0f, -2.0f, 3.0f)));
    CHECK(nearv(server.bodyGetAngularVelocity(id), Math::Vec3(-1.0f, 0.5f, 2.0f)));
    CHECK(nearv(server.bodyGetConstantForce(id), Math::Vec3(4.0f, 5.0f, 6.0f)));
    CHECK(nearv(server.bodyGetConstantTorque(id), Math::Vec3(-4.0f, -5.0f, -6.0f)));
    CHECK(nearf(server.bodyGetFriction(id), 0.25f));
    CHECK(nearf(server.bodyGetBounce(id), 0.5f));
    CHECK(server.bodyGetCollisionLayer(id) == 6u);
    CHECK(server.bodyGetCollisionMask(id) == 10u);
}

void testPhysicsWithoutServer()
{
    CHECK(PhysicsServer3D::singleton() == nullptr);
    Node staging;
    RigidBody3D* body = staging.createChild<RigidBody3D>();
    CHECK(!body->getBody().valid());
    CHECK(nearf(body->getMass(), 1.0f));
    CHECK(body->getCollisionLayer() == 1u && body->getCollisionMask() == 1u);
    CHECK(nearf(body->getFriction(), 1.0f) && nearf(body->getBounce(), 0.0f));
    CHECK(body->canSleep() && body->isInterpolated() && !body->isSleeping());
    CHECK(body->getMaxContacts() == 8 && !body->isContactMonitor());

    CollisionShape3D* shape = body->createChild<CollisionShape3D>();
    CHECK(shape->getKind() == CollisionShapeKind::None);
    shape->setCapsule(0.4f, 1.8f);
    CHECK(shape->getKind() == CollisionShapeKind::Capsule);
    CHECK(nearf(shape->getCapsuleRadius(), 0.4f) && nearf(shape->getCapsuleHeight(), 1.8f));
    CHECK(!shape->getShape().valid());
    shape->setBoxSize(Math::Vec3(2.0f, 3.0f, 4.0f));
    CHECK(shape->getKind() == CollisionShapeKind::Capsule);
    CHECK(nearv(shape->getBoxSize(), Math::Vec3(2.0f, 3.0f, 4.0f)));
    shape->setPlaneNormal(Math::Vec3(0.0f, 0.0f, 2.0f));
    shape->setPlaneDistance(-3.0f);
    CHECK(nearv(shape->getPlaneNormal(), Math::Vec3(0.0f, 0.0f, 2.0f)));
    CHECK(nearf(shape->getPlaneDistance(), -3.0f));

    writeBodyState(body);
    checkBodyState(body);
    CHECK(!body->getBody().valid());

    body->setSleeping(true);
    CHECK(body->isSleeping());
    CHECK(nearv(body->getLinearVelocity(), Math::Vec3(0.0f, 0.0f, 0.0f)));
    body->setSleeping(false);
    CHECK(!body->isSleeping());
    body->setLinearVelocity(Math::Vec3(1.0f, -2.0f, 3.0f));
    body->setAngularVelocity(Math::Vec3(-1.0f, 0.5f, 2.0f));
    CHECK(!body->isSleeping());
    checkBodyState(body);

    StaticBody3D* wall = staging.createChild<StaticBody3D>();
    wall->setFriction(0.75f);
    wall->setBounce(2.0f);
    CHECK(nearf(wall->getFriction(), 0.75f));
    CHECK(nearf(wall->getBounce(), 1.0f));

    {
        PhysicsServer3D server;
        CHECK(!body->getBody().valid());
        checkBodyState(body);
        CHECK(shape->getKind() == CollisionShapeKind::Capsule);
        CHECK(nearf(shape->getCapsuleRadius(), 0.4f));

        body->setSleeping(true);
        CHECK(body->isSleeping());
        body->setSleeping(false);
        CHECK(!body->isSleeping());
        body->setLinearVelocity(Math::Vec3(1.0f, -2.0f, 3.0f));
        body->setAngularVelocity(Math::Vec3(-1.0f, 0.5f, 2.0f));
        body->setMaxContacts(6);
        CHECK(body->getBody().valid());
        CHECK(body->getMaxContacts() == 6);
        body->setMaxContacts(5);
        checkServerState(server, body->getBody());
        checkBodyState(body);

        wall->setBounce(0.25f);
        CHECK(wall->getBody().valid());
        CHECK(nearf(server.bodyGetFriction(wall->getBody()), 0.75f));
        CHECK(nearf(server.bodyGetBounce(wall->getBody()), 0.25f));
    }
    checkBodyState(body);
}

void testPhysicsServerAppearsLater()
{
    CHECK(PhysicsServer3D::singleton() == nullptr);
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    Node staging;

    RigidBody3D* body = staging.createChild<RigidBody3D>();
    writeBodyState(body);
    CollisionShape3D* shape = body->createChild<CollisionShape3D>();
    shape->setBox(Math::Vec3(2.0f, 4.0f, 6.0f));
    CHECK(shape->getKind() == CollisionShapeKind::Box);

    CharacterBody3D* character = staging.createChild<CharacterBody3D>();
    character->setMaxSlides(9);
    character->setCollisionLayer(4u);
    CHECK(character->getMaxSlides() == 9);
    CHECK(character->getCollisionLayer() == 4u);

    {
        PhysicsServer3D server;
        CHECK(body->reparent(root));
        CHECK(character->reparent(root));
        CHECK(body->getBody().valid());
        CHECK(character->getBody().valid());
        CHECK(body->isSimulated());
        checkServerState(server, body->getBody());
        checkBodyState(body);
        CHECK(shape->getShape().valid());
        CHECK(server.bodyGetShape(body->getBody()) == shape->getShape());
        const Math::Vec3 half = server.shapeGetBox(shape->getShape());
        CHECK(nearv(half, Math::Vec3(1.0f, 2.0f, 3.0f)));
        CHECK(server.bodyGetCollisionLayer(character->getBody()) == 4u);

        body->setMass(9.0f);
        CHECK(nearf(server.bodyGetMass(body->getBody()), 9.0f));
        CHECK(nearf(body->getMass(), 9.0f));
        shape->setBoxSize(Math::Vec3(4.0f, 4.0f, 4.0f));
        CHECK(nearv(server.shapeGetBox(shape->getShape()), Math::Vec3(2.0f, 2.0f, 2.0f)));
        CHECK(shape->getKind() == CollisionShapeKind::Box);
        shape->setSphereRadius(1.25f);
        CHECK(shape->getKind() == CollisionShapeKind::Box);
        CHECK(nearf(shape->getSphereRadius(), 1.25f));
        shape->setKind(CollisionShapeKind::Sphere);
        CHECK(shape->getKind() == CollisionShapeKind::Sphere);
        CHECK(nearf(server.shapeGetSphere(shape->getShape()), 1.25f));
        CHECK(server.bodyGetShape(body->getBody()) == shape->getShape());
        shape->setKind(CollisionShapeKind::None);
        CHECK(shape->getKind() == CollisionShapeKind::None);
        CHECK(!shape->getShape().valid());
        CHECK(!server.shapeValid(server.bodyGetShape(body->getBody())));
    }
    CHECK(nearf(body->getMass(), 9.0f));
}

void testPhysicsServerVanishes()
{
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    RigidBody3D* body = nullptr;
    CollisionShape3D* shape = nullptr;
    {
        PhysicsServer3D server;
        body = root->createChild<RigidBody3D>();
        shape = body->createChild<CollisionShape3D>();
        shape->setCylinder(0.75f, 3.0f);
        writeBodyState(body);
        checkServerState(server, body->getBody());
        checkBodyState(body);
    }
    checkBodyState(body);
    CHECK(shape->getKind() == CollisionShapeKind::Cylinder);
    CHECK(nearf(shape->getCylinderRadius(), 0.75f) && nearf(shape->getCylinderHeight(), 3.0f));
}

void testDistinctSettersKeepOtherState()
{
    PhysicsServer3D physics;
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    RigidBody3D* body = root->createChild<RigidBody3D>();
    CHECK(body->set("contact_monitor", PropertyValue::fromBool(true)));
    CHECK(body->getMaxContacts() == 8);
    CHECK(body->set("max_contacts_reported", PropertyValue::fromInt(3)));
    CHECK(body->isContactMonitor() && body->getMaxContacts() == 3);
    CHECK(body->set("contact_monitor", PropertyValue::fromBool(false)));
    CHECK(body->getMaxContacts() == 3);
    CHECK(body->set("linear_damp_mode", PropertyValue::fromInt(1)));
    CHECK(body->set("linear_damp", PropertyValue::fromFloat(2.0)));
    CHECK(body->getLinearDampMode() == PhysicsDampMode::Replace);
    CHECK(body->set("linear_damp_mode", PropertyValue::fromInt(0)));
    CHECK(nearf(body->getLinearDamp(), 2.0f));
    CHECK(body->get("angular_damp") == PropertyValue::fromFloat(0.0));

    DestructibleBody3D* crate = root->createChild<DestructibleBody3D>();
    CHECK(crate->set("seed", PropertyValue::fromInt(12345)));
    CHECK(crate->get("seed") == PropertyValue::fromInt(12345));
    CHECK(crate->set("break_speed", PropertyValue::fromFloat(3.0)));
    CHECK(!crate->isContactMonitor());
    CHECK(crate->set("contact_monitor", PropertyValue::fromBool(true)));
    CHECK(crate->set("break_speed", PropertyValue::fromFloat(0.0)));
    CHECK(crate->isContactMonitor());
    CHECK(crate->get("seed") == PropertyValue::fromInt(12345));

    Terrain3D* terrain = root->createChild<Terrain3D>();
    CHECK(terrain->set("layer_parallax_scale", PropertyValue::fromFloat(0.125)));
    CHECK(terrain->set("layer_parallax_mode", PropertyValue::fromInt(2)));
    CHECK(terrain->get("layer_parallax_scale") == PropertyValue::fromFloat(0.125));
    CHECK(terrain->get("layer_parallax_mode") == PropertyValue::fromInt(2));

    DistanceJoint3D* distance = root->createChild<DistanceJoint3D>();
    CHECK(distance->set("distance_minimum", PropertyValue::fromFloat(1.0)));
    CHECK(distance->set("distance_maximum", PropertyValue::fromFloat(2.0)));
    CHECK(!distance->isCustomDistance());
    CHECK(distance->set("custom_distance", PropertyValue::fromBool(true)));
    CHECK(distance->isCustomDistance());
    CHECK(distance->getDistanceMinimum() == 1.0f && distance->getDistanceMaximum() == 2.0f);

    MouseJoint3D* mouse = root->createChild<MouseJoint3D>();
    CHECK(mouse->set("spring/frequency", PropertyValue::fromFloat(9.0)));
    CHECK(!mouse->isUsingTarget());
    CHECK(mouse->set("target", PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0f))));
    CHECK(!mouse->isUsingTarget());
    CHECK(mouse->set("use_target", PropertyValue::fromBool(true)));
    CHECK(mouse->isUsingTarget());
    CHECK(mouse->set("use_target", PropertyValue::fromBool(false)));
    CHECK(mouse->getTarget() == Math::Vec3(1.0f, 2.0f, 3.0f));
    mouse->setTarget(Math::Vec3(4.0f, 5.0f, 6.0f));
    CHECK(mouse->isUsingTarget());

    PistonJoint3D* piston = root->createChild<PistonJoint3D>();
    CHECK(piston->set("linear_limit/upper_distance", PropertyValue::fromFloat(4.0)));
    CHECK(!piston->isLinearLimitEnabled());
    CHECK(piston->set("linear_limit/enabled", PropertyValue::fromBool(true)));
    CHECK(piston->getLinearLimitUpper() == 4.0f && piston->getLinearLimitLower() == -1.0f);

    Ragdoll3D* ragdoll = root->createChild<Ragdoll3D>();
    CHECK(ragdoll->set("collision_mask", PropertyValue::fromInt(0x30)));
    CHECK(ragdoll->getCollisionMask() == 0x30u);

    Node3D* plain = root->createChild<Node3D>();
    CHECK(plain->getCollisionShape() == CollisionShapeType::None);
    CHECK(plain->set("collision_box_max", PropertyValue::fromVec3(Math::Vec3(2.0f, 2.0f, 2.0f))));
    CHECK(plain->set("collision_box_min", PropertyValue::fromVec3(Math::Vec3(1.0f, 1.0f, 1.0f))));
    CHECK(plain->getCollisionBoxMin() == Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(plain->getCollisionBoxMax() == Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(plain->set("collision_shape", PropertyValue::fromInt(2)));
    CHECK(plain->getCollisionShape() == CollisionShapeType::Box);
    CHECK(plain->set("collision_radii", PropertyValue::fromVec3(Math::Vec3(1.0f, 2.0f, 3.0f))));
    CHECK(plain->getCollisionRadii() == Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(plain->getCollisionShape() == CollisionShapeType::Box);
}

class CountingNode : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(CountingNode, Node3D);
    static void bindProperties()
    {
        ++bindCalls;
        NodeClassDB::addPropertyGroup<CountingNode>("Counting", "counting_");
        NodeClassDB::addProperty<CountingNode>(PropertyInfo(PropertyType::Int, "counting_value"),
                &CountingNode::setValue, &CountingNode::getValue);
        NodeClassDB::addProperty<CountingNode>(PropertyInfo(PropertyType::Float, "counting_ratio"),
                &CountingNode::setRatio, &CountingNode::getRatio);
        NodeClassDB::setPropertyDefaultValue(classNameStatic(), "counting_value",
                PropertyValue::fromInt(99));
    }

    void setValue(int value)
    {
        mValue = value;
    }
    int getValue() const
    {
        return mValue;
    }
    void setRatio(float ratio)
    {
        mRatio = ratio;
    }
    float getRatio() const
    {
        return mRatio;
    }

    static int bindCalls;

private:
    int mValue = 7;
    float mRatio = 0.5f;
};

int CountingNode::bindCalls = 0;

class CountingChild : public CountingNode
{
public:
    PRISMA3D_NODE_CLASS(CountingChild, CountingNode);
};

class CountingLeaf : public CountingChild
{
public:
    PRISMA3D_NODE_CLASS(CountingLeaf, CountingChild);
    static void bindProperties()
    {
        NodeClassDB::addProperty<CountingLeaf>(PropertyInfo(PropertyType::Bool, "leaf_flag"),
                &CountingLeaf::setFlag, &CountingLeaf::getFlag);
    }
    void setFlag(bool flag)
    {
        mFlag = flag;
    }
    bool getFlag() const
    {
        return mFlag;
    }

private:
    bool mFlag = false;
};

void testCustomClasses()
{
    CHECK(CountingNode::bindCalls == 0);
    NodeClassDB::registerClass<CountingLeaf>();
    CHECK(CountingNode::bindCalls == 1);
    NodeClassDB::registerClass<CountingLeaf>();
    NodeClassDB::registerClass<CountingChild>();
    NodeClassDB::registerClass<CountingNode>();
    NodeClassDB::registerAbstractClass<CountingLeaf>();
    CHECK(CountingNode::bindCalls == 1);

    ct::Vector<PropertyInfo> inherited;
    NodeClassDB::getPropertyList("CountingChild", inherited);
    ct::Vector<PropertyInfo> base;
    NodeClassDB::getPropertyList("CountingNode", base);
    CHECK(inherited.size() == base.size());
    ct::Vector<PropertyInfo> own;
    NodeClassDB::getPropertyList("CountingChild", own, true);
    CHECK(own.size() == 0);
    NodeClassDB::getPropertyList("CountingNode", own, true);
    CHECK(own.size() == 3);

    ct::Vector<PropertyInfo> leaf;
    NodeClassDB::getPropertyList("CountingLeaf", leaf);
    CHECK(leaf.size() == base.size() + 1);
    CHECK(leaf[leaf.size() - 1].name == "leaf_flag");

    SceneTree tree;
    tree.initialize();
    Node* made = tree.getRoot()->createChild("CountingLeaf");
    CHECK(made != nullptr);
    if (!made) return;
    CHECK(made->set("counting_value", PropertyValue::fromInt(11)));
    CHECK(made->get("counting_value") == PropertyValue::fromInt(11));
    CHECK(made->set("counting_ratio", PropertyValue::fromFloat(0.25)));
    CHECK(made->castTo<CountingNode>()->getRatio() == 0.25f);
    CHECK(made->set("leaf_flag", PropertyValue::fromBool(true)));
    CHECK(made->get("leaf_flag") == PropertyValue::fromBool(true));
    CHECK(made->set("position", PropertyValue::fromVec3(Math::Vec3(1.0f, 0.0f, 0.0f))));

    PropertyValue value;
    CHECK(NodeClassDB::getDefaultPropertyValue("CountingNode", "counting_value", value));
    CHECK(value == PropertyValue::fromInt(99));
    CHECK(NodeClassDB::getDefaultPropertyValue("CountingNode", "counting_ratio", value));
    CHECK(value == PropertyValue::fromFloat(0.5));
    CHECK(NodeClassDB::getDefaultPropertyValue("CountingLeaf", "counting_value", value));
    CHECK(value == PropertyValue::fromInt(7));
    CHECK(NodeClassDB::getDefaultPropertyValue("CountingLeaf", "leaf_flag", value));
    CHECK(value == PropertyValue::fromBool(false));
    CHECK(!NodeClassDB::getDefaultPropertyValue("CountingChild", "no_such_property", value));
}


class ValidatedNode : public Node
{
public:
    PRISMA3D_NODE_CLASS(ValidatedNode, Node);
    static void bindProperties()
    {
        NodeClassDB::addProperty<ValidatedNode>(PropertyInfo(PropertyType::Bool, "advanced"),
                &ValidatedNode::setAdvanced, &ValidatedNode::isAdvanced);
        NodeClassDB::addProperty<ValidatedNode>(PropertyInfo(PropertyType::Int, "detail"),
                &ValidatedNode::setDetail, &ValidatedNode::getDetail);
        NodeClassDB::addProperty<ValidatedNode>(PropertyInfo(PropertyType::Int, "removed"),
                &ValidatedNode::setDetail, &ValidatedNode::getDetail);
    }
    void setAdvanced(bool advanced)
    {
        mAdvanced = advanced;
        notifyPropertyListChanged();
    }
    bool isAdvanced() const
    {
        return mAdvanced;
    }
    void setDetail(int detail)
    {
        mDetail = detail;
    }
    int getDetail() const
    {
        return mDetail;
    }

protected:
    void validateProperty(PropertyInfo& info) const override
    {
        Node::validateProperty(info);
        if (info.name == "detail" && !mAdvanced) info.usage = PropertyUsage::NoEditor;
        if (info.name == "removed") info.usage = PropertyUsage::None;
    }

private:
    bool mAdvanced = false;
    int mDetail = 3;
};

std::uint32_t usageOf(const Node* node, const char* name)
{
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getInstancePropertyList(node, node->getClassName(), list);
    for (size_t i = 0; i < list.size(); ++i)
        if (list[i].name == name) return list[i].usage;
    return 0xffffffffu;
}

bool listed(const Node* node, const char* name)
{
    return usageOf(node, name) != 0xffffffffu;
}

bool editable(const Node* node, const char* name)
{
    const std::uint32_t usage = usageOf(node, name);
    return usage != 0xffffffffu && (usage & PropertyUsage::Editor) != 0;
}

bool stored(const Node* node, const char* name)
{
    const std::uint32_t usage = usageOf(node, name);
    return usage != 0xffffffffu && (usage & PropertyUsage::Storage) != 0;
}

void testValidateProperty()
{
    NodeClassDB::registerClass<ValidatedNode>();
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    ValidatedNode* custom = root->createChild<ValidatedNode>();
    CHECK(!editable(custom, "detail") && stored(custom, "detail"));
    CHECK(listed(custom, "removed") && !editable(custom, "removed") && !stored(custom, "removed"));
    const std::uint32_t revision = custom->getPropertyListRevision();
    CHECK(custom->set("advanced", PropertyValue::fromBool(true)));
    CHECK(custom->getPropertyListRevision() != revision);
    CHECK(editable(custom, "detail") && stored(custom, "detail"));
    const std::uint32_t after = custom->getPropertyListRevision();
    CHECK(custom->set("detail", PropertyValue::fromInt(5)));
    CHECK(custom->getPropertyListRevision() == after);

    ct::Vector<PropertyInfo> plain;
    NodeClassDB::getPropertyList("ValidatedNode", plain);
    for (size_t i = 0; i < plain.size(); ++i)
        if (plain[i].name == "removed" || plain[i].name == "detail")
            CHECK(plain[i].usage == PropertyUsage::Default);
    ct::Vector<PropertyInfo> withoutNode;
    NodeClassDB::getInstancePropertyList(nullptr, "ValidatedNode", withoutNode);
    CHECK(withoutNode.size() == plain.size());

    Camera3D* camera = root->createChild<Camera3D>();
    CHECK(editable(camera, "fov") && !editable(camera, "size"));
    const std::uint32_t cameraRevision = camera->getPropertyListRevision();
    camera->setFov(70.0f);
    CHECK(camera->getPropertyListRevision() == cameraRevision);
    CHECK(camera->set("projection", PropertyValue::fromInt(1)));
    CHECK(camera->getPropertyListRevision() != cameraRevision);
    CHECK(!editable(camera, "fov") && stored(camera, "fov"));
    CHECK(editable(camera, "size") && stored(camera, "size"));
    CHECK(editable(camera, "near") && editable(camera, "far") && editable(camera, "cull_mask"));
    camera->setPerspective(60.0f, 0.1f, 100.0f);
    CHECK(editable(camera, "fov") && !editable(camera, "size"));

    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    CHECK(!editable(sun, "directional_shadow_mode") && !editable(sun, "shadow_bias") &&
            editable(sun, "shadow_enabled") && stored(sun, "shadow_bias"));
    sun->setShadow(true);
    CHECK(editable(sun, "directional_shadow_mode") && editable(sun, "shadow_bias"));
    sun->setShadowMode(DirectionalShadowMode::Orthogonal);
    CHECK(!editable(sun, "directional_shadow_split_1") &&
            !editable(sun, "directional_shadow_blend_splits"));
    CHECK(!editable(sun, "directional_shadow_split_2") &&
            !editable(sun, "directional_shadow_split_3"));
    CHECK(editable(sun, "directional_shadow_fade_start") && editable(sun, "light_energy"));
    sun->setShadowMode(DirectionalShadowMode::Parallel2Splits);
    CHECK(editable(sun, "directional_shadow_split_1") &&
            editable(sun, "directional_shadow_blend_splits"));
    CHECK(!editable(sun, "directional_shadow_split_2") &&
            !editable(sun, "directional_shadow_split_3"));
    sun->setShadowMode(DirectionalShadowMode::Parallel4Splits);
    CHECK(editable(sun, "directional_shadow_split_2") &&
            editable(sun, "directional_shadow_split_3"));
    CHECK(stored(sun, "directional_shadow_split_3"));

    ReflectionProbe3D* probe = root->createChild<ReflectionProbe3D>();
    probe->setAmbientMode(ReflectionProbeAmbientMode::Disabled);
    CHECK(!editable(probe, "ambient_color") && !editable(probe, "ambient_color_energy"));
    CHECK(editable(probe, "ambient_mode") && stored(probe, "ambient_color"));
    probe->setAmbientMode(ReflectionProbeAmbientMode::Color);
    CHECK(editable(probe, "ambient_color") && editable(probe, "ambient_color_energy"));

    LightmapGI* gi = root->createChild<LightmapGI>();
    CHECK(!editable(gi, "supersampling_factor") && stored(gi, "supersampling_factor"));
    gi->setSupersamplingEnabled(true);
    CHECK(editable(gi, "supersampling_factor"));
    CHECK(!editable(gi, "specular_intensity") && stored(gi, "specular_intensity"));
    gi->setDirectional(true);
    CHECK(editable(gi, "specular_intensity") && editable(gi, "directional"));
    CHECK(editable(gi, "denoiser_strength") && editable(gi, "denoiser_range"));
    gi->setUseDenoiser(false);
    CHECK(!editable(gi, "denoiser_strength") && !editable(gi, "denoiser_range"));
    CHECK(!editable(gi, "environment_custom_color") && !editable(gi, "environment_custom_energy"));
    gi->setEnvironmentMode(LightmapGI::EnvironmentMode::CustomColor);
    CHECK(editable(gi, "environment_custom_color") && editable(gi, "environment_custom_energy"));
    CHECK(editable(gi, "light_data") && editable(gi, "quality") && editable(gi, "shadowmask_mode"));

    RigidBody3D* body = root->createChild<RigidBody3D>();
    CHECK(!editable(body, "max_contacts_reported") && stored(body, "max_contacts_reported"));
    CHECK(body->set("contact_monitor", PropertyValue::fromBool(true)));
    CHECK(editable(body, "max_contacts_reported"));
    CHECK(editable(body, "contact_monitor") && editable(body, "mass"));

    CollisionShape3D* shape = root->createChild<CollisionShape3D>();
    CHECK(!editable(shape, "box_size") && !editable(shape, "sphere_radius"));
    CHECK(shape->set("shape_type",
            PropertyValue::fromInt(static_cast<int>(CollisionShapeKind::Capsule))));
    CHECK(editable(shape, "capsule_radius") && editable(shape, "capsule_height"));
    CHECK(!editable(shape, "box_size") && !editable(shape, "cylinder_radius") &&
            !editable(shape, "plane_normal"));
    CHECK(stored(shape, "box_size"));
    CHECK(shape->set("shape_type",
            PropertyValue::fromInt(static_cast<int>(CollisionShapeKind::Plane))));
    CHECK(editable(shape, "plane_normal") && editable(shape, "plane_distance") &&
            !editable(shape, "capsule_height"));
    CHECK(editable(shape, "disabled") && editable(shape, "shape_type"));

    Node3D* spatial = root->createChild<Node3D>();
    CHECK(!editable(spatial, "collision_radii") && !editable(spatial, "collision_box_min") &&
            !editable(spatial, "collision_mesh") && !editable(spatial, "collision_shape"));
    CHECK(editable(spatial, "collision_enabled") && !editable(spatial, "collision_type") &&
            stored(spatial, "collision_shape"));
    spatial->setCollisionEnabled(true);
    CHECK(editable(spatial, "collision_type") && spatial->getCollisionType() == 1);
    spatial->setCollisionType(1);
    CHECK(editable(spatial, "collision_shape") && editable(spatial, "collision_type"));
    spatial->setCollisionRadius(2.0f);
    CHECK(editable(spatial, "collision_radii") && !editable(spatial, "collision_box_max"));
    spatial->setCollisionBox(Math::Vec3(-1.0f, -1.0f, -1.0f), Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(!editable(spatial, "collision_radii") && editable(spatial, "collision_box_min") &&
            editable(spatial, "collision_box_max"));
    CHECK(stored(spatial, "collision_radii"));

    ParticleSystem3D* particles = root->createChild<ParticleSystem3D>();
    particles->setContinuous(10.0f);
    CHECK(editable(particles, "emission_rate") && !editable(particles, "burst_count") &&
            !editable(particles, "burst_interval") && !editable(particles, "one_shot_count") &&
            !editable(particles, "pulse_rate") && !editable(particles, "particles_per_pulse"));
    particles->setBurst(5, 0.5f);
    CHECK(!editable(particles, "emission_rate") && editable(particles, "burst_count") &&
            editable(particles, "burst_interval"));
    particles->setOneShot(8);
    CHECK(editable(particles, "one_shot_count") && !editable(particles, "burst_count"));
    particles->setPulse(2.0f, 4);
    CHECK(editable(particles, "pulse_rate") && editable(particles, "particles_per_pulse"));
    particles->setShapePoint();
    CHECK(!editable(particles, "shape_radius") && !editable(particles, "shape_box_size") &&
            !editable(particles, "shape_cone_angle") && !editable(particles, "shape_inner_radius"));
    particles->setShapeSphere(2.0f);
    CHECK(editable(particles, "shape_radius") && !editable(particles, "shape_box_size"));
    particles->setShapeBox(Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(!editable(particles, "shape_radius") && editable(particles, "shape_box_size"));
    particles->setShapeCone(30.0f, 1.0f);
    CHECK(editable(particles, "shape_radius") && editable(particles, "shape_cone_angle") &&
            !editable(particles, "shape_inner_radius"));
    particles->setShapeRing(2.0f, 1.0f);
    CHECK(editable(particles, "shape_radius") && editable(particles, "shape_inner_radius") &&
            !editable(particles, "shape_cone_angle"));
    particles->setShapeCircle(1.0f);
    CHECK(editable(particles, "shape_radius") && !editable(particles, "shape_inner_radius"));
    particles->clearAtlas();
    CHECK(editable(particles, "atlas_enabled") && !editable(particles, "atlas_columns") &&
            !editable(particles, "atlas_rows") && !editable(particles, "atlas_anim_loop"));
    particles->setAtlasGrid(4, 4);
    CHECK(editable(particles, "atlas_columns") && editable(particles, "atlas_anim_loop"));

    AnimationPlayer* player = root->createChild<AnimationPlayer>();
    PropertyInfo autoplay;
    CHECK(NodeClassDB::getPropertyInfo("AnimationPlayer", "autoplay", autoplay));
    CHECK(autoplay.hint == PropertyHint::None);
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getInstancePropertyList(player, "AnimationPlayer", list);
    for (size_t i = 0; i < list.size(); ++i)
        if (list[i].name == "autoplay") CHECK(list[i].hint == PropertyHint::None);
    AnimationClip walk;
    AnimationClip run;
    const std::uint32_t playerRevision = player->getPropertyListRevision();
    CHECK(player->addAnimation("walk", &walk) && player->addAnimation("run", &run));
    CHECK(player->getPropertyListRevision() != playerRevision);
    list.clear();
    NodeClassDB::getInstancePropertyList(player, "AnimationPlayer", list);
    bool found = false;
    for (size_t i = 0; i < list.size(); ++i)
    {
        if (list[i].name != "autoplay") continue;
        found = true;
        CHECK(list[i].hint == PropertyHint::Enum && list[i].hintString == ",walk,run");
        CHECK(list[i].enumCount() == 3 && list[i].enumName(0).empty() &&
                list[i].enumName(2) == "run");
    }
    CHECK(found);
    CHECK(player->removeAnimation("walk"));
    list.clear();
    NodeClassDB::getInstancePropertyList(player, "AnimationPlayer", list);
    for (size_t i = 0; i < list.size(); ++i)
        if (list[i].name == "autoplay") CHECK(list[i].hintString == ",run");

    Skeleton3D* skeleton = root->createChild<Skeleton3D>();
    Skeleton bones;
    bones.addBone("hips", -1, Math::Mat4::Identity(), Math::Mat4::Identity());
    bones.addBone("spine", 0, Math::Mat4::Identity(), Math::Mat4::Identity());
    bones.finalize();
    skeleton->setSkeleton(bones);
    BoneAttachment3D* attachment = skeleton->createChild<BoneAttachment3D>();
    list.clear();
    NodeClassDB::getInstancePropertyList(attachment, "BoneAttachment3D", list);
    found = false;
    for (size_t i = 0; i < list.size(); ++i)
    {
        if (list[i].name == "bone_name")
        {
            found = true;
            CHECK(list[i].hint == PropertyHint::Enum && list[i].hintString == "hips,spine");
        }
    }
    CHECK(found);
    CHECK(listed(attachment, "external_skeleton") && !editable(attachment, "external_skeleton"));
    const std::uint32_t attachmentRevision = attachment->getPropertyListRevision();
    attachment->setUseExternalSkeleton(true);
    CHECK(attachment->getPropertyListRevision() != attachmentRevision);
    CHECK(editable(attachment, "external_skeleton"));
    list.clear();
    NodeClassDB::getInstancePropertyList(attachment, "BoneAttachment3D", list);
    for (size_t i = 0; i < list.size(); ++i)
        if (list[i].name == "bone_name") CHECK(list[i].hint == PropertyHint::None);
    attachment->setExternalSkeleton("../..");
    attachment->setUseExternalSkeleton(false);
    BoneAttachment3D* loose = root->createChild<BoneAttachment3D>();
    list.clear();
    NodeClassDB::getInstancePropertyList(loose, "BoneAttachment3D", list);
    for (size_t i = 0; i < list.size(); ++i)
        if (list[i].name == "bone_name") CHECK(list[i].hint == PropertyHint::None);

    MeshInstance3D* mesh = root->createChild<MeshInstance3D>();
    CHECK(!editable(mesh, "skeleton") && stored(mesh, "skeleton"));
    CHECK(editable(mesh, "mesh") && editable(mesh, "material_override"));

    WorldEnvironment* world = root->createChild<WorldEnvironment>();
    CHECK(!editable(world, "glow_intensity") && !editable(world, "fog_density") &&
            editable(world, "glow_enabled") && editable(world, "fog_enabled"));
    CHECK(stored(world, "glow_intensity") && stored(world, "fog_density"));
    CHECK(world->set("glow_enabled", PropertyValue::fromBool(true)));
    CHECK(world->set("fog_enabled", PropertyValue::fromBool(true)));
    CHECK(editable(world, "glow_intensity") && editable(world, "fog_density"));
    CHECK(!editable(world, "sky_top_color") && !editable(world, "sky_mode") &&
            !editable(world, "sky_texture") && !editable(world, "ambient_light_sky_contribution"));
    CHECK(!editable(world, "background_color") && editable(world, "ambient_light_color"));
    CHECK(!editable(world, "fog_depth_begin") && !editable(world, "fog_depth_end") &&
            !editable(world, "fog_depth_curve") && !editable(world, "fog_aerial_perspective"));
    CHECK(!editable(world, "tonemap_agx_contrast") && !editable(world, "glow_mix"));
    CHECK(editable(world, "glow_intensity") && !editable(world, "tonemap_white"));
    const std::uint32_t worldRevision = world->getPropertyListRevision();
    CHECK(world->set("background_mode", PropertyValue::fromInt(2)));
    CHECK(world->getPropertyListRevision() != worldRevision);
    CHECK(editable(world, "sky_mode") && editable(world, "sky_top_color") &&
            editable(world, "sky_ground_curve") && !editable(world, "sky_texture") &&
            !editable(world, "sky_texture_energy_multiplier"));
    CHECK(editable(world, "sky_use_debanding") && editable(world, "sky_radiance_size"));
    CHECK(editable(world, "fog_aerial_perspective") &&
            editable(world, "ambient_light_sky_contribution"));
    CHECK(world->set("sky_mode", PropertyValue::fromInt(1)));
    CHECK(!editable(world, "sky_top_color") && !editable(world, "sky_energy_multiplier") &&
            !editable(world, "sky_overall_energy_multiplier") && editable(world, "sky_texture") &&
            editable(world, "sky_texture_energy_multiplier"));
    CHECK(world->set("background_mode", PropertyValue::fromInt(1)));
    CHECK(editable(world, "background_color") && !editable(world, "sky_mode"));
    CHECK(world->set("ambient_light_source", PropertyValue::fromInt(2)));
    CHECK(!editable(world, "ambient_light_color") && !editable(world, "ambient_light_energy"));
    CHECK(world->set("ambient_light_source", PropertyValue::fromInt(1)));
    CHECK(editable(world, "ambient_light_color") && editable(world, "background_color"));
    CHECK(world->set("ambient_light_source", PropertyValue::fromInt(3)));
    CHECK(editable(world, "sky_mode") && editable(world, "ambient_light_sky_contribution"));
    CHECK(world->set("fog_mode", PropertyValue::fromInt(1)));
    CHECK(editable(world, "fog_depth_begin") && editable(world, "fog_depth_end") &&
            editable(world, "fog_depth_curve"));
    CHECK(world->set("tonemap_mode", PropertyValue::fromInt(4)));
    CHECK(editable(world, "tonemap_agx_contrast") && !editable(world, "tonemap_white"));
    CHECK(world->set("tonemap_mode", PropertyValue::fromInt(2)));
    CHECK(!editable(world, "tonemap_agx_contrast") && editable(world, "tonemap_white"));
    CHECK(world->set("glow_blend_mode", PropertyValue::fromInt(4)));
    CHECK(editable(world, "glow_mix") && !editable(world, "glow_intensity"));
    CHECK(stored(world, "glow_intensity") && stored(world, "sky_top_color"));
}

} // namespace

int main()
{
    testPropertyValue();
    testRegisteredProperties(true);
    testRegisteredProperties(false);
    testLookups();
    testGroups();
    testDefaults();
    testPhysicsWithoutServer();
    testPhysicsServerAppearsLater();
    testPhysicsServerVanishes();
    testDistinctSettersKeepOtherState();
    testCustomClasses();
    testValidateProperty();

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("node properties ok\n");
    return 0;
}
