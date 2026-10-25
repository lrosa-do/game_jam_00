#include "Check.h"

#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

struct FloatField
{
    const char* name;
    float EnvironmentSettings::*member;
    float defaultValue;
    float changed;
};

const FloatField kFloats[] = {
    { "ssao_radius", &EnvironmentSettings::ssaoRadius, 1.0f, 2.5f },
    { "ssao_intensity", &EnvironmentSettings::ssaoIntensity, 2.0f, 3.5f },
    { "ssao_power", &EnvironmentSettings::ssaoPower, 1.5f, 2.25f },
    { "ssao_detail", &EnvironmentSettings::ssaoDetail, 0.5f, 1.75f },
    { "ssao_horizon", &EnvironmentSettings::ssaoHorizon, 0.06f, 0.25f },
    { "ssao_sharpness", &EnvironmentSettings::ssaoSharpness, 0.98f, 0.5f },
    { "ssao_light_affect", &EnvironmentSettings::ssaoLightAffect, 0.0f, 0.75f },
    { "ssao_ao_channel_affect", &EnvironmentSettings::ssaoAoChannelAffect, 0.0f, 0.5f },
    { "ssr_fade_in", &EnvironmentSettings::ssrFadeIn, 0.15f, 0.75f },
    { "ssr_fade_out", &EnvironmentSettings::ssrFadeOut, 2.0f, 3.5f },
    { "ssr_depth_tolerance", &EnvironmentSettings::ssrDepthTolerance, 0.5f, 4.0f },
    { "adjustment_brightness", &EnvironmentSettings::adjustmentBrightness, 1.0f, 1.5f },
    { "adjustment_contrast", &EnvironmentSettings::adjustmentContrast, 1.0f, 1.125f },
    { "adjustment_saturation", &EnvironmentSettings::adjustmentSaturation, 1.0f, 0.25f },
};

const PropertyInfo* findInfo(const ct::Vector<PropertyInfo>& list, const char* name)
{
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i].name == name) return &list[i];
    return nullptr;
}

int indexOf(const ct::Vector<PropertyInfo>& list, const char* name)
{
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i].name == name) return static_cast<int>(i);
    return -1;
}

void testRegistry()
{
    printf("the SSR, SSAO and Adjustments groups are registered like the ones of the "
           "Environment\n");
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList("WorldEnvironment", list);

    const int ssrGroup = indexOf(list, "SSR");
    CHECK(ssrGroup >= 0);
    if (ssrGroup >= 0)
    {
        CHECK(list[static_cast<std::size_t>(ssrGroup)].isGroup());
        CHECK(list[static_cast<std::size_t>(ssrGroup)].hintString == "ssr_");
    }
    const char* ssr[] = { "ssr_enabled", "ssr_max_steps", "ssr_fade_in", "ssr_fade_out",
        "ssr_depth_tolerance" };
    int previousSsr = ssrGroup;
    for (const char* name: ssr)
    {
        const int at = indexOf(list, name);
        CHECK(at > previousSsr);
        previousSsr = at;
    }
    CHECK(indexOf(list, "SSAO") > previousSsr);
    const PropertyInfo* ssrEnabled = findInfo(list, "ssr_enabled");
    CHECK(ssrEnabled != nullptr && ssrEnabled->type == PropertyType::Bool &&
            ssrEnabled->hint == PropertyHint::GroupEnable &&
            (ssrEnabled->usage & PropertyUsage::Editor) != 0 &&
            (ssrEnabled->usage & PropertyUsage::Storage) != 0);
    const PropertyInfo* steps = findInfo(list, "ssr_max_steps");
    CHECK(steps != nullptr && steps->type == PropertyType::Int &&
            steps->hint == PropertyHint::Range && steps->rangeMin == 32.0 &&
            steps->rangeMax == 512.0);
    const PropertyInfo* tolerance = findInfo(list, "ssr_depth_tolerance");
    CHECK(tolerance != nullptr && tolerance->hint == PropertyHint::Range &&
            tolerance->rangeMin == 0.01 && tolerance->rangeMax == 128.0 &&
            !tolerance->rangeOrGreater);
    const PropertyInfo* fadeIn = findInfo(list, "ssr_fade_in");
    CHECK(fadeIn != nullptr && fadeIn->rangeMin == 0.0 && fadeIn->rangeOrGreater);

    const int ssaoGroup = indexOf(list, "SSAO");
    const int adjustmentGroup = indexOf(list, "Adjustments");
    CHECK(ssaoGroup >= 0 && adjustmentGroup >= 0);
    if (ssaoGroup >= 0)
    {
        CHECK(list[static_cast<std::size_t>(ssaoGroup)].isGroup());
        CHECK(list[static_cast<std::size_t>(ssaoGroup)].hintString == "ssao_");
    }
    if (adjustmentGroup >= 0)
    {
        CHECK(list[static_cast<std::size_t>(adjustmentGroup)].isGroup());
        CHECK(list[static_cast<std::size_t>(adjustmentGroup)].hintString == "adjustment_");
    }

    const char* ssao[] = { "ssao_enabled", "ssao_radius", "ssao_intensity", "ssao_power",
        "ssao_detail", "ssao_horizon", "ssao_sharpness", "ssao_light_affect",
        "ssao_ao_channel_affect" };
    int previous = ssaoGroup;
    for (const char* name: ssao)
    {
        const int at = indexOf(list, name);
        CHECK(at > previous);
        previous = at;
    }
    const char* adjustment[] = { "adjustment_enabled", "adjustment_brightness",
        "adjustment_contrast", "adjustment_saturation", "adjustment_color_correction" };
    previous = adjustmentGroup;
    for (const char* name: adjustment)
    {
        const int at = indexOf(list, name);
        CHECK(at > previous);
        previous = at;
    }

    const PropertyInfo* ssaoEnabled = findInfo(list, "ssao_enabled");
    const PropertyInfo* adjustmentEnabled = findInfo(list, "adjustment_enabled");
    CHECK(ssaoEnabled != nullptr && adjustmentEnabled != nullptr);
    if (ssaoEnabled)
    {
        CHECK(ssaoEnabled->type == PropertyType::Bool);
        CHECK(ssaoEnabled->hint == PropertyHint::GroupEnable);
        CHECK((ssaoEnabled->usage & PropertyUsage::Editor) != 0);
        CHECK((ssaoEnabled->usage & PropertyUsage::Storage) != 0);
    }
    if (adjustmentEnabled)
    {
        CHECK(adjustmentEnabled->type == PropertyType::Bool);
        CHECK(adjustmentEnabled->hint == PropertyHint::GroupEnable);
        CHECK((adjustmentEnabled->usage & PropertyUsage::Storage) != 0);
    }
    const PropertyInfo* radius = findInfo(list, "ssao_radius");
    CHECK(radius != nullptr && radius->hint == PropertyHint::Range && radius->rangeMin == 0.01 &&
            radius->rangeMax == 16.0 && radius->rangeOrGreater);
    const PropertyInfo* contrast = findInfo(list, "adjustment_contrast");
    CHECK(contrast != nullptr && contrast->rangeMin == 0.75 && contrast->rangeMax == 1.25 &&
            contrast->rangeOrGreater && contrast->rangeOrLess);
    const PropertyInfo* correction = findInfo(list, "adjustment_color_correction");
    CHECK(correction != nullptr && correction->type == PropertyType::ResourcePath);
}

void testDefaults()
{
    printf("everything is off by default and the defaults are the ones of the original\n");
    Node root;
    WorldEnvironment* world = root.createChild<WorldEnvironment>();
    const EnvironmentSettings& env = world->getEnvironment();
    CHECK(!env.ssaoEnabled && !env.adjustmentEnabled && !env.ssrEnabled);
    CHECK(env.ssrMaxSteps == 64);
    for (const FloatField& field: kFloats)
    {
        CHECK(env.*(field.member) == field.defaultValue);
        bool valid = false;
        const PropertyValue value = world->get(field.name, &valid);
        CHECK(valid);
        CHECK(fabs(value.asFloat() - static_cast<double>(field.defaultValue)) < 1e-6);
        PropertyValue stored;
        CHECK(NodeClassDB::getDefaultPropertyValue("WorldEnvironment", field.name, stored));
        CHECK(fabs(stored.asFloat() - static_cast<double>(field.defaultValue)) < 1e-6);
    }
    bool valid = false;
    CHECK(!world->get("ssao_enabled", &valid).asBool() && valid);
    CHECK(!world->get("ssr_enabled", &valid).asBool() && valid);
    CHECK(world->get("ssr_max_steps", &valid).asInt() == 64 && valid);
    PropertyValue storedSteps;
    CHECK(NodeClassDB::getDefaultPropertyValue("WorldEnvironment", "ssr_max_steps", storedSteps));
    CHECK(storedSteps.asInt() == 64);
    CHECK(!world->get("adjustment_enabled", &valid).asBool() && valid);
    world->get("adjustment_color_correction", &valid);
    CHECK(valid);
    CHECK(!env.adjustmentColorCorrection.valid());
}

void testSetters()
{
    printf("each property reaches the environment and comes back\n");
    Node root;
    WorldEnvironment* world = root.createChild<WorldEnvironment>();
    for (const FloatField& field: kFloats)
    {
        CHECK(world->set(field.name, PropertyValue::fromSingle(field.changed)));
        CHECK(world->getEnvironment().*(field.member) == field.changed);
        bool valid = false;
        const PropertyValue value = world->get(field.name, &valid);
        CHECK(valid);
        CHECK(fabs(value.asFloat() - static_cast<double>(field.changed)) < 1e-6);
    }
    CHECK(world->set("ssr_max_steps", PropertyValue::fromInt(200)));
    CHECK(world->getEnvironment().ssrMaxSteps == 200);
    bool steps = false;
    CHECK(world->get("ssr_max_steps", &steps).asInt() == 200 && steps);
    CHECK(world->set("ssr_enabled", PropertyValue::fromBool(true)));
    CHECK(world->getEnvironment().ssrEnabled);
    CHECK(world->set("ssao_enabled", PropertyValue::fromBool(true)));
    CHECK(world->set("adjustment_enabled", PropertyValue::fromBool(true)));
    CHECK(world->getEnvironment().ssaoEnabled && world->getEnvironment().adjustmentEnabled);
    CHECK(world->set("ssao_enabled", PropertyValue::fromBool(false)));
    CHECK(!world->getEnvironment().ssaoEnabled && world->getEnvironment().adjustmentEnabled);
}

void testRoundTrip()
{
    printf("the new properties survive saving and loading, and defaults are not written\n");
    {
        Node root;
        root.createChild<WorldEnvironment>();
        ct::String text;
        CHECK(SceneSerializer::save(root.getChild(0), text));
        CHECK(strstr(text.c_str(), "ssao_") == nullptr);
        CHECK(strstr(text.c_str(), "ssr_") == nullptr);
        CHECK(strstr(text.c_str(), "adjustment_") == nullptr);
    }

    Node root;
    WorldEnvironment* world = root.createChild<WorldEnvironment>();
    world->set("ssao_enabled", PropertyValue::fromBool(true));
    world->set("ssr_enabled", PropertyValue::fromBool(true));
    world->set("ssr_max_steps", PropertyValue::fromInt(300));
    world->set("adjustment_enabled", PropertyValue::fromBool(true));
    for (const FloatField& field: kFloats)
        world->set(field.name, PropertyValue::fromSingle(field.changed));

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(world, text, &saved));
    CHECK(strstr(text.c_str(), "ssao_enabled") != nullptr);
    CHECK(strstr(text.c_str(), "adjustment_enabled") != nullptr);
    CHECK(strstr(text.c_str(), "ssr_enabled") != nullptr);
    CHECK(strstr(text.c_str(), "ssr_max_steps") != nullptr);
    for (const FloatField& field: kFloats) CHECK(strstr(text.c_str(), field.name) != nullptr);

    Node target;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &target, loaded);
    CHECK(copy != nullptr);
    if (!copy) return;
    WorldEnvironment* other = copy->castTo<WorldEnvironment>();
    CHECK(other != nullptr);
    if (!other) return;
    const EnvironmentSettings& a = world->getEnvironment();
    const EnvironmentSettings& b = other->getEnvironment();
    CHECK(b.ssaoEnabled && b.adjustmentEnabled && b.ssrEnabled && b.ssrMaxSteps == 300);
    for (const FloatField& field: kFloats) CHECK(a.*(field.member) == b.*(field.member));

    ct::String again;
    CHECK(SceneSerializer::save(other, again));
    CHECK(again == text);
}


struct AttributesField
{
    const char* name;
    float CameraAttributesSettings::*member;
    float defaultValue;
    float changed;
};

const AttributesField kAttributes[] = {
    { "dof_blur_far_distance", &CameraAttributesSettings::dofBlurFarDistance, 10.0f, 25.0f },
    { "dof_blur_far_transition", &CameraAttributesSettings::dofBlurFarTransition, 5.0f, -1.0f },
    { "dof_blur_near_distance", &CameraAttributesSettings::dofBlurNearDistance, 2.0f, 6.5f },
    { "dof_blur_near_transition", &CameraAttributesSettings::dofBlurNearTransition, 1.0f, 3.0f },
    { "dof_blur_amount", &CameraAttributesSettings::dofBlurAmount, 0.1f, 0.4f },
};

void testDepthOfField()
{
    printf("depth of field: Far and Near groups with their own switch, the defaults of the "
           "original, no change unless asked\n");
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList("WorldEnvironment", list);
    const int dofGroup = indexOf(list, "Depth of Field");
    const int farGroup = indexOf(list, "Depth of Field Far");
    const int nearGroup = indexOf(list, "Depth of Field Near");
    CHECK(dofGroup > indexOf(list, "Adjustments"));
    CHECK(farGroup > dofGroup && nearGroup > farGroup);
    if (dofGroup >= 0) CHECK(list[static_cast<std::size_t>(dofGroup)].hintString == "dof_blur_");
    if (farGroup >= 0)
        CHECK(list[static_cast<std::size_t>(farGroup)].hintString == "dof_blur_far_");
    if (nearGroup >= 0)
        CHECK(list[static_cast<std::size_t>(nearGroup)].hintString == "dof_blur_near_");
    CHECK(indexOf(list, "dof_blur_amount") > dofGroup &&
            indexOf(list, "dof_blur_amount") < farGroup);
    const char* far[] = { "dof_blur_far_enabled", "dof_blur_far_distance",
        "dof_blur_far_transition" };
    int previous = farGroup;
    for (const char* name: far)
    {
        const int at = indexOf(list, name);
        CHECK(at > previous);
        previous = at;
    }
    const char* near[] = { "dof_blur_near_enabled", "dof_blur_near_distance",
        "dof_blur_near_transition" };
    previous = nearGroup;
    for (const char* name: near)
    {
        const int at = indexOf(list, name);
        CHECK(at > previous);
        previous = at;
    }
    const char* gates[] = { "dof_blur_far_enabled", "dof_blur_near_enabled" };
    for (const char* name: gates)
    {
        const PropertyInfo* gate = findInfo(list, name);
        CHECK(gate != nullptr && gate->type == PropertyType::Bool &&
                gate->hint == PropertyHint::GroupEnable &&
                (gate->usage & PropertyUsage::Editor) != 0 &&
                (gate->usage & PropertyUsage::Storage) != 0);
    }
    const PropertyInfo* distance = findInfo(list, "dof_blur_far_distance");
    CHECK(distance != nullptr && distance->hint == PropertyHint::Range &&
            distance->rangeMin == 0.01 && distance->rangeMax == 8192.0);
    const PropertyInfo* transition = findInfo(list, "dof_blur_near_transition");
    CHECK(transition != nullptr && transition->rangeMin == -1.0 && transition->rangeMax == 8192.0);
    const PropertyInfo* amount = findInfo(list, "dof_blur_amount");
    CHECK(amount != nullptr && amount->rangeMin == 0.0 && amount->rangeMax == 1.0);

    Node root;
    WorldEnvironment* world = root.createChild<WorldEnvironment>();
    const CameraAttributesSettings& attributes = world->getEnvironment().cameraAttributes;
    CHECK(!attributes.dofBlurFarEnabled && !attributes.dofBlurNearEnabled && !attributes.usesDof());
    for (const AttributesField& field: kAttributes)
    {
        CHECK(attributes.*(field.member) == field.defaultValue);
        bool valid = false;
        const PropertyValue value = world->get(field.name, &valid);
        CHECK(valid && fabs(value.asFloat() - static_cast<double>(field.defaultValue)) < 1e-6);
        PropertyValue stored;
        CHECK(NodeClassDB::getDefaultPropertyValue("WorldEnvironment", field.name, stored));
        CHECK(fabs(stored.asFloat() - static_cast<double>(field.defaultValue)) < 1e-6);
    }
    bool valid = false;
    CHECK(!world->get("dof_blur_far_enabled", &valid).asBool() && valid);
    CHECK(!world->get("dof_blur_near_enabled", &valid).asBool() && valid);

    ct::String untouched;
    CHECK(SceneSerializer::save(world, untouched));
    CHECK(strstr(untouched.c_str(), "dof_blur_") == nullptr);

    CHECK(world->set("dof_blur_far_enabled", PropertyValue::fromBool(true)));
    CHECK(world->getEnvironment().cameraAttributes.dofBlurFarEnabled);
    CHECK(world->getEnvironment().cameraAttributes.usesDof());
    CHECK(world->set("dof_blur_amount", PropertyValue::fromSingle(0.0f)));
    CHECK(!world->getEnvironment().cameraAttributes.usesDof());
    CHECK(world->set("dof_blur_amount", PropertyValue::fromSingle(0.1f)));
    CHECK(world->set("dof_blur_far_enabled", PropertyValue::fromBool(false)));
    CHECK(world->set("dof_blur_near_enabled", PropertyValue::fromBool(true)));
    CHECK(world->getEnvironment().cameraAttributes.usesDof());
    CHECK(!world->getEnvironment().cameraAttributes.dofBlurFarEnabled);
    for (const AttributesField& field: kAttributes)
    {
        CHECK(world->set(field.name, PropertyValue::fromSingle(field.changed)));
        CHECK(world->getEnvironment().cameraAttributes.*(field.member) == field.changed);
        const PropertyValue value = world->get(field.name, &valid);
        CHECK(valid && fabs(value.asFloat() - static_cast<double>(field.changed)) < 1e-6);
    }
    CHECK(world->set("dof_blur_far_enabled", PropertyValue::fromBool(true)));

    ct::String text;
    CHECK(SceneSerializer::save(world, text));
    CHECK(strstr(text.c_str(), "dof_blur_far_enabled") != nullptr);
    CHECK(strstr(text.c_str(), "dof_blur_near_enabled") != nullptr);
    for (const AttributesField& field: kAttributes)
        CHECK(strstr(text.c_str(), field.name) != nullptr);
    Node target;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &target, loaded);
    CHECK(copy != nullptr);
    if (copy)
    {
        WorldEnvironment* other = copy->castTo<WorldEnvironment>();
        CHECK(other != nullptr);
        if (other)
        {
            const CameraAttributesSettings& a = world->getEnvironment().cameraAttributes;
            const CameraAttributesSettings& b = other->getEnvironment().cameraAttributes;
            CHECK(b.dofBlurFarEnabled && b.dofBlurNearEnabled);
            for (const AttributesField& field: kAttributes)
                CHECK(a.*(field.member) == b.*(field.member));
            ct::String again;
            CHECK(SceneSerializer::save(other, again));
            CHECK(again == text);
        }
    }
}

void testCameraAttributes()
{
    printf("the camera can replace the attributes of the environment, saved and loaded\n");
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList("Camera3D", list);
    const int group = indexOf(list, "Attributes");
    const int farGroup = indexOf(list, "Attributes DOF Far");
    const int nearGroup = indexOf(list, "Attributes DOF Near");
    CHECK(group > indexOf(list, "cull_mask"));
    CHECK(farGroup > group && nearGroup > farGroup);
    const char* gates[] = { "attributes_override", "attributes_dof_blur_far_enabled",
        "attributes_dof_blur_near_enabled" };
    for (const char* name: gates)
    {
        const PropertyInfo* gate = findInfo(list, name);
        CHECK(gate != nullptr && gate->type == PropertyType::Bool &&
                gate->hint == PropertyHint::GroupEnable &&
                (gate->usage & PropertyUsage::Storage) != 0);
    }

    Node root;
    Camera3D* camera = root.createChild<Camera3D>();
    CHECK(!camera->isAttributesOverride());
    CHECK(!camera->getAttributes().usesDof());
    bool valid = false;
    CHECK(!camera->get("attributes_override", &valid).asBool() && valid);
    ct::Vector<PropertyInfo> shown;
    NodeClassDB::getInstancePropertyList(camera, "Camera3D", shown, true);
    const PropertyInfo* hidden = findInfo(shown, "attributes_dof_blur_far_distance");
    CHECK(hidden != nullptr && (hidden->usage & PropertyUsage::Editor) == 0);
    CHECK(findInfo(shown, "attributes_override") != nullptr &&
            (findInfo(shown, "attributes_override")->usage & PropertyUsage::Editor) != 0);

    ct::String untouched;
    CHECK(SceneSerializer::save(camera, untouched));
    CHECK(strstr(untouched.c_str(), "attributes_") == nullptr);

    CHECK(camera->set("attributes_override", PropertyValue::fromBool(true)));
    CHECK(camera->isAttributesOverride());
    shown.clear();
    NodeClassDB::getInstancePropertyList(camera, "Camera3D", shown, true);
    const PropertyInfo* visible = findInfo(shown, "attributes_dof_blur_far_distance");
    CHECK(visible != nullptr && (visible->usage & PropertyUsage::Editor) != 0);
    CHECK(camera->set("attributes_dof_blur_far_enabled", PropertyValue::fromBool(true)));
    CHECK(camera->set("attributes_dof_blur_far_distance", PropertyValue::fromSingle(14.0f)));
    CHECK(camera->set("attributes_dof_blur_far_transition", PropertyValue::fromSingle(-1.0f)));
    CHECK(camera->set("attributes_dof_blur_near_enabled", PropertyValue::fromBool(true)));
    CHECK(camera->set("attributes_dof_blur_near_distance", PropertyValue::fromSingle(1.5f)));
    CHECK(camera->set("attributes_dof_blur_near_transition", PropertyValue::fromSingle(0.5f)));
    CHECK(camera->set("attributes_dof_blur_amount", PropertyValue::fromSingle(0.25f)));
    const CameraAttributesSettings& a = camera->getAttributes();
    CHECK(a.dofBlurFarEnabled && a.dofBlurNearEnabled && a.usesDof());
    CHECK(a.dofBlurFarDistance == 14.0f && a.dofBlurFarTransition == -1.0f);
    CHECK(a.dofBlurNearDistance == 1.5f && a.dofBlurNearTransition == 0.5f);
    CHECK(a.dofBlurAmount == 0.25f);

    ct::String text;
    CHECK(SceneSerializer::save(camera, text));
    CHECK(strstr(text.c_str(), "attributes_override") != nullptr);
    Node target;
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, &target, loaded);
    CHECK(copy != nullptr);
    if (copy)
    {
        Camera3D* other = copy->castTo<Camera3D>();
        CHECK(other != nullptr);
        if (other)
        {
            CHECK(other->isAttributesOverride());
            const CameraAttributesSettings& b = other->getAttributes();
            CHECK(b.dofBlurFarEnabled && b.dofBlurNearEnabled);
            CHECK(b.dofBlurFarDistance == 14.0f && b.dofBlurFarTransition == -1.0f);
            CHECK(b.dofBlurNearDistance == 1.5f && b.dofBlurNearTransition == 0.5f);
            CHECK(b.dofBlurAmount == 0.25f);
            ct::String again;
            CHECK(SceneSerializer::save(other, again));
            CHECK(again == text);
        }
    }
}

} // namespace

int main()
{
    testRegistry();
    testDefaults();
    testSetters();
    testRoundTrip();
    testDepthOfField();
    testCameraAttributes();

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("environment effects ok\n");
    return 0;
}
