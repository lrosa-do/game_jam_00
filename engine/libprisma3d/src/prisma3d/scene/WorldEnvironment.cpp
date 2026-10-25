#include "prisma3d/scene/WorldEnvironment.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

namespace prisma3d
{

WorldEnvironment::WorldEnvironment()
        : Node(),
          mEnvironment(),
          mScenario()
{
}

WorldEnvironment::~WorldEnvironment()
{
}

void WorldEnvironment::setEnvironment(const EnvironmentSettings& environment)
{
    const bool changed = mEnvironment.background != environment.background ||
                         mEnvironment.ambientSource != environment.ambientSource ||
                         mEnvironment.reflectionSource != environment.reflectionSource ||
                         mEnvironment.skyMode != environment.skyMode ||
                         mEnvironment.fogMode != environment.fogMode ||
                         mEnvironment.tonemapMode != environment.tonemapMode ||
                         mEnvironment.fogEnabled != environment.fogEnabled ||
                         mEnvironment.glowEnabled != environment.glowEnabled ||
                         mEnvironment.glowBlendMode != environment.glowBlendMode;
    mEnvironment = environment;
    if (mScenario.valid()) updateCurrentEnvironment();
    if (changed) notifyPropertyListChanged();
}

void WorldEnvironment::validateProperty(PropertyInfo& info) const
{
    Node::validateProperty(info);
    const EnvironmentSettings& environment = mEnvironment;

    if (!environment.fogEnabled && info.name.starts_with("fog_") && info.name != "fog_enabled")
    {
        info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (!environment.glowEnabled && info.name.starts_with("glow_") && info.name != "glow_enabled")
    {
        info.usage = PropertyUsage::NoEditor;
        return;
    }

    if (info.name.starts_with("sky_"))
    {
        const bool skyUsed = environment.background == BackgroundMode::Sky ||
                             environment.ambientSource == AmbientSource::Sky ||
                             environment.reflectionSource == ReflectionSource::Sky;
        if (!skyUsed)
        {
            info.usage = PropertyUsage::NoEditor;
        }
        else if (info.name != "sky_mode" && info.name != "sky_use_debanding" &&
                 info.name != "sky_radiance_size")
        {
            const bool textured =
                    info.name == "sky_texture" || info.name == "sky_texture_energy_multiplier";
            if (textured == (environment.skyMode == SkyMode::Procedural))
                info.usage = PropertyUsage::NoEditor;
        }
        return;
    }
    if (info.name == "ambient_light_sky_contribution")
    {
        if (environment.background != BackgroundMode::Sky &&
                environment.ambientSource != AmbientSource::Sky &&
                environment.reflectionSource != ReflectionSource::Sky)
            info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "fog_depth_curve" || info.name == "fog_depth_begin" ||
            info.name == "fog_depth_end")
    {
        if (environment.fogMode == FogMode::Exponential) info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "ambient_light_color" || info.name == "ambient_light_energy")
    {
        if (environment.ambientSource == AmbientSource::Disabled)
            info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "fog_aerial_perspective")
    {
        if (environment.background != BackgroundMode::Sky) info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "tonemap_white")
    {
        if (environment.tonemapMode == TonemapMode::Linear ||
                environment.tonemapMode == TonemapMode::Agx)
            info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "tonemap_agx_contrast")
    {
        if (environment.tonemapMode != TonemapMode::Agx) info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "glow_intensity")
    {
        if (environment.glowBlendMode == GlowBlendMode::Mix) info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "glow_mix")
    {
        if (environment.glowBlendMode != GlowBlendMode::Mix) info.usage = PropertyUsage::NoEditor;
        return;
    }
    if (info.name == "background_color")
    {
        if (environment.background != BackgroundMode::Color &&
                environment.ambientSource != AmbientSource::Color)
            info.usage = PropertyUsage::NoEditor;
        return;
    }
}

const EnvironmentSettings& WorldEnvironment::getEnvironment() const
{
    return mEnvironment;
}

void WorldEnvironment::notificationHandler(int what)
{
    Node::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            if (Viewport* viewport = Viewport::findViewport(this))
            {
                mScenario = viewport->getScenario();
                addToGroup(groupName());
                updateCurrentEnvironment();
            }
            break;
        case kNotificationExitTree:
            if (mScenario.valid())
            {
                removeFromGroup(groupName());
                updateCurrentEnvironment();
                mScenario = ScenarioId();
            }
            break;
        default:
            break;
    }
}

ct::String WorldEnvironment::groupName() const
{
    return ct::String("_world_environment_") + ct::String::number(mScenario.bits());
}

void WorldEnvironment::updateCurrentEnvironment()
{
    RenderingServer* server = RenderingServer::singleton();
    SceneTree* tree = getTree();
    if (!server || !tree || !mScenario.valid()) return;
    Node* first = tree->getFirstNodeInGroup(groupName());
    WorldEnvironment* current = first ? first->castTo<WorldEnvironment>() : nullptr;
    if (current) server->scenarioSetEnvironment(mScenario, current->mEnvironment);
    else
        server->scenarioClearEnvironment(mScenario);
}

namespace
{

template<typename F>
void addSkyField(const PropertyInfo& info, F ProceduralSkySettings::*member)
{
    NodeClassDB::addProperty<WorldEnvironment>(
            info,
            [member](WorldEnvironment* node, const PropertyValue& value)
            {
                EnvironmentSettings environment = node->getEnvironment();
                environment.sky.*member = PropertyConvert<F>::from(value);
                node->setEnvironment(environment);
            },
            [member, type = info.type](const WorldEnvironment* node)
            { return PropertyConvert<F>::to(node->getEnvironment().sky.*member, type); });
}

template<typename F>
void addAttributesField(const PropertyInfo& info, F CameraAttributesSettings::*member)
{
    NodeClassDB::addProperty<WorldEnvironment>(
            info,
            [member](WorldEnvironment* node, const PropertyValue& value)
            {
                EnvironmentSettings environment = node->getEnvironment();
                environment.cameraAttributes.*member = PropertyConvert<F>::from(value);
                node->setEnvironment(environment);
            },
            [member, type = info.type](const WorldEnvironment* node) {
                return PropertyConvert<F>::to(node->getEnvironment().cameraAttributes.*member,
                        type);
            });
}

void addGlowLevel(const char* name, int level)
{
    NodeClassDB::addProperty<WorldEnvironment>(
            PropertyInfo::range(PropertyType::Float, name, 0.0, 16.0, 0.01, true),
            [level](WorldEnvironment* node, const PropertyValue& value)
            {
                EnvironmentSettings environment = node->getEnvironment();
                environment.glowLevels[level] = static_cast<float>(value.asFloat());
                node->setEnvironment(environment);
            },
            [level](const WorldEnvironment* node)
            { return PropertyValue::fromSingle(node->getEnvironment().glowLevels[level]); });
}

} // namespace

void WorldEnvironment::bindProperties()
{
    using DB = NodeClassDB;
    using E = EnvironmentSettings;
    using W = WorldEnvironment;
    using S = ProceduralSkySettings;
    auto getter = &W::getEnvironment;
    auto setter = &W::setEnvironment;
    auto range = [](const char* name, double minimum, double maximum, double step,
                         bool orGreater = false, bool orLess = false)
    {
        return PropertyInfo::range(PropertyType::Float, name, minimum, maximum, step, orGreater,
                orLess);
    };

    DB::addPropertyGroup<W>("Background", "background_");
    DB::addFieldProperty<W>(
            PropertyInfo::enumeration("background_mode", "Clear Color,Custom Color,Sky"), getter,
            setter, &E::background);
    DB::addFieldProperty<W>(PropertyInfo::color("background_color", true), getter, setter,
            &E::backgroundColor);
    DB::addFieldProperty<W>(range("background_energy_multiplier", 0.0, 16.0, 0.01), getter, setter,
            &E::backgroundEnergy);

    DB::addPropertyGroup<W>("Sky", "sky_");
    DB::addFieldProperty<W>(PropertyInfo::enumeration("sky_mode", "Procedural,Panorama,Cubemap"),
            getter, setter, &E::skyMode);
    resourceprop::addField<W>(PropertyInfo::resource("sky_texture", "Texture2D"), getter, setter,
            &E::skyTexture);
    DB::addFieldProperty<W>(range("sky_texture_energy_multiplier", 0.0, 16.0, 0.01), getter, setter,
            &E::skyTextureEnergy);
    addSkyField(PropertyInfo::color("sky_top_color", true), &S::skyTopColor);
    addSkyField(PropertyInfo::color("sky_horizon_color", true), &S::skyHorizonColor);
    addSkyField(range("sky_curve", 0.0, 1.0, 0.001, true), &S::skyCurve);
    addSkyField(range("sky_energy_multiplier", 0.0, 64.0, 0.01, true), &S::skyEnergy);
    addSkyField(PropertyInfo::color("sky_ground_bottom_color", true), &S::groundBottomColor);
    addSkyField(PropertyInfo::color("sky_ground_horizon_color", true), &S::groundHorizonColor);
    addSkyField(range("sky_ground_curve", 0.0, 1.0, 0.001, true), &S::groundCurve);
    addSkyField(range("sky_ground_energy_multiplier", 0.0, 64.0, 0.01, true), &S::groundEnergy);
    addSkyField(range("sky_sun_angle_max", 0.0, 360.0, 0.01), &S::sunAngleMax);
    addSkyField(range("sky_sun_curve", 0.0, 1.0, 0.001, true), &S::sunCurve);
    addSkyField(range("sky_overall_energy_multiplier", 0.0, 64.0, 0.01, true), &S::energy);
    addSkyField(PropertyInfo(PropertyType::Bool, "sky_use_debanding"), &S::debanding);
    addSkyField(PropertyInfo(PropertyType::Int, "sky_radiance_size"), &S::radianceSize);

    DB::addPropertyGroup<W>("Ambient Light", "ambient_light_");
    DB::addFieldProperty<W>(
            PropertyInfo::enumeration("ambient_light_source", "Background,Color,Disabled,Sky"),
            getter, setter, &E::ambientSource);
    DB::addFieldProperty<W>(PropertyInfo::color("ambient_light_color", true), getter, setter,
            &E::ambientColor);
    DB::addFieldProperty<W>(range("ambient_light_sky_contribution", 0.0, 1.0, 0.01), getter, setter,
            &E::ambientSkyContribution);
    DB::addFieldProperty<W>(range("ambient_light_energy", 0.0, 16.0, 0.01), getter, setter,
            &E::ambientEnergy);

    DB::addPropertyGroup<W>("Reflected Light", "reflected_light_");
    DB::addFieldProperty<W>(
            PropertyInfo::enumeration("reflected_light_source", "Background,Disabled,Sky"), getter,
            setter, &E::reflectionSource);

    DB::addPropertyGroup<W>("Tonemap", "tonemap_");
    DB::addFieldProperty<W>(
            PropertyInfo::enumeration("tonemap_mode", "Linear,Reinhard,Filmic,ACES,AgX"), getter,
            setter, &E::tonemapMode);
    DB::addFieldProperty<W>(range("tonemap_exposure", 0.0, 4.0, 0.01, true), getter, setter,
            &E::tonemapExposure);
    DB::addFieldProperty<W>(range("tonemap_white", 1.0, 16.0, 0.01, true), getter, setter,
            &E::tonemapWhite);
    DB::addFieldProperty<W>(range("tonemap_agx_contrast", 1.0, 2.0, 0.01, true), getter, setter,
            &E::tonemapAgxContrast);

    DB::addPropertyGroup<W>("SSR", "ssr_");
    DB::addFieldProperty<W>(
            PropertyInfo(PropertyType::Bool, "ssr_enabled", PropertyHint::GroupEnable), getter,
            setter, &E::ssrEnabled);
    DB::addFieldProperty<W>(PropertyInfo::range(PropertyType::Int, "ssr_max_steps", 32, 512, 1),
            getter, setter, &E::ssrMaxSteps);
    DB::addFieldProperty<W>(range("ssr_fade_in", 0.0, 8.0, 0.01, true), getter, setter,
            &E::ssrFadeIn);
    DB::addFieldProperty<W>(range("ssr_fade_out", 0.0, 8.0, 0.01, true), getter, setter,
            &E::ssrFadeOut);
    DB::addFieldProperty<W>(range("ssr_depth_tolerance", 0.01, 128.0, 0.1), getter, setter,
            &E::ssrDepthTolerance);

    DB::addPropertyGroup<W>("SSAO", "ssao_");
    DB::addFieldProperty<W>(
            PropertyInfo(PropertyType::Bool, "ssao_enabled", PropertyHint::GroupEnable), getter,
            setter, &E::ssaoEnabled);
    DB::addFieldProperty<W>(range("ssao_radius", 0.01, 16.0, 0.01, true), getter, setter,
            &E::ssaoRadius);
    DB::addFieldProperty<W>(range("ssao_intensity", 0.0, 16.0, 0.01, true), getter, setter,
            &E::ssaoIntensity);
    DB::addFieldProperty<W>(range("ssao_power", 0.01, 16.0, 0.01, true), getter, setter,
            &E::ssaoPower);
    DB::addFieldProperty<W>(range("ssao_detail", 0.0, 5.0, 0.01), getter, setter, &E::ssaoDetail);
    DB::addFieldProperty<W>(range("ssao_horizon", 0.0, 1.0, 0.01), getter, setter, &E::ssaoHorizon);
    DB::addFieldProperty<W>(range("ssao_sharpness", 0.0, 1.0, 0.01), getter, setter,
            &E::ssaoSharpness);
    DB::addFieldProperty<W>(range("ssao_light_affect", 0.0, 1.0, 0.01), getter, setter,
            &E::ssaoLightAffect);
    DB::addFieldProperty<W>(range("ssao_ao_channel_affect", 0.0, 1.0, 0.01), getter, setter,
            &E::ssaoAoChannelAffect);

    DB::addPropertyGroup<W>("Glow", "glow_");
    DB::addFieldProperty<W>(
            PropertyInfo(PropertyType::Bool, "glow_enabled", PropertyHint::GroupEnable), getter,
            setter, &E::glowEnabled);
    addGlowLevel("glow_levels/1", 0);
    addGlowLevel("glow_levels/2", 1);
    addGlowLevel("glow_levels/3", 2);
    addGlowLevel("glow_levels/4", 3);
    addGlowLevel("glow_levels/5", 4);
    addGlowLevel("glow_levels/6", 5);
    addGlowLevel("glow_levels/7", 6);
    DB::addFieldProperty<W>(PropertyInfo(PropertyType::Bool, "glow_normalized"), getter, setter,
            &E::glowNormalized);
    DB::addFieldProperty<W>(range("glow_intensity", 0.0, 8.0, 0.01), getter, setter,
            &E::glowIntensity);
    DB::addFieldProperty<W>(range("glow_strength", 0.0, 2.0, 0.01), getter, setter,
            &E::glowStrength);
    DB::addFieldProperty<W>(range("glow_mix", 0.0, 1.0, 0.001), getter, setter, &E::glowMix);
    DB::addFieldProperty<W>(range("glow_bloom", 0.0, 1.0, 0.01), getter, setter, &E::glowBloom);
    DB::addFieldProperty<W>(
            PropertyInfo::enumeration("glow_blend_mode", "Additive,Screen,Soft Light,Replace,Mix"),
            getter, setter, &E::glowBlendMode);
    DB::addFieldProperty<W>(range("glow_hdr_threshold", 0.0, 4.0, 0.01), getter, setter,
            &E::glowHdrBleedThreshold);
    DB::addFieldProperty<W>(range("glow_hdr_scale", 0.0, 4.0, 0.01), getter, setter,
            &E::glowHdrBleedScale);
    DB::addFieldProperty<W>(range("glow_hdr_luminance_cap", 0.0, 256.0, 0.01), getter, setter,
            &E::glowHdrLuminanceCap);
    DB::addFieldProperty<W>(range("glow_map_strength", 0.0, 1.0, 0.01), getter, setter,
            &E::glowMapStrength);
    resourceprop::addField<W>(PropertyInfo::resource("glow_map", "Texture2D"), getter, setter,
            &E::glowMap, false);

    DB::addPropertyGroup<W>("Fog", "fog_");
    DB::addFieldProperty<W>(
            PropertyInfo(PropertyType::Bool, "fog_enabled", PropertyHint::GroupEnable), getter,
            setter, &E::fogEnabled);
    DB::addFieldProperty<W>(PropertyInfo::enumeration("fog_mode", "Exponential,Depth"), getter,
            setter, &E::fogMode);
    DB::addFieldProperty<W>(PropertyInfo::color("fog_light_color", true), getter, setter,
            &E::fogLightColor);
    DB::addFieldProperty<W>(range("fog_light_energy", 0.0, 16.0, 0.01, true), getter, setter,
            &E::fogLightEnergy);
    DB::addFieldProperty<W>(range("fog_sun_scatter", 0.0, 1.0, 0.01, true), getter, setter,
            &E::fogSunScatter);
    DB::addFieldProperty<W>(range("fog_density", 0.0, 1.0, 0.0001, true), getter, setter,
            &E::fogDensity);
    DB::addFieldProperty<W>(range("fog_aerial_perspective", 0.0, 1.0, 0.001), getter, setter,
            &E::fogAerialPerspective);
    DB::addFieldProperty<W>(range("fog_sky_affect", 0.0, 1.0, 0.001), getter, setter,
            &E::fogSkyAffect);
    DB::addFieldProperty<W>(range("fog_height", -1024.0, 1024.0, 0.01, true, true), getter, setter,
            &E::fogHeight);
    DB::addFieldProperty<W>(range("fog_height_density", -16.0, 16.0, 0.0001, true, true), getter,
            setter, &E::fogHeightDensity);
    DB::addFieldProperty<W>(PropertyInfo(PropertyType::Float, "fog_depth_curve"), getter, setter,
            &E::fogDepthCurve);
    DB::addFieldProperty<W>(range("fog_depth_begin", 0.0, 4000.0, 0.1, true, true), getter, setter,
            &E::fogDepthBegin);
    DB::addFieldProperty<W>(range("fog_depth_end", 0.0, 4000.0, 0.1, true, true), getter, setter,
            &E::fogDepthEnd);

    DB::addPropertyGroup<W>("Adjustments", "adjustment_");
    DB::addFieldProperty<W>(
            PropertyInfo(PropertyType::Bool, "adjustment_enabled", PropertyHint::GroupEnable),
            getter, setter, &E::adjustmentEnabled);
    DB::addFieldProperty<W>(range("adjustment_brightness", 0.0, 2.0, 0.01, true), getter, setter,
            &E::adjustmentBrightness);
    DB::addFieldProperty<W>(range("adjustment_contrast", 0.75, 1.25, 0.005, true, true), getter,
            setter, &E::adjustmentContrast);
    DB::addFieldProperty<W>(range("adjustment_saturation", 0.0, 2.0, 0.01, true, true), getter,
            setter, &E::adjustmentSaturation);
    resourceprop::addField<W>(PropertyInfo::resource("adjustment_color_correction", "Texture2D"),
            getter, setter, &E::adjustmentColorCorrection, false);

    using A = CameraAttributesSettings;
    DB::addPropertyGroup<W>("Depth of Field", "dof_blur_");
    addAttributesField(range("dof_blur_amount", 0.0, 1.0, 0.01), &A::dofBlurAmount);

    DB::addPropertyGroup<W>("Depth of Field Far", "dof_blur_far_");
    addAttributesField(
            PropertyInfo(PropertyType::Bool, "dof_blur_far_enabled", PropertyHint::GroupEnable),
            &A::dofBlurFarEnabled);
    addAttributesField(range("dof_blur_far_distance", 0.01, 8192.0, 0.01), &A::dofBlurFarDistance);
    addAttributesField(range("dof_blur_far_transition", -1.0, 8192.0, 0.01),
            &A::dofBlurFarTransition);

    DB::addPropertyGroup<W>("Depth of Field Near", "dof_blur_near_");
    addAttributesField(
            PropertyInfo(PropertyType::Bool, "dof_blur_near_enabled", PropertyHint::GroupEnable),
            &A::dofBlurNearEnabled);
    addAttributesField(range("dof_blur_near_distance", 0.01, 8192.0, 0.01),
            &A::dofBlurNearDistance);
    addAttributesField(range("dof_blur_near_transition", -1.0, 8192.0, 0.01),
            &A::dofBlurNearTransition);
}

} // namespace prisma3d
