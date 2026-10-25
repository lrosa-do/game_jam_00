#include "prisma3d/scene/Light3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/Viewport.h"

#include <ct/json.hpp>

namespace prisma3d
{

Light3D::Light3D(LightType type)
        : Node3D(),
          mLight(),
          mType(type),
          mColor(1.0f, 1.0f, 1.0f),
          mParams(),
          mCullMask(0xffffffffu),
          mNegative(false),
          mShadow(false),
          mBakeMode(LightBakeMode::Dynamic),
          mVisibleInWorld(false),
          mLensFlare(),
          mLensFlareCount(0)
{
    for (std::size_t i = 0; i < static_cast<std::size_t>(LightParam::Count); ++i)
        mParams[i] = kLightParamDefaults[i];
    setNotifyTransform(true);

    RenderingServer* server = RenderingServer::singleton();
    if (server) mLight = server->lightCreate(type);
}

Light3D::~Light3D()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightFree(mLight);
}

LightId Light3D::getLight() const
{
    return mLight;
}

LightType Light3D::getLightType() const
{
    return mType;
}

void Light3D::setColor(const Math::Vec3& color)
{
    mColor = color;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetColor(mLight, color);
}

Math::Vec3 Light3D::getColor() const
{
    return mColor;
}

void Light3D::setEnergy(float energy)
{
    setParam(LightParam::Energy, energy);
}

float Light3D::getEnergy() const
{
    return getParam(LightParam::Energy);
}

void Light3D::setSpecular(float specular)
{
    setParam(LightParam::Specular, specular);
}

float Light3D::getSpecular() const
{
    return getParam(LightParam::Specular);
}

void Light3D::setCullMask(std::uint32_t mask)
{
    mCullMask = mask;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetCullMask(mLight, mask);
}

std::uint32_t Light3D::getCullMask() const
{
    return mCullMask;
}

void Light3D::setBakeMode(LightBakeMode mode)
{
    mBakeMode = mode;
    if (RenderingServer* server = RenderingServer::singleton())
        server->lightSetBakeMode(mLight, mode);
}

LightBakeMode Light3D::getBakeMode() const
{
    return mBakeMode;
}

void Light3D::setNegative(bool negative)
{
    mNegative = negative;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetNegative(mLight, negative);
}

bool Light3D::isNegative() const
{
    return mNegative;
}

void Light3D::setShadow(bool enabled)
{
    const bool changed = mShadow != enabled;
    mShadow = enabled;
    if (changed) notifyPropertyListChanged();

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetShadow(mLight, enabled);
}

void Light3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (mShadow) return;
    if ((info.name.starts_with("shadow_") && info.name != "shadow_enabled") ||
            info.name.starts_with("directional_shadow_") || info.name == "omni_shadow_mode")
        info.usage = PropertyUsage::NoEditor;
}

bool Light3D::hasShadow() const
{
    return mShadow;
}

void Light3D::setShadowBias(float bias)
{
    setParam(LightParam::ShadowBias, bias);
}

float Light3D::getShadowBias() const
{
    return getParam(LightParam::ShadowBias);
}

void Light3D::setShadowNormalBias(float bias)
{
    setParam(LightParam::ShadowNormalBias, bias);
}

float Light3D::getShadowNormalBias() const
{
    return getParam(LightParam::ShadowNormalBias);
}

void Light3D::setShadowBlur(float blur)
{
    setParam(LightParam::ShadowBlur, blur);
}

float Light3D::getShadowBlur() const
{
    return getParam(LightParam::ShadowBlur);
}

void Light3D::setShadowOpacity(float opacity)
{
    setParam(LightParam::ShadowOpacity, opacity);
}

float Light3D::getShadowOpacity() const
{
    return getParam(LightParam::ShadowOpacity);
}

void Light3D::setParam(LightParam param, float value)
{
    if (param >= LightParam::Count) return;
    mParams[static_cast<std::size_t>(param)] = value;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetParam(mLight, param, value);
}

float Light3D::getParam(LightParam param) const
{
    return param < LightParam::Count ? mParams[static_cast<std::size_t>(param)] : 0.0f;
}

void Light3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    RenderingServer* server = RenderingServer::singleton();

    switch (what)
    {
        case kNotificationEnterWorld:
        {
            Viewport* viewport = Viewport::findViewport(this);
            if (!viewport) break;
            if (server) server->lightSetScenario(mLight, viewport->getScenario());
            updateVisibility();
            break;
        }
        case kNotificationTransformChanged:
            if (mVisibleInWorld && server) server->lightSetTransform(mLight, getGlobalTransform());
            break;
        case kNotificationVisibilityChanged:
            updateVisibility();
            break;
        case kNotificationExitWorld:
            if (server) server->lightSetScenario(mLight, ScenarioId());
            mVisibleInWorld = false;
            break;
        default:
            break;
    }
}

void Light3D::setLensFlare(const TextureId* textures, std::uint32_t count)
{
    mLensFlareCount = 0;
    for (std::uint32_t i = 0; i < count && mLensFlareCount < kMaxLensFlareElements; ++i)
        if (textures[i].valid()) mLensFlare[mLensFlareCount++] = textures[i];

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetLensFlare(mLight, mLensFlare, mLensFlareCount);
}

void Light3D::setLensFlare(const ct::Vector<TextureId>& textures)
{
    setLensFlare(textures.data(), static_cast<std::uint32_t>(textures.size()));
}

void Light3D::clearLensFlare()
{
    setLensFlare(nullptr, 0);
}

std::uint32_t Light3D::getLensFlareCount() const
{
    return mLensFlareCount;
}

TextureId Light3D::getLensFlareTexture(std::uint32_t index) const
{
    return index < mLensFlareCount ? mLensFlare[index] : TextureId();
}

void Light3D::updateVisibility()
{
    if (!isInsideTree()) return;

    RenderingServer* server = RenderingServer::singleton();

    const bool alreadyVisible = mVisibleInWorld;
    const bool visible = isVisibleInTree();
    mVisibleInWorld = visible;

    if (!server) return;

    if (visible && !alreadyVisible) server->lightSetTransform(mLight, getGlobalTransform());

    server->lightSetVisible(mLight, visible);
}

DirectionalLight3D::DirectionalLight3D()
        : Light3D(LightType::Directional),
          mShadowMode(DirectionalShadowMode::Parallel4Splits),
          mBlendSplits(false)
{
}

void DirectionalLight3D::setAngularDistance(float degrees)
{
    setParam(LightParam::Size, degrees);
}

float DirectionalLight3D::getAngularDistance() const
{
    return getParam(LightParam::Size);
}

void DirectionalLight3D::setShadowMode(DirectionalShadowMode mode)
{
    const bool changed = mShadowMode != mode;
    mShadowMode = mode;
    if (changed) notifyPropertyListChanged();

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetDirectionalShadowMode(getLight(), mode);
}

DirectionalShadowMode DirectionalLight3D::getShadowMode() const
{
    return mShadowMode;
}

void DirectionalLight3D::validateProperty(PropertyInfo& info) const
{
    Light3D::validateProperty(info);
    if (mShadowMode == DirectionalShadowMode::Orthogonal &&
            (info.name == "directional_shadow_split_1" ||
                    info.name == "directional_shadow_blend_splits"))
        info.usage = PropertyUsage::NoEditor;
    if ((mShadowMode == DirectionalShadowMode::Orthogonal ||
                mShadowMode == DirectionalShadowMode::Parallel2Splits) &&
            (info.name == "directional_shadow_split_2" ||
                    info.name == "directional_shadow_split_3"))
        info.usage = PropertyUsage::NoEditor;
}

void DirectionalLight3D::setBlendSplits(bool blend)
{
    mBlendSplits = blend;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetBlendSplits(getLight(), blend);
}

bool DirectionalLight3D::getBlendSplits() const
{
    return mBlendSplits;
}

void DirectionalLight3D::setShadowMaxDistance(float distance)
{
    setParam(LightParam::ShadowMaxDistance, distance);
}

float DirectionalLight3D::getShadowMaxDistance() const
{
    return getParam(LightParam::ShadowMaxDistance);
}

void DirectionalLight3D::setShadowSplit(int index, float offset)
{
    if (index < 1 || index > 3) return;
    setParam(static_cast<LightParam>(static_cast<int>(LightParam::ShadowSplit1) + index - 1),
            offset);
}

float DirectionalLight3D::getShadowSplit(int index) const
{
    if (index < 1 || index > 3) return 0.0f;
    return getParam(
            static_cast<LightParam>(static_cast<int>(LightParam::ShadowSplit1) + index - 1));
}

void DirectionalLight3D::setShadowFadeStart(float start)
{
    setParam(LightParam::ShadowFadeStart, start);
}

float DirectionalLight3D::getShadowFadeStart() const
{
    return getParam(LightParam::ShadowFadeStart);
}

void DirectionalLight3D::setShadowPancakeSize(float size)
{
    setParam(LightParam::ShadowPancakeSize, size);
}

float DirectionalLight3D::getShadowPancakeSize() const
{
    return getParam(LightParam::ShadowPancakeSize);
}

OmniLight3D::OmniLight3D()
        : Light3D(LightType::Omni),
          mShadowMode(OmniShadowMode::Cube)
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetOmniShadowMode(getLight(), mShadowMode);
}

void OmniLight3D::setRange(float range)
{
    setParam(LightParam::Range, range);
}

float OmniLight3D::getRange() const
{
    return getParam(LightParam::Range);
}

void OmniLight3D::setAttenuation(float attenuation)
{
    setParam(LightParam::Attenuation, attenuation);
}

float OmniLight3D::getAttenuation() const
{
    return getParam(LightParam::Attenuation);
}

void OmniLight3D::setSize(float size)
{
    setParam(LightParam::Size, size);
}

float OmniLight3D::getSize() const
{
    return getParam(LightParam::Size);
}

void OmniLight3D::setShadowMode(OmniShadowMode mode)
{
    mShadowMode = mode;

    RenderingServer* server = RenderingServer::singleton();
    if (server) server->lightSetOmniShadowMode(getLight(), mode);
}

OmniShadowMode OmniLight3D::getShadowMode() const
{
    return mShadowMode;
}

SpotLight3D::SpotLight3D()
        : Light3D(LightType::Spot)
{
}

void SpotLight3D::setRange(float range)
{
    setParam(LightParam::Range, range);
}

float SpotLight3D::getRange() const
{
    return getParam(LightParam::Range);
}

void SpotLight3D::setAttenuation(float attenuation)
{
    setParam(LightParam::Attenuation, attenuation);
}

float SpotLight3D::getAttenuation() const
{
    return getParam(LightParam::Attenuation);
}

void SpotLight3D::setSize(float size)
{
    setParam(LightParam::Size, size);
}

float SpotLight3D::getSize() const
{
    return getParam(LightParam::Size);
}

void SpotLight3D::setSpotAngle(float degrees)
{
    setParam(LightParam::SpotAngle, degrees);
}

float SpotLight3D::getSpotAngle() const
{
    return getParam(LightParam::SpotAngle);
}

void SpotLight3D::setSpotAttenuation(float attenuation)
{
    setParam(LightParam::SpotAttenuation, attenuation);
}

float SpotLight3D::getSpotAttenuation() const
{
    return getParam(LightParam::SpotAttenuation);
}

void Light3D::bindProperties()
{
    using DB = NodeClassDB;
    DB::addPropertyGroup<Light3D>("Light", "light_");
    DB::addProperty<Light3D>(PropertyInfo::color("light_color", true), &Light3D::setColor,
            &Light3D::getColor);
    DB::addProperty<Light3D>(
            PropertyInfo::range(PropertyType::Float, "light_energy", 0.0, 16.0, 0.001, true),
            &Light3D::setEnergy, &Light3D::getEnergy);
    DB::addProperty<Light3D>(PropertyInfo(PropertyType::Bool, "light_negative"),
            &Light3D::setNegative, &Light3D::isNegative);
    DB::addProperty<Light3D>(
            PropertyInfo::range(PropertyType::Float, "light_specular", 0.0, 16.0, 0.001, true),
            &Light3D::setSpecular, &Light3D::getSpecular);
    DB::addProperty<Light3D>(PropertyInfo::layers3DRender("light_cull_mask"), &Light3D::setCullMask,
            &Light3D::getCullMask);
    DB::addProperty<Light3D>(
            PropertyInfo::enumeration("light_bake_mode", "Disabled,Static,Dynamic"),
            &Light3D::setBakeMode, &Light3D::getBakeMode);

    DB::addPropertyGroup<Light3D>("Shadow", "shadow_");
    DB::addProperty<Light3D>(
            PropertyInfo(PropertyType::Bool, "shadow_enabled", PropertyHint::GroupEnable),
            &Light3D::setShadow, &Light3D::hasShadow);
    DB::addProperty<Light3D>(
            PropertyInfo::range(PropertyType::Float, "shadow_bias", 0.0, 10.0, 0.001),
            &Light3D::setShadowBias, &Light3D::getShadowBias);
    DB::addProperty<Light3D>(
            PropertyInfo::range(PropertyType::Float, "shadow_normal_bias", 0.0, 10.0, 0.001),
            &Light3D::setShadowNormalBias, &Light3D::getShadowNormalBias);
    DB::addProperty<Light3D>(
            PropertyInfo::range(PropertyType::Float, "shadow_opacity", 0.0, 1.0, 0.01),
            &Light3D::setShadowOpacity, &Light3D::getShadowOpacity);
    DB::addProperty<Light3D>(
            PropertyInfo::range(PropertyType::Float, "shadow_blur", 0.0, 10.0, 0.001),
            &Light3D::setShadowBlur, &Light3D::getShadowBlur);
}

void DirectionalLight3D::bindProperties()
{
    using DB = NodeClassDB;
    DB::addPropertyGroup<DirectionalLight3D>("Light", "light_");
    DB::addProperty<DirectionalLight3D>(
            PropertyInfo::range(PropertyType::Float, "light_angular_distance", 0.0, 90.0, 0.01),
            &DirectionalLight3D::setAngularDistance, &DirectionalLight3D::getAngularDistance);

    DB::addPropertyGroup<DirectionalLight3D>("Directional Shadow", "directional_shadow_");
    DB::addProperty<DirectionalLight3D>(
            PropertyInfo::enumeration("directional_shadow_mode",
                    "Orthogonal (Fast),PSSM 2 Splits (Average),PSSM 4 Splits (Slow)"),
            &DirectionalLight3D::setShadowMode, &DirectionalLight3D::getShadowMode);
    const char* splitNames[3] = { "directional_shadow_split_1", "directional_shadow_split_2",
        "directional_shadow_split_3" };
    DB::addProperty<DirectionalLight3D>(
            PropertyInfo::range(PropertyType::Float, splitNames[0], 0.0, 1.0, 0.001),
            [](DirectionalLight3D* light, const PropertyValue& value)
            { light->setShadowSplit(1, static_cast<float>(value.asFloat())); },
            [](const DirectionalLight3D* light)
            { return PropertyValue::fromSingle(light->getShadowSplit(1)); });
    DB::addProperty<DirectionalLight3D>(
            PropertyInfo::range(PropertyType::Float, splitNames[1], 0.0, 1.0, 0.001),
            [](DirectionalLight3D* light, const PropertyValue& value)
            { light->setShadowSplit(2, static_cast<float>(value.asFloat())); },
            [](const DirectionalLight3D* light)
            { return PropertyValue::fromSingle(light->getShadowSplit(2)); });
    DB::addProperty<DirectionalLight3D>(
            PropertyInfo::range(PropertyType::Float, splitNames[2], 0.0, 1.0, 0.001),
            [](DirectionalLight3D* light, const PropertyValue& value)
            { light->setShadowSplit(3, static_cast<float>(value.asFloat())); },
            [](const DirectionalLight3D* light)
            { return PropertyValue::fromSingle(light->getShadowSplit(3)); });
    DB::addProperty<DirectionalLight3D>(
            PropertyInfo(PropertyType::Bool, "directional_shadow_blend_splits"),
            &DirectionalLight3D::setBlendSplits, &DirectionalLight3D::getBlendSplits);
    DB::addProperty<DirectionalLight3D>(PropertyInfo::range(PropertyType::Float,
                                                "directional_shadow_fade_start", 0.0, 1.0, 0.001),
            &DirectionalLight3D::setShadowFadeStart, &DirectionalLight3D::getShadowFadeStart);
    DB::addProperty<DirectionalLight3D>(PropertyInfo::range(PropertyType::Float,
                                                "directional_shadow_max_distance", 0.0, 8192.0, 0.1,
                                                true),
            &DirectionalLight3D::setShadowMaxDistance, &DirectionalLight3D::getShadowMaxDistance);
    DB::addProperty<DirectionalLight3D>(PropertyInfo::range(PropertyType::Float,
                                                "directional_shadow_pancake_size", 0.0, 1024.0, 0.1,
                                                true),
            &DirectionalLight3D::setShadowPancakeSize, &DirectionalLight3D::getShadowPancakeSize);
}

void OmniLight3D::bindProperties()
{
    using DB = NodeClassDB;
    DB::addPropertyGroup<OmniLight3D>("Light", "light_");
    DB::addProperty<OmniLight3D>(
            PropertyInfo::range(PropertyType::Float, "light_size", 0.0, 1.0, 0.001, true),
            &OmniLight3D::setSize, &OmniLight3D::getSize);

    DB::addPropertyGroup<OmniLight3D>("Omni", "omni_");
    DB::addProperty<OmniLight3D>(
            PropertyInfo::range(PropertyType::Float, "omni_range", 0.0, 4096.0, 0.001, true),
            &OmniLight3D::setRange, &OmniLight3D::getRange);
    DB::addProperty<OmniLight3D>(PropertyInfo::range(PropertyType::Float, "omni_attenuation", -10.0,
                                         10.0, 0.001, true, true),
            &OmniLight3D::setAttenuation, &OmniLight3D::getAttenuation);
    DB::addProperty<OmniLight3D>(
            PropertyInfo::enumeration("omni_shadow_mode", "Dual Paraboloid,Cube"),
            &OmniLight3D::setShadowMode, &OmniLight3D::getShadowMode);
}

void SpotLight3D::bindProperties()
{
    using DB = NodeClassDB;
    DB::addPropertyGroup<SpotLight3D>("Light", "light_");
    DB::addProperty<SpotLight3D>(
            PropertyInfo::range(PropertyType::Float, "light_size", 0.0, 1.0, 0.001, true),
            &SpotLight3D::setSize, &SpotLight3D::getSize);

    DB::addPropertyGroup<SpotLight3D>("Spot", "spot_");
    DB::addProperty<SpotLight3D>(
            PropertyInfo::range(PropertyType::Float, "spot_range", 0.0, 4096.0, 0.001, true),
            &SpotLight3D::setRange, &SpotLight3D::getRange);
    DB::addProperty<SpotLight3D>(PropertyInfo::range(PropertyType::Float, "spot_attenuation", -10.0,
                                         10.0, 0.01, true, true),
            &SpotLight3D::setAttenuation, &SpotLight3D::getAttenuation);
    DB::addProperty<SpotLight3D>(
            PropertyInfo::range(PropertyType::Float, "spot_angle", 0.0, 180.0, 0.01),
            &SpotLight3D::setSpotAngle, &SpotLight3D::getSpotAngle);
    DB::addProperty<SpotLight3D>(PropertyInfo(PropertyType::Float, "spot_angle_attenuation"),
            &SpotLight3D::setSpotAttenuation, &SpotLight3D::getSpotAttenuation);
}

namespace
{

constexpr std::int64_t kLightDataVersion = 1;

}

bool Light3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    if (mLensFlareCount == 0) return false;

    data["version"] = ct::Json(static_cast<long long>(kLightDataVersion));
    ct::Json flare = ct::Json::array();
    for (std::uint32_t i = 0; i < mLensFlareCount; ++i)
    {
        const ct::String texture = sceneres::pathOf(mLensFlare[i]);
        if (texture.empty())
        {
            result.addWarning(scenejson::element(path, "lens_flare", i),
                    "the texture has no path; the lens flare element was not saved");
            continue;
        }
        flare.push_back(ct::Json(texture));
    }
    data["lens_flare"] = flare;
    return true;
}

bool Light3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kLightDataVersion, result, path)) return false;

    const ct::Json* flare = scenejson::arrayField(data, "lens_flare", result, path);
    if (!flare) return true;
    ct::Vector<TextureId> textures;
    for (std::size_t i = 0; i < flare->size(); ++i)
    {
        const ct::Json& entry = flare->at(i);
        const ct::String entryPath = scenejson::element(path, "lens_flare", i);
        if (!entry.is_string())
        {
            result.addError(entryPath, "expected a texture path");
            continue;
        }
        sceneres::clearError();
        const TextureId texture = sceneres::loadTexture(entry.str(), true);
        if (!texture.valid())
        {
            result.addWarning(entryPath, sceneres::takeError());
            continue;
        }
        textures.push_back(texture);
    }
    setLensFlare(textures);
    return true;
}

} // namespace prisma3d
