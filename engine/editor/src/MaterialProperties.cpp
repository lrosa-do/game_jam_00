#include "prisma3d/editor/MaterialProperties.h"

#include "prisma3d/scene/SceneResources.h"

#include <string.h>

namespace prisma3d
{

namespace
{

using Reader = MaterialField::Reader;
using Writer = MaterialField::Writer;
using Visibility = MaterialField::Visibility;

constexpr int kFeatureCount = static_cast<int>(MaterialFeature::Count);

const char* const kChannelNames = "Red,Green,Blue,Alpha,Gray";
const char* const kBlendNames = "Mix,Add,Subtract,Multiply,Screen";
const char* const kDetailBlendNames = "Mix,Add,Subtract,Multiply";

inline PropertyValue single(float value)
{
    return PropertyValue::fromSingle(value);
}

inline PropertyValue choice(int value)
{
    return PropertyValue::fromInt(value);
}

inline bool isBlack(const Math::Vec3& color)
{
    return color.x <= 0.0f && color.y <= 0.0f && color.z <= 0.0f;
}

template<typename E, int Count>
E chosen(const PropertyValue& value)
{
    std::int64_t index = value.asInt();
    if (index < 0) index = 0;
    if (index >= Count) index = Count - 1;
    return static_cast<E>(index);
}

template<float MaterialInfo::*Member>
PropertyValue readFloat(const MaterialInfo& info)
{
    return single(info.*Member);
}

template<bool MaterialInfo::*Member>
PropertyValue readBool(const MaterialInfo& info)
{
    return PropertyValue::fromBool(info.*Member);
}

template<typename E, E MaterialInfo::*Member>
PropertyValue readEnum(const MaterialInfo& info)
{
    return choice(static_cast<int>(info.*Member));
}

template<Math::Vec3 MaterialInfo::*Member>
PropertyValue readVec3(const MaterialInfo& info)
{
    return PropertyValue::fromVec3(info.*Member);
}

template<Math::Vec3 MaterialInfo::*Member>
PropertyValue readTint(const MaterialInfo& info)
{
    const Math::Vec3 value = info.*Member;
    return PropertyValue::fromColor(Math::Vec4(value.x, value.y, value.z, 1.0f));
}

template<float WaterMaterialData::*Member>
PropertyValue readWaterFloat(const MaterialInfo& info)
{
    return single(info.waterData.*Member);
}

template<Math::Vec2 WaterMaterialData::*Member>
PropertyValue readWaterVec2(const MaterialInfo& info)
{
    return PropertyValue::fromVec2(info.waterData.*Member);
}

template<MaterialTexture Slot>
PropertyValue readTexture(const MaterialInfo& info)
{
    return PropertyValue::fromResourcePath(
            sceneres::pathOf(info.textures[static_cast<std::size_t>(Slot)]));
}

PropertyValue readAlbedo(const MaterialInfo& info)
{
    return PropertyValue::fromColor(info.albedo);
}

PropertyValue readEmission(const MaterialInfo& info)
{
    return PropertyValue::fromColor(
            Math::Vec4(info.emission.x, info.emission.y, info.emission.z, 1.0f));
}

PropertyValue readBacklight(const MaterialInfo& info)
{
    return PropertyValue::fromColor(
            Math::Vec4(info.backlight.x, info.backlight.y, info.backlight.z, 1.0f));
}

template<MaterialParam Param>
bool writeParam(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setFloat(material, Param, static_cast<float>(value.asFloat()));
}

template<MaterialParam Param>
bool writeParamVec3(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setVec3(material, Param, value.asVec3());
}

template<MaterialParam Param>
bool writeParamColor(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setColor(material, Param, value.asColor());
}

template<MaterialTexture Slot>
bool writeChannel(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setTextureChannel(material, Slot, chosen<MaterialTextureChannel, 5>(value));
}

template<MaterialTexture Slot>
bool writeTexture(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const ct::String path = value.asString();
    if (path.empty()) return store.setTexture(material, Slot, TextureId());
    sceneres::clearError();
    const TextureId texture = sceneres::loadTexture(path, MaterialProperties::srgb(Slot));
    if (!texture.valid()) return false;
    return store.setTexture(material, Slot, texture);
}

bool writeTransparency(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setTransparency(material, chosen<MaterialTransparency, 3>(value));
}

bool writeAlphaAntialiasing(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setAlphaAntialiasing(material, chosen<MaterialAlphaAntialiasing, 2>(value),
            info->alphaAntialiasingEdge);
}

bool writeAlphaEdge(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    float edge = static_cast<float>(value.asFloat());
    if (edge < 0.0f) edge = 0.0f;
    if (edge > 1.0f) edge = 1.0f;
    return store.setAlphaAntialiasing(material, info->alphaAntialiasing, edge);
}

bool writeBlend(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setBlendMode(material, chosen<MaterialBlend, 5>(value));
}

bool writeCull(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setCull(material, chosen<MaterialCull, 3>(value));
}

bool writeDepthTest(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setDepthTest(material, value.asBool());
}

bool writeShading(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setShading(material, chosen<MaterialShading, 2>(value));
}

bool writeEmissionOperator(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setEmissionOperator(material, chosen<MaterialEmissionOperator, 2>(value));
}

bool writeBacklight(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const Math::Vec4 color = value.asColor();
    return store.setBacklight(material, Math::Vec3(color.x, color.y, color.z));
}

bool writeDetail(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setDetail(material, value.asBool());
}

bool writeDetailBlend(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setDetailBlendMode(material, chosen<MaterialBlend, 4>(value));
}

bool writeDetailUv(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setDetailUv(material, chosen<MaterialDetailUv, 2>(value));
}

bool writeReceiveShadows(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setReceiveShadows(material, value.asBool());
}

bool writeBillboard(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setBillboard(material, chosen<MaterialBillboard, 2>(value),
            info->billboardKeepScale);
}

bool writeBillboardKeepScale(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setBillboard(material, info->billboard, value.asBool());
}

bool writeGrass(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setGrass(material, value.asBool(), info->grassDrawDistance, info->grassCameraBend,
            info->grassTintA, info->grassTintB);
}

bool writeGrassDistance(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setGrass(material, info->grass, static_cast<float>(value.asFloat()),
            info->grassCameraBend, info->grassTintA, info->grassTintB);
}

bool writeGrassBend(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setGrass(material, info->grass, info->grassDrawDistance,
            static_cast<float>(value.asFloat()), info->grassTintA, info->grassTintB);
}

bool writeGrassTintA(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    const Math::Vec4 color = value.asColor();
    return store.setGrass(material, info->grass, info->grassDrawDistance, info->grassCameraBend,
            Math::Vec3(color.x, color.y, color.z), info->grassTintB);
}

bool writeGrassTintB(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    const Math::Vec4 color = value.asColor();
    return store.setGrass(material, info->grass, info->grassDrawDistance, info->grassCameraBend,
            info->grassTintA, Math::Vec3(color.x, color.y, color.z));
}

bool writeGrassWindDirection(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setGrassWind(material, value.asVec3(), info->grassWindAmplitude,
            info->grassWindSpeed);
}

bool writeGrassWindAmplitude(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setGrassWind(material, info->grassWindDirection,
            static_cast<float>(value.asFloat()), info->grassWindSpeed);
}

bool writeGrassWindSpeed(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setGrassWind(material, info->grassWindDirection, info->grassWindAmplitude,
            static_cast<float>(value.asFloat()));
}

bool writeLeafWind(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setLeafWind(material, value.asBool(), info->leafWindHeight, info->leafWindStrength,
            info->leafWindSpeed, info->leafWindDirection);
}

bool writeLeafWindHeight(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setLeafWind(material, info->leafWind, static_cast<float>(value.asFloat()),
            info->leafWindStrength, info->leafWindSpeed, info->leafWindDirection);
}

bool writeLeafWindStrength(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setLeafWind(material, info->leafWind, info->leafWindHeight,
            static_cast<float>(value.asFloat()), info->leafWindSpeed, info->leafWindDirection);
}

bool writeLeafWindSpeed(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setLeafWind(material, info->leafWind, info->leafWindHeight, info->leafWindStrength,
            static_cast<float>(value.asFloat()), info->leafWindDirection);
}

bool writeLeafWindDirection(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setLeafWind(material, info->leafWind, info->leafWindHeight, info->leafWindStrength,
            info->leafWindSpeed, value.asVec3());
}

bool writeTerrainMorph(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    return store.setTerrainMorph(material, value.asBool());
}

bool writeWater(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    return store.setWater(material, value.asBool(), info->waterData);
}

template<float WaterMaterialData::*Member>
bool writeWaterFloat(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    WaterMaterialData data = info->waterData;
    data.*Member = static_cast<float>(value.asFloat());
    return store.setWater(material, info->water, data);
}

template<Math::Vec2 WaterMaterialData::*Member>
bool writeWaterVec2(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    WaterMaterialData data = info->waterData;
    data.*Member = value.asVec2();
    return store.setWater(material, info->water, data);
}

bool writeWaterDepthFade(MaterialStore& store, MaterialId material, const PropertyValue& value)
{
    const MaterialInfo* info = store.get(material);
    if (!info) return false;
    WaterMaterialData data = info->waterData;
    data.depthFade = value.asVec3();
    return store.setWater(material, info->water, data);
}

bool unused(MaterialStore&, MaterialId, const PropertyValue&)
{
    return false;
}

bool isLit(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.shading == MaterialShading::Lit;
}

bool isScissor(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.transparency == MaterialTransparency::AlphaScissor;
}

bool antialiasingOn(const MaterialInfo& info)
{
    return info.alphaAntialiasing != MaterialAlphaAntialiasing::Off &&
           info.transparency == MaterialTransparency::AlphaScissor;
}

bool isAntialiased(const MaterialInfo& info, const MaterialFeatures&)
{
    return antialiasingOn(info);
}

bool notAntialiased(const MaterialInfo& info, const MaterialFeatures&)
{
    return !antialiasingOn(info);
}

bool litSeparateMaps(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.shading == MaterialShading::Lit && !MaterialProperties::usesOrm(info);
}

bool litEmission(const MaterialInfo& info, const MaterialFeatures& features)
{
    return info.shading == MaterialShading::Lit && features.on(MaterialFeature::Emission);
}

bool hasNormalMap(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.shading == MaterialShading::Lit &&
           info.textures[static_cast<std::size_t>(MaterialTexture::Normal)].valid();
}

bool litClearcoat(const MaterialInfo& info, const MaterialFeatures& features)
{
    return info.shading == MaterialShading::Lit && features.on(MaterialFeature::Clearcoat);
}

bool litBacklight(const MaterialInfo& info, const MaterialFeatures& features)
{
    return info.shading == MaterialShading::Lit && features.on(MaterialFeature::Backlight);
}

bool aoTextureShown(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.shading == MaterialShading::Lit && !MaterialProperties::usesOrm(info);
}

bool aoAmountShown(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.shading == MaterialShading::Lit &&
           (info.textures[static_cast<std::size_t>(MaterialTexture::AmbientOcclusion)].valid() ||
                   MaterialProperties::usesOrm(info));
}

bool aoChannelShown(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.shading == MaterialShading::Lit &&
           info.textures[static_cast<std::size_t>(MaterialTexture::AmbientOcclusion)].valid();
}

bool refractionOn(const MaterialInfo&, const MaterialFeatures& features)
{
    return features.on(MaterialFeature::Refraction);
}

bool detailOn(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.detail;
}

bool billboardOn(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.billboard != MaterialBillboard::Disabled;
}

bool grassOn(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.grass;
}

bool leafWindOn(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.leafWind;
}

bool waterOn(const MaterialInfo& info, const MaterialFeatures&)
{
    return info.water;
}

MaterialField makeField(const char* group, const char* key, const char* label,
        MaterialFieldKind kind, Reader read, Writer write, Visibility visible)
{
    MaterialField field = {};
    field.group = group;
    field.key = key;
    field.label = label;
    field.kind = kind;
    field.names = "";
    field.slot = MaterialTexture::Count;
    field.feature = MaterialFeature::None;
    field.read = read;
    field.write = write;
    field.visible = visible;
    return field;
}

MaterialField boolField(const char* group, const char* key, const char* label, Reader read,
        Writer write, Visibility visible = nullptr)
{
    return makeField(group, key, label, MaterialFieldKind::Bool, read, write, visible);
}

MaterialField featureField(const char* group, const char* key, const char* label,
        MaterialFeature feature, Visibility visible = nullptr)
{
    MaterialField field =
            makeField(group, key, label, MaterialFieldKind::Bool, nullptr, unused, visible);
    field.feature = feature;
    return field;
}

MaterialField floatField(const char* group, const char* key, const char* label, Reader read,
        Writer write, Visibility visible = nullptr)
{
    return makeField(group, key, label, MaterialFieldKind::Float, read, write, visible);
}

MaterialField rangedField(const char* group, const char* key, const char* label, double minimum,
        double maximum, double step, Reader read, Writer write, Visibility visible = nullptr)
{
    MaterialField field = floatField(group, key, label, read, write, visible);
    field.ranged = true;
    field.minimum = minimum;
    field.maximum = maximum;
    field.step = step;
    return field;
}

MaterialField enumField(const char* group, const char* key, const char* label, const char* names,
        Reader read, Writer write, Visibility visible = nullptr)
{
    MaterialField field =
            makeField(group, key, label, MaterialFieldKind::Enum, read, write, visible);
    field.names = names;
    return field;
}

MaterialField vec2Field(const char* group, const char* key, const char* label, Reader read,
        Writer write, Visibility visible = nullptr)
{
    return makeField(group, key, label, MaterialFieldKind::Vec2, read, write, visible);
}

MaterialField vec3Field(const char* group, const char* key, const char* label, Reader read,
        Writer write, Visibility visible = nullptr)
{
    return makeField(group, key, label, MaterialFieldKind::Vec3, read, write, visible);
}

MaterialField colorField(const char* group, const char* key, const char* label, bool noAlpha,
        Reader read, Writer write, Visibility visible = nullptr)
{
    MaterialField field =
            makeField(group, key, label, MaterialFieldKind::Color, read, write, visible);
    field.noAlpha = noAlpha;
    return field;
}

MaterialField textureField(const char* group, const char* key, const char* label,
        MaterialTexture slot, Reader read, Writer write, Visibility visible = nullptr)
{
    MaterialField field =
            makeField(group, key, label, MaterialFieldKind::Texture, read, write, visible);
    field.slot = slot;
    return field;
}

struct Group
{
    const char* name;
    bool open;
};

const Group kGroups[] = { { "Transparency", false }, { "Shading", false }, { "Albedo", true },
    { "ORM", false }, { "Metallic", true }, { "Roughness", true }, { "Emission", false },
    { "Normal Map", false }, { "Clearcoat", false }, { "Ambient Occlusion", false },
    { "Back Lighting", false }, { "Refraction", false }, { "Detail", false }, { "UV1", false },
    { "UV2", false }, { "Shadows", false }, { "Billboard", false }, { "Grass", false },
    { "Leaf Wind", false }, { "Water", false }, { "Terrain", false } };

const std::size_t kGroupCount = sizeof(kGroups) / sizeof(kGroups[0]);

const MaterialField kFields[] = {
    enumField("Transparency", "transparency", "Transparency", "Opaque,Alpha Scissor,Alpha Blend",
            readEnum<MaterialTransparency, &MaterialInfo::transparency>, writeTransparency),
    rangedField("Transparency", "alpha_scissor_threshold", "Alpha Scissor Threshold", 0.0, 1.0,
            0.001, readFloat<&MaterialInfo::alphaScissorThreshold>,
            writeParam<MaterialParam::AlphaScissorThreshold>, isScissor),
    enumField("Transparency", "alpha_antialiasing_mode", "Alpha Antialiasing",
            "Disabled,Alpha To Coverage",
            readEnum<MaterialAlphaAntialiasing, &MaterialInfo::alphaAntialiasing>,
            writeAlphaAntialiasing, isScissor),
    rangedField("Transparency", "alpha_antialiasing_edge", "Alpha Antialiasing Edge", 0.0, 1.0,
            0.01, readFloat<&MaterialInfo::alphaAntialiasingEdge>, writeAlphaEdge, isAntialiased),
    enumField("Transparency", "blend_mode", "Blend Mode", kBlendNames,
            readEnum<MaterialBlend, &MaterialInfo::blend>, writeBlend, notAntialiased),
    enumField("Transparency", "cull_mode", "Cull Mode", "Back,Front,Disabled",
            readEnum<MaterialCull, &MaterialInfo::cull>, writeCull),
    boolField("Transparency", "depth_test", "Depth Test", readBool<&MaterialInfo::depthTest>,
            writeDepthTest),

    enumField("Shading", "shading_mode", "Shading Mode", "Lit,Unshaded",
            readEnum<MaterialShading, &MaterialInfo::shading>, writeShading),

    colorField("Albedo", "albedo_color", "Color", false, readAlbedo,
            writeParamColor<MaterialParam::Albedo>),
    textureField("Albedo", "albedo_texture", "Texture", MaterialTexture::Albedo,
            readTexture<MaterialTexture::Albedo>, writeTexture<MaterialTexture::Albedo>),

    textureField("ORM", "orm_texture", "Texture", MaterialTexture::Orm,
            readTexture<MaterialTexture::Orm>, writeTexture<MaterialTexture::Orm>),

    rangedField("Metallic", "metallic", "Metallic", 0.0, 1.0, 0.01,
            readFloat<&MaterialInfo::metallic>, writeParam<MaterialParam::Metallic>, isLit),
    rangedField("Metallic", "metallic_specular", "Specular", 0.0, 1.0, 0.01,
            readFloat<&MaterialInfo::specular>, writeParam<MaterialParam::Specular>, isLit),
    textureField("Metallic", "metallic_texture", "Texture", MaterialTexture::Metallic,
            readTexture<MaterialTexture::Metallic>, writeTexture<MaterialTexture::Metallic>,
            litSeparateMaps),
    enumField("Metallic", "metallic_texture_channel", "Channel", kChannelNames,
            readEnum<MaterialTextureChannel, &MaterialInfo::metallicChannel>,
            writeChannel<MaterialTexture::Metallic>, litSeparateMaps),

    rangedField("Roughness", "roughness", "Roughness", 0.0, 1.0, 0.01,
            readFloat<&MaterialInfo::roughness>, writeParam<MaterialParam::Roughness>, isLit),
    textureField("Roughness", "roughness_texture", "Texture", MaterialTexture::Roughness,
            readTexture<MaterialTexture::Roughness>, writeTexture<MaterialTexture::Roughness>,
            litSeparateMaps),
    enumField("Roughness", "roughness_texture_channel", "Channel", kChannelNames,
            readEnum<MaterialTextureChannel, &MaterialInfo::roughnessChannel>,
            writeChannel<MaterialTexture::Roughness>, litSeparateMaps),

    featureField("Emission", "emission_enabled", "Enabled", MaterialFeature::Emission, isLit),
    colorField("Emission", "emission", "Color", true, readEmission,
            writeParamColor<MaterialParam::Emission>, litEmission),
    rangedField("Emission", "emission_energy_multiplier", "Energy Multiplier", 0.0, 16.0, 0.01,
            readFloat<&MaterialInfo::emissionEnergy>, writeParam<MaterialParam::EmissionEnergy>,
            litEmission),
    enumField("Emission", "emission_operator", "Operator", "Add,Multiply",
            readEnum<MaterialEmissionOperator, &MaterialInfo::emissionOperator>,
            writeEmissionOperator, litEmission),
    textureField("Emission", "emission_texture", "Texture", MaterialTexture::Emission,
            readTexture<MaterialTexture::Emission>, writeTexture<MaterialTexture::Emission>,
            litEmission),

    rangedField("Normal Map", "normal_scale", "Scale", -16.0, 16.0, 0.01,
            readFloat<&MaterialInfo::normalScale>, writeParam<MaterialParam::NormalScale>,
            hasNormalMap),
    textureField("Normal Map", "normal_texture", "Texture", MaterialTexture::Normal,
            readTexture<MaterialTexture::Normal>, writeTexture<MaterialTexture::Normal>, isLit),

    featureField("Clearcoat", "clearcoat_enabled", "Enabled", MaterialFeature::Clearcoat, isLit),
    rangedField("Clearcoat", "clearcoat", "Amount", 0.0, 1.0, 0.01,
            readFloat<&MaterialInfo::clearcoat>, writeParam<MaterialParam::Clearcoat>,
            litClearcoat),
    rangedField("Clearcoat", "clearcoat_roughness", "Roughness", 0.0, 1.0, 0.01,
            readFloat<&MaterialInfo::clearcoatRoughness>,
            writeParam<MaterialParam::ClearcoatRoughness>, litClearcoat),
    textureField("Clearcoat", "clearcoat_texture", "Texture", MaterialTexture::Clearcoat,
            readTexture<MaterialTexture::Clearcoat>, writeTexture<MaterialTexture::Clearcoat>,
            litClearcoat),

    rangedField("Ambient Occlusion", "ao_light_affect", "Light Affect", 0.0, 1.0, 0.01,
            readFloat<&MaterialInfo::aoLightAffect>, writeParam<MaterialParam::AoLightAffect>,
            aoAmountShown),
    textureField("Ambient Occlusion", "ao_texture", "Texture", MaterialTexture::AmbientOcclusion,
            readTexture<MaterialTexture::AmbientOcclusion>,
            writeTexture<MaterialTexture::AmbientOcclusion>, aoTextureShown),
    enumField("Ambient Occlusion", "ao_texture_channel", "Channel", kChannelNames,
            readEnum<MaterialTextureChannel, &MaterialInfo::aoChannel>,
            writeChannel<MaterialTexture::AmbientOcclusion>, aoChannelShown),

    featureField("Back Lighting", "backlight_enabled", "Enabled", MaterialFeature::Backlight,
            isLit),
    colorField("Back Lighting", "backlight", "Color", true, readBacklight, writeBacklight,
            litBacklight),

    featureField("Refraction", "refraction_enabled", "Enabled", MaterialFeature::Refraction),
    rangedField("Refraction", "refraction_scale", "Scale", -1.0, 1.0, 0.01,
            readFloat<&MaterialInfo::refraction>, writeParam<MaterialParam::Refraction>,
            refractionOn),
    textureField("Refraction", "refraction_texture", "Texture", MaterialTexture::Refraction,
            readTexture<MaterialTexture::Refraction>, writeTexture<MaterialTexture::Refraction>,
            refractionOn),
    enumField("Refraction", "refraction_texture_channel", "Channel", kChannelNames,
            readEnum<MaterialTextureChannel, &MaterialInfo::refractionChannel>,
            writeChannel<MaterialTexture::Refraction>, refractionOn),

    boolField("Detail", "detail_enabled", "Enabled", readBool<&MaterialInfo::detail>, writeDetail),
    textureField("Detail", "detail_mask", "Mask", MaterialTexture::DetailMask,
            readTexture<MaterialTexture::DetailMask>, writeTexture<MaterialTexture::DetailMask>,
            detailOn),
    enumField("Detail", "detail_blend_mode", "Blend Mode", kDetailBlendNames,
            readEnum<MaterialBlend, &MaterialInfo::detailBlend>, writeDetailBlend, detailOn),
    enumField("Detail", "detail_uv_layer", "UV Layer", "UV1,UV2",
            readEnum<MaterialDetailUv, &MaterialInfo::detailUv>, writeDetailUv, detailOn),
    textureField("Detail", "detail_albedo", "Albedo", MaterialTexture::DetailAlbedo,
            readTexture<MaterialTexture::DetailAlbedo>, writeTexture<MaterialTexture::DetailAlbedo>,
            detailOn),
    textureField("Detail", "detail_normal", "Normal", MaterialTexture::DetailNormal,
            readTexture<MaterialTexture::DetailNormal>, writeTexture<MaterialTexture::DetailNormal>,
            detailOn),

    vec3Field("UV1", "uv1_scale", "Scale", readVec3<&MaterialInfo::uv1Scale>,
            writeParamVec3<MaterialParam::Uv1Scale>),
    vec3Field("UV1", "uv1_offset", "Offset", readVec3<&MaterialInfo::uv1Offset>,
            writeParamVec3<MaterialParam::Uv1Offset>),
    vec3Field("UV2", "uv2_scale", "Scale", readVec3<&MaterialInfo::uv2Scale>,
            writeParamVec3<MaterialParam::Uv2Scale>),
    vec3Field("UV2", "uv2_offset", "Offset", readVec3<&MaterialInfo::uv2Offset>,
            writeParamVec3<MaterialParam::Uv2Offset>),

    boolField("Shadows", "receive_shadows", "Receive Shadows",
            readBool<&MaterialInfo::receiveShadows>, writeReceiveShadows),

    enumField("Billboard", "billboard_mode", "Mode", "Disabled,Y-Billboard",
            readEnum<MaterialBillboard, &MaterialInfo::billboard>, writeBillboard),
    boolField("Billboard", "billboard_keep_scale", "Keep Scale",
            readBool<&MaterialInfo::billboardKeepScale>, writeBillboardKeepScale, billboardOn),

    boolField("Grass", "grass", "Enabled", readBool<&MaterialInfo::grass>, writeGrass),
    floatField("Grass", "grass_draw_distance", "Draw Distance",
            readFloat<&MaterialInfo::grassDrawDistance>, writeGrassDistance, grassOn),
    rangedField("Grass", "grass_camera_bend", "Camera Bend", 0.0, 1.0, 0.01,
            readFloat<&MaterialInfo::grassCameraBend>, writeGrassBend, grassOn),
    colorField("Grass", "grass_tint_a", "Tint A", true, readTint<&MaterialInfo::grassTintA>,
            writeGrassTintA, grassOn),
    colorField("Grass", "grass_tint_b", "Tint B", true, readTint<&MaterialInfo::grassTintB>,
            writeGrassTintB, grassOn),
    vec3Field("Grass", "grass_wind_direction", "Wind Direction",
            readVec3<&MaterialInfo::grassWindDirection>, writeGrassWindDirection, grassOn),
    floatField("Grass", "grass_wind_amplitude", "Wind Amplitude",
            readFloat<&MaterialInfo::grassWindAmplitude>, writeGrassWindAmplitude, grassOn),
    floatField("Grass", "grass_wind_speed", "Wind Speed", readFloat<&MaterialInfo::grassWindSpeed>,
            writeGrassWindSpeed, grassOn),

    boolField("Leaf Wind", "leaf_wind", "Enabled", readBool<&MaterialInfo::leafWind>,
            writeLeafWind),
    floatField("Leaf Wind", "leaf_wind_height", "Model Height",
            readFloat<&MaterialInfo::leafWindHeight>, writeLeafWindHeight, leafWindOn),
    floatField("Leaf Wind", "leaf_wind_strength", "Strength",
            readFloat<&MaterialInfo::leafWindStrength>, writeLeafWindStrength, leafWindOn),
    floatField("Leaf Wind", "leaf_wind_speed", "Speed", readFloat<&MaterialInfo::leafWindSpeed>,
            writeLeafWindSpeed, leafWindOn),
    vec3Field("Leaf Wind", "leaf_wind_direction", "Direction",
            readVec3<&MaterialInfo::leafWindDirection>, writeLeafWindDirection, leafWindOn),

    boolField("Water", "water", "Enabled", readBool<&MaterialInfo::water>, writeWater),
    vec2Field("Water", "water_wind_direction", "Wind Direction",
            readWaterVec2<&WaterMaterialData::windDirection>,
            writeWaterVec2<&WaterMaterialData::windDirection>, waterOn),
    floatField("Water", "water_wind_speed", "Wind Speed",
            readWaterFloat<&WaterMaterialData::windSpeed>,
            writeWaterFloat<&WaterMaterialData::windSpeed>, waterOn),
    vec2Field("Water", "water_waves_big", "Big Waves", readWaterVec2<&WaterMaterialData::bigWaves>,
            writeWaterVec2<&WaterMaterialData::bigWaves>, waterOn),
    vec2Field("Water", "water_waves_mid", "Mid Waves", readWaterVec2<&WaterMaterialData::midWaves>,
            writeWaterVec2<&WaterMaterialData::midWaves>, waterOn),
    vec2Field("Water", "water_waves_small", "Small Waves",
            readWaterVec2<&WaterMaterialData::smallWaves>,
            writeWaterVec2<&WaterMaterialData::smallWaves>, waterOn),
    floatField("Water", "water_wave_scale", "Wave Scale",
            readWaterFloat<&WaterMaterialData::waveScale>,
            writeWaterFloat<&WaterMaterialData::waveScale>, waterOn),
    floatField("Water", "water_wave_chop", "Wave Chop",
            readWaterFloat<&WaterMaterialData::waveChop>,
            writeWaterFloat<&WaterMaterialData::waveChop>, waterOn),
    floatField("Water", "water_bump", "Bump", readWaterFloat<&WaterMaterialData::bump>,
            writeWaterFloat<&WaterMaterialData::bump>, waterOn),
    floatField("Water", "water_bump2", "Bump 2", readWaterFloat<&WaterMaterialData::bump2>,
            writeWaterFloat<&WaterMaterialData::bump2>, waterOn),
    floatField("Water", "water_specular_mix", "Specular Mix",
            readWaterFloat<&WaterMaterialData::specularMix>,
            writeWaterFloat<&WaterMaterialData::specularMix>, waterOn),
    vec3Field(
            "Water", "water_depth_fade", "Depth Fade", [](const MaterialInfo& info)
            { return PropertyValue::fromVec3(info.waterData.depthFade); }, writeWaterDepthFade,
            waterOn),
    floatField("Water", "water_depth_scale", "Depth Scale",
            readWaterFloat<&WaterMaterialData::depthScale>,
            writeWaterFloat<&WaterMaterialData::depthScale>, waterOn),
    floatField("Water", "water_shore_fade", "Shore Fade",
            readWaterFloat<&WaterMaterialData::shoreFade>,
            writeWaterFloat<&WaterMaterialData::shoreFade>, waterOn),

    boolField("Terrain", "terrain_morph", "Morph", readBool<&MaterialInfo::terrainMorph>,
            writeTerrainMorph),
};

const std::size_t kFieldCount = sizeof(kFields) / sizeof(kFields[0]);

} // namespace

MaterialFeatures MaterialFeatures::of(const MaterialInfo& info)
{
    MaterialFeatures features;
    features.enabled[static_cast<int>(MaterialFeature::Emission)] = !isBlack(info.emission);
    features.enabled[static_cast<int>(MaterialFeature::Clearcoat)] = info.clearcoat > 0.0f;
    features.enabled[static_cast<int>(MaterialFeature::Backlight)] = !isBlack(info.backlight);
    features.enabled[static_cast<int>(MaterialFeature::Refraction)] = info.refraction != 0.0f;
    if (!isBlack(info.emission)) features.emissionColor = info.emission;
    if (info.clearcoat > 0.0f) features.clearcoatAmount = info.clearcoat;
    if (!isBlack(info.backlight)) features.backlightColor = info.backlight;
    if (info.refraction != 0.0f) features.refractionScale = info.refraction;
    return features;
}

bool MaterialFeatures::on(MaterialFeature feature) const
{
    const int slot = static_cast<int>(feature);
    return slot >= 0 && slot < kFeatureCount && enabled[slot];
}

bool MaterialFeatures::set(MaterialStore& store, MaterialId material, MaterialFeature feature,
        bool value)
{
    const MaterialInfo* info = store.get(material);
    const int slot = static_cast<int>(feature);
    if (!info || slot < 0 || slot >= kFeatureCount) return false;
    enabled[slot] = value;
    switch (feature)
    {
        case MaterialFeature::Emission:
            if (!value)
            {
                if (!isBlack(info->emission)) emissionColor = info->emission;
                return store.setColor(material, MaterialParam::Emission,
                        Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            }
            if (!isBlack(info->emission)) return true;
            return store.setColor(material, MaterialParam::Emission,
                    Math::Vec4(emissionColor.x, emissionColor.y, emissionColor.z, 1.0f));
        case MaterialFeature::Clearcoat:
            if (!value)
            {
                if (info->clearcoat > 0.0f) clearcoatAmount = info->clearcoat;
                return store.setFloat(material, MaterialParam::Clearcoat, 0.0f);
            }
            if (info->clearcoat > 0.0f) return true;
            return store.setFloat(material, MaterialParam::Clearcoat, clearcoatAmount);
        case MaterialFeature::Backlight:
            if (!value)
            {
                if (!isBlack(info->backlight)) backlightColor = info->backlight;
                return store.setBacklight(material, Math::Vec3(0.0f, 0.0f, 0.0f));
            }
            if (!isBlack(info->backlight)) return true;
            return store.setBacklight(material, backlightColor);
        case MaterialFeature::Refraction:
            if (!value)
            {
                if (info->refraction != 0.0f) refractionScale = info->refraction;
                return store.setFloat(material, MaterialParam::Refraction, 0.0f);
            }
            if (info->refraction != 0.0f) return true;
            return store.setFloat(material, MaterialParam::Refraction, refractionScale);
        default:
            return false;
    }
}

std::size_t MaterialProperties::count()
{
    return kFieldCount;
}

const MaterialField& MaterialProperties::field(std::size_t index)
{
    return kFields[index < kFieldCount ? index : 0];
}

const MaterialField* MaterialProperties::find(const char* key)
{
    const std::size_t found = indexOf(key);
    return found < kFieldCount ? &kFields[found] : nullptr;
}

std::size_t MaterialProperties::indexOf(const char* key)
{
    for (std::size_t i = 0; key && i < kFieldCount; ++i)
        if (strcmp(kFields[i].key, key) == 0) return i;
    return kFieldCount;
}

std::size_t MaterialProperties::groupCount()
{
    return kGroupCount;
}

const char* MaterialProperties::groupName(std::size_t index)
{
    return index < kGroupCount ? kGroups[index].name : "";
}

bool MaterialProperties::groupOpenByDefault(std::size_t index)
{
    return index < kGroupCount && kGroups[index].open;
}

std::size_t MaterialProperties::groupOf(std::size_t fieldIndex)
{
    if (fieldIndex >= kFieldCount) return kGroupCount;
    for (std::size_t i = 0; i < kGroupCount; ++i)
        if (strcmp(kGroups[i].name, kFields[fieldIndex].group) == 0) return i;
    return kGroupCount;
}

PropertyInfo MaterialProperties::infoOf(const MaterialField& field)
{
    switch (field.kind)
    {
        case MaterialFieldKind::Bool:
            return PropertyInfo(PropertyType::Bool, field.key);
        case MaterialFieldKind::Float:
            if (field.ranged)
                return PropertyInfo::range(PropertyType::Float, field.key, field.minimum,
                        field.maximum, field.step);
            return PropertyInfo(PropertyType::Float, field.key);
        case MaterialFieldKind::Enum:
            return PropertyInfo::enumeration(field.key, field.names);
        case MaterialFieldKind::Vec2:
            return PropertyInfo(PropertyType::Vec2, field.key);
        case MaterialFieldKind::Vec3:
            return PropertyInfo(PropertyType::Vec3, field.key);
        case MaterialFieldKind::Color:
            return PropertyInfo::color(field.key, field.noAlpha);
        case MaterialFieldKind::Texture:
            return PropertyInfo::resource(field.key, "Texture2D");
    }
    return PropertyInfo();
}

PropertyValue MaterialProperties::read(const MaterialInfo& info, const MaterialFeatures& features,
        const MaterialField& field)
{
    if (field.feature != MaterialFeature::None)
        return PropertyValue::fromBool(features.on(field.feature));
    return field.read(info);
}

bool MaterialProperties::write(MaterialStore& store, MaterialId material,
        MaterialFeatures& features, const MaterialField& field, const PropertyValue& value)
{
    if (!store.valid(material)) return false;
    if (field.feature != MaterialFeature::None)
        return features.set(store, material, field.feature, value.asBool());
    return field.write(store, material, value);
}

bool MaterialProperties::visible(const MaterialInfo& info, const MaterialFeatures& features,
        const MaterialField& field)
{
    return field.visible == nullptr || field.visible(info, features);
}

bool MaterialProperties::endsWithNoCase(const ct::String& text, const char* suffix)
{
    const std::size_t length = strlen(suffix);
    if (text.size() < length) return false;
    for (std::size_t i = 0; i < length; ++i)
    {
        char a = text[text.size() - length + i];
        char b = suffix[i];
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
        if (a != b) return false;
    }
    return true;
}

bool MaterialProperties::editable(const MaterialInfo& info)
{
    return !info.name.empty() && endsWithNoCase(info.path, kLibraryExtension);
}

ct::String MaterialProperties::sourceOf(const MaterialInfo* info)
{
    if (!info) return ct::String(kNoneText);
    if (info->path.empty()) return ct::String(kUnsavedText);
    if (!endsWithNoCase(info->path, kLibraryExtension)) return ct::String(kBuiltInText);
    return info->path + "::" + info->name;
}

ct::String MaterialProperties::sourceOf(MaterialId material)
{
    MaterialStore* store = MaterialStore::singleton();
    return sourceOf(store ? store->get(material) : nullptr);
}

bool MaterialProperties::srgb(MaterialTexture slot)
{
    return slot == MaterialTexture::Albedo || slot == MaterialTexture::Emission ||
           slot == MaterialTexture::DetailAlbedo;
}

bool MaterialProperties::usesOrm(const MaterialInfo& info)
{
    return !info.textures[static_cast<std::size_t>(MaterialTexture::Metallic)].valid() &&
           !info.textures[static_cast<std::size_t>(MaterialTexture::Roughness)].valid() &&
           info.textures[static_cast<std::size_t>(MaterialTexture::Orm)].valid();
}

} // namespace prisma3d
