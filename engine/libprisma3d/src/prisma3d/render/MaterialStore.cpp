#include "prisma3d/render/MaterialStore.h"

#include "platform.h"

#include <string.h>

namespace prisma3d
{

namespace
{

typedef ct::Handle32<MaterialInfo> SlotKey;

SlotKey toKey(MaterialId id)
{
    return SlotKey::from_bits(id.bits());
}

MaterialId toId(SlotKey key)
{
    return MaterialId::from_bits(key.bits());
}

const std::size_t kTextureCount = static_cast<std::size_t>(MaterialTexture::Count);
const std::size_t kBaseTextureCount = static_cast<std::size_t>(MaterialTexture::DetailAlbedo);

const MaterialParamInfo kParameters[] = {
    { "albedo", MaterialParamType::Color, 0.0f, 1.0f },
    { "metallic", MaterialParamType::Float, 0.0f, 1.0f },
    { "specular", MaterialParamType::Float, 0.0f, 1.0f },
    { "roughness", MaterialParamType::Float, 0.0f, 1.0f },
    { "emission", MaterialParamType::Color, 0.0f, 1.0f },
    { "emission_energy", MaterialParamType::Float, 0.0f, 16.0f },
    { "normal_scale", MaterialParamType::Float, -16.0f, 16.0f },
    { "ao_light_affect", MaterialParamType::Float, 0.0f, 1.0f },
    { "clearcoat", MaterialParamType::Float, 0.0f, 1.0f },
    { "clearcoat_roughness", MaterialParamType::Float, 0.0f, 1.0f },
    { "alpha_scissor_threshold", MaterialParamType::Float, 0.0f, 1.0f },
    { "uv1_scale", MaterialParamType::Vec3, -100.0f, 100.0f },
    { "uv1_offset", MaterialParamType::Vec3, -100.0f, 100.0f },
    { "uv2_scale", MaterialParamType::Vec3, -100.0f, 100.0f },
    { "uv2_offset", MaterialParamType::Vec3, -100.0f, 100.0f },
    { "refraction", MaterialParamType::Float, -1.0f, 1.0f },
};

const std::uint32_t kParameterCount = sizeof(kParameters) / sizeof(kParameters[0]);

static_assert(sizeof(kParameters) / sizeof(kParameters[0]) ==
                      static_cast<std::size_t>(MaterialParam::Count),
        "the parameter table must match MaterialParam");

float clampTo(float value, const MaterialParamInfo& info)
{
    if (!(value >= info.minimum)) return info.minimum;
    if (value > info.maximum) return info.maximum;
    return value;
}

template<typename Info>
auto floatSlot(Info& info, MaterialParam param) -> decltype(&info.metallic)
{
    switch (param)
    {
        case MaterialParam::Metallic:
            return &info.metallic;
        case MaterialParam::Specular:
            return &info.specular;
        case MaterialParam::Roughness:
            return &info.roughness;
        case MaterialParam::EmissionEnergy:
            return &info.emissionEnergy;
        case MaterialParam::NormalScale:
            return &info.normalScale;
        case MaterialParam::AoLightAffect:
            return &info.aoLightAffect;
        case MaterialParam::Clearcoat:
            return &info.clearcoat;
        case MaterialParam::ClearcoatRoughness:
            return &info.clearcoatRoughness;
        case MaterialParam::AlphaScissorThreshold:
            return &info.alphaScissorThreshold;
        case MaterialParam::Refraction:
            return &info.refraction;
        default:
            return nullptr;
    }
}

template<typename Info>
auto vec3Slot(Info& info, MaterialParam param) -> decltype(&info.uv1Scale)
{
    switch (param)
    {
        case MaterialParam::Uv1Scale:
            return &info.uv1Scale;
        case MaterialParam::Uv1Offset:
            return &info.uv1Offset;
        case MaterialParam::Uv2Scale:
            return &info.uv2Scale;
        case MaterialParam::Uv2Offset:
            return &info.uv2Offset;
        default:
            return nullptr;
    }
}

bool hasType(MaterialParam param, MaterialParamType type)
{
    return param < MaterialParam::Count &&
           kParameters[static_cast<std::size_t>(param)].type == type;
}

bool storeColor(MaterialInfo& info, MaterialParam param, const Math::Vec4& value)
{
    const MaterialParamInfo& description = kParameters[static_cast<std::size_t>(param)];
    if (param == MaterialParam::Albedo)
    {
        info.albedo = Math::Vec4(clampTo(value.x, description), clampTo(value.y, description),
                clampTo(value.z, description), clampTo(value.w, description));
        return true;
    }
    if (param == MaterialParam::Emission)
    {
        info.emission = Math::Vec3(clampTo(value.x, description), clampTo(value.y, description),
                clampTo(value.z, description));
        return true;
    }
    return false;
}

bool storeVec3(MaterialInfo& info, MaterialParam param, const Math::Vec3& value)
{
    Math::Vec3* slot = vec3Slot(info, param);
    if (!slot) return false;
    const MaterialParamInfo& description = kParameters[static_cast<std::size_t>(param)];
    *slot = Math::Vec3(clampTo(value.x, description), clampTo(value.y, description),
            clampTo(value.z, description));
    return true;
}

bool storeFloat(MaterialInfo& info, MaterialParam param, float value)
{
    float* slot = floatSlot(info, param);
    if (!slot) return false;
    *slot = clampTo(value, kParameters[static_cast<std::size_t>(param)]);
    return true;
}

} // namespace

std::uint32_t MaterialInfo::variantKey() const
{
    std::uint32_t key = static_cast<std::uint32_t>(transparency);
    key |= static_cast<std::uint32_t>(cull) << 2;
    key |= static_cast<std::uint32_t>(shading) << 4;
    for (std::size_t i = 0; i < kBaseTextureCount; ++i)
        if (textures[i].valid()) key |= 1u << (5 + i);
    if (emission.x > 0.0f || emission.y > 0.0f || emission.z > 0.0f) key |= 1u << 13;
    if (clearcoat > 0.0f) key |= 1u << 14;
    key |= static_cast<std::uint32_t>(blend) << 15;
    if (!depthTest) key |= 1u << 18;
    key |= static_cast<std::uint32_t>(billboard) << 19;
    if (grass) key |= 1u << 21;
    for (std::size_t i = kBaseTextureCount; i < kTextureCount; ++i)
        if (textures[i].valid()) key |= 1u << (22 + i - kBaseTextureCount);
    if (detail)
    {
        key |= 1u << 25;
        key |= static_cast<std::uint32_t>(detailBlend) << 26;
        key |= static_cast<std::uint32_t>(detailUv) << 28;
    }
    if (alphaAntialiasing != MaterialAlphaAntialiasing::Off) key |= 1u << 29;
    if (ocean.enabled) key |= 1u << 30;
    return key;
}

MaterialStore* MaterialStore::sSingleton = nullptr;

MaterialStore::MaterialStore(TextureStore& textures)
        : mTextures(textures)
{
    sSingleton = this;
}

MaterialStore::~MaterialStore()
{
    clear();
    if (sSingleton == this) sSingleton = nullptr;
}

MaterialStore* MaterialStore::singleton()
{
    return sSingleton;
}

MaterialId MaterialStore::create(const char* name)
{
    MaterialInfo info;
    if (name) info.name = name;
    return toId(mMaterials.insert(static_cast<MaterialInfo&&>(info)));
}

MaterialId MaterialStore::duplicate(MaterialId material, const char* name)
{
    const MaterialInfo* source = mMaterials.get(toKey(material));
    if (!source) return MaterialId();
    MaterialInfo copy = *source;
    copy.path = ct::String();
    if (name) copy.name = name;
    copy.version = 0;
    return toId(mMaterials.insert(static_cast<MaterialInfo&&>(copy)));
}

void MaterialStore::destroy(MaterialId material)
{
    if (material == mDefault) return;
    mMaterials.erase(toKey(material));
}

void MaterialStore::clear()
{
    mMaterials.clear();
    mDefault = MaterialId();
}

bool MaterialStore::valid(MaterialId material) const
{
    return mMaterials.contains(toKey(material));
}

const MaterialInfo* MaterialStore::get(MaterialId material) const
{
    return mMaterials.get(toKey(material));
}

MaterialId MaterialStore::find(const ct::String& path, const ct::String& name) const
{
    if (path.empty() && name.empty()) return MaterialId();
    const auto materials = mMaterials.items();
    for (std::size_t i = 0; i < materials.size(); ++i)
    {
        const MaterialInfo& info = materials[i];
        if (info.path == path && info.name == name) return toId(mMaterials.handle_at(i));
    }
    return MaterialId();
}

std::size_t MaterialStore::count() const
{
    return mMaterials.size();
}

MaterialId MaterialStore::defaultMaterial()
{
    if (!valid(mDefault)) mDefault = create("default");
    return mDefault;
}

void MaterialStore::setName(MaterialId material, const char* name)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return;
    info->name = name ? name : "";
}

void MaterialStore::setPath(MaterialId material, const ct::String& path)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (info) info->path = path;
}

bool MaterialStore::setFloat(MaterialId material, MaterialParam param, float value)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || !hasType(param, MaterialParamType::Float)) return false;
    if (!storeFloat(*info, param, value)) return false;
    ++info->version;
    return true;
}

bool MaterialStore::setColor(MaterialId material, MaterialParam param, const Math::Vec4& value)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || !hasType(param, MaterialParamType::Color)) return false;
    if (!storeColor(*info, param, value)) return false;
    ++info->version;
    return true;
}

bool MaterialStore::setVec3(MaterialId material, MaterialParam param, const Math::Vec3& value)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || !hasType(param, MaterialParamType::Vec3)) return false;
    if (!storeVec3(*info, param, value)) return false;
    ++info->version;
    return true;
}

bool MaterialStore::getFloat(MaterialId material, MaterialParam param, float& out) const
{
    const MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || !hasType(param, MaterialParamType::Float)) return false;
    const float* slot = floatSlot(*info, param);
    if (!slot) return false;
    out = *slot;
    return true;
}

bool MaterialStore::getColor(MaterialId material, MaterialParam param, Math::Vec4& out) const
{
    const MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || !hasType(param, MaterialParamType::Color)) return false;
    if (param == MaterialParam::Albedo)
    {
        out = info->albedo;
        return true;
    }
    out = Math::Vec4(info->emission.x, info->emission.y, info->emission.z, 1.0f);
    return true;
}

bool MaterialStore::getVec3(MaterialId material, MaterialParam param, Math::Vec3& out) const
{
    const MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || !hasType(param, MaterialParamType::Vec3)) return false;
    const Math::Vec3* slot = vec3Slot(*info, param);
    if (!slot) return false;
    out = *slot;
    return true;
}

bool MaterialStore::setParameter(MaterialId material, const char* name, const float* values,
        std::uint32_t count)
{
    MaterialParam param;
    if (!values || !findParameter(name, param)) return false;
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    switch (kParameters[static_cast<std::size_t>(param)].type)
    {
        case MaterialParamType::Float:
            if (count != 1) return false;
            return setFloat(material, param, values[0]);
        case MaterialParamType::Vec3:
            if (count != 3) return false;
            return setVec3(material, param, Math::Vec3(values[0], values[1], values[2]));
        case MaterialParamType::Color:
            if (param == MaterialParam::Emission ? (count != 3 && count != 4) : count != 4)
                return false;
            return setColor(material, param,
                    Math::Vec4(values[0], values[1], values[2], count == 4 ? values[3] : 1.0f));
    }
    return false;
}

bool MaterialStore::getParameter(MaterialId material, const char* name, float* values,
        std::uint32_t count) const
{
    MaterialParam param;
    if (!values || !findParameter(name, param) || !valid(material)) return false;
    switch (kParameters[static_cast<std::size_t>(param)].type)
    {
        case MaterialParamType::Float:
        {
            if (count != 1) return false;
            return getFloat(material, param, values[0]);
        }
        case MaterialParamType::Vec3:
        {
            if (count != 3) return false;
            Math::Vec3 value;
            if (!getVec3(material, param, value)) return false;
            values[0] = value.x;
            values[1] = value.y;
            values[2] = value.z;
            return true;
        }
        case MaterialParamType::Color:
        {
            if (param == MaterialParam::Emission ? (count != 3 && count != 4) : count != 4)
                return false;
            Math::Vec4 value;
            if (!getColor(material, param, value)) return false;
            values[0] = value.x;
            values[1] = value.y;
            values[2] = value.z;
            if (count == 4) values[3] = value.w;
            return true;
        }
    }
    return false;
}

bool MaterialStore::setTexture(MaterialId material, MaterialTexture slot, TextureId texture)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || slot >= MaterialTexture::Count) return false;
    if (texture.valid() && !mTextures.valid(texture)) return false;
    info->textures[static_cast<std::size_t>(slot)] = texture;
    ++info->version;
    return true;
}

TextureId MaterialStore::getTexture(MaterialId material, MaterialTexture slot) const
{
    const MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || slot >= MaterialTexture::Count) return TextureId();
    return info->textures[static_cast<std::size_t>(slot)];
}

bool MaterialStore::setTextureChannel(MaterialId material, MaterialTexture slot,
        MaterialTextureChannel channel)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    switch (slot)
    {
        case MaterialTexture::Metallic:
            info->metallicChannel = channel;
            break;
        case MaterialTexture::Roughness:
            info->roughnessChannel = channel;
            break;
        case MaterialTexture::AmbientOcclusion:
            info->aoChannel = channel;
            break;
        case MaterialTexture::Refraction:
            info->refractionChannel = channel;
            break;
        default:
            return false;
    }
    ++info->version;
    return true;
}

bool MaterialStore::setTransparency(MaterialId material, MaterialTransparency mode)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->transparency = mode;
    ++info->version;
    return true;
}

bool MaterialStore::setAlphaAntialiasing(MaterialId material, MaterialAlphaAntialiasing mode,
        float edge)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    if (!(edge >= 0.0f)) edge = 0.0f;
    if (edge > 1.0f) edge = 1.0f;
    info->alphaAntialiasing = mode;
    info->alphaAntialiasingEdge = edge;
    ++info->version;
    return true;
}

bool MaterialStore::setCull(MaterialId material, MaterialCull mode)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->cull = mode;
    ++info->version;
    return true;
}

bool MaterialStore::setEmissionOperator(MaterialId material, MaterialEmissionOperator mode)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->emissionOperator = mode;
    ++info->version;
    return true;
}

bool MaterialStore::setShading(MaterialId material, MaterialShading mode)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->shading = mode;
    ++info->version;
    return true;
}

bool MaterialStore::setBlendMode(MaterialId material, MaterialBlend mode)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->blend = mode;
    ++info->version;
    return true;
}

bool MaterialStore::setDepthTest(MaterialId material, bool enabled)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->depthTest = enabled;
    ++info->version;
    return true;
}

bool MaterialStore::setDetail(MaterialId material, bool enabled)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->detail = enabled;
    ++info->version;
    return true;
}

bool MaterialStore::setDetailBlendMode(MaterialId material, MaterialBlend mode)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info || mode == MaterialBlend::Screen) return false;
    info->detailBlend = mode;
    ++info->version;
    return true;
}

bool MaterialStore::setDetailUv(MaterialId material, MaterialDetailUv uv)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->detailUv = uv;
    ++info->version;
    return true;
}

bool MaterialStore::setBillboard(MaterialId material, MaterialBillboard mode, bool keepScale)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->billboard = mode;
    info->billboardKeepScale = keepScale;
    ++info->version;
    return true;
}

bool MaterialStore::setTerrainMorph(MaterialId material, bool enabled)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->terrainMorph = enabled;
    ++info->version;
    return true;
}

bool MaterialStore::setReceiveShadows(MaterialId material, bool receive)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->receiveShadows = receive;
    ++info->version;
    return true;
}

bool MaterialStore::setBacklight(MaterialId material, const Math::Vec3& colour)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->backlight = colour;
    ++info->version;
    return true;
}

bool MaterialStore::setTerrainMaps(MaterialId material, const TerrainMaterialData& data)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->terrain = data;
    ++info->version;
    return true;
}

bool MaterialStore::setGrass(MaterialId material, bool enabled, float drawDistance,
        float cameraBend, const Math::Vec3& tintA, const Math::Vec3& tintB)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->grass = enabled;
    info->grassDrawDistance = drawDistance > 0.0f ? drawDistance : 0.0f;
    info->grassCameraBend = cameraBend;
    info->grassTintA = tintA;
    info->grassTintB = tintB;
    if (enabled) info->billboard = MaterialBillboard::FixedY;
    else
        info->billboard = MaterialBillboard::Disabled;
    ++info->version;
    return true;
}

bool MaterialStore::setGrassWind(MaterialId material, const Math::Vec3& direction, float amplitude,
        float speed)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->grassWindDirection = direction;
    info->grassWindAmplitude = amplitude;
    info->grassWindSpeed = speed;
    ++info->version;
    return true;
}

bool MaterialStore::setLeafWind(MaterialId material, bool enabled, float modelHeight,
        float strength, float speed, const Math::Vec3& direction)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->leafWind = enabled;
    info->leafWindHeight = modelHeight;
    info->leafWindStrength = strength;
    info->leafWindSpeed = speed;
    info->leafWindDirection = direction;
    ++info->version;
    return true;
}

bool MaterialStore::setWater(MaterialId material, bool enabled, const WaterMaterialData& data)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->water = enabled;
    info->waterData = data;
    ++info->version;
    return true;
}

bool MaterialStore::setOcean(MaterialId material, const OceanMaterialData& data)
{
    MaterialInfo* info = mMaterials.get(toKey(material));
    if (!info) return false;
    info->ocean = data;
    ++info->version;
    return true;
}

std::uint32_t MaterialStore::parameterCount()
{
    return kParameterCount;
}

const MaterialParamInfo& MaterialStore::parameter(std::uint32_t index)
{
    return kParameters[index < kParameterCount ? index : 0];
}

const MaterialParamInfo& MaterialStore::parameter(MaterialParam param)
{
    return parameter(static_cast<std::uint32_t>(param));
}

bool MaterialStore::findParameter(const char* name, MaterialParam& out)
{
    if (!name) return false;
    for (std::uint32_t i = 0; i < kParameterCount; ++i)
    {
        if (strcmp(kParameters[i].name, name) == 0)
        {
            out = static_cast<MaterialParam>(i);
            return true;
        }
    }
    return false;
}

} // namespace prisma3d
