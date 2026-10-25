#pragma once

#include "prisma3d/render/TerrainMaterialData.h"
#include "prisma3d/render/TextureStore.h"

#include "mathc.h"

#include <ct/slotmap32.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

struct MaterialTag;
using MaterialId = ct::Handle32<MaterialTag>;

enum class MaterialParam : std::uint8_t
{
    Albedo,
    Metallic,
    Specular,
    Roughness,
    Emission,
    EmissionEnergy,
    NormalScale,
    AoLightAffect,
    Clearcoat,
    ClearcoatRoughness,
    AlphaScissorThreshold,
    Uv1Scale,
    Uv1Offset,
    Uv2Scale,
    Uv2Offset,
    Refraction,
    Count
};

enum class MaterialTexture : std::uint8_t
{
    Albedo,
    Metallic,
    Roughness,
    Emission,
    Normal,
    AmbientOcclusion,
    Clearcoat,
    Orm,
    DetailAlbedo,
    DetailNormal,
    DetailMask,
    Refraction,
    Count
};

enum class MaterialParamType : std::uint8_t
{
    Float,
    Color,
    Vec3
};

enum class MaterialTransparency : std::uint8_t
{
    Opaque,
    AlphaScissor,
    AlphaBlend
};

enum class MaterialAlphaAntialiasing : std::uint8_t
{
    Off,
    AlphaToCoverage
};

enum class MaterialEmissionOperator : std::uint8_t
{
    Add,
    Multiply
};

enum class MaterialCull : std::uint8_t
{
    Back,
    Front,
    Disabled
};

enum class MaterialShading : std::uint8_t
{
    Lit,
    Unshaded
};

enum class MaterialBillboard : std::uint8_t
{
    Disabled,
    FixedY
};

enum class MaterialBlend : std::uint8_t
{
    Mix,
    Add,
    Sub,
    Mul,
    Screen
};

enum class MaterialDetailUv : std::uint8_t
{
    Uv1,
    Uv2
};

enum class MaterialTextureChannel : std::uint8_t
{
    Red,
    Green,
    Blue,
    Alpha,
    Grayscale
};

struct WaterMaterialData
{
    Math::Vec2 windDirection = Math::Vec2(0.1f, -2.1f);
    float windSpeed = 1.0f;
    Math::Vec2 bigWaves = Math::Vec2(0.3f, 0.3f);
    Math::Vec2 midWaves = Math::Vec2(0.3f, 0.15f);
    Math::Vec2 smallWaves = Math::Vec2(0.15f, 0.1f);
    float waveScale = 25.0f;
    float waveChop = 0.15f;
    float bump = 1.5f;
    float bump2 = 6.0f;
    float specularMix = 0.4f;
    Math::Vec3 depthFade = Math::Vec3(0.06f, 0.02f, 0.01f);
    float depthScale = 0.6f;
    float shoreFade = 16.4f;
};

struct OceanMaterialData
{
    bool enabled = false;
    TextureId displacementMap;
    TextureId gradientMap;
    TextureId perlinMap;
    Math::Vec4 waterColor = Math::Vec4(0.0f, 2.0f / 255.0f, 6.0f / 255.0f, 0.6f);
    Math::Vec3 extinctionColor = Math::Vec3(0.0f, 0.9f, 1.0f);
    float waterHeight = 0.0f;
    float patchLength = 50.0f;
    float waveAmplitude = 1000.0f;
    float timeScale = 0.3f;
    float displacementTolerance = 2.0f;
    std::uint32_t gridWidth = 640;
    std::uint32_t gridHeight = 360;
};

struct MaterialParamInfo
{
    const char* name;
    MaterialParamType type;
    float minimum;
    float maximum;
};

struct MaterialInfo
{
    ct::String name;
    ct::String path;
    Math::Vec4 albedo = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
    float metallic = 0.0f;
    float specular = 0.5f;
    float roughness = 1.0f;
    Math::Vec3 emission = Math::Vec3(0.0f, 0.0f, 0.0f);
    float emissionEnergy = 1.0f;
    float normalScale = 1.0f;
    float aoLightAffect = 0.0f;
    float clearcoat = 0.0f;
    float clearcoatRoughness = 0.5f;
    float alphaScissorThreshold = 0.5f;
    MaterialAlphaAntialiasing alphaAntialiasing = MaterialAlphaAntialiasing::Off;
    float alphaAntialiasingEdge = 0.3f;
    Math::Vec3 uv1Scale = Math::Vec3(1.0f, 1.0f, 1.0f);
    Math::Vec3 uv1Offset = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 uv2Scale = Math::Vec3(1.0f, 1.0f, 1.0f);
    Math::Vec3 uv2Offset = Math::Vec3(0.0f, 0.0f, 0.0f);
    TextureId textures[static_cast<std::size_t>(MaterialTexture::Count)];
    MaterialTextureChannel metallicChannel = MaterialTextureChannel::Red;
    MaterialTextureChannel roughnessChannel = MaterialTextureChannel::Red;
    MaterialTextureChannel aoChannel = MaterialTextureChannel::Red;
    MaterialTextureChannel refractionChannel = MaterialTextureChannel::Red;
    float refraction = 0.0f;
    MaterialTransparency transparency = MaterialTransparency::Opaque;
    MaterialCull cull = MaterialCull::Back;
    MaterialEmissionOperator emissionOperator = MaterialEmissionOperator::Add;
    MaterialShading shading = MaterialShading::Lit;
    MaterialBlend blend = MaterialBlend::Mix;
    bool depthTest = true;
    bool detail = false;
    MaterialBlend detailBlend = MaterialBlend::Mix;
    MaterialDetailUv detailUv = MaterialDetailUv::Uv1;
    MaterialBillboard billboard = MaterialBillboard::Disabled;
    bool billboardKeepScale = false;
    bool receiveShadows = true;
    Math::Vec3 backlight = Math::Vec3(0.0f, 0.0f, 0.0f);
    bool grass = false;
    float grassDrawDistance = 60.0f;
    float grassCameraBend = 0.55f;
    Math::Vec3 grassTintA = Math::Vec3(0.62f, 0.78f, 0.42f);
    Math::Vec3 grassTintB = Math::Vec3(0.95f, 0.92f, 0.60f);
    Math::Vec3 grassWindDirection = Math::Vec3(0.82f, 0.0f, 0.57f);
    float grassWindAmplitude = 0.0f;
    float grassWindSpeed = 1.0f;
    bool leafWind = false;
    Math::Vec3 leafWindDirection = Math::Vec3(0.82f, 0.0f, 0.57f);
    float leafWindStrength = 1.0f;
    float leafWindSpeed = 1.0f;
    float leafWindHeight = 1.0f;
    bool terrainMorph = false;
    bool water = false;
    WaterMaterialData waterData;
    TerrainMaterialData terrain;
    OceanMaterialData ocean;
    std::uint32_t version = 0;

    std::uint32_t variantKey() const;
};

class MaterialStore
{
public:
    explicit MaterialStore(TextureStore& textures);
    ~MaterialStore();
    MaterialStore(const MaterialStore&) = delete;
    MaterialStore& operator=(const MaterialStore&) = delete;

    static MaterialStore* singleton();

    MaterialId create(const char* name = nullptr);
    MaterialId duplicate(MaterialId material, const char* name = nullptr);
    void destroy(MaterialId material);
    void clear();

    bool valid(MaterialId material) const;
    const MaterialInfo* get(MaterialId material) const;
    MaterialId find(const ct::String& path, const ct::String& name) const;
    std::size_t count() const;
    MaterialId defaultMaterial();

    void setName(MaterialId material, const char* name);
    void setPath(MaterialId material, const ct::String& path);

    bool setFloat(MaterialId material, MaterialParam param, float value);
    bool setColor(MaterialId material, MaterialParam param, const Math::Vec4& value);
    bool setVec3(MaterialId material, MaterialParam param, const Math::Vec3& value);
    bool getFloat(MaterialId material, MaterialParam param, float& out) const;
    bool getColor(MaterialId material, MaterialParam param, Math::Vec4& out) const;
    bool getVec3(MaterialId material, MaterialParam param, Math::Vec3& out) const;

    bool setParameter(MaterialId material, const char* name, const float* values,
            std::uint32_t count);
    bool getParameter(MaterialId material, const char* name, float* values,
            std::uint32_t count) const;

    bool setTexture(MaterialId material, MaterialTexture slot, TextureId texture);
    TextureId getTexture(MaterialId material, MaterialTexture slot) const;
    bool setTextureChannel(MaterialId material, MaterialTexture slot,
            MaterialTextureChannel channel);
    bool setTransparency(MaterialId material, MaterialTransparency mode);
    bool setAlphaAntialiasing(MaterialId material, MaterialAlphaAntialiasing mode,
            float edge = 0.3f);
    bool setCull(MaterialId material, MaterialCull mode);
    bool setEmissionOperator(MaterialId material, MaterialEmissionOperator mode);
    bool setShading(MaterialId material, MaterialShading mode);
    bool setBlendMode(MaterialId material, MaterialBlend mode);
    bool setDepthTest(MaterialId material, bool enabled);
    bool setDetail(MaterialId material, bool enabled);
    bool setDetailBlendMode(MaterialId material, MaterialBlend mode);
    bool setDetailUv(MaterialId material, MaterialDetailUv uv);
    bool setBillboard(MaterialId material, MaterialBillboard mode, bool keepScale);
    bool setGrass(MaterialId material, bool enabled, float drawDistance, float cameraBend,
            const Math::Vec3& tintA = Math::Vec3(0.62f, 0.78f, 0.42f),
            const Math::Vec3& tintB = Math::Vec3(0.95f, 0.92f, 0.60f));
    bool setGrassWind(MaterialId material, const Math::Vec3& direction, float amplitude,
            float speed);
    bool setLeafWind(MaterialId material, bool enabled, float modelHeight, float strength = 1.0f,
            float speed = 1.0f, const Math::Vec3& direction = Math::Vec3(0.82f, 0.0f, 0.57f));
    bool setTerrainMorph(MaterialId material, bool enabled);
    bool setWater(MaterialId material, bool enabled,
            const WaterMaterialData& data = WaterMaterialData());
    bool setReceiveShadows(MaterialId material, bool receive);
    bool setBacklight(MaterialId material, const Math::Vec3& colour);
    bool setTerrainMaps(MaterialId material, const TerrainMaterialData& data);
    bool setOcean(MaterialId material, const OceanMaterialData& data);

    static std::uint32_t parameterCount();
    static const MaterialParamInfo& parameter(std::uint32_t index);
    static const MaterialParamInfo& parameter(MaterialParam param);
    static bool findParameter(const char* name, MaterialParam& out);

private:
    static MaterialStore* sSingleton;

    TextureStore& mTextures;
    ct::SlotMap32<MaterialInfo> mMaterials;
    MaterialId mDefault;
};

} // namespace prisma3d
