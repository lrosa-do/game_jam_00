#pragma once

#include <cstdint>

namespace prisma3d
{

constexpr std::uint32_t kMaxRenderViews = 2;

constexpr std::uint32_t kSceneFlagUseAmbientLight = 1u << 0;
constexpr std::uint32_t kSceneFlagUseAmbientCubemap = 1u << 1;
constexpr std::uint32_t kSceneFlagUseReflectionColor = 1u << 2;
constexpr std::uint32_t kSceneFlagUseReflectionCubemap = 1u << 3;
constexpr std::uint32_t kSceneFlagUseRoughnessLimiter = 1u << 4;
constexpr std::uint32_t kSceneFlagUseFog = 1u << 5;
constexpr std::uint32_t kSceneFlagUseUv2Material = 1u << 6;
constexpr std::uint32_t kSceneFlagUsePancakeShadows = 1u << 7;
constexpr std::uint32_t kSceneFlagInShadowPass = 1u << 8;
constexpr std::uint32_t kSceneFlagDebugNormal = 1u << 9;
constexpr std::uint32_t kSceneFlagUseLightmapSpecular = 1u << 10;

constexpr std::uint32_t kSceneConstantDepthFog = 1u << 6;

struct SceneUbo
{
    float projectionMatrix[16];
    float invProjectionMatrix[16];
    float invViewMatrix[12];
    float viewMatrix[12];

    float projectionMatrixView[kMaxRenderViews][16];
    float invProjectionMatrixView[kMaxRenderViews][16];
    float eyeOffset[kMaxRenderViews][4];

    float mainCamInvViewMatrix[16];

    float viewportSize[2];
    float screenPixelSize[2];

    float directionalPenumbraShadowKernel[128];
    float directionalSoftShadowKernel[128];
    float penumbraShadowKernel[128];
    float softShadowKernel[128];

    float shadowAtlasPixelSize[2];
    float directionalShadowPixelSize[2];

    float radiancePixelSize;
    float radianceBorderSize;
    float reflectionAtlasBorderSize[2];

    std::uint32_t directionalLightCount;
    float dualParaboloidSide;
    float zFar;
    float zNear;

    float roughnessLimiterAmount;
    float roughnessLimiterLimit;
    float opaquePrepassThreshold;
    std::uint32_t flags;

    float radianceInverseXform[12];

    float ambientLightColorEnergy[4];

    float reflectionColor[3];
    std::uint32_t packedConstants;

    float ambientColorSkyMix;
    float fogDensity;
    float fogHeight;
    float fogHeightDensity;

    float fogDepthCurve;
    float fogDepthBegin;
    float fogDepthEnd;
    float fogSunScatter;

    float fogLightColor[3];
    float fogAerialPerspective;

    float time;
    float taaFrameCount;
    float taaJitter[2];

    float emissiveExposureNormalization;
    float iblExposureNormalization;
    std::uint32_t cameraVisibleLayers;
    float passAlphaMultiplier;
};

struct SceneUboData
{
    SceneUbo data;
    SceneUbo prev;
};

static_assert(sizeof(SceneUbo) == 2864, "SceneUbo must match the std140 SceneData block");

struct InstanceDataGpu
{
    float transform[12];
    float compressedAabbPosition[4];
    float compressedAabbSize[3];
    std::uint32_t materialFeedbackIndex;
    float uvScale[4];
    std::uint32_t flags;
    std::uint32_t instanceUniformsOfs;
    std::uint32_t giOffset;
    std::uint32_t layerMask;
    float prevTransform[12];
    float lightmapUvScale[4];
};

static_assert(sizeof(InstanceDataGpu) == 176, "InstanceDataGpu must match the std430 InstanceData");

struct LightmapDataGpu
{
    float normalTransformAndSpecular[12];
    float textureSize[2];
    float exposureNormalization;
    std::uint32_t flags;
};

static_assert(sizeof(LightmapDataGpu) == 64, "LightmapDataGpu must match the std140 Lightmap");

struct MaterialDataGpu
{
    float albedo[4];
    float emission[4];
    float uv1ScaleOffset[4];
    float metallicChannel[4];
    float roughnessChannel[4];
    float aoChannel[4];
    float surface[4];
    float surface2[4];
    float billboard[4];
    float grass[4];
    float grassTintA[4];
    float grassTintB[4];
    float grassWind[4];
    float terrainUv0[4];
    float terrainUv1[4];
    float terrainParams[4];
    float uv2ScaleOffset[4];
    float detail[4];
    float backlight[4];
    float leafWind[4];
    float leafWindModel[4];
    float refraction[4];
    float refractionChannel[4];
    float water[4];
    float waterWind[4];
    float waterWaves[4];
    float waterWaves2[4];
    float waterDepth[4];
    float oceanColor[4];
    float oceanExtinction[4];
    float oceanParams[4];
    float oceanMisc[4];
};

static_assert(sizeof(MaterialDataGpu) == 512, "MaterialDataGpu must match the std140 block");

struct DrawCallData
{
    std::uint32_t instanceIndex;
    std::uint32_t pad[3];
    float color[4];
    std::uint32_t planar[4] = { 0, 0, 0, 0 };
};

struct PlanarReflectionsGpu
{
    float plane[4][4];
    float projection[4][16];
    float params[4][4];
};

static_assert(sizeof(PlanarReflectionsGpu) == 384,
        "PlanarReflectionsGpu must match the std140 block");

static_assert(sizeof(DrawCallData) == 48, "DrawCallData must match the std140 DrawCall block");

constexpr std::uint32_t kMaxDirectionalLights = 8;
constexpr std::uint32_t kMaxDecals = 512;
constexpr std::uint32_t kMaxReflectionProbes = 512;

struct LightDataGpu
{
    float position[3];
    float invRadius;
    float direction[3];
    float size;
    float color[3];
    float attenuation;
    float areaWidth[3];
    float coneAttenuation;
    float areaHeight[3];
    float coneAngle;
    float specularAmount;
    float shadowOpacity;
    float pad[2];
    float atlasRect[4];
    float shadowMatrix[16];
    float shadowBias;
    float shadowNormalBias;
    float transmittanceBias;
    float softShadowSize;
    float softShadowScale;
    std::uint32_t mask;
    float volumetricFogEnergy;
    std::uint32_t bakeMode;
    float projectorRect[4];
};

static_assert(sizeof(LightDataGpu) == 224, "LightDataGpu must match the std430 LightData");

struct DirectionalLightDataGpu
{
    float direction[3];
    float energy;
    float color[3];
    float size;
    float specular;
    std::uint32_t mask;
    float softshadowAngle;
    float softShadowScale;
    std::uint32_t blendSplits;
    float shadowOpacity;
    float fadeFrom;
    float fadeTo;
    std::uint32_t sscsIndex;
    std::uint32_t pad;
    std::uint32_t bakeMode;
    float volumetricFogEnergy;
    float shadowBias[4];
    float shadowNormalBias[4];
    float shadowTransmittanceBias[4];
    float shadowZRange[4];
    float shadowRangeBegin[4];
    float shadowSplitOffsets[4];
    float shadowMatrix[4][16];
    float uvScale[4][2];
};

static_assert(sizeof(DirectionalLightDataGpu) == 464,
        "DirectionalLightDataGpu must match the std140 DirectionalLightData");

struct LightCountsGpu
{
    std::uint32_t omni;
    std::uint32_t spot;
    std::uint32_t pad[2];
};

static_assert(sizeof(LightCountsGpu) == 16, "LightCountsGpu must match the std140 block");

struct ImplementationDataGpu
{
    std::uint32_t clusterShift;
    std::uint32_t clusterWidth;
    std::uint32_t clusterTypeSize;
    std::uint32_t maxClusterElementCountDiv32;
    std::uint32_t ssEffectsFlags;
    float ssaoLightAffect;
    float ssaoAoAffect;
    std::uint32_t pad1;
    float sdfToBounds[16];
    std::int32_t sdfOffset[3];
    std::uint32_t pad2;
    std::int32_t sdfSize[3];
    std::uint32_t giUpscaleForMsaa;
    std::uint32_t volumetricFogEnabled;
    float volumetricFogInvLength;
    float volumetricFogDetailSpread;
    std::uint32_t volumetricFogPad;
};

static_assert(sizeof(ImplementationDataGpu) == 144,
        "ImplementationDataGpu must match the std140 block");

constexpr std::uint32_t kMaxClusterElements = 512;

constexpr std::uint32_t kMaxSkyDirectionalLights = 4;

struct SkyParamsGpu
{
    float orientation[3][4];
    float projection[4];
    float position[3];
    float time;
    float borderSize[2];
    float luminanceMultiplier;
    float brightnessMultiplier;
};

static_assert(sizeof(SkyParamsGpu) == 96, "SkyParamsGpu must match the std140 SkyParams");

struct SkyDirectionalLightGpu
{
    float direction[3];
    float energy;
    float color[3];
    float size;
    std::uint32_t enabled;
    std::uint32_t pad[3];
};

static_assert(sizeof(SkyDirectionalLightGpu) == 48,
        "SkyDirectionalLightGpu must match the std140 DirectionalLightData");

struct SkyMaterialGpu
{
    float skyTopColor[4];
    float skyHorizonColor[4];
    float groundBottomColor[4];
    float groundHorizonColor[4];
    float invSkyCurve;
    float invGroundCurve;
    float sunAngleMax;
    float invSunCurve;
    float exposure;
    float pad[3];
};

static_assert(sizeof(SkyMaterialGpu) == 96, "SkyMaterialGpu must match the std140 block");

struct SkySceneDataGpu
{
    float combinedReprojection[2][16];
    float viewInvProjections[2][16];
    float viewEyeOffsets[2][4];

    std::uint32_t volumetricFogEnabled;
    float volumetricFogInvLength;
    float volumetricFogDetailSpread;
    float volumetricFogSkyAffect;

    std::uint32_t fogEnabled;
    float fogSkyAffect;
    float fogDensity;
    float fogSunScatter;

    float fogLightColor[3];
    float fogAerialPerspective;

    float zFar;
    std::uint32_t directionalLightCount;
    std::uint32_t fogUseLegacyBlending;
    std::uint32_t pad1;
};

static_assert(sizeof(SkySceneDataGpu) == 352, "SkySceneDataGpu must match the std140 SkySceneData");

constexpr std::uint32_t kRadianceLayers = 8;
constexpr std::uint32_t kRadianceSamples = 128;

struct OctmapDownsampleParamsGpu
{
    float borderSize;
    std::uint32_t size;
    std::uint32_t sourceLod;
    std::uint32_t sourceLayer;
};

static_assert(sizeof(OctmapDownsampleParamsGpu) == 16, "must match the std140 OctmapParams");

struct OctmapRoughnessParamsGpu
{
    std::uint32_t sampleCount;
    float roughness;
    std::uint32_t sourceSize;
    std::uint32_t destSize;
    float borderSize[2];
    std::uint32_t useDirectWrite;
    std::uint32_t pad;
};

static_assert(sizeof(OctmapRoughnessParamsGpu) == 32, "must match the std140 OctmapParams");

} // namespace prisma3d
