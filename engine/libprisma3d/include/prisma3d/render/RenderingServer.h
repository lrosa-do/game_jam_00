#pragma once

#include "prisma3d/render/DecalData.h"
#include "prisma3d/render/Impostor.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/ReflectionProbeData.h"
#include "prisma3d/render/TextureStore.h"

#include "mathc.h"

#include <ct/function.hpp>
#include <ct/slotmap32.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class BatchRenderer;

struct ScenarioTag;
struct InstanceTag;
struct CameraTag;
struct ViewportTag;
struct SkeletonTag;
class DecalAtlas;
struct LightTag;
struct DecalTag;
struct ReflectionProbeTag;
struct LightmapTag;
struct MultiMeshTag;
using ScenarioId = ct::Handle32<ScenarioTag>;
using InstanceId = ct::Handle32<InstanceTag>;
using CameraId = ct::Handle32<CameraTag>;
using ViewportId = ct::Handle32<ViewportTag>;
using SkeletonId = ct::Handle32<SkeletonTag>;
using LightId = ct::Handle32<LightTag>;
struct ReflectorTag;
using ReflectorId = ct::Handle32<ReflectorTag>;
using DecalId = ct::Handle32<DecalTag>;
using ReflectionProbeId = ct::Handle32<ReflectionProbeTag>;
using LightmapId = ct::Handle32<LightmapTag>;
using MultiMeshId = ct::Handle32<MultiMeshTag>;

enum class CameraProjection : std::uint8_t
{
    Perspective,
    Orthogonal
};

inline constexpr std::uint32_t kMaxLensFlareElements = 7;
inline constexpr std::uint32_t kMaxActiveReflectors = 4;

enum class LightType : std::uint8_t
{
    Directional,
    Omni,
    Spot
};

enum class LightParam : std::uint8_t
{
    Energy,
    Specular,
    Range,
    Size,
    Attenuation,
    SpotAngle,
    SpotAttenuation,
    ShadowBias,
    ShadowNormalBias,
    ShadowBlur,
    ShadowOpacity,
    ShadowMaxDistance,
    ShadowSplit1,
    ShadowSplit2,
    ShadowSplit3,
    ShadowFadeStart,
    ShadowPancakeSize,
    Count
};

inline constexpr float kLightParamDefaults[static_cast<std::size_t>(LightParam::Count)] = { 1.0f,
    0.5f, 5.0f, 0.0f, 1.0f, 45.0f, 1.0f, 0.1f, 2.0f, 1.0f, 1.0f, 100.0f, 0.1f, 0.2f, 0.5f, 0.8f,
    20.0f };

enum class LightBakeMode : std::uint8_t
{
    Disabled,
    Static,
    Dynamic
};

enum class ShadowmaskMode : std::uint8_t
{
    None,
    Replace,
    Overlay,
    Only
};

inline constexpr std::uint32_t kMaxLightmaps = 8;

enum class DirectionalShadowMode : std::uint8_t
{
    Orthogonal,
    Parallel2Splits,
    Parallel4Splits
};

enum class OmniShadowMode : std::uint8_t
{
    DualParaboloid,
    Cube
};

enum class ShadowQuality : std::uint8_t
{
    Hard,
    SoftVeryLow,
    SoftLow,
    SoftMedium,
    SoftHigh,
    SoftUltra
};

enum class BackgroundMode : std::uint8_t
{
    ClearColor,
    Color,
    Sky
};

enum class AmbientSource : std::uint8_t
{
    Background,
    Color,
    Disabled,
    Sky
};

enum class ReflectionSource : std::uint8_t
{
    Background,
    Disabled,
    Sky
};

struct ProceduralSkySettings
{
    Math::Vec3 skyTopColor = Math::Vec3(0.385f, 0.454f, 0.55f);
    Math::Vec3 skyHorizonColor = Math::Vec3(0.6463f, 0.6558f, 0.6708f);
    float skyCurve = 0.15f;
    float skyEnergy = 1.0f;
    Math::Vec3 groundBottomColor = Math::Vec3(0.2f, 0.169f, 0.133f);
    Math::Vec3 groundHorizonColor = Math::Vec3(0.6463f, 0.6558f, 0.6708f);
    float groundCurve = 0.02f;
    float groundEnergy = 1.0f;
    float sunAngleMax = 30.0f;
    float sunCurve = 0.15f;
    float energy = 1.0f;
    bool debanding = true;
    std::uint32_t radianceSize = 256;
};

enum class SkyMode : std::uint8_t
{
    Procedural,
    Panorama,
    Cubemap
};

enum class ViewportUpdateMode : std::uint8_t
{
    Disabled,
    Once,
    Always
};

enum class ViewportDebugDraw : std::uint8_t
{
    Disabled,
    Unshaded,
    NormalBuffer
};

enum class ViewportMsaa : std::uint8_t
{
    Disabled,
    Msaa2x,
    Msaa4x,
    Msaa8x
};

// How a 3D picture of a fixed size (viewportSetScaling3DSize) is fitted to the window.
enum class Scaling3DAspect : std::uint8_t
{
    // Stretched to the window; its aspect is lost.
    Stretch,
    // Its aspect kept, black bars fill the rest.
    Keep,
    // Its aspect kept, scaled until it covers the window; the edges that fall outside are cut.
    Crop,
    // Its lines (or columns, on a taller window) kept and the picture widened to the window's
    // aspect, so there are no bars and no distortion.
    Expand
};

// How the 3D picture is scaled to the window when it is not drawn at its size: one linear
// sample per pixel, or FSR 1.0 (EASU then RCAS, in compute).
enum class Scaling3DMode : std::uint8_t
{
    Bilinear,
    Fsr
};

// Where the 3D is drawn and where it lands in the window: the picture size in pixels and the
// rectangle it fills in window pixels (outside the window with Crop, inside with Keep).
struct Scaling3DLayout
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    float x = 0.0f;
    float y = 0.0f;
    float rectWidth = 0.0f;
    float rectHeight = 0.0f;
};

enum class TonemapMode : std::uint8_t
{
    Linear,
    Reinhard,
    Filmic,
    Aces,
    Agx
};

enum class FogMode : std::uint8_t
{
    Exponential,
    Depth
};

enum class SsaoQuality : std::uint8_t
{
    VeryLow,
    Low,
    Medium,
    High,
    Ultra
};

enum class DofBokehShape : std::uint8_t
{
    Box,
    Hexagon,
    Circle
};

enum class DofBlurQuality : std::uint8_t
{
    VeryLow,
    Low,
    Medium,
    High
};

struct CameraAttributesSettings
{
    bool dofBlurFarEnabled = false;
    float dofBlurFarDistance = 10.0f;
    float dofBlurFarTransition = 5.0f;
    bool dofBlurNearEnabled = false;
    float dofBlurNearDistance = 2.0f;
    float dofBlurNearTransition = 1.0f;
    float dofBlurAmount = 0.1f;

    bool usesDof() const
    {
        return (dofBlurNearEnabled || dofBlurFarEnabled) && dofBlurAmount > 0.0f;
    }
};

enum class GlowBlendMode : std::uint8_t
{
    Additive,
    Screen,
    SoftLight,
    Replace,
    Mix
};

struct EnvironmentSettings
{
    BackgroundMode background = BackgroundMode::ClearColor;
    Math::Vec3 backgroundColor = Math::Vec3(0.0f, 0.0f, 0.0f);
    float backgroundEnergy = 1.0f;
    AmbientSource ambientSource = AmbientSource::Background;
    Math::Vec3 ambientColor = Math::Vec3(0.0f, 0.0f, 0.0f);
    float ambientEnergy = 1.0f;
    ReflectionSource reflectionSource = ReflectionSource::Background;
    float ambientSkyContribution = 1.0f;
    ProceduralSkySettings sky;
    SkyMode skyMode = SkyMode::Procedural;
    TextureId skyTexture;
    float skyTextureEnergy = 1.0f;
    float tonemapExposure = 1.0f;
    float tonemapWhite = 1.0f;
    TonemapMode tonemapMode = TonemapMode::Linear;
    float tonemapAgxContrast = 1.25f;
    bool fogEnabled = false;
    FogMode fogMode = FogMode::Exponential;
    Math::Vec3 fogLightColor = Math::Vec3(0.518f, 0.553f, 0.608f);
    float fogLightEnergy = 1.0f;
    float fogSunScatter = 0.0f;
    float fogDensity = 0.01f;
    float fogAerialPerspective = 0.0f;
    float fogSkyAffect = 1.0f;
    float fogHeight = 0.0f;
    float fogHeightDensity = 0.0f;
    float fogDepthCurve = 1.0f;
    float fogDepthBegin = 10.0f;
    float fogDepthEnd = 100.0f;
    bool glowEnabled = false;
    float glowLevels[7] = { 0.0f, 0.8f, 0.4f, 0.1f, 0.0f, 0.0f, 0.0f };
    bool glowNormalized = false;
    float glowIntensity = 0.3f;
    float glowStrength = 1.0f;
    float glowMix = 0.05f;
    float glowBloom = 0.0f;
    GlowBlendMode glowBlendMode = GlowBlendMode::Screen;
    float glowHdrBleedThreshold = 1.0f;
    float glowHdrBleedScale = 2.0f;
    float glowHdrLuminanceCap = 12.0f;
    float glowMapStrength = 0.8f;
    TextureId glowMap;
    bool ssaoEnabled = false;
    float ssaoRadius = 1.0f;
    float ssaoIntensity = 2.0f;
    float ssaoPower = 1.5f;
    float ssaoDetail = 0.5f;
    float ssaoHorizon = 0.06f;
    float ssaoSharpness = 0.98f;
    float ssaoLightAffect = 0.0f;
    float ssaoAoChannelAffect = 0.0f;
    bool ssrEnabled = false;
    std::int32_t ssrMaxSteps = 64;
    float ssrFadeIn = 0.15f;
    float ssrFadeOut = 2.0f;
    float ssrDepthTolerance = 0.5f;
    CameraAttributesSettings cameraAttributes;
    bool adjustmentEnabled = false;
    float adjustmentBrightness = 1.0f;
    float adjustmentContrast = 1.0f;
    float adjustmentSaturation = 1.0f;
    TextureId adjustmentColorCorrection;
};

struct RenderStats
{
    std::uint32_t viewportsDrawn = 0;
    std::uint32_t instancesCulled = 0;
    std::uint32_t visibilityRangeHidden = 0;
    std::uint32_t lodSurfaces = 0;
    std::uint32_t instancesDrawn = 0;
    std::uint32_t multimeshInstancesDrawn = 0;
    std::uint32_t multimeshUploadBytes = 0;
    std::uint32_t surfacesDrawn = 0;
    std::uint32_t drawCalls = 0;
    std::uint32_t cullEntries = 0;
    std::uint32_t cullNodes = 0;
    std::uint32_t cullDepth = 0;
    std::uint32_t cullNodesVisited = 0;
    std::uint32_t cullCandidates = 0;
    std::uint32_t cullAccepted = 0;
    float cullQuality = 0.0f;
    bool cullRebuildPending = false;
    bool cullTree = false;
    std::uint32_t triangles = 0;
    std::uint32_t skinnedSurfaces = 0;
    std::uint32_t skinDispatches = 0;
    std::uint32_t targetsCreated = 0;
    std::uint32_t pipelinesBuilt = 0;
    float pipelineBuildMilliseconds = 0.0f;
    std::uint32_t glowPasses = 0;
    std::uint32_t glowTargetsCreated = 0;
    std::uint32_t glowTexturesLive = 0;
    std::uint32_t ssaoDispatches = 0;
    std::uint32_t ssaoTargetsCreated = 0;
    std::uint32_t ssaoTexturesLive = 0;
    std::uint32_t normalRoughnessDrawCalls = 0;
    std::uint32_t normalRoughnessTexturesLive = 0;
    std::uint32_t ssrDispatches = 0;
    std::uint32_t ssrTargetsCreated = 0;
    std::uint32_t ssrTexturesLive = 0;
    std::uint32_t dofDispatches = 0;
    std::uint32_t dofDraws = 0;
    std::uint32_t dofTargetsCreated = 0;
    std::uint32_t dofTexturesLive = 0;
    std::uint32_t lightsDrawn = 0;
    std::uint32_t lensFlaresDrawn = 0;
    std::uint32_t screenCopies = 0;
    std::uint32_t reflectorsDrawn = 0;
    std::uint32_t decalsDrawn = 0;
    std::uint32_t reflectionProbesDrawn = 0;
    std::uint32_t lightmapsUsed = 0;
    std::uint32_t lightmapSurfaces = 0;
    std::uint32_t shadowCascades = 0;
    std::uint32_t shadowLights = 0;
    std::uint32_t shadowPasses = 0;
    std::uint32_t shadowCasters = 0;
    std::uint32_t shadowDrawCalls = 0;
    std::uint32_t shadowTriangles = 0;
    std::uint32_t depthPrePassSurfaces = 0;
    std::uint32_t depthPrePassDrawCalls = 0;
    std::uint32_t depthPrePassTriangles = 0;
    std::uint32_t coverageSurfaces = 0;
    std::uint32_t batchDrawCalls = 0;
    std::uint32_t batchVertices = 0;
    std::uint32_t impostorSets = 0;
    std::uint32_t impostorDrawCalls = 0;
};

class RenderingServer
{
public:
    RenderingServer(prisma::Driver* driver, MeshStore& meshes, TextureStore& textures,
            MaterialStore& materials);
    ~RenderingServer();
    RenderingServer(const RenderingServer&) = delete;
    RenderingServer& operator=(const RenderingServer&) = delete;

    static RenderingServer* singleton();
    prisma::Driver* driver() const;
    MeshStore& meshes();
    TextureStore& textures();
    MaterialStore& materials();
    BatchRenderer& batch();

    ScenarioId scenarioCreate();
    void scenarioFree(ScenarioId scenario);
    void scenarioSetEnvironment(ScenarioId scenario, const EnvironmentSettings& environment);
    void scenarioClearEnvironment(ScenarioId scenario);

    LightId lightCreate(LightType type);
    void lightSetScenario(LightId light, ScenarioId scenario);
    void lightSetTransform(LightId light, const Math::Mat4& transform);
    void lightSetColor(LightId light, const Math::Vec3& color);
    void lightSetParam(LightId light, LightParam param, float value);
    void lightSetCullMask(LightId light, std::uint32_t mask);
    void lightSetNegative(LightId light, bool negative);
    void lightSetVisible(LightId light, bool visible);
    void lightSetShadow(LightId light, bool enabled);
    void lightSetDirectionalShadowMode(LightId light, DirectionalShadowMode mode);
    void lightSetBlendSplits(LightId light, bool blend);
    void lightSetOmniShadowMode(LightId light, OmniShadowMode mode);
    void lightSetLensFlare(LightId light, const TextureId* textures, std::uint32_t count);
    void lightSetBakeMode(LightId light, LightBakeMode mode);

    ReflectorId reflectorCreate(ScenarioId scenario);
    void reflectorSetScenario(ReflectorId reflector, ScenarioId scenario);
    void reflectorSetTransform(ReflectorId reflector, const Math::Mat4& transform);
    void reflectorSetHalfSize(ReflectorId reflector, const Math::Vec2& halfSize);
    void reflectorSetResolution(ReflectorId reflector, std::uint32_t resolution);
    void reflectorSetMaxDistance(ReflectorId reflector, float distance);
    void reflectorSetVisible(ReflectorId reflector, bool visible);
    TextureId reflectorGetTexture(ReflectorId reflector) const;
    void reflectorFree(ReflectorId reflector);
    void instanceSetReflector(InstanceId instance, ReflectorId reflector);
    void lightFree(LightId light);

    DecalId decalCreate();
    void decalSetScenario(DecalId decal, ScenarioId scenario);
    void decalSetTransform(DecalId decal, const Math::Mat4& transform);
    void decalSetVisible(DecalId decal, bool visible);
    void decalSetSize(DecalId decal, const Math::Vec3& size);
    void decalSetTexture(DecalId decal, DecalTexture type, TextureId texture);
    void decalSetEmissionEnergy(DecalId decal, float energy);
    void decalSetAlbedoMix(DecalId decal, float mix);
    void decalSetModulate(DecalId decal, const Math::Vec4& modulate);
    void decalSetCullMask(DecalId decal, std::uint32_t mask);
    void decalSetDistanceFade(DecalId decal, bool enabled, float begin, float length);
    void decalSetFade(DecalId decal, float above, float below);
    void decalSetNormalFade(DecalId decal, float fade);
    void decalSetSortingOffset(DecalId decal, float offset);
    void decalFree(DecalId decal);
    const DecalParams* decalGetParams(DecalId decal) const;
    const DecalAtlas& getDecalAtlas() const;
    std::uint32_t getDecalFrameCount() const;
    const DecalDataGpu* getDecalFrameData() const;

    ReflectionProbeId reflectionProbeCreate();
    void reflectionProbeSetScenario(ReflectionProbeId probe, ScenarioId scenario);
    void reflectionProbeSetTransform(ReflectionProbeId probe, const Math::Mat4& transform);
    void reflectionProbeSetVisible(ReflectionProbeId probe, bool visible);
    void reflectionProbeSetParams(ReflectionProbeId probe, const ReflectionProbeParams& params);
    void reflectionProbeFree(ReflectionProbeId probe);
    const ReflectionProbeParams* reflectionProbeGetParams(ReflectionProbeId probe) const;
    std::uint32_t getReflectionProbeFrameCount() const;
    const ReflectionDataGpu* getReflectionProbeFrameData() const;

    LightmapId lightmapCreate();
    void lightmapSetScenario(LightmapId lightmap, ScenarioId scenario);
    void lightmapSetTransform(LightmapId lightmap, const Math::Mat4& transform);
    void lightmapSetVisible(LightmapId lightmap, bool visible);
    void lightmapSetTextures(LightmapId lightmap, TextureId light, bool usesSphericalHarmonics);
    void lightmapSetShadowmaskTextures(LightmapId lightmap, TextureId shadowmask);
    void lightmapSetShadowmaskMode(LightmapId lightmap, ShadowmaskMode mode);
    void lightmapSetSpecularIntensity(LightmapId lightmap, float intensity);
    void lightmapSetBakedExposureNormalization(LightmapId lightmap, float exposure);
    void lightmapSetBounds(LightmapId lightmap, const Math::Box& bounds);
    void lightmapSetProbeGrid(LightmapId lightmap, const Math::Vec3& origin, const Math::Vec3& cell,
            const std::uint32_t dims[3], const float* sh);
    void lightmapFree(LightmapId lightmap);
    void instanceGeometrySetTransparency(InstanceId instance, float transparency);
    void instanceGeometrySetLightmap(InstanceId instance, LightmapId lightmap,
            const Math::Vec4& uvScale, int sliceIndex);
    void setLightmapBicubicFilter(bool enabled);
    bool getLightmapBicubicFilter() const;

    InstanceId instanceCreate();
    InstanceId instanceCreate(MeshId base, ScenarioId scenario);
    void instanceSetBase(InstanceId instance, MeshId base);
    void instanceSetBase(InstanceId instance, MultiMeshId base);
    void instanceSetScenario(InstanceId instance, ScenarioId scenario);
    void instanceSetTransform(InstanceId instance, const Math::Mat4& transform);
    void instanceSetVisible(InstanceId instance, bool visible);
    void instanceSetLayerMask(InstanceId instance, std::uint32_t mask);
    void instanceSetVisibilityRange(InstanceId instance, float begin, float end, float beginMargin,
            float endMargin);
    void instanceSetLodBias(InstanceId instance, float bias);
    void instanceSetColor(InstanceId instance, const Math::Vec4& color);
    void instanceSetCastShadows(InstanceId instance, bool cast);
    void instanceSetSurfaceMaterial(InstanceId instance, std::uint32_t surface,
            MaterialId material);
    void instanceSetMaterialOverride(InstanceId instance, MaterialId material);
    void instanceSetSurfaceVisible(InstanceId instance, std::uint32_t surface, bool visible);
    MaterialId instanceResolveMaterial(InstanceId instance, std::uint32_t surface) const;
    void instanceAttachSkeleton(InstanceId instance, SkeletonId skeleton);
    SkeletonId instanceGetSkeleton(InstanceId instance) const;
    prisma::BufferHandle instanceSkinnedVertexBuffer(InstanceId instance,
            std::uint32_t surface) const;
    void instanceFree(InstanceId instance);
    Math::Box instanceGetAabb(InstanceId instance) const;

    MultiMeshId multimeshCreate();
    void multimeshAllocateData(MultiMeshId multimesh, std::uint32_t instances, bool useColors,
            bool useCustomData);
    std::uint32_t multimeshGetInstanceCount(MultiMeshId multimesh) const;
    void multimeshSetMesh(MultiMeshId multimesh, MeshId mesh);
    MeshId multimeshGetMesh(MultiMeshId multimesh) const;
    void multimeshInstanceSetTransform(MultiMeshId multimesh, std::uint32_t index,
            const Math::Mat4& transform);
    void multimeshInstanceSetColor(MultiMeshId multimesh, std::uint32_t index,
            const Math::Vec4& color);
    void multimeshInstanceSetCustomData(MultiMeshId multimesh, std::uint32_t index,
            const Math::Vec4& data);
    Math::Mat4 multimeshInstanceGetTransform(MultiMeshId multimesh, std::uint32_t index);
    Math::Vec4 multimeshInstanceGetColor(MultiMeshId multimesh, std::uint32_t index);
    Math::Vec4 multimeshInstanceGetCustomData(MultiMeshId multimesh, std::uint32_t index);
    bool multimeshSetBuffer(MultiMeshId multimesh, const float* data, std::size_t count);
    bool multimeshGetBuffer(MultiMeshId multimesh, ct::Vector<float>& out);
    void multimeshSetVisibleInstances(MultiMeshId multimesh, int visible);
    int multimeshGetVisibleInstances(MultiMeshId multimesh) const;
    void multimeshSetDissolve(MultiMeshId multimesh, bool enable);
    bool multimeshGetDissolve(MultiMeshId multimesh) const;
    void multimeshSetCustomAabb(MultiMeshId multimesh, const Math::Box& aabb);
    Math::Box multimeshGetCustomAabb(MultiMeshId multimesh) const;
    Math::Box multimeshGetAabb(MultiMeshId multimesh);
    bool multimeshValid(MultiMeshId multimesh) const;
    void multimeshFree(MultiMeshId multimesh);

    ImpostorId impostorCreate(MeshId mesh, const ct::Vector<MaterialId>& surfaceMaterials,
            float swapInDistance);
    void impostorFree(ImpostorId impostor);
    void impostorSetSwapInDistance(ImpostorId impostor, float distance);
    float impostorGetSwapInDistance(ImpostorId impostor) const;
    void impostorSetDirty(ImpostorId impostor);
    int impostorGetSlot(ImpostorId impostor) const;
    bool impostorIsCaptured(ImpostorId impostor) const;
    ImpostorSetId impostorSetCreate(ScenarioId scenario);
    void impostorSetFree(ImpostorSetId set);
    void impostorSetResize(ImpostorSetId set, std::uint32_t count);
    std::uint32_t impostorSetGetCount(ImpostorSetId set) const;
    void impostorSetInstance(ImpostorSetId set, std::uint32_t index, ImpostorId impostor,
            const Math::Mat4& transform, const Math::Vec4& color);
    bool impostorSetGetInstance(ImpostorSetId set, std::uint32_t index,
            ImpostorInstanceInfo& info) const;
    void impostorSetEnabled(ImpostorSetId set, bool enabled);
    bool impostorSetReadCards(ImpostorSetId set, std::uint32_t& indexCount,
            ct::Vector<ImpostorCardReadback>& cards);
    std::uint32_t impostorRefresh();
    bool impostorReadSlice(ImpostorId impostor, std::uint32_t angle, ImpostorProperty property,
            ct::Vector<std::uint8_t>& rgba);
    std::uint32_t impostorCaptureSamples() const;
    std::uint64_t impostorTextureBytes() const;

    SkeletonId skeletonCreate();
    void skeletonAllocateData(SkeletonId skeleton, std::uint32_t bones);
    std::uint32_t skeletonGetBoneCount(SkeletonId skeleton) const;
    void skeletonBoneSetTransform(SkeletonId skeleton, std::uint32_t bone,
            const Math::Mat4& transform);
    Math::Mat4 skeletonBoneGetTransform(SkeletonId skeleton, std::uint32_t bone) const;
    void skeletonFree(SkeletonId skeleton);

    CameraId cameraCreate();
    void cameraSetPerspective(CameraId camera, float fovyDegrees, float zNear, float zFar);
    void cameraSetOrthogonal(CameraId camera, float size, float zNear, float zFar);
    void cameraSetTransform(CameraId camera, const Math::Mat4& transform);
    void cameraSetCustomProjection(CameraId camera, const Math::Mat4& projection);
    void cameraClearCustomProjection(CameraId camera);
    void cameraSetCullMask(CameraId camera, std::uint32_t mask);
    void cameraSetAttributes(CameraId camera, const CameraAttributesSettings& attributes);
    void cameraClearAttributes(CameraId camera);
    void cameraFree(CameraId camera);

    ViewportId viewportCreate();
    void viewportSetSize(ViewportId viewport, std::uint32_t width, std::uint32_t height);
    void viewportAttachToScreen(ViewportId viewport, bool attach);
    void viewportSetActive(ViewportId viewport, bool active);
    void viewportAttachCamera(ViewportId viewport, CameraId camera);
    void viewportSetScenario(ViewportId viewport, ScenarioId scenario);
    void viewportSetClearColor(ViewportId viewport, const Math::Vec4& color);
    void viewportSetTransparentBackground(ViewportId viewport, bool enabled);
    bool viewportGetTransparentBackground(ViewportId viewport) const;
    void viewportSetUpdateMode(ViewportId viewport, ViewportUpdateMode mode);
    ViewportUpdateMode viewportGetUpdateMode(ViewportId viewport) const;
    void viewportSetSrgbOutput(ViewportId viewport, bool enabled);
    bool viewportGetSrgbOutput(ViewportId viewport) const;
    void viewportSetDrawBatch(ViewportId viewport, bool enabled);
    bool viewportGetDrawBatch(ViewportId viewport) const;
    void viewportSetDebugDraw(ViewportId viewport, ViewportDebugDraw mode);
    ViewportDebugDraw viewportGetDebugDraw(ViewportId viewport) const;
    void viewportSetMsaa3D(ViewportId viewport, ViewportMsaa msaa);
    void viewportSetScaling3D(ViewportId viewport, float scale);
    float viewportGetScaling3D(ViewportId viewport) const;
    // A fixed size for the 3D picture (0 in either goes back to the scale), fitted to the
    // window by the aspect mode.
    void viewportSetScaling3DSize(ViewportId viewport, std::uint32_t width, std::uint32_t height);
    void viewportGetScaling3DSize(ViewportId viewport, std::uint32_t& width,
            std::uint32_t& height) const;
    void viewportSetScaling3DAspect(ViewportId viewport, Scaling3DAspect aspect);
    Scaling3DAspect viewportGetScaling3DAspect(ViewportId viewport) const;
    void viewportSetScaling3DMode(ViewportId viewport, Scaling3DMode mode);
    Scaling3DMode viewportGetScaling3DMode(ViewportId viewport) const;
    // FSR sharpness in stops: 0 the sharpest, 2 the softest.
    void viewportSetFsrSharpness(ViewportId viewport, float sharpness);
    float viewportGetFsrSharpness(ViewportId viewport) const;
    static Scaling3DLayout scaling3DLayout(std::uint32_t outputWidth, std::uint32_t outputHeight,
            float scale, std::uint32_t sizeWidth, std::uint32_t sizeHeight, Scaling3DAspect aspect);
    // FXAA on the final image (on the 3D resolution when it is scaled): smooths the jagged edges
    // for about the cost of one more full screen pass.
    void viewportSetFxaa(ViewportId viewport, bool enabled);
    bool viewportGetFxaa(ViewportId viewport) const;
    // SMAA on the tonemapped picture (edges, blending weights, blending: three passes at the 3D
    // resolution) instead of FXAA.
    void viewportSetSmaa(ViewportId viewport, bool enabled);
    bool viewportGetSmaa(ViewportId viewport) const;
    // The contrast between neighbouring pixels that SMAA takes for an edge (0.01 .. 0.2).
    void setSmaaEdgeDetectionThreshold(float threshold);
    float getSmaaEdgeDetectionThreshold() const;
    void viewportSetMeshLodThreshold(ViewportId viewport, float pixels);
    float viewportGetMeshLodThreshold(ViewportId viewport) const;
    ViewportMsaa viewportGetMsaa3D(ViewportId viewport) const;
    std::uint32_t viewportGetMsaaSamples(ViewportId viewport) const;
    TextureId viewportGetTexture(ViewportId viewport);
    bool viewportRender(ViewportId viewport);
    bool viewportReadDepth(ViewportId viewport, int x, int y, float& depth);
    prisma::TextureHandle viewportGetAmbientOcclusion(ViewportId viewport) const;
    prisma::TextureHandle viewportGetNormalRoughness(ViewportId viewport) const;
    prisma::TextureHandle viewportGetReflections(ViewportId viewport) const;
    void viewportSetPositionalShadowAtlasSize(ViewportId viewport, std::uint32_t size,
            bool sixteenBits);
    void viewportSetPositionalShadowAtlasQuadrantSubdivision(ViewportId viewport,
            std::uint32_t quadrant, std::uint32_t subdivision);
    bool viewportGetPositionalShadowSlot(ViewportId viewport, LightId light,
            std::uint32_t& quadrant, std::uint32_t& index, bool& omni) const;
    void viewportFree(ViewportId viewport);

    void setDirectionalShadowSize(std::uint32_t size, bool sixteenBits);
    void setDirectionalShadowQuality(ShadowQuality quality);
    void setPositionalShadowQuality(ShadowQuality quality);
    std::uint32_t directionalShadowSize() const;
    bool directionalShadowSixteenBits() const;
    ShadowQuality directionalShadowQuality() const;
    ShadowQuality positionalShadowQuality() const;

    void setRoughnessLimiter(bool enabled, float amount = 0.25f, float limit = 0.18f);
    bool roughnessLimiterEnabled() const;
    float roughnessLimiterAmount() const;
    float roughnessLimiterLimit() const;
    void setDepthPrePassEnabled(bool enabled);
    bool depthPrePassEnabled() const;

    void setSsaoQuality(SsaoQuality quality, bool halfSize, float adaptiveTarget,
            std::uint32_t blurPasses, float fadeoutFrom, float fadeoutTo);
    SsaoQuality ssaoQuality() const;
    bool ssaoHalfSize() const;
    float ssaoAdaptiveTarget() const;
    std::uint32_t ssaoBlurPasses() const;
    float ssaoFadeoutFrom() const;
    float ssaoFadeoutTo() const;
    void setSsrHalfSize(bool halfSize);
    bool ssrHalfSize() const;
    void setDepthOfFieldBokeh(DofBokehShape shape, DofBlurQuality quality, bool useJitter);
    DofBokehShape depthOfFieldShape() const;
    DofBlurQuality depthOfFieldQuality() const;
    bool depthOfFieldJitter() const;
    void setDepthOfFieldComputeAllowed(bool allowed);
    bool depthOfFieldComputeAllowed() const;
    bool screenSpaceEffectsSupported() const;
    void setScreenSpaceEffectsAllowed(bool allowed);
    bool screenSpaceEffectsAllowed() const;

    std::uint32_t scenePipelineCount() const;

    void setCullingTreeEnabled(bool enabled);
    bool cullingTreeEnabled() const;
    // After the first frame, scene pipelines are built off the main thread where the driver can
    // and their surfaces are not drawn until they are ready. Off makes every build block.
    void setDeferredPipelinesEnabled(bool enabled);
    bool deferredPipelinesEnabled() const;
    void setCullingLeafCapacity(std::uint32_t capacity);
    std::uint32_t cullingLeafCapacity() const;

    void framePreDrawConnect(void* owner, ct::Function<void()> callback);
    void framePreDrawDisconnect(void* owner);
    void draw(bool present, double frameStep);
    void renderViewports();
    void endFrame(bool present);
    bool isFrameOpen() const;
    void openFrame();
    float time() const;
    const RenderStats& getStats() const;

    static Math::Mat4 perspective(float fovyDegrees, float aspect, float zNear, float zFar);
    static Math::Mat4 orthogonal(float size, float aspect, float zNear, float zFar);

private:
    struct Scenario;
    struct Instance;
    struct Camera;
    struct Viewport;
    struct Skeleton;
    struct Light;
    struct Reflector;
    struct Decal;
    struct ReflectionProbe;
    struct Lightmap;
    struct MeshInstance;
    struct State;

    static RenderingServer* sSingleton;

    void drawViewport(Viewport& viewport);
    void renderReflectors(Viewport& viewport);
    void updateSkeletons();
    void updateMeshInstances();
    Math::Box meshAabb(const MeshInfo& mesh, SkeletonId skeleton) const;

    prisma::Driver* mDriver;
    MeshStore& mMeshes;
    TextureStore& mTextures;
    MaterialStore& mMaterials;
    State* mState;
    RenderStats mStats;
    bool mGpuFrameScope;
    bool mFrameOpen;
};

} // namespace prisma3d
