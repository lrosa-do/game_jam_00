#include "prisma3d/render/RenderingServer.h"
#include "SmaaTextures.h"
#include "prisma3d/render/ShaderVariants.h"
#include "stb_image.h"

#include "DepthOfField.h"
#include "ImpostorManager.h"
#include "MultiMeshStorage.h"
#include "SceneData.h"
#include "SceneMaterial.h"
#include "ScreenSpaceEffects.h"
#include "prisma3d/Device.h"
#include "prisma3d/core/BoundsTree.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/render/ClusterBuilder.h"
#include "prisma3d/render/DecalAtlas.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/ShadowAtlas.h"

#include "platform.h"

#include <math.h>

#include <ct/hashmap.hpp>
#include <ct/ptr.hpp>
#include <ct/sort.hpp>
#include <ct/thread.hpp>
#include <ct/threadpool.hpp>

#include <string.h>

namespace prisma3d
{

namespace
{

const std::uint32_t kBlockSize = sizeof(DrawCallData);
const std::uint32_t kSkinParamsSize = 32;
const std::uint32_t kSkinGroupSize = 64;
const std::uint32_t kBoneFloats = 12;
const float kDegreesToRadians = 0.017453292519943295f;

template<typename Map, typename Id>
typename Map::value_type* find(Map& map, Id id)
{
    return map.get(Map::handle_type::from_bits(id.bits()));
}

template<typename Map, typename Id>
const typename Map::value_type* find(const Map& map, Id id)
{
    return map.get(Map::handle_type::from_bits(id.bits()));
}

template<typename Map, typename Id>
bool erase(Map& map, Id id)
{
    return map.erase(Map::handle_type::from_bits(id.bits()));
}

template<typename Id, typename Key>
Id toId(Key key)
{
    return Id::from_bits(key.bits());
}

Math::Mat4 zeroToOneDepth()
{
    Math::Mat4 matrix = Math::Mat4::Identity();
    matrix.col2.z = 0.5f;
    matrix.col3.z = 0.5f;
    return matrix;
}

Math::Mat4 basePerspective(float fovyDegrees, float aspect, float zNear, float zFar)
{
    return Math::Mat4::Perspective(fovyDegrees * kDegreesToRadians, aspect, zNear, zFar);
}

Math::Mat4 baseOrthogonal(float size, float aspect, float zNear, float zFar)
{
    const float halfWidth = size * aspect * 0.5f;
    const float halfHeight = size * 0.5f;
    return Math::Mat4::Ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, zNear, zFar);
}

Math::Box transformBox(const Math::Box& box, const Math::Mat4& transform)
{
    Math::Vec3 corners[8];
    box.GetCorners(corners);
    Math::Box result;
    for (int i = 0; i < 8; ++i) result = result.Expanded(transform.TransformPoint(corners[i]));
    return result;
}

bool usableMesh(const MeshInfo* mesh)
{
    return mesh != nullptr && !mesh->surfaces.empty() && !emptyBox(mesh->aabb);
}

std::uint32_t alignUp(std::uint32_t value, std::uint32_t alignment)
{
    if (alignment <= 1) return value;
    return (value + alignment - 1) / alignment * alignment;
}

std::uint32_t triangleCount(prisma::Topology topology, std::uint32_t count)
{
    if (topology == prisma::Topology::Triangles) return count / 3;
    if (topology == prisma::Topology::TriangleStrip) return count >= 3 ? count - 2 : 0;
    return 0;
}

constexpr std::uint32_t kMaxRepeats = (1u << 20) - 1;

struct RenderItem
{
    std::uint64_t sortKey1 = 0;
    std::uint64_t sortKey2 = 0;
    std::uint64_t pipelineKey = 0;
    std::uint32_t variant = 0;
    bool noShadows = false;
    const SurfaceInfo* surface = nullptr;
    prisma::BufferHandle vertexBuffer;
    prisma::BufferHandle indexBuffer;
    std::uint32_t indexCount = 0;
    std::uint32_t lod = 0;
    bool skinned = false;
    Math::Mat4 transform;
    Math::Vec4 color;
    MaterialId material;
    std::uint32_t layerMask = 1;
    MaterialCull cull = MaterialCull::Back;
    MaterialBlend blend = MaterialBlend::Mix;
    bool depthTest = true;
    bool transparent = false;
    bool coverage = false;
    bool prepass = false;
    bool fadeDepth = false;
    bool dissolve = false;
    float fade = 1.0f;
    std::uint32_t multimeshMode = 0;
    std::uint32_t billboard = 0;
    std::uint32_t reflectorSlot = 0;
    bool lightmap = false;
    std::uint32_t lightmapIndex = 0;
    std::uint32_t lightmapFlags = 0;
    std::uint32_t giOffset = 0;
    float lightmapUvScale[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    prisma::BufferHandle transforms;
    std::uint32_t transformBytes = 0;
    std::uint32_t instanceCount = 1;
};

struct SortEntry
{
    std::uint64_t key2;
    std::uint64_t key1;
    std::uint32_t index;
};

struct DrawRun
{
    std::uint32_t first;
    std::uint32_t count;
};

bool sortEntryLess(const SortEntry& a, const SortEntry& b)
{
    if (a.key2 != b.key2) return a.key2 < b.key2;
    return a.key1 < b.key1;
}

bool sameColor(const Math::Vec4& a, const Math::Vec4& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

void removeInstance(ct::Vector<InstanceId>& list, InstanceId instance)
{
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        if (list[i] == instance)
        {
            list[i] = list.back();
            list.pop_back();
            return;
        }
    }
}

} // namespace

struct RenderingServer::Scenario
{
    struct BuildJob
    {
        ct::Atomic<std::uint32_t> done;
        ct::Vector<Math::Box> bounds;
        BoundsTree* target = nullptr;
    };

    Scenario()
            : tree(ct::make_unique<BoundsTree>()),
              treeNext(ct::make_unique<BoundsTree>()),
              job(ct::make_unique<BuildJob>())
    {
    }

    struct Radiance
    {
        prisma::TextureHandle array;
        prisma::TextureHandle downsampled;
        std::uint32_t size = 0;
        std::uint32_t width = 0;
        std::uint32_t mips = 0;
        float border = 0.0f;
        bool dirty = true;
        std::uint32_t processing = 0;
        ct::Vector<std::uint8_t> signature;
    };

    ct::Vector<InstanceId> instances;
    Radiance radiance;
    EnvironmentSettings environment;
    std::uint32_t skyMode = 0;
    prisma::TextureHandle skyGpu;
    bool hasEnvironment = false;
    ct::Unique<BoundsTree> tree;
    ct::Unique<BoundsTree> treeNext;
    ct::Unique<BuildJob> job;
    ct::Vector<InstanceId> entries;
    ct::Vector<std::uint32_t> entrySurfaces;
    ct::Vector<Math::Box> bounds;
    ct::Vector<std::uint32_t> meshVersions;
    ct::HashMap<std::uint32_t, std::uint32_t> entryIndex;
    ct::HashMap<std::uint32_t, std::uint32_t> maskCounts;
    ct::Vector<InstanceId> waiting;
    ct::Vector<InstanceId> dirty;
    ct::Vector<std::uint32_t> candidates;
    std::uint64_t usedRangeBits = 0;
    bool indexDirty = true;
    bool buildPending = false;
    bool boundsChanged = false;
    std::uint64_t updatedFrame = 0;
};

struct RenderingServer::Instance
{
    MeshId base;
    MultiMeshId multimesh;
    ScenarioId scenario;
    Math::Mat4 transform = Math::Mat4::Identity();
    Math::Vec4 color = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
    MaterialId materialOverride;
    ct::Vector<MaterialId> surfaceMaterials;
    std::uint32_t layerMask = 1;
    bool visible = true;
    bool castShadows = true;
    bool queued = false;
    SkeletonId skeleton;
    ReflectorId reflector;
    LightmapId lightmap;
    Math::Vec4 lightmapUvScale = Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
    std::int32_t lightmapSlice = 0;
    float transparency = 0.0f;
    MeshInstance* meshInstance = nullptr;
    float lodBias = 1.0f;
    float rangeBegin = 0.0f;
    float rangeEnd = 0.0f;
    float rangeBeginMargin = 0.0f;
    float rangeEndMargin = 0.0f;
    ct::Vector<std::uint64_t> rangeStates;
    ct::Vector<std::uint8_t> hiddenSurfaces;
    // Last frame a camera drew it: skinning skips instances nobody has seen for a frame.
    mutable std::uint64_t seenFrame = 0;
};

struct RenderingServer::Skeleton
{
    std::uint32_t size = 0;
    ct::Vector<float> data;
    prisma::BufferHandle buffer;
    bool dirty = false;
    std::uint64_t version = 1;
};

struct RenderingServer::MeshInstance
{
    struct Surface
    {
        prisma::BufferHandle vertexBuffer;
        std::uint32_t bytes = 0;
    };

    MeshId mesh;
    std::uint32_t meshVersion = 0;
    ct::Vector<Surface> surfaces;
    std::uint64_t skeletonVersion = 0;
    bool dirty = true;
};

struct RenderingServer::Camera
{
    CameraProjection projection = CameraProjection::Perspective;
    float fovy = 75.0f;
    float size = 1.0f;
    float zNear = 0.05f;
    float zFar = 4000.0f;
    Math::Mat4 transform = Math::Mat4::Identity();
    std::uint32_t cullMask = 0xffffffff;
    bool customProjection = false;
    Math::Mat4 customMatrix = Math::Mat4::Identity();
    bool hasAttributes = false;
    CameraAttributesSettings attributes;
};

struct RenderingServer::Reflector
{
    ScenarioId scenario;
    Math::Mat4 transform = Math::Mat4::Identity();
    Math::Vec2 halfSize = Math::Vec2(1.0f, 1.0f);
    std::uint32_t resolution = 512;
    float maxDistance = 5.0f;
    bool visible = true;
    ViewportId viewport;
    CameraId camera;
    Math::Mat4 viewProjection = Math::Mat4::Identity();
    Math::Vec3 planeNormal = Math::Vec3(0.0f, 0.0f, 1.0f);
    Math::Vec3 planePoint;
};

struct RenderingServer::Decal
{
    DecalParams params;
    ScenarioId scenario;
    Math::Mat4 transform = Math::Mat4::Identity();
    float sortingOffset = 0.0f;
    bool visible = true;
};

struct RenderingServer::ReflectionProbe
{
    ReflectionProbeParams params;
    ScenarioId scenario;
    Math::Mat4 transform = Math::Mat4::Identity();
    bool visible = true;
};

struct RenderingServer::Lightmap
{
    ScenarioId scenario;
    Math::Mat4 transform = Math::Mat4::Identity();
    bool visible = true;
    TextureId light;
    TextureId shadowmask;
    bool usesSphericalHarmonics = false;
    ShadowmaskMode shadowmaskMode = ShadowmaskMode::None;
    float specularIntensity = 0.0f;
    float bakedExposure = 1.0f;
    bool hasBounds = false;
    Math::Box bounds;
    Math::Vec3 probeOrigin = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 probeCell = Math::Vec3(1.0f, 1.0f, 1.0f);
    std::uint32_t probeDims[3] = { 0, 0, 0 };
    ct::Vector<float> probeSh;
};

struct RenderingServer::Light
{
    LightType type = LightType::Omni;
    ScenarioId scenario;
    Math::Mat4 transform = Math::Mat4::Identity();
    Math::Vec3 color = Math::Vec3(1.0f, 1.0f, 1.0f);
    float params[static_cast<std::size_t>(LightParam::Count)] = {};
    std::uint32_t cullMask = 0xffffffff;
    bool negative = false;
    bool shadow = false;
    LightBakeMode bakeMode = LightBakeMode::Dynamic;
    bool blendSplits = false;
    DirectionalShadowMode shadowMode = DirectionalShadowMode::Parallel4Splits;
    TextureId lensFlare[kMaxLensFlareElements];
    std::uint32_t lensFlareCount = 0;
    struct Cascade
    {
        Math::Mat4 projection;
        Math::Mat4 transform;
        Math::Frustum frustum;
        float zFar = 0.0f;
        float split = 0.0f;
        float texelSize = 0.0f;
        float biasScale = 0.0f;
        float rangeBegin = 0.0f;
        float uvScale[2] = { 0.0f, 0.0f };
        float atlasRect[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    };
    Cascade cascades[4];
    std::uint32_t cascadeCount = 0;
    bool shadowActive = false;
    OmniShadowMode omniShadowMode = OmniShadowMode::Cube;
    std::uint32_t serial = 0;
    struct Positional
    {
        bool active = false;
        Math::Mat4 projection;
        Math::Mat4 transform;
        std::uint32_t shadowSize = 0;
        float atlasRect[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        int omniOffset[2] = { 0, 0 };
    };
    Positional positional;
    bool visible = true;
};

namespace
{

const std::uint32_t kLightmapLightSlot = 24;
const std::uint32_t kLightmapShadowmaskSlot = 25;
const std::uint32_t kAoSlot = 21;
const std::uint32_t kBestFitSlot = 22;
const std::uint32_t kSsrSlot = 22;
const std::uint32_t kSsrMipSlot = 23;
const std::uint32_t kSsEffectsUseSsao = 1u << 0;
const std::uint32_t kSsEffectsUseSsr = 1u << 2;
const std::uint32_t kSsEffectsResolveSsr = 1u << 3;
const std::uint32_t kTonemapFlagUseBcs = 1u << 0;
const std::uint32_t kTonemapFlagUseGlow = 1u << 1;
const std::uint32_t kTonemapFlagUseColorCorrection = 1u << 3;
const std::uint32_t kTonemapFlagUseFxaa = 1u << 4;
const std::uint32_t kTonemapFlagConvertToSrgb = 1u << 6;
const std::uint32_t kGlowFlagHorizontal = 1u << 0;
const std::uint32_t kGlowFlagFirstPass = 1u << 4;
const std::uint32_t kGlowLevels = 7;
const std::uint32_t kGlowMaxPasses = kGlowLevels * 2;

struct TonemapParams
{
    float bcs[3];
    std::uint32_t flags;
    float pixelSize[2];
    std::uint32_t tonemapper;
    float outputMaxValue;
    std::uint32_t glowTextureSize[2];
    float glowIntensity;
    float glowMapStrength;
    std::uint32_t glowMode;
    std::uint32_t padding[3];
    float glowLevels[7][4];
    float exposure;
    float white;
    float autoExposureScale;
    float luminanceMultiplier;
    float tonemapperParams[4];
};

static_assert(sizeof(TonemapParams) == 208, "TonemapParams must match the std140 block");

// The upscale pass (upscale.frag): the 3D picture put on the rectangle it fills in the window
// (uv of the window * uvScale + uvOffset is the uv of the picture; outside 0..1 the window shows
// black bars).
struct UpscaleParams
{
    float uvScale[2];
    float uvOffset[2];
};

static_assert(sizeof(UpscaleParams) == 16, "UpscaleParams must match the std140 block");

// The FSR 1.0 passes (fsr_upscale_compute.glsl): EASU scales the picture to the window's
// rectangle, RCAS sharpens it.
struct FsrParams
{
    float resolutionWidth;
    float resolutionHeight;
    float upscaledWidth;
    float upscaledHeight;
    float sharpness;
    std::int32_t pass;
    std::int32_t padding[2];
};

static_assert(sizeof(FsrParams) == 32, "FsrParams must match the std140 block");

// The SMAA passes (smaa_edge_detection, smaa_weight_calculation, smaa_blending).
struct SmaaEdgeParams
{
    float invSize[2];
    float threshold;
    float padding;
};

struct SmaaWeightParams
{
    float invSize[2];
    std::uint32_t size[2];
    float subsampleIndices[4];
};

struct SmaaBlendParams
{
    float invSize[2];
    std::uint32_t useDebanding;
    float padding;
};

static_assert(sizeof(SmaaEdgeParams) == 16 && sizeof(SmaaWeightParams) == 32 &&
                      sizeof(SmaaBlendParams) == 16,
        "the SMAA params must match their std140 blocks");

struct LensFlareParamsGpu
{
    float lightScreenPos[4];
    float offsetSize[4];
    float screenSizeRcp[4];
};

constexpr std::uint32_t kLensFlareMaxDraws = 8 * kMaxLensFlareElements;
constexpr float kLensFlareOcclusionRadius = 6.0f;
constexpr float kLensFlareOffsets[kMaxLensFlareElements] = { 1.0f, 0.55f, 0.4f, 0.1f, -0.1f, -0.3f,
    -0.5f };

struct GlowParamsGpu
{
    float destPixelSize[2];
    std::uint32_t flags;
    float sourceLod;
    float glowStrength;
    float glowBloom;
    float glowHdrThreshold;
    float glowHdrScale;
    float glowExposure;
    float glowWhite;
    float glowLuminanceCap;
    float glowAutoExposureScale;
};

static_assert(sizeof(GlowParamsGpu) == 48, "GlowParamsGpu must match the std140 block");

struct GlowPass
{
    prisma::TextureHandle target;
    prisma::TextureHandle source;
    std::uint32_t mip = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t offset = 0;
};

void storeMatrix(const Math::Mat4& matrix, float* out)
{
    memcpy(out, &matrix, sizeof(float) * 16);
}

void storeTransform3x4(const Math::Mat4& matrix, float* out)
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) out[r * 4 + c] = matrix[c][r];
}

float tonemapWhiteFor(TonemapMode mode, float white, float outputMaxValue)
{
    if (mode == TonemapMode::Linear) return outputMaxValue;
    if (mode == TonemapMode::Filmic || mode == TonemapMode::Aces)
        return white > 1.0f ? white : 1.0f;
    if (mode == TonemapMode::Agx) return (white > 2.0f ? white : 2.0f) * outputMaxValue;
    return white > outputMaxValue ? white : outputMaxValue;
}

void tonemapParameters(TonemapMode mode, float white, float agxContrast, float outputMaxValue,
        float params[4])
{
    params[0] = params[1] = params[2] = params[3] = 0.0f;
    if (mode == TonemapMode::Reinhard)
    {
        params[0] = (white * white) / outputMaxValue;
    }
    else if (mode == TonemapMode::Filmic)
    {
        const float exposureBias = 2.0f;
        const float A = 0.22f * exposureBias * exposureBias;
        const float B = 0.30f * exposureBias;
        const float C = 0.10f;
        const float D = 0.20f;
        const float E = 0.01f;
        const float F = 0.30f;
        params[0] =
                ((white * (A * white + C * B) + D * E) / (white * (A * white + B) + D * F)) - E / F;
    }
    else if (mode == TonemapMode::Aces)
    {
        const float exposureBias = 1.8f;
        const float A = 0.0245786f;
        const float B = 0.000090537f;
        const float C = 0.983729f;
        const float D = 0.432951f;
        const float E = 0.238081f;
        white *= exposureBias;
        params[0] = (white * (white + A) - B) / (white * (C * white + D) + E);
    }
    else if (mode == TonemapMode::Agx)
    {
        const float crossoverPoint = 0.18f;
        const float shoulderMax = outputMaxValue - crossoverPoint;
        const float toeA = ((1.0f / crossoverPoint) - 1.0f) * powf(crossoverPoint, agxContrast);
        const float slopeDenom = powf(crossoverPoint, agxContrast) + toeA;
        const float slope = (agxContrast * powf(crossoverPoint, agxContrast - 1.0f) * toeA) /
                            (slopeDenom * slopeDenom);
        float w = white - crossoverPoint;
        w = w * w;
        w = w / shoulderMax;
        w = w * slope;
        params[0] = agxContrast;
        params[1] = toeA;
        params[2] = slope;
        params[3] = w;
    }
}

void projectionPlanes(const Math::Mat4& projection, bool& orthogonal, float& zNear, float& zFar)
{
    if (projection.col2.w != 0.0f)
    {
        orthogonal = false;
        zNear = projection.col3.z / (projection.col2.z - 1.0f);
        zFar = projection.col3.z / (projection.col2.z + 1.0f);
    }
    else
    {
        orthogonal = true;
        zNear = (projection.col3.z + 1.0f) / projection.col2.z;
        zFar = (projection.col3.z - 1.0f) / projection.col2.z;
    }
}

std::uint32_t sceneBillboard(const MaterialInfo* material)
{
    if (!material) return 0;
    if (material->ocean.enabled) return 6;
    if (material->grass) return 2;
    if (material->terrainMorph)
        return material->terrain.enabled && material->shading == MaterialShading::Lit ? 4u : 3u;
    if (material->leafWind) return 5;
    return material->billboard == MaterialBillboard::FixedY ? 1u : 0u;
}

} // namespace

struct RenderingServer::Viewport
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t targetWidth = 0;
    std::uint32_t targetHeight = 0;
    std::uint32_t baseWidth = 0;
    std::uint32_t baseHeight = 0;
    float scaling3D = 1.0f;
    std::uint32_t scaling3DWidth = 0;
    std::uint32_t scaling3DHeight = 0;
    Scaling3DAspect scaling3DAspect = Scaling3DAspect::Stretch;
    Scaling3DMode scaling3DMode = Scaling3DMode::Bilinear;
    float fsrSharpness = 0.2f;
    // The FSR 1.0 targets at the size of the window's rectangle: EASU's output, then RCAS's.
    prisma::TextureHandle fsrEasu;
    prisma::TextureHandle fsrOutput;
    std::uint32_t fsrWidth = 0;
    std::uint32_t fsrHeight = 0;
    // The rectangle of the window the 3D picture fills, in base pixels (the last draw).
    float pictureX = 0.0f;
    float pictureY = 0.0f;
    float pictureWidth = 0.0f;
    float pictureHeight = 0.0f;
    bool fxaa = false;
    bool smaa = false;
    // The SMAA targets at the 3D size: the edges with their stencil, the blending weights, and
    // the antialiased picture.
    prisma::TextureHandle smaaEdges;
    prisma::TextureHandle smaaStencil;
    prisma::TextureHandle smaaBlend;
    prisma::TextureHandle smaaOutput;
    std::uint32_t smaaWidth = 0;
    std::uint32_t smaaHeight = 0;
    float meshLodThreshold = 1.0f;
    prisma::TextureHandle colorTexture;
    prisma::TextureHandle depthTexture;
    prisma::TextureHandle colorMsaa;
    prisma::TextureHandle depthMsaa;
    prisma::TextureHandle glowTexture;
    prisma::TextureHandle glowTemp;
    prisma::TextureHandle normalRoughness;
    std::uint32_t normalRoughnessWidth = 0;
    std::uint32_t normalRoughnessHeight = 0;
    SsaoBuffers ssao;
    SsrBuffers ssr;
    DofBuffers dof;
    bool colorStorage = false;
    bool colorStorageWanted = false;
    bool reflection = false;
    std::uint64_t rangeMask = 0;
    ScenarioId rangeScenario;
    prisma::TextureHandle screenColor;
    prisma::TextureHandle screenDepth;
    // The tonemapped picture at the 3D size, before the upscale pass scales it to the window.
    prisma::TextureHandle upscaleSource;
    std::uint32_t upscaleWidth = 0;
    std::uint32_t upscaleHeight = 0;
    std::uint32_t screenWidth = 0;
    std::uint32_t screenHeight = 0;
    std::uint32_t screenMips = 0;
    std::uint32_t glowWidth = 0;
    std::uint32_t glowHeight = 0;
    std::uint32_t glowMips = 0;
    ViewportMsaa msaa = ViewportMsaa::Disabled;
    std::uint32_t samples = 1;
    std::uint32_t samplesWanted = 1;
    bool toScreen = false;
    bool active = true;
    CameraId camera;
    ScenarioId scenario;
    Math::Vec4 clearColor = Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    ClusterBuilder* cluster = nullptr;
    ShadowAtlas shadowAtlas;
    prisma::TextureHandle shadowAtlasTexture;
    std::uint32_t shadowAtlasTextureSize = 0;
    bool transparent = false;
    ViewportUpdateMode updateMode = ViewportUpdateMode::Disabled;
    ViewportDebugDraw debugDraw = ViewportDebugDraw::Disabled;
    TextureId output;
    std::uint32_t outputWidth = 0;
    std::uint32_t outputHeight = 0;
    bool outputLinear = false;
    bool srgbOutput = false;
    bool drawBatch = false;
};

struct RenderingServer::State
{
    struct PreDrawListener
    {
        void* owner = nullptr;
        ct::Function<void()> callback;
    };

    ct::Vector<PreDrawListener> preDraw;
    bool preDrawDispatching = false;
    bool preDrawCompact = false;
    ct::SlotMap32<Scenario> scenarios;
    ct::SlotMap32<Instance> instances;
    ct::SlotMap32<Camera> cameras;
    ct::SlotMap32<Viewport> viewports;
    ct::SlotMap32<Skeleton> skeletons;
    MultiMeshStorage multimeshes;
    ImpostorManager impostors;
    prisma::ShaderHandle sceneMultiMeshVertexShaders[3][4];
    prisma::ShaderHandle sceneBillboardVertexShaders[3][3][5];
    prisma::ShaderHandle sceneTerrainVertexShaders[3];
    prisma::ShaderHandle shadowTerrainVertexShaders[2];
    prisma::ShaderHandle sceneTerrainFragmentShaders[2];
    prisma::ShaderHandle sceneOceanVertexShader;
    prisma::ShaderHandle sceneOceanFragmentShader;
    prisma::ShaderHandle shadowMultiMeshVertexShaders[4];
    prisma::ShaderHandle shadowDualMultiMeshVertexShaders[4];
    ct::SlotMap32<Light> lights;
    ct::SlotMap32<Decal> decals;
    DecalAtlas decalAtlas;
    ct::Vector<DecalInstanceInput> decalInputs;
    ct::Vector<DecalDataGpu> decalData;
    ct::Vector<DecalSorted> decalSorted;
    prisma::BufferHandle decalBuffer;
    std::uint32_t decalBufferSize = 0;
    std::uint32_t decalCount = 0;
    prisma::SamplerHandle decalSampler;
    ct::SlotMap32<Lightmap> lightmaps;
    prisma::BufferHandle lightmapBuffer;
    std::uint32_t lightmapBufferSize = 0;
    prisma::BufferHandle lightmapCaptureBuffer;
    LightmapDataGpu lightmapData[kMaxLightmaps];
    LightmapId lightmapIds[kMaxLightmaps];
    bool lightmapHasSh[kMaxLightmaps] = {};
    bool lightmapHasSpecular[kMaxLightmaps] = {};
    const Lightmap* lightmapRefs[kMaxLightmaps] = {};
    Math::Mat4 lightmapInverse[kMaxLightmaps];
    ct::Vector<float> captureData;
    std::uint32_t lightmapCaptureSize = 0;
    prisma::TextureHandle lightmapLight[kMaxLightmaps];
    prisma::TextureHandle lightmapShadowmask[kMaxLightmaps];
    std::uint32_t lightmapsUsed = 0;
    bool lightmapSpecularFrame = false;
    bool lightmapBicubic = true;
    bool lightmapWarnedMax = false;
    bool lightmapWarnedDetail = false;
    ct::SlotMap32<ReflectionProbe> reflectionProbes;
    ct::Vector<ReflectionInstanceInput> reflectionInputs;
    ct::Vector<ReflectionDataGpu> reflectionData;
    ct::Vector<ReflectionSorted> reflectionSorted;
    prisma::BufferHandle reflectionBuffer;
    std::uint32_t reflectionBufferSize = 0;
    std::uint32_t reflectionCount = 0;
    ClusterBuilderShared clusterShared;
    prisma::BufferHandle implementationBuffer;
    prisma::BufferHandle omniLightBuffer;
    prisma::BufferHandle spotLightBuffer;
    prisma::BufferHandle directionalLightBuffer;
    prisma::BufferHandle lightCountsBuffer;
    std::uint32_t omniLightBufferSize = 0;
    std::uint32_t spotLightBufferSize = 0;
    ct::Vector<LightDataGpu> omniLightData;
    ct::Vector<LightDataGpu> spotLightData;
    DirectionalLightDataGpu directionalLightData[kMaxDirectionalLights];
    LightCountsGpu lightCounts = {};
    ct::Vector<InstanceId> skinnedInstances;
    ct::Vector<Instance*> skinQueue;
    prisma::ShaderHandle sceneVertexShaders[kSceneVariantCount];
    prisma::ShaderHandle sceneFragmentShaders[kSceneVariantCount];
    prisma::ShaderHandle sceneNoShadowFragmentShaders[kSceneVariantCount];
    prisma::ShaderHandle sceneCoverageFragmentShaders[kSceneVariantCount];
    prisma::ShaderHandle sceneCoverageNoShadowFragmentShaders[kSceneVariantCount];
    prisma::ShaderHandle depthCoverageFragmentShaders[2];
    prisma::ShaderHandle normalVertexShaders[4];
    prisma::ShaderHandle normalMultiMeshVertexShaders[2][4];
    prisma::ShaderHandle normalFragmentShaders[128];
    ScreenSpaceEffects screenSpace;
    DepthOfField dof;
    bool dofWarnedTransparent = false;
    prisma::PipelineDesc tonemapDescription;
    prisma::PipelineHandle tonemapMaskedPipelines[2];
    bool screenSpaceAllowed = true;
    bool screenSpaceWarned = false;
    bool screenSpaceWarnedMsaa = false;
    bool screenSpaceWarnedSsr = false;
    bool screenSpaceWarnedSsrMsaa = false;
    bool screenSpaceWarnedSsrTransparent = false;
    prisma::BufferHandle prepassSceneBuffer;
    ct::HashMap<std::uint32_t, prisma::PipelineHandle> skyMsaaPipelines;
    prisma::ShaderHandle depthGrassVertexShaders[2];
    prisma::ShaderHandle depthLeafWindVertexShaders[5];
    prisma::BufferHandle materialBuffer;
    std::uint32_t materialBufferSize = 0;
    ct::Vector<std::uint8_t> materialBytes;
    ct::HashMap<std::uint32_t, std::uint32_t> materialSlots;
    prisma::SamplerHandle materialSampler;
    BatchRenderer batch;
    prisma::ShaderHandle skyVertexShader;
    prisma::ShaderHandle skyFragmentShaders[kSkyVariantCount];
    prisma::PipelineHandle skyPipelines[kSkyVariantCount];
    prisma::BufferHandle skyParamsBuffer;
    prisma::BufferHandle skyCubemapParamsBuffer;
    prisma::ShaderHandle octmapVertexShader;
    prisma::ShaderHandle octmapDownsampleShaders[2];
    prisma::ShaderHandle octmapRoughnessShader;
    prisma::PipelineHandle octmapDownsamplePipelines[2];
    prisma::PipelineHandle octmapRoughnessPipeline;
    prisma::SamplerHandle mipSampler;
    prisma::SamplerHandle skySampler;
    prisma::BufferHandle octmapParamsBuffer;
    std::uint32_t octmapParamsSize = 0;
    prisma::TextureHandle dummyRadiance;
    bool octmapReady = false;
    bool octmapFailed = false;
    prisma::BufferHandle skyLightsBuffer;
    prisma::BufferHandle skyMaterialBuffer;
    prisma::BufferHandle skySceneBuffer;
    prisma::BufferHandle skyFlatParamsBuffer;
    prisma::BufferHandle skyFlatColorBuffer;
    struct ShadowItem
    {
        std::uint64_t key = 0;
        const SurfaceInfo* surface = nullptr;
        prisma::BufferHandle vertexBuffer;
        prisma::BufferHandle indexBuffer;
        std::uint32_t indexCount = 0;
        Math::Mat4 transform;
        bool skinned = false;
        std::uint32_t multimeshMode = 0;
        prisma::BufferHandle transforms;
        std::uint32_t transformBytes = 0;
        std::uint32_t instanceCount = 1;
        bool scissor = false;
        bool albedo = false;
        std::uint32_t material = 0;
        prisma::TextureHandle albedoTexture;
        Math::Vec4 color;
        bool terrain = false;
        bool grass = false;
        // A double-sided material casts with both faces: a mesh open at the bottom (a barrel)
        // would otherwise leave the ground inside its footprint without a caster.
        bool doubleSided = false;
    };

    struct ShadowRun
    {
        std::uint32_t first = 0;
        std::uint32_t count = 0;
        const SurfaceInfo* surface = nullptr;
        prisma::BufferHandle vertexBuffer;
        prisma::BufferHandle indexBuffer;
        std::uint32_t indexCount = 0;
        std::uint32_t variant = 0;
        prisma::BufferHandle transforms;
        std::uint32_t transformBytes = 0;
        std::uint32_t material = 0;
        std::uint32_t materialSlot = 0;
        prisma::TextureHandle albedoTexture;
        float color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    };

    static constexpr std::uint32_t kShadowDual = 1u << 16;
    static constexpr std::uint32_t kShadowFrontCull = 1u << 17;
    static constexpr std::uint32_t kShadowReverseDepth = 1u << 18;
    static constexpr std::uint32_t kShadowMultiMeshShift = 19;
    static constexpr std::uint32_t kShadowScissor = 1u << 22;
    static constexpr std::uint32_t kShadowAlbedo = 1u << 23;
    static constexpr std::uint32_t kDepthPrePass = 1u << 24;
    static constexpr std::uint32_t kDepthNoCull = 1u << 25;
    static constexpr std::uint32_t kShadowTerrain = 1u << 26;
    static constexpr std::uint32_t kShadowGrass = 1u << 27;
    static constexpr std::uint32_t kDepthSamplesShift = 28;
    static constexpr std::uint32_t kDepthCoverage = 1u << 30;
    static constexpr std::uint32_t kDepthLeafWind = 1u << 31;

    struct ShadowPass
    {
        std::uint32_t x = 0;
        std::uint32_t y = 0;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t sceneOffset = 0;
        std::uint32_t firstRun = 0;
        std::uint32_t runCount = 0;
        prisma::TextureHandle target;
        std::uint32_t targetSize = 0;
        std::uint32_t layer = 0;
        std::uint32_t variant = 0;
        float clearDepth = 1.0f;
        bool begin = false;
        bool clear = false;
        bool geometry = true;
        bool copy = false;
        prisma::TextureHandle source;
        std::uint32_t copyOffset = 0;
    };

    struct ShadowCopyParams
    {
        float zFar = 0.0f;
        float zNear = 0.0f;
        float texelSize[2] = { 0.0f, 0.0f };
    };

    prisma::TextureHandle shadowAtlas;
    std::uint32_t shadowAtlasSize = 4096;
    bool shadowSixteenBits = false;
    prisma::SamplerHandle shadowSampler;
    prisma::SamplerHandle shadowDepthSampler;
    prisma::TextureHandle dummyShadow;
    prisma::ShaderHandle shadowVertexShader;
    prisma::ShaderHandle shadowFragmentShader;
    prisma::ShaderHandle shadowDualVertexShader;
    prisma::ShaderHandle shadowDualFragmentShader;
    prisma::ShaderHandle cubeCopyFragmentShader;
    prisma::PipelineHandle cubeCopyPipeline;
    prisma::BufferHandle shadowCopyBuffer;
    std::uint32_t shadowCopyBufferSize = 0;
    ct::Vector<std::uint8_t> shadowCopyBytes;
    ct::HashMap<std::uint32_t, prisma::TextureHandle> shadowCubes;
    bool cubeShadowsReady = false;
    prisma::SamplerHandle positionalShadowSampler;
    prisma::SamplerHandle positionalShadowDepthSampler;
    std::uint32_t positionalSoftSamples = 4;
    std::uint32_t positionalPenumbraSamples = 8;
    float positionalQualityRadius = 2.0f;
    float positionalSoftKernel[128] = {};
    float positionalPenumbraKernel[128] = {};
    ShadowQuality positionalQuality = ShadowQuality::SoftLow;
    std::uint32_t lightSerial = 0;
    std::uint64_t scenePassCounter = 0;
    bool directionalShadowsActive = false;
    bool positionalShadowsActive = false;
    bool positionalSoftShadows = false;
    prisma::TextureHandle activePositionalAtlas;
    ct::HashMap<std::uint32_t, prisma::PipelineHandle> shadowPipelines;
    prisma::BufferHandle shadowSceneBuffer;
    prisma::BufferHandle shadowInstanceBuffer;
    prisma::BufferHandle shadowDrawBuffer;
    std::uint32_t shadowSceneBufferSize = 0;
    std::uint32_t shadowInstanceBufferSize = 0;
    std::uint32_t shadowDrawBufferSize = 0;
    ct::Vector<ShadowItem> shadowItems;
    ct::Vector<ShadowItem> shadowSorted;
    ct::Vector<InstanceDataGpu> shadowInstanceData;
    ct::Vector<ShadowRun> shadowRuns;
    ct::Vector<ShadowPass> shadowPasses;
    ct::Vector<std::uint8_t> shadowSceneBytes;
    ct::Vector<std::uint8_t> shadowDrawBytes;
    prisma::BufferHandle shadowMaterialBuffer;
    std::uint32_t shadowMaterialBufferSize = 0;
    ct::Vector<std::uint8_t> shadowMaterialBytes;
    ct::HashMap<std::uint32_t, std::uint32_t> shadowMaterialSlots;
    prisma::ShaderHandle depthVertexShaders[2][5];
    prisma::ShaderHandle depthFragmentShaders[2][2];
    ct::Vector<std::uint8_t> prepassDrawn;
    bool depthPrePass = true;
    bool roughnessLimiter = true;
    float roughnessLimiterAmount = 0.25f;
    float roughnessLimiterLimit = 0.18f;
    bool shadowReady = false;
    bool shadowFailed = false;
    ShadowQuality shadowQuality = ShadowQuality::SoftLow;
    std::uint32_t directionalSoftSamples = 4;
    std::uint32_t directionalPenumbraSamples = 8;
    float directionalQualityRadius = 2.0f;
    float directionalSoftKernel[128] = {};
    float directionalPenumbraKernel[128] = {};
    bool directionalSoftShadows = false;
    prisma::ShaderHandle dfgShader;
    prisma::PipelineHandle dfgPipeline;
    prisma::TextureHandle dfgTexture;
    bool dfgGenerated = false;
    prisma::ShaderHandle skinShader;
    prisma::PipelineHandle skinPipeline;
    prisma::BufferHandle skinParamsBuffer;
    std::uint32_t skinParamsSize = 0;
    ct::Vector<std::uint8_t> skinParamsBytes;
    bool skinningSupported = false;
    ct::HashMap<std::uint64_t, prisma::PipelineHandle> pipelines;
    // Scene pipelines still being linked off the main thread, with the time they were requested.
    ct::HashMap<std::uint64_t, std::uint64_t> pendingPipelines;
    bool deferredPipelines = true;
    prisma::BufferHandle uniformBuffer;
    std::uint32_t uniformBufferSize = 0;
    prisma::BufferHandle sceneBuffer;
    prisma::BufferHandle instanceBuffer;
    std::uint32_t instanceBufferSize = 0;
    ct::Vector<InstanceDataGpu> instanceData;
    std::uint64_t startNanos = 0;
    prisma::ShaderHandle tonemapVertex;
    prisma::ShaderHandle tonemapFragment;
    prisma::ShaderHandle depthReadFragment;
    prisma::PipelineHandle depthReadPipeline;
    prisma::ShaderHandle screenCopyFragment;
    prisma::PipelineHandle screenCopyColorPipeline;
    prisma::PipelineHandle screenCopyDepthPipeline;
    prisma::SamplerHandle screenSampler;
    bool screenCopyRegistered = false;
    prisma::BufferHandle depthReadParams;
    prisma::TextureHandle depthReadTarget;
    prisma::SamplerHandle nearestSampler;
    prisma::PipelineHandle tonemapPipeline;
    prisma::PipelineHandle tonemapTexturePipelines[2];
    prisma::BufferHandle tonemapParams;
    prisma::ShaderHandle upscaleFragment;
    prisma::PipelineHandle upscalePipeline;
    prisma::BufferHandle upscaleParams;
    prisma::ShaderHandle fsrShader;
    prisma::PipelineHandle fsrPipeline;
    prisma::BufferHandle fsrParams[2];
    prisma::ShaderHandle smaaShaders[6];
    prisma::PipelineHandle smaaEdgePipeline;
    prisma::PipelineHandle smaaWeightPipeline;
    prisma::PipelineHandle smaaBlendPipeline;
    prisma::BufferHandle smaaParams[3];
    prisma::TextureHandle smaaAreaTex;
    prisma::TextureHandle smaaSearchTex;
    float smaaEdgeThreshold = 0.05f;
    prisma::SamplerHandle linearSampler;
    prisma::ShaderHandle glowFragment;
    prisma::PipelineHandle glowPipeline;
    prisma::BufferHandle glowParams;
    prisma::SamplerHandle glowSampler;
    ct::Vector<std::uint8_t> glowBytes;
    bool glowReady = false;
    bool glowFailed = false;
    bool glowRegistered = false;
    prisma::ShaderHandle lensFlareVertex;
    prisma::ShaderHandle lensFlareFragment;
    prisma::PipelineHandle lensFlarePipeline;
    prisma::BufferHandle lensFlareParams;
    ct::Vector<std::uint8_t> lensFlareBytes;
    ct::SlotMap32<Reflector> reflectors;
    ReflectorId activeReflectors[kMaxActiveReflectors];
    std::uint32_t activeReflectorCount = 0;
    PlanarReflectionsGpu planarData;
    prisma::BufferHandle planarBuffer;
    bool lensFlareReady = false;
    bool lensFlareFailed = false;
    std::uint32_t buildCount = 0;
    double buildMilliseconds = 0.0;

    void noteBuild(const char* what, std::uint32_t key, std::uint64_t startNanos)
    {
        const double milliseconds = static_cast<double>(time_nanos() - startNanos) * 1e-6;
        ++buildCount;
        buildMilliseconds += milliseconds;
        if (milliseconds >= 5.0)
            log_info("render: %s %u built in %.1f ms", what, key, milliseconds);
    }
    ct::Vector<RenderItem> items;
    ct::Vector<SortEntry> order;
    ct::Vector<DrawRun> runs;
    ct::Vector<std::uint8_t> uniformBytes;
    std::uint64_t frameNumber = 0;
    bool cullingTree = true;
    std::uint32_t leafCapacity = 2;
    ct::ThreadPool pool{ 1 };

    void markIndexDirty(ScenarioId id)
    {
        if (Scenario* scenario = find(scenarios, id)) scenario->indexDirty = true;
    }

    void markAllIndexDirty()
    {
        for (Scenario& scenario: scenarios) scenario.indexDirty = true;
    }

    static void addMask(Scenario& scenario, std::uint32_t mask)
    {
        ++scenario.maskCounts[mask];
    }

    static void removeMask(Scenario& scenario, std::uint32_t mask)
    {
        std::uint32_t* count = scenario.maskCounts.find(mask);
        if (!count) return;
        if (*count > 1) --*count;
        else
            scenario.maskCounts.erase(mask);
    }

    static MaterialId resolveMaterial(const Instance& instance, const MeshInfo* mesh,
            std::uint32_t surface, MaterialStore& materials)
    {
        if (materials.valid(instance.materialOverride)) return instance.materialOverride;
        if (surface < instance.surfaceMaterials.size() &&
                materials.valid(instance.surfaceMaterials[surface]))
            return instance.surfaceMaterials[surface];
        if (mesh && surface < mesh->surfaces.size() &&
                materials.valid(mesh->surfaces[surface].material))
            return mesh->surfaces[surface].material;
        return materials.defaultMaterial();
    }

    Math::Box meshBox(const MeshInfo& mesh, SkeletonId skeletonId)
    {
        const Skeleton* skeleton = find(skeletons, skeletonId);
        if (!skeleton || skeleton->size == 0) return mesh.aabb;
        Math::Box result;
        bool any = false;
        for (std::size_t s = 0; s < mesh.surfaces.size(); ++s)
        {
            const SurfaceInfo& surface = mesh.surfaces[s];
            if (!surface.skinned() || surface.boneAabbs.empty())
            {
                result = result.Union(surface.aabb);
                any = true;
                continue;
            }
            const std::size_t bones = surface.boneAabbs.size() < skeleton->size
                                              ? surface.boneAabbs.size()
                                              : skeleton->size;
            bool found = false;
            Math::Box skinnedBox;
            for (std::size_t b = 0; b < bones; ++b)
            {
                const Math::Box& box = surface.boneAabbs[b];
                if (emptyBox(box)) continue;
                skinnedBox = skinnedBox.Union(transformBox(box, boneTransform(*skeleton, b)));
                found = true;
            }
            result = result.Union(found ? skinnedBox : surface.aabb);
            any = true;
        }
        return any ? result : mesh.aabb;
    }

    Math::Box instanceBounds(const Instance& instance, const MeshInfo& mesh)
    {
        if (instance.multimesh.valid())
        {
            const MultiMeshData* data = multimeshes.get(instance.multimesh);
            if (!data) return Math::Box();
            return transformBox(MultiMeshStorage::effectiveAabb(*data), instance.transform);
        }
        return transformBox(meshBox(mesh, instance.skeleton), instance.transform);
    }

    const MeshInfo* instanceMesh(const MeshStore& meshes, const Instance& instance) const
    {
        if (instance.multimesh.valid())
        {
            const MultiMeshData* data = multimeshes.get(instance.multimesh);
            return data ? meshes.get(data->mesh) : nullptr;
        }
        return meshes.get(instance.base);
    }

    bool instanceUsable(const MeshStore& meshes, const Instance& instance) const
    {
        if (!usableMesh(instanceMesh(meshes, instance))) return false;
        if (!instance.multimesh.valid()) return true;
        const MultiMeshData* data = multimeshes.get(instance.multimesh);
        return data && !emptyBox(MultiMeshStorage::effectiveAabb(*data));
    }

    std::uint32_t instanceStamp(const Instance& instance, const MeshInfo& mesh) const
    {
        if (!instance.multimesh.valid()) return mesh.version;
        const MultiMeshData* data = multimeshes.get(instance.multimesh);
        return mesh.version + (data ? data->version : 0);
    }

    static constexpr std::uint32_t kWholeInstance = 0xffffffffu;

    static bool splitsSurfaces(const Instance& instance, const MeshInfo& mesh)
    {
        if (instance.multimesh.valid() || instance.skeleton.valid() || mesh.surfaces.size() < 2)
            return false;
        for (std::size_t s = 0; s < mesh.surfaces.size(); ++s)
            if (emptyBox(mesh.surfaces[s].aabb)) return false;
        return true;
    }

    Math::Box entryBounds(const Instance& instance, const MeshInfo& mesh, std::uint32_t surface)
    {
        if (surface == kWholeInstance) return instanceBounds(instance, mesh);
        return transformBox(mesh.surfaces[surface].aabb, instance.transform);
    }

    struct LodView
    {
        Math::Vec3 cameraPosition;
        float distanceMultiplier = 0.0f;
        float threshold = 0.0f;
        bool orthogonal = false;
    };

    static std::uint32_t surfaceLod(const SurfaceInfo& surface, const Instance& instance,
            const Math::Box& bounds, const LodView& view)
    {
        if (!(view.threshold > 0.0f) || surface.lods.empty()) return 0;
        float distance = 1.0f;
        if (!view.orthogonal)
        {
            const Math::Vec3 below = bounds.min - view.cameraPosition;
            const Math::Vec3 above = view.cameraPosition - bounds.max;
            const Math::Vec3 gap(fmaxf(0.0f, fmaxf(below.x, above.x)),
                    fmaxf(0.0f, fmaxf(below.y, above.y)), fmaxf(0.0f, fmaxf(below.z, above.z)));
            distance = gap.Length();
        }
        const Math::Mat4& transform = instance.transform;
        const float scaleX = Math::Vec3(transform[0].x, transform[0].y, transform[0].z).Length();
        const float scaleY = Math::Vec3(transform[1].x, transform[1].y, transform[1].z).Length();
        const float scaleZ = Math::Vec3(transform[2].x, transform[2].y, transform[2].z).Length();
        const float modelScale = fmaxf(scaleX, fmaxf(scaleY, scaleZ)) * instance.lodBias;
        const float distanceThreshold = distance * view.distanceMultiplier;
        std::uint32_t current = 0;
        for (std::size_t i = 0; i < surface.lods.size(); ++i)
        {
            const float screenSize = surface.lods[i].edgeLength * modelScale / distanceThreshold;
            if (screenSize > view.threshold) break;
            current = static_cast<std::uint32_t>(i) + 1;
        }
        return current;
    }

    static bool surfaceHidden(const Instance& instance, std::size_t surface)
    {
        return surface < instance.hiddenSurfaces.size() && instance.hiddenSurfaces[surface] != 0;
    }

    static bool hasVisibilityRange(const Instance& instance)
    {
        return instance.rangeBegin > 0.0f || instance.rangeEnd > 0.0f;
    }

    static int visibilityRangeCheck(Instance& instance, std::uint32_t surface,
            std::uint64_t viewportMask, const Math::Vec3& cameraPosition, const Math::Box& bounds)
    {
        const std::size_t slot = surface == kWholeInstance ? 0 : surface;
        if (instance.rangeStates.size() <= slot) instance.rangeStates.resize(slot + 1, 0);
        std::uint64_t& viewportState = instance.rangeStates[slot];
        const float distance = (bounds.Center() - cameraPosition).Length();
        float beginOffset = -instance.rangeBeginMargin;
        float endOffset = instance.rangeEndMargin;
        if (!(viewportMask & viewportState))
        {
            beginOffset = -beginOffset;
            endOffset = -endOffset;
        }
        if (instance.rangeEnd > 0.0f && distance > instance.rangeEnd + endOffset)
        {
            viewportState &= ~viewportMask;
            return -1;
        }
        if (instance.rangeBegin > 0.0f && distance < instance.rangeBegin + beginOffset)
        {
            viewportState &= ~viewportMask;
            return 1;
        }
        viewportState |= viewportMask;
        return 0;
    }

    void releaseRangeMask(Viewport& viewport)
    {
        if (viewport.rangeMask == 0) return;
        if (Scenario* scenario = find(scenarios, viewport.rangeScenario))
            scenario->usedRangeBits &= ~viewport.rangeMask;
        viewport.rangeMask = 0;
    }

    std::uint64_t acquireRangeMask(Viewport& viewport, Scenario& scenario)
    {
        if (viewport.rangeMask != 0 && viewport.rangeScenario == viewport.scenario)
            return viewport.rangeMask;
        releaseRangeMask(viewport);
        std::uint64_t mask = 1;
        while (mask & scenario.usedRangeBits) mask <<= 1;
        if (mask == 0) mask = 1ull << 63;
        scenario.usedRangeBits |= mask;
        viewport.rangeMask = mask;
        viewport.rangeScenario = viewport.scenario;
        return mask;
    }

    bool refreshEntries(Scenario& scenario, std::uint32_t first, InstanceId id,
            const Instance& instance, const MeshInfo& mesh)
    {
        const bool split = splitsSurfaces(instance, mesh);
        const std::size_t expected = split ? mesh.surfaces.size() : 1;
        std::size_t count = 0;
        while (first + count < scenario.entries.size() && scenario.entries[first + count] == id)
            ++count;
        if (count != expected || (scenario.entrySurfaces[first] != kWholeInstance) != split)
            return false;
        const std::uint32_t stamp = instanceStamp(instance, mesh);
        for (std::size_t i = 0; i < count; ++i)
        {
            scenario.bounds[first + i] =
                    entryBounds(instance, mesh, scenario.entrySurfaces[first + i]);
            scenario.meshVersions[first + i] = stamp;
        }
        return true;
    }

    static Math::Mat4 boneTransform(const Skeleton& skeleton, std::size_t bone)
    {
        const float* row = skeleton.data.data() + bone * kBoneFloats;
        Math::Mat4 result = Math::Mat4::Identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) result[c][r] = row[r * 4 + c];
        return result;
    }

    static void storeBoneTransform(Skeleton& skeleton, std::size_t bone, const Math::Mat4& m)
    {
        float* row = skeleton.data.data() + bone * kBoneFloats;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) row[r * 4 + c] = m[c][r];
    }

    void freeMeshInstance(prisma::Driver* driver, Instance& instance)
    {
        if (!instance.meshInstance) return;
        for (MeshInstance::Surface& surface: instance.meshInstance->surfaces)
            if (surface.vertexBuffer.valid()) driver->destroy(surface.vertexBuffer);
        delete instance.meshInstance;
        instance.meshInstance = nullptr;
    }

    static bool needsMeshInstance(const Instance& instance, const MeshInfo* mesh)
    {
        if (!instance.skeleton.valid() || !mesh) return false;
        for (const SurfaceInfo& surface: mesh->surfaces)
            if (surface.skinned()) return true;
        return false;
    }

    bool syncMeshInstance(prisma::Driver* driver, Instance& instance, const MeshInfo* mesh)
    {
        const bool needed = needsMeshInstance(instance, mesh);
        if (!needed)
        {
            freeMeshInstance(driver, instance);
            return false;
        }
        if (instance.meshInstance && instance.meshInstance->mesh == instance.base &&
                instance.meshInstance->meshVersion == mesh->version)
            return true;
        freeMeshInstance(driver, instance);
        MeshInstance* result = new MeshInstance();
        result->mesh = instance.base;
        result->meshVersion = mesh->version;
        result->surfaces.resize(mesh->surfaces.size());
        for (std::size_t s = 0; s < mesh->surfaces.size(); ++s)
        {
            const SurfaceInfo& surface = mesh->surfaces[s];
            if (!surface.skinned() || !surface.ready) continue;
            prisma::BufferDesc desc;
            desc.usage = prisma::BufferUsage::Storage;
            desc.update = prisma::BufferUpdate::Static;
            desc.size = surface.vertexCount * surface.vertexStride;
            desc.debugName = "skinned vertices";
            result->surfaces[s].vertexBuffer = driver->createBuffer(desc);
            result->surfaces[s].bytes = desc.size;
            if (!result->surfaces[s].vertexBuffer.valid())
                log_error("render: could not create a skinned vertex buffer");
            else
                driver->copyBuffer(surface.vertexBuffer, 0, result->surfaces[s].vertexBuffer, 0,
                        desc.size);
        }
        instance.meshInstance = result;
        return true;
    }

    bool ensureSkinParams(prisma::Driver* driver, std::uint32_t size)
    {
        if (skinParamsBuffer.valid() && skinParamsSize >= size) return true;
        std::uint32_t grown = skinParamsSize * 2;
        if (grown < size) grown = size;
        if (skinParamsBuffer.valid()) driver->destroy(skinParamsBuffer);
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Uniform;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = grown;
        desc.debugName = "skinning params";
        skinParamsBuffer = driver->createBuffer(desc);
        skinParamsSize = skinParamsBuffer.valid() ? grown : 0;
        if (!skinParamsBuffer.valid()) log_error("render: could not create the skinning params");
        return skinParamsBuffer.valid();
    }

    void queueBounds(Instance& instance, InstanceId id)
    {
        if (instance.queued) return;
        Scenario* scenario = indexedScenario(instance, id);
        if (!scenario) return;
        instance.queued = true;
        scenario->dirty.push_back(id);
    }

    Scenario* indexedScenario(const Instance& instance, InstanceId id)
    {
        if (!cullingTree) return nullptr;
        Scenario* scenario = find(scenarios, instance.scenario);
        if (!scenario || scenario->indexDirty || !scenario->entryIndex.contains(id.bits()))
            return nullptr;
        return scenario;
    }

    void waitForBuild(Scenario& scenario)
    {
        if (!scenario.buildPending) return;
        pool.wait_all();
        scenario.buildPending = false;
    }

    void rebuildIndex(Scenario& scenario, const MeshStore& meshes)
    {
        waitForBuild(scenario);
        for (InstanceId id: scenario.dirty)
        {
            if (Instance* instance = find(instances, id)) instance->queued = false;
        }
        scenario.dirty.clear();
        scenario.entries.clear();
        scenario.entrySurfaces.clear();
        scenario.bounds.clear();
        scenario.meshVersions.clear();
        scenario.entryIndex.clear();
        scenario.waiting.clear();
        scenario.maskCounts.clear();
        for (InstanceId id: scenario.instances)
        {
            const Instance* instance = find(instances, id);
            if (!instance) continue;
            const MeshInfo* mesh = instanceMesh(meshes, *instance);
            if (!mesh)
            {
                if (multimeshes.valid(instance->multimesh)) scenario.waiting.push_back(id);
                continue;
            }
            if (!instanceUsable(meshes, *instance))
            {
                scenario.waiting.push_back(id);
                continue;
            }
            scenario.entryIndex[id.bits()] = static_cast<std::uint32_t>(scenario.entries.size());
            const std::uint32_t stamp = instanceStamp(*instance, *mesh);
            const std::uint32_t surfaces =
                    splitsSurfaces(*instance, *mesh)
                            ? static_cast<std::uint32_t>(mesh->surfaces.size())
                            : 1;
            for (std::uint32_t s = 0; s < surfaces; ++s)
            {
                const std::uint32_t surface = surfaces > 1 ? s : kWholeInstance;
                scenario.entries.push_back(id);
                scenario.entrySurfaces.push_back(surface);
                scenario.bounds.push_back(entryBounds(*instance, *mesh, surface));
                scenario.meshVersions.push_back(stamp);
            }
            if (instance->visible) addMask(scenario, instance->layerMask);
        }
        scenario.tree->setLeafCapacity(leafCapacity);
        scenario.tree->build(scenario.bounds.data(),
                static_cast<std::uint32_t>(scenario.bounds.size()));
        scenario.indexDirty = false;
        scenario.boundsChanged = false;
    }

    void refreshBounds(Scenario& scenario, const MeshStore& meshes)
    {
        for (InstanceId id: scenario.dirty)
        {
            Instance* instance = find(instances, id);
            if (!instance) continue;
            instance->queued = false;
            const std::uint32_t* index = scenario.entryIndex.find(id.bits());
            if (!index) continue;
            const MeshInfo* mesh = instanceMesh(meshes, *instance);
            if (!instanceUsable(meshes, *instance))
            {
                scenario.indexDirty = true;
                continue;
            }
            if (!refreshEntries(scenario, *index, id, *instance, *mesh))
            {
                scenario.indexDirty = true;
                continue;
            }
            scenario.boundsChanged = true;
        }
        scenario.dirty.clear();

        for (std::size_t i = 0; i < scenario.entries.size(); ++i)
        {
            const Instance* instance = find(instances, scenario.entries[i]);
            const MeshInfo* mesh = instance ? instanceMesh(meshes, *instance) : nullptr;
            if (!instance || !instanceUsable(meshes, *instance))
            {
                scenario.indexDirty = true;
                continue;
            }
            const std::uint32_t stamp = instanceStamp(*instance, *mesh);
            if (stamp == scenario.meshVersions[i]) continue;
            const InstanceId id = scenario.entries[i];
            const std::uint32_t* first = scenario.entryIndex.find(id.bits());
            if (!first || !refreshEntries(scenario, *first, id, *instance, *mesh))
            {
                scenario.indexDirty = true;
                continue;
            }
            scenario.boundsChanged = true;
        }

        for (InstanceId id: scenario.waiting)
        {
            const Instance* instance = find(instances, id);
            if (instance && instanceUsable(meshes, *instance)) scenario.indexDirty = true;
        }
    }

    bool updateTree(Scenario& scenario, const MeshStore& meshes)
    {
        if (scenario.updatedFrame == frameNumber) return false;
        scenario.updatedFrame = frameNumber;

        if (!scenario.indexDirty) refreshBounds(scenario, meshes);
        if (scenario.indexDirty)
        {
            rebuildIndex(scenario, meshes);
            return true;
        }

        if (scenario.buildPending && scenario.job->done.load() != 0)
        {
            scenario.tree.swap(scenario.treeNext);
            scenario.buildPending = false;
        }

        if (scenario.entries.empty()) return true;
        if (!scenario.tree->refit(scenario.bounds.data(),
                    static_cast<std::uint32_t>(scenario.bounds.size())))
        {
            rebuildIndex(scenario, meshes);
            return true;
        }

        if (scenario.buildPending || !scenario.boundsChanged) return true;

        Scenario::BuildJob& job = *scenario.job;
        job.bounds.resize(scenario.bounds.size());
        for (std::size_t i = 0; i < scenario.bounds.size(); ++i) job.bounds[i] = scenario.bounds[i];
        scenario.treeNext->setLeafCapacity(leafCapacity);
        job.target = scenario.treeNext.get();
        job.done.store(0);
        scenario.boundsChanged = false;
        scenario.buildPending = true;
        Scenario::BuildJob* work = &job;
        pool.submit(ct::ThreadPool::Job(
                [work]()
                {
                    work->target->build(work->bounds.data(),
                            static_cast<std::uint32_t>(work->bounds.size()));
                    work->done.store(1);
                }));
        return true;
    }

    static constexpr std::uint64_t kEqualDepthKey = 1ull << 32;
    static constexpr std::uint64_t kNoShadowKey = 1ull << 33;
    static constexpr std::uint64_t kCoverageKey = 1ull << 34;
    static constexpr std::uint32_t kSamplesKeyShift = 35;
    static constexpr std::uint64_t kNormalRoughnessKey = 1ull << 40;
    static constexpr std::uint64_t kLightmapKey = 1ull << 37;
    static constexpr std::uint64_t kDepthOnlyKey = 1ull << 38;

    static std::uint32_t sampleIndex(std::uint32_t samples)
    {
        return samples >= 8 ? 3u : (samples >= 4 ? 2u : (samples >= 2 ? 1u : 0u));
    }

    static std::uint64_t pipelineKey(const SurfaceInfo& surface, std::uint32_t variant,
            MaterialCull cull, bool blend, MaterialBlend blendMode, bool depthTest,
            std::uint32_t multimeshMode, std::uint32_t billboard)
    {
        return surface.attributeMask | (static_cast<std::uint32_t>(surface.topology) << 8) |
               (variant << 12) | (static_cast<std::uint32_t>(cull) << 19) |
               (blend ? 1u << 21 : 0u) | (static_cast<std::uint32_t>(blendMode) << 22) |
               (depthTest ? 0u : 1u << 25) | (multimeshMode << 26) | (billboard << 29);
    }

    static std::uint32_t multimeshModeFor(const MultiMeshData& data)
    {
        return 1u + (data.usesColors ? 1u : 0u) + (data.usesCustomData ? 2u : 0u);
    }

    static std::uint32_t multimeshTypeIndex(std::uint32_t variant)
    {
        if (variant & 32u) return 2;
        return (variant & 8u) ? 1 : 0;
    }

    bool ensureSceneShaders(prisma::Driver* driver, std::uint32_t variant, bool vertex)
    {
        if ((!vertex || sceneVertexShaders[variant].valid()) &&
                sceneFragmentShaders[variant].valid())
            return true;
        const prisma::Caps& caps = driver->caps();
        const std::uint64_t start = time_nanos();
        if (vertex && !sceneVertexShaders[variant].valid())
            sceneVertexShaders[variant] =
                    driver->createShader(prisma3d::shaderDesc(*kSceneVertexBlobs[variant], caps));
        if (!sceneFragmentShaders[variant].valid())
            sceneFragmentShaders[variant] =
                    driver->createShader(prisma3d::shaderDesc(*kSceneFragmentBlobs[variant], caps));
        noteBuild("scene shaders variant", variant, start);
        return (!vertex || sceneVertexShaders[variant].valid()) &&
               sceneFragmentShaders[variant].valid();
    }

    prisma::ShaderHandle multimeshVertexShader(prisma::Driver* driver, std::uint32_t variant,
            std::uint32_t multimeshMode)
    {
        prisma::ShaderHandle& shader =
                sceneMultiMeshVertexShaders[multimeshTypeIndex(variant)][multimeshMode - 1];
        if (!shader.valid())
            shader = driver->createShader(prisma3d::shaderDesc(
                    *kSceneMultiMeshVertexBlobs[multimeshTypeIndex(variant)][multimeshMode - 1],
                    driver->caps()));
        return shader;
    }

    prisma::ShaderHandle billboardVertexShader(prisma::Driver* driver, std::uint32_t billboard,
            std::uint32_t variant, std::uint32_t multimeshMode)
    {
        const std::uint32_t slot = billboard == 5 ? 2u : billboard - 1;
        prisma::ShaderHandle& shader =
                sceneBillboardVertexShaders[slot][multimeshTypeIndex(variant)][multimeshMode];
        const ShaderVariant* blob =
                kSceneBillboardVertexBlobs[slot][multimeshTypeIndex(variant)][multimeshMode];
        if (!shader.valid() && blob)
            shader = driver->createShader(prisma3d::shaderDesc(*blob, driver->caps()));
        return shader;
    }

    prisma::ShaderHandle terrainVertexShader(prisma::Driver* driver, std::uint32_t variant)
    {
        prisma::ShaderHandle& shader = sceneTerrainVertexShaders[multimeshTypeIndex(variant)];
        if (!shader.valid())
            shader = driver->createShader(prisma3d::shaderDesc(
                    *kSceneTerrainVertexBlobs[multimeshTypeIndex(variant)], driver->caps()));
        return shader;
    }

    prisma::ShaderHandle terrainFragmentShader(prisma::Driver* driver, std::uint32_t variant)
    {
        const std::uint32_t index = variant & 1u;
        prisma::ShaderHandle& shader = sceneTerrainFragmentShaders[index];
        if (!shader.valid())
            shader = driver->createShader(
                    prisma3d::shaderDesc(*kSceneTerrainFragmentBlobs[index], driver->caps()));
        return shader;
    }

    prisma::ShaderHandle coverageFragmentShader(prisma::Driver* driver, std::uint32_t variant,
            bool noShadows)
    {
        const bool shadowsOff = noShadows && kSceneCoverageNoShadowFragmentBlobs[variant];
        const ShaderVariant* blob = shadowsOff ? kSceneCoverageNoShadowFragmentBlobs[variant]
                                               : kSceneCoverageFragmentBlobs[variant];
        if (!blob) return prisma::ShaderHandle();
        prisma::ShaderHandle& shader = shadowsOff ? sceneCoverageNoShadowFragmentShaders[variant]
                                                  : sceneCoverageFragmentShaders[variant];
        if (!shader.valid())
            shader = driver->createShader(prisma3d::shaderDesc(*blob, driver->caps()));
        return shader;
    }

    static void fillSceneVertexLayout(prisma::PipelineDesc& desc, const SurfaceInfo& surface,
            bool normalMap, bool detail)
    {
        const std::uint32_t mask = surface.attributeMask;
        desc.vertexBuffers[0].stride = surface.vertexStride;
        desc.vertexBufferCount = 1;
        std::uint32_t count = 0;
        desc.attributes[count].location = 0;
        desc.attributes[count].format = prisma::VertexFormat::Float3;
        desc.attributes[count++].offset = MeshStore::attributeOffset(mask, kVertexPosition);
        desc.attributes[count].location = 1;
        desc.attributes[count].format = prisma::VertexFormat::Float3;
        desc.attributes[count++].offset = MeshStore::attributeOffset(mask, kVertexNormal);
        if (normalMap)
        {
            desc.attributes[count].location = 2;
            desc.attributes[count].format = prisma::VertexFormat::Float4;
            desc.attributes[count++].offset = MeshStore::attributeOffset(mask, kVertexTangent);
        }
        desc.attributes[count].location = 3;
        desc.attributes[count].format = prisma::VertexFormat::Float4;
        desc.attributes[count++].offset = MeshStore::attributeOffset(mask, kVertexColor);
        desc.attributes[count].location = 4;
        desc.attributes[count].format = prisma::VertexFormat::Float2;
        desc.attributes[count++].offset =
                (mask & kVertexUv) ? MeshStore::attributeOffset(mask, kVertexUv)
                                   : MeshStore::attributeOffset(mask, kVertexPosition);
        if (detail)
        {
            desc.attributes[count].location = 5;
            desc.attributes[count].format = prisma::VertexFormat::Float2;
            desc.attributes[count++].offset =
                    (mask & kVertexUv2) ? MeshStore::attributeOffset(mask, kVertexUv2)
                                        : MeshStore::attributeOffset(mask, kVertexPosition);
        }
        desc.attributeCount = count;
    }

    static std::uint32_t normalRoughnessVariant(std::uint32_t variant, bool scissor)
    {
        std::uint32_t reduced = variant & (2u | 8u);
        if (scissor) reduced |= 16u | (variant & 1u);
        if (reduced & 8u) reduced |= variant & 64u;
        return reduced;
    }

    prisma::PipelineHandle normalRoughnessPipelineFor(prisma::Driver* driver,
            const SurfaceInfo& surface, std::uint32_t variant, MaterialCull cull,
            std::uint32_t multimeshMode)
    {
        const std::uint64_t key = pipelineKey(surface, variant, cull, false, MaterialBlend::Mix,
                                          true, multimeshMode, 0) |
                                  kNormalRoughnessKey;
        if (const prisma::PipelineHandle* found = pipelines.find(key)) return *found;
        const std::uint32_t normalBit = (variant >> 3) & 1u;
        const bool detail = (variant & 64u) != 0;
        const ShaderVariant* fragmentBlob = kNormalFragmentBlobs[variant];
        const ShaderVariant* vertexBlob =
                multimeshMode != 0 ? kNormalMultiMeshVertexBlobs[normalBit][multimeshMode - 1]
                                   : kNormalVertexBlobs[normalBit | (detail ? 2u : 0u)];
        prisma::ShaderHandle& vertexShader =
                multimeshMode != 0 ? normalMultiMeshVertexShaders[normalBit][multimeshMode - 1]
                                   : normalVertexShaders[normalBit | (detail ? 2u : 0u)];
        prisma::ShaderHandle& fragmentShader = normalFragmentShaders[variant];
        const std::uint64_t start = time_nanos();
        if (!vertexShader.valid() && vertexBlob)
            vertexShader = driver->createShader(prisma3d::shaderDesc(*vertexBlob, driver->caps()));
        if (!fragmentShader.valid() && fragmentBlob)
            fragmentShader =
                    driver->createShader(prisma3d::shaderDesc(*fragmentBlob, driver->caps()));
        if (!vertexShader.valid() || !fragmentShader.valid())
        {
            log_error("render: could not create the normal and roughness shaders");
            pipelines[key] = prisma::PipelineHandle();
            return prisma::PipelineHandle();
        }
        prisma::PipelineDesc desc;
        desc.vertexShader = vertexShader;
        desc.fragmentShader = fragmentShader;
        fillSceneVertexLayout(desc, surface, normalBit != 0, detail);
        desc.topology = surface.topology;
        desc.uniformBlockCount = 3;
        desc.uniformBlocks[0].name = "SceneDataBlock";
        desc.uniformBlocks[0].slot = 0;
        desc.uniformBlocks[1].name = "DrawCall";
        desc.uniformBlocks[1].slot = 1;
        desc.uniformBlocks[2].name = "MaterialUniforms";
        desc.uniformBlocks[2].slot = 2;
        desc.storageBufferCount = 1;
        desc.storageBuffers[0].name = "InstanceDataBuffer";
        desc.storageBuffers[0].slot = 0;
        if (multimeshMode != 0)
        {
            desc.storageBufferCount = 2;
            desc.storageBuffers[1].name = "Transforms";
            desc.storageBuffers[1].slot = 5;
        }
        std::uint32_t textureCount = 0;
        auto addTexture = [&](const char* name, std::uint32_t slot)
        {
            desc.textures[textureCount].name = name;
            desc.textures[textureCount++].slot = slot;
        };
        if (variant & 1u) addTexture("texture_albedo", 0);
        if (variant & 2u) addTexture("texture_roughness", 2);
        if (variant & 8u) addTexture("texture_normal", 4);
        if (variant & 64u)
        {
            addTexture("texture_detail_albedo", 14);
            addTexture("texture_detail_normal", 15);
            addTexture("texture_detail_mask", 16);
        }
        addTexture("best_fit_normal_texture", kBestFitSlot);
        desc.textureCount = textureCount;
        desc.targets.window = false;
        desc.targets.samples = 1;
        desc.targets.colors[0] = ScreenSpaceEffects::normalRoughnessFormat();
        desc.targets.colorCount = 1;
        desc.targets.depth = prisma::TextureFormat::Depth32F;
        desc.depthTest = true;
        desc.depthWrite = true;
        desc.depthCompare = prisma::CompareOp::LessEqual;
        desc.cullMode = cull == MaterialCull::Disabled ? prisma::CullMode::None
                        : cull == MaterialCull::Front  ? prisma::CullMode::Front
                                                       : prisma::CullMode::Back;
        desc.frontFace = prisma::FrontFace::CounterClockwise;
        desc.debugName = "normal roughness pipeline";
        const prisma::PipelineHandle pipeline = driver->createPipeline(desc);
        noteBuild("normal roughness pipeline variant", variant, start);
        if (!pipeline.valid())
            log_error("render: could not create a normal and roughness pipeline");
        pipelines[key] = pipeline;
        return pipeline;
    }

    prisma::PipelineHandle pipelineFor(prisma::Driver* driver, const SurfaceInfo& surface,
            std::uint32_t variant, MaterialCull cull, bool blend, MaterialBlend blendMode,
            bool depthTest, std::uint32_t multimeshMode, std::uint32_t billboard, bool equalDepth,
            bool noShadows, bool coverage, std::uint32_t samples, bool lightmap,
            bool depthOnly = false)
    {
        const std::uint64_t key =
                pipelineKey(surface, variant, cull, blend, blendMode, depthTest, multimeshMode,
                        billboard) |
                (equalDepth ? kEqualDepthKey : 0ull) | (noShadows ? kNoShadowKey : 0ull) |
                (coverage ? kCoverageKey : 0ull) | (lightmap ? kLightmapKey : 0ull) |
                (depthOnly ? kDepthOnlyKey : 0ull) |
                (static_cast<std::uint64_t>(sampleIndex(samples)) << kSamplesKeyShift);
        const std::uint32_t shaderVariant = lightmap ? (variant | 128u) : variant;
        if (const prisma::PipelineHandle* found = pipelines.find(key))
        {
            if (pendingPipelines.empty()) return *found;
            return builtPipeline(driver, key, *found, variant);
        }
        if (!ensureSceneShaders(driver, shaderVariant, multimeshMode == 0 && billboard == 0))
            return prisma::PipelineHandle();
        prisma::ShaderHandle vertexShader = sceneVertexShaders[shaderVariant];
        prisma::ShaderHandle fragmentShader = sceneFragmentShaders[shaderVariant];
        const bool shadowsOff = noShadows && kSceneNoShadowFragmentBlobs[shaderVariant] &&
                                billboard != 4 && billboard != 6;
        if (shadowsOff)
        {
            prisma::ShaderHandle& shader = sceneNoShadowFragmentShaders[shaderVariant];
            if (!shader.valid())
                shader = driver->createShader(prisma3d::shaderDesc(
                        *kSceneNoShadowFragmentBlobs[shaderVariant], driver->caps()));
            if (!shader.valid()) return prisma::PipelineHandle();
            fragmentShader = shader;
        }
        if (coverage)
        {
            fragmentShader = coverageFragmentShader(driver, shaderVariant, shadowsOff);
            if (!fragmentShader.valid()) return prisma::PipelineHandle();
        }
        if (multimeshMode != 0)
        {
            vertexShader = multimeshVertexShader(driver, variant, multimeshMode);
            if (!vertexShader.valid()) return prisma::PipelineHandle();
        }
        if (billboard == 6)
        {
            if (!sceneOceanVertexShader.valid())
                sceneOceanVertexShader = driver->createShader(
                        prisma3d::shaderDesc(*kSceneOceanBlobs[0], driver->caps()));
            if (!sceneOceanFragmentShader.valid())
                sceneOceanFragmentShader = driver->createShader(
                        prisma3d::shaderDesc(*kSceneOceanBlobs[1], driver->caps()));
            vertexShader = sceneOceanVertexShader;
            fragmentShader = sceneOceanFragmentShader;
            if (!vertexShader.valid() || !fragmentShader.valid()) return prisma::PipelineHandle();
        }
        else if (billboard == 3 || billboard == 4)
        {
            vertexShader = terrainVertexShader(driver, variant);
            if (!vertexShader.valid()) return prisma::PipelineHandle();
            if (billboard == 4)
            {
                fragmentShader = terrainFragmentShader(driver, variant);
                if (!fragmentShader.valid()) return prisma::PipelineHandle();
            }
        }
        else if (billboard != 0)
        {
            vertexShader = billboardVertexShader(driver, billboard, variant, multimeshMode);
            if (!vertexShader.valid()) return prisma::PipelineHandle();
        }

        const bool normalMap = (variant & 8u) != 0;
        prisma::PipelineDesc desc;
        desc.vertexShader = vertexShader;
        desc.fragmentShader = fragmentShader;
        fillSceneVertexLayout(desc, surface, normalMap, (variant & 64u) != 0 || lightmap);
        desc.topology = surface.topology;
        desc.uniformBlockCount = 5;
        desc.uniformBlocks[3].name = "LightCountsBlock";
        desc.uniformBlocks[3].slot = 3;
        desc.uniformBlocks[4].name = "DirectionalLights";
        desc.uniformBlocks[4].slot = 4;
        desc.uniformBlocks[0].name = "SceneDataBlock";
        desc.uniformBlocks[0].slot = 0;
        desc.uniformBlocks[1].name = "DrawCall";
        desc.uniformBlocks[1].slot = 1;
        desc.uniformBlocks[2].name = "MaterialUniforms";
        desc.uniformBlocks[2].slot = 2;
        desc.storageBufferCount = 3;
        desc.storageBuffers[0].name = "InstanceDataBuffer";
        desc.storageBuffers[0].slot = 0;
        desc.storageBuffers[1].name = "OmniLights";
        desc.storageBuffers[1].slot = 1;
        desc.storageBuffers[2].name = "SpotLights";
        desc.storageBuffers[2].slot = 2;
        if (!(variant & 32u))
        {
            desc.uniformBlockCount = 6;
            desc.uniformBlocks[5].name = "ImplementationDataBlock";
            desc.uniformBlocks[5].slot = 5;
            desc.storageBufferCount = 5;
            desc.storageBuffers[3].name = "ClusterBuffer";
            desc.storageBuffers[3].slot = 3;
            desc.storageBuffers[4].name = "Decals";
            desc.storageBuffers[4].slot = 4;
        }
        if (multimeshMode != 0)
        {
            desc.storageBuffers[desc.storageBufferCount].name = "Transforms";
            desc.storageBuffers[desc.storageBufferCount].slot = 5;
            ++desc.storageBufferCount;
        }
        if (lightmap)
        {
            desc.storageBuffers[desc.storageBufferCount].name = "Lightmaps";
            desc.storageBuffers[desc.storageBufferCount].slot = 6;
            ++desc.storageBufferCount;
            desc.storageBuffers[desc.storageBufferCount].name = "LightmapCaptures";
            desc.storageBuffers[desc.storageBufferCount].slot = 7;
            ++desc.storageBufferCount;
        }
        std::uint32_t textureCount = 0;
        auto addTexture = [&](const char* name, std::uint32_t slot)
        {
            desc.textures[textureCount].name = name;
            desc.textures[textureCount++].slot = slot;
        };
        if (variant & 32u) addTexture("radiance_octmap", 7);
        if (!(variant & 32u))
        {
            addTexture("dfg", 6);
            addTexture("radiance_octmap", 7);
            if (!shadowsOff)
            {
                addTexture("directional_shadow_atlas", 8);
                addTexture("directional_shadow_depth", 9);
            }
            addTexture("decal_atlas", 10);
            addTexture("decal_atlas_srgb", 11);
            if (!shadowsOff)
            {
                addTexture("shadow_atlas", 12);
                addTexture("shadow_atlas_depth", 13);
            }
        }
        if (variant & 1u) addTexture("texture_albedo", 0);
        if (variant & 2u)
        {
            addTexture("texture_metallic", 1);
            addTexture("texture_roughness", 2);
            addTexture("texture_ambient_occlusion", 5);
        }
        if (variant & 4u) addTexture("texture_emission", 3);
        if (normalMap) addTexture("texture_normal", 4);
        if (billboard == 6)
        {
            addTexture("screen_texture", 18);
            addTexture("depth_texture", 19);
            addTexture("planar_reflection", 20);
            addTexture("ocean_displacement", 21);
            addTexture("ocean_gradient", 22);
            addTexture("ocean_perlin", 23);
            desc.uniformBlocks[desc.uniformBlockCount].name = "PlanarReflections";
            desc.uniformBlocks[desc.uniformBlockCount++].slot = 7;
        }
        else if (billboard != 4 && !(variant & 32u))
        {
            addTexture("texture_refraction", 17);
            addTexture("screen_texture", 18);
            addTexture("depth_texture", 19);
            addTexture("planar_reflection", 20);
            desc.uniformBlocks[desc.uniformBlockCount].name = "PlanarReflections";
            desc.uniformBlocks[desc.uniformBlockCount++].slot = 7;
        }
        if (variant & 64u)
        {
            addTexture("texture_detail_albedo", 14);
            if (normalMap) addTexture("texture_detail_normal", 15);
            addTexture("texture_detail_mask", 16);
        }
        if (!(variant & 32u) && billboard != 4 && billboard != 6)
        {
            addTexture("ao_buffer", kAoSlot);
            addTexture("ssr_buffer", kSsrSlot);
            addTexture("ssr_mip_level_buffer", kSsrMipSlot);
        }
        if (lightmap)
        {
            addTexture("lightmap_light_texture", kLightmapLightSlot);
            addTexture("lightmap_shadowmask_texture", kLightmapShadowmaskSlot);
        }
        if (billboard == 4)
        {
            addTexture("terrain_normal_map", 14);
            addTexture("terrain_lightmap", 15);
            addTexture("terrain_colour_map", 16);
            addTexture("terrain_blend", 17);
            addTexture("terrain_diffuse0", 18);
            addTexture("terrain_diffuse1", 19);
            addTexture("terrain_diffuse2", 20);
            addTexture("terrain_diffuse3", 21);
            addTexture("terrain_diffuse4", 22);
            addTexture("terrain_normalheight0", 23);
            addTexture("terrain_normalheight1", 1);
            addTexture("terrain_normalheight2", 2);
            addTexture("terrain_normalheight3", 3);
            addTexture("terrain_normalheight4", 4);
        }
        desc.textureCount = textureCount;
        desc.targets.window = false;
        desc.targets.samples = samples;
        desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
        desc.targets.colorCount = 1;
        desc.targets.depth = prisma::TextureFormat::Depth32F;
        const bool mixBlend = blend && !coverage;
        desc.depthTest = depthTest;
        desc.depthWrite = depthTest && !mixBlend && !equalDepth;
        desc.depthCompare = equalDepth && !coverage ? prisma::CompareOp::Equal
                            : mixBlend              ? prisma::CompareOp::Less
                                                    : prisma::CompareOp::LessEqual;
        desc.alphaToCoverage = coverage;
        desc.cullMode = cull == MaterialCull::Disabled ? prisma::CullMode::None
                        : cull == MaterialCull::Front  ? prisma::CullMode::Front
                                                       : prisma::CullMode::Back;
        desc.frontFace = prisma::FrontFace::CounterClockwise;
        if (coverage)
        {
            desc.blend = true;
            desc.srcColor = prisma::BlendFactor::SrcAlpha;
            desc.dstColor = prisma::BlendFactor::OneMinusSrcAlpha;
            desc.srcAlpha = prisma::BlendFactor::One;
            desc.dstAlpha = prisma::BlendFactor::Zero;
        }
        else if (blend)
        {
            desc.blend = true;
            switch (blendMode)
            {
                case MaterialBlend::Mix:
                    desc.srcColor = prisma::BlendFactor::SrcAlpha;
                    desc.dstColor = prisma::BlendFactor::OneMinusSrcAlpha;
                    desc.srcAlpha = prisma::BlendFactor::One;
                    desc.dstAlpha = prisma::BlendFactor::OneMinusSrcAlpha;
                    break;
                case MaterialBlend::Add:
                    desc.srcColor = prisma::BlendFactor::SrcAlpha;
                    desc.dstColor = prisma::BlendFactor::One;
                    desc.srcAlpha = prisma::BlendFactor::SrcAlpha;
                    desc.dstAlpha = prisma::BlendFactor::One;
                    break;
                case MaterialBlend::Sub:
                    desc.colorBlendOp = prisma::BlendOp::ReverseSubtract;
                    desc.alphaBlendOp = prisma::BlendOp::ReverseSubtract;
                    desc.srcColor = prisma::BlendFactor::SrcAlpha;
                    desc.dstColor = prisma::BlendFactor::One;
                    desc.srcAlpha = prisma::BlendFactor::SrcAlpha;
                    desc.dstAlpha = prisma::BlendFactor::One;
                    break;
                case MaterialBlend::Mul:
                    desc.srcColor = prisma::BlendFactor::DstColor;
                    desc.dstColor = prisma::BlendFactor::Zero;
                    desc.srcAlpha = prisma::BlendFactor::DstAlpha;
                    desc.dstAlpha = prisma::BlendFactor::Zero;
                    break;
                case MaterialBlend::Screen:
                    desc.srcColor = prisma::BlendFactor::One;
                    desc.dstColor = prisma::BlendFactor::OneMinusSrcColor;
                    desc.srcAlpha = prisma::BlendFactor::One;
                    desc.dstAlpha = prisma::BlendFactor::OneMinusSrcAlpha;
                    break;
            }
        }
        if (depthOnly)
        {
            desc.blend = false;
            desc.colorMask = 0;
            desc.depthWrite = depthTest;
            desc.depthCompare = prisma::CompareOp::LessEqual;
        }
        desc.debugName = "scene pipeline";
        const std::uint64_t start = time_nanos();
        // The first frame waits for its pipelines; later ones draw without the surfaces whose
        // pipeline is still being linked instead of freezing.
        const bool deferred = deferredPipelines && frameNumber > 1;
        const prisma::PipelineHandle pipeline =
                deferred ? driver->createPipelineDeferred(desc) : driver->createPipeline(desc);
        if (!pipeline.valid())
        {
            noteBuild("scene pipeline variant", variant, start);
            log_error("render: could not create a pipeline");
            return pipeline;
        }
        pipelines[key] = pipeline;
        if (deferred && driver->pipelineStatus(pipeline) == prisma::PipelineStatus::Pending)
        {
            pendingPipelines[key] = start;
            return prisma::PipelineHandle();
        }
        noteBuild("scene pipeline variant", variant, start);
        return pipeline;
    }

    prisma::PipelineHandle builtPipeline(prisma::Driver* driver, std::uint64_t key,
            prisma::PipelineHandle pipeline, std::uint32_t variant)
    {
        const std::uint64_t* start = pendingPipelines.find(key);
        if (!start) return pipeline;
        const prisma::PipelineStatus status = driver->pipelineStatus(pipeline);
        if (status == prisma::PipelineStatus::Pending) return prisma::PipelineHandle();
        noteBuild("scene pipeline variant", variant, *start);
        pendingPipelines.erase(key);
        if (status == prisma::PipelineStatus::Ready) return pipeline;
        log_error("render: could not create a pipeline");
        driver->destroy(pipeline);
        pipelines.erase(key);
        return prisma::PipelineHandle();
    }

    bool ensureMaterialBuffer(prisma::Driver* driver, std::uint32_t size)
    {
        if (materialBuffer.valid() && materialBufferSize >= size) return true;
        std::uint32_t grown = materialBufferSize * 2;
        if (grown < size) grown = size;
        if (materialBuffer.valid()) driver->destroy(materialBuffer);
        materialBuffer = prisma::BufferHandle();
        materialBufferSize = 0;
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Uniform;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = grown;
        desc.debugName = "material data";
        materialBuffer = driver->createBuffer(desc);
        if (!materialBuffer.valid())
        {
            log_error("render: could not create the material buffer");
            return false;
        }
        materialBufferSize = grown;
        return true;
    }

    bool ensureUniformBuffer(prisma::Driver* driver, std::uint32_t size)
    {
        if (uniformBuffer.valid() && uniformBufferSize >= size) return true;
        std::uint32_t grown = uniformBufferSize * 2;
        if (grown < size) grown = size;
        if (uniformBuffer.valid()) driver->destroy(uniformBuffer);
        uniformBuffer = prisma::BufferHandle();
        uniformBufferSize = 0;
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Uniform;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = grown;
        desc.debugName = "instance uniforms";
        uniformBuffer = driver->createBuffer(desc);
        if (!uniformBuffer.valid())
        {
            log_error("render: could not create the uniform buffer");
            return false;
        }
        uniformBufferSize = grown;
        return true;
    }

    bool ensureInstanceBuffer(prisma::Driver* driver, std::uint32_t size)
    {
        if (instanceBuffer.valid() && instanceBufferSize >= size) return true;
        std::uint32_t grown = instanceBufferSize * 2;
        if (grown < size) grown = size;
        if (instanceBuffer.valid()) driver->destroy(instanceBuffer);
        instanceBuffer = prisma::BufferHandle();
        instanceBufferSize = 0;
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Storage;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = grown;
        desc.debugName = "instance data";
        instanceBuffer = driver->createBuffer(desc);
        if (!instanceBuffer.valid())
        {
            log_error("render: could not create the instance data buffer");
            return false;
        }
        instanceBufferSize = grown;
        return true;
    }

    prisma::PipelineHandle skyPipelineFor(std::uint32_t variant, std::uint32_t samples) const
    {
        if (samples <= 1) return skyPipelines[variant];
        const prisma::PipelineHandle* found = skyMsaaPipelines.find(variant | (samples << 8));
        return found ? *found : prisma::PipelineHandle();
    }

    bool ensureSky(prisma::Driver* driver, std::uint32_t variant, std::uint32_t samples = 1)
    {
        if (skyPipelineFor(variant, samples).valid()) return true;
        const prisma::Caps& caps = driver->caps();
        if (!skyVertexShader.valid())
            skyVertexShader = driver->createShader(prisma3d::shaderDesc(*kSkyVertexBlob, caps));
        if (!skyFragmentShaders[variant].valid())
            skyFragmentShaders[variant] =
                    driver->createShader(prisma3d::shaderDesc(*kSkyFragmentBlobs[variant], caps));
        if (!skyParamsBuffer.valid())
        {
            prisma::BufferDesc desc;
            desc.usage = prisma::BufferUsage::Uniform;
            desc.update = prisma::BufferUpdate::Dynamic;
            desc.size = sizeof(SkyParamsGpu);
            desc.debugName = "sky params";
            skyParamsBuffer = driver->createBuffer(desc);
            desc.debugName = "sky cubemap params";
            skyCubemapParamsBuffer = driver->createBuffer(desc);
            desc.size = sizeof(SkyDirectionalLightGpu) * kMaxSkyDirectionalLights;
            desc.debugName = "sky lights";
            skyLightsBuffer = driver->createBuffer(desc);
            desc.size = sizeof(SkyMaterialGpu);
            desc.debugName = "sky material";
            skyMaterialBuffer = driver->createBuffer(desc);
            desc.size = sizeof(SkySceneDataGpu);
            desc.debugName = "sky scene";
            skySceneBuffer = driver->createBuffer(desc);
            desc.size = sizeof(SkyParamsGpu);
            desc.debugName = "sky flat params";
            skyFlatParamsBuffer = driver->createBuffer(desc);
            desc.size = 16;
            desc.debugName = "sky flat colour";
            skyFlatColorBuffer = driver->createBuffer(desc);
        }
        prisma::PipelineDesc desc;
        desc.vertexShader = skyVertexShader;
        desc.fragmentShader = skyFragmentShaders[variant];
        desc.topology = prisma::Topology::Triangles;
        desc.uniformBlockCount = 4;
        desc.uniformBlocks[0].name = "SkyParams";
        desc.uniformBlocks[0].slot = 0;
        desc.uniformBlocks[1].name = "DirectionalLights";
        desc.uniformBlocks[1].slot = 1;
        desc.uniformBlocks[2].name = "MaterialUniforms";
        desc.uniformBlocks[2].slot = 2;
        desc.uniformBlocks[3].name = "SkySceneData";
        desc.uniformBlocks[3].slot = 3;
        const std::uint32_t material = variant >> 2;
        if (material == 1 || material == 2)
        {
            desc.textures[0].name = material == 1 ? "source_panorama" : "source_cubemap";
            desc.textures[0].slot = 0;
            desc.textureCount = 1;
        }
        desc.targets.window = false;
        desc.targets.samples = samples;
        desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
        desc.targets.colorCount = 1;
        const bool cubemap = (variant & 2u) != 0;
        desc.targets.depth =
                cubemap ? prisma::TextureFormat::None : prisma::TextureFormat::Depth32F;
        desc.depthTest = !cubemap;
        desc.depthWrite = false;
        desc.depthCompare = prisma::CompareOp::LessEqual;
        desc.cullMode = prisma::CullMode::None;
        desc.debugName = "sky pipeline";
        const std::uint64_t start = time_nanos();
        const prisma::PipelineHandle pipeline = driver->createPipeline(desc);
        noteBuild("sky pipeline variant", variant, start);
        if (!pipeline.valid()) log_error("render: could not create the sky pipeline");
        if (samples <= 1) skyPipelines[variant] = pipeline;
        else if (pipeline.valid())
            skyMsaaPipelines[variant | (samples << 8)] = pipeline;
        return pipeline.valid();
    }

    void collectSkyLights(ScenarioId scenario, SkyDirectionalLightGpu* out) const
    {
        memset(out, 0, sizeof(SkyDirectionalLightGpu) * kMaxSkyDirectionalLights);
        std::uint32_t count = 0;
        for (const Light& light: lights)
        {
            if (!light.visible || light.scenario != scenario ||
                    light.type != LightType::Directional)
                continue;
            if (count >= kMaxSkyDirectionalLights) break;
            SkyDirectionalLightGpu& item = out[count++];
            const Math::Vec3 direction =
                    light.transform.TransformVector(Math::Vec3(0.0f, 0.0f, 1.0f)).Normalized();
            item.direction[0] = direction.x;
            item.direction[1] = direction.y;
            item.direction[2] = direction.z;
            item.energy = (light.negative ? -1.0f : 1.0f) *
                          light.params[static_cast<std::size_t>(LightParam::Energy)];
            item.color[0] = srgbToLinear(light.color.x);
            item.color[1] = srgbToLinear(light.color.y);
            item.color[2] = srgbToLinear(light.color.z);
            item.size = light.params[static_cast<std::size_t>(LightParam::Size)] *
                        (3.14159265358979f / 180.0f);
            item.enabled = 1;
        }
    }

    static constexpr std::uint32_t kSkyFlatVariant = 12;

    static std::uint32_t skyVariant(const EnvironmentSettings& env, std::uint32_t mode,
            bool cubemap)
    {
        return (env.sky.debanding ? 1u : 0u) | (cubemap ? 2u : 0u) | (mode << 2);
    }

    static void fillSkyMaterial(const EnvironmentSettings& env, std::uint32_t mode,
            SkyMaterialGpu& material)
    {
        const ProceduralSkySettings& sky = env.sky;
        memset(&material, 0, sizeof(material));
        auto color = [](const Math::Vec3& value, float energy, float* out)
        {
            out[0] = srgbToLinear(value.x * energy);
            out[1] = srgbToLinear(value.y * energy);
            out[2] = srgbToLinear(value.z * energy);
            out[3] = 1.0f;
        };
        color(sky.skyTopColor, sky.skyEnergy, material.skyTopColor);
        color(sky.skyHorizonColor, sky.skyEnergy, material.skyHorizonColor);
        color(sky.groundBottomColor, sky.groundEnergy, material.groundBottomColor);
        color(sky.groundHorizonColor, sky.groundEnergy, material.groundHorizonColor);
        material.invSkyCurve = 0.6f / sky.skyCurve;
        material.invGroundCurve = 0.6f / sky.groundCurve;
        material.sunAngleMax = cosf(sky.sunAngleMax * (3.14159265358979f / 180.0f));
        material.invSunCurve = 1.6f / powf(sky.sunCurve, 1.4f);
        material.exposure = mode == 0 ? sky.energy : env.skyTextureEnergy;
    }

    static void fillSkyScene(const EnvironmentSettings& env, float zFar,
            const SkyDirectionalLightGpu* lights, SkySceneDataGpu& out)
    {
        memset(&out, 0, sizeof(out));
        for (int i = 0; i < 2; ++i)
        {
            out.combinedReprojection[i][0] = 1.0f;
            out.combinedReprojection[i][5] = 1.0f;
            out.combinedReprojection[i][10] = 1.0f;
            out.combinedReprojection[i][15] = 1.0f;
        }
        out.fogEnabled = env.fogEnabled ? 1u : 0u;
        out.fogDensity = env.fogDensity;
        out.fogAerialPerspective = env.fogAerialPerspective;
        out.fogLightColor[0] = srgbToLinear(env.fogLightColor.x) * env.fogLightEnergy;
        out.fogLightColor[1] = srgbToLinear(env.fogLightColor.y) * env.fogLightEnergy;
        out.fogLightColor[2] = srgbToLinear(env.fogLightColor.z) * env.fogLightEnergy;
        out.fogSunScatter = env.fogSunScatter;
        out.fogSkyAffect = env.fogSkyAffect;
        out.zFar = zFar;
        for (std::uint32_t i = 0; i < kMaxSkyDirectionalLights; ++i)
            if (lights[i].enabled) ++out.directionalLightCount;
    }

    static void fillSkyParams(const Camera& camera, const Math::Mat4& baseProjection, float time,
            float brightness, SkyParamsGpu& params)
    {
        memset(&params, 0, sizeof(params));
        for (int c = 0; c < 3; ++c)
        {
            const Math::Vec4& column =
                    c == 0 ? camera.transform.col0
                           : (c == 1 ? camera.transform.col1 : camera.transform.col2);
            params.orientation[c][0] = column.x;
            params.orientation[c][1] = column.y;
            params.orientation[c][2] = column.z;
        }
        params.projection[0] = baseProjection.col2.x;
        params.projection[1] = baseProjection.col0.x;
        params.projection[2] = baseProjection.col2.y;
        params.projection[3] = baseProjection.col1.y;
        const Math::Vec3 origin = camera.transform.GetTranslation();
        params.position[0] = origin.x;
        params.position[1] = origin.y;
        params.position[2] = origin.z;
        params.time = time;
        params.borderSize[1] = 1.0f;
        params.luminanceMultiplier = 1.0f;
        params.brightnessMultiplier = brightness;
    }

    void uploadSky(prisma::Driver* driver, ScenarioId scenario, const EnvironmentSettings& env,
            std::uint32_t mode, const Camera& camera, const Math::Mat4& baseProjection, float time,
            float border)
    {
        SkyParamsGpu params;
        fillSkyParams(camera, baseProjection, time, env.backgroundEnergy, params);
        driver->updateBuffer(skyParamsBuffer, 0, &params, sizeof(params));
        const Math::Vec3 origin = camera.transform.GetTranslation();

        SkyParamsGpu cubemap;
        memset(&cubemap, 0, sizeof(cubemap));
        cubemap.orientation[0][0] = cubemap.orientation[1][1] = cubemap.orientation[2][2] = 1.0f;
        cubemap.projection[1] = cubemap.projection[3] = 1.0f;
        cubemap.position[0] = origin.x;
        cubemap.position[1] = origin.y;
        cubemap.position[2] = origin.z;
        cubemap.time = time;
        cubemap.borderSize[0] = border;
        cubemap.borderSize[1] = 1.0f - border * 2.0f;
        cubemap.luminanceMultiplier = 1.0f;
        cubemap.brightnessMultiplier = env.backgroundEnergy;
        driver->updateBuffer(skyCubemapParamsBuffer, 0, &cubemap, sizeof(cubemap));

        SkyDirectionalLightGpu lightData[kMaxSkyDirectionalLights];
        collectSkyLights(scenario, lightData);
        driver->updateBuffer(skyLightsBuffer, 0, lightData, sizeof(lightData));

        SkySceneDataGpu scene;
        fillSkyScene(env, camera.zFar, lightData, scene);
        driver->updateBuffer(skySceneBuffer, 0, &scene, sizeof(scene));

        SkyMaterialGpu material;
        fillSkyMaterial(env, mode, material);
        driver->updateBuffer(skyMaterialBuffer, 0, &material, sizeof(material));
    }

    void uploadFlatSky(prisma::Driver* driver, ScenarioId scenario, const EnvironmentSettings& env,
            const Camera& camera, const Math::Mat4& baseProjection, const float* color)
    {
        SkyParamsGpu params;
        fillSkyParams(camera, baseProjection, 0.0f, 1.0f, params);
        driver->updateBuffer(skyFlatParamsBuffer, 0, &params, sizeof(params));

        const float flat[4] = { color[0], color[1], color[2], 1.0f };
        driver->updateBuffer(skyFlatColorBuffer, 0, flat, sizeof(flat));

        SkyDirectionalLightGpu lightData[kMaxSkyDirectionalLights];
        collectSkyLights(scenario, lightData);
        driver->updateBuffer(skyLightsBuffer, 0, lightData, sizeof(lightData));

        SkySceneDataGpu scene;
        fillSkyScene(env, camera.zFar, lightData, scene);
        driver->updateBuffer(skySceneBuffer, 0, &scene, sizeof(scene));
    }

    struct OctmapPass
    {
        prisma::TextureHandle target;
        prisma::TextureHandle source;
        prisma::PipelineHandle pipeline;
        std::uint32_t mip = 0;
        std::uint32_t layer = 0;
        std::uint32_t size = 0;
        std::uint32_t offset = 0;
        std::uint32_t bytes = 0;
        bool sky = false;
    };

    bool ensureOctmap(prisma::Driver* driver)
    {
        if (octmapReady) return true;
        if (octmapFailed) return false;
        octmapFailed = true;
        const prisma::Caps& caps = driver->caps();

        prisma::SamplerDesc samplerDesc;
        samplerDesc.minFilter = prisma::Filter::Linear;
        samplerDesc.magFilter = prisma::Filter::Linear;
        samplerDesc.mipFilter = prisma::MipFilter::Linear;
        samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
        samplerDesc.debugName = "linear mip clamp";
        mipSampler = driver->createSampler(samplerDesc);
        if (!skySampler.valid())
        {
            prisma::SamplerDesc skyDesc;
            skyDesc.minFilter = prisma::Filter::Linear;
            skyDesc.magFilter = prisma::Filter::Linear;
            skyDesc.mipFilter = prisma::MipFilter::None;
            skyDesc.addressU = prisma::AddressMode::Repeat;
            skyDesc.addressV = prisma::AddressMode::ClampToEdge;
            skyDesc.addressW = prisma::AddressMode::ClampToEdge;
            skyDesc.debugName = "sky source";
            skySampler = driver->createSampler(skyDesc);
        }

        static const std::uint16_t black[4] = { 0, 0, 0, 0 };
        prisma::TextureDesc dummyDesc;
        dummyDesc.type = prisma::TextureType::Texture2DArray;
        dummyDesc.format = prisma::TextureFormat::RGBA16F;
        dummyDesc.width = 1;
        dummyDesc.height = 1;
        dummyDesc.depth = 1;
        dummyDesc.data = black;
        dummyDesc.debugName = "dummy radiance";
        dummyRadiance = driver->createTexture(dummyDesc);

        octmapVertexShader = driver->createShader(prisma3d::shaderDesc(octmap_vert, caps));
        octmapRoughnessShader =
                driver->createShader(prisma3d::shaderDesc(octmap_roughness_frag, caps));
        for (std::uint32_t i = 0; i < 2; ++i)
            octmapDownsampleShaders[i] =
                    driver->createShader(prisma3d::shaderDesc(*kOctmapDownsampleBlobs[i], caps));

        auto make = [&](prisma::ShaderHandle fragment, const char* texture, const char* name)
        {
            prisma::PipelineDesc desc;
            desc.vertexShader = octmapVertexShader;
            desc.fragmentShader = fragment;
            desc.topology = prisma::Topology::Triangles;
            desc.uniformBlockCount = 1;
            desc.uniformBlocks[0].name = "OctmapParams";
            desc.uniformBlocks[0].slot = 0;
            desc.textureCount = 1;
            desc.textures[0].name = texture;
            desc.textures[0].slot = 0;
            desc.targets.window = false;
            desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
            desc.targets.colorCount = 1;
            desc.targets.depth = prisma::TextureFormat::None;
            desc.depthTest = false;
            desc.depthWrite = false;
            desc.cullMode = prisma::CullMode::None;
            desc.debugName = name;
            return driver->createPipeline(desc);
        };
        octmapDownsamplePipelines[0] =
                make(octmapDownsampleShaders[0], "source_octmap", "octmap downsample");
        octmapDownsamplePipelines[1] =
                make(octmapDownsampleShaders[1], "source_octmap", "octmap downsample array");
        octmapRoughnessPipeline = make(octmapRoughnessShader, "source_oct", "octmap roughness");

        octmapReady = mipSampler.valid() && dummyRadiance.valid() &&
                      octmapDownsamplePipelines[0].valid() &&
                      octmapDownsamplePipelines[1].valid() && octmapRoughnessPipeline.valid();
        if (octmapReady) octmapFailed = false;
        else
            log_error("render: could not create the radiance pipelines");
        return octmapReady;
    }

    static std::uint32_t radianceMips(std::uint32_t size)
    {
        std::uint32_t levels = 0;
        for (std::uint32_t value = size; value > 1; value >>= 1) ++levels;
        return levels > 1 ? levels - 1 : 1;
    }

    static void freeRadiance(prisma::Driver* driver, Scenario::Radiance& radiance)
    {
        if (radiance.array.valid()) driver->destroy(radiance.array);
        if (radiance.downsampled.valid()) driver->destroy(radiance.downsampled);
        radiance = Scenario::Radiance();
    }

    bool prepareRadiance(prisma::Driver* driver, Scenario& scenario)
    {
        if (!ensureOctmap(driver)) return false;
        Scenario::Radiance& radiance = scenario.radiance;
        std::uint32_t size = scenario.environment.sky.radianceSize;
        size = size < 32 ? 32 : (size > 2048 ? 2048 : size);
        if (radiance.array.valid() && radiance.size == size) return true;
        freeRadiance(driver, radiance);

        radiance.size = size;
        radiance.mips = radianceMips(size);
        const std::uint32_t padding = 1u << (radiance.mips - 1);
        radiance.width = size * 2 + padding * 2;
        radiance.border = static_cast<float>(padding) / static_cast<float>(radiance.width);

        prisma::TextureDesc arrayDesc;
        arrayDesc.type = prisma::TextureType::Texture2DArray;
        arrayDesc.format = prisma::TextureFormat::RGBA16F;
        arrayDesc.width = radiance.width;
        arrayDesc.height = radiance.width;
        arrayDesc.depth = kRadianceLayers;
        arrayDesc.mipLevels = radiance.mips;
        arrayDesc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        arrayDesc.debugName = "radiance octmap";
        radiance.array = driver->createTexture(arrayDesc);

        prisma::TextureDesc downsampledDesc;
        downsampledDesc.format = prisma::TextureFormat::RGBA16F;
        downsampledDesc.width = radiance.width >> 1;
        downsampledDesc.height = radiance.width >> 1;
        downsampledDesc.mipLevels = radiance.mips > 1 ? radiance.mips - 1 : 1;
        downsampledDesc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        downsampledDesc.debugName = "downsampled radiance octmap";
        radiance.downsampled = driver->createTexture(downsampledDesc);

        if (!radiance.array.valid() || !radiance.downsampled.valid())
        {
            log_error("render: could not create the radiance octmap");
            freeRadiance(driver, radiance);
            return false;
        }
        return true;
    }

    bool runRadiance(prisma::Driver* driver, ScenarioId id, Scenario& scenario)
    {
        Scenario::Radiance& radiance = scenario.radiance;
        const EnvironmentSettings& env = scenario.environment;

        struct Signature
        {
            SkyMaterialGpu material;
            float backgroundEnergy;
            float debanding;
            std::uint32_t mode;
            std::uint32_t texture;
            float fog[8];
            SkyDirectionalLightGpu lights[kMaxSkyDirectionalLights];
        };
        Signature signature;
        memset(&signature, 0, sizeof(signature));
        fillSkyMaterial(env, scenario.skyMode, signature.material);
        signature.mode = scenario.skyMode;
        signature.texture = scenario.skyMode != 0 ? env.skyTexture.bits() : 0u;
        signature.backgroundEnergy = env.backgroundEnergy;
        signature.debanding = env.sky.debanding ? 1.0f : 0.0f;
        collectSkyLights(id, signature.lights);
        {
            SkySceneDataGpu fog;
            fillSkyScene(env, 0.0f, signature.lights, fog);
            signature.fog[0] = static_cast<float>(fog.fogEnabled);
            signature.fog[1] = fog.fogSkyAffect;
            signature.fog[2] = fog.fogSunScatter;
            signature.fog[3] = fog.fogAerialPerspective;
            signature.fog[4] = fog.fogLightColor[0];
            signature.fog[5] = fog.fogLightColor[1];
            signature.fog[6] = fog.fogLightColor[2];
        }
        if (radiance.signature.size() != sizeof(signature) ||
                memcmp(radiance.signature.data(), &signature, sizeof(signature)) != 0)
        {
            radiance.signature.resize(sizeof(signature));
            memcpy(radiance.signature.data(), &signature, sizeof(signature));
            radiance.dirty = true;
        }

        const std::uint32_t skyVariant = State::skyVariant(env, scenario.skyMode, true);
        if (!ensureSky(driver, skyVariant)) return false;

        const std::uint32_t stride = alignUp(sizeof(OctmapRoughnessParamsGpu),
                driver->caps().uniformBufferOffsetAlignment);
        ct::Vector<OctmapPass> passes;
        ct::Vector<std::uint8_t> bytes;
        const std::uint32_t width = radiance.width;
        const std::uint32_t downsampledWidth = width >> 1;
        const std::uint32_t downsampledMips = radiance.mips > 1 ? radiance.mips - 1 : 1;
        auto mipSize = [](std::uint32_t base, std::uint32_t mip)
        {
            const std::uint32_t value = base >> mip;
            return value < 1 ? 1u : value;
        };
        auto addBytes = [&](const void* data, std::uint32_t size)
        {
            const std::uint32_t offset = static_cast<std::uint32_t>(bytes.size());
            bytes.resize(offset + stride);
            memset(bytes.data() + offset, 0, stride);
            memcpy(bytes.data() + offset, data, size);
            return offset;
        };
        auto addDownsample = [&](prisma::TextureHandle target, std::uint32_t mip,
                                     std::uint32_t layer, std::uint32_t size,
                                     prisma::TextureHandle source, bool arraySource,
                                     std::uint32_t sourceLod, std::uint32_t sourceLayer)
        {
            OctmapDownsampleParamsGpu params;
            params.borderSize = 1.0f - radiance.border * 2.0f;
            params.size = size;
            params.sourceLod = sourceLod;
            params.sourceLayer = sourceLayer;
            OctmapPass pass;
            pass.target = target;
            pass.source = source;
            pass.pipeline = octmapDownsamplePipelines[arraySource ? 1 : 0];
            pass.mip = mip;
            pass.layer = layer;
            pass.size = size;
            pass.bytes = sizeof(params);
            pass.offset = addBytes(&params, sizeof(params));
            passes.push_back(pass);
        };
        auto addImportance = [&](std::uint32_t layer)
        {
            if (layer == 1)
            {
                addDownsample(radiance.downsampled, 0, 0, downsampledWidth, radiance.array, true, 0,
                        0);
                for (std::uint32_t mip = 1; mip < downsampledMips; ++mip)
                    addDownsample(radiance.downsampled, mip, 0, mipSize(downsampledWidth, mip),
                            radiance.downsampled, false, mip - 1, 0);
            }
            const float perceptual =
                    static_cast<float>(layer) / static_cast<float>(kRadianceLayers - 1);
            OctmapRoughnessParamsGpu params;
            memset(&params, 0, sizeof(params));
            params.roughness = perceptual * perceptual;
            const float scaled = static_cast<float>(kRadianceSamples * 4u) * params.roughness;
            params.sampleCount = static_cast<std::uint32_t>(scaled) > 4u
                                         ? static_cast<std::uint32_t>(scaled)
                                         : 4u;
            params.sourceSize = downsampledWidth;
            params.destSize = width;
            params.useDirectWrite = perceptual == 0.0f ? 1u : 0u;
            params.borderSize[0] = radiance.border;
            params.borderSize[1] = 1.0f - radiance.border * 2.0f;
            OctmapPass pass;
            pass.target = radiance.array;
            pass.source = radiance.downsampled;
            pass.pipeline = octmapRoughnessPipeline;
            pass.mip = 0;
            pass.layer = layer;
            pass.size = width;
            pass.bytes = sizeof(params);
            pass.offset = addBytes(&params, sizeof(params));
            passes.push_back(pass);
        };
        auto addMipmaps = [&](std::uint32_t first, std::uint32_t last)
        {
            for (std::uint32_t layer = first; layer < last; ++layer)
                for (std::uint32_t mip = 0; mip + 1 < radiance.mips; ++mip)
                    addDownsample(radiance.array, mip + 1, layer, mipSize(width, mip + 1),
                            radiance.array, true, mip, layer);
        };

        const bool single = radiance.processing == 0;
        if (radiance.dirty && (radiance.processing >= kRadianceLayers || single))
        {
            OctmapPass sky;
            sky.target = radiance.array;
            sky.pipeline = skyPipelines[skyVariant];
            sky.size = width;
            sky.sky = true;
            passes.push_back(sky);
            if (single)
            {
                for (std::uint32_t layer = 1; layer < kRadianceLayers; ++layer)
                    addImportance(layer);
                addMipmaps(0, kRadianceLayers);
                radiance.processing = kRadianceLayers;
            }
            else
            {
                addMipmaps(0, 1);
                radiance.processing = 1;
            }
            radiance.dirty = false;
        }
        else if (radiance.processing < kRadianceLayers)
        {
            addImportance(radiance.processing);
            addMipmaps(radiance.processing, radiance.processing + 1);
            ++radiance.processing;
        }
        if (passes.empty()) return true;

        const std::uint32_t total = static_cast<std::uint32_t>(bytes.size());
        if (total > 0)
        {
            if (!ensureStorage(driver, octmapParamsBuffer, octmapParamsSize, total, "octmap params",
                        prisma::BufferUsage::Uniform))
                return false;
            driver->updateBuffer(octmapParamsBuffer, 0, bytes.data(), total);
        }

        for (const OctmapPass& pass: passes)
        {
            prisma::RenderPassDesc desc;
            desc.colors[0].texture = pass.target;
            desc.colors[0].mip = pass.mip;
            desc.colors[0].layer = pass.layer;
            desc.colorCount = 1;
            desc.colorLoad = prisma::LoadOp::DontCare;
            driver->beginRenderPass(desc);
            prisma::Viewport area;
            area.width = static_cast<float>(pass.size);
            area.height = static_cast<float>(pass.size);
            driver->setViewport(area);
            driver->bindPipeline(pass.pipeline);
            if (pass.sky)
            {
                driver->bindUniformBuffer(0, skyCubemapParamsBuffer, 0, sizeof(SkyParamsGpu));
                driver->bindUniformBuffer(1, skyLightsBuffer, 0,
                        sizeof(SkyDirectionalLightGpu) * kMaxSkyDirectionalLights);
                driver->bindUniformBuffer(2, skyMaterialBuffer, 0, sizeof(SkyMaterialGpu));
                driver->bindUniformBuffer(3, skySceneBuffer, 0, sizeof(SkySceneDataGpu));
                if (scenario.skyMode != 0) driver->bindTexture(0, scenario.skyGpu, skySampler);
            }
            else
            {
                driver->bindUniformBuffer(0, octmapParamsBuffer, pass.offset, pass.bytes);
                driver->bindTexture(0, pass.source, mipSampler);
            }
            driver->draw(3, 0);
            driver->endRenderPass();
        }
        return true;
    }

    static float snapped(float value, float step)
    {
        return step != 0.0f ? floorf(value / step + 0.5f) * step : value;
    }

    void computeCascades(Light& light, const Camera& camera, float aspect, float textureSize)
    {
        const float* p = light.params;
        auto param = [&](LightParam value) { return p[static_cast<std::size_t>(value)]; };
        const Math::Vec3 xv = light.transform.col0.xyz().Normalized();
        const Math::Vec3 yv = light.transform.col1.xyz().Normalized();
        const Math::Vec3 zv = light.transform.col2.xyz().Normalized();
        const Math::Vec3 camRight = camera.transform.col0.xyz().Normalized();
        const Math::Vec3 camUp = camera.transform.col1.xyz().Normalized();
        const Math::Vec3 camBack = camera.transform.col2.xyz().Normalized();
        const Math::Vec3 camOrigin = camera.transform.GetTranslation();
        const bool orthogonal = camera.projection == CameraProjection::Orthogonal;

        float maxDistance = camera.zFar;
        const float shadowMax = param(LightParam::ShadowMaxDistance);
        if (shadowMax > 0.0f && !orthogonal) maxDistance = fminf(shadowMax, maxDistance);
        maxDistance = fmaxf(maxDistance, camera.zNear + 0.001f);
        const float minDistance = fminf(camera.zNear, maxDistance);
        const float pancake = param(LightParam::ShadowPancakeSize);
        const float range = maxDistance - minDistance;

        int splits = 4;
        if (light.shadowMode == DirectionalShadowMode::Orthogonal) splits = 1;
        else if (light.shadowMode == DirectionalShadowMode::Parallel2Splits)
            splits = 2;

        float distances[5];
        distances[0] = minDistance;
        for (int i = 0; i < splits; ++i)
        {
            const float offset = i < 3 ? p[static_cast<std::size_t>(LightParam::ShadowSplit1) +
                                                 static_cast<std::size_t>(i)]
                                       : 1.0f;
            distances[i + 1] = minDistance + offset * range;
        }
        distances[splits] = maxDistance;

        const bool overlap = light.blendSplits;
        light.cascadeCount = static_cast<std::uint32_t>(splits);
        const float tanHalf = tanf(camera.fovy * 0.5f * (3.14159265358979f / 180.0f));

        for (int i = 0; i < splits; ++i)
        {
            const float nearDistance = distances[(i == 0 || !overlap) ? i : i - 1];
            const float farDistance = distances[i + 1];
            Math::Vec3 endpoints[8];
            for (int e = 0; e < 8; ++e)
            {
                const float distance = (e & 4) ? farDistance : nearDistance;
                const float halfHeight = orthogonal ? camera.size * 0.5f : distance * tanHalf;
                const float halfWidth = halfHeight * aspect;
                const float sx = (e & 1) ? 1.0f : -1.0f;
                const float sy = (e & 2) ? 1.0f : -1.0f;
                endpoints[e] = camOrigin - camBack * distance + camRight * (sx * halfWidth) +
                               camUp * (sy * halfHeight);
            }

            float zMin = 0.0f;
            for (int e = 0; e < 8; ++e)
            {
                const float dz = Math::Vec3::Dot(zv, endpoints[e]);
                if (e == 0 || dz < zMin) zMin = dz;
            }
            (void) zMin;

            Math::Vec3 center(0.0f, 0.0f, 0.0f);
            for (int e = 0; e < 8; ++e) center = center + endpoints[e];
            center = center * (1.0f / 8.0f);
            float radius = 0.0f;
            for (int e = 0; e < 8; ++e)
            {
                const float distance = (endpoints[e] - center).Length();
                if (distance > radius) radius = distance;
            }
            radius *= textureSize / (textureSize - 2.0f);

            const float zMinCam = Math::Vec3::Dot(zv, center) - radius;
            float softExpand = 0.0f;
            const float softAngle = param(LightParam::Size);
            if (softAngle > 0.0f)
            {
                const float zRange = (Math::Vec3::Dot(zv, center) + radius + pancake) - zMinCam;
                softExpand = tanf(softAngle * (3.14159265358979f / 180.0f)) * zRange;
            }
            const float unit = (radius + softExpand) * 4.0f / textureSize;
            const float xMaxCam = snapped(Math::Vec3::Dot(xv, center) + radius + softExpand, unit);
            const float xMinCam = snapped(Math::Vec3::Dot(xv, center) - radius - softExpand, unit);
            const float yMaxCam = snapped(Math::Vec3::Dot(yv, center) + radius + softExpand, unit);
            const float yMinCam = snapped(Math::Vec3::Dot(yv, center) - radius - softExpand, unit);
            const float zMax = Math::Vec3::Dot(zv, center) + radius + pancake;

            const float halfX = (xMaxCam - xMinCam) * 0.5f;
            const float halfY = (yMaxCam - yMinCam) * 0.5f;
            Light::Cascade& cascade = light.cascades[i];
            cascade.projection =
                    Math::Mat4::Ortho(-halfX, halfX, -halfY, halfY, 0.0f, zMax - zMinCam);
            cascade.transform = Math::Mat4(Math::Mat3(xv, yv, zv),
                    xv * (xMinCam + halfX) + yv * (yMinCam + halfY) + zv * zMax);
            float sliceMinX = 0.0f;
            float sliceMaxX = 0.0f;
            float sliceMinY = 0.0f;
            float sliceMaxY = 0.0f;
            for (int e = 0; e < 8; ++e)
            {
                const float px = Math::Vec3::Dot(xv, endpoints[e]);
                const float py = Math::Vec3::Dot(yv, endpoints[e]);
                if (e == 0 || px < sliceMinX) sliceMinX = px;
                if (e == 0 || px > sliceMaxX) sliceMaxX = px;
                if (e == 0 || py < sliceMinY) sliceMinY = py;
                if (e == 0 || py > sliceMaxY) sliceMaxY = py;
            }
            const float cullMargin = softExpand + radius * 2.0f / textureSize * 16.0f;
            const float centerX = xMinCam + halfX;
            const float centerY = yMinCam + halfY;
            const float cullLeft = fmaxf(sliceMinX - cullMargin - centerX, -halfX);
            const float cullRight = fminf(sliceMaxX + cullMargin - centerX, halfX);
            const float cullBottom = fmaxf(sliceMinY - cullMargin - centerY, -halfY);
            const float cullTop = fminf(sliceMaxY + cullMargin - centerY, halfY);
            const Math::Mat4 cullProjection = Math::Mat4::Ortho(cullLeft, cullRight, cullBottom,
                    cullTop, -1.0e6f, zMax - zMinCam);
            cascade.frustum = Math::Frustum::FromViewProjection(
                    cullProjection * cascade.transform.InverseAffine());
            cascade.zFar = zMax - zMinCam;
            cascade.split = farDistance;
            cascade.texelSize = radius * 2.0f / textureSize;
            cascade.biasScale = zMax - zMinCam;
            cascade.rangeBegin = zMax - Math::Vec3::Dot(zv, camOrigin);
            cascade.uvScale[0] = 1.0f / (xMaxCam - xMinCam);
            cascade.uvScale[1] = 1.0f / (yMaxCam - yMinCam);
        }
    }

    static void directionalShadowRect(std::uint32_t atlas, std::uint32_t count, std::uint32_t index,
            std::uint32_t* out)
    {
        std::uint32_t splitH = 1;
        std::uint32_t splitV = 1;
        while (splitH * splitV < count)
        {
            if (splitH == splitV) splitH <<= 1;
            else
                splitV <<= 1;
        }
        out[2] = atlas / splitH;
        out[3] = atlas / splitV;
        out[0] = out[2] * (index % splitH);
        out[1] = out[3] * (index / splitH);
    }

    bool ensureShadowResources(prisma::Driver* driver)
    {
        if (shadowReady) return true;
        if (shadowFailed) return false;
        shadowFailed = true;
        const prisma::Caps& caps = driver->caps();
        prisma::SamplerDesc samplerDesc;
        samplerDesc.minFilter = prisma::Filter::Linear;
        samplerDesc.magFilter = prisma::Filter::Linear;
        samplerDesc.mipFilter = prisma::MipFilter::None;
        samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
        samplerDesc.compare = true;
        samplerDesc.compareOp = prisma::CompareOp::LessEqual;
        samplerDesc.debugName = "shadow";
        shadowSampler = driver->createSampler(samplerDesc);
        samplerDesc.compare = false;
        samplerDesc.debugName = "shadow depth";
        shadowDepthSampler = driver->createSampler(samplerDesc);

        samplerDesc.minFilter = prisma::Filter::Nearest;
        samplerDesc.magFilter = prisma::Filter::Nearest;
        samplerDesc.compare = true;
        samplerDesc.compareOp = prisma::CompareOp::Greater;
        samplerDesc.debugName = "positional shadow";
        positionalShadowSampler = driver->createSampler(samplerDesc);
        samplerDesc.minFilter = prisma::Filter::Linear;
        samplerDesc.magFilter = prisma::Filter::Linear;
        samplerDesc.compare = false;
        samplerDesc.debugName = "positional shadow depth";
        positionalShadowDepthSampler = driver->createSampler(samplerDesc);

        prisma::TextureDesc dummyDesc;
        dummyDesc.format = prisma::TextureFormat::Depth32F;
        dummyDesc.width = 1;
        dummyDesc.height = 1;
        dummyDesc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        dummyDesc.debugName = "dummy shadow";
        dummyShadow = driver->createTexture(dummyDesc);

        shadowVertexShader = driver->createShader(prisma3d::shaderDesc(shadow_vert, caps));
        shadowFragmentShader = driver->createShader(prisma3d::shaderDesc(shadow_frag, caps));
        shadowDualVertexShader = driver->createShader(prisma3d::shaderDesc(shadow_dp_vert, caps));
        shadowDualFragmentShader = driver->createShader(prisma3d::shaderDesc(shadow_dp_frag, caps));
        shadowReady = shadowSampler.valid() && shadowDepthSampler.valid() &&
                      positionalShadowSampler.valid() && positionalShadowDepthSampler.valid() &&
                      dummyShadow.valid() && shadowVertexShader.valid() &&
                      shadowFragmentShader.valid() && shadowDualVertexShader.valid() &&
                      shadowDualFragmentShader.valid();
        if (shadowReady) shadowFailed = false;
        else
            log_error("render: could not create the shadow resources");
        if (shadowReady) createCubeCopy(driver);
        return shadowReady;
    }

    void createCubeCopy(prisma::Driver* driver)
    {
        cubeCopyFragmentShader =
                driver->createShader(prisma3d::shaderDesc(cube_to_dp_frag, driver->caps()));
        if (!cubeCopyFragmentShader.valid() || !tonemapVertex.valid()) return;
        prisma::PipelineDesc desc;
        desc.vertexShader = tonemapVertex;
        desc.fragmentShader = cubeCopyFragmentShader;
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "Params";
        desc.uniformBlocks[0].slot = 0;
        desc.textureCount = 1;
        desc.textures[0].name = "source_cube";
        desc.textures[0].slot = 0;
        desc.targets.window = false;
        desc.targets.colorCount = 0;
        desc.targets.depth = prisma::TextureFormat::Depth32F;
        desc.depthTest = true;
        desc.depthWrite = true;
        desc.depthCompare = prisma::CompareOp::Always;
        desc.cullMode = prisma::CullMode::None;
        desc.debugName = "cube to dual paraboloid";
        cubeCopyPipeline = driver->createPipeline(desc);
        cubeShadowsReady = cubeCopyPipeline.valid();
        if (!cubeShadowsReady) log_error("render: could not create the cube shadow copy");
    }

    prisma::TextureHandle shadowCube(prisma::Driver* driver, std::uint32_t size)
    {
        if (const prisma::TextureHandle* found = shadowCubes.find(size)) return *found;
        prisma::TextureDesc desc;
        desc.type = prisma::TextureType::TextureCube;
        desc.format = prisma::TextureFormat::Depth32F;
        desc.width = size;
        desc.height = size;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "shadow cube";
        const prisma::TextureHandle texture = driver->createTexture(desc);
        if (!texture.valid()) log_error("render: could not create a shadow cube");
        shadowCubes[size] = texture;
        return texture;
    }

    bool ensureShadowAtlas(prisma::Driver* driver)
    {
        if (shadowAtlas.valid()) return true;
        prisma::TextureDesc desc;
        desc.format = prisma::TextureFormat::Depth32F;
        desc.width = shadowAtlasSize;
        desc.height = shadowAtlasSize;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "directional shadow atlas";
        shadowAtlas = driver->createTexture(desc);
        if (!shadowAtlas.valid())
            log_error("render: could not create the directional shadow atlas");
        return shadowAtlas.valid();
    }

    static void destroyPositionalAtlas(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.shadowAtlasTexture.valid()) driver->destroy(viewport.shadowAtlasTexture);
        viewport.shadowAtlasTexture = prisma::TextureHandle();
        viewport.shadowAtlasTextureSize = 0;
    }

    static bool ensurePositionalAtlas(prisma::Driver* driver, Viewport& viewport)
    {
        const std::uint32_t size = viewport.shadowAtlas.getSize();
        if (viewport.shadowAtlasTexture.valid() && viewport.shadowAtlasTextureSize == size)
            return true;
        destroyPositionalAtlas(driver, viewport);
        if (size == 0) return false;
        prisma::TextureDesc desc;
        desc.format = prisma::TextureFormat::Depth32F;
        desc.width = size;
        desc.height = size;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "positional shadow atlas";
        viewport.shadowAtlasTexture = driver->createTexture(desc);
        if (!viewport.shadowAtlasTexture.valid())
        {
            log_error("render: could not create the positional shadow atlas");
            return false;
        }
        viewport.shadowAtlasTextureSize = size;
        return true;
    }

    prisma::ShaderHandle depthVertexShader(prisma::Driver* driver, bool dual,
            std::uint32_t multimeshMode)
    {
        prisma::ShaderHandle& shader = depthVertexShaders[dual ? 1 : 0][multimeshMode];
        if (!shader.valid())
            shader = driver->createShader(prisma3d::shaderDesc(
                    *kDepthVertexBlobs[dual ? 1 : 0][multimeshMode], driver->caps()));
        return shader;
    }

    prisma::ShaderHandle depthFragmentShader(prisma::Driver* driver, bool dual, bool albedo)
    {
        prisma::ShaderHandle& shader = depthFragmentShaders[dual ? 1 : 0][albedo ? 1 : 0];
        if (!shader.valid())
            shader = driver->createShader(prisma3d::shaderDesc(
                    *kDepthFragmentBlobs[dual ? 1 : 0][albedo ? 1 : 0], driver->caps()));
        return shader;
    }

    prisma::ShaderHandle depthCoverageFragmentShader(prisma::Driver* driver, bool albedo)
    {
        prisma::ShaderHandle& shader = depthCoverageFragmentShaders[albedo ? 1 : 0];
        if (!shader.valid())
            shader = driver->createShader(prisma3d::shaderDesc(
                    *kDepthCoverageFragmentBlobs[albedo ? 1 : 0], driver->caps()));
        return shader;
    }

    prisma::PipelineHandle shadowPipelineFor(prisma::Driver* driver, const SurfaceInfo& surface,
            std::uint32_t variant)
    {
        const std::uint32_t key = surface.attributeMask |
                                  (static_cast<std::uint32_t>(surface.topology) << 8) | variant;
        if (const prisma::PipelineHandle* found = shadowPipelines.find(key)) return *found;
        const bool dual = (variant & kShadowDual) != 0;
        const bool scissor = (variant & kShadowScissor) != 0;
        const bool albedo = (variant & kShadowAlbedo) != 0;
        const bool terrain = (variant & kShadowTerrain) != 0;
        const std::uint32_t multimeshMode = (variant >> kShadowMultiMeshShift) & 7u;
        prisma::PipelineDesc desc;
        desc.vertexShader = dual ? shadowDualVertexShader : shadowVertexShader;
        if (multimeshMode != 0)
        {
            prisma::ShaderHandle& shader =
                    dual ? shadowDualMultiMeshVertexShaders[multimeshMode - 1]
                         : shadowMultiMeshVertexShaders[multimeshMode - 1];
            if (!shader.valid())
                shader = driver->createShader(prisma3d::shaderDesc(
                        dual ? *kShadowDualMultiMeshVertexBlobs[multimeshMode - 1]
                             : *kShadowMultiMeshVertexBlobs[multimeshMode - 1],
                        driver->caps()));
            desc.vertexShader = shader;
        }
        desc.fragmentShader = dual ? shadowDualFragmentShader : shadowFragmentShader;
        if (scissor)
        {
            desc.vertexShader = depthVertexShader(driver, dual, multimeshMode);
            desc.fragmentShader = (variant & kDepthCoverage) != 0
                                          ? depthCoverageFragmentShader(driver, albedo)
                                          : depthFragmentShader(driver, dual, albedo);
            if ((variant & kShadowGrass) != 0)
            {
                prisma::ShaderHandle& shader = depthGrassVertexShaders[dual ? 1 : 0];
                if (!shader.valid())
                    shader = driver->createShader(prisma3d::shaderDesc(
                            *kDepthGrassVertexBlobs[dual ? 1 : 0], driver->caps()));
                desc.vertexShader = shader;
            }
            if ((variant & kDepthLeafWind) != 0)
            {
                prisma::ShaderHandle& shader = depthLeafWindVertexShaders[multimeshMode];
                if (!shader.valid())
                    shader = driver->createShader(prisma3d::shaderDesc(
                            *kDepthLeafWindVertexBlobs[multimeshMode], driver->caps()));
                desc.vertexShader = shader;
            }
        }
        else if (terrain)
        {
            prisma::ShaderHandle& shader = shadowTerrainVertexShaders[dual ? 1 : 0];
            if (!shader.valid())
                shader = driver->createShader(prisma3d::shaderDesc(
                        *kShadowTerrainVertexBlobs[dual ? 1 : 0], driver->caps()));
            desc.vertexShader = shader;
        }
        desc.vertexBuffers[0].stride = surface.vertexStride;
        desc.vertexBufferCount = 1;
        desc.attributes[0].location = 0;
        desc.attributes[0].format = prisma::VertexFormat::Float3;
        desc.attributes[0].offset =
                MeshStore::attributeOffset(surface.attributeMask, kVertexPosition);
        desc.attributeCount = 1;
        if (scissor)
        {
            desc.attributes[1].location = 3;
            desc.attributes[1].format = prisma::VertexFormat::Float4;
            desc.attributes[1].offset =
                    MeshStore::attributeOffset(surface.attributeMask, kVertexColor);
            if (surface.attributeMask & kVertexUv)
            {
                desc.attributes[2].location = 4;
                desc.attributes[2].format = prisma::VertexFormat::Float2;
                desc.attributes[2].offset =
                        MeshStore::attributeOffset(surface.attributeMask, kVertexUv);
                desc.attributeCount = 3;
            }
            else
                desc.attributeCount = 2;
        }
        else if (terrain)
        {
            desc.attributes[1].location = 3;
            desc.attributes[1].format = prisma::VertexFormat::Float4;
            desc.attributes[1].offset =
                    MeshStore::attributeOffset(surface.attributeMask, kVertexColor);
            desc.attributeCount = 2;
        }
        desc.topology = surface.topology;
        desc.uniformBlockCount = 2;
        desc.uniformBlocks[0].name = "SceneDataBlock";
        desc.uniformBlocks[0].slot = 0;
        desc.uniformBlocks[1].name = "DrawCall";
        desc.uniformBlocks[1].slot = 1;
        if (scissor)
        {
            desc.uniformBlockCount = 3;
            desc.uniformBlocks[2].name = "MaterialUniforms";
            desc.uniformBlocks[2].slot = 2;
            if (albedo)
            {
                desc.textures[0].name = "texture_albedo";
                desc.textures[0].slot = 0;
                desc.textureCount = 1;
            }
        }
        desc.storageBufferCount = 1;
        desc.storageBuffers[0].name = "InstanceDataBuffer";
        desc.storageBuffers[0].slot = 0;
        if (multimeshMode != 0)
        {
            desc.storageBufferCount = 2;
            desc.storageBuffers[1].name = "Transforms";
            desc.storageBuffers[1].slot = 5;
        }
        desc.targets.window = false;
        desc.targets.samples = 1u << ((variant >> kDepthSamplesShift) & 3u);
        desc.targets.colorCount = 0;
        desc.targets.depth = prisma::TextureFormat::Depth32F;
        desc.depthTest = true;
        desc.depthWrite = true;
        desc.depthCompare = (variant & kShadowReverseDepth) ? prisma::CompareOp::GreaterEqual
                                                            : prisma::CompareOp::LessEqual;
        desc.cullMode = (variant & kDepthNoCull)       ? prisma::CullMode::None
                        : (variant & kShadowFrontCull) ? prisma::CullMode::Front
                                                       : prisma::CullMode::Back;
        desc.frontFace = prisma::FrontFace::CounterClockwise;
        desc.debugName = (variant & kDepthPrePass) ? "depth pipeline" : "shadow pipeline";
        const std::uint64_t start = time_nanos();
        const prisma::PipelineHandle pipeline = driver->createPipeline(desc);
        noteBuild((variant & kDepthPrePass) ? "depth pipeline variant" : "shadow pipeline variant",
                variant, start);
        if (!pipeline.valid()) log_error("render: could not create a depth pipeline");
        shadowPipelines[key] = pipeline;
        return pipeline;
    }

    struct ShadowStats
    {
        std::uint32_t cascades = 0;
        std::uint32_t lights = 0;
        std::uint32_t passes = 0;
        std::uint32_t casters = 0;
        std::uint32_t drawCalls = 0;
        std::uint32_t triangles = 0;
    };

    struct ShadowRef
    {
        Light* light = nullptr;
        Math::Frustum frustum;
        Math::Mat4 projection;
        Math::Mat4 transform;
        float zFar = 0.0f;
        float dualSide = 0.0f;
        bool pancake = false;
        ShadowPass pass;
        ShadowCopyParams copyParams;
    };

    static Math::Mat4 reverseDepth(bool remap, bool flipY = false)
    {
        Math::Mat4 matrix = Math::Mat4::Identity();
        matrix.col1.y = flipY ? -1.0f : 1.0f;
        matrix.col2.z = remap ? -0.5f : -1.0f;
        matrix.col3.z = remap ? 0.5f : 0.0f;
        return matrix;
    }

    static Math::Mat4 lookingAt(const Math::Vec3& target, const Math::Vec3& up)
    {
        const Math::Vec3 z = (target * -1.0f).Normalized();
        const Math::Vec3 x = Math::Vec3::Cross(up, z).Normalized();
        const Math::Vec3 y = Math::Vec3::Cross(z, x);
        return Math::Mat4(Math::Mat3(x, y, z), Math::Vec3(0.0f, 0.0f, 0.0f));
    }

    static Math::Mat4 orthonormalized(const Math::Mat4& transform)
    {
        Math::Vec3 x = transform.col0.xyz();
        Math::Vec3 y = transform.col1.xyz();
        Math::Vec3 z = transform.col2.xyz();
        x = x.Normalized();
        y = (y - x * Math::Vec3::Dot(x, y)).Normalized();
        z = (z - x * Math::Vec3::Dot(x, z) - y * Math::Vec3::Dot(y, z)).Normalized();
        return Math::Mat4(Math::Mat3(x, y, z), transform.GetTranslation());
    }

    static Math::Box positionalLightBox(const Light& light, const Math::Mat4& transform)
    {
        const float radius = light.params[static_cast<std::size_t>(LightParam::Range)];
        if (light.type == LightType::Omni)
            return Math::Box(Math::Vec3(-radius, -radius, -radius),
                    Math::Vec3(radius, radius, radius))
                    .Transformed(transform);
        const float angle = light.params[static_cast<std::size_t>(LightParam::SpotAngle)];
        const float w = radius * sinf(angle * kDegreesToRadians);
        return Math::Box(Math::Vec3(-w, -w, -radius), Math::Vec3(w, w, 0.0f))
                .Transformed(transform);
    }

    static float positionalCoverage(const Light& light, const Camera& camera, float aspect)
    {
        const Math::Vec3 right = camera.transform.col0.xyz();
        const Math::Vec3 back = camera.transform.col2.xyz();
        const Math::Vec3 origin = camera.transform.GetTranslation();
        const float zn = camera.zNear;
        const Math::Vec3 planeNormal = back * -1.0f;
        const float planeD = Math::Vec3::Dot(planeNormal, origin + back * -zn);
        const bool orthogonal = camera.projection == CameraProjection::Orthogonal;

        float halfY = camera.size * 0.5f;
        if (!orthogonal) halfY = zn * tanf(camera.fovy * 0.5f * kDegreesToRadians);
        const float halfX = halfY * aspect;

        const float radius = light.params[static_cast<std::size_t>(LightParam::Range)];
        const Math::Vec3 lightOrigin = light.transform.GetTranslation();
        Math::Vec3 points[2];
        if (light.type == LightType::Omni)
        {
            points[0] = lightOrigin;
            points[1] = lightOrigin + right * radius;
        }
        else
        {
            const float angle = light.params[static_cast<std::size_t>(LightParam::SpotAngle)] *
                                kDegreesToRadians;
            const float w = radius * sinf(angle);
            const float d = radius * cosf(angle);
            const Math::Vec3 base = lightOrigin - light.transform.col2.xyz().Normalized() * d;
            points[0] = base;
            points[1] = base + right * w;
        }

        if (!orthogonal)
        {
            for (int j = 0; j < 2; ++j)
            {
                if (Math::Vec3::Dot(planeNormal, points[j]) - planeD < 0.0f) points[j].z = -zn;

                const Math::Vec3 segment = origin - points[j];
                const float den = Math::Vec3::Dot(planeNormal, segment);
                if (fabsf(den) < 1.0e-5f) continue;
                float dist = (Math::Vec3::Dot(planeNormal, origin) - planeD) / den;
                if (dist < -1.0e-5f || dist > 1.0f + 1.0e-5f) continue;
                dist = -dist;
                points[j] = origin + segment * dist;
            }
        }

        const float screenDiameter = (points[1] - points[0]).Length() * 2.0f;
        return screenDiameter / (halfX + halfY);
    }

    static Math::Frustum dualParaboloidFrustum(const Math::Mat4& transform, float radius,
            float side)
    {
        const Math::Vec3 origin = transform.GetTranslation();
        const float k = 0.70710678118654752f;
        const Math::Vec3 normals[6] = { Math::Vec3(0.0f, 0.0f, side), Math::Vec3(k, 0.0f, side * k),
            Math::Vec3(-k, 0.0f, side * k), Math::Vec3(0.0f, k, side * k),
            Math::Vec3(0.0f, -k, side * k), Math::Vec3(0.0f, 0.0f, -side) };
        Math::Frustum frustum;
        for (int i = 0; i < 6; ++i)
        {
            const Math::Vec3 world = transform.TransformVector(normals[i]);
            const float d = (i == 5 ? 0.0f : radius) + Math::Vec3::Dot(world, origin);
            frustum.planes[i] = Math::Plane(world * -1.0f, d);
        }
        return frustum;
    }

    void collectDirectional(prisma::Driver* driver, ScenarioId id, const Camera& camera,
            float aspect, ct::Vector<ShadowRef>& refs)
    {
        ct::Vector<Light*> casters;
        std::uint32_t directionalCount = 0;
        for (Light& light: lights)
        {
            light.shadowActive = false;
            light.positional.active = false;
            if (!light.visible || light.scenario != id || light.type != LightType::Directional)
                continue;
            if (directionalCount++ >= kMaxDirectionalLights) continue;
            if (light.shadow &&
                    light.params[static_cast<std::size_t>(LightParam::ShadowOpacity)] > 0.001f)
                casters.push_back(&light);
        }
        if (casters.empty() || !ensureShadowResources(driver) || !ensureShadowAtlas(driver)) return;

        const std::uint32_t lightCount = static_cast<std::uint32_t>(casters.size());
        std::uint32_t first[4];
        directionalShadowRect(shadowAtlasSize, lightCount, 0, first);
        const float atlas = static_cast<float>(shadowAtlasSize);

        for (std::uint32_t li = 0; li < lightCount; ++li)
        {
            Light& light = *casters[li];
            std::uint32_t rect[4];
            directionalShadowRect(shadowAtlasSize, lightCount, li, rect);
            std::uint32_t cascadeW = rect[2];
            std::uint32_t cascadeH = rect[3];
            std::uint32_t textureSize = first[2] > first[3] ? first[2] : first[3];
            if (light.shadowMode == DirectionalShadowMode::Parallel4Splits)
            {
                cascadeW /= 2;
                cascadeH /= 2;
                textureSize = (first[2] / 2 > first[3] / 2) ? first[2] / 2 : first[3] / 2;
            }
            else if (light.shadowMode == DirectionalShadowMode::Parallel2Splits)
            {
                cascadeH /= 2;
                textureSize = first[2] > first[3] / 2 ? first[2] : first[3] / 2;
            }
            computeCascades(light, camera, aspect, static_cast<float>(textureSize));
            light.shadowActive = true;
            directionalShadowsActive = true;

            const float angle = light.params[static_cast<std::size_t>(LightParam::Size)];
            if (angle > 0.0f &&
                    light.params[static_cast<std::size_t>(LightParam::ShadowBlur)] > 0.0f)
                directionalSoftShadows = true;

            for (std::uint32_t c = 0; c < light.cascadeCount; ++c)
            {
                std::uint32_t x = rect[0];
                std::uint32_t y = rect[1];
                if (light.shadowMode == DirectionalShadowMode::Parallel4Splits)
                {
                    if (c == 1) x += cascadeW;
                    else if (c == 2)
                        y += cascadeH;
                    else if (c == 3)
                    {
                        x += cascadeW;
                        y += cascadeH;
                    }
                }
                else if (light.shadowMode == DirectionalShadowMode::Parallel2Splits && c == 1)
                    y += cascadeH;
                Light::Cascade& cascade = light.cascades[c];
                cascade.atlasRect[0] = static_cast<float>(x) / atlas;
                cascade.atlasRect[1] = static_cast<float>(y) / atlas;
                cascade.atlasRect[2] = static_cast<float>(cascadeW) / atlas;
                cascade.atlasRect[3] = static_cast<float>(cascadeH) / atlas;
                ShadowRef ref;
                ref.light = &light;
                ref.frustum = cascade.frustum;
                ref.projection = zeroToOneDepth() * cascade.projection;
                ref.transform = cascade.transform;
                ref.zFar = cascade.zFar;
                ref.pancake =
                        light.params[static_cast<std::size_t>(LightParam::ShadowPancakeSize)] >
                        0.0f;
                ref.pass.x = x;
                ref.pass.y = y;
                ref.pass.width = cascadeW;
                ref.pass.height = cascadeH;
                ref.pass.target = shadowAtlas;
                ref.pass.targetSize = shadowAtlasSize;
                ref.pass.clear = refs.empty();
                ref.pass.clearDepth = 1.0f;
                refs.push_back(ref);
            }
        }
    }

    void collectPositional(prisma::Driver* driver, Viewport& viewport, ScenarioId id,
            const Camera& camera, float aspect, const Math::Frustum& cameraFrustum,
            ct::Vector<ShadowRef>& refs)
    {
        ShadowAtlas& atlas = viewport.shadowAtlas;
        if (atlas.getSize() == 0 || atlas.getSmallestSubdivision() == 0) return;

        ct::Vector<Light*> candidates;
        for (Light& light: lights)
        {
            if (!light.visible || light.scenario != id || light.type == LightType::Directional)
                continue;
            if (!light.shadow ||
                    light.params[static_cast<std::size_t>(LightParam::ShadowOpacity)] <= 0.001f ||
                    light.params[static_cast<std::size_t>(LightParam::Range)] <= 0.0f)
                continue;
            if (!cameraFrustum.IntersectsBox(
                        positionalLightBox(light, orthonormalized(light.transform))))
                continue;
            candidates.push_back(&light);
        }
        if (candidates.empty() || !ensureShadowResources(driver) ||
                !ensurePositionalAtlas(driver, viewport))
            return;

        const std::uint64_t scenePass = ++scenePassCounter;
        const std::uint64_t tick = time_nanos() / 1000000ull;
        for (Light* light: candidates) atlas.markVisible(light->serial, scenePass);

        ct::Vector<Light*> shadowed;
        for (Light* light: candidates)
        {
            if (!atlas.updateLight(light->serial, light->type == LightType::Omni,
                        positionalCoverage(*light, camera, aspect), tick, scenePass))
                continue;
            shadowed.push_back(light);
        }
        if (shadowed.empty()) return;

        const std::uint32_t atlasSize = atlas.getSize();
        positionalShadowsActive = true;
        activePositionalAtlas = viewport.shadowAtlasTexture;

        ShadowRef clearRef;
        clearRef.pass.geometry = false;
        clearRef.pass.target = viewport.shadowAtlasTexture;
        clearRef.pass.targetSize = atlasSize;
        clearRef.pass.clear = true;
        clearRef.pass.clearDepth = 0.0f;
        refs.push_back(clearRef);

        for (Light* light: shadowed)
        {
            std::uint32_t pixels[4];
            Light::Positional& positional = light->positional;
            if (!atlas.getPixelRect(light->serial, pixels) ||
                    !atlas.getRect(light->serial, positional.atlasRect, positional.omniOffset))
                continue;
            if (pixels[2] < 4) continue;
            const float radius = light->params[static_cast<std::size_t>(LightParam::Range)];
            positional.active = true;
            positional.transform = orthonormalized(light->transform);
            positional.shadowSize = pixels[2];

            if (light->type == LightType::Spot)
            {
                const float angle = light->params[static_cast<std::size_t>(LightParam::SpotAngle)];
                const float zNear = fminf(0.025f, radius);
                positional.projection = Math::Mat4::Perspective(angle * 2.0f * kDegreesToRadians,
                        1.0f, zNear, radius);
                ShadowRef ref;
                ref.light = light;
                ref.frustum = Math::Frustum::FromViewProjection(
                        positional.projection * positional.transform.InverseAffine());
                ref.projection = reverseDepth(true) * positional.projection;
                ref.transform = positional.transform;
                ref.zFar = radius;
                ref.pass.x = pixels[0];
                ref.pass.y = pixels[1];
                ref.pass.width = pixels[2];
                ref.pass.height = pixels[3];
                ref.pass.target = viewport.shadowAtlasTexture;
                ref.pass.targetSize = atlasSize;
                ref.pass.variant = kShadowReverseDepth;
                refs.push_back(ref);
                continue;
            }

            const bool cube = light->omniShadowMode == OmniShadowMode::Cube && cubeShadowsReady &&
                              pixels[2] >= 4;
            if (cube)
            {
                static const Math::Vec3 viewNormals[6] = { Math::Vec3(1.0f, 0.0f, 0.0f),
                    Math::Vec3(-1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, -1.0f, 0.0f),
                    Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f),
                    Math::Vec3(0.0f, 0.0f, -1.0f) };
                static const Math::Vec3 viewUp[6] = { Math::Vec3(0.0f, -1.0f, 0.0f),
                    Math::Vec3(0.0f, -1.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -1.0f),
                    Math::Vec3(0.0f, 0.0f, 1.0f), Math::Vec3(0.0f, -1.0f, 0.0f),
                    Math::Vec3(0.0f, -1.0f, 0.0f) };
                const float zNear = fminf(0.025f, radius);
                positional.projection =
                        Math::Mat4::Perspective(90.0f * kDegreesToRadians, 1.0f, zNear, radius);
                const std::uint32_t cubeSize = pixels[2] / 2;
                const prisma::TextureHandle cubeTexture = shadowCube(driver, cubeSize);
                if (!cubeTexture.valid()) continue;
                for (std::uint32_t face = 0; face < 6; ++face)
                {
                    const Math::Mat4 faceTransform =
                            positional.transform * lookingAt(viewNormals[face], viewUp[face]);
                    ShadowRef ref;
                    ref.light = light;
                    ref.frustum = Math::Frustum::FromViewProjection(
                            positional.projection * faceTransform.InverseAffine());
                    ref.projection = reverseDepth(true, true) * positional.projection;
                    ref.transform = faceTransform;
                    ref.zFar = radius;
                    ref.pass.width = cubeSize;
                    ref.pass.height = cubeSize;
                    ref.pass.target = cubeTexture;
                    ref.pass.targetSize = cubeSize;
                    ref.pass.layer = face;
                    ref.pass.clear = true;
                    ref.pass.clearDepth = 0.0f;
                    ref.pass.variant = kShadowReverseDepth | kShadowFrontCull;
                    refs.push_back(ref);
                }
                for (std::uint32_t half = 0; half < 2; ++half)
                {
                    const int shift = static_cast<int>(half);
                    ShadowRef ref;
                    ref.light = light;
                    ref.pass.geometry = false;
                    ref.pass.copy = true;
                    ref.pass.source = cubeTexture;
                    ref.pass.target = viewport.shadowAtlasTexture;
                    ref.pass.targetSize = atlasSize;
                    ref.pass.x = static_cast<std::uint32_t>(
                            static_cast<int>(pixels[0]) +
                            shift * positional.omniOffset[0] * static_cast<int>(pixels[2]));
                    ref.pass.y = static_cast<std::uint32_t>(
                            static_cast<int>(pixels[1]) +
                            shift * positional.omniOffset[1] * static_cast<int>(pixels[3]));
                    ref.pass.width = pixels[2];
                    ref.pass.height = pixels[3];
                    ref.copyParams.zFar = radius;
                    ref.copyParams.zNear = zNear;
                    const float texel = 1.0f / static_cast<float>(pixels[2]);
                    ref.copyParams.texelSize[0] = half == 1 ? -texel : texel;
                    ref.copyParams.texelSize[1] = texel;
                    refs.push_back(ref);
                }
                continue;
            }

            positional.projection = Math::Mat4::Identity();
            for (std::uint32_t half = 0; half < 2; ++half)
            {
                ShadowRef ref;
                ref.light = light;
                ref.frustum = dualParaboloidFrustum(positional.transform, radius,
                        half == 0 ? -1.0f : 1.0f);
                ref.projection = reverseDepth(true);
                ref.transform = positional.transform;
                ref.zFar = radius;
                ref.dualSide = half == 0 ? 1.0f : -1.0f;
                const int shift = static_cast<int>(half);
                ref.pass.x = static_cast<std::uint32_t>(
                        static_cast<int>(pixels[0]) + 1 +
                        shift * positional.omniOffset[0] * static_cast<int>(pixels[2]));
                ref.pass.y = static_cast<std::uint32_t>(
                        static_cast<int>(pixels[1]) + 1 +
                        shift * positional.omniOffset[1] * static_cast<int>(pixels[3]));
                ref.pass.width = pixels[2] - 2;
                ref.pass.height = pixels[3] - 2;
                ref.pass.target = viewport.shadowAtlasTexture;
                ref.pass.targetSize = atlasSize;
                ref.pass.variant =
                        kShadowDual | kShadowReverseDepth | (half == 1 ? kShadowFrontCull : 0u);
                refs.push_back(ref);
            }
        }
    }

    ShadowStats prepareShadows(prisma::Driver* driver, MeshStore& meshes, MaterialStore& materials,
            TextureStore& textures, Viewport& viewport, ScenarioId id, Scenario& scenario,
            const Camera& camera, float aspect, const Math::Frustum& cameraFrustum,
            const LodView& lodView)
    {
        ShadowStats stats;
        shadowPasses.clear();
        shadowRuns.clear();
        shadowInstanceData.clear();
        directionalSoftShadows = false;
        directionalShadowsActive = false;
        positionalShadowsActive = false;
        activePositionalAtlas = prisma::TextureHandle();

        ct::Vector<ShadowRef> refs;
        collectDirectional(driver, id, camera, aspect, refs);
        collectPositional(driver, viewport, id, camera, aspect, cameraFrustum, refs);
        if (refs.empty()) return stats;

        std::uint32_t geometryCount = 0;
        for (const ShadowRef& ref: refs)
            if (ref.pass.geometry) ++geometryCount;

        const prisma::Caps& caps = driver->caps();
        const std::uint32_t sceneStride = alignUp(static_cast<std::uint32_t>(sizeof(SceneUboData)),
                caps.uniformBufferOffsetAlignment);
        const std::uint32_t drawStride = alignUp(kBlockSize, caps.uniformBufferOffsetAlignment);
        shadowSceneBytes.resize(static_cast<std::size_t>(geometryCount) * sceneStride);
        memset(shadowSceneBytes.data(), 0, shadowSceneBytes.size());
        shadowDrawBytes.clear();
        shadowCopyBytes.clear();
        const std::uint32_t copyStride =
                alignUp(static_cast<std::uint32_t>(sizeof(ShadowCopyParams)),
                        caps.uniformBufferOffsetAlignment);

        std::uint32_t sceneIndex = 0;
        for (std::size_t r = 0; r < refs.size(); ++r)
        {
            ShadowRef& ref = refs[r];
            ShadowPass pass = ref.pass;
            pass.firstRun = static_cast<std::uint32_t>(shadowRuns.size());
            pass.begin = shadowPasses.empty() || shadowPasses.back().target != pass.target ||
                         shadowPasses.back().layer != pass.layer || pass.clear;
            if (!pass.geometry)
            {
                if (pass.copy)
                {
                    pass.copyOffset = static_cast<std::uint32_t>(shadowCopyBytes.size());
                    shadowCopyBytes.resize(shadowCopyBytes.size() + copyStride);
                    memset(shadowCopyBytes.data() + pass.copyOffset, 0, copyStride);
                    memcpy(shadowCopyBytes.data() + pass.copyOffset, &ref.copyParams,
                            sizeof(ShadowCopyParams));
                }
                shadowPasses.push_back(pass);
                continue;
            }
            const bool positionalPass = ref.light->type != LightType::Directional;
            if (positionalPass) ++stats.passes;
            else
            {
                ++stats.cascades;
                ++stats.passes;
            }

            shadowItems.clear();
            auto addCaster = [&](const Instance& instance, const MeshInfo& mesh, std::uint32_t only,
                                     const Math::Box& bounds)
            {
                if (only != kWholeInstance && only >= mesh.surfaces.size()) return;
                const MultiMeshData* multimesh = nullptr;
                std::uint32_t multimeshMode = 0;
                std::uint32_t multimeshInstances = 1;
                std::uint32_t transformBytes = 0;
                MeshId meshId = instance.base;
                if (instance.multimesh.valid())
                {
                    multimesh = multimeshes.get(instance.multimesh);
                    if (!multimesh || !multimesh->buffer.valid()) return;
                    multimeshInstances = MultiMeshStorage::instancesToDraw(*multimesh);
                    if (multimeshInstances == 0) return;
                    multimeshMode = multimeshModeFor(*multimesh);
                    transformBytes = multimesh->instances * multimesh->strideCache *
                                     static_cast<std::uint32_t>(sizeof(float));
                    meshId = multimesh->mesh;
                }
                const MeshInfo* shadowMesh = nullptr;
                if (mesh.shadowMesh.valid() && !instance.meshInstance)
                    shadowMesh = meshes.get(mesh.shadowMesh);
                const std::size_t firstSurface = only == kWholeInstance ? 0 : only;
                const std::size_t lastSurface =
                        only == kWholeInstance ? mesh.surfaces.size() : only + 1;
                for (std::size_t sIndex = firstSurface; sIndex < lastSurface; ++sIndex)
                {
                    if (surfaceHidden(instance, sIndex)) continue;
                    const MaterialId materialId = resolveMaterial(instance, &mesh,
                            static_cast<std::uint32_t>(sIndex), materials);
                    const MaterialInfo* material = materials.get(materialId);
                    const std::uint32_t kind =
                            sceneVariant(material, mesh.surfaces[sIndex], textures);
                    const bool substituted = shadowMesh && sIndex < shadowMesh->surfaces.size();
                    const bool scissor =
                            (kind & 16u) != 0 &&
                            (!substituted ||
                                    (shadowMesh->surfaces[sIndex].attributeMask & kVertexUv) != 0);
                    const SurfaceInfo& surface =
                            substituted ? shadowMesh->surfaces[sIndex] : mesh.surfaces[sIndex];
                    if (!surface.ready || surface.vertexCount == 0) continue;
                    if (material && (material->transparency == MaterialTransparency::AlphaBlend ||
                                            material->blend != MaterialBlend::Mix ||
                                            material->billboard != MaterialBillboard::Disabled))
                        continue;
                    ShadowItem item;
                    item.surface = &surface;
                    item.indexBuffer = surface.indexBuffer;
                    item.indexCount = surface.indexCount;
                    const std::uint32_t lod = surfaceLod(surface, instance, bounds, lodView);
                    if (lod > 0)
                    {
                        item.indexBuffer = surface.lods[lod - 1].indexBuffer;
                        item.indexCount = surface.lods[lod - 1].indexCount;
                    }
                    if (scissor)
                    {
                        item.scissor = true;
                        item.albedo = (kind & 1u) != 0;
                        item.material = materialId.bits();
                        item.color = instance.color;
                        if (item.albedo)
                            item.albedoTexture =
                                    textures.gpu(sceneTexture(*material, MaterialTexture::Albedo));
                    }
                    else if (material && material->terrainMorph &&
                             (surface.attributeMask & kVertexColor) != 0)
                    {
                        item.terrain = true;
                        item.color = instance.color;
                    }
                    item.vertexBuffer = surface.vertexBuffer;
                    if (instance.meshInstance && sIndex < instance.meshInstance->surfaces.size() &&
                            instance.meshInstance->surfaces[sIndex].vertexBuffer.valid())
                    {
                        item.vertexBuffer = instance.meshInstance->surfaces[sIndex].vertexBuffer;
                        item.skinned = true;
                    }
                    item.transform = instance.transform;
                    if (multimesh)
                    {
                        item.multimeshMode = multimeshMode;
                        item.transforms = multimesh->buffer;
                        item.transformBytes = transformBytes;
                        item.instanceCount = multimeshInstances;
                    }
                    item.grass = item.scissor && material->grass && multimeshMode == 4;
                    item.doubleSided = material && material->cull == MaterialCull::Disabled;
                    const std::uint64_t pipeline =
                            surface.attributeMask |
                            (static_cast<std::uint64_t>(surface.topology) << 8) |
                            (static_cast<std::uint64_t>(multimeshMode) << 12) |
                            (static_cast<std::uint64_t>(item.scissor ? 1 : 0) << 15) |
                            (static_cast<std::uint64_t>(item.albedo ? 1 : 0) << 16) |
                            (static_cast<std::uint64_t>(item.terrain ? 1 : 0) << 17) |
                            (static_cast<std::uint64_t>(item.grass ? 1 : 0) << 18) |
                            (static_cast<std::uint64_t>(item.doubleSided ? 1 : 0) << 19);
                    item.key = (static_cast<std::uint64_t>(lod) << 59) | (pipeline << 40) |
                               (static_cast<std::uint64_t>(
                                        (substituted ? mesh.shadowMesh : meshId).bits() & 0xffffffu)
                                       << 16) |
                               (sIndex & 0xffffu);
                    shadowItems.push_back(item);
                }
            };

            if (cullingTree && !scenario.entries.empty())
            {
                scenario.tree->queryCandidates(ref.frustum, scenario.candidates);
                ct::sort(scenario.candidates.begin(), scenario.candidates.end());
                for (std::size_t i = 0; i < scenario.candidates.size(); ++i)
                {
                    const std::uint32_t entry = scenario.candidates[i];
                    const Instance* instance = find(instances, scenario.entries[entry]);
                    if (!instance || !instance->visible || !instance->castShadows) continue;
                    if (!ref.frustum.IntersectsBox(scenario.bounds[entry])) continue;
                    const MeshInfo* mesh = instanceMesh(meshes, *instance);
                    if (mesh)
                        addCaster(*instance, *mesh, scenario.entrySurfaces[entry],
                                scenario.bounds[entry]);
                }
            }
            else
            {
                for (std::size_t i = 0; i < scenario.instances.size(); ++i)
                {
                    const Instance* instance = find(instances, scenario.instances[i]);
                    if (!instance || !instance->visible || !instance->castShadows) continue;
                    if (!instanceUsable(meshes, *instance)) continue;
                    const MeshInfo* mesh = instanceMesh(meshes, *instance);
                    const Math::Box instanceBox = instanceBounds(*instance, *mesh);
                    if (!ref.frustum.IntersectsBox(instanceBox)) continue;
                    if (!splitsSurfaces(*instance, *mesh))
                    {
                        addCaster(*instance, *mesh, kWholeInstance, instanceBox);
                        continue;
                    }
                    for (std::uint32_t s = 0; s < mesh->surfaces.size(); ++s)
                    {
                        const Math::Box surfaceBox = entryBounds(*instance, *mesh, s);
                        if (ref.frustum.IntersectsBox(surfaceBox))
                            addCaster(*instance, *mesh, s, surfaceBox);
                    }
                }
            }

            shadowSorted.resize(shadowItems.size());
            for (std::size_t i = 0; i < shadowItems.size(); ++i) shadowSorted[i] = shadowItems[i];
            ct::stable_sort(shadowSorted.begin(), shadowSorted.end(),
                    [](const ShadowItem& a, const ShadowItem& b)
                    {
                        if (a.key != b.key) return a.key < b.key;
                        return a.material < b.material;
                    });

            pass.sceneOffset = sceneIndex * sceneStride;
            const std::uint32_t base = static_cast<std::uint32_t>(shadowInstanceData.size());
            for (std::size_t i = 0; i < shadowSorted.size(); ++i)
            {
                const ShadowItem& item = shadowSorted[i];
                if (item.surface->topology == prisma::Topology::Triangles)
                    stats.triangles +=
                            (item.indexCount > 0 ? item.indexCount : item.surface->vertexCount) /
                            3 * item.instanceCount;
                InstanceDataGpu data;
                memset(&data, 0, sizeof(data));
                storeTransform3x4(item.transform, data.transform);
                memcpy(data.prevTransform, data.transform, sizeof(data.transform));
                data.compressedAabbSize[0] = data.compressedAabbSize[1] =
                        data.compressedAabbSize[2] = 1.0f;
                data.materialFeedbackIndex = 0xffffffffu;
                data.flags = 255u << 24;
                data.layerMask = 0xffffffffu;
                shadowInstanceData.push_back(data);

                if (!shadowRuns.empty() && shadowRuns.size() > pass.firstRun)
                {
                    ShadowRun& last = shadowRuns.back();
                    const ShadowItem& head = shadowSorted[last.first - base];
                    if (!item.skinned && !head.skinned && !item.terrain &&
                            item.multimeshMode == 0 && head.multimeshMode == 0 &&
                            last.count < kMaxRepeats && item.key == head.key &&
                            (!item.scissor || (item.material == head.material &&
                                                      sameColor(item.color, head.color))))
                    {
                        ++last.count;
                        continue;
                    }
                }
                ShadowRun run;
                run.first = base + static_cast<std::uint32_t>(i);
                run.count = 1;
                run.surface = item.surface;
                run.vertexBuffer = item.vertexBuffer;
                run.indexBuffer = item.indexBuffer;
                run.indexCount = item.indexCount;
                run.variant = pass.variant | (item.multimeshMode << kShadowMultiMeshShift);
                if (item.doubleSided) run.variant |= kDepthNoCull;
                if (item.scissor)
                {
                    run.variant |= kShadowScissor;
                    if (item.albedo) run.variant |= kShadowAlbedo;
                    if (item.grass) run.variant |= kShadowGrass;
                    run.material = item.material;
                    run.albedoTexture = item.albedoTexture;
                    run.color[0] = srgbToLinear(item.color.x);
                    run.color[1] = srgbToLinear(item.color.y);
                    run.color[2] = srgbToLinear(item.color.z);
                    run.color[3] = item.color.w;
                }
                else if (item.terrain)
                {
                    run.variant |= kShadowTerrain;
                    run.color[3] = item.color.w;
                }
                run.transforms = item.transforms;
                run.transformBytes = item.transformBytes;
                if (item.multimeshMode != 0) run.count = item.instanceCount;
                shadowRuns.push_back(run);
            }
            pass.runCount = static_cast<std::uint32_t>(shadowRuns.size()) - pass.firstRun;
            shadowPasses.push_back(pass);
            stats.casters += static_cast<std::uint32_t>(shadowSorted.size());
            stats.drawCalls += pass.runCount;

            SceneUboData data;
            memset(&data, 0, sizeof(data));
            SceneUbo& ubo = data.data;
            const Math::Mat4 lightView = ref.transform.InverseAffine();
            storeMatrix(ref.projection, ubo.projectionMatrix);
            storeMatrix(ref.projection.Inverse(), ubo.invProjectionMatrix);
            storeTransform3x4(ref.transform, ubo.invViewMatrix);
            storeTransform3x4(lightView, ubo.viewMatrix);
            storeMatrix(ref.projection, ubo.projectionMatrixView[0]);
            storeMatrix(ref.projection.Inverse(), ubo.invProjectionMatrixView[0]);
            ubo.viewportSize[0] = static_cast<float>(pass.width);
            ubo.viewportSize[1] = static_cast<float>(pass.height);
            ubo.zFar = ref.zFar;
            ubo.zNear = 0.0f;
            ubo.dualParaboloidSide = ref.dualSide;
            ubo.cameraVisibleLayers = 0xffffffffu;
            ubo.passAlphaMultiplier = 1.0f;
            if (ref.pancake) ubo.flags |= kSceneFlagUsePancakeShadows;
            ubo.flags |= kSceneFlagInShadowPass;
            data.prev = data.data;
            memcpy(shadowSceneBytes.data() + static_cast<std::size_t>(sceneIndex) * sceneStride,
                    &data, sizeof(data));
            ++sceneIndex;
        }

        std::uint32_t lightCountStat = 0;
        for (const Light& light: lights)
            if (light.positional.active) ++lightCountStat;
        stats.lights = lightCountStat;

        shadowDrawBytes.resize(shadowRuns.size() * drawStride);
        memset(shadowDrawBytes.data(), 0, shadowDrawBytes.size());
        shadowMaterialSlots.clear();
        std::uint32_t shadowMaterialCount = 0;
        for (std::size_t r = 0; r < shadowRuns.size(); ++r)
        {
            ShadowRun& run = shadowRuns[r];
            DrawCallData draw = {};
            draw.instanceIndex = run.first;
            if (run.variant & kShadowScissor)
            {
                memcpy(draw.color, run.color, sizeof(draw.color));
                const std::uint32_t* slot = shadowMaterialSlots.find(run.material);
                if (slot) run.materialSlot = *slot;
                else
                {
                    shadowMaterialSlots[run.material] = shadowMaterialCount;
                    run.materialSlot = shadowMaterialCount++;
                }
            }
            else if (run.variant & kShadowTerrain)
                memcpy(draw.color, run.color, sizeof(draw.color));
            memcpy(shadowDrawBytes.data() + r * drawStride, &draw, sizeof(draw));
            shadowPipelineFor(driver, *run.surface, run.variant);
        }
        const std::uint32_t materialStride =
                alignUp(static_cast<std::uint32_t>(sizeof(MaterialDataGpu)),
                        caps.uniformBufferOffsetAlignment);
        const std::uint32_t materialBytes = shadowMaterialCount * materialStride;
        shadowMaterialBytes.resize(materialBytes);
        for (auto& entry: shadowMaterialSlots)
        {
            MaterialDataGpu data;
            fillMaterial(materials.get(MaterialId::from_bits(entry.key)), data);
            memcpy(shadowMaterialBytes.data() + entry.value * materialStride, &data, sizeof(data));
        }

        const std::uint32_t copyBytes = static_cast<std::uint32_t>(shadowCopyBytes.size());
        const std::uint32_t sceneBytes = static_cast<std::uint32_t>(shadowSceneBytes.size());
        const std::uint32_t drawBytes = static_cast<std::uint32_t>(shadowDrawBytes.size());
        const std::uint32_t instanceBytes =
                static_cast<std::uint32_t>(shadowInstanceData.size() * sizeof(InstanceDataGpu));
        if ((copyBytes > 0 && !ensureStorage(driver, shadowCopyBuffer, shadowCopyBufferSize,
                                      copyBytes, "shadow copy", prisma::BufferUsage::Uniform)) ||
                (sceneBytes > 0 &&
                        !ensureStorage(driver, shadowSceneBuffer, shadowSceneBufferSize, sceneBytes,
                                "shadow scene", prisma::BufferUsage::Uniform)) ||
                (drawBytes > 0 &&
                        !ensureStorage(driver, shadowDrawBuffer, shadowDrawBufferSize, drawBytes,
                                "shadow draw", prisma::BufferUsage::Uniform)) ||
                (instanceBytes > 0 &&
                        !ensureStorage(driver, shadowInstanceBuffer, shadowInstanceBufferSize,
                                instanceBytes, "shadow instances")) ||
                (materialBytes > 0 &&
                        !ensureStorage(driver, shadowMaterialBuffer, shadowMaterialBufferSize,
                                materialBytes, "shadow materials", prisma::BufferUsage::Uniform)))
        {
            shadowPasses.clear();
            directionalShadowsActive = false;
            positionalShadowsActive = false;
            activePositionalAtlas = prisma::TextureHandle();
            stats = ShadowStats();
            return stats;
        }
        if (copyBytes > 0)
            driver->updateBuffer(shadowCopyBuffer, 0, shadowCopyBytes.data(), copyBytes);
        if (sceneBytes > 0)
            driver->updateBuffer(shadowSceneBuffer, 0, shadowSceneBytes.data(), sceneBytes);
        if (drawBytes > 0)
            driver->updateBuffer(shadowDrawBuffer, 0, shadowDrawBytes.data(), drawBytes);
        if (instanceBytes > 0)
            driver->updateBuffer(shadowInstanceBuffer, 0, shadowInstanceData.data(), instanceBytes);
        if (materialBytes > 0)
            driver->updateBuffer(shadowMaterialBuffer, 0, shadowMaterialBytes.data(),
                    materialBytes);
        return stats;
    }

    void renderShadows(prisma::Driver* driver)
    {
        if (shadowPasses.empty()) return;
        const prisma::Caps& caps = driver->caps();
        const std::uint32_t drawStride = alignUp(kBlockSize, caps.uniformBufferOffsetAlignment);
        const std::uint32_t instanceBytes =
                static_cast<std::uint32_t>(shadowInstanceData.size() * sizeof(InstanceDataGpu));
        const std::uint32_t materialStride =
                alignUp(static_cast<std::uint32_t>(sizeof(MaterialDataGpu)),
                        caps.uniformBufferOffsetAlignment);
        prisma::PipelineHandle bound;
        bool open = false;
        for (const ShadowPass& pass: shadowPasses)
        {
            if (pass.begin)
            {
                if (open) driver->endRenderPass();
                prisma::RenderPassDesc desc;
                desc.colorCount = 0;
                desc.depth.texture = pass.target;
                desc.depth.layer = pass.layer;
                desc.depthLoad = pass.clear ? prisma::LoadOp::Clear : prisma::LoadOp::Load;
                desc.depthStore = prisma::StoreOp::Store;
                desc.clearDepth = pass.clearDepth;
                driver->beginRenderPass(desc);
                open = true;
                bound = prisma::PipelineHandle();
            }
            if (!pass.geometry && !pass.copy) continue;
            const std::uint32_t top = pass.targetSize - (pass.y + pass.height);
            prisma::Viewport area;
            area.x = static_cast<float>(pass.x);
            area.y = static_cast<float>(top);
            area.width = static_cast<float>(pass.width);
            area.height = static_cast<float>(pass.height);
            driver->setViewport(area);
            prisma::Rect scissor;
            scissor.x = static_cast<std::int32_t>(pass.x);
            scissor.y = static_cast<std::int32_t>(top);
            scissor.width = pass.width;
            scissor.height = pass.height;
            driver->setScissor(scissor);
            if (pass.copy)
            {
                driver->bindPipeline(cubeCopyPipeline);
                bound = cubeCopyPipeline;
                driver->bindUniformBuffer(0, shadowCopyBuffer, pass.copyOffset,
                        static_cast<std::uint32_t>(sizeof(ShadowCopyParams)));
                driver->bindTexture(0, pass.source, shadowDepthSampler);
                driver->draw(3, 0);
                continue;
            }
            driver->bindUniformBuffer(0, shadowSceneBuffer, pass.sceneOffset,
                    static_cast<std::uint32_t>(sizeof(SceneUboData)));
            if (instanceBytes > 0)
                driver->bindStorageBuffer(0, shadowInstanceBuffer, 0, instanceBytes);
            for (std::uint32_t r = 0; r < pass.runCount; ++r)
            {
                const std::uint32_t runIndex = pass.firstRun + r;
                const ShadowRun& run = shadowRuns[runIndex];
                const prisma::PipelineHandle pipeline =
                        shadowPipelineFor(driver, *run.surface, run.variant);
                if (!pipeline.valid()) continue;
                if (!bound.valid() || bound != pipeline)
                {
                    driver->bindPipeline(pipeline);
                    bound = pipeline;
                }
                driver->bindVertexBuffer(0, run.vertexBuffer, 0);
                if (run.variant & kShadowScissor)
                {
                    driver->bindUniformBuffer(2, shadowMaterialBuffer,
                            run.materialSlot * materialStride,
                            static_cast<std::uint32_t>(sizeof(MaterialDataGpu)));
                    if (run.variant & kShadowAlbedo)
                        driver->bindTexture(0, run.albedoTexture, materialSampler);
                }
                if (run.transforms.valid())
                    driver->bindStorageBuffer(5, run.transforms, 0, run.transformBytes);
                driver->bindUniformBuffer(1, shadowDrawBuffer, runIndex * drawStride, kBlockSize);
                if (run.indexCount > 0)
                {
                    driver->bindIndexBuffer(run.indexBuffer);
                    driver->drawIndexed(run.indexCount, 0, run.count);
                }
                else
                    driver->draw(run.surface->vertexCount, 0, run.count);
            }
        }
        if (open) driver->endRenderPass();
    }

    static constexpr std::uint32_t kDfgSize = 128;

    void generateDfg(prisma::Driver* driver)
    {
        if (dfgGenerated) return;
        dfgGenerated = true;
        const prisma::Caps& caps = driver->caps();
        prisma::TextureDesc textureDesc;
        textureDesc.format = prisma::TextureFormat::RG16F;
        textureDesc.width = kDfgSize;
        textureDesc.height = kDfgSize;
        textureDesc.usage = prisma::kTextureSampled | prisma::kTextureStorage;
        textureDesc.debugName = "dfg";
        dfgTexture = driver->createTexture(textureDesc);
        if (!dfgTexture.valid() || !caps.compute)
        {
            log_error("render: the DFG table is not available on this driver");
            return;
        }
        dfgShader = driver->createShader(prisma3d::shaderDesc(integrate_dfg_comp, caps));
        prisma::ComputePipelineDesc desc;
        desc.shader = dfgShader;
        desc.storageTextureCount = 1;
        desc.storageTextures[0].name = "current_image";
        desc.storageTextures[0].slot = 0;
        desc.debugName = "dfg";
        if (dfgShader.valid()) dfgPipeline = driver->createComputePipeline(desc);
        if (!dfgPipeline.valid())
        {
            log_error("render: could not create the DFG pipeline");
            return;
        }
        driver->beginComputePass();
        driver->bindPipeline(dfgPipeline);
        driver->bindStorageTexture(0, dfgTexture, 0, prisma::StorageAccess::Write);
        driver->dispatch(kDfgSize / 8, kDfgSize / 8, 1);
        driver->endComputePass();
    }

    static bool ensureStorage(prisma::Driver* driver, prisma::BufferHandle& buffer,
            std::uint32_t& capacity, std::uint32_t size, const char* name,
            prisma::BufferUsage usage = prisma::BufferUsage::Storage)
    {
        if (buffer.valid() && capacity >= size) return true;
        std::uint32_t grown = capacity * 2;
        if (grown < size) grown = size;
        if (buffer.valid()) driver->destroy(buffer);
        buffer = prisma::BufferHandle();
        capacity = 0;
        prisma::BufferDesc desc;
        desc.usage = usage;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = grown;
        desc.debugName = name;
        buffer = driver->createBuffer(desc);
        if (!buffer.valid())
        {
            log_error("render: could not create a light buffer");
            return false;
        }
        capacity = grown;
        return true;
    }

    static void packLight(const Light& light, const Math::Mat4& view, LightDataGpu& out)
    {
        memset(&out, 0, sizeof(out));
        const float* params = light.params;
        const float sign = light.negative ? -1.0f : 1.0f;
        const float energy =
                sign * params[static_cast<std::size_t>(LightParam::Energy)] * 3.14159265358979f;
        out.color[0] = srgbToLinear(light.color.x) * energy;
        out.color[1] = srgbToLinear(light.color.y) * energy;
        out.color[2] = srgbToLinear(light.color.z) * energy;
        out.attenuation = params[static_cast<std::size_t>(LightParam::Attenuation)];
        out.specularAmount = params[static_cast<std::size_t>(LightParam::Specular)] * 2.0f;
        out.volumetricFogEnergy = 1.0f;
        out.bakeMode = static_cast<std::uint32_t>(light.bakeMode);
        const float radius = fmaxf(0.001f, params[static_cast<std::size_t>(LightParam::Range)]);
        out.invRadius = 1.0f / radius;
        const Math::Vec3 position = view.TransformPoint(light.transform.GetTranslation());
        out.position[0] = position.x;
        out.position[1] = position.y;
        out.position[2] = position.z;
        const Math::Vec3 direction =
                view.TransformVector(light.transform.TransformVector(Math::Vec3(0.0f, 0.0f, -1.0f)))
                        .Normalized();
        out.direction[0] = direction.x;
        out.direction[1] = direction.y;
        out.direction[2] = direction.z;
        out.size = params[static_cast<std::size_t>(LightParam::Size)];
        out.coneAttenuation = 1.0f / params[static_cast<std::size_t>(LightParam::SpotAttenuation)];
        out.coneAngle = cosf(params[static_cast<std::size_t>(LightParam::SpotAngle)] *
                             (3.14159265358979f / 180.0f));
        out.mask = light.cullMask;
    }

    void packPositionalShadow(const Light& light, const Math::Mat4& cameraWorld,
            LightDataGpu& out) const
    {
        const Light::Positional& positional = light.positional;
        const float* params = light.params;
        out.shadowOpacity = params[static_cast<std::size_t>(LightParam::ShadowOpacity)];
        const float texelSize = 1.0f / static_cast<float>(positional.shadowSize);
        out.shadowNormalBias =
                params[static_cast<std::size_t>(LightParam::ShadowNormalBias)] * texelSize * 10.0f;
        const bool spot = light.type == LightType::Spot;
        out.shadowBias =
                params[static_cast<std::size_t>(LightParam::ShadowBias)] * (spot ? 0.01f : 1.0f);
        out.transmittanceBias = 0.0f;
        for (int i = 0; i < 4; ++i) out.atlasRect[i] = positional.atlasRect[i];
        out.softShadowScale = params[static_cast<std::size_t>(LightParam::ShadowBlur)];

        const float size = params[static_cast<std::size_t>(LightParam::Size)];
        const float radius = fmaxf(0.001f, params[static_cast<std::size_t>(LightParam::Range)]);
        const Math::Mat4 modelview = positional.transform.InverseAffine() * cameraWorld;
        if (!spot)
        {
            storeMatrix(modelview, out.shadowMatrix);
            if (size > 0.0f && out.softShadowScale > 0.0f) out.softShadowSize = size;
            else
            {
                out.softShadowSize = 0.0f;
                out.softShadowScale *= positionalQualityRadius;
            }
            out.direction[0] =
                    static_cast<float>(positional.omniOffset[0]) * positional.atlasRect[2];
            out.direction[1] =
                    static_cast<float>(positional.omniOffset[1]) * positional.atlasRect[3];
            return;
        }

        Math::Mat4 bias = Math::Mat4::Identity();
        bias.col0.x = 0.5f;
        bias.col1.y = 0.5f;
        bias.col2.z = 0.5f;
        bias.col3.x = 0.5f;
        bias.col3.y = 0.5f;
        bias.col3.z = 0.5f;
        const Math::Mat4 matrix = bias * (reverseDepth(false) * positional.projection) * modelview;
        storeMatrix(matrix, out.shadowMatrix);
        if (size > 0.0f && out.softShadowScale > 0.0f)
        {
            const float tangent = tanf(
                    params[static_cast<std::size_t>(LightParam::SpotAngle)] * kDegreesToRadians);
            out.softShadowSize = (size * 0.5f / radius) / tangent * positional.atlasRect[2];
        }
        else
        {
            out.softShadowSize = 0.0f;
            out.softShadowScale *= positionalQualityRadius;
        }
        out.shadowBias *= out.softShadowScale;
    }

    static void vogelDisk(float* kernel, int count)
    {
        for (int i = 0; i < 128; ++i) kernel[i] = 0.0f;
        for (int i = 0; i < count && i < 32; ++i)
        {
            const float r = sqrtf(static_cast<float>(i) + 0.5f) / sqrtf(static_cast<float>(count));
            const float theta = static_cast<float>(i) * 2.4f;
            kernel[i * 4] = cosf(theta) * r;
            kernel[i * 4 + 1] = sinf(theta) * r;
        }
    }

    void applyShadowQuality(ShadowQuality quality)
    {
        shadowQuality = quality;
        switch (quality)
        {
            case ShadowQuality::Hard:
                directionalPenumbraSamples = 4;
                directionalSoftSamples = 0;
                directionalQualityRadius = 1.0f;
                break;
            case ShadowQuality::SoftVeryLow:
                directionalPenumbraSamples = 4;
                directionalSoftSamples = 1;
                directionalQualityRadius = 1.5f;
                break;
            case ShadowQuality::SoftLow:
                directionalPenumbraSamples = 8;
                directionalSoftSamples = 4;
                directionalQualityRadius = 2.0f;
                break;
            case ShadowQuality::SoftMedium:
                directionalPenumbraSamples = 12;
                directionalSoftSamples = 8;
                directionalQualityRadius = 2.0f;
                break;
            case ShadowQuality::SoftHigh:
                directionalPenumbraSamples = 24;
                directionalSoftSamples = 16;
                directionalQualityRadius = 3.0f;
                break;
            case ShadowQuality::SoftUltra:
                directionalPenumbraSamples = 32;
                directionalSoftSamples = 32;
                directionalQualityRadius = 4.0f;
                break;
        }
        vogelDisk(directionalPenumbraKernel, static_cast<int>(directionalPenumbraSamples));
        vogelDisk(directionalSoftKernel, static_cast<int>(directionalSoftSamples));
    }

    void applyPositionalShadowQuality(ShadowQuality quality)
    {
        positionalQuality = quality;
        switch (quality)
        {
            case ShadowQuality::Hard:
                positionalPenumbraSamples = 4;
                positionalSoftSamples = 0;
                positionalQualityRadius = 1.0f;
                break;
            case ShadowQuality::SoftVeryLow:
                positionalPenumbraSamples = 4;
                positionalSoftSamples = 1;
                positionalQualityRadius = 1.5f;
                break;
            case ShadowQuality::SoftLow:
                positionalPenumbraSamples = 8;
                positionalSoftSamples = 4;
                positionalQualityRadius = 2.0f;
                break;
            case ShadowQuality::SoftMedium:
                positionalPenumbraSamples = 12;
                positionalSoftSamples = 8;
                positionalQualityRadius = 2.0f;
                break;
            case ShadowQuality::SoftHigh:
                positionalPenumbraSamples = 24;
                positionalSoftSamples = 16;
                positionalQualityRadius = 3.0f;
                break;
            case ShadowQuality::SoftUltra:
                positionalPenumbraSamples = 32;
                positionalSoftSamples = 32;
                positionalQualityRadius = 4.0f;
                break;
        }
        vogelDisk(positionalPenumbraKernel, static_cast<int>(positionalPenumbraSamples));
        vogelDisk(positionalSoftKernel, static_cast<int>(positionalSoftSamples));
    }

    std::uint32_t packedConstants() const
    {
        std::uint32_t value = positionalSoftShadows ? (1u << 2) : 0u;
        value |= directionalSoftShadows ? (1u << 3) : 0u;
        value |= 1u << 4;
        value |= lightmapBicubic ? (1u << 7) : 0u;
        value |= positionalSoftSamples << 8;
        value |= positionalPenumbraSamples << 14;
        value |= directionalSoftSamples << 20;
        value |= directionalPenumbraSamples << 26;
        return value;
    }

    void packDirectional(const Light& light, const Math::Mat4& view, const Math::Mat4& cameraWorld,
            DirectionalLightDataGpu& out) const
    {
        memset(&out, 0, sizeof(out));
        const float* params = light.params;
        const Math::Vec3 direction =
                view.TransformVector(light.transform.TransformVector(Math::Vec3(0.0f, 0.0f, 1.0f)))
                        .Normalized();
        out.direction[0] = direction.x;
        out.direction[1] = direction.y;
        out.direction[2] = direction.z;
        const float sign = light.negative ? -1.0f : 1.0f;
        out.energy =
                sign * params[static_cast<std::size_t>(LightParam::Energy)] * 3.14159265358979f;
        out.color[0] = srgbToLinear(light.color.x);
        out.color[1] = srgbToLinear(light.color.y);
        out.color[2] = srgbToLinear(light.color.z);
        out.specular = params[static_cast<std::size_t>(LightParam::Specular)];
        out.volumetricFogEnergy = 1.0f;
        out.mask = light.cullMask;
        out.size = 1.0f - cosf(params[static_cast<std::size_t>(LightParam::Size)] *
                                  (3.14159265358979f / 180.0f));
        out.sscsIndex = 0xffffffffu;
        out.bakeMode = static_cast<std::uint32_t>(light.bakeMode);
        if (!light.shadowActive) return;

        const float opacity = params[static_cast<std::size_t>(LightParam::ShadowOpacity)];
        out.shadowOpacity = opacity;
        if (opacity <= 0.001f) return;

        const float size = params[static_cast<std::size_t>(LightParam::Size)];
        float angularDiameter = 0.0f;
        if (size > 0.0f) angularDiameter = tanf(size * (3.14159265358979f / 180.0f));
        const float blur = params[static_cast<std::size_t>(LightParam::ShadowBlur)];
        out.softShadowScale = blur;
        out.softshadowAngle = angularDiameter;
        if (angularDiameter <= 0.0f) out.softShadowScale *= directionalQualityRadius;

        const DirectionalShadowMode mode = light.shadowMode;
        const int limit = mode == DirectionalShadowMode::Orthogonal
                                  ? 0
                                  : (mode == DirectionalShadowMode::Parallel2Splits ? 1 : 3);
        out.blendSplits =
                (mode != DirectionalShadowMode::Orthogonal && light.blendSplits) ? 1u : 0u;

        Math::Mat4 bias = Math::Mat4::Identity();
        bias.col0.x = 0.5f;
        bias.col1.y = 0.5f;
        bias.col2.z = 0.5f;
        bias.col3.x = 0.5f;
        bias.col3.y = 0.5f;
        bias.col3.z = 0.5f;

        for (int j = 0; j < 4; ++j)
        {
            const Light::Cascade& cascade = light.cascades[j];
            Math::Mat4 rect = Math::Mat4::Identity();
            rect.col0.x = cascade.atlasRect[2];
            rect.col1.y = cascade.atlasRect[3];
            rect.col3.x = cascade.atlasRect[0];
            rect.col3.y = cascade.atlasRect[1];
            const Math::Mat4 modelview = cascade.transform.InverseAffine() * cameraWorld;
            const Math::Mat4 matrix = rect * bias * cascade.projection * modelview;
            const float split = j <= limit ? cascade.split : 0.0f;
            out.shadowSplitOffsets[j] = split;
            const float biasScale = cascade.biasScale * out.softShadowScale;
            out.shadowBias[j] =
                    params[static_cast<std::size_t>(LightParam::ShadowBias)] / 100.0f * biasScale;
            out.shadowNormalBias[j] =
                    params[static_cast<std::size_t>(LightParam::ShadowNormalBias)] *
                    cascade.texelSize;
            out.shadowTransmittanceBias[j] = 0.0f;
            out.shadowZRange[j] = cascade.zFar;
            out.shadowRangeBegin[j] = cascade.rangeBegin;
            storeMatrix(matrix, out.shadowMatrix[j]);
            out.uvScale[j][0] = cascade.uvScale[0] * cascade.atlasRect[2];
            out.uvScale[j][1] = cascade.uvScale[1] * cascade.atlasRect[3];
        }
        const float fadeStart = params[static_cast<std::size_t>(LightParam::ShadowFadeStart)];
        out.fadeFrom = -out.shadowSplitOffsets[limit] * fminf(fadeStart, 0.999f);
        out.fadeTo = -out.shadowSplitOffsets[limit];
    }

    std::uint32_t uploadLights(prisma::Driver* driver, ScenarioId scenario, const Math::Mat4& view,
            const Math::Mat4& cameraWorld, const Math::Frustum& frustum, ClusterBuilder* cluster,
            bool none)
    {
        omniLightData.clear();
        spotLightData.clear();
        positionalSoftShadows = false;
        std::uint32_t directionalCount = 0;
        for (const Light& light: lights)
        {
            if (none || !light.visible || light.scenario != scenario) continue;
            if (light.type == LightType::Directional)
            {
                if (directionalCount < kMaxDirectionalLights)
                    packDirectional(light, view, cameraWorld,
                            directionalLightData[directionalCount++]);
                continue;
            }
            const float range = light.params[static_cast<std::size_t>(LightParam::Range)];
            if (!frustum.IntersectsSphere(light.transform.GetTranslation(), range)) continue;
            LightDataGpu data;
            packLight(light, view, data);
            if (light.params[static_cast<std::size_t>(LightParam::Size)] > 0.0001f)
                positionalSoftShadows = true;
            if (light.positional.active) packPositionalShadow(light, cameraWorld, data);
            if (light.type == LightType::Omni)
            {
                omniLightData.push_back(data);
                if (cluster)
                    cluster->addLight(ClusterBuilder::kLightOmni, light.transform, range, 0.0f);
            }
            else
            {
                spotLightData.push_back(data);
                if (cluster)
                    cluster->addLight(ClusterBuilder::kLightSpot, light.transform, range,
                            light.params[static_cast<std::size_t>(LightParam::SpotAngle)]);
            }
        }
        lightCounts.omni = static_cast<std::uint32_t>(omniLightData.size());
        lightCounts.spot = static_cast<std::uint32_t>(spotLightData.size());

        LightDataGpu empty;
        memset(&empty, 0, sizeof(empty));
        if (omniLightData.empty()) omniLightData.push_back(empty);
        if (spotLightData.empty()) spotLightData.push_back(empty);
        const std::uint32_t omniBytes =
                static_cast<std::uint32_t>(omniLightData.size() * sizeof(LightDataGpu));
        const std::uint32_t spotBytes =
                static_cast<std::uint32_t>(spotLightData.size() * sizeof(LightDataGpu));
        if (!ensureStorage(driver, omniLightBuffer, omniLightBufferSize, omniBytes,
                    "omni lights") ||
                !ensureStorage(driver, spotLightBuffer, spotLightBufferSize, spotBytes,
                        "spot lights"))
            return 0;
        driver->updateBuffer(omniLightBuffer, 0, omniLightData.data(), omniBytes);
        driver->updateBuffer(spotLightBuffer, 0, spotLightData.data(), spotBytes);
        driver->updateBuffer(lightCountsBuffer, 0, &lightCounts, sizeof(lightCounts));
        if (directionalCount > 0)
            driver->updateBuffer(directionalLightBuffer, 0, directionalLightData,
                    directionalCount * sizeof(DirectionalLightDataGpu));
        return directionalCount;
    }

    std::uint32_t uploadDecals(prisma::Driver* driver, ScenarioId scenario,
            const Math::Mat4& cameraWorld, const Math::Frustum& frustum, ClusterBuilder* cluster,
            bool none)
    {
        decalAtlas.update();

        decalInputs.clear();
        for (const Decal& decal: decals)
        {
            if (none || !decal.visible || decal.scenario != scenario) continue;
            const Math::Box bounds = Math::Box::FromCenterExtents(Math::Vec3(0.0f, 0.0f, 0.0f),
                    decal.params.size * 0.5f)
                                             .Transformed(decal.transform);
            if (!frustum.IntersectsBox(bounds)) continue;
            DecalInstanceInput input;
            input.decal = &decal.params;
            input.transform = decal.transform;
            input.sortingOffset = decal.sortingOffset;
            for (std::size_t i = 0; i < static_cast<std::size_t>(DecalTexture::Max); ++i)
                input.rects[i] = decalAtlas.getTextureRect(decal.params.textures[i]);
            decalInputs.push_back(input);
        }

        decalCount =
                buildDecalData(decalInputs.data(), static_cast<std::uint32_t>(decalInputs.size()),
                        cameraWorld, kMaxDecals, decalData, decalSorted);
        if (cluster)
        {
            for (std::size_t i = 0; i < decalSorted.size(); ++i)
                cluster->addBox(ClusterBuilder::kBoxDecal, decalSorted[i].transform,
                        decalSorted[i].extents);
        }

        if (decalData.empty())
        {
            DecalDataGpu empty;
            memset(&empty, 0, sizeof(empty));
            decalData.push_back(empty);
        }
        const std::uint32_t bytes =
                static_cast<std::uint32_t>(decalData.size() * sizeof(DecalDataGpu));
        if (!ensureStorage(driver, decalBuffer, decalBufferSize, bytes, "decals")) return 0;
        driver->updateBuffer(decalBuffer, 0, decalData.data(), bytes);
        return decalCount;
    }

    std::uint32_t uploadReflectionProbes(prisma::Driver* driver, ScenarioId scenario,
            const Math::Mat4& cameraWorld, ClusterBuilder* cluster, bool none)
    {
        reflectionInputs.clear();
        for (const ReflectionProbe& probe: reflectionProbes)
        {
            if (none || !probe.visible || probe.scenario != scenario) continue;
            ReflectionInstanceInput input;
            input.probe = &probe.params;
            input.transform = probe.transform;
            input.atlasIndex = -1;
            reflectionInputs.push_back(input);
        }

        reflectionCount = buildReflectionData(reflectionInputs.data(),
                static_cast<std::uint32_t>(reflectionInputs.size()), cameraWorld, 1.0f,
                kMaxReflectionProbes, reflectionData, reflectionSorted);
        if (cluster)
        {
            for (std::size_t i = 0; i < reflectionSorted.size(); ++i)
                cluster->addBox(ClusterBuilder::kBoxReflectionProbe, reflectionSorted[i].transform,
                        reflectionSorted[i].extents);
        }

        if (reflectionData.empty())
        {
            ReflectionDataGpu empty;
            memset(&empty, 0, sizeof(empty));
            reflectionData.push_back(empty);
        }
        const std::uint32_t bytes =
                static_cast<std::uint32_t>(reflectionData.size() * sizeof(ReflectionDataGpu));
        if (!ensureStorage(driver, reflectionBuffer, reflectionBufferSize, bytes,
                    "reflection probes"))
            return 0;
        driver->updateBuffer(reflectionBuffer, 0, reflectionData.data(), bytes);
        return reflectionCount;
    }

    bool tapCapture(const Math::Vec3& world, std::uint32_t& slot, float* out) const
    {
        std::size_t bestCells = 0;
        bool found = false;
        for (std::uint32_t l = 0; l < lightmapsUsed; ++l)
        {
            const Lightmap* lightmap = lightmapRefs[l];
            if (!lightmap || lightmap->probeSh.empty()) continue;
            const std::uint32_t* dims = lightmap->probeDims;
            const std::size_t cells = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
            if (found && cells <= bestCells) continue;
            const Math::Vec3 local = lightmapInverse[l].TransformPoint(world);
            const float gx = (local.x - lightmap->probeOrigin.x) / lightmap->probeCell.x;
            const float gy = (local.y - lightmap->probeOrigin.y) / lightmap->probeCell.y;
            const float gz = (local.z - lightmap->probeOrigin.z) / lightmap->probeCell.z;
            if (gx < -0.5f || gy < -0.5f || gz < -0.5f || gx > static_cast<float>(dims[0]) - 0.5f ||
                    gy > static_cast<float>(dims[1]) - 0.5f ||
                    gz > static_cast<float>(dims[2]) - 0.5f)
                continue;
            const float cx = fminf(fmaxf(gx, 0.0f), static_cast<float>(dims[0] - 1));
            const float cy = fminf(fmaxf(gy, 0.0f), static_cast<float>(dims[1] - 1));
            const float cz = fminf(fmaxf(gz, 0.0f), static_cast<float>(dims[2] - 1));
            const std::uint32_t x0 = static_cast<std::uint32_t>(cx);
            const std::uint32_t y0 = static_cast<std::uint32_t>(cy);
            const std::uint32_t z0 = static_cast<std::uint32_t>(cz);
            const std::uint32_t x1 = x0 + 1 < dims[0] ? x0 + 1 : x0;
            const std::uint32_t y1 = y0 + 1 < dims[1] ? y0 + 1 : y0;
            const std::uint32_t z1 = z0 + 1 < dims[2] ? z0 + 1 : z0;
            const float fx = cx - static_cast<float>(x0);
            const float fy = cy - static_cast<float>(y0);
            const float fz = cz - static_cast<float>(z0);
            const float* sh = lightmap->probeSh.data();
            auto cell = [&](std::uint32_t x, std::uint32_t y, std::uint32_t z)
            {
                return sh + (static_cast<std::size_t>(z) * dims[1] * dims[0] +
                                    static_cast<std::size_t>(y) * dims[0] + x) *
                                    27;
            };
            const float* c000 = cell(x0, y0, z0);
            const float* c100 = cell(x1, y0, z0);
            const float* c010 = cell(x0, y1, z0);
            const float* c110 = cell(x1, y1, z0);
            const float* c001 = cell(x0, y0, z1);
            const float* c101 = cell(x1, y0, z1);
            const float* c011 = cell(x0, y1, z1);
            const float* c111 = cell(x1, y1, z1);
            for (int c = 0; c < 27; ++c)
            {
                const float a = c000[c] + (c100[c] - c000[c]) * fx;
                const float b = c010[c] + (c110[c] - c010[c]) * fx;
                const float d = c001[c] + (c101[c] - c001[c]) * fx;
                const float e = c011[c] + (c111[c] - c011[c]) * fx;
                const float f = a + (b - a) * fy;
                const float g = d + (e - d) * fy;
                out[(c / 3) * 4 + (c % 3)] = f + (g - f) * fz;
            }
            for (int i = 0; i < 9; ++i) out[i * 4 + 3] = 0.0f;
            slot = l;
            bestCells = cells;
            found = true;
        }
        return found;
    }

    void setupLightmaps(prisma::Driver* driver, const TextureStore& textures, ScenarioId scenario,
            const Math::Frustum& frustum, const Math::Mat4& cameraWorld)
    {
        lightmapsUsed = 0;
        lightmapSpecularFrame = false;
        const Math::Mat3 cameraBasis(cameraWorld);
        const auto lightmapItems = lightmaps.items();
        for (std::size_t i = 0; i < lightmapItems.size(); ++i)
        {
            const Lightmap& lightmap = lightmapItems[i];
            if (!lightmap.visible || lightmap.scenario != scenario) continue;
            const TextureInfo* light = textures.get(lightmap.light);
            if (!light || !light->gpu.valid() || light->type != prisma::TextureType::Texture2DArray)
                continue;
            if (lightmap.hasBounds &&
                    !frustum.IntersectsBox(lightmap.bounds.Transformed(lightmap.transform)))
                continue;
            if (lightmapsUsed >= kMaxLightmaps)
            {
                if (!lightmapWarnedMax)
                {
                    lightmapWarnedMax = true;
                    log_warn("render: more than %u lightmaps are visible, the extra ones are not "
                             "drawn",
                            kMaxLightmaps);
                }
                break;
            }
            const std::uint32_t slot = lightmapsUsed;
            LightmapDataGpu& data = lightmapData[slot];
            memset(&data, 0, sizeof(data));
            Math::Mat3 toLightmap = Math::Mat3(lightmap.transform).Inverse() * cameraBasis;
            toLightmap = toLightmap.Inverse().Transposed();
            for (int c = 0; c < 3; ++c)
            {
                data.normalTransformAndSpecular[c * 4 + 0] = toLightmap[c].x;
                data.normalTransformAndSpecular[c * 4 + 1] = toLightmap[c].y;
                data.normalTransformAndSpecular[c * 4 + 2] = toLightmap[c].z;
                data.normalTransformAndSpecular[c * 4 + 3] = 0.0f;
            }
            data.textureSize[0] = static_cast<float>(light->width);
            data.textureSize[1] = static_cast<float>(light->height);
            data.exposureNormalization = 1.0f;
            data.flags = static_cast<std::uint32_t>(lightmap.shadowmaskMode);
            const TextureInfo* shadowmask = textures.get(lightmap.shadowmask);
            if (lightmap.shadowmaskMode != ShadowmaskMode::None &&
                    (!shadowmask || !shadowmask->gpu.valid() ||
                            shadowmask->type != prisma::TextureType::Texture2DArray))
                data.flags = static_cast<std::uint32_t>(ShadowmaskMode::None);
            data.normalTransformAndSpecular[3] = lightmap.specularIntensity;
            lightmapIds[slot] = toId<LightmapId>(lightmaps.handle_at(i));
            lightmapHasSh[slot] = lightmap.usesSphericalHarmonics;
            lightmapRefs[slot] = &lightmap;
            lightmapInverse[slot] = lightmap.transform.InverseAffine();
            lightmapHasSpecular[slot] =
                    lightmap.usesSphericalHarmonics && lightmap.specularIntensity > 0.0f;
            lightmapLight[slot] = light->gpu;
            lightmapShadowmask[slot] = data.flags != 0 && shadowmask ? shadowmask->gpu : light->gpu;
            if (lightmapHasSpecular[slot]) lightmapSpecularFrame = true;
            ++lightmapsUsed;
        }
        if (lightmapsUsed == 0) return;
        const std::uint32_t bytes =
                static_cast<std::uint32_t>(sizeof(LightmapDataGpu)) * lightmapsUsed;
        if (!ensureStorage(driver, lightmapBuffer, lightmapBufferSize, bytes, "lightmaps"))
        {
            lightmapsUsed = 0;
            return;
        }
        driver->updateBuffer(lightmapBuffer, 0, lightmapData, bytes);
        if (!lightmapCaptureBuffer.valid() &&
                ensureStorage(driver, lightmapCaptureBuffer, lightmapCaptureSize,
                        static_cast<std::uint32_t>(36 * sizeof(float)), "lightmap captures"))
        {
            float zero[36] = {};
            driver->updateBuffer(lightmapCaptureBuffer, 0, zero, sizeof(zero));
        }
        if (!lightmapCaptureBuffer.valid()) lightmapsUsed = 0;
    }

    void createLightBuffers(prisma::Driver* driver)
    {
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Uniform;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = sizeof(DirectionalLightDataGpu) * kMaxDirectionalLights;
        desc.debugName = "directional lights";
        directionalLightBuffer = driver->createBuffer(desc);
        desc.size = sizeof(LightCountsGpu);
        desc.debugName = "light counts";
        lightCountsBuffer = driver->createBuffer(desc);
        if (!directionalLightBuffer.valid() || !lightCountsBuffer.valid())
            log_error("render: could not create the light buffers");
    }

    void createSceneBuffer(prisma::Driver* driver)
    {
        createLightBuffers(driver);
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Uniform;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = sizeof(SceneUboData);
        desc.debugName = "scene data";
        sceneBuffer = driver->createBuffer(desc);
        if (!sceneBuffer.valid()) log_error("render: could not create the scene data buffer");
        desc.debugName = "depth pre-pass scene data";
        prepassSceneBuffer = driver->createBuffer(desc);
        if (!prepassSceneBuffer.valid())
            log_error("render: could not create the depth pre-pass scene data buffer");
        startNanos = time_nanos();
    }

    void createTonemap(prisma::Driver* driver)
    {
        const prisma::Caps& caps = driver->caps();
        tonemapVertex = driver->createShader(prisma3d::shaderDesc(tonemap_vert, caps));
        tonemapFragment = driver->createShader(prisma3d::shaderDesc(tonemap_frag, caps));

        prisma::SamplerDesc samplerDesc;
        samplerDesc.minFilter = prisma::Filter::Linear;
        samplerDesc.magFilter = prisma::Filter::Linear;
        samplerDesc.mipFilter = prisma::MipFilter::None;
        samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
        samplerDesc.debugName = "linear clamp";
        linearSampler = driver->createSampler(samplerDesc);

        prisma::BufferDesc bufferDesc;
        bufferDesc.usage = prisma::BufferUsage::Uniform;
        bufferDesc.update = prisma::BufferUpdate::Dynamic;
        bufferDesc.size = sizeof(TonemapParams);
        bufferDesc.debugName = "tonemap params";
        tonemapParams = driver->createBuffer(bufferDesc);

        static const char* const textureNames[5] = { "source_color", "source_auto_exposure",
            "source_glow", "glow_map", "source_color_correction" };
        prisma::PipelineDesc desc;
        desc.vertexShader = tonemapVertex;
        desc.fragmentShader = tonemapFragment;
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "Params";
        desc.uniformBlocks[0].slot = 0;
        desc.textureCount = 5;
        for (std::uint32_t i = 0; i < 5; ++i)
        {
            desc.textures[i].name = textureNames[i];
            desc.textures[i].slot = i;
        }
        desc.depthTest = false;
        desc.depthWrite = false;
        desc.debugName = "tonemap";
        tonemapDescription = desc;
        if (tonemapVertex.valid() && tonemapFragment.valid() && linearSampler.valid() &&
                tonemapParams.valid())
            tonemapPipeline = driver->createPipeline(desc);
        if (!tonemapPipeline.valid()) log_error("render: could not create the tonemap pipeline");

        if (tonemapVertex.valid() && tonemapFragment.valid() && linearSampler.valid() &&
                tonemapParams.valid())
        {
            desc.targets.window = false;
            desc.targets.colorCount = 1;
            desc.targets.depth = prisma::TextureFormat::None;
            desc.targets.colors[0] = prisma::TextureFormat::RGBA8Srgb;
            desc.debugName = "tonemap texture srgb";
            tonemapTexturePipelines[0] = driver->createPipeline(desc);
            desc.targets.colors[0] = prisma::TextureFormat::RGBA8;
            desc.debugName = "tonemap texture linear";
            tonemapTexturePipelines[1] = driver->createPipeline(desc);
        }
        if (!tonemapTexturePipelines[0].valid() || !tonemapTexturePipelines[1].valid())
            log_error("render: could not create the viewport texture pipelines");

        upscaleFragment = driver->createShader(prisma3d::shaderDesc(upscale_frag, caps));
        bufferDesc.size = sizeof(UpscaleParams);
        bufferDesc.debugName = "upscale params";
        upscaleParams = driver->createBuffer(bufferDesc);
        prisma::PipelineDesc upscaleDesc;
        upscaleDesc.vertexShader = tonemapVertex;
        upscaleDesc.fragmentShader = upscaleFragment;
        upscaleDesc.uniformBlockCount = 1;
        upscaleDesc.uniformBlocks[0].name = "UpscaleParams";
        upscaleDesc.uniformBlocks[0].slot = 0;
        upscaleDesc.textureCount = 1;
        upscaleDesc.textures[0].name = "source";
        upscaleDesc.textures[0].slot = 0;
        upscaleDesc.depthTest = false;
        upscaleDesc.depthWrite = false;
        upscaleDesc.debugName = "upscale";
        if (tonemapVertex.valid() && upscaleFragment.valid() && upscaleParams.valid())
            upscalePipeline = driver->createPipeline(upscaleDesc);
        if (!upscalePipeline.valid())
            log_error("render: could not create the upscale pipeline (the 3D is scaled in the "
                      "tonemap)");
        if (caps.compute)
        {
            fsrShader = driver->createShader(prisma3d::shaderDesc(*kFsrUpscaleBlobs[0], caps));
            prisma::ComputePipelineDesc fsrDesc;
            fsrDesc.shader = fsrShader;
            fsrDesc.uniformBlockCount = 1;
            fsrDesc.uniformBlocks[0].name = "Params";
            fsrDesc.uniformBlocks[0].slot = 0;
            fsrDesc.textureCount = 1;
            fsrDesc.textures[0].name = "source_image";
            fsrDesc.textures[0].slot = 0;
            fsrDesc.storageTextureCount = 1;
            fsrDesc.storageTextures[0].name = "fsr_image";
            fsrDesc.storageTextures[0].slot = 0;
            fsrDesc.debugName = "fsr upscale";
            if (fsrShader.valid()) fsrPipeline = driver->createComputePipeline(fsrDesc);
            bufferDesc.size = sizeof(FsrParams);
            for (int i = 0; i < 2; ++i)
            {
                bufferDesc.debugName = i == 0 ? "fsr easu params" : "fsr rcas params";
                fsrParams[i] = driver->createBuffer(bufferDesc);
                if (!fsrParams[i].valid()) fsrPipeline = prisma::PipelineHandle();
            }
            if (!fsrPipeline.valid())
                log_error("render: could not create the FSR pipeline (scaling_3d_mode FSR scales "
                          "like Bilinear)");
        }
        createSmaa(driver, caps);

        depthReadFragment = driver->createShader(prisma3d::shaderDesc(depth_read_frag, caps));
        samplerDesc.minFilter = prisma::Filter::Nearest;
        samplerDesc.magFilter = prisma::Filter::Nearest;
        samplerDesc.debugName = "nearest clamp";
        nearestSampler = driver->createSampler(samplerDesc);
        bufferDesc.size = sizeof(float) * 4;
        bufferDesc.debugName = "depth read params";
        depthReadParams = driver->createBuffer(bufferDesc);
        prisma::TextureDesc targetDesc;
        targetDesc.width = 1;
        targetDesc.height = 1;
        targetDesc.format = prisma::TextureFormat::RGBA8;
        targetDesc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        targetDesc.debugName = "depth read";
        depthReadTarget = driver->createTexture(targetDesc);

        prisma::PipelineDesc readDesc;
        readDesc.vertexShader = tonemapVertex;
        readDesc.fragmentShader = depthReadFragment;
        readDesc.uniformBlockCount = 1;
        readDesc.uniformBlocks[0].name = "DepthReadParams";
        readDesc.uniformBlocks[0].slot = 0;
        readDesc.textureCount = 1;
        readDesc.textures[0].name = "source_depth";
        readDesc.textures[0].slot = 0;
        readDesc.targets.window = false;
        readDesc.targets.colors[0] = prisma::TextureFormat::RGBA8;
        readDesc.targets.colorCount = 1;
        readDesc.targets.depth = prisma::TextureFormat::None;
        readDesc.depthTest = false;
        readDesc.depthWrite = false;
        readDesc.cullMode = prisma::CullMode::None;
        readDesc.debugName = "depth read";
        if (tonemapVertex.valid() && depthReadFragment.valid() && nearestSampler.valid() &&
                depthReadParams.valid() && depthReadTarget.valid())
            depthReadPipeline = driver->createPipeline(readDesc);
        if (!depthReadPipeline.valid())
            log_error("render: could not create the depth read pipeline");

        screenCopyFragment = driver->createShader(prisma3d::shaderDesc(screen_copy_frag, caps));
        samplerDesc.minFilter = prisma::Filter::Linear;
        samplerDesc.magFilter = prisma::Filter::Linear;
        samplerDesc.mipFilter = prisma::MipFilter::Linear;
        samplerDesc.debugName = "screen copy";
        screenSampler = driver->createSampler(samplerDesc);
        prisma::PipelineDesc copyDesc;
        copyDesc.vertexShader = tonemapVertex;
        copyDesc.fragmentShader = screenCopyFragment;
        copyDesc.textureCount = 1;
        copyDesc.textures[0].name = "source";
        copyDesc.textures[0].slot = 0;
        copyDesc.targets.window = false;
        copyDesc.targets.colorCount = 1;
        copyDesc.targets.depth = prisma::TextureFormat::None;
        copyDesc.depthTest = false;
        copyDesc.depthWrite = false;
        copyDesc.cullMode = prisma::CullMode::None;
        copyDesc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
        copyDesc.debugName = "screen copy color";
        if (tonemapVertex.valid() && screenCopyFragment.valid())
            screenCopyColorPipeline = driver->createPipeline(copyDesc);
        copyDesc.targets.colors[0] = prisma::TextureFormat::R32F;
        copyDesc.debugName = "screen copy depth";
        if (tonemapVertex.valid() && screenCopyFragment.valid())
            screenCopyDepthPipeline = driver->createPipeline(copyDesc);
        if (!screenCopyColorPipeline.valid() || !screenCopyDepthPipeline.valid())
            log_error("render: could not create the screen copy pipelines");
    }

    static void destroyScreenTextures(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.screenColor.valid()) driver->destroy(viewport.screenColor);
        if (viewport.screenDepth.valid()) driver->destroy(viewport.screenDepth);
        viewport.screenColor = prisma::TextureHandle();
        viewport.screenDepth = prisma::TextureHandle();
        viewport.screenWidth = 0;
        viewport.screenHeight = 0;
        viewport.screenMips = 0;
    }

    static bool ensureScreenTextures(prisma::Driver* driver, Viewport& viewport,
            std::uint32_t width, std::uint32_t height)
    {
        if (viewport.screenColor.valid() && viewport.screenDepth.valid() &&
                viewport.screenWidth == width && viewport.screenHeight == height)
            return true;
        destroyScreenTextures(driver, viewport);
        std::uint32_t mips = 1;
        for (std::uint32_t side = width > height ? width : height; side > 1; side >>= 1) ++mips;
        prisma::TextureDesc desc;
        desc.width = width;
        desc.height = height;
        desc.format = prisma::TextureFormat::RGBA16F;
        desc.mipLevels = mips;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "viewport screen color";
        viewport.screenColor = driver->createTexture(desc);
        desc.format = prisma::TextureFormat::R32F;
        desc.mipLevels = 1;
        desc.debugName = "viewport screen depth";
        viewport.screenDepth = driver->createTexture(desc);
        if (!viewport.screenColor.valid() || !viewport.screenDepth.valid())
        {
            destroyScreenTextures(driver, viewport);
            log_error("render: could not create the screen copy textures");
            return false;
        }
        viewport.screenWidth = width;
        viewport.screenHeight = height;
        viewport.screenMips = mips;
        return true;
    }

    void copyScreen(prisma::Driver* driver, Viewport& viewport, const prisma::Viewport& area)
    {
        for (int step = 0; step < 2; ++step)
        {
            prisma::RenderPassDesc pass;
            pass.colors[0].texture = step == 0 ? viewport.screenColor : viewport.screenDepth;
            pass.colorCount = 1;
            pass.colorLoad = prisma::LoadOp::DontCare;
            pass.depthLoad = prisma::LoadOp::DontCare;
            pass.depthStore = prisma::StoreOp::Discard;
            driver->beginRenderPass(pass);
            driver->setViewport(area);
            driver->bindPipeline(step == 0 ? screenCopyColorPipeline : screenCopyDepthPipeline);
            driver->bindTexture(0, step == 0 ? viewport.colorTexture : viewport.depthTexture,
                    nearestSampler);
            driver->draw(3, 0);
            driver->endRenderPass();
        }
        if (viewport.screenMips > 1) driver->generateMipmaps(viewport.screenColor);
    }

    void destroyTonemap(prisma::Driver* driver)
    {
        if (planarBuffer.valid()) driver->destroy(planarBuffer);
        if (screenCopyColorPipeline.valid()) driver->destroy(screenCopyColorPipeline);
        if (screenCopyDepthPipeline.valid()) driver->destroy(screenCopyDepthPipeline);
        if (screenCopyFragment.valid()) driver->destroy(screenCopyFragment);
        if (screenSampler.valid()) driver->destroy(screenSampler);
        if (glowPipeline.valid()) driver->destroy(glowPipeline);
        if (glowParams.valid()) driver->destroy(glowParams);
        if (lensFlarePipeline.valid()) driver->destroy(lensFlarePipeline);
        if (lensFlareParams.valid()) driver->destroy(lensFlareParams);
        if (lensFlareVertex.valid()) driver->destroy(lensFlareVertex);
        if (lensFlareFragment.valid()) driver->destroy(lensFlareFragment);
        if (glowSampler.valid()) driver->destroy(glowSampler);
        if (glowFragment.valid()) driver->destroy(glowFragment);
        if (depthReadPipeline.valid()) driver->destroy(depthReadPipeline);
        if (depthReadTarget.valid()) driver->destroy(depthReadTarget);
        if (depthReadParams.valid()) driver->destroy(depthReadParams);
        if (nearestSampler.valid()) driver->destroy(nearestSampler);
        if (depthReadFragment.valid()) driver->destroy(depthReadFragment);
        if (tonemapPipeline.valid()) driver->destroy(tonemapPipeline);
        for (prisma::PipelineHandle& pipeline: tonemapTexturePipelines)
            if (pipeline.valid()) driver->destroy(pipeline);
        for (prisma::PipelineHandle& pipeline: tonemapMaskedPipelines)
            if (pipeline.valid()) driver->destroy(pipeline);
        if (tonemapParams.valid()) driver->destroy(tonemapParams);
        if (upscalePipeline.valid()) driver->destroy(upscalePipeline);
        if (upscaleParams.valid()) driver->destroy(upscaleParams);
        if (upscaleFragment.valid()) driver->destroy(upscaleFragment);
        if (fsrPipeline.valid()) driver->destroy(fsrPipeline);
        for (int i = 0; i < 2; ++i)
            if (fsrParams[i].valid()) driver->destroy(fsrParams[i]);
        if (fsrShader.valid()) driver->destroy(fsrShader);
        if (smaaEdgePipeline.valid()) driver->destroy(smaaEdgePipeline);
        if (smaaWeightPipeline.valid()) driver->destroy(smaaWeightPipeline);
        if (smaaBlendPipeline.valid()) driver->destroy(smaaBlendPipeline);
        for (int i = 0; i < 3; ++i)
            if (smaaParams[i].valid()) driver->destroy(smaaParams[i]);
        for (int i = 0; i < 6; ++i)
            if (smaaShaders[i].valid()) driver->destroy(smaaShaders[i]);
        if (smaaAreaTex.valid()) driver->destroy(smaaAreaTex);
        if (smaaSearchTex.valid()) driver->destroy(smaaSearchTex);
        if (linearSampler.valid()) driver->destroy(linearSampler);
        if (tonemapVertex.valid()) driver->destroy(tonemapVertex);
        if (tonemapFragment.valid()) driver->destroy(tonemapFragment);
    }

    bool ensureTonemapMasked(prisma::Driver* driver)
    {
        if (tonemapMaskedPipelines[0].valid() && tonemapMaskedPipelines[1].valid()) return true;
        if (!tonemapVertex.valid() || !tonemapFragment.valid()) return false;
        prisma::PipelineDesc desc = tonemapDescription;
        desc.targets.window = false;
        desc.targets.colorCount = 1;
        desc.targets.depth = prisma::TextureFormat::None;
        desc.colorMask = prisma::kColorRed | prisma::kColorGreen | prisma::kColorBlue;
        if (!tonemapMaskedPipelines[0].valid())
        {
            desc.targets.colors[0] = prisma::TextureFormat::RGBA8Srgb;
            desc.debugName = "tonemap texture srgb opaque";
            tonemapMaskedPipelines[0] = driver->createPipeline(desc);
        }
        if (!tonemapMaskedPipelines[1].valid())
        {
            desc.targets.colors[0] = prisma::TextureFormat::RGBA8;
            desc.debugName = "tonemap texture linear opaque";
            tonemapMaskedPipelines[1] = driver->createPipeline(desc);
        }
        if (!tonemapMaskedPipelines[0].valid() || !tonemapMaskedPipelines[1].valid())
        {
            log_error("render: could not create the viewport texture pipelines for depth of "
                      "field");
            return false;
        }
        return true;
    }

    bool ensureGlow(prisma::Driver* driver)
    {
        if (glowReady) return true;
        if (glowFailed) return false;
        glowFailed = true;
        if (!tonemapVertex.valid()) return false;

        glowFragment = driver->createShader(prisma3d::shaderDesc(glow_blur_frag, driver->caps()));

        prisma::SamplerDesc samplerDesc;
        samplerDesc.minFilter = prisma::Filter::Linear;
        samplerDesc.magFilter = prisma::Filter::Linear;
        samplerDesc.mipFilter = prisma::MipFilter::Linear;
        samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
        samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
        samplerDesc.debugName = "glow";
        glowSampler = driver->createSampler(samplerDesc);

        prisma::BufferDesc bufferDesc;
        bufferDesc.usage = prisma::BufferUsage::Uniform;
        bufferDesc.update = prisma::BufferUpdate::Dynamic;
        bufferDesc.size = glowParamsStride(driver) * kGlowMaxPasses;
        bufferDesc.debugName = "glow params";
        glowParams = driver->createBuffer(bufferDesc);

        prisma::PipelineDesc desc;
        desc.vertexShader = tonemapVertex;
        desc.fragmentShader = glowFragment;
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "Params";
        desc.uniformBlocks[0].slot = 0;
        desc.textureCount = 1;
        desc.textures[0].name = "source_color";
        desc.textures[0].slot = 0;
        desc.targets.window = false;
        desc.targets.colorCount = 1;
        desc.targets.depth = prisma::TextureFormat::None;
        desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
        desc.depthTest = false;
        desc.depthWrite = false;
        desc.debugName = "glow blur";
        if (glowFragment.valid() && glowSampler.valid() && glowParams.valid())
            glowPipeline = driver->createPipeline(desc);
        if (!glowPipeline.valid())
        {
            log_error("render: could not create the glow pipeline");
            return false;
        }
        glowReady = true;
        glowFailed = false;
        return true;
    }

    bool ensureLensFlare(prisma::Driver* driver)
    {
        if (lensFlareReady) return true;
        if (lensFlareFailed) return false;
        lensFlareFailed = true;
        const prisma::Caps& caps = driver->caps();
        lensFlareVertex = driver->createShader(prisma3d::shaderDesc(lensflare_vert, caps));
        lensFlareFragment = driver->createShader(prisma3d::shaderDesc(lensflare_frag, caps));

        prisma::BufferDesc bufferDesc;
        bufferDesc.usage = prisma::BufferUsage::Uniform;
        bufferDesc.update = prisma::BufferUpdate::Dynamic;
        bufferDesc.size = lensFlareParamsStride(driver) * kLensFlareMaxDraws;
        bufferDesc.debugName = "lens flare params";
        lensFlareParams = driver->createBuffer(bufferDesc);

        prisma::PipelineDesc desc;
        desc.vertexShader = lensFlareVertex;
        desc.fragmentShader = lensFlareFragment;
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "LensFlareParams";
        desc.uniformBlocks[0].slot = 0;
        desc.textureCount = 2;
        desc.textures[0].name = "source_depth";
        desc.textures[0].slot = 0;
        desc.textures[1].name = "flare_texture";
        desc.textures[1].slot = 1;
        desc.topology = prisma::Topology::TriangleStrip;
        desc.targets.window = false;
        desc.targets.colorCount = 1;
        desc.targets.depth = prisma::TextureFormat::None;
        desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
        desc.depthTest = false;
        desc.depthWrite = false;
        desc.cullMode = prisma::CullMode::None;
        desc.blend = true;
        desc.srcColor = prisma::BlendFactor::One;
        desc.dstColor = prisma::BlendFactor::One;
        desc.srcAlpha = prisma::BlendFactor::Zero;
        desc.dstAlpha = prisma::BlendFactor::One;
        desc.debugName = "lens flare";
        if (lensFlareVertex.valid() && lensFlareFragment.valid() && lensFlareParams.valid())
            lensFlarePipeline = driver->createPipeline(desc);
        if (!lensFlarePipeline.valid())
        {
            log_error("render: could not create the lens flare pipeline");
            return false;
        }
        if (RenderProfiler* profiler = RenderProfiler::singleton())
            profiler->registerPass("Lens flare");
        lensFlareReady = true;
        lensFlareFailed = false;
        return true;
    }

    static std::uint32_t lensFlareParamsStride(prisma::Driver* driver)
    {
        return alignUp(static_cast<std::uint32_t>(sizeof(LensFlareParamsGpu)),
                driver->caps().uniformBufferOffsetAlignment);
    }

    void drawLensFlares(prisma::Driver* driver, const TextureStore& textures,
            const Viewport& viewport, const Camera& camera, const Math::Mat4& view,
            const Math::Mat4& baseProjection, const prisma::Viewport& area,
            RenderProfiler* profiler, RenderStats& stats, std::uint32_t& drawCalls)
    {
        if (!viewport.colorTexture.valid() || !viewport.depthTexture.valid()) return;
        const Math::Mat4 viewProjection = zeroToOneDepth() * baseProjection * view;
        const Math::Vec3 cameraPosition = camera.transform.GetTranslation();
        const Math::Vec3 cameraForward =
                camera.transform.TransformVector(Math::Vec3(0.0f, 0.0f, -1.0f)).Normalized();
        const std::uint32_t stride = lensFlareParamsStride(driver);
        lensFlareBytes.resize(static_cast<std::size_t>(stride) * kLensFlareMaxDraws);
        prisma::TextureHandle flareTextures[kLensFlareMaxDraws];
        std::uint32_t count = 0;
        for (const Light& light: lights)
        {
            if (count >= kLensFlareMaxDraws) break;
            if (!light.visible || light.scenario != viewport.scenario || light.lensFlareCount == 0)
                continue;
            Math::Vec3 position;
            if (light.type == LightType::Directional)
            {
                const Math::Vec3 toLight =
                        light.transform.TransformVector(Math::Vec3(0.0f, 0.0f, 1.0f)).Normalized();
                if (toLight.y < 0.0f) continue;
                position = cameraPosition + toLight * camera.zFar;
            }
            else
            {
                position = light.transform.GetTranslation();
            }
            if ((position - cameraPosition).Dot(cameraForward) <= 0.0f) continue;
            const Math::Vec4 clip = viewProjection * Math::Vec4(position, 1.0f);
            if (clip.w <= 0.0f) continue;
            const float screenX = clip.x / clip.w * 0.5f + 0.5f;
            const float screenY = clip.y / clip.w * 0.5f + 0.5f;
            const float screenZ = clip.z / clip.w;
            for (std::uint32_t i = 0; i < light.lensFlareCount && count < kLensFlareMaxDraws; ++i)
            {
                const TextureInfo* info = textures.get(light.lensFlare[i]);
                const prisma::TextureHandle gpu = textures.gpu(light.lensFlare[i]);
                if (!info || !gpu.valid() || info->width == 0 || info->height == 0) continue;
                LensFlareParamsGpu block;
                block.lightScreenPos[0] = screenX;
                block.lightScreenPos[1] = screenY;
                block.lightScreenPos[2] = screenZ;
                block.lightScreenPos[3] = kLensFlareOcclusionRadius;
                block.offsetSize[0] = kLensFlareOffsets[i];
                block.offsetSize[1] = static_cast<float>(info->width);
                block.offsetSize[2] = static_cast<float>(info->height);
                block.offsetSize[3] = 0.0f;
                block.screenSizeRcp[0] = 1.0f / area.width;
                block.screenSizeRcp[1] = 1.0f / area.height;
                block.screenSizeRcp[2] = 0.0f;
                block.screenSizeRcp[3] = 0.0f;
                memcpy(lensFlareBytes.data() + static_cast<std::size_t>(count) * stride, &block,
                        sizeof(block));
                flareTextures[count++] = gpu;
            }
        }
        if (count == 0 || !ensureLensFlare(driver)) return;

        const bool scope = profiler != nullptr && profiler->beginGpu("Lens flare");
        driver->updateBuffer(lensFlareParams, 0, lensFlareBytes.data(),
                static_cast<std::size_t>(count) * stride);
        prisma::RenderPassDesc pass;
        pass.colors[0].texture = viewport.colorTexture;
        pass.colorCount = 1;
        pass.colorLoad = prisma::LoadOp::Load;
        pass.depthLoad = prisma::LoadOp::DontCare;
        pass.depthStore = prisma::StoreOp::Discard;
        driver->beginRenderPass(pass);
        driver->setViewport(area);
        driver->bindPipeline(lensFlarePipeline);
        driver->bindTexture(0, viewport.depthTexture, nearestSampler);
        for (std::uint32_t i = 0; i < count; ++i)
        {
            driver->bindUniformBuffer(0, lensFlareParams, i * stride,
                    static_cast<std::uint32_t>(sizeof(LensFlareParamsGpu)));
            driver->bindTexture(1, flareTextures[i], linearSampler);
            driver->draw(4, 0);
            ++drawCalls;
            ++stats.lensFlaresDrawn;
        }
        driver->endRenderPass();
        if (scope) profiler->endGpu();
    }

    void fillPlanarData(prisma::Driver* driver, const Math::Mat4& view,
            const Math::Mat4& cameraTransform, std::uint32_t count)
    {
        memset(&planarData, 0, sizeof(planarData));
        for (std::uint32_t i = 0; i < count && i < kMaxActiveReflectors; ++i)
        {
            const Reflector* reflector = find(reflectors, activeReflectors[i]);
            if (!reflector) continue;
            const Math::Vec3 normal = view.TransformVector(reflector->planeNormal).Normalized();
            const Math::Vec3 point = view.TransformPoint(reflector->planePoint);
            planarData.plane[i][0] = normal.x;
            planarData.plane[i][1] = normal.y;
            planarData.plane[i][2] = normal.z;
            planarData.plane[i][3] = -normal.Dot(point);
            const Math::Mat4 fromView = reflector->viewProjection * cameraTransform;
            memcpy(planarData.projection[i], &fromView, sizeof(planarData.projection[i]));
            planarData.params[i][0] =
                    reflector->maxDistance > 0.0f ? 1.0f / reflector->maxDistance : 0.0f;
        }
        if (!planarBuffer.valid())
        {
            prisma::BufferDesc desc;
            desc.usage = prisma::BufferUsage::Uniform;
            desc.update = prisma::BufferUpdate::Dynamic;
            desc.size = sizeof(PlanarReflectionsGpu);
            desc.debugName = "planar reflections";
            planarBuffer = driver->createBuffer(desc);
        }
        if (planarBuffer.valid())
            driver->updateBuffer(planarBuffer, 0, &planarData, sizeof(planarData));
    }

    static std::uint32_t glowParamsStride(prisma::Driver* driver)
    {
        return alignUp(static_cast<std::uint32_t>(sizeof(GlowParamsGpu)),
                driver->caps().uniformBufferOffsetAlignment);
    }

    static std::uint32_t levelExtent(std::uint32_t base, std::uint32_t level)
    {
        const std::uint32_t extent = base >> level;
        return extent > 0 ? extent : 1;
    }

    static void destroyGlowTargets(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.glowTexture.valid()) driver->destroy(viewport.glowTexture);
        if (viewport.glowTemp.valid()) driver->destroy(viewport.glowTemp);
        viewport.glowTexture = prisma::TextureHandle();
        viewport.glowTemp = prisma::TextureHandle();
        viewport.glowWidth = 0;
        viewport.glowHeight = 0;
        viewport.glowMips = 0;
    }

    static bool ensureGlowTargets(prisma::Driver* driver, Viewport& viewport, bool& created)
    {
        created = false;
        if (viewport.glowTexture.valid() && viewport.glowTemp.valid()) return true;
        destroyGlowTargets(driver, viewport);

        std::uint32_t longest = viewport.targetWidth > viewport.targetHeight
                                        ? viewport.targetWidth
                                        : viewport.targetHeight;
        std::uint32_t required = 0;
        while (longest > 1)
        {
            longest >>= 1;
            ++required;
        }
        std::uint32_t mips = required > 1 ? required - 1 : 1;
        if (mips > kGlowLevels) mips = kGlowLevels;

        prisma::TextureDesc desc;
        desc.width = viewport.targetWidth > 1 ? viewport.targetWidth >> 1 : 1;
        desc.height = viewport.targetHeight > 1 ? viewport.targetHeight >> 1 : 1;
        desc.mipLevels = mips;
        desc.format = prisma::TextureFormat::RGBA16F;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "glow levels";
        viewport.glowTexture = driver->createTexture(desc);
        desc.debugName = "glow blur temporary";
        viewport.glowTemp = driver->createTexture(desc);
        if (!viewport.glowTexture.valid() || !viewport.glowTemp.valid())
        {
            destroyGlowTargets(driver, viewport);
            log_error("render: could not create the glow targets");
            return false;
        }
        viewport.glowWidth = desc.width;
        viewport.glowHeight = desc.height;
        viewport.glowMips = mips;

        for (std::uint32_t mip = 0; mip < mips; ++mip)
        {
            prisma::RenderPassDesc pass;
            pass.colors[0].texture = viewport.glowTexture;
            pass.colors[0].mip = mip;
            pass.colorCount = 1;
            pass.colorLoad = prisma::LoadOp::Clear;
            pass.clearColor[3] = 0.0f;
            driver->beginRenderPass(pass);
            prisma::Viewport area;
            area.width = static_cast<float>(levelExtent(desc.width, mip));
            area.height = static_cast<float>(levelExtent(desc.height, mip));
            driver->setViewport(area);
            driver->endRenderPass();
        }
        created = true;
        return true;
    }

    bool prepareGlow(prisma::Driver* driver, Viewport& viewport, const EnvironmentSettings& env,
            GlowPass* passes, std::uint32_t& passCount, float* levels, RenderStats& stats)
    {
        passCount = 0;
        if (!ensureGlow(driver)) return false;
        bool created = false;
        if (!ensureGlowTargets(driver, viewport, created)) return false;
        if (created) ++stats.glowTargetsCreated;
        if (!glowRegistered)
        {
            glowRegistered = true;
            if (RenderProfiler* profiler = RenderProfiler::singleton())
                profiler->registerPass("Glow");
        }

        float size = 0.0f;
        for (std::uint32_t i = 0; i < kGlowLevels; ++i) size += env.glowLevels[i];
        for (std::uint32_t i = 0; i < kGlowLevels; ++i)
            levels[i] = env.glowNormalized ? env.glowLevels[i] / size : env.glowLevels[i];

        int maxGlowIndex = -1;
        for (std::uint32_t i = 0; i < kGlowLevels; ++i)
            if (levels[i] > 0.01f) maxGlowIndex = static_cast<int>(i);
        if (maxGlowIndex > static_cast<int>(viewport.glowMips) - 1)
            maxGlowIndex = static_cast<int>(viewport.glowMips) - 1;
        const std::uint32_t levelCount =
                maxGlowIndex < 0 ? 1u : static_cast<std::uint32_t>(maxGlowIndex) + 1u;

        const std::uint32_t stride = glowParamsStride(driver);
        glowBytes.resize(static_cast<std::size_t>(stride) * levelCount * 2);
        memset(glowBytes.data(), 0, glowBytes.size());
        for (std::uint32_t level = 0; level < levelCount; ++level)
        {
            const std::uint32_t width = levelExtent(viewport.glowWidth, level);
            const std::uint32_t height = levelExtent(viewport.glowHeight, level);
            for (std::uint32_t direction = 0; direction < 2; ++direction)
            {
                const bool horizontal = direction == 0;
                GlowPass& pass = passes[passCount];
                pass.target = horizontal ? viewport.glowTemp : viewport.glowTexture;
                pass.source = horizontal
                                      ? (level == 0 ? viewport.colorTexture : viewport.glowTexture)
                                      : viewport.glowTemp;
                pass.mip = level;
                pass.width = width;
                pass.height = height;
                pass.offset = stride * passCount;

                GlowParamsGpu params;
                memset(&params, 0, sizeof(params));
                params.destPixelSize[0] = 1.0f / static_cast<float>(width);
                params.destPixelSize[1] = 1.0f / static_cast<float>(height);
                params.flags = (horizontal ? kGlowFlagHorizontal : 0u) |
                               (level == 0 ? kGlowFlagFirstPass : 0u);
                params.sourceLod = horizontal ? (level == 0 ? 0.0f : static_cast<float>(level - 1))
                                              : static_cast<float>(level);
                params.glowStrength = env.glowStrength;
                params.glowBloom = env.glowBloom;
                params.glowHdrThreshold = env.glowHdrBleedThreshold;
                params.glowHdrScale = env.glowHdrBleedScale;
                params.glowExposure = env.tonemapExposure;
                params.glowLuminanceCap = env.glowHdrLuminanceCap;
                memcpy(glowBytes.data() + pass.offset, &params, sizeof(params));
                ++passCount;
            }
        }
        driver->updateBuffer(glowParams, 0, glowBytes.data(),
                static_cast<std::uint32_t>(glowBytes.size()));
        return true;
    }

    void runGlow(prisma::Driver* driver, const GlowPass* passes, std::uint32_t passCount)
    {
        for (std::uint32_t i = 0; i < passCount; ++i)
        {
            const GlowPass& pass = passes[i];
            prisma::RenderPassDesc desc;
            desc.colors[0].texture = pass.target;
            desc.colors[0].mip = pass.mip;
            desc.colorCount = 1;
            desc.colorLoad = prisma::LoadOp::DontCare;
            driver->beginRenderPass(desc);
            prisma::Viewport area;
            area.width = static_cast<float>(pass.width);
            area.height = static_cast<float>(pass.height);
            driver->setViewport(area);
            driver->bindPipeline(glowPipeline);
            driver->bindUniformBuffer(0, glowParams, pass.offset,
                    static_cast<std::uint32_t>(sizeof(GlowParamsGpu)));
            driver->bindTexture(0, pass.source, glowSampler);
            driver->draw(3, 0);
            driver->endRenderPass();
        }
    }

    bool screenSpaceUsable() const
    {
        return screenSpaceAllowed && screenSpace.supported();
    }

    static void destroyNormalRoughness(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.normalRoughness.valid()) driver->destroy(viewport.normalRoughness);
        viewport.normalRoughness = prisma::TextureHandle();
        viewport.normalRoughnessWidth = 0;
        viewport.normalRoughnessHeight = 0;
    }

    static void destroyScreenSpace(prisma::Driver* driver, Viewport& viewport)
    {
        ScreenSpaceEffects::releaseSsao(driver, viewport.ssao);
        ScreenSpaceEffects::releaseSsr(driver, viewport.ssr);
        destroyNormalRoughness(driver, viewport);
    }

    static void destroyDof(prisma::Driver* driver, Viewport& viewport)
    {
        DepthOfField::release(driver, viewport.dof);
    }

    static bool ensureNormalRoughness(prisma::Driver* driver, Viewport& viewport,
            std::uint32_t width, std::uint32_t height)
    {
        if (viewport.normalRoughness.valid() && viewport.normalRoughnessWidth == width &&
                viewport.normalRoughnessHeight == height)
            return true;
        if (viewport.normalRoughness.valid()) driver->destroy(viewport.normalRoughness);
        prisma::TextureDesc desc;
        desc.width = width;
        desc.height = height;
        desc.format = ScreenSpaceEffects::normalRoughnessFormat();
        desc.usage = ScreenSpaceEffects::normalRoughnessUsage();
        desc.debugName = "viewport normal roughness";
        viewport.normalRoughness = driver->createTexture(desc);
        viewport.normalRoughnessWidth = width;
        viewport.normalRoughnessHeight = height;
        if (!viewport.normalRoughness.valid())
        {
            viewport.normalRoughnessWidth = 0;
            viewport.normalRoughnessHeight = 0;
            log_error("render: could not create the normal and roughness target");
            return false;
        }
        return true;
    }

    static void destroyMsaaTargets(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.colorMsaa.valid()) driver->destroy(viewport.colorMsaa);
        if (viewport.depthMsaa.valid()) driver->destroy(viewport.depthMsaa);
        viewport.colorMsaa = prisma::TextureHandle();
        viewport.depthMsaa = prisma::TextureHandle();
        viewport.samples = 1;
        viewport.samplesWanted = 1;
    }

    static void destroyFsrTargets(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.fsrEasu.valid()) driver->destroy(viewport.fsrEasu);
        if (viewport.fsrOutput.valid()) driver->destroy(viewport.fsrOutput);
        viewport.fsrEasu = prisma::TextureHandle();
        viewport.fsrOutput = prisma::TextureHandle();
        viewport.fsrWidth = 0;
        viewport.fsrHeight = 0;
    }

    static bool ensureFsrTargets(prisma::Driver* driver, Viewport& viewport, std::uint32_t width,
            std::uint32_t height)
    {
        if (viewport.fsrEasu.valid() && viewport.fsrOutput.valid() && viewport.fsrWidth == width &&
                viewport.fsrHeight == height)
            return true;
        destroyFsrTargets(driver, viewport);
        prisma::TextureDesc desc;
        desc.width = width;
        desc.height = height;
        desc.format = prisma::TextureFormat::RGBA16F;
        desc.usage = prisma::kTextureSampled | prisma::kTextureStorage;
        desc.debugName = "viewport fsr easu";
        viewport.fsrEasu = driver->createTexture(desc);
        desc.debugName = "viewport fsr output";
        viewport.fsrOutput = driver->createTexture(desc);
        if (!viewport.fsrEasu.valid() || !viewport.fsrOutput.valid())
        {
            destroyFsrTargets(driver, viewport);
            return false;
        }
        viewport.fsrWidth = width;
        viewport.fsrHeight = height;
        return true;
    }

    // EASU from the picture to the window's rectangle, then RCAS over it, 16x16 pixels per group.
    void runFsr(prisma::Driver* driver, Viewport& viewport, prisma::TextureHandle source,
            std::uint32_t sourceWidth, std::uint32_t sourceHeight)
    {
        FsrParams params = {};
        params.resolutionWidth = static_cast<float>(sourceWidth);
        params.resolutionHeight = static_cast<float>(sourceHeight);
        params.upscaledWidth = static_cast<float>(viewport.fsrWidth);
        params.upscaledHeight = static_cast<float>(viewport.fsrHeight);
        params.sharpness = viewport.fsrSharpness;
        params.pass = 0;
        driver->updateBuffer(fsrParams[0], 0, &params, sizeof(params));
        params.pass = 1;
        driver->updateBuffer(fsrParams[1], 0, &params, sizeof(params));
        const std::uint32_t groupsX = (viewport.fsrWidth + 15) / 16;
        const std::uint32_t groupsY = (viewport.fsrHeight + 15) / 16;
        driver->beginComputePass();
        driver->bindPipeline(fsrPipeline);
        driver->bindUniformBuffer(0, fsrParams[0], 0, sizeof(FsrParams));
        driver->bindTexture(0, source, linearSampler);
        driver->bindStorageTexture(0, viewport.fsrEasu, 0, prisma::StorageAccess::Write);
        driver->dispatch(groupsX, groupsY, 1);
        driver->endComputePass();
        driver->beginComputePass();
        driver->bindPipeline(fsrPipeline);
        driver->bindUniformBuffer(0, fsrParams[1], 0, sizeof(FsrParams));
        driver->bindTexture(0, viewport.fsrEasu, linearSampler);
        driver->bindStorageTexture(0, viewport.fsrOutput, 0, prisma::StorageAccess::Write);
        driver->dispatch(groupsX, groupsY, 1);
        driver->endComputePass();
    }

    static prisma::TextureHandle smaaTable(prisma::Driver* driver, const unsigned char* png,
            unsigned size, int channels, const char* name)
    {
        int width = 0;
        int height = 0;
        int components = 0;
        unsigned char* pixels = stbi_load_from_memory(png, static_cast<int>(size), &width, &height,
                &components, channels);
        if (!pixels) return prisma::TextureHandle();
        prisma::TextureDesc desc;
        desc.width = static_cast<std::uint32_t>(width);
        desc.height = static_cast<std::uint32_t>(height);
        desc.format = channels == 2 ? prisma::TextureFormat::RG8 : prisma::TextureFormat::R8;
        desc.usage = prisma::kTextureSampled;
        desc.data = pixels;
        desc.debugName = name;
        const prisma::TextureHandle texture = driver->createTexture(desc);
        stbi_image_free(pixels);
        return texture;
    }

    void createSmaa(prisma::Driver* driver, const prisma::Caps& caps)
    {
        const ShaderVariant* const sources[6] = { &smaa_edge_detection_vert,
            &smaa_edge_detection_frag, &smaa_weight_calculation_vert, &smaa_weight_calculation_frag,
            &smaa_blending_vert, &smaa_blending_frag };
        bool ok = true;
        for (int i = 0; i < 6; ++i)
        {
            smaaShaders[i] = driver->createShader(prisma3d::shaderDesc(*sources[i], caps));
            ok = ok && smaaShaders[i].valid();
        }
        prisma::BufferDesc bufferDesc;
        bufferDesc.usage = prisma::BufferUsage::Uniform;
        bufferDesc.update = prisma::BufferUpdate::Dynamic;
        const std::uint32_t sizes[3] = { sizeof(SmaaEdgeParams), sizeof(SmaaWeightParams),
            sizeof(SmaaBlendParams) };
        const char* const names[3] = { "smaa edge params", "smaa weight params",
            "smaa blend params" };
        for (int i = 0; i < 3; ++i)
        {
            bufferDesc.size = sizes[i];
            bufferDesc.debugName = names[i];
            smaaParams[i] = driver->createBuffer(bufferDesc);
            ok = ok && smaaParams[i].valid();
        }
        smaaAreaTex = smaaTable(driver, kSmaaAreaTexPng, kSmaaAreaTexPngSize, 2, "smaa area");
        smaaSearchTex =
                smaaTable(driver, kSmaaSearchTexPng, kSmaaSearchTexPngSize, 1, "smaa search");
        ok = ok && smaaAreaTex.valid() && smaaSearchTex.valid();
        if (ok)
        {
            // The edges pass writes 0xff to the stencil where it finds an edge; the weights
            // pass runs only there.
            prisma::PipelineDesc desc;
            desc.vertexShader = smaaShaders[0];
            desc.fragmentShader = smaaShaders[1];
            desc.uniformBlockCount = 1;
            desc.uniformBlocks[0].name = "Params";
            desc.uniformBlocks[0].slot = 0;
            desc.textureCount = 1;
            desc.textures[0].name = "color_tex";
            desc.textures[0].slot = 0;
            desc.targets.window = false;
            desc.targets.colorCount = 1;
            desc.targets.colors[0] = prisma::TextureFormat::RG8;
            desc.targets.depth = prisma::TextureFormat::Depth24Stencil8;
            desc.depthTest = false;
            desc.depthWrite = false;
            desc.cullMode = prisma::CullMode::None;
            desc.stencilTest = true;
            desc.stencilFront.compare = prisma::CompareOp::Always;
            desc.stencilFront.passOp = prisma::StencilOp::Replace;
            desc.stencilBack = desc.stencilFront;
            desc.stencilReadMask = 0xff;
            desc.stencilWriteMask = 0xff;
            desc.debugName = "smaa edge detection";
            smaaEdgePipeline = driver->createPipeline(desc);

            desc.vertexShader = smaaShaders[2];
            desc.fragmentShader = smaaShaders[3];
            desc.textureCount = 3;
            desc.textures[0].name = "edges_tex";
            desc.textures[0].slot = 0;
            desc.textures[1].name = "area_tex";
            desc.textures[1].slot = 1;
            desc.textures[2].name = "search_tex";
            desc.textures[2].slot = 2;
            desc.targets.colors[0] = prisma::TextureFormat::RGBA8;
            desc.stencilFront.compare = prisma::CompareOp::Equal;
            desc.stencilFront.passOp = prisma::StencilOp::Keep;
            desc.stencilBack = desc.stencilFront;
            desc.stencilWriteMask = 0;
            desc.debugName = "smaa weight calculation";
            smaaWeightPipeline = driver->createPipeline(desc);

            desc.vertexShader = smaaShaders[4];
            desc.fragmentShader = smaaShaders[5];
            desc.textureCount = 2;
            desc.textures[0].name = "color_tex";
            desc.textures[0].slot = 0;
            desc.textures[1].name = "blend_tex";
            desc.textures[1].slot = 1;
            desc.targets.depth = prisma::TextureFormat::None;
            desc.stencilTest = false;
            desc.debugName = "smaa blending";
            smaaBlendPipeline = driver->createPipeline(desc);
        }
        if (!smaaEdgePipeline.valid() || !smaaWeightPipeline.valid() || !smaaBlendPipeline.valid())
            log_error("render: could not create the SMAA pipelines (screen_space_aa SMAA is off)");
    }

    bool smaaReady() const
    {
        return smaaEdgePipeline.valid() && smaaWeightPipeline.valid() && smaaBlendPipeline.valid();
    }

    static void destroySmaaTargets(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.smaaEdges.valid()) driver->destroy(viewport.smaaEdges);
        if (viewport.smaaStencil.valid()) driver->destroy(viewport.smaaStencil);
        if (viewport.smaaBlend.valid()) driver->destroy(viewport.smaaBlend);
        if (viewport.smaaOutput.valid()) driver->destroy(viewport.smaaOutput);
        viewport.smaaEdges = prisma::TextureHandle();
        viewport.smaaStencil = prisma::TextureHandle();
        viewport.smaaBlend = prisma::TextureHandle();
        viewport.smaaOutput = prisma::TextureHandle();
        viewport.smaaWidth = 0;
        viewport.smaaHeight = 0;
    }

    static bool ensureSmaaTargets(prisma::Driver* driver, Viewport& viewport, std::uint32_t width,
            std::uint32_t height)
    {
        if (viewport.smaaOutput.valid() && viewport.smaaWidth == width &&
                viewport.smaaHeight == height)
            return true;
        destroySmaaTargets(driver, viewport);
        prisma::TextureDesc desc;
        desc.width = width;
        desc.height = height;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.format = prisma::TextureFormat::RG8;
        desc.debugName = "viewport smaa edges";
        viewport.smaaEdges = driver->createTexture(desc);
        desc.format = prisma::TextureFormat::RGBA8;
        desc.debugName = "viewport smaa blend";
        viewport.smaaBlend = driver->createTexture(desc);
        desc.debugName = "viewport smaa output";
        viewport.smaaOutput = driver->createTexture(desc);
        desc.format = prisma::TextureFormat::Depth24Stencil8;
        desc.usage = prisma::kTextureRenderTarget;
        desc.debugName = "viewport smaa stencil";
        viewport.smaaStencil = driver->createTexture(desc);
        if (!viewport.smaaEdges.valid() || !viewport.smaaBlend.valid() ||
                !viewport.smaaOutput.valid() || !viewport.smaaStencil.valid())
        {
            destroySmaaTargets(driver, viewport);
            return false;
        }
        viewport.smaaWidth = width;
        viewport.smaaHeight = height;
        return true;
    }

    // The three passes over the tonemapped picture: the edges (with the stencil marking them),
    // the blending weights where the stencil is set, and the blend into the output.
    void runSmaa(prisma::Driver* driver, Viewport& viewport, prisma::TextureHandle source,
            std::uint32_t width, std::uint32_t height)
    {
        const float invSize[2] = { 1.0f / static_cast<float>(width),
            1.0f / static_cast<float>(height) };
        SmaaEdgeParams edge = {};
        edge.invSize[0] = invSize[0];
        edge.invSize[1] = invSize[1];
        edge.threshold = smaaEdgeThreshold;
        SmaaWeightParams weight = {};
        weight.invSize[0] = invSize[0];
        weight.invSize[1] = invSize[1];
        weight.size[0] = width;
        weight.size[1] = height;
        SmaaBlendParams blend = {};
        blend.invSize[0] = invSize[0];
        blend.invSize[1] = invSize[1];
        driver->updateBuffer(smaaParams[0], 0, &edge, sizeof(edge));
        driver->updateBuffer(smaaParams[1], 0, &weight, sizeof(weight));
        driver->updateBuffer(smaaParams[2], 0, &blend, sizeof(blend));
        prisma::Viewport area;
        area.width = static_cast<float>(width);
        area.height = static_cast<float>(height);

        prisma::RenderPassDesc edges;
        edges.colors[0].texture = viewport.smaaEdges;
        edges.colorCount = 1;
        edges.depth.texture = viewport.smaaStencil;
        edges.colorLoad = prisma::LoadOp::Clear;
        edges.clearColor[0] = 0.0f;
        edges.clearColor[1] = 0.0f;
        edges.clearColor[2] = 0.0f;
        edges.clearColor[3] = 0.0f;
        edges.depthLoad = prisma::LoadOp::Clear;
        edges.stencilLoad = prisma::LoadOp::Clear;
        edges.clearStencil = 0;
        edges.depthStore = prisma::StoreOp::Store;
        driver->beginRenderPass(edges);
        driver->setViewport(area);
        driver->bindPipeline(smaaEdgePipeline);
        driver->setStencilReference(0xff);
        driver->bindUniformBuffer(0, smaaParams[0], 0, sizeof(SmaaEdgeParams));
        driver->bindTexture(0, source, linearSampler);
        driver->draw(3, 0);
        driver->endRenderPass();

        prisma::RenderPassDesc weights;
        weights.colors[0].texture = viewport.smaaBlend;
        weights.colorCount = 1;
        weights.depth.texture = viewport.smaaStencil;
        weights.colorLoad = prisma::LoadOp::Clear;
        weights.clearColor[0] = 0.0f;
        weights.clearColor[1] = 0.0f;
        weights.clearColor[2] = 0.0f;
        weights.clearColor[3] = 0.0f;
        weights.depthLoad = prisma::LoadOp::Load;
        weights.stencilLoad = prisma::LoadOp::Load;
        weights.depthStore = prisma::StoreOp::Discard;
        driver->beginRenderPass(weights);
        driver->setViewport(area);
        driver->bindPipeline(smaaWeightPipeline);
        driver->setStencilReference(0xff);
        driver->bindUniformBuffer(0, smaaParams[1], 0, sizeof(SmaaWeightParams));
        driver->bindTexture(0, viewport.smaaEdges, linearSampler);
        driver->bindTexture(1, smaaAreaTex, linearSampler);
        driver->bindTexture(2, smaaSearchTex, linearSampler);
        driver->draw(3, 0);
        driver->endRenderPass();

        prisma::RenderPassDesc blending;
        blending.colors[0].texture = viewport.smaaOutput;
        blending.colorCount = 1;
        blending.colorLoad = prisma::LoadOp::DontCare;
        blending.depthLoad = prisma::LoadOp::DontCare;
        blending.depthStore = prisma::StoreOp::Discard;
        driver->beginRenderPass(blending);
        driver->setViewport(area);
        driver->bindPipeline(smaaBlendPipeline);
        driver->bindUniformBuffer(0, smaaParams[2], 0, sizeof(SmaaBlendParams));
        driver->bindTexture(0, source, linearSampler);
        driver->bindTexture(1, viewport.smaaBlend, linearSampler);
        driver->draw(3, 0);
        driver->endRenderPass();
    }

    static void destroyUpscaleSource(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.upscaleSource.valid()) driver->destroy(viewport.upscaleSource);
        viewport.upscaleSource = prisma::TextureHandle();
        viewport.upscaleWidth = 0;
        viewport.upscaleHeight = 0;
        destroyFsrTargets(driver, viewport);
        destroySmaaTargets(driver, viewport);
    }

    static bool ensureUpscaleSource(prisma::Driver* driver, Viewport& viewport, std::uint32_t width,
            std::uint32_t height)
    {
        if (viewport.upscaleSource.valid() && viewport.upscaleWidth == width &&
                viewport.upscaleHeight == height)
            return true;
        destroyUpscaleSource(driver, viewport);
        prisma::TextureDesc desc;
        desc.width = width;
        desc.height = height;
        desc.format = prisma::TextureFormat::RGBA8;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "viewport upscale source";
        viewport.upscaleSource = driver->createTexture(desc);
        if (!viewport.upscaleSource.valid()) return false;
        viewport.upscaleWidth = width;
        viewport.upscaleHeight = height;
        return true;
    }

    static void destroyTargets(prisma::Driver* driver, Viewport& viewport)
    {
        destroyMsaaTargets(driver, viewport);
        destroyUpscaleSource(driver, viewport);
        destroyGlowTargets(driver, viewport);
        destroyScreenSpace(driver, viewport);
        destroyDof(driver, viewport);
        destroyScreenTextures(driver, viewport);
        if (viewport.colorTexture.valid()) driver->destroy(viewport.colorTexture);
        if (viewport.depthTexture.valid()) driver->destroy(viewport.depthTexture);
        if (viewport.cluster)
        {
            viewport.cluster->destroy();
            delete viewport.cluster;
            viewport.cluster = nullptr;
        }
        viewport.colorTexture = prisma::TextureHandle();
        viewport.depthTexture = prisma::TextureHandle();
        viewport.targetWidth = 0;
        viewport.targetHeight = 0;
    }

    static std::uint32_t requestedSamples(const prisma::Driver* driver, ViewportMsaa msaa)
    {
        std::uint32_t samples = 1;
        switch (msaa)
        {
            case ViewportMsaa::Msaa2x:
                samples = 2;
                break;
            case ViewportMsaa::Msaa4x:
                samples = 4;
                break;
            case ViewportMsaa::Msaa8x:
                samples = 8;
                break;
            case ViewportMsaa::Disabled:
                break;
        }
        const std::uint32_t limit = driver->caps().maxSamples;
        while (samples > 1 && samples > limit) samples >>= 1;
        return samples;
    }

    static void ensureMsaaTargets(prisma::Driver* driver, Viewport& viewport, std::uint32_t wanted,
            bool& created)
    {
        if (viewport.samplesWanted == wanted) return;
        destroyMsaaTargets(driver, viewport);
        viewport.samplesWanted = wanted;
        created = true;
        while (wanted > 1)
        {
            prisma::TextureDesc desc;
            desc.width = viewport.targetWidth;
            desc.height = viewport.targetHeight;
            desc.samples = wanted;
            desc.usage = prisma::kTextureRenderTarget;
            desc.format = prisma::TextureFormat::RGBA16F;
            desc.debugName = "viewport color multisampled";
            viewport.colorMsaa = driver->createTexture(desc);
            desc.format = prisma::TextureFormat::Depth32F;
            desc.debugName = "viewport depth multisampled";
            viewport.depthMsaa = driver->createTexture(desc);
            if (viewport.colorMsaa.valid() && viewport.depthMsaa.valid())
            {
                viewport.samples = wanted;
                return;
            }
            if (viewport.colorMsaa.valid()) driver->destroy(viewport.colorMsaa);
            if (viewport.depthMsaa.valid()) driver->destroy(viewport.depthMsaa);
            viewport.colorMsaa = prisma::TextureHandle();
            viewport.depthMsaa = prisma::TextureHandle();
            wanted >>= 1;
        }
        viewport.samples = 1;
    }

    static void ensureColorStorage(prisma::Driver* driver, Viewport& viewport)
    {
        if (viewport.colorStorage == viewport.colorStorageWanted) return;
        prisma::TextureDesc desc;
        desc.width = viewport.targetWidth;
        desc.height = viewport.targetHeight;
        desc.format = prisma::TextureFormat::RGBA16F;
        desc.usage = DepthOfField::colorUsage(viewport.colorStorageWanted);
        desc.debugName = "viewport color";
        const prisma::TextureHandle texture = driver->createTexture(desc);
        if (!texture.valid())
        {
            log_error("render: could not recreate the viewport color target");
            return;
        }
        driver->destroy(viewport.colorTexture);
        viewport.colorTexture = texture;
        viewport.colorStorage = viewport.colorStorageWanted;
    }

    static bool ensureTargets(prisma::Driver* driver, Viewport& viewport, std::uint32_t width,
            std::uint32_t height, bool& created)
    {
        created = false;
        const std::uint32_t wanted = requestedSamples(driver, viewport.msaa);
        if (viewport.colorTexture.valid() && viewport.depthTexture.valid() &&
                viewport.targetWidth == width && viewport.targetHeight == height)
        {
            ensureColorStorage(driver, viewport);
            ensureMsaaTargets(driver, viewport, wanted, created);
            return true;
        }
        destroyTargets(driver, viewport);
        prisma::TextureDesc desc;
        desc.width = width;
        desc.height = height;
        desc.format = prisma::TextureFormat::RGBA16F;
        desc.usage = DepthOfField::colorUsage(viewport.colorStorageWanted);
        desc.debugName = "viewport color";
        viewport.colorTexture = driver->createTexture(desc);
        viewport.colorStorage = viewport.colorStorageWanted;
        desc.format = prisma::TextureFormat::Depth32F;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "viewport depth";
        viewport.depthTexture = driver->createTexture(desc);
        if (!viewport.colorTexture.valid() || !viewport.depthTexture.valid())
        {
            destroyTargets(driver, viewport);
            log_error("render: could not create the viewport targets");
            return false;
        }
        viewport.targetWidth = width;
        viewport.targetHeight = height;
        created = true;
        ensureMsaaTargets(driver, viewport, wanted, created);
        return true;
    }

    static void destroyOutput(TextureStore& textures, Viewport& viewport)
    {
        if (viewport.output.valid()) textures.destroy(viewport.output);
        viewport.output = TextureId();
        viewport.outputWidth = 0;
        viewport.outputHeight = 0;
    }

    static bool outputIsLinear(const Viewport& viewport)
    {
        return viewport.debugDraw == ViewportDebugDraw::NormalBuffer || viewport.srgbOutput;
    }

    static bool ensureOutput(TextureStore& textures, Viewport& viewport, std::uint32_t width,
            std::uint32_t height, bool linear)
    {
        if (viewport.output.valid() && textures.valid(viewport.output) &&
                viewport.outputWidth == width && viewport.outputHeight == height &&
                viewport.outputLinear == linear)
            return true;
        destroyOutput(textures, viewport);
        prisma::TextureDesc desc;
        desc.width = width;
        desc.height = height;
        desc.format = linear ? prisma::TextureFormat::RGBA8 : prisma::TextureFormat::RGBA8Srgb;
        desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        desc.debugName = "viewport texture";
        viewport.output = textures.create(desc, !linear);
        if (!viewport.output.valid())
        {
            log_error("render: could not create the viewport texture");
            return false;
        }
        viewport.outputWidth = width;
        viewport.outputHeight = height;
        viewport.outputLinear = linear;
        return true;
    }
};

RenderingServer* RenderingServer::sSingleton = nullptr;

RenderingServer::RenderingServer(prisma::Driver* driver, MeshStore& meshes, TextureStore& textures,
        MaterialStore& materials)
        : mDriver(driver),
          mMeshes(meshes),
          mTextures(textures),
          mMaterials(materials),
          mState(new State()),
          mGpuFrameScope(false),
          mFrameOpen(false)
{
    sSingleton = this;
    mState->multimeshes.setup(driver, &meshes);
    if (RenderProfiler* profiler = RenderProfiler::singleton())
    {
        profiler->registerPass("Opaque");
        profiler->registerPass("Sky");
        profiler->registerPass("Final");
    }
    const prisma::Caps& caps = driver->caps();
    prisma::SamplerDesc materialSamplerDesc;
    materialSamplerDesc.minFilter = prisma::Filter::Linear;
    materialSamplerDesc.magFilter = prisma::Filter::Linear;
    materialSamplerDesc.mipFilter = prisma::MipFilter::Linear;
    materialSamplerDesc.maxAnisotropy = 16.0f;
    materialSamplerDesc.addressU = prisma::AddressMode::Repeat;
    materialSamplerDesc.addressV = prisma::AddressMode::Repeat;
    materialSamplerDesc.addressW = prisma::AddressMode::Repeat;
    materialSamplerDesc.debugName = "material";
    mState->materialSampler = driver->createSampler(materialSamplerDesc);
    if (!mState->decalAtlas.create(driver, &textures))
        log_error("render: could not create the decal atlas");
    {
        prisma::SamplerDesc decalSamplerDesc;
        decalSamplerDesc.minFilter = prisma::Filter::Linear;
        decalSamplerDesc.magFilter = prisma::Filter::Linear;
        decalSamplerDesc.mipFilter = prisma::MipFilter::Linear;
        decalSamplerDesc.addressU = prisma::AddressMode::ClampToEdge;
        decalSamplerDesc.addressV = prisma::AddressMode::ClampToEdge;
        decalSamplerDesc.addressW = prisma::AddressMode::ClampToEdge;
        decalSamplerDesc.debugName = "decal";
        mState->decalSampler = driver->createSampler(decalSamplerDesc);

        prisma::BufferDesc implementationDesc;
        implementationDesc.usage = prisma::BufferUsage::Uniform;
        implementationDesc.update = prisma::BufferUpdate::Dynamic;
        implementationDesc.size = sizeof(ImplementationDataGpu);
        implementationDesc.debugName = "implementation data";
        mState->implementationBuffer = driver->createBuffer(implementationDesc);

        if (!mState->clusterShared.create(driver))
            log_error("render: could not create the light cluster builder");
    }
    if (!mState->materialSampler.valid())
        log_error("render: could not create the material sampler");
    mState->applyShadowQuality(ShadowQuality::SoftLow);
    mState->applyPositionalShadowQuality(ShadowQuality::SoftLow);
    BatchRenderer::Config batchConfig;
    batchConfig.maxVertices = 65536;
    batchConfig.uniformSlots = 32;
    if (!mState->batch.init(driver, batchConfig)) log_error("render: the batch is not available");
    mState->createTonemap(driver);
    mState->screenSpace.create(driver);
    mState->dof.create(driver);
    mState->createSceneBuffer(driver);
    if (caps.compute)
    {
        if (RenderProfiler* profiler = RenderProfiler::singleton())
            profiler->registerPass("Skinning");
        mState->skinShader = driver->createShader(prisma3d::shaderDesc(skeleton_comp, caps));
        prisma::ComputePipelineDesc desc;
        desc.shader = mState->skinShader;
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "Params";
        desc.uniformBlocks[0].slot = 0;
        desc.storageBufferCount = 3;
        desc.storageBuffers[0].name = "SrcVertexData";
        desc.storageBuffers[0].slot = 0;
        desc.storageBuffers[1].name = "DstVertexData";
        desc.storageBuffers[1].slot = 1;
        desc.storageBuffers[2].name = "SkeletonData";
        desc.storageBuffers[2].slot = 2;
        desc.debugName = "skinning";
        if (mState->skinShader.valid()) mState->skinPipeline = driver->createComputePipeline(desc);
        mState->skinningSupported = mState->skinPipeline.valid();
    }
    if (!mState->skinningSupported) log_error("render: skinning is not available on this driver");
    if (RenderProfiler* profiler = RenderProfiler::singleton()) profiler->registerPass("Radiance");
    if (RenderProfiler* profiler = RenderProfiler::singleton()) profiler->registerPass("Shadows");
    if (RenderProfiler* profiler = RenderProfiler::singleton()) profiler->registerPass("Depth");
    if (RenderProfiler* profiler = RenderProfiler::singleton()) profiler->registerPass("Impostor");
}

RenderingServer::~RenderingServer()
{
    mState->pool.wait_all();
    for (auto& entry: mState->pipelines) mDriver->destroy(entry.value);
    for (Viewport& viewport: mState->viewports)
    {
        State::destroyTargets(mDriver, viewport);
        State::destroyPositionalAtlas(mDriver, viewport);
        State::destroyOutput(mTextures, viewport);
    }
    mState->destroyTonemap(mDriver);
    mState->screenSpace.destroy();
    mState->dof.destroy();
    for (std::uint32_t i = 0; i < 4; ++i)
    {
        if (mState->normalVertexShaders[i].valid())
            mDriver->destroy(mState->normalVertexShaders[i]);
        for (std::uint32_t t = 0; t < 2; ++t)
            if (mState->normalMultiMeshVertexShaders[t][i].valid())
                mDriver->destroy(mState->normalMultiMeshVertexShaders[t][i]);
    }
    for (std::uint32_t i = 0; i < 128; ++i)
        if (mState->normalFragmentShaders[i].valid())
            mDriver->destroy(mState->normalFragmentShaders[i]);
    for (Instance& instance: mState->instances) mState->freeMeshInstance(mDriver, instance);
    for (Skeleton& skeleton: mState->skeletons)
        if (skeleton.buffer.valid()) mDriver->destroy(skeleton.buffer);
    mState->multimeshes.destroyAll();
    mState->impostors.destroy(mDriver);
    for (std::uint32_t i = 0; i < 4; ++i)
    {
        for (std::uint32_t t = 0; t < 3; ++t)
            if (mState->sceneMultiMeshVertexShaders[t][i].valid())
                mDriver->destroy(mState->sceneMultiMeshVertexShaders[t][i]);
        if (mState->shadowMultiMeshVertexShaders[i].valid())
            mDriver->destroy(mState->shadowMultiMeshVertexShaders[i]);
        if (mState->shadowDualMultiMeshVertexShaders[i].valid())
            mDriver->destroy(mState->shadowDualMultiMeshVertexShaders[i]);
    }
    if (mState->sceneOceanVertexShader.valid()) mDriver->destroy(mState->sceneOceanVertexShader);
    if (mState->sceneOceanFragmentShader.valid())
        mDriver->destroy(mState->sceneOceanFragmentShader);
    for (std::uint32_t type = 0; type < 3; ++type)
        if (mState->sceneTerrainVertexShaders[type].valid())
            mDriver->destroy(mState->sceneTerrainVertexShaders[type]);
    for (std::uint32_t dual = 0; dual < 2; ++dual)
        if (mState->shadowTerrainVertexShaders[dual].valid())
            mDriver->destroy(mState->shadowTerrainVertexShaders[dual]);
    for (std::uint32_t dual = 0; dual < 2; ++dual)
        if (mState->depthGrassVertexShaders[dual].valid())
            mDriver->destroy(mState->depthGrassVertexShaders[dual]);
    for (std::uint32_t mm = 0; mm < 5; ++mm)
        if (mState->depthLeafWindVertexShaders[mm].valid())
            mDriver->destroy(mState->depthLeafWindVertexShaders[mm]);
    for (std::uint32_t i = 0; i < kSceneVariantCount; ++i)
    {
        if (mState->sceneNoShadowFragmentShaders[i].valid())
            mDriver->destroy(mState->sceneNoShadowFragmentShaders[i]);
        if (mState->sceneCoverageFragmentShaders[i].valid())
            mDriver->destroy(mState->sceneCoverageFragmentShaders[i]);
        if (mState->sceneCoverageNoShadowFragmentShaders[i].valid())
            mDriver->destroy(mState->sceneCoverageNoShadowFragmentShaders[i]);
    }
    for (std::uint32_t albedo = 0; albedo < 2; ++albedo)
        if (mState->depthCoverageFragmentShaders[albedo].valid())
            mDriver->destroy(mState->depthCoverageFragmentShaders[albedo]);
    for (auto& entry: mState->skyMsaaPipelines) mDriver->destroy(entry.value);
    if (mState->prepassSceneBuffer.valid()) mDriver->destroy(mState->prepassSceneBuffer);
    for (std::uint32_t type = 0; type < 2; ++type)
        if (mState->sceneTerrainFragmentShaders[type].valid())
            mDriver->destroy(mState->sceneTerrainFragmentShaders[type]);
    for (std::uint32_t mode = 0; mode < 3; ++mode)
    {
        for (std::uint32_t type = 0; type < 3; ++type)
            for (std::uint32_t mm = 0; mm < 5; ++mm)
                if (mState->sceneBillboardVertexShaders[mode][type][mm].valid())
                    mDriver->destroy(mState->sceneBillboardVertexShaders[mode][type][mm]);
    }
    for (std::uint32_t dual = 0; dual < 2; ++dual)
    {
        for (std::uint32_t mm = 0; mm < 5; ++mm)
            if (mState->depthVertexShaders[dual][mm].valid())
                mDriver->destroy(mState->depthVertexShaders[dual][mm]);
        for (std::uint32_t albedo = 0; albedo < 2; ++albedo)
            if (mState->depthFragmentShaders[dual][albedo].valid())
                mDriver->destroy(mState->depthFragmentShaders[dual][albedo]);
    }
    mState->batch.shutdown();
    for (auto& entry: mState->shadowPipelines) mDriver->destroy(entry.value);
    if (mState->shadowVertexShader.valid()) mDriver->destroy(mState->shadowVertexShader);
    if (mState->shadowFragmentShader.valid()) mDriver->destroy(mState->shadowFragmentShader);
    for (auto& entry: mState->shadowCubes)
        if (entry.value.valid()) mDriver->destroy(entry.value);
    if (mState->cubeCopyPipeline.valid()) mDriver->destroy(mState->cubeCopyPipeline);
    if (mState->cubeCopyFragmentShader.valid()) mDriver->destroy(mState->cubeCopyFragmentShader);
    if (mState->shadowCopyBuffer.valid()) mDriver->destroy(mState->shadowCopyBuffer);
    if (mState->shadowDualVertexShader.valid()) mDriver->destroy(mState->shadowDualVertexShader);
    if (mState->shadowDualFragmentShader.valid())
        mDriver->destroy(mState->shadowDualFragmentShader);
    if (mState->positionalShadowSampler.valid()) mDriver->destroy(mState->positionalShadowSampler);
    if (mState->positionalShadowDepthSampler.valid())
        mDriver->destroy(mState->positionalShadowDepthSampler);
    if (mState->shadowSampler.valid()) mDriver->destroy(mState->shadowSampler);
    if (mState->shadowDepthSampler.valid()) mDriver->destroy(mState->shadowDepthSampler);
    if (mState->dummyShadow.valid()) mDriver->destroy(mState->dummyShadow);
    if (mState->shadowAtlas.valid()) mDriver->destroy(mState->shadowAtlas);
    if (mState->shadowSceneBuffer.valid()) mDriver->destroy(mState->shadowSceneBuffer);
    if (mState->shadowInstanceBuffer.valid()) mDriver->destroy(mState->shadowInstanceBuffer);
    if (mState->shadowDrawBuffer.valid()) mDriver->destroy(mState->shadowDrawBuffer);
    if (mState->shadowMaterialBuffer.valid()) mDriver->destroy(mState->shadowMaterialBuffer);
    for (Scenario& scenario: mState->scenarios) State::freeRadiance(mDriver, scenario.radiance);
    for (std::uint32_t i = 0; i < 2; ++i)
    {
        if (mState->octmapDownsamplePipelines[i].valid())
            mDriver->destroy(mState->octmapDownsamplePipelines[i]);
        if (mState->octmapDownsampleShaders[i].valid())
            mDriver->destroy(mState->octmapDownsampleShaders[i]);
    }
    if (mState->octmapRoughnessPipeline.valid()) mDriver->destroy(mState->octmapRoughnessPipeline);
    if (mState->octmapRoughnessShader.valid()) mDriver->destroy(mState->octmapRoughnessShader);
    if (mState->octmapVertexShader.valid()) mDriver->destroy(mState->octmapVertexShader);
    if (mState->octmapParamsBuffer.valid()) mDriver->destroy(mState->octmapParamsBuffer);
    if (mState->mipSampler.valid()) mDriver->destroy(mState->mipSampler);
    if (mState->skySampler.valid()) mDriver->destroy(mState->skySampler);
    if (mState->dummyRadiance.valid()) mDriver->destroy(mState->dummyRadiance);
    if (mState->skyCubemapParamsBuffer.valid()) mDriver->destroy(mState->skyCubemapParamsBuffer);
    for (std::uint32_t i = 0; i < kSkyVariantCount; ++i)
    {
        if (mState->skyPipelines[i].valid()) mDriver->destroy(mState->skyPipelines[i]);
        if (mState->skyFragmentShaders[i].valid()) mDriver->destroy(mState->skyFragmentShaders[i]);
    }
    if (mState->skyVertexShader.valid()) mDriver->destroy(mState->skyVertexShader);
    if (mState->skyParamsBuffer.valid()) mDriver->destroy(mState->skyParamsBuffer);
    if (mState->skyLightsBuffer.valid()) mDriver->destroy(mState->skyLightsBuffer);
    if (mState->skyMaterialBuffer.valid()) mDriver->destroy(mState->skyMaterialBuffer);
    if (mState->skySceneBuffer.valid()) mDriver->destroy(mState->skySceneBuffer);
    if (mState->skyFlatParamsBuffer.valid()) mDriver->destroy(mState->skyFlatParamsBuffer);
    if (mState->skyFlatColorBuffer.valid()) mDriver->destroy(mState->skyFlatColorBuffer);
    if (mState->dfgPipeline.valid()) mDriver->destroy(mState->dfgPipeline);
    if (mState->dfgShader.valid()) mDriver->destroy(mState->dfgShader);
    if (mState->dfgTexture.valid()) mDriver->destroy(mState->dfgTexture);
    if (mState->skinPipeline.valid()) mDriver->destroy(mState->skinPipeline);
    if (mState->skinShader.valid()) mDriver->destroy(mState->skinShader);
    if (mState->skinParamsBuffer.valid()) mDriver->destroy(mState->skinParamsBuffer);
    if (mState->uniformBuffer.valid()) mDriver->destroy(mState->uniformBuffer);
    if (mState->sceneBuffer.valid()) mDriver->destroy(mState->sceneBuffer);
    if (mState->instanceBuffer.valid()) mDriver->destroy(mState->instanceBuffer);
    mState->decalAtlas.destroy();
    mState->clusterShared.destroy();
    if (mState->decalSampler.valid()) mDriver->destroy(mState->decalSampler);
    if (mState->implementationBuffer.valid()) mDriver->destroy(mState->implementationBuffer);
    if (mState->decalBuffer.valid()) mDriver->destroy(mState->decalBuffer);
    if (mState->reflectionBuffer.valid()) mDriver->destroy(mState->reflectionBuffer);
    if (mState->lightmapBuffer.valid()) mDriver->destroy(mState->lightmapBuffer);
    if (mState->lightmapCaptureBuffer.valid()) mDriver->destroy(mState->lightmapCaptureBuffer);
    if (mState->omniLightBuffer.valid()) mDriver->destroy(mState->omniLightBuffer);
    if (mState->spotLightBuffer.valid()) mDriver->destroy(mState->spotLightBuffer);
    if (mState->directionalLightBuffer.valid()) mDriver->destroy(mState->directionalLightBuffer);
    if (mState->lightCountsBuffer.valid()) mDriver->destroy(mState->lightCountsBuffer);
    for (std::uint32_t i = 0; i < kSceneVariantCount; ++i)
    {
        if (mState->sceneVertexShaders[i].valid()) mDriver->destroy(mState->sceneVertexShaders[i]);
        if (mState->sceneFragmentShaders[i].valid())
            mDriver->destroy(mState->sceneFragmentShaders[i]);
    }
    if (mState->materialBuffer.valid()) mDriver->destroy(mState->materialBuffer);
    if (mState->materialSampler.valid()) mDriver->destroy(mState->materialSampler);
    delete mState;
    if (sSingleton == this) sSingleton = nullptr;
}

RenderingServer* RenderingServer::singleton()
{
    return sSingleton;
}

prisma::Driver* RenderingServer::driver() const
{
    return mDriver;
}

MeshStore& RenderingServer::meshes()
{
    return mMeshes;
}

TextureStore& RenderingServer::textures()
{
    return mTextures;
}

MaterialStore& RenderingServer::materials()
{
    return mMaterials;
}

BatchRenderer& RenderingServer::batch()
{
    return mState->batch;
}

ScenarioId RenderingServer::scenarioCreate()
{
    return toId<ScenarioId>(mState->scenarios.insert(Scenario()));
}

void RenderingServer::scenarioFree(ScenarioId scenario)
{
    if (Scenario* item = find(mState->scenarios, scenario))
    {
        mState->waitForBuild(*item);
        State::freeRadiance(mDriver, item->radiance);
    }
    erase(mState->scenarios, scenario);
}

void RenderingServer::scenarioSetEnvironment(ScenarioId scenario,
        const EnvironmentSettings& environment)
{
    Scenario* item = find(mState->scenarios, scenario);
    if (!item) return;
    item->environment = environment;
    item->hasEnvironment = true;
}

void RenderingServer::scenarioClearEnvironment(ScenarioId scenario)
{
    if (Scenario* item = find(mState->scenarios, scenario)) item->hasEnvironment = false;
}

LightId RenderingServer::lightCreate(LightType type)
{
    Light light;
    light.type = type;
    light.serial = ++mState->lightSerial;
    for (std::size_t i = 0; i < static_cast<std::size_t>(LightParam::Count); ++i)
        light.params[i] = kLightParamDefaults[i];
    return toId<LightId>(mState->lights.insert(light));
}

void RenderingServer::lightSetScenario(LightId light, ScenarioId scenario)
{
    if (Light* item = find(mState->lights, light)) item->scenario = scenario;
}

void RenderingServer::lightSetTransform(LightId light, const Math::Mat4& transform)
{
    if (Light* item = find(mState->lights, light)) item->transform = transform;
}

void RenderingServer::lightSetColor(LightId light, const Math::Vec3& color)
{
    if (Light* item = find(mState->lights, light)) item->color = color;
}

void RenderingServer::lightSetParam(LightId light, LightParam param, float value)
{
    Light* item = find(mState->lights, light);
    if (!item || param >= LightParam::Count) return;
    item->params[static_cast<std::size_t>(param)] = value;
}

void RenderingServer::lightSetCullMask(LightId light, std::uint32_t mask)
{
    if (Light* item = find(mState->lights, light)) item->cullMask = mask;
}

void RenderingServer::lightSetNegative(LightId light, bool negative)
{
    if (Light* item = find(mState->lights, light)) item->negative = negative;
}

void RenderingServer::lightSetVisible(LightId light, bool visible)
{
    if (Light* item = find(mState->lights, light)) item->visible = visible;
}

void RenderingServer::lightSetLensFlare(LightId light, const TextureId* textures,
        std::uint32_t count)
{
    Light* item = find(mState->lights, light);
    if (!item) return;
    item->lensFlareCount = 0;
    for (std::uint32_t i = 0; i < count && item->lensFlareCount < kMaxLensFlareElements; ++i)
        if (textures[i].valid()) item->lensFlare[item->lensFlareCount++] = textures[i];
}

void RenderingServer::lightSetBakeMode(LightId light, LightBakeMode mode)
{
    if (Light* item = find(mState->lights, light)) item->bakeMode = mode;
}

LightmapId RenderingServer::lightmapCreate()
{
    return toId<LightmapId>(mState->lightmaps.insert(Lightmap()));
}

void RenderingServer::lightmapSetScenario(LightmapId lightmap, ScenarioId scenario)
{
    if (Lightmap* item = find(mState->lightmaps, lightmap)) item->scenario = scenario;
}

void RenderingServer::lightmapSetTransform(LightmapId lightmap, const Math::Mat4& transform)
{
    if (Lightmap* item = find(mState->lightmaps, lightmap)) item->transform = transform;
}

void RenderingServer::lightmapSetVisible(LightmapId lightmap, bool visible)
{
    if (Lightmap* item = find(mState->lightmaps, lightmap)) item->visible = visible;
}

void RenderingServer::lightmapSetTextures(LightmapId lightmap, TextureId light,
        bool usesSphericalHarmonics)
{
    Lightmap* item = find(mState->lightmaps, lightmap);
    if (!item) return;
    item->light = light;
    item->usesSphericalHarmonics = usesSphericalHarmonics;
}

void RenderingServer::lightmapSetShadowmaskTextures(LightmapId lightmap, TextureId shadowmask)
{
    if (Lightmap* item = find(mState->lightmaps, lightmap)) item->shadowmask = shadowmask;
}

void RenderingServer::lightmapSetShadowmaskMode(LightmapId lightmap, ShadowmaskMode mode)
{
    if (Lightmap* item = find(mState->lightmaps, lightmap)) item->shadowmaskMode = mode;
}

void RenderingServer::lightmapSetSpecularIntensity(LightmapId lightmap, float intensity)
{
    if (Lightmap* item = find(mState->lightmaps, lightmap)) item->specularIntensity = intensity;
}

void RenderingServer::lightmapSetBakedExposureNormalization(LightmapId lightmap, float exposure)
{
    if (Lightmap* item = find(mState->lightmaps, lightmap)) item->bakedExposure = exposure;
}

void RenderingServer::lightmapSetProbeGrid(LightmapId lightmap, const Math::Vec3& origin,
        const Math::Vec3& cell, const std::uint32_t dims[3], const float* sh)
{
    Lightmap* item = find(mState->lightmaps, lightmap);
    if (!item) return;
    item->probeSh.clear();
    item->probeDims[0] = item->probeDims[1] = item->probeDims[2] = 0;
    const std::size_t cells = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
    if (cells == 0 || !sh || cell.x <= 0.0f || cell.y <= 0.0f || cell.z <= 0.0f) return;
    item->probeOrigin = origin;
    item->probeCell = cell;
    item->probeDims[0] = dims[0];
    item->probeDims[1] = dims[1];
    item->probeDims[2] = dims[2];
    item->probeSh.resize(cells * 27);
    memcpy(item->probeSh.data(), sh, cells * 27 * sizeof(float));
}

void RenderingServer::lightmapSetBounds(LightmapId lightmap, const Math::Box& bounds)
{
    Lightmap* item = find(mState->lightmaps, lightmap);
    if (!item) return;
    item->bounds = bounds;
    const Math::Vec3 size = bounds.Size();
    item->hasBounds = size.x > 0.0f && size.y > 0.0f && size.z > 0.0f;
}

void RenderingServer::lightmapFree(LightmapId lightmap)
{
    if (!find(mState->lightmaps, lightmap)) return;
    for (Instance& instance: mState->instances)
        if (instance.lightmap == lightmap) instance.lightmap = LightmapId();
    erase(mState->lightmaps, lightmap);
}

void RenderingServer::instanceGeometrySetTransparency(InstanceId instance, float transparency)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    item->transparency = transparency < 0.0f ? 0.0f : (transparency > 1.0f ? 1.0f : transparency);
}

void RenderingServer::instanceGeometrySetLightmap(InstanceId instance, LightmapId lightmap,
        const Math::Vec4& uvScale, int sliceIndex)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    item->lightmap = lightmap;
    item->lightmapUvScale = uvScale;
    item->lightmapSlice = sliceIndex;
}

void RenderingServer::setLightmapBicubicFilter(bool enabled)
{
    mState->lightmapBicubic = enabled;
}

bool RenderingServer::getLightmapBicubicFilter() const
{
    return mState->lightmapBicubic;
}

void RenderingServer::cameraSetCustomProjection(CameraId camera, const Math::Mat4& projection)
{
    if (Camera* item = find(mState->cameras, camera))
    {
        item->customProjection = true;
        item->customMatrix = projection;
    }
}

void RenderingServer::cameraClearCustomProjection(CameraId camera)
{
    if (Camera* item = find(mState->cameras, camera)) item->customProjection = false;
}

ReflectorId RenderingServer::reflectorCreate(ScenarioId scenario)
{
    Reflector reflector;
    reflector.scenario = scenario;
    return toId<ReflectorId>(mState->reflectors.insert(reflector));
}

void RenderingServer::reflectorSetScenario(ReflectorId reflector, ScenarioId scenario)
{
    if (Reflector* item = find(mState->reflectors, reflector)) item->scenario = scenario;
}

void RenderingServer::reflectorSetTransform(ReflectorId reflector, const Math::Mat4& transform)
{
    if (Reflector* item = find(mState->reflectors, reflector)) item->transform = transform;
}

void RenderingServer::reflectorSetHalfSize(ReflectorId reflector, const Math::Vec2& halfSize)
{
    if (Reflector* item = find(mState->reflectors, reflector)) item->halfSize = halfSize;
}

void RenderingServer::reflectorSetResolution(ReflectorId reflector, std::uint32_t resolution)
{
    if (Reflector* item = find(mState->reflectors, reflector))
        item->resolution = resolution < 16 ? 16 : resolution;
}

void RenderingServer::reflectorSetMaxDistance(ReflectorId reflector, float distance)
{
    if (Reflector* item = find(mState->reflectors, reflector)) item->maxDistance = distance;
}

void RenderingServer::reflectorSetVisible(ReflectorId reflector, bool visible)
{
    if (Reflector* item = find(mState->reflectors, reflector)) item->visible = visible;
}

TextureId RenderingServer::reflectorGetTexture(ReflectorId reflector) const
{
    const Reflector* item = find(mState->reflectors, reflector);
    if (!item) return TextureId();
    const Viewport* viewport = find(mState->viewports, item->viewport);
    return viewport ? viewport->output : TextureId();
}

void RenderingServer::reflectorFree(ReflectorId reflector)
{
    Reflector* item = find(mState->reflectors, reflector);
    if (!item) return;
    const ViewportId viewport = item->viewport;
    const CameraId camera = item->camera;
    if (viewport.valid()) viewportFree(viewport);
    if (camera.valid()) cameraFree(camera);
    for (Instance& instance: mState->instances)
        if (instance.reflector == reflector) instance.reflector = ReflectorId();
    erase(mState->reflectors, reflector);
}

void RenderingServer::instanceSetReflector(InstanceId instance, ReflectorId reflector)
{
    if (Instance* item = find(mState->instances, instance)) item->reflector = reflector;
}

namespace
{

Math::Vec3 reflectPoint(const Math::Vec3& point, const Math::Vec3& normal, float d)
{
    return point - normal * (2.0f * (normal.Dot(point) + d));
}

Math::Vec3 reflectVector(const Math::Vec3& vector, const Math::Vec3& normal)
{
    return vector - normal * (2.0f * normal.Dot(vector));
}

float signOf(float value)
{
    return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f);
}

Math::Mat4 obliqueProjection(const Math::Mat4& projection, const Math::Vec3& normalView,
        float dView)
{
    Math::Mat4 result = projection;
    const float qx = (signOf(normalView.x) + projection.col2.x) / projection.col0.x;
    const float qy = (signOf(normalView.y) + projection.col2.y) / projection.col1.y;
    const float qz = -1.0f;
    const float qw = (1.0f + projection.col2.z) / projection.col3.z;
    const float dot = normalView.x * qx + normalView.y * qy + normalView.z * qz + dView * qw;
    const float scale = 2.0f / dot;
    result.col0.z = normalView.x * scale;
    result.col1.z = normalView.y * scale;
    result.col2.z = normalView.z * scale + 1.0f;
    result.col3.z = dView * scale;
    return result;
}

} // namespace

void RenderingServer::renderReflectors(Viewport& viewport)
{
    State& state = *mState;
    state.activeReflectorCount = 0;
    if (viewport.reflection || viewport.width == 0 || viewport.height == 0) return;
    const Camera* camera = find(state.cameras, viewport.camera);
    if (!camera) return;

    struct Candidate
    {
        ReflectorId id;
        float distance;
    };
    Candidate candidates[kMaxActiveReflectors];
    std::uint32_t candidateCount = 0;

    const float aspect = static_cast<float>(viewport.width) / static_cast<float>(viewport.height);
    const Math::Mat4 projection =
            camera->projection == CameraProjection::Perspective
                    ? basePerspective(camera->fovy, aspect, camera->zNear, camera->zFar)
                    : baseOrthogonal(camera->size, aspect, camera->zNear, camera->zFar);
    const Math::Mat4 view = camera->transform.InverseAffine();
    const Math::Frustum frustum = Math::Frustum::FromViewProjection(projection * view);
    const Math::Vec3 eye = camera->transform.GetTranslation();

    for (std::size_t index = 0; index < state.reflectors.size(); ++index)
    {
        const Reflector& reflector = state.reflectors.begin()[index];
        if (!reflector.visible || reflector.scenario != viewport.scenario) continue;
        Math::Box box;
        for (int corner = 0; corner < 4; ++corner)
        {
            const Math::Vec3 local((corner & 1) ? reflector.halfSize.x : -reflector.halfSize.x,
                    (corner & 2) ? reflector.halfSize.y : -reflector.halfSize.y, 0.0f);
            const Math::Vec3 world = reflector.transform.TransformPoint(local);
            if (corner == 0)
            {
                box.min = world;
                box.max = world;
            }
            else
            {
                box.min = Math::Vec3::Min(box.min, world);
                box.max = Math::Vec3::Max(box.max, world);
            }
        }
        if (!frustum.IntersectsBox(box)) continue;
        const float distance = (reflector.transform.GetTranslation() - eye).Length();
        const ReflectorId id = toId<ReflectorId>(state.reflectors.handle_at(index));
        std::uint32_t slot = candidateCount;
        if (candidateCount == kMaxActiveReflectors)
        {
            std::uint32_t farthest = 0;
            for (std::uint32_t i = 1; i < kMaxActiveReflectors; ++i)
                if (candidates[i].distance > candidates[farthest].distance) farthest = i;
            if (candidates[farthest].distance <= distance) continue;
            slot = farthest;
        }
        else
        {
            ++candidateCount;
        }
        candidates[slot].id = id;
        candidates[slot].distance = distance;
    }

    for (std::uint32_t c = 0; c < candidateCount; ++c)
    {
        Reflector* reflector = find(state.reflectors, candidates[c].id);
        if (!reflector) continue;
        if (!reflector->camera.valid()) reflector->camera = cameraCreate();
        if (!reflector->viewport.valid())
        {
            reflector->viewport = viewportCreate();
            viewportAttachCamera(reflector->viewport, reflector->camera);
            viewportSetUpdateMode(reflector->viewport, ViewportUpdateMode::Disabled);
            reflector = find(state.reflectors, candidates[c].id);
            if (Viewport* target = find(state.viewports, reflector->viewport))
            {
                target->reflection = true;
                target->outputLinear = true;
            }
        }
        viewportSetScenario(reflector->viewport, reflector->scenario);
        const std::uint32_t width = reflector->resolution;
        const std::uint32_t height = static_cast<std::uint32_t>(
                static_cast<float>(reflector->resolution) / aspect + 0.5f);
        viewportSetSize(reflector->viewport, width, height < 16 ? 16 : height);
        reflector = find(state.reflectors, candidates[c].id);

        Math::Vec3 normal =
                reflector->transform.TransformVector(Math::Vec3(0.0f, 0.0f, 1.0f)).Normalized();
        const Math::Vec3 point = reflector->transform.GetTranslation();
        if (normal.Dot(eye - point) < 0.0f) normal = normal * -1.0f;
        const float d = -normal.Dot(point);
        reflector->planeNormal = normal;
        reflector->planePoint = point;

        const Math::Vec3 forward =
                camera->transform.TransformVector(Math::Vec3(0.0f, 0.0f, -1.0f)).Normalized();
        const Math::Vec3 up = camera->transform.TransformVector(Math::Vec3(0.0f, 1.0f, 0.0f));
        const Math::Vec3 mirroredEye = reflectPoint(eye, normal, d);
        const Math::Vec3 mirroredForward = reflectVector(forward, normal).Normalized();
        const Math::Vec3 mirroredUp = reflectVector(up, normal);
        Math::Vec3 right = mirroredForward.Cross(mirroredUp);
        if (right.Length() < 0.0001f) right = mirroredForward.Cross(Math::Vec3(1.0f, 0.0f, 0.0f));
        right = right.Normalized();
        const Math::Vec3 trueUp = right.Cross(mirroredForward);
        Math::Mat4 mirrored = Math::Mat4::Identity();
        mirrored.col0 = Math::Vec4(right, 0.0f);
        mirrored.col1 = Math::Vec4(trueUp, 0.0f);
        mirrored.col2 = Math::Vec4(mirroredForward * -1.0f, 0.0f);
        mirrored.col3 = Math::Vec4(mirroredEye, 1.0f);
        cameraSetTransform(reflector->camera, mirrored);
        if (camera->projection == CameraProjection::Perspective)
            cameraSetPerspective(reflector->camera, camera->fovy, camera->zNear, camera->zFar);
        else
            cameraSetOrthogonal(reflector->camera, camera->size, camera->zNear, camera->zFar);

        const Math::Mat4 mirroredView = mirrored.InverseAffine();
        const Math::Vec3 normalView = mirroredView.TransformVector(normal).Normalized();
        const Math::Vec3 pointView = mirroredView.TransformPoint(point);
        const Math::Mat4 oblique =
                obliqueProjection(projection, normalView, -normalView.Dot(pointView));
        cameraSetCustomProjection(reflector->camera, oblique);
        reflector->viewProjection = zeroToOneDepth() * oblique * mirroredView;

        state.activeReflectors[state.activeReflectorCount++] = candidates[c].id;
    }

    for (std::uint32_t c = 0; c < state.activeReflectorCount; ++c)
    {
        Reflector* reflector = find(state.reflectors, state.activeReflectors[c]);
        Viewport* target = reflector ? find(state.viewports, reflector->viewport) : nullptr;
        if (!target) continue;
        drawViewport(*target);
        ++mStats.reflectorsDrawn;
    }
}

void RenderingServer::lightSetShadow(LightId light, bool enabled)
{
    if (Light* item = find(mState->lights, light)) item->shadow = enabled;
}

void RenderingServer::lightSetDirectionalShadowMode(LightId light, DirectionalShadowMode mode)
{
    if (Light* item = find(mState->lights, light)) item->shadowMode = mode;
}

void RenderingServer::lightSetBlendSplits(LightId light, bool blend)
{
    if (Light* item = find(mState->lights, light)) item->blendSplits = blend;
}

void RenderingServer::lightSetOmniShadowMode(LightId light, OmniShadowMode mode)
{
    if (Light* item = find(mState->lights, light)) item->omniShadowMode = mode;
}

void RenderingServer::instanceSetCastShadows(InstanceId instance, bool cast)
{
    if (Instance* item = find(mState->instances, instance)) item->castShadows = cast;
}

void RenderingServer::setDirectionalShadowSize(std::uint32_t size, bool sixteenBits)
{
    std::uint32_t rounded = 1;
    while (rounded < size) rounded <<= 1;
    if (rounded < 256) rounded = 256;
    if (mState->shadowAtlasSize == rounded && mState->shadowSixteenBits == sixteenBits) return;
    mState->shadowAtlasSize = rounded;
    mState->shadowSixteenBits = sixteenBits;
    if (mState->shadowAtlas.valid()) mDriver->destroy(mState->shadowAtlas);
    mState->shadowAtlas = prisma::TextureHandle();
}

void RenderingServer::setDirectionalShadowQuality(ShadowQuality quality)
{
    mState->applyShadowQuality(quality);
}

void RenderingServer::setPositionalShadowQuality(ShadowQuality quality)
{
    mState->applyPositionalShadowQuality(quality);
}

std::uint32_t RenderingServer::directionalShadowSize() const
{
    return mState->shadowAtlasSize;
}

bool RenderingServer::directionalShadowSixteenBits() const
{
    return mState->shadowSixteenBits;
}

ShadowQuality RenderingServer::directionalShadowQuality() const
{
    return mState->shadowQuality;
}

ShadowQuality RenderingServer::positionalShadowQuality() const
{
    return mState->positionalQuality;
}

void RenderingServer::lightFree(LightId light)
{
    if (const Light* item = find(mState->lights, light))
        for (Viewport& viewport: mState->viewports) viewport.shadowAtlas.releaseLight(item->serial);
    erase(mState->lights, light);
}

DecalId RenderingServer::decalCreate()
{
    return toId<DecalId>(mState->decals.insert(Decal()));
}

void RenderingServer::decalSetScenario(DecalId decal, ScenarioId scenario)
{
    if (Decal* item = find(mState->decals, decal)) item->scenario = scenario;
}

void RenderingServer::decalSetTransform(DecalId decal, const Math::Mat4& transform)
{
    if (Decal* item = find(mState->decals, decal)) item->transform = transform;
}

void RenderingServer::decalSetVisible(DecalId decal, bool visible)
{
    if (Decal* item = find(mState->decals, decal)) item->visible = visible;
}

void RenderingServer::decalSetSize(DecalId decal, const Math::Vec3& size)
{
    if (Decal* item = find(mState->decals, decal)) item->params.size = size;
}

void RenderingServer::decalSetTexture(DecalId decal, DecalTexture type, TextureId texture)
{
    Decal* item = find(mState->decals, decal);
    if (!item || type >= DecalTexture::Max) return;
    TextureId& slot = item->params.textures[static_cast<std::size_t>(type)];
    if (slot == texture) return;
    if (slot.valid()) mState->decalAtlas.removeTexture(slot);
    slot = texture;
    if (slot.valid()) mState->decalAtlas.addTexture(slot);
}

void RenderingServer::decalSetEmissionEnergy(DecalId decal, float energy)
{
    if (Decal* item = find(mState->decals, decal)) item->params.emissionEnergy = energy;
}

void RenderingServer::decalSetAlbedoMix(DecalId decal, float mix)
{
    if (Decal* item = find(mState->decals, decal)) item->params.albedoMix = mix;
}

void RenderingServer::decalSetModulate(DecalId decal, const Math::Vec4& modulate)
{
    if (Decal* item = find(mState->decals, decal)) item->params.modulate = modulate;
}

void RenderingServer::decalSetCullMask(DecalId decal, std::uint32_t mask)
{
    if (Decal* item = find(mState->decals, decal)) item->params.cullMask = mask;
}

void RenderingServer::decalSetDistanceFade(DecalId decal, bool enabled, float begin, float length)
{
    Decal* item = find(mState->decals, decal);
    if (!item) return;
    item->params.distanceFade = enabled;
    item->params.distanceFadeBegin = begin;
    item->params.distanceFadeLength = length;
}

void RenderingServer::decalSetFade(DecalId decal, float above, float below)
{
    Decal* item = find(mState->decals, decal);
    if (!item) return;
    item->params.upperFade = above;
    item->params.lowerFade = below;
}

void RenderingServer::decalSetNormalFade(DecalId decal, float fade)
{
    if (Decal* item = find(mState->decals, decal)) item->params.normalFade = fade;
}

void RenderingServer::decalSetSortingOffset(DecalId decal, float offset)
{
    if (Decal* item = find(mState->decals, decal)) item->sortingOffset = offset;
}

void RenderingServer::decalFree(DecalId decal)
{
    Decal* item = find(mState->decals, decal);
    if (!item) return;
    for (std::size_t i = 0; i < static_cast<std::size_t>(DecalTexture::Max); ++i)
        if (item->params.textures[i].valid())
            mState->decalAtlas.removeTexture(item->params.textures[i]);
    erase(mState->decals, decal);
}

const DecalParams* RenderingServer::decalGetParams(DecalId decal) const
{
    const Decal* item = find(mState->decals, decal);
    return item ? &item->params : nullptr;
}

const DecalAtlas& RenderingServer::getDecalAtlas() const
{
    return mState->decalAtlas;
}

std::uint32_t RenderingServer::getDecalFrameCount() const
{
    return mState->decalCount;
}

const DecalDataGpu* RenderingServer::getDecalFrameData() const
{
    return mState->decalData.data();
}

ReflectionProbeId RenderingServer::reflectionProbeCreate()
{
    return toId<ReflectionProbeId>(mState->reflectionProbes.insert(ReflectionProbe()));
}

void RenderingServer::reflectionProbeSetScenario(ReflectionProbeId probe, ScenarioId scenario)
{
    if (ReflectionProbe* item = find(mState->reflectionProbes, probe)) item->scenario = scenario;
}

void RenderingServer::reflectionProbeSetTransform(ReflectionProbeId probe,
        const Math::Mat4& transform)
{
    if (ReflectionProbe* item = find(mState->reflectionProbes, probe)) item->transform = transform;
}

void RenderingServer::reflectionProbeSetVisible(ReflectionProbeId probe, bool visible)
{
    if (ReflectionProbe* item = find(mState->reflectionProbes, probe)) item->visible = visible;
}

void RenderingServer::reflectionProbeSetParams(ReflectionProbeId probe,
        const ReflectionProbeParams& params)
{
    if (ReflectionProbe* item = find(mState->reflectionProbes, probe)) item->params = params;
}

void RenderingServer::reflectionProbeFree(ReflectionProbeId probe)
{
    if (find(mState->reflectionProbes, probe)) erase(mState->reflectionProbes, probe);
}

const ReflectionProbeParams* RenderingServer::reflectionProbeGetParams(
        ReflectionProbeId probe) const
{
    const ReflectionProbe* item = find(mState->reflectionProbes, probe);
    return item ? &item->params : nullptr;
}

std::uint32_t RenderingServer::getReflectionProbeFrameCount() const
{
    return mState->reflectionCount;
}

const ReflectionDataGpu* RenderingServer::getReflectionProbeFrameData() const
{
    return mState->reflectionData.data();
}

InstanceId RenderingServer::instanceCreate()
{
    return toId<InstanceId>(mState->instances.insert(Instance()));
}

InstanceId RenderingServer::instanceCreate(MeshId base, ScenarioId scenario)
{
    const InstanceId id = instanceCreate();
    instanceSetBase(id, base);
    instanceSetScenario(id, scenario);
    return id;
}

void RenderingServer::instanceSetBase(InstanceId instance, MeshId base)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    item->base = base;
    item->multimesh = MultiMeshId();
    mState->freeMeshInstance(mDriver, *item);
    mState->markIndexDirty(item->scenario);
}

void RenderingServer::instanceSetBase(InstanceId instance, MultiMeshId base)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    item->base = MeshId();
    item->multimesh = base;
    if (item->skeleton.valid())
    {
        removeInstance(mState->skinnedInstances, instance);
        item->skeleton = SkeletonId();
    }
    mState->freeMeshInstance(mDriver, *item);
    mState->markIndexDirty(item->scenario);
}

void RenderingServer::instanceAttachSkeleton(InstanceId instance, SkeletonId skeleton)
{
    Instance* item = find(mState->instances, instance);
    if (!item || item->skeleton == skeleton) return;
    if (item->multimesh.valid() && skeleton.valid()) return;
    const bool wasSkinned = item->skeleton.valid();
    item->skeleton = skeleton;
    if (item->meshInstance)
    {
        item->meshInstance->skeletonVersion = 0;
        item->meshInstance->dirty = true;
    }
    if (skeleton.valid() != wasSkinned) mState->markIndexDirty(item->scenario);
    if (skeleton.valid() && !wasSkinned) mState->skinnedInstances.push_back(instance);
    if (!skeleton.valid() && wasSkinned)
    {
        removeInstance(mState->skinnedInstances, instance);
        mState->freeMeshInstance(mDriver, *item);
    }
    mState->queueBounds(*item, instance);
}

SkeletonId RenderingServer::instanceGetSkeleton(InstanceId instance) const
{
    const Instance* item = find(mState->instances, instance);
    return item ? item->skeleton : SkeletonId();
}

prisma::BufferHandle RenderingServer::instanceSkinnedVertexBuffer(InstanceId instance,
        std::uint32_t surface) const
{
    const Instance* item = find(mState->instances, instance);
    if (!item || !item->meshInstance || surface >= item->meshInstance->surfaces.size())
        return prisma::BufferHandle();
    return item->meshInstance->surfaces[surface].vertexBuffer;
}

MultiMeshId RenderingServer::multimeshCreate()
{
    return mState->multimeshes.create();
}

void RenderingServer::multimeshAllocateData(MultiMeshId multimesh, std::uint32_t instances,
        bool useColors, bool useCustomData)
{
    mState->multimeshes.allocateData(multimesh, instances, useColors, useCustomData);
}

std::uint32_t RenderingServer::multimeshGetInstanceCount(MultiMeshId multimesh) const
{
    return mState->multimeshes.getInstanceCount(multimesh);
}

void RenderingServer::multimeshSetMesh(MultiMeshId multimesh, MeshId mesh)
{
    mState->multimeshes.setMesh(multimesh, mesh);
}

MeshId RenderingServer::multimeshGetMesh(MultiMeshId multimesh) const
{
    return mState->multimeshes.getMesh(multimesh);
}

void RenderingServer::multimeshInstanceSetTransform(MultiMeshId multimesh, std::uint32_t index,
        const Math::Mat4& transform)
{
    mState->multimeshes.instanceSetTransform(multimesh, index, transform);
}

void RenderingServer::multimeshInstanceSetColor(MultiMeshId multimesh, std::uint32_t index,
        const Math::Vec4& color)
{
    mState->multimeshes.instanceSetColor(multimesh, index, color);
}

void RenderingServer::multimeshInstanceSetCustomData(MultiMeshId multimesh, std::uint32_t index,
        const Math::Vec4& data)
{
    mState->multimeshes.instanceSetCustomData(multimesh, index, data);
}

Math::Mat4 RenderingServer::multimeshInstanceGetTransform(MultiMeshId multimesh,
        std::uint32_t index)
{
    return mState->multimeshes.instanceGetTransform(multimesh, index);
}

Math::Vec4 RenderingServer::multimeshInstanceGetColor(MultiMeshId multimesh, std::uint32_t index)
{
    return mState->multimeshes.instanceGetColor(multimesh, index);
}

Math::Vec4 RenderingServer::multimeshInstanceGetCustomData(MultiMeshId multimesh,
        std::uint32_t index)
{
    return mState->multimeshes.instanceGetCustomData(multimesh, index);
}

bool RenderingServer::multimeshSetBuffer(MultiMeshId multimesh, const float* data,
        std::size_t count)
{
    return mState->multimeshes.setBuffer(multimesh, data, count);
}

bool RenderingServer::multimeshGetBuffer(MultiMeshId multimesh, ct::Vector<float>& out)
{
    return mState->multimeshes.getBuffer(multimesh, out);
}

void RenderingServer::multimeshSetVisibleInstances(MultiMeshId multimesh, int visible)
{
    mState->multimeshes.setVisibleInstances(multimesh, visible);
}

int RenderingServer::multimeshGetVisibleInstances(MultiMeshId multimesh) const
{
    return mState->multimeshes.getVisibleInstances(multimesh);
}

void RenderingServer::multimeshSetDissolve(MultiMeshId multimesh, bool enable)
{
    mState->multimeshes.setDissolve(multimesh, enable);
}

bool RenderingServer::multimeshGetDissolve(MultiMeshId multimesh) const
{
    return mState->multimeshes.getDissolve(multimesh);
}

void RenderingServer::multimeshSetCustomAabb(MultiMeshId multimesh, const Math::Box& aabb)
{
    mState->multimeshes.setCustomAabb(multimesh, aabb);
}

Math::Box RenderingServer::multimeshGetCustomAabb(MultiMeshId multimesh) const
{
    return mState->multimeshes.getCustomAabb(multimesh);
}

Math::Box RenderingServer::multimeshGetAabb(MultiMeshId multimesh)
{
    return mState->multimeshes.getAabb(multimesh);
}

bool RenderingServer::multimeshValid(MultiMeshId multimesh) const
{
    return mState->multimeshes.valid(multimesh);
}

void RenderingServer::multimeshFree(MultiMeshId multimesh)
{
    mState->multimeshes.free(multimesh);
}

ImpostorId RenderingServer::impostorCreate(MeshId mesh,
        const ct::Vector<MaterialId>& surfaceMaterials, float swapInDistance)
{
    return mState->impostors.create(mesh, surfaceMaterials, swapInDistance);
}

void RenderingServer::impostorFree(ImpostorId impostor)
{
    mState->impostors.free(impostor);
}

void RenderingServer::impostorSetSwapInDistance(ImpostorId impostor, float distance)
{
    ImpostorManager::Entry* entry = mState->impostors.get(impostor);
    if (!entry) return;
    entry->swapInDistance = distance > 0.0f ? distance : 0.0f;
    mState->impostors.markSetsDirty();
}

float RenderingServer::impostorGetSwapInDistance(ImpostorId impostor) const
{
    const ImpostorManager::Entry* entry = mState->impostors.get(impostor);
    return entry ? entry->swapInDistance : 0.0f;
}

void RenderingServer::impostorSetDirty(ImpostorId impostor)
{
    mState->impostors.setDirty(impostor);
}

int RenderingServer::impostorGetSlot(ImpostorId impostor) const
{
    const ImpostorManager::Entry* entry = mState->impostors.get(impostor);
    return entry ? entry->slot : -1;
}

bool RenderingServer::impostorIsCaptured(ImpostorId impostor) const
{
    const ImpostorManager::Entry* entry = mState->impostors.get(impostor);
    return entry && entry->captured && !entry->dirty;
}

ImpostorSetId RenderingServer::impostorSetCreate(ScenarioId scenario)
{
    return mState->impostors.createSet(scenario.bits());
}

void RenderingServer::impostorSetFree(ImpostorSetId set)
{
    mState->impostors.freeSet(mDriver, set);
}

void RenderingServer::impostorSetResize(ImpostorSetId set, std::uint32_t count)
{
    ImpostorManager::Set* item = mState->impostors.getSet(set);
    if (!item) return;
    item->records.resize(count);
    item->dirty = true;
}

std::uint32_t RenderingServer::impostorSetGetCount(ImpostorSetId set) const
{
    const ImpostorManager::Set* item = mState->impostors.getSet(set);
    return item ? static_cast<std::uint32_t>(item->records.size()) : 0;
}

void RenderingServer::impostorSetInstance(ImpostorSetId set, std::uint32_t index,
        ImpostorId impostor, const Math::Mat4& transform, const Math::Vec4& color)
{
    ImpostorManager::Set* item = mState->impostors.getSet(set);
    if (!item || index >= item->records.size()) return;
    auto channel = [](float value)
    {
        const float clamped = value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
        return static_cast<std::uint32_t>(clamped * 255.0f + 0.5f);
    };
    ImpostorManager::Record& record = item->records[index];
    record.impostor = impostor;
    record.transform = transform;
    record.color = channel(color.x) | (channel(color.y) << 8) | (channel(color.z) << 16) |
                   (channel(color.w) << 24);
    item->dirty = true;
}

bool RenderingServer::impostorSetGetInstance(ImpostorSetId set, std::uint32_t index,
        ImpostorInstanceInfo& info) const
{
    const ImpostorManager::Set* item = mState->impostors.getSet(set);
    if (!item || index >= item->records.size()) return false;
    return mState->impostors.instanceInfo(mMeshes, item->records[index], info);
}

void RenderingServer::impostorSetEnabled(ImpostorSetId set, bool enabled)
{
    ImpostorManager::Set* item = mState->impostors.getSet(set);
    if (!item) return;
    item->enabled = enabled;
    item->dirty = true;
}

bool RenderingServer::impostorSetReadCards(ImpostorSetId set, std::uint32_t& indexCount,
        ct::Vector<ImpostorCardReadback>& cards)
{
    return mState->impostors.readCards(mDriver, set, indexCount, cards);
}

std::uint32_t RenderingServer::impostorRefresh()
{
    if (mState->impostors.count() == 0) return 0;
    openFrame();
    return mState->impostors.refresh(mDriver, mMeshes, mMaterials, mTextures);
}

bool RenderingServer::impostorReadSlice(ImpostorId impostor, std::uint32_t angle,
        ImpostorProperty property, ct::Vector<std::uint8_t>& rgba)
{
    return mState->impostors.readSlice(mDriver, impostor, angle, property, rgba);
}

std::uint32_t RenderingServer::impostorCaptureSamples() const
{
    return mState->impostors.samples();
}

std::uint64_t RenderingServer::impostorTextureBytes() const
{
    return mState->impostors.textureBytes();
}

SkeletonId RenderingServer::skeletonCreate()
{
    return toId<SkeletonId>(mState->skeletons.insert(Skeleton()));
}

void RenderingServer::skeletonAllocateData(SkeletonId skeleton, std::uint32_t bones)
{
    Skeleton* item = find(mState->skeletons, skeleton);
    if (!item || item->size == bones) return;
    if (item->buffer.valid()) mDriver->destroy(item->buffer);
    item->buffer = prisma::BufferHandle();
    item->data.clear();
    item->size = bones;
    if (bones > 0)
    {
        item->data.resize(static_cast<std::size_t>(bones) * kBoneFloats);
        memset(item->data.data(), 0, item->data.size() * sizeof(float));
        for (std::uint32_t b = 0; b < bones; ++b)
            State::storeBoneTransform(*item, b, Math::Mat4::Identity());
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Storage;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = static_cast<std::uint32_t>(item->data.size() * sizeof(float));
        desc.data = item->data.data();
        desc.debugName = "skeleton bones";
        item->buffer = mDriver->createBuffer(desc);
        if (!item->buffer.valid()) log_error("render: could not create a skeleton buffer");
    }
    item->dirty = true;
}

std::uint32_t RenderingServer::skeletonGetBoneCount(SkeletonId skeleton) const
{
    const Skeleton* item = find(mState->skeletons, skeleton);
    return item ? item->size : 0;
}

void RenderingServer::skeletonBoneSetTransform(SkeletonId skeleton, std::uint32_t bone,
        const Math::Mat4& transform)
{
    Skeleton* item = find(mState->skeletons, skeleton);
    if (!item || bone >= item->size) return;
    State::storeBoneTransform(*item, bone, transform);
    item->dirty = true;
}

Math::Mat4 RenderingServer::skeletonBoneGetTransform(SkeletonId skeleton, std::uint32_t bone) const
{
    const Skeleton* item = find(mState->skeletons, skeleton);
    if (!item || bone >= item->size) return Math::Mat4::Identity();
    return State::boneTransform(*item, bone);
}

void RenderingServer::skeletonFree(SkeletonId skeleton)
{
    Skeleton* item = find(mState->skeletons, skeleton);
    if (!item) return;
    for (std::size_t i = 0; i < mState->skinnedInstances.size();)
    {
        const InstanceId id = mState->skinnedInstances[i];
        Instance* instance = find(mState->instances, id);
        if (instance && instance->skeleton == skeleton)
        {
            instanceAttachSkeleton(id, SkeletonId());
            continue;
        }
        ++i;
    }
    if (item->buffer.valid()) mDriver->destroy(item->buffer);
    erase(mState->skeletons, skeleton);
}

Math::Box RenderingServer::meshAabb(const MeshInfo& mesh, SkeletonId skeleton) const
{
    return mState->meshBox(mesh, skeleton);
}

void RenderingServer::updateSkeletons()
{
    for (Skeleton& skeleton: mState->skeletons)
    {
        if (!skeleton.dirty) continue;
        if (skeleton.size > 0 && skeleton.buffer.valid())
            mDriver->updateBuffer(skeleton.buffer, 0, skeleton.data.data(),
                    static_cast<std::uint32_t>(skeleton.data.size() * sizeof(float)));
        ++skeleton.version;
        skeleton.dirty = false;
    }
}

void RenderingServer::updateMeshInstances()
{
    State& state = *mState;
    state.skinQueue.clear();
    for (std::size_t i = 0; i < state.skinnedInstances.size(); ++i)
    {
        const InstanceId id = state.skinnedInstances[i];
        Instance* instance = find(state.instances, id);
        if (!instance) continue;
        const Skeleton* skeleton = find(state.skeletons, instance->skeleton);
        const MeshInfo* mesh = mMeshes.get(instance->base);
        if (!skeleton || !state.syncMeshInstance(mDriver, *instance, mesh)) continue;
        MeshInstance& meshInstance = *instance->meshInstance;
        if (!meshInstance.dirty && meshInstance.skeletonVersion == skeleton->version) continue;
        state.queueBounds(*instance, id);
        // Out of every view last frame: the pose waits (still dirty) until a camera sees it
        // again; one that was never skinned is done once, so it does not show its bind pose.
        if (meshInstance.skeletonVersion != 0 && instance->seenFrame + 1 < state.frameNumber)
            continue;
        if (skeleton->size == 0 || !skeleton->buffer.valid() || !state.skinningSupported)
        {
            meshInstance.dirty = false;
            meshInstance.skeletonVersion = skeleton->version;
            continue;
        }
        state.skinQueue.push_back(instance);
    }
    if (state.skinQueue.empty()) return;

    RenderProfiler* profiler = RenderProfiler::singleton();
    if (profiler && !profiler->passEnabled("Skinning")) return;

    std::uint32_t dispatches = 0;
    for (const Instance* instance: state.skinQueue)
        for (const MeshInstance::Surface& surface: instance->meshInstance->surfaces)
            if (surface.vertexBuffer.valid()) ++dispatches;
    if (dispatches == 0) return;

    const std::uint32_t stride =
            alignUp(kSkinParamsSize, mDriver->caps().uniformBufferOffsetAlignment);
    const std::uint32_t paramsSize = dispatches * stride;
    if (!state.ensureSkinParams(mDriver, paramsSize)) return;
    state.skinParamsBytes.resize(paramsSize);
    memset(state.skinParamsBytes.data(), 0, paramsSize);

    std::uint32_t slot = 0;
    for (const Instance* instance: state.skinQueue)
    {
        const MeshInfo* mesh = mMeshes.get(instance->base);
        for (std::size_t s = 0; s < instance->meshInstance->surfaces.size(); ++s)
        {
            if (!instance->meshInstance->surfaces[s].vertexBuffer.valid()) continue;
            const SurfaceInfo& surface = mesh->surfaces[s];
            const std::uint32_t mask = surface.attributeMask;
            std::uint32_t* params = reinterpret_cast<std::uint32_t*>(
                    state.skinParamsBytes.data() + static_cast<std::size_t>(slot) * stride);
            params[0] = surface.vertexCount;
            params[1] = surface.vertexStride / 4;
            params[2] = MeshStore::attributeOffset(mask, kVertexPosition) / 4;
            params[3] = MeshStore::attributeOffset(mask, kVertexNormal) / 4;
            params[4] = MeshStore::attributeOffset(mask, kVertexTangent) / 4;
            params[5] = MeshStore::attributeOffset(mask, kVertexJoints) / 4;
            params[6] = MeshStore::attributeOffset(mask, kVertexWeights) / 4;
            params[7] = ((mask & kVertexNormal) ? 1u : 0u) | ((mask & kVertexTangent) ? 2u : 0u);
            ++slot;
        }
    }
    mDriver->updateBuffer(state.skinParamsBuffer, 0, state.skinParamsBytes.data(), paramsSize);

    const bool gpuScope = profiler != nullptr && profiler->beginGpu("Skinning");
    mDriver->beginComputePass();
    mDriver->bindPipeline(state.skinPipeline);
    slot = 0;
    for (Instance* instance: state.skinQueue)
    {
        const MeshInfo* mesh = mMeshes.get(instance->base);
        const Skeleton* skeleton = find(state.skeletons, instance->skeleton);
        const std::uint32_t boneBytes =
                static_cast<std::uint32_t>(skeleton->data.size() * sizeof(float));
        MeshInstance& meshInstance = *instance->meshInstance;
        for (std::size_t s = 0; s < meshInstance.surfaces.size(); ++s)
        {
            const MeshInstance::Surface& target = meshInstance.surfaces[s];
            if (!target.vertexBuffer.valid()) continue;
            const SurfaceInfo& surface = mesh->surfaces[s];
            mDriver->bindUniformBuffer(0, state.skinParamsBuffer, slot * stride, kSkinParamsSize);
            mDriver->bindStorageBuffer(0, surface.vertexBuffer, 0, target.bytes);
            mDriver->bindStorageBuffer(1, target.vertexBuffer, 0, target.bytes);
            mDriver->bindStorageBuffer(2, skeleton->buffer, 0, boneBytes);
            mDriver->dispatch((surface.vertexCount + kSkinGroupSize - 1) / kSkinGroupSize, 1, 1);
            ++slot;
        }
        meshInstance.dirty = false;
        meshInstance.skeletonVersion = skeleton->version;
    }
    mDriver->endComputePass();
    if (gpuScope) profiler->endGpu();
    mStats.skinDispatches += dispatches;
}

void RenderingServer::instanceSetScenario(InstanceId instance, ScenarioId scenario)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    if (Scenario* old = find(mState->scenarios, item->scenario))
        removeInstance(old->instances, instance);
    mState->markIndexDirty(item->scenario);
    item->scenario = scenario;
    if (Scenario* target = find(mState->scenarios, scenario))
    {
        target->instances.push_back(instance);
        target->indexDirty = true;
    }
}

void RenderingServer::instanceSetTransform(InstanceId instance, const Math::Mat4& transform)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    item->transform = transform;
    if (item->queued) return;
    Scenario* scenario = mState->indexedScenario(*item, instance);
    if (!scenario) return;
    item->queued = true;
    scenario->dirty.push_back(instance);
}

void RenderingServer::instanceSetVisible(InstanceId instance, bool visible)
{
    Instance* item = find(mState->instances, instance);
    if (!item || item->visible == visible) return;
    item->visible = visible;
    if (Scenario* scenario = mState->indexedScenario(*item, instance))
    {
        if (visible) State::addMask(*scenario, item->layerMask);
        else
            State::removeMask(*scenario, item->layerMask);
    }
}

void RenderingServer::instanceSetLayerMask(InstanceId instance, std::uint32_t mask)
{
    Instance* item = find(mState->instances, instance);
    if (!item || item->layerMask == mask) return;
    if (item->visible)
    {
        if (Scenario* scenario = mState->indexedScenario(*item, instance))
        {
            State::removeMask(*scenario, item->layerMask);
            State::addMask(*scenario, mask);
        }
    }
    item->layerMask = mask;
}

void RenderingServer::instanceSetLodBias(InstanceId instance, float bias)
{
    if (Instance* item = find(mState->instances, instance)) item->lodBias = bias;
}

void RenderingServer::instanceSetVisibilityRange(InstanceId instance, float begin, float end,
        float beginMargin, float endMargin)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    item->rangeBegin = begin;
    item->rangeEnd = end;
    item->rangeBeginMargin = beginMargin;
    item->rangeEndMargin = endMargin;
}

void RenderingServer::instanceSetColor(InstanceId instance, const Math::Vec4& color)
{
    if (Instance* item = find(mState->instances, instance)) item->color = color;
}

void RenderingServer::instanceSetSurfaceMaterial(InstanceId instance, std::uint32_t surface,
        MaterialId material)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    if (surface >= item->surfaceMaterials.size())
    {
        if (!material.valid()) return;
        item->surfaceMaterials.resize(surface + 1);
    }
    item->surfaceMaterials[surface] = material;
}

void RenderingServer::instanceSetSurfaceVisible(InstanceId instance, std::uint32_t surface,
        bool visible)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    if (surface >= item->hiddenSurfaces.size())
    {
        if (visible) return;
        item->hiddenSurfaces.resize(surface + 1, 0);
    }
    item->hiddenSurfaces[surface] = visible ? 0 : 1;
}

void RenderingServer::instanceSetMaterialOverride(InstanceId instance, MaterialId material)
{
    if (Instance* item = find(mState->instances, instance)) item->materialOverride = material;
}

MaterialId RenderingServer::instanceResolveMaterial(InstanceId instance,
        std::uint32_t surface) const
{
    const Instance* item = find(mState->instances, instance);
    if (!item) return mMaterials.defaultMaterial();
    return State::resolveMaterial(*item, mState->instanceMesh(mMeshes, *item), surface, mMaterials);
}

void RenderingServer::instanceFree(InstanceId instance)
{
    Instance* item = find(mState->instances, instance);
    if (!item) return;
    if (Scenario* scenario = find(mState->scenarios, item->scenario))
    {
        removeInstance(scenario->instances, instance);
        scenario->indexDirty = true;
    }
    if (item->skeleton.valid()) removeInstance(mState->skinnedInstances, instance);
    mState->freeMeshInstance(mDriver, *item);
    erase(mState->instances, instance);
}

float RenderingServer::time() const
{
    return static_cast<float>(static_cast<double>(time_nanos() - mState->startNanos) * 1.0e-9);
}

Math::Box RenderingServer::instanceGetAabb(InstanceId instance) const
{
    const Instance* item = find(mState->instances, instance);
    if (!item) return Math::Box();
    if (!mState->instanceUsable(mMeshes, *item)) return Math::Box();
    return mState->instanceBounds(*item, *mState->instanceMesh(mMeshes, *item));
}

CameraId RenderingServer::cameraCreate()
{
    return toId<CameraId>(mState->cameras.insert(Camera()));
}

void RenderingServer::cameraSetPerspective(CameraId camera, float fovyDegrees, float zNear,
        float zFar)
{
    Camera* item = find(mState->cameras, camera);
    if (!item) return;
    item->projection = CameraProjection::Perspective;
    item->fovy = fovyDegrees;
    item->zNear = zNear;
    item->zFar = zFar;
}

void RenderingServer::cameraSetOrthogonal(CameraId camera, float size, float zNear, float zFar)
{
    Camera* item = find(mState->cameras, camera);
    if (!item) return;
    item->projection = CameraProjection::Orthogonal;
    item->size = size;
    item->zNear = zNear;
    item->zFar = zFar;
}

void RenderingServer::cameraSetTransform(CameraId camera, const Math::Mat4& transform)
{
    if (Camera* item = find(mState->cameras, camera)) item->transform = transform;
}

void RenderingServer::cameraSetCullMask(CameraId camera, std::uint32_t mask)
{
    if (Camera* item = find(mState->cameras, camera)) item->cullMask = mask;
}

void RenderingServer::cameraSetAttributes(CameraId camera,
        const CameraAttributesSettings& attributes)
{
    if (Camera* item = find(mState->cameras, camera))
    {
        item->hasAttributes = true;
        item->attributes = attributes;
    }
}

void RenderingServer::cameraClearAttributes(CameraId camera)
{
    if (Camera* item = find(mState->cameras, camera))
    {
        item->hasAttributes = false;
        item->attributes = CameraAttributesSettings();
    }
}

void RenderingServer::cameraFree(CameraId camera)
{
    erase(mState->cameras, camera);
}

ViewportId RenderingServer::viewportCreate()
{
    return toId<ViewportId>(mState->viewports.insert(Viewport()));
}

void RenderingServer::viewportSetSize(ViewportId viewport, std::uint32_t width,
        std::uint32_t height)
{
    Viewport* item = find(mState->viewports, viewport);
    if (!item) return;
    if (item->width == width && item->height == height) return;
    item->width = width;
    item->height = height;
    State::destroyTargets(mDriver, *item);
    State::destroyOutput(mTextures, *item);
}

void RenderingServer::viewportAttachToScreen(ViewportId viewport, bool attach)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->toScreen = attach;
}

void RenderingServer::viewportSetActive(ViewportId viewport, bool active)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->active = active;
}

void RenderingServer::viewportAttachCamera(ViewportId viewport, CameraId camera)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->camera = camera;
}

void RenderingServer::viewportSetScenario(ViewportId viewport, ScenarioId scenario)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->scenario = scenario;
}

void RenderingServer::viewportSetClearColor(ViewportId viewport, const Math::Vec4& color)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->clearColor = color;
}

void RenderingServer::viewportSetTransparentBackground(ViewportId viewport, bool enabled)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->transparent = enabled;
}

bool RenderingServer::viewportGetTransparentBackground(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item != nullptr && item->transparent;
}

void RenderingServer::viewportSetUpdateMode(ViewportId viewport, ViewportUpdateMode mode)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->updateMode = mode;
}

void RenderingServer::viewportSetSrgbOutput(ViewportId viewport, bool enabled)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->srgbOutput = enabled;
}

bool RenderingServer::viewportGetSrgbOutput(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item != nullptr && item->srgbOutput;
}

void RenderingServer::viewportSetDrawBatch(ViewportId viewport, bool enabled)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->drawBatch = enabled;
}

bool RenderingServer::viewportGetDrawBatch(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item != nullptr && item->drawBatch;
}

ViewportUpdateMode RenderingServer::viewportGetUpdateMode(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->updateMode : ViewportUpdateMode::Disabled;
}

void RenderingServer::viewportSetDebugDraw(ViewportId viewport, ViewportDebugDraw mode)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->debugDraw = mode;
}

ViewportDebugDraw RenderingServer::viewportGetDebugDraw(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->debugDraw : ViewportDebugDraw::Disabled;
}

void RenderingServer::viewportSetMsaa3D(ViewportId viewport, ViewportMsaa msaa)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->msaa = msaa;
}

void RenderingServer::viewportSetScaling3D(ViewportId viewport, float scale)
{
    Viewport* item = find(mState->viewports, viewport);
    if (!item) return;
    item->scaling3D = scale < 0.25f ? 0.25f : scale > 2.0f ? 2.0f : scale;
}

void RenderingServer::viewportSetScaling3DSize(ViewportId viewport, std::uint32_t width,
        std::uint32_t height)
{
    Viewport* item = find(mState->viewports, viewport);
    if (!item) return;
    item->scaling3DWidth = width > 16384u ? 16384u : width;
    item->scaling3DHeight = height > 16384u ? 16384u : height;
}

void RenderingServer::viewportGetScaling3DSize(ViewportId viewport, std::uint32_t& width,
        std::uint32_t& height) const
{
    const Viewport* item = find(mState->viewports, viewport);
    width = item ? item->scaling3DWidth : 0;
    height = item ? item->scaling3DHeight : 0;
}

void RenderingServer::viewportSetScaling3DAspect(ViewportId viewport, Scaling3DAspect aspect)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->scaling3DAspect = aspect;
}

Scaling3DAspect RenderingServer::viewportGetScaling3DAspect(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->scaling3DAspect : Scaling3DAspect::Stretch;
}

void RenderingServer::viewportSetScaling3DMode(ViewportId viewport, Scaling3DMode mode)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->scaling3DMode = mode;
}

Scaling3DMode RenderingServer::viewportGetScaling3DMode(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->scaling3DMode : Scaling3DMode::Bilinear;
}

void RenderingServer::viewportSetFsrSharpness(ViewportId viewport, float sharpness)
{
    Viewport* item = find(mState->viewports, viewport);
    if (!item) return;
    item->fsrSharpness = sharpness < 0.0f ? 0.0f : sharpness > 2.0f ? 2.0f : sharpness;
}

float RenderingServer::viewportGetFsrSharpness(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->fsrSharpness : 0.2f;
}

Scaling3DLayout RenderingServer::scaling3DLayout(std::uint32_t outputWidth,
        std::uint32_t outputHeight, float scale, std::uint32_t sizeWidth, std::uint32_t sizeHeight,
        Scaling3DAspect aspect)
{
    Scaling3DLayout layout;
    layout.width = outputWidth;
    layout.height = outputHeight;
    layout.rectWidth = static_cast<float>(outputWidth);
    layout.rectHeight = static_cast<float>(outputHeight);
    if (outputWidth == 0 || outputHeight == 0) return layout;
    if (sizeWidth == 0 || sizeHeight == 0)
    {
        if (scale == 1.0f) return layout;
        const float scaledWidth = static_cast<float>(outputWidth) * scale + 0.5f;
        const float scaledHeight = static_cast<float>(outputHeight) * scale + 0.5f;
        layout.width = scaledWidth < 1.0f ? 1u : static_cast<std::uint32_t>(scaledWidth);
        layout.height = scaledHeight < 1.0f ? 1u : static_cast<std::uint32_t>(scaledHeight);
        return layout;
    }
    layout.width = sizeWidth;
    layout.height = sizeHeight;
    const float windowAspect = static_cast<float>(outputWidth) / static_cast<float>(outputHeight);
    const float pictureAspect = static_cast<float>(sizeWidth) / static_cast<float>(sizeHeight);
    if (aspect == Scaling3DAspect::Stretch || fabsf(pictureAspect - windowAspect) < 0.0001f)
        return layout;
    if (aspect == Scaling3DAspect::Expand)
    {
        if (pictureAspect < windowAspect)
            layout.width = static_cast<std::uint32_t>(
                    floorf(static_cast<float>(sizeHeight) * windowAspect));
        else
            layout.height = static_cast<std::uint32_t>(
                    floorf(static_cast<float>(sizeWidth) / windowAspect));
        if (layout.width == 0) layout.width = 1;
        if (layout.height == 0) layout.height = 1;
        return layout;
    }
    const bool fullHeight = (pictureAspect < windowAspect) == (aspect == Scaling3DAspect::Keep);
    if (fullHeight) layout.rectWidth = floorf(static_cast<float>(outputHeight) * pictureAspect);
    else
        layout.rectHeight = floorf(static_cast<float>(outputWidth) / pictureAspect);
    layout.x = roundf((static_cast<float>(outputWidth) - layout.rectWidth) * 0.5f);
    layout.y = roundf((static_cast<float>(outputHeight) - layout.rectHeight) * 0.5f);
    return layout;
}

void RenderingServer::viewportSetFxaa(ViewportId viewport, bool enabled)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->fxaa = enabled;
}

bool RenderingServer::viewportGetFxaa(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item && item->fxaa;
}

void RenderingServer::viewportSetSmaa(ViewportId viewport, bool enabled)
{
    if (Viewport* item = find(mState->viewports, viewport)) item->smaa = enabled;
}

bool RenderingServer::viewportGetSmaa(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item && item->smaa;
}

void RenderingServer::setSmaaEdgeDetectionThreshold(float threshold)
{
    mState->smaaEdgeThreshold = threshold < 0.01f ? 0.01f : threshold > 0.2f ? 0.2f : threshold;
}

float RenderingServer::getSmaaEdgeDetectionThreshold() const
{
    return mState->smaaEdgeThreshold;
}

float RenderingServer::viewportGetScaling3D(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->scaling3D : 1.0f;
}

void RenderingServer::viewportSetMeshLodThreshold(ViewportId viewport, float pixels)
{
    if (Viewport* item = find(mState->viewports, viewport))
        item->meshLodThreshold = pixels < 0.0f ? 0.0f : pixels;
}

float RenderingServer::viewportGetMeshLodThreshold(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->meshLodThreshold : 1.0f;
}

ViewportMsaa RenderingServer::viewportGetMsaa3D(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->msaa : ViewportMsaa::Disabled;
}

std::uint32_t RenderingServer::viewportGetMsaaSamples(ViewportId viewport) const
{
    const Viewport* item = find(mState->viewports, viewport);
    return item ? item->samples : 1u;
}

TextureId RenderingServer::viewportGetTexture(ViewportId viewport)
{
    Viewport* item = find(mState->viewports, viewport);
    if (!item || item->toScreen || item->width == 0 || item->height == 0) return TextureId();
    if (!State::ensureOutput(mTextures, *item, item->width, item->height,
                State::outputIsLinear(*item)))
        return TextureId();
    return item->output;
}

bool RenderingServer::viewportRender(ViewportId viewport)
{
    Viewport* item = find(mState->viewports, viewport);
    if (!item || item->toScreen || item->width == 0 || item->height == 0) return false;
    openFrame();
    updateSkeletons();
    updateMeshInstances();
    if (mState->impostors.count() > 0)
        mState->impostors.refresh(mDriver, mMeshes, mMaterials, mTextures);
    mState->impostors.uploadSets(mDriver, mMeshes);
    mState->multimeshes.updateDirty();
    const RenderStats saved = mStats;
    drawViewport(*item);
    const bool drawn = mStats.viewportsDrawn > saved.viewportsDrawn;
    mStats = saved;
    return drawn;
}

prisma::TextureHandle RenderingServer::viewportGetAmbientOcclusion(ViewportId viewport) const
{
    const Viewport* target = find(mState->viewports, viewport);
    return target ? target->ssao.finalAo : prisma::TextureHandle();
}

prisma::TextureHandle RenderingServer::viewportGetReflections(ViewportId viewport) const
{
    const Viewport* target = find(mState->viewports, viewport);
    return target ? target->ssr.result() : prisma::TextureHandle();
}

prisma::TextureHandle RenderingServer::viewportGetNormalRoughness(ViewportId viewport) const
{
    const Viewport* target = find(mState->viewports, viewport);
    return target ? target->normalRoughness : prisma::TextureHandle();
}

bool RenderingServer::viewportReadDepth(ViewportId viewport, int x, int y, float& depth)
{
    State& state = *mState;
    const Viewport* item = find(state.viewports, viewport);
    if (!item || !item->depthTexture.valid() || !state.depthReadPipeline.valid()) return false;
    if (x < 0 || y < 0 || static_cast<std::uint32_t>(x) >= item->baseWidth ||
            static_cast<std::uint32_t>(y) >= item->baseHeight || item->pictureWidth <= 0.0f ||
            item->pictureHeight <= 0.0f)
        return false;

    const float params[4] = { (static_cast<float>(x) + 0.5f - item->pictureX) / item->pictureWidth,
        (static_cast<float>(y) + 0.5f - item->pictureY) / item->pictureHeight, 0.0f, 0.0f };
    if (params[0] < 0.0f || params[0] > 1.0f || params[1] < 0.0f || params[1] > 1.0f) return false;
    openFrame();
    mDriver->updateBuffer(state.depthReadParams, 0, params, sizeof(params));

    prisma::RenderPassDesc pass;
    pass.colors[0].texture = state.depthReadTarget;
    pass.colorCount = 1;
    pass.colorLoad = prisma::LoadOp::DontCare;
    pass.depthLoad = prisma::LoadOp::DontCare;
    pass.depthStore = prisma::StoreOp::Discard;
    mDriver->beginRenderPass(pass);
    prisma::Viewport onePixel;
    onePixel.width = 1.0f;
    onePixel.height = 1.0f;
    mDriver->setViewport(onePixel);
    mDriver->bindPipeline(state.depthReadPipeline);
    mDriver->bindUniformBuffer(0, state.depthReadParams, 0, sizeof(params));
    mDriver->bindTexture(0, item->depthTexture, state.nearestSampler);
    mDriver->draw(3, 0);
    mDriver->endRenderPass();

    unsigned char pixel[4] = { 0, 0, 0, 0 };
    prisma::RenderTarget source;
    source.texture = state.depthReadTarget;
    prisma::Rect area;
    area.width = 1;
    area.height = 1;
    if (!mDriver->readPixels(source, area, pixel)) return false;
    const std::uint32_t bits = static_cast<std::uint32_t>(pixel[0]) |
                               (static_cast<std::uint32_t>(pixel[1]) << 8) |
                               (static_cast<std::uint32_t>(pixel[2]) << 16) |
                               (static_cast<std::uint32_t>(pixel[3]) << 24);
    memcpy(&depth, &bits, sizeof(depth));
    return true;
}

void RenderingServer::viewportSetPositionalShadowAtlasSize(ViewportId viewport, std::uint32_t size,
        bool sixteenBits)
{
    Viewport* item = find(mState->viewports, viewport);
    if (!item) return;
    const std::uint32_t before = item->shadowAtlas.getSize();
    if (size != 0 && size < 256) size = 256;
    item->shadowAtlas.setSize(size, sixteenBits);
    if (item->shadowAtlas.getSize() != before) State::destroyPositionalAtlas(mDriver, *item);
}

void RenderingServer::viewportSetPositionalShadowAtlasQuadrantSubdivision(ViewportId viewport,
        std::uint32_t quadrant, std::uint32_t subdivision)
{
    if (Viewport* item = find(mState->viewports, viewport))
        item->shadowAtlas.setQuadrantSubdivision(quadrant, subdivision);
}

bool RenderingServer::viewportGetPositionalShadowSlot(ViewportId viewport, LightId light,
        std::uint32_t& quadrant, std::uint32_t& index, bool& omni) const
{
    const Viewport* target = find(mState->viewports, viewport);
    const Light* item = find(mState->lights, light);
    if (!target || !item) return false;
    ShadowAtlas::Slot slot;
    if (!target->shadowAtlas.getSlot(item->serial, slot)) return false;
    quadrant = slot.quadrant;
    index = slot.shadow;
    omni = slot.omni;
    return true;
}

void RenderingServer::viewportFree(ViewportId viewport)
{
    if (Viewport* target = find(mState->viewports, viewport))
    {
        State::destroyTargets(mDriver, *target);
        State::destroyPositionalAtlas(mDriver, *target);
        State::destroyOutput(mTextures, *target);
        mState->releaseRangeMask(*target);
    }
    erase(mState->viewports, viewport);
}

void RenderingServer::setRoughnessLimiter(bool enabled, float amount, float limit)
{
    mState->roughnessLimiter = enabled;
    mState->roughnessLimiterAmount = amount;
    mState->roughnessLimiterLimit = limit;
}

bool RenderingServer::roughnessLimiterEnabled() const
{
    return mState->roughnessLimiter;
}

float RenderingServer::roughnessLimiterAmount() const
{
    return mState->roughnessLimiterAmount;
}

float RenderingServer::roughnessLimiterLimit() const
{
    return mState->roughnessLimiterLimit;
}

void RenderingServer::setDepthPrePassEnabled(bool enabled)
{
    mState->depthPrePass = enabled;
}

bool RenderingServer::depthPrePassEnabled() const
{
    return mState->depthPrePass;
}

void RenderingServer::setSsaoQuality(SsaoQuality quality, bool halfSize, float adaptiveTarget,
        std::uint32_t blurPasses, float fadeoutFrom, float fadeoutTo)
{
    mState->screenSpace.setSsaoQuality(quality, halfSize, adaptiveTarget, blurPasses, fadeoutFrom,
            fadeoutTo);
}

SsaoQuality RenderingServer::ssaoQuality() const
{
    return mState->screenSpace.ssaoQuality();
}

bool RenderingServer::ssaoHalfSize() const
{
    return mState->screenSpace.ssaoHalfSize();
}

float RenderingServer::ssaoAdaptiveTarget() const
{
    return mState->screenSpace.ssaoAdaptiveTarget();
}

std::uint32_t RenderingServer::ssaoBlurPasses() const
{
    return mState->screenSpace.ssaoBlurPasses();
}

float RenderingServer::ssaoFadeoutFrom() const
{
    return mState->screenSpace.ssaoFadeoutFrom();
}

float RenderingServer::ssaoFadeoutTo() const
{
    return mState->screenSpace.ssaoFadeoutTo();
}

void RenderingServer::setSsrHalfSize(bool halfSize)
{
    mState->screenSpace.setSsrHalfSize(halfSize);
}

bool RenderingServer::ssrHalfSize() const
{
    return mState->screenSpace.ssrHalfSize();
}

void RenderingServer::setDepthOfFieldBokeh(DofBokehShape shape, DofBlurQuality quality,
        bool useJitter)
{
    mState->dof.setBokeh(shape, quality, useJitter);
}

DofBokehShape RenderingServer::depthOfFieldShape() const
{
    return mState->dof.shape();
}

DofBlurQuality RenderingServer::depthOfFieldQuality() const
{
    return mState->dof.quality();
}

bool RenderingServer::depthOfFieldJitter() const
{
    return mState->dof.useJitter();
}

void RenderingServer::setDepthOfFieldComputeAllowed(bool allowed)
{
    mState->dof.setComputeAllowed(allowed);
}

bool RenderingServer::depthOfFieldComputeAllowed() const
{
    return mState->dof.computeAllowed();
}

bool RenderingServer::screenSpaceEffectsSupported() const
{
    return mState->screenSpace.supported();
}

void RenderingServer::setScreenSpaceEffectsAllowed(bool allowed)
{
    mState->screenSpaceAllowed = allowed;
}

bool RenderingServer::screenSpaceEffectsAllowed() const
{
    return mState->screenSpaceAllowed;
}

void RenderingServer::setCullingTreeEnabled(bool enabled)
{
    if (mState->cullingTree == enabled) return;
    mState->cullingTree = enabled;
    if (enabled) mState->markAllIndexDirty();
}

bool RenderingServer::cullingTreeEnabled() const
{
    return mState->cullingTree;
}

void RenderingServer::setDeferredPipelinesEnabled(bool enabled)
{
    mState->deferredPipelines = enabled;
}

bool RenderingServer::deferredPipelinesEnabled() const
{
    return mState->deferredPipelines;
}

void RenderingServer::setCullingLeafCapacity(std::uint32_t capacity)
{
    if (capacity < 1) capacity = 1;
    if (mState->leafCapacity == capacity) return;
    mState->leafCapacity = capacity;
    mState->markAllIndexDirty();
}

std::uint32_t RenderingServer::cullingLeafCapacity() const
{
    return mState->leafCapacity;
}

std::uint32_t RenderingServer::scenePipelineCount() const
{
    return static_cast<std::uint32_t>(mState->pipelines.size());
}

void RenderingServer::framePreDrawConnect(void* owner, ct::Function<void()> callback)
{
    if (!owner) return;
    for (State::PreDrawListener& listener: mState->preDraw)
    {
        if (listener.owner != owner) continue;
        listener.callback = callback;
        return;
    }
    State::PreDrawListener listener;
    listener.owner = owner;
    listener.callback = callback;
    mState->preDraw.push_back(listener);
}

void RenderingServer::framePreDrawDisconnect(void* owner)
{
    if (!owner) return;
    ct::Vector<State::PreDrawListener>& listeners = mState->preDraw;
    for (std::size_t i = 0; i < listeners.size(); ++i)
    {
        if (listeners[i].owner != owner) continue;
        if (mState->preDrawDispatching)
        {
            listeners[i].owner = nullptr;
            listeners[i].callback.reset();
            mState->preDrawCompact = true;
        }
        else
            listeners.erase(listeners.begin() + i);
        return;
    }
}

void RenderingServer::draw(bool present, double frameStep)
{
    (void) frameStep;
    renderViewports();
    endFrame(present);
}

void RenderingServer::renderViewports()
{
    if (!mState->preDrawDispatching)
    {
        mState->preDrawDispatching = true;
        const std::size_t count = mState->preDraw.size();
        for (std::size_t i = 0; i < count; ++i)
        {
            if (!mState->preDraw[i].owner) continue;
            ct::Function<void()> callback = mState->preDraw[i].callback;
            callback();
        }
        mState->preDrawDispatching = false;
        if (mState->preDrawCompact)
        {
            mState->preDrawCompact = false;
            for (std::size_t i = mState->preDraw.size(); i > 0; --i)
            {
                if (!mState->preDraw[i - 1].owner)
                    mState->preDraw.erase(mState->preDraw.begin() + (i - 1));
            }
        }
    }
    mStats = RenderStats();
    mStats.cullTree = mState->cullingTree;
    ++mState->frameNumber;
    openFrame();
    RenderProfiler* profiler = RenderProfiler::singleton();
    mGpuFrameScope = profiler != nullptr && profiler->beginGpu("Frame");
    {
        ProfileScope scope("Skinning");
        updateSkeletons();
        updateMeshInstances();
    }
    if (mState->impostors.count() > 0)
    {
        ProfileScope scope("Impostor capture");
        mState->impostors.refresh(mDriver, mMeshes, mMaterials, mTextures);
    }
    mState->impostors.uploadSets(mDriver, mMeshes);
    mState->multimeshes.updateDirty();
    mStats.multimeshUploadBytes = mState->multimeshes.takeUploadedBytes();
    mState->batch.resetStats();
    mState->batch.upload();
    for (Viewport& viewport: mState->viewports)
    {
        if (!viewport.active) continue;
        if (viewport.toScreen)
        {
            renderReflectors(viewport);
            drawViewport(viewport);
            continue;
        }
        if (viewport.updateMode == ViewportUpdateMode::Disabled) continue;
        renderReflectors(viewport);
        drawViewport(viewport);
        if (viewport.updateMode == ViewportUpdateMode::Once)
            viewport.updateMode = ViewportUpdateMode::Disabled;
    }
    mStats.batchDrawCalls = static_cast<std::uint32_t>(mState->batch.getStats().drawCalls);
    mStats.batchVertices = static_cast<std::uint32_t>(mState->batch.getStats().verticesDrawn);
    mStats.pipelinesBuilt = mState->buildCount;
    mStats.pipelineBuildMilliseconds = static_cast<float>(mState->buildMilliseconds);
    mState->buildCount = 0;
    mState->buildMilliseconds = 0.0;
    mState->batch.clear();
}

void RenderingServer::openFrame()
{
    if (mFrameOpen) return;
    ProfileScope scope("Frame wait");
    mDriver->beginFrame();
    mFrameOpen = true;
}

bool RenderingServer::isFrameOpen() const
{
    return mFrameOpen;
}

void RenderingServer::endFrame(bool present)
{
    if (mGpuFrameScope)
    {
        if (RenderProfiler* profiler = RenderProfiler::singleton()) profiler->endGpu();
        mGpuFrameScope = false;
    }
    mDriver->endFrame();
    mFrameOpen = false;
    if (present) mDriver->present();
}

void RenderingServer::drawViewport(Viewport& viewport)
{
    const Camera* camera = find(mState->cameras, viewport.camera);
    Scenario* scenario = find(mState->scenarios, viewport.scenario);
    if (!camera || !scenario) return;

    const bool toScreen = viewport.toScreen;
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    if (toScreen)
    {
        if (Device* device = Device::singleton())
            device->windowGetFramebufferSize(&framebufferWidth, &framebufferHeight);
    }
    const std::uint32_t windowWidth =
            framebufferWidth > 0 ? static_cast<std::uint32_t>(framebufferWidth) : 0;
    const std::uint32_t windowHeight =
            framebufferHeight > 0 ? static_cast<std::uint32_t>(framebufferHeight) : 0;
    if (toScreen && (windowWidth == 0 || windowHeight == 0)) return;

    std::uint32_t width = viewport.width;
    std::uint32_t height = viewport.height;
    if (toScreen && (width == 0 || height == 0))
    {
        const bool created = viewport.colorTexture.valid();
        width = created ? viewport.baseWidth : windowWidth;
        height = created ? viewport.baseHeight : windowHeight;
    }
    if (width == 0 || height == 0) return;
    const std::uint32_t outputWidth = width;
    const std::uint32_t outputHeight = height;
    const ViewportDebugDraw debugDraw = viewport.debugDraw;
    const bool debug = debugDraw != ViewportDebugDraw::Disabled;
    if (!toScreen && !State::ensureOutput(mTextures, viewport, width, height,
                             State::outputIsLinear(viewport)))
        return;
    Scaling3DLayout layout;
    layout.width = width;
    layout.height = height;
    layout.rectWidth = static_cast<float>(width);
    layout.rectHeight = static_cast<float>(height);
    if (!viewport.reflection)
    {
        layout = scaling3DLayout(width, height, viewport.scaling3D, viewport.scaling3DWidth,
                viewport.scaling3DHeight,
                toScreen ? viewport.scaling3DAspect : Scaling3DAspect::Stretch);
        width = layout.width;
        height = layout.height;
    }
    const CameraAttributesSettings* attributes = camera->hasAttributes ? &camera->attributes
                                                 : scenario->hasEnvironment
                                                         ? &scenario->environment.cameraAttributes
                                                         : nullptr;
    RenderProfiler* dofProfiler = RenderProfiler::singleton();
    bool dofRequested = attributes != nullptr && attributes->usesDof() && !viewport.reflection &&
                        !debug && width >= 8 && height >= 8 && !mState->dof.failed() &&
                        (dofProfiler == nullptr || dofProfiler->passEnabled("DOF"));
    if (dofRequested && viewport.transparent)
    {
        dofRequested = false;
        if (!mState->dofWarnedTransparent)
        {
            mState->dofWarnedTransparent = true;
            log_warn("render: depth of field is skipped in a viewport with a transparent "
                     "background");
        }
    }
    viewport.colorStorageWanted = dofRequested && mState->dof.usesCompute();
    bool createdTargets = false;
    if (!State::ensureTargets(mDriver, viewport, width, height, createdTargets)) return;
    viewport.baseWidth = outputWidth;
    viewport.baseHeight = outputHeight;
    viewport.pictureX = layout.x;
    viewport.pictureY = layout.y;
    viewport.pictureWidth = layout.rectWidth;
    viewport.pictureHeight = layout.rectHeight;
    if (createdTargets) ++mStats.targetsCreated;

    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const Math::Mat4 baseProjection =
            camera->customProjection ? camera->customMatrix
            : camera->projection == CameraProjection::Perspective
                    ? basePerspective(camera->fovy, aspect, camera->zNear, camera->zFar)
                    : baseOrthogonal(camera->size, aspect, camera->zNear, camera->zFar);
    const Math::Mat4 view = camera->transform.InverseAffine();
    const Math::Frustum frustum = Math::Frustum::FromViewProjection(baseProjection * view);
    State::LodView shadowLodView;
    shadowLodView.cameraPosition = camera->transform.GetTranslation();
    shadowLodView.orthogonal =
            !camera->customProjection && camera->projection == CameraProjection::Orthogonal;
    shadowLodView.distanceMultiplier = 2.0f / baseProjection[0].x;
    shadowLodView.threshold = viewport.meshLodThreshold / static_cast<float>(outputWidth);
    State::LodView lodView = shadowLodView;
    if (!viewport.reflection)
        lodView.distanceMultiplier /= static_cast<float>(width) / layout.rectWidth;

    State& state = *mState;
    const std::uint32_t activeReflectors = viewport.reflection ? 0 : state.activeReflectorCount;
    state.fillPlanarData(mDriver, view, camera->transform, activeReflectors);
    RenderProfiler* profiler = RenderProfiler::singleton();
    const bool drawItems = profiler == nullptr || profiler->passEnabled("Opaque");
    const bool prepassWanted = state.depthPrePass && drawItems &&
                               (profiler == nullptr || profiler->passEnabled("Depth"));
    const bool prepareScope = profiler != nullptr && profiler->begin("Prepare");
    bool usedScreenTexture = false;
    state.setupLightmaps(mDriver, mTextures, viewport.scenario, frustum, camera->transform);
    mStats.lightmapsUsed += state.lightmapsUsed;
    state.items.clear();
    state.captureData.clear();
    std::uint32_t instancesDrawn = 0;
    std::uint32_t multimeshInstancesDrawn = 0;
    std::uint32_t lodSurfaces = 0;

    auto addItems = [&](const Instance& instance, const MeshInfo& mesh, std::uint32_t only,
                            const Math::Box& bounds)
    {
        if (only != State::kWholeInstance && only >= mesh.surfaces.size()) return;
        instance.seenFrame = state.frameNumber;
        const MultiMeshData* multimesh = nullptr;
        std::uint32_t multimeshMode = 0;
        std::uint32_t multimeshInstances = 1;
        std::uint32_t transformBytes = 0;
        if (instance.multimesh.valid())
        {
            multimesh = state.multimeshes.get(instance.multimesh);
            if (!multimesh || !multimesh->buffer.valid()) return;
            multimeshInstances = MultiMeshStorage::instancesToDraw(*multimesh);
            if (multimeshInstances == 0) return;
            multimeshMode = State::multimeshModeFor(*multimesh);
            transformBytes = multimesh->instances * multimesh->strideCache *
                             static_cast<std::uint32_t>(sizeof(float));
            multimeshInstancesDrawn += multimeshInstances;
        }
        const std::size_t firstSurface = only == State::kWholeInstance ? 0 : only;
        const std::size_t lastSurface =
                only == State::kWholeInstance ? mesh.surfaces.size() : only + 1;
        for (std::size_t s = firstSurface; s < lastSurface; ++s)
        {
            const SurfaceInfo& surface = mesh.surfaces[s];
            if (!surface.ready || surface.vertexCount == 0) continue;
            if (State::surfaceHidden(instance, s)) continue;
            RenderItem item;
            item.surface = &surface;
            item.vertexBuffer = surface.vertexBuffer;
            item.indexBuffer = surface.indexBuffer;
            item.indexCount = surface.indexCount;
            item.lod = State::surfaceLod(surface, instance, bounds, lodView);
            if (item.lod > 0)
            {
                item.indexBuffer = surface.lods[item.lod - 1].indexBuffer;
                item.indexCount = surface.lods[item.lod - 1].indexCount;
                ++lodSurfaces;
            }
            if (instance.meshInstance && s < instance.meshInstance->surfaces.size() &&
                    instance.meshInstance->surfaces[s].vertexBuffer.valid())
            {
                item.vertexBuffer = instance.meshInstance->surfaces[s].vertexBuffer;
                item.skinned = true;
            }
            item.transform = instance.transform;
            item.color = instance.color;
            item.layerMask = instance.layerMask;
            item.reflectorSlot = 0;
            if (instance.reflector.valid())
                for (std::uint32_t ri = 0; ri < activeReflectors; ++ri)
                    if (state.activeReflectors[ri] == instance.reflector)
                        item.reflectorSlot = ri + 1;
            item.material = State::resolveMaterial(instance, &mesh, static_cast<std::uint32_t>(s),
                    mMaterials);
            const MaterialInfo* material = mMaterials.get(item.material);
            item.variant = sceneVariant(material, surface, mTextures);
            item.billboard = sceneBillboard(material);
            if (item.billboard != 0 || multimeshMode != 0) item.variant &= ~64u;
            if (item.billboard == 4) item.variant &= 1u;
            if (item.billboard == 6)
            {
                if (viewport.reflection) continue;
                item.variant = 0;
            }
            if (instance.lightmap.valid() && state.lightmapsUsed > 0 && multimeshMode == 0 &&
                    item.billboard == 0 && (item.variant & (32u | 64u)) == 0 &&
                    (surface.attributeMask & kVertexUv2) != 0)
            {
                for (std::uint32_t l = 0; l < state.lightmapsUsed; ++l)
                {
                    if (state.lightmapIds[l] != instance.lightmap) continue;
                    item.lightmap = true;
                    item.lightmapIndex = l;
                    item.giOffset = (static_cast<std::uint32_t>(instance.lightmapSlice) << 16) | l;
                    item.lightmapFlags = 1u << 8;
                    if (state.lightmapHasSh[l]) item.lightmapFlags |= 1u << 9;
                    item.lightmapUvScale[0] = instance.lightmapUvScale.x;
                    item.lightmapUvScale[1] = instance.lightmapUvScale.y;
                    item.lightmapUvScale[2] = instance.lightmapUvScale.z;
                    item.lightmapUvScale[3] = instance.lightmapUvScale.w;
                    ++mStats.lightmapSurfaces;
                    break;
                }
            }
            else if (instance.lightmap.valid() && state.lightmapsUsed > 0 && multimeshMode == 0 &&
                     item.billboard == 0 && (item.variant & 64u) != 0 &&
                     !state.lightmapWarnedDetail)
            {
                state.lightmapWarnedDetail = true;
                log_warn("render: a surface with a detail texture does not use its lightmap");
            }
            else if (!instance.lightmap.valid() && state.lightmapsUsed > 0 && multimeshMode == 0 &&
                     item.billboard == 0 && (item.variant & (32u | 64u)) == 0)
            {
                float capture[36];
                std::uint32_t slot = 0;
                if (state.tapCapture(bounds.Center(), slot, capture))
                {
                    item.lightmap = true;
                    item.lightmapIndex = slot;
                    item.giOffset = static_cast<std::uint32_t>(state.captureData.size() / 36);
                    item.lightmapFlags = 1u << 7;
                    for (int c = 0; c < 36; ++c) state.captureData.push_back(capture[c]);
                }
            }
            bool blended = false;
            if (material)
            {
                item.cull = material->cull;
                item.blend = material->blend;
                item.depthTest = material->depthTest;
                blended = material->transparency == MaterialTransparency::AlphaBlend ||
                          material->blend != MaterialBlend::Mix || material->refraction != 0.0f ||
                          item.billboard == 6;
                if (material->refraction != 0.0f || item.billboard == 6) usedScreenTexture = true;
                item.transparent = blended;
                item.coverage = viewport.samples > 1 && !blended &&
                                material->transparency == MaterialTransparency::AlphaScissor &&
                                material->alphaAntialiasing != MaterialAlphaAntialiasing::Off &&
                                (item.variant & 16u) != 0 && item.billboard != 4 && !item.lightmap;
                if (item.coverage) item.transparent = true;
            }
            const bool materialBlended = blended;
            if (instance.transparency > 0.0f)
            {
                blended = true;
                item.transparent = true;
                item.coverage = false;
                item.fade = 1.0f - instance.transparency;
                item.fadeDepth = !materialBlended && item.depthTest && item.billboard == 0;
                if (item.fadeDepth) item.variant &= ~16u;
            }
            item.prepass =
                    prepassWanted && !blended && item.depthTest &&
                    (item.billboard == 0 || (item.billboard == 5 && (item.variant & 16u) != 0));
            item.multimeshMode = multimeshMode;
            if (multimesh)
            {
                item.dissolve = multimesh->dissolve && multimesh->usesCustomData;
                item.transforms = multimesh->buffer;
                item.transformBytes = transformBytes;
                item.instanceCount = multimeshInstances;
            }
            item.noShadows = material && !material->receiveShadows &&
                             material->shading != MaterialShading::Unshaded;
            item.pipelineKey =
                    State::pipelineKey(surface, item.variant, item.cull, item.transparent,
                            item.blend, item.depthTest, multimeshMode, item.billboard) |
                    (item.noShadows ? State::kNoShadowKey : 0ull) |
                    (item.coverage ? State::kCoverageKey : 0ull) |
                    (item.lightmap ? State::kLightmapKey : 0ull);
            const std::uint64_t materialId = item.material.bits();
            item.sortKey1 = (static_cast<std::uint64_t>(item.lod) & 0xffu) |
                            ((static_cast<std::uint64_t>(s) & 0xffu) << 16) |
                            (static_cast<std::uint64_t>(
                                     multimesh ? instance.multimesh.bits() : instance.base.bits())
                                    << 24) |
                            ((materialId >> 24) << 56);
            item.sortKey2 =
                    (materialId & 0xffffffu) | (static_cast<std::uint64_t>(item.pipelineKey) << 24);
            if (item.transparent)
            {
                const Math::Vec3 center =
                        (multimesh ? MultiMeshStorage::effectiveAabb(*multimesh) : surface.aabb)
                                .Transformed(instance.transform)
                                .Center();
                const Math::Vec4 inView = view * Math::Vec4(center.x, center.y, center.z, 1.0f);
                const float depth = inView.z < 0.0f ? -inView.z : 0.0f;
                std::uint32_t depthBits = 0;
                memcpy(&depthBits, &depth, sizeof(depthBits));
                item.sortKey2 = (1ull << 63) | (0xffffffffu - depthBits);
            }
            state.items.push_back(item);
        }
    };

    const Math::Vec3 cameraPosition = camera->transform.GetTranslation();
    auto hiddenByRange = [&](Instance& instance, std::uint32_t surface, const Math::Box& bounds)
    {
        if (!State::hasVisibilityRange(instance)) return false;
        const std::uint64_t mask = state.acquireRangeMask(viewport, *scenario);
        if (State::visibilityRangeCheck(instance, surface, mask, cameraPosition, bounds) == 0)
            return false;
        ++mStats.visibilityRangeHidden;
        return true;
    };

    bool treeCulling = false;
    if (state.cullingTree)
    {
        {
            ProfileScope scope("Cull tree");
            if (state.updateTree(*scenario, mMeshes))
            {
                mStats.cullEntries += static_cast<std::uint32_t>(scenario->entries.size());
                mStats.cullNodes += scenario->tree->nodeCount();
                if (scenario->tree->depth() > mStats.cullDepth)
                    mStats.cullDepth = scenario->tree->depth();
                const float quality = scenario->tree->quality();
                if (quality > mStats.cullQuality) mStats.cullQuality = quality;
                if (scenario->buildPending) mStats.cullRebuildPending = true;
            }
        }
        treeCulling = !scenario->entries.empty();
    }

    if (treeCulling)
    {
        std::uint32_t accepted = 0;
        std::uint32_t acceptedInstances = 0;
        {
            ProfileScope scope("Cull tree");
            scenario->tree->queryCandidates(frustum, scenario->candidates);
            ct::sort(scenario->candidates.begin(), scenario->candidates.end());
            mStats.cullNodesVisited += scenario->tree->lastQueryStats().nodesVisited;
            mStats.cullCandidates += static_cast<std::uint32_t>(scenario->candidates.size());
            for (std::size_t i = 0; i < scenario->candidates.size(); ++i)
            {
                const std::uint32_t entry = scenario->candidates[i];
                Instance* instance = find(state.instances, scenario->entries[entry]);
                if (!instance || !instance->visible ||
                        (instance->layerMask & camera->cullMask) == 0)
                    continue;
                if (!frustum.IntersectsBox(scenario->bounds[entry])) continue;
                if (hiddenByRange(*instance, scenario->entrySurfaces[entry],
                            scenario->bounds[entry]))
                    continue;
                if (accepted == 0 || scenario->entries[scenario->candidates[accepted - 1]] !=
                                             scenario->entries[entry])
                    ++acceptedInstances;
                scenario->candidates[accepted++] = entry;
            }
            scenario->candidates.resize(accepted);
        }
        mStats.cullAccepted += acceptedInstances;
        std::uint32_t eligible = 0;
        for (const auto& entry: scenario->maskCounts)
        {
            if ((entry.key & camera->cullMask) != 0) eligible += entry.value;
        }
        if (eligible > acceptedInstances) mStats.instancesCulled += eligible - acceptedInstances;
        for (std::size_t i = 0; i < scenario->candidates.size(); ++i)
        {
            const std::uint32_t entry = scenario->candidates[i];
            const Instance* instance = find(state.instances, scenario->entries[entry]);
            const MeshInfo* mesh = instance ? state.instanceMesh(mMeshes, *instance) : nullptr;
            if (!mesh) continue;
            if (i == 0 ||
                    scenario->entries[scenario->candidates[i - 1]] != scenario->entries[entry])
                ++instancesDrawn;
            addItems(*instance, *mesh, scenario->entrySurfaces[entry], scenario->bounds[entry]);
        }
    }
    else
    {
        for (std::size_t i = 0; i < scenario->instances.size(); ++i)
        {
            Instance* instance = find(state.instances, scenario->instances[i]);
            if (!instance || !instance->visible || (instance->layerMask & camera->cullMask) == 0)
                continue;
            if (!state.instanceUsable(mMeshes, *instance)) continue;
            const MeshInfo* mesh = state.instanceMesh(mMeshes, *instance);
            const Math::Box instanceBox = state.instanceBounds(*instance, *mesh);
            if (!frustum.IntersectsBox(instanceBox))
            {
                ++mStats.instancesCulled;
                continue;
            }
            if (!State::splitsSurfaces(*instance, *mesh))
            {
                if (hiddenByRange(*instance, State::kWholeInstance, instanceBox))
                {
                    ++mStats.instancesCulled;
                    continue;
                }
                ++instancesDrawn;
                addItems(*instance, *mesh, State::kWholeInstance, instanceBox);
                continue;
            }
            bool any = false;
            for (std::uint32_t s = 0; s < mesh->surfaces.size(); ++s)
            {
                const Math::Box surfaceBox = state.entryBounds(*instance, *mesh, s);
                if (!frustum.IntersectsBox(surfaceBox)) continue;
                if (hiddenByRange(*instance, s, surfaceBox)) continue;
                addItems(*instance, *mesh, s, surfaceBox);
                any = true;
            }
            if (any) ++instancesDrawn;
            else
                ++mStats.instancesCulled;
        }
    }

    const std::uint32_t itemCount = static_cast<std::uint32_t>(state.items.size());
    state.order.resize(itemCount);
    for (std::uint32_t i = 0; i < itemCount; ++i)
    {
        state.order[i].key2 = state.items[i].sortKey2;
        state.order[i].key1 = state.items[i].sortKey1;
        state.order[i].index = i;
    }
    ct::stable_sort(state.order.begin(), state.order.end(), sortEntryLess);

    state.runs.clear();
    for (std::uint32_t i = 0; i < itemCount; ++i)
    {
        const RenderItem& item = state.items[state.order[i].index];
        if (!state.runs.empty())
        {
            DrawRun& last = state.runs.back();
            const RenderItem& first = state.items[state.order[last.first].index];
            if (!item.skinned && !first.skinned && item.multimeshMode == 0 &&
                    first.multimeshMode == 0 && !item.transparent && !first.transparent &&
                    last.count < kMaxRepeats && item.sortKey1 == first.sortKey1 &&
                    item.sortKey2 == first.sortKey2 && sameColor(item.color, first.color) &&
                    item.lightmap == first.lightmap &&
                    (!item.lightmap || item.lightmapIndex == first.lightmapIndex))
            {
                ++last.count;
                continue;
            }
        }
        DrawRun run;
        run.first = i;
        run.count = 1;
        state.runs.push_back(run);
    }

    const std::uint32_t stride = alignUp(kBlockSize, mDriver->caps().uniformBufferOffsetAlignment);
    const std::uint32_t materialStride =
            alignUp(static_cast<std::uint32_t>(sizeof(MaterialDataGpu)),
                    mDriver->caps().uniformBufferOffsetAlignment);
    const std::uint32_t uniformSize = static_cast<std::uint32_t>(state.runs.size()) * stride;
    const std::uint32_t instanceSize =
            itemCount * static_cast<std::uint32_t>(sizeof(InstanceDataGpu));
    if (itemCount > 0)
    {
        if (!state.ensureUniformBuffer(mDriver, uniformSize) ||
                !state.ensureInstanceBuffer(mDriver, instanceSize))
        {
            if (prepareScope) profiler->end();
            return;
        }
        state.uniformBytes.resize(uniformSize);
        state.instanceData.resize(itemCount);
        for (std::uint32_t i = 0; i < itemCount; ++i)
        {
            const RenderItem& item = state.items[state.order[i].index];
            InstanceDataGpu& data = state.instanceData[i];
            storeTransform3x4(item.transform, data.transform);
            memcpy(data.prevTransform, data.transform, sizeof(data.transform));
            data.compressedAabbPosition[0] = 0.0f;
            data.compressedAabbPosition[1] = 0.0f;
            data.compressedAabbPosition[2] = 0.0f;
            data.compressedAabbPosition[3] = 0.0f;
            data.compressedAabbSize[0] = 1.0f;
            data.compressedAabbSize[1] = 1.0f;
            data.compressedAabbSize[2] = 1.0f;
            data.materialFeedbackIndex = 0xffffffffu;
            memset(data.uvScale, 0, sizeof(data.uvScale));
            data.flags = static_cast<std::uint32_t>(item.fade * 255.0f + 0.5f) << 24;
            if ((item.variant & 64u) && !(item.surface->attributeMask & kVertexUv2))
                data.flags |= 1u << 12;
            if (item.dissolve) data.flags |= 1u << 13;
            data.instanceUniformsOfs = 0;
            data.giOffset = item.giOffset;
            data.flags |= item.lightmapFlags;
            data.layerMask = item.layerMask;
            memcpy(data.lightmapUvScale, item.lightmapUvScale, sizeof(data.lightmapUvScale));
        }
        for (std::size_t r = 0; r < state.runs.size(); ++r)
        {
            const RenderItem& item = state.items[state.order[state.runs[r].first].index];
            DrawCallData draw;
            draw.instanceIndex = state.runs[r].first;
            draw.pad[0] = 0;
            draw.pad[1] = 0;
            draw.pad[2] = 0;
            draw.color[0] = srgbToLinear(item.color.x);
            draw.color[1] = srgbToLinear(item.color.y);
            draw.color[2] = srgbToLinear(item.color.z);
            draw.color[3] = item.color.w;
            draw.planar[0] = item.reflectorSlot;
            memcpy(state.uniformBytes.data() + r * stride, &draw, sizeof(draw));
        }
        mDriver->updateBuffer(state.uniformBuffer, 0, state.uniformBytes.data(), uniformSize);

        state.materialSlots.clear();
        std::uint32_t materialCount = 0;
        for (std::size_t r = 0; r < state.runs.size(); ++r)
        {
            const std::uint32_t bits =
                    state.items[state.order[state.runs[r].first].index].material.bits();
            if (!state.materialSlots.find(bits)) state.materialSlots[bits] = materialCount++;
        }
        const std::uint32_t materialSize = materialCount * materialStride;
        if (!state.ensureMaterialBuffer(mDriver, materialSize))
        {
            if (prepareScope) profiler->end();
            return;
        }
        state.materialBytes.resize(materialSize);
        for (auto& entry: state.materialSlots)
        {
            MaterialDataGpu data;
            fillMaterial(mMaterials.get(MaterialId::from_bits(entry.key)), data, &mTextures);
            memcpy(state.materialBytes.data() + entry.value * materialStride, &data, sizeof(data));
        }
        mDriver->updateBuffer(state.materialBuffer, 0, state.materialBytes.data(), materialSize);
        mDriver->updateBuffer(state.instanceBuffer, 0, state.instanceData.data(), instanceSize);
        if (!state.captureData.empty())
        {
            const std::uint32_t bytes =
                    static_cast<std::uint32_t>(state.captureData.size() * sizeof(float));
            if (State::ensureStorage(mDriver, state.lightmapCaptureBuffer,
                        state.lightmapCaptureSize, bytes, "lightmap captures"))
                mDriver->updateBuffer(state.lightmapCaptureBuffer, 0, state.captureData.data(),
                        bytes);
        }
    }

    const bool useEnvironment = scenario->hasEnvironment && !debug;
    float clearLinear[3] = { srgbToLinear(viewport.clearColor.x),
        srgbToLinear(viewport.clearColor.y), srgbToLinear(viewport.clearColor.z) };
    if (useEnvironment && scenario->environment.background == BackgroundMode::Color)
    {
        const EnvironmentSettings& env = scenario->environment;
        clearLinear[0] = srgbToLinear(env.backgroundColor.x);
        clearLinear[1] = srgbToLinear(env.backgroundColor.y);
        clearLinear[2] = srgbToLinear(env.backgroundColor.z);
    }
    const bool skyBackground =
            useEnvironment && scenario->environment.background == BackgroundMode::Sky;
    if (skyBackground) clearLinear[0] = clearLinear[1] = clearLinear[2] = 0.0f;
    const float clearEnergy =
            useEnvironment && scenario->environment.background == BackgroundMode::Color
                    ? scenario->environment.backgroundEnergy
                    : 1.0f;
    const EnvironmentSettings* environment = useEnvironment ? &scenario->environment : nullptr;
    const bool ambientSky =
            environment != nullptr &&
            ((environment->ambientSource == AmbientSource::Background && skyBackground) ||
                    environment->ambientSource == AmbientSource::Sky);
    const bool reflectionSky =
            environment != nullptr &&
            ((environment->reflectionSource == ReflectionSource::Background && skyBackground) ||
                    environment->reflectionSource == ReflectionSource::Sky);
    bool ssaoWanted = false;
    bool ssrWanted = false;
    const bool screenSpaceCandidate =
            environment != nullptr && !viewport.reflection && prepassWanted && !state.items.empty();
    const bool ssaoRequested = screenSpaceCandidate && environment->ssaoEnabled &&
                               (profiler == nullptr || profiler->passEnabled("SSAO"));
    bool ssrRequested = screenSpaceCandidate && environment->ssrEnabled &&
                        (profiler == nullptr || profiler->passEnabled("SSR"));
    if (ssrRequested && viewport.transparent)
    {
        ssrRequested = false;
        if (!state.screenSpaceWarnedSsrTransparent)
        {
            state.screenSpaceWarnedSsrTransparent = true;
            log_warn("render: screen space reflections are skipped in a viewport with a "
                     "transparent background");
        }
    }
    if (ssaoRequested || ssrRequested)
    {
        if (!state.screenSpaceUsable())
        {
            if (ssaoRequested && !state.screenSpaceWarned)
            {
                state.screenSpaceWarned = true;
                log_warn("render: ambient occlusion is skipped, this driver has no compute "
                         "shaders");
            }
            if (ssrRequested && !state.screenSpaceWarnedSsr)
            {
                state.screenSpaceWarnedSsr = true;
                log_warn("render: screen space reflections are skipped, this driver has no "
                         "compute shaders");
            }
        }
        else if (viewport.samples > 1)
        {
            if (ssaoRequested && !state.screenSpaceWarnedMsaa)
            {
                state.screenSpaceWarnedMsaa = true;
                log_warn("render: ambient occlusion is skipped while multisampling is on");
            }
            if (ssrRequested && !state.screenSpaceWarnedSsrMsaa)
            {
                state.screenSpaceWarnedSsrMsaa = true;
                log_warn("render: screen space reflections are skipped while multisampling is "
                         "on");
            }
        }
        else
        {
            for (const RenderItem& item: state.items)
            {
                if (item.prepass && item.billboard == 0 && !item.coverage)
                {
                    ssaoWanted = ssaoRequested;
                    ssrWanted = ssrRequested;
                    break;
                }
            }
        }
    }
    bool normalRoughnessWanted = ssaoWanted || ssrWanted;
    if (normalRoughnessWanted)
        normalRoughnessWanted = State::ensureNormalRoughness(mDriver, viewport, width, height) &&
                                state.screenSpace.ensureBestFitNormal();
    if (!normalRoughnessWanted)
    {
        ssaoWanted = false;
        ssrWanted = false;
        State::destroyNormalRoughness(mDriver, viewport);
    }
    bool ssaoCreated = false;
    if (ssaoWanted)
        ssaoWanted = state.screenSpace.allocateSsao(viewport.ssao, width, height, ssaoCreated);
    if (!ssaoWanted) ScreenSpaceEffects::releaseSsao(mDriver, viewport.ssao);
    else if (ssaoCreated)
        ++mStats.ssaoTargetsCreated;
    bool ssrCreated = false;
    if (ssrWanted)
        ssrWanted = state.screenSpace.allocateSsr(viewport.ssr, width, height, ssrCreated);
    if (!ssrWanted) ScreenSpaceEffects::releaseSsr(mDriver, viewport.ssr);
    else if (ssrCreated)
        ++mStats.ssrTargetsCreated;
    const bool fogEnabled = environment != nullptr && environment->fogEnabled;
    const bool fogAerial = fogEnabled && environment->fogAerialPerspective > 0.0f;
    const bool fogFlatSky = fogEnabled && !skyBackground && !viewport.transparent &&
                            environment->fogSkyAffect > 0.0f;
    state.ensureOctmap(mDriver);
    const bool radianceReady = (skyBackground || ambientSky || reflectionSky || fogAerial) &&
                               state.prepareRadiance(mDriver, *scenario);
    state.ensureShadowResources(mDriver);
    State::ShadowStats shadowStats;
    if (debug)
    {
        state.shadowPasses.clear();
        state.shadowRuns.clear();
        state.directionalSoftShadows = false;
        state.directionalShadowsActive = false;
        state.positionalShadowsActive = false;
        state.activePositionalAtlas = prisma::TextureHandle();
    }
    else
        shadowStats = state.prepareShadows(mDriver, mMeshes, mMaterials, mTextures, viewport,
                viewport.scenario, *scenario, *camera, aspect, frustum, shadowLodView);
    mStats.shadowCascades += shadowStats.cascades;
    mStats.shadowLights += shadowStats.lights;
    mStats.shadowPasses += shadowStats.passes;
    mStats.shadowCasters += shadowStats.casters;
    mStats.shadowDrawCalls += shadowStats.drawCalls;
    mStats.shadowTriangles += shadowStats.triangles;
    ClusterBuilder* cluster = nullptr;
    if (state.clusterShared.ready())
    {
        if (!viewport.cluster)
        {
            viewport.cluster = new ClusterBuilder();
            viewport.cluster->setShared(&state.clusterShared);
        }
        cluster = viewport.cluster;
        if ((cluster->getScreenWidth() != width || cluster->getScreenHeight() != height) &&
                !cluster->setup(width, height, kMaxClusterElements))
            cluster = nullptr;
    }
    if (cluster) cluster->begin(camera->transform, baseProjection, false);
    const std::uint32_t directionalCount = state.uploadLights(mDriver, viewport.scenario, view,
            camera->transform, frustum, cluster, debug);
    mStats.lightsDrawn +=
            static_cast<std::uint32_t>(state.lightCounts.omni + state.lightCounts.spot) +
            directionalCount;
    mStats.decalsDrawn += state.uploadDecals(mDriver, viewport.scenario, camera->transform, frustum,
            cluster, debug);
    mStats.reflectionProbesDrawn += state.uploadReflectionProbes(mDriver, viewport.scenario,
            camera->transform, cluster, debug);
    ImplementationDataGpu implementation;
    memset(&implementation, 0, sizeof(implementation));
    bool implementationReady = false;
    if (cluster)
    {
        const bool clusterScope = profiler != nullptr && profiler->beginGpu("Cluster");
        cluster->bake();
        if (clusterScope) profiler->endGpu();

        implementation.clusterShift = cluster->getClusterShift();
        implementation.clusterWidth = cluster->getClusterWidth();
        implementation.clusterTypeSize = cluster->getClusterTypeSize();
        implementation.maxClusterElementCountDiv32 = cluster->getMaxClusterElementCountDiv32();
        implementationReady = true;
    }

    SceneUboData sceneData;
    memset(&sceneData, 0, sizeof(sceneData));
    {
        SceneUbo& ubo = sceneData.data;
        const Math::Mat4 projection = zeroToOneDepth() * baseProjection;
        storeMatrix(projection, ubo.projectionMatrix);
        storeMatrix(projection.Inverse(), ubo.invProjectionMatrix);
        storeTransform3x4(camera->transform, ubo.invViewMatrix);
        storeTransform3x4(view, ubo.viewMatrix);
        storeMatrix(projection, ubo.projectionMatrixView[0]);
        storeMatrix(projection.Inverse(), ubo.invProjectionMatrixView[0]);
        storeMatrix(camera->transform, ubo.mainCamInvViewMatrix);
        ubo.viewportSize[0] = static_cast<float>(width);
        ubo.viewportSize[1] = static_cast<float>(height);
        ubo.screenPixelSize[0] = 1.0f / static_cast<float>(width);
        ubo.screenPixelSize[1] = 1.0f / static_cast<float>(height);
        ubo.zFar = camera->zFar;
        ubo.zNear = camera->zNear;
        ubo.cameraVisibleLayers = camera->cullMask;
        ubo.passAlphaMultiplier = 1.0f;
        ubo.emissiveExposureNormalization = 1.0f;
        ubo.iblExposureNormalization = 1.0f;
        ubo.time =
                static_cast<float>(static_cast<double>(time_nanos() - state.startNanos) * 1.0e-9);
        ubo.directionalLightCount = directionalCount;
        ubo.packedConstants = state.packedConstants();
        if (state.lightmapSpecularFrame) ubo.flags |= kSceneFlagUseLightmapSpecular;
        ubo.directionalShadowPixelSize[0] = ubo.directionalShadowPixelSize[1] =
                1.0f / static_cast<float>(state.shadowAtlasSize);
        if (viewport.shadowAtlas.getSize() > 0)
            ubo.shadowAtlasPixelSize[0] = ubo.shadowAtlasPixelSize[1] =
                    1.0f / static_cast<float>(viewport.shadowAtlas.getSize());
        memcpy(ubo.penumbraShadowKernel, state.positionalPenumbraKernel,
                sizeof(ubo.penumbraShadowKernel));
        memcpy(ubo.softShadowKernel, state.positionalSoftKernel, sizeof(ubo.softShadowKernel));
        memcpy(ubo.directionalPenumbraShadowKernel, state.directionalPenumbraKernel,
                sizeof(ubo.directionalPenumbraShadowKernel));
        memcpy(ubo.directionalSoftShadowKernel, state.directionalSoftKernel,
                sizeof(ubo.directionalSoftShadowKernel));
        if (state.roughnessLimiter)
        {
            ubo.flags |= kSceneFlagUseRoughnessLimiter;
            ubo.roughnessLimiterAmount = state.roughnessLimiterAmount;
            ubo.roughnessLimiterLimit = state.roughnessLimiterLimit;
        }
        float ambient[3] = { clearLinear[0], clearLinear[1], clearLinear[2] };
        ubo.ambientLightColorEnergy[3] = 1.0f;
        if (debug)
        {
            ubo.flags |= kSceneFlagUseAmbientLight;
            ambient[0] = ambient[1] = ambient[2] = 1.0f;
            if (debugDraw == ViewportDebugDraw::NormalBuffer) ubo.flags |= kSceneFlagDebugNormal;
        }
        else if (!scenario->hasEnvironment)
            ubo.flags |= kSceneFlagUseAmbientLight;
        else
        {
            const EnvironmentSettings& env = scenario->environment;
            ubo.ambientLightColorEnergy[3] = env.backgroundEnergy;
            if (env.ambientSource == AmbientSource::Background && !skyBackground)
            {
                const float energy = env.backgroundEnergy;
                ambient[0] = clearLinear[0] * energy;
                ambient[1] = clearLinear[1] * energy;
                ambient[2] = clearLinear[2] * energy;
                ubo.flags |= kSceneFlagUseAmbientLight;
            }
            else
            {
                const float energy = env.ambientEnergy;
                ambient[0] = srgbToLinear(env.ambientColor.x) * energy;
                ambient[1] = srgbToLinear(env.ambientColor.y) * energy;
                ambient[2] = srgbToLinear(env.ambientColor.z) * energy;
                if (env.ambientSource == AmbientSource::Color)
                    ubo.flags |= kSceneFlagUseAmbientLight;
            }
            ubo.ambientColorSkyMix = env.ambientSkyContribution;
            if (env.fogEnabled)
            {
                ubo.flags |= kSceneFlagUseFog;
                ubo.fogDensity = env.fogDensity;
                ubo.fogHeight = env.fogHeight;
                ubo.fogHeightDensity = env.fogHeightDensity;
                ubo.fogAerialPerspective = env.fogAerialPerspective;
                ubo.fogDepthCurve = env.fogDepthCurve;
                ubo.fogDepthEnd = env.fogDepthEnd > 0.0f ? env.fogDepthEnd : ubo.zFar;
                ubo.fogDepthBegin = fminf(env.fogDepthBegin, ubo.fogDepthEnd - 0.001f);
                ubo.fogLightColor[0] = srgbToLinear(env.fogLightColor.x) * env.fogLightEnergy;
                ubo.fogLightColor[1] = srgbToLinear(env.fogLightColor.y) * env.fogLightEnergy;
                ubo.fogLightColor[2] = srgbToLinear(env.fogLightColor.z) * env.fogLightEnergy;
                ubo.fogSunScatter = env.fogSunScatter;
                if (env.fogMode == FogMode::Depth) ubo.packedConstants |= kSceneConstantDepthFog;
            }
            if (ambientSky && radianceReady)
                ubo.flags |= kSceneFlagUseAmbientCubemap | kSceneFlagUseAmbientLight;
        }
        if (useEnvironment && !skyBackground &&
                scenario->environment.reflectionSource == ReflectionSource::Background)
        {
            ubo.flags |= kSceneFlagUseReflectionColor;
            const float energy = scenario->environment.backgroundEnergy;
            ubo.reflectionColor[0] = clearLinear[0] * energy;
            ubo.reflectionColor[1] = clearLinear[1] * energy;
            ubo.reflectionColor[2] = clearLinear[2] * energy;
        }
        else if (reflectionSky && radianceReady)
            ubo.flags |= kSceneFlagUseReflectionCubemap;
        if (radianceReady)
        {
            ubo.radiancePixelSize = 1.0f / static_cast<float>(scenario->radiance.size);
            ubo.radianceBorderSize = scenario->radiance.border;
            for (int c = 0; c < 3; ++c)
            {
                const Math::Vec4& column =
                        c == 0 ? camera->transform.col0
                               : (c == 1 ? camera->transform.col1 : camera->transform.col2);
                ubo.radianceInverseXform[c * 4 + 0] = column.x;
                ubo.radianceInverseXform[c * 4 + 1] = column.y;
                ubo.radianceInverseXform[c * 4 + 2] = column.z;
            }
        }
        ubo.ambientLightColorEnergy[0] = ambient[0];
        ubo.ambientLightColorEnergy[1] = ambient[1];
        ubo.ambientLightColorEnergy[2] = ambient[2];
        sceneData.prev = ubo;
    }
    if (state.sceneBuffer.valid())
        mDriver->updateBuffer(state.sceneBuffer, 0, &sceneData, sizeof(sceneData));
    std::uint32_t coverageItems = 0;
    bool coveragePrepass = false;
    for (const RenderItem& item: state.items)
    {
        if (!item.coverage) continue;
        ++coverageItems;
        if (item.prepass) coveragePrepass = true;
    }
    coveragePrepass = coveragePrepass && state.prepassSceneBuffer.valid();
    if (coveragePrepass)
    {
        SceneUboData prepassData = sceneData;
        prepassData.data.opaquePrepassThreshold = 0.99f;
        prepassData.prev.opaquePrepassThreshold = 0.99f;
        mDriver->updateBuffer(state.prepassSceneBuffer, 0, &prepassData, sizeof(prepassData));
    }

    TonemapParams tonemap;
    memset(&tonemap, 0, sizeof(tonemap));
    tonemap.bcs[0] = 1.0f;
    tonemap.bcs[1] = 1.0f;
    tonemap.bcs[2] = 1.0f;
    tonemap.flags = kTonemapFlagConvertToSrgb;
    tonemap.pixelSize[0] = 1.0f / static_cast<float>(width);
    tonemap.pixelSize[1] = 1.0f / static_cast<float>(height);
    tonemap.outputMaxValue = 1.0f;
    if (!toScreen && !(viewport.srgbOutput && debugDraw != ViewportDebugDraw::NormalBuffer))
        tonemap.flags = 0;
    const bool smaaOn = viewport.smaa && toScreen && !debug && state.smaaReady();
    if (viewport.fxaa && !smaaOn && !viewport.reflection && !debug)
        tonemap.flags |= kTonemapFlagUseFxaa;
    tonemap.exposure = useEnvironment ? scenario->environment.tonemapExposure : 1.0f;
    const TonemapMode tonemapMode =
            useEnvironment ? scenario->environment.tonemapMode : TonemapMode::Linear;
    const float tonemapWhite = useEnvironment ? scenario->environment.tonemapWhite : 1.0f;
    const float agxContrast = useEnvironment ? scenario->environment.tonemapAgxContrast : 1.25f;
    tonemap.tonemapper = static_cast<std::uint32_t>(tonemapMode);
    tonemap.white = tonemapWhiteFor(tonemapMode, tonemapWhite, tonemap.outputMaxValue);
    tonemapParameters(tonemapMode, tonemap.white, agxContrast, tonemap.outputMaxValue,
            tonemap.tonemapperParams);
    tonemap.autoExposureScale = 0.5f;
    tonemap.luminanceMultiplier = 1.0f;
    GlowPass glowPasses[kGlowMaxPasses];
    std::uint32_t glowPassCount = 0;
    bool glowOn = false;
    prisma::TextureHandle glowMapTexture;
    if (useEnvironment && scenario->environment.glowEnabled && width >= 8 && height >= 8 &&
            (profiler == nullptr || profiler->passEnabled("Glow")))
    {
        const EnvironmentSettings& env = scenario->environment;
        float glowLevels[kGlowLevels] = {};
        glowOn = state.prepareGlow(mDriver, viewport, env, glowPasses, glowPassCount, glowLevels,
                mStats);
        if (glowOn)
        {
            tonemap.flags |= kTonemapFlagUseGlow;
            tonemap.glowIntensity =
                    env.glowBlendMode == GlowBlendMode::Mix ? env.glowMix : env.glowIntensity;
            tonemap.glowMode = static_cast<std::uint32_t>(env.glowBlendMode);
            for (std::uint32_t i = 0; i < kGlowLevels; ++i)
                tonemap.glowLevels[i][0] = glowLevels[i];
            tonemap.glowTextureSize[0] = viewport.glowWidth;
            tonemap.glowTextureSize[1] = viewport.glowHeight;
            glowMapTexture = mTextures.gpu(mTextures.white());
            if (mTextures.valid(env.glowMap))
            {
                tonemap.glowMapStrength = env.glowMapStrength;
                glowMapTexture = mTextures.gpu(env.glowMap);
            }
        }
    }
    prisma::TextureHandle colorCorrectionTexture;
    if (useEnvironment && scenario->environment.adjustmentEnabled)
    {
        const EnvironmentSettings& env = scenario->environment;
        tonemap.flags |= kTonemapFlagUseBcs;
        tonemap.bcs[0] =
                env.adjustmentBrightness < 0.04045f
                        ? env.adjustmentBrightness * (1.0f / 12.92f)
                        : powf((env.adjustmentBrightness + 0.055f) * (1.0f / 1.055f), 2.4f);
        tonemap.bcs[1] = env.adjustmentContrast;
        tonemap.bcs[2] = env.adjustmentSaturation;
        if (mTextures.valid(env.adjustmentColorCorrection) &&
                (tonemap.flags & kTonemapFlagConvertToSrgb) != 0)
        {
            tonemap.flags |= kTonemapFlagUseColorCorrection;
            colorCorrectionTexture = mTextures.gpu(env.adjustmentColorCorrection);
        }
    }
    if (state.tonemapParams.valid())
        mDriver->updateBuffer(state.tonemapParams, 0, &tonemap, sizeof(tonemap));
    // A 3D picture smaller than the window is tonemapped (glow, FXAA, grading) at its own size and
    // then scaled up by the upscale pass: one cheap pass at the window's size, with crisp edges,
    // where tonemapping straight to the window ran every effect for every window pixel.
    const bool upscaleWanted = toScreen && !debug && state.upscalePipeline.valid() &&
                               (width != outputWidth || height != outputHeight ||
                                       layout.x != 0.0f || layout.y != 0.0f || smaaOn);
    if (upscaleWanted)
    {
        UpscaleParams upscale = {};
        upscale.uvScale[0] = static_cast<float>(outputWidth) / layout.rectWidth;
        upscale.uvScale[1] = static_cast<float>(outputHeight) / layout.rectHeight;
        upscale.uvOffset[0] = -layout.x / layout.rectWidth;
        upscale.uvOffset[1] = -layout.y / layout.rectHeight;
        mDriver->updateBuffer(state.upscaleParams, 0, &upscale, sizeof(upscale));
    }

    if (prepareScope) profiler->end();

    prisma::Viewport area;
    area.width = static_cast<float>(width);
    area.height = static_cast<float>(height);
    prisma::Viewport outputArea;
    outputArea.width = static_cast<float>(outputWidth);
    outputArea.height = static_cast<float>(outputHeight);
    prisma::Viewport windowArea;
    windowArea.width = static_cast<float>(windowWidth);
    windowArea.height = static_cast<float>(windowHeight);

    bool skyReady = false;
    if (skyBackground || radianceReady)
    {
        const EnvironmentSettings& env = scenario->environment;
        scenario->skyMode = 0;
        scenario->skyGpu = prisma::TextureHandle();
        if (env.skyMode != SkyMode::Procedural)
        {
            const TextureInfo* source = mTextures.get(env.skyTexture);
            const bool cube = source != nullptr && source->type == prisma::TextureType::TextureCube;
            if (source != nullptr && cube == (env.skyMode == SkyMode::Cubemap))
            {
                scenario->skyMode = static_cast<std::uint32_t>(env.skyMode);
                scenario->skyGpu = source->gpu;
            }
        }
        skyReady = !skyBackground ||
                   state.ensureSky(mDriver, State::skyVariant(env, scenario->skyMode, false),
                           viewport.samples);
        if (skyReady && radianceReady)
            skyReady = state.ensureSky(mDriver, State::skyVariant(env, scenario->skyMode, true));
        if (skyReady)
        {
            state.uploadSky(mDriver, viewport.scenario, env, scenario->skyMode, *camera,
                    baseProjection,
                    static_cast<float>(
                            static_cast<double>(time_nanos() - state.startNanos) * 1.0e-9),
                    radianceReady ? scenario->radiance.border : 0.0f);
            if (radianceReady && (profiler == nullptr || profiler->passEnabled("Radiance")))
            {
                const bool radianceScope = profiler != nullptr && profiler->beginGpu("Radiance");
                state.runRadiance(mDriver, viewport.scenario, *scenario);
                if (radianceScope) profiler->endGpu();
            }
        }
    }

    bool flatSkyReady = false;
    if (fogFlatSky && state.ensureSky(mDriver, State::kSkyFlatVariant, viewport.samples))
    {
        const float flatColor[3] = { clearLinear[0] * clearEnergy, clearLinear[1] * clearEnergy,
            clearLinear[2] * clearEnergy };
        state.uploadFlatSky(mDriver, viewport.scenario, scenario->environment, *camera,
                baseProjection, flatColor);
        flatSkyReady = true;
    }

    const std::uint32_t batchSlot = 1 + (mStats.viewportsDrawn < 14 ? mStats.viewportsDrawn : 14);
    const bool drawBatch = (toScreen || viewport.drawBatch) && !debug;
    const std::uint32_t screenSlot =
            toScreen ? 0u : 16u + (mStats.viewportsDrawn < 15 ? mStats.viewportsDrawn : 15u);
    if (drawBatch && !state.batch.empty(BatchRenderer::Space::World))
        state.batch.setUniforms(batchSlot, zeroToOneDepth() * baseProjection * view, true);
    if (drawBatch && !state.batch.empty(BatchRenderer::Space::Screen))
        state.batch.setUniforms(screenSlot,
                BatchRenderer::screenProjection(
                        static_cast<float>(toScreen ? windowWidth : outputWidth),
                        static_cast<float>(toScreen ? windowHeight : outputHeight)),
                !toScreen && !viewport.outputLinear);

    state.generateDfg(mDriver);
    {
        const bool shadowScope = profiler != nullptr && !state.shadowPasses.empty() &&
                                 profiler->passEnabled("Shadows") && profiler->beginGpu("Shadows");
        if (profiler == nullptr || profiler->passEnabled("Shadows")) state.renderShadows(mDriver);
        if (shadowScope) profiler->endGpu();
    }

    bool impostorsPrepared = false;
    bool impostorsInPrepass = false;
    if (state.impostors.setCount() > 0 && drawItems)
    {
        const bool impostorScope = profiler != nullptr && profiler->beginGpu("Impostor");
        const Math::Vec4& eye = camera->transform.col3;
        impostorsPrepared = state.impostors.prepare(mDriver, viewport.scenario.bits(), frustum,
                                    Math::Vec3(eye.x, eye.y, eye.z)) > 0;
        if (impostorScope) profiler->endGpu();
    }

    auto bindVariantTextures = [&](const MaterialInfo* material, std::uint32_t variant, bool full)
    {
        if (!material || variant == 0) return;
        auto gpu = [&](MaterialTexture texture, bool white)
        {
            const TextureId id = sceneTexture(*material, texture);
            if (mTextures.valid(id)) return mTextures.gpu(id);
            return mTextures.gpu(white ? mTextures.white() : mTextures.black());
        };
        const prisma::SamplerHandle sampler = state.materialSampler;
        if (variant & 1u) mDriver->bindTexture(0, gpu(MaterialTexture::Albedo, true), sampler);
        if (variant & 2u)
        {
            if (full) mDriver->bindTexture(1, gpu(MaterialTexture::Metallic, true), sampler);
            mDriver->bindTexture(2, gpu(MaterialTexture::Roughness, true), sampler);
            if (full)
                mDriver->bindTexture(5, gpu(MaterialTexture::AmbientOcclusion, true), sampler);
        }
        if (full && (variant & 4u))
            mDriver->bindTexture(3, gpu(MaterialTexture::Emission, false), sampler);
        if (variant & 8u)
        {
            const TextureId normal = sceneTexture(*material, MaterialTexture::Normal);
            mDriver->bindTexture(4,
                    mTextures.gpu(mTextures.valid(normal) ? normal : mTextures.normal()), sampler);
        }
        if (variant & 64u)
        {
            mDriver->bindTexture(14, gpu(MaterialTexture::DetailAlbedo, true), sampler);
            if (variant & 8u)
            {
                const TextureId detailNormal =
                        sceneTexture(*material, MaterialTexture::DetailNormal);
                mDriver->bindTexture(15,
                        mTextures.gpu(
                                mTextures.valid(detailNormal) ? detailNormal : mTextures.normal()),
                        sampler);
            }
            mDriver->bindTexture(16, gpu(MaterialTexture::DetailMask, true), sampler);
        }
    };

    state.prepassDrawn.resize(state.runs.size());
    memset(state.prepassDrawn.data(), 0, state.prepassDrawn.size());
    std::uint32_t depthSurfaces = 0;
    std::uint32_t depthDrawCalls = 0;
    std::uint32_t depthTriangles = 0;
    std::uint32_t normalDrawCalls = 0;
    if (prepassWanted && !state.items.empty())
    {
        bool any = false;
        for (std::size_t r = 0; r < state.runs.size() && !any; ++r)
            any = state.items[state.order[state.runs[r].first].index].prepass;
        const bool depthScope = any && profiler != nullptr && profiler->beginGpu("Depth");
        if (any)
        {
            prisma::RenderPassDesc depthPass;
            depthPass.colorCount = 0;
            if (normalRoughnessWanted)
            {
                depthPass.colors[0].texture = viewport.normalRoughness;
                depthPass.colorCount = 1;
                depthPass.clearColor[0] = 0.0f;
                depthPass.clearColor[1] = 0.0f;
                depthPass.clearColor[2] = 0.0f;
                depthPass.clearColor[3] = 0.0f;
            }
            depthPass.depth.texture =
                    viewport.samples > 1 ? viewport.depthMsaa : viewport.depthTexture;
            depthPass.depthLoad = prisma::LoadOp::Clear;
            depthPass.depthStore = prisma::StoreOp::Store;
            mDriver->beginRenderPass(depthPass);
            mDriver->setViewport(area);
            const prisma::BufferHandle depthSceneBuffer =
                    coveragePrepass ? state.prepassSceneBuffer : state.sceneBuffer;
            if (depthSceneBuffer.valid())
                mDriver->bindUniformBuffer(0, depthSceneBuffer, 0, sizeof(SceneUboData));
            mDriver->bindStorageBuffer(0, state.instanceBuffer, 0, instanceSize);
            prisma::PipelineHandle depthBound;
            std::uint32_t boundDepthMaterial = 0xffffffffu;
            if (normalRoughnessWanted)
                mDriver->bindTexture(kBestFitSlot, state.screenSpace.bestFitNormal(),
                        state.nearestSampler);
            for (std::size_t r = 0; r < state.runs.size(); ++r)
            {
                const DrawRun& run = state.runs[r];
                const RenderItem& item = state.items[state.order[run.first].index];
                if (!item.prepass) continue;
                const bool scissor = (item.variant & 16u) != 0 || item.dissolve;
                if (normalRoughnessWanted)
                {
                    if (item.billboard != 0 || item.coverage) continue;
                    const std::uint32_t reduced =
                            State::normalRoughnessVariant(item.variant, scissor);
                    const prisma::PipelineHandle pipeline = state.normalRoughnessPipelineFor(
                            mDriver, *item.surface, reduced, item.cull, item.multimeshMode);
                    if (!pipeline.valid()) continue;
                    if (!depthBound.valid() || depthBound != pipeline)
                    {
                        mDriver->bindPipeline(pipeline);
                        depthBound = pipeline;
                        boundDepthMaterial = 0xffffffffu;
                    }
                    if (item.material.bits() != boundDepthMaterial)
                    {
                        boundDepthMaterial = item.material.bits();
                        const std::uint32_t* slot = state.materialSlots.find(boundDepthMaterial);
                        if (slot && state.materialBuffer.valid())
                            mDriver->bindUniformBuffer(2, state.materialBuffer,
                                    *slot * materialStride, sizeof(MaterialDataGpu));
                        bindVariantTextures(mMaterials.get(item.material), reduced, false);
                    }
                    mDriver->bindVertexBuffer(0, item.vertexBuffer, 0);
                    if (item.multimeshMode != 0)
                        mDriver->bindStorageBuffer(5, item.transforms, 0, item.transformBytes);
                    mDriver->bindUniformBuffer(1, state.uniformBuffer,
                            static_cast<std::uint32_t>(r) * stride, kBlockSize);
                    const std::uint32_t normalRepeats =
                            item.multimeshMode != 0 ? item.instanceCount : run.count;
                    if (item.indexCount > 0)
                    {
                        mDriver->bindIndexBuffer(item.indexBuffer);
                        mDriver->drawIndexed(item.indexCount, 0, normalRepeats);
                    }
                    else
                        mDriver->draw(item.surface->vertexCount, 0, normalRepeats);
                    state.prepassDrawn[r] = 1;
                    ++depthDrawCalls;
                    ++normalDrawCalls;
                    depthSurfaces += item.multimeshMode != 0 ? 1u : run.count;
                    depthTriangles += triangleCount(item.surface->topology,
                                              item.indexCount > 0 ? item.indexCount
                                                                  : item.surface->vertexCount) *
                                      normalRepeats;
                    continue;
                }
                std::uint32_t depthVariant =
                        State::kDepthPrePass | (item.multimeshMode << State::kShadowMultiMeshShift);
                depthVariant |= State::sampleIndex(viewport.samples) << State::kDepthSamplesShift;
                if (item.cull == MaterialCull::Front) depthVariant |= State::kShadowFrontCull;
                else if (item.cull == MaterialCull::Disabled)
                    depthVariant |= State::kDepthNoCull;
                if (scissor)
                {
                    depthVariant |= State::kShadowScissor;
                    if (item.variant & 1u) depthVariant |= State::kShadowAlbedo;
                    if (item.coverage) depthVariant |= State::kDepthCoverage;
                    if (item.billboard == 5) depthVariant |= State::kDepthLeafWind;
                }
                const prisma::PipelineHandle pipeline =
                        state.shadowPipelineFor(mDriver, *item.surface, depthVariant);
                if (!pipeline.valid()) continue;
                if (!depthBound.valid() || depthBound != pipeline)
                {
                    mDriver->bindPipeline(pipeline);
                    depthBound = pipeline;
                    boundDepthMaterial = 0xffffffffu;
                }
                if (scissor && item.material.bits() != boundDepthMaterial)
                {
                    boundDepthMaterial = item.material.bits();
                    const std::uint32_t* slot = state.materialSlots.find(boundDepthMaterial);
                    if (slot && state.materialBuffer.valid())
                        mDriver->bindUniformBuffer(2, state.materialBuffer, *slot * materialStride,
                                sizeof(MaterialDataGpu));
                    if (item.variant & 1u)
                    {
                        const MaterialInfo* material = mMaterials.get(item.material);
                        const TextureId id =
                                material ? sceneTexture(*material, MaterialTexture::Albedo)
                                         : TextureId();
                        mDriver->bindTexture(0,
                                mTextures.valid(id) ? mTextures.gpu(id)
                                                    : mTextures.gpu(mTextures.white()),
                                state.materialSampler);
                    }
                }
                mDriver->bindVertexBuffer(0, item.vertexBuffer, 0);
                if (item.multimeshMode != 0)
                    mDriver->bindStorageBuffer(5, item.transforms, 0, item.transformBytes);
                mDriver->bindUniformBuffer(1, state.uniformBuffer,
                        static_cast<std::uint32_t>(r) * stride, kBlockSize);
                const std::uint32_t repeats =
                        item.multimeshMode != 0 ? item.instanceCount : run.count;
                if (item.indexCount > 0)
                {
                    mDriver->bindIndexBuffer(item.indexBuffer);
                    mDriver->drawIndexed(item.indexCount, 0, repeats);
                }
                else
                    mDriver->draw(item.surface->vertexCount, 0, repeats);
                state.prepassDrawn[r] = 1;
                ++depthDrawCalls;
                depthSurfaces += item.multimeshMode != 0 ? 1u : run.count;
                depthTriangles +=
                        triangleCount(item.surface->topology,
                                item.indexCount > 0 ? item.indexCount : item.surface->vertexCount) *
                        repeats;
            }
            if (impostorsPrepared && viewport.samples == 1 && !normalRoughnessWanted)
            {
                impostorsInPrepass =
                        state.impostors.drawDepth(mDriver, viewport.scenario.bits(), 1) > 0;
            }
            mDriver->endRenderPass();
        }
        if (depthScope) profiler->endGpu();
    }
    const bool prepassUsed = depthDrawCalls > 0 || impostorsInPrepass;

    bool ssaoApplied = false;
    if (ssaoWanted && normalDrawCalls > 0)
    {
        SsaoSettings ssao;
        ssao.radius = environment->ssaoRadius;
        ssao.intensity = environment->ssaoIntensity;
        ssao.power = environment->ssaoPower;
        ssao.detail = environment->ssaoDetail;
        ssao.horizon = environment->ssaoHorizon;
        ssao.sharpness = environment->ssaoSharpness;
        const std::uint32_t dispatches = state.screenSpace.generateSsao(viewport.ssao,
                viewport.depthTexture, viewport.normalRoughness, zeroToOneDepth() * baseProjection,
                shadowLodView.orthogonal, camera->zNear, camera->zFar, ssao);
        ssaoApplied = dispatches > 0 && !state.screenSpace.failed();
        mStats.ssaoDispatches += dispatches;
    }
    bool ssrApplied = false;
    if (ssrWanted && normalDrawCalls > 0)
    {
        SsrSettings ssr;
        ssr.maxSteps = environment->ssrMaxSteps;
        ssr.fadeIn = environment->ssrFadeIn > 0.0f ? environment->ssrFadeIn : 0.0f;
        ssr.fadeOut = environment->ssrFadeOut > 0.0f ? environment->ssrFadeOut : 0.0f;
        ssr.depthTolerance = environment->ssrDepthTolerance;
        Math::Mat4 reverseDepth = Math::Mat4::Identity();
        reverseDepth.col2.z = -1.0f;
        reverseDepth.col3.z = 1.0f;
        const std::uint32_t dispatches =
                state.screenSpace.generateSsr(viewport.ssr, viewport.depthTexture,
                        viewport.normalRoughness, reverseDepth * zeroToOneDepth() * baseProjection,
                        view, shadowLodView.orthogonal, ssr);
        ssrApplied = dispatches > 0 && !state.screenSpace.failed();
        mStats.ssrDispatches += dispatches;
    }
    mStats.normalRoughnessDrawCalls += normalDrawCalls;
    if (implementationReady)
    {
        if (ssaoApplied)
        {
            implementation.ssEffectsFlags |= kSsEffectsUseSsao;
            implementation.ssaoLightAffect = environment->ssaoLightAffect;
            implementation.ssaoAoAffect = environment->ssaoAoChannelAffect;
        }
        if (ssrApplied)
        {
            implementation.ssEffectsFlags |= kSsEffectsUseSsr;
            if (!viewport.ssr.halfSize) implementation.ssEffectsFlags |= kSsEffectsResolveSsr;
        }
        mDriver->updateBuffer(state.implementationBuffer, 0, &implementation,
                sizeof(implementation));
    }

    const bool opaqueScope = profiler != nullptr && profiler->beginGpu("Opaque");
    prisma::RenderPassDesc pass;
    pass.colors[0].texture = viewport.colorTexture;
    pass.colorCount = 1;
    pass.depth.texture = viewport.depthTexture;
    if (viewport.samples > 1)
    {
        pass.colors[0].texture = viewport.colorMsaa;
        pass.depth.texture = viewport.depthMsaa;
        pass.resolves[0].texture = viewport.colorTexture;
        pass.depthResolve.texture = viewport.depthTexture;
        pass.colorStore = prisma::StoreOp::Discard;
    }
    pass.clearColor[0] = clearLinear[0] * clearEnergy;
    pass.clearColor[1] = clearLinear[1] * clearEnergy;
    pass.clearColor[2] = clearLinear[2] * clearEnergy;
    pass.clearColor[3] = viewport.transparent ? 0.0f : viewport.clearColor.w;
    pass.depthStore = viewport.samples > 1 ? prisma::StoreOp::Discard : prisma::StoreOp::Store;
    if (prepassUsed) pass.depthLoad = prisma::LoadOp::Load;
    const bool screenCopy = usedScreenTexture && state.screenCopyColorPipeline.valid() &&
                            (profiler == nullptr || profiler->passEnabled("Screen copy")) &&
                            State::ensureScreenTextures(mDriver, viewport, width, height);
    if (screenCopy && viewport.samples > 1)
    {
        pass.colorStore = prisma::StoreOp::Store;
        pass.depthStore = prisma::StoreOp::Store;
    }
    mDriver->beginRenderPass(pass);
    mDriver->setViewport(area);

    bool screenCopied = false;
    const prisma::TextureHandle aoTexture = ssaoApplied && viewport.ssao.finalAo.valid()
                                                    ? viewport.ssao.finalAo
                                                    : mTextures.gpu(mTextures.white());
    bool aoStale = false;
    bool ssrBound = ssrApplied;
    auto bindScreenSpace = [&]()
    {
        mDriver->bindTexture(kAoSlot, aoTexture, state.linearSampler);
        if (ssrBound)
        {
            mDriver->bindTexture(kSsrSlot, viewport.ssr.result(),
                    state.screenSpace.ssrColorSampler());
            mDriver->bindTexture(kSsrMipSlot,
                    viewport.ssr.halfSize ? mTextures.gpu(mTextures.black())
                                          : viewport.ssr.mipLevel,
                    state.screenSpace.ssrMipSampler());
        }
        else
        {
            mDriver->bindTexture(kSsrSlot, mTextures.gpu(mTextures.black()), state.linearSampler);
            mDriver->bindTexture(kSsrMipSlot, mTextures.gpu(mTextures.black()),
                    state.linearSampler);
        }
    };
    auto bindPassResources = [&]()
    {
        if (state.sceneBuffer.valid())
            mDriver->bindUniformBuffer(0, state.sceneBuffer, 0, sizeof(SceneUboData));
        if (state.instanceBuffer.valid() && !state.items.empty())
            mDriver->bindStorageBuffer(0, state.instanceBuffer, 0, instanceSize);
        mDriver->bindTexture(6,
                state.dfgGenerated && state.dfgPipeline.valid() ? state.dfgTexture
                                                                : mTextures.gpu(mTextures.black()),
                state.linearSampler);
        if (state.mipSampler.valid())
            mDriver->bindTexture(7, radianceReady ? scenario->radiance.array : state.dummyRadiance,
                    state.mipSampler);
        const prisma::TextureHandle directionalAtlas =
                state.shadowAtlas.valid() && state.directionalShadowsActive ? state.shadowAtlas
                                                                            : state.dummyShadow;
        const prisma::TextureHandle positionalAtlas =
                state.positionalShadowsActive && state.activePositionalAtlas.valid()
                        ? state.activePositionalAtlas
                        : state.dummyShadow;
        if (state.shadowSampler.valid())
            mDriver->bindTexture(8, directionalAtlas, state.shadowSampler);
        if (state.shadowDepthSampler.valid())
            mDriver->bindTexture(9, directionalAtlas, state.shadowDepthSampler);
        if (state.positionalShadowSampler.valid())
            mDriver->bindTexture(12, positionalAtlas, state.positionalShadowSampler);
        if (state.positionalShadowDepthSampler.valid())
            mDriver->bindTexture(13, positionalAtlas, state.positionalShadowDepthSampler);
        if (state.lightCountsBuffer.valid())
            mDriver->bindUniformBuffer(3, state.lightCountsBuffer, 0, sizeof(LightCountsGpu));
        if (state.directionalLightBuffer.valid())
            mDriver->bindUniformBuffer(4, state.directionalLightBuffer, 0,
                    sizeof(DirectionalLightDataGpu) * kMaxDirectionalLights);
        if (viewport.cluster && viewport.cluster->getClusterBuffer().valid())
        {
            mDriver->bindStorageBuffer(3, viewport.cluster->getClusterBuffer(), 0,
                    viewport.cluster->getClusterBufferSize());
            mDriver->bindUniformBuffer(5, state.implementationBuffer, 0,
                    sizeof(ImplementationDataGpu));
        }
        if (state.decalBuffer.valid())
            mDriver->bindStorageBuffer(4, state.decalBuffer, 0,
                    static_cast<std::uint32_t>(state.decalData.size() * sizeof(DecalDataGpu)));
        if (state.decalAtlas.getTexture().valid())
        {
            mDriver->bindTexture(10, state.decalAtlas.getTexture(), state.decalSampler);
            mDriver->bindTexture(11, state.decalAtlas.getTextureSrgb(), state.decalSampler);
        }
        if (state.planarBuffer.valid())
            mDriver->bindUniformBuffer(7, state.planarBuffer, 0, sizeof(PlanarReflectionsGpu));
        bindScreenSpace();
        mDriver->bindTexture(18,
                viewport.screenColor.valid() && screenCopied ? viewport.screenColor
                                                             : mTextures.gpu(mTextures.black()),
                state.screenSampler);
        mDriver->bindTexture(19,
                viewport.screenDepth.valid() && screenCopied ? viewport.screenDepth
                                                             : mTextures.gpu(mTextures.white()),
                state.nearestSampler);
        if (state.omniLightBuffer.valid())
            mDriver->bindStorageBuffer(1, state.omniLightBuffer, 0,
                    static_cast<std::uint32_t>(state.omniLightData.size() * sizeof(LightDataGpu)));
        if (state.spotLightBuffer.valid())
            mDriver->bindStorageBuffer(2, state.spotLightBuffer, 0,
                    static_cast<std::uint32_t>(state.spotLightData.size() * sizeof(LightDataGpu)));
    };
    bindPassResources();

    prisma::PipelineHandle bound;
    std::uint64_t boundKey = 0;
    std::uint32_t boundMaterial = 0xffffffffu;
    std::uint32_t boundVariant = 0xffffffffu;
    std::uint32_t drawCalls = 0;
    std::uint32_t triangles = 0;
    std::uint32_t skinnedSurfaces = 0;
    auto drawRuns = [&](bool transparent)
    {
        std::uint32_t boundLightmap = 0xffffffffu;
        for (std::size_t r = 0; r < state.runs.size() && drawItems; ++r)
        {
            const DrawRun& run = state.runs[r];
            const RenderItem& item = state.items[state.order[run.first].index];
            if (item.transparent != transparent) continue;
            const bool replacesAo = item.billboard == 4 || item.billboard == 6;
            if (aoStale && !replacesAo)
            {
                bindScreenSpace();
                aoStale = false;
            }
            if (replacesAo) aoStale = true;
            prisma::PipelineHandle depthPipeline;
            if (transparent && item.fadeDepth)
                depthPipeline = state.pipelineFor(mDriver, *item.surface, item.variant, item.cull,
                        item.transparent, item.blend, item.depthTest, item.multimeshMode,
                        item.billboard, false, item.noShadows, false, viewport.samples,
                        item.lightmap, true);
            const bool equalDepth =
                    ((!transparent || item.coverage) && state.prepassDrawn[r] != 0) ||
                    depthPipeline.valid();
            const prisma::PipelineHandle pipeline = state.pipelineFor(mDriver, *item.surface,
                    item.variant, item.cull, item.transparent, item.blend, item.depthTest,
                    item.multimeshMode, item.billboard, equalDepth, item.noShadows, item.coverage,
                    viewport.samples, item.lightmap);
            if (!pipeline.valid()) continue;
            const std::uint64_t drawKey =
                    item.pipelineKey | (equalDepth ? State::kEqualDepthKey : 0ull);
            if (!bound.valid() || boundKey != drawKey)
            {
                mDriver->bindPipeline(pipeline);
                bound = pipeline;
                boundKey = drawKey;
            }
            if (item.material.bits() != boundMaterial || item.variant != boundVariant)
            {
                boundMaterial = item.material.bits();
                boundVariant = item.variant;
                const std::uint32_t* slot = state.materialSlots.find(boundMaterial);
                if (slot && state.materialBuffer.valid())
                    mDriver->bindUniformBuffer(2, state.materialBuffer, *slot * materialStride,
                            sizeof(MaterialDataGpu));
                const MaterialInfo* material = mMaterials.get(item.material);
                if (material && item.variant != 0)
                    bindVariantTextures(material, item.variant, true);
                if (material && item.billboard == 6)
                {
                    const OceanMaterialData& ocean = material->ocean;
                    auto oceanGpu = [&](TextureId id)
                    { return mTextures.gpu(mTextures.valid(id) ? id : mTextures.black()); };
                    mDriver->bindTexture(21, oceanGpu(ocean.displacementMap),
                            state.materialSampler);
                    mDriver->bindTexture(22, oceanGpu(ocean.gradientMap), state.materialSampler);
                    mDriver->bindTexture(23, oceanGpu(ocean.perlinMap), state.materialSampler);
                }
                if (item.billboard != 4)
                {
                    const TextureId mask =
                            material ? sceneTexture(*material, MaterialTexture::Refraction)
                                     : TextureId();
                    if (item.billboard != 6)
                        mDriver->bindTexture(17,
                                mTextures.gpu(mTextures.valid(mask) ? mask : mTextures.white()),
                                state.materialSampler);
                    prisma::TextureHandle planar = mTextures.gpu(mTextures.black());
                    if (item.reflectorSlot > 0 && item.reflectorSlot <= activeReflectors)
                    {
                        const Reflector* reflector = find(state.reflectors,
                                state.activeReflectors[item.reflectorSlot - 1]);
                        if (reflector)
                            if (const Viewport* target = find(state.viewports, reflector->viewport))
                                if (mTextures.valid(target->output))
                                    planar = mTextures.gpu(target->output);
                    }
                    mDriver->bindTexture(20, planar, state.linearSampler);
                }
                if (material && item.billboard == 4)
                {
                    const TerrainMaterialData& terrain = material->terrain;
                    auto terrainGpu = [&](TextureId id, TextureId fallback)
                    { return mTextures.gpu(mTextures.valid(id) ? id : fallback); };
                    const prisma::SamplerHandle mapSampler = state.linearSampler;
                    const prisma::SamplerHandle layerSampler = state.materialSampler;
                    mDriver->bindTexture(14, terrainGpu(terrain.normalMap, mTextures.white()),
                            mapSampler);
                    mDriver->bindTexture(15, terrainGpu(terrain.lightmap, mTextures.white()),
                            mapSampler);
                    mDriver->bindTexture(16, terrainGpu(terrain.colourMap, mTextures.white()),
                            mapSampler);
                    mDriver->bindTexture(17, terrainGpu(terrain.blendMap, mTextures.black()),
                            mapSampler);
                    for (std::uint32_t layer = 0; layer < TerrainMaterialData::kMaxLayers; ++layer)
                    {
                        mDriver->bindTexture(18 + layer,
                                terrainGpu(terrain.layerDiffuse[layer], mTextures.white()),
                                layerSampler);
                        mDriver->bindTexture(layer == 0 ? 23 : layer,
                                terrainGpu(terrain.layerNormal[layer], mTextures.normal()),
                                layerSampler);
                    }
                }
            }
            if (item.lightmap && boundLightmap != item.lightmapIndex)
            {
                boundLightmap = item.lightmapIndex;
                mDriver->bindTexture(kLightmapLightSlot, state.lightmapLight[boundLightmap],
                        state.linearSampler);
                mDriver->bindTexture(kLightmapShadowmaskSlot,
                        state.lightmapShadowmask[boundLightmap], state.linearSampler);
            }
            if (item.lightmap)
            {
                mDriver->bindStorageBuffer(6, state.lightmapBuffer, 0,
                        static_cast<std::uint32_t>(sizeof(LightmapDataGpu)) * state.lightmapsUsed);
                mDriver->bindStorageBuffer(7, state.lightmapCaptureBuffer, 0,
                        static_cast<std::uint32_t>(
                                state.captureData.empty()
                                        ? 36 * sizeof(float)
                                        : state.captureData.size() * sizeof(float)));
            }
            mDriver->bindVertexBuffer(0, item.vertexBuffer, 0);
            if (item.skinned) ++skinnedSurfaces;
            if (item.multimeshMode != 0)
                mDriver->bindStorageBuffer(5, item.transforms, 0, item.transformBytes);
            mDriver->bindUniformBuffer(1, state.uniformBuffer,
                    static_cast<std::uint32_t>(r) * stride, kBlockSize);
            const std::uint32_t repeats = item.multimeshMode != 0 ? item.instanceCount : run.count;
            auto issueDraw = [&]()
            {
                if (item.indexCount > 0)
                {
                    mDriver->bindIndexBuffer(item.indexBuffer);
                    mDriver->drawIndexed(item.indexCount, 0, repeats);
                }
                else
                    mDriver->draw(item.surface->vertexCount, 0, repeats);
                ++drawCalls;
                triangles +=
                        triangleCount(item.surface->topology,
                                item.indexCount > 0 ? item.indexCount : item.surface->vertexCount) *
                        repeats;
            };
            if (depthPipeline.valid())
            {
                mDriver->bindPipeline(depthPipeline);
                issueDraw();
                mDriver->bindPipeline(pipeline);
            }
            issueDraw();
        }
    };
    drawRuns(false);
    if (impostorsPrepared && drawItems)
    {
        const std::uint32_t cardSets = state.impostors.drawColor(mDriver, viewport.scenario.bits(),
                viewport.samples, impostorsInPrepass);
        if (cardSets > 0)
        {
            mStats.impostorSets += cardSets;
            mStats.impostorDrawCalls += cardSets;
            if (state.instanceBuffer.valid() && !state.items.empty())
                mDriver->bindStorageBuffer(0, state.instanceBuffer, 0, instanceSize);
            bound = prisma::PipelineHandle();
            boundKey = 0;
            boundMaterial = 0xffffffffu;
            boundVariant = 0xffffffffu;
        }
    }
    if (opaqueScope) profiler->endGpu();

    const bool skyScope = profiler != nullptr && profiler->beginGpu("Sky");
    if ((skyBackground || flatSkyReady) && (profiler == nullptr || profiler->passEnabled("Sky")))
    {
        const std::uint32_t variant =
                skyBackground ? State::skyVariant(scenario->environment, scenario->skyMode, false)
                              : State::kSkyFlatVariant;
        if (skyBackground ? skyReady : flatSkyReady)
        {
            mDriver->bindPipeline(state.skyPipelineFor(variant, viewport.samples));
            if (skyBackground && scenario->skyMode != 0)
                mDriver->bindTexture(0, scenario->skyGpu, state.skySampler);
            mDriver->bindUniformBuffer(0,
                    skyBackground ? state.skyParamsBuffer : state.skyFlatParamsBuffer, 0,
                    sizeof(SkyParamsGpu));
            mDriver->bindUniformBuffer(1, state.skyLightsBuffer, 0,
                    sizeof(SkyDirectionalLightGpu) * kMaxSkyDirectionalLights);
            if (skyBackground)
                mDriver->bindUniformBuffer(2, state.skyMaterialBuffer, 0, sizeof(SkyMaterialGpu));
            else
                mDriver->bindUniformBuffer(2, state.skyFlatColorBuffer, 0, 16);
            mDriver->bindUniformBuffer(3, state.skySceneBuffer, 0, sizeof(SkySceneDataGpu));
            mDriver->draw(3, 0);

            if (state.sceneBuffer.valid())
                mDriver->bindUniformBuffer(0, state.sceneBuffer, 0, sizeof(SceneUboData));
            if (state.lightCountsBuffer.valid())
                mDriver->bindUniformBuffer(3, state.lightCountsBuffer, 0, sizeof(LightCountsGpu));
            bound = prisma::PipelineHandle();
            boundKey = 0;
            boundMaterial = 0xffffffffu;
            boundVariant = 0xffffffffu;
        }
    }
    if (skyScope) profiler->endGpu();

    if (screenCopy)
    {
        mDriver->endRenderPass();
        if (!state.screenCopyRegistered)
        {
            state.screenCopyRegistered = true;
            if (profiler) profiler->registerPass("Screen copy");
        }
        const bool copyScope = profiler != nullptr && profiler->beginGpu("Screen copy");
        state.copyScreen(mDriver, viewport, area);
        if (copyScope) profiler->endGpu();
        ++mStats.screenCopies;
        prisma::RenderPassDesc resume = pass;
        resume.colorLoad = prisma::LoadOp::Load;
        resume.depthLoad = prisma::LoadOp::Load;
        mDriver->beginRenderPass(resume);
        mDriver->setViewport(area);
        screenCopied = true;
        bindPassResources();
        aoStale = false;
        bound = prisma::PipelineHandle();
        boundKey = 0;
        boundMaterial = 0xffffffffu;
        boundVariant = 0xffffffffu;
    }

    const bool transparentScope = profiler != nullptr && profiler->beginGpu("Transparent");
    if (ssrBound)
    {
        ssrBound = false;
        bindScreenSpace();
    }
    if (profiler == nullptr || profiler->passEnabled("Transparent")) drawRuns(true);
    if (transparentScope) profiler->endGpu();
    if (drawBatch && !state.batch.empty(BatchRenderer::Space::World))
        state.batch.draw(BatchRenderer::Space::World, batchSlot, viewport.samples);
    mDriver->endRenderPass();

    if (ssrApplied)
        mStats.ssrDispatches +=
                state.screenSpace.copyLastFrame(viewport.ssr, viewport.colorTexture);

    bool dofApplied = false;
    if (dofRequested)
    {
        bool dofCreated = false;
        if (state.dof.allocate(viewport.dof, width, height, dofCreated) &&
                (viewport.dof.raster || viewport.colorStorage))
        {
            if (dofCreated) ++mStats.dofTargetsCreated;
            bool dofOrthogonal = false;
            float dofNear = 0.0f;
            float dofFar = 0.0f;
            projectionPlanes(baseProjection, dofOrthogonal, dofNear, dofFar);
            const std::uint32_t passes = state.dof.run(viewport.dof, viewport.colorTexture,
                    viewport.depthTexture, *attributes, dofNear, dofFar, dofOrthogonal);
            if (viewport.dof.raster) mStats.dofDraws += passes;
            else
                mStats.dofDispatches += passes;
            dofApplied = passes > 0;
        }
    }
    if (!dofApplied) State::destroyDof(mDriver, viewport);

    if (profiler == nullptr || profiler->passEnabled("Lens flare"))
        state.drawLensFlares(mDriver, mTextures, viewport, *camera, view, baseProjection, area,
                profiler, mStats, drawCalls);

    if (glowOn)
    {
        const bool glowScope = profiler != nullptr && profiler->beginGpu("Glow");
        state.runGlow(mDriver, glowPasses, glowPassCount);
        mStats.glowPasses += glowPassCount;
        if (glowScope) profiler->endGpu();
    }

    const bool drawFinal = profiler == nullptr || profiler->passEnabled("Final");
    const bool finalScope = profiler != nullptr && profiler->beginGpu("Final");
    prisma::RenderPassDesc finalPass;
    finalPass.colorLoad = drawFinal ? prisma::LoadOp::DontCare : prisma::LoadOp::Clear;
    finalPass.depthLoad = prisma::LoadOp::DontCare;
    finalPass.depthStore = prisma::StoreOp::Discard;
    const bool keepAlpha = dofApplied && !toScreen && state.ensureTonemapMasked(mDriver);
    prisma::PipelineHandle finalPipeline =
            toScreen ? state.tonemapPipeline
                     : state.tonemapTexturePipelines[viewport.outputLinear ? 1 : 0];
    if (keepAlpha)
    {
        finalPipeline = state.tonemapMaskedPipelines[viewport.outputLinear ? 1 : 0];
        finalPass.colorLoad = prisma::LoadOp::Clear;
        finalPass.clearColor[0] = 0.0f;
        finalPass.clearColor[1] = 0.0f;
        finalPass.clearColor[2] = 0.0f;
        finalPass.clearColor[3] = viewport.clearColor.w;
    }
    const bool upscale = upscaleWanted && drawFinal && !keepAlpha &&
                         State::ensureUpscaleSource(mDriver, viewport, width, height);
    if (upscale)
    {
        finalPass.colors[0].texture = viewport.upscaleSource;
        finalPass.colorCount = 1;
        finalPipeline = state.tonemapTexturePipelines[1];
    }
    else if (!toScreen)
    {
        finalPass.colors[0].texture = mTextures.gpu(viewport.output);
        finalPass.colorCount = 1;
    }
    mDriver->beginRenderPass(finalPass);
    mDriver->setViewport(upscale ? area : toScreen ? windowArea : outputArea);
    if (drawFinal && finalPipeline.valid())
    {
        const prisma::TextureHandle white = mTextures.gpu(mTextures.white());
        const prisma::TextureHandle black = mTextures.gpu(mTextures.black());
        mDriver->bindPipeline(finalPipeline);
        mDriver->bindUniformBuffer(0, state.tonemapParams, 0, sizeof(TonemapParams));
        mDriver->bindTexture(0, viewport.colorTexture, state.linearSampler);
        mDriver->bindTexture(1, white, state.linearSampler);
        if (glowOn)
        {
            mDriver->bindTexture(2, viewport.glowTexture, state.glowSampler);
            mDriver->bindTexture(3, glowMapTexture, state.glowSampler);
        }
        else
        {
            mDriver->bindTexture(2, black, state.linearSampler);
            mDriver->bindTexture(3, white, state.linearSampler);
        }
        mDriver->bindTexture(4, colorCorrectionTexture.valid() ? colorCorrectionTexture : white,
                state.linearSampler);
        mDriver->draw(3, 0);
        if (!upscale && drawBatch && !state.batch.empty(BatchRenderer::Space::Screen))
            state.batch.draw(BatchRenderer::Space::Screen, screenSlot, 1,
                    toScreen ? prisma::TextureFormat::None
                             : (viewport.outputLinear ? prisma::TextureFormat::RGBA8
                                                      : prisma::TextureFormat::RGBA8Srgb));
    }
    mDriver->endRenderPass();
    if (upscale)
    {
        // The window: the picture scaled up (by FSR first, when asked), then the screen-space
        // batch at full size.
        prisma::TextureHandle picture = viewport.upscaleSource;
        if (smaaOn && State::ensureSmaaTargets(mDriver, viewport, width, height))
        {
            const bool smaaScope = profiler != nullptr && profiler->beginGpu("SMAA");
            state.runSmaa(mDriver, viewport, picture, width, height);
            if (smaaScope) profiler->endGpu();
            picture = viewport.smaaOutput;
        }
        else if (!smaaOn)
            State::destroySmaaTargets(mDriver, viewport);
        if (viewport.scaling3DMode == Scaling3DMode::Fsr && state.fsrPipeline.valid())
        {
            const std::uint32_t rectWidth = static_cast<std::uint32_t>(layout.rectWidth + 0.5f);
            const std::uint32_t rectHeight = static_cast<std::uint32_t>(layout.rectHeight + 0.5f);
            if (rectWidth > 0 && rectHeight > 0 &&
                    State::ensureFsrTargets(mDriver, viewport, rectWidth, rectHeight))
            {
                const bool fsrScope = profiler != nullptr && profiler->beginGpu("FSR");
                state.runFsr(mDriver, viewport, picture, width, height);
                if (fsrScope) profiler->endGpu();
                picture = viewport.fsrOutput;
            }
        }
        else
            State::destroyFsrTargets(mDriver, viewport);
        prisma::RenderPassDesc windowPass;
        windowPass.colorLoad = prisma::LoadOp::DontCare;
        windowPass.depthLoad = prisma::LoadOp::DontCare;
        windowPass.depthStore = prisma::StoreOp::Discard;
        mDriver->beginRenderPass(windowPass);
        mDriver->setViewport(windowArea);
        mDriver->bindPipeline(state.upscalePipeline);
        mDriver->bindUniformBuffer(0, state.upscaleParams, 0, sizeof(UpscaleParams));
        mDriver->bindTexture(0, picture, state.linearSampler);
        mDriver->draw(3, 0);
        if (drawBatch && !state.batch.empty(BatchRenderer::Space::Screen))
            state.batch.draw(BatchRenderer::Space::Screen, screenSlot, 1,
                    prisma::TextureFormat::None);
        mDriver->endRenderPass();
    }
    else if (toScreen)
    {
        State::destroyUpscaleSource(mDriver, viewport);
    }
    if (finalScope) profiler->endGpu();

    ++mStats.viewportsDrawn;
    mStats.instancesDrawn += instancesDrawn;
    if (drawItems) mStats.multimeshInstancesDrawn += multimeshInstancesDrawn;
    if (drawItems) mStats.surfacesDrawn += static_cast<std::uint32_t>(state.items.size());
    mStats.drawCalls += drawCalls;
    mStats.triangles += triangles;
    if (drawItems) mStats.lodSurfaces += lodSurfaces;
    mStats.depthPrePassSurfaces += depthSurfaces;
    mStats.depthPrePassDrawCalls += depthDrawCalls;
    mStats.depthPrePassTriangles += depthTriangles;
    mStats.coverageSurfaces += coverageItems;
    mStats.skinnedSurfaces += skinnedSurfaces;
    std::uint32_t glowLive = 0;
    for (const Viewport& other: state.viewports)
        glowLive += (other.glowTexture.valid() ? 1u : 0u) + (other.glowTemp.valid() ? 1u : 0u);
    mStats.glowTexturesLive = glowLive;
    std::uint32_t ssaoLive = 0;
    std::uint32_t normalLive = 0;
    for (const Viewport& other: state.viewports)
    {
        if (other.normalRoughness.valid()) ++normalLive;
        ssaoLive += (other.ssao.linearDepth.valid() ? 1u : 0u) +
                    (other.ssao.deinterleaved.valid() ? 1u : 0u) +
                    (other.ssao.pong.valid() ? 1u : 0u) +
                    (other.ssao.importance.valid() ? 1u : 0u) +
                    (other.ssao.importancePong.valid() ? 1u : 0u) +
                    (other.ssao.finalAo.valid() ? 1u : 0u);
    }
    mStats.ssaoTexturesLive = ssaoLive;
    std::uint32_t ssrLive = 0;
    for (const Viewport& other: state.viewports)
        ssrLive += (other.ssr.lastFrame.valid() ? 1u : 0u) + (other.ssr.hiz.valid() ? 1u : 0u) +
                   (other.ssr.color.valid() ? 1u : 0u) + (other.ssr.mipLevel.valid() ? 1u : 0u) +
                   (other.ssr.normalRoughness.valid() ? 1u : 0u) +
                   (other.ssr.resolved.valid() ? 1u : 0u);
    mStats.ssrTexturesLive = ssrLive;
    std::uint32_t dofLive = 0;
    for (const Viewport& other: state.viewports) dofLive += other.dof.textureCount();
    mStats.dofTexturesLive = dofLive;
    mStats.normalRoughnessTexturesLive = normalLive;
}

const RenderStats& RenderingServer::getStats() const
{
    return mStats;
}

Math::Mat4 RenderingServer::perspective(float fovyDegrees, float aspect, float zNear, float zFar)
{
    return zeroToOneDepth() * basePerspective(fovyDegrees, aspect, zNear, zFar);
}

Math::Mat4 RenderingServer::orthogonal(float size, float aspect, float zNear, float zFar)
{
    return zeroToOneDepth() * baseOrthogonal(size, aspect, zNear, zFar);
}

} // namespace prisma3d
