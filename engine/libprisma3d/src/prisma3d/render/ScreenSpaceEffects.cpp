#include "ScreenSpaceEffects.h"
#include "prisma3d/render/ShaderVariants.h"

#include "prisma3d/render/RenderProfiler.h"

#include "platform.h"

#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const std::uint32_t kBestFitSize = 1024;
const std::uint32_t kMaxBlocks = 64;
const std::uint32_t kBlockSize = 128;

enum DownsampleMode : std::uint32_t
{
    kDownsample,
    kDownsampleHalfSize,
    kDownsampleMips,
    kDownsampleMipsHalfSize,
    kDownsampleHalfBuffers,
    kDownsampleHalfBuffersHalfSize
};

enum GatherMode : std::uint32_t
{
    kGather,
    kGatherBase,
    kGatherAdaptive
};

enum ImportanceMode : std::uint32_t
{
    kImportanceGenerate,
    kImportanceProcessA,
    kImportanceProcessB
};

enum BlurMode : std::uint32_t
{
    kBlurNonSmart,
    kBlurSmart,
    kBlurWide
};

enum InterleaveMode : std::uint32_t
{
    kInterleaveNonSmart,
    kInterleaveSmart,
    kInterleaveHalf
};

struct DownsampleParams
{
    float pixelSize[2];
    float zFar;
    float zNear;
    std::uint32_t orthogonal;
    float radiusSq;
    std::uint32_t pad[2];
};

struct GatherParams
{
    std::int32_t screenSize[2];
    std::int32_t pass;
    std::int32_t quality;
    float halfScreenPixelSize[2];
    std::int32_t sizeMultiplier;
    float detailIntensity;
    float ndcToViewMul[2];
    float ndcToViewAdd[2];
    float pad2[2];
    float halfScreenPixelSizeX025[2];
    float radius;
    float intensity;
    float shadowPower;
    float shadowClamp;
    float fadeOutMul;
    float fadeOutAdd;
    float horizonAngleThreshold;
    float invRadiusNearLimit;
    std::uint32_t isOrthogonal;
    float negInvRadius;
    float loadCounterAvgDiv;
    float adaptiveSampleLimit;
    std::int32_t passCoordOffset[2];
    float passUvOffset[2];
};

struct ImportanceParams
{
    float halfScreenPixelSize[2];
    float intensity;
    float power;
};

struct BlurParams
{
    float edgeSharpness;
    float pad;
    float halfScreenPixelSize[2];
    std::int32_t layer;
    std::int32_t pad2[3];
};

struct InterleaveParams
{
    float invSharpness;
    std::uint32_t sizeModifier;
    float pixelSize[2];
};

enum SsrCopyMode : std::uint32_t
{
    kSsrCopySimple,
    kSsrCopyDepth,
    kSsrCopyMipmap
};

const std::uint32_t kSsrFlagSanitize = 1u << 9;
const std::uint32_t kSsrBlockSize = 48;
const std::uint32_t kSsrMaxBlocks = 64;

struct SsrCopyParams
{
    std::int32_t section[4];
    std::int32_t target[2];
    std::uint32_t flags;
    float luminanceMultiplier;
};

struct SsrSizeParams
{
    std::int32_t screenSize[2];
    std::int32_t pad[2];
};

struct SsrTraceParams
{
    std::int32_t screenSize[2];
    std::int32_t mipmaps;
    std::int32_t numSteps;
    float distanceFade;
    float curveFadeIn;
    float depthTolerance;
    std::uint32_t orthogonal;
    std::int32_t viewIndex;
    std::int32_t pad[3];
};

struct SsrFilterParams
{
    std::int32_t screenSize[2];
    std::uint32_t mipLevel;
    std::int32_t pad;
};

struct SsrSceneData
{
    float projection[2][16];
    float invProjection[2][16];
    float reprojection[2][16];
    float eyeOffset[2][4];
};

static_assert(sizeof(SsrCopyParams) == 32, "SsrCopyParams must match the std140 block");
static_assert(sizeof(SsrSizeParams) == 16, "SsrSizeParams must match the std140 block");
static_assert(sizeof(SsrTraceParams) == 48, "SsrTraceParams must match the std140 block");
static_assert(sizeof(SsrFilterParams) == 16, "SsrFilterParams must match the std140 block");
static_assert(sizeof(SsrSceneData) == 416, "SsrSceneData must match the std140 block");

static_assert(sizeof(DownsampleParams) == 32, "DownsampleParams must match the std140 block");
static_assert(sizeof(GatherParams) == 128, "GatherParams must match the std140 block");
static_assert(sizeof(ImportanceParams) == 16, "ImportanceParams must match the std140 block");
static_assert(sizeof(BlurParams) == 32, "BlurParams must match the std140 block");
static_assert(sizeof(InterleaveParams) == 16, "InterleaveParams must match the std140 block");

std::uint32_t alignUp(std::uint32_t value, std::uint32_t alignment)
{
    return alignment > 1 ? (value + alignment - 1) / alignment * alignment : value;
}

float exactReciprocal(float size)
{
    float value = 1.0f / size;
    while (1.0f / value < size) value = nextafterf(value, 0.0f);
    return value;
}

std::uint32_t dispatchGroups(std::uint32_t threads)
{
    return (threads + 7u) / 8u;
}

} // namespace

struct ScreenSpaceEffects::Pipelines
{
    prisma::ShaderHandle downsampleShaders[6];
    prisma::PipelineHandle downsample[6];
    prisma::ShaderHandle gatherShaders[3];
    prisma::PipelineHandle gather[3];
    prisma::ShaderHandle importanceShaders[3];
    prisma::PipelineHandle importance[3];
    prisma::ShaderHandle blurShaders[3];
    prisma::PipelineHandle blur[3];
    prisma::ShaderHandle interleaveShaders[3];
    prisma::PipelineHandle interleave[3];
    prisma::SamplerHandle mirrorSampler;
    prisma::SamplerHandle linearSampler;
    prisma::SamplerHandle nearestSampler;
    prisma::BufferHandle constants;
    prisma::BufferHandle params;
    prisma::BufferHandle counter;
    prisma::TextureHandle bestFit;
    prisma::ShaderHandle bestFitShader;
    prisma::PipelineHandle bestFitPipeline;
    ct::Vector<std::uint8_t> bytes;
    std::uint32_t blockStride = 0;
    bool ssaoReady = false;
    bool ssaoRegistered = false;
    prisma::ShaderHandle ssrCopyShaders[3];
    prisma::PipelineHandle ssrCopy[3];
    prisma::ShaderHandle ssrDownsampleShaders[4];
    prisma::PipelineHandle ssrDownsample[4];
    prisma::ShaderHandle ssrHizShaders[4];
    prisma::PipelineHandle ssrHiz[4];
    prisma::ShaderHandle ssrTraceShader;
    prisma::PipelineHandle ssrTrace;
    prisma::ShaderHandle ssrFilterShader;
    prisma::PipelineHandle ssrFilter;
    prisma::ShaderHandle ssrResolveShader;
    prisma::PipelineHandle ssrResolve;
    prisma::SamplerHandle ssrLinearSampler;
    prisma::SamplerHandle ssrNearestSampler;
    prisma::BufferHandle ssrParams;
    prisma::BufferHandle ssrScene;
    prisma::BufferHandle ssrCopyParams;
    ct::Vector<std::uint8_t> ssrBytes;
    std::uint32_t ssrBlockStride = 0;
    bool ssrReady = false;
    bool ssrRegistered = false;
};

ScreenSpaceEffects::ScreenSpaceEffects()
        : mDriver(nullptr),
          mFailed(false),
          mBestFitFailed(false),
          mQuality(SsaoQuality::Medium),
          mHalfSize(true),
          mSsrHalfSize(true),
          mAdaptiveTarget(0.5f),
          mBlurPasses(2),
          mFadeoutFrom(50.0f),
          mFadeoutTo(300.0f),
          mPipelines(new Pipelines())
{
}

ScreenSpaceEffects::~ScreenSpaceEffects()
{
    destroy();
    delete mPipelines;
}

bool ScreenSpaceEffects::create(prisma::Driver* driver)
{
    destroy();
    mDriver = driver;
    mFailed = false;
    mBestFitFailed = false;
    return driver != nullptr;
}

void ScreenSpaceEffects::destroy()
{
    if (!mDriver) return;
    Pipelines& p = *mPipelines;
    for (std::uint32_t i = 0; i < 6; ++i)
    {
        if (p.downsample[i].valid()) mDriver->destroy(p.downsample[i]);
        if (p.downsampleShaders[i].valid()) mDriver->destroy(p.downsampleShaders[i]);
    }
    for (std::uint32_t i = 0; i < 3; ++i)
    {
        if (p.gather[i].valid()) mDriver->destroy(p.gather[i]);
        if (p.gatherShaders[i].valid()) mDriver->destroy(p.gatherShaders[i]);
        if (p.importance[i].valid()) mDriver->destroy(p.importance[i]);
        if (p.importanceShaders[i].valid()) mDriver->destroy(p.importanceShaders[i]);
        if (p.blur[i].valid()) mDriver->destroy(p.blur[i]);
        if (p.blurShaders[i].valid()) mDriver->destroy(p.blurShaders[i]);
        if (p.interleave[i].valid()) mDriver->destroy(p.interleave[i]);
        if (p.interleaveShaders[i].valid()) mDriver->destroy(p.interleaveShaders[i]);
    }
    for (std::uint32_t i = 0; i < 3; ++i)
    {
        if (p.ssrCopy[i].valid()) mDriver->destroy(p.ssrCopy[i]);
        if (p.ssrCopyShaders[i].valid()) mDriver->destroy(p.ssrCopyShaders[i]);
    }
    for (std::uint32_t i = 0; i < 4; ++i)
    {
        if (p.ssrDownsample[i].valid()) mDriver->destroy(p.ssrDownsample[i]);
        if (p.ssrDownsampleShaders[i].valid()) mDriver->destroy(p.ssrDownsampleShaders[i]);
        if (p.ssrHiz[i].valid()) mDriver->destroy(p.ssrHiz[i]);
        if (p.ssrHizShaders[i].valid()) mDriver->destroy(p.ssrHizShaders[i]);
    }
    if (p.ssrTrace.valid()) mDriver->destroy(p.ssrTrace);
    if (p.ssrTraceShader.valid()) mDriver->destroy(p.ssrTraceShader);
    if (p.ssrFilter.valid()) mDriver->destroy(p.ssrFilter);
    if (p.ssrFilterShader.valid()) mDriver->destroy(p.ssrFilterShader);
    if (p.ssrResolve.valid()) mDriver->destroy(p.ssrResolve);
    if (p.ssrResolveShader.valid()) mDriver->destroy(p.ssrResolveShader);
    if (p.ssrLinearSampler.valid()) mDriver->destroy(p.ssrLinearSampler);
    if (p.ssrNearestSampler.valid()) mDriver->destroy(p.ssrNearestSampler);
    if (p.ssrParams.valid()) mDriver->destroy(p.ssrParams);
    if (p.ssrScene.valid()) mDriver->destroy(p.ssrScene);
    if (p.ssrCopyParams.valid()) mDriver->destroy(p.ssrCopyParams);
    if (p.mirrorSampler.valid()) mDriver->destroy(p.mirrorSampler);
    if (p.linearSampler.valid()) mDriver->destroy(p.linearSampler);
    if (p.nearestSampler.valid()) mDriver->destroy(p.nearestSampler);
    if (p.constants.valid()) mDriver->destroy(p.constants);
    if (p.params.valid()) mDriver->destroy(p.params);
    if (p.counter.valid()) mDriver->destroy(p.counter);
    if (p.bestFit.valid()) mDriver->destroy(p.bestFit);
    if (p.bestFitPipeline.valid()) mDriver->destroy(p.bestFitPipeline);
    if (p.bestFitShader.valid()) mDriver->destroy(p.bestFitShader);
    delete mPipelines;
    mPipelines = new Pipelines();
    mDriver = nullptr;
}

bool ScreenSpaceEffects::supported() const
{
    return mDriver != nullptr && mDriver->caps().compute && !mFailed;
}

bool ScreenSpaceEffects::failed() const
{
    return mFailed;
}

void ScreenSpaceEffects::setSsaoQuality(SsaoQuality quality, bool halfSize, float adaptiveTarget,
        std::uint32_t blurPasses, float fadeoutFrom, float fadeoutTo)
{
    mQuality = quality;
    mHalfSize = halfSize;
    mAdaptiveTarget = adaptiveTarget;
    mBlurPasses = blurPasses > 6 ? 6 : blurPasses;
    mFadeoutFrom = fadeoutFrom;
    mFadeoutTo = fadeoutTo;
}

SsaoQuality ScreenSpaceEffects::ssaoQuality() const
{
    return mQuality;
}

bool ScreenSpaceEffects::ssaoHalfSize() const
{
    return mHalfSize;
}

float ScreenSpaceEffects::ssaoAdaptiveTarget() const
{
    return mAdaptiveTarget;
}

std::uint32_t ScreenSpaceEffects::ssaoBlurPasses() const
{
    return mBlurPasses;
}

float ScreenSpaceEffects::ssaoFadeoutFrom() const
{
    return mFadeoutFrom;
}

float ScreenSpaceEffects::ssaoFadeoutTo() const
{
    return mFadeoutTo;
}

prisma::TextureFormat ScreenSpaceEffects::normalRoughnessFormat()
{
    return prisma::TextureFormat::RGBA8;
}

std::uint32_t ScreenSpaceEffects::normalRoughnessUsage()
{
    return prisma::kTextureSampled | prisma::kTextureRenderTarget | prisma::kTextureStorage;
}

prisma::TextureHandle ScreenSpaceEffects::bestFitNormal() const
{
    return mPipelines->bestFit;
}

bool ScreenSpaceEffects::ensureBestFitNormal()
{
    Pipelines& p = *mPipelines;
    if (p.bestFit.valid()) return true;
    if (mBestFitFailed || !mDriver) return false;
    mBestFitFailed = true;
    if (!mDriver->caps().compute)
    {
        log_error("render: the normal and roughness buffer needs compute shaders");
        return false;
    }
    prisma::TextureDesc textureDesc;
    textureDesc.format = prisma::TextureFormat::R8;
    textureDesc.width = kBestFitSize;
    textureDesc.height = kBestFitSize;
    textureDesc.usage = prisma::kTextureSampled | prisma::kTextureStorage;
    textureDesc.debugName = "best fit normal";
    p.bestFit = mDriver->createTexture(textureDesc);
    prisma::ComputePipelineDesc desc;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "current_image";
    desc.storageTextures[0].slot = 0;
    desc.debugName = "best fit normal";
    p.bestFitPipeline = createPipeline(best_fit_normal_comp, desc, p.bestFitShader);
    if (!p.bestFit.valid() || !p.bestFitPipeline.valid())
    {
        if (p.bestFit.valid()) mDriver->destroy(p.bestFit);
        p.bestFit = prisma::TextureHandle();
        log_error("render: could not create the best fit normal table");
        return false;
    }
    mDriver->beginComputePass();
    mDriver->bindPipeline(p.bestFitPipeline);
    mDriver->bindStorageTexture(0, p.bestFit, 0, prisma::StorageAccess::Write);
    mDriver->dispatch(kBestFitSize / 8, kBestFitSize / 8, 1);
    mDriver->endComputePass();
    mBestFitFailed = false;
    return true;
}

prisma::PipelineHandle ScreenSpaceEffects::createPipeline(const ShaderVariant& variant,
        const prisma::ComputePipelineDesc& desc, prisma::ShaderHandle& shader)
{
    shader = mDriver->createShader(prisma3d::shaderDesc(variant, mDriver->caps()));
    if (!shader.valid()) return prisma::PipelineHandle();
    prisma::ComputePipelineDesc filled = desc;
    filled.shader = shader;
    return mDriver->createComputePipeline(filled);
}

bool ScreenSpaceEffects::ensureSsao()
{
    Pipelines& p = *mPipelines;
    if (p.ssaoReady) return true;
    if (mFailed || !mDriver) return false;
    if (!mDriver->caps().compute)
    {
        mFailed = true;
        log_error("render: screen space ambient occlusion needs compute shaders");
        return false;
    }

    prisma::SamplerDesc samplerDesc;
    samplerDesc.minFilter = prisma::Filter::Nearest;
    samplerDesc.magFilter = prisma::Filter::Nearest;
    samplerDesc.mipFilter = prisma::MipFilter::Nearest;
    samplerDesc.addressU = prisma::AddressMode::MirroredRepeat;
    samplerDesc.addressV = prisma::AddressMode::MirroredRepeat;
    samplerDesc.addressW = prisma::AddressMode::MirroredRepeat;
    samplerDesc.debugName = "ssao mirror";
    p.mirrorSampler = mDriver->createSampler(samplerDesc);
    samplerDesc.minFilter = prisma::Filter::Linear;
    samplerDesc.magFilter = prisma::Filter::Linear;
    samplerDesc.mipFilter = prisma::MipFilter::None;
    samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
    samplerDesc.debugName = "ssao linear";
    p.linearSampler = mDriver->createSampler(samplerDesc);
    samplerDesc.minFilter = prisma::Filter::Nearest;
    samplerDesc.magFilter = prisma::Filter::Nearest;
    samplerDesc.debugName = "ssao nearest";
    p.nearestSampler = mDriver->createSampler(samplerDesc);

    float rotations[80];
    const int subPassCount = 5;
    const int subPassMap[5] = { 0, 1, 4, 3, 2 };
    const float pi = 3.14159265358979323846f;
    for (int pass = 0; pass < 4; ++pass)
    {
        for (int subPass = 0; subPass < subPassCount; ++subPass)
        {
            const int a = pass;
            const int b = subPassMap[subPass];
            const float angle0 = (static_cast<float>(a) +
                                         static_cast<float>(b) / static_cast<float>(subPassCount)) *
                                 pi * 0.5f;
            const float ca = cosf(angle0);
            const float sa = sinf(angle0);
            const float scale =
                    1.0f + (static_cast<float>(a) - 1.5f +
                                   (static_cast<float>(b) -
                                           (static_cast<float>(subPassCount) - 1.0f) * 0.5f) /
                                           static_cast<float>(subPassCount)) *
                                   0.07f;
            rotations[pass * 20 + subPass * 4 + 0] = scale * ca;
            rotations[pass * 20 + subPass * 4 + 1] = scale * -sa;
            rotations[pass * 20 + subPass * 4 + 2] = -scale * sa;
            rotations[pass * 20 + subPass * 4 + 3] = -scale * ca;
        }
    }

    prisma::BufferDesc bufferDesc;
    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.update = prisma::BufferUpdate::Static;
    bufferDesc.size = sizeof(rotations);
    bufferDesc.data = rotations;
    bufferDesc.debugName = "ssao constants";
    p.constants = mDriver->createBuffer(bufferDesc);

    p.blockStride = alignUp(kBlockSize, mDriver->caps().uniformBufferOffsetAlignment);
    bufferDesc = prisma::BufferDesc();
    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.size = p.blockStride * kMaxBlocks;
    bufferDesc.debugName = "ssao params";
    p.params = mDriver->createBuffer(bufferDesc);

    const std::uint32_t zero = 0;
    bufferDesc = prisma::BufferDesc();
    bufferDesc.usage = prisma::BufferUsage::Storage;
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.size = sizeof(std::uint32_t);
    bufferDesc.data = &zero;
    bufferDesc.debugName = "ssao importance counter";
    p.counter = mDriver->createBuffer(bufferDesc);

    if (!p.mirrorSampler.valid() || !p.linearSampler.valid() || !p.nearestSampler.valid() ||
            !p.constants.valid() || !p.params.valid() || !p.counter.valid())
    {
        mFailed = true;
        log_error("render: could not create the ambient occlusion resources");
        return false;
    }
    p.bytes.resize(static_cast<std::size_t>(p.blockStride) * kMaxBlocks);
    p.ssaoReady = true;
    return true;
}

prisma::PipelineHandle ScreenSpaceEffects::downsamplePipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.downsample[mode].valid()) return p.downsample[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "source_depth";
    desc.textures[0].slot = 0;
    static const char* const names[4] = { "dest_image0", "dest_image1", "dest_image2",
        "dest_image3" };
    desc.storageTextureCount = (mode == kDownsampleMips || mode == kDownsampleMipsHalfSize) ? 4 : 1;
    for (std::uint32_t i = 0; i < desc.storageTextureCount; ++i)
    {
        desc.storageTextures[i].name = names[i];
        desc.storageTextures[i].slot = i;
    }
    desc.debugName = "ssao downsample";
    p.downsample[mode] =
            createPipeline(*kSsaoDownsampleBlobs[mode], desc, p.downsampleShaders[mode]);
    return p.downsample[mode];
}

prisma::PipelineHandle ScreenSpaceEffects::gatherPipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.gather[mode].valid()) return p.gather[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 2;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.uniformBlocks[1].name = "Constants";
    desc.uniformBlocks[1].slot = 1;
    desc.textureCount = 1;
    desc.textures[0].name = "source_depth_mipmaps";
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 2;
    desc.storageTextures[0].name = "source_normal";
    desc.storageTextures[0].slot = 0;
    desc.storageTextures[1].name = "dest_image";
    desc.storageTextures[1].slot = 2;
    if (mode == kGatherAdaptive)
    {
        desc.textureCount = 2;
        desc.textures[1].name = "source_importance";
        desc.textures[1].slot = 1;
        desc.storageTextureCount = 3;
        desc.storageTextures[2].name = "source_ssao";
        desc.storageTextures[2].slot = 1;
        desc.storageBufferCount = 1;
        desc.storageBuffers[0].name = "Counter";
        desc.storageBuffers[0].slot = 0;
    }
    desc.debugName = "ssao gather";
    p.gather[mode] = createPipeline(*kSsaoGatherBlobs[mode], desc, p.gatherShaders[mode]);
    return p.gather[mode];
}

prisma::PipelineHandle ScreenSpaceEffects::importancePipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.importance[mode].valid()) return p.importance[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = mode == kImportanceGenerate ? "source_texture" : "source_importance";
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "dest_image";
    desc.storageTextures[0].slot = 0;
    if (mode == kImportanceProcessB)
    {
        desc.storageBufferCount = 1;
        desc.storageBuffers[0].name = "Counter";
        desc.storageBuffers[0].slot = 0;
    }
    desc.debugName = "ssao importance";
    p.importance[mode] =
            createPipeline(*kSsaoImportanceBlobs[mode], desc, p.importanceShaders[mode]);
    return p.importance[mode];
}

prisma::PipelineHandle ScreenSpaceEffects::blurPipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.blur[mode].valid()) return p.blur[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "source_ssao";
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "dest_image";
    desc.storageTextures[0].slot = 0;
    desc.debugName = "ssao blur";
    p.blur[mode] = createPipeline(*kSsaoBlurBlobs[mode], desc, p.blurShaders[mode]);
    return p.blur[mode];
}

prisma::PipelineHandle ScreenSpaceEffects::interleavePipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.interleave[mode].valid()) return p.interleave[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "source_texture";
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "dest_image";
    desc.storageTextures[0].slot = 0;
    desc.debugName = "ssao interleave";
    p.interleave[mode] =
            createPipeline(*kSsaoInterleaveBlobs[mode], desc, p.interleaveShaders[mode]);
    return p.interleave[mode];
}

void ScreenSpaceEffects::releaseSsao(prisma::Driver* driver, SsaoBuffers& buffers)
{
    if (driver)
    {
        if (buffers.linearDepth.valid()) driver->destroy(buffers.linearDepth);
        if (buffers.deinterleaved.valid()) driver->destroy(buffers.deinterleaved);
        if (buffers.pong.valid()) driver->destroy(buffers.pong);
        if (buffers.importance.valid()) driver->destroy(buffers.importance);
        if (buffers.importancePong.valid()) driver->destroy(buffers.importancePong);
        if (buffers.finalAo.valid()) driver->destroy(buffers.finalAo);
    }
    buffers = SsaoBuffers();
}

bool ScreenSpaceEffects::allocateSsao(SsaoBuffers& buffers, std::uint32_t width,
        std::uint32_t height, bool& created)
{
    created = false;
    if (!ensureSsao()) return false;
    const bool mips = mQuality > SsaoQuality::Medium;
    const bool ultra = mQuality == SsaoQuality::Ultra;
    if (buffers.valid() && buffers.fullWidth == width && buffers.fullHeight == height &&
            buffers.halfSize == mHalfSize && buffers.mips == mips && buffers.ultra == ultra)
        return true;
    releaseSsao(mDriver, buffers);

    buffers.fullWidth = width;
    buffers.fullHeight = height;
    buffers.halfSize = mHalfSize;
    buffers.mips = mips;
    buffers.ultra = ultra;
    buffers.depthWidth = (width + 1) / 2;
    buffers.depthHeight = (height + 1) / 2;
    if (mHalfSize)
    {
        buffers.depthWidth = buffers.depthWidth >> 1 > 0 ? buffers.depthWidth >> 1 : 1;
        buffers.depthHeight = buffers.depthHeight >> 1 > 0 ? buffers.depthHeight >> 1 : 1;
        buffers.bufferWidth = (width + 3) / 4;
        buffers.bufferHeight = (height + 3) / 4;
        buffers.halfBufferWidth = (width + 7) / 8;
        buffers.halfBufferHeight = (height + 7) / 8;
    }
    else
    {
        buffers.bufferWidth = (width + 1) / 2;
        buffers.bufferHeight = (height + 1) / 2;
        buffers.halfBufferWidth = (width + 3) / 4;
        buffers.halfBufferHeight = (height + 3) / 4;
    }
    buffers.depthMips = mips ? 4 : 1;

    prisma::TextureDesc desc;
    desc.usage = prisma::kTextureSampled | prisma::kTextureStorage;
    desc.type = prisma::TextureType::Texture2DArray;
    desc.format = prisma::TextureFormat::R16F;
    desc.width = buffers.depthWidth;
    desc.height = buffers.depthHeight;
    desc.depth = 4;
    desc.mipLevels = buffers.depthMips;
    desc.debugName = "ssao linear depth";
    buffers.linearDepth = mDriver->createTexture(desc);
    desc.format = prisma::TextureFormat::RG8;
    desc.width = buffers.bufferWidth;
    desc.height = buffers.bufferHeight;
    desc.mipLevels = 1;
    desc.debugName = "ssao deinterleaved";
    buffers.deinterleaved = mDriver->createTexture(desc);
    desc.debugName = "ssao deinterleaved pong";
    buffers.pong = mDriver->createTexture(desc);
    desc.type = prisma::TextureType::Texture2D;
    desc.depth = 1;
    desc.format = prisma::TextureFormat::R8;
    if (ultra)
    {
        desc.width = buffers.halfBufferWidth;
        desc.height = buffers.halfBufferHeight;
        desc.debugName = "ssao importance";
        buffers.importance = mDriver->createTexture(desc);
        desc.debugName = "ssao importance pong";
        buffers.importancePong = mDriver->createTexture(desc);
    }
    desc.width = width;
    desc.height = height;
    desc.usage = prisma::kTextureSampled | prisma::kTextureStorage | prisma::kTextureRenderTarget;
    desc.debugName = "ssao";
    buffers.finalAo = mDriver->createTexture(desc);
    const bool importanceOk =
            !ultra || (buffers.importance.valid() && buffers.importancePong.valid());
    if (!buffers.linearDepth.valid() || !buffers.deinterleaved.valid() || !buffers.pong.valid() ||
            !buffers.finalAo.valid() || !importanceOk)
    {
        releaseSsao(mDriver, buffers);
        mFailed = true;
        log_error("render: could not create the ambient occlusion targets");
        return false;
    }
    created = true;
    return true;
}

std::uint32_t ScreenSpaceEffects::generateSsao(SsaoBuffers& buffers, prisma::TextureHandle depth,
        prisma::TextureHandle normalRoughness, const Math::Mat4& projection, bool orthogonal,
        float zNear, float zFar, const SsaoSettings& settings)
{
    if (!buffers.valid() || !mPipelines->ssaoReady) return 0;
    Pipelines& p = *mPipelines;
    RenderProfiler* profiler = RenderProfiler::singleton();
    if (profiler && !p.ssaoRegistered)
    {
        p.ssaoRegistered = true;
        profiler->registerPass("SSAO");
    }

    const int quality = static_cast<int>(mQuality);
    const bool veryLow = mQuality == SsaoQuality::VeryLow;
    const bool ultra = mQuality == SsaoQuality::Ultra;
    const bool half = buffers.halfSize;
    const float fullWidth = static_cast<float>(buffers.fullWidth);
    const float fullHeight = static_cast<float>(buffers.fullHeight);

    std::uint32_t downsampleMode =
            veryLow ? kDownsampleHalfBuffers : (buffers.mips ? kDownsampleMips : kDownsample);
    if (half) ++downsampleMode;

    float linearizeMul = -projection[3].z;
    float linearizeAdd = -projection[2].z;
    if (linearizeMul * linearizeAdd < 0.0f) linearizeAdd = -linearizeAdd;

    DownsampleParams downsample;
    memset(&downsample, 0, sizeof(downsample));
    downsample.orthogonal = orthogonal ? 1u : 0u;
    downsample.zNear = linearizeMul;
    downsample.zFar = linearizeAdd;
    if (orthogonal)
    {
        downsample.zNear = zFar;
        downsample.zFar = zNear;
    }
    downsample.pixelSize[0] = 1.0f / fullWidth;
    downsample.pixelSize[1] = 1.0f / fullHeight;
    downsample.radiusSq = 1.0f;

    GatherParams gather;
    memset(&gather, 0, sizeof(gather));
    gather.screenSize[0] = static_cast<std::int32_t>(buffers.fullWidth);
    gather.screenSize[1] = static_cast<std::int32_t>(buffers.fullHeight);
    gather.halfScreenPixelSize[0] = 2.0f / fullWidth;
    gather.halfScreenPixelSize[1] = 2.0f / fullHeight;
    if (half)
    {
        gather.halfScreenPixelSize[0] *= 2.0f;
        gather.halfScreenPixelSize[1] *= 2.0f;
    }
    gather.halfScreenPixelSizeX025[0] = gather.halfScreenPixelSize[0] * 0.75f;
    gather.halfScreenPixelSizeX025[1] = gather.halfScreenPixelSize[1] * 0.75f;
    const float tanHalfFovX = 1.0f / projection[0].x;
    const float tanHalfFovY = 1.0f / projection[1].y;
    gather.ndcToViewMul[0] = tanHalfFovX * 2.0f;
    gather.ndcToViewMul[1] = tanHalfFovY * 2.0f;
    gather.ndcToViewAdd[0] = tanHalfFovX * -1.0f;
    gather.ndcToViewAdd[1] = tanHalfFovY * -1.0f;
    gather.isOrthogonal = orthogonal ? 1u : 0u;
    gather.radius = settings.radius;
    float radiusNearLimit = settings.radius * 1.2f;
    if (mQuality <= SsaoQuality::Low)
    {
        radiusNearLimit *= 1.50f;
        if (veryLow) gather.radius *= 0.8f;
    }
    radiusNearLimit /= tanHalfFovY;
    gather.intensity = settings.intensity;
    gather.shadowPower = settings.power;
    gather.shadowClamp = 0.98f;
    gather.fadeOutMul = -1.0f / (mFadeoutTo - mFadeoutFrom);
    gather.fadeOutAdd = mFadeoutFrom / (mFadeoutTo - mFadeoutFrom) + 1.0f;
    gather.horizonAngleThreshold = settings.horizon;
    gather.invRadiusNearLimit = 1.0f / radiusNearLimit;
    gather.negInvRadius = -1.0f / gather.radius;
    gather.loadCounterAvgDiv =
            9.0f / static_cast<float>(buffers.halfBufferWidth * buffers.halfBufferHeight * 255u);
    gather.adaptiveSampleLimit = mAdaptiveTarget;
    gather.detailIntensity = settings.detail;
    gather.quality = quality - 1 > 0 ? quality - 1 : 0;
    gather.sizeMultiplier = half ? 2 : 1;

    ImportanceParams importance;
    importance.halfScreenPixelSize[0] = 1.0f / static_cast<float>(buffers.bufferWidth);
    importance.halfScreenPixelSize[1] = 1.0f / static_cast<float>(buffers.bufferHeight);
    importance.intensity = settings.intensity;
    importance.power = settings.power;

    BlurParams blur;
    memset(&blur, 0, sizeof(blur));
    blur.edgeSharpness = 1.0f - settings.sharpness;
    blur.halfScreenPixelSize[0] = 1.0f / static_cast<float>(buffers.bufferWidth);
    blur.halfScreenPixelSize[1] = 1.0f / static_cast<float>(buffers.bufferHeight);

    InterleaveParams interleave;
    interleave.invSharpness = 1.0f - settings.sharpness;
    interleave.pixelSize[0] = exactReciprocal(fullWidth);
    interleave.pixelSize[1] = exactReciprocal(fullHeight);
    interleave.sizeModifier = half ? 4u : 2u;

    std::uint32_t blockCount = 0;
    auto addBlock = [&](const void* data, std::uint32_t size)
    {
        const std::uint32_t offset = blockCount * p.blockStride;
        memset(p.bytes.data() + offset, 0, p.blockStride);
        memcpy(p.bytes.data() + offset, data, size);
        ++blockCount;
        return offset;
    };

    const std::uint32_t downsampleOffset = addBlock(&downsample, sizeof(downsample));
    std::uint32_t gatherOffsets[4];
    for (std::uint32_t i = 0; i < 4; ++i)
    {
        gather.pass = static_cast<std::int32_t>(i);
        gather.passCoordOffset[0] = static_cast<std::int32_t>(i % 2);
        gather.passCoordOffset[1] = static_cast<std::int32_t>(i / 2);
        gather.passUvOffset[0] = static_cast<float>(i % 2) / fullWidth;
        gather.passUvOffset[1] = static_cast<float>(i / 2) / fullHeight;
        gatherOffsets[i] = addBlock(&gather, sizeof(gather));
    }
    const std::uint32_t importanceOffset = addBlock(&importance, sizeof(importance));
    const std::uint32_t blurPasses = mQuality > SsaoQuality::VeryLow ? mBlurPasses : 1u;
    std::uint32_t blurOffsets[6][4];
    for (std::uint32_t pass = 0; pass < blurPasses && pass < 6; ++pass)
    {
        for (std::uint32_t i = 0; i < 4; ++i)
        {
            blur.layer = static_cast<std::int32_t>(i);
            blurOffsets[pass][i] = addBlock(&blur, sizeof(blur));
        }
    }
    const std::uint32_t interleaveOffset = addBlock(&interleave, sizeof(interleave));
    mDriver->updateBuffer(p.params, 0, p.bytes.data(), blockCount * p.blockStride);

    const std::uint32_t zero = 0;
    if (ultra) mDriver->updateBuffer(p.counter, 0, &zero, sizeof(zero));

    std::uint32_t dispatches = 0;

    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSAO downsample");
        const prisma::PipelineHandle pipeline = downsamplePipeline(downsampleMode);
        if (pipeline.valid())
        {
            mDriver->beginComputePass();
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.params, downsampleOffset, sizeof(DownsampleParams));
            mDriver->bindTexture(0, depth, p.nearestSampler);
            mDriver->bindStorageTexture(0, buffers.linearDepth, 0, prisma::StorageAccess::Write);
            if (buffers.mips)
                for (std::uint32_t i = 1; i < 4; ++i)
                    mDriver->bindStorageTexture(i, buffers.linearDepth, i,
                            prisma::StorageAccess::Write);
            mDriver->dispatch(dispatchGroups(buffers.depthWidth),
                    dispatchGroups(buffers.depthHeight), 1);
            ++dispatches;
            mDriver->endComputePass();
        }
        if (scope) profiler->endGpu();
        if (!pipeline.valid())
        {
            mFailed = true;
            log_error("render: could not create the ambient occlusion pipelines");
            return 0;
        }
    }

    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSAO gather");
        const prisma::PipelineHandle regular = gatherPipeline(kGather);
        prisma::PipelineHandle baseGather;
        prisma::PipelineHandle adaptive;
        prisma::PipelineHandle generate;
        prisma::PipelineHandle processA;
        prisma::PipelineHandle processB;
        bool ok = regular.valid();
        if (ultra)
        {
            baseGather = gatherPipeline(kGatherBase);
            adaptive = gatherPipeline(kGatherAdaptive);
            generate = importancePipeline(kImportanceGenerate);
            processA = importancePipeline(kImportanceProcessA);
            processB = importancePipeline(kImportanceProcessB);
            ok = baseGather.valid() && adaptive.valid() && generate.valid() && processA.valid() &&
                 processB.valid();
        }
        if (ok)
        {
            mDriver->beginComputePass();
            const std::uint32_t threadsX =
                    half ? (buffers.fullWidth + 3) / 4 : (buffers.fullWidth + 1) / 2;
            const std::uint32_t threadsY =
                    half ? (buffers.fullHeight + 3) / 4 : (buffers.fullHeight + 1) / 2;
            auto gatherSlices = [&](prisma::PipelineHandle pipeline, prisma::TextureHandle target,
                                        bool useAdaptive)
            {
                mDriver->bindPipeline(pipeline);
                mDriver->bindUniformBuffer(1, p.constants, 0, 80 * sizeof(float));
                mDriver->bindTexture(0, buffers.linearDepth, p.mirrorSampler);
                mDriver->bindStorageTexture(0, normalRoughness, 0, prisma::StorageAccess::Read);
                mDriver->bindStorageTexture(2, target, 0, prisma::StorageAccess::Write);
                if (useAdaptive)
                {
                    mDriver->bindStorageTexture(1, buffers.pong, 0, prisma::StorageAccess::Read);
                    mDriver->bindTexture(1, buffers.importance, p.linearSampler);
                    mDriver->bindStorageBuffer(0, p.counter, 0, sizeof(std::uint32_t));
                }
                for (std::uint32_t i = 0; i < 4; ++i)
                {
                    if (veryLow && (i == 1 || i == 2)) continue;
                    mDriver->bindUniformBuffer(0, p.params, gatherOffsets[i], sizeof(GatherParams));
                    mDriver->dispatch(dispatchGroups(threadsX), dispatchGroups(threadsY), 1);
                    ++dispatches;
                }
            };
            if (ultra)
            {
                gatherSlices(baseGather, buffers.pong, false);

                mDriver->bindPipeline(generate);
                mDriver->bindUniformBuffer(0, p.params, importanceOffset, sizeof(ImportanceParams));
                mDriver->bindTexture(0, buffers.pong, p.linearSampler);
                mDriver->bindStorageTexture(0, buffers.importance, 0, prisma::StorageAccess::Write);
                mDriver->dispatch(dispatchGroups(buffers.halfBufferWidth),
                        dispatchGroups(buffers.halfBufferHeight), 1);
                ++dispatches;

                mDriver->bindPipeline(processA);
                mDriver->bindUniformBuffer(0, p.params, importanceOffset, sizeof(ImportanceParams));
                mDriver->bindTexture(0, buffers.importance, p.linearSampler);
                mDriver->bindStorageTexture(0, buffers.importancePong, 0,
                        prisma::StorageAccess::Write);
                mDriver->dispatch(dispatchGroups(buffers.halfBufferWidth),
                        dispatchGroups(buffers.halfBufferHeight), 1);
                ++dispatches;

                mDriver->bindPipeline(processB);
                mDriver->bindUniformBuffer(0, p.params, importanceOffset, sizeof(ImportanceParams));
                mDriver->bindTexture(0, buffers.importancePong, p.linearSampler);
                mDriver->bindStorageTexture(0, buffers.importance, 0, prisma::StorageAccess::Write);
                mDriver->bindStorageBuffer(0, p.counter, 0, sizeof(std::uint32_t));
                mDriver->dispatch(dispatchGroups(buffers.halfBufferWidth),
                        dispatchGroups(buffers.halfBufferHeight), 1);
                ++dispatches;

                gatherSlices(adaptive, buffers.deinterleaved, true);
            }
            else
                gatherSlices(regular, buffers.deinterleaved, false);
            mDriver->endComputePass();
        }
        if (scope) profiler->endGpu();
        if (!ok)
        {
            mFailed = true;
            log_error("render: could not create the ambient occlusion pipelines");
            return dispatches;
        }
    }

    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSAO blur");
        prisma::TextureHandle source = buffers.deinterleaved;
        prisma::TextureHandle target = buffers.pong;
        bool ok = true;
        if (blurPasses > 0)
        {
            mDriver->beginComputePass();
            for (std::uint32_t pass = 0; pass < blurPasses && pass < 6; ++pass)
            {
                std::uint32_t mode = kBlurNonSmart;
                if (mQuality > SsaoQuality::VeryLow)
                    mode = (pass + 2 < blurPasses) ? kBlurWide : kBlurSmart;
                const prisma::PipelineHandle pipeline = blurPipeline(mode);
                if (!pipeline.valid())
                {
                    ok = false;
                    break;
                }
                source = (pass % 2 == 0) ? buffers.deinterleaved : buffers.pong;
                target = (pass % 2 == 0) ? buffers.pong : buffers.deinterleaved;
                mDriver->bindPipeline(pipeline);
                mDriver->bindTexture(0, source, veryLow ? p.linearSampler : p.mirrorSampler);
                mDriver->bindStorageTexture(0, target, 0, prisma::StorageAccess::Write);
                for (std::uint32_t i = 0; i < 4; ++i)
                {
                    if (veryLow && (i == 1 || i == 2)) continue;
                    mDriver->bindUniformBuffer(0, p.params, blurOffsets[pass][i],
                            sizeof(BlurParams));
                    mDriver->dispatch(dispatchGroups(buffers.bufferWidth),
                            dispatchGroups(buffers.bufferHeight), 1);
                    ++dispatches;
                }
            }
            mDriver->endComputePass();
        }
        if (scope) profiler->endGpu();
        if (!ok)
        {
            mFailed = true;
            log_error("render: could not create the ambient occlusion pipelines");
            return dispatches;
        }
    }

    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSAO interleave");
        std::uint32_t mode = kInterleaveHalf;
        if (mQuality == SsaoQuality::Low) mode = kInterleaveNonSmart;
        else if (mQuality >= SsaoQuality::Medium)
            mode = kInterleaveSmart;
        const prisma::PipelineHandle pipeline = interleavePipeline(mode);
        if (pipeline.valid())
        {
            mDriver->beginComputePass();
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.params, interleaveOffset, sizeof(InterleaveParams));
            mDriver->bindStorageTexture(0, buffers.finalAo, 0, prisma::StorageAccess::Write);
            const bool fromDeinterleaved = mQuality > SsaoQuality::VeryLow && mBlurPasses % 2 == 0;
            mDriver->bindTexture(0, fromDeinterleaved ? buffers.deinterleaved : buffers.pong,
                    p.linearSampler);
            mDriver->dispatch(dispatchGroups(buffers.fullWidth), dispatchGroups(buffers.fullHeight),
                    1);
            ++dispatches;
            mDriver->endComputePass();
        }
        if (scope) profiler->endGpu();
        if (!pipeline.valid())
        {
            mFailed = true;
            log_error("render: could not create the ambient occlusion pipelines");
        }
    }
    return dispatches;
}

void ScreenSpaceEffects::setSsrHalfSize(bool halfSize)
{
    mSsrHalfSize = halfSize;
}

bool ScreenSpaceEffects::ssrHalfSize() const
{
    return mSsrHalfSize;
}

prisma::SamplerHandle ScreenSpaceEffects::ssrColorSampler() const
{
    return mPipelines->ssrLinearSampler;
}

prisma::SamplerHandle ScreenSpaceEffects::ssrMipSampler() const
{
    return mPipelines->ssrNearestSampler;
}

bool ScreenSpaceEffects::ensureSsr()
{
    Pipelines& p = *mPipelines;
    if (p.ssrReady) return true;
    if (mFailed || !mDriver) return false;
    if (!mDriver->caps().compute)
    {
        mFailed = true;
        log_error("render: screen space reflections need compute shaders");
        return false;
    }

    prisma::SamplerDesc samplerDesc;
    samplerDesc.minFilter = prisma::Filter::Linear;
    samplerDesc.magFilter = prisma::Filter::Linear;
    samplerDesc.mipFilter = prisma::MipFilter::Linear;
    samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
    samplerDesc.debugName = "ssr linear";
    p.ssrLinearSampler = mDriver->createSampler(samplerDesc);
    samplerDesc.minFilter = prisma::Filter::Nearest;
    samplerDesc.magFilter = prisma::Filter::Nearest;
    samplerDesc.mipFilter = prisma::MipFilter::Nearest;
    samplerDesc.debugName = "ssr nearest";
    p.ssrNearestSampler = mDriver->createSampler(samplerDesc);

    p.ssrBlockStride = alignUp(kSsrBlockSize, mDriver->caps().uniformBufferOffsetAlignment);
    prisma::BufferDesc bufferDesc;
    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.size = p.ssrBlockStride * kSsrMaxBlocks;
    bufferDesc.debugName = "ssr params";
    p.ssrParams = mDriver->createBuffer(bufferDesc);
    bufferDesc = prisma::BufferDesc();
    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.size = sizeof(SsrSceneData);
    bufferDesc.debugName = "ssr scene data";
    p.ssrScene = mDriver->createBuffer(bufferDesc);
    bufferDesc.size = sizeof(SsrCopyParams);
    bufferDesc.debugName = "ssr copy params";
    p.ssrCopyParams = mDriver->createBuffer(bufferDesc);

    if (!p.ssrLinearSampler.valid() || !p.ssrNearestSampler.valid() || !p.ssrParams.valid() ||
            !p.ssrScene.valid() || !p.ssrCopyParams.valid())
    {
        mFailed = true;
        log_error("render: could not create the screen space reflection resources");
        return false;
    }
    p.ssrBytes.resize(static_cast<std::size_t>(p.ssrBlockStride) * kSsrMaxBlocks);
    p.ssrReady = true;
    return true;
}

prisma::PipelineHandle ScreenSpaceEffects::ssrCopyPipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.ssrCopy[mode].valid()) return p.ssrCopy[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "source_color";
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "dest_buffer";
    desc.storageTextures[0].slot = 0;
    desc.debugName = "ssr copy";
    p.ssrCopy[mode] = createPipeline(*kSsrCopyBlobs[mode], desc, p.ssrCopyShaders[mode]);
    return p.ssrCopy[mode];
}

prisma::PipelineHandle ScreenSpaceEffects::ssrDownsamplePipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.ssrDownsample[mode].valid()) return p.ssrDownsample[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 2;
    desc.textures[0].name = "source_depth";
    desc.textures[0].slot = 0;
    desc.textures[1].name = "source_normal_roughness";
    desc.textures[1].slot = 1;
    desc.storageTextureCount = 2;
    desc.storageTextures[0].name = "dest_depth";
    desc.storageTextures[0].slot = 0;
    desc.storageTextures[1].name = "dest_normal_roughness";
    desc.storageTextures[1].slot = 1;
    desc.debugName = "ssr downsample";
    p.ssrDownsample[mode] =
            createPipeline(*kSsrDownsampleBlobs[mode], desc, p.ssrDownsampleShaders[mode]);
    return p.ssrDownsample[mode];
}

prisma::PipelineHandle ScreenSpaceEffects::ssrHizPipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.ssrHiz[mode].valid()) return p.ssrHiz[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "source";
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "dest";
    desc.storageTextures[0].slot = 0;
    desc.debugName = "ssr hiz";
    p.ssrHiz[mode] = createPipeline(*kSsrHizBlobs[mode], desc, p.ssrHizShaders[mode]);
    return p.ssrHiz[mode];
}

prisma::PipelineHandle ScreenSpaceEffects::ssrTracePipeline()
{
    Pipelines& p = *mPipelines;
    if (p.ssrTrace.valid()) return p.ssrTrace;
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 2;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.uniformBlocks[1].name = "SceneData";
    desc.uniformBlocks[1].slot = 1;
    desc.textureCount = 3;
    desc.textures[0].name = "source_last_frame";
    desc.textures[0].slot = 0;
    desc.textures[1].name = "source_hiz";
    desc.textures[1].slot = 1;
    desc.textures[2].name = "source_normal_roughness";
    desc.textures[2].slot = 2;
    desc.storageTextureCount = 2;
    desc.storageTextures[0].name = "output_color";
    desc.storageTextures[0].slot = 0;
    desc.storageTextures[1].name = "output_mip_level";
    desc.storageTextures[1].slot = 1;
    desc.debugName = "ssr trace";
    p.ssrTrace = createPipeline(*kSsrTraceBlobs[0], desc, p.ssrTraceShader);
    return p.ssrTrace;
}

prisma::PipelineHandle ScreenSpaceEffects::ssrFilterPipeline()
{
    Pipelines& p = *mPipelines;
    if (p.ssrFilter.valid()) return p.ssrFilter;
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].name = "source";
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "dest";
    desc.storageTextures[0].slot = 0;
    desc.debugName = "ssr filter";
    p.ssrFilter = createPipeline(*kSsrFilterBlobs[0], desc, p.ssrFilterShader);
    return p.ssrFilter;
}

prisma::PipelineHandle ScreenSpaceEffects::ssrResolvePipeline()
{
    Pipelines& p = *mPipelines;
    if (p.ssrResolve.valid()) return p.ssrResolve;
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 6;
    desc.textures[0].name = "source_depth";
    desc.textures[0].slot = 0;
    desc.textures[1].name = "source_normal_roughness";
    desc.textures[1].slot = 1;
    desc.textures[2].name = "source_depth_half";
    desc.textures[2].slot = 2;
    desc.textures[3].name = "source_normal_roughness_half";
    desc.textures[3].slot = 3;
    desc.textures[4].name = "source_color";
    desc.textures[4].slot = 4;
    desc.textures[5].name = "source_mip_level";
    desc.textures[5].slot = 5;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].name = "output_color";
    desc.storageTextures[0].slot = 0;
    desc.debugName = "ssr resolve";
    p.ssrResolve = createPipeline(*kSsrResolveBlobs[0], desc, p.ssrResolveShader);
    return p.ssrResolve;
}

void ScreenSpaceEffects::releaseSsr(prisma::Driver* driver, SsrBuffers& buffers)
{
    if (driver)
    {
        if (buffers.lastFrame.valid()) driver->destroy(buffers.lastFrame);
        if (buffers.hiz.valid()) driver->destroy(buffers.hiz);
        if (buffers.color.valid()) driver->destroy(buffers.color);
        if (buffers.mipLevel.valid()) driver->destroy(buffers.mipLevel);
        if (buffers.normalRoughness.valid()) driver->destroy(buffers.normalRoughness);
        if (buffers.resolved.valid()) driver->destroy(buffers.resolved);
    }
    buffers = SsrBuffers();
}

bool ScreenSpaceEffects::allocateSsr(SsrBuffers& buffers, std::uint32_t width, std::uint32_t height,
        bool& created)
{
    created = false;
    if (!ensureSsr()) return false;
    if (buffers.valid() && buffers.fullWidth == width && buffers.fullHeight == height &&
            buffers.halfSize == mSsrHalfSize)
        return true;
    releaseSsr(mDriver, buffers);

    buffers.fullWidth = width;
    buffers.fullHeight = height;
    buffers.halfSize = mSsrHalfSize;
    buffers.width = mSsrHalfSize ? width / 2 : width;
    buffers.height = mSsrHalfSize ? height / 2 : height;
    if (buffers.width < 1) buffers.width = 1;
    if (buffers.height < 1) buffers.height = 1;

    std::uint32_t currentWidth = buffers.width;
    std::uint32_t currentHeight = buffers.height;
    buffers.mipmaps = 1;
    while (currentWidth > 1 && currentHeight > 1)
    {
        if (currentWidth > 1) currentWidth /= 2;
        if (currentHeight > 1) currentHeight /= 2;
        ++buffers.mipmaps;
    }

    const std::uint32_t usage = prisma::kTextureSampled | prisma::kTextureStorage;
    prisma::TextureDesc desc;
    desc.usage = usage;
    desc.width = buffers.width;
    desc.height = buffers.height;
    if (mSsrHalfSize)
    {
        desc.format = prisma::TextureFormat::RGBA8;
        desc.debugName = "ssr normal roughness";
        buffers.normalRoughness = mDriver->createTexture(desc);
    }
    desc.format = prisma::TextureFormat::R32F;
    desc.mipLevels = buffers.mipmaps;
    desc.debugName = "ssr hiz";
    buffers.hiz = mDriver->createTexture(desc);
    desc.format = prisma::TextureFormat::RGBA16F;
    desc.debugName = "ssr";
    buffers.color = mDriver->createTexture(desc);
    desc.format = prisma::TextureFormat::RGBA16F;
    desc.mipLevels = 1;
    desc.debugName = "ssr last frame";
    buffers.lastFrame = mDriver->createTexture(desc);
    desc.format = prisma::TextureFormat::R8;
    desc.debugName = "ssr mip level";
    buffers.mipLevel = mDriver->createTexture(desc);
    if (mSsrHalfSize)
    {
        desc.format = prisma::TextureFormat::RGBA16F;
        desc.width = width;
        desc.height = height;
        desc.debugName = "ssr resolved";
        buffers.resolved = mDriver->createTexture(desc);
    }
    const bool halfOk =
            !mSsrHalfSize || (buffers.normalRoughness.valid() && buffers.resolved.valid());
    if (!buffers.hiz.valid() || !buffers.color.valid() || !buffers.lastFrame.valid() ||
            !buffers.mipLevel.valid() || !halfOk)
    {
        releaseSsr(mDriver, buffers);
        mFailed = true;
        log_error("render: could not create the screen space reflection targets");
        return false;
    }
    ct::Vector<std::uint8_t> zeros;
    zeros.resize(static_cast<std::size_t>(buffers.width) * buffers.height * 8);
    memset(zeros.data(), 0, zeros.size());
    mDriver->updateTexture(buffers.lastFrame, 0, 0, zeros.data());
    created = true;
    return true;
}

std::uint32_t ScreenSpaceEffects::copyLastFrame(SsrBuffers& buffers, prisma::TextureHandle color)
{
    if (!buffers.valid() || !mPipelines->ssrReady) return 0;
    Pipelines& p = *mPipelines;
    RenderProfiler* profiler = RenderProfiler::singleton();
    const bool scope = profiler != nullptr && profiler->beginGpu("SSR last frame");
    SsrCopyParams params;
    memset(&params, 0, sizeof(params));
    params.section[2] = static_cast<std::int32_t>(buffers.width);
    params.section[3] = static_cast<std::int32_t>(buffers.height);
    if (!buffers.halfSize) params.flags = kSsrFlagSanitize;
    mDriver->updateBuffer(p.ssrCopyParams, 0, &params, sizeof(params));
    const prisma::PipelineHandle pipeline =
            ssrCopyPipeline(buffers.halfSize ? kSsrCopyMipmap : kSsrCopySimple);
    std::uint32_t dispatches = 0;
    if (pipeline.valid())
    {
        mDriver->beginComputePass();
        mDriver->bindPipeline(pipeline);
        mDriver->bindUniformBuffer(0, p.ssrCopyParams, 0, sizeof(SsrCopyParams));
        mDriver->bindTexture(0, color, p.ssrNearestSampler);
        mDriver->bindStorageTexture(0, buffers.lastFrame, 0, prisma::StorageAccess::Write);
        mDriver->dispatch(dispatchGroups(buffers.width), dispatchGroups(buffers.height), 1);
        mDriver->endComputePass();
        ++dispatches;
        buffers.haveLast = true;
    }
    if (scope) profiler->endGpu();
    if (!pipeline.valid())
    {
        mFailed = true;
        log_error("render: could not create the screen space reflection pipelines");
    }
    return dispatches;
}

std::uint32_t ScreenSpaceEffects::generateSsr(SsrBuffers& buffers, prisma::TextureHandle depth,
        prisma::TextureHandle normalRoughness, const Math::Mat4& projection, const Math::Mat4& view,
        bool orthogonal, const SsrSettings& settings)
{
    if (!buffers.valid() || !mPipelines->ssrReady) return 0;
    Pipelines& p = *mPipelines;
    RenderProfiler* profiler = RenderProfiler::singleton();
    if (profiler && !p.ssrRegistered)
    {
        p.ssrRegistered = true;
        profiler->registerPass("SSR");
    }

    const bool half = buffers.halfSize;
    const std::int32_t fullWidth = static_cast<std::int32_t>(buffers.fullWidth);
    const std::int32_t fullHeight = static_cast<std::int32_t>(buffers.fullHeight);
    const std::int32_t width = static_cast<std::int32_t>(buffers.width);
    const std::int32_t height = static_cast<std::int32_t>(buffers.height);
    const std::uint32_t levels = buffers.mipmaps - 1 < 32 ? buffers.mipmaps - 1 : 32;

    Math::Mat4 reprojection = Math::Mat4::Identity();
    if (buffers.haveLast)
        reprojection = buffers.lastProjection * buffers.lastView * view.InverseAffine() *
                       projection.Inverse();
    buffers.lastProjection = projection;
    buffers.lastView = view;

    SsrSceneData scene;
    memset(&scene, 0, sizeof(scene));
    const Math::Mat4 inverseProjection = projection.Inverse();
    for (int v = 0; v < 2; ++v)
    {
        memcpy(scene.projection[v], &projection, sizeof(float) * 16);
        memcpy(scene.invProjection[v], &inverseProjection, sizeof(float) * 16);
        memcpy(scene.reprojection[v], &reprojection, sizeof(float) * 16);
    }
    mDriver->updateBuffer(p.ssrScene, 0, &scene, sizeof(scene));

    auto oddMode = [](std::uint32_t w, std::uint32_t h)
    {
        const bool oddWidth = (w % 2) != 0;
        const bool oddHeight = (h % 2) != 0;
        return oddWidth && oddHeight ? 3u : oddWidth ? 1u : oddHeight ? 2u : 0u;
    };
    auto mipSize = [&](std::uint32_t size, std::uint32_t level)
    {
        const std::uint32_t value = size >> level;
        return value > 1 ? value : 1u;
    };

    std::uint32_t blockCount = 0;
    auto addBlock = [&](const void* data, std::uint32_t size)
    {
        const std::uint32_t offset = blockCount * p.ssrBlockStride;
        memset(p.ssrBytes.data() + offset, 0, p.ssrBlockStride);
        memcpy(p.ssrBytes.data() + offset, data, size);
        ++blockCount;
        return offset;
    };

    std::uint32_t firstOffset = 0;
    if (half)
    {
        SsrSizeParams params;
        memset(&params, 0, sizeof(params));
        params.screenSize[0] = width;
        params.screenSize[1] = height;
        firstOffset = addBlock(&params, sizeof(params));
    }
    else
    {
        SsrCopyParams params;
        memset(&params, 0, sizeof(params));
        params.section[2] = width;
        params.section[3] = height;
        firstOffset = addBlock(&params, sizeof(params));
    }
    std::uint32_t hizOffsets[32];
    std::uint32_t hizModes[32];
    std::uint32_t filterOffsets[32];
    for (std::uint32_t i = 0; i < levels; ++i)
    {
        SsrSizeParams params;
        memset(&params, 0, sizeof(params));
        params.screenSize[0] = static_cast<std::int32_t>(mipSize(buffers.width, i + 1));
        params.screenSize[1] = static_cast<std::int32_t>(mipSize(buffers.height, i + 1));
        params.pad[0] = static_cast<std::int32_t>(i);
        hizModes[i] = oddMode(mipSize(buffers.width, i), mipSize(buffers.height, i));
        hizOffsets[i] = addBlock(&params, sizeof(params));
    }
    SsrTraceParams trace;
    memset(&trace, 0, sizeof(trace));
    trace.screenSize[0] = width;
    trace.screenSize[1] = height;
    trace.mipmaps = static_cast<std::int32_t>(buffers.mipmaps);
    trace.numSteps = settings.maxSteps;
    trace.distanceFade = settings.fadeOut;
    trace.curveFadeIn = settings.fadeIn;
    trace.depthTolerance = settings.depthTolerance;
    trace.orthogonal = orthogonal ? 1u : 0u;
    const std::uint32_t traceOffset = addBlock(&trace, sizeof(trace));
    for (std::uint32_t i = 0; i < levels; ++i)
    {
        SsrFilterParams params;
        memset(&params, 0, sizeof(params));
        params.screenSize[0] = static_cast<std::int32_t>(mipSize(buffers.width, i + 1));
        params.screenSize[1] = static_cast<std::int32_t>(mipSize(buffers.height, i + 1));
        params.mipLevel = i + 1;
        filterOffsets[i] = addBlock(&params, sizeof(params));
    }
    std::uint32_t resolveOffset = 0;
    if (half)
    {
        SsrSizeParams params;
        memset(&params, 0, sizeof(params));
        params.screenSize[0] = fullWidth;
        params.screenSize[1] = fullHeight;
        resolveOffset = addBlock(&params, sizeof(params));
    }
    mDriver->updateBuffer(p.ssrParams, 0, p.ssrBytes.data(), blockCount * p.ssrBlockStride);

    std::uint32_t dispatches = 0;
    bool ok = true;

    {
        const bool scope = profiler != nullptr &&
                           profiler->beginGpu(half ? "SSR downsample" : "SSR copy depth");
        const prisma::PipelineHandle pipeline =
                half ? ssrDownsamplePipeline(oddMode(buffers.fullWidth, buffers.fullHeight))
                     : ssrCopyPipeline(kSsrCopyDepth);
        if (pipeline.valid())
        {
            mDriver->beginComputePass();
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.ssrParams, firstOffset,
                    half ? sizeof(SsrSizeParams) : sizeof(SsrCopyParams));
            mDriver->bindTexture(0, depth, p.ssrNearestSampler);
            mDriver->bindStorageTexture(0, buffers.hiz, 0, prisma::StorageAccess::Write);
            if (half)
            {
                mDriver->bindTexture(1, normalRoughness, p.ssrNearestSampler);
                mDriver->bindStorageTexture(1, buffers.normalRoughness, 0,
                        prisma::StorageAccess::Write);
            }
            mDriver->dispatch(dispatchGroups(buffers.width), dispatchGroups(buffers.height), 1);
            mDriver->endComputePass();
            ++dispatches;
        }
        else
            ok = false;
        if (scope) profiler->endGpu();
    }

    if (ok && levels > 0)
    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSR hi-z");
        mDriver->beginComputePass();
        for (std::uint32_t i = 0; i < levels && ok; ++i)
        {
            const prisma::PipelineHandle pipeline = ssrHizPipeline(hizModes[i]);
            if (!pipeline.valid())
            {
                ok = false;
                break;
            }
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.ssrParams, hizOffsets[i], sizeof(SsrSizeParams));
            mDriver->bindTexture(0, buffers.hiz, p.ssrNearestSampler);
            mDriver->bindStorageTexture(0, buffers.hiz, i + 1, prisma::StorageAccess::Write);
            mDriver->dispatch(dispatchGroups(mipSize(buffers.width, i + 1)),
                    dispatchGroups(mipSize(buffers.height, i + 1)), 1);
            ++dispatches;
        }
        mDriver->endComputePass();
        if (scope) profiler->endGpu();
    }

    if (ok)
    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSR main");
        const prisma::PipelineHandle pipeline = ssrTracePipeline();
        if (pipeline.valid())
        {
            mDriver->beginComputePass();
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.ssrParams, traceOffset, sizeof(SsrTraceParams));
            mDriver->bindUniformBuffer(1, p.ssrScene, 0, sizeof(SsrSceneData));
            mDriver->bindTexture(0, buffers.lastFrame, p.ssrLinearSampler);
            mDriver->bindTexture(1, buffers.hiz, p.ssrNearestSampler);
            mDriver->bindTexture(2, half ? buffers.normalRoughness : normalRoughness,
                    p.ssrNearestSampler);
            mDriver->bindStorageTexture(0, buffers.color, 0, prisma::StorageAccess::Write);
            mDriver->bindStorageTexture(1, buffers.mipLevel, 0, prisma::StorageAccess::Write);
            mDriver->dispatch(dispatchGroups(buffers.width), dispatchGroups(buffers.height), 1);
            mDriver->endComputePass();
            ++dispatches;
        }
        else
            ok = false;
        if (scope) profiler->endGpu();
    }

    if (ok && levels > 0)
    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSR filter");
        const prisma::PipelineHandle pipeline = ssrFilterPipeline();
        if (pipeline.valid())
        {
            mDriver->beginComputePass();
            mDriver->bindPipeline(pipeline);
            for (std::uint32_t i = 0; i < levels; ++i)
            {
                mDriver->bindUniformBuffer(0, p.ssrParams, filterOffsets[i],
                        sizeof(SsrFilterParams));
                mDriver->bindTexture(0, buffers.color, p.ssrLinearSampler);
                mDriver->bindStorageTexture(0, buffers.color, i + 1, prisma::StorageAccess::Write);
                mDriver->dispatch(dispatchGroups(mipSize(buffers.width, i + 1)),
                        dispatchGroups(mipSize(buffers.height, i + 1)), 1);
                ++dispatches;
            }
            mDriver->endComputePass();
        }
        else
            ok = false;
        if (scope) profiler->endGpu();
    }

    if (ok && half)
    {
        const bool scope = profiler != nullptr && profiler->beginGpu("SSR resolve");
        const prisma::PipelineHandle pipeline = ssrResolvePipeline();
        if (pipeline.valid())
        {
            mDriver->beginComputePass();
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.ssrParams, resolveOffset, sizeof(SsrSizeParams));
            mDriver->bindTexture(0, depth, p.ssrNearestSampler);
            mDriver->bindTexture(1, normalRoughness, p.ssrNearestSampler);
            mDriver->bindTexture(2, buffers.hiz, p.ssrNearestSampler);
            mDriver->bindTexture(3, buffers.normalRoughness, p.ssrNearestSampler);
            mDriver->bindTexture(4, buffers.color, p.ssrLinearSampler);
            mDriver->bindTexture(5, buffers.mipLevel, p.ssrNearestSampler);
            mDriver->bindStorageTexture(0, buffers.resolved, 0, prisma::StorageAccess::Write);
            mDriver->dispatch(dispatchGroups(buffers.fullWidth), dispatchGroups(buffers.fullHeight),
                    1);
            mDriver->endComputePass();
            ++dispatches;
        }
        else
            ok = false;
        if (scope) profiler->endGpu();
    }

    if (!ok)
    {
        mFailed = true;
        log_error("render: could not create the screen space reflection pipelines");
    }
    return dispatches;
}

} // namespace prisma3d
