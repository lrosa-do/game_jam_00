#include "DepthOfField.h"
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

const std::uint32_t kMaxBlocks = 8;
const std::uint32_t kBlockSize = 96;

enum ComputeMode : std::uint32_t
{
    kComputeBlurSize,
    kComputeBox,
    kComputeHexagon,
    kComputeCircular,
    kComputeComposite,
    kComputeModes
};

enum RasterMode : std::uint32_t
{
    kRasterBlurSize,
    kRasterBox,
    kRasterBoxNoWeight,
    kRasterHexagon,
    kRasterHexagonNoWeight,
    kRasterCircular,
    kRasterComposite,
    kRasterModes
};

struct DofParams
{
    std::int32_t size[2];
    float zFar;
    float zNear;
    std::uint32_t orthogonal;
    float blurSize;
    float blurScale;
    std::int32_t steps;
    std::uint32_t blurNearActive;
    float blurNearBegin;
    float blurNearEnd;
    std::uint32_t blurFarActive;
    float blurFarBegin;
    float blurFarEnd;
    std::uint32_t secondPass;
    std::uint32_t halfSize;
    std::uint32_t useJitter;
    float jitterSeed;
    std::uint32_t usePhysicalNear;
    std::uint32_t usePhysicalFar;
    float blurSizeNear;
    float blurSizeFar;
    std::uint32_t pad[2];
};

static_assert(sizeof(DofParams) == kBlockSize, "DofParams must match the std140 block");

const std::int32_t kQualitySamples[4] = { 6, 12, 12, 24 };
const float kQualityScale[4] = { 8.0f, 4.0f, 1.0f, 0.5f };

std::uint32_t alignUp(std::uint32_t value, std::uint32_t alignment)
{
    return alignment > 1 ? (value + alignment - 1) / alignment * alignment : value;
}

std::uint32_t dispatchGroups(std::uint32_t threads)
{
    return (threads + 7u) / 8u;
}

std::uint32_t halfExtent(std::uint32_t size)
{
    const std::uint32_t half = size >> 1;
    return half > 0 ? half : 1u;
}

} // namespace

struct DepthOfField::Pipelines
{
    prisma::ShaderHandle computeShaders[kComputeModes];
    prisma::PipelineHandle compute[kComputeModes];
    prisma::ShaderHandle vertexShader;
    prisma::ShaderHandle rasterShaders[kRasterModes];
    prisma::PipelineHandle raster[kRasterModes];
    prisma::SamplerHandle linearSampler;
    prisma::SamplerHandle nearestSampler;
    prisma::BufferHandle params;
    ct::Vector<std::uint8_t> bytes;
    std::uint32_t blockStride = 0;
    bool ready = false;
    bool registered = false;
};

DepthOfField::DepthOfField()
        : mDriver(nullptr),
          mFailed(false),
          mComputeAllowed(true),
          mShape(DofBokehShape::Hexagon),
          mQuality(DofBlurQuality::Low),
          mJitter(false),
          mFrame(0),
          mPipelines(new Pipelines())
{
}

DepthOfField::~DepthOfField()
{
    destroy();
    delete mPipelines;
}

bool DepthOfField::create(prisma::Driver* driver)
{
    destroy();
    mDriver = driver;
    mFailed = false;
    return driver != nullptr;
}

void DepthOfField::destroy()
{
    if (!mDriver) return;
    Pipelines& p = *mPipelines;
    for (std::uint32_t i = 0; i < kComputeModes; ++i)
    {
        if (p.compute[i].valid()) mDriver->destroy(p.compute[i]);
        if (p.computeShaders[i].valid()) mDriver->destroy(p.computeShaders[i]);
    }
    for (std::uint32_t i = 0; i < kRasterModes; ++i)
    {
        if (p.raster[i].valid()) mDriver->destroy(p.raster[i]);
        if (p.rasterShaders[i].valid()) mDriver->destroy(p.rasterShaders[i]);
    }
    if (p.vertexShader.valid()) mDriver->destroy(p.vertexShader);
    if (p.linearSampler.valid()) mDriver->destroy(p.linearSampler);
    if (p.nearestSampler.valid()) mDriver->destroy(p.nearestSampler);
    if (p.params.valid()) mDriver->destroy(p.params);
    delete mPipelines;
    mPipelines = new Pipelines();
    mDriver = nullptr;
}

bool DepthOfField::failed() const
{
    return mFailed;
}

void DepthOfField::setBokeh(DofBokehShape shape, DofBlurQuality quality, bool useJitter)
{
    mShape = shape;
    mQuality = quality;
    mJitter = useJitter;
}

DofBokehShape DepthOfField::shape() const
{
    return mShape;
}

DofBlurQuality DepthOfField::quality() const
{
    return mQuality;
}

bool DepthOfField::useJitter() const
{
    return mJitter;
}

void DepthOfField::setComputeAllowed(bool allowed)
{
    mComputeAllowed = allowed;
}

bool DepthOfField::computeAllowed() const
{
    return mComputeAllowed;
}

bool DepthOfField::usesCompute() const
{
    return mComputeAllowed && mDriver != nullptr && mDriver->caps().compute;
}

std::uint32_t DepthOfField::colorUsage(bool compute)
{
    std::uint32_t usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
    if (compute) usage |= prisma::kTextureStorage;
    return usage;
}

bool DepthOfField::ensure()
{
    Pipelines& p = *mPipelines;
    if (p.ready) return true;
    if (mFailed || !mDriver) return false;

    prisma::SamplerDesc samplerDesc;
    samplerDesc.minFilter = prisma::Filter::Linear;
    samplerDesc.magFilter = prisma::Filter::Linear;
    samplerDesc.mipFilter = prisma::MipFilter::None;
    samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
    samplerDesc.debugName = "dof linear";
    p.linearSampler = mDriver->createSampler(samplerDesc);
    samplerDesc.minFilter = prisma::Filter::Nearest;
    samplerDesc.magFilter = prisma::Filter::Nearest;
    samplerDesc.debugName = "dof nearest";
    p.nearestSampler = mDriver->createSampler(samplerDesc);

    p.blockStride = alignUp(kBlockSize, mDriver->caps().uniformBufferOffsetAlignment);
    prisma::BufferDesc bufferDesc;
    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.size = p.blockStride * kMaxBlocks;
    bufferDesc.debugName = "dof params";
    p.params = mDriver->createBuffer(bufferDesc);

    if (!p.linearSampler.valid() || !p.nearestSampler.valid() || !p.params.valid())
    {
        mFailed = true;
        log_error("render: could not create the depth of field resources");
        return false;
    }
    p.bytes.resize(static_cast<std::size_t>(p.blockStride) * kMaxBlocks);
    p.ready = true;
    return true;
}

prisma::PipelineHandle DepthOfField::computePipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.compute[mode].valid()) return p.compute[mode];
    prisma::ComputePipelineDesc desc;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 1;
    desc.textures[0].slot = 0;
    desc.storageTextureCount = 1;
    desc.storageTextures[0].slot = 0;
    switch (mode)
    {
        case kComputeBlurSize:
            desc.textures[0].name = "source_depth";
            desc.storageTextures[0].name = "color_image";
            desc.debugName = "dof blur size";
            break;
        case kComputeComposite:
            desc.textures[0].name = "source_bokeh";
            desc.storageTextures[0].name = "color_image";
            desc.debugName = "dof composite";
            break;
        default:
            desc.textures[0].name = "color_texture";
            desc.storageTextures[0].name = "bokeh_image";
            desc.debugName = mode == kComputeBox       ? "dof box"
                             : mode == kComputeHexagon ? "dof hexagon"
                                                       : "dof circular";
            break;
    }
    p.computeShaders[mode] =
            mDriver->createShader(prisma3d::shaderDesc(*kDofComputeBlobs[mode], mDriver->caps()));
    if (!p.computeShaders[mode].valid()) return prisma::PipelineHandle();
    desc.shader = p.computeShaders[mode];
    p.compute[mode] = mDriver->createComputePipeline(desc);
    return p.compute[mode];
}

prisma::PipelineHandle DepthOfField::rasterPipeline(std::uint32_t mode)
{
    Pipelines& p = *mPipelines;
    if (p.raster[mode].valid()) return p.raster[mode];
    if (!p.vertexShader.valid())
        p.vertexShader = mDriver->createShader(prisma3d::shaderDesc(tonemap_vert, mDriver->caps()));
    p.rasterShaders[mode] =
            mDriver->createShader(prisma3d::shaderDesc(*kDofRasterBlobs[mode], mDriver->caps()));
    if (!p.vertexShader.valid() || !p.rasterShaders[mode].valid()) return prisma::PipelineHandle();

    prisma::PipelineDesc desc;
    desc.vertexShader = p.vertexShader;
    desc.fragmentShader = p.rasterShaders[mode];
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.targets.window = false;
    desc.targets.depth = prisma::TextureFormat::None;
    desc.depthTest = false;
    desc.depthWrite = false;
    switch (mode)
    {
        case kRasterBlurSize:
            desc.textureCount = 1;
            desc.textures[0].name = "source_depth";
            desc.textures[0].slot = 0;
            desc.targets.colorCount = 1;
            desc.targets.colors[0] = prisma::TextureFormat::R16F;
            desc.debugName = "dof blur size raster";
            break;
        case kRasterComposite:
            desc.textureCount = 3;
            desc.textures[0].name = "source_color";
            desc.textures[0].slot = 0;
            desc.textures[1].name = "source_weight";
            desc.textures[1].slot = 1;
            desc.textures[2].name = "original_weight";
            desc.textures[2].slot = 2;
            desc.targets.colorCount = 1;
            desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
            desc.blend = true;
            desc.srcColor = prisma::BlendFactor::SrcAlpha;
            desc.dstColor = prisma::BlendFactor::OneMinusSrcAlpha;
            desc.srcAlpha = prisma::BlendFactor::SrcAlpha;
            desc.dstAlpha = prisma::BlendFactor::OneMinusSrcAlpha;
            desc.debugName = "dof composite raster";
            break;
        default:
        {
            desc.textureCount = 2;
            desc.textures[0].name = "source_color";
            desc.textures[0].slot = 0;
            desc.textures[1].name = "source_weight";
            desc.textures[1].slot = 1;
            const bool weight = mode != kRasterBoxNoWeight && mode != kRasterHexagonNoWeight;
            desc.targets.colorCount = weight ? 2 : 1;
            desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
            if (weight) desc.targets.colors[1] = prisma::TextureFormat::R16F;
            desc.debugName = "dof bokeh raster";
            break;
        }
    }
    p.raster[mode] = mDriver->createPipeline(desc);
    return p.raster[mode];
}

void DepthOfField::release(prisma::Driver* driver, DofBuffers& buffers)
{
    if (driver)
    {
        if (buffers.secondary.valid()) driver->destroy(buffers.secondary);
        if (buffers.half[0].valid()) driver->destroy(buffers.half[0]);
        if (buffers.half[1].valid()) driver->destroy(buffers.half[1]);
        for (const prisma::TextureHandle& texture: buffers.weight)
            if (texture.valid()) driver->destroy(texture);
    }
    buffers = DofBuffers();
}

bool DepthOfField::allocate(DofBuffers& buffers, std::uint32_t width, std::uint32_t height,
        bool& created)
{
    created = false;
    if (!ensure()) return false;
    const bool raster = !usesCompute();
    if (buffers.valid() && buffers.width == width && buffers.height == height &&
            buffers.raster == raster)
        return true;
    release(mDriver, buffers);

    buffers.width = width;
    buffers.height = height;
    buffers.raster = raster;
    const std::uint32_t usage = raster ? (prisma::kTextureSampled | prisma::kTextureRenderTarget)
                                       : (prisma::kTextureSampled | prisma::kTextureStorage);
    prisma::TextureDesc desc;
    desc.format = prisma::TextureFormat::RGBA16F;
    desc.usage = usage;
    desc.width = width;
    desc.height = height;
    desc.debugName = "dof secondary";
    buffers.secondary = mDriver->createTexture(desc);
    desc.width = halfExtent(width);
    desc.height = halfExtent(height);
    desc.debugName = "dof half 0";
    buffers.half[0] = mDriver->createTexture(desc);
    desc.debugName = "dof half 1";
    buffers.half[1] = mDriver->createTexture(desc);
    bool ok = buffers.secondary.valid() && buffers.half[0].valid() && buffers.half[1].valid();
    if (raster)
    {
        desc.format = prisma::TextureFormat::R16F;
        for (std::uint32_t i = 0; i < 4; ++i)
        {
            desc.width = i < 2 ? width : halfExtent(width);
            desc.height = i < 2 ? height : halfExtent(height);
            desc.debugName = "dof weight";
            buffers.weight[i] = mDriver->createTexture(desc);
            ok = ok && buffers.weight[i].valid();
        }
    }
    if (!ok)
    {
        release(mDriver, buffers);
        mFailed = true;
        log_error("render: could not create the depth of field targets");
        return false;
    }
    created = true;
    return true;
}

std::uint32_t DepthOfField::run(DofBuffers& buffers, prisma::TextureHandle color,
        prisma::TextureHandle depth, const CameraAttributesSettings& attributes, float zNear,
        float zFar, bool orthogonal)
{
    if (!buffers.valid() || !mPipelines->ready) return 0;
    RenderProfiler* profiler = RenderProfiler::singleton();
    if (profiler && !mPipelines->registered)
    {
        mPipelines->registered = true;
        profiler->registerPass("DOF");
    }
    ++mFrame;
    const std::uint32_t passes =
            buffers.raster ? runRaster(buffers, color, depth, attributes, zNear, zFar, orthogonal)
                           : runCompute(buffers, color, depth, attributes, zNear, zFar, orthogonal);
    if (passes == 0 && !mFailed)
    {
        mFailed = true;
        log_error("render: could not create the depth of field pipelines");
    }
    return passes;
}

std::uint32_t DepthOfField::runCompute(DofBuffers& buffers, prisma::TextureHandle color,
        prisma::TextureHandle depth, const CameraAttributesSettings& attributes, float zNear,
        float zFar, bool orthogonal)
{
    Pipelines& p = *mPipelines;
    RenderProfiler* profiler = RenderProfiler::singleton();
    const std::uint32_t quality = static_cast<std::uint32_t>(mQuality);
    const bool boxOrHexagon = mShape != DofBokehShape::Circle;
    const bool lowQuality = quality <= 1;

    const bool farEnabled = attributes.dofBlurFarEnabled;
    const float farBegin = attributes.dofBlurFarDistance;
    const float farSize = attributes.dofBlurFarTransition;
    const bool nearEnabled = attributes.dofBlurNearEnabled;
    const float nearBegin = attributes.dofBlurNearDistance;
    const float nearSize = attributes.dofBlurNearTransition;
    const float bokehSize = attributes.dofBlurAmount * 64.0f;

    DofParams pc;
    memset(&pc, 0, sizeof(pc));
    pc.blurFarActive = farEnabled ? 1u : 0u;
    pc.blurFarBegin = farBegin;
    pc.blurFarEnd = farBegin + farSize;
    pc.usePhysicalFar = farSize < 0.0f ? 1u : 0u;
    pc.blurSizeFar = bokehSize;
    pc.blurNearActive = nearEnabled ? 1u : 0u;
    pc.blurNearBegin = nearBegin;
    pc.blurNearEnd = nearBegin - nearSize;
    pc.usePhysicalNear = nearSize < 0.0f ? 1u : 0u;
    pc.blurSizeNear = bokehSize;
    pc.useJitter = mJitter ? 1u : 0u;
    pc.jitterSeed = fmodf(static_cast<float>(mFrame) * 0.61803398875f, 1.0f) * 1000.0f;
    pc.zNear = zNear;
    pc.zFar = zFar;
    pc.orthogonal = orthogonal ? 1u : 0u;
    pc.blurSize = (nearSize < 0.0f && farSize < 0.0f) ? 32.0f : bokehSize;
    pc.secondPass = 0;
    pc.halfSize = 0;
    pc.blurScale = 0.5f;
    pc.size[0] = static_cast<std::int32_t>(buffers.width);
    pc.size[1] = static_cast<std::int32_t>(buffers.height);

    std::uint32_t blockCount = 0;
    auto addBlock = [&](const DofParams& params)
    {
        const std::uint32_t offset = blockCount * p.blockStride;
        memset(p.bytes.data() + offset, 0, p.blockStride);
        memcpy(p.bytes.data() + offset, &params, sizeof(params));
        ++blockCount;
        return offset;
    };

    const std::uint32_t fullGroupsX = dispatchGroups(buffers.width);
    const std::uint32_t fullGroupsY = dispatchGroups(buffers.height);

    std::uint32_t offsets[4] = { 0, 0, 0, 0 };
    std::uint32_t pass1Groups[2] = { 0, 0 };
    offsets[0] = addBlock(pc);

    if (boxOrHexagon)
    {
        pc.steps = kQualitySamples[quality];
        if (lowQuality)
        {
            pc.size[0] = static_cast<std::int32_t>(buffers.width >> 1);
            pc.size[1] = static_cast<std::int32_t>(buffers.height >> 1);
            pc.halfSize = 1;
            pc.blurSize *= 0.5f;
        }
        pass1Groups[0] = dispatchGroups(static_cast<std::uint32_t>(pc.size[0]));
        pass1Groups[1] = dispatchGroups(static_cast<std::uint32_t>(pc.size[1]));
        offsets[1] = addBlock(pc);
        pc.secondPass = 1;
        offsets[2] = addBlock(pc);
        if (lowQuality)
        {
            pc.size[0] = static_cast<std::int32_t>(buffers.width);
            pc.size[1] = static_cast<std::int32_t>(buffers.height);
            pc.halfSize = 0;
            pc.secondPass = 0;
            offsets[3] = addBlock(pc);
        }
    }
    else
    {
        pc.steps = 0;
        pc.blurScale = kQualityScale[quality];
        pc.size[0] = static_cast<std::int32_t>(buffers.width >> 1);
        pc.size[1] = static_cast<std::int32_t>(buffers.height >> 1);
        pc.halfSize = 1;
        pass1Groups[0] = dispatchGroups(static_cast<std::uint32_t>(pc.size[0]));
        pass1Groups[1] = dispatchGroups(static_cast<std::uint32_t>(pc.size[1]));
        offsets[1] = addBlock(pc);
        pc.size[0] = static_cast<std::int32_t>(buffers.width);
        pc.size[1] = static_cast<std::int32_t>(buffers.height);
        pc.halfSize = 0;
        pc.secondPass = 0;
        offsets[2] = addBlock(pc);
    }
    mDriver->updateBuffer(p.params, 0, p.bytes.data(), blockCount * p.blockStride);

    std::uint32_t dispatches = 0;
    bool ok = true;

    auto dispatchPass = [&](const char* name, std::uint32_t mode, std::uint32_t offset,
                                prisma::TextureHandle sampled, prisma::SamplerHandle sampler,
                                prisma::TextureHandle storage, prisma::StorageAccess access,
                                std::uint32_t groupsX, std::uint32_t groupsY)
    {
        if (!ok) return;
        const bool scope = profiler != nullptr && profiler->beginGpu(name);
        const prisma::PipelineHandle pipeline = computePipeline(mode);
        if (pipeline.valid())
        {
            mDriver->beginComputePass();
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.params, offset, kBlockSize);
            mDriver->bindTexture(0, sampled, sampler);
            mDriver->bindStorageTexture(0, storage, 0, access);
            mDriver->dispatch(groupsX, groupsY, 1);
            mDriver->endComputePass();
            ++dispatches;
        }
        else
            ok = false;
        if (scope) profiler->endGpu();
    };

    dispatchPass("DOF blur size", kComputeBlurSize, offsets[0], depth, p.nearestSampler, color,
            prisma::StorageAccess::ReadWrite, fullGroupsX, fullGroupsY);

    if (boxOrHexagon)
    {
        const std::uint32_t mode = mShape == DofBokehShape::Box ? kComputeBox : kComputeHexagon;
        if (lowQuality)
        {
            dispatchPass("DOF bokeh 1", mode, offsets[1], color, p.linearSampler, buffers.half[0],
                    prisma::StorageAccess::Write, pass1Groups[0], pass1Groups[1]);
            dispatchPass("DOF bokeh 2", mode, offsets[2], buffers.half[0], p.linearSampler,
                    buffers.half[1], prisma::StorageAccess::Write, pass1Groups[0], pass1Groups[1]);
            dispatchPass("DOF composite", kComputeComposite, offsets[3], buffers.half[1],
                    p.linearSampler, color, prisma::StorageAccess::ReadWrite, fullGroupsX,
                    fullGroupsY);
        }
        else
        {
            dispatchPass("DOF bokeh 1", mode, offsets[1], color, p.linearSampler, buffers.secondary,
                    prisma::StorageAccess::Write, pass1Groups[0], pass1Groups[1]);
            dispatchPass("DOF bokeh 2", mode, offsets[2], buffers.secondary, p.linearSampler, color,
                    prisma::StorageAccess::Write, pass1Groups[0], pass1Groups[1]);
        }
    }
    else
    {
        dispatchPass("DOF bokeh 1", kComputeCircular, offsets[1], color, p.linearSampler,
                buffers.half[0], prisma::StorageAccess::Write, pass1Groups[0], pass1Groups[1]);
        dispatchPass("DOF composite", kComputeComposite, offsets[2], buffers.half[0],
                p.linearSampler, color, prisma::StorageAccess::ReadWrite, fullGroupsX, fullGroupsY);
    }
    return ok ? dispatches : 0;
}

std::uint32_t DepthOfField::runRaster(DofBuffers& buffers, prisma::TextureHandle color,
        prisma::TextureHandle depth, const CameraAttributesSettings& attributes, float zNear,
        float zFar, bool orthogonal)
{
    Pipelines& p = *mPipelines;
    RenderProfiler* profiler = RenderProfiler::singleton();
    const std::uint32_t quality = static_cast<std::uint32_t>(mQuality);
    const bool boxOrHexagon = mShape != DofBokehShape::Circle;
    const bool lowQuality = quality <= 1;

    const float bokehSize = attributes.dofBlurAmount * 64.0f;

    DofParams pc;
    memset(&pc, 0, sizeof(pc));
    pc.orthogonal = orthogonal ? 1u : 0u;
    pc.size[0] = static_cast<std::int32_t>(buffers.width);
    pc.size[1] = static_cast<std::int32_t>(buffers.height);
    pc.zFar = zFar;
    pc.zNear = zNear;
    pc.secondPass = 0;
    pc.halfSize = 0;
    pc.blurSize = bokehSize;
    if (attributes.dofBlurFarEnabled)
    {
        pc.blurFarActive = 1;
        pc.blurFarBegin = attributes.dofBlurFarDistance;
        pc.blurFarEnd = attributes.dofBlurFarDistance + attributes.dofBlurFarTransition;
    }
    if (attributes.dofBlurNearEnabled)
    {
        pc.blurNearActive = 1;
        pc.blurNearBegin = attributes.dofBlurNearDistance;
        pc.blurNearEnd = attributes.dofBlurNearDistance - attributes.dofBlurNearTransition;
    }

    std::uint32_t blockCount = 0;
    auto addBlock = [&](const DofParams& params)
    {
        const std::uint32_t offset = blockCount * p.blockStride;
        memset(p.bytes.data() + offset, 0, p.blockStride);
        memcpy(p.bytes.data() + offset, &params, sizeof(params));
        ++blockCount;
        return offset;
    };

    const std::uint32_t fullWidth = buffers.width;
    const std::uint32_t fullHeight = buffers.height;
    const std::uint32_t halfWidth = halfExtent(fullWidth);
    const std::uint32_t halfHeight = halfExtent(fullHeight);

    std::uint32_t offsets[4] = { 0, 0, 0, 0 };
    offsets[0] = addBlock(pc);
    bool half = false;
    if (boxOrHexagon)
    {
        if (lowQuality)
        {
            pc.size[0] = static_cast<std::int32_t>(fullWidth >> 1);
            pc.size[1] = static_cast<std::int32_t>(fullHeight >> 1);
            pc.halfSize = 1;
            pc.blurSize *= 0.5f;
            half = true;
        }
        pc.blurScale = 0.5f;
        pc.steps = kQualitySamples[quality];
        offsets[1] = addBlock(pc);
        pc.secondPass = 1;
        offsets[2] = addBlock(pc);
        if (half)
        {
            pc.size[0] = static_cast<std::int32_t>(fullWidth);
            pc.size[1] = static_cast<std::int32_t>(fullHeight);
            offsets[3] = addBlock(pc);
        }
    }
    else
    {
        pc.size[0] = static_cast<std::int32_t>(fullWidth >> 1);
        pc.size[1] = static_cast<std::int32_t>(fullHeight >> 1);
        pc.halfSize = 1;
        half = true;
        pc.blurScale = kQualityScale[quality];
        pc.steps = 0;
        offsets[1] = addBlock(pc);
        pc.size[0] = static_cast<std::int32_t>(fullWidth);
        pc.size[1] = static_cast<std::int32_t>(fullHeight);
        offsets[2] = addBlock(pc);
    }
    mDriver->updateBuffer(p.params, 0, p.bytes.data(), blockCount * p.blockStride);

    std::uint32_t draws = 0;
    bool ok = true;

    struct Target
    {
        prisma::TextureHandle color;
        prisma::TextureHandle weight;
        std::uint32_t width;
        std::uint32_t height;
        bool load;
    };

    auto drawPass = [&](const char* name, std::uint32_t mode, std::uint32_t offset,
                            const Target& target, prisma::TextureHandle first,
                            prisma::SamplerHandle firstSampler, prisma::TextureHandle second,
                            prisma::TextureHandle third)
    {
        if (!ok) return;
        const bool scope = profiler != nullptr && profiler->beginGpu(name);
        const prisma::PipelineHandle pipeline = rasterPipeline(mode);
        if (pipeline.valid())
        {
            prisma::RenderPassDesc pass;
            if (target.color.valid()) pass.colors[pass.colorCount++].texture = target.color;
            if (target.weight.valid()) pass.colors[pass.colorCount++].texture = target.weight;
            pass.colorLoad = target.load ? prisma::LoadOp::Load : prisma::LoadOp::DontCare;
            mDriver->beginRenderPass(pass);
            prisma::Viewport area;
            area.width = static_cast<float>(target.width);
            area.height = static_cast<float>(target.height);
            mDriver->setViewport(area);
            mDriver->bindPipeline(pipeline);
            mDriver->bindUniformBuffer(0, p.params, offset, kBlockSize);
            mDriver->bindTexture(0, first, firstSampler);
            if (second.valid()) mDriver->bindTexture(1, second, p.linearSampler);
            if (third.valid()) mDriver->bindTexture(2, third, p.linearSampler);
            mDriver->draw(3, 0);
            mDriver->endRenderPass();
            ++draws;
        }
        else
            ok = false;
        if (scope) profiler->endGpu();
    };

    Target target;
    target.color = prisma::TextureHandle();
    target.weight = buffers.weight[0];
    target.width = fullWidth;
    target.height = fullHeight;
    target.load = false;
    drawPass("DOF blur size", kRasterBlurSize, offsets[0], target, depth, p.nearestSampler,
            prisma::TextureHandle(), prisma::TextureHandle());

    if (boxOrHexagon)
    {
        const bool box = mShape == DofBokehShape::Box;
        const std::uint32_t weighted = box ? kRasterBox : kRasterHexagon;
        const std::uint32_t unweighted = box ? kRasterBoxNoWeight : kRasterHexagonNoWeight;
        if (half)
        {
            target.color = buffers.half[0];
            target.weight = buffers.weight[2];
            target.width = halfWidth;
            target.height = halfHeight;
            drawPass("DOF bokeh 1", weighted, offsets[1], target, color, p.linearSampler,
                    buffers.weight[0], prisma::TextureHandle());
            target.color = buffers.half[1];
            target.weight = buffers.weight[3];
            drawPass("DOF bokeh 2", weighted, offsets[2], target, buffers.half[0], p.linearSampler,
                    buffers.weight[2], prisma::TextureHandle());
            target.color = color;
            target.weight = prisma::TextureHandle();
            target.width = fullWidth;
            target.height = fullHeight;
            target.load = true;
            drawPass("DOF composite", kRasterComposite, offsets[3], target, buffers.half[1],
                    p.linearSampler, buffers.weight[3], buffers.weight[0]);
        }
        else
        {
            target.color = buffers.secondary;
            target.weight = buffers.weight[1];
            drawPass("DOF bokeh 1", weighted, offsets[1], target, color, p.linearSampler,
                    buffers.weight[0], prisma::TextureHandle());
            target.color = color;
            target.weight = prisma::TextureHandle();
            drawPass("DOF bokeh 2", unweighted, offsets[2], target, buffers.secondary,
                    p.linearSampler, buffers.weight[1], prisma::TextureHandle());
        }
    }
    else
    {
        target.color = buffers.half[0];
        target.weight = buffers.weight[2];
        target.width = halfWidth;
        target.height = halfHeight;
        drawPass("DOF bokeh 1", kRasterCircular, offsets[1], target, color, p.linearSampler,
                buffers.weight[0], prisma::TextureHandle());
        target.color = color;
        target.weight = prisma::TextureHandle();
        target.width = fullWidth;
        target.height = fullHeight;
        target.load = true;
        drawPass("DOF composite", kRasterComposite, offsets[2], target, buffers.half[0],
                p.linearSampler, buffers.weight[2], buffers.weight[0]);
    }
    return ok ? draws : 0;
}

} // namespace prisma3d
