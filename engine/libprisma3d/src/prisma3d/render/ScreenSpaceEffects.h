#pragma once

#include "prisma3d/render/ShaderCompiler.h"

#include "prisma/rhi/Driver.h"
#include "prisma/rhi/ShaderBlob.h"
#include "prisma3d/render/RenderingServer.h"

#include "mathc.h"

#include <cstdint>

namespace prisma3d
{

struct SsaoSettings
{
    float radius = 1.0f;
    float intensity = 2.0f;
    float power = 1.5f;
    float detail = 0.5f;
    float horizon = 0.06f;
    float sharpness = 0.98f;
};

struct SsaoBuffers
{
    prisma::TextureHandle linearDepth;
    prisma::TextureHandle deinterleaved;
    prisma::TextureHandle pong;
    prisma::TextureHandle importance;
    prisma::TextureHandle importancePong;
    prisma::TextureHandle finalAo;
    std::uint32_t fullWidth = 0;
    std::uint32_t fullHeight = 0;
    std::uint32_t depthWidth = 0;
    std::uint32_t depthHeight = 0;
    std::uint32_t bufferWidth = 0;
    std::uint32_t bufferHeight = 0;
    std::uint32_t halfBufferWidth = 0;
    std::uint32_t halfBufferHeight = 0;
    std::uint32_t depthMips = 0;
    bool halfSize = false;
    bool mips = false;
    bool ultra = false;

    bool valid() const
    {
        return finalAo.valid();
    }
};

struct SsrSettings
{
    std::int32_t maxSteps = 64;
    float fadeIn = 0.15f;
    float fadeOut = 2.0f;
    float depthTolerance = 0.5f;
};

struct SsrBuffers
{
    prisma::TextureHandle lastFrame;
    prisma::TextureHandle hiz;
    prisma::TextureHandle color;
    prisma::TextureHandle mipLevel;
    prisma::TextureHandle normalRoughness;
    prisma::TextureHandle resolved;
    std::uint32_t fullWidth = 0;
    std::uint32_t fullHeight = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t mipmaps = 0;
    bool halfSize = false;
    bool haveLast = false;
    Math::Mat4 lastProjection = Math::Mat4::Identity();
    Math::Mat4 lastView = Math::Mat4::Identity();

    bool valid() const
    {
        return color.valid();
    }

    prisma::TextureHandle result() const
    {
        return halfSize ? resolved : color;
    }
};

class ScreenSpaceEffects
{
public:
    ScreenSpaceEffects();
    ~ScreenSpaceEffects();
    ScreenSpaceEffects(const ScreenSpaceEffects&) = delete;
    ScreenSpaceEffects& operator=(const ScreenSpaceEffects&) = delete;

    bool create(prisma::Driver* driver);
    void destroy();
    bool supported() const;
    bool failed() const;

    void setSsaoQuality(SsaoQuality quality, bool halfSize, float adaptiveTarget,
            std::uint32_t blurPasses, float fadeoutFrom, float fadeoutTo);
    SsaoQuality ssaoQuality() const;
    bool ssaoHalfSize() const;
    float ssaoAdaptiveTarget() const;
    std::uint32_t ssaoBlurPasses() const;
    float ssaoFadeoutFrom() const;
    float ssaoFadeoutTo() const;

    bool ensureBestFitNormal();
    prisma::TextureHandle bestFitNormal() const;

    bool allocateSsao(SsaoBuffers& buffers, std::uint32_t width, std::uint32_t height,
            bool& created);
    static void releaseSsao(prisma::Driver* driver, SsaoBuffers& buffers);
    std::uint32_t generateSsao(SsaoBuffers& buffers, prisma::TextureHandle depth,
            prisma::TextureHandle normalRoughness, const Math::Mat4& projection, bool orthogonal,
            float zNear, float zFar, const SsaoSettings& settings);

    void setSsrHalfSize(bool halfSize);
    bool ssrHalfSize() const;
    bool allocateSsr(SsrBuffers& buffers, std::uint32_t width, std::uint32_t height, bool& created);
    static void releaseSsr(prisma::Driver* driver, SsrBuffers& buffers);
    std::uint32_t generateSsr(SsrBuffers& buffers, prisma::TextureHandle depth,
            prisma::TextureHandle normalRoughness, const Math::Mat4& projection,
            const Math::Mat4& view, bool orthogonal, const SsrSettings& settings);
    std::uint32_t copyLastFrame(SsrBuffers& buffers, prisma::TextureHandle color);
    prisma::SamplerHandle ssrColorSampler() const;
    prisma::SamplerHandle ssrMipSampler() const;

    static prisma::TextureFormat normalRoughnessFormat();
    static std::uint32_t normalRoughnessUsage();

private:
    struct Pipelines;

    bool ensureSsao();
    bool ensureSsr();
    prisma::PipelineHandle ssrCopyPipeline(std::uint32_t mode);
    prisma::PipelineHandle ssrDownsamplePipeline(std::uint32_t mode);
    prisma::PipelineHandle ssrHizPipeline(std::uint32_t mode);
    prisma::PipelineHandle ssrTracePipeline();
    prisma::PipelineHandle ssrFilterPipeline();
    prisma::PipelineHandle ssrResolvePipeline();
    prisma::PipelineHandle downsamplePipeline(std::uint32_t mode);
    prisma::PipelineHandle gatherPipeline(std::uint32_t mode);
    prisma::PipelineHandle importancePipeline(std::uint32_t mode);
    prisma::PipelineHandle blurPipeline(std::uint32_t mode);
    prisma::PipelineHandle interleavePipeline(std::uint32_t mode);
    prisma::PipelineHandle createPipeline(const ShaderVariant& variant,
            const prisma::ComputePipelineDesc& desc, prisma::ShaderHandle& shader);
    prisma::Driver* mDriver;
    bool mFailed;
    bool mBestFitFailed;
    SsaoQuality mQuality;
    bool mHalfSize;
    bool mSsrHalfSize;
    float mAdaptiveTarget;
    std::uint32_t mBlurPasses;
    float mFadeoutFrom;
    float mFadeoutTo;
    Pipelines* mPipelines;
};

} // namespace prisma3d
