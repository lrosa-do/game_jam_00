#pragma once

#include "prisma/rhi/Driver.h"
#include "prisma3d/render/RenderingServer.h"

#include <cstdint>

namespace prisma3d
{

struct DofBuffers
{
    prisma::TextureHandle secondary;
    prisma::TextureHandle half[2];
    prisma::TextureHandle weight[4];
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    bool raster = false;

    bool valid() const
    {
        return secondary.valid();
    }

    std::uint32_t textureCount() const
    {
        std::uint32_t count = (secondary.valid() ? 1u : 0u) + (half[0].valid() ? 1u : 0u) +
                              (half[1].valid() ? 1u : 0u);
        for (const prisma::TextureHandle& texture: weight) count += texture.valid() ? 1u : 0u;
        return count;
    }
};

class DepthOfField
{
public:
    DepthOfField();
    ~DepthOfField();
    DepthOfField(const DepthOfField&) = delete;
    DepthOfField& operator=(const DepthOfField&) = delete;

    bool create(prisma::Driver* driver);
    void destroy();
    bool failed() const;

    void setBokeh(DofBokehShape shape, DofBlurQuality quality, bool useJitter);
    DofBokehShape shape() const;
    DofBlurQuality quality() const;
    bool useJitter() const;
    void setComputeAllowed(bool allowed);
    bool computeAllowed() const;
    bool usesCompute() const;

    static std::uint32_t colorUsage(bool compute);
    bool allocate(DofBuffers& buffers, std::uint32_t width, std::uint32_t height, bool& created);
    static void release(prisma::Driver* driver, DofBuffers& buffers);

    std::uint32_t run(DofBuffers& buffers, prisma::TextureHandle color, prisma::TextureHandle depth,
            const CameraAttributesSettings& attributes, float zNear, float zFar, bool orthogonal);

private:
    struct Pipelines;

    bool ensure();
    prisma::PipelineHandle computePipeline(std::uint32_t mode);
    prisma::PipelineHandle rasterPipeline(std::uint32_t mode);
    std::uint32_t runCompute(DofBuffers& buffers, prisma::TextureHandle color,
            prisma::TextureHandle depth, const CameraAttributesSettings& attributes, float zNear,
            float zFar, bool orthogonal);
    std::uint32_t runRaster(DofBuffers& buffers, prisma::TextureHandle color,
            prisma::TextureHandle depth, const CameraAttributesSettings& attributes, float zNear,
            float zFar, bool orthogonal);

    prisma::Driver* mDriver;
    bool mFailed;
    bool mComputeAllowed;
    DofBokehShape mShape;
    DofBlurQuality mQuality;
    bool mJitter;
    std::uint32_t mFrame;
    Pipelines* mPipelines;
};

} // namespace prisma3d
