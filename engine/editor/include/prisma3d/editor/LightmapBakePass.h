#pragma once

#include "prisma/rhi/Driver.h"
#include "prisma3d/render/MeshStore.h"

#include "mathc.h"

#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct LightmapBakeSettings
{
    float bias = 0.0f;
    float biasTexels = 3.0f;
    Math::Vec3 ambient = Math::Vec3(0.12f, 0.16f, 0.24f);
    float ambientGround = 0.35f;
    float filterRadius = 1.0f;
    float sunAngularRadius = 0.0f;
    std::uint32_t sampleCount = 1;
    std::uint32_t shadowResolution = 0;
};

class LightmapBakePass
{
public:
    struct Caster
    {
        MeshId mesh;
        Math::Mat4 transform;
        Math::Box bounds;
    };

    LightmapBakePass();
    ~LightmapBakePass();
    LightmapBakePass(const LightmapBakePass&) = delete;
    LightmapBakePass& operator=(const LightmapBakePass&) = delete;

    bool begin(MeshId receiver, const Math::Mat4& model, const ct::Vector<Caster>& casters,
            const Math::Vec3& lightDirection, const Math::Vec3& lightColor,
            std::uint32_t resolution, const LightmapBakeSettings& settings);
    bool step();
    void cancel();
    void shutdown();

    bool active() const;
    bool done() const;
    bool failed() const;
    std::uint32_t samplesDone() const;
    std::uint32_t samplesTotal() const;
    std::uint32_t resolution() const;
    const ct::Vector<std::uint16_t>& pixels() const;

    static void normalizeCoverage(ct::Vector<std::uint16_t>& pixels, std::uint32_t samples);
    static void dilate(ct::Vector<std::uint16_t>& pixels, std::uint32_t resolution,
            std::uint32_t iterations);

private:
    struct BakeBlock;
    struct Pipelines
    {
        prisma::PipelineHandle shadow;
        prisma::PipelineHandle bake;
    };

    enum class Stage : std::uint8_t
    {
        Idle,
        Sampling,
        Packing,
        Done,
        Failed
    };

    bool ensureShaders();
    bool ensureBlocks(std::size_t count);
    bool ensureTargets(std::uint32_t resolution, std::uint32_t shadowResolution);
    bool pipelinesFor(std::uint32_t attributeMask, Pipelines& out);
    void drawMesh(MeshId mesh, std::uint32_t block, prisma::PipelineHandle Pipelines::*which,
            bool bakePass);
    bool renderSample();
    bool packAndRead();
    void releaseTargets();
    void fail();

    prisma::Driver* mDriver;
    MeshId mMesh;
    Stage mStage;
    Math::Mat4 mModel;
    Math::Vec3 mDirection;
    Math::Vec3 mColor;
    LightmapBakeSettings mSettings;
    Math::Box mWorldBounds;
    ct::Vector<Caster> mCasters;
    std::uint32_t mBlockStride;
    std::size_t mBlockCapacity;
    std::uint32_t mResolution;
    std::uint32_t mShadowResolution;
    std::uint32_t mSample;
    std::uint32_t mSamples;

    prisma::ShaderHandle mShadowVertex;
    prisma::ShaderHandle mShadowFragment;
    prisma::ShaderHandle mBakeVertex;
    prisma::ShaderHandle mBakeFragment;
    prisma::ShaderHandle mPackVertex;
    prisma::ShaderHandle mPackFragment;
    prisma::PipelineHandle mPackPipeline;
    ct::HashMap<std::uint32_t, Pipelines> mPipelines;
    prisma::BufferHandle mBlock;
    prisma::BufferHandle mPackBlock;
    prisma::TextureHandle mShadowMap;
    prisma::TextureHandle mLightmap;
    prisma::TextureHandle mPacked;
    prisma::SamplerHandle mShadowSampler;
    prisma::SamplerHandle mNearestSampler;
    ct::Vector<std::uint16_t> mPixels;
};

} // namespace prisma3d
