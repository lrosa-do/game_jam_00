#include "prisma3d/editor/LightmapBakePass.h"

#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/LightmapGI.h"

#include "lightmap_bake.frag.h"
#include "lightmap_bake.vert.h"
#include "lightmap_pack.frag.h"
#include "lightmap_pack.vert.h"
#include "lightmap_shadow.frag.h"
#include "lightmap_shadow.vert.h"

#include "platform.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

struct LightmapBakePass::BakeBlock
{
    Math::Mat4 shadowViewProjection;
    Math::Mat4 model;
    Math::Vec4 lightDirection;
    Math::Vec4 lightColor;
    Math::Vec4 params;
    Math::Vec4 ambientSky;
    Math::Vec4 jitter;
};

namespace
{

const std::uint16_t kHalfOne = 0x3C00;

float halton(std::uint32_t index, std::uint32_t base)
{
    float result = 0.0f;
    float fraction = 1.0f / static_cast<float>(base);
    for (std::uint32_t i = index + 1; i > 0; i /= base)
    {
        result += static_cast<float>(i % base) * fraction;
        fraction /= static_cast<float>(base);
    }
    return result;
}

void projectBounds(const Math::Box& bounds, const Math::Mat4& view, Math::Vec3& minimum,
        Math::Vec3& maximum)
{
    minimum = Math::Vec3(1.0e30f);
    maximum = Math::Vec3(-1.0e30f);
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const Math::Vec3 point((corner & 1) ? bounds.max.x : bounds.min.x,
                (corner & 2) ? bounds.max.y : bounds.min.y,
                (corner & 4) ? bounds.max.z : bounds.min.z);
        const Math::Vec3 viewPoint = view.TransformPoint(point);
        minimum = Math::Vec3::Min(minimum, viewPoint);
        maximum = Math::Vec3::Max(maximum, viewPoint);
    }
}

Math::Vec2 vogelDisk(std::uint32_t index, std::uint32_t count)
{
    const float goldenAngle = 2.39996323f;
    const float radius = sqrtf((static_cast<float>(index) + 0.5f) / static_cast<float>(count));
    const float theta = static_cast<float>(index) * goldenAngle;
    return Math::Vec2(radius * cosf(theta), radius * sinf(theta));
}

Math::Vec3 jitterSunDirection(const Math::Vec3& direction, float angularRadius, std::uint32_t index,
        std::uint32_t count)
{
    if (angularRadius <= 0.0f || count <= 1) return direction;
    const Math::Vec3 reference = fabsf(direction.y) > 0.99f ? Math::Vec3(1.0f, 0.0f, 0.0f)
                                                            : Math::Vec3(0.0f, 1.0f, 0.0f);
    const Math::Vec3 right = reference.Cross(direction).Normalized();
    const Math::Vec3 up = direction.Cross(right);
    const float spread = tanf(angularRadius * 3.14159265358979f / 180.0f);
    const Math::Vec2 offset = vogelDisk(index, count) * spread;
    return (direction + right * offset.x + up * offset.y).Normalized();
}

std::uint32_t clampu(std::uint32_t value, std::uint32_t low, std::uint32_t high)
{
    return value < low ? low : (value > high ? high : value);
}

} // namespace

LightmapBakePass::LightmapBakePass()
        : mDriver(nullptr),
          mStage(Stage::Idle),
          mDirection(0.0f, -1.0f, 0.0f),
          mColor(1.0f, 1.0f, 1.0f),
          mBlockStride(0),
          mBlockCapacity(0),
          mResolution(0),
          mShadowResolution(0),
          mSample(0),
          mSamples(1)
{
}

LightmapBakePass::~LightmapBakePass()
{
    shutdown();
}

bool LightmapBakePass::ensureShaders()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return false;
    prisma::Driver* driver = server->driver();
    if (!driver) return false;
    if (mDriver == driver && mPackPipeline.valid() && mBlock.valid()) return true;
    shutdown();
    mDriver = driver;

    const prisma::Caps& caps = driver->caps();
    mShadowVertex = driver->createShader(prisma::shaderDesc(lightmap_shadow_vert, caps));
    mShadowFragment = driver->createShader(prisma::shaderDesc(lightmap_shadow_frag, caps));
    mBakeVertex = driver->createShader(prisma::shaderDesc(lightmap_bake_vert, caps));
    mBakeFragment = driver->createShader(prisma::shaderDesc(lightmap_bake_frag, caps));
    mPackVertex = driver->createShader(prisma::shaderDesc(lightmap_pack_vert, caps));
    mPackFragment = driver->createShader(prisma::shaderDesc(lightmap_pack_frag, caps));

    const std::uint32_t alignment =
            caps.uniformBufferOffsetAlignment > 0 ? caps.uniformBufferOffsetAlignment : 1u;
    mBlockStride =
            (static_cast<std::uint32_t>(sizeof(BakeBlock)) + alignment - 1) / alignment * alignment;
    mBlockCapacity = 0;
    prisma::BufferDesc block;
    block.usage = prisma::BufferUsage::Uniform;
    block.update = prisma::BufferUpdate::Dynamic;
    block.size = 16;
    block.debugName = "lightmap pack block";
    mPackBlock = driver->createBuffer(block);
    ensureBlocks(1);

    prisma::SamplerDesc sampler;
    sampler.minFilter = prisma::Filter::Linear;
    sampler.magFilter = prisma::Filter::Linear;
    sampler.mipFilter = prisma::MipFilter::None;
    sampler.addressU = prisma::AddressMode::ClampToEdge;
    sampler.addressV = prisma::AddressMode::ClampToEdge;
    sampler.debugName = "lightmap shadow";
    mShadowSampler = driver->createSampler(sampler);
    sampler.minFilter = prisma::Filter::Nearest;
    sampler.magFilter = prisma::Filter::Nearest;
    sampler.debugName = "lightmap pack";
    mNearestSampler = driver->createSampler(sampler);

    prisma::PipelineDesc pack;
    pack.vertexShader = mPackVertex;
    pack.fragmentShader = mPackFragment;
    pack.uniformBlockCount = 1;
    pack.uniformBlocks[0].name = "PackBlock";
    pack.uniformBlocks[0].slot = 0;
    pack.textureCount = 1;
    pack.textures[0].name = "uLightmap";
    pack.textures[0].slot = 0;
    pack.targets.window = false;
    pack.targets.colorCount = 1;
    pack.targets.colors[0] = prisma::TextureFormat::RGBA8;
    pack.targets.depth = prisma::TextureFormat::None;
    pack.depthTest = false;
    pack.depthWrite = false;
    pack.cullMode = prisma::CullMode::None;
    pack.debugName = "lightmap pack";
    if (mPackVertex.valid() && mPackFragment.valid()) mPackPipeline = driver->createPipeline(pack);

    const bool ready = mShadowVertex.valid() && mShadowFragment.valid() && mBakeVertex.valid() &&
                       mBakeFragment.valid() && mBlock.valid() && mPackBlock.valid() &&
                       mShadowSampler.valid() && mNearestSampler.valid() && mPackPipeline.valid();
    if (!ready)
    {
        log_error("lightmap bake: could not create the bake resources");
        shutdown();
    }
    return ready;
}

bool LightmapBakePass::ensureBlocks(std::size_t count)
{
    if (mBlock.valid() && mBlockCapacity >= count) return true;
    if (mBlock.valid()) mDriver->destroy(mBlock);
    std::size_t capacity = mBlockCapacity > 0 ? mBlockCapacity : 1;
    while (capacity < count) capacity *= 2;
    prisma::BufferDesc block;
    block.usage = prisma::BufferUsage::Uniform;
    block.update = prisma::BufferUpdate::Dynamic;
    block.size = static_cast<std::uint32_t>(mBlockStride * capacity);
    block.debugName = "lightmap bake block";
    mBlock = mDriver->createBuffer(block);
    mBlockCapacity = mBlock.valid() ? capacity : 0;
    return mBlock.valid();
}

bool LightmapBakePass::pipelinesFor(std::uint32_t attributeMask, Pipelines& out)
{
    if (const Pipelines* found = mPipelines.find(attributeMask))
    {
        out = *found;
        return out.shadow.valid() && out.bake.valid();
    }
    const std::uint32_t stride = MeshStore::vertexStride(attributeMask);
    const std::uint32_t position = MeshStore::attributeOffset(attributeMask, kVertexPosition);

    prisma::PipelineDesc shadow;
    shadow.vertexShader = mShadowVertex;
    shadow.fragmentShader = mShadowFragment;
    shadow.vertexBuffers[0].stride = stride;
    shadow.vertexBufferCount = 1;
    shadow.attributes[0].location = 0;
    shadow.attributes[0].format = prisma::VertexFormat::Float3;
    shadow.attributes[0].offset = position;
    shadow.attributeCount = 1;
    shadow.uniformBlockCount = 1;
    shadow.uniformBlocks[0].name = "BakeBlock";
    shadow.uniformBlocks[0].slot = 0;
    shadow.targets.window = false;
    shadow.targets.colorCount = 0;
    shadow.targets.depth = prisma::TextureFormat::Depth32F;
    shadow.depthTest = true;
    shadow.depthWrite = true;
    shadow.depthCompare = prisma::CompareOp::LessEqual;
    shadow.cullMode = prisma::CullMode::None;
    shadow.debugName = "lightmap shadow";

    prisma::PipelineDesc bake;
    bake.vertexShader = mBakeVertex;
    bake.fragmentShader = mBakeFragment;
    bake.vertexBuffers[0].stride = stride;
    bake.vertexBufferCount = 1;
    bake.attributes[0].location = 0;
    bake.attributes[0].format = prisma::VertexFormat::Float3;
    bake.attributes[0].offset = position;
    bake.attributes[1].location = 1;
    bake.attributes[1].format = prisma::VertexFormat::Float3;
    bake.attributes[1].offset = MeshStore::attributeOffset(attributeMask, kVertexNormal);
    bake.attributes[2].location = 2;
    bake.attributes[2].format = prisma::VertexFormat::Float2;
    bake.attributes[2].offset = MeshStore::attributeOffset(attributeMask, kVertexUv2);
    bake.attributeCount = 3;
    bake.uniformBlockCount = 1;
    bake.uniformBlocks[0].name = "BakeBlock";
    bake.uniformBlocks[0].slot = 0;
    bake.textureCount = 1;
    bake.textures[0].name = "uShadowMap";
    bake.textures[0].slot = 0;
    bake.targets.window = false;
    bake.targets.colorCount = 1;
    bake.targets.colors[0] = prisma::TextureFormat::RGBA16F;
    bake.targets.depth = prisma::TextureFormat::None;
    bake.depthTest = false;
    bake.depthWrite = false;
    bake.cullMode = prisma::CullMode::None;
    bake.blend = true;
    bake.srcColor = prisma::BlendFactor::One;
    bake.dstColor = prisma::BlendFactor::One;
    bake.srcAlpha = prisma::BlendFactor::One;
    bake.dstAlpha = prisma::BlendFactor::One;
    bake.debugName = "lightmap bake";

    out.shadow = mDriver->createPipeline(shadow);
    out.bake = mDriver->createPipeline(bake);
    mPipelines[attributeMask] = out;
    return out.shadow.valid() && out.bake.valid();
}

bool LightmapBakePass::ensureTargets(std::uint32_t resolution, std::uint32_t shadowResolution)
{
    if (mShadowResolution != shadowResolution || !mShadowMap.valid())
    {
        if (mShadowMap.valid()) mDriver->destroy(mShadowMap);
        prisma::TextureDesc shadow;
        shadow.format = prisma::TextureFormat::Depth32F;
        shadow.width = shadowResolution;
        shadow.height = shadowResolution;
        shadow.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        shadow.debugName = "lightmap shadow depth";
        mShadowMap = mDriver->createTexture(shadow);
        mShadowResolution = shadowResolution;
    }
    if (mResolution != resolution || !mLightmap.valid() || !mPacked.valid())
    {
        if (mLightmap.valid()) mDriver->destroy(mLightmap);
        if (mPacked.valid()) mDriver->destroy(mPacked);
        prisma::TextureDesc color;
        color.format = prisma::TextureFormat::RGBA16F;
        color.width = resolution;
        color.height = resolution;
        color.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        color.debugName = "lightmap color";
        mLightmap = mDriver->createTexture(color);
        prisma::TextureDesc packed;
        packed.format = prisma::TextureFormat::RGBA8;
        packed.width = resolution * 2;
        packed.height = resolution;
        packed.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
        packed.debugName = "lightmap packed";
        mPacked = mDriver->createTexture(packed);
        mResolution = resolution;
    }
    return mShadowMap.valid() && mLightmap.valid() && mPacked.valid();
}

void LightmapBakePass::releaseTargets()
{
    if (!mDriver) return;
    if (mShadowMap.valid()) mDriver->destroy(mShadowMap);
    if (mLightmap.valid()) mDriver->destroy(mLightmap);
    if (mPacked.valid()) mDriver->destroy(mPacked);
    mShadowMap = prisma::TextureHandle();
    mLightmap = prisma::TextureHandle();
    mPacked = prisma::TextureHandle();
    mResolution = 0;
    mShadowResolution = 0;
}

void LightmapBakePass::shutdown()
{
    if (mDriver)
    {
        releaseTargets();
        for (auto& entry: mPipelines)
        {
            if (entry.value.shadow.valid()) mDriver->destroy(entry.value.shadow);
            if (entry.value.bake.valid()) mDriver->destroy(entry.value.bake);
        }
        mPipelines.clear();
        if (mPackPipeline.valid()) mDriver->destroy(mPackPipeline);
        if (mBlock.valid()) mDriver->destroy(mBlock);
        if (mPackBlock.valid()) mDriver->destroy(mPackBlock);
        if (mShadowSampler.valid()) mDriver->destroy(mShadowSampler);
        if (mNearestSampler.valid()) mDriver->destroy(mNearestSampler);
        if (mShadowVertex.valid()) mDriver->destroy(mShadowVertex);
        if (mShadowFragment.valid()) mDriver->destroy(mShadowFragment);
        if (mBakeVertex.valid()) mDriver->destroy(mBakeVertex);
        if (mBakeFragment.valid()) mDriver->destroy(mBakeFragment);
        if (mPackVertex.valid()) mDriver->destroy(mPackVertex);
        if (mPackFragment.valid()) mDriver->destroy(mPackFragment);
    }
    mPackPipeline = prisma::PipelineHandle();
    mBlock = prisma::BufferHandle();
    mPackBlock = prisma::BufferHandle();
    mShadowSampler = prisma::SamplerHandle();
    mNearestSampler = prisma::SamplerHandle();
    mShadowVertex = prisma::ShaderHandle();
    mShadowFragment = prisma::ShaderHandle();
    mBakeVertex = prisma::ShaderHandle();
    mBakeFragment = prisma::ShaderHandle();
    mPackVertex = prisma::ShaderHandle();
    mPackFragment = prisma::ShaderHandle();
    mPixels.clear();
    mCasters.clear();
    mBlockCapacity = 0;
    mStage = Stage::Idle;
    mDriver = nullptr;
}

void LightmapBakePass::fail()
{
    mStage = Stage::Failed;
    mPixels.clear();
}

bool LightmapBakePass::begin(MeshId receiver, const Math::Mat4& model,
        const ct::Vector<Caster>& casters, const Math::Vec3& lightDirection,
        const Math::Vec3& lightColor, std::uint32_t resolution,
        const LightmapBakeSettings& settings)
{
    mStage = Stage::Idle;
    mPixels.clear();
    MeshStore* meshes = MeshStore::singleton();
    const MeshInfo* info = meshes ? meshes->get(receiver) : nullptr;
    if (!info || info->surfaces.empty())
    {
        log_error("lightmap bake: invalid mesh");
        return false;
    }
    for (std::size_t i = 0; i < info->surfaces.size(); ++i)
    {
        const SurfaceInfo& surface = info->surfaces[i];
        if (surface.skinned() || surface.topology != prisma::Topology::Triangles ||
                (surface.attributeMask & kVertexUv2) == 0)
        {
            log_error(
                    "lightmap bake: the mesh must be a static triangle mesh with a second UV set");
            return false;
        }
    }
    if (!ensureShaders()) return false;

    resolution = clampu(resolution, 64u, 4096u);
    const std::uint32_t shadowResolution =
            clampu(settings.shadowResolution ? settings.shadowResolution : resolution, 64u, 16384u);
    if (!ensureTargets(resolution, shadowResolution))
    {
        log_error("lightmap bake: could not create the bake targets");
        return false;
    }

    mMesh = receiver;
    mModel = model;
    mDirection = lightDirection.Normalized();
    mColor = lightColor;
    mSettings = settings;
    mWorldBounds = info->aabb.Transformed(model);
    mCasters.clear();
    for (std::size_t i = 0; i < casters.size(); ++i)
    {
        const MeshInfo* casterInfo = meshes->get(casters[i].mesh);
        if (!casterInfo || casterInfo->surfaces.empty()) continue;
        Caster caster;
        caster.mesh = casters[i].mesh;
        caster.transform = casters[i].transform;
        caster.bounds = casterInfo->aabb.Transformed(casters[i].transform);
        mCasters.push_back(caster);
    }
    if (!ensureBlocks(mCasters.size() + 1)) return false;
    mSample = 0;
    mSamples = clampu(settings.sampleCount, 1u, 64u);
    mStage = Stage::Sampling;
    return true;
}

void LightmapBakePass::drawMesh(MeshId mesh, std::uint32_t block,
        prisma::PipelineHandle Pipelines::*which, bool bakePass)
{
    const MeshInfo* info = MeshStore::singleton()->get(mesh);
    if (!info) return;
    for (std::size_t i = 0; i < info->surfaces.size(); ++i)
    {
        const SurfaceInfo& surface = info->surfaces[i];
        Pipelines pipelines;
        if (!pipelinesFor(surface.attributeMask, pipelines)) continue;
        mDriver->bindPipeline(pipelines.*which);
        mDriver->bindUniformBuffer(0, mBlock, block * mBlockStride,
                static_cast<std::uint32_t>(sizeof(BakeBlock)));
        if (bakePass) mDriver->bindTexture(0, mShadowMap, mShadowSampler);
        mDriver->bindVertexBuffer(0, surface.vertexBuffer, 0);
        if (surface.indexCount > 0)
        {
            mDriver->bindIndexBuffer(surface.indexBuffer);
            mDriver->drawIndexed(surface.indexCount, 0);
        }
        else
            mDriver->draw(surface.vertexCount, 0);
    }
}

bool LightmapBakePass::renderSample()
{
    const float weight = 1.0f / static_cast<float>(mSamples);
    const Math::Vec3 sampleDirection =
            jitterSunDirection(mDirection, mSettings.sunAngularRadius, mSample, mSamples);
    Math::Vec3 up(0.0f, 1.0f, 0.0f);
    if (fabsf(up.Dot(sampleDirection)) > 0.95f) up = Math::Vec3(1.0f, 0.0f, 0.0f);

    Math::Box scene = mWorldBounds;
    for (std::size_t i = 0; i < mCasters.size(); ++i) scene = scene.Union(mCasters[i].bounds);
    const Math::Vec3 sceneCenter = scene.Center();
    const float sceneRadius = fmaxf(scene.Extents().Length(), 1.0f);
    const float radius = fmaxf(mWorldBounds.Extents().Length(), 1.0f);
    const Math::Mat4 shadowView =
            Math::Mat4::LookAt(sceneCenter - sampleDirection * sceneRadius * 2.5f, sceneCenter, up);

    Math::Vec3 viewMinimum;
    Math::Vec3 viewMaximum;
    projectBounds(mWorldBounds, shadowView, viewMinimum, viewMaximum);
    const float margin = fmaxf(radius * 0.01f, 0.01f);
    float depthMinimum = viewMinimum.z;
    float depthMaximum = viewMaximum.z;
    ct::Vector<std::uint32_t> kept;
    for (std::size_t i = 0; i < mCasters.size(); ++i)
    {
        Math::Vec3 low;
        Math::Vec3 high;
        projectBounds(mCasters[i].bounds, shadowView, low, high);
        if (high.x < viewMinimum.x - margin || low.x > viewMaximum.x + margin ||
                high.y < viewMinimum.y - margin || low.y > viewMaximum.y + margin)
            continue;
        if (low.z < depthMinimum) depthMinimum = low.z;
        if (high.z > depthMaximum) depthMaximum = high.z;
        kept.push_back(static_cast<std::uint32_t>(i));
    }
    const float nearPlane = fmaxf(-depthMaximum - margin, 0.01f);
    const float farPlane = -depthMinimum + margin;
    const Math::Mat4 depthZeroToOne(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f),
            Math::Vec4(0.0f, 1.0f, 0.0f, 0.0f), Math::Vec4(0.0f, 0.0f, 0.5f, 0.0f),
            Math::Vec4(0.0f, 0.0f, 0.5f, 1.0f));
    const Math::Mat4 shadowProjection =
            depthZeroToOne * Math::Mat4::Ortho(viewMinimum.x - margin, viewMaximum.x + margin,
                                     viewMinimum.y - margin, viewMaximum.y + margin, nearPlane,
                                     farPlane);

    BakeBlock block;
    block.shadowViewProjection = shadowProjection * shadowView;
    block.model = mModel;
    block.lightDirection = Math::Vec4(sampleDirection, 0.0f);
    block.lightColor = Math::Vec4(mColor, 1.0f);
    const float depthRange = fmaxf(farPlane - nearPlane, 0.001f);
    const float texelWorldSize =
            (viewMaximum.x - viewMinimum.x + 2.0f * margin) / static_cast<float>(mShadowResolution);
    const float worldBias =
            mSettings.bias > 0.0f ? mSettings.bias : mSettings.biasTexels * texelWorldSize;
    block.params = Math::Vec4(worldBias / depthRange, mSettings.ambientGround,
            mSettings.filterRadius, weight);
    block.ambientSky = Math::Vec4(mSettings.ambient, 0.0f);
    const float texelWidth = 2.0f / static_cast<float>(mResolution);
    block.jitter = Math::Vec4((halton(mSample, 2) * 2.0f - 1.0f) * texelWidth * 0.7f,
            (halton(mSample, 3) * 2.0f - 1.0f) * texelWidth * 0.7f, 0.0f, 0.0f);

    ct::Vector<std::uint8_t> bytes;
    bytes.resize(mBlockStride * (kept.size() + 1));
    memset(bytes.data(), 0, bytes.size());
    memcpy(bytes.data(), &block, sizeof(block));
    for (std::size_t i = 0; i < kept.size(); ++i)
    {
        BakeBlock caster = block;
        caster.model = mCasters[kept[i]].transform;
        memcpy(bytes.data() + mBlockStride * (i + 1), &caster, sizeof(caster));
    }
    mDriver->updateBuffer(mBlock, 0, bytes.data(), static_cast<std::uint32_t>(bytes.size()));

    RenderProfiler* profiler = RenderProfiler::singleton();
    bool scoped = profiler != nullptr && profiler->beginGpu("Lightmap shadow");
    prisma::RenderPassDesc shadowPass;
    shadowPass.colorCount = 0;
    shadowPass.depth.texture = mShadowMap;
    shadowPass.depthLoad = prisma::LoadOp::Clear;
    shadowPass.depthStore = prisma::StoreOp::Store;
    shadowPass.clearDepth = 1.0f;
    mDriver->beginRenderPass(shadowPass);
    prisma::Viewport shadowArea;
    shadowArea.width = static_cast<float>(mShadowResolution);
    shadowArea.height = static_cast<float>(mShadowResolution);
    mDriver->setViewport(shadowArea);
    for (std::size_t i = 0; i < kept.size(); ++i)
        drawMesh(mCasters[kept[i]].mesh, static_cast<std::uint32_t>(i + 1), &Pipelines::shadow,
                false);
    mDriver->endRenderPass();
    if (scoped) profiler->endGpu();

    scoped = profiler != nullptr && profiler->beginGpu("Lightmap bake");
    prisma::RenderPassDesc bakePass;
    bakePass.colors[0].texture = mLightmap;
    bakePass.colorCount = 1;
    bakePass.colorLoad = mSample == 0 ? prisma::LoadOp::Clear : prisma::LoadOp::Load;
    bakePass.colorStore = prisma::StoreOp::Store;
    bakePass.clearColor[0] = 0.0f;
    bakePass.clearColor[1] = 0.0f;
    bakePass.clearColor[2] = 0.0f;
    bakePass.clearColor[3] = 0.0f;
    bakePass.depthLoad = prisma::LoadOp::DontCare;
    bakePass.depthStore = prisma::StoreOp::Discard;
    mDriver->beginRenderPass(bakePass);
    prisma::Viewport bakeArea;
    bakeArea.width = static_cast<float>(mResolution);
    bakeArea.height = static_cast<float>(mResolution);
    mDriver->setViewport(bakeArea);
    drawMesh(mMesh, 0, &Pipelines::bake, true);
    mDriver->endRenderPass();
    if (scoped) profiler->endGpu();
    return true;
}

bool LightmapBakePass::packAndRead()
{
    const std::int32_t size[4] = { static_cast<std::int32_t>(mResolution),
        static_cast<std::int32_t>(mResolution), static_cast<std::int32_t>(mResolution * 2),
        static_cast<std::int32_t>(mResolution) };
    mDriver->updateBuffer(mPackBlock, 0, size, sizeof(size));

    RenderProfiler* profiler = RenderProfiler::singleton();
    const bool scoped = profiler != nullptr && profiler->beginGpu("Lightmap pack");
    prisma::RenderPassDesc pass;
    pass.colors[0].texture = mPacked;
    pass.colorCount = 1;
    pass.colorLoad = prisma::LoadOp::DontCare;
    pass.colorStore = prisma::StoreOp::Store;
    pass.depthLoad = prisma::LoadOp::DontCare;
    pass.depthStore = prisma::StoreOp::Discard;
    mDriver->beginRenderPass(pass);
    prisma::Viewport area;
    area.width = static_cast<float>(mResolution * 2);
    area.height = static_cast<float>(mResolution);
    mDriver->setViewport(area);
    mDriver->bindPipeline(mPackPipeline);
    mDriver->bindUniformBuffer(0, mPackBlock, 0, sizeof(size));
    mDriver->bindTexture(0, mLightmap, mNearestSampler);
    mDriver->draw(3, 0);
    mDriver->endRenderPass();
    if (scoped) profiler->endGpu();

    const std::size_t texels = static_cast<std::size_t>(mResolution) * mResolution;
    mPixels.resize(texels * 4);
    prisma::RenderTarget source;
    source.texture = mPacked;
    prisma::Rect rect;
    rect.width = mResolution * 2;
    rect.height = mResolution;
    return mDriver->readPixels(source, rect, mPixels.data());
}

bool LightmapBakePass::step()
{
    if (mStage == Stage::Done) return true;
    if (mStage != Stage::Sampling && mStage != Stage::Packing) return false;
    RenderingServer* server = RenderingServer::singleton();
    if (!server || server->driver() != mDriver)
    {
        fail();
        return false;
    }
    server->openFrame();

    if (mStage == Stage::Sampling)
    {
        if (!renderSample())
        {
            fail();
            return false;
        }
        ++mSample;
        if (mSample >= mSamples) mStage = Stage::Packing;
        return true;
    }

    if (!packAndRead())
    {
        log_error("lightmap bake: could not read the lightmap back");
        fail();
        return false;
    }
    normalizeCoverage(mPixels, mSamples);
    dilate(mPixels, mResolution, 8);
    for (std::size_t i = 3; i < mPixels.size(); i += 4) mPixels[i] = kHalfOne;
    mStage = Stage::Done;
    return true;
}

void LightmapBakePass::cancel()
{
    mStage = Stage::Idle;
    mPixels.clear();
}

bool LightmapBakePass::active() const
{
    return mStage == Stage::Sampling || mStage == Stage::Packing;
}

bool LightmapBakePass::done() const
{
    return mStage == Stage::Done;
}

bool LightmapBakePass::failed() const
{
    return mStage == Stage::Failed;
}

std::uint32_t LightmapBakePass::samplesDone() const
{
    return mSample;
}

std::uint32_t LightmapBakePass::samplesTotal() const
{
    return mSamples;
}

std::uint32_t LightmapBakePass::resolution() const
{
    return mResolution;
}

const ct::Vector<std::uint16_t>& LightmapBakePass::pixels() const
{
    return mPixels;
}

void LightmapBakePass::normalizeCoverage(ct::Vector<std::uint16_t>& pixels, std::uint32_t samples)
{
    const float total = static_cast<float>(samples);
    for (std::size_t i = 0; i + 3 < pixels.size(); i += 4)
    {
        const float covered = LightmapGIData::halfToFloat(pixels[i + 3]);
        if (covered <= 0.0f || covered >= total) continue;
        const float scale = total / covered;
        for (int c = 0; c < 3; ++c)
            pixels[i + c] =
                    LightmapGIData::floatToHalf(LightmapGIData::halfToFloat(pixels[i + c]) * scale);
    }
}

void LightmapBakePass::dilate(ct::Vector<std::uint16_t>& pixels, std::uint32_t resolution,
        std::uint32_t iterations)
{
    struct Update
    {
        std::size_t index;
        std::uint16_t rgb[3];
    };
    ct::Vector<Update> updates;
    const std::int32_t size = static_cast<std::int32_t>(resolution);
    for (std::uint32_t iteration = 0; iteration < iterations; ++iteration)
    {
        updates.clear();
        for (std::int32_t y = 0; y < size; ++y)
            for (std::int32_t x = 0; x < size; ++x)
            {
                const std::size_t i = (static_cast<std::size_t>(y) * resolution + x) * 4;
                if (pixels[i + 3] != 0) continue;
                float sum[3] = { 0.0f, 0.0f, 0.0f };
                int count = 0;
                for (std::int32_t dy = -1; dy <= 1; ++dy)
                    for (std::int32_t dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0) continue;
                        const std::int32_t nx = x + dx;
                        const std::int32_t ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;
                        const std::size_t n = (static_cast<std::size_t>(ny) * resolution + nx) * 4;
                        if (pixels[n + 3] == 0) continue;
                        for (int c = 0; c < 3; ++c)
                            sum[c] += LightmapGIData::halfToFloat(pixels[n + c]);
                        ++count;
                    }
                if (count == 0) continue;
                Update update;
                update.index = i;
                for (int c = 0; c < 3; ++c)
                    update.rgb[c] = LightmapGIData::floatToHalf(sum[c] / static_cast<float>(count));
                updates.push_back(update);
            }
        if (updates.empty()) break;
        for (std::size_t u = 0; u < updates.size(); ++u)
        {
            const Update& update = updates[u];
            for (int c = 0; c < 3; ++c) pixels[update.index + c] = update.rgb[c];
            pixels[update.index + 3] = kHalfOne;
        }
    }
}

} // namespace prisma3d
