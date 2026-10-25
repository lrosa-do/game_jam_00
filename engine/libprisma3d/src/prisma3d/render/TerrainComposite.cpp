#include "prisma3d/render/TerrainComposite.h"
#include "prisma3d/render/ShaderVariants.h"

#include "platform.h"


namespace prisma3d
{

namespace
{

struct CompositeParams
{
    float uvMul0[4];
    float uvMul1[4];
    float lightDirection[4];
    float ambient[4];
    float diffuse[4];
    float params[4];
};

static_assert(sizeof(CompositeParams) == 96, "CompositeParams must match the std140 block");

const char* const kTextureNames[14] = { "terrain_normal_map", "terrain_lightmap",
    "terrain_colour_map", "terrain_blend", "terrain_diffuse0", "terrain_diffuse1",
    "terrain_diffuse2", "terrain_diffuse3", "terrain_diffuse4", "terrain_normalheight0",
    "terrain_normalheight1", "terrain_normalheight2", "terrain_normalheight3",
    "terrain_normalheight4" };

} // namespace

TerrainComposite::TerrainComposite()
        : mDriver(nullptr),
          mTextures(nullptr),
          mVertex(),
          mFragment(),
          mPipeline(),
          mParams(),
          mSampler(),
          mLayerSampler()
{
}

TerrainComposite::~TerrainComposite()
{
    destroy();
}

bool TerrainComposite::isCreated() const
{
    return mPipeline.valid();
}

bool TerrainComposite::create(prisma::Driver* driver, TextureStore* textures)
{
    destroy();
    mDriver = driver;
    mTextures = textures;
    if (!mDriver || !mTextures) return false;

    const prisma::Caps& caps = mDriver->caps();
    mVertex = mDriver->createShader(prisma3d::shaderDesc(tonemap_vert, caps));
    mFragment = mDriver->createShader(prisma3d::shaderDesc(terrain_composite_frag, caps));

    prisma::SamplerDesc samplerDesc;
    samplerDesc.minFilter = prisma::Filter::Linear;
    samplerDesc.magFilter = prisma::Filter::Linear;
    samplerDesc.mipFilter = prisma::MipFilter::None;
    samplerDesc.addressU = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressV = prisma::AddressMode::ClampToEdge;
    samplerDesc.addressW = prisma::AddressMode::ClampToEdge;
    samplerDesc.debugName = "terrain composite maps";
    mSampler = mDriver->createSampler(samplerDesc);

    prisma::SamplerDesc layerDesc;
    layerDesc.minFilter = prisma::Filter::Linear;
    layerDesc.magFilter = prisma::Filter::Linear;
    layerDesc.mipFilter = prisma::MipFilter::Linear;
    layerDesc.maxAnisotropy = 16.0f;
    layerDesc.addressU = prisma::AddressMode::Repeat;
    layerDesc.addressV = prisma::AddressMode::Repeat;
    layerDesc.addressW = prisma::AddressMode::Repeat;
    layerDesc.debugName = "terrain composite layers";
    mLayerSampler = mDriver->createSampler(layerDesc);

    prisma::BufferDesc bufferDesc;
    bufferDesc.usage = prisma::BufferUsage::Uniform;
    bufferDesc.size = sizeof(CompositeParams);
    bufferDesc.update = prisma::BufferUpdate::Dynamic;
    bufferDesc.debugName = "terrain composite params";
    mParams = mDriver->createBuffer(bufferDesc);

    if (!mVertex.valid() || !mFragment.valid() || !mSampler.valid() || !mLayerSampler.valid() ||
            !mParams.valid())
    {
        log_error("terrain: could not create the composite map resources");
        destroy();
        return false;
    }

    prisma::PipelineDesc desc;
    desc.vertexShader = mVertex;
    desc.fragmentShader = mFragment;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "TerrainCompositeParams";
    desc.uniformBlocks[0].slot = 0;
    desc.textureCount = 14;
    for (std::uint32_t i = 0; i < 14; ++i)
    {
        desc.textures[i].name = kTextureNames[i];
        desc.textures[i].slot = i;
    }
    desc.targets.window = false;
    desc.targets.colors[0] = prisma::TextureFormat::RGBA8Srgb;
    desc.targets.colorCount = 1;
    desc.targets.depth = prisma::TextureFormat::None;
    desc.depthTest = false;
    desc.depthWrite = false;
    desc.cullMode = prisma::CullMode::None;
    desc.debugName = "terrain composite";
    mPipeline = mDriver->createPipeline(desc);
    if (!mPipeline.valid())
    {
        log_error("terrain: could not create the composite map pipeline");
        destroy();
        return false;
    }
    return true;
}

void TerrainComposite::destroy()
{
    if (!mDriver) return;
    if (mPipeline.valid()) mDriver->destroy(mPipeline);
    if (mParams.valid()) mDriver->destroy(mParams);
    if (mSampler.valid()) mDriver->destroy(mSampler);
    if (mLayerSampler.valid()) mDriver->destroy(mLayerSampler);
    if (mFragment.valid()) mDriver->destroy(mFragment);
    if (mVertex.valid()) mDriver->destroy(mVertex);
    mPipeline = prisma::PipelineHandle();
    mParams = prisma::BufferHandle();
    mSampler = prisma::SamplerHandle();
    mLayerSampler = prisma::SamplerHandle();
    mFragment = prisma::ShaderHandle();
    mVertex = prisma::ShaderHandle();
    mDriver = nullptr;
    mTextures = nullptr;
}

bool TerrainComposite::render(const TerrainCompositeInputs& inputs, TextureId target,
        const TerrainRect& rect)
{
    if (!mPipeline.valid() || rect.isNull()) return false;
    const TextureInfo* info = mTextures->get(target);
    if (!info || !info->gpu.valid()) return false;

    const TerrainMaterialData& maps = inputs.maps;
    CompositeParams params = {};
    for (std::uint32_t i = 0; i < 4; ++i) params.uvMul0[i] = maps.uvMultiplier[i];
    params.uvMul1[0] = maps.uvMultiplier[4];
    params.uvMul1[1] = maps.detailModulation ? 1.0f : 0.0f;
    params.uvMul1[2] = maps.detailStrength;
    params.lightDirection[0] = inputs.lightDirection.x;
    params.lightDirection[1] = inputs.lightDirection.y;
    params.lightDirection[2] = inputs.lightDirection.z;
    params.ambient[0] = inputs.ambient.x;
    params.ambient[1] = inputs.ambient.y;
    params.ambient[2] = inputs.ambient.z;
    params.diffuse[0] = inputs.diffuse.x;
    params.diffuse[1] = inputs.diffuse.y;
    params.diffuse[2] = inputs.diffuse.z;
    params.params[0] = static_cast<float>(maps.layerCount);
    params.params[1] = maps.normalMapping ? 1.0f : 0.0f;
    params.params[2] = maps.useLightmap ? 1.0f : 0.0f;
    params.params[3] = maps.useColourMap ? 1.0f : 0.0f;
    mDriver->updateBuffer(mParams, 0, &params, sizeof(params));

    auto gpu = [&](TextureId id, TextureId fallback)
    { return mTextures->valid(id) ? mTextures->gpu(id) : mTextures->gpu(fallback); };

    prisma::RenderPassDesc pass;
    pass.colors[0].texture = info->gpu;
    pass.colorCount = 1;
    pass.colorLoad = prisma::LoadOp::Load;
    pass.depthLoad = prisma::LoadOp::DontCare;
    pass.depthStore = prisma::StoreOp::Discard;
    mDriver->beginRenderPass(pass);

    prisma::Viewport area;
    area.width = static_cast<float>(info->width);
    area.height = static_cast<float>(info->height);
    mDriver->setViewport(area);
    prisma::Rect scissor;
    scissor.x = rect.left;
    scissor.y = static_cast<std::int32_t>(info->height) - rect.bottom;
    scissor.width = static_cast<std::uint32_t>(rect.width());
    scissor.height = static_cast<std::uint32_t>(rect.height());
    mDriver->setScissor(scissor);

    mDriver->bindPipeline(mPipeline);
    mDriver->bindUniformBuffer(0, mParams, 0, sizeof(CompositeParams));
    mDriver->bindTexture(0, gpu(maps.normalMap, mTextures->white()), mSampler);
    mDriver->bindTexture(1, gpu(maps.lightmap, mTextures->white()), mSampler);
    mDriver->bindTexture(2, gpu(maps.colourMap, mTextures->white()), mSampler);
    mDriver->bindTexture(3, gpu(maps.blendMap, mTextures->black()), mSampler);
    for (std::uint32_t i = 0; i < TerrainMaterialData::kMaxLayers; ++i)
    {
        mDriver->bindTexture(4 + i, gpu(maps.layerDiffuse[i], mTextures->white()), mLayerSampler);
        mDriver->bindTexture(9 + i, gpu(maps.layerNormal[i], mTextures->normal()), mLayerSampler);
    }
    mDriver->draw(3, 0);
    mDriver->endRenderPass();
    mDriver->generateMipmaps(info->gpu);
    return true;
}

} // namespace prisma3d
