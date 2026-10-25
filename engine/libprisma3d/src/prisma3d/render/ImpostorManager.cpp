#include "ImpostorManager.h"
#include "prisma3d/render/ShaderVariants.h"

#include "SceneData.h"
#include "SceneMaterial.h"

#include "platform.h"

#include <ct/hashmap.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const std::uint32_t kCaptureVariantMask = 1u | 2u | 8u | 16u | 64u;
const std::uint32_t kMaxCaptureSurfaces = 64;
const std::uint32_t kMaxCaptureScenes = kMaxImpostorCount * kImpostorCaptureAngles;
const std::uint32_t kRequestedSamples = 8;
const float kTwoPi = 6.28318530717958647692f;
const std::uint32_t kCardCut = 0;
const std::uint32_t kCardCoverage = 1;
const std::uint32_t kCardDepth = 2;

std::uint32_t alignUp(std::uint32_t value, std::uint32_t alignment)
{
    if (alignment <= 1) return value;
    return (value + alignment - 1) / alignment * alignment;
}

void storeMatrix(const Math::Mat4& matrix, float* out)
{
    memcpy(out, &matrix, sizeof(float) * 16);
}

void storeTransform3x4(const Math::Mat4& matrix, float* out)
{
    const float* m = reinterpret_cast<const float*>(&matrix);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) out[r * 4 + c] = m[c * 4 + r];
}

Math::Mat4 zeroToOneDepth()
{
    Math::Mat4 matrix = Math::Mat4::Identity();
    matrix.col2.z = 0.5f;
    matrix.col3.z = 0.5f;
    return matrix;
}

bool validBox(const Math::Box& box)
{
    return box.max.x >= box.min.x && box.max.y >= box.min.y && box.max.z >= box.min.z;
}

void fillCaptureScene(SceneUboData& out, const Math::Mat4& camera, float radius)
{
    memset(&out, 0, sizeof(out));
    SceneUbo& ubo = out.data;
    const Math::Mat4 view = camera.InverseAffine();
    const Math::Mat4 projection =
            zeroToOneDepth() * Math::Mat4::Ortho(-radius, radius, -radius, radius, -radius, radius);
    storeMatrix(projection, ubo.projectionMatrix);
    storeMatrix(projection.Inverse(), ubo.invProjectionMatrix);
    storeTransform3x4(camera, ubo.invViewMatrix);
    storeTransform3x4(view, ubo.viewMatrix);
    storeMatrix(projection, ubo.projectionMatrixView[0]);
    storeMatrix(projection.Inverse(), ubo.invProjectionMatrixView[0]);
    storeMatrix(camera, ubo.mainCamInvViewMatrix);
    ubo.viewportSize[0] = ubo.viewportSize[1] = static_cast<float>(kImpostorTextureDim);
    ubo.screenPixelSize[0] = ubo.screenPixelSize[1] =
            1.0f / static_cast<float>(kImpostorTextureDim);
    ubo.zFar = radius;
    ubo.zNear = -radius;
    ubo.cameraVisibleLayers = 0xffffffffu;
    ubo.passAlphaMultiplier = 1.0f;
    ubo.emissiveExposureNormalization = 1.0f;
    ubo.iblExposureNormalization = 1.0f;
    out.prev = ubo;
}

void addBindings(prisma::PipelineDesc& desc, const prisma::ShaderBlob& blob)
{
    for (std::uint32_t i = 0; i < blob.bindingCount; ++i)
    {
        const prisma::ShaderBinding& binding = blob.bindings[i];
        if (binding.kind == prisma::BindingKind::UniformBlock)
        {
            bool known = false;
            for (std::uint32_t k = 0; k < desc.uniformBlockCount; ++k)
                known = known || desc.uniformBlocks[k].slot == binding.slot;
            if (known) continue;
            desc.uniformBlocks[desc.uniformBlockCount].name = binding.name;
            desc.uniformBlocks[desc.uniformBlockCount++].slot = binding.slot;
        }
        else if (binding.kind == prisma::BindingKind::Texture)
        {
            bool known = false;
            for (std::uint32_t k = 0; k < desc.textureCount; ++k)
                known = known || desc.textures[k].slot == binding.slot;
            if (known) continue;
            desc.textures[desc.textureCount].name = binding.name;
            desc.textures[desc.textureCount++].slot = binding.slot;
        }
        else if (binding.kind == prisma::BindingKind::StorageBuffer)
        {
            bool known = false;
            for (std::uint32_t k = 0; k < desc.storageBufferCount; ++k)
                known = known || desc.storageBuffers[k].slot == binding.slot;
            if (known) continue;
            desc.storageBuffers[desc.storageBufferCount].name = binding.name;
            desc.storageBuffers[desc.storageBufferCount++].slot = binding.slot;
        }
    }
}

void addBindings(prisma::PipelineDesc& desc, const ShaderVariant& variant, const prisma::Caps& caps)
{
    if (const prisma::ShaderBlob* blob = compileShader(variant, caps)) addBindings(desc, *blob);
}

struct CaptureLayout
{
    std::uint32_t entry;
    std::uint32_t firstScene;
    std::uint32_t firstSurface;
    float radius;
    Math::Vec3 centre;
};

} // namespace

ImpostorManager::ImpostorManager()
        : mPrepareParamsSize(0),
          mPrepareFailed(false),
          mSceneStride(0),
          mMaterialStride(0),
          mSamples(1),
          mCaptures(0),
          mFailed(false)
{
    for (std::uint32_t i = 0; i < kMaxImpostorCount; ++i) mTaken[i] = false;
}

ImpostorId ImpostorManager::create(MeshId mesh, const ct::Vector<MaterialId>& materials,
        float swapInDistance)
{
    int slot = -1;
    for (std::uint32_t i = 0; i < kMaxImpostorCount && slot < 0; ++i)
        if (!mTaken[i]) slot = static_cast<int>(i);
    if (slot < 0)
    {
        log_error("render: no free impostor slot");
        return ImpostorId();
    }
    Entry entry;
    entry.mesh = mesh;
    entry.materials = materials;
    entry.swapInDistance = swapInDistance;
    entry.slot = slot;
    mTaken[slot] = true;
    const ct::SlotMap32<Entry>::handle_type handle = mEntries.insert(entry);
    const ImpostorId id = ImpostorId::from_bits(handle.bits());
    mEntries.get(handle)->self = id;
    return id;
}

void ImpostorManager::free(ImpostorId id)
{
    Entry* entry = get(id);
    if (!entry) return;
    mTaken[entry->slot] = false;
    markSetsDirty();
    mEntries.erase(ct::SlotMap32<Entry>::handle_type::from_bits(id.bits()));
}

ImpostorManager::Entry* ImpostorManager::get(ImpostorId id)
{
    return mEntries.get(ct::SlotMap32<Entry>::handle_type::from_bits(id.bits()));
}

const ImpostorManager::Entry* ImpostorManager::get(ImpostorId id) const
{
    return mEntries.get(ct::SlotMap32<Entry>::handle_type::from_bits(id.bits()));
}

void ImpostorManager::setDirty(ImpostorId id)
{
    if (Entry* entry = get(id)) entry->dirty = true;
}

std::uint32_t ImpostorManager::count() const
{
    return static_cast<std::uint32_t>(mEntries.size());
}

prisma::TextureHandle ImpostorManager::array() const
{
    return mArray;
}

std::uint32_t ImpostorManager::captures() const
{
    return mCaptures;
}

std::uint32_t ImpostorManager::samples() const
{
    return mSamples;
}

std::uint64_t ImpostorManager::textureBytes() const
{
    if (!mArray.valid()) return 0;
    std::uint64_t bytes = static_cast<std::uint64_t>(kImpostorTextureDim) * kImpostorTextureDim *
                          4 * kMaxImpostorCount * kImpostorCaptureAngles * kImpostorProperties;
    const std::uint64_t target =
            static_cast<std::uint64_t>(kImpostorTextureDim) * kImpostorTextureDim * mSamples;
    bytes += target * 4 * kImpostorProperties + target * 4;
    return bytes;
}

bool ImpostorManager::ensureResources(prisma::Driver* driver)
{
    if (mArray.valid()) return true;
    if (mFailed) return false;
    const prisma::Caps& caps = driver->caps();
    prisma::TextureDesc arrayDesc;
    arrayDesc.type = prisma::TextureType::Texture2DArray;
    arrayDesc.format = prisma::TextureFormat::RGBA8;
    arrayDesc.width = kImpostorTextureDim;
    arrayDesc.height = kImpostorTextureDim;
    arrayDesc.depth = kMaxImpostorCount * kImpostorCaptureAngles * kImpostorProperties;
    arrayDesc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
    arrayDesc.debugName = "impostor array";
    mArray = driver->createTexture(arrayDesc);

    std::uint32_t samples = kRequestedSamples;
    while (samples > caps.maxSamples) samples >>= 1;
    if (samples < 1) samples = 1;
    for (;;)
    {
        bool created = true;
        for (std::uint32_t i = 0; i < kImpostorProperties && created; ++i)
        {
            prisma::TextureDesc desc;
            desc.format = prisma::TextureFormat::RGBA8;
            desc.width = kImpostorTextureDim;
            desc.height = kImpostorTextureDim;
            desc.samples = samples;
            desc.usage = prisma::kTextureRenderTarget;
            desc.debugName = "impostor capture colour";
            mColor[i] = driver->createTexture(desc);
            created = mColor[i].valid();
        }
        if (created)
        {
            prisma::TextureDesc desc;
            desc.format = prisma::TextureFormat::Depth32F;
            desc.width = kImpostorTextureDim;
            desc.height = kImpostorTextureDim;
            desc.samples = samples;
            desc.usage = prisma::kTextureRenderTarget;
            desc.debugName = "impostor capture depth";
            mDepth = driver->createTexture(desc);
            created = mDepth.valid();
        }
        if (created) break;
        for (std::uint32_t i = 0; i < kImpostorProperties; ++i)
        {
            if (mColor[i].valid()) driver->destroy(mColor[i]);
            mColor[i] = prisma::TextureHandle();
        }
        if (mDepth.valid()) driver->destroy(mDepth);
        mDepth = prisma::TextureHandle();
        if (samples == 1) break;
        samples >>= 1;
    }
    mSamples = samples;

    prisma::SamplerDesc sampler;
    sampler.minFilter = prisma::Filter::Linear;
    sampler.magFilter = prisma::Filter::Linear;
    sampler.mipFilter = prisma::MipFilter::Linear;
    sampler.addressU = prisma::AddressMode::Repeat;
    sampler.addressV = prisma::AddressMode::Repeat;
    mSampler = driver->createSampler(sampler);
    prisma::SamplerDesc cardSampler;
    cardSampler.minFilter = prisma::Filter::Linear;
    cardSampler.magFilter = prisma::Filter::Linear;
    cardSampler.mipFilter = prisma::MipFilter::Linear;
    cardSampler.addressU = prisma::AddressMode::ClampToEdge;
    cardSampler.addressV = prisma::AddressMode::ClampToEdge;
    cardSampler.addressW = prisma::AddressMode::ClampToEdge;
    mCardSampler = driver->createSampler(cardSampler);

    mSceneStride = alignUp(static_cast<std::uint32_t>(sizeof(SceneUboData)),
            caps.uniformBufferOffsetAlignment);
    mMaterialStride = alignUp(static_cast<std::uint32_t>(sizeof(MaterialDataGpu)),
            caps.uniformBufferOffsetAlignment);

    prisma::BufferDesc scene;
    scene.usage = prisma::BufferUsage::Uniform;
    scene.update = prisma::BufferUpdate::Dynamic;
    scene.size = mSceneStride * kMaxCaptureScenes;
    scene.debugName = "impostor capture scene";
    mSceneBuffer = driver->createBuffer(scene);

    prisma::BufferDesc material;
    material.usage = prisma::BufferUsage::Uniform;
    material.update = prisma::BufferUpdate::Dynamic;
    material.size = mMaterialStride * kMaxCaptureSurfaces;
    material.debugName = "impostor capture materials";
    mMaterialBuffer = driver->createBuffer(material);

    DrawCallData draw = {};
    draw.color[0] = draw.color[1] = draw.color[2] = draw.color[3] = 1.0f;
    prisma::BufferDesc drawDesc;
    drawDesc.usage = prisma::BufferUsage::Uniform;
    drawDesc.size = static_cast<std::uint32_t>(sizeof(DrawCallData));
    drawDesc.data = &draw;
    drawDesc.debugName = "impostor capture draw call";
    mDrawBuffer = driver->createBuffer(drawDesc);

    InstanceDataGpu instance;
    memset(&instance, 0, sizeof(instance));
    instance.transform[0] = instance.transform[5] = instance.transform[10] = 1.0f;
    instance.prevTransform[0] = instance.prevTransform[5] = instance.prevTransform[10] = 1.0f;
    instance.compressedAabbSize[0] = instance.compressedAabbSize[1] =
            instance.compressedAabbSize[2] = 1.0f;
    instance.materialFeedbackIndex = 0xffffffffu;
    instance.flags = 255u << 24;
    instance.layerMask = 0xffffffffu;
    prisma::BufferDesc instanceDesc;
    instanceDesc.usage = prisma::BufferUsage::Storage;
    instanceDesc.size = static_cast<std::uint32_t>(sizeof(InstanceDataGpu));
    instanceDesc.data = &instance;
    instanceDesc.debugName = "impostor capture instance";
    mInstanceBuffer = driver->createBuffer(instanceDesc);

    if (!mArray.valid() || !mColor[0].valid() || !mDepth.valid() || !mSampler.valid() ||
            !mCardSampler.valid() || !mSceneBuffer.valid() || !mMaterialBuffer.valid() ||
            !mDrawBuffer.valid() || !mInstanceBuffer.valid())
    {
        log_error("render: could not create the impostor resources");
        destroyCapture(driver);
        mFailed = true;
        return false;
    }
    return true;
}

prisma::PipelineHandle ImpostorManager::capturePipeline(prisma::Driver* driver,
        const SurfaceInfo& surface, std::uint32_t variant)
{
    const std::uint64_t key = static_cast<std::uint64_t>(surface.attributeMask) |
                              (static_cast<std::uint64_t>(surface.topology) << 8) |
                              (static_cast<std::uint64_t>(variant) << 16);
    if (const prisma::PipelineHandle* found = mPipelines.find(key)) return *found;

    const bool normalMap = (variant & 8u) != 0;
    const bool detail = (variant & 64u) != 0;
    const std::uint32_t vertexIndex = (normalMap ? 1u : 0u) | (detail ? 2u : 0u);
    const ShaderVariant* vertexBlob = kImpostorCaptureVertexBlobs[vertexIndex];
    const ShaderVariant* fragmentBlob = kImpostorCaptureFragmentBlobs[variant];
    if (!fragmentBlob) return prisma::PipelineHandle();
    prisma::ShaderHandle* vertexShader = mShaders.find(0x100u | vertexIndex);
    if (!vertexShader)
    {
        mShaders[0x100u | vertexIndex] =
                driver->createShader(prisma3d::shaderDesc(*vertexBlob, driver->caps()));
        vertexShader = mShaders.find(0x100u | vertexIndex);
    }
    prisma::ShaderHandle* fragmentShader = mShaders.find(variant);
    if (!fragmentShader)
    {
        mShaders[variant] =
                driver->createShader(prisma3d::shaderDesc(*fragmentBlob, driver->caps()));
        fragmentShader = mShaders.find(variant);
    }
    if (!vertexShader->valid() || !fragmentShader->valid()) return prisma::PipelineHandle();

    const std::uint32_t mask = surface.attributeMask;
    prisma::PipelineDesc desc;
    desc.vertexShader = *vertexShader;
    desc.fragmentShader = *fragmentShader;
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
    desc.attributes[count++].offset = (mask & kVertexUv)
                                              ? MeshStore::attributeOffset(mask, kVertexUv)
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
    desc.topology = surface.topology;

    addBindings(desc, *vertexBlob, driver->caps());
    addBindings(desc, *fragmentBlob, driver->caps());

    desc.targets.window = false;
    desc.targets.samples = mSamples;
    for (std::uint32_t i = 0; i < kImpostorProperties; ++i)
        desc.targets.colors[i] = prisma::TextureFormat::RGBA8;
    desc.targets.colorCount = kImpostorProperties;
    desc.targets.depth = prisma::TextureFormat::Depth32F;
    desc.depthTest = true;
    desc.depthWrite = true;
    desc.depthCompare = prisma::CompareOp::Less;
    desc.cullMode = prisma::CullMode::None;
    desc.frontFace = prisma::FrontFace::CounterClockwise;
    desc.debugName = "impostor capture pipeline";
    const prisma::PipelineHandle pipeline = driver->createPipeline(desc);
    if (!pipeline.valid()) log_error("render: could not create the impostor capture pipeline");
    mPipelines[key] = pipeline;
    return pipeline;
}

bool ImpostorManager::entryReady(const Entry& entry, const MeshStore& meshes,
        const MaterialStore& materials, const TextureStore& textures) const
{
    const MeshInfo* mesh = meshes.get(entry.mesh);
    if (!mesh || mesh->surfaces.empty() || mesh->pendingSurfaces != 0 || !validBox(mesh->aabb))
        return false;
    for (const SurfaceInfo& surface: mesh->surfaces)
        if (!surface.ready) return false;
    for (std::size_t s = 0; s < mesh->surfaces.size(); ++s)
    {
        MaterialId id = s < entry.materials.size() ? entry.materials[s] : MaterialId();
        if (!id.valid()) id = mesh->surfaces[s].material;
        const MaterialInfo* material = materials.get(id);
        if (!material) continue;
        for (std::size_t t = 0; t < static_cast<std::size_t>(MaterialTexture::Count); ++t)
        {
            const TextureInfo* texture = textures.get(material->textures[t]);
            if (texture && texture->state == TextureState::Pending) return false;
        }
    }
    return true;
}

std::uint32_t ImpostorManager::refresh(prisma::Driver* driver, MeshStore& meshes,
        MaterialStore& materials, TextureStore& textures)
{
    const prisma::Caps& caps = driver->caps();
    if (!caps.compute || !caps.indirectDraw || !caps.storageBuffersInGraphics) return 0;

    ct::Vector<ImpostorId> ready;
    for (Entry& entry: mEntries)
    {
        if (!entry.dirty) continue;
        if (!entryReady(entry, meshes, materials, textures)) continue;
        ready.push_back(entry.self);
    }
    if (ready.empty() || !ensureResources(driver)) return 0;

    ct::Vector<CaptureLayout> layouts;
    ct::Vector<std::uint8_t> sceneBytes;
    ct::Vector<std::uint8_t> materialBytes;
    ct::Vector<ImpostorId> batch;
    std::uint32_t scenes = 0;
    std::uint32_t surfaces = 0;
    sceneBytes.resize(static_cast<std::size_t>(mSceneStride) * kMaxCaptureScenes);
    materialBytes.resize(static_cast<std::size_t>(mMaterialStride) * kMaxCaptureSurfaces);
    for (ImpostorId id: ready)
    {
        Entry* entry = get(id);
        const MeshInfo* mesh = meshes.get(entry->mesh);
        const std::uint32_t surfaceCount = static_cast<std::uint32_t>(mesh->surfaces.size());
        if (scenes + kImpostorCaptureAngles > kMaxCaptureScenes ||
                surfaces + surfaceCount > kMaxCaptureSurfaces)
            continue;
        CaptureLayout layout;
        layout.entry = static_cast<std::uint32_t>(batch.size());
        layout.firstScene = scenes;
        layout.firstSurface = surfaces;
        layout.centre = mesh->aabb.Center();
        const Math::Vec3 half = mesh->aabb.Extents();
        layout.radius = sqrtf(half.x * half.x + half.y * half.y + half.z * half.z);
        if (layout.radius <= 0.0f) continue;
        for (std::uint32_t angle = 0; angle < kImpostorCaptureAngles; ++angle)
        {
            const float phi = -kTwoPi * static_cast<float>(angle) /
                              static_cast<float>(kImpostorCaptureAngles);
            const Math::Mat4 camera =
                    Math::Mat4::Translation(layout.centre) * Math::Mat4::RotationY(phi);
            SceneUboData data;
            fillCaptureScene(data, camera, layout.radius);
            memcpy(sceneBytes.data() + static_cast<std::size_t>(scenes + angle) * mSceneStride,
                    &data, sizeof(data));
        }
        for (std::uint32_t s = 0; s < surfaceCount; ++s)
        {
            MaterialId materialId =
                    s < entry->materials.size() ? entry->materials[s] : MaterialId();
            if (!materialId.valid()) materialId = mesh->surfaces[s].material;
            MaterialDataGpu data;
            fillMaterial(materials.get(materialId), data, &textures);
            memcpy(materialBytes.data() + static_cast<std::size_t>(surfaces + s) * mMaterialStride,
                    &data, sizeof(data));
        }
        scenes += kImpostorCaptureAngles;
        surfaces += surfaceCount;
        layouts.push_back(layout);
        batch.push_back(id);
    }
    if (batch.empty()) return 0;

    driver->updateBuffer(mSceneBuffer, 0, sceneBytes.data(), mSceneStride * scenes);
    driver->updateBuffer(mMaterialBuffer, 0, materialBytes.data(), mMaterialStride * surfaces);

    prisma::Viewport area;
    area.width = static_cast<float>(kImpostorTextureDim);
    area.height = static_cast<float>(kImpostorTextureDim);
    for (std::size_t b = 0; b < batch.size(); ++b)
    {
        Entry* entry = get(batch[b]);
        const MeshInfo* mesh = meshes.get(entry->mesh);
        const CaptureLayout& layout = layouts[b];
        for (std::uint32_t angle = 0; angle < kImpostorCaptureAngles; ++angle)
        {
            prisma::RenderPassDesc pass;
            for (std::uint32_t i = 0; i < kImpostorProperties; ++i)
            {
                pass.colors[i].texture = mColor[i];
                pass.resolves[i].texture = mArray;
                pass.resolves[i].layer = impostorSlice(static_cast<std::uint32_t>(entry->slot),
                        angle, static_cast<ImpostorProperty>(i));
            }
            pass.colorCount = kImpostorProperties;
            pass.depth.texture = mDepth;
            pass.colorStore = prisma::StoreOp::Discard;
            pass.depthStore = prisma::StoreOp::Discard;
            pass.clearColor[0] = pass.clearColor[1] = pass.clearColor[2] = pass.clearColor[3] =
                    0.0f;
            pass.clearDepth = 1.0f;
            driver->beginRenderPass(pass);
            driver->setViewport(area);
            driver->bindUniformBuffer(0, mSceneBuffer, (layout.firstScene + angle) * mSceneStride,
                    static_cast<std::uint32_t>(sizeof(SceneUboData)));
            driver->bindUniformBuffer(1, mDrawBuffer, 0,
                    static_cast<std::uint32_t>(sizeof(DrawCallData)));
            driver->bindStorageBuffer(0, mInstanceBuffer, 0,
                    static_cast<std::uint32_t>(sizeof(InstanceDataGpu)));
            for (std::size_t s = 0; s < mesh->surfaces.size(); ++s)
            {
                const SurfaceInfo& surface = mesh->surfaces[s];
                if (surface.vertexCount == 0) continue;
                MaterialId materialId =
                        s < entry->materials.size() ? entry->materials[s] : MaterialId();
                if (!materialId.valid()) materialId = surface.material;
                const MaterialInfo* material = materials.get(materialId);
                std::uint32_t variant = sceneVariant(material, surface, textures);
                if (variant & 32u) variant &= 1u | 16u | 64u;
                variant &= kCaptureVariantMask;
                const prisma::PipelineHandle pipeline = capturePipeline(driver, surface, variant);
                if (!pipeline.valid()) continue;
                driver->bindPipeline(pipeline);
                driver->bindUniformBuffer(2, mMaterialBuffer,
                        (layout.firstSurface + static_cast<std::uint32_t>(s)) * mMaterialStride,
                        static_cast<std::uint32_t>(sizeof(MaterialDataGpu)));
                if (material)
                {
                    auto gpu = [&](MaterialTexture texture, bool white)
                    {
                        const TextureId id = sceneTexture(*material, texture);
                        if (textures.valid(id)) return textures.gpu(id);
                        return textures.gpu(white ? textures.white() : textures.black());
                    };
                    if (variant & 1u)
                        driver->bindTexture(0, gpu(MaterialTexture::Albedo, true), mSampler);
                    if (variant & 2u)
                    {
                        driver->bindTexture(1, gpu(MaterialTexture::Metallic, true), mSampler);
                        driver->bindTexture(2, gpu(MaterialTexture::Roughness, true), mSampler);
                        driver->bindTexture(5, gpu(MaterialTexture::AmbientOcclusion, true),
                                mSampler);
                    }
                    if (variant & 8u)
                    {
                        const TextureId normal = sceneTexture(*material, MaterialTexture::Normal);
                        driver->bindTexture(4,
                                textures.gpu(textures.valid(normal) ? normal : textures.normal()),
                                mSampler);
                    }
                    if (variant & 64u)
                    {
                        driver->bindTexture(14, gpu(MaterialTexture::DetailAlbedo, true), mSampler);
                        if (variant & 8u)
                        {
                            const TextureId detailNormal =
                                    sceneTexture(*material, MaterialTexture::DetailNormal);
                            driver->bindTexture(15,
                                    textures.gpu(textures.valid(detailNormal) ? detailNormal
                                                                              : textures.normal()),
                                    mSampler);
                        }
                        driver->bindTexture(16, gpu(MaterialTexture::DetailMask, true), mSampler);
                    }
                }
                driver->bindVertexBuffer(0, surface.vertexBuffer, 0);
                if (surface.indexCount > 0)
                {
                    driver->bindIndexBuffer(surface.indexBuffer);
                    driver->drawIndexed(surface.indexCount, 0);
                }
                else
                    driver->draw(surface.vertexCount, 0);
            }
            driver->endRenderPass();
        }
        entry->dirty = false;
        entry->captured = true;
        ++mCaptures;
    }
    return static_cast<std::uint32_t>(batch.size());
}

ImpostorSetId ImpostorManager::createSet(std::uint32_t scenario)
{
    Set set;
    set.scenario = scenario;
    return ImpostorSetId::from_bits(mSets.insert(set).bits());
}

void ImpostorManager::freeSet(prisma::Driver* driver, ImpostorSetId id)
{
    const ct::SlotMap32<Set>::handle_type handle =
            ct::SlotMap32<Set>::handle_type::from_bits(id.bits());
    Set* set = mSets.get(handle);
    if (!set) return;
    if (set->instances.valid()) driver->destroy(set->instances);
    if (set->output.valid()) driver->destroy(set->output);
    if (set->indices.valid()) driver->destroy(set->indices);
    if (set->cardParams.valid()) driver->destroy(set->cardParams);
    mSets.erase(handle);
}

ImpostorManager::Set* ImpostorManager::getSet(ImpostorSetId id)
{
    return mSets.get(ct::SlotMap32<Set>::handle_type::from_bits(id.bits()));
}

const ImpostorManager::Set* ImpostorManager::getSet(ImpostorSetId id) const
{
    return mSets.get(ct::SlotMap32<Set>::handle_type::from_bits(id.bits()));
}

std::uint32_t ImpostorManager::setCount() const
{
    return static_cast<std::uint32_t>(mSets.size());
}

void ImpostorManager::markSetsDirty()
{
    for (Set& set: mSets) set.dirty = true;
}

bool ImpostorManager::instanceInfo(const MeshStore& meshes, const Record& record,
        ImpostorInstanceInfo& info) const
{
    const Entry* entry = get(record.impostor);
    if (!entry) return false;
    const MeshInfo* mesh = meshes.get(entry->mesh);
    if (!mesh || !validBox(mesh->aabb)) return false;
    const Math::Box box = mesh->aabb.Transformed(record.transform);
    const Math::Vec3 centre = box.Center();
    const Math::Vec3 half = box.Extents();
    info.center[0] = centre.x;
    info.center[1] = centre.y;
    info.center[2] = centre.z;
    info.radius = sqrtf(half.x * half.x + half.y * half.y + half.z * half.z);
    info.fadeDistance = entry->swapInDistance;
    info.sliceOffset =
            entry->slot >= 0
                    ? static_cast<int>(impostorSliceOffset(static_cast<std::uint32_t>(entry->slot)))
                    : -1;
    return true;
}

bool ImpostorManager::ensureSetBuffers(prisma::Driver* driver, Set& set, std::uint32_t count)
{
    if (set.capacity >= count && set.instances.valid()) return true;
    std::uint32_t capacity = set.capacity > 16 ? set.capacity : 16;
    while (capacity < count) capacity *= 2;
    if (set.instances.valid()) driver->destroy(set.instances);
    if (set.output.valid()) driver->destroy(set.output);
    if (set.indices.valid()) driver->destroy(set.indices);
    if (set.cardParams.valid()) driver->destroy(set.cardParams);
    set.instances = prisma::BufferHandle();
    set.output = prisma::BufferHandle();
    set.indices = prisma::BufferHandle();
    set.cardParams = prisma::BufferHandle();
    set.capacity = 0;

    prisma::BufferDesc instances;
    instances.usage = prisma::BufferUsage::Storage;
    instances.update = prisma::BufferUpdate::Dynamic;
    instances.size = capacity * 32;
    instances.debugName = "impostor instances";
    set.instances = driver->createBuffer(instances);

    prisma::BufferDesc output;
    output.usage = prisma::BufferUsage::Storage;
    output.update = prisma::BufferUpdate::Dynamic;
    output.size = (8 + capacity * 16 * 2 + capacity * 2) * 4;
    output.debugName = "impostor output";
    set.output = driver->createBuffer(output);

    const bool wide = capacity * 4 > 65536;
    ct::Vector<std::uint8_t> bytes;
    bytes.resize(static_cast<std::size_t>(capacity) * 6 * (wide ? 4 : 2));
    for (std::uint32_t i = 0; i < capacity; ++i)
    {
        const std::uint32_t base = i * 4;
        const std::uint32_t pattern[6] = { base, base + 1, base + 2, base + 3, base + 2, base + 1 };
        for (std::uint32_t k = 0; k < 6; ++k)
        {
            if (wide) memcpy(bytes.data() + (i * 6 + k) * 4, &pattern[k], 4);
            else
            {
                const std::uint16_t narrow = static_cast<std::uint16_t>(pattern[k]);
                memcpy(bytes.data() + (i * 6 + k) * 2, &narrow, 2);
            }
        }
    }
    prisma::BufferDesc indices;
    indices.usage = prisma::BufferUsage::Index;
    indices.indexFormat = wide ? prisma::IndexFormat::UInt32 : prisma::IndexFormat::UInt16;
    indices.size = static_cast<std::uint32_t>(bytes.size());
    indices.data = bytes.data();
    indices.debugName = "impostor indices";
    set.indices = driver->createBuffer(indices);

    DrawCallData params = {};
    params.pad[0] = capacity;
    params.color[0] = params.color[1] = params.color[2] = params.color[3] = 1.0f;
    prisma::BufferDesc paramsDesc;
    paramsDesc.usage = prisma::BufferUsage::Uniform;
    paramsDesc.size = static_cast<std::uint32_t>(sizeof(DrawCallData));
    paramsDesc.data = &params;
    paramsDesc.debugName = "impostor card params";
    set.cardParams = driver->createBuffer(paramsDesc);

    if (!set.instances.valid() || !set.output.valid() || !set.indices.valid() ||
            !set.cardParams.valid())
    {
        log_error("render: could not create the impostor buffers");
        return false;
    }
    set.capacity = capacity;
    set.dirty = true;
    return true;
}

void ImpostorManager::uploadSets(prisma::Driver* driver, const MeshStore& meshes)
{
    for (Set& set: mSets)
    {
        if (!set.enabled || (!set.dirty && !set.waiting)) continue;
        const std::uint32_t count = static_cast<std::uint32_t>(set.records.size());
        if (count == 0)
        {
            set.uploaded = 0;
            set.dirty = false;
            set.waiting = false;
            continue;
        }
        if (!ensureSetBuffers(driver, set, count)) continue;
        ct::Vector<float> words;
        words.resize(static_cast<std::size_t>(count) * 8);
        bool waiting = false;
        for (std::uint32_t i = 0; i < count; ++i)
        {
            float* at = words.data() + static_cast<std::size_t>(i) * 8;
            ImpostorInstanceInfo info;
            const Record& record = set.records[i];
            const bool resolved = instanceInfo(meshes, record, info);
            if (!resolved && record.impostor.valid() && get(record.impostor)) waiting = true;
            at[0] = info.center[0];
            at[1] = info.center[1];
            at[2] = info.center[2];
            at[3] = info.radius;
            at[4] = info.fadeDistance;
            const std::int32_t slice = resolved ? info.sliceOffset : -1;
            memcpy(&at[5], &slice, 4);
            memcpy(&at[6], &record.color, 4);
            at[7] = 0.0f;
        }
        driver->updateBuffer(set.instances, 0, words.data(),
                static_cast<std::uint32_t>(words.size() * sizeof(float)));
        set.uploaded = count;
        set.dirty = false;
        set.waiting = waiting;
    }
}

bool ImpostorManager::ensurePrepare(prisma::Driver* driver)
{
    if (mPreparePipeline.valid()) return true;
    if (mPrepareFailed) return false;
    const prisma::Caps& caps = driver->caps();
    if (!caps.compute || !caps.indirectDraw || !caps.storageBuffersInGraphics)
    {
        mPrepareFailed = true;
        return false;
    }
    mPrepareShader = driver->createShader(prisma3d::shaderDesc(impostor_prepare_comp, caps));
    prisma::ComputePipelineDesc desc;
    desc.shader = mPrepareShader;
    desc.uniformBlockCount = 1;
    desc.uniformBlocks[0].name = "Params";
    desc.uniformBlocks[0].slot = 0;
    desc.storageBufferCount = 2;
    desc.storageBuffers[0].name = "Instances";
    desc.storageBuffers[0].slot = 0;
    desc.storageBuffers[1].name = "Output";
    desc.storageBuffers[1].slot = 1;
    desc.debugName = "impostor prepare";
    if (mPrepareShader.valid()) mPreparePipeline = driver->createComputePipeline(desc);
    if (!mPreparePipeline.valid())
    {
        log_error("render: could not create the impostor prepare pipeline");
        mPrepareFailed = true;
        return false;
    }
    return true;
}

std::uint32_t ImpostorManager::prepare(prisma::Driver* driver, std::uint32_t scenario,
        const Math::Frustum& frustum, const Math::Vec3& camera)
{
    struct PrepareParams
    {
        float camera[4];
        float planes[6][4];
        std::uint32_t count;
        std::uint32_t capacity;
        std::uint32_t pad[2];
    };
    static_assert(sizeof(PrepareParams) == 128, "PrepareParams must match the std140 Params block");

    ct::Vector<Set*> sets;
    for (Set& set: mSets)
        if (set.enabled && set.scenario == scenario && set.uploaded > 0 && set.output.valid())
            sets.push_back(&set);
    if (sets.empty() || !ensurePrepare(driver)) return 0;

    const std::uint32_t stride = alignUp(static_cast<std::uint32_t>(sizeof(PrepareParams)),
            driver->caps().uniformBufferOffsetAlignment);
    const std::uint32_t needed = stride * static_cast<std::uint32_t>(sets.size());
    if (!mPrepareParams.valid() || mPrepareParamsSize < needed)
    {
        if (mPrepareParams.valid()) driver->destroy(mPrepareParams);
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Uniform;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = needed * 2;
        desc.debugName = "impostor prepare params";
        mPrepareParams = driver->createBuffer(desc);
        mPrepareParamsSize = mPrepareParams.valid() ? desc.size : 0;
        if (!mPrepareParams.valid()) return 0;
    }
    ct::Vector<std::uint8_t> bytes;
    bytes.resize(needed);
    memset(bytes.data(), 0, bytes.size());
    for (std::size_t i = 0; i < sets.size(); ++i)
    {
        PrepareParams params;
        memset(&params, 0, sizeof(params));
        params.camera[0] = camera.x;
        params.camera[1] = camera.y;
        params.camera[2] = camera.z;
        for (int p = 0; p < 6; ++p)
        {
            params.planes[p][0] = frustum.planes[p].normal.x;
            params.planes[p][1] = frustum.planes[p].normal.y;
            params.planes[p][2] = frustum.planes[p].normal.z;
            params.planes[p][3] = frustum.planes[p].d;
        }
        params.count = sets[i]->uploaded;
        params.capacity = sets[i]->capacity;
        memcpy(bytes.data() + i * stride, &params, sizeof(params));
    }
    driver->updateBuffer(mPrepareParams, 0, bytes.data(), needed);

    const std::uint32_t clear[8] = { 0, 1, 0, 0, 0, 0, 0, 0 };
    for (Set* set: sets) driver->updateBuffer(set->output, 0, clear, sizeof(clear));

    driver->beginComputePass();
    driver->bindPipeline(mPreparePipeline);
    for (std::size_t i = 0; i < sets.size(); ++i)
    {
        Set& set = *sets[i];
        driver->bindUniformBuffer(0, mPrepareParams, static_cast<std::uint32_t>(i) * stride,
                static_cast<std::uint32_t>(sizeof(PrepareParams)));
        driver->bindStorageBuffer(0, set.instances, 0, set.uploaded * 32);
        driver->bindStorageBuffer(1, set.output, 0,
                (8 + set.capacity * 16 * 2 + set.capacity * 2) * 4);
        driver->dispatch((set.uploaded + 63) / 64, 1, 1);
    }
    driver->endComputePass();
    return static_cast<std::uint32_t>(sets.size());
}

prisma::PipelineHandle ImpostorManager::cardPipeline(prisma::Driver* driver, std::uint32_t variant,
        std::uint32_t samples, bool equalDepth)
{
    const std::uint32_t key = variant | (samples << 4) | (equalDepth ? 0x1000u : 0u);
    if (const prisma::PipelineHandle* found = mCardPipelines.find(key)) return *found;

    const ShaderVariant* vertexBlob = kImpostorCardVertexBlob;
    const ShaderVariant* fragmentBlob = kImpostorCardFragmentBlobs[variant];
    prisma::ShaderHandle* vertexShader = mShaders.find(0x200u);
    if (!vertexShader)
    {
        mShaders[0x200u] = driver->createShader(prisma3d::shaderDesc(*vertexBlob, driver->caps()));
        vertexShader = mShaders.find(0x200u);
    }
    prisma::ShaderHandle* fragmentShader = mShaders.find(0x210u + variant);
    if (!fragmentShader)
    {
        mShaders[0x210u + variant] =
                driver->createShader(prisma3d::shaderDesc(*fragmentBlob, driver->caps()));
        fragmentShader = mShaders.find(0x210u + variant);
    }
    if (!vertexShader->valid() || !fragmentShader->valid()) return prisma::PipelineHandle();

    const bool depthOnly = variant == kCardDepth;
    prisma::PipelineDesc desc;
    desc.vertexShader = *vertexShader;
    desc.fragmentShader = *fragmentShader;
    desc.attributeCount = 0;
    desc.vertexBufferCount = 0;
    desc.topology = prisma::Topology::Triangles;
    addBindings(desc, *vertexBlob, driver->caps());
    addBindings(desc, *fragmentBlob, driver->caps());
    desc.targets.window = false;
    desc.targets.samples = samples;
    desc.targets.colorCount = depthOnly ? 0 : 1;
    desc.targets.colors[0] = prisma::TextureFormat::RGBA16F;
    desc.targets.depth = prisma::TextureFormat::Depth32F;
    desc.depthTest = true;
    desc.depthWrite = !equalDepth;
    desc.depthCompare = equalDepth ? prisma::CompareOp::Equal : prisma::CompareOp::LessEqual;
    desc.alphaToCoverage = variant == kCardCoverage;
    desc.cullMode = prisma::CullMode::None;
    desc.frontFace = prisma::FrontFace::CounterClockwise;
    desc.debugName = "impostor card pipeline";
    const prisma::PipelineHandle pipeline = driver->createPipeline(desc);
    if (!pipeline.valid()) log_error("render: could not create the impostor card pipeline");
    mCardPipelines[key] = pipeline;
    return pipeline;
}

std::uint32_t ImpostorManager::drawCards(prisma::Driver* driver, std::uint32_t scenario,
        std::uint32_t variant, std::uint32_t samples, bool equalDepth)
{
    if (!mArray.valid() || !mCardSampler.valid() || !mInstanceBuffer.valid()) return 0;
    ct::Vector<Set*> sets;
    for (Set& set: mSets)
        if (set.enabled && set.scenario == scenario && set.uploaded > 0 && set.output.valid() &&
                set.indices.valid() && set.cardParams.valid() && !mPrepareFailed)
            sets.push_back(&set);
    if (sets.empty()) return 0;
    const prisma::PipelineHandle pipeline = cardPipeline(driver, variant, samples, equalDepth);
    if (!pipeline.valid()) return 0;
    driver->bindPipeline(pipeline);
    driver->bindStorageBuffer(0, mInstanceBuffer, 0,
            static_cast<std::uint32_t>(sizeof(InstanceDataGpu)));
    driver->bindTexture(14, mArray, mCardSampler);
    for (Set* set: sets)
    {
        driver->bindUniformBuffer(1, set->cardParams, 0,
                static_cast<std::uint32_t>(sizeof(DrawCallData)));
        driver->bindStorageBuffer(5, set->output, 0,
                (8 + set->capacity * 16 * 2 + set->capacity * 2) * 4);
        driver->bindIndexBuffer(set->indices);
        driver->drawIndexedIndirect(set->output, 0, 1);
    }
    return static_cast<std::uint32_t>(sets.size());
}

std::uint32_t ImpostorManager::drawDepth(prisma::Driver* driver, std::uint32_t scenario,
        std::uint32_t samples)
{
    return drawCards(driver, scenario, kCardDepth, samples, false);
}

std::uint32_t ImpostorManager::drawColor(prisma::Driver* driver, std::uint32_t scenario,
        std::uint32_t samples, bool equalDepth)
{
    const std::uint32_t variant = equalDepth || samples <= 1 ? kCardCut : kCardCoverage;
    return drawCards(driver, scenario, variant, samples, equalDepth);
}

bool ImpostorManager::readCards(prisma::Driver* driver, ImpostorSetId id, std::uint32_t& indexCount,
        ct::Vector<ImpostorCardReadback>& cards) const
{
    const Set* set = getSet(id);
    if (!set) return false;
    cards.clear();
    indexCount = 0;
    if (!set->enabled || set->uploaded == 0 || !set->output.valid()) return true;
    const std::uint32_t words = 8 + set->capacity * 16 * 2 + set->capacity * 2;
    ct::Vector<std::uint32_t> data;
    data.resize(words);
    if (!driver->readBuffer(set->output, 0, words * 4, data.data())) return false;
    indexCount = data[0];
    const std::uint32_t cardCount = indexCount / 6;
    const std::uint32_t positionsBase = 8;
    const std::uint32_t normalsBase = positionsBase + set->capacity * 16;
    const std::uint32_t dataBase = normalsBase + set->capacity * 16;
    cards.clear();
    for (std::uint32_t c = 0; c < cardCount && c < set->capacity; ++c)
    {
        ImpostorCardReadback card;
        const std::uint32_t first = data[dataBase + c * 2];
        card.slice = first & 0xFFFFFFu;
        card.dither = static_cast<float>(first >> 24) / 255.0f;
        card.color = data[dataBase + c * 2 + 1];
        for (std::uint32_t v = 0; v < 4; ++v)
            for (std::uint32_t k = 0; k < 3; ++k)
                memcpy(&card.positions[v][k], &data[positionsBase + (c * 4 + v) * 4 + k], 4);
        for (std::uint32_t k = 0; k < 3; ++k)
            memcpy(&card.normal[k], &data[normalsBase + c * 16 + k], 4);
        cards.push_back(card);
    }
    return true;
}

bool ImpostorManager::readSlice(prisma::Driver* driver, ImpostorId id, std::uint32_t angle,
        ImpostorProperty property, ct::Vector<std::uint8_t>& rgba) const
{
    const Entry* entry = get(id);
    if (!entry || !entry->captured || angle >= kImpostorCaptureAngles || !mArray.valid())
        return false;
    prisma::RenderTarget source;
    source.texture = mArray;
    source.layer = impostorSlice(static_cast<std::uint32_t>(entry->slot), angle, property);
    prisma::Rect rect;
    rect.width = kImpostorTextureDim;
    rect.height = kImpostorTextureDim;
    rgba.resize(static_cast<std::size_t>(kImpostorTextureDim) * kImpostorTextureDim * 4);
    return driver->readPixels(source, rect, rgba.data());
}

void ImpostorManager::destroy(prisma::Driver* driver)
{
    for (Set& set: mSets)
    {
        if (set.instances.valid()) driver->destroy(set.instances);
        if (set.output.valid()) driver->destroy(set.output);
        if (set.indices.valid()) driver->destroy(set.indices);
        if (set.cardParams.valid()) driver->destroy(set.cardParams);
    }
    mSets.clear();
    if (mPreparePipeline.valid()) driver->destroy(mPreparePipeline);
    if (mPrepareShader.valid()) driver->destroy(mPrepareShader);
    if (mPrepareParams.valid()) driver->destroy(mPrepareParams);
    mPreparePipeline = prisma::PipelineHandle();
    mPrepareShader = prisma::ShaderHandle();
    mPrepareParams = prisma::BufferHandle();
    destroyCapture(driver);
}

void ImpostorManager::destroyCapture(prisma::Driver* driver)
{
    for (auto& entry: mPipelines)
        if (entry.value.valid()) driver->destroy(entry.value);
    mPipelines.clear();
    for (auto& entry: mCardPipelines)
        if (entry.value.valid()) driver->destroy(entry.value);
    mCardPipelines.clear();
    for (auto& entry: mShaders)
        if (entry.value.valid()) driver->destroy(entry.value);
    mShaders.clear();
    if (mArray.valid()) driver->destroy(mArray);
    for (std::uint32_t i = 0; i < kImpostorProperties; ++i)
    {
        if (mColor[i].valid()) driver->destroy(mColor[i]);
        mColor[i] = prisma::TextureHandle();
    }
    if (mDepth.valid()) driver->destroy(mDepth);
    if (mSampler.valid()) driver->destroy(mSampler);
    if (mCardSampler.valid()) driver->destroy(mCardSampler);
    if (mSceneBuffer.valid()) driver->destroy(mSceneBuffer);
    if (mMaterialBuffer.valid()) driver->destroy(mMaterialBuffer);
    if (mDrawBuffer.valid()) driver->destroy(mDrawBuffer);
    if (mInstanceBuffer.valid()) driver->destroy(mInstanceBuffer);
    mArray = prisma::TextureHandle();
    mDepth = prisma::TextureHandle();
    mSampler = prisma::SamplerHandle();
    mCardSampler = prisma::SamplerHandle();
    mSceneBuffer = prisma::BufferHandle();
    mMaterialBuffer = prisma::BufferHandle();
    mDrawBuffer = prisma::BufferHandle();
    mInstanceBuffer = prisma::BufferHandle();
}

} // namespace prisma3d
