#include "prisma3d/scene/Hair3D.h"
#include "prisma3d/render/ShaderVariants.h"

#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <ct/json.hpp>
#include <ct/threadpool.hpp>

#include <string.h>

namespace prisma3d
{

namespace
{

const char* const kMeshName = "hair";
const std::uint32_t kMaxGpuColliders = 16;
const std::uint32_t kMaxGpuForces = 8;
const std::uint32_t kMaxGpuAtlasRects = 64;
const std::uint32_t kVertexFloats = 12;
const std::uint32_t kFlagRegenerate = 1u;
const std::uint32_t kFlagCameraBend = 2u;
const std::uint32_t kFlagWind = 4u;

struct HairColliderGpu
{
    float positionRadius[4];
    float tipType[4];
    float normal[4];
    float projection[16];
};

struct HairForceGpu
{
    float positionType[4];
    float directionGravity[4];
    float range[4];
};

struct HairParamsGpu
{
    float transform[16];
    float inverseTransform[16];
    float cameraPositionDt[4];
    float cameraUpTime[4];
    std::uint32_t counts[4];
    float dynamics[4];
    float shape[4];
    std::uint32_t flagsCounts[4];
    float windDirection[4];
    float atlasMulAdd[kMaxGpuAtlasRects][4];
    float atlasSizeAspect[kMaxGpuAtlasRects][4];
    HairColliderGpu colliders[kMaxGpuColliders];
    HairForceGpu forces[kMaxGpuForces];
};

static_assert(sizeof(HairParamsGpu) == 4464, "HairParamsGpu must match the std140 block");
static_assert(sizeof(Math::Mat4) == 64, "Mat4 must be four packed columns");

struct ScalpVertexGpu
{
    float positionLength[4];
    float normal[4];
};

struct HairPipelines
{
    prisma::Driver* driver = nullptr;
    prisma::ShaderHandle simulateShader;
    prisma::ShaderHandle windShader;
    prisma::PipelineHandle simulate;
    prisma::PipelineHandle wind;
    prisma::SamplerHandle windSampler;
    int users = 0;
    bool failed = false;

    bool ensure(prisma::Driver* device)
    {
        if (simulate.valid()) return true;
        if (failed) return false;
        failed = true;
        driver = device;
        const prisma::Caps& caps = driver->caps();
        simulateShader = driver->createShader(prisma3d::shaderDesc(hair_simulate_comp, caps));
        windShader = driver->createShader(prisma3d::shaderDesc(hair_wind_comp, caps));

        prisma::ComputePipelineDesc desc;
        desc.shader = simulateShader;
        desc.uniformBlockCount = 1;
        desc.uniformBlocks[0].name = "HairParams";
        desc.uniformBlocks[0].slot = 0;
        desc.textureCount = 1;
        desc.textures[0].name = "wind_texture";
        desc.textures[0].slot = 0;
        desc.storageBufferCount = 4;
        desc.storageBuffers[0].name = "Scalp";
        desc.storageBuffers[0].slot = 0;
        desc.storageBuffers[1].name = "ScalpIndices";
        desc.storageBuffers[1].slot = 1;
        desc.storageBuffers[2].name = "Tails";
        desc.storageBuffers[2].slot = 2;
        desc.storageBuffers[3].name = "Vertices";
        desc.storageBuffers[3].slot = 3;
        desc.debugName = "hair simulate";
        if (simulateShader.valid()) simulate = driver->createComputePipeline(desc);

        prisma::ComputePipelineDesc windDesc;
        windDesc.shader = windShader;
        windDesc.uniformBlockCount = 1;
        windDesc.uniformBlocks[0].name = "Params";
        windDesc.uniformBlocks[0].slot = 0;
        windDesc.storageTextureCount = 1;
        windDesc.storageTextures[0].name = "wind_image";
        windDesc.storageTextures[0].slot = 0;
        windDesc.debugName = "hair wind";
        if (windShader.valid()) wind = driver->createComputePipeline(windDesc);

        prisma::SamplerDesc samplerDesc;
        samplerDesc.minFilter = prisma::Filter::Linear;
        samplerDesc.magFilter = prisma::Filter::Linear;
        samplerDesc.mipFilter = prisma::MipFilter::None;
        samplerDesc.addressU = prisma::AddressMode::MirroredRepeat;
        samplerDesc.addressV = prisma::AddressMode::MirroredRepeat;
        samplerDesc.addressW = prisma::AddressMode::MirroredRepeat;
        samplerDesc.debugName = "hair wind";
        windSampler = driver->createSampler(samplerDesc);

        if (!simulate.valid() || !wind.valid() || !windSampler.valid())
        {
            log_error("hair: could not create the compute pipelines; simulating on the CPU");
            release();
            return false;
        }
        failed = false;
        return true;
    }

    void release()
    {
        if (!driver) return;
        if (simulate.valid()) driver->destroy(simulate);
        if (wind.valid()) driver->destroy(wind);
        if (simulateShader.valid()) driver->destroy(simulateShader);
        if (windShader.valid()) driver->destroy(windShader);
        if (windSampler.valid()) driver->destroy(windSampler);
        simulate = prisma::PipelineHandle();
        wind = prisma::PipelineHandle();
        simulateShader = prisma::ShaderHandle();
        windShader = prisma::ShaderHandle();
        windSampler = prisma::SamplerHandle();
    }
};

HairPipelines& hairPipelines()
{
    static HairPipelines pipelines;
    return pipelines;
}

ct::ThreadPool& hairPool()
{
    static ct::ThreadPool pool;
    return pool;
}

void storeVec(float* out, const Math::Vec3& v, float w)
{
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
    out[3] = w;
}

bool ensureBuffer(prisma::Driver* driver, prisma::BufferHandle& buffer, std::uint32_t& capacity,
        std::uint32_t size, const char* name)
{
    if (buffer.valid() && capacity >= size) return true;
    if (buffer.valid()) driver->destroy(buffer);
    buffer = prisma::BufferHandle();
    capacity = 0;
    prisma::BufferDesc desc;
    desc.usage = prisma::BufferUsage::Storage;
    desc.update = prisma::BufferUpdate::Dynamic;
    desc.size = size;
    desc.debugName = name;
    buffer = driver->createBuffer(desc);
    if (!buffer.valid()) return false;
    capacity = size;
    return true;
}

} // namespace

struct Hair3D::Gpu
{
    prisma::Driver* driver = nullptr;
    prisma::BufferHandle params;
    prisma::BufferHandle windParams;
    prisma::BufferHandle scalp;
    prisma::BufferHandle indices;
    prisma::BufferHandle tails;
    prisma::TextureHandle wind;
    std::uint32_t scalpCapacity = 0;
    std::uint32_t indexCapacity = 0;
    std::uint32_t tailCapacity = 0;
    ct::Vector<ScalpVertexGpu> scalpData;
    bool windWritten = false;

    void release()
    {
        if (!driver) return;
        if (params.valid()) driver->destroy(params);
        if (windParams.valid()) driver->destroy(windParams);
        if (scalp.valid()) driver->destroy(scalp);
        if (indices.valid()) driver->destroy(indices);
        if (tails.valid()) driver->destroy(tails);
        if (wind.valid()) driver->destroy(wind);
        params = prisma::BufferHandle();
        windParams = prisma::BufferHandle();
        scalp = prisma::BufferHandle();
        indices = prisma::BufferHandle();
        tails = prisma::BufferHandle();
        wind = prisma::TextureHandle();
        scalpCapacity = indexCapacity = tailCapacity = 0;
        windWritten = false;
    }
};

Hair3D::Hair3D()
        : VisualInstance3D(),
          mSkeletonPath(".."),
          mWindDirection(0.0f, 0.0f, 0.0f),
          mWindSpeed(1.0f),
          mWindRandomness(5.0f),
          mWindWaveSize(1.0f),
          mTime(0.0f),
          mSurfaceVertices(0),
          mSurfaceIndices(0),
          mSurfaceReady(false),
          mSurfaceStorage(false),
          mScalpDirty(true),
          mScalpIndicesDirty(true),
          mGpuRegenerate(true),
          mSimulation(HairSimulation::Auto),
          mGpu(nullptr)
{
    HairParams params;
    params.strandCount = 1000;
    params.segmentCount = 4;
    params.length = 0.2f;
    params.width = 0.02f;
    params.stiffness = 0.5f;
    params.gravityPower = 1.0f;
    mStrands.setParams(params);
    mStrands.setWindSlicesPerFrame(4);
}

Hair3D::~Hair3D()
{
    disconnectPreDraw();
    releaseGpu();
    if (RenderingServer* server = RenderingServer::singleton())
    {
        server->instanceSetMaterialOverride(getInstance(), MaterialId());
        server->instanceSetBase(getInstance(), MeshId());
    }
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
    if (MaterialStore* materials = MaterialStore::singleton())
        if (mDefaultMaterial.valid()) materials->destroy(mDefaultMaterial);
}

void Hair3D::setScalp(const SurfaceData& surface)
{
    mScalpPositions = surface.positions;
    mScalpNormals = surface.normals;
    mScalpJoints = surface.joints;
    mScalpWeights = surface.weights;
    mScalpIndices = surface.indices;
    if (mScalpNormals.size() != mScalpPositions.size())
    {
        mScalpNormals.resize(mScalpPositions.size());
        for (std::size_t i = 0; i < mScalpNormals.size(); ++i)
            mScalpNormals[i] = Math::Vec3(0.0f, 1.0f, 0.0f);
    }
    mScalpLengths.resize(mScalpPositions.size());
    for (std::size_t i = 0; i < mScalpLengths.size(); ++i)
        mScalpLengths[i] = i < surface.colors.size() ? surface.colors[i].w : 1.0f;
    mSkinnedPositions = mScalpPositions;
    mSkinnedNormals = mScalpNormals;
    mScalpDirty = true;
    mScalpIndicesDirty = true;
    mGpuRegenerate = true;
}

void Hair3D::setScalpLengths(const ct::Vector<float>& lengths)
{
    for (std::size_t i = 0; i < mScalpLengths.size() && i < lengths.size(); ++i)
        mScalpLengths[i] = lengths[i];
    mScalpDirty = true;
    mScalpIndicesDirty = true;
    mGpuRegenerate = true;
}

std::uint32_t Hair3D::getScalpTriangleCount() const
{
    return mStrands.scalpTriangleCount();
}

void Hair3D::setSkeletonPath(const ct::String& path)
{
    mSkeletonPath = path;
}

const ct::String& Hair3D::getSkeletonPath() const
{
    return mSkeletonPath;
}

Skeleton3D* Hair3D::getSkeleton() const
{
    if (mSkeletonPath.empty() || !isInsideTree()) return nullptr;
    Node* node = getNode(mSkeletonPath);
    return node ? node->castTo<Skeleton3D>() : nullptr;
}

void Hair3D::setParams(const HairParams& params)
{
    mStrands.setParams(params);
    mGpuRegenerate = true;
}

const HairParams& Hair3D::getParams() const
{
    return mStrands.params();
}

#define PRISMA3D_HAIR_SETTER(name, field, type)                                                    \
    void Hair3D::name(type value)                                                                  \
    {                                                                                              \
        HairParams params = mStrands.params();                                                     \
        params.field = value;                                                                      \
        setParams(params);                                                                         \
    }

PRISMA3D_HAIR_SETTER(setStrandCount, strandCount, std::uint32_t)
PRISMA3D_HAIR_SETTER(setSegmentCount, segmentCount, std::uint32_t)
PRISMA3D_HAIR_SETTER(setBillboardCount, billboardCount, std::uint32_t)
PRISMA3D_HAIR_SETTER(setRandomSeed, randomSeed, std::uint32_t)
PRISMA3D_HAIR_SETTER(setLength, length, float)
PRISMA3D_HAIR_SETTER(setStiffness, stiffness, float)
PRISMA3D_HAIR_SETTER(setDrag, drag, float)
PRISMA3D_HAIR_SETTER(setGravityPower, gravityPower, float)
PRISMA3D_HAIR_SETTER(setRandomness, randomness, float)
PRISMA3D_HAIR_SETTER(setViewDistance, viewDistance, float)
PRISMA3D_HAIR_SETTER(setWidth, width, float)
PRISMA3D_HAIR_SETTER(setUniformity, uniformity, float)
PRISMA3D_HAIR_SETTER(setCameraBend, cameraBend, bool)

#undef PRISMA3D_HAIR_SETTER

void Hair3D::setAtlasRects(const ct::Vector<HairAtlasRect>& rects)
{
    HairParams params = mStrands.params();
    params.atlasRects = rects;
    setParams(params);
}

void Hair3D::setMaterial(MaterialId material)
{
    mMaterial = material;
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetMaterialOverride(getInstance(),
                mMaterial.valid() ? mMaterial : mDefaultMaterial);
}

MaterialId Hair3D::getMaterial() const
{
    return mMaterial.valid() ? mMaterial : mDefaultMaterial;
}

MaterialId Hair3D::getAssignedMaterial() const
{
    return mMaterial;
}

void Hair3D::setTexture(TextureId texture)
{
    mTexture = texture;
    ensureMaterial();
    MaterialStore* materials = MaterialStore::singleton();
    TextureStore* textures = TextureStore::singleton();
    if (materials && mDefaultMaterial.valid())
    {
        materials->setTexture(mDefaultMaterial, MaterialTexture::Albedo, texture);
        materials->setTransparency(mDefaultMaterial, texture.valid()
                                                             ? MaterialTransparency::AlphaScissor
                                                             : MaterialTransparency::Opaque);
    }
    HairParams params = mStrands.params();
    params.textureAspect = 1.0f;
    if (textures)
        if (const TextureInfo* info = textures->get(texture))
            if (info->width > 0 && info->height > 0)
                params.textureAspect =
                        static_cast<float>(info->width) / static_cast<float>(info->height);
    setParams(params);
}

TextureId Hair3D::getTexture() const
{
    return mTexture;
}

void Hair3D::setWind(const Math::Vec3& direction, float speed, float randomness, float waveSize)
{
    mWindDirection = direction;
    mWindSpeed = speed;
    mWindRandomness = randomness;
    mWindWaveSize = waveSize;
}

void Hair3D::setWindDirection(const Math::Vec3& direction)
{
    mWindDirection = direction;
}

const Math::Vec3& Hair3D::getWindDirection() const
{
    return mWindDirection;
}

void Hair3D::setWindSpeed(float speed)
{
    mWindSpeed = speed;
}

float Hair3D::getWindSpeed() const
{
    return mWindSpeed;
}

void Hair3D::setWindRandomness(float randomness)
{
    mWindRandomness = randomness;
}

float Hair3D::getWindRandomness() const
{
    return mWindRandomness;
}

void Hair3D::setWindWaveSize(float waveSize)
{
    mWindWaveSize = waveSize;
}

float Hair3D::getWindWaveSize() const
{
    return mWindWaveSize;
}

void Hair3D::clearColliders()
{
    mColliders.clear();
}

void Hair3D::addCollider(const HairCollider& collider)
{
    mColliders.push_back(collider);
}

std::uint32_t Hair3D::getColliderCount() const
{
    return static_cast<std::uint32_t>(mColliders.size());
}

void Hair3D::clearForceFields()
{
    mForceFields.clear();
}

void Hair3D::addForceField(const HairForceField& field)
{
    mForceFields.push_back(field);
}

void Hair3D::setSimulation(HairSimulation mode)
{
    if (mSimulation == mode) return;
    mSimulation = mode;
    mGpuRegenerate = true;
    mStrands.reset();
}

HairSimulation Hair3D::getSimulation() const
{
    return mSimulation;
}

bool Hair3D::isSimulatingOnGpu() const
{
    return mGpu != nullptr && mSurfaceStorage;
}

bool Hair3D::wantsGpu() const
{
    if (mSimulation == HairSimulation::Cpu) return false;
    MeshStore* store = MeshStore::singleton();
    if (!store || !store->driver()) return false;
    return store->driver()->caps().compute;
}

void Hair3D::reset()
{
    mStrands.reset();
    mGpuRegenerate = true;
}

const HairStrands& Hair3D::getStrands() const
{
    return mStrands;
}

void Hair3D::ensureMaterial()
{
    if (mDefaultMaterial.valid()) return;
    MaterialStore* materials = MaterialStore::singleton();
    if (!materials) return;
    mDefaultMaterial = materials->create("hair");
    materials->setCull(mDefaultMaterial, MaterialCull::Disabled);
    materials->setFloat(mDefaultMaterial, MaterialParam::AlphaScissorThreshold, 0.5f);
    if (RenderingServer* server = RenderingServer::singleton())
        if (!mMaterial.valid())
            server->instanceSetMaterialOverride(getInstance(), mDefaultMaterial);
}

bool Hair3D::ensureSurface(bool storage)
{
    MeshStore* store = MeshStore::singleton();
    if (!store) return false;
    const std::uint32_t vertices = mStrands.vertexCount();
    const std::uint32_t indices = mStrands.indexCount();
    if (store->valid(mMesh) && mSurfaceReady &&
            (vertices != mSurfaceVertices || indices != mSurfaceIndices ||
                    storage != mSurfaceStorage))
    {
        setBase(MeshId());
        store->destroy(mMesh);
        mMesh = MeshId();
        mSurfaceReady = false;
    }
    if (!store->valid(mMesh))
    {
        mMesh = store->create(kMeshName);
        mSurfaceReady = false;
    }
    if (!mSurfaceReady && vertices > 0)
    {
        SurfaceLayout layout;
        layout.attributeMask = kVertexPosition | kVertexNormal | kVertexUv;
        layout.vertexCount = vertices;
        layout.indexCount = indices;
        layout.indexFormat =
                vertices <= 65536 ? prisma::IndexFormat::UInt16 : prisma::IndexFormat::UInt32;
        layout.storageVertices = storage;
        mSurfaceReady = store->addDynamicSurface(mMesh, layout, kMeshName) >= 0;
        mSurfaceVertices = vertices;
        mSurfaceIndices = indices;
        mSurfaceStorage = storage;
        mGpuRegenerate = true;
        setBase(mMesh);
        applyIdentityTransform();
    }
    return mSurfaceReady;
}

void Hair3D::skinScalp()
{
    Skeleton3D* skeleton = getSkeleton();
    const bool skinned = skeleton && mScalpJoints.size() == mScalpPositions.size() &&
                         mScalpWeights.size() == mScalpPositions.size();
    if (!skinned)
    {
        if (mScalpDirty)
        {
            mSkinnedPositions = mScalpPositions;
            mSkinnedNormals = mScalpNormals;
        }
        return;
    }
    const ct::Vector<Math::Mat4>& palette = skeleton->getBonePalette();
    if (palette.empty()) return;
    const std::size_t bones = palette.size();
    for (std::size_t i = 0; i < mScalpPositions.size(); ++i)
    {
        Math::Vec3 position(0.0f, 0.0f, 0.0f);
        Math::Vec3 normal(0.0f, 0.0f, 0.0f);
        float total = 0.0f;
        for (int c = 0; c < 4; ++c)
        {
            const float weight = (&mScalpWeights[i].x)[c];
            if (weight <= 0.0f) continue;
            std::size_t joint = (mScalpJoints[i] >> (8 * c)) & 0xffu;
            if (joint >= bones) joint = 0;
            position = position + palette[joint].TransformPoint(mScalpPositions[i]) * weight;
            normal = normal + palette[joint].TransformVector(mScalpNormals[i]) * weight;
            total += weight;
        }
        if (total <= 0.0f)
        {
            position = mScalpPositions[i];
            normal = mScalpNormals[i];
        }
        mSkinnedPositions[i] = position;
        mSkinnedNormals[i] = normal.Normalized();
    }
}

void Hair3D::fillFrame(HairFrame& frame, float deltaTime)
{
    frame.transform = getGlobalTransform();
    frame.deltaTime = deltaTime;
    frame.time = mTime;
    frame.windDirection = mWindDirection;
    frame.windSpeed = mWindSpeed;
    frame.windRandomness = mWindRandomness;
    frame.windWaveSize = mWindWaveSize;
    frame.colliders = mColliders.data();
    frame.colliderCount = static_cast<std::uint32_t>(mColliders.size());
    frame.forces = mForceFields.data();
    frame.forceCount = static_cast<std::uint32_t>(mForceFields.size());
    Viewport* viewport = Viewport::findViewport(this);
    Camera3D* camera = viewport ? viewport->getCamera3D() : nullptr;
    if (camera)
    {
        const Math::Mat4 cameraTransform = camera->getGlobalTransform();
        frame.cameraPosition = cameraTransform.GetTranslation();
        frame.cameraUp = cameraTransform.TransformVector(Math::Vec3(0.0f, 1.0f, 0.0f)).Normalized();
    }
}

void Hair3D::update(float deltaTime)
{
    if (mScalpPositions.empty() || mStrands.params().strandCount == 0) return;
    ensureMaterial();
    skinScalp();
    if (mScalpDirty)
    {
        mStrands.setScalp(mSkinnedPositions.data(), mSkinnedNormals.data(), mScalpLengths.data(),
                static_cast<std::uint32_t>(mSkinnedPositions.size()), mScalpIndices.data(),
                static_cast<std::uint32_t>(mScalpIndices.size()));
        mScalpDirty = false;
    }
    else
    {
        mStrands.updateScalp(mSkinnedPositions.data(), mSkinnedNormals.data());
    }
    if (mStrands.scalpTriangleCount() == 0) return;

    const bool gpu = wantsGpu();
    if (!gpu) releaseGpu();
    if (!ensureSurface(gpu)) return;

    mTime += deltaTime;
    HairFrame frame;
    fillFrame(frame, deltaTime);
    if (gpu) updateGpu(frame);
    else
        updateCpu(frame);
}

void Hair3D::updateCpu(const HairFrame& frame)
{
    mStrands.beginFrame(frame);
    HairStrands* work = &mStrands;
    std::uint32_t windBegin = 0;
    std::uint32_t windEnd = 0;
    mStrands.windSlicesToCompute(windBegin, windEnd);
    if (windEnd > windBegin)
        hairPool().parallel_for(
                windBegin, windEnd,
                [work](std::size_t slice)
                {
                    work->computeWindSlices(static_cast<std::uint32_t>(slice),
                            static_cast<std::uint32_t>(slice) + 1);
                },
                1);
    const std::uint32_t strands = mStrands.params().strandCount;
    static constexpr std::uint32_t kChunk = 128;
    const std::uint32_t chunks = (strands + kChunk - 1) / kChunk;
    hairPool().parallel_for(0, chunks,
            [work, strands](std::size_t chunk)
            {
                const std::uint32_t begin = static_cast<std::uint32_t>(chunk) * kChunk;
                const std::uint32_t end = begin + kChunk < strands ? begin + kChunk : strands;
                work->simulateRange(begin, end);
            });
    mStrands.endFrame();
    uploadGeometry();
}

void Hair3D::uploadGeometry()
{
    const std::uint32_t vertices = mStrands.vertexCount();
    const std::uint32_t indices = mStrands.indexCount();
    mGeometry.positions.resize(vertices);
    mGeometry.normals.resize(vertices);
    mGeometry.uvs.resize(vertices);
    mGeometry.indices.resize(indices);
    memcpy(mGeometry.positions.data(), mStrands.positions(), sizeof(Math::Vec3) * vertices);
    memcpy(mGeometry.normals.data(), mStrands.normals(), sizeof(Math::Vec3) * vertices);
    memcpy(mGeometry.uvs.data(), mStrands.uvs(), sizeof(Math::Vec2) * vertices);
    memcpy(mGeometry.indices.data(), mStrands.indices(), sizeof(std::uint32_t) * indices);
    if (MeshStore* store = MeshStore::singleton()) store->updateSurface(mMesh, 0, mGeometry);
}

void Hair3D::releaseGpu()
{
    if (!mGpu) return;
    mGpu->release();
    delete mGpu;
    mGpu = nullptr;
    HairPipelines& pipelines = hairPipelines();
    if (--pipelines.users <= 0)
    {
        pipelines.users = 0;
        pipelines.release();
    }
}

void Hair3D::updateGpu(const HairFrame& frame)
{
    MeshStore* store = MeshStore::singleton();
    RenderingServer* server = RenderingServer::singleton();
    if (!store || !server) return;
    prisma::Driver* driver = store->driver();
    HairPipelines& pipelines = hairPipelines();
    if (!mGpu)
    {
        if (!pipelines.ensure(driver))
        {
            mSimulation = HairSimulation::Cpu;
            return;
        }
        ++pipelines.users;
        mGpu = new Gpu();
        mGpu->driver = driver;
        mGpuRegenerate = true;
    }
    Gpu& gpu = *mGpu;

    if (mGpuRegenerate)
    {
        mStrands.simulate(frame);
        uploadGeometry();
    }

    const HairParams& params = mStrands.params();
    const std::uint32_t strands = params.strandCount;
    const std::uint32_t scalpVertices = static_cast<std::uint32_t>(mSkinnedPositions.size());
    const std::uint32_t indexCount = mStrands.scalpTriangleCount() * 3;
    const std::uint32_t particles = strands * params.segmentCount;
    const std::uint32_t scalpBytes =
            scalpVertices * static_cast<std::uint32_t>(sizeof(ScalpVertexGpu));
    const std::uint32_t indexBytes = indexCount * static_cast<std::uint32_t>(sizeof(std::uint32_t));

    if (!ensureBuffer(driver, gpu.scalp, gpu.scalpCapacity, scalpBytes, "hair scalp") ||
            !ensureBuffer(driver, gpu.indices, gpu.indexCapacity, indexBytes,
                    "hair scalp indices") ||
            !ensureBuffer(driver, gpu.tails, gpu.tailCapacity, particles * 32u, "hair tails"))
    {
        log_error("hair: could not create the GPU buffers; simulating on the CPU");
        mSimulation = HairSimulation::Cpu;
        return;
    }
    if (!gpu.params.valid())
    {
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Uniform;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = sizeof(HairParamsGpu);
        desc.debugName = "hair params";
        gpu.params = driver->createBuffer(desc);
        desc.size = 16;
        desc.debugName = "hair wind params";
        gpu.windParams = driver->createBuffer(desc);
    }
    if (!gpu.wind.valid())
    {
        prisma::TextureDesc desc;
        desc.type = prisma::TextureType::Texture3D;
        desc.format = prisma::TextureFormat::R32F;
        desc.width = HairStrands::kWindTextureSize;
        desc.height = HairStrands::kWindTextureSize;
        desc.depth = HairStrands::kWindTextureSize;
        desc.usage = prisma::kTextureSampled | prisma::kTextureStorage;
        desc.debugName = "hair wind";
        gpu.wind = driver->createTexture(desc);
        gpu.windWritten = false;
    }
    if (!gpu.params.valid() || !gpu.windParams.valid() || !gpu.wind.valid())
    {
        log_error("hair: could not create the GPU resources; simulating on the CPU");
        mSimulation = HairSimulation::Cpu;
        return;
    }

    gpu.scalpData.resize(scalpVertices);
    const float* lengths = mStrands.scalpLengths();
    for (std::uint32_t i = 0; i < scalpVertices; ++i)
    {
        storeVec(gpu.scalpData[i].positionLength, mSkinnedPositions[i], lengths[i]);
        storeVec(gpu.scalpData[i].normal, mSkinnedNormals[i], 0.0f);
    }
    driver->updateBuffer(gpu.scalp, 0, gpu.scalpData.data(), scalpBytes);
    if (mScalpIndicesDirty || mGpuRegenerate)
    {
        driver->updateBuffer(gpu.indices, 0, mStrands.scalpIndices(), indexBytes);
        mScalpIndicesDirty = false;
    }

    const bool windActive = frame.windDirection.x != 0.0f || frame.windDirection.y != 0.0f ||
                            frame.windDirection.z != 0.0f;

    HairParamsGpu block;
    memset(&block, 0, sizeof(block));
    memcpy(block.transform, &frame.transform, sizeof(block.transform));
    const Math::Mat4 inverse = frame.transform.Inverse();
    memcpy(block.inverseTransform, &inverse, sizeof(block.inverseTransform));
    float dt = frame.deltaTime;
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 1.0f / 30.0f) dt = 1.0f / 30.0f;
    storeVec(block.cameraPositionDt, frame.cameraPosition, dt);
    storeVec(block.cameraUpTime, frame.cameraUp, frame.time);
    block.counts[0] = strands;
    block.counts[1] = params.segmentCount;
    block.counts[2] = params.billboardCount;
    block.counts[3] = params.randomSeed;
    block.dynamics[0] = params.length;
    block.dynamics[1] = params.stiffness;
    block.dynamics[2] = params.drag;
    block.dynamics[3] = params.gravityPower;
    block.shape[0] = params.randomness;
    block.shape[1] = params.viewDistance;
    block.shape[2] = params.width * params.textureAspect;
    block.shape[3] = params.uniformity;
    std::uint32_t flags = 0;
    if (mGpuRegenerate) flags |= kFlagRegenerate;
    if (params.cameraBend) flags |= kFlagCameraBend;
    if (windActive) flags |= kFlagWind;
    const std::uint32_t atlasCount = static_cast<std::uint32_t>(
            params.atlasRects.size() < kMaxGpuAtlasRects ? params.atlasRects.size()
                                                         : kMaxGpuAtlasRects);
    const std::uint32_t colliderCount =
            frame.colliderCount < kMaxGpuColliders ? frame.colliderCount : kMaxGpuColliders;
    const std::uint32_t forceCount =
            frame.forceCount < kMaxGpuForces ? frame.forceCount : kMaxGpuForces;
    block.flagsCounts[0] = flags;
    block.flagsCounts[1] = atlasCount;
    block.flagsCounts[2] = colliderCount;
    block.flagsCounts[3] = forceCount;
    storeVec(block.windDirection, frame.windDirection, 0.0f);
    for (std::uint32_t i = 0; i < atlasCount; ++i)
    {
        const HairAtlasRect& rect = params.atlasRects[i];
        block.atlasMulAdd[i][0] = rect.texMulAdd.x;
        block.atlasMulAdd[i][1] = rect.texMulAdd.y;
        block.atlasMulAdd[i][2] = rect.texMulAdd.z;
        block.atlasMulAdd[i][3] = rect.texMulAdd.w;
        block.atlasSizeAspect[i][0] = rect.size;
        block.atlasSizeAspect[i][1] = rect.texMulAdd.x / rect.texMulAdd.y;
    }
    for (std::uint32_t i = 0; i < colliderCount; ++i)
    {
        const HairCollider& collider = frame.colliders[i];
        HairColliderGpu& out = block.colliders[i];
        storeVec(out.positionRadius, collider.position, collider.radius);
        storeVec(out.tipType, collider.tip, static_cast<float>(collider.type));
        storeVec(out.normal, collider.normal, 0.0f);
        memcpy(out.projection, &collider.planeProjection, sizeof(out.projection));
    }
    for (std::uint32_t i = 0; i < forceCount; ++i)
    {
        const HairForceField& field = frame.forces[i];
        HairForceGpu& out = block.forces[i];
        storeVec(out.positionType, field.position, static_cast<float>(field.type));
        storeVec(out.directionGravity, field.direction, field.gravity);
        out.range[0] = field.range;
    }
    driver->updateBuffer(gpu.params, 0, &block, sizeof(block));

    const MeshInfo* info = store->get(mMesh);
    if (!info || info->surfaces.empty()) return;
    const prisma::BufferHandle vertexBuffer = info->surfaces[0].vertexBuffer;

    server->openFrame();
    driver->beginComputePass();
    if (windActive || !gpu.windWritten)
    {
        const float windBlock[4] = { frame.time, frame.windSpeed, frame.windRandomness,
            frame.windWaveSize };
        driver->updateBuffer(gpu.windParams, 0, windBlock, sizeof(windBlock));
        driver->bindPipeline(pipelines.wind);
        driver->bindUniformBuffer(0, gpu.windParams, 0, sizeof(windBlock));
        driver->bindStorageTexture(0, gpu.wind, 0, prisma::StorageAccess::Write);
        const std::uint32_t groups = HairStrands::kWindTextureSize / 8;
        driver->dispatch(groups, groups, groups);
        gpu.windWritten = true;
    }
    driver->bindPipeline(pipelines.simulate);
    driver->bindUniformBuffer(0, gpu.params, 0, sizeof(HairParamsGpu));
    driver->bindTexture(0, gpu.wind, pipelines.windSampler);
    driver->bindStorageBuffer(0, gpu.scalp, 0, scalpBytes);
    driver->bindStorageBuffer(1, gpu.indices, 0, indexBytes);
    driver->bindStorageBuffer(2, gpu.tails, 0, particles * 32u);
    driver->bindStorageBuffer(3, vertexBuffer, 0,
            mStrands.vertexCount() * kVertexFloats * static_cast<std::uint32_t>(sizeof(float)));
    driver->dispatch((strands + 63) / 64, 1, 1);
    driver->endComputePass();
    mGpuRegenerate = false;

    Math::Box scalpBounds;
    scalpBounds.min = mSkinnedPositions[0];
    scalpBounds.max = mSkinnedPositions[0];
    for (std::uint32_t i = 1; i < scalpVertices; ++i)
    {
        scalpBounds.min = Math::Vec3::Min(scalpBounds.min, mSkinnedPositions[i]);
        scalpBounds.max = Math::Vec3::Max(scalpBounds.max, mSkinnedPositions[i]);
    }
    store->setDynamicSurfaceRange(mMesh, 0, mStrands.vertexCount(), mStrands.indexCount(),
            HairStrands::expandBounds(scalpBounds, params, frame.transform));
}

Math::Box Hair3D::getAabb() const
{
    MeshStore* meshes = MeshStore::singleton();
    if (!meshes || !meshes->valid(mMesh)) return Math::Box();
    const MeshInfo* info = meshes->get(mMesh);
    if (!info || info->surfaces.empty()) return Math::Box();
    return info->surfaces[0].aabb;
}

void Hair3D::applyIdentityTransform()
{
    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetTransform(getInstance(), Math::Mat4::Identity());
}

void Hair3D::notificationHandler(int what)
{
    VisualInstance3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            connectPreDraw();
            break;
        case kNotificationExitTree:
            disconnectPreDraw();
            break;
        case kNotificationEnterWorld:
        case kNotificationTransformChanged:
        case kNotificationVisibilityChanged:
            applyIdentityTransform();
            break;
        default:
            break;
    }
}

void Hair3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    server->framePreDrawConnect(this,
            [this]()
            {
                SceneTree* tree = getTree();
                update(tree ? static_cast<float>(tree->getProcessDeltaTime()) : 0.0f);
            });
}

void Hair3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->framePreDrawDisconnect(this);
}

void Hair3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Hair3D;
    using H = HairParams;
    auto params = &P::getParams;
    auto setParams = &P::setParams;
    auto real = [](const char* name, double minimum, double maximum, double step)
    { return PropertyInfo::range(PropertyType::Float, name, minimum, maximum, step, true); };

    DB::addProperty<P>(PropertyInfo::nodePath("skeleton", "Skeleton3D"), &P::setSkeletonPath,
            &P::getSkeletonPath);
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("material", "Material"),
            &P::setMaterial, &P::getAssignedMaterial);
    resourceprop::add<P, TextureId>(PropertyInfo::resource("texture", "Texture2D"), &P::setTexture,
            &P::getTexture);
    DB::addProperty<P>(PropertyInfo::enumeration("simulation", "Auto,Cpu,Gpu"), &P::setSimulation,
            &P::getSimulation);

    DB::addPropertyGroup<P>("Strands", "");
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Int, "strand_count", 0.0, 1000000.0, 1.0, true),
            params, setParams, &H::strandCount);
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Int, "segment_count", 1.0, 64.0, 1.0, true), params,
            setParams, &H::segmentCount);
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Int, "billboard_count", 1.0, 64.0, 1.0, true), params,
            setParams, &H::billboardCount);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Int, "random_seed"), params, setParams,
            &H::randomSeed);
    DB::addFieldProperty<P>(real("length", 0.0, 100.0, 0.001), params, setParams, &H::length);
    DB::addFieldProperty<P>(real("width", 0.0, 100.0, 0.001), params, setParams, &H::width);
    DB::addFieldProperty<P>(real("randomness", 0.0, 1.0, 0.001), params, setParams, &H::randomness);
    DB::addFieldProperty<P>(real("uniformity", 0.0, 1.0, 0.001), params, setParams, &H::uniformity);
    DB::addFieldProperty<P>(real("view_distance", 0.0, 100000.0, 0.1), params, setParams,
            &H::viewDistance);
    DB::addFieldProperty<P>(PropertyInfo(PropertyType::Bool, "camera_bend"), params, setParams,
            &H::cameraBend);

    DB::addPropertyGroup<P>("Simulation", "");
    DB::addFieldProperty<P>(real("stiffness", 0.0, 1.0, 0.001), params, setParams, &H::stiffness);
    DB::addFieldProperty<P>(real("drag", 0.0, 1.0, 0.001), params, setParams, &H::drag);
    DB::addFieldProperty<P>(real("gravity_power", 0.0, 100.0, 0.001), params, setParams,
            &H::gravityPower);

    DB::addPropertyGroup<P>("Wind", "wind_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "wind_direction"), &P::setWindDirection,
            &P::getWindDirection);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "wind_speed", 0.0, 100.0, 0.01, true, true),
            &P::setWindSpeed, &P::getWindSpeed);
    DB::addProperty<P>(real("wind_randomness", 0.0, 100.0, 0.01), &P::setWindRandomness,
            &P::getWindRandomness);
    DB::addProperty<P>(real("wind_wave_size", 0.0, 100.0, 0.01), &P::setWindWaveSize,
            &P::getWindWaveSize);
}

namespace
{

constexpr std::int64_t kHairDataVersion = 1;

}

bool Hair3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    const HairParams& params = mStrands.params();
    if (mScalpPositions.empty() && params.atlasRects.empty() && mColliders.empty() &&
            mForceFields.empty())
        return false;

    data["version"] = ct::Json(static_cast<long long>(kHairDataVersion));
    if (!mScalpPositions.empty())
    {
        SurfaceData scalp;
        scalp.positions = mScalpPositions;
        scalp.normals = mScalpNormals;
        scalp.joints = mScalpJoints;
        scalp.weights = mScalpWeights;
        scalp.indices = mScalpIndices;
        data["scalp"] = scenejson::fromSurface(scalp);
        data["scalp_lengths"] =
                scenejson::fromFloatBlock(mScalpLengths.data(), mScalpLengths.size());
    }
    if (!params.atlasRects.empty())
    {
        ct::Json rects = ct::Json::array();
        for (std::size_t i = 0; i < params.atlasRects.size(); ++i)
        {
            ct::Json rect = ct::Json::object();
            rect["tex_mul_add"] = scenejson::fromVec4(params.atlasRects[i].texMulAdd);
            rect["size"] = scenejson::fromFloat(params.atlasRects[i].size);
            rects.push_back(rect);
        }
        data["atlas_rects"] = rects;
    }
    if (!mColliders.empty())
    {
        ct::Json colliders = ct::Json::array();
        for (std::size_t i = 0; i < mColliders.size(); ++i)
        {
            const HairCollider& collider = mColliders[i];
            ct::Json entry = ct::Json::object();
            entry["type"] = ct::Json(static_cast<long long>(collider.type));
            entry["position"] = scenejson::fromVec3(collider.position);
            entry["tip"] = scenejson::fromVec3(collider.tip);
            entry["normal"] = scenejson::fromVec3(collider.normal);
            entry["radius"] = scenejson::fromFloat(collider.radius);
            entry["plane_projection"] = scenejson::fromMat4(collider.planeProjection);
            colliders.push_back(entry);
        }
        data["colliders"] = colliders;
    }
    if (!mForceFields.empty())
    {
        ct::Json fields = ct::Json::array();
        for (std::size_t i = 0; i < mForceFields.size(); ++i)
        {
            const HairForceField& field = mForceFields[i];
            ct::Json entry = ct::Json::object();
            entry["type"] = ct::Json(static_cast<long long>(field.type));
            entry["position"] = scenejson::fromVec3(field.position);
            entry["direction"] = scenejson::fromVec3(field.direction);
            entry["gravity"] = scenejson::fromFloat(field.gravity);
            entry["range"] = scenejson::fromFloat(field.range);
            fields.push_back(entry);
        }
        data["force_fields"] = fields;
    }
    return true;
}

bool Hair3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kHairDataVersion, result, path)) return false;

    if (const ct::Json* scalp = data.find("scalp"))
    {
        SurfaceData surface;
        if (!scenejson::toSurface(*scalp, surface, result, scenejson::field(path, "scalp")))
            return false;
        setScalp(surface);
        if (const ct::Json* lengths = data.find("scalp_lengths"))
        {
            ct::Vector<float> values;
            if (!scenejson::toFloatBlock(*lengths, values) ||
                    values.size() != surface.positions.size())
                result.addError(scenejson::field(path, "scalp_lengths"),
                        "expected one length for each scalp vertex");
            else
                setScalpLengths(values);
        }
    }
    if (const ct::Json* rects = scenejson::arrayField(data, "atlas_rects", result, path))
    {
        ct::Vector<HairAtlasRect> list;
        for (std::size_t i = 0; i < rects->size(); ++i)
        {
            const ct::Json& entry = rects->at(i);
            const ct::String entryPath = scenejson::element(path, "atlas_rects", i);
            HairAtlasRect rect;
            if (!entry.is_object() || !scenejson::readVec4Field(entry, "tex_mul_add",
                                              rect.texMulAdd, result, entryPath))
            {
                result.addError(entryPath, "needs tex_mul_add and size");
                continue;
            }
            rect.size = scenejson::getFloat(entry, "size", 1.0f);
            list.push_back(rect);
        }
        setAtlasRects(list);
    }
    clearColliders();
    if (const ct::Json* colliders = scenejson::arrayField(data, "colliders", result, path))
    {
        for (std::size_t i = 0; i < colliders->size(); ++i)
        {
            const ct::Json& entry = colliders->at(i);
            const ct::String entryPath = scenejson::element(path, "colliders", i);
            const std::int64_t type = entry.is_object() ? scenejson::getInt(entry, "type", -1) : -1;
            if (type < 0 || type > static_cast<std::int64_t>(HairColliderType::Plane))
            {
                result.addError(scenejson::field(entryPath, "type"), "not a known collider type");
                continue;
            }
            HairCollider collider;
            collider.type = static_cast<HairColliderType>(type);
            scenejson::readVec3Field(entry, "position", collider.position, result, entryPath);
            scenejson::readVec3Field(entry, "tip", collider.tip, result, entryPath);
            scenejson::readVec3Field(entry, "normal", collider.normal, result, entryPath);
            collider.radius = scenejson::getFloat(entry, "radius", 1.0f);
            if (const ct::Json* projection = entry.find("plane_projection"))
                if (!scenejson::toMat4(*projection, collider.planeProjection))
                    result.addError(scenejson::field(entryPath, "plane_projection"),
                            "expected 16 numbers");
            addCollider(collider);
        }
    }
    clearForceFields();
    if (const ct::Json* fields = scenejson::arrayField(data, "force_fields", result, path))
    {
        for (std::size_t i = 0; i < fields->size(); ++i)
        {
            const ct::Json& entry = fields->at(i);
            const ct::String entryPath = scenejson::element(path, "force_fields", i);
            const std::int64_t type = entry.is_object() ? scenejson::getInt(entry, "type", -1) : -1;
            if (type < 0 || type > static_cast<std::int64_t>(HairForceType::Plane))
            {
                result.addError(scenejson::field(entryPath, "type"), "not a known force type");
                continue;
            }
            HairForceField field;
            field.type = static_cast<HairForceType>(type);
            scenejson::readVec3Field(entry, "position", field.position, result, entryPath);
            scenejson::readVec3Field(entry, "direction", field.direction, result, entryPath);
            field.gravity = scenejson::getFloat(entry, "gravity", 0.0f);
            field.range = scenejson::getFloat(entry, "range", 1.0f);
            addForceField(field);
        }
    }
    return true;
}

} // namespace prisma3d
