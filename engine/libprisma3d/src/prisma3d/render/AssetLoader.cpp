#include "prisma3d/render/AssetLoader.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/render/AnimatedTexture.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"
#include "prisma3d/scene/Collision.h"

#include "platform.h"

#include <stdio.h>
#include <string.h>

namespace prisma3d
{

struct ModelCollisionState
{
    struct Part
    {
        ct::Vector<Math::Vec3> positions;
        ct::Vector<std::uint32_t> indices;

        std::uint64_t bytes() const
        {
            return static_cast<std::uint64_t>(positions.size()) * sizeof(Math::Vec3) +
                   static_cast<std::uint64_t>(indices.size()) * sizeof(std::uint32_t);
        }
    };

    struct MeshBuild
    {
        ct::Vector<Part> parts;
        std::uint32_t remaining = 0;
        bool failed = false;
    };

    ct::Vector<MeshBuild> meshes;
};

ModelInfo::~ModelInfo()
{
    delete collision;
}

namespace
{

typedef ct::SlotMap32<ModelInfo*>::handle_type ModelKey;

const std::uint64_t kMinimumSlice = 64u << 10;

enum class BandResult
{
    Failed,
    More,
    Done
};

bool endsWithNoCase(const ct::String& text, const char* suffix)
{
    const size_t length = text.size();
    const size_t tail = strlen(suffix);
    if (tail > length) return false;
    for (size_t i = 0; i < tail; ++i)
    {
        char a = text[length - tail + i];
        char b = suffix[i];
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a + 32);
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b + 32);
        if (a != b) return false;
    }
    return true;
}

ct::String makeKey(const ct::String& path, bool srgb, bool generateMips)
{
    ct::String key = path;
    key += srgb ? "|s" : "|l";
    key += generateMips ? "|m" : "|f";
    return key;
}

ct::String makeImageName(const ct::String& path, std::size_t image)
{
    ct::String name = path;
    name += "::image";
    char number[24];
    snprintf(number, sizeof(number), "%zu", image);
    name += number;
    return name;
}

TextureId createPendingTexture(TextureStore& textures, bool srgb, const ct::String& path)
{
    const std::uint8_t magenta[4] = { 255, 0, 255, 255 };
    prisma::TextureDesc desc;
    desc.format = srgb ? prisma::TextureFormat::RGBA8Srgb : prisma::TextureFormat::RGBA8;
    desc.width = 1;
    desc.height = 1;
    desc.mipLevels = 1;
    desc.data = magenta;
    desc.debugName = "asset.pending";
    const TextureId id = textures.create(desc, srgb);
    if (!id.valid()) return TextureId();
    textures.setPath(id, path);
    textures.setState(id, TextureState::Pending);
    return id;
}

void scaleModelData(ModelData& document, float scale)
{
    for (std::size_t i = 0; i < document.nodes.size(); ++i)
        document.nodes[i].translation = document.nodes[i].translation * scale;
    for (std::size_t m = 0; m < document.meshes.size(); ++m)
    {
        ModelMeshData& mesh = document.meshes[m];
        for (std::size_t p = 0; p < mesh.primitives.size(); ++p)
        {
            ModelPrimitiveData& primitive = mesh.primitives[p];
            if (!primitive.hasAabb) continue;
            primitive.aabb.min = primitive.aabb.min * scale;
            primitive.aabb.max = primitive.aabb.max * scale;
        }
        for (std::size_t s = 0; s < mesh.shadowSurfaces.size(); ++s)
        {
            ct::Vector<Math::Vec3>& positions = mesh.shadowSurfaces[s].positions;
            for (std::size_t i = 0; i < positions.size(); ++i) positions[i] = positions[i] * scale;
        }
    }
    for (std::size_t s = 0; s < document.skeletons.size(); ++s)
        document.skeletons[s].scalePositions(scale);
    for (std::size_t a = 0; a < document.animations.size(); ++a)
    {
        AnimationClip& clip = document.animations[a];
        for (std::size_t t = 0; t < clip.tracks().size(); ++t)
        {
            ct::Vector<Math::Vec3>& positions = clip.tracks()[t].positions;
            for (std::size_t i = 0; i < positions.size(); ++i) positions[i] = positions[i] * scale;
        }
        for (std::size_t t = 0; t < clip.nodeTracks().size(); ++t)
        {
            ct::Vector<Math::Vec3>& positions = clip.nodeTracks()[t].positions;
            for (std::size_t i = 0; i < positions.size(); ++i) positions[i] = positions[i] * scale;
        }
    }
}

void requestTangents(ModelData& document)
{
    const std::uint32_t needed = kVertexNormal | kVertexUv;
    for (std::size_t m = 0; m < document.meshes.size(); ++m)
    {
        ct::Vector<ModelPrimitiveData>& primitives = document.meshes[m].primitives;
        for (std::size_t p = 0; p < primitives.size(); ++p)
        {
            ModelPrimitiveData& primitive = primitives[p];
            if (primitive.topology == prisma::Topology::Triangles &&
                    (primitive.attributeMask & needed) == needed)
                primitive.attributeMask |= kVertexTangent;
        }
    }
}

const std::uint32_t kSplitMaxCells = 32;

std::uint64_t primitiveBytes(const ModelPrimitiveData& primitive)
{
    if (primitive.vertexCount == 0) return 0;
    const std::uint32_t mask =
            primitive.attributeMask | kVertexPosition | kVertexNormal | kVertexColor;
    const std::uint64_t vertexCount = primitive.vertexCount;
    return vertexCount * MeshStore::vertexStride(mask) +
           static_cast<std::uint64_t>(primitive.indexCount) * (vertexCount <= 65536 ? 2u : 4u);
}

std::uint64_t unpackedBytes(const ModelPrimitiveData& primitive)
{
    if (primitive.vertexCount == 0) return 0;
    const std::uint32_t mask = primitive.attributeMask;
    std::uint64_t perVertex = 12;
    if (mask & kVertexNormal) perVertex += 12;
    if (mask & kVertexTangent) perVertex += 16;
    if (mask & kVertexColor) perVertex += 16;
    if (mask & kVertexUv) perVertex += 8;
    if (mask & kVertexUv2) perVertex += 8;
    if (mask & kVertexJoints) perVertex += 4;
    if (mask & kVertexWeights) perVertex += 16;
    return static_cast<std::uint64_t>(primitive.vertexCount) * perVertex +
           static_cast<std::uint64_t>(primitive.indexCount) * 4;
}

BandResult uploadBands(TextureStore& textures, const Pixmap& pixmap, TextureId& target,
        std::uint32_t& rowsUploaded, bool srgb, bool generateMips, const char* name,
        std::uint64_t& budget, std::uint64_t& bytesUploaded)
{
    bytesUploaded = 0;
    const std::uint32_t width = static_cast<std::uint32_t>(pixmap.width);
    const std::uint32_t height = static_cast<std::uint32_t>(pixmap.height);
    if (!target.valid())
    {
        target = textures.createEmpty(width, height, srgb, generateMips, name);
        if (!target.valid()) return BandResult::Failed;
    }
    const std::uint64_t rowBytes = static_cast<std::uint64_t>(width) * 4;
    const std::uint32_t remaining = height - rowsUploaded;
    std::uint64_t rows = budget / rowBytes;
    if (rows < 1) rows = 1;
    if (rows > remaining) rows = remaining;
    if (rows > 0)
    {
        const std::uint8_t* source =
                pixmap.pixels + static_cast<std::uint64_t>(rowsUploaded) * rowBytes;
        if (!textures.uploadRows(target, rowsUploaded, static_cast<std::uint32_t>(rows), source))
            return BandResult::Failed;
        rowsUploaded += static_cast<std::uint32_t>(rows);
        bytesUploaded = rows * rowBytes;
        budget -= bytesUploaded < budget ? bytesUploaded : budget;
    }
    if (rowsUploaded < height) return BandResult::More;
    if (!textures.finish(target)) return BandResult::Failed;
    return BandResult::Done;
}

} // namespace

struct AssetLoader::Job
{
    enum Kind
    {
        Texture,
        Parse,
        Primitive,
        Image,
        Collision
    };

    explicit Job(Kind jobKind)
            : kind(jobKind),
              info(nullptr),
              ok(false),
              bytes(0),
              uploaded(0)
    {
    }
    virtual ~Job()
    {
    }

    Kind kind;
    ModelId model;
    ModelInfo* info;
    bool ok;
    std::uint64_t bytes;
    std::uint64_t uploaded;
};

struct AssetLoader::TextureJob : AssetLoader::Job
{
    TextureJob()
            : Job(Texture),
              srgb(false),
              generateMips(true),
              dds(false),
              pixmap(nullptr),
              uploadBytes(0),
              rowsUploaded(0)
    {
    }
    ~TextureJob() override
    {
        delete pixmap;
    }

    ct::String path;
    ct::String key;
    bool srgb;
    bool generateMips;
    bool dds;
    ct::Vector<std::uint8_t> data;
    Pixmap* pixmap;
    std::uint64_t uploadBytes;
    TextureId target;
    std::uint32_t rowsUploaded;
};

struct AssetLoader::ModelParseJob : AssetLoader::Job
{
    ModelParseJob()
            : Job(Parse)
    {
    }

    ct::String path;
    float scale = 1.0f;
    ModelData document;
    ModelImporter* importer = nullptr;
};

struct AssetLoader::PrimitiveJob : AssetLoader::Job
{
    PrimitiveJob()
            : Job(Primitive),
              document(nullptr),
              mesh(0),
              primitive(0)
    {
    }

    struct Lod
    {
        float edgeLength = 0.0f;
        std::uint32_t indexCount = 0;
        ct::Vector<std::uint8_t> indices;
    };

    struct Piece
    {
        SurfaceLayout layout;
        ct::Vector<std::uint8_t> vertices;
        ct::Vector<std::uint8_t> indices;
        ct::Vector<Lod> lods;
        std::size_t lodsUploaded = 0;
        std::uint64_t uploaded = 0;
        int surface = -1;
        SurfaceData source;
    };

    std::uint64_t lodBytes() const
    {
        std::uint64_t bytes = 0;
        for (std::size_t i = 0; i < pieces.size(); ++i)
            for (std::size_t l = 0; l < pieces[i].lods.size(); ++l)
                bytes += pieces[i].lods[l].indices.size();
        return bytes;
    }

    const ModelData* document;
    ModelImporter* importer = nullptr;
    std::uint32_t mesh;
    std::uint32_t primitive;
    ct::Vector<Piece> pieces;
    std::size_t current = 0;
    float splitCellSize = 0.0f;
    std::uint32_t splitMinTriangles = 0;
    bool generateLods = false;
    float lodNormalMergeAngle = 20.0f;
    double lodSeconds = 0.0;
    bool collision = false;
    float scale = 1.0f;
    ModelCollisionState::Part part;
};

struct AssetLoader::CollisionJob : AssetLoader::Job
{
    CollisionJob()
            : Job(Collision),
              mesh(0),
              result(nullptr)
    {
    }
    ~CollisionJob() override
    {
        delete result;
    }

    std::uint32_t mesh;
    ct::Vector<ModelCollisionState::Part> parts;
    CollisionMesh* result;
};

struct AssetLoader::ImageJob : AssetLoader::Job
{
    ImageJob()
            : Job(Image),
              document(nullptr),
              image(0),
              srgb(false),
              pixmap(nullptr),
              rowsUploaded(0)
    {
    }
    ~ImageJob() override
    {
        delete pixmap;
    }

    const ModelData* document;
    ModelImporter* importer = nullptr;
    std::uint32_t image;
    bool srgb;
    ct::String key;
    ct::String name;
    Pixmap* pixmap;
    TextureId placeholder;
    TextureId target;
    std::uint32_t rowsUploaded;
};

AssetLoader* AssetLoader::sSingleton = nullptr;

AssetLoader::AssetLoader(TextureStore& textures, MaterialStore& materials, MeshStore& meshes,
        ModelLoader& importers, unsigned threads)
        : mTextures(textures),
          mMaterials(materials),
          mMeshes(meshes),
          mImporters(importers),
          mPool(threads),
          mPendingBytes(0),
          mPendingByteLimit(kDefaultPendingByteLimit),
          mStopping(false),
          mActiveJobs(0),
          mTotalRequests(0),
          mCompletedRequests(0),
          mUploadedBytes(0)
{
    sSingleton = this;
}

AssetLoader::~AssetLoader()
{
    {
        ct::LockGuard guard(mPendingMutex);
        mStopping = true;
    }
    mPendingCv.notify_all();

    mDoneMutex.lock();
    for (;;)
    {
        while (!mDone.empty())
        {
            Job* job = mDone.back();
            mDone.pop_back();
            mDoneMutex.unlock();
            discard(job);
            mDoneMutex.lock();
        }
        if (mActiveJobs.load() == 0) break;
        mDoneCv.wait(mDoneMutex);
    }
    mDoneMutex.unlock();

    for (std::size_t i = 0; i < mUploads.size(); ++i) discard(mUploads[i]);
    mUploads.clear();
    for (std::size_t i = 0; i < mWaiting.size(); ++i) discard(mWaiting[i]);
    mWaiting.clear();
    for (ModelInfo*& info: mModels) delete info;
    mModels.clear();
    for (auto& entry: mAnimated) delete entry.value;
    mAnimated.clear();
    if (sSingleton == this) sSingleton = nullptr;
}

AssetLoader* AssetLoader::singleton()
{
    return sSingleton;
}

void AssetLoader::submit(Job* job)
{
    mActiveJobs.fetch_add(1);
    mPool.submit([this, job]() { runJob(job); });
}

void AssetLoader::runJob(Job* job)
{
    switch (job->kind)
    {
        case Job::Texture:
            runTextureJob(static_cast<TextureJob*>(job));
            break;
        case Job::Parse:
            runParseJob(static_cast<ModelParseJob*>(job));
            break;
        case Job::Primitive:
            runPrimitiveJob(static_cast<PrimitiveJob*>(job));
            break;
        case Job::Image:
            runImageJob(static_cast<ImageJob*>(job));
            break;
        case Job::Collision:
            runCollisionJob(static_cast<CollisionJob*>(job));
            break;
    }
    finished(job);
}

void AssetLoader::finished(Job* job)
{
    ct::LockGuard guard(mDoneMutex);
    mDone.push_back(job);
    mActiveJobs.fetch_sub(1);
    mDoneCv.notify_all();
}

bool AssetLoader::reserve(std::uint64_t bytes)
{
    ct::LockGuard guard(mPendingMutex);
    while (!mStopping && mPendingBytes != 0 && mPendingBytes + bytes > mPendingByteLimit)
        mPendingCv.wait(mPendingMutex);
    if (mStopping) return false;
    mPendingBytes += bytes;
    return true;
}

bool AssetLoader::reservePrimitive(ModelInfo& info, std::uint32_t mesh, std::uint32_t primitive,
        std::uint64_t bytes)
{
    {
        ct::LockGuard guard(mPendingMutex);
        while (!mStopping &&
                (info.reservedPrimitives[mesh] != primitive ||
                        (mPendingBytes != 0 && mPendingBytes + bytes > mPendingByteLimit)))
            mPendingCv.wait(mPendingMutex);
        if (mStopping) return false;
        mPendingBytes += bytes;
        ++info.reservedPrimitives[mesh];
    }
    mPendingCv.notify_all();
    return true;
}

void AssetLoader::release(std::uint64_t bytes)
{
    {
        ct::LockGuard guard(mPendingMutex);
        mPendingBytes -= bytes < mPendingBytes ? bytes : mPendingBytes;
    }
    mPendingCv.notify_all();
}

void AssetLoader::adjustPending(std::uint64_t from, std::uint64_t to)
{
    {
        ct::LockGuard guard(mPendingMutex);
        mPendingBytes -= from < mPendingBytes ? from : mPendingBytes;
        mPendingBytes += to;
    }
    if (to < from) mPendingCv.notify_all();
}

void AssetLoader::setPendingByteLimit(std::uint64_t bytes)
{
    {
        ct::LockGuard guard(mPendingMutex);
        mPendingByteLimit = bytes;
    }
    mPendingCv.notify_all();
}

std::uint64_t AssetLoader::pendingByteLimit() const
{
    ct::LockGuard guard(mPendingMutex);
    return mPendingByteLimit;
}

std::uint64_t AssetLoader::pendingBytes() const
{
    ct::LockGuard guard(mPendingMutex);
    return mPendingBytes;
}

TextureId AssetLoader::requestTexture(const ct::String& path, bool srgb, bool generateMips)
{
    if (path.empty()) return TextureId();

    const ct::String key = makeKey(path, srgb, generateMips);
    const TextureEntry* cached = mTextureCache.find(key);
    if (cached)
    {
        if (mTextures.valid(cached->texture)) return cached->texture;
        mTextureKeys.erase(cached->texture.bits());
        mTextureCache.erase(key);
    }

    const TextureId id = createPendingTexture(mTextures, srgb, path);
    if (!id.valid()) return TextureId();

    TextureEntry entry;
    entry.texture = id;
    entry.srgb = srgb;
    entry.generateMips = generateMips;
    entry.status = LoadStatus::InProgress;
    mTextureCache[key] = entry;
    mTextureKeys[id.bits()] = key;
    ++mTotalRequests;

    TextureJob* job = new TextureJob();
    job->path = path;
    job->key = key;
    job->srgb = srgb;
    job->generateMips = generateMips;
    submit(job);
    return id;
}

TextureId AssetLoader::requestAnimatedTexture(const ct::Vector<ct::String>& paths, bool srgb,
        float fps, bool generateMips)
{
    if (paths.empty() || paths.size() > static_cast<std::size_t>(AnimatedTexture::kMaxFrames))
        return TextureId();

    ct::String key;
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        key += paths[i];
        key += "\n";
    }
    key += srgb ? "|s" : "|l";
    key += generateMips ? "|m|" : "|f|";
    char rate[32];
    snprintf(rate, sizeof(rate), "%g", static_cast<double>(fps));
    key += rate;

    if (AnimatedTexture** cached = mAnimated.find(key))
        if (mTextures.valid((*cached)->getTexture())) return (*cached)->getTexture();

    ct::Vector<TextureId> frames;
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        const TextureId frame = requestTexture(paths[i], srgb, generateMips);
        if (!frame.valid()) return TextureId();
        frames.push_back(frame);
    }

    AnimatedTexture* animated = new AnimatedTexture(&mTextures);
    if (!mTextures.valid(animated->getTexture()))
    {
        delete animated;
        return TextureId();
    }
    const float duration = fps > 0.0f ? 1.0f / fps : 1.0f;
    animated->setFrames(static_cast<int>(frames.size()));
    for (std::size_t i = 0; i < frames.size(); ++i)
    {
        animated->setFrameTexture(static_cast<int>(i), frames[i]);
        animated->setFrameDuration(static_cast<int>(i), duration);
    }
    animated->update(0.0f);
    mTextures.setSequence(animated->getTexture(), paths, fps);
    if (AnimatedTexture** old = mAnimated.find(key)) delete *old;
    mAnimated[key] = animated;
    return animated->getTexture();
}

AnimatedTexture* AssetLoader::animatedTexture(TextureId texture) const
{
    for (const auto& entry: mAnimated)
        if (entry.value->getTexture() == texture) return entry.value;
    return nullptr;
}

LoadStatus AssetLoader::textureStatus(TextureId texture) const
{
    const ct::String* key = mTextureKeys.find(texture.bits());
    if (!key) return LoadStatus::Invalid;
    const TextureEntry* entry = mTextureCache.find(*key);
    return entry ? entry->status : LoadStatus::Invalid;
}

LoadStatus AssetLoader::status(const ct::String& path, bool srgb) const
{
    const TextureEntry* withMips = mTextureCache.find(makeKey(path, srgb, true));
    if (withMips) return withMips->status;
    const TextureEntry* withoutMips = mTextureCache.find(makeKey(path, srgb, false));
    if (withoutMips) return withoutMips->status;
    return LoadStatus::Invalid;
}

ct::String AssetLoader::modelFile(const ct::String& path)
{
    const std::size_t query = path.find('?');
    return query == ct::String::npos ? path : path.substr(0, query);
}

void AssetLoader::parseModelOptions(const ct::String& path, ModelOptions& options)
{
    const std::size_t query = path.find('?');
    if (query == ct::String::npos) return;
    const char* text = path.c_str() + query + 1;
    while (*text)
    {
        const char* end = text;
        while (*end && *end != '&') ++end;
        const char* equals = text;
        while (equals < end && *equals != '=') ++equals;
        const ct::String key(text, static_cast<std::size_t>(equals - text));
        const ct::String value =
                equals < end ? ct::String(equals + 1, static_cast<std::size_t>(end - equals - 1))
                             : ct::String();
        if (key == "scale")
        {
            const float scale = value.to_float();
            if (scale > 0.0f) options.scale = scale;
            else
                log_warn("AssetLoader: '%s': the scale must be positive", path.c_str());
        }
        else if (key == "physics")
        {
            if (value == "convex") options.physicsShape = 1;
            else if (value == "concave")
                options.physicsShape = 2;
            else if (value == "none")
                options.physicsShape = 0;
            else
                log_warn("AssetLoader: '%s': physics must be convex, concave or none",
                        path.c_str());
        }
        else if (key == "collision")
            options.collisionType = static_cast<int>(value.to_float());
        else if (key == "lightmap")
            options.lightmapEnergy = value.to_float();
        else if (key == "collide")
            options.collidePrefixes = value;
        else if (key == "hide")
            options.hidePrefixes = value;
        else if (key == "body")
        {
            if (value == "kinematic") options.kinematicBody = true;
            else if (value == "static")
                options.kinematicBody = false;
            else
                log_warn("AssetLoader: '%s': body must be static or kinematic", path.c_str());
        }
        else
            log_warn("AssetLoader: '%s': unknown option '%s'", path.c_str(), key.c_str());
        text = *end ? end + 1 : end;
    }
}

ModelId AssetLoader::requestModel(const ct::String& path, const ModelOptions& requested)
{
    if (path.empty()) return ModelId();

    ModelOptions options = requested;
    parseModelOptions(path, options);
    ModelInfo* info = new ModelInfo();
    info->path = path;
    info->collisionType = options.collisionType;
    info->materialSidecar = options.materialSidecar;
    info->scale = options.scale;
    info->physicsShape = options.physicsShape;
    info->kinematicBody = options.kinematicBody;
    info->lightmapEnergy = options.lightmapEnergy;
    info->collidePrefixes = options.collidePrefixes;
    info->hidePrefixes = options.hidePrefixes;
    info->splitCellSize = options.splitCellSize;
    info->splitMinTriangles = options.splitMinTriangles;
    info->generateLods = options.generateLods;
    info->lodNormalMergeAngle = options.lodNormalMergeAngle;
    info->status = LoadStatus::InProgress;
    const ModelKey key = mModels.insert(info);
    const ModelId id = ModelId::from_bits(key.bits());
    ++mTotalRequests;

    ModelParseJob* job = new ModelParseJob();
    job->model = id;
    job->info = info;
    job->path = modelFile(path);
    job->scale = options.scale;
    submit(job);
    return id;
}

ModelId AssetLoader::findModelByPath(const ct::String& path) const
{
    for (std::size_t i = 0; i < mModels.size(); ++i)
        if (mModels.items()[i]->path == path)
            return ModelId::from_bits(mModels.handle_at(i).bits());
    return ModelId();
}

ModelInfo* AssetLoader::findModel(ModelId model) const
{
    ModelInfo* const* entry = mModels.get(ModelKey::from_bits(model.bits()));
    return entry ? *entry : nullptr;
}

LoadStatus AssetLoader::modelStatus(ModelId model) const
{
    const ModelInfo* info = findModel(model);
    return info ? info->status : LoadStatus::Invalid;
}

const ModelInfo* AssetLoader::model(ModelId model) const
{
    return findModel(model);
}

float AssetLoader::modelProgress(ModelId model) const
{
    const ModelInfo* info = findModel(model);
    if (!info) return 0.0f;
    if (info->status == LoadStatus::Loaded) return 1.0f;
    if (info->totalBytes == 0) return 0.0f;
    const double ratio =
            static_cast<double>(info->uploadedBytes) / static_cast<double>(info->totalBytes);
    if (info->status == LoadStatus::Failed) return ratio >= 1.0 ? 1.0f : 0.0f;
    return ratio > 1.0 ? 1.0f : static_cast<float>(ratio);
}

bool AssetLoader::freeModel(ModelId model)
{
    ModelInfo* info = findModel(model);
    if (!info || info->status == LoadStatus::InProgress) return false;
    mModels.erase(ModelKey::from_bits(model.bits()));
    delete info;
    return true;
}

bool AssetLoader::replaceMesh(ModelId model, MeshId from, MeshId to)
{
    ModelInfo* info = findModel(model);
    if (!info) return false;
    bool replaced = false;
    for (std::size_t i = 0; i < info->meshes.size(); ++i)
        if (info->meshes[i] == from)
        {
            info->meshes[i] = to;
            replaced = true;
        }
    return replaced;
}

void AssetLoader::runTextureJob(TextureJob* job)
{
    job->data = FileSystem::getSingleton().readBinary(job->path);
    bool ok = false;
    if (!job->data.empty())
    {
        if (endsWithNoCase(job->path, ".dds"))
        {
            job->dds = true;
            DdsInfo info;
            ok = parseDds(job->data.data(), job->data.size(), job->srgb, &info);
            job->uploadBytes = job->data.size();
        }
        else
        {
            Pixmap* pixmap = new Pixmap();
            job->pixmap = pixmap;
            ok = pixmap->load_from_memory(job->data.data(),
                    static_cast<std::uint32_t>(job->data.size()));
            if (ok && pixmap->components != 4)
            {
                Pixmap* converted = pixmap->convert_to_rgba();
                delete pixmap;
                job->pixmap = converted;
                pixmap = converted;
                ok = converted != nullptr;
            }
            if (ok) job->uploadBytes = static_cast<std::uint64_t>(pixmap->get_size());
            job->data = ct::Vector<std::uint8_t>();
        }
    }
    job->ok = ok;
}

void AssetLoader::runParseJob(ModelParseJob* job)
{
    job->importer = mImporters.importerFor(job->path);
    if (!job->importer)
    {
        const ct::String extension = ModelLoader::extensionOf(job->path);
        if (extension.empty()) job->document.error = "model path has no file extension";
        else
        {
            job->document.error = "no importer registered for '.";
            job->document.error += extension;
            job->document.error += "'";
        }
        job->ok = false;
        return;
    }
    job->ok = job->importer->import(job->path, job->document);
    if (job->ok && job->scale != 1.0f) scaleModelData(job->document, job->scale);
    if (job->ok) requestTangents(job->document);
}

void AssetLoader::runPrimitiveJob(PrimitiveJob* job)
{
    const ModelPrimitiveData& primitive =
            job->document->meshes[job->mesh].primitives[job->primitive];
    const std::uint64_t estimate = primitiveBytes(primitive);
    const std::uint64_t peak = estimate + unpackedBytes(primitive);
    if (!reservePrimitive(*job->info, job->mesh, job->primitive, peak))
    {
        job->ok = false;
        return;
    }
    job->bytes = peak;

    bool ok = false;
    {
        SurfaceData data;
        ok = job->importer->readPrimitive(*job->document, primitive, data);
        if (ok && job->scale != 1.0f)
            for (std::size_t i = 0; i < data.positions.size(); ++i)
                data.positions[i] = data.positions[i] * job->scale;
        if (ok && (primitive.attributeMask & kVertexTangent) && data.tangents.empty())
            MeshStore::generateTangents(data);
        auto buildLods = [job](const SurfaceData& surface, PrimitiveJob::Piece& piece)
        {
            if (!job->generateLods) return;
            const std::uint64_t start = time_nanos();
            ct::Vector<SurfaceLod> lods;
            if (MeshStore::generateLods(surface, job->lodNormalMergeAngle, lods))
            {
                piece.lods.resize(lods.size());
                for (std::size_t l = 0; l < lods.size(); ++l)
                {
                    piece.lods[l].edgeLength = lods[l].edgeLength;
                    piece.lods[l].indexCount = static_cast<std::uint32_t>(lods[l].indices.size());
                    MeshStore::packIndices(lods[l].indices, piece.layout.indexFormat,
                            piece.lods[l].indices);
                }
            }
            job->lodSeconds += static_cast<double>(time_nanos() - start) * 1e-9;
        };
        ct::Vector<SurfaceData> split;
        const std::size_t triangles =
                (data.indices.empty() ? data.positions.size() : data.indices.size()) / 3;
        if (ok && job->splitCellSize > 0.0f && !primitive.skinned &&
                triangles >= job->splitMinTriangles &&
                MeshStore::splitByGrid(data, job->splitCellSize, kSplitMaxCells, split))
        {
            job->pieces.resize(split.size());
            for (std::size_t i = 0; ok && i < split.size(); ++i)
            {
                PrimitiveJob::Piece& piece = job->pieces[i];
                ok = MeshStore::interleave(split[i], piece.layout, piece.vertices, piece.indices);
                if (ok) buildLods(split[i], piece);
                split[i] = SurfaceData();
            }
        }
        else if (ok)
        {
            job->pieces.resize(1);
            PrimitiveJob::Piece& piece = job->pieces[0];
            ok = MeshStore::interleave(data, piece.layout, piece.vertices, piece.indices);
            if (ok) buildLods(data, piece);
            if (ok && primitive.skinned) piece.source = data;
        }
        if (ok && job->collision && data.topology == prisma::Topology::Triangles &&
                !data.positions.empty())
        {
            job->part.positions = static_cast<ct::Vector<Math::Vec3>&&>(data.positions);
            if (data.indices.empty())
            {
                const std::uint32_t count = static_cast<std::uint32_t>(job->part.positions.size());
                job->part.indices.resize(count);
                for (std::uint32_t i = 0; i < count; ++i) job->part.indices[i] = i;
            }
            else
                job->part.indices = static_cast<ct::Vector<std::uint32_t>&&>(data.indices);
        }
    }
    if (ok)
    {
        const std::uint64_t held = estimate + job->part.bytes() + job->lodBytes();
        adjustPending(job->bytes, held);
        job->bytes = held;
    }
    else
    {
        job->pieces = ct::Vector<PrimitiveJob::Piece>();
        release(job->bytes);
        job->bytes = 0;
    }
    job->ok = ok;
}

void AssetLoader::runCollisionJob(CollisionJob* job)
{
    std::uint64_t triangleIndices = 0;
    for (std::size_t i = 0; i < job->parts.size(); ++i)
        triangleIndices += job->parts[i].indices.size();
    if (triangleIndices == 0)
    {
        job->parts = ct::Vector<ModelCollisionState::Part>();
        release(job->bytes);
        job->bytes = 0;
        job->ok = true;
        return;
    }

    adjustPending(job->bytes, job->bytes * 2);
    job->bytes *= 2;

    CollisionMesh* mesh = new CollisionMesh();
    for (std::size_t i = 0; i < job->parts.size(); ++i)
    {
        const ModelCollisionState::Part& part = job->parts[i];
        mesh->append(part.positions.data(), static_cast<std::uint32_t>(part.positions.size()),
                part.indices.data(), static_cast<std::uint32_t>(part.indices.size()));
    }
    job->parts = ct::Vector<ModelCollisionState::Part>();

    const bool ok = mesh->finish() && mesh->triangleCount() == triangleIndices / 3;
    if (!ok)
    {
        delete mesh;
        mesh = nullptr;
    }
    adjustPending(job->bytes, 0);
    job->bytes = 0;
    job->result = mesh;
    job->ok = ok;
}

void AssetLoader::runImageJob(ImageJob* job)
{
    const ModelImageData& image = job->document->images[job->image];
    if (!reserve(image.size))
    {
        job->ok = false;
        return;
    }
    job->bytes = image.size;

    Pixmap* pixmap = nullptr;
    bool ok = false;
    {
        ct::Vector<std::uint8_t> encoded;
        ok = job->importer->readImage(*job->document, image, encoded);
        if (ok)
        {
            pixmap = new Pixmap();
            ok = pixmap->load_from_memory(encoded.data(),
                    static_cast<std::uint32_t>(encoded.size()));
            if (ok && pixmap->components != 4)
            {
                Pixmap* converted = pixmap->convert_to_rgba();
                delete pixmap;
                pixmap = converted;
                ok = converted != nullptr;
            }
        }
    }
    if (!ok)
    {
        delete pixmap;
        pixmap = nullptr;
    }
    const std::uint64_t decoded = ok ? static_cast<std::uint64_t>(pixmap->get_size()) : 0;
    adjustPending(job->bytes, decoded);
    job->bytes = decoded;
    job->pixmap = pixmap;
    job->ok = ok;
}

MaterialId AssetLoader::createMaterial(const ModelMaterialData& data,
        const ct::Vector<TextureId>& textures)
{
    const MaterialId material = mMaterials.create(data.name.c_str());
    mMaterials.setColor(material, MaterialParam::Albedo, data.baseColor);
    mMaterials.setFloat(material, MaterialParam::Metallic, data.metallic);
    mMaterials.setFloat(material, MaterialParam::Roughness, data.roughness);
    mMaterials.setColor(material, MaterialParam::Emission,
            Math::Vec4(data.emissive.x, data.emissive.y, data.emissive.z, 1.0f));
    if (data.alphaMode == ModelAlphaMode::Mask)
    {
        mMaterials.setTransparency(material, MaterialTransparency::AlphaScissor);
        mMaterials.setFloat(material, MaterialParam::AlphaScissorThreshold, data.alphaCutoff);
    }
    else if (data.alphaMode == ModelAlphaMode::Blend)
        mMaterials.setTransparency(material, MaterialTransparency::AlphaBlend);
    if (data.doubleSided) mMaterials.setCull(material, MaterialCull::Disabled);

    auto textureAt = [&](int index)
    {
        if (index < 0 || static_cast<std::size_t>(index) >= textures.size()) return TextureId();
        return textures[static_cast<std::size_t>(index)];
    };
    mMaterials.setTexture(material, MaterialTexture::Albedo, textureAt(data.baseColorTexture));
    const TextureId metallicRoughness = textureAt(data.metallicRoughnessTexture);
    mMaterials.setTexture(material, MaterialTexture::Metallic, metallicRoughness);
    mMaterials.setTextureChannel(material, MaterialTexture::Metallic, MaterialTextureChannel::Blue);
    mMaterials.setTexture(material, MaterialTexture::Roughness, metallicRoughness);
    mMaterials.setTextureChannel(material, MaterialTexture::Roughness,
            MaterialTextureChannel::Green);
    mMaterials.setTexture(material, MaterialTexture::Normal, textureAt(data.normalTexture));
    mMaterials.setTexture(material, MaterialTexture::AmbientOcclusion,
            textureAt(data.occlusionTexture));
    mMaterials.setTextureChannel(material, MaterialTexture::AmbientOcclusion,
            MaterialTextureChannel::Red);
    const TextureId emissive = textureAt(data.emissiveTexture);
    mMaterials.setTexture(material, MaterialTexture::Emission, emissive);
    if (emissive.valid())
        mMaterials.setEmissionOperator(material, MaterialEmissionOperator::Multiply);
    return material;
}

void AssetLoader::finishParse(ModelParseJob* job)
{
    ModelInfo* info = job->info;
    if (!job->ok)
    {
        info->status = LoadStatus::Failed;
        info->error = job->document.error.empty() ? ct::String("could not parse the model")
                                                  : job->document.error;
        log_error("AssetLoader: model '%s' failed to parse: %s", job->path.c_str(),
                info->error.c_str());
        ++mCompletedRequests;
        return;
    }

    info->document = static_cast<ModelData&&>(job->document);
    info->importer = job->importer;
    info->parsed = true;
    const ModelData& document = info->document;

    info->meshes.resize(document.meshes.size());
    info->nextPrimitives.resize(document.meshes.size(), 0);
    info->reservedPrimitives.resize(document.meshes.size(), 0);
    for (std::size_t i = 0; i < document.meshes.size(); ++i)
    {
        info->meshes[i] = mMeshes.create(document.meshes[i].name.c_str());
        ct::String part = document.meshes[i].name;
        bool repeated = false;
        for (std::size_t j = 0; j < i && !repeated; ++j) repeated = document.meshes[j].name == part;
        if (part.empty() || part[0] == '#' || repeated)
        {
            part = "#";
            part.append_number(static_cast<unsigned long long>(i));
        }
        ct::String path = info->path;
        path += "::";
        path += part;
        mMeshes.setPath(info->meshes[i], path);
    }

    ct::Vector<std::uint8_t> colorTexture;
    colorTexture.resize(document.textures.size());
    for (std::size_t i = 0; i < colorTexture.size(); ++i) colorTexture[i] = 0;
    for (std::size_t i = 0; i < document.materials.size(); ++i)
    {
        const int color[2] = { document.materials[i].baseColorTexture,
            document.materials[i].emissiveTexture };
        for (int index: color)
            if (index >= 0 && static_cast<std::size_t>(index) < colorTexture.size())
                colorTexture[static_cast<std::size_t>(index)] = 1;
    }

    ct::Vector<ImageJob*> imageJobs;
    info->textures.resize(document.textures.size());
    for (std::size_t t = 0; t < document.textures.size(); ++t)
    {
        info->textures[t] = TextureId();
        const int imageIndex = document.textures[t].image;
        if (imageIndex < 0 || static_cast<std::size_t>(imageIndex) >= document.images.size())
            continue;
        const ModelImageData& image = document.images[static_cast<std::size_t>(imageIndex)];
        const bool srgb = colorTexture[t] != 0;
        if (!image.embedded())
        {
            info->textures[t] = requestTexture(image.path, srgb, true);
            continue;
        }

        const ct::String name = makeImageName(document.path, static_cast<std::size_t>(imageIndex));
        ct::String key = name;
        key += srgb ? "|s" : "|l";
        const TextureEntry* cached = mTextureCache.find(key);
        if (cached && mTextures.valid(cached->texture))
        {
            info->textures[t] = cached->texture;
            continue;
        }
        const TextureId placeholder = createPendingTexture(mTextures, srgb, name);
        if (!placeholder.valid()) continue;
        TextureEntry entry;
        entry.texture = placeholder;
        entry.srgb = srgb;
        entry.generateMips = true;
        entry.status = LoadStatus::InProgress;
        mTextureCache[key] = entry;
        mTextureKeys[placeholder.bits()] = key;
        info->textures[t] = placeholder;

        ImageJob* imageJob = new ImageJob();
        imageJob->model = job->model;
        imageJob->info = info;
        imageJob->document = &info->document;
        imageJob->importer = info->importer;
        imageJob->image = static_cast<std::uint32_t>(imageIndex);
        imageJob->srgb = srgb;
        imageJob->key = key;
        imageJob->name = name;
        imageJob->placeholder = placeholder;
        imageJobs.push_back(imageJob);
    }

    info->materials.resize(document.materials.size());
    for (std::size_t i = 0; i < document.materials.size(); ++i)
        info->materials[i] = createMaterial(document.materials[i], info->textures);
    if (info->materialSidecar)
    {
        applyModelSidecar(mMaterials, *this, info->path, info->materials,
                &info->unmatchedMaterials);
        for (std::size_t i = 0; i < info->unmatchedMaterials.size(); ++i)
            log_warn("AssetLoader: model '%s': material '%s' is not in the material library",
                    info->path.c_str(), info->unmatchedMaterials[i].c_str());
    }
    for (std::size_t i = 0; i < info->materials.size(); ++i)
        mMaterials.setPath(info->materials[i], info->path);

    for (std::size_t i = 0; i < document.meshes.size(); ++i)
    {
        const ct::Vector<SurfaceData>& shadow = document.meshes[i].shadowSurfaces;
        if (shadow.empty()) continue;
        const MeshId shadowMesh = mMeshes.create(document.meshes[i].name.c_str());
        bool ok = shadowMesh.valid();
        if (ok)
        {
            const MeshInfo* owner = mMeshes.get(info->meshes[i]);
            if (owner) mMeshes.setPath(shadowMesh, owner->path + "#shadow");
        }
        for (std::size_t s = 0; ok && s < shadow.size(); ++s)
            ok = mMeshes.addSurface(shadowMesh, shadow[s]);
        if (ok) ok = mMeshes.setShadowMesh(info->meshes[i], shadowMesh);
        if (!ok)
        {
            log_error("AssetLoader: model '%s': could not create the shadow mesh of mesh %zu",
                    info->path.c_str(), i);
            if (shadowMesh.valid()) mMeshes.destroy(shadowMesh);
        }
    }

    std::uint32_t collisionJobs = 0;
    if (info->collisionType != 0 || info->physicsShape != 0)
    {
        info->collision = new ModelCollisionState();
        info->collision->meshes.resize(document.meshes.size());
    }

    ct::Vector<PrimitiveJob*> primitiveJobs;
    for (std::size_t m = 0; m < document.meshes.size(); ++m)
    {
        const std::size_t primitiveCount = document.meshes[m].primitives.size();
        if (info->collision && primitiveCount != 0)
        {
            ModelCollisionState::MeshBuild& build = info->collision->meshes[m];
            build.parts.resize(primitiveCount);
            build.remaining = static_cast<std::uint32_t>(primitiveCount);
            ++collisionJobs;
        }
        for (std::size_t p = 0; p < primitiveCount; ++p)
        {
            info->totalBytes += primitiveBytes(document.meshes[m].primitives[p]);
            PrimitiveJob* primitiveJob = new PrimitiveJob();
            primitiveJob->model = job->model;
            primitiveJob->info = info;
            primitiveJob->document = &info->document;
            primitiveJob->importer = info->importer;
            primitiveJob->mesh = static_cast<std::uint32_t>(m);
            primitiveJob->primitive = static_cast<std::uint32_t>(p);
            primitiveJob->collision = info->collision != nullptr;
            primitiveJob->scale = info->scale;
            primitiveJob->generateLods = info->generateLods;
            primitiveJob->lodNormalMergeAngle = info->lodNormalMergeAngle;
            if (document.meshes[m].shadowSurfaces.empty())
            {
                primitiveJob->splitCellSize = info->splitCellSize;
                primitiveJob->splitMinTriangles = info->splitMinTriangles;
            }
            primitiveJobs.push_back(primitiveJob);
        }
    }

    info->pendingJobs =
            static_cast<std::uint32_t>(primitiveJobs.size() + imageJobs.size()) + collisionJobs;
    if (info->pendingJobs == 0)
    {
        info->status = LoadStatus::Loaded;
        ++mCompletedRequests;
        return;
    }
    for (std::size_t i = 0; i < primitiveJobs.size(); ++i) submit(primitiveJobs[i]);
    for (std::size_t i = 0; i < imageJobs.size(); ++i) submit(imageJobs[i]);
}

bool AssetLoader::advance(Job* job, std::uint64_t& budget)
{
    switch (job->kind)
    {
        case Job::Texture:
            return advanceTexture(static_cast<TextureJob*>(job), budget);
        case Job::Parse:
            finishParse(static_cast<ModelParseJob*>(job));
            return true;
        case Job::Primitive:
            return advancePrimitive(static_cast<PrimitiveJob*>(job), budget);
        case Job::Image:
            return advanceImage(static_cast<ImageJob*>(job), budget);
        case Job::Collision:
            return true;
    }
    return true;
}

bool AssetLoader::advanceTexture(TextureJob* job, std::uint64_t& budget)
{
    if (!job->ok) return true;
    TextureEntry* entry = mTextureCache.find(job->key);
    if (!entry)
    {
        job->ok = false;
        return true;
    }

    if (job->dds)
    {
        const TextureId loaded = mTextures.createFromDds(job->data.data(), job->data.size(),
                job->srgb, job->generateMips, job->path.c_str());
        job->ok = loaded.valid() && mTextures.replace(entry->texture, loaded);
        if (job->ok) mUploadedBytes += job->uploadBytes;
        budget -= job->uploadBytes < budget ? job->uploadBytes : budget;
        return true;
    }

    std::uint64_t uploadedNow = 0;
    const BandResult result = uploadBands(mTextures, *job->pixmap, job->target, job->rowsUploaded,
            job->srgb, job->generateMips, job->path.c_str(), budget, uploadedNow);
    mUploadedBytes += uploadedNow;
    job->uploaded += uploadedNow;
    if (result == BandResult::More) return false;
    if (result == BandResult::Failed)
    {
        job->ok = false;
        return true;
    }
    job->ok = mTextures.replace(entry->texture, job->target);
    if (job->ok) job->target = TextureId();
    return true;
}

bool AssetLoader::advancePrimitive(PrimitiveJob* job, std::uint64_t& budget)
{
    if (!job->ok) return true;
    ModelInfo* info = job->info;
    const MeshId mesh = info->meshes[job->mesh];
    const ModelPrimitiveData& primitive =
            info->document.meshes[job->mesh].primitives[job->primitive];
    if (job->current == 0 && job->pieces.size() > 1 && job->pieces[0].surface < 0)
    {
        std::uint64_t actual = 0;
        for (std::size_t i = 0; i < job->pieces.size(); ++i)
            actual += job->pieces[i].layout.vertexBytes() + job->pieces[i].layout.indexBytes();
        const std::uint64_t estimate = primitiveBytes(primitive);
        info->totalBytes = (info->totalBytes > estimate ? info->totalBytes - estimate : 0) + actual;
    }

    while (job->current < job->pieces.size())
    {
        PrimitiveJob::Piece& piece = job->pieces[job->current];
        if (piece.surface < 0)
        {
            piece.surface = mMeshes.beginSurface(mesh, piece.layout,
                    info->document.meshes[job->mesh].name.c_str());
            if (piece.surface < 0)
            {
                job->ok = false;
                return true;
            }
            const int material = primitive.material;
            if (material >= 0 && static_cast<std::size_t>(material) < info->materials.size())
                mMeshes.setSurfaceMaterial(mesh, static_cast<std::uint32_t>(piece.surface),
                        info->materials[static_cast<std::size_t>(material)]);
        }

        const std::uint32_t surface = static_cast<std::uint32_t>(piece.surface);
        const std::uint64_t vertexBytes = piece.layout.vertexBytes();
        const std::uint64_t total = vertexBytes + piece.layout.indexBytes();
        std::uint64_t left = total - piece.uploaded;
        std::uint64_t want = budget < left ? budget : left;
        const std::uint64_t floor = left < kMinimumSlice ? left : kMinimumSlice;
        if (want < floor) want = floor;
        const std::uint64_t granted = want;

        while (want > 0)
        {
            bool uploaded = false;
            std::uint64_t size = 0;
            if (piece.uploaded < vertexBytes)
            {
                size = vertexBytes - piece.uploaded;
                if (size > want) size = want;
                uploaded = mMeshes.uploadVertices(mesh, surface,
                        static_cast<std::uint32_t>(piece.uploaded),
                        piece.vertices.data() + piece.uploaded, static_cast<std::uint32_t>(size));
            }
            else
            {
                const std::uint64_t offset = piece.uploaded - vertexBytes;
                size = total - piece.uploaded;
                if (size > want) size = want;
                uploaded = mMeshes.uploadIndices(mesh, surface, static_cast<std::uint32_t>(offset),
                        piece.indices.data() + offset, static_cast<std::uint32_t>(size));
            }
            if (!uploaded)
            {
                job->ok = false;
                return true;
            }
            piece.uploaded += size;
            job->uploaded += size;
            want -= size;
            info->uploadedBytes += size;
            mUploadedBytes += size;
            if (piece.uploaded == total) break;
        }
        budget -= granted < budget ? granted : budget;

        if (piece.uploaded < total) return false;
        while (piece.lodsUploaded < piece.lods.size())
        {
            PrimitiveJob::Lod& lod = piece.lods[piece.lodsUploaded];
            if (!mMeshes.addSurfaceLod(mesh, surface, lod.edgeLength, lod.indices.data(),
                        lod.indexCount))
            {
                job->ok = false;
                return true;
            }
            const std::uint64_t size = lod.indices.size();
            info->lodBytes += size;
            ++info->lodLevels;
            lod.indices = ct::Vector<std::uint8_t>();
            ++piece.lodsUploaded;
            budget -= size < budget ? size : budget;
            if (budget == 0 && piece.lodsUploaded < piece.lods.size()) return false;
        }
        if (!piece.source.positions.empty())
            mMeshes.setSurfaceSource(mesh, surface, static_cast<SurfaceData&&>(piece.source));
        if (!mMeshes.finishSurface(mesh, surface))
        {
            job->ok = false;
            return true;
        }
        ++job->current;
        if (budget == 0 && job->current < job->pieces.size()) return false;
    }
    info->lodSeconds += job->lodSeconds;
    job->lodSeconds = 0.0;
    return true;
}

bool AssetLoader::advanceImage(ImageJob* job, std::uint64_t& budget)
{
    if (!job->ok) return true;
    ModelInfo* info = job->info;
    if (!job->target.valid())
        info->totalBytes += static_cast<std::uint64_t>(job->pixmap->get_size());

    std::uint64_t uploadedNow = 0;
    const BandResult result = uploadBands(mTextures, *job->pixmap, job->target, job->rowsUploaded,
            job->srgb, true, job->name.c_str(), budget, uploadedNow);
    mUploadedBytes += uploadedNow;
    info->uploadedBytes += uploadedNow;
    job->uploaded += uploadedNow;
    if (result == BandResult::More) return false;
    if (result == BandResult::Failed)
    {
        job->ok = false;
        return true;
    }
    job->ok = mTextures.replace(job->placeholder, job->target);
    if (job->ok) job->target = TextureId();
    return true;
}

void AssetLoader::finishTextureJob(TextureJob* job)
{
    TextureEntry* entry = mTextureCache.find(job->key);
    if (entry)
    {
        if (job->ok)
        {
            entry->status = LoadStatus::Loaded;
            mTextures.setState(entry->texture, TextureState::Ready);
        }
        else
        {
            entry->status = LoadStatus::Failed;
            mTextures.setState(entry->texture, TextureState::Failed);
            log_error("AssetLoader: texture '%s' failed to load", job->path.c_str());
        }
    }
    if (job->target.valid()) mTextures.destroy(job->target);
    ++mCompletedRequests;
}

void AssetLoader::completeModelJob(ModelInfo& info, bool ok)
{
    if (info.pendingJobs > 0) --info.pendingJobs;
    if (!ok)
    {
        ++info.failedJobs;
        log_error("AssetLoader: a load job of model '%s' failed", info.path.c_str());
    }
    if (info.pendingJobs == 0)
    {
        info.status = info.failedJobs != 0 ? LoadStatus::Failed : LoadStatus::Loaded;
        if (info.failedJobs != 0 && info.error.empty()) info.error = "a load job failed";
        ++mCompletedRequests;
    }
}

void AssetLoader::complete(Job* job)
{
    std::uint64_t held = job->bytes;
    switch (job->kind)
    {
        case Job::Texture:
            finishTextureJob(static_cast<TextureJob*>(job));
            break;
        case Job::Parse:
            break;
        case Job::Primitive:
        {
            PrimitiveJob* primitive = static_cast<PrimitiveJob*>(job);
            if (primitive->info->collision) collectCollisionPart(primitive);
            held = primitive->bytes;
            completeModelJob(*job->info, job->ok);
            const std::uint32_t next = ++job->info->nextPrimitives[primitive->mesh];
            for (std::size_t i = 0; i < mWaiting.size(); ++i)
            {
                PrimitiveJob* waiting = static_cast<PrimitiveJob*>(mWaiting[i]);
                if (waiting->info != job->info || waiting->mesh != primitive->mesh ||
                        waiting->primitive != next)
                    continue;
                mWaiting.erase(mWaiting.begin() + i);
                mUploads.push_back(waiting);
                break;
            }
            break;
        }
        case Job::Image:
        {
            ImageJob* image = static_cast<ImageJob*>(job);
            TextureEntry* entry = mTextureCache.find(image->key);
            if (entry)
            {
                entry->status = image->ok ? LoadStatus::Loaded : LoadStatus::Failed;
                mTextures.setState(entry->texture,
                        image->ok ? TextureState::Ready : TextureState::Failed);
            }
            if (image->target.valid()) mTextures.destroy(image->target);
            completeModelJob(*job->info, job->ok);
            break;
        }
        case Job::Collision:
        {
            CollisionJob* collision = static_cast<CollisionJob*>(job);
            if (collision->ok && collision->result)
            {
                mMeshes.setCollision(collision->info->meshes[collision->mesh], collision->result);
                collision->result = nullptr;
            }
            completeModelJob(*job->info, job->ok);
            break;
        }
    }
    delete job;
    if (held != 0) release(held);
}

void AssetLoader::collectCollisionPart(PrimitiveJob* job)
{
    ModelCollisionState::MeshBuild& build = job->info->collision->meshes[job->mesh];
    if (job->ok)
    {
        const std::uint64_t partBytes = job->part.bytes();
        build.parts[job->primitive] = static_cast<ModelCollisionState::Part&&>(job->part);
        job->bytes -= partBytes < job->bytes ? partBytes : job->bytes;
    }
    else
        build.failed = true;

    if (--build.remaining != 0) return;

    if (build.failed)
    {
        build.parts = ct::Vector<ModelCollisionState::Part>();
        completeModelJob(*job->info, true);
        return;
    }

    CollisionJob* collision = new CollisionJob();
    collision->model = job->model;
    collision->info = job->info;
    collision->mesh = job->mesh;
    collision->parts = static_cast<ct::Vector<ModelCollisionState::Part>&&>(build.parts);
    for (std::size_t i = 0; i < collision->parts.size(); ++i)
        collision->bytes += collision->parts[i].bytes();
    submit(collision);
}

void AssetLoader::discard(Job* job)
{
    const std::uint64_t held = job->bytes;
    switch (job->kind)
    {
        case Job::Texture:
        {
            TextureJob* texture = static_cast<TextureJob*>(job);
            TextureEntry* entry = mTextureCache.find(texture->key);
            if (entry)
            {
                entry->status = LoadStatus::Failed;
                mTextures.setState(entry->texture, TextureState::Failed);
            }
            if (texture->target.valid()) mTextures.destroy(texture->target);
            break;
        }
        case Job::Parse:
            job->info->status = LoadStatus::Failed;
            job->info->error = "cancelled";
            break;
        case Job::Primitive:
            job->info->status = LoadStatus::Failed;
            job->info->error = "cancelled";
            break;
        case Job::Image:
        {
            ImageJob* image = static_cast<ImageJob*>(job);
            TextureEntry* entry = mTextureCache.find(image->key);
            if (entry)
            {
                entry->status = LoadStatus::Failed;
                mTextures.setState(entry->texture, TextureState::Failed);
            }
            if (image->target.valid()) mTextures.destroy(image->target);
            job->info->status = LoadStatus::Failed;
            job->info->error = "cancelled";
            break;
        }
        case Job::Collision:
            job->info->status = LoadStatus::Failed;
            job->info->error = "cancelled";
            break;
    }
    delete job;
    if (held != 0) release(held);
}

void AssetLoader::pump(std::uint64_t byteBudget, std::uint32_t maxUploads)
{
    {
        ct::LockGuard guard(mDoneMutex);
        for (std::size_t i = 0; i < mDone.size(); ++i) mUploads.push_back(mDone[i]);
        mDone.clear();
    }

    std::uint64_t remaining = byteBudget;
    std::uint32_t touched = 0;
    std::size_t kept = 0;
    std::size_t index = 0;
    for (; index < mUploads.size(); ++index)
    {
        Job* job = mUploads[index];
        if (job->kind == Job::Primitive)
        {
            const PrimitiveJob* primitive = static_cast<const PrimitiveJob*>(job);
            if (primitive->primitive != job->info->nextPrimitives[primitive->mesh])
            {
                mWaiting.push_back(job);
                continue;
            }
        }
        const bool allowed =
                touched == 0 || (remaining > 0 && (maxUploads == 0 || touched < maxUploads));
        if (!allowed) break;
        ++touched;
        if (advance(job, remaining)) complete(job);
        else
            mUploads[kept++] = job;
    }
    if (kept < index) mUploads.erase(mUploads.begin() + kept, mUploads.begin() + index);
}

void AssetLoader::waitAll()
{
    for (;;)
    {
        pump(~0ull, 0);
        ct::LockGuard guard(mDoneMutex);
        if (!mDone.empty()) continue;
        if (mActiveJobs.load() == 0)
        {
            if (mUploads.empty()) break;
            continue;
        }
        while (mDone.empty() && mActiveJobs.load() != 0) mDoneCv.wait(mDoneMutex);
    }
}

std::uint32_t AssetLoader::totalRequests() const
{
    return mTotalRequests;
}

std::uint32_t AssetLoader::completedRequests() const
{
    return mCompletedRequests;
}

std::uint32_t AssetLoader::pendingRequests() const
{
    return mTotalRequests - mCompletedRequests;
}

void AssetLoader::resetCounters()
{
    mTotalRequests = pendingRequests();
    mCompletedRequests = 0;
}

std::uint64_t AssetLoader::uploadedBytes() const
{
    return mUploadedBytes;
}

} // namespace prisma3d
