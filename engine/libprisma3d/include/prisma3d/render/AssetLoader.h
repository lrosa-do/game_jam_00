#pragma once

#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"

#include <ct/hashmap.hpp>
#include <ct/slotmap32.hpp>
#include <ct/string.hpp>
#include <ct/thread.hpp>
#include <ct/threadpool.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Pixmap;
class AnimatedTexture;

enum class LoadStatus : std::uint8_t
{
    Invalid,
    InProgress,
    Failed,
    Loaded
};

struct ModelTag;
using ModelId = ct::Handle32<ModelTag>;

struct ModelOptions
{
    int collisionType = 0;
    bool materialSidecar = true;
    float splitCellSize = 64.0f;
    std::uint32_t splitMinTriangles = 50000;
    bool generateLods = true;
    float lodNormalMergeAngle = 20.0f;
    float scale = 1.0f;
    int physicsShape = 0;
    bool kinematicBody = false;
    float lightmapEnergy = 1.0f;
    ct::String collidePrefixes;
    ct::String hidePrefixes;
};

struct ModelCollisionState;

struct ModelInfo
{
    ModelInfo() = default;
    ModelInfo(const ModelInfo&) = delete;
    ModelInfo& operator=(const ModelInfo&) = delete;
    ~ModelInfo();

    ct::String path;
    LoadStatus status = LoadStatus::InProgress;
    bool parsed = false;
    int collisionType = 0;
    bool materialSidecar = true;
    float scale = 1.0f;
    int physicsShape = 0;
    bool kinematicBody = false;
    float lightmapEnergy = 1.0f;
    ct::String collidePrefixes;
    ct::String hidePrefixes;
    ModelCollisionState* collision = nullptr;
    ModelData document;
    ModelImporter* importer = nullptr;
    ct::Vector<MeshId> meshes;
    ct::Vector<std::uint32_t> nextPrimitives;
    ct::Vector<std::uint32_t> reservedPrimitives;
    ct::Vector<MaterialId> materials;
    ct::Vector<TextureId> textures;
    ct::Vector<ct::String> unmatchedMaterials;
    float splitCellSize = 0.0f;
    std::uint32_t splitMinTriangles = 0;
    bool generateLods = true;
    float lodNormalMergeAngle = 20.0f;
    std::uint32_t lodLevels = 0;
    std::uint64_t lodBytes = 0;
    double lodSeconds = 0.0;
    std::uint64_t totalBytes = 0;
    std::uint64_t uploadedBytes = 0;
    std::uint32_t pendingJobs = 0;
    std::uint32_t failedJobs = 0;
    ct::String error;
};

class AssetLoader
{
public:
    static constexpr std::uint64_t kDefaultByteBudget = 8u << 20;
    static constexpr std::uint64_t kDefaultPendingByteLimit = 256u << 20;

    AssetLoader(TextureStore& textures, MaterialStore& materials, MeshStore& meshes,
            ModelLoader& importers, unsigned threads = 0);
    ~AssetLoader();
    AssetLoader(const AssetLoader&) = delete;
    AssetLoader& operator=(const AssetLoader&) = delete;

    static AssetLoader* singleton();

    static ct::String modelFile(const ct::String& path);
    static void parseModelOptions(const ct::String& path, ModelOptions& options);

    TextureId requestTexture(const ct::String& path, bool srgb, bool generateMips = true);
    TextureId requestAnimatedTexture(const ct::Vector<ct::String>& paths, bool srgb, float fps,
            bool generateMips = true);
    AnimatedTexture* animatedTexture(TextureId texture) const;
    LoadStatus textureStatus(TextureId texture) const;
    LoadStatus status(const ct::String& path, bool srgb) const;

    ModelId requestModel(const ct::String& path, const ModelOptions& options = ModelOptions());
    ModelId findModelByPath(const ct::String& path) const;
    LoadStatus modelStatus(ModelId model) const;
    const ModelInfo* model(ModelId model) const;
    float modelProgress(ModelId model) const;
    bool freeModel(ModelId model);
    bool replaceMesh(ModelId model, MeshId from, MeshId to);

    void pump(std::uint64_t byteBudget = kDefaultByteBudget, std::uint32_t maxUploads = 0);
    void waitAll();

    void setPendingByteLimit(std::uint64_t bytes);
    std::uint64_t pendingByteLimit() const;
    std::uint64_t pendingBytes() const;

    std::uint32_t totalRequests() const;
    std::uint32_t completedRequests() const;
    std::uint32_t pendingRequests() const;
    void resetCounters();
    std::uint64_t uploadedBytes() const;

private:
    struct TextureEntry
    {
        TextureId texture;
        bool srgb = false;
        bool generateMips = true;
        LoadStatus status = LoadStatus::InProgress;
    };

    struct Job;
    struct TextureJob;
    struct ModelParseJob;
    struct PrimitiveJob;
    struct ImageJob;
    struct CollisionJob;

    static AssetLoader* sSingleton;

    void submit(Job* job);
    void runJob(Job* job);
    void runTextureJob(TextureJob* job);
    void runParseJob(ModelParseJob* job);
    void runPrimitiveJob(PrimitiveJob* job);
    void runImageJob(ImageJob* job);
    void runCollisionJob(CollisionJob* job);
    bool reserve(std::uint64_t bytes);
    bool reservePrimitive(ModelInfo& info, std::uint32_t mesh, std::uint32_t primitive,
            std::uint64_t bytes);
    void release(std::uint64_t bytes);
    void adjustPending(std::uint64_t from, std::uint64_t to);
    void finished(Job* job);

    MaterialId createMaterial(const ModelMaterialData& data, const ct::Vector<TextureId>& textures);
    void finishParse(ModelParseJob* job);
    bool advance(Job* job, std::uint64_t& budget);
    bool advanceTexture(TextureJob* job, std::uint64_t& budget);
    bool advancePrimitive(PrimitiveJob* job, std::uint64_t& budget);
    bool advanceImage(ImageJob* job, std::uint64_t& budget);
    void complete(Job* job);
    void collectCollisionPart(PrimitiveJob* job);
    void finishTextureJob(TextureJob* job);
    void completeModelJob(ModelInfo& info, bool ok);
    void discard(Job* job);
    ModelInfo* findModel(ModelId model) const;

    TextureStore& mTextures;
    MaterialStore& mMaterials;
    MeshStore& mMeshes;
    ModelLoader& mImporters;
    ct::ThreadPool mPool;
    ct::HashMap<ct::String, TextureEntry> mTextureCache;
    ct::HashMap<std::uint32_t, ct::String> mTextureKeys;
    ct::HashMap<ct::String, AnimatedTexture*> mAnimated;
    ct::SlotMap32<ModelInfo*> mModels;
    ct::Mutex mDoneMutex;
    ct::CondVar mDoneCv;
    ct::Vector<Job*> mDone;
    ct::Vector<Job*> mUploads;
    ct::Vector<Job*> mWaiting;
    mutable ct::Mutex mPendingMutex;
    ct::CondVar mPendingCv;
    std::uint64_t mPendingBytes;
    std::uint64_t mPendingByteLimit;
    bool mStopping;
    ct::Atomic<std::uint32_t> mActiveJobs;
    std::uint32_t mTotalRequests;
    std::uint32_t mCompletedRequests;
    std::uint64_t mUploadedBytes;
};

} // namespace prisma3d
