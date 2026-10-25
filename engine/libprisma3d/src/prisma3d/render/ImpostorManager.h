#pragma once

#include "prisma/rhi/Driver.h"
#include "prisma3d/render/Impostor.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/slotmap32.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class ImpostorManager
{
public:
    struct Entry
    {
        ImpostorId self;
        MeshId mesh;
        ct::Vector<MaterialId> materials;
        float swapInDistance = 100.0f;
        int slot = -1;
        bool dirty = true;
        bool captured = false;
    };

    struct Record
    {
        ImpostorId impostor;
        Math::Mat4 transform;
        std::uint32_t color = 0xffffffffu;
    };

    struct Set
    {
        std::uint32_t scenario = 0;
        bool enabled = true;
        bool dirty = true;
        bool waiting = false;
        ct::Vector<Record> records;
        prisma::BufferHandle instances;
        prisma::BufferHandle output;
        prisma::BufferHandle indices;
        prisma::BufferHandle cardParams;
        std::uint32_t capacity = 0;
        std::uint32_t uploaded = 0;
        std::uint32_t visible = 0;
    };

    ImpostorManager();

    ImpostorId create(MeshId mesh, const ct::Vector<MaterialId>& materials, float swapInDistance);
    void free(ImpostorId id);
    Entry* get(ImpostorId id);
    const Entry* get(ImpostorId id) const;
    void setDirty(ImpostorId id);
    std::uint32_t count() const;

    ImpostorSetId createSet(std::uint32_t scenario);
    void freeSet(prisma::Driver* driver, ImpostorSetId id);
    Set* getSet(ImpostorSetId id);
    const Set* getSet(ImpostorSetId id) const;
    bool instanceInfo(const MeshStore& meshes, const Record& record,
            ImpostorInstanceInfo& info) const;
    void markSetsDirty();
    void uploadSets(prisma::Driver* driver, const MeshStore& meshes);
    std::uint32_t prepare(prisma::Driver* driver, std::uint32_t scenario,
            const Math::Frustum& frustum, const Math::Vec3& camera);
    std::uint32_t drawDepth(prisma::Driver* driver, std::uint32_t scenario, std::uint32_t samples);
    std::uint32_t drawColor(prisma::Driver* driver, std::uint32_t scenario, std::uint32_t samples,
            bool equalDepth);
    bool readCards(prisma::Driver* driver, ImpostorSetId id, std::uint32_t& indexCount,
            ct::Vector<ImpostorCardReadback>& cards) const;
    std::uint32_t setCount() const;

    std::uint32_t refresh(prisma::Driver* driver, MeshStore& meshes, MaterialStore& materials,
            TextureStore& textures);
    bool readSlice(prisma::Driver* driver, ImpostorId id, std::uint32_t angle,
            ImpostorProperty property, ct::Vector<std::uint8_t>& rgba) const;

    prisma::TextureHandle array() const;
    std::uint64_t textureBytes() const;
    std::uint32_t captures() const;
    std::uint32_t samples() const;
    void destroy(prisma::Driver* driver);

private:
    bool ensureResources(prisma::Driver* driver);
    prisma::PipelineHandle capturePipeline(prisma::Driver* driver, const SurfaceInfo& surface,
            std::uint32_t variant);
    bool entryReady(const Entry& entry, const MeshStore& meshes, const MaterialStore& materials,
            const TextureStore& textures) const;

    void destroyCapture(prisma::Driver* driver);
    bool ensurePrepare(prisma::Driver* driver);
    prisma::PipelineHandle cardPipeline(prisma::Driver* driver, std::uint32_t variant,
            std::uint32_t samples, bool equalDepth);
    std::uint32_t drawCards(prisma::Driver* driver, std::uint32_t scenario, std::uint32_t variant,
            std::uint32_t samples, bool equalDepth);
    bool ensureSetBuffers(prisma::Driver* driver, Set& set, std::uint32_t count);

    ct::SlotMap32<Entry> mEntries;
    ct::SlotMap32<Set> mSets;
    prisma::ShaderHandle mPrepareShader;
    prisma::PipelineHandle mPreparePipeline;
    prisma::BufferHandle mPrepareParams;
    std::uint32_t mPrepareParamsSize;
    bool mPrepareFailed;
    bool mTaken[kMaxImpostorCount];
    prisma::TextureHandle mArray;
    prisma::TextureHandle mColor[kImpostorProperties];
    prisma::TextureHandle mDepth;
    prisma::SamplerHandle mSampler;
    prisma::SamplerHandle mCardSampler;
    prisma::BufferHandle mSceneBuffer;
    prisma::BufferHandle mMaterialBuffer;
    prisma::BufferHandle mDrawBuffer;
    prisma::BufferHandle mInstanceBuffer;
    std::uint32_t mSceneStride;
    std::uint32_t mMaterialStride;
    std::uint32_t mSamples;
    std::uint32_t mCaptures;
    bool mFailed;
    ct::HashMap<std::uint64_t, prisma::PipelineHandle> mPipelines;
    ct::HashMap<std::uint32_t, prisma::ShaderHandle> mShaders;
    ct::HashMap<std::uint32_t, prisma::PipelineHandle> mCardPipelines;
};

} // namespace prisma3d
