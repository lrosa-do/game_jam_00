#pragma once

#include "prisma/rhi/Driver.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"

#include "mathc.h"

#include <ct/slotmap32.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

constexpr std::uint32_t kMultiMeshDirtyRegionSize = 512;
constexpr std::uint32_t kMultiMeshTransformFloats = 12;

struct MultiMeshData
{
    MultiMeshId self;
    MeshId mesh;
    std::uint32_t instances = 0;
    bool usesColors = false;
    bool usesCustomData = false;
    bool dissolve = false;
    int visibleInstances = -1;
    Math::Box aabb;
    Math::Box customAabb;
    bool aabbDirty = false;
    bool bufferSet = false;
    std::uint32_t strideCache = 0;
    std::uint32_t colorOffsetCache = 0;
    std::uint32_t customDataOffsetCache = 0;
    ct::Vector<float> dataCache;
    ct::Vector<std::uint8_t> dirtyRegions;
    std::uint32_t dirtyRegionCount = 0;
    prisma::BufferHandle buffer;
    bool dirty = false;
    std::uint32_t version = 1;
    MeshId aabbMesh;
    std::uint32_t aabbMeshVersion = 0;
};

class MultiMeshStorage
{
public:
    MultiMeshStorage();
    MultiMeshStorage(const MultiMeshStorage&) = delete;
    MultiMeshStorage& operator=(const MultiMeshStorage&) = delete;

    void setup(prisma::Driver* driver, MeshStore* meshes);
    void destroyAll();

    MultiMeshId create();
    void free(MultiMeshId multimesh);
    bool valid(MultiMeshId multimesh) const;
    const MultiMeshData* get(MultiMeshId multimesh) const;
    std::size_t count() const;

    void allocateData(MultiMeshId multimesh, std::uint32_t instances, bool useColors,
            bool useCustomData);
    std::uint32_t getInstanceCount(MultiMeshId multimesh) const;
    void setMesh(MultiMeshId multimesh, MeshId mesh);
    MeshId getMesh(MultiMeshId multimesh) const;
    void instanceSetTransform(MultiMeshId multimesh, std::uint32_t index,
            const Math::Mat4& transform);
    void instanceSetColor(MultiMeshId multimesh, std::uint32_t index, const Math::Vec4& color);
    void instanceSetCustomData(MultiMeshId multimesh, std::uint32_t index, const Math::Vec4& data);
    Math::Mat4 instanceGetTransform(MultiMeshId multimesh, std::uint32_t index);
    Math::Vec4 instanceGetColor(MultiMeshId multimesh, std::uint32_t index);
    Math::Vec4 instanceGetCustomData(MultiMeshId multimesh, std::uint32_t index);
    bool setBuffer(MultiMeshId multimesh, const float* data, std::size_t count);
    bool getBuffer(MultiMeshId multimesh, ct::Vector<float>& out);
    void setVisibleInstances(MultiMeshId multimesh, int visible);
    int getVisibleInstances(MultiMeshId multimesh) const;
    void setDissolve(MultiMeshId multimesh, bool enable);
    bool getDissolve(MultiMeshId multimesh) const;
    void setCustomAabb(MultiMeshId multimesh, const Math::Box& aabb);
    Math::Box getCustomAabb(MultiMeshId multimesh) const;
    Math::Box getAabb(MultiMeshId multimesh);
    std::uint32_t getVersion(MultiMeshId multimesh) const;
    void updateDirty();
    std::uint32_t takeUploadedBytes();

    static std::uint32_t instancesToDraw(const MultiMeshData& data);
    static Math::Box effectiveAabb(const MultiMeshData& data);

private:
    MultiMeshData* find(MultiMeshId multimesh);
    const MultiMeshData* find(MultiMeshId multimesh) const;
    void makeLocal(MultiMeshData& data);
    void markDirty(MultiMeshData& data, MultiMeshId id, int index, bool aabb);
    void markAllDirty(MultiMeshData& data, MultiMeshId id, bool dataRegions, bool aabb);
    void recreateAabb(MultiMeshData& data, const float* values, int instances);
    void recreateAabbFromBuffer(MultiMeshData& data);
    void freeBuffer(MultiMeshData& data);
    void noteMeshDependency(MultiMeshData& data);
    void addToDirtyList(MultiMeshData& data, MultiMeshId id);

    prisma::Driver* mDriver;
    MeshStore* mMeshes;
    ct::SlotMap32<MultiMeshData> mItems;
    ct::Vector<MultiMeshId> mDirtyList;
    std::uint32_t mUploadedBytes;
};

} // namespace prisma3d
