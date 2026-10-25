#pragma once

#include "prisma/rhi/Driver.h"
#include "prisma3d/render/MaterialStore.h"

#include "mathc.h"

#include <ct/slotmap32.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

struct MeshTag;
using MeshId = ct::Handle32<MeshTag>;

enum VertexAttributeBit : std::uint32_t
{
    kVertexPosition = 1u << 0,
    kVertexNormal = 1u << 1,
    kVertexTangent = 1u << 2,
    kVertexColor = 1u << 3,
    kVertexUv = 1u << 4,
    kVertexUv2 = 1u << 5,
    kVertexJoints = 1u << 6,
    kVertexWeights = 1u << 7
};

struct SurfaceData
{
    prisma::Topology topology = prisma::Topology::Triangles;
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec3> normals;
    ct::Vector<Math::Vec4> tangents;
    ct::Vector<Math::Vec4> colors;
    ct::Vector<Math::Vec2> uvs;
    ct::Vector<Math::Vec2> uvs2;
    ct::Vector<std::uint32_t> joints;
    ct::Vector<Math::Vec4> weights;
    ct::Vector<std::uint32_t> indices;
};

struct SurfaceLod
{
    float edgeLength = 0.0f;
    ct::Vector<std::uint32_t> indices;
};

struct SurfaceLayout
{
    prisma::Topology topology = prisma::Topology::Triangles;
    std::uint32_t attributeMask = kVertexPosition | kVertexNormal | kVertexColor;
    std::uint32_t vertexCount = 0;
    std::uint32_t indexCount = 0;
    prisma::IndexFormat indexFormat = prisma::IndexFormat::UInt16;
    bool storageVertices = false;
    Math::Box aabb;
    ct::Vector<Math::Box> boneAabbs;

    std::uint32_t vertexBytes() const;
    std::uint32_t indexBytes() const;
};

struct SurfaceLodInfo
{
    prisma::BufferHandle indexBuffer;
    std::uint32_t indexCount = 0;
    float edgeLength = 0.0f;
};

struct SurfaceInfo
{
    prisma::BufferHandle vertexBuffer;
    prisma::BufferHandle indexBuffer;
    std::uint32_t attributeMask = kVertexPosition;
    std::uint32_t vertexStride = 0;
    std::uint32_t vertexCount = 0;
    std::uint32_t indexCount = 0;
    std::uint32_t vertexCapacity = 0;
    std::uint32_t indexCapacity = 0;
    prisma::IndexFormat indexFormat = prisma::IndexFormat::UInt32;
    prisma::Topology topology = prisma::Topology::Triangles;
    Math::Box aabb;
    ct::Vector<Math::Box> boneAabbs;
    bool ready = true;
    bool dynamic = false;
    bool sharedVertices = false;
    bool sharedIndices = false;
    MaterialId material;
    ct::Vector<SurfaceLodInfo> lods;
    SurfaceData source;

    bool skinned() const;
};

class CollisionMesh;

struct MeshInfo
{
    ct::Vector<SurfaceInfo> surfaces;
    Math::Box aabb;
    ct::String name;
    ct::String path;
    std::uint32_t pendingSurfaces = 0;
    std::uint32_t version = 0;
    CollisionMesh* collision = nullptr;
    MeshId shadowMesh;
};

class MeshStore
{
public:
    explicit MeshStore(prisma::Driver* driver);
    ~MeshStore();
    MeshStore(const MeshStore&) = delete;
    MeshStore& operator=(const MeshStore&) = delete;

    static MeshStore* singleton();
    prisma::Driver* driver() const;

    MeshId create(const char* name = nullptr);
    bool addSurface(MeshId mesh, const SurfaceData& surface, const char* name = nullptr,
            bool updatableVertices = false);
    int beginSurface(MeshId mesh, const SurfaceLayout& layout, const char* name = nullptr);
    bool uploadVertices(MeshId mesh, std::uint32_t surface, std::uint32_t offset, const void* data,
            std::uint32_t size);
    bool uploadIndices(MeshId mesh, std::uint32_t surface, std::uint32_t offset, const void* data,
            std::uint32_t size);
    bool updateVertices(MeshId mesh, std::uint32_t surface, std::uint32_t offset, const void* data,
            std::uint32_t size);
    bool setSurfaceAabb(MeshId mesh, std::uint32_t surface, const Math::Box& aabb);
    bool addSurfaceLod(MeshId mesh, std::uint32_t surface, float edgeLength, const void* indices,
            std::uint32_t indexCount);
    bool finishSurface(MeshId mesh, std::uint32_t surface);
    int addDynamicSurface(MeshId mesh, const SurfaceLayout& capacity, const char* name = nullptr);
    int addIndexedSurface(MeshId mesh, MeshId vertexMesh, std::uint32_t vertexSurface,
            const std::uint16_t* indices, std::uint32_t indexCount, const Math::Box& aabb,
            const char* name = nullptr);
    int addSharedSurface(MeshId mesh, MeshId vertexMesh, std::uint32_t vertexSurface,
            MeshId indexMesh, std::uint32_t indexSurface, const Math::Box& aabb);
    bool updateSurface(MeshId mesh, std::uint32_t surface, const SurfaceData& data);
    bool setDynamicSurfaceRange(MeshId mesh, std::uint32_t surface, std::uint32_t vertexCount,
            std::uint32_t indexCount, const Math::Box& aabb);
    bool complete(MeshId mesh) const;
    bool transform(MeshId mesh, const Math::Mat4& matrix);
    bool setSurfaceMaterial(MeshId mesh, std::uint32_t surface, MaterialId material);
    bool setSurfaceSource(MeshId mesh, std::uint32_t surface, SurfaceData&& source);
    const SurfaceData* surfaceSource(MeshId mesh, std::uint32_t surface) const;
    bool setShadowMesh(MeshId mesh, MeshId shadowMesh);
    MeshId getShadowMesh(MeshId mesh) const;
    bool setCollision(MeshId mesh, CollisionMesh* collision);
    bool addCollisionSurface(MeshId mesh, const SurfaceData& surface);
    const CollisionMesh* collision(MeshId mesh) const;
    void clearSurfaces(MeshId mesh);
    void destroy(MeshId mesh);
    bool adopt(MeshId target, MeshId donor);
    void clear();

    bool valid(MeshId mesh) const;
    MeshId find(const ct::String& path) const;
    void setPath(MeshId mesh, const ct::String& path);
    const MeshInfo* get(MeshId mesh) const;
    std::size_t count() const;

    static bool interleave(const SurfaceData& surface, SurfaceLayout& layout,
            ct::Vector<std::uint8_t>& vertices, ct::Vector<std::uint8_t>& indices);
    static std::uint32_t attributeOffset(std::uint32_t attributeMask, VertexAttributeBit attribute);
    static std::uint32_t vertexStride(std::uint32_t attributeMask);
    static Math::Box computeAabb(const ct::Vector<Math::Vec3>& positions);
    static void computeBoneAabbs(const SurfaceData& surface, ct::Vector<Math::Box>& boneAabbs);
    static bool generateTangents(SurfaceData& surface);
    static bool generateLods(const SurfaceData& surface, float normalMergeAngle,
            ct::Vector<SurfaceLod>& lods);
    static void packIndices(const ct::Vector<std::uint32_t>& indices, prisma::IndexFormat format,
            ct::Vector<std::uint8_t>& bytes);
    static bool splitByGrid(const SurfaceData& surface, float cellSize,
            std::uint32_t maxCellsPerAxis, ct::Vector<SurfaceData>& pieces);

private:
    static MeshStore* sSingleton;

    prisma::Driver* mDriver;
    ct::SlotMap32<MeshInfo> mMeshes;
    ct::Vector<std::uint8_t> mScratchVertices;
    ct::Vector<std::uint8_t> mScratchIndices;
};

} // namespace prisma3d
