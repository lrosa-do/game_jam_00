#include "prisma3d/render/MeshStore.h"

#include "prisma3d/scene/Collision.h"

#include "platform.h"

#include "meshoptimizer.h"

#include <ct/sort.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

typedef ct::Handle32<MeshInfo> SlotKey;

const float kCmpEpsilon = 0.00001f;
const float kCmpEpsilon2 = kCmpEpsilon * kCmpEpsilon;
const std::uint32_t kNoVertex = 0xffffffffu;

bool approximately(float a, float b)
{
    if (a == b) return true;
    float tolerance = kCmpEpsilon * fabsf(a);
    if (tolerance < kCmpEpsilon) tolerance = kCmpEpsilon;
    return fabsf(a - b) < tolerance;
}

bool samePosition(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

std::uint32_t positionHash(const Math::Vec3& position)
{
    const float values[3] = { position.x == 0.0f ? 0.0f : position.x,
        position.y == 0.0f ? 0.0f : position.y, position.z == 0.0f ? 0.0f : position.z };
    std::uint32_t bits[3];
    memcpy(bits, values, sizeof bits);
    std::uint32_t hash = 2166136261u;
    for (int i = 0; i < 3; ++i)
    {
        std::uint32_t k = bits[i] * 0xcc9e2d51u;
        k = (k << 15) | (k >> 17);
        hash ^= k * 0x1b873593u;
        hash = ((hash << 13) | (hash >> 19)) * 5u + 0xe6546b64u;
    }
    hash ^= hash >> 16;
    hash *= 0x85ebca6bu;
    hash ^= hash >> 13;
    return hash;
}

SlotKey toKey(MeshId id)
{
    return SlotKey::from_bits(id.bits());
}

MeshId toId(SlotKey key)
{
    return MeshId::from_bits(key.bits());
}

const VertexAttributeBit kLayoutOrder[] = { kVertexPosition, kVertexNormal, kVertexColor,
    kVertexTangent, kVertexUv, kVertexUv2, kVertexJoints, kVertexWeights };

std::uint32_t attributeSize(VertexAttributeBit attribute)
{
    switch (attribute)
    {
        case kVertexPosition:
        case kVertexNormal:
            return 12;
        case kVertexColor:
        case kVertexTangent:
        case kVertexWeights:
            return 16;
        case kVertexJoints:
            return 4;
        case kVertexUv:
        case kVertexUv2:
            return 8;
    }
    return 0;
}

SurfaceInfo makeSurface(const SurfaceLayout& layout)
{
    SurfaceInfo result;
    result.attributeMask = layout.attributeMask;
    result.vertexStride = MeshStore::vertexStride(layout.attributeMask);
    result.vertexCount = layout.vertexCount;
    result.indexCount = layout.indexCount;
    result.indexFormat = layout.indexFormat;
    result.topology = layout.topology;
    result.aabb = layout.aabb;
    result.boneAabbs = layout.boneAabbs;
    return result;
}

void destroySurfaceBuffers(prisma::Driver* driver, SurfaceInfo& surface)
{
    if (!surface.sharedVertices) driver->destroy(surface.vertexBuffer);
    if (surface.indexBuffer.valid() && !surface.sharedIndices) driver->destroy(surface.indexBuffer);
    for (SurfaceLodInfo& lod: surface.lods) driver->destroy(lod.indexBuffer);
    surface.lods.clear();
}

prisma::BufferUsage vertexUsage(std::uint32_t attributeMask)
{
    return (attributeMask & kVertexJoints) ? prisma::BufferUsage::Storage
                                           : prisma::BufferUsage::Vertex;
}

void put(float*& cursor, const float* values, unsigned count)
{
    for (unsigned i = 0; i < count; ++i) *cursor++ = values[i];
}

} // namespace

MeshStore* MeshStore::sSingleton = nullptr;

MeshStore::MeshStore(prisma::Driver* driver)
        : mDriver(driver)
{
    sSingleton = this;
}

MeshStore::~MeshStore()
{
    clear();
    if (sSingleton == this) sSingleton = nullptr;
}

MeshStore* MeshStore::singleton()
{
    return sSingleton;
}

prisma::Driver* MeshStore::driver() const
{
    return mDriver;
}

MeshId MeshStore::create(const char* name)
{
    MeshInfo info;
    if (name) info.name = name;
    return toId(mMeshes.insert(static_cast<MeshInfo&&>(info)));
}

std::uint32_t SurfaceLayout::vertexBytes() const
{
    return vertexCount * MeshStore::vertexStride(attributeMask);
}

std::uint32_t SurfaceLayout::indexBytes() const
{
    return indexCount * (indexFormat == prisma::IndexFormat::UInt16 ? 2u : 4u);
}

bool MeshStore::interleave(const SurfaceData& surface, SurfaceLayout& layout,
        ct::Vector<std::uint8_t>& vertices, ct::Vector<std::uint8_t>& indices)
{
    const std::size_t count = surface.positions.size();
    if (count == 0)
    {
        log_error("mesh: a surface needs positions");
        return false;
    }
    const bool hasNormals = !surface.normals.empty();
    const bool hasColors = !surface.colors.empty();
    const bool hasTangents = !surface.tangents.empty();
    const bool hasUv = !surface.uvs.empty();
    const bool hasUv2 = !surface.uvs2.empty();
    const bool hasJoints = !surface.joints.empty();
    const bool hasWeights = !surface.weights.empty();
    if (hasJoints != hasWeights)
    {
        log_error("mesh: joints and weights must come together");
        return false;
    }
    if ((hasJoints && surface.joints.size() != count) ||
            (hasWeights && surface.weights.size() != count))
    {
        log_error("mesh: attribute arrays must match the position count");
        return false;
    }
    if ((hasNormals && surface.normals.size() != count) ||
            (hasColors && surface.colors.size() != count) ||
            (hasTangents && surface.tangents.size() != count) ||
            (hasUv && surface.uvs.size() != count) || (hasUv2 && surface.uvs2.size() != count))
    {
        log_error("mesh: attribute arrays must match the position count");
        return false;
    }

    for (std::size_t i = 0; i < surface.indices.size(); ++i)
    {
        if (surface.indices[i] >= count)
        {
            log_error("mesh: index out of range");
            return false;
        }
    }

    std::uint32_t mask = kVertexPosition | kVertexNormal | kVertexColor;
    if (hasTangents) mask |= kVertexTangent;
    if (hasUv) mask |= kVertexUv;
    if (hasUv2) mask |= kVertexUv2;
    if (hasJoints) mask |= kVertexJoints | kVertexWeights;
    const std::uint32_t stride = vertexStride(mask);

    layout.topology = surface.topology;
    layout.attributeMask = mask;
    layout.vertexCount = static_cast<std::uint32_t>(count);
    layout.indexCount = static_cast<std::uint32_t>(surface.indices.size());
    layout.indexFormat = count <= 65536 ? prisma::IndexFormat::UInt16 : prisma::IndexFormat::UInt32;
    layout.aabb = computeAabb(surface.positions);
    layout.boneAabbs.clear();
    if (hasJoints) computeBoneAabbs(surface, layout.boneAabbs);

    vertices.resize(count * stride);
    float* cursor = reinterpret_cast<float*>(vertices.data());
    const float zero[3] = { 0.0f, 0.0f, 0.0f };
    const float white[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    for (std::size_t i = 0; i < count; ++i)
    {
        put(cursor, &surface.positions[i].x, 3);
        put(cursor, hasNormals ? &surface.normals[i].x : zero, 3);
        put(cursor, hasColors ? &surface.colors[i].x : white, 4);
        if (hasTangents) put(cursor, &surface.tangents[i].x, 4);
        if (hasUv) put(cursor, &surface.uvs[i].x, 2);
        if (hasUv2) put(cursor, &surface.uvs2[i].x, 2);
        if (hasJoints)
        {
            memcpy(cursor, &surface.joints[i], sizeof(std::uint32_t));
            ++cursor;
            put(cursor, &surface.weights[i].x, 4);
        }
    }

    indices.resize(layout.indexBytes());
    if (layout.indexFormat == prisma::IndexFormat::UInt16)
    {
        std::uint16_t* small = reinterpret_cast<std::uint16_t*>(indices.data());
        for (std::size_t i = 0; i < surface.indices.size(); ++i)
            small[i] = static_cast<std::uint16_t>(surface.indices[i]);
    }
    else if (!surface.indices.empty())
    {
        memcpy(indices.data(), surface.indices.data(), indices.size());
    }
    return true;
}

bool MeshStore::addSurface(MeshId mesh, const SurfaceData& surface, const char* name,
        bool updatableVertices)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info)
    {
        log_error("mesh: invalid mesh");
        return false;
    }

    SurfaceLayout layout;
    ct::Vector<std::uint8_t> vertices;
    ct::Vector<std::uint8_t> indices;
    if (!interleave(surface, layout, vertices, indices)) return false;

    SurfaceInfo result = makeSurface(layout);
    prisma::BufferDesc vertexDesc;
    vertexDesc.usage = vertexUsage(layout.attributeMask);
    vertexDesc.update =
            updatableVertices ? prisma::BufferUpdate::Dynamic : prisma::BufferUpdate::Static;
    vertexDesc.size = layout.vertexBytes();
    vertexDesc.data = vertices.data();
    vertexDesc.debugName = name;
    result.vertexBuffer = mDriver->createBuffer(vertexDesc);
    if (!result.vertexBuffer.valid())
    {
        log_error("mesh: the driver could not create the vertex buffer");
        return false;
    }

    if (result.indexCount > 0)
    {
        prisma::BufferDesc indexDesc;
        indexDesc.usage = prisma::BufferUsage::Index;
        indexDesc.update = prisma::BufferUpdate::Static;
        indexDesc.size = layout.indexBytes();
        indexDesc.data = indices.data();
        indexDesc.indexFormat = layout.indexFormat;
        indexDesc.debugName = name;
        result.indexBuffer = mDriver->createBuffer(indexDesc);
        if (!result.indexBuffer.valid())
        {
            mDriver->destroy(result.vertexBuffer);
            log_error("mesh: the driver could not create the index buffer");
            return false;
        }
    }

    result.ready = true;
    if (!surface.joints.empty()) result.source = surface;
    info->aabb = info->aabb.Union(result.aabb);
    info->surfaces.push_back(static_cast<SurfaceInfo&&>(result));
    ++info->version;
    return true;
}

int MeshStore::beginSurface(MeshId mesh, const SurfaceLayout& layout, const char* name)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info)
    {
        log_error("mesh: invalid mesh");
        return -1;
    }
    if (layout.vertexCount == 0)
    {
        log_error("mesh: a surface needs vertices");
        return -1;
    }

    SurfaceInfo result = makeSurface(layout);
    prisma::BufferDesc vertexDesc;
    vertexDesc.usage = vertexUsage(layout.attributeMask);
    vertexDesc.update = prisma::BufferUpdate::Static;
    vertexDesc.size = layout.vertexBytes();
    vertexDesc.data = nullptr;
    vertexDesc.debugName = name;
    result.vertexBuffer = mDriver->createBuffer(vertexDesc);
    if (!result.vertexBuffer.valid())
    {
        log_error("mesh: the driver could not create the vertex buffer");
        return -1;
    }

    if (layout.indexCount > 0)
    {
        prisma::BufferDesc indexDesc;
        indexDesc.usage = prisma::BufferUsage::Index;
        indexDesc.update = prisma::BufferUpdate::Static;
        indexDesc.size = layout.indexBytes();
        indexDesc.data = nullptr;
        indexDesc.indexFormat = layout.indexFormat;
        indexDesc.debugName = name;
        result.indexBuffer = mDriver->createBuffer(indexDesc);
        if (!result.indexBuffer.valid())
        {
            mDriver->destroy(result.vertexBuffer);
            log_error("mesh: the driver could not create the index buffer");
            return -1;
        }
    }

    result.ready = false;
    info->surfaces.push_back(result);
    ++info->pendingSurfaces;
    return static_cast<int>(info->surfaces.size() - 1);
}

int MeshStore::addIndexedSurface(MeshId mesh, MeshId vertexMesh, std::uint32_t vertexSurface,
        const std::uint16_t* indices, std::uint32_t indexCount, const Math::Box& aabb,
        const char* name)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    const MeshInfo* source = mMeshes.get(toKey(vertexMesh));
    if (!info || !source || vertexSurface >= source->surfaces.size() || !indices || indexCount == 0)
    {
        log_error("mesh: invalid shared surface");
        return -1;
    }
    const SurfaceInfo& vertices = source->surfaces[vertexSurface];
    if (!vertices.vertexBuffer.valid() || !vertices.ready) return -1;

    SurfaceInfo result;
    result.attributeMask = vertices.attributeMask;
    result.vertexStride = vertices.vertexStride;
    result.vertexCount = vertices.vertexCount;
    result.vertexCapacity = vertices.vertexCount;
    result.topology = vertices.topology;
    result.vertexBuffer = vertices.vertexBuffer;
    result.sharedVertices = true;
    result.aabb = aabb;

    prisma::BufferDesc indexDesc;
    indexDesc.usage = prisma::BufferUsage::Index;
    indexDesc.update = prisma::BufferUpdate::Static;
    indexDesc.size = indexCount * 2u;
    indexDesc.data = indices;
    indexDesc.indexFormat = prisma::IndexFormat::UInt16;
    indexDesc.debugName = name;
    result.indexBuffer = mDriver->createBuffer(indexDesc);
    if (!result.indexBuffer.valid())
    {
        log_error("mesh: the driver could not create the index buffer");
        return -1;
    }
    result.indexCount = indexCount;
    result.indexCapacity = indexCount;
    result.indexFormat = prisma::IndexFormat::UInt16;
    result.ready = true;

    info->aabb = info->aabb.Union(result.aabb);
    info->surfaces.push_back(result);
    ++info->version;
    return static_cast<int>(info->surfaces.size() - 1);
}

int MeshStore::addSharedSurface(MeshId mesh, MeshId vertexMesh, std::uint32_t vertexSurface,
        MeshId indexMesh, std::uint32_t indexSurface, const Math::Box& aabb)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    const MeshInfo* vertexSource = mMeshes.get(toKey(vertexMesh));
    const MeshInfo* indexSource = mMeshes.get(toKey(indexMesh));
    if (!info || !vertexSource || !indexSource || vertexSurface >= vertexSource->surfaces.size() ||
            indexSurface >= indexSource->surfaces.size())
    {
        log_error("mesh: invalid shared surface");
        return -1;
    }
    const SurfaceInfo& vertices = vertexSource->surfaces[vertexSurface];
    const SurfaceInfo& indices = indexSource->surfaces[indexSurface];
    if (!vertices.vertexBuffer.valid() || !indices.indexBuffer.valid() || !vertices.ready ||
            !indices.ready)
        return -1;

    SurfaceInfo result;
    result.attributeMask = vertices.attributeMask;
    result.vertexStride = vertices.vertexStride;
    result.vertexCount = vertices.vertexCount;
    result.vertexCapacity = vertices.vertexCount;
    result.topology = vertices.topology;
    result.vertexBuffer = vertices.vertexBuffer;
    result.sharedVertices = true;
    result.indexBuffer = indices.indexBuffer;
    result.indexCount = indices.indexCount;
    result.indexCapacity = indices.indexCount;
    result.indexFormat = indices.indexFormat;
    result.sharedIndices = true;
    result.aabb = aabb;
    result.ready = true;

    info->aabb = info->aabb.Union(result.aabb);
    info->surfaces.push_back(result);
    ++info->version;
    return static_cast<int>(info->surfaces.size() - 1);
}

bool MeshStore::uploadVertices(MeshId mesh, std::uint32_t surface, std::uint32_t offset,
        const void* data, std::uint32_t size)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    SurfaceInfo& target = info->surfaces[surface];
    if (target.ready || !target.vertexBuffer.valid()) return false;
    const std::uint64_t capacity =
            static_cast<std::uint64_t>(target.vertexCount) * target.vertexStride;
    if (static_cast<std::uint64_t>(offset) + size > capacity) return false;
    if (size == 0) return true;
    if (!data) return false;
    mDriver->updateBuffer(target.vertexBuffer, offset, data, size);
    return true;
}

bool MeshStore::uploadIndices(MeshId mesh, std::uint32_t surface, std::uint32_t offset,
        const void* data, std::uint32_t size)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    SurfaceInfo& target = info->surfaces[surface];
    if (target.ready || !target.indexBuffer.valid()) return false;
    const std::uint64_t capacity = static_cast<std::uint64_t>(target.indexCount) *
                                   (target.indexFormat == prisma::IndexFormat::UInt16 ? 2u : 4u);
    if (static_cast<std::uint64_t>(offset) + size > capacity) return false;
    if (size == 0) return true;
    if (!data) return false;
    mDriver->updateBuffer(target.indexBuffer, offset, data, size);
    return true;
}

bool MeshStore::updateVertices(MeshId mesh, std::uint32_t surface, std::uint32_t offset,
        const void* data, std::uint32_t size)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    SurfaceInfo& target = info->surfaces[surface];
    if (!target.ready || target.sharedVertices || !target.vertexBuffer.valid()) return false;
    const std::uint64_t capacity =
            static_cast<std::uint64_t>(target.vertexCount) * target.vertexStride;
    if (static_cast<std::uint64_t>(offset) + size > capacity) return false;
    if (size == 0) return true;
    if (!data) return false;
    mDriver->updateBuffer(target.vertexBuffer, offset, data, size);
    return true;
}

bool MeshStore::setSurfaceAabb(MeshId mesh, std::uint32_t surface, const Math::Box& aabb)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    info->surfaces[surface].aabb = aabb;
    Math::Box total = info->surfaces[0].aabb;
    for (std::size_t i = 1; i < info->surfaces.size(); ++i)
        total = total.Union(info->surfaces[i].aabb);
    info->aabb = total;
    ++info->version;
    return true;
}

bool MeshStore::addSurfaceLod(MeshId mesh, std::uint32_t surface, float edgeLength,
        const void* indices, std::uint32_t indexCount)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    SurfaceInfo& target = info->surfaces[surface];
    if (!target.indexBuffer.valid() || target.dynamic || target.sharedIndices ||
            target.topology != prisma::Topology::Triangles)
        return false;
    if (!indices || indexCount == 0 || indexCount % 3 != 0 || indexCount >= target.indexCount)
        return false;
    if (!(edgeLength > 0.0f)) return false;
    if (!target.lods.empty() && (edgeLength < target.lods.back().edgeLength ||
                                        indexCount >= target.lods.back().indexCount))
        return false;

    prisma::BufferDesc desc;
    desc.usage = prisma::BufferUsage::Index;
    desc.update = prisma::BufferUpdate::Static;
    desc.size = indexCount * (target.indexFormat == prisma::IndexFormat::UInt16 ? 2u : 4u);
    desc.data = nullptr;
    desc.indexFormat = target.indexFormat;
    SurfaceLodInfo lod;
    lod.indexBuffer = mDriver->createBuffer(desc);
    if (!lod.indexBuffer.valid())
    {
        log_error("mesh: the driver could not create the index buffer of a level of detail");
        return false;
    }
    mDriver->updateBuffer(lod.indexBuffer, 0, indices, desc.size);
    lod.indexCount = indexCount;
    lod.edgeLength = edgeLength;
    target.lods.push_back(lod);
    return true;
}

bool MeshStore::finishSurface(MeshId mesh, std::uint32_t surface)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    SurfaceInfo& target = info->surfaces[surface];
    if (target.ready) return false;
    target.ready = true;
    info->aabb = info->aabb.Union(target.aabb);
    if (info->pendingSurfaces > 0) --info->pendingSurfaces;
    ++info->version;
    return true;
}

int MeshStore::addDynamicSurface(MeshId mesh, const SurfaceLayout& capacity, const char* name)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info)
    {
        log_error("mesh: invalid mesh");
        return -1;
    }
    if (capacity.vertexCount == 0)
    {
        log_error("mesh: a dynamic surface needs room for vertices");
        return -1;
    }
    if (capacity.attributeMask & (kVertexJoints | kVertexWeights))
    {
        log_error("mesh: a dynamic surface cannot be skinned");
        return -1;
    }

    SurfaceLayout layout = capacity;
    layout.attributeMask |= kVertexPosition | kVertexNormal | kVertexColor;
    layout.indexFormat =
            layout.vertexCount <= 65536 ? prisma::IndexFormat::UInt16 : prisma::IndexFormat::UInt32;
    layout.aabb = Math::Box();
    layout.boneAabbs.clear();

    SurfaceInfo result = makeSurface(layout);
    result.vertexCapacity = layout.vertexCount;
    result.indexCapacity = layout.indexCount;
    result.vertexCount = 0;
    result.indexCount = 0;
    result.dynamic = true;

    prisma::BufferDesc vertexDesc;
    vertexDesc.usage =
            capacity.storageVertices ? prisma::BufferUsage::Storage : prisma::BufferUsage::Vertex;
    vertexDesc.update = prisma::BufferUpdate::Dynamic;
    vertexDesc.size = layout.vertexBytes();
    vertexDesc.data = nullptr;
    vertexDesc.debugName = name;
    result.vertexBuffer = mDriver->createBuffer(vertexDesc);
    if (!result.vertexBuffer.valid())
    {
        log_error("mesh: the driver could not create the vertex buffer");
        return -1;
    }

    if (layout.indexCount > 0)
    {
        prisma::BufferDesc indexDesc;
        indexDesc.usage = prisma::BufferUsage::Index;
        indexDesc.update = prisma::BufferUpdate::Dynamic;
        indexDesc.size = layout.indexBytes();
        indexDesc.data = nullptr;
        indexDesc.indexFormat = layout.indexFormat;
        indexDesc.debugName = name;
        result.indexBuffer = mDriver->createBuffer(indexDesc);
        if (!result.indexBuffer.valid())
        {
            mDriver->destroy(result.vertexBuffer);
            log_error("mesh: the driver could not create the index buffer");
            return -1;
        }
    }

    result.ready = true;
    info->surfaces.push_back(result);
    ++info->version;
    return static_cast<int>(info->surfaces.size() - 1);
}

bool MeshStore::setDynamicSurfaceRange(MeshId mesh, std::uint32_t surface,
        std::uint32_t vertexCount, std::uint32_t indexCount, const Math::Box& aabb)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    SurfaceInfo& target = info->surfaces[surface];
    if (!target.dynamic) return false;
    if (vertexCount > target.vertexCapacity || indexCount > target.indexCapacity) return false;
    target.vertexCount = vertexCount;
    target.indexCount = indexCount;
    target.aabb = aabb;
    return true;
}

bool MeshStore::updateSurface(MeshId mesh, std::uint32_t surface, const SurfaceData& data)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    SurfaceInfo& target = info->surfaces[surface];
    if (!target.dynamic) return false;

    if (data.positions.empty())
    {
        target.vertexCount = 0;
        target.indexCount = 0;
        target.aabb = Math::Box();
    }
    else
    {
        SurfaceLayout layout;
        if (!interleave(data, layout, mScratchVertices, mScratchIndices)) return false;
        if (layout.attributeMask != target.attributeMask || layout.topology != target.topology)
        {
            log_error("mesh: the update does not match the layout of the dynamic surface");
            return false;
        }
        if (layout.vertexCount > target.vertexCapacity || layout.indexCount > target.indexCapacity)
        {
            log_error("mesh: the update does not fit in the dynamic surface");
            return false;
        }

        mDriver->updateBuffer(target.vertexBuffer, 0, mScratchVertices.data(),
                layout.vertexBytes());
        if (layout.indexCount > 0)
        {
            if (layout.indexFormat == target.indexFormat)
                mDriver->updateBuffer(target.indexBuffer, 0, mScratchIndices.data(),
                        layout.indexBytes());
            else
                mDriver->updateBuffer(target.indexBuffer, 0, data.indices.data(),
                        layout.indexCount * 4u);
        }
        target.vertexCount = layout.vertexCount;
        target.indexCount = layout.indexCount;
        target.aabb = layout.aabb;
    }

    info->aabb = Math::Box();
    for (const SurfaceInfo& each: info->surfaces)
        if (each.ready && each.vertexCount > 0) info->aabb = info->aabb.Union(each.aabb);
    ++info->version;
    return true;
}

bool MeshStore::complete(MeshId mesh) const
{
    const MeshInfo* info = mMeshes.get(toKey(mesh));
    return info && info->pendingSurfaces == 0;
}

bool MeshStore::transform(MeshId mesh, const Math::Mat4& matrix)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info)
    {
        log_error("mesh: invalid mesh");
        return false;
    }
    if (info->pendingSurfaces != 0)
    {
        log_error("mesh: cannot transform a mesh that is still loading");
        return false;
    }
    for (const SurfaceInfo& surface: info->surfaces)
    {
        if (surface.skinned())
        {
            log_error("mesh: a skinned mesh cannot be transformed");
            return false;
        }
    }

    const Math::Mat3 linear = matrix.UpperLeft3x3();
    const float determinant = linear.Determinant();
    if (fabsf(determinant) < 1e-12f)
    {
        log_error("mesh: the transform is not invertible");
        return false;
    }
    const Math::Mat3 normalMatrix = linear.Inverse().Transposed();
    const bool mirrored = determinant < 0.0f;

    struct Pending
    {
        ct::Vector<std::uint8_t> vertices;
        ct::Vector<std::uint8_t> indices;
        ct::Vector<ct::Vector<std::uint8_t>> lods;
        Math::Box aabb;
    };
    const float scaleX = Math::Vec3(matrix[0].x, matrix[0].y, matrix[0].z).Length();
    const float scaleY = Math::Vec3(matrix[1].x, matrix[1].y, matrix[1].z).Length();
    const float scaleZ = Math::Vec3(matrix[2].x, matrix[2].y, matrix[2].z).Length();
    const float lodScale = fmaxf(scaleX, fmaxf(scaleY, scaleZ));
    ct::Vector<Pending> pending;
    pending.resize(info->surfaces.size());

    for (std::size_t s = 0; s < info->surfaces.size(); ++s)
    {
        const SurfaceInfo& surface = info->surfaces[s];
        Pending& work = pending[s];
        const std::uint32_t stride = surface.vertexStride;
        const std::uint32_t vertexBytes = surface.vertexCount * stride;
        work.vertices.resize(vertexBytes);
        if (!mDriver->readBuffer(surface.vertexBuffer, 0, vertexBytes, work.vertices.data()))
        {
            log_error("mesh: could not read back the vertices");
            return false;
        }

        const std::uint32_t normalOffset = attributeOffset(surface.attributeMask, kVertexNormal);
        const bool hasTangents = (surface.attributeMask & kVertexTangent) != 0;
        const std::uint32_t tangentOffset = attributeOffset(surface.attributeMask, kVertexTangent);
        for (std::uint32_t v = 0; v < surface.vertexCount; ++v)
        {
            std::uint8_t* base = work.vertices.data() + static_cast<std::size_t>(v) * stride;
            Math::Vec3 position;
            memcpy(&position, base, sizeof(float) * 3);
            position = matrix.TransformPoint(position);
            memcpy(base, &position, sizeof(float) * 3);
            work.aabb = work.aabb.Expanded(position);

            Math::Vec3 normal;
            memcpy(&normal, base + normalOffset, sizeof(float) * 3);
            normal = normalMatrix * normal;
            const float normalLength =
                    sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            if (normalLength > 0.0f) normal = normal * (1.0f / normalLength);
            memcpy(base + normalOffset, &normal, sizeof(float) * 3);

            if (hasTangents)
            {
                float tangent[4];
                memcpy(tangent, base + tangentOffset, sizeof(tangent));
                Math::Vec3 direction = linear * Math::Vec3(tangent[0], tangent[1], tangent[2]);
                const float length = sqrtf(direction.x * direction.x + direction.y * direction.y +
                                           direction.z * direction.z);
                if (length > 0.0f) direction = direction * (1.0f / length);
                tangent[0] = direction.x;
                tangent[1] = direction.y;
                tangent[2] = direction.z;
                if (mirrored) tangent[3] = -tangent[3];
                memcpy(base + tangentOffset, tangent, sizeof(tangent));
            }
        }

        if (!mirrored || surface.topology != prisma::Topology::Triangles) continue;

        if (surface.indexCount > 0)
        {
            const std::uint32_t indexBytes =
                    surface.indexCount *
                    (surface.indexFormat == prisma::IndexFormat::UInt16 ? 2u : 4u);
            work.indices.resize(indexBytes);
            if (!mDriver->readBuffer(surface.indexBuffer, 0, indexBytes, work.indices.data()))
            {
                log_error("mesh: could not read back the indices");
                return false;
            }
            const std::uint32_t size = surface.indexFormat == prisma::IndexFormat::UInt16 ? 2u : 4u;
            for (std::uint32_t i = 0; i + 2 < surface.indexCount; i += 3)
            {
                std::uint8_t swap[4];
                std::uint8_t* second = work.indices.data() + (i + 1) * size;
                std::uint8_t* third = work.indices.data() + (i + 2) * size;
                memcpy(swap, second, size);
                memcpy(second, third, size);
                memcpy(third, swap, size);
            }
            work.lods.resize(surface.lods.size());
            for (std::size_t l = 0; l < surface.lods.size(); ++l)
            {
                const SurfaceLodInfo& lod = surface.lods[l];
                ct::Vector<std::uint8_t>& bytes = work.lods[l];
                bytes.resize(lod.indexCount * size);
                if (!mDriver->readBuffer(lod.indexBuffer, 0, lod.indexCount * size, bytes.data()))
                {
                    log_error("mesh: could not read back the indices of a level of detail");
                    return false;
                }
                for (std::uint32_t i = 0; i + 2 < lod.indexCount; i += 3)
                {
                    std::uint8_t swap[4];
                    std::uint8_t* second = bytes.data() + (i + 1) * size;
                    std::uint8_t* third = bytes.data() + (i + 2) * size;
                    memcpy(swap, second, size);
                    memcpy(second, third, size);
                    memcpy(third, swap, size);
                }
            }
        }
        else
        {
            for (std::uint32_t i = 0; i + 2 < surface.vertexCount; i += 3)
            {
                std::uint8_t* second =
                        work.vertices.data() + static_cast<std::size_t>(i + 1) * stride;
                std::uint8_t* third =
                        work.vertices.data() + static_cast<std::size_t>(i + 2) * stride;
                for (std::uint32_t b = 0; b < stride; ++b)
                {
                    const std::uint8_t swap = second[b];
                    second[b] = third[b];
                    third[b] = swap;
                }
            }
        }
    }

    info->aabb = Math::Box();
    for (std::size_t s = 0; s < info->surfaces.size(); ++s)
    {
        SurfaceInfo& surface = info->surfaces[s];
        mDriver->updateBuffer(surface.vertexBuffer, 0, pending[s].vertices.data(),
                static_cast<std::uint32_t>(pending[s].vertices.size()));
        if (!pending[s].indices.empty())
            mDriver->updateBuffer(surface.indexBuffer, 0, pending[s].indices.data(),
                    static_cast<std::uint32_t>(pending[s].indices.size()));
        for (std::size_t l = 0; l < surface.lods.size(); ++l)
        {
            if (l < pending[s].lods.size())
                mDriver->updateBuffer(surface.lods[l].indexBuffer, 0, pending[s].lods[l].data(),
                        static_cast<std::uint32_t>(pending[s].lods[l].size()));
            surface.lods[l].edgeLength *= lodScale;
        }
        surface.aabb = pending[s].aabb;
        info->aabb = info->aabb.Union(surface.aabb);
    }
    if (info->collision) info->collision->transform(matrix);
    ++info->version;
    return true;
}

bool MeshStore::setShadowMesh(MeshId mesh, MeshId shadowMesh)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info) return false;
    if (shadowMesh.valid() && (shadowMesh == mesh || !mMeshes.contains(toKey(shadowMesh))))
    {
        log_error("mesh: invalid shadow mesh");
        return false;
    }
    info->shadowMesh = shadowMesh;
    return true;
}

MeshId MeshStore::getShadowMesh(MeshId mesh) const
{
    const MeshInfo* info = get(mesh);
    return info ? info->shadowMesh : MeshId();
}

bool MeshStore::setSurfaceMaterial(MeshId mesh, std::uint32_t surface, MaterialId material)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    info->surfaces[surface].material = material;
    ++info->version;
    return true;
}

bool MeshStore::setSurfaceSource(MeshId mesh, std::uint32_t surface, SurfaceData&& source)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info || surface >= info->surfaces.size()) return false;
    info->surfaces[surface].source = static_cast<SurfaceData&&>(source);
    return true;
}

const SurfaceData* MeshStore::surfaceSource(MeshId mesh, std::uint32_t surface) const
{
    const MeshInfo* info = get(mesh);
    if (!info || surface >= info->surfaces.size()) return nullptr;
    const SurfaceData& source = info->surfaces[surface].source;
    return source.positions.empty() ? nullptr : &source;
}

bool MeshStore::setCollision(MeshId mesh, CollisionMesh* collision)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info)
    {
        delete collision;
        return false;
    }
    if (info->collision != collision) delete info->collision;
    info->collision = collision;
    return true;
}

bool MeshStore::addCollisionSurface(MeshId mesh, const SurfaceData& surface)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info) return false;
    if (surface.topology != prisma::Topology::Triangles)
    {
        if (info->collision) info->collision->append(nullptr, 0, nullptr, 0);
        return false;
    }
    const bool created = info->collision == nullptr;
    if (created) info->collision = new CollisionMesh();
    if (!info->collision->append(surface) || !info->collision->finish())
    {
        if (created)
        {
            delete info->collision;
            info->collision = nullptr;
        }
        return false;
    }
    return true;
}

const CollisionMesh* MeshStore::collision(MeshId mesh) const
{
    const MeshInfo* info = mMeshes.get(toKey(mesh));
    return info ? info->collision : nullptr;
}

void MeshStore::clearSurfaces(MeshId mesh)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (!info) return;
    delete info->collision;
    info->collision = nullptr;
    for (SurfaceInfo& surface: info->surfaces) destroySurfaceBuffers(mDriver, surface);
    info->surfaces.clear();
    info->aabb = Math::Box();
    ++info->version;
}

void MeshStore::destroy(MeshId mesh)
{
    if (!mMeshes.contains(toKey(mesh))) return;
    for (MeshInfo& info: mMeshes)
        if (info.shadowMesh == mesh) info.shadowMesh = MeshId();
    clearSurfaces(mesh);
    mMeshes.erase(toKey(mesh));
}

bool MeshStore::adopt(MeshId target, MeshId donor)
{
    if (target == donor || !valid(target) || !valid(donor)) return false;
    const MeshId oldShadow = mMeshes.get(toKey(target))->shadowMesh;
    if (oldShadow.valid() && oldShadow != donor) destroy(oldShadow);
    MeshInfo* to = mMeshes.get(toKey(target));
    MeshInfo* from = mMeshes.get(toKey(donor));
    delete to->collision;
    for (SurfaceInfo& surface: to->surfaces) destroySurfaceBuffers(mDriver, surface);
    to->surfaces = static_cast<ct::Vector<SurfaceInfo>&&>(from->surfaces);
    to->aabb = from->aabb;
    to->collision = from->collision;
    to->shadowMesh = from->shadowMesh;
    from->surfaces.clear();
    from->collision = nullptr;
    from->shadowMesh = MeshId();
    ++to->version;
    for (MeshInfo& info: mMeshes)
        if (info.shadowMesh == donor) info.shadowMesh = target;
    mMeshes.erase(toKey(donor));
    return true;
}

void MeshStore::clear()
{
    for (MeshInfo& info: mMeshes)
    {
        delete info.collision;
        info.collision = nullptr;
        for (SurfaceInfo& surface: info.surfaces) destroySurfaceBuffers(mDriver, surface);
    }
    mMeshes.clear();
}

bool MeshStore::valid(MeshId mesh) const
{
    return mMeshes.contains(toKey(mesh));
}

MeshId MeshStore::find(const ct::String& path) const
{
    if (path.empty()) return MeshId();
    for (std::size_t i = 0; i < mMeshes.size(); ++i)
        if (mMeshes.items()[i].path == path) return toId(mMeshes.handle_at(i));
    return MeshId();
}

void MeshStore::setPath(MeshId mesh, const ct::String& path)
{
    MeshInfo* info = mMeshes.get(toKey(mesh));
    if (info) info->path = path;
}

const MeshInfo* MeshStore::get(MeshId mesh) const
{
    return mMeshes.get(toKey(mesh));
}

std::size_t MeshStore::count() const
{
    return mMeshes.size();
}

std::uint32_t MeshStore::attributeOffset(std::uint32_t attributeMask, VertexAttributeBit attribute)
{
    std::uint32_t offset = 0;
    for (const VertexAttributeBit bit: kLayoutOrder)
    {
        if (bit == attribute) break;
        if (attributeMask & bit) offset += attributeSize(bit);
    }
    return offset;
}

std::uint32_t MeshStore::vertexStride(std::uint32_t attributeMask)
{
    std::uint32_t stride = 0;
    for (const VertexAttributeBit bit: kLayoutOrder)
        if (attributeMask & bit) stride += attributeSize(bit);
    return stride;
}

Math::Box MeshStore::computeAabb(const ct::Vector<Math::Vec3>& positions)
{
    Math::Box box;
    for (std::size_t i = 0; i < positions.size(); ++i) box = box.Expanded(positions[i]);
    return box;
}

void MeshStore::packIndices(const ct::Vector<std::uint32_t>& indices, prisma::IndexFormat format,
        ct::Vector<std::uint8_t>& bytes)
{
    if (format == prisma::IndexFormat::UInt16)
    {
        bytes.resize(indices.size() * 2);
        for (std::size_t i = 0; i < indices.size(); ++i)
        {
            const std::uint16_t value = static_cast<std::uint16_t>(indices[i]);
            memcpy(bytes.data() + i * 2, &value, 2);
        }
        return;
    }
    bytes.resize(indices.size() * 4);
    if (!indices.empty()) memcpy(bytes.data(), indices.data(), indices.size() * 4);
}

bool MeshStore::generateLods(const SurfaceData& surface, float normalMergeAngle,
        ct::Vector<SurfaceLod>& lods)
{
    lods.clear();
    const std::size_t vertexCount = surface.positions.size();
    const std::size_t indexCount = surface.indices.size();
    if (surface.topology != prisma::Topology::Triangles || vertexCount == 0 || indexCount == 0 ||
            indexCount % 3 != 0)
        return false;
    for (std::size_t i = 0; i < indexCount; ++i)
        if (surface.indices[i] >= vertexCount) return false;

    const Math::Vec3* positions = surface.positions.data();
    const std::uint32_t* indices = surface.indices.data();

    ct::Vector<Math::Vec3> faceNormals;
    const Math::Vec3* normals = surface.normals.data();
    if (surface.normals.size() != vertexCount)
    {
        faceNormals.resize(vertexCount, Math::Vec3(0.0f, 0.0f, 0.0f));
        for (std::size_t i = 0; i < indexCount; i += 3)
        {
            const Math::Vec3& v0 = positions[indices[i + 0]];
            const Math::Vec3& v1 = positions[indices[i + 1]];
            const Math::Vec3& v2 = positions[indices[i + 2]];
            const Math::Vec3 normal = Math::Vec3::Cross(v0 - v2, v0 - v1).NormalizedSafe();
            for (int c = 0; c < 3; ++c)
                faceNormals[indices[i + c]] = faceNormals[indices[i + c]] + normal;
        }
        for (std::size_t i = 0; i < vertexCount; ++i) faceNormals[i].NormalizeSafe();
        normals = faceNormals.data();
    }

    const Math::Vec2* uvs = surface.uvs.size() == vertexCount ? surface.uvs.data() : nullptr;
    const Math::Vec2* uvs2 = surface.uvs2.size() == vertexCount ? surface.uvs2.data() : nullptr;
    const Math::Vec4* tangents =
            surface.tangents.size() == vertexCount ? surface.tangents.data() : nullptr;
    const Math::Vec4* colors =
            surface.colors.size() == vertexCount ? surface.colors.data() : nullptr;
    const bool deformable = !surface.joints.empty();
    const float normalMergeThreshold = cosf(normalMergeAngle * 3.14159265358979323846f / 180.0f);

    std::size_t capacity = 16;
    while (capacity < vertexCount * 2) capacity <<= 1;
    const std::size_t mask = capacity - 1;
    ct::Vector<std::uint32_t> table;
    table.resize(capacity, kNoVertex);

    ct::Vector<std::uint32_t> vertexRemap;
    ct::Vector<std::uint32_t> vertexInverseRemap;
    ct::Vector<std::uint32_t> chainNext;
    ct::Vector<std::uint32_t> chainTail;
    ct::Vector<Math::Vec3> mergedVertices;
    ct::Vector<Math::Vec3> mergedNormals;
    ct::Vector<std::uint32_t> mergedNormalCounts;
    vertexRemap.resize(vertexCount);

    for (std::size_t j = 0; j < vertexCount; ++j)
    {
        const Math::Vec3& v = positions[j];
        const Math::Vec3& n = normals[j];
        std::size_t slot = positionHash(v) & mask;
        while (table[slot] != kNoVertex && !samePosition(mergedVertices[table[slot]], v))
            slot = (slot + 1) & mask;

        bool found = false;
        for (std::uint32_t merged = table[slot]; merged != kNoVertex; merged = chainNext[merged])
        {
            const std::uint32_t other = vertexInverseRemap[merged];
            const bool uvsClose = !uvs || (uvs[j] - uvs[other]).LengthSquared() < kCmpEpsilon2;
            const bool uvs2Close = !uvs2 || (uvs2[j] - uvs2[other]).LengthSquared() < kCmpEpsilon2;
            const bool tangentAligned = !tangents || (tangents[j].w < 0) == (tangents[other].w < 0);
            const bool normalsClose = Math::Vec3::Dot(normals[other], n) > normalMergeThreshold;
            const bool colorClose =
                    !colors || (approximately(colors[j].x, colors[other].x) &&
                                       approximately(colors[j].y, colors[other].y) &&
                                       approximately(colors[j].z, colors[other].z) &&
                                       approximately(colors[j].w, colors[other].w));
            if (uvsClose && uvs2Close && normalsClose && tangentAligned && colorClose)
            {
                vertexRemap[j] = merged;
                mergedNormals[merged] = mergedNormals[merged] + normals[other];
                ++mergedNormalCounts[merged];
                found = true;
                break;
            }
        }
        if (found) continue;

        const std::uint32_t merged = static_cast<std::uint32_t>(mergedVertices.size());
        chainNext.push_back(kNoVertex);
        chainTail.push_back(merged);
        if (table[slot] == kNoVertex) table[slot] = merged;
        else
        {
            chainNext[chainTail[table[slot]]] = merged;
            chainTail[table[slot]] = merged;
        }
        vertexInverseRemap.push_back(static_cast<std::uint32_t>(j));
        mergedVertices.push_back(v);
        vertexRemap[j] = merged;
        mergedNormals.push_back(n);
        mergedNormalCounts.push_back(1);
    }

    ct::Vector<std::uint32_t> mergedIndices;
    mergedIndices.resize(indexCount);
    for (std::size_t j = 0; j < indexCount; ++j) mergedIndices[j] = vertexRemap[indices[j]];

    const std::size_t mergedVertexCount = mergedVertices.size();
    for (std::size_t j = 0; j < mergedVertexCount; ++j)
        mergedNormals[j] = mergedNormals[j] / static_cast<float>(mergedNormalCounts[j]);

    const float scale =
            meshopt_simplifyScale(&mergedVertices[0].x, mergedVertexCount, sizeof(Math::Vec3));

    const std::size_t attributeCount = 6;
    float attributeWeights[attributeCount] = {};
    attributeWeights[0] = attributeWeights[1] = attributeWeights[2] = 1.0f;
    if (colors) attributeWeights[3] = attributeWeights[4] = attributeWeights[5] = 1.0f;

    ct::Vector<float> mergedAttributes;
    mergedAttributes.resize(mergedVertexCount * attributeCount, 0.0f);
    for (std::size_t j = 0; j < mergedVertexCount; ++j)
    {
        mergedAttributes[j * attributeCount + 0] = mergedNormals[j].x;
        mergedAttributes[j * attributeCount + 1] = mergedNormals[j].y;
        mergedAttributes[j * attributeCount + 2] = mergedNormals[j].z;
        if (colors)
        {
            const std::uint32_t original = vertexInverseRemap[j];
            mergedAttributes[j * attributeCount + 3] = colors[original].x;
            mergedAttributes[j * attributeCount + 4] = colors[original].y;
            mergedAttributes[j * attributeCount + 5] = colors[original].z;
        }
    }

    const float maxMeshError = 1.0f;
    const std::size_t minTargetIndices = 12;

    ct::Vector<std::uint32_t> currentIndices = mergedIndices;
    ct::Vector<std::uint32_t> newIndices;
    float currentError = 0.0f;
    bool allowPrune = true;

    while (currentIndices.size() > minTargetIndices * 2)
    {
        const std::size_t currentIndexCount = currentIndices.size();
        std::size_t targetIndexCount = ((currentIndexCount / 3) / 2) * 3;
        if (targetIndexCount < minTargetIndices) targetIndexCount = minTargetIndices;

        newIndices.resize(currentIndexCount);

        unsigned int options = meshopt_SimplifySparse | meshopt_SimplifyLockBorder;
        if (allowPrune) options |= meshopt_SimplifyPrune;
        if (deformable) options |= meshopt_SimplifyRegularize;

        float stepError = 0.0f;
        const std::size_t newIndexCount = meshopt_simplifyWithAttributes(newIndices.data(),
                currentIndices.data(), currentIndexCount, &mergedVertices[0].x, mergedVertexCount,
                sizeof(Math::Vec3), mergedAttributes.data(), sizeof(float) * attributeCount,
                attributeWeights, attributeCount, nullptr, targetIndexCount, maxMeshError, options,
                &stepError);

        if (newIndexCount == 0 && allowPrune)
        {
            allowPrune = false;
            continue;
        }

        currentError = currentError * 1.5f > stepError ? currentError * 1.5f : stepError;

        newIndices.resize(newIndexCount);
        currentIndices = newIndices;

        if (newIndexCount == 0 ||
                static_cast<float>(newIndexCount) >= static_cast<float>(currentIndexCount) * 0.75f)
            break;
        if (currentError > maxMeshError) break;

        SurfaceLod lod;
        lod.edgeLength = currentError * scale > kCmpEpsilon2 ? currentError * scale : kCmpEpsilon2;
        lod.indices.resize(newIndexCount);
        for (std::size_t j = 0; j < newIndexCount; ++j)
            lod.indices[j] = vertexInverseRemap[newIndices[j]];
        lods.push_back(lod);
    }

    ct::stable_sort(lods.begin(), lods.end(),
            [](const SurfaceLod& a, const SurfaceLod& b) { return a.edgeLength < b.edgeLength; });
    return true;
}

bool MeshStore::splitByGrid(const SurfaceData& surface, float cellSize,
        std::uint32_t maxCellsPerAxis, ct::Vector<SurfaceData>& pieces)
{
    pieces.clear();
    const std::size_t vertexCount = surface.positions.size();
    const bool indexed = !surface.indices.empty();
    const std::size_t indexCount = indexed ? surface.indices.size() : vertexCount;
    if (surface.topology != prisma::Topology::Triangles || cellSize <= 0.0f ||
            maxCellsPerAxis == 0 || indexCount < 6 || indexCount % 3 != 0 ||
            !surface.joints.empty() || !surface.weights.empty())
        return false;
    const bool hasNormals = !surface.normals.empty();
    const bool hasTangents = !surface.tangents.empty();
    const bool hasColors = !surface.colors.empty();
    const bool hasUvs = !surface.uvs.empty();
    const bool hasUvs2 = !surface.uvs2.empty();
    if ((hasNormals && surface.normals.size() != vertexCount) ||
            (hasTangents && surface.tangents.size() != vertexCount) ||
            (hasColors && surface.colors.size() != vertexCount) ||
            (hasUvs && surface.uvs.size() != vertexCount) ||
            (hasUvs2 && surface.uvs2.size() != vertexCount))
        return false;
    if (indexed)
        for (std::size_t i = 0; i < indexCount; ++i)
            if (surface.indices[i] >= vertexCount) return false;

    const Math::Box box = computeAabb(surface.positions);
    const Math::Vec3 size = box.Size();
    const float extent[3] = { size.x, size.y, size.z };
    const float origin[3] = { box.min.x, box.min.y, box.min.z };
    std::uint32_t cells[3];
    float step[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        const float wanted = ceilf(extent[axis] / cellSize);
        std::uint32_t count = wanted > 1.0f ? (wanted < static_cast<float>(maxCellsPerAxis)
                                                              ? static_cast<std::uint32_t>(wanted)
                                                              : maxCellsPerAxis)
                                            : 1;
        cells[axis] = count;
        step[axis] = count > 1 ? extent[axis] / static_cast<float>(count) : 0.0f;
    }
    const std::uint32_t cellCount = cells[0] * cells[1] * cells[2];
    if (cellCount < 2) return false;

    const std::uint32_t none = 0xffffffffu;
    const std::size_t triangleCount = indexCount / 3;
    auto corner = [&](std::size_t i)
    { return indexed ? surface.indices[i] : static_cast<std::uint32_t>(i); };
    ct::Vector<std::uint32_t> cellOf;
    cellOf.resize(triangleCount);
    ct::Vector<std::uint32_t> counts;
    counts.resize(cellCount, 0u);
    for (std::size_t t = 0; t < triangleCount; ++t)
    {
        const Math::Vec3 centroid =
                (surface.positions[corner(t * 3)] + surface.positions[corner(t * 3 + 1)] +
                        surface.positions[corner(t * 3 + 2)]) *
                (1.0f / 3.0f);
        const float point[3] = { centroid.x, centroid.y, centroid.z };
        std::uint32_t at[3] = { 0, 0, 0 };
        for (int axis = 0; axis < 3; ++axis)
        {
            if (step[axis] <= 0.0f) continue;
            const float offset = (point[axis] - origin[axis]) / step[axis];
            if (offset <= 0.0f) continue;
            const std::uint32_t index = static_cast<std::uint32_t>(offset);
            at[axis] = index < cells[axis] ? index : cells[axis] - 1;
        }
        const std::uint32_t cell = at[0] + cells[0] * (at[1] + cells[1] * at[2]);
        cellOf[t] = cell;
        ++counts[cell];
    }

    ct::Vector<std::uint32_t> pieceOf;
    pieceOf.resize(cellCount, none);
    ct::Vector<std::uint32_t> starts;
    std::uint32_t running = 0;
    for (std::uint32_t cell = 0; cell < cellCount; ++cell)
    {
        if (counts[cell] == 0) continue;
        pieceOf[cell] = static_cast<std::uint32_t>(starts.size());
        starts.push_back(running);
        running += counts[cell];
    }
    const std::size_t pieceCount = starts.size();
    if (pieceCount < 2) return false;

    ct::Vector<std::uint32_t> order;
    order.resize(triangleCount);
    ct::Vector<std::uint32_t> cursor = starts;
    for (std::size_t t = 0; t < triangleCount; ++t)
        order[cursor[pieceOf[cellOf[t]]]++] = static_cast<std::uint32_t>(t);

    ct::Vector<std::uint32_t> owner;
    owner.resize(vertexCount, none);
    ct::Vector<std::uint32_t> remap;
    remap.resize(vertexCount, 0u);
    pieces.resize(pieceCount);
    for (std::size_t p = 0; p < pieceCount; ++p)
    {
        SurfaceData& piece = pieces[p];
        piece.topology = surface.topology;
        const std::uint32_t first = starts[p];
        const std::uint32_t last = p + 1 < pieceCount ? starts[p + 1] : running;
        piece.indices.reserve(static_cast<std::size_t>(last - first) * 3);
        for (std::uint32_t o = first; o < last; ++o)
        {
            const std::size_t t = order[o];
            for (int c = 0; c < 3; ++c)
            {
                const std::uint32_t vertex = corner(t * 3 + static_cast<std::size_t>(c));
                if (owner[vertex] != p)
                {
                    owner[vertex] = static_cast<std::uint32_t>(p);
                    remap[vertex] = static_cast<std::uint32_t>(piece.positions.size());
                    piece.positions.push_back(surface.positions[vertex]);
                    if (hasNormals) piece.normals.push_back(surface.normals[vertex]);
                    if (hasTangents) piece.tangents.push_back(surface.tangents[vertex]);
                    if (hasColors) piece.colors.push_back(surface.colors[vertex]);
                    if (hasUvs) piece.uvs.push_back(surface.uvs[vertex]);
                    if (hasUvs2) piece.uvs2.push_back(surface.uvs2[vertex]);
                }
                piece.indices.push_back(remap[vertex]);
            }
        }
    }
    return true;
}

bool MeshStore::generateTangents(SurfaceData& surface)
{
    const std::size_t vertexCount = surface.positions.size();
    if (surface.topology != prisma::Topology::Triangles || vertexCount == 0 ||
            surface.normals.size() != vertexCount || surface.uvs.size() != vertexCount)
        return false;

    const bool indexed = !surface.indices.empty();
    const std::size_t cornerCount = indexed ? surface.indices.size() : vertexCount;
    if (cornerCount % 3 != 0) return false;
    for (std::size_t i = 0; i < surface.indices.size(); ++i)
        if (surface.indices[i] >= vertexCount) return false;

    ct::Vector<float> corners;
    corners.resize(cornerCount * 4);
    meshopt_generateTangents(corners.data(), indexed ? surface.indices.data() : nullptr,
            cornerCount, &surface.positions[0].x, vertexCount, sizeof(Math::Vec3),
            &surface.normals[0].x, sizeof(Math::Vec3), &surface.uvs[0].x, sizeof(Math::Vec2), 0);

    surface.tangents.resize(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
        surface.tangents[i] = Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
    for (std::size_t i = 0; i < cornerCount; ++i)
    {
        const std::size_t vertex = indexed ? surface.indices[i] : i;
        surface.tangents[vertex] = Math::Vec4(corners[i * 4 + 0], corners[i * 4 + 1],
                corners[i * 4 + 2], -corners[i * 4 + 3]);
    }
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        Math::Vec4& tangent = surface.tangents[i];
        const Math::Vec3 normal = surface.normals[i];
        const Math::Vec3 axis(tangent.x, tangent.y, tangent.z);
        const Math::Vec3 flat = axis - normal * Math::Vec3::Dot(normal, axis);
        const float length = Math::Vec3::Dot(flat, flat);
        const float normalLength = Math::Vec3::Dot(normal, normal);
        if (length > 0.02f * normalLength && length < 4.0f) continue;
        const float ax = fabsf(normal.x);
        const float ay = fabsf(normal.y);
        const float az = fabsf(normal.z);
        const Math::Vec3 other = ax <= ay && ax <= az ? Math::Vec3(1.0f, 0.0f, 0.0f)
                                 : ay <= az           ? Math::Vec3(0.0f, 1.0f, 0.0f)
                                                      : Math::Vec3(0.0f, 0.0f, 1.0f);
        const Math::Vec3 side = Math::Vec3::Cross(other, normal);
        const float sideLength = sqrtf(Math::Vec3::Dot(side, side));
        const Math::Vec3 fixed =
                sideLength > 0.0f ? side * (1.0f / sideLength) : Math::Vec3(1.0f, 0.0f, 0.0f);
        tangent = Math::Vec4(fixed.x, fixed.y, fixed.z, tangent.w < 0.0f ? -1.0f : 1.0f);
    }
    return true;
}

void MeshStore::computeBoneAabbs(const SurfaceData& surface, ct::Vector<Math::Box>& boneAabbs)
{
    boneAabbs.clear();
    const std::size_t count = surface.positions.size();
    if (surface.joints.size() != count || surface.weights.size() != count) return;
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::uint32_t packed = surface.joints[i];
        const float* weight = &surface.weights[i].x;
        for (unsigned j = 0; j < 4; ++j)
        {
            if (weight[j] == 0.0f) continue;
            const std::size_t bone = (packed >> (8 * j)) & 0xFFu;
            if (bone >= boneAabbs.size()) boneAabbs.resize(bone + 1);
            boneAabbs[bone] = boneAabbs[bone].Expanded(surface.positions[i]);
        }
    }
}

bool SurfaceInfo::skinned() const
{
    return (attributeMask & kVertexJoints) != 0;
}

} // namespace prisma3d
