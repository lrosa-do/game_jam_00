#include "MultiMeshStorage.h"

#include "prisma3d/core/BoundsTree.h"

#include "platform.h"

#include <string.h>

namespace prisma3d
{

namespace
{

typedef ct::Handle32<MultiMeshData> SlotKey;

SlotKey toKey(MultiMeshId id)
{
    return SlotKey::from_bits(id.bits());
}

MultiMeshId toId(SlotKey key)
{
    return MultiMeshId::from_bits(key.bits());
}

std::uint32_t divisionRoundUp(std::uint32_t value, std::uint32_t divisor)
{
    return (value + divisor - 1) / divisor;
}

bool sameBox(const Math::Box& a, const Math::Box& b)
{
    return a.min.x == b.min.x && a.min.y == b.min.y && a.min.z == b.min.z && a.max.x == b.max.x &&
           a.max.y == b.max.y && a.max.z == b.max.z;
}

void readTransform(const float* values, Math::Mat4& transform)
{
    transform = Math::Mat4::Identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) transform[c][r] = values[r * 4 + c];
}

void writeTransform(const Math::Mat4& transform, float* values)
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) values[r * 4 + c] = transform[c][r];
}

} // namespace

MultiMeshStorage::MultiMeshStorage()
        : mDriver(nullptr),
          mMeshes(nullptr),
          mUploadedBytes(0)
{
}

void MultiMeshStorage::setup(prisma::Driver* driver, MeshStore* meshes)
{
    mDriver = driver;
    mMeshes = meshes;
}

void MultiMeshStorage::destroyAll()
{
    for (MultiMeshData& data: mItems) freeBuffer(data);
    mItems.clear();
    mDirtyList.clear();
}

MultiMeshData* MultiMeshStorage::find(MultiMeshId multimesh)
{
    return mItems.get(toKey(multimesh));
}

const MultiMeshData* MultiMeshStorage::find(MultiMeshId multimesh) const
{
    return mItems.get(toKey(multimesh));
}

MultiMeshId MultiMeshStorage::create()
{
    const MultiMeshId id = toId(mItems.insert(MultiMeshData()));
    if (MultiMeshData* data = find(id)) data->self = id;
    return id;
}

void MultiMeshStorage::free(MultiMeshId multimesh)
{
    if (!find(multimesh)) return;
    updateDirty();
    if (MultiMeshData* data = find(multimesh)) freeBuffer(*data);
    mItems.erase(toKey(multimesh));
}

bool MultiMeshStorage::valid(MultiMeshId multimesh) const
{
    return find(multimesh) != nullptr;
}

const MultiMeshData* MultiMeshStorage::get(MultiMeshId multimesh) const
{
    return find(multimesh);
}

std::size_t MultiMeshStorage::count() const
{
    return mItems.size();
}

void MultiMeshStorage::freeBuffer(MultiMeshData& data)
{
    if (data.buffer.valid() && mDriver) mDriver->destroy(data.buffer);
    data.buffer = prisma::BufferHandle();
}

void MultiMeshStorage::addToDirtyList(MultiMeshData& data, MultiMeshId id)
{
    if (data.dirty) return;
    data.dirty = true;
    mDirtyList.push_back(id);
}

void MultiMeshStorage::allocateData(MultiMeshId multimesh, std::uint32_t instances, bool useColors,
        bool useCustomData)
{
    MultiMeshData* found = find(multimesh);
    if (!found) return;
    MultiMeshData& data = *found;

    if (data.instances == instances && data.usesColors == useColors &&
            data.usesCustomData == useCustomData)
        return;

    freeBuffer(data);

    data.dirtyRegions.clear();
    data.dirtyRegionCount = 0;
    data.instances = instances;
    data.usesColors = useColors;
    data.colorOffsetCache = kMultiMeshTransformFloats;
    data.usesCustomData = useCustomData;
    data.customDataOffsetCache = data.colorOffsetCache + (useColors ? 4 : 0);
    data.strideCache = data.customDataOffsetCache + (useCustomData ? 4 : 0);
    data.bufferSet = false;
    data.dataCache.clear();
    data.aabb = Math::Box();
    data.aabbDirty = false;
    if (data.visibleInstances > static_cast<int>(data.instances))
        data.visibleInstances = static_cast<int>(data.instances);
    ++data.version;

    if (data.instances > 0 && mDriver)
    {
        ct::Vector<float> zeros;
        zeros.resize(static_cast<std::size_t>(data.instances) * data.strideCache);
        memset(zeros.data(), 0, zeros.size() * sizeof(float));
        prisma::BufferDesc desc;
        desc.usage = prisma::BufferUsage::Storage;
        desc.update = prisma::BufferUpdate::Dynamic;
        desc.size = static_cast<std::uint32_t>(zeros.size() * sizeof(float));
        desc.data = zeros.data();
        desc.debugName = "multimesh";
        data.buffer = mDriver->createBuffer(desc);
        if (!data.buffer.valid()) log_error("multimesh: could not create the instance buffer");
    }
}

std::uint32_t MultiMeshStorage::getInstanceCount(MultiMeshId multimesh) const
{
    const MultiMeshData* data = find(multimesh);
    return data ? data->instances : 0;
}

void MultiMeshStorage::noteMeshDependency(MultiMeshData& data)
{
    const MeshInfo* mesh = mMeshes ? mMeshes->get(data.mesh) : nullptr;
    data.aabbMesh = data.mesh;
    data.aabbMeshVersion = mesh ? mesh->version : 0;
}

void MultiMeshStorage::setMesh(MultiMeshId multimesh, MeshId mesh)
{
    MultiMeshData* found = find(multimesh);
    if (!found) return;
    MultiMeshData& data = *found;
    if (data.mesh == mesh) return;
    data.mesh = mesh;
    ++data.version;

    if (data.instances == 0)
    {
        noteMeshDependency(data);
        return;
    }

    if (!data.dataCache.empty()) markAllDirty(data, multimesh, false, true);
    else if (data.instances > 0)
        recreateAabbFromBuffer(data);
    noteMeshDependency(data);
}

MeshId MultiMeshStorage::getMesh(MultiMeshId multimesh) const
{
    const MultiMeshData* data = find(multimesh);
    return data ? data->mesh : MeshId();
}

void MultiMeshStorage::makeLocal(MultiMeshData& data)
{
    if (!data.dataCache.empty()) return;

    const std::size_t total = static_cast<std::size_t>(data.instances) * data.strideCache;
    data.dataCache.resize(total);
    if (data.bufferSet && data.buffer.valid() && mDriver)
    {
        if (!mDriver->readBuffer(data.buffer, 0, static_cast<std::uint32_t>(total * sizeof(float)),
                    data.dataCache.data()))
        {
            log_error("multimesh: could not read back the instance buffer");
            memset(data.dataCache.data(), 0, total * sizeof(float));
        }
    }
    else
        memset(data.dataCache.data(), 0, total * sizeof(float));

    const std::uint32_t regions = divisionRoundUp(data.instances, kMultiMeshDirtyRegionSize);
    data.dirtyRegions.resize(regions);
    memset(data.dirtyRegions.data(), 0, regions);
    data.dirtyRegionCount = 0;
}

void MultiMeshStorage::markDirty(MultiMeshData& data, MultiMeshId id, int index, bool aabb)
{
    const std::uint32_t region = static_cast<std::uint32_t>(index) / kMultiMeshDirtyRegionSize;
    if (region >= data.dirtyRegions.size()) return;
    if (!data.dirtyRegions[region])
    {
        data.dirtyRegions[region] = 1;
        ++data.dirtyRegionCount;
    }
    if (aabb) data.aabbDirty = true;
    addToDirtyList(data, id);
}

void MultiMeshStorage::markAllDirty(MultiMeshData& data, MultiMeshId id, bool dataRegions,
        bool aabb)
{
    if (dataRegions)
    {
        const std::uint32_t regions = divisionRoundUp(data.instances, kMultiMeshDirtyRegionSize);
        for (std::uint32_t i = 0; i < regions && i < data.dirtyRegions.size(); ++i)
        {
            if (!data.dirtyRegions[i])
            {
                data.dirtyRegions[i] = 1;
                ++data.dirtyRegionCount;
            }
        }
    }
    if (aabb) data.aabbDirty = true;
    addToDirtyList(data, id);
}

void MultiMeshStorage::recreateAabb(MultiMeshData& data, const float* values, int instances)
{
    const MeshInfo* mesh = mMeshes ? mMeshes->get(data.mesh) : nullptr;
    if (!mesh) return;
    if (!emptyBox(data.customAabb)) return;

    Math::Box result;
    if (!emptyBox(mesh->aabb))
    {
        for (int i = 0; i < instances; ++i)
        {
            Math::Mat4 transform;
            readTransform(values + static_cast<std::size_t>(data.strideCache) * i, transform);
            result = result.Union(mesh->aabb.Transformed(transform));
        }
    }
    if (!sameBox(result, data.aabb))
    {
        data.aabb = result;
        ++data.version;
    }
}

void MultiMeshStorage::recreateAabbFromBuffer(MultiMeshData& data)
{
    if (!data.bufferSet || !data.buffer.valid() || !mDriver) return;
    ct::Vector<float> values;
    values.resize(static_cast<std::size_t>(data.instances) * data.strideCache);
    if (!mDriver->readBuffer(data.buffer, 0,
                static_cast<std::uint32_t>(values.size() * sizeof(float)), values.data()))
        return;
    recreateAabb(data, values.data(), static_cast<int>(data.instances));
}

void MultiMeshStorage::instanceSetTransform(MultiMeshId multimesh, std::uint32_t index,
        const Math::Mat4& transform)
{
    MultiMeshData* found = find(multimesh);
    if (!found || index >= found->instances) return;
    MultiMeshData& data = *found;

    makeLocal(data);
    writeTransform(transform,
            data.dataCache.data() + static_cast<std::size_t>(index) * data.strideCache);
    markDirty(data, multimesh, static_cast<int>(index), true);
}

void MultiMeshStorage::instanceSetColor(MultiMeshId multimesh, std::uint32_t index,
        const Math::Vec4& color)
{
    MultiMeshData* found = find(multimesh);
    if (!found || index >= found->instances || !found->usesColors) return;
    MultiMeshData& data = *found;

    makeLocal(data);
    float* values = data.dataCache.data() + static_cast<std::size_t>(index) * data.strideCache +
                    data.colorOffsetCache;
    values[0] = color.x;
    values[1] = color.y;
    values[2] = color.z;
    values[3] = color.w;
    markDirty(data, multimesh, static_cast<int>(index), false);
}

void MultiMeshStorage::instanceSetCustomData(MultiMeshId multimesh, std::uint32_t index,
        const Math::Vec4& custom)
{
    MultiMeshData* found = find(multimesh);
    if (!found || index >= found->instances || !found->usesCustomData) return;
    MultiMeshData& data = *found;

    makeLocal(data);
    float* values = data.dataCache.data() + static_cast<std::size_t>(index) * data.strideCache +
                    data.customDataOffsetCache;
    values[0] = custom.x;
    values[1] = custom.y;
    values[2] = custom.z;
    values[3] = custom.w;
    markDirty(data, multimesh, static_cast<int>(index), false);
}

Math::Mat4 MultiMeshStorage::instanceGetTransform(MultiMeshId multimesh, std::uint32_t index)
{
    MultiMeshData* found = find(multimesh);
    if (!found || index >= found->instances) return Math::Mat4::Identity();
    makeLocal(*found);
    Math::Mat4 transform;
    readTransform(found->dataCache.data() + static_cast<std::size_t>(index) * found->strideCache,
            transform);
    return transform;
}

Math::Vec4 MultiMeshStorage::instanceGetColor(MultiMeshId multimesh, std::uint32_t index)
{
    MultiMeshData* found = find(multimesh);
    if (!found || index >= found->instances || !found->usesColors) return Math::Vec4();
    makeLocal(*found);
    const float* values = found->dataCache.data() +
                          static_cast<std::size_t>(index) * found->strideCache +
                          found->colorOffsetCache;
    return Math::Vec4(values[0], values[1], values[2], values[3]);
}

Math::Vec4 MultiMeshStorage::instanceGetCustomData(MultiMeshId multimesh, std::uint32_t index)
{
    MultiMeshData* found = find(multimesh);
    if (!found || index >= found->instances || !found->usesCustomData) return Math::Vec4();
    makeLocal(*found);
    const float* values = found->dataCache.data() +
                          static_cast<std::size_t>(index) * found->strideCache +
                          found->customDataOffsetCache;
    return Math::Vec4(values[0], values[1], values[2], values[3]);
}

bool MultiMeshStorage::setBuffer(MultiMeshId multimesh, const float* values, std::size_t count)
{
    MultiMeshData* found = find(multimesh);
    if (!found) return false;
    MultiMeshData& data = *found;
    if (count != static_cast<std::size_t>(data.instances) * data.strideCache)
    {
        log_error("multimesh: the buffer size does not match the instance count and layout");
        return false;
    }
    if (data.instances == 0 || !data.buffer.valid()) return true;

    mDriver->updateBuffer(data.buffer, 0, values,
            static_cast<std::uint32_t>(count * sizeof(float)));
    mUploadedBytes += static_cast<std::uint32_t>(count * sizeof(float));
    data.bufferSet = true;

    if (!data.dataCache.empty())
    {
        memcpy(data.dataCache.data(), values, count * sizeof(float));
        markAllDirty(data, multimesh, true, true);
    }
    else if (mMeshes && mMeshes->valid(data.mesh))
    {
        if (emptyBox(data.customAabb))
        {
            recreateAabb(data, values, static_cast<int>(data.instances));
            noteMeshDependency(data);
        }
    }
    return true;
}

bool MultiMeshStorage::getBuffer(MultiMeshId multimesh, ct::Vector<float>& out)
{
    MultiMeshData* found = find(multimesh);
    out.clear();
    if (!found || !found->buffer.valid()) return false;
    const std::size_t total = static_cast<std::size_t>(found->instances) * found->strideCache;
    out.resize(total);
    if (!found->dataCache.empty())
        memcpy(out.data(), found->dataCache.data(), total * sizeof(float));
    else if (found->bufferSet)
    {
        if (!mDriver->readBuffer(found->buffer, 0,
                    static_cast<std::uint32_t>(total * sizeof(float)), out.data()))
            return false;
    }
    else
        memset(out.data(), 0, total * sizeof(float));
    return true;
}

void MultiMeshStorage::setVisibleInstances(MultiMeshId multimesh, int visible)
{
    MultiMeshData* found = find(multimesh);
    if (!found) return;
    MultiMeshData& data = *found;
    if (visible < -1 || visible > static_cast<int>(data.instances)) return;
    if (data.visibleInstances == visible) return;

    if (!data.dataCache.empty())
    {
        markAllDirty(data, multimesh, false, true);
        const int start = data.visibleInstances >= 0 ? data.visibleInstances
                                                     : static_cast<int>(data.instances);
        for (int i = start; i < visible; ++i) markDirty(data, multimesh, i, true);
    }

    data.visibleInstances = visible;
}

int MultiMeshStorage::getVisibleInstances(MultiMeshId multimesh) const
{
    const MultiMeshData* data = find(multimesh);
    return data ? data->visibleInstances : 0;
}

void MultiMeshStorage::setDissolve(MultiMeshId multimesh, bool enable)
{
    MultiMeshData* found = find(multimesh);
    if (found) found->dissolve = enable;
}

bool MultiMeshStorage::getDissolve(MultiMeshId multimesh) const
{
    const MultiMeshData* data = find(multimesh);
    return data && data->dissolve;
}

void MultiMeshStorage::setCustomAabb(MultiMeshId multimesh, const Math::Box& aabb)
{
    MultiMeshData* found = find(multimesh);
    if (!found) return;
    MultiMeshData& data = *found;
    const bool cleared = !emptyBox(data.customAabb) && emptyBox(aabb);
    data.customAabb = aabb;
    ++data.version;
    if (cleared)
    {
        if (!data.dataCache.empty()) markAllDirty(data, multimesh, false, true);
        else
            recreateAabbFromBuffer(data);
    }
}

Math::Box MultiMeshStorage::getCustomAabb(MultiMeshId multimesh) const
{
    const MultiMeshData* data = find(multimesh);
    return data ? data->customAabb : Math::Box();
}

Math::Box MultiMeshStorage::getAabb(MultiMeshId multimesh)
{
    MultiMeshData* found = find(multimesh);
    if (!found) return Math::Box();
    if (!emptyBox(found->customAabb)) return found->customAabb;
    if (found->aabbDirty) updateDirty();
    found = find(multimesh);
    return found ? found->aabb : Math::Box();
}

std::uint32_t MultiMeshStorage::getVersion(MultiMeshId multimesh) const
{
    const MultiMeshData* data = find(multimesh);
    return data ? data->version : 0;
}

std::uint32_t MultiMeshStorage::instancesToDraw(const MultiMeshData& data)
{
    if (data.visibleInstances >= 0) return static_cast<std::uint32_t>(data.visibleInstances);
    return data.instances;
}

Math::Box MultiMeshStorage::effectiveAabb(const MultiMeshData& data)
{
    return emptyBox(data.customAabb) ? data.aabb : data.customAabb;
}

std::uint32_t MultiMeshStorage::takeUploadedBytes()
{
    const std::uint32_t bytes = mUploadedBytes;
    mUploadedBytes = 0;
    return bytes;
}

void MultiMeshStorage::updateDirty()
{
    if (mMeshes)
    {
        for (MultiMeshData& data: mItems)
        {
            const MeshInfo* mesh = mMeshes->get(data.mesh);
            if (!mesh || (data.aabbMesh == data.mesh && data.aabbMeshVersion == mesh->version))
                continue;
            data.aabbMesh = data.mesh;
            data.aabbMeshVersion = mesh->version;
            if (data.instances == 0) continue;
            if (!data.dataCache.empty())
            {
                addToDirtyList(data, data.self);
                data.aabbDirty = true;
            }
            else
                recreateAabbFromBuffer(data);
        }
    }

    for (std::size_t n = 0; n < mDirtyList.size(); ++n)
    {
        MultiMeshData* found = find(mDirtyList[n]);
        if (!found) continue;
        MultiMeshData& data = *found;

        if (!data.dataCache.empty())
        {
            const std::uint32_t visible = static_cast<std::uint32_t>(
                    data.visibleInstances >= 0 ? data.visibleInstances : data.instances);
            const float* values = data.dataCache.data();

            if (data.dirtyRegionCount != 0 && data.buffer.valid())
            {
                const std::uint32_t visibleRegions =
                        visible == 0 ? 0 : divisionRoundUp(visible, kMultiMeshDirtyRegionSize);
                const std::uint32_t regionSize =
                        data.strideCache * kMultiMeshDirtyRegionSize * sizeof(float);
                const std::uint32_t size = data.strideCache * data.instances * sizeof(float);
                if (data.dirtyRegionCount > 32 || data.dirtyRegionCount > visibleRegions / 2)
                {
                    const std::uint32_t bytes =
                            visibleRegions * regionSize < size ? visibleRegions * regionSize : size;
                    if (bytes > 0)
                    {
                        mDriver->updateBuffer(data.buffer, 0, values, bytes);
                        mUploadedBytes += bytes;
                    }
                }
                else
                {
                    for (std::uint32_t i = 0; i < visibleRegions; ++i)
                    {
                        if (!data.dirtyRegions[i]) continue;
                        const std::uint32_t offset = i * regionSize;
                        const std::uint32_t start =
                                data.strideCache * kMultiMeshDirtyRegionSize * i;
                        const std::uint32_t bytes =
                                regionSize < size - offset ? regionSize : size - offset;
                        mDriver->updateBuffer(data.buffer, offset, &values[start], bytes);
                        mUploadedBytes += bytes;
                    }
                }
            }
            if (data.dirtyRegionCount != 0)
            {
                memset(data.dirtyRegions.data(), 0, data.dirtyRegions.size());
                data.dirtyRegionCount = 0;
            }

            if (data.aabbDirty)
            {
                data.aabbDirty = false;
                if (emptyBox(data.customAabb))
                {
                    recreateAabb(data, values, static_cast<int>(visible));
                    noteMeshDependency(data);
                }
            }
        }

        data.dirty = false;
    }
    mDirtyList.clear();
}

} // namespace prisma3d
