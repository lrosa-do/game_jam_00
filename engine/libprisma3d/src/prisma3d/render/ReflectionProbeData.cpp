#include "prisma3d/render/ReflectionProbeData.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

struct SortEntry
{
    float size;
    std::uint32_t instance;
};

float srgbToLinear(float value)
{
    return value < 0.04045f ? value * (1.0f / 12.92f)
                            : powf((value + 0.055f) * (1.0f / 1.055f), 2.4f);
}

} // namespace

std::uint32_t buildReflectionData(const ReflectionInstanceInput* instances,
        std::uint32_t instanceCount, const Math::Mat4& cameraTransform, float cameraExposure,
        std::uint32_t maxReflections, ct::Vector<ReflectionDataGpu>& data,
        ct::Vector<ReflectionSorted>& sorted)
{
    static_assert(sizeof(ReflectionDataGpu) == 144, "reflection data layout");

    data.clear();
    sorted.clear();

    ct::Vector<SortEntry> order;
    for (std::uint32_t i = 0; i < instanceCount; ++i)
    {
        if (order.size() == maxReflections) break;
        if (!instances[i].probe) continue;
        const Math::Vec3 extents = instances[i].probe->size * 0.5f;
        SortEntry entry;
        entry.size = -extents.Length();
        entry.instance = i;
        order.push_back(entry);
    }

    for (std::size_t i = 1; i < order.size(); ++i)
    {
        const SortEntry moving = order[i];
        std::size_t slot = i;
        while (slot > 0 && moving.size < order[slot - 1].size)
        {
            order[slot] = order[slot - 1];
            --slot;
        }
        order[slot] = moving;
    }

    for (std::size_t i = 0; i < order.size(); ++i)
    {
        const ReflectionInstanceInput& input = instances[order[i].instance];
        const ReflectionProbeParams& probe = *input.probe;
        const Math::Vec3 extents = probe.size * 0.5f;

        ReflectionDataGpu rd;
        memset(&rd, 0, sizeof(rd));
        rd.boxExtents[0] = extents.x;
        rd.boxExtents[1] = extents.y;
        rd.boxExtents[2] = extents.z;
        rd.index = static_cast<float>(input.atlasIndex);
        rd.boxOffset[0] = probe.originOffset.x;
        rd.boxOffset[1] = probe.originOffset.y;
        rd.boxOffset[2] = probe.originOffset.z;
        rd.mask = probe.reflectionMask;
        rd.intensity = probe.intensity;
        rd.blendDistance = probe.blendDistance;
        rd.ambientMode = static_cast<std::uint32_t>(probe.ambientMode);
        rd.exterior = probe.interior ? 0u : 1u;
        rd.boxProject = probe.boxProjection ? 1u : 0u;
        rd.exposureNormalization =
                probe.bakedExposure > 0.0f ? cameraExposure / probe.bakedExposure : 1.0f;
        rd.ambient[0] = srgbToLinear(probe.ambientColor.x) * probe.ambientEnergy;
        rd.ambient[1] = srgbToLinear(probe.ambientColor.y) * probe.ambientEnergy;
        rd.ambient[2] = srgbToLinear(probe.ambientColor.z) * probe.ambientEnergy;

        const Math::Mat4 local =
                (cameraTransform.InverseAffine() * input.transform).InverseAffine();
        const Math::Vec4 columns[4] = { local.col0, local.col1, local.col2, local.col3 };
        for (int column = 0; column < 4; ++column)
        {
            rd.localMatrix[column * 4 + 0] = columns[column].x;
            rd.localMatrix[column * 4 + 1] = columns[column].y;
            rd.localMatrix[column * 4 + 2] = columns[column].z;
            rd.localMatrix[column * 4 + 3] = column == 3 ? 1.0f : 0.0f;
        }
        data.push_back(rd);

        ReflectionSorted entry;
        entry.instance = order[i].instance;
        entry.transform = input.transform;
        entry.extents = extents;
        sorted.push_back(entry);
    }

    return static_cast<std::uint32_t>(data.size());
}

} // namespace prisma3d
