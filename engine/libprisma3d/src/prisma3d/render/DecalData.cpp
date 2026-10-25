#include "prisma3d/render/DecalData.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

struct SortEntry
{
    float depth;
    std::uint32_t instance;
};

Math::Vec3 axis(const Math::Vec4& column)
{
    return Math::Vec3(column.x, column.y, column.z);
}

float srgbToLinear(float value)
{
    return value < 0.04045f ? value * (1.0f / 12.92f)
                            : powf((value + 0.055f) * (1.0f / 1.055f), 2.4f);
}

bool isZero(const Math::Vec4& rect)
{
    return rect.x == 0.0f && rect.y == 0.0f && rect.z == 0.0f && rect.w == 0.0f;
}

void storeRect(const Math::Vec4& rect, float* out)
{
    out[0] = rect.x;
    out[1] = rect.y;
    out[2] = rect.z;
    out[3] = rect.w;
}

void storeColumn(const Math::Vec3& column, float* out)
{
    out[0] = column.x;
    out[1] = column.y;
    out[2] = column.z;
    out[3] = 0.0f;
}

} // namespace

std::uint32_t buildDecalData(const DecalInstanceInput* instances, std::uint32_t instanceCount,
        const Math::Mat4& cameraTransform, std::uint32_t maxDecals, ct::Vector<DecalDataGpu>& data,
        ct::Vector<DecalSorted>& sorted)
{
    static_assert(sizeof(DecalDataGpu) == 240, "decal data layout");

    data.clear();
    sorted.clear();

    const Math::Vec3 cameraOrigin = axis(cameraTransform.col3);

    ct::Vector<SortEntry> order;
    for (std::uint32_t i = 0; i < instanceCount; ++i)
    {
        if (order.size() == maxDecals) break;
        const DecalInstanceInput& input = instances[i];
        if (!input.decal) continue;

        const float distance = (cameraOrigin - axis(input.transform.col3)).Length();
        if (input.decal->distanceFade &&
                distance > input.decal->distanceFadeBegin + input.decal->distanceFadeLength)
            continue;

        SortEntry entry;
        entry.depth = distance - input.sortingOffset;
        entry.instance = i;
        order.push_back(entry);
    }

    for (std::size_t i = 1; i < order.size(); ++i)
    {
        const SortEntry moving = order[i];
        std::size_t slot = i;
        while (slot > 0 && moving.depth < order[slot - 1].depth)
        {
            order[slot] = order[slot - 1];
            --slot;
        }
        order[slot] = moving;
    }

    const Math::Mat4 cameraInverse = cameraTransform.InverseAffine();
    const Math::Vec3 cameraX = axis(cameraInverse.col0);
    const Math::Vec3 cameraY = axis(cameraInverse.col1);
    const Math::Vec3 cameraZ = axis(cameraInverse.col2);

    for (std::size_t i = 0; i < order.size(); ++i)
    {
        const DecalInstanceInput& input = instances[order[i].instance];
        const DecalParams& decal = *input.decal;

        float fade = 1.0f;
        if (decal.distanceFade)
        {
            const float distance = order[i].depth + input.sortingOffset;
            if (distance > decal.distanceFadeBegin)
            {
                float t = 1.0f - (distance - decal.distanceFadeBegin) / decal.distanceFadeLength;
                t = fminf(fmaxf(t, 0.0f), 1.0f);
                fade = t * t * (3.0f - 2.0f * t);
            }
        }

        DecalDataGpu dd;
        memset(&dd, 0, sizeof(dd));

        const Math::Vec3 extents = decal.size * 0.5f;

        DecalSorted entry;
        entry.instance = order[i].instance;
        entry.transform = input.transform;
        entry.extents = extents;
        sorted.push_back(entry);

        const Math::Vec4& albedoRect = input.rects[static_cast<std::size_t>(DecalTexture::Albedo)];
        const Math::Vec4& normalRect = input.rects[static_cast<std::size_t>(DecalTexture::Normal)];
        const Math::Vec4& ormRect = input.rects[static_cast<std::size_t>(DecalTexture::Orm)];
        const Math::Vec4& emissionRect =
                input.rects[static_cast<std::size_t>(DecalTexture::Emission)];

        if (isZero(albedoRect) && isZero(emissionRect))
        {
            data.push_back(dd);
            continue;
        }

        Math::Mat4 local = Math::Mat4::Identity();
        local.col0 = Math::Vec4(2.0f * extents.x, 0.0f, 0.0f, 0.0f);
        local.col1 = Math::Vec4(0.0f, extents.y, 0.0f, 0.0f);
        local.col2 = Math::Vec4(0.0f, 0.0f, 2.0f * extents.z, 0.0f);
        local.col3 = Math::Vec4(-extents.x, 0.0f, -extents.z, 1.0f);

        const Math::Mat4 toDecal = (cameraInverse * input.transform * local).InverseAffine();
        const Math::Vec4 columns[4] = { toDecal.col0, toDecal.col1, toDecal.col2, toDecal.col3 };
        for (int column = 0; column < 4; ++column)
        {
            dd.xform[column * 4 + 0] = columns[column].x;
            dd.xform[column * 4 + 1] = columns[column].y;
            dd.xform[column * 4 + 2] = columns[column].z;
            dd.xform[column * 4 + 3] = column == 3 ? 1.0f : 0.0f;
        }

        const Math::Vec3 up = axis(input.transform.col1).Normalized();
        const Math::Vec3 normal = cameraX * up.x + cameraY * up.y + cameraZ * up.z;
        dd.normal[0] = normal.x;
        dd.normal[1] = normal.y;
        dd.normal[2] = normal.z;
        dd.normalFade = decal.normalFade;

        storeRect(albedoRect, dd.albedoRect);
        storeRect(normalRect, dd.normalRect);
        storeRect(ormRect, dd.ormRect);
        storeRect(emissionRect, dd.emissionRect);

        if (!isZero(normalRect))
        {
            Math::Vec3 x = axis(input.transform.col0).Normalized();
            Math::Vec3 y = axis(input.transform.col1);
            Math::Vec3 z = axis(input.transform.col2);
            y = (y - x * Math::Vec3::Dot(x, y)).Normalized();
            z = (z - x * Math::Vec3::Dot(x, z) - y * Math::Vec3::Dot(y, z)).Normalized();
            storeColumn(cameraX * x.x + cameraY * x.y + cameraZ * x.z, dd.normalXform);
            storeColumn(cameraX * y.x + cameraY * y.y + cameraZ * y.z, dd.normalXform + 4);
            storeColumn(cameraX * z.x + cameraY * z.y + cameraZ * z.z, dd.normalXform + 8);
        }

        dd.modulate[0] = srgbToLinear(decal.modulate.x);
        dd.modulate[1] = srgbToLinear(decal.modulate.y);
        dd.modulate[2] = srgbToLinear(decal.modulate.z);
        dd.modulate[3] = decal.modulate.w * fade;
        dd.emissionEnergy = decal.emissionEnergy * fade;
        dd.albedoMix = decal.albedoMix;
        dd.mask = decal.cullMask;
        dd.upperFade = decal.upperFade;
        dd.lowerFade = decal.lowerFade;

        data.push_back(dd);
    }

    return static_cast<std::uint32_t>(data.size());
}

} // namespace prisma3d
