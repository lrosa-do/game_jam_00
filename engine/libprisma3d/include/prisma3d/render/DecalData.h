#pragma once

#include "prisma3d/render/TextureStore.h"

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

enum class DecalTexture : std::uint8_t
{
    Albedo,
    Normal,
    Orm,
    Emission,
    Max
};

struct DecalParams
{
    Math::Vec3 size = Math::Vec3(2.0f, 2.0f, 2.0f);
    TextureId textures[static_cast<std::size_t>(DecalTexture::Max)];
    float emissionEnergy = 1.0f;
    float albedoMix = 1.0f;
    Math::Vec4 modulate = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
    std::uint32_t cullMask = (1u << 20) - 1u;
    float upperFade = 0.3f;
    float lowerFade = 0.3f;
    bool distanceFade = false;
    float distanceFadeBegin = 40.0f;
    float distanceFadeLength = 10.0f;
    float normalFade = 0.0f;
};

struct DecalDataGpu
{
    float xform[16];
    float invExtents[3];
    float albedoMix;
    float albedoRect[4];
    float normalRect[4];
    float ormRect[4];
    float emissionRect[4];
    float modulate[4];
    float emissionEnergy;
    std::uint32_t mask;
    float upperFade;
    float lowerFade;
    float normalXform[12];
    float normal[3];
    float normalFade;
};

struct DecalInstanceInput
{
    const DecalParams* decal = nullptr;
    Math::Mat4 transform;
    float sortingOffset = 0.0f;
    Math::Vec4 rects[static_cast<std::size_t>(DecalTexture::Max)];
};

struct DecalSorted
{
    std::uint32_t instance = 0;
    Math::Mat4 transform;
    Math::Vec3 extents;
};

std::uint32_t buildDecalData(const DecalInstanceInput* instances, std::uint32_t instanceCount,
        const Math::Mat4& cameraTransform, std::uint32_t maxDecals, ct::Vector<DecalDataGpu>& data,
        ct::Vector<DecalSorted>& sorted);

} // namespace prisma3d
