#pragma once

#include "prisma3d/render/TextureStore.h"

#include <cstdint>

namespace prisma3d
{

struct TerrainMaterialData
{
    static constexpr std::uint32_t kMaxLayers = 5;

    bool enabled = false;
    TextureId normalMap;
    TextureId lightmap;
    TextureId colourMap;
    TextureId blendMap;
    TextureId layerDiffuse[kMaxLayers];
    TextureId layerNormal[kMaxLayers];
    std::uint32_t layerCount = 0;
    float uvMultiplier[kMaxLayers] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    bool normalMapping = false;
    std::uint32_t parallax = 0;
    float parallaxScale = 0.04f;
    bool useLightmap = false;
    bool useColourMap = false;
    bool detailModulation = false;
    float detailStrength = 1.0f;
};

} // namespace prisma3d
