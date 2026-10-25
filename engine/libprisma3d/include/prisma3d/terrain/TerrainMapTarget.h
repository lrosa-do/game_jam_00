#pragma once

#include "prisma3d/terrain/Terrain.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct TerrainPixelBox
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t components = 0;
    ct::Vector<std::uint8_t> data;

    void create(std::uint32_t w, std::uint32_t h, std::uint32_t c)
    {
        width = w;
        height = h;
        components = c;
        data.resize(static_cast<std::size_t>(w) * h * c);
    }
    std::uint8_t* at(std::uint32_t x, std::uint32_t y)
    {
        return &data[(static_cast<std::size_t>(y) * width + x) * components];
    }
    const std::uint8_t* at(std::uint32_t x, std::uint32_t y) const
    {
        return &data[(static_cast<std::size_t>(y) * width + x) * components];
    }
};

class TerrainMapTarget
{
public:
    virtual ~TerrainMapTarget() = default;

    virtual bool createNormalMap(std::uint16_t size) = 0;
    virtual void destroyNormalMap() = 0;
    virtual void writeNormalMap(const TerrainRect& box, const TerrainPixelBox& rgb) = 0;

    virtual bool createLightmap(std::uint16_t size) = 0;
    virtual void destroyLightmap() = 0;
    virtual void writeLightmap(const TerrainRect& box, const TerrainPixelBox& l8) = 0;

    virtual bool createColourMap(std::uint16_t size) = 0;
    virtual void destroyColourMap() = 0;
    virtual void writeColourMap(const TerrainRect& box, const TerrainPixelBox& rgb) = 0;

    virtual bool createCompositeMap(std::uint16_t size) = 0;
    virtual void destroyCompositeMap() = 0;
    virtual void renderCompositeMap(const TerrainRect& imageRect) = 0;

    virtual bool createBlendTexture(std::uint8_t index, std::uint16_t size) = 0;
    virtual void destroyBlendTexture(std::uint8_t index) = 0;
    virtual void writeBlendTexture(std::uint8_t index, const TerrainRect& box,
            const TerrainPixelBox& rgba) = 0;
};

} // namespace prisma3d
