#pragma once

#include "prisma3d/terrain/Terrain.h"

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class Pixmap;

class TerrainLayerBlendMap
{
public:
    TerrainLayerBlendMap(Terrain* parent, std::uint8_t layerIndex, std::uint8_t textureIndex,
            std::uint16_t size);
    ~TerrainLayerBlendMap();
    TerrainLayerBlendMap(const TerrainLayerBlendMap&) = delete;
    TerrainLayerBlendMap& operator=(const TerrainLayerBlendMap&) = delete;

    void convertWorldToUVSpace(const Math::Vec3& worldPos, float* outX, float* outY);
    void convertUVToWorldSpace(float x, float y, Math::Vec3* outWorldPos);
    void convertUVToImageSpace(float x, float y, std::size_t* outX, std::size_t* outY);
    void convertImageToUVSpace(std::size_t x, std::size_t y, float* outX, float* outY);
    void convertImageToTerrainSpace(std::size_t x, std::size_t y, float* outX, float* outY);
    void convertTerrainToImageSpace(float x, float y, std::size_t* outX, std::size_t* outY);

    float getBlendValue(std::uint32_t x, std::uint32_t y);
    void setBlendValue(std::uint32_t x, std::uint32_t y, float val);
    float* getBlendPointer();

    void dirty();
    void dirtyRect(const TerrainRect& rect);
    void update();

    void blit(const Pixmap& src, const TerrainRect& dstBox);
    void blit(const Pixmap& src);
    void loadImage(const Pixmap& image);

    std::uint8_t getLayerIndex() const;
    std::uint8_t getChannel() const;
    std::uint16_t getSize() const;
    bool isDirty() const;
    const TerrainRect& getDirtyBox() const;

private:
    void download();

    Terrain* mParent;
    std::uint8_t mLayerIdx;
    std::uint8_t mChannel;
    std::uint8_t mTextureIndex;
    std::uint16_t mSize;
    bool mDirty;
    TerrainRect mDirtyBox;
    ct::Vector<float> mData;
};

} // namespace prisma3d
