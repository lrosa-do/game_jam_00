#include "prisma3d/terrain/TerrainLayerBlendMap.h"

#include "prisma3d/core/Pixmap.h"

#include <math.h>

namespace prisma3d
{

TerrainLayerBlendMap::TerrainLayerBlendMap(Terrain* parent, std::uint8_t layerIndex,
        std::uint8_t textureIndex, std::uint16_t size)
        : mParent(parent),
          mLayerIdx(layerIndex),
          mChannel(static_cast<std::uint8_t>((layerIndex - 1) % 4)),
          mTextureIndex(textureIndex),
          mSize(size),
          mDirty(false),
          mDirtyBox(),
          mData()
{
    mData.resize(static_cast<std::size_t>(size) * size);
    download();
}

TerrainLayerBlendMap::~TerrainLayerBlendMap()
{
}

void TerrainLayerBlendMap::download()
{
    const ct::Vector<std::uint8_t>& source = mParent->getBlendTextureData(mTextureIndex);
    if (source.size() < mData.size() * 4)
    {
        for (std::size_t i = 0; i < mData.size(); ++i) mData[i] = 0.0f;
        return;
    }
    const std::uint8_t* pSrc = source.data() + mChannel;
    for (std::size_t i = 0; i < mData.size(); ++i)
    {
        mData[i] = static_cast<float>(*pSrc) / 255.0f;
        pSrc += 4;
    }
}

void TerrainLayerBlendMap::convertWorldToUVSpace(const Math::Vec3& worldPos, float* outX,
        float* outY)
{
    Math::Vec3 terrainSpace;
    mParent->getTerrainPosition(worldPos, terrainSpace);
    *outX = terrainSpace.x;
    *outY = 1.0f - terrainSpace.y;
}

void TerrainLayerBlendMap::convertUVToWorldSpace(float x, float y, Math::Vec3* outWorldPos)
{
    mParent->getPosition(x, 1.0f - y, 0.0f, *outWorldPos);
}

void TerrainLayerBlendMap::convertUVToImageSpace(float x, float y, std::size_t* outX,
        std::size_t* outY)
{
    *outX = static_cast<unsigned long>(x * (mSize - 1));
    *outY = static_cast<unsigned long>(y * (mSize - 1));
}

void TerrainLayerBlendMap::convertImageToUVSpace(std::size_t x, std::size_t y, float* outX,
        float* outY)
{
    *outX = static_cast<float>(x) / static_cast<float>(mSize - 1);
    *outY = static_cast<float>(y) / static_cast<float>(mSize - 1);
}

void TerrainLayerBlendMap::convertImageToTerrainSpace(std::size_t x, std::size_t y, float* outX,
        float* outY)
{
    convertImageToUVSpace(x, y, outX, outY);
    *outY = 1.0f - *outY;
}

void TerrainLayerBlendMap::convertTerrainToImageSpace(float x, float y, std::size_t* outX,
        std::size_t* outY)
{
    convertUVToImageSpace(x, 1.0f - y, outX, outY);
}

float TerrainLayerBlendMap::getBlendValue(std::uint32_t x, std::uint32_t y)
{
    return mData[static_cast<std::size_t>(y) * mSize + x];
}

void TerrainLayerBlendMap::setBlendValue(std::uint32_t x, std::uint32_t y, float val)
{
    mData[static_cast<std::size_t>(y) * mSize + x] = val;
    dirtyRect(TerrainRect(static_cast<int>(x), static_cast<int>(y), static_cast<int>(x) + 1,
            static_cast<int>(y) + 1));
}

float* TerrainLayerBlendMap::getBlendPointer()
{
    return mData.data();
}

void TerrainLayerBlendMap::dirty()
{
    dirtyRect(TerrainRect(0, 0, mSize, mSize));
}

void TerrainLayerBlendMap::dirtyRect(const TerrainRect& rect)
{
    if (mDirty)
    {
        mDirtyBox.left = rect.left < mDirtyBox.left ? rect.left : mDirtyBox.left;
        mDirtyBox.top = rect.top < mDirtyBox.top ? rect.top : mDirtyBox.top;
        mDirtyBox.right = rect.right > mDirtyBox.right ? rect.right : mDirtyBox.right;
        mDirtyBox.bottom = rect.bottom > mDirtyBox.bottom ? rect.bottom : mDirtyBox.bottom;
    }
    else
    {
        mDirtyBox = rect;
        mDirty = true;
    }
}

void TerrainLayerBlendMap::update()
{
    if (!mData.empty() && mDirty)
    {
        mDirtyBox = mDirtyBox.intersect(TerrainRect(0, 0, mSize, mSize));
        ct::Vector<std::uint8_t>& destination = mParent->getBlendTextureData(mTextureIndex);
        const int width = mDirtyBox.width();
        const int height = mDirtyBox.height();
        for (int y = 0; y < height; ++y)
        {
            const float* pSrc =
                    &mData[static_cast<std::size_t>(mDirtyBox.top + y) * mSize + mDirtyBox.left];
            std::uint8_t* pDst =
                    &destination[((static_cast<std::size_t>(mDirtyBox.top + y) * mSize) +
                                         mDirtyBox.left) *
                                         4 +
                                 mChannel];
            for (int x = 0; x < width; ++x)
            {
                *pDst = static_cast<std::uint8_t>(*pSrc++ * 255);
                pDst += 4;
            }
        }
        mParent->uploadBlendTextureRect(mTextureIndex, mDirtyBox);

        mDirty = false;

        TerrainRect compositeMapRect;
        const float blendToTerrain =
                static_cast<float>(mParent->getSize()) / static_cast<float>(mSize);
        compositeMapRect.left = static_cast<int>(mDirtyBox.left * blendToTerrain);
        compositeMapRect.right = static_cast<int>(mDirtyBox.right * blendToTerrain + 1);
        compositeMapRect.top = static_cast<int>((mSize - mDirtyBox.bottom) * blendToTerrain);
        compositeMapRect.bottom = static_cast<int>((mSize - mDirtyBox.top) * blendToTerrain + 1);
        mParent->_dirtyCompositeMapRect(compositeMapRect);
        mParent->updateCompositeMapWithDelay();
    }
}

void TerrainLayerBlendMap::blit(const Pixmap& src, const TerrainRect& dstBox)
{
    const int dstWidth = dstBox.width();
    const int dstHeight = dstBox.height();
    if (dstWidth <= 0 || dstHeight <= 0 || !src.is_valid() || src.components < 1) return;
    if (dstBox.left < 0 || dstBox.top < 0 || dstBox.right > mSize || dstBox.bottom > mSize) return;

    for (int y = 0; y < dstHeight; ++y)
    {
        const int sy = (y * src.height) / dstHeight;
        for (int x = 0; x < dstWidth; ++x)
        {
            const int sx = (x * src.width) / dstWidth;
            const std::uint8_t value =
                    src.pixels[(static_cast<std::size_t>(sy) * src.width + sx) * src.components];
            mData[static_cast<std::size_t>(dstBox.top + y) * mSize + dstBox.left + x] =
                    static_cast<float>(value) / 255.0f;
        }
    }

    dirtyRect(dstBox);
}

void TerrainLayerBlendMap::blit(const Pixmap& src)
{
    blit(src, TerrainRect(0, 0, mSize, mSize));
}

void TerrainLayerBlendMap::loadImage(const Pixmap& image)
{
    blit(image);
}

std::uint8_t TerrainLayerBlendMap::getLayerIndex() const
{
    return mLayerIdx;
}

std::uint8_t TerrainLayerBlendMap::getChannel() const
{
    return mChannel;
}

std::uint16_t TerrainLayerBlendMap::getSize() const
{
    return mSize;
}

bool TerrainLayerBlendMap::isDirty() const
{
    return mDirty;
}

const TerrainRect& TerrainLayerBlendMap::getDirtyBox() const
{
    return mDirtyBox;
}

} // namespace prisma3d
