#include "TerrainMath.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/terrain/Terrain.h"
#include "prisma3d/terrain/TerrainGeometryTarget.h"
#include "prisma3d/terrain/TerrainLayerBlendMap.h"
#include "prisma3d/terrain/TerrainMapTarget.h"
#include "prisma3d/terrain/TerrainQuadTreeNode.h"

#include "platform.h"

#include <float.h>
#include <math.h>

namespace prisma3d
{

namespace
{

const ct::String kBlankString;

bool realEqual(float a, float b)
{
    return fabsf(b - a) <= FLT_EPSILON;
}

} // namespace

void Terrain::setGeometryTarget(TerrainGeometryTarget* target)
{
    mGeometryTarget = target;
}

TerrainGeometryTarget* Terrain::getGeometryTarget() const
{
    return mGeometryTarget;
}

TerrainVertexData& Terrain::getVertexScratch(std::uint32_t vertexCount)
{
    if (!mVertexScratch) mVertexScratch = new TerrainVertexData();
    if (mVertexScratch->positions.size() < vertexCount)
    {
        mVertexScratch->positions.resize(vertexCount);
        mVertexScratch->uvs.resize(vertexCount);
        mVertexScratch->deltas.resize(vertexCount);
    }
    return *mVertexScratch;
}

void Terrain::setHeightAtPoint(std::uint32_t x, std::uint32_t y, float h)
{
    if (!mQuadTree) return;
    x = x < static_cast<std::uint32_t>(mSize) - 1u ? x : static_cast<std::uint32_t>(mSize) - 1u;
    y = y < static_cast<std::uint32_t>(mSize) - 1u ? y : static_cast<std::uint32_t>(mSize) - 1u;

    *getHeightData(x, y) = h;
    TerrainRect rect;
    rect.left = static_cast<int>(x);
    rect.right = static_cast<int>(x) + 1;
    rect.top = static_cast<int>(y);
    rect.bottom = static_cast<int>(y) + 1;
    dirtyRect(rect);
}

void Terrain::update(bool synchronous)
{
    updateGeometry();
    updateDerivedData(synchronous);
}

void Terrain::updateGeometry()
{
    if (mQuadTree && !mDirtyGeometryRect.isNull())
    {
        const TerrainRect rect = mDirtyGeometryRect;
        mQuadTree->updateVertexData(true, false, rect, false);
        mQuadTree->refreshBounds(rect);
        mDirtyGeometryRect = TerrainRect();
        if (mGeometryTarget) mGeometryTarget->geometryUpdated(rect);
    }
}

const TerrainRect& Terrain::getDirtyGeometryRect() const
{
    return mDirtyGeometryRect;
}

void Terrain::setMapTarget(TerrainMapTarget* target)
{
    mTarget = target;
}

TerrainMapTarget* Terrain::getMapTarget() const
{
    return mTarget;
}

void Terrain::freeMaps()
{
    deleteBlendMaps(0);
    mLayerBlendMapList.clear();
    if (mTarget)
    {
        for (std::size_t i = 0; i < mBlendTextureData.size(); ++i)
            mTarget->destroyBlendTexture(static_cast<std::uint8_t>(i));
        if (mNormalMapReady) mTarget->destroyNormalMap();
        if (mLightmapReady) mTarget->destroyLightmap();
        if (mColourMapReady) mTarget->destroyColourMap();
        if (mCompositeMapReady) mTarget->destroyCompositeMap();
    }
    mBlendTextureData.clear();
    mNormalMapReady = false;
    mLightmapReady = false;
    mLightmapPadValid = false;
    mColourMapReady = false;
    mColourMapData.clear();
    mCompositeMapReady = false;
    mNormalMapRequired = false;
    mLightMapRequired = false;
    mCompositeMapRequired = false;
    mGlobalColourMapEnabled = false;
    mDirtyDerivedDataRect = TerrainRect();
    mDirtyLightmapFromNeighboursRect = TerrainRect();
    mCompositeMapDirtyRect = TerrainRect();
    mCompositeMapDirtyRectLightmapUpdate = false;
    mCompositeMapUpdateCountdown = 0;
    mLayers.clear();
    mLayerUVMultiplier.clear();
}

void Terrain::loadMaps(bool normalMap, bool lightmap)
{
    if (!mPrepared) return;
    checkLayers(true);
    createOrDestroyGPUColourMap();
    createOrDestroyGPUNormalMap();
    createOrDestroyGPULightmap();
    _setNormalMapRequired(normalMap);
    _setLightMapRequired(lightmap, true);
}

std::uint8_t Terrain::getLayerCount() const
{
    return static_cast<std::uint8_t>(mLayers.size());
}

float Terrain::getLayerWorldSize(std::uint8_t index) const
{
    if (index < mLayers.size()) return mLayers[index].worldSize;
    if (!mLayers.empty()) return mLayers[0].worldSize;
    return mDefaultLayerTextureWorldSize;
}

void Terrain::setLayerWorldSize(std::uint8_t index, float size)
{
    if (index < mLayers.size())
    {
        if (index >= mLayerUVMultiplier.size()) mLayerUVMultiplier.resize(index + 1);

        mLayers[index].worldSize = size;
        mLayerUVMultiplier[index] = mWorldSize / size;
        mMaterialParamsDirty = true;
    }
}

float Terrain::getLayerUVMultiplier(std::uint8_t index) const
{
    if (index < mLayerUVMultiplier.size()) return mLayerUVMultiplier[index];
    if (!mLayerUVMultiplier.empty()) return mLayerUVMultiplier[0];
    return 100.0f;
}

void Terrain::deriveUVMultipliers()
{
    mLayerUVMultiplier.resize(mLayers.size());
    for (std::size_t i = 0; i < mLayers.size(); ++i)
        mLayerUVMultiplier[i] = mWorldSize / mLayers[i].worldSize;
}

const ct::String& Terrain::getLayerTextureName(std::uint8_t layerIndex,
        std::uint8_t samplerIndex) const
{
    if (layerIndex < mLayers.size() && samplerIndex < kLayerSamplerCount)
        return mLayers[layerIndex].textureNames[samplerIndex];
    return kBlankString;
}

void Terrain::setLayerTextureName(std::uint8_t layerIndex, std::uint8_t samplerIndex,
        const ct::String& textureName)
{
    if (layerIndex < mLayers.size() && samplerIndex < kLayerSamplerCount)
    {
        if (!(mLayers[layerIndex].textureNames[samplerIndex] == textureName))
        {
            mLayers[layerIndex].textureNames[samplerIndex] = textureName;
            mMaterialDirty = true;
            mMaterialParamsDirty = true;
        }
    }
}

std::uint16_t Terrain::getLayerBlendMapSize() const
{
    return mLayerBlendMapSize;
}

std::uint8_t Terrain::getBlendTextureCount() const
{
    return static_cast<std::uint8_t>(mBlendTextureData.size());
}

std::uint8_t Terrain::getBlendTextureCount(std::uint8_t numLayers)
{
    return static_cast<std::uint8_t>(((static_cast<int>(numLayers) - 2) / 4) + 1);
}

TerrainBlendChannel Terrain::getLayerBlendTextureIndex(std::uint8_t layerIndex) const
{
    TerrainBlendChannel result;
    const int idx = static_cast<int>(layerIndex) - 1;
    result.texture = static_cast<std::uint8_t>(idx / 4);
    result.channel = static_cast<std::uint8_t>(idx % 4);
    return result;
}

std::uint8_t Terrain::getBlendTextureIndex(std::uint8_t layerIndex) const
{
    if (layerIndex == 0 || layerIndex - 1 >= static_cast<int>(mLayerBlendMapList.size()))
    {
        log_error("terrain: invalid layer index");
        return 0;
    }
    return static_cast<std::uint8_t>((layerIndex - 1) % 4);
}

ct::Vector<std::uint8_t>& Terrain::getBlendTextureData(std::uint8_t index)
{
    return mBlendTextureData[index];
}

const ct::Vector<std::uint8_t>& Terrain::getBlendTextureData(std::uint8_t index) const
{
    return mBlendTextureData[index];
}

void Terrain::uploadBlendTextureRect(std::uint8_t index, const TerrainRect& box)
{
    if (!mTarget || index >= mBlendTextureData.size() || box.isNull()) return;
    const ct::Vector<std::uint8_t>& source = mBlendTextureData[index];
    TerrainPixelBox pixels;
    pixels.create(static_cast<std::uint32_t>(box.width()), static_cast<std::uint32_t>(box.height()),
            4);
    for (int y = 0; y < box.height(); ++y)
    {
        const std::uint8_t* row =
                &source[(static_cast<std::size_t>(box.top + y) * mLayerBlendMapSize + box.left) *
                        4];
        std::uint8_t* out = pixels.at(0, static_cast<std::uint32_t>(y));
        for (int i = 0; i < box.width() * 4; ++i) out[i] = row[i];
    }
    mTarget->writeBlendTexture(index, box, pixels);
}

std::uint8_t Terrain::getMaxLayers() const
{
    std::uint8_t freeTextureUnits = kTextureBudget;
    --freeTextureUnits;
    --freeTextureUnits;
    if (mGlobalColourMapEnabled) --freeTextureUnits;
    const std::uint8_t byBudget = static_cast<std::uint8_t>(freeTextureUnits / 2.25f);
    return byBudget < kMaxLayerSlots ? byBudget : kMaxLayerSlots;
}

bool Terrain::isMaterialDirty() const
{
    return mMaterialDirty;
}

bool Terrain::isMaterialParamsDirty() const
{
    return mMaterialParamsDirty;
}

void Terrain::clearMaterialDirty()
{
    mMaterialDirty = false;
    mMaterialParamsDirty = false;
}

void Terrain::checkLayers(bool includeGPUResources)
{
    for (std::size_t i = 0; i < mLayers.size(); ++i)
        mLayers[i].textureNames.resize(kLayerSamplerCount);

    if (includeGPUResources)
    {
        createGPUBlendTextures();
        createLayerBlendMaps();
    }
}

void Terrain::createGPUBlendTextures()
{
    const std::uint8_t numTex = getBlendTextureCount(getLayerCount());

    while (mBlendTextureData.size() > numTex)
    {
        if (mTarget)
            mTarget->destroyBlendTexture(static_cast<std::uint8_t>(mBlendTextureData.size() - 1));
        mBlendTextureData.pop_back();
    }

    const std::size_t currentTex = mBlendTextureData.size();
    mBlendTextureData.resize(numTex);
    for (std::size_t i = currentTex; i < numTex; ++i)
    {
        const std::size_t bytes =
                static_cast<std::size_t>(mLayerBlendMapSize) * mLayerBlendMapSize * 4;
        mBlendTextureData[i].resize(bytes);
        for (std::size_t b = 0; b < bytes; ++b) mBlendTextureData[i][b] = 0;
        if (mTarget) mTarget->createBlendTexture(static_cast<std::uint8_t>(i), mLayerBlendMapSize);
    }
}

void Terrain::createLayerBlendMaps()
{
    const std::size_t keep = mLayers.empty() ? 0 : mLayers.size() - 1;
    while (mLayerBlendMapList.size() > keep)
    {
        delete mLayerBlendMapList.back();
        mLayerBlendMapList.pop_back();
    }

    if (mLayers.size() > 1) mLayerBlendMapList.resize(mLayers.size() - 1, nullptr);
}

void Terrain::deleteBlendMaps(std::uint8_t lowIndex)
{
    for (std::size_t i = lowIndex; i < mLayerBlendMapList.size(); ++i)
    {
        delete mLayerBlendMapList[i];
        mLayerBlendMapList[i] = nullptr;
    }
}

TerrainLayerBlendMap* Terrain::getLayerBlendMap(std::uint8_t layerIndex)
{
    if (layerIndex == 0 || layerIndex - 1 >= static_cast<int>(mLayerBlendMapList.size()))
    {
        log_error("terrain: invalid layer index");
        return nullptr;
    }

    const std::uint8_t idx = static_cast<std::uint8_t>(layerIndex - 1);
    if (!mLayerBlendMapList[idx])
    {
        if (mBlendTextureData.size() <= static_cast<std::size_t>(idx / 4)) checkLayers(true);
        if (mBlendTextureData.size() <= static_cast<std::size_t>(idx / 4)) return nullptr;
        mLayerBlendMapList[idx] = new TerrainLayerBlendMap(this, layerIndex,
                static_cast<std::uint8_t>(idx / 4), mLayerBlendMapSize);
    }

    return mLayerBlendMapList[idx];
}

void Terrain::replaceLayer(std::uint8_t index, bool keepBlends, float worldSize,
        const ct::Vector<ct::String>* textureNames)
{
    if (getLayerCount() > 0)
    {
        if (index >= getLayerCount()) index = static_cast<std::uint8_t>(getLayerCount() - 1);

        if (textureNames) mLayers[index].textureNames = *textureNames;

        setLayerWorldSize(index, worldSize);

        if (!keepBlends && index > 0)
        {
            if (mLayerBlendMapList[index - 1])
            {
                delete mLayerBlendMapList[index - 1];
                mLayerBlendMapList[index - 1] = nullptr;
            }

            const TerrainBlendChannel layerPair = getLayerBlendTextureIndex(index);
            clearGPUBlendChannel(layerPair.texture, layerPair.channel);
        }

        mMaterialDirty = true;
        mMaterialParamsDirty = true;
    }
}

void Terrain::addLayer(float worldSize, const ct::Vector<ct::String>* textureNames)
{
    addLayer(getLayerCount(), worldSize, textureNames);
}

void Terrain::addLayer(std::uint8_t index, float worldSize,
        const ct::Vector<ct::String>* textureNames)
{
    if (worldSize == 0.0f) worldSize = mDefaultLayerTextureWorldSize;

    const std::uint8_t blendIndex = static_cast<std::uint8_t>(index > 1 ? index - 1 : 0);
    if (index >= getLayerCount())
    {
        mLayers.push_back(TerrainLayerInstance());
        index = static_cast<std::uint8_t>(getLayerCount() - 1);
    }
    else
    {
        mLayers.insert(mLayers.begin() + index, TerrainLayerInstance());
        mLayerUVMultiplier.insert(
                mLayerUVMultiplier.begin() +
                        (index < mLayerUVMultiplier.size() ? index : mLayerUVMultiplier.size()),
                0.0f);
        mLayerBlendMapList.insert(mLayerBlendMapList.begin() +
                                          (blendIndex < mLayerBlendMapList.size()
                                                          ? blendIndex
                                                          : mLayerBlendMapList.size()),
                static_cast<TerrainLayerBlendMap*>(nullptr));
    }
    if (textureNames) mLayers[index].textureNames = *textureNames;

    setLayerWorldSize(index, worldSize);
    checkLayers(true);

    if (index < getLayerCount() - 1)
    {
        shiftUpGPUBlendChannels(blendIndex);
        deleteBlendMaps(index);
    }

    mMaterialDirty = true;
    mMaterialParamsDirty = true;
}

void Terrain::removeLayer(std::uint8_t index)
{
    if (index < mLayers.size())
    {
        const std::uint8_t blendIndex = static_cast<std::uint8_t>(index > 1 ? index - 1 : 0);

        shiftDownGPUBlendChannels(blendIndex);

        mLayers.erase(mLayers.begin() + index);
        mLayerUVMultiplier.erase(mLayerUVMultiplier.begin() + index);

        if (mLayerBlendMapList.size() > 0)
        {
            if (blendIndex < mLayerBlendMapList.size())
            {
                delete mLayerBlendMapList[blendIndex];
                mLayerBlendMapList.erase(mLayerBlendMapList.begin() + blendIndex);
            }

            checkLayers(true);

            deleteBlendMaps(blendIndex);
        }

        mMaterialDirty = true;
        mMaterialParamsDirty = true;
    }
}

void Terrain::shiftUpGPUBlendChannels(std::uint8_t index)
{
    const std::uint8_t layerIndex = static_cast<std::uint8_t>(index + 1);
    for (int i = getLayerCount() - 1; i > layerIndex; --i)
    {
        const TerrainBlendChannel destPair =
                getLayerBlendTextureIndex(static_cast<std::uint8_t>(i));
        const TerrainBlendChannel srcPair =
                getLayerBlendTextureIndex(static_cast<std::uint8_t>(i - 1));

        copyBlendTextureChannel(srcPair.texture, srcPair.channel, destPair.texture,
                destPair.channel);
    }

    const TerrainBlendChannel layerPair = getLayerBlendTextureIndex(layerIndex);
    clearGPUBlendChannel(layerPair.texture, layerPair.channel);
}

void Terrain::shiftDownGPUBlendChannels(std::uint8_t index)
{
    const std::uint8_t layerIndex = static_cast<std::uint8_t>(index + 1);
    for (int i = layerIndex; i < getLayerCount() - 1; ++i)
    {
        const TerrainBlendChannel destPair =
                getLayerBlendTextureIndex(static_cast<std::uint8_t>(i));
        const TerrainBlendChannel srcPair =
                getLayerBlendTextureIndex(static_cast<std::uint8_t>(i + 1));

        copyBlendTextureChannel(srcPair.texture, srcPair.channel, destPair.texture,
                destPair.channel);
    }

    if (getLayerCount() > 1)
    {
        const TerrainBlendChannel layerPair =
                getLayerBlendTextureIndex(static_cast<std::uint8_t>(getLayerCount() - 1));
        clearGPUBlendChannel(layerPair.texture, layerPair.channel);
    }
}

void Terrain::copyBlendTextureChannel(std::uint8_t srcIndex, std::uint8_t srcChannel,
        std::uint8_t destIndex, std::uint8_t destChannel)
{
    if (srcIndex >= mBlendTextureData.size() || destIndex >= mBlendTextureData.size()) return;
    const std::size_t texels = static_cast<std::size_t>(mLayerBlendMapSize) * mLayerBlendMapSize;
    const ct::Vector<std::uint8_t>& source = mBlendTextureData[srcIndex];
    ct::Vector<std::uint8_t>& destination = mBlendTextureData[destIndex];
    for (std::size_t i = 0; i < texels; ++i)
        destination[i * 4 + destChannel] = source[i * 4 + srcChannel];
    uploadBlendTextureRect(destIndex, TerrainRect(0, 0, mLayerBlendMapSize, mLayerBlendMapSize));
}

void Terrain::clearGPUBlendChannel(std::uint8_t index, unsigned channel)
{
    if (index >= mBlendTextureData.size()) return;
    const std::size_t texels = static_cast<std::size_t>(mLayerBlendMapSize) * mLayerBlendMapSize;
    ct::Vector<std::uint8_t>& data = mBlendTextureData[index];
    for (std::size_t i = 0; i < texels; ++i) data[i * 4 + channel] = 0;
    uploadBlendTextureRect(index, TerrainRect(0, 0, mLayerBlendMapSize, mLayerBlendMapSize));
}

void Terrain::createOrDestroyGPUNormalMap()
{
    if (!mTarget) return;
    if (mNormalMapRequired && !mNormalMapReady) mNormalMapReady = mTarget->createNormalMap(mSize);
    else if (!mNormalMapRequired && mNormalMapReady)
    {
        mTarget->destroyNormalMap();
        mNormalMapReady = false;
    }
}

void Terrain::createOrDestroyGPULightmap()
{
    if (!mTarget) return;
    if (mLightMapRequired && !mLightmapReady)
        mLightmapReady = mTarget->createLightmap(mLightmapSize);
    else if (!mLightMapRequired && mLightmapReady)
    {
        mTarget->destroyLightmap();
        mLightmapReady = false;
        mLightmapPadValid = false;
    }
}

void Terrain::createOrDestroyGPUColourMap()
{
    if (!mTarget) return;
    if (mGlobalColourMapEnabled && !mColourMapReady)
        mColourMapReady = mTarget->createColourMap(mGlobalColourMapSize);
    else if (!mGlobalColourMapEnabled && mColourMapReady)
    {
        mTarget->destroyColourMap();
        mColourMapReady = false;
    }
}

void Terrain::createOrDestroyGPUCompositeMap()
{
    if (!mTarget) return;
    if (mCompositeMapRequired && !mCompositeMapReady)
        mCompositeMapReady = mTarget->createCompositeMap(mCompositeMapSize);
    else if (!mCompositeMapRequired && mCompositeMapReady)
    {
        mTarget->destroyCompositeMap();
        mCompositeMapReady = false;
    }
}

void Terrain::_setNormalMapRequired(bool normalMap)
{
    if (normalMap != mNormalMapRequired)
    {
        mNormalMapRequired = normalMap;

        createOrDestroyGPUNormalMap();

        if (mNormalMapRequired)
        {
            mDirtyDerivedDataRect.left = mDirtyDerivedDataRect.top = 0;
            mDirtyDerivedDataRect.right = mDirtyDerivedDataRect.bottom = mSize;
            updateDerivedData(false, DERIVED_DATA_NORMALS);
        }
    }
}

void Terrain::_setLightMapRequired(bool lightMap, bool shadowsOnly)
{
    if (lightMap != mLightMapRequired || shadowsOnly != mLightMapShadowsOnly)
    {
        mLightMapRequired = lightMap;
        mLightMapShadowsOnly = shadowsOnly;

        createOrDestroyGPULightmap();

        if (mLightMapRequired)
        {
            mDirtyDerivedDataRect.left = mDirtyDerivedDataRect.top = 0;
            mDirtyDerivedDataRect.right = mDirtyDerivedDataRect.bottom = mSize;
            updateDerivedData(false, DERIVED_DATA_LIGHTMAP);
        }
    }
}

void Terrain::_setCompositeMapRequired(bool compositeMap)
{
    if (compositeMap != mCompositeMapRequired)
    {
        mCompositeMapRequired = compositeMap;

        createOrDestroyGPUCompositeMap();

        if (mCompositeMapRequired)
        {
            mCompositeMapDirtyRect.left = mCompositeMapDirtyRect.top = 0;
            mCompositeMapDirtyRect.right = mCompositeMapDirtyRect.bottom = mSize;
            updateCompositeMap();
        }
    }
}

bool Terrain::_getNormalMapRequired() const
{
    return mNormalMapRequired;
}

bool Terrain::_getLightMapRequired() const
{
    return mLightMapRequired;
}

bool Terrain::_getCompositeMapRequired() const
{
    return mCompositeMapRequired;
}

void Terrain::dirty()
{
    dirtyRect(TerrainRect(0, 0, mSize, mSize));
}

void Terrain::dirtyRect(const TerrainRect& rect)
{
    mDirtyGeometryRect.merge(rect);
    mDirtyDerivedDataRect.merge(rect);
    mCompositeMapDirtyRect.merge(rect);
}

void Terrain::_dirtyCompositeMapRect(const TerrainRect& rect)
{
    mCompositeMapDirtyRect.merge(rect);
}

void Terrain::dirtyLightmapRect(const TerrainRect& rect)
{
    mDirtyDerivedDataRect.merge(rect);
}

void Terrain::dirtyLightmap()
{
    dirtyLightmapRect(TerrainRect(0, 0, mSize, mSize));
}

void Terrain::updateDerivedData(bool synchronous, std::uint8_t typeMask)
{
    (void) synchronous;
    if (!mDirtyDerivedDataRect.isNull() || !mDirtyLightmapFromNeighboursRect.isNull())
    {
        updateDerivedDataImpl(mDirtyDerivedDataRect, mDirtyLightmapFromNeighboursRect, typeMask);
        mDirtyDerivedDataRect = TerrainRect();
        mDirtyLightmapFromNeighboursRect = TerrainRect();
    }
    else
    {
        updateCompositeMap();
    }
}

void Terrain::updateDerivedDataImpl(const TerrainRect& rect, const TerrainRect& lightmapExtraRect,
        std::uint8_t typeMask)
{
    std::uint8_t mask = typeMask;
    if (!mNormalMapRequired) mask = static_cast<std::uint8_t>(mask & ~DERIVED_DATA_NORMALS);
    if (!mLightMapRequired) mask = static_cast<std::uint8_t>(mask & ~DERIVED_DATA_LIGHTMAP);
    mask = static_cast<std::uint8_t>(
            mask & (DERIVED_DATA_DELTAS | DERIVED_DATA_NORMALS | DERIVED_DATA_LIGHTMAP));

    while (mask)
    {
        TerrainRect finalRect;
        TerrainPixelBox box;
        if (mask & DERIVED_DATA_DELTAS)
        {
            finalRect = calculateHeightDeltas(rect);
            mask = static_cast<std::uint8_t>(mask & ~DERIVED_DATA_DELTAS);
            finaliseHeightDeltas(finalRect, false);
            if (mGeometryTarget) mGeometryTarget->geometryUpdated(finalRect);
        }
        else if (mask & DERIVED_DATA_NORMALS)
        {
            calculateNormals(rect, finalRect, box);
            mask = static_cast<std::uint8_t>(mask & ~DERIVED_DATA_NORMALS);
            finaliseNormals(finalRect, box);
            mCompositeMapDirtyRect.merge(rect);
        }
        else if (mask & DERIVED_DATA_LIGHTMAP)
        {
            const float pad = lightmapHeightPad();
            const TerrainRect lightRect = mLightmapPadValid && pad != mLightmapPad
                                                  ? TerrainRect(0, 0, mSize, mSize)
                                                  : rect;
            mLightmapPad = pad;
            mLightmapPadValid = true;
            calculateLightmap(lightRect, lightmapExtraRect, finalRect, box);
            mask = static_cast<std::uint8_t>(mask & ~DERIVED_DATA_LIGHTMAP);
            finaliseLightmap(finalRect, box);
            mCompositeMapDirtyRect.merge(lightRect);
            mCompositeMapDirtyRectLightmapUpdate = true;
        }
    }

    if (mCompositeMapRequired) updateCompositeMap();
}

void Terrain::calculateNormals(const TerrainRect& rect, TerrainRect& finalRect,
        TerrainPixelBox& out)
{
    const int size = static_cast<int>(mSize);
    const TerrainRect widenedRect(rect.left - 1 > 0 ? rect.left - 1 : 0,
            rect.top - 1 > 0 ? rect.top - 1 : 0, rect.right + 1 < size ? rect.right + 1 : size,
            rect.bottom + 1 < size ? rect.bottom + 1 : size);

    const int width = widenedRect.width() > 0 ? widenedRect.width() : 0;
    const int height = widenedRect.height() > 0 ? widenedRect.height() : 0;
    out.create(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height), 3);

    for (int y = widenedRect.top; y < widenedRect.bottom; ++y)
    {
        for (int x = widenedRect.left; x < widenedRect.right; ++x)
        {
            const Math::Vec3 cumulativeNormal = calculateNormal(x, y);

            const long storeX = x - widenedRect.left;
            const long storeY = widenedRect.bottom - y - 1;

            std::uint8_t* pStore =
                    out.at(static_cast<std::uint32_t>(storeX), static_cast<std::uint32_t>(storeY));
            *pStore++ = static_cast<std::uint8_t>((cumulativeNormal.x + 1.0f) * 0.5f * 255.0f);
            *pStore++ = static_cast<std::uint8_t>((cumulativeNormal.y + 1.0f) * 0.5f * 255.0f);
            *pStore++ = static_cast<std::uint8_t>((cumulativeNormal.z + 1.0f) * 0.5f * 255.0f);
        }
    }

    finalRect = widenedRect;
}

void Terrain::finaliseNormals(const TerrainRect& rect, const TerrainPixelBox& normalsBox)
{
    createOrDestroyGPUNormalMap();
    if (mNormalMapReady && mTarget && !rect.isNull())
    {
        TerrainRect dstBox;
        dstBox.left = rect.left;
        dstBox.right = rect.right;
        dstBox.top = static_cast<int>(mSize) - rect.bottom;
        dstBox.bottom = static_cast<int>(mSize) - rect.top;
        mTarget->writeNormalMap(dstBox, normalsBox);
    }
}

void Terrain::widenRectByVector(const Math::Vec3& vec, const TerrainRect& inRect,
        TerrainRect& outRect) const
{
    widenRectByVector(vec, inRect, getMinHeight(), getMaxHeight(), outRect);
}

void Terrain::widenRectByVector(const Math::Vec3& vec, const TerrainRect& inRect, float minHeight,
        float maxHeight, TerrainRect& outRect) const
{
    outRect = inRect;

    Math::Vec3 normal;
    Math::Vec3 point(0.0f, 0.0f, 0.0f);
    switch (mAlign)
    {
        case TerrainAlignment::AlignXY:
            normal = Math::Vec3(0.0f, 0.0f, 1.0f);
            point = Math::Vec3(0.0f, 0.0f, vec.z < 0.0f ? minHeight : maxHeight);
            break;
        case TerrainAlignment::AlignXZ:
            normal = Math::Vec3(0.0f, 1.0f, 0.0f);
            point = Math::Vec3(0.0f, vec.y < 0.0f ? minHeight : maxHeight, 0.0f);
            break;
        case TerrainAlignment::AlignYZ:
            normal = Math::Vec3(1.0f, 0.0f, 0.0f);
            point = Math::Vec3(vec.x < 0.0f ? minHeight : maxHeight, 0.0f, 0.0f);
            break;
    }
    const Math::Vec4 plane(normal.x, normal.y, normal.z, -normal.Dot(point));
    const float verticalVal = vec.Dot(normal);

    if (realEqual(verticalVal, 0.0f)) return;

    Math::Vec3 corners[4];
    const float startHeight = verticalVal < 0.0f ? maxHeight : minHeight;
    getPoint(static_cast<std::uint32_t>(inRect.left), static_cast<std::uint32_t>(inRect.top),
            startHeight, corners[0]);
    getPoint(static_cast<std::uint32_t>(inRect.right - 1), static_cast<std::uint32_t>(inRect.top),
            startHeight, corners[1]);
    getPoint(static_cast<std::uint32_t>(inRect.left), static_cast<std::uint32_t>(inRect.bottom - 1),
            startHeight, corners[2]);
    getPoint(static_cast<std::uint32_t>(inRect.right - 1),
            static_cast<std::uint32_t>(inRect.bottom - 1), startHeight, corners[3]);

    for (const Math::Vec3& corner: corners)
    {
        const Math::Ray ray(corner + mPos, vec);
        const terrain_math::Hit rayHit = terrain_math::intersectsPlane(ray, plane);
        if (rayHit.hit)
        {
            const Math::Vec3 pt = ray.origin + ray.direction * rayHit.distance;
            Math::Vec3 terrainHitPos;
            getTerrainPosition(pt, terrainHitPos);
            const float last = static_cast<float>(mSize - 1);
            const TerrainRect mergeRect(static_cast<int>(terrainHitPos.x * last),
                    static_cast<int>(terrainHitPos.y * last),
                    static_cast<int>((terrainHitPos.x * last + 0.5) + 1),
                    static_cast<int>((terrainHitPos.y * last + 0.5) + 1));
            outRect.merge(mergeRect);
        }
    }
}

float Terrain::lightmapHeightPad() const
{
    if (!mLightmapPadFollowsRange && mLightmapPadValid) return mLightmapPad;
    return (getMaxHeight() - getMinHeight()) * 1.0e-3f;
}

void Terrain::calculateLightmap(const TerrainRect& rect, const TerrainRect& extraTargetRect,
        TerrainRect& outFinalRect, TerrainPixelBox& out)
{
    const Math::Vec3& lightVec = mLightMapDir;
    const int size = static_cast<int>(mSize);
    const TerrainRect paddedRect =
            rect.isNull() ? rect
                          : TerrainRect(rect.left - 1 > 0 ? rect.left - 1 : 0,
                                    rect.top - 1 > 0 ? rect.top - 1 : 0,
                                    rect.right + 1 < size ? rect.right + 1 : size,
                                    rect.bottom + 1 < size ? rect.bottom + 1 : size);
    TerrainRect widenedRect;
    if (paddedRect.isNull()) widenedRect = paddedRect;
    else
        widenRectByVector(lightVec, paddedRect, widenedRect);

    widenedRect.merge(extraTargetRect);

    const float terrainToLightmapScale =
            static_cast<float>(mLightmapSize) / static_cast<float>(mSize);
    widenedRect.left = static_cast<int>(widenedRect.left * terrainToLightmapScale);
    widenedRect.right = static_cast<int>(widenedRect.right * terrainToLightmapScale);
    widenedRect.top = static_cast<int>(widenedRect.top * terrainToLightmapScale);
    widenedRect.bottom = static_cast<int>(widenedRect.bottom * terrainToLightmapScale);

    widenedRect = widenedRect.intersect(TerrainRect(0, 0, mLightmapSize, mLightmapSize));

    outFinalRect = widenedRect;

    const int width = widenedRect.width() > 0 ? widenedRect.width() : 0;
    const int height = widenedRect.height() > 0 ? widenedRect.height() : 0;
    out.create(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height), 1);

    const float heightPad = lightmapHeightPad();

    for (long y = widenedRect.top; y < widenedRect.bottom; ++y)
    {
        for (long x = widenedRect.left; x < widenedRect.right; ++x)
        {
            float litVal = 1.0f;

            const float Tx = static_cast<float>(x) / static_cast<float>(mLightmapSize - 1);
            const float Ty = static_cast<float>(y) / static_cast<float>(mLightmapSize - 1);

            Math::Vec3 wpos(0.0f, 0.0f, 0.0f);
            getPosition(Tx, Ty, getHeightAtTerrainPosition(Tx, Ty) + heightPad, wpos);
            wpos += mPos;
            const Math::Ray ray(wpos, -lightVec);

            const TerrainRayResult rayHit = rayIntersects(ray);

            if (rayHit.hit) litVal = 0.0f;

            const long storeX = x - widenedRect.left;
            const long storeY = widenedRect.bottom - y - 1;

            *out.at(static_cast<std::uint32_t>(storeX), static_cast<std::uint32_t>(storeY)) =
                    static_cast<std::uint8_t>(litVal * 255.0);
        }
    }
}

void Terrain::finaliseLightmap(const TerrainRect& rect, const TerrainPixelBox& lightmapBox)
{
    createOrDestroyGPULightmap();
    if (mLightmapReady && mTarget && !rect.isNull())
    {
        TerrainRect dstBox;
        dstBox.left = rect.left;
        dstBox.right = rect.right;
        dstBox.top = static_cast<int>(mLightmapSize) - rect.bottom;
        dstBox.bottom = static_cast<int>(mLightmapSize) - rect.top;
        mTarget->writeLightmap(dstBox, lightmapBox);
    }
}

TerrainRect Terrain::compositeImageRect(const TerrainRect& rect) const
{
    const int compSize = mCompositeMapSize;
    TerrainRect imgRect;
    Math::Vec3 inVec;
    Math::Vec3 outVec;
    inVec.x = static_cast<float>(rect.left);
    inVec.y = static_cast<float>(rect.bottom - 1);
    inVec.z = 0.0f;
    convertPosition(TerrainSpace::Point, inVec, TerrainSpace::Terrain, outVec);
    imgRect.left = static_cast<int>(outVec.x * compSize);
    imgRect.top = static_cast<int>((1.0f - outVec.y) * compSize);
    inVec.x = static_cast<float>(rect.right - 1);
    inVec.y = static_cast<float>(rect.top);
    convertPosition(TerrainSpace::Point, inVec, TerrainSpace::Terrain, outVec);
    imgRect.right = static_cast<int>(outVec.x * static_cast<float>(compSize) + 1);
    imgRect.bottom = static_cast<int>((1.0 - outVec.y) * compSize + 1);

    return imgRect.intersect(TerrainRect(0, 0, compSize, compSize));
}

void Terrain::updateCompositeMap()
{
    if (mCompositeMapRequired && !mCompositeMapDirtyRect.isNull())
    {
        createOrDestroyGPUCompositeMap();
        TerrainRect rect = mCompositeMapDirtyRect;
        if (mCompositeMapDirtyRectLightmapUpdate &&
                (mCompositeMapDirtyRect.width() < mSize || mCompositeMapDirtyRect.height() < mSize))
        {
            TerrainRect widenedRect;
            widenRectByVector(mLightMapDir, mCompositeMapDirtyRect, widenedRect);
            rect = widenedRect.intersect(TerrainRect(0, 0, mSize, mSize));
        }
        if (mTarget && mCompositeMapReady && !rect.isNull())
            mTarget->renderCompositeMap(compositeImageRect(rect));

        mCompositeMapDirtyRectLightmapUpdate = false;
        mCompositeMapDirtyRect = TerrainRect();
    }
}

void Terrain::updateCompositeMapWithDelay(float delay)
{
    mCompositeMapUpdateCountdown = static_cast<long>(delay * 1000);
}

void Terrain::advanceDeferredUpdates(unsigned long elapsedMillis)
{
    if (mCompositeMapUpdateCountdown > 0 && elapsedMillis)
    {
        if (elapsedMillis > static_cast<unsigned long>(mCompositeMapUpdateCountdown))
            mCompositeMapUpdateCountdown = 0;
        else
            mCompositeMapUpdateCountdown -= static_cast<long>(elapsedMillis);

        if (!mCompositeMapUpdateCountdown) updateCompositeMap();
    }
}

std::uint16_t Terrain::getMaterialLodIndex(float value) const
{
    const float lodValues[2] = { 0.0f, mCompositeMapDistance * mCompositeMapDistance };
    const std::uint16_t count = mCompositeMapRequired ? 2 : 1;
    std::uint16_t index = 0;
    for (std::uint16_t i = 0; i < count; ++i)
    {
        if (lodValues[i] > value) return index ? static_cast<std::uint16_t>(index - 1) : 0;
        ++index;
    }
    return static_cast<std::uint16_t>(count - 1);
}

void Terrain::setGlobalColourMapEnabled(bool enabled, std::uint16_t sz)
{
    if (!sz) sz = mDefaultGlobalColourMapSize;
    if (!enabled && !mGlobalColourMapEnabled) mGlobalColourMapSize = sz;

    if (enabled != mGlobalColourMapEnabled || (enabled && mGlobalColourMapSize != sz))
    {
        mGlobalColourMapEnabled = enabled;
        mGlobalColourMapSize = sz;
        mColourMapData.clear();

        if (mColourMapReady && mTarget)
        {
            mTarget->destroyColourMap();
            mColourMapReady = false;
        }
        createOrDestroyGPUColourMap();

        mMaterialDirty = true;
        mMaterialParamsDirty = true;
    }
}

bool Terrain::getGlobalColourMapEnabled() const
{
    return mGlobalColourMapEnabled;
}

std::uint16_t Terrain::getGlobalColourMapSize() const
{
    return mGlobalColourMapSize;
}

bool Terrain::loadGlobalColourMap(const Pixmap& image)
{
    if (!mGlobalColourMapEnabled || !mColourMapReady || !mTarget) return false;
    if (!image.is_valid() || image.components < 1) return false;

    const int size = mGlobalColourMapSize;
    TerrainPixelBox box;
    box.create(static_cast<std::uint32_t>(size), static_cast<std::uint32_t>(size), 3);
    for (int y = 0; y < size; ++y)
    {
        const int sy = (y * image.height) / size;
        for (int x = 0; x < size; ++x)
        {
            const int sx = (x * image.width) / size;
            const std::uint8_t* in =
                    &image.pixels[(static_cast<std::size_t>(sy) * image.width + sx) *
                                  image.components];
            std::uint8_t* out =
                    box.at(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
            const bool gray = image.components < 3;
            out[0] = in[0];
            out[1] = gray ? in[0] : in[1];
            out[2] = gray ? in[0] : in[2];
        }
    }
    mTarget->writeColourMap(TerrainRect(0, 0, size, size), box);
    mColourMapData = box.data;
    return true;
}

const ct::Vector<std::uint8_t>& Terrain::getGlobalColourMapData() const
{
    return mColourMapData;
}

std::uint8_t* Terrain::globalColourMapData()
{
    if (!mGlobalColourMapEnabled || !mColourMapReady || !mTarget) return nullptr;
    const std::size_t size = mGlobalColourMapSize;
    const std::size_t count = size * size * 3;
    if (mColourMapData.size() != count)
    {
        mColourMapData.resize(count);
        for (std::size_t i = 0; i < count; ++i) mColourMapData[i] = 255;
        dirtyGlobalColourMapRect(TerrainRect(0, 0, mGlobalColourMapSize, mGlobalColourMapSize));
    }
    return mColourMapData.data();
}

void Terrain::dirtyGlobalColourMapRect(const TerrainRect& rect)
{
    if (!mTarget || !mColourMapReady || mColourMapData.empty()) return;
    const int size = mGlobalColourMapSize;
    const TerrainRect box = rect.intersect(TerrainRect(0, 0, size, size));
    if (box.isNull() || box.width() <= 0 || box.height() <= 0) return;
    TerrainPixelBox pixels;
    pixels.create(static_cast<std::uint32_t>(box.width()), static_cast<std::uint32_t>(box.height()),
            3);
    for (int y = 0; y < box.height(); ++y)
    {
        const std::uint8_t* source =
                &mColourMapData[(static_cast<std::size_t>(box.top + y) * size + box.left) * 3];
        std::uint8_t* destination = pixels.at(0, static_cast<std::uint32_t>(y));
        for (int i = 0; i < box.width() * 3; ++i) destination[i] = source[i];
    }
    mTarget->writeColourMap(box, pixels);
    const int last = mSize - 1;
    TerrainRect points(box.left * last / size, box.top * last / size,
            (box.right * last + size - 1) / size + 1, (box.bottom * last + size - 1) / size + 1);
    _dirtyCompositeMapRect(points.intersect(TerrainRect(0, 0, mSize, mSize)));
}

std::uint16_t Terrain::getLightmapSize() const
{
    return mLightmapSize;
}

std::uint16_t Terrain::getCompositeMapSize() const
{
    return mCompositeMapSize;
}

const Math::Vec3& Terrain::getLightMapDirection() const
{
    return mLightMapDir;
}

void Terrain::setLightMapDirection(const Math::Vec3& direction)
{
    mLightMapDir = direction;
}

bool Terrain::getLightMapPadFollowsRange() const
{
    return mLightmapPadFollowsRange;
}

void Terrain::setLightMapPadFollowsRange(bool follows)
{
    mLightmapPadFollowsRange = follows;
}

const Math::Vec3& Terrain::getCompositeMapAmbient() const
{
    return mCompositeMapAmbient;
}

void Terrain::setCompositeMapAmbient(const Math::Vec3& ambient)
{
    mCompositeMapAmbient = ambient;
}

const Math::Vec3& Terrain::getCompositeMapDiffuse() const
{
    return mCompositeMapDiffuse;
}

void Terrain::setCompositeMapDiffuse(const Math::Vec3& diffuse)
{
    mCompositeMapDiffuse = diffuse;
}

float Terrain::getCompositeMapDistance() const
{
    return mCompositeMapDistance;
}

void Terrain::setCompositeMapDistance(float distance)
{
    mCompositeMapDistance = distance;
}

float Terrain::getDefaultLayerTextureWorldSize() const
{
    return mDefaultLayerTextureWorldSize;
}

} // namespace prisma3d
