#include "prisma3d/terrain/Terrain.h"

#include "TerrainMath.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/terrain/TerrainGeometryTarget.h"
#include "prisma3d/terrain/TerrainQuadTreeNode.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace
{

bool isPowerOfTwoPlusOne(std::uint32_t value)
{
    if (value < 3) return false;
    const std::uint32_t n = value - 1;
    return (n & (n - 1)) == 0;
}

std::uint16_t log2Exact(std::uint32_t powerOfTwo)
{
    std::uint16_t result = 0;
    while (powerOfTwo > 1)
    {
        powerOfTwo >>= 1;
        ++result;
    }
    return result;
}

float clampf(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

void resampleBilinear(const ct::Vector<float>& source, int sourceWidth, int sourceHeight,
        ct::Vector<float>& target, int targetSize)
{
    target.resize(static_cast<std::size_t>(targetSize) * targetSize);
    for (int y = 0; y < targetSize; ++y)
    {
        const float fy = targetSize > 1 ? static_cast<float>(y) * (sourceHeight - 1) /
                                                  static_cast<float>(targetSize - 1)
                                        : 0.0f;
        int y0 = static_cast<int>(fy);
        if (y0 > sourceHeight - 2) y0 = sourceHeight > 1 ? sourceHeight - 2 : 0;
        const int y1 = y0 + 1 < sourceHeight ? y0 + 1 : y0;
        const float ty = fy - static_cast<float>(y0);
        for (int x = 0; x < targetSize; ++x)
        {
            const float fx = targetSize > 1 ? static_cast<float>(x) * (sourceWidth - 1) /
                                                      static_cast<float>(targetSize - 1)
                                            : 0.0f;
            int x0 = static_cast<int>(fx);
            if (x0 > sourceWidth - 2) x0 = sourceWidth > 1 ? sourceWidth - 2 : 0;
            const int x1 = x0 + 1 < sourceWidth ? x0 + 1 : x0;
            const float tx = fx - static_cast<float>(x0);
            const float a = source[static_cast<std::size_t>(y0) * sourceWidth + x0];
            const float b = source[static_cast<std::size_t>(y0) * sourceWidth + x1];
            const float c = source[static_cast<std::size_t>(y1) * sourceWidth + x0];
            const float d = source[static_cast<std::size_t>(y1) * sourceWidth + x1];
            target[static_cast<std::size_t>(y) * targetSize + x] =
                    (a * (1.0f - tx) + b * tx) * (1.0f - ty) + (c * (1.0f - tx) + d * tx) * ty;
        }
    }
}

} // namespace

void TerrainRect::merge(const TerrainRect& other)
{
    if (other.isNull()) return;
    if (isNull())
    {
        *this = other;
        return;
    }
    left = other.left < left ? other.left : left;
    right = other.right > right ? other.right : right;
    top = other.top < top ? other.top : top;
    bottom = other.bottom > bottom ? other.bottom : bottom;
}

TerrainRect TerrainRect::intersect(const TerrainRect& other) const
{
    if (isNull() || other.isNull()) return TerrainRect();
    TerrainRect result;
    result.left = left > other.left ? left : other.left;
    result.right = right < other.right ? right : other.right;
    result.top = top > other.top ? top : other.top;
    result.bottom = bottom < other.bottom ? bottom : other.bottom;
    if (result.left >= result.right || result.top >= result.bottom) return TerrainRect();
    return result;
}

std::uint64_t TerrainIndexKey::pack() const
{
    std::uint64_t key = batchSize;
    key = (key << 8) | (vdatasize & 0xffu);
    key = (key << 8) | (vertexIncrement & 0xffu);
    key = (key << 8) | (xoffset & 0xffu);
    key = (key << 8) | (yoffset & 0xffu);
    key = (key << 8) | (numSkirtRowsCols & 0xffu);
    key = (key << 8) | (skirtRowColSkip & 0xffu);
    return key;
}

Terrain::Terrain()
        : mPrepared(false),
          mLoaded(false),
          mAlign(TerrainAlignment::AlignXZ),
          mWorldSize(0.0f),
          mSize(0),
          mMaxBatchSize(0),
          mMinBatchSize(0),
          mPos(0.0f, 0.0f, 0.0f),
          mQuadTree(nullptr),
          mNumLodLevels(0),
          mNumLodLevelsPerLeafNode(0),
          mTreeDepth(0),
          mBase(0.0f),
          mScale(0.0f),
          mSkirtSize(0.0f),
          mMaxPixelError(3.0f),
          mUseRayBoxDistanceCalculation(false),
          mLodMorphRequired(false),
          mHeightData(),
          mDeltaData(),
          mTarget(nullptr),
          mGeometryTarget(nullptr),
          mVertexScratch(nullptr),
          mDirtyGeometryRect(),
          mLayers(),
          mLayerUVMultiplier(),
          mLayerBlendMapList(),
          mBlendTextureData(),
          mMaterialDirty(false),
          mMaterialParamsDirty(false),
          mLayerBlendMapSize(1024),
          mLightmapSize(1024),
          mCompositeMapSize(1024),
          mGlobalColourMapSize(1024),
          mDefaultGlobalColourMapSize(1024),
          mGlobalColourMapEnabled(false),
          mDefaultLayerTextureWorldSize(10.0f),
          mLightMapDir(0.70710678f, -0.70710678f, 0.0f),
          mCompositeMapAmbient(1.0f, 1.0f, 1.0f),
          mCompositeMapDiffuse(1.0f, 1.0f, 1.0f),
          mCompositeMapDistance(4000.0f),
          mNormalMapRequired(false),
          mLightMapRequired(false),
          mLightMapShadowsOnly(true),
          mCompositeMapRequired(false),
          mNormalMapReady(false),
          mLightmapReady(false),
          mLightmapPadValid(false),
          mLightmapPadFollowsRange(true),
          mLightmapPad(0.0f),
          mColourMapReady(false),
          mCompositeMapReady(false),
          mDirtyDerivedDataRect(),
          mDirtyLightmapFromNeighboursRect(),
          mCompositeMapDirtyRect(),
          mCompositeMapDirtyRectLightmapUpdate(false),
          mCompositeMapUpdateCountdown(0)
{
}

Terrain::~Terrain()
{
    freeAll();
    delete mVertexScratch;
}

void Terrain::freeAll()
{
    freeMaps();
    delete mQuadTree;
    mQuadTree = nullptr;
    mDirtyGeometryRect = TerrainRect();
    mHeightData.clear();
    mHeightData.shrink_to_fit();
    mDeltaData.clear();
    mDeltaData.shrink_to_fit();
    mPrepared = false;
    mLoaded = false;
}

void Terrain::unprepare()
{
    freeAll();
}

bool Terrain::isPrepared() const
{
    return mPrepared;
}

void Terrain::load()
{
    if (!mPrepared || !mQuadTree) return;
    mQuadTree->load();
    mLoaded = true;
}

void Terrain::unload()
{
    if (mQuadTree) mQuadTree->unload();
    mLoaded = false;
}

bool Terrain::isLoaded() const
{
    return mLoaded;
}

bool Terrain::prepare(const TerrainImportData& importData)
{
    freeAll();

    if (!isPowerOfTwoPlusOne(importData.terrainSize) ||
            !isPowerOfTwoPlusOne(importData.minBatchSize) ||
            !isPowerOfTwoPlusOne(importData.maxBatchSize))
    {
        log_error("terrain: terrainSize, minBatchSize and maxBatchSize must all be 2^n + 1");
        return false;
    }
    if (importData.minBatchSize > importData.maxBatchSize)
    {
        log_error("terrain: minBatchSize must be less than or equal to maxBatchSize");
        return false;
    }
    if (importData.maxBatchSize > kMaxBatchSize)
    {
        log_error("terrain: maxBatchSize must be no larger than 129");
        return false;
    }
    if (importData.minBatchSize > importData.terrainSize)
    {
        log_error("terrain: minBatchSize must not exceed terrainSize");
        return false;
    }

    mSkirtSize = importData.skirtSize;
    mMaxPixelError = importData.maxPixelError;
    mUseRayBoxDistanceCalculation = importData.useRayBoxDistanceCalculation;
    mLayerBlendMapSize = importData.layerBlendMapSize;
    mLightmapSize = importData.lightMapSize;
    mCompositeMapSize = importData.compositeMapSize;
    mDefaultGlobalColourMapSize = importData.defaultGlobalColourMapSize;
    mDefaultLayerTextureWorldSize = importData.defaultLayerTextureWorldSize;
    mLightMapDir = importData.lightMapDirection;
    mCompositeMapAmbient = importData.compositeMapAmbient;
    mCompositeMapDiffuse = importData.compositeMapDiffuse;
    mCompositeMapDistance = importData.compositeMapDistance;

    mAlign = importData.terrainAlign;
    mSize = importData.terrainSize;
    mWorldSize = importData.worldSize;
    mLayers = importData.layerList;
    checkLayers(false);
    deriveUVMultipliers();
    mMaxBatchSize = importData.maxBatchSize;
    mMinBatchSize = importData.minBatchSize;
    mPos = importData.pos;

    updateBaseScale();
    determineLodLevels();

    const std::size_t numVertices = static_cast<std::size_t>(mSize) * mSize;
    mHeightData.resize(numVertices);

    if (importData.inputFloat)
    {
        if (fabsf(importData.inputBias) <= FLT_EPSILON &&
                fabsf(importData.inputScale - 1.0f) <= FLT_EPSILON)
        {
            for (std::size_t i = 0; i < numVertices; ++i) mHeightData[i] = importData.inputFloat[i];
        }
        else
        {
            for (std::size_t i = 0; i < numVertices; ++i)
                mHeightData[i] =
                        (importData.inputFloat[i] * importData.inputScale) + importData.inputBias;
        }
    }
    else if (importData.inputImage || importData.inputImage16)
    {
        int width = 0;
        int height = 0;
        ct::Vector<float> source;
        if (importData.inputImage)
        {
            const Pixmap& image = *importData.inputImage;
            if (!image.is_valid() || image.components < 1)
            {
                log_error("terrain: the height image is not valid");
                freeAll();
                return false;
            }
            width = image.width;
            height = image.height;
            source.resize(static_cast<std::size_t>(width) * height);
            for (int y = 0; y < height; ++y)
                for (int x = 0; x < width; ++x)
                    source[static_cast<std::size_t>(y) * width + x] =
                            static_cast<float>(
                                    image.pixels[(static_cast<std::size_t>(y) * width + x) *
                                                 image.components]) /
                            255.0f;
        }
        else
        {
            width = importData.inputImage16Width;
            height = importData.inputImage16Height;
            if (width < 2 || height < 2)
            {
                log_error("terrain: the 16-bit height image has no size");
                freeAll();
                return false;
            }
            source.resize(static_cast<std::size_t>(width) * height);
            for (std::size_t i = 0; i < source.size(); ++i)
                source[i] = static_cast<float>(importData.inputImage16[i]) / 65535.0f;
        }

        ct::Vector<float> resampled;
        const ct::Vector<float>* data = &source;
        if (width != mSize || height != mSize)
        {
            resampleBilinear(source, width, height, resampled, mSize);
            data = &resampled;
        }

        for (std::uint16_t i = 0; i < mSize; ++i)
        {
            const std::uint32_t srcy = static_cast<std::uint32_t>(mSize) - i - 1;
            for (std::uint16_t x = 0; x < mSize; ++x)
                mHeightData[static_cast<std::size_t>(i) * mSize + x] =
                        (*data)[static_cast<std::size_t>(srcy) * mSize + x];
        }

        if (fabsf(importData.inputBias) > FLT_EPSILON ||
                fabsf(importData.inputScale - 1.0f) > FLT_EPSILON)
        {
            for (std::size_t i = 0; i < numVertices; ++i)
                mHeightData[i] = (mHeightData[i] * importData.inputScale) + importData.inputBias;
        }
    }
    else
    {
        for (std::size_t i = 0; i < numVertices; ++i) mHeightData[i] = importData.constantHeight;
    }

    mDeltaData.resize(numVertices);
    for (std::size_t i = 0; i < numVertices; ++i) mDeltaData[i] = 0.0f;

    mQuadTree = new TerrainQuadTreeNode(this, nullptr, 0, 0, mSize,
            static_cast<std::uint16_t>(mNumLodLevels - 1), 0);

    const TerrainRect rect(0, 0, mSize, mSize);
    calculateHeightDeltas(rect);
    finaliseHeightDeltas(rect);

    distributeVertexData();
    mQuadTree->refreshBounds(rect);

    mPrepared = true;
    return true;
}

void Terrain::determineLodLevels()
{
    mNumLodLevelsPerLeafNode = static_cast<std::uint16_t>(
            log2Exact(mMaxBatchSize - 1u) - log2Exact(mMinBatchSize - 1u) + 1);
    mNumLodLevels =
            static_cast<std::uint16_t>(log2Exact(mSize - 1u) - log2Exact(mMinBatchSize - 1u) + 1);
    mTreeDepth = static_cast<std::uint16_t>(mNumLodLevels - mNumLodLevelsPerLeafNode + 1);
}

void Terrain::distributeVertexData()
{
    std::uint16_t depth = mTreeDepth;
    std::uint16_t prevdepth = depth;
    std::uint16_t currresolution = mSize;
    std::uint16_t bakedresolution = mSize;
    std::uint16_t targetSplits =
            static_cast<std::uint16_t>((bakedresolution - 1) / (kMaxBatchSize - 1));
    while (depth-- && targetSplits)
    {
        const std::uint32_t splits = 1u << depth;
        if (splits == targetSplits)
        {
            const std::uint32_t sz = ((bakedresolution - 1u) / splits) + 1u;
            mQuadTree->assignVertexData(depth, prevdepth, bakedresolution, sz);

            bakedresolution = static_cast<std::uint16_t>(((currresolution - 1) >> 1) + 1);
            targetSplits = static_cast<std::uint16_t>((bakedresolution - 1) / (kMaxBatchSize - 1));
            prevdepth = depth;
        }

        currresolution = static_cast<std::uint16_t>(((currresolution - 1) >> 1) + 1);
    }

    if (prevdepth > 0) mQuadTree->assignVertexData(0, 1, bakedresolution, bakedresolution);
}

void Terrain::updateBaseScale()
{
    mBase = -mWorldSize * 0.5f;
    mScale = mWorldSize / static_cast<float>(mSize - 1);
}

const Math::Box& Terrain::getAABB() const
{
    if (!mQuadTree) return Math::Box::Empty;
    return mQuadTree->getBoundingBox();
}

Math::Box Terrain::getWorldAABB() const
{
    const Math::Box box = getAABB();
    if (terrain_math::isNullBox(box)) return box;
    return Math::Box(box.min + mPos, box.max + mPos);
}

float Terrain::getMinHeight() const
{
    return mQuadTree ? mQuadTree->getMinHeight() : 0.0f;
}

float Terrain::getMaxHeight() const
{
    return mQuadTree ? mQuadTree->getMaxHeight() : 0.0f;
}

float Terrain::getBoundingRadius() const
{
    return mQuadTree ? mQuadTree->getBoundingRadius() : 0.0f;
}

float* Terrain::getHeightData()
{
    return mHeightData.data();
}

const float* Terrain::getHeightData() const
{
    return mHeightData.data();
}

float* Terrain::getHeightData(std::uint32_t x, std::uint32_t y)
{
    return &mHeightData[static_cast<std::size_t>(y) * mSize + x];
}

const float* Terrain::getHeightData(std::uint32_t x, std::uint32_t y) const
{
    return &mHeightData[static_cast<std::size_t>(y) * mSize + x];
}

const float* Terrain::getDeltaData() const
{
    return mDeltaData.data();
}

const float* Terrain::getDeltaData(std::uint32_t x, std::uint32_t y) const
{
    return &mDeltaData[static_cast<std::size_t>(y) * mSize + x];
}

float Terrain::getHeightAtPoint(std::uint32_t x, std::uint32_t y) const
{
    x = x < mSize - 1u ? x : mSize - 1u;
    y = y < mSize - 1u ? y : mSize - 1u;
    return *getHeightData(x, y);
}

Math::Vec4 Terrain::heightPlaneAt(float x, float y) const
{
    const float factor = static_cast<float>(mSize) - 1.0f;
    const float invFactor = 1.0f / factor;

    const std::uint32_t startX = static_cast<std::uint32_t>(x * factor);
    const std::uint32_t startY = static_cast<std::uint32_t>(y * factor);
    std::uint32_t endX = startX + 1;
    std::uint32_t endY = startY + 1;

    const float startXTS = startX * invFactor;
    const float startYTS = startY * invFactor;
    const float endXTS = endX * invFactor;
    const float endYTS = endY * invFactor;

    endX = endX < mSize - 1u ? endX : mSize - 1u;
    endY = endY < mSize - 1u ? endY : mSize - 1u;

    const float xParam = (x - startXTS) / invFactor;
    const float yParam = (y - startYTS) / invFactor;

    const Math::Vec3 v0(startXTS, startYTS, getHeightAtPoint(startX, startY));
    const Math::Vec3 v1(endXTS, startYTS, getHeightAtPoint(endX, startY));
    const Math::Vec3 v2(endXTS, endYTS, getHeightAtPoint(endX, endY));
    const Math::Vec3 v3(startXTS, endYTS, getHeightAtPoint(startX, endY));
    if (startY % 2)
    {
        const bool secondTri = ((1.0 - yParam) > xParam);
        if (secondTri) return terrain_math::faceNormalPlaneWithoutNormalize(v0, v1, v3);
        return terrain_math::faceNormalPlaneWithoutNormalize(v1, v2, v3);
    }
    const bool secondTri = (yParam > xParam);
    if (secondTri) return terrain_math::faceNormalPlaneWithoutNormalize(v0, v2, v3);
    return terrain_math::faceNormalPlaneWithoutNormalize(v0, v1, v2);
}

float Terrain::getHeightAtTerrainPosition(float x, float y) const
{
    x = clampf(x, 0.0f, 1.0f);
    y = clampf(y, 0.0f, 1.0f);
    const Math::Vec4 plane = heightPlaneAt(x, y);
    return (-plane.x * x - plane.y * y - plane.w) / plane.z;
}

float Terrain::getSlopeAtTerrainPosition(float x, float y) const
{
    x = clampf(x, 0.0f, 1.0f);
    y = clampf(y, 0.0f, 1.0f);
    const Math::Vec4 plane = heightPlaneAt(x, y);
    const float gradientX = -plane.x / plane.z / mWorldSize;
    const float gradientY = -plane.y / plane.z / mWorldSize;
    return atanf(sqrtf(gradientX * gradientX + gradientY * gradientY)) *
           (180.0f / 3.14159265358979f);
}

float Terrain::getSlopeAtWorldPosition(const Math::Vec3& pos) const
{
    Math::Vec3 terrPos;
    getTerrainPosition(pos, terrPos);
    return getSlopeAtTerrainPosition(terrPos.x, terrPos.y);
}

float Terrain::getHeightAtWorldPosition(float x, float y, float z) const
{
    Math::Vec3 terrPos;
    getTerrainPosition(x, y, z, terrPos);
    return getHeightAtTerrainPosition(terrPos.x, terrPos.y);
}

float Terrain::getHeightAtWorldPosition(const Math::Vec3& pos) const
{
    return getHeightAtWorldPosition(pos.x, pos.y, pos.z);
}

Math::Vec3 Terrain::convertPosition(TerrainSpace inSpace, const Math::Vec3& inPos,
        TerrainSpace outSpace) const
{
    Math::Vec3 ret;
    convertPosition(inSpace, inPos, outSpace, ret);
    return ret;
}

Math::Vec3 Terrain::convertDirection(TerrainSpace inSpace, const Math::Vec3& inDir,
        TerrainSpace outSpace) const
{
    Math::Vec3 ret;
    convertDirection(inSpace, inDir, outSpace, ret);
    return ret;
}

void Terrain::convertPosition(TerrainSpace inSpace, const Math::Vec3& inPos, TerrainSpace outSpace,
        Math::Vec3& outPos) const
{
    convertSpace(inSpace, inPos, outSpace, outPos, true);
}

void Terrain::convertDirection(TerrainSpace inSpace, const Math::Vec3& inDir, TerrainSpace outSpace,
        Math::Vec3& outDir) const
{
    convertSpace(inSpace, inDir, outSpace, outDir, false);
}

void Terrain::convertSpace(TerrainSpace inSpace, const Math::Vec3& inVec, TerrainSpace outSpace,
        Math::Vec3& outVec, bool translation) const
{
    TerrainSpace currSpace = inSpace;
    outVec = inVec;
    while (currSpace != outSpace)
    {
        switch (currSpace)
        {
            case TerrainSpace::World:
                if (translation) outVec -= mPos;
                currSpace = TerrainSpace::Local;
                break;
            case TerrainSpace::Local:
                switch (outSpace)
                {
                    case TerrainSpace::World:
                        if (translation) outVec += mPos;
                        currSpace = TerrainSpace::World;
                        break;
                    case TerrainSpace::Point:
                    case TerrainSpace::Terrain:
                        outVec = convertWorldToTerrainAxes(outVec);
                        if (translation)
                        {
                            outVec.x -= mBase;
                            outVec.y -= mBase;
                            outVec.x /= (mSize - 1) * mScale;
                            outVec.y /= (mSize - 1) * mScale;
                        }
                        currSpace = TerrainSpace::Terrain;
                        break;
                    case TerrainSpace::Local:
                    default:
                        break;
                }
                break;
            case TerrainSpace::Terrain:
                switch (outSpace)
                {
                    case TerrainSpace::World:
                    case TerrainSpace::Local:
                        if (translation)
                        {
                            outVec.x *= (mSize - 1) * mScale;
                            outVec.y *= (mSize - 1) * mScale;
                            outVec.x += mBase;
                            outVec.y += mBase;
                        }
                        outVec = convertTerrainToWorldAxes(outVec);
                        currSpace = TerrainSpace::Local;
                        break;
                    case TerrainSpace::Point:
                        if (translation)
                        {
                            outVec.x *= (mSize - 1);
                            outVec.y *= (mSize - 1);
                            outVec.x = static_cast<float>(static_cast<int>(outVec.x + 0.5f));
                            outVec.y = static_cast<float>(static_cast<int>(outVec.y + 0.5f));
                        }
                        currSpace = TerrainSpace::Point;
                        break;
                    case TerrainSpace::Terrain:
                    default:
                        break;
                }
                break;
            case TerrainSpace::Point:
                if (translation)
                {
                    outVec.x /= (mSize - 1);
                    outVec.y /= (mSize - 1);
                }
                currSpace = TerrainSpace::Terrain;
                break;
        }
    }
}

void Terrain::convertWorldToTerrainAxes(TerrainAlignment align, const Math::Vec3& worldVec,
        Math::Vec3& terrainVec)
{
    switch (align)
    {
        case TerrainAlignment::AlignXZ:
            terrainVec.z = worldVec.y;
            terrainVec.x = worldVec.x;
            terrainVec.y = -worldVec.z;
            break;
        case TerrainAlignment::AlignYZ:
            terrainVec.z = worldVec.x;
            terrainVec.x = -worldVec.z;
            terrainVec.y = worldVec.y;
            break;
        case TerrainAlignment::AlignXY:
            terrainVec = worldVec;
            break;
    }
}

void Terrain::convertTerrainToWorldAxes(TerrainAlignment align, const Math::Vec3& terrainVec,
        Math::Vec3& worldVec)
{
    switch (align)
    {
        case TerrainAlignment::AlignXZ:
            worldVec.x = terrainVec.x;
            worldVec.y = terrainVec.z;
            worldVec.z = -terrainVec.y;
            break;
        case TerrainAlignment::AlignYZ:
            worldVec.x = terrainVec.z;
            worldVec.y = terrainVec.y;
            worldVec.z = -terrainVec.x;
            break;
        case TerrainAlignment::AlignXY:
            worldVec = terrainVec;
            break;
    }
}

Math::Vec3 Terrain::convertWorldToTerrainAxes(const Math::Vec3& inVec) const
{
    Math::Vec3 ret;
    convertWorldToTerrainAxes(mAlign, inVec, ret);
    return ret;
}

Math::Vec3 Terrain::convertTerrainToWorldAxes(const Math::Vec3& inVec) const
{
    Math::Vec3 ret;
    convertTerrainToWorldAxes(mAlign, inVec, ret);
    return ret;
}

void Terrain::getPoint(std::uint32_t x, std::uint32_t y, Math::Vec3& outpos) const
{
    getPointAlign(x, y, mAlign, outpos);
}

void Terrain::getPoint(std::uint32_t x, std::uint32_t y, float height, Math::Vec3& outpos) const
{
    getPointAlign(x, y, height, mAlign, outpos);
}

void Terrain::getPointAlign(std::uint32_t x, std::uint32_t y, TerrainAlignment align,
        Math::Vec3& outpos) const
{
    getPointAlign(x, y, *getHeightData(x, y), align, outpos);
}

void Terrain::getPointAlign(std::uint32_t x, std::uint32_t y, float height, TerrainAlignment align,
        Math::Vec3& outpos) const
{
    switch (align)
    {
        case TerrainAlignment::AlignXZ:
            outpos.y = height;
            outpos.x = x * mScale + mBase;
            outpos.z = y * -mScale - mBase;
            break;
        case TerrainAlignment::AlignYZ:
            outpos.x = height;
            outpos.z = x * -mScale - mBase;
            outpos.y = y * mScale + mBase;
            break;
        case TerrainAlignment::AlignXY:
            outpos.z = height;
            outpos.x = x * mScale + mBase;
            outpos.y = y * mScale + mBase;
            break;
    }
}

void Terrain::getVector(const Math::Vec3& inVec, Math::Vec3& outVec) const
{
    getVectorAlign(inVec.x, inVec.y, inVec.z, mAlign, outVec);
}

void Terrain::getVector(float x, float y, float z, Math::Vec3& outVec) const
{
    getVectorAlign(x, y, z, mAlign, outVec);
}

void Terrain::getVectorAlign(const Math::Vec3& inVec, TerrainAlignment align,
        Math::Vec3& outVec) const
{
    getVectorAlign(inVec.x, inVec.y, inVec.z, align, outVec);
}

void Terrain::getVectorAlign(float x, float y, float z, TerrainAlignment align,
        Math::Vec3& outVec) const
{
    switch (align)
    {
        case TerrainAlignment::AlignXZ:
            outVec.y = z;
            outVec.x = x;
            outVec.z = -y;
            break;
        case TerrainAlignment::AlignYZ:
            outVec.x = z;
            outVec.y = y;
            outVec.z = -x;
            break;
        case TerrainAlignment::AlignXY:
            outVec.x = x;
            outVec.y = y;
            outVec.z = z;
            break;
    }
}

void Terrain::getPosition(const Math::Vec3& tsPos, Math::Vec3& outWsPos) const
{
    getPositionAlign(tsPos, mAlign, outWsPos);
}

void Terrain::getPosition(float x, float y, float z, Math::Vec3& outWsPos) const
{
    getPositionAlign(x, y, z, mAlign, outWsPos);
}

void Terrain::getTerrainPosition(const Math::Vec3& wsPos, Math::Vec3& outTsPos) const
{
    getTerrainPositionAlign(wsPos, mAlign, outTsPos);
}

void Terrain::getTerrainPosition(float x, float y, float z, Math::Vec3& outTsPos) const
{
    getTerrainPositionAlign(x, y, z, mAlign, outTsPos);
}

void Terrain::getPositionAlign(const Math::Vec3& tsPos, TerrainAlignment align,
        Math::Vec3& outWsPos) const
{
    getPositionAlign(tsPos.x, tsPos.y, tsPos.z, align, outWsPos);
}

void Terrain::getPositionAlign(float x, float y, float z, TerrainAlignment align,
        Math::Vec3& outWsPos) const
{
    switch (align)
    {
        case TerrainAlignment::AlignXZ:
            outWsPos.y = z;
            outWsPos.x = x * (mSize - 1) * mScale + mBase;
            outWsPos.z = y * (mSize - 1) * -mScale - mBase;
            break;
        case TerrainAlignment::AlignYZ:
            outWsPos.x = z;
            outWsPos.y = y * (mSize - 1) * mScale + mBase;
            outWsPos.z = x * (mSize - 1) * -mScale - mBase;
            break;
        case TerrainAlignment::AlignXY:
            outWsPos.z = z;
            outWsPos.x = x * (mSize - 1) * mScale + mBase;
            outWsPos.y = y * (mSize - 1) * mScale + mBase;
            break;
    }
}

void Terrain::getTerrainPositionAlign(const Math::Vec3& wsPos, TerrainAlignment align,
        Math::Vec3& outTsPos) const
{
    getTerrainPositionAlign(wsPos.x, wsPos.y, wsPos.z, align, outTsPos);
}

void Terrain::getTerrainPositionAlign(float x, float y, float z, TerrainAlignment align,
        Math::Vec3& outTsPos) const
{
    switch (align)
    {
        case TerrainAlignment::AlignXZ:
            outTsPos.x = (x - mBase - mPos.x) / ((mSize - 1) * mScale);
            outTsPos.y = (z + mBase - mPos.z) / ((mSize - 1) * -mScale);
            outTsPos.z = y;
            break;
        case TerrainAlignment::AlignYZ:
            outTsPos.x = (z - mBase - mPos.z) / ((mSize - 1) * -mScale);
            outTsPos.y = (y + mBase - mPos.y) / ((mSize - 1) * mScale);
            outTsPos.z = x;
            break;
        case TerrainAlignment::AlignXY:
            outTsPos.x = (x - mBase - mPos.x) / ((mSize - 1) * mScale);
            outTsPos.y = (y - mBase - mPos.y) / ((mSize - 1) * mScale);
            outTsPos.z = z;
            break;
    }
}

void Terrain::getTerrainVector(const Math::Vec3& inVec, Math::Vec3& outVec) const
{
    getTerrainVectorAlign(inVec.x, inVec.y, inVec.z, mAlign, outVec);
}

void Terrain::getTerrainVector(float x, float y, float z, Math::Vec3& outVec) const
{
    getTerrainVectorAlign(x, y, z, mAlign, outVec);
}

void Terrain::getTerrainVectorAlign(const Math::Vec3& inVec, TerrainAlignment align,
        Math::Vec3& outVec) const
{
    getTerrainVectorAlign(inVec.x, inVec.y, inVec.z, align, outVec);
}

void Terrain::getTerrainVectorAlign(float x, float y, float z, TerrainAlignment align,
        Math::Vec3& outVec) const
{
    switch (align)
    {
        case TerrainAlignment::AlignXZ:
            outVec.z = y;
            outVec.x = x;
            outVec.y = -z;
            break;
        case TerrainAlignment::AlignYZ:
            outVec.z = x;
            outVec.y = y;
            outVec.x = -z;
            break;
        case TerrainAlignment::AlignXY:
            outVec.x = x;
            outVec.y = y;
            outVec.z = z;
            break;
    }
}

TerrainAlignment Terrain::getAlignment() const
{
    return mAlign;
}

std::uint16_t Terrain::getSize() const
{
    return mSize;
}

std::uint16_t Terrain::getMaxBatchSize() const
{
    return mMaxBatchSize;
}

std::uint16_t Terrain::getMinBatchSize() const
{
    return mMinBatchSize;
}

float Terrain::getWorldSize() const
{
    return mWorldSize;
}

const Math::Vec3& Terrain::getPosition() const
{
    return mPos;
}

void Terrain::setPosition(const Math::Vec3& pos)
{
    if (pos != mPos)
    {
        mPos = pos;
        updateBaseScale();
    }
}

float Terrain::getBase() const
{
    return mBase;
}

float Terrain::getScale() const
{
    return mScale;
}

float Terrain::getSkirtSize() const
{
    return mSkirtSize;
}

void Terrain::setSkirtSize(float skirtSize)
{
    mSkirtSize = skirtSize;
}

float Terrain::getMaxPixelError() const
{
    return mMaxPixelError;
}

void Terrain::setMaxPixelError(float maxPixelError)
{
    mMaxPixelError = maxPixelError;
}

bool Terrain::getUseRayBoxDistanceCalculation() const
{
    return mUseRayBoxDistanceCalculation;
}

void Terrain::setUseRayBoxDistanceCalculation(bool rayBox)
{
    mUseRayBoxDistanceCalculation = rayBox;
}

bool Terrain::_getMorphRequired() const
{
    return mLodMorphRequired;
}

void Terrain::_setMorphRequired(bool morph)
{
    mLodMorphRequired = morph;
}

bool Terrain::_getUseVertexCompression() const
{
    return false;
}

std::uint16_t Terrain::getNumLodLevels() const
{
    return mNumLodLevels;
}

std::uint16_t Terrain::getNumLodLevelsPerLeaf() const
{
    return mNumLodLevelsPerLeafNode;
}

std::uint16_t Terrain::getTreeDepth() const
{
    return mTreeDepth;
}

std::uint16_t Terrain::getResolutionAtLod(std::uint16_t lodLevel) const
{
    return static_cast<std::uint16_t>(((mSize - 1) >> lodLevel) + 1);
}

std::uint32_t Terrain::getGeoDataSizeAtLod(std::uint16_t lodLevel) const
{
    const std::uint32_t size = getResolutionAtLod(lodLevel);
    const std::uint32_t prevSize =
            (lodLevel < mNumLodLevels - 1)
                    ? getResolutionAtLod(static_cast<std::uint16_t>(lodLevel + 1))
                    : 0;
    return size * size - prevSize * prevSize;
}

std::uint16_t Terrain::getLODLevelWhenVertexEliminated(long x, long y) const
{
    const std::uint16_t a = getLODLevelWhenVertexEliminated(x);
    const std::uint16_t b = getLODLevelWhenVertexEliminated(y);
    return a < b ? a : b;
}

std::uint16_t Terrain::getLODLevelWhenVertexEliminated(long rowOrColumn) const
{
    std::uint16_t currentElim = static_cast<std::uint16_t>((mSize - 1) / (mMinBatchSize - 1));
    std::uint16_t currentLod = mNumLodLevels;

    while (rowOrColumn % currentElim)
    {
        currentElim = static_cast<std::uint16_t>(currentElim / 2);
        --currentLod;
    }

    return currentLod;
}

TerrainQuadTreeNode* Terrain::getQuadTree() const
{
    return mQuadTree;
}

namespace
{

void collectDeltaNodes(TerrainQuadTreeNode* node, int numLodLevels,
        const ct::Vector<TerrainRect>& widened, ct::Vector<TerrainQuadTreeNode*>& out)
{
    const TerrainRect region(node->getXOffset(), node->getYOffset(),
            node->getXOffset() + node->getSize(), node->getYOffset() + node->getSize());
    bool affected = false;
    for (std::uint16_t i = 0; i < node->getLodCount(); ++i)
    {
        const int lod = node->getBaseLod() + i;
        if (lod <= numLodLevels - 2 &&
                !widened[static_cast<std::size_t>(lod)].intersect(region).isNull())
            affected = true;
    }
    if (affected) out.push_back(node);
    if (!node->isLeaf())
        for (unsigned short i = 0; i < 4; ++i)
            collectDeltaNodes(node->getChild(i), numLodLevels, widened, out);
}

} // namespace

void Terrain::calculateNodeDeltas(TerrainQuadTreeNode* node, int sourceLevel)
{
    const int step = 1 << (sourceLevel + 1);
    const int last = static_cast<int>(mSize) - 1;
    const int nodeLeft = node->getXOffset();
    const int nodeTop = node->getYOffset();
    const int nodeRight = nodeLeft + node->getSize() < last ? nodeLeft + node->getSize() : last;
    const int nodeBottom = nodeTop + node->getSize() < last ? nodeTop + node->getSize() : last;

    for (int j = nodeTop - nodeTop % step; j < nodeBottom; j += step)
    {
        for (int i = nodeLeft - nodeLeft % step; i < nodeRight; i += step)
        {
            Math::Vec3 v0, v1, v2, v3;
            getPointAlign(i, j, TerrainAlignment::AlignXY, v0);
            getPointAlign(i + step, j, TerrainAlignment::AlignXY, v1);
            getPointAlign(i, j + step, TerrainAlignment::AlignXY, v2);
            getPointAlign(i + step, j + step, TerrainAlignment::AlignXY, v3);

            Math::Vec4 t1, t2;
            bool backwardTri = false;
            if ((j / step) % 2 == 0)
            {
                t1 = terrain_math::faceNormalPlaneWithoutNormalize(v0, v1, v3);
                t2 = terrain_math::faceNormalPlaneWithoutNormalize(v0, v3, v2);
            }
            else
            {
                t1 = terrain_math::faceNormalPlaneWithoutNormalize(v1, v3, v2);
                t2 = terrain_math::faceNormalPlaneWithoutNormalize(v0, v1, v2);
                backwardTri = true;
            }

            const int yFirst = nodeTop > j ? nodeTop - j : 0;
            const int yEnd = nodeBottom < j + step ? nodeBottom - j : step;
            const int xFirst = nodeLeft > i ? nodeLeft - i : 0;
            const int xEnd = nodeRight < i + step ? nodeRight - i : step;
            const int halfStep = step / 2;
            float blockMax = 0.0f;
            for (int y = yFirst; y < yEnd; y++)
            {
                const int fulldetaily = j + y;
                const float ypct = static_cast<float>(y) / static_cast<float>(step);
                const float ay = static_cast<float>(fulldetaily) * mScale + mBase;
                const float* heightRow =
                        &mHeightData[static_cast<std::size_t>(fulldetaily) * mSize];
                float* deltaRow = &mDeltaData[static_cast<std::size_t>(fulldetaily) * mSize];
                const bool rowMultiple = fulldetaily % step == 0;
                for (int x = xFirst; x < xEnd; x++)
                {
                    const int fulldetailx = i + x;
                    if (rowMultiple && fulldetailx % step == 0) continue;

                    const float xpct = static_cast<float>(x) / static_cast<float>(step);

                    const float ax = static_cast<float>(fulldetailx) * mScale + mBase;
                    float interp_h;
                    if ((xpct > ypct && !backwardTri) || (xpct > (1 - ypct) && backwardTri))
                    {
                        interp_h = (-t1.x * ax - t1.y * ay - t1.w) / t1.z;
                    }
                    else
                    {
                        interp_h = (-t2.x * ax - t2.y * ay - t2.w) / t2.z;
                    }

                    const float delta = interp_h - heightRow[fulldetailx];
                    blockMax = blockMax > delta ? blockMax : delta;

                    if (((fulldetailx % step) == halfStep && (fulldetaily % halfStep) == 0) ||
                            ((fulldetaily % step) == halfStep && (fulldetailx % halfStep) == 0))
                        deltaRow[fulldetailx] = delta;
                }
            }
            node->noteDirectDelta(static_cast<std::uint16_t>(sourceLevel), blockMax);
        }
    }
}

TerrainRect Terrain::calculateHeightDeltas(const TerrainRect& rect)
{
    const TerrainRect clampedRect = rect.intersect(TerrainRect(0, 0, mSize, mSize));
    TerrainRect finalRect(clampedRect);

    ct::Vector<TerrainRect> widened;
    widened.resize(static_cast<std::size_t>(mNumLodLevels));
    for (int targetLevel = 1; targetLevel < mNumLodLevels; ++targetLevel)
    {
        const int step = 1 << targetLevel;

        TerrainRect widenedRect(rect);
        widenedRect.left = widenedRect.left - step > 0 ? widenedRect.left - step : 0;
        widenedRect.top = widenedRect.top - step > 0 ? widenedRect.top - step : 0;
        widenedRect.right = widenedRect.right + step < static_cast<int>(mSize)
                                    ? widenedRect.right + step
                                    : static_cast<int>(mSize);
        widenedRect.bottom = widenedRect.bottom + step < static_cast<int>(mSize)
                                     ? widenedRect.bottom + step
                                     : static_cast<int>(mSize);

        finalRect.merge(widenedRect);
        widened[static_cast<std::size_t>(targetLevel - 1)] = widenedRect;
    }

    ct::Vector<TerrainQuadTreeNode*> nodes;
    nodes.reserve(64);
    collectDeltaNodes(mQuadTree, mNumLodLevels, widened, nodes);
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        TerrainQuadTreeNode* node = nodes[i];
        node->resetDirectDeltas();
        for (std::uint16_t l = 0; l < node->getLodCount(); ++l)
        {
            const int lod = node->getBaseLod() + l;
            if (lod <= mNumLodLevels - 2) calculateNodeDeltas(node, lod);
        }
    }

    mQuadTree->postDeltaCalculation();

    return finalRect;
}

void Terrain::finaliseHeightDeltas(const TerrainRect& rect, bool cpuData)
{
    const TerrainRect clampedRect = rect.intersect(TerrainRect(0, 0, mSize, mSize));
    mQuadTree->finaliseDeltaValues();
    mQuadTree->updateVertexData(false, true, clampedRect, cpuData);
}

float Terrain::getCFactor(const TerrainLodView& view) const
{
    const float A = 1.0f / tanf(view.fovY * 0.5f);
    const float maxPixelError = mMaxPixelError * view.lodBiasInverse;
    const float T = 2.0f * maxPixelError / view.viewportHeight;
    return A / T;
}

void Terrain::calculateCurrentLod(const TerrainLodView& view)
{
    if (!mQuadTree) return;
    mQuadTree->calculateCurrentLod(view, getCFactor(view));
}

Math::Vec3 Terrain::calculateNormal(int x, int y) const
{
    auto fetch = [this](int px, int py, Math::Vec3& out)
    {
        px = px < static_cast<int>(mSize) - 1 ? px : static_cast<int>(mSize) - 1;
        py = py < static_cast<int>(mSize) - 1 ? py : static_cast<int>(mSize) - 1;
        px = px > 0 ? px : 0;
        py = py > 0 ? py : 0;
        getPoint(static_cast<std::uint32_t>(px), static_cast<std::uint32_t>(py), out);
    };

    Math::Vec3 cumulativeNormal(0.0f, 0.0f, 0.0f);
    Math::Vec3 centrePoint;
    Math::Vec3 adjacentPoints[8];
    fetch(x, y, centrePoint);
    fetch(x + 1, y, adjacentPoints[0]);
    fetch(x + 1, y + 1, adjacentPoints[1]);
    fetch(x, y + 1, adjacentPoints[2]);
    fetch(x - 1, y + 1, adjacentPoints[3]);
    fetch(x - 1, y, adjacentPoints[4]);
    fetch(x - 1, y - 1, adjacentPoints[5]);
    fetch(x, y - 1, adjacentPoints[6]);
    fetch(x + 1, y - 1, adjacentPoints[7]);

    for (int i = 0; i < 8; ++i)
        cumulativeNormal += terrain_math::basicFaceNormal(centrePoint, adjacentPoints[i],
                adjacentPoints[(i + 1) % 8]);

    return cumulativeNormal.NormalizedSafe();
}

void Terrain::freeCpuVertexData()
{
    struct Visitor
    {
        static void run(TerrainQuadTreeNode* node)
        {
            node->destroyCpuVertexData();
            if (!node->isLeaf())
                for (unsigned short i = 0; i < 4; ++i) run(node->getChild(i));
        }
    };
    if (mQuadTree) Visitor::run(mQuadTree);
}

TerrainRayResult Terrain::rayIntersects(const Math::Ray& ray) const
{
    TerrainRayResult result;
    if (!mQuadTree) return result;

    Math::Vec3 rayOrigin = ray.origin - mPos;
    Math::Vec3 rayDirection = ray.direction;
    Math::Vec3 tmp;
    switch (mAlign)
    {
        case TerrainAlignment::AlignXY:
        {
            const float o = rayOrigin.y;
            rayOrigin.y = rayOrigin.z;
            rayOrigin.z = o;
            const float d = rayDirection.y;
            rayDirection.y = rayDirection.z;
            rayDirection.z = d;
            break;
        }
        case TerrainAlignment::AlignYZ:
            tmp.x = rayOrigin.z;
            tmp.z = rayOrigin.y;
            tmp.y = -rayOrigin.x;
            rayOrigin = tmp;
            tmp.x = rayDirection.z;
            tmp.z = rayDirection.y;
            tmp.y = -rayDirection.x;
            rayDirection = tmp;
            break;
        case TerrainAlignment::AlignXZ:
            rayOrigin.z = -rayOrigin.z;
            rayDirection.z = -rayDirection.z;
            break;
    }
    rayOrigin.x += mWorldSize / 2;
    rayOrigin.z += mWorldSize / 2;
    rayOrigin.x /= mScale;
    rayOrigin.z /= mScale;
    rayDirection.x /= mScale;
    rayDirection.z /= mScale;
    rayDirection = rayDirection.NormalizedSafe();
    const Math::Ray localRay(rayOrigin, rayDirection);

    const float maxHeight = getMaxHeight();
    const float minHeight = getMinHeight();

    const Math::Box aabb(Math::Vec3(0.0f, minHeight, 0.0f),
            Math::Vec3(static_cast<float>(mSize), maxHeight, static_cast<float>(mSize)));
    const terrain_math::Hit aabbTest = terrain_math::intersectsBox(localRay, aabb);
    if (!aabbTest.hit) return result;

    Math::Vec3 cur = localRay.PointAt(aabbTest.distance);

    int quadX = static_cast<int>(cur.x);
    quadX = quadX > 0 ? quadX : 0;
    quadX = quadX < static_cast<int>(mSize) - 2 ? quadX : static_cast<int>(mSize) - 2;
    int quadZ = static_cast<int>(cur.z);
    quadZ = quadZ > 0 ? quadZ : 0;
    quadZ = quadZ < static_cast<int>(mSize) - 2 ? quadZ : static_cast<int>(mSize) - 2;
    const int flipX = (rayDirection.x < 0 ? 0 : 1);
    const int flipZ = (rayDirection.z < 0 ? 0 : 1);
    const int xDir = (rayDirection.x < 0 ? -1 : 1);
    const int zDir = (rayDirection.z < 0 ? -1 : 1);

    QuadHit quad;
    const float dummyHighValue = static_cast<float>(mSize) * 10000.0f;

    while (cur.y >= (minHeight - 1e-3) && cur.y <= (maxHeight + 1e-3))
    {
        if (quadX < 0 || quadX >= static_cast<int>(mSize) - 1 || quadZ < 0 ||
                quadZ >= static_cast<int>(mSize) - 1)
            break;

        quad = checkQuadIntersection(quadX, quadZ, localRay);
        if (quad.hit) break;

        const float xDist = fabsf(rayDirection.x) <= FLT_EPSILON
                                    ? dummyHighValue
                                    : (quadX - cur.x + flipX) / rayDirection.x;
        const float zDist = fabsf(rayDirection.z) <= FLT_EPSILON
                                    ? dummyHighValue
                                    : (quadZ - cur.z + flipZ) / rayDirection.z;
        if (xDist < zDist)
        {
            quadX += xDir;
            cur += rayDirection * xDist;
        }
        else
        {
            quadZ += zDir;
            cur += rayDirection * zDist;
        }
    }

    if (quad.hit)
    {
        Math::Vec3 out = quad.position;
        out.x *= mScale;
        out.z *= mScale;
        out.x -= mWorldSize / 2;
        out.z -= mWorldSize / 2;
        switch (mAlign)
        {
            case TerrainAlignment::AlignXY:
            {
                const float y = out.y;
                out.y = out.z;
                out.z = y;
                break;
            }
            case TerrainAlignment::AlignYZ:
                tmp.x = -out.y;
                tmp.y = out.z;
                tmp.z = out.x;
                out = tmp;
                break;
            case TerrainAlignment::AlignXZ:
                out.z = -out.z;
                break;
        }
        result.hit = true;
        result.position = out + mPos;
    }
    return result;
}

Terrain::QuadHit Terrain::checkQuadIntersection(int x, int z, const Math::Ray& ray) const
{
    const Math::Vec3 v1(static_cast<float>(x), *getHeightData(x, z), static_cast<float>(z));
    const Math::Vec3 v2(static_cast<float>(x) + 1, *getHeightData(x + 1, z), static_cast<float>(z));
    const Math::Vec3 v3(static_cast<float>(x), *getHeightData(x, z + 1), static_cast<float>(z) + 1);
    const Math::Vec3 v4(static_cast<float>(x) + 1, *getHeightData(x + 1, z + 1),
            static_cast<float>(z) + 1);

    Math::Vec4 p1, p2;
    bool oddRow = false;
    if (z % 2)
    {
        p1 = terrain_math::faceNormalPlaneWithoutNormalize(v2, v4, v3);
        p2 = terrain_math::faceNormalPlaneWithoutNormalize(v1, v2, v3);
        oddRow = true;
    }
    else
    {
        p1 = terrain_math::faceNormalPlaneWithoutNormalize(v1, v2, v4);
        p2 = terrain_math::faceNormalPlaneWithoutNormalize(v1, v4, v3);
    }

    QuadHit hit;
    terrain_math::Hit planeInt = terrain_math::intersectsPlane(ray, p1);
    if (planeInt.hit)
    {
        const Math::Vec3 where = ray.PointAt(planeInt.distance);
        const Math::Vec3 rel = where - v1;
        if (rel.x >= -0.01 && rel.x <= 1.01 && rel.z >= -0.01 && rel.z <= 1.01 &&
                ((rel.x >= rel.z && !oddRow) || (rel.x >= (1 - rel.z) && oddRow)))
        {
            hit.hit = true;
            hit.position = where;
            return hit;
        }
    }
    planeInt = terrain_math::intersectsPlane(ray, p2);
    if (planeInt.hit)
    {
        const Math::Vec3 where = ray.PointAt(planeInt.distance);
        const Math::Vec3 rel = where - v1;
        if (rel.x >= -0.01 && rel.x <= 1.01 && rel.z >= -0.01 && rel.z <= 1.01 &&
                ((rel.x <= rel.z && !oddRow) || (rel.x <= (1 - rel.z) && oddRow)))
        {
            hit.hit = true;
            hit.position = where;
            return hit;
        }
    }
    return hit;
}

std::size_t Terrain::getNumIndexesForBatchSize(std::uint16_t batchSize)
{
    const std::size_t mainIndexesPerRow = batchSize * 2 + 1;
    const std::size_t numRows = batchSize - 1;
    const std::size_t mainIndexCount = mainIndexesPerRow * numRows;
    const std::size_t skirtIndexCount = (batchSize - 1) * 2 * 4 + 2;
    return mainIndexCount + skirtIndexCount;
}

void Terrain::populateIndexBuffer(std::uint16_t* pI, std::uint16_t batchSize,
        std::uint16_t vdatasize, std::uint16_t vertexIncrement, std::uint16_t xoffset,
        std::uint16_t yoffset, std::uint16_t numSkirtRowsCols, std::uint16_t skirtRowColSkip)
{
    const std::uint16_t rowSize = static_cast<std::uint16_t>(vdatasize * vertexIncrement);
    const std::uint16_t numRows = static_cast<std::uint16_t>(batchSize - 1);

    std::uint16_t currentVertex = static_cast<std::uint16_t>((batchSize - 1) * vertexIncrement);
    const std::uint16_t columnStart = xoffset;
    const std::uint16_t rowStart = yoffset;
    currentVertex = static_cast<std::uint16_t>(currentVertex + rowStart * vdatasize + columnStart);
    bool rightToLeft = true;
    for (std::uint16_t r = 0; r < numRows; ++r)
    {
        for (std::uint16_t c = 0; c < batchSize; ++c)
        {
            *pI++ = currentVertex;
            *pI++ = static_cast<std::uint16_t>(currentVertex + rowSize);

            if (c + 1 < batchSize)
            {
                currentVertex =
                        rightToLeft ? static_cast<std::uint16_t>(currentVertex - vertexIncrement)
                                    : static_cast<std::uint16_t>(currentVertex + vertexIncrement);
            }
        }
        rightToLeft = !rightToLeft;
        currentVertex = static_cast<std::uint16_t>(currentVertex + rowSize);
        *pI++ = currentVertex;
    }

    for (std::uint16_t s = 0; s < 4; ++s)
    {
        int edgeIncrement = 0, skirtIncrement = 0;
        switch (s)
        {
            case 0:
                edgeIncrement = -static_cast<int>(vertexIncrement);
                skirtIncrement = -static_cast<int>(vertexIncrement);
                break;
            case 1:
                edgeIncrement = -static_cast<int>(rowSize);
                skirtIncrement = -static_cast<int>(vertexIncrement);
                break;
            case 2:
                edgeIncrement = static_cast<int>(vertexIncrement);
                skirtIncrement = static_cast<int>(vertexIncrement);
                break;
            case 3:
                edgeIncrement = static_cast<int>(rowSize);
                skirtIncrement = static_cast<int>(vertexIncrement);
                break;
        }
        std::uint16_t skirtIndex = calcSkirtVertexIndex(currentVertex, vdatasize, (s % 2) != 0,
                numSkirtRowsCols, skirtRowColSkip);
        for (std::uint16_t c = 0; c < batchSize - 1; ++c)
        {
            *pI++ = currentVertex;
            *pI++ = skirtIndex;
            currentVertex = static_cast<std::uint16_t>(currentVertex + edgeIncrement);
            skirtIndex = static_cast<std::uint16_t>(skirtIndex + skirtIncrement);
        }
        if (s == 3)
        {
            *pI++ = currentVertex;
            *pI++ = skirtIndex;
            currentVertex = static_cast<std::uint16_t>(currentVertex + edgeIncrement);
        }
    }
}

std::uint16_t Terrain::calcSkirtVertexIndex(std::uint16_t mainIndex, std::uint16_t vdatasize,
        bool isCol, std::uint16_t numSkirtRowsCols, std::uint16_t skirtRowColSkip)
{
    const std::uint16_t row = static_cast<std::uint16_t>(mainIndex / vdatasize);
    const std::uint16_t col = static_cast<std::uint16_t>(mainIndex % vdatasize);

    const std::uint16_t base = static_cast<std::uint16_t>(vdatasize * vdatasize);

    if (isCol)
    {
        const std::uint16_t skirtNum = static_cast<std::uint16_t>(col / skirtRowColSkip);
        const std::uint16_t colbase = static_cast<std::uint16_t>(numSkirtRowsCols * vdatasize);
        return static_cast<std::uint16_t>(base + colbase + vdatasize * skirtNum + row);
    }
    const std::uint16_t skirtNum = static_cast<std::uint16_t>(row / skirtRowColSkip);
    return static_cast<std::uint16_t>(base + vdatasize * skirtNum + col);
}

} // namespace prisma3d
