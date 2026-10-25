#pragma once

#include "mathc.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Pixmap;
class TerrainQuadTreeNode;
class TerrainLayerBlendMap;
class TerrainMapTarget;
class TerrainGeometryTarget;
struct TerrainVertexData;
struct TerrainPixelBox;

enum class TerrainAlignment : std::uint8_t
{
    AlignXZ = 0,
    AlignXY = 1,
    AlignYZ = 2
};

enum class TerrainSpace : std::uint8_t
{
    World,
    Local,
    Terrain,
    Point
};

struct TerrainRect
{
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;

    TerrainRect() = default;
    TerrainRect(int l, int t, int r, int b)
            : left(l),
              top(t),
              right(r),
              bottom(b)
    {
    }

    int width() const
    {
        return right - left;
    }
    int height() const
    {
        return bottom - top;
    }
    bool isNull() const
    {
        return width() == 0 || height() == 0;
    }
    void merge(const TerrainRect& other);
    TerrainRect intersect(const TerrainRect& other) const;
};

struct TerrainLayerInstance
{
    float worldSize = 100.0f;
    ct::Vector<ct::String> textureNames;
};

struct TerrainBlendChannel
{
    std::uint8_t texture = 0;
    std::uint8_t channel = 0;
};

struct TerrainImportData
{
    TerrainAlignment terrainAlign = TerrainAlignment::AlignXZ;
    std::uint16_t terrainSize = 1025;
    std::uint16_t maxBatchSize = 65;
    std::uint16_t minBatchSize = 17;
    Math::Vec3 pos = Math::Vec3(0.0f, 0.0f, 0.0f);
    float worldSize = 1000.0f;
    const Pixmap* inputImage = nullptr;
    const std::uint16_t* inputImage16 = nullptr;
    int inputImage16Width = 0;
    int inputImage16Height = 0;
    const float* inputFloat = nullptr;
    float constantHeight = 0.0f;
    float inputScale = 1.0f;
    float inputBias = 0.0f;
    float skirtSize = 30.0f;
    float maxPixelError = 3.0f;
    bool useRayBoxDistanceCalculation = false;
    ct::Vector<TerrainLayerInstance> layerList;
    std::uint16_t layerBlendMapSize = 1024;
    std::uint16_t lightMapSize = 1024;
    std::uint16_t compositeMapSize = 1024;
    std::uint16_t defaultGlobalColourMapSize = 1024;
    float defaultLayerTextureWorldSize = 10.0f;
    Math::Vec3 lightMapDirection = Math::Vec3(0.70710678f, -0.70710678f, 0.0f);
    Math::Vec3 compositeMapAmbient = Math::Vec3(1.0f, 1.0f, 1.0f);
    Math::Vec3 compositeMapDiffuse = Math::Vec3(1.0f, 1.0f, 1.0f);
    float compositeMapDistance = 4000.0f;
};

struct TerrainLodView
{
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    const Math::Frustum* frustum = nullptr;
    float fovY = 1.04719755f;
    float viewportHeight = 720.0f;
    float lodBiasInverse = 1.0f;
};

struct TerrainRayResult
{
    bool hit = false;
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
};

struct TerrainIndexKey
{
    std::uint16_t batchSize = 0;
    std::uint16_t vdatasize = 0;
    std::uint16_t vertexIncrement = 0;
    std::uint16_t xoffset = 0;
    std::uint16_t yoffset = 0;
    std::uint16_t numSkirtRowsCols = 0;
    std::uint16_t skirtRowColSkip = 0;

    std::uint64_t pack() const;
};

class Terrain
{
public:
    static constexpr std::uint16_t kMaxBatchSize = 129;
    static constexpr std::uint8_t kLayerSamplerCount = 2;
    static constexpr std::uint8_t kMaxLayerSlots = 5;
    static constexpr std::uint8_t kTextureBudget = 16;
    static constexpr std::uint8_t DERIVED_DATA_DELTAS = 1;
    static constexpr std::uint8_t DERIVED_DATA_NORMALS = 2;
    static constexpr std::uint8_t DERIVED_DATA_LIGHTMAP = 4;
    static constexpr std::uint8_t DERIVED_DATA_ALL = 7;

    Terrain();
    ~Terrain();
    Terrain(const Terrain&) = delete;
    Terrain& operator=(const Terrain&) = delete;

    bool prepare(const TerrainImportData& importData);
    void unprepare();
    bool isPrepared() const;
    void load();
    void unload();
    bool isLoaded() const;

    const Math::Box& getAABB() const;
    Math::Box getWorldAABB() const;
    float getMinHeight() const;
    float getMaxHeight() const;
    float getBoundingRadius() const;

    float* getHeightData();
    const float* getHeightData() const;
    float* getHeightData(std::uint32_t x, std::uint32_t y);
    const float* getHeightData(std::uint32_t x, std::uint32_t y) const;
    const float* getDeltaData() const;
    const float* getDeltaData(std::uint32_t x, std::uint32_t y) const;
    float getHeightAtPoint(std::uint32_t x, std::uint32_t y) const;
    float getHeightAtTerrainPosition(float x, float y) const;
    float getSlopeAtTerrainPosition(float x, float y) const;
    float getSlopeAtWorldPosition(const Math::Vec3& pos) const;
    float getHeightAtWorldPosition(float x, float y, float z) const;
    float getHeightAtWorldPosition(const Math::Vec3& pos) const;

    Math::Vec3 convertPosition(TerrainSpace inSpace, const Math::Vec3& inPos,
            TerrainSpace outSpace) const;
    Math::Vec3 convertDirection(TerrainSpace inSpace, const Math::Vec3& inDir,
            TerrainSpace outSpace) const;
    void convertPosition(TerrainSpace inSpace, const Math::Vec3& inPos, TerrainSpace outSpace,
            Math::Vec3& outPos) const;
    void convertDirection(TerrainSpace inSpace, const Math::Vec3& inDir, TerrainSpace outSpace,
            Math::Vec3& outDir) const;
    static void convertWorldToTerrainAxes(TerrainAlignment align, const Math::Vec3& worldVec,
            Math::Vec3& terrainVec);
    static void convertTerrainToWorldAxes(TerrainAlignment align, const Math::Vec3& terrainVec,
            Math::Vec3& worldVec);
    Math::Vec3 convertWorldToTerrainAxes(const Math::Vec3& inVec) const;
    Math::Vec3 convertTerrainToWorldAxes(const Math::Vec3& inVec) const;

    void getPoint(std::uint32_t x, std::uint32_t y, Math::Vec3& outpos) const;
    void getPoint(std::uint32_t x, std::uint32_t y, float height, Math::Vec3& outpos) const;
    void getPointAlign(std::uint32_t x, std::uint32_t y, TerrainAlignment align,
            Math::Vec3& outpos) const;
    void getPointAlign(std::uint32_t x, std::uint32_t y, float height, TerrainAlignment align,
            Math::Vec3& outpos) const;
    void getVector(const Math::Vec3& inVec, Math::Vec3& outVec) const;
    void getVector(float x, float y, float z, Math::Vec3& outVec) const;
    void getVectorAlign(const Math::Vec3& inVec, TerrainAlignment align, Math::Vec3& outVec) const;
    void getVectorAlign(float x, float y, float z, TerrainAlignment align,
            Math::Vec3& outVec) const;
    void getPosition(const Math::Vec3& tsPos, Math::Vec3& outWsPos) const;
    void getPosition(float x, float y, float z, Math::Vec3& outWsPos) const;
    void getTerrainPosition(const Math::Vec3& wsPos, Math::Vec3& outTsPos) const;
    void getTerrainPosition(float x, float y, float z, Math::Vec3& outTsPos) const;
    void getPositionAlign(const Math::Vec3& tsPos, TerrainAlignment align,
            Math::Vec3& outWsPos) const;
    void getPositionAlign(float x, float y, float z, TerrainAlignment align,
            Math::Vec3& outWsPos) const;
    void getTerrainPositionAlign(const Math::Vec3& wsPos, TerrainAlignment align,
            Math::Vec3& outTsPos) const;
    void getTerrainPositionAlign(float x, float y, float z, TerrainAlignment align,
            Math::Vec3& outTsPos) const;
    void getTerrainVector(const Math::Vec3& inVec, Math::Vec3& outVec) const;
    void getTerrainVector(float x, float y, float z, Math::Vec3& outVec) const;
    void getTerrainVectorAlign(const Math::Vec3& inVec, TerrainAlignment align,
            Math::Vec3& outVec) const;
    void getTerrainVectorAlign(float x, float y, float z, TerrainAlignment align,
            Math::Vec3& outVec) const;

    TerrainAlignment getAlignment() const;
    std::uint16_t getSize() const;
    std::uint16_t getMaxBatchSize() const;
    std::uint16_t getMinBatchSize() const;
    float getWorldSize() const;
    const Math::Vec3& getPosition() const;
    void setPosition(const Math::Vec3& pos);
    float getBase() const;
    float getScale() const;
    float getSkirtSize() const;
    void setSkirtSize(float skirtSize);
    float getMaxPixelError() const;
    void setMaxPixelError(float maxPixelError);
    bool getUseRayBoxDistanceCalculation() const;
    void setUseRayBoxDistanceCalculation(bool rayBox);
    bool _getMorphRequired() const;
    void _setMorphRequired(bool morph);
    bool _getUseVertexCompression() const;

    std::uint16_t getNumLodLevels() const;
    std::uint16_t getNumLodLevelsPerLeaf() const;
    std::uint16_t getTreeDepth() const;
    std::uint16_t getResolutionAtLod(std::uint16_t lodLevel) const;
    std::uint32_t getGeoDataSizeAtLod(std::uint16_t lodLevel) const;
    std::uint16_t getLODLevelWhenVertexEliminated(long x, long y) const;
    std::uint16_t getLODLevelWhenVertexEliminated(long rowOrColumn) const;

    TerrainQuadTreeNode* getQuadTree() const;
    TerrainRect calculateHeightDeltas(const TerrainRect& rect);
    void finaliseHeightDeltas(const TerrainRect& rect, bool cpuData = true);
    float getCFactor(const TerrainLodView& view) const;
    void calculateCurrentLod(const TerrainLodView& view);
    Math::Vec3 calculateNormal(int x, int y) const;
    void freeCpuVertexData();

    void setGeometryTarget(TerrainGeometryTarget* target);
    TerrainGeometryTarget* getGeometryTarget() const;
    TerrainVertexData& getVertexScratch(std::uint32_t vertexCount);
    void setHeightAtPoint(std::uint32_t x, std::uint32_t y, float h);
    void update(bool synchronous = true);
    void updateGeometry();
    const TerrainRect& getDirtyGeometryRect() const;

    void setMapTarget(TerrainMapTarget* target);
    TerrainMapTarget* getMapTarget() const;
    void loadMaps(bool normalMap, bool lightmap);

    std::uint8_t getLayerCount() const;
    void addLayer(float worldSize = 0.0f, const ct::Vector<ct::String>* textureNames = nullptr);
    void addLayer(std::uint8_t index, float worldSize,
            const ct::Vector<ct::String>* textureNames = nullptr);
    void replaceLayer(std::uint8_t index, bool keepBlends, float worldSize,
            const ct::Vector<ct::String>* textureNames);
    void removeLayer(std::uint8_t index);
    std::uint8_t getMaxLayers() const;
    float getLayerWorldSize(std::uint8_t index) const;
    void setLayerWorldSize(std::uint8_t index, float size);
    float getLayerUVMultiplier(std::uint8_t index) const;
    const ct::String& getLayerTextureName(std::uint8_t layerIndex, std::uint8_t samplerIndex) const;
    void setLayerTextureName(std::uint8_t layerIndex, std::uint8_t samplerIndex,
            const ct::String& textureName);
    TerrainLayerBlendMap* getLayerBlendMap(std::uint8_t layerIndex);
    std::uint8_t getBlendTextureCount() const;
    static std::uint8_t getBlendTextureCount(std::uint8_t numLayers);
    TerrainBlendChannel getLayerBlendTextureIndex(std::uint8_t layerIndex) const;
    std::uint8_t getBlendTextureIndex(std::uint8_t layerIndex) const;
    std::uint16_t getLayerBlendMapSize() const;
    ct::Vector<std::uint8_t>& getBlendTextureData(std::uint8_t index);
    const ct::Vector<std::uint8_t>& getBlendTextureData(std::uint8_t index) const;
    void uploadBlendTextureRect(std::uint8_t index, const TerrainRect& box);
    bool isMaterialDirty() const;
    bool isMaterialParamsDirty() const;
    void clearMaterialDirty();

    void dirty();
    void dirtyRect(const TerrainRect& rect);
    void _dirtyCompositeMapRect(const TerrainRect& rect);
    void dirtyLightmapRect(const TerrainRect& rect);
    void dirtyLightmap();
    void updateDerivedData(bool synchronous = true, std::uint8_t typeMask = DERIVED_DATA_ALL);
    void _setNormalMapRequired(bool normalMap);
    void _setLightMapRequired(bool lightMap, bool shadowsOnly = true);
    void _setCompositeMapRequired(bool compositeMap);
    bool _getNormalMapRequired() const;
    bool _getLightMapRequired() const;
    bool _getCompositeMapRequired() const;
    void calculateNormals(const TerrainRect& rect, TerrainRect& finalRect, TerrainPixelBox& out);
    void finaliseNormals(const TerrainRect& rect, const TerrainPixelBox& normalsBox);
    void calculateLightmap(const TerrainRect& rect, const TerrainRect& extraTargetRect,
            TerrainRect& outFinalRect, TerrainPixelBox& out);
    void finaliseLightmap(const TerrainRect& rect, const TerrainPixelBox& lightmapBox);
    void widenRectByVector(const Math::Vec3& vec, const TerrainRect& inRect,
            TerrainRect& outRect) const;
    void widenRectByVector(const Math::Vec3& vec, const TerrainRect& inRect, float minHeight,
            float maxHeight, TerrainRect& outRect) const;
    void updateCompositeMap();
    void updateCompositeMapWithDelay(float delay = 2.0f);
    void advanceDeferredUpdates(unsigned long elapsedMillis);
    TerrainRect compositeImageRect(const TerrainRect& rect) const;
    std::uint16_t getMaterialLodIndex(float value) const;

    void setGlobalColourMapEnabled(bool enabled, std::uint16_t sz = 0);
    bool getGlobalColourMapEnabled() const;
    std::uint16_t getGlobalColourMapSize() const;
    bool loadGlobalColourMap(const Pixmap& image);
    const ct::Vector<std::uint8_t>& getGlobalColourMapData() const;
    std::uint8_t* globalColourMapData();
    void dirtyGlobalColourMapRect(const TerrainRect& rect);
    std::uint16_t getLightmapSize() const;
    std::uint16_t getCompositeMapSize() const;
    const Math::Vec3& getLightMapDirection() const;
    void setLightMapDirection(const Math::Vec3& direction);
    bool getLightMapPadFollowsRange() const;
    void setLightMapPadFollowsRange(bool follows);
    const Math::Vec3& getCompositeMapAmbient() const;
    void setCompositeMapAmbient(const Math::Vec3& ambient);
    const Math::Vec3& getCompositeMapDiffuse() const;
    void setCompositeMapDiffuse(const Math::Vec3& diffuse);
    float getCompositeMapDistance() const;
    void setCompositeMapDistance(float distance);
    float getDefaultLayerTextureWorldSize() const;

    TerrainRayResult rayIntersects(const Math::Ray& ray) const;

    static std::size_t getNumIndexesForBatchSize(std::uint16_t batchSize);
    static void populateIndexBuffer(std::uint16_t* pI, std::uint16_t batchSize,
            std::uint16_t vdatasize, std::uint16_t vertexIncrement, std::uint16_t xoffset,
            std::uint16_t yoffset, std::uint16_t numSkirtRowsCols, std::uint16_t skirtRowColSkip);
    static std::uint16_t calcSkirtVertexIndex(std::uint16_t mainIndex, std::uint16_t vdatasize,
            bool isCol, std::uint16_t numSkirtRowsCols, std::uint16_t skirtRowColSkip);

private:
    struct QuadHit
    {
        bool hit = false;
        Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    };

    void determineLodLevels();
    Math::Vec4 heightPlaneAt(float x, float y) const;
    void distributeVertexData();
    void calculateNodeDeltas(TerrainQuadTreeNode* node, int sourceLevel);
    void updateBaseScale();
    void convertSpace(TerrainSpace inSpace, const Math::Vec3& inVec, TerrainSpace outSpace,
            Math::Vec3& outVec, bool translation) const;
    QuadHit checkQuadIntersection(int x, int z, const Math::Ray& ray) const;
    void freeAll();
    void freeMaps();
    void checkLayers(bool includeGPUResources);
    void deriveUVMultipliers();
    void createGPUBlendTextures();
    void createLayerBlendMaps();
    void deleteBlendMaps(std::uint8_t lowIndex);
    void shiftUpGPUBlendChannels(std::uint8_t index);
    void shiftDownGPUBlendChannels(std::uint8_t index);
    void copyBlendTextureChannel(std::uint8_t srcIndex, std::uint8_t srcChannel,
            std::uint8_t destIndex, std::uint8_t destChannel);
    void clearGPUBlendChannel(std::uint8_t index, unsigned channel);
    void createOrDestroyGPUNormalMap();
    void createOrDestroyGPULightmap();
    float lightmapHeightPad() const;
    void createOrDestroyGPUColourMap();
    void createOrDestroyGPUCompositeMap();
    void updateDerivedDataImpl(const TerrainRect& rect, const TerrainRect& lightmapExtraRect,
            std::uint8_t typeMask);

    bool mPrepared;
    bool mLoaded;
    TerrainAlignment mAlign;
    float mWorldSize;
    std::uint16_t mSize;
    std::uint16_t mMaxBatchSize;
    std::uint16_t mMinBatchSize;
    Math::Vec3 mPos;
    TerrainQuadTreeNode* mQuadTree;
    std::uint16_t mNumLodLevels;
    std::uint16_t mNumLodLevelsPerLeafNode;
    std::uint16_t mTreeDepth;
    float mBase;
    float mScale;
    float mSkirtSize;
    float mMaxPixelError;
    bool mUseRayBoxDistanceCalculation;
    bool mLodMorphRequired;
    ct::Vector<float> mHeightData;
    ct::Vector<float> mDeltaData;

    TerrainMapTarget* mTarget;
    TerrainGeometryTarget* mGeometryTarget;
    TerrainVertexData* mVertexScratch;
    TerrainRect mDirtyGeometryRect;
    ct::Vector<TerrainLayerInstance> mLayers;
    ct::Vector<float> mLayerUVMultiplier;
    ct::Vector<TerrainLayerBlendMap*> mLayerBlendMapList;
    ct::Vector<ct::Vector<std::uint8_t>> mBlendTextureData;
    bool mMaterialDirty;
    bool mMaterialParamsDirty;
    std::uint16_t mLayerBlendMapSize;
    std::uint16_t mLightmapSize;
    std::uint16_t mCompositeMapSize;
    std::uint16_t mGlobalColourMapSize;
    std::uint16_t mDefaultGlobalColourMapSize;
    bool mGlobalColourMapEnabled;
    float mDefaultLayerTextureWorldSize;
    Math::Vec3 mLightMapDir;
    Math::Vec3 mCompositeMapAmbient;
    Math::Vec3 mCompositeMapDiffuse;
    float mCompositeMapDistance;
    bool mNormalMapRequired;
    bool mLightMapRequired;
    bool mLightMapShadowsOnly;
    bool mCompositeMapRequired;
    bool mNormalMapReady;
    bool mLightmapReady;
    bool mLightmapPadValid;
    bool mLightmapPadFollowsRange;
    float mLightmapPad;
    bool mColourMapReady;
    ct::Vector<std::uint8_t> mColourMapData;
    bool mCompositeMapReady;
    TerrainRect mDirtyDerivedDataRect;
    TerrainRect mDirtyLightmapFromNeighboursRect;
    TerrainRect mCompositeMapDirtyRect;
    bool mCompositeMapDirtyRectLightmapUpdate;
    long mCompositeMapUpdateCountdown;
};

} // namespace prisma3d
