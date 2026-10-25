#pragma once

#include "prisma3d/terrain/Terrain.h"

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct TerrainVertexData
{
    std::uint16_t resolution = 0;
    std::uint16_t size = 0;
    std::uint16_t treeLevels = 0;
    std::uint16_t numSkirtRowsCols = 0;
    std::uint16_t skirtRowColSkip = 0;
    std::uint32_t vertexCount = 0;
    bool cpuDataPresent = false;
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec2> uvs;
    ct::Vector<Math::Vec2> deltas;
};

struct TerrainVertexRange
{
    std::uint32_t first = 0;
    std::uint32_t count = 0;
};

class TerrainQuadTreeNode
{
public:
    struct LodLevel
    {
        std::uint16_t batchSize = 0;
        float maxHeightDelta = 0.0f;
        float calcMaxHeightDelta = 0.0f;
        float directHeightDelta = 0.0f;
        float lastTransitionDist = 0.0f;
        float lastCFactor = 0.0f;
    };

    TerrainQuadTreeNode(Terrain* terrain, TerrainQuadTreeNode* parent, std::uint16_t xoff,
            std::uint16_t yoff, std::uint16_t size, std::uint16_t lod, std::uint16_t depth);
    ~TerrainQuadTreeNode();
    TerrainQuadTreeNode(const TerrainQuadTreeNode&) = delete;
    TerrainQuadTreeNode& operator=(const TerrainQuadTreeNode&) = delete;

    std::uint16_t getXOffset() const;
    std::uint16_t getYOffset() const;
    std::uint16_t getSize() const;
    std::uint16_t getDepth() const;
    bool isLeaf() const;
    std::uint16_t getBaseLod() const;
    std::uint16_t getLodCount() const;
    TerrainQuadTreeNode* getChild(unsigned short child) const;
    TerrainQuadTreeNode* getParent() const;
    Terrain* getTerrain() const;

    void load();
    void load(std::uint16_t depthStart, std::uint16_t depthEnd);
    void loadSelf();
    void unload();
    bool isAttached() const;

    const LodLevel* getLodLevel(std::uint16_t lod) const;

    void resetDirectDeltas();
    void noteDirectDelta(std::uint16_t lod, float delta);
    void postDeltaCalculation();
    void finaliseDeltaValues();

    void assignVertexData(std::uint16_t treeDepthStart, std::uint16_t treeDepthEnd,
            std::uint16_t resolution, std::uint32_t sz);
    void useAncestorVertexData(TerrainQuadTreeNode* owner, std::uint16_t treeDepthEnd,
            std::uint16_t resolution);
    void updateVertexData(bool positions, bool deltas, const TerrainRect& rect,
            bool cpuData = true);
    void destroyCpuVertexData();

    TerrainQuadTreeNode* getNodeWithVertexData() const;
    bool ownsVertexData() const;
    const TerrainVertexData* getVertexDataRecord() const;
    bool getIndexKey(std::uint16_t lod, TerrainIndexKey& key) const;

    void refreshBounds(const TerrainRect& rect);
    bool rectIntersectsNode(const TerrainRect& rect) const;

    const Math::Box& getBoundingBox() const;
    float getBoundingRadius() const;
    const Math::Vec3& getLocalCentre() const;
    float getMinHeight() const;
    float getMaxHeight() const;
    Math::Box getWorldBoundingBox() const;

    bool calculateCurrentLod(const TerrainLodView& view, float cFactor);
    int getCurrentLod() const;
    std::uint16_t getMaterialLodIndex() const;
    bool isRenderedAtCurrentLod() const;
    bool isSelfOrChildRenderedAtCurrentLod() const;
    void setCurrentLod(int lod);
    float getLodTransition() const;
    void setLodTransition(float t);
    Math::Vec2 getLodMorph() const;

private:
    void createCpuVertexData();
    void updateVertexBuffer(const TerrainVertexData& layout, TerrainVertexData& data,
            bool positions, bool deltas, const TerrainRect& rect);
    void writePosVertex(TerrainVertexData& data, std::size_t index, std::uint16_t x,
            std::uint16_t y, float height, const Math::Vec3& pos, float uvScale);
    void writeDeltaVertex(TerrainVertexData& data, std::size_t index, float delta,
            float deltaThresh);
    void noteVertex(std::size_t index);
    std::uint16_t calcSkirtVertexIndex(std::uint16_t mainIndex, bool isCol) const;
    void refreshLodMorph();
    void clearCurrentLod();

    Terrain* mTerrain;
    TerrainQuadTreeNode* mParent;
    TerrainQuadTreeNode* mChildren[4];
    ct::Vector<LodLevel> mLodLevels;

    std::uint16_t mOffsetX;
    std::uint16_t mOffsetY;
    std::uint16_t mBoundaryX;
    std::uint16_t mBoundaryY;
    std::uint16_t mSize;
    std::uint16_t mBaseLod;
    std::uint16_t mDepth;
    Math::Vec3 mLocalCentre;
    Math::Box mAABB;
    float mBoundingRadius;
    int mCurrentLod;
    std::uint16_t mMaterialLodIndex;
    float mLodTransition;
    Math::Vec2 mLodMorph;
    TerrainQuadTreeNode* mChildWithMaxHeightDelta;
    bool mSelfOrChildRendered;
    bool mAttached;
    TerrainQuadTreeNode* mNodeWithVertexData;
    TerrainVertexData* mVertexDataRecord;
    ct::Vector<TerrainVertexRange>* mTouched;
};

} // namespace prisma3d
