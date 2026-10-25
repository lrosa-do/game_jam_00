#include "prisma3d/terrain/TerrainQuadTreeNode.h"

#include "TerrainMath.h"
#include "prisma3d/terrain/TerrainGeometryTarget.h"

#include <float.h>
#include <math.h>

namespace prisma3d
{

TerrainQuadTreeNode::TerrainQuadTreeNode(Terrain* terrain, TerrainQuadTreeNode* parent,
        std::uint16_t xoff, std::uint16_t yoff, std::uint16_t size, std::uint16_t lod,
        std::uint16_t depth)
        : mTerrain(terrain),
          mParent(parent),
          mChildren{ nullptr, nullptr, nullptr, nullptr },
          mLodLevels(),
          mOffsetX(xoff),
          mOffsetY(yoff),
          mBoundaryX(static_cast<std::uint16_t>(xoff + size)),
          mBoundaryY(static_cast<std::uint16_t>(yoff + size)),
          mSize(size),
          mBaseLod(lod),
          mDepth(depth),
          mLocalCentre(0.0f, 0.0f, 0.0f),
          mAABB(),
          mBoundingRadius(0.0f),
          mCurrentLod(-1),
          mMaterialLodIndex(0),
          mLodTransition(0.0f),
          mLodMorph(0.0f, 0.0f),
          mChildWithMaxHeightDelta(nullptr),
          mSelfOrChildRendered(false),
          mAttached(false),
          mNodeWithVertexData(nullptr),
          mVertexDataRecord(nullptr),
          mTouched(nullptr)
{
    if (terrain->getMaxBatchSize() < size)
    {
        const std::uint16_t childSize = static_cast<std::uint16_t>(((size - 1) * 0.5f) + 1);
        const std::uint16_t childOff = static_cast<std::uint16_t>(childSize - 1);
        const std::uint16_t childLod = static_cast<std::uint16_t>(lod - 1);
        const std::uint16_t childDepth = static_cast<std::uint16_t>(depth + 1);
        mChildren[0] =
                new TerrainQuadTreeNode(terrain, this, xoff, yoff, childSize, childLod, childDepth);
        mChildren[1] = new TerrainQuadTreeNode(terrain, this,
                static_cast<std::uint16_t>(xoff + childOff), yoff, childSize, childLod, childDepth);
        mChildren[2] = new TerrainQuadTreeNode(terrain, this, xoff,
                static_cast<std::uint16_t>(yoff + childOff), childSize, childLod, childDepth);
        mChildren[3] = new TerrainQuadTreeNode(terrain, this,
                static_cast<std::uint16_t>(xoff + childOff),
                static_cast<std::uint16_t>(yoff + childOff), childSize, childLod, childDepth);

        LodLevel level;
        level.batchSize = terrain->getMinBatchSize();
        level.maxHeightDelta = 0.0f;
        level.calcMaxHeightDelta = 0.0f;
        mLodLevels.push_back(level);
    }
    else
    {
        std::uint16_t ownLod = terrain->getNumLodLevelsPerLeaf();
        mBaseLod = 0;
        std::uint16_t sz = terrain->getMaxBatchSize();

        while (ownLod--)
        {
            LodLevel level;
            level.batchSize = sz;
            level.maxHeightDelta = 0.0f;
            level.calcMaxHeightDelta = 0.0f;
            mLodLevels.push_back(level);
            if (ownLod) sz = static_cast<std::uint16_t>(((sz - 1) * 0.5) + 1);
        }
    }

    const std::uint16_t midoffset = static_cast<std::uint16_t>((size - 1) / 2);
    const std::uint16_t midpointx = static_cast<std::uint16_t>(mOffsetX + midoffset);
    const std::uint16_t midpointy = static_cast<std::uint16_t>(mOffsetY + midoffset);
    mTerrain->getPoint(midpointx, midpointy, 0.0f, mLocalCentre);
}

TerrainQuadTreeNode::~TerrainQuadTreeNode()
{
    for (int i = 0; i < 4; ++i) delete mChildren[i];
    delete mVertexDataRecord;
}

std::uint16_t TerrainQuadTreeNode::getXOffset() const
{
    return mOffsetX;
}

std::uint16_t TerrainQuadTreeNode::getYOffset() const
{
    return mOffsetY;
}

std::uint16_t TerrainQuadTreeNode::getSize() const
{
    return mSize;
}

std::uint16_t TerrainQuadTreeNode::getDepth() const
{
    return mDepth;
}

bool TerrainQuadTreeNode::isLeaf() const
{
    return mChildren[0] == nullptr;
}

std::uint16_t TerrainQuadTreeNode::getBaseLod() const
{
    return mBaseLod;
}

std::uint16_t TerrainQuadTreeNode::getLodCount() const
{
    return static_cast<std::uint16_t>(mLodLevels.size());
}

TerrainQuadTreeNode* TerrainQuadTreeNode::getChild(unsigned short child) const
{
    if (isLeaf() || child >= 4) return nullptr;
    return mChildren[child];
}

TerrainQuadTreeNode* TerrainQuadTreeNode::getParent() const
{
    return mParent;
}

Terrain* TerrainQuadTreeNode::getTerrain() const
{
    return mTerrain;
}

void TerrainQuadTreeNode::load()
{
    loadSelf();
    if (!isLeaf())
        for (int i = 0; i < 4; ++i) mChildren[i]->load();
}

void TerrainQuadTreeNode::load(std::uint16_t depthStart, std::uint16_t depthEnd)
{
    if (mDepth >= depthEnd) return;

    if (mDepth >= depthStart && mDepth < depthEnd && mNodeWithVertexData) loadSelf();

    if (!isLeaf())
        for (int i = 0; i < 4; ++i) mChildren[i]->load(depthStart, depthEnd);
}

void TerrainQuadTreeNode::loadSelf()
{
    mAttached = true;
}

void TerrainQuadTreeNode::unload()
{
    if (!isLeaf())
        for (int i = 0; i < 4; ++i) mChildren[i]->unload();
    mAttached = false;
    mCurrentLod = -1;
}

bool TerrainQuadTreeNode::isAttached() const
{
    return mAttached;
}

const TerrainQuadTreeNode::LodLevel* TerrainQuadTreeNode::getLodLevel(std::uint16_t lod) const
{
    if (lod >= mLodLevels.size()) return nullptr;
    return &mLodLevels[lod];
}

void TerrainQuadTreeNode::resetDirectDeltas()
{
    for (std::size_t i = 0; i < mLodLevels.size(); ++i) mLodLevels[i].directHeightDelta = 0.0f;
}

void TerrainQuadTreeNode::noteDirectDelta(std::uint16_t lod, float delta)
{
    if (lod >= mBaseLod && lod < mBaseLod + mLodLevels.size())
    {
        LodLevel& level = mLodLevels[static_cast<std::size_t>(lod - mBaseLod)];
        level.directHeightDelta = level.directHeightDelta > delta ? level.directHeightDelta : delta;
    }
}

void TerrainQuadTreeNode::postDeltaCalculation()
{
    if (!isLeaf())
    {
        float maxChildDelta = -1.0f;
        TerrainQuadTreeNode* childWithMaxHeightDelta = nullptr;
        for (int i = 0; i < 4; ++i)
        {
            TerrainQuadTreeNode* child = mChildren[i];

            child->postDeltaCalculation();

            const float childDelta =
                    child->mLodLevels[child->mLodLevels.size() - 1].calcMaxHeightDelta;
            if (childDelta > maxChildDelta)
            {
                childWithMaxHeightDelta = child;
                maxChildDelta = childDelta;
            }
        }

        const float scaled = maxChildDelta * 1.05f;
        mLodLevels[0].calcMaxHeightDelta =
                mLodLevels[0].directHeightDelta > scaled ? mLodLevels[0].directHeightDelta : scaled;
        mChildWithMaxHeightDelta = childWithMaxHeightDelta;
    }
    else
    {
        mLodLevels[0].calcMaxHeightDelta = mLodLevels[0].directHeightDelta;
        for (std::size_t i = 0; i + 1 < mLodLevels.size(); ++i)
        {
            const float scaled = mLodLevels[i].calcMaxHeightDelta * 1.05f;
            mLodLevels[i + 1].calcMaxHeightDelta = mLodLevels[i + 1].directHeightDelta > scaled
                                                           ? mLodLevels[i + 1].directHeightDelta
                                                           : scaled;
        }
    }
}

void TerrainQuadTreeNode::finaliseDeltaValues()
{
    if (!isLeaf())
        for (int i = 0; i < 4; ++i) mChildren[i]->finaliseDeltaValues();

    for (std::size_t i = 0; i < mLodLevels.size(); ++i)
    {
        mLodLevels[i].maxHeightDelta = mLodLevels[i].calcMaxHeightDelta;
        mLodLevels[i].lastCFactor = 0.0f;
    }
}

void TerrainQuadTreeNode::assignVertexData(std::uint16_t treeDepthStart, std::uint16_t treeDepthEnd,
        std::uint16_t resolution, std::uint32_t sz)
{
    if (mDepth == treeDepthStart)
    {
        mNodeWithVertexData = this;
        if (!mVertexDataRecord)
        {
            mVertexDataRecord = new TerrainVertexData();
            mVertexDataRecord->resolution = resolution;
            mVertexDataRecord->size = static_cast<std::uint16_t>(sz);
            mVertexDataRecord->treeLevels =
                    static_cast<std::uint16_t>(treeDepthEnd - treeDepthStart);
        }

        createCpuVertexData();

        if (!isLeaf() && treeDepthEnd > (mDepth + 1))
            for (int i = 0; i < 4; ++i)
                mChildren[i]->useAncestorVertexData(this, treeDepthEnd, resolution);
    }
    else
    {
        if (isLeaf()) return;
        for (int i = 0; i < 4; ++i)
            mChildren[i]->assignVertexData(treeDepthStart, treeDepthEnd, resolution, sz);
    }
}

void TerrainQuadTreeNode::useAncestorVertexData(TerrainQuadTreeNode* owner,
        std::uint16_t treeDepthEnd, std::uint16_t resolution)
{
    mNodeWithVertexData = owner;
    delete mVertexDataRecord;
    mVertexDataRecord = nullptr;

    if (!isLeaf() && treeDepthEnd > (mDepth + 1))
        for (int i = 0; i < 4; ++i)
            mChildren[i]->useAncestorVertexData(owner, treeDepthEnd, resolution);
}

void TerrainQuadTreeNode::updateVertexData(bool positions, bool deltas, const TerrainRect& rect,
        bool cpuData)
{
    const TerrainRect updateRect =
            rect.intersect(TerrainRect(mOffsetX, mOffsetY, mBoundaryX, mBoundaryY));
    if (updateRect.isNull()) return;

    if (mVertexDataRecord)
    {
        if (cpuData)
        {
            if (mVertexDataRecord->cpuDataPresent)
                updateVertexBuffer(*mVertexDataRecord, *mVertexDataRecord, positions, deltas,
                        updateRect);
        }
        else
        {
            TerrainVertexData& scratch = mTerrain->getVertexScratch(mVertexDataRecord->vertexCount);
            ct::Vector<TerrainVertexRange> touched;
            mTouched = &touched;
            updateVertexBuffer(*mVertexDataRecord, scratch, true, true, updateRect);
            mTouched = nullptr;
            TerrainGeometryTarget* target = mTerrain->getGeometryTarget();
            if (target)
                for (std::size_t i = 0; i < touched.size(); ++i)
                    target->writeVertexRange(this, touched[i].first, touched[i].count, scratch);
        }
    }

    if (!isLeaf())
        for (int i = 0; i < 4; ++i)
            mChildren[i]->updateVertexData(positions, deltas, rect, cpuData);
}

TerrainQuadTreeNode* TerrainQuadTreeNode::getNodeWithVertexData() const
{
    return mNodeWithVertexData;
}

bool TerrainQuadTreeNode::ownsVertexData() const
{
    return mVertexDataRecord != nullptr;
}

const TerrainVertexData* TerrainQuadTreeNode::getVertexDataRecord() const
{
    return mNodeWithVertexData ? mNodeWithVertexData->mVertexDataRecord : nullptr;
}

bool TerrainQuadTreeNode::getIndexKey(std::uint16_t lod, TerrainIndexKey& key) const
{
    const TerrainVertexData* vdr = getVertexDataRecord();
    if (!vdr || lod >= mLodLevels.size()) return false;

    const std::uint16_t batchSize = mLodLevels[lod].batchSize;
    const std::uint16_t resolutionRatio =
            static_cast<std::uint16_t>((mTerrain->getSize() - 1) / (vdr->resolution - 1));
    std::uint16_t vertexIncrement = static_cast<std::uint16_t>((mSize - 1) / (batchSize - 1));
    vertexIncrement = static_cast<std::uint16_t>(vertexIncrement / resolutionRatio);
    const std::uint16_t vdatasizeOffsetX = static_cast<std::uint16_t>(
            (mOffsetX - mNodeWithVertexData->mOffsetX) / resolutionRatio);
    const std::uint16_t vdatasizeOffsetY = static_cast<std::uint16_t>(
            (mOffsetY - mNodeWithVertexData->mOffsetY) / resolutionRatio);

    key.batchSize = batchSize;
    key.vdatasize = vdr->size;
    key.vertexIncrement = vertexIncrement;
    key.xoffset = vdatasizeOffsetX;
    key.yoffset = vdatasizeOffsetY;
    key.numSkirtRowsCols = vdr->numSkirtRowsCols;
    key.skirtRowColSkip = vdr->skirtRowColSkip;
    return true;
}

void TerrainQuadTreeNode::createCpuVertexData()
{
    if (!mVertexDataRecord) return;
    destroyCpuVertexData();

    TerrainVertexData& data = *mVertexDataRecord;

    std::size_t numVerts = static_cast<std::size_t>(data.size) * data.size;
    const std::uint16_t levels = data.treeLevels;
    data.numSkirtRowsCols = static_cast<std::uint16_t>(powf(2.0f, static_cast<float>(levels)) + 1);
    data.skirtRowColSkip =
            static_cast<std::uint16_t>((data.size - 1) / (data.numSkirtRowsCols - 1));
    numVerts += static_cast<std::size_t>(data.size) * data.numSkirtRowsCols;
    numVerts += static_cast<std::size_t>(data.size) * data.numSkirtRowsCols;

    data.vertexCount = static_cast<std::uint32_t>(numVerts);
    data.positions.resize(numVerts);
    data.uvs.resize(numVerts);
    data.deltas.resize(numVerts);
    data.cpuDataPresent = true;

    const TerrainRect updateRect(mOffsetX, mOffsetY, mBoundaryX, mBoundaryY);
    updateVertexBuffer(data, data, true, true, updateRect);
}

void TerrainQuadTreeNode::destroyCpuVertexData()
{
    if (mVertexDataRecord && mVertexDataRecord->cpuDataPresent)
    {
        mVertexDataRecord->positions.clear();
        mVertexDataRecord->uvs.clear();
        mVertexDataRecord->deltas.clear();
        mVertexDataRecord->positions.shrink_to_fit();
        mVertexDataRecord->uvs.shrink_to_fit();
        mVertexDataRecord->deltas.shrink_to_fit();
        mVertexDataRecord->cpuDataPresent = false;
    }
}

void TerrainQuadTreeNode::updateVertexBuffer(const TerrainVertexData& layout,
        TerrainVertexData& data, bool positions, bool deltas, const TerrainRect& rect)
{
    const int inc = (mTerrain->getSize() - 1) / (layout.resolution - 1);
    int mainLeft = rect.left;
    int mainTop = rect.top;
    if (mainLeft % inc) mainLeft += inc - (mainLeft % inc);
    if (mainTop % inc) mainTop += inc - (mainTop % inc);
    const long destOffsetX = mainLeft <= mOffsetX ? 0 : (mainLeft - mOffsetX) / inc;
    const long destOffsetY = mainTop <= mOffsetY ? 0 : (mainTop - mOffsetY) / inc;

    const float uvScale = 1.0f / (mTerrain->getSize() - 1);
    const int rowskip = mTerrain->getSize() * inc;
    const std::size_t destRowSkip = layout.size;
    const std::size_t mainCount = static_cast<std::size_t>(layout.size) * layout.size;
    Math::Vec3 pos;

    std::size_t rowPos = static_cast<std::size_t>(destOffsetY) * destRowSkip +
                         static_cast<std::size_t>(destOffsetX);
    std::size_t rowDelta = rowPos;
    std::uint32_t baseHeight = static_cast<std::uint32_t>(mainTop) * mTerrain->getSize() +
                               static_cast<std::uint32_t>(mainLeft);
    std::uint32_t baseDelta = baseHeight;

    for (int y = mainTop; y < rect.bottom; y += inc)
    {
        std::uint32_t heightIndex = baseHeight;
        std::uint32_t deltaIndex = baseDelta;
        std::size_t posIndex = rowPos;
        std::size_t deltaDest = rowDelta;
        for (int x = mainLeft; x < rect.right; x += inc)
        {
            if (positions)
            {
                const float height = mTerrain->getHeightData()[heightIndex];
                mTerrain->getPoint(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                        height, pos);

                pos -= mLocalCentre;

                writePosVertex(data, posIndex, static_cast<std::uint16_t>(x),
                        static_cast<std::uint16_t>(y), height, pos, uvScale);
                ++posIndex;
                heightIndex += static_cast<std::uint32_t>(inc);
            }

            if (deltas)
            {
                writeDeltaVertex(data, deltaDest, mTerrain->getDeltaData()[deltaIndex],
                        static_cast<float>(mTerrain->getLODLevelWhenVertexEliminated(x, y)) - 1.0f);
                ++deltaDest;
                deltaIndex += static_cast<std::uint32_t>(inc);
            }
        }
        baseHeight += static_cast<std::uint32_t>(rowskip);
        baseDelta += static_cast<std::uint32_t>(rowskip);
        rowPos += destRowSkip;
        rowDelta += destRowSkip;
    }

    const int skirtSpacing = layout.skirtRowColSkip * inc;
    Math::Vec3 skirtOffset;
    mTerrain->getVector(0.0f, 0.0f, -mTerrain->getSkirtSize(), skirtOffset);

    int skirtStartX = rect.left;
    int skirtStartY = rect.top;
    if (skirtStartY % skirtSpacing) skirtStartY += skirtSpacing - (skirtStartY % skirtSpacing);
    if (skirtStartX % inc) skirtStartX += inc - (skirtStartX % inc);
    skirtStartY =
            skirtStartY > static_cast<int>(mOffsetY) ? skirtStartY : static_cast<int>(mOffsetY);

    std::size_t skirtRowStart =
            mainCount +
            destRowSkip * static_cast<std::size_t>((skirtStartY - mOffsetY) / skirtSpacing) +
            static_cast<std::size_t>((skirtStartX - mOffsetX) / inc);
    std::uint32_t skirtBaseHeight = static_cast<std::uint32_t>(skirtStartY) * mTerrain->getSize() +
                                    static_cast<std::uint32_t>(skirtStartX);
    for (int y = skirtStartY; y < rect.bottom; y += skirtSpacing)
    {
        std::uint32_t heightIndex = skirtBaseHeight;
        std::size_t index = skirtRowStart;
        for (int x = skirtStartX; x < rect.right; x += inc)
        {
            if (positions)
            {
                const float height = mTerrain->getHeightData()[heightIndex];
                mTerrain->getPoint(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                        height, pos);
                pos -= mLocalCentre;
                pos += skirtOffset;
                writePosVertex(data, index, static_cast<std::uint16_t>(x),
                        static_cast<std::uint16_t>(y), height - mTerrain->getSkirtSize(), pos,
                        uvScale);
                heightIndex += static_cast<std::uint32_t>(inc);
            }

            if (deltas) writeDeltaVertex(data, index, 0.0f, 99.0f);
            ++index;
        }
        skirtBaseHeight += static_cast<std::uint32_t>(mTerrain->getSize() * skirtSpacing);
        skirtRowStart += destRowSkip;
    }

    skirtStartX = rect.left;
    if (skirtStartX % skirtSpacing) skirtStartX += skirtSpacing - (skirtStartX % skirtSpacing);
    skirtStartY = rect.top;
    if (skirtStartY % inc) skirtStartY += inc - (skirtStartY % inc);
    skirtStartX =
            skirtStartX > static_cast<int>(mOffsetX) ? skirtStartX : static_cast<int>(mOffsetX);

    std::size_t skirtColStart =
            mainCount + static_cast<std::size_t>(layout.numSkirtRowsCols) * layout.size +
            destRowSkip * static_cast<std::size_t>((skirtStartX - mOffsetX) / skirtSpacing) +
            static_cast<std::size_t>((skirtStartY - mOffsetY) / inc);
    for (int x = skirtStartX; x < rect.right; x += skirtSpacing)
    {
        std::size_t index = skirtColStart;
        for (int y = skirtStartY; y < rect.bottom; y += inc)
        {
            if (positions)
            {
                const float height = mTerrain->getHeightAtPoint(static_cast<std::uint32_t>(x),
                        static_cast<std::uint32_t>(y));
                mTerrain->getPoint(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                        height, pos);
                pos -= mLocalCentre;
                pos += skirtOffset;

                writePosVertex(data, index, static_cast<std::uint16_t>(x),
                        static_cast<std::uint16_t>(y), height - mTerrain->getSkirtSize(), pos,
                        uvScale);
            }
            if (deltas) writeDeltaVertex(data, index, 0.0f, 99.0f);
            ++index;
        }
        skirtColStart += destRowSkip;
    }
}

void TerrainQuadTreeNode::writePosVertex(TerrainVertexData& data, std::size_t index,
        std::uint16_t x, std::uint16_t y, float height, const Math::Vec3& pos, float uvScale)
{
    (void) height;
    data.positions[index] = pos;
    data.uvs[index] = Math::Vec2(x * uvScale, 1.0f - (y * uvScale));
    noteVertex(index);
}

void TerrainQuadTreeNode::noteVertex(std::size_t index)
{
    if (!mTouched) return;
    const std::uint32_t vertex = static_cast<std::uint32_t>(index);
    if (!mTouched->empty())
    {
        TerrainVertexRange& last = (*mTouched)[mTouched->size() - 1];
        if (vertex >= last.first && vertex < last.first + last.count) return;
        if (vertex == last.first + last.count)
        {
            ++last.count;
            return;
        }
    }
    TerrainVertexRange range;
    range.first = vertex;
    range.count = 1;
    mTouched->push_back(range);
}

void TerrainQuadTreeNode::writeDeltaVertex(TerrainVertexData& data, std::size_t index, float delta,
        float deltaThresh)
{
    data.deltas[index] = Math::Vec2(delta, deltaThresh);
    noteVertex(index);
}

std::uint16_t TerrainQuadTreeNode::calcSkirtVertexIndex(std::uint16_t mainIndex, bool isCol) const
{
    const TerrainVertexData* vdr = getVertexDataRecord();
    return Terrain::calcSkirtVertexIndex(mainIndex, vdr->size, isCol, vdr->numSkirtRowsCols,
            vdr->skirtRowColSkip);
}

void TerrainQuadTreeNode::refreshBounds(const TerrainRect& rect)
{
    if (rect.intersect(TerrainRect(mOffsetX, mOffsetY, mBoundaryX, mBoundaryY)).isNull()) return;

    if (!isLeaf())
        for (int i = 0; i < 4; ++i) mChildren[i]->refreshBounds(rect);

    mAABB = Math::Box();
    mBoundingRadius = 0.0f;

    const TerrainVertexData* layout = getVertexDataRecord();
    const int inc = layout && layout->resolution > 1
                            ? (mTerrain->getSize() - 1) / (layout->resolution - 1)
                            : 1;
    int startX = mOffsetX;
    int startY = mOffsetY;
    if (startX % inc) startX += inc - (startX % inc);
    if (startY % inc) startY += inc - (startY % inc);

    if (startX < mBoundaryX && startY < mBoundaryY)
    {
        int xAxis = 0;
        int yAxis = 2;
        int hAxis = 1;
        switch (mTerrain->getAlignment())
        {
            case TerrainAlignment::AlignXZ:
                break;
            case TerrainAlignment::AlignYZ:
                xAxis = 2;
                yAxis = 1;
                hAxis = 0;
                break;
            case TerrainAlignment::AlignXY:
                xAxis = 0;
                yAxis = 1;
                hAxis = 2;
                break;
        }

        const int columns = (mBoundaryX - 1 - startX) / inc + 1;
        const int rows = (mBoundaryY - 1 - startY) / inc + 1;
        ct::Vector<float> columnLocal;
        ct::Vector<float> columnSquare;
        ct::Vector<float> rowLocal;
        columnLocal.reserve(static_cast<std::size_t>(columns));
        columnSquare.reserve(static_cast<std::size_t>(columns));
        rowLocal.reserve(static_cast<std::size_t>(rows));
        Math::Vec3 pos;
        const float centre[3] = { mLocalCentre.x, mLocalCentre.y, mLocalCentre.z };
        float lo[3] = { FLT_MAX, FLT_MAX, FLT_MAX };
        float hi[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
        for (int c = 0; c < columns; ++c)
        {
            mTerrain->getPoint(static_cast<std::uint32_t>(startX + c * inc),
                    static_cast<std::uint32_t>(startY), 0.0f, pos);
            const float v[3] = { pos.x, pos.y, pos.z };
            const float local = v[xAxis] - centre[xAxis];
            columnLocal.push_back(local);
            columnSquare.push_back(local * local);
            lo[xAxis] = lo[xAxis] < local ? lo[xAxis] : local;
            hi[xAxis] = hi[xAxis] > local ? hi[xAxis] : local;
        }
        for (int r = 0; r < rows; ++r)
        {
            mTerrain->getPoint(static_cast<std::uint32_t>(startX),
                    static_cast<std::uint32_t>(startY + r * inc), 0.0f, pos);
            const float v[3] = { pos.x, pos.y, pos.z };
            const float local = v[yAxis] - centre[yAxis];
            rowLocal.push_back(local);
            lo[yAxis] = lo[yAxis] < local ? lo[yAxis] : local;
            hi[yAxis] = hi[yAxis] > local ? hi[yAxis] : local;
        }

        const float* heights = mTerrain->getHeightData();
        const std::size_t stride = mTerrain->getSize();
        const float heightCentre = centre[hAxis];
        float minHeight = FLT_MAX;
        float maxHeight = -FLT_MAX;
        float maxSquare = 0.0f;
        for (int r = 0; r < rows; ++r)
        {
            const float* row = heights + static_cast<std::size_t>(startY + r * inc) * stride +
                               static_cast<std::size_t>(startX);
            const float rowSquare =
                    rowLocal[static_cast<std::size_t>(r)] * rowLocal[static_cast<std::size_t>(r)];
            for (int c = 0; c < columns; ++c)
            {
                const float h = row[static_cast<std::size_t>(c * inc)];
                const float local = h - heightCentre;
                const float square =
                        columnSquare[static_cast<std::size_t>(c)] + rowSquare + local * local;
                minHeight = minHeight < local ? minHeight : local;
                maxHeight = maxHeight > local ? maxHeight : local;
                maxSquare = maxSquare > square ? maxSquare : square;
            }
        }
        lo[hAxis] = minHeight;
        hi[hAxis] = maxHeight;
        mAABB = Math::Box(Math::Vec3(lo[0], lo[1], lo[2]), Math::Vec3(hi[0], hi[1], hi[2]));
        mBoundingRadius = sqrtf(maxSquare);
    }

    if (!isLeaf())
    {
        for (int i = 0; i < 4; ++i)
        {
            Math::Box childBox = mChildren[i]->getBoundingBox();
            if (terrain_math::isNullBox(childBox)) continue;
            const Math::Vec3 boxoffset = mChildren[i]->getLocalCentre() - getLocalCentre();
            childBox.min = childBox.min + boxoffset;
            childBox.max = childBox.max + boxoffset;
            mAABB = mAABB.Union(childBox);
        }
    }
}

bool TerrainQuadTreeNode::rectIntersectsNode(const TerrainRect& rect) const
{
    return rect.right >= mOffsetX && rect.left <= mBoundaryX && rect.bottom >= mOffsetY &&
           rect.top <= mBoundaryY;
}

const Math::Box& TerrainQuadTreeNode::getBoundingBox() const
{
    return mAABB;
}

float TerrainQuadTreeNode::getBoundingRadius() const
{
    return mBoundingRadius;
}

const Math::Vec3& TerrainQuadTreeNode::getLocalCentre() const
{
    return mLocalCentre;
}

float TerrainQuadTreeNode::getMinHeight() const
{
    switch (mTerrain->getAlignment())
    {
        case TerrainAlignment::AlignXY:
        default:
            return mAABB.min.z;
        case TerrainAlignment::AlignXZ:
            return mAABB.min.y;
        case TerrainAlignment::AlignYZ:
            return mAABB.min.x;
    }
}

float TerrainQuadTreeNode::getMaxHeight() const
{
    switch (mTerrain->getAlignment())
    {
        case TerrainAlignment::AlignXY:
        default:
            return mAABB.max.z;
        case TerrainAlignment::AlignXZ:
            return mAABB.max.y;
        case TerrainAlignment::AlignYZ:
            return mAABB.max.x;
    }
}

Math::Box TerrainQuadTreeNode::getWorldBoundingBox() const
{
    if (terrain_math::isNullBox(mAABB)) return mAABB;
    const Math::Vec3 origin = mTerrain->getPosition() + mLocalCentre;
    return Math::Box(mAABB.min + origin, mAABB.max + origin);
}

bool TerrainQuadTreeNode::calculateCurrentLod(const TerrainLodView& view, float cFactor)
{
    mSelfOrChildRendered = false;

    const Math::Box worldBox = getWorldBoundingBox();
    const bool visible = !terrain_math::isNullBox(worldBox) &&
                         (view.frustum == nullptr || view.frustum->IntersectsBox(worldBox));
    if (!visible)
    {
        clearCurrentLod();
        return mSelfOrChildRendered;
    }

    int childRenderedCount = 0;
    if (!isLeaf())
    {
        for (int i = 0; i < 4; ++i)
            if (mChildren[i]->calculateCurrentLod(view, cFactor)) ++childRenderedCount;
    }

    if (!isAttached())
    {
        mCurrentLod = -1;
        return mSelfOrChildRendered;
    }

    if (childRenderedCount == 0)
    {
        const Math::Vec3 localPos = view.position - mLocalCentre - mTerrain->getPosition();
        float dist;
        if (mTerrain->getUseRayBoxDistanceCalculation())
        {
            const Math::Vec3 dir = (mAABB.Center() - localPos).Normalized();
            const Math::Ray ray(localPos, dir);
            dist = terrain_math::intersectsBox(ray, mAABB).distance;
        }
        else
        {
            dist = localPos.Length();
            dist -= (mBoundingRadius * 0.5f);
        }

        float materialDistance = localPos.Dot(localPos) - mBoundingRadius * mBoundingRadius;
        materialDistance = materialDistance > 0.0f ? materialDistance : 0.0f;
        mMaterialLodIndex = mTerrain->getMaterialLodIndex(materialDistance * view.lodBiasInverse);

        std::uint32_t lodLvl = 0;
        mCurrentLod = -1;
        for (std::size_t i = 0; i < mLodLevels.size(); ++i, ++lodLvl)
        {
            if (lodLvl + 1 == mLodLevels.size() && !mParent)
            {
                mCurrentLod = static_cast<int>(lodLvl);
                mSelfOrChildRendered = true;
                mLodTransition = 0.0f;
            }
            else
            {
                LodLevel& ll = mLodLevels[i];

                float distTransition;
                if (fabsf(ll.lastCFactor - cFactor) <= FLT_EPSILON)
                {
                    distTransition = ll.lastTransitionDist;
                }
                else
                {
                    distTransition = ll.maxHeightDelta * cFactor;
                    ll.lastCFactor = cFactor;
                    ll.lastTransitionDist = distTransition;
                }

                if (dist < distTransition)
                {
                    mCurrentLod = static_cast<int>(lodLvl);
                    mSelfOrChildRendered = true;

                    if (mTerrain->_getMorphRequired())
                    {
                        float distTotal = distTransition;
                        if (isLeaf())
                        {
                            if (i != 0) distTotal -= mLodLevels[i - 1].lastTransitionDist;
                        }
                        else
                        {
                            const LodLevel* childLod = mChildWithMaxHeightDelta->getLodLevel(
                                    static_cast<std::uint16_t>(
                                            mChildWithMaxHeightDelta->getLodCount() - 1));
                            distTotal -= childLod->lastTransitionDist;
                        }
                        const float distMorphRegion = distTotal * 0.25f;
                        const float distRemain = distTransition - dist;

                        mLodTransition = 1.0f - (distRemain / distMorphRegion);
                        mLodTransition = mLodTransition < 1.0f ? mLodTransition : 1.0f;
                        mLodTransition = mLodTransition > 0.0f ? mLodTransition : 0.0f;

                        refreshLodMorph();
                    }
                    break;
                }
            }
        }
    }
    else
    {
        mCurrentLod = -1;
        mSelfOrChildRendered = true;
        if (childRenderedCount < 4)
        {
            for (int i = 0; i < 4; ++i)
            {
                TerrainQuadTreeNode* child = mChildren[i];
                if (!child->isSelfOrChildRenderedAtCurrentLod())
                {
                    child->setCurrentLod(child->getLodCount() - 1);
                    child->setLodTransition(1.0f);
                }
            }
        }
    }

    return mSelfOrChildRendered;
}

void TerrainQuadTreeNode::clearCurrentLod()
{
    mCurrentLod = -1;
    mSelfOrChildRendered = false;
    if (!isLeaf())
        for (int i = 0; i < 4; ++i) mChildren[i]->clearCurrentLod();
}

void TerrainQuadTreeNode::refreshLodMorph()
{
    mLodMorph = Math::Vec2(mLodTransition, static_cast<float>(mCurrentLod + mBaseLod + 1));
}

void TerrainQuadTreeNode::setCurrentLod(int lod)
{
    mCurrentLod = lod;
    refreshLodMorph();
}

int TerrainQuadTreeNode::getCurrentLod() const
{
    return mCurrentLod;
}

std::uint16_t TerrainQuadTreeNode::getMaterialLodIndex() const
{
    return mMaterialLodIndex;
}

bool TerrainQuadTreeNode::isRenderedAtCurrentLod() const
{
    return mCurrentLod != -1;
}

bool TerrainQuadTreeNode::isSelfOrChildRenderedAtCurrentLod() const
{
    return mSelfOrChildRendered;
}

float TerrainQuadTreeNode::getLodTransition() const
{
    return mLodTransition;
}

void TerrainQuadTreeNode::setLodTransition(float t)
{
    mLodTransition = t;
    refreshLodMorph();
}

Math::Vec2 TerrainQuadTreeNode::getLodMorph() const
{
    return mLodMorph;
}

} // namespace prisma3d
