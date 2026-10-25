#include "Check.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/terrain/Terrain.h"
#include "prisma3d/terrain/TerrainQuadTreeNode.h"

#include <ct/vector.hpp>

#include <malloc.h>
#include <math.h>
#include <time.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

bool isVec3(const Math::Vec3& v, float x, float y, float z, float tolerance = 1e-3f)
{
    return near(v.x, x, tolerance) && near(v.y, y, tolerance) && near(v.z, z, tolerance);
}

double nowMilliseconds()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0 + static_cast<double>(ts.tv_nsec) / 1.0e6;
}

TerrainImportData importData(std::uint16_t size, std::uint16_t minBatch, std::uint16_t maxBatch,
        float worldSize)
{
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = minBatch;
    data.maxBatchSize = maxBatch;
    data.worldSize = worldSize;
    return data;
}

enum class Shape
{
    Plane,
    Ramp,
    Saddle,
    Pyramid,
    Spike
};

float shapeHeight(Shape shape, int x, int y)
{
    switch (shape)
    {
        case Shape::Plane:
            return 7.0f;
        case Shape::Ramp:
            return 0.5f * static_cast<float>(x);
        case Shape::Saddle:
            return 0.01f * static_cast<float>(x) * static_cast<float>(y);
        case Shape::Pyramid:
        {
            const float dx = static_cast<float>(abs(x - 64));
            const float dy = static_cast<float>(abs(y - 64));
            const float h = 10.0f - 0.5f * (dx > dy ? dx : dy);
            return h > 0.0f ? h : 0.0f;
        }
        case Shape::Spike:
            return (x == 64 && y == 64) ? 8.0f : 0.0f;
    }
    return 0.0f;
}

bool buildShape(Terrain& terrain, Shape shape, std::uint16_t size, std::uint16_t minBatch,
        std::uint16_t maxBatch, float worldSize, const Math::Vec3& pos = Math::Vec3(0, 0, 0))
{
    ct::Vector<float> heights;
    heights.resize(static_cast<std::size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            heights[static_cast<std::size_t>(y) * size + x] = shapeHeight(shape, x, y);
    TerrainImportData data = importData(size, minBatch, maxBatch, worldSize);
    data.inputFloat = heights.data();
    data.pos = pos;
    terrain._setMorphRequired(true);
    const bool ok = terrain.prepare(data);
    if (ok) terrain.load();
    return ok;
}

void countNodes(const TerrainQuadTreeNode* node, int* perDepth, int& owners)
{
    ++perDepth[node->getDepth()];
    if (node->ownsVertexData()) ++owners;
    if (!node->isLeaf())
        for (unsigned short i = 0; i < 4; ++i) countNodes(node->getChild(i), perDepth, owners);
}

const TerrainQuadTreeNode* findNode(const TerrainQuadTreeNode* node, int xoff, int yoff, int depth)
{
    if (node->getDepth() == depth)
        return (node->getXOffset() == xoff && node->getYOffset() == yoff) ? node : nullptr;
    if (node->isLeaf()) return nullptr;
    for (unsigned short i = 0; i < 4; ++i)
        if (const TerrainQuadTreeNode* found = findNode(node->getChild(i), xoff, yoff, depth))
            return found;
    return nullptr;
}

void testShapes()
{
    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 129, 17, 33, 128.0f));
        CHECK(terrain.getNumLodLevels() == 4);
        CHECK(terrain.getNumLodLevelsPerLeaf() == 2);
        CHECK(terrain.getTreeDepth() == 3);
        int perDepth[8] = {};
        int owners = 0;
        countNodes(terrain.getQuadTree(), perDepth, owners);
        CHECK(perDepth[0] == 1 && perDepth[1] == 4 && perDepth[2] == 16 && perDepth[3] == 0);
        CHECK(owners == 1);

        const TerrainQuadTreeNode* root = terrain.getQuadTree();
        CHECK(root->ownsVertexData() && !root->isLeaf());
        CHECK(root->getBaseLod() == 3 && root->getLodCount() == 1);
        CHECK(root->getLodLevel(0)->batchSize == 17);
        const TerrainVertexData* record = root->getVertexDataRecord();
        CHECK(record && record->resolution == 129 && record->size == 129 &&
                record->treeLevels == 3);
        CHECK(record->numSkirtRowsCols == 9 && record->skirtRowColSkip == 16);
        CHECK(record->vertexCount == 129u * 129u + 2u * 129u * 9u);

        const TerrainQuadTreeNode* middle = root->getChild(3);
        CHECK(middle->getDepth() == 1 && middle->getBaseLod() == 2 && middle->getLodCount() == 1);
        CHECK(middle->getXOffset() == 64 && middle->getYOffset() == 64 && middle->getSize() == 65);
        CHECK(middle->getNodeWithVertexData() == root && !middle->ownsVertexData());

        const TerrainQuadTreeNode* leaf = findNode(root, 96, 32, 2);
        CHECK(leaf && leaf->isLeaf() && leaf->getBaseLod() == 0 && leaf->getLodCount() == 2);
        CHECK(leaf->getSize() == 33);
        CHECK(leaf->getLodLevel(0)->batchSize == 33 && leaf->getLodLevel(1)->batchSize == 17);
        CHECK(leaf->getNodeWithVertexData() == root);
        CHECK(terrain.getLODLevelWhenVertexEliminated(0) == 4);
        CHECK(terrain.getLODLevelWhenVertexEliminated(8) == 4);
        CHECK(terrain.getLODLevelWhenVertexEliminated(16) == 4);
        CHECK(terrain.getLODLevelWhenVertexEliminated(4) == 3);
        CHECK(terrain.getLODLevelWhenVertexEliminated(12) == 3);
        CHECK(terrain.getLODLevelWhenVertexEliminated(2) == 2);
        CHECK(terrain.getLODLevelWhenVertexEliminated(6) == 2);
        CHECK(terrain.getLODLevelWhenVertexEliminated(1) == 1);
        CHECK(terrain.getLODLevelWhenVertexEliminated(127) == 1);
        CHECK(terrain.getLODLevelWhenVertexEliminated(8, 3) == 1);
        CHECK(terrain.getLODLevelWhenVertexEliminated(8, 4) == 3);
        CHECK(terrain.getLODLevelWhenVertexEliminated(16, 16) == 4);
        CHECK(terrain.getResolutionAtLod(0) == 129 && terrain.getResolutionAtLod(1) == 65);
        CHECK(terrain.getResolutionAtLod(3) == 17);
        CHECK(terrain.getGeoDataSizeAtLod(0) == 129u * 129u - 65u * 65u);
        CHECK(terrain.getGeoDataSizeAtLod(3) == 17u * 17u);
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 513, 33, 65, 12000.0f));
        CHECK(terrain.getNumLodLevels() == 5);
        CHECK(terrain.getNumLodLevelsPerLeaf() == 2);
        CHECK(terrain.getTreeDepth() == 4);
        int perDepth[8] = {};
        int owners = 0;
        countNodes(terrain.getQuadTree(), perDepth, owners);
        CHECK(perDepth[0] == 1 && perDepth[1] == 4 && perDepth[2] == 16 && perDepth[3] == 64);
        CHECK(owners == 17);

        const TerrainQuadTreeNode* root = terrain.getQuadTree();
        const TerrainVertexData* top = root->getVertexDataRecord();
        CHECK(top && top->resolution == 129 && top->size == 129 && top->treeLevels == 2);
        CHECK(top->numSkirtRowsCols == 5 && top->skirtRowColSkip == 32);
        CHECK(root->getChild(2)->getNodeWithVertexData() == root);
        CHECK(root->getBaseLod() == 4 && root->getChild(0)->getBaseLod() == 3);

        const TerrainQuadTreeNode* owner = findNode(root, 128, 256, 2);
        CHECK(owner && owner->ownsVertexData());
        const TerrainVertexData* record = owner->getVertexDataRecord();
        CHECK(record->resolution == 513 && record->size == 129 && record->treeLevels == 2);
        CHECK(record->numSkirtRowsCols == 5 && record->skirtRowColSkip == 32);
        CHECK(record->vertexCount == 129u * 129u + 2u * 129u * 5u);
        const TerrainQuadTreeNode* leaf = findNode(root, 192, 256, 3);
        CHECK(leaf && leaf->isLeaf() && leaf->getNodeWithVertexData() == owner);

        TerrainIndexKey key;
        const TerrainQuadTreeNode* sibling = findNode(root, 64, 0, 3);
        CHECK(sibling && sibling->getIndexKey(0, key));
        CHECK(key.batchSize == 65 && key.vdatasize == 129 && key.vertexIncrement == 1);
        CHECK(key.xoffset == 64 && key.yoffset == 0 && key.numSkirtRowsCols == 5);
        CHECK(key.skirtRowColSkip == 32);
        TerrainIndexKey leafKey;
        CHECK(sibling->getIndexKey(1, leafKey));
        CHECK(leafKey.batchSize == 33 && leafKey.vertexIncrement == 2 && leafKey.xoffset == 64);
        TerrainIndexKey middleKey;
        const TerrainQuadTreeNode* middle = findNode(root, 256, 0, 1);
        CHECK(middle && middle->getIndexKey(0, middleKey));
        CHECK(middleKey.pack() == leafKey.pack());
        CHECK(!sibling->getIndexKey(2, key));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 257, 17, 33, 256.0f));
        CHECK(terrain.getNumLodLevels() == 5 && terrain.getTreeDepth() == 4);
        int perDepth[8] = {};
        int owners = 0;
        countNodes(terrain.getQuadTree(), perDepth, owners);
        CHECK(perDepth[0] == 1 && perDepth[1] == 4 && perDepth[2] == 16 && perDepth[3] == 64);
        CHECK(owners == 5);
        const TerrainQuadTreeNode* root = terrain.getQuadTree();
        const TerrainVertexData* top = root->getVertexDataRecord();
        CHECK(top->resolution == 33 && top->size == 33 && top->treeLevels == 1);
        CHECK(top->numSkirtRowsCols == 3 && top->skirtRowColSkip == 16);
        CHECK(top->vertexCount == 33u * 33u + 2u * 33u * 3u);
        const TerrainQuadTreeNode* owner = root->getChild(1);
        CHECK(owner->ownsVertexData());
        const TerrainVertexData* record = owner->getVertexDataRecord();
        CHECK(record->resolution == 257 && record->size == 129 && record->treeLevels == 3);
        CHECK(record->numSkirtRowsCols == 9 && record->skirtRowColSkip == 16);
        CHECK(findNode(root, 224, 0, 3)->getNodeWithVertexData() == owner);
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 33, 5, 33, 32.0f));
        CHECK(terrain.getNumLodLevels() == 4 && terrain.getNumLodLevelsPerLeaf() == 4);
        CHECK(terrain.getTreeDepth() == 1);
        const TerrainQuadTreeNode* root = terrain.getQuadTree();
        CHECK(root->isLeaf() && root->ownsVertexData() && root->getLodCount() == 4);
        CHECK(root->getLodLevel(0)->batchSize == 33 && root->getLodLevel(1)->batchSize == 17);
        CHECK(root->getLodLevel(2)->batchSize == 9 && root->getLodLevel(3)->batchSize == 5);
        const TerrainVertexData* record = root->getVertexDataRecord();
        CHECK(record->resolution == 33 && record->size == 33 && record->treeLevels == 1);
        CHECK(record->vertexCount == 1287);
        CHECK(Terrain::getNumIndexesForBatchSize(33) == 2402);
        CHECK(Terrain::getNumIndexesForBatchSize(17) == 690);
        CHECK(Terrain::getNumIndexesForBatchSize(9) == 218);
        CHECK(Terrain::getNumIndexesForBatchSize(5) == 78);
    }

    {
        Terrain terrain;
        CHECK(!terrain.prepare(importData(130, 17, 33, 100.0f)));
        CHECK(!terrain.prepare(importData(129, 18, 33, 100.0f)));
        CHECK(!terrain.prepare(importData(129, 33, 17, 100.0f)));
        CHECK(!terrain.prepare(importData(129, 17, 257, 100.0f)));
        CHECK(!terrain.prepare(importData(2, 17, 33, 100.0f)));
        CHECK(!terrain.isPrepared());
        CHECK(terrain.getQuadTree() == nullptr);
        CHECK(terrain.getMinHeight() == 0.0f && terrain.getMaxHeight() == 0.0f);
    }
}

void testQueries()
{
    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 129, 17, 33, 128.0f));
        CHECK(near(terrain.getScale(), 1.0f) && near(terrain.getBase(), -64.0f));
        CHECK(near(terrain.getHeightAtPoint(3, 4), 7.0f));
        CHECK(near(terrain.getHeightAtPoint(500, 500), 7.0f));
        CHECK(near(terrain.getHeightAtTerrainPosition(0.3f, 0.7f), 7.0f));
        CHECK(near(terrain.getHeightAtTerrainPosition(0.0f, 0.0f), 7.0f));
        CHECK(near(terrain.getHeightAtTerrainPosition(1.0f, 1.0f), 7.0f));
        CHECK(near(terrain.getHeightAtWorldPosition(10.5f, 99.0f, -20.25f), 7.0f));
        CHECK(near(terrain.getHeightAtWorldPosition(Math::Vec3(-1000.0f, 0.0f, 1000.0f)), 7.0f));
        CHECK(near(terrain.getMinHeight(), 7.0f) && near(terrain.getMaxHeight(), 7.0f));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Ramp, 129, 17, 33, 128.0f,
                Math::Vec3(10.0f, 5.0f, 20.0f)));
        CHECK(near(terrain.getHeightAtPoint(40, 3), 20.0f));
        CHECK(near(terrain.getHeightAtTerrainPosition(0.3f, 0.2f), 19.2f, 1e-3f));
        CHECK(near(terrain.getHeightAtTerrainPosition(0.5f, 0.9f), 32.0f, 1e-3f));
        CHECK(near(terrain.getHeightAtTerrainPosition(1.0f, 0.5f), 64.0f, 1e-3f));
        CHECK(near(terrain.getHeightAtWorldPosition(-15.6f, 200.0f, 58.4f), 19.2f, 1e-3f));
        CHECK(near(terrain.getMinHeight(), 0.0f) && near(terrain.getMaxHeight(), 64.0f));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Saddle, 129, 17, 33, 128.0f));
        CHECK(near(terrain.getHeightAtTerrainPosition(10.0f / 128.0f, 10.0f / 128.0f), 1.0f,
                1e-3f));
        CHECK(near(terrain.getHeightAtTerrainPosition(10.75f / 128.0f, 10.25f / 128.0f), 1.1025f,
                1e-3f));
        CHECK(near(terrain.getHeightAtTerrainPosition(10.25f / 128.0f, 11.25f / 128.0f), 1.1525f,
                1e-3f));
        CHECK(near(terrain.getHeightAtTerrainPosition(10.5f / 128.0f, 10.5f / 128.0f), 1.105f,
                1e-3f));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Pyramid, 129, 17, 33, 128.0f));
        CHECK(near(terrain.getHeightAtWorldPosition(0.0f, 0.0f, 0.0f), 10.0f));
        CHECK(near(terrain.getHeightAtWorldPosition(1.0f, 0.0f, 0.0f), 9.5f));
        CHECK(near(terrain.getHeightAtWorldPosition(0.5f, 0.0f, 0.0f), 9.75f, 1e-3f));
        CHECK(near(terrain.getHeightAtWorldPosition(0.0f, 0.0f, -0.5f), 9.75f, 1e-3f));
        CHECK(near(terrain.getHeightAtWorldPosition(30.0f, 0.0f, 0.0f), 0.0f));
        CHECK(near(terrain.getMaxHeight(), 10.0f) && near(terrain.getMinHeight(), 0.0f));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 129, 17, 33, 128.0f,
                Math::Vec3(100.0f, 5.0f, -30.0f)));
        const Math::Vec3 corner = terrain.convertPosition(TerrainSpace::World,
                Math::Vec3(36.0f, 9.0f, 34.0f), TerrainSpace::Terrain);
        CHECK(isVec3(corner, 0.0f, 0.0f, 4.0f));
        const Math::Vec3 opposite = terrain.convertPosition(TerrainSpace::World,
                Math::Vec3(164.0f, 5.0f, -94.0f), TerrainSpace::Terrain);
        CHECK(isVec3(opposite, 1.0f, 1.0f, 0.0f));
        const Math::Vec3 point = terrain.convertPosition(TerrainSpace::Terrain,
                Math::Vec3(0.2f, 0.5f, 3.0f), TerrainSpace::Point);
        CHECK(isVec3(point, 26.0f, 64.0f, 3.0f));
        const Math::Vec3 back = terrain.convertPosition(TerrainSpace::Point,
                Math::Vec3(64.0f, 64.0f, 3.0f), TerrainSpace::World);
        CHECK(isVec3(back, 100.0f, 8.0f, -30.0f));
        const Math::Vec3 local = terrain.convertPosition(TerrainSpace::World,
                Math::Vec3(110.0f, 9.0f, -25.0f), TerrainSpace::Local);
        CHECK(isVec3(local, 10.0f, 4.0f, 5.0f));
        const Math::Vec3 direct = terrain.convertPosition(TerrainSpace::Terrain,
                Math::Vec3(0.25f, 0.75f, 2.0f), TerrainSpace::World);
        const Math::Vec3 roundTrip =
                terrain.convertPosition(TerrainSpace::World, direct, TerrainSpace::Terrain);
        CHECK(isVec3(direct, 68.0f, 7.0f, -62.0f));
        CHECK(isVec3(roundTrip, 0.25f, 0.75f, 2.0f));
        CHECK(isVec3(terrain.convertDirection(TerrainSpace::World, Math::Vec3(1, 0, 0),
                             TerrainSpace::Terrain),
                1.0f, 0.0f, 0.0f));
        CHECK(isVec3(terrain.convertDirection(TerrainSpace::World, Math::Vec3(0, 1, 0),
                             TerrainSpace::Terrain),
                0.0f, 0.0f, 1.0f));
        CHECK(isVec3(terrain.convertDirection(TerrainSpace::World, Math::Vec3(0, 0, -1),
                             TerrainSpace::Terrain),
                0.0f, 1.0f, 0.0f));
        CHECK(isVec3(terrain.convertDirection(TerrainSpace::Terrain, Math::Vec3(0, 1, 0),
                             TerrainSpace::World),
                0.0f, 0.0f, -1.0f));
        Math::Vec3 absolute;
        terrain.getTerrainPosition(36.0f, 9.0f, 34.0f, absolute);
        CHECK(isVec3(absolute, 0.0f, 0.0f, 9.0f));
        Math::Vec3 world;
        terrain.getPosition(0.5f, 0.5f, 2.0f, world);
        CHECK(isVec3(world, 0.0f, 2.0f, 0.0f));
        Math::Vec3 vector;
        terrain.getVector(1.0f, 2.0f, 3.0f, vector);
        CHECK(isVec3(vector, 1.0f, 3.0f, -2.0f));
        Math::Vec3 terrainVector;
        terrain.getTerrainVector(1.0f, 3.0f, -2.0f, terrainVector);
        CHECK(isVec3(terrainVector, 1.0f, 2.0f, 3.0f));
        Math::Vec3 gridPoint;
        terrain.getPoint(10, 20, gridPoint);
        CHECK(isVec3(gridPoint, -54.0f, 7.0f, 44.0f));
        terrain.getPoint(10, 20, 2.5f, gridPoint);
        CHECK(isVec3(gridPoint, -54.0f, 2.5f, 44.0f));
        Math::Vec3 other;
        terrain.getPointAlign(10, 20, 2.5f, TerrainAlignment::AlignXY, other);
        CHECK(isVec3(other, -54.0f, -44.0f, 2.5f));
        terrain.getPointAlign(10, 20, 2.5f, TerrainAlignment::AlignYZ, other);
        CHECK(isVec3(other, 2.5f, -44.0f, 54.0f));
    }

    {
        Terrain terrain;
        TerrainImportData data = importData(129, 17, 33, 128.0f);
        data.terrainAlign = TerrainAlignment::AlignXY;
        data.constantHeight = 3.0f;
        CHECK(terrain.prepare(data));
        const Math::Vec3 world = terrain.convertPosition(TerrainSpace::Terrain,
                Math::Vec3(0.25f, 0.75f, 2.0f), TerrainSpace::World);
        CHECK(isVec3(world, -32.0f, 32.0f, 2.0f));
        CHECK(isVec3(terrain.convertPosition(TerrainSpace::World, world, TerrainSpace::Terrain),
                0.25f, 0.75f, 2.0f));
        CHECK(near(terrain.getHeightAtWorldPosition(-32.0f, 32.0f, 99.0f), 3.0f));
        data.terrainAlign = TerrainAlignment::AlignYZ;
        CHECK(terrain.prepare(data));
        const Math::Vec3 yz = terrain.convertPosition(TerrainSpace::Terrain,
                Math::Vec3(0.25f, 0.75f, 2.0f), TerrainSpace::World);
        CHECK(isVec3(yz, 2.0f, 32.0f, -(0.25f * 128.0f - 64.0f) * 1.0f));
        CHECK(isVec3(terrain.convertPosition(TerrainSpace::World, yz, TerrainSpace::Terrain), 0.25f,
                0.75f, 2.0f));
    }
}

void testRays()
{
    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 129, 17, 33, 128.0f));
        TerrainRayResult hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(10.0f, 100.0f, 5.0f), Math::Vec3(0.0f, -1.0f, 0.0f)));
        CHECK(hit.hit && isVec3(hit.position, 10.0f, 7.0f, 5.0f, 1e-2f));
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(-60.0f, 8.0f, 0.0f), Math::Vec3(1.0f, -0.01f, 0.0f)));
        CHECK(hit.hit && isVec3(hit.position, 40.0f, 7.0f, 0.0f, 5e-2f));
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(-60.0f, 8.0f, -60.0f), Math::Vec3(1.0f, -0.02f, 1.0f)));
        CHECK(hit.hit && near(hit.position.y, 7.0f, 5e-2f));
        CHECK(near(hit.position.x - hit.position.z, 0.0f, 5e-2f));
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(0.0f, -5.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f)));
        CHECK(hit.hit && isVec3(hit.position, 0.0f, 7.0f, 0.0f, 1e-2f));
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(10.0f, 100.0f, 5.0f), Math::Vec3(0.0f, 1.0f, 0.0f)));
        CHECK(!hit.hit);
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(500.0f, 50.0f, 0.0f), Math::Vec3(0.0f, -1.0f, 0.0f)));
        CHECK(!hit.hit);
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(-100.0f, 50.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f)));
        CHECK(!hit.hit);
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(0.0f, 8.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f)));
        CHECK(!hit.hit);
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Ramp, 129, 17, 33, 128.0f,
                Math::Vec3(100.0f, 5.0f, -30.0f)));
        const TerrainRayResult hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(110.0f, 200.0f, -25.0f), Math::Vec3(0.0f, -1.0f, 0.0f)));
        CHECK(hit.hit && isVec3(hit.position, 110.0f, 42.0f, -25.0f, 1e-2f));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Pyramid, 129, 17, 33, 128.0f));
        TerrainRayResult hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(0.0f, 50.0f, 0.0f), Math::Vec3(0.0f, -1.0f, 0.0f)));
        CHECK(hit.hit && isVec3(hit.position, 0.0f, 10.0f, 0.0f, 1e-2f));
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(20.0f, 30.0f, 0.0f), Math::Vec3(-1.0f, -2.0f, 0.0f)));
        CHECK(hit.hit && isVec3(hit.position, 8.0f, 6.0f, 0.0f, 5e-2f));
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(-6.0f, 20.0f, 1.0f), Math::Vec3(0.0f, -1.0f, 0.0f)));
        CHECK(hit.hit && isVec3(hit.position, -6.0f, 7.0f, 1.0f, 1e-2f));
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(0.0f, 12.0f, 40.0f), Math::Vec3(0.0f, 0.0f, -1.0f)));
        CHECK(!hit.hit);
        hit = terrain.rayIntersects(
                Math::Ray(Math::Vec3(0.0f, 8.0f, 40.0f), Math::Vec3(0.0f, 0.0f, -1.0f)));
        CHECK(hit.hit && near(hit.position.y, 8.0f, 0.05f) && near(hit.position.z, 4.0f, 0.1f));
    }
}

void testDeltas()
{
    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Plane, 129, 17, 33, 128.0f));
        bool allZero = true;
        for (std::size_t i = 0; i < 129u * 129u; ++i)
            if (terrain.getDeltaData()[i] != 0.0f) allZero = false;
        CHECK(allZero);
        const TerrainQuadTreeNode* leaf = findNode(terrain.getQuadTree(), 32, 64, 2);
        CHECK(leaf->getLodLevel(0)->maxHeightDelta == 0.0f);
        CHECK(leaf->getLodLevel(1)->maxHeightDelta == 0.0f);
        CHECK(terrain.getQuadTree()->getLodLevel(0)->maxHeightDelta == 0.0f);
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Ramp, 129, 17, 33, 128.0f));
        bool allZero = true;
        for (std::size_t i = 0; i < 129u * 129u; ++i)
            if (fabsf(terrain.getDeltaData()[i]) > 1e-4f) allZero = false;
        CHECK(allZero);
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Spike, 129, 17, 33, 128.0f));
        CHECK(near(*terrain.getDeltaData(65, 64), 4.0f));
        CHECK(near(*terrain.getDeltaData(64, 65), 4.0f));
        CHECK(near(*terrain.getDeltaData(63, 64), 4.0f));
        CHECK(near(*terrain.getDeltaData(64, 63), 4.0f));
        CHECK(near(*terrain.getDeltaData(65, 65), 4.0f));
        CHECK(near(*terrain.getDeltaData(63, 63), 0.0f));
        CHECK(near(*terrain.getDeltaData(64, 64), 0.0f));
        CHECK(near(*terrain.getDeltaData(66, 64), 4.0f));
        CHECK(near(*terrain.getDeltaData(68, 64), 4.0f));
        CHECK(near(*terrain.getDeltaData(67, 64), 0.0f));
        CHECK(near(*terrain.getDeltaData(72, 64), 0.0f));
        CHECK(near(*terrain.getDeltaData(10, 10), 0.0f));

        const TerrainQuadTreeNode* leaf = findNode(terrain.getQuadTree(), 64, 64, 2);
        CHECK(leaf && near(leaf->getLodLevel(0)->maxHeightDelta, 4.0f));
        CHECK(near(leaf->getLodLevel(1)->maxHeightDelta, 6.0f));
        const TerrainQuadTreeNode* far = findNode(terrain.getQuadTree(), 96, 96, 2);
        CHECK(far && far->getLodLevel(0)->maxHeightDelta == 0.0f);
        CHECK(far->getLodLevel(1)->maxHeightDelta == 0.0f);
        const TerrainQuadTreeNode* middle = findNode(terrain.getQuadTree(), 64, 64, 1);
        CHECK(middle && near(middle->getLodLevel(0)->maxHeightDelta, 7.0f));
        CHECK(near(terrain.getQuadTree()->getLodLevel(0)->maxHeightDelta, 7.35f));
        const TerrainQuadTreeNode* flat = findNode(terrain.getQuadTree(), 0, 0, 2);
        CHECK(flat && flat->getLodLevel(0)->maxHeightDelta == 0.0f);
        CHECK(flat->getLodLevel(1)->maxHeightDelta == 0.0f);
    }
}

bool leafRenders(const TerrainQuadTreeNode* node, int expectedLod)
{
    return node->getCurrentLod() == expectedLod;
}

void testLodSelection()
{
    Terrain terrain;
    CHECK(buildShape(terrain, Shape::Spike, 129, 17, 33, 128.0f));
    terrain.setMaxPixelError(10.0f);

    TerrainLodView view;
    view.fovY = 90.0f * Math::DEG2RAD;
    view.viewportHeight = 1000.0f;
    CHECK(near(terrain.getCFactor(view), 50.0f, 1e-2f));
    terrain.setMaxPixelError(20.0f);
    CHECK(near(terrain.getCFactor(view), 25.0f, 1e-2f));
    view.lodBiasInverse = 2.0f;
    CHECK(near(terrain.getCFactor(view), 12.5f, 1e-2f));
    view.lodBiasInverse = 1.0f;
    view.viewportHeight = 500.0f;
    CHECK(near(terrain.getCFactor(view), 12.5f, 1e-2f));
    view.viewportHeight = 1000.0f;
    view.fovY = 60.0f * Math::DEG2RAD;
    CHECK(near(terrain.getCFactor(view),
            1.0f / tanf(30.0f * Math::DEG2RAD) / (2.0f * 20.0f / 1000.0f), 1e-2f));
    terrain.setMaxPixelError(10.0f);
    view.fovY = 90.0f * Math::DEG2RAD;

    const TerrainQuadTreeNode* leaf = findNode(terrain.getQuadTree(), 64, 64, 2);
    const TerrainQuadTreeNode* middle = findNode(terrain.getQuadTree(), 64, 64, 1);
    const float leafHalfRadius = 12.0f;

    view.position = Math::Vec3(16.0f, 200.0f, -16.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 0));
    CHECK(near(leaf->getLodTransition(), 1.0f - (200.0f - (200.0f - leafHalfRadius)) / 50.0f,
            2e-3f));
    CHECK(near(leaf->getLodMorph().x, 0.76f, 2e-3f) && near(leaf->getLodMorph().y, 1.0f));
    CHECK(middle->getCurrentLod() == -1 && terrain.getQuadTree()->getCurrentLod() == -1);

    view.position = Math::Vec3(16.0f, 162.0f, -16.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 0) && near(leaf->getLodTransition(), 0.0f, 2e-3f));
    view.position = Math::Vec3(16.0f, 187.0f, -16.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 0) && near(leaf->getLodTransition(), 0.5f, 2e-3f));
    view.position = Math::Vec3(16.0f, 211.99f, -16.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 0) && near(leaf->getLodTransition(), 1.0f, 2e-3f));
    view.position = Math::Vec3(16.0f, 212.01f, -16.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 1));

    view.position = Math::Vec3(16.0f, 250.0f, -16.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 1) && near(leaf->getLodTransition(), 0.0f, 2e-3f));
    CHECK(near(leaf->getLodMorph().y, 2.0f));
    view.position = Math::Vec3(16.0f, 290.0f, -16.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 1));
    CHECK(near(leaf->getLodTransition(), 1.0f - (300.0f - (290.0f - leafHalfRadius)) / 25.0f,
            2e-3f));
    CHECK(near(leaf->getLodTransition(), 0.12f, 2e-3f));

    view.position = Math::Vec3(32.0f, 370.0f, -32.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leaf->getCurrentLod() == -1);
    CHECK(middle->getCurrentLod() == 0);
    const float middleRadius = sqrtf(32.0f * 32.0f + 8.0f * 8.0f + 32.0f * 32.0f);
    const float middleDist = 370.0f - middleRadius * 0.5f;
    CHECK(near(middle->getLodTransition(), 1.0f - (350.0f - middleDist) / 12.5f, 5e-3f));
    CHECK(near(middle->getLodMorph().y, 3.0f));
    int rendered = 0;
    for (unsigned short i = 0; i < 4; ++i)
    {
        const TerrainQuadTreeNode* child = terrain.getQuadTree()->getChild(i);
        if (child->getCurrentLod() == 0) ++rendered;
        else
            CHECK(child->getCurrentLod() == -1);
    }
    CHECK(rendered == 4);
    CHECK(terrain.getQuadTree()->getCurrentLod() == -1);
    CHECK(findNode(terrain.getQuadTree(), 0, 0, 1)->getLodTransition() == 1.0f);
    CHECK(near(findNode(terrain.getQuadTree(), 0, 0, 1)->getLodMorph().x, 1.0f));
    CHECK(near(findNode(terrain.getQuadTree(), 0, 0, 1)->getLodMorph().y, 3.0f));

    view.position = Math::Vec3(0.0f, 100000.0f, 0.0f);
    terrain.calculateCurrentLod(view);
    CHECK(terrain.getQuadTree()->getCurrentLod() == 0);
    CHECK(terrain.getQuadTree()->getLodTransition() == 0.0f);
    CHECK(terrain.getQuadTree()->getChild(0)->getCurrentLod() == -1);
    CHECK(leaf->getCurrentLod() == -1 && middle->getCurrentLod() == -1);

    view.position = Math::Vec3(16.0f, 200.0f, -16.0f);
    terrain.setMaxPixelError(5.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leafRenders(leaf, 0));
    CHECK(near(leaf->getLodTransition(), 0.0f, 2e-3f));
    terrain.setMaxPixelError(20.0f);
    terrain.calculateCurrentLod(view);
    CHECK(leaf->getCurrentLod() == -1 && middle->getCurrentLod() == -1);
    CHECK(terrain.getQuadTree()->getCurrentLod() == 0);
    terrain.setMaxPixelError(10.0f);
}

bool anyRendered(const TerrainQuadTreeNode* node)
{
    if (node->isRenderedAtCurrentLod() || node->isSelfOrChildRenderedAtCurrentLod()) return true;
    if (!node->isLeaf())
        for (unsigned short i = 0; i < 4; ++i)
            if (anyRendered(node->getChild(i))) return true;
    return false;
}

void testLodCulling()
{
    Terrain terrain;
    CHECK(buildShape(terrain, Shape::Spike, 129, 17, 33, 128.0f));
    terrain.setMaxPixelError(10.0f);

    const float fovY = 6.0f * Math::DEG2RAD;
    const Math::Mat4 projection = Math::Mat4::Perspective(fovY, 1.0f, 0.5f, 2000.0f);
    const Math::Vec3 eye(16.0f, 200.0f, -16.0f);

    TerrainLodView view;
    view.position = eye;
    view.fovY = fovY;
    view.viewportHeight = 1000.0f;
    terrain.setMaxPixelError((1.0f / tanf(fovY * 0.5f)) * view.viewportHeight / (2.0f * 50.0f));

    const TerrainQuadTreeNode* leaf = findNode(terrain.getQuadTree(), 64, 64, 2);
    const TerrainQuadTreeNode* neighbour = findNode(terrain.getQuadTree(), 32, 32, 2);
    terrain.calculateCurrentLod(view);
    CHECK(leaf->getCurrentLod() == 0);
    CHECK(neighbour->getCurrentLod() == 0);

    const Math::Frustum down = Math::Frustum::FromViewProjection(
            projection * Math::Mat4::LookAt(eye, Math::Vec3(16.0f, 0.0f, -16.0f),
                                 Math::Vec3(0.0f, 0.0f, -1.0f)));
    view.frustum = &down;
    terrain.calculateCurrentLod(view);
    CHECK(leaf->getCurrentLod() == 0);
    CHECK(neighbour->getCurrentLod() == -1);
    const TerrainQuadTreeNode* sibling = findNode(terrain.getQuadTree(), 96, 96, 2);
    CHECK(sibling->getCurrentLod() == 1 && sibling->getLodTransition() == 1.0f);
    CHECK(near(sibling->getLodMorph().x, 1.0f) && near(sibling->getLodMorph().y, 2.0f));
    CHECK(anyRendered(terrain.getQuadTree()));

    const Math::Frustum away = Math::Frustum::FromViewProjection(
            projection * Math::Mat4::LookAt(eye, Math::Vec3(16.0f, 400.0f, -16.0f),
                                 Math::Vec3(0.0f, 0.0f, -1.0f)));
    view.frustum = &away;
    terrain.calculateCurrentLod(view);
    CHECK(!anyRendered(terrain.getQuadTree()));
    CHECK(leaf->getCurrentLod() == -1);
}

void collectVertexChecks(const TerrainQuadTreeNode* node, int& checked, int& failures_)
{
    const TerrainVertexData* data = node->getVertexDataRecord();
    const TerrainQuadTreeNode* owner = node->getNodeWithVertexData();
    if (data && data->cpuDataPresent)
    {
        const Terrain* terrain = node->getTerrain();
        const int inc = (terrain->getSize() - 1) / (data->resolution - 1);
        const Math::Box box = node->getBoundingBox();
        for (int y = node->getYOffset(); y < node->getYOffset() + node->getSize(); y += inc)
        {
            for (int x = node->getXOffset(); x < node->getXOffset() + node->getSize(); x += inc)
            {
                const int column = (x - owner->getXOffset()) / inc;
                const int row = (y - owner->getYOffset()) / inc;
                const Math::Vec3 position =
                        data->positions[static_cast<std::size_t>(row) * data->size + column] +
                        owner->getLocalCentre() - node->getLocalCentre();
                ++checked;
                const float eps = 1e-3f;
                if (position.x < box.min.x - eps || position.x > box.max.x + eps ||
                        position.y < box.min.y - eps || position.y > box.max.y + eps ||
                        position.z < box.min.z - eps || position.z > box.max.z + eps ||
                        position.Length() > node->getBoundingRadius() + eps)
                    ++failures_;
            }
        }
    }
    if (!node->isLeaf())
        for (unsigned short i = 0; i < 4; ++i)
            collectVertexChecks(node->getChild(i), checked, failures_);
}

void testBounds()
{
    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Pyramid, 129, 17, 33, 128.0f));
        int checked = 0;
        int bad = 0;
        collectVertexChecks(terrain.getQuadTree(), checked, bad);
        CHECK(checked > 129 * 129);
        CHECK(bad == 0);
        const TerrainQuadTreeNode* apexLeaf = findNode(terrain.getQuadTree(), 64, 64, 2);
        CHECK(isVec3(apexLeaf->getLocalCentre(), 16.0f, 0.0f, -16.0f));
        CHECK(near(apexLeaf->getMinHeight(), 0.0f));
        CHECK(near(apexLeaf->getMaxHeight(), 10.0f));
        CHECK(near(apexLeaf->getBoundingBox().min.x, -16.0f) &&
                near(apexLeaf->getBoundingBox().max.x, 16.0f));
        CHECK(near(apexLeaf->getBoundingBox().min.z, -16.0f) &&
                near(apexLeaf->getBoundingBox().max.z, 16.0f));
        const Math::Box world = terrain.getWorldAABB();
        CHECK(isVec3(world.min, -64.0f, 0.0f, -64.0f) && isVec3(world.max, 64.0f, 10.0f, 64.0f));
        CHECK(near(terrain.getBoundingRadius(), sqrtf(64.0f * 64.0f * 2.0f), 0.01f));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Spike, 129, 17, 33, 128.0f));
        const TerrainQuadTreeNode* leaf = findNode(terrain.getQuadTree(), 64, 64, 2);
        CHECK(near(leaf->getBoundingRadius(), 24.0f, 1e-3f));
        CHECK(near(leaf->getBoundingBox().max.y, 8.0f) && near(leaf->getBoundingBox().min.y, 0.0f));
        const Math::Box world = leaf->getWorldBoundingBox();
        CHECK(isVec3(world.min, 0.0f, 0.0f, -32.0f) && isVec3(world.max, 32.0f, 8.0f, 0.0f));
    }

    {
        Terrain terrain;
        CHECK(buildShape(terrain, Shape::Pyramid, 513, 33, 65, 12000.0f));
        int checked = 0;
        int bad = 0;
        collectVertexChecks(terrain.getQuadTree(), checked, bad);
        CHECK(checked > 513 * 513 && bad == 0);
    }
}

void testVertexData()
{
    Terrain terrain;
    CHECK(buildShape(terrain, Shape::Ramp, 33, 5, 33, 32.0f));
    terrain.setSkirtSize(30.0f);
    const TerrainQuadTreeNode* root = terrain.getQuadTree();
    const TerrainVertexData* data = root->getVertexDataRecord();
    CHECK(data && data->cpuDataPresent && data->positions.size() == 1287);
    CHECK(data->uvs.size() == 1287 && data->deltas.size() == 1287);

    CHECK(near(root->getLocalCentre().x, 0.0f) && near(root->getLocalCentre().z, 0.0f));
    CHECK(isVec3(data->positions[5 * 33 + 7], -9.0f, 3.5f, 11.0f));
    CHECK(near(data->uvs[5 * 33 + 7].x, 7.0f / 32.0f) &&
            near(data->uvs[5 * 33 + 7].y, 1.0f - 5.0f / 32.0f));
    const float slope = 0.5f / sqrtf(1.25f);
    const Math::Vec3 rampNormal = terrain.calculateNormal(7, 5);
    CHECK(near(rampNormal.x, -slope, 1e-3f));
    CHECK(near(rampNormal.y, 1.0f / sqrtf(1.25f), 1e-3f));
    CHECK(near(rampNormal.z, 0.0f, 1e-3f));
    CHECK(near(data->deltas[5 * 33 + 7].x, 0.0f));
    CHECK(near(data->deltas[5 * 33 + 7].y,
            static_cast<float>(terrain.getLODLevelWhenVertexEliminated(7, 5)) - 1.0f));
    CHECK(near(data->deltas[0].y, 3.0f));

    const std::size_t main = 33 * 33;
    const std::size_t rowsBase = main;
    const std::size_t colsBase = main + 3 * 33;
    CHECK(isVec3(data->positions[rowsBase + 0 * 33 + 4], -12.0f, 2.0f - 30.0f, 16.0f));
    CHECK(isVec3(data->positions[rowsBase + 1 * 33 + 4], -12.0f, 2.0f - 30.0f, 0.0f));
    CHECK(isVec3(data->positions[rowsBase + 2 * 33 + 4], -12.0f, 2.0f - 30.0f, -16.0f));
    CHECK(isVec3(data->positions[colsBase + 0 * 33 + 6], -16.0f, 0.0f - 30.0f, 10.0f));
    CHECK(isVec3(data->positions[colsBase + 2 * 33 + 6], 16.0f, 16.0f - 30.0f, 10.0f));
    CHECK(data->deltas[rowsBase + 5].x == 0.0f && data->deltas[rowsBase + 5].y == 99.0f);
    CHECK(data->deltas[colsBase + 5].x == 0.0f && data->deltas[colsBase + 5].y == 99.0f);

    std::size_t skirtVerticesUsed = 0;
    const std::uint16_t lodBatch[4] = { 33, 17, 9, 5 };
    const std::size_t expectedCount[4] = { 2402, 690, 218, 78 };
    for (std::uint16_t lod = 0; lod < 4; ++lod)
    {
        TerrainIndexKey key;
        CHECK(root->getIndexKey(lod, key));
        CHECK(key.batchSize == lodBatch[lod] && key.vdatasize == 33);
        CHECK(key.vertexIncrement == 32 / (lodBatch[lod] - 1));
        CHECK(key.xoffset == 0 && key.yoffset == 0);
        CHECK(key.numSkirtRowsCols == 3 && key.skirtRowColSkip == 16);
        ct::Vector<std::uint16_t> indices;
        indices.resize(Terrain::getNumIndexesForBatchSize(key.batchSize));
        CHECK(indices.size() == expectedCount[lod]);
        Terrain::populateIndexBuffer(indices.data(), key.batchSize, key.vdatasize,
                key.vertexIncrement, key.xoffset, key.yoffset, key.numSkirtRowsCols,
                key.skirtRowColSkip);

        std::size_t mainIndices = 0;
        std::size_t skirtIndices = 0;
        bool inRange = true;
        for (std::size_t i = 0; i < indices.size(); ++i)
        {
            if (indices[i] >= data->vertexCount) inRange = false;
            if (indices[i] >= main) ++skirtIndices;
            else
                ++mainIndices;
        }
        CHECK(inRange);
        const std::size_t b = key.batchSize;
        CHECK(skirtIndices == (b - 1) * 4 + 1);
        CHECK(mainIndices == indices.size() - skirtIndices);
        if (lod == 0) skirtVerticesUsed = skirtIndices;

        const std::size_t mainStripLength = (b * 2 + 1) * (b - 1);
        std::size_t upward = 0;
        std::size_t degenerate = 0;
        std::size_t downward = 0;
        for (std::size_t i = 0; i + 2 < mainStripLength; ++i)
        {
            std::uint16_t a = indices[i];
            std::uint16_t bb = indices[i + 1];
            std::uint16_t c = indices[i + 2];
            if (a == bb || bb == c || a == c)
            {
                ++degenerate;
                continue;
            }
            if (i % 2)
            {
                const std::uint16_t t = a;
                a = bb;
                bb = t;
            }
            const Math::Vec3 n = (data->positions[bb] - data->positions[a])
                                         .Cross(data->positions[c] - data->positions[a]);
            if (n.y > 0.0f) ++upward;
            else
                ++downward;
        }
        CHECK(downward == 0);
        CHECK(upward == 2 * (b - 1) * (b - 1));
        CHECK(degenerate == mainStripLength - 2 - upward);

        std::size_t pairs = 0;
        bool skirtsHang = true;
        for (std::size_t i = mainStripLength; i + 1 < indices.size(); i += 2)
        {
            const std::uint16_t edge = indices[i];
            const std::uint16_t skirt = indices[i + 1];
            ++pairs;
            if (edge >= main || skirt < main) skirtsHang = false;
            else
            {
                const Math::Vec3& e = data->positions[edge];
                const Math::Vec3& s = data->positions[skirt];
                if (!near(e.x, s.x) || !near(e.z, s.z) || !near(e.y - s.y, 30.0f, 1e-3f))
                    skirtsHang = false;
            }
        }
        CHECK(skirtsHang);
        CHECK(pairs == (b - 1) * 4 + 1);
    }
    CHECK(skirtVerticesUsed == 129);

    terrain.freeCpuVertexData();
    CHECK(!root->getVertexDataRecord()->cpuDataPresent);
    CHECK(root->getVertexDataRecord()->positions.empty());
}

void testSkirtSize()
{
    Terrain terrain;
    TerrainImportData data = importData(33, 5, 33, 32.0f);
    data.constantHeight = 2.0f;
    data.skirtSize = 5.0f;
    CHECK(terrain.prepare(data));
    const TerrainVertexData* record = terrain.getQuadTree()->getVertexDataRecord();
    CHECK(near(record->positions[33 * 33].y, 2.0f - 5.0f));
    CHECK(near(terrain.getSkirtSize(), 5.0f));
    data.skirtSize = 0.0f;
    CHECK(terrain.prepare(data));
    CHECK(near(terrain.getQuadTree()->getVertexDataRecord()->positions[33 * 33].y, 2.0f));
}

void testImport()
{
    {
        ct::Vector<float> source;
        source.resize(33u * 33u);
        for (std::size_t i = 0; i < source.size(); ++i) source[i] = static_cast<float>(i) * 0.5f;
        Terrain terrain;
        TerrainImportData data = importData(33, 5, 33, 32.0f);
        data.inputFloat = source.data();
        data.inputScale = 2.0f;
        data.inputBias = 3.0f;
        CHECK(terrain.prepare(data));
        CHECK(near(*terrain.getHeightData(0, 0), 3.0f));
        CHECK(near(*terrain.getHeightData(1, 0), 4.0f));
        CHECK(near(*terrain.getHeightData(0, 2), 2.0f * 0.5f * 66.0f + 3.0f));
        CHECK(near(*terrain.getHeightData(32, 32), 2.0f * 0.5f * 1088.0f + 3.0f));
        data.inputScale = 1.0f;
        data.inputBias = 0.0f;
        CHECK(terrain.prepare(data));
        CHECK(near(*terrain.getHeightData(5, 1), 0.5f * 38.0f));
    }

    {
        Pixmap image(33, 33, 1);
        for (int i = 0; i < 33 * 33; ++i) image.pixels[i] = 0;
        for (int x = 0; x < 33; ++x) image.pixels[x] = 255;
        image.pixels[16 * 33 + 5] = 51;
        Terrain terrain;
        TerrainImportData data = importData(33, 5, 33, 32.0f);
        data.inputImage = &image;
        data.inputScale = 600.0f;
        CHECK(terrain.prepare(data));
        CHECK(near(*terrain.getHeightData(7, 32), 600.0f));
        CHECK(near(*terrain.getHeightData(32, 32), 600.0f));
        CHECK(near(*terrain.getHeightData(7, 31), 0.0f));
        CHECK(near(*terrain.getHeightData(7, 0), 0.0f));
        CHECK(near(*terrain.getHeightData(5, 32 - 16), 600.0f * 51.0f / 255.0f, 1e-2f));
    }

    {
        ct::Vector<std::uint16_t> source;
        source.resize(33u * 33u);
        for (std::size_t i = 0; i < source.size(); ++i) source[i] = 0;
        source[0] = 65535;
        source[32 * 33 + 3] = 32768;
        Terrain terrain;
        TerrainImportData data = importData(33, 5, 33, 32.0f);
        data.inputImage16 = source.data();
        data.inputImage16Width = 33;
        data.inputImage16Height = 33;
        CHECK(terrain.prepare(data));
        CHECK(near(*terrain.getHeightData(0, 32), 1.0f));
        CHECK(near(*terrain.getHeightData(3, 0), 32768.0f / 65535.0f));
        CHECK(near(*terrain.getHeightData(1, 1), 0.0f));
    }

    {
        Pixmap small(17, 17, 1);
        for (int y = 0; y < 17; ++y)
            for (int x = 0; x < 17; ++x)
                small.pixels[y * 17 + x] = static_cast<unsigned char>(x * 15);
        Terrain terrain;
        TerrainImportData data = importData(33, 5, 33, 32.0f);
        data.inputImage = &small;
        CHECK(terrain.prepare(data));
        CHECK(near(*terrain.getHeightData(0, 5), 0.0f));
        CHECK(near(*terrain.getHeightData(32, 5), 240.0f / 255.0f, 1e-3f));
        CHECK(near(*terrain.getHeightData(16, 5), 120.0f / 255.0f, 1e-3f));
        CHECK(near(*terrain.getHeightData(17, 5), 127.5f / 255.0f, 1e-3f));
    }
}

#if defined(__GLIBC__)
std::size_t liveBytes()
{
    const struct mallinfo2 info = mallinfo2();
    return info.uordblks + info.hblkhd;
}
#else
std::size_t liveBytes()
{
    return 0;
}
#endif

void ownershipRound(int round)
{
    Terrain terrain;
    CHECK(buildShape(terrain, Shape::Pyramid, 513, 33, 65, 12000.0f));
    TerrainLodView view;
    view.position = Math::Vec3(0.0f, 300.0f, 0.0f);
    terrain.setMaxPixelError(4.0f);
    terrain.calculateCurrentLod(view);
    terrain.freeCpuVertexData();
    if (round == 1) terrain.unprepare();
}

void testOwnership()
{
    ownershipRound(0);
    ownershipRound(1);
    ownershipRound(2);
    ownershipRound(3);
    const std::size_t before = liveBytes();
    for (int round = 0; round < 9; ++round) ownershipRound(round);
    CHECK(liveBytes() == before);
    printf("  live bytes before %zu, after %zu\n", before, liveBytes());
}

void measureDemoTerrain()
{
    const ct::String path = ct::String(PRISMA3D_ASSETS_DIR) + "/textures/terrain/terrain.png";
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    Pixmap image;
    CHECK(!bytes.empty());
    CHECK(image.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size())));
    CHECK(image.width == 513 && image.height == 513);
    if (!image.is_valid()) return;

    TerrainImportData data = importData(513, 33, 65, 12000.0f);
    data.inputImage = &image;
    data.inputScale = 600.0f;
    data.maxPixelError = 8.0f;
    data.pos = Math::Vec3(1000.0f, 0.0f, 5000.0f);

    Terrain terrain;
    terrain._setMorphRequired(true);
    const double start = nowMilliseconds();
    CHECK(terrain.prepare(data));
    terrain.load();
    const double built = nowMilliseconds() - start;
    printf("  demo terrain: prepare + load %.1f ms\n", built);
    printf("  height range %.2f .. %.2f, deltas root level %.2f\n", terrain.getMinHeight(),
            terrain.getMaxHeight(), terrain.getQuadTree()->getLodLevel(0)->maxHeightDelta);

    struct Position
    {
        const char* name;
        Math::Vec3 eye;
        Math::Vec3 target;
    };
    const Position positions[] = {
        { "high above", Math::Vec3(1000.0f, 4000.0f, 5000.0f), Math::Vec3(1000.0f, 0.0f, 4800.0f) },
        { "mid height", Math::Vec3(1000.0f, 900.0f, 7000.0f),
            Math::Vec3(1000.0f, 100.0f, 4000.0f) },
        { "ground", Math::Vec3(-2000.0f, 300.0f, 9000.0f), Math::Vec3(0.0f, 100.0f, 5000.0f) },
        { "from the edge", Math::Vec3(1000.0f, 400.0f, 11500.0f),
            Math::Vec3(1000.0f, 200.0f, 5000.0f) },
    };
    const float fovY = 60.0f * Math::DEG2RAD;
    const Math::Mat4 projection = Math::Mat4::Perspective(fovY, 16.0f / 9.0f, 0.5f, 20000.0f);
    for (int p = 0; p < 4; ++p)
    {
        const Math::Frustum frustum = Math::Frustum::FromViewProjection(
                projection * Math::Mat4::LookAt(positions[p].eye, positions[p].target,
                                     Math::Vec3(0.0f, 1.0f, 0.0f)));
        TerrainLodView view;
        view.position = positions[p].eye;
        view.frustum = &frustum;
        view.fovY = fovY;
        view.viewportHeight = 720.0f;

        const int iterations = 2000;
        const double t0 = nowMilliseconds();
        for (int i = 0; i < iterations; ++i) terrain.calculateCurrentLod(view);
        const double perCall = (nowMilliseconds() - t0) * 1000.0 / iterations;

        int perLod[8] = {};
        int nodes = 0;
        std::size_t triangles = 0;
        struct Walk
        {
            static void run(const TerrainQuadTreeNode* node, int* perLod, int& nodes,
                    std::size_t& triangles)
            {
                if (node->isRenderedAtCurrentLod())
                {
                    const int lod = node->getCurrentLod();
                    ++perLod[lod + node->getBaseLod()];
                    ++nodes;
                    triangles +=
                            Terrain::getNumIndexesForBatchSize(node->getLodLevel(lod)->batchSize) -
                            2;
                }
                if (!node->isLeaf())
                    for (unsigned short i = 0; i < 4; ++i)
                        run(node->getChild(i), perLod, nodes, triangles);
            }
        };
        Walk::run(terrain.getQuadTree(), perLod, nodes, triangles);
        printf("  %-14s nodes/draws %2d, strip triangles %6zu, per LOD [0..4] %d %d %d %d %d, LOD "
               "selection %.1f us\n",
                positions[p].name, nodes, triangles, perLod[0], perLod[1], perLod[2], perLod[3],
                perLod[4], perCall);
        CHECK(nodes > 0);
    }
}

} // namespace

int main()
{
    printf("terrain test\n");
    testShapes();
    testQueries();
    testRays();
    testDeltas();
    testLodSelection();
    testLodCulling();
    testBounds();
    testVertexData();
    testSkirtSize();
    testImport();
    testOwnership();
    measureDemoTerrain();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("terrain ok\n");
    return 0;
}
