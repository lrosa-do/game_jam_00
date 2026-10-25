#include "Check.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/terrain/Terrain.h"
#include "prisma3d/terrain/TerrainBrush.h"
#include "prisma3d/terrain/TerrainGeometryTarget.h"
#include "prisma3d/terrain/TerrainLayerBlendMap.h"
#include "prisma3d/terrain/TerrainMapTarget.h"
#include "prisma3d/terrain/TerrainQuadTreeNode.h"

#include <ct/vector.hpp>

#include <math.h>
#include <string.h>
#include <time.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

double nowMilliseconds()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0 + static_cast<double>(ts.tv_nsec) / 1.0e6;
}

class Mirror : public TerrainGeometryTarget
{
public:
    struct Entry
    {
        int depth = 0;
        int x = 0;
        int y = 0;
        TerrainVertexData data;
    };

    ct::Vector<Entry> entries;
    std::uint64_t vertices = 0;
    std::uint32_t ranges = 0;
    ct::Vector<int> touchedOwners;
    ct::Vector<TerrainRect> updates;

    void capture(const TerrainQuadTreeNode* node)
    {
        if (node->ownsVertexData())
        {
            Entry entry;
            entry.depth = node->getDepth();
            entry.x = node->getXOffset();
            entry.y = node->getYOffset();
            entry.data = *node->getVertexDataRecord();
            entries.push_back(entry);
        }
        if (!node->isLeaf())
            for (unsigned short i = 0; i < 4; ++i) capture(node->getChild(i));
    }

    Entry* find(const TerrainQuadTreeNode* node)
    {
        for (std::size_t i = 0; i < entries.size(); ++i)
            if (entries[i].depth == node->getDepth() && entries[i].x == node->getXOffset() &&
                    entries[i].y == node->getYOffset())
                return &entries[i];
        return nullptr;
    }

    void resetCounters()
    {
        vertices = 0;
        ranges = 0;
        touchedOwners.clear();
        updates.clear();
    }

    void writeVertexRange(const TerrainQuadTreeNode* owner, std::uint32_t first,
            std::uint32_t count, const TerrainVertexData& source) override
    {
        Entry* entry = find(owner);
        CHECK(entry != nullptr);
        if (!entry) return;
        for (std::uint32_t i = 0; i < count; ++i)
        {
            entry->data.positions[first + i] = source.positions[first + i];
            entry->data.uvs[first + i] = source.uvs[first + i];
            entry->data.deltas[first + i] = source.deltas[first + i];
        }
        vertices += count;
        ++ranges;
        const int id = static_cast<int>(entry - entries.data());
        bool known = false;
        for (std::size_t i = 0; i < touchedOwners.size(); ++i)
            if (touchedOwners[i] == id) known = true;
        if (!known) touchedOwners.push_back(id);
    }

    void geometryUpdated(const TerrainRect& rect) override
    {
        updates.push_back(rect);
    }
};

class Maps : public TerrainMapTarget
{
public:
    TerrainPixelBox normal;
    TerrainPixelBox light;
    std::uint64_t normalBytes = 0;
    std::uint64_t lightBytes = 0;

    bool createNormalMap(std::uint16_t size) override
    {
        normal.create(size, size, 3);
        return true;
    }
    void destroyNormalMap() override
    {
    }
    void writeNormalMap(const TerrainRect& box, const TerrainPixelBox& rgb) override
    {
        copy(normal, box, rgb);
        normalBytes += rgb.data.size();
    }
    bool createLightmap(std::uint16_t size) override
    {
        light.create(size, size, 1);
        return true;
    }
    void destroyLightmap() override
    {
    }
    void writeLightmap(const TerrainRect& box, const TerrainPixelBox& l8) override
    {
        copy(light, box, l8);
        lightBytes += l8.data.size();
    }
    bool createColourMap(std::uint16_t) override
    {
        return true;
    }
    void destroyColourMap() override
    {
    }
    void writeColourMap(const TerrainRect&, const TerrainPixelBox&) override
    {
    }
    bool createCompositeMap(std::uint16_t) override
    {
        return true;
    }
    void destroyCompositeMap() override
    {
    }
    void renderCompositeMap(const TerrainRect&) override
    {
    }
    bool createBlendTexture(std::uint8_t index, std::uint16_t size) override
    {
        while (blend.size() <= index) blend.push_back(TerrainPixelBox());
        blend[index].create(size, size, 4);
        return true;
    }
    void destroyBlendTexture(std::uint8_t) override
    {
    }
    void writeBlendTexture(std::uint8_t index, const TerrainRect& box,
            const TerrainPixelBox& rgba) override
    {
        copy(blend[index], box, rgba);
        ++blendWrites;
    }

    ct::Vector<TerrainPixelBox> blend;
    int blendWrites = 0;

private:
    static void copy(TerrainPixelBox& target, const TerrainRect& box, const TerrainPixelBox& src)
    {
        for (int y = 0; y < box.height(); ++y)
            for (int x = 0; x < box.width(); ++x)
                for (std::uint32_t c = 0; c < src.components; ++c)
                    target.at(static_cast<std::uint32_t>(box.left + x),
                            static_cast<std::uint32_t>(box.top + y))[c] =
                            src.at(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y))[c];
    }
};

std::uint32_t gSeed = 12345;

std::uint32_t nextRandom()
{
    gSeed = gSeed * 1664525u + 1013904223u;
    return gSeed >> 8;
}

float randomFloat()
{
    return static_cast<float>(nextRandom() & 0xffff) / 65535.0f;
}

TerrainImportData baseData(std::uint16_t size, std::uint16_t minBatch, std::uint16_t maxBatch,
        float world, const float* heights)
{
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = minBatch;
    data.maxBatchSize = maxBatch;
    data.worldSize = world;
    data.inputFloat = heights;
    data.lightMapSize = size;
    data.layerBlendMapSize = size;
    data.compositeMapSize = 64;
    data.lightMapDirection = Math::Vec3(0.55f, -0.5f, 0.75f).Normalized();
    return data;
}

ct::Vector<float> hills(int size)
{
    ct::Vector<float> heights;
    heights.resize(static_cast<std::size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            heights[static_cast<std::size_t>(y) * size + x] =
                    12.0f + 6.0f * sinf(static_cast<float>(x) * 0.09f) *
                                    cosf(static_cast<float>(y) * 0.07f);
    return heights;
}

struct Setup
{
    Terrain terrain;
    Mirror mirror;
    Maps maps;
};

bool open(Setup& setup, const TerrainImportData& data, bool withMaps)
{
    setup.terrain._setMorphRequired(true);
    setup.terrain.setGeometryTarget(&setup.mirror);
    if (withMaps) setup.terrain.setMapTarget(&setup.maps);
    if (!setup.terrain.prepare(data)) return false;
    setup.mirror.capture(setup.terrain.getQuadTree());
    setup.terrain.freeCpuVertexData();
    setup.terrain.load();
    if (withMaps) setup.terrain.loadMaps(true, true);
    setup.mirror.resetCounters();
    return true;
}

bool sameVertices(const TerrainVertexData& a, const TerrainVertexData& b)
{
    if (a.positions.size() != b.positions.size() || a.uvs.size() != b.uvs.size() ||
            a.deltas.size() != b.deltas.size())
        return false;
    for (std::size_t i = 0; i < a.positions.size(); ++i)
    {
        if (a.positions[i].x != b.positions[i].x || a.positions[i].y != b.positions[i].y ||
                a.positions[i].z != b.positions[i].z)
            return false;
        if (a.uvs[i].x != b.uvs[i].x || a.uvs[i].y != b.uvs[i].y) return false;
        if (a.deltas[i].x != b.deltas[i].x || a.deltas[i].y != b.deltas[i].y) return false;
    }
    return true;
}

struct Comparison
{
    int vertexOwners = 0;
    int vertexMismatches = 0;
    int boxMismatches = 0;
    int radiusMismatches = 0;
    int deltaMismatches = 0;
    int levelDifferent = 0;
    int normalMismatches = 0;
    int lightMismatches = 0;
};

void compareNodes(const TerrainQuadTreeNode* edited, const TerrainQuadTreeNode* fresh,
        Mirror& mirror, Comparison& out)
{
    if (fresh->ownsVertexData())
    {
        Mirror::Entry* entry = mirror.find(fresh);
        ++out.vertexOwners;
        if (!entry || !sameVertices(entry->data, *fresh->getVertexDataRecord()))
            ++out.vertexMismatches;
    }
    const Math::Box& a = edited->getBoundingBox();
    const Math::Box& b = fresh->getBoundingBox();
    if (a.min.x != b.min.x || a.min.y != b.min.y || a.min.z != b.min.z || a.max.x != b.max.x ||
            a.max.y != b.max.y || a.max.z != b.max.z)
        ++out.boxMismatches;
    if (edited->getBoundingRadius() != fresh->getBoundingRadius()) ++out.radiusMismatches;
    for (std::uint16_t lod = 0; lod < fresh->getLodCount(); ++lod)
        if (edited->getLodLevel(lod)->maxHeightDelta != fresh->getLodLevel(lod)->maxHeightDelta)
            ++out.levelDifferent;
    if (!fresh->isLeaf())
        for (unsigned short i = 0; i < 4; ++i)
            compareNodes(edited->getChild(i), fresh->getChild(i), mirror, out);
}

Comparison compareWithFresh(Setup& edited, const TerrainImportData& data)
{
    const int size = edited.terrain.getSize();
    ct::Vector<float> heights;
    heights.resize(static_cast<std::size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            heights[static_cast<std::size_t>(y) * size + x] = edited.terrain.getHeightAtPoint(
                    static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
    TerrainImportData freshData = data;
    freshData.inputFloat = heights.data();
    freshData.inputImage = nullptr;

    Terrain fresh;
    Maps freshMaps;
    fresh._setMorphRequired(true);
    fresh.setMapTarget(&freshMaps);
    CHECK(fresh.prepare(freshData));
    fresh.load();
    fresh.loadMaps(true, true);

    Comparison out;
    compareNodes(edited.terrain.getQuadTree(), fresh.getQuadTree(), edited.mirror, out);
    for (int i = 0; i < size * size; ++i)
        if (edited.terrain.getDeltaData()[i] != fresh.getDeltaData()[i]) ++out.deltaMismatches;
    if (edited.maps.normal.data.size() == freshMaps.normal.data.size())
        for (std::size_t i = 0; i < freshMaps.normal.data.size(); ++i)
            if (freshMaps.normal.data[i] != edited.maps.normal.data[i]) ++out.normalMismatches;
    if (edited.maps.light.data.size() == freshMaps.light.data.size())
        for (std::size_t i = 0; i < freshMaps.light.data.size(); ++i)
            if (freshMaps.light.data[i] != edited.maps.light.data[i]) ++out.lightMismatches;
    return out;
}

void testSingleEdit()
{
    ct::Vector<float> heights = hills(129);
    Setup s;
    const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
    CHECK(open(s, data, true));
    CHECK(s.mirror.entries.size() == 1);

    const float before = s.terrain.getHeightAtPoint(50, 60);
    s.terrain.setHeightAtPoint(50, 60, before + 30.0f);
    CHECK(near(s.terrain.getHeightAtPoint(50, 60), before + 30.0f));
    CHECK(near(s.terrain.getHeightAtTerrainPosition(50.0f / 128.0f, 60.0f / 128.0f), before + 30.0f,
            1e-3f));

    s.terrain.updateGeometry();
    CHECK(s.mirror.vertices == 1 && s.mirror.ranges == 1 && s.mirror.touchedOwners.size() == 1);
    CHECK(s.mirror.updates.size() == 1 && s.mirror.updates[0].left == 50 &&
            s.mirror.updates[0].right == 51 && s.mirror.updates[0].top == 60 &&
            s.mirror.updates[0].bottom == 61);
    const TerrainVertexData& data0 = s.mirror.entries[0].data;
    const TerrainQuadTreeNode* root = s.terrain.getQuadTree();
    const std::size_t vertex =
            static_cast<std::size_t>(60) * root->getVertexDataRecord()->size + 50;
    CHECK(near(data0.positions[vertex].y, before + 30.0f));

    s.terrain.updateDerivedData();
    CHECK(s.maps.normalBytes > 0);
    const Math::Vec3 inWorld = Math::Vec3(50.0f - 64.0f, 200.0f, 64.0f - 60.0f);
    const TerrainRayResult hit = s.terrain.rayIntersects(Math::Ray(inWorld, Math::Vec3(0, -1, 0)));
    CHECK(hit.hit && near(hit.position.y, before + 30.0f, 1e-3f) &&
            near(hit.position.x, inWorld.x, 1e-3f));
    CHECK(s.terrain.getMaxHeight() >= before + 30.0f - 1e-3f);

    Comparison c = compareWithFresh(s, data);
    printf("  one point raised: %d owners, %d vertex mismatches, %d delta mismatches, %d normal "
           "mismatches, %d light mismatches, %d boxes differ\n",
            c.vertexOwners, c.vertexMismatches, c.deltaMismatches, c.normalMismatches,
            c.lightMismatches, c.boxMismatches);
    CHECK(c.vertexMismatches == 0 && c.deltaMismatches == 0 && c.normalMismatches == 0);
    CHECK(c.lightMismatches == 0 && c.boxMismatches == 0 && c.radiusMismatches == 0);
    CHECK(c.levelDifferent == 0);
}

void testEdgeAndSkirt()
{
    ct::Vector<float> heights = hills(129);
    Setup s;
    const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
    CHECK(open(s, data, false));
    s.terrain.setHeightAtPoint(40, 0, 77.0f);
    s.terrain.update();
    const TerrainQuadTreeNode* root = s.terrain.getQuadTree();
    const TerrainVertexData* record = root->getVertexDataRecord();
    const std::size_t main = 0 * static_cast<std::size_t>(record->size) + 40;
    const std::uint16_t skirt = Terrain::calcSkirtVertexIndex(static_cast<std::uint16_t>(main),
            record->size, false, record->numSkirtRowsCols, record->skirtRowColSkip);
    const TerrainVertexData& live = s.mirror.entries[0].data;
    CHECK(near(live.positions[main].y, 77.0f));
    CHECK(near(live.positions[skirt].y, 77.0f - 30.0f));
    CHECK(near(live.positions[skirt].x, live.positions[main].x) &&
            near(live.positions[skirt].z, live.positions[main].z));
    CHECK(live.deltas[skirt].x == 0.0f && live.deltas[skirt].y == 99.0f);

    s.terrain.setHeightAtPoint(128, 70, 55.0f);
    s.terrain.update();
    const std::size_t mainColumn = static_cast<std::size_t>(70) * record->size + 128;
    const std::uint16_t skirtColumn =
            Terrain::calcSkirtVertexIndex(static_cast<std::uint16_t>(mainColumn), record->size,
                    true, record->numSkirtRowsCols, record->skirtRowColSkip);
    CHECK(near(live.positions[mainColumn].y, 55.0f));
    CHECK(near(live.positions[skirtColumn].y, 55.0f - 30.0f));
    Comparison c = compareWithFresh(s, data);
    CHECK(c.vertexMismatches == 0 && c.deltaMismatches == 0);
}

void testRandomEdits()
{
    struct Pass
    {
        std::uint16_t size;
        float low;
        float high;
        int rounds;
        const char* name;
    };
    const Pass passes[6] = {
        { 129, 6.5f, 17.5f, 12, "inside the height range" },
        { 257, 6.5f, 17.5f, 6, "inside the height range" },
        { 129, 0.0f, 40.0f, 12, "outside the height range" },
        { 257, 0.0f, 40.0f, 6, "outside the height range" },
        { 129, -300.0f, 300.0f, 12, "far outside the height range" },
        { 257, -300.0f, 300.0f, 4, "far outside the height range" },
    };
    for (int pass = 0; pass < 6; ++pass)
    {
        const int size = passes[pass].size;
        const float low = passes[pass].low;
        const float high = passes[pass].high;
        ct::Vector<float> heights = hills(size);
        heights[0] = 6.0f;
        heights[heights.size() - 1] = 18.0f;
        Setup s;
        const TerrainImportData data = baseData(static_cast<std::uint16_t>(size), 17, 33,
                static_cast<float>(size - 1), heights.data());
        CHECK(open(s, data, true));
        CHECK(!s.mirror.entries.empty());
        gSeed = 777u + static_cast<std::uint32_t>(pass);

        int comparisons = 0;
        int mismatches = 0;
        int boxes = 0;
        int radii = 0;
        int levels = 0;
        int lightDifferences = 0;
        for (int round = 0; round < passes[pass].rounds; ++round)
        {
            const int edits = 1 + static_cast<int>(nextRandom() % 40);
            for (int e = 0; e < edits; ++e)
            {
                const std::uint32_t x = 1 + nextRandom() % static_cast<std::uint32_t>(size - 2);
                const std::uint32_t y = 1 + nextRandom() % static_cast<std::uint32_t>(size - 2);
                s.terrain.setHeightAtPoint(x, y, low + randomFloat() * (high - low));
            }
            if (round % 4 == 3)
                for (int e = 0; e < 200; ++e)
                {
                    const std::uint32_t x = 20 + nextRandom() % 30u;
                    const std::uint32_t y = 30 + nextRandom() % 30u;
                    s.terrain.setHeightAtPoint(x, y, low + randomFloat() * (high - low) * 0.1f);
                }
            s.terrain.update();
            const Comparison c = compareWithFresh(s, data);
            ++comparisons;
            mismatches +=
                    c.vertexMismatches + c.deltaMismatches + c.normalMismatches + c.lightMismatches;
            lightDifferences += c.lightMismatches;
            boxes += c.boxMismatches;
            radii += c.radiusMismatches;
            levels += c.levelDifferent;
            if (c.vertexMismatches || c.deltaMismatches || c.normalMismatches ||
                    c.lightMismatches || c.boxMismatches || c.radiusMismatches || c.levelDifferent)
                printf("    round %d: vertices %d, deltas %d, normals %d, lights %d, boxes %d, "
                       "radii %d, levels %d\n",
                        round, c.vertexMismatches, c.deltaMismatches, c.normalMismatches,
                        c.lightMismatches, c.boxMismatches, c.radiusMismatches, c.levelDifferent);
        }
        printf("  %dx%d random edits %s against a fresh build: %d comparisons, %d mismatches in "
               "vertices, deltas, normal map and lightmap (%d lightmap texels), %d node boxes, "
               "%d radii and %d height-delta levels different\n",
                size, size, passes[pass].name, comparisons, mismatches, lightDifferences, boxes,
                radii, levels);
        CHECK(mismatches == 0 && boxes == 0 && radii == 0 && levels == 0);
    }
}

void testOwnerGroups()
{
    const int size = 513;
    ct::Vector<float> heights = hills(size);
    Setup s;
    const TerrainImportData data = baseData(static_cast<std::uint16_t>(size), 33, 65,
            static_cast<float>(size - 1), heights.data());
    CHECK(open(s, data, true));
    gSeed = 4242u;
    int boxes = 0;
    int radii = 0;
    int levels = 0;
    int lights = 0;
    int others = 0;
    for (int round = 0; round < 4; ++round)
    {
        const int edits = 5 + static_cast<int>(nextRandom() % 30);
        for (int e = 0; e < edits; ++e)
        {
            const std::uint32_t x = static_cast<std::uint32_t>(nextRandom() % size);
            const std::uint32_t y = static_cast<std::uint32_t>(nextRandom() % size);
            const float value =
                    round == 3 ? -60.0f + randomFloat() * 120.0f : 6.5f + randomFloat() * 11.0f;
            s.terrain.setHeightAtPoint(x, y, value);
        }
        s.terrain.setHeightAtPoint(512, 7, 14.0f);
        s.terrain.setHeightAtPoint(9, 512, 3.0f);
        s.terrain.update();
        const Comparison c = compareWithFresh(s, data);
        boxes += c.boxMismatches;
        radii += c.radiusMismatches;
        levels += c.levelDifferent;
        lights += c.lightMismatches;
        others += c.vertexMismatches + c.deltaMismatches + c.normalMismatches;
    }
    printf("  513x513 random edits against a fresh build (several owner groups): %d vertex, "
           "delta or normal mismatches, %d lightmap texels, %d boxes, %d radii and %d "
           "height-delta levels different\n",
            others, lights, boxes, radii, levels);
    CHECK(others == 0 && lights == 0 && boxes == 0 && radii == 0 && levels == 0);
}

void testWholeRectangleUpdate()
{
    ct::Vector<float> heights = hills(129);
    Setup s;
    const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
    CHECK(open(s, data, true));
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x)
            *s.terrain.getHeightData(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y)) =
                    5.0f + static_cast<float>((x * 7 + y * 3) % 11);
    s.terrain.dirty();
    s.terrain.update();
    const Comparison c = compareWithFresh(s, data);
    printf("  whole terrain redone through dirty and update: %d vertex, %d delta, %d normal, %d "
           "light mismatches, %d boxes differ, %d levels differ\n",
            c.vertexMismatches, c.deltaMismatches, c.normalMismatches, c.lightMismatches,
            c.boxMismatches, c.levelDifferent);
    CHECK(c.vertexMismatches == 0 && c.deltaMismatches == 0 && c.normalMismatches == 0);
    CHECK(c.lightMismatches == 0 && c.boxMismatches == 0 && c.radiusMismatches == 0 &&
            c.levelDifferent == 0);
}

void testNodeBoxesAndLevels()
{
    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = 0.0f;
    Setup s;
    const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
    CHECK(open(s, data, false));
    const TerrainQuadTreeNode* root = s.terrain.getQuadTree();
    CHECK(near(root->getMaxHeight(), 0.0f) && near(root->getLodLevel(0)->maxHeightDelta, 0.0f));

    s.terrain.setHeightAtPoint(64, 64, 8.0f);
    s.terrain.update();
    CHECK(near(root->getMaxHeight(), 8.0f) && near(root->getMinHeight(), 0.0f));
    CHECK(near(s.terrain.getDeltaData(64, 64)[0], 0.0f));
    CHECK(near(s.terrain.getDeltaData(65, 64)[0], 4.0f));
    CHECK(near(s.terrain.getDeltaData(63, 64)[0], 4.0f));
    CHECK(near(s.terrain.getDeltaData(64, 65)[0], 4.0f));
    CHECK(near(s.terrain.getDeltaData(64, 63)[0], 4.0f));
    float highest = 0.0f;
    for (unsigned short i = 0; i < 4; ++i)
    {
        const TerrainQuadTreeNode* child = root->getChild(i);
        const bool holds = 64 >= child->getXOffset() &&
                           64 < child->getXOffset() + child->getSize() &&
                           64 >= child->getYOffset() && 64 < child->getYOffset() + child->getSize();
        CHECK(holds == (child->getMaxHeight() > 7.9f) || child->getMaxHeight() > 7.9f);
        for (std::uint16_t lod = 0; lod < child->getLodCount(); ++lod)
            highest = highest > child->getLodLevel(lod)->maxHeightDelta
                              ? highest
                              : child->getLodLevel(lod)->maxHeightDelta;
    }
    CHECK(highest > 0.0f);
    CHECK(near(root->getLodLevel(0)->maxHeightDelta, highest * 1.05f, 1e-3f));
    CHECK(root->getLodLevel(0)->maxHeightDelta > 0.0f);

    s.terrain.setHeightAtPoint(64, 64, 0.0f);
    s.terrain.update();
    CHECK(near(root->getMaxHeight(), 0.0f) && near(root->getMinHeight(), 0.0f));
    CHECK(near(s.terrain.getDeltaData(65, 64)[0], 0.0f));
    for (unsigned short i = 0; i < 4; ++i) CHECK(near(root->getChild(i)->getMaxHeight(), 0.0f));
    CHECK(root->getLodLevel(0)->maxHeightDelta == 0.0f);
    for (unsigned short i = 0; i < 4; ++i)
        for (std::uint16_t lod = 0; lod < root->getChild(i)->getLodCount(); ++lod)
            CHECK(root->getChild(i)->getLodLevel(lod)->maxHeightDelta == 0.0f);
    CHECK(near(root->getBoundingRadius(), sqrtf(64.0f * 64.0f * 2.0f), 0.05f));

    const std::uint32_t edges[4][2] = { { 128, 70 }, { 70, 128 }, { 128, 128 }, { 0, 0 } };
    for (int e = 0; e < 4; ++e)
    {
        s.terrain.setHeightAtPoint(edges[e][0], edges[e][1], 9.0f);
        s.terrain.update();
        CHECK(near(root->getMaxHeight(), 9.0f));
        const TerrainQuadTreeNode* holder = root;
        while (!holder->isLeaf())
        {
            const TerrainQuadTreeNode* next = nullptr;
            for (unsigned short i = 0; i < 4; ++i)
            {
                const TerrainQuadTreeNode* child = holder->getChild(i);
                if (edges[e][0] >= child->getXOffset() &&
                        edges[e][0] < child->getXOffset() + child->getSize() &&
                        edges[e][1] >= child->getYOffset() &&
                        edges[e][1] < child->getYOffset() + child->getSize())
                {
                    next = child;
                    break;
                }
            }
            CHECK(next != nullptr);
            if (!next) break;
            CHECK(near(next->getMaxHeight(), 9.0f));
            holder = next;
        }
        s.terrain.setHeightAtPoint(edges[e][0], edges[e][1], 0.0f);
        s.terrain.update();
        CHECK(near(root->getMaxHeight(), 0.0f));
        CHECK(root->getLodLevel(0)->maxHeightDelta == 0.0f);
        for (unsigned short i = 0; i < 4; ++i) CHECK(near(root->getChild(i)->getMaxHeight(), 0.0f));
    }

    s.terrain.setHeightAtPoint(64, 64, 8.0f);
    s.terrain.update();
    s.terrain.setHeightAtPoint(64, 64, 0.0f);
    s.terrain.dirty();
    s.terrain.update();
    CHECK(near(root->getMaxHeight(), 0.0f));
    CHECK(near(root->getChild(3)->getMaxHeight(), 0.0f));
    const Comparison c = compareWithFresh(s, data);
    CHECK(c.boxMismatches == 0 && c.radiusMismatches == 0 && c.levelDifferent == 0 &&
            c.lightMismatches == 0);
}

void testFourNodes()
{
    ct::Vector<float> heights;
    heights.resize(513u * 513u);
    for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = 0.0f;
    Setup s;
    TerrainImportData data = baseData(513, 33, 65, 512.0f, heights.data());
    CHECK(open(s, data, false));
    CHECK(s.mirror.entries.size() == 17);

    s.terrain.setHeightAtPoint(128, 128, 9.0f);
    s.terrain.updateGeometry();
    printf("  corner shared by four nodes: %zu owners written, %llu vertices in %u ranges\n",
            s.mirror.touchedOwners.size(), static_cast<unsigned long long>(s.mirror.vertices),
            s.mirror.ranges);
    CHECK(s.mirror.touchedOwners.size() == 5);
    int depthTwo = 0;
    int depthZero = 0;
    for (std::size_t i = 0; i < s.mirror.touchedOwners.size(); ++i)
    {
        const Mirror::Entry& entry =
                s.mirror.entries[static_cast<std::size_t>(s.mirror.touchedOwners[i])];
        if (entry.depth == 2) ++depthTwo;
        if (entry.depth == 0) ++depthZero;
    }
    CHECK(depthTwo == 4 && depthZero == 1);

    s.mirror.resetCounters();
    s.terrain.setHeightAtPoint(70, 70, 9.0f);
    s.terrain.updateGeometry();
    CHECK(s.mirror.touchedOwners.size() == 1 && s.mirror.vertices == 1);

    s.mirror.resetCounters();
    s.terrain.setHeightAtPoint(127, 130, 3.0f);
    s.terrain.updateGeometry();
    CHECK(s.mirror.touchedOwners.size() == 1);

    const Comparison c = compareWithFresh(s, data);
    s.terrain.update();
    const Comparison d = compareWithFresh(s, data);
    CHECK(d.vertexMismatches == 0 && d.deltaMismatches == 0);
    (void) c;
}

double weightOf(double distance, double radius)
{
    double w = distance / radius;
    w = w < 1.0 ? w : 1.0;
    return 1.0 - w * w;
}

double smoothOf(double t)
{
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    return t * t * (3.0 - 2.0 * t);
}

void testBrushes()
{
    CHECK(near(terrainBrushWeight(0.0f, 16.0f), 1.0f));
    CHECK(near(terrainBrushWeight(8.0f, 16.0f), 0.75f));
    CHECK(near(terrainBrushWeight(16.0f, 16.0f), 0.0f));
    CHECK(near(terrainBrushWeight(40.0f, 16.0f), 0.0f));

    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = 10.0f;
    {
        Setup s;
        const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
        CHECK(open(s, data, false));
        CHECK(terrainBrushHeight(s.terrain, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 250.0f, 0.1f,
                false));
        CHECK(near(s.terrain.getHeightAtPoint(64, 64), 35.0f, 1e-4f));
        CHECK(near(s.terrain.getHeightAtPoint(72, 64), 10.0f + 18.75f, 1e-4f));
        CHECK(near(s.terrain.getHeightAtPoint(64, 56), 10.0f + 18.75f, 1e-4f));
        CHECK(near(s.terrain.getHeightAtPoint(80, 64), 10.0f));
        CHECK(near(s.terrain.getHeightAtPoint(48, 64), 10.0f));
        CHECK(near(s.terrain.getHeightAtPoint(64, 100), 10.0f));
        int changed = 0;
        for (int y = 0; y < 129; ++y)
            for (int x = 0; x < 129; ++x)
            {
                const double d =
                        sqrt(static_cast<double>((x - 64) * (x - 64) + (y - 64) * (y - 64)));
                const bool inside = d < 16.0;
                const float h = s.terrain.getHeightAtPoint(static_cast<std::uint32_t>(x),
                        static_cast<std::uint32_t>(y));
                if (inside)
                {
                    CHECK(fabs(h - (10.0 + weightOf(d, 16.0) * 250.0 * 0.1)) < 2e-4);
                    ++changed;
                }
                else
                    CHECK(h == 10.0f);
            }
        printf("  height brush radius 16: %d points raised, centre %.3f, half radius %.3f\n",
                changed, s.terrain.getHeightAtPoint(64, 64), s.terrain.getHeightAtPoint(72, 64));
        CHECK(terrainBrushHeight(s.terrain, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 250.0f, 0.1f,
                true));
        CHECK(near(s.terrain.getHeightAtPoint(64, 64), 10.0f, 1e-4f));
        CHECK(near(s.terrain.getHeightAtPoint(72, 64), 10.0f, 1e-4f));
        CHECK(!terrainBrushHeight(s.terrain, Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, 250.0f, 0.1f,
                true));
    }
    {
        Setup s;
        const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
        CHECK(open(s, data, true));
        s.terrain.addLayer(10.0f, nullptr);
        s.terrain.addLayer(10.0f, nullptr);
        TerrainLayerBlendMap* map = s.terrain.getLayerBlendMap(1);
        CHECK(map != nullptr);
        s.maps.blendWrites = 0;
        CHECK(terrainBrushBlend(s.terrain, 1, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 1.0f, 0.5f,
                false));
        CHECK(s.maps.blendWrites == 1 && !map->isDirty());
        int touched = 0;
        int expectedTouched = 0;
        int wrong = 0;
        for (int y = 0; y < 129; ++y)
        {
            for (int x = 0; x < 129; ++x)
            {
                const double tx = static_cast<double>(x) / 129.0 - 0.5;
                const double ty = static_cast<double>(y) / 129.0 - 0.5;
                const double w = weightOf(sqrt(tx * tx + ty * ty), 0.125);
                const int imgY = 129 - y;
                const bool expected =
                        w > 0.0 && imgY < 129 && x >= 32 && y >= 32 && x <= 96 && y <= 96;
                if (expected) ++expectedTouched;
                if (expected)
                {
                    const float v = map->getBlendValue(static_cast<std::uint32_t>(x),
                            static_cast<std::uint32_t>(imgY));
                    if (fabs(v - w * 0.5) > 1e-5) ++wrong;
                }
            }
        }
        for (int y = 0; y < 129; ++y)
            for (int x = 0; x < 129; ++x)
                if (map->getBlendValue(static_cast<std::uint32_t>(x),
                            static_cast<std::uint32_t>(y)) > 0.0f)
                    ++touched;
        printf("  blend brush radius 16 on 129x129: %d texels painted, %d expected, %d wrong "
               "values, "
               "centre %.4f\n",
                touched, expectedTouched, wrong, map->getBlendValue(64, 65));
        CHECK(touched == expectedTouched && wrong == 0);
        CHECK(near(map->getBlendValue(64, 65),
                0.5f * static_cast<float>(weightOf(sqrt((64.0 / 129 - 0.5) * (64.0 / 129 - 0.5) +
                                                           (64.0 / 129 - 0.5) * (64.0 / 129 - 0.5)),
                               0.125)),
                1e-5f));
        CHECK(terrainBrushBlend(s.terrain, 1, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 1.0f, 5.0f,
                true));
        CHECK(near(map->getBlendValue(64, 65), 0.0f));
        for (int k = 0; k < 4; ++k)
            terrainBrushBlend(s.terrain, 1, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 1.0f, 5.0f, false);
        CHECK(near(map->getBlendValue(64, 65), 1.0f));
        CHECK(!terrainBrushBlend(s.terrain, 0, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 1.0f, 0.5f,
                false));
    }
    {
        Setup s;
        ct::Vector<float> plane = heights;
        for (std::size_t i = 0; i < plane.size(); ++i) plane[i] = 0.0f;
        plane[64u * 129u + 64u] = 10.0f;
        const TerrainImportData data = baseData(129, 17, 33, 128.0f, plane.data());
        CHECK(open(s, data, false));
        CHECK(terrainBrushSmooth(s.terrain, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 1.0f, 1.0f));
        const double average = 10.0 / 9.0;
        CHECK(fabs(s.terrain.getHeightAtPoint(64, 64) - average) < 1e-5);
        const double falloff = 1.0 - smoothOf(1.0 / 16.0);
        CHECK(fabs(s.terrain.getHeightAtPoint(65, 64) - average * falloff) < 1e-5);
        CHECK(fabs(s.terrain.getHeightAtPoint(64, 63) - average * falloff) < 1e-5);
        CHECK(s.terrain.getHeightAtPoint(64, 70) == 0.0f ||
                fabs(s.terrain.getHeightAtPoint(64, 70)) < 1e-6);
        CHECK(s.terrain.getHeightAtPoint(100, 100) == 0.0f);
        CHECK(!terrainBrushSmooth(s.terrain, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 0.0f, 1.0f));
    }
    {
        Setup s;
        const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
        CHECK(open(s, data, false));
        CHECK(terrainBrushFlatten(s.terrain, Math::Vec3(0.0f, 0.0f, 0.0f), 16.0f, 4.0f, 1.0f,
                1.0f));
        CHECK(near(s.terrain.getHeightAtPoint(64, 64), 4.0f));
        CHECK(near(s.terrain.getHeightAtPoint(72, 64), 7.0f, 1e-4f));
        CHECK(near(s.terrain.getHeightAtPoint(64, 56), 7.0f, 1e-4f));
        CHECK(near(s.terrain.getHeightAtPoint(80, 64), 10.0f));
        CHECK(near(s.terrain.getHeightAtPoint(64, 64 + 15),
                10.0f - 6.0f * (1.0f - static_cast<float>(smoothOf(15.0 / 16.0))), 1e-4f));
    }
}

double pathWeight(double beyond, double bank)
{
    if (beyond <= 0.0) return 1.0;
    if (beyond >= bank) return 0.0;
    const double t = beyond / bank;
    return 1.0 - t * t;
}

void testPathNearest()
{
    TerrainPath path;
    path.add(Math::Vec3(-10.0f, 5.0f, 0.0f), 2.0f);
    path.add(Math::Vec3(10.0f, 9.0f, 0.0f), 4.0f);
    CHECK(path.getSegmentCount() == 1);
    TerrainPathHit hit;
    CHECK(path.nearest(0.0f, 5.0f, hit));
    CHECK(near(hit.distance, 5.0f) && near(hit.halfWidth, 3.0f) && near(hit.clearance, 2.0f));
    CHECK(near(hit.height, 7.0f));
    CHECK(path.nearest(15.0f, 0.0f, hit));
    CHECK(near(hit.distance, 5.0f) && near(hit.halfWidth, 4.0f) && near(hit.height, 9.0f));
    CHECK(path.nearest(-10.0f, -3.0f, hit));
    CHECK(near(hit.distance, 3.0f) && near(hit.clearance, 1.0f));

    TerrainPath square;
    square.add(Math::Vec3(0.0f, 0.0f, 0.0f), 1.0f);
    square.add(Math::Vec3(10.0f, 0.0f, 0.0f), 1.0f);
    square.add(Math::Vec3(10.0f, 0.0f, 10.0f), 1.0f);
    square.add(Math::Vec3(0.0f, 0.0f, 10.0f), 1.0f);
    CHECK(square.getSegmentCount() == 3);
    square.closed = true;
    CHECK(square.getSegmentCount() == 4);
    CHECK(square.nearest(-2.0f, 5.0f, hit));
    CHECK(near(hit.distance, 2.0f) && near(hit.clearance, 1.0f));
    CHECK(square.nearest(5.0f, 5.0f, hit));
    CHECK(near(hit.distance, 5.0f));

    CHECK(near(terrainPathWeight(1.0f, 1.0f, 8.0f), 1.0f));
    CHECK(near(terrainPathWeight(0.5f, 1.0f, 8.0f), 1.0f));
    CHECK(near(terrainPathWeight(3.0f, 1.0f, 8.0f), 0.9375f));
    CHECK(near(terrainPathWeight(5.0f, 1.0f, 8.0f), 0.75f));
    CHECK(near(terrainPathWeight(9.0f, 1.0f, 8.0f), 0.0f));
    CHECK(near(terrainPathWeight(20.0f, 1.0f, 8.0f), 0.0f));
    CHECK(near(terrainPathWeight(1.5f, 1.0f, 0.0f), 0.0f));
}

void testSlope()
{
    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x)
            heights[static_cast<std::size_t>(y) * 129 + x] = x < 64 ? 0.0f : 0.5f * (x - 64);
    Setup s;
    const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
    CHECK(open(s, data, false));
    const float expected = atanf(0.5f) * 180.0f / 3.14159265358979f;
    CHECK(near(s.terrain.getSlopeAtWorldPosition(Math::Vec3(-30.0f, 0.0f, 10.0f)), 0.0f, 1e-3f));
    CHECK(near(s.terrain.getSlopeAtWorldPosition(Math::Vec3(0.5f, 0.0f, 10.3f)), expected, 1e-2f));
    CHECK(near(s.terrain.getSlopeAtWorldPosition(Math::Vec3(30.2f, 0.0f, -20.7f)), expected,
            1e-2f));
    CHECK(near(s.terrain.getSlopeAtWorldPosition(Math::Vec3(-0.5f, 0.0f, 3.0f)), 0.0f, 1e-3f));
}

void testFlattenRoad()
{
    const int size = 257;
    ct::Vector<float> original = hills(size);
    Setup s;
    const TerrainImportData data = baseData(static_cast<std::uint16_t>(size), 17, 33,
            static_cast<float>(size - 1), original.data());
    CHECK(open(s, data, true));

    TerrainPath path;
    path.add(Math::Vec3(-100.0f, 20.0f, 60.0f), 2.0f);
    path.add(Math::Vec3(100.0f, 20.0f, 60.0f), 2.0f);
    TerrainPathBrush brush;
    brush.shoulder = 1.0f;
    brush.bank = 8.0f;

    s.mirror.resetCounters();
    const double t0 = nowMilliseconds();
    const std::size_t written = terrainFlattenAlongPath(s.terrain, path, brush);
    const double flattenMs = nowMilliseconds() - t0;
    const double t1 = nowMilliseconds();
    s.terrain.update();
    const double updateMs = nowMilliseconds() - t1;

    int wrong = 0;
    int inside = 0;
    int unchangedWrong = 0;
    int changed = 0;
    for (int iy = 0; iy < size; ++iy)
    {
        for (int ix = 0; ix < size; ++ix)
        {
            const double x = ix - 128;
            const double z = 128 - iy;
            const double h0 = original[static_cast<std::size_t>(iy) * size + ix];
            const double h = s.terrain.getHeightAtPoint(static_cast<std::uint32_t>(ix),
                    static_cast<std::uint32_t>(iy));
            if (h != h0) ++changed;
            if (x >= -100.0 && x <= 100.0)
            {
                const double d = fabs(z - 60.0);
                const double w = pathWeight(d - 3.0, 8.0);
                const double expected = h0 + (20.0 - h0) * w;
                if (w >= 1.0) ++inside;
                if (fabs(h - expected) > 1e-4) ++wrong;
            }
            else if (x < -111.0 || x > 111.0)
            {
                if (h != h0) ++unchangedWrong;
            }
        }
    }
    printf("  road flattened into a 257 terrain: %zu points written, %d changed, %d inside the "
           "core, %d wrong, %d moved outside the footprint; flatten %.2f ms, update %.2f ms "
           "(%zu owners, %llu vertices)\n",
            written, changed, inside, wrong, unchangedWrong, flattenMs, updateMs,
            s.mirror.touchedOwners.size(), static_cast<unsigned long long>(s.mirror.vertices));
    CHECK(wrong == 0 && unchangedWrong == 0);
    CHECK(static_cast<int>(written) == changed);
    CHECK(inside == 201 * 7);

    const double hand[4][2] = { { 3.0, 1.0 }, { 4.0, 0.984375 }, { 7.0, 0.75 }, { 11.0, 0.0 } };
    for (int k = 0; k < 4; ++k)
    {
        for (int side = -1; side <= 1; side += 2)
        {
            const int iy = 128 - static_cast<int>(60.0 + side * hand[k][0]);
            const double h0 = original[static_cast<std::size_t>(iy) * size + 128];
            const double h = s.terrain.getHeightAtPoint(128, static_cast<std::uint32_t>(iy));
            CHECK(fabs((h - h0) - (20.0 - h0) * hand[k][1]) < 1e-4);
        }
    }
    double previous = 2.0;
    for (int d = 3; d <= 11; ++d)
    {
        const int iy = 128 - (60 + d);
        const double h0 = original[static_cast<std::size_t>(iy) * size + 128];
        const double h = s.terrain.getHeightAtPoint(128, static_cast<std::uint32_t>(iy));
        const double w = (h - h0) / (20.0 - h0);
        CHECK(w <= previous + 1e-9);
        previous = w;
    }

    int depthZero = 0;
    int depthOne = 0;
    int others = 0;
    for (std::size_t i = 0; i < s.mirror.touchedOwners.size(); ++i)
    {
        const Mirror::Entry& entry =
                s.mirror.entries[static_cast<std::size_t>(s.mirror.touchedOwners[i])];
        if (entry.depth == 0) ++depthZero;
        else if (entry.depth == 1 && entry.y == 0)
            ++depthOne;
        else
            ++others;
    }
    CHECK(depthZero == 1 && depthOne == 2 && others == 0);

    const Comparison c = compareWithFresh(s, data);
    printf("  flattened terrain against a fresh build with the same heights: %d vertex, %d delta, "
           "%d normal, %d lightmap, %d box, %d radius and %d level differences\n",
            c.vertexMismatches, c.deltaMismatches, c.normalMismatches, c.lightMismatches,
            c.boxMismatches, c.radiusMismatches, c.levelDifferent);
    CHECK(c.vertexMismatches == 0 && c.deltaMismatches == 0 && c.normalMismatches == 0);
    CHECK(c.lightMismatches == 0 && c.boxMismatches == 0 && c.radiusMismatches == 0 &&
            c.levelDifferent == 0);

    TerrainPath closedLoop;
    closedLoop.add(Math::Vec3(-30.0f, 14.0f, -30.0f), 2.0f);
    closedLoop.add(Math::Vec3(30.0f, 14.0f, -30.0f), 2.0f);
    closedLoop.add(Math::Vec3(30.0f, 14.0f, 30.0f), 2.0f);
    closedLoop.add(Math::Vec3(-30.0f, 14.0f, 30.0f), 2.0f);
    closedLoop.closed = true;
    CHECK(terrainFlattenAlongPath(s.terrain, closedLoop, brush) > 0);
    CHECK(near(s.terrain.getHeightAtPoint(128 - 30, 128 + 30), 14.0f, 1e-4f));
    CHECK(near(s.terrain.getHeightAtPoint(128 - 30, 128), 14.0f, 1e-4f));
    s.terrain.update();
    const Comparison d = compareWithFresh(s, data);
    CHECK(d.vertexMismatches == 0 && d.deltaMismatches == 0 && d.lightMismatches == 0 &&
            d.boxMismatches == 0 && d.levelDifferent == 0);
}

void testPaintRoad()
{
    ct::Vector<float> heights = hills(129);
    Setup s;
    const TerrainImportData data = baseData(129, 17, 33, 128.0f, heights.data());
    CHECK(open(s, data, true));
    for (int i = 0; i < 4; ++i) s.terrain.addLayer(10.0f, nullptr);
    TerrainLayerBlendMap* road = s.terrain.getLayerBlendMap(1);
    TerrainLayerBlendMap* above = s.terrain.getLayerBlendMap(2);
    TerrainLayerBlendMap* top = s.terrain.getLayerBlendMap(3);
    CHECK(road && above && top);
    if (!road || !above || !top) return;
    for (std::uint32_t y = 0; y < 129; ++y)
        for (std::uint32_t x = 0; x < 129; ++x)
        {
            above->setBlendValue(x, y, 0.8f);
            top->setBlendValue(x, y, 0.4f);
        }
    above->update();
    top->update();

    TerrainPath path;
    path.add(Math::Vec3(-40.0f, 0.0f, 10.0f), 2.0f);
    path.add(Math::Vec3(40.0f, 0.0f, 10.0f), 2.0f);
    TerrainPathBrush brush;
    brush.shoulder = 1.0f;
    brush.bank = 8.0f;
    s.maps.blendWrites = 0;
    const std::size_t written = terrainPaintAlongPath(s.terrain, 1, path, brush);
    CHECK(!road->isDirty() && !above->isDirty() && !top->isDirty());
    CHECK(s.maps.blendWrites == 3);

    int wrong = 0;
    int painted = 0;
    int outsideChanged = 0;
    for (int iy = 0; iy < 129; ++iy)
    {
        for (int ix = 0; ix < 129; ++ix)
        {
            const double x = ix - 64;
            const double d = fabs(static_cast<double>(iy - 64) - 10.0);
            const std::uint32_t ux = static_cast<std::uint32_t>(ix);
            const std::uint32_t uy = static_cast<std::uint32_t>(iy);
            const double a = road->getBlendValue(ux, uy);
            const double b = above->getBlendValue(ux, uy);
            const double t = top->getBlendValue(ux, uy);
            if (x >= -40.0 && x <= 40.0)
            {
                const double w = pathWeight(d - 3.0, 8.0);
                if (fabs(a - w) > 1e-5 || fabs(b - 0.8 * (1.0 - w)) > 1e-5 ||
                        fabs(t - 0.4 * (1.0 - w)) > 1e-5)
                    ++wrong;
                if (a > 0.0) ++painted;
            }
            else if ((x < -51.0 || x > 51.0) && (a != 0.0 || b != 0.8f))
                ++outsideChanged;
        }
    }
    printf("  road layer painted into a 129 terrain: %zu texels written, %d in the straight part "
           "(expected %d), %d wrong, %d outside the footprint\n",
            written, painted, 81 * 21, wrong, outsideChanged);
    CHECK(wrong == 0 && outsideChanged == 0);
    CHECK(painted == 81 * 21);
    CHECK(near(road->getBlendValue(64, 74), 1.0f) && near(road->getBlendValue(64, 71), 1.0f) &&
            near(road->getBlendValue(64, 81), 0.75f) &&
            near(road->getBlendValue(64, 79), 0.9375f) && near(road->getBlendValue(64, 85), 0.0f));
    CHECK(near(above->getBlendValue(64, 74), 0.0f) && near(above->getBlendValue(64, 81), 0.2f) &&
            near(above->getBlendValue(64, 85), 0.8f));
    CHECK(terrainPaintAlongPath(s.terrain, 0, path, brush) == 0);
    CHECK(terrainPaintAlongPath(s.terrain, 9, path, brush) == 0);
}

void measureStrokes()
{
    const ct::String path = ct::String(PRISMA3D_ASSETS_DIR) + "/textures/terrain/terrain.png";
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    Pixmap image;
    CHECK(!bytes.empty());
    CHECK(image.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size())));
    if (!image.is_valid()) return;

    TerrainImportData data;
    data.terrainSize = 513;
    data.minBatchSize = 33;
    data.maxBatchSize = 65;
    data.worldSize = 12000.0f;
    data.inputImage = &image;
    data.inputScale = 600.0f;
    data.lightMapSize = 512;
    data.lightMapDirection = Math::Vec3(0.55f, -0.3f, 0.75f).Normalized();

    Setup s;
    CHECK(open(s, data, true));
    const float sizes[3] = { 0.002f, 0.02f, 0.1f };
    const char* const names[3] = { "minimum 0.002", "default 0.02", "maximum 0.1" };
    for (int i = 0; i < 3; ++i)
    {
        const float radius = 0.5f * sizes[i] * 12000.0f;
        const Math::Vec3 centre(1000.0f + 700.0f, 0.0f, 5000.0f - 900.0f);
        s.mirror.resetCounters();
        const std::uint64_t normalBefore = s.maps.normalBytes;
        const std::uint64_t lightBefore = s.maps.lightBytes;

        double t0 = nowMilliseconds();
        s.terrain.setPosition(Math::Vec3(1000.0f, 0.0f, 5000.0f));
        const bool changed = terrainBrushHeight(s.terrain, centre, radius, 250.0f, 0.05f, false);
        const double brushMs = nowMilliseconds() - t0;
        CHECK(changed);

        Math::Vec3 ts;
        s.terrain.getTerrainPosition(centre, ts);
        const float points = radius / (12000.0f / 512.0f);
        TerrainRect rect(static_cast<int>(ts.x * 512.0f - points) - 1,
                static_cast<int>(ts.y * 512.0f - points) - 1,
                static_cast<int>(ts.x * 512.0f + points) + 2,
                static_cast<int>(ts.y * 512.0f + points) + 2);
        rect = rect.intersect(TerrainRect(0, 0, 513, 513));

        t0 = nowMilliseconds();
        s.terrain.updateGeometry();
        const double geometryMs = nowMilliseconds() - t0;
        t0 = nowMilliseconds();
        s.terrain.getQuadTree()->refreshBounds(rect);
        const double boundsMs = nowMilliseconds() - t0;
        const std::uint64_t geometryVertices = s.mirror.vertices;
        const std::uint32_t geometryRanges = s.mirror.ranges;
        const std::size_t geometryOwners = s.mirror.touchedOwners.size();

        t0 = nowMilliseconds();
        const TerrainRect deltaRect = s.terrain.calculateHeightDeltas(rect);
        const double deltaMs = nowMilliseconds() - t0;
        s.mirror.resetCounters();
        t0 = nowMilliseconds();
        s.terrain.finaliseHeightDeltas(deltaRect, false);
        const double deltaUploadMs = nowMilliseconds() - t0;

        TerrainRect normalRect;
        TerrainPixelBox normalBox;
        t0 = nowMilliseconds();
        s.terrain.calculateNormals(rect, normalRect, normalBox);
        s.terrain.finaliseNormals(normalRect, normalBox);
        const double normalMs = nowMilliseconds() - t0;

        TerrainRect lightRect;
        TerrainPixelBox lightBox;
        t0 = nowMilliseconds();
        s.terrain.calculateLightmap(rect, TerrainRect(), lightRect, lightBox);
        s.terrain.finaliseLightmap(lightRect, lightBox);
        const double lightMs = nowMilliseconds() - t0;

        TerrainRect widened;
        s.terrain.widenRectByVector(s.terrain.getLightMapDirection(), rect, widened);
        widened = widened.intersect(TerrainRect(0, 0, 513, 513));
        const TerrainRect composite = s.terrain.compositeImageRect(widened);

        printf("  stroke %s (radius %.0f, rectangle %dx%d points): brush %.3f ms; geometry %.3f ms "
               "of which node bounds %.3f ms (%llu vertices, %llu bytes of 48-byte vertices, %u "
               "ranges, %zu owners); deltas "
               "%.3f "
               "ms plus %.3f ms for the vertex pass (%llu vertices, %zu owners, rectangle %dx%d); "
               "normal map %.3f ms (%llu bytes); lightmap %.2f ms (%llu bytes, %dx%d texels); "
               "composite rectangle %dx%d texels\n",
                names[i], radius, rect.width(), rect.height(), brushMs, geometryMs, boundsMs,
                static_cast<unsigned long long>(geometryVertices),
                static_cast<unsigned long long>(geometryVertices * 48), geometryRanges,
                geometryOwners, deltaMs, deltaUploadMs,
                static_cast<unsigned long long>(s.mirror.vertices), s.mirror.touchedOwners.size(),
                deltaRect.width(), deltaRect.height(), normalMs,
                static_cast<unsigned long long>(s.maps.normalBytes - normalBefore), lightMs,
                static_cast<unsigned long long>(s.maps.lightBytes - lightBefore), lightRect.width(),
                lightRect.height(), composite.width(), composite.height());
        s.terrain.update();
    }

    const float top = s.terrain.getMaxHeight();
    s.maps.lightBytes = 0;
    s.terrain.setHeightAtPoint(100, 100, top + 100.0f);
    double t0 = nowMilliseconds();
    s.terrain.update();
    const double fullMs = nowMilliseconds() - t0;
    printf("  one point raised 100 above the highest point: update %.1f ms, lightmap %llu bytes "
           "(the whole lightmap, because the height pad followed the new range)\n",
            fullMs, static_cast<unsigned long long>(s.maps.lightBytes));
    CHECK(s.maps.lightBytes == 512u * 512u);
    s.maps.lightBytes = 0;
    s.terrain.setHeightAtPoint(101, 100, top + 50.0f);
    s.terrain.update();
    CHECK(s.maps.lightBytes < 512u * 512u / 4);

    s.terrain.setLightMapPadFollowsRange(false);
    s.maps.lightBytes = 0;
    s.terrain.setHeightAtPoint(102, 100, top + 400.0f);
    t0 = nowMilliseconds();
    s.terrain.update();
    printf("  with the pad frozen, a point raised 400 above the highest point: update %.1f ms, "
           "lightmap %llu bytes\n",
            nowMilliseconds() - t0, static_cast<unsigned long long>(s.maps.lightBytes));
    CHECK(s.maps.lightBytes < 512u * 512u / 4);
}

} // namespace

int main()
{
    testSingleEdit();
    testEdgeAndSkirt();
    testNodeBoxesAndLevels();
    testWholeRectangleUpdate();
    testRandomEdits();
    testOwnerGroups();
    testFourNodes();
    testBrushes();
    testPathNearest();
    testSlope();
    testFlattenRoad();
    testPaintRoad();
    measureStrokes();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("terrain edit ok\n");
    return 0;
}
