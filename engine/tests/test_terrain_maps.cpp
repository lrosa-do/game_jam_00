#include "Check.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/terrain/Terrain.h"
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

bool sameRect(const TerrainRect& r, int l, int t, int rr, int b)
{
    return r.left == l && r.top == t && r.right == rr && r.bottom == b;
}

struct Write
{
    int kind = 0;
    TerrainRect box;
};

enum
{
    kNormal = 1,
    kLight = 2,
    kColour = 3,
    kBlend = 4
};

class Recorder : public TerrainMapTarget
{
public:
    TerrainPixelBox normal;
    TerrainPixelBox light;
    TerrainPixelBox colour;
    ct::Vector<TerrainPixelBox> blend;
    ct::Vector<Write> writes;
    ct::Vector<TerrainRect> composites;
    std::uint64_t bytes = 0;
    int created = 0;
    int destroyed = 0;
    bool normalAlive = false;
    bool lightAlive = false;
    bool colourAlive = false;
    bool compositeAlive = false;

    int count(int kind) const
    {
        int total = 0;
        for (std::size_t i = 0; i < writes.size(); ++i)
            if (writes[i].kind == kind) ++total;
        return total;
    }
    void reset()
    {
        writes.clear();
        composites.clear();
        bytes = 0;
    }

    bool createNormalMap(std::uint16_t size) override
    {
        normal.create(size, size, 3);
        for (std::size_t i = 0; i < normal.data.size(); ++i) normal.data[i] = 0;
        normalAlive = true;
        ++created;
        return true;
    }
    void destroyNormalMap() override
    {
        normalAlive = false;
        ++destroyed;
    }
    void writeNormalMap(const TerrainRect& box, const TerrainPixelBox& rgb) override
    {
        copyBox(normal, box, rgb);
        record(kNormal, box, rgb);
    }

    bool createLightmap(std::uint16_t size) override
    {
        light.create(size, size, 1);
        for (std::size_t i = 0; i < light.data.size(); ++i) light.data[i] = 255;
        lightAlive = true;
        ++created;
        return true;
    }
    void destroyLightmap() override
    {
        lightAlive = false;
        ++destroyed;
    }
    void writeLightmap(const TerrainRect& box, const TerrainPixelBox& l8) override
    {
        copyBox(light, box, l8);
        record(kLight, box, l8);
    }

    bool createColourMap(std::uint16_t size) override
    {
        colour.create(size, size, 3);
        for (std::size_t i = 0; i < colour.data.size(); ++i) colour.data[i] = 255;
        colourAlive = true;
        ++created;
        return true;
    }
    void destroyColourMap() override
    {
        colourAlive = false;
        ++destroyed;
    }
    void writeColourMap(const TerrainRect& box, const TerrainPixelBox& rgb) override
    {
        copyBox(colour, box, rgb);
        record(kColour, box, rgb);
    }

    bool createCompositeMap(std::uint16_t) override
    {
        compositeAlive = true;
        ++created;
        return true;
    }
    void destroyCompositeMap() override
    {
        compositeAlive = false;
        ++destroyed;
    }
    void renderCompositeMap(const TerrainRect& imageRect) override
    {
        composites.push_back(imageRect);
    }

    bool createBlendTexture(std::uint8_t index, std::uint16_t size) override
    {
        while (blend.size() <= index) blend.push_back(TerrainPixelBox());
        blend[index].create(size, size, 4);
        for (std::size_t i = 0; i < blend[index].data.size(); ++i) blend[index].data[i] = 0;
        ++created;
        return true;
    }
    void destroyBlendTexture(std::uint8_t) override
    {
        ++destroyed;
    }
    void writeBlendTexture(std::uint8_t index, const TerrainRect& box,
            const TerrainPixelBox& rgba) override
    {
        copyBox(blend[index], box, rgba);
        record(kBlend, box, rgba);
    }

private:
    static void copyBox(TerrainPixelBox& target, const TerrainRect& box, const TerrainPixelBox& src)
    {
        CHECK(static_cast<std::uint32_t>(box.width()) == src.width);
        CHECK(static_cast<std::uint32_t>(box.height()) == src.height);
        for (int y = 0; y < box.height(); ++y)
            for (int x = 0; x < box.width(); ++x)
                for (std::uint32_t c = 0; c < src.components; ++c)
                    target.at(static_cast<std::uint32_t>(box.left + x),
                            static_cast<std::uint32_t>(box.top + y))[c] =
                            src.at(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y))[c];
    }
    void record(int kind, const TerrainRect& box, const TerrainPixelBox& src)
    {
        Write w;
        w.kind = kind;
        w.box = box;
        writes.push_back(w);
        bytes += src.data.size();
    }
};

TerrainImportData baseData(std::uint16_t size, float worldSize, const float* heights)
{
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = size;
    data.maxBatchSize = size;
    data.worldSize = worldSize;
    data.inputFloat = heights;
    data.layerBlendMapSize = 129;
    data.lightMapSize = size;
    data.compositeMapSize = 64;
    data.defaultGlobalColourMapSize = 64;
    return data;
}

ct::Vector<float> heightsOf(int size, float (*shape)(int, int))
{
    ct::Vector<float> heights;
    heights.resize(static_cast<std::size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            heights[static_cast<std::size_t>(y) * size + x] = shape(x, y);
    return heights;
}

float planeShape(int, int)
{
    return 7.0f;
}

float rampShape(int x, int)
{
    return 0.5f * static_cast<float>(x);
}

float rampYShape(int, int y)
{
    return 0.5f * static_cast<float>(y);
}

float pyramidShape(int x, int y)
{
    const int dx = abs(x - 16);
    const int dy = abs(y - 16);
    return static_cast<float>(16 - (dx > dy ? dx : dy));
}

float pillarShape(int x, int y)
{
    return (x >= 10 && x <= 12 && y >= 10 && y <= 12) ? 10.0f : 0.0f;
}

bool texel(const TerrainPixelBox& box, int x, int y, int r, int g, int b)
{
    const std::uint8_t* p = box.at(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
    return p[0] == r && p[1] == g && p[2] == b;
}

void testNormalMaps()
{
    {
        ct::Vector<float> heights = heightsOf(33, planeShape);
        Terrain terrain;
        Recorder recorder;
        terrain.setMapTarget(&recorder);
        CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
        terrain.loadMaps(true, false);
        CHECK(recorder.normalAlive && recorder.normal.width == 33 && recorder.normal.height == 33);
        CHECK(recorder.count(kNormal) == 1 && sameRect(recorder.writes[0].box, 0, 0, 33, 33));
        bool all = true;
        for (int y = 0; y < 33; ++y)
            for (int x = 0; x < 33; ++x) all = all && texel(recorder.normal, x, y, 127, 255, 127);
        CHECK(all);
        terrain.unprepare();
        CHECK(!recorder.normalAlive);
    }
    {
        ct::Vector<float> heights = heightsOf(33, rampShape);
        Terrain terrain;
        Recorder recorder;
        terrain.setMapTarget(&recorder);
        CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
        terrain.loadMaps(true, false);
        bool all = true;
        for (int y = 1; y < 32; ++y)
            for (int x = 1; x < 32; ++x) all = all && texel(recorder.normal, x, y, 70, 241, 127);
        CHECK(all);
    }
    {
        ct::Vector<float> heights = heightsOf(33, rampYShape);
        Terrain terrain;
        Recorder recorder;
        terrain.setMapTarget(&recorder);
        CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
        terrain.loadMaps(true, false);
        bool all = true;
        for (int y = 1; y < 32; ++y)
            for (int x = 1; x < 32; ++x) all = all && texel(recorder.normal, x, y, 127, 241, 184);
        CHECK(all);
    }
    {
        ct::Vector<float> heights = heightsOf(33, pyramidShape);
        Terrain terrain;
        Recorder recorder;
        terrain.setMapTarget(&recorder);
        CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
        terrain.loadMaps(true, false);
        CHECK(texel(recorder.normal, 16, 32 - 16, 127, 255, 127));
        CHECK(texel(recorder.normal, 24, 32 - 16, 217, 217, 127));
        CHECK(texel(recorder.normal, 8, 32 - 16, 37, 217, 127));
        CHECK(texel(recorder.normal, 16, 32 - 24, 127, 217, 37));
        CHECK(texel(recorder.normal, 16, 32 - 8, 127, 217, 217));
        const Math::Vec3 flank = terrain.calculateNormal(24, 16);
        CHECK(near(flank.x, 0.70710678f, 1e-5f) && near(flank.y, 0.70710678f, 1e-5f) &&
                near(flank.z, 0.0f, 1e-5f));
    }
}

void testDirtyRectangles()
{
    ct::Vector<float> heights = heightsOf(33, pyramidShape);
    Terrain terrain;
    Recorder recorder;
    terrain.setMapTarget(&recorder);
    CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
    terrain.loadMaps(true, false);
    CHECK(terrain.getQuadTree() != nullptr);
    recorder.reset();

    terrain.dirtyRect(TerrainRect(10, 12, 14, 20));
    terrain.updateDerivedData();
    CHECK(recorder.writes.size() == 1 && recorder.writes[0].kind == kNormal);
    CHECK(sameRect(recorder.writes[0].box, 9, 12, 15, 22));
    CHECK(recorder.bytes == 6u * 10u * 3u);
    CHECK(recorder.composites.empty());

    recorder.reset();
    terrain.dirtyRect(TerrainRect(0, 0, 4, 4));
    terrain.updateDerivedData();
    CHECK(recorder.writes.size() == 1 && sameRect(recorder.writes[0].box, 0, 28, 5, 33));
    CHECK(recorder.bytes == 5u * 5u * 3u);

    recorder.reset();
    terrain.dirtyRect(TerrainRect(30, 30, 33, 33));
    terrain.dirtyRect(TerrainRect(2, 2, 4, 4));
    terrain.updateDerivedData();
    CHECK(recorder.writes.size() == 1 && sameRect(recorder.writes[0].box, 1, 0, 33, 32));

    recorder.reset();
    terrain.updateDerivedData();
    CHECK(recorder.writes.empty());

    recorder.reset();
    terrain.dirty();
    terrain.updateDerivedData();
    CHECK(recorder.writes.size() == 1 && sameRect(recorder.writes[0].box, 0, 0, 33, 33));
    CHECK(recorder.bytes == 33u * 33u * 3u);
}

void testLightmap()
{
    ct::Vector<float> heights = heightsOf(33, pillarShape);
    Terrain terrain;
    Recorder recorder;
    terrain.setMapTarget(&recorder);
    TerrainImportData data = baseData(33, 32.0f, heights.data());
    data.lightMapDirection = Math::Vec3(1.0f, -1.0f, 0.0f).Normalized();
    CHECK(terrain.prepare(data));
    terrain.loadMaps(false, true);
    CHECK(recorder.lightAlive && recorder.light.width == 33);
    CHECK(recorder.count(kLight) == 1 && sameRect(recorder.writes[0].box, 0, 0, 33, 33));

    auto lit = [&](int x, int y) {
        return recorder.light.at(static_cast<std::uint32_t>(x),
                static_cast<std::uint32_t>(32 - y))[0];
    };
    CHECK(lit(11, 11) == 255);
    CHECK(lit(13, 11) == 0);
    CHECK(lit(18, 11) == 0);
    CHECK(lit(21, 11) == 0);
    CHECK(lit(24, 11) == 255);
    CHECK(lit(30, 11) == 255);
    CHECK(lit(9, 11) == 255);
    CHECK(lit(13, 5) == 255);
    CHECK(lit(18, 25) == 255);
    int outside = 0;
    int row = 0;
    for (int y = 0; y < 33; ++y)
        for (int x = 0; x < 33; ++x)
            if (lit(x, y) == 0)
            {
                if (y >= 10 && y <= 12) ++row;
                else
                    ++outside;
            }
    printf("  pillar lightmap from +x: %d dark texels in the three rows of the pillar, %d "
           "elsewhere\n",
            row, outside);
    CHECK(outside == 0);
    for (int x = 0; x < 33; ++x) CHECK((lit(x, 11) == 0) == (x >= 13 && x <= 22));

    TerrainRect widened;
    terrain.widenRectByVector(Math::Vec3(1.0f, -1.0f, 0.0f), TerrainRect(10, 10, 13, 13), widened);
    CHECK(sameRect(widened, 10, 10, 23, 13));
    terrain.widenRectByVector(Math::Vec3(0.0f, 0.0f, 0.0f), TerrainRect(10, 10, 13, 13), widened);
    CHECK(sameRect(widened, 10, 10, 13, 13));

    recorder.reset();
    terrain.dirtyLightmapRect(TerrainRect(10, 10, 13, 13));
    terrain.updateDerivedData();
    CHECK(recorder.count(kLight) == 1 && recorder.count(kNormal) == 0);
    CHECK(sameRect(recorder.writes[0].box, 9, 33 - 14, 24, 33 - 9));
    CHECK(lit(13, 11) == 0 && lit(22, 11) == 0 && lit(23, 11) == 255);

    terrain.setLightMapDirection(Math::Vec3(-1.0f, -1.0f, 0.0f).Normalized());
    terrain.dirty();
    terrain.updateDerivedData();
    CHECK(lit(9, 11) == 0 && lit(1, 11) == 0 && lit(13, 11) == 255 && lit(24, 11) == 255);
    CHECK(lit(11, 11) == 255);

    terrain.setLightMapDirection(Math::Vec3(0.0f, -1.0f, 1.0f).Normalized());
    terrain.dirty();
    terrain.updateDerivedData();
    CHECK(lit(11, 9) == 0 && lit(11, 5) == 0);
    CHECK(lit(11, 13) == 255 && lit(11, 20) == 255 && lit(11, 11) == 255);
    CHECK(lit(5, 9) == 255 && lit(20, 5) == 255);
}

void testComposite()
{
    ct::Vector<float> heights = heightsOf(33, pyramidShape);
    Terrain terrain;
    Recorder recorder;
    terrain.setMapTarget(&recorder);
    CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
    terrain.loadMaps(true, true);
    terrain._setCompositeMapRequired(true);
    CHECK(recorder.compositeAlive);
    CHECK(recorder.composites.size() == 1 && sameRect(recorder.composites[0], 0, 0, 64, 64));

    recorder.reset();
    terrain.dirtyRect(TerrainRect(8, 8, 16, 16));
    terrain.updateDerivedData();
    CHECK(recorder.count(kNormal) == 1 && recorder.count(kLight) == 1);
    CHECK(recorder.composites.size() == 1);
    TerrainRect widened;
    terrain.widenRectByVector(terrain.getLightMapDirection(), TerrainRect(8, 8, 16, 16), widened);
    widened = widened.intersect(TerrainRect(0, 0, 33, 33));
    const TerrainRect expected = terrain.compositeImageRect(widened);
    CHECK(recorder.composites.size() == 1 &&
            sameRect(recorder.composites[0], expected.left, expected.top, expected.right,
                    expected.bottom));
    CHECK(sameRect(terrain.compositeImageRect(TerrainRect(8, 8, 16, 16)), 16, 34, 31, 49));
    CHECK(sameRect(terrain.compositeImageRect(TerrainRect(0, 0, 33, 33)), 0, 0, 64, 64));

    CHECK(terrain.getMaterialLodIndex(0.0f) == 0);
    CHECK(terrain.getMaterialLodIndex(3999.0f * 3999.0f) == 0);
    CHECK(terrain.getMaterialLodIndex(4000.0f * 4000.0f) == 1);
    terrain.setCompositeMapDistance(100.0f);
    CHECK(terrain.getMaterialLodIndex(99.0f * 99.0f) == 0);
    CHECK(terrain.getMaterialLodIndex(101.0f * 101.0f) == 1);
    terrain._setCompositeMapRequired(false);
    CHECK(!recorder.compositeAlive);
    CHECK(terrain.getMaterialLodIndex(1.0e9f) == 0);

    {
        Terrain far;
        Recorder farRecorder;
        far.setMapTarget(&farRecorder);
        TerrainImportData data = baseData(33, 32.0f, heights.data());
        data.compositeMapDistance = 50.0f;
        CHECK(far.prepare(data));
        far._setCompositeMapRequired(true);
        far.load();
        TerrainLodView view;
        view.position = Math::Vec3(0.0f, 10.0f, 0.0f);
        far.calculateCurrentLod(view);
        const TerrainQuadTreeNode* root = far.getQuadTree();
        CHECK(root->getMaterialLodIndex() == 0);
        view.position = Math::Vec3(0.0f, 10.0f, 200.0f);
        far.calculateCurrentLod(view);
        CHECK(root->getMaterialLodIndex() == 1);
    }
}

void testColourMap()
{
    ct::Vector<float> heights = heightsOf(33, planeShape);
    Terrain terrain;
    Recorder recorder;
    terrain.setMapTarget(&recorder);
    CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
    terrain.loadMaps(true, false);

    Pixmap image(128, 128, 3);
    for (int y = 0; y < 128; ++y)
        for (int x = 0; x < 128; ++x)
            image.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    x < 64 ? 255 : 0, 0, x < 64 ? 0 : 255, 255);
    CHECK(!terrain.loadGlobalColourMap(image));
    CHECK(!terrain.getGlobalColourMapEnabled());
    terrain.setGlobalColourMapEnabled(true);
    CHECK(terrain.getGlobalColourMapEnabled() && terrain.getGlobalColourMapSize() == 64);
    CHECK(recorder.colourAlive && recorder.colour.width == 64);
    CHECK(terrain.isMaterialDirty());
    recorder.reset();
    CHECK(terrain.loadGlobalColourMap(image));
    CHECK(recorder.count(kColour) == 1 && sameRect(recorder.writes[0].box, 0, 0, 64, 64));
    CHECK(texel(recorder.colour, 5, 5, 255, 0, 0));
    CHECK(texel(recorder.colour, 10, 40, 255, 0, 0));
    CHECK(texel(recorder.colour, 31, 63, 255, 0, 0));
    CHECK(texel(recorder.colour, 32, 0, 0, 0, 255));
    CHECK(texel(recorder.colour, 60, 40, 0, 0, 255));
    terrain.setGlobalColourMapEnabled(true, 32);
    CHECK(terrain.getGlobalColourMapSize() == 32 && recorder.colour.width == 32);
    terrain.setGlobalColourMapEnabled(false);
    CHECK(!recorder.colourAlive);
}

void testLayers()
{
    ct::Vector<float> heights = heightsOf(129, planeShape);
    {
        TerrainImportData data = baseData(129, 12000.0f, heights.data());
        data.defaultLayerTextureWorldSize = 10.0f;
        TerrainLayerInstance a;
        a.worldSize = 200.0f;
        a.textureNames.push_back("a_diff");
        a.textureNames.push_back("a_norm");
        TerrainLayerInstance b;
        b.worldSize = 400.0f;
        b.textureNames.push_back("b_diff");
        data.layerList.push_back(a);
        data.layerList.push_back(b);

        Terrain terrain;
        Recorder recorder;
        terrain.setMapTarget(&recorder);
        CHECK(terrain.prepare(data));
        CHECK(terrain.getLayerCount() == 2);
        CHECK(near(terrain.getLayerWorldSize(0), 200.0f) &&
                near(terrain.getLayerWorldSize(1), 400.0f));
        CHECK(near(terrain.getLayerUVMultiplier(0), 60.0f) &&
                near(terrain.getLayerUVMultiplier(1), 30.0f));
        CHECK(near(terrain.getLayerUVMultiplier(7), 60.0f));
        CHECK(near(terrain.getLayerWorldSize(9), 200.0f));
        CHECK(terrain.getLayerTextureName(0, 0) == "a_diff" &&
                terrain.getLayerTextureName(0, 1) == "a_norm");
        CHECK(terrain.getLayerTextureName(1, 0) == "b_diff" &&
                terrain.getLayerTextureName(1, 1).empty());
        CHECK(terrain.getLayerTextureName(5, 0).empty() &&
                terrain.getLayerTextureName(0, 2).empty());

        terrain.setLayerWorldSize(1, 100.0f);
        CHECK(near(terrain.getLayerUVMultiplier(1), 120.0f) && terrain.isMaterialParamsDirty());
        terrain.clearMaterialDirty();
        terrain.setLayerTextureName(1, 0, "b_diff");
        CHECK(!terrain.isMaterialDirty());
        terrain.setLayerTextureName(1, 0, "c_diff");
        CHECK(terrain.isMaterialDirty());

        terrain.addLayer(0.0f, nullptr);
        CHECK(terrain.getLayerCount() == 3 && near(terrain.getLayerWorldSize(2), 10.0f));
        CHECK(near(terrain.getLayerUVMultiplier(2), 1200.0f));
        CHECK(terrain.getMaxLayers() == 5);
        terrain.setGlobalColourMapEnabled(true);
        CHECK(terrain.getMaxLayers() == 5);
        CHECK(Terrain::getBlendTextureCount(2) == 1 && Terrain::getBlendTextureCount(5) == 1 &&
                Terrain::getBlendTextureCount(6) == 2 && Terrain::getBlendTextureCount(9) == 2);
        CHECK(terrain.getLayerBlendMap(0) == nullptr);
        CHECK(terrain.getLayerBlendMap(3) == nullptr);
        CHECK(terrain.getLayerBlendMap(1) != nullptr);
    }

    {
        TerrainImportData data = baseData(129, 128.0f, heights.data());
        data.worldSize = 128.0f;
        Terrain terrain;
        Recorder recorder;
        terrain.setMapTarget(&recorder);
        CHECK(terrain.prepare(data));
        terrain.addLayer(10.0f, nullptr);
        terrain.addLayer(10.0f, nullptr);
        terrain.addLayer(10.0f, nullptr);
        terrain.loadMaps(false, false);
        CHECK(recorder.blend.size() == 1 && recorder.blend[0].width == 129);
        CHECK(terrain.getBlendTextureCount() == 1);

        TerrainLayerBlendMap* one = terrain.getLayerBlendMap(1);
        TerrainLayerBlendMap* two = terrain.getLayerBlendMap(2);
        CHECK(one && two && one != two && terrain.getLayerBlendMap(1) == one);
        CHECK(one->getChannel() == 0 && two->getChannel() == 1 && one->getSize() == 129);
        CHECK(terrain.getBlendTextureIndex(2) == 1);
        CHECK(terrain.getLayerBlendTextureIndex(2).texture == 0 &&
                terrain.getLayerBlendTextureIndex(2).channel == 1);

        float u = 0.0f;
        float v = 0.0f;
        one->convertImageToUVSpace(64, 32, &u, &v);
        CHECK(near(u, 0.5f) && near(v, 0.25f));
        one->convertImageToTerrainSpace(64, 32, &u, &v);
        CHECK(near(u, 0.5f) && near(v, 0.75f));
        std::size_t ix = 0;
        std::size_t iy = 0;
        one->convertTerrainToImageSpace(0.5f, 0.75f, &ix, &iy);
        CHECK(ix == 64 && iy == 32);
        one->convertUVToImageSpace(0.5f, 0.25f, &ix, &iy);
        CHECK(ix == 64 && iy == 32);
        one->convertUVToImageSpace(1.0f, 1.0f, &ix, &iy);
        CHECK(ix == 128 && iy == 128);
        one->convertWorldToUVSpace(Math::Vec3(0.0f, 5.0f, 32.0f), &u, &v);
        CHECK(near(u, 0.5f) && near(v, 0.75f));
        one->convertWorldToUVSpace(Math::Vec3(-64.0f, 5.0f, 64.0f), &u, &v);
        CHECK(near(u, 0.0f) && near(v, 1.0f));
        Math::Vec3 world;
        one->convertUVToWorldSpace(0.5f, 0.75f, &world);
        CHECK(near(world.x, 0.0f, 1e-3f) && near(world.z, 32.0f, 1e-3f));
        one->convertUVToWorldSpace(1.0f, 0.0f, &world);
        CHECK(near(world.x, 64.0f, 1e-3f) && near(world.z, -64.0f, 1e-3f));

        recorder.reset();
        one->setBlendValue(10, 20, 0.5f);
        CHECK(near(one->getBlendValue(10, 20), 0.5f) && one->isDirty());
        CHECK(sameRect(one->getDirtyBox(), 10, 20, 11, 21));
        one->setBlendValue(30, 40, 1.0f);
        CHECK(sameRect(one->getDirtyBox(), 10, 20, 31, 41));
        CHECK(recorder.writes.empty());
        one->update();
        CHECK(!one->isDirty());
        CHECK(recorder.writes.size() == 1 && recorder.writes[0].kind == kBlend);
        CHECK(sameRect(recorder.writes[0].box, 10, 20, 31, 41));
        CHECK(recorder.bytes == 21u * 21u * 4u);
        const std::uint8_t* a = recorder.blend[0].at(10, 20);
        const std::uint8_t* b = recorder.blend[0].at(30, 40);
        const std::uint8_t* c = recorder.blend[0].at(11, 20);
        CHECK(a[0] == 127 && a[1] == 0 && a[2] == 0 && a[3] == 0);
        CHECK(b[0] == 255 && b[1] == 0);
        CHECK(c[0] == 0);

        recorder.reset();
        two->setBlendValue(5, 6, 1.0f);
        two->update();
        CHECK(recorder.writes.size() == 1 && sameRect(recorder.writes[0].box, 5, 6, 6, 7));
        CHECK(recorder.bytes == 4u);
        CHECK(recorder.blend[0].at(5, 6)[1] == 255 && recorder.blend[0].at(5, 6)[0] == 0);
        CHECK(near(one->getBlendValue(10, 20), 0.5f));

        two->update();
        CHECK(recorder.writes.size() == 1);

        float* pointer = one->getBlendPointer();
        for (int i = 0; i < 129 * 129; ++i) pointer[i] = i % 2 ? 1.0f : 0.0f;
        one->dirty();
        recorder.reset();
        one->update();
        CHECK(recorder.writes.size() == 1 && sameRect(recorder.writes[0].box, 0, 0, 129, 129));
        CHECK(recorder.blend[0].at(1, 0)[0] == 255 && recorder.blend[0].at(2, 0)[0] == 0);

        Pixmap image(2, 2, 1);
        image.pixels[0] = 255;
        image.pixels[1] = 0;
        image.pixels[2] = 0;
        image.pixels[3] = 255;
        two->blit(image, TerrainRect(0, 0, 4, 4));
        CHECK(sameRect(two->getDirtyBox(), 0, 0, 4, 4));
        two->update();
        CHECK(recorder.blend[0].at(0, 0)[1] == 255 && recorder.blend[0].at(1, 0)[1] == 255);
        CHECK(recorder.blend[0].at(2, 0)[1] == 0 && recorder.blend[0].at(0, 3)[1] == 0);
        CHECK(recorder.blend[0].at(3, 3)[1] == 255 && recorder.blend[0].at(4, 3)[1] == 0);
        two->loadImage(image);
        CHECK(sameRect(two->getDirtyBox(), 0, 0, 129, 129));

        recorder.reset();
        terrain._setCompositeMapRequired(true);
        recorder.reset();
        one->setBlendValue(64, 100, 1.0f);
        one->update();
        CHECK(recorder.composites.empty());
        terrain.advanceDeferredUpdates(1999);
        CHECK(recorder.composites.empty());
        terrain.advanceDeferredUpdates(1);
        CHECK(recorder.composites.size() == 1);
        CHECK(sameRect(recorder.composites[0], 32, 49, 33, 51));
        terrain.advanceDeferredUpdates(5000);
        CHECK(recorder.composites.size() == 1);
    }

    {
        ct::Vector<float> small = heightsOf(33, planeShape);
        TerrainImportData data = baseData(33, 32.0f, small.data());
        data.layerBlendMapSize = 8;
        Terrain terrain;
        Recorder recorder;
        terrain.setMapTarget(&recorder);
        CHECK(terrain.prepare(data));
        for (int i = 0; i < 4; ++i) terrain.addLayer(10.0f + static_cast<float>(i), nullptr);
        terrain.loadMaps(false, false);
        for (std::uint8_t layer = 1; layer < 4; ++layer)
        {
            TerrainLayerBlendMap* map = terrain.getLayerBlendMap(layer);
            CHECK(map != nullptr);
            map->setBlendValue(layer, 0, 1.0f);
            map->update();
        }
        CHECK(recorder.blend[0].at(1, 0)[0] == 255 && recorder.blend[0].at(2, 0)[1] == 255 &&
                recorder.blend[0].at(3, 0)[2] == 255);

        terrain.removeLayer(1);
        CHECK(terrain.getLayerCount() == 3);
        CHECK(near(terrain.getLayerWorldSize(1), 12.0f) &&
                near(terrain.getLayerWorldSize(2), 13.0f));
        CHECK(recorder.blend[0].at(2, 0)[0] == 255 && recorder.blend[0].at(3, 0)[1] == 255);
        CHECK(recorder.blend[0].at(1, 0)[0] == 0);
        CHECK(recorder.blend[0].at(3, 0)[2] == 0 && recorder.blend[0].at(2, 0)[1] == 0);
        CHECK(recorder.blend[0].at(1, 0)[1] == 0);

        terrain.addLayer(1, 77.0f, nullptr);
        CHECK(terrain.getLayerCount() == 4 && near(terrain.getLayerWorldSize(1), 77.0f));
        CHECK(near(terrain.getLayerWorldSize(2), 12.0f) &&
                near(terrain.getLayerWorldSize(3), 13.0f));
        CHECK(recorder.blend[0].at(1, 0)[0] == 0 && recorder.blend[0].at(1, 0)[1] == 0);
    }
}

void testOptionsAndTexts()
{
    ct::Vector<float> heights = heightsOf(33, planeShape);
    Terrain terrain;
    CHECK(terrain.prepare(baseData(33, 32.0f, heights.data())));
    CHECK(terrain.getLightmapSize() == 33 && terrain.getCompositeMapSize() == 64);
    CHECK(near(terrain.getCompositeMapDistance(), 4000.0f));
    CHECK(near(terrain.getLightMapDirection().x, 0.70710678f) &&
            near(terrain.getLightMapDirection().y, -0.70710678f));
    CHECK(terrain.getLayerBlendMapSize() == 129);
    CHECK(terrain.getLayerCount() == 0 && near(terrain.getLayerUVMultiplier(0), 100.0f));
    terrain.unprepare();
}

void measureDemoMaps()
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
    data.pos = Math::Vec3(1000.0f, 0.0f, 5000.0f);
    data.lightMapDirection = Math::Vec3(0.55f, -0.3f, 0.75f).Normalized();

    Terrain terrain;
    Recorder recorder;
    terrain.setMapTarget(&recorder);
    CHECK(terrain.prepare(data));
    terrain.load();

    double t0 = nowMilliseconds();
    terrain.loadMaps(true, false);
    printf("  normal map %ux%u, whole terrain: %.2f ms (%llu bytes)\n", recorder.normal.width,
            recorder.normal.height, nowMilliseconds() - t0,
            static_cast<unsigned long long>(recorder.bytes));

    recorder.reset();
    t0 = nowMilliseconds();
    terrain.dirtyRect(TerrainRect(200, 200, 216, 216));
    terrain.updateDerivedData();
    printf("  normal map, 16x16 rectangle: %.3f ms (%llu bytes)\n", nowMilliseconds() - t0,
            static_cast<unsigned long long>(recorder.bytes));

    const std::uint16_t sizes[2] = { 512, 1024 };
    for (int i = 0; i < 2; ++i)
    {
        Terrain lit;
        Recorder litRecorder;
        lit.setMapTarget(&litRecorder);
        TerrainImportData litData = data;
        litData.lightMapSize = sizes[i];
        CHECK(lit.prepare(litData));
        t0 = nowMilliseconds();
        lit.loadMaps(false, true);
        printf("  lightmap %ux%u, whole terrain: %.1f ms (%llu bytes)\n", sizes[i], sizes[i],
                nowMilliseconds() - t0, static_cast<unsigned long long>(litRecorder.bytes));
        int dark = 0;
        for (std::size_t k = 0; k < litRecorder.light.data.size(); ++k)
            if (litRecorder.light.data[k] == 0) ++dark;
        printf("    %d of %zu texels in shadow\n", dark, litRecorder.light.data.size());
        CHECK(dark > 0 && dark < static_cast<int>(litRecorder.light.data.size()));

        if (i == 0)
        {
            litRecorder.reset();
            t0 = nowMilliseconds();
            lit.dirtyRect(TerrainRect(200, 200, 216, 216));
            lit.updateDerivedData(true, Terrain::DERIVED_DATA_LIGHTMAP);
            printf("  lightmap, 16x16 rectangle: %.2f ms (%llu bytes, box %d x %d)\n",
                    nowMilliseconds() - t0, static_cast<unsigned long long>(litRecorder.bytes),
                    litRecorder.writes.empty() ? 0 : litRecorder.writes[0].box.width(),
                    litRecorder.writes.empty() ? 0 : litRecorder.writes[0].box.height());
        }
    }

    Terrain layered;
    Recorder layeredRecorder;
    layered.setMapTarget(&layeredRecorder);
    TerrainImportData layeredData = data;
    layeredData.layerList.resize(3);
    CHECK(layered.prepare(layeredData));
    layered.loadMaps(false, false);
    TerrainLayerBlendMap* blend0 = layered.getLayerBlendMap(1);
    TerrainLayerBlendMap* blend1 = layered.getLayerBlendMap(2);
    CHECK(blend0 && blend1);
    t0 = nowMilliseconds();
    float* p0 = blend0->getBlendPointer();
    float* p1 = blend1->getBlendPointer();
    const std::uint16_t blendSize = layered.getLayerBlendMapSize();
    for (std::uint16_t y = 0; y < blendSize; ++y)
    {
        for (std::uint16_t x = 0; x < blendSize; ++x)
        {
            float tx = 0.0f;
            float ty = 0.0f;
            blend0->convertImageToTerrainSpace(x, y, &tx, &ty);
            const float height = layered.getHeightAtTerrainPosition(tx, ty);
            float w0 = (height - 20.0f) / 15.0f;
            float w1 = (height - 70.0f) / 15.0f;
            w0 = w0 < 0.0f ? 0.0f : (w0 > 1.0f ? 1.0f : w0);
            w1 = w1 < 0.0f ? 0.0f : (w1 > 1.0f ? 1.0f : w1);
            *p0++ = w0;
            *p1++ = w1;
        }
    }
    const double init = nowMilliseconds() - t0;
    blend0->dirty();
    blend1->dirty();
    layeredRecorder.reset();
    t0 = nowMilliseconds();
    blend0->update();
    blend1->update();
    printf("  blend maps 1024x1024 from height: fill %.1f ms, convert+upload both %.2f ms (%llu "
           "bytes)\n",
            init, nowMilliseconds() - t0, static_cast<unsigned long long>(layeredRecorder.bytes));
    layeredRecorder.reset();
    t0 = nowMilliseconds();
    for (int y = 500; y < 532; ++y)
        for (int x = 500; x < 532; ++x) blend0->setBlendValue(x, y, 1.0f);
    blend0->update();
    printf("  blend map, 32x32 brush: %.3f ms (%llu bytes)\n", nowMilliseconds() - t0,
            static_cast<unsigned long long>(layeredRecorder.bytes));
}

} // namespace

int main()
{
    testNormalMaps();
    testDirtyRectangles();
    testLightmap();
    testComposite();
    testColourMap();
    testLayers();
    testOptionsAndTexts();
    measureDemoMaps();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("terrain maps ok\n");
    return 0;
}
