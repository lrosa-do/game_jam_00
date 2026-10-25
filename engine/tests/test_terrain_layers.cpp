#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/vector.hpp>

#include <math.h>
#include <string.h>
#include <time.h>

using namespace prisma3d;

namespace
{

const int kWidth = 160;
const int kHeight = 120;

double nowMilliseconds()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0 + static_cast<double>(ts.tv_nsec) / 1.0e6;
}

struct Picture
{
    ct::Vector<unsigned char> pixels;
    const unsigned char* at(int x, int y) const
    {
        return &pixels[(static_cast<std::size_t>(y) * kWidth + x) * 4];
    }
};

bool readPicture(Picture& picture)
{
    picture.pixels.resize(static_cast<std::size_t>(kWidth) * kHeight * 4);
    prisma::Rect rect;
    rect.width = kWidth;
    rect.height = kHeight;
    return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect,
            picture.pixels.data());
}

struct Frame
{
    explicit Frame(SceneTree& tree)
    {
        tree.process(1.0 / 60.0);
        Main::rendering()->renderViewports();
    }
    ~Frame()
    {
        Main::rendering()->endFrame(true);
    }
};

struct Scene
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Camera3D* camera = nullptr;
};

bool renderPicture(Scene& scene, Picture& picture)
{
    {
        Frame prime(*scene.tree);
    }
    Frame frame(*scene.tree);
    return readPicture(picture);
}

void place(Scene& scene, const Math::Vec3& position)
{
    scene.camera->setGlobalTransform(Math::Mat4::LookAt(position, Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -1.0f))
                                             .InverseAffine());
}

float srgbToLinear(unsigned char value)
{
    const float c = static_cast<float>(value) / 255.0f;
    return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

bool pixelIs(const unsigned char* p, int r, int g, int b, int tolerance = 4)
{
    return abs(p[0] - r) <= tolerance && abs(p[1] - g) <= tolerance && abs(p[2] - b) <= tolerance;
}

TextureId solid(unsigned char r, unsigned char g, unsigned char b, bool srgb, const char* name)
{
    const unsigned char pixel[4] = { r, g, b, 255 };
    prisma::TextureDesc desc;
    desc.format = srgb ? prisma::TextureFormat::RGBA8Srgb : prisma::TextureFormat::RGBA8;
    desc.width = 1;
    desc.height = 1;
    desc.data = pixel;
    desc.debugName = name;
    const TextureId texture = Main::textures()->create(desc, srgb);
    if (texture.valid() && name) Main::textures()->setPath(texture, name);
    return texture;
}

TextureId pair(unsigned char r0, unsigned char g0, unsigned char b0, unsigned char r1,
        unsigned char g1, unsigned char b1)
{
    const unsigned char pixels[8] = { r0, g0, b0, 255, r1, g1, b1, 255 };
    prisma::TextureDesc desc;
    desc.format = prisma::TextureFormat::RGBA8Srgb;
    desc.width = 2;
    desc.height = 1;
    desc.data = pixels;
    desc.debugName = "pair";
    return Main::textures()->create(desc, true);
}

ct::Vector<float> flatHeights(int size, float height = 0.0f)
{
    ct::Vector<float> heights;
    heights.resize(static_cast<std::size_t>(size) * size);
    for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = height;
    return heights;
}

TerrainImportData pageData(const float* heights, std::uint16_t size = 129, float world = 128.0f)
{
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = size;
    data.maxBatchSize = size;
    data.worldSize = world;
    data.inputFloat = heights;
    data.layerBlendMapSize = 129;
    data.lightMapSize = 129;
    data.compositeMapSize = 64;
    data.defaultGlobalColourMapSize = 64;
    return data;
}

Terrain3D* makePage(Scene& scene, bool composite)
{
    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setCompositeMapEnabled(composite);
    terrain->setLayerParallax(0);
    return terrain;
}

void flush(Scene& scene)
{
    Frame frame(*scene.tree);
}

void paintQuadrants(TerrainLayerBlendMap* blend)
{
    float* p = blend->getBlendPointer();
    const int size = blend->getSize();
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
        {
            float value = 0.0f;
            if (x > 64 && y < 64) value = 1.0f;
            if (x == 64 && y < 64) value = 0.5f;
            p[y * size + x] = value;
        }
    blend->dirty();
    blend->update();
}

void testLayersBlend(Scene& scene)
{
    const std::size_t textureBaseline = Main::textures()->count();
    const std::size_t meshBaseline = Main::meshes()->count();
    const std::size_t materialBaseline = Main::materials()->count();

    const TextureId red = solid(255, 0, 0, true, "red");
    const TextureId green = solid(0, 255, 0, true, "green");
    const TextureId flat = solid(128, 128, 255, false, "flat normal");
    ct::Vector<float> heights = flatHeights(129);

    Terrain3D* terrain = makePage(scene, false);
    CHECK(terrain->load(pageData(heights.data())));
    CHECK(terrain->addLayer(100.0f, red, flat));
    CHECK(terrain->addLayer(100.0f, green, flat));
    CHECK(terrain->getTerrain().getLayerCount() == 2);
    TerrainLayerBlendMap* blend = terrain->getLayerBlendMap(1);
    CHECK(blend != nullptr);
    if (!blend) return;
    paintQuadrants(blend);

    scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);
    place(scene, Math::Vec3(0.0f, 60.0f, 0.01f));
    Picture picture;
    CHECK(renderPicture(scene, picture));
    const unsigned char* upperLeft = picture.at(kWidth / 2 - 52, kHeight / 2 - 40);
    const unsigned char* upperRight = picture.at(kWidth / 2 + 52, kHeight / 2 - 40);
    const unsigned char* lowerLeft = picture.at(kWidth / 2 - 52, kHeight / 2 + 40);
    const unsigned char* lowerRight = picture.at(kWidth / 2 + 52, kHeight / 2 + 40);
    const unsigned char* seam = picture.at(kWidth / 2, kHeight / 2 - 40);
    printf("  two layers: upper left %d %d %d, upper right %d %d %d, lower right %d %d %d, seam "
           "%d %d %d\n",
            upperLeft[0], upperLeft[1], upperLeft[2], upperRight[0], upperRight[1], upperRight[2],
            lowerRight[0], lowerRight[1], lowerRight[2], seam[0], seam[1], seam[2]);
    CHECK(pixelIs(upperLeft, 255, 0, 0));
    CHECK(pixelIs(lowerLeft, 255, 0, 0));
    CHECK(pixelIs(lowerRight, 255, 0, 0));
    CHECK(!pixelIs(upperRight, 255, 0, 0));
    CHECK(pixelIs(upperRight, 0, 255, 0));
    CHECK(seam[2] <= 4 && fabsf(srgbToLinear(seam[0]) + srgbToLinear(seam[1]) - 1.0f) < 0.05f);
    CHECK(seam[0] > 20 && seam[1] > 20);

    terrain->resetUploadStats();
    for (int y = 100; y < 105; ++y)
        for (int x = 100; x < 105; ++x) blend->setBlendValue(x, y, 1.0f);
    blend->update();
    CHECK(terrain->getUploadCount() == 1 && terrain->getUploadedBytes() == 100u);
    CHECK(terrain->getTerrain().getLayerBlendMapSize() == 129);

    terrain->resetUploadStats();
    blend->setBlendValue(1, 1, 1.0f);
    blend->setBlendValue(1, 2, 1.0f);
    blend->update();
    CHECK(terrain->getUploadCount() == 1 && terrain->getUploadedBytes() == 1u * 2u * 4u);

    CHECK(terrain->addLayer(10.0f, red, flat));
    CHECK(terrain->addLayer(10.0f, red, flat));
    CHECK(terrain->addLayer(10.0f, red, flat));
    CHECK(terrain->getTerrain().getLayerCount() == 5);
    CHECK(!terrain->addLayer(10.0f, red, flat));
    CHECK(terrain->getTerrain().getLayerCount() == 5);
    CHECK(renderPicture(scene, picture));
    CHECK(pixelIs(picture.at(kWidth / 2 - 52, kHeight / 2 - 40), 255, 0, 0));

    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(red);
    Main::textures()->destroy(green);
    Main::textures()->destroy(flat);
    CHECK(Main::textures()->count() == textureBaseline);
    CHECK(Main::meshes()->count() == meshBaseline);
    CHECK(Main::materials()->count() == materialBaseline);

    Terrain3D* many = makePage(scene, false);
    TerrainImportData data = pageData(heights.data());
    data.layerList.resize(6);
    CHECK(!many->load(data));
    many->queueFree();
    flush(scene);
    CHECK(Main::textures()->count() == textureBaseline);
}

float peakWorld(const Picture& picture, int row, int channel, float lo, float hi)
{
    int best = -1;
    int bestValue = -1;
    const int from = kWidth / 2 + static_cast<int>(lo * 1.732f);
    const int to = kWidth / 2 + static_cast<int>(hi * 1.732f);
    for (int x = from; x <= to; ++x)
    {
        if (picture.at(x, row)[channel] > bestValue)
        {
            bestValue = picture.at(x, row)[channel];
            best = x;
        }
    }
    return static_cast<float>(best - kWidth / 2) / 1.732f;
}

void testLayerScale(Scene& scene)
{
    const TextureId stripes = pair(255, 0, 0, 0, 0, 255);
    const TextureId flat = solid(128, 128, 255, false, "flat normal");
    ct::Vector<float> heights = flatHeights(129);
    Terrain3D* terrain = makePage(scene, false);
    CHECK(terrain->load(pageData(heights.data())));
    CHECK(terrain->addLayer(64.0f, stripes, flat));

    place(scene, Math::Vec3(0.0f, 60.0f, 0.01f));
    Picture picture;
    const int cy = kHeight / 2;
    CHECK(renderPicture(scene, picture));
    CHECK(fabsf(peakWorld(picture, cy, 0, 0.0f, 32.0f) - 16.0f) < 3.0f);
    CHECK(fabsf(peakWorld(picture, cy, 2, -32.0f, 0.0f) + 16.0f) < 3.0f);

    terrain->setLayerWorldSize(0, 128.0f);
    CHECK(fabsf(terrain->getTerrain().getLayerUVMultiplier(0) - 1.0f) < 1e-5f);
    CHECK(renderPicture(scene, picture));
    CHECK(fabsf(peakWorld(picture, cy, 0, -44.0f, -12.0f) + 32.0f) < 3.0f);
    CHECK(fabsf(peakWorld(picture, cy, 2, 12.0f, 44.0f) - 32.0f) < 3.0f);

    terrain->setLayerWorldSize(0, 32.0f);
    CHECK(renderPicture(scene, picture));
    CHECK(fabsf(peakWorld(picture, cy, 0, -32.0f, -16.0f) + 24.0f) < 2.5f);
    CHECK(fabsf(peakWorld(picture, cy, 2, -16.0f, 0.0f) + 8.0f) < 2.5f);
    CHECK(fabsf(peakWorld(picture, cy, 0, 0.0f, 16.0f) - 8.0f) < 2.5f);
    CHECK(fabsf(peakWorld(picture, cy, 2, 16.0f, 32.0f) - 24.0f) < 2.5f);

    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(stripes);
    Main::textures()->destroy(flat);
}

void testColourMap(Scene& scene)
{
    const TextureId white = solid(255, 255, 255, true, "white");
    const TextureId flat = solid(128, 128, 255, false, "flat normal");
    ct::Vector<float> heights = flatHeights(129);
    Terrain3D* terrain = makePage(scene, false);
    CHECK(terrain->load(pageData(heights.data())));
    CHECK(terrain->addLayer(100.0f, white, flat));

    place(scene, Math::Vec3(0.0f, 60.0f, 0.01f));
    Picture picture;
    CHECK(renderPicture(scene, picture));
    CHECK(pixelIs(picture.at(kWidth / 2 - 52, kHeight / 2), 255, 255, 255));

    terrain->setGlobalColourMapEnabled(true, 64);
    Pixmap image(64, 64, 3);
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x)
            image.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    x < 32 ? 255 : 0, 0, x < 32 ? 0 : 255, 255);
    CHECK(terrain->setGlobalColourMap(image));
    CHECK(renderPicture(scene, picture));
    CHECK(pixelIs(picture.at(kWidth / 2 - 52, kHeight / 2), 255, 0, 0));
    CHECK(pixelIs(picture.at(kWidth / 2 + 52, kHeight / 2), 0, 0, 255));

    terrain->setGlobalColourMapEnabled(false);
    CHECK(renderPicture(scene, picture));
    CHECK(pixelIs(picture.at(kWidth / 2 - 52, kHeight / 2), 255, 255, 255));

    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(white);
    Main::textures()->destroy(flat);
}

int compositeRegionDiffers(const Picture& a, const Picture& b)
{
    int count = 0;
    for (int y = 0; y < kHeight; ++y)
        for (int x = 0; x < kWidth; ++x)
            if (abs(a.at(x, y)[0] - b.at(x, y)[0]) > 10 ||
                    abs(a.at(x, y)[1] - b.at(x, y)[1]) > 10 ||
                    abs(a.at(x, y)[2] - b.at(x, y)[2]) > 10)
                ++count;
    return count;
}

unsigned char linearToSrgb(float v)
{
    v = v > 1.0f ? 1.0f : v;
    const float c = v <= 0.0031308f ? v * 12.92f : 1.055f * powf(v, 1.0f / 2.4f) - 0.055f;
    return static_cast<unsigned char>(c * 255.0f + 0.5f);
}

int redRuns(const Picture& picture, int row, int from, int to)
{
    int runs = 0;
    bool inside = false;
    for (int x = from; x <= to; ++x)
    {
        const unsigned char* p = picture.at(x, row);
        const bool red = p[0] > p[2] + 10;
        if (red && !inside) ++runs;
        inside = red;
    }
    return runs;
}

void testColourMapDetail(Scene& scene)
{
    ct::Vector<float> heights = flatHeights(129);
    {
        const TextureId detail = solid(100, 200, 255, true, "detail");
        Terrain3D* terrain = makePage(scene, false);
        CHECK(terrain->load(pageData(heights.data())));
        CHECK(terrain->addLayer(10.0f, detail, TextureId()));
        terrain->setGlobalColourMapEnabled(true, 64);
        Pixmap colour(64, 64, 3);
        for (int i = 0; i < 64 * 64; ++i)
        {
            colour.pixels[i * 3] = 200;
            colour.pixels[i * 3 + 1] = 100;
            colour.pixels[i * 3 + 2] = 50;
        }
        CHECK(terrain->setGlobalColourMap(colour));
        CHECK(terrain->getTerrain().getLayerCount() == 1);

        place(scene, Math::Vec3(0.0f, 60.0f, 0.01f));
        Picture picture;
        CHECK(renderPicture(scene, picture));
        const float d[3] = { srgbToLinear(100), srgbToLinear(200), srgbToLinear(255) };
        const float c[3] = { srgbToLinear(200), srgbToLinear(100), srgbToLinear(50) };
        const int e0 = linearToSrgb(d[0] * c[0]);
        const int e1 = linearToSrgb(d[1] * c[1]);
        const int e2 = linearToSrgb(d[2] * c[2]);
        const unsigned char* p = picture.at(kWidth / 2 - 30, kHeight / 2 + 20);
        printf("  detail (100,200,255) x colour map (200,100,50): pixel %d %d %d, expected %d %d "
               "%d\n",
                p[0], p[1], p[2], e0, e1, e2);
        CHECK(pixelIs(p, e0, e1, e2, 3));
        CHECK(pixelIs(picture.at(kWidth / 2 + 40, kHeight / 2 - 30), e0, e1, e2, 3));

        terrain->queueFree();
        flush(scene);
        Main::textures()->destroy(detail);
    }
    {
        const TextureId stripes = pair(255, 0, 0, 0, 0, 255);
        Terrain3D* terrain = makePage(scene, false);
        CHECK(terrain->load(pageData(heights.data())));
        CHECK(terrain->addLayer(64.0f, stripes, TextureId()));
        terrain->setGlobalColourMapEnabled(true, 64);
        Pixmap white(64, 64, 3);
        for (int i = 0; i < 64 * 64 * 3; ++i) white.pixels[i] = 255;
        CHECK(terrain->setGlobalColourMap(white));

        place(scene, Math::Vec3(0.0f, 120.0f, 0.01f));
        Picture picture;
        CHECK(renderPicture(scene, picture));
        const int row = kHeight / 2;
        const int whole = redRuns(picture, row, kWidth / 2 - 50, kWidth / 2 + 50);
        terrain->setLayerWorldSize(0, 32.0f);
        CHECK(renderPicture(scene, picture));
        const int halved = redRuns(picture, row, kWidth / 2 - 50, kWidth / 2 + 50);
        printf("  detail repeats along the row: %d with world size 64, %d with 32\n", whole,
                halved);
        CHECK(whole == 2 && halved == 4);

        terrain->queueFree();
        flush(scene);
        Main::textures()->destroy(stripes);
    }
}

int darkGaps(const Picture& picture, int row, int from, int to, int threshold)
{
    int gaps = 0;
    int state = 0;
    for (int x = from; x <= to; ++x)
    {
        const bool dark = picture.at(x, row)[0] < threshold;
        if (dark && state == 1) state = 2;
        else if (!dark && state == 2)
        {
            ++gaps;
            state = 1;
        }
        else if (!dark && state == 0)
            state = 1;
    }
    return gaps;
}

void fillColour(Pixmap& map, unsigned char r, unsigned char g, unsigned char b)
{
    for (int i = 0; i < map.width * map.height; ++i)
    {
        map.pixels[i * 3] = r;
        map.pixels[i * 3 + 1] = g;
        map.pixels[i * 3 + 2] = b;
    }
}

void expectDetail(Scene& scene, const char* label, unsigned char stored, float strength)
{
    ct::Vector<float> heights = flatHeights(129);
    const TextureId detail = solid(stored, stored, stored, true, "detail");
    Terrain3D* terrain = makePage(scene, false);
    CHECK(terrain->load(pageData(heights.data())));
    CHECK(terrain->addLayer(10.0f, detail, TextureId()));
    terrain->setGlobalColourMapEnabled(true, 64);
    Pixmap colour(64, 64, 3);
    fillColour(colour, 200, 100, 50);
    CHECK(terrain->setGlobalColourMap(colour));
    terrain->setDetailModulation(true, strength);
    place(scene, Math::Vec3(0.0f, 60.0f, 0.01f));
    Picture picture;
    CHECK(renderPicture(scene, picture));
    const float factor = 1.0f + (static_cast<float>(stored) / 255.0f * 2.0f - 1.0f) * strength;
    const int e0 = linearToSrgb(srgbToLinear(200) * factor);
    const int e1 = linearToSrgb(srgbToLinear(100) * factor);
    const int e2 = linearToSrgb(srgbToLinear(50) * factor);
    const unsigned char* p = picture.at(kWidth / 2 - 30, kHeight / 2 + 20);
    printf("  detail %s: stored %d, strength %.1f, factor %.4f: pixel %d %d %d, expected %d %d "
           "%d\n",
            label, stored, strength, factor, p[0], p[1], p[2], e0, e1, e2);
    CHECK(pixelIs(p, e0, e1, e2, 2));

    terrain->setDetailModulation(false);
    CHECK(renderPicture(scene, picture));
    const unsigned char* q = picture.at(kWidth / 2 - 30, kHeight / 2 + 20);
    const int o0 =
            linearToSrgb(srgbToLinear(static_cast<unsigned char>(stored)) * srgbToLinear(200));
    CHECK(pixelIs(q, o0,
            linearToSrgb(srgbToLinear(static_cast<unsigned char>(stored)) * srgbToLinear(100)),
            linearToSrgb(srgbToLinear(static_cast<unsigned char>(stored)) * srgbToLinear(50)), 2));

    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(detail);
}

void testDetailModulation(Scene& scene)
{
    expectDetail(scene, "neutral", 128, 1.0f);
    expectDetail(scene, "half", 64, 1.0f);
    expectDetail(scene, "doubled", 255, 1.0f);
    expectDetail(scene, "strength 0", 255, 0.0f);
    expectDetail(scene, "strength 0.5", 255, 0.5f);

    ct::Vector<float> heights = flatHeights(129);
    {
        const TextureId detail = solid(64, 64, 64, true, "detail");
        Terrain3D* terrain = makePage(scene, true);
        CHECK(terrain->load(pageData(heights.data())));
        CHECK(terrain->addLayer(10.0f, detail, TextureId()));
        terrain->setGlobalColourMapEnabled(true, 64);
        Pixmap colour(64, 64, 3);
        fillColour(colour, 200, 100, 50);
        CHECK(terrain->setGlobalColourMap(colour));
        terrain->setDetailModulation(true, 1.0f);
        terrain->setCompositeMapLighting(Math::Vec3(1.0f, 1.0f, 1.0f),
                Math::Vec3(0.0f, 0.0f, 0.0f));
        place(scene, Math::Vec3(0.0f, 300.0f, 0.01f));
        Picture nearPicture;
        Picture farPicture;
        terrain->setCompositeMapDistance(10000.0f);
        CHECK(renderPicture(scene, nearPicture));
        CHECK(terrain->getNodesDrawnComposite() == 0);
        terrain->setCompositeMapDistance(10.0f);
        CHECK(renderPicture(scene, farPicture));
        CHECK(terrain->getNodesDrawnComposite() == 1);
        const unsigned char* a = nearPicture.at(kWidth / 2 - 10, kHeight / 2 + 10);
        const unsigned char* b = farPicture.at(kWidth / 2 - 10, kHeight / 2 + 10);
        printf("  detail near %d %d %d against composite %d %d %d\n", a[0], a[1], a[2], b[0], b[1],
                b[2]);
        CHECK(pixelIs(a, b[0], b[1], b[2], 3));
        const float factor = 64.0f / 255.0f * 2.0f;
        CHECK(pixelIs(b, linearToSrgb(srgbToLinear(200) * factor),
                linearToSrgb(srgbToLinear(100) * factor), linearToSrgb(srgbToLinear(50) * factor),
                3));
        terrain->queueFree();
        flush(scene);
        Main::textures()->destroy(detail);
    }
    {
        const TextureId stripes = pair(255, 255, 255, 128, 128, 128);
        Terrain3D* terrain = makePage(scene, false);
        CHECK(terrain->load(pageData(heights.data())));
        CHECK(!terrain->getCompositeMaterial().valid());
        CHECK(terrain->addLayer(64.0f, stripes, TextureId()));
        terrain->setGlobalColourMapEnabled(true, 64);
        Pixmap white(64, 64, 3);
        fillColour(white, 100, 100, 100);
        CHECK(terrain->setGlobalColourMap(white));
        terrain->setDetailModulation(true, 1.0f);
        terrain->setCompositeMapDistance(10.0f);
        place(scene, Math::Vec3(0.0f, 120.0f, 0.01f));
        Picture picture;
        CHECK(renderPicture(scene, picture));
        CHECK(terrain->getNodesDrawnComposite() == 0);
        CHECK(!terrain->isNodeComposite(terrain->getTerrain().getQuadTree()));
        const int whole = darkGaps(picture, kHeight / 2, kWidth / 2 - 54, kWidth / 2 + 54, 118);
        terrain->setLayerWorldSize(0, 32.0f);
        CHECK(renderPicture(scene, picture));
        CHECK(terrain->getNodesDrawnComposite() == 0);
        const int halved = darkGaps(picture, kHeight / 2, kWidth / 2 - 54, kWidth / 2 + 54, 118);
        printf("  far detail with the composite map off: %d dark gaps at world size 64, %d at 32\n",
                whole, halved);
        CHECK(whole == 2 && halved == 4);
        terrain->queueFree();
        flush(scene);
        Main::textures()->destroy(stripes);
    }
}

void fillAlignedHeights(Pixmap& heights)
{
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x)
            heights.pixels[y * 129 + x] = (x >= 16 && x < 56 && y >= 16 && y < 30) ? 255 : 0;
}

void testAlignment(Scene& scene)
{
    Pixmap heights(129, 129, 1);
    fillAlignedHeights(heights);
    Pixmap colour(128, 128, 3);
    for (int y = 0; y < 128; ++y)
    {
        for (int x = 0; x < 128; ++x)
        {
            const float fx = (static_cast<float>(x) + 0.5f) / 128.0f * 129.0f;
            const float fy = (static_cast<float>(y) + 0.5f) / 128.0f * 129.0f;
            const bool block = fx >= 16.0f && fx < 56.0f && fy >= 16.0f && fy < 30.0f;
            unsigned char* p = &colour.pixels[(y * 128 + x) * 3];
            p[0] = block ? 255 : 0;
            p[1] = 0;
            p[2] = block ? 0 : 255;
        }
    }
    const TextureId white = solid(255, 255, 255, true, "white");

    TerrainImportData data;
    data.terrainSize = 129;
    data.worldSize = 1280.0f;
    data.minBatchSize = 17;
    data.maxBatchSize = 33;
    data.inputImage = &heights;
    data.inputScale = 50.0f;
    data.maxPixelError = 0.05f;
    data.layerBlendMapSize = 129;
    data.lightMapSize = 129;
    data.compositeMapSize = 64;
    Terrain3D* terrain = makePage(scene, false);
    CHECK(terrain->load(data));
    CHECK(terrain->addLayer(100.0f, white, TextureId()));
    terrain->setGlobalColourMapEnabled(true, 128);
    CHECK(terrain->setGlobalColourMap(colour));

    scene.camera->setPerspective(20.0f, 1.0f, 5000.0f);
    int high = 0;
    int highRed = 0;
    int low = 0;
    int lowBlue = 0;
    float highX = 0.0f;
    float highZ = 0.0f;
    for (int j = 0; j < 21; ++j)
    {
        for (int i = 0; i < 21; ++i)
        {
            const float x = -620.0f + 62.0f * static_cast<float>(i);
            const float z = -620.0f + 62.0f * static_cast<float>(j);
            const float h = terrain->getWorldHeightAt(Math::Vec3(x, 0.0f, z));
            scene.camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(x, h + 300.0f, z),
                    Math::Vec3(x, h, z), Math::Vec3(0.0f, 0.0f, -1.0f))
                                                     .InverseAffine());
            Picture picture;
            CHECK(renderPicture(scene, picture));
            const unsigned char* c = picture.at(kWidth / 2, kHeight / 2);
            const bool red = c[0] > 150 && c[2] < 100;
            const bool blue = c[2] > 150 && c[0] < 100;
            if (h > 40.0f)
            {
                ++high;
                highX += x;
                highZ += z;
                if (red) ++highRed;
            }
            else if (h < 10.0f)
            {
                ++low;
                if (blue) ++lowBlue;
            }
        }
    }
    printf("  height map against colour map: %d of %d high points red, %d of %d low points blue\n",
            highRed, high, lowBlue, low);
    CHECK(high > 0 && highRed == high);
    CHECK(low > 0 && lowBlue == low);
    CHECK(highX / static_cast<float>(high) < 0.0f && highZ / static_cast<float>(high) < 0.0f);

    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(white);
    scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);
}

void testComposite(Scene& scene)
{
    const TextureId red = solid(255, 0, 0, true, "red");
    const TextureId green = solid(0, 255, 0, true, "green");
    const TextureId flat = solid(128, 128, 255, false, "flat normal");
    ct::Vector<float> heights = flatHeights(129);

    Terrain3D* terrain = makePage(scene, true);
    CHECK(terrain->load(pageData(heights.data())));
    CHECK(terrain->addLayer(100.0f, red, flat));
    CHECK(terrain->addLayer(100.0f, green, flat));
    terrain->setCompositeMapLighting(Math::Vec3(1.0f, 1.0f, 1.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
    paintQuadrants(terrain->getLayerBlendMap(1));
    terrain->getTerrain().updateCompositeMap();
    CHECK(terrain->getCompositeMapTexture().valid() && terrain->getCompositeMaterial().valid());

    place(scene, Math::Vec3(0.0f, 300.0f, 0.01f));
    Picture near;
    terrain->setCompositeMapDistance(10000.0f);
    CHECK(renderPicture(scene, near));
    CHECK(terrain->getNodesDrawnComposite() == 0 && terrain->getNodesDrawn() == 1);

    Picture far;
    terrain->setCompositeMapDistance(10.0f);
    const std::uint32_t bakesBefore = terrain->getCompositeBakes();
    CHECK(renderPicture(scene, far));
    CHECK(terrain->getNodesDrawnComposite() == 1);
    CHECK(terrain->isNodeComposite(terrain->getTerrain().getQuadTree()));
    printf("  composite bakes %u (%llu texels)\n", terrain->getCompositeBakes(),
            static_cast<unsigned long long>(terrain->getCompositeBakeTexels()));
    CHECK(terrain->getCompositeBakes() >= bakesBefore);
    const int differing = compositeRegionDiffers(near, far);
    printf("  composite against near shading, ambient only: %d of %d pixels differ by more than "
           "10\n",
            differing, kWidth * kHeight);
    CHECK(differing < 400);
    CHECK(pixelIs(far.at(kWidth / 2 - 10, kHeight / 2 - 10), 255, 0, 0));
    CHECK(pixelIs(far.at(kWidth / 2 + 10, kHeight / 2 - 10), 0, 255, 0));
    CHECK(pixelIs(far.at(kWidth / 2 + 10, kHeight / 2 + 10), 255, 0, 0));
    CHECK(pixelIs(far.at(kWidth / 2 - 10, kHeight / 2 + 10), 255, 0, 0));

    TerrainLayerBlendMap* blend = terrain->getLayerBlendMap(1);
    for (int y = 80; y < 120; ++y)
        for (int x = 80; x < 120; ++x) blend->setBlendValue(x, y, 1.0f);
    blend->update();
    terrain->getTerrain().updateCompositeMap();
    const std::uint64_t before = terrain->getCompositeBakeTexels();
    const std::uint32_t bakes = terrain->getCompositeBakes();
    Picture after;
    CHECK(renderPicture(scene, after));
    const std::uint64_t texels = terrain->getCompositeBakeTexels() - before;
    printf("  after painting 40x40 blend texels: composite re-baked %u time(s), %llu texels of "
           "%d\n",
            terrain->getCompositeBakes() - bakes, static_cast<unsigned long long>(texels), 64 * 64);
    CHECK(terrain->getCompositeBakes() == bakes + 1 && texels > 0 && texels < 64u * 64u / 2u);
    const int changed = compositeRegionDiffers(far, after);
    CHECK(changed > 20 && changed < 800);
    CHECK(pixelIs(after.at(kWidth / 2 - 10, kHeight / 2 - 10), 255, 0, 0));
    CHECK(pixelIs(after.at(kWidth / 2 + 10, kHeight / 2 + 10), 0, 255, 0));

    terrain->setCompositeMapDistance(10000.0f);
    Picture nearAfter;
    CHECK(renderPicture(scene, nearAfter));
    CHECK(compositeRegionDiffers(nearAfter, after) < 400);

    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(red);
    Main::textures()->destroy(green);
    Main::textures()->destroy(flat);
}

float hill(int x, int y)
{
    return 5.0f * sinf(static_cast<float>(x) * 0.07f) * cosf(static_cast<float>(y) * 0.05f) +
           3.0f * sinf(static_cast<float>(x + y) * 0.12f);
}

void testNormalMapLighting(Scene& scene, WorldEnvironment* environment)
{
    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x) heights[static_cast<std::size_t>(y) * 129 + x] = hill(x, y);

    MaterialStore& materials = *Main::materials();
    const MaterialId grey = materials.create("grey");
    materials.setColor(grey, MaterialParam::Albedo, Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    materials.setFloat(grey, MaterialParam::Roughness, 1.0f);
    const MaterialId plain = materials.duplicate(grey, "plain grey");

    EnvironmentSettings settings;
    settings.ambientSource = AmbientSource::Color;
    settings.ambientColor = Math::Vec3(0.15f, 0.15f, 0.15f);
    settings.reflectionSource = ReflectionSource::Disabled;
    environment->setEnvironment(settings);

    DirectionalLight3D* sun = scene.root->createChild<DirectionalLight3D>();
    sun->setShadow(false);
    sun->setRotation(Math::Vec3(-1.0f, 0.6f, 0.0f));

    Terrain3D* terrain = makePage(scene, false);
    terrain->setMaterial(grey);
    terrain->setVisible(false);
    CHECK(terrain->load(pageData(heights.data())));

    SurfaceData data;
    for (int y = 0; y < 129; ++y)
    {
        for (int x = 0; x < 129; ++x)
        {
            Math::Vec3 p;
            terrain->getTerrain().getPoint(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y), p);
            data.positions.push_back(p);
            data.normals.push_back(terrain->getTerrain().calculateNormal(x, y));
            data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            data.uvs.push_back(Math::Vec2(x / 128.0f, 1.0f - y / 128.0f));
        }
    }
    for (int y = 0; y < 128; ++y)
    {
        for (int x = 0; x < 128; ++x)
        {
            const std::uint32_t i = static_cast<std::uint32_t>(y * 129 + x);
            const std::uint32_t quad[6] = { i, i + 1, i + 129, i + 1, i + 130, i + 129 };
            for (std::uint32_t index: quad) data.indices.push_back(index);
        }
    }
    const MeshId mesh = Main::meshes()->create("reference terrain");
    CHECK(Main::meshes()->addSurface(mesh, data));
    MeshInstance3D* reference = scene.root->createChild<MeshInstance3D>();
    reference->setMesh(mesh);
    reference->setMaterialOverride(plain);

    place(scene, Math::Vec3(0.0f, 80.0f, 0.01f));
    Picture mapPicture;
    Picture vertexPicture;
    terrain->setVisible(true);
    reference->setVisible(false);
    terrain->setMaxPixelError(0.0001f);
    CHECK(renderPicture(scene, mapPicture));
    CHECK(terrain->getNodesDrawn() == 1);
    terrain->setVisible(false);
    reference->setVisible(true);
    CHECK(renderPicture(scene, vertexPicture));

    long sum = 0;
    int worst = 0;
    int covered = 0;
    for (int y = 10; y < kHeight - 10; ++y)
    {
        for (int x = 10; x < kWidth - 10; ++x)
        {
            const unsigned char* a = mapPicture.at(x, y);
            const unsigned char* b = vertexPicture.at(x, y);
            const int d = abs(a[1] - b[1]);
            sum += d;
            if (d > worst) worst = d;
            ++covered;
        }
    }
    const double mean = static_cast<double>(sum) / covered;
    printf("  normal map against per-vertex normals on a smooth hill: mean difference %.2f, worst "
           "%d (of 255), centre %d against %d\n",
            mean, worst, mapPicture.at(kWidth / 2, kHeight / 2)[1],
            vertexPicture.at(kWidth / 2, kHeight / 2)[1]);
    CHECK(mean < 1.5);
    CHECK(worst < 12);
    CHECK(mapPicture.at(kWidth / 2, kHeight / 2)[1] > 40);

    sun->queueFree();
    reference->queueFree();
    terrain->queueFree();
    flush(scene);
    Main::meshes()->destroy(mesh);
    materials.destroy(grey);
    materials.destroy(plain);
    environment->setEnvironment(whiteAmbient());
}

void testCompositeLit(Scene& scene, WorldEnvironment* environment)
{
    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x) heights[static_cast<std::size_t>(y) * 129 + x] = hill(x, y);
    const TextureId white = solid(200, 200, 200, true, "light grey");
    const TextureId flat = solid(128, 128, 255, false, "flat normal");

    EnvironmentSettings settings;
    settings.ambientSource = AmbientSource::Color;
    settings.ambientColor = Math::Vec3(0.15f, 0.15f, 0.15f);
    settings.reflectionSource = ReflectionSource::Disabled;
    environment->setEnvironment(settings);
    DirectionalLight3D* sun = scene.root->createChild<DirectionalLight3D>();
    sun->setShadow(false);
    sun->setRotation(Math::Vec3(-1.0f, 0.6f, 0.0f));
    {
        Frame frame(*scene.tree);
    }
    const Math::Vec3 forward =
            sun->getGlobalTransform().TransformVector(Math::Vec3(0.0f, 0.0f, -1.0f));

    Terrain3D* terrain = makePage(scene, true);
    terrain->setLightMapDirection(forward);
    terrain->setCompositeMapLighting(Math::Vec3(0.15f, 0.15f, 0.15f), Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(terrain->load(pageData(heights.data())));
    CHECK(terrain->addLayer(100.0f, white, flat));
    terrain->getTerrain().updateCompositeMap();
    terrain->setMaxPixelError(0.0001f);

    place(scene, Math::Vec3(0.0f, 300.0f, 0.01f));
    Picture nearPicture;
    Picture farPicture;
    terrain->setCompositeMapDistance(100000.0f);
    CHECK(renderPicture(scene, nearPicture));
    terrain->setCompositeMapDistance(10.0f);
    const float diffuseValues[6] = { 0.2f, 0.3f, 0.4f, 0.5f, 0.7f, 1.0f };
    double bestMean = 1.0e9;
    for (int k = 0; k < 6; ++k)
    {
        terrain->setCompositeMapLighting(Math::Vec3(0.15f, 0.15f, 0.15f),
                Math::Vec3(diffuseValues[k], diffuseValues[k], diffuseValues[k]));
        CHECK(renderPicture(scene, farPicture));
        CHECK(terrain->getNodesDrawnComposite() == 1);
        long sum = 0;
        long nearSum = 0;
        long farSum = 0;
        int worst = 0;
        int covered = 0;
        for (int y = 40; y < 80; ++y)
        {
            for (int x = 60; x < 100; ++x)
            {
                const int a = nearPicture.at(x, y)[1];
                const int b = farPicture.at(x, y)[1];
                sum += abs(a - b);
                nearSum += a;
                farSum += b;
                if (abs(a - b) > worst) worst = abs(a - b);
                ++covered;
            }
        }
        const double mean = static_cast<double>(sum) / covered;
        printf("  composite diffuse %.2f against near shading under the sun (hill, ambient "
               "0.15): mean near %.1f, mean composite %.1f, mean difference %.2f, worst %d\n",
                diffuseValues[k], static_cast<double>(nearSum) / covered,
                static_cast<double>(farSum) / covered, mean, worst);
        if (mean < bestMean) bestMean = mean;
    }
    CHECK(bestMean < 20.0);

    sun->queueFree();
    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(white);
    Main::textures()->destroy(flat);
    environment->setEnvironment(whiteAmbient());
}

bool loadPixmap(const char* name, Pixmap& out)
{
    const ct::String path = ct::String(PRISMA3D_ASSETS_DIR) + "/textures/terrain/" + name;
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    return !bytes.empty() &&
           out.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size()));
}

void testSampleTextures(Scene& scene)
{
    TextureStore& textures = *Main::textures();
    const ct::String dir = ct::String(PRISMA3D_ASSETS_DIR) + "/textures/terrain/";
    const char* const diffuse[2] = { "Ground37_diffspec.dds", "Rock20_diffspec.dds" };
    const char* const heights[3] = { "Ground37_normheight.dds", "Ground23_normheight.dds",
        "Rock20_normheight.dds" };
    TextureId loaded[5];
    for (int i = 0; i < 2; ++i) loaded[i] = textures.createFromFile(dir + diffuse[i], true, true);
    for (int i = 0; i < 3; ++i)
        loaded[2 + i] = textures.createFromFile(dir + heights[i], false, true);
    for (int i = 0; i < 5; ++i)
    {
        CHECK(loaded[i].valid());
        const TextureInfo* info = textures.get(loaded[i]);
        CHECK(info && info->width == 1024 && info->height == 1024 && info->mipLevels == 11);
        if (info && i == 0)
            printf("  sample layer texture: %ux%u, %u mips, format %d (BC1..BC7 are %d..%d)\n",
                    info->width, info->height, info->mipLevels, static_cast<int>(info->format),
                    static_cast<int>(prisma::TextureFormat::BC1),
                    static_cast<int>(prisma::TextureFormat::BC7Srgb));
    }

    Pixmap colour;
    Pixmap specular;
    CHECK(loadPixmap("Ground23_col.jpg", colour) && loadPixmap("Ground23_spec.png", specular));
    CHECK(colour.width == 1024 && specular.width == 1024 && specular.height == 1024);
    Pixmap combined(1024, 1024, 4);
    for (int y = 0; y < 1024; ++y)
    {
        for (int x = 0; x < 1024; ++x)
        {
            const std::uint8_t* c =
                    &colour.pixels[(static_cast<std::size_t>(y) * 1024 + x) * colour.components];
            std::uint8_t* out = &combined.pixels[(static_cast<std::size_t>(y) * 1024 + x) * 4];
            out[0] = c[0];
            out[1] = colour.components > 1 ? c[1] : c[0];
            out[2] = colour.components > 2 ? c[2] : c[0];
            out[3] =
                    specular.pixels[(static_cast<std::size_t>(y) * 1024 + x) * specular.components];
        }
    }
    const TextureId ground23 = textures.createFromPixmap(combined, true, true);
    CHECK(ground23.valid());

    ct::Vector<float> hills;
    hills.resize(129u * 129u);
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x)
            hills[static_cast<std::size_t>(y) * 129 + x] =
                    40.0f + 40.0f * sinf(static_cast<float>(x) * 0.08f);
    Terrain3D* terrain = makePage(scene, false);
    terrain->setLayerParallax(1);
    CHECK(terrain->load(pageData(hills.data())));
    CHECK(terrain->addLayer(40.0f, loaded[0], loaded[2]));
    CHECK(terrain->addLayer(40.0f, ground23, loaded[3]));
    CHECK(terrain->addLayer(80.0f, loaded[1], loaded[4]));
    TerrainLayerBlendMap* blend0 = terrain->getLayerBlendMap(1);
    TerrainLayerBlendMap* blend1 = terrain->getLayerBlendMap(2);
    CHECK(blend0 && blend1);
    float* p0 = blend0->getBlendPointer();
    float* p1 = blend1->getBlendPointer();
    for (int y = 0; y < 129; ++y)
    {
        for (int x = 0; x < 129; ++x)
        {
            float tx = 0.0f;
            float ty = 0.0f;
            blend0->convertImageToTerrainSpace(static_cast<std::size_t>(x),
                    static_cast<std::size_t>(y), &tx, &ty);
            const float h = terrain->getTerrain().getHeightAtTerrainPosition(tx, ty);
            float a = (h - 20.0f) / 15.0f;
            float b = (h - 70.0f) / 15.0f;
            a = a < 0.0f ? 0.0f : (a > 1.0f ? 1.0f : a);
            b = b < 0.0f ? 0.0f : (b > 1.0f ? 1.0f : b);
            p0[y * 129 + x] = a;
            p1[y * 129 + x] = b;
        }
    }
    blend0->dirty();
    blend1->dirty();
    blend0->update();
    blend1->update();

    place(scene, Math::Vec3(0.0f, 120.0f, 0.01f));
    Picture picture;
    CHECK(renderPicture(scene, picture));
    long sum[3] = { 0, 0, 0 };
    int distinct = 0;
    for (int x = 10; x < kWidth - 10; x += 10)
    {
        const unsigned char* p = picture.at(x, kHeight / 2);
        sum[0] += p[0];
        sum[1] += p[1];
        sum[2] += p[2];
        const unsigned char* q = picture.at(x + 10 < kWidth ? x + 10 : x, kHeight / 2);
        if (abs(p[0] - q[0]) + abs(p[1] - q[1]) + abs(p[2] - q[2]) > 12) ++distinct;
    }
    printf("  three sample layers on a ridge (height blend): mean row colour %ld %ld %ld, %d "
           "changes along the row\n",
            sum[0] / 15, sum[1] / 15, sum[2] / 15, distinct);
    CHECK(sum[0] + sum[1] + sum[2] > 15 * 3 * 20);
    CHECK(distinct > 3);

    terrain->queueFree();
    flush(scene);
    for (int i = 0; i < 5; ++i) textures.destroy(loaded[i]);
    textures.destroy(ground23);
}

void measure(Scene& scene)
{
    TextureStore& textures = *Main::textures();
    const TextureId red = solid(255, 0, 0, true, "red");
    const TextureId flat = solid(128, 128, 255, false, "flat normal");
    const int size = 513;
    ct::Vector<float> heights;
    heights.resize(static_cast<std::size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            heights[static_cast<std::size_t>(y) * size + x] =
                    60.0f + 40.0f * sinf(static_cast<float>(x) * 0.03f) *
                                    cosf(static_cast<float>(y) * 0.02f);

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLayerParallax(0);
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = 33;
    data.maxBatchSize = 65;
    data.worldSize = 12000.0f;
    data.inputFloat = heights.data();
    data.lightMapSize = 512;
    data.lightMapDirection = Math::Vec3(0.55f, -0.3f, 0.75f).Normalized();
    double t0 = nowMilliseconds();
    CHECK(terrain->load(data));
    printf("  load with normal map, lightmap 512 and composite map: %.0f ms\n",
            nowMilliseconds() - t0);
    CHECK(terrain->addLayer(200.0f, red, flat));
    CHECK(terrain->addLayer(200.0f, red, flat));
    CHECK(terrain->addLayer(400.0f, red, flat));
    terrain->setGlobalColourMapEnabled(true);
    printf("  GPU texture memory of the derived maps: %.2f MB (normal map, lightmap, colour map, "
           "composite map, blend map)\n",
            static_cast<double>(terrain->getTextureMemoryBytes()) / (1024.0 * 1024.0));

    place(scene, Math::Vec3(1000.0f, 400.0f, 7000.0f));
    scene.camera->setPerspective(60.0f, 0.5f, 30000.0f);
    {
        Frame prime(*scene.tree);
    }

    terrain->getTerrain()._dirtyCompositeMapRect(TerrainRect(0, 0, size, size));
    terrain->getTerrain().updateCompositeMap();
    t0 = nowMilliseconds();
    CHECK(terrain->flushCompositeMap());
    Picture picture;
    CHECK(readPicture(picture));
    printf("  composite map 1024x1024 bake, whole terrain, wall clock with a read-back: %.2f ms\n",
            nowMilliseconds() - t0);
    Main::rendering()->endFrame(true);

    const std::uint64_t texelsBefore = terrain->getCompositeBakeTexels();
    terrain->getTerrain()._dirtyCompositeMapRect(TerrainRect(200, 200, 216, 216));
    terrain->getTerrain().updateCompositeMap();
    Main::rendering()->openFrame();
    t0 = nowMilliseconds();
    CHECK(terrain->flushCompositeMap());
    CHECK(readPicture(picture));
    printf("  composite map bake, 16x16 point rectangle (%llu texels), wall clock: %.2f ms\n",
            static_cast<unsigned long long>(terrain->getCompositeBakeTexels() - texelsBefore),
            nowMilliseconds() - t0);
    Main::rendering()->endFrame(true);

    terrain->queueFree();
    flush(scene);
    textures.destroy(red);
    textures.destroy(flat);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "terrain layers test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    Window* window = root->castTo<Window>();
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

    WorldEnvironment* environment = root->createChild<WorldEnvironment>();
    environment->setEnvironment(whiteAmbient());

    Scene scene;
    scene.tree = &tree;
    scene.root = root;
    scene.camera = root->createChild<Camera3D>();
    scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);
    scene.camera->makeCurrent();
    Main::materials()->defaultMaterial();
    Main::textures()->white();
    Main::textures()->black();
    Main::textures()->normal();
    {
        Frame frame(tree);
    }

    testLayersBlend(scene);
    testLayerScale(scene);
    testColourMap(scene);
    testColourMapDetail(scene);
    testDetailModulation(scene);
    testAlignment(scene);
    testComposite(scene);
    testNormalMapLighting(scene, environment);
    testCompositeLit(scene, environment);
    testSampleTextures(scene);
    measure(scene);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("terrain layers ok\n");
    return 0;
}
