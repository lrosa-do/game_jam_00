#include "Check.h"
#include "ColouredBox.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include "platform.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 64;

int gErrors = 0;

void onLog(LogLevel level, const char* message, void* user)
{
    (void) user;
    if (level < LOGLEVEL_ERROR) return;
    ++gErrors;
    printf("logged error: %s\n", message);
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

struct Picture
{
    ct::Vector<std::uint8_t> pixels;
    RenderStats stats;

    int value(int x, int y) const
    {
        return pixels[(static_cast<std::size_t>(y) * kSize + static_cast<std::size_t>(x)) * 4];
    }

    const std::uint8_t* at(int x, int y) const
    {
        return &pixels[(static_cast<std::size_t>(y) * kSize + static_cast<std::size_t>(x)) * 4];
    }
};

bool readPicture(Picture& picture)
{
    picture.pixels.resize(static_cast<std::size_t>(kSize) * kSize * 4);
    prisma::Rect rect;
    rect.width = kSize;
    rect.height = kSize;
    return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect,
            picture.pixels.data());
}

struct Context
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Window* window = nullptr;
    Camera3D* camera = nullptr;
    WorldEnvironment* environment = nullptr;
    MeshStore* meshes = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
};

void drawDebugBatch();

Picture shoot(Context& c, ViewportMsaa msaa, int primes = 1, bool batch = false)
{
    c.window->setMsaa3D(msaa);
    for (int i = 0; i < primes; ++i)
    {
        if (batch) drawDebugBatch();
        Frame frame(*c.tree);
    }
    Picture picture;
    if (batch) drawDebugBatch();
    Frame frame(*c.tree);
    picture.stats = Main::rendering()->getStats();
    CHECK(readPicture(picture));
    return picture;
}

float readDepth(Context& c, int x, int top)
{
    float depth = -1.0f;
    CHECK(Main::rendering()->viewportReadDepth(c.window->getViewportId(), x, kSize - 1 - top,
            depth));
    Main::rendering()->endFrame(true);
    return depth;
}

struct Histogram
{
    int exactBlack = 0;
    int exactWhite = 0;
    int intermediate = 0;
    int distinct = 0;
    bool seen[256] = {};
};

Histogram histogram(const Picture& picture)
{
    Histogram h;
    for (int y = 0; y < kSize; ++y)
    {
        for (int x = 0; x < kSize; ++x)
        {
            const std::uint8_t* p = picture.at(x, y);
            CHECK(p[0] == p[1] && p[1] == p[2]);
            if (p[0] == 0) ++h.exactBlack;
            else if (p[0] == 255)
                ++h.exactWhite;
            else
            {
                ++h.intermediate;
                if (!h.seen[p[0]])
                {
                    h.seen[p[0]] = true;
                    ++h.distinct;
                }
            }
        }
    }
    return h;
}

SurfaceData triangleSurface()
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-32.0f, 32.0f, 0.0f));
    data.positions.push_back(Math::Vec3(-32.0f, -31.95f, 0.0f));
    data.positions.push_back(Math::Vec3(31.95f, 32.0f, 0.0f));
    for (int i = 0; i < 3; ++i) data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    const std::uint32_t indices[3] = { 0, 1, 2 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

SurfaceData quadSurface(float half)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, half, 0.0f));
    data.positions.push_back(Math::Vec3(-half, half, 0.0f));
    for (int i = 0; i < 4; ++i) data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

MaterialId blackMaterial(Context& c)
{
    const MaterialId material = c.materials->create("black");
    c.materials->setColor(material, MaterialParam::Albedo, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    c.materials->setShading(material, MaterialShading::Unshaded);
    c.materials->setCull(material, MaterialCull::Disabled);
    return material;
}

void placeOrthogonal(Context& c)
{
    c.camera->setOrthogonal(static_cast<float>(kSize), 0.1f, 100.0f);
    c.camera->setPosition(Math::Vec3(0.0f, 0.0f, 10.0f));
}

void testTriangleEdge(Context& c)
{
    printf("triangle edge\n");
    placeOrthogonal(c);
    c.window->setClearColor(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    const MeshId mesh = c.meshes->create("triangle");
    CHECK(c.meshes->addSurface(mesh, triangleSurface()));
    MeshInstance3D* instance = c.root->createChild<MeshInstance3D>();
    instance->setMesh(mesh);
    instance->setMaterialOverride(blackMaterial(c));

    const Picture off = shoot(c, ViewportMsaa::Disabled);
    CHECK(c.window->getMsaaSamples() == 1);
    const Histogram flat = histogram(off);
    printf("  off: %d black, %d white, %d intermediate\n", flat.exactBlack, flat.exactWhite,
            flat.intermediate);
    CHECK(flat.intermediate == 0);
    const int expectedBlack = 63 * 64 / 2;
    CHECK(flat.exactBlack == expectedBlack);
    const float flatDepth = readDepth(c, 10, 10);
    printf("  off: depth inside %.9f\n", flatDepth);
    CHECK(flatDepth > 0.0f && flatDepth < 1.0f);
    CHECK(readDepth(c, 60, 60) == 1.0f);

    const std::uint32_t maxSamples = Main::device()->driver()->caps().maxSamples;
    const ViewportMsaa levels[3] = { ViewportMsaa::Msaa2x, ViewportMsaa::Msaa4x,
        ViewportMsaa::Msaa8x };
    const std::uint32_t counts[3] = { 2, 4, 8 };
    for (int level = 0; level < 3; ++level)
    {
        if (maxSamples < counts[level])
        {
            printf("  %ux is not supported by this driver (maximum %u)\n", counts[level],
                    maxSamples);
            continue;
        }
        const Picture on = shoot(c, levels[level]);
        CHECK(c.window->getMsaaSamples() == counts[level]);
        const Histogram h = histogram(on);
        int exactEdge = 0;
        int interiorWrong = 0;
        for (int y = 0; y < kSize; ++y)
        {
            for (int x = 0; x < kSize; ++x)
            {
                const int sum = x + y;
                const int v = on.value(x, y);
                if (sum <= 62 && v != 0) ++interiorWrong;
                if (sum >= 64 && v != 255) ++interiorWrong;
                if (sum == 63 && v >= 180 && v <= 196) ++exactEdge;
            }
        }
        printf("  %ux: %d black, %d white, %d intermediate, %d on the edge near 188, %d interior "
               "pixels not exact\n",
                counts[level], h.exactBlack, h.exactWhite, h.intermediate, exactEdge,
                interiorWrong);
        CHECK(interiorWrong == 0);
        CHECK(h.exactBlack + h.exactWhite + h.intermediate == kSize * kSize);
        if (counts[level] >= 4) CHECK(h.intermediate >= 62 && h.intermediate <= 66);
        if (counts[level] == 4) CHECK(exactEdge >= 62 && exactEdge <= 66);

        const float depth = readDepth(c, 10, 10);
        CHECK(memcmp(&depth, &flatDepth, sizeof(float)) == 0);
        CHECK(readDepth(c, 60, 60) == 1.0f);
        for (int x = 0; x < kSize; ++x)
        {
            const float edge = readDepth(c, x, 63 - x);
            CHECK(edge == 1.0f || memcmp(&edge, &flatDepth, sizeof(float)) == 0);
        }
    }

    instance->queueFree();
    {
        Frame frame(*c.tree);
    }
}

TextureId gradientTexture(Context& c)
{
    Pixmap pixels(128, 128, 4);
    for (std::uint32_t y = 0; y < 128; ++y)
    {
        for (std::uint32_t x = 0; x < 128; ++x)
        {
            const float u = (static_cast<float>(x) + 0.5f) / 128.0f;
            const float v = (static_cast<float>(y) + 0.5f) / 128.0f;
            const float alpha = u * 0.8f + v * 0.2f;
            pixels.set_pixel(x, y, 255, 255, 255, static_cast<std::uint8_t>(alpha * 255.0f + 0.5f));
        }
    }
    return c.textures->createFromPixmap(pixels, false, false, "gradient");
}

void testScissorCoverage(Context& c)
{
    printf("alpha scissor and coverage\n");
    placeOrthogonal(c);
    c.window->setClearColor(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    const MeshId mesh = c.meshes->create("quad");
    CHECK(c.meshes->addSurface(mesh, quadSurface(32.0f)));
    const MaterialId material = blackMaterial(c);
    c.materials->setTexture(material, MaterialTexture::Albedo, gradientTexture(c));
    c.materials->setTransparency(material, MaterialTransparency::AlphaScissor);
    c.materials->setFloat(material, MaterialParam::AlphaScissorThreshold, 0.3f);
    MeshInstance3D* instance = c.root->createChild<MeshInstance3D>();
    instance->setMesh(mesh);
    instance->setMaterialOverride(material);

    const Picture plain = shoot(c, ViewportMsaa::Disabled);
    const Histogram flat = histogram(plain);
    printf("  off, coverage mode off: %d black, %d white, %d intermediate\n", flat.exactBlack,
            flat.exactWhite, flat.intermediate);
    CHECK(flat.intermediate == 0 && flat.exactBlack > 100 && flat.exactWhite > 100);
    CHECK(plain.stats.coverageSurfaces == 0);

    c.materials->setAlphaAntialiasing(material, MaterialAlphaAntialiasing::AlphaToCoverage, 0.3f);
    const Picture offFlagged = shoot(c, ViewportMsaa::Disabled);
    std::size_t different = 0;
    for (std::size_t i = 0; i < plain.pixels.size(); ++i)
        if (plain.pixels[i] != offFlagged.pixels[i]) ++different;
    printf("  off, coverage mode on: %zu bytes differ from coverage mode off\n", different);
    CHECK(different == 0);
    CHECK(offFlagged.stats.coverageSurfaces == 0);

    if (Main::device()->driver()->caps().maxSamples >= 4)
    {
        c.materials->setAlphaAntialiasing(material, MaterialAlphaAntialiasing::Off);
        const Picture msaaPlain = shoot(c, ViewportMsaa::Msaa4x);
        const Histogram hard = histogram(msaaPlain);
        printf("  4x, coverage mode off: %d black, %d white, %d intermediate\n", hard.exactBlack,
                hard.exactWhite, hard.intermediate);
        CHECK(hard.intermediate == 0 && hard.exactBlack > 100 && hard.exactWhite > 100);
        CHECK(msaaPlain.stats.coverageSurfaces == 0);
        Main::rendering()->setDepthPrePassEnabled(false);
        const Picture noPrepass = shoot(c, ViewportMsaa::Msaa4x);
        Main::rendering()->setDepthPrePassEnabled(true);
        std::size_t prepassDifferent = 0;
        for (std::size_t i = 0; i < msaaPlain.pixels.size(); ++i)
            if (msaaPlain.pixels[i] != noPrepass.pixels[i]) ++prepassDifferent;
        printf("  4x, %zu bytes differ without the pre-pass\n", prepassDifferent);
        CHECK(prepassDifferent == 0);
        CHECK(noPrepass.stats.depthPrePassSurfaces == 0 &&
                msaaPlain.stats.depthPrePassSurfaces == 1);

        c.materials->setAlphaAntialiasing(material, MaterialAlphaAntialiasing::AlphaToCoverage,
                0.3f);
        const Picture soft = shoot(c, ViewportMsaa::Msaa4x);
        const Histogram h = histogram(soft);
        printf("  4x, coverage mode on: %d black, %d white, %d intermediate, %d distinct levels\n",
                h.exactBlack, h.exactWhite, h.intermediate, h.distinct);
        CHECK(h.distinct >= 3);
        CHECK(h.intermediate >= 20);
        CHECK(h.exactBlack > 100 && h.exactWhite > 100);
        CHECK(soft.stats.coverageSurfaces == 1);
        CHECK(soft.stats.depthPrePassSurfaces == 1);

        c.materials->setAlphaAntialiasing(material, MaterialAlphaAntialiasing::AlphaToCoverage,
                0.0f);
        const Picture edgeless = shoot(c, ViewportMsaa::Msaa4x);
        const Histogram e = histogram(edgeless);
        printf("  4x, coverage mode on, edge 0: %d black, %d white, %d intermediate\n",
                e.exactBlack, e.exactWhite, e.intermediate);
        CHECK(e.exactBlack > 100 && e.exactWhite > 100);

        c.materials->setReceiveShadows(material, false);
        const Picture noShadow = shoot(c, ViewportMsaa::Msaa4x);
        CHECK(noShadow.stats.coverageSurfaces == 1);
        c.materials->setReceiveShadows(material, true);
        const float depth = readDepth(c, 60, 60);
        CHECK(depth > 0.0f && depth <= 1.0f);
    }

    instance->queueFree();
    {
        Frame frame(*c.tree);
    }
}

void testDepthReadback(Context& c)
{
    printf("depth read-back\n");
    placeOrthogonal(c);
    c.window->setClearColor(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    const MeshId mesh = c.meshes->create("quad");
    CHECK(c.meshes->addSurface(mesh, quadSurface(20.0f)));
    MeshInstance3D* instance = c.root->createChild<MeshInstance3D>();
    instance->setMesh(mesh);
    instance->setMaterialOverride(blackMaterial(c));
    instance->setPosition(Math::Vec3(0.0f, 0.0f, -30.0f));

    shoot(c, ViewportMsaa::Disabled);
    const float near1 = readDepth(c, 32, 32);
    const float corner1 = readDepth(c, 1, 1);
    printf("  off: centre %.9f, corner %.9f\n", near1, corner1);
    CHECK(near1 > 0.0f && near1 < 1.0f);
    CHECK(corner1 == 1.0f);
    if (Main::device()->driver()->caps().maxSamples >= 4)
    {
        shoot(c, ViewportMsaa::Msaa4x);
        const float near4 = readDepth(c, 32, 32);
        const float corner4 = readDepth(c, 1, 1);
        printf("  4x: centre %.9f, corner %.9f\n", near4, corner4);
        CHECK(memcmp(&near4, &near1, sizeof(float)) == 0);
        CHECK(corner4 == 1.0f);
    }
    instance->queueFree();
    {
        Frame frame(*c.tree);
    }
}

struct World
{
    Terrain3D* terrain = nullptr;
    Forest3D* forest = nullptr;
    Grass3D* grass = nullptr;
    ParticleSystem3D* particles = nullptr;
    Decal3D* decal = nullptr;
};

World buildWorld(Context& c)
{
    World world;
    c.window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
    c.camera->setPerspective(60.0f, 0.1f, 300.0f);
    c.camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(0.0f, 3.0f, 8.0f),
            Math::Vec3(0.0f, 1.0f, -12.0f), Math::Vec3(0.0f, 1.0f, 0.0f))
                                         .InverseAffine());

    EnvironmentSettings environment;
    environment.background = BackgroundMode::Sky;
    environment.ambientSource = AmbientSource::Sky;
    environment.reflectionSource = ReflectionSource::Sky;
    environment.fogEnabled = true;
    environment.fogDensity = 0.01f;
    c.environment->setEnvironment(environment);

    DirectionalLight3D* sun = c.root->createChild<DirectionalLight3D>();
    sun->setShadow(true);
    sun->setShadowMode(DirectionalShadowMode::Parallel2Splits);
    sun->setShadowMaxDistance(60.0f);
    sun->lookAtFromPosition(Math::Vec3(8.0f, 16.0f, 6.0f), Math::Vec3(0.0f, 0.0f, 0.0f));

    const MaterialId green = c.materials->create("ground");
    c.materials->setColor(green, MaterialParam::Albedo, Math::Vec4(0.2f, 0.5f, 0.2f, 1.0f));
    c.materials->setFloat(green, MaterialParam::Roughness, 0.9f);
    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = 0.0f;
    world.terrain = c.root->createChild<Terrain3D>();
    world.terrain->setLightmapEnabled(false);
    world.terrain->setCompositeMapEnabled(false);
    world.terrain->setMaterial(green);
    TerrainImportData data;
    data.terrainSize = 129;
    data.minBatchSize = 129;
    data.maxBatchSize = 129;
    data.worldSize = 128.0f;
    data.inputFloat = heights.data();
    data.layerBlendMapSize = 129;
    data.lightMapSize = 129;
    data.compositeMapSize = 64;
    CHECK(world.terrain->load(data));

    const MeshId box = c.meshes->create("box");
    CHECK(c.meshes->addSurface(box, colouredBox()));
    const MaterialId red = c.materials->create("red");
    c.materials->setColor(red, MaterialParam::Albedo, Math::Vec4(1.0f, 0.1f, 0.1f, 1.0f));
    MeshInstance3D* solid = c.root->createChild<MeshInstance3D>();
    solid->setMesh(box);
    solid->setMaterialOverride(red);
    solid->setPosition(Math::Vec3(-1.5f, 0.0f, -6.0f));
    solid->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));

    const MeshId quad = c.meshes->create("quad");
    CHECK(c.meshes->addSurface(quad, quadSurface(1.2f)));
    const MaterialId glass = c.materials->create("glass");
    c.materials->setColor(glass, MaterialParam::Albedo, Math::Vec4(0.3f, 0.5f, 1.0f, 0.5f));
    c.materials->setTransparency(glass, MaterialTransparency::AlphaBlend);
    c.materials->setCull(glass, MaterialCull::Disabled);
    MeshInstance3D* pane = c.root->createChild<MeshInstance3D>();
    pane->setMesh(quad);
    pane->setMaterialOverride(glass);
    pane->setPosition(Math::Vec3(1.0f, 1.5f, -3.0f));

    Pixmap cutPixels(64, 8, 4);
    for (std::uint32_t y = 0; y < 8; ++y)
        for (std::uint32_t x = 0; x < 64; ++x)
            cutPixels.set_pixel(x, y, 40, 200, 40, static_cast<std::uint8_t>(x * 4));
    const TextureId cutTexture = c.textures->createFromPixmap(cutPixels, true, true, "leaf");
    const MaterialId leaf = c.materials->create("leaf");
    c.materials->setTexture(leaf, MaterialTexture::Albedo, cutTexture);
    c.materials->setTransparency(leaf, MaterialTransparency::AlphaScissor);
    c.materials->setFloat(leaf, MaterialParam::AlphaScissorThreshold, 0.4f);
    c.materials->setAlphaAntialiasing(leaf, MaterialAlphaAntialiasing::AlphaToCoverage, 0.3f);
    c.materials->setCull(leaf, MaterialCull::Disabled);
    MeshInstance3D* leaves = c.root->createChild<MeshInstance3D>();
    leaves->setMesh(quad);
    leaves->setMaterialOverride(leaf);
    leaves->setPosition(Math::Vec3(2.5f, 1.4f, -7.0f));

    world.forest = c.root->createChild<Forest3D>();
    world.forest->setSwapDistance(20.0f);
    world.forest->setDrawDistance(100.0f);
    ForestSpeciesDesc species;
    species.mesh = box;
    CHECK(world.forest->addSpecies(species) == 0);
    world.forest->plant(0, Math::Vec3(-4.0f, 0.0f, -10.0f), 0.0f, 1.5f);
    world.forest->plant(0, Math::Vec3(5.0f, 0.0f, -34.0f), 0.0f, 2.0f);
    world.forest->plant(0, Math::Vec3(-6.0f, 0.0f, -40.0f), 0.0f, 2.0f);

    Pixmap grassPixels(16, 16, 4);
    for (std::uint32_t y = 0; y < 16; ++y)
        for (std::uint32_t x = 0; x < 16; ++x)
            grassPixels.set_pixel(x, y, 255, 255, 255,
                    static_cast<std::uint8_t>(255 - 16 * (y < 15 ? y : 15)));
    const TextureId grassTexture =
            c.textures->createFromPixmap(grassPixels, true, true, "blade", 4);
    world.grass = c.root->createChild<Grass3D>();
    world.grass->setTexture(grassTexture);
    world.grass->setSeed(7u);
    world.grass->setCellSize(8.0f);
    world.grass->setDensity(0.5f);
    world.grass->setDrawDistance(30.0f);
    world.grass->setExtent(-30.0f, -30.0f, 30.0f, 30.0f);
    world.grass->setTerrain(world.terrain);
    world.grass->setAlphaAntialiasing(MaterialAlphaAntialiasing::AlphaToCoverage, 0.3f);

    world.particles = c.root->createChild<ParticleSystem3D>(200);
    world.particles->setContinuous(60.0f)
            ->setShapePoint()
            ->setEmissionDirection(Math::Vec3(0.0f, 1.0f, 0.0f))
            ->setSpreadAngle(20.0f)
            ->setSpeed(1.0f, 2.0f)
            ->setLifetime(1.0f, 2.0f)
            ->setSize(Math::Vec2(0.4f, 0.4f), Math::Vec2(0.1f, 0.1f))
            ->setColor(Math::Vec4(1.0f, 0.6f, 0.2f, 0.9f), Math::Vec4(0.5f, 0.1f, 0.05f, 0.0f));
    world.particles->setPosition(Math::Vec3(0.0f, 0.2f, 0.0f));

    Pixmap stainPixels(16, 16, 4);
    for (std::uint32_t y = 0; y < 16; ++y)
        for (std::uint32_t x = 0; x < 16; ++x) stainPixels.set_pixel(x, y, 255, 0, 0, 255);
    world.decal = c.root->createChild<Decal3D>();
    world.decal->setPosition(Math::Vec3(-1.0f, 0.1f, -2.5f));
    world.decal->setSize(Math::Vec3(2.0f, 1.0f, 2.0f));
    world.decal->setTexture(DecalTexture::Albedo,
            c.textures->createFromPixmap(stainPixels, true, false, "stain"));
    return world;
}

void drawDebugBatch()
{
    BatchRenderer& batch = Main::rendering()->batch();
    batch.setDefault3DState();
    batch.setColor(1.0f, 1.0f, 0.0f, 1.0f);
    batch.drawTriangle3D(Math::Vec3(2.0f, 0.2f, -2.0f), Math::Vec3(4.0f, 0.2f, -2.0f),
            Math::Vec3(3.0f, 2.2f, -2.0f));
    batch.drawWireBox(-4.0f, 0.0f, -4.0f, -3.0f, 1.0f, -3.0f);
}

double meanDifference(const Picture& a, const Picture& b)
{
    double total = 0.0;
    std::size_t count = 0;
    for (int y = 0; y < kSize; ++y)
    {
        for (int x = 0; x < kSize; ++x)
        {
            for (int k = 0; k < 3; ++k)
            {
                total += fabs(
                        static_cast<double>(a.at(x, y)[k]) - static_cast<double>(b.at(x, y)[k]));
                ++count;
            }
        }
    }
    return total / static_cast<double>(count);
}

void testWorld(Context& c)
{
    printf("world scene\n");
    const World world = buildWorld(c);
    for (int i = 0; i < 4; ++i)
    {
        drawDebugBatch();
        Frame frame(*c.tree);
    }

    const std::uint32_t maxSamples = Main::device()->driver()->caps().maxSamples;
    const int errorsBefore = gErrors;
    const Picture off = shoot(c, ViewportMsaa::Disabled, 1, true);
    CHECK(off.stats.depthPrePassSurfaces > 0);
    CHECK(off.stats.batchDrawCalls >= 1);
    CHECK(off.stats.shadowPasses > 0);
    CHECK(off.stats.decalsDrawn == 1);
    CHECK(off.stats.coverageSurfaces == 0);
    CHECK(world.forest->getImpostorInstanceCount() > 0);
    CHECK(world.forest->getMeshInstanceCount() > 0);
    CHECK(world.terrain->getNodesDrawn() > 0);
    CHECK(world.grass->getActiveCellCount() > 0);
    CHECK(world.particles->getActiveCount() > 0);
    printf("  off: %u draw calls, %u pre-pass surfaces, %u shadow passes, %u batch calls\n",
            off.stats.drawCalls, off.stats.depthPrePassSurfaces, off.stats.shadowPasses,
            off.stats.batchDrawCalls);

    const ViewportMsaa levels[3] = { ViewportMsaa::Msaa2x, ViewportMsaa::Msaa4x,
        ViewportMsaa::Msaa8x };
    const std::uint32_t counts[3] = { 2, 4, 8 };
    for (int level = 0; level < 3; ++level)
    {
        if (maxSamples < counts[level]) continue;
        const Picture on = shoot(c, levels[level], 3, true);
        CHECK(c.window->getMsaaSamples() == counts[level]);
        const double drift = meanDifference(on, off);
        printf("  %ux: %u draw calls, %u coverage surfaces, %u pre-pass surfaces, %u batch calls, "
               "mean difference from the unsampled frame %.3f levels\n",
                counts[level], on.stats.drawCalls, on.stats.coverageSurfaces,
                on.stats.depthPrePassSurfaces, on.stats.batchDrawCalls, drift);
        CHECK(drift < 16.0);
        CHECK(on.stats.coverageSurfaces >= 2);
        CHECK(on.stats.depthPrePassSurfaces > 0);
        CHECK(on.stats.shadowPasses > 0);
        CHECK(on.stats.decalsDrawn == 1);
        CHECK(on.stats.batchDrawCalls >= 1);
        const float depth = readDepth(c, kSize / 2, kSize - 4);
        CHECK(depth >= 0.0f && depth <= 1.0f);
    }
    CHECK(gErrors == errorsBefore);
}

void testSwitching(Context& c)
{
    printf("switching at run time\n");
    if (Main::device()->driver()->caps().maxSamples < 4)
    {
        printf("  skipped, fewer than 4 samples\n");
        return;
    }
    RenderingServer* server = Main::rendering();
    c.window->setMsaa3D(ViewportMsaa::Disabled);
    {
        Frame frame(*c.tree);
    }
    std::uint32_t pipelinesAfterFirst = 0;
    for (int cycle = 0; cycle < 4; ++cycle)
    {
        c.window->setMsaa3D(ViewportMsaa::Msaa4x);
        {
            Frame frame(*c.tree);
            CHECK(server->getStats().targetsCreated == 1);
        }
        CHECK(c.window->getMsaaSamples() == 4);
        {
            Frame frame(*c.tree);
            CHECK(server->getStats().targetsCreated == 0);
        }
        c.window->setMsaa3D(ViewportMsaa::Disabled);
        {
            Frame frame(*c.tree);
            CHECK(server->getStats().targetsCreated == 1);
        }
        CHECK(c.window->getMsaaSamples() == 1);
        {
            Frame frame(*c.tree);
            CHECK(server->getStats().targetsCreated == 0);
        }
        if (cycle == 0) pipelinesAfterFirst = server->scenePipelineCount();
        else
            CHECK(server->scenePipelineCount() == pipelinesAfterFirst);
    }
    printf("  %u scene pipelines after the first cycle and after the fourth\n",
            pipelinesAfterFirst);
    c.window->setMsaa3D(ViewportMsaa::Msaa4x);
    {
        Frame frame(*c.tree);
    }
    c.window->setMsaa3D(ViewportMsaa::Msaa2x);
    {
        Frame frame(*c.tree);
        CHECK(server->getStats().targetsCreated == 1);
    }
    CHECK(c.window->getMsaaSamples() == 2);
    c.window->setMsaa3D(ViewportMsaa::Disabled);
    {
        Frame frame(*c.tree);
    }
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    log_set_callback(onLog, nullptr);
    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "msaa test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kSize;
    desc.height = kSize;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    printf("driver maximum samples: %u\n", Main::device()->driver()->caps().maxSamples);

    SceneTree tree;
    CHECK(Main::start(&tree));
    Context c;
    c.tree = &tree;
    c.root = tree.getRoot();
    c.window = c.root ? c.root->castTo<Window>() : nullptr;
    CHECK(c.window != nullptr);
    if (!c.window) return 1;
    c.meshes = Main::meshes();
    c.textures = Main::textures();
    c.materials = Main::materials();
    c.environment = c.root->createChild<WorldEnvironment>();
    c.environment->setEnvironment(whiteAmbient());
    c.camera = c.root->createChild<Camera3D>();
    c.camera->makeCurrent();

    testTriangleEdge(c);
    testScissorCoverage(c);
    testDepthReadback(c);
    testWorld(c);
    testSwitching(c);

    CHECK(gErrors == 0);
    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures == 0) printf("all checks passed\n");
    return failures == 0 ? 0 : 1;
}
