#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/core/Random.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/TreeGenerator.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const float kPi = 3.14159265358979f;
const int kWarmup = 8;
const int kMeasured = 40;

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

const ProfileSample* gpuSample(const char* name)
{
    RenderProfiler* profiler = Main::profiler();
    for (std::uint32_t i = 0; i < profiler->gpuSampleCount(); ++i)
        if (strcmp(profiler->gpuSamples()[i].name, name) == 0) return &profiler->gpuSamples()[i];
    return nullptr;
}

Math::Box treeBox(const TreeGeometry& geometry)
{
    return MeshStore::computeAabb(geometry.bark.positions)
            .Union(MeshStore::computeAabb(geometry.leaves.positions));
}

ct::String assetPath(const char* relative)
{
    ct::String path(PRISMA3D_ASSETS_DIR);
    path += "/";
    path += relative;
    return path;
}

void plantForest(Forest3D& forest, const float* fits, const int* counts, Random& random)
{
    const float radius = 300.0f;
    int total = 0;
    for (int s = 0; s < 3; ++s) total += counts[s];
    const float area = kPi * radius * radius;
    const float cell = sqrtf(area / static_cast<float>(total));

    ct::Vector<Math::Vec2> spots;
    for (float z = -radius; z < radius; z += cell)
    {
        for (float x = -radius; x < radius; x += cell)
        {
            const float px = x + random.randfRange(0.1f, 0.9f) * cell;
            const float pz = z + random.randfRange(0.1f, 0.9f) * cell;
            if (px * px + pz * pz <= radius * radius) spots.push_back(Math::Vec2(px, pz));
        }
    }
    for (std::size_t i = spots.size(); i > 1; --i)
    {
        const std::size_t j =
                static_cast<std::size_t>(random.randiRange(0, static_cast<int>(i) - 1));
        const Math::Vec2 held = spots[i - 1];
        spots[i - 1] = spots[j];
        spots[j] = held;
    }

    std::size_t next = 0;
    for (int s = 0; s < 3; ++s)
    {
        for (int i = 0; i < counts[s]; ++i)
        {
            Math::Vec2 point(0.0f, 0.0f);
            if (next < spots.size()) point = spots[next++];
            else
            {
                const float angle = random.randfRange(0.0f, 6.2831853f);
                const float distance = radius * sqrtf(random.randf());
                point = Math::Vec2(cosf(angle) * distance, sinf(angle) * distance);
            }
            const float scale = fits[s] * random.randfRange(0.8f, 1.25f);
            const float yaw = random.randfRange(0.0f, 6.2831853f);
            forest.plant(s, Math::Vec3(point.x, 0.0f, point.y), yaw, scale);
        }
    }
}


struct Sample
{
    float opaque = 0.0f;
    float depth = 0.0f;
    float sky = 0.0f;
    float frame = 0.0f;
    int count = 0;
};

const int kModes = 3;
const char* const kModeNames[kModes] = { "fog off", "fog exponential",
    "fog depth, height, sun, aerial" };

void configure(WorldEnvironment* world, EnvironmentSettings settings, int mode)
{
    settings.fogEnabled = mode != 0;
    if (mode == 1)
    {
        settings.fogMode = FogMode::Exponential;
        settings.fogDensity = 0.004f;
    }
    else if (mode == 2)
    {
        settings.fogMode = FogMode::Depth;
        settings.fogDepthBegin = 20.0f;
        settings.fogDepthEnd = 500.0f;
        settings.fogDensity = 0.8f;
        settings.fogHeightDensity = 0.02f;
        settings.fogHeight = 10.0f;
        settings.fogSunScatter = 0.3f;
        settings.fogAerialPerspective = 0.5f;
    }
    world->setEnvironment(settings);
}

void run(WorldEnvironment* world, const EnvironmentSettings& settings, const char* name)
{
    RenderProfiler* profiler = Main::profiler();
    if (!profiler || !profiler->gpuAvailable())
    {
        printf("%s: GPU timer queries are not available on this driver\n", name);
        return;
    }
    Main::rendering()->setDepthPrePassEnabled(true);
    const int rounds = 8;
    Sample samples[kModes][rounds];
    for (int round = 0; round < rounds; ++round)
    {
        for (int mode = 0; mode < kModes; ++mode)
        {
            configure(world, settings, mode);
            Sample& sample = samples[mode][round];
            for (int i = 0; i < kWarmup + kMeasured; ++i)
            {
                Main::iteration();
                if (i < kWarmup) continue;
                const ProfileSample* opaque = gpuSample("Opaque");
                const ProfileSample* depth = gpuSample("Depth");
                const ProfileSample* sky = gpuSample("Sky");
                const ProfileSample* frame = gpuSample("Frame");
                if (!opaque || !frame) continue;
                sample.opaque += opaque->milliseconds;
                if (depth) sample.depth += depth->milliseconds;
                if (sky) sample.sky += sky->milliseconds;
                sample.frame += frame->milliseconds;
                ++sample.count;
            }
            if (sample.count > 0)
            {
                const float n = static_cast<float>(sample.count);
                sample.opaque /= n;
                sample.depth /= n;
                sample.sky /= n;
                sample.frame /= n;
            }
        }
    }
    for (int mode = 0; mode < kModes; ++mode)
    {
        float values[rounds];
        float depth = 0.0f;
        float sky = 0.0f;
        float total = 0.0f;
        for (int round = 0; round < rounds; ++round)
        {
            values[round] = samples[mode][round].opaque;
            depth += samples[mode][round].depth / rounds;
            sky += samples[mode][round].sky / rounds;
            total += values[round] / rounds;
        }
        for (int a = 0; a < rounds; ++a)
            for (int b = a + 1; b < rounds; ++b)
                if (values[b] < values[a])
                {
                    const float held = values[a];
                    values[a] = values[b];
                    values[b] = held;
                }
        printf("%s, %-32s: opaque median %.3f ms (min %.3f, max %.3f, mean %.3f), depth %.3f, "
               "sky %.3f\n",
                name, kModeNames[mode], (values[rounds / 2 - 1] + values[rounds / 2]) * 0.5f,
                values[0], values[rounds - 1], total, depth, sky);
    }
}

EnvironmentSettings skyEnvironment()
{
    EnvironmentSettings environment;
    environment.background = BackgroundMode::Sky;
    environment.ambientSource = AmbientSource::Sky;
    environment.reflectionSource = ReflectionSource::Sky;
    environment.tonemapMode = TonemapMode::Aces;
    environment.tonemapWhite = 6.0f;
    return environment;
}

void buildForest(Node* root, MeshStore& meshes, TextureStore& textures, MaterialStore& materials)
{
    const MaterialId groundMaterial = materials.create("ground");
    materials.setColor(groundMaterial, MaterialParam::Albedo,
            Math::Vec4(0.30f, 0.38f, 0.28f, 1.0f));
    materials.setFloat(groundMaterial, MaterialParam::Roughness, 0.9f);
    MeshInstance3D* ground = root->createChild<MeshInstance3D>();
    ground->setMesh(createPlaneMesh(meshes, 1400.0f, 1400.0f));
    ground->setMaterialOverride(groundMaterial);

    const TextureId barkAlbedo =
            textures.createFromFile(assetPath("textures/trees/bark.jpg"), true, true);
    const TextureId barkNormal =
            textures.createFromFile(assetPath("textures/trees/bark_normal.png"), false, true);
    const char* const leafFiles[3] = { "textures/trees/twig.png", "textures/trees/Branches1.png",
        "textures/trees/Branches2.png" };
    TextureId leafCards[3];
    for (int i = 0; i < 3; ++i)
    {
        const ct::Vector<std::uint8_t> bytes =
                FileSystem::getSingleton().readBinary(assetPath(leafFiles[i]));
        Pixmap pixels;
        CHECK(!bytes.empty() &&
                pixels.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size())));
        if (bytes.empty()) continue;
        pixels.fill_transparent_color();
        leafCards[i] = textures.createFromPixmap(pixels, true, true, leafFiles[i], 4);
        CHECK(leafCards[i].valid());
    }
    CHECK(barkAlbedo.valid() && barkNormal.valid());

    const std::uint32_t presets[3] = { 0, 4, 2 };
    const float heights[3] = { 12.0f, 16.0f, 3.0f };
    const int counts[3] = { 1200, 600, 1800 };
    float fits[3] = { 1.0f, 1.0f, 1.0f };

    Forest3D* forest = root->createChild<Forest3D>();
    forest->setSwapDistance(120.0f);
    forest->setCastShadows(true);
    for (int s = 0; s < 3; ++s)
    {
        const TreePreset& preset = TreeGenerator::preset(presets[s]);
        TreeGeometry geometry;
        TreeGenerator::generate(preset.params, geometry);
        const Math::Box extent = treeBox(geometry);
        fits[s] = heights[s] / (extent.max.y - extent.min.y);

        const MaterialId bark = materials.create("bark");
        materials.setFloat(bark, MaterialParam::Roughness, 0.85f);
        materials.setTexture(bark, MaterialTexture::Albedo, barkAlbedo);
        materials.setTexture(bark, MaterialTexture::Normal, barkNormal);
        const MaterialId leaves = materials.create("leaves");
        materials.setFloat(leaves, MaterialParam::Roughness, 0.6f);
        materials.setFloat(leaves, MaterialParam::AlphaScissorThreshold, 0.4f);
        materials.setTransparency(leaves, MaterialTransparency::AlphaScissor);
        materials.setCull(leaves, MaterialCull::Back);
        materials.setTexture(leaves, MaterialTexture::Albedo, leafCards[preset.twigTexture % 3]);

        ForestSpeciesDesc description;
        description.mesh = meshes.create("forest tree");
        CHECK(meshes.addSurface(description.mesh, geometry.bark, "bark"));
        CHECK(meshes.addSurface(description.mesh, geometry.leaves, "leaves"));
        meshes.setSurfaceMaterial(description.mesh, 0, bark);
        meshes.setSurfaceMaterial(description.mesh, 1, leaves);
        TreeGeometry shadow;
        TreeGenerator::buildShadowGeometry(geometry, 6, shadow);
        description.shadowMesh = meshes.create("forest tree shadow");
        CHECK(meshes.addSurface(description.shadowMesh, shadow.bark, "shadow bark"));
        CHECK(meshes.addSurface(description.shadowMesh, shadow.leaves, "shadow leaves"));
        description.materials.push_back(bark);
        description.materials.push_back(leaves);
        CHECK(forest->addSpecies(description) == s);
    }
    Random random;
    random.seed(20261010u);
    plantForest(*forest, fits, counts, random);
    CHECK(forest->getInstanceCount() == 3600);
}

void buildTerrain(Node* root)
{
    const int size = 513;
    static ct::Vector<float> heights;
    heights.resize(static_cast<std::size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            heights[static_cast<std::size_t>(y) * size + x] = 30.0f *
                                                              sinf(static_cast<float>(x) * 0.05f) *
                                                              cosf(static_cast<float>(y) * 0.04f);
    Terrain3D* terrain = root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = 33;
    data.maxBatchSize = 65;
    data.worldSize = 2000.0f;
    data.inputFloat = heights.data();
    CHECK(terrain->load(data));
}

} // namespace

int main(int argc, char** argv)
{
    bool vulkan = false;
    bool terrainScene = false;
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "vulkan") == 0) vulkan = true;
        if (strcmp(argv[i], "terrain") == 0) terrainScene = true;
    }

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "fog bench";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    Window* window = root ? root->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

    EnvironmentSettings environment = skyEnvironment();
    WorldEnvironment* world = root->createChild<WorldEnvironment>();
    world->setEnvironment(environment);

    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    sun->setShadow(true);
    sun->setShadowMode(DirectionalShadowMode::Parallel4Splits);
    sun->setShadowMaxDistance(120.0f);
    {
        const float elevation = 55.0f * kPi / 180.0f;
        const float azimuth = 40.0f * kPi / 180.0f;
        const Math::Vec3 direction(cosf(elevation) * sinf(azimuth), sinf(elevation),
                cosf(elevation) * cosf(azimuth));
        sun->lookAtFromPosition(direction * 20.0f, Math::Vec3(0.0f, 0.0f, 0.0f));
    }

    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPerspective(60.0f, 0.1f, 600.0f);
    if (terrainScene)
    {
        buildTerrain(root);
        camera->setPerspective(60.0f, 0.5f, 3000.0f);
        camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(0.0f, 60.0f, 300.0f),
                Math::Vec3(0.0f, 20.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f))
                                           .InverseAffine());
    }
    else
    {
        buildForest(root, *Main::meshes(), *Main::textures(), *Main::materials());
        camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(0.0f, 8.0f, 100.0f),
                Math::Vec3(0.0f, -4.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f))
                                           .InverseAffine());
    }
    camera->makeCurrent();

    run(world, environment, terrainScene ? "terrain" : "forest");

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    return failures == 0 ? 0 : 1;
}
