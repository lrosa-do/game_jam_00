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
#include "prisma3d/scene/TreeGenerator.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdlib.h>
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

Window* gWindow = nullptr;
WorldEnvironment* gWorld = nullptr;
EnvironmentSettings gEnvironment;

struct Timing
{
    float depth = 0.0f;
    float shadows = 0.0f;
    float opaque = 0.0f;
    float sky = 0.0f;
    float transparent = 0.0f;
    float final = 0.0f;
    float glow = 0.0f;
    float frame = 0.0f;
    int samples = 0;
    RenderStats stats;
};

struct Mode
{
    const char* name;
    bool glow;
    bool allLevels;
};

struct View
{
    const char* name;
    Math::Vec3 position;
    Math::Vec3 target;
};

float sampleOf(const char* name)
{
    const ProfileSample* sample = gpuSample(name);
    return sample ? sample->milliseconds : 0.0f;
}

Timing measure()
{
    Timing timing;
    for (int i = 0; i < kWarmup + kMeasured; ++i)
    {
        Main::iteration();
        if (i < kWarmup) continue;
        if (!gpuSample("Opaque") || !gpuSample("Frame")) continue;
        timing.depth += sampleOf("Depth");
        timing.shadows += sampleOf("Shadows");
        timing.opaque += sampleOf("Opaque");
        timing.sky += sampleOf("Sky");
        timing.transparent += sampleOf("Transparent");
        timing.final += sampleOf("Final");
        timing.glow += sampleOf("Glow");
        timing.frame += sampleOf("Frame");
        ++timing.samples;
    }
    timing.stats = Main::rendering()->getStats();
    if (timing.samples > 0)
    {
        const float n = static_cast<float>(timing.samples);
        timing.depth /= n;
        timing.shadows /= n;
        timing.opaque /= n;
        timing.sky /= n;
        timing.transparent /= n;
        timing.final /= n;
        timing.glow /= n;
        timing.frame /= n;
    }
    return timing;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;
    const int rounds = argc > 2 ? atoi(argv[2]) : 3;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "glow bench";
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
    gWindow = window;
    window->setMsaa3D(ViewportMsaa::Msaa4x);
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

    EnvironmentSettings environment;
    environment.background = BackgroundMode::Sky;
    environment.ambientSource = AmbientSource::Sky;
    environment.reflectionSource = ReflectionSource::Sky;
    environment.tonemapMode = TonemapMode::Aces;
    environment.tonemapWhite = 6.0f;
    WorldEnvironment* world = root->createChild<WorldEnvironment>();
    world->setEnvironment(environment);
    gWorld = world;
    gEnvironment = environment;

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

    MeshStore& meshes = *Main::meshes();
    TextureStore& textures = *Main::textures();
    MaterialStore& materials = *Main::materials();

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

    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPerspective(60.0f, 0.1f, 600.0f);
    camera->makeCurrent();

    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return 1;
    const bool available = profiler->gpuAvailable();
    printf("%s, %dx%d, GPU timers %s\n", vulkan ? "Vulkan" : "OpenGL", kWidth, kHeight,
            available ? "available" : "not available");
    if (!available) printf("GPU timer queries are not available on this driver\n");

    const Mode modes[3] = { { "glow off", false, false },
        { "glow on, default levels", true, false }, { "glow on, all seven levels", true, true } };
    const View views[2] = { { "far (as the depth pre-pass measurement)",
                                Math::Vec3(0.0f, 8.0f, 100.0f), Math::Vec3(0.0f, -4.0f, 0.0f) },
        { "near", Math::Vec3(0.0f, 4.0f, 30.0f), Math::Vec3(0.0f, 4.0f, 0.0f) } };

    for (int v = 0; v < 2; ++v)
    {
        camera->setGlobalTransform(
                Math::Mat4::LookAt(views[v].position, views[v].target, Math::Vec3(0.0f, 1.0f, 0.0f))
                        .InverseAffine());
        printf("view: %s\n", views[v].name);
        Timing results[3][8];
        const int used = rounds < 8 ? rounds : 8;
        for (int round = 0; round < used; ++round)
        {
            for (int m = 0; m < 3; ++m)
            {
                EnvironmentSettings settings = gEnvironment;
                settings.glowEnabled = modes[m].glow;
                if (modes[m].allLevels)
                    for (int l = 0; l < 7; ++l) settings.glowLevels[l] = 1.0f;
                gWorld->setEnvironment(settings);
                results[m][round] = measure();
            }
        }
        for (int m = 0; m < 3; ++m)
        {
            const Timing& first = results[m][0];
            printf("  %-28s glow passes %u, glow textures %u\n", modes[m].name,
                    first.stats.glowPasses, first.stats.glowTexturesLive);
            if (!available) continue;
            const char* labels[9] = { "depth", "shadows", "opaque", "sky", "transparent", "final",
                "glow", "frame scope", "sum of passes" };
            for (int k = 0; k < 9; ++k)
            {
                float low = 1.0e9f;
                float high = 0.0f;
                float sum = 0.0f;
                for (int round = 0; round < used; ++round)
                {
                    const Timing& t = results[m][round];
                    const float values[9] = { t.depth, t.shadows, t.opaque, t.sky, t.transparent,
                        t.final, t.glow, t.frame,
                        t.depth + t.shadows + t.opaque + t.sky + t.transparent + t.final + t.glow };
                    low = values[k] < low ? values[k] : low;
                    high = values[k] > high ? values[k] : high;
                    sum += values[k];
                }
                printf("    %-13s mean %8.3f ms, min %8.3f, max %8.3f, spread %6.3f\n", labels[k],
                        sum / static_cast<float>(used), low, high, high - low);
            }
        }
    }

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();
    return failures == 0 ? 0 : 1;
}
