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

struct Timing
{
    float opaque = 0.0f;
    float depth = 0.0f;
    float shadows = 0.0f;
    float sky = 0.0f;
    float final = 0.0f;
    float frame = 0.0f;
    int samples = 0;
    RenderStats stats;
};

Timing measure(bool prepass)
{
    Main::rendering()->setDepthPrePassEnabled(prepass);
    Timing timing;
    for (int i = 0; i < kWarmup + kMeasured; ++i)
    {
        Main::iteration();
        if (i < kWarmup) continue;
        const ProfileSample* opaque = gpuSample("Opaque");
        const ProfileSample* depth = gpuSample("Depth");
        const ProfileSample* shadows = gpuSample("Shadows");
        const ProfileSample* sky = gpuSample("Sky");
        const ProfileSample* final = gpuSample("Final");
        const ProfileSample* frame = gpuSample("Frame");
        if (!opaque || !frame) continue;
        timing.opaque += opaque->milliseconds;
        if (prepass && depth) timing.depth += depth->milliseconds;
        if (shadows) timing.shadows += shadows->milliseconds;
        if (sky) timing.sky += sky->milliseconds;
        if (final) timing.final += final->milliseconds;
        timing.frame += frame->milliseconds;
        ++timing.samples;
    }
    timing.stats = Main::rendering()->getStats();
    if (timing.samples > 0)
    {
        const float n = static_cast<float>(timing.samples);
        timing.opaque /= n;
        timing.depth /= n;
        timing.shadows /= n;
        timing.sky /= n;
        timing.final /= n;
        timing.frame /= n;
    }
    return timing;
}

void report(const char* name, const Timing& timing, bool available)
{
    printf("%s: pre-pass %u draws %u triangles, opaque %u draws %u triangles, shadows %u draws "
           "%u triangles\n",
            name, timing.stats.depthPrePassDrawCalls, timing.stats.depthPrePassTriangles,
            timing.stats.drawCalls, timing.stats.triangles, timing.stats.shadowDrawCalls,
            timing.stats.shadowTriangles);
    if (!available)
    {
        printf("%s: GPU timer queries are not available on this driver\n", name);
        return;
    }
    printf("%s: GPU ms over %d frames: depth %.3f, shadows %.3f, opaque %.3f, sky %.3f, final "
           "%.3f, frame %.3f (depth + opaque %.3f)\n",
            name, timing.samples, timing.depth, timing.shadows, timing.opaque, timing.sky,
            timing.final, timing.frame, timing.depth + timing.opaque);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "forest depth pre-pass";
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

    EnvironmentSettings environment;
    environment.background = BackgroundMode::Sky;
    environment.ambientSource = AmbientSource::Sky;
    environment.reflectionSource = ReflectionSource::Sky;
    environment.tonemapMode = TonemapMode::Aces;
    environment.tonemapWhite = 6.0f;
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
    camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(0.0f, 8.0f, 100.0f),
            Math::Vec3(0.0f, -4.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f))
                                       .InverseAffine());
    camera->makeCurrent();

    ct::Vector<std::uint8_t> withPrepass;
    ct::Vector<std::uint8_t> withoutPrepass;
    prisma::Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = kWidth;
    rect.height = kHeight;
    withPrepass.resize(static_cast<std::size_t>(kWidth) * kHeight * 4);
    withoutPrepass.resize(withPrepass.size());
    Main::rendering()->setRoughnessLimiter(false);
    for (int pass = 0; pass < 2; ++pass)
    {
        Main::rendering()->setDepthPrePassEnabled(pass == 1);
        {
            Frame warm(tree);
        }
        Frame frame(tree);
        CHECK(Main::device()->driver()->readPixels(prisma::RenderTarget(), rect,
                pass == 1 ? withPrepass.data() : withoutPrepass.data()));
        if (pass == 1)
        {
            const RenderStats& stats = Main::rendering()->getStats();
            CHECK(stats.depthPrePassDrawCalls > 0);
            CHECK(stats.depthPrePassTriangles > 0);
        }
    }
    std::size_t different = 0;
    std::size_t differentPixels = 0;
    std::size_t offByOne = 0;
    for (std::size_t i = 0; i < withPrepass.size(); i += 4)
    {
        bool pixel = false;
        bool small = false;
        for (int k = 0; k < 4; ++k)
        {
            const int delta = abs(
                    static_cast<int>(withPrepass[i + k]) - static_cast<int>(withoutPrepass[i + k]));
            if (delta > 1)
            {
                ++different;
                pixel = true;
            }
            else if (delta == 1)
                small = true;
        }
        if (pixel) ++differentPixels;
        else if (small)
            ++offByOne;
    }
    printf("forest %dx%d: %zu pixels differ with the pre-pass (%zu bytes), %zu by one level\n",
            kWidth, kHeight, differentPixels, different, offByOne);
    CHECK(differentPixels == 0);
    CHECK(offByOne == 0);

    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    const bool available = profiler != nullptr && profiler->gpuAvailable();
    if (profiler)
    {
        Timing off;
        Timing on;
        for (int round = 0; round < 3; ++round)
        {
            off = measure(false);
            on = measure(true);
            report("pre-pass off", off, available);
            report("pre-pass on ", on, available);
        }
        if (available && off.samples > 0 && on.samples > 0)
        {
            printf("opaque pass %.3f ms -> %.3f ms; with the pre-pass added %.3f ms; frame %.3f "
                   "ms -> %.3f ms\n",
                    off.opaque, on.opaque, on.depth + on.opaque, off.frame, on.frame);
        }
    }

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures == 0) printf("all checks passed\n");
    return failures == 0 ? 0 : 1;
}
