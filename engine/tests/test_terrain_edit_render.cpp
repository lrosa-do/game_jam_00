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
#include "prisma3d/scene/WorldEnvironment.h"
#include "prisma3d/terrain/TerrainBrush.h"

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

void flush(Scene& scene)
{
    Frame frame(*scene.tree);
}

void place(Scene& scene, const Math::Vec3& position, const Math::Vec3& target)
{
    scene.camera->setGlobalTransform(
            Math::Mat4::LookAt(position, target, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine());
}

TerrainImportData pageData(const float* heights, std::uint16_t size, std::uint16_t batch,
        std::uint16_t minBatch, float world)
{
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = minBatch;
    data.maxBatchSize = batch;
    data.worldSize = world;
    data.inputFloat = heights;
    data.layerBlendMapSize = 129;
    data.lightMapSize = 129;
    data.compositeMapSize = 64;
    return data;
}

Terrain3D* makePage(Scene& scene, MaterialId material)
{
    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setCompositeMapEnabled(false);
    terrain->setMaterial(material);
    return terrain;
}

int differing(const Picture& a, const Picture& b, int tolerance, int* left, int* top, int* right,
        int* bottom)
{
    int count = 0;
    *left = kWidth;
    *top = kHeight;
    *right = -1;
    *bottom = -1;
    for (int y = 0; y < kHeight; ++y)
    {
        for (int x = 0; x < kWidth; ++x)
        {
            const unsigned char* p = a.at(x, y);
            const unsigned char* q = b.at(x, y);
            if (abs(p[0] - q[0]) > tolerance || abs(p[1] - q[1]) > tolerance ||
                    abs(p[2] - q[2]) > tolerance)
            {
                ++count;
                if (x < *left) *left = x;
                if (x > *right) *right = x;
                if (y < *top) *top = y;
                if (y > *bottom) *bottom = y;
            }
        }
    }
    return count;
}

void testPixels(Scene& scene, WorldEnvironment* environment)
{
    const std::size_t meshBaseline = Main::meshes()->count();
    const std::size_t textureBaseline = Main::textures()->count();
    const std::size_t materialBaseline = Main::materials()->count();

    EnvironmentSettings settings;
    settings.ambientSource = AmbientSource::Color;
    settings.ambientColor = Math::Vec3(0.15f, 0.15f, 0.15f);
    settings.reflectionSource = ReflectionSource::Disabled;
    environment->setEnvironment(settings);
    DirectionalLight3D* sun = scene.root->createChild<DirectionalLight3D>();
    sun->setShadow(false);
    sun->setRotation(Math::Vec3(-0.9f, 0.7f, 0.0f));

    MaterialStore& materials = *Main::materials();
    const MaterialId grey = materials.create("grey");
    materials.setColor(grey, MaterialParam::Albedo, Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    materials.setFloat(grey, MaterialParam::Roughness, 1.0f);

    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = 0.0f;

    Terrain3D* terrain = makePage(scene, grey);
    CHECK(terrain->load(pageData(heights.data(), 129, 129, 129, 128.0f)));
    scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);
    place(scene, Math::Vec3(0.0f, 45.0f, 80.0f), Math::Vec3(0.0f, 0.0f, 0.0f));

    Picture flat;
    CHECK(renderPicture(scene, flat));

    terrain->resetEditStats();
    terrain->resetUploadStats();
    const Math::Vec3 centre(30.0f, 0.0f, -20.0f);
    CHECK(terrainBrushHeight(terrain->getTerrain(), centre, 14.0f, 100.0f, 0.1f, false));
    terrain->update();
    const std::uint64_t vertexBytes = terrain->getVertexUploadBytes();
    printf("  brush radius 14 on a 129x129 single-node page: %llu vertex bytes in %u ranges, %u "
           "owner, %u other uploads of %llu bytes\n",
            static_cast<unsigned long long>(vertexBytes), terrain->getVertexUploadRanges(),
            terrain->getLastUpdateOwners(), terrain->getUploadCount(),
            static_cast<unsigned long long>(terrain->getUploadedBytes()));
    CHECK(terrain->getLastUpdateOwners() == 1);
    CHECK(vertexBytes > 0 && vertexBytes < 129ull * 129ull * 48ull / 4ull);
    CHECK(terrain->getUploadCount() == 1);

    Picture edited;
    CHECK(renderPicture(scene, edited));
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
    const int changed = differing(flat, edited, 0, &left, &top, &right, &bottom);
    float minX = 1.0e9f;
    float maxX = -1.0e9f;
    float minY = 1.0e9f;
    float maxY = -1.0e9f;
    for (int cx = -1; cx <= 1; cx += 2)
    {
        for (int cz = -1; cz <= 1; cz += 2)
        {
            for (int h = 0; h <= 1; ++h)
            {
                const Math::Vec2 p = scene.camera->unprojectPosition(Math::Vec3(
                        centre.x + 15.0f * static_cast<float>(cx), 11.0f * static_cast<float>(h),
                        centre.z + 15.0f * static_cast<float>(cz)));
                minX = p.x < minX ? p.x : minX;
                maxX = p.x > maxX ? p.x : maxX;
                minY = p.y < minY ? p.y : minY;
                maxY = p.y > maxY ? p.y : maxY;
            }
        }
    }
    printf("  edit changed %d pixels in (%d,%d)-(%d,%d); the edited area projects to "
           "(%.0f,%.0f)-(%.0f,%.0f)\n",
            changed, left, top, right, bottom, minX, minY, maxX, maxY);
    CHECK(changed > 50);
    CHECK(static_cast<float>(left) >= minX - 3.0f && static_cast<float>(right) <= maxX + 3.0f);
    CHECK(static_cast<float>(top) >= minY - 3.0f && static_cast<float>(bottom) <= maxY + 3.0f);

    ct::Vector<float> now;
    now.resize(129u * 129u);
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x)
            now[static_cast<std::size_t>(y) * 129 + x] = terrain->getTerrain().getHeightAtPoint(
                    static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
    Terrain3D* fresh = makePage(scene, grey);
    CHECK(fresh->load(pageData(now.data(), 129, 129, 129, 128.0f)));
    terrain->setVisible(false);
    Picture rebuilt;
    CHECK(renderPicture(scene, rebuilt));
    const int apart = differing(edited, rebuilt, 0, &left, &top, &right, &bottom);
    printf("  edited page against a page built from scratch with the same heights: %d pixels "
           "differ\n",
            apart);
    CHECK(apart == 0);
    fresh->queueFree();
    terrain->setVisible(true);

    terrain->resetEditStats();
    terrain->resetUploadStats();
    terrain->getTerrain().setHeightAtPoint(60, 60, 5.0f);
    terrain->getTerrain().updateGeometry();
    CHECK(terrain->getVertexUploadBytes() == 48 && terrain->getVertexUploadRanges() == 1);
    terrain->update();
    CHECK(terrain->getUploadCount() == 1 && terrain->getUploadedBytes() == 9u * 4u);

    sun->queueFree();
    terrain->queueFree();
    flush(scene);
    materials.destroy(grey);
    environment->setEnvironment(whiteAmbient());
    CHECK(Main::meshes()->count() == meshBaseline);
    CHECK(Main::textures()->count() == textureBaseline);
    CHECK(Main::materials()->count() == materialBaseline);
}

void testFourNodes(Scene& scene, WorldEnvironment* environment)
{
    EnvironmentSettings settings;
    settings.ambientSource = AmbientSource::Color;
    settings.ambientColor = Math::Vec3(0.15f, 0.15f, 0.15f);
    settings.reflectionSource = ReflectionSource::Disabled;
    environment->setEnvironment(settings);
    DirectionalLight3D* sun = scene.root->createChild<DirectionalLight3D>();
    sun->setShadow(false);
    sun->setRotation(Math::Vec3(-0.9f, 0.7f, 0.0f));

    MaterialStore& materials = *Main::materials();
    const MaterialId grey = materials.create("grey");
    materials.setColor(grey, MaterialParam::Albedo, Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    materials.setFloat(grey, MaterialParam::Roughness, 1.0f);

    ct::Vector<float> heights;
    heights.resize(513u * 513u);
    for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = 0.0f;
    Terrain3D* terrain = makePage(scene, grey);
    CHECK(terrain->load(pageData(heights.data(), 513, 65, 33, 512.0f)));

    scene.camera->setPerspective(60.0f, 0.5f, 3000.0f);
    place(scene, Math::Vec3(-175.0f, 45.0f, 180.0f), Math::Vec3(-128.0f, 0.0f, 128.0f));
    terrain->setMaxPixelError(0.0001f);
    Picture before;
    CHECK(renderPicture(scene, before));

    terrain->resetEditStats();
    for (int y = 118; y <= 138; ++y)
        for (int x = 118; x <= 138; ++x)
        {
            const float dx = static_cast<float>(x - 128);
            const float dy = static_cast<float>(y - 128);
            const float h = 30.0f - 2.5f * sqrtf(dx * dx + dy * dy);
            if (h > 0.0f)
                terrain->getTerrain().setHeightAtPoint(static_cast<std::uint32_t>(x),
                        static_cast<std::uint32_t>(y), h);
        }
    terrain->update();
    printf("  hill on the corner shared by four nodes: %u owners written, %llu vertex bytes\n",
            terrain->getLastUpdateOwners(),
            static_cast<unsigned long long>(terrain->getVertexUploadBytes()));
    CHECK(terrain->getLastUpdateOwners() == 5);

    Picture after;
    CHECK(renderPicture(scene, after));
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
    CHECK(differing(before, after, 0, &left, &top, &right, &bottom) > 20);

    ct::Vector<float> now;
    now.resize(513u * 513u);
    for (int y = 0; y < 513; ++y)
        for (int x = 0; x < 513; ++x)
            now[static_cast<std::size_t>(y) * 513 + x] = terrain->getTerrain().getHeightAtPoint(
                    static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
    Terrain3D* fresh = makePage(scene, grey);
    CHECK(fresh->load(pageData(now.data(), 513, 65, 33, 512.0f)));
    fresh->setMaxPixelError(0.0001f);
    terrain->setVisible(false);
    Picture rebuilt;
    CHECK(renderPicture(scene, rebuilt));
    const int apart = differing(after, rebuilt, 0, &left, &top, &right, &bottom);
    printf("  four-node hill against a page built from scratch: %d pixels differ\n", apart);
    CHECK(apart == 0);

    fresh->queueFree();
    sun->queueFree();
    terrain->queueFree();
    flush(scene);
    materials.destroy(grey);
    environment->setEnvironment(whiteAmbient());
}

void measureStrokes(Scene& scene)
{
    const ct::String path = ct::String(PRISMA3D_ASSETS_DIR) + "/textures/terrain/terrain.png";
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    Pixmap image;
    CHECK(!bytes.empty());
    CHECK(image.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size())));
    if (!image.is_valid()) return;

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLayerParallax(0);
    TerrainImportData data;
    data.terrainSize = 513;
    data.minBatchSize = 33;
    data.maxBatchSize = 65;
    data.worldSize = 12000.0f;
    data.inputImage = &image;
    data.inputScale = 600.0f;
    data.lightMapSize = 512;
    data.lightMapDirection = Math::Vec3(0.55f, -0.3f, 0.75f).Normalized();
    CHECK(terrain->load(data));
    terrain->setGlobalPosition(Math::Vec3(1000.0f, 0.0f, 5000.0f));
    scene.camera->setPerspective(60.0f, 1.0f, 30000.0f);
    place(scene, Math::Vec3(1700.0f, 1500.0f, 4600.0f), Math::Vec3(1700.0f, 100.0f, 4100.0f));
    {
        Frame prime(*scene.tree);
    }

    const float sizes[3] = { 0.002f, 0.02f, 0.1f };
    for (int i = 0; i < 3; ++i)
    {
        const float radius = 0.5f * sizes[i] * 12000.0f;
        const Math::Vec3 centre(1700.0f, 0.0f, 4100.0f);
        terrain->resetEditStats();
        terrain->resetUploadStats();
        const std::uint64_t bakeBefore = terrain->getCompositeBakeTexels();
        const double t0 = nowMilliseconds();
        CHECK(terrainBrushHeight(terrain->getTerrain(), centre, radius, 250.0f, 0.05f, false));
        terrain->update();
        const double updateMs = nowMilliseconds() - t0;
        const std::uint64_t vertexBytes = terrain->getVertexUploadBytes();
        const std::uint32_t ranges = terrain->getVertexUploadRanges();
        const std::uint32_t owners = terrain->getLastUpdateOwners();
        const std::uint64_t otherBytes = terrain->getUploadedBytes();
        const std::uint32_t others = terrain->getUploadCount();
        const double t1 = nowMilliseconds();
        CHECK(terrain->flushCompositeMap());
        Picture picture;
        CHECK(readPicture(picture));
        const double bakeMs = nowMilliseconds() - t1;
        Main::rendering()->endFrame(true);
        printf("  GPU stroke %.3f (radius %.0f): brush and update %.2f ms (%u owners, %llu vertex "
               "bytes in %u ranges, %llu bytes of map textures in %u uploads); composite re-bake "
               "%.2f ms for %llu texels\n",
                sizes[i], radius, updateMs, owners, static_cast<unsigned long long>(vertexBytes),
                ranges, static_cast<unsigned long long>(otherBytes), others, bakeMs,
                static_cast<unsigned long long>(terrain->getCompositeBakeTexels() - bakeBefore));
    }
    terrain->queueFree();
    flush(scene);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "terrain edit test";
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

    testPixels(scene, environment);
    testFourNodes(scene, environment);
    measureStrokes(scene);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("terrain edit render ok\n");
    return 0;
}
