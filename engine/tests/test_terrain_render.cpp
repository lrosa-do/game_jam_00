#include "Check.h"
#include "ColouredBox.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
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

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };

double nowMilliseconds()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0 + static_cast<double>(ts.tv_nsec) / 1.0e6;
}

bool near(float a, float b, float tolerance = 1e-3f)
{
    return fabsf(a - b) <= tolerance;
}

struct Picture
{
    ct::Vector<unsigned char> pixels;

    bool isBackground(int x, int y) const
    {
        const unsigned char* p = &pixels[(static_cast<std::size_t>(y) * kWidth + x) * 4];
        return abs(p[0] - kClear.r) <= 4 && abs(p[1] - kClear.g) <= 4 && abs(p[2] - kClear.b) <= 4;
    }

    const unsigned char* at(int x, int y) const
    {
        return &pixels[(static_cast<std::size_t>(y) * kWidth + x) * 4];
    }

    int countTerrain() const
    {
        int count = 0;
        for (int y = 0; y < kHeight; ++y)
            for (int x = 0; x < kWidth; ++x)
                if (!isBackground(x, y)) ++count;
        return count;
    }
};

bool readPicture(Picture& picture)
{
    picture.pixels.resize(static_cast<std::size_t>(kWidth) * kHeight * 4);
    prisma::Rect rect;
    rect.x = 0;
    rect.y = 0;
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

    const RenderStats& stats() const
    {
        return Main::rendering()->getStats();
    }
};

struct Scene
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Camera3D* camera = nullptr;
};

void place(Scene& scene, const Math::Vec3& position, const Math::Vec3& target,
        const Math::Vec3& up = Math::Vec3(0.0f, 1.0f, 0.0f))
{
    scene.camera->setGlobalTransform(Math::Mat4::LookAt(position, target, up).InverseAffine());
}

bool renderPicture(Scene& scene, Picture& picture, RenderStats* stats = nullptr)
{
    {
        Frame prime(*scene.tree);
    }
    Frame frame(*scene.tree);
    if (stats) *stats = frame.stats();
    return readPicture(picture);
}

TerrainImportData terrainData(std::uint16_t size, std::uint16_t minBatch, std::uint16_t maxBatch,
        float worldSize, const float* heights)
{
    TerrainImportData data;
    data.terrainSize = size;
    data.minBatchSize = minBatch;
    data.maxBatchSize = maxBatch;
    data.worldSize = worldSize;
    data.inputFloat = heights;
    return data;
}

MaterialId redMaterial(MaterialStore& materials)
{
    const MaterialId material = materials.create("terrain red");
    materials.setColor(material, MaterialParam::Albedo, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    materials.setFloat(material, MaterialParam::Roughness, 1.0f);
    return material;
}

void testCounts(Scene& scene)
{
    MeshStore& meshes = *Main::meshes();
    MaterialStore& materials = *Main::materials();
    const std::size_t meshBaseline = meshes.count();
    const std::size_t materialBaseline = materials.count();

    ct::Vector<float> heights;
    heights.resize(513u * 513u);
    for (int y = 0; y < 513; ++y)
        for (int x = 0; x < 513; ++x)
            heights[static_cast<std::size_t>(y) * 513 + x] = 20.0f *
                                                             sinf(static_cast<float>(x) * 0.05f) *
                                                             cosf(static_cast<float>(y) * 0.04f);

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    CHECK(terrain->castTo<Terrain3D>() == terrain);
    CHECK(terrain->castTo<Node3D>() == terrain);
    CHECK(!terrain->isLoaded());
    CHECK(materials.count() == materialBaseline + 1);

    TerrainImportData bad = terrainData(130, 33, 65, 1000.0f, heights.data());
    CHECK(!terrain->load(bad));
    bad = terrainData(513, 33, 65, 1000.0f, heights.data());
    bad.terrainAlign = TerrainAlignment::AlignXY;
    CHECK(!terrain->load(bad));
    CHECK(!terrain->isLoaded() && meshes.count() == meshBaseline);
    bad = terrainData(33, 5, 17, 32.0f, heights.data());
    CHECK(!terrain->load(bad));
    CHECK(!terrain->isLoaded() && meshes.count() == meshBaseline && terrain->getChildCount() == 0);

    const double start = nowMilliseconds();
    CHECK(terrain->load(terrainData(513, 33, 65, 12000.0f, heights.data())));
    const double built = nowMilliseconds() - start;
    printf("  513x513 page, 12000 wide, GPU geometry built with the page in %.1f ms\n", built);
    CHECK(terrain->isLoaded());
    CHECK(terrain->getNodeCount() == 85);
    CHECK(terrain->getVertexMeshCount() == 17);
    CHECK(terrain->getIndexPoolCount() == 9);
    CHECK(terrain->getNodeMeshCount() == 149);
    CHECK(meshes.count() == meshBaseline + 17 + 9 + 149);
    CHECK(terrain->getLodLevelCount() == 5);
    CHECK(terrain->getChildCount() == 85);

    const TerrainQuadTreeNode* root = terrain->getTerrain().getQuadTree();
    CHECK(terrain->getNodeInstance(root) != nullptr);
    CHECK(terrain->getNodeMesh(root, 0).valid() && !terrain->getNodeMesh(root, 1).valid());
    const MeshInfo* info = meshes.get(terrain->getNodeMesh(root, 0));
    CHECK(info && info->surfaces.size() == 1 && info->surfaces[0].sharedVertices &&
            info->surfaces[0].sharedIndices);
    CHECK(info->surfaces[0].topology == prisma::Topology::TriangleStrip);
    CHECK(info->surfaces[0].indexCount == Terrain::getNumIndexesForBatchSize(33));
    const TerrainQuadTreeNode* leaf = root->getChild(0)->getChild(0)->getChild(0);
    CHECK(leaf->isLeaf() && terrain->getNodeMesh(leaf, 1).valid());
    CHECK(meshes.get(terrain->getNodeMesh(leaf, 0))->surfaces[0].indexCount ==
            Terrain::getNumIndexesForBatchSize(65));

    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }
    CHECK(meshes.count() == meshBaseline);
    CHECK(materials.count() == materialBaseline);
}

void testColourAndStats(Scene& scene)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId red = redMaterial(materials);

    ct::Vector<float> flat;
    flat.resize(129u * 129u);
    for (std::size_t i = 0; i < flat.size(); ++i) flat[i] = 0.0f;

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setMaterial(red);
    CHECK(terrain->getMaterial() == red);
    CHECK(materials.get(red)->terrainMorph);
    CHECK(terrain->load(terrainData(129, 17, 33, 100.0f, flat.data())));
    CHECK(terrain->getCastShadows());

    place(scene, Math::Vec3(0.0f, 90.0f, 60.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
    scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);

    Picture picture;
    RenderStats stats;
    CHECK(renderPicture(scene, picture, &stats));
    const unsigned char* centre = picture.at(kWidth / 2, kHeight / 2);
    printf("  centre pixel %d %d %d\n", centre[0], centre[1], centre[2]);
    CHECK(abs(centre[0] - 255) <= 3 && centre[1] <= 3 && centre[2] <= 3);
    CHECK(terrain->getNodesDrawn() >= 1);

    int total = 0;
    for (int lod = 0; lod < terrain->getLodLevelCount(); ++lod)
        total += terrain->getNodesDrawnAtLod(lod);
    CHECK(total == terrain->getNodesDrawn());
    CHECK(stats.instancesDrawn == static_cast<std::uint32_t>(terrain->getNodesDrawn()));
    CHECK(stats.triangles == terrain->getTrianglesDrawn());
    CHECK(stats.drawCalls == static_cast<std::uint32_t>(terrain->getNodesDrawn()));
    printf("  flat page from above: %d nodes, %u triangles, %u draw calls\n",
            terrain->getNodesDrawn(), stats.triangles, stats.drawCalls);

    terrain->setMaterial(MaterialId());
    CHECK(terrain->getMaterial() != red);
    CHECK(renderPicture(scene, picture));
    const unsigned char* shaded = picture.at(kWidth / 2, kHeight / 2);
    CHECK(shaded[0] > 100 && shaded[1] > 100 && shaded[2] > 100);

    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(red);
}

float rough(int x, int y)
{
    return 6.0f * sinf(static_cast<float>(x) * 0.21f) * cosf(static_cast<float>(y) * 0.17f) +
           3.0f * sinf(static_cast<float>(x + 2 * y) * 0.45f);
}

int holesBelowTopEdge(const Picture& picture)
{
    int holes = 0;
    for (int x = 0; x < kWidth; ++x)
    {
        int top = -1;
        for (int y = 0; y < kHeight; ++y)
            if (!picture.isBackground(x, y))
            {
                top = y;
                break;
            }
        if (top < 0) continue;
        for (int y = top; y < kHeight; ++y)
            if (picture.isBackground(x, y)) ++holes;
    }
    return holes;
}

int differingPixels(const Picture& a, const Picture& b)
{
    int count = 0;
    for (int y = 0; y < kHeight; ++y)
        for (int x = 0; x < kWidth; ++x)
            if (a.isBackground(x, y) != b.isBackground(x, y)) ++count;
    return count;
}

void testSilhouettes(Scene& scene)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId red = redMaterial(materials);

    ct::Vector<float> bump;
    bump.resize(129u * 129u);
    for (std::size_t i = 0; i < bump.size(); ++i) bump[i] = 0.0f;
    bump[64u * 129u + 64u] = 3.0f;

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setMaterial(red);
    CHECK(terrain->load(terrainData(129, 17, 33, 100.0f, bump.data())));
    place(scene, Math::Vec3(0.0f, 70.0f, 70.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
    scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);

    Picture fine;
    Picture coarse;
    RenderStats fineStats;
    RenderStats coarseStats;

    terrain->setMaxPixelError(0.0001f);
    CHECK(renderPicture(scene, fine, &fineStats));
    const int fineNodes = terrain->getNodesDrawn();
    const int fineLod0 = terrain->getNodesDrawnAtLod(0);
    terrain->setMaxPixelError(50.0f);
    CHECK(renderPicture(scene, coarse, &coarseStats));
    const int coarseNodes = terrain->getNodesDrawn();
    printf("  bumped flat page: fine %d nodes (%d at LOD 0), %u triangles; coarse %d nodes, %u "
           "triangles\n",
            fineNodes, fineLod0, fineStats.triangles, coarseNodes, coarseStats.triangles);
    CHECK(fineLod0 > 0);
    CHECK(coarseStats.triangles < fineStats.triangles);
    CHECK(fine.countTerrain() > 1500);
    const int differing = differingPixels(fine, coarse);
    int interior = 0;
    for (int y = 1; y < kHeight - 1; ++y)
        for (int x = 1; x < kWidth - 1; ++x)
        {
            if (fine.isBackground(x, y) == coarse.isBackground(x, y)) continue;
            const bool boundary = fine.isBackground(x - 1, y) || fine.isBackground(x + 1, y) ||
                                  fine.isBackground(x, y - 1) || fine.isBackground(x, y + 1) ||
                                  coarse.isBackground(x - 1, y) || coarse.isBackground(x + 1, y) ||
                                  coarse.isBackground(x, y - 1) || coarse.isBackground(x, y + 1);
            if (!boundary) ++interior;
        }
    printf("  silhouette of %d pixels, %d pixels differ between the two detail levels, %d away "
           "from the outline\n",
            fine.countTerrain(), differing, interior);
    CHECK(interior == 0);
    CHECK(differing <= 4);

    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }

    ct::Vector<float> hills;
    hills.resize(129u * 129u);
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x) hills[static_cast<std::size_t>(y) * 129 + x] = rough(x, y);

    for (int pass = 0; pass < 2; ++pass)
    {
        Terrain3D* page = scene.root->createChild<Terrain3D>();
        page->setLightmapEnabled(false);
        page->setMaterial(red);
        TerrainImportData data = terrainData(129, 17, 33, 128.0f, hills.data());
        data.skirtSize = pass == 0 ? 30.0f : 0.0f;
        CHECK(page->load(data));
        place(scene, Math::Vec3(0.0f, 12.0f, 90.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
        scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);

        int mixed = 0;
        int worst = 0;
        const float errors[4] = { 2.0f, 6.0f, 14.0f, 30.0f };
        for (int e = 0; e < 4; ++e)
        {
            page->setMaxPixelError(errors[e]);
            Picture picture;
            CHECK(renderPicture(scene, picture));
            int levels = 0;
            for (int lod = 0; lod < page->getLodLevelCount(); ++lod)
                if (page->getNodesDrawnAtLod(lod) > 0) ++levels;
            if (levels > 1) ++mixed;
            const int holes = holesBelowTopEdge(picture);
            if (holes > worst) worst = holes;
            printf("  hills, skirt %.0f, pixel error %4.1f: %2d nodes over %d LOD levels, %d "
                   "background pixels under the top edge\n",
                    data.skirtSize, errors[e], page->getNodesDrawn(), levels, holes);
            CHECK(picture.countTerrain() > 4000);
        }
        CHECK(mixed >= 2);
        if (pass == 0) CHECK(worst == 0);
        else
            printf("  control without skirts: worst %d background pixels\n", worst);
        page->queueFree();
        {
            Frame frame(*scene.tree);
        }
    }
    materials.destroy(red);
}

int spikeArea(const Picture& picture)
{
    int count = 0;
    for (int y = 0; y < 76; ++y)
        for (int x = 0; x < kWidth; ++x)
            if (!picture.isBackground(x, y)) ++count;
    return count;
}

void testMorph(Scene& scene)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId red = redMaterial(materials);

    ct::Vector<float> spike;
    spike.resize(33u * 33u);
    for (std::size_t i = 0; i < spike.size(); ++i) spike[i] = 0.0f;
    spike[16u * 33u + 16u] = 8.0f;

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setMaterial(red);
    CHECK(terrain->load(terrainData(33, 5, 33, 32.0f, spike.data())));
    place(scene, Math::Vec3(0.0f, 4.0f, 60.0f), Math::Vec3(0.0f, 4.0f, 0.0f));
    scene.camera->setPerspective(20.0f, 0.5f, 1000.0f);

    const float radius = sqrtf(16.0f * 16.0f * 2.0f);
    const float dist = sqrtf(4.0f * 4.0f + 60.0f * 60.0f) - radius * 0.5f;
    const float aspectTan = 1.0f / tanf(10.0f * Math::DEG2RAD);
    const float targets[4] = { 0.0f, 0.5f, 0.99f, -1.0f };

    int areas[4] = { 0, 0, 0, 0 };
    float reached[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < 4; ++i)
    {
        float distTransition = dist / (1.0f - 0.25f * (1.0f - targets[i]));
        if (i == 3) distTransition = dist * 0.95f;
        const float cFactor = distTransition / 4.0f;
        terrain->setMaxPixelError(aspectTan * static_cast<float>(kHeight) / (2.0f * cFactor));

        Picture picture;
        CHECK(renderPicture(scene, picture));
        const TerrainQuadTreeNode* root = terrain->getTerrain().getQuadTree();
        reached[i] = root->getLodTransition();
        areas[i] = spikeArea(picture);
        printf("  spike: target %.2f, LOD %d, transition %.3f, morph %.3f/%.0f, silhouette %d "
               "pixels\n",
                targets[i], root->getCurrentLod(), root->getLodTransition(), root->getLodMorph().x,
                root->getLodMorph().y, areas[i]);
        if (i < 3)
        {
            CHECK(root->getCurrentLod() == 0);
            CHECK(near(root->getLodTransition(), targets[i], 0.03f));
        }
        else
            CHECK(root->getCurrentLod() == 1);
    }
    CHECK(areas[0] > 100);
    const float half = static_cast<float>(areas[1]) / static_cast<float>(areas[0]);
    const float full = static_cast<float>(areas[2]) / static_cast<float>(areas[0]);
    printf("  silhouette growth with the morph: %.2f at 0.5, %.2f at 1.0\n", half, full);
    CHECK(half > 1.25f && half < 1.75f);
    CHECK(full > 1.7f && full < 2.3f);
    CHECK(areas[0] < areas[1] && areas[1] < areas[2]);
    CHECK(areas[3] > areas[0] * 3 / 2);
    (void) reached;

    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(red);
}

void testDepthOrder(Scene& scene, MeshId box)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId green = materials.create("terrain green");
    materials.setColor(green, MaterialParam::Albedo, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    materials.setFloat(green, MaterialParam::Roughness, 1.0f);

    ct::Vector<float> flat;
    flat.resize(129u * 129u);
    for (std::size_t i = 0; i < flat.size(); ++i) flat[i] = 0.0f;
    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setMaterial(green);
    CHECK(terrain->load(terrainData(129, 17, 33, 100.0f, flat.data())));

    scene.camera->setPerspective(60.0f, 0.5f, 1000.0f);
    place(scene, Math::Vec3(0.0f, 60.0f, 1.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -1.0f));

    MeshInstance3D* block = scene.root->createChild<MeshInstance3D>();
    block->setMesh(box);
    block->setScale(Math::Vec3(10.0f, 10.0f, 10.0f));
    block->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    Picture standing;
    CHECK(renderPicture(scene, standing));
    const unsigned char* top = standing.at(kWidth / 2, kHeight / 2);
    const unsigned char* around = standing.at(kWidth / 2 + 55, kHeight / 2);
    printf("  block standing on the page: top pixel %d %d %d, terrain pixel %d %d %d\n", top[0],
            top[1], top[2], around[0], around[1], around[2]);
    CHECK(top[0] > 200 && top[1] > 200 && top[2] > 200);
    CHECK(around[0] <= 3 && around[1] >= 252 && around[2] <= 3);

    block->setPosition(Math::Vec3(0.0f, -20.0f, 0.0f));
    Picture buried;
    CHECK(renderPicture(scene, buried));
    const unsigned char* hidden = buried.at(kWidth / 2, kHeight / 2);
    printf("  block under the page: centre pixel %d %d %d\n", hidden[0], hidden[1], hidden[2]);
    CHECK(hidden[0] <= 3 && hidden[1] >= 252 && hidden[2] <= 3);

    block->queueFree();
    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(green);
}

int darkerPixels(const Picture& lit, const Picture& shadowed)
{
    int count = 0;
    for (int y = 0; y < kHeight; ++y)
        for (int x = 0; x < kWidth; ++x)
            if (static_cast<float>(shadowed.at(x, y)[1]) <
                    0.6f * static_cast<float>(lit.at(x, y)[1]))
                ++count;
    return count;
}

void testShadows(Scene& scene, MeshId box)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId grey = materials.create("terrain grey");
    materials.setColor(grey, MaterialParam::Albedo, Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    materials.setFloat(grey, MaterialParam::Roughness, 1.0f);

    ct::Vector<float> flat;
    flat.resize(129u * 129u);
    for (std::size_t i = 0; i < flat.size(); ++i) flat[i] = 0.0f;

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setMaterial(grey);
    CHECK(terrain->load(terrainData(129, 17, 33, 100.0f, flat.data())));

    DirectionalLight3D* sun = scene.root->createChild<DirectionalLight3D>();
    sun->setShadow(true);
    sun->setShadowMaxDistance(300.0f);
    sun->setRotation(Math::Vec3(-1.0f, 0.6f, 0.0f));

    scene.camera->setPerspective(50.0f, 0.5f, 1000.0f);
    place(scene, Math::Vec3(0.0f, 140.0f, 1.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, -1.0f));

    Picture withShadows;
    Picture withoutShadows;
    RenderStats stats;
    CHECK(renderPicture(scene, withShadows, &stats));
    CHECK(stats.shadowCasters > 0);
    const std::uint32_t terrainCasters = stats.shadowCasters;
    sun->setShadow(false);
    CHECK(renderPicture(scene, withoutShadows));
    sun->setShadow(true);
    const unsigned char* sample = withShadows.at(kWidth / 2, kHeight / 2);
    printf("  flat page under the sun: pixel %d %d %d, %u shadow casters, %d pixels darker with "
           "shadows\n",
            sample[0], sample[1], sample[2], terrainCasters,
            darkerPixels(withoutShadows, withShadows));
    CHECK(sample[1] > 60);
    CHECK(darkerPixels(withoutShadows, withShadows) < 40);

    terrain->setCastShadows(false);
    CHECK(!terrain->getCastShadows());
    CHECK(renderPicture(scene, withShadows, &stats));
    CHECK(stats.shadowCasters == 0);
    terrain->setCastShadows(true);

    MeshInstance3D* block = scene.root->createChild<MeshInstance3D>();
    block->setMesh(box);
    block->setScale(Math::Vec3(14.0f, 14.0f, 14.0f));
    block->setPosition(Math::Vec3(0.0f, 8.0f, 0.0f));
    CHECK(renderPicture(scene, withShadows, &stats));
    CHECK(stats.shadowCasters > terrainCasters);
    sun->setShadow(false);
    CHECK(renderPicture(scene, withoutShadows));
    sun->setShadow(true);
    const int footprint = darkerPixels(withoutShadows, withShadows);
    printf("  with a block above: %d pixels darkened by its shadow on the page\n", footprint);
    CHECK(footprint > 150);

    block->queueFree();
    sun->queueFree();
    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(grey);
}

void testShadowMorph(Scene& scene)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId grey = materials.create("terrain grey");
    materials.setColor(grey, MaterialParam::Albedo, Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    materials.setFloat(grey, MaterialParam::Roughness, 1.0f);

    ct::Vector<float> heights;
    heights.resize(513u * 513u);
    for (int y = 0; y < 513; ++y)
        for (int x = 0; x < 513; ++x)
        {
            const float fx = static_cast<float>(x);
            const float fy = static_cast<float>(y);
            heights[static_cast<std::size_t>(y) * 513 + x] =
                    250.0f * sinf(fx * 0.11f) * cosf(fy * 0.09f) +
                    80.0f * sinf(fx * 0.31f + 1.0f) * sinf(fy * 0.27f);
        }

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setMaterial(grey);
    CHECK(terrain->load(terrainData(513, 33, 65, 12000.0f, heights.data())));

    DirectionalLight3D* sun = scene.root->createChild<DirectionalLight3D>();
    sun->setShadow(true);
    sun->setShadowMaxDistance(3000.0f);
    sun->setRotation(Math::Vec3(-1.25f, 0.6f, 0.0f));

    scene.camera->setPerspective(60.0f, 1.0f, 30000.0f);

    int lodTotal = 0;
    int fullTotal = 0;
    int lodWorst = 0;
    for (int i = 0; i < 8; ++i)
    {
        const float t = static_cast<float>(i) / 7.0f;
        const float x = -4000.0f + 8000.0f * t;
        const float z = 2500.0f * sinf(t * 9.0f);
        const float ground = terrain->getWorldHeightAt(Math::Vec3(x, 0.0f, z));
        place(scene, Math::Vec3(x, ground + 120.0f, z),
                Math::Vec3(x + 400.0f * cosf(t * 5.0f), ground + 20.0f,
                        z + 400.0f * sinf(t * 5.0f)));
        Picture lit;
        Picture shadowed;
        terrain->setMaxPixelError(8.0f);
        sun->setShadow(false);
        CHECK(renderPicture(scene, lit));
        sun->setShadow(true);
        CHECK(renderPicture(scene, shadowed));
        const int lod = darkerPixels(lit, shadowed);
        terrain->setMaxPixelError(0.05f);
        sun->setShadow(false);
        CHECK(renderPicture(scene, lit));
        sun->setShadow(true);
        CHECK(renderPicture(scene, shadowed));
        const int full = darkerPixels(lit, shadowed);
        lodTotal += lod;
        fullTotal += full;
        if (lod - full > lodWorst) lodWorst = lod - full;
    }
    printf("  hills under a high sun, 8 views: %d pixels in shadow with levels of detail, %d at "
           "full detail, worst view %d more\n",
            lodTotal, fullTotal, lodWorst);
    CHECK(lodWorst <= 100);
    CHECK(lodTotal <= fullTotal + 200);

    sun->queueFree();
    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(grey);
}

void testHook(Scene& scene)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId red = redMaterial(materials);
    ct::Vector<float> hills;
    hills.resize(257u * 257u);
    for (int y = 0; y < 257; ++y)
        for (int x = 0; x < 257; ++x)
            hills[static_cast<std::size_t>(y) * 257 + x] = 4.0f * rough(x / 2, y / 2);

    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setMaterial(red);
    TerrainImportData data = terrainData(257, 17, 33, 400.0f, hills.data());
    data.maxPixelError = 4.0f;
    CHECK(terrain->load(data));
    CHECK(near(terrain->getMaxPixelError(), 4.0f));
    scene.camera->setPerspective(60.0f, 0.5f, 3000.0f);

    const Math::Vec3 positions[3] = { Math::Vec3(0.0f, 40.0f, 230.0f),
        Math::Vec3(0.0f, 400.0f, 300.0f), Math::Vec3(0.0f, 1500.0f, 1500.0f) };
    int drawn[3] = { 0, 0, 0 };
    std::uint32_t triangles[3] = { 0, 0, 0 };
    for (int i = 0; i < 3; ++i)
    {
        place(scene, positions[i], Math::Vec3(0.0f, 0.0f, 0.0f));
        Picture picture;
        RenderStats stats;
        CHECK(renderPicture(scene, picture, &stats));
        drawn[i] = terrain->getNodesDrawn();
        triangles[i] = terrain->getTrianglesDrawn();
        CHECK(stats.triangles <= triangles[i]);
        CHECK(stats.instancesDrawn <= static_cast<std::uint32_t>(drawn[i]));
        printf("  camera (%.0f, %.0f, %.0f): %d nodes, per LOD", positions[i].x, positions[i].y,
                positions[i].z, drawn[i]);
        for (int lod = 0; lod < terrain->getLodLevelCount(); ++lod)
            printf(" %d", terrain->getNodesDrawnAtLod(lod));
        printf(", %u triangles, %u draw calls\n", triangles[i], stats.drawCalls);
        CHECK(drawn[i] > 0);
    }
    CHECK(triangles[0] > triangles[2] || drawn[0] != drawn[2]);

    terrain->setGlobalPosition(Math::Vec3(500.0f, 0.0f, 0.0f));
    {
        Frame frame(*scene.tree);
    }
    CHECK(near(terrain->getTerrain().getPosition().x, 500.0f));
    CHECK(near(terrain->getWorldHeightAt(Math::Vec3(500.0f, 0.0f, 0.0f)),
            terrain->getTerrain().getHeightAtWorldPosition(500.0f, 0.0f, 0.0f)));
    Math::Vec3 hit;
    CHECK(terrain->rayIntersects(
            Math::Ray(Math::Vec3(520.0f, 500.0f, 10.0f), Math::Vec3(0.0f, -1.0f, 0.0f)), hit));
    CHECK(near(hit.x, 520.0f, 0.05f) && near(hit.z, 10.0f, 0.05f));
    CHECK(near(hit.y, terrain->getWorldHeightAt(Math::Vec3(520.0f, 0.0f, 10.0f)), 0.2f));

    terrain->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(red);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "terrain render test";
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

    const MeshId box = Main::meshes()->create("box");
    CHECK(Main::meshes()->addSurface(box, colouredBox()));
    Main::materials()->defaultMaterial();
    {
        Frame frame(tree);
    }

    testCounts(scene);
    testColourAndStats(scene);
    testSilhouettes(scene);
    testMorph(scene);
    testHook(scene);
    testDepthOrder(scene, box);
    {
        WorldEnvironment* shaded = environment;
        EnvironmentSettings settings;
        settings.ambientSource = AmbientSource::Color;
        settings.ambientColor = Math::Vec3(0.15f, 0.15f, 0.15f);
        settings.reflectionSource = ReflectionSource::Disabled;
        shaded->setEnvironment(settings);
    }
    testShadows(scene, box);
    testShadowMorph(scene);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("terrain render ok\n");
    return 0;
}
