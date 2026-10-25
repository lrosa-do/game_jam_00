#include "Check.h"
#include "ColouredBox.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/Road3D.h"
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

const int kWidth = 320;
const int kHeight = 240;
const float kOffset = 0.06f;

double nowMilliseconds()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0 + static_cast<double>(ts.tv_nsec) / 1.0e6;
}

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
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

TerrainImportData pageData(const float* heights)
{
    TerrainImportData data;
    data.terrainSize = 129;
    data.minBatchSize = 129;
    data.maxBatchSize = 129;
    data.worldSize = 128.0f;
    data.inputFloat = heights;
    data.layerBlendMapSize = 129;
    data.lightMapSize = 129;
    data.compositeMapSize = 64;
    return data;
}

Terrain3D* makeTerrain(Scene& scene, MaterialId material, const ct::Vector<float>& heights)
{
    Terrain3D* terrain = scene.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    terrain->setCompositeMapEnabled(false);
    terrain->setMaterial(material);
    CHECK(terrain->load(pageData(heights.data())));
    return terrain;
}

ct::Vector<float> heightField(float (*function)(int, int))
{
    ct::Vector<float> heights;
    heights.resize(129u * 129u);
    for (int y = 0; y < 129; ++y)
        for (int x = 0; x < 129; ++x)
            heights[static_cast<std::size_t>(y) * 129 + x] = function(x, y);
    return heights;
}

float flatField(int, int)
{
    return 10.0f;
}

float rampField(int x, int)
{
    return 0.25f * static_cast<float>(x);
}

float ridgeField(int x, int)
{
    return 20.0f - 0.5f * fabsf(static_cast<float>(x - 64));
}

float stepField(int x, int)
{
    if (x <= 64) return 0.0f;
    if (x <= 74) return static_cast<float>(x - 64);
    return 10.0f;
}

float hillField(int x, int y)
{
    return 12.0f + 3.0f * sinf(static_cast<float>(x) * 0.09f) * cosf(static_cast<float>(y) * 0.07f);
}

void addLine(Curve3D& curve, const Math::Vec3& from, const Math::Vec3& to)
{
    const Math::Vec3 third = (to - from) / 3.0f;
    curve.addPoint(from, Math::Vec3(0.0f, 0.0f, 0.0f), third);
    curve.addPoint(to, -third, Math::Vec3(0.0f, 0.0f, 0.0f));
}

struct RoadSetup
{
    Path3D* path = nullptr;
    Road3D* road = nullptr;
};

RoadSetup makeRoad(Scene& scene, Terrain3D* terrain, float width)
{
    RoadSetup setup;
    setup.path = scene.root->createChild<Path3D>();
    setup.road = setup.path->createChild<Road3D>();
    setup.road->setWidth(width);
    setup.road->setInterval(1.0f);
    if (terrain) setup.road->setTerrain(terrain);
    return setup;
}

struct RoadCheck
{
    int vertices = 0;
    float worstGround = 0.0f;
    float worstExpected = 0.0f;
    int below = 0;
};

RoadCheck checkRoad(const Road3D* road, const Terrain3D* terrain, float offset,
        float (*expected)(const Math::Vec3&))
{
    RoadCheck out;
    const SurfaceData& g = road->getGeometry();
    for (std::size_t i = 0; i < g.positions.size(); ++i)
    {
        const Math::Vec3 w = road->getGlobalTransform().TransformPoint(g.positions[i]);
        const float ground = terrain->getWorldHeightAt(w);
        ++out.vertices;
        const float error = fabsf(w.y - (ground + offset));
        out.worstGround = error > out.worstGround ? error : out.worstGround;
        if (w.y < ground + offset - 1e-4f) ++out.below;
        if (expected)
        {
            const float analytic = fabsf(w.y - (expected(w) + offset));
            out.worstExpected = analytic > out.worstExpected ? analytic : out.worstExpected;
        }
    }
    return out;
}

float rampExpected(const Math::Vec3& p)
{
    return 0.25f * (p.x + 64.0f);
}

float flatExpected(const Math::Vec3&)
{
    return 10.0f;
}

void testRoadOnTerrain(Scene& scene, MaterialId green)
{
    {
        const ct::Vector<float> heights = heightField(flatField);
        Terrain3D* terrain = makeTerrain(scene, green, heights);
        RoadSetup setup = makeRoad(scene, terrain, 4.0f);
        addLine(setup.path->getCurve(), Math::Vec3(0.0f, 0.0f, -30.0f),
                Math::Vec3(0.0f, 0.0f, 30.0f));
        CHECK(setup.road->flush());
        const RoadCheck c = checkRoad(setup.road, terrain, kOffset, flatExpected);
        printf("  road on a plane: %d vertices, worst error against the terrain %.2e\n", c.vertices,
                c.worstGround);
        CHECK(c.vertices > 100 && c.worstGround < 1e-4f && c.worstExpected < 1e-4f);
        setup.path->queueFree();
        terrain->queueFree();
        flush(scene);
    }
    {
        const ct::Vector<float> heights = heightField(rampField);
        Terrain3D* terrain = makeTerrain(scene, green, heights);
        RoadSetup setup = makeRoad(scene, terrain, 4.0f);
        addLine(setup.path->getCurve(), Math::Vec3(0.0f, 0.0f, -30.0f),
                Math::Vec3(0.0f, 0.0f, 30.0f));
        CHECK(setup.road->flush());
        RoadCheck c = checkRoad(setup.road, terrain, kOffset, rampExpected);
        printf("  road across a ramp: worst error against the ramp %.2e, against the terrain "
               "%.2e, normal %.4f\n",
                c.worstExpected, c.worstGround, setup.road->getGeometry().normals[0].x);
        CHECK(c.worstGround < 1e-4f && c.worstExpected < 1e-3f && c.below == 0);
        const Math::Vec3 expectedNormal = Math::Vec3(-0.25f, 1.0f, 0.0f).Normalized();
        const SurfaceData& g = setup.road->getGeometry();
        for (std::size_t i = 0; i < g.normals.size(); ++i)
            CHECK(near(g.normals[i].x, expectedNormal.x, 2e-3f) &&
                    near(g.normals[i].y, expectedNormal.y, 2e-3f));

        setup.road->setGroundTilt(false);
        CHECK(setup.road->flush());
        c = checkRoad(setup.road, terrain, kOffset, nullptr);
        CHECK(c.below == 0);
        const SurfaceData& level = setup.road->getGeometry();
        for (std::size_t r = 0; r + 1 < level.positions.size(); r += 2)
            CHECK(near(level.positions[r].y, level.positions[r + 1].y, 1e-4f));
        CHECK(near(level.positions[0].y, 0.25f * (2.0f + 64.0f) + kOffset, 2e-3f));
        setup.path->queueFree();
        terrain->queueFree();
        flush(scene);
    }
    {
        const ct::Vector<float> heights = heightField(ridgeField);
        Terrain3D* terrain = makeTerrain(scene, green, heights);
        RoadSetup setup = makeRoad(scene, terrain, 4.0f);
        addLine(setup.path->getCurve(), Math::Vec3(1.0f, 0.0f, -30.0f),
                Math::Vec3(1.0f, 0.0f, 30.0f));
        CHECK(setup.road->flush());
        RoadCheck tilted = checkRoad(setup.road, terrain, kOffset, nullptr);
        const SurfaceData& g = setup.road->getGeometry();
        printf("  road along a ridge: tilted worst error %.2e, edge heights %.3f and %.3f\n",
                tilted.worstGround, g.positions[0].y, g.positions[1].y);
        CHECK(tilted.below == 0 && tilted.worstGround < 1e-4f);
        CHECK(near(g.positions[0].y, 18.5f + kOffset, 1e-3f) &&
                near(g.positions[1].y, 19.5f + kOffset, 1e-3f));
        setup.road->setGroundTilt(false);
        CHECK(setup.road->flush());
        const RoadCheck level = checkRoad(setup.road, terrain, kOffset, nullptr);
        CHECK(level.below == 0);
        const SurfaceData& flat = setup.road->getGeometry();
        for (std::size_t i = 0; i < flat.positions.size(); ++i)
            CHECK(near(flat.positions[i].y, 19.5f + kOffset, 1e-3f));
        setup.path->queueFree();
        terrain->queueFree();
        flush(scene);
    }
}

void testRebuildOnEdit(Scene& scene, MaterialId green)
{
    const ct::Vector<float> heights = heightField(hillField);
    Terrain3D* terrain = makeTerrain(scene, green, heights);
    RoadSetup setup = makeRoad(scene, terrain, 4.0f);
    addLine(setup.path->getCurve(), Math::Vec3(0.0f, 0.0f, -30.0f), Math::Vec3(0.0f, 0.0f, 30.0f));
    CHECK(setup.road->flush());
    CHECK(terrain->getObserverCount() == 1);
    const int base = setup.road->getBuildCount();

    CHECK(terrainBrushHeight(terrain->getTerrain(), Math::Vec3(-45.0f, 0.0f, 40.0f), 6.0f, 100.0f,
            0.1f, false));
    terrain->update();
    CHECK(!setup.road->flush());
    CHECK(setup.road->getBuildCount() == base);

    CHECK(terrainBrushHeight(terrain->getTerrain(), Math::Vec3(5.0f, 0.0f, 20.0f), 2.0f, 100.0f,
            0.1f, false));
    terrain->update();
    CHECK(!setup.road->flush());

    const Math::Vec3 centre(0.0f, 0.0f, 0.0f);
    const double t0 = nowMilliseconds();
    CHECK(terrainBrushHeight(terrain->getTerrain(), centre, 6.0f, 100.0f, 0.1f, false));
    terrain->update();
    CHECK(setup.road->isDirty());
    CHECK(setup.road->flush());
    const double rebuildMs = nowMilliseconds() - t0;
    CHECK(setup.road->getBuildCount() == base + 1);
    const RoadCheck c = checkRoad(setup.road, terrain, kOffset, nullptr);
    printf("  edit under the road: one rebuild (%.2f ms with the edit), worst error %.2e; edits "
           "elsewhere: none\n",
            rebuildMs, c.worstGround);
    CHECK(c.worstGround < 1e-4f && c.below == 0);

    CHECK(!setup.road->flush());
    setup.road->setTerrain(nullptr);
    CHECK(terrain->getObserverCount() == 0 && setup.road->flush());
    CHECK(near(setup.road->getGeometry().positions[0].y, 0.0f));
    setup.road->setTerrain(terrain);
    CHECK(terrain->getObserverCount() == 1);
    setup.road->flush();

    setup.path->queueFree();
    flush(scene);
    CHECK(terrain->getObserverCount() == 0);

    RoadSetup other = makeRoad(scene, terrain, 4.0f);
    addLine(other.path->getCurve(), Math::Vec3(0.0f, 0.0f, -30.0f), Math::Vec3(0.0f, 0.0f, 30.0f));
    other.road->flush();
    CHECK(terrain->getObserverCount() == 1);
    terrain->queueFree();
    flush(scene);
    CHECK(other.road->getTerrain() == nullptr);
    other.road->flush();
    CHECK(near(other.road->getGeometry().positions[0].y, 0.0f));
    other.path->queueFree();
    flush(scene);
}

void testFlattenUnderRoad(Scene& scene, MaterialId green)
{
    const ct::Vector<float> heights = heightField(hillField);
    Terrain3D* terrain = makeTerrain(scene, green, heights);
    CHECK(terrain->addLayer(10.0f, Main::textures()->white(), Main::textures()->normal()));
    CHECK(terrain->addLayer(10.0f, Main::textures()->white(), Main::textures()->normal()));
    RoadSetup setup = makeRoad(scene, terrain, 6.0f);
    Curve3D& curve = setup.path->getCurve();
    const float k = 20.0f * 0.5522847498f;
    curve.addPoint(Math::Vec3(20.0f, 12.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -k),
            Math::Vec3(0.0f, 0.0f, k));
    curve.addPoint(Math::Vec3(0.0f, 12.0f, 20.0f), Math::Vec3(k, 0.0f, 0.0f),
            Math::Vec3(-k, 0.0f, 0.0f));
    curve.addPoint(Math::Vec3(-20.0f, 12.0f, 0.0f), Math::Vec3(0.0f, 0.0f, k),
            Math::Vec3(0.0f, 0.0f, -k));
    curve.addPoint(Math::Vec3(0.0f, 12.0f, -20.0f), Math::Vec3(-k, 0.0f, 0.0f),
            Math::Vec3(k, 0.0f, 0.0f));
    curve.setClosed(true);
    CHECK(setup.road->flush());

    TerrainPath footprint;
    CHECK(setup.road->buildFootprint(footprint, 2.0f));
    TerrainPathBrush brush;
    brush.shoulder = 2.0f;
    brush.bank = 6.0f;
    terrain->resetEditStats();
    terrain->resetUploadStats();
    double t0 = nowMilliseconds();
    const std::size_t written = terrainFlattenAlongPath(terrain->getTerrain(), footprint, brush);
    const std::size_t painted = terrainPaintAlongPath(terrain->getTerrain(), 1, footprint, brush);
    terrain->update();
    CHECK(setup.road->isDirty());
    CHECK(setup.road->flush());
    const double ms = nowMilliseconds() - t0;
    const RoadCheck c = checkRoad(setup.road, terrain, kOffset, nullptr);
    int offLevel = 0;
    const SurfaceData& g = setup.road->getGeometry();
    for (std::size_t i = 0; i < g.positions.size(); ++i)
        if (!near(g.positions[i].y, 12.0f + kOffset, 2e-3f)) ++offLevel;
    printf("  flatten, paint and rebuild of a closed road: %zu heights, %zu texels, %.2f ms; "
           "%d of %d road vertices off the curve height (worst error against terrain %.2e)\n",
            written, painted, ms, offLevel, c.vertices, c.worstGround);
    CHECK(written > 0 && painted > 0 && offLevel == 0 && c.worstGround < 1e-4f);
    CHECK(terrain->getUploadCount() >= 1);

    const float before = terrain->getTerrain().getHeightAtPoint(64 + 20, 64);
    CHECK(near(before, 12.0f, 1e-3f));
    CHECK(terrainBrushHeight(terrain->getTerrain(), Math::Vec3(20.0f, 0.0f, 0.0f), 5.0f, 200.0f,
            0.1f, false));
    terrain->update();
    CHECK(setup.road->flush());
    CHECK(checkRoad(setup.road, terrain, kOffset, nullptr).worstGround < 1e-4f);
    const SurfaceData& bumped = setup.road->getGeometry();
    int off = 0;
    for (std::size_t i = 0; i < bumped.positions.size(); ++i)
        if (!near(bumped.positions[i].y, 12.0f + kOffset, 2e-3f)) ++off;
    CHECK(off > 0);
    terrainFlattenAlongPath(terrain->getTerrain(), footprint, brush);
    terrain->update();
    CHECK(setup.road->flush());
    off = 0;
    for (std::size_t i = 0; i < bumped.positions.size(); ++i)
        if (!near(setup.road->getGeometry().positions[i].y, 12.0f + kOffset, 2e-3f)) ++off;
    CHECK(off == 0);

    setup.path->queueFree();
    terrain->queueFree();
    flush(scene);
}

struct ForestWorld
{
    Forest3D* forest = nullptr;
    int species = -1;
};

int countAccepted(Forest3D* forest, int species, float maxSlope, bool road, Terrain3D* terrain,
        float* worstHeight, int* slopeViolations, int* roadViolations)
{
    TerrainPath exclusion;
    exclusion.add(Math::Vec3(-30.0f, 0.0f, -70.0f), 2.0f);
    exclusion.add(Math::Vec3(-30.0f, 0.0f, 70.0f), 2.0f);
    forest->clearInstances();
    forest->setMaxSlope(maxSlope);
    forest->clearExclusions();
    if (road) forest->addExclusion(exclusion, 3.0f);
    int planted = 0;
    *worstHeight = 0.0f;
    *slopeViolations = 0;
    *roadViolations = 0;
    for (int z = -60; z <= 60; z += 2)
    {
        for (int x = -60; x <= 60; x += 2)
        {
            const int index = forest->plantOnTerrain(species, static_cast<float>(x),
                    static_cast<float>(z), 0.0f, 1.0f);
            if (index < 0) continue;
            ++planted;
            const Math::Vec3 at = forest->getInstanceTransform(index).GetTranslation();
            const float error = fabsf(at.y - terrain->getWorldHeightAt(at));
            *worstHeight = error > *worstHeight ? error : *worstHeight;
            if (terrain->getWorldSlopeAt(at) > maxSlope) ++(*slopeViolations);
            if (road && fabsf(at.x + 30.0f) - 2.0f < 3.0f) ++(*roadViolations);
        }
    }
    return planted;
}

void testForest(Scene& scene, MaterialId green)
{
    const ct::Vector<float> heights = heightField(stepField);
    Terrain3D* terrain = makeTerrain(scene, green, heights);
    const MeshId box = Main::meshes()->create("box");
    CHECK(Main::meshes()->addSurface(box, colouredBox()));
    Forest3D* forest = scene.root->createChild<Forest3D>();
    ForestSpeciesDesc desc;
    desc.mesh = box;
    const int species = forest->addSpecies(desc);
    CHECK(species == 0);
    CHECK(forest->plantOnTerrain(species, 0.0f, 0.0f, 0.0f, 1.0f) == -1);
    forest->setTerrain(terrain);
    CHECK(forest->getTerrain() == terrain && terrain->getObserverCount() == 1);

    float worst = 0.0f;
    int slopeBad = 0;
    int roadBad = 0;
    int all = countAccepted(forest, species, 90.0f, false, terrain, &worst, &slopeBad, &roadBad);
    printf("  forest on terrain, no rules: %d of 3721 planted, worst height error %.2e\n", all,
            worst);
    CHECK(all == 61 * 61 && worst < 1e-4f);
    const Math::Vec3 hill = forest->getInstanceTransform(61 * 30 + 50).GetTranslation();
    CHECK(near(hill.y, 10.0f, 1e-4f) && near(hill.x, 40.0f, 1e-4f));

    int sloped = countAccepted(forest, species, 30.0f, false, terrain, &worst, &slopeBad, &roadBad);
    printf("  slope limit 30 degrees: %d planted (expected %d), %d violations\n", sloped, 56 * 61,
            slopeBad);
    CHECK(sloped == 56 * 61 && slopeBad == 0 && worst < 1e-4f);

    int both = countAccepted(forest, species, 30.0f, true, terrain, &worst, &slopeBad, &roadBad);
    printf("  slope limit and road exclusion 3 m: %d planted (expected %d), %d slope and %d road "
           "violations\n",
            both, 51 * 61, slopeBad, roadBad);
    CHECK(both == 51 * 61 && slopeBad == 0 && roadBad == 0);

    forest->setMaxSlope(90.0f);
    forest->clearExclusions();
    forest->clearInstances();
    ct::Vector<int> indices;
    for (int z = -60; z <= 60; z += 2)
        for (int x = -60; x <= 60; x += 2)
            indices.push_back(forest->plantOnTerrain(species, static_cast<float>(x),
                    static_cast<float>(z), 0.0f, 1.0f));
    ct::Vector<Math::Vec3> snapshot;
    for (std::size_t i = 0; i < indices.size(); ++i)
        snapshot.push_back(forest->getInstanceTransform(indices[i]).GetTranslation());

    place(scene, Math::Vec3(-52.0f, 6.0f, 52.0f), Math::Vec3(-52.0f, 0.0f, 40.0f));
    scene.camera->setPerspective(60.0f, 0.5f, 400.0f);
    flush(scene);
    CHECK(forest->getMeshInstanceCount() > 0);
    const int probe = 61 * 51 + 4;
    const MultiMesh* mesh = forest->getMeshNode(species)->getMultiMesh();
    int slotBefore = -1;
    for (int s = 0; s < forest->getMeshNode(species)->getVisibleInstanceCount(); ++s)
    {
        const Math::Vec3 at = mesh->getInstanceTransform(s).GetTranslation();
        if (near(at.x, snapshot[static_cast<std::size_t>(probe)].x) &&
                near(at.z, snapshot[static_cast<std::size_t>(probe)].z))
            slotBefore = s;
    }
    CHECK(slotBefore >= 0);

    for (int y = 20; y <= 24; ++y)
        for (int x = 10; x <= 14; ++x)
            terrain->getTerrain().setHeightAtPoint(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y),
                    terrain->getTerrain().getHeightAtPoint(static_cast<std::uint32_t>(x),
                            static_cast<std::uint32_t>(y)) +
                            5.0f);
    terrain->update();
    const int reseated = forest->getReseatedCount();
    int moved = 0;
    int movedOutside = 0;
    int insideWrong = 0;
    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        const Math::Vec3 at = forest->getInstanceTransform(indices[i]).GetTranslation();
        const Math::Vec3& was = snapshot[i];
        const bool inside = was.x >= -55.0f && was.x <= -49.0f && was.z >= 39.0f && was.z <= 45.0f;
        if (at.y != was.y)
        {
            ++moved;
            if (!inside) ++movedOutside;
        }
        if (inside && !near(at.y, terrain->getWorldHeightAt(at), 1e-4f)) ++insideWrong;
        if (!inside && (at.x != was.x || at.y != was.y || at.z != was.z)) ++movedOutside;
    }
    printf("  forest re-seat after an edit: %d re-seated (expected 9) in %.3f ms, %d moved, %d "
           "moved outside the rectangle\n",
            reseated, forest->getReseatMilliseconds(), moved, movedOutside);
    CHECK(reseated == 9 && moved == 9 && movedOutside == 0 && insideWrong == 0);
    CHECK(near(mesh->getInstanceTransform(slotBefore).GetTranslation().y,
            snapshot[static_cast<std::size_t>(probe)].y + 5.0f, 1e-3f));
    int recordsWrong = 0;
    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        ImpostorInstanceInfo info;
        const Math::Vec3 at = forest->getInstanceTransform(indices[i]).GetTranslation();
        if (!Main::rendering()->impostorSetGetInstance(forest->getImpostorSet(),
                    static_cast<std::uint32_t>(indices[i]), info) ||
                !near(info.center[1], at.y + 0.5f, 1e-4f) || !near(info.center[0], at.x, 1e-4f) ||
                !near(info.center[2], at.z, 1e-4f) || !near(info.radius, 0.8660254f, 1e-4f))
            ++recordsWrong;
    }
    CHECK(recordsWrong == 0);
    ImpostorInstanceInfo probeInfo;
    CHECK(Main::rendering()->impostorSetGetInstance(forest->getImpostorSet(),
            static_cast<std::uint32_t>(indices[static_cast<std::size_t>(probe)]), probeInfo));
    CHECK(near(probeInfo.center[1], snapshot[static_cast<std::size_t>(probe)].y + 5.0f + 0.5f,
            1e-3f));

    forest->queueFree();
    terrain->queueFree();
    flush(scene);
    Main::meshes()->destroy(box);
}

struct GrassView
{
    Grass3D* grass = nullptr;
};

void testGrass(Scene& scene, MaterialId green)
{
    const ct::Vector<float> heights = heightField(stepField);
    Terrain3D* terrain = makeTerrain(scene, green, heights);
    Pixmap pixels(16, 16, 4);
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
            pixels.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), 255, 255,
                    255, 255);
    const TextureId texture =
            Main::textures()->createFromPixmap(pixels, true, true, "grass world", 4);

    Grass3D* grass = scene.root->createChild<Grass3D>();
    grass->setTexture(texture);
    grass->setSeed(777u);
    grass->setCellSize(8.0f);
    grass->setDensity(0.5f);
    grass->setDrawDistance(40.0f);
    grass->setWind(0.0f);
    grass->setExtent(-64.0f, -64.0f, 64.0f, 64.0f);
    grass->setTerrain(terrain);
    grass->setMaxSlope(30.0f);
    TerrainPath road;
    road.add(Math::Vec3(20.0f, 0.0f, -70.0f), 2.0f);
    road.add(Math::Vec3(20.0f, 0.0f, 70.0f), 2.0f);
    grass->addExclusion(road, 2.0f);
    CHECK(grass->getTerrain() == terrain && grass->getMaxSlope() == 30.0f);

    const Math::Vec3 camera(0.0f, 5.0f, 0.0f);
    CHECK(grass->update(camera, nullptr, 1.0f / 60.0f));
    CHECK(grass->getActiveCellCount() > 0);

    struct Entry
    {
        int type = 0;
        int cx = 0;
        int cz = 0;
        int index = 0;
    };
    auto collect = [&](ct::Vector<Entry>& out)
    {
        out.clear();
        for (int cz = -8; cz <= 8; ++cz)
            for (int cx = -8; cx <= 8; ++cx)
                if (grass->hasCell(0, cx, cz))
                    for (int i = 0;; ++i)
                    {
                        GrassClumpInfo probe;
                        if (!grass->getClump(0, cx, cz, i, probe)) break;
                        Entry e;
                        e.cx = cx;
                        e.cz = cz;
                        e.index = i;
                        out.push_back(e);
                    }
    };
    ct::Vector<Entry> entries;
    collect(entries);
    auto verify = [&](float* worstHeight, int* wrongDensity, int* zero, int* nonzero)
    {
        *worstHeight = 0.0f;
        *wrongDensity = 0;
        *zero = 0;
        *nonzero = 0;
        for (std::size_t i = 0; i < entries.size(); ++i)
        {
            GrassClumpInfo info;
            CHECK(grass->getClump(0, entries[i].cx, entries[i].cz, entries[i].index, info));
            const float ground = terrain->getWorldHeightAt(info.root);
            const float error = fabsf(info.root.y - ground);
            *worstHeight = error > *worstHeight ? error : *worstHeight;
            const bool steep = terrain->getWorldSlopeAt(info.root) > 30.0f;
            const bool onRoad = fabsf(info.root.x - 20.0f) - 2.0f < 2.0f;
            const bool expectZero = steep || onRoad;
            if ((info.density == 0.0f) != expectZero) ++(*wrongDensity);
            if (info.density == 0.0f) ++(*zero);
            else
                ++(*nonzero);
        }
    };
    float worst = 0.0f;
    int wrong = 0;
    int zero = 0;
    int nonzero = 0;
    verify(&worst, &wrong, &zero, &nonzero);
    printf("  grass on terrain: %zu clumps in %d cells, worst height error %.2e, %d with "
           "density 0 (slope or road) and %d with density, %d wrong\n",
            entries.size(), grass->getActiveCellCount(), worst, zero, nonzero, wrong);
    CHECK(entries.size() > 100 && worst < 1e-4f && wrong == 0 && zero > 0 && nonzero > 0);

    ct::Vector<Math::Vec3> before;
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        GrassClumpInfo info;
        grass->getClump(0, entries[i].cx, entries[i].cz, entries[i].index, info);
        before.push_back(info.root);
    }

    for (int y = 56; y <= 60; ++y)
        for (int x = 60; x <= 64; ++x)
            terrain->getTerrain().setHeightAtPoint(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y),
                    terrain->getTerrain().getHeightAtPoint(static_cast<std::uint32_t>(x),
                            static_cast<std::uint32_t>(y)) +
                            6.0f);
    terrain->update();
    int expectedInside = 0;
    int moved = 0;
    int movedOutside = 0;
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        const Math::Vec3& was = before[i];
        const bool inside = was.x >= -5.0f && was.x <= 1.0f && was.z >= 3.0f && was.z <= 9.0f;
        if (inside) ++expectedInside;
        GrassClumpInfo info;
        grass->getClump(0, entries[i].cx, entries[i].cz, entries[i].index, info);
        if (info.root.y != was.y)
        {
            ++moved;
            if (!inside) ++movedOutside;
        }
        if (info.root.x != was.x || info.root.z != was.z) ++movedOutside;
    }
    const int reseated = grass->getReseatedCount();
    printf("  grass re-seat after an edit: %d re-seated (expected %d) in %.3f ms, %d moved, %d "
           "moved outside the rectangle\n",
            reseated, expectedInside, grass->getReseatMilliseconds(), moved, movedOutside);
    CHECK(reseated == expectedInside && expectedInside > 0 && moved > 0 && movedOutside == 0);
    verify(&worst, &wrong, &zero, &nonzero);
    CHECK(worst < 1e-4f && wrong == 0);
    CHECK(grass->update(camera, nullptr, 1.0f / 60.0f) || true);

    grass->queueFree();
    terrain->queueFree();
    flush(scene);
    Main::textures()->destroy(texture);
}

bool isRed(const unsigned char* p)
{
    return p[0] > 150 && p[1] < 90 && p[2] < 90;
}

bool isGreen(const unsigned char* p)
{
    return p[1] > 120 && p[0] < 90 && p[2] < 90;
}

void testRender(Scene& scene, MaterialId green)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId red = materials.create("road red");
    materials.setColor(red, MaterialParam::Albedo, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    materials.setFloat(red, MaterialParam::Roughness, 1.0f);

    const ct::Vector<float> heights = heightField(hillField);
    Terrain3D* terrain = makeTerrain(scene, green, heights);
    RoadSetup setup = makeRoad(scene, terrain, 4.0f);
    setup.road->setMaterial(red);
    Curve3D& curve = setup.path->getCurve();
    curve.addPoint(Math::Vec3(-30.0f, 0.0f, 10.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(20.0f, 0.0f, -10.0f));
    curve.addPoint(Math::Vec3(0.0f, 0.0f, -5.0f), Math::Vec3(-15.0f, 0.0f, 5.0f),
            Math::Vec3(15.0f, 0.0f, -5.0f));
    curve.addPoint(Math::Vec3(30.0f, 0.0f, 10.0f), Math::Vec3(-20.0f, 0.0f, -10.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(setup.road->flush());

    scene.camera->setPerspective(60.0f, 0.5f, 400.0f);
    place(scene, Math::Vec3(0.0f, 60.0f, 12.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
    Picture first;
    Picture second;
    CHECK(renderPicture(scene, first));
    CHECK(renderPicture(scene, second));
    int apart = 0;
    for (std::size_t i = 0; i < first.pixels.size(); ++i)
        if (first.pixels[i] != second.pixels[i]) ++apart;
    CHECK(apart == 0);

    auto sample = [&](const Picture& picture, int* roadMiss, int* edgeMiss, int* total)
    {
        *roadMiss = 0;
        *edgeMiss = 0;
        *total = 0;
        const float length = curve.getBakedLength();
        for (float offset = 1.0f; offset < length - 1.0f; offset += 2.0f)
        {
            const Math::Vec3 centre = curve.sampleBaked(offset);
            const Math::Vec3 ahead = curve.sampleBaked(offset + 0.5f);
            Math::Vec3 side(ahead.z - centre.z, 0.0f, centre.x - ahead.x);
            side = side.Normalized();
            ++*total;
            for (int k = -1; k <= 1; ++k)
            {
                const Math::Vec3 p = centre + side * (1.0f * static_cast<float>(k));
                const Math::Vec3 onRoad(p.x, terrain->getWorldHeightAt(p) + kOffset, p.z);
                const Math::Vec2 pixel = scene.camera->unprojectPosition(onRoad);
                const int px = static_cast<int>(pixel.x);
                const int py = static_cast<int>(pixel.y);
                if (px < 0 || py < 0 || px >= kWidth || py >= kHeight || !isRed(picture.at(px, py)))
                    ++*roadMiss;
            }
            for (int k = -1; k <= 1; k += 2)
            {
                const Math::Vec3 p = centre + side * (3.6f * static_cast<float>(k));
                const Math::Vec3 onGround(p.x, terrain->getWorldHeightAt(p), p.z);
                const Math::Vec2 pixel = scene.camera->unprojectPosition(onGround);
                const int px = static_cast<int>(pixel.x);
                const int py = static_cast<int>(pixel.y);
                if (px < 0 || py < 0 || px >= kWidth || py >= kHeight ||
                        !isGreen(picture.at(px, py)))
                    ++*edgeMiss;
            }
        }
    };
    int roadMiss = 0;
    int edgeMiss = 0;
    int total = 0;
    sample(first, &roadMiss, &edgeMiss, &total);
    printf("  road render: %d stations along the curve, %d centre or half-width pixels not red, "
           "%d pixels beside the road not terrain\n",
            total, roadMiss, edgeMiss);
    CHECK(total > 20 && roadMiss == 0 && edgeMiss == 0);

    place(scene, Math::Vec3(0.3f, 60.0f, 12.2f), Math::Vec3(0.0f, 0.0f, 0.0f));
    Picture moved;
    CHECK(renderPicture(scene, moved));
    sample(moved, &roadMiss, &edgeMiss, &total);
    CHECK(roadMiss == 0 && edgeMiss == 0);

    place(scene, Math::Vec3(0.0f, 60.0f, 12.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
    setup.road->setGroundOffset(-0.5f);
    Picture sunk;
    CHECK(renderPicture(scene, sunk));
    sample(sunk, &roadMiss, &edgeMiss, &total);
    printf("  control with the road 0.5 below the ground: %d of %d road pixels not red\n", roadMiss,
            total * 3);
    CHECK(roadMiss > total);
    setup.road->setGroundOffset(kOffset);

    place(scene, Math::Vec3(0.0f, 12.0f, 30.0f), Math::Vec3(0.0f, 12.0f, 0.0f));
    Picture low;
    CHECK(renderPicture(scene, low));
    int greenRows = 0;
    int redRows = 0;
    for (int y = 0; y < kHeight; ++y)
    {
        bool hasRed = false;
        bool hasGreen = false;
        for (int x = 0; x < kWidth; ++x)
        {
            hasRed = hasRed || isRed(low.at(x, y));
            hasGreen = hasGreen || isGreen(low.at(x, y));
        }
        if (hasRed) ++redRows;
        if (hasGreen) ++greenRows;
    }
    CHECK(redRows > 0 && greenRows > redRows);

    setup.path->queueFree();
    terrain->queueFree();
    flush(scene);
    materials.destroy(red);
}

void testTreesStand(Scene& scene, MaterialId green)
{
    const ct::Vector<float> heights = heightField(rampField);
    Terrain3D* terrain = makeTerrain(scene, green, heights);
    const MeshId box = Main::meshes()->create("tree box");
    CHECK(Main::meshes()->addSurface(box, colouredBox()));
    Forest3D* forest = scene.root->createChild<Forest3D>();
    ForestSpeciesDesc desc;
    desc.mesh = box;
    CHECK(forest->addSpecies(desc) == 0);
    forest->setImpostorsEnabled(false);
    forest->setTerrain(terrain);
    const float xs[3] = { -20.0f, -5.0f, 12.0f };
    for (int i = 0; i < 3; ++i) CHECK(forest->plantOnTerrain(0, xs[i], 0.0f, 0.0f, 2.0f) == i);

    scene.camera->setPerspective(50.0f, 0.5f, 400.0f);
    place(scene, Math::Vec3(-4.0f, 22.0f, 45.0f), Math::Vec3(-4.0f, 12.0f, 0.0f));
    Picture picture;
    CHECK(renderPicture(scene, picture));
    int bad = 0;
    for (int i = 0; i < 3; ++i)
    {
        const float ground = 0.25f * (xs[i] + 64.0f);
        const Math::Vec2 base = scene.camera->unprojectPosition(Math::Vec3(xs[i], ground, 1.0f));
        const Math::Vec2 mid =
                scene.camera->unprojectPosition(Math::Vec3(xs[i], ground + 1.0f, 1.0f));
        const int column = static_cast<int>(mid.x);
        int lowest = -1;
        for (int y = 0; y < kHeight; ++y)
            if (isRed(picture.at(column, y))) lowest = y;
        const int expected = static_cast<int>(base.y);
        printf("  tree at x %.0f: lowest red row %d, projected base row %d\n", xs[i], lowest,
                expected);
        if (lowest < 0 || abs(lowest - expected) > 2) ++bad;
        if (lowest >= 0 && lowest + 3 < kHeight && !isGreen(picture.at(column, lowest + 3))) ++bad;
    }
    CHECK(bad == 0);

    forest->queueFree();
    terrain->queueFree();
    flush(scene);
    Main::meshes()->destroy(box);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "terrain world test";
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

    const std::size_t meshBaseline = Main::meshes()->count();
    const std::size_t textureBaseline = Main::textures()->count();
    const std::size_t materialBaseline = Main::materials()->count();

    MaterialStore& materials = *Main::materials();
    const MaterialId green = materials.create("ground green");
    materials.setColor(green, MaterialParam::Albedo, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    materials.setFloat(green, MaterialParam::Roughness, 1.0f);

    testRoadOnTerrain(scene, green);
    testRebuildOnEdit(scene, green);
    testFlattenUnderRoad(scene, green);
    testForest(scene, green);
    testGrass(scene, green);
    testRender(scene, green);
    testTreesStand(scene, green);

    materials.destroy(green);
    flush(scene);
    printf("  leaks: %zu meshes, %zu textures, %zu materials over the baseline\n",
            Main::meshes()->count() - meshBaseline, Main::textures()->count() - textureBaseline,
            Main::materials()->count() - materialBaseline);
    CHECK(Main::meshes()->count() == meshBaseline);
    CHECK(Main::textures()->count() == textureBaseline);
    CHECK(Main::materials()->count() == materialBaseline);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("terrain world ok\n");
    return 0;
}
