#include "Check.h"
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
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 128;
const int kHeight = 96;
const float kPi = 3.14159265358979f;
const float kDt = 1.0f / 60.0f;

bool near(float a, float b, float tolerance = 0.0005f)
{
    return fabsf(a - b) < tolerance;
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

struct Image
{
    unsigned char pixels[kWidth * kHeight * 4];

    bool read()
    {
        prisma::Rect rect;
        rect.x = 0;
        rect.y = 0;
        rect.width = kWidth;
        rect.height = kHeight;
        return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixels);
    }

    const unsigned char* at(int x, int y) const
    {
        return pixels + (y * kWidth + x) * 4;
    }

    bool isClear(int x, int y) const
    {
        const unsigned char* p = at(x, y);
        return abs(p[0] - 26) <= 3 && abs(p[1] - 51) <= 3 && abs(p[2] - 77) <= 3;
    }
};

struct Bounds
{
    int minX = kWidth;
    int maxX = -1;
    int minY = kHeight;
    int maxY = -1;
    int count = 0;
};

Bounds grassBounds(const Image& image)
{
    Bounds bounds;
    for (int y = 0; y < kHeight; ++y)
    {
        for (int x = 0; x < kWidth; ++x)
        {
            if (image.isClear(x, y)) continue;
            ++bounds.count;
            if (x < bounds.minX) bounds.minX = x;
            if (x > bounds.maxX) bounds.maxX = x;
            if (y < bounds.minY) bounds.minY = y;
            if (y > bounds.maxY) bounds.maxY = y;
        }
    }
    return bounds;
}

Bounds grassRows(const Image& image, int minPixels)
{
    Bounds bounds;
    for (int y = 0; y < kHeight; ++y)
    {
        int count = 0;
        for (int x = 0; x < kWidth; ++x)
            if (!image.isClear(x, y)) ++count;
        if (count < minPixels) continue;
        bounds.count += count;
        if (y < bounds.minY) bounds.minY = y;
        if (y > bounds.maxY) bounds.maxY = y;
    }
    return bounds;
}

struct Scene
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Camera3D* camera = nullptr;
};

bool grab(Scene& scene, Image& image)
{
    Frame frame(*scene.tree);
    return image.read();
}

void place(Scene& scene, const Math::Vec3& position, const Math::Vec3& target)
{
    scene.camera->setGlobalTransform(
            Math::Mat4::LookAt(position, target, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine());
}

TextureId makeTexture(TextureStore& textures, int topAlpha, int bottomAlpha, const char* name)
{
    Pixmap pixels(16, 16, 4);
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
        {
            const int alpha = y < 8 ? topAlpha : bottomAlpha;
            pixels.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), 255, 255,
                    255, static_cast<std::uint8_t>(alpha));
        }
    return textures.createFromPixmap(pixels, true, true, name, 4);
}

void testPure()
{
    int cx = 0;
    int cz = 0;
    Grass3D::cellOf(4.0f, 4.0f, 8.0f, cx, cz);
    CHECK(cx == 0 && cz == 0);
    Grass3D::cellOf(8.0f, -0.001f, 8.0f, cx, cz);
    CHECK(cx == 1 && cz == -1);
    Grass3D::cellOf(-8.0f, -8.5f, 8.0f, cx, cz);
    CHECK(cx == -1 && cz == -2);

    CHECK(near(Grass3D::cellDistance(0, 0, 8.0f, 4.0f, 4.0f), 0.0f));
    CHECK(near(Grass3D::cellDistance(1, 0, 8.0f, 4.0f, 4.0f), 4.0f));
    CHECK(near(Grass3D::cellDistance(-1, 0, 8.0f, 4.0f, 4.0f), 4.0f));
    CHECK(near(Grass3D::cellDistance(2, 0, 8.0f, 4.0f, 4.0f), 12.0f));
    CHECK(near(Grass3D::cellDistance(1, 1, 8.0f, 4.0f, 4.0f), sqrtf(32.0f)));
    CHECK(near(Grass3D::cellDistance(3, 0, 8.0f, 4.0f, 4.0f), 20.0f));

    CHECK(Grass3D::candidateCount(0.5f, 8.0f) == 32);
    CHECK(Grass3D::candidateCount(4.0f, 8.0f) == 256);
    CHECK(Grass3D::candidateCount(0.015625f, 8.0f) == 1);
    CHECK(Grass3D::candidateCount(0.0f, 8.0f) == 0);
    CHECK(Grass3D::candidateCount(0.35f, 8.0f) == 22);

    CHECK(near(Grass3D::distanceKeep(0.0f, 60.0f, 0.35f), 1.0f));
    CHECK(near(Grass3D::distanceKeep(21.0f, 60.0f, 0.35f), 1.0f));
    CHECK(near(Grass3D::distanceKeep(40.5f, 60.0f, 0.35f), 0.5f));
    CHECK(near(Grass3D::distanceKeep(60.0f, 60.0f, 0.35f), 0.0f));
    CHECK(near(Grass3D::distanceKeep(80.0f, 60.0f, 0.35f), 0.0f));

    CHECK(near(Grass3D::drawFade(10.0f, 100.0f), 1.0f));
    CHECK(near(Grass3D::drawFade(70.0f, 100.0f), 1.0f));
    CHECK(near(Grass3D::drawFade(85.0f, 100.0f), 0.5f));
    CHECK(near(Grass3D::drawFade(100.0f, 100.0f), 0.0f));

    CHECK(near(Grass3D::brushFalloff(0.0f, 3.0f), 1.0f));
    CHECK(near(Grass3D::brushFalloff(1.5f, 3.0f), 0.5f));
    CHECK(near(Grass3D::brushFalloff(3.0f, 3.0f), 0.0f));

    std::uint32_t state = 1u;
    CHECK(near(Grass3D::nextRandom(state), 3967065.0f / 16777216.0f, 1e-7f));
    CHECK(state == 1015568748u);

    CHECK(Grass3D::typeSeed(12345u, 0) == 12345u);
    CHECK(Grass3D::typeSeed(12345u, 1) != 12345u);
    CHECK(Grass3D::cellSeed(12345u, 0, 0) != Grass3D::cellSeed(12345u, 1, 0));
    CHECK(Grass3D::cellSeed(12345u, 0, 0) != Grass3D::cellSeed(12345u, 0, 1));

    CHECK(near(Grass3D::windWave(Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, 1.0f), 0.0f));
    const float quarter = kPi / 3.2f;
    CHECK(near(Grass3D::windWave(Math::Vec3(0.0f, 0.0f, 0.0f), quarter, 1.0f),
            1.0f + 0.5f * sinf(2.7f * quarter), 1e-4f));
    CHECK(near(Grass3D::windWave(Math::Vec3(0.0f, 0.0f, 0.0f), quarter * 0.5f, 2.0f),
            Grass3D::windWave(Math::Vec3(0.0f, 0.0f, 0.0f), quarter, 1.0f), 1e-4f));

    const Math::Vec3 tint = Grass3D::tintOf(Math::Vec3(3.0f, 0.0f, 5.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(tint.x >= 0.0f && tint.x <= 1.0f && near(tint.x, tint.y) && near(tint.y, tint.z));
}

void testSimulation()
{
    const Math::Vec3 base(0.0f, 0.0f, 0.0f);
    const Math::Vec3 up(0.0f, 1.0f, 0.0f);
    const float length = 1.2f;
    GrassSimParams params;
    params.wind = 0.0f;
    params.deltaTime = kDt;

    GrassTip rest;
    Grass3D::simulate(base, up, length, rest, params, nullptr, 0);
    CHECK(rest.initialised);
    CHECK(near(rest.current.x, 0.0f, 1e-6f) && near(rest.current.y, 1.2f, 1e-6f) &&
            near(rest.current.z, 0.0f, 1e-6f));

    GrassInfluencer near1;
    near1.centre = Math::Vec3(0.0f, 1.2f, -1.0f);
    near1.radius = 3.0f;
    near1.force = 30.0f;
    GrassTip pushed;
    Grass3D::simulate(base, up, length, pushed, params, &near1, 1);
    CHECK(near(pushed.current.z, 0.32117f, 2e-4f));
    CHECK(near(pushed.current.y, 1.15622f, 2e-4f));
    CHECK(near(pushed.current.Length(), length, 1e-5f));

    GrassInfluencer edge = near1;
    edge.centre = Math::Vec3(0.0f, 1.2f, -2.5f);
    GrassTip lighter;
    Grass3D::simulate(base, up, length, lighter, params, &edge, 1);
    CHECK(near(lighter.current.z, 0.08313f, 2e-4f));
    CHECK(lighter.current.z > 0.0f && lighter.current.z < pushed.current.z);

    GrassInfluencer outside = near1;
    outside.centre = Math::Vec3(0.0f, 1.2f, -3.5f);
    GrassTip untouched;
    Grass3D::simulate(base, up, length, untouched, params, &outside, 1);
    CHECK(near(untouched.current.z, 0.0f, 1e-6f) && near(untouched.current.y, 1.2f, 1e-6f));

    GrassInfluencer down;
    down.centre = Math::Vec3(0.5f, 2.0f, 0.0f);
    down.radius = 5.0f;
    down.force = 600.0f;
    GrassTip floor;
    Grass3D::simulate(base, up, length, floor, params, &down, 1);
    CHECK(floor.current.y >= -1e-6f);
    CHECK(near(floor.current.Length(), length, 1e-5f));
    CHECK(near(floor.current.x, -1.2f, 1e-4f));

    GrassTip swing;
    for (int i = 0; i < 20; ++i) Grass3D::simulate(base, up, length, swing, params, &near1, 1);
    CHECK((swing.current - Math::Vec3(0.0f, 1.2f, 0.0f)).Length() > 0.5f);
    int steps = 0;
    for (; steps < 150; ++steps)
    {
        Grass3D::simulate(base, up, length, swing, params, nullptr, 0);
        CHECK(near(swing.current.Length(), length, 1e-4f));
        if ((swing.current - Math::Vec3(0.0f, 1.2f, 0.0f)).Length() < 1e-3f) break;
    }
    CHECK(steps < 150);
    printf("grass: tip back at rest after %d steps\n", steps + 1);

    for (int pass = 0; pass < 2; ++pass)
    {
        GrassSimParams windy;
        windy.wind = pass == 0 ? 1.0f : 2.0f;
        windy.deltaTime = kDt;
        windy.time = kPi / 3.2f;
        GrassTip tip;
        for (int i = 0; i < 1500; ++i) Grass3D::simulate(base, up, length, tip, windy, nullptr, 0);
        const float wave = 1.0f + 0.5f * sinf(2.7f * kPi / 3.2f);
        const float horizontal =
                sqrtf(tip.current.x * tip.current.x + tip.current.z * tip.current.z);
        CHECK(near(horizontal, wave * windy.wind / 12.0f, 0.004f * windy.wind));
        CHECK(near(tip.current.x / tip.current.z, 0.82f / 0.57f, 0.01f));
        CHECK(near(tip.current.Length(), length, 1e-4f));
    }
}

void testNodeClass()
{
    Grass3D grass;
    Forest3D forest;
    CHECK(grass.castTo<Grass3D>() == &grass);
    CHECK(grass.castTo<Node3D>() == &grass);
    CHECK(grass.castTo<Node>() == &grass);
    CHECK(grass.castTo<Forest3D>() == nullptr);
    CHECK(forest.castTo<Grass3D>() == nullptr);
    CHECK(forest.castTo<Forest3D>() == &forest);
    CHECK(forest.castTo<Node3D>() == &forest);
    CHECK(grass.isClass("Grass3D") && !grass.isClass("Forest3D"));
}

struct Reference
{
    Math::Vec3 root;
    float rotation;
    float scale;
    float keep;
};

void testPlacement(TextureStore& textures)
{
    const TextureId texture = makeTexture(textures, 255, 255, "grass placement");
    Grass3D grass;
    grass.setTexture(texture);
    grass.setSeed(12345u);
    grass.setCellSize(8.0f);
    grass.setDensity(0.5f);
    grass.setDrawDistance(1000.0f);
    grass.setWind(0.0f);
    grass.setExtent(0.0f, 0.0f, 8.0f, 8.0f);
    CHECK(grass.hasExtent());
    CHECK(grass.getTypeCount() == 1);
    CHECK(grass.getActiveCellCount() == 0);

    const Math::Vec3 camera(4.0f, 2.0f, 4.0f);
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.getActiveCellCount() == 1 && grass.hasCell(0, 0, 0));
    CHECK(grass.getCellClumpCount(0, 0, 0) == 32);
    CHECK(grass.getInstanceCount() == 32);
    GrassClumpInfo first;
    CHECK(grass.getClump(0, 0, 0, 0, first));
    CHECK(near(first.root.x, 4.42695f, 1e-4f) && near(first.root.z, 1.53768f, 1e-4f));
    CHECK(near(first.root.y, 0.0f, 1e-6f));
    CHECK(near(first.rotation, 4.30093f, 1e-4f));
    CHECK(near(first.scale, 1.16880f, 1e-4f));
    CHECK(near(first.keep, 0.317196f, 1e-5f));
    GrassClumpInfo second;
    CHECK(grass.getClump(0, 0, 0, 1, second));
    CHECK(near(second.root.x, 5.30825f, 1e-4f) && near(second.root.z, 2.07848f, 1e-4f));
    CHECK(!grass.getClump(0, 0, 0, 32, second) && !grass.getClump(0, 5, 5, 0, second));

    ct::Vector<Reference> before;
    for (int i = 0; i < 32; ++i)
    {
        GrassClumpInfo info;
        CHECK(grass.getClump(0, 0, 0, i, info));
        CHECK(info.live);
        before.push_back(Reference{ info.root, info.rotation, info.scale, info.keep });
    }

    grass.setExtent(0.0f, 0.0f, 8.0f, 8.0f);
    grass.setDensityFunction([](float, float) { return 0.5f; });
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.getCellClumpCount(0, 0, 0) == 16);
    int liveBelow = 0;
    for (int i = 0; i < 32; ++i)
    {
        GrassClumpInfo info;
        CHECK(grass.getClump(0, 0, 0, i, info));
        CHECK(info.live == (info.keep < 0.5f));
        if (info.live) ++liveBelow;
    }
    CHECK(liveBelow == 16);

    grass.setDensityFunction(Grass3D::FieldFunction());
    grass.setDrawDistance(20.0f);
    grass.setExtent(-8.0f, 0.0f, 8.0f, 8.0f);
    grass.setDensityFunction([](float x, float) { return x < 0.0f ? 0.0f : 1.0f; });
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.hasCell(0, -1, 0) && grass.hasCell(0, 0, 0));
    CHECK(grass.getCellClumpCount(0, -1, 0) == 0);
    int inside = 0;
    for (int i = 0; i < 32; ++i)
    {
        GrassClumpInfo info;
        CHECK(grass.getClump(0, 0, 0, i, info));
        const float dx = info.root.x - camera.x;
        const float dz = info.root.z - camera.z;
        const float d = sqrtf(dx * dx + dz * dz + 4.0f);
        if (info.keep < Grass3D::distanceKeep(d, 20.0f, 0.35f)) ++inside;
    }
    CHECK(grass.getCellClumpCount(0, 0, 0) == inside);
    CHECK(inside > 0);

    grass.setDensityFunction(Grass3D::FieldFunction());
    CHECK(grass.update(camera, nullptr, kDt));
    ct::Vector<Reference> snapshot;
    ct::Vector<int> liveSet;
    for (int i = 0; i < 32; ++i)
    {
        GrassClumpInfo info;
        CHECK(grass.getClump(0, 0, 0, i, info));
        snapshot.push_back(Reference{ info.root, info.rotation, info.scale, info.keep });
        liveSet.push_back(info.live ? 1 : 0);
        CHECK(near(info.root.x, before[static_cast<std::size_t>(i)].root.x, 1e-6f));
    }
    CHECK(grass.update(Math::Vec3(500.0f, 2.0f, 4.0f), nullptr, kDt));
    CHECK(!grass.hasCell(0, 0, 0) && grass.getActiveCellCount() == 0);
    CHECK(grass.getPooledCellCount() > 0);
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.hasCell(0, 0, 0));
    for (int i = 0; i < 32; ++i)
    {
        GrassClumpInfo info;
        CHECK(grass.getClump(0, 0, 0, i, info));
        const Reference& was = snapshot[static_cast<std::size_t>(i)];
        CHECK(info.root == was.root && info.rotation == was.rotation && info.scale == was.scale &&
                info.keep == was.keep);
        CHECK((info.live ? 1 : 0) == liveSet[static_cast<std::size_t>(i)]);
    }

    grass.setSeed(777u);
    CHECK(grass.update(camera, nullptr, kDt));
    GrassClumpInfo reseeded;
    CHECK(grass.getClump(0, 0, 0, 0, reseeded));
    CHECK(fabsf(reseeded.root.x - first.root.x) > 1e-3f);
    grass.setSeed(12345u);
    grass.setDensity(1.0f);
    CHECK(grass.update(camera, nullptr, kDt));
    GrassClumpInfo again;
    CHECK(grass.getClump(0, 0, 0, 63, again));
    CHECK(!grass.getClump(0, 0, 0, 64, again));
    CHECK(grass.getClump(0, 0, 0, 0, again));
    CHECK(near(again.root.x, first.root.x, 1e-6f));
    textures.destroy(texture);
}

void testCells(TextureStore& textures)
{
    const TextureId texture = makeTexture(textures, 255, 255, "grass cells");
    Grass3D grass;
    grass.setTexture(texture);
    grass.setCellSize(8.0f);
    grass.setDensity(0.5f);
    grass.setDrawDistance(20.0f);
    grass.setWind(0.0f);

    const Math::Vec3 camera(4.0f, 2.0f, 4.0f);
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.getActiveCellCount() == 29);
    CHECK(grass.hasCell(0, 0, 0) && grass.hasCell(0, 3, 0) && grass.hasCell(0, -3, 0));
    CHECK(!grass.hasCell(0, 3, 1) && !grass.hasCell(0, 4, 0) && !grass.hasCell(0, 3, 2));
    CHECK(grass.hasCell(0, 2, 2) && grass.hasCell(0, -2, -2));
    CHECK(grass.getInstanceCount() > 0);
    CHECK(grass.getDrawnInstanceCount() == grass.getInstanceCount());

    CHECK(!grass.update(camera, nullptr, kDt));
    CHECK(grass.getWrittenInstanceCount() == 0 && grass.getFilledCellCount() == 0);

    CHECK(grass.update(Math::Vec3(12.0f, 2.0f, 4.0f), nullptr, kDt));
    CHECK(grass.hasCell(0, -3, 0));
    CHECK(grass.hasCell(0, 4, 0));
    CHECK(grass.update(Math::Vec3(13.0f, 2.0f, 4.0f), nullptr, kDt));
    CHECK(!grass.hasCell(0, -3, 0));

    grass.update(camera, nullptr, kDt);
    CHECK(grass.getActiveCellCount() >= 29);
    const int childrenBefore = static_cast<int>(grass.getChildCount());
    CHECK(grass.update(Math::Vec3(84.0f, 2.0f, 4.0f), nullptr, kDt));
    CHECK(grass.getActiveCellCount() == 29);
    CHECK(!grass.hasCell(0, 0, 0));
    CHECK(static_cast<int>(grass.getChildCount()) == childrenBefore);
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.hasCell(0, 0, 0));
    CHECK(static_cast<int>(grass.getChildCount()) == childrenBefore);
    grass.setDrawDistance(5.0f);
    CHECK(grass.update(Math::Vec3(500.0f, 2.0f, 500.0f), nullptr, kDt));
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.getActiveCellCount() > 0 && grass.getActiveCellCount() < 29);
    CHECK(grass.getPooledCellCount() > 0);
    CHECK(grass.getActiveCellCount() + grass.getPooledCellCount() == childrenBefore);
    grass.setDrawDistance(20.0f);

    grass.setDensityFalloffStart(1.0f);
    CHECK(grass.getDensityFalloffStart() == 1.0f);
    grass.update(camera, nullptr, kDt);
    const int fullRange = grass.getInstanceCount();
    grass.setDensityFalloffStart(0.0f);
    grass.update(camera, nullptr, kDt);
    const int thinned = grass.getInstanceCount();
    CHECK(thinned < fullRange);
    grass.setDensityFalloffStart(0.35f);
    grass.update(camera, nullptr, kDt);
    const int middle = grass.getInstanceCount();
    CHECK(thinned < middle && middle < fullRange);
    for (int cx = -3; cx <= 3; ++cx)
    {
        for (int cz = -3; cz <= 3; ++cz)
        {
            if (!grass.hasCell(0, cx, cz)) continue;
            for (int k = 0; k < 22; ++k)
            {
                GrassClumpInfo info;
                if (!grass.getClump(0, cx, cz, k, info)) break;
                const Math::Vec3 offset = info.root - camera;
                const bool expect =
                        info.inside &&
                        info.keep < Grass3D::distanceKeep(offset.Length(), 20.0f, 0.35f);
                CHECK(info.live == expect);
                if (info.live) CHECK(offset.Length() < 20.0f);
            }
        }
    }

    textures.destroy(texture);
}

void testCulling(TextureStore& textures, Scene& scene)
{
    const TextureId texture = makeTexture(textures, 255, 255, "grass culling");
    Grass3D grass;
    grass.setTexture(texture);
    grass.setCellSize(8.0f);
    grass.setDensity(0.5f);
    grass.setDrawDistance(30.0f);
    grass.setWind(0.0f);
    scene.camera->setPerspective(60.0f, 0.1f, 200.0f);

    place(scene, Math::Vec3(0.0f, 1.5f, 0.0f), Math::Vec3(0.0f, 1.5f, -10.0f));
    const Math::Frustum ahead =
            Math::Frustum::FromViewProjection(scene.camera->getCameraProjection() *
                                              scene.camera->getGlobalTransform().InverseAffine());
    const Math::Vec3 eye(0.0f, 1.5f, 0.0f);
    grass.update(eye, &ahead, kDt);
    const int active = grass.getActiveCellCount();
    CHECK(active > 20);
    CHECK(grass.getVisibleCellCount() > 0 && grass.getVisibleCellCount() < active);
    CHECK(grass.getDrawnInstanceCount() > 0 &&
            grass.getDrawnInstanceCount() < grass.getInstanceCount());
    CHECK(grass.getCellNode(0, 0, -2)->getVisibleInstanceCount() > 0);
    CHECK(grass.getCellNode(0, 0, 1)->getVisibleInstanceCount() == 0);
    CHECK(grass.getCellNode(0, 0, 0)->getVisibleInstanceCount() ==
                    grass.getCellClumpCount(0, 0, 0) ||
            grass.getCellNode(0, 0, 0)->getVisibleInstanceCount() == 0);
    const int drawnAhead = grass.getDrawnInstanceCount();

    place(scene, Math::Vec3(0.0f, 1.5f, 0.0f), Math::Vec3(0.0f, 1.5f, 10.0f));
    const Math::Frustum behind =
            Math::Frustum::FromViewProjection(scene.camera->getCameraProjection() *
                                              scene.camera->getGlobalTransform().InverseAffine());
    grass.update(eye, &behind, kDt);
    CHECK(grass.getActiveCellCount() == active);
    CHECK(grass.getCellNode(0, 0, -2)->getVisibleInstanceCount() == 0);
    CHECK(grass.getCellNode(0, 0, 1)->getVisibleInstanceCount() > 0);
    CHECK(grass.getDrawnInstanceCount() > 0);
    CHECK(abs(grass.getDrawnInstanceCount() - drawnAhead) < drawnAhead);

    grass.setFrustumCullingEnabled(false);
    grass.update(eye, &behind, kDt);
    CHECK(grass.getDrawnInstanceCount() == grass.getInstanceCount());
    CHECK(!grass.isFrustumCullingEnabled());
    textures.destroy(texture);
}

void testInfluencers(TextureStore& textures)
{
    const TextureId texture = makeTexture(textures, 255, 255, "grass influence");
    Grass3D grass;
    grass.setTexture(texture);
    grass.setCellSize(8.0f);
    grass.setDensity(0.5f);
    grass.setDrawDistance(1000.0f);
    grass.setWind(0.0f);
    grass.setExtent(0.0f, 0.0f, 8.0f, 8.0f);
    const Math::Vec3 camera(4.0f, 2.0f, 4.0f);
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getAwakeInstanceCount() == 0 && grass.getSimulatedInstanceCount() == 0);

    CHECK(grass.getInfluencerCount() == 0);
    for (int i = 0; i < 8; ++i)
        CHECK(grass.addInfluencer(Math::Vec3(4.0f, 0.5f, 4.0f), 0.01f, 1.0f));
    CHECK(grass.getInfluencerCount() == 8);
    CHECK(!grass.addInfluencer(Math::Vec3(0.0f, 0.0f, 0.0f), 1.0f, 1.0f));
    CHECK(grass.getInfluencerCount() == 8);
    CHECK(!grass.addInfluencer(Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, 1.0f));
    grass.clearInfluencers();
    CHECK(grass.getInfluencerCount() == 0);

    GrassClumpInfo target;
    int index = 0;
    for (; index < 32; ++index)
    {
        CHECK(grass.getClump(0, 0, 0, index, target));
        if (target.live) break;
    }
    CHECK(!target.awake);
    const Math::Vec3 centre = target.root + Math::Vec3(-1.0f, 0.5f, 0.0f);
    CHECK(grass.addInfluencer(centre, 3.0f, 30.0f));
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.getInfluencerCount() == 0);
    CHECK(grass.getAwakeInstanceCount() > 0);
    CHECK(grass.getSimulatedInstanceCount() >= grass.getAwakeInstanceCount());
    GrassClumpInfo moved;
    CHECK(grass.getClump(0, 0, 0, index, moved));
    CHECK(moved.awake);
    const Math::Vec3 relative = moved.tip - moved.root;
    CHECK(relative.x > 0.0f);
    const float stem = 1.2f * moved.scale;
    CHECK(near(relative.Length(), stem, 1e-4f));
    CHECK(moved.tip.y >= moved.root.y);
    const float firstLean = relative.x;

    int farAwake = 0;
    for (int k = 0; k < 32; ++k)
    {
        GrassClumpInfo other;
        CHECK(grass.getClump(0, 0, 0, k, other));
        const float reach = (other.root - centre).Length();
        if (other.awake) CHECK(reach < 3.0f + 1.2f * other.scale);
        if (reach >= 3.0f + 1.2f * other.scale) CHECK(!other.awake);
        if (reach >= 3.0f + 1.5f) ++farAwake;
    }
    CHECK(farAwake > 0);

    grass.update(camera, nullptr, kDt);
    CHECK(grass.getClump(0, 0, 0, index, moved));
    CHECK(moved.awake && (moved.tip - moved.root).x != firstLean);

    int steps = 0;
    while (grass.getAwakeInstanceCount() > 0 && steps < 250)
    {
        grass.update(camera, nullptr, kDt);
        ++steps;
    }
    printf("grass: all awake clumps asleep again after %d steps\n", steps);
    CHECK(steps < 250);
    CHECK(grass.getClump(0, 0, 0, index, moved));
    CHECK(!moved.awake);
    CHECK((moved.tip - moved.root - Math::Vec3(0.0f, stem, 0.0f)).Length() < 1e-4f);
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getWrittenInstanceCount() == 0 && grass.getSimulatedInstanceCount() == 0);

    grass.setWind(1.0f);
    grass.update(camera, nullptr, kDt);
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getAwakeInstanceCount() == 0 && grass.getWrittenInstanceCount() == 0);
    textures.destroy(texture);
}

void testMask(TextureStore& textures)
{
    const TextureId texture = makeTexture(textures, 255, 255, "grass mask");
    Grass3D grass;
    grass.setTexture(texture);
    grass.setSeed(12345u);
    grass.setCellSize(8.0f);
    grass.setDensity(4.0f);
    grass.setDrawDistance(1000.0f);
    grass.setWind(0.0f);
    CHECK(!grass.createMask());
    CHECK(!grass.paint(Math::Vec3(0.0f, 0.0f, 0.0f), 3.0f, 1.0f, 0));
    grass.setExtent(-16.0f, -16.0f, 16.0f, 16.0f);
    const Math::Vec3 camera(0.0f, 2.0f, 0.0f);

    grass.update(camera, nullptr, kDt);
    CHECK(grass.getActiveCellCount() == 16);
    CHECK(grass.getInstanceCount() == 16 * 256);
    CHECK(!grass.hasMask());

    grass.setMaskResolution(33, 33);
    CHECK(grass.createMask());
    CHECK(grass.hasMask() && grass.getMaskWidth() == 33 && grass.getMaskDepth() == 33);
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getInstanceCount() == 0);
    CHECK(grass.getFilledCellCount() == 16);

    CHECK(!grass.paint(Math::Vec3(4.0f, 0.0f, 4.0f), 0.0f, 1.0f, 0));
    CHECK(!grass.paint(Math::Vec3(4.0f, 0.0f, 4.0f), 3.0f, 0.0f, 0));
    CHECK(!grass.paint(Math::Vec3(4.0f, 0.0f, 4.0f), 3.0f, 1.0f, 1));
    CHECK(!grass.paint(Math::Vec3(400.0f, 0.0f, 400.0f), 3.0f, 1.0f, 0));

    CHECK(grass.paint(Math::Vec3(4.0f, 0.0f, 4.0f), 3.0f, 1.0f, 0));
    const MultiMeshInstance3D* untouchedNode = grass.getCellNode(0, -1, -1);
    GrassClumpInfo untouched;
    CHECK(grass.getClump(0, -1, -1, 0, untouched));
    CHECK(grass.update(camera, nullptr, kDt));
    CHECK(grass.getFilledCellCount() == 1);
    CHECK(grass.getCellNode(0, -1, -1) == untouchedNode);
    GrassClumpInfo untouchedAfter;
    CHECK(grass.getClump(0, -1, -1, 0, untouchedAfter));
    CHECK(untouchedAfter.root == untouched.root);
    CHECK(grass.getInstanceCount() == 23);
    CHECK(grass.getCellClumpCount(0, 0, 0) == 23);
    int inner = 0;
    int annulus = 0;
    int beyond = 0;
    const float equalArea = 3.0f / sqrtf(2.0f);
    for (int k = 0; k < 256; ++k)
    {
        GrassClumpInfo info;
        CHECK(grass.getClump(0, 0, 0, k, info));
        if (!info.live) continue;
        const float d = sqrtf((info.root.x - 4.0f) * (info.root.x - 4.0f) +
                              (info.root.z - 4.0f) * (info.root.z - 4.0f));
        if (d < equalArea) ++inner;
        else if (d < 3.0f)
            ++annulus;
        else
            ++beyond;
    }
    CHECK(inner == 19);
    CHECK(annulus == 4);
    CHECK(beyond == 0);

    CHECK(grass.erase(Math::Vec3(4.0f, 0.0f, 4.0f), 3.0f, 1.0f, 0));
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getInstanceCount() == 0);
    CHECK(grass.getFilledCellCount() == 1);
    CHECK(!grass.erase(Math::Vec3(4.0f, 0.0f, 4.0f), 3.0f, 1.0f, 0));

    grass.update(camera, nullptr, kDt);
    CHECK(grass.getFilledCellCount() == 0);
    CHECK(grass.paint(Math::Vec3(0.0f, 0.0f, 0.0f), 4.0f, 1.0f, 0));
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getFilledCellCount() == 4);
    CHECK(grass.erase(Math::Vec3(0.0f, 0.0f, 0.0f), 4.0f, 1.0f, 0));
    grass.update(camera, nullptr, kDt);

    GrassTypeDesc second;
    second.texture = makeTexture(textures, 255, 255, "grass second");
    second.colourA = Math::Vec3(0.0f, 0.0f, 1.0f);
    second.colourB = Math::Vec3(0.0f, 0.0f, 1.0f);
    CHECK(grass.addType(second) == 1);
    CHECK(grass.getTypeCount() == 2);
    CHECK(grass.paint(Math::Vec3(-8.0f, 0.0f, -8.0f), 3.0f, 1.0f, 0));
    CHECK(grass.paint(Math::Vec3(8.0f, 0.0f, 8.0f), 3.0f, 1.0f, 1));
    CHECK(grass.paint(Math::Vec3(0.0f, 0.0f, 0.0f), 4.0f, 1.0f, 0));
    CHECK(grass.paint(Math::Vec3(0.0f, 0.0f, 0.0f), 4.0f, 1.0f, 1));
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getInstanceCount(0) == 88);
    CHECK(grass.getInstanceCount(1) == 88);
    CHECK(grass.getInstanceCount() == 176);
    CHECK(grass.getCellNode(0, 0, 0) != nullptr && grass.getCellNode(1, 0, 0) != nullptr);
    CHECK(grass.getCellNode(0, 0, 0) != grass.getCellNode(1, 0, 0));
    CHECK(grass.getCellNode(0, 0, 0)->getVisibleInstanceCount() ==
            grass.getCellClumpCount(0, 0, 0));
    CHECK(grass.getCellNode(1, 0, 0)->getVisibleInstanceCount() ==
            grass.getCellClumpCount(1, 0, 0));
    CHECK(grass.getCellNode(0, 0, 0)->getSurfaceMaterial(0) == grass.getMaterial(0));
    CHECK(grass.getCellNode(1, 0, 0)->getSurfaceMaterial(0) == grass.getMaterial(1));
    CHECK(grass.getMaterial(0) != grass.getMaterial(1));
    CHECK(grass.getCellClumpCount(1, 1, 1) > 0 && grass.getCellClumpCount(0, 1, 1) == 0);
    CHECK(grass.getCellClumpCount(0, -2, -2) > 0 && grass.getCellClumpCount(1, -2, -2) == 0);
    GrassClumpInfo a;
    GrassClumpInfo b;
    CHECK(grass.getClump(0, 0, 0, 0, a) && grass.getClump(1, 0, 0, 0, b));
    CHECK(!(a.root == b.root));

    CHECK(grass.saveMask(PRISMA3D_TEST_DIR "/grass_mask.png"));
    ct::Vector<Math::Vec3> saved;
    for (int cx = -2; cx < 2; ++cx)
        for (int cz = -2; cz < 2; ++cz)
            for (int type = 0; type < 2; ++type)
                for (int k = 0; k < 256; ++k)
                {
                    GrassClumpInfo info;
                    if (grass.getClump(type, cx, cz, k, info) && info.live)
                        saved.push_back(info.root);
                }
    CHECK(saved.size() == 176);

    {
        Grass3D loaded;
        loaded.setTexture(texture);
        CHECK(loaded.addType(second) == 1);
        loaded.setSeed(12345u);
        loaded.setCellSize(8.0f);
        loaded.setDensity(4.0f);
        loaded.setDrawDistance(1000.0f);
        loaded.setWind(0.0f);
        CHECK(!loaded.loadMask(PRISMA3D_TEST_DIR "/grass_mask.png"));
        loaded.setExtent(-16.0f, -16.0f, 16.0f, 16.0f);
        CHECK(!loaded.loadMask(PRISMA3D_TEST_DIR "/missing_mask.png"));
        CHECK(loaded.loadMask(PRISMA3D_TEST_DIR "/grass_mask.png"));
        CHECK(loaded.getMaskWidth() == 33 && loaded.getMaskDepth() == 33);
        loaded.update(camera, nullptr, kDt);
        CHECK(loaded.getInstanceCount(0) == 88 && loaded.getInstanceCount(1) == 88);
        std::size_t next = 0;
        for (int cx = -2; cx < 2; ++cx)
            for (int cz = -2; cz < 2; ++cz)
                for (int type = 0; type < 2; ++type)
                    for (int k = 0; k < 256; ++k)
                    {
                        GrassClumpInfo info;
                        if (loaded.getClump(type, cx, cz, k, info) && info.live)
                        {
                            CHECK(next < saved.size() && info.root == saved[next]);
                            ++next;
                        }
                    }
        CHECK(next == saved.size());
    }

    CHECK(grass.erase(Math::Vec3(0.0f, 0.0f, 0.0f), 4.0f, 1.0f));
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getInstanceCount(0) == 33);
    CHECK(grass.getInstanceCount(1) == 31);

    grass.fill(1, 1.0f);
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getInstanceCount(1) == 4096);
    CHECK(grass.getInstanceCount(0) == 33);
    grass.fill(1, 0.0f);
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getInstanceCount(1) == 0);
    grass.clearMask();
    CHECK(!grass.hasMask());
    grass.update(camera, nullptr, kDt);
    CHECK(grass.getInstanceCount(0) == 4096 && grass.getInstanceCount(1) == 4096);

    CHECK(near(grass.getMaskDensity(0, 0.0f, 0.0f), 1.0f));
    CHECK(grass.createMask(9, 9));
    CHECK(near(grass.getMaskDensity(0, 0.0f, 0.0f), 0.0f));
    grass.fill(0, 0.5f);
    CHECK(near(grass.getMaskDensity(0, 3.3f, -7.1f), 128.0f / 255.0f, 1e-4f));
    CHECK(near(grass.getMaskDensity(0, 20.0f, 0.0f), 0.0f));
    CHECK(near(grass.getMaskDensity(1, 3.3f, -7.1f), 0.0f));

    grass.removeType(0);
    CHECK(grass.getTypeCount() == 1);
    CHECK(near(grass.getMaskDensity(0, 3.3f, -7.1f), 0.0f));
    textures.destroy(second.texture);
    textures.destroy(texture);
}

void testRender(TextureStore& textures, Scene& scene, Node* root)
{
    const TextureId solid = makeTexture(textures, 255, 255, "grass solid");
    const TextureId halfTop = makeTexture(textures, 0, 255, "grass half");
    scene.camera->setPerspective(60.0f, 0.1f, 200.0f);
    place(scene, Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec3(0.0f, 1.0f, -10.0f));

    {
        Grass3D* grass = root->createChild<Grass3D>();
        grass->setTexture(solid);
        grass->setCellSize(4.0f);
        grass->setDensity(2.0f);
        grass->setDrawDistance(40.0f);
        grass->setReleaseMargin(0.0f);
        grass->setWind(0.0f);
        grass->setExtent(-3.0f, -20.0f, 3.0f, -10.0f);
        {
            Frame frame(*scene.tree);
        }
        Image image;
        CHECK(grab(scene, image));
        const Bounds bounds = grassBounds(image);
        printf("grass: plane in view covers %d pixels, x %d..%d, y %d..%d\n", bounds.count,
                bounds.minX, bounds.maxX, bounds.minY, bounds.maxY);
        CHECK(bounds.count > 300);
        CHECK(image.isClear(0, 0) && image.isClear(kWidth - 1, 0) && image.isClear(0, kHeight - 1));
        CHECK(bounds.minX > 20 && bounds.maxX < kWidth - 20);
        CHECK(grass->getActiveCellCount() > 0);
        CHECK(grass->getDrawnInstanceCount() > 0);

        grass->setDrawDistance(5.0f);
        {
            Frame frame(*scene.tree);
        }
        CHECK(grass->getActiveCellCount() == 0);
        CHECK(grass->getDrawnInstanceCount() == 0);
        CHECK(grab(scene, image));
        CHECK(grassBounds(image).count == 0);

        grass->setDrawDistance(14.0f);
        {
            Frame frame(*scene.tree);
        }
        CHECK(grab(scene, image));
        const Bounds near14 = grassBounds(image);
        CHECK(near14.count > 0);
        grass->setDrawDistance(40.0f);
        {
            Frame frame(*scene.tree);
        }
        CHECK(grab(scene, image));
        CHECK(grassBounds(image).count > near14.count);
        grass->queueFree();
        Frame frame(*scene.tree);
    }

    {
        Grass3D* grass = root->createChild<Grass3D>();
        grass->setTexture(solid);
        grass->setCellSize(8.0f);
        grass->setDensity(0.015625f);
        grass->setDrawDistance(100.0f);
        grass->setWind(0.0f);
        grass->setExtent(0.0f, 0.0f, 8.0f, 8.0f);
        grass->setHeight(1.2f);
        grass->setDensityFalloffStart(1.0f);
        {
            Frame frame(*scene.tree);
        }
        GrassClumpInfo tuft;
        CHECK(grass->getClump(0, 0, 0, 0, tuft) && tuft.live);
        const Math::Vec3 target = tuft.root + Math::Vec3(0.0f, 0.6f, 0.0f);
        place(scene, tuft.root + Math::Vec3(0.0f, 0.7f, 2.5f), target);
        for (int i = 0; i < 3; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image rest;
        CHECK(grab(scene, rest));
        const Bounds restBounds = grassBounds(rest);
        printf("grass: single tuft at rest covers %d pixels, x %d..%d, y %d..%d\n",
                restBounds.count, restBounds.minX, restBounds.maxX, restBounds.minY,
                restBounds.maxY);
        CHECK(restBounds.count > 100);
        CHECK(restBounds.minY > 2 && restBounds.maxY < kHeight - 2);

        grass->setWind(10.0f);
        grass->setWindDirection(Math::Vec3(1.0f, 0.0f, 0.0f));
        for (int i = 0; i < 90; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image windy;
        CHECK(grab(scene, windy));
        CHECK(grass->getAwakeInstanceCount() == 0 && grass->getWrittenInstanceCount() == 0);
        float tipRest = 0.0f;
        float tipWindy = 0.0f;
        int tipRestCount = 0;
        int tipWindyCount = 0;
        const Bounds restRows = grassRows(rest, 5);
        int restLow = kWidth;
        int restHigh = -1;
        int windyLow = kWidth;
        int windyHigh = -1;
        for (int x = 0; x < kWidth; ++x)
        {
            if (!rest.isClear(x, restRows.maxY))
            {
                if (x < restLow) restLow = x;
                if (x > restHigh) restHigh = x;
            }
            if (!windy.isClear(x, restRows.maxY))
            {
                if (x < windyLow) windyLow = x;
                if (x > windyHigh) windyHigh = x;
            }
        }
        for (int pass = 0; pass < 2; ++pass)
        {
            const Image& image = pass == 0 ? rest : windy;
            const Bounds own = grassRows(image, 8);
            for (int y = own.minY; y <= own.minY + 3; ++y)
            {
                for (int x = 0; x < kWidth; ++x)
                {
                    if (image.isClear(x, y)) continue;
                    if (pass == 0)
                    {
                        tipRest += static_cast<float>(x);
                        ++tipRestCount;
                    }
                    else
                    {
                        tipWindy += static_cast<float>(x);
                        ++tipWindyCount;
                    }
                }
            }
        }
        CHECK(tipRestCount > 0 && tipWindyCount > 0 && restHigh >= 0 && windyHigh >= 0);
        const float tipShift = tipWindy / static_cast<float>(tipWindyCount) -
                               tipRest / static_cast<float>(tipRestCount);
        printf("grass: wind with the clock running, base row spans %d..%d at rest and %d..%d in "
               "wind, top rows moved %.2f pixels\n",
                restLow, restHigh, windyLow, windyHigh, tipShift);
        CHECK(abs(restLow - windyLow) <= 2 && abs(restHigh - windyHigh) <= 2);
        CHECK(restHigh - restLow > 15);
        CHECK(fabsf(tipShift) > 3.0f);

        grass->setWind(0.0f);
        for (int i = 0; i < 400; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image settled;
        CHECK(grab(scene, settled));
        int different = 0;
        for (int y = 0; y < kHeight; ++y)
            for (int x = 0; x < kWidth; ++x)
                if (memcmp(rest.at(x, y), settled.at(x, y), 3) != 0) ++different;
        CHECK(different == 0);
        {
            Frame frame(*scene.tree);
            CHECK(grass->getWrittenInstanceCount() == 0);
            CHECK(frame.stats().multimeshUploadBytes == 0);
        }
        {
            Frame frame(*scene.tree);
            CHECK(frame.stats().multimeshUploadBytes == 0);
            CHECK(frame.stats().multimeshInstancesDrawn == 1);
            CHECK(frame.stats().drawCalls == 1);
            CHECK(frame.stats().triangles == 6);
        }

        grass->setWindSpeed(0.0f);
        grass->setWind(6.0f);
        grass->setWindDirection(Math::Vec3(1.0f, 0.0f, 0.0f));
        for (int i = 0; i < 2; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image frozen;
        CHECK(grab(scene, frozen));
        const float wave = Grass3D::windWave(tuft.root, 0.0f, 1.0f);
        const float lean = wave * 6.0f / 12.0f;
        const float stem = 1.2f * tuft.scale;
        const float tipX = stem * lean / sqrtf(stem * stem + lean * lean);
        float restCentroid = 0.0f;
        float frozenCentroid = 0.0f;
        int restCount = 0;
        int frozenCount = 0;
        for (int y = restBounds.minY + 3; y <= restBounds.minY + 6; ++y)
        {
            for (int x = 0; x < kWidth; ++x)
            {
                if (!rest.isClear(x, y))
                {
                    restCentroid += static_cast<float>(x);
                    ++restCount;
                }
                if (!frozen.isClear(x, y))
                {
                    frozenCentroid += static_cast<float>(x);
                    ++frozenCount;
                }
            }
        }
        CHECK(restCount > 0 && frozenCount > 0);
        const float shift = frozenCentroid / static_cast<float>(frozenCount) -
                            restCentroid / static_cast<float>(restCount);
        const float pixelsPerMetre = static_cast<float>(kHeight) * 0.5f / tanf(kPi / 6.0f) / 2.5f;
        const float expected = tipX * pixelsPerMetre * 0.9f;
        printf("grass: wind wave %.3f, tip offset %.3f m, top rows shifted %.2f pixels, expected "
               "about %.2f\n",
                wave, tipX, shift, expected);
        CHECK(fabsf(tipX) > 0.1f);
        CHECK(shift * expected > 0.0f);
        CHECK(fabsf(shift - expected) < 0.3f * fabsf(expected) + 1.0f);
        grass->setWind(0.0f);
        grass->setWindSpeed(1.0f);
        for (int i = 0; i < 2; ++i)
        {
            Frame frame(*scene.tree);
        }

        grass->addInfluencer(tuft.root + Math::Vec3(-0.8f, 0.6f, 0.0f), 3.0f, 30.0f);
        {
            Frame frame(*scene.tree);
            CHECK(frame.stats().multimeshUploadBytes > 0);
        }
        for (int i = 0; i < 2; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image pushed;
        CHECK(grab(scene, pushed));
        int changed = 0;
        for (int y = 0; y < kHeight; ++y)
            for (int x = 0; x < kWidth; ++x)
                if (memcmp(rest.at(x, y), pushed.at(x, y), 3) != 0) ++changed;
        CHECK(changed > 15);
        grass->queueFree();
        Frame frame(*scene.tree);
    }

    {
        Grass3D* full = root->createChild<Grass3D>();
        full->setTexture(solid);
        full->setCellSize(8.0f);
        full->setDensity(0.015625f);
        full->setDrawDistance(100.0f);
        full->setWind(0.0f);
        full->setExtent(0.0f, 0.0f, 8.0f, 8.0f);
        full->setDensityFalloffStart(1.0f);
        {
            Frame frame(*scene.tree);
        }
        GrassClumpInfo tuft;
        CHECK(full->getClump(0, 0, 0, 0, tuft));
        place(scene, tuft.root + Math::Vec3(0.0f, 0.7f, 2.5f),
                tuft.root + Math::Vec3(0.0f, 0.6f, 0.0f));
        for (int i = 0; i < 3; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image whole;
        CHECK(grab(scene, whole));
        const Bounds wholeBounds = grassBounds(whole);
        full->queueFree();
        {
            Frame frame(*scene.tree);
        }

        Grass3D* half = root->createChild<Grass3D>();
        half->setTexture(halfTop);
        half->setCellSize(8.0f);
        half->setDensity(0.015625f);
        half->setDrawDistance(100.0f);
        half->setWind(0.0f);
        half->setExtent(0.0f, 0.0f, 8.0f, 8.0f);
        half->setDensityFalloffStart(1.0f);
        for (int i = 0; i < 4; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image upper;
        CHECK(grab(scene, upper));
        const Bounds upperBounds = grassRows(upper, 10);
        const Bounds wholeRows = grassRows(whole, 10);
        const int wholeHeight = wholeRows.maxY - wholeRows.minY + 1;
        const int upperHeight = upperBounds.maxY - upperBounds.minY + 1;
        printf("grass: opaque texture %d rows, upper half transparent %d rows\n", wholeHeight,
                upperHeight);
        CHECK(upperHeight * 100 > wholeHeight * 40 && upperHeight * 100 < wholeHeight * 60);
        CHECK(upperBounds.maxY >= wholeRows.maxY - 1 && upperBounds.maxY <= wholeRows.maxY + 1);

        CHECK(half->addRegion(0, 8, 16, 8, 1.0f, 1.0f));
        CHECK(!half->addRegion(0, 12, 16, 8, 1.0f, 1.0f));
        CHECK(!half->addRegion(0, 0, 0, 8, 1.0f, 1.0f));
        CHECK(half->getRegionCount() == 1);
        GrassRegion region;
        CHECK(half->getRegion(0, region));
        CHECK(near(region.aspect, 2.0f) && near(region.texMulAdd.x, 1.0f) &&
                near(region.texMulAdd.y, 0.5f) && near(region.texMulAdd.w, 0.5f));
        for (int i = 0; i < 4; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image regioned;
        CHECK(grab(scene, regioned));
        const Bounds regionBounds = grassRows(regioned, 10);
        const int regionHeight = regionBounds.maxY - regionBounds.minY + 1;
        printf("grass: bottom-half region %d rows (the whole quad is opaque)\n", regionHeight);
        CHECK(regionBounds.count > wholeBounds.count * 3 / 2);
        CHECK(regionHeight >= wholeHeight - 2);
        half->queueFree();
        Frame frame(*scene.tree);
    }

    textures.destroy(halfTop);
    textures.destroy(solid);
}

void testLighting(TextureStore& textures, Scene& scene, Node* root, WorldEnvironment* environment)
{
    const TextureId solid = makeTexture(textures, 255, 255, "grass lit");
    EnvironmentSettings dark = whiteAmbient();
    dark.ambientColor = Math::Vec3(0.02f, 0.02f, 0.02f);
    environment->setEnvironment(dark);

    Grass3D* grass = root->createChild<Grass3D>();
    grass->setTexture(solid);
    grass->setCellSize(8.0f);
    grass->setDensity(0.015625f);
    grass->setDrawDistance(100.0f);
    grass->setWind(0.0f);
    grass->setExtent(0.0f, 0.0f, 8.0f, 8.0f);
    grass->setDensityFalloffStart(1.0f);
    {
        Frame frame(*scene.tree);
    }
    GrassClumpInfo tuft;
    CHECK(grass->getClump(0, 0, 0, 0, tuft));

    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    sun->setShadow(false);
    sun->setRotation(Math::Vec3(-kPi * 0.5f, 0.0f, 0.0f));

    float mean[3] = { 0.0f, 0.0f, 0.0f };
    for (int pass = 0; pass < 3; ++pass)
    {
        const float side = pass == 1 ? -1.0f : 1.0f;
        sun->setEnergy(pass == 2 ? 0.0f : 1.0f);
        place(scene, tuft.root + Math::Vec3(0.0f, 0.7f, 2.5f * side),
                tuft.root + Math::Vec3(0.0f, 0.6f, 0.0f));
        for (int i = 0; i < 3; ++i)
        {
            Frame frame(*scene.tree);
        }
        Image image;
        CHECK(grab(scene, image));
        float sum = 0.0f;
        int count = 0;
        for (int y = 0; y < kHeight; ++y)
            for (int x = 0; x < kWidth; ++x)
            {
                if (image.isClear(x, y)) continue;
                const unsigned char* p = image.at(x, y);
                sum += static_cast<float>(p[0] + p[1] + p[2]) / 3.0f;
                ++count;
            }
        CHECK(count > 100);
        mean[pass] = count > 0 ? sum / static_cast<float>(count) : 0.0f;
    }
    printf("grass: mean brightness front %.1f, back %.1f, sun off %.1f\n", mean[0], mean[1],
            mean[2]);
    CHECK(mean[2] < 40.0f);
    CHECK(mean[0] > mean[2] + 30.0f);
    CHECK(mean[1] > mean[2] + 30.0f);
    CHECK(fabsf(mean[0] - mean[1]) < 0.1f * mean[0]);

    sun->queueFree();
    grass->queueFree();
    {
        Frame frame(*scene.tree);
    }
    environment->setEnvironment(whiteAmbient());
    textures.destroy(solid);
}

void testTypesRender(TextureStore& textures, Scene& scene, Node* root)
{
    const TextureId solid = makeTexture(textures, 255, 255, "grass types");
    scene.camera->setPerspective(60.0f, 0.1f, 200.0f);
    place(scene, Math::Vec3(0.0f, 1.2f, 6.0f), Math::Vec3(0.0f, 0.5f, 0.0f));

    Grass3D* grass = root->createChild<Grass3D>();
    GrassTypeDesc red;
    red.texture = solid;
    red.colourA = Math::Vec3(1.0f, 0.0f, 0.0f);
    red.colourB = Math::Vec3(1.0f, 0.0f, 0.0f);
    GrassTypeDesc blue = red;
    blue.colourA = Math::Vec3(0.0f, 0.0f, 1.0f);
    blue.colourB = Math::Vec3(0.0f, 0.0f, 1.0f);
    CHECK(grass->addType(red) == 0);
    CHECK(grass->addType(blue) == 1);
    CHECK(grass->getTypeCount() == 2);
    grass->setCellSize(4.0f);
    grass->setDensity(3.0f);
    grass->setDrawDistance(40.0f);
    grass->setWind(0.0f);
    grass->setExtent(-4.0f, -4.0f, 4.0f, 4.0f);
    grass->setMaskResolution(65, 65);
    CHECK(grass->paint(Math::Vec3(-2.5f, 0.0f, 0.0f), 1.5f, 1.0f, 0));
    CHECK(grass->paint(Math::Vec3(2.5f, 0.0f, 0.0f), 1.5f, 1.0f, 1));
    for (int i = 0; i < 3; ++i)
    {
        Frame frame(*scene.tree);
    }
    Image image;
    CHECK(grab(scene, image));
    int redLeft = 0;
    int blueRight = 0;
    int wrong = 0;
    for (int y = 0; y < kHeight; ++y)
    {
        for (int x = 0; x < kWidth; ++x)
        {
            if (image.isClear(x, y)) continue;
            const unsigned char* p = image.at(x, y);
            const bool isRed = p[0] > p[2] + 8;
            const bool isBlue = p[2] > p[0] + 8;
            if (x < kWidth / 2)
            {
                if (isRed) ++redLeft;
                if (isBlue) ++wrong;
            }
            else
            {
                if (isBlue) ++blueRight;
                if (isRed) ++wrong;
            }
        }
    }
    printf("grass: two types, %d red pixels on the left, %d blue on the right, %d misplaced\n",
            redLeft, blueRight, wrong);
    CHECK(redLeft > 50 && blueRight > 50 && wrong == 0);
    CHECK(grass->getInstanceCount(0) > 0 && grass->getInstanceCount(1) > 0);
    CHECK(grass->getDrawnInstanceCount(0) > 0 && grass->getDrawnInstanceCount(1) > 0);
    grass->queueFree();
    Frame frame(*scene.tree);
    textures.destroy(solid);
}

void testShadows(TextureStore& textures, Scene& scene, Node* root)
{
    const TextureId solid = makeTexture(textures, 255, 255, "grass shadows");
    scene.camera->setPerspective(60.0f, 0.1f, 200.0f);
    place(scene, Math::Vec3(0.0f, 3.0f, 8.0f), Math::Vec3(0.0f, 0.5f, 0.0f));

    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    sun->setShadow(true);
    sun->setShadowMaxDistance(60.0f);
    sun->setRotation(Math::Vec3(-0.9f, 0.6f, 0.0f));
    MeshInstance3D* caster = root->createChild<MeshInstance3D>();
    const MeshId boxMesh = createBoxMesh(*Main::meshes(), Math::Vec3(1.0f, 1.0f, 1.0f));
    caster->setMesh(boxMesh);
    caster->setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));

    Grass3D* grass = root->createChild<Grass3D>();
    grass->setTexture(solid);
    grass->setDensity(1.0f);
    grass->setDrawDistance(30.0f);
    grass->setExtent(-6.0f, -6.0f, 6.0f, 6.0f);
    for (int i = 0; i < 3; ++i)
    {
        Frame frame(*scene.tree);
    }
    RenderStats on;
    {
        Frame frame(*scene.tree);
        on = frame.stats();
    }
    CHECK(grass->getDrawnInstanceCount() > 0);
    CHECK(on.multimeshInstancesDrawn > 0);
    for (int cx = -2; cx <= 1; ++cx)
        for (int cz = -2; cz <= 1; ++cz)
            if (MultiMeshInstance3D* node = grass->getCellNode(0, cx, cz))
                CHECK(!node->getCastShadows());

    grass->queueFree();
    for (int i = 0; i < 2; ++i)
    {
        Frame frame(*scene.tree);
    }
    RenderStats off;
    {
        Frame frame(*scene.tree);
        off = frame.stats();
    }
    printf("grass: shadows with grass %u draws %u triangles, without %u draws %u triangles; "
           "colour pass %u draws with, %u without\n",
            on.shadowDrawCalls, on.shadowTriangles, off.shadowDrawCalls, off.shadowTriangles,
            on.drawCalls, off.drawCalls);
    CHECK(on.shadowTriangles > 0 && on.shadowDrawCalls > 0);
    CHECK(on.shadowTriangles == off.shadowTriangles);
    CHECK(on.shadowDrawCalls == off.shadowDrawCalls);
    CHECK(on.drawCalls > off.drawCalls);
    sun->queueFree();
    caster->queueFree();
    {
        Frame frame(*scene.tree);
    }
    Main::meshes()->destroy(boxMesh);
    textures.destroy(solid);
}

void measure(TextureStore& textures, Scene& scene, Node* root)
{
    const TextureId solid = makeTexture(textures, 255, 255, "grass measure");
    scene.camera->setPerspective(60.0f, 0.1f, 600.0f);
    place(scene, Math::Vec3(0.0f, 8.0f, 100.0f), Math::Vec3(0.0f, 4.0f, 0.0f));

    Grass3D* grass = nullptr;
    for (int pass = 0; pass < 4; ++pass)
    {
        if (pass == 3)
        {
            grass->fill(0, 1.0f);
            grass->fill(1, 0.0f);
        }
        if (pass == 1)
        {
            grass = root->createChild<Grass3D>();
            grass->setTexture(solid);
            GrassTypeDesc second;
            second.texture = solid;
            second.height = 1.4f;
            second.colourA = Math::Vec3(0.85f, 0.75f, 0.45f);
            second.colourB = Math::Vec3(0.95f, 0.88f, 0.60f);
            grass->addType(second);
            grass->setExtent(-150.0f, -150.0f, 150.0f, 150.0f);
            grass->setMaskResolution(512, 512);
            grass->createMask();
            grass->paint(Math::Vec3(-25.0f, 0.0f, 70.0f), 30.0f, 1.0f, 0);
            grass->paint(Math::Vec3(25.0f, 0.0f, 75.0f), 30.0f, 1.0f, 0);
            grass->paint(Math::Vec3(0.0f, 0.0f, 55.0f), 25.0f, 1.0f, 0);
            grass->paint(Math::Vec3(0.0f, 0.0f, 80.0f), 14.0f, 1.0f, 1);
            grass->paint(Math::Vec3(-45.0f, 0.0f, 60.0f), 12.0f, 1.0f, 1);
            grass->paint(Math::Vec3(40.0f, 0.0f, 60.0f), 12.0f, 1.0f, 1);
        }
        const char* const labels[4] = { "without grass", "with grass   ",
            "with grass and two influencers", "ground fully painted with the first type" };
        for (int i = 0; i < 4; ++i)
        {
            if (pass == 2)
            {
                grass->addInfluencer(Math::Vec3(0.0f, 0.5f, 70.0f), 3.0f, 30.0f);
                grass->addInfluencer(Math::Vec3(14.0f, 0.5f, 70.0f), 1.5f, 30.0f);
            }
            Frame frame(*scene.tree);
        }
        float total = 0.0f;
        std::uint32_t upload = 0;
        int awake = 0;
        for (int i = 0; i < 30; ++i)
        {
            if (pass == 2)
            {
                grass->addInfluencer(Math::Vec3(0.0f, 0.5f, 70.0f), 3.0f, 30.0f);
                grass->addInfluencer(Math::Vec3(14.0f, 0.5f, 70.0f), 1.5f, 30.0f);
            }
            Frame frame(*scene.tree);
            if (grass) total += grass->getUpdateMilliseconds();
            if (grass) awake += grass->getAwakeInstanceCount();
            upload += frame.stats().multimeshUploadBytes;
        }
        if (pass == 2)
        {
            grass->addInfluencer(Math::Vec3(0.0f, 0.5f, 70.0f), 3.0f, 30.0f);
            grass->addInfluencer(Math::Vec3(14.0f, 0.5f, 70.0f), 1.5f, 30.0f);
        }
        Frame frame(*scene.tree);
        const RenderStats& stats = frame.stats();
        printf("grass measurement, the demo configuration (two painted types, density 0.35, draw "
               "distance 60), camera (0, 8, 100), wind 1:\n");
        printf("  %s: draws %u, triangles %u, multimesh instances %u\n", labels[pass],
                stats.drawCalls, stats.triangles, stats.multimeshInstancesDrawn);
        if (grass)
        {
            printf("  cells %d (%d drawn), tufts %d, tufts drawn %d, awake %d\n",
                    grass->getActiveCellCount(), grass->getVisibleCellCount(),
                    grass->getInstanceCount(), grass->getDrawnInstanceCount(),
                    grass->getAwakeInstanceCount());
            printf("  update CPU %.3f ms per frame, instance upload %u bytes per frame, awake %d "
                   "on "
                   "average\n",
                    total / 30.0f, upload / 30, awake / 30);
        }
    }
    grass->queueFree();
    Frame frame(*scene.tree);
    textures.destroy(solid);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    testPure();
    testSimulation();

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "grass test";
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

    Node3D holder;
    WorldEnvironment* environment = holder.createChild<WorldEnvironment>();
    environment->setEnvironment(whiteAmbient());
    environment->reparent(root);

    Scene scene;
    scene.tree = &tree;
    scene.root = root;
    scene.camera = holder.createChild<Camera3D>();
    scene.camera->setPerspective(60.0f, 0.1f, 200.0f);
    scene.camera->reparent(root);
    scene.camera->makeCurrent();

    MeshStore& meshes = *Main::meshes();
    TextureStore& textures = *Main::textures();
    MaterialStore& materials = *Main::materials();
    materials.defaultMaterial();
    {
        Frame frame(tree);
    }

    testNodeClass();

    const std::size_t meshBaseline = meshes.count();
    const std::size_t textureBaseline = textures.count();
    const std::size_t materialBaseline = materials.count();

    testPlacement(textures);
    testCells(textures);
    testCulling(textures, scene);
    testInfluencers(textures);
    testMask(textures);
    CHECK(textures.count() == textureBaseline);
    CHECK(materials.count() == materialBaseline);
    CHECK(meshes.count() == meshBaseline);

    testRender(textures, scene, root);
    testLighting(textures, scene, root, environment);
    testTypesRender(textures, scene, root);
    testShadows(textures, scene, root);
    {
        Frame frame(tree);
    }
    CHECK(textures.count() == textureBaseline);
    CHECK(materials.count() == materialBaseline);
    CHECK(meshes.count() == meshBaseline);

    {
        const TextureId texture = makeTexture(textures, 255, 255, "grass leak");
        Grass3D* grass = root->createChild<Grass3D>();
        grass->setTexture(texture);
        grass->setExtent(-20.0f, -20.0f, 20.0f, 20.0f);
        grass->setDrawDistance(30.0f);
        GrassTypeDesc extra;
        extra.texture = texture;
        CHECK(grass->addType(extra) == 1);
        CHECK(grass->createMask(33, 33));
        CHECK(grass->paint(Math::Vec3(0.0f, 0.0f, 0.0f), 5.0f, 1.0f, 1));
        place(scene, Math::Vec3(0.0f, 2.0f, 5.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
        {
            Frame frame(tree);
        }
        CHECK(grass->getActiveCellCount() > 0);
        CHECK(meshes.count() > meshBaseline && materials.count() > materialBaseline);
        ct::Vector<MultiMeshId> ids;
        for (int type = 0; type < 2; ++type)
            for (int cx = -5; cx < 5; ++cx)
                for (int cz = -5; cz < 5; ++cz)
                    if (MultiMeshInstance3D* node = grass->getCellNode(type, cx, cz))
                        ids.push_back(node->getMultiMesh()->getId());
        CHECK(ids.size() > 4);
        grass->queueFree();
        {
            Frame frame(tree);
        }
        for (std::size_t i = 0; i < ids.size(); ++i)
            CHECK(!Main::rendering()->multimeshValid(ids[i]));
        textures.destroy(texture);
        CHECK(textures.count() == textureBaseline);
        CHECK(materials.count() == materialBaseline);
        CHECK(meshes.count() == meshBaseline);
    }

    {
        Grass3D loose;
        loose.setExtent(-4.0f, -4.0f, 4.0f, 4.0f);
        loose.update(Math::Vec3(0.0f, 1.0f, 0.0f), nullptr, kDt);
        CHECK(loose.getActiveCellCount() > 0);
        CHECK(loose.getTypeCount() == 1);
    }
    CHECK(textures.count() == textureBaseline);
    CHECK(materials.count() == materialBaseline);
    CHECK(meshes.count() == meshBaseline);

    measure(textures, scene, root);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("grass ok\n");
    return 0;
}
