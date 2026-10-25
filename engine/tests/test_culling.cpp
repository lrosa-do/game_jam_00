#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/ManualMesh3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/vector.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 320;
const int kHeight = 240;
const int kColumns = 20;
const int kRows = 10;
const int kGridCount = kColumns * kRows;
const float kSpacing = 3.0f;
const int kTolerance = 3;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kYellow = { 255, 255, 0 };
const Rgb kGreen = { 0, 255, 0 };

struct Image
{
    ct::Vector<unsigned char> pixels;

    bool read()
    {
        pixels.resize(static_cast<std::size_t>(kWidth) * kHeight * 4);
        prisma::Rect rect;
        rect.x = 0;
        rect.y = 0;
        rect.width = kWidth;
        rect.height = kHeight;
        return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixels.data());
    }

    bool same(const Image& other) const
    {
        return pixels.size() == other.pixels.size() &&
               memcmp(pixels.data(), other.pixels.data(), pixels.size()) == 0;
    }

    bool near(int x, int y, Rgb expected, const char* label) const
    {
        const unsigned char* p = pixels.data() + (static_cast<std::size_t>(y) * kWidth + x) * 4;
        const int dr = p[0] - expected.r;
        const int dg = p[1] - expected.g;
        const int db = p[2] - expected.b;
        const bool match = dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance &&
                           dg <= kTolerance && db >= -kTolerance && db <= kTolerance;
        if (!match)
            printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, p[0], p[1],
                    p[2], expected.r, expected.g, expected.b);
        return match;
    }
};

struct Capture
{
    Image image;
    RenderStats stats;
};

bool capture(SceneTree& tree, Capture& out)
{
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    out.stats = Main::rendering()->getStats();
    const bool read = out.image.read();
    Main::rendering()->endFrame(true);
    return read;
}

void frame(SceneTree& tree)
{
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    Main::rendering()->endFrame(true);
}

bool sameDraw(const RenderStats& a, const RenderStats& b)
{
    return a.instancesDrawn == b.instancesDrawn && a.instancesCulled == b.instancesCulled &&
           a.surfacesDrawn == b.surfacesDrawn && a.drawCalls == b.drawCalls &&
           a.triangles == b.triangles;
}

void compareModes(SceneTree& tree, const char* label)
{
    RenderingServer* server = Main::rendering();
    Capture withTree;
    Capture linear;
    server->setCullingTreeEnabled(true);
    frame(tree);
    CHECK(capture(tree, withTree));
    server->setCullingTreeEnabled(false);
    CHECK(capture(tree, linear));
    server->setCullingTreeEnabled(true);
    CHECK(withTree.stats.cullTree);
    CHECK(!linear.stats.cullTree);
    CHECK(linear.stats.cullEntries == 0 && linear.stats.cullNodes == 0);
    if (!sameDraw(withTree.stats, linear.stats))
        printf("%s: tree drew %u culled %u, linear drew %u culled %u\n", label,
                withTree.stats.instancesDrawn, withTree.stats.instancesCulled,
                linear.stats.instancesDrawn, linear.stats.instancesCulled);
    CHECK(sameDraw(withTree.stats, linear.stats));
    if (!withTree.image.same(linear.image)) printf("%s: pixels differ\n", label);
    CHECK(withTree.image.same(linear.image));
}

Math::Vec3 gridPosition(int index)
{
    return Math::Vec3(static_cast<float>(index % kColumns) * kSpacing,
            static_cast<float>(index / kColumns) * kSpacing, 0.0f);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "culling test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr);
    if (!Main::rendering()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;
    Window* window = root->castTo<Window>();
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
    RenderingServer::singleton()->scenarioSetEnvironment(window->getScenario(), whiteAmbient());

    RenderingServer* server = Main::rendering();
    CHECK(server->cullingTreeEnabled());
    CHECK(server->cullingLeafCapacity() == 2);

    Node3D holder;
    Camera3D* camera = holder.createChild<Camera3D>();
    camera->setPosition(Math::Vec3(10.5f, 4.5f, 12.0f));
    camera->reparent(root);
    camera->makeCurrent();

    const MeshId boxMesh = createBoxMesh(*Main::meshes(), Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(boxMesh.valid());

    ct::Vector<MeshInstance3D*> grid;
    for (int i = 0; i < kGridCount; ++i)
    {
        MeshInstance3D* box = holder.createChild<MeshInstance3D>();
        box->setMesh(boxMesh);
        box->setPosition(gridPosition(i));
        box->setColor(Math::Vec4(0.3f + 0.7f * static_cast<float>(i % kColumns) / kColumns,
                0.3f + 0.7f * static_cast<float>(i / kColumns) / kRows, 0.5f, 1.0f));
        box->reparent(root);
        grid.push_back(box);
    }

    {
        Capture withTree;
        Capture linear;
        server->setCullingTreeEnabled(true);
        frame(tree);
        CHECK(capture(tree, withTree));
        server->setCullingTreeEnabled(false);
        CHECK(capture(tree, linear));
        server->setCullingTreeEnabled(true);

        CHECK(withTree.stats.instancesDrawn > 10);
        CHECK(withTree.stats.instancesDrawn < kGridCount);
        CHECK(withTree.stats.instancesCulled > 0);
        CHECK(withTree.stats.instancesDrawn + withTree.stats.instancesCulled == kGridCount);
        CHECK(sameDraw(withTree.stats, linear.stats));
        CHECK(withTree.image.same(linear.image));
        CHECK(!linear.stats.cullTree);
        CHECK(linear.stats.cullEntries == 0);
        CHECK(linear.stats.cullNodes == 0);
        CHECK(linear.stats.cullNodesVisited == 0);
        CHECK(linear.stats.cullAccepted == 0);

        const RenderStats& stats = withTree.stats;
        CHECK(stats.cullTree);
        CHECK(stats.cullEntries == static_cast<std::uint32_t>(kGridCount));
        CHECK(stats.cullNodes > 0);
        CHECK(stats.cullNodes <= 2 * stats.cullEntries);
        CHECK(stats.cullDepth > 0);
        CHECK(stats.cullAccepted == stats.instancesDrawn);
        CHECK(stats.cullCandidates >= stats.cullAccepted);
        CHECK(stats.cullNodesVisited > 0);
        CHECK(stats.cullQuality >= 1.0f);
        printf("grid %d entries, %u nodes, depth %u: visited %u nodes, %u candidates, %u accepted,"
               " quality %.2f\n",
                stats.cullEntries, stats.cullNodes, stats.cullDepth, stats.cullNodesVisited,
                stats.cullCandidates, stats.cullAccepted, stats.cullQuality);
        CHECK(stats.cullNodesVisited < stats.cullNodes);
        CHECK(stats.cullNodesVisited < stats.cullEntries);
    }

    MeshInstance3D* mover = grid[8 * kColumns + 17];
    Capture before;
    Capture after;
    CHECK(capture(tree, before));
    CHECK(before.image.near(160, 120, kClear, "centre before the move"));
    mover->setColor(Math::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
    mover->setPosition(Math::Vec3(10.5f, 4.5f, 6.0f));
    CHECK(capture(tree, after));
    CHECK(after.stats.instancesDrawn == before.stats.instancesDrawn + 1);
    CHECK(after.stats.cullAccepted == before.stats.cullAccepted + 1);
    CHECK(after.stats.cullEntries == static_cast<std::uint32_t>(kGridCount));
    CHECK(after.image.near(160, 120, kYellow, "centre after the move"));
    compareModes(tree, "moved instance");

    mover->setPosition(gridPosition(8 * kColumns + 17));
    CHECK(capture(tree, after));
    CHECK(after.stats.instancesDrawn == before.stats.instancesDrawn);
    CHECK(after.image.near(160, 120, kClear, "centre after the move back"));

    {
        MeshInstance3D* hidden = grid[2 * kColumns + 3];
        MeshInstance3D* masked = grid[3 * kColumns + 4];
        Capture visible;
        CHECK(capture(tree, visible));
        hidden->hide();
        masked->setLayerMask(2);
        camera->setCullMask(1);
        compareModes(tree, "hidden and masked instances");
        Capture hiddenFrame;
        CHECK(capture(tree, hiddenFrame));
        CHECK(hiddenFrame.stats.instancesDrawn + 2 == visible.stats.instancesDrawn);
        CHECK(hiddenFrame.stats.instancesCulled == visible.stats.instancesCulled);
        hidden->show();
        masked->setLayerMask(1);
        camera->setCullMask(0xffffffffu);
        CHECK(capture(tree, hiddenFrame));
        CHECK(hiddenFrame.stats.instancesDrawn == visible.stats.instancesDrawn);
        CHECK(hiddenFrame.stats.instancesCulled == visible.stats.instancesCulled);

        camera->setCullMask(4);
        compareModes(tree, "camera mask");
        Capture maskFrame;
        CHECK(capture(tree, maskFrame));
        CHECK(maskFrame.stats.instancesDrawn == 0);
        CHECK(maskFrame.stats.instancesCulled == 0);
        camera->setCullMask(0xffffffffu);
        CHECK(capture(tree, maskFrame));
        CHECK(maskFrame.stats.instancesDrawn == visible.stats.instancesDrawn);
    }

    {
        Capture counted;
        CHECK(capture(tree, counted));
        const std::uint32_t drawn = counted.stats.instancesDrawn;
        MeshInstance3D* removed = grid[0];
        removed->queueFree();
        grid[0] = nullptr;
        Capture removedFrame;
        CHECK(capture(tree, removedFrame));
        CHECK(removedFrame.stats.cullEntries == static_cast<std::uint32_t>(kGridCount - 1));
        CHECK(removedFrame.stats.instancesDrawn == drawn - 1);
        CHECK(removedFrame.stats.cullAccepted == drawn - 1);

        MeshInstance3D* added = holder.createChild<MeshInstance3D>();
        added->setMesh(boxMesh);
        added->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
        added->reparent(root);
        grid[0] = added;
        Capture addedFrame;
        CHECK(capture(tree, addedFrame));
        CHECK(addedFrame.stats.cullEntries == static_cast<std::uint32_t>(kGridCount));
        CHECK(addedFrame.stats.instancesDrawn == drawn);
        CHECK(addedFrame.stats.cullAccepted == drawn);
        compareModes(tree, "membership");
    }

    {
        Capture counted;
        CHECK(capture(tree, counted));
        const std::uint32_t drawn = counted.stats.instancesDrawn;
        const std::uint32_t entries = counted.stats.cullEntries;

        SurfaceData data;
        boxMeshArrays(data, Math::Vec3(1.0f, 1.0f, 1.0f));
        SurfaceLayout layout;
        ct::Vector<std::uint8_t> vertices;
        ct::Vector<std::uint8_t> indices;
        CHECK(MeshStore::interleave(data, layout, vertices, indices));

        const MeshId streamed = Main::meshes()->create("streamed");
        CHECK(streamed.valid());
        const int surface = Main::meshes()->beginSurface(streamed, layout, "streamed surface");
        CHECK(surface == 0);

        MeshInstance3D* node = holder.createChild<MeshInstance3D>();
        node->setMesh(streamed);
        node->setColor(Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        node->setPosition(Math::Vec3(13.5f, 1.5f, 6.0f));
        node->reparent(root);

        Capture pending;
        CHECK(capture(tree, pending));
        CHECK(pending.stats.instancesDrawn == drawn);
        CHECK(pending.stats.cullEntries == entries);
        CHECK(pending.image.near(238, 198, kClear, "streamed mesh before it is finished"));
        CHECK(capture(tree, pending));
        CHECK(pending.stats.instancesDrawn == drawn);

        CHECK(Main::meshes()->uploadVertices(streamed, static_cast<std::uint32_t>(surface), 0,
                vertices.data(), static_cast<std::uint32_t>(vertices.size())));
        CHECK(Main::meshes()->uploadIndices(streamed, static_cast<std::uint32_t>(surface), 0,
                indices.data(), static_cast<std::uint32_t>(indices.size())));
        CHECK(Main::meshes()->finishSurface(streamed, static_cast<std::uint32_t>(surface)));

        Capture finished;
        CHECK(capture(tree, finished));
        CHECK(finished.stats.instancesDrawn == drawn + 1);
        CHECK(finished.stats.cullEntries == entries + 1);
        CHECK(finished.image.near(238, 198, kGreen, "streamed mesh after it is finished"));
        compareModes(tree, "streamed mesh");

        node->queueFree();
        tree.process(1.0 / 60.0);
        Main::meshes()->destroy(streamed);
        CHECK(capture(tree, finished));
        CHECK(finished.stats.instancesDrawn == drawn);
        CHECK(finished.stats.cullEntries == entries);
    }

    {
        Capture counted;
        CHECK(capture(tree, counted));
        const std::uint32_t drawn = counted.stats.instancesDrawn;
        const std::uint32_t entries = counted.stats.cullEntries;

        SurfaceData first;
        boxMeshArrays(first, Math::Vec3(1.0f, 1.0f, 1.0f));
        SurfaceData second;
        boxMeshArrays(second, Math::Vec3(1.0f, 1.0f, 1.0f));
        for (std::size_t i = 0; i < second.positions.size(); ++i) second.positions[i].x -= 50.0f;

        const MeshId growing = Main::meshes()->create("growing");
        CHECK(Main::meshes()->addSurface(growing, first, "first"));
        MeshInstance3D* node = holder.createChild<MeshInstance3D>();
        node->setMesh(growing);
        node->setColor(Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        node->setPosition(Math::Vec3(60.5f, 4.5f, 6.0f));
        node->reparent(root);

        Capture grown;
        CHECK(capture(tree, grown));
        CHECK(grown.stats.cullEntries == entries + 1);
        CHECK(grown.stats.instancesDrawn == drawn);
        CHECK(grown.image.near(160, 120, kClear, "growing mesh before the second surface"));

        CHECK(Main::meshes()->addSurface(growing, second, "second"));
        CHECK(capture(tree, grown));
        CHECK(grown.stats.cullEntries == entries + 2);
        CHECK(grown.stats.instancesDrawn == drawn + 1);
        CHECK(grown.image.near(160, 120, kGreen, "growing mesh after the second surface"));
        compareModes(tree, "growing mesh");

        Main::meshes()->clearSurfaces(growing);
        CHECK(capture(tree, grown));
        CHECK(grown.stats.cullEntries == entries);
        CHECK(grown.stats.instancesDrawn == drawn);

        node->queueFree();
        tree.process(1.0 / 60.0);
        Main::meshes()->destroy(growing);
        CHECK(capture(tree, grown));
        CHECK(grown.stats.cullEntries == entries);
    }

    {
        bool sawPending = false;
        bool sawIdle = false;
        MeshInstance3D* shaker = grid[5 * kColumns + 5];
        for (int i = 0; i < 6; ++i)
        {
            shaker->setPosition(gridPosition(5 * kColumns + 5) +
                                Math::Vec3(0.01f * static_cast<float>(i + 1), 0.0f, 0.0f));
            Capture moving;
            CHECK(capture(tree, moving));
            if (moving.stats.cullRebuildPending) sawPending = true;
        }
        for (int i = 0; i < 400 && !sawIdle; ++i)
        {
            Capture idle;
            CHECK(capture(tree, idle));
            if (!idle.stats.cullRebuildPending) sawIdle = true;
        }
        CHECK(sawPending);
        CHECK(sawIdle);
        compareModes(tree, "after rebuilds");
    }

    {
        Capture capacity2;
        CHECK(capture(tree, capacity2));
        server->setCullingLeafCapacity(8);
        CHECK(server->cullingLeafCapacity() == 8);
        Capture capacity8;
        CHECK(capture(tree, capacity8));
        CHECK(capacity8.stats.cullNodes > 0);
        CHECK(capacity8.stats.cullNodes < capacity2.stats.cullNodes);
        CHECK(capacity8.stats.instancesDrawn == capacity2.stats.instancesDrawn);
        CHECK(capacity8.image.same(capacity2.image));
        for (int i = 0; i < 5; ++i) frame(tree);
        CHECK(capture(tree, capacity8));
        CHECK(capacity8.stats.cullNodes < capacity2.stats.cullNodes);
        compareModes(tree, "leaf capacity 8");
        server->setCullingLeafCapacity(0);
        CHECK(server->cullingLeafCapacity() == 1);
        compareModes(tree, "leaf capacity 1");
        server->setCullingLeafCapacity(2);
    }

    {
        for (int i = 0; i < 6; ++i)
        {
            server->setCullingTreeEnabled(i % 2 == 0);
            CHECK(server->cullingTreeEnabled() == (i % 2 == 0));
            grid[i]->setPosition(gridPosition(i) + Math::Vec3(0.0f, 0.0f, 0.5f));
            frame(tree);
        }
        server->setCullingTreeEnabled(true);
        Capture toggled;
        CHECK(capture(tree, toggled));
        CHECK(toggled.stats.cullTree);
        CHECK(toggled.stats.cullEntries == static_cast<std::uint32_t>(kGridCount));
        compareModes(tree, "toggled");
    }

    {
        for (int i = 0; i < 40; ++i)
        {
            grid[100]->setPosition(
                    gridPosition(100) + Math::Vec3(0.0f, 0.001f * static_cast<float>(i), 0.0f));
            frame(tree);
        }
        grid[101]->setPosition(gridPosition(101) + Math::Vec3(0.0f, 0.0f, 1.0f));
        frame(tree);
    }

    {
        Capture before;
        CHECK(capture(tree, before));
        ManualMesh3D* pair = holder.createChild<ManualMesh3D>();
        const float centers[2] = { 10.5f, 1000.0f };
        for (int s = 0; s < 2; ++s)
        {
            const float x = centers[s];
            CHECK(pair->surfaceBegin(prisma::Topology::Triangles));
            pair->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
            pair->surfaceAddVertex(Math::Vec3(x - 0.5f, 4.0f, 6.0f));
            pair->surfaceAddVertex(Math::Vec3(x + 0.5f, 4.0f, 6.0f));
            pair->surfaceAddVertex(Math::Vec3(x, 5.0f, 6.0f));
            CHECK(pair->surfaceEnd());
        }
        pair->reparent(root);
        frame(tree);
        Capture split;
        CHECK(capture(tree, split));
        CHECK(split.stats.cullEntries == before.stats.cullEntries + 2);
        CHECK(split.stats.instancesDrawn == before.stats.instancesDrawn + 1);
        CHECK(split.stats.cullAccepted == before.stats.cullAccepted + 1);
        CHECK(split.stats.surfacesDrawn == before.stats.surfacesDrawn + 1);
        CHECK(split.stats.triangles == before.stats.triangles + 1);
        compareModes(tree, "surface entries");
        server->instanceSetSurfaceVisible(pair->getInstance(), 0, false);
        Capture hiddenSurface;
        CHECK(capture(tree, hiddenSurface));
        CHECK(hiddenSurface.stats.surfacesDrawn == before.stats.surfacesDrawn);
        CHECK(hiddenSurface.stats.triangles == before.stats.triangles);
        CHECK(hiddenSurface.image.same(before.image));
        compareModes(tree, "hidden surface");
        server->instanceSetSurfaceVisible(pair->getInstance(), 0, true);
        Capture shownSurface;
        CHECK(capture(tree, shownSurface));
        CHECK(shownSurface.stats.surfacesDrawn == split.stats.surfacesDrawn);
        CHECK(shownSurface.image.same(split.image));
        CHECK(grid[0]->isSurfaceVisible(0));
        grid[0]->setSurfaceVisible(0, false);
        CHECK(!grid[0]->isSurfaceVisible(0));
        CHECK(grid[0]->isSurfaceVisible(1));
        Capture hiddenBox;
        CHECK(capture(tree, hiddenBox));
        CHECK(hiddenBox.stats.surfacesDrawn + 1 == split.stats.surfacesDrawn);
        compareModes(tree, "hidden box surface");
        grid[0]->setSurfaceVisible(0, true);
        CHECK(grid[0]->isSurfaceVisible(0));
        CHECK(capture(tree, hiddenBox));
        CHECK(hiddenBox.stats.surfacesDrawn == split.stats.surfacesDrawn);
        pair->setPosition(Math::Vec3(-989.5f, 0.0f, 0.0f));
        frame(tree);
        Capture swapped;
        CHECK(capture(tree, swapped));
        CHECK(swapped.stats.cullEntries == split.stats.cullEntries);
        CHECK(swapped.stats.surfacesDrawn == split.stats.surfacesDrawn);
        CHECK(!swapped.image.same(before.image));
        compareModes(tree, "surface entries moved");
        pair->setPosition(Math::Vec3(0.0f, 500.0f, 0.0f));
        frame(tree);
        Capture gone;
        CHECK(capture(tree, gone));
        CHECK(gone.stats.instancesDrawn == before.stats.instancesDrawn);
        CHECK(gone.stats.instancesCulled == before.stats.instancesCulled + 1);
        CHECK(gone.image.same(before.image));
        compareModes(tree, "surface entries culled");
    }

    {
        Capture native;
        CHECK(capture(tree, native));
        CHECK(window->getScaling3D() == 1.0f);
        window->setScaling3D(2.0f);
        CHECK(window->getScaling3D() == 2.0f);
        frame(tree);
        Capture doubled;
        CHECK(capture(tree, doubled));
        CHECK(sameDraw(doubled.stats, native.stats));
        CHECK(doubled.image.near(2, 2, kClear, "render scale 2, corner"));
        CHECK(doubled.image.near(kWidth - 3, kHeight - 3, kClear, "render scale 2, far corner"));
        int different = 0;
        for (std::size_t i = 0; i < native.image.pixels.size(); ++i)
        {
            const int delta = native.image.pixels[i] - doubled.image.pixels[i];
            if (delta > 96 || delta < -96) ++different;
        }
        CHECK(different < kWidth * kHeight / 50);
        float depth = 0.0f;
        CHECK(Main::rendering()->viewportReadDepth(window->getViewportId(), kWidth - 1, kHeight - 1,
                depth));
        CHECK(!Main::rendering()->viewportReadDepth(window->getViewportId(), kWidth, kHeight,
                depth));
        window->setScaling3D(0.5f);
        frame(tree);
        Capture halved;
        CHECK(capture(tree, halved));
        CHECK(sameDraw(halved.stats, native.stats));
        CHECK(halved.image.near(2, 2, kClear, "render scale 0.5, corner"));
        window->setScaling3D(8.0f);
        CHECK(window->getScaling3D() == 2.0f);
        window->setScaling3D(1.0f);
        frame(tree);
        Capture restored;
        CHECK(capture(tree, restored));
        CHECK(restored.image.same(native.image));
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
