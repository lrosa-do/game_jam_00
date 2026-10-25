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
const int kTolerance = 3;
const int kBoxCount = 4;
const Math::Vec3 kBoxPositions[kBoxCount] = { Math::Vec3(-3.0f, 0.0f, -10.0f),
    Math::Vec3(-1.0f, 0.0f, -20.0f), Math::Vec3(0.0f, 0.0f, -30.0f),
    Math::Vec3(3.0f, 0.0f, -40.0f) };

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kYellow = { 255, 255, 0 };

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

bool sameDraw(const RenderStats& a, const RenderStats& b)
{
    return a.instancesDrawn == b.instancesDrawn && a.instancesCulled == b.instancesCulled &&
           a.surfacesDrawn == b.surfacesDrawn && a.drawCalls == b.drawCalls &&
           a.triangles == b.triangles && a.visibilityRangeHidden == b.visibilityRangeHidden;
}

void compareModes(SceneTree& tree, Capture& withTree, const char* label)
{
    RenderingServer* server = Main::rendering();
    Capture linear;
    server->setCullingTreeEnabled(true);
    CHECK(capture(tree, withTree));
    CHECK(capture(tree, withTree));
    server->setCullingTreeEnabled(false);
    CHECK(capture(tree, linear));
    server->setCullingTreeEnabled(true);
    CHECK(withTree.stats.cullTree);
    CHECK(!linear.stats.cullTree);
    if (!sameDraw(withTree.stats, linear.stats))
        printf("%s: tree drew %u culled %u hidden %u, linear drew %u culled %u hidden %u\n", label,
                withTree.stats.instancesDrawn, withTree.stats.instancesCulled,
                withTree.stats.visibilityRangeHidden, linear.stats.instancesDrawn,
                linear.stats.instancesCulled, linear.stats.visibilityRangeHidden);
    CHECK(sameDraw(withTree.stats, linear.stats));
    if (!withTree.image.same(linear.image)) printf("%s: pixels differ\n", label);
    CHECK(withTree.image.same(linear.image));
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "visibility range test";
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

    Node3D holder;
    Camera3D* camera = holder.createChild<Camera3D>();
    camera->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    camera->reparent(root);
    camera->makeCurrent();

    const MeshId boxMesh = createBoxMesh(*Main::meshes(), Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(boxMesh.valid());

    MeshInstance3D* boxes[kBoxCount];
    for (int i = 0; i < kBoxCount; ++i)
    {
        boxes[i] = holder.createChild<MeshInstance3D>();
        boxes[i]->setMesh(boxMesh);
        boxes[i]->setPosition(kBoxPositions[i]);
        boxes[i]->setColor(Math::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
        boxes[i]->reparent(root);
    }

    auto setRanges = [&](float begin, float end, float beginMargin, float endMargin)
    {
        for (int i = 0; i < kBoxCount; ++i)
            server->instanceSetVisibilityRange(boxes[i]->getInstance(), begin, end, beginMargin,
                    endMargin);
    };

    Capture frame;

    compareModes(tree, frame, "no range");
    CHECK(frame.stats.instancesDrawn == 4);
    CHECK(frame.stats.visibilityRangeHidden == 0);
    CHECK(frame.image.near(160, 120, kYellow, "centre box without a range"));

    setRanges(0.0f, 25.0f, 0.0f, 0.0f);
    compareModes(tree, frame, "end at 25");
    CHECK(frame.stats.instancesDrawn == 2);
    CHECK(frame.stats.instancesCulled == 2);
    CHECK(frame.stats.visibilityRangeHidden == 2);
    CHECK(frame.image.near(160, 120, kClear, "centre box beyond the end"));

    setRanges(15.0f, 0.0f, 0.0f, 0.0f);
    compareModes(tree, frame, "begin at 15");
    CHECK(frame.stats.instancesDrawn == 3);
    CHECK(frame.stats.visibilityRangeHidden == 1);
    CHECK(frame.image.near(160, 120, kYellow, "centre box past the begin"));

    setRanges(15.0f, 35.0f, 0.0f, 0.0f);
    compareModes(tree, frame, "between 15 and 35");
    CHECK(frame.stats.instancesDrawn == 2);
    CHECK(frame.stats.visibilityRangeHidden == 2);

    setRanges(0.0f, 0.0f, 0.0f, 0.0f);
    compareModes(tree, frame, "range removed");
    CHECK(frame.stats.instancesDrawn == 4);
    CHECK(frame.stats.visibilityRangeHidden == 0);

    {
        server->instanceSetVisibilityRange(boxes[1]->getInstance(), 0.0f, 22.0f, 0.0f, 4.0f);
        const float cameraZ[5] = { 0.0f, 7.0f, 0.0f, -3.0f, 0.0f };
        const std::uint32_t expected[5] = { 4, 3, 3, 4, 4 };
        const char* const labels[5] = { "inside, was visible", "past end plus margin",
            "back inside the margin, was hidden", "below end minus margin",
            "back inside the margin, was visible" };
        for (int step = 0; step < 5; ++step)
        {
            camera->setPosition(Math::Vec3(0.0f, 0.0f, cameraZ[step]));
            CHECK(capture(tree, frame));
            if (frame.stats.instancesDrawn != expected[step])
                printf("hysteresis, %s: drew %u, expected %u\n", labels[step],
                        frame.stats.instancesDrawn, expected[step]);
            CHECK(frame.stats.instancesDrawn == expected[step]);
            CHECK(frame.stats.visibilityRangeHidden == 4 - expected[step]);
        }
        server->instanceSetVisibilityRange(boxes[1]->getInstance(), 0.0f, 0.0f, 0.0f, 0.0f);
    }

    {
        ManualMesh3D* pair = holder.createChild<ManualMesh3D>();
        const float depths[2] = { -10.0f, -40.0f };
        for (int s = 0; s < 2; ++s)
        {
            const float z = depths[s];
            CHECK(pair->surfaceBegin(prisma::Topology::Triangles));
            pair->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
            pair->surfaceAddVertex(Math::Vec3(2.0f, 2.0f, z));
            pair->surfaceAddVertex(Math::Vec3(3.0f, 2.0f, z));
            pair->surfaceAddVertex(Math::Vec3(2.5f, 3.0f, z));
            CHECK(pair->surfaceEnd());
        }
        pair->reparent(root);

        Capture both;
        compareModes(tree, both, "two surfaces without a range");
        CHECK(both.stats.instancesDrawn == 5);
        CHECK(both.stats.visibilityRangeHidden == 0);

        server->instanceSetVisibilityRange(pair->getInstance(), 0.0f, 25.0f, 0.0f, 0.0f);
        Capture nearOnly;
        compareModes(tree, nearOnly, "two surfaces, end at 25");
        CHECK(nearOnly.stats.instancesDrawn == 5);
        CHECK(nearOnly.stats.visibilityRangeHidden == 1);
        CHECK(nearOnly.stats.surfacesDrawn + 1 == both.stats.surfacesDrawn);

        server->instanceSetVisibilityRange(pair->getInstance(), 0.0f, 5.0f, 0.0f, 0.0f);
        Capture none;
        compareModes(tree, none, "two surfaces, end at 5");
        CHECK(none.stats.instancesDrawn == 4);
        CHECK(none.stats.instancesCulled == both.stats.instancesCulled + 1);
        CHECK(none.stats.visibilityRangeHidden == 2);
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
