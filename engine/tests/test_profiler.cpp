#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/gui/ProfilerPanel.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 320;
const int kHeight = 240;
const int kTolerance = 2;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kRed = { 255, 0, 0 };
const Rgb kBlack = { 0, 0, 0 };

void spin(double milliseconds)
{
    const std::uint64_t until = time_nanos() + static_cast<std::uint64_t>(milliseconds * 1000000.0);
    while (time_nanos() < until)
    {
    }
}

const ProfileSample* findSample(const ProfileSample* samples, std::uint32_t count, const char* name)
{
    for (std::uint32_t i = 0; i < count; ++i)
    {
        if (strcmp(samples[i].name, name) == 0) return &samples[i];
    }
    return nullptr;
}

const ProfileSample* cpuSample(const char* name)
{
    return findSample(Main::profiler()->cpuSamples(), Main::profiler()->cpuSampleCount(), name);
}

const ProfileSample* gpuSample(const char* name)
{
    return findSample(Main::profiler()->gpuSamples(), Main::profiler()->gpuSampleCount(), name);
}

bool pixelIs(int x, int y, Rgb expected, const char* label)
{
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    if (!Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixel))
    {
        printf("%s: pixel (%d,%d) could not be read\n", label, x, y);
        return false;
    }
    const int dr = pixel[0] - expected.r;
    const int dg = pixel[1] - expected.g;
    const int db = pixel[2] - expected.b;
    const bool same = dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance &&
                      dg <= kTolerance && db >= -kTolerance && db <= kTolerance;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, pixel[0],
                pixel[1], pixel[2], expected.r, expected.g, expected.b);
    return same;
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

class PanelNode : public Node
{
public:
    PanelNode()
            : frames(0)
    {
        setProcess(true);
    }

    ProfilerPanel panel;
    int frames;

protected:
    void notificationHandler(int what) override
    {
        Node::notificationHandler(what);
        if (what != kNotificationProcess || !Main::gui() || !Main::profiler()) return;
        ++frames;
        panel.draw(Main::gui()->context(), *Main::profiler(), Main::rendering()->getStats());
    }
};

std::uint64_t cpuFrames = 0;

void cpuFrame(double firstMilliseconds, double nestedMilliseconds)
{
    RenderProfiler* profiler = Main::profiler();
    profiler->beginFrame();
    CHECK(profiler->begin("A"));
    spin(firstMilliseconds);
    if (nestedMilliseconds > 0.0)
    {
        CHECK(profiler->begin("B"));
        spin(nestedMilliseconds);
        profiler->end();
    }
    profiler->end();
    profiler->endFrame();
    ++cpuFrames;
}

void testCpu()
{
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;
    CHECK(RenderProfiler::singleton() == profiler);
    CHECK(profiler->enabled());
    CHECK(profiler->cpuSampleCount() == 0);
    CHECK(profiler->frameNumber() == 0);

    profiler->beginFrame();
    CHECK(profiler->begin("A"));
    spin(2.0);
    CHECK(profiler->begin("B"));
    spin(1.0);
    profiler->end();
    profiler->end();
    profiler->addSample("X", 1.5f);
    profiler->endFrame();
    ++cpuFrames;
    CHECK(profiler->frameNumber() == 1);
    CHECK(profiler->cpuSampleCount() == 4);

    const ProfileSample* a = cpuSample("A");
    const ProfileSample* b = cpuSample("B");
    const ProfileSample* x = cpuSample("X");
    const ProfileSample* frame = cpuSample("Frame");
    CHECK(a && b && x && frame);
    if (!a || !b || !x || !frame) return;
    CHECK(a->milliseconds >= 2.0f && a->milliseconds < 30.0f);
    CHECK(b->milliseconds >= 1.0f && b->milliseconds < 30.0f);
    CHECK(a->depth == 0 && b->depth == 1);
    CHECK(frame->milliseconds >= a->milliseconds);
    CHECK(fabsf(x->milliseconds - 1.5f) < 0.0001f);
    CHECK(profiler->cpuFrameMilliseconds() >= 0.0f);
    CHECK(a->historyCount == 1);

    profiler->beginFrame();
    std::uint32_t opened = 0;
    for (std::uint32_t i = 0; i < RenderProfiler::kMaxDepth; ++i)
    {
        if (profiler->begin("D")) ++opened;
    }
    CHECK(opened == RenderProfiler::kMaxDepth);
    CHECK(!profiler->begin("D"));
    CHECK(!profiler->begin("A"));
    for (std::uint32_t i = 0; i < opened; ++i) profiler->end();
    CHECK(profiler->begin("A"));
    profiler->end();
    profiler->endFrame();
    ++cpuFrames;
    CHECK(profiler->cpuSampleCount() == 5);
    CHECK(cpuSample("D") != nullptr);

    for (int i = 0; i < 5; ++i) cpuFrame(0.5 + i * 0.25, 0.25);
    a = cpuSample("A");
    CHECK(a && a->historyCount == cpuFrames);
    if (a)
    {
        float total = 0.0f;
        float peak = 0.0f;
        for (std::uint32_t i = 0; i < a->historyCount; ++i)
        {
            total += a->history[i];
            if (a->history[i] > peak) peak = a->history[i];
        }
        const float mean = total / static_cast<float>(a->historyCount);
        CHECK(fabsf(a->average - mean) < 0.001f);
        CHECK(fabsf(a->maximum - peak) < 0.0001f);
        CHECK(a->maximum >= a->average);
        CHECK(a->average > 0.0f);
    }
    const ProfileSample* d = cpuSample("D");
    CHECK(d && d->historyCount == cpuFrames - 1);

    time_sleep(300);
    cpuFrame(2.0, 0.0);
    a = cpuSample("A");
    CHECK(a != nullptr);
    if (!a) return;
    const float firstDisplay = a->display;
    CHECK(firstDisplay == a->milliseconds);
    CHECK(profiler->fps() > 0.0f);

    cpuFrame(5.0, 0.0);
    CHECK(a->milliseconds >= 5.0f);
    CHECK(a->display == firstDisplay);
    CHECK(profiler->cpuFrameMilliseconds() >= 0.0f);

    time_sleep(300);
    cpuFrame(3.0, 0.0);
    CHECK(a->display == a->milliseconds);
    CHECK(a->display != firstDisplay);
    CHECK(profiler->fps() > 0.0f);
    CHECK(profiler->frameNumber() == cpuFrames);

    const std::uint32_t countBefore = profiler->cpuSampleCount();
    const std::uint32_t historyBefore = a->historyCount;
    const std::uint64_t numberBefore = profiler->frameNumber();
    profiler->setEnabled(false);
    CHECK(!profiler->enabled());
    profiler->beginFrame();
    CHECK(!profiler->begin("Z"));
    CHECK(!profiler->begin("A"));
    CHECK(!profiler->beginGpu("Frame"));
    profiler->addSample("W", 1.0f);
    profiler->endFrame();
    CHECK(profiler->cpuSampleCount() == countBefore);
    CHECK(a->historyCount == historyBefore);
    CHECK(profiler->frameNumber() == numberBefore + 1);
    profiler->setEnabled(true);
    cpuFrames = numberBefore + 1;

    profiler->beginFrame();
    CHECK(profiler->begin("Z"));
    profiler->end();
    profiler->endFrame();
    ++cpuFrames;
    CHECK(profiler->cpuSampleCount() == countBefore + 1);
    CHECK(profiler->frameNumber() == cpuFrames);

    {
        ProfileScope scope("Scoped");
        spin(1.0);
    }
    CHECK(cpuSample("Scoped") == nullptr || cpuSample("Scoped")->milliseconds >= 1.0f);
}

void testPasses(SceneTree& tree)
{
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;

    const std::uint32_t registered = profiler->passCount();
    CHECK(registered == 8);
    CHECK(profiler->passName(0) != nullptr && strcmp(profiler->passName(0), "Opaque") == 0);
    CHECK(profiler->passName(1) != nullptr && strcmp(profiler->passName(1), "Sky") == 0);
    CHECK(profiler->passName(2) != nullptr && strcmp(profiler->passName(2), "Final") == 0);
    CHECK(profiler->passName(3) != nullptr && strcmp(profiler->passName(3), "Skinning") == 0);
    CHECK(profiler->passName(4) != nullptr && strcmp(profiler->passName(4), "Radiance") == 0);
    CHECK(profiler->passName(5) != nullptr && strcmp(profiler->passName(5), "Shadows") == 0);
    CHECK(profiler->passName(6) != nullptr && strcmp(profiler->passName(6), "Depth") == 0);
    CHECK(profiler->passName(7) != nullptr && strcmp(profiler->passName(7), "Impostor") == 0);
    CHECK(profiler->passEnabled("Impostor"));
    CHECK(profiler->passEnabled("Depth"));
    CHECK(profiler->passEnabled("Radiance"));
    CHECK(profiler->passEnabled("Shadows"));
    CHECK(profiler->passEnabled("Opaque"));
    CHECK(profiler->passEnabled("Sky"));
    CHECK(profiler->passEnabled("Final"));
    CHECK(profiler->passEnabled("Skinning"));
    CHECK(profiler->passEnabled("Unknown"));
    CHECK(profiler->registerPass("Opaque"));
    CHECK(profiler->passCount() == registered);

    static char names[RenderProfiler::kMaxPasses][8];
    for (std::uint32_t i = registered; i < RenderProfiler::kMaxPasses; ++i)
    {
        snprintf(names[i], sizeof(names[i]), "P%u", i);
        CHECK(profiler->registerPass(names[i]));
        CHECK(profiler->passCount() == i + 1);
        CHECK(profiler->passEnabled(names[i]));
    }
    CHECK(profiler->passCount() == RenderProfiler::kMaxPasses);
    CHECK(!profiler->registerPass("Overflow"));
    CHECK(profiler->registerPass(names[registered]));
    CHECK(profiler->registerPass("Opaque"));
    CHECK(profiler->passCount() == RenderProfiler::kMaxPasses);
    CHECK(profiler->passName(RenderProfiler::kMaxPasses) == nullptr);
    CHECK(strcmp(profiler->passName(registered), names[registered]) == 0);

    profiler->setPassEnabled(names[registered], false);
    CHECK(!profiler->passEnabled(names[registered]));
    CHECK(profiler->passEnabled(names[registered + 1]));
    profiler->setPassEnabled(names[registered], true);
    CHECK(profiler->passEnabled(names[registered]));
    profiler->setPassEnabled("Missing", false);
    CHECK(profiler->passEnabled("Missing"));
    CHECK(profiler->passCount() == RenderProfiler::kMaxPasses);

    profiler->setPassEnabled("Opaque", false);
    CHECK(!profiler->passEnabled("Opaque"));
    {
        Frame frame(tree);
        CHECK(frame.stats().viewportsDrawn == 1);
        CHECK(frame.stats().drawCalls == 0);
        CHECK(frame.stats().surfacesDrawn == 0);
        CHECK(frame.stats().triangles == 0);
        CHECK(pixelIs(kWidth / 2, kHeight / 2, kClear, "disabled pass, centre"));
        CHECK(pixelIs(2, 2, kClear, "disabled pass, corner"));
    }

    profiler->setPassEnabled("Opaque", true);
    {
        Frame frame(tree);
        CHECK(frame.stats().viewportsDrawn == 1);
        CHECK(frame.stats().drawCalls > 0);
        CHECK(frame.stats().triangles == 12);
        CHECK(frame.stats().depthPrePassDrawCalls > 0);
        CHECK(frame.stats().depthPrePassTriangles == 12);
        CHECK(pixelIs(kWidth / 2, kHeight / 2, kRed, "enabled pass, centre"));
        CHECK(pixelIs(2, 2, kClear, "enabled pass, corner"));
    }

    profiler->setPassEnabled("Depth", false);
    CHECK(!profiler->passEnabled("Depth"));
    {
        Frame frame(tree);
        CHECK(frame.stats().drawCalls > 0);
        CHECK(frame.stats().triangles == 12);
        CHECK(frame.stats().depthPrePassDrawCalls == 0);
        CHECK(frame.stats().depthPrePassTriangles == 0);
        CHECK(pixelIs(kWidth / 2, kHeight / 2, kRed, "disabled depth pass, centre"));
        CHECK(pixelIs(2, 2, kClear, "disabled depth pass, corner"));
    }
    profiler->setPassEnabled("Depth", true);
    {
        Frame frame(tree);
        CHECK(frame.stats().depthPrePassDrawCalls > 0);
        CHECK(pixelIs(kWidth / 2, kHeight / 2, kRed, "enabled depth pass, centre"));
    }

    profiler->setPassEnabled("Final", false);
    {
        Frame frame(tree);
        CHECK(frame.stats().drawCalls > 0);
        CHECK(pixelIs(kWidth / 2, kHeight / 2, kBlack, "disabled final, centre"));
        CHECK(pixelIs(2, 2, kBlack, "disabled final, corner"));
    }
    profiler->setPassEnabled("Final", true);
    {
        Frame frame(tree);
        CHECK(pixelIs(kWidth / 2, kHeight / 2, kRed, "enabled final, centre"));
        CHECK(pixelIs(2, 2, kClear, "enabled final, corner"));
    }
}

void testPanelHandDriven(SceneTree& tree)
{
    Gui* gui = Main::gui();
    RenderProfiler* profiler = Main::profiler();
    CHECK(gui != nullptr && profiler != nullptr);
    if (!gui || !profiler) return;
    CHECK(profiler->gpuSampleCount() == 0);

    ProfilerPanel panel;
    CHECK(panel.open);
    for (int i = 0; i < 2; ++i)
    {
        Main::device()->processEvents();
        gui->beginFrame(1.0f / 60.0f);
        CHECK(gui->frameOpen());
        panel.draw(gui->context(), *profiler, Main::rendering()->getStats());
        tree.process(1.0 / 60.0);
        Main::rendering()->renderViewports();
        gui->endFrame(true);
        CHECK(!gui->frameOpen());
        CHECK(gui->backend().drawCalls() > 0);
        Main::rendering()->endFrame(true);
    }

    panel.open = false;
    Main::device()->processEvents();
    gui->beginFrame(1.0f / 60.0f);
    panel.draw(gui->context(), *profiler, Main::rendering()->getStats());
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    gui->endFrame(true);
    CHECK(gui->backend().drawCalls() == 0);
    Main::rendering()->endFrame(true);

    panel.open = true;
    panel.showCpu = false;
    panel.showGpu = false;
    panel.showHistory = false;
    panel.showPasses = false;
    Main::device()->processEvents();
    gui->beginFrame(1.0f / 60.0f);
    panel.draw(gui->context(), *profiler, Main::rendering()->getStats());
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    gui->endFrame(true);
    CHECK(gui->backend().drawCalls() > 0);
    Main::rendering()->endFrame(true);
}

void testGpu(SceneTree& tree, PanelNode* panelNode)
{
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;

    const bool available = profiler->gpuAvailable();
    CHECK(available == Main::device()->driver()->caps().timerQueries);
    printf("gpu timer queries available: %s\n", available ? "yes" : "no");

    (void) tree;
    const std::uint64_t drawnBefore = Main::engine()->getFramesDrawn();
    for (int i = 0; i < 8; ++i) Main::iteration();
    CHECK(Main::engine()->getFramesDrawn() >= drawnBefore + 8);
    CHECK(panelNode->frames >= 8);

    for (int i = 0; i < 60; ++i)
    {
        const ProfileSample* frame = gpuSample("Frame");
        const ProfileSample* viewport = gpuSample("Opaque");
        if (!available ||
                (frame && viewport && frame->historyCount >= 2 && viewport->historyCount >= 2))
            break;
        time_sleep(2);
        Main::iteration();
    }

    CHECK(!profiler->beginGpu("Outside"));

    if (!available)
    {
        CHECK(profiler->gpuSampleCount() == 0);
        CHECK(!profiler->beginGpu("Frame"));
        CHECK(profiler->gpuFrameMilliseconds() == 0.0f);
        printf("gpu part skipped: timer queries unavailable on this driver\n");
        return;
    }

    CHECK(profiler->gpuSampleCount() >= 2);
    const ProfileSample* frame = gpuSample("Frame");
    const ProfileSample* viewport = gpuSample("Opaque");
    CHECK(frame != nullptr && viewport != nullptr);
    CHECK(gpuSample("Sky") != nullptr && gpuSample("Final") != nullptr);
    CHECK(gpuSample("Shadows") != nullptr);
    CHECK(gpuSample("Depth") != nullptr);
    CHECK(Main::rendering()->getStats().shadowLights == 2);
    CHECK(Main::rendering()->getStats().shadowPasses == 7);
    for (std::uint32_t i = 0; i < profiler->gpuSampleCount(); ++i)
    {
        const ProfileSample& sample = profiler->gpuSamples()[i];
        CHECK(sample.milliseconds >= 0.0f);
        CHECK(sample.average >= 0.0f);
        CHECK(sample.maximum >= sample.average);
        CHECK(sample.display >= 0.0f);
        printf("gpu %-10s ms %.4f avg %.4f max %.4f history %u\n", sample.name, sample.milliseconds,
                sample.average, sample.maximum, sample.historyCount);
    }
    CHECK(profiler->gpuFrameMilliseconds() >= 0.0f);
    CHECK(frame == nullptr || frame->maximum > 0.0f);
    printf("gpu frame %.4f ms\n", profiler->gpuFrameMilliseconds());
    if (frame && viewport)
    {
        CHECK(frame->historyCount >= 1);
        CHECK(viewport->historyCount >= 1);
        CHECK(viewport->depth > frame->depth);
    }
}

void testOverflow()
{
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;

    static char names[RenderProfiler::kMaxSamples + 4][8];
    profiler->beginFrame();
    for (std::uint32_t i = 0; i < RenderProfiler::kMaxSamples + 4; ++i)
    {
        snprintf(names[i], sizeof(names[i]), "S%u", i);
        const bool expected = profiler->cpuSampleCount() < RenderProfiler::kMaxSamples;
        const bool opened = profiler->begin(names[i]);
        CHECK(opened == expected);
        if (opened) profiler->end();
    }
    CHECK(profiler->cpuSampleCount() == RenderProfiler::kMaxSamples);
    CHECK(!profiler->begin("Another"));
    CHECK(profiler->begin("A"));
    profiler->end();
    profiler->endFrame();
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "profiler test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::profiler() != nullptr);
    CHECK(Main::gui() != nullptr);
    if (!Main::profiler() || !Main::gui()) return 1;

    // The profiler starts off and runs on request; these tests read it every frame.
    Main::profiler()->setEnabled(true);
    testCpu();

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

    Node3D holder;
    Camera3D* camera = holder.createChild<Camera3D>();
    camera->setPosition(Math::Vec3(0.0f, 0.0f, 3.0f));
    camera->reparent(root);
    camera->makeCurrent();

    MeshInstance3D* box = holder.createChild<MeshInstance3D>();
    box->setMesh(createBoxMesh(*Main::meshes(), Math::Vec3(1.0f, 1.0f, 1.0f)));
    box->setColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    box->reparent(root);

    testPasses(tree);
    testPanelHandDriven(tree);

    MeshInstance3D* floorBox = holder.createChild<MeshInstance3D>();
    floorBox->setMesh(createBoxMesh(*Main::meshes(), Math::Vec3(8.0f, 8.0f, 0.2f)));
    floorBox->setPosition(Math::Vec3(0.0f, 0.0f, -1.0f));
    floorBox->reparent(root);
    SpotLight3D* lamp = holder.createChild<SpotLight3D>();
    lamp->setRange(10.0f);
    lamp->setShadow(true);
    lamp->lookAtFromPosition(Math::Vec3(-2.5f, 0.0f, 1.5f), Math::Vec3(2.5f, 0.0f, -0.9f));
    lamp->reparent(root);
    OmniLight3D* bulb = holder.createChild<OmniLight3D>();
    bulb->setRange(10.0f);
    bulb->setShadow(true);
    bulb->setPosition(Math::Vec3(2.5f, 0.0f, 1.5f));
    bulb->reparent(root);
    window->setPositionalShadowAtlasSize(1024, false);

    PanelNode* panelNode = root->createChild<PanelNode>();
    testGpu(tree, panelNode);
    testOverflow();

    tree.finalize();
    Main::cleanup();
    CHECK(Main::profiler() == nullptr);
    CHECK(RenderProfiler::singleton() == nullptr);

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("profiler ok\n");
    return 0;
}
