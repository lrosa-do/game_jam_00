#include "prisma3d/Device.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/gui/ProfilerPanel.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const int kWarmup = 30;
const int kMeasured = 120;

class PanelNode : public Node
{
public:
    PanelNode()
    {
        setProcess(true);
        for (int i = 0; i < 40; ++i) values[i] = 0.5f;
        for (int i = 0; i < 12; ++i) flags[i] = (i & 1) != 0;
    }

    ProfilerPanel profiler;
    float values[40];
    bool flags[12];

protected:
    void notificationHandler(int what) override
    {
        Node::notificationHandler(what);
        if (what != kNotificationProcess || !Main::gui() || !Main::profiler()) return;
        ig::Context& ui = Main::gui()->context();
        profiler.draw(ui, *Main::profiler(), *Main::rendering());
        if (ui.beginWindow("Controls", ig::Rect(10.0f, 10.0f, 300.0f, 560.0f)))
        {
            char text[48];
            for (int i = 0; i < 40; ++i)
            {
                snprintf(text, sizeof(text), "Value %d", i);
                ui.sliderFloat(text, values[i], 0.0f, 1.0f);
            }
            for (int i = 0; i < 12; ++i)
            {
                snprintf(text, sizeof(text), "Option %d", i);
                ui.checkbox(text, flags[i]);
            }
        }
        ui.endWindow();
    }
};

const ProfileSample* sample(const ProfileSample* samples, std::uint32_t count, const char* name)
{
    for (std::uint32_t i = 0; i < count; ++i)
        if (samples[i].name && strcmp(samples[i].name, name) == 0) return &samples[i];
    return nullptr;
}

struct Result
{
    double wall = 0.0;
    double cpuFrame = 0.0;
    double cpuGui = 0.0;
    double gpuFrame = 0.0;
    double gpuGui = 0.0;
    int frames = 0;
    int gpuFrames = 0;
};

Result measure(bool guiOn)
{
    Main::gui()->setEnabled(guiOn);
    RenderProfiler* profiler = Main::profiler();
    Result result;
    for (int f = 0; f < kWarmup + kMeasured; ++f)
    {
        const std::uint64_t start = time_nanos();
        Main::iteration();
        const std::uint64_t end = time_nanos();
        if (f < kWarmup) continue;
        result.wall += static_cast<double>(end - start) * 1.0e-6;
        result.cpuFrame += static_cast<double>(profiler->cpuFrameMilliseconds());
        const ProfileSample* gui =
                sample(profiler->cpuSamples(), profiler->cpuSampleCount(), "GUI");
        if (gui) result.cpuGui += static_cast<double>(gui->milliseconds);
        ++result.frames;
        const ProfileSample* gpuFrame =
                sample(profiler->gpuSamples(), profiler->gpuSampleCount(), "Frame");
        if (gpuFrame)
        {
            result.gpuFrame += static_cast<double>(gpuFrame->milliseconds);
            const ProfileSample* gpuGui =
                    sample(profiler->gpuSamples(), profiler->gpuSampleCount(), "GUI");
            if (gpuGui) result.gpuGui += static_cast<double>(gpuGui->milliseconds);
            ++result.gpuFrames;
        }
    }
    return result;
}

void print(const char* name, const Result& r)
{
    const double n = r.frames > 0 ? r.frames : 1;
    const double g = r.gpuFrames > 0 ? r.gpuFrames : 1;
    printf("%-8s wall %7.3f ms (%6.1f fps)  cpu frame %7.3f  cpu GUI %6.3f  gpu frame %7.3f  "
           "gpu GUI %6.3f\n",
            name, r.wall / n, 1000.0 / (r.wall / n), r.cpuFrame / n, r.cpuGui / n, r.gpuFrame / g,
            r.gpuGui / g);
}

} // namespace

int main(int argc, char** argv)
{
    const int count = argc > 1 ? atoi(argv[1]) : 2000;
    const bool vulkan = argc > 2 && strcmp(argv[2], "vulkan") == 0;
    const int rounds = argc > 3 ? atoi(argv[3]) : 3;

    DeviceDesc desc;
    desc.title = "gui bench";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;
    if (!Main::setup(desc) || !Main::gui()) return 1;

    SceneTree tree;
    Main::start(&tree);
    Node* root = tree.getRoot();

    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    sun->setShadow(true);
    sun->lookAtFromPosition(Math::Vec3(20.0f, 40.0f, 10.0f), Math::Vec3(0.0f, 0.0f, 0.0f));

    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPosition(Math::Vec3(0.0f, 30.0f, 90.0f));
    camera->lookAt(Math::Vec3(0.0f, 0.0f, 0.0f));
    camera->makeCurrent();

    MeshInstance3D* ground = root->createChild<MeshInstance3D>();
    ground->setMesh(createPlaneMesh(*Main::meshes(), 400.0f, 400.0f));

    const MeshId box = createBoxMesh(*Main::meshes(), Math::Vec3(1.0f, 1.0f, 1.0f));
    const int columns = 60;
    for (int i = 0; i < count; ++i)
    {
        MeshInstance3D* node = root->createChild<MeshInstance3D>();
        node->setMesh(box);
        node->setPosition(Math::Vec3(static_cast<float>(i % columns) * 2.5f - 75.0f, 0.5f,
                static_cast<float>(i / columns) * 2.5f - 75.0f));
    }

    root->createChild<PanelNode>();

    printf("boxes %d, %dx%d, %s, gpu timers %s\n", count, kWidth, kHeight,
            vulkan ? "vulkan" : "opengl", Main::profiler()->gpuAvailable() ? "yes" : "no");
    for (int round = 0; round < rounds; ++round)
    {
        const Result off = measure(false);
        const Result on = measure(true);
        print("gui off", off);
        print("gui on", on);
    }
    tree.finalize();
    Main::cleanup();
    return 0;
}
