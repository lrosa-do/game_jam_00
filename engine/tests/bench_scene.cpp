#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

int main(int argc, char** argv)
{
    const int count = argc > 1 ? atoi(argv[1]) : 20000;
    const bool vulkan = argc > 2 && strcmp(argv[2], "vulkan") == 0;
    const int frames = 120;

    DeviceDesc desc;
    desc.title = "bench";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = 64;
    desc.height = 48;
    desc.headless = true;
    if (!Main::setup(desc)) return 1;

    SceneTree tree;
    Main::start(&tree);
    Node* root = tree.getRoot();
    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPosition(Math::Vec3(0.0f, 0.0f, 400.0f));
    camera->makeCurrent();

    const MeshId mesh = createBoxMesh(*Main::meshes(), Math::Vec3(0.5f, 0.5f, 0.5f));
    const int columns = 200;
    for (int i = 0; i < count; ++i)
    {
        MeshInstance3D* node = root->createChild<MeshInstance3D>();
        node->setMesh(mesh);
        node->setPosition(Math::Vec3(static_cast<float>(i % columns) - 100.0f,
                static_cast<float>(i / columns) - 50.0f, 0.0f));
    }

    RenderProfiler* profiler = Main::profiler();
    double prepare = 0.0;
    double wall = 0.0;
    int measured = 0;
    for (int f = 0; f < frames; ++f)
    {
        const std::uint64_t start = time_nanos();
        Main::iteration();
        const std::uint64_t end = time_nanos();
        if (f >= 20)
        {
            wall += static_cast<double>(end - start) * 1.0e-6;
            for (std::uint32_t i = 0; i < profiler->cpuSampleCount(); ++i)
                if (strcmp(profiler->cpuSamples()[i].name, "Prepare") == 0)
                {
                    prepare += profiler->cpuSamples()[i].milliseconds;
                    break;
                }
            ++measured;
        }
    }
    const RenderStats& stats = Main::rendering()->getStats();
    printf("instances %d drawn %u drawCalls %u prepare %.3f ms frame %.3f ms\n", count,
            stats.instancesDrawn, stats.drawCalls, prepare / measured, wall / measured);
    tree.finalize();
    Main::cleanup();
    return 0;
}
