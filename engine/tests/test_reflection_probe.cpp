#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/ReflectionProbe3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 32;
int gErrors = 0;

void onLog(LogLevel level, const char* message, void*)
{
    if (level < LOGLEVEL_ERROR) return;
    ++gErrors;
    printf("logged error: %s\n", message);
}

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

RenderStats frame(SceneTree& tree)
{
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    const RenderStats stats = Main::rendering()->getStats();
    Main::rendering()->endFrame(true);
    return stats;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    log_set_callback(onLog, nullptr);
    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "reflection probe test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kSize;
    desc.height = kSize;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;

    WorldEnvironment* environment = root->createChild<WorldEnvironment>();
    environment->setEnvironment(whiteAmbient());
    Camera3D* camera = root->createChild<Camera3D>();
    camera->makeCurrent();
    camera->setPosition(Math::Vec3(0.0f, 0.0f, 10.0f));

    RenderingServer* server = Main::rendering();

    CHECK(frame(tree).reflectionProbesDrawn == 0);

    ReflectionProbe3D* probe = root->createChild<ReflectionProbe3D>();
    probe->setSize(Math::Vec3(8.0f, 4.0f, 8.0f));
    probe->setIntensity(0.5f);
    probe->setBlendDistance(2.0f);
    probe->setInterior(true);
    probe->setBoxProjection(true);
    probe->setOriginOffset(Math::Vec3(0.0f, 1.0f, 0.0f));
    probe->setReflectionMask(0x6u);
    probe->setPosition(Math::Vec3(3.0f, 0.0f, 0.0f));

    RenderStats stats = frame(tree);
    printf("  probes drawn %u\n", stats.reflectionProbesDrawn);
    CHECK(stats.reflectionProbesDrawn == 1);
    CHECK(server->getReflectionProbeFrameCount() == 1);
    const ReflectionDataGpu* data = server->getReflectionProbeFrameData();
    CHECK(near(data[0].boxExtents[0], 4.0f));
    CHECK(near(data[0].boxExtents[1], 2.0f));
    CHECK(near(data[0].boxOffset[1], 1.0f));
    CHECK(near(data[0].intensity, 0.5f));
    CHECK(near(data[0].blendDistance, 2.0f));
    CHECK(data[0].exterior == 0);
    CHECK(data[0].boxProject == 1);
    CHECK(data[0].mask == 0x6u);
    CHECK(near(data[0].index, -1.0f));
    CHECK(near(data[0].localMatrix[12], -3.0f));
    CHECK(near(data[0].localMatrix[14], 10.0f));

    ReflectionProbe3D* big = root->createChild<ReflectionProbe3D>();
    big->setSize(Math::Vec3(100.0f, 100.0f, 100.0f));
    stats = frame(tree);
    CHECK(stats.reflectionProbesDrawn == 2);
    data = server->getReflectionProbeFrameData();
    CHECK(near(data[0].boxExtents[0], 50.0f));
    CHECK(near(data[1].boxExtents[0], 4.0f));

    probe->setVisible(false);
    stats = frame(tree);
    CHECK(stats.reflectionProbesDrawn == 1);

    probe->setVisible(true);
    big->queueFree();
    stats = frame(tree);
    CHECK(stats.reflectionProbesDrawn == 1);
    CHECK(near(server->getReflectionProbeFrameData()[0].boxExtents[0], 4.0f));

    const Math::Box box = probe->getAabb();
    CHECK(near(box.min.x, -4.0f) && near(box.max.y, 2.0f));

    CHECK(gErrors == 0);
    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures == 0) printf("all checks passed\n");
    return failures == 0 ? 0 : 1;
}
