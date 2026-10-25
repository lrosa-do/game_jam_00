#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/InputMap.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

ct::String projectParent(bool vulkan)
{
    ct::String parent = PRISMA3D_TEST_DIR;
    parent += vulkan ? "/project_gpu_test_vulkan" : "/project_gpu_test";
    return parent;
}

bool makeProject(bool vulkan, ct::String& manifest)
{
    Project project;
    if (!project.create(projectParent(vulkan), "Game")) return false;

    project.set("display/window/size/viewport_width", PropertyValue::fromInt(320));
    project.set("display/window/size/viewport_height", PropertyValue::fromInt(200));
    project.set("display/window/vsync/vsync_mode", PropertyValue::fromInt(0));
    project.set("physics/common/physics_ticks_per_second", PropertyValue::fromInt(90));
    project.set("physics/common/max_physics_steps_per_frame", PropertyValue::fromInt(5));
    project.set("physics/3d/default_gravity", PropertyValue::fromFloat(4.5));
    project.set("application/run/max_fps", PropertyValue::fromInt(75));
    project.set("rendering/anti_aliasing/quality/msaa_3d", PropertyValue::fromInt(2));
    project.set("rendering/scaling_3d/scale", PropertyValue::fromFloat(0.5));
    project.set("rendering/lights_and_shadows/directional_shadow/size",
            PropertyValue::fromInt(2048));
    project.set("rendering/lights_and_shadows/directional_shadow/soft_shadow_filter_quality",
            PropertyValue::fromInt(4));
    project.set("rendering/lights_and_shadows/positional_shadow/soft_shadow_filter_quality",
            PropertyValue::fromInt(1));
    project.set("rendering/lights_and_shadows/positional_shadow/atlas_size",
            PropertyValue::fromInt(1024));
    project.set("rendering/anti_aliasing/screen_space_roughness_limiter/enabled",
            PropertyValue::fromBool(false));
    project.set("rendering/anti_aliasing/screen_space_roughness_limiter/amount",
            PropertyValue::fromFloat(0.5));
    CHECK(project.set("rendering/environment/ssao/quality", PropertyValue::fromInt(3)));
    CHECK(project.set("rendering/environment/ssao/half_size", PropertyValue::fromBool(false)));
    CHECK(project.set("rendering/environment/ssao/adaptive_target",
            PropertyValue::fromFloat(0.25)));
    CHECK(project.set("rendering/environment/ssao/blur_passes", PropertyValue::fromInt(3)));
    CHECK(project.set("rendering/environment/ssao/fadeout_from", PropertyValue::fromFloat(20.0)));
    CHECK(project.set("rendering/environment/ssao/fadeout_to", PropertyValue::fromFloat(120.0)));
    CHECK(project.set("rendering/environment/screen_space_reflection/half_size",
            PropertyValue::fromBool(false)));
    CHECK(project.set("rendering/camera/depth_of_field/depth_of_field_bokeh_shape",
            PropertyValue::fromInt(2)));
    CHECK(project.set("rendering/camera/depth_of_field/depth_of_field_bokeh_quality",
            PropertyValue::fromInt(3)));
    CHECK(project.set("rendering/camera/depth_of_field/depth_of_field_use_jitter",
            PropertyValue::fromBool(true)));
    CHECK(project.set("rendering/lightmapping/lightmap_gi/use_bicubic_filter",
            PropertyValue::fromBool(false)));
    project.set("rendering/environment/defaults/default_clear_color",
            PropertyValue::fromColor(Math::Vec4(0.5f, 0.25f, 0.125f, 1.0f)));

    ProjectInputAction jump;
    jump.name = "jump";
    jump.deadzone = 0.6f;
    InputEvent space;
    space.type = InputEventType::Key;
    space.key.key = Key::Space;
    space.key.scancode = static_cast<int>(Key::Space);
    jump.events.push_back(space);
    project.setInputAction(jump);

    Node3D world;
    world.setName("world");
    Camera3D* camera = world.createChild<Camera3D>();
    camera->setName("camera");
    camera->setPosition(Math::Vec3(0.0f, 1.0f, 5.0f));
    DirectionalLight3D* sun = world.createChild<DirectionalLight3D>();
    sun->setName("sun");
    sun->setShadow(true);
    SceneResult saved;
    if (!SceneSerializer::saveFile(&world, project.absolutePath("Scenes/main.scene.json"), &saved))
        return false;
    if (!project.addScene("Scenes/main.scene.json") ||
            !project.setMainScene("Scenes/main.scene.json"))
        return false;
    if (!FileSystem::writeText(project.assetPath() + "/Textures/marker.txt", "marker"))
        return false;
    if (!project.save()) return false;
    manifest = project.manifestPath();
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    removeTree(projectParent(vulkan));
    ct::String manifest;
    CHECK(makeProject(vulkan, manifest));
    if (manifest.empty()) return 1;

    Project closedProject;
    DeviceDesc base;
    base.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    base.headless = true;
    CHECK(!Main::setupProject(closedProject, base));
    CHECK(Main::device() == nullptr);

    Project project;
    CHECK(project.open(manifest));
    FileSystem& files = FileSystem::getSingleton();
    CHECK(!files.exists("Textures/marker.txt"));
    const size_t searchPaths = files.getSearchPaths().size();

    CHECK(Main::setupProject(project, base));
    if (!Main::device()) return 1;
    CHECK(Main::project() == &project);
    CHECK(!Main::setupProject(project, base));

    int width = 0;
    int height = 0;
    Main::device()->windowGetSize(&width, &height);
    CHECK(width == 320 && height == 200);
    CHECK(Main::device()->windowGetVSyncMode() == VSyncMode::Disabled);

    CHECK(Main::engine()->getPhysicsTicksPerSecond() == 90);
    CHECK(Main::engine()->getMaxPhysicsStepsPerFrame() == 5);
    CHECK(Main::engine()->getMaxFps() == 75);
    CHECK(Main::engine()->getPhysicsJitterFix() == 0.5);

    CHECK(fabsf(Main::physics()->getGravity().y + 4.5f) < 1e-5f);
    CHECK(Main::physics()->getGravity().x == 0.0f);

    InputMap& map = Main::device()->input().map();
    CHECK(map.hasAction("jump"));
    CHECK(map.actionGetDeadzone("jump") == 0.6f);
    const ct::Vector<InputEvent>* events = map.actionGetEvents("jump");
    CHECK(events != nullptr && events->size() == 1);
    if (events && events->size() == 1) CHECK((*events)[0].key.key == Key::Space);

    RenderingServer& server = *Main::rendering();
    CHECK(server.directionalShadowSize() == 2048);
    CHECK(server.directionalShadowQuality() == ShadowQuality::SoftHigh);
    CHECK(server.positionalShadowQuality() == ShadowQuality::SoftVeryLow);
    CHECK(!server.roughnessLimiterEnabled());
    CHECK(server.roughnessLimiterAmount() == 0.5f);
    CHECK(server.roughnessLimiterLimit() == 0.18f);
    CHECK(server.ssaoQuality() == SsaoQuality::High);
    CHECK(!server.ssaoHalfSize());
    CHECK(server.ssaoAdaptiveTarget() == 0.25f);
    CHECK(server.ssaoBlurPasses() == 3);
    CHECK(server.ssaoFadeoutFrom() == 20.0f);
    CHECK(server.ssaoFadeoutTo() == 120.0f);
    CHECK(!server.ssrHalfSize());
    CHECK(server.depthOfFieldShape() == DofBokehShape::Circle);
    CHECK(server.depthOfFieldQuality() == DofBlurQuality::High);
    CHECK(server.depthOfFieldJitter());
    CHECK(!server.getLightmapBicubicFilter());

    CHECK(files.getSearchPaths().size() == searchPaths + 2);
    CHECK(files.exists("Textures/marker.txt"));

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;
    Window* window = root->castTo<Window>();
    CHECK(window != nullptr);
    if (window)
    {
        CHECK(window->getMsaa3D() == ViewportMsaa::Msaa4x);
        CHECK(server.viewportGetMsaa3D(window->getViewportId()) == ViewportMsaa::Msaa4x);
        CHECK(window->getScaling3D() == 0.5f);
        CHECK(server.viewportGetScaling3D(window->getViewportId()) == 0.5f);
        CHECK(window->getPositionalShadowAtlasSize() == 1024);
        CHECK(window->getClearColor().x == 0.5f && window->getClearColor().y == 0.25f &&
                window->getClearColor().z == 0.125f);
    }

    const ViewportId raw = server.viewportCreate();
    CHECK(raw.valid());
    project.applyToViewport(server, raw, false);
    CHECK(server.viewportGetMsaa3D(raw) == ViewportMsaa::Msaa4x);
    CHECK(server.viewportGetScaling3D(raw) == 0.5f);
    CHECK(server.viewportGetMeshLodThreshold(raw) == 1.0f);
    Project defaults;
    defaults.applyToViewport(server, raw);
    CHECK(server.viewportGetMsaa3D(raw) == ViewportMsaa::Disabled);
    CHECK(server.viewportGetScaling3D(raw) == 1.0f);
    server.viewportFree(raw);

    const size_t before = root->getChildCount();
    SceneResult result;
    Node* scene = Main::loadMainScene(result);
    CHECK(result.diagnostics.size() == 0);
    CHECK(scene != nullptr);
    CHECK(root->getChildCount() == before + 1);
    if (scene)
    {
        CHECK(scene->getParent() == root);
        CHECK(scene->getName() == "world");
        CHECK(scene->getChildCount() == 2);
        CHECK(scene->isInsideTree());
        Camera3D* camera =
                scene->getChildCount() > 0 ? scene->getChild(0)->castTo<Camera3D>() : nullptr;
        CHECK(camera != nullptr && camera->getPosition().z == 5.0f);
    }

    for (int i = 0; i < 3; ++i)
    {
        tree.process(1.0 / 60.0);
        server.renderViewports();
        server.endFrame(true);
    }

    if (window) CHECK(window->getMsaaSamples() == 4);

    tree.finalize();
    Main::cleanup();
    CHECK(Main::project() == nullptr);
    CHECK(files.getSearchPaths().size() == searchPaths);
    CHECK(!files.exists("Textures/marker.txt"));

    SceneResult after;
    CHECK(Main::loadMainScene(after) == nullptr);
    CHECK(after.errorCount() == 1);

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("project gpu ok\n");
    return 0;
}
