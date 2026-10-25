#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/LightmapBakePass.h"
#include "prisma3d/editor/LightmapBaker.h"
#include "prisma3d/editor/LightmapMeshIO.h"
#include "prisma3d/editor/LightmapPanel.h"
#include "prisma3d/editor/LightmapUnwrapJob.h"
#include "prisma3d/editor/LightmapUnwrapper.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <igui/widgets/WidgetApp.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const int kShot = 200;

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_lightmap_test_vulkan" : "/editor_lightmap_test";
    return root;
}

InputEvent motion(int x, int y)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = 0;
    event.mouseMotion.relativeY = 0;
    event.mouseMotion.buttonMask = 0;
    return event;
}

InputEvent button(int x, int y, MouseButton which, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = which;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = 0;
    return event;
}

struct Rig
{
    SceneTree& tree;
    WidgetHost& host;
    EditorShell& shell;
    EditorSelection& selection;
    SceneTreePanel& hierarchy;
    InspectorPanel& inspector;
    EditorDocument& document;
    ProjectUi& ui;
    EditorViewportPanel& panel;
    HierarchyActions& actions;
    EditorMenus& menus;
    EditorStatusBar& status;
    LightmapBaker& baker;
    LightmapPanel& lightmap;
    bool stepBaker = true;

    bool send(const InputEvent& event)
    {
        if (ui.handleEvent(event)) return true;
        if (actions.handleEvent(event)) return true;
        host.handleEvent(event);
        panel.handleEvent(event);
        return false;
    }

    void frame(float delta = 1.0f / 60.0f)
    {
        tree.process(delta);
        panel.update(delta);
        Main::rendering()->renderViewports();
        selection.update();
        selection.flush();
        hierarchy.update();
        inspector.update();
        ui.update();
        actions.update();
        menus.update();
        status.update(delta);
        if (stepBaker) lightmap.update();
        host.frame(delta, true);
        Main::rendering()->endFrame(true);
    }

    void frames(int count)
    {
        for (int i = 0; i < count; ++i) frame();
    }

    void click(int x, int y)
    {
        send(motion(x, y));
        frames(2);
        send(button(x, y, MouseButton::Left, true));
        frames(2);
        send(button(x, y, MouseButton::Left, false));
        frames(3);
    }

    bool waitIdle(int limit = 4000)
    {
        for (int i = 0; i < limit; ++i)
        {
            frame();
            if (!baker.busy()) return true;
        }
        return false;
    }
};

struct Image
{
    int width = 0;
    int height = 0;
    ct::Vector<std::uint8_t> pixels;

    int channel(int x, int y, int c) const
    {
        return pixels[(static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                              static_cast<std::size_t>(x)) *
                              4 +
                      static_cast<std::size_t>(c)];
    }
};

float linearToSrgb(float value)
{
    if (value <= 0.0f) return 0.0f;
    if (value >= 1.0f) return 1.0f;
    return value <= 0.0031308f ? value * 12.92f : 1.055f * powf(value, 1.0f / 2.4f) - 0.055f;
}

float srgbToLinear(float value)
{
    return value <= 0.04045f ? value / 12.92f : powf((value + 0.055f) / 1.055f, 2.4f);
}

int encode(float linear)
{
    return static_cast<int>(floorf(linearToSrgb(linear) * 255.0f + 0.5f));
}

bool exists(const ct::String& path)
{
    return file_exists(path.c_str()) || dir_exists(path.c_str());
}

void centerOf(const rt::Rect& rect, int& x, int& y)
{
    x = static_cast<int>(rect.x + rect.w * 0.5f);
    y = static_cast<int>(rect.y + rect.h * 0.5f);
}

void scrollTo(Rig& rig, const rt::Widget* widget)
{
    rt::ScrollView* scroll = rig.lightmap.scrollView();
    CHECK(scroll != nullptr && widget != nullptr);
    if (!scroll || !widget) return;
    for (int pass = 0; pass < 4; ++pass)
    {
        const rt::Rect view = scroll->absoluteRect();
        const rt::Rect target = widget->absoluteRect();
        const float delta = target.y - (view.y + view.h * 0.5f);
        if (fabsf(delta) < 4.0f) break;
        scroll->setScrollY(scroll->scrollY() + delta);
        rig.frames(2);
    }
}

void clickWidget(Rig& rig, const rt::Widget* widget)
{
    CHECK(widget != nullptr);
    if (!widget) return;
    scrollTo(rig, widget);
    int x = 0;
    int y = 0;
    centerOf(widget->absoluteRect(), x, y);
    rig.click(x, y);
}

void showPanel(Rig& rig)
{
    rig.shell.showPanel(EditorShell::kLightmapPanel);
    rig.shell.dock()->layout();
    rig.frames(4);
}

SurfaceData planeSurface(float half)
{
    SurfaceData data;
    const float y = 0.0f;
    const Math::Vec3 corners[4] = { Math::Vec3(-half, y, -half), Math::Vec3(half, y, -half),
        Math::Vec3(half, y, half), Math::Vec3(-half, y, half) };
    const Math::Vec2 uvs[4] = { Math::Vec2(0.0f, 0.0f), Math::Vec2(1.0f, 0.0f),
        Math::Vec2(1.0f, 1.0f), Math::Vec2(0.0f, 1.0f) };
    for (int i = 0; i < 4; ++i)
    {
        data.positions.push_back(corners[i]);
        data.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
        data.uvs.push_back(uvs[i]);
        data.uvs2.push_back(uvs[i]);
    }
    const std::uint32_t indices[6] = { 0, 2, 1, 0, 3, 2 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

float halfRead(const ct::Vector<std::uint16_t>& pixels, int resolution, int x, int row, int c)
{
    return LightmapGIData::halfToFloat(
            pixels[(static_cast<std::size_t>(row) * static_cast<std::size_t>(resolution) +
                           static_cast<std::size_t>(x)) *
                            4 +
                    static_cast<std::size_t>(c)]);
}

bool rayHitsBox(const Math::Vec3& origin, const Math::Vec3& direction, const Math::Vec3& low,
        const Math::Vec3& high)
{
    float tMin = 0.0f;
    float tMax = 1.0e9f;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (fabsf(direction[axis]) < 1.0e-8f)
        {
            if (origin[axis] < low[axis] || origin[axis] > high[axis]) return false;
            continue;
        }
        float t0 = (low[axis] - origin[axis]) / direction[axis];
        float t1 = (high[axis] - origin[axis]) / direction[axis];
        if (t0 > t1)
        {
            const float swap = t0;
            t0 = t1;
            t1 = swap;
        }
        if (t0 > tMin) tMin = t0;
        if (t1 < tMax) tMax = t1;
        if (tMin > tMax) return false;
    }
    return true;
}

bool runPass(Rig& rig, LightmapBakePass& pass, int limit = 400)
{
    for (int i = 0; i < limit && !pass.done() && !pass.failed(); ++i)
    {
        pass.step();
        rig.frame();
    }
    return pass.done();
}

struct PassScene
{
    MeshId plane;
    MeshId box;
    Math::Mat4 planeModel;
    Math::Mat4 boxModel;
    Math::Vec3 direction;
    Math::Vec3 color;
    Math::Vec3 ambient;
    ct::Vector<LightmapBakePass::Caster> casters;
};

PassScene makePassScene()
{
    PassScene scene;
    MeshStore& meshes = *Main::meshes();
    scene.plane = meshes.create("lightmap test plane");
    CHECK(meshes.addSurface(scene.plane, planeSurface(4.0f), "plane"));
    scene.box = createBoxMesh(meshes, Math::Vec3(1.0f, 1.0f, 1.0f), "lightmap test box");
    scene.planeModel = Math::Mat4::Identity();
    scene.boxModel = Math::Mat4::Translation(Math::Vec3(0.0f, 0.5f, 0.0f));
    scene.direction = Math::Vec3(0.3f, -0.8f, 0.2f).Normalized();
    scene.color = Math::Vec3(1.0f, 0.9f, 0.7f);
    scene.ambient = Math::Vec3(0.1f, 0.2f, 0.3f);
    LightmapBakePass::Caster plane;
    plane.mesh = scene.plane;
    plane.transform = scene.planeModel;
    LightmapBakePass::Caster box;
    box.mesh = scene.box;
    box.transform = scene.boxModel;
    scene.casters.push_back(plane);
    scene.casters.push_back(box);
    return scene;
}

bool shadowed(const PassScene& scene, float x, float z)
{
    const Math::Vec3 origin(x, 0.001f, z);
    return rayHitsBox(origin, scene.direction * -1.0f, Math::Vec3(-0.5f, 0.0f, -0.5f),
            Math::Vec3(0.5f, 1.0f, 0.5f));
}

LightmapBakeSettings passSettings(const PassScene& scene)
{
    LightmapBakeSettings settings;
    settings.ambient = scene.ambient;
    settings.shadowResolution = 2048;
    settings.filterRadius = 0.0f;
    settings.sampleCount = 1;
    return settings;
}

float litValue(const PassScene& scene, int channel)
{
    return scene.ambient[channel] + scene.color[channel] * -scene.direction.y;
}

void testBakePass(Rig& rig)
{
    printf("the bake pass writes the sun, the ambient and the shadow of the other meshes\n");
    PassScene scene = makePassScene();
    const int resolution = 128;
    LightmapBakePass pass;
    LightmapBakeSettings settings = passSettings(scene);
    CHECK(pass.begin(scene.plane, scene.planeModel, scene.casters, scene.direction, scene.color,
            resolution, settings));
    CHECK(pass.active() && !pass.done());
    CHECK(runPass(rig, pass));
    CHECK(pass.resolution() == static_cast<std::uint32_t>(resolution));
    const ct::Vector<std::uint16_t>& pixels = pass.pixels();
    CHECK(pixels.size() == static_cast<std::size_t>(resolution * resolution * 4));
    if (pixels.size() != static_cast<std::size_t>(resolution * resolution * 4)) return;

    float worstLit = 0.0f;
    float worstShadow = 0.0f;
    int litCount = 0;
    int shadowCount = 0;
    int wrongSide = 0;
    for (int row = 0; row < resolution; ++row)
    {
        for (int x = 0; x < resolution; ++x)
        {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(resolution);
            const float v = (static_cast<float>(row) + 0.5f) / static_cast<float>(resolution);
            const float worldX = (u * 2.0f - 1.0f) * 4.0f;
            const float worldZ = (v * 2.0f - 1.0f) * 4.0f;
            bool inside = shadowed(scene, worldX, worldZ);
            bool near = false;
            for (int dz = -1; dz <= 1 && !near; ++dz)
                for (int dx = -1; dx <= 1 && !near; ++dx)
                    if (shadowed(scene, worldX + static_cast<float>(dx) * 0.16f,
                                worldZ + static_cast<float>(dz) * 0.16f) != inside)
                        near = true;
            if (near) continue;
            for (int c = 0; c < 3; ++c)
            {
                const float value = halfRead(pixels, resolution, x, row, c);
                const float expected = inside ? scene.ambient[c] : litValue(scene, c);
                const float error = fabsf(value - expected);
                if (inside) worstShadow = fmaxf(worstShadow, error);
                else
                    worstLit = fmaxf(worstLit, error);
                if (fabsf(value - (inside ? litValue(scene, c) : scene.ambient[c])) < 0.02f)
                    ++wrongSide;
            }
            if (inside) ++shadowCount;
            else
                ++litCount;
        }
    }
    printf("  lit texels %d (worst error %.4f), shadow texels %d (worst error %.4f)\n", litCount,
            static_cast<double>(worstLit), shadowCount, static_cast<double>(worstShadow));
    CHECK(litCount > 8000 && shadowCount > 100);
    CHECK(worstLit < 0.02f);
    CHECK(worstShadow < 0.02f);
    CHECK(wrongSide == 0);
    CHECK(halfRead(pixels, resolution, 10, 10, 3) == 1.0f);

    const std::size_t index = (static_cast<std::size_t>(64) * resolution + 64) * 4;
    CHECK(LightmapGIData::halfToFloat(pixels[index + 3]) == 1.0f);

    LightmapBakeSettings tilted = settings;
    tilted.ambientGround = 0.5f;
    CHECK(pass.begin(scene.plane, scene.planeModel, scene.casters, scene.direction, scene.color,
            resolution, tilted));
    pass.cancel();
    CHECK(!pass.active() && !pass.done() && pass.pixels().empty());
}

int transitionTexels(const ct::Vector<std::uint16_t>& pixels, int resolution, int row, float low,
        float high)
{
    int count = 0;
    for (int x = resolution / 2; x < resolution; ++x)
    {
        const float value = halfRead(pixels, resolution, x, row, 0);
        const float t = (value - low) / (high - low);
        if (t > 0.1f && t < 0.9f) ++count;
    }
    return count;
}

void testPenumbra(Rig& rig)
{
    printf("an angular radius softens the baked shadow edge\n");
    PassScene scene = makePassScene();
    const int resolution = 256;
    const float lit = litValue(scene, 0);
    const float dark = scene.ambient[0];
    const int row = static_cast<int>((0.25f / 4.0f * 0.5f + 0.5f) * static_cast<float>(resolution));

    LightmapBakeSettings hard = passSettings(scene);
    LightmapBakePass pass;
    CHECK(pass.begin(scene.plane, scene.planeModel, scene.casters, scene.direction, scene.color,
            resolution, hard));
    CHECK(runPass(rig, pass));
    const int hardWidth = transitionTexels(pass.pixels(), resolution, row, dark, lit);

    LightmapBakeSettings soft = passSettings(scene);
    soft.sunAngularRadius = 5.0f;
    soft.sampleCount = 16;
    CHECK(pass.begin(scene.plane, scene.planeModel, scene.casters, scene.direction, scene.color,
            resolution, soft));
    CHECK(pass.samplesTotal() == 16);
    int frames = 0;
    while (pass.active() && frames < 100)
    {
        pass.step();
        rig.frame();
        ++frames;
    }
    CHECK(pass.done());
    CHECK(frames == 17);
    const int softWidth = transitionTexels(pass.pixels(), resolution, row, dark, lit);
    printf("  edge width: hard %d texels, soft %d texels\n", hardWidth, softWidth);
    CHECK(hardWidth <= 2);
    CHECK(softWidth >= hardWidth + 3);
}

void testLargeFloor(Rig& rig)
{
    printf("a large floor with a soft sun bakes flat outside the shadow\n");
    MeshStore& meshes = *Main::meshes();
    PassScene scene = makePassScene();
    scene.plane = meshes.create("lightmap large plane");
    CHECK(meshes.addSurface(scene.plane, planeSurface(10.0f), "plane"));
    scene.casters[0].mesh = scene.plane;
    scene.direction = Math::Vec3(-4.0f, -6.0f, -3.0f).Normalized();
    const int resolution = 1024;
    LightmapBakeSettings settings;
    settings.ambient = scene.ambient;
    settings.shadowResolution = 2048;
    settings.sampleCount = 4;
    settings.sunAngularRadius = 2.0f;
    settings.filterRadius = 1.0f;
    LightmapBakePass pass;
    CHECK(pass.begin(scene.plane, scene.planeModel, scene.casters, scene.direction, scene.color,
            resolution, settings));
    CHECK(runPass(rig, pass));
    const ct::Vector<std::uint16_t>& pixels = pass.pixels();
    if (pixels.size() != static_cast<std::size_t>(resolution) * resolution * 4) return;
    const float lit = scene.ambient[0] + scene.color[0] * -scene.direction.y;
    float worst = 0.0f;
    int worstX = 0;
    int worstRow = 0;
    int count = 0;
    for (int row = 0; row < resolution; ++row)
        for (int x = 0; x < resolution; ++x)
        {
            const float worldX =
                    ((static_cast<float>(x) + 0.5f) / resolution * 2.0f - 1.0f) * 10.0f;
            const float worldZ =
                    ((static_cast<float>(row) + 0.5f) / resolution * 2.0f - 1.0f) * 10.0f;
            bool shadowedNear = false;
            for (int dz = -4; dz <= 4 && !shadowedNear; ++dz)
                for (int dx = -4; dx <= 4 && !shadowedNear; ++dx)
                    if (shadowed(scene, worldX + static_cast<float>(dx) * 0.25f,
                                worldZ + static_cast<float>(dz) * 0.25f))
                        shadowedNear = true;
            if (shadowedNear) continue;
            ++count;
            const float error = fabsf(halfRead(pixels, resolution, x, row, 0) - lit);
            if (error > worst)
            {
                worst = error;
                worstX = x;
                worstRow = row;
            }
        }
    printf("  %d texels away from the shadow, worst error %.4f at (%d, %d)\n", count,
            static_cast<double>(worst), worstX, worstRow);
    CHECK(count > 500000);
    CHECK(worst < 0.02f);
}

void testPassRefusals(Rig& rig)
{
    printf("the bake pass refuses what it cannot bake\n");
    (void) rig;
    MeshStore& meshes = *Main::meshes();
    const MeshId noUv2 = createBoxMesh(meshes, Math::Vec3(1.0f, 1.0f, 1.0f), "no uv2");
    ct::Vector<LightmapBakePass::Caster> casters;
    LightmapBakePass pass;
    LightmapBakeSettings settings;
    CHECK(!pass.begin(noUv2, Math::Mat4::Identity(), casters, Math::Vec3(0, -1, 0),
            Math::Vec3(1, 1, 1), 64, settings));
    CHECK(!pass.begin(MeshId(), Math::Mat4::Identity(), casters, Math::Vec3(0, -1, 0),
            Math::Vec3(1, 1, 1), 64, settings));
    CHECK(!pass.active() && !pass.done());
    CHECK(!pass.step());
}


bool progressStop(const char*, std::uint32_t, void*)
{
    return false;
}

struct Coverage
{
    int overlaps = 0;
    int covered = 0;
    float minimum = 1.0e9f;
    float maximum = -1.0e9f;
};

Coverage coverageOf(const ct::Vector<SurfaceData>& surfaces, std::uint32_t width,
        std::uint32_t height)
{
    Coverage result;
    ct::Vector<std::uint8_t> counts;
    counts.resize(static_cast<std::size_t>(width) * height, 0);
    for (std::size_t s = 0; s < surfaces.size(); ++s)
    {
        const SurfaceData& surface = surfaces[s];
        for (std::size_t v = 0; v < surface.uvs2.size(); ++v)
        {
            result.minimum = fminf(result.minimum, fminf(surface.uvs2[v].x, surface.uvs2[v].y));
            result.maximum = fmaxf(result.maximum, fmaxf(surface.uvs2[v].x, surface.uvs2[v].y));
        }
        for (std::size_t i = 0; i + 2 < surface.indices.size(); i += 3)
        {
            Math::Vec2 p[3];
            for (int k = 0; k < 3; ++k)
                p[k] = Math::Vec2(surface.uvs2[surface.indices[i + k]].x *
                                          static_cast<float>(width),
                        surface.uvs2[surface.indices[i + k]].y * static_cast<float>(height));
            const float area =
                    (p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[1].y - p[0].y) * (p[2].x - p[0].x);
            if (fabsf(area) < 1.0e-6f) continue;
            const float sign = area > 0.0f ? 1.0f : -1.0f;
            const int x0 = static_cast<int>(floorf(fminf(p[0].x, fminf(p[1].x, p[2].x))));
            const int x1 = static_cast<int>(ceilf(fmaxf(p[0].x, fmaxf(p[1].x, p[2].x))));
            const int y0 = static_cast<int>(floorf(fminf(p[0].y, fminf(p[1].y, p[2].y))));
            const int y1 = static_cast<int>(ceilf(fmaxf(p[0].y, fmaxf(p[1].y, p[2].y))));
            for (int y = y0; y < y1; ++y)
                for (int x = x0; x < x1; ++x)
                {
                    if (x < 0 || y < 0 || x >= static_cast<int>(width) ||
                            y >= static_cast<int>(height))
                        continue;
                    const float cx = static_cast<float>(x) + 0.5f;
                    const float cy = static_cast<float>(y) + 0.5f;
                    bool inside = true;
                    for (int k = 0; k < 3 && inside; ++k)
                    {
                        const Math::Vec2& a = p[k];
                        const Math::Vec2& b = p[(k + 1) % 3];
                        const float edge =
                                ((b.x - a.x) * (cy - a.y) - (b.y - a.y) * (cx - a.x)) * sign;
                        if (edge <= 1.0e-4f) inside = false;
                    }
                    if (!inside) continue;
                    std::uint8_t& count = counts[static_cast<std::size_t>(y) * width + x];
                    if (count == 0) ++result.covered;
                    else
                        ++result.overlaps;
                    if (count < 255) ++count;
                }
        }
    }
    return result;
}

float totalArea(const ct::Vector<SurfaceData>& surfaces)
{
    return LightmapUnwrapper::worldSurfaceArea(surfaces, Math::Vec3(1.0f, 1.0f, 1.0f));
}

std::size_t triangleCount(const ct::Vector<SurfaceData>& surfaces)
{
    std::size_t count = 0;
    for (std::size_t s = 0; s < surfaces.size(); ++s) count += surfaces[s].indices.size() / 3;
    return count;
}

void testUnwrapper(Rig& rig)
{
    printf("xatlas unwraps a mesh into a second UV set inside [0,1] without overlaps\n");
    ct::Vector<SurfaceData> input;
    SurfaceData box;
    boxMeshArrays(box, Math::Vec3(2.0f, 1.0f, 1.5f));
    input.push_back(box);
    SurfaceData sphere;
    sphereMeshArrays(sphere, 1.0f, 12, 16);
    input.push_back(sphere);
    SurfaceData plane;
    planeMeshArrays(plane, 4.0f, 4.0f, 2, 2);
    input.push_back(plane);

    const float area = totalArea(input);
    LightmapUnwrapper unwrapper;
    LightmapUnwrapSettings settings;
    settings.resolution = 0;
    settings.padding = 4;
    settings.texelsPerUnit = LightmapUnwrapper::fittedTexelsPerUnit(area, 256);
    CHECK(settings.texelsPerUnit > 0.0f);
    ct::Vector<SurfaceData> output;
    LightmapUnwrapResult atlas;
    ct::String error;
    CHECK(unwrapper.unwrap(input, output, settings, &atlas, &error));
    CHECK(error.empty());
    CHECK(atlas.chartCount > 0 && atlas.width > 0 && atlas.height > 0);
    CHECK(output.size() == input.size());
    CHECK(triangleCount(output) == triangleCount(input));
    for (std::size_t s = 0; s < output.size(); ++s)
    {
        CHECK(output[s].uvs2.size() == output[s].positions.size());
        CHECK(output[s].normals.size() == output[s].positions.size());
        CHECK(output[s].uvs.size() == output[s].positions.size());
        CHECK(output[s].positions.size() >= input[s].positions.size());
    }
    CHECK(fabsf(totalArea(output) - area) < area * 1.0e-3f);
    const Coverage coverage = coverageOf(output, atlas.width, atlas.height);
    printf("  atlas %ux%u, %u charts, %d texels covered, %d overlapping\n", atlas.width,
            atlas.height, atlas.chartCount, coverage.covered, coverage.overlaps);
    CHECK(coverage.minimum >= -1.0e-4f && coverage.maximum <= 1.0f + 1.0e-4f);
    CHECK(coverage.covered > 0);
    CHECK(coverage.overlaps == 0);
    CHECK(static_cast<float>(coverage.covered) >
            0.5f * area * settings.texelsPerUnit * settings.texelsPerUnit * 0.5f);

    LightmapUnwrapSettings stopping = settings;
    stopping.progress = &progressStop;
    ct::Vector<SurfaceData> none;
    CHECK(!unwrapper.unwrap(input, none, stopping, &atlas, &error));
    CHECK(none.empty() && error == "cancelled" && atlas.width == 0);

    ct::Vector<SurfaceData> bad = input;
    bad[0].joints.push_back(0);
    CHECK(!unwrapper.unwrap(bad, none, settings, nullptr, &error) && !error.empty());
    bad = input;
    bad[1].topology = prisma::Topology::Lines;
    CHECK(!unwrapper.unwrap(bad, none, settings, nullptr, &error) && !error.empty());
    bad = input;
    bad[2].indices.push_back(0);
    CHECK(!unwrapper.unwrap(bad, none, settings, nullptr, &error) && !error.empty());
    bad = input;
    bad[2].indices[0] = 100000;
    CHECK(!unwrapper.unwrap(bad, none, settings, nullptr, &error) && !error.empty());
    CHECK(!unwrapper.unwrap(ct::Vector<SurfaceData>(), none, settings, nullptr, &error));
    rig.frames(1);
}

void testUnwrapJob(Rig& rig)
{
    printf("the unwrap runs on a worker thread with progress and can be cancelled\n");
    ct::Vector<SurfaceData> input;
    SurfaceData sphere;
    sphereMeshArrays(sphere, 1.0f, 12, 16);
    input.push_back(sphere);
    LightmapUnwrapSettings settings;
    settings.resolution = 0;
    settings.texelsPerUnit = LightmapUnwrapper::fittedTexelsPerUnit(totalArea(input), 256);

    LightmapUnwrapJob job;
    CHECK(!job.running());
    CHECK(job.start(input, settings));
    CHECK(!job.start(input, settings));
    bool sawStage = false;
    std::uint32_t last = 0;
    ct::Vector<SurfaceData> output;
    bool succeeded = false;
    int polls = 0;
    while (!job.collect(output, succeeded) && polls < 4000)
    {
        sawStage = sawStage || !job.stage().empty();
        last = job.percent() > last ? job.percent() : last;
        rig.frame();
        ++polls;
    }
    CHECK(succeeded && !job.running());
    CHECK(sawStage && output.size() == 1 && output[0].uvs2.size() == output[0].positions.size());
    CHECK(job.result().width > 0 && job.result().chartCount > 0 && job.error().empty());
    CHECK(!job.collect(output, succeeded));

    SurfaceData heavy;
    sphereMeshArrays(heavy, 2.0f, 240, 240);
    ct::Vector<SurfaceData> big;
    big.push_back(heavy);
    LightmapUnwrapSettings large;
    large.resolution = 0;
    large.texelsPerUnit = 40.0f;
    CHECK(job.start(big, large));
    job.cancel();
    polls = 0;
    while (!job.collect(output, succeeded) && polls < 4000)
    {
        rig.frame();
        ++polls;
    }
    CHECK(!succeeded && job.cancelled() && output.empty());
    CHECK(job.error() == "cancelled");
    CHECK(!job.running());
}

InputEvent keyEvent(Key key, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = key;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = modifiers;
    return event;
}

void tap(Rig& rig, Key key, unsigned modifiers = 0)
{
    rig.send(keyEvent(key, true, modifiers));
    rig.frames(2);
    rig.send(keyEvent(key, false, modifiers));
    rig.frames(2);
}

bool openBarMenu(Rig& rig, rt::Menu* menu)
{
    const rt::Rect barRect = rig.shell.menuBar()->absoluteRect();
    const int by = static_cast<int>(barRect.y + barRect.h * 0.5f);
    for (int x = static_cast<int>(barRect.x) + 4; x < 500; x += 4)
    {
        rig.send(motion(x, by));
        rig.frames(2);
        rig.send(button(x, by, MouseButton::Left, true));
        rig.frames(2);
        rig.send(button(x, by, MouseButton::Left, false));
        rig.frames(2);
        if (rt::WidgetApp::instance().popup() == menu) return true;
    }
    return false;
}

void clickMenuItem(Rig& rig, rt::Menu* menu, int item)
{
    const rt::Theme& theme = rt::Theme::instance();
    const rt::Rect rect = menu->absoluteRect();
    float y = rect.y + 4.0f;
    for (int i = 0; i < item; ++i)
        y += menu->action(i)->isSeparator() ? 9.0f : theme.menuItemHeight;
    const int px = static_cast<int>(rect.x + 24.0f);
    const int py = static_cast<int>(y + theme.menuItemHeight * 0.5f);
    rig.send(motion(px, py));
    rig.frames(2);
    rig.send(button(px, py, MouseButton::Left, true));
    rig.frames(2);
    rig.send(button(px, py, MouseButton::Left, false));
    rig.frames(3);
}

int countChildren(Node* scene, const char* type)
{
    int count = 0;
    for (std::size_t i = 0; i < scene->getChildCount(); ++i)
        if (strcmp(scene->getChild(i)->getClassName(), type) == 0) ++count;
    return count;
}

struct World
{
    MeshInstance3D* floor = nullptr;
    MeshInstance3D* box = nullptr;
    DirectionalLight3D* sun = nullptr;
    WorldEnvironment* environment = nullptr;
    ViewportId view;
    CameraId camera;
    Math::Vec3 direction;
    Math::Vec3 sunLinear;
    Math::Vec3 ambientLinear;
};

const float kOrthoSize = 10.0f;
const float kPixelsPerUnit = static_cast<float>(kShot) / kOrthoSize;

Math::Vec3 ambientColour()
{
    return Math::Vec3(0.35f, 0.45f, 0.6f);
}

Math::Vec3 sunColour()
{
    return Math::Vec3(1.0f, 0.9f, 0.7f);
}

EnvironmentSettings worldEnvironment()
{
    EnvironmentSettings env;
    env.background = BackgroundMode::Color;
    env.backgroundColor = Math::Vec3(0.0f, 0.0f, 0.0f);
    env.ambientSource = AmbientSource::Color;
    env.ambientColor = ambientColour();
    env.ambientEnergy = 1.0f;
    env.reflectionSource = ReflectionSource::Disabled;
    return env;
}

void findNodes(Node* scene, World& world)
{
    world.floor = static_cast<MeshInstance3D*>(scene->findChild("Floor"));
    world.box = static_cast<MeshInstance3D*>(scene->findChild("Box"));
    world.sun = static_cast<DirectionalLight3D*>(scene->findChild("Sun"));
    world.environment = static_cast<WorldEnvironment*>(scene->findChild("Environment"));
    world.direction =
            world.sun->getGlobalTransform().TransformVector(Math::Vec3(0, 0, -1)).Normalized();
    world.sunLinear = Math::Vec3(srgbToLinear(sunColour().x), srgbToLinear(sunColour().y),
            srgbToLinear(sunColour().z));
    world.ambientLinear = Math::Vec3(srgbToLinear(ambientColour().x),
            srgbToLinear(ambientColour().y), srgbToLinear(ambientColour().z));
}

void buildWorld(Rig& rig, World& world)
{
    CHECK(rig.document.createProject(testRoot() + "/projects", "Lightmaps"));
    rig.frames(4);
    Node* scene = rig.document.scene();
    CHECK(scene != nullptr);
    if (!scene) return;

    MeshInstance3D* floor = scene->createChild<MeshInstance3D>();
    floor->setName("Floor");
    floor->setMesh(createPlaneMesh(*Main::meshes(), 8.0f, 8.0f));
    MeshInstance3D* box = scene->createChild<MeshInstance3D>();
    box->setName("Box");
    box->setMesh(createBoxMesh(*Main::meshes(), Math::Vec3(1.0f, 1.0f, 1.0f)));
    box->setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));
    DirectionalLight3D* sun = scene->createChild<DirectionalLight3D>();
    sun->setName("Sun");
    sun->setPosition(Math::Vec3(5.0f, 5.0f, 3.0f));
    sun->lookAt(Math::Vec3(0.0f, 0.0f, 0.0f));
    sun->setColor(sunColour());
    sun->setEnergy(1.0f);
    sun->setShadow(true);
    sun->setShadowMode(DirectionalShadowMode::Orthogonal);
    WorldEnvironment* environment = scene->createChild<WorldEnvironment>();
    environment->setName("Environment");
    environment->setEnvironment(worldEnvironment());
    rig.frames(3);
    findNodes(scene, world);

    RenderingServer& server = *Main::rendering();
    server.setDeferredPipelinesEnabled(false);
    world.view = server.viewportCreate();
    server.viewportSetSize(world.view, kShot, kShot);
    server.viewportSetScenario(world.view, rig.tree.getRoot()->castTo<Window>()->getScenario());
    server.viewportSetClearColor(world.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    world.camera = server.cameraCreate();
    server.cameraSetOrthogonal(world.camera, kOrthoSize, 0.1f, 60.0f);
    server.cameraSetTransform(world.camera,
            Math::Mat4(Math::Vec4(1, 0, 0, 0), Math::Vec4(0, 0, -1, 0), Math::Vec4(0, 1, 0, 0),
                    Math::Vec4(0, 12, 0, 1)));
    server.viewportAttachCamera(world.view, world.camera);
    server.viewportSetUpdateMode(world.view, ViewportUpdateMode::Always);
    server.viewportSetActive(world.view, true);
    CHECK(server.viewportGetTexture(world.view).valid());
    rig.frames(4);
}

bool capture(Rig& rig, const World& world, Image& image)
{
    rig.frames(3);
    RenderingServer& server = *Main::rendering();
    image.width = kShot;
    image.height = kShot;
    image.pixels.resize(static_cast<std::size_t>(kShot) * kShot * 4);
    prisma::RenderTarget source;
    source.texture = Main::textures()->gpu(server.viewportGetTexture(world.view));
    prisma::Rect rect;
    rect.width = kShot;
    rect.height = kShot;
    return Main::device()->driver()->readPixels(source, rect, image.pixels.data());
}

void dump(const Image& image, const char* tag)
{
    const char* directory = getenv("PRISMA3D_SHOTS");
    if (!directory || !directory[0]) return;
    Pixmap pixmap(image.width, image.height, 4);
    for (int y = 0; y < image.height; ++y)
        for (int x = 0; x < image.width; ++x)
            pixmap.set_pixel(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(image.height - 1 - y),
                    static_cast<std::uint8_t>(image.channel(x, y, 0)),
                    static_cast<std::uint8_t>(image.channel(x, y, 1)),
                    static_cast<std::uint8_t>(image.channel(x, y, 2)), 255);
    char path[512];
    snprintf(path, sizeof(path), "%s/lightmap_%s_%s.png", directory, useVulkan ? "vulkan" : "gl",
            tag);
    pixmap.save(path);
}

int maxDifference(const Image& a, const Image& b)
{
    int worst = 0;
    for (std::size_t i = 0; i < a.pixels.size(); ++i)
    {
        if ((i & 3) == 3) continue;
        const int d = abs(static_cast<int>(a.pixels[i]) - static_cast<int>(b.pixels[i]));
        if (d > worst) worst = d;
    }
    return worst;
}

int pixelX(float worldX)
{
    return static_cast<int>(floorf(static_cast<float>(kShot) * 0.5f + worldX * kPixelsPerUnit));
}

int pixelY(float worldZ)
{
    const int topDown =
            static_cast<int>(floorf(static_cast<float>(kShot) * 0.5f + worldZ * kPixelsPerUnit));
    return kShot - 1 - topDown;
}

bool shadowedByBox(const World& world, float x, float z)
{
    return rayHitsBox(Math::Vec3(x, 0.001f, z), world.direction * -1.0f,
            Math::Vec3(-0.5f, 0.0f, -0.5f), Math::Vec3(0.5f, 1.0f, 0.5f));
}

struct FloorCheck
{
    int lit = 0;
    int shadow = 0;
    int worstLit = 0;
    int worstShadow = 0;
    int wrongSide = 0;
};

FloorCheck checkFloor(const World& world, const Image& image)
{
    FloorCheck result;
    const float lightNormal = -world.direction.y;
    for (int y = 0; y < image.height; ++y)
    {
        for (int x = 0; x < image.width; ++x)
        {
            const float worldX = (static_cast<float>(x) + 0.5f - static_cast<float>(kShot) * 0.5f) /
                                 kPixelsPerUnit;
            const int topDown = kShot - 1 - y;
            const float worldZ =
                    (static_cast<float>(topDown) + 0.5f - static_cast<float>(kShot) * 0.5f) /
                    kPixelsPerUnit;
            if (fabsf(worldX) > 3.6f || fabsf(worldZ) > 3.6f) continue;
            if (fabsf(worldX) < 0.55f && fabsf(worldZ) < 0.55f) continue;
            const bool inside = shadowedByBox(world, worldX, worldZ);
            bool near = false;
            for (int dz = -2; dz <= 2 && !near; ++dz)
                for (int dx = -2; dx <= 2 && !near; ++dx)
                    if (shadowedByBox(world, worldX + static_cast<float>(dx) * 0.12f,
                                worldZ + static_cast<float>(dz) * 0.12f) != inside)
                        near = true;
            if (near) continue;
            int worst = 0;
            int wrong = 0;
            for (int c = 0; c < 3; ++c)
            {
                const float direct = world.sunLinear[c] * lightNormal;
                const float ambient = world.ambientLinear[c];
                const int expected = encode(inside ? ambient : ambient + direct);
                const int other = encode(inside ? ambient + direct : ambient);
                const int value = image.channel(x, y, c);
                const int d = abs(value - expected);
                if (d > worst) worst = d;
                if (abs(value - other) < d && abs(expected - other) > 8) ++wrong;
            }
            if (inside)
            {
                ++result.shadow;
                if (worst > result.worstShadow) result.worstShadow = worst;
            }
            else
            {
                ++result.lit;
                if (worst > result.worstLit) result.worstLit = worst;
            }
            if (wrong > 0) ++result.wrongSide;
        }
    }
    return result;
}

int litFloorPixel(const Image& image, int c)
{
    return image.channel(pixelX(-3.0f), pixelY(-3.0f), c);
}

int shadowFloorPixel(const Image& image, int c)
{
    return image.channel(pixelX(-1.0f), pixelY(-0.6f), c);
}

int boxTopPixel(const Image& image, int c)
{
    return image.channel(pixelX(0.0f), pixelY(0.0f), c);
}

LightmapGI* lightmapOf(Rig& rig)
{
    return rig.baker.lightmapNode();
}

void testSceneSetup(Rig& rig, World& world)
{
    printf("the scene renders its sun with real shadows before any bake\n");
    buildWorld(rig, world);
    CHECK(world.floor && world.box && world.sun && world.environment);
    if (!world.floor || !world.box || !world.sun || !world.environment) return;
    CHECK(world.floor->getGiMode() == GiMode::Static);
    CHECK(world.sun->getBakeMode() == LightBakeMode::Dynamic);
    CHECK(rig.document.saveScene());
    CHECK(!rig.document.dirty());
    CHECK(rig.baker.lightmapNode() == nullptr);
    CHECK(rig.baker.sun() == world.sun && !rig.baker.sunIsStatic());
    CHECK(rig.baker.meshes().size() == 2);

    Image dynamic;
    CHECK(capture(rig, world, dynamic));
    dump(dynamic, "1_dynamic");
    const int floorLit = litFloorPixel(dynamic, 0);
    const int floorShadow = shadowFloorPixel(dynamic, 0);
    printf("  dynamic: lit floor %d, shadowed floor %d, box top %d\n", floorLit, floorShadow,
            boxTopPixel(dynamic, 0));
    CHECK(floorLit > floorShadow + 40);
    CHECK(boxTopPixel(dynamic, 0) > floorShadow + 40);
}

void expectBaked(Rig& rig, const World& world, const Image& baked, const char* tag)
{
    const FloorCheck check = checkFloor(world, baked);
    printf("  %s: floor lit %d (worst %d), shadow %d (worst %d), wrong side %d\n", tag, check.lit,
            check.worstLit, check.shadow, check.worstShadow, check.wrongSide);
    CHECK(check.lit > 4000 && check.shadow > 100);
    CHECK(check.worstLit <= 4);
    CHECK(check.worstShadow <= 4);
    CHECK(check.wrongSide == 0);
    const float lightNormal = -world.direction.y;
    for (int c = 0; c < 3; ++c)
    {
        const int top = encode(world.ambientLinear[c] + world.sunLinear[c] * lightNormal);
        CHECK(abs(boxTopPixel(baked, c) - top) <= 4);
    }
    (void) rig;
}

void testPanelFlow(Rig& rig, World& world)
{
    printf("Scene > Bake Lightmaps... adds the node and the panel bakes the scene\n");
    showPanel(rig);
    CHECK(rig.lightmap.nodeButton()->text() == "Add LightmapGI");
    CHECK(rig.lightmap.nodeLabel()->text() == LightmapPanel::kNoNodeText);
    CHECK(rig.lightmap.bakeButton()->isEnabled() && rig.lightmap.unwrapButton()->isEnabled());
    CHECK(rig.lightmap.rowCount() == 2);
    rt::MenuAction* entry = rig.menus.action(EditorMenus::kScene, EditorMenus::kSceneBakeLightmaps);
    CHECK(entry && entry->text() == "Bake Lightmaps..." && entry->isEnabled());

    rt::Menu* sceneMenu = rig.menus.menu(EditorMenus::kScene);
    const std::size_t undoBefore = rig.document.undoRedo().undoCount();
    CHECK(openBarMenu(rig, sceneMenu));
    clickMenuItem(rig, sceneMenu, EditorMenus::kSceneBakeLightmaps);
    LightmapGI* gi = lightmapOf(rig);
    CHECK(gi != nullptr);
    if (!gi) return;
    CHECK(gi->getParent() == rig.document.scene());
    CHECK(rig.document.undoRedo().undoCount() == undoBefore + 1);
    CHECK(rig.document.undoRedo().currentActionName() == HierarchyActions::kAddAction);
    CHECK(rig.selection.isSelected(gi));
    CHECK(rig.shell.panel(EditorShell::kLightmapPanel)->isVisible());
    CHECK(rig.lightmap.nodeButton()->text() == "Select LightmapGI");
    CHECK(rig.lightmap.nodeLabel()->text().starts_with("LightmapGI: LightmapGI"));
    CHECK(rig.lightmap.dataLabel()->text() == "data: none");
    CHECK(rig.lightmap.sunLabel()->text().ends_with("bake mode not Static"));
    CHECK(rig.lightmap.sunButton()->isEnabled() && rig.lightmap.bakeButton()->isEnabled());


    clickWidget(rig, rig.lightmap.nodeButton());
    CHECK(countChildren(rig.document.scene(), "LightmapGI") == 1);

    CHECK(!rig.baker.sunIsStatic());
    clickWidget(rig, rig.lightmap.sunButton());
    CHECK(world.sun->getBakeMode() == LightBakeMode::Static && rig.baker.sunIsStatic());
    CHECK(rig.document.undoRedo().currentActionName() == LightmapBaker::kStaticAction);
    CHECK(rig.lightmap.sunLabel()->text().ends_with("bake mode Static"));
    CHECK(!rig.lightmap.sunButton()->isEnabled());

    clickWidget(rig, rig.lightmap.draftButton());
    CHECK(rig.baker.settings().resolution == 1024 && rig.baker.settings().bake.sampleCount == 4);
    CHECK(rig.baker.settings().bake.shadowResolution == 2048);
    CHECK(gi->getBakeQuality() == LightmapGI::BakeQuality::Low && gi->getMaxTextureSize() == 2048);
    CHECK(!gi->isUsingDenoiser() && gi->getTexelScale() == 1.0f);
    clickWidget(rig, rig.lightmap.finalButton());
    CHECK(rig.baker.settings().resolution == 4096 && rig.baker.settings().bake.sampleCount == 16);
    CHECK(gi->getBakeQuality() == LightmapGI::BakeQuality::High && gi->getMaxTextureSize() == 4096);
    CHECK(rig.lightmap.field(LightmapPanel::kResolution)->value() == 4096.0f);
    clickWidget(rig, rig.lightmap.draftButton());
    CHECK(rig.lightmap.field(LightmapPanel::kResolution)->value() == 1024.0f);

    LightmapBaker::Settings& settings = rig.baker.settings();
    settings.resolution = 256;
    settings.bake.shadowResolution = 2048;
    settings.bake.sampleCount = 1;
    settings.bake.sunAngularRadius = 0.0f;
    settings.bake.filterRadius = 0.0f;
    gi->setEnvironmentMode(LightmapGI::EnvironmentMode::Scene);
    rig.frames(2);
    const Math::Vec3 ambient = rig.baker.ambient();
    CHECK(fabsf(ambient.x - world.ambientLinear.x) < 1.0e-4f &&
            fabsf(ambient.z - world.ambientLinear.z) < 1.0e-4f);
    CHECK(rig.lightmap.ambientLabel()->text().starts_with("Ambient: "));
}

ct::String readFile(const ct::String& path)
{
    char* text = file_read_text(path.c_str());
    if (!text) return ct::String();
    const ct::String result(text);
    fs_free(text);
    return result;
}

ct::String sceneBase(Rig& rig)
{
    ct::String path = rig.document.sceneAbsolutePath(rig.document.scenePath());
    const std::size_t at = path.size() - strlen(EditorDocument::kSceneSuffix);
    return path.substr(0, at);
}

void testCancelBake(Rig& rig, World& world)
{
    printf("cancelling in the middle of a bake leaves nothing behind\n");
    LightmapGI* gi = lightmapOf(rig);
    CHECK(gi != nullptr);
    if (!gi) return;
    LightmapBaker::Settings& settings = rig.baker.settings();
    settings.resolution = 512;
    settings.bake.shadowResolution = 4096;
    settings.bake.sampleCount = 48;
    settings.bake.sunAngularRadius = 3.0f;

    const std::size_t undoBefore = rig.document.undoRedo().undoCount();
    CHECK(rig.lightmap.unwrapButton()->isEnabled());
    clickWidget(rig, rig.lightmap.unwrapButton());
    CHECK(rig.baker.busy() || rig.baker.summary().ok);
    CHECK(rig.waitIdle());
    CHECK(rig.baker.summary().ok && rig.baker.summary().text == "Unwrapped 2 meshes");
    CHECK(rig.document.undoRedo().undoCount() == undoBefore + 2);
    CHECK(rig.document.undoRedo().currentActionName() == LightmapBaker::kUnwrapAction);
    CHECK(LightmapMeshIO::hasUv2(world.floor->getMesh()) &&
            LightmapMeshIO::hasUv2(world.box->getMesh()));
    const ct::String floorMesh = sceneres::pathOf(world.floor->getMesh());
    printf("  floor mesh is now %s\n", floorMesh.c_str());
    CHECK(strstr(floorMesh.c_str(), "Scenes/main_lightmap/Floor.pmesh") != nullptr);
    CHECK(exists(sceneBase(rig) + "_lightmap/Floor.pmesh") &&
            exists(sceneBase(rig) + "_lightmap/Box.pmesh"));
    CHECK(rig.lightmap.rowHasUv2(0) && rig.lightmap.rowHasUv2(1));
    CHECK(rig.lightmap.rowCount() == 2);
    CHECK(rig.lightmap.baker().lastAtlas().width > 0);
    CHECK(rig.lightmap.unwrapButton()->isEnabled());
    clickWidget(rig, rig.lightmap.unwrapButton());
    CHECK(!rig.baker.busy());
    CHECK(rig.lightmap.error() == "Every Static mesh already has a second UV set.");
    rig.frames(2);
    CHECK(rig.lightmap.statusLabel()->text() == "Every Static mesh already has a second UV set.");

    clickWidget(rig, rig.lightmap.bakeButton());
    CHECK(rig.baker.state() == LightmapBaker::State::Baking);
    rig.frames(12);
    CHECK(rig.baker.busy() && rig.baker.progress() > 0.0f);
    CHECK(rig.lightmap.cancelButton()->isEnabled());
    CHECK(!rig.lightmap.bakeButton()->isEnabled());
    CHECK(rig.lightmap.progressBar()->value() > 0.0f);
    const std::size_t undoMid = rig.document.undoRedo().undoCount();
    clickWidget(rig, rig.lightmap.cancelButton());
    CHECK(rig.waitIdle(60));
    CHECK(rig.baker.summary().text == "Cancelled" && !rig.baker.summary().ok);
    CHECK(gi->getLightData() == nullptr && gi->getLightDataPath().empty());
    CHECK(!exists(sceneBase(rig) + ".lmdata") && !exists(sceneBase(rig) + ".light.dds"));
    CHECK(rig.document.undoRedo().undoCount() == undoMid);
    rig.frames(2);
    CHECK(rig.lightmap.statusLabel()->text() == "Cancelled");
    CHECK(rig.lightmap.bakeButton()->isEnabled() && !rig.lightmap.cancelButton()->isEnabled());

    Image after;
    CHECK(capture(rig, world, after));
    dump(after, "2_unwrapped_not_baked");
}

void testBake(Rig& rig, World& world, Image& baked)
{
    printf("the bake writes the lightmap next to the scene and the viewport shows it\n");
    LightmapGI* gi = lightmapOf(rig);
    CHECK(gi != nullptr);
    if (!gi) return;
    LightmapBaker::Settings& settings = rig.baker.settings();
    settings.resolution = 256;
    settings.bake.shadowResolution = 2048;
    settings.bake.sampleCount = 1;
    settings.bake.sunAngularRadius = 0.0f;
    settings.bake.filterRadius = 0.0f;

    const std::size_t undoBefore = rig.document.undoRedo().undoCount();
    clickWidget(rig, rig.lightmap.bakeButton());
    CHECK(rig.baker.busy());
    int sawProgress = 0;
    for (int i = 0; i < 400 && rig.baker.busy(); ++i)
    {
        rig.frame();
        if (rig.baker.progress() > 0.0f && rig.baker.progress() < 1.0f) ++sawProgress;
    }
    CHECK(!rig.baker.busy() && sawProgress > 0);
    CHECK(rig.baker.summary().ok);
    printf("  %s in %.3f s\n", rig.baker.summary().text.c_str(), rig.baker.summary().seconds);
    CHECK(rig.baker.summary().text == "Baked 2 meshes at 256, 1 sample");
    CHECK(rig.document.undoRedo().undoCount() == undoBefore + 1);
    CHECK(rig.document.undoRedo().currentActionName() == LightmapBaker::kBakeAction);
    CHECK(exists(sceneBase(rig) + ".lmdata") && exists(sceneBase(rig) + ".light.dds"));
    CHECK(gi->getLightDataPath() == "Scenes/main.lmdata");
    LightmapGIData* data = gi->getLightData();
    CHECK(data != nullptr);
    if (!data) return;
    CHECK(data->getUserCount() == 2 && data->getLayerCount() == 2);
    CHECK(data->getUser(0).path == "../Floor" && data->getUser(0).sliceIndex == 0);
    CHECK(data->getUser(1).path == "../Box" && data->getUser(1).sliceIndex == 1);
    CHECK(data->getUser(0).uvScale.z == 1.0f && data->getUser(0).uvScale.w == 1.0f);
    CHECK(!data->isUsingSphericalHarmonics() && !data->hasShadowmaskTextures());
    CHECK(data->getBounds().max.y > 0.9f && data->getBounds().min.x < -3.9f);
    CHECK(rig.document.dirty());
    rig.frames(2);
    CHECK(rig.lightmap.dataLabel()->text() == "data: Scenes/main.lmdata");
    CHECK(rig.lightmap.summaryLabel()->text().ends_with(" s"));

    CHECK(capture(rig, world, baked));
    dump(baked, "3_baked_static_sun");
    expectBaked(rig, world, baked, "baked");
    printf("  baked: lit floor %d, shadowed floor %d, box top %d\n", litFloorPixel(baked, 0),
            shadowFloorPixel(baked, 0), boxTopPixel(baked, 0));
}

void testStaticAgainstDynamic(Rig& rig, World& world, const Image& baked)
{
    printf("a Static sun no longer lights the lightmapped meshes and a Dynamic one adds to them\n");
    NodeClassDB::setProperty(world.sun, "light_bake_mode",
            PropertyValue::fromInt(static_cast<int>(LightBakeMode::Dynamic)));
    Image dynamic;
    CHECK(capture(rig, world, dynamic));
    dump(dynamic, "4_baked_plus_dynamic_sun");
    printf("  static sun: lit floor %d, dynamic sun on top: %d\n", litFloorPixel(baked, 0),
            litFloorPixel(dynamic, 0));
    CHECK(litFloorPixel(dynamic, 0) > litFloorPixel(baked, 0) + 20);
    CHECK(litFloorPixel(dynamic, 2) > litFloorPixel(baked, 2) + 20);

    NodeClassDB::setProperty(world.sun, "light_bake_mode",
            PropertyValue::fromInt(static_cast<int>(LightBakeMode::Static)));
    Image again;
    CHECK(capture(rig, world, again));
    CHECK(maxDifference(baked, again) == 0);

    NodeClassDB::setProperty(world.sun, "light_bake_mode",
            PropertyValue::fromInt(static_cast<int>(LightBakeMode::Disabled)));
    Image disabled;
    CHECK(capture(rig, world, disabled));
    CHECK(maxDifference(dynamic, disabled) == 0);
    NodeClassDB::setProperty(world.sun, "light_bake_mode",
            PropertyValue::fromInt(static_cast<int>(LightBakeMode::Static)));
    rig.frames(2);
}

void testSaveReload(Rig& rig, World& world, const Image& baked)
{
    printf("saved and reloaded, the scene renders the same pixels\n");
    CHECK(rig.document.saveScene());
    CHECK(!rig.document.dirty());
    const ct::String scenePath = rig.document.scenePath();
    const ct::String text = readFile(rig.document.sceneAbsolutePath(scenePath));
    CHECK(strstr(text.c_str(), "main.lmdata") != nullptr);
    CHECK(strstr(text.c_str(), "main_lightmap/Floor.pmesh") != nullptr);
    CHECK(strstr(text.c_str(), "primitive") == nullptr);

    Image before;
    CHECK(capture(rig, world, before));
    CHECK(maxDifference(before, baked) == 0);

    CHECK(rig.document.newScene());
    rig.frames(3);
    CHECK(lightmapOf(rig) == nullptr);
    CHECK(rig.document.openScene(scenePath));
    rig.frames(4);
    World reopened = world;
    findNodes(rig.document.scene(), reopened);
    CHECK(reopened.floor && reopened.box && reopened.sun);
    LightmapGI* gi = lightmapOf(rig);
    CHECK(gi != nullptr && gi->getLightData() != nullptr);
    CHECK(reopened.sun && reopened.sun->getBakeMode() == LightBakeMode::Static);
    if (gi && gi->getLightData()) CHECK(gi->getLightData()->getUserCount() == 2);
    Image after;
    CHECK(capture(rig, reopened, after));
    dump(after, "5_reloaded");
    CHECK(maxDifference(before, after) == 0);
    world = reopened;
}

void testUndoBake(Rig& rig, World& world, const Image& baked)
{
    printf("undoing the bake removes the lightmap and redoing brings it back\n");
    LightmapGI* gi = lightmapOf(rig);
    CHECK(gi != nullptr);
    if (!gi) return;
    CHECK(rig.document.undoRedo().currentActionName() == LightmapBaker::kBakeAction);
    tap(rig, Key::Z, kModControl);
    CHECK(gi->getLightData() == nullptr && gi->getLightDataPath().empty());
    CHECK(exists(sceneBase(rig) + ".lmdata"));
    Image undone;
    CHECK(capture(rig, world, undone));
    dump(undone, "6_bake_undone");
    CHECK(maxDifference(baked, undone) > 10);
    tap(rig, Key::Y, kModControl);
    CHECK(gi->getLightData() != nullptr && gi->getLightDataPath() == "Scenes/main.lmdata");
    Image redone;
    CHECK(capture(rig, world, redone));
    CHECK(maxDifference(baked, redone) == 0);
}

void testRowButtons(Rig& rig, World& world)
{
    printf("the mesh list, Mark selected Static and the per mesh Unwrap button\n");
    CHECK(rig.lightmap.rowCount() == 2 && rig.lightmap.rowName(0) == "Floor");
    NodeClassDB::setProperty(world.box, "gi_mode",
            PropertyValue::fromInt(static_cast<int>(GiMode::Disabled)));
    rig.frames(3);
    CHECK(rig.lightmap.rowCount() == 1 && rig.baker.meshes().size() == 1);
    rig.selection.clear();
    rig.selection.add(world.box);
    rig.frames(3);
    const std::size_t undoBefore = rig.document.undoRedo().undoCount();
    clickWidget(rig, rig.lightmap.staticButton());
    CHECK(world.box->getGiMode() == GiMode::Static && rig.lightmap.rowCount() == 2);
    CHECK(rig.document.undoRedo().undoCount() == undoBefore + 1);
    CHECK(rig.document.undoRedo().currentActionName() == "Set GI Mode");
    tap(rig, Key::Z, kModControl);
    CHECK(world.box->getGiMode() == GiMode::Disabled && rig.lightmap.rowCount() == 1);
    tap(rig, Key::Y, kModControl);
    CHECK(world.box->getGiMode() == GiMode::Static && rig.lightmap.rowCount() == 2);
    clickWidget(rig, rig.lightmap.staticButton());
    CHECK(rig.lightmap.error() == "Select a mesh that is not Static yet.");

    const ct::String before = sceneres::pathOf(world.floor->getMesh());
    clickWidget(rig, rig.lightmap.rowButton(0));
    CHECK(rig.waitIdle());
    CHECK(rig.baker.summary().ok && rig.baker.summary().text == "Unwrapped 1 mesh");
    const ct::String after = sceneres::pathOf(world.floor->getMesh());
    CHECK(after != before && strstr(after.c_str(), "main_lightmap/Floor_2.pmesh") != nullptr);
    CHECK(LightmapMeshIO::hasUv2(world.floor->getMesh()));
    tap(rig, Key::Z, kModControl);
    CHECK(sceneres::pathOf(world.floor->getMesh()) == before);
}

void testUnwrapCancel(Rig& rig, World& world)
{
    printf("cancelling an unwrap leaves the mesh as it was\n");
    (void) world;
    Node* scene = rig.document.scene();
    MeshInstance3D* heavy = scene->createChild<MeshInstance3D>();
    heavy->setName("Heavy");
    heavy->setMesh(createSphereMesh(*Main::meshes(), 2.0f, 220, 220));
    heavy->setPosition(Math::Vec3(30.0f, 0.0f, 0.0f));
    rig.frames(2);
    const ct::String before = sceneres::pathOf(heavy->getMesh());
    ct::Vector<MeshInstance3D*> list;
    list.push_back(heavy);
    ct::String error;
    CHECK(rig.baker.startUnwrap(list, error));
    CHECK(rig.baker.state() == LightmapBaker::State::Unwrapping);
    CHECK(!rig.baker.startUnwrap(list, error) && !error.empty());
    CHECK(!rig.baker.startBake(error) && !error.empty());
    rig.baker.cancel();
    CHECK(rig.waitIdle(2000));
    CHECK(rig.baker.summary().text == "Cancelled" && !rig.baker.summary().ok);
    CHECK(sceneres::pathOf(heavy->getMesh()) == before);
    CHECK(!LightmapMeshIO::hasUv2(heavy->getMesh()));
    CHECK(!exists(sceneBase(rig) + "_lightmap/Heavy.pmesh"));

    CHECK(rig.baker.startUnwrap(list, error));
    CHECK(rig.waitIdle(6000));
    CHECK(rig.baker.summary().ok);
    CHECK(LightmapMeshIO::hasUv2(heavy->getMesh()));
    CHECK(exists(sceneBase(rig) + "_lightmap/Heavy.pmesh"));
    CHECK(LightmapMeshIO::vertexCount(heavy->getMesh()) > 0);
    heavy->queueFree();
    rig.frames(2);
}

void testBakeMakesSunStatic(Rig& rig, World& world)
{
    printf("Bake with a Dynamic sun sets it to Static and bakes\n");
    CHECK(lightmapOf(rig) != nullptr);
    CHECK(NodeClassDB::setProperty(world.sun, "light_bake_mode",
            PropertyValue::fromInt(static_cast<int>(LightBakeMode::Dynamic))));
    CHECK(!rig.baker.sunIsStatic());
    LightmapBaker::Settings& settings = rig.baker.settings();
    settings.resolution = 256;
    settings.bake.sampleCount = 1;
    clickWidget(rig, rig.lightmap.bakeButton());
    CHECK(rig.lightmap.error().empty());
    CHECK(rig.baker.sunIsStatic() && world.sun->getBakeMode() == LightBakeMode::Static);
    for (int i = 0; i < 400 && rig.baker.busy(); ++i) rig.frame();
    CHECK(!rig.baker.busy() && rig.baker.summary().ok);
}

void testShellPanel(Rig& rig)
{
    printf("the Lightmap panel is part of the shell and old layouts get it\n");
    CHECK(strcmp(EditorShell::panelName(EditorShell::kLightmapPanel), "Lightmap") == 0);
    CHECK(EditorShell::kPanelCount == 10);
    CHECK(rig.shell.panel(EditorShell::kLightmapPanel) != nullptr);
    CHECK(rig.lightmap.attached());
    const rt::String layout = rig.shell.layout();
    CHECK(strstr(layout.c_str(), "Lightmap") != nullptr);
    CHECK(strstr(layout.c_str(), "{Inspector,Material,Lightmap") != nullptr);

    CHECK(rig.shell.setLayout("H200[{Hierarchy}|H780[V450[{Viewport}|{Console,Assets/1}]|"
                              "{Inspector,Material/1}]]"));
    rig.frames(3);
    CHECK(strstr(rig.shell.layout().c_str(), "Lightmap") != nullptr);
    CHECK(rig.shell.panel(EditorShell::kMaterialPanel)->isVisible());
    CHECK(!rig.shell.panel(EditorShell::kInspectorPanel)->isVisible());
    rig.shell.showPanel(EditorShell::kLightmapPanel);
    rig.frames(3);
    CHECK(rig.shell.panel(EditorShell::kLightmapPanel)->isVisible());
    CHECK(rig.shell.setLayout("H200[{Hierarchy}|H780[V450[{Scene,Game/0}|{Console,Assets/1}]|"
                              "{Inspector,Material/0}]]"));
    rig.frames(3);
}

} // namespace

int main(int argc, char** argv)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor lightmap test";
    desc.driver = useVulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;

    removeTree(testRoot());
    CHECK(fs_create_directory(testRoot().c_str()));
    CHECK(fs_create_directory((testRoot() + "/projects").c_str()));

    SceneTree tree;
    CHECK(Main::start(&tree));
    tree.setAutoAcceptQuit(false);
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    RenderingServer& server = *Main::rendering();
    server.viewportSetActive(window->getViewportId(), false);

    Node3D* scene = tree.getRoot()->createChild<Node3D>();
    scene->setName("Scene");

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        EditorShell shell(host);
        CHECK(host.init());
        CHECK(shell.build());

        EditorSelection selection(tree);
        SceneTreePanel hierarchy(tree, selection);
        InspectorPanel inspector(tree, selection);
        CHECK(hierarchy.attach(shell.panel(0)));
        CHECK(inspector.attach(shell.panel(2)));
        hierarchy.setEditedScene(scene);

        EditorSettings settings;
        Project project;
        EditorDocument document(tree, selection, hierarchy, project, settings);
        document.setSettingsPath(testRoot() + "/settings.json");
        inspector.setChangeListener(&document);
        inspector.setProject(&project);
        hierarchy.setChangeListener(&document);
        ProjectUi ui(document);
        CHECK(ui.attach(shell.toolBar()));

        EditorViewportPanel panel(server);
        panel.bindEditor(tree, selection);
        panel.setChangeListener(&document);
        EditorGizmo* gizmo = panel.gizmo();
        CHECK(gizmo != nullptr);
        if (gizmo)
            gizmo->setRootProvider([&document]() -> Node* { return document.scene(); });
        CHECK(panel.attach(shell.panel(1)));
        panel.setScenario(window->getScenario());

        HierarchyActions actions(tree, document, selection, hierarchy);
        CHECK(actions.attach());
        EditorMenus menus(shell, ui, panel, actions);
        CHECK(menus.attach());
        EditorStatusBar status(document, selection, server);
        CHECK(status.attach(shell.statusBar()));
        status.setGizmo(gizmo);

        LightmapBaker baker(document, selection, actions);
        LightmapPanel lightmap(baker, document, selection);
        CHECK(lightmap.attach(shell.panel(EditorShell::kLightmapPanel)));
        CHECK(!lightmap.attach(shell.panel(EditorShell::kLightmapPanel)));
        menus.setLightmapPanel(&lightmap);

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, panel,
            actions, menus, status, baker, lightmap };
        CHECK(shell.setLayout("H200[{Hierarchy}|H780[V450[{Viewport}|{Console,Assets/1}]|"
                              "{Inspector,Material/1}]]"));
        rig.frames(4);

        testShellPanel(rig);
        testUnwrapper(rig);
        testUnwrapJob(rig);
        testBakePass(rig);
        testPenumbra(rig);
        testLargeFloor(rig);
        testPassRefusals(rig);

        World world;
        testSceneSetup(rig, world);
        testPanelFlow(rig, world);
        testCancelBake(rig, world);
        Image baked;
        testBake(rig, world, baked);
        testStaticAgainstDynamic(rig, world, baked);
        testUndoBake(rig, world, baked);
        testBakeMakesSunStatic(rig, world);
        testSaveReload(rig, world, baked);
        testRowButtons(rig, world);
        testUnwrapCancel(rig, world);

        rig.frames(3);
        host.shutdown();
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);
    removeTree(testRoot());

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
