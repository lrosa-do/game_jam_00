#include "Check.h"
#include "EditorIconCheck.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/EditorAbout.h"
#include "prisma3d/editor/EditorCamera.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorGizmo.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorNodeIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/DockPanel.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

using namespace prisma3d;
namespace rt = ig::retained;
using Math::Vec3;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const float kPi = 3.14159265358979323846f;
const float kEps = 1e-3f;

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_gizmo_test_vulkan" : "/editor_gizmo_test";
    return root;
}

bool near(double a, double b, double eps = kEps)
{
    return fabs(a - b) <= eps;
}

bool nearVec(const Vec3& a, const Vec3& b, float eps = kEps)
{
    return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps);
}

InputEvent motion(int x, int y, unsigned mask = 0)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = 0;
    event.mouseMotion.relativeY = 0;
    event.mouseMotion.buttonMask = mask;
    return event;
}

InputEvent button(int x, int y, MouseButton which, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = which;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = modifiers;
    return event;
}

InputEvent key(Key which, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = which;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = modifiers;
    return event;
}

const unsigned kLeftMask = 1u << static_cast<int>(MouseButton::Left);

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
    EditorMenus& menus;
    EditorStatusBar& status;

    void send(const InputEvent& event)
    {
        if (ui.handleEvent(event)) return;
        host.handleEvent(event);
        panel.handleEvent(event);
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
        menus.update();
        status.update(delta);
        host.frame(delta, true);
        Main::rendering()->endFrame(true);
    }

    void frames(int count)
    {
        for (int i = 0; i < count; ++i) frame();
    }
};

struct Cam
{
    Math::Mat4 transform;
    EditorCamera::Projection projection;
    float width;
    float height;
    float originX;
    float originY;

    Vec3 position() const
    {
        return transform.GetTranslation();
    }

    Vec3 forward() const
    {
        return Vec3(-transform.col2.x, -transform.col2.y, -transform.col2.z).Normalized();
    }

    Vec3 ray(float x, float y) const
    {
        const float aspect = width / height;
        const float halfY = projection.zNear * tanf(projection.fov * kPi / 360.0f);
        const float halfX = halfY * aspect;
        const Vec3 local(((x / width) * 2.0f - 1.0f) * halfX,
                ((1.0f - (y / height)) * 2.0f - 1.0f) * halfY, -projection.zNear);
        return transform.TransformVector(local.Normalized()).Normalized();
    }

    Math::Vec2 project(const Vec3& world) const
    {
        const Vec3 local = transform.InverseAffine().TransformPoint(world);
        const float aspect = width / height;
        const float tanHalf = tanf(projection.fov * kPi / 360.0f);
        const float depth = -local.z;
        const float ndcX = local.x / (depth * tanHalf * aspect);
        const float ndcY = local.y / (depth * tanHalf);
        return Math::Vec2((ndcX * 0.5f + 0.5f) * width, (-ndcY * 0.5f + 0.5f) * height);
    }

    float depthOf(const Vec3& world) const
    {
        return forward().Dot(world - position());
    }

    float gizmoScale(const Vec3& world, float size = 80.0f) const
    {
        const float pixelsPerUnit =
                height / (2.0f * depthOf(world) * tanf(projection.fov * kPi / 360.0f));
        const float factor = (height < 400.0f ? height : 400.0f) / 400.0f;
        return size / pixelsPerUnit * factor;
    }
};

Cam currentCam(const EditorViewportPanel& panel)
{
    Cam cam;
    cam.transform = panel.camera().interpToCameraTransform();
    cam.projection = panel.camera().projection();
    cam.width = static_cast<float>(panel.width());
    cam.height = static_cast<float>(panel.height());
    const rt::Rect rect = panel.view()->absoluteRect();
    cam.originX = rect.x;
    cam.originY = rect.y;
    return cam;
}

struct Pixel
{
    int x;
    int y;
    float localX;
    float localY;
};

Pixel pixelFor(const Cam& cam, const Vec3& world)
{
    const Math::Vec2 local = cam.project(world);
    Pixel pixel;
    pixel.x = static_cast<int>(floorf(cam.originX + local.x + 0.5f));
    pixel.y = static_cast<int>(floorf(cam.originY + local.y + 0.5f));
    pixel.localX = static_cast<float>(pixel.x) - cam.originX;
    pixel.localY = static_cast<float>(pixel.y) - cam.originY;
    return pixel;
}

Pixel pixelAt(const Cam& cam, int localX, int localY)
{
    Pixel pixel;
    pixel.x = static_cast<int>(floorf(cam.originX + static_cast<float>(localX) + 0.5f));
    pixel.y = static_cast<int>(floorf(cam.originY + static_cast<float>(localY) + 0.5f));
    pixel.localX = static_cast<float>(pixel.x) - cam.originX;
    pixel.localY = static_cast<float>(pixel.y) - cam.originY;
    return pixel;
}

Vec3 hitPlane(const Vec3& origin, const Vec3& direction, const Vec3& normal, const Vec3& point)
{
    const float t = normal.Dot(point - origin) / normal.Dot(direction);
    return origin + direction * t;
}

Vec3 axisPlaneNormal(const Vec3& axis, const Vec3& cameraNormal)
{
    return axis.Cross(axis.Cross(cameraNormal)).Normalized();
}

Vec3 axisMotion(const Cam& cam, const Vec3& center, const Vec3& axis, const Pixel& from,
        const Pixel& to)
{
    const Vec3 normal = axisPlaneNormal(axis, cam.forward());
    const Vec3 a = hitPlane(cam.position(), cam.ray(from.localX, from.localY), normal, center);
    const Vec3 b = hitPlane(cam.position(), cam.ray(to.localX, to.localY), normal, center);
    return axis * axis.Dot(b - a);
}

float signedAngle(const Vec3& from, const Vec3& to, const Vec3& axis)
{
    const Vec3 cross = from.Cross(to);
    const float angle = atan2f(cross.Length(), from.Dot(to));
    return cross.Dot(axis) < 0.0f ? -angle : angle;
}

Vec3 rotateAbout(const Vec3& v, const Vec3& axis, float angle)
{
    const Vec3 k = axis.Normalized();
    return v * cosf(angle) + k.Cross(v) * sinf(angle) + k * (k.Dot(v) * (1.0f - cosf(angle)));
}

bool readTexel(TextureId texture, int x, int y, unsigned char* out)
{
    prisma::RenderTarget source;
    source.texture = Main::textures()->gpu(texture);
    prisma::Rect area;
    area.x = x;
    area.y = y;
    area.width = 1;
    area.height = 1;
    return Main::device()->driver()->readPixels(source, area, out);
}

enum Dominant
{
    kRed,
    kGreen,
    kBlue
};

bool dominant(const unsigned char* texel, Dominant which, int margin = 40)
{
    switch (which)
    {
        case kRed:
            return texel[0] > texel[1] + margin && texel[0] > texel[2] + margin;
        case kGreen:
            return texel[1] > texel[0] + margin && texel[1] > texel[2] + margin;
        default:
            return texel[2] > texel[0] + margin && texel[2] > texel[1] + margin;
    }
}

bool dominantNear(Rig& rig, const Pixel& pixel, Dominant which, int radius = 2, int margin = 40)
{
    const int h = static_cast<int>(rig.panel.height());
    const int w = static_cast<int>(rig.panel.width());
    const rt::Rect rect = rig.panel.view()->absoluteRect();
    const int lx = pixel.x - static_cast<int>(floorf(rect.x + 0.5f));
    const int ly = pixel.y - static_cast<int>(floorf(rect.y + 0.5f));
    for (int dy = -radius; dy <= radius; ++dy)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            const int x = lx + dx;
            const int y = h - 1 - (ly + dy);
            if (x < 0 || y < 0 || x >= w || y >= h) continue;
            unsigned char texel[4] = { 0, 0, 0, 0 };
            if (readTexel(rig.panel.texture(), x, y, texel) && dominant(texel, which, margin))
                return true;
        }
    }
    return false;
}

void drag(Rig& rig, const Pixel& from, const Pixel& through, const Pixel& to,
        unsigned modifiers = 0)
{
    rig.send(motion(from.x, from.y));
    rig.frame();
    rig.send(button(from.x, from.y, MouseButton::Left, true, modifiers));
    rig.frame();
    rig.send(motion(through.x, through.y, kLeftMask));
    rig.frame();
    rig.send(motion(to.x, to.y, kLeftMask));
    rig.frame();
}

void release(Rig& rig, const Pixel& at, unsigned modifiers = 0)
{
    rig.send(button(at.x, at.y, MouseButton::Left, false, modifiers));
    rig.frame();
}

void settle(Rig& rig, int count = 120)
{
    rig.frames(count);
}

void focusCamera(Rig& rig, const Vec3& target, float distance)
{
    EditorCamera::Cursor& cursor = rig.panel.camera().cursor();
    cursor.posX = target.x;
    cursor.posY = target.y;
    cursor.posZ = target.z;
    cursor.distance = distance;
    settle(rig);
}

void select(Rig& rig, Node* node)
{
    rig.selection.clear();
    if (node) rig.selection.add(node);
    rig.frames(3);
}

void resetNode(Node3D* node, const Vec3& position)
{
    node->setPosition(position);
    node->setRotationDegrees(Vec3(0.0f, 0.0f, 0.0f));
    node->setScale(Vec3(1.0f, 1.0f, 1.0f));
}

int countBright(Rig& rig, int rowFrom, int rowTo, int threshold)
{
    const int w = static_cast<int>(rig.panel.width());
    const int h = static_cast<int>(rig.panel.height());
    int count = 0;
    for (int y = rowFrom; y < rowTo && y < h; ++y)
    {
        for (int x = 0; x < w; x += 2)
        {
            unsigned char texel[4] = { 0, 0, 0, 0 };
            if (!readTexel(rig.panel.texture(), x, y, texel)) continue;
            if (texel[0] > threshold && texel[1] > threshold && texel[2] > threshold &&
                    abs(texel[0] - texel[1]) < 12)
                ++count;
        }
    }
    return count;
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor gizmo test";
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

    SceneTree tree;
    CHECK(Main::start(&tree));
    tree.setAutoAcceptQuit(false);
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    RenderingServer& server = *Main::rendering();
    server.viewportSetActive(window->getViewportId(), false);

    MeshStore& meshes = *Main::meshes();
    Node3D* scene = tree.getRoot()->createChild<Node3D>();
    scene->setName("Scene");
    const Vec3 home(2.0f, 1.5f, -1.0f);
    MeshInstance3D* box = scene->createChild<MeshInstance3D>();
    box->setName("Box");
    box->setMesh(createBoxMesh(meshes, Vec3(1.0f, 1.0f, 1.0f)));
    box->setColor(Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    box->setPosition(home);
    MeshInstance3D* other = scene->createChild<MeshInstance3D>();
    other->setName("Other");
    other->setMesh(createBoxMesh(meshes, Vec3(1.0f, 1.0f, 1.0f)));
    other->setColor(Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    other->setPosition(home + Vec3(-3.0f, 0.0f, 1.5f));

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
        CHECK(panel.gizmo() != nullptr);
        EditorGizmo& gizmo = *panel.gizmo();
        gizmo.setRootProvider([&document]() -> Node* { return document.scene(); });
        CHECK(panel.attach(shell.panel(1)));
        panel.setScenario(window->getScenario());
        CHECK(server.viewportGetDrawBatch(panel.viewportId()));

        HierarchyActions actions(tree, document, selection, hierarchy);
        CHECK(actions.attach());
        EditorMenus menus(shell, ui, panel, actions);
        CHECK(menus.attach());
        EditorStatusBar status(document, selection, server);
        CHECK(status.attach(shell.statusBar()));
        status.setGizmo(&gizmo);
        status.setSaveHandler([&ui] { ui.save(); });

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, panel, menus,
            status };
        rig.frames(4);
        CHECK(!document.dirty());

        // shell: the top bar must hold its icons
        {
            const rt::Rect bar = shell.toolBar()->absoluteRect();
            const float iconSize =
                    EditorShell::kToolBarButtonSize - EditorShell::kToolBarIconPadding * 2.0f;
            CHECK(bar.h >= EditorShell::kToolBarButtonSize + 4.0f);
            CHECK(bar.h >= iconSize + 8.0f);
            CHECK(shell.toolBar()->buttonSize() == EditorShell::kToolBarButtonSize);
            CHECK(iconSize == 16.0f);
            CHECK(isAtlasIcon(EditorShell::resetLayoutIcon()));
            for (int i = 0; i < ProjectUi::kButtonCount; ++i)
            {
                const rt::IconId icon = ProjectUi::buttonIcon(static_cast<ProjectUi::Button>(i));
                CHECK(isAtlasIcon(icon));
                for (int j = 0; j < i; ++j)
                    CHECK(icon != ProjectUi::buttonIcon(static_cast<ProjectUi::Button>(j)));
            }
            for (int i = 0; i < EditorViewportPanel::kButtonCount; ++i)
            {
                const rt::IconId icon = EditorViewportPanel::buttonIcon(
                        static_cast<EditorViewportPanel::Button>(i));
                CHECK(isAtlasIcon(icon));
                for (int j = 0; j < i; ++j)
                    CHECK(icon != EditorViewportPanel::buttonIcon(
                                          static_cast<EditorViewportPanel::Button>(j)));
            }
            CHECK(EditorViewportPanel::buttonIcon(EditorViewportPanel::kMove) ==
                    EditorIcons::get("ToolMove"));
            CHECK(EditorViewportPanel::buttonIcon(EditorViewportPanel::kRotate) ==
                    EditorIcons::get("ToolRotate"));
            CHECK(EditorViewportPanel::buttonIcon(EditorViewportPanel::kScale) ==
                    EditorIcons::get("ToolScale"));
            CHECK(bar.x == 0.0f && bar.y >= shell.menuBar()->absoluteRect().y +
                                                    shell.menuBar()->absoluteRect().h - 0.5f);
            CHECK(shell.menuBar()->absoluteRect().y == 0.0f);
            CHECK(shell.menuBar()->absoluteRect().w == static_cast<float>(kWidth));
            CHECK(shell.statusBar()->absoluteRect().y + shell.statusBar()->absoluteRect().h >=
                    static_cast<float>(kHeight) - 1.0f);
            CHECK(shell.statusBar()->absoluteRect().y >=
                    shell.dock()->absoluteRect().y + shell.dock()->absoluteRect().h - 1.0f);
            CHECK(shell.statusBar()->absoluteRect().w == static_cast<float>(kWidth));
            CHECK(shell.dock()->absoluteRect().y >= bar.y + bar.h - 0.5f);
        }

        // viewport strip
        {
            rt::ToolBar* strip = panel.toolBar();
            CHECK(strip != nullptr &&
                    panel.view()->absoluteRect().y >=
                            strip->absoluteRect().y + strip->absoluteRect().h - 0.5f);
            CHECK(strip->absoluteRect().h >= strip->buttonSize() + 4.0f);
            for (int i = 0; i < EditorViewportPanel::kButtonCount; ++i)
                CHECK(panel.buttonId(static_cast<EditorViewportPanel::Button>(i)) >= 0);
            CHECK(strcmp(EditorViewportPanel::buttonTooltip(EditorViewportPanel::kMove),
                          "Move Mode (W)") == 0);
            CHECK(strcmp(EditorViewportPanel::buttonTooltip(EditorViewportPanel::kSelect),
                          "Select Mode (V)") == 0);
            CHECK(strcmp(EditorViewportPanel::buttonTooltip(EditorViewportPanel::kLocalSpace),
                          "Use Local Space (T)") == 0);
            CHECK(strcmp(EditorViewportPanel::buttonTooltip(EditorViewportPanel::kSnap),
                          "Use Snap (Y)") == 0);
            CHECK(strcmp(EditorViewportPanel::buttonTooltip(EditorViewportPanel::kGrid),
                          "Show Grid") == 0);
        }

        focusCamera(rig, home, 7.0f);
        const Cam cam0 = currentCam(panel);
        CHECK(panel.width() > 600 && panel.height() > 350);
        // no selection: no gizmo, nothing of the axis colors around the node
        {
            CHECK(selection.selectedNodes().empty());
            CHECK(!gizmo.visible());
            CHECK(gizmo.highlight() == -1);
            const float s = cam0.gizmoScale(home);
            for (int axis = 0; axis < 3; ++axis)
            {
                Vec3 direction(0.0f, 0.0f, 0.0f);
                direction[axis] = 1.0f;
                const Pixel pixel = pixelFor(cam0, home + direction * (s * 1.45f));
                CHECK(!dominantNear(rig, pixel, static_cast<Dominant>(axis), 3, 25));
            }
        }

        // grid and axes
        {
            const int h = static_cast<int>(panel.height());
            const int gridOn = countBright(rig, h / 5, h * 2 / 5, 52);
            panel.grid().setGridEnabled(false);
            rig.frames(2);
            const int gridOff = countBright(rig, h / 5, h * 2 / 5, 52);
            panel.grid().setGridEnabled(true);
            rig.frames(2);
            CHECK(gridOn > 200);
            CHECK(gridOff < gridOn / 8);
            CHECK(panel.grid().lineCount() > 20);

            Pixel onAxis = pixelFor(cam0, Vec3(4.0f, 0.0f, 0.0f));
            bool found = false;
            for (float x = 2.0f; x < 40.0f && !found; x += 0.5f)
            {
                onAxis = pixelFor(cam0, Vec3(x, 0.0f, 0.0f));
                const Math::Vec2 local = cam0.project(Vec3(x, 0.0f, 0.0f));
                if (local.x > 40 && local.x < cam0.width - 40 && local.y > 40 &&
                        local.y < cam0.height - 40)
                    found = true;
            }
            CHECK(found);
            CHECK(dominantNear(rig, onAxis, kRed, 2, 40));
            panel.grid().setOriginEnabled(false);
            rig.frames(2);
            CHECK(!dominantNear(rig, onAxis, kRed, 2, 40));
            panel.grid().setOriginEnabled(true);
            rig.frames(2);
            CHECK(dominantNear(rig, onAxis, kRed, 2, 40));

            Pixel onZ = pixelFor(cam0, Vec3(0.0f, 0.0f, 4.0f));
            found = false;
            for (float z = 2.0f; z < 40.0f && !found; z += 0.5f)
            {
                onZ = pixelFor(cam0, Vec3(0.0f, 0.0f, z));
                const Math::Vec2 local = cam0.project(Vec3(0.0f, 0.0f, z));
                if (local.x > 40 && local.x < cam0.width - 40 && local.y > 40 &&
                        local.y < cam0.height - 40)
                    found = true;
            }
            if (found) CHECK(dominantNear(rig, onZ, kBlue, 2, 40));
        }

        // select by clicking: Box, then Other, then the empty space
        {
            const Pixel onBox = pixelFor(cam0, home);
            rig.send(motion(onBox.x, onBox.y));
            rig.frame();
            rig.send(button(onBox.x, onBox.y, MouseButton::Left, true));
            rig.frame();
            CHECK(selection.selectedNodes().empty());
            rig.send(button(onBox.x, onBox.y, MouseButton::Left, false));
            rig.frame();
            CHECK(selection.selectedNodes().size() == 1 && selection.isSelected(box));
            CHECK(gizmo.visible());

            const Pixel onOther = pixelFor(cam0, other->getGlobalPosition());
            CHECK(gizmo.pick(onOther.localX, onOther.localY) == other);
            CHECK(gizmo.pick(onBox.localX, onBox.localY) == box);
            rig.send(motion(onOther.x, onOther.y));
            rig.send(button(onOther.x, onOther.y, MouseButton::Left, true));
            rig.send(button(onOther.x, onOther.y, MouseButton::Left, false));
            rig.frames(2);
            CHECK(selection.selectedNodes().size() == 1 && selection.isSelected(other));
            CHECK(inspector.inspected() == other);

            const Pixel empty = pixelAt(cam0, 40, 120);
            CHECK(gizmo.pick(empty.localX, empty.localY) == nullptr);
            rig.send(motion(empty.x, empty.y));
            rig.send(button(empty.x, empty.y, MouseButton::Left, true));
            rig.send(button(empty.x, empty.y, MouseButton::Left, false));
            rig.frames(2);
            CHECK(selection.selectedNodes().empty());
            CHECK(!gizmo.visible());

            rig.send(motion(onBox.x, onBox.y));
            rig.send(button(onBox.x, onBox.y, MouseButton::Left, true));
            rig.send(button(onBox.x, onBox.y, MouseButton::Left, false));
            rig.frames(2);
            CHECK(selection.isSelected(box));
            rig.send(motion(onOther.x, onOther.y));
            rig.send(button(onOther.x, onOther.y, MouseButton::Left, true, kModShift));
            rig.send(button(onOther.x, onOther.y, MouseButton::Left, false, kModShift));
            rig.frames(2);
            CHECK(selection.selectedNodes().size() == 2 && selection.isSelected(box) &&
                    selection.isSelected(other));
            rig.send(motion(empty.x, empty.y));
            rig.send(button(empty.x, empty.y, MouseButton::Left, true));
            rig.send(button(empty.x, empty.y, MouseButton::Left, false));
            rig.frames(2);
            CHECK(selection.selectedNodes().empty());
            CHECK(!document.dirty());
        }

        // icons of the nodes that have no shape: seen, clicked, hidden with the node
        {
            const EditorCamera::Cursor savedCursor = panel.camera().cursor();
            focusCamera(rig, home, 14.0f);
            const Cam icam = currentCam(panel);
            const Vec3 right = icam.transform.col0.xyz().Normalized();
            const Vec3 up = icam.transform.col1.xyz().Normalized();
            const Vec3 spots[5] = { home + up * 3.0f + right * -3.0f,
                home + up * 3.0f + right * -1.0f, home + up * 3.0f + right * 1.0f,
                home + up * 3.0f + right * 3.0f, home + up * -3.5f + right * -1.0f };

            struct Patch
            {
                int value[169];
            };
            auto capture = [&](const Pixel& pixel)
            {
                Patch patch;
                const int h = static_cast<int>(panel.height());
                const int w = static_cast<int>(panel.width());
                const rt::Rect rect = panel.view()->absoluteRect();
                const int lx = pixel.x - static_cast<int>(floorf(rect.x + 0.5f));
                const int ly = pixel.y - static_cast<int>(floorf(rect.y + 0.5f));
                for (int dy = -6; dy <= 6; ++dy)
                {
                    for (int dx = -6; dx <= 6; ++dx)
                    {
                        const int x = lx + dx;
                        const int y = h - 1 - (ly + dy);
                        int sum = 0;
                        unsigned char texel[4] = { 0, 0, 0, 0 };
                        if (x >= 0 && y >= 0 && x < w && y < h &&
                                readTexel(panel.texture(), x, y, texel))
                            sum = texel[0] + texel[1] + texel[2];
                        patch.value[(dy + 6) * 13 + dx + 6] = sum;
                    }
                }
                return patch;
            };
            auto difference = [](const Patch& a, const Patch& b)
            {
                int total = 0;
                for (int i = 0; i < 169; ++i) total += abs(a.value[i] - b.value[i]);
                return total;
            };

            Patch before[5];
            for (int i = 0; i < 5; ++i) before[i] = capture(pixelFor(icam, spots[i]));

            const std::uint32_t icons0 = panel.nodeIcons().iconCount();
            const std::uint32_t markers0 = panel.nodeIcons().markerCount();
            DirectionalLight3D* sun = scene->createChild<DirectionalLight3D>();
            OmniLight3D* lamp = scene->createChild<OmniLight3D>();
            SpotLight3D* spot = scene->createChild<SpotLight3D>();
            Camera3D* camera = scene->createChild<Camera3D>();
            Node3D* empty3d = scene->createChild<Node3D>();
            sun->setName("IconSun");
            lamp->setName("IconLamp");
            spot->setName("IconSpot");
            camera->setName("IconCamera");
            empty3d->setName("IconEmpty");
            Node3D* nodes[5] = { sun, lamp, spot, camera, empty3d };
            for (int i = 0; i < 5; ++i)
                nodes[i]->setPosition(spots[i] - scene->getGlobalPosition());
            rig.frames(4);

            CHECK(panel.nodeIcons().loaded());
            CHECK(panel.nodeIcons().iconCount() == icons0 + 4);
            CHECK(panel.nodeIcons().markerCount() == markers0 + 1);
            CHECK(EditorNodeIcons::kindOf(*sun) == EditorNodeIcons::Kind::Icon);
            CHECK(EditorNodeIcons::kindOf(*empty3d) == EditorNodeIcons::Kind::Marker);
            CHECK(EditorNodeIcons::kindOf(*box) == EditorNodeIcons::Kind::None);
            CHECK(strcmp(EditorNodeIcons::iconName(*lamp), "GizmoLight") == 0);
            CHECK(strcmp(EditorNodeIcons::iconName(*spot), "GizmoSpotLight") == 0);
            CHECK(strcmp(EditorNodeIcons::iconName(*camera), "GizmoCamera3D") == 0);
            for (int i = 0; i < 4; ++i)
                CHECK(panel.nodeIcons().hasIcon(EditorNodeIcons::iconName(*nodes[i])));

            for (int i = 0; i < 5; ++i)
            {
                const Pixel at = pixelFor(icam, spots[i]);
                CHECK(difference(capture(at), before[i]) > 600);
                CHECK(gizmo.pick(at.localX, at.localY) == nodes[i]);
            }

            const Pixel off = pixelFor(icam, spots[0] - right * 2.0f);
            CHECK(gizmo.pick(off.localX, off.localY) == nullptr);

            for (int i = 0; i < 5; ++i)
            {
                selection.clear();
                rig.frames(2);
                Vec3 target = spots[i];
                if (i == 4) target = spots[i] + Vec3(0.12f, 0.0f, 0.0f);
                const Pixel at = pixelFor(icam, target);
                rig.send(motion(at.x, at.y));
                rig.frame();
                rig.send(button(at.x, at.y, MouseButton::Left, true));
                rig.frame();
                rig.send(button(at.x, at.y, MouseButton::Left, false));
                rig.frames(3);
                CHECK(selection.selectedNodes().size() == 1 && selection.isSelected(nodes[i]));
                CHECK(inspector.inspected() == nodes[i]);
            }

            {
                selection.clear();
                rig.frames(2);
                const Pixel at = pixelFor(icam, spots[0]);
                sun->setVisible(false);
                camera->setVisible(false);
                rig.frames(3);
                CHECK(panel.nodeIcons().iconCount() == icons0 + 2);
                CHECK(gizmo.pick(at.localX, at.localY) == nullptr);
                CHECK(difference(capture(at), before[0]) < 100);
                sun->setVisible(true);
                camera->setVisible(true);
                rig.frames(3);
                CHECK(panel.nodeIcons().iconCount() == icons0 + 4);
            }

            {
                const Pixel at = pixelFor(icam, spots[2]);
                const float distance = EditorNodeIcons::pickDistance(gizmo.view(), *spot,
                        EditorNodeIcons::Settings(), at.localX, at.localY);
                CHECK(distance > 0.0f);
                const EditorNodeIcons::Settings defaults;
                const float depth = EditorNodeIcons::billboardScale(gizmo.view(), spots[2]);
                const float halfPixels = defaults.iconScale * 0.5f * gizmo.view().height /
                                         tanf(gizmo.view().projection.fov * kPi / 360.0f);
                CHECK(depth > 1.0f);
                const Pixel inside = pixelAt(icam, static_cast<int>(at.localX + halfPixels - 3.0f),
                        static_cast<int>(at.localY));
                const Pixel outside = pixelAt(icam, static_cast<int>(at.localX + halfPixels + 4.0f),
                        static_cast<int>(at.localY));
                CHECK(gizmo.pick(inside.localX, inside.localY) == spot);
                CHECK(gizmo.pick(outside.localX, outside.localY) == nullptr);
            }

            selection.clear();
            for (int i = 0; i < 5; ++i) nodes[i]->queueFree();
            rig.frames(3);
            CHECK(panel.nodeIcons().iconCount() == icons0);
            CHECK(panel.nodeIcons().markerCount() == markers0);
            panel.camera().cursor() = savedCursor;
            settle(rig);
        }

        // the gizmo of the selected node
        select(rig, box);
        CHECK(gizmo.visible());
        box->setVisible(false);
        rig.frames(2);
        CHECK(!gizmo.visible());
        box->setVisible(true);
        rig.frames(2);
        CHECK(gizmo.visible());
        {
            Node3D* hiddenParent = scene->createChild<Node3D>();
            hiddenParent->setName("HiddenParent");
            Node3D* below = hiddenParent->createChild<Node3D>();
            below->setName("Below");
            hiddenParent->setVisible(false);
            select(rig, below);
            CHECK(!gizmo.visible());
            hiddenParent->setVisible(true);
            rig.frames(2);
            CHECK(gizmo.visible());
            select(rig, box);
            hiddenParent->queueFree();
            rig.frames(2);
            CHECK(gizmo.visible());
        }
        const float scale = cam0.gizmoScale(home);
        CHECK(near(gizmo.scale(), scale, scale * 1e-3));
        CHECK(nearVec(gizmo.transform().GetTranslation(), home));
        CHECK(gizmo.tool() == EditorGizmo::Tool::Transform);
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                Vec3 direction(0.0f, 0.0f, 0.0f);
                direction[axis] = 1.0f;
                const Pixel pixel = pixelFor(cam0, home + direction * (scale * 1.45f));
                CHECK(dominantNear(rig, pixel, static_cast<Dominant>(axis), 2, 40));
            }

            Vec3 toCamera = cam0.position() - home;
            toCamera.y = 0.0f;
            toCamera = toCamera.Normalized();
            const Pixel ring = pixelFor(cam0, home + toCamera * (scale * EditorGizmo::kCircleSize));
            CHECK(dominantNear(rig, ring, kGreen, 2, 40));
            Vec3 away = toCamera * -1.0f;
            const Pixel farRing = pixelFor(cam0, home + away * (scale * EditorGizmo::kCircleSize));
            CHECK(!dominantNear(rig, farRing, kGreen, 0, 40) ||
                    !dominantNear(rig, farRing, kGreen, 0, 80));

            const Vec3 dir = (home - cam0.position()).Normalized();
            float sign[3];
            for (int j = 0; j < 3; ++j)
            {
                Vec3 axisVector(0.0f, 0.0f, 0.0f);
                axisVector[j] = 1.0f;
                sign[j] = axisVector.Dot(dir) > 1e-5f ? -1.0f : 1.0f;
            }
            for (int i = 0; i < 3; ++i)
            {
                Vec3 a(0.0f, 0.0f, 0.0f);
                Vec3 b(0.0f, 0.0f, 0.0f);
                a[(i + 1) % 3] = sign[(i + 1) % 3];
                b[(i + 2) % 3] = sign[(i + 2) % 3];
                const Vec3 grabber =
                        home + (a + b) * (scale * (EditorGizmo::kPlaneDst +
                                                          EditorGizmo::kPlaneSize * 0.5f));
                const Pixel pixel = pixelFor(cam0, grabber);
                CHECK(dominantNear(rig, pixel, static_cast<Dominant>(i), 1, 40));
            }
        }

        // hover highlight
        {
            const Pixel onArrowY = pixelFor(cam0, home + Vec3(0.0f, 1.0f, 0.0f) * (scale * 1.575f));
            rig.send(motion(onArrowY.x, onArrowY.y));
            rig.frame();
            CHECK(gizmo.highlight() == EditorGizmo::kHighlightMove + 1);
            const Pixel onArrowX = pixelFor(cam0, home + Vec3(1.0f, 0.0f, 0.0f) * (scale * 1.575f));
            rig.send(motion(onArrowX.x, onArrowX.y));
            rig.frame();
            CHECK(gizmo.highlight() == EditorGizmo::kHighlightMove);
            const Pixel nowhere = pixelAt(cam0, 60, 140);
            rig.send(motion(nowhere.x, nowhere.y));
            rig.frame();
            CHECK(gizmo.highlight() == -1);
        }

        // move along X
        const unsigned editsBefore = gizmo.editCount();
        {
            resetNode(box, home);
            rig.frames(2);
            const Vec3 axis(1.0f, 0.0f, 0.0f);
            const Pixel grab = pixelFor(cam0, home + axis * (scale * 1.575f));
            const Pixel through = pixelAt(cam0, static_cast<int>(grab.localX) + 12,
                    static_cast<int>(grab.localY) + 3);
            const Pixel to = pixelAt(cam0, static_cast<int>(grab.localX) + 60,
                    static_cast<int>(grab.localY) + 14);
            drag(rig, grab, through, to);
            CHECK(gizmo.mode() == EditorGizmo::Mode::Translate);
            CHECK(gizmo.plane() == EditorGizmo::Plane::XAxis);
            CHECK(gizmo.editing());
            const Vec3 expected = home + axisMotion(cam0, home, axis, grab, to);
            const Vec3 moved = box->getPosition();
            CHECK(nearVec(moved, expected, 2e-3f));
            CHECK(near(moved.y, home.y, 1e-5) && near(moved.z, home.z, 1e-5));
            CHECK(fabs(moved.x - home.x) > 0.2);
            release(rig, to);
            CHECK(!gizmo.editing());
            CHECK(gizmo.editCount() == editsBefore + 1);
            CHECK(document.dirty());
            rig.frames(2);
            CHECK(near(inspector.field(InspectorPanel::kPosition, 0)->value(), moved.x, 1e-3));
            CHECK(near(inspector.field(InspectorPanel::kPosition, 1)->value(), moved.y, 1e-3));
            CHECK(inspector.inspected() == box);
            CHECK(nearVec(gizmo.transform().GetTranslation(), moved));
        }

        // move on the XZ plane
        {
            resetNode(box, home);
            document.markDirty();
            rig.frames(2);
            const Vec3 dir = (home - cam0.position()).Normalized();
            const float signX = dir.x > 1e-5f ? -1.0f : 1.0f;
            const float signZ = dir.z > 1e-5f ? -1.0f : 1.0f;
            const Vec3 grabber =
                    home +
                    Vec3(signX, 0.0f, signZ) *
                            (scale * (EditorGizmo::kPlaneDst + EditorGizmo::kPlaneSize * 0.5f));
            const Pixel grab = pixelFor(cam0, grabber);
            const Pixel through = pixelAt(cam0, static_cast<int>(grab.localX) + 10,
                    static_cast<int>(grab.localY) - 4);
            const Pixel to = pixelAt(cam0, static_cast<int>(grab.localX) + 70,
                    static_cast<int>(grab.localY) - 25);
            drag(rig, grab, through, to);
            CHECK(gizmo.plane() == EditorGizmo::Plane::XZ);
            const Vec3 normal(0.0f, 1.0f, 0.0f);
            const Vec3 a =
                    hitPlane(cam0.position(), cam0.ray(grab.localX, grab.localY), normal, home);
            const Vec3 b = hitPlane(cam0.position(), cam0.ray(to.localX, to.localY), normal, home);
            const Vec3 moved = box->getPosition();
            CHECK(nearVec(moved, home + (b - a), 2e-3f));
            CHECK(fabs(moved.x - home.x) > 0.1 && fabs(moved.z - home.z) > 0.1);
            CHECK(near(moved.y, home.y, 1e-5));
            release(rig, to);
        }

        // move by dragging the body of the selected node (view plane)
        {
            resetNode(box, home);
            rig.frames(2);
            const Pixel onBody = pixelFor(cam0, home);
            const Pixel to = pixelAt(cam0, static_cast<int>(onBody.localX) + 40,
                    static_cast<int>(onBody.localY) + 20);
            rig.send(motion(onBody.x, onBody.y));
            rig.frame();
            rig.send(button(onBody.x, onBody.y, MouseButton::Left, true));
            rig.send(motion(onBody.x + 20, onBody.y + 10, kLeftMask));
            rig.send(motion(to.x, to.y, kLeftMask));
            rig.frame();
            const Vec3 normal = cam0.forward();
            const Vec3 a =
                    hitPlane(cam0.position(), cam0.ray(onBody.localX, onBody.localY), normal, home);
            const Vec3 b = hitPlane(cam0.position(), cam0.ray(to.localX, to.localY), normal, home);
            CHECK(gizmo.plane() == EditorGizmo::Plane::View);
            CHECK(nearVec(box->getPosition(), home + (b - a), 2e-3f));
            release(rig, to);
        }

        // snap with Ctrl and with the snap switch
        {
            resetNode(box, home);
            rig.frames(2);
            const Vec3 axis(1.0f, 0.0f, 0.0f);
            const Pixel grab = pixelFor(cam0, home + axis * (scale * 1.575f));
            const Pixel through = pixelAt(cam0, static_cast<int>(grab.localX) + 8,
                    static_cast<int>(grab.localY) + 2);
            const Pixel to = pixelAt(cam0, static_cast<int>(grab.localX) + 83,
                    static_cast<int>(grab.localY) + 21);
            rig.send(key(Key::LeftControl, true, kModControl));
            CHECK(gizmo.snapActive());
            drag(rig, grab, through, to, kModControl);
            const float raw = axisMotion(cam0, home, axis, grab, to).x;
            const Vec3 moved = box->getPosition();
            CHECK(fabs(raw) > 1.0f);
            CHECK(near(moved.x, home.x + floor(raw + 0.5), 1e-4));
            CHECK(near(moved.x - floor(moved.x), 0.0, 1e-4));
            CHECK(near(moved.y, home.y, 1e-5) && near(moved.z, home.z, 1e-5));
            release(rig, to, kModControl);
            rig.send(key(Key::LeftControl, false, 0));
            CHECK(!gizmo.snapActive());

            resetNode(box, home);
            rig.frames(2);
            CHECK(!gizmo.snapEnabled());
            rig.send(key(Key::Y, true));
            rig.send(key(Key::Y, false));
            CHECK(gizmo.snapEnabled() && gizmo.snapActive());
            gizmo.settings().translateSnap = 0.5f;
            drag(rig, grab, through, to);
            const Vec3 half = box->getPosition();
            CHECK(near(half.x, home.x + floor(raw / 0.5f + 0.5f) * 0.5f, 1e-4));
            release(rig, to);
            gizmo.settings().translateSnap = 1.0f;
            rig.send(key(Key::LeftControl, true, kModControl));
            CHECK(!gizmo.snapActive());
            rig.send(key(Key::LeftControl, false, 0));
            rig.send(key(Key::Y, true));
            rig.send(key(Key::Y, false));
            CHECK(!gizmo.snapEnabled());
        }

        // cancel with Esc and with the right button
        {
            resetNode(box, home);
            rig.frames(2);
            const unsigned edits = gizmo.editCount();
            document.markDirty();
            const Vec3 axis(1.0f, 0.0f, 0.0f);
            const Pixel grab = pixelFor(cam0, home + axis * (scale * 1.575f));
            const Pixel through = pixelAt(cam0, static_cast<int>(grab.localX) + 10,
                    static_cast<int>(grab.localY));
            const Pixel to = pixelAt(cam0, static_cast<int>(grab.localX) + 50,
                    static_cast<int>(grab.localY) + 5);
            drag(rig, grab, through, to);
            CHECK(gizmo.editing() && !nearVec(box->getPosition(), home));
            rig.send(key(Key::Escape, true));
            rig.send(key(Key::Escape, false));
            rig.frame();
            CHECK(!gizmo.editing());
            CHECK(box->getPosition().x == home.x && box->getPosition().y == home.y &&
                    box->getPosition().z == home.z);
            CHECK(selection.isSelected(box));
            rig.send(button(to.x, to.y, MouseButton::Left, false));
            rig.frame();
            CHECK(gizmo.editCount() == edits);
            CHECK(box->getPosition().x == home.x);

            drag(rig, grab, through, to);
            CHECK(gizmo.editing() && !nearVec(box->getPosition(), home));
            rig.send(button(to.x, to.y, MouseButton::Right, true));
            rig.send(button(to.x, to.y, MouseButton::Right, false));
            rig.frame();
            CHECK(!gizmo.editing());
            CHECK(box->getPosition().x == home.x && box->getPosition().z == home.z);
            rig.send(button(to.x, to.y, MouseButton::Left, false));
            rig.frame();
            CHECK(gizmo.editCount() == edits);
            CHECK(!panel.camera().freelookEnabled());

            rig.send(key(Key::Escape, true));
            rig.send(key(Key::Escape, false));
            rig.frames(2);
            CHECK(selection.selectedNodes().empty());
            select(rig, box);
        }

        // two selected nodes: the gizmo sits at their average and both move
        {
            resetNode(box, home);
            const Vec3 otherHome = home + Vec3(-3.0f, 0.0f, 1.5f);
            resetNode(other, otherHome);
            selection.add(other);
            rig.frames(3);
            const Vec3 mid = (home + otherHome) * 0.5f;
            CHECK(selection.selectedNodes().size() == 2);
            CHECK(nearVec(gizmo.transform().GetTranslation(), mid));
            const float scale2 = cam0.gizmoScale(mid);
            CHECK(near(gizmo.scale(), scale2, scale2 * 1e-3));
            const Vec3 axis(1.0f, 0.0f, 0.0f);
            const Pixel grab = pixelFor(cam0, mid + axis * (scale2 * 1.575f));
            const Pixel through = pixelAt(cam0, static_cast<int>(grab.localX) + 10,
                    static_cast<int>(grab.localY) + 2);
            const Pixel to = pixelAt(cam0, static_cast<int>(grab.localX) + 45,
                    static_cast<int>(grab.localY) + 9);
            drag(rig, grab, through, to);
            const Vec3 delta = axisMotion(cam0, mid, axis, grab, to);
            CHECK(fabs(delta.x) > 0.1);
            CHECK(nearVec(box->getPosition(), home + delta, 2e-3f));
            CHECK(nearVec(other->getPosition(), otherHome + delta, 2e-3f));
            release(rig, to);
            resetNode(other, otherHome);
            select(rig, box);
            resetNode(box, home);
            rig.frames(2);
        }

        // rotate around Y
        {
            resetNode(box, home);
            rig.send(key(Key::E, true));
            rig.send(key(Key::E, false));
            rig.frame();
            CHECK(gizmo.tool() == EditorGizmo::Tool::Rotate);
            CHECK(panel.toolBar()->isItemChecked(panel.buttonId(EditorViewportPanel::kRotate)));
            CHECK(!panel.toolBar()->isItemChecked(panel.buttonId(EditorViewportPanel::kMove)));
            Vec3 toCamera = cam0.position() - home;
            toCamera.y = 0.0f;
            toCamera = toCamera.Normalized();
            const Vec3 up(0.0f, 1.0f, 0.0f);
            const float radius = scale * EditorGizmo::kCircleSize;
            const Pixel grab = pixelFor(cam0, home + toCamera * radius);
            const Pixel nudge = pixelFor(cam0, home + rotateAbout(toCamera, up, 0.05f) * radius);
            const Pixel to = pixelFor(cam0, home + rotateAbout(toCamera, up, 0.45f) * radius);

            rig.send(motion(grab.x, grab.y));
            rig.frame();
            CHECK(gizmo.highlight() == EditorGizmo::kHighlightRotate + 1);
            drag(rig, grab, nudge, to);
            CHECK(gizmo.mode() == EditorGizmo::Mode::Rotate);
            CHECK(gizmo.plane() == EditorGizmo::Plane::YAxis);

            const Vec3 normal = (home - cam0.position()).Normalized();
            const Vec3 first =
                    (hitPlane(cam0.position(), cam0.ray(nudge.localX, nudge.localY), normal, home) -
                            home)
                            .Normalized();
            const Vec3 second =
                    (hitPlane(cam0.position(), cam0.ray(to.localX, to.localY), normal, home) - home)
                            .Normalized();
            const float angle = signedAngle(first, second, up);
            CHECK(fabsf(angle) > 0.2f);
            const Vec3 degrees = box->getRotationDegrees();
            CHECK(near(degrees.y, angle * 180.0f / kPi, 0.02));
            CHECK(near(degrees.x, 0.0, 0.02) && near(degrees.z, 0.0, 0.02));
            CHECK(nearVec(box->getPosition(), home, 1e-4f));
            CHECK(nearVec(box->getScale(), Vec3(1.0f, 1.0f, 1.0f), 1e-5f));
            release(rig, to);
            rig.frames(2);
            CHECK(near(inspector.field(InspectorPanel::kRotation, 1)->value(), angle * 180.0f / kPi,
                    0.02));

            resetNode(box, home);
            rig.frames(2);
            rig.send(key(Key::LeftControl, true, kModControl));
            drag(rig, grab, nudge, to, kModControl);
            const float snapped = floorf(angle * 180.0f / kPi / 15.0f + 0.5f) * 15.0f;
            CHECK(near(box->getRotationDegrees().y, snapped, 0.02));
            CHECK(near(fmod(fabs(box->getRotationDegrees().y), 15.0), 0.0, 0.02) ||
                    near(fmod(fabs(box->getRotationDegrees().y), 15.0), 15.0, 0.02));
            release(rig, to, kModControl);
            rig.send(key(Key::LeftControl, false, 0));
        }

        // scale along X
        {
            resetNode(box, home);
            rig.send(key(Key::R, true));
            rig.send(key(Key::R, false));
            rig.frame();
            CHECK(gizmo.tool() == EditorGizmo::Tool::Scale);
            const Vec3 axis(1.0f, 0.0f, 0.0f);
            const Pixel grab = pixelFor(cam0, home + axis * (scale * 1.475f));
            const Pixel through = pixelAt(cam0, static_cast<int>(grab.localX) + 6,
                    static_cast<int>(grab.localY) + 1);
            const Pixel to = pixelAt(cam0, static_cast<int>(grab.localX) + 55,
                    static_cast<int>(grab.localY) + 9);
            rig.send(motion(grab.x, grab.y));
            rig.frame();
            CHECK(gizmo.highlight() == EditorGizmo::kHighlightScale);
            drag(rig, grab, through, to);
            CHECK(gizmo.mode() == EditorGizmo::Mode::Scale &&
                    gizmo.plane() == EditorGizmo::Plane::XAxis);
            const Vec3 normal = axisPlaneNormal(axis, cam0.forward());
            const Vec3 a =
                    hitPlane(cam0.position(), cam0.ray(grab.localX, grab.localY), normal, home);
            const Vec3 b = hitPlane(cam0.position(), cam0.ray(to.localX, to.localY), normal, home);
            const float expected = axis.Dot(b - a) / (a - home).Length();
            const Vec3 scaled = box->getScale();
            CHECK(fabs(expected) > 0.05);
            CHECK(near(scaled.x, 1.0f + expected, 2e-3));
            CHECK(near(scaled.y, 1.0f, 1e-5) && near(scaled.z, 1.0f, 1e-5));
            CHECK(nearVec(box->getPosition(), home, 1e-4f));
            release(rig, to);

            resetNode(box, home);
            rig.frames(2);
            rig.send(key(Key::LeftControl, true, kModControl));
            drag(rig, grab, through, to, kModControl);
            CHECK(near(box->getScale().x, 1.0f + floor(expected / 0.1 + 0.5) * 0.1, 2e-3));
            release(rig, to, kModControl);
            rig.send(key(Key::LeftControl, false, 0));
        }

        // local against global with a rotated node
        {
            rig.send(key(Key::W, true));
            rig.send(key(Key::W, false));
            rig.frame();
            CHECK(gizmo.tool() == EditorGizmo::Tool::Move);
            resetNode(box, home);
            box->setRotationDegrees(Vec3(0.0f, 45.0f, 0.0f));
            rig.frames(3);
            const Vec3 localX = box->getGlobalTransform().col0.xyz().Normalized();
            const Vec3 globalX(1.0f, 0.0f, 0.0f);
            CHECK(near(localX.x, cosf(kPi / 4.0f), 1e-5) &&
                    near(localX.z, -sinf(kPi / 4.0f), 1e-5));

            Pixel grab = pixelFor(cam0, home + globalX * (scale * 1.575f));
            Pixel through = pixelAt(cam0, static_cast<int>(grab.localX) + 9,
                    static_cast<int>(grab.localY) + 2);
            Pixel to = pixelAt(cam0, static_cast<int>(grab.localX) + 48,
                    static_cast<int>(grab.localY) + 11);
            CHECK(!gizmo.localSpace());
            drag(rig, grab, through, to);
            Vec3 moved = box->getPosition() - home;
            CHECK(nearVec(moved, axisMotion(cam0, home, globalX, grab, to), 2e-3f));
            CHECK(near(moved.y, 0.0, 1e-5) && near(moved.z, 0.0, 1e-5) && fabs(moved.x) > 0.1);
            CHECK(near(box->getRotationDegrees().y, 45.0, 0.01));
            release(rig, to);

            resetNode(box, home);
            box->setRotationDegrees(Vec3(0.0f, 45.0f, 0.0f));
            rig.frames(2);
            rig.send(key(Key::T, true));
            rig.send(key(Key::T, false));
            rig.frame();
            CHECK(gizmo.localSpace());
            CHECK(panel.toolBar()->isItemChecked(panel.buttonId(EditorViewportPanel::kLocalSpace)));
            CHECK(nearVec(gizmo.transform().UpperLeft3x3().col0.Normalized(), localX, 1e-4f));

            grab = pixelFor(cam0, home + localX * (scale * 1.575f));
            through = pixelAt(cam0, static_cast<int>(grab.localX) + 9,
                    static_cast<int>(grab.localY) + 2);
            to = pixelAt(cam0, static_cast<int>(grab.localX) + 48,
                    static_cast<int>(grab.localY) + 11);
            rig.send(motion(grab.x, grab.y));
            rig.frame();
            CHECK(gizmo.highlight() == EditorGizmo::kHighlightMove);
            drag(rig, grab, through, to);
            moved = box->getPosition() - home;
            CHECK(nearVec(moved, axisMotion(cam0, home, localX, grab, to), 2e-3f));
            CHECK(fabs(moved.x) > 0.05 && fabs(moved.z) > 0.05);
            CHECK(moved.Cross(localX).Length() < 1e-3f);
            release(rig, to);

            rig.send(key(Key::T, true));
            rig.send(key(Key::T, false));
            CHECK(!gizmo.localSpace());
            resetNode(box, home);
            rig.send(key(Key::Q, true));
            rig.send(key(Key::Q, false));
            CHECK(gizmo.tool() == EditorGizmo::Tool::Transform);
            rig.frames(2);
        }

        // modes through the viewport strip
        {
            rt::ToolBar* strip = panel.toolBar();
            const rt::Rect bar = strip->absoluteRect();
            const float buttonSize = strip->buttonSize();
            const float spacing = strip->spacing();
            const EditorViewportPanel::Button order[EditorViewportPanel::kButtonCount] = {
                EditorViewportPanel::kSelect, EditorViewportPanel::kTransform,
                EditorViewportPanel::kMove, EditorViewportPanel::kRotate,
                EditorViewportPanel::kScale, EditorViewportPanel::kTerrain,
                EditorViewportPanel::kLocalSpace, EditorViewportPanel::kSnap,
                EditorViewportPanel::kSnapSettings, EditorViewportPanel::kGrid,
                EditorViewportPanel::kGizmos, EditorViewportPanel::kOrthogonal
            };
            float centers[EditorViewportPanel::kButtonCount];
            float x = 2.0f;
            for (int i = 0; i < EditorViewportPanel::kButtonCount; ++i)
            {
                if (order[i] == EditorViewportPanel::kLocalSpace ||
                        order[i] == EditorViewportPanel::kGrid)
                    x += 8.0f + spacing;
                centers[order[i]] = bar.x + x + buttonSize * 0.5f;
                x += buttonSize + spacing;
            }
            const int y = static_cast<int>(bar.y + bar.h * 0.5f);
            const auto press = [&](EditorViewportPanel::Button which)
            {
                const int px = static_cast<int>(centers[which]);
                rig.send(motion(px, y));
                rig.frame();
                rig.send(button(px, y, MouseButton::Left, true));
                rig.frame();
                rig.send(button(px, y, MouseButton::Left, false));
                rig.frames(2);
            };
            press(EditorViewportPanel::kMove);
            CHECK(gizmo.tool() == EditorGizmo::Tool::Move);
            CHECK(panel.toolBar()->isItemChecked(panel.buttonId(EditorViewportPanel::kMove)));
            CHECK(!panel.toolBar()->isItemChecked(panel.buttonId(EditorViewportPanel::kTransform)));
            press(EditorViewportPanel::kRotate);
            CHECK(gizmo.tool() == EditorGizmo::Tool::Rotate);
            press(EditorViewportPanel::kScale);
            CHECK(gizmo.tool() == EditorGizmo::Tool::Scale);
            press(EditorViewportPanel::kSelect);
            CHECK(gizmo.tool() == EditorGizmo::Tool::Select);
            CHECK(selection.isSelected(box));
            {
                const Pixel pixel = pixelFor(cam0, home + Vec3(1.0f, 0.0f, 0.0f) * (scale * 1.45f));
                CHECK(!dominantNear(rig, pixel, kRed, 2, 40));
            }
            press(EditorViewportPanel::kTransform);
            CHECK(gizmo.tool() == EditorGizmo::Tool::Transform);
            press(EditorViewportPanel::kTransform);
            CHECK(gizmo.tool() == EditorGizmo::Tool::Transform);
            CHECK(panel.toolBar()->isItemChecked(panel.buttonId(EditorViewportPanel::kTransform)));

            press(EditorViewportPanel::kSnap);
            CHECK(gizmo.snapEnabled());
            CHECK(panel.toolBar()->isItemChecked(panel.buttonId(EditorViewportPanel::kSnap)));
            press(EditorViewportPanel::kSnap);
            CHECK(!gizmo.snapEnabled());
            press(EditorViewportPanel::kLocalSpace);
            CHECK(gizmo.localSpace());
            press(EditorViewportPanel::kLocalSpace);
            CHECK(!gizmo.localSpace());
            press(EditorViewportPanel::kGrid);
            CHECK(!panel.grid().gridEnabled());
            press(EditorViewportPanel::kGrid);
            CHECK(panel.grid().gridEnabled());

            press(EditorViewportPanel::kSnapSettings);
            rt::FloatWindow* dialog = panel.snapDialog();
            CHECK(dialog != nullptr);
            CHECK(gizmo.settings().translateSnap == 1.0f && gizmo.settings().rotateSnap == 15.0f &&
                    gizmo.settings().scaleSnap == 10.0f);
            if (dialog) rt::WidgetApp::instance().removeFloat(dialog);
            rig.frames(2);
            CHECK(panel.snapDialog() == nullptr);

            press(EditorViewportPanel::kOrthogonal);
            CHECK(panel.camera().isOrthogonal());
            rig.frames(2);
            press(EditorViewportPanel::kOrthogonal);
            CHECK(!panel.camera().isOrthogonal());
            settle(rig, 60);
        }

        // menus
        {
            rt::MenuBar* bar = shell.menuBar();
            CHECK(bar->menuCount() == EditorMenus::kMenuCount);
            CHECK(strcmp(bar->menuTitle(0).c_str(), "File") == 0);
            CHECK(strcmp(bar->menuTitle(1).c_str(), "Edit") == 0);
            CHECK(strcmp(bar->menuTitle(2).c_str(), "Create") == 0);
            CHECK(strcmp(bar->menuTitle(3).c_str(), "View") == 0);
            CHECK(strcmp(bar->menuTitle(4).c_str(), "Scene") == 0);
            CHECK(strcmp(bar->menuTitle(EditorMenus::kHelp).c_str(), "Help") == 0);
            CHECK(menus.menu(EditorMenus::kHelp)->actionCount() == EditorMenus::kHelpItemCount);
            CHECK(menus.action(EditorMenus::kHelp, EditorMenus::kHelpAbout)->text() ==
                    "About prisma3d editor...");
            CHECK(EditorAbout::dialog() == nullptr);
            menus.action(EditorMenus::kHelp, EditorMenus::kHelpAbout)->triggered.emit();
            CHECK(EditorAbout::dialog() != nullptr);
            rt::WidgetApp::instance().removeFloat(EditorAbout::dialog());
            CHECK(EditorAbout::dialog() == nullptr);
            CHECK(bar->menu(0) == menus.menu(EditorMenus::kFile));
            CHECK(menus.menu(EditorMenus::kFile)->actionCount() == EditorMenus::kFileItemCount);
            CHECK(menus.menu(EditorMenus::kEdit)->actionCount() == EditorMenus::kEditItemCount);
            CHECK(menus.menu(EditorMenus::kView)->actionCount() == EditorMenus::kViewItemCount);
            CHECK(menus.menu(EditorMenus::kScene)->actionCount() == EditorMenus::kSceneItemCount);
            CHECK(menus.action(EditorMenus::kFile, EditorMenus::kFileSave)->text() == "Save Scene");
            CHECK(menus.action(EditorMenus::kFile, EditorMenus::kFileSave)->shortcut() == "Ctrl+S");
            CHECK(menus.action(EditorMenus::kFile, EditorMenus::kFileNewProject)->text() ==
                    "New Project...");
            CHECK(menus.action(EditorMenus::kFile, EditorMenus::kFileRecent)->submenu() ==
                    ui.recentMenu());
            CHECK(menus.action(EditorMenus::kScene, EditorMenus::kSceneOpen)->submenu() ==
                    ui.sceneMenu());
            CHECK(menus.action(EditorMenus::kScene, EditorMenus::kSceneNew)->text() == "New Scene");
            CHECK(menus.action(EditorMenus::kView, EditorMenus::kViewGrid)->isCheckable());
            CHECK(menus.action(EditorMenus::kView, EditorMenus::kViewGrid)->isChecked());
            CHECK(menus.action(EditorMenus::kView, EditorMenus::kViewPerspective)->isChecked());
            CHECK(!menus.action(EditorMenus::kView, EditorMenus::kViewOrthogonal)->isChecked());
            CHECK(menus.action(EditorMenus::kView, EditorMenus::kViewResetLayout)->text() ==
                    "Reset Layout");

            const rt::Rect barRect = bar->absoluteRect();
            const int by = static_cast<int>(barRect.y + barRect.h * 0.5f);
            const auto openMenu = [&](EditorMenus::MenuIndex which)
            {
                rt::Menu* target = menus.menu(which);
                for (int x = static_cast<int>(barRect.x) + 4; x < 400; x += 4)
                {
                    rig.send(motion(x, by));
                    rig.frame();
                    rig.send(button(x, by, MouseButton::Left, true));
                    rig.frame();
                    rig.send(button(x, by, MouseButton::Left, false));
                    rig.frame();
                    if (rt::WidgetApp::instance().popup() == target) return true;
                }
                return false;
            };
            const rt::Theme& theme = rt::Theme::instance();
            const auto clickItem = [&](EditorMenus::MenuIndex which, int item)
            {
                rt::Menu* target = menus.menu(which);
                const rt::Rect rect = target->absoluteRect();
                float y = rect.y + 4.0f;
                for (int i = 0; i < item; ++i)
                    y += target->action(i)->isSeparator() ? 9.0f : theme.menuItemHeight;
                const int px = static_cast<int>(rect.x + 24.0f);
                const int py = static_cast<int>(y + theme.menuItemHeight * 0.5f);
                rig.send(motion(px, py));
                rig.frame();
                rig.send(button(px, py, MouseButton::Left, true));
                rig.frame();
                rig.send(button(px, py, MouseButton::Left, false));
                rig.frames(2);
            };

            CHECK(openMenu(EditorMenus::kView));
            clickItem(EditorMenus::kView, EditorMenus::kViewGrid);
            CHECK(!panel.grid().gridEnabled());
            CHECK(!menus.action(EditorMenus::kView, EditorMenus::kViewGrid)->isChecked());
            CHECK(openMenu(EditorMenus::kView));
            clickItem(EditorMenus::kView, EditorMenus::kViewGrid);
            CHECK(panel.grid().gridEnabled());
            CHECK(menus.action(EditorMenus::kView, EditorMenus::kViewGrid)->isChecked());

            CHECK(openMenu(EditorMenus::kView));
            clickItem(EditorMenus::kView, EditorMenus::kViewOrthogonal);
            CHECK(panel.camera().isOrthogonal());
            CHECK(menus.action(EditorMenus::kView, EditorMenus::kViewOrthogonal)->isChecked());
            CHECK(openMenu(EditorMenus::kView));
            clickItem(EditorMenus::kView, EditorMenus::kViewPerspective);
            CHECK(!panel.camera().isOrthogonal());
            settle(rig, 40);

            document.markDirty();
            CHECK(document.dirty());
            const unsigned serial = document.sceneSerial();
            CHECK(openMenu(EditorMenus::kScene));
            clickItem(EditorMenus::kScene, EditorMenus::kSceneNew);
            CHECK(document.sceneSerial() == serial || ui.confirming());
            if (ui.confirming())
            {
                rt::Dialog* confirm = ui.confirmDialog();
                const rt::Rect discard = confirm->btnRect_(1);
                const int px = static_cast<int>(discard.x + discard.w * 0.5f);
                const int py = static_cast<int>(discard.y + discard.h * 0.5f);
                rig.send(motion(px, py));
                rig.frame();
                rig.send(button(px, py, MouseButton::Left, true));
                rig.frame();
                rig.send(button(px, py, MouseButton::Left, false));
                rig.frames(3);
            }
            CHECK(document.sceneSerial() == serial + 1);
            CHECK(!document.dirty());
            CHECK(document.scene() != scene);
            CHECK(strcmp(document.scene()->getName().c_str(), "Scene") == 0);
            CHECK(selection.selectedNodes().empty());
            rig.frames(4);
        }

        // the status bar
        {
            CHECK(status.attached());
            const rt::Rect barRect = shell.statusBar()->absoluteRect();
            const rt::Rect dockRect = shell.dock()->absoluteRect();
            CHECK(barRect.y >= dockRect.y + dockRect.h - 1.0f);
            CHECK(barRect.w == static_cast<float>(kWidth));
            for (int i = 0; i < EditorStatusBar::kFieldCount; ++i)
            {
                const rt::Widget* field = status.field(static_cast<EditorStatusBar::Field>(i));
                CHECK(field != nullptr && field->parent() == shell.statusBar());
                const rt::Rect rect = field->absoluteRect();
                CHECK(rect.x >= barRect.x && rect.x + rect.w <= barRect.x + barRect.w + 0.5f);
                CHECK(rect.y >= barRect.y - 0.5f && rect.h > 0.0f);
            }
            CHECK(strncmp(status.text(EditorStatusBar::kScene).c_str(), "Saved", 5) == 0);
            CHECK(status.text(EditorStatusBar::kSelection) == "No selection");
            CHECK(status.text(EditorStatusBar::kTool) == "Transform · Global");
            CHECK(status.text(EditorStatusBar::kVersion) == EditorAbout::signature());
            const rt::Rect versionRect = status.field(EditorStatusBar::kVersion)->absoluteRect();
            CHECK(versionRect.x + versionRect.w >= barRect.x + barRect.w - 24.0f);
            CHECK(versionRect.x >= status.field(EditorStatusBar::kPerformance)->absoluteRect().x);
            const std::size_t fieldCount = shell.statusBar()->children().size();

            Node3D* fresh = document.scene()->createChild<Node3D>();
            fresh->setName("Fresh");
            MeshInstance3D* proof = document.scene()->createChild<MeshInstance3D>();
            proof->setName("Proof");
            proof->setMesh(createBoxMesh(meshes, Vec3(1.0f, 1.0f, 1.0f)));
            proof->setPosition(home);
            rig.frames(3);
            CHECK(document.dirty());
            CHECK(strncmp(status.text(EditorStatusBar::kScene).c_str(), "Unsaved", 7) == 0);
            CHECK(strstr(status.text(EditorStatusBar::kScene).c_str(), "Untitled") != nullptr);

            select(rig, fresh);
            CHECK(status.text(EditorStatusBar::kSelection) == "1 node · Fresh (Node3D)");
            Node3D* second = document.scene()->createChild<Node3D>();
            second->setName("Second");
            selection.add(second);
            rig.frames(3);
            CHECK(status.text(EditorStatusBar::kSelection) == "2 nodes · Fresh (Node3D)");
            selection.clear();
            rig.frames(2);

            gizmo.setTool(EditorGizmo::Tool::Move);
            rig.frames(2);
            CHECK(status.text(EditorStatusBar::kTool) == "Move · Global");
            gizmo.setSnapEnabled(true);
            rig.frames(2);
            CHECK(status.text(EditorStatusBar::kTool) == "Move · Global · Snap 1 m");
            gizmo.setLocalSpace(true);
            gizmo.setTool(EditorGizmo::Tool::Rotate);
            rig.frames(2);
            CHECK(status.text(EditorStatusBar::kTool) == "Rotate · Local · Snap 15°");
            gizmo.setTool(EditorGizmo::Tool::Scale);
            rig.frames(2);
            CHECK(status.text(EditorStatusBar::kTool) == "Scale · Local · Snap 10%");
            gizmo.setSnapEnabled(false);
            gizmo.setLocalSpace(false);
            gizmo.setTool(EditorGizmo::Tool::Transform);
            rig.frames(2);

            for (int i = 0; i < 40; ++i) rig.frame();
            CHECK(status.fps() > 0.0f && status.frameMilliseconds() > 0.0f);
            CHECK(near(status.fps() * status.frameMilliseconds(), 1000.0, 1.0));
            const ct::String drawText = status.text(EditorStatusBar::kDraw);
            CHECK(strncmp(drawText.c_str(), "Draws ", 6) == 0);
            CHECK(strstr(drawText.c_str(), "Triangles") != nullptr);
            CHECK(server.getStats().drawCalls > 0);
            char expectedDraw[64];
            snprintf(expectedDraw, sizeof expectedDraw, "Draws %u · Triangles %u",
                    server.getStats().drawCalls, server.getStats().triangles);
            CHECK(strcmp(drawText.c_str(), expectedDraw) == 0);
            CHECK(strstr(status.text(EditorStatusBar::kPerformance).c_str(), "FPS") != nullptr);
            CHECK(strstr(status.text(EditorStatusBar::kPerformance).c_str(), "ms") != nullptr);
            CHECK(shell.statusBar()->children().size() == fieldCount);

            status.postMessage(static_cast<int>(LOGLEVEL_WARN), "texture is missing\nsecond line");
            rig.frame();
            CHECK(status.text(EditorStatusBar::kLog) == "texture is missing");
            status.captureLog(true);
            CHECK(status.capturingLog());
            log_error("gizmo test error");
            rig.frame();
            CHECK(status.text(EditorStatusBar::kLog) == "gizmo test error");
            log_warn("gizmo test warning");
            rig.frame();
            CHECK(status.text(EditorStatusBar::kLog) == "gizmo test warning");
            status.captureLog(false);

            CHECK(document.createProject(testRoot(), "StatusProject"));
            rig.frames(4);
            CHECK(project.isOpen());
            CHECK(!document.dirty());
            CHECK(strncmp(status.text(EditorStatusBar::kScene).c_str(), "Saved", 5) == 0);
            CHECK(strstr(status.text(EditorStatusBar::kScene).c_str(), "main") != nullptr);
            CHECK(document.savedAt() > 0);
            Node3D* added = document.scene()->createChild<Node3D>();
            added->setName("Added");
            rig.frames(3);
            CHECK(document.dirty());
            CHECK(strncmp(status.text(EditorStatusBar::kScene).c_str(), "Unsaved", 7) == 0);

            const rt::Rect field = status.field(EditorStatusBar::kScene)->absoluteRect();
            const int fx = static_cast<int>(field.x + field.w * 0.5f);
            const int fy = static_cast<int>(field.y + field.h * 0.5f);
            rig.send(motion(fx, fy));
            rig.frame();
            rig.send(button(fx, fy, MouseButton::Left, true));
            rig.frame();
            rig.send(button(fx, fy, MouseButton::Left, false));
            rig.frames(3);
            CHECK(!document.dirty());
            CHECK(document.savedAt() > 0);
            CHECK(strncmp(status.text(EditorStatusBar::kScene).c_str(), "Saved", 5) == 0);
            const ct::String stamp = EditorStatusBar::formatTime(document.savedAt());
            CHECK(stamp.size() == 5 && stamp.data()[2] == ':');
            CHECK(strstr(status.text(EditorStatusBar::kScene).c_str(), stamp.c_str()) != nullptr);
            CHECK(file_exists(project.absolutePath(document.scenePath()).c_str()));
        }

        // F1 and F2 inside the editor viewport
        {
            struct FrameListener : MainListener
            {
                Rig* rig = nullptr;
                void drawOverlay() override
                {
                    rig->selection.update();
                    rig->selection.flush();
                    rig->inspector.update();
                    rig->status.update(1.0f / 60.0f);
                    rig->host.frame(1.0f / 60.0f, true);
                }
            } listener;
            listener.rig = &rig;

            StaticBody3D* body = document.scene()->createChild<StaticBody3D>();
            CollisionShape3D* shape = body->createChild<CollisionShape3D>();
            shape->setBox(Vec3(2.0f, 2.0f, 2.0f));
            body->setPosition(Vec3(0.0f, 0.0f, 0.0f));
            focusCamera(rig, Vec3(0.0f, 0.0f, 0.0f), 6.0f);
            panel.connectPreDraw();
            Main::setListener(&listener);
            Input& input = Main::device()->input();
            const int h = static_cast<int>(panel.height());
            const int w = static_cast<int>(panel.width());
            const int spotX = w - 40;
            const int spotY = h - 1 - 14;
            const auto frames = [&](int count)
            {
                for (int i = 0; i < count; ++i) Main::iteration();
            };
            frames(6);
            int overlayWidth = 0;
            int overlayHeight = 0;
            Main::overlaySize(overlayWidth, overlayHeight);
            CHECK(overlayWidth == w && overlayHeight == h);

            unsigned char off[4] = { 0, 0, 0, 0 };
            unsigned char on[4] = { 0, 0, 0, 0 };
            CHECK(readTexel(panel.texture(), spotX, spotY, off));
            const unsigned verticesOff = server.getStats().batchVertices;
            input.parseInputEvent(key(Key::F1, true, kModShift));
            input.parseInputEvent(key(Key::F1, false, kModShift));
            frames(5);
            frames(1);
            CHECK(readTexel(panel.texture(), spotX, spotY, on));
            CHECK(on[0] + 12 < off[0] && on[1] + 12 < off[1]);
            CHECK(server.getStats().batchVertices > verticesOff + 20);
            input.parseInputEvent(key(Key::F1, true, kModShift));
            input.parseInputEvent(key(Key::F1, false, kModShift));
            frames(6);
            CHECK(readTexel(panel.texture(), spotX, spotY, on));
            CHECK(abs(on[0] - off[0]) <= 3);

            CHECK(!Main::isPhysicsDebug());
            frames(3);
            const unsigned baseline = server.getStats().batchVertices;
            input.parseInputEvent(key(Key::F2, true, kModShift));
            input.parseInputEvent(key(Key::F2, false, kModShift));
            CHECK(Main::isPhysicsDebug());
            frames(5);
            frames(1);
            CHECK(server.getStats().batchVertices > baseline + 20);
            input.parseInputEvent(key(Key::F2, true, kModShift));
            input.parseInputEvent(key(Key::F2, false, kModShift));
            CHECK(!Main::isPhysicsDebug());
            frames(3);
            CHECK(server.getStats().batchVertices <= baseline + 2);

            Main::setListener(nullptr);
            panel.disconnectPreDraw();
            Main::setOverlaySize(0, 0);
        }

        host.shutdown();
    }

    tree.finalize();
    Main::cleanup();
    removeTree(testRoot());

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
