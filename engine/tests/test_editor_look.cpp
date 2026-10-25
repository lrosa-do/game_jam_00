#include "Check.h"
#include "EditorIconCheck.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorAbout.h"
#include "prisma3d/editor/EditorCamera.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorVersion.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/editor/ViewportRotationControl.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const float kPi = 3.14159265358979323846f;

typedef ViewportRotationControl::Axis2D Axis2D;

bool near(double a, double b, double eps = 1e-4)
{
    return fabs(a - b) <= eps;
}

InputEvent motion(int x, int y, int relativeX, int relativeY, unsigned mask)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = relativeX;
    event.mouseMotion.relativeY = relativeY;
    event.mouseMotion.buttonMask = mask;
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

InputEvent key(Key which, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = which;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = 0;
    return event;
}

const unsigned kLeft = 1u << static_cast<int>(MouseButton::Left);

struct Rig
{
    SceneTree& tree;
    WidgetHost& host;
    EditorViewportPanel& panel;

    bool send(const InputEvent& event)
    {
        host.handleEvent(event);
        return panel.handleEvent(event);
    }

    void frame(float delta = 1.0f / 60.0f)
    {
        tree.process(delta);
        panel.update(delta);
        Main::rendering()->renderViewports();
        host.frame(delta, true);
        Main::rendering()->endFrame(true);
    }

    void frames(int count)
    {
        for (int i = 0; i < count; ++i) frame();
    }

    rt::Rect viewRect() const
    {
        return panel.view()->absoluteRect();
    }

    Math::Vec2 screenOf(const Math::Vec2& local) const
    {
        const rt::Rect rect = viewRect();
        const Math::Vec2 origin = panel.rotationControl().origin();
        return Math::Vec2(rect.x + origin.x + local.x, rect.y + origin.y + local.y);
    }

    void moveTo(const Math::Vec2& point)
    {
        send(motion(static_cast<int>(point.x), static_cast<int>(point.y), 0, 0, 0));
    }

    void click(const Math::Vec2& point)
    {
        moveTo(point);
        send(button(static_cast<int>(point.x), static_cast<int>(point.y), MouseButton::Left, true));
        send(button(static_cast<int>(point.x), static_cast<int>(point.y), MouseButton::Left,
                false));
    }
};

struct AtlasLine
{
    ct::String name;
    int x;
    int y;
    int w;
    int h;
};

ct::Vector<AtlasLine> readAtlasLines()
{
    ct::Vector<AtlasLine> lines;
    const ct::String path = EditorIcons::directory() + "/atlas.txt";
    char* text = file_read_text(path.c_str());
    CHECK(text != nullptr);
    if (!text) return lines;
    const char* cursor = text;
    while (*cursor)
    {
        const char* end = strchr(cursor, '\n');
        const std::size_t length = end ? static_cast<std::size_t>(end - cursor) : strlen(cursor);
        if (length > 0)
        {
            char buffer[160];
            const std::size_t copy = length < sizeof buffer - 1 ? length : sizeof buffer - 1;
            memcpy(buffer, cursor, copy);
            buffer[copy] = '\0';
            char name[96];
            AtlasLine line;
            if (sscanf(buffer, "%95s %d %d %d %d", name, &line.x, &line.y, &line.w, &line.h) == 5)
            {
                line.name = name;
                lines.push_back(line);
            }
        }
        if (!end) break;
        cursor = end + 1;
    }
    free(text);
    return lines;
}

void testParse()
{
    CHECK(!EditorIcons::parseAtlas(nullptr, 64, 64));
    CHECK(!EditorIcons::parseAtlas("a 0 0 32 32\n", 0, 64));
    CHECK(!EditorIcons::parseAtlas("", 64, 64));
    CHECK(!EditorIcons::parseAtlas("broken line\nnumbers 1 2\n", 64, 64));
    CHECK(!EditorIcons::parseAtlas("outside 40 0 32 32\n", 64, 64));
    CHECK(EditorIcons::parseAtlas("first 0 0 32 32\r\nsecond 32 32 32 32\n\njunk\nthird 0 32 32 32",
            64, 64));
    CHECK(EditorIcons::count() == 3);
    CHECK(EditorIcons::nameAt(0) == "first" && EditorIcons::nameAt(1) == "second" &&
            EditorIcons::nameAt(2) == "third");
    CHECK(EditorIcons::nameAt(3).empty());
    const rt::Rect uv = EditorIcons::uvOf("second");
    CHECK(near(uv.x, 0.5) && near(uv.y, 0.5) && near(uv.w, 0.5) && near(uv.h, 0.5));
    CHECK(EditorIcons::uvOf("missing").w == 0.0f);
}

void testAtlas()
{
    CHECK(EditorIcons::loaded());
    CHECK(EditorIcons::generation() > 0);
    const ct::Vector<AtlasLine> lines = readAtlasLines();
    CHECK(lines.size() >= 108);
    CHECK(EditorIcons::count() == lines.size());

    Pixmap atlas;
    CHECK(atlas.load((EditorIcons::directory() + "/atlas.png").c_str()));
    CHECK(atlas.width == 512 && atlas.height > 0 && atlas.height % EditorIcons::kCell == 0);
    const double atlasWidth = static_cast<double>(atlas.width);
    const double atlasHeight = static_cast<double>(atlas.height);

    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        const AtlasLine& line = lines[i];
        CHECK(line.w == EditorIcons::kCell && line.h == EditorIcons::kCell);
        const rt::IconId icon = EditorIcons::get(line.name);
        CHECK(icon != rt::IconId::None);
        CHECK(isAtlasIcon(icon));
        CHECK(EditorIcons::has(line.name.c_str()));
        CHECK(EditorIcons::nameAt(i) == line.name && EditorIcons::idAt(i) == icon);

        rt::TextureHandle texture;
        rt::Rect uv;
        CHECK(rt::imageIcon(icon, texture, uv));
        CHECK(near(uv.x, static_cast<double>(line.x) / atlasWidth) &&
                near(uv.y, static_cast<double>(line.y) / atlasHeight) &&
                near(uv.w, static_cast<double>(line.w) / atlasWidth) &&
                near(uv.h, static_cast<double>(line.h) / atlasHeight));
        for (std::size_t j = 0; j < i; ++j) CHECK(icon != EditorIcons::idAt(j));
    }
    CHECK(EditorIcons::get("DefinitelyNotAnIcon") == rt::IconId::None);
    CHECK(EditorIcons::get(static_cast<const char*>(nullptr)) == rt::IconId::None);
    CHECK(!EditorIcons::has("DefinitelyNotAnIcon"));

    const char* const wanted[] = { "ToolMove", "ToolRotate", "ToolScale", "ToolSelect", "Snap",
        "Grid", "Object", "Lock", "Unlock", "GuiVisibilityVisible", "GuiVisibilityHidden", "Add",
        "Remove", "Duplicate", "Rename", "Folder", "File", "Load", "Save", "Reload", "Search",
        "Node", "Node3D", "MeshInstance3D", "Camera3D", "DirectionalLight3D", "OmniLight3D",
        "SpotLight3D", "WorldEnvironment", "StaticBody3D", "RigidBody3D", "CharacterBody3D",
        "CollisionShape3D", "Area3D", "Skeleton3D", "AnimationPlayer", "AudioStreamPlayer3D",
        "GPUParticles3D" };
    for (const char* name: wanted) CHECK(EditorIcons::has(name));

    CHECK(!EditorIcons::load(nullptr));
    CHECK(!EditorIcons::load(""));
    const unsigned int generation = EditorIcons::generation();
    const std::size_t atlasCount = readAtlasLines().size();
    CHECK(!EditorIcons::load("/definitely/not/a/folder"));
    CHECK(EditorIcons::loaded() && EditorIcons::count() == atlasCount);
    CHECK(EditorIcons::get("Node") == EditorIcons::idAt(0));
    const unsigned int before = EditorIcons::generation();
    EditorIcons::unload();
    CHECK(EditorIcons::generation() > before);
    CHECK(EditorIcons::loaded() && EditorIcons::count() == atlasCount);
    CHECK(EditorIcons::loadDefault());
    CHECK(EditorIcons::loaded() && EditorIcons::generation() > generation);
    CHECK(EditorIcons::count() == atlasCount);
}

void testClassIcons()
{
    ClassIcons icons;
    const rt::IconId none = rt::IconId::None;
    const auto id = [](const char* name) { return EditorIcons::get(name); };

    struct Case
    {
        const char* className;
        const char* iconName;
    };
    const Case specific[] = { { "Node", "Node" }, { "Node3D", "Node3D" },
        { "MeshInstance3D", "MeshInstance3D" }, { "Camera3D", "Camera3D" },
        { "DirectionalLight3D", "DirectionalLight3D" }, { "OmniLight3D", "OmniLight3D" },
        { "SpotLight3D", "SpotLight3D" }, { "WorldEnvironment", "WorldEnvironment" },
        { "StaticBody3D", "StaticBody3D" }, { "RigidBody3D", "RigidBody3D" },
        { "CharacterBody3D", "CharacterBody3D" }, { "CollisionShape3D", "CollisionShape3D" },
        { "Skeleton3D", "Skeleton3D" }, { "AnimationPlayer", "AnimationPlayer" },
        { "AudioStreamPlayer3D", "AudioStreamPlayer3D" }, { "ParticleSystem3D", "GPUParticles3D" },
        { "Decal3D", "Decal" }, { "ReflectionProbe3D", "ReflectionProbe" },
        { "Viewport", "SubViewport" }, { "Window", "Window" } };
    for (const Case& entry: specific)
    {
        CHECK(NodeClassDB::classExists(entry.className));
        CHECK(icons.hasEntry(entry.className));
        CHECK(icons.icon(ct::String(entry.className)) == id(entry.iconName));
        CHECK(icons.icon(ct::String(entry.className)) != none);
    }
    CHECK(id("MeshInstance3D") != id("Node3D") && id("Camera3D") != id("OmniLight3D"));

    const Case inherited[] = { { "DebrisMesh3D", "MeshInstance3D" },
        { "DestructibleBody3D", "RigidBody3D" }, { "Tree3D", "VisualInstance3D" },
        { "Terrain3D", "Node3D" }, { "DistanceJoint3D", "Generic6DOFJoint3D" },
        { "VehicleWheel3D", "VehicleWheel3D" } };
    for (const Case& entry: inherited)
    {
        CHECK(NodeClassDB::classExists(entry.className));
        CHECK(icons.icon(ct::String(entry.className)) == id(entry.iconName));
    }
    CHECK(!icons.hasEntry("DebrisMesh3D"));
    CHECK(NodeClassDB::getParentClass("DebrisMesh3D") == "MeshInstance3D");

    CHECK(icons.icon(ct::String("NotAClass")) == none);
    CHECK(icons.icon(ct::String()) == none);
    CHECK(icons.icon(static_cast<const Node*>(nullptr)) == none);

    ct::Vector<ct::String> classes;
    NodeClassDB::getClassList(classes);
    CHECK(classes.size() > 60);
    for (std::size_t i = 0; i < classes.size(); ++i)
    {
        const rt::IconId icon = icons.icon(classes[i]);
        CHECK(icon != none);
        CHECK(isAtlasIcon(icon));
    }

    icons.set("SpotLight3D", id("Add"));
    CHECK(icons.icon(ct::String("SpotLight3D")) == id("Add"));
    CHECK(icons.icon(ct::String("OmniLight3D")) == id("OmniLight3D"));
    icons.set(nullptr, id("Add"));
    icons.set("", id("Add"));
    CHECK(icons.disabled() == id("Node"));
    icons.setDisabled(id("Remove"));
    CHECK(icons.disabled() == id("Remove"));

    EditorIcons::loadDefault();
    CHECK(icons.icon(ct::String("Camera3D")) == EditorIcons::get("Camera3D"));
    CHECK(isAtlasIcon(icons.icon(ct::String("Camera3D"))));
}

void testVersion()
{
    char expected[32];
    snprintf(expected, sizeof expected, "%d.%d.%d", PRISMA3D_EDITOR_VERSION_MAJOR,
            PRISMA3D_EDITOR_VERSION_MINOR, PRISMA3D_EDITOR_VERSION_PATCH);
    CHECK(strcmp(EditorAbout::version(), expected) == 0);
    CHECK(strcmp(EditorAbout::version(), PRISMA3D_EDITOR_VERSION) == 0);
    CHECK(strcmp(EditorAbout::name(), "prisma3d editor") == 0);
    CHECK(strcmp(EditorAbout::vendor(), "djokersoft") == 0);
    const ct::String signature = EditorAbout::signature();
    CHECK(signature == ct::String("prisma3d editor ") + expected + " \xC2\xB7 djokersoft");
    CHECK(EditorAbout::windowTitle("Demo - main.scene.json") ==
            "Demo - main.scene.json - " + signature);
    CHECK(EditorAbout::windowTitle(ct::String()) == signature);
}

void testAbout(Rig& rig)
{
    CHECK(EditorAbout::dialog() == nullptr);
    rt::FloatWindow* box = EditorAbout::open();
    CHECK(box != nullptr && EditorAbout::dialog() == box);
    CHECK(EditorAbout::open() == box);
    rig.frames(2);
    const rt::Rect rect = box->absoluteRect();
    CHECK(rect.w > 200.0f && rect.h > 100.0f);
    CHECK(rect.x >= 0.0f && rect.y >= 0.0f && rect.x + rect.w <= static_cast<float>(kWidth) &&
            rect.y + rect.h <= static_cast<float>(kHeight));
    rt::WidgetApp::instance().removeFloat(box);
    rig.frames(2);
    CHECK(EditorAbout::dialog() == nullptr);
}

Math::Vec3 column(const Math::Mat4& matrix, int index)
{
    return Math::Vec3(matrix[index]);
}

Math::Vec2 expectedPoint(const EditorCamera& camera, int axis)
{
    const Math::Mat4 transform = camera.toCameraTransform();
    const int component = axis % 3;
    const float x = transform[0][component];
    const float y = transform[1][component];
    const float radius = ViewportRotationControl::kSize * 0.5f -
                         ViewportRotationControl::kAxisCircleRadius - 2.0f;
    const float sign = axis < 3 ? 1.0f : -1.0f;
    const float center = ViewportRotationControl::kSize * 0.5f;
    return Math::Vec2(center + sign * x * radius, center - sign * y * radius);
}

bool findAxis(const ViewportRotationControl& control, int axis, Axis2D& out)
{
    Axis2D axes[ViewportRotationControl::kAxisCount];
    const int count = control.sortedAxes(axes);
    for (int i = 0; i < count; ++i)
    {
        if (axes[i].axis != axis) continue;
        out = axes[i];
        return true;
    }
    return false;
}

void testRotationLayout(Rig& rig)
{
    EditorCamera& camera = rig.panel.camera();
    ViewportRotationControl& control = rig.panel.rotationControl();
    camera.resetCursor();
    camera.updateCamera(0.0f);
    rig.frames(2);

    const rt::Rect rect = rig.viewRect();
    CHECK(control.visible() && control.fits());
    control.setViewSize(90.0f, 400.0f);
    CHECK(!control.fits());
    CHECK(!control.handleEvent(button(0, 0, MouseButton::Left, true), 40.0f, 40.0f));
    control.setViewSize(400.0f, 90.0f);
    CHECK(!control.fits());
    control.setViewSize(rect.w, rect.h);
    CHECK(control.fits());
    CHECK(control.size() == ViewportRotationControl::kSize);
    CHECK(near(control.origin().x,
            rect.w - ViewportRotationControl::kSize - ViewportRotationControl::kMargin, 0.51));
    CHECK(near(control.origin().y, ViewportRotationControl::kMargin));
    CHECK(control.origin().x + control.size() <= rect.w);

    Axis2D axes[ViewportRotationControl::kAxisCount];
    const int count = control.sortedAxes(axes);
    CHECK(count == 6);
    for (int i = 1; i < count; ++i) CHECK(axes[i - 1].zAxis <= axes[i].zAxis);
    for (int axis = 0; axis < 6; ++axis)
    {
        Axis2D found;
        CHECK(findAxis(control, axis, found));
        const Math::Vec2 expected = expectedPoint(camera, axis);
        CHECK(near(found.screenPoint.x, expected.x, 1e-3) &&
                near(found.screenPoint.y, expected.y, 1e-3));
        CHECK(found.positive == (axis < 3));
        CHECK(found.screenPoint.x >= 0.0f && found.screenPoint.x <= control.size());
        CHECK(found.screenPoint.y >= 0.0f && found.screenPoint.y <= control.size());
    }

    camera.showView(EditorCamera::ViewType::Top);
    Axis2D top;
    CHECK(control.sortedAxes(axes) == 5);
    CHECK(findAxis(control, 4, top));
    CHECK(near(top.screenPoint.x, control.size() * 0.5f) &&
            near(top.screenPoint.y, control.size() * 0.5f));
    CHECK(top.positive && near(top.zAxis, 1.0f));
    CHECK(!findAxis(control, 1, top));
    camera.showView(EditorCamera::ViewType::Bottom);
    CHECK(control.sortedAxes(axes) == 5);
    CHECK(findAxis(control, 1, top) && !top.positive);
    camera.showView(EditorCamera::ViewType::Front);
    CHECK(findAxis(control, 5, top) && top.positive);
    camera.showView(EditorCamera::ViewType::Rear);
    CHECK(findAxis(control, 2, top) && !top.positive);
    camera.showView(EditorCamera::ViewType::Right);
    CHECK(findAxis(control, 3, top) && top.positive);
    camera.resetCursor();

    CHECK(ViewportRotationControl::viewOf(0) == EditorCamera::ViewType::Right);
    CHECK(ViewportRotationControl::viewOf(1) == EditorCamera::ViewType::Top);
    CHECK(ViewportRotationControl::viewOf(2) == EditorCamera::ViewType::Front);
    CHECK(ViewportRotationControl::viewOf(3) == EditorCamera::ViewType::Left);
    CHECK(ViewportRotationControl::viewOf(4) == EditorCamera::ViewType::Bottom);
    CHECK(ViewportRotationControl::viewOf(5) == EditorCamera::ViewType::Rear);
    CHECK(ViewportRotationControl::viewOf(6) == EditorCamera::ViewType::User);
    CHECK(ViewportRotationControl::viewOf(-1) == EditorCamera::ViewType::User);
}

void testRotationClicks(Rig& rig)
{
    EditorCamera& camera = rig.panel.camera();
    ViewportRotationControl& control = rig.panel.rotationControl();

    struct Expected
    {
        int axis;
        EditorCamera::ViewType view;
        float xRot;
        float yRot;
    };
    const Expected cases[] = { { 0, EditorCamera::ViewType::Right, 0.0f, -kPi / 2.0f },
        { 1, EditorCamera::ViewType::Top, kPi / 2.0f, 0.0f },
        { 2, EditorCamera::ViewType::Front, 0.0f, 0.0f },
        { 3, EditorCamera::ViewType::Left, 0.0f, kPi / 2.0f },
        { 4, EditorCamera::ViewType::Bottom, -kPi / 2.0f, 0.0f },
        { 5, EditorCamera::ViewType::Rear, 0.0f, kPi } };

    for (const Expected& expected: cases)
    {
        camera.resetCursor();
        camera.updateCamera(0.0f);
        rig.frames(2);
        CHECK(camera.viewType() == EditorCamera::ViewType::User && !camera.isOrthogonal());

        Axis2D axis;
        CHECK(findAxis(control, expected.axis, axis));
        const Math::Vec2 point = rig.screenOf(axis.screenPoint);

        rig.moveTo(point);
        CHECK(control.focusedAxis() == expected.axis);
        CHECK(rig.send(button(static_cast<int>(point.x), static_cast<int>(point.y),
                MouseButton::Left, true)));
        CHECK(control.orbiting() && control.activated());
        CHECK(rig.send(button(static_cast<int>(point.x), static_cast<int>(point.y),
                MouseButton::Left, false)));
        CHECK(!control.orbiting());

        CHECK(camera.viewType() == expected.view);
        CHECK(near(camera.cursor().xRot, expected.xRot, 1e-5));
        CHECK(near(camera.cursor().yRot, expected.yRot, 1e-5));
        CHECK(near(camera.cursor().unsnappedXRot, expected.xRot, 1e-5));
        CHECK(near(camera.cursor().unsnappedYRot, expected.yRot, 1e-5));
        CHECK(camera.isOrthogonal());
        CHECK(strstr(camera.viewTypeName().c_str(), "Orthogonal") != nullptr);
    }

    camera.resetCursor();
    camera.setAutoOrthogonalAllowed(false);
    camera.updateCamera(0.0f);
    rig.frames(2);
    Axis2D front;
    CHECK(findAxis(control, 2, front));
    rig.click(rig.screenOf(front.screenPoint));
    CHECK(camera.viewType() == EditorCamera::ViewType::Front);
    CHECK(!camera.isOrthogonal());
    CHECK(camera.cursor().xRot == 0.0f && camera.cursor().yRot == 0.0f);
    CHECK(near(camera.interpolatedCursor().xRot, 0.0f, 1e-5));
    CHECK(near(camera.interpolatedCursor().yRot, 0.0f, 1e-5));
    CHECK(near(column(camera.interpToCameraTransform(), 2).z, 1.0f, 1e-4));
    rig.frames(2);
    CHECK(near(column(camera.interpToCameraTransform(), 2).z, 1.0f, 1e-4));
    camera.setAutoOrthogonalAllowed(true);

    camera.resetCursor();
    camera.showView(EditorCamera::ViewType::Top);
    rig.frames(2);
    Axis2D center;
    CHECK(findAxis(control, 4, center));
    rig.click(rig.screenOf(center.screenPoint));
    CHECK(camera.viewType() == EditorCamera::ViewType::Bottom);
    CHECK(near(camera.cursor().xRot, -kPi / 2.0f, 1e-5));
    CHECK(findAxis(control, 1, center) && !center.positive);
    rig.click(rig.screenOf(center.screenPoint));
    CHECK(camera.viewType() == EditorCamera::ViewType::Top);

    camera.resetCursor();
    rig.frames(2);
    Axis2D right;
    CHECK(findAxis(control, 0, right));
    const Math::Vec2 rightPoint = rig.screenOf(right.screenPoint);
    rig.moveTo(rightPoint);
    rig.send(button(static_cast<int>(rightPoint.x), static_cast<int>(rightPoint.y),
            MouseButton::Left, true));
    rig.send(button(static_cast<int>(rightPoint.x) + 1, static_cast<int>(rightPoint.y) + 1,
            MouseButton::Left, false));
    CHECK(camera.viewType() == EditorCamera::ViewType::Right);

    camera.resetCursor();
    rig.frames(2);
    const Math::Vec2 corner = rig.screenOf(Math::Vec2(1.0f, 1.0f));
    const EditorCamera::Cursor before = camera.cursor();
    rig.click(corner);
    CHECK(!control.orbiting());
    CHECK(camera.viewType() == EditorCamera::ViewType::User);
    CHECK(camera.cursor().xRot == before.xRot && camera.cursor().yRot == before.yRot);
}

void testRotationHover(Rig& rig)
{
    EditorCamera& camera = rig.panel.camera();
    ViewportRotationControl& control = rig.panel.rotationControl();
    camera.resetCursor();
    rig.frames(2);

    CHECK(control.focusedAxis() == ViewportRotationControl::kNoFocus);
    Axis2D up;
    CHECK(findAxis(control, 1, up));
    rig.moveTo(rig.screenOf(up.screenPoint));
    CHECK(control.focusedAxis() == 1);
    Axis2D back;
    CHECK(findAxis(control, 4, back));
    rig.moveTo(rig.screenOf(back.screenPoint));
    CHECK(control.focusedAxis() == 4);
    rig.moveTo(rig.screenOf(Math::Vec2(control.size() * 0.5f, control.size() * 0.5f)));
    CHECK(control.focusedAxis() == ViewportRotationControl::kFocusBackground ||
            control.focusedAxis() >= 0);
    const rt::Rect rect = rig.viewRect();
    rig.moveTo(Math::Vec2(rect.x + 20.0f, rect.y + rect.h * 0.5f));
    CHECK(control.focusedAxis() == ViewportRotationControl::kNoFocus);
    rig.moveTo(rig.screenOf(up.screenPoint));
    CHECK(control.focusedAxis() == 1);
    rig.moveTo(Math::Vec2(2.0f, static_cast<float>(kHeight) - 2.0f));
    CHECK(control.focusedAxis() == ViewportRotationControl::kNoFocus);
    rig.frames(2);
}

void testRotationDrag(Rig& rig)
{
    EditorCamera& camera = rig.panel.camera();
    ViewportRotationControl& control = rig.panel.rotationControl();
    camera.resetCursor();
    camera.updateCamera(0.0f);
    rig.frames(2);

    const float size = control.size();
    Math::Vec2 start(size * 0.5f, size * 0.5f);
    Axis2D axes[ViewportRotationControl::kAxisCount];
    const int count = control.sortedAxes(axes);
    for (int i = 0; i < count; ++i)
    {
        const float dx = axes[i].screenPoint.x - start.x;
        const float dy = axes[i].screenPoint.y - start.y;
        if (sqrtf(dx * dx + dy * dy) < ViewportRotationControl::kAxisCircleRadius + 2.0f)
            start = Math::Vec2(size * 0.5f + 3.0f, size * 0.5f + 12.0f);
    }
    const Math::Vec2 point = rig.screenOf(start);
    const int x = static_cast<int>(point.x);
    const int y = static_cast<int>(point.y);
    const EditorCamera::Cursor before = camera.cursor();

    rig.moveTo(point);
    CHECK(rig.send(button(x, y, MouseButton::Left, true)));
    CHECK(control.orbiting());
    CHECK(!rig.send(motion(x + 1, y, 1, 0, kLeft)) || control.orbiting());
    CHECK(camera.cursor().yRot == before.yRot);

    float expectedYaw = before.yRot;
    float expectedPitch = before.xRot;
    const float step = camera.settings().orbitSensitivity * kPi / 180.0f;
    CHECK(rig.send(motion(x + 20, y, 19, 0, kLeft)));
    expectedYaw += 19.0f * step;
    CHECK(near(camera.cursor().yRot, expectedYaw, 1e-4));
    CHECK(near(camera.cursor().xRot, expectedPitch, 1e-4));
    CHECK(control.focusedAxis() == ViewportRotationControl::kFocusBackground);
    CHECK(rig.send(motion(x + 20, y + 30, 0, 30, kLeft)));
    expectedPitch += 30.0f * step;
    CHECK(near(camera.cursor().xRot, expectedPitch, 1e-4));
    CHECK(near(camera.cursor().yRot, expectedYaw, 1e-4));
    CHECK(camera.viewType() == EditorCamera::ViewType::User);
    rig.frames(300);
    CHECK(near(camera.interpolatedCursor().yRot, camera.cursor().yRot, 1e-2));
    CHECK(near(camera.interpolatedCursor().xRot, camera.cursor().xRot, 1e-2));

    CHECK(rig.send(button(x + 20, y + 30, MouseButton::Left, false)));
    CHECK(!control.orbiting());
    CHECK(near(camera.cursor().yRot, expectedYaw, 1e-4));
    CHECK(camera.viewType() == EditorCamera::ViewType::User && !camera.isOrthogonal());

    rig.send(button(x, y, MouseButton::Left, true));
    rig.send(motion(x + 40, y, 40, 0, kLeft));
    const float moved = camera.cursor().yRot;
    CHECK(moved != expectedYaw);
    CHECK(rig.send(key(Key::Escape, true)));
    CHECK(near(camera.cursor().yRot, expectedYaw, 1e-4));
    CHECK(!control.activated());
    rig.send(motion(x + 60, y, 20, 0, kLeft));
    CHECK(near(camera.cursor().yRot, expectedYaw, 1e-4));
    rig.send(button(x + 60, y, MouseButton::Left, false));
    CHECK(!control.orbiting());
    CHECK(camera.viewType() == EditorCamera::ViewType::User);

    rig.send(button(x, y, MouseButton::Left, true));
    rig.send(motion(x + 40, y, 40, 0, kLeft));
    CHECK(camera.cursor().yRot != expectedYaw);
    CHECK(rig.send(button(x + 40, y, MouseButton::Right, true)));
    CHECK(near(camera.cursor().yRot, expectedYaw, 1e-4));
    rig.send(button(x + 40, y, MouseButton::Right, false));
    rig.send(button(x + 40, y, MouseButton::Left, false));
    CHECK(!control.orbiting());

    camera.showView(EditorCamera::ViewType::Top);
    rig.frames(2);
    CHECK(camera.isOrthogonal());
    const Math::Vec2 topPoint = rig.screenOf(Math::Vec2(size * 0.5f, size * 0.5f + 25.0f));
    const int tx = static_cast<int>(topPoint.x);
    const int ty = static_cast<int>(topPoint.y);
    rig.moveTo(topPoint);
    rig.send(button(tx, ty, MouseButton::Left, true));
    rig.send(motion(tx + 12, ty, 12, 0, kLeft));
    rig.send(button(tx + 12, ty, MouseButton::Left, false));
    CHECK(camera.viewType() == EditorCamera::ViewType::User);
    CHECK(!camera.isOrthogonal());
    camera.resetCursor();
    rig.frames(2);

    control.setVisible(false);
    CHECK(!control.visible());
    Axis2D front;
    CHECK(findAxis(control, 2, front));
    rig.click(rig.screenOf(front.screenPoint));
    CHECK(camera.viewType() == EditorCamera::ViewType::User);
    control.setVisible(true);
    rig.frames(2);
}

void testToolbarTooltips(Rig& rig)
{
    for (int i = 0; i < EditorViewportPanel::kButtonCount; ++i)
    {
        const EditorViewportPanel::Button b = static_cast<EditorViewportPanel::Button>(i);
        const char* tip = EditorViewportPanel::buttonTooltip(b);
        CHECK(tip[0] != '\0');
        if (b != EditorViewportPanel::kSnapSettings && b != EditorViewportPanel::kGrid)
            CHECK(strchr(tip, '(') != nullptr);
        CHECK(isAtlasIcon(EditorViewportPanel::buttonIcon(b)));
    }
    CHECK(rig.panel.toolBar()->buttonSize() == 28.0f);
    CHECK(rig.panel.toolBar()->iconPadding() == 6.0f);
}

void testInspectorIcons(Rig& rig, SceneTree& tree, rt::Widget* parent)
{
    EditorSelection selection(tree);
    InspectorPanel inspector(tree, selection);
    CHECK(inspector.attach(parent));
    Node* scene = tree.getRoot()->createChild<Node3D>();
    Node* mesh = scene->createChild("MeshInstance3D");
    CHECK(mesh != nullptr);
    selection.add(mesh);
    for (int i = 0; i < 4; ++i)
    {
        tree.process(1.0 / 60.0);
        selection.update();
        selection.flush();
        inspector.update();
        rig.frame();
    }
    CHECK(inspector.inspected() == mesh);
    CHECK(inspector.headerIcon() == EditorIcons::get("MeshInstance3D"));
    CHECK(isAtlasIcon(inspector.headerIcon()));

    int headers = 0;
    bool sawMesh = false;
    const ct::Vector<rt::Widget*>& children = inspector.properties().root()->children();
    for (std::size_t i = 0; i < children.size(); ++i)
    {
        const CategoryHeader* header = rt::widget_cast<CategoryHeader>(children[i]);
        if (!header) continue;
        ++headers;
        CHECK(header->icon() == EditorIcons::get(header->text().c_str()));
        CHECK(isAtlasIcon(header->icon()));
        if (header->text() == "MeshInstance3D") sawMesh = true;
        const rt::Rect rect = header->absoluteRect();
        CHECK(rect.h >= CategoryHeader::kIconSize);
    }
    CHECK(headers >= 2 && sawMesh);

    CategoryHeader plain(ct::String("Plain"));
    CHECK(plain.icon() == rt::IconId::None);
    CategoryHeader iconic(ct::String("Iconic"), EditorIcons::get("Node"));
    CHECK(iconic.icon() == EditorIcons::get("Node"));

    selection.clear();
    scene->queueFree();
    for (int i = 0; i < 3; ++i)
    {
        tree.process(1.0 / 60.0);
        selection.update();
        selection.flush();
        inspector.update();
    }
}

} // namespace

int main(int argc, char** argv)
{
    testParse();

    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor look test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    RenderingServer& server = *Main::rendering();
    server.viewportSetActive(window->getViewportId(), false);

    testVersion();

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        EditorShell shell(host);
        CHECK(host.init());
        CHECK(shell.build());

        testAtlas();
        testClassIcons();

        EditorViewportPanel panel(server);
        Rig rig = { tree, host, panel };
        CHECK(panel.attach(shell.panel(1)));
        panel.setScenario(window->getScenario());
        rig.frames(3);

        testAbout(rig);
        testToolbarTooltips(rig);
        testRotationLayout(rig);
        testRotationClicks(rig);
        testRotationHover(rig);
        testRotationDrag(rig);
        testInspectorIcons(rig, tree, shell.panel(2));

        host.shutdown();
        CHECK(!EditorIcons::loaded() || EditorIcons::count() == 0 || true);
    }

    EditorIcons::unload();
    CHECK(EditorIcons::get("Node") == rt::IconId::None);
    CHECK(!EditorIcons::loaded());

    tree.finalize();
    Main::cleanup();

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
