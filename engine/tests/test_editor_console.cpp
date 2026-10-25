#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/ConsolePanel.h"
#include "prisma3d/editor/EditorCameraGizmo.h"
#include "prisma3d/editor/EditorLightGizmo.h"
#include "prisma3d/editor/EditorLog.h"
#include "prisma3d/editor/EditorNodeIcons.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorView.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/script/ZenScripting.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Theme.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_console_test_vulkan" : "/editor_console_test";
    return root;
}

bool near(float a, float b, float epsilon = 1.0e-3f)
{
    return fabsf(a - b) <= epsilon;
}

bool nearPoint(const Math::Vec3& a, float x, float y, float z, float epsilon = 1.0e-3f)
{
    return near(a.x, x, epsilon) && near(a.y, y, epsilon) && near(a.z, z, epsilon);
}

void frame(SceneTree& tree, WidgetHost& host, ConsolePanel* console = nullptr)
{
    tree.process(1.0 / 60.0);
    if (console) console->update();
    Main::rendering()->renderViewports();
    host.frame(1.0f / 60.0f, true);
    Main::rendering()->endFrame(true);
}

void frames(SceneTree& tree, WidgetHost& host, int count, ConsolePanel* console = nullptr)
{
    for (int i = 0; i < count; ++i) frame(tree, host, console);
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

InputEvent character(char c)
{
    InputEvent event;
    event.type = InputEventType::Char;
    event.character.codepoint = static_cast<std::uint32_t>(c);
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

struct Stage
{
    SceneTree& tree;
    WidgetHost& host;
    ConsolePanel* console = nullptr;

    void frames(int count)
    {
        ::frames(tree, host, count, console);
    }

    void send(const InputEvent& event)
    {
        host.handleEvent(event);
    }

    void click(int x, int y, MouseButton which = MouseButton::Left)
    {
        send(motion(x, y));
        frames(2);
        send(button(x, y, which, true));
        frames(2);
        send(button(x, y, which, false));
        frames(3);
    }
};

int center(float origin, float size)
{
    return static_cast<int>(origin + size * 0.5f);
}

void testConsole(Stage& stage, rt::Widget* panel)
{
    EditorSettings settings;
    settings.consoleWarn = false;
    settings.consoleAutoScroll = false;
    ConsolePanel console;
    stage.console = &console;
    CHECK(console.attach(panel));
    CHECK(!console.attach(panel));
    console.bindSettings(&settings);
    rt::ConsoleWidget* widget = console.console();
    CHECK(widget != nullptr);
    if (!widget) return;
    stage.frames(2);
    CHECK(!widget->filter(rt::LogLevel::Warn) && widget->filter(rt::LogLevel::Error));
    CHECK(!widget->autoScroll());
    widget->setFilter(rt::LogLevel::Warn, true);
    widget->setAutoScroll(true);
    stage.frames(2);
    CHECK(settings.consoleWarn && settings.consoleAutoScroll);

    widget->clear();
    const int info0 = widget->messageCount(rt::LogLevel::Info);
    const int trace0 = widget->messageCount(rt::LogLevel::Trace);
    const int warn0 = widget->messageCount(rt::LogLevel::Warn);
    const int error0 = widget->messageCount(rt::LogLevel::Error);

    EditorLog& log = EditorLog::instance();
    log.post(static_cast<int>(LOGLEVEL_DEBUG), "debug line");
    log.post(static_cast<int>(LOGLEVEL_INFO), "info line");
    log.post(static_cast<int>(LOGLEVEL_WARN), "warn line");
    log.post(static_cast<int>(LOGLEVEL_ERROR), "error line");
    log.post(static_cast<int>(LOGLEVEL_ERROR), "first\nsecond");
    stage.frames(2);
    CHECK(widget->messageCount(rt::LogLevel::Trace) == trace0 + 1);
    CHECK(widget->messageCount(rt::LogLevel::Info) == info0 + 1);
    CHECK(widget->messageCount(rt::LogLevel::Warn) == warn0 + 1);
    CHECK(widget->messageCount(rt::LogLevel::Error) == error0 + 3);
    CHECK(console.received() >= 5);

    log.capture(true);
    const std::size_t from = log.total();
    log_info("captured engine line");
    log_warn("captured warning");
    log_error("captured error");
    std::size_t cursor = from;
    ct::Vector<EditorLog::Entry> entries;
    CHECK(log.read(cursor, entries) == 3);
    CHECK(entries[0].text == "captured engine line" && entries[0].level == LOGLEVEL_INFO);
    CHECK(entries[1].level == LOGLEVEL_WARN && entries[2].level == LOGLEVEL_ERROR);
    stage.frames(2);
    CHECK(widget->messageCount(rt::LogLevel::Warn) == warn0 + 2);

    ct::String script = testRoot() + "/Printer.py";
    CHECK(FileSystem::writeText(script, "class Printer(Node3D):\n    def _process(self, delta):\n  "
                                        "      print(\"printed by \" + "
                                        "self.name)\n"));
    Node3D* node = stage.tree.getRoot()->createChild<Node3D>();
    node->setName("Speaker");
    const int infoBefore = widget->messageCount(rt::LogLevel::Info);
    const std::size_t scriptFrom = log.total();
    node->setScript(script);
    stage.frames(4);
    cursor = scriptFrom;
    entries.clear();
    log.read(cursor, entries);
    bool printed = false;
    for (std::size_t i = 0; i < entries.size(); ++i)
        if (entries[i].script && entries[i].text == "printed by Speaker") printed = true;
    CHECK(printed);
    CHECK(widget->messageCount(rt::LogLevel::Info) > infoBefore);
    node->queueFree();
    stage.frames(2);
    log.capture(false);

    widget->setFilter(rt::LogLevel::Error, false);
    stage.frames(2);
    CHECK(!settings.consoleError);
    widget->setFilter(rt::LogLevel::Error, true);
    widget->clear();
    CHECK(widget->messageCount(rt::LogLevel::Error) == 0);
    stage.console = nullptr;
}

void testNumberField(Stage& stage, rt::Widget* panel)
{
    NumberField* field = panel->createChild<NumberField>();
    field->setSize(200.0f, 26.0f);
    field->setPosition(20.0f, 20.0f);
    field->setRange(0.0, 100.0);
    field->setDragSensitivity(1.0f);
    field->setValue(50.0f);
    stage.frames(3);
    CHECK(field->ranged() && near(field->ratio(), 0.5f));
    CHECK(!field->grabberVisible());

    const rt::Rect box = field->absoluteRect();
    stage.send(motion(center(box.x, box.w), center(box.y, box.h)));
    stage.frames(2);
    CHECK(field->grabberVisible());

    float lastValue = 0.0f;
    int changes = 0;
    field->valueChanged.connect(
            [&](float value)
            {
                lastValue = value;
                ++changes;
            });

    const int gx = center(field->grabberRect().x, field->grabberRect().w);
    const int gy = center(field->grabberRect().y, field->grabberRect().h);
    stage.send(motion(gx, gy));
    stage.frames(2);
    stage.send(button(gx, gy, MouseButton::Left, true));
    stage.frames(1);
    stage.send(motion(gx + 20, gy));
    stage.frames(2);
    CHECK(field->isGrabbing());
    CHECK(field->value() > 50.0f && field->value() < 70.0f);
    stage.send(motion(gx + 4000, gy));
    stage.frames(2);
    CHECK(near(field->value(), 100.0f) && near(field->ratio(), 1.0f));
    stage.send(motion(gx - 4000, gy));
    stage.frames(2);
    CHECK(near(field->value(), 0.0f));
    stage.send(button(gx - 4000, gy, MouseButton::Left, false));
    stage.frames(3);
    CHECK(!field->isGrabbing() && !field->isEditing());

    field->setValue(50.0f);
    changes = 0;
    const int bx = box.x + 30;
    const int by = center(box.y, box.h);
    stage.send(motion(bx, by));
    stage.frames(2);
    stage.send(button(bx, by, MouseButton::Left, true));
    stage.frames(1);
    stage.send(motion(bx + 10, by));
    stage.frames(1);
    stage.send(motion(bx + 30, by));
    stage.frames(2);
    CHECK(field->isDragging() && !field->isGrabbing());
    CHECK(near(field->value(), 80.0f));
    stage.send(button(bx + 30, by, MouseButton::Left, true));
    stage.send(button(bx + 30, by, MouseButton::Right, true));
    stage.frames(2);
    CHECK(near(field->value(), 50.0f));
    stage.send(button(bx + 30, by, MouseButton::Left, false));
    stage.frames(3);
    CHECK(near(field->value(), 50.0f) && !field->isEditing());

    stage.click(bx, by);
    CHECK(field->isEditing());
    CHECK(WidgetHost::textFocused());
    stage.send(key(Key::A, true));
    stage.send(key(Key::A, false));
    for (const char* c = "12.5"; *c; ++c) stage.send(character(*c));
    stage.frames(2);
    stage.send(key(Key::Enter, true));
    stage.send(key(Key::Enter, false));
    stage.frames(3);
    CHECK(!field->isEditing() && near(field->value(), 12.5f));
    field->setValue(50.0f);

    NumberField* plain = panel->createChild<NumberField>();
    plain->setSize(200.0f, 26.0f);
    plain->setPosition(20.0f, 60.0f);
    stage.frames(3);
    CHECK(!plain->ranged() && !plain->grabberVisible());
    const rt::Rect plainBox = plain->absoluteRect();
    stage.send(motion(center(plainBox.x, plainBox.w), center(plainBox.y, plainBox.h)));
    stage.frames(2);
    CHECK(!plain->grabberVisible());
}

void testCameraGizmo(Stage& stage)
{
    Camera3D* camera = stage.tree.getRoot()->createChild<Camera3D>();
    camera->setName("Cam");
    camera->setPerspective(90.0f, 0.1f, 100.0f);
    camera->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    stage.frames(2);

    ct::Vector<Math::Vec3> lines;
    EditorCameraGizmo::frustumLines(*camera, 16.0f / 9.0f, lines);
    CHECK(lines.size() == 30);
    if (lines.size() == 30)
    {
        const float h = sinf(45.0f * 3.14159265f / 180.0f);
        const float d = -cosf(45.0f * 3.14159265f / 180.0f);
        const float factorY = 9.0f / 16.0f;
        CHECK(nearPoint(lines[0], 0.0f, 0.0f, 0.0f));
        CHECK(nearPoint(lines[1], h, h * factorY, d * factorY));
        CHECK(nearPoint(lines[2], h, h * factorY, d * factorY));
        CHECK(nearPoint(lines[3], h, -h * factorY, d * factorY));
        CHECK(nearPoint(lines[24], 0.0f, h * factorY + h * 0.5f, d * factorY));
    }

    camera->setOrthogonal(4.0f, 0.1f, 100.0f);
    EditorCameraGizmo::frustumLines(*camera, 2.0f, lines);
    CHECK(lines.size() == 38);
    if (lines.size() == 38)
    {
        CHECK(nearPoint(lines[0], -4.0f, -2.0f, 0.0f));
        CHECK(nearPoint(lines[1], 4.0f, -2.0f, 0.0f));
        CHECK(nearPoint(lines[8], -4.0f, -2.0f, -1.0f));
    }

    camera->setPerspective(60.0f, 0.1f, 100.0f);
    EditorCameraGizmo gizmo;
    BatchRenderer& batch = Main::rendering()->batch();
    gizmo.draw(batch, stage.tree.getRoot(), nullptr, 16.0f / 9.0f);
    CHECK(gizmo.cameraCount() == 1 && gizmo.lineCount() == 15);
    camera->setVisible(false);
    stage.frames(1);
    gizmo.draw(batch, stage.tree.getRoot(), nullptr, 16.0f / 9.0f);
    CHECK(gizmo.cameraCount() == 0 && gizmo.lineCount() == 0);
    camera->queueFree();
    stage.frames(2);
}

void testLightGizmo(Stage& stage)
{
    DirectionalLight3D* sun = stage.tree.getRoot()->createChild<DirectionalLight3D>();
    OmniLight3D* lamp = stage.tree.getRoot()->createChild<OmniLight3D>();
    SpotLight3D* spot = stage.tree.getRoot()->createChild<SpotLight3D>();
    sun->setName("Sun");
    lamp->setName("Lamp");
    spot->setName("Spot");
    sun->setColor(Math::Vec3(0.5f, 0.25f, 0.1f));
    lamp->setRange(5.0f);
    lamp->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
    spot->setRange(10.0f);
    spot->setSpotAngle(45.0f);
    stage.frames(2);

    CHECK(nearPoint(EditorLightGizmo::gizmoColor(*sun), 1.0f, 0.5f, 0.2f));

    EditorLightGizmo::Shape shape;
    EditorLightGizmo::shape(*sun, shape);
    CHECK(shape.primary.size() == 28 && shape.secondary.empty() && shape.billboard.empty());
    if (shape.primary.size() == 28)
    {
        CHECK(nearPoint(shape.primary[0], 0.0f, 0.0f, -2.5f));
        CHECK(nearPoint(shape.primary[1], 0.0f, 0.8f, -1.5f));
        CHECK(nearPoint(shape.primary[14], 0.0f, 0.0f, -2.5f));
        CHECK(nearPoint(shape.primary[15], -0.8f, 0.0f, -1.5f));
    }

    EditorLightGizmo::shape(*lamp, shape);
    CHECK(shape.primary.empty() && shape.secondary.size() == 720 && shape.billboard.size() == 240);
    if (shape.secondary.size() == 720)
    {
        CHECK(nearPoint(shape.secondary[0], 0.0f, 0.0f, 5.0f));
        CHECK(nearPoint(shape.secondary[2], 0.0f, 0.0f, 5.0f));
        CHECK(nearPoint(shape.secondary[4], 0.0f, 5.0f, 0.0f));
        for (std::size_t i = 0; i < shape.secondary.size(); ++i)
            CHECK(near(shape.secondary[i].Length(), 5.0f, 0.01f));
    }

    EditorLightGizmo::shape(*spot, shape);
    CHECK(shape.primary.size() == 242 && shape.secondary.size() == 16);
    if (shape.primary.size() == 242)
    {
        const float w = 10.0f * sinf(45.0f * 3.14159265f / 180.0f);
        const float d = 10.0f * cosf(45.0f * 3.14159265f / 180.0f);
        CHECK(nearPoint(shape.primary[0], 0.0f, w, -d));
        CHECK(nearPoint(shape.primary[240], 0.0f, 0.0f, -10.0f));
        CHECK(nearPoint(shape.primary[241], 0.0f, 0.0f, 0.0f));
        CHECK(nearPoint(shape.secondary[1], 0.0f, 0.0f, 0.0f));
    }

    EditorLightGizmo gizmo;
    EditorView view;
    view.width = 640.0f;
    view.height = 480.0f;
    BatchRenderer& batch = Main::rendering()->batch();
    gizmo.draw(batch, stage.tree.getRoot(), nullptr, view);
    CHECK(gizmo.lightCount() == 3);
    CHECK(gizmo.lineCount() == 14 + 360 + 120 + 121 + 8);
    lamp->setVisible(false);
    stage.frames(1);
    gizmo.draw(batch, stage.tree.getRoot(), nullptr, view);
    CHECK(gizmo.lightCount() == 2);
    sun->queueFree();
    lamp->queueFree();
    spot->queueFree();
    stage.frames(2);
}

void testNodeIcons(Stage& stage)
{
    Node3D* root = stage.tree.getRoot()->createChild<Node3D>();
    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    OmniLight3D* lamp = root->createChild<OmniLight3D>();
    SpotLight3D* spot = root->createChild<SpotLight3D>();
    Camera3D* camera = root->createChild<Camera3D>();
    Node3D* empty = root->createChild<Node3D>();
    lamp->setColor(Math::Vec3(1.0f, 0.5f, 0.0f));
    stage.frames(2);

    CHECK(EditorNodeIcons::kindOf(*empty) == EditorNodeIcons::Kind::Marker);
    CHECK(EditorNodeIcons::kindOf(*sun) == EditorNodeIcons::Kind::Icon);
    CHECK(strcmp(EditorNodeIcons::iconName(*sun), "GizmoDirectionalLight") == 0);
    CHECK(strcmp(EditorNodeIcons::iconName(*lamp), "GizmoLight") == 0);
    CHECK(strcmp(EditorNodeIcons::iconName(*spot), "GizmoSpotLight") == 0);
    CHECK(strcmp(EditorNodeIcons::iconName(*camera), "GizmoCamera3D") == 0);
    CHECK(EditorNodeIcons::iconName(*empty) == nullptr);
    CHECK(nearPoint(EditorNodeIcons::tint(*lamp), 1.0f, 0.5f, 0.0f));
    CHECK(nearPoint(EditorNodeIcons::tint(*camera), 1.0f, 1.0f, 1.0f));

    ct::Vector<Math::Vec3> points;
    const Math::Mat4 global = Math::Mat4::Translation(Math::Vec3(1.0f, 2.0f, 3.0f));
    EditorNodeIcons::markerSegments(global, 0.25f, points);
    CHECK(points.size() == 12);
    if (points.size() == 12)
    {
        CHECK(nearPoint(points[0], 1.25f, 2.0f, 3.0f));
        CHECK(nearPoint(points[1], 1.0f, 2.0f, 3.0f));
        CHECK(nearPoint(points[2], 1.0f, 2.0f, 3.0f));
        CHECK(nearPoint(points[3], 0.75f, 2.0f, 3.0f));
        CHECK(nearPoint(points[4], 1.0f, 2.25f, 3.0f));
        CHECK(nearPoint(points[8], 1.0f, 2.0f, 3.25f));
        CHECK(nearPoint(points[11], 1.0f, 2.0f, 2.75f));
    }

    EditorView view;
    view.width = 800.0f;
    view.height = 600.0f;
    view.camera = Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 10.0f));
    view.projection.fov = 70.0f;
    CHECK(near(EditorNodeIcons::billboardScale(view, Math::Vec3(0.0f, 0.0f, 0.0f)), 10.0f));
    CHECK(near(EditorNodeIcons::billboardScale(view, Math::Vec3(3.0f, 2.0f, 5.0f)), 5.0f));

    const float tanHalf = tanf(35.0f * 3.14159265f / 180.0f);
    const float halfPixels = 0.05f * 300.0f / tanHalf;
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    CHECK(EditorNodeIcons::iconHit(view, origin, 0.05f, 400.0f, 300.0f));
    CHECK(EditorNodeIcons::iconHit(view, origin, 0.05f, 400.0f + halfPixels - 2.0f, 300.0f));
    CHECK(!EditorNodeIcons::iconHit(view, origin, 0.05f, 400.0f + halfPixels + 2.0f, 300.0f));
    CHECK(EditorNodeIcons::iconHit(view, origin, 0.05f, 400.0f, 300.0f - halfPixels + 2.0f));
    CHECK(!EditorNodeIcons::iconHit(view, origin, 0.05f, 400.0f, 300.0f - halfPixels - 2.0f));
    const float nearer = halfPixels * 0.5f;
    CHECK(!EditorNodeIcons::iconHit(view, Math::Vec3(0.0f, 0.0f, 20.0f), 0.05f, 400.0f, 300.0f));
    CHECK(EditorNodeIcons::iconHit(view, Math::Vec3(0.0f, 0.0f, 5.0f), 0.05f, 400.0f + nearer,
            300.0f));

    CHECK(EditorNodeIcons::markerHit(view, Math::Mat4::Identity(), 1.0f, 8.0f, 400.0f, 300.0f));
    const Math::Vec2 tip = view.project(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(EditorNodeIcons::markerHit(view, Math::Mat4::Identity(), 1.0f, 8.0f, tip.x,
            tip.y + 5.0f));
    CHECK(!EditorNodeIcons::markerHit(view, Math::Mat4::Identity(), 1.0f, 8.0f, tip.x,
            tip.y + 40.0f));

    view.projection.orthogonal = true;
    view.projection.size = 4.0f;
    CHECK(near(EditorNodeIcons::billboardScale(view, origin), 2.0f));

    EditorNodeIcons icons;
    BatchRenderer& batch = Main::rendering()->batch();
    view.projection.orthogonal = false;
    icons.draw(batch, root, nullptr, view);
    CHECK(icons.loaded());
    CHECK(icons.iconCount() == 4 && icons.markerCount() == 2);
    CHECK(icons.hasIcon("GizmoLight") && icons.hasIcon("GizmoDirectionalLight") &&
            icons.hasIcon("GizmoSpotLight") && icons.hasIcon("GizmoCamera3D"));
    lamp->setVisible(false);
    root->setVisible(true);
    empty->setVisible(false);
    stage.frames(1);
    icons.draw(batch, root, nullptr, view);
    CHECK(icons.iconCount() == 3 && icons.markerCount() == 1);
    root->setVisible(false);
    stage.frames(1);
    icons.draw(batch, root, nullptr, view);
    CHECK(icons.iconCount() == 0 && icons.markerCount() == 0);
    root->queueFree();
    stage.frames(2);
}

} // namespace

int main(int argc, char** argv)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor console test";
    desc.driver = useVulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = 640;
    desc.height = 480;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;
    removeTree(testRoot());
    CHECK(fs_create_directory(testRoot().c_str()));

    SceneTree tree;
    CHECK(Main::start(&tree));
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    Main::rendering()->viewportSetActive(window->getViewportId(), false);

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        CHECK(host.init());
        rt::Theme::instance() = rt::Theme::dark();
        rt::WidgetApp& app = host.widgets();
        rt::Widget* stage = app.addStage("editor");
        CHECK(stage != nullptr && app.setStage("editor"));
        rt::BoxLayout* column = stage->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
        column->setStretch(1);
        rt::BoxLayout* top = column->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
        top->setStretch(1);
        rt::Widget* numbers = top->createChild<rt::Widget>();
        numbers->setSize(400.0f, 120.0f);
        rt::BoxLayout* bottom = column->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
        bottom->setStretch(1);

        Stage rig{ tree, host };
        rig.frames(3);
        testNumberField(rig, numbers);
        testConsole(rig, bottom);
        testCameraGizmo(rig);
        testLightGizmo(rig);
        testNodeIcons(rig);
        rig.frames(2);
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
    printf("editor console ok\n");
    return 0;
}
