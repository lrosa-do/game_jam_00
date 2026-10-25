#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

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

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kRed = { 255, 0, 0 };
const Rgb kBlue = { 0, 0, 255 };

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

    const unsigned char* at(int x, int y) const
    {
        return pixels.data() + (static_cast<std::size_t>(y) * kWidth + x) * 4;
    }

    bool near(int x, int y, Rgb expected, const char* label) const
    {
        const unsigned char* p = at(x, y);
        const int dr = p[0] - expected.r;
        const int dg = p[1] - expected.g;
        const int db = p[2] - expected.b;
        const bool same = dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance &&
                          dg <= kTolerance && db >= -kTolerance && db <= kTolerance;
        if (!same)
            printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, p[0], p[1],
                    p[2], expected.r, expected.g, expected.b);
        return same;
    }

    bool isClear(int x, int y) const
    {
        const unsigned char* p = at(x, y);
        return abs(p[0] - kClear.r) <= 1 && abs(p[1] - kClear.g) <= 1 && abs(p[2] - kClear.b) <= 1;
    }
};

class PanelNode : public Node
{
public:
    PanelNode()
            : clicks(0),
              frames(0)
    {
        setProcess(true);
    }

    int clicks;
    int frames;

protected:
    void notificationHandler(int what) override
    {
        Node::notificationHandler(what);
        if (what != kNotificationProcess || !Main::gui()) return;
        ig::Context& ui = Main::gui()->context();
        ++frames;
        if (ui.beginWindow("Panel", ig::Rect(20.0f, 20.0f, 200.0f, 160.0f)))
        {
            if (ui.button("Click", ig::Rect(30.0f, 60.0f, 80.0f, 24.0f))) ++clicks;
            char text[32];
            snprintf(text, sizeof(text), "clicks %d", clicks);
            ui.label(text, ig::Vec2(30.0f, 16.0f));
        }
        ui.endWindow();
    }
};

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

InputEvent key(Key which, bool pressed, unsigned modifiers = 0, bool echo = false)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = which;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = echo;
    event.key.modifiers = modifiers;
    return event;
}

InputEvent character(std::uint32_t codepoint)
{
    InputEvent event;
    event.type = InputEventType::Char;
    event.character.codepoint = codepoint;
    return event;
}

bool keyMaps(Key which, ig::KeyCode expected, unsigned modifiers, bool control, bool shift)
{
    ig::Event out;
    if (!Gui::translateEvent(key(which, true, modifiers), out)) return false;
    return out.type == ig::EventType::KeyDown && out.key == expected && out.control == control &&
           out.shift == shift;
}

void testBackend(SceneTree& tree)
{
    Gui* gui = Main::gui();
    CHECK(gui != nullptr);
    if (!gui) return;
    GuiBackend& backend = gui->backend();
    CHECK(backend.ready());
    CHECK(backend.fontTexture().valid());
    CHECK(Main::textures()->valid(backend.fontTexture()));
    CHECK(GuiBackend::engineTexture(GuiBackend::guiTexture(backend.fontTexture())) ==
            backend.fontTexture());
    CHECK(GuiBackend::guiTexture(TextureId()).value == 0);
    CHECK(!GuiBackend::engineTexture(ig::TextureId()).valid());
    CHECK(backend.fontAtlas().texture().value != 0);

    ig::DrawList list;
    const ig::Rect full(0.0f, 0.0f, static_cast<float>(kWidth), static_cast<float>(kHeight));
    list.addRectFilled(ig::Rect(10.0f, 10.0f, 100.0f, 50.0f), ig::Color(255, 0, 0, 255), full);
    list.addRectFilled(ig::Rect(140.0f, 10.0f, 80.0f, 80.0f), ig::Color(0, 0, 255, 255),
            ig::Rect(150.0f, 20.0f, 40.0f, 40.0f));
    list.addRectFilled(ig::Rect(10.0f, 120.0f, 60.0f, 30.0f), ig::Color(255, 255, 255, 128), full);
    CHECK(backend.fontAtlas().appendText(list, backend.fontAtlas().defaultFont(), "Hello",
            ig::Vec2(10.0f, 170.0f), 16.0f, ig::Color(255, 255, 255, 255), full));
    const ig::DrawData data =
            list.data(ig::Vec2(static_cast<float>(kWidth), static_cast<float>(kHeight)), 1.0f);

    std::uint32_t geometry = 0;
    for (std::size_t i = 0; i < data.commands.size(); ++i)
    {
        if (data.commands[i].type == ig::DrawCommandType::Geometry) ++geometry;
    }
    CHECK(geometry >= 4);
    CHECK(geometry == data.commands.size());

    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    CHECK(backend.render(data));
    CHECK(backend.drawCalls() > 0);
    CHECK(backend.drawCalls() == geometry);

    Image image;
    CHECK(image.read());
    Main::rendering()->endFrame(true);
    if (image.pixels.size() != static_cast<std::size_t>(kWidth) * kHeight * 4) return;

    CHECK(image.near(50, 30, kRed, "red rect"));
    CHECK(image.near(5, 30, kClear, "left of the red rect"));
    CHECK(image.near(115, 30, kClear, "right of the red rect"));
    CHECK(image.near(50, 65, kClear, "below the red rect"));
    CHECK(image.near(160, 30, kBlue, "blue inside the clip"));
    CHECK(image.near(145, 15, kClear, "blue outside the clip, top left"));
    CHECK(image.near(200, 60, kClear, "blue outside the clip, right"));
    CHECK(image.near(160, 70, kClear, "blue outside the clip, below"));
    const Rgb mid = { (kClear.r + 255) / 2, (kClear.g + 255) / 2, (kClear.b + 255) / 2 };
    CHECK(image.near(40, 135, mid, "blended rect"));
    CHECK(image.near(75, 135, kClear, "right of the blended rect"));

    int bright = 0;
    int untouched = 0;
    for (int y = 170; y < 190; ++y)
    {
        for (int x = 10; x < 60; ++x)
        {
            if (image.at(x, y)[0] > kClear.r + 40) ++bright;
            if (image.isClear(x, y)) ++untouched;
        }
    }
    CHECK(bright > 10);
    CHECK(untouched > 10);

    ig::DrawData empty;
    empty.displaySize = ig::Vec2(static_cast<float>(kWidth), static_cast<float>(kHeight));
    empty.dpiScale = 1.0f;
    CHECK(backend.render(empty));
    CHECK(backend.drawCalls() == 0);
    ig::DrawData zero;
    CHECK(backend.render(zero));
    CHECK(backend.drawCalls() == 0);

    const ig::TextMetrics metrics =
            backend.measureText(backend.fontAtlas().defaultFont(), "Hello", 16.0f, 1.0f);
    CHECK(metrics.width > 10.0f && metrics.height > 5.0f);
}

void testTranslate()
{
    ig::Event out;

    CHECK(Gui::translateEvent(motion(12, 34), out));
    CHECK(out.type == ig::EventType::PointerMove && out.position.x == 12.0f &&
            out.position.y == 34.0f);

    CHECK(Gui::translateEvent(button(5, 6, MouseButton::Left, true), out));
    CHECK(out.type == ig::EventType::PointerDown && out.button == ig::PointerButton::Left &&
            out.position.x == 5.0f && out.position.y == 6.0f);
    CHECK(Gui::translateEvent(button(5, 6, MouseButton::Left, false), out));
    CHECK(out.type == ig::EventType::PointerUp && out.button == ig::PointerButton::Left);
    CHECK(Gui::translateEvent(button(7, 8, MouseButton::Middle, true), out));
    CHECK(out.type == ig::EventType::PointerDown && out.button == ig::PointerButton::Middle);
    CHECK(Gui::translateEvent(button(7, 8, MouseButton::Right, false), out));
    CHECK(out.type == ig::EventType::PointerUp && out.button == ig::PointerButton::Right);
    CHECK(!Gui::translateEvent(button(1, 1, MouseButton::X1, true), out));
    CHECK(!Gui::translateEvent(button(1, 1, MouseButton::X2, false), out));

    InputEvent wheel;
    wheel.type = InputEventType::MouseWheel;
    wheel.mouseWheel.x = 1;
    wheel.mouseWheel.y = 2;
    wheel.mouseWheel.deltaX = 0.5f;
    wheel.mouseWheel.deltaY = -2.0f;
    CHECK(Gui::translateEvent(wheel, out));
    CHECK(out.type == ig::EventType::PointerWheel && out.wheelX == 0.5f && out.wheelY == -2.0f);

    CHECK(keyMaps(Key::Backspace, ig::KeyCode::Backspace, 0, false, false));
    CHECK(keyMaps(Key::Enter, ig::KeyCode::Enter, 0, false, false));
    CHECK(keyMaps(Key::KpEnter, ig::KeyCode::Enter, 0, false, false));
    CHECK(keyMaps(Key::Delete, ig::KeyCode::Delete, 0, false, false));
    CHECK(keyMaps(Key::Tab, ig::KeyCode::Tab, 0, false, false));
    CHECK(keyMaps(Key::Left, ig::KeyCode::Left, 0, false, false));
    CHECK(keyMaps(Key::Right, ig::KeyCode::Right, 0, false, false));
    CHECK(keyMaps(Key::Up, ig::KeyCode::Up, 0, false, false));
    CHECK(keyMaps(Key::Down, ig::KeyCode::Down, 0, false, false));
    CHECK(keyMaps(Key::Home, ig::KeyCode::Home, 0, false, false));
    CHECK(keyMaps(Key::End, ig::KeyCode::End, 0, false, false));
    CHECK(keyMaps(Key::PageUp, ig::KeyCode::PageUp, 0, false, false));
    CHECK(keyMaps(Key::PageDown, ig::KeyCode::PageDown, 0, false, false));
    CHECK(keyMaps(Key::Escape, ig::KeyCode::Escape, 0, false, false));
    CHECK(keyMaps(Key::A, ig::KeyCode::A, 0, false, false));
    CHECK(keyMaps(Key::C, ig::KeyCode::C, 0, false, false));
    CHECK(keyMaps(Key::D, ig::KeyCode::D, 0, false, false));
    CHECK(keyMaps(Key::F, ig::KeyCode::F, 0, false, false));
    CHECK(keyMaps(Key::H, ig::KeyCode::H, 0, false, false));
    CHECK(keyMaps(Key::S, ig::KeyCode::S, 0, false, false));
    CHECK(keyMaps(Key::V, ig::KeyCode::V, 0, false, false));
    CHECK(keyMaps(Key::X, ig::KeyCode::X, 0, false, false));
    CHECK(keyMaps(Key::Y, ig::KeyCode::Y, 0, false, false));
    CHECK(keyMaps(Key::Z, ig::KeyCode::Z, 0, false, false));
    CHECK(keyMaps(Key::N, ig::KeyCode::N, 0, false, false));
    CHECK(keyMaps(Key::O, ig::KeyCode::O, 0, false, false));
    CHECK(keyMaps(Key::F4, ig::KeyCode::F4, 0, false, false));
    CHECK(keyMaps(Key::F5, ig::KeyCode::F5, 0, false, false));

    CHECK(keyMaps(Key::C, ig::KeyCode::C, kModControl, true, false));
    CHECK(keyMaps(Key::Left, ig::KeyCode::Left, kModShift, false, true));
    CHECK(keyMaps(Key::Z, ig::KeyCode::Z, kModControl | kModShift, true, true));
    CHECK(keyMaps(Key::A, ig::KeyCode::A, kModAlt | kModSuper, false, false));

    CHECK(Gui::translateEvent(key(Key::Backspace, true, 0, true), out));
    CHECK(out.type == ig::EventType::KeyDown && out.key == ig::KeyCode::Backspace);

    CHECK(!Gui::translateEvent(key(Key::B, true), out));
    CHECK(!Gui::translateEvent(key(Key::F1, true), out));
    CHECK(!Gui::translateEvent(key(Key::Space, true), out));
    CHECK(!Gui::translateEvent(key(Key::Enter, false), out));
    CHECK(!Gui::translateEvent(key(Key::A, false, kModControl), out));

    CHECK(Gui::translateEvent(character('a'), out));
    CHECK(out.type == ig::EventType::TextInput && out.textLength == 1 && out.text[0] == 'a');
    CHECK(Gui::translateEvent(character(0xe9), out));
    CHECK(out.textLength == 2 && static_cast<unsigned char>(out.text[0]) == 0xc3 &&
            static_cast<unsigned char>(out.text[1]) == 0xa9 && out.text[2] == '\0');
    CHECK(Gui::translateEvent(character(0x20ac), out));
    CHECK(out.textLength == 3 && static_cast<unsigned char>(out.text[0]) == 0xe2 &&
            static_cast<unsigned char>(out.text[1]) == 0x82 &&
            static_cast<unsigned char>(out.text[2]) == 0xac);
    CHECK(Gui::translateEvent(character(0x1f600), out));
    CHECK(out.textLength == 4 && static_cast<unsigned char>(out.text[0]) == 0xf0 &&
            static_cast<unsigned char>(out.text[1]) == 0x9f &&
            static_cast<unsigned char>(out.text[2]) == 0x98 &&
            static_cast<unsigned char>(out.text[3]) == 0x80 && out.text[4] == '\0');
    CHECK(!Gui::translateEvent(character(0), out));
    CHECK(!Gui::translateEvent(character(0x110000), out));

    InputEvent touch;
    touch.type = InputEventType::Touch;
    touch.touch.id = 0;
    touch.touch.x = 1.0f;
    touch.touch.y = 2.0f;
    touch.touch.pressure = 1.0f;
    touch.touch.phase = TouchPhase::Down;
    CHECK(!Gui::translateEvent(touch, out));

    InputEvent action;
    action.type = InputEventType::Action;
    action.action.name = "jump";
    action.action.pressed = true;
    action.action.strength = 1.0f;
    action.action.eventIndex = 0;
    CHECK(!Gui::translateEvent(action, out));

    CHECK(!Gui::translateEvent(InputEvent(), out));
}

void iterate(int count)
{
    for (int i = 0; i < count; ++i) Main::iteration();
}

void testLoop(SceneTree& tree, PanelNode* panel)
{
    Gui* gui = Main::gui();
    CHECK(gui != nullptr);
    if (!gui) return;
    Input* input = Input::singleton();
    CHECK(input != nullptr);
    if (!input) return;

    iterate(3);
    CHECK(panel->frames >= 3);
    CHECK(!gui->frameOpen());
    CHECK(panel->clicks == 0);
    CHECK(!gui->wantsPointer());

    const int buttonX = 98;
    const int buttonY = 124;

    input->parseInputEvent(motion(300, 230));
    iterate(2);
    CHECK(!gui->wantsPointer());

    input->parseInputEvent(motion(buttonX, buttonY));
    iterate(2);
    CHECK(gui->wantsPointer());
    CHECK(panel->clicks == 0);

    input->parseInputEvent(button(buttonX, buttonY, MouseButton::Left, true));
    iterate(2);
    input->parseInputEvent(button(buttonX, buttonY, MouseButton::Left, false));
    iterate(3);
    CHECK(panel->clicks == 1);
    CHECK(gui->wantsPointer());

    input->parseInputEvent(motion(300, 230));
    iterate(2);
    CHECK(!gui->wantsPointer());
    CHECK(panel->clicks == 1);

    input->parseInputEvent(motion(buttonX, buttonY));
    Main::device()->processEvents();
    gui->beginFrame(1.0f / 60.0f);
    CHECK(gui->frameOpen());
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    gui->endFrame(true);
    CHECK(!gui->frameOpen());
    Image image;
    CHECK(image.read());
    Main::rendering()->endFrame(true);
    if (image.pixels.size() != static_cast<std::size_t>(kWidth) * kHeight * 4) return;
    CHECK(!image.isClear(120, 100));
    CHECK(image.near(300, 230, kClear, "outside the window"));
    CHECK(image.isClear(10, 10));
    CHECK(!image.isClear(98, 124));
    CHECK(gui->backend().drawCalls() > 0);

    CHECK(gui->enabled());
    input->parseInputEvent(key(Key::F10, true));
    input->parseInputEvent(key(Key::F10, false));
    iterate(2);
    CHECK(!gui->enabled());
    CHECK(!gui->wantsPointer());

    const int clicksBefore = panel->clicks;
    input->parseInputEvent(motion(buttonX, buttonY));
    input->parseInputEvent(button(buttonX, buttonY, MouseButton::Left, true));
    iterate(2);
    input->parseInputEvent(button(buttonX, buttonY, MouseButton::Left, false));
    iterate(2);
    CHECK(panel->clicks == clicksBefore);
    CHECK(!gui->wantsPointer());

    Main::device()->processEvents();
    gui->beginFrame(1.0f / 60.0f);
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    gui->endFrame(true);
    Image hidden;
    CHECK(hidden.read());
    Main::rendering()->endFrame(true);
    if (hidden.pixels.size() == static_cast<std::size_t>(kWidth) * kHeight * 4)
    {
        CHECK(hidden.isClear(120, 100));
        CHECK(hidden.isClear(98, 124));
    }

    input->parseInputEvent(key(Key::F10, true, 0, true));
    iterate(1);
    CHECK(!gui->enabled());
    input->parseInputEvent(key(Key::F10, true));
    iterate(2);
    CHECK(gui->enabled());

    iterate(2);
    CHECK(Main::rendering()->getStats().batchDrawCalls == 0);
    input->parseInputEvent(key(Key::F1, true, kModShift));
    input->parseInputEvent(key(Key::F1, false, kModShift));
    iterate(3);
    CHECK(Main::rendering()->getStats().batchDrawCalls > 0);
    CHECK(Main::rendering()->getStats().batchVertices > 0);
    CHECK(gui->enabled());

    input->parseInputEvent(key(Key::F10, true));
    iterate(2);
    CHECK(!gui->enabled());
    CHECK(Main::rendering()->getStats().batchDrawCalls > 0);
    input->parseInputEvent(key(Key::F10, true));
    iterate(2);
    CHECK(gui->enabled());

    input->parseInputEvent(key(Key::F1, true, kModShift, true));
    iterate(2);
    CHECK(Main::rendering()->getStats().batchDrawCalls > 0);
    input->parseInputEvent(key(Key::F1, true, kModShift));
    iterate(3);
    CHECK(Main::rendering()->getStats().batchDrawCalls == 0);

    CHECK(!Main::isPhysicsDebug());
    input->parseInputEvent(key(Key::F2, true, kModShift, true));
    iterate(1);
    CHECK(!Main::isPhysicsDebug());
    input->parseInputEvent(key(Key::F2, true));
    iterate(1);
    CHECK(!Main::isPhysicsDebug());
    input->parseInputEvent(key(Key::F2, true, kModShift));
    iterate(2);
    CHECK(Main::isPhysicsDebug());
    input->parseInputEvent(key(Key::F2, true, kModShift));
    iterate(1);
    CHECK(!Main::isPhysicsDebug());
    Main::setPhysicsDebug(true);
    CHECK(Main::isPhysicsDebug());
    Main::setPhysicsDebug(false);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "gui test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::gui() != nullptr);
    if (!Main::gui()) return 1;
    CHECK(Gui::singleton() == Main::gui());
    CHECK(!Main::gui()->frameOpen());
    CHECK(!Main::gui()->wantsPointer());

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;
    Window* window = root->castTo<Window>();
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

    Camera3D* camera = root->createChild<Camera3D>();
    camera->makeCurrent();
    CHECK(camera->isCurrent());

    testBackend(tree);
    testTranslate();

    PanelNode* panel = root->createChild<PanelNode>();
    testLoop(tree, panel);

    tree.finalize();
    Main::cleanup();
    CHECK(Main::gui() == nullptr);
    CHECK(Gui::singleton() == nullptr);
    CHECK(Main::device() == nullptr);

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("gui ok\n");
    return 0;
}
