#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/DockPanel.hpp>
#include <igui/widgets/Theme.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 640;
const int kHeight = 480;
const int kTolerance = 3;

bool readPixel(int x, int y, unsigned char* out)
{
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, out);
}

bool sameColor(const unsigned char* pixel, const ig::Color& color)
{
    const int dr = pixel[0] - color.r;
    const int dg = pixel[1] - color.g;
    const int db = pixel[2] - color.b;
    return dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance && dg <= kTolerance &&
           db >= -kTolerance && db <= kTolerance;
}

void frame(SceneTree& tree, WidgetHost& host, int x = -1, int y = -1,
        unsigned char* pixel = nullptr)
{
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    host.frame(1.0f / 60.0f, true);
    if (pixel) CHECK(readPixel(x, y, pixel));
    Main::rendering()->endFrame(true);
}

InputEvent motion(int x, int y)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = 0;
    event.mouseMotion.relativeY = 0;
    return event;
}

InputEvent button(int x, int y, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = MouseButton::Left;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = 0;
    return event;
}

void click(SceneTree& tree, WidgetHost& host, int x, int y)
{
    host.handleEvent(motion(x, y));
    frame(tree, host);
    host.handleEvent(button(x, y, true));
    frame(tree, host);
    host.handleEvent(button(x, y, false));
    frame(tree, host);
}

bool sameRect(const rt::Rect& a, const rt::Rect& b)
{
    const float d = 1.0f;
    return a.x - b.x <= d && b.x - a.x <= d && a.y - b.y <= d && b.y - a.y <= d && a.w - b.w <= d &&
           b.w - a.w <= d && a.h - b.h <= d && b.h - a.h <= d;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor widgets test";
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
    window->setClearColor(Math::Vec4(1.0f, 0.0f, 1.0f, 1.0f));

    CHECK(WidgetHost::translateKey(Key::A) == 'a');
    CHECK(WidgetHost::translateKey(Key::Z) == 'z');
    CHECK(WidgetHost::translateKey(Key::Space) == ' ');
    CHECK(WidgetHost::translateKey(Key::Zero) == '0');
    CHECK(WidgetHost::translateKey(Key::Escape) == rt::Key::Escape);
    CHECK(WidgetHost::translateKey(Key::Enter) == rt::Key::Return);
    CHECK(WidgetHost::translateKey(Key::KpEnter) == rt::Key::KPEnter);
    CHECK(WidgetHost::translateKey(Key::F1) == rt::Key::F1);
    CHECK(WidgetHost::translateKey(Key::F12) == rt::Key::F12);
    CHECK(WidgetHost::translateKey(Key::Left) == rt::Key::Left);
    CHECK(WidgetHost::translateKey(Key::None) == 0);
    CHECK(WidgetHost::translateCursor(0) == CursorShape::Arrow);
    CHECK(WidgetHost::translateCursor(2) == CursorShape::IBeam);
    CHECK(WidgetHost::translateCursor(7) == CursorShape::Forbidden);

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        CHECK(!host.ready());
        CHECK(host.init());
        CHECK(host.ready() && WidgetHost::singleton() == &host);
        CHECK(host.font() != nullptr);
        CHECK(host.textureCount() == 1);

        WidgetHost second(*Main::device(), *Main::gui(), *Main::textures());
        CHECK(!second.init());

        rt::Theme::instance() = rt::Theme::dark();
        const ig::Color panelColor = rt::Theme::instance().panelColor;

        rt::WidgetApp& app = host.widgets();
        rt::Widget* stage = app.addStage("editor");
        CHECK(stage != nullptr);
        CHECK(app.setStage("editor"));
        rt::BoxLayout* outer = stage->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
        outer->setStretch(1);
        rt::DockPanel* dock = outer->createChild<rt::DockPanel>();
        dock->setStretch(1);
        rt::Panel* hierarchy = dock->addPanel<rt::Panel>("Hierarchy");
        rt::Panel* viewport = dock->addPanel<rt::Panel>("Viewport");
        rt::Panel* inspector = dock->addPanel<rt::Panel>("Inspector");
        rt::Panel* console = dock->addPanel<rt::Panel>("Console");
        rt::Panel* assets = dock->addPanel<rt::Panel>("Assets");
        dock->splitOff("Hierarchy", rt::DockSide::Left, 0.20f);
        dock->splitOff("Inspector", rt::DockSide::Right, 0.22f);
        dock->splitOff("Console", rt::DockSide::Bottom, 0.25f);
        dock->moveTabToLeaf("Assets", "Console");
        dock->showPanel("Console");

        ct::Vector<rt::String> activated;
        dock->panelActivated.connect(
                [&activated](const rt::String& name) { activated.push_back(name); });

        frame(tree, host);
        frame(tree, host);
        CHECK(host.drawCalls() > 0);

        const rt::Rect left = hierarchy->rect();
        const rt::Rect middle = viewport->rect();
        const rt::Rect right = inspector->rect();
        const rt::Rect bottom = console->rect();
        printf("hierarchy %.0f,%.0f %.0fx%.0f | viewport %.0f,%.0f %.0fx%.0f | inspector %.0f,%.0f "
               "%.0fx%.0f | console %.0f,%.0f %.0fx%.0f\n",
                left.x, left.y, left.w, left.h, middle.x, middle.y, middle.w, middle.h, right.x,
                right.y, right.w, right.h, bottom.x, bottom.y, bottom.w, bottom.h);
        CHECK(left.w > 0.14f * kWidth && left.w < 0.26f * kWidth);
        CHECK(right.w > 0.12f * kWidth && right.w < 0.28f * kWidth);
        CHECK(bottom.h > 0.12f * kHeight && bottom.h < 0.32f * kHeight);
        CHECK(left.x < middle.x && middle.x < right.x);
        CHECK(bottom.y > middle.y);
        CHECK(middle.w > left.w && middle.w > right.w);

        unsigned char pixel[4] = { 0, 0, 0, 0 };
        const rt::Rect origin = dock->absoluteRect();
        const int px = static_cast<int>(origin.x + left.x + left.w * 0.5f);
        const int py = static_cast<int>(origin.y + left.y + left.h * 0.5f);
        frame(tree, host, px, py, pixel);
        if (!sameColor(pixel, panelColor))
            printf("panel pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", px, py, pixel[0],
                    pixel[1], pixel[2], panelColor.r, panelColor.g, panelColor.b);
        CHECK(sameColor(pixel, panelColor));
        frame(tree, host, static_cast<int>(origin.x + left.x + 8.0f),
                static_cast<int>(origin.y + 8.0f), pixel);
        CHECK(!(pixel[0] > 250 && pixel[1] < 5 && pixel[2] > 250));

        const rt::String saved = dock->saveLayout();
        CHECK(!saved.empty());
        dock->moveTabToLeaf("Assets", "Hierarchy");
        dock->showPanel("Assets");
        frame(tree, host);
        frame(tree, host);
        CHECK(!sameRect(assets->rect(), bottom));
        CHECK(!dock->restoreLayout("not a layout"));
        CHECK(dock->restoreLayout(saved));
        dock->showPanel("Console");
        dock->layout();
        frame(tree, host);
        frame(tree, host);
        CHECK(sameRect(hierarchy->rect(), left));
        CHECK(sameRect(viewport->rect(), middle));
        CHECK(sameRect(inspector->rect(), right));
        CHECK(sameRect(console->rect(), bottom));

        activated.clear();
        const rt::Rect dockRect = dock->absoluteRect();
        const int tabY = static_cast<int>(dockRect.y + bottom.y - dock->tabBarHeight() * 0.5f);
        const int tabX = static_cast<int>(dockRect.x + bottom.x);
        rt::String first;
        rt::String other;
        for (int pass = 0; pass < 2; ++pass)
        {
            rt::String& target = pass == 0 ? first : other;
            for (int x = tabX + 6; x < tabX + 320 && target.empty(); x += 6)
            {
                const std::size_t before = activated.size();
                click(tree, host, x, tabY);
                if (activated.size() > before && !(activated.back() == first))
                    target = activated.back();
            }
        }
        printf("tabs activated by clicks: '%s' then '%s'\n", first.c_str(), other.c_str());
        CHECK(first == rt::String("Assets"));
        CHECK(other == rt::String("Console"));
        CHECK(console->isVisible() && !assets->isVisible());

        rt::SetCurrentContext(rt::GetCurrentContext());
        InputEvent typed;
        typed.type = InputEventType::Char;
        typed.character.codepoint = 'a';
        host.handleEvent(typed);
        typed.character.codepoint = 7;
        host.handleEvent(typed);
        CHECK(rt::GetIO().inputChars.size() == 1);
        InputEvent pressed;
        pressed.type = InputEventType::Key;
        pressed.key.key = Key::Enter;
        pressed.key.scancode = 36;
        pressed.key.pressed = true;
        pressed.key.echo = false;
        pressed.key.modifiers = kModControl | kModShift;
        host.handleEvent(pressed);
        pressed.key.key = Key::None;
        host.handleEvent(pressed);
        CHECK(rt::GetIO().keyEvents.size() == 1);
        if (rt::GetIO().keyEvents.size() == 1)
        {
            CHECK(rt::GetIO().keyEvents[0].key == rt::Key::Return);
            CHECK(rt::GetIO().keyEvents[0].ctrl && rt::GetIO().keyEvents[0].shift &&
                    !rt::GetIO().keyEvents[0].alt && rt::GetIO().keyEvents[0].down);
        }
        InputEvent wheel;
        wheel.type = InputEventType::MouseWheel;
        wheel.mouseWheel.x = 10;
        wheel.mouseWheel.y = 10;
        wheel.mouseWheel.deltaX = 0.0f;
        wheel.mouseWheel.deltaY = 2.0f;
        host.handleEvent(wheel);
        host.handleEvent(wheel);
        CHECK(rt::GetIO().mouseWheelY == 4.0f);
        host.handleEvent(motion(123, 45));
        CHECK(rt::GetIO().mouseX == 123.0f && rt::GetIO().mouseY == 45.0f);
        frame(tree, host);
        CHECK(rt::GetIO().inputChars.empty() && rt::GetIO().keyEvents.empty());
        CHECK(rt::GetIO().mouseWheelY == 0.0f);
        CHECK(rt::GetIO().displayWidth == static_cast<float>(kWidth));
        CHECK(rt::GetIO().displayHeight == static_cast<float>(kHeight));

        host.shutdown();
        CHECK(!host.ready() && WidgetHost::singleton() == nullptr);
        CHECK(host.textureCount() == 0);
        host.frame(1.0f / 60.0f, true);
        CHECK(host.drawCalls() == 0);
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
