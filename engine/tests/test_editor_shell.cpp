#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
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

const int kWidth = 800;
const int kHeight = 600;

void frame(SceneTree& tree, WidgetHost& host)
{
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    host.frame(1.0f / 60.0f, true);
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

struct CountingListener : MainListener
{
    int inputs = 0;
    int overlays = 0;

    void inputEvent(const InputEvent&) override
    {
        ++inputs;
    }

    void drawOverlay() override
    {
        ++overlays;
    }
};

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor shell test";
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

    {
        CountingListener listener;
        CHECK(Main::listener() == nullptr);
        Main::setListener(&listener);
        CHECK(Main::listener() == &listener);
        Input* input = Input::singleton();
        CHECK(input != nullptr);
        if (input) input->parseInputEvent(motion(10, 10));
        CHECK(listener.inputs == 1);
        Main::setListener(nullptr);
        if (input) input->parseInputEvent(motion(20, 20));
        CHECK(listener.inputs == 1);
        CHECK(listener.overlays == 0);
    }

    char layoutFile[512];
    snprintf(layoutFile, sizeof layoutFile, "%s/editor_shell_layout_%s.txt", PRISMA3D_TEST_DIR,
            vulkan ? "vulkan" : "gl");

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        EditorShell early(host);
        CHECK(!early.build());
        CHECK(host.init());

        EditorShell shell(host);
        CHECK(!shell.built());
        CHECK(shell.layout().empty());
        CHECK(!shell.saveLayout(layoutFile));
        CHECK(shell.build());
        CHECK(shell.built());
        CHECK(shell.build());
        CHECK(shell.dock() != nullptr && shell.toolBar() != nullptr);
        CHECK(shell.resetLayoutButton() >= 0);
        CHECK(strcmp(EditorShell::panelName(0), "Hierarchy") == 0);
        CHECK(strcmp(EditorShell::panelName(EditorShell::kScenePanel), "Scene") == 0);
        CHECK(strcmp(EditorShell::panelName(4), "Assets") == 0);
        CHECK(strcmp(EditorShell::panelName(5), "Material") == 0);
        CHECK(strcmp(EditorShell::panelName(EditorShell::kGamePanel), "Game") == 0);
        CHECK(strcmp(EditorShell::panelName(EditorShell::kAiPanel), "AI") == 0);
        CHECK(strcmp(EditorShell::panelName(EditorShell::kLightmapPanel), "Lightmap") == 0);
        CHECK(strcmp(EditorShell::panelName(EditorShell::kPanelCount), "") == 0);
        CHECK(EditorShell::kPanelCount == 10);
        for (int i = 0; i < EditorShell::kPanelCount; ++i) CHECK(shell.panel(i) != nullptr);
        CHECK(shell.panel(-1) == nullptr && shell.panel(EditorShell::kPanelCount) == nullptr);

        frame(tree, host);
        frame(tree, host);
        CHECK(host.drawCalls() > 0);

        const rt::String defaults = shell.layout();
        CHECK(!defaults.empty());
        for (int i = 0; i < EditorShell::kPanelCount; ++i)
            CHECK(strstr(defaults.c_str(), EditorShell::panelName(i)) != nullptr);

        const rt::Rect hierarchy = shell.panel(0)->rect();
        const rt::Rect viewport = shell.panel(1)->rect();
        const rt::Rect inspector = shell.panel(2)->rect();
        const rt::Rect console = shell.panel(3)->rect();
        CHECK(hierarchy.x < viewport.x && viewport.x < inspector.x);
        CHECK(console.y > viewport.y);
        CHECK(viewport.w > hierarchy.w && viewport.w > inspector.w);
        CHECK(shell.panel(3)->isVisible() && !shell.panel(4)->isVisible());
        CHECK(!shell.panel(EditorShell::kMaterialPanel)->isVisible() &&
                shell.panel(2)->isVisible());
        CHECK(strstr(defaults.c_str(), "{Inspector,Material,Lightmap}") != nullptr);
        CHECK(strstr(defaults.c_str(), "{Scene,Game,Script}") != nullptr);
        CHECK(shell.panel(EditorShell::kScenePanel)->isVisible());
        CHECK(!shell.panel(EditorShell::kGamePanel)->isVisible());
        CHECK(shell.runBar() != nullptr);
        if (shell.runBar())
        {
            const rt::Rect run = shell.runBar()->absoluteRect();
            const rt::Rect row = shell.column()->absoluteRect();
            CHECK(fabsf((run.x + run.w * 0.5f) - (row.x + row.w * 0.5f)) < 1.5f);
            CHECK(run.y == shell.toolBar()->absoluteRect().y);
        }
        CHECK(shell.toolBar()->absoluteRect().y < shell.dock()->absoluteRect().y);
        CHECK(shell.toolBar()->isItemVisible(shell.resetLayoutButton()));
        CHECK(shell.toolBar()->isItemEnabled(shell.resetLayoutButton()));

        shell.dock()->moveTabToLeaf("Assets", "Hierarchy");
        shell.dock()->layout();
        frame(tree, host);
        const rt::String changed = shell.layout();
        CHECK(!(changed == defaults));
        CHECK(shell.saveLayout(layoutFile));

        const rt::Rect bar = shell.toolBar()->absoluteRect();
        bool reset = false;
        for (int x = static_cast<int>(bar.x) + 4; x < static_cast<int>(bar.x) + 80 && !reset;
                x += 4)
        {
            click(tree, host, x, static_cast<int>(bar.y + bar.h * 0.5f));
            reset = shell.layout() == defaults;
        }
        CHECK(reset);
        frame(tree, host);
        CHECK(shell.panel(3)->isVisible());

        CHECK(shell.loadLayout(layoutFile));
        CHECK(shell.layout() == changed);
        shell.resetLayout();
        CHECK(shell.layout() == defaults);
        CHECK(!shell.setLayout("not a layout"));
        CHECK(!shell.setLayout(rt::String()));
        CHECK(shell.layout() == defaults);
        CHECK(!shell.loadLayout("/nonexistent/prisma3d/editor_layout.txt"));
        CHECK(!shell.loadLayout(nullptr));
        CHECK(shell.setLayout(changed));
        frame(tree, host);
        CHECK(shell.layout() == changed);

        shell.resetLayout();
        rt::String legacy = defaults;
        const char* const gameTab = ",Game,Script";
        const char* const sceneTab = "Scene";
        const char* at = strstr(legacy.c_str(), gameTab);
        CHECK(at != nullptr);
        if (at)
            legacy = legacy.substr(0, static_cast<size_t>(at - legacy.c_str())) +
                     legacy.substr(static_cast<size_t>(at - legacy.c_str()) + strlen(gameTab));
        at = strstr(legacy.c_str(), sceneTab);
        CHECK(at != nullptr);
        if (at)
            legacy = legacy.substr(0, static_cast<size_t>(at - legacy.c_str())) + "Viewport" +
                     legacy.substr(static_cast<size_t>(at - legacy.c_str()) + strlen(sceneTab));
        CHECK(strstr(legacy.c_str(), "Viewport") != nullptr &&
                strstr(legacy.c_str(), "Game") == nullptr &&
                strstr(legacy.c_str(), "Script") == nullptr);
        CHECK(shell.setLayout(legacy));
        frame(tree, host);
        CHECK(shell.layout() == defaults);
        CHECK(shell.panel(EditorShell::kScenePanel)->isVisible());
        CHECK(!shell.panel(EditorShell::kGamePanel)->isVisible());
        fs_remove_path(layoutFile);

        host.shutdown();
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);
    CHECK(Main::listener() == nullptr);

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
