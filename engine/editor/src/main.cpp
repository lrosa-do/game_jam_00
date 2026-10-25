#include "prisma3d/main/Main.h"
#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/ConsolePanel.h"
#include "prisma3d/editor/DefaultProject.h"
#include "prisma3d/editor/EditorAbout.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorGizmo.h"
#include "prisma3d/editor/EditorInput.h"
#include "prisma3d/editor/EditorLog.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorPlay.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/GamePanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/LightmapBaker.h"
#include "prisma3d/editor/LightmapPanel.h"
#include "prisma3d/editor/MaterialPanel.h"
#include "prisma3d/editor/McpPanel.h"
#include "prisma3d/editor/McpServer.h"
#include "prisma3d/editor/McpTools.h"
#include "prisma3d/editor/ProjectSettingsEditor.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/ScriptEditorPanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <string.h>

using namespace prisma3d;

namespace
{

const char* const kLegacyLayoutFile = "prisma3d_editor_layout.txt";
const char* const kLegacySettingsFile = "prisma3d_editor_settings.json";

class EditorFrame : public MainListener
{
public:
    EditorFrame(WidgetHost& host, EditorSelection& selection, SceneTreePanel& hierarchy,
            InspectorPanel& inspector, EditorViewportPanel& viewport, ProjectUi& projectUi,
            HierarchyActions& actions, EditorMenus& menus, EditorStatusBar& statusBar,
            AssetsPanel& assets, MaterialPanel& materials, GamePanel& game, EditorPlay& play,
            ConsolePanel& console, ProjectSettingsEditor& projectSettings, McpPanel& mcp,
            LightmapPanel& lightmap, ScriptEditorPanel& scripts)
            : mInput(host, projectUi, actions, viewport),
              mHost(host),
              mSelection(selection),
              mHierarchy(hierarchy),
              mInspector(inspector),
              mProjectUi(projectUi),
              mActions(actions),
              mMenus(menus),
              mStatusBar(statusBar),
              mAssets(assets),
              mMaterials(materials),
              mGame(game),
              mPlay(play),
              mConsole(console),
              mProjectSettings(projectSettings),
              mMcp(mcp),
              mLightmap(lightmap),
              mScripts(scripts),
              mLast(time_nanos())
    {
    }

    bool forwardsToScene(const InputEvent& event) override
    {
        return !mPlay.playing() || mGame.wantsInput(event);
    }

    void inputEvent(const InputEvent& event) override
    {
        if (mPlay.handleEvent(event)) return;
        mGame.handleEvent(event);
        if (mPlay.playing() && mGame.ownsInput(event)) return;
        if (mScripts.handleEvent(event)) return;
        mInput.handle(event);
    }

    void drawOverlay() override
    {
        const std::uint64_t now = time_nanos();
        const float delta = static_cast<float>(now - mLast) * 1e-9f;
        mLast = now;
        Device* device = Main::device();
        if (device && device->closeRequested())
        {
            device->cancelClose();
            mProjectUi.requestQuit();
        }
        if (device)
        {
            const ct::Vector<ct::String> dropped = device->takeDroppedFiles();
            if (!dropped.empty())
            {
                int x = 0;
                int y = 0;
                device->mouseGetPosition(&x, &y);
                mHost.dropFiles(dropped, x, y);
            }
        }
        mMcp.update();
        mProjectUi.update();
        mPlay.update();
        if (Input* input = Input::singleton())
        {
            int mouseX = 0;
            int mouseY = 0;
            if (device) device->mouseGetPosition(&mouseX, &mouseY);
            if (!mPlay.playing()) mGame.releaseFocus();
            input->setPollingEnabled(!mPlay.playing() || mGame.wantsPolling(mouseX, mouseY));
        }
        mSelection.update();
        mSelection.flush();
        mHierarchy.update();
        mInspector.update();
        mActions.update();
        mMenus.update();
        mStatusBar.update(delta);
        mAssets.update(delta);
        mMaterials.update();
        mConsole.update();
        mProjectSettings.update(delta);
        mLightmap.update();
        mScripts.update();
        mHost.frame(delta, true);
    }

private:
    EditorInput mInput;
    WidgetHost& mHost;
    EditorSelection& mSelection;
    SceneTreePanel& mHierarchy;
    InspectorPanel& mInspector;
    ProjectUi& mProjectUi;
    HierarchyActions& mActions;
    EditorMenus& mMenus;
    EditorStatusBar& mStatusBar;
    AssetsPanel& mAssets;
    MaterialPanel& mMaterials;
    GamePanel& mGame;
    EditorPlay& mPlay;
    ConsolePanel& mConsole;
    ProjectSettingsEditor& mProjectSettings;
    McpPanel& mMcp;
    LightmapPanel& mLightmap;
    ScriptEditorPanel& mScripts;
    std::uint64_t mLast;
};

} // namespace

int main(int argc, char** argv)
{
    DeviceDesc desc;
    const ct::String windowTitle = EditorAbout::signature();
    desc.title = windowTitle.c_str();
    desc.width = 1600;
    desc.height = 900;
    const char* projectArgument = nullptr;
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "vulkan") == 0) desc.driver = prisma::DriverType::Vulkan;
        else if (strcmp(argv[i], "opengl") == 0)
            desc.driver = prisma::DriverType::OpenGL;
        else if (argv[i][0] != '-')
            projectArgument = argv[i];
    }

    EditorLog::instance().capture(true);

    const ct::String settingsPath = EditorSettings::defaultPath();
    EditorSettings settings;
    if (!settings.load(settingsPath))
    {
        const bool legacy = settings.loadLegacyJson(kLegacySettingsFile);
        const bool layout = settings.loadLegacyLayout(kLegacyLayoutFile);
        if (legacy || layout) settings.save(settingsPath);
    }
    settings.setPath(settingsPath);
    if (settings.windowWidth > 0 && settings.windowHeight > 0)
    {
        desc.width = settings.windowWidth;
        desc.height = settings.windowHeight;
    }
    if (settings.windowMaximized) desc.mode = prisma3d::WindowMode::Maximized;
    Project project;
    ct::String startup = projectArgument ? ct::String(projectArgument) : settings.lastProjectPath;
    const bool openDefault = DefaultProject::wanted(projectArgument, startup);
    if (openDefault)
    {
        if (!startup.empty()) settings.forgetProject(startup);
        startup = ct::String();
    }

    if (!Main::setup(desc)) return 1;
    if (settings.hasWindowPosition && !settings.windowMaximized && Main::device())
        Main::device()->windowSetPosition(settings.windowX, settings.windowY);
    if (Engine* engine = Engine::singleton()) engine->setEditorHint(true);
    if (!Main::gui()) return 1;

    SceneTree tree;
    Main::start(&tree);
    if (Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr)
        window->setClearColor(Math::Vec4(0.08f, 0.08f, 0.09f, 1.0f));

    Node* scene = nullptr;
    if (tree.getRoot())
    {
        scene = tree.getRoot()->createChild<Node3D>();
        if (scene) scene->setName("Scene");
    }
    if (!scene)
    {
        tree.finalize();
        Main::cleanup();
        return 1;
    }

    if (Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr)
        Main::rendering()->viewportSetActive(window->getViewportId(), false);

    int status = 0;
    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        EditorShell shell(host);
        EditorSelection selection(tree);
        SceneTreePanel hierarchy(tree, selection);
        InspectorPanel inspector(tree, selection);
        EditorViewportPanel viewport(*Main::rendering());
        if (host.init() && shell.build() &&
                hierarchy.attach(shell.panel(EditorShell::kHierarchyPanel)) &&
                inspector.attach(shell.panel(EditorShell::kInspectorPanel)))
        {
            hierarchy.setEditedScene(scene);
            EditorDocument document(tree, selection, hierarchy, project, settings);
            document.setSettingsPath(settingsPath);
            inspector.setChangeListener(&document);
            inspector.setProject(&project);
            inspector.setSettings(&settings);
            hierarchy.setChangeListener(&document);

            ProjectUi projectUi(document);
            projectUi.attach(shell.toolBar());
            projectUi.setTitleHandler(
                    [](const ct::String& title)
                    {
                        if (Device* device = Main::device()) device->windowSetTitle(title.c_str());
                    });
            projectUi.setQuitHandler([&tree] { tree.quit(); });
            tree.setAutoAcceptQuit(false);

            viewport.bindEditor(tree, selection);
            viewport.setCameraAspectProvider(
                    [&project]
                    {
                        if (!project.isOpen()) return 1152.0f / 648.0f;
                        const double width =
                                project.get("display/window/size/viewport_width").asFloat();
                        const double height =
                                project.get("display/window/size/viewport_height").asFloat();
                        return height > 0.0 ? static_cast<float>(width / height) : 1.0f;
                    });
            viewport.setChangeListener(&document);
            viewport.gizmo()->setRootProvider([&document]() -> Node* { return document.scene(); });

            HierarchyActions actions(tree, document, selection, hierarchy);
            actions.attach();
            actions.setViewport(&viewport);
            inspector.properties().setMeshSplitHandler(
                    [&actions](Node* node, const ct::String& mesh,
                            const ct::Vector<ct::String>& pieces)
                    { return actions.splitMesh(node, mesh, pieces); });
            inspector.properties().setMeshRewriteHandler(
                    [&actions](Node* node, const ct::String& file, const ct::String& absolute,
                            const PMeshSurfaceEdit& edit, ct::String& error)
                    { return actions.rewriteMesh(node, file, absolute, edit, error); });
            inspector.setRenameHandler(
                    [&actions](Node* node, const ct::String& name, ct::String& error)
                    { return actions.rename(node, name, error); });

            GamePanel game(*Main::rendering(), document);
            ProjectSettingsEditor projectSettings(document);
            EditorPlay play(tree, document);
            projectUi.setStopHandler([&play] { play.stop(); });

            if (!viewport.attach(shell.panel(EditorShell::kScenePanel)) ||
                    !game.attach(shell.panel(EditorShell::kGamePanel)) ||
                    !play.attach(shell.runBar()))
                status = 1;
            else
            {
                play.setViewHandler(
                        [&shell, &viewport](bool showGame)
                        {
                            viewport.setSuspended(showGame);
                            shell.showPanel(
                                    showGame ? EditorShell::kGamePanel : EditorShell::kScenePanel);
                        });
                projectSettings.bindRendering(*Main::rendering(), &viewport, &game);
                EditorMenus menus(shell, projectUi, viewport, actions, &play);
                menus.setProjectSettings(&projectSettings);
                menus.attach();
                EditorStatusBar statusBar(document, selection, *Main::rendering());
                statusBar.attach(shell.statusBar());
                statusBar.setGizmo(viewport.gizmo());
                statusBar.setSaveHandler([&projectUi] { projectUi.save(); });
                statusBar.captureLog(true);

                ConsolePanel console;
                if (!console.attach(shell.panel(EditorShell::kConsolePanel))) status = 1;
                console.bindSettings(&settings);
                console.captureLog(true);

                MaterialPanel materials(*Main::rendering(), tree, selection, document);
                if (!materials.attach(shell.panel(EditorShell::kMaterialPanel))) status = 1;
                materials.bindViewport(viewport);
                materials.setRevealHandler([&shell] { shell.dock()->showPanel("Material"); });

                AssetsPanel assets(document, selection, hierarchy, actions, projectUi);
                actions.setAssetsChangedHandler([&assets] { assets.refresh(); });
                if (!assets.attach(shell.panel(4))) status = 1;
                assets.bindViewport(viewport);
                assets.bindMaterials(materials);

                LightmapBaker lightmapBaker(document, selection, actions);
                LightmapPanel lightmapPanel(lightmapBaker, document, selection);
                if (!lightmapPanel.attach(shell.panel(EditorShell::kLightmapPanel))) status = 1;
                lightmapPanel.setRevealHandler(
                        [&shell] { shell.showPanel(EditorShell::kLightmapPanel); });
                menus.setLightmapPanel(&lightmapPanel);

                ScriptEditorPanel scripts(document);
                if (!scripts.attach(shell.panel(EditorShell::kScriptPanel))) status = 1;
                scripts.setRevealHandler([&shell] { shell.showPanel(EditorShell::kScriptPanel); });
                assets.bindScripts(scripts);

                McpServer mcpServer;
                McpPanel mcpPanel(mcpServer, settings);
                if (!mcpPanel.attach(shell.panel(EditorShell::kAiPanel))) status = 1;
                McpContext mcpContext = { tree, document, selection, actions, viewport, game, play,
                    materials, assets, shell };
                McpTools mcpTools(mcpContext);
                mcpTools.registerTools(mcpServer);

                if (Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr)
                {
                    viewport.setScenario(window->getScenario());
                    game.setScenario(window->getScenario());
                }
                viewport.connectPreDraw();
                game.connectPreDraw();
                if (!settings.dockLayout.empty()) shell.setLayout(settings.dockLayout);

                if (openDefault)
                {
                    if (!DefaultProject::open(document, EditorSettings::appFolder()))
                    {
                        log_error("editor: could not open the default project: %s",
                                document.error().c_str());
                        document.newScene();
                    }
                    else if (!document.error().empty())
                        log_error("editor: main scene: %s", document.error().c_str());
                }
                else if (!startup.empty() && !document.openProject(startup))
                {
                    log_error("editor: could not open the project '%s': %s", startup.c_str(),
                            document.error().c_str());
                    settings.forgetProject(startup);
                    document.newScene();
                }
                else if (!document.error().empty())
                    log_error("editor: main scene: %s", document.error().c_str());
                for (size_t i = 0; i < project.warnings().size(); ++i)
                    log_info("editor: project: %s", project.warnings()[i].c_str());

                EditorFrame frame(host, selection, hierarchy, inspector, viewport, projectUi,
                        actions, menus, statusBar, assets, materials, game, play, console,
                        projectSettings, mcpPanel, lightmapPanel, scripts);
                Main::setListener(&frame);
                while (!Main::iteration())
                {
                }
                Main::setListener(nullptr);
                mcpServer.stop();
                console.captureLog(false);
                statusBar.captureLog(false);
                settings.dockLayout = shell.layout();
                if (Device* device = Main::device())
                {
                    settings.windowMaximized =
                            device->windowGetMode() == prisma3d::WindowMode::Maximized;
                    if (!settings.windowMaximized)
                    {
                        device->windowGetSize(&settings.windowWidth, &settings.windowHeight);
                        device->windowGetPosition(&settings.windowX, &settings.windowY);
                        settings.hasWindowPosition = true;
                    }
                }
            }
        }
        else
            status = 1;
    }

    settings.save(settingsPath);
    tree.finalize();
    project.close();
    Main::cleanup();
    EditorLog::instance().capture(false);
    return status;
}
