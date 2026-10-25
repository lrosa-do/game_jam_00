#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorPlay.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/GamePanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/script/ZenScripting.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(_WIN32)
#include <sys/stat.h>
#endif

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_play_test_vulkan" : "/editor_play_test";
    return root;
}

const char kTicker[] = R"(class Ticker(Node3D):
    ticks = 0
    inputs = 0
    def _ready(self):
        print("ready")
    def _process(self, delta):
        self.ticks += 1
    def _input(self, event):
        self.inputs += 1
    def _exit_tree(self):
        print("exit")
)";

ct::Vector<ct::String> printed;

void collectPrint(const char* line, void*)
{
    printed.push_back(ct::String(line));
}

ct::String printedLines()
{
    ct::String text;
    for (std::size_t i = 0; i < printed.size(); ++i)
    {
        if (i > 0) text += ",";
        text += printed[i];
    }
    return text;
}

struct Rig
{
    SceneTree& tree;
    WidgetHost& host;
    EditorShell& shell;
    EditorSelection& selection;
    SceneTreePanel& hierarchy;
    EditorDocument& document;
    ProjectUi& ui;
    HierarchyActions& actions;
    EditorViewportPanel& viewport;
    GamePanel& game;
    EditorPlay& play;
    EditorMenus& menus;
};

void frame(Rig& rig)
{
    rig.tree.process(1.0 / 60.0);
    rig.tree.physicsProcess(1.0 / 60.0);
    rig.ui.update();
    rig.play.update();
    rig.menus.update();
    rig.selection.update();
    rig.selection.flush();
    rig.hierarchy.update();
    Main::rendering()->renderViewports();
    rig.host.frame(1.0f / 60.0f, true);
    Main::rendering()->endFrame(true);
}

void frames(Rig& rig, int count)
{
    for (int i = 0; i < count; ++i) frame(rig);
}

InputEvent keyEvent(Key key, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = key;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = 0;
    return event;
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

InputEvent press(int x, int y, bool pressed)
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

ct::String serialize(Node* scene)
{
    ct::String text;
    SceneResult result;
    CHECK(SceneSerializer::save(scene, text, &result));
    return text;
}

int ticksOf(Node* node)
{
    bool valid = false;
    const PropertyValue value = node->get("ticks", &valid);
    CHECK(valid);
    return static_cast<int>(value.asInt());
}

int inputsOf(Node* node)
{
    bool valid = false;
    const PropertyValue value = node->get("inputs", &valid);
    CHECK(valid);
    return static_cast<int>(value.asInt());
}

Node* find(Rig& rig, const char* path)
{
    return rig.document.scene() ? rig.document.scene()->getNode(path) : nullptr;
}

struct ViewLog
{
    int shown = 0;
    bool lastGame = false;
};

class FilterListener : public MainListener
{
public:
    bool forwards = true;
    int inputs = 0;

    bool forwardsToScene(const InputEvent&) override
    {
        return forwards;
    }

    void inputEvent(const InputEvent&) override
    {
        ++inputs;
    }
};

void testPlayStop(Rig& rig, ViewLog& views)
{
    Engine* engine = Engine::singleton();
    CHECK(engine != nullptr);
    CHECK(engine->isEditorHint());
    CHECK(PhysicsServer3D::singleton() != nullptr && !PhysicsServer3D::singleton()->isActive());
    CHECK(rig.play.state() == EditorPlay::kStopped);
    CHECK(!rig.play.playing() && !rig.document.playing());
    CHECK(rig.play.canPlay() && !rig.play.canPause() && !rig.play.canStop());

    rt::ToolBar* bar = rig.shell.runBar();
    CHECK(bar != nullptr);
    CHECK(bar->isItemEnabled(rig.play.buttonId(EditorPlay::kPlay)));
    CHECK(!bar->isItemEnabled(rig.play.buttonId(EditorPlay::kPause)));
    CHECK(!bar->isItemEnabled(rig.play.buttonId(EditorPlay::kStop)));

    Node* pivot = find(rig, "Pivot");
    CHECK(pivot != nullptr);
    frames(rig, 3);
    CHECK(pivot && ticksOf(pivot) == 0);

    rig.selection.clear();
    rig.selection.add(find(rig, "Pivot/Child"));
    rig.selection.flush();
    frame(rig);

    Node* editScene = rig.document.scene();
    const std::size_t childCount = editScene->getChildCount();
    const ct::String before = serialize(editScene);
    CHECK(!rig.document.dirty());
    const unsigned int serialBefore = rig.document.sceneSerial();

    CHECK(printedLines().empty());
    CHECK(rig.play.play());
    CHECK(rig.play.state() == EditorPlay::kPlaying);
    CHECK(rig.play.playing() && rig.document.playing());
    CHECK(PhysicsServer3D::singleton()->isActive());
    CHECK(!engine->isEditorHint());
    CHECK(printedLines() == "ready");
    CHECK(views.shown == 1 && views.lastGame);
    CHECK(rig.document.sceneSerial() != serialBefore);
    CHECK(rig.document.scene() != editScene);
    CHECK(rig.document.scene() != nullptr && rig.document.scene()->getName() == "Scene");
    CHECK(serialize(rig.document.scene()) == before);
    CHECK(bar->isItemChecked(rig.play.buttonId(EditorPlay::kPlay)));
    CHECK(bar->isItemEnabled(rig.play.buttonId(EditorPlay::kPause)));
    CHECK(bar->isItemEnabled(rig.play.buttonId(EditorPlay::kStop)));
    CHECK(!bar->isItemEnabled(rig.play.buttonId(EditorPlay::kRun)));
    CHECK(rig.selection.selectedNodes().size() == 1 &&
            rig.selection.selectedNodes()[0] == find(rig, "Pivot/Child"));

    frames(rig, 3);
    CHECK(printedLines() == "ready");
    Node* playPivot = find(rig, "Pivot");
    CHECK(playPivot != nullptr && playPivot != pivot);
    const int ticks = playPivot ? ticksOf(playPivot) : -1;
    CHECK(ticks >= 3);

    Node3D* movable = playPivot ? playPivot->castTo<Node3D>() : nullptr;
    if (movable) movable->setPosition(Math::Vec3(9.0f, 9.0f, 9.0f));
    if (playPivot) playPivot->createChild<Node3D>()->setName("Spawned");
    playPivot->setName("Renamed");
    frames(rig, 2);
    CHECK(!rig.document.dirty());
    rig.document.markDirty();
    CHECK(!rig.document.dirty());
    rig.document.sceneEdited();
    CHECK(!rig.document.dirty());
    playPivot->setName("Pivot");

    rig.play.pause();
    CHECK(rig.play.state() == EditorPlay::kPaused && rig.play.paused());
    CHECK(rig.tree.isPaused());
    CHECK(bar->isItemChecked(rig.play.buttonId(EditorPlay::kPause)));
    const int frozen = ticksOf(playPivot);
    frames(rig, 5);
    CHECK(ticksOf(playPivot) == frozen);
    rig.play.pause();
    CHECK(rig.play.state() == EditorPlay::kPlaying && !rig.tree.isPaused());
    frames(rig, 3);
    CHECK(ticksOf(playPivot) > frozen);

    CHECK(rig.play.play());
    CHECK(rig.play.playing());
    CHECK(views.shown == 2);
    playPivot = find(rig, "Pivot");
    CHECK(playPivot != nullptr && ticksOf(playPivot) == 0);
    CHECK(rig.document.scene()->getChildCount() == childCount);
    frames(rig, 2);

    printed.clear();
    CHECK(rig.play.stop());
    CHECK(rig.play.state() == EditorPlay::kStopped);
    CHECK(!rig.document.playing());
    CHECK(engine->isEditorHint());
    CHECK(!PhysicsServer3D::singleton()->isActive());
    frame(rig);
    CHECK(engine->isEditorHint());
    CHECK(printedLines() == "exit");
    frames(rig, 3);
    CHECK(printedLines() == "exit");
    CHECK(!rig.tree.isPaused());
    CHECK(views.shown == 3 && !views.lastGame);
    frame(rig);
    CHECK(rig.document.scene() == editScene);
    CHECK(find(rig, "Pivot") == pivot);
    CHECK(serialize(rig.document.scene()) == before);
    CHECK(!rig.document.dirty());
    CHECK(rig.selection.selectedNodes().size() == 1 &&
            rig.selection.selectedNodes()[0] == find(rig, "Pivot/Child"));
    CHECK(rig.hierarchy.editedScene() == rig.document.scene());
    CHECK(!bar->isItemChecked(rig.play.buttonId(EditorPlay::kPlay)));
    CHECK(!bar->isItemEnabled(rig.play.buttonId(EditorPlay::kStop)));
    CHECK(!bar->isItemChecked(rig.play.buttonId(EditorPlay::kPause)));
    CHECK(rig.play.stop());

    Node* restored = find(rig, "Pivot");
    CHECK(restored != nullptr && ticksOf(restored) == 0);
    frames(rig, 5);
    CHECK(ticksOf(restored) == 0);
    Node3D* pose = restored ? restored->castTo<Node3D>() : nullptr;
    CHECK(pose != nullptr && pose->getPosition() == Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(pose != nullptr && pose->getScale() == Math::Vec3(1.0f, 2.0f, 1.0f));
    CHECK(restored != nullptr && restored->getChildCount() == 1 &&
            restored->getChild(0)->getName() == "Child");
    CHECK(restored != nullptr && restored->getScript().contains("ticker.py"));

    rig.document.markDirty();
    CHECK(rig.document.dirty());
    CHECK(rig.play.play());
    frames(rig, 2);
    CHECK(rig.document.dirty());
    CHECK(rig.play.stop());
    CHECK(rig.document.dirty());
    CHECK(serialize(rig.document.scene()) == before);
}

void testEditingDisabled(Rig& rig)
{
    Node* child = find(rig, "Pivot/Child");
    rig.selection.clear();
    rig.selection.add(child);
    rig.selection.flush();
    frame(rig);
    CHECK(rig.actions.canAddChild() && rig.actions.canDelete() && rig.actions.canRename() &&
            rig.actions.canDuplicate() && rig.actions.canReparent());

    CHECK(rig.play.play());
    frames(rig, 2);
    child = find(rig, "Pivot/Child");
    rig.selection.clear();
    rig.selection.add(child);
    rig.selection.flush();
    frame(rig);

    CHECK(!rig.actions.canAddChild());
    CHECK(!rig.actions.canDelete());
    CHECK(!rig.actions.canRename());
    CHECK(!rig.actions.canDuplicate());
    CHECK(!rig.actions.canReparent());
    CHECK(!rig.actions.canMoveUp() && !rig.actions.canMoveDown());
    ct::String error;
    const std::size_t count = rig.document.scene()->getChildCount();
    CHECK(!rig.actions.deleteSelection(error));
    CHECK(!rig.actions.duplicateSelection(error));
    CHECK(!rig.actions.createNode("Node3D", error));
    CHECK(rig.document.scene()->getChildCount() == count);
    CHECK(find(rig, "Pivot/Child") == child);

    CHECK(!rig.document.newScene());
    CHECK(!rig.document.saveScene());
    CHECK(!rig.document.saveSceneAs("nope"));
    CHECK(!rig.document.openScene("nope.scene.json"));
    CHECK(!rig.document.openProject("nope/projecto.json"));
    CHECK(!rig.document.saveProject());
    CHECK(!rig.document.setMainScene());
    CHECK(rig.document.error() == "stop the game first");
    rig.ui.newScene();
    rig.ui.save();
    rig.ui.openSceneFile();
    frames(rig, 2);
    CHECK(!rig.ui.confirming());
    CHECK(rig.ui.fileDialog() == nullptr);

    rt::ToolBar* toolBar = rig.shell.toolBar();
    CHECK(!toolBar->isItemEnabled(rig.ui.buttonId(ProjectUi::kNewScene)));
    CHECK(!toolBar->isItemEnabled(rig.ui.buttonId(ProjectUi::kOpenScene)));
    CHECK(!toolBar->isItemEnabled(rig.ui.buttonId(ProjectUi::kNewProject)));
    CHECK(!toolBar->isItemEnabled(rig.ui.buttonId(ProjectUi::kOpenProject)));
    CHECK(!rig.menus.action(EditorMenus::kFile, EditorMenus::kFileSave)->isEnabled());
    CHECK(!rig.menus.action(EditorMenus::kFile, EditorMenus::kFileNewProject)->isEnabled());
    CHECK(!rig.menus.action(EditorMenus::kScene, EditorMenus::kSceneNew)->isEnabled());
    CHECK(!rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditDelete)->isEnabled());
    CHECK(!rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditAddChild)->isEnabled());

    rt::Menu* run = rig.menus.menu(EditorMenus::kRun);
    CHECK(run != nullptr && run->actionCount() == EditorMenus::kRunItemCount);
    CHECK(run->action(EditorMenus::kRunPlay)->isEnabled());
    CHECK(run->action(EditorMenus::kRunPause)->isEnabled());
    CHECK(run->action(EditorMenus::kRunStop)->isEnabled());
    CHECK(!run->action(EditorMenus::kRunInRunner)->isEnabled());
    CHECK(!run->action(EditorMenus::kRunStopRunner)->isEnabled());

    CHECK(rig.play.stop());
    frames(rig, 2);
    CHECK(rig.actions.canDelete() && rig.actions.canAddChild());
    CHECK(toolBar->isItemEnabled(rig.ui.buttonId(ProjectUi::kNewScene)));
    CHECK(rig.menus.action(EditorMenus::kFile, EditorMenus::kFileSave)->isEnabled());
    CHECK(rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditDelete)->isEnabled());
    CHECK(!run->action(EditorMenus::kRunStop)->isEnabled());
    CHECK(run->action(EditorMenus::kRunPlay)->isEnabled());
}

void testShortcuts(Rig& rig, ViewLog& views)
{
    rt::ToolBar* bar = rig.shell.runBar();
    const int shown = views.shown;

    CHECK(!rig.play.handleEvent(keyEvent(Key::F5, false)));
    InputEvent withControl = keyEvent(Key::F5, true);
    withControl.key.modifiers = kModControl;
    CHECK(!rig.play.handleEvent(withControl));
    CHECK(!rig.play.playing());
    CHECK(!rig.play.handleEvent(keyEvent(Key::A, true)));

    CHECK(rig.play.handleEvent(keyEvent(Key::F5, true)));
    CHECK(rig.play.state() == EditorPlay::kPlaying);
    CHECK(views.shown == shown + 1 && views.lastGame);
    CHECK(rig.play.handleEvent(keyEvent(Key::F7, true)));
    CHECK(rig.play.state() == EditorPlay::kPaused);
    CHECK(rig.menus.action(EditorMenus::kRun, EditorMenus::kRunPause) != nullptr);
    frame(rig);
    CHECK(rig.menus.action(EditorMenus::kRun, EditorMenus::kRunPause)->isChecked());
    CHECK(rig.play.handleEvent(keyEvent(Key::F7, true)));
    CHECK(rig.play.state() == EditorPlay::kPlaying);
    CHECK(rig.play.handleEvent(keyEvent(Key::F8, true)));
    CHECK(rig.play.state() == EditorPlay::kStopped);
    CHECK(views.shown == shown + 2 && !views.lastGame);

    rig.play.pause();
    CHECK(rig.play.state() == EditorPlay::kStopped && !rig.tree.isPaused());
    CHECK(rig.play.stop());

    bar->toggleChanged.emit(rig.play.buttonId(EditorPlay::kPlay), true);
    CHECK(rig.play.state() == EditorPlay::kPlaying);
    bar->toggleChanged.emit(rig.play.buttonId(EditorPlay::kPause), true);
    CHECK(rig.play.state() == EditorPlay::kPaused);
    bar->toggleChanged.emit(rig.play.buttonId(EditorPlay::kPause), false);
    CHECK(rig.play.state() == EditorPlay::kPlaying);
    bar->buttonClicked.emit(rig.play.buttonId(EditorPlay::kStop));
    CHECK(rig.play.state() == EditorPlay::kStopped);

    CHECK(strcmp(EditorPlay::buttonTooltip(EditorPlay::kPlay), "Play (F5)") == 0);
    CHECK(strcmp(EditorPlay::buttonTooltip(EditorPlay::kPause), "Pause (F7)") == 0);
    CHECK(strcmp(EditorPlay::buttonTooltip(EditorPlay::kStop), "Stop (F8)") == 0);
    CHECK(strcmp(EditorPlay::buttonTooltip(EditorPlay::kRun), "Run in standalone window (F6)") ==
            0);
}

void testInputGate(Rig& rig)
{
    FilterListener listener;
    Main::setListener(&listener);
    Input* input = Input::singleton();
    CHECK(input != nullptr);

    CHECK(rig.play.play());
    frames(rig, 2);
    Node* pivot = find(rig, "Pivot");
    CHECK(pivot != nullptr && pivot->isProcessingInput());
    const int base = inputsOf(pivot);

    input->parseInputEvent(keyEvent(Key::A, true));
    CHECK(inputsOf(pivot) == base + 1);
    listener.forwards = false;
    input->parseInputEvent(keyEvent(Key::A, true));
    input->parseInputEvent(keyEvent(Key::B, true));
    CHECK(inputsOf(pivot) == base + 1);
    CHECK(listener.inputs == 3);
    listener.forwards = true;
    input->parseInputEvent(keyEvent(Key::B, true));
    CHECK(inputsOf(pivot) == base + 2);

    Main::setListener(nullptr);
    CHECK(rig.play.stop());

    CHECK(!rig.game.wantsInput(keyEvent(Key::A, true)));
    rig.shell.showPanel(EditorShell::kGamePanel);
    frames(rig, 3);
    rt::Widget* view = rig.game.view();
    CHECK(view != nullptr);
    if (!view) return;
    const rt::Rect rect = view->absoluteRect();
    const int insideX = static_cast<int>(rect.x + rect.w * 0.5f);
    const int insideY = static_cast<int>(rect.y + rect.h * 0.5f);

    CHECK(rig.game.wantsInput(motion(insideX, insideY)));
    CHECK(!rig.game.wantsInput(motion(2, 2)));
    CHECK(!rig.game.wantsInput(keyEvent(Key::A, true)));
    rig.game.handleEvent(motion(insideX, insideY));
    CHECK(rig.game.wantsInput(keyEvent(Key::A, true)));
    rig.game.handleEvent(motion(2, 2));
    CHECK(!rig.game.wantsInput(keyEvent(Key::A, true)));
    rig.game.handleEvent(press(insideX, insideY, true));
    CHECK(rig.game.focused());
    rig.game.handleEvent(motion(2, 2));
    CHECK(rig.game.wantsInput(keyEvent(Key::A, true)));
    CHECK(rig.game.wantsInput(motion(2, 2)));
    rig.game.handleEvent(press(insideX, insideY, false));
    rig.game.handleEvent(press(2, 2, true));
    CHECK(!rig.game.focused());
    rig.game.handleEvent(press(2, 2, false));
    CHECK(!rig.game.wantsInput(keyEvent(Key::A, true)));

    rig.shell.showPanel(EditorShell::kScenePanel);
    frames(rig, 3);
    CHECK(!rig.game.wantsInput(motion(insideX, insideY)));
}

void testGamePanel(Rig& rig)
{
    CHECK(GamePanel::findCamera(nullptr) == nullptr);
    CHECK(strcmp(GamePanel::kNoCameraMessage, "No camera in the scene") == 0);

    rig.shell.showPanel(EditorShell::kScenePanel);
    frames(rig, 4);
    CHECK(rig.viewport.rendering());
    CHECK(!rig.game.rendering());
    CHECK(rig.game.view() != nullptr);

    rig.shell.showPanel(EditorShell::kGamePanel);
    frames(rig, 4);
    CHECK(rig.game.rendering());
    CHECK(!rig.viewport.rendering());
    CHECK(rig.game.width() > 100 && rig.game.height() > 100);
    CHECK(rig.game.width() == static_cast<std::uint32_t>(rig.game.view()->absoluteRect().w + 0.5f));

    Camera3D* main = find(rig, "Cam") ? find(rig, "Cam")->castTo<Camera3D>() : nullptr;
    Camera3D* second = find(rig, "Cam2") ? find(rig, "Cam2")->castTo<Camera3D>() : nullptr;
    CHECK(main != nullptr && second != nullptr);
    if (!main || !second) return;
    CHECK(GamePanel::findCamera(rig.document.scene()) == main);
    CHECK(rig.game.camera() == main);
    CHECK(rig.game.cameraId().valid() && rig.game.cameraId() == main->getCameraId());
    CHECK(rig.game.cameraId() != rig.viewport.cameraId());
    CHECK(!rig.game.showingMessage());
    CHECK(rig.game.caption().contains("Cam"));
    CHECK(rig.game.shownTexture().value != 0);
    CHECK(rig.game.texture() == Main::rendering()->viewportGetTexture(rig.game.viewportId()));

    second->makeCurrent();
    frames(rig, 2);
    CHECK(GamePanel::findCamera(rig.document.scene()) == second);
    CHECK(rig.game.camera() == second);
    CHECK(rig.game.caption().contains("Cam2"));
    main->makeCurrent();
    frames(rig, 2);
    CHECK(rig.game.camera() == main);

    CHECK(rig.play.play());
    frames(rig, 3);
    CHECK(rig.game.camera() != nullptr && rig.game.camera() != main);
    CHECK(rig.game.camera() == GamePanel::findCamera(rig.document.scene()));
    CHECK(rig.game.rendering());
    CHECK(rig.play.stop());
    rig.shell.showPanel(EditorShell::kGamePanel);
    frames(rig, 3);
    CHECK(rig.game.camera() != nullptr && rig.game.rendering());

    ct::Vector<Node*> cameras;
    cameras.push_back(find(rig, "Cam"));
    cameras.push_back(find(rig, "Cam2"));
    for (std::size_t i = 0; i < cameras.size(); ++i)
        if (cameras[i]) cameras[i]->queueFree();
    frames(rig, 3);
    CHECK(GamePanel::findCamera(rig.document.scene()) == nullptr);
    CHECK(rig.game.camera() == nullptr);
    CHECK(rig.game.showingMessage());
    CHECK(rig.game.message() == "No camera in the scene");
    CHECK(!rig.game.rendering());
    CHECK(rig.game.shownTexture().value == 0);

    Camera3D* fresh = rig.document.scene()->createChild<Camera3D>();
    fresh->setName("Fresh");
    frames(rig, 3);
    CHECK(rig.game.camera() == fresh);
    CHECK(!rig.game.showingMessage());
    CHECK(rig.game.rendering());
    CHECK(rig.game.caption().contains("Fresh"));

    rig.shell.showPanel(EditorShell::kScenePanel);
    frames(rig, 3);
    CHECK(!rig.game.rendering());
    CHECK(rig.viewport.rendering());
}

void writeExecutable(const ct::String& path, const ct::String& text)
{
    CHECK(file_write_text(path.c_str(), text.c_str()));
#if !defined(_WIN32)
    CHECK(chmod(path.c_str(), 0755) == 0);
#endif
}

bool waitForRunner(Rig& rig, bool running)
{
    for (int i = 0; i < 400; ++i)
    {
        frame(rig);
        if (rig.play.runnerRunning() == running)
        {
            frame(rig);
            return true;
        }
        time_sleep(10);
    }
    return false;
}

void testRunner(Rig& rig)
{
#if defined(_WIN32)
    (void) rig;
#else
    const ct::String root = testRoot();
    const ct::String argsFile = root + "/runner_args.txt";
    const ct::String exiting = root + "/fake_exit.sh";
    const ct::String sleeping = root + "/fake_sleep.sh";
    writeExecutable(exiting, ct::String("#!/bin/sh\necho \"$1\" > ") + argsFile + "\nexit 3\n");
    writeExecutable(sleeping, "#!/bin/sh\nexec sleep 30\n");

    rig.play.setRunnerPath(root + "/does_not_exist");
    CHECK(rig.play.runTarget().empty() || rig.document.project().isOpen());
    ct::String error;
    CHECK(rig.document.createProject(root + "/projects", "runnerproj"));
    CHECK(rig.document.project().isOpen());
    CHECK(rig.play.runTarget() == rig.document.project().manifestPath());
    CHECK(rig.play.canRun());
    CHECK(!rig.play.run());
    CHECK(!rig.play.runnerRunning() && rig.play.runnerLaunches() == 0);

    Node* extra = rig.document.scene()->createChild<Node3D>();
    extra->setName("Unsaved");
    frame(rig);
    CHECK(rig.document.dirty());

    rig.play.setRunnerPath(exiting);
    CHECK(rig.play.runnerPath() == exiting);
    CHECK(rig.play.run());
    CHECK(rig.play.runnerLaunches() == 1);
    CHECK(!rig.document.dirty());
    {
        char* saved = file_read_text(
                rig.document.project().absolutePath(rig.document.scenePath()).c_str());
        CHECK(saved != nullptr && strstr(saved, "Unsaved") != nullptr);
        free(saved);
    }
    CHECK(waitForRunner(rig, false));
    CHECK(rig.play.runnerExitCode() == 3);
    {
        char* args = file_read_text(argsFile.c_str());
        CHECK(args != nullptr);
        if (args)
        {
            ct::String text(args);
            free(args);
            CHECK(text.trimmed() == rig.document.project().manifestPath());
        }
    }

    rig.play.setRunnerPath(sleeping);
    CHECK(rig.play.run());
    CHECK(rig.play.runnerLaunches() == 2);
    CHECK(rig.play.runnerRunning());
    CHECK(!rig.play.run());
    CHECK(rig.play.runnerLaunches() == 2);
    frame(rig);
    rt::ToolBar* bar = rig.shell.runBar();
    CHECK(!bar->isItemEnabled(rig.play.buttonId(EditorPlay::kRun)));
    CHECK(bar->isItemEnabled(rig.play.buttonId(EditorPlay::kStop)));
    CHECK(rig.play.canStop());
    rt::Menu* menu = rig.menus.menu(EditorMenus::kRun);
    CHECK(!menu->action(EditorMenus::kRunInRunner)->isEnabled());
    CHECK(menu->action(EditorMenus::kRunStopRunner)->isEnabled());
    CHECK(!rig.play.play() || rig.play.stop());
    CHECK(rig.play.stop());
    CHECK(!rig.play.runnerRunning());
    frame(rig);
    CHECK(bar->isItemEnabled(rig.play.buttonId(EditorPlay::kRun)));
    CHECK(!bar->isItemEnabled(rig.play.buttonId(EditorPlay::kStop)));

    CHECK(rig.play.run());
    CHECK(rig.play.runnerRunning());
    CHECK(rig.play.play());
    CHECK(rig.play.playing());
    CHECK(!rig.play.run());
    CHECK(rig.play.stop());

    CHECK(rig.play.run());
    rig.play.stopRunner();
    CHECK(!rig.play.runnerRunning());

    rig.play.setRunnerPath(PRISMA3D_RUNNER_PATH);
    const ct::String manifest = rig.document.project().manifestPath();
    CHECK(file_exists(PRISMA3D_RUNNER_PATH));

    const auto runHeadless = [&](const char* target, const char* frameCount)
    {
        const char* const arguments[] = { target, "--headless", "--frames", frameCount,
            useVulkan ? "vulkan" : "--headless" };
        const std::int64_t id = process_spawn(PRISMA3D_RUNNER_PATH, arguments, 5, false);
        CHECK(id > 0);
        if (id <= 0) return -2;
        for (int i = 0; i < 3000 && process_is_running(id); ++i) time_sleep(10);
        if (process_is_running(id))
        {
            process_kill(id);
            return -3;
        }
        return process_exit_code(id);
    };
    CHECK(runHeadless(manifest.c_str(), "5") == 0);
    CHECK(runHeadless((root + "/projects/runnerproj/missing.json").c_str(), "5") == 1);
    const ct::String scenePath = rig.document.project().absolutePath(rig.document.scenePath());
    CHECK(runHeadless(scenePath.c_str(), "5") == 0);
    CHECK(runHeadless((root + "/nothing.scene.json").c_str(), "5") == 1);
    const char* const badArguments[] = { "--bogus" };
    const std::int64_t bad = process_spawn(PRISMA3D_RUNNER_PATH, badArguments, 1, false);
    CHECK(bad > 0);
    for (int i = 0; i < 1000 && process_is_running(bad); ++i) time_sleep(10);
    CHECK(process_exit_code(bad) == 2);
#endif
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    removeTree(testRoot());
    CHECK(fs_create_directory(testRoot().c_str()));
    CHECK(fs_create_directory((testRoot() + "/projects").c_str()));

    DeviceDesc desc;
    desc.title = "editor play test";
    desc.driver = useVulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;
    Engine::singleton()->setEditorHint(true);
    CHECK(ZenScripting::instance() != nullptr);
    if (ZenScripting::instance()) ZenScripting::instance()->setPrintHandler(&collectPrint, nullptr);

    SceneTree tree;
    CHECK(Main::start(&tree));
    tree.setAutoAcceptQuit(false);
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (window) Main::rendering()->viewportSetActive(window->getViewportId(), false);

    const ct::String ticker = testRoot() + "/ticker.py";
    CHECK(file_write_text(ticker.c_str(), kTicker));

    Node3D* scene = tree.getRoot() ? tree.getRoot()->createChild<Node3D>() : nullptr;
    CHECK(scene != nullptr);
    if (scene && window)
    {
        scene->setName("Scene");
        Node3D* pivot = scene->createChild<Node3D>();
        pivot->setName("Pivot");
        pivot->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
        pivot->setScale(Math::Vec3(1.0f, 2.0f, 1.0f));
        pivot->setScript(ticker);
        Node3D* child = pivot->createChild<Node3D>();
        child->setName("Child");
        child->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
        Camera3D* camera = scene->createChild<Camera3D>();
        camera->setName("Cam");
        camera->setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));
        Camera3D* second = scene->createChild<Camera3D>();
        second->setName("Cam2");
        second->setPosition(Math::Vec3(0.0f, 4.0f, 5.0f));
        second->setFov(40.0f);
        OmniLight3D* lamp = scene->createChild<OmniLight3D>();
        lamp->setName("Lamp");
        lamp->setPosition(Math::Vec3(2.0f, 3.0f, 0.0f));

        {
            WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
            CHECK(host.init());
            EditorShell shell(host);
            CHECK(shell.build());
            {
                EditorSelection selection(tree);
                SceneTreePanel hierarchy(tree, selection);
                CHECK(hierarchy.attach(shell.panel(EditorShell::kHierarchyPanel)));
                hierarchy.setEditedScene(scene);

                EditorSettings settings;
                Project project;
                EditorDocument document(tree, selection, hierarchy, project, settings);
                document.setSettingsPath(testRoot() + "/settings.json");
                hierarchy.setChangeListener(&document);

                ProjectUi ui(document);
                CHECK(ui.attach(shell.toolBar()));
                HierarchyActions actions(tree, document, selection, hierarchy);
                actions.attach();

                EditorViewportPanel viewport(*Main::rendering());
                CHECK(viewport.attach(shell.panel(EditorShell::kScenePanel)));
                viewport.setScenario(window->getScenario());
                viewport.connectPreDraw();
                actions.setViewport(&viewport);

                GamePanel game(*Main::rendering(), document);
                CHECK(game.attach(shell.panel(EditorShell::kGamePanel)));
                CHECK(!game.attach(shell.panel(EditorShell::kGamePanel)));
                game.setScenario(window->getScenario());
                game.connectPreDraw();

                EditorPlay play(tree, document);
                CHECK(play.attach(shell.runBar()));
                CHECK(!play.attach(shell.runBar()));
                ViewLog views;
                play.setViewHandler(
                        [&views, &shell](bool showGame)
                        {
                            ++views.shown;
                            views.lastGame = showGame;
                            shell.showPanel(
                                    showGame ? EditorShell::kGamePanel : EditorShell::kScenePanel);
                        });
                EditorMenus menus(shell, ui, viewport, actions, &play);
                CHECK(menus.attach());
                CHECK(strcmp(EditorMenus::menuTitle(EditorMenus::kRun), "Run") == 0);

                Rig rig = { tree, host, shell, selection, hierarchy, document, ui, actions,
                    viewport, game, play, menus };
                frames(rig, 3);
                testPlayStop(rig, views);
                testEditingDisabled(rig);
                testShortcuts(rig, views);
                testInputGate(rig);
                testGamePanel(rig);
                testRunner(rig);
            }
            host.shutdown();
        }
    }

    CHECK(Engine::singleton() != nullptr && Engine::singleton()->isEditorHint());
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
