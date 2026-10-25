#include "prisma3d/editor/EditorPlay.h"

#include "prisma3d/Device.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/script/ZenScripting.h"

#include "platform.h"

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kTooltips[EditorPlay::kButtonCount] = { "Play (F5)", "Pause (F7)", "Stop (F8)",
    "Run in standalone window (F6)" };

const rt::IconId kIcons[EditorPlay::kButtonCount] = { rt::IconId::Play, rt::IconId::Pause,
    rt::IconId::Stop, rt::IconId::StepForward };

bool editorHint()
{
    const Engine* engine = Engine::singleton();
    return engine && engine->isEditorHint();
}

void setEditorHint(bool edit)
{
    if (Engine* engine = Engine::singleton()) engine->setEditorHint(edit);
}

void setPhysicsActive(bool active)
{
    if (PhysicsServer3D* physics = PhysicsServer3D::singleton()) physics->setActive(active);
}

} // namespace

EditorPlay::EditorPlay(SceneTree& tree, EditorDocument& document)
        : mTree(tree),
          mDocument(document),
          mLink(ct::make_rc<Link>(Link{ this })),
          mBar(nullptr),
          mButtons{},
          mRunner(0),
          mLaunches(0),
          mExitCode(-1),
          mState(kStopped),
          mSyncing(false),
          mPreviousHint(false),
          mPreviousPhysics(true)
{
    for (int& id: mButtons) id = -1;
    mPreviousHint = editorHint();
    if (PhysicsServer3D* physics = PhysicsServer3D::singleton())
        mPreviousPhysics = physics->isActive();
    setEditorHint(true);
    setPhysicsActive(false);
}

EditorPlay::~EditorPlay()
{
    mLink->owner = nullptr;
    if (mState != kStopped) stop();
    if (mRunner > 0) process_kill(mRunner);
    if (Engine* engine = Engine::singleton()) engine->setEditorHint(mPreviousHint);
    if (PhysicsServer3D* physics = PhysicsServer3D::singleton())
        physics->setActive(mPreviousPhysics);
}

const char* EditorPlay::buttonTooltip(Button button)
{
    return button >= 0 && button < kButtonCount ? kTooltips[button] : "";
}

rt::IconId EditorPlay::buttonIcon(Button button)
{
    return button >= 0 && button < kButtonCount ? kIcons[button] : rt::IconId::None;
}

bool EditorPlay::attach(rt::ToolBar* bar)
{
    if (mBar || !bar) return false;
    mBar = bar;
    mButtons[kPlay] = mBar->addToggle(kIcons[kPlay], kTooltips[kPlay], false);
    mButtons[kPause] = mBar->addToggle(kIcons[kPause], kTooltips[kPause], false);
    mButtons[kStop] = mBar->addButton(kIcons[kStop], kTooltips[kStop]);
    mBar->addSeparator();
    mButtons[kRun] = mBar->addButton(kIcons[kRun], kTooltips[kRun]);
    for (int id: mButtons) mBar->setItemDraggable(id, false);

    const ct::Rc<Link> link = mLink;
    mBar->buttonClicked.connect(
            [link](int id)
            {
                if (link->owner) link->owner->clickedButton(id);
            });
    mBar->toggleChanged.connect(
            [link](int id, bool checked)
            {
                if (link->owner) link->owner->toggledButton(id, checked);
            });
    syncButtons();
    return true;
}

bool EditorPlay::attached() const
{
    return mBar != nullptr;
}

int EditorPlay::buttonId(Button button) const
{
    return button >= 0 && button < kButtonCount ? mButtons[button] : -1;
}

void EditorPlay::setViewHandler(const ViewHandler& handler)
{
    mViewHandler = handler;
}

EditorPlay::State EditorPlay::state() const
{
    return mState;
}

bool EditorPlay::playing() const
{
    return mState != kStopped;
}

bool EditorPlay::paused() const
{
    return mState == kPaused;
}

bool EditorPlay::canPlay() const
{
    return mDocument.scene() != nullptr;
}

bool EditorPlay::canPause() const
{
    return mState != kStopped;
}

bool EditorPlay::canStop() const
{
    return mState != kStopped || runnerRunning();
}

bool EditorPlay::canRun() const
{
    return mState == kStopped && !runnerRunning() && !runTarget().empty();
}

bool EditorPlay::play()
{
    if (!mDocument.scene())
    {
        log_error("editor: there is no scene to play");
        return false;
    }

    if (ZenScripting* scripting = ZenScripting::instance())
        if (const int reloaded = scripting->reloadChanged())
            log_info("editor: recompiled %d changed script(s)", reloaded);

    const bool restart = mState != kStopped;
    mTree.setPaused(false);
    setEditorHint(false);
    const bool started = mDocument.beginPlay();
    if (started)
    {
        if (!restart) setEditorHint(true);
        mTree.flushDeleteQueue();
    }
    setEditorHint(started || restart ? false : true);
    if (!started)
    {
        mTree.setPaused(mState == kPaused);
        log_error("editor: could not start the game: %s", mDocument.error().c_str());
        syncButtons();
        return false;
    }
    setPhysicsActive(true);
    mState = kPlaying;
    if (mViewHandler) mViewHandler(true);
    log_info("editor: playing '%s'", mDocument.sceneName().c_str());
    syncButtons();
    return true;
}

void EditorPlay::setPaused(bool paused)
{
    if (mState == kStopped) return;
    mTree.setPaused(paused);
    mState = paused ? kPaused : kPlaying;
    syncButtons();
}

void EditorPlay::pause()
{
    setPaused(mState == kPlaying);
}

bool EditorPlay::stop()
{
    if (mRunner > 0 && runnerRunning()) stopRunner();
    if (mState == kStopped)
    {
        syncButtons();
        return true;
    }
    mTree.setPaused(false);
    setEditorHint(true);
    const bool restored = mDocument.endPlay();
    if (!restored)
    {
        setEditorHint(false);
        log_error("editor: could not restore the scene, staying in play: %s",
                mDocument.error().c_str());
        mTree.setPaused(mState == kPaused);
        syncButtons();
        return false;
    }
    setEditorHint(false);
    mTree.flushDeleteQueue();
    setEditorHint(true);
    setPhysicsActive(false);
    if (Device* device = Device::singleton()) device->mouseSetMode(MouseMode::Visible);
    mState = kStopped;
    if (mViewHandler) mViewHandler(false);
    log_info("editor: stopped");
    syncButtons();
    return true;
}

void EditorPlay::setRunnerPath(const ct::String& path)
{
    mRunnerPath = path;
}

ct::String EditorPlay::runnerPath() const
{
    if (!mRunnerPath.empty()) return file_exists(mRunnerPath.c_str()) ? mRunnerPath : ct::String();
    ct::String path = dir_app();
    path += "/";
    path += kRunnerName;
#if defined(_WIN32)
    path += ".exe";
#endif
    return file_exists(path.c_str()) ? path : ct::String();
}

ct::String EditorPlay::runTarget() const
{
    const Project& project = mDocument.project();
    if (project.isOpen()) return project.manifestPath();
    if (mDocument.hasScenePath()) return mDocument.sceneAbsolutePath(mDocument.scenePath());
    return ct::String();
}

bool EditorPlay::run()
{
    if (mState != kStopped)
    {
        log_warn("editor: stop the game before running the standalone window");
        return false;
    }
    if (mRunner > 0)
    {
        if (runnerRunning()) return false;
        finishRunner();
    }

    const ct::String runner = runnerPath();
    if (runner.empty())
    {
        log_error("editor: %s not found next to the editor, build the runner target", kRunnerName);
        return false;
    }
    if (mDocument.project().isOpen())
    {
        if (!mDocument.saveProject())
        {
            log_error("editor: run: %s", mDocument.error().c_str());
            return false;
        }
    }
    if (mDocument.hasScenePath())
    {
        if (!mDocument.saveScene())
        {
            log_error("editor: run: %s", mDocument.error().c_str());
            return false;
        }
    }
    const ct::String target = runTarget();
    if (target.empty())
    {
        log_warn("editor: save the scene or open a project before running");
        return false;
    }

    // Run from the editor stays in a window even when the game is set to full screen.
    const char* const arguments[] = { target.c_str(), "--windowed" };
    const std::int64_t id = process_spawn(runner.c_str(), arguments, 2, false);
    if (id <= 0)
    {
        log_error("editor: could not start the runner: %s", platform_get_error());
        return false;
    }
    mRunner = id;
    mExitCode = -1;
    ++mLaunches;
    log_info("editor: runner started on '%s'", target.c_str());
    syncButtons();
    return true;
}

void EditorPlay::stopRunner()
{
    if (mRunner <= 0) return;
    if (process_is_running(mRunner))
    {
        process_kill(mRunner);
        mExitCode = process_exit_code(mRunner);
        mRunner = 0;
        log_info("editor: runner stopped");
        syncButtons();
        return;
    }
    finishRunner();
}

bool EditorPlay::runnerRunning() const
{
    return mRunner > 0 && process_is_running(mRunner);
}

int EditorPlay::runnerExitCode() const
{
    return mExitCode;
}

unsigned int EditorPlay::runnerLaunches() const
{
    return mLaunches;
}

void EditorPlay::finishRunner()
{
    if (mRunner <= 0) return;
    mExitCode = process_exit_code(mRunner);
    mRunner = 0;
    if (mExitCode == 0) log_info("editor: runner finished");
    else
        log_warn("editor: runner exited with status %d", mExitCode);
    syncButtons();
}

void EditorPlay::update()
{
    if (mRunner > 0 && !process_is_running(mRunner)) finishRunner();
    syncButtons();
}

void EditorPlay::syncButtons()
{
    if (!mBar || mButtons[kPlay] < 0) return;
    mSyncing = true;
    const auto enable = [this](Button button, bool enabled)
    {
        if (mBar->isItemEnabled(mButtons[button]) != enabled)
            mBar->setItemEnabled(mButtons[button], enabled);
    };
    const auto check = [this](Button button, bool checked)
    {
        if (mBar->isItemChecked(mButtons[button]) != checked)
            mBar->setItemChecked(mButtons[button], checked);
    };
    enable(kPlay, canPlay());
    enable(kPause, canPause());
    enable(kStop, canStop());
    enable(kRun, canRun());
    check(kPlay, mState != kStopped);
    check(kPause, mState == kPaused);
    mSyncing = false;
}

void EditorPlay::clickedButton(int id)
{
    if (id == mButtons[kStop]) stop();
    else if (id == mButtons[kRun])
        run();
}

void EditorPlay::toggledButton(int id, bool checked)
{
    if (mSyncing) return;
    if (id == mButtons[kPlay])
    {
        if (checked || mState != kStopped) play();
    }
    else if (id == mButtons[kPause])
        setPaused(checked);
    syncButtons();
}

bool EditorPlay::handleEvent(const InputEvent& event)
{
    if (event.type != InputEventType::Key || !event.key.pressed || event.key.echo) return false;
    if (event.key.modifiers != 0) return false;
    switch (event.key.key)
    {
        case Key::F5:
            play();
            return true;
        case Key::F6:
            run();
            return true;
        case Key::F7:
            pause();
            return true;
        case Key::F8:
            stop();
            return true;
        default:
            return false;
    }
}

} // namespace prisma3d
