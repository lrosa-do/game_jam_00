#include "prisma3d/editor/EditorMenus.h"

#include "prisma3d/editor/EditorAbout.h"
#include "prisma3d/editor/EditorPlay.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/LightmapPanel.h"
#include "prisma3d/editor/ProjectSettingsEditor.h"
#include "prisma3d/main/Project.h"

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kMenuTitles[EditorMenus::kMenuCount] = { "File", "Edit", "Create", "View",
    "Scene", "Run", "Help" };

std::uint64_t mix(std::uint64_t hash, std::uint64_t value)
{
    return (hash ^ value) * 1099511628211ull;
}

std::uint64_t hashString(std::uint64_t hash, const ct::String& text)
{
    for (std::size_t i = 0; i < text.size(); ++i)
        hash = mix(hash, static_cast<unsigned char>(text.data()[i]));
    return mix(hash, text.size());
}

} // namespace

EditorMenus::EditorMenus(EditorShell& shell, ProjectUi& projectUi, EditorViewportPanel& viewport,
        HierarchyActions& actions, EditorPlay* play)
        : mShell(shell),
          mProjectUi(projectUi),
          mViewport(viewport),
          mActions(actions),
          mPlay(play),
          mSettings(nullptr),
          mLightmap(nullptr),
          mLink(ct::make_rc<Link>(Link{ this })),
          mMenus{},
          mHash(0),
          mBuilt(false)
{
}

EditorMenus::~EditorMenus()
{
    mLink->owner = nullptr;
}

void EditorMenus::setProjectSettings(ProjectSettingsEditor* settings)
{
    mSettings = settings;
}

void EditorMenus::setLightmapPanel(LightmapPanel* panel)
{
    mLightmap = panel;
}

const char* EditorMenus::menuTitle(MenuIndex index)
{
    return index >= 0 && index < kMenuCount ? kMenuTitles[index] : "";
}

bool EditorMenus::attach()
{
    if (mBuilt || !mShell.menuBar() || !mProjectUi.attached()) return false;
    build();
    mBuilt = true;
    update();
    return true;
}

bool EditorMenus::attached() const
{
    return mBuilt;
}

void EditorMenus::build()
{
    rt::MenuBar* bar = mShell.menuBar();
    const ct::Rc<Link> link = mLink;
    for (int i = 0; i < kMenuCount; ++i) mMenus[i] = bar->addMenu(kMenuTitles[i]);

    rt::Menu* file = mMenus[kFile];
    file->addAction("New Project...",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.newProject();
            });
    file->addAction("Open Project...",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.openProject();
            });
    rt::MenuAction* recent = file->addAction("Recent Projects");
    recent->setSubmenu(mProjectUi.recentMenu());
    file->addSeparator();
    rt::MenuAction* save = file->addAction("Save Scene",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.save();
            });
    save->setShortcut("Ctrl+S");
    rt::MenuAction* saveAs = file->addAction("Save Scene As...",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.saveAs();
            });
    saveAs->setShortcut("Ctrl+Shift+S");
    file->addSeparator();
    file->addAction("Project Settings...",
            [link]
            {
                if (link->owner && link->owner->mSettings) link->owner->mSettings->open();
            });
    file->addAction("Export Game...",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.exportGame();
            });
    file->addAction("Close Project",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.closeProject();
            });
    file->addSeparator();
    file->addAction("Quit",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.requestQuit();
            });

    rt::Menu* edit = mMenus[kEdit];
    rt::MenuAction* undo = edit->addAction("Undo",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.undo();
            });
    undo->setShortcut("Ctrl+Z");
    rt::MenuAction* redo = edit->addAction("Redo",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.redo();
            });
    redo->setShortcut("Ctrl+Y");
    edit->addSeparator();
    mActions.populate(edit);
    mActions.populateCreate(mMenus[kCreate]);

    rt::Menu* view = mMenus[kView];
    rt::MenuAction* perspective = view->addCheckable("Perspective", true);
    perspective->triggered.connect(
            [link]
            {
                if (link->owner) link->owner->mViewport.setOrthogonal(false);
            });
    rt::MenuAction* orthogonal = view->addCheckable("Orthogonal", false);
    orthogonal->triggered.connect(
            [link]
            {
                if (link->owner) link->owner->mViewport.setOrthogonal(true);
            });
    view->addSeparator();
    rt::MenuAction* grid = view->addCheckable("Grid", true);
    grid->triggered.connect(
            [link]
            {
                if (!link->owner) return;
                EditorViewportPanel& panel = link->owner->mViewport;
                panel.setGridVisible(!panel.grid().gridEnabled());
            });
    view->addSeparator();
    view->addAction("Reset Layout",
            [link]
            {
                if (link->owner) link->owner->mShell.resetLayout();
            });

    rt::Menu* scene = mMenus[kScene];
    rt::MenuAction* newScene = scene->addAction("New Scene",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.newScene();
            });
    newScene->setShortcut("Ctrl+N");
    rt::MenuAction* open = scene->addAction("Open Scene");
    open->setSubmenu(mProjectUi.sceneMenu());
    open->setShortcut("Ctrl+O");
    scene->addSeparator();
    scene->addAction("Set as Main Scene",
            [link]
            {
                if (link->owner) link->owner->mProjectUi.markMainScene();
            });
    scene->addSeparator();
    scene->addAction("Environment...",
            [link]
            {
                if (link->owner) link->owner->mActions.requestEnvironment();
            });
    scene->addAction("Bake Lightmaps...",
            [link]
            {
                if (!link->owner || !link->owner->mLightmap) return;
                link->owner->mLightmap->createNode();
                link->owner->mShell.showPanel(EditorShell::kLightmapPanel);
            });

    rt::Menu* run = mMenus[kRun];
    rt::MenuAction* play = run->addAction("Play",
            [link]
            {
                if (link->owner && link->owner->mPlay) link->owner->mPlay->play();
            });
    play->setShortcut("F5");
    rt::MenuAction* pause = run->addCheckable("Pause", false);
    pause->setShortcut("F7");
    pause->triggered.connect(
            [link]
            {
                if (link->owner && link->owner->mPlay) link->owner->mPlay->pause();
            });
    rt::MenuAction* stop = run->addAction("Stop",
            [link]
            {
                if (link->owner && link->owner->mPlay) link->owner->mPlay->stop();
            });
    stop->setShortcut("F8");
    run->addSeparator();
    rt::MenuAction* runner = run->addAction("Run in Runner",
            [link]
            {
                if (link->owner && link->owner->mPlay) link->owner->mPlay->run();
            });
    runner->setShortcut("F6");
    run->addAction("Stop Runner",
            [link]
            {
                if (link->owner && link->owner->mPlay) link->owner->mPlay->stopRunner();
            });
    mMenus[kHelp]->addAction("About prisma3d editor...", [] { EditorAbout::open(); });
}

std::uint64_t EditorMenus::stateHash() const
{
    const EditorDocument& document = mProjectUi.document();
    std::uint64_t hash = 1469598103934665603ull;
    const ct::Vector<ct::String>& recent = document.settings().recentProjectPaths;
    hash = mix(hash, recent.size());
    for (std::size_t i = 0; i < recent.size(); ++i) hash = hashString(hash, recent[i]);
    hash = mix(hash, document.project().isOpen() ? 1 : 0);
    hash = hashString(hash, document.scenePath());
    if (document.project().isOpen())
    {
        const ct::Vector<ct::String>& scenes = document.project().scenes();
        hash = mix(hash, scenes.size());
        for (std::size_t i = 0; i < scenes.size(); ++i) hash = hashString(hash, scenes[i]);
        hash = hashString(hash, document.project().mainScene());
    }
    return hash;
}

void EditorMenus::update()
{
    if (!mBuilt) return;
    const std::uint64_t hash = stateHash();
    if (hash != mHash)
    {
        mHash = hash;
        mProjectUi.rebuildRecentMenu();
        mProjectUi.rebuildSceneMenu();
    }

    const EditorDocument& document = mProjectUi.document();
    const bool project = document.project().isOpen();
    const bool editing = !document.playing();
    rt::Menu* file = mMenus[kFile];
    file->action(kFileNewProject)->setEnabled(editing);
    file->action(kFileOpenProject)->setEnabled(editing);
    file->action(kFileSave)->setEnabled(editing);
    file->action(kFileSettings)->setEnabled(project && mSettings);
    file->action(kFileExport)->setEnabled(project && editing);
    file->action(kFileClose)->setEnabled(project && editing);
    file->action(kFileSaveAs)->setEnabled(project && editing);
    file->action(kFileRecent)
            ->setEnabled(editing && document.settings().recentProjectPaths.size() > 0);
    mMenus[kScene]->action(kSceneNew)->setEnabled(editing);
    mMenus[kScene]->action(kSceneMain)->setEnabled(document.sceneInProject() && editing);
    mMenus[kScene]->action(kSceneOpen)->setEnabled(editing);
    mMenus[kScene]->action(kSceneEnvironment)->setEnabled(editing && document.scene() != nullptr);
    mMenus[kScene]
            ->action(kSceneBakeLightmaps)
            ->setEnabled(editing && document.scene() != nullptr && mLightmap != nullptr &&
                         !mLightmap->baker().busy());

    rt::Menu* run = mMenus[kRun];
    run->action(kRunPlay)->setEnabled(mPlay && mPlay->canPlay());
    run->action(kRunPause)->setEnabled(mPlay && mPlay->canPause());
    run->action(kRunPause)->setChecked(mPlay && mPlay->paused());
    run->action(kRunStop)->setEnabled(mPlay && mPlay->state() != EditorPlay::kStopped);
    run->action(kRunInRunner)->setEnabled(mPlay && mPlay->canRun());
    run->action(kRunStopRunner)->setEnabled(mPlay && mPlay->runnerRunning());

    const EditorUndoRedoManager& history = document.undoRedo();
    rt::MenuAction* undo = mMenus[kEdit]->action(kEditUndo);
    rt::MenuAction* redo = mMenus[kEdit]->action(kEditRedo);
    const bool canUndo = editing && history.hasUndo();
    const bool canRedo = editing && history.hasRedo();
    const ct::String undoText =
            canUndo ? ct::String("Undo ") + history.currentActionName() : ct::String("Undo");
    const ct::String redoText =
            canRedo ? ct::String("Redo ") + history.redoActionName() : ct::String("Redo");
    if (undo->text() != undoText) undo->setText(undoText);
    if (redo->text() != redoText) redo->setText(redoText);
    undo->setEnabled(canUndo);
    redo->setEnabled(canRedo);

    mActions.refresh(mMenus[kEdit]);
    mActions.refreshCreate(mMenus[kCreate]);

    rt::Menu* view = mMenus[kView];
    const bool orthogonal = mViewport.camera().isOrthogonal();
    view->action(kViewPerspective)->setChecked(!orthogonal);
    view->action(kViewOrthogonal)->setChecked(orthogonal);
    view->action(kViewGrid)->setChecked(mViewport.grid().gridEnabled());
}

rt::Menu* EditorMenus::menu(MenuIndex index) const
{
    return index >= 0 && index < kMenuCount ? mMenus[index] : nullptr;
}

rt::MenuAction* EditorMenus::action(MenuIndex index, int item) const
{
    rt::Menu* m = menu(index);
    return m ? m->action(item) : nullptr;
}

} // namespace prisma3d
