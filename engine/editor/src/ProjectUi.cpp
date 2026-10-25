#include "prisma3d/editor/ProjectUi.h"

#include "prisma3d/editor/EditorAbout.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/FileDialogMemory.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Project.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include "platform.h"

#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kButtonTooltips[ProjectUi::kButtonCount] = { "New Project...", "Open Project...",
    "Recent Projects", "Save Scene and Project (Ctrl+S)", "New Scene (Ctrl+N)",
    "Open Scene (Ctrl+O)", "Save Scene As... (Ctrl+Shift+S)", "Use This Scene as the Main Scene",
    "Undo (Ctrl+Z)", "Redo (Ctrl+Y)" };

const char* const kButtonIcons[ProjectUi::kButtonCount] = { "Add", "Folder", "FileList", "Save",
    "New", "Load", "Duplicate", "Favorites", "UndoRedo", "Redo" };

bool isFolder(const ct::String& path)
{
    PathInfo info;
    return !path.empty() && fs_get_path_info(path.c_str(), &info) &&
           info.type == PATH_TYPE_DIRECTORY;
}

ct::String slashed(const ct::String& path)
{
    ct::String result = path;
    for (std::size_t i = 0; i < result.size(); ++i)
        if (result.data()[i] == '\\') result.data()[i] = '/';
    return result;
}

ct::String parentOf(const ct::String& path)
{
    const ct::String text = slashed(path);
    const std::size_t slash = text.rfind('/');
    if (slash == ct::String::npos) return ct::String();
    if (slash == 0) return ct::String("/");
    return text.substr(0, slash);
}

ct::String nameOf(const ct::String& path)
{
    const ct::String text = slashed(path);
    const std::size_t slash = text.rfind('/');
    return slash == ct::String::npos ? text : text.substr(slash + 1);
}

} // namespace

NewProjectDialog::NewProjectDialog(const ct::String& parentFolder, const ct::String& name)
        : rt::FloatWindow("New project"),
          mParent(nullptr),
          mName(nullptr),
          mError(nullptr),
          mBrowse(nullptr),
          mCreate(nullptr),
          mCancel(nullptr)
{
    setResizable(false);
    setMinimizable(false);

    rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    main->setSpacing(8.0f);
    main->setPadding(16.0f);

    main->createChild<rt::Label>("Parent folder");
    rt::BoxLayout* row = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    row->setPadding(0.0f);
    row->setSpacing(6.0f);
    mParent = row->createChild<rt::TextInput>(parentFolder);
    mParent->setStretch(1.0f);
    mBrowse = row->createChild<rt::Button>();
    mBrowse->setIconId(EditorIcons::get("Load"));
    mBrowse->setSize(40.0f, 0.0f);
    mBrowse->setTooltip("Choose the folder");
    mBrowse->clicked.connect([this] { browseRequested.emit(); });

    main->createChild<rt::Label>("Project name");
    mName = main->createChild<rt::TextInput>(name);

    mError = main->createChild<rt::Label>("");

    rt::BoxLayout* buttons = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    buttons->setPadding(0.0f);
    buttons->setSpacing(8.0f);
    buttons->setMainAlign(rt::MainAlign::End);
    mCancel = buttons->createChild<rt::Button>("Cancel");
    mCancel->setSize(90.0f, 0.0f);
    mCancel->clicked.connect([this] { cancel(); });
    mCreate = buttons->createChild<rt::Button>("Create");
    mCreate->setSize(90.0f, 0.0f);
    mCreate->clicked.connect([this] { create(); });

    mParent->submitted.connect([this](const ct::String&) { create(); });
    mName->submitted.connect([this](const ct::String&) { create(); });

    const float width = 480.0f;
    const float height = 270.0f;
    setFloatSize(width, height);
    const rt::IO& io = rt::GetIO();
    setFloatPos((io.displayWidth - width) * 0.5f, (io.displayHeight - height) * 0.5f);
}

void NewProjectDialog::setCreateHandler(const CreateHandler& handler)
{
    mHandler = handler;
}

void NewProjectDialog::setParentFolder(const ct::String& folder)
{
    mParent->setText(folder);
}

ct::String NewProjectDialog::parentFolder() const
{
    return mParent->text();
}

ct::String NewProjectDialog::projectName() const
{
    return mName->text();
}

const ct::String& NewProjectDialog::error() const
{
    return mMessage;
}

rt::TextInput* NewProjectDialog::parentInput() const
{
    return mParent;
}

rt::TextInput* NewProjectDialog::nameInput() const
{
    return mName;
}

rt::Button* NewProjectDialog::browseButton() const
{
    return mBrowse;
}

rt::Button* NewProjectDialog::createButton() const
{
    return mCreate;
}

rt::Button* NewProjectDialog::cancelButton() const
{
    return mCancel;
}

void NewProjectDialog::create()
{
    const ct::String parent = mParent->text().trimmed();
    const ct::String name = mName->text().trimmed();
    mMessage = ct::String();
    if (parent.empty()) mMessage = "Choose the parent folder";
    else if (name.empty())
        mMessage = "Enter a project name";
    else if (mHandler)
        mMessage = mHandler(parent, name);
    mError->setText(mMessage);
    if (!mMessage.empty()) return;
    rt::WidgetApp::instance().removeFloat(this);
}

void NewProjectDialog::cancel()
{
    rt::WidgetApp::instance().removeFloat(this);
}

void NewProjectDialog::onKeyPress(rt::KeyEvent& event)
{
    if (event.key == rt::Key::Escape)
    {
        event.consumed = true;
        cancel();
        return;
    }
    if (event.key == rt::Key::Return || event.key == rt::Key::KPEnter)
    {
        event.consumed = true;
        create();
        return;
    }
    rt::FloatWindow::onKeyPress(event);
}

ProjectUi::ProjectUi(EditorDocument& document)
        : mDocument(document),
          mLink(ct::make_rc<Link>(Link{ this })),
          mToolBar(nullptr),
          mButtons{},
          mRecentMenu(nullptr),
          mSceneMenu(nullptr),
          mTitleSerial(0),
          mExport(0)
{
    for (int& id: mButtons) id = -1;
}

ProjectUi::~ProjectUi()
{
    mLink->owner = nullptr;
    delete mRecentMenu;
    delete mSceneMenu;
}

rt::IconId ProjectUi::buttonIcon(Button button)
{
    return button >= 0 && button < kButtonCount ? EditorIcons::get(kButtonIcons[button])
                                                : rt::IconId::None;
}

const char* ProjectUi::buttonTooltip(Button button)
{
    return button >= 0 && button < kButtonCount ? kButtonTooltips[button] : "";
}

bool ProjectUi::attach(rt::ToolBar* toolBar)
{
    if (mToolBar || !toolBar) return false;
    mToolBar = toolBar;

    mToolBar->addSeparator();
    for (int i = 0; i < kButtonCount; ++i)
    {
        if (i == kSave || i == kNewScene || i == kUndo) mToolBar->addSeparator();
        mButtons[i] = mToolBar->addButton(buttonIcon(static_cast<Button>(i)), kButtonTooltips[i]);
    }
    mToolBar->addSeparator();

    const ct::Rc<Link> link = mLink;
    mToolBar->buttonClicked.connect(
            [link](int id)
            {
                if (link->owner) link->owner->clickedButton(id);
            });

    mRecentMenu = new rt::Menu();
    mSceneMenu = new rt::Menu();
    update();
    return true;
}

bool ProjectUi::attached() const
{
    return mToolBar != nullptr;
}

int ProjectUi::buttonId(Button button) const
{
    return button >= 0 && button < kButtonCount ? mButtons[button] : -1;
}

EditorDocument& ProjectUi::document()
{
    return mDocument;
}

void ProjectUi::setTitleHandler(const ct::Function<void(const ct::String&)>& handler)
{
    mTitleHandler = handler;
}

void ProjectUi::setStopHandler(const ct::Function<void()>& handler)
{
    mStopHandler = handler;
}

void ProjectUi::setQuitHandler(const ct::Function<void()>& handler)
{
    mQuitHandler = handler;
}

ct::String ProjectUi::windowTitle() const
{
    ct::String document;
    if (mDocument.project().isOpen())
    {
        document += mDocument.project().name();
        document += " - ";
    }
    document += mDocument.sceneName();
    if (mDocument.dirty()) document += '*';
    return EditorAbout::windowTitle(document);
}

unsigned int ProjectUi::titleSerial() const
{
    return mTitleSerial;
}

void ProjectUi::update()
{
    mDocument.update();
    if (mExport > 0 && !process_is_running(mExport))
    {
        const int status = process_exit_code(mExport);
        mExport = 0;
        if (char* text = file_read_text(mExportLog.c_str()))
        {
            const char* line = text;
            while (*line)
            {
                const char* end = strchr(line, '\n');
                const ct::String item(line, end ? static_cast<size_t>(end - line) : strlen(line));
                if (!item.empty())
                {
                    if (item.starts_with("export:")) log_error("%s", item.c_str());
                    else
                        log_info("export: %s", item.c_str());
                }
                if (!end) break;
                line = end + 1;
            }
            fs_free(text);
            fs_remove_path(mExportLog.c_str());
        }
        if (status == 0) log_info("editor: export finished: %s", mExportOutput.c_str());
        else
            log_error("editor: export failed with status %d", status);
    }

    const bool project = mDocument.project().isOpen();
    if (mToolBar)
    {
        const auto enable = [this](Button button, bool enabled)
        {
            if (mToolBar->isItemEnabled(mButtons[button]) != enabled)
                mToolBar->setItemEnabled(mButtons[button], enabled);
        };
        const bool editing = !mDocument.playing();
        enable(kNewProject, editing);
        enable(kOpenProject, editing);
        enable(kRecentProjects,
                editing && (mDocument.settings().recentProjectPaths.size() > 0 || project));
        enable(kSave, editing && project);
        enable(kNewScene, editing);
        enable(kOpenScene, editing);
        enable(kSaveAs, editing && project);
        enable(kMainScene, editing && mDocument.sceneInProject() && !mDocument.isMainScene());

        const EditorUndoRedoManager& history = mDocument.undoRedo();
        enable(kUndo, editing && history.hasUndo());
        enable(kRedo, editing && history.hasRedo());
        ct::String undoTip = kButtonTooltips[kUndo];
        ct::String redoTip = kButtonTooltips[kRedo];
        if (editing && history.hasUndo())
            undoTip = ct::String("Undo ") + history.currentActionName() + " (Ctrl+Z)";
        if (editing && history.hasRedo())
            redoTip = ct::String("Redo ") + history.redoActionName() + " (Ctrl+Y)";
        if (undoTip != mUndoTip)
        {
            mUndoTip = undoTip;
            mToolBar->setItemTooltip(mButtons[kUndo], undoTip);
        }
        if (redoTip != mRedoTip)
        {
            mRedoTip = redoTip;
            mToolBar->setItemTooltip(mButtons[kRedo], redoTip);
        }
    }

    const ct::String title = windowTitle();
    if (title != mTitle)
    {
        mTitle = title;
        ++mTitleSerial;
        if (mTitleHandler) mTitleHandler(mTitle);
    }
}

bool ProjectUi::handleEvent(const InputEvent& event)
{
    if (event.type != InputEventType::Key || !event.key.pressed || event.key.echo) return false;
    if (!(event.key.modifiers & kModControl)) return false;
    if (WidgetHost::modalOpen()) return false;
    const bool shift = (event.key.modifiers & kModShift) != 0;
    switch (event.key.key)
    {
        case Key::S:
            if (shift) saveAs();
            else
                save();
            return true;
        case Key::N:
            newScene();
            return true;
        case Key::O:
            openScene();
            return true;
        case Key::Z:
            if (WidgetHost::textFocused()) return false;
            if (shift) redo();
            else
                undo();
            return true;
        case Key::Y:
            if (WidgetHost::textFocused() || shift) return false;
            redo();
            return true;
        default:
            return false;
    }
}

bool ProjectUi::undo()
{
    return mDocument.undo();
}

bool ProjectUi::redo()
{
    return mDocument.redo();
}

void ProjectUi::clickedButton(int id)
{
    for (int i = 0; i < kButtonCount; ++i)
    {
        if (mButtons[i] != id) continue;
        switch (static_cast<Button>(i))
        {
            case kNewProject:
                newProject();
                break;
            case kOpenProject:
                openProject();
                break;
            case kRecentProjects:
                rebuildRecentMenu();
                showMenu(mRecentMenu);
                break;
            case kSave:
                save();
                break;
            case kNewScene:
                newScene();
                break;
            case kOpenScene:
                openScene();
                break;
            case kSaveAs:
                saveAs();
                break;
            case kMainScene:
                markMainScene();
                break;
            case kUndo:
                undo();
                break;
            case kRedo:
                redo();
                break;
            case kButtonCount:
                break;
        }
        return;
    }
}

void ProjectUi::showMenu(rt::Menu* menu)
{
    if (!menu || menu->actionCount() == 0) return;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    menu->exec(app.mouseX(), app.mouseY() + 6.0f);
}

bool ProjectUi::refuseWhilePlaying()
{
    if (!mDocument.playing()) return false;
    log_warn("editor: stop the game first");
    rt::Toast::show("Stop the game first", rt::Toast::Type::Warning);
    return true;
}

void ProjectUi::reportError(const ct::String& message)
{
    log_error("editor: %s", message.c_str());
    rt::Toast::show(message, rt::Toast::Type::Error, 6.0f);
}

template<typename T>
T* ProjectUi::floatOfType() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (T* found = rt::widget_cast<T>(floats[i - 1])) return found;
    return nullptr;
}

rt::Dialog* ProjectUi::confirmDialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return app ? rt::widget_cast<rt::Dialog>(app->popup()) : nullptr;
}

bool ProjectUi::confirming() const
{
    return confirmDialog() != nullptr;
}

rt::Menu* ProjectUi::recentMenu() const
{
    return mRecentMenu;
}

rt::Menu* ProjectUi::sceneMenu() const
{
    return mSceneMenu;
}

NewProjectDialog* ProjectUi::newProjectDialog() const
{
    return floatOfType<NewProjectDialog>();
}

rt::FileDialog* ProjectUi::fileDialog() const
{
    return floatOfType<rt::FileDialog>();
}

rt::InputBox* ProjectUi::saveAsBox() const
{
    return floatOfType<rt::InputBox>();
}

void ProjectUi::guard(const char* action, const ct::Function<void()>& next)
{
    if (!mDocument.dirty())
    {
        next();
        return;
    }
    openConfirm(action, next);
}

void ProjectUi::openConfirm(const char* action, const ct::Function<void()>& next)
{
    if (confirming()) return;
    mPending = next;

    ct::String message = "'";
    message += mDocument.sceneName();
    message += "' has unsaved changes.\nSave them before ";
    message += action;
    message += "?";
    rt::Dialog* dialog = new rt::Dialog("Unsaved changes", message);
    const ct::Rc<Link> link = mLink;
    dialog->addButton("Save", rt::Dialog::Role::Accept,
            [link]
            {
                if (link->owner) link->owner->confirmSave();
            });
    dialog->addButton("Discard", rt::Dialog::Role::Danger,
            [link]
            {
                if (link->owner) link->owner->confirmDiscard();
            });
    dialog->addButton("Cancel", rt::Dialog::Role::Cancel,
            [link]
            {
                if (link->owner) link->owner->confirmCancel();
            });
    dialog->show();
}

void ProjectUi::confirmSave()
{
    const ct::Function<void()> next = mPending;
    mPending = ct::Function<void()>();
    saveThen(next);
}

void ProjectUi::confirmDiscard()
{
    const ct::Function<void()> next = mPending;
    mPending = ct::Function<void()>();
    if (next) next();
}

void ProjectUi::confirmCancel()
{
    mPending = ct::Function<void()>();
}

void ProjectUi::saveThen(const ct::Function<void()>& next)
{
    if (mDocument.hasScenePath())
    {
        if (!mDocument.saveScene())
        {
            reportError(mDocument.error());
            return;
        }
        if (next) next();
        return;
    }
    if (!mDocument.project().isOpen())
    {
        reportError("Open or create a project to save the scene");
        return;
    }
    openSaveAsBox(next);
}

ct::String ProjectUi::defaultSceneName() const
{
    ct::String reason;
    ct::String name = "NewScene";
    for (int index = 2; !mDocument.validSceneName(name, reason) && index < 1000; ++index)
    {
        name = "NewScene";
        name.append_number(index);
    }
    return name;
}

void ProjectUi::openSaveAsBox(const ct::Function<void()>& next)
{
    if (!mDocument.project().isOpen())
    {
        reportError("Open or create a project to save the scene");
        return;
    }
    rt::WidgetApp& app = rt::WidgetApp::instance();
    if (rt::InputBox* existing = saveAsBox())
    {
        app.bringToFront(existing);
        return;
    }
    rt::InputBox* box = app.addFloat<rt::InputBox>("Save scene as",
            "Scene name (saved inside the Scenes folder):");
    box->setText(defaultSceneName());
    box->setAcceptLabel("Save");
    const ct::Rc<Link> link = mLink;
    box->setValidator(
            [link](const ct::String& name)
            {
                ct::String reason;
                if (!link->owner) return ct::String();
                return link->owner->mDocument.validSceneName(name, reason) ? ct::String() : reason;
            });
    box->accepted.connect(
            [link, next](const ct::String& name)
            {
                if (!link->owner) return;
                ProjectUi& ui = *link->owner;
                if (!ui.mDocument.saveSceneAs(name))
                {
                    ui.reportError(ui.mDocument.error());
                    return;
                }
                rt::Toast::show(ct::String("Saved ") + ui.mDocument.scenePath(),
                        rt::Toast::Type::Success);
                if (next) next();
            });
}

void ProjectUi::openFileChooser(const char* title, const char* purpose, const ct::String& filter,
        const ct::Function<void(const ct::String&)>& chosen)
{
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>(title);
    dialog->setMode(rt::FileDialog::Mode::Open);
    if (!filter.empty()) dialog->setFilter(filter);
    EditorSettings* settings = &mDocument.settings();
    FileDialogMemory::prepare(*dialog, settings, purpose, &mDocument.project());
    dialog->accepted.connect(
            [settings, purpose, chosen](const ct::String& path)
            {
                FileDialogMemory::remember(settings, purpose, path);
                chosen(path);
            });
}

ct::String ProjectUi::defaultParentFolder(const EditorSettings& settings)
{
    return settings.folderFor(EditorSettings::kFolderNewProject);
}

void ProjectUi::newProject()
{
    if (refuseWhilePlaying()) return;
    const ct::Rc<Link> link = mLink;
    guard("creating the project",
            [link]
            {
                if (link->owner) link->owner->showNewProjectDialog();
            });
}

void ProjectUi::showNewProjectDialog()
{
    if (NewProjectDialog* existing = newProjectDialog())
    {
        rt::WidgetApp::instance().bringToFront(existing);
        return;
    }
    rt::WidgetApp& app = rt::WidgetApp::instance();
    NewProjectDialog* dialog = app.addFloat<NewProjectDialog>(
            defaultParentFolder(mDocument.settings()), ct::String("NewProject"));
    const ct::Rc<Link> link = mLink;
    dialog->setCreateHandler(
            [link](const ct::String& parent, const ct::String& name)
            {
                if (!link->owner) return ct::String();
                return link->owner->createProject(parent, name);
            });
    dialog->browseRequested.connect(
            [link]
            {
                if (link->owner) link->owner->pickBrowseFolder();
            });
}

ct::String ProjectUi::createProject(const ct::String& parent, const ct::String& name)
{
    if (!mDocument.createProject(parent, name))
    {
        ct::String message = mDocument.error();
        if (message.empty()) message = "The project could not be created";
        return message;
    }
    mDocument.settings().rememberFolder(EditorSettings::kFolderNewProject, parent);
    mDocument.settings().flush();
    rt::Toast::show(ct::String("Project created: ") + name, rt::Toast::Type::Success);
    return ct::String();
}

void ProjectUi::pickBrowseFolder()
{
    NewProjectDialog* dialog = newProjectDialog();
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* chooser = app.addFloat<rt::FileDialog>("Choose the parent folder");
    chooser->setMode(rt::FileDialog::Mode::SelectFolder);
    EditorSettings* settings = &mDocument.settings();
    FileDialogMemory::prepare(*chooser, settings, EditorSettings::kFolderNewProject,
            &mDocument.project());
    if (dialog && isFolder(dialog->parentFolder())) chooser->setPath(dialog->parentFolder());
    const ct::Rc<Link> link = mLink;
    chooser->accepted.connect(
            [link, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, EditorSettings::kFolderNewProject, path);
                if (!link->owner) return;
                if (NewProjectDialog* target = link->owner->newProjectDialog())
                    target->setParentFolder(path);
            });
}

void ProjectUi::openProject()
{
    if (refuseWhilePlaying()) return;
    const ct::Rc<Link> link = mLink;
    guard("opening another project",
            [link]
            {
                if (!link->owner) return;
                ProjectUi& ui = *link->owner;
                ui.openFileChooser("Open project", EditorSettings::kFolderOpenProject, "*.json",
                        [link](const ct::String& path)
                        {
                            if (link->owner) link->owner->openProjectPath(path);
                        });
            });
}

void ProjectUi::openProjectPath(const ct::String& path)
{
    if (!mDocument.openProject(path))
    {
        reportError(mDocument.error());
        mDocument.settings().forgetProject(path);
        if (!mDocument.settingsPath().empty()) mDocument.settings().save(mDocument.settingsPath());
        return;
    }
    if (!mDocument.error().empty())
        rt::Toast::show(ct::String("The main scene could not be opened: ") + mDocument.error(),
                rt::Toast::Type::Warning, 6.0f);
    else
        rt::Toast::show(ct::String("Opened ") + mDocument.project().name(),
                rt::Toast::Type::Success);
}

void ProjectUi::openRecent(std::size_t index)
{
    if (refuseWhilePlaying()) return;
    const ct::Vector<ct::String>& recent = mDocument.settings().recentProjectPaths;
    if (index >= recent.size()) return;
    const ct::String path = recent[index];
    const ct::Rc<Link> link = mLink;
    guard("opening another project",
            [link, path]
            {
                if (link->owner) link->owner->openProjectPath(path);
            });
}

void ProjectUi::clearRecent()
{
    EditorSettings& settings = mDocument.settings();
    settings.recentProjectPaths.clear();
    if (!mDocument.settingsPath().empty()) settings.save(mDocument.settingsPath());
}

void ProjectUi::closeProject()
{
    if (refuseWhilePlaying()) return;
    const ct::Rc<Link> link = mLink;
    guard("closing the project",
            [link]
            {
                if (link->owner) link->owner->mDocument.closeProject();
            });
}

void ProjectUi::rebuildRecentMenu()
{
    if (!mRecentMenu) return;
    mRecentMenu->clearActions();
    const ct::Vector<ct::String>& recent = mDocument.settings().recentProjectPaths;
    const ct::Rc<Link> link = mLink;
    for (std::size_t i = 0; i < recent.size(); ++i)
    {
        ct::String label = nameOf(parentOf(recent[i]));
        if (label.empty()) label = recent[i];
        mRecentMenu->addAction(label,
                [link, i]
                {
                    if (link->owner) link->owner->openRecent(i);
                });
    }
    if (recent.size() > 0)
    {
        mRecentMenu->addSeparator();
        mRecentMenu->addAction("Clear recent projects",
                [link]
                {
                    if (link->owner) link->owner->clearRecent();
                });
    }
    if (mDocument.project().isOpen())
    {
        mRecentMenu->addSeparator();
        mRecentMenu->addAction("Close project",
                [link]
                {
                    if (link->owner) link->owner->closeProject();
                });
    }
}

void ProjectUi::save()
{
    if (refuseWhilePlaying()) return;
    if (!mDocument.project().isOpen())
    {
        reportError("Open or create a project to save the scene");
        return;
    }
    if (!mDocument.saveProject())
    {
        reportError(mDocument.error());
        return;
    }
    if (mDocument.hasScenePath())
    {
        if (!mDocument.saveScene())
        {
            reportError(mDocument.error());
            return;
        }
        rt::Toast::show(ct::String("Saved ") + mDocument.scenePath(), rt::Toast::Type::Success);
        return;
    }
    openSaveAsBox(ct::Function<void()>());
}

void ProjectUi::saveAs()
{
    if (refuseWhilePlaying()) return;
    openSaveAsBox(ct::Function<void()>());
}

ct::String ProjectUi::exporterPath() const
{
    ct::String path = dir_app();
    path += "/";
    path += kExporterName;
#if defined(_WIN32)
    path += ".exe";
#endif
    return file_exists(path.c_str()) ? path : ct::String();
}

void ProjectUi::exportGame()
{
    if (refuseWhilePlaying()) return;
    if (!mDocument.project().isOpen())
    {
        log_warn("editor: open a project before exporting");
        return;
    }
    if (mExport > 0 && process_is_running(mExport))
    {
        log_warn("editor: an export is still running");
        return;
    }
    if (exporterPath().empty())
    {
        log_error("editor: %s not found next to the editor, build the export target",
                kExporterName);
        return;
    }
    if (!mDocument.saveProject() || (mDocument.hasScenePath() && !mDocument.saveScene()))
    {
        reportError(mDocument.error());
        return;
    }
    const ct::Rc<Link> link = mLink;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>("Export Game");
    dialog->setMode(rt::FileDialog::Mode::Save);
    ct::String name = mDocument.project().name();
#if defined(_WIN32)
    name += ".exe";
#endif
    dialog->setFileName(name);
    EditorSettings* settings = &mDocument.settings();
    FileDialogMemory::prepare(*dialog, settings, "export", &mDocument.project());
    dialog->accepted.connect(
            [link, settings](const ct::String& path)
            {
                FileDialogMemory::remember(settings, "export", path);
                if (link->owner) link->owner->startExport(path);
            });
}

void ProjectUi::startExport(const ct::String& output)
{
    const ct::String exporter = exporterPath();
    if (exporter.empty()) return;
    const ct::String manifest = mDocument.project().manifestPath();
    const ct::String logPath = output + ".export.log";
    const char* const arguments[] = { manifest.c_str(), "-o", output.c_str(), "--log",
        logPath.c_str() };
    const long long id = process_spawn(exporter.c_str(), arguments, 5, false);
    if (id <= 0)
    {
        log_error("editor: could not start the export: %s", platform_get_error());
        return;
    }
    mExport = id;
    mExportOutput = output;
    mExportLog = logPath;
    log_info("editor: exporting to '%s'", output.c_str());
}

void ProjectUi::newScene()
{
    if (refuseWhilePlaying()) return;
    const ct::Rc<Link> link = mLink;
    guard("creating a new scene",
            [link]
            {
                if (!link->owner) return;
                if (!link->owner->mDocument.newScene())
                    link->owner->reportError(link->owner->mDocument.error());
            });
}

void ProjectUi::rebuildSceneMenu()
{
    if (!mSceneMenu) return;
    mSceneMenu->clearActions();
    const ct::Rc<Link> link = mLink;
    if (mDocument.project().isOpen())
    {
        const ct::Vector<ct::String>& scenes = mDocument.project().scenes();
        for (std::size_t i = 0; i < scenes.size(); ++i)
        {
            const ct::String scene = scenes[i];
            ct::String label = scene;
            if (scene == mDocument.project().mainScene()) label += "  (main)";
            rt::MenuAction* action =
                    mSceneMenu->addCheckable(label, scene == mDocument.scenePath());
            action->triggered.connect(
                    [link, scene]
                    {
                        if (link->owner) link->owner->openSceneNamed(scene);
                    });
        }
        if (scenes.size() > 0) mSceneMenu->addSeparator();
    }
    mSceneMenu->addAction("From file...",
            [link]
            {
                if (link->owner) link->owner->openSceneFile();
            });
}

void ProjectUi::openScene()
{
    if (refuseWhilePlaying()) return;
    rebuildSceneMenu();
    showMenu(mSceneMenu);
}

void ProjectUi::openSceneNamed(const ct::String& scene)
{
    if (refuseWhilePlaying()) return;
    const ct::Rc<Link> link = mLink;
    guard("opening another scene",
            [link, scene]
            {
                if (!link->owner) return;
                if (!link->owner->mDocument.openScene(scene))
                    link->owner->reportError(link->owner->mDocument.error());
            });
}

void ProjectUi::openSceneFile()
{
    if (refuseWhilePlaying()) return;
    const ct::Rc<Link> link = mLink;
    guard("opening another scene",
            [link]
            {
                if (!link->owner) return;
                ProjectUi& ui = *link->owner;
                ui.openFileChooser("Open scene", EditorSettings::kFolderScene, "*.json",
                        [link](const ct::String& path)
                        {
                            if (!link->owner) return;
                            if (!link->owner->mDocument.openScene(path))
                                link->owner->reportError(link->owner->mDocument.error());
                        });
            });
}

void ProjectUi::markMainScene()
{
    if (refuseWhilePlaying()) return;
    if (!mDocument.setMainScene())
    {
        reportError(mDocument.error());
        return;
    }
    rt::Toast::show(ct::String("Main scene: ") + mDocument.scenePath(), rt::Toast::Type::Success);
}

void ProjectUi::requestQuit()
{
    if (mDocument.playing() && mStopHandler) mStopHandler();
    const ct::Rc<Link> link = mLink;
    guard("closing the editor",
            [link]
            {
                if (link->owner && link->owner->mQuitHandler) link->owner->mQuitHandler();
            });
}

} // namespace prisma3d
