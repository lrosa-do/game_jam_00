#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/editor/EditorDocument.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/FileDialog.hpp>
#include <igui/widgets/MenuWidgets.hpp>
#include <igui/widgets/ToolBar.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class NewProjectDialog : public ig::retained::FloatWindow
{
public:
    IG_WIDGET_CLASS(NewProjectDialog, ig::retained::FloatWindow);

    using CreateHandler = ct::Function<ct::String(const ct::String&, const ct::String&)>;

    NewProjectDialog(const ct::String& parentFolder, const ct::String& name);

    void setCreateHandler(const CreateHandler& handler);
    void setParentFolder(const ct::String& folder);
    ct::String parentFolder() const;
    ct::String projectName() const;
    const ct::String& error() const;

    ig::retained::TextInput* parentInput() const;
    ig::retained::TextInput* nameInput() const;
    ig::retained::Button* browseButton() const;
    ig::retained::Button* createButton() const;
    ig::retained::Button* cancelButton() const;

    ig::retained::Signal<> browseRequested;

    void create();
    void cancel();

    void onKeyPress(ig::retained::KeyEvent& event) override;

private:
    CreateHandler mHandler;
    ig::retained::TextInput* mParent;
    ig::retained::TextInput* mName;
    ig::retained::Label* mError;
    ig::retained::Button* mBrowse;
    ig::retained::Button* mCreate;
    ig::retained::Button* mCancel;
    ct::String mMessage;
};

class ProjectUi
{
public:
    static constexpr const char* kExporterName = "export";

    enum Button : int
    {
        kNewProject,
        kOpenProject,
        kRecentProjects,
        kSave,
        kNewScene,
        kOpenScene,
        kSaveAs,
        kMainScene,
        kUndo,
        kRedo,
        kButtonCount
    };

    explicit ProjectUi(EditorDocument& document);
    ~ProjectUi();
    ProjectUi(const ProjectUi&) = delete;
    ProjectUi& operator=(const ProjectUi&) = delete;

    bool attach(ig::retained::ToolBar* toolBar);
    bool attached() const;
    int buttonId(Button button) const;
    static const char* buttonTooltip(Button button);
    static ig::retained::IconId buttonIcon(Button button);

    void update();
    bool handleEvent(const InputEvent& event);

    void setTitleHandler(const ct::Function<void(const ct::String&)>& handler);
    void setQuitHandler(const ct::Function<void()>& handler);
    void setStopHandler(const ct::Function<void()>& handler);

    ct::String windowTitle() const;
    unsigned int titleSerial() const;

    void newProject();
    void openProject();
    void openRecent(std::size_t index);
    void clearRecent();
    void closeProject();
    void save();
    void newScene();
    void openScene();
    void openSceneNamed(const ct::String& scene);
    void openSceneFile();
    void saveAs();
    void exportGame();
    void markMainScene();
    bool undo();
    bool redo();
    void requestQuit();

    void rebuildRecentMenu();
    void rebuildSceneMenu();

    bool confirming() const;
    ig::retained::Dialog* confirmDialog() const;
    ig::retained::Menu* recentMenu() const;
    ig::retained::Menu* sceneMenu() const;
    NewProjectDialog* newProjectDialog() const;
    ig::retained::FileDialog* fileDialog() const;
    ig::retained::InputBox* saveAsBox() const;

    EditorDocument& document();

    static ct::String defaultParentFolder(const EditorSettings& settings);

private:
    struct Link
    {
        ProjectUi* owner;
    };

    void guard(const char* action, const ct::Function<void()>& next);
    void openConfirm(const char* action, const ct::Function<void()>& next);
    void saveThen(const ct::Function<void()>& next);
    void openSaveAsBox(const ct::Function<void()>& next);
    void openFileChooser(const char* title, const char* purpose, const ct::String& filter,
            const ct::Function<void(const ct::String&)>& chosen);
    void showNewProjectDialog();
    ct::String createProject(const ct::String& parent, const ct::String& name);
    void openProjectPath(const ct::String& path);
    ct::String exporterPath() const;
    void startExport(const ct::String& output);
    void reportError(const ct::String& message);
    bool refuseWhilePlaying();
    void pickBrowseFolder();
    ct::String defaultSceneName() const;
    void clickedButton(int id);
    void showMenu(ig::retained::Menu* menu);
    void confirmSave();
    void confirmDiscard();
    void confirmCancel();

    template<typename T>
    T* floatOfType() const;

    EditorDocument& mDocument;
    ct::Rc<Link> mLink;
    ig::retained::ToolBar* mToolBar;
    int mButtons[kButtonCount];
    ig::retained::Menu* mRecentMenu;
    ig::retained::Menu* mSceneMenu;
    ct::Function<void(const ct::String&)> mTitleHandler;
    ct::Function<void()> mQuitHandler;
    ct::Function<void()> mStopHandler;
    ct::Function<void()> mPending;
    ct::String mTitle;
    ct::String mUndoTip;
    ct::String mRedoTip;
    unsigned int mTitleSerial;
    long long mExport;
    ct::String mExportOutput;
    ct::String mExportLog;
};

} // namespace prisma3d
