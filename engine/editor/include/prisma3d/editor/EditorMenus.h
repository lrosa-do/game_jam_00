#pragma once

#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/ProjectUi.h"

#include <igui/widgets/MenuWidgets.hpp>

#include <ct/ptr.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorPlay;
class LightmapPanel;
class ProjectSettingsEditor;

class EditorMenus
{
public:
    enum MenuIndex : int
    {
        kFile,
        kEdit,
        kCreate,
        kView,
        kScene,
        kRun,
        kHelp,
        kMenuCount
    };

    enum FileItem : int
    {
        kFileNewProject,
        kFileOpenProject,
        kFileRecent,
        kFileSep1,
        kFileSave,
        kFileSaveAs,
        kFileSep2,
        kFileSettings,
        kFileExport,
        kFileClose,
        kFileSep3,
        kFileQuit,
        kFileItemCount
    };

    static constexpr int kEditHeader = 3;

    enum EditItem : int
    {
        kEditUndo,
        kEditRedo,
        kEditSepHistory,
        kEditAddChild = kEditHeader + HierarchyActions::kItemAddChild,
        kEditSep1 = kEditHeader + HierarchyActions::kItemSeparator1,
        kEditDuplicate = kEditHeader + HierarchyActions::kItemDuplicate,
        kEditSep2 = kEditHeader + HierarchyActions::kItemSeparator2,
        kEditRename = kEditHeader + HierarchyActions::kItemRename,
        kEditSep3 = kEditHeader + HierarchyActions::kItemSeparator3,
        kEditMoveUp = kEditHeader + HierarchyActions::kItemMoveUp,
        kEditMoveDown = kEditHeader + HierarchyActions::kItemMoveDown,
        kEditReparent = kEditHeader + HierarchyActions::kItemReparent,
        kEditReparentToNew = kEditHeader + HierarchyActions::kItemReparentToNew,
        kEditSep4 = kEditHeader + HierarchyActions::kItemSeparator4,
        kEditDelete = kEditHeader + HierarchyActions::kItemDelete,
        kEditItemCount = kEditHeader + HierarchyActions::kItemCount
    };

    enum ViewItem : int
    {
        kViewPerspective,
        kViewOrthogonal,
        kViewSep1,
        kViewGrid,
        kViewSep2,
        kViewResetLayout,
        kViewItemCount
    };

    enum SceneItem : int
    {
        kSceneNew,
        kSceneOpen,
        kSceneSep1,
        kSceneMain,
        kSceneSep2,
        kSceneEnvironment,
        kSceneBakeLightmaps,
        kSceneItemCount
    };

    enum RunItem : int
    {
        kRunPlay,
        kRunPause,
        kRunStop,
        kRunSep1,
        kRunInRunner,
        kRunStopRunner,
        kRunItemCount
    };

    enum HelpItem : int
    {
        kHelpAbout,
        kHelpItemCount
    };

    EditorMenus(EditorShell& shell, ProjectUi& projectUi, EditorViewportPanel& viewport,
            HierarchyActions& actions, EditorPlay* play = nullptr);
    ~EditorMenus();
    EditorMenus(const EditorMenus&) = delete;
    EditorMenus& operator=(const EditorMenus&) = delete;

    void setProjectSettings(ProjectSettingsEditor* settings);
    void setLightmapPanel(LightmapPanel* panel);

    bool attach();
    bool attached() const;
    void update();

    ig::retained::Menu* menu(MenuIndex index) const;
    ig::retained::MenuAction* action(MenuIndex index, int item) const;
    static const char* menuTitle(MenuIndex index);

private:
    struct Link
    {
        EditorMenus* owner;
    };

    void build();
    std::uint64_t stateHash() const;

    EditorShell& mShell;
    ProjectUi& mProjectUi;
    EditorViewportPanel& mViewport;
    HierarchyActions& mActions;
    EditorPlay* mPlay;
    ProjectSettingsEditor* mSettings;
    LightmapPanel* mLightmap;
    ct::Rc<Link> mLink;
    ig::retained::Menu* mMenus[kMenuCount];
    std::uint64_t mHash;
    bool mBuilt;
};

} // namespace prisma3d
