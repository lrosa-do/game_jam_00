#pragma once

#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/AssetImport.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/ImportDialog.h"
#include "prisma3d/render/TextureStore.h"

#include <igui/widgets/AppWidgets.hpp>
#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/FileDialog.hpp>
#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/MenuWidgets.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/ToolBar.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/hashset.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include "mathc.h"

#include <cstddef>

namespace prisma3d
{

class EditorSelection;
class EditorViewportPanel;
class HierarchyActions;
class MaterialPanel;
class ScriptEditorPanel;
class Node;
class ProjectUi;
class SceneTreePanel;

class AssetsPanel
{
public:
    static constexpr std::size_t kThumbnailLimit = 128;
    static constexpr unsigned kThumbnailsPerFrame = 2;
    static constexpr std::size_t kHistoryLimit = 64;
    static constexpr const char* kInstantiateAction = "Instantiate Scene";
    static constexpr const char* kAttachScriptAction = "Attach Script";
    static constexpr const char* kEmptyText = "Open or create a project to browse its files.";
    static constexpr const char* kNewProjectText = "New Project...";
    static constexpr const char* kOpenProjectText = "Open Project...";

    enum Button : int
    {
        kHome,
        kBack,
        kForward,
        kUp,
        kNewFolder,
        kImport,
        kRefresh,
        kGridView,
        kListView,
        kDetailsView,
        kButtonCount
    };

    AssetsPanel(EditorDocument& document, EditorSelection& selection, SceneTreePanel& hierarchy,
            HierarchyActions& actions, ProjectUi& ui);
    ~AssetsPanel();
    AssetsPanel(const AssetsPanel&) = delete;
    AssetsPanel& operator=(const AssetsPanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    void bindViewport(EditorViewportPanel& viewport);
    void bindMaterials(MaterialPanel& materials);
    void bindScripts(ScriptEditorPanel& scripts);
    void openScript(const ct::String& path);
    void requestMaterial(const ct::String& folder);
    bool openMaterial(const ct::String& path);
    void update(float delta);
    void setFocusProbe(const ct::Function<bool()>& probe);

    int buttonId(Button button) const;
    static const char* buttonTooltip(Button button);
    static ig::retained::IconId buttonIcon(Button button);

    bool hasProject() const;
    const ct::String& folder() const;
    bool navigateTo(const ct::String& folder);
    bool goBack();
    bool goForward();
    bool goUp();
    bool goHome();
    bool canGoBack() const;
    bool canGoForward() const;
    bool canGoUp() const;
    std::size_t historySize() const;

    void refresh();
    unsigned listingCount() const;
    const ct::Vector<AssetEntry>& entries();
    const ct::Vector<AssetEntry>& shown();
    int indexOf(const ct::String& name);
    void setFilter(const ct::String& filter);
    const ct::String& filter() const;
    void setViewMode(AssetViewMode mode);
    AssetViewMode viewMode() const;
    void setThumbnailSize(float size);

    void select(int index);
    int selected() const;
    void activate(int index);
    AssetThumbnail thumbnailFor(const AssetEntry& entry);
    std::size_t thumbnailCount() const;

    bool createFolder(const ct::String& parent, const ct::String& name, ct::String& error);
    bool createScript(const ct::String& parent, const ct::String& name, ct::String& error);
    void requestFolder(const ct::String& parent);
    void requestScript(const ct::String& parent);
    void requestDelete(const ct::String& path);
    bool renamePath(const ct::String& path, const ct::String& name, ct::String& error);
    void requestRename(const ct::String& path);
    bool deletePath(const ct::String& path, ct::String& error);
    void requestImport(const ct::String& folder);
    void importFiles(const ct::Vector<ct::String>& sources, const ct::String& folder);
    bool importing() const;
    bool copyPath(const ct::String& path);
    void convertToPMesh(const ct::String& path);
    const ct::String& lastCopied() const;
    void showMenu(int index, float x, float y);

    Node* instantiate(const ct::String& path, Node* parent, const Math::Vec3* worldPosition,
            const ImportOptions* options);
    Node* instantiateAt(const ct::String& path, const Math::Vec3& worldPosition);
    Node* instantiateOn(const ct::String& path, Node* parent);
    bool attachScript(const ct::String& path, Node* node);
    Node* dropTarget(float x, float y);
    Node* materialRowAt(float x, float y);

    static bool validEntryName(const ct::String& name, ct::String& reason);
    static ct::String scriptClassName(const ct::String& name);
    static ct::String scriptTemplate(const ct::String& name);
    static ct::String scriptFileName(const ct::String& name);

    ig::retained::ToolBar* toolBar() const;
    ig::retained::Breadcrumbs* breadcrumbs() const;
    ig::retained::TextInput* searchInput() const;
    ig::retained::Slider* zoomSlider() const;
    ig::retained::Splitter* splitter() const;
    ig::retained::TreeView* tree() const;
    AssetGrid* grid() const;
    ig::retained::Label* footer() const;
    ig::retained::Menu* contextMenu() const;
    ig::retained::Widget* root() const;
    ig::retained::Widget* emptyState() const;
    ig::retained::Button* newProjectButton() const;
    ig::retained::Button* openProjectButton() const;
    ig::retained::Dialog* confirmDialog() const;
    ig::retained::FileDialog* fileDialog() const;
    ig::retained::InputBox* nameBox() const;
    ImportDialog* importDialog() const;
    ig::retained::TreeNode* treeNodeOf(const ct::String& path);
    ct::String pathOfTreeNode(const ig::retained::TreeNode* node) const;
    std::size_t bookmarkCount() const;
    ct::String bookmarkPath(std::size_t index) const;

private:
    struct Link
    {
        AssetsPanel* owner;
    };

    struct Bookmark
    {
        ct::String label;
        ct::String path;
    };

    struct Thumbnail
    {
        TextureId texture;
        float aspect = 1.0f;
    };

    template<typename T>
    T* floatOfType() const;

    bool normalizeFolder(const ct::String& folder, ct::String& out) const;
    ct::String absoluteOf(const ct::String& relative) const;
    void modelCompanions(const ct::String& model, ct::Vector<ct::String>& out) const;
    bool folderExists(const ct::String& relative) const;
    ct::String displayPath(const ct::String& relative) const;
    ct::String uniqueName(const ct::String& parent, const ct::String& stem,
            const ct::String& suffix) const;
    bool visibleInTree() const;
    bool protectedPath(const ct::String& path) const;
    bool changeFolder(const ct::String& folder);
    void invalidate(bool tree);
    void ensureListing();
    void listFolder();
    void applyFilter();
    void syncProject();
    void resetForProject();
    void syncButtons();
    void syncFooter();
    void syncBreadcrumbs();
    void rebuildTree();
    void fillChildren(ig::retained::TreeNode* node, ct::String path);
    ig::retained::TreeNode* findNode(ig::retained::TreeNode* root, const ct::String& path,
            bool expand);
    ig::retained::TreeNode* addFolderNode(ig::retained::TreeNode* parent, const ct::String& label,
            const ct::String& path, bool root);
    std::size_t bestRoot(const ct::String& path) const;
    ig::retained::TreeNode* expandTo(const ct::String& path);
    void syncTree();
    void treeSelected(ig::retained::TreeNode* node);
    void treeExpanded(ig::retained::TreeNode* node);
    bool hasSubfolder(const ct::String& relative) const;
    void gridClicked(int index);
    void gridActivated(int index);
    void gridDragFinished(const ct::String& path, bool accepted);
    void breadcrumbClicked(int index);
    void toggledButton(int id, bool checked);
    void clickedButton(int id);
    void filterChanged(const ct::String& text);
    void filesDropped(const ct::Vector<ct::String>& paths);
    void openScene(const ct::String& path);
    void showReplaceDialog();
    void showImportOptions();
    void finishImport(const ImportOptions& options);
    void cancelImport();
    bool loadThumbnail(const AssetEntry& entry, Thumbnail& out);
    void clearThumbnails(bool immediate);
    void dropViewport(const ct::String& path, const Math::Vec3& position);
    void materialCreated(const ct::String& path);
    Node* editableScene() const;
    Node* defaultParent() const;
    bool usableParent(const Node* node) const;
    Node* buildModel(const ct::String& path, Node* parent, ct::String& error);
    Node* buildPrefab(const ct::String& path, Node* parent, ct::String& error);
    void removeMissingScenes();
    void report(const ct::String& message, bool error);

    EditorDocument& mDocument;
    EditorSelection& mSelection;
    SceneTreePanel& mHierarchy;
    HierarchyActions& mActions;
    ProjectUi& mUi;
    MaterialPanel* mMaterials;
    ScriptEditorPanel* mScripts;
    ct::Rc<Link> mLink;

    ig::retained::BoxLayout* mColumn;
    ig::retained::ToolBar* mToolBar;
    ig::retained::BoxLayout* mPathRow;
    ig::retained::Breadcrumbs* mBreadcrumbs;
    ig::retained::TextInput* mSearch;
    ig::retained::Slider* mZoom;
    ig::retained::Splitter* mBody;
    ig::retained::TreeView* mTree;
    AssetGrid* mGrid;
    ig::retained::Label* mFooter;
    ig::retained::Menu* mMenu;
    ig::retained::BoxLayout* mEmpty;
    ig::retained::Button* mNewProject;
    ig::retained::Button* mOpenProject;
    int mButtons[kButtonCount];

    ct::Function<bool()> mFocusProbe;
    ct::String mProjectRoot;
    ct::String mProjectName;
    bool mProjectOpen;
    bool mWasVisible;
    bool mWasFocused;

    ct::String mFolder;
    ct::Vector<ct::String> mHistory;
    std::size_t mHistoryPosition;
    ct::Vector<AssetEntry> mEntries;
    ct::Vector<AssetEntry> mShown;
    ct::String mListedFolder;
    ct::String mFilter;
    ct::String mFilterLower;
    ct::String mSelectedPath;
    unsigned mListings;
    bool mListingDirty;
    bool mTreeDirty;
    bool mTreeFollow;
    AssetViewMode mViewMode;
    bool mSyncing;
    ct::String mFooterText;

    ct::Vector<Bookmark> mBookmarks;
    ct::Vector<ig::retained::TreeNode*> mRoots;
    ct::HashMap<ig::retained::TreeNode*, ct::String> mPathOf;
    ct::HashSet<ig::retained::TreeNode*> mLoaded;

    ct::HashMap<ct::String, Thumbnail> mThumbnails;
    ct::Vector<ct::String> mThumbnailOrder;
    ct::Vector<TextureId> mDoomed;
    unsigned mThumbnailBudget;

    ImportPlan mPlan;
    ct::String mPlanFolder;
    bool mPlanActive;
    ct::String mLastCopied;
};

} // namespace prisma3d
