#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/FileDialogMemory.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/MaterialPanel.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/ScriptEditorPanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/import/PMeshFormat.h"
#include "prisma3d/import/PMeshImporter.h"
#include "prisma3d/import/PMeshWriter.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/ModelInstance.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"

#include <ct/sort.hpp>

#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include "platform.h"

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kButtonTooltips[AssetsPanel::kButtonCount] = { "Project Folder", "Back",
    "Forward", "Parent Folder", "New Folder...", "Import Files...", "Refresh", "Icon Grid", "List",
    "Details" };

const char* const kButtonIcons[AssetsPanel::kButtonCount] = { "Folder", "Back", "Forward", "MoveUp",
    "Add", "MoveDown", "Reload", "FileThumbnail", "FileList", "ListSelect" };

const float kZoomWidth = 100.0f;
const float kZoomHeight = 18.0f;
const float kSearchWidth = 170.0f;
const float kSplitRatio = 0.24f;
const std::size_t kDeleteListLimit = 6;

class ZoomSlider : public rt::Slider
{
public:
    IG_WIDGET_CLASS(ZoomSlider, rt::Slider);

    ZoomSlider(float minimum, float maximum, float value)
            : rt::Slider(minimum, maximum, value)
    {
    }

    Vec2f sizeHint() const override
    {
        return { kZoomWidth, kZoomHeight };
    }
};

struct DirItem
{
    ct::String name;
    bool directory = false;
    std::uint64_t size = 0;
};

struct ListContext
{
    ct::Vector<DirItem>* items;
    bool stopAtFolder;
    bool found;
};

bool collectItem(const char* path, PathType type, void* user)
{
    ListContext* context = static_cast<ListContext*>(user);
    const ct::String name = AssetImport::nameOf(ct::String(path));
    if (name.empty() || name[0] == '.') return true;
    const bool directory = type == PATH_TYPE_DIRECTORY;
    if (!directory && type != PATH_TYPE_FILE) return true;
    if (context->stopAtFolder)
    {
        if (!directory) return true;
        context->found = true;
        return false;
    }
    DirItem item;
    item.name = name;
    item.directory = directory;
    if (!directory)
    {
        PathInfo info;
        if (fs_get_path_info(path, &info) && info.size > 0)
            item.size = static_cast<std::uint64_t>(info.size);
    }
    context->items->push_back(item);
    return true;
}

bool lessName(const ct::String& a, const ct::String& b)
{
    const ct::String first = AssetTypes::lowered(a);
    const ct::String second = AssetTypes::lowered(b);
    const int order = first.compare(second);
    if (order != 0) return order < 0;
    return a.compare(b) < 0;
}

bool lessItem(const DirItem& a, const DirItem& b)
{
    if (a.directory != b.directory) return a.directory;
    return lessName(a.name, b.name);
}

struct CollectedPaths
{
    ct::Vector<ct::String> paths;
};

bool collectPath(const char* path, PathType type, void* user)
{
    (void) type;
    static_cast<CollectedPaths*>(user)->paths.push_back(ct::String(path));
    return true;
}

bool longerPath(const ct::String& a, const ct::String& b)
{
    return a.size() > b.size();
}

ct::String parentFolderOf(const ct::String& folder)
{
    const std::size_t slash = folder.rfind('/');
    if (slash == ct::String::npos) return ct::String();
    return folder.substr(0, slash);
}

bool startsWithFolder(const ct::String& path, const ct::String& folder)
{
    if (folder.empty()) return true;
    if (path == folder) return true;
    return path.size() > folder.size() && path.starts_with(folder) && path[folder.size()] == '/';
}

} // namespace

AssetsPanel::AssetsPanel(EditorDocument& document, EditorSelection& selection,
        SceneTreePanel& hierarchy, HierarchyActions& actions, ProjectUi& ui)
        : mDocument(document),
          mSelection(selection),
          mHierarchy(hierarchy),
          mActions(actions),
          mUi(ui),
          mMaterials(nullptr),
          mScripts(nullptr),
          mLink(ct::make_rc<Link>(Link{ this })),
          mColumn(nullptr),
          mToolBar(nullptr),
          mPathRow(nullptr),
          mBreadcrumbs(nullptr),
          mSearch(nullptr),
          mZoom(nullptr),
          mBody(nullptr),
          mTree(nullptr),
          mGrid(nullptr),
          mFooter(nullptr),
          mMenu(nullptr),
          mEmpty(nullptr),
          mNewProject(nullptr),
          mOpenProject(nullptr),
          mButtons{},
          mProjectOpen(false),
          mWasVisible(false),
          mWasFocused(false),
          mHistoryPosition(0),
          mListings(0),
          mListingDirty(true),
          mTreeDirty(true),
          mTreeFollow(true),
          mViewMode(AssetViewMode::Grid),
          mSyncing(false),
          mThumbnailBudget(0),
          mPlanActive(false)
{
    for (int& id: mButtons) id = -1;
    mHistory.push_back(ct::String());
}

AssetsPanel::~AssetsPanel()
{
    mLink->owner = nullptr;
    clearThumbnails(true);
    delete mMenu;
}

rt::IconId AssetsPanel::buttonIcon(Button button)
{
    return button >= 0 && button < kButtonCount ? EditorIcons::get(kButtonIcons[button])
                                                : rt::IconId::None;
}

const char* AssetsPanel::buttonTooltip(Button button)
{
    return button >= 0 && button < kButtonCount ? kButtonTooltips[button] : "";
}

bool AssetsPanel::attach(rt::Widget* parent)
{
    if (mColumn || !parent) return false;
    const ct::Rc<Link> link = mLink;

    mColumn = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mColumn->setPadding(0.0f);
    mColumn->setSpacing(0.0f);

    mPathRow = mColumn->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    mPathRow->setPadding(2.0f, 4.0f);
    mPathRow->setSpacing(6.0f);

    mToolBar = mPathRow->createChild<rt::ToolBar>();
    mToolBar->setButtonSize(28.0f);
    mToolBar->setIconPadding(6.0f);
    mToolBar->setDraggable(false);
    for (int i = 0; i < kButtonCount; ++i)
    {
        if (i == kNewFolder || i == kGridView) mToolBar->addSeparator();
        const bool toggle = i >= kGridView;
        const rt::IconId icon = buttonIcon(static_cast<Button>(i));
        mButtons[i] = toggle ? mToolBar->addToggle(icon, kButtonTooltips[i],
                                       static_cast<AssetViewMode>(i - kGridView) == mViewMode)
                             : mToolBar->addButton(icon, kButtonTooltips[i]);
        mToolBar->setItemDraggable(mButtons[i], false);
    }
    mToolBar->buttonClicked.connect(
            [link](int id)
            {
                if (link->owner) link->owner->clickedButton(id);
            });
    mToolBar->toggleChanged.connect(
            [link](int id, bool checked)
            {
                if (link->owner) link->owner->toggledButton(id, checked);
            });

    mBreadcrumbs = mPathRow->createChild<rt::Breadcrumbs>();
    mBreadcrumbs->setSeparator("/");
    mBreadcrumbs->setStretch(1.0f);
    mBreadcrumbs->itemClicked.connect(
            [link](int index)
            {
                if (link->owner) link->owner->breadcrumbClicked(index);
            });
    mSearch = mPathRow->createChild<rt::TextInput>();
    mSearch->setPlaceholder("Search");
    mSearch->setSize(kSearchWidth, 0.0f);
    mSearch->textChanged.connect(
            [link](const ct::String& text)
            {
                if (link->owner) link->owner->filterChanged(text);
            });
    mZoom = mPathRow->createChild<ZoomSlider>(AssetGrid::kMinThumbnail, AssetGrid::kMaxThumbnail,
            96.0f);
    mZoom->setLayoutAlign(rt::Align::Center);
    mZoom->setTooltip("Thumbnail size");
    mZoom->onValueChanged.connect(
            [link](float value)
            {
                if (link->owner) link->owner->setThumbnailSize(value);
            });

    mBody = mColumn->createChild<rt::Splitter>(rt::LayoutDir::Horizontal);
    mBody->setStretch(1.0f);
    mBody->setRatio(kSplitRatio);
    mBody->setMinRatio(0.08f);
    mBody->setMaxRatio(0.6f);
    mTree = mBody->createChild<rt::TreeView>();
    mTree->selectionChanged.connect(
            [link](rt::TreeNode* node)
            {
                if (link->owner) link->owner->treeSelected(node);
            });
    mTree->nodeExpanded.connect(
            [link](rt::TreeNode* node)
            {
                if (link->owner) link->owner->treeExpanded(node);
            });
    mGrid = mBody->createChild<AssetGrid>();
    mGrid->setThumbnailProvider([link](const AssetEntry& entry)
            { return link->owner ? link->owner->thumbnailFor(entry) : AssetThumbnail(); });
    mGrid->itemClicked.connect(
            [link](int index)
            {
                if (link->owner) link->owner->gridClicked(index);
            });
    mGrid->itemActivated.connect(
            [link](int index)
            {
                if (link->owner) link->owner->gridActivated(index);
            });
    mGrid->contextRequested.connect(
            [link](int index, float x, float y)
            {
                if (link->owner) link->owner->showMenu(index, x, y);
            });
    mGrid->dragFinished.connect(
            [link](const ct::String& path, bool accepted)
            {
                if (link->owner) link->owner->gridDragFinished(path, accepted);
            });

    mEmpty = mColumn->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mEmpty->setStretch(1.0f);
    mEmpty->setMainAlign(rt::MainAlign::Center);
    mEmpty->setSpacing(10.0f);
    rt::Label* emptyText = mEmpty->createChild<rt::Label>("No project is open.");
    emptyText->setAlign(rt::TextAlign::CENTER);
    emptyText->setSize(0.0f, 22.0f);
    rt::BoxLayout* emptyRow = mEmpty->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    emptyRow->setSize(0.0f, 32.0f);
    emptyRow->setSpacing(10.0f);
    emptyRow->setPadding(0.0f);
    emptyRow->setMainAlign(rt::MainAlign::Center);
    mNewProject = emptyRow->createChild<rt::Button>(kNewProjectText);
    mNewProject->setSize(140.0f, 0.0f);
    mNewProject->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->mUi.newProject();
            });
    mOpenProject = emptyRow->createChild<rt::Button>(kOpenProjectText);
    mOpenProject->setSize(140.0f, 0.0f);
    mOpenProject->clicked.connect(
            [link]
            {
                if (link->owner) link->owner->mUi.openProject();
            });

    mFooter = mColumn->createChild<rt::Label>("");
    mFooter->setSize(0.0f, 22.0f);

    mColumn->fileDrop.connect(
            [link](const ct::Vector<ct::String>& paths)
            {
                if (link->owner) link->owner->filesDropped(paths);
            });

    mMenu = new rt::Menu();
    const EditorSettings& stored = mDocument.settings();
    if (stored.assetViewMode >= 0 && stored.assetViewMode < 3)
        setViewMode(static_cast<AssetViewMode>(stored.assetViewMode));
    if (stored.thumbnailSize >= AssetGrid::kMinThumbnail &&
            stored.thumbnailSize <= AssetGrid::kMaxThumbnail)
        setThumbnailSize(stored.thumbnailSize);
    syncButtons();
    return true;
}

bool AssetsPanel::attached() const
{
    return mColumn != nullptr;
}

void AssetsPanel::bindViewport(EditorViewportPanel& viewport)
{
    const ct::Rc<Link> link = mLink;
    viewport.setScriptDropHandler(
            [link](const ct::String& path, Node* node)
            {
                if (link->owner) link->owner->attachScript(path, node);
            });
    viewport.setAssetDropHandler(
            [link](const ct::String& path, const Math::Vec3& position)
            {
                if (link->owner) link->owner->dropViewport(path, position);
            });
}

void AssetsPanel::bindMaterials(MaterialPanel& materials)
{
    mMaterials = &materials;
    const ct::Rc<Link> link = mLink;
    materials.setCreatedHandler(
            [link](const ct::String& path)
            {
                if (link->owner) link->owner->materialCreated(path);
            });
    materials.setRowProbe([link](float x, float y) -> Node*
            { return link->owner ? link->owner->materialRowAt(x, y) : nullptr; });
}

void AssetsPanel::requestMaterial(const ct::String& folder)
{
    if (!mProjectOpen || !mMaterials) return;
    ct::String base;
    if (!normalizeFolder(folder, base) || !folderExists(base)) base = mFolder;
    mMaterials->requestNew(base, MaterialPanel::kNoSlot);
}

void AssetsPanel::bindScripts(ScriptEditorPanel& scripts)
{
    mScripts = &scripts;
}

void AssetsPanel::openScript(const ct::String& path)
{
    if (mScripts) mScripts->open(absoluteOf(path));
}

bool AssetsPanel::openMaterial(const ct::String& path)
{
    return mMaterials && mMaterials->openLibrary(path);
}

void AssetsPanel::materialCreated(const ct::String& path)
{
    invalidate(false);
    ensureListing();
    if (AssetImport::parentOf(path) == mFolder) select(indexOf(AssetImport::nameOf(path)));
}

void AssetsPanel::setFocusProbe(const ct::Function<bool()>& probe)
{
    mFocusProbe = probe;
}

int AssetsPanel::buttonId(Button button) const
{
    return button >= 0 && button < kButtonCount ? mButtons[button] : -1;
}

bool AssetsPanel::visibleInTree() const
{
    for (const rt::Widget* widget = mColumn; widget; widget = widget->parent())
        if (!widget->isVisible()) return false;
    return mColumn != nullptr;
}

bool AssetsPanel::hasProject() const
{
    return mProjectOpen;
}

const ct::String& AssetsPanel::folder() const
{
    return mFolder;
}

ct::String AssetsPanel::absoluteOf(const ct::String& relative) const
{
    const Project& project = mDocument.project();
    if (relative.empty()) return project.root();
    return project.absolutePath(relative);
}

void AssetsPanel::modelCompanions(const ct::String& model, ct::Vector<ct::String>& out) const
{
    const ct::String material = AssetImport::stemOf(model) + ".mat";
    if (file_exists(absoluteOf(material).c_str())) out.push_back(material);
    if (AssetTypes::extensionOf(AssetImport::nameOf(model)) != pmesh::kMeshExtension) return;
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(absoluteOf(model));
    ct::Vector<ct::String> skeletons;
    ct::Vector<ct::String> animations;
    ct::String error;
    if (bytes.empty() || !readPMeshLinks(bytes.data(), bytes.size(), skeletons, animations, error))
        return;
    for (std::size_t i = 0; i < animations.size(); ++i) skeletons.push_back(animations[i]);
    const ct::String folder = AssetImport::parentOf(model);
    for (std::size_t i = 0; i < skeletons.size(); ++i)
    {
        ct::String link = skeletons[i];
        for (std::size_t c = 0; c < link.size(); ++c)
            if (link[c] == '\\') link[c] = '/';
        ct::String path = AssetImport::joinPath(folder, link);
        if (!file_exists(absoluteOf(path).c_str())) path = link;
        if (link.empty() || !file_exists(absoluteOf(path).c_str())) continue;
        bool seen = false;
        for (std::size_t k = 0; k < out.size() && !seen; ++k) seen = out[k] == path;
        if (!seen) out.push_back(path);
    }
}

bool AssetsPanel::folderExists(const ct::String& relative) const
{
    if (!mProjectOpen) return false;
    return dir_exists(absoluteOf(relative).c_str());
}

ct::String AssetsPanel::displayPath(const ct::String& relative) const
{
    if (!relative.empty()) return relative;
    const ct::String& name = mDocument.project().name();
    return name.empty() ? ct::String("project") : name;
}

bool AssetsPanel::normalizeFolder(const ct::String& folder, ct::String& out) const
{
    out = ct::String();
    ct::String text = folder.trimmed();
    for (std::size_t i = 0; i < text.size(); ++i)
        if (text.data()[i] == '\\') text.data()[i] = '/';
    while (text.ends_with("/")) text.pop_back();
    if (text.empty() || text == ".") return true;
    ct::String reason;
    return Project::normalizeRelativePath(text, out, reason);
}

bool AssetsPanel::protectedPath(const ct::String& path) const
{
    const Project& project = mDocument.project();
    if (path.empty()) return true;
    return path == Project::kScenesFolder || path == project.assetRoot();
}

void AssetsPanel::invalidate(bool tree)
{
    mListingDirty = true;
    if (tree) mTreeDirty = true;
}

void AssetsPanel::refresh()
{
    invalidate(true);
    clearThumbnails(false);
    ensureListing();
}

unsigned AssetsPanel::listingCount() const
{
    return mListings;
}

const ct::Vector<AssetEntry>& AssetsPanel::entries()
{
    ensureListing();
    return mEntries;
}

const ct::Vector<AssetEntry>& AssetsPanel::shown()
{
    ensureListing();
    return mShown;
}

int AssetsPanel::indexOf(const ct::String& name)
{
    ensureListing();
    for (std::size_t i = 0; i < mShown.size(); ++i)
        if (mShown[i].name == name) return static_cast<int>(i);
    return -1;
}

void AssetsPanel::setFilter(const ct::String& filter)
{
    if (mSearch && mSearch->text() != filter) mSearch->setText(filter);
    if (filter == mFilter) return;
    mFilter = filter;
    mFilterLower = AssetTypes::lowered(filter.trimmed());
    if (!mListingDirty) applyFilter();
}

const ct::String& AssetsPanel::filter() const
{
    return mFilter;
}

void AssetsPanel::filterChanged(const ct::String& text)
{
    setFilter(text);
}

void AssetsPanel::setViewMode(AssetViewMode mode)
{
    mViewMode = mode;
    mDocument.settings().assetViewMode = static_cast<int>(mode);
    if (mGrid) mGrid->setViewMode(mode);
    if (!mToolBar) return;
    mSyncing = true;
    for (int i = kGridView; i < kButtonCount; ++i)
        mToolBar->setItemChecked(mButtons[i], static_cast<AssetViewMode>(i - kGridView) == mode);
    mSyncing = false;
}

AssetViewMode AssetsPanel::viewMode() const
{
    return mViewMode;
}

void AssetsPanel::setThumbnailSize(float size)
{
    mDocument.settings().thumbnailSize = size;
    if (mGrid) mGrid->setThumbnailSize(size);
    if (mZoom && mZoom->value() != size) mZoom->setValue(size);
}

bool AssetsPanel::changeFolder(const ct::String& folder)
{
    mFolder = folder;
    mListingDirty = true;
    mTreeFollow = true;
    if (mSearch && !mSearch->text().empty()) mSearch->setText(ct::String());
    mFilter = ct::String();
    mFilterLower = ct::String();
    mSelectedPath = ct::String();
    if (mGrid)
    {
        mGrid->setSelected(-1);
        mGrid->setScroll(0.0f);
    }
    return true;
}

bool AssetsPanel::navigateTo(const ct::String& folder)
{
    syncProject();
    if (!mProjectOpen) return false;
    ct::String normalized;
    if (!normalizeFolder(folder, normalized) || !folderExists(normalized)) return false;
    if (normalized == mFolder) return true;
    if (mHistoryPosition + 1 < mHistory.size())
        mHistory.erase(mHistory.begin() + static_cast<std::ptrdiff_t>(mHistoryPosition) + 1,
                mHistory.end());
    mHistory.push_back(normalized);
    if (mHistory.size() > kHistoryLimit) mHistory.erase(mHistory.begin());
    mHistoryPosition = mHistory.size() - 1;
    changeFolder(normalized);
    return true;
}

bool AssetsPanel::goBack()
{
    if (!canGoBack()) return false;
    --mHistoryPosition;
    changeFolder(mHistory[mHistoryPosition]);
    return true;
}

bool AssetsPanel::goForward()
{
    if (!canGoForward()) return false;
    ++mHistoryPosition;
    changeFolder(mHistory[mHistoryPosition]);
    return true;
}

bool AssetsPanel::goUp()
{
    if (!canGoUp()) return false;
    return navigateTo(parentFolderOf(mFolder));
}

bool AssetsPanel::goHome()
{
    return navigateTo(ct::String());
}

bool AssetsPanel::canGoBack() const
{
    return mProjectOpen && mHistoryPosition > 0;
}

bool AssetsPanel::canGoForward() const
{
    return mProjectOpen && mHistoryPosition + 1 < mHistory.size();
}

bool AssetsPanel::canGoUp() const
{
    return mProjectOpen && !mFolder.empty();
}

std::size_t AssetsPanel::historySize() const
{
    return mHistory.size();
}

void AssetsPanel::listFolder()
{
    bool moved = false;
    while (!mFolder.empty() && !folderExists(mFolder))
    {
        mFolder = parentFolderOf(mFolder);
        moved = true;
        mTreeFollow = true;
    }
    if (moved && mHistoryPosition < mHistory.size()) mHistory[mHistoryPosition] = mFolder;

    ct::Vector<DirItem> items;
    ListContext context{ &items, false, false };
    fs_enumerate_directory(absoluteOf(mFolder).c_str(), false, &collectItem, &context);
    ct::sort(items.begin(), items.end(), &lessItem);

    mEntries.clear();
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        AssetEntry entry;
        entry.name = items[i].name;
        entry.path = AssetImport::joinPath(mFolder, items[i].name);
        entry.directory = items[i].directory;
        entry.kind = AssetTypes::classify(items[i].name, items[i].directory);
        entry.size = items[i].size;
        mEntries.push_back(entry);
    }
    ++mListings;
    mListingDirty = false;
    mListedFolder = mFolder;
    applyFilter();
}

void AssetsPanel::applyFilter()
{
    if (mGrid)
    {
        const int current = mGrid->selected();
        if (current >= 0 && current < static_cast<int>(mShown.size()))
            mSelectedPath = mShown[static_cast<std::size_t>(current)].path;
    }
    mShown.clear();
    for (std::size_t i = 0; i < mEntries.size(); ++i)
    {
        if (!mFilterLower.empty() && !AssetTypes::lowered(mEntries[i].name).contains(mFilterLower))
            continue;
        mShown.push_back(mEntries[i]);
    }
    if (!mGrid) return;
    mGrid->setItems(mShown);
    if (mSelectedPath.empty()) return;
    for (std::size_t i = 0; i < mShown.size(); ++i)
        if (mShown[i].path == mSelectedPath)
        {
            mGrid->setSelected(static_cast<int>(i));
            break;
        }
}

void AssetsPanel::ensureListing()
{
    syncProject();
    if (!mProjectOpen) return;
    if (mListingDirty || mListedFolder != mFolder) listFolder();
    if (mTreeDirty) rebuildTree();
    if (mTreeFollow) syncTree();
    syncBreadcrumbs();
}

void AssetsPanel::syncProject()
{
    const Project& project = mDocument.project();
    const bool open = project.isOpen();
    const ct::String root = open ? project.root() : ct::String();
    const ct::String name = open ? project.name() : ct::String();
    if (open == mProjectOpen && root == mProjectRoot)
    {
        if (name != mProjectName)
        {
            mProjectName = name;
            mTreeDirty = true;
            syncBreadcrumbs();
        }
        return;
    }
    mProjectOpen = open;
    mProjectRoot = root;
    mProjectName = name;
    resetForProject();
}

void AssetsPanel::resetForProject()
{
    clearThumbnails(false);
    mHistory.clear();
    mHistory.push_back(ct::String());
    mHistoryPosition = 0;
    mFolder = ct::String();
    mListedFolder = ct::String();
    mEntries.clear();
    mShown.clear();
    mFilter = ct::String();
    mFilterLower = ct::String();
    mSelectedPath = ct::String();
    mPlanActive = false;
    if (mSearch && !mSearch->text().empty()) mSearch->setText(ct::String());
    if (mGrid)
    {
        mGrid->setItems(mShown);
        mGrid->setScroll(0.0f);
    }
    invalidate(true);
    if (!mProjectOpen && mTree)
    {
        mTree->clear();
        mPathOf.clear();
        mLoaded.clear();
        mRoots.clear();
        mBookmarks.clear();
        mTreeDirty = false;
    }
    syncBreadcrumbs();
}

void AssetsPanel::update(float delta)
{
    (void) delta;
    if (!mColumn) return;
    for (std::size_t i = 0; i < mDoomed.size(); ++i)
        if (TextureStore* store = TextureStore::singleton()) store->destroy(mDoomed[i]);
    mDoomed.clear();
    mThumbnailBudget = 0;

    syncProject();
    const bool visible = visibleInTree();
    bool focused = false;
    if (mFocusProbe) focused = mFocusProbe();
    else if (Device* device = Device::singleton())
        focused = device->windowIsFocused();
    if (mProjectOpen && visible && (!mWasVisible || (focused && !mWasFocused))) invalidate(true);
    mWasVisible = visible;
    mWasFocused = focused;

    if (mGrid)
    {
        const int current = mGrid->selected();
        mSelectedPath = current >= 0 && current < static_cast<int>(mShown.size())
                                ? mShown[static_cast<std::size_t>(current)].path
                                : ct::String();
    }
    if (mProjectOpen && visible) ensureListing();
    syncButtons();
    syncFooter();
}

void AssetsPanel::syncButtons()
{
    if (!mToolBar) return;
    const bool enabled[kButtonCount] = { mProjectOpen && !mFolder.empty(), canGoBack(),
        canGoForward(), canGoUp(), mProjectOpen, mProjectOpen, mProjectOpen, mProjectOpen,
        mProjectOpen, mProjectOpen };
    for (int i = 0; i < kButtonCount; ++i)
        if (mToolBar->isItemEnabled(mButtons[i]) != enabled[i])
            mToolBar->setItemEnabled(mButtons[i], enabled[i]);
    if (mSearch) mSearch->setEnabled(mProjectOpen);
    if (mPathRow && mPathRow->isVisible() != mProjectOpen) mPathRow->setVisible(mProjectOpen);
    if (mBody && mBody->isVisible() != mProjectOpen) mBody->setVisible(mProjectOpen);
    if (mEmpty && mEmpty->isVisible() == mProjectOpen) mEmpty->setVisible(!mProjectOpen);
}

void AssetsPanel::syncBreadcrumbs()
{
    if (!mBreadcrumbs) return;
    ct::Vector<ct::String> segments;
    if (mProjectOpen)
    {
        segments.push_back(displayPath(ct::String()));
        std::size_t start = 0;
        for (std::size_t i = 0; i <= mFolder.size(); ++i)
        {
            if (i != mFolder.size() && mFolder[i] != '/') continue;
            if (i > start) segments.push_back(mFolder.substr(start, i - start));
            start = i + 1;
        }
    }
    if (segments.size() == mBreadcrumbs->path().size())
    {
        bool same = true;
        for (std::size_t i = 0; i < segments.size() && same; ++i)
            same = segments[i] == mBreadcrumbs->path()[i];
        if (same) return;
    }
    mBreadcrumbs->setPath(segments);
}

void AssetsPanel::breadcrumbClicked(int index)
{
    if (!mProjectOpen || index < 0) return;
    ct::String folder;
    std::size_t depth = 0;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= mFolder.size() && static_cast<int>(depth) < index; ++i)
    {
        if (i != mFolder.size() && mFolder[i] != '/') continue;
        if (i > start)
        {
            folder = mFolder.substr(0, i);
            ++depth;
        }
        start = i + 1;
    }
    navigateTo(folder);
}

void AssetsPanel::syncFooter()
{
    if (!mFooter) return;
    ct::String text;
    if (!mProjectOpen) text = ct::String();
    else
    {
        text = displayPath(mFolder);
        text += "  -  ";
        if (mShown.size() != mEntries.size())
        {
            text.append_number(static_cast<unsigned long long>(mShown.size()));
            text += " of ";
        }
        text.append_number(static_cast<unsigned long long>(mEntries.size()));
        text += mEntries.size() == 1 ? " item" : " items";
        const int current = mGrid ? mGrid->selected() : -1;
        if (current >= 0 && current < static_cast<int>(mShown.size()))
        {
            const AssetEntry& entry = mShown[static_cast<std::size_t>(current)];
            text += "  -  ";
            text += entry.name;
            text += " (";
            text += AssetTypes::label(entry.kind);
            text += ")";
        }
    }
    if (text == mFooterText) return;
    mFooterText = text;
    mFooter->setText(text);
}

void AssetsPanel::select(int index)
{
    ensureListing();
    if (mGrid) mGrid->setSelected(index);
    if (index >= 0 && index < static_cast<int>(mShown.size()))
        mSelectedPath = mShown[static_cast<std::size_t>(index)].path;
    else
        mSelectedPath = ct::String();
}

int AssetsPanel::selected() const
{
    return mGrid ? mGrid->selected() : -1;
}

void AssetsPanel::gridClicked(int index)
{
    if (index >= 0 && index < static_cast<int>(mShown.size()))
        mSelectedPath = mShown[static_cast<std::size_t>(index)].path;
}

void AssetsPanel::gridActivated(int index)
{
    activate(index);
}

void AssetsPanel::activate(int index)
{
    ensureListing();
    if (index < 0 || index >= static_cast<int>(mShown.size())) return;
    const AssetEntry entry = mShown[static_cast<std::size_t>(index)];
    switch (entry.kind)
    {
        case AssetKind::Folder:
            navigateTo(entry.path);
            break;
        case AssetKind::Scene:
            openScene(entry.path);
            break;
        case AssetKind::Prefab:
        case AssetKind::Model:
            instantiate(entry.path, nullptr, nullptr, nullptr);
            break;
        case AssetKind::Material:
            select(index);
            openMaterial(entry.path);
            break;
        case AssetKind::Script:
            select(index);
            openScript(entry.path);
            break;
        default:
            select(index);
            break;
    }
}

void AssetsPanel::openScene(const ct::String& path)
{
    if (path == mDocument.scenePath()) return;
    mUi.openSceneNamed(path);
}

void AssetsPanel::report(const ct::String& message, bool error)
{
    if (error) log_error("editor: %s", message.c_str());
    rt::Toast::show(message, error ? rt::Toast::Type::Error : rt::Toast::Type::Success,
            error ? 6.0f : rt::Toast::kDefaultDuration);
}

bool AssetsPanel::hasSubfolder(const ct::String& relative) const
{
    ListContext context{ nullptr, true, false };
    fs_enumerate_directory(absoluteOf(relative).c_str(), false, &collectItem, &context);
    return context.found;
}

rt::TreeNode* AssetsPanel::addFolderNode(rt::TreeNode* parent, const ct::String& label,
        const ct::String& path, bool root)
{
    rt::TreeNode* node = nullptr;
    if (root) node = mTree->addRoot(label);
    else
        node = parent->addChild(label);
    node->setIconId(EditorIcons::get("Folder"));
    mPathOf.put(node, path);
    if (hasSubfolder(path)) node->addChild(ct::String());
    node->setExpanded(false);
    return node;
}

void AssetsPanel::fillChildren(rt::TreeNode* node, ct::String path)
{
    ct::Vector<rt::TreeNode*> stale;
    for (std::size_t i = 0; i < node->children().size(); ++i) stale.push_back(node->children()[i]);
    while (!stale.empty())
    {
        rt::TreeNode* current = stale.back();
        stale.pop_back();
        mPathOf.erase(current);
        mLoaded.erase(current);
        for (std::size_t i = 0; i < current->children().size(); ++i)
            stale.push_back(current->children()[i]);
    }
    node->clear();

    ct::Vector<DirItem> items;
    ListContext context{ &items, false, false };
    fs_enumerate_directory(absoluteOf(path).c_str(), false, &collectItem, &context);
    ct::sort(items.begin(), items.end(), &lessItem);
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        if (!items[i].directory) continue;
        addFolderNode(node, items[i].name, AssetImport::joinPath(path, items[i].name), false);
    }
    mLoaded.insert(node);
}

void AssetsPanel::treeExpanded(rt::TreeNode* node)
{
    if (!node || mLoaded.contains(node)) return;
    const ct::String* path = mPathOf.find(node);
    if (!path) return;
    const ct::String copy = *path;
    fillChildren(node, copy);
}

void AssetsPanel::rebuildTree()
{
    mTreeDirty = false;
    if (!mTree) return;

    struct Record
    {
        ct::String root;
        ct::String path;
        bool expanded;
    };
    ct::Vector<Record> records;
    for (std::size_t r = 0; r < mRoots.size() && r < mBookmarks.size(); ++r)
    {
        ct::Vector<rt::TreeNode*> pending;
        pending.push_back(mRoots[r]);
        while (!pending.empty())
        {
            rt::TreeNode* node = pending.back();
            pending.pop_back();
            const ct::String* path = mPathOf.find(node);
            if (!path || !mLoaded.contains(node)) continue;
            records.push_back({ mBookmarks[r].path, *path, node->isExpanded() });
            for (std::size_t c = node->children().size(); c > 0; --c)
                pending.push_back(node->children()[c - 1]);
        }
    }

    mTree->clear();
    mPathOf.clear();
    mLoaded.clear();
    mRoots.clear();
    mBookmarks.clear();
    if (!mProjectOpen) return;

    const Project& project = mDocument.project();
    const ct::String assets = project.assetRoot();
    const Bookmark wanted[] = {
        { displayPath(ct::String()), ct::String() },
        { Project::kScenesFolder, ct::String(Project::kScenesFolder) },
        { assets, assets },
        { "Models", AssetImport::joinPath(assets, "Models") },
        { "Textures", AssetImport::joinPath(assets, "Textures") },
        { "Scripts", AssetImport::joinPath(assets, "Scripts") },
    };
    for (const Bookmark& bookmark: wanted)
    {
        if (!bookmark.path.empty() && !folderExists(bookmark.path)) continue;
        bool duplicate = false;
        for (std::size_t i = 0; i < mBookmarks.size(); ++i)
            duplicate = duplicate || mBookmarks[i].path == bookmark.path;
        if (duplicate) continue;
        mBookmarks.push_back(bookmark);
        mRoots.push_back(addFolderNode(nullptr, bookmark.label, bookmark.path, true));
    }
    if (!mRoots.empty())
    {
        fillChildren(mRoots[0], mBookmarks[0].path);
        mRoots[0]->setExpanded(true);
    }

    for (std::size_t i = 0; i < records.size(); ++i)
    {
        rt::TreeNode* root = nullptr;
        for (std::size_t r = 0; r < mBookmarks.size(); ++r)
            if (mBookmarks[r].path == records[i].root) root = mRoots[r];
        if (!root) continue;
        rt::TreeNode* node = findNode(root, records[i].path, false);
        if (!node) continue;
        if (!mLoaded.contains(node)) fillChildren(node, records[i].path);
        node->setExpanded(records[i].expanded);
    }
    mTreeFollow = true;
}

rt::TreeNode* AssetsPanel::findNode(rt::TreeNode* root, const ct::String& path, bool expand)
{
    const ct::String* rootPath = mPathOf.find(root);
    if (!rootPath || !startsWithFolder(path, *rootPath)) return nullptr;
    const ct::String base = *rootPath;
    rt::TreeNode* node = root;
    ct::String current = base;
    std::size_t start = base.empty() ? 0 : base.size() + 1;
    while (current != path)
    {
        std::size_t end = path.find('/', start);
        if (end == ct::String::npos) end = path.size();
        const ct::String* nodePath = mPathOf.find(node);
        if (!nodePath) return nullptr;
        if (!mLoaded.contains(node)) fillChildren(node, *nodePath);
        if (expand) node->setExpanded(true);
        current = path.substr(0, end);
        rt::TreeNode* next = nullptr;
        for (std::size_t k = 0; k < node->children().size() && !next; ++k)
        {
            const ct::String* childPath = mPathOf.find(node->children()[k]);
            if (childPath && *childPath == current) next = node->children()[k];
        }
        if (!next) return nullptr;
        node = next;
        start = end + 1;
    }
    return node;
}

std::size_t AssetsPanel::bestRoot(const ct::String& path) const
{
    std::size_t best = mRoots.size();
    std::size_t bestLength = 0;
    for (std::size_t r = 0; r < mBookmarks.size() && r < mRoots.size(); ++r)
    {
        if (!startsWithFolder(path, mBookmarks[r].path)) continue;
        if (best == mRoots.size() || mBookmarks[r].path.size() > bestLength)
        {
            best = r;
            bestLength = mBookmarks[r].path.size();
        }
    }
    return best;
}

rt::TreeNode* AssetsPanel::expandTo(const ct::String& path)
{
    const std::size_t best = bestRoot(path);
    if (best == mRoots.size()) return nullptr;
    return findNode(mRoots[best], path, true);
}

void AssetsPanel::syncTree()
{
    mTreeFollow = false;
    if (!mTree || mRoots.empty()) return;
    rt::TreeNode* node = expandTo(mFolder);
    if (!node || mTree->selectedNode() == node) return;
    mSyncing = true;
    mTree->setSelectedNode(node);
    mSyncing = false;
}

void AssetsPanel::treeSelected(rt::TreeNode* node)
{
    if (mSyncing || !node) return;
    const ct::String* path = mPathOf.find(node);
    if (!path) return;
    const ct::String target = *path;
    navigateTo(target);
    mTreeFollow = false;
}

rt::TreeNode* AssetsPanel::treeNodeOf(const ct::String& path)
{
    const std::size_t best = bestRoot(path);
    if (best == mRoots.size()) return nullptr;
    return findNode(mRoots[best], path, false);
}

ct::String AssetsPanel::pathOfTreeNode(const rt::TreeNode* node) const
{
    for (auto it = mPathOf.begin(); it != mPathOf.end(); ++it)
        if (it->key == node) return it->value;
    return ct::String();
}

std::size_t AssetsPanel::bookmarkCount() const
{
    return mBookmarks.size();
}

ct::String AssetsPanel::bookmarkPath(std::size_t index) const
{
    return index < mBookmarks.size() ? mBookmarks[index].path : ct::String();
}

std::size_t AssetsPanel::thumbnailCount() const
{
    return mThumbnails.size();
}

bool AssetsPanel::loadThumbnail(const AssetEntry& entry, Thumbnail& out)
{
    TextureStore* store = TextureStore::singleton();
    if (!store) return false;
    const ct::String absolute = absoluteOf(entry.path);
    const ct::String name = ct::String("thumbnail:") + entry.path;
    const ct::String extension = AssetTypes::extensionOf(entry.name);

    TextureId texture;
    if (extension == "dds")
    {
        std::size_t size = 0;
        std::uint8_t* data = file_read(absolute.c_str(), &size);
        if (!data) return false;
        texture = store->createFromDds(data, size, false, false, name.c_str());
        fs_free(data);
        const TextureInfo* info = store->get(texture);
        if (info && info->height > 0)
            out.aspect = static_cast<float>(info->width) / static_cast<float>(info->height);
    }
    else
    {
        Pixmap pixmap;
        if (!pixmap.load(absolute.c_str())) return false;
        if (pixmap.width <= 0 || pixmap.height <= 0) return false;
        out.aspect = static_cast<float>(pixmap.width) / static_cast<float>(pixmap.height);
        const int longest = pixmap.width > pixmap.height ? pixmap.width : pixmap.height;
        const int limit = static_cast<int>(AssetGrid::kMaxThumbnail);
        if (longest > limit)
        {
            int width = pixmap.width * limit / longest;
            int height = pixmap.height * limit / longest;
            if (width < 1) width = 1;
            if (height < 1) height = 1;
            Pixmap* small = pixmap.resize(width, height);
            texture = store->createFromPixmap(*small, false, false, name.c_str(), 1);
            delete small;
        }
        else
            texture = store->createFromPixmap(pixmap, false, false, name.c_str(), 1);
    }
    if (!texture.valid()) return false;
    out.texture = texture;
    return true;
}

AssetThumbnail AssetsPanel::thumbnailFor(const AssetEntry& entry)
{
    AssetThumbnail result;
    if (entry.kind != AssetKind::Texture || !mProjectOpen) return result;
    const Thumbnail* cached = mThumbnails.find(entry.path);
    if (!cached)
    {
        if (mThumbnailBudget >= kThumbnailsPerFrame) return result;
        ++mThumbnailBudget;
        Thumbnail loaded;
        if (!loadThumbnail(entry, loaded)) loaded = Thumbnail();
        if (mThumbnailOrder.size() >= kThumbnailLimit)
        {
            const ct::String oldest = mThumbnailOrder.front();
            mThumbnailOrder.erase(mThumbnailOrder.begin());
            if (const Thumbnail* old = mThumbnails.find(oldest))
            {
                if (old->texture.valid()) mDoomed.push_back(old->texture);
                mThumbnails.erase(oldest);
            }
        }
        mThumbnails.put(entry.path, loaded);
        mThumbnailOrder.push_back(entry.path);
        cached = mThumbnails.find(entry.path);
    }
    if (!cached || !cached->texture.valid()) return result;
    result.texture = WidgetHost::textureHandle(cached->texture);
    result.aspect = cached->aspect;
    return result;
}

void AssetsPanel::clearThumbnails(bool immediate)
{
    TextureStore* store = TextureStore::singleton();
    for (auto it = mThumbnails.begin(); it != mThumbnails.end(); ++it)
    {
        if (!it->value.texture.valid()) continue;
        if (immediate && store) store->destroy(it->value.texture);
        else
            mDoomed.push_back(it->value.texture);
    }
    mThumbnails.clear();
    mThumbnailOrder.clear();
    if (immediate)
    {
        for (std::size_t i = 0; i < mDoomed.size(); ++i)
            if (store) store->destroy(mDoomed[i]);
        mDoomed.clear();
    }
}

bool AssetsPanel::validEntryName(const ct::String& name, ct::String& reason)
{
    if (name.empty())
    {
        reason = "Enter a name.";
        return false;
    }
    if (name == "." || name == ".." || name.contains('/') || name.contains('\\'))
    {
        reason = "Use one name, without / or \\.";
        return false;
    }
    for (std::size_t i = 0; i < name.size(); ++i)
        if (static_cast<unsigned char>(name[i]) < 32 || name[i] == ':' || name[i] == '*' ||
                name[i] == '?' || name[i] == '"' || name[i] == '<' || name[i] == '>' ||
                name[i] == '|')
        {
            reason = "The name has a character that a file name cannot have.";
            return false;
        }
    return true;
}

ct::String AssetsPanel::scriptFileName(const ct::String& name)
{
    return AssetTypes::extensionOf(name) == AssetTypes::kScriptExtension
                   ? name
                   : name + "." + AssetTypes::kScriptExtension;
}

ct::String AssetsPanel::scriptClassName(const ct::String& name)
{
    ct::String source = name;
    const std::size_t dot = source.rfind('.');
    if (dot != ct::String::npos) source = source.substr(0, dot);

    ct::String result;
    bool capitalize = true;
    for (std::size_t i = 0; i < source.size(); ++i)
    {
        const char c = source[i];
        const bool digit = c >= '0' && c <= '9';
        const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
        if (digit || letter || c == '_')
        {
            if (result.empty() && digit) result = "Script";
            char value = c;
            if (capitalize && c >= 'a' && c <= 'z') value = static_cast<char>(c - 'a' + 'A');
            result.push_back(value);
            capitalize = false;
        }
        else
            capitalize = true;
    }
    return result.empty() ? ct::String("NewScript") : result;
}

ct::String AssetsPanel::scriptTemplate(const ct::String& name)
{
    ct::String text = "class ";
    text += scriptClassName(name);
    text += "(Node3D):\n"
            "    def _ready(self):\n"
            "        pass\n"
            "\n"
            "    def _process(self, dt):\n"
            "        pass\n";
    return text;
}

ct::String AssetsPanel::uniqueName(const ct::String& parent, const ct::String& stem,
        const ct::String& suffix) const
{
    ct::String name = stem + suffix;
    for (int index = 2; index < 1000; ++index)
    {
        const ct::String path = absoluteOf(AssetImport::joinPath(parent, name));
        if (!file_exists(path.c_str()) && !dir_exists(path.c_str())) break;
        name = stem;
        name.append_number(index);
        name += suffix;
    }
    return name;
}

bool AssetsPanel::createFolder(const ct::String& parent, const ct::String& name, ct::String& error)
{
    error = ct::String();
    syncProject();
    if (!mProjectOpen)
    {
        error = kEmptyText;
        return false;
    }
    ct::String base;
    if (!normalizeFolder(parent, base) || !folderExists(base))
    {
        error = "The folder does not exist.";
        return false;
    }
    const ct::String trimmed = name.trimmed();
    if (!validEntryName(trimmed, error)) return false;
    const ct::String relative = AssetImport::joinPath(base, trimmed);
    const ct::String absolute = absoluteOf(relative);
    if (file_exists(absolute.c_str()) || dir_exists(absolute.c_str()))
    {
        error = "A folder or file with that name already exists.";
        return false;
    }
    if (!fs_create_directory(absolute.c_str()))
    {
        error = ct::String("Could not create folder ") + trimmed;
        return false;
    }
    invalidate(true);
    ensureListing();
    if (base == mFolder) select(indexOf(trimmed));
    return true;
}

bool AssetsPanel::createScript(const ct::String& parent, const ct::String& name, ct::String& error)
{
    error = ct::String();
    syncProject();
    if (!mProjectOpen)
    {
        error = kEmptyText;
        return false;
    }
    ct::String base;
    if (!normalizeFolder(parent, base) || !folderExists(base))
    {
        error = "The folder does not exist.";
        return false;
    }
    const ct::String trimmed = name.trimmed();
    if (!validEntryName(trimmed, error)) return false;
    const ct::String fileName = scriptFileName(trimmed);
    const ct::String relative = AssetImport::joinPath(base, fileName);
    const ct::String absolute = absoluteOf(relative);
    if (file_exists(absolute.c_str()) || dir_exists(absolute.c_str()))
    {
        error = fileName + " already exists.";
        return false;
    }
    if (!file_write_text(absolute.c_str(), scriptTemplate(trimmed).c_str()))
    {
        error = ct::String("Could not create script ") + fileName;
        return false;
    }
    invalidate(false);
    ensureListing();
    if (base == mFolder) select(indexOf(fileName));
    return true;
}

void AssetsPanel::requestFolder(const ct::String& parent)
{
    if (!mProjectOpen || nameBox()) return;
    ct::String base;
    if (!normalizeFolder(parent, base) || !folderExists(base)) base = mFolder;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::InputBox* box = app.addFloat<rt::InputBox>("New Folder",
            ct::String("Folder name in ") + displayPath(base) + ":");
    box->setText(uniqueName(base, "New Folder", ct::String()));
    box->setAcceptLabel("Create");
    const ct::Rc<Link> link = mLink;
    box->setValidator(
            [link, base](const ct::String& text)
            {
                if (!link->owner) return ct::String();
                const ct::String name = text.trimmed();
                ct::String reason;
                if (!validEntryName(name, reason)) return reason;
                const ct::String path = link->owner->absoluteOf(AssetImport::joinPath(base, name));
                if (file_exists(path.c_str()) || dir_exists(path.c_str()))
                    return ct::String("A folder or file with that name already exists.");
                return ct::String();
            });
    box->accepted.connect(
            [link, base](const ct::String& text)
            {
                if (!link->owner) return;
                ct::String error;
                if (!link->owner->createFolder(base, text, error)) link->owner->report(error, true);
                else
                    link->owner->report(ct::String("Created folder ") + text.trimmed(), false);
            });
}

void AssetsPanel::requestScript(const ct::String& parent)
{
    if (!mProjectOpen || nameBox()) return;
    ct::String base;
    if (!normalizeFolder(parent, base) || !folderExists(base)) base = mFolder;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::InputBox* box = app.addFloat<rt::InputBox>("New Script",
            ct::String("Script name in ") + displayPath(base) + ":");
    box->setText(uniqueName(base, "NewScript", ct::String(".") + AssetTypes::kScriptExtension));
    box->setAcceptLabel("Create");
    const ct::Rc<Link> link = mLink;
    box->setValidator(
            [link, base](const ct::String& text)
            {
                if (!link->owner) return ct::String();
                const ct::String name = text.trimmed();
                ct::String reason;
                if (!validEntryName(name, reason)) return reason;
                const ct::String path =
                        link->owner->absoluteOf(AssetImport::joinPath(base, scriptFileName(name)));
                if (file_exists(path.c_str()) || dir_exists(path.c_str()))
                    return scriptFileName(name) + " already exists.";
                return ct::String();
            });
    box->accepted.connect(
            [link, base](const ct::String& text)
            {
                if (!link->owner) return;
                ct::String error;
                if (!link->owner->createScript(base, text, error)) link->owner->report(error, true);
                else
                    link->owner->report(
                            ct::String("Created script ") + scriptFileName(text.trimmed()), false);
            });
}

bool AssetsPanel::copyPath(const ct::String& path)
{
    if (path.empty()) return false;
    mLastCopied = path;
    if (Device* device = Device::singleton()) device->clipboardSet(path.c_str());
    return true;
}

void AssetsPanel::convertToPMesh(const ct::String& path)
{
    AssetLoader* loader = AssetLoader::singleton();
    MaterialStore* materials = Main::materials();
    if (!loader || !materials) return;
    const ct::String target = AssetImport::stemOf(path) + "." + pmesh::kMeshExtension;
    if (file_exists(absoluteOf(target).c_str()))
    {
        report(AssetImport::nameOf(target) + " already exists", true);
        return;
    }
    ct::String error;
    if (!convertModelToPMesh(*loader, *materials, path, absoluteOf(target), PMeshConvertOptions(),
                error))
    {
        report(ct::String("Could not convert ") + AssetImport::nameOf(path) + ": " + error, true);
        return;
    }
    report(ct::String("Converted to ") + AssetImport::nameOf(target), false);
    invalidate(true);
    ensureListing();
}

const ct::String& AssetsPanel::lastCopied() const
{
    return mLastCopied;
}

void AssetsPanel::removeMissingScenes()
{
    Project& project = mDocument.project();
    if (!project.isOpen()) return;
    const ct::Vector<ct::String> scenes = project.scenes();
    bool changed = false;
    for (std::size_t i = 0; i < scenes.size(); ++i)
    {
        if (file_exists(project.absolutePath(scenes[i]).c_str())) continue;
        if (project.removeScene(scenes[i])) changed = true;
    }
    if (changed) project.save();
}

bool AssetsPanel::renamePath(const ct::String& path, const ct::String& name, ct::String& error)
{
    error = ct::String();
    syncProject();
    if (!mProjectOpen)
    {
        error = kEmptyText;
        return false;
    }
    ct::String from;
    if (!normalizeFolder(path, from) || from.empty() || protectedPath(from))
    {
        error = "This folder cannot be renamed.";
        return false;
    }
    const ct::String absolute = absoluteOf(from);
    PathInfo info;
    if (!fs_get_path_info(absolute.c_str(), &info))
    {
        error = from + " does not exist.";
        return false;
    }
    const ct::String trimmed = name.trimmed();
    if (!validEntryName(trimmed, error)) return false;
    const ct::String& scene = mDocument.scenePath();
    if (!scene.empty() && startsWithFolder(scene, from))
    {
        error = "The open scene cannot be renamed.";
        return false;
    }
    const ct::String to = AssetImport::joinPath(parentFolderOf(from), trimmed);
    if (to == from) return true;
    const ct::String target = absoluteOf(to);
    if (file_exists(target.c_str()) || dir_exists(target.c_str()))
    {
        error = "A folder or file with that name already exists.";
        return false;
    }

    ct::Vector<ct::String> companionFrom;
    ct::Vector<ct::String> companionTo;
    if (info.type == PATH_TYPE_FILE &&
            AssetTypes::classify(AssetImport::nameOf(from), false) == AssetKind::Model)
    {
        const ct::String source = absoluteOf(AssetImport::stemOf(from) + ".mat");
        const ct::String destination = absoluteOf(AssetImport::stemOf(to) + ".mat");
        if (file_exists(source.c_str()) && !file_exists(destination.c_str()))
        {
            companionFrom.push_back(source);
            companionTo.push_back(destination);
        }
    }

    if (!fs_rename_path(absolute.c_str(), target.c_str()))
    {
        error = ct::String("Could not rename ") + AssetImport::nameOf(from);
        return false;
    }
    for (std::size_t i = 0; i < companionFrom.size(); ++i)
        fs_rename_path(companionFrom[i].c_str(), companionTo[i].c_str());

    Project& project = mDocument.project();
    const ct::Vector<ct::String> scenes = project.scenes();
    bool changed = false;
    for (std::size_t i = 0; i < scenes.size(); ++i)
    {
        if (!startsWithFolder(scenes[i], from)) continue;
        if (project.renameScene(scenes[i], to + scenes[i].substr(from.size()))) changed = true;
    }
    if (changed) project.save();

    clearThumbnails(false);
    invalidate(true);
    ensureListing();
    if (parentFolderOf(to) == mFolder) select(indexOf(trimmed));
    return true;
}

void AssetsPanel::requestRename(const ct::String& path)
{
    if (!mProjectOpen || nameBox()) return;
    ct::String from;
    if (!normalizeFolder(path, from) || from.empty() || protectedPath(from))
    {
        report("This folder cannot be renamed.", true);
        return;
    }
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::InputBox* box = app.addFloat<rt::InputBox>("Rename",
            ct::String("New name for ") + AssetImport::nameOf(from) + ":");
    box->setText(AssetImport::nameOf(from));
    box->setAcceptLabel("Rename");
    const ct::Rc<Link> link = mLink;
    box->setValidator(
            [link, from](const ct::String& text)
            {
                if (!link->owner) return ct::String();
                const ct::String name = text.trimmed();
                ct::String reason;
                if (!validEntryName(name, reason)) return reason;
                if (name == AssetImport::nameOf(from)) return ct::String();
                const ct::String path =
                        link->owner->absoluteOf(AssetImport::joinPath(parentFolderOf(from), name));
                if (file_exists(path.c_str()) || dir_exists(path.c_str()))
                    return ct::String("A folder or file with that name already exists.");
                return ct::String();
            });
    box->accepted.connect(
            [link, from](const ct::String& text)
            {
                if (!link->owner) return;
                ct::String error;
                if (!link->owner->renamePath(from, text, error)) link->owner->report(error, true);
                else
                    link->owner->report(ct::String("Renamed to ") + text.trimmed(), false);
            });
}

void AssetsPanel::requestDelete(const ct::String& path)
{
    if (!mProjectOpen || confirmDialog()) return;
    ct::String normalized;
    if (!normalizeFolder(path, normalized) || normalized.empty() || protectedPath(normalized))
    {
        report("This folder cannot be deleted.", true);
        return;
    }
    const ct::String absolute = absoluteOf(normalized);
    PathInfo info;
    if (!fs_get_path_info(absolute.c_str(), &info))
    {
        report(ct::String(normalized) + " does not exist.", true);
        return;
    }

    ct::String message = "Delete ";
    message += normalized;
    message += "?\n\n";
    if (info.type == PATH_TYPE_DIRECTORY)
    {
        CollectedPaths inside;
        fs_enumerate_directory(absolute.c_str(), true, &collectPath, &inside);
        message += "This deletes the folder and ";
        message.append_number(static_cast<unsigned long long>(inside.paths.size()));
        message += inside.paths.size() == 1 ? " item" : " items";
        message += " inside it from disk. ";
    }
    else
        message += "This deletes the file from disk. ";
    message += "The editor's undo cannot bring it back, and any scene still pointing at it "
               "will fail to load it.";

    if (info.type == PATH_TYPE_FILE &&
            AssetTypes::classify(AssetImport::nameOf(normalized), false) == AssetKind::Model)
    {
        ct::Vector<ct::String> extras;
        modelCompanions(normalized, extras);
        if (!extras.empty())
        {
            message += "\n\nAlso deletes:";
            for (std::size_t i = 0; i < extras.size() && i < kDeleteListLimit; ++i)
            {
                message += "\n  ";
                message += extras[i];
            }
        }
    }

    rt::Dialog* dialog = new rt::Dialog("Delete Asset", message);
    const ct::Rc<Link> link = mLink;
    dialog->addButton("Delete", rt::Dialog::Role::Danger,
            [link, normalized]
            {
                if (!link->owner) return;
                ct::String error;
                if (!link->owner->deletePath(normalized, error)) link->owner->report(error, true);
            });
    dialog->addButton("Cancel", rt::Dialog::Role::Cancel);
    dialog->show();
}

bool AssetsPanel::deletePath(const ct::String& path, ct::String& error)
{
    error = ct::String();
    syncProject();
    if (!mProjectOpen)
    {
        error = kEmptyText;
        return false;
    }
    ct::String normalized;
    if (!normalizeFolder(path, normalized) || normalized.empty() || protectedPath(normalized))
    {
        error = "This folder cannot be deleted.";
        return false;
    }
    const ct::String absolute = absoluteOf(normalized);
    PathInfo info;
    if (!fs_get_path_info(absolute.c_str(), &info))
    {
        error = normalized + " does not exist.";
        return false;
    }
    const ct::String& scene = mDocument.scenePath();
    if (!scene.empty() && startsWithFolder(scene, normalized))
    {
        error = "The open scene cannot be deleted.";
        return false;
    }

    ct::Vector<ct::String> doomed;
    if (info.type == PATH_TYPE_DIRECTORY)
    {
        CollectedPaths inside;
        fs_enumerate_directory(absolute.c_str(), true, &collectPath, &inside);
        ct::sort(inside.paths.begin(), inside.paths.end(), &longerPath);
        for (std::size_t i = 0; i < inside.paths.size(); ++i) doomed.push_back(inside.paths[i]);
    }
    doomed.push_back(absolute);
    if (info.type == PATH_TYPE_FILE &&
            AssetTypes::classify(AssetImport::nameOf(normalized), false) == AssetKind::Model)
    {
        ct::Vector<ct::String> extras;
        modelCompanions(normalized, extras);
        for (std::size_t i = 0; i < extras.size(); ++i) doomed.push_back(absoluteOf(extras[i]));
    }

    std::size_t removed = 0;
    for (std::size_t i = 0; i < doomed.size(); ++i)
        if (fs_remove_path(doomed[i].c_str())) ++removed;
    if (file_exists(absolute.c_str()) || dir_exists(absolute.c_str()))
    {
        error = ct::String("Could not delete ") + AssetImport::nameOf(normalized);
        invalidate(true);
        return false;
    }

    removeMissingScenes();
    clearThumbnails(false);
    invalidate(true);
    ensureListing();

    ct::String message = "Deleted ";
    message += AssetImport::nameOf(normalized);
    if (removed > 1)
    {
        message += " and ";
        message.append_number(static_cast<unsigned long long>(removed - 1));
        message += " more";
    }
    report(message, false);
    return true;
}

template<typename T>
T* AssetsPanel::floatOfType() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (T* found = rt::widget_cast<T>(floats[i - 1])) return found;
    return nullptr;
}

rt::Dialog* AssetsPanel::confirmDialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return app ? rt::widget_cast<rt::Dialog>(app->popup()) : nullptr;
}

rt::FileDialog* AssetsPanel::fileDialog() const
{
    return floatOfType<rt::FileDialog>();
}

rt::InputBox* AssetsPanel::nameBox() const
{
    return floatOfType<rt::InputBox>();
}

ImportDialog* AssetsPanel::importDialog() const
{
    return floatOfType<ImportDialog>();
}

void AssetsPanel::requestImport(const ct::String& folder)
{
    if (!mProjectOpen || fileDialog() || mPlanActive) return;
    ct::String target;
    if (!normalizeFolder(folder, target) || !folderExists(target)) target = mFolder;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::FileDialog* dialog = app.addFloat<rt::FileDialog>("Import files");
    dialog->setMode(rt::FileDialog::Mode::Open);
    EditorSettings* settings = &mDocument.settings();
    FileDialogMemory::prepare(*dialog, settings, EditorSettings::kFolderImport,
            &mDocument.project());
    const ct::Rc<Link> link = mLink;
    dialog->accepted.connect(
            [link, target, settings](const ct::String& path)
            {
                if (!link->owner) return;
                FileDialogMemory::remember(settings, EditorSettings::kFolderImport, path);
                ct::Vector<ct::String> sources;
                sources.push_back(path);
                link->owner->importFiles(sources, target);
            });
}

void AssetsPanel::filesDropped(const ct::Vector<ct::String>& paths)
{
    importFiles(paths, mFolder);
}

bool AssetsPanel::importing() const
{
    return mPlanActive;
}

void AssetsPanel::importFiles(const ct::Vector<ct::String>& sources, const ct::String& folder)
{
    syncProject();
    if (!mProjectOpen)
    {
        report(kEmptyText, true);
        return;
    }
    if (mPlanActive)
    {
        report("Another import is waiting for an answer.", true);
        return;
    }
    ct::String target;
    if (!normalizeFolder(folder, target) || !folderExists(target))
    {
        report("The destination folder does not exist.", true);
        return;
    }

    ImportPlan plan;
    AssetImport::plan(mDocument.project(), sources, target, plan);
    for (std::size_t i = 0; i < plan.skipped.size(); ++i)
        rt::Toast::show(plan.skipped[i], rt::Toast::Type::Warning, 5.0f);
    if (plan.empty())
    {
        if (plan.skipped.empty()) report("Nothing to import.", true);
        return;
    }

    mPlan = plan;
    mPlanFolder = target;
    mPlanActive = true;
    if (plan.clashCount() > 0) showReplaceDialog();
    else
        showImportOptions();
}

void AssetsPanel::showReplaceDialog()
{
    ct::String message;
    message.append_number(static_cast<unsigned long long>(mPlan.clashCount()));
    message += mPlan.clashCount() == 1 ? " file already exists in " : " files already exist in ";
    message += displayPath(mPlanFolder);
    message += ":\n";
    std::size_t listed = 0;
    for (std::size_t i = 0; i < mPlan.files.size(); ++i)
    {
        if (!mPlan.files[i].exists) continue;
        if (listed == kDeleteListLimit)
        {
            message += "\n  ...";
            break;
        }
        message += "\n  ";
        message += mPlan.files[i].relative;
        ++listed;
    }
    message += "\n\nReplace them?";

    rt::Dialog* dialog = new rt::Dialog("Replace Files", message);
    const ct::Rc<Link> link = mLink;
    dialog->addButton("Replace", rt::Dialog::Role::Danger,
            [link]
            {
                if (link->owner) link->owner->showImportOptions();
            });
    dialog->addButton("Cancel", rt::Dialog::Role::Cancel,
            [link]
            {
                if (link->owner) link->owner->cancelImport();
            });
    dialog->show();
}

void AssetsPanel::showImportOptions()
{
    const int model = mPlan.firstModel();
    if (model < 0)
    {
        ImportOptions none;
        none.instantiate = false;
        finishImport(none);
        return;
    }
    ct::String summary = mPlan.files[static_cast<std::size_t>(model)].relative;
    if (mPlan.files.size() > 1)
    {
        summary += " (+";
        summary.append_number(static_cast<unsigned long long>(mPlan.files.size() - 1));
        summary += mPlan.files.size() == 2 ? " file)" : " files)";
    }
    rt::WidgetApp& app = rt::WidgetApp::instance();
    ImportDialog* dialog = app.addFloat<ImportDialog>(summary);
    const ct::Rc<Link> link = mLink;
    dialog->importRequested.connect(
            [link](const ImportOptions& options)
            {
                if (link->owner) link->owner->finishImport(options);
            });
    dialog->dismissed.connect(
            [link]
            {
                if (link->owner) link->owner->cancelImport();
            });
}

void AssetsPanel::cancelImport()
{
    mPlanActive = false;
    mPlan = ImportPlan();
}

void AssetsPanel::finishImport(const ImportOptions& options)
{
    if (!mPlanActive) return;
    const ImportPlan plan = mPlan;
    const ct::String folder = mPlanFolder;
    mPlanActive = false;
    mPlan = ImportPlan();

    ct::Vector<ct::String> failed;
    const std::size_t copied = AssetImport::copy(plan, true, failed);
    invalidate(true);
    for (std::size_t i = 0; i < failed.size(); ++i)
        report(ct::String("Could not import ") + failed[i], true);
    if (copied == 0) return;

    ct::String message = "Imported ";
    message.append_number(static_cast<unsigned long long>(copied));
    message += copied == 1 ? " file into " : " files into ";
    message += displayPath(folder);
    report(message, false);

    ensureListing();
    const int first = plan.firstModel() >= 0 ? plan.firstModel() : 0;
    const ct::String primary = plan.files[static_cast<std::size_t>(first)].relative;
    if (folder == mFolder) select(indexOf(AssetImport::nameOf(primary)));

    const int model = plan.firstModel();
    if (options.instantiate && model >= 0)
    {
        const ct::String relative = plan.files[static_cast<std::size_t>(model)].relative;
        if (file_exists(absoluteOf(relative).c_str()))
            instantiate(relative, nullptr, nullptr, &options);
    }
}

Node* AssetsPanel::editableScene() const
{
    return mDocument.scene();
}

bool AssetsPanel::usableParent(const Node* node) const
{
    const Node* scene = editableScene();
    if (!scene || !node) return false;
    return node == scene || scene->isAncestorOf(node);
}

Node* AssetsPanel::defaultParent() const
{
    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    for (std::size_t i = selected.size(); i > 0; --i)
        if (usableParent(selected[i - 1]) && !selected[i - 1]->isInternal()) return selected[i - 1];
    return editableScene();
}

Node* AssetsPanel::buildPrefab(const ct::String& path, Node* parent, ct::String& error)
{
    SceneResult result;
    Node* node = Prefab::instantiate(path, parent, result);
    if (!node)
    {
        error = result.describe();
        if (error.empty()) error = ct::String("Could not instantiate ") + path;
    }
    return node;
}

Node* AssetsPanel::buildModel(const ct::String& path, Node* parent, ct::String& error)
{
    const ct::String file = FileSystem::simplifyPath(path);
    if (!FileSystem::getSingleton().exists(file))
    {
        error = ct::String("The model ") + file + " was not found.";
        return nullptr;
    }
    sceneres::clearError();
    const MeshId first = sceneres::loadMesh(file);
    if (!first.valid())
    {
        error = sceneres::takeError();
        if (error.empty()) error = ct::String("Could not load ") + file;
        return nullptr;
    }
    AssetLoader* loader = AssetLoader::singleton();
    const ModelId id = loader ? loader->findModelByPath(sceneres::normalizePath(file)) : ModelId();
    Node3D* root = id.valid() ? instantiateModel(parent, id) : nullptr;
    if (!root)
    {
        error = ct::String("Could not build the nodes of ") + file;
        return nullptr;
    }
    const ct::String stem = AssetImport::stemOf(AssetImport::nameOf(file));
    if (!stem.empty()) root->setName(stem);
    return root;
}

Node* AssetsPanel::instantiate(const ct::String& path, Node* parent,
        const Math::Vec3* worldPosition, const ImportOptions* options)
{
    syncProject();
    if (!mProjectOpen || !editableScene())
    {
        report("Open a scene to instantiate assets.", true);
        return nullptr;
    }
    ct::String relative;
    if (!mDocument.project().relativePath(path, relative))
    {
        report("Only files inside the project can be instantiated.", true);
        return nullptr;
    }
    const AssetKind kind = AssetTypes::classify(AssetImport::nameOf(relative), false);
    if (kind != AssetKind::Model && kind != AssetKind::Prefab) return nullptr;

    if (!parent) parent = defaultParent();
    if (!usableParent(parent))
    {
        report("The node is not part of the open scene.", true);
        return nullptr;
    }

    const ct::Vector<Node*> selectionBefore = mSelection.selectedNodes();
    EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
    mActions.takeEditable();
    mActions.allowChildren(parent);
    ct::String error;
    Node* node = kind == AssetKind::Prefab ? buildPrefab(relative, parent, error)
                                           : buildModel(relative, parent, error);
    if (!node)
    {
        report(error, true);
        return nullptr;
    }

    if (Node3D* spatial = node->castTo<Node3D>())
    {
        if (options)
        {
            if (kind == AssetKind::Model) spatial->setRotationDegrees(options->rotate);
            if (kind == AssetKind::Model) spatial->setScale(Math::Vec3(options->scale));
            spatial->setPosition(spatial->getPosition() + options->translate);
        }
        if (worldPosition) spatial->setGlobalPosition(*worldPosition);
    }

    mSelection.clear();
    mSelection.add(node);
    ct::Vector<Node*> created;
    created.push_back(node);
    mActions.recordCreated(kInstantiateAction, created, selectionBefore);
    rt::Toast::show(ct::String("Instantiated ") + AssetImport::nameOf(relative),
            rt::Toast::Type::Success);
    return node;
}

Node* AssetsPanel::instantiateAt(const ct::String& path, const Math::Vec3& worldPosition)
{
    return instantiate(path, editableScene(), &worldPosition, nullptr);
}

Node* AssetsPanel::instantiateOn(const ct::String& path, Node* parent)
{
    return instantiate(path, parent, nullptr, nullptr);
}

void AssetsPanel::dropViewport(const ct::String& path, const Math::Vec3& position)
{
    instantiateAt(path, position);
}

Node* AssetsPanel::dropTarget(float x, float y)
{
    rt::Outliner* outliner = mHierarchy.outliner();
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!outliner || !app || app->popup()) return nullptr;
    for (const rt::Widget* widget = outliner; widget; widget = widget->parent())
        if (!widget->isVisible()) return nullptr;
    if (!outliner->absoluteRect().contains(x, y)) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = 0; i < floats.size(); ++i)
        if (floats[i]->isVisible() && floats[i]->absoluteRect().contains(x, y)) return nullptr;
    Node* row = mHierarchy.nodeAt(x, y);
    return row ? row : editableScene();
}

Node* AssetsPanel::materialRowAt(float x, float y)
{
    return dropTarget(x, y) ? mHierarchy.nodeAt(x, y) : nullptr;
}

bool AssetsPanel::attachScript(const ct::String& path, Node* node)
{
    if (!node || !usableParent(node))
    {
        report("Drop the script on a node of the scene.", true);
        return false;
    }
    PropertyValue before;
    if (!NodeClassDB::getProperty(node, "script", before))
    {
        report(ct::String("Could not attach ") + path + " to " + node->getName(), true);
        return false;
    }
    EditorTreeEdits::setProperty(&mDocument.undoRedo(), kAttachScriptAction, node, "script", before,
            PropertyValue::fromResourcePath(path));
    mSelection.clear();
    mSelection.add(node);
    rt::Toast::show(ct::String("Attached ") + AssetImport::nameOf(path) + " to " + node->getName(),
            rt::Toast::Type::Success);
    return true;
}

void AssetsPanel::gridDragFinished(const ct::String& path, bool accepted)
{
    if (accepted) return;
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return;
    if (AssetTypes::classify(AssetTypes::filePart(path), false) == AssetKind::Script)
    {
        if (!dropTarget(app->mouseX(), app->mouseY())) return;
        Node* row = mHierarchy.nodeAt(app->mouseX(), app->mouseY());
        if (row) attachScript(path, row);
        return;
    }
    Node* target = dropTarget(app->mouseX(), app->mouseY());
    if (!target) return;
    if (AssetTypes::classify(AssetTypes::filePart(path), false) == AssetKind::Material)
    {
        if (mMaterials)
            mMaterials->dropMaterial(path, mHierarchy.nodeAt(app->mouseX(), app->mouseY()));
        return;
    }
    instantiateOn(path, target);
}

void AssetsPanel::showMenu(int index, float x, float y)
{
    if (!mProjectOpen || !mMenu) return;
    ensureListing();
    const bool onEntry = index >= 0 && index < static_cast<int>(mShown.size());
    const AssetEntry entry = onEntry ? mShown[static_cast<std::size_t>(index)] : AssetEntry();
    const ct::String target = onEntry && entry.directory ? entry.path : mFolder;

    mMenu->clearActions();
    const ct::Rc<Link> link = mLink;
    rt::MenuAction* header = mMenu->addAction(ct::String("In ") + displayPath(target));
    header->setEnabled(false);
    mMenu->addSeparator();

    if (onEntry && (entry.kind == AssetKind::Folder || entry.kind == AssetKind::Scene ||
                           entry.kind == AssetKind::Prefab || entry.kind == AssetKind::Model ||
                           entry.kind == AssetKind::Material || entry.kind == AssetKind::Script))
    {
        const ct::String path = entry.path;
        switch (entry.kind)
        {
            case AssetKind::Folder:
                mMenu->addAction("Open",
                        [link, path]
                        {
                            if (link->owner) link->owner->navigateTo(path);
                        });
                break;
            case AssetKind::Scene:
                mMenu->addAction("Open Scene",
                        [link, path]
                        {
                            if (link->owner) link->owner->openScene(path);
                        });
                break;
            case AssetKind::Prefab:
            case AssetKind::Model:
                mMenu->addAction("Instantiate",
                        [link, path]
                        {
                            if (link->owner) link->owner->instantiateOn(path, nullptr);
                        });
                if (entry.kind == AssetKind::Model &&
                        AssetTypes::extensionOf(entry.name) != pmesh::kMeshExtension)
                    mMenu->addAction("Convert to .pmesh",
                            [link, path]
                            {
                                if (link->owner) link->owner->convertToPMesh(path);
                            });
                break;
            case AssetKind::Material:
                mMenu->addAction("Edit",
                        [link, path]
                        {
                            if (link->owner) link->owner->openMaterial(path);
                        });
                break;
            case AssetKind::Script:
                mMenu->addAction("Edit Script",
                        [link, path]
                        {
                            if (link->owner) link->owner->openScript(path);
                        });
                break;
            default:
                break;
        }
        mMenu->addSeparator();
    }

    mMenu->addAction("New Folder...",
            [link, target]
            {
                if (link->owner) link->owner->requestFolder(target);
            });
    mMenu->addAction("New Script...",
            [link, target]
            {
                if (link->owner) link->owner->requestScript(target);
            });
    rt::MenuAction* material = mMenu->addAction("New Material...",
            [link, target]
            {
                if (link->owner) link->owner->requestMaterial(target);
            });
    material->setEnabled(mMaterials != nullptr);
    mMenu->addAction("Import...",
            [link, target]
            {
                if (link->owner) link->owner->requestImport(target);
            });
    mMenu->addSeparator();
    rt::MenuAction* copy = mMenu->addAction("Copy Path",
            [link, target, entry, onEntry]
            {
                if (!link->owner) return;
                link->owner->copyPath(onEntry ? entry.path : target);
            });
    copy->setEnabled(onEntry || !target.empty());
    mMenu->addAction("Refresh",
            [link]
            {
                if (link->owner) link->owner->refresh();
            });
    if (onEntry)
    {
        mMenu->addSeparator();
        const ct::String path = entry.path;
        rt::MenuAction* rename = mMenu->addAction("Rename...",
                [link, path]
                {
                    if (link->owner) link->owner->requestRename(path);
                });
        rename->setEnabled(!protectedPath(path));
        rt::MenuAction* remove = mMenu->addAction("Delete...",
                [link, path]
                {
                    if (link->owner) link->owner->requestDelete(path);
                });
        remove->setEnabled(!protectedPath(path));
    }
    mMenu->exec(x, y);
}

void AssetsPanel::clickedButton(int id)
{
    for (int i = 0; i < kButtonCount; ++i)
    {
        if (mButtons[i] != id) continue;
        switch (static_cast<Button>(i))
        {
            case kHome:
                goHome();
                break;
            case kBack:
                goBack();
                break;
            case kForward:
                goForward();
                break;
            case kUp:
                goUp();
                break;
            case kNewFolder:
                requestFolder(mFolder);
                break;
            case kImport:
                requestImport(mFolder);
                break;
            case kRefresh:
                refresh();
                break;
            default:
                break;
        }
        return;
    }
}

void AssetsPanel::toggledButton(int id, bool checked)
{
    if (mSyncing) return;
    for (int i = kGridView; i < kButtonCount; ++i)
    {
        if (mButtons[i] != id) continue;
        const AssetViewMode mode = static_cast<AssetViewMode>(i - kGridView);
        if (checked) setViewMode(mode);
        else if (mode == mViewMode)
            setViewMode(mode);
        return;
    }
}

rt::ToolBar* AssetsPanel::toolBar() const
{
    return mToolBar;
}

rt::Breadcrumbs* AssetsPanel::breadcrumbs() const
{
    return mBreadcrumbs;
}

rt::TextInput* AssetsPanel::searchInput() const
{
    return mSearch;
}

rt::Slider* AssetsPanel::zoomSlider() const
{
    return mZoom;
}

rt::Splitter* AssetsPanel::splitter() const
{
    return mBody;
}

rt::TreeView* AssetsPanel::tree() const
{
    return mTree;
}

AssetGrid* AssetsPanel::grid() const
{
    return mGrid;
}

rt::Label* AssetsPanel::footer() const
{
    return mFooter;
}

rt::Menu* AssetsPanel::contextMenu() const
{
    return mMenu;
}

rt::Widget* AssetsPanel::emptyState() const
{
    return mEmpty;
}

rt::Button* AssetsPanel::newProjectButton() const
{
    return mNewProject;
}

rt::Button* AssetsPanel::openProjectButton() const
{
    return mOpenProject;
}

rt::Widget* AssetsPanel::root() const
{
    return mColumn;
}

} // namespace prisma3d
