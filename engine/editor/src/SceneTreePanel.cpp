#include "prisma3d/editor/SceneTreePanel.h"

#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kRowHeight = 22.0f;
const float kMinWidth = 48.0f;
const float kToolBarButtonSize = 28.0f;
const float kToolBarIconPadding = 6.0f;

std::uint64_t hashStep(std::uint64_t in, std::uint64_t previous)
{
    return ((previous << 5) + previous) ^ in;
}

void hashNodes(const Node* node, std::uint64_t& hash)
{
    hash = hashStep(static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(node)), hash);
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (!node->getChild(i)->isInternal()) hashNodes(node->getChild(i), hash);
}

bool findRow(const rt::TreeNode* node, const rt::TreeNode* wanted, int& row)
{
    if (node == wanted) return true;
    ++row;
    if (!node->isExpanded()) return false;
    for (const rt::TreeNode* child: node->children())
        if (findRow(child, wanted, row)) return true;
    return false;
}

ct::String labelOf(const Node* node)
{
    return node->getName().empty() ? ct::String(node->getClassName()) : node->getName();
}

} // namespace

SceneTreePanel::SceneTreePanel(SceneTree& tree, EditorSelection& selection)
        : mTree(tree),
          mSelection(selection),
          mColumn(nullptr),
          mToolBar(nullptr),
          mOutliner(nullptr),
          mLink(ct::make_rc<Link>(Link{ this })),
          mScene(nullptr),
          mListener(nullptr),
          mLastHash(0),
          mRebuilds(0),
          mDirty(true),
          mRevealSelection(false),
          mPendingTest(false),
          mSyncing(false)
{
    mTree.addListener(this);
    mSelection.addListener(this);
}

SceneTreePanel::~SceneTreePanel()
{
    mSelection.removeListener(this);
    mTree.removeListener(this);
    mLink->owner = nullptr;
}

bool SceneTreePanel::attach(rt::Widget* parent)
{
    if (mOutliner || !parent) return false;
    mColumn = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mColumn->setPadding(0.0f);
    mColumn->setSpacing(0.0f);
    mToolBar = mColumn->createChild<rt::ToolBar>();
    mToolBar->setButtonSize(kToolBarButtonSize);
    mToolBar->setIconPadding(kToolBarIconPadding);
    mOutliner = mColumn->createChild<rt::Outliner>();
    mOutliner->setStretch(1.0f);
    const ct::Rc<Link> link = mLink;
    mOutliner->selectionChanged.connect(
            [link](rt::TreeNode* item)
            {
                if (link->owner) link->owner->itemSelected(item);
            });
    mOutliner->visibilityChanged.connect(
            [link](rt::TreeNode* item, bool)
            {
                if (link->owner) link->owner->visibilityToggled(item);
            });
    mOutliner->nodeActivated.connect(
            [link](rt::TreeNode* item)
            {
                if (link->owner) link->owner->itemActivated(item);
            });
    mDirty = true;
    return true;
}

void SceneTreePanel::itemActivated(rt::TreeNode* item)
{
    if (Node* node = nodeOf(item)) nodeActivated.emit(node);
}

bool SceneTreePanel::attached() const
{
    return mOutliner != nullptr;
}

rt::Outliner* SceneTreePanel::outliner() const
{
    return mOutliner;
}

rt::ToolBar* SceneTreePanel::toolBar() const
{
    return mToolBar;
}

ClassIcons& SceneTreePanel::icons()
{
    return mIcons;
}

const ClassIcons& SceneTreePanel::icons() const
{
    return mIcons;
}

void SceneTreePanel::setEditedScene(Node* scene)
{
    mScene = scene;
    mExpanded.clear();
    mDirty = true;
}

Node* SceneTreePanel::editedScene() const
{
    return mScene;
}

void SceneTreePanel::setChangeListener(EditorChangeListener* listener)
{
    mListener = listener;
}

bool SceneTreePanel::dirty() const
{
    return mDirty;
}

unsigned int SceneTreePanel::rebuildCount() const
{
    return mRebuilds;
}

rt::TreeNode* SceneTreePanel::itemOf(const Node* node) const
{
    rt::TreeNode* const* item = mItemOf.find(const_cast<Node*>(node));
    return item ? *item : nullptr;
}

Node* SceneTreePanel::nodeOf(rt::TreeNode* item) const
{
    Node* const* node = mNodeOf.find(item);
    return node ? *node : nullptr;
}

bool SceneTreePanel::selectRowAt(float x, float y)
{
    if (!mOutliner || mDirty || !mOutliner->absoluteRect().contains(x, y)) return false;
    rt::MouseEvent press;
    press.localX = 0.0f;
    press.localY = y - mOutliner->absoluteRect().y;
    mOutliner->onMousePress(press);
    return press.consumed;
}

Node* SceneTreePanel::nodeAt(float x, float y)
{
    if (!mOutliner || mDirty || !mOutliner->absoluteRect().contains(x, y)) return nullptr;
    rt::TreeNode* hit = nullptr;
    bool locked = false;
    const auto slot = mOutliner->lockChanged.connect(
            [&hit, &locked](rt::TreeNode* item, bool state)
            {
                hit = item;
                locked = state;
            });
    rt::MouseEvent press;
    press.localX = mOutliner->rect().w - 12.0f;
    press.localY = y - mOutliner->absoluteRect().y;
    mOutliner->onMousePress(press);
    mOutliner->lockChanged.disconnect(slot);
    if (!hit) return nullptr;
    mOutliner->setNodeLocked(hit, !locked);
    return nodeOf(hit);
}

bool SceneTreePanel::inScene(const Node* node) const
{
    return mScene && (node == mScene || mScene->isAncestorOf(node));
}

void SceneTreePanel::treeChanged()
{
    if (mPendingTest || mDirty) return;
    mPendingTest = true;
}

void SceneTreePanel::nodeAdded(Node* node)
{
    if (!inScene(node)) return;
    mDirty = true;
}

void SceneTreePanel::nodeRemoved(Node* node)
{
    const bool wasScene = node == mScene;
    if (!wasScene && !inScene(node)) return;

    if (rt::TreeNode** item = mItemOf.find(node))
    {
        mNodeOf.erase(*item);
        mItemOf.erase(node);
    }
    if (wasScene) mScene = nullptr;
    mDirty = true;
}

void SceneTreePanel::nodeRenamed(Node* node)
{
    if (!inScene(node)) return;
    mDirty = true;
}

void SceneTreePanel::selectionChanged()
{
    if (mDirty)
    {
        mRevealSelection = true;
        return;
    }
    applySelection(true);
}

std::uint64_t SceneTreePanel::hashScene() const
{
    std::uint64_t hash = hashStep(0, 5381);
    if (mScene) hashNodes(mScene, hash);
    return hash;
}

void SceneTreePanel::update()
{
    if (!mOutliner) return;
    if (mPendingTest)
    {
        mPendingTest = false;
        if (!mDirty && hashScene() != mLastHash) mDirty = true;
    }
    if (mDirty) rebuild();
    else
        syncVisibility();
}

void SceneTreePanel::setAllExpanded(bool expanded)
{
    if (!mOutliner) return;
    for (const auto& entry: mItemOf)
    {
        if (!expanded && entry.key == mScene) continue;
        entry.value->setExpanded(expanded);
    }
    mDirty = true;
}

void SceneTreePanel::rebuild()
{
    ++mRebuilds;

    mExpanded.clear();
    for (const auto& entry: mItemOf) mExpanded.put(entry.key, entry.value->isExpanded());

    mOutliner->clear();
    mItemOf.clear();
    mNodeOf.clear();
    mDirty = false;

    if (mScene)
    {
        rt::TreeNode* root = mOutliner->addRoot(labelOf(mScene));
        addNodes(mScene, root);
    }
    mLastHash = hashScene();

    const bool reveal = mRevealSelection;
    mRevealSelection = false;
    applySelection(reveal);
}

void SceneTreePanel::fillItem(Node* node, rt::TreeNode* item)
{
    item->setIconId(mIcons.icon(node));
    mItemOf.put(node, item);
    mNodeOf.put(item, node);

    const bool* expanded = mExpanded.find(node);
    if (expanded)
    {
        if (!*expanded) item->setExpanded(false);
    }
    else if (node != mScene)
        item->setExpanded(false);

    const Node3D* spatial = node->castTo<Node3D>();
    mOutliner->setNodeVisible(item, spatial ? spatial->isVisible() : true);
    mOutliner->setNodeLocked(item, false);
}

void SceneTreePanel::addNodes(Node* node, rt::TreeNode* item)
{
    fillItem(node, item);
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        Node* child = node->getChild(i);
        if (child->isInternal()) continue;
        addNodes(child, item->addChild(labelOf(child)));
    }
}

void SceneTreePanel::applySelection(bool expand)
{
    if (!mOutliner || mDirty) return;

    rt::TreeNode* target = nullptr;
    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    for (std::size_t i = selected.size(); i > 0 && !target; --i) target = itemOf(selected[i - 1]);

    rt::TreeNode* current = mOutliner->selectedNode();
    if (!target)
    {
        if (current) mDirty = true;
        return;
    }
    if (current == target) return;

    if (expand)
    {
        bool opened = false;
        for (rt::TreeNode* parent = target->parent(); parent; parent = parent->parent())
        {
            if (parent->isExpanded()) continue;
            parent->setExpanded(true);
            opened = true;
        }
        if (opened)
        {
            mDirty = true;
            return;
        }
    }
    selectItem(target);
}

void SceneTreePanel::selectItem(rt::TreeNode* item)
{
    const rt::Rect area = mOutliner->rect();
    if (area.w < kMinWidth) return;

    const rt::TreeNode* root = itemOf(mScene);
    int row = 0;
    if (!root || !findRow(root, item, row)) return;

    mSyncing = true;

    rt::MouseEvent top;
    top.scrollY = 1.0e6f;
    mOutliner->onMouseScroll(top);

    rt::MouseEvent press;
    press.localX = 0.0f;
    press.localY = (static_cast<float>(row) + 0.5f) * kRowHeight;
    mOutliner->onMousePress(press);

    const float wanted = (static_cast<float>(row) + 0.5f) * kRowHeight - area.h * 0.5f;
    if (wanted > 0.0f)
    {
        rt::MouseEvent down;
        down.scrollY = -wanted / (kRowHeight * 3.0f);
        mOutliner->onMouseScroll(down);
    }

    mSyncing = false;
}

void SceneTreePanel::itemSelected(rt::TreeNode* item)
{
    if (mSyncing) return;
    Node* node = nodeOf(item);
    if (!node) return;
    mSelection.clear();
    mSelection.add(node);
}

void SceneTreePanel::toggleVisible(Node* node)
{
    Node3D* spatial = node ? node->castTo<Node3D>() : nullptr;
    if (spatial) spatial->setVisible(!spatial->isVisible());
}

void SceneTreePanel::refreshVisible(Node* node)
{
    const Node3D* spatial = node ? node->castTo<Node3D>() : nullptr;
    rt::TreeNode* item = itemOf(node);
    if (spatial && item) mOutliner->setNodeVisible(item, spatial->isVisible());
}

void SceneTreePanel::visibilityToggled(rt::TreeNode* item)
{
    Node* node = nodeOf(item);
    if (!node) return;
    if (!node->castTo<Node3D>())
    {
        mOutliner->setNodeVisible(item, true);
        return;
    }

    ct::Vector<Node3D*> targets;
    targets.push_back(node->castTo<Node3D>());
    const ct::Vector<Node*> top = mSelection.topSelectedNodes();
    bool selected = false;
    for (std::size_t i = 0; i < top.size(); ++i) selected = selected || top[i] == node;
    if (top.size() > 1 && selected)
    {
        for (std::size_t i = 0; i < top.size(); ++i)
        {
            if (top[i] == node) continue;
            if (Node3D* other = top[i]->castTo<Node3D>()) targets.push_back(other);
        }
    }

    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;
    if (undo)
    {
        ct::String name = "Toggle Visible";
        if (targets.size() > 1)
        {
            name = "Toggle Visibility of ";
            name.append_number(static_cast<unsigned long long>(targets.size()));
            name += " Nodes";
        }
        undo->createAction(name);
        for (std::size_t i = 0; i < targets.size(); ++i)
        {
            undo->addDoProperty(targets[i], "visible",
                    PropertyValue::fromBool(!targets[i]->isVisible()));
            undo->addUndoProperty(targets[i], "visible",
                    PropertyValue::fromBool(targets[i]->isVisible()));
        }
        undo->commitAction();
    }
    else
    {
        for (std::size_t i = 0; i < targets.size(); ++i) toggleVisible(targets[i]);
        if (mListener) mListener->sceneEdited();
    }
    for (std::size_t i = 0; i < targets.size(); ++i) refreshVisible(targets[i]);
}

void SceneTreePanel::syncVisibility()
{
    for (const auto& entry: mItemOf)
    {
        const Node3D* spatial = entry.key->castTo<Node3D>();
        if (spatial && mOutliner->isNodeVisible(entry.value) != spatial->isVisible())
            mOutliner->setNodeVisible(entry.value, spatial->isVisible());
    }
}

} // namespace prisma3d
