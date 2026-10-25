#include "prisma3d/editor/HierarchyActions.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/NodeNaming.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <igui/widgets/AppWidgets.hpp>
#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/CodeEditor.hpp>
#include <igui/widgets/ConsoleWidget.hpp>
#include <igui/widgets/DataWidgets.hpp>
#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <ct/hashmap.hpp>
#include <ct/sort.hpp>

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kRenameTitle = "Rename Node";
const char* const kRenameAction = "Rename Node";
const char* const kDuplicateAction = "Duplicate Node(s)";
const char* const kDeleteAction = "Delete Node(s)";
const char* const kMoveAction = "Move Node(s)";
const char* const kReparentAction = "Reparent Node";
const char* const kReparentToNewAction = "Reparent to New Node";
const char* const kPrefabTitle = "Save as Prefab";
const char* const kSaveAsPrefabAction = "Save as Prefab";
const char* const kStaticCollisionAction = "Create Static Collision";
const char* const kPrefabFolder = "Assets/Prefabs";
const char* const kActionLabels[HierarchyActions::kActionCount] = { "Add Child Node...",
    "Duplicate", "Rename", "Move Up", "Move Down", "Reparent...", "Reparent to New Node...",
    "Save as Prefab...", "Create Static Collision", "Delete" };
const char* const kActionShortcuts[HierarchyActions::kActionCount] = { "Ctrl+A", "Ctrl+D", "F2",
    "Ctrl+Up", "Ctrl+Down", "", "", "", "", "Del" };
const char* const kPreferredTypes[] = { "Control", "Node2D", "Node3D" };

const unsigned kChordMask = kModShift | kModControl | kModAlt | kModSuper;

bool visibleInTree(const rt::Widget* widget)
{
    for (const rt::Widget* current = widget; current; current = current->parent())
        if (!current->isVisible()) return false;
    return true;
}

Node* firstEnvironment(Node* node)
{
    if (node->castTo<WorldEnvironment>()) return node;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (Node* found = firstEnvironment(node->getChild(i))) return found;
    return nullptr;
}

void collectMeshInstances(Node* node, ct::Vector<MeshInstance3D*>& out)
{
    if (MeshInstance3D* instance = node->castTo<MeshInstance3D>()) out.push_back(instance);
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        Node* child = node->getChild(i);
        if (!child->isInternal()) collectMeshInstances(child, out);
    }
}

bool shapeUses(Node* body, const ct::String& path)
{
    for (std::size_t i = 0; i < body->getChildCount(); ++i)
    {
        const CollisionShape3D* shape = body->getChild(i)->castTo<CollisionShape3D>();
        if (shape && shape->getSourceMeshPath() == path) return true;
    }
    return false;
}

bool hasStaticCollision(MeshInstance3D* instance, const ct::String& path)
{
    Node* parent = instance->getParent();
    if (parent && parent->castTo<StaticBody3D>() && shapeUses(parent, path)) return true;
    for (std::size_t i = 0; i < instance->getChildCount(); ++i)
    {
        Node* child = instance->getChild(i);
        if (child->castTo<StaticBody3D>() && shapeUses(child, path)) return true;
    }
    return false;
}

bool hasShownChildren(const Node* node)
{
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (!node->getChild(i)->isInternal()) return true;
    return false;
}

bool treeBefore(const Node* a, const Node* b)
{
    if (a == b) return false;
    ct::Vector<std::ptrdiff_t> first;
    ct::Vector<std::ptrdiff_t> second;
    for (const Node* node = a; node; node = node->getParent()) first.push_back(node->getIndex());
    for (const Node* node = b; node; node = node->getParent()) second.push_back(node->getIndex());

    std::size_t i = first.size();
    std::size_t j = second.size();
    while (i > 0 && j > 0)
    {
        const std::ptrdiff_t left = first[--i];
        const std::ptrdiff_t right = second[--j];
        if (left != right) return left < right;
    }
    return i == 0 && j > 0;
}

void sortInTreeOrder(ct::Vector<Node*>& nodes)
{
    ct::sort(nodes.begin(), nodes.end(), [](Node* a, Node* b) { return treeBefore(a, b); });
}

std::ptrdiff_t shownIndex(const Node* node)
{
    const Node* parent = node->getParent();
    if (!parent) return 0;
    std::ptrdiff_t index = 0;
    for (std::size_t i = 0; i < parent->getChildCount(); ++i)
    {
        const Node* child = parent->getChild(i);
        if (child == node) return index;
        if (!child->isInternal()) ++index;
    }
    return index;
}

std::ptrdiff_t shownCount(const Node* parent)
{
    std::ptrdiff_t count = 0;
    for (std::size_t i = 0; i < parent->getChildCount(); ++i)
        if (!parent->getChild(i)->isInternal()) ++count;
    return count;
}

bool dropKeepsOrder(const Node* parent, const ct::Vector<Node*>& nodes, const Node* before)
{
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i]->getParent() != parent) return false;

    ct::Vector<const Node*> current;
    ct::Vector<const Node*> wanted;
    for (std::size_t i = 0; i < parent->getChildCount(); ++i)
    {
        const Node* child = parent->getChild(i);
        if (child->isInternal()) continue;
        current.push_back(child);
        bool moving = false;
        for (std::size_t n = 0; n < nodes.size(); ++n) moving = moving || nodes[n] == child;
        if (moving) continue;
        if (child == before)
            for (std::size_t n = 0; n < nodes.size(); ++n) wanted.push_back(nodes[n]);
        wanted.push_back(child);
    }
    if (current.size() != wanted.size()) return false;
    for (std::size_t i = 0; i < current.size(); ++i)
        if (current[i] != wanted[i]) return false;
    return true;
}

struct PathReference
{
    Node* node;
    ct::String property;
    ct::String path;
    Node* target;
};

void collectPaths(const Node* scene, Node* node, ct::Vector<PathReference>& references)
{
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList(node->getClassName(), list);
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        if (list[i].isGroup() || list[i].type != PropertyType::NodePath) continue;
        PropertyValue value;
        if (!NodeClassDB::getProperty(node, list[i].name, value)) continue;
        const ct::String path = value.asString();
        if (path.empty()) continue;
        Node* target = node->getNode(path);
        if (!target || (target != scene && !scene->isAncestorOf(target))) continue;
        references.push_back(PathReference{ node, list[i].name, path, target });
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collectPaths(scene, node->getChild(i), references);
}

ct::Vector<EditorTreeEdits::PropertyChange> repairPaths(const ct::Vector<PathReference>& references)
{
    ct::Vector<EditorTreeEdits::PropertyChange> changes;
    for (std::size_t i = 0; i < references.size(); ++i)
    {
        const PathReference& reference = references[i];
        if (reference.node->getNode(reference.path) == reference.target) continue;
        const ct::String path = reference.node->getPathTo(reference.target);
        if (path.empty()) continue;
        EditorTreeEdits::PropertyChange change;
        change.node = reference.node;
        change.property = reference.property;
        change.before = PropertyValue::fromNodePath(reference.path);
        change.after = PropertyValue::fromNodePath(path);
        NodeClassDB::setProperty(reference.node, reference.property, change.after);
        changes.push_back(change);
    }
    return changes;
}

HierarchyActions::DropPlace dropPlace(rt::Outliner::DropZone zone)
{
    switch (zone)
    {
        case rt::Outliner::DropZone::Before:
            return HierarchyActions::kDropBefore;
        case rt::Outliner::DropZone::Inside:
            return HierarchyActions::kDropInside;
        default:
            return HierarchyActions::kDropAfter;
    }
}

int depthOf(const Node* node, const Node* scene)
{
    int depth = 0;
    for (const Node* current = node; current && current != scene; current = current->getParent())
        ++depth;
    return depth;
}

std::ptrdiff_t shownNeighbour(const Node* node, bool down)
{
    const Node* parent = node->getParent();
    if (!parent) return -1;
    const std::ptrdiff_t index = node->getIndex();
    if (down)
    {
        for (std::size_t i = static_cast<std::size_t>(index) + 1; i < parent->getChildCount(); ++i)
            if (!parent->getChild(i)->isInternal()) return static_cast<std::ptrdiff_t>(i);
        return -1;
    }
    for (std::size_t i = static_cast<std::size_t>(index); i > 0; --i)
        if (!parent->getChild(i - 1)->isInternal()) return static_cast<std::ptrdiff_t>(i - 1);
    return -1;
}

} // namespace

HierarchyActions::HierarchyActions(SceneTree& tree, EditorDocument& document,
        EditorSelection& selection, SceneTreePanel& hierarchy)
        : mTree(tree),
          mDocument(document),
          mSelection(selection),
          mHierarchy(hierarchy),
          mLink(ct::make_rc<Link>(Link{ this })),
          mViewport(nullptr),
          mCreate(ct::make_unique<CreateMenu>(*this)),
          mContextMenu(nullptr),
          mRenameTarget(nullptr),
          mPrefabTarget(nullptr),
          mCreateMode(kCreateChild),
          mAddButton(-1),
          mDuplicateButton(-1),
          mDeleteButton(-1),
          mCollapseButton(-1),
          mExpandButton(-1)
{
    mTree.addListener(this);
}

HierarchyActions::~HierarchyActions()
{
    mTree.removeListener(this);
    mLink->owner = nullptr;
}

const char* HierarchyActions::label(Action action)
{
    return action >= 0 && action < kActionCount ? kActionLabels[action] : "";
}

const char* HierarchyActions::shortcut(Action action)
{
    return action >= 0 && action < kActionCount ? kActionShortcuts[action] : "";
}

bool HierarchyActions::attach()
{
    rt::Outliner* outliner = mHierarchy.outliner();
    if (mContextMenu || !outliner) return false;
    mContextMenu = new rt::Menu();
    populateContext(mContextMenu);
    outliner->setContextMenu(mContextMenu);

    const ct::Rc<Link> dragLink = mLink;
    outliner->setDragEnabled(true);
    outliner->setDragFilter(
            [dragLink](rt::TreeNode* item)
            {
                HierarchyActions* owner = dragLink->owner;
                return owner && owner->canDragNode(owner->mHierarchy.nodeOf(item));
            });
    outliner->setDropFilter(
            [dragLink](rt::TreeNode* item, rt::TreeNode* target, rt::Outliner::DropZone zone)
            {
                HierarchyActions* owner = dragLink->owner;
                if (!owner) return false;
                Node* over = target ? owner->mHierarchy.nodeOf(target) : nullptr;
                return owner->canDropNode(owner->mHierarchy.nodeOf(item), over, dropPlace(zone));
            });
    mHierarchy.nodeActivated.connect(
            [dragLink](Node* node)
            {
                if (dragLink->owner) dragLink->owner->focusNode(node);
            });
    outliner->nodeDropped.connect(
            [dragLink](rt::TreeNode* item, rt::TreeNode* target, rt::Outliner::DropZone zone)
            {
                HierarchyActions* owner = dragLink->owner;
                if (!owner) return;
                Node* over = target ? owner->mHierarchy.nodeOf(target) : nullptr;
                ct::String error;
                if (!owner->dropNode(owner->mHierarchy.nodeOf(item), over, dropPlace(zone),
                            error) &&
                        !error.empty())
                    owner->showError(error);
            });

    if (rt::ToolBar* bar = mHierarchy.toolBar())
    {
        mAddButton = bar->addButton(EditorIcons::get("Add"), kAddTooltip);
        bar->addSeparator();
        mDuplicateButton = bar->addButton(EditorIcons::get("Duplicate"), kDuplicateTooltip);
        mDeleteButton = bar->addButton(EditorIcons::get("Remove"), kDeleteTooltip);
        bar->addSeparator();
        mCollapseButton = bar->addButton(EditorIcons::get("MoveUp"), "Collapse all");
        mExpandButton = bar->addButton(EditorIcons::get("MoveDown"), "Expand all");
        mCreate->buildDropdown();
        const ct::Rc<Link> link = mLink;
        bar->buttonClicked.connect(
                [link](int id)
                {
                    HierarchyActions* owner = link->owner;
                    if (!owner) return;
                    if (id == owner->mAddButton) owner->showAddMenu();
                    else if (id == owner->mDuplicateButton)
                        owner->run(kDuplicate);
                    else if (id == owner->mDeleteButton)
                        owner->run(kDelete);
                    else if (id == owner->mCollapseButton)
                        owner->mHierarchy.setAllExpanded(false);
                    else if (id == owner->mExpandButton)
                        owner->mHierarchy.setAllExpanded(true);
                });
    }
    update();
    return true;
}

bool HierarchyActions::attached() const
{
    return mContextMenu != nullptr;
}

void HierarchyActions::update()
{
    if (mContextMenu) refresh(mContextMenu);
    mCreate->refresh(mCreate->dropdown());
    if (mAddButton >= 0)
    {
        if (rt::ToolBar* bar = mHierarchy.toolBar())
        {
            bar->setItemEnabled(mAddButton, canAddChild());
            bar->setItemEnabled(mDuplicateButton, canDuplicate());
            bar->setItemEnabled(mDeleteButton, canDelete());
        }
    }
}

void HierarchyActions::setViewport(EditorViewportPanel* viewport)
{
    mViewport = viewport;
}

Math::Vec3 HierarchyActions::focusPoint() const
{
    if (!mViewport) return Math::Vec3(0.0f, 0.0f, 0.0f);
    const EditorCamera::Cursor& cursor = mViewport->camera().cursor();
    return Math::Vec3(static_cast<float>(cursor.posX), static_cast<float>(cursor.posY),
            static_cast<float>(cursor.posZ));
}

CreateMenu& HierarchyActions::createMenu()
{
    return *mCreate;
}

int HierarchyActions::addButtonId() const
{
    return mAddButton;
}

int HierarchyActions::duplicateButtonId() const
{
    return mDuplicateButton;
}

int HierarchyActions::deleteButtonId() const
{
    return mDeleteButton;
}

rt::Menu* HierarchyActions::contextMenu() const
{
    return mContextMenu;
}

void HierarchyActions::populate(rt::Menu* menu)
{
    if (!menu) return;
    const ct::Rc<Link> link = mLink;
    const auto add = [&](Action action)
    {
        rt::MenuAction* item = menu->addAction(label(action),
                [link, action]
                {
                    if (link->owner) link->owner->run(action);
                });
        item->setShortcut(shortcut(action));
    };
    add(kAddChild);
    menu->addSeparator();
    add(kDuplicate);
    menu->addSeparator();
    add(kRename);
    menu->addSeparator();
    add(kMoveUp);
    add(kMoveDown);
    add(kReparent);
    add(kReparentToNew);
    add(kSaveAsPrefab);
    add(kStaticCollision);
    menu->addSeparator();
    add(kDelete);
}

void HierarchyActions::populateContext(rt::Menu* menu)
{
    if (!menu) return;
    rt::MenuAction* create = menu->addAction(CreateMenu::kTitle);
    rt::Menu* submenu = mCreate->newMenu();
    populateCreate(submenu);
    create->setSubmenu(submenu);
    menu->addSeparator();
    populate(menu);
}

void HierarchyActions::populateCreate(rt::Menu* menu)
{
    mCreate->populate(menu);
}

void HierarchyActions::refresh(rt::Menu* menu) const
{
    if (!menu) return;
    const int offset = menu->actionCount() - kItemCount;
    if (offset < 0) return;
    if (offset == kContextOffset) refreshCreate(menu->action(0)->submenu());
    const auto at = [&](MenuItem item) { return menu->action(offset + item); };
    at(kItemAddChild)->setEnabled(canAddChild());
    at(kItemDuplicate)->setEnabled(canDuplicate());
    at(kItemRename)->setEnabled(canRename());
    at(kItemMoveUp)->setEnabled(canMoveUp());
    at(kItemMoveDown)->setEnabled(canMoveDown());
    at(kItemReparent)->setEnabled(canReparent());
    at(kItemReparentToNew)->setEnabled(canReparentToNew());
    at(kItemSaveAsPrefab)->setEnabled(canSaveAsPrefab());
    at(kItemStaticCollision)->setEnabled(canCreateStaticCollision());
    at(kItemDelete)->setEnabled(canDelete());
}

void HierarchyActions::refreshCreate(rt::Menu* menu) const
{
    mCreate->refresh(menu);
}

void HierarchyActions::showAddMenu()
{
    if (!canAddChild()) return;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    float x = app.mouseX();
    float y = app.mouseY();
    if (rt::ToolBar* bar = mHierarchy.toolBar())
    {
        const rt::Rect rect = bar->absoluteRect();
        if (x < rect.x) x = rect.x;
        if (x > rect.x + rect.w) x = rect.x + rect.w;
        y = rect.y + rect.h;
    }
    mCreate->showDropdown(x, y);
}

void HierarchyActions::nodeRemoved(Node* node)
{
    if (node == mRenameTarget) mRenameTarget = nullptr;
    if (node == mPrefabTarget) mPrefabTarget = nullptr;
    if (ReparentDialog* dialogBox = reparentDialog())
        if (dialogBox->tracks(node)) dialogBox->cancel();
}

bool HierarchyActions::usable(const Node* node) const
{
    const Node* scene = mDocument.scene();
    if (!scene || mDocument.playing() || !node || node->isQueuedForDeletion()) return false;
    if (node != scene && !scene->isAncestorOf(node)) return false;
    for (const Node* current = node; current && current != scene; current = current->getParent())
        if (current->isInternal()) return false;
    return true;
}

bool HierarchyActions::foreign(const Node* node) const
{
    const Node* scene = mDocument.scene();
    const Node* owner = node->getOwner();
    return node != scene && owner && owner != scene;
}

bool HierarchyActions::anyForeign(const ct::Vector<Node*>& nodes) const
{
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (foreign(nodes[i])) return true;
    return false;
}

bool HierarchyActions::rootSelected() const
{
    const Node* scene = mDocument.scene();
    return scene && mSelection.isSelected(scene);
}

ct::Vector<Node*> HierarchyActions::topNodes() const
{
    ct::Vector<Node*> nodes;
    const ct::Vector<Node*> top = mSelection.topSelectedNodes();
    for (std::size_t i = 0; i < top.size(); ++i)
        if (usable(top[i])) nodes.push_back(top[i]);
    sortInTreeOrder(nodes);
    return nodes;
}

ct::Vector<Node*> HierarchyActions::allNodes(bool sorted) const
{
    ct::Vector<Node*> nodes;
    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    for (std::size_t i = 0; i < selected.size(); ++i)
        if (usable(selected[i])) nodes.push_back(selected[i]);
    if (sorted) sortInTreeOrder(nodes);
    return nodes;
}

bool HierarchyActions::anyUsable() const
{
    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    for (std::size_t i = 0; i < selected.size(); ++i)
        if (usable(selected[i])) return true;
    return false;
}

bool HierarchyActions::rename(Node* node, const ct::String& requested, ct::String& error)
{
    error = ct::String();
    if (!usable(node)) return false;
    if (foreign(node))
    {
        error = kForeignRefusal;
        return false;
    }

    const ct::String name = NodeNaming::resolve(node, requested, error);
    if (name == node->getName()) return true;

    EditorUndoRedoManager& undo = mDocument.undoRedo();
    undo.createAction(kRenameAction);
    undo.addDoProperty(node, "name", PropertyValue::fromString(name));
    undo.addUndoProperty(node, "name", PropertyValue::fromString(node->getName()));
    undo.commitAction();
    return true;
}

bool HierarchyActions::duplicateSelection(ct::String& error)
{
    error = ct::String();
    Node* scene = mDocument.scene();
    if (!scene) return false;
    if (rootSelected())
    {
        error = kRootRefusal;
        return false;
    }

    const ct::Vector<Node*> nodes = topNodes();
    if (nodes.empty()) return false;

    ct::HashMap<Node*, Node*> addBelow;
    for (std::size_t i = 0; i < nodes.size(); ++i) addBelow.put(nodes[i]->getParent(), nodes[i]);

    const ct::Vector<Node*> selectionBefore = mSelection.selectedNodes();
    EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
    ct::Vector<Node*> duplicates;
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        Node* node = nodes[i];
        Node* parent = node->getParent();

        ct::String text;
        SceneResult saved;
        if (!SceneSerializer::save(node, text, &saved))
        {
            log_error("editor: could not duplicate '%s':\n%s", node->getName().c_str(),
                    saved.describe().c_str());
            continue;
        }

        SceneResult loaded;
        Node* duplicate = SceneSerializer::load(text, parent, loaded);
        if (!duplicate || !loaded.success())
        {
            log_error("editor: could not duplicate '%s':\n%s", node->getName().c_str(),
                    loaded.describe().c_str());
            if (duplicate) duplicate->queueFree();
            continue;
        }
        if (loaded.warningCount() > 0)
            log_warn("editor: duplicating '%s':\n%s", node->getName().c_str(),
                    loaded.describe().c_str());

        Node* const* anchor = addBelow.find(parent);
        if (anchor && *anchor)
            parent->moveChild(duplicate, static_cast<std::size_t>((*anchor)->getIndex() + 1));
        addBelow.put(parent, duplicate);
        duplicates.push_back(duplicate);
    }

    if (duplicates.empty()) return false;

    mSelection.clear();
    for (std::size_t i = 0; i < duplicates.size(); ++i) mSelection.add(duplicates[i]);
    recordCreated(kDuplicateAction, duplicates, selectionBefore);
    return true;
}

bool HierarchyActions::deleteSelection(ct::String& error)
{
    error = ct::String();
    if (!canAddChild()) return false;
    if (rootSelected())
    {
        error = kRootRefusal;
        return false;
    }

    const ct::Vector<Node*> nodes = topNodes();
    if (nodes.empty()) return false;
    if (anyForeign(nodes))
    {
        error = kForeignRefusal;
        return false;
    }

    EditorTreeEdits::Record record = EditorTreeEdits::removed(kDeleteAction, nodes);
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
    return true;
}

bool HierarchyActions::moveBlocked(const ct::Vector<Node*>& nodes, bool down) const
{
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (shownNeighbour(nodes[i], down) < 0) return true;
    return false;
}

bool HierarchyActions::moveSelection(bool down, ct::String& error)
{
    error = ct::String();
    if (!canAddChild()) return false;
    if (rootSelected())
    {
        error = kRootRefusal;
        return false;
    }

    const ct::Vector<Node*> nodes = allNodes();
    if (nodes.empty()) return false;
    if (anyForeign(nodes))
    {
        error = kForeignRefusal;
        return false;
    }
    if (moveBlocked(nodes, down)) return false;

    ct::Vector<Node*> parents;
    ct::Vector<ct::Vector<Node*>> before;
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        Node* parent = nodes[i]->getParent();
        bool known = false;
        for (std::size_t p = 0; p < parents.size(); ++p) known = known || parents[p] == parent;
        if (known) continue;
        parents.push_back(parent);
        before.push_back(EditorTreeEdits::childOrder(parent));
    }

    {
        EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            Node* node = down ? nodes[nodes.size() - 1 - i] : nodes[i];
            const std::ptrdiff_t target = shownNeighbour(node, down);
            if (target < 0) continue;
            node->getParent()->moveChild(node, static_cast<std::size_t>(target));
        }
    }

    ct::Vector<ct::Vector<Node*>> after;
    for (std::size_t p = 0; p < parents.size(); ++p)
        after.push_back(EditorTreeEdits::childOrder(parents[p]));

    EditorUndoRedoManager& undo = mDocument.undoRedo();
    undo.createAction(kMoveAction);
    undo.addDoMethod(
            [parents, after]
            {
                for (std::size_t p = 0; p < parents.size(); ++p)
                    EditorTreeEdits::restoreOrder(parents[p], after[p]);
            });
    undo.addUndoMethod(
            [parents, before]
            {
                for (std::size_t p = 0; p < parents.size(); ++p)
                    EditorTreeEdits::restoreOrder(parents[p], before[p]);
            });
    undo.commitAction(false);
    return true;
}

bool HierarchyActions::canAddChild() const
{
    return mDocument.scene() != nullptr && !mDocument.playing();
}

bool HierarchyActions::canReparent() const
{
    return canAddChild() && !rootSelected() && anyUsable();
}

bool HierarchyActions::canReparentToNew() const
{
    return canReparent();
}

bool HierarchyActions::canRename() const
{
    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    return selected.size() == 1 && usable(selected[0]);
}

bool HierarchyActions::canDuplicate() const
{
    return canAddChild() && !rootSelected() && anyUsable();
}

bool HierarchyActions::canDelete() const
{
    return canDuplicate();
}

bool HierarchyActions::canMove(bool down) const
{
    if (!canAddChild() || rootSelected()) return false;
    const ct::Vector<Node*> nodes = allNodes(false);
    return !nodes.empty() && !moveBlocked(nodes, down);
}

bool HierarchyActions::canMoveUp() const
{
    return canMove(false);
}

bool HierarchyActions::canMoveDown() const
{
    return canMove(true);
}

bool HierarchyActions::can(Action action) const
{
    switch (action)
    {
        case kAddChild:
            return canAddChild();
        case kRename:
            return canRename();
        case kDuplicate:
            return canDuplicate();
        case kMoveUp:
            return canMoveUp();
        case kMoveDown:
            return canMoveDown();
        case kReparent:
            return canReparent();
        case kReparentToNew:
            return canReparentToNew();
        case kSaveAsPrefab:
            return canSaveAsPrefab();
        case kStaticCollision:
            return canCreateStaticCollision();
        case kDelete:
            return canDelete();
        case kActionCount:
            break;
    }
    return false;
}

void HierarchyActions::run(Action action)
{
    switch (action)
    {
        case kAddChild:
            requestAddChild();
            break;
        case kRename:
            requestRename();
            break;
        case kDuplicate:
            requestDuplicate();
            break;
        case kMoveUp:
            requestMove(false);
            break;
        case kMoveDown:
            requestMove(true);
            break;
        case kReparent:
            requestReparent();
            break;
        case kReparentToNew:
            requestReparentToNew();
            break;
        case kSaveAsPrefab:
            requestSaveAsPrefab();
            break;
        case kStaticCollision:
            requestStaticCollision();
            break;
        case kDelete:
            requestDelete(true);
            break;
        case kActionCount:
            break;
    }
}

void HierarchyActions::showError(const ct::String& message)
{
    log_error("editor: %s", message.c_str());
    rt::AlertDialog::show("Error", message);
}

rt::InputBox* HierarchyActions::renameBox() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (rt::InputBox* box = rt::widget_cast<rt::InputBox>(floats[i - 1]))
            if (box->title() == kRenameTitle) return box;
    return nullptr;
}

rt::Dialog* HierarchyActions::dialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return app ? rt::widget_cast<rt::Dialog>(app->popup()) : nullptr;
}

void HierarchyActions::requestRename()
{
    if (!canRename()) return;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    if (rt::InputBox* existing = renameBox())
    {
        app.bringToFront(existing);
        return;
    }

    if (foreign(mSelection.selectedNodes()[0]))
    {
        showError(kForeignRefusal);
        return;
    }
    mRenameTarget = mSelection.selectedNodes()[0];
    rt::InputBox* box = app.addFloat<rt::InputBox>(kRenameTitle, "Name:");
    box->setText(mRenameTarget->getName());
    box->setAcceptLabel("Rename");
    const ct::Rc<Link> link = mLink;
    box->accepted.connect(
            [link](const ct::String& text)
            {
                if (link->owner) link->owner->renameAccepted(text);
            });
}

void HierarchyActions::renameAccepted(const ct::String& text)
{
    Node* node = mRenameTarget;
    mRenameTarget = nullptr;
    if (!node) return;

    ct::String error;
    rename(node, text, error);
    if (!error.empty()) showError(error);
}

void HierarchyActions::requestDuplicate()
{
    ct::String error;
    duplicateSelection(error);
    if (!error.empty()) showError(error);
}

void HierarchyActions::requestMove(bool down)
{
    ct::String error;
    moveSelection(down, error);
    if (!error.empty()) showError(error);
}

ct::String HierarchyActions::deleteMessage() const
{
    const ct::Vector<Node*> nodes = topNodes();
    if (nodes.empty()) return ct::String();

    if (nodes.size() > 1)
    {
        bool children = false;
        for (std::size_t i = 0; i < nodes.size(); ++i)
            children = children || hasShownChildren(nodes[i]);
        ct::String message = "Delete ";
        message.append_number(static_cast<unsigned long long>(nodes.size()));
        message += children ? " nodes and any children?" : " nodes?";
        return message;
    }

    const Node* node = nodes[0];
    ct::String message = "Delete node \"";
    message += node->getName();
    message += (!node->isInstance() && hasShownChildren(node)) ? "\" and its children?" : "\"?";
    return message;
}

void HierarchyActions::requestDelete(bool confirm)
{
    if (!canAddChild() || mSelection.selectedNodes().empty()) return;
    if (rootSelected())
    {
        showError(kRootRefusal);
        return;
    }
    if (topNodes().empty()) return;
    if (anyForeign(topNodes()))
    {
        showError(kForeignRefusal);
        return;
    }

    if (!confirm)
    {
        ct::String error;
        deleteSelection(error);
        if (!error.empty()) showError(error);
        return;
    }
    openDeleteDialog(deleteMessage());
}

void HierarchyActions::openDeleteDialog(const ct::String& message)
{
    if (dialog()) return;
    rt::Dialog* box = new rt::Dialog("Delete", message);
    const ct::Rc<Link> link = mLink;
    box->addButton("Delete", rt::Dialog::Role::Danger,
            [link]
            {
                if (link->owner) link->owner->confirmDelete();
            });
    box->addButton("Cancel", rt::Dialog::Role::Cancel);
    box->show();
}

void HierarchyActions::confirmDelete()
{
    ct::String error;
    deleteSelection(error);
    if (!error.empty()) showError(error);
}

CreateNodeDialog* HierarchyActions::createDialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (CreateNodeDialog* box = rt::widget_cast<CreateNodeDialog>(floats[i - 1])) return box;
    return nullptr;
}

ReparentDialog* HierarchyActions::reparentDialog() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (ReparentDialog* box = rt::widget_cast<ReparentDialog>(floats[i - 1])) return box;
    return nullptr;
}

ct::String HierarchyActions::preferredType() const
{
    const Node* scene = mDocument.scene();
    if (!scene) return ct::String();
    for (const char* type: kPreferredTypes)
        if (scene->isClass(type)) return ct::String(type);
    return ct::String();
}

bool HierarchyActions::validType(const ct::String& type, ct::String& error) const
{
    if (!type.empty() && !CreateNodeDialog::shouldHideType(type)) return true;
    error = ct::String("Can't create a node of the class '") + type + "'.";
    return false;
}

void HierarchyActions::allowChildren(Node* parent)
{
    Node* scene = mDocument.scene();
    if (!scene || !parent) return;

    Node* instance = nullptr;
    for (Node* current = parent; current && current != scene; current = current->getParent())
    {
        if (current->isInstance())
        {
            instance = current;
            break;
        }
    }
    if (!instance) return;

    for (Node* owner = instance->getOwner(); owner && owner != scene && owner->isInstance();
            owner = instance->getOwner())
        instance = owner;

    if (parent == instance || scene->isEditableInstance(instance)) return;
    scene->setEditableInstance(instance, true);
    mMadeEditable.push_back(instance);
}

ct::Vector<Node*> HierarchyActions::takeEditable()
{
    ct::Vector<Node*> taken = mMadeEditable;
    mMadeEditable.clear();
    return taken;
}

ct::Vector<Node*> HierarchyActions::createMeshPieces(MeshInstance3D* instance,
        const ct::Vector<ct::String>& pieces)
{
    ct::Vector<Node*> created;
    Node* parent = instance->getParent();
    PropertyValue material;
    const bool hasMaterial = NodeClassDB::getProperty(instance, "material_override", material);
    for (std::size_t i = 0; i < pieces.size(); ++i)
    {
        allowChildren(parent);
        Node* child = createUnder(parent, "MeshInstance3D");
        MeshInstance3D* piece = child ? child->castTo<MeshInstance3D>() : nullptr;
        if (!piece) continue;
        const std::size_t at = pieces[i].find("::");
        piece->setName(at == ct::String::npos ? instance->getName() : pieces[i].substr(at + 2));
        piece->setTransform(instance->getTransform());
        NodeClassDB::setProperty(piece, "mesh", PropertyValue::fromResourcePath(pieces[i]));
        if (hasMaterial) NodeClassDB::setProperty(piece, "material_override", material);
        created.push_back(piece);
    }
    return created;
}

bool HierarchyActions::splitMesh(Node* node, const ct::String& meshPath,
        const ct::Vector<ct::String>& pieces)
{
    MeshInstance3D* instance = node ? node->castTo<MeshInstance3D>() : nullptr;
    if (!instance || !node->getParent()) return false;
    PropertyValue before;
    if (!NodeClassDB::getProperty(node, "mesh", before)) return false;
    const PropertyValue after = PropertyValue::fromResourcePath(meshPath);
    const ct::Vector<Node*> selectionBefore = mSelection.selectedNodes();

    NodeClassDB::setProperty(node, "mesh", after);
    const ct::Vector<Node*> created = createMeshPieces(instance, pieces);
    EditorTreeEdits::Record record =
            EditorTreeEdits::added("Extract Surface", created, selectionBefore);
    record.scene = mDocument.scene();
    record.editable = takeEditable();
    EditorTreeEdits::PropertyChange change;
    change.node = node;
    change.property = "mesh";
    change.before = before;
    change.after = after;
    record.properties.push_back(change);
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
    return true;
}

bool HierarchyActions::rewriteMesh(Node* node, const ct::String& file, const ct::String& absolute,
        const PMeshSurfaceEdit& edit, ct::String& error)
{
    MeshInstance3D* instance = node ? node->castTo<MeshInstance3D>() : nullptr;
    AssetLoader* loader = AssetLoader::singleton();
    MaterialStore* materials = MaterialStore::singleton();
    if (!instance || !node->getParent() || !loader || !materials)
    {
        error = "the node cannot be edited";
        return false;
    }
    const ct::Vector<std::uint8_t> previous = FileSystem::getSingleton().readBinary(absolute);
    if (previous.empty())
    {
        error = ct::String("could not read '") + absolute + "'";
        return false;
    }
    ct::Vector<ct::String> extracted;
    if (!editModelToPMesh(*loader, *materials, absolute, edit, absolute, extracted, error))
        return false;
    const ct::Vector<std::uint8_t> next = FileSystem::getSingleton().readBinary(absolute);
    sceneres::reloadModel(file);

    ct::Vector<ct::String> pieces;
    for (std::size_t i = 0; i < extracted.size(); ++i) pieces.push_back(file + "::" + extracted[i]);
    const ct::Vector<Node*> selectionBefore = mSelection.selectedNodes();
    const ct::Vector<Node*> created = createMeshPieces(instance, pieces);

    EditorTreeEdits::Record record = EditorTreeEdits::added(
            created.empty() ? "Delete Surface" : "Extract Surface", created, selectionBefore);
    record.scene = mDocument.scene();
    record.editable = takeEditable();
    record.selectionAfter = created.empty() ? selectionBefore : created;
    record.afterDo = [absolute, file, next]
    {
        FileSystem::writeBinary(absolute, next.data(), next.size());
        sceneres::reloadModel(file);
    };
    record.afterUndo = [absolute, file, previous]
    {
        FileSystem::writeBinary(absolute, previous.data(), previous.size());
        sceneres::reloadModel(file);
    };
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
    return true;
}

void HierarchyActions::recordCreated(const ct::String& name, const ct::Vector<Node*>& created,
        const ct::Vector<Node*>& selectionBefore)
{
    EditorTreeEdits::Record record = EditorTreeEdits::added(name, created, selectionBefore);
    record.scene = mDocument.scene();
    record.editable = takeEditable();
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
}

Node* HierarchyActions::createUnder(Node* parent, const ct::String& type)
{
    allowChildren(parent);
    return parent->createChild(type.c_str(), true);
}

ct::Vector<Node*> HierarchyActions::createParents() const
{
    ct::Vector<Node*> parents;
    Node* scene = mDocument.scene();
    if (!scene) return parents;
    if (mSelection.selectedNodes().empty()) parents.push_back(scene);
    else
        parents = allNodes();
    return parents;
}

bool HierarchyActions::createNode(const ct::String& type, ct::String& error)
{
    error = ct::String();
    Node* scene = mDocument.scene();
    if (!scene) return false;
    if (!validType(type, error)) return false;

    return createAmong(type, createParents());
}

bool HierarchyActions::createNodeUnder(const ct::String& type, Node* parent, ct::String& error)
{
    error = ct::String();
    if (!mDocument.scene() || !parent) return false;
    if (!validType(type, error)) return false;

    ct::Vector<Node*> parents;
    parents.push_back(parent);
    return createAmong(type, parents);
}

bool HierarchyActions::createAmong(const ct::String& type, const ct::Vector<Node*>& parents)
{
    if (parents.empty()) return false;

    const ct::Vector<Node*> selectionBefore = mSelection.selectedNodes();
    EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
    mMadeEditable.clear();
    ct::Vector<Node*> created;
    for (std::size_t i = 0; i < parents.size(); ++i)
        if (Node* child = createUnder(parents[i], type)) created.push_back(child);
    if (created.empty()) return false;

    mSelection.clear();
    for (std::size_t i = 0; i < created.size(); ++i) mSelection.add(created[i]);
    recordCreated(kAddAction, created, selectionBefore);
    return true;
}

void HierarchyActions::requestEnvironment()
{
    ct::String error;
    selectEnvironment(error);
    if (!error.empty()) showError(error);
}

Node* HierarchyActions::findEnvironment() const
{
    Node* scene = mDocument.scene();
    if (!scene) return nullptr;
    return firstEnvironment(scene);
}

bool HierarchyActions::selectEnvironment(ct::String& error)
{
    error = ct::String();
    Node* scene = mDocument.scene();
    if (!scene || mDocument.playing()) return false;
    Node* environment = firstEnvironment(scene);
    if (!environment) return mCreate->createUnder(CreateMenu::kWorldEnvironment, scene, error);
    mSelection.clear();
    mSelection.add(environment);
    return true;
}

bool HierarchyActions::doReparent(Node* newParent, const ct::Vector<Node*>& nodes,
        const ct::Vector<Node*>& keepSelection, bool keepGlobalTransform, ct::String& error,
        ReparentOutcome& outcome, Node* before)
{
    if (nodes.empty()) return false;

    const std::ptrdiff_t firstIndex = shownCount(newParent);
    std::ptrdiff_t nodesBefore = firstIndex;
    bool noChange = true;
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        if (nodes[i] == newParent || nodes[i] == before) return false;

        if (shownIndex(nodes[i]) < firstIndex) --nodesBefore;

        if (nodes[i]->getParent() != newParent) noChange = false;
    }

    for (std::size_t i = 0; i < nodes.size() && noChange && !before; ++i)
        if (shownIndex(nodes[i]) != nodesBefore + static_cast<std::ptrdiff_t>(i)) noChange = false;

    if (before)
        noChange = before->getParent() == newParent && dropKeepsOrder(newParent, nodes, before);

    if (noChange) return false;
    if (before && before->getParent() != newParent) return false;

    for (const Node* validate = newParent; validate; validate = validate->getParent())
    {
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            if (nodes[i] == validate)
            {
                error = kSelectionChanged;
                return false;
            }
        }
    }

    mMadeEditable.clear();
    allowChildren(newParent);

    ct::Vector<PathReference> references;
    collectPaths(mDocument.scene(), mDocument.scene(), references);

    outcome.before = EditorTreeEdits::capture(nodes, true);

    EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        Node* node = nodes[i];
        Node3D* spatial = keepGlobalTransform ? node->castTo<Node3D>() : nullptr;
        Math::Mat4 global;
        if (spatial) global = spatial->getGlobalTransform();

        if (node->getParent() == newParent) newParent->moveChild(node, newParent->getChildCount());
        else
            node->reparent(newParent, true);
        if (before) newParent->moveChild(node, static_cast<std::size_t>(before->getIndex()));

        if (spatial) spatial->setGlobalTransform(global);
    }

    outcome.changes = repairPaths(references);
    outcome.after = EditorTreeEdits::capture(nodes, true);

    mSelection.clear();
    for (std::size_t i = 0; i < keepSelection.size(); ++i) mSelection.add(keepSelection[i]);
    return true;
}

bool HierarchyActions::reparentSelection(Node* newParent, bool keepGlobalTransform,
        ct::String& error)
{
    error = ct::String();
    if (!canAddChild() || !newParent) return false;
    if (rootSelected())
    {
        error = kRootRefusal;
        return false;
    }

    const ct::Vector<Node*> nodes = topNodes();
    if (nodes.empty()) return false;
    if (anyForeign(nodes))
    {
        error = kForeignRefusal;
        return false;
    }
    if (!usable(newParent)) return false;

    const ct::Vector<Node*> selection = mSelection.selectedNodes();
    ReparentOutcome outcome;
    if (!doReparent(newParent, nodes, selection, keepGlobalTransform, error, outcome)) return false;

    EditorTreeEdits::Record record;
    record.name = kReparentAction;
    record.before = outcome.before;
    record.after = outcome.after;
    record.properties = outcome.changes;
    record.selectionBefore = selection;
    record.selectionAfter = selection;
    record.scene = mDocument.scene();
    record.editable = takeEditable();
    record.execute = false;
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
    return true;
}

ct::Vector<Node*> HierarchyActions::draggedNodes(Node* node) const
{
    ct::Vector<Node*> nodes = topNodes();
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i] == node) return nodes;
    nodes.clear();
    if (node && usable(node)) nodes.push_back(node);
    return nodes;
}

Node* HierarchyActions::dropParent(const ct::Vector<Node*>& nodes, Node* target, DropPlace place,
        Node*& before) const
{
    before = nullptr;
    Node* parent = mDocument.scene();
    if (target && place == kDropInside) parent = target;
    else if (target)
    {
        const ig::retained::TreeNode* item = mHierarchy.itemOf(target);
        const bool opened =
                place == kDropAfter && item && item->hasChildren() && item->isExpanded();
        std::size_t first = 0;
        if (opened) parent = target;
        else
        {
            parent = target->getParent();
            if (!parent) return nullptr;
            first = static_cast<std::size_t>(target->getIndex());
        }

        if (place == kDropBefore) before = target;
        else
        {
            if (!opened) ++first;
            for (std::size_t i = first; i < parent->getChildCount() && !before; ++i)
            {
                Node* next = parent->getChild(i);
                bool moving = false;
                for (std::size_t n = 0; n < nodes.size(); ++n) moving = moving || nodes[n] == next;
                if (!next->isInternal() && !moving) before = next;
            }
        }
    }
    return parent && usable(parent) ? parent : nullptr;
}

bool HierarchyActions::canDragNode(Node* node) const
{
    return canAddChild() && node && node != mDocument.scene() && usable(node) && !foreign(node);
}

bool HierarchyActions::canDropNode(Node* draggedNode, Node* target, DropPlace place) const
{
    if (!canDragNode(draggedNode)) return false;
    const ct::Vector<Node*> nodes = draggedNodes(draggedNode);
    if (nodes.empty() || anyForeign(nodes)) return false;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i] == mDocument.scene() || nodes[i] == target) return false;

    Node* before = nullptr;
    Node* parent = dropParent(nodes, target, place, before);
    if (!parent) return false;
    for (const Node* walk = parent; walk; walk = walk->getParent())
        for (std::size_t i = 0; i < nodes.size(); ++i)
            if (nodes[i] == walk) return false;
    return true;
}

bool HierarchyActions::dropNode(Node* draggedNode, Node* target, DropPlace place, ct::String& error)
{
    error = ct::String();
    if (!canDropNode(draggedNode, target, place)) return false;

    const ct::Vector<Node*> nodes = draggedNodes(draggedNode);
    Node* before = nullptr;
    Node* parent = dropParent(nodes, target, place, before);
    if (!parent) return false;

    bool sameParent = true;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        sameParent = sameParent && nodes[i]->getParent() == parent;

    const ct::Vector<Node*> selection = mSelection.selectedNodes();
    ReparentOutcome outcome;
    if (!doReparent(parent, nodes, selection, true, error, outcome, before)) return false;

    EditorTreeEdits::Record record;
    record.name = sameParent ? kMoveAction : kReparentAction;
    record.before = outcome.before;
    record.after = outcome.after;
    record.properties = outcome.changes;
    record.selectionBefore = selection;
    record.selectionAfter = selection;
    record.scene = mDocument.scene();
    record.editable = takeEditable();
    record.execute = false;
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
    return true;
}

namespace
{

// A box worth framing: an empty one (a particle system with no particles reports min +FLT_MAX,
// max -FLT_MAX) or one with non-finite or absurd sizes would send the camera to infinity.
bool frameableAxis(float lo, float hi)
{
    const float limit = 1.0e6f;
    return isfinite(lo) && isfinite(hi) && lo <= hi && fabsf(lo) <= limit &&
           fabsf(hi) <= limit;
}

bool frameable(const Math::Box& box)
{
    return frameableAxis(box.min.x, box.max.x) && frameableAxis(box.min.y, box.max.y) &&
           frameableAxis(box.min.z, box.max.z);
}

void gatherBounds(const Node* node, Math::Box& box, bool& found)
{
    const VisualInstance3D* visual = node->castTo<VisualInstance3D>();
    if (visual && frameable(visual->getAabb()))
    {
        Math::Vec3 corners[8];
        visual->getAabb().GetCorners(corners);
        const Math::Mat4 global = visual->getGlobalTransform();
        for (int i = 0; i < 8; ++i)
        {
            const Math::Vec3 world = global.TransformPoint(corners[i]);
            if (!found)
            {
                box = Math::Box(world, world);
                found = true;
                continue;
            }
            box.min = Math::Vec3(fminf(box.min.x, world.x), fminf(box.min.y, world.y),
                    fminf(box.min.z, world.z));
            box.max = Math::Vec3(fmaxf(box.max.x, world.x), fmaxf(box.max.y, world.y),
                    fmaxf(box.max.z, world.z));
        }
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        gatherBounds(node->getChild(i), box, found);
}

} // namespace

void HierarchyActions::focusNode(Node* node)
{
    if (!mViewport || !node || !usable(node)) return;
    const Node3D* spatial = node->castTo<Node3D>();
    if (!spatial) return;
    Math::Box box;
    bool found = false;
    gatherBounds(node, box, found);
    Math::Vec3 center = spatial->getGlobalPosition();
    float radius = 1.0f;
    if (found && frameable(box))
    {
        center = box.Center();
        radius = fmaxf(box.Extents().Length(), 0.1f);
    }
    if (!isfinite(center.x) || !isfinite(center.y) || !isfinite(center.z)) return;
    EditorCamera& camera = mViewport->camera();
    EditorCamera::Cursor& cursor = camera.cursor();
    cursor.posX = center.x;
    cursor.posY = center.y;
    cursor.posZ = center.z;
    const float halfFov = camera.fov() * 0.5f * 3.14159265f / 180.0f;
    cursor.distance = fmaxf(radius / sinf(halfFov) * 1.1f, 0.5f);
}

bool HierarchyActions::canSaveAsPrefab() const
{
    if (!canAddChild() || !mDocument.project().isOpen()) return false;
    const ct::Vector<Node*> nodes = topNodes();
    return nodes.size() == 1 && nodes[0] != mDocument.scene() && !foreign(nodes[0]);
}

rt::InputBox* HierarchyActions::prefabBox() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (rt::InputBox* box = rt::widget_cast<rt::InputBox>(floats[i - 1]))
            if (box->title() == kPrefabTitle) return box;
    return nullptr;
}

void HierarchyActions::setAssetsChangedHandler(ct::Function<void()> handler)
{
    mAssetsChanged = static_cast<ct::Function<void()>&&>(handler);
}

bool HierarchyActions::canCreateStaticCollision() const
{
    if (!canAddChild()) return false;
    const ct::Vector<Node*> nodes = topNodes();
    ct::Vector<MeshInstance3D*> meshes;
    for (std::size_t i = 0; i < nodes.size(); ++i) collectMeshInstances(nodes[i], meshes);
    for (std::size_t i = 0; i < meshes.size(); ++i)
    {
        const ct::String path = sceneres::pathOf(meshes[i]->getMesh());
        if (!path.empty() && !hasStaticCollision(meshes[i], path)) return true;
    }
    return false;
}

bool HierarchyActions::createStaticCollision(ct::String& error)
{
    error = ct::String();
    if (!canCreateStaticCollision()) return false;
    const ct::Vector<Node*> nodes = topNodes();
    ct::Vector<MeshInstance3D*> meshes;
    for (std::size_t i = 0; i < nodes.size(); ++i) collectMeshInstances(nodes[i], meshes);
    const ct::Vector<Node*> selectionBefore = mSelection.selectedNodes();

    EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
    mMadeEditable.clear();
    ct::Vector<Node*> created;
    for (std::size_t i = 0; i < meshes.size(); ++i)
    {
        const ct::String path = sceneres::pathOf(meshes[i]->getMesh());
        if (path.empty() || hasStaticCollision(meshes[i], path)) continue;
        Node* body = createUnder(meshes[i], "StaticBody3D");
        if (!body) continue;
        Node* shapeNode = body->createChild("CollisionShape3D", true);
        CollisionShape3D* shape = shapeNode ? shapeNode->castTo<CollisionShape3D>() : nullptr;
        if (shape)
        {
            shape->setKind(CollisionShapeKind::ConcaveMesh);
            shape->setSourceMeshPath(path);
        }
        created.push_back(body);
    }
    if (created.empty())
    {
        error = "No mesh to build a collision from.";
        return false;
    }
    recordCreated(kStaticCollisionAction, created, selectionBefore);
    return true;
}

void HierarchyActions::requestStaticCollision()
{
    ct::String error;
    if (!createStaticCollision(error) && !error.empty()) showError(error);
}

void HierarchyActions::requestSaveAsPrefab()
{
    if (!canSaveAsPrefab()) return;
    rt::WidgetApp& app = rt::WidgetApp::instance();
    if (rt::InputBox* existing = prefabBox())
    {
        app.bringToFront(existing);
        return;
    }
    mPrefabTarget = topNodes()[0];
    rt::InputBox* box = app.addFloat<rt::InputBox>(kPrefabTitle, "Prefab name:");
    box->setText(mPrefabTarget->getName());
    box->setAcceptLabel("Save");
    const ct::Rc<Link> link = mLink;
    box->accepted.connect(
            [link](const ct::String& text)
            {
                if (link->owner) link->owner->prefabAccepted(text);
            });
}

void HierarchyActions::prefabAccepted(const ct::String& text)
{
    Node* target = mPrefabTarget;
    mPrefabTarget = nullptr;
    if (!target) return;
    ct::String error;
    if (!saveAsPrefab(target, text, error) && !error.empty()) showError(error);
}

bool HierarchyActions::saveAsPrefab(Node* node, const ct::String& name, ct::String& error)
{
    error = ct::String();
    if (!node || !canAddChild() || !mDocument.project().isOpen() || !usable(node)) return false;
    Node* parent = node->getParent();
    if (!parent || node == mDocument.scene())
    {
        error = kRootRefusal;
        return false;
    }
    if (foreign(node))
    {
        error = kForeignRefusal;
        return false;
    }
    ct::String stem = name;
    while (!stem.empty() && stem[stem.size() - 1] == ' ') stem.pop_back();
    if (stem.empty() || stem.contains('/') || stem.contains('\\') || stem.contains(':'))
    {
        error = "Enter a prefab name without folders.";
        return false;
    }

    const Project& project = mDocument.project();
    const ct::String relative = ct::String(kPrefabFolder) + "/" + stem + Prefab::kExtension;
    const ct::String folder = project.absolutePath(kPrefabFolder);
    if (!fs_create_directory(folder.c_str()))
    {
        error = ct::String("Could not create '") + folder + "'.";
        return false;
    }
    const ct::String absolute = project.absolutePath(relative);
    if (file_exists(absolute.c_str()))
    {
        error = ct::String("'") + relative + "' already exists.";
        return false;
    }

    SceneResult result;
    if (!Prefab::saveFromNode(node, absolute, result, nullptr))
    {
        error = result.describe();
        if (error.empty()) error = ct::String("Could not save '") + relative + "'.";
        return false;
    }

    const std::ptrdiff_t index = node->getIndex();
    const ct::Vector<Node*> selectionBefore = mSelection.selectedNodes();
    ct::Vector<Node*> olds;
    olds.push_back(node);
    EditorTreeEdits::Record record;
    record.before = EditorTreeEdits::capture(olds, true);

    Node* instance = nullptr;
    {
        EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
        mMadeEditable.clear();
        allowChildren(parent);
        instance = Prefab::instantiate(relative, parent, result);
        if (instance) parent->moveChild(instance, static_cast<std::size_t>(index));
    }
    if (!instance)
    {
        error = result.describe();
        if (error.empty()) error = ct::String("Could not instantiate '") + relative + "'.";
        if (mAssetsChanged) mAssetsChanged();
        return false;
    }

    ct::Vector<Node*> news;
    news.push_back(instance);
    EditorTreeEdits::Placement detachedInstance = EditorTreeEdits::capture(instance, true);
    detachedInstance.parent = nullptr;
    record.before.push_back(detachedInstance);
    record.after = EditorTreeEdits::capture(news, true);
    EditorTreeEdits::Placement detachedOld = EditorTreeEdits::capture(node, true);
    detachedOld.parent = nullptr;
    record.after.push_back(detachedOld);
    record.name = kSaveAsPrefabAction;
    record.selectionBefore = selectionBefore;
    record.selectionAfter = news;
    record.doReferences = news;
    record.undoReferences = olds;
    record.scene = mDocument.scene();
    record.editable = takeEditable();
    record.execute = true;
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
    if (mAssetsChanged) mAssetsChanged();
    return true;
}

bool HierarchyActions::reparentToNewNode(const ct::String& type, ct::String& error)
{
    error = ct::String();
    Node* scene = mDocument.scene();
    if (!scene) return false;
    if (rootSelected())
    {
        error = kRootRefusal;
        return false;
    }

    const ct::Vector<Node*> selection = topNodes();
    if (selection.empty()) return false;
    if (anyForeign(selection))
    {
        error = kForeignRefusal;
        return false;
    }
    if (!validType(type, error)) return false;

    bool onlyOneTopNode = true;

    Node* first = selection[0];
    int smallerPathToTop = depthOf(first, scene);
    Node* topNode = first;

    for (std::size_t i = 1; i < selection.size(); ++i)
    {
        Node* n = selection[i];
        const int pathLength = depthOf(n, scene);

        if (topNode != n)
        {
            if (smallerPathToTop > pathLength)
            {
                topNode = n;
                smallerPathToTop = pathLength;
                onlyOneTopNode = true;
            }
            else if (smallerPathToTop == pathLength)
            {
                if (onlyOneTopNode && topNode->getParent() != n->getParent())
                    onlyOneTopNode = false;
                if (onlyOneTopNode && shownIndex(n) < shownIndex(topNode)) topNode = n;
            }
        }
    }

    Node* parent = nullptr;
    std::ptrdiff_t originalPosition = -1;
    if (onlyOneTopNode)
    {
        parent = topNode->getParent();
        originalPosition = topNode->getIndex();
    }
    else
    {
        parent = topNode->getParent() ? topNode->getParent()->getParent() : nullptr;
    }
    if (!parent) return false;

    const ct::Vector<Node*> keepSelection = mSelection.selectedNodes();

    EditorUndoRedoManager::Applying applying(&mDocument.undoRedo());
    mMadeEditable.clear();
    Node* lastCreated = createUnder(parent, type);
    if (!lastCreated) return false;
    ct::Vector<Node*> madeEditable = takeEditable();

    ReparentOutcome outcome;
    if (!doReparent(lastCreated, selection, keepSelection, true, error, outcome))
    {
        lastCreated->queueFree();
        return false;
    }
    const ct::Vector<Node*> reparentEditable = takeEditable();
    for (std::size_t i = 0; i < reparentEditable.size(); ++i)
        madeEditable.push_back(reparentEditable[i]);

    if (onlyOneTopNode) parent->moveChild(lastCreated, static_cast<std::size_t>(originalPosition));

    const EditorTreeEdits::Placement created = EditorTreeEdits::capture(lastCreated, true);
    EditorTreeEdits::Placement createdBefore = created;
    createdBefore.parent = nullptr;

    EditorTreeEdits::Record record;
    record.name = kReparentToNewAction;
    record.before.push_back(createdBefore);
    for (std::size_t i = 0; i < outcome.before.size(); ++i)
        record.before.push_back(outcome.before[i]);
    record.after.push_back(created);
    for (std::size_t i = 0; i < outcome.after.size(); ++i) record.after.push_back(outcome.after[i]);
    record.properties = outcome.changes;
    record.selectionBefore = keepSelection;
    record.selectionAfter = keepSelection;
    record.doReferences.push_back(lastCreated);
    record.scene = mDocument.scene();
    record.editable = madeEditable;
    record.execute = false;
    EditorTreeEdits::record(mDocument.undoRedo(), &mSelection, record);
    return true;
}

void HierarchyActions::openCreateDialog(CreateMode mode)
{
    rt::WidgetApp& app = rt::WidgetApp::instance();
    mCreateMode = mode;
    if (CreateNodeDialog* existing = createDialog())
    {
        app.bringToFront(existing);
        return;
    }

    CreateNodeDialog* box = app.addFloat<CreateNodeDialog>(mDocument.settings(), mHierarchy.icons(),
            preferredType(), mCreateSearch);
    const ct::Rc<Link> link = mLink;
    box->setPersistHandler(
            [link]
            {
                if (!link->owner) return;
                EditorDocument& document = link->owner->mDocument;
                if (!document.settingsPath().empty())
                    document.settings().save(document.settingsPath());
            });
    box->closedWith.connect(
            [link](const ct::String& search)
            {
                if (link->owner) link->owner->mCreateSearch = search;
            });
    box->created.connect(
            [link](const ct::String& type)
            {
                if (link->owner) link->owner->createAccepted(type);
            });
}

void HierarchyActions::createAccepted(const ct::String& type)
{
    ct::String error;
    if (mCreateMode == kCreateChild) createNode(type, error);
    else
        reparentToNewNode(type, error);
    if (!error.empty()) showError(error);
}

void HierarchyActions::requestAddChild()
{
    if (!canAddChild()) return;
    openCreateDialog(kCreateChild);
}

void HierarchyActions::requestReparentToNew()
{
    if (!canAddChild() || mSelection.selectedNodes().empty()) return;
    if (rootSelected())
    {
        showError(kRootRefusal);
        return;
    }
    const ct::Vector<Node*> nodes = topNodes();
    if (nodes.empty()) return;
    if (anyForeign(nodes))
    {
        showError(kForeignRefusal);
        return;
    }
    openCreateDialog(kCreateReparent);
}

void HierarchyActions::requestReparent()
{
    if (!canAddChild() || mSelection.selectedNodes().empty()) return;
    if (rootSelected())
    {
        showError(kRootRefusal);
        return;
    }
    const ct::Vector<Node*> nodes = topNodes();
    if (nodes.empty()) return;
    if (anyForeign(nodes))
    {
        showError(kForeignRefusal);
        return;
    }

    if (ReparentDialog* existing = reparentDialog()) existing->cancel();

    rt::WidgetApp& app = rt::WidgetApp::instance();
    ReparentDialog* box = app.addFloat<ReparentDialog>(static_cast<const Node*>(mDocument.scene()),
            mHierarchy.icons(), nodes);
    const ct::Rc<Link> link = mLink;
    box->reparentRequested.connect(
            [link](const ct::String& path, bool keep)
            {
                if (link->owner) link->owner->reparentAccepted(path, keep);
            });
}

void HierarchyActions::reparentAccepted(const ct::String& path, bool keepGlobalTransform)
{
    Node* scene = mDocument.scene();
    if (!scene) return;
    Node* newParent = scene->getNode(path);
    if (!newParent) return;

    ct::String error;
    reparentSelection(newParent, keepGlobalTransform, error);
    if (!error.empty()) showError(error);
}

bool HierarchyActions::textFocused() const
{
    return WidgetHost::textFocused();
}

bool HierarchyActions::buttonFocused() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return app && rt::widget_cast<rt::Button>(app->focused()) != nullptr;
}

bool HierarchyActions::overlayOpen() const
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return false;
    if (app->popup() != nullptr) return true;
    const rt::Widget* own = createDialog();
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = 0; i < floats.size(); ++i)
        if (floats[i]->isVisible() && floats[i] != own) return true;
    return false;
}

bool HierarchyActions::pointerOnOutliner(float x, float y) const
{
    rt::Outliner* outliner = mHierarchy.outliner();
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!outliner || !app || app->popup() || !visibleInTree(outliner)) return false;
    if (!outliner->absoluteRect().contains(x, y)) return false;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = 0; i < floats.size(); ++i)
        if (floats[i]->isVisible() && floats[i]->absoluteRect().contains(x, y)) return false;
    return true;
}

bool HierarchyActions::actionKey(const InputEvent& event)
{
    const unsigned chord = event.key.modifiers & kChordMask;
    const Key key = event.key.key;

    Action action = kActionCount;
    bool confirm = true;
    bool repeats = false;
    if (key == Key::A && chord == kModControl) action = kAddChild;
    else if (key == Key::F2 && chord == 0)
        action = kRename;
    else if (key == Key::D && chord == kModControl)
    {
        action = kDuplicate;
        repeats = true;
    }
    else if (key == Key::Up && chord == kModControl)
    {
        action = kMoveUp;
        repeats = true;
    }
    else if (key == Key::Down && chord == kModControl)
    {
        action = kMoveDown;
        repeats = true;
    }
    else if (key == Key::Delete && (chord == 0 || chord == kModShift))
    {
        action = kDelete;
        confirm = chord == 0;
    }
    if (action == kActionCount) return false;

    if (event.key.echo && !repeats) return false;
    if (overlayOpen() || textFocused()) return false;
    if (action == kRename && buttonFocused()) return false;
    if (!canAddChild()) return false;
    if (action != kAddChild && mSelection.selectedNodes().empty()) return false;

    if (action == kDelete) requestDelete(confirm);
    else
        run(action);
    return true;
}

bool HierarchyActions::handleEvent(const InputEvent& event)
{
    if (event.type == InputEventType::MouseButton)
    {
        if (event.mouseButton.pressed && event.mouseButton.button == MouseButton::Right)
        {
            const float x = static_cast<float>(event.mouseButton.x);
            const float y = static_cast<float>(event.mouseButton.y);
            if (pointerOnOutliner(x, y)) mHierarchy.selectRowAt(x, y);
        }
        return false;
    }
    if (event.type != InputEventType::Key || !event.key.pressed) return false;
    return actionKey(event);
}

} // namespace prisma3d
