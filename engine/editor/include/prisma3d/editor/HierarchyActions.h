#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/editor/CreateMenu.h"
#include "prisma3d/editor/CreateNodeDialog.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/NodeNaming.h"
#include "prisma3d/editor/ReparentDialog.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/import/PMeshWriter.h"
#include "prisma3d/scene/SceneTreeListener.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/MenuWidgets.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class EditorViewportPanel;
class MeshInstance3D;
class Node;
class SceneTree;

class HierarchyActions : private SceneTreeListener
{
public:
    static constexpr const char* kRootRefusal = "This operation can't be done on the tree root.";
    static constexpr const char* kForeignRefusal = "Can't operate on nodes from a foreign scene!";
    static constexpr const char* kInvalidNameMessage = NodeNaming::kInvalidNameMessage;
    static constexpr const char* kSelectionChanged =
            "Selection changed at some point. Can't reparent.";
    static constexpr const char* kAddTooltip = "Add/Create a New Node. (Ctrl+A)";
    static constexpr const char* kDuplicateTooltip = "Duplicate Node (Ctrl+D)";
    static constexpr const char* kDeleteTooltip = "Delete Node (Del)";
    static constexpr const char* kAddAction = "Add Node";

    enum Action : int
    {
        kAddChild,
        kDuplicate,
        kRename,
        kMoveUp,
        kMoveDown,
        kReparent,
        kReparentToNew,
        kSaveAsPrefab,
        kStaticCollision,
        kDelete,
        kActionCount
    };

    enum MenuItem : int
    {
        kItemAddChild,
        kItemSeparator1,
        kItemDuplicate,
        kItemSeparator2,
        kItemRename,
        kItemSeparator3,
        kItemMoveUp,
        kItemMoveDown,
        kItemReparent,
        kItemReparentToNew,
        kItemSaveAsPrefab,
        kItemStaticCollision,
        kItemSeparator4,
        kItemDelete,
        kItemCount
    };

    static constexpr int kContextOffset = 2;

    HierarchyActions(SceneTree& tree, EditorDocument& document, EditorSelection& selection,
            SceneTreePanel& hierarchy);
    ~HierarchyActions() override;
    HierarchyActions(const HierarchyActions&) = delete;
    HierarchyActions& operator=(const HierarchyActions&) = delete;

    bool attach();
    bool attached() const;
    void update();

    void setViewport(EditorViewportPanel* viewport);
    Math::Vec3 focusPoint() const;
    CreateMenu& createMenu();

    bool rename(Node* node, const ct::String& requested, ct::String& error);
    bool splitMesh(Node* node, const ct::String& meshPath, const ct::Vector<ct::String>& pieces);
    bool rewriteMesh(Node* node, const ct::String& file, const ct::String& absolute,
            const PMeshSurfaceEdit& edit, ct::String& error);
    bool duplicateSelection(ct::String& error);
    bool deleteSelection(ct::String& error);
    bool moveSelection(bool down, ct::String& error);
    bool createNode(const ct::String& type, ct::String& error);
    bool createNodeUnder(const ct::String& type, Node* parent, ct::String& error);
    bool reparentSelection(Node* newParent, bool keepGlobalTransform, ct::String& error);
    bool reparentToNewNode(const ct::String& type, ct::String& error);
    bool selectEnvironment(ct::String& error);
    Node* findEnvironment() const;
    bool saveAsPrefab(Node* node, const ct::String& name, ct::String& error);
    bool createStaticCollision(ct::String& error);
    void focusNode(Node* node);

    enum DropPlace : int
    {
        kDropBefore,
        kDropInside,
        kDropAfter
    };

    bool canDragNode(Node* node) const;
    bool canDropNode(Node* dragged, Node* target, DropPlace place) const;
    bool dropNode(Node* dragged, Node* target, DropPlace place, ct::String& error);

    bool canAddChild() const;
    bool canReparent() const;
    bool canReparentToNew() const;
    bool canSaveAsPrefab() const;
    bool canCreateStaticCollision() const;
    bool canRename() const;
    bool canDuplicate() const;
    bool canDelete() const;
    bool canMoveUp() const;
    bool canMoveDown() const;
    bool can(Action action) const;

    void run(Action action);
    void requestRename();
    void requestDuplicate();
    void requestDelete(bool confirm);
    void requestMove(bool down);
    void requestAddChild();
    void requestReparent();
    void requestReparentToNew();
    void requestSaveAsPrefab();
    void requestStaticCollision();
    void requestEnvironment();

    ct::String preferredType() const;
    int addButtonId() const;
    int duplicateButtonId() const;
    int deleteButtonId() const;

    ct::String deleteMessage() const;

    void allowChildren(Node* parent);
    ct::Vector<Node*> takeEditable();
    void recordCreated(const ct::String& name, const ct::Vector<Node*>& created,
            const ct::Vector<Node*>& selectionBefore);

    bool handleEvent(const InputEvent& event);

    void populate(ig::retained::Menu* menu);
    void populateContext(ig::retained::Menu* menu);
    void populateCreate(ig::retained::Menu* menu);
    void refresh(ig::retained::Menu* menu) const;
    void refreshCreate(ig::retained::Menu* menu) const;
    void showAddMenu();

    static const char* label(Action action);
    static const char* shortcut(Action action);

    ig::retained::Menu* contextMenu() const;
    ig::retained::InputBox* renameBox() const;
    ig::retained::Dialog* dialog() const;
    CreateNodeDialog* createDialog() const;
    ReparentDialog* reparentDialog() const;
    ig::retained::InputBox* prefabBox() const;
    void setAssetsChangedHandler(ct::Function<void()> handler);

private:
    friend class CreateMenu;

    enum CreateMode : int
    {
        kCreateChild,
        kCreateReparent
    };

    struct Link
    {
        HierarchyActions* owner;
    };

    void nodeRemoved(Node* node) override;

    bool usable(const Node* node) const;
    ct::Vector<Node*> topNodes() const;
    ct::Vector<Node*> allNodes(bool sorted = true) const;
    bool anyUsable() const;
    bool rootSelected() const;
    bool foreign(const Node* node) const;
    bool anyForeign(const ct::Vector<Node*>& nodes) const;
    bool moveBlocked(const ct::Vector<Node*>& nodes, bool down) const;
    bool canMove(bool down) const;
    bool actionKey(const InputEvent& event);
    bool textFocused() const;
    bool buttonFocused() const;
    bool overlayOpen() const;
    bool pointerOnOutliner(float x, float y) const;
    void showError(const ct::String& message);
    void renameAccepted(const ct::String& text);
    void openDeleteDialog(const ct::String& message);
    void confirmDelete();
    void openCreateDialog(CreateMode mode);
    void createAccepted(const ct::String& type);
    void reparentAccepted(const ct::String& path, bool keepGlobalTransform);
    void prefabAccepted(const ct::String& text);
    Node* createUnder(Node* parent, const ct::String& type);
    ct::Vector<Node*> createMeshPieces(MeshInstance3D* instance,
            const ct::Vector<ct::String>& pieces);
    ct::Vector<Node*> createParents() const;
    bool createAmong(const ct::String& type, const ct::Vector<Node*>& parents);
    bool validType(const ct::String& type, ct::String& error) const;
    struct ReparentOutcome
    {
        ct::Vector<EditorTreeEdits::Placement> before;
        ct::Vector<EditorTreeEdits::Placement> after;
        ct::Vector<EditorTreeEdits::PropertyChange> changes;
    };

    bool doReparent(Node* newParent, const ct::Vector<Node*>& nodes,
            const ct::Vector<Node*>& keepSelection, bool keepGlobalTransform, ct::String& error,
            ReparentOutcome& outcome, Node* before = nullptr);
    ct::Vector<Node*> draggedNodes(Node* node) const;
    Node* dropParent(const ct::Vector<Node*>& nodes, Node* target, DropPlace place,
            Node*& before) const;

    SceneTree& mTree;
    EditorDocument& mDocument;
    EditorSelection& mSelection;
    SceneTreePanel& mHierarchy;
    ct::Rc<Link> mLink;
    EditorViewportPanel* mViewport;
    ct::Unique<CreateMenu> mCreate;
    ig::retained::Menu* mContextMenu;
    Node* mRenameTarget;
    Node* mPrefabTarget;
    ct::String mCreateSearch;
    ct::Vector<Node*> mMadeEditable;
    CreateMode mCreateMode;
    int mAddButton;
    int mDuplicateButton;
    int mDeleteButton;
    int mCollapseButton;
    int mExpandButton;
    ct::Function<void()> mAssetsChanged;
};

} // namespace prisma3d
