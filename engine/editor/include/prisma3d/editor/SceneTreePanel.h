#pragma once

#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/scene/SceneTreeListener.h"

#include <igui/widgets/AppWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/ToolBar.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>

#include <ct/hashmap.hpp>
#include <ct/ptr.hpp>

#include <cstdint>

namespace prisma3d
{

class Node;
class SceneTree;

class SceneTreePanel : private SceneTreeListener, private EditorSelectionListener
{
public:
    SceneTreePanel(SceneTree& tree, EditorSelection& selection);
    ~SceneTreePanel() override;
    SceneTreePanel(const SceneTreePanel&) = delete;
    SceneTreePanel& operator=(const SceneTreePanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    ig::retained::Outliner* outliner() const;
    ig::retained::ToolBar* toolBar() const;

    ClassIcons& icons();
    const ClassIcons& icons() const;

    void setEditedScene(Node* scene);
    Node* editedScene() const;
    void setChangeListener(EditorChangeListener* listener);

    void update();
    bool dirty() const;
    void setAllExpanded(bool expanded);
    unsigned int rebuildCount() const;

    ig::retained::TreeNode* itemOf(const Node* node) const;
    Node* nodeOf(ig::retained::TreeNode* item) const;
    bool selectRowAt(float x, float y);

    ig::retained::Signal<Node*> nodeActivated;
    Node* nodeAt(float x, float y);

private:
    struct Link
    {
        SceneTreePanel* owner;
    };

    void treeChanged() override;
    void nodeAdded(Node* node) override;
    void nodeRemoved(Node* node) override;
    void nodeRenamed(Node* node) override;
    void selectionChanged() override;

    bool inScene(const Node* node) const;
    std::uint64_t hashScene() const;
    void rebuild();
    void fillItem(Node* node, ig::retained::TreeNode* item);
    void addNodes(Node* node, ig::retained::TreeNode* item);
    void applySelection(bool expand);
    void selectItem(ig::retained::TreeNode* item);
    void itemSelected(ig::retained::TreeNode* item);
    void itemActivated(ig::retained::TreeNode* item);
    void visibilityToggled(ig::retained::TreeNode* item);
    void toggleVisible(Node* node);
    void refreshVisible(Node* node);
    void syncVisibility();

    SceneTree& mTree;
    EditorSelection& mSelection;
    ClassIcons mIcons;
    ig::retained::BoxLayout* mColumn;
    ig::retained::ToolBar* mToolBar;
    ig::retained::Outliner* mOutliner;
    ct::Rc<Link> mLink;
    Node* mScene;
    EditorChangeListener* mListener;
    ct::HashMap<Node*, ig::retained::TreeNode*> mItemOf;
    ct::HashMap<ig::retained::TreeNode*, Node*> mNodeOf;
    ct::HashMap<Node*, bool> mExpanded;
    std::uint64_t mLastHash;
    unsigned int mRebuilds;
    bool mDirty;
    // The selection changed while the tree waited for a rebuild: reveal it after the rebuild.
    bool mRevealSelection;
    bool mPendingTest;
    bool mSyncing;
};

} // namespace prisma3d
