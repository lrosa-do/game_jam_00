#pragma once

#include "prisma3d/editor/ActivatableTree.h"
#include "prisma3d/editor/ClassIcons.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class Node;

class ReparentDialog : public ig::retained::FloatWindow
{
public:
    IG_WIDGET_CLASS(ReparentDialog, ig::retained::FloatWindow);

    static constexpr const char* kTitle = "Reparent Node";
    static constexpr const char* kPrompt = "Select new parent:";
    static constexpr const char* kKeepText = "Keep Global Transform";
    static constexpr const char* kAcceptText = "Reparent";

    ReparentDialog(const Node* scene, const ClassIcons& icons, const ct::Vector<Node*>& moving);

    bool canPick(const Node* node) const;
    bool pick(const Node* node);
    const Node* picked() const;
    bool tracks(const Node* node) const;
    unsigned int refusedCount() const;
    ig::retained::TreeNode* itemOf(const Node* node) const;

    void accept();
    void cancel();

    ActivatableTree* tree() const;
    ig::retained::CheckBox* keepTransformBox() const;
    ig::retained::Button* reparentButton() const;
    ig::retained::Button* cancelButton() const;

    ig::retained::Signal<const ct::String&, bool> reparentRequested;

    void onKeyPress(ig::retained::KeyEvent& event) override;
    ig::retained::Widget* focusOnOpen() override;

private:
    void addNodes(const Node* node, ig::retained::TreeNode* item);
    void itemSelected(ig::retained::TreeNode* item);
    void close();

    const Node* mScene;
    const ClassIcons& mIcons;
    ct::Vector<const Node*> mMoving;
    ct::HashMap<const Node*, ig::retained::TreeNode*> mItemOf;
    ct::HashMap<ig::retained::TreeNode*, const Node*> mNodeOf;
    ActivatableTree* mTree;
    ig::retained::CheckBox* mKeep;
    ig::retained::Button* mAccept;
    ig::retained::Button* mCancel;
    ig::retained::TreeNode* mPicked;
    unsigned int mRefused;
    bool mReverting;
    bool mClosed;
};

} // namespace prisma3d
