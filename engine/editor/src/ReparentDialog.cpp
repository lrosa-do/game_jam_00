#include "prisma3d/editor/ReparentDialog.h"

#include "prisma3d/scene/Node.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Retained.hpp>
#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

ReparentDialog::ReparentDialog(const Node* scene, const ClassIcons& icons,
        const ct::Vector<Node*>& moving)
        : rt::FloatWindow(kTitle),
          mScene(scene),
          mIcons(icons),
          mTree(nullptr),
          mKeep(nullptr),
          mAccept(nullptr),
          mCancel(nullptr),
          mPicked(nullptr),
          mRefused(0),
          mReverting(false),
          mClosed(false)
{
    for (std::size_t i = 0; i < moving.size(); ++i) mMoving.push_back(moving[i]);

    setMinimizable(false);

    rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
    main->setSpacing(8.0f);
    main->setPadding(10.0f);

    main->createChild<rt::Label>(kPrompt);
    mTree = main->createChild<ActivatableTree>();
    mTree->setStretch(1.0f);
    mKeep = main->createChild<rt::CheckBox>(kKeepText);
    mKeep->setChecked(true);

    rt::BoxLayout* buttons = main->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    buttons->setPadding(0.0f);
    buttons->setSpacing(8.0f);
    buttons->setMainAlign(rt::MainAlign::End);
    mCancel = buttons->createChild<rt::Button>("Cancel");
    mCancel->setSize(90.0f, 0.0f);
    mAccept = buttons->createChild<rt::Button>(kAcceptText);
    mAccept->setSize(90.0f, 0.0f);

    if (mScene)
    {
        rt::TreeNode* root = mTree->addRoot(mScene->getName());
        addNodes(mScene, root);
    }

    mTree->selectionChanged.connect([this](rt::TreeNode* item) { itemSelected(item); });
    mTree->activated.connect([this](rt::TreeNode*) { accept(); });
    mAccept->clicked.connect([this] { accept(); });
    mCancel->clicked.connect([this] { cancel(); });
    closed.connect([this] { cancel(); });

    const float width = 350.0f;
    const float height = 560.0f;
    setFloatSize(width, height);
    const rt::IO& io = rt::GetIO();
    setFloatPos((io.displayWidth - width) * 0.5f, (io.displayHeight - height) * 0.5f);
}

void ReparentDialog::addNodes(const Node* node, rt::TreeNode* item)
{
    item->setIconId(mIcons.icon(node));
    mItemOf.put(node, item);
    mNodeOf.put(item, node);
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        const Node* child = node->getChild(i);
        if (child->isInternal()) continue;
        addNodes(child, item->addChild(child->getName()));
    }
}

bool ReparentDialog::canPick(const Node* node) const
{
    if (!node || !mScene) return false;
    if (node != mScene && !mScene->isAncestorOf(node)) return false;
    for (std::size_t i = 0; i < mMoving.size(); ++i)
        if (node == mMoving[i] || mMoving[i]->isAncestorOf(node)) return false;
    return true;
}

bool ReparentDialog::pick(const Node* node)
{
    rt::TreeNode* const* item = mItemOf.find(node);
    if (!item) return false;
    mTree->setSelectedNode(*item);
    return mPicked == *item;
}

const Node* ReparentDialog::picked() const
{
    if (!mPicked) return nullptr;
    const Node* const* node = mNodeOf.find(mPicked);
    return node ? *node : nullptr;
}

bool ReparentDialog::tracks(const Node* node) const
{
    return mItemOf.contains(node);
}

unsigned int ReparentDialog::refusedCount() const
{
    return mRefused;
}

rt::TreeNode* ReparentDialog::itemOf(const Node* node) const
{
    rt::TreeNode* const* item = mItemOf.find(node);
    return item ? *item : nullptr;
}

void ReparentDialog::itemSelected(rt::TreeNode* item)
{
    if (mReverting) return;
    if (!item)
    {
        mPicked = nullptr;
        return;
    }
    const Node* const* node = mNodeOf.find(item);
    if (node && canPick(*node))
    {
        mPicked = item;
        return;
    }
    ++mRefused;
    mReverting = true;
    mTree->setSelectedNode(mPicked);
    mReverting = false;
}

void ReparentDialog::accept()
{
    if (mClosed || !mPicked || !mScene) return;
    const Node* node = picked();
    if (!node) return;
    const ct::String path = mScene->getPathTo(node);
    if (path.empty()) return;
    const bool keep = mKeep->isChecked();
    close();
    reparentRequested.emit(path, keep);
}

void ReparentDialog::cancel()
{
    if (mClosed) return;
    close();
}

void ReparentDialog::close()
{
    mClosed = true;
    rt::WidgetApp::instance().removeFloat(this);
}

ActivatableTree* ReparentDialog::tree() const
{
    return mTree;
}

rt::CheckBox* ReparentDialog::keepTransformBox() const
{
    return mKeep;
}

rt::Button* ReparentDialog::reparentButton() const
{
    return mAccept;
}

rt::Button* ReparentDialog::cancelButton() const
{
    return mCancel;
}

ig::retained::Widget* ReparentDialog::focusOnOpen()
{
    return mTree;
}

void ReparentDialog::onKeyPress(rt::KeyEvent& event)
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
        accept();
        return;
    }
    rt::FloatWindow::onKeyPress(event);
}

} // namespace prisma3d
