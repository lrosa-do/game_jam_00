#include "prisma3d/editor/EditorSelection.h"

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/SceneTree.h"

namespace prisma3d
{

EditorSelection::EditorSelection(SceneTree& tree)
        : mTree(tree),
          mEmitDepth(0),
          mListenersDirty(false),
          mEmitted(false),
          mChanged(false)
{
    mTree.addListener(this);
}

EditorSelection::~EditorSelection()
{
    mTree.removeListener(this);
    clear();
}

bool EditorSelection::add(Node* node)
{
    if (!node || !node->isInsideTree()) return false;
    if (isSelected(node)) return true;

    mChanged = true;
    mSelection.push_back(node);
    return true;
}

void EditorSelection::remove(Node* node)
{
    for (std::size_t i = 0; i < mSelection.size(); ++i)
    {
        if (mSelection[i] != node) continue;
        mChanged = true;
        mSelection.erase(mSelection.begin() + i);
        return;
    }
}

bool EditorSelection::isSelected(const Node* node) const
{
    if (!node) return false;
    for (std::size_t i = 0; i < mSelection.size(); ++i)
        if (mSelection[i] == node) return true;
    return false;
}

void EditorSelection::clear()
{
    mSelection.clear();
    mChanged = true;
}

const ct::Vector<Node*>& EditorSelection::selectedNodes() const
{
    return mSelection;
}

ct::Vector<Node*> EditorSelection::topSelectedNodes() const
{
    ct::Vector<Node*> top;
    for (std::size_t i = 0; i < mSelection.size(); ++i)
    {
        bool skip = false;
        for (const Node* parent = mSelection[i]->getParent(); parent && !skip;
                parent = parent->getParent())
            skip = isSelected(parent);
        if (!skip) top.push_back(mSelection[i]);
    }
    return top;
}

void EditorSelection::nodeRemoved(Node* node)
{
    for (std::size_t i = 0; i < mSelection.size(); ++i)
    {
        if (mSelection[i] != node) continue;
        mSelection.erase(mSelection.begin() + i);
        mChanged = true;
        return;
    }
}

void EditorSelection::update(bool deferred)
{
    if (!mChanged) return;
    mChanged = false;
    if (deferred) mEmitted = true;
    else
        emitChange();
}

void EditorSelection::flush()
{
    if (!mEmitted) return;
    emitChange();
}

bool EditorSelection::changePending() const
{
    return mChanged || mEmitted;
}

void EditorSelection::emitChange()
{
    mEmitted = false;
    const std::size_t count = mListeners.size();
    ++mEmitDepth;
    for (std::size_t i = 0; i < count; ++i)
        if (EditorSelectionListener* listener = mListeners[i]) listener->selectionChanged();
    endEmit();
}

void EditorSelection::endEmit()
{
    if (--mEmitDepth > 0 || !mListenersDirty) return;
    mListenersDirty = false;
    std::size_t kept = 0;
    for (std::size_t i = 0; i < mListeners.size(); ++i)
        if (mListeners[i]) mListeners[kept++] = mListeners[i];
    mListeners.resize(kept);
}

void EditorSelection::addListener(EditorSelectionListener* listener)
{
    if (!listener) return;
    for (std::size_t i = 0; i < mListeners.size(); ++i)
        if (mListeners[i] == listener) return;
    mListeners.push_back(listener);
}

void EditorSelection::removeListener(EditorSelectionListener* listener)
{
    for (std::size_t i = 0; i < mListeners.size(); ++i)
    {
        if (mListeners[i] != listener) continue;
        if (mEmitDepth > 0)
        {
            mListeners[i] = nullptr;
            mListenersDirty = true;
        }
        else
            mListeners.erase(mListeners.begin() + i);
        return;
    }
}

} // namespace prisma3d
