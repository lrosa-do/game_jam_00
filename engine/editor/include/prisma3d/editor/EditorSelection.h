#pragma once

#include "prisma3d/scene/SceneTreeListener.h"

#include <ct/vector.hpp>

namespace prisma3d
{

class Node;
class SceneTree;

class EditorSelectionListener
{
public:
    virtual ~EditorSelectionListener()
    {
    }
    virtual void selectionChanged()
    {
    }
};

class EditorSelection : private SceneTreeListener
{
public:
    explicit EditorSelection(SceneTree& tree);
    ~EditorSelection() override;
    EditorSelection(const EditorSelection&) = delete;
    EditorSelection& operator=(const EditorSelection&) = delete;

    bool add(Node* node);
    void remove(Node* node);
    bool isSelected(const Node* node) const;
    void clear();

    const ct::Vector<Node*>& selectedNodes() const;
    ct::Vector<Node*> topSelectedNodes() const;

    void update(bool deferred = true);
    void flush();
    bool changePending() const;

    void addListener(EditorSelectionListener* listener);
    void removeListener(EditorSelectionListener* listener);

private:
    void nodeRemoved(Node* node) override;
    void emitChange();
    void endEmit();

    SceneTree& mTree;
    ct::Vector<Node*> mSelection;
    ct::Vector<EditorSelectionListener*> mListeners;
    int mEmitDepth;
    bool mListenersDirty;
    bool mEmitted;
    bool mChanged;
};

} // namespace prisma3d
