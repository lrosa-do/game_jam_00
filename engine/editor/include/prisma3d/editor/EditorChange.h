#pragma once

namespace prisma3d
{

class EditorUndoRedoManager;

class EditorChangeListener
{
public:
    virtual ~EditorChangeListener()
    {
    }
    virtual void sceneEdited()
    {
    }
    virtual EditorUndoRedoManager* undoRedoManager()
    {
        return nullptr;
    }
};

} // namespace prisma3d
