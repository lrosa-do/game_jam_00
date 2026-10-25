#pragma once

#include "prisma3d/editor/UndoRedo.h"

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorUndoRedoManager
{
public:
    static constexpr std::uint64_t kUnsavedVersion = 0;

    struct Action
    {
        double timestamp = 0.0;
        ct::String name;
        UndoRedo::MergeMode mergeMode = UndoRedo::kMergeDisable;
        bool backwardUndoOps = false;
        bool markUnsaved = true;
    };

    class Applying
    {
    public:
        explicit Applying(EditorUndoRedoManager* manager);
        ~Applying();
        Applying(const Applying&) = delete;
        Applying& operator=(const Applying&) = delete;

    private:
        EditorUndoRedoManager* mManager;
    };

    EditorUndoRedoManager();
    EditorUndoRedoManager(const EditorUndoRedoManager&) = delete;
    EditorUndoRedoManager& operator=(const EditorUndoRedoManager&) = delete;

    UndoRedo& history();
    const UndoRedo& history() const;

    void setNamePrefix(const ct::String& prefix);
    const ct::String& namePrefix() const;

    void createAction(const ct::String& name = ct::String(),
            UndoRedo::MergeMode mode = UndoRedo::kMergeDisable, bool backwardUndoOps = false,
            bool markUnsaved = true);

    void createPropertyAction(const ct::String& name, Node* object);

    void addDoMethod(const UndoRedo::Callable& callable, Node* object = nullptr);
    void addUndoMethod(const UndoRedo::Callable& callable, Node* object = nullptr);
    void addDoProperty(Node* object, const ct::String& property, const PropertyValue& value);
    void addUndoProperty(Node* object, const ct::String& property, const PropertyValue& value);
    void addDoReference(Node* object);
    void addUndoReference(Node* object);

    void commitAction(bool execute = true);
    bool isCommittingAction() const;
    bool pendingAction() const;

    bool undo();
    bool redo();
    void clearHistory(bool increaseVersion = true);

    void setHistoryAsSaved();
    void setHistoryAsUnsaved();
    bool isHistoryUnsaved() const;

    bool hasUndo() const;
    bool hasRedo() const;
    ct::String currentActionName() const;
    ct::String redoActionName() const;
    std::size_t undoCount() const;
    std::size_t redoCount() const;

    void setFrozen(bool frozen);
    bool frozen() const;

    bool busy() const;
    std::uint64_t version() const;
    std::uint64_t savedVersion() const;

private:
    UndoRedo& active();
    const UndoRedo& active() const;
    double timestamp() const;

    UndoRedo mUndoRedo;
    UndoRedo mScratch;
    Node* mEditObject;
    ct::String mEditName;
    Action mPending;
    ct::String mNamePrefix;
    ct::Vector<Action> mUndoStack;
    ct::Vector<Action> mRedoStack;
    std::uint64_t mSavedVersion;
    int mBusy;
    bool mHasPending;
    bool mFrozen;
};

} // namespace prisma3d
