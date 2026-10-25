#pragma once

#include "prisma3d/scene/PropertyValue.h"

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Node;

class UndoRedo
{
public:
    enum MergeMode : int
    {
        kMergeDisable,
        kMergeEnds,
        kMergeAll
    };

    static constexpr std::uint64_t kMergeWindowMs = 800;

    using Callable = ct::Function<void()>;
    using CommitNotify = ct::Function<void(const ct::String&)>;
    using VersionNotify = ct::Function<void()>;
    using Clock = ct::Function<std::uint64_t()>;
    using Deleter = ct::Function<void(Node*)>;

    UndoRedo();
    ~UndoRedo();
    UndoRedo(const UndoRedo&) = delete;
    UndoRedo& operator=(const UndoRedo&) = delete;

    void createAction(const ct::String& name = ct::String(), MergeMode mode = kMergeDisable,
            bool backwardUndoOps = false);

    void addDoMethod(const Callable& callable, Node* object = nullptr);
    void addUndoMethod(const Callable& callable, Node* object = nullptr);
    void addDoProperty(Node* object, const ct::String& property, const PropertyValue& value);
    void addUndoProperty(Node* object, const ct::String& property, const PropertyValue& value);
    void addDoReference(Node* object);
    void addUndoReference(Node* object);

    void startForceKeepInMergeEnds();
    void endForceKeepInMergeEnds();

    bool isCommittingAction() const;
    void commitAction(bool execute = true);

    bool redo();
    bool undo();
    ct::String getCurrentActionName() const;
    ct::String getNextActionName() const;
    int getActionLevel() const;

    int getHistoryCount() const;
    int getCurrentAction() const;
    ct::String getActionName(int id) const;
    void clearHistory(bool increaseVersion = true);
    void discardRedo();

    bool hasUndo() const;
    bool hasRedo() const;
    bool isMerging() const;
    std::uint64_t getVersion() const;

    void setMaxSteps(int maxSteps);
    int getMaxSteps() const;

    void setCommitNotifyCallback(const CommitNotify& callback);
    void setVersionChangedCallback(const VersionNotify& callback);
    void setClock(const Clock& clock);
    void setDeleter(const Deleter& deleter);

private:
    struct Operation
    {
        enum Type : int
        {
            kMethod,
            kProperty,
            kReference
        };

        Type type = kMethod;
        bool forceKeepInMergeEnds = false;
        Node* object = nullptr;
        ct::String name;
        Callable callable;
        PropertyValue value;
    };

    struct Action
    {
        ct::String name;
        ct::Vector<Operation> doOps;
        ct::Vector<Operation> undoOps;
        std::uint64_t lastTick = 0;
        bool backwardUndoOps = false;
    };

    void deleteReference(Operation& operation);
    void popHistoryTail();
    void processOperationList(const ct::Vector<Operation>& operations, std::size_t first,
            bool execute);
    bool redoAction(bool execute);
    bool pending() const;
    std::uint64_t now() const;
    static void reverseOperations(ct::Vector<Operation>& operations);

    ct::Vector<Action> mActions;
    int mCurrentAction;
    bool mForceKeepInMergeEnds;
    int mActionLevel;
    int mMaxSteps;
    MergeMode mMergeMode;
    bool mMerging;
    std::uint64_t mVersion;
    std::size_t mMergeTotal;
    int mCommitting;
    CommitNotify mCommitNotify;
    VersionNotify mVersionChanged;
    Clock mClock;
    Deleter mDeleter;
};

} // namespace prisma3d
