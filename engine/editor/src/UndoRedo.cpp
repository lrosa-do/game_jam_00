#include "prisma3d/editor/UndoRedo.h"

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/NodeClassDB.h"

#include "platform.h"

namespace prisma3d
{

UndoRedo::UndoRedo()
        : mCurrentAction(-1),
          mForceKeepInMergeEnds(false),
          mActionLevel(0),
          mMaxSteps(0),
          mMergeMode(kMergeDisable),
          mMerging(false),
          mVersion(1),
          mMergeTotal(0),
          mCommitting(0)
{
    mClock = []() -> std::uint64_t { return time_nanos() / 1000000ull; };
    mDeleter = [](Node* node)
    {
        if (node) node->queueFree();
    };
}

UndoRedo::~UndoRedo()
{
    clearHistory();
}

std::uint64_t UndoRedo::now() const
{
    return mClock ? mClock() : 0;
}

bool UndoRedo::pending() const
{
    return mActionLevel > 0 && static_cast<std::size_t>(mCurrentAction + 1) < mActions.size();
}

void UndoRedo::reverseOperations(ct::Vector<Operation>& operations)
{
    if (operations.size() < 2) return;
    for (std::size_t low = 0, high = operations.size() - 1; low < high; ++low, --high)
    {
        Operation temporary = operations[low];
        operations[low] = operations[high];
        operations[high] = temporary;
    }
}

void UndoRedo::deleteReference(Operation& operation)
{
    if (operation.type != Operation::kReference) return;
    if (mDeleter) mDeleter(operation.object);
    operation.object = nullptr;
}

void UndoRedo::discardRedo()
{
    if (mCurrentAction == static_cast<int>(mActions.size()) - 1) return;

    for (std::size_t i = static_cast<std::size_t>(mCurrentAction + 1); i < mActions.size(); ++i)
        for (std::size_t o = 0; o < mActions[i].doOps.size(); ++o)
            deleteReference(mActions[i].doOps[o]);

    mActions.resize(static_cast<std::size_t>(mCurrentAction + 1));
}

bool UndoRedo::redoAction(bool execute)
{
    if (mActionLevel > 0) return false;
    if (static_cast<std::size_t>(mCurrentAction + 1) >= mActions.size()) return false;

    ++mCurrentAction;

    std::size_t first = 0;
    const std::size_t count = mActions[static_cast<std::size_t>(mCurrentAction)].doOps.size();
    while (mMergeTotal > 0 && first < count)
    {
        ++first;
        --mMergeTotal;
    }

    processOperationList(mActions[static_cast<std::size_t>(mCurrentAction)].doOps, first, execute);
    ++mVersion;
    if (mVersionChanged) mVersionChanged();

    return true;
}

void UndoRedo::createAction(const ct::String& name, MergeMode mode, bool backwardUndoOps)
{
    const std::uint64_t ticks = now();

    if (mActionLevel == 0)
    {
        discardRedo();

        if (mode != kMergeDisable && !mActions.empty() && mActions.back().name == name &&
                mActions.back().backwardUndoOps == backwardUndoOps &&
                mActions.back().lastTick + kMergeWindowMs > ticks)
        {
            mCurrentAction = static_cast<int>(mActions.size()) - 2;
            Action& last = mActions.back();

            if (mode == kMergeEnds)
            {
                ct::Vector<Operation> kept;
                for (std::size_t i = 0; i < last.doOps.size(); ++i)
                {
                    if (last.doOps[i].forceKeepInMergeEnds) kept.push_back(last.doOps[i]);
                    else
                        deleteReference(last.doOps[i]);
                }
                last.doOps = kept;
            }

            if (mode == kMergeAll) mMergeTotal = last.doOps.size();
            else
                mMergeTotal = 0;

            last.lastTick = ticks;

            if (last.backwardUndoOps) reverseOperations(last.undoOps);

            mMergeMode = mode;
            mMerging = true;
        }
        else
        {
            Action action;
            action.name = name;
            action.lastTick = ticks;
            action.backwardUndoOps = backwardUndoOps;
            mActions.push_back(action);

            mMergeMode = kMergeDisable;
            mMergeTotal = 0;
        }
    }

    ++mActionLevel;

    mForceKeepInMergeEnds = false;
}

void UndoRedo::addDoMethod(const Callable& callable, Node* object)
{
    if (!callable || !pending()) return;

    Operation operation;
    operation.type = Operation::kMethod;
    operation.callable = callable;
    operation.object = object;
    mActions[static_cast<std::size_t>(mCurrentAction + 1)].doOps.push_back(operation);
}

void UndoRedo::addUndoMethod(const Callable& callable, Node* object)
{
    if (!callable || !pending()) return;

    if (!mForceKeepInMergeEnds && mMergeMode == kMergeEnds) return;

    Operation operation;
    operation.type = Operation::kMethod;
    operation.callable = callable;
    operation.object = object;
    operation.forceKeepInMergeEnds = mForceKeepInMergeEnds;
    mActions[static_cast<std::size_t>(mCurrentAction + 1)].undoOps.push_back(operation);
}

void UndoRedo::addDoProperty(Node* object, const ct::String& property, const PropertyValue& value)
{
    if (!object || !pending()) return;

    Operation operation;
    operation.type = Operation::kProperty;
    operation.object = object;
    operation.name = property;
    operation.value = value;
    mActions[static_cast<std::size_t>(mCurrentAction + 1)].doOps.push_back(operation);
}

void UndoRedo::addUndoProperty(Node* object, const ct::String& property, const PropertyValue& value)
{
    if (!object || !pending()) return;

    if (!mForceKeepInMergeEnds && mMergeMode == kMergeEnds) return;

    Operation operation;
    operation.type = Operation::kProperty;
    operation.object = object;
    operation.name = property;
    operation.value = value;
    operation.forceKeepInMergeEnds = mForceKeepInMergeEnds;
    mActions[static_cast<std::size_t>(mCurrentAction + 1)].undoOps.push_back(operation);
}

void UndoRedo::addDoReference(Node* object)
{
    if (!object || !pending()) return;

    Operation operation;
    operation.type = Operation::kReference;
    operation.object = object;
    mActions[static_cast<std::size_t>(mCurrentAction + 1)].doOps.push_back(operation);
}

void UndoRedo::addUndoReference(Node* object)
{
    if (!object || !pending()) return;

    if (!mForceKeepInMergeEnds && mMergeMode == kMergeEnds) return;

    Operation operation;
    operation.type = Operation::kReference;
    operation.object = object;
    operation.forceKeepInMergeEnds = mForceKeepInMergeEnds;
    mActions[static_cast<std::size_t>(mCurrentAction + 1)].undoOps.push_back(operation);
}

void UndoRedo::startForceKeepInMergeEnds()
{
    if (!pending()) return;
    mForceKeepInMergeEnds = true;
}

void UndoRedo::endForceKeepInMergeEnds()
{
    if (!pending()) return;
    mForceKeepInMergeEnds = false;
}

void UndoRedo::popHistoryTail()
{
    discardRedo();

    if (mActions.empty()) return;

    for (std::size_t i = 0; i < mActions[0].undoOps.size(); ++i)
        deleteReference(mActions[0].undoOps[i]);

    mActions.erase(mActions.begin());
    if (mCurrentAction >= 0) --mCurrentAction;
}

bool UndoRedo::isCommittingAction() const
{
    return mCommitting > 0;
}

void UndoRedo::commitAction(bool execute)
{
    if (mActionLevel <= 0) return;
    --mActionLevel;
    if (mActionLevel > 0) return;

    const bool addMessage = !mMerging;

    if (mMerging)
    {
        --mVersion;
        mMerging = false;
    }

    if (mActions.back().backwardUndoOps) reverseOperations(mActions.back().undoOps);

    ++mCommitting;
    redoAction(execute);
    --mCommitting;

    if (mMaxSteps > 0)
    {
        while (mActions.size() > static_cast<std::size_t>(mMaxSteps)) popHistoryTail();
    }

    if (addMessage && mCommitNotify && !mActions.empty()) mCommitNotify(mActions.back().name);
}

void UndoRedo::processOperationList(const ct::Vector<Operation>& operations, std::size_t first,
        bool execute)
{
    for (std::size_t i = first; i < operations.size(); ++i)
    {
        const Operation& operation = operations[i];
        switch (operation.type)
        {
            case Operation::kMethod:
                if (execute && operation.callable) operation.callable();
                break;
            case Operation::kProperty:
                if (execute && operation.object)
                    NodeClassDB::setProperty(operation.object, operation.name, operation.value);
                break;
            case Operation::kReference:
                break;
        }
    }
}

bool UndoRedo::redo()
{
    return redoAction(true);
}

bool UndoRedo::undo()
{
    if (mActionLevel > 0) return false;
    if (mCurrentAction < 0) return false;
    processOperationList(mActions[static_cast<std::size_t>(mCurrentAction)].undoOps, 0, true);
    --mCurrentAction;
    --mVersion;
    if (mVersionChanged) mVersionChanged();

    return true;
}

int UndoRedo::getHistoryCount() const
{
    if (mActionLevel > 0) return -1;
    return static_cast<int>(mActions.size());
}

int UndoRedo::getCurrentAction() const
{
    if (mActionLevel > 0) return -1;
    return mCurrentAction;
}

ct::String UndoRedo::getActionName(int id) const
{
    if (id < 0 || id >= static_cast<int>(mActions.size())) return ct::String();
    return mActions[static_cast<std::size_t>(id)].name;
}

void UndoRedo::clearHistory(bool increaseVersion)
{
    if (mActionLevel > 0) return;
    discardRedo();

    while (!mActions.empty()) popHistoryTail();

    if (increaseVersion)
    {
        ++mVersion;
        if (mVersionChanged) mVersionChanged();
    }
}

ct::String UndoRedo::getCurrentActionName() const
{
    if (mActionLevel > 0 || mCurrentAction < 0) return ct::String();
    return mActions[static_cast<std::size_t>(mCurrentAction)].name;
}

ct::String UndoRedo::getNextActionName() const
{
    if (mActionLevel > 0 || !hasRedo()) return ct::String();
    return mActions[static_cast<std::size_t>(mCurrentAction + 1)].name;
}

int UndoRedo::getActionLevel() const
{
    return mActionLevel;
}

bool UndoRedo::hasUndo() const
{
    return mCurrentAction >= 0;
}

bool UndoRedo::hasRedo() const
{
    return static_cast<std::size_t>(mCurrentAction + 1) < mActions.size();
}

bool UndoRedo::isMerging() const
{
    return mMerging;
}

std::uint64_t UndoRedo::getVersion() const
{
    return mVersion;
}

void UndoRedo::setMaxSteps(int maxSteps)
{
    mMaxSteps = maxSteps;
}

int UndoRedo::getMaxSteps() const
{
    return mMaxSteps;
}

void UndoRedo::setCommitNotifyCallback(const CommitNotify& callback)
{
    mCommitNotify = callback;
}

void UndoRedo::setVersionChangedCallback(const VersionNotify& callback)
{
    mVersionChanged = callback;
}

void UndoRedo::setClock(const Clock& clock)
{
    mClock = clock;
}

void UndoRedo::setDeleter(const Deleter& deleter)
{
    mDeleter = deleter;
}

} // namespace prisma3d
