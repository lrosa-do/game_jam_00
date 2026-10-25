#include "prisma3d/editor/EditorUndoRedoManager.h"

#include "platform.h"

namespace prisma3d
{

EditorUndoRedoManager::Applying::Applying(EditorUndoRedoManager* manager)
        : mManager(manager)
{
    if (mManager) ++mManager->mBusy;
}

EditorUndoRedoManager::Applying::~Applying()
{
    if (mManager) --mManager->mBusy;
}

EditorUndoRedoManager::EditorUndoRedoManager()
        : mEditObject(nullptr),
          mSavedVersion(1),
          mBusy(0),
          mHasPending(false),
          mFrozen(false)
{
}

UndoRedo& EditorUndoRedoManager::history()
{
    return mUndoRedo;
}

const UndoRedo& EditorUndoRedoManager::history() const
{
    return mUndoRedo;
}

UndoRedo& EditorUndoRedoManager::active()
{
    return mFrozen ? mScratch : mUndoRedo;
}

const UndoRedo& EditorUndoRedoManager::active() const
{
    return mFrozen ? mScratch : mUndoRedo;
}

double EditorUndoRedoManager::timestamp() const
{
    return static_cast<double>(time_nanos()) * 1e-9;
}

void EditorUndoRedoManager::setNamePrefix(const ct::String& prefix)
{
    mNamePrefix = prefix;
}

const ct::String& EditorUndoRedoManager::namePrefix() const
{
    return mNamePrefix;
}

void EditorUndoRedoManager::createAction(const ct::String& name, UndoRedo::MergeMode mode,
        bool backwardUndoOps, bool markUnsaved)
{
    if (!mHasPending)
    {
        mEditObject = nullptr;
        mEditName = ct::String();
        mPending = Action();
        mPending.name = mNamePrefix + name;
        mPending.timestamp = timestamp();
        mPending.mergeMode = mode;
        mPending.backwardUndoOps = backwardUndoOps;
        mPending.markUnsaved = markUnsaved;
        mHasPending = true;
    }
    active().createAction(mPending.name, mPending.mergeMode, mPending.backwardUndoOps);
}

void EditorUndoRedoManager::createPropertyAction(const ct::String& name, Node* object)
{
    const bool same = !mHasPending && object == mEditObject && name == mEditName;
    createAction(name, same ? UndoRedo::kMergeEnds : UndoRedo::kMergeDisable);
    mEditObject = object;
    mEditName = name;
}

void EditorUndoRedoManager::addDoMethod(const UndoRedo::Callable& callable, Node* object)
{
    active().addDoMethod(callable, object);
}

void EditorUndoRedoManager::addUndoMethod(const UndoRedo::Callable& callable, Node* object)
{
    active().addUndoMethod(callable, object);
}

void EditorUndoRedoManager::addDoProperty(Node* object, const ct::String& property,
        const PropertyValue& value)
{
    active().addDoProperty(object, property, value);
}

void EditorUndoRedoManager::addUndoProperty(Node* object, const ct::String& property,
        const PropertyValue& value)
{
    active().addUndoProperty(object, property, value);
}

void EditorUndoRedoManager::addDoReference(Node* object)
{
    active().addDoReference(object);
}

void EditorUndoRedoManager::addUndoReference(Node* object)
{
    active().addUndoReference(object);
}

bool EditorUndoRedoManager::pendingAction() const
{
    return mHasPending;
}

void EditorUndoRedoManager::commitAction(bool execute)
{
    if (!mHasPending) return;

    if (mFrozen)
    {
        ++mBusy;
        mScratch.commitAction(execute);
        --mBusy;
        if (mScratch.getActionLevel() > 0) return;
        mScratch.clearHistory(false);
        mHasPending = false;
        mPending = Action();
        return;
    }

    ++mBusy;
    const bool merging = mUndoRedo.isMerging();
    mUndoRedo.commitAction(execute);
    --mBusy;

    if (mUndoRedo.getActionLevel() > 0) return;

    mRedoStack.clear();
    if (mSavedVersion >= mUndoRedo.getVersion()) mSavedVersion = kUnsavedVersion;

    if (!merging) mUndoStack.push_back(mPending);
    const std::size_t kept = static_cast<std::size_t>(mUndoRedo.getCurrentAction() + 1);
    while (mUndoStack.size() > kept) mUndoStack.erase(mUndoStack.begin());

    mPending = Action();
    mHasPending = false;
}

bool EditorUndoRedoManager::isCommittingAction() const
{
    return mUndoRedo.isCommittingAction() || mScratch.isCommittingAction();
}

bool EditorUndoRedoManager::undo()
{
    if (mFrozen || !hasUndo()) return false;

    const Action action = mUndoStack.back();
    mUndoStack.pop_back();
    mRedoStack.push_back(action);
    mEditObject = nullptr;

    ++mBusy;
    const bool success = mUndoRedo.undo();
    --mBusy;
    return success;
}

bool EditorUndoRedoManager::redo()
{
    if (mFrozen || !hasRedo()) return false;

    const Action action = mRedoStack.back();
    mRedoStack.pop_back();
    mUndoStack.push_back(action);
    mEditObject = nullptr;

    ++mBusy;
    const bool success = mUndoRedo.redo();
    --mBusy;
    return success;
}

void EditorUndoRedoManager::clearHistory(bool increaseVersion)
{
    mEditObject = nullptr;
    mUndoRedo.clearHistory(increaseVersion);
    mUndoStack.clear();
    mRedoStack.clear();

    if (increaseVersion) mSavedVersion = kUnsavedVersion;
    else
        setHistoryAsSaved();
}

void EditorUndoRedoManager::setHistoryAsSaved()
{
    mSavedVersion = mUndoRedo.getVersion();
}

void EditorUndoRedoManager::setHistoryAsUnsaved()
{
    mSavedVersion = kUnsavedVersion;
}

bool EditorUndoRedoManager::isHistoryUnsaved() const
{
    if (mSavedVersion == kUnsavedVersion) return true;

    const std::int64_t difference = static_cast<std::int64_t>(mUndoRedo.getVersion()) -
                                    static_cast<std::int64_t>(mSavedVersion);
    if (difference > 0)
    {
        std::size_t index = mUndoStack.size();
        for (std::int64_t i = 0; i < difference; ++i)
        {
            if (index == 0) return true;
            --index;
            if (mUndoStack[index].markUnsaved) return true;
        }
    }
    else if (difference < 0)
    {
        std::size_t index = mRedoStack.size();
        for (std::int64_t i = 0; i > difference; --i)
        {
            if (index == 0) return true;
            --index;
            if (mRedoStack[index].markUnsaved) return true;
        }
    }
    return false;
}

bool EditorUndoRedoManager::hasUndo() const
{
    return !mFrozen && !mUndoStack.empty() && mUndoRedo.hasUndo();
}

bool EditorUndoRedoManager::hasRedo() const
{
    return !mFrozen && !mRedoStack.empty() && mUndoRedo.hasRedo();
}

ct::String EditorUndoRedoManager::currentActionName() const
{
    if (!hasUndo()) return ct::String();
    return mUndoRedo.getCurrentActionName();
}

ct::String EditorUndoRedoManager::redoActionName() const
{
    if (!hasRedo()) return ct::String();
    return mUndoRedo.getNextActionName();
}

std::size_t EditorUndoRedoManager::undoCount() const
{
    return mUndoStack.size();
}

std::size_t EditorUndoRedoManager::redoCount() const
{
    return mRedoStack.size();
}

void EditorUndoRedoManager::setFrozen(bool frozen)
{
    mFrozen = frozen;
}

bool EditorUndoRedoManager::frozen() const
{
    return mFrozen;
}

bool EditorUndoRedoManager::busy() const
{
    return mBusy > 0;
}

std::uint64_t EditorUndoRedoManager::version() const
{
    return mUndoRedo.getVersion();
}

std::uint64_t EditorUndoRedoManager::savedVersion() const
{
    return mSavedVersion;
}

} // namespace prisma3d
