#include "prisma3d/scene/Node.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/ScriptInstance.h"
#include "prisma3d/scene/ScriptServer.h"
#include "prisma3d/scene/Tween.h"

#include "platform.h"

#include <ct/thread.hpp>

namespace prisma3d
{

namespace
{

std::size_t sOwnedNodes = 0;
std::size_t sInputNodes = 0;
ct::Atomic<std::uint32_t> sGeneratedNameCount(1);
Node::NameNumSeparator sNameNumSeparator = Node::NameNumSeparator::None;

const char kInvalidNameCharacters[] = ".:@/\"%";

const char* separatorText()
{
    switch (sNameNumSeparator)
    {
        case Node::NameNumSeparator::None:
            return "";
        case Node::NameNumSeparator::Space:
            return " ";
        case Node::NameNumSeparator::Underscore:
            return "_";
        case Node::NameNumSeparator::Dash:
            return "-";
    }
    return " ";
}

bool isDigit(char c)
{
    return c >= '0' && c <= '9';
}

ct::String increaseNumericString(const ct::String& s)
{
    ct::String result = s;
    bool carry = result.size() > 0;

    for (std::size_t i = result.size(); i > 0; --i)
    {
        if (!carry) break;
        const std::size_t at = i - 1;
        if (s[at] == '9')
        {
            result[at] = '0';
        }
        else
        {
            result[at] = static_cast<char>(s[at] + 1);
            carry = false;
        }
    }

    if (carry) result = ct::String("1") + result;

    return result;
}

} // namespace

Node::Node()
        : mAncestry(0),
          mPropertyListRevision(0),
          mName(),
          mParent(nullptr),
          mChildren(),
          mTree(nullptr),
          mInsideTree(false),
          mReady(false),
          mReadyFirst(true),
          mProcessMode(ProcessMode::Inherit),
          mProcessOwner(nullptr),
          mProcess(false),
          mPhysicsProcess(false),
          mProcessCount(0),
          mPhysicsProcessCount(0),
          mProcessInput(false),
          mDestroying(false),
          mQueuedForDeletion(false),
          mInternal(false),
          mEditableInstance(false),
          mSceneFilePath(),
          mScriptPath(),
          mScript(nullptr),
          mOwner(nullptr),
          mIndex(-1)
{
    defineAncestry(NodeClass::Node);
}

Node::~Node()
{
    mDestroying = true;
    if (mOwner) --sOwnedNodes;

    {
        SceneTree* trees[2] = { mTree, SceneTree::singleton() };
        for (SceneTree* tree: trees)
            if (tree) tree->dropTweenNode(this);
    }

    if (mQueuedForDeletion)
    {
        SceneTree* trees[2] = { mTree, SceneTree::singleton() };
        for (SceneTree* tree: trees)
        {
            if (!tree) continue;
            for (std::size_t i = 0; i < tree->mDeleteQueue.size(); ++i)
            {
                if (tree->mDeleteQueue[i] == this)
                {
                    tree->mDeleteQueue.erase(tree->mDeleteQueue.begin() + i);
                    break;
                }
            }
        }
    }

    if (mParent) mParent->removeChild(this);
    else if (mTree)
        setTree(nullptr);

    while (!mChildren.empty())
    {
        Node* child = mChildren.back();
        removeChild(child);
        delete child;
    }

    delete mScript;
    mScript = nullptr;
    if (mProcessInput) --sInputNodes;
}

bool Node::isClass(NodeClass nodeClass) const
{
    return (mAncestry & static_cast<std::uint32_t>(nodeClass)) != 0;
}

bool Node::isClass(const char* className) const
{
    return className && NodeClassDB::isParentClass(getClassName(), className);
}

void Node::defineAncestry(NodeClass nodeClass)
{
    mAncestry |= static_cast<std::uint32_t>(nodeClass);
}

Node* Node::createChild(const char* className, bool forceReadableName)
{
    return createChildWithScript(className, ct::String(), forceReadableName);
}

Node* Node::createChildWithScript(const char* className, const ct::String& scriptPath,
        bool forceReadableName)
{
    if (!className)
    {
        log_error("node: no class name to create a child");
        return nullptr;
    }
    Node* child = NodeClassDB::instantiate(className);
    if (!child) return nullptr;
    if (!scriptPath.empty()) child->setScript(scriptPath);
    addChild(child, forceReadableName);
    return child;
}

ct::String Node::validateNodeName(const ct::String& name)
{
    ct::String validated = name;
    for (std::size_t i = 0; i < validated.size(); ++i)
    {
        for (const char* c = kInvalidNameCharacters; *c; ++c)
        {
            if (validated[i] == *c)
            {
                validated[i] = '_';
                break;
            }
        }
    }
    return validated;
}

void Node::setNameNumSeparator(NameNumSeparator separator)
{
    sNameNumSeparator = separator;
}

Node::NameNumSeparator Node::getNameNumSeparator()
{
    return sNameNumSeparator;
}

void Node::setNameNoCheck(const ct::String& name)
{
    if (!mParent) mName = name;
}

void Node::generateSerialChildName(const Node* child, ct::String& name) const
{
    if (name.empty()) name = child->getClassName();

    Node* const* existing = mChildrenByName.find(name);
    if (!existing || *existing == child) return;

    ct::String nameString = name;
    std::size_t digits = nameString.size();
    while (digits > 0 && isDigit(nameString[digits - 1])) --digits;
    ct::String nums = nameString.substr(digits);

    const ct::String separator(separatorText());
    const std::ptrdiff_t lastIndex = static_cast<std::ptrdiff_t>(nameString.size()) -
                                     static_cast<std::ptrdiff_t>(separator.size()) -
                                     static_cast<std::ptrdiff_t>(nums.size());

    if (nums.size() > 0 && lastIndex >= 0 &&
            nameString.substr(static_cast<std::size_t>(lastIndex), separator.size()) == separator)
    {
        nameString = nameString.substr(0, static_cast<std::size_t>(lastIndex) + separator.size());
    }
    else
    {
        nums = ct::String();
    }

    for (;;)
    {
        const ct::String attempt = nameString + nums;

        existing = mChildrenByName.find(attempt);
        const bool exists = existing != nullptr && *existing != child;

        if (!exists)
        {
            name = attempt;
            return;
        }

        if (nums.size() == 0)
        {
            nums = "2";
            nameString += separator;
        }
        else
        {
            nums = increaseNumericString(nums);
        }
    }
}

void Node::validateChildName(Node* child, bool forceReadableName)
{
    if (forceReadableName)
    {
        ct::String name = child->mName;
        generateSerialChildName(child, name);
        child->mName = name;
        return;
    }

    bool unique = true;
    if (child->mName.empty())
    {
        unique = false;
    }
    else
    {
        Node* const* existing = mChildrenByName.find(child->mName);
        unique = !existing || *existing == child;
    }

    if (!unique)
    {
        const std::uint32_t count = sGeneratedNameCount.fetch_add(1) + 1;
        ct::String name = "@";
        name += child->getClassName();
        name += '@';
        name.append_number(static_cast<unsigned long long>(count));
        child->mName = name;
    }
}

void Node::setName(const ct::String& name)
{
    if (name.empty())
    {
        log_error("node: a node cannot be given an empty name");
        return;
    }

    const ct::String oldName = mName;
    mName = validateNodeName(name);

    if (mParent)
    {
        mParent->validateChildName(this, true);
        mParent->mChildrenByName.erase(oldName);
        mParent->mChildrenByName.put(mName, this);
    }

    if (mInsideTree && mTree)
    {
        mTree->nodeRenamed(this);
        mTree->treeChanged();
    }
}

const ct::String& Node::getName() const
{
    return mName;
}

bool Node::set(const ct::String& property, const PropertyValue& value)
{
    return NodeClassDB::setProperty(this, property, value);
}

PropertyValue Node::get(const ct::String& property, bool* valid) const
{
    PropertyValue value;
    const bool found = NodeClassDB::getProperty(this, property, value);
    if (valid) *valid = found;
    return value;
}

void Node::notifyPropertyListChanged()
{
    ++mPropertyListRevision;
}

std::uint32_t Node::getPropertyListRevision() const
{
    return mPropertyListRevision;
}

void Node::validateProperty(PropertyInfo& info) const
{
    (void) info;
}

void Node::bindProperties()
{
    NodeClassDB::addProperty<Node>(
            PropertyInfo(PropertyType::String, "name", PropertyHint::None, "", PropertyUsage::None),
            &Node::setName, &Node::getName);
    NodeClassDB::addProperty<Node>(PropertyInfo::enumeration("process_mode",
                                           "Inherit,Pausable,When Paused,Always,Disabled",
                                           PropertyUsage::NoEditor),
            &Node::setProcessMode, &Node::getProcessMode);
    NodeClassDB::addProperty<Node>(PropertyInfo::file("script", "py"), &Node::setScript,
            &Node::getScript);
}

void* Node::operator new(std::size_t size)
{
    return ::operator new(size);
}

void Node::operator delete(void* pointer)
{
    ::operator delete(pointer);
}

bool Node::reparent(Node* newParent, bool forceReadableName)
{
    if (!newParent || newParent == this || newParent == mParent || isAncestorOf(newParent))
        return false;
    if (mParent) mParent->removeChild(this);
    newParent->addChild(this, forceReadableName);
    return true;
}

void Node::addChild(Node* child, bool forceReadableName)
{
    if (!child || child == this || child->mParent || child->mInsideTree ||
            child->isAncestorOf(this))
        return;

    validateChildName(child, forceReadableName);

    child->mParent = this;
    child->mIndex = static_cast<std::ptrdiff_t>(mChildren.size());
    mChildren.push_back(child);
    mChildrenByName.put(child->mName, child);
    if (child->mProcessCount || child->mPhysicsProcessCount)
        adjustProcessCounts(static_cast<int>(child->mProcessCount),
                static_cast<int>(child->mPhysicsProcessCount));

    child->notification(kNotificationParented);

    if (mTree) child->setTree(mTree);

    addChildNotify(child);
    notification(kNotificationChildOrderChanged);
}

void Node::removeChild(Node* child)
{
    if (!child || child->mParent != this) return;

    child->setTree(nullptr);

    removeChildNotify(child);
    child->notification(kNotificationUnparented);

    const std::size_t removed = static_cast<std::size_t>(child->mIndex);
    mChildren.erase(mChildren.begin() + removed);
    mChildrenByName.erase(child->mName);
    if (child->mProcessCount || child->mPhysicsProcessCount)
        adjustProcessCounts(-static_cast<int>(child->mProcessCount),
                -static_cast<int>(child->mPhysicsProcessCount));
    for (std::size_t i = removed; i < mChildren.size(); ++i)
        mChildren[i]->mIndex = static_cast<std::ptrdiff_t>(i);

    child->mParent = nullptr;
    child->mIndex = -1;

    if (sOwnedNodes > 0) child->clearForeignOwners(child);

    notification(kNotificationChildOrderChanged);

    if (mTree) child->propagateAfterExitTree();
}

std::size_t Node::getChildCount() const
{
    return mChildren.size();
}

Node* Node::getChild(std::size_t index) const
{
    return index < mChildren.size() ? mChildren[index] : nullptr;
}

Node* Node::getParent() const
{
    return mParent;
}

std::ptrdiff_t Node::getIndex() const
{
    return mIndex;
}

void Node::moveChild(Node* child, std::size_t toIndex)
{
    if (!child || child->mParent != this) return;
    if (toIndex > mChildren.size()) return;
    if (toIndex == mChildren.size()) --toIndex;

    const std::size_t from = static_cast<std::size_t>(child->mIndex);
    if (from == toIndex) return;

    const std::size_t motionFrom = from < toIndex ? from : toIndex;
    const std::size_t motionTo = from < toIndex ? toIndex : from;

    mChildren.erase(mChildren.begin() + from);
    mChildren.insert(mChildren.begin() + toIndex, child);

    if (mTree) mTree->treeChanged();

    for (std::size_t i = motionFrom; i <= motionTo; ++i)
        mChildren[i]->mIndex = static_cast<std::ptrdiff_t>(i);

    moveChildNotify(child);
    child->notification(kNotificationMovedInParent);
    notification(kNotificationChildOrderChanged);
}

Node* Node::findChild(const ct::String& name) const
{
    Node* const* child = mChildrenByName.find(name);
    return child ? *child : nullptr;
}

Node* Node::getNode(const ct::String& path) const
{
    if (path.empty()) return nullptr;

    const Node* current = this;
    const Node* root = nullptr;
    if (path.front() == '/')
    {
        if (!mInsideTree) return nullptr;
        root = this;
        while (root->mParent) root = root->mParent;
        current = nullptr;
    }

    std::size_t pos = 0;
    while (pos < path.size())
    {
        std::size_t end = path.find('/', pos);
        if (end == ct::String::npos) end = path.size();

        if (end > pos)
        {
            const ct::String name = path.substr(pos, end - pos);
            if (!current)
            {
                if (name != root->mName) return nullptr;
                current = root;
            }
            else if (name == ".")
            {
            }
            else if (name == "..")
            {
                if (!current->mParent) return nullptr;
                current = current->mParent;
            }
            else
            {
                current = current->findChild(name);
                if (!current) return nullptr;
            }
        }

        pos = end + 1;
    }

    return const_cast<Node*>(current);
}

bool Node::isAncestorOf(const Node* node) const
{
    if (!node || node == this) return false;
    for (const Node* n = node->mParent; n; n = n->mParent)
    {
        if (n == this) return true;
    }
    return false;
}

ct::String Node::getPath() const
{
    if (!mInsideTree) return ct::String();

    ct::Vector<const Node*> chain;
    for (const Node* n = this; n; n = n->mParent) chain.push_back(n);

    ct::String path;
    for (std::size_t i = chain.size(); i > 0; --i)
    {
        path += '/';
        path += chain[i - 1]->mName;
    }
    return path;
}

ct::String Node::getPathTo(const Node* node) const
{
    if (!node) return ct::String();
    if (node == this) return ct::String(".");

    ct::Vector<const Node*> mine;
    for (const Node* n = this; n; n = n->mParent) mine.push_back(n);
    ct::Vector<const Node*> theirs;
    for (const Node* n = node; n; n = n->mParent) theirs.push_back(n);

    if (mine.back() != theirs.back()) return ct::String();

    std::size_t common = 0;
    while (common < mine.size() && common < theirs.size() &&
            mine[mine.size() - 1 - common] == theirs[theirs.size() - 1 - common])
        ++common;

    ct::String path;
    for (std::size_t i = 0; i < mine.size() - common; ++i)
    {
        if (!path.empty()) path += '/';
        path += "..";
    }
    for (std::size_t i = theirs.size() - common; i > 0; --i)
    {
        if (!path.empty()) path += '/';
        path += theirs[i - 1]->mName;
    }
    if (getNode(path) != node) return ct::String();
    return path;
}

bool Node::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) data;
    (void) result;
    (void) path;
    return false;
}

bool Node::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    (void) data;
    (void) result;
    (void) path;
    return true;
}

void Node::setInternal(bool internal)
{
    mInternal = internal;
}

bool Node::isInternal() const
{
    return mInternal;
}

void Node::setSceneFilePath(const ct::String& path)
{
    mSceneFilePath = path;
}

const ct::String& Node::getSceneFilePath() const
{
    return mSceneFilePath;
}

bool Node::isInstance() const
{
    return !mSceneFilePath.empty();
}

void Node::setOwner(Node* owner)
{
    if (owner == mOwner) return;
    if (owner && !owner->isAncestorOf(this)) return;
    if (mOwner && !owner) --sOwnedNodes;
    else if (!mOwner && owner)
        ++sOwnedNodes;
    mOwner = owner;
}

Node* Node::getOwner() const
{
    return mOwner;
}

void Node::clearForeignOwners(const Node* subtree)
{
    if (mOwner && mOwner != subtree && !subtree->isAncestorOf(mOwner))
    {
        mOwner = nullptr;
        --sOwnedNodes;
    }
    for (std::size_t i = 0; i < mChildren.size(); ++i) mChildren[i]->clearForeignOwners(subtree);
}

void Node::setEditableInstance(Node* node, bool editable)
{
    if (!node || !isAncestorOf(node)) return;
    node->mEditableInstance = editable;
}

bool Node::isEditableInstance(const Node* node) const
{
    if (!node || !isAncestorOf(node)) return false;
    return node->mEditableInstance;
}

bool Node::isInsideTree() const
{
    return mInsideTree;
}

SceneTree* Node::getTree() const
{
    return mTree;
}

bool Node::isReady() const
{
    return mReady;
}

void Node::updateProcessOwner()
{
    if (mProcessMode == ProcessMode::Inherit)
    {
        if (mParent) mProcessOwner = mParent->mProcessOwner;
        else
        {
            log_error("node: the root node can't be set to Inherit process mode, reverting to "
                      "Pausable instead");
            mProcessMode = ProcessMode::Pausable;
            mProcessOwner = this;
        }
    }
    else
        mProcessOwner = this;
}

void Node::setProcessMode(ProcessMode mode)
{
    if (mProcessMode == mode) return;

    if (!mInsideTree)
    {
        mProcessMode = mode;
        return;
    }

    const bool previousCanProcess = canProcess();
    const bool previousEnabled = enabledNow();

    if (mode == ProcessMode::Inherit)
    {
        if (mParent) mProcessOwner = mParent->mProcessOwner;
        else
        {
            log_error("node: the root node can't be set to Inherit process mode");
            return;
        }
    }
    else
        mProcessOwner = this;

    mProcessMode = mode;

    const bool nextCanProcess = canProcess();
    const bool nextEnabled = enabledNow();

    int pauseNotification = 0;
    if (previousCanProcess && !nextCanProcess) pauseNotification = kNotificationPaused;
    else if (!previousCanProcess && nextCanProcess)
        pauseNotification = kNotificationUnpaused;

    int enabledNotification = 0;
    if (previousEnabled && !nextEnabled) enabledNotification = kNotificationDisabled;
    else if (!previousEnabled && nextEnabled)
        enabledNotification = kNotificationEnabled;

    propagateProcessOwner(mProcessOwner, pauseNotification, enabledNotification);
}

Node::ProcessMode Node::getProcessMode() const
{
    return mProcessMode;
}

void Node::propagateProcessOwner(Node* owner, int pauseNotification, int enabledNotification)
{
    mProcessOwner = owner;

    if (pauseNotification != 0) notification(pauseNotification);
    if (enabledNotification != 0) notification(enabledNotification);

    for (std::size_t i = 0; i < mChildren.size(); ++i)
    {
        Node* child = mChildren[i];
        if (child->mProcessMode == ProcessMode::Inherit)
            child->propagateProcessOwner(owner, pauseNotification, enabledNotification);
    }
}

void Node::propagatePauseNotification(bool enable)
{
    const bool previousCanProcess = canProcessFor(!enable);
    const bool nextCanProcess = canProcessFor(enable);

    if (previousCanProcess && !nextCanProcess) notification(kNotificationPaused);
    else if (!previousCanProcess && nextCanProcess)
        notification(kNotificationUnpaused);

    for (std::size_t i = 0; i < mChildren.size(); ++i)
        mChildren[i]->propagatePauseNotification(enable);
}

bool Node::canProcess() const
{
    return mInsideTree && mTree && canProcessFor(mTree->isPaused());
}

bool Node::canProcessFor(bool paused) const
{
    ProcessMode mode = mProcessMode;
    if (mode == ProcessMode::Inherit)
        mode = mProcessOwner ? mProcessOwner->mProcessMode : ProcessMode::Pausable;

    if (mode == ProcessMode::Inherit) return false;
    if (mode == ProcessMode::Disabled) return false;
    if (mode == ProcessMode::Always) return true;
    if (paused) return mode == ProcessMode::WhenPaused;
    return mode == ProcessMode::Pausable;
}

bool Node::enabledNow() const
{
    ProcessMode mode = mProcessMode;
    if (mode == ProcessMode::Inherit)
        mode = mProcessOwner ? mProcessOwner->mProcessMode : ProcessMode::Pausable;
    return mode != ProcessMode::Disabled;
}

bool Node::isEnabled() const
{
    return mInsideTree && enabledNow();
}

bool Node::isInGroup(const ct::String& group) const
{
    for (std::size_t i = 0; i < mGroups.size(); ++i)
        if (mGroups[i].name == group) return true;
    return false;
}

void Node::addToGroup(const ct::String& group, bool persistent)
{
    if (group.empty())
    {
        log_error("node: cannot add node '%s' to a group with an empty name", mName.c_str());
        return;
    }
    if (isInGroup(group)) return;

    GroupInfo info;
    info.name = group;
    info.persistent = persistent;
    mGroups.push_back(info);
    if (mTree) mTree->addToGroup(group, this);
}

void Node::removeFromGroup(const ct::String& group)
{
    for (std::size_t i = 0; i < mGroups.size(); ++i)
    {
        if (mGroups[i].name != group) continue;
        if (mTree) mTree->removeFromGroup(group, this);
        mGroups.erase(mGroups.begin() + i);
        return;
    }
}

void Node::getGroups(ct::Vector<GroupInfo>& groups) const
{
    for (std::size_t i = 0; i < mGroups.size(); ++i) groups.push_back(mGroups[i]);
}

std::size_t Node::getPersistentGroupCount() const
{
    std::size_t count = 0;
    for (std::size_t i = 0; i < mGroups.size(); ++i)
        if (mGroups[i].persistent) ++count;
    return count;
}

void Node::setScript(const ct::String& path)
{
    if (path == mScriptPath && mScript && mScript->hasScript()) return;
    mScriptPath = path;
    ScriptServer* server = ScriptServer::singleton();
    if (server) server->attachScript(this, path);
    notifyPropertyListChanged();
}

const ct::String& Node::getScript() const
{
    return mScriptPath;
}

ScriptInstance* Node::getScriptInstance() const
{
    return mScript;
}

void Node::setScriptInstance(ScriptInstance* instance)
{
    if (mScript == instance) return;
    ScriptInstance* previous = mScript;
    mScript = instance;
    delete previous;
}

bool Node::isDestroying() const
{
    return mDestroying;
}

void Node::adjustProcessCounts(int process, int physics)
{
    for (Node* node = this; node; node = node->mParent)
    {
        node->mProcessCount = static_cast<std::uint32_t>(static_cast<int>(node->mProcessCount) + process);
        node->mPhysicsProcessCount =
                static_cast<std::uint32_t>(static_cast<int>(node->mPhysicsProcessCount) + physics);
    }
}

void Node::setProcess(bool enable)
{
    if (mProcess == enable) return;
    mProcess = enable;
    adjustProcessCounts(enable ? 1 : -1, 0);
}

bool Node::isProcessing() const
{
    return mProcess;
}

void Node::setPhysicsProcess(bool enable)
{
    if (mPhysicsProcess == enable) return;
    mPhysicsProcess = enable;
    adjustProcessCounts(0, enable ? 1 : -1);
}

bool Node::isPhysicsProcessing() const
{
    return mPhysicsProcess;
}

void Node::setProcessInput(bool enable)
{
    if (mProcessInput == enable) return;
    mProcessInput = enable;
    if (enable) ++sInputNodes;
    else
        --sInputNodes;
}

std::size_t Node::countProcessingInput()
{
    return sInputNodes;
}

bool Node::isProcessingInput() const
{
    return mProcessInput;
}

void Node::input(const InputEvent& event)
{
    if (mDestroying) return;
    inputHandler(event);
    if (mScript && !mDestroying) mScript->input(event);
}

void Node::inputHandler(const InputEvent& event)
{
    (void) event;
}

double Node::getProcessDeltaTime() const
{
    return mTree ? mTree->getProcessDeltaTime() : 0.0;
}

double Node::getPhysicsProcessDeltaTime() const
{
    return mTree ? mTree->getPhysicsProcessDeltaTime() : 0.0;
}

void Node::notification(int what, bool reversed)
{
    if (reversed)
    {
        if (mScript && !mDestroying) mScript->notification(what);
        notificationHandler(what);
    }
    else
    {
        notificationHandler(what);
        if (mScript && !mDestroying) mScript->notification(what);
    }
}

void Node::propagateNotification(int what)
{
    notification(what);
    for (std::size_t i = 0; i < mChildren.size(); ++i) mChildren[i]->propagateNotification(what);
}

void Node::queueFree()
{
    if (mQueuedForDeletion) return;

    SceneTree* tree = mTree ? mTree : SceneTree::singleton();
    if (tree) tree->queueDelete(this);
    else
        mQueuedForDeletion = true;
}

Tween* Node::createTween()
{
    SceneTree* tree = mTree ? mTree : SceneTree::singleton();
    if (!tree)
    {
        log_error("node: no available scene tree to create the tween");
        return nullptr;
    }

    Tween* tween = tree->createTween();
    tween->bindNode(this);
    return tween;
}

void Node::tweenOnce(const ct::Function<void(Tween&)>& build)
{
    SceneTree* tree = mTree ? mTree : SceneTree::singleton();
    if (!tree || !build)
    {
        if (!tree) log_error("node: no available scene tree to create the tween");
        return;
    }

    Node* self = this;
    tree->tweenOnce(
            [self, &build](Tween& tween)
            {
                tween.bindNode(self);
                build(tween);
            });
}

bool Node::isQueuedForDeletion() const
{
    return mQueuedForDeletion;
}

void Node::notificationHandler(int what)
{
    (void) what;
}

void Node::addChildNotify(Node* child)
{
    (void) child;
}

void Node::removeChildNotify(Node* child)
{
    (void) child;
}

void Node::moveChildNotify(Node* child)
{
    (void) child;
}

void Node::setTree(SceneTree* tree)
{
    SceneTree* changed = nullptr;

    if (mTree) propagateExitTree();

    mTree = tree;

    if (mTree)
    {
        propagateEnterTree();
        if (!mParent || mParent->mReady) propagateReady();

        changed = mTree;
    }

    if (changed) changed->treeChanged();
}

void Node::propagateEnterTree()
{
    if (mParent) mTree = mParent->mTree;

    mInsideTree = true;

    if (mTree)
        for (std::size_t i = 0; i < mGroups.size(); ++i) mTree->addToGroup(mGroups[i].name, this);

    updateProcessOwner();

    notification(kNotificationEnterTree);

    if (mTree) mTree->nodeAdded(this);

    for (std::size_t i = 0; i < mChildren.size(); ++i)
    {
        Node* child = mChildren[i];
        if (!child->mInsideTree) child->propagateEnterTree();
    }
}

void Node::propagateReady()
{
    mReady = true;

    const std::size_t count = mChildren.size();
    for (std::size_t i = 0; i < count && i < mChildren.size(); ++i) mChildren[i]->propagateReady();

    notification(kNotificationPostEnterTree);

    if (mReadyFirst)
    {
        mReadyFirst = false;
        notification(kNotificationReady);
    }
}

void Node::propagateExitTree()
{
    for (std::size_t i = mChildren.size(); i > 0; --i)
    {
        if (i - 1 < mChildren.size() && mChildren[i - 1]->mInsideTree)
            mChildren[i - 1]->propagateExitTree();
    }

    notification(kNotificationExitTree, true);

    if (mTree)
    {
        mTree->nodeRemoved(this);
        mTree->treeChanged();
        for (std::size_t i = 0; i < mGroups.size(); ++i)
            mTree->removeFromGroup(mGroups[i].name, this);
    }

    mProcessOwner = nullptr;

    mReady = false;
    mTree = nullptr;
    mInsideTree = false;
}

void Node::propagateAfterExitTree()
{
    for (std::size_t i = mChildren.size(); i > 0; --i)
    {
        if (i - 1 < mChildren.size()) mChildren[i - 1]->propagateAfterExitTree();
    }
}

} // namespace prisma3d
