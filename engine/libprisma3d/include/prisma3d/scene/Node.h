#pragma once

#include "prisma3d/scene/PropertyValue.h"

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ct
{
class Json;
}

namespace prisma3d
{

struct SceneResult;
struct PropertyInfo;
struct InputEvent;
class SceneTree;
class ScriptInstance;
class Tween;

enum class NodeClass : std::uint32_t
{
    None = 0,
    Node = 1u << 0,
    Node3D = 1u << 1,
    VisualInstance3D = 1u << 2,
    MeshInstance3D = 1u << 3,
    PhysicsBody3D = 1u << 4
};

#define PRISMA3D_NODE_CLASS(m_class, m_inherits)                                                   \
public:                                                                                            \
    using SelfType = m_class;                                                                      \
    static void* classPtrStatic()                                                                  \
    {                                                                                              \
        static int ptr;                                                                            \
        return &ptr;                                                                               \
    }                                                                                              \
    bool isClassPtr(void* ptr) const override                                                      \
    {                                                                                              \
        return (ptr == classPtrStatic()) || m_inherits::isClassPtr(ptr);                           \
    }                                                                                              \
    static const char* classNameStatic()                                                           \
    {                                                                                              \
        return #m_class;                                                                           \
    }                                                                                              \
    const char* getClassName() const override                                                      \
    {                                                                                              \
        return classNameStatic();                                                                  \
    }                                                                                              \
    using SuperType = m_inherits

class Node
{
public:
    using SelfType = Node;
    static constexpr NodeClass kAncestralClass = NodeClass::Node;
    static void* classPtrStatic()
    {
        static int ptr;
        return &ptr;
    }
    virtual bool isClassPtr(void* ptr) const
    {
        return ptr == classPtrStatic();
    }
    static const char* classNameStatic()
    {
        return "Node";
    }
    virtual const char* getClassName() const
    {
        return classNameStatic();
    }
    static void bindProperties();

    enum Notification : int
    {
        kNotificationEnterTree = 10,
        kNotificationExitTree = 11,
        kNotificationMovedInParent = 12,
        kNotificationReady = 13,
        kNotificationPaused = 14,
        kNotificationUnpaused = 15,
        kNotificationPhysicsProcess = 16,
        kNotificationProcess = 17,
        kNotificationParented = 18,
        kNotificationUnparented = 19,
        kNotificationChildOrderChanged = 24,
        kNotificationPostEnterTree = 27,
        kNotificationDisabled = 28,
        kNotificationEnabled = 29,
        kNotificationDraw = 30,
        kNotificationWmCloseRequest = 1006,
        kNotificationWmSizeChanged = 1008,
        kNotificationTransformChanged = 2000,
        kNotificationPredelete = 1
    };

    Node();
    virtual ~Node();
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    bool isClass(NodeClass nodeClass) const;
    bool isClass(const char* className) const;
    template<typename T>
    bool derivesFrom() const
    {
        if constexpr (std::is_base_of<T, Node>::value)
        {
            return true;
        }
        else
        {
            static_assert(std::is_base_of<Node, T>::value, "derivesFrom needs a Node subclass");
            static_assert(std::is_same<T, typename T::SelfType>::value,
                    "T must use PRISMA3D_NODE_CLASS");
            if constexpr (T::kAncestralClass != T::SuperType::kAncestralClass)
                return isClass(T::kAncestralClass);
            else
                return isClassPtr(T::classPtrStatic());
        }
    }
    template<typename T>
    T* castTo()
    {
        return derivesFrom<T>() ? static_cast<T*>(this) : nullptr;
    }
    template<typename T>
    const T* castTo() const
    {
        return derivesFrom<T>() ? static_cast<const T*>(this) : nullptr;
    }

    enum class ProcessMode : std::uint8_t
    {
        Inherit,
        Pausable,
        WhenPaused,
        Always,
        Disabled
    };

    struct GroupInfo
    {
        ct::String name;
        bool persistent = false;
    };

    enum class NameNumSeparator : std::uint8_t
    {
        None,
        Space,
        Underscore,
        Dash
    };

    void setName(const ct::String& name);
    const ct::String& getName() const;
    static ct::String validateNodeName(const ct::String& name);
    static void setNameNumSeparator(NameNumSeparator separator);
    static NameNumSeparator getNameNumSeparator();

    bool set(const ct::String& property, const PropertyValue& value);
    PropertyValue get(const ct::String& property, bool* valid = nullptr) const;

    template<typename T, typename... Args>
    T* createChild(Args&&... args)
    {
        T* child = new T(static_cast<Args&&>(args)...);
        addChild(child);
        return child;
    }
    Node* createChild(const char* className, bool forceReadableName = false);
    Node* createChildWithScript(const char* className, const ct::String& scriptPath,
            bool forceReadableName = false);

    void addChild(Node* child, bool forceReadableName = false);
    void removeChild(Node* child);
    bool reparent(Node* newParent, bool forceReadableName = false);
    std::size_t getChildCount() const;
    Node* getChild(std::size_t index) const;
    Node* getParent() const;
    std::ptrdiff_t getIndex() const;
    void moveChild(Node* child, std::size_t toIndex);
    Node* findChild(const ct::String& name) const;
    Node* getNode(const ct::String& path) const;
    bool isAncestorOf(const Node* node) const;
    ct::String getPath() const;
    ct::String getPathTo(const Node* node) const;

    void setInternal(bool internal);
    bool isInternal() const;

    void setSceneFilePath(const ct::String& path);
    const ct::String& getSceneFilePath() const;
    bool isInstance() const;
    void setOwner(Node* owner);
    Node* getOwner() const;
    void setEditableInstance(Node* node, bool editable);
    bool isEditableInstance(const Node* node) const;

    bool isInsideTree() const;
    SceneTree* getTree() const;
    bool isReady() const;

    void setProcessMode(ProcessMode mode);
    ProcessMode getProcessMode() const;
    bool canProcess() const;
    bool isEnabled() const;

    bool isInGroup(const ct::String& group) const;
    void addToGroup(const ct::String& group, bool persistent = false);
    void removeFromGroup(const ct::String& group);
    void getGroups(ct::Vector<GroupInfo>& groups) const;
    std::size_t getPersistentGroupCount() const;

    void setScript(const ct::String& path);
    const ct::String& getScript() const;
    ScriptInstance* getScriptInstance() const;
    void setScriptInstance(ScriptInstance* instance);
    bool isDestroying() const;

    void setProcess(bool enable);
    bool isProcessing() const;
    void setProcessInput(bool enable);
    bool isProcessingInput() const;
    static std::size_t countProcessingInput();
    void input(const InputEvent& event);
    void setPhysicsProcess(bool enable);
    bool isPhysicsProcessing() const;
    double getProcessDeltaTime() const;
    double getPhysicsProcessDeltaTime() const;

    void notification(int what, bool reversed = false);
    void propagateNotification(int what);

    void queueFree();
    bool isQueuedForDeletion() const;

    Tween* createTween();
    void tweenOnce(const ct::Function<void(Tween&)>& build);

    virtual bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const;
    virtual bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path);

    void notifyPropertyListChanged();
    std::uint32_t getPropertyListRevision() const;

protected:
    virtual void validateProperty(PropertyInfo& info) const;

    static void* operator new(std::size_t size);
    static void operator delete(void* pointer);

    void defineAncestry(NodeClass nodeClass);

    virtual void notificationHandler(int what);
    virtual void inputHandler(const InputEvent& event);
    virtual void addChildNotify(Node* child);
    virtual void removeChildNotify(Node* child);
    virtual void moveChildNotify(Node* child);

private:
    friend class SceneTree;
    friend class NodeFactory;
    friend class NodeClassDB;
    friend class SceneSerializer;
    friend class Prefab;

    void validateChildName(Node* child, bool forceReadableName);
    void generateSerialChildName(const Node* child, ct::String& name) const;
    void setNameNoCheck(const ct::String& name);
    void clearForeignOwners(const Node* subtree);

    bool canProcessFor(bool paused) const;
    bool enabledNow() const;
    void updateProcessOwner();
    void propagateProcessOwner(Node* owner, int pauseNotification, int enabledNotification);
    void propagatePauseNotification(bool enable);
    void adjustProcessCounts(int process, int physics);

    void setTree(SceneTree* tree);
    void propagateEnterTree();
    void propagateReady();
    void propagateExitTree();
    void propagateAfterExitTree();

    std::uint32_t mAncestry;
    std::uint32_t mPropertyListRevision;
    ct::String mName;
    Node* mParent;
    ct::Vector<Node*> mChildren;
    ct::HashMap<ct::String, Node*> mChildrenByName;
    SceneTree* mTree;
    bool mInsideTree;
    bool mReady;
    bool mReadyFirst;
    ProcessMode mProcessMode;
    Node* mProcessOwner;
    ct::Vector<GroupInfo> mGroups;
    bool mProcess;
    bool mPhysicsProcess;
    // How many nodes of this subtree (this one included) process / physics-process: the tree
    // walk skips a subtree with none (a level's thousands of static pieces).
    std::uint32_t mProcessCount;
    std::uint32_t mPhysicsProcessCount;
    bool mProcessInput;
    bool mDestroying;
    bool mQueuedForDeletion;
    bool mInternal;
    bool mEditableInstance;
    ct::String mSceneFilePath;
    ct::String mScriptPath;
    ScriptInstance* mScript;
    Node* mOwner;
    std::ptrdiff_t mIndex;
};

} // namespace prisma3d
