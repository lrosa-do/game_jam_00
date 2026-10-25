#include "ZenInternal.h"

#include "prisma3d/Device.h"
#include "prisma3d/InputMap.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/AudioListener3D.h"
#include "prisma3d/scene/AudioStreamPlayer.h"
#include "prisma3d/scene/AudioStreamPlayer3D.h"
#include "prisma3d/scene/BoneAttachment3D.h"
#include "prisma3d/scene/Button.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/CarDamage3D.h"
#include "prisma3d/scene/CharacterBody3D.h"
#include "prisma3d/scene/CharacterRigidBody3D.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Curve3D.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Destructible3D.h"
#include "prisma3d/scene/Dismember3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/GroundVehicle3D.h"
#include "prisma3d/scene/Hair3D.h"
#include "prisma3d/scene/ImpactFx3D.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/LineEdit.h"
#include "prisma3d/scene/ManualMesh3D.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/NavigationRegion3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/Ocean3D.h"
#include "prisma3d/scene/ParticleCollision3D.h"
#include "prisma3d/scene/ParticleSystem3D.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/PlanarReflector3D.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/Ragdoll3D.h"
#include "prisma3d/scene/ReflectionProbe3D.h"
#include "prisma3d/scene/RibbonTrail3D.h"
#include "prisma3d/scene/Road3D.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/SkeletonModifier3D.h"
#include "prisma3d/scene/SkidMarks3D.h"
#include "prisma3d/scene/SoftBody3D.h"
#include "prisma3d/scene/SpringBoneCollision3D.h"
#include "prisma3d/scene/SpringBoneSimulator3D.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Traffic3D.h"
#include "prisma3d/scene/Tree3D.h"
#include "prisma3d/scene/Vehicle3D.h"
#include "prisma3d/scene/VehicleDriver3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/Water3D.h"
#include "prisma3d/terrain/TerrainBrush.h"

#include "platform.h"

#include <stdio.h>
#include <string.h>
#include <type_traits>

namespace prisma3d
{

namespace
{

const float kDegreesToRadians = 0.017453292519943295f;

struct EventData
{
    InputEvent event;
    ct::String action;
};

ZenScripting::Impl& impl()
{
    return *ZenScripting::instance()->getImpl();
}

bool isIdentifier(const ct::String& name)
{
    if (name.empty()) return false;
    for (std::size_t i = 0; i < name.size(); ++i)
    {
        const char c = name[i];
        const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
        const bool digit = c >= '0' && c <= '9';
        if (!(letter || (digit && i > 0))) return false;
    }
    return true;
}

ct::String attributeName(const ct::String& name)
{
    ct::String result;
    for (std::size_t i = 0; i < name.size(); ++i) result += name[i] == '/' ? '_' : name[i];
    return result;
}

zen::Value makeString(zen::VM* vm, const ct::String& text)
{
    return zen::val_obj(reinterpret_cast<zen::Obj*>(
            vm->make_string(text.c_str(), static_cast<int>(text.size()))));
}

zen::Value makeString(zen::VM* vm, const char* text)
{
    return zen::val_obj(reinterpret_cast<zen::Obj*>(vm->make_string(text)));
}

zen::Value makeVec3(const Math::Vec3& v)
{
    return zen::val_vec3(v.x, v.y, v.z);
}

zen::Value makeVec2(const Math::Vec2& v)
{
    return zen::val_vec2(v.x, v.y);
}

Node* selfNode(zen::VM* vm, zen::Value* args)
{
    const zen::Value self = args[-1];
    if (!zen::is_instance(self))
    {
        vm->runtime_error("method called on something that is not a node");
        return nullptr;
    }
    Node* node = static_cast<Node*>(zen::as_instance(self)->native_data);
    if (!node) vm->runtime_error("the node was freed");
    return node;
}

template<typename T>
T* selfData(zen::VM* vm, zen::Value* args, const char* what)
{
    const zen::Value self = args[-1];
    if (!zen::is_instance(self) || !zen::as_instance(self)->native_data)
    {
        vm->runtime_error("%s is not available", what);
        return nullptr;
    }
    return static_cast<T*>(zen::as_instance(self)->native_data);
}

bool needArgs(zen::VM* vm, int nargs, int minimum, int maximum, const char* name)
{
    if (nargs >= minimum && nargs <= maximum) return true;
    if (minimum == maximum)
        vm->runtime_error("%s: expected %d argument(s), got %d", name, minimum, nargs);
    else
        vm->runtime_error("%s: expected %d to %d arguments, got %d", name, minimum, maximum, nargs);
    return false;
}

bool argString(zen::VM* vm, zen::Value* args, int index, const char* name, ct::String& out)
{
    if (!zen::is_string(args[index]))
    {
        vm->runtime_error("%s: argument %d must be a string", name, index + 1);
        return false;
    }
    out = ct::String(zen::safe_string_chars(args[index]),
            static_cast<std::size_t>(zen::safe_string_len(args[index])));
    return true;
}

bool argNumber(zen::VM* vm, zen::Value* args, int index, const char* name, double& out)
{
    if (!zen::is_numeric_like(args[index]))
    {
        vm->runtime_error("%s: argument %d must be a number", name, index + 1);
        return false;
    }
    out = zen::to_number(args[index]);
    return true;
}

bool argVec3(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Vec3& out)
{
    if (!zen::is_vec3(args[index]))
    {
        vm->runtime_error("%s: argument %d must be a Vec3", name, index + 1);
        return false;
    }
    out = Math::Vec3(zen::vec_x(args[index]), zen::vec_y(args[index]), zen::vec_z(args[index]));
    return true;
}

bool argNode(zen::VM* vm, zen::Value* args, int index, const char* name, Node*& out)
{
    if (!impl().isNodeValue(args[index]))
    {
        vm->runtime_error("%s: argument %d must be a live node", name, index + 1);
        return false;
    }
    out = impl().nodeOf(args[index]);
    return true;
}

bool argMask(zen::VM* vm, zen::Value* args, int nargs, int index, const char* name,
        std::uint32_t& out)
{
    out = 0xFFFFFFFFu;
    if (index >= nargs || zen::is_nil(args[index])) return true;
    double value = 0.0;
    if (!argNumber(vm, args, index, name, value)) return false;
    if (!(value >= 0.0 && value <= 4294967295.0))
    {
        vm->runtime_error("%s: argument %d must be a collision mask between 0 and 4294967295", name,
                index + 1);
        return false;
    }
    out = static_cast<std::uint32_t>(value);
    return true;
}

bool argPoint2(zen::VM* vm, zen::Value* args, int nargs, const char* name, Math::Vec2& out)
{
    if (nargs == 1 && zen::is_vec2(args[0]))
    {
        out = Math::Vec2(zen::vec_x(args[0]), zen::vec_y(args[0]));
        return true;
    }
    double x = 0.0;
    double y = 0.0;
    if (nargs != 2)
    {
        vm->runtime_error("%s: expected a Vec2 or 2 numbers, got %d argument(s)", name, nargs);
        return false;
    }
    if (!argNumber(vm, args, 0, name, x) || !argNumber(vm, args, 1, name, y)) return false;
    out = Math::Vec2(static_cast<float>(x), static_cast<float>(y));
    return true;
}

int returnNode(zen::VM*, zen::Value* args, Node* node)
{
    args[0] = node ? impl().wrap(node) : zen::val_nil();
    return 1;
}

int returnNodes(zen::VM* vm, zen::Value* args, const ct::Vector<Node*>& nodes)
{
    zen::GC* gc = &vm->get_gc();
    zen::ObjArray* array = zen::new_array(gc);
    args[0] = zen::val_obj(reinterpret_cast<zen::Obj*>(array));
    for (std::size_t i = 0; i < nodes.size(); ++i)
        zen::array_push(gc, array, impl().wrap(nodes[i]));
    return 1;
}

bool propertyInfoFor(Node* node, const ct::String& name, PropertyInfo& info)
{
    if (NodeClassDB::getPropertyInfo(node->getClassName(), name, info)) return true;
    const ScriptInstance* script = node->getScriptInstance();
    if (!script || !script->hasScript()) return false;
    ct::Vector<PropertyInfo> exported;
    script->getPropertyList(exported);
    for (std::size_t i = 0; i < exported.size(); ++i)
    {
        if (exported[i].name == name)
        {
            info = exported[i];
            return true;
        }
    }
    return false;
}

int propertyGet(zen::VM* vm, int slot, zen::Value* args)
{
    Node* node = selfNode(vm, args);
    if (!node) return -1;
    ZenScripting::Impl& scripting = impl();
    const ct::String& name = scripting.propertyName(slot);
    PropertyValue value;
    if (!NodeClassDB::getProperty(node, name, value))
    {
        vm->runtime_error("%s.%s cannot be read", node->getClassName(), name.c_str());
        return -1;
    }
    args[0] = scripting.toZen(value);
    return 1;
}

int propertySet(zen::VM* vm, int slot, zen::Value* args)
{
    Node* node = selfNode(vm, args);
    if (!node) return -1;
    ZenScripting::Impl& scripting = impl();
    const ct::String& name = scripting.propertyName(slot);
    PropertyInfo info;
    if (!NodeClassDB::getPropertyInfo(node->getClassName(), name, info))
    {
        vm->runtime_error("%s has no property '%s'", node->getClassName(), name.c_str());
        return -1;
    }
    PropertyValue converted;
    ct::String error;
    if (!scripting.fromZen(args[0], info.type, converted, error))
    {
        vm->runtime_error("%s.%s: %s", node->getClassName(), name.c_str(), error.c_str());
        return -1;
    }
    if (!NodeClassDB::setProperty(node, name, converted))
    {
        vm->runtime_error("%s.%s is read-only", node->getClassName(), name.c_str());
        return -1;
    }
    return 0;
}

template<int N>
int getThunk(zen::VM* vm, zen::Value* args, int)
{
    return propertyGet(vm, N, args);
}

template<int N>
int setThunk(zen::VM* vm, zen::Value* args, int)
{
    return propertySet(vm, N, args);
}

template<int Lo, int Count>
struct ThunkFill
{
    static void run(zen::NativeFn* getters, zen::NativeFn* setters)
    {
        ThunkFill<Lo, Count / 2>::run(getters, setters);
        ThunkFill<Lo + Count / 2, Count - Count / 2>::run(getters, setters);
    }
};

template<int Lo>
struct ThunkFill<Lo, 1>
{
    static void run(zen::NativeFn* getters, zen::NativeFn* setters)
    {
        getters[Lo] = &getThunk<Lo>;
        setters[Lo] = &setThunk<Lo>;
    }
};

constexpr int kMaxPropertySlots = 1024;

zen::NativeFn sGetters[kMaxPropertySlots];
zen::NativeFn sSetters[kMaxPropertySlots];
bool sThunksReady = false;

void ensureThunks()
{
    if (sThunksReady) return;
    ThunkFill<0, kMaxPropertySlots>::run(sGetters, sSetters);
    sThunksReady = true;
}

int nodeGetName(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = makeString(vm, self->getName());
    return 1;
}

int nodeSetName(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String name;
    if (!self || !needArgs(vm, nargs, 1, 1, "set_name") ||
            !argString(vm, args, 0, "set_name", name))
        return -1;
    self->setName(name);
    return 0;
}

int nodeGetParent(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    return returnNode(vm, args, self->getParent());
}

int nodeGetChildCount(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_int(static_cast<long long>(self->getChildCount()));
    return 1;
}

int nodeGetChild(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    double index = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_child") ||
            !argNumber(vm, args, 0, "get_child", index))
        return -1;
    if (index < 0.0) return returnNode(vm, args, nullptr);
    return returnNode(vm, args, self->getChild(static_cast<std::size_t>(index)));
}

int nodeGetChildren(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    ct::Vector<Node*> children;
    for (std::size_t i = 0; i < self->getChildCount(); ++i) children.push_back(self->getChild(i));
    return returnNodes(vm, args, children);
}

int nodeGetIndex(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_int(static_cast<long long>(self->getIndex()));
    return 1;
}

int nodeGetNode(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String path;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_node") ||
            !argString(vm, args, 0, "get_node", path))
        return -1;
    return returnNode(vm, args, self->getNode(path));
}

Node* findDescendant(Node* node, const ct::String& name, bool recursive)
{
    if (Node* direct = node->findChild(name)) return direct;
    if (!recursive) return nullptr;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (Node* found = findDescendant(node->getChild(i), name, true)) return found;
    return nullptr;
}

int nodeFindChild(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String name;
    if (!self || !needArgs(vm, nargs, 1, 2, "find_child") ||
            !argString(vm, args, 0, "find_child", name))
        return -1;
    const bool recursive = nargs >= 2 ? zen::is_truthy_full(args[1]) : true;
    return returnNode(vm, args, findDescendant(self, name, recursive));
}

int nodeCreateChild(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String className;
    if (!self || !needArgs(vm, nargs, 1, 2, "create_child") ||
            !argString(vm, args, 0, "create_child", className))
        return -1;
    ct::String script;
    if (nargs == 2 && !zen::is_nil(args[1]) && !argString(vm, args, 1, "create_child", script))
        return -1;
    if (!NodeClassDB::classExists(className) || !NodeClassDB::isParentClass(className, "Node"))
    {
        vm->runtime_error("create_child: '%s' is not a node class", className.c_str());
        return -1;
    }
    if (!NodeClassDB::canInstantiate(className))
    {
        vm->runtime_error("create_child: '%s' is abstract", className.c_str());
        return -1;
    }
    Node* child = self->createChildWithScript(className.c_str(), script);
    if (!child)
    {
        vm->runtime_error("create_child: cannot create '%s'", className.c_str());
        return -1;
    }
    return returnNode(vm, args, child);
}

int nodeQueueFree(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    self->queueFree();
    return 0;
}

int nodeIsQueuedForDeletion(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isQueuedForDeletion());
    return 1;
}

int nodeAddToGroup(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String group;
    if (!self || !needArgs(vm, nargs, 1, 1, "add_to_group") ||
            !argString(vm, args, 0, "add_to_group", group))
        return -1;
    self->addToGroup(group);
    return 0;
}

int nodeRemoveFromGroup(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String group;
    if (!self || !needArgs(vm, nargs, 1, 1, "remove_from_group") ||
            !argString(vm, args, 0, "remove_from_group", group))
        return -1;
    self->removeFromGroup(group);
    return 0;
}

int nodeIsInGroup(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String group;
    if (!self || !needArgs(vm, nargs, 1, 1, "is_in_group") ||
            !argString(vm, args, 0, "is_in_group", group))
        return -1;
    args[0] = zen::val_bool(self->isInGroup(group));
    return 1;
}

int nodeGetGroups(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    ct::Vector<Node::GroupInfo> groups;
    self->getGroups(groups);
    zen::GC* gc = &vm->get_gc();
    zen::ObjArray* array = zen::new_array(gc);
    args[0] = zen::val_obj(reinterpret_cast<zen::Obj*>(array));
    for (std::size_t i = 0; i < groups.size(); ++i)
        zen::array_push(gc, array, makeString(vm, groups[i].name));
    return 1;
}

int nodeGetTree(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = impl().sceneTreeValue(self->getTree());
    return 1;
}

int nodeIsInsideTree(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isInsideTree());
    return 1;
}

int nodeIsReady(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isReady());
    return 1;
}

int nodeSetProcess(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    if (!self || !needArgs(vm, nargs, 1, 1, "set_process")) return -1;
    self->setProcess(zen::is_truthy_full(args[0]));
    return 0;
}

int nodeIsProcessing(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isProcessing());
    return 1;
}

int nodeSetPhysicsProcess(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    if (!self || !needArgs(vm, nargs, 1, 1, "set_physics_process")) return -1;
    self->setPhysicsProcess(zen::is_truthy_full(args[0]));
    return 0;
}

int nodeIsPhysicsProcessing(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isPhysicsProcessing());
    return 1;
}

int nodeSetProcessInput(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    if (!self || !needArgs(vm, nargs, 1, 1, "set_process_input")) return -1;
    self->setProcessInput(zen::is_truthy_full(args[0]));
    return 0;
}

int nodeIsProcessingInput(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isProcessingInput());
    return 1;
}

int nodeCanProcess(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->canProcess());
    return 1;
}

int nodeGetPath(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = makeString(vm, self->getPath());
    return 1;
}

int nodeGetClass(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = makeString(vm, self->getClassName());
    return 1;
}

int nodeIsClass(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String name;
    if (!self || !needArgs(vm, nargs, 1, 1, "is_class") ||
            !argString(vm, args, 0, "is_class", name))
        return -1;
    args[0] = zen::val_bool(self->isClass(name.c_str()));
    return 1;
}

int nodeReparent(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    Node* parent = nullptr;
    if (!self || !needArgs(vm, nargs, 1, 1, "reparent") ||
            !argNode(vm, args, 0, "reparent", parent))
        return -1;
    args[0] = zen::val_bool(self->reparent(parent));
    return 1;
}

int nodeGetScript(zen::VM* vm, zen::Value* args, int)
{
    Node* self = selfNode(vm, args);
    if (!self) return -1;
    args[0] = makeString(vm, self->getScript());
    return 1;
}

int nodeGet(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String name;
    if (!self || !needArgs(vm, nargs, 1, 1, "get") || !argString(vm, args, 0, "get", name))
        return -1;
    PropertyValue value;
    if (!NodeClassDB::getProperty(self, name, value))
    {
        vm->runtime_error("get: %s has no property '%s'", self->getClassName(), name.c_str());
        return -1;
    }
    args[0] = impl().toZen(value);
    return 1;
}

int nodeSet(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String name;
    if (!self || !needArgs(vm, nargs, 2, 2, "set") || !argString(vm, args, 0, "set", name))
        return -1;
    PropertyInfo info;
    if (!propertyInfoFor(self, name, info))
    {
        vm->runtime_error("set: %s has no property '%s'", self->getClassName(), name.c_str());
        return -1;
    }
    PropertyValue converted;
    ct::String error;
    if (!impl().fromZen(args[1], info.type, converted, error))
    {
        vm->runtime_error("set: %s.%s: %s", self->getClassName(), name.c_str(), error.c_str());
        return -1;
    }
    if (!NodeClassDB::setProperty(self, name, converted))
    {
        vm->runtime_error("set: %s.%s is read-only", self->getClassName(), name.c_str());
        return -1;
    }
    return 0;
}

Node3D* selfNode3D(zen::VM* vm, zen::Value* args)
{
    Node* node = selfNode(vm, args);
    if (!node) return nullptr;
    Node3D* node3d = node->castTo<Node3D>();
    if (!node3d) vm->runtime_error("%s is not a Node3D", node->getClassName());
    return node3d;
}

template<typename T>
T* selfNodeAs(zen::VM* vm, zen::Value* args)
{
    Node* node = selfNode(vm, args);
    if (!node) return nullptr;
    T* typed = node->castTo<T>();
    if (!typed) vm->runtime_error("%s is not a %s", node->getClassName(), T::classNameStatic());
    return typed;
}

int node3dTranslate(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 offset;
    if (!self || !needArgs(vm, nargs, 1, 1, "translate") ||
            !argVec3(vm, args, 0, "translate", offset))
        return -1;
    self->translate(offset);
    return 0;
}

int node3dTranslateObjectLocal(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 offset;
    if (!self || !needArgs(vm, nargs, 1, 1, "translate_object_local") ||
            !argVec3(vm, args, 0, "translate_object_local", offset))
        return -1;
    self->translateObjectLocal(offset);
    return 0;
}

int node3dRotate(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 axis;
    double angle = 0.0;
    if (!self || !needArgs(vm, nargs, 2, 2, "rotate") || !argVec3(vm, args, 0, "rotate", axis) ||
            !argNumber(vm, args, 1, "rotate", angle))
        return -1;
    self->rotate(axis, static_cast<float>(angle));
    return 0;
}

int node3dRotateObjectLocal(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 axis;
    double angle = 0.0;
    if (!self || !needArgs(vm, nargs, 2, 2, "rotate_object_local") ||
            !argVec3(vm, args, 0, "rotate_object_local", axis) ||
            !argNumber(vm, args, 1, "rotate_object_local", angle))
        return -1;
    self->rotateObjectLocal(axis, static_cast<float>(angle));
    return 0;
}

int node3dRotateX(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double angle = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "rotate_x") ||
            !argNumber(vm, args, 0, "rotate_x", angle))
        return -1;
    self->rotateX(static_cast<float>(angle));
    return 0;
}

int node3dRotateY(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double angle = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "rotate_y") ||
            !argNumber(vm, args, 0, "rotate_y", angle))
        return -1;
    self->rotateY(static_cast<float>(angle));
    return 0;
}

int node3dRotateZ(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double angle = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "rotate_z") ||
            !argNumber(vm, args, 0, "rotate_z", angle))
        return -1;
    self->rotateZ(static_cast<float>(angle));
    return 0;
}

int node3dYaw(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double degrees = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "yaw") || !argNumber(vm, args, 0, "yaw", degrees))
        return -1;
    self->rotateY(static_cast<float>(degrees) * kDegreesToRadians);
    return 0;
}

int node3dPitch(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double degrees = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "pitch") || !argNumber(vm, args, 0, "pitch", degrees))
        return -1;
    self->rotateObjectLocal(Math::Vec3(1.0f, 0.0f, 0.0f),
            static_cast<float>(degrees) * kDegreesToRadians);
    return 0;
}

int node3dRoll(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double degrees = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "roll") || !argNumber(vm, args, 0, "roll", degrees))
        return -1;
    self->rotateObjectLocal(Math::Vec3(0.0f, 0.0f, 1.0f),
            static_cast<float>(degrees) * kDegreesToRadians);
    return 0;
}

int node3dMoveForward(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double distance = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "move_forward") ||
            !argNumber(vm, args, 0, "move_forward", distance))
        return -1;
    self->translateObjectLocal(Math::Vec3(0.0f, 0.0f, -static_cast<float>(distance)));
    return 0;
}

int node3dMoveRight(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double distance = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "move_right") ||
            !argNumber(vm, args, 0, "move_right", distance))
        return -1;
    self->translateObjectLocal(Math::Vec3(static_cast<float>(distance), 0.0f, 0.0f));
    return 0;
}

int node3dMoveUp(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    double distance = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "move_up") ||
            !argNumber(vm, args, 0, "move_up", distance))
        return -1;
    self->translateObjectLocal(Math::Vec3(0.0f, static_cast<float>(distance), 0.0f));
    return 0;
}

int node3dLookAt(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 target;
    Math::Vec3 up(0.0f, 1.0f, 0.0f);
    if (!self || !needArgs(vm, nargs, 1, 2, "look_at") || !argVec3(vm, args, 0, "look_at", target))
        return -1;
    if (nargs == 2 && !argVec3(vm, args, 1, "look_at", up)) return -1;
    self->lookAt(target, up);
    return 0;
}

int node3dLookAtFromPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 position;
    Math::Vec3 target;
    Math::Vec3 up(0.0f, 1.0f, 0.0f);
    if (!self || !needArgs(vm, nargs, 2, 3, "look_at_from_position") ||
            !argVec3(vm, args, 0, "look_at_from_position", position) ||
            !argVec3(vm, args, 1, "look_at_from_position", target))
        return -1;
    if (nargs == 3 && !argVec3(vm, args, 2, "look_at_from_position", up)) return -1;
    self->lookAtFromPosition(position, target, up);
    return 0;
}

int node3dToLocal(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 point;
    if (!self || !needArgs(vm, nargs, 1, 1, "to_local") || !argVec3(vm, args, 0, "to_local", point))
        return -1;
    args[0] = makeVec3(self->toLocal(point));
    return 1;
}

int node3dToGlobal(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    Math::Vec3 point;
    if (!self || !needArgs(vm, nargs, 1, 1, "to_global") ||
            !argVec3(vm, args, 0, "to_global", point))
        return -1;
    args[0] = makeVec3(self->toGlobal(point));
    return 1;
}

int returnDirection(zen::VM* vm, zen::Value* args, const Math::Vec3& local)
{
    Node3D* self = selfNode3D(vm, args);
    if (!self) return -1;
    const Math::Vec3 origin = self->toGlobal(Math::Vec3(0.0f, 0.0f, 0.0f));
    const Math::Vec3 tip = self->toGlobal(local);
    args[0] = makeVec3((tip - origin).NormalizedSafe());
    return 1;
}

int node3dForward(zen::VM* vm, zen::Value* args, int)
{
    return returnDirection(vm, args, Math::Vec3(0.0f, 0.0f, -1.0f));
}

int node3dRight(zen::VM* vm, zen::Value* args, int)
{
    return returnDirection(vm, args, Math::Vec3(1.0f, 0.0f, 0.0f));
}

int node3dUp(zen::VM* vm, zen::Value* args, int)
{
    return returnDirection(vm, args, Math::Vec3(0.0f, 1.0f, 0.0f));
}

int node3dShow(zen::VM* vm, zen::Value* args, int)
{
    Node3D* self = selfNode3D(vm, args);
    if (!self) return -1;
    self->show();
    return 0;
}

int node3dHide(zen::VM* vm, zen::Value* args, int)
{
    Node3D* self = selfNode3D(vm, args);
    if (!self) return -1;
    self->hide();
    return 0;
}

int node3dIsVisibleInTree(zen::VM* vm, zen::Value* args, int)
{
    Node3D* self = selfNode3D(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isVisibleInTree());
    return 1;
}

int node3dCountCollisions(zen::VM* vm, zen::Value* args, int)
{
    Node3D* self = selfNode3D(vm, args);
    if (!self) return -1;
    args[0] = zen::val_int(static_cast<long long>(self->countCollisions()));
    return 1;
}

bool collisionIndex(zen::VM* vm, Node3D* self, zen::Value* args, int nargs, const char* name,
        std::uint32_t& index)
{
    double value = 0.0;
    if (!needArgs(vm, nargs, 1, 1, name) || !argNumber(vm, args, 0, name, value)) return false;
    if (value < 0.0 || value >= static_cast<double>(self->countCollisions()))
    {
        vm->runtime_error("%s: collision index %d is out of range", name, static_cast<int>(value));
        return false;
    }
    index = static_cast<std::uint32_t>(value);
    return true;
}

int node3dGetCollisionNode(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    std::uint32_t index = 0;
    if (!self || !collisionIndex(vm, self, args, nargs, "get_collision_node", index)) return -1;
    return returnNode(vm, args, self->getCollision(index).node);
}

int node3dGetCollisionPoint(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    std::uint32_t index = 0;
    if (!self || !collisionIndex(vm, self, args, nargs, "get_collision_point", index)) return -1;
    args[0] = makeVec3(self->getCollision(index).point);
    return 1;
}

int node3dGetCollisionNormal(zen::VM* vm, zen::Value* args, int nargs)
{
    Node3D* self = selfNode3D(vm, args);
    std::uint32_t index = 0;
    if (!self || !collisionIndex(vm, self, args, nargs, "get_collision_normal", index)) return -1;
    args[0] = makeVec3(self->getCollision(index).normal);
    return 1;
}

int rigidSetVector(zen::VM* vm, zen::Value* args, int nargs, const char* name,
        void (RigidBody3D::*apply)(const Math::Vec3&))
{
    RigidBody3D* self = selfNodeAs<RigidBody3D>(vm, args);
    Math::Vec3 value;
    if (!self || !needArgs(vm, nargs, 1, 1, name) || !argVec3(vm, args, 0, name, value)) return -1;
    (self->*apply)(value);
    return 0;
}

int rigidSetVectorAt(zen::VM* vm, zen::Value* args, int nargs, const char* name,
        void (RigidBody3D::*apply)(const Math::Vec3&, const Math::Vec3&))
{
    RigidBody3D* self = selfNodeAs<RigidBody3D>(vm, args);
    Math::Vec3 value;
    Math::Vec3 offset;
    if (!self || !needArgs(vm, nargs, 2, 2, name) || !argVec3(vm, args, 0, name, value) ||
            !argVec3(vm, args, 1, name, offset))
        return -1;
    (self->*apply)(value, offset);
    return 0;
}

int rigidGetVector(zen::VM* vm, zen::Value* args, Math::Vec3 (RigidBody3D::*read)() const)
{
    RigidBody3D* self = selfNodeAs<RigidBody3D>(vm, args);
    if (!self) return -1;
    args[0] = makeVec3((self->*read)());
    return 1;
}

int rigidApplyCentralImpulse(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVector(vm, args, nargs, "apply_central_impulse",
            &RigidBody3D::applyCentralImpulse);
}

int rigidApplyImpulse(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVectorAt(vm, args, nargs, "apply_impulse", &RigidBody3D::applyImpulse);
}

int rigidApplyTorqueImpulse(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVector(vm, args, nargs, "apply_torque_impulse",
            &RigidBody3D::applyTorqueImpulse);
}

int rigidApplyCentralForce(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVector(vm, args, nargs, "apply_central_force", &RigidBody3D::applyCentralForce);
}

int rigidApplyForce(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVectorAt(vm, args, nargs, "apply_force", &RigidBody3D::applyForce);
}

int rigidApplyTorque(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVector(vm, args, nargs, "apply_torque", &RigidBody3D::applyTorque);
}

int rigidSetLinearVelocity(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVector(vm, args, nargs, "set_linear_velocity", &RigidBody3D::setLinearVelocity);
}

int rigidGetLinearVelocity(zen::VM* vm, zen::Value* args, int)
{
    return rigidGetVector(vm, args, &RigidBody3D::getLinearVelocity);
}

int rigidSetAngularVelocity(zen::VM* vm, zen::Value* args, int nargs)
{
    return rigidSetVector(vm, args, nargs, "set_angular_velocity",
            &RigidBody3D::setAngularVelocity);
}

int rigidGetAngularVelocity(zen::VM* vm, zen::Value* args, int)
{
    return rigidGetVector(vm, args, &RigidBody3D::getAngularVelocity);
}

int rigidGetVelocityAtPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    RigidBody3D* self = selfNodeAs<RigidBody3D>(vm, args);
    Math::Vec3 point;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_velocity_at_position") ||
            !argVec3(vm, args, 0, "get_velocity_at_position", point))
        return -1;
    args[0] = makeVec3(self->getVelocityAtPosition(point));
    return 1;
}

int cameraMakeCurrent(zen::VM* vm, zen::Value* args, int)
{
    Camera3D* self = selfNodeAs<Camera3D>(vm, args);
    if (!self) return -1;
    self->makeCurrent();
    return 0;
}

int cameraIsCurrent(zen::VM* vm, zen::Value* args, int)
{
    Camera3D* self = selfNodeAs<Camera3D>(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isCurrent());
    return 1;
}

bool cameraCanProject(zen::VM* vm, const Camera3D* camera, const char* name)
{
    const Viewport* viewport = camera->getViewport();
    if (camera->isInsideTree() && viewport && viewport->getWidth() > 0 && viewport->getHeight() > 0)
        return true;
    vm->runtime_error("%s: the camera is not inside a scene with a viewport", name);
    return false;
}

int cameraProjectRayOrigin(zen::VM* vm, zen::Value* args, int nargs)
{
    Camera3D* self = selfNodeAs<Camera3D>(vm, args);
    Math::Vec2 point;
    if (!self || !argPoint2(vm, args, nargs, "project_ray_origin", point) ||
            !cameraCanProject(vm, self, "project_ray_origin"))
        return -1;
    args[0] = makeVec3(self->projectRayOrigin(point));
    return 1;
}

int cameraProjectRayNormal(zen::VM* vm, zen::Value* args, int nargs)
{
    Camera3D* self = selfNodeAs<Camera3D>(vm, args);
    Math::Vec2 point;
    if (!self || !argPoint2(vm, args, nargs, "project_ray_normal", point) ||
            !cameraCanProject(vm, self, "project_ray_normal"))
        return -1;
    args[0] = makeVec3(self->projectRayNormal(point));
    return 1;
}

int animationPlay(zen::VM* vm, zen::Value* args, int nargs)
{
    AnimationPlayer* self = selfNodeAs<AnimationPlayer>(vm, args);
    if (!self || !needArgs(vm, nargs, 0, 4, "play")) return -1;
    ct::String name;
    double blend = -1.0;
    double speed = 1.0;
    if (nargs >= 1 && !argString(vm, args, 0, "play", name)) return -1;
    if (nargs >= 2 && !argNumber(vm, args, 1, "play", blend)) return -1;
    if (nargs >= 3 && !argNumber(vm, args, 2, "play", speed)) return -1;
    const bool fromEnd = nargs >= 4 && zen::is_truthy_full(args[3]);
    const ct::String target = name.empty() ? self->getAssignedAnimation() : name;
    if (!self->hasAnimation(target))
    {
        vm->runtime_error("play: animation '%s' not found", target.c_str());
        return -1;
    }
    self->play(name, blend, static_cast<float>(speed), fromEnd);
    return 0;
}

int animationImportAnimations(zen::VM* vm, zen::Value* args, int nargs)
{
    AnimationPlayer* self = selfNodeAs<AnimationPlayer>(vm, args);
    if (!self || !needArgs(vm, nargs, 1, 2, "import_animations")) return -1;
    ct::String path;
    ct::String prefix;
    if (!argString(vm, args, 0, "import_animations", path)) return -1;
    if (nargs >= 2 && !argString(vm, args, 1, "import_animations", prefix)) return -1;
    args[0] = zen::val_int(self->importAnimations(path, prefix));
    return 1;
}

int animationStop(zen::VM* vm, zen::Value* args, int nargs)
{
    AnimationPlayer* self = selfNodeAs<AnimationPlayer>(vm, args);
    if (!self || !needArgs(vm, nargs, 0, 1, "stop")) return -1;
    self->stop(nargs == 1 && zen::is_truthy_full(args[0]));
    return 0;
}

int animationIsPlaying(zen::VM* vm, zen::Value* args, int)
{
    AnimationPlayer* self = selfNodeAs<AnimationPlayer>(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isPlaying());
    return 1;
}

template<typename T>
int audioPlay(zen::VM* vm, zen::Value* args, int)
{
    T* self = selfNodeAs<T>(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->play());
    return 1;
}

template<typename T>
int audioStop(zen::VM* vm, zen::Value* args, int)
{
    T* self = selfNodeAs<T>(vm, args);
    if (!self) return -1;
    self->stop();
    return 0;
}

template<typename T>
int audioIsPlaying(zen::VM* vm, zen::Value* args, int)
{
    T* self = selfNodeAs<T>(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isPlaying());
    return 1;
}

template<typename T>
int audioGetPlaybackPosition(zen::VM* vm, zen::Value* args, int)
{
    T* self = selfNodeAs<T>(vm, args);
    if (!self) return -1;
    args[0] = zen::val_float(self->getPlaybackPosition());
    return 1;
}

template<typename T>
int audioSeek(zen::VM* vm, zen::Value* args, int nargs)
{
    T* self = selfNodeAs<T>(vm, args);
    double seconds = 0.0;
    if (!self || !needArgs(vm, nargs, 1, 1, "seek") || !argNumber(vm, args, 0, "seek", seconds)) return -1;
    args[0] = zen::val_bool(self->seek(static_cast<float>(seconds)));
    return 1;
}

SceneTree* selfTree(zen::VM* vm, zen::Value* args)
{
    return selfData<SceneTree>(vm, args, "the scene tree");
}

int treeGetNodesInGroup(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    ct::String group;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_nodes_in_group") ||
            !argString(vm, args, 0, "get_nodes_in_group", group))
        return -1;
    ct::Vector<Node*> nodes;
    self->getNodesInGroup(group, nodes);
    return returnNodes(vm, args, nodes);
}

int treeGetFirstNodeInGroup(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    ct::String group;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_first_node_in_group") ||
            !argString(vm, args, 0, "get_first_node_in_group", group))
        return -1;
    return returnNode(vm, args, self->getFirstNodeInGroup(group));
}

int treeHasGroup(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    ct::String group;
    if (!self || !needArgs(vm, nargs, 1, 1, "has_group") ||
            !argString(vm, args, 0, "has_group", group))
        return -1;
    args[0] = zen::val_bool(self->hasGroup(group));
    return 1;
}

int treeGetNodeCountInGroup(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    ct::String group;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_node_count_in_group") ||
            !argString(vm, args, 0, "get_node_count_in_group", group))
        return -1;
    args[0] = zen::val_int(static_cast<long long>(self->getNodeCountInGroup(group)));
    return 1;
}

ct::String resolveScenePath(const ct::String& path)
{
    Project* project = Main::project();
    if (project && project->isOpen())
    {
        const ct::String absolute = project->absolutePath(path);
        if (file_exists(absolute.c_str())) return absolute;
    }
    return path;
}

int nodeInstantiate(zen::VM* vm, zen::Value* args, int nargs)
{
    Node* self = selfNode(vm, args);
    ct::String path;
    if (!self || !needArgs(vm, nargs, 1, 1, "instantiate") ||
            !argString(vm, args, 0, "instantiate", path))
        return -1;
    SceneResult result;
    Node* instance = Prefab::instantiate(resolveScenePath(path), self, result);
    if (!instance || result.errorCount() > 0)
    {
        const ct::String detail = result.describe();
        impl().reportMessage("instantiate '%s': %s", path.c_str(),
                detail.empty() ? "failed" : detail.c_str());
    }
    return returnNode(vm, args, instance);
}

int treeChangeScene(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    ct::String path;
    ct::String effect;
    double seconds = 0.5;
    if (!self || !needArgs(vm, nargs, 1, 3, "change_scene") ||
            !argString(vm, args, 0, "change_scene", path))
        return -1;
    if (nargs >= 2 && !argString(vm, args, 1, "change_scene", effect)) return -1;
    if (nargs >= 3 && !argNumber(vm, args, 2, "change_scene", seconds)) return -1;
    const SceneTree::Transition transition = SceneTree::transitionFromName(effect);
    if (!effect.empty() && transition == SceneTree::Transition::None)
        log_warn("script: change_scene: unknown transition '%s'", effect.c_str());
    self->changeSceneToFile(resolveScenePath(path), transition, seconds);
    return 0;
}

int treeReloadCurrentScene(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->reloadCurrentScene());
    return 1;
}

int treeQuit(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    double code = 0.0;
    if (!self || !needArgs(vm, nargs, 0, 1, "quit")) return -1;
    if (nargs == 1 && !argNumber(vm, args, 0, "quit", code)) return -1;
    self->quit(static_cast<int>(code));
    return 0;
}

int treeSetPaused(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    if (!self || !needArgs(vm, nargs, 1, 1, "set_paused")) return -1;
    self->setPaused(zen::is_truthy_full(args[0]));
    return 0;
}

int treeSetAutoAcceptQuit(zen::VM* vm, zen::Value* args, int nargs)
{
    SceneTree* self = selfTree(vm, args);
    if (!self || !needArgs(vm, nargs, 1, 1, "set_auto_accept_quit")) return -1;
    self->setAutoAcceptQuit(zen::is_truthy_full(args[0]));
    return 0;
}

int treeIsAutoAcceptQuit(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isAutoAcceptQuit());
    return 1;
}

int treeIsPaused(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isPaused());
    return 1;
}

int treeGetRoot(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    return returnNode(vm, args, self->getRoot());
}

int treeGetCurrentScene(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    return returnNode(vm, args, self->getCurrentScene());
}

int treeGetFrame(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    args[0] = zen::val_int(static_cast<long long>(self->getFrame()));
    return 1;
}

int treeGetNodeCount(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    args[0] = zen::val_int(static_cast<long long>(self->getNodeCount()));
    return 1;
}

int treeSetInputAsHandled(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    self->setInputAsHandled();
    return 0;
}

int treeGetProcessDelta(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    args[0] = zen::val_float(self->getProcessDeltaTime());
    return 1;
}

int treeGetPhysicsDelta(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* self = selfTree(vm, args);
    if (!self) return -1;
    args[0] = zen::val_float(self->getPhysicsProcessDeltaTime());
    return 1;
}

bool actionName(zen::VM* vm, zen::Value* args, int nargs, const char* name, ct::String& out)
{
    return needArgs(vm, nargs, 1, 1, name) && argString(vm, args, 0, name, out);
}

int inputIsActionPressed(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_check", action)) return -1;
    const Input* input = Input::singleton();
    args[0] = zen::val_bool(input && input->isActionPressed(action.c_str()));
    return 1;
}

int inputIsActionJustPressed(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_check_pressed", action)) return -1;
    const Input* input = Input::singleton();
    args[0] = zen::val_bool(input && input->isActionJustPressed(action.c_str()));
    return 1;
}

int inputIsActionJustReleased(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_check_released", action)) return -1;
    const Input* input = Input::singleton();
    args[0] = zen::val_bool(input && input->isActionJustReleased(action.c_str()));
    return 1;
}

int inputGetActionStrength(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_strength", action)) return -1;
    const Input* input = Input::singleton();
    args[0] = zen::val_float(input ? input->getActionStrength(action.c_str()) : 0.0f);
    return 1;
}

int inputGetAxis(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String negative;
    ct::String positive;
    if (!needArgs(vm, nargs, 2, 2, "action_axis") ||
            !argString(vm, args, 0, "action_axis", negative) ||
            !argString(vm, args, 1, "action_axis", positive))
        return -1;
    const Input* input = Input::singleton();
    args[0] = zen::val_float(input ? input->getAxis(negative.c_str(), positive.c_str()) : 0.0f);
    return 1;
}

int inputGetVector(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String names[4];
    if (!needArgs(vm, nargs, 4, 5, "action_vector")) return -1;
    for (int i = 0; i < 4; ++i)
        if (!argString(vm, args, i, "action_vector", names[i])) return -1;
    double deadzone = -1.0;
    if (nargs == 5 && !argNumber(vm, args, 4, "action_vector", deadzone)) return -1;
    const Input* input = Input::singleton();
    Math::Vec2 result(0.0f, 0.0f);
    if (input)
        result = input->getVector(names[0].c_str(), names[1].c_str(), names[2].c_str(),
                names[3].c_str(), static_cast<float>(deadzone));
    args[0] = makeVec2(result);
    return 1;
}

const int kAnyInput = -1;

bool argInputCode(zen::VM* vm, zen::Value* args, int nargs, const char* name, int& out)
{
    double code = 0.0;
    if (!needArgs(vm, nargs, 1, 1, name) || !argNumber(vm, args, 0, name, code)) return false;
    out = static_cast<int>(code);
    return true;
}

bool keyState(const Input& input, int key, bool (Input::*test)(Key) const)
{
    if (key != kAnyInput) return (input.*test)(static_cast<Key>(key));
    for (int i = 1; i < static_cast<int>(Key::Max); ++i)
        if ((input.*test)(static_cast<Key>(i))) return true;
    return false;
}

bool mouseState(const Input& input, int button, bool (Input::*test)(MouseButton) const)
{
    if (button != kAnyInput)
    {
        if (button < 0 || button >= static_cast<int>(MouseButton::Max)) return false;
        return (input.*test)(static_cast<MouseButton>(button));
    }
    for (int i = 0; i < static_cast<int>(MouseButton::Max); ++i)
        if ((input.*test)(static_cast<MouseButton>(i))) return true;
    return false;
}

int keyCheck(zen::VM* vm, zen::Value* args, int nargs, const char* name,
        bool (Input::*test)(Key) const)
{
    int key = 0;
    if (!argInputCode(vm, args, nargs, name, key)) return -1;
    const Input* input = Input::singleton();
    args[0] = zen::val_bool(input && keyState(*input, key, test));
    return 1;
}

int mouseCheck(zen::VM* vm, zen::Value* args, int nargs, const char* name,
        bool (Input::*test)(MouseButton) const)
{
    int button = 0;
    if (!argInputCode(vm, args, nargs, name, button)) return -1;
    const Input* input = Input::singleton();
    args[0] = zen::val_bool(input && mouseState(*input, button, test));
    return 1;
}

int keyboardCheck(zen::VM* vm, zen::Value* args, int nargs)
{
    return keyCheck(vm, args, nargs, "keyboard_check", &Input::isKeyPressed);
}

int keyboardCheckPressed(zen::VM* vm, zen::Value* args, int nargs)
{
    return keyCheck(vm, args, nargs, "keyboard_check_pressed", &Input::isKeyJustPressed);
}

int keyboardCheckReleased(zen::VM* vm, zen::Value* args, int nargs)
{
    return keyCheck(vm, args, nargs, "keyboard_check_released", &Input::isKeyJustReleased);
}

int mouseCheckButton(zen::VM* vm, zen::Value* args, int nargs)
{
    return mouseCheck(vm, args, nargs, "mouse_check_button", &Input::isMouseButtonPressed);
}

int mouseCheckButtonPressed(zen::VM* vm, zen::Value* args, int nargs)
{
    return mouseCheck(vm, args, nargs, "mouse_check_button_pressed",
            &Input::isMouseButtonJustPressed);
}

int mouseCheckButtonReleased(zen::VM* vm, zen::Value* args, int nargs)
{
    return mouseCheck(vm, args, nargs, "mouse_check_button_released",
            &Input::isMouseButtonJustReleased);
}

int userDir(zen::VM* vm, zen::Value* args, int)
{
    args[0] = makeString(vm, FileSystem::getSingleton().getUserDirectory());
    return 1;
}

int mouseX(zen::VM*, zen::Value* args, int)
{
    const Input* input = Input::singleton();
    args[0] = zen::val_float(input ? input->getMousePosition().x : 0.0f);
    return 1;
}

int mouseY(zen::VM*, zen::Value* args, int)
{
    const Input* input = Input::singleton();
    args[0] = zen::val_float(input ? input->getMousePosition().y : 0.0f);
    return 1;
}

int mouseDelta(zen::VM*, zen::Value* args, int)
{
    const Input* input = Input::singleton();
    args[0] = makeVec2(input ? input->getMouseDelta() : Math::Vec2(0.0f, 0.0f));
    return 1;
}

int mouseWheel(zen::VM*, zen::Value* args, int)
{
    const Input* input = Input::singleton();
    args[0] = zen::val_float(input ? input->getMouseWheel().y : 0.0f);
    return 1;
}

int inputGetMousePosition(zen::VM*, zen::Value* args, int)
{
    const Input* input = Input::singleton();
    args[0] = makeVec2(input ? input->getMousePosition() : Math::Vec2(0.0f, 0.0f));
    return 1;
}

int inputGetMouseVelocity(zen::VM*, zen::Value* args, int)
{
    Input* input = Input::singleton();
    args[0] = makeVec2(input ? input->getLastMouseVelocity() : Math::Vec2(0.0f, 0.0f));
    return 1;
}

int inputSetMouseMode(zen::VM* vm, zen::Value* args, int nargs)
{
    double mode = 0.0;
    if (!needArgs(vm, nargs, 1, 1, "mouse_set_mode") ||
            !argNumber(vm, args, 0, "mouse_set_mode", mode))
        return -1;
    const double last = static_cast<double>(MouseMode::Captured);
    if (!(mode >= 0.0 && mode <= last) || mode != static_cast<double>(static_cast<int>(mode)))
    {
        vm->runtime_error("mouse_set_mode: expected MOUSE_MODE_VISIBLE, MOUSE_MODE_HIDDEN or "
                          "MOUSE_MODE_CAPTURED");
        return -1;
    }
    if (Device* device = Device::singleton())
        device->mouseSetMode(static_cast<MouseMode>(static_cast<int>(mode)));
    return 0;
}

int inputGetMouseMode(zen::VM*, zen::Value* args, int)
{
    const Device* device = Device::singleton();
    const MouseMode mode = device ? device->mouseGetMode() : MouseMode::Visible;
    args[0] = zen::val_int(static_cast<long long>(mode));
    return 1;
}

int inputActionPress(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    double strength = 1.0;
    if (!needArgs(vm, nargs, 1, 2, "action_press") ||
            !argString(vm, args, 0, "action_press", action))
        return -1;
    if (nargs == 2 && !argNumber(vm, args, 1, "action_press", strength)) return -1;
    if (Input* input = Input::singleton())
        input->actionPress(action.c_str(), static_cast<float>(strength));
    return 0;
}

int inputActionRelease(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_release", action)) return -1;
    if (Input* input = Input::singleton()) input->actionRelease(action.c_str());
    return 0;
}

int mapHasAction(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_exists", action)) return -1;
    const InputMap* map = InputMap::singleton();
    args[0] = zen::val_bool(map && map->hasAction(action.c_str()));
    return 1;
}

int mapAddAction(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    double deadzone = InputMap::kDefaultDeadzone;
    if (!needArgs(vm, nargs, 1, 2, "action_add") || !argString(vm, args, 0, "action_add", action))
        return -1;
    if (nargs == 2 && !argNumber(vm, args, 1, "action_add", deadzone)) return -1;
    if (InputMap* map = InputMap::singleton())
        map->addAction(action.c_str(), static_cast<float>(deadzone));
    return 0;
}

int mapEraseAction(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_remove", action)) return -1;
    if (InputMap* map = InputMap::singleton()) map->eraseAction(action.c_str());
    return 0;
}

int mapActionAddKey(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    double key = 0.0;
    if (!needArgs(vm, nargs, 2, 2, "action_add_key") ||
            !argString(vm, args, 0, "action_add_key", action) ||
            !argNumber(vm, args, 1, "action_add_key", key))
        return -1;
    InputMap* map = InputMap::singleton();
    if (!map || !map->hasAction(action.c_str()))
    {
        vm->runtime_error("action_add_key: the action '%s' does not exist", action.c_str());
        return -1;
    }
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = static_cast<Key>(static_cast<int>(key));
    event.key.scancode = 0;
    event.key.pressed = true;
    event.key.echo = false;
    event.key.modifiers = 0;
    map->actionAddEvent(action.c_str(), event);
    return 0;
}

int mapActionAddMouseButton(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    double button = 0.0;
    if (!needArgs(vm, nargs, 2, 2, "action_add_mouse_button") ||
            !argString(vm, args, 0, "action_add_mouse_button", action) ||
            !argNumber(vm, args, 1, "action_add_mouse_button", button))
        return -1;
    InputMap* map = InputMap::singleton();
    if (!map || !map->hasAction(action.c_str()))
    {
        vm->runtime_error("action_add_mouse_button: the action '%s' does not exist",
                action.c_str());
        return -1;
    }
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = 0;
    event.mouseButton.y = 0;
    event.mouseButton.button = static_cast<MouseButton>(static_cast<int>(button));
    event.mouseButton.pressed = true;
    event.mouseButton.modifiers = 0;
    map->actionAddEvent(action.c_str(), event);
    return 0;
}

int mapActionEraseEvents(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String action;
    if (!actionName(vm, args, nargs, "action_clear", action)) return -1;
    if (InputMap* map = InputMap::singleton()) map->actionEraseEvents(action.c_str());
    return 0;
}

EventData* selfEvent(zen::VM* vm, zen::Value* args)
{
    return selfData<EventData>(vm, args, "the input event");
}

int eventMatches(zen::VM* vm, zen::Value* args, int nargs, const char* name, int mode)
{
    EventData* self = selfEvent(vm, args);
    ct::String action;
    if (!self || !needArgs(vm, nargs, 1, 1, name) || !argString(vm, args, 0, name, action))
        return -1;
    const InputMap* map = InputMap::singleton();
    bool pressed = false;
    float strength = 0.0f;
    float raw = 0.0f;
    int index = -1;
    const bool matched = map && map->eventGetActionStatus(self->event, action.c_str(), false,
                                        &pressed, &strength, &raw, &index);
    if (mode == 0) args[0] = zen::val_bool(matched);
    else if (mode == 1)
        args[0] = zen::val_bool(matched && pressed);
    else if (mode == 2)
        args[0] = zen::val_bool(matched && !pressed);
    else
        args[0] = zen::val_float(matched ? strength : 0.0f);
    return 1;
}

int eventIsAction(zen::VM* vm, zen::Value* args, int nargs)
{
    return eventMatches(vm, args, nargs, "is_action", 0);
}

int eventIsActionPressed(zen::VM* vm, zen::Value* args, int nargs)
{
    return eventMatches(vm, args, nargs, "is_action_pressed", 1);
}

int eventIsActionReleased(zen::VM* vm, zen::Value* args, int nargs)
{
    return eventMatches(vm, args, nargs, "is_action_released", 2);
}

int eventGetActionStrength(zen::VM* vm, zen::Value* args, int nargs)
{
    return eventMatches(vm, args, nargs, "get_action_strength", 3);
}

void eventDestroy(zen::VM*, void* data)
{
    delete static_cast<EventData*>(data);
}

int timeGetTicksMsec(zen::VM*, zen::Value* args, int)
{
    args[0] = zen::val_int(static_cast<long long>(time_nanos() / 1000000ull));
    return 1;
}

int timeGetTicksUsec(zen::VM*, zen::Value* args, int)
{
    args[0] = zen::val_int(static_cast<long long>(time_nanos() / 1000ull));
    return 1;
}

int timeGetDelta(zen::VM*, zen::Value* args, int)
{
    SceneTree* tree = SceneTree::singleton();
    args[0] = zen::val_float(tree ? tree->getProcessDeltaTime() : 0.0);
    return 1;
}

int timeGetPhysicsDelta(zen::VM*, zen::Value* args, int)
{
    SceneTree* tree = SceneTree::singleton();
    args[0] = zen::val_float(tree ? tree->getPhysicsProcessDeltaTime() : 0.0);
    return 1;
}

int timeGetFrames(zen::VM*, zen::Value* args, int)
{
    SceneTree* tree = SceneTree::singleton();
    args[0] = zen::val_int(tree ? static_cast<long long>(tree->getFrame()) : 0);
    return 1;
}

int engineIsEditorHint(zen::VM*, zen::Value* args, int)
{
    const Engine* engine = Engine::singleton();
    args[0] = zen::val_bool(engine && engine->isEditorHint());
    return 1;
}

int engineGetTimeScale(zen::VM*, zen::Value* args, int)
{
    const Engine* engine = Engine::singleton();
    args[0] = zen::val_float(engine ? engine->getTimeScale() : 1.0);
    return 1;
}

int engineSetTimeScale(zen::VM* vm, zen::Value* args, int nargs)
{
    double scale = 1.0;
    if (!needArgs(vm, nargs, 1, 1, "set_time_scale") ||
            !argNumber(vm, args, 0, "set_time_scale", scale))
        return -1;
    if (Engine* engine = Engine::singleton()) engine->setTimeScale(scale);
    return 0;
}

int audioSetBusPitch(zen::VM* vm, zen::Value* args, int nargs)
{
    double bus = 0.0;
    double pitch = 1.0;
    if (!needArgs(vm, nargs, 2, 2, "set_bus_pitch") ||
            !argNumber(vm, args, 0, "set_bus_pitch", bus) ||
            !argNumber(vm, args, 1, "set_bus_pitch", pitch))
        return -1;
    if (bus < 0.0 || bus >= static_cast<double>(AudioBus::Count))
    {
        vm->runtime_error("set_bus_pitch: unknown bus");
        return -1;
    }
    if (AudioServer* audio = AudioServer::singleton())
        audio->setBusPitch(static_cast<AudioBus>(static_cast<int>(bus)), static_cast<float>(pitch));
    return 0;
}

int audioGetBusPitch(zen::VM* vm, zen::Value* args, int nargs)
{
    double bus = 0.0;
    if (!needArgs(vm, nargs, 1, 1, "bus_pitch") || !argNumber(vm, args, 0, "bus_pitch", bus))
        return -1;
    if (bus < 0.0 || bus >= static_cast<double>(AudioBus::Count))
    {
        vm->runtime_error("bus_pitch: unknown bus");
        return -1;
    }
    const AudioServer* audio = AudioServer::singleton();
    args[0] = zen::val_float(
            audio ? audio->getBusPitch(static_cast<AudioBus>(static_cast<int>(bus))) : 1.0);
    return 1;
}

int engineGetPhysicsFraction(zen::VM*, zen::Value* args, int)
{
    const Engine* engine = Engine::singleton();
    args[0] = zen::val_float(engine ? engine->getPhysicsInterpolationFraction() : 1.0);
    return 1;
}

int engineGetFps(zen::VM*, zen::Value* args, int)
{
    const Engine* engine = Engine::singleton();
    args[0] = zen::val_float(engine ? engine->getFramesPerSecond() : 0.0);
    return 1;
}

enum RayHitField : int
{
    kHitNode = 0,
    kHitPosition,
    kHitNormal,
    kHitDistance,
    kHitTriangle,
    kHitSurface,
    kHitMaterial,
    kHitFieldCount
};

const char* const kRayHitFields[kHitFieldCount] = { "node", "position", "normal", "distance",
    "triangle", "surface", "material" };

int hitSurface(PhysicsBody3D* body, int triangle, MeshId& mesh)
{
    if (!body || triangle < 0) return -1;
    for (std::size_t i = 0; i < body->getChildCount(); ++i)
    {
        const CollisionShape3D* shape = body->getChild(i)->castTo<CollisionShape3D>();
        if (!shape || !shape->getConcaveMesh().valid()) continue;
        mesh = shape->getConcaveMesh();
        return shape->getSurfaceOfTriangle(triangle);
    }
    return -1;
}

ct::String surfaceMaterialName(MeshId mesh, int surface)
{
    const MeshStore* meshes = MeshStore::singleton();
    const MeshInfo* info = meshes ? meshes->get(mesh) : nullptr;
    if (!info || surface < 0 || static_cast<std::size_t>(surface) >= info->surfaces.size())
        return ct::String();
    const MaterialStore* materials = MaterialStore::singleton();
    const MaterialInfo* material =
            materials ? materials->get(info->surfaces[static_cast<std::size_t>(surface)].material)
                      : nullptr;
    return material ? material->name : ct::String();
}

void collectBodyNodes(const ct::Vector<PhysicsBodyId>& bodies, ct::Vector<Node*>& nodes)
{
    for (std::size_t i = 0; i < bodies.size(); ++i)
        if (PhysicsBody3D* body = PhysicsBody3D::fromBody(bodies[i])) nodes.push_back(body);
}

int physicsRaycast(zen::VM* vm, zen::Value* args, int nargs)
{
    PhysicsRayParameters parameters;
    if (!needArgs(vm, nargs, 2, 4, "raycast") ||
            !argVec3(vm, args, 0, "raycast", parameters.from) ||
            !argVec3(vm, args, 1, "raycast", parameters.to) ||
            !argMask(vm, args, nargs, 2, "raycast", parameters.collisionMask))
        return -1;
    if (nargs == 4 && !zen::is_nil(args[3]))
    {
        Node* excluded = nullptr;
        if (!argNode(vm, args, 3, "raycast", excluded)) return -1;
        PhysicsBody3D* body = excluded->castTo<PhysicsBody3D>();
        if (!body)
        {
            vm->runtime_error("raycast: argument 4 must be a physics body");
            return -1;
        }
        parameters.exclude = body->getBody();
    }

    const PhysicsServer3D* server = PhysicsServer3D::singleton();
    PhysicsRayResult hit;
    if (!server || !server->intersectRay(parameters, hit))
    {
        args[0] = zen::val_nil();
        return 1;
    }

    ZenScripting::Impl& scripting = impl();
    zen::GC* gc = &vm->get_gc();
    zen::gc_pause(gc);
    zen::Value value = vm->make_instance(scripting.rayHitClass);
    zen::Value* fields = zen::as_instance(value)->fields;
    PhysicsBody3D* body = PhysicsBody3D::fromBody(hit.body);
    fields[kHitNode] = body ? scripting.wrap(body) : zen::val_nil();
    fields[kHitPosition] = makeVec3(hit.position);
    fields[kHitNormal] = makeVec3(hit.normal);
    fields[kHitDistance] = zen::val_float(hit.distance);
    MeshId mesh;
    const int surface = hitSurface(body, hit.triangle, mesh);
    fields[kHitTriangle] = zen::val_int(hit.triangle);
    fields[kHitSurface] = zen::val_int(surface);
    fields[kHitMaterial] = makeString(vm, surfaceMaterialName(mesh, surface));
    zen::gc_resume(gc);
    args[0] = value;
    return 1;
}

int physicsOverlapSphere(zen::VM* vm, zen::Value* args, int nargs)
{
    Math::Vec3 center;
    double radius = 0.0;
    std::uint32_t mask = 0;
    if (!needArgs(vm, nargs, 2, 3, "overlap_sphere") ||
            !argVec3(vm, args, 0, "overlap_sphere", center) ||
            !argNumber(vm, args, 1, "overlap_sphere", radius) ||
            !argMask(vm, args, nargs, 2, "overlap_sphere", mask))
        return -1;
    if (radius < 0.0)
    {
        vm->runtime_error("overlap_sphere: the radius must not be negative");
        return -1;
    }
    ct::Vector<Node*> nodes;
    if (const PhysicsServer3D* server = PhysicsServer3D::singleton())
    {
        ct::Vector<PhysicsBodyId> bodies;
        server->overlapSphere(center, static_cast<float>(radius), mask, bodies);
        collectBodyNodes(bodies, nodes);
    }
    return returnNodes(vm, args, nodes);
}

int physicsOverlapBox(zen::VM* vm, zen::Value* args, int nargs)
{
    Math::Vec3 center;
    Math::Vec3 extents;
    std::uint32_t mask = 0;
    if (!needArgs(vm, nargs, 2, 3, "overlap_box") || !argVec3(vm, args, 0, "overlap_box", center) ||
            !argVec3(vm, args, 1, "overlap_box", extents) ||
            !argMask(vm, args, nargs, 2, "overlap_box", mask))
        return -1;
    if (extents.x < 0.0f || extents.y < 0.0f || extents.z < 0.0f)
    {
        vm->runtime_error("overlap_box: the half extents must not be negative");
        return -1;
    }
    ct::Vector<Node*> nodes;
    if (const PhysicsServer3D* server = PhysicsServer3D::singleton())
    {
        ct::Vector<PhysicsBodyId> bodies;
        server->overlapBox(Math::Box::FromCenterExtents(center, extents), mask, bodies);
        collectBodyNodes(bodies, nodes);
    }
    return returnNodes(vm, args, nodes);
}

int globalMoveToward(zen::VM* vm, zen::Value* args, int nargs)
{
    double from = 0.0;
    double to = 0.0;
    double step = 0.0;
    if (!needArgs(vm, nargs, 3, 3, "move_toward") || !argNumber(vm, args, 0, "move_toward", from) ||
            !argNumber(vm, args, 1, "move_toward", to) ||
            !argNumber(vm, args, 2, "move_toward", step))
        return -1;
    const double difference = to - from;
    const double magnitude = difference < 0.0 ? -difference : difference;
    const double sign = difference > 0.0 ? 1.0 : (difference < 0.0 ? -1.0 : 0.0);
    args[0] = zen::val_float(magnitude <= step ? to : from + sign * step);
    return 1;
}

bool argQuat(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Quaternion& out)
{
    const zen::Value value = args[index];
    if (!zen::is_native_struct(value) || zen::as_native_struct(value)->def != impl().quatDef)
    {
        vm->runtime_error("%s: argument %d must be a Quat", name, index + 1);
        return false;
    }
    const float* data = static_cast<const float*>(zen::as_native_struct(value)->data);
    out = Math::Quaternion(data[0], data[1], data[2], data[3]);
    return true;
}

zen::Value makeQuat(zen::VM* vm, const Math::Quaternion& q)
{
    zen::Value parts[4] = { zen::val_float(q.x), zen::val_float(q.y), zen::val_float(q.z),
        zen::val_float(q.w) };
    return vm->make_native_struct(impl().quatDef, parts, 4);
}

int globalQuatBlend(zen::VM* vm, zen::Value* args, int nargs, const char* name, bool spherical)
{
    Math::Quaternion from;
    Math::Quaternion to;
    double t = 0.0;
    if (!needArgs(vm, nargs, 3, 3, name) || !argQuat(vm, args, 0, name, from) ||
            !argQuat(vm, args, 1, name, to) || !argNumber(vm, args, 2, name, t))
        return -1;
    const float weight = static_cast<float>(t);
    args[0] = makeQuat(vm, spherical ? Math::Quaternion::Slerp(from, to, weight)
                                     : Math::Quaternion::Lerp(from, to, weight));
    return 1;
}

int globalQuatSlerp(zen::VM* vm, zen::Value* args, int nargs)
{
    return globalQuatBlend(vm, args, nargs, "quat_slerp", true);
}

int globalQuatLerp(zen::VM* vm, zen::Value* args, int nargs)
{
    return globalQuatBlend(vm, args, nargs, "quat_lerp", false);
}

int globalQuatNormalized(zen::VM* vm, zen::Value* args, int nargs)
{
    Math::Quaternion value;
    if (!needArgs(vm, nargs, 1, 1, "quat_normalized") ||
            !argQuat(vm, args, 0, "quat_normalized", value))
        return -1;
    args[0] = makeQuat(vm, value.NormalizedSafe());
    return 1;
}

int globalQuatLength(zen::VM* vm, zen::Value* args, int nargs)
{
    Math::Quaternion value;
    if (!needArgs(vm, nargs, 1, 1, "quat_length") || !argQuat(vm, args, 0, "quat_length", value))
        return -1;
    args[0] = zen::val_float(value.Length());
    return 1;
}

int globalQuatDot(zen::VM* vm, zen::Value* args, int nargs)
{
    Math::Quaternion a;
    Math::Quaternion b;
    if (!needArgs(vm, nargs, 2, 2, "quat_dot") || !argQuat(vm, args, 0, "quat_dot", a) ||
            !argQuat(vm, args, 1, "quat_dot", b))
        return -1;
    args[0] = zen::val_float(a.Dot(b));
    return 1;
}

int globalIsValid(zen::VM*, zen::Value* args, int nargs)
{
    args[0] = zen::val_bool(nargs >= 1 && zen::is_instance(args[0]) &&
                            zen::as_instance(args[0])->native_data != nullptr);
    return 1;
}

zen::ObjClass* sAabbClass = nullptr;
zen::ObjClass* sRayClass = nullptr;

template<unsigned... I>
struct IndexSeq
{
};

template<unsigned N, unsigned... I>
struct MakeIndexSeq : MakeIndexSeq<N - 1, N - 1, I...>
{
};

template<unsigned... I>
struct MakeIndexSeq<0, I...>
{
    using type = IndexSeq<I...>;
};

struct CStr
{
    ct::String text;

    operator const char*() const
    {
        return text.c_str();
    }
};

template<typename A>
struct ArgStorage
{
    using type = typename std::decay<A>::type;
};

template<>
struct ArgStorage<const char*>
{
    using type = CStr;
};

template<typename E>
struct EnumLimit;

template<>
struct EnumLimit<PhysicsRagdollPart>
{
    static constexpr int kLast = static_cast<int>(PhysicsRagdollPart::Count) - 1;
};

template<>
struct EnumLimit<RagdollBone>
{
    static constexpr int kLast = static_cast<int>(RagdollBone::Count) - 1;
};

template<>
struct EnumLimit<SkeletonModifier3D::BoneDirection>
{
    static constexpr int kLast = static_cast<int>(SkeletonModifier3D::BoneDirection::FromParent);
};

template<>
struct EnumLimit<SkeletonModifier3D::RotationAxis>
{
    static constexpr int kLast = static_cast<int>(SkeletonModifier3D::RotationAxis::Custom);
};

template<>
struct EnumLimit<SpringBoneSimulator3D::CenterFrom>
{
    static constexpr int kLast = static_cast<int>(SpringBoneSimulator3D::CenterFrom::Bone);
};

template<>
struct EnumLimit<prisma::Topology>
{
    static constexpr int kLast = static_cast<int>(prisma::Topology::Patches);
};

template<>
struct EnumLimit<MaterialAlphaAntialiasing>
{
    static constexpr int kLast = static_cast<int>(MaterialAlphaAntialiasing::AlphaToCoverage);
};

template<typename I>
constexpr double integerLowest()
{
    return std::is_signed<I>::value ? -static_cast<double>(1ull << (sizeof(I) * 8 - 1)) : 0.0;
}

template<typename I>
constexpr double integerHighest()
{
    return std::is_signed<I>::value ? static_cast<double>((1ull << (sizeof(I) * 8 - 1)) - 1ull)
                                    : static_cast<double>(~0ull >> (64 - sizeof(I) * 8));
}

bool argVec2(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Vec2& out)
{
    if (!zen::is_vec2(args[index]))
    {
        vm->runtime_error("%s: argument %d must be a Vec2", name, index + 1);
        return false;
    }
    out = Math::Vec2(zen::vec_x(args[index]), zen::vec_y(args[index]));
    return true;
}

bool argVec4(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Vec4& out)
{
    const zen::Value value = args[index];
    if (zen::is_vec3(value))
    {
        out = Math::Vec4(zen::vec_x(value), zen::vec_y(value), zen::vec_z(value), 1.0f);
        return true;
    }
    if (zen::is_native_struct(value))
    {
        const zen::NativeStructDef* def = zen::as_native_struct(value)->def;
        if (def == impl().vec4Def || def == impl().colorDef)
        {
            const float* data = static_cast<const float*>(zen::as_native_struct(value)->data);
            out = Math::Vec4(data[0], data[1], data[2], data[3]);
            return true;
        }
    }
    vm->runtime_error("%s: argument %d must be a Color or Vec4", name, index + 1);
    return false;
}

template<typename E>
bool readListOf(zen::VM* vm, zen::Value* args, int index, const char* name, ct::Vector<E>& out)
{
    constexpr bool isVec2 = std::is_same<E, Math::Vec2>::value;
    if (!zen::is_array(args[index]))
    {
        vm->runtime_error("%s: argument %d must be a list of %s", name, index + 1,
                isVec2 ? "Vec2" : "Vec3");
        return false;
    }
    zen::ObjArray* array = zen::as_array(args[index]);
    out.clear();
    for (int i = 0; i < arr_count(array); ++i)
    {
        const zen::Value item = zen::array_get(array, i);
        if constexpr (isVec2)
        {
            if (!zen::is_vec2(item))
            {
                vm->runtime_error("%s: argument %d must be a list of Vec2", name, index + 1);
                return false;
            }
            out.push_back(Math::Vec2(zen::vec_x(item), zen::vec_y(item)));
        }
        else
        {
            if (!zen::is_vec3(item))
            {
                vm->runtime_error("%s: argument %d must be a list of Vec3", name, index + 1);
                return false;
            }
            out.push_back(Math::Vec3(zen::vec_x(item), zen::vec_y(item), zen::vec_z(item)));
        }
    }
    return true;
}

bool readValue(zen::VM*, zen::Value* args, int index, const char*, bool& out)
{
    out = zen::is_truthy_full(args[index]);
    return true;
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, float& out)
{
    double value = 0.0;
    if (!argNumber(vm, args, index, name, value)) return false;
    out = static_cast<float>(value);
    return true;
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, double& out)
{
    return argNumber(vm, args, index, name, out);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Vec2& out)
{
    return argVec2(vm, args, index, name, out);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Vec3& out)
{
    return argVec3(vm, args, index, name, out);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Vec4& out)
{
    return argVec4(vm, args, index, name, out);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, Math::Quaternion& out)
{
    return argQuat(vm, args, index, name, out);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, ct::String& out)
{
    return argString(vm, args, index, name, out);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, CStr& out)
{
    return argString(vm, args, index, name, out.text);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name,
        ct::Vector<Math::Vec2>& out)
{
    return readListOf(vm, args, index, name, out);
}

bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name,
        ct::Vector<Math::Vec3>& out)
{
    return readListOf(vm, args, index, name, out);
}

template<typename I,
        typename std::enable_if<std::is_integral<I>::value && !std::is_same<I, bool>::value,
                int>::type = 0>
bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, I& out)
{
    double value = 0.0;
    if (!argNumber(vm, args, index, name, value)) return false;
    if (!(value >= integerLowest<I>() && value <= integerHighest<I>()))
    {
        vm->runtime_error("%s: argument %d is out of range", name, index + 1);
        return false;
    }
    out = static_cast<I>(value);
    return true;
}

template<typename E, typename std::enable_if<std::is_enum<E>::value, int>::type = 0>
bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, E& out)
{
    double value = 0.0;
    if (!argNumber(vm, args, index, name, value)) return false;
    if (!(value >= 0.0 && value <= static_cast<double>(EnumLimit<E>::kLast)) ||
            value != static_cast<double>(static_cast<int>(value)))
    {
        vm->runtime_error("%s: argument %d is not a valid value", name, index + 1);
        return false;
    }
    out = static_cast<E>(static_cast<int>(value));
    return true;
}

template<typename P, typename std::enable_if<std::is_base_of<Node, P>::value, int>::type = 0>
bool readValue(zen::VM* vm, zen::Value* args, int index, const char* name, P*& out)
{
    using N = typename std::remove_cv<P>::type;
    if (zen::is_nil(args[index]))
    {
        out = nullptr;
        return true;
    }
    Node* node = nullptr;
    if (!argNode(vm, args, index, name, node)) return false;
    N* typed = node->template castTo<N>();
    if (!typed)
    {
        vm->runtime_error("%s: argument %d must be a %s", name, index + 1, N::classNameStatic());
        return false;
    }
    out = typed;
    return true;
}

template<typename T>
bool readOptional(zen::VM* vm, zen::Value* args, int nargs, int index, const char* name, T& out)
{
    if (index >= nargs) return true;
    return readValue(vm, args, index, name, out);
}

template<typename T>
void assignFallback(T& out, double value)
{
    if constexpr (std::is_arithmetic<T>::value) out = static_cast<T>(value);
    else if constexpr (std::is_enum<T>::value)
        out = static_cast<T>(static_cast<long long>(value));
}

zen::Value makeFloat4Value(zen::VM* vm, zen::NativeStructDef* def, float x, float y, float z,
        float w)
{
    zen::Value parts[4] = { zen::val_float(x), zen::val_float(y), zen::val_float(z),
        zen::val_float(w) };
    return vm->make_native_struct(def, parts, 4);
}

zen::Value makeColor(zen::VM* vm, const Math::Vec4& color)
{
    return makeFloat4Value(vm, impl().colorDef, color.x, color.y, color.z, color.w);
}

zen::Value makeVec3Pair(zen::VM* vm, zen::ObjClass* klass, const Math::Vec3& first,
        const Math::Vec3& second)
{
    zen::GC* gc = &vm->get_gc();
    zen::gc_pause(gc);
    zen::Value value = vm->make_instance(klass);
    zen::Value* fields = zen::as_instance(value)->fields;
    fields[0] = makeVec3(first);
    fields[1] = makeVec3(second);
    zen::gc_resume(gc);
    return value;
}

zen::Value toValue(zen::VM*, bool value)
{
    return zen::val_bool(value);
}

zen::Value toValue(zen::VM*, float value)
{
    return zen::val_float(value);
}

zen::Value toValue(zen::VM*, double value)
{
    return zen::val_float(value);
}

template<typename I,
        typename std::enable_if<std::is_integral<I>::value && !std::is_same<I, bool>::value,
                int>::type = 0>
zen::Value toValue(zen::VM*, I value)
{
    return zen::val_int(static_cast<long long>(value));
}

template<typename E, typename std::enable_if<std::is_enum<E>::value, int>::type = 0>
zen::Value toValue(zen::VM*, E value)
{
    return zen::val_int(static_cast<long long>(value));
}

zen::Value toValue(zen::VM*, const Math::Vec2& value)
{
    return makeVec2(value);
}

zen::Value toValue(zen::VM*, const Math::Vec3& value)
{
    return makeVec3(value);
}

zen::Value toValue(zen::VM* vm, const Math::Vec4& value)
{
    return makeFloat4Value(vm, impl().vec4Def, value.x, value.y, value.z, value.w);
}

zen::Value toValue(zen::VM* vm, const Math::Quaternion& value)
{
    return makeQuat(vm, value);
}

zen::Value toValue(zen::VM* vm, const ct::String& value)
{
    return makeString(vm, value);
}

zen::Value toValue(zen::VM* vm, const Math::Box& box)
{
    return makeVec3Pair(vm, sAabbClass, box.min, box.max);
}

zen::Value toValue(zen::VM* vm, const Math::Ray& ray)
{
    return makeVec3Pair(vm, sRayClass, ray.origin, ray.direction);
}

template<typename N, typename std::enable_if<std::is_base_of<Node, N>::value, int>::type = 0>
zen::Value toValue(zen::VM*, N* node)
{
    return node ? impl().wrap(node) : zen::val_nil();
}

template<typename E>
zen::Value toValue(zen::VM* vm, const ct::Vector<E>& list)
{
    zen::GC* gc = &vm->get_gc();
    zen::gc_pause(gc);
    zen::ObjArray* array = zen::new_array(gc);
    for (std::size_t i = 0; i < list.size(); ++i) zen::array_push(gc, array, toValue(vm, list[i]));
    zen::gc_resume(gc);
    return zen::val_obj(reinterpret_cast<zen::Obj*>(array));
}

template<typename F>
struct MemberTraits;

template<typename... A>
struct ArgPack;

template<>
struct ArgPack<>
{
    bool read(zen::VM*, zen::Value*, int, int, const char*, const double*)
    {
        return true;
    }
};

template<typename H, typename... R>
struct ArgPack<H, R...>
{
    typename ArgStorage<H>::type head{};
    ArgPack<R...> tail;

    bool read(zen::VM* vm, zen::Value* args, int nargs, int index, const char* name,
            const double* fallbacks)
    {
        if (index < nargs)
        {
            if (!readValue(vm, args, index, name, head)) return false;
        }
        else
            assignFallback(head, fallbacks[index]);
        return tail.read(vm, args, nargs, index + 1, name, fallbacks);
    }

    template<unsigned I>
    auto& get()
    {
        if constexpr (I == 0) return head;
        else
            return tail.template get<I - 1>();
    }
};

template<typename R, typename C, typename... A>
struct MemberTraits<R (C::*)(A...)>
{
    using Result = R;
    using Class = C;
    using Pack = ArgPack<A...>;
    static constexpr unsigned kCount = sizeof...(A);
};

template<typename R, typename C, typename... A>
struct MemberTraits<R (C::*)(A...) const>
{
    using Result = R;
    using Class = C;
    using Pack = ArgPack<A...>;
    static constexpr unsigned kCount = sizeof...(A);
};

template<typename Resolver, typename Self>
struct SelfOf
{
    static Self* get(zen::VM* vm, zen::Value* args)
    {
        return Resolver::get(vm, args);
    }
};

template<typename Self>
struct SelfOf<void, Self>
{
    static Self* get(zen::VM* vm, zen::Value* args)
    {
        return selfNodeAs<Self>(vm, args);
    }
};

template<auto Fn, int Min, typename Resolver>
struct MemberCall
{
    using Traits = MemberTraits<decltype(Fn)>;
    using Self = typename Traits::Class;
    static constexpr int kMax = static_cast<int>(Traits::kCount);
    static constexpr int kMin = Min < 0 ? kMax : Min;

    static inline const char* name = "";
    static inline double fallbacks[kMax + 1] = {};

    static int invoke(zen::VM* vm, zen::Value* args, int nargs)
    {
        Self* self = SelfOf<Resolver, Self>::get(vm, args);
        if (!self || !needArgs(vm, nargs, kMin, kMax, name)) return -1;
        typename Traits::Pack pack;
        if (!pack.read(vm, args, nargs, 0, name, fallbacks)) return -1;
        return call(vm, args, self, pack, typename MakeIndexSeq<Traits::kCount>::type());
    }

    template<unsigned... I>
    static int call(zen::VM* vm, zen::Value* args, Self* self, typename Traits::Pack& pack,
            IndexSeq<I...>)
    {
        if constexpr (std::is_void<typename Traits::Result>::value)
        {
            (self->*Fn)(pack.template get<I>()...);
            return 0;
        }
        else
        {
            args[0] = toValue(vm, (self->*Fn)(pack.template get<I>()...));
            return 1;
        }
    }
};

template<auto Fn, int Min = -1, typename Resolver = void, typename... D>
void bindMethod(zen::VM::ClassBuilder& builder, const char* name, D... fallbacks)
{
    using Call = MemberCall<Fn, Min, Resolver>;
    static_assert(static_cast<int>(sizeof...(D)) <= Call::kMax - Call::kMin,
            "more fallbacks than optional arguments");
    Call::name = name;
    const double values[] = { static_cast<double>(fallbacks)..., 0.0 };
    for (int i = 0; i < static_cast<int>(sizeof...(D)); ++i)
        Call::fallbacks[Call::kMin + i] = values[i];
    builder.method(name, &Call::invoke, Call::kMin == Call::kMax ? Call::kMax : -1);
}

struct PathCurve
{
    static Curve3D* get(zen::VM* vm, zen::Value* args)
    {
        Path3D* path = selfNodeAs<Path3D>(vm, args);
        return path ? &path->getCurve() : nullptr;
    }
};

Terrain3D* loadedTerrainNode(zen::VM* vm, zen::Value* args)
{
    Terrain3D* terrain = selfNodeAs<Terrain3D>(vm, args);
    if (terrain && !terrain->isLoaded())
    {
        vm->runtime_error("the terrain is not loaded");
        return nullptr;
    }
    return terrain;
}

Terrain& syncedTerrain(Terrain3D* node)
{
    Terrain& terrain = node->getTerrain();
    terrain.setPosition(node->getGlobalPosition());
    return terrain;
}

struct LoadedTerrain
{
    static Terrain3D* get(zen::VM* vm, zen::Value* args)
    {
        return loadedTerrainNode(vm, args);
    }
};

struct TerrainData
{
    static Terrain* get(zen::VM* vm, zen::Value* args)
    {
        Terrain3D* node = selfNodeAs<Terrain3D>(vm, args);
        return node ? &node->getTerrain() : nullptr;
    }
};

struct LoadedTerrainData
{
    static Terrain* get(zen::VM* vm, zen::Value* args)
    {
        Terrain3D* node = loadedTerrainNode(vm, args);
        return node ? &syncedTerrain(node) : nullptr;
    }
};

struct ProjectingCamera
{
    static Camera3D* get(zen::VM* vm, zen::Value* args)
    {
        Camera3D* camera = selfNodeAs<Camera3D>(vm, args);
        if (!camera) return nullptr;
        const Viewport* viewport = camera->getViewport();
        if (camera->isInsideTree() && viewport && viewport->getWidth() > 0 &&
                viewport->getHeight() > 0)
            return camera;
        vm->runtime_error("the camera is not inside a scene with a viewport");
        return nullptr;
    }
};

ct::String resolveFilePath(const ct::String& path)
{
    Project* project = Main::project();
    if (!project || !project->isOpen() || path.empty()) return path;
    if (path[0] == '/' || path[0] == '\\' || (path.size() > 1 && path[1] == ':')) return path;
    return project->absolutePath(path);
}

int rigidGetCollidingBodies(zen::VM* vm, zen::Value* args, int)
{
    RigidBody3D* self = selfNodeAs<RigidBody3D>(vm, args);
    if (!self) return -1;
    ct::Vector<PhysicsBody3D*> bodies;
    self->getCollidingBodies(bodies);
    ct::Vector<Node*> nodes;
    for (std::size_t i = 0; i < bodies.size(); ++i) nodes.push_back(bodies[i]);
    return returnNodes(vm, args, nodes);
}

int audioListenerGetCurrent(zen::VM* vm, zen::Value* args, int)
{
    if (!selfNodeAs<AudioListener3D>(vm, args)) return -1;
    return returnNode(vm, args, AudioListener3D::getCurrent());
}

int animationGetList(zen::VM* vm, zen::Value* args, int)
{
    AnimationPlayer* self = selfNodeAs<AnimationPlayer>(vm, args);
    if (!self) return -1;
    zen::GC* gc = &vm->get_gc();
    zen::ObjArray* array = zen::new_array(gc);
    args[0] = zen::val_obj(reinterpret_cast<zen::Obj*>(array));
    for (std::size_t i = 0; i < self->getAnimationCount(); ++i)
        zen::array_push(gc, array, makeString(vm, self->getAnimationName(i)));
    return 1;
}

int particleEmitParticle(zen::VM* vm, zen::Value* args, int nargs)
{
    ParticleSystem3D* self = selfNodeAs<ParticleSystem3D>(vm, args);
    Math::Vec3 position;
    Math::Vec3 velocity;
    if (!self || !needArgs(vm, nargs, 1, 2, "emit_particle") ||
            !readValue(vm, args, 0, "emit_particle", position))
        return -1;
    if (nargs == 2)
    {
        if (!readValue(vm, args, 1, "emit_particle", velocity)) return -1;
        args[0] = zen::val_bool(self->emitParticle(position, velocity));
    }
    else
        args[0] = zen::val_bool(self->emitParticle(position));
    return 1;
}

int particleCollisionCollide(zen::VM* vm, zen::Value* args, int nargs)
{
    ParticleCollision3D* self = selfNodeAs<ParticleCollision3D>(vm, args);
    Math::Vec3 position;
    float size = 0.0f;
    if (!self || !needArgs(vm, nargs, 2, 2, "collide") ||
            !readValue(vm, args, 0, "collide", position) ||
            !readValue(vm, args, 1, "collide", size))
        return -1;
    Math::Vec3 normal;
    float depth = 0.0f;
    if (!self->collide(position, size, normal, depth))
    {
        args[0] = zen::val_nil();
        return 1;
    }
    zen::GC* gc = &vm->get_gc();
    zen::ObjArray* array = zen::new_array(gc);
    args[0] = zen::val_obj(reinterpret_cast<zen::Obj*>(array));
    zen::array_push(gc, array, makeVec3(normal));
    zen::array_push(gc, array, zen::val_float(depth));
    return 1;
}

int ribbonAddChain(zen::VM* vm, zen::Value* args, int nargs)
{
    RibbonTrail3D* self = selfNodeAs<RibbonTrail3D>(vm, args);
    ct::String emitter;
    Math::Vec4 startColor(1.0f, 1.0f, 1.0f, 1.0f);
    Math::Vec4 endColor(1.0f, 1.0f, 1.0f, 0.0f);
    float startWidth = 0.7f;
    float endWidth = 0.05f;
    float fadeTime = 0.3f;
    if (!self || !needArgs(vm, nargs, 1, 6, "add_chain") ||
            !readValue(vm, args, 0, "add_chain", emitter) ||
            !readOptional(vm, args, nargs, 1, "add_chain", startColor) ||
            !readOptional(vm, args, nargs, 2, "add_chain", endColor) ||
            !readOptional(vm, args, nargs, 3, "add_chain", startWidth) ||
            !readOptional(vm, args, nargs, 4, "add_chain", endWidth) ||
            !readOptional(vm, args, nargs, 5, "add_chain", fadeTime))
        return -1;
    args[0] = zen::val_int(
            self->addChain(emitter, startColor, endColor, startWidth, endWidth, fadeTime));
    return 1;
}

int ribbonAddBladeChain(zen::VM* vm, zen::Value* args, int nargs)
{
    RibbonTrail3D* self = selfNodeAs<RibbonTrail3D>(vm, args);
    ct::String base;
    ct::String tip;
    Math::Vec4 startColor(1.0f, 1.0f, 1.0f, 1.0f);
    Math::Vec4 endColor(1.0f, 1.0f, 1.0f, 0.0f);
    float fadeTime = 0.3f;
    float startSpan = 1.0f;
    float endSpan = 0.0f;
    if (!self || !needArgs(vm, nargs, 2, 7, "add_blade_chain") ||
            !readValue(vm, args, 0, "add_blade_chain", base) ||
            !readValue(vm, args, 1, "add_blade_chain", tip) ||
            !readOptional(vm, args, nargs, 2, "add_blade_chain", startColor) ||
            !readOptional(vm, args, nargs, 3, "add_blade_chain", endColor) ||
            !readOptional(vm, args, nargs, 4, "add_blade_chain", fadeTime) ||
            !readOptional(vm, args, nargs, 5, "add_blade_chain", startSpan) ||
            !readOptional(vm, args, nargs, 6, "add_blade_chain", endSpan))
        return -1;
    args[0] = zen::val_int(
            self->addBladeChain(base, tip, startColor, endColor, fadeTime, startSpan, endSpan));
    return 1;
}

int collisionShapeSetConvexFromPoints(zen::VM* vm, zen::Value* args, int nargs)
{
    CollisionShape3D* self = selfNodeAs<CollisionShape3D>(vm, args);
    ct::Vector<Math::Vec3> points;
    if (!self || !needArgs(vm, nargs, 1, 1, "set_convex_from_points") ||
            !readValue(vm, args, 0, "set_convex_from_points", points))
        return -1;
    args[0] = zen::val_bool(self->setConvexFromPoints(points.data(), points.size()));
    return 1;
}

int collisionShapeSetHeightMapFromTerrain(zen::VM* vm, zen::Value* args, int nargs)
{
    CollisionShape3D* self = selfNodeAs<CollisionShape3D>(vm, args);
    Node* node = nullptr;
    if (!self || !needArgs(vm, nargs, 1, 1, "set_height_map_from_terrain") ||
            !argNode(vm, args, 0, "set_height_map_from_terrain", node))
        return -1;
    Terrain3D* terrain = node->castTo<Terrain3D>();
    if (!terrain)
    {
        vm->runtime_error("set_height_map_from_terrain: argument 1 must be a Terrain3D");
        return -1;
    }
    args[0] = zen::val_bool(self->setHeightMapFromTerrain(*terrain));
    return 1;
}

int collisionShapeGetHullPoints(zen::VM* vm, zen::Value* args, int)
{
    CollisionShape3D* self = selfNodeAs<CollisionShape3D>(vm, args);
    if (!self) return -1;
    ct::Vector<Math::Vec3> points;
    self->getHullPoints(points);
    args[0] = toValue(vm, points);
    return 1;
}

int terrainGetNormalAt(zen::VM* vm, zen::Value* args, int nargs)
{
    Terrain3D* self = loadedTerrainNode(vm, args);
    Math::Vec3 position;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_normal_at") ||
            !readValue(vm, args, 0, "get_normal_at", position))
        return -1;
    const Terrain& terrain = syncedTerrain(self);
    Math::Vec3 local;
    terrain.getTerrainPosition(position, local);
    const float last = static_cast<float>(terrain.getSize() - 1);
    auto cell = [last](float value)
    {
        const float scaled = value * last + 0.5f;
        return scaled < 0.0f ? 0
                             : (scaled > last ? static_cast<int>(last) : static_cast<int>(scaled));
    };
    args[0] = makeVec3(terrain.calculateNormal(cell(local.x), cell(local.y)));
    return 1;
}

int terrainRayIntersects(zen::VM* vm, zen::Value* args, int nargs)
{
    Terrain3D* self = loadedTerrainNode(vm, args);
    Math::Vec3 origin;
    Math::Vec3 direction;
    if (!self || !needArgs(vm, nargs, 2, 2, "ray_intersects") ||
            !readValue(vm, args, 0, "ray_intersects", origin) ||
            !readValue(vm, args, 1, "ray_intersects", direction))
        return -1;
    Math::Vec3 hit;
    if (!self->rayIntersects(Math::Ray(origin, direction.NormalizedSafe()), hit))
    {
        args[0] = zen::val_nil();
        return 1;
    }
    args[0] = makeVec3(hit);
    return 1;
}

int terrainBrushHeightCall(zen::VM* vm, zen::Value* args, int nargs)
{
    Terrain3D* self = loadedTerrainNode(vm, args);
    Math::Vec3 centre;
    float radius = 0.0f;
    float strength = 0.0f;
    float elapsed = 0.0f;
    bool lower = false;
    if (!self || !needArgs(vm, nargs, 4, 5, "brush_height") ||
            !readValue(vm, args, 0, "brush_height", centre) ||
            !readValue(vm, args, 1, "brush_height", radius) ||
            !readValue(vm, args, 2, "brush_height", strength) ||
            !readValue(vm, args, 3, "brush_height", elapsed) ||
            !readOptional(vm, args, nargs, 4, "brush_height", lower))
        return -1;
    args[0] = zen::val_bool(
            terrainBrushHeight(syncedTerrain(self), centre, radius, strength, elapsed, lower));
    return 1;
}

int terrainBrushBlendCall(zen::VM* vm, zen::Value* args, int nargs)
{
    Terrain3D* self = loadedTerrainNode(vm, args);
    std::uint8_t layer = 0;
    Math::Vec3 centre;
    float radius = 0.0f;
    float strength = 0.0f;
    float elapsed = 0.0f;
    bool remove = false;
    if (!self || !needArgs(vm, nargs, 5, 6, "brush_blend") ||
            !readValue(vm, args, 0, "brush_blend", layer) ||
            !readValue(vm, args, 1, "brush_blend", centre) ||
            !readValue(vm, args, 2, "brush_blend", radius) ||
            !readValue(vm, args, 3, "brush_blend", strength) ||
            !readValue(vm, args, 4, "brush_blend", elapsed) ||
            !readOptional(vm, args, nargs, 5, "brush_blend", remove))
        return -1;
    args[0] = zen::val_bool(terrainBrushBlend(syncedTerrain(self), layer, centre, radius, strength,
            elapsed, remove));
    return 1;
}

int terrainBrushSmoothCall(zen::VM* vm, zen::Value* args, int nargs)
{
    Terrain3D* self = loadedTerrainNode(vm, args);
    Math::Vec3 centre;
    float radius = 0.0f;
    float strength = 0.0f;
    float elapsed = 0.0f;
    if (!self || !needArgs(vm, nargs, 4, 4, "brush_smooth") ||
            !readValue(vm, args, 0, "brush_smooth", centre) ||
            !readValue(vm, args, 1, "brush_smooth", radius) ||
            !readValue(vm, args, 2, "brush_smooth", strength) ||
            !readValue(vm, args, 3, "brush_smooth", elapsed))
        return -1;
    args[0] = zen::val_bool(
            terrainBrushSmooth(syncedTerrain(self), centre, radius, strength, elapsed));
    return 1;
}

int terrainBrushFlattenCall(zen::VM* vm, zen::Value* args, int nargs)
{
    Terrain3D* self = loadedTerrainNode(vm, args);
    Math::Vec3 centre;
    float radius = 0.0f;
    float height = 0.0f;
    float strength = 0.0f;
    float elapsed = 0.0f;
    if (!self || !needArgs(vm, nargs, 5, 5, "brush_flatten") ||
            !readValue(vm, args, 0, "brush_flatten", centre) ||
            !readValue(vm, args, 1, "brush_flatten", radius) ||
            !readValue(vm, args, 2, "brush_flatten", height) ||
            !readValue(vm, args, 3, "brush_flatten", strength) ||
            !readValue(vm, args, 4, "brush_flatten", elapsed))
        return -1;
    args[0] = zen::val_bool(
            terrainBrushFlatten(syncedTerrain(self), centre, radius, height, strength, elapsed));
    return 1;
}

int forestInstanceCount(zen::VM* vm, zen::Value* args, int nargs, const char* name, bool impostors)
{
    Forest3D* self = selfNodeAs<Forest3D>(vm, args);
    int species = -1;
    if (!self || !needArgs(vm, nargs, 0, 1, name) ||
            !readOptional(vm, args, nargs, 0, name, species))
        return -1;
    int count = 0;
    if (nargs == 0)
        count = impostors ? self->getImpostorInstanceCount() : self->getMeshInstanceCount();
    else
        count = impostors ? self->getImpostorInstanceCount(species)
                          : self->getMeshInstanceCount(species);
    args[0] = zen::val_int(count);
    return 1;
}

int forestGetMeshInstanceCount(zen::VM* vm, zen::Value* args, int nargs)
{
    return forestInstanceCount(vm, args, nargs, "get_mesh_instance_count", false);
}

int forestGetImpostorInstanceCount(zen::VM* vm, zen::Value* args, int nargs)
{
    return forestInstanceCount(vm, args, nargs, "get_impostor_instance_count", true);
}

int grassErase(zen::VM* vm, zen::Value* args, int nargs)
{
    Grass3D* self = selfNodeAs<Grass3D>(vm, args);
    Math::Vec3 position;
    float radius = 0.0f;
    float amount = 0.0f;
    int type = 0;
    if (!self || !needArgs(vm, nargs, 3, 4, "erase") ||
            !readValue(vm, args, 0, "erase", position) ||
            !readValue(vm, args, 1, "erase", radius) || !readValue(vm, args, 2, "erase", amount) ||
            !readOptional(vm, args, nargs, 3, "erase", type))
        return -1;
    args[0] = zen::val_bool(nargs == 4 ? self->erase(position, radius, amount, type)
                                       : self->erase(position, radius, amount));
    return 1;
}

int grassSaveMask(zen::VM* vm, zen::Value* args, int nargs)
{
    Grass3D* self = selfNodeAs<Grass3D>(vm, args);
    ct::String path;
    if (!self || !needArgs(vm, nargs, 1, 1, "save_mask") ||
            !readValue(vm, args, 0, "save_mask", path))
        return -1;
    args[0] = zen::val_bool(self->saveMask(resolveFilePath(path).c_str()));
    return 1;
}

int grassLoadMask(zen::VM* vm, zen::Value* args, int nargs)
{
    Grass3D* self = selfNodeAs<Grass3D>(vm, args);
    ct::String path;
    if (!self || !needArgs(vm, nargs, 1, 1, "load_mask") ||
            !readValue(vm, args, 0, "load_mask", path))
        return -1;
    args[0] = zen::val_bool(self->loadMask(resolveScenePath(path).c_str()));
    return 1;
}

int manualMeshSurfaceBegin(zen::VM* vm, zen::Value* args, int nargs)
{
    ManualMesh3D* self = selfNodeAs<ManualMesh3D>(vm, args);
    prisma::Topology topology = prisma::Topology::Triangles;
    if (!self || !needArgs(vm, nargs, 1, 1, "surface_begin") ||
            !readValue(vm, args, 0, "surface_begin", topology))
        return -1;
    args[0] = zen::val_bool(self->surfaceBegin(topology));
    return 1;
}

int multiMeshGetInstanceColor(zen::VM* vm, zen::Value* args, int nargs)
{
    MultiMeshInstance3D* self = selfNodeAs<MultiMeshInstance3D>(vm, args);
    int instance = 0;
    if (!self || !needArgs(vm, nargs, 1, 1, "get_instance_color") ||
            !readValue(vm, args, 0, "get_instance_color", instance))
        return -1;
    args[0] = makeColor(vm, self->getInstanceColor(instance));
    return 1;
}

int softBodyPinInBox(zen::VM* vm, zen::Value* args, int nargs)
{
    SoftBody3D* self = selfNodeAs<SoftBody3D>(vm, args);
    Math::Vec3 minimum;
    Math::Vec3 maximum;
    bool pinned = true;
    if (!self || !needArgs(vm, nargs, 2, 3, "pin_in_box") ||
            !readValue(vm, args, 0, "pin_in_box", minimum) ||
            !readValue(vm, args, 1, "pin_in_box", maximum) ||
            !readOptional(vm, args, nargs, 2, "pin_in_box", pinned))
        return -1;
    self->pinInBox(Math::Box(minimum, maximum), pinned);
    return 0;
}

template<typename R>
int cameraProjectPoint(zen::VM* vm, zen::Value* args, int nargs, const char* name,
        R (Camera3D::*project)(const Math::Vec2&) const)
{
    Camera3D* self = selfNodeAs<Camera3D>(vm, args);
    Math::Vec2 point;
    if (!self || !argPoint2(vm, args, nargs, name, point) || !cameraCanProject(vm, self, name))
        return -1;
    args[0] = toValue(vm, (self->*project)(point));
    return 1;
}

int cameraProjectLocalRayNormal(zen::VM* vm, zen::Value* args, int nargs)
{
    return cameraProjectPoint(vm, args, nargs, "project_local_ray_normal",
            &Camera3D::projectLocalRayNormal);
}

int cameraProjectRay(zen::VM* vm, zen::Value* args, int nargs)
{
    return cameraProjectPoint(vm, args, nargs, "project_ray", &Camera3D::projectRay);
}

int cameraProjectPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    Camera3D* self = selfNodeAs<Camera3D>(vm, args);
    if (!self) return -1;
    Math::Vec2 point;
    double depth = 0.0;
    if (nargs == 2 && zen::is_vec2(args[0]))
    {
        if (!argVec2(vm, args, 0, "project_position", point) ||
                !argNumber(vm, args, 1, "project_position", depth))
            return -1;
    }
    else if (nargs == 3)
    {
        double x = 0.0;
        double y = 0.0;
        if (!argNumber(vm, args, 0, "project_position", x) ||
                !argNumber(vm, args, 1, "project_position", y) ||
                !argNumber(vm, args, 2, "project_position", depth))
            return -1;
        point = Math::Vec2(static_cast<float>(x), static_cast<float>(y));
    }
    else
    {
        vm->runtime_error("project_position: expected a Vec2 and a depth or 2 numbers and a "
                          "depth, got %d argument(s)",
                nargs);
        return -1;
    }
    if (!cameraCanProject(vm, self, "project_position")) return -1;
    args[0] = makeVec3(self->projectPosition(point, static_cast<float>(depth)));
    return 1;
}

int cameraPickDepth(zen::VM* vm, zen::Value* args, int nargs)
{
    Camera3D* self = selfNodeAs<Camera3D>(vm, args);
    Math::Vec2 point;
    if (!self || !argPoint2(vm, args, nargs, "pick_depth", point) ||
            !cameraCanProject(vm, self, "pick_depth"))
        return -1;
    Math::Vec3 position;
    args[0] = self->pickDepth(point, position) ? makeVec3(position) : zen::val_nil();
    return 1;
}

using TerrainAddLayerByPath = bool (Terrain3D::*)(float, const ct::String&, const ct::String&);

void bindNodeMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Node::moveChild>(builder, "move_child");
    bindMethod<&Node::getPathTo>(builder, "get_path_to");
    bindMethod<&Node::isAncestorOf>(builder, "is_ancestor_of");
    bindMethod<&Node::getOwner>(builder, "get_owner");
    bindMethod<&Node::isInstance>(builder, "is_instance");
    bindMethod<&Node::getSceneFilePath>(builder, "get_scene_file_path");
    bindMethod<&Node::isEnabled>(builder, "is_enabled");
    bindMethod<&Node::getProcessDeltaTime>(builder, "get_process_delta_time");
    bindMethod<&Node::getPhysicsProcessDeltaTime>(builder, "get_physics_process_delta_time");
}

void bindNode3DMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Node3D::getParentNode3D>(builder, "get_parent_node_3d");
    bindMethod<&Node3D::globalRotate>(builder, "global_rotate");
    bindMethod<&Node3D::globalTranslate>(builder, "global_translate");
    bindMethod<&Node3D::globalScale>(builder, "global_scale");
    bindMethod<&Node3D::scaleObjectLocal>(builder, "scale_object_local");
    bindMethod<&Node3D::orthonormalize>(builder, "orthonormalize");
    bindMethod<&Node3D::setIdentity>(builder, "set_identity");
    bindMethod<&Node3D::forceUpdateTransform>(builder, "force_update_transform");
    bindMethod<&Node3D::setCollisionCapsule>(builder, "set_collision_capsule");
    bindMethod<&Node3D::collidedWith>(builder, "collided_with");
    bindMethod<&Node3D::resetCollision>(builder, "reset_collision");
}

void bindPhysicsBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&PhysicsBody3D::getMode>(builder, "get_mode");
    bindMethod<&PhysicsBody3D::isSimulated>(builder, "is_simulated");
    bindMethod<&PhysicsBody3D::setContactMonitor, 1>(builder, "set_contact_monitor", 8);
}

void bindRigidBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&RigidBody3D::getComputedInertia>(builder, "get_computed_inertia");
    bindMethod<&RigidBody3D::getContactCount>(builder, "get_contact_count");
    builder.method("get_colliding_bodies", &rigidGetCollidingBodies, 0);
}

void bindCharacterBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&CharacterBody3D::setVelocity>(builder, "set_velocity");
    bindMethod<&CharacterBody3D::getVelocity>(builder, "get_velocity");
    bindMethod<&CharacterBody3D::moveAndSlide, 0>(builder, "move_and_slide", 0.0);
    bindMethod<&CharacterBody3D::isOnFloor>(builder, "is_on_floor");
    bindMethod<&CharacterBody3D::isOnWall>(builder, "is_on_wall");
    bindMethod<&CharacterBody3D::isOnCeiling>(builder, "is_on_ceiling");
    bindMethod<&CharacterBody3D::getFloorNormal>(builder, "get_floor_normal");
    bindMethod<&CharacterBody3D::getWallNormal>(builder, "get_wall_normal");
    bindMethod<&CharacterBody3D::getCeilingNormal>(builder, "get_ceiling_normal");
    bindMethod<&CharacterBody3D::getFloorAngle>(builder, "get_floor_angle");
    bindMethod<&CharacterBody3D::isNearGround>(builder, "is_near_ground");
    bindMethod<&CharacterBody3D::getSlideCollisionCount>(builder, "get_slide_collision_count");
    bindMethod<&CharacterBody3D::getSlideCollider>(builder, "get_slide_collider");
    bindMethod<&CharacterBody3D::getSlideNormal>(builder, "get_slide_normal");
}

void bindCharacterRigidBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&CharacterRigidBody3D::probeGround>(builder, "probe_ground");
    bindMethod<&CharacterRigidBody3D::getGroundState>(builder, "get_ground_state");
    bindMethod<&CharacterRigidBody3D::isOnGround>(builder, "is_on_ground");
    bindMethod<&CharacterRigidBody3D::isSupported>(builder, "is_supported");
    bindMethod<&CharacterRigidBody3D::getGroundNormal>(builder, "get_ground_normal");
    bindMethod<&CharacterRigidBody3D::getGroundPosition>(builder, "get_ground_position");
    bindMethod<&CharacterRigidBody3D::getGroundVelocity>(builder, "get_ground_velocity");
    bindMethod<&CharacterRigidBody3D::getGroundBody>(builder, "get_ground_body");
}

void bindCollisionShapeMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&CollisionShape3D::setBox>(builder, "set_box");
    bindMethod<&CollisionShape3D::setSphere>(builder, "set_sphere");
    bindMethod<&CollisionShape3D::setCapsule>(builder, "set_capsule");
    bindMethod<&CollisionShape3D::setCylinder>(builder, "set_cylinder");
    bindMethod<&CollisionShape3D::setPlane>(builder, "set_plane");
    builder.method("set_convex_from_points", &collisionShapeSetConvexFromPoints, 1);
    builder.method("set_height_map_from_terrain", &collisionShapeSetHeightMapFromTerrain, 1);
    builder.method("get_hull_points", &collisionShapeGetHullPoints, 0);
}

void bindParticleSystemMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&ParticleSystem3D::play>(builder, "play");
    bindMethod<&ParticleSystem3D::stop>(builder, "stop");
    bindMethod<&ParticleSystem3D::pause>(builder, "pause");
    bindMethod<&ParticleSystem3D::simulate>(builder, "simulate");
    bindMethod<&ParticleSystem3D::emitBurst>(builder, "emit_burst");
    builder.method("emit_particle", &particleEmitParticle, -1);
    bindMethod<&ParticleSystem3D::getState>(builder, "get_state");
    bindMethod<&ParticleSystem3D::getActiveCount>(builder, "get_active_count");

    bindMethod<&ParticleSystem3D::setContinuous>(builder, "set_continuous");
    bindMethod<&ParticleSystem3D::setBurst, 1>(builder, "set_burst", 1.0);
    bindMethod<&ParticleSystem3D::setOneShot>(builder, "set_one_shot");
    bindMethod<&ParticleSystem3D::setPulse>(builder, "set_pulse");

    bindMethod<&ParticleSystem3D::setShapePoint>(builder, "set_shape_point");
    bindMethod<&ParticleSystem3D::setShapeSphere>(builder, "set_shape_sphere");
    bindMethod<&ParticleSystem3D::setShapeBox>(builder, "set_shape_box");
    bindMethod<&ParticleSystem3D::setShapeCone, 1>(builder, "set_shape_cone", 0.0);
    bindMethod<&ParticleSystem3D::setShapeCircle>(builder, "set_shape_circle");
    bindMethod<&ParticleSystem3D::setShapeRing>(builder, "set_shape_ring");

    bindMethod<&ParticleSystem3D::addGravity>(builder, "add_gravity");
    bindMethod<&ParticleSystem3D::addDrag>(builder, "add_drag");
    bindMethod<&ParticleSystem3D::addVortex>(builder, "add_vortex");
    bindMethod<&ParticleSystem3D::addAttractor, 3>(builder, "add_attractor", 0);
    bindMethod<&ParticleSystem3D::addTurbulence>(builder, "add_turbulence");
    bindMethod<&ParticleSystem3D::addColorOverLifetime>(builder, "add_color_over_lifetime");
    bindMethod<&ParticleSystem3D::addSizeOverLifetime>(builder, "add_size_over_lifetime");
    bindMethod<&ParticleSystem3D::clearAffectors>(builder, "clear_affectors");
    bindMethod<&ParticleSystem3D::getAffectorCount>(builder, "get_affector_count");
}

void bindRibbonTrailMethods(zen::VM::ClassBuilder& builder)
{
    builder.method("add_chain", &ribbonAddChain, -1);
    builder.method("add_blade_chain", &ribbonAddBladeChain, -1);
    bindMethod<&RibbonTrail3D::clearChains>(builder, "clear_chains");
    bindMethod<&RibbonTrail3D::getChainCount>(builder, "get_chain_count");
    bindMethod<&RibbonTrail3D::getElementCount>(builder, "get_element_count");
}

void bindTerrainMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Terrain3D::isLoaded>(builder, "is_loaded");
    bindMethod<&Terrain3D::unload>(builder, "unload");
    bindMethod<&Terrain3D::update>(builder, "update");
    bindMethod<&Terrain3D::updateFromCamera>(builder, "update_from_camera");
    bindMethod<&Terrain3D::flushCompositeMap>(builder, "flush_composite_map");

    bindMethod<&Terrain3D::getWorldHeightAt, -1, LoadedTerrain>(builder, "get_world_height_at");
    bindMethod<&Terrain3D::getWorldSlopeAt, -1, LoadedTerrain>(builder, "get_world_slope_at");
    bindMethod<&Terrain3D::getAabb, -1, LoadedTerrain>(builder, "get_aabb");
    builder.method("get_normal_at", &terrainGetNormalAt, 1);
    builder.method("ray_intersects", &terrainRayIntersects, 2);

    bindMethod<&Terrain::getSize, -1, LoadedTerrainData>(builder, "get_size");
    bindMethod<&Terrain::getWorldSize, -1, LoadedTerrainData>(builder, "get_world_size");
    bindMethod<&Terrain::getMinHeight, -1, LoadedTerrainData>(builder, "get_min_height");
    bindMethod<&Terrain::getMaxHeight, -1, LoadedTerrainData>(builder, "get_max_height");

    bindMethod<static_cast<TerrainAddLayerByPath>(&Terrain3D::addLayer)>(builder, "add_layer");
    bindMethod<&Terrain3D::removeLayer>(builder, "remove_layer");
    bindMethod<&Terrain3D::setLayerWorldSize>(builder, "set_layer_world_size");
    bindMethod<&Terrain::getLayerCount, -1, TerrainData>(builder, "get_layer_count");
    bindMethod<&Terrain::getLayerWorldSize, -1, TerrainData>(builder, "get_layer_world_size");

    builder.method("brush_height", &terrainBrushHeightCall, -1);
    builder.method("brush_blend", &terrainBrushBlendCall, -1);
    builder.method("brush_smooth", &terrainBrushSmoothCall, 4);
    builder.method("brush_flatten", &terrainBrushFlattenCall, 5);

    bindMethod<&Terrain3D::setGlobalColourMapEnabled, 1>(builder, "set_global_colour_map_enabled",
            0);
    bindMethod<&Terrain3D::setLightMapDirection>(builder, "set_light_map_direction");
    bindMethod<&Terrain3D::setCompositeMapLighting>(builder, "set_composite_map_lighting");

    bindMethod<&Terrain3D::getNodeCount>(builder, "get_node_count");
    bindMethod<&Terrain3D::getNodesDrawn>(builder, "get_nodes_drawn");
    bindMethod<&Terrain3D::getNodesDrawnAtLod>(builder, "get_nodes_drawn_at_lod");
    bindMethod<&Terrain3D::getNodesDrawnComposite>(builder, "get_nodes_drawn_composite");
    bindMethod<&Terrain3D::getLodLevelCount>(builder, "get_lod_level_count");
    bindMethod<&Terrain3D::getTrianglesDrawn>(builder, "get_triangles_drawn");
    bindMethod<&Terrain3D::getVertexMeshCount>(builder, "get_vertex_mesh_count");
    bindMethod<&Terrain3D::getIndexPoolCount>(builder, "get_index_pool_count");
    bindMethod<&Terrain3D::getNodeMeshCount>(builder, "get_node_mesh_count");
    bindMethod<&Terrain3D::getVertexUploadBytes>(builder, "get_vertex_upload_bytes");
    bindMethod<&Terrain3D::getVertexUploadRanges>(builder, "get_vertex_upload_ranges");
    bindMethod<&Terrain3D::getLastUpdateOwners>(builder, "get_last_update_owners");
    bindMethod<&Terrain3D::resetEditStats>(builder, "reset_edit_stats");
    bindMethod<&Terrain3D::getUploadedBytes>(builder, "get_uploaded_bytes");
    bindMethod<&Terrain3D::getUploadCount>(builder, "get_upload_count");
    bindMethod<&Terrain3D::resetUploadStats>(builder, "reset_upload_stats");
    bindMethod<&Terrain3D::getTextureMemoryBytes>(builder, "get_texture_memory_bytes");
    bindMethod<&Terrain3D::getCompositeBakes>(builder, "get_composite_bakes");
    bindMethod<&Terrain3D::getCompositeBakeTexels>(builder, "get_composite_bake_texels");
}

void bindGrassMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Grass3D::removeType>(builder, "remove_type");
    bindMethod<&Grass3D::getTypeCount>(builder, "get_type_count");
    bindMethod<&Grass3D::addTypeRegion, 5>(builder, "add_type_region", 1.0, 1.0);
    bindMethod<&Grass3D::clearTypeRegions>(builder, "clear_type_regions");
    bindMethod<&Grass3D::getTypeRegionCount>(builder, "get_type_region_count");
    bindMethod<&Grass3D::addRegion, 4>(builder, "add_region", 1.0, 1.0);
    bindMethod<&Grass3D::clearRegions>(builder, "clear_regions");
    bindMethod<&Grass3D::getRegionCount>(builder, "get_region_count");

    bindMethod<&Grass3D::setHeight>(builder, "set_height");
    bindMethod<&Grass3D::getHeight>(builder, "get_height");
    bindMethod<&Grass3D::setWidth>(builder, "set_width");
    bindMethod<&Grass3D::getWidth>(builder, "get_width");
    bindMethod<&Grass3D::setAlphaCut>(builder, "set_alpha_cut");
    bindMethod<&Grass3D::getAlphaCut>(builder, "get_alpha_cut");
    bindMethod<&Grass3D::setAlphaAntialiasing, 1>(builder, "set_alpha_antialiasing", 0.3);
    bindMethod<&Grass3D::setExtent>(builder, "set_extent");
    bindMethod<&Grass3D::clearExtent>(builder, "clear_extent");

    bindMethod<&Grass3D::setTerrain>(builder, "set_terrain");
    bindMethod<&Grass3D::getTerrain>(builder, "get_terrain");
    bindMethod<&Grass3D::clearExclusions>(builder, "clear_exclusions");
    bindMethod<&Grass3D::getReseatedCount>(builder, "get_reseated_count");
    bindMethod<&Grass3D::getReseatMilliseconds>(builder, "get_reseat_milliseconds");

    bindMethod<&Grass3D::clearInfluencers>(builder, "clear_influencers");
    bindMethod<&Grass3D::addInfluencer>(builder, "add_influencer");
    bindMethod<&Grass3D::getInfluencerCount>(builder, "get_influencer_count");

    bindMethod<&Grass3D::createMask, 0>(builder, "create_mask", 0, 0);
    bindMethod<&Grass3D::hasMask>(builder, "has_mask");
    bindMethod<&Grass3D::getMaskWidth>(builder, "get_mask_width");
    bindMethod<&Grass3D::getMaskDepth>(builder, "get_mask_depth");
    bindMethod<&Grass3D::clearMask>(builder, "clear_mask");
    bindMethod<&Grass3D::paint>(builder, "paint");
    builder.method("erase", &grassErase, -1);
    bindMethod<&Grass3D::fill>(builder, "fill");
    bindMethod<&Grass3D::getMaskDensity>(builder, "get_mask_density");
    builder.method("save_mask", &grassSaveMask, 1);
    builder.method("load_mask", &grassLoadMask, 1);

    bindMethod<&Grass3D::invalidate>(builder, "invalidate");
    bindMethod<&Grass3D::getActiveCellCount, 0>(builder, "get_active_cell_count", -1);
    bindMethod<&Grass3D::getVisibleCellCount, 0>(builder, "get_visible_cell_count", -1);
    bindMethod<&Grass3D::getPooledCellCount, 0>(builder, "get_pooled_cell_count", -1);
    bindMethod<&Grass3D::getInstanceCount, 0>(builder, "get_instance_count", -1);
    bindMethod<&Grass3D::getDrawnInstanceCount, 0>(builder, "get_drawn_instance_count", -1);
    bindMethod<&Grass3D::getWrittenInstanceCount>(builder, "get_written_instance_count");
    bindMethod<&Grass3D::getAwakeInstanceCount>(builder, "get_awake_instance_count");
    bindMethod<&Grass3D::getSimulatedInstanceCount>(builder, "get_simulated_instance_count");
    bindMethod<&Grass3D::getFilledCellCount>(builder, "get_filled_cell_count");
    bindMethod<&Grass3D::getUpdateMilliseconds>(builder, "get_update_milliseconds");
    bindMethod<&Grass3D::hasCell>(builder, "has_cell");
    bindMethod<&Grass3D::getCellClumpCount>(builder, "get_cell_clump_count");
    bindMethod<&Grass3D::getCellNode>(builder, "get_cell_node");
}

void bindForestMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Forest3D::removeSpecies>(builder, "remove_species");
    bindMethod<&Forest3D::getSpeciesCount>(builder, "get_species_count");
    bindMethod<&Forest3D::getMeshNode>(builder, "get_mesh_node");
    bindMethod<&Forest3D::plant>(builder, "plant");
    bindMethod<&Forest3D::plantOnTerrain>(builder, "plant_on_terrain");
    bindMethod<&Forest3D::clearInstances>(builder, "clear_instances");
    bindMethod<&Forest3D::setTerrain>(builder, "set_terrain");
    bindMethod<&Forest3D::getTerrain>(builder, "get_terrain");
    bindMethod<&Forest3D::clearExclusions>(builder, "clear_exclusions");
    bindMethod<&Forest3D::getReseatedCount>(builder, "get_reseated_count");
    bindMethod<&Forest3D::getReseatMilliseconds>(builder, "get_reseat_milliseconds");
    bindMethod<&Forest3D::getInstanceCount>(builder, "get_instance_count");
    bindMethod<&Forest3D::getInstanceSpecies>(builder, "get_instance_species");
    bindMethod<&Forest3D::isInstanceOnTerrain>(builder, "is_instance_on_terrain");
    bindMethod<&Forest3D::getInstanceCentre>(builder, "get_instance_centre");
    bindMethod<&Forest3D::getInstanceRadius>(builder, "get_instance_radius");
    builder.method("get_mesh_instance_count", &forestGetMeshInstanceCount, -1);
    builder.method("get_impostor_instance_count", &forestGetImpostorInstanceCount, -1);
    bindMethod<&Forest3D::getWrittenInstanceCount>(builder, "get_written_instance_count");
}

void bindTreeMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Tree3D::rebuild>(builder, "rebuild");
    bindMethod<&Tree3D::isDirty>(builder, "is_dirty");
    bindMethod<&Tree3D::getBuildCount>(builder, "get_build_count");
}

void bindRoadMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Road3D::setProfile>(builder, "set_profile");
    bindMethod<&Road3D::clearProfile>(builder, "clear_profile");
    bindMethod<&Road3D::getProfile>(builder, "get_profile");
    bindMethod<&Road3D::setPointWidth>(builder, "set_point_width");
    bindMethod<&Road3D::getPointWidth>(builder, "get_point_width");
    bindMethod<&Road3D::clearPointWidths>(builder, "clear_point_widths");
    bindMethod<&Road3D::clearGround>(builder, "clear_ground");
    bindMethod<&Road3D::hasGround>(builder, "has_ground");
    bindMethod<&Road3D::setTerrain>(builder, "set_terrain");
    bindMethod<&Road3D::getTerrain>(builder, "get_terrain");
    bindMethod<&Road3D::flush>(builder, "flush");
    bindMethod<&Road3D::rebuild>(builder, "rebuild");
    bindMethod<&Road3D::isDirty>(builder, "is_dirty");
    bindMethod<&Road3D::getBuildCount>(builder, "get_build_count");
}

void bindWaterMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Water3D::setWind>(builder, "set_wind");
    bindMethod<&Water3D::getPlanarReflector>(builder, "get_planar_reflector");
}

void bindOceanMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Ocean3D::isSupported>(builder, "is_supported");
    bindMethod<&Ocean3D::getWaterHeight>(builder, "get_water_height");
    bindMethod<&Ocean3D::getDisplacedPosition>(builder, "get_displaced_position");
    bindMethod<&Ocean3D::update>(builder, "update");
}

void bindSkeletonMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Skeleton3D::clearBones>(builder, "clear_bones");
    bindMethod<&Skeleton3D::getBoneCount>(builder, "get_bone_count");
    bindMethod<&Skeleton3D::findBone>(builder, "find_bone");
    bindMethod<&Skeleton3D::getBoneName>(builder, "get_bone_name");
    bindMethod<&Skeleton3D::getBoneParent>(builder, "get_bone_parent");
    bindMethod<&Skeleton3D::setBonePosePosition>(builder, "set_bone_pose_position");
    bindMethod<&Skeleton3D::setBonePoseRotation>(builder, "set_bone_pose_rotation");
    bindMethod<&Skeleton3D::setBonePoseScale>(builder, "set_bone_pose_scale");
    bindMethod<&Skeleton3D::getBonePosePosition>(builder, "get_bone_pose_position");
    bindMethod<&Skeleton3D::getBonePoseRotation>(builder, "get_bone_pose_rotation");
    bindMethod<&Skeleton3D::getBonePoseScale>(builder, "get_bone_pose_scale");
    bindMethod<&Skeleton3D::resetBonePose>(builder, "reset_bone_pose");
    bindMethod<&Skeleton3D::resetBonePoses>(builder, "reset_bone_poses");
    bindMethod<&Skeleton3D::forceUpdateAllBoneTransforms>(builder,
            "force_update_all_bone_transforms");
    bindMethod<&Skeleton3D::updateSkeleton>(builder, "update_skeleton");
    bindMethod<&Skeleton3D::isDirty>(builder, "is_dirty");
    bindMethod<&Skeleton3D::getPoseVersion>(builder, "get_pose_version");
    bindMethod<&Skeleton3D::advance>(builder, "advance");
}

void bindSkeletonModifierMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&SkeletonModifier3D::getSkeleton>(builder, "get_skeleton");
}

void bindSpringBoneSimulatorMethods(zen::VM::ClassBuilder& builder)
{
    using S = SpringBoneSimulator3D;
    bindMethod<&S::addSetting>(builder, "add_setting");
    bindMethod<&S::clearSettings>(builder, "clear_settings");
    bindMethod<&S::reset>(builder, "reset");
    bindMethod<&S::getBoneVector>(builder, "get_bone_vector");

    bindMethod<&S::setRootBoneName>(builder, "set_setting_root_bone_name");
    bindMethod<&S::getRootBoneName>(builder, "get_setting_root_bone_name");
    bindMethod<&S::setRootBone>(builder, "set_setting_root_bone");
    bindMethod<&S::getRootBone>(builder, "get_setting_root_bone");
    bindMethod<&S::setEndBoneName>(builder, "set_setting_end_bone_name");
    bindMethod<&S::getEndBoneName>(builder, "get_setting_end_bone_name");
    bindMethod<&S::setEndBone>(builder, "set_setting_end_bone");
    bindMethod<&S::getEndBone>(builder, "get_setting_end_bone");
    bindMethod<&S::setExtendEndBone>(builder, "set_setting_extend_end_bone");
    bindMethod<&S::isEndBoneExtended>(builder, "get_setting_extend_end_bone");
    bindMethod<&S::setEndBoneDirection>(builder, "set_setting_end_bone_direction");
    bindMethod<&S::getEndBoneDirection>(builder, "get_setting_end_bone_direction");
    bindMethod<&S::setEndBoneLength>(builder, "set_setting_end_bone_length");
    bindMethod<&S::getEndBoneLength>(builder, "get_setting_end_bone_length");
    bindMethod<&S::setCenterFrom>(builder, "set_setting_center_from");
    bindMethod<&S::getCenterFrom>(builder, "get_setting_center_from");
    bindMethod<&S::setCenterNode>(builder, "set_setting_center_node");
    bindMethod<&S::getCenterNode>(builder, "get_setting_center_node");
    bindMethod<&S::setCenterBoneName>(builder, "set_setting_center_bone_name");
    bindMethod<&S::getCenterBoneName>(builder, "get_setting_center_bone_name");
    bindMethod<&S::setCenterBone>(builder, "set_setting_center_bone");
    bindMethod<&S::getCenterBone>(builder, "get_setting_center_bone");
    bindMethod<&S::setRotationAxis>(builder, "set_setting_rotation_axis");
    bindMethod<&S::getRotationAxis>(builder, "get_setting_rotation_axis");
    bindMethod<&S::setRotationAxisVector>(builder, "set_setting_rotation_axis_vector");
    bindMethod<&S::getRotationAxisVector>(builder, "get_setting_rotation_axis_vector");
    bindMethod<&S::setRadius>(builder, "set_setting_radius");
    bindMethod<&S::getRadius>(builder, "get_setting_radius");
    bindMethod<&S::setStiffness>(builder, "set_setting_stiffness");
    bindMethod<&S::getStiffness>(builder, "get_setting_stiffness");
    bindMethod<&S::setDrag>(builder, "set_setting_drag");
    bindMethod<&S::getDrag>(builder, "get_setting_drag");
    bindMethod<&S::setGravity>(builder, "set_setting_gravity");
    bindMethod<&S::getGravity>(builder, "get_setting_gravity");
    bindMethod<&S::setGravityDirection>(builder, "set_setting_gravity_direction");
    bindMethod<&S::getGravityDirection>(builder, "get_setting_gravity_direction");
    bindMethod<&S::setIndividualConfig>(builder, "set_setting_individual_config");
    bindMethod<&S::isConfigIndividual>(builder, "get_setting_individual_config");

    bindMethod<&S::getJointCount>(builder, "get_joint_count");
    bindMethod<&S::getJointBoneName>(builder, "get_joint_bone_name");
    bindMethod<&S::getJointBone>(builder, "get_joint_bone");
    bindMethod<&S::setJointRotationAxis>(builder, "set_joint_rotation_axis");
    bindMethod<&S::getJointRotationAxis>(builder, "get_joint_rotation_axis");
    bindMethod<&S::setJointRotationAxisVector>(builder, "set_joint_rotation_axis_vector");
    bindMethod<&S::getJointRotationAxisVector>(builder, "get_joint_rotation_axis_vector");
    bindMethod<&S::setJointRadius>(builder, "set_joint_radius");
    bindMethod<&S::getJointRadius>(builder, "get_joint_radius");
    bindMethod<&S::setJointStiffness>(builder, "set_joint_stiffness");
    bindMethod<&S::getJointStiffness>(builder, "get_joint_stiffness");
    bindMethod<&S::setJointDrag>(builder, "set_joint_drag");
    bindMethod<&S::getJointDrag>(builder, "get_joint_drag");
    bindMethod<&S::setJointGravity>(builder, "set_joint_gravity");
    bindMethod<&S::getJointGravity>(builder, "get_joint_gravity");
    bindMethod<&S::setJointGravityDirection>(builder, "set_joint_gravity_direction");
    bindMethod<&S::getJointGravityDirection>(builder, "get_joint_gravity_direction");

    bindMethod<&S::setEnableAllChildCollisions>(builder, "set_enable_all_child_collisions");
    bindMethod<&S::areAllChildCollisionsEnabled>(builder, "are_all_child_collisions_enabled");
    bindMethod<&S::setExcludeCollisionPath>(builder, "set_exclude_collision_path");
    bindMethod<&S::getExcludeCollisionPath>(builder, "get_exclude_collision_path");
    bindMethod<&S::setExcludeCollisionCount>(builder, "set_exclude_collision_count");
    bindMethod<&S::getExcludeCollisionCount>(builder, "get_exclude_collision_count");
    bindMethod<&S::clearExcludeCollisions>(builder, "clear_exclude_collisions");
    bindMethod<&S::setCollisionPath>(builder, "set_collision_path");
    bindMethod<&S::getCollisionPath>(builder, "get_collision_path");
    bindMethod<&S::setCollisionCount>(builder, "set_collision_count");
    bindMethod<&S::getCollisionCount>(builder, "get_collision_count");
    bindMethod<&S::clearCollisions>(builder, "clear_collisions");
    bindMethod<&S::getValidCollisions>(builder, "get_valid_collisions");
}

void bindSpringBoneCollisionMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&SpringBoneCollision3D::getSkeleton>(builder, "get_skeleton");
    bindMethod<&SpringBoneCollision3D::syncPose>(builder, "sync_pose");
}

void bindSpringBoneCollisionCapsuleMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&SpringBoneCollisionCapsule3D::setMidHeight>(builder, "set_mid_height");
    bindMethod<&SpringBoneCollisionCapsule3D::getMidHeight>(builder, "get_mid_height");
}

void bindBoneAttachmentMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&BoneAttachment3D::getSkeleton>(builder, "get_skeleton");
}

void bindRagdollMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Ragdoll3D::getSkeleton>(builder, "get_skeleton");
    bindMethod<&Ragdoll3D::getAnimationPlayer>(builder, "get_animation_player");
    bindMethod<&Ragdoll3D::setBone>(builder, "set_bone");
    bindMethod<&Ragdoll3D::getBone>(builder, "get_bone");
    bindMethod<&Ragdoll3D::setBoneName>(builder, "set_bone_name");
    bindMethod<&Ragdoll3D::autoMapBones>(builder, "auto_map_bones");
    bindMethod<&Ragdoll3D::build>(builder, "build");
    bindMethod<&Ragdoll3D::isBuilt>(builder, "is_built");
    bindMethod<&Ragdoll3D::start>(builder, "start");
    bindMethod<&Ragdoll3D::stop>(builder, "stop");
    bindMethod<&Ragdoll3D::isActive>(builder, "is_active");
    bindMethod<&Ragdoll3D::getFacing>(builder, "get_facing");
    bindMethod<&Ragdoll3D::getPartBone>(builder, "get_part_bone");
    bindMethod<&Ragdoll3D::getPartMass>(builder, "get_part_mass");
    bindMethod<&Ragdoll3D::applyImpulse, 2>(builder, "apply_impulse");
    bindMethod<&Ragdoll3D::applyBlast>(builder, "apply_blast");
    bindMethod<&Ragdoll3D::getPartVelocity>(builder, "get_part_velocity");
}

void bindDismemberMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Dismember3D::getRagdoll>(builder, "get_ragdoll");
    bindMethod<&Dismember3D::detach>(builder, "detach");
    bindMethod<&Dismember3D::isDetached>(builder, "is_detached");
    bindMethod<&Dismember3D::getDetachedCount>(builder, "get_detached_count");
    bindMethod<&Dismember3D::explode>(builder, "explode");
}

int navigationFindPath(zen::VM* vm, zen::Value* args, int nargs)
{
    NavigationRegion3D* self = selfNodeAs<NavigationRegion3D>(vm, args);
    Math::Vec3 from;
    Math::Vec3 to;
    if (!self || !needArgs(vm, nargs, 2, 2, "find_path") ||
            !readValue(vm, args, 0, "find_path", from) || !readValue(vm, args, 1, "find_path", to))
        return -1;
    ct::Vector<Math::Vec3> path;
    self->findPath(from, to, path);
    args[0] = toValue(vm, path);
    return 1;
}

int navigationNearestPoint(zen::VM* vm, zen::Value* args, int nargs)
{
    NavigationRegion3D* self = selfNodeAs<NavigationRegion3D>(vm, args);
    Math::Vec3 point;
    if (!self || !needArgs(vm, nargs, 1, 1, "nearest_point") ||
            !readValue(vm, args, 0, "nearest_point", point))
        return -1;
    Math::Vec3 out;
    args[0] = self->nearestPoint(point, out) ? makeVec3(out) : zen::val_nil();
    return 1;
}

int navigationMoveAlongSurface(zen::VM* vm, zen::Value* args, int nargs)
{
    NavigationRegion3D* self = selfNodeAs<NavigationRegion3D>(vm, args);
    Math::Vec3 from;
    Math::Vec3 to;
    if (!self || !needArgs(vm, nargs, 2, 2, "move_along_surface") ||
            !readValue(vm, args, 0, "move_along_surface", from) ||
            !readValue(vm, args, 1, "move_along_surface", to))
        return -1;
    Math::Vec3 out;
    args[0] = self->moveAlongSurface(from, to, out) ? makeVec3(out) : zen::val_nil();
    return 1;
}

int navigationRaycast(zen::VM* vm, zen::Value* args, int nargs)
{
    NavigationRegion3D* self = selfNodeAs<NavigationRegion3D>(vm, args);
    Math::Vec3 from;
    Math::Vec3 to;
    if (!self || !needArgs(vm, nargs, 2, 2, "raycast") ||
            !readValue(vm, args, 0, "raycast", from) || !readValue(vm, args, 1, "raycast", to))
        return -1;
    float fraction = 1.0f;
    self->raycast(from, to, fraction);
    args[0] = zen::val_float(fraction);
    return 1;
}

int navigationRandomPoint(zen::VM* vm, zen::Value* args, int)
{
    NavigationRegion3D* self = selfNodeAs<NavigationRegion3D>(vm, args);
    if (!self) return -1;
    Math::Vec3 out;
    args[0] = self->randomPoint(out) ? makeVec3(out) : zen::val_nil();
    return 1;
}

void bindNavigationRegionMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&NavigationRegion3D::bake>(builder, "bake");
    bindMethod<&NavigationRegion3D::clear>(builder, "clear");
    bindMethod<&NavigationRegion3D::isBaked>(builder, "is_baked");
    bindMethod<&NavigationRegion3D::getTriangleCount>(builder, "get_triangle_count");
    builder.method("find_path", &navigationFindPath, 2);
    builder.method("nearest_point", &navigationNearestPoint, 1);
    builder.method("move_along_surface", &navigationMoveAlongSurface, 2);
    builder.method("raycast", &navigationRaycast, 2);
    builder.method("random_point", &navigationRandomPoint, 0);
}

void bindAnimationPlayerMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&AnimationPlayer::removeAnimation>(builder, "remove_animation");
    bindMethod<&AnimationPlayer::hasAnimation>(builder, "has_animation");
    bindMethod<&AnimationPlayer::getAnimationCount>(builder, "get_animation_count");
    bindMethod<&AnimationPlayer::getAnimationName>(builder, "get_animation_name");
    builder.method("get_animation_list", &animationGetList, 0);
    bindMethod<&AnimationPlayer::getSkeleton>(builder, "get_skeleton");
    bindMethod<&AnimationPlayer::setAnimationLoop>(builder, "set_animation_loop");
    bindMethod<&AnimationPlayer::getAnimationLoop>(builder, "get_animation_loop");
    bindMethod<&AnimationPlayer::animationSetNext>(builder, "animation_set_next");
    bindMethod<&AnimationPlayer::animationGetNext>(builder, "animation_get_next");
    bindMethod<&AnimationPlayer::setBlendTime>(builder, "set_blend_time");
    bindMethod<&AnimationPlayer::getBlendTime>(builder, "get_blend_time");
    bindMethod<&AnimationPlayer::playBackwards, 0>(builder, "play_backwards", 0, -1.0);
    bindMethod<&AnimationPlayer::queue>(builder, "queue");
    bindMethod<&AnimationPlayer::getQueueCount>(builder, "get_queue_count");
    bindMethod<&AnimationPlayer::getQueued>(builder, "get_queued");
    bindMethod<&AnimationPlayer::clearQueue>(builder, "clear_queue");
    bindMethod<&AnimationPlayer::pause>(builder, "pause");
    bindMethod<&AnimationPlayer::getCurrentAnimation>(builder, "get_current_animation");
    bindMethod<&AnimationPlayer::getAssignedAnimation>(builder, "get_assigned_animation");
    bindMethod<&AnimationPlayer::getPlayingSpeed>(builder, "get_playing_speed");
    bindMethod<&AnimationPlayer::seek, 1>(builder, "seek", 0);
    bindMethod<&AnimationPlayer::getCurrentAnimationPosition>(builder,
            "get_current_animation_position");
    bindMethod<&AnimationPlayer::getCurrentAnimationLength>(builder,
            "get_current_animation_length");
    bindMethod<&AnimationPlayer::advance>(builder, "advance");
}

template<typename T>
void bindAudioPlayerMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&T::pause>(builder, "pause");
    bindMethod<&T::resume>(builder, "resume");
}

void bindAudioListenerMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&AudioListener3D::makeCurrent>(builder, "make_current");
    bindMethod<&AudioListener3D::clearCurrent, 0>(builder, "clear_current", 1);
    builder.method("get_current", &audioListenerGetCurrent, 0);
}

void bindPathMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Curve3D::getPointCount, -1, PathCurve>(builder, "get_point_count");
    bindMethod<&Curve3D::setPointCount, -1, PathCurve>(builder, "set_point_count");
    bindMethod<&Curve3D::addPoint, 1, PathCurve>(builder, "add_point", 0, 0, -1);
    bindMethod<&Curve3D::setPointPosition, -1, PathCurve>(builder, "set_point_position");
    bindMethod<&Curve3D::getPointPosition, -1, PathCurve>(builder, "get_point_position");
    bindMethod<&Curve3D::setPointTilt, -1, PathCurve>(builder, "set_point_tilt");
    bindMethod<&Curve3D::getPointTilt, -1, PathCurve>(builder, "get_point_tilt");
    bindMethod<&Curve3D::setPointIn, -1, PathCurve>(builder, "set_point_in");
    bindMethod<&Curve3D::getPointIn, -1, PathCurve>(builder, "get_point_in");
    bindMethod<&Curve3D::setPointOut, -1, PathCurve>(builder, "set_point_out");
    bindMethod<&Curve3D::getPointOut, -1, PathCurve>(builder, "get_point_out");
    bindMethod<&Curve3D::removePoint, -1, PathCurve>(builder, "remove_point");
    bindMethod<&Curve3D::clearPoints, -1, PathCurve>(builder, "clear_points");
    bindMethod<&Curve3D::sample, -1, PathCurve>(builder, "sample");
    bindMethod<&Curve3D::samplef, -1, PathCurve>(builder, "samplef");
    bindMethod<&Curve3D::setClosed, -1, PathCurve>(builder, "set_closed");
    bindMethod<&Curve3D::isClosed, -1, PathCurve>(builder, "is_closed");
    bindMethod<&Curve3D::setBakeInterval, -1, PathCurve>(builder, "set_bake_interval");
    bindMethod<&Curve3D::getBakeInterval, -1, PathCurve>(builder, "get_bake_interval");
    bindMethod<&Curve3D::setUpVectorEnabled, -1, PathCurve>(builder, "set_up_vector_enabled");
    bindMethod<&Curve3D::isUpVectorEnabled, -1, PathCurve>(builder, "is_up_vector_enabled");
    bindMethod<&Curve3D::getBakedLength, -1, PathCurve>(builder, "get_baked_length");
    bindMethod<&Curve3D::sampleBaked, 1, PathCurve>(builder, "sample_baked", 0);
    bindMethod<&Curve3D::sampleBakedTilt, -1, PathCurve>(builder, "sample_baked_tilt");
    bindMethod<&Curve3D::sampleBakedUpVector, 1, PathCurve>(builder, "sample_baked_up_vector", 0);
    bindMethod<&Curve3D::getBakedPoints, -1, PathCurve>(builder, "get_baked_points");
    bindMethod<&Curve3D::getBakedTilts, -1, PathCurve>(builder, "get_baked_tilts");
    bindMethod<&Curve3D::getBakedUpVectors, -1, PathCurve>(builder, "get_baked_up_vectors");
    bindMethod<&Curve3D::getBakedDistCache, -1, PathCurve>(builder, "get_baked_dist_cache");
    bindMethod<&Curve3D::getClosestPoint, -1, PathCurve>(builder, "get_closest_point");
    bindMethod<&Curve3D::getClosestOffset, -1, PathCurve>(builder, "get_closest_offset");
    bindMethod<&Curve3D::getPoints, -1, PathCurve>(builder, "get_points");
    bindMethod<&Curve3D::tessellate, 0, PathCurve>(builder, "tessellate", 5, 4.0);
    bindMethod<&Curve3D::tessellateEvenLength, 0, PathCurve>(builder, "tessellate_even_length", 5,
            0.2);
}

void bindPathFollowMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&PathFollow3D::setProgressRatio>(builder, "set_progress_ratio");
    bindMethod<&PathFollow3D::getProgressRatio>(builder, "get_progress_ratio");
    bindMethod<&PathFollow3D::updateTransform>(builder, "update_transform");
}

void bindVehicleWheelMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&VehicleWheel3D::isInContact>(builder, "is_in_contact");
    bindMethod<&VehicleWheel3D::getContactBody>(builder, "get_contact_body");
    bindMethod<&VehicleWheel3D::getContactPoint>(builder, "get_contact_point");
    bindMethod<&VehicleWheel3D::getContactNormal>(builder, "get_contact_normal");
    bindMethod<&VehicleWheel3D::getSkidInfo>(builder, "get_skid_info");
    bindMethod<&VehicleWheel3D::getRpm>(builder, "get_rpm");
}

void bindVehicleBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&VehicleBody3D::getSpeedKmh>(builder, "get_speed_kmh");
    bindMethod<&VehicleBody3D::getWheelCount>(builder, "get_wheel_count");
    bindMethod<&VehicleBody3D::getWheel>(builder, "get_wheel");
    bindMethod<&VehicleBody3D::getLeanAngle>(builder, "get_lean_angle");
}

void bindGroundVehicleMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&GroundVehicle3D::getCollisionHalfExtents>(builder, "get_collision_half_extents");
    bindMethod<&GroundVehicle3D::getRestHeight>(builder, "get_rest_height");
    bindMethod<&GroundVehicle3D::getSpeedKmh>(builder, "get_speed_kmh");
    bindMethod<&GroundVehicle3D::getForwardSpeed>(builder, "get_forward_speed");
    bindMethod<&GroundVehicle3D::getRpm>(builder, "get_rpm");
    bindMethod<&GroundVehicle3D::getGear>(builder, "get_gear");
    bindMethod<&GroundVehicle3D::getWheelCount>(builder, "get_wheel_count");
    bindMethod<&GroundVehicle3D::isWheelInContact>(builder, "is_wheel_in_contact");
    bindMethod<&GroundVehicle3D::getWheelSlipRatio>(builder, "get_wheel_slip_ratio");
    bindMethod<&GroundVehicle3D::getWheelSlide>(builder, "get_wheel_slide");
    bindMethod<&GroundVehicle3D::getWheelSlipAngle>(builder, "get_wheel_slip_angle");
    bindMethod<&GroundVehicle3D::getWheelRadius>(builder, "get_wheel_radius");
    bindMethod<&GroundVehicle3D::isWheelDetached>(builder, "is_wheel_detached");
    bindMethod<&GroundVehicle3D::setWheelDetached>(builder, "set_wheel_detached");
    bindMethod<&GroundVehicle3D::setWheelRadiusScale>(builder, "set_wheel_radius_scale");
    bindMethod<&GroundVehicle3D::stopWheels>(builder, "stop_wheels");
    bindMethod<&GroundVehicle3D::park>(builder, "park");
    bindMethod<&GroundVehicle3D::setAxleGrip>(builder, "set_axle_grip");
    bindMethod<&GroundVehicle3D::isReversing>(builder, "is_reversing");
}

void bindCarDamageMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&CarDamage3D::isBuilt>(builder, "is_built");
    bindMethod<&CarDamage3D::getPeak>(builder, "get_peak");
    bindMethod<&CarDamage3D::getNoseCrush>(builder, "get_nose_crush");
    bindMethod<&CarDamage3D::getTailCrush>(builder, "get_tail_crush");
    bindMethod<&CarDamage3D::getStepImpulse>(builder, "get_step_impulse");
    bindMethod<&CarDamage3D::getTotalImpulse>(builder, "get_total_impulse");
    bindMethod<&CarDamage3D::damageNear>(builder, "damage_near");
    bindMethod<&CarDamage3D::hit>(builder, "hit");
    bindMethod<&CarDamage3D::reset>(builder, "reset");
}

void bindSkidMarksMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&SkidMarks3D::clear>(builder, "clear");
}

void bindImpactFxMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&ImpactFx3D::impact>(builder, "impact");
    bindMethod<&ImpactFx3D::smoke>(builder, "smoke");
    bindMethod<&ImpactFx3D::flame>(builder, "flame");
    bindMethod<&ImpactFx3D::blast>(builder, "blast");
    bindMethod<&ImpactFx3D::crash>(builder, "crash");
}

void bindTrafficMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Traffic3D::getActiveCount>(builder, "get_active_count");
}

void bindVehicleDriverMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&VehicleDriver3D::getProgress>(builder, "get_progress");
    bindMethod<&VehicleDriver3D::wantsBoost>(builder, "wants_boost");
}

void bindJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Joint3D::setBodies>(builder, "set_bodies");
    bindMethod<&Joint3D::getBodyA>(builder, "get_body_a");
    bindMethod<&Joint3D::getBodyB>(builder, "get_body_b");
    bindMethod<&Joint3D::isConfigured>(builder, "is_configured");
    bindMethod<&Joint3D::getConfigurationWarning>(builder, "get_configuration_warning");
}

void bindPinJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&PinJoint3D::setMotor>(builder, "set_motor");
    bindMethod<&PinJoint3D::disableMotor>(builder, "disable_motor");
}

void bindHingeJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&HingeJoint3D::setMotor>(builder, "set_motor");
    bindMethod<&HingeJoint3D::disableMotor>(builder, "disable_motor");
    bindMethod<&HingeJoint3D::getAngle>(builder, "get_angle");
}

void bindSliderJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&SliderJoint3D::setMotor>(builder, "set_motor");
    bindMethod<&SliderJoint3D::disableMotor>(builder, "disable_motor");
    bindMethod<&SliderJoint3D::getPosition>(builder, "get_position");
}

void bindDistanceJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&DistanceJoint3D::setDistance>(builder, "set_distance");
}

void bindPistonJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&PistonJoint3D::setLinearLimit>(builder, "set_linear_limit");
    bindMethod<&PistonJoint3D::setAngularLimit>(builder, "set_angular_limit");
    bindMethod<&PistonJoint3D::setLinearMotor>(builder, "set_linear_motor");
    bindMethod<&PistonJoint3D::setAngularMotor>(builder, "set_angular_motor");
    bindMethod<&PistonJoint3D::disableLinearMotor>(builder, "disable_linear_motor");
    bindMethod<&PistonJoint3D::disableAngularMotor>(builder, "disable_angular_motor");
    bindMethod<&PistonJoint3D::getPosition>(builder, "get_position");
    bindMethod<&PistonJoint3D::getAngle>(builder, "get_angle");
}

void bindUniversalJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&UniversalJoint3D::setLimitX>(builder, "set_limit_x");
    bindMethod<&UniversalJoint3D::setLimitY>(builder, "set_limit_y");
    bindMethod<&UniversalJoint3D::setMotorX>(builder, "set_motor_x");
    bindMethod<&UniversalJoint3D::setMotorY>(builder, "set_motor_y");
    bindMethod<&UniversalJoint3D::disableMotorX>(builder, "disable_motor_x");
    bindMethod<&UniversalJoint3D::disableMotorY>(builder, "disable_motor_y");
    bindMethod<&UniversalJoint3D::getAngleX>(builder, "get_angle_x");
    bindMethod<&UniversalJoint3D::getAngleY>(builder, "get_angle_y");
}

void bindWheelJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&WheelJoint3D::setSteeringLimit>(builder, "set_steering_limit");
    bindMethod<&WheelJoint3D::setSteeringServo>(builder, "set_steering_servo");
    bindMethod<&WheelJoint3D::disableSteeringServo>(builder, "disable_steering_servo");
    bindMethod<&WheelJoint3D::setSpinMotor>(builder, "set_spin_motor");
    bindMethod<&WheelJoint3D::disableSpinMotor>(builder, "disable_spin_motor");
    bindMethod<&WheelJoint3D::getSteeringAngle>(builder, "get_steering_angle");
    bindMethod<&WheelJoint3D::getSuspensionTravel>(builder, "get_suspension_travel");
}

void bindMouseJointMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&MouseJoint3D::setTarget>(builder, "set_target");
}

void bindSoftBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&SoftBody3D::makeCloth, 4>(builder, "make_cloth", 1.0);
    bindMethod<&SoftBody3D::makeBall, 1>(builder, "make_ball", 12, 18);
    bindMethod<&SoftBody3D::setPinned>(builder, "set_pinned");
    bindMethod<&SoftBody3D::isPinned>(builder, "is_pinned");
    bindMethod<&SoftBody3D::pinInSphere, 2>(builder, "pin_in_sphere", 1);
    builder.method("pin_in_box", &softBodyPinInBox, -1);
    bindMethod<&SoftBody3D::clearPins>(builder, "clear_pins");
    bindMethod<&SoftBody3D::addVelocity>(builder, "add_velocity");
    bindMethod<&SoftBody3D::reset>(builder, "reset");
    bindMethod<&SoftBody3D::isBuilt>(builder, "is_built");
    bindMethod<&SoftBody3D::getParticleCount>(builder, "get_particle_count");
    bindMethod<&SoftBody3D::getParticlePosition>(builder, "get_particle_position");
    bindMethod<&SoftBody3D::getWorstStretch>(builder, "get_worst_stretch");
}

void bindDestructibleBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&DestructibleBody3D::breakAt, 1>(builder, "break_at");
    bindMethod<&DestructibleBody3D::isBroken>(builder, "is_broken");
}

void bindDebrisBodyMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&DebrisBody3D::getAge>(builder, "get_age");
}

void bindVisualInstanceMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&VisualInstance3D::setLayerMaskValue>(builder, "set_layer_mask_value");
    bindMethod<&VisualInstance3D::getLayerMaskValue>(builder, "get_layer_mask_value");
    bindMethod<&VisualInstance3D::getReflector>(builder, "get_reflector");
    bindMethod<&VisualInstance3D::getAabb>(builder, "get_aabb");
}

void bindMeshInstanceMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&MeshInstance3D::setSurfaceVisible>(builder, "set_surface_visible");
    bindMethod<&MeshInstance3D::isSurfaceVisible>(builder, "is_surface_visible");
    bindMethod<&MeshInstance3D::getSkeleton>(builder, "get_skeleton");
}

void bindMultiMeshInstanceMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&MultiMeshInstance3D::setInstanceColor>(builder, "set_instance_color");
    builder.method("get_instance_color", &multiMeshGetInstanceColor, 1);
    bindMethod<&MultiMeshInstance3D::setInstanceCustomData>(builder, "set_instance_custom_data");
    bindMethod<&MultiMeshInstance3D::getInstanceCustomData>(builder, "get_instance_custom_data");
}

void bindManualMeshMethods(zen::VM::ClassBuilder& builder)
{
    builder.method("surface_begin", &manualMeshSurfaceBegin, 1);
    bindMethod<&ManualMesh3D::surfaceSetColor>(builder, "surface_set_color");
    bindMethod<&ManualMesh3D::surfaceSetNormal>(builder, "surface_set_normal");
    bindMethod<&ManualMesh3D::surfaceSetTangent>(builder, "surface_set_tangent");
    bindMethod<&ManualMesh3D::surfaceSetUv>(builder, "surface_set_uv");
    bindMethod<&ManualMesh3D::surfaceSetUv2>(builder, "surface_set_uv2");
    bindMethod<&ManualMesh3D::surfaceAddVertex>(builder, "surface_add_vertex");
    bindMethod<&ManualMesh3D::surfaceAddIndex>(builder, "surface_add_index");
    bindMethod<&ManualMesh3D::surfaceEnd>(builder, "surface_end");
    bindMethod<&ManualMesh3D::setSurfaceMaterialPath>(builder, "set_surface_material");
    bindMethod<&ManualMesh3D::isSurfaceActive>(builder, "is_surface_active");
    bindMethod<&ManualMesh3D::clearSurfaces>(builder, "clear_surfaces");
    bindMethod<&ManualMesh3D::getSurfaceCount>(builder, "get_surface_count");
    bindMethod<&ManualMesh3D::isDynamic>(builder, "is_dynamic");
}

void bindHairMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Hair3D::reset>(builder, "reset");
    bindMethod<&Hair3D::getScalpTriangleCount>(builder, "get_scalp_triangle_count");
    bindMethod<&Hair3D::getSkeleton>(builder, "get_skeleton");
    bindMethod<&Hair3D::clearColliders>(builder, "clear_colliders");
    bindMethod<&Hair3D::getColliderCount>(builder, "get_collider_count");
    bindMethod<&Hair3D::clearForceFields>(builder, "clear_force_fields");
    bindMethod<&Hair3D::isSimulatingOnGpu>(builder, "is_simulating_on_gpu");
}

void bindLightMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Light3D::getLightType>(builder, "get_light_type");
    bindMethod<&Light3D::clearLensFlare>(builder, "clear_lens_flare");
    bindMethod<&Light3D::getLensFlareCount>(builder, "get_lens_flare_count");
}

void bindDecalMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Decal3D::getAabb>(builder, "get_aabb");
}

void bindReflectionProbeMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&ReflectionProbe3D::getAabb>(builder, "get_aabb");
}

void bindLightmapMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&LightmapGI::updateLightData>(builder, "update_light_data");
    bindMethod<&LightmapGI::getAabb>(builder, "get_aabb");
}

void bindViewportMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Viewport::getAspect>(builder, "get_aspect");
    bindMethod<&Viewport::getMousePosition>(builder, "get_mouse_position");
    bindMethod<&Viewport::getMsaaSamples>(builder, "get_msaa_samples");
    bindMethod<&Viewport::getCamera3D>(builder, "get_camera_3d");
}

void bindCameraMethods(zen::VM::ClassBuilder& builder)
{
    bindMethod<&Camera3D::setPerspective>(builder, "set_perspective");
    bindMethod<&Camera3D::setOrthogonal>(builder, "set_orthogonal");
    bindMethod<&Camera3D::clearCurrent, 0>(builder, "clear_current", 1);
    builder.method("project_local_ray_normal", &cameraProjectLocalRayNormal, -1);
    builder.method("project_ray", &cameraProjectRay, -1);
    bindMethod<&Camera3D::projectMouseRay, -1, ProjectingCamera>(builder, "project_mouse_ray");
    builder.method("project_position", &cameraProjectPosition, -1);
    bindMethod<&Camera3D::unprojectPosition, -1, ProjectingCamera>(builder, "unproject_position");
    builder.method("pick_depth", &cameraPickDepth, -1);
    bindMethod<&Camera3D::isPositionBehind>(builder, "is_position_behind");
    bindMethod<&Camera3D::isPositionInFrustum>(builder, "is_position_in_frustum");
    bindMethod<&Camera3D::getViewport>(builder, "get_viewport");
}

struct KeyName
{
    const char* name;
    int value;
};

const KeyName kNamedKeys[] = {
    { "KEY_SPACE", 32 },
    { "KEY_APOSTROPHE", 39 },
    { "KEY_COMMA", 44 },
    { "KEY_MINUS", 45 },
    { "KEY_PERIOD", 46 },
    { "KEY_SLASH", 47 },
    { "KEY_SEMICOLON", 59 },
    { "KEY_EQUAL", 61 },
    { "KEY_LEFT_BRACKET", 91 },
    { "KEY_BACKSLASH", 92 },
    { "KEY_RIGHT_BRACKET", 93 },
    { "KEY_GRAVE", 96 },
    { "KEY_ESCAPE", 256 },
    { "KEY_ENTER", 257 },
    { "KEY_TAB", 258 },
    { "KEY_BACKSPACE", 259 },
    { "KEY_INSERT", 260 },
    { "KEY_DELETE", 261 },
    { "KEY_RIGHT", 262 },
    { "KEY_LEFT", 263 },
    { "KEY_DOWN", 264 },
    { "KEY_UP", 265 },
    { "KEY_PAGE_UP", 266 },
    { "KEY_PAGE_DOWN", 267 },
    { "KEY_HOME", 268 },
    { "KEY_END", 269 },
    { "KEY_CAPS_LOCK", 280 },
    { "KEY_PAUSE", 283 },
    { "KEY_KP_ENTER", 335 },
    { "KEY_LEFT_SHIFT", 340 },
    { "KEY_LEFT_CONTROL", 341 },
    { "KEY_LEFT_ALT", 342 },
    { "KEY_RIGHT_SHIFT", 344 },
    { "KEY_RIGHT_CONTROL", 345 },
    { "KEY_RIGHT_ALT", 346 },
    { "MOUSE_LEFT", 0 },
    { "MOUSE_RIGHT", 1 },
    { "MOUSE_MIDDLE", 2 },
    { "KEY_ANY", kAnyInput },
    { "MOUSE_ANY", kAnyInput },
    { "MOUSE_MODE_VISIBLE", static_cast<int>(MouseMode::Visible) },
    { "MOUSE_MODE_HIDDEN", static_cast<int>(MouseMode::Hidden) },
    { "MOUSE_MODE_CAPTURED", static_cast<int>(MouseMode::Captured) },
    { "EVENT_KEY", static_cast<int>(InputEventType::Key) },
    { "EVENT_CHAR", static_cast<int>(InputEventType::Char) },
    { "EVENT_MOUSE_MOTION", static_cast<int>(InputEventType::MouseMotion) },
    { "EVENT_MOUSE_BUTTON", static_cast<int>(InputEventType::MouseButton) },
    { "EVENT_MOUSE_WHEEL", static_cast<int>(InputEventType::MouseWheel) },
    { "EVENT_TOUCH", static_cast<int>(InputEventType::Touch) },
    { "EVENT_ACTION", static_cast<int>(InputEventType::Action) },
};

const KeyName kEnumConstants[] = {
    { "EMITTER_STATE_STOPPED", static_cast<int>(EmitterState::Stopped) },
    { "EMITTER_STATE_PLAYING", static_cast<int>(EmitterState::Playing) },
    { "EMITTER_STATE_PAUSED", static_cast<int>(EmitterState::Paused) },
    { "BODY_MODE_STATIC", static_cast<int>(PhysicsBodyMode::Static) },
    { "BODY_MODE_KINEMATIC", static_cast<int>(PhysicsBodyMode::Kinematic) },
    { "BODY_MODE_RIGID", static_cast<int>(PhysicsBodyMode::Rigid) },
    { "GROUND_STATE_ON_GROUND", static_cast<int>(PhysicsGroundState::OnGround) },
    { "GROUND_STATE_ON_STEEP_GROUND", static_cast<int>(PhysicsGroundState::OnSteepGround) },
    { "GROUND_STATE_NOT_SUPPORTED", static_cast<int>(PhysicsGroundState::NotSupported) },
    { "GROUND_STATE_IN_AIR", static_cast<int>(PhysicsGroundState::InAir) },
    { "LIGHT_TYPE_DIRECTIONAL", static_cast<int>(LightType::Directional) },
    { "LIGHT_TYPE_OMNI", static_cast<int>(LightType::Omni) },
    { "LIGHT_TYPE_SPOT", static_cast<int>(LightType::Spot) },
    { "TOPOLOGY_TRIANGLES", static_cast<int>(prisma::Topology::Triangles) },
    { "TOPOLOGY_TRIANGLE_STRIP", static_cast<int>(prisma::Topology::TriangleStrip) },
    { "TOPOLOGY_LINES", static_cast<int>(prisma::Topology::Lines) },
    { "TOPOLOGY_LINE_STRIP", static_cast<int>(prisma::Topology::LineStrip) },
    { "TOPOLOGY_POINTS", static_cast<int>(prisma::Topology::Points) },
    { "ALPHA_ANTIALIASING_OFF", static_cast<int>(MaterialAlphaAntialiasing::Off) },
    { "ALPHA_ANTIALIASING_ALPHA_TO_COVERAGE",
        static_cast<int>(MaterialAlphaAntialiasing::AlphaToCoverage) },
    { "AUDIO_BUS_SFX", static_cast<int>(AudioBus::Sfx) },
    { "AUDIO_BUS_MUSIC", static_cast<int>(AudioBus::Music) },
    { "RAGDOLL_PART_PELVIS", static_cast<int>(PhysicsRagdollPart::Pelvis) },
    { "RAGDOLL_PART_SPINE", static_cast<int>(PhysicsRagdollPart::Spine) },
    { "RAGDOLL_PART_HEAD", static_cast<int>(PhysicsRagdollPart::Head) },
    { "RAGDOLL_PART_LEFT_UPPER_LEG", static_cast<int>(PhysicsRagdollPart::LeftUpperLeg) },
    { "RAGDOLL_PART_LEFT_LOWER_LEG", static_cast<int>(PhysicsRagdollPart::LeftLowerLeg) },
    { "RAGDOLL_PART_RIGHT_UPPER_LEG", static_cast<int>(PhysicsRagdollPart::RightUpperLeg) },
    { "RAGDOLL_PART_RIGHT_LOWER_LEG", static_cast<int>(PhysicsRagdollPart::RightLowerLeg) },
    { "RAGDOLL_PART_LEFT_UPPER_ARM", static_cast<int>(PhysicsRagdollPart::LeftUpperArm) },
    { "RAGDOLL_PART_LEFT_LOWER_ARM", static_cast<int>(PhysicsRagdollPart::LeftLowerArm) },
    { "RAGDOLL_PART_RIGHT_UPPER_ARM", static_cast<int>(PhysicsRagdollPart::RightUpperArm) },
    { "RAGDOLL_PART_RIGHT_LOWER_ARM", static_cast<int>(PhysicsRagdollPart::RightLowerArm) },
    { "RAGDOLL_PART_LEFT_FOOT", static_cast<int>(PhysicsRagdollPart::LeftFoot) },
    { "RAGDOLL_PART_RIGHT_FOOT", static_cast<int>(PhysicsRagdollPart::RightFoot) },
    { "RAGDOLL_BONE_HIPS", static_cast<int>(RagdollBone::Hips) },
    { "RAGDOLL_BONE_SPINE", static_cast<int>(RagdollBone::Spine) },
    { "RAGDOLL_BONE_NECK", static_cast<int>(RagdollBone::Neck) },
    { "RAGDOLL_BONE_HEAD", static_cast<int>(RagdollBone::Head) },
    { "RAGDOLL_BONE_LEFT_ARM", static_cast<int>(RagdollBone::LeftArm) },
    { "RAGDOLL_BONE_LEFT_FORE_ARM", static_cast<int>(RagdollBone::LeftForeArm) },
    { "RAGDOLL_BONE_LEFT_HAND", static_cast<int>(RagdollBone::LeftHand) },
    { "RAGDOLL_BONE_RIGHT_ARM", static_cast<int>(RagdollBone::RightArm) },
    { "RAGDOLL_BONE_RIGHT_FORE_ARM", static_cast<int>(RagdollBone::RightForeArm) },
    { "RAGDOLL_BONE_RIGHT_HAND", static_cast<int>(RagdollBone::RightHand) },
    { "RAGDOLL_BONE_LEFT_UP_LEG", static_cast<int>(RagdollBone::LeftUpLeg) },
    { "RAGDOLL_BONE_LEFT_LEG", static_cast<int>(RagdollBone::LeftLeg) },
    { "RAGDOLL_BONE_LEFT_FOOT", static_cast<int>(RagdollBone::LeftFoot) },
    { "RAGDOLL_BONE_LEFT_TOES", static_cast<int>(RagdollBone::LeftToes) },
    { "RAGDOLL_BONE_RIGHT_UP_LEG", static_cast<int>(RagdollBone::RightUpLeg) },
    { "RAGDOLL_BONE_RIGHT_LEG", static_cast<int>(RagdollBone::RightLeg) },
    { "RAGDOLL_BONE_RIGHT_FOOT", static_cast<int>(RagdollBone::RightFoot) },
    { "RAGDOLL_BONE_RIGHT_TOES", static_cast<int>(RagdollBone::RightToes) },
    { "BONE_DIRECTION_PLUS_X", static_cast<int>(SkeletonModifier3D::BoneDirection::PlusX) },
    { "BONE_DIRECTION_MINUS_X", static_cast<int>(SkeletonModifier3D::BoneDirection::MinusX) },
    { "BONE_DIRECTION_PLUS_Y", static_cast<int>(SkeletonModifier3D::BoneDirection::PlusY) },
    { "BONE_DIRECTION_MINUS_Y", static_cast<int>(SkeletonModifier3D::BoneDirection::MinusY) },
    { "BONE_DIRECTION_PLUS_Z", static_cast<int>(SkeletonModifier3D::BoneDirection::PlusZ) },
    { "BONE_DIRECTION_MINUS_Z", static_cast<int>(SkeletonModifier3D::BoneDirection::MinusZ) },
    { "BONE_DIRECTION_FROM_PARENT",
        static_cast<int>(SkeletonModifier3D::BoneDirection::FromParent) },
    { "ROTATION_AXIS_X", static_cast<int>(SkeletonModifier3D::RotationAxis::X) },
    { "ROTATION_AXIS_Y", static_cast<int>(SkeletonModifier3D::RotationAxis::Y) },
    { "ROTATION_AXIS_Z", static_cast<int>(SkeletonModifier3D::RotationAxis::Z) },
    { "ROTATION_AXIS_ALL", static_cast<int>(SkeletonModifier3D::RotationAxis::All) },
    { "ROTATION_AXIS_CUSTOM", static_cast<int>(SkeletonModifier3D::RotationAxis::Custom) },
    { "CENTER_FROM_WORLD_ORIGIN",
        static_cast<int>(SpringBoneSimulator3D::CenterFrom::WorldOrigin) },
    { "CENTER_FROM_NODE", static_cast<int>(SpringBoneSimulator3D::CenterFrom::Node) },
    { "CENTER_FROM_BONE", static_cast<int>(SpringBoneSimulator3D::CenterFrom::Bone) },
    { "MODIFIER_CALLBACK_MODE_PHYSICS",
        static_cast<int>(Skeleton3D::ModifierCallbackModeProcess::Physics) },
    { "MODIFIER_CALLBACK_MODE_IDLE",
        static_cast<int>(Skeleton3D::ModifierCallbackModeProcess::Idle) },
    { "MODIFIER_CALLBACK_MODE_MANUAL",
        static_cast<int>(Skeleton3D::ModifierCallbackModeProcess::Manual) },
    { "TRANS_LINEAR", static_cast<int>(TransitionType::Linear) },
    { "TRANS_SINE", static_cast<int>(TransitionType::Sine) },
    { "TRANS_QUINT", static_cast<int>(TransitionType::Quint) },
    { "TRANS_QUART", static_cast<int>(TransitionType::Quart) },
    { "TRANS_QUAD", static_cast<int>(TransitionType::Quad) },
    { "TRANS_EXPO", static_cast<int>(TransitionType::Expo) },
    { "TRANS_ELASTIC", static_cast<int>(TransitionType::Elastic) },
    { "TRANS_CUBIC", static_cast<int>(TransitionType::Cubic) },
    { "TRANS_CIRC", static_cast<int>(TransitionType::Circ) },
    { "TRANS_BOUNCE", static_cast<int>(TransitionType::Bounce) },
    { "TRANS_BACK", static_cast<int>(TransitionType::Back) },
    { "TRANS_SPRING", static_cast<int>(TransitionType::Spring) },
    { "EASE_IN", static_cast<int>(EaseType::In) },
    { "EASE_OUT", static_cast<int>(EaseType::Out) },
    { "EASE_IN_OUT", static_cast<int>(EaseType::InOut) },
    { "EASE_OUT_IN", static_cast<int>(EaseType::OutIn) },
};

enum EventField : int
{
    kFieldType = 0,
    kFieldPressed,
    kFieldEcho,
    kFieldKey,
    kFieldScancode,
    kFieldModifiers,
    kFieldPosition,
    kFieldRelative,
    kFieldButton,
    kFieldButtonMask,
    kFieldWheel,
    kFieldCodepoint,
    kFieldTouchId,
    kFieldPressure,
    kFieldAction,
    kFieldStrength,
    kFieldCount
};

const char* const kEventFields[kFieldCount] = { "type", "pressed", "echo", "key", "scancode",
    "modifiers", "position", "relative", "button", "button_mask", "wheel", "codepoint", "touch_id",
    "pressure", "action", "strength" };

Gui* hudGui()
{
    Gui* gui = Main::gui();
    return gui && gui->frameOpen() ? gui : nullptr;
}

ig::Color hudColor(const Math::Vec4& color)
{
    const auto channel = [](float value)
    {
        const float scaled = value * 255.0f + 0.5f;
        return static_cast<std::uint8_t>(scaled < 0.0f ? 0.0f : scaled > 255.0f ? 255.0f : scaled);
    };
    return ig::Color(channel(color.x), channel(color.y), channel(color.z), channel(color.w));
}

bool argHudColor(zen::VM* vm, zen::Value* args, int nargs, int index, const char* name,
        ig::Color& out)
{
    out = ig::Color(255, 255, 255, 255);
    if (nargs <= index) return true;
    Math::Vec4 color;
    if (!argVec4(vm, args, index, name, color)) return false;
    out = hudColor(color);
    return true;
}

bool argNumbers(zen::VM* vm, zen::Value* args, int count, const char* name, double* out)
{
    for (int i = 0; i < count; ++i)
        if (!argNumber(vm, args, i, name, out[i])) return false;
    return true;
}

int hudDrawText(zen::VM* vm, zen::Value* args, int nargs)
{
    double values[2] = { 0.0, 0.0 };
    double size = 0.0;
    ct::String text;
    ig::Color color;
    if (!needArgs(vm, nargs, 3, 5, "hud_text") || !argNumbers(vm, args, 2, "hud_text", values) ||
            !argString(vm, args, 2, "hud_text", text) ||
            !argHudColor(vm, args, nargs, 3, "hud_text", color) ||
            (nargs > 4 && !argNumber(vm, args, 4, "hud_text", size)))
        return -1;
    if (Gui* gui = hudGui())
        gui->hudText(ig::StringView(text.c_str(), text.size()), static_cast<float>(values[0]),
                static_cast<float>(values[1]), color, static_cast<float>(size));
    return 0;
}

int hudDrawRect(zen::VM* vm, zen::Value* args, int nargs)
{
    double values[4] = { 0.0, 0.0, 0.0, 0.0 };
    ig::Color color;
    if (!needArgs(vm, nargs, 4, 5, "hud_rect") || !argNumbers(vm, args, 4, "hud_rect", values) ||
            !argHudColor(vm, args, nargs, 4, "hud_rect", color))
        return -1;
    if (Gui* gui = hudGui())
        gui->hudRectFilled(ig::Rect(static_cast<float>(values[0]), static_cast<float>(values[1]),
                                   static_cast<float>(values[2]), static_cast<float>(values[3])),
                color);
    return 0;
}

int hudDrawRectOutline(zen::VM* vm, zen::Value* args, int nargs)
{
    double values[4] = { 0.0, 0.0, 0.0, 0.0 };
    double thickness = 1.0;
    ig::Color color;
    if (!needArgs(vm, nargs, 4, 6, "hud_rect_outline") ||
            !argNumbers(vm, args, 4, "hud_rect_outline", values) ||
            !argHudColor(vm, args, nargs, 4, "hud_rect_outline", color) ||
            (nargs > 5 && !argNumber(vm, args, 5, "hud_rect_outline", thickness)))
        return -1;
    if (Gui* gui = hudGui())
        gui->hudRectOutline(ig::Rect(static_cast<float>(values[0]), static_cast<float>(values[1]),
                                    static_cast<float>(values[2]), static_cast<float>(values[3])),
                color, static_cast<float>(thickness));
    return 0;
}

int hudDrawLine(zen::VM* vm, zen::Value* args, int nargs)
{
    double values[4] = { 0.0, 0.0, 0.0, 0.0 };
    double thickness = 1.0;
    ig::Color color;
    if (!needArgs(vm, nargs, 4, 6, "hud_line") || !argNumbers(vm, args, 4, "hud_line", values) ||
            !argHudColor(vm, args, nargs, 4, "hud_line", color) ||
            (nargs > 5 && !argNumber(vm, args, 5, "hud_line", thickness)))
        return -1;
    if (Gui* gui = hudGui())
        gui->hudLine(static_cast<float>(values[0]), static_cast<float>(values[1]),
                static_cast<float>(values[2]), static_cast<float>(values[3]), color,
                static_cast<float>(thickness));
    return 0;
}

int hudDrawCircle(zen::VM* vm, zen::Value* args, int nargs)
{
    double values[3] = { 0.0, 0.0, 0.0 };
    ig::Color color;
    if (!needArgs(vm, nargs, 3, 4, "hud_circle") ||
            !argNumbers(vm, args, 3, "hud_circle", values) ||
            !argHudColor(vm, args, nargs, 3, "hud_circle", color))
        return -1;
    if (Gui* gui = hudGui())
        gui->hudCircleFilled(static_cast<float>(values[0]), static_cast<float>(values[1]),
                static_cast<float>(values[2]), color);
    return 0;
}

bool argHudImage(zen::VM* vm, zen::Value* args, const char* name, TextureId& out)
{
    if (zen::is_string(args[0]))
    {
        ct::String path;
        if (!argString(vm, args, 0, name, path)) return false;
        out = sceneres::loadTexture(path, true);
        return true;
    }
    if (zen::is_numeric_like(args[0]))
    {
        const double bits = zen::to_number(args[0]);
        out = bits < 0.0 ? TextureId() : TextureId::from_bits(static_cast<std::uint32_t>(bits));
        return true;
    }
    vm->runtime_error("%s: argument 1 must be an image path or a value from hud_load_image", name);
    return false;
}

int hudLoadImage(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String path;
    if (!needArgs(vm, nargs, 1, 1, "hud_load_image") ||
            !argString(vm, args, 0, "hud_load_image", path))
        return -1;
    const TextureId texture = sceneres::loadTexture(path, true);
    args[0] = zen::val_int(texture.valid() ? static_cast<long long>(texture.bits()) : -1LL);
    return 1;
}

int hudDrawImage(zen::VM* vm, zen::Value* args, int nargs)
{
    TextureId texture;
    double values[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0 };
    ig::Color color;
    if (!needArgs(vm, nargs, 5, 11, "hud_image") || !argHudImage(vm, args, "hud_image", texture))
        return -1;
    for (int i = 1; i < nargs && i < 10; ++i)
        if (!argNumber(vm, args, i, "hud_image", values[i - 1])) return -1;
    if (nargs == 7 || nargs == 8 || nargs == 9)
    {
        vm->runtime_error("hud_image: give all four of u0, v0, u1, v1");
        return -1;
    }
    if (!argHudColor(vm, args, nargs, 10, "hud_image", color)) return -1;
    Gui* gui = hudGui();
    if (!gui || !texture.valid()) return 0;
    gui->hudImage(texture,
            ig::Rect(static_cast<float>(values[0]), static_cast<float>(values[1]),
                    static_cast<float>(values[2]), static_cast<float>(values[3])),
            static_cast<float>(values[4]),
            ig::Vec2(static_cast<float>(values[5]), static_cast<float>(values[6])),
            ig::Vec2(static_cast<float>(values[7]), static_cast<float>(values[8])), color);
    return 0;
}

int hudGetTextSize(zen::VM* vm, zen::Value* args, int nargs)
{
    ct::String text;
    double size = 0.0;
    if (!needArgs(vm, nargs, 1, 2, "hud_text_size") ||
            !argString(vm, args, 0, "hud_text_size", text) ||
            (nargs > 1 && !argNumber(vm, args, 1, "hud_text_size", size)))
        return -1;
    ig::Vec2 metrics;
    if (Gui* gui = Main::gui())
        metrics = gui->hudTextSize(ig::StringView(text.c_str(), text.size()),
                static_cast<float>(size));
    args[0] = makeVec2(Math::Vec2(metrics.x, metrics.y));
    return 1;
}

int windowSetFullscreen(zen::VM* vm, zen::Value* args, int nargs)
{
    if (!needArgs(vm, nargs, 1, 1, "window_set_fullscreen")) return -1;
    Device* device = Main::device();
    if (!device) return 0;
    const bool fullscreen = zen::is_bool(args[0]) ? args[0].as.boolean : !zen::is_nil(args[0]);
    device->windowSetMode(fullscreen ? WindowMode::Fullscreen : WindowMode::Windowed);
    return 0;
}

int windowIsFullscreen(zen::VM*, zen::Value* args, int)
{
    const Device* device = Main::device();
    const WindowMode mode = device ? device->windowGetMode() : WindowMode::Windowed;
    args[0] = zen::val_bool(
            mode == WindowMode::Fullscreen || mode == WindowMode::ExclusiveFullscreen);
    return 1;
}

int buttonPress(zen::VM* vm, zen::Value* args, int)
{
    Button* self = selfNodeAs<Button>(vm, args);
    if (!self) return -1;
    self->press();
    return 0;
}

int buttonIsHovered(zen::VM* vm, zen::Value* args, int)
{
    Button* self = selfNodeAs<Button>(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->isHovered());
    return 1;
}

int lineEditGrabFocus(zen::VM* vm, zen::Value* args, int)
{
    LineEdit* self = selfNodeAs<LineEdit>(vm, args);
    if (!self) return -1;
    self->grabFocus();
    return 0;
}

int lineEditReleaseFocus(zen::VM* vm, zen::Value* args, int)
{
    LineEdit* self = selfNodeAs<LineEdit>(vm, args);
    if (!self) return -1;
    self->releaseFocus();
    return 0;
}

int lineEditHasFocus(zen::VM* vm, zen::Value* args, int)
{
    LineEdit* self = selfNodeAs<LineEdit>(vm, args);
    if (!self) return -1;
    args[0] = zen::val_bool(self->hasFocus());
    return 1;
}

int lineEditClear(zen::VM* vm, zen::Value* args, int)
{
    LineEdit* self = selfNodeAs<LineEdit>(vm, args);
    if (!self) return -1;
    self->clear();
    return 0;
}

int lineEditSubmit(zen::VM* vm, zen::Value* args, int)
{
    LineEdit* self = selfNodeAs<LineEdit>(vm, args);
    if (!self) return -1;
    self->submit();
    return 0;
}

Tween* selfTween(zen::VM* vm, zen::Value* args, const char* name)
{
    Tween* tween = selfData<Tween>(vm, args, "the tween");
    if (!tween) return nullptr;
    SceneTree* tree = SceneTree::singleton();
    if (!tree || !tree->hasTween(tween))
    {
        vm->runtime_error("%s: the tween has finished or was killed", name);
        return nullptr;
    }
    return tween;
}

ZenScripting::Impl::TweenerHandle* selfTweener(zen::VM* vm, zen::Value* args, const char* name)
{
    ZenScripting::Impl::TweenerHandle* handle =
            selfData<ZenScripting::Impl::TweenerHandle>(vm, args, "the tweener");
    if (!handle) return nullptr;
    SceneTree* tree = SceneTree::singleton();
    if (!tree || !tree->hasTween(handle->tween))
    {
        vm->runtime_error("%s: the tween has finished or was killed", name);
        return nullptr;
    }
    return handle;
}

bool tweenValueFrom(zen::VM* vm, zen::Value value, const char* name, int index, TweenValue& out)
{
    if (zen::is_numeric_like(value))
    {
        out = TweenValue(zen::to_number(value));
        return true;
    }
    if (zen::is_vec2(value))
    {
        out = TweenValue(Math::Vec3(zen::vec_x(value), zen::vec_y(value), 0.0f));
        return true;
    }
    if (zen::is_vec3(value))
    {
        out = TweenValue(Math::Vec3(zen::vec_x(value), zen::vec_y(value), zen::vec_z(value)));
        return true;
    }
    if (zen::is_native_struct(value))
    {
        const zen::NativeStructDef* def = zen::as_native_struct(value)->def;
        if (def == impl().vec4Def || def == impl().colorDef)
        {
            const float* data = static_cast<const float*>(zen::as_native_struct(value)->data);
            out = TweenValue(Math::Vec4(data[0], data[1], data[2], data[3]));
            return true;
        }
    }
    vm->runtime_error("%s: argument %d must be a number, Vec2, Vec3 or Color", name, index + 1);
    return false;
}

TweenValue tweenValueOfProperty(const PropertyValue& value)
{
    switch (value.type())
    {
        case PropertyType::Bool:
            return TweenValue(value.asBool() ? 1.0 : 0.0);
        case PropertyType::Int:
            return TweenValue(static_cast<double>(value.asInt()));
        case PropertyType::Float:
            return TweenValue(value.asFloat());
        case PropertyType::Vec2:
        {
            const Math::Vec2 v = value.asVec2();
            return TweenValue(Math::Vec3(v.x, v.y, 0.0f));
        }
        case PropertyType::Vec3:
            return TweenValue(value.asVec3());
        case PropertyType::Vec4:
            return TweenValue(value.asVec4());
        case PropertyType::Color:
            return TweenValue(value.asColor());
        default:
            return TweenValue(0.0);
    }
}

PropertyValue propertyFromTween(const TweenValue& value, PropertyType type)
{
    switch (type)
    {
        case PropertyType::Bool:
            return PropertyValue::fromBool(value.asFloat() >= 0.5);
        case PropertyType::Int:
            return PropertyValue::fromInt(static_cast<std::int64_t>(
                    value.asFloat() + (value.asFloat() < 0.0 ? -0.5 : 0.5)));
        case PropertyType::Float:
            return PropertyValue::fromFloat(value.asFloat());
        case PropertyType::Vec2:
        {
            const Math::Vec3 v = value.asVec3();
            return PropertyValue::fromVec2(Math::Vec2(v.x, v.y));
        }
        case PropertyType::Vec3:
            return PropertyValue::fromVec3(value.asVec3());
        case PropertyType::Vec4:
            return PropertyValue::fromVec4(value.asColor());
        case PropertyType::Color:
            return PropertyValue::fromColor(value.asColor());
        default:
            return PropertyValue();
    }
}

int createTween(zen::VM* vm, zen::Value* args, int)
{
    SceneTree* tree = SceneTree::singleton();
    if (!tree)
    {
        args[0] = zen::val_nil();
        return 1;
    }
    args[0] = vm->make_instance_borrowed(impl().tweenClass, tree->createTween());
    return 1;
}

int tweenTweenProperty(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "tween_property");
    Node* node = nullptr;
    ct::String name;
    double seconds = 0.0;
    if (!tween || !needArgs(vm, nargs, 4, 4, "tween_property") ||
            !argNode(vm, args, 0, "tween_property", node) ||
            !argString(vm, args, 1, "tween_property", name) ||
            !argNumber(vm, args, 3, "tween_property", seconds))
        return -1;
    PropertyValue current;
    if (!node || !NodeClassDB::getProperty(node, name, current))
    {
        vm->runtime_error("tween_property: %s has no property '%s'",
                node ? node->getClassName() : "None", name.c_str());
        return -1;
    }
    TweenValue to;
    if (!tweenValueFrom(vm, args[2], "tween_property", 2, to)) return -1;
    const PropertyType type = current.type();
    if (tweenValueOfProperty(current).type() != to.type())
    {
        vm->runtime_error("tween_property: the value does not match the type of '%s'",
                name.c_str());
        return -1;
    }
    const ct::Function<TweenValue()> getter = [node, name]()
    {
        PropertyValue value;
        NodeClassDB::getProperty(node, name, value);
        return tweenValueOfProperty(value);
    };
    const ct::Function<void(const TweenValue&)> setter = [node, name, type](const TweenValue& value)
    { NodeClassDB::setProperty(node, name, propertyFromTween(value, type)); };
    PropertyTweener* tweener = tween->tweenProperty(getter, setter, to, seconds);
    if (!tweener)
    {
        vm->runtime_error("tween_property: the tween could not take the property");
        return -1;
    }
    if (!tween->getBoundNode()) tween->bindNode(node);
    ZenScripting::Impl::TweenerHandle* handle =
            new ZenScripting::Impl::TweenerHandle{ tween, tweener };
    impl().tweenerHandles.push_back(handle);
    args[0] = vm->make_instance_borrowed(impl().tweenerClass, handle);
    return 1;
}

int tweenTweenInterval(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "tween_interval");
    double seconds = 0.0;
    if (!tween || !needArgs(vm, nargs, 1, 1, "tween_interval") ||
            !argNumber(vm, args, 0, "tween_interval", seconds))
        return -1;
    tween->tweenInterval(seconds);
    args[0] = args[-1];
    return 1;
}

int tweenTweenCallback(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "tween_callback");
    if (!tween || !needArgs(vm, nargs, 1, 1, "tween_callback")) return -1;
    if (!zen::is_closure(args[0]) && !zen::is_bound_method(args[0]) && !zen::is_native(args[0]))
    {
        vm->runtime_error("tween_callback: argument 1 must be a function");
        return -1;
    }
    ZenScripting::Impl::TweenHold hold;
    hold.tween = tween;
    hold.ref = zen::Ref(*vm, args[0]);
    impl().tweenHolds.push_back(static_cast<ZenScripting::Impl::TweenHold&&>(hold));
    const zen::Value callback = args[0];
    tween->tweenCallback(
            [vm, callback]()
            {
                vm->call_fn(callback, nullptr, 0);
                if (vm->had_error())
                {
                    log_error("script: a tween callback failed");
                    impl().clearVmError();
                }
            });
    args[0] = args[-1];
    return 1;
}

int tweenSetTrans(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "set_trans");
    double value = 0.0;
    if (!tween || !needArgs(vm, nargs, 1, 1, "set_trans") ||
            !argNumber(vm, args, 0, "set_trans", value))
        return -1;
    const int index = static_cast<int>(value);
    if (index < 0 || index >= static_cast<int>(TransitionType::Max))
    {
        vm->runtime_error("set_trans: unknown transition %d", index);
        return -1;
    }
    tween->setTrans(static_cast<TransitionType>(index));
    args[0] = args[-1];
    return 1;
}

int tweenSetEase(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "set_ease");
    double value = 0.0;
    if (!tween || !needArgs(vm, nargs, 1, 1, "set_ease") ||
            !argNumber(vm, args, 0, "set_ease", value))
        return -1;
    const int index = static_cast<int>(value);
    if (index < 0 || index >= static_cast<int>(EaseType::Max))
    {
        vm->runtime_error("set_ease: unknown easing %d", index);
        return -1;
    }
    tween->setEase(static_cast<EaseType>(index));
    args[0] = args[-1];
    return 1;
}

int tweenSetParallel(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "set_parallel");
    if (!tween || !needArgs(vm, nargs, 0, 1, "set_parallel")) return -1;
    const bool parallel = nargs == 0 || !(zen::is_bool(args[0]) && !args[0].as.boolean);
    tween->setParallel(parallel);
    args[0] = args[-1];
    return 1;
}

int tweenParallel(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfTween(vm, args, "parallel");
    if (!tween) return -1;
    tween->parallel();
    args[0] = args[-1];
    return 1;
}

int tweenChain(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfTween(vm, args, "chain");
    if (!tween) return -1;
    tween->chain();
    args[0] = args[-1];
    return 1;
}

int tweenSetLoops(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "set_loops");
    double loops = 0.0;
    if (!tween || !needArgs(vm, nargs, 0, 1, "set_loops") ||
            (nargs == 1 && !argNumber(vm, args, 0, "set_loops", loops)))
        return -1;
    tween->setLoops(static_cast<int>(loops));
    args[0] = args[-1];
    return 1;
}

int tweenSetSpeedScale(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "set_speed_scale");
    double speed = 1.0;
    if (!tween || !needArgs(vm, nargs, 1, 1, "set_speed_scale") ||
            !argNumber(vm, args, 0, "set_speed_scale", speed))
        return -1;
    tween->setSpeedScale(static_cast<float>(speed));
    args[0] = args[-1];
    return 1;
}

int tweenBindNode(zen::VM* vm, zen::Value* args, int nargs)
{
    Tween* tween = selfTween(vm, args, "bind_node");
    Node* node = nullptr;
    if (!tween || !needArgs(vm, nargs, 1, 1, "bind_node") ||
            !argNode(vm, args, 0, "bind_node", node))
        return -1;
    tween->bindNode(node);
    args[0] = args[-1];
    return 1;
}

int tweenStop(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfTween(vm, args, "stop");
    if (!tween) return -1;
    tween->stop();
    return 0;
}

int tweenPause(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfTween(vm, args, "pause");
    if (!tween) return -1;
    tween->pause();
    return 0;
}

int tweenPlay(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfTween(vm, args, "play");
    if (!tween) return -1;
    tween->play();
    return 0;
}

int tweenKill(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfData<Tween>(vm, args, "the tween");
    SceneTree* tree = SceneTree::singleton();
    if (tween && tree && tree->hasTween(tween)) tween->kill();
    return 0;
}

int tweenIsRunning(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfData<Tween>(vm, args, "the tween");
    SceneTree* tree = SceneTree::singleton();
    args[0] = zen::val_bool(tween && tree && tree->hasTween(tween) && tween->isRunning());
    return 1;
}

int tweenIsValid(zen::VM* vm, zen::Value* args, int)
{
    Tween* tween = selfData<Tween>(vm, args, "the tween");
    SceneTree* tree = SceneTree::singleton();
    args[0] = zen::val_bool(tween && tree && tree->hasTween(tween) && tween->isValid());
    return 1;
}

int tweenerSetTrans(zen::VM* vm, zen::Value* args, int nargs)
{
    ZenScripting::Impl::TweenerHandle* handle = selfTweener(vm, args, "set_trans");
    double value = 0.0;
    if (!handle || !needArgs(vm, nargs, 1, 1, "set_trans") ||
            !argNumber(vm, args, 0, "set_trans", value))
        return -1;
    const int index = static_cast<int>(value);
    if (index < 0 || index >= static_cast<int>(TransitionType::Max))
    {
        vm->runtime_error("set_trans: unknown transition %d", index);
        return -1;
    }
    handle->tweener->setTrans(static_cast<TransitionType>(index));
    args[0] = args[-1];
    return 1;
}

int tweenerSetEase(zen::VM* vm, zen::Value* args, int nargs)
{
    ZenScripting::Impl::TweenerHandle* handle = selfTweener(vm, args, "set_ease");
    double value = 0.0;
    if (!handle || !needArgs(vm, nargs, 1, 1, "set_ease") ||
            !argNumber(vm, args, 0, "set_ease", value))
        return -1;
    const int index = static_cast<int>(value);
    if (index < 0 || index >= static_cast<int>(EaseType::Max))
    {
        vm->runtime_error("set_ease: unknown easing %d", index);
        return -1;
    }
    handle->tweener->setEase(static_cast<EaseType>(index));
    args[0] = args[-1];
    return 1;
}

int tweenerSetDelay(zen::VM* vm, zen::Value* args, int nargs)
{
    ZenScripting::Impl::TweenerHandle* handle = selfTweener(vm, args, "set_delay");
    double delay = 0.0;
    if (!handle || !needArgs(vm, nargs, 1, 1, "set_delay") ||
            !argNumber(vm, args, 0, "set_delay", delay))
        return -1;
    handle->tweener->setDelay(delay);
    args[0] = args[-1];
    return 1;
}

int tweenerFrom(zen::VM* vm, zen::Value* args, int nargs)
{
    ZenScripting::Impl::TweenerHandle* handle = selfTweener(vm, args, "from_value");
    TweenValue value;
    if (!handle || !needArgs(vm, nargs, 1, 1, "from_value") ||
            !tweenValueFrom(vm, args[0], "from", 0, value))
        return -1;
    handle->tweener->from(value);
    args[0] = args[-1];
    return 1;
}

int tweenerFromCurrent(zen::VM* vm, zen::Value* args, int)
{
    ZenScripting::Impl::TweenerHandle* handle = selfTweener(vm, args, "from_current");
    if (!handle) return -1;
    handle->tweener->fromCurrent();
    args[0] = args[-1];
    return 1;
}

int tweenerAsRelative(zen::VM* vm, zen::Value* args, int)
{
    ZenScripting::Impl::TweenerHandle* handle = selfTweener(vm, args, "as_relative");
    if (!handle) return -1;
    handle->tweener->asRelative();
    args[0] = args[-1];
    return 1;
}

int hudGetWidth(zen::VM*, zen::Value* args, int)
{
    const Gui* gui = Main::gui();
    args[0] = zen::val_float(gui ? gui->hudRect().width : 0.0f);
    return 1;
}

int hudGetHeight(zen::VM*, zen::Value* args, int)
{
    const Gui* gui = Main::gui();
    args[0] = zen::val_float(gui ? gui->hudRect().height : 0.0f);
    return 1;
}

} // namespace

int ZenScripting::Impl::propertySlot(const char* name)
{
    const ct::String key(name);
    if (const int* found = propertyIndex.find(key)) return *found;
    if (static_cast<int>(propertyNames.size()) >= kMaxPropertySlots)
    {
        if (!propertyOverflowLogged)
        {
            propertyOverflowLogged = true;
            log_warn("script: more than %d distinct property names, the rest are only reachable "
                     "with get()/set()",
                    kMaxPropertySlots);
        }
        return -1;
    }
    const int slot = static_cast<int>(propertyNames.size());
    propertyNames.push_back(key);
    propertyIndex.put(key, slot);
    return slot;
}

const ct::String& ZenScripting::Impl::propertyName(int slot) const
{
    return propertyNames[static_cast<std::size_t>(slot)];
}

void ZenScripting::Impl::registerNodeClasses()
{
    ensureThunks();

    ct::Vector<ct::String> names;
    NodeClassDB::getClassList(names);
    ct::Vector<char> done;
    for (std::size_t i = 0; i < names.size(); ++i) done.push_back(0);

    bool progress = true;
    while (progress)
    {
        progress = false;
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            if (done[i]) continue;
            const ct::String parent = NodeClassDB::getParentClass(names[i]);
            if (!parent.empty() && !classes.contains(parent)) continue;

            zen::VM::ClassBuilder builder = vm.def_class(names[i].c_str());
            if (!parent.empty()) builder.parent(parent.c_str());
            builder.constructable(false);

            ct::Vector<PropertyInfo> properties;
            NodeClassDB::getPropertyList(names[i], properties, true);
            for (std::size_t p = 0; p < properties.size(); ++p)
            {
                const PropertyInfo& info = properties[p];
                if (info.isGroup()) continue;
                const ct::String attribute = attributeName(info.name);
                if (!isIdentifier(attribute)) continue;
                const int slot = propertySlot(info.name.c_str());
                if (slot < 0) continue;
                builder.property(attribute.c_str(), sGetters[slot], sSetters[slot]);
            }

            if (names[i] == "Node")
            {
                builder.method("get_name", &nodeGetName, 0);
                builder.method("set_name", &nodeSetName, 1);
                builder.method("get_parent", &nodeGetParent, 0);
                builder.method("get_child_count", &nodeGetChildCount, 0);
                builder.method("get_child", &nodeGetChild, 1);
                builder.method("get_children", &nodeGetChildren, 0);
                builder.method("get_index", &nodeGetIndex, 0);
                builder.method("get_node", &nodeGetNode, 1);
                builder.method("find_child", &nodeFindChild, -1);
                builder.method("create_child", &nodeCreateChild, -1);
                builder.method("instantiate", &nodeInstantiate, 1);
                builder.method("queue_free", &nodeQueueFree, 0);
                builder.method("is_queued_for_deletion", &nodeIsQueuedForDeletion, 0);
                builder.method("add_to_group", &nodeAddToGroup, 1);
                builder.method("remove_from_group", &nodeRemoveFromGroup, 1);
                builder.method("is_in_group", &nodeIsInGroup, 1);
                builder.method("get_groups", &nodeGetGroups, 0);
                builder.method("get_tree", &nodeGetTree, 0);
                builder.method("is_inside_tree", &nodeIsInsideTree, 0);
                builder.method("is_ready", &nodeIsReady, 0);
                builder.method("set_process", &nodeSetProcess, 1);
                builder.method("is_processing", &nodeIsProcessing, 0);
                builder.method("set_physics_process", &nodeSetPhysicsProcess, 1);
                builder.method("is_physics_processing", &nodeIsPhysicsProcessing, 0);
                builder.method("set_process_input", &nodeSetProcessInput, 1);
                builder.method("is_processing_input", &nodeIsProcessingInput, 0);
                builder.method("can_process", &nodeCanProcess, 0);
                builder.method("get_path", &nodeGetPath, 0);
                builder.method("get_class", &nodeGetClass, 0);
                builder.method("is_class", &nodeIsClass, 1);
                builder.method("reparent", &nodeReparent, 1);
                builder.method("get_script", &nodeGetScript, 0);
                builder.method("get", &nodeGet, 1);
                builder.method("set", &nodeSet, 2);
                bindNodeMethods(builder);
            }
            else if (names[i] == "Node3D")
            {
                builder.method("translate", &node3dTranslate, 1);
                builder.method("translate_object_local", &node3dTranslateObjectLocal, 1);
                builder.method("rotate", &node3dRotate, 2);
                builder.method("rotate_object_local", &node3dRotateObjectLocal, 2);
                builder.method("rotate_x", &node3dRotateX, 1);
                builder.method("rotate_y", &node3dRotateY, 1);
                builder.method("rotate_z", &node3dRotateZ, 1);
                builder.method("yaw", &node3dYaw, 1);
                builder.method("pitch", &node3dPitch, 1);
                builder.method("roll", &node3dRoll, 1);
                builder.method("move_forward", &node3dMoveForward, 1);
                builder.method("move_right", &node3dMoveRight, 1);
                builder.method("move_up", &node3dMoveUp, 1);
                builder.method("look_at", &node3dLookAt, -1);
                builder.method("look_at_from_position", &node3dLookAtFromPosition, -1);
                builder.method("to_local", &node3dToLocal, 1);
                builder.method("to_global", &node3dToGlobal, 1);
                builder.method("forward", &node3dForward, 0);
                builder.method("right", &node3dRight, 0);
                builder.method("up", &node3dUp, 0);
                builder.method("show", &node3dShow, 0);
                builder.method("hide", &node3dHide, 0);
                builder.method("is_visible_in_tree", &node3dIsVisibleInTree, 0);
                builder.method("count_collisions", &node3dCountCollisions, 0);
                builder.method("get_collision_node", &node3dGetCollisionNode, 1);
                builder.method("get_collision_point", &node3dGetCollisionPoint, 1);
                builder.method("get_collision_normal", &node3dGetCollisionNormal, 1);
                bindNode3DMethods(builder);
            }
            else if (names[i] == "PhysicsBody3D")
                bindPhysicsBodyMethods(builder);
            else if (names[i] == "CharacterBody3D")
                bindCharacterBodyMethods(builder);
            else if (names[i] == "CharacterRigidBody3D")
                bindCharacterRigidBodyMethods(builder);
            else if (names[i] == "CollisionShape3D")
                bindCollisionShapeMethods(builder);
            else if (names[i] == "RigidBody3D")
            {
                builder.method("apply_central_impulse", &rigidApplyCentralImpulse, 1);
                builder.method("apply_impulse", &rigidApplyImpulse, 2);
                builder.method("apply_torque_impulse", &rigidApplyTorqueImpulse, 1);
                builder.method("apply_central_force", &rigidApplyCentralForce, 1);
                builder.method("apply_force", &rigidApplyForce, 2);
                builder.method("apply_torque", &rigidApplyTorque, 1);
                builder.method("set_linear_velocity", &rigidSetLinearVelocity, 1);
                builder.method("get_linear_velocity", &rigidGetLinearVelocity, 0);
                builder.method("set_angular_velocity", &rigidSetAngularVelocity, 1);
                builder.method("get_angular_velocity", &rigidGetAngularVelocity, 0);
                builder.method("get_velocity_at_position", &rigidGetVelocityAtPosition, 1);
                bindRigidBodyMethods(builder);
            }
            else if (names[i] == "DestructibleBody3D")
                bindDestructibleBodyMethods(builder);
            else if (names[i] == "DebrisBody3D")
                bindDebrisBodyMethods(builder);
            else if (names[i] == "VehicleBody3D")
                bindVehicleBodyMethods(builder);
            else if (names[i] == "GroundVehicle3D")
                bindGroundVehicleMethods(builder);
            else if (names[i] == "CarDamage3D")
                bindCarDamageMethods(builder);
            else if (names[i] == "SkidMarks3D")
                bindSkidMarksMethods(builder);
            else if (names[i] == "ImpactFx3D")
                bindImpactFxMethods(builder);
            else if (names[i] == "VehicleDriver3D")
                bindVehicleDriverMethods(builder);
            else if (names[i] == "Traffic3D")
                bindTrafficMethods(builder);
            else if (names[i] == "VehicleWheel3D")
                bindVehicleWheelMethods(builder);
            else if (names[i] == "Joint3D")
                bindJointMethods(builder);
            else if (names[i] == "PinJoint3D")
                bindPinJointMethods(builder);
            else if (names[i] == "HingeJoint3D")
                bindHingeJointMethods(builder);
            else if (names[i] == "SliderJoint3D")
                bindSliderJointMethods(builder);
            else if (names[i] == "DistanceJoint3D")
                bindDistanceJointMethods(builder);
            else if (names[i] == "PistonJoint3D")
                bindPistonJointMethods(builder);
            else if (names[i] == "UniversalJoint3D")
                bindUniversalJointMethods(builder);
            else if (names[i] == "WheelJoint3D")
                bindWheelJointMethods(builder);
            else if (names[i] == "MouseJoint3D")
                bindMouseJointMethods(builder);
            else if (names[i] == "SoftBody3D")
                bindSoftBodyMethods(builder);
            else if (names[i] == "ParticleSystem3D")
                bindParticleSystemMethods(builder);
            else if (names[i] == "RibbonTrail3D")
                bindRibbonTrailMethods(builder);
            else if (names[i] == "ParticleCollision3D")
                builder.method("collide", &particleCollisionCollide, 2);
            else if (names[i] == "Terrain3D")
                bindTerrainMethods(builder);
            else if (names[i] == "Grass3D")
                bindGrassMethods(builder);
            else if (names[i] == "Forest3D")
                bindForestMethods(builder);
            else if (names[i] == "Tree3D")
                bindTreeMethods(builder);
            else if (names[i] == "Road3D")
                bindRoadMethods(builder);
            else if (names[i] == "Water3D")
                bindWaterMethods(builder);
            else if (names[i] == "Ocean3D")
                bindOceanMethods(builder);
            else if (names[i] == "Skeleton3D")
                bindSkeletonMethods(builder);
            else if (names[i] == "BoneAttachment3D")
                bindBoneAttachmentMethods(builder);
            else if (names[i] == "SkeletonModifier3D")
                bindSkeletonModifierMethods(builder);
            else if (names[i] == "SpringBoneSimulator3D")
                bindSpringBoneSimulatorMethods(builder);
            else if (names[i] == "SpringBoneCollision3D")
                bindSpringBoneCollisionMethods(builder);
            else if (names[i] == "SpringBoneCollisionCapsule3D")
                bindSpringBoneCollisionCapsuleMethods(builder);
            else if (names[i] == "Ragdoll3D")
                bindRagdollMethods(builder);
            else if (names[i] == "Dismember3D")
                bindDismemberMethods(builder);
            else if (names[i] == "NavigationRegion3D")
                bindNavigationRegionMethods(builder);
            else if (names[i] == "Path3D")
                bindPathMethods(builder);
            else if (names[i] == "PathFollow3D")
                bindPathFollowMethods(builder);
            else if (names[i] == "VisualInstance3D")
                bindVisualInstanceMethods(builder);
            else if (names[i] == "MeshInstance3D")
                bindMeshInstanceMethods(builder);
            else if (names[i] == "MultiMeshInstance3D")
                bindMultiMeshInstanceMethods(builder);
            else if (names[i] == "ManualMesh3D")
                bindManualMeshMethods(builder);
            else if (names[i] == "Hair3D")
                bindHairMethods(builder);
            else if (names[i] == "Light3D")
                bindLightMethods(builder);
            else if (names[i] == "Decal3D")
                bindDecalMethods(builder);
            else if (names[i] == "ReflectionProbe3D")
                bindReflectionProbeMethods(builder);
            else if (names[i] == "LightmapGI")
                bindLightmapMethods(builder);
            else if (names[i] == "Viewport")
                bindViewportMethods(builder);
            else if (names[i] == "AudioListener3D")
                bindAudioListenerMethods(builder);
            else if (names[i] == "Camera3D")
            {
                builder.method("make_current", &cameraMakeCurrent, 0);
                builder.method("is_current", &cameraIsCurrent, 0);
                builder.method("project_ray_origin", &cameraProjectRayOrigin, -1);
                builder.method("project_ray_normal", &cameraProjectRayNormal, -1);
                bindCameraMethods(builder);
            }
            else if (names[i] == "Button")
            {
                builder.method("press", &buttonPress, 0);
                builder.method("is_hovered", &buttonIsHovered, 0);
            }
            else if (names[i] == "LineEdit")
            {
                builder.method("grab_focus", &lineEditGrabFocus, 0);
                builder.method("release_focus", &lineEditReleaseFocus, 0);
                builder.method("has_focus", &lineEditHasFocus, 0);
                builder.method("clear", &lineEditClear, 0);
                builder.method("submit", &lineEditSubmit, 0);
            }
            else if (names[i] == "AnimationPlayer")
            {
                builder.method("play", &animationPlay, -1);
                builder.method("stop", &animationStop, -1);
                builder.method("is_playing", &animationIsPlaying, 0);
                builder.method("import_animations", &animationImportAnimations, -1);
                bindAnimationPlayerMethods(builder);
            }
            else if (names[i] == "AudioStreamPlayer")
            {
                builder.method("play", &audioPlay<AudioStreamPlayer>, 0);
                builder.method("stop", &audioStop<AudioStreamPlayer>, 0);
                builder.method("is_playing", &audioIsPlaying<AudioStreamPlayer>, 0);
                builder.method("get_playback_position", &audioGetPlaybackPosition<AudioStreamPlayer>, 0);
                builder.method("seek", &audioSeek<AudioStreamPlayer>, 1);
                bindAudioPlayerMethods<AudioStreamPlayer>(builder);
            }
            else if (names[i] == "AudioStreamPlayer3D")
            {
                builder.method("play", &audioPlay<AudioStreamPlayer3D>, 0);
                builder.method("stop", &audioStop<AudioStreamPlayer3D>, 0);
                builder.method("is_playing", &audioIsPlaying<AudioStreamPlayer3D>, 0);
                builder.method("get_playback_position", &audioGetPlaybackPosition<AudioStreamPlayer3D>, 0);
                builder.method("seek", &audioSeek<AudioStreamPlayer3D>, 1);
                bindAudioPlayerMethods<AudioStreamPlayer3D>(builder);
            }

            zen::ObjClass* klass = builder.end();
            classes.put(names[i], klass);
            if (names[i] == "Node") nodeClass = klass;
            done[i] = 1;
            progress = true;
        }
    }
}

void ZenScripting::Impl::registerGlobals()
{
    {
        zen::VM::ClassBuilder builder = vm.def_class("SceneTree");
        builder.constructable(false);
        builder.method("get_nodes_in_group", &treeGetNodesInGroup, 1);
        builder.method("get_first_node_in_group", &treeGetFirstNodeInGroup, 1);
        builder.method("has_group", &treeHasGroup, 1);
        builder.method("get_node_count_in_group", &treeGetNodeCountInGroup, 1);
        builder.method("change_scene", &treeChangeScene, 1);
        builder.method("change_scene_to_file", &treeChangeScene, 1);
        builder.method("reload_current_scene", &treeReloadCurrentScene, 0);
        builder.method("quit", &treeQuit, -1);
        builder.method("set_paused", &treeSetPaused, 1);
        builder.method("is_paused", &treeIsPaused, 0);
        builder.method("set_auto_accept_quit", &treeSetAutoAcceptQuit, 1);
        builder.method("is_auto_accept_quit", &treeIsAutoAcceptQuit, 0);
        builder.method("get_root", &treeGetRoot, 0);
        builder.method("get_current_scene", &treeGetCurrentScene, 0);
        builder.method("get_frame", &treeGetFrame, 0);
        builder.method("get_node_count", &treeGetNodeCount, 0);
        builder.method("set_input_as_handled", &treeSetInputAsHandled, 0);
        builder.method("get_process_delta_time", &treeGetProcessDelta, 0);
        builder.method("get_physics_process_delta_time", &treeGetPhysicsDelta, 0);
        sceneTreeClass = builder.end();
    }

    {
        zen::VM::ClassBuilder builder = vm.def_class("InputEvent");
        builder.constructable(false);
        for (int i = 0; i < kFieldCount; ++i) builder.field(kEventFields[i]);
        builder.method("is_action", &eventIsAction, 1);
        builder.method("is_action_pressed", &eventIsActionPressed, 1);
        builder.method("is_action_released", &eventIsActionReleased, 1);
        builder.method("get_action_strength", &eventGetActionStrength, 1);
        builder.dtor(&eventDestroy);
        inputEventClass = builder.end();
    }

    vm.def_native("keyboard_check", &keyboardCheck, 1);
    vm.def_native("keyboard_check_pressed", &keyboardCheckPressed, 1);
    vm.def_native("keyboard_check_released", &keyboardCheckReleased, 1);
    vm.def_native("mouse_check_button", &mouseCheckButton, 1);
    vm.def_native("mouse_check_button_pressed", &mouseCheckButtonPressed, 1);
    vm.def_native("mouse_check_button_released", &mouseCheckButtonReleased, 1);
    vm.def_native("user_dir", &userDir, 0);
    vm.def_native("window_set_fullscreen", &windowSetFullscreen, 1);
    vm.def_native("window_is_fullscreen", &windowIsFullscreen, 0);
    vm.def_native("mouse_x", &mouseX, 0);
    vm.def_native("mouse_y", &mouseY, 0);
    vm.def_native("mouse_position", &inputGetMousePosition, 0);
    vm.def_native("mouse_delta", &mouseDelta, 0);
    vm.def_native("mouse_wheel", &mouseWheel, 0);
    vm.def_native("mouse_velocity", &inputGetMouseVelocity, 0);
    vm.def_native("mouse_set_mode", &inputSetMouseMode, 1);
    vm.def_native("mouse_get_mode", &inputGetMouseMode, 0);
    vm.def_native("action_check", &inputIsActionPressed, 1);
    vm.def_native("action_check_pressed", &inputIsActionJustPressed, 1);
    vm.def_native("action_check_released", &inputIsActionJustReleased, 1);
    vm.def_native("action_strength", &inputGetActionStrength, 1);
    vm.def_native("action_axis", &inputGetAxis, 2);
    vm.def_native("action_vector", &inputGetVector, -1);
    vm.def_native("action_press", &inputActionPress, -1);
    vm.def_native("action_release", &inputActionRelease, 1);
    vm.def_native("action_exists", &mapHasAction, 1);
    vm.def_native("action_add", &mapAddAction, -1);
    vm.def_native("action_remove", &mapEraseAction, 1);
    vm.def_native("action_add_key", &mapActionAddKey, 2);
    vm.def_native("action_add_mouse_button", &mapActionAddMouseButton, 2);
    vm.def_native("action_clear", &mapActionEraseEvents, 1);
    vm.def_native("delta_time", &timeGetDelta, 0);
    vm.def_native("physics_delta_time", &timeGetPhysicsDelta, 0);
    vm.def_native("current_time", &timeGetTicksMsec, 0);
    vm.def_native("current_time_usec", &timeGetTicksUsec, 0);
    vm.def_native("frame_count", &timeGetFrames, 0);
    vm.def_native("fps", &engineGetFps, 0);
    vm.def_native("time_scale", &engineGetTimeScale, 0);
    vm.def_native("set_time_scale", &engineSetTimeScale, 1);
    vm.def_native("physics_interpolation_fraction", &engineGetPhysicsFraction, 0);
    vm.def_native("set_bus_pitch", &audioSetBusPitch, 2);
    vm.def_native("bus_pitch", &audioGetBusPitch, 1);
    vm.def_native("is_editor", &engineIsEditorHint, 0);
    vm.def_native("hud_text", &hudDrawText, -1);
    vm.def_native("hud_rect", &hudDrawRect, -1);
    vm.def_native("hud_rect_outline", &hudDrawRectOutline, -1);
    vm.def_native("hud_line", &hudDrawLine, -1);
    vm.def_native("hud_circle", &hudDrawCircle, -1);
    vm.def_native("hud_image", &hudDrawImage, -1);
    vm.def_native("hud_load_image", &hudLoadImage, 1);
    vm.def_native("hud_text_size", &hudGetTextSize, -1);
    vm.def_native("hud_width", &hudGetWidth, 0);
    vm.def_native("hud_height", &hudGetHeight, 0);

    {
        zen::VM::ClassBuilder builder = vm.def_class("RayHit");
        builder.constructable(false);
        for (int i = 0; i < kHitFieldCount; ++i) builder.field(kRayHitFields[i]);
        rayHitClass = builder.end();
    }

    {
        zen::VM::ClassBuilder builder = vm.def_class("Aabb");
        builder.constructable(false);
        builder.field("min");
        builder.field("max");
        sAabbClass = builder.end();
    }

    {
        zen::VM::ClassBuilder builder = vm.def_class("Ray");
        builder.constructable(false);
        builder.field("origin");
        builder.field("direction");
        sRayClass = builder.end();
    }

    vm.def_native("raycast", &physicsRaycast, -1);
    vm.def_native("overlap_sphere", &physicsOverlapSphere, -1);
    vm.def_native("overlap_box", &physicsOverlapBox, -1);

    vm.def_native("is_valid", &globalIsValid, 1);
    vm.def_native("move_toward", &globalMoveToward, 3);
    vm.def_native("quat_slerp", &globalQuatSlerp, 3);
    vm.def_native("quat_lerp", &globalQuatLerp, 3);
    vm.def_native("quat_normalized", &globalQuatNormalized, 1);
    vm.def_native("quat_length", &globalQuatLength, 1);
    vm.def_native("quat_dot", &globalQuatDot, 2);

    for (const KeyName& key: kNamedKeys) vm.def_global(key.name, zen::val_int(key.value));
    for (const KeyName& constant: kEnumConstants)
        vm.def_global(constant.name, zen::val_int(constant.value));
    for (int c = 'A'; c <= 'Z'; ++c)
    {
        char name[16];
        snprintf(name, sizeof(name), "KEY_%c", c);
        vm.def_global(name, zen::val_int(c));
    }
    for (int c = '0'; c <= '9'; ++c)
    {
        char name[16];
        snprintf(name, sizeof(name), "KEY_%c", c);
        vm.def_global(name, zen::val_int(c));
    }
    for (int f = 1; f <= 12; ++f)
    {
        char name[16];
        snprintf(name, sizeof(name), "KEY_F%d", f);
        vm.def_global(name, zen::val_int(289 + f));
    }
}

zen::Value ZenScripting::Impl::makeInputEvent(const InputEvent& event)
{
    zen::GC* gc = &vm.get_gc();
    zen::gc_pause(gc);
    zen::Value value = vm.make_instance(inputEventClass);
    zen::ObjInstance* instance = zen::as_instance(value);

    EventData* data = new EventData();
    data->event = event;
    if (event.type == InputEventType::Action && event.action.name)
    {
        data->action = event.action.name;
        data->event.action.name = data->action.c_str();
    }
    instance->native_data = data;

    zen::Value* f = instance->fields;
    for (int i = 0; i < kFieldCount; ++i) f[i] = zen::val_nil();
    f[kFieldType] = zen::val_int(static_cast<int>(event.type));
    f[kFieldPressed] = zen::val_bool(false);
    f[kFieldEcho] = zen::val_bool(false);
    f[kFieldKey] = zen::val_int(0);
    f[kFieldScancode] = zen::val_int(0);
    f[kFieldModifiers] = zen::val_int(0);
    f[kFieldPosition] = zen::val_vec2(0.0f, 0.0f);
    f[kFieldRelative] = zen::val_vec2(0.0f, 0.0f);
    f[kFieldButton] = zen::val_int(0);
    f[kFieldButtonMask] = zen::val_int(0);
    f[kFieldWheel] = zen::val_vec2(0.0f, 0.0f);
    f[kFieldCodepoint] = zen::val_int(0);
    f[kFieldTouchId] = zen::val_int(0);
    f[kFieldPressure] = zen::val_float(0.0);
    f[kFieldAction] = makeString(&vm, "");
    f[kFieldStrength] = zen::val_float(0.0);

    switch (event.type)
    {
        case InputEventType::Key:
            f[kFieldPressed] = zen::val_bool(event.key.pressed);
            f[kFieldEcho] = zen::val_bool(event.key.echo);
            f[kFieldKey] = zen::val_int(static_cast<int>(event.key.key));
            f[kFieldScancode] = zen::val_int(event.key.scancode);
            f[kFieldModifiers] = zen::val_int(static_cast<long long>(event.key.modifiers));
            break;
        case InputEventType::Char:
            f[kFieldCodepoint] = zen::val_int(static_cast<long long>(event.character.codepoint));
            break;
        case InputEventType::MouseMotion:
            f[kFieldPosition] = zen::val_vec2(static_cast<float>(event.mouseMotion.x),
                    static_cast<float>(event.mouseMotion.y));
            f[kFieldRelative] = zen::val_vec2(static_cast<float>(event.mouseMotion.relativeX),
                    static_cast<float>(event.mouseMotion.relativeY));
            f[kFieldButtonMask] =
                    zen::val_int(static_cast<long long>(event.mouseMotion.buttonMask));
            break;
        case InputEventType::MouseButton:
            f[kFieldPressed] = zen::val_bool(event.mouseButton.pressed);
            f[kFieldPosition] = zen::val_vec2(static_cast<float>(event.mouseButton.x),
                    static_cast<float>(event.mouseButton.y));
            f[kFieldButton] = zen::val_int(static_cast<int>(event.mouseButton.button));
            f[kFieldModifiers] = zen::val_int(static_cast<long long>(event.mouseButton.modifiers));
            break;
        case InputEventType::MouseWheel:
            f[kFieldPosition] = zen::val_vec2(static_cast<float>(event.mouseWheel.x),
                    static_cast<float>(event.mouseWheel.y));
            f[kFieldWheel] = zen::val_vec2(event.mouseWheel.deltaX, event.mouseWheel.deltaY);
            break;
        case InputEventType::Touch:
            f[kFieldPressed] = zen::val_bool(
                    event.touch.phase == TouchPhase::Down || event.touch.phase == TouchPhase::Move);
            f[kFieldPosition] = zen::val_vec2(event.touch.x, event.touch.y);
            f[kFieldTouchId] = zen::val_int(event.touch.id);
            f[kFieldPressure] = zen::val_float(event.touch.pressure);
            break;
        case InputEventType::Action:
            f[kFieldPressed] = zen::val_bool(event.action.pressed);
            f[kFieldAction] = makeString(&vm, data->action);
            f[kFieldStrength] = zen::val_float(event.action.strength);
            break;
        case InputEventType::None:
            break;
    }
    zen::gc_resume(gc);
    return value;
}

zen::Value ZenScripting::Impl::sceneTreeValue(SceneTree* tree)
{
    if (!tree) tree = SceneTree::singleton();
    if (!tree) return zen::val_nil();
    if (tree != treePointer || !treeRef)
    {
        treeRef = zen::Ref(vm, vm.make_instance_borrowed(sceneTreeClass, tree));
        treePointer = tree;
    }
    return treeRef.get();
}

void ZenScripting::Impl::registerTweenClasses()
{
    {
        zen::VM::ClassBuilder builder = vm.def_class("Tween");
        builder.constructable(false);
        builder.method("tween_property", &tweenTweenProperty, 4);
        builder.method("tween_interval", &tweenTweenInterval, 1);
        builder.method("tween_callback", &tweenTweenCallback, 1);
        builder.method("set_trans", &tweenSetTrans, 1);
        builder.method("set_ease", &tweenSetEase, 1);
        builder.method("set_parallel", &tweenSetParallel, -1);
        builder.method("parallel", &tweenParallel, 0);
        builder.method("chain", &tweenChain, 0);
        builder.method("set_loops", &tweenSetLoops, -1);
        builder.method("set_speed_scale", &tweenSetSpeedScale, 1);
        builder.method("bind_node", &tweenBindNode, 1);
        builder.method("stop", &tweenStop, 0);
        builder.method("pause", &tweenPause, 0);
        builder.method("play", &tweenPlay, 0);
        builder.method("kill", &tweenKill, 0);
        builder.method("is_running", &tweenIsRunning, 0);
        builder.method("is_valid", &tweenIsValid, 0);
        tweenClass = builder.end();
    }
    {
        zen::VM::ClassBuilder builder = vm.def_class("PropertyTweener");
        builder.constructable(false);
        builder.method("set_trans", &tweenerSetTrans, 1);
        builder.method("set_ease", &tweenerSetEase, 1);
        builder.method("set_delay", &tweenerSetDelay, 1);
        builder.method("from_value", &tweenerFrom, 1);
        builder.method("from_current", &tweenerFromCurrent, 0);
        builder.method("as_relative", &tweenerAsRelative, 0);
        tweenerClass = builder.end();
    }
    vm.def_native("create_tween", &createTween, 0);
}

} // namespace prisma3d
