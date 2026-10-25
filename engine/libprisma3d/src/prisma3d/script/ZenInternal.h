#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"
#include "prisma3d/scene/ScriptInstance.h"
#include "prisma3d/scene/Tween.h"
#include "prisma3d/script/ZenScripting.h"

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <zen/memory.h>
#include <zen/object.h>
#include <zen/value.h>
#include <zen/vm.h>

namespace prisma3d
{

class SceneTree;

enum ZenHook : int
{
    kHookEnterTree = 0,
    kHookReady,
    kHookProcess,
    kHookPhysicsProcess,
    kHookInput,
    kHookExitTree,
    kHookBodyEntered,
    kHookBodyExited,
    kHookButtonPressed,
    kHookTextSubmitted,
    kHookTextChanged,
    kHookCloseRequested,
    kHookCount
};

struct ZenHookInfo
{
    const char* primary;
    const char* alias;
    int arity;
};

extern const ZenHookInfo kZenHooks[kHookCount];

struct ZenExport
{
    ct::String name;
    PropertyType type = PropertyType::Nil;
    PropertyValue defaultValue;
};

struct ZenEntry
{
    ct::String path;
    ct::String resolved;
    ct::String baseClass;
    long long mtime = 0;
    long long size = 0;
    long long failedMtime = -1;
    long long failedSize = -1;
    bool tool = false;
    bool noNodeClass = false;
    zen::ObjClass* klass = nullptr;
    zen::Ref classRef;
    zen::Ref function;
    int hookSlot[kHookCount];
    int initSlot = -1;
    ct::Vector<ZenExport> exports;
    int version = 0;

    ZenEntry()
    {
        for (int i = 0; i < kHookCount; ++i) hookSlot[i] = -1;
    }
};

class ZenNodeInstance : public ScriptInstance
{
public:
    ZenNodeInstance(ZenScripting::Impl& impl, Node* node);
    ~ZenNodeInstance() override;

    bool hasScript() const override;
    const ct::String& getPath() const override;
    void notification(int what) override;
    void input(const InputEvent& event) override;
    void bodyEntered(Node* other) override;
    void bodyExited(Node* other) override;
    void buttonPressed(Node* button) override;
    void textSubmitted(Node* field, const ct::String& text) override;
    void textChanged(Node* field, const ct::String& text) override;
    bool wantsBodyContacts() const override;
    void getPropertyList(ct::Vector<PropertyInfo>& list) const override;
    bool setProperty(const ct::String& name, const PropertyValue& value) override;
    bool getProperty(const ct::String& name, PropertyValue& value) const override;
    bool getDefaultProperty(const ct::String& name, PropertyValue& value) const override;

    zen::Value value() const;
    void bind(ZenEntry* newEntry, bool preserve);
    void unbind();
    bool canRun() const;
    bool ensureInit();
    void applyProcessFlags();
    bool call(int hook, zen::Value* args, int nargs);

    ZenScripting::Impl& impl;
    Node* node;
    ZenEntry* entry;
    zen::Ref ref;
    bool initDone;
    bool faulted;
};

struct ZenScripting::Impl
{
    explicit Impl(ZenScripting& scripting);
    ~Impl();

    ZenEntry* loadEntry(const ct::String& path);
    bool compileEntry(ZenEntry* entry, bool reload);
    int reloadChanged();
    void releaseAll();

    zen::Value wrap(Node* node);
    ZenNodeInstance* instanceFor(Node* node);
    zen::ObjClass* classFor(const char* className);
    Node* nodeOf(zen::Value value) const;
    bool isNodeValue(zen::Value value) const;

    bool invokeGuarded(ZenNodeInstance* instance, int slot, zen::Value* args, int nargs);
    void clearVmError();
    void clearFrameRegisters(zen::Value value, int slot, int nargs);
    void emitHostText(const char* text, std::size_t length, bool isError);
    void reportError(const ZenNodeInstance* instance, const ct::String& text);
    void reportMessage(const char* format, ...);
    bool rebindInstance(zen::Value instance, zen::ObjClass* target, bool preserve);

    void registerNodeClasses();
    void registerGlobals();
    void registerValueTypes();

    zen::Value toZen(const PropertyValue& value);
    bool fromZen(zen::Value value, PropertyType type, PropertyValue& out, ct::String& error);
    zen::Value makeInputEvent(const InputEvent& event);
    zen::Value sceneTreeValue(SceneTree* tree);

    int propertySlot(const char* name);
    const ct::String& propertyName(int slot) const;

    ZenScripting& scripting;
    zen::VM vm;
    ct::HashMap<ct::String, zen::ObjClass*> classes;
    ct::HashMap<ct::String, ZenEntry*> entryIndex;
    ct::Vector<ZenEntry*> entries;
    ct::Vector<ZenNodeInstance*> instances;
    ct::Vector<ct::String> propertyNames;
    ct::HashMap<ct::String, int> propertyIndex;

    ct::String errorBuffer;
    ct::String printBuffer;
    ct::String hostErrorBuffer;
    bool compiling;
    ct::String lastError;
    int errorCount;
    int compileCount;
    bool propertyOverflowLogged;

    int hookSelector[kHookCount][2];
    int initSelector;
    zen::Value noop;

    zen::ObjClass* nodeClass;
    zen::ObjClass* sceneTreeClass;
    zen::ObjClass* inputEventClass;
    zen::ObjClass* rayHitClass;
    zen::ObjClass* tweenClass;
    zen::ObjClass* tweenerClass;
    struct TweenHold
    {
        Tween* tween;
        zen::Ref ref;
    };
    struct TweenerHandle
    {
        Tween* tween;
        PropertyTweener* tweener;
    };
    ct::Vector<TweenHold> tweenHolds;
    ct::Vector<TweenerHandle*> tweenerHandles;
    void pruneTweens();
    void registerTweenClasses();
    zen::NativeStructDef* vec4Def;
    zen::NativeStructDef* quatDef;
    zen::NativeStructDef* colorDef;
    zen::Ref treeRef;
    SceneTree* treePointer;

    ZenScripting::PrintHandler printHandler;
    void* printUser;

    bool hotReload;
    double reloadInterval;
    double reloadElapsed;
    unsigned long long bindingsSignature;
    ct::Vector<ct::String> vmSearchPaths;
    bool probing = false;
};

} // namespace prisma3d
