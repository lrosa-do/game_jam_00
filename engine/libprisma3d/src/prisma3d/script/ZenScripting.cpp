#include "ZenInternal.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneTree.h"

#include "platform.h"

#include <ct/json.hpp>

#include <zen/bytecode.h>
#include <zen/compiler.h>
#include <zen/module.h>
#include <zen/zen_host_output.h>
#include <zen/zen_script_info.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace prisma3d
{

const ZenHookInfo kZenHooks[kHookCount] = {
    { "_enter_tree", "enter_tree", 0 },
    { "_ready", "ready", 0 },
    { "_process", "process", 1 },
    { "_physics_process", "physics_process", 1 },
    { "_input", "input", 1 },
    { "_exit_tree", "exit_tree", 0 },
    { "_body_entered", "body_entered", 1 },
    { "_body_exited", "body_exited", 1 },
    { "_button_pressed", "button_pressed", 1 },
    { "_text_submitted", "text_submitted", 2 },
    { "_text_changed", "text_changed", 2 },
    { "_close_requested", "close_requested", 0 },
};

namespace
{

ZenScripting* sInstance = nullptr;

void captureError(const char* text, int length, void* user)
{
    ZenScripting::Impl* impl = static_cast<ZenScripting::Impl*>(user);
    impl->errorBuffer.append(text, static_cast<std::size_t>(length));
}

void hostWrite(const char* text, size_t length, int isError, void* user)
{
    static_cast<ZenScripting::Impl*>(user)->emitHostText(text, length, isError != 0);
}

int noopNative(zen::VM*, zen::Value*, int)
{
    return 0;
}

struct ZenFileHandle
{
    IoStream* stream;
    FILE* file;
};

zen::ZenFile fileOpen(const char* path, const char* mode, void*)
{
    FileSystem& files = FileSystem::getSingleton();
    ZenFileHandle* handle = new ZenFileHandle{ nullptr, nullptr };
    if (strchr(mode, 'w') || strchr(mode, 'a') || strchr(mode, '+'))
    {
        const ct::String target = files.globalize(ct::String(path));
        if (!target.empty()) handle->file = fopen(target.c_str(), mode);
    }
    else
    {
        const ct::String name(path);
        if (files.exists(name)) handle->stream = files.open(name);
    }
    if (!handle->stream && !handle->file)
    {
        delete handle;
        return nullptr;
    }
    return handle;
}

long fileRead(zen::ZenFile file, void* buffer, long size, void*)
{
    ZenFileHandle* handle = static_cast<ZenFileHandle*>(file);
    if (size <= 0) return 0;
    if (handle->stream)
        return static_cast<long>(io_read(handle->stream, buffer, static_cast<size_t>(size)));
    return static_cast<long>(fread(buffer, 1, static_cast<size_t>(size), handle->file));
}

long fileWrite(zen::ZenFile file, const void* buffer, long size, void*)
{
    ZenFileHandle* handle = static_cast<ZenFileHandle*>(file);
    if (size <= 0 || !handle->file) return 0;
    return static_cast<long>(fwrite(buffer, 1, static_cast<size_t>(size), handle->file));
}

long fileSeek(zen::ZenFile file, long offset, int whence, void*)
{
    ZenFileHandle* handle = static_cast<ZenFileHandle*>(file);
    if (handle->stream)
    {
        const IoWhence origin = whence == 1 ? IO_SEEK_CUR : whence == 2 ? IO_SEEK_END : IO_SEEK_SET;
        if (io_seek(handle->stream, offset, origin) < 0) return -1;
        return static_cast<long>(io_tell(handle->stream));
    }
    if (fseek(handle->file, offset,
                whence == 1   ? SEEK_CUR
                : whence == 2 ? SEEK_END
                              : SEEK_SET) != 0)
        return -1;
    return ftell(handle->file);
}

int fileClose(zen::ZenFile file, void*)
{
    ZenFileHandle* handle = static_cast<ZenFileHandle*>(file);
    int status = 0;
    if (handle->stream) io_close(handle->stream);
    if (handle->file) status = fclose(handle->file);
    delete handle;
    return status;
}

int fileExists(const char* path, void*)
{
    FileSystem& files = FileSystem::getSingleton();
    const ct::String name(path);
    if (files.exists(name)) return 1;
    const ct::String target = files.globalize(name);
    return !target.empty() && file_exists(target.c_str()) ? 1 : 0;
}

bool editorHint()
{
    const Engine* engine = Engine::singleton();
    return engine && engine->isEditorHint();
}

bool arityAccepts(const zen::Value method, int expected)
{
    if (!zen::is_closure(method)) return false;
    const zen::ObjFunc* function = zen::as_closure(method)->func;
    if (function->arity < 0) return true;
    return expected <= function->arity && expected >= function->arity - function->default_count;
}

zen::ObjClosure* makeClosure(zen::VM& vm, zen::ObjFunc* function)
{
    zen::GC* gc = &vm.get_gc();
    zen::gc_pause(gc);
    zen::ObjClosure* closure =
            static_cast<zen::ObjClosure*>(zen::zen_alloc(gc, sizeof(zen::ObjClosure)));
    zen::gc_resume(gc);
    closure->obj.type = zen::OBJ_CLOSURE;
    closure->obj.color = gc->black_val;
    closure->obj.hash = 0;
    closure->obj.gc_next = gc->objects;
    gc->objects = reinterpret_cast<zen::Obj*>(closure);
    closure->func = function;
    closure->upvalues = nullptr;
    closure->upvalue_count = 0;
    return closure;
}

ct::String stemOf(const ct::String& path)
{
    std::size_t start = 0;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') start = i + 1;
    std::size_t end = path.size();
    for (std::size_t i = start; i < path.size(); ++i)
        if (path[i] == '.') end = i;
    return path.substr(start, end - start);
}

ct::String describeVmError(const ct::String& text)
{
    static const char kPrefix[] = "[zen runtime error] ";
    ct::String body = text;
    if (body.size() >= sizeof(kPrefix) - 1 &&
            memcmp(body.c_str(), kPrefix, sizeof(kPrefix) - 1) == 0)
        body = body.substr(sizeof(kPrefix) - 1);

    const std::size_t newline = body.find('\n');
    ct::String message = newline == ct::String::npos ? body : body.substr(0, newline);
    ct::String trace = newline == ct::String::npos ? ct::String() : body.substr(newline + 1);

    const std::size_t fileAt = trace.find("File \"");
    if (fileAt == ct::String::npos) return body;
    const std::size_t nameStart = fileAt + 6;
    const std::size_t nameEnd = trace.find('"', nameStart);
    if (nameEnd == ct::String::npos) return body;
    const ct::String file = trace.substr(nameStart, nameEnd - nameStart);
    const std::size_t lineAt = trace.find("line ", nameEnd);
    if (lineAt == ct::String::npos) return body;
    std::size_t lineEnd = lineAt + 5;
    while (lineEnd < trace.size() && trace[lineEnd] >= '0' && trace[lineEnd] <= '9') ++lineEnd;
    const ct::String line = trace.substr(lineAt + 5, lineEnd - lineAt - 5);

    ct::String result = file + ":" + line + ": " + message;
    if (!trace.empty())
    {
        result += "\n";
        result += trace;
    }
    return result;
}

void fieldLookup(const zen::ObjInstance* instance, const char* name, int& index)
{
    index = -1;
    const zen::ObjClass* klass = instance->klass;
    for (int i = 0; i < klass->num_fields && i < instance->num_fields; ++i)
    {
        if (klass->field_names[i] && strcmp(klass->field_names[i]->chars, name) == 0)
        {
            index = i;
            return;
        }
    }
}

} // namespace

ZenNodeInstance::ZenNodeInstance(ZenScripting::Impl& owner, Node* owningNode)
        : impl(owner),
          node(owningNode),
          entry(nullptr),
          initDone(false),
          faulted(false)
{
    zen::ObjClass* klass = impl.classFor(node->getClassName());
    ref = zen::Ref(impl.vm, impl.vm.make_instance_borrowed(klass, node));
    impl.instances.push_back(this);
}

ZenNodeInstance::~ZenNodeInstance()
{
    for (std::size_t i = 0; i < impl.instances.size(); ++i)
    {
        if (impl.instances[i] == this)
        {
            impl.instances[i] = impl.instances.back();
            impl.instances.pop_back();
            break;
        }
    }
    zen::Value held = ref.get();
    if (zen::is_instance(held)) zen::as_instance(held)->native_data = nullptr;
}

bool ZenNodeInstance::hasScript() const
{
    return entry != nullptr;
}

const ct::String& ZenNodeInstance::getPath() const
{
    static const ct::String empty;
    return entry ? entry->path : empty;
}

zen::Value ZenNodeInstance::value() const
{
    return ref.get();
}

bool ZenNodeInstance::canRun() const
{
    return entry && (!editorHint() || entry->tool);
}

void ZenNodeInstance::bind(ZenEntry* newEntry, bool preserve)
{
    entry = newEntry;
    faulted = false;
    initDone = false;
    impl.rebindInstance(ref.get(), newEntry->klass, preserve);
}

void ZenNodeInstance::unbind()
{
    entry = nullptr;
    faulted = false;
    initDone = false;
    impl.rebindInstance(ref.get(), impl.classFor(node->getClassName()), false);
}

bool ZenNodeInstance::ensureInit()
{
    if (!entry || faulted) return false;
    if (initDone) return true;
    initDone = true;
    if (entry->initSlot < 0) return true;
    return impl.invokeGuarded(this, entry->initSlot, nullptr, 0);
}

void ZenNodeInstance::applyProcessFlags()
{
    if (!entry) return;
    if (entry->hookSlot[kHookProcess] >= 0) node->setProcess(true);
    if (entry->hookSlot[kHookPhysicsProcess] >= 0) node->setPhysicsProcess(true);
    if (entry->hookSlot[kHookInput] >= 0) node->setProcessInput(true);
    if (wantsBodyContacts())
        if (PhysicsBody3D* body = node->castTo<PhysicsBody3D>())
            body->setContactMonitor(body->isContactMonitor(), body->getMaxContacts());
}

void ZenNodeInstance::bodyEntered(Node* other)
{
    if (!entry || entry->hookSlot[kHookBodyEntered] < 0) return;
    zen::Value args[1] = { other ? impl.wrap(other) : zen::val_nil() };
    call(kHookBodyEntered, args, 1);
}

void ZenNodeInstance::bodyExited(Node* other)
{
    if (!entry || entry->hookSlot[kHookBodyExited] < 0) return;
    zen::Value args[1] = { other ? impl.wrap(other) : zen::val_nil() };
    call(kHookBodyExited, args, 1);
}

void ZenNodeInstance::buttonPressed(Node* button)
{
    if (!entry || entry->hookSlot[kHookButtonPressed] < 0) return;
    zen::Value args[1] = { button ? impl.wrap(button) : zen::val_nil() };
    call(kHookButtonPressed, args, 1);
}

void ZenNodeInstance::textSubmitted(Node* field, const ct::String& text)
{
    if (!entry || entry->hookSlot[kHookTextSubmitted] < 0) return;
    zen::Value args[2] = { field ? impl.wrap(field) : zen::val_nil(),
        impl.toZen(PropertyValue::fromString(text)) };
    call(kHookTextSubmitted, args, 2);
}

void ZenNodeInstance::textChanged(Node* field, const ct::String& text)
{
    if (!entry || entry->hookSlot[kHookTextChanged] < 0) return;
    zen::Value args[2] = { field ? impl.wrap(field) : zen::val_nil(),
        impl.toZen(PropertyValue::fromString(text)) };
    call(kHookTextChanged, args, 2);
}

bool ZenNodeInstance::wantsBodyContacts() const
{
    return entry &&
           (entry->hookSlot[kHookBodyEntered] >= 0 || entry->hookSlot[kHookBodyExited] >= 0);
}

bool ZenNodeInstance::call(int hook, zen::Value* args, int nargs)
{
    if (!entry || faulted || !canRun() || !ensureInit()) return false;
    const int slot = entry->hookSlot[hook];
    if (slot < 0) return true;
    return impl.invokeGuarded(this, slot, args, nargs);
}

void ZenNodeInstance::notification(int what)
{
    if (!entry || faulted) return;
    if (what == Node::kNotificationReady) applyProcessFlags();
    if (!canRun()) return;
    switch (what)
    {
        case Node::kNotificationEnterTree:
            call(kHookEnterTree, nullptr, 0);
            break;
        case Node::kNotificationReady:
            call(kHookReady, nullptr, 0);
            break;
        case Node::kNotificationProcess:
        {
            zen::Value delta[1] = { zen::val_float(node->getProcessDeltaTime()) };
            call(kHookProcess, delta, 1);
            break;
        }
        case Node::kNotificationPhysicsProcess:
        {
            zen::Value delta[1] = { zen::val_float(node->getPhysicsProcessDeltaTime()) };
            call(kHookPhysicsProcess, delta, 1);
            break;
        }
        case Node::kNotificationExitTree:
            call(kHookExitTree, nullptr, 0);
            break;
        case Node::kNotificationWmCloseRequest:
            call(kHookCloseRequested, nullptr, 0);
            break;
        default:
            break;
    }
}

void ZenNodeInstance::input(const InputEvent& event)
{
    if (!entry || faulted || !canRun() || !ensureInit()) return;
    if (entry->hookSlot[kHookInput] < 0) return;
    zen::Value args[1] = { impl.makeInputEvent(event) };
    impl.invokeGuarded(this, entry->hookSlot[kHookInput], args, 1);
}

void ZenNodeInstance::getPropertyList(ct::Vector<PropertyInfo>& list) const
{
    if (!entry) return;
    for (std::size_t i = 0; i < entry->exports.size(); ++i)
        list.push_back(PropertyInfo(entry->exports[i].type, entry->exports[i].name.c_str()));
}

bool ZenNodeInstance::getDefaultProperty(const ct::String& name, PropertyValue& value) const
{
    if (!entry) return false;
    for (std::size_t i = 0; i < entry->exports.size(); ++i)
    {
        if (entry->exports[i].name == name)
        {
            value = entry->exports[i].defaultValue;
            return true;
        }
    }
    return false;
}

bool ZenNodeInstance::getProperty(const ct::String& name, PropertyValue& value) const
{
    if (!entry) return false;
    const ZenExport* exported = nullptr;
    for (std::size_t i = 0; i < entry->exports.size() && !exported; ++i)
        if (entry->exports[i].name == name) exported = &entry->exports[i];
    if (!exported) return false;

    const zen::Value held = ref.get();
    if (!zen::is_instance(held)) return false;
    const zen::ObjInstance* instance = zen::as_instance(held);
    int index = -1;
    fieldLookup(instance, name.c_str(), index);
    if (index < 0)
    {
        value = exported->defaultValue;
        return true;
    }

    const zen::Value field = instance->fields[index];
    PropertyValue actual;
    if (zen::is_bool(field)) actual = PropertyValue::fromBool(field.as.boolean);
    else if (zen::is_int(field))
        actual = PropertyValue::fromInt(field.as.integer);
    else if (zen::is_float(field))
        actual = PropertyValue::fromFloat(field.as.number);
    else if (zen::is_string(field))
        actual = PropertyValue::fromString(ct::String(zen::safe_string_chars(field),
                static_cast<std::size_t>(zen::safe_string_len(field))));
    else
    {
        value = exported->defaultValue;
        return true;
    }
    if (!actual.convertTo(exported->type, value)) value = exported->defaultValue;
    return true;
}

bool ZenNodeInstance::setProperty(const ct::String& name, const PropertyValue& value)
{
    if (!entry) return false;
    const ZenExport* exported = nullptr;
    for (std::size_t i = 0; i < entry->exports.size() && !exported; ++i)
        if (entry->exports[i].name == name) exported = &entry->exports[i];
    if (!exported) return false;

    PropertyValue converted;
    if (!value.convertTo(exported->type, converted)) return false;

    const zen::Value held = ref.get();
    if (!zen::is_instance(held)) return false;
    zen::ObjInstance* instance = zen::as_instance(held);
    int index = -1;
    fieldLookup(instance, name.c_str(), index);
    if (index < 0) return false;

    zen::Value field = zen::val_nil();
    switch (exported->type)
    {
        case PropertyType::Bool:
            field = zen::val_bool(converted.asBool());
            break;
        case PropertyType::Int:
            field = zen::val_int(converted.asInt());
            break;
        case PropertyType::Float:
            field = zen::val_float(converted.asFloat());
            break;
        case PropertyType::String:
        {
            const ct::String text = converted.asString();
            field = zen::val_obj(reinterpret_cast<zen::Obj*>(
                    impl.vm.make_string(text.c_str(), static_cast<int>(text.size()))));
            break;
        }
        default:
            return false;
    }
    instance->fields[index] = field;
    return true;
}

ZenScripting::Impl::Impl(ZenScripting& owner)
        : scripting(owner),
          compiling(false),
          errorCount(0),
          compileCount(0),
          propertyOverflowLogged(false),
          initSelector(-1),
          noop(zen::val_nil()),
          nodeClass(nullptr),
          sceneTreeClass(nullptr),
          inputEventClass(nullptr),
          rayHitClass(nullptr),
          tweenClass(nullptr),
          tweenerClass(nullptr),
          vec4Def(nullptr),
          quatDef(nullptr),
          colorDef(nullptr),
          treePointer(nullptr),
          printHandler(nullptr),
          printUser(nullptr),
          hotReload(true),
          reloadInterval(ZenScripting::kDefaultReloadInterval),
          reloadElapsed(0.0),
          bindingsSignature(0)
{
    zen::ZenCallbacks callbacks = zen::zen_default_callbacks();
    callbacks.io.open = &fileOpen;
    callbacks.io.read = &fileRead;
    callbacks.io.write = &fileWrite;
    callbacks.io.seek = &fileSeek;
    callbacks.io.close = &fileClose;
    callbacks.io.exists = &fileExists;
    callbacks.print_err = &captureError;
    callbacks.userdata = this;
    vm.set_callbacks(callbacks);
    zen_host_set_writer(&hostWrite, this);

    vm.open_lib_globals(&zen::zen_lib_base);
    vm.register_lib(&zen::zen_lib_math);
    vm.register_lib(&zen::zen_lib_time);
    vm.register_lib(&zen::zen_lib_struct);
    vm.register_lib(&zen::zen_lib_io);
    vm.register_lib(&zen::zen_lib_os);
    vm.register_lib(&zen::zen_lib_path);
    vm.register_lib(&zen::zen_lib_json);

    for (int i = 0; i < kHookCount; ++i)
    {
        hookSelector[i][0] = vm.intern_selector(kZenHooks[i].primary,
                static_cast<int>(strlen(kZenHooks[i].primary)));
        hookSelector[i][1] = vm.intern_selector(kZenHooks[i].alias,
                static_cast<int>(strlen(kZenHooks[i].alias)));
    }
    initSelector = vm.intern_selector("__init__", 8);

    vm.def_native("__noop", &noopNative, 0);
    noop = vm.get_global("__noop");

    registerValueTypes();
    registerNodeClasses();
    registerGlobals();
    registerTweenClasses();
    vm.resolve_native_globals();

    unsigned long long hash = 1469598103934665603ull;
    const auto mix = [&hash](const char* text)
    {
        for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text ? text : "");;
                ++p)
        {
            hash = (hash ^ *p) * 1099511628211ull;
            if (!*p) break;
        }
    };
    for (int i = 0; i < vm.num_globals(); ++i) mix(vm.global_name(i));
    mix("|");
    for (int i = 0; i < vm.num_selectors(); ++i) mix(vm.selector_name(i));
    bindingsSignature = hash;
}

ZenScripting::Impl::~Impl()
{
    releaseAll();
    zen_host_set_writer(nullptr, nullptr);
}

void ZenScripting::Impl::emitHostText(const char* text, std::size_t length, bool isError)
{
    ct::String& buffer = isError ? hostErrorBuffer : printBuffer;
    for (std::size_t i = 0; i < length; ++i)
    {
        if (text[i] != '\n')
        {
            buffer += text[i];
            continue;
        }
        if (isError)
        {
            if (!compiling) log_error("script: %s", buffer.c_str());
        }
        else if (printHandler)
            printHandler(buffer.c_str(), printUser);
        else
            log_info("%s", buffer.c_str());
        buffer = ct::String();
    }
}

void ZenScripting::Impl::pruneTweens()
{
    SceneTree* tree = SceneTree::singleton();
    for (std::size_t i = 0; i < tweenHolds.size();)
    {
        if (tree && tree->hasTween(tweenHolds[i].tween)) ++i;
        else
        {
            tweenHolds[i] = static_cast<TweenHold&&>(tweenHolds[tweenHolds.size() - 1]);
            tweenHolds.pop_back();
        }
    }
    for (std::size_t i = 0; i < tweenerHandles.size();)
    {
        if (tree && tree->hasTween(tweenerHandles[i]->tween)) ++i;
        else
        {
            delete tweenerHandles[i];
            tweenerHandles[i] = tweenerHandles[tweenerHandles.size() - 1];
            tweenerHandles.pop_back();
        }
    }
}

void ZenScripting::Impl::releaseAll()
{
    for (std::size_t i = 0; i < tweenerHandles.size(); ++i) delete tweenerHandles[i];
    tweenerHandles.clear();
    tweenHolds.clear();
    while (!instances.empty())
    {
        ZenNodeInstance* instance = instances.back();
        instance->node->setScriptInstance(nullptr);
    }
    for (std::size_t i = 0; i < entries.size(); ++i) delete entries[i];
    entries.clear();
    entryIndex.clear();
    treeRef.reset();
}

zen::ObjClass* ZenScripting::Impl::classFor(const char* className)
{
    zen::ObjClass* const* found = classes.find(ct::String(className));
    if (found) return *found;
    zen::ObjClass* const* base = classes.find(ct::String("Node"));
    return base ? *base : nullptr;
}

ZenNodeInstance* ZenScripting::Impl::instanceFor(Node* node)
{
    if (!node) return nullptr;
    ScriptInstance* existing = node->getScriptInstance();
    if (existing) return static_cast<ZenNodeInstance*>(existing);
    ZenNodeInstance* instance = new ZenNodeInstance(*this, node);
    node->setScriptInstance(instance);
    return instance;
}

zen::Value ZenScripting::Impl::wrap(Node* node)
{
    ZenNodeInstance* instance = instanceFor(node);
    return instance ? instance->value() : zen::val_nil();
}

bool ZenScripting::Impl::isNodeValue(zen::Value value) const
{
    if (!zen::is_instance(value) || !nodeClass) return false;
    const zen::ObjInstance* instance = zen::as_instance(value);
    if (!instance->native_data) return false;
    for (const zen::ObjClass* klass = instance->klass; klass; klass = klass->parent)
        if (klass == nodeClass) return true;
    return false;
}

Node* ZenScripting::Impl::nodeOf(zen::Value value) const
{
    if (!isNodeValue(value)) return nullptr;
    return static_cast<Node*>(zen::as_instance(value)->native_data);
}

void ZenScripting::Impl::clearVmError()
{
    if (vm.had_error()) vm.call_fn(noop, nullptr, 0);
}

void ZenScripting::Impl::reportMessage(const char* format, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    ++errorCount;
    lastError = buffer;
    log_error("script: %s", buffer);
}

void ZenScripting::Impl::reportError(const ZenNodeInstance* instance, const ct::String& text)
{
    ++errorCount;
    lastError = describeVmError(text);
    if (instance && instance->node)
        log_error("script error on node '%s': %s\n(the script of this node is disabled)",
                instance->node->getName().c_str(), lastError.c_str());
    else
        log_error("script error: %s", lastError.c_str());
}

void ZenScripting::Impl::clearFrameRegisters(zen::Value value, int slot, int nargs)
{
    zen::ObjFiber* fiber = vm.current_fiber();
    if (!fiber || !zen::is_instance(value)) return;
    const zen::ObjClass* klass = zen::as_instance(value)->klass;
    if (slot < 0 || slot >= klass->vtable_size || !zen::is_closure(klass->vtable[slot])) return;
    const int count = zen::as_closure(klass->vtable[slot])->func->num_regs + nargs + 2;
    zen::Value* top = fiber->stack_top;
    if (top + count > fiber->stack_end) return;
    for (int i = 0; i < count; ++i) top[i] = zen::val_nil();
}

bool ZenScripting::Impl::invokeGuarded(ZenNodeInstance* instance, int slot, zen::Value* args,
        int nargs)
{
    clearVmError();
    const std::size_t mark = errorBuffer.size();
    clearFrameRegisters(instance->value(), slot, nargs);
    vm.invoke(instance->value(), slot, args, nargs);
    if (!vm.had_error()) return true;

    const ct::String text = errorBuffer.substr(mark);
    errorBuffer = errorBuffer.substr(0, mark);
    clearVmError();
    instance->faulted = true;
    reportError(instance, text);
    return false;
}

bool ZenScripting::Impl::rebindInstance(zen::Value value, zen::ObjClass* target, bool preserve)
{
    if (!zen::is_instance(value) || !target) return false;
    zen::ObjInstance* instance = zen::as_instance(value);
    zen::ObjClass* previous = instance->klass;

    zen::GC* gc = &vm.get_gc();
    zen::gc_pause(gc);

    const int count = target->num_fields;
    zen::Value* fresh = nullptr;
    if (count > 0)
    {
        fresh = static_cast<zen::Value*>(
                zen::zen_alloc(gc, sizeof(zen::Value) * static_cast<std::size_t>(count)));
        const int defaults = target->field_defaults ? target->num_field_defaults : 0;
        for (int k = 0; k < count; ++k)
            fresh[k] = k < defaults ? target->field_defaults[k] : zen::val_nil();

        if (preserve)
        {
            const int oldDefaults = previous->field_defaults ? previous->num_field_defaults : 0;
            for (int j = 0; j < previous->num_fields && j < instance->num_fields; ++j)
            {
                for (int k = 0; k < count; ++k)
                {
                    if (target->field_names[k] != previous->field_names[j]) continue;
                    const bool unmodified =
                            j < oldDefaults && k < defaults &&
                            zen::values_equal(instance->fields[j], previous->field_defaults[j]);
                    if (!unmodified) fresh[k] = instance->fields[j];
                    break;
                }
            }
        }
    }

    if (instance->fields && !instance->fields_inline)
        zen::zen_free(gc, instance->fields,
                sizeof(zen::Value) * static_cast<std::size_t>(instance->num_fields));
    instance->fields = fresh;
    instance->num_fields = count;
    instance->fields_inline = false;
    instance->klass = target;

    zen::gc_resume(gc);
    return true;
}

bool ZenScripting::Impl::compileEntry(ZenEntry* entry, bool reload)
{
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(entry->path);
    const bool bytecode = zen::is_bytecode_buffer(bytes.data(), bytes.size());
    const ct::String resolved = FileSystem::getSingleton().resolve(entry->path);
    const ct::String name = resolved.empty() ? entry->path : resolved;

    long long mtime = 0;
    long long size = 0;
    if (!resolved.empty())
    {
        mtime = file_mod_time(resolved.c_str());
        size = file_size(resolved.c_str());
    }
    entry->resolved = resolved;
    entry->mtime = mtime;
    entry->size = size;
    entry->noNodeClass = false;

    if (bytes.empty())
    {
        reportMessage("%s: the script cannot be read or is empty", entry->path.c_str());
        return false;
    }

    const int globalsBefore = vm.num_globals();
    ct::Vector<zen::Ref> keep;
    ct::Vector<zen::ObjClass*> before;
    for (int i = 0; i < globalsBefore; ++i)
    {
        const zen::Value global = vm.get_global(i);
        if (zen::is_class(global))
        {
            before.push_back(zen::as_class(global));
            keep.push_back(zen::Ref(vm, global));
        }
        else
            before.push_back(nullptr);
    }

    const std::size_t mark = errorBuffer.size();
    clearVmError();

    zen::ObjFunc* function = nullptr;
    if (bytecode)
    {
        char message[256] = "";
        function =
                zen::load_bytecode_buffer(&vm, bytes.data(), bytes.size(), message, sizeof message);
        if (!function)
        {
            vm.shrink_globals(globalsBefore);
            errorBuffer = errorBuffer.substr(0, mark);
            reportMessage("%s: %s", name.c_str(),
                    message[0] ? message : "the bytecode cannot be loaded");
            return false;
        }
    }
    else
    {
        const ct::String source(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        zen::Compiler* compiler = new zen::Compiler();
        compiling = true;
        function = compiler->compile(&vm.get_gc(), &vm, source.c_str(), name.c_str());
        compiling = false;
        if (!function)
        {
            char line[32] = "";
            ct::String message = "compilation failed";
            if (compiler->error_info_count() > 0)
            {
                snprintf(line, sizeof(line), "%d", compiler->error_line(0));
                message = compiler->error_message(0);
            }
            delete compiler;
            vm.shrink_globals(globalsBefore);
            errorBuffer = errorBuffer.substr(0, mark);
            reportMessage("%s:%s: %s", name.c_str(), line, message.c_str());
            return false;
        }
        delete compiler;
    }
    entry->function = zen::Ref(vm, zen::val_obj(reinterpret_cast<zen::Obj*>(function)));

    vm.call_fn(zen::val_obj(reinterpret_cast<zen::Obj*>(makeClosure(vm, function))), nullptr, 0);
    if (vm.had_error())
    {
        const ct::String text = errorBuffer.substr(mark);
        errorBuffer = errorBuffer.substr(0, mark);
        clearVmError();
        reportError(nullptr, text);
        return false;
    }
    errorBuffer = errorBuffer.substr(0, mark);
    ++compileCount;

    const ct::String stem = stemOf(entry->path);
    zen::ObjClass* picked = nullptr;
    ct::String pickedBase;
    int candidates = 0;
    for (int i = 0; i < vm.num_globals(); ++i)
    {
        const zen::Value global = vm.get_global(i);
        if (!zen::is_class(global)) continue;
        zen::ObjClass* klass = zen::as_class(global);
        if (i < globalsBefore && before[static_cast<std::size_t>(i)] == klass) continue;
        ct::String base;
        for (zen::ObjClass* parent = klass->parent; parent && base.empty(); parent = parent->parent)
        {
            if (!parent->name) continue;
            zen::ObjClass* const* known = classes.find(ct::String(parent->name->chars));
            if (known && *known == parent) base = parent->name->chars;
        }
        if (base.empty()) continue;
        ++candidates;
        const bool matchesStem = klass->name && stem == klass->name->chars;
        if (!picked || matchesStem)
        {
            picked = klass;
            pickedBase = base;
        }
    }
    if (!picked)
    {
        entry->noNodeClass = true;
        if (!probing)
            reportMessage("%s: the script defines no class that extends a node class "
                          "(for example 'class Player(Node3D):')",
                    name.c_str());
        return false;
    }
    if (candidates > 1)
        log_warn("script: %s defines %d node classes; using '%s'", name.c_str(), candidates,
                picked->name->chars);

    int slots[kHookCount];
    for (int h = 0; h < kHookCount; ++h)
    {
        slots[h] = -1;
        for (int variant = 0; variant < 2 && slots[h] < 0; ++variant)
        {
            const int selector = hookSelector[h][variant];
            if (selector < 0 || selector >= picked->vtable_size) continue;
            const zen::Value method = picked->vtable[selector];
            if (!zen::is_closure(method)) continue;
            if (arityAccepts(method, kZenHooks[h].arity)) slots[h] = selector;
        }
    }
    int initSlot = -1;
    if (initSelector >= 0 && initSelector < picked->vtable_size &&
            arityAccepts(picked->vtable[initSelector], 0))
        initSlot = initSelector;

    zen::ScriptHook hooks[kHookCount * 2];
    for (int h = 0; h < kHookCount; ++h)
    {
        hooks[h * 2] = { kZenHooks[h].primary, kZenHooks[h].arity, true };
        hooks[h * 2 + 1] = { kZenHooks[h].alias, kZenHooks[h].arity, true };
    }
    zen::ScriptDiagnostic diagnostics[16];
    const int diagnosticCount =
            zen::check_script_contract(picked, hooks, kHookCount * 2, diagnostics, 16, 0);
    for (int d = 0; d < diagnosticCount; ++d)
    {
        if (diagnostics[d].severity == zen::SCRIPT_DIAG_ERROR)
            reportMessage("%s: %s", name.c_str(), diagnostics[d].message);
        else
            log_warn("script: %s: %s", name.c_str(), diagnostics[d].message);
    }

    ct::Vector<ZenExport> exports;
    bool tool = false;
    zen::ScriptPropertyInfo infos[64];
    const int infoCount = zen::script_class_properties(picked, infos, 64);
    for (int i = 0; i < infoCount; ++i)
    {
        if (strcmp(infos[i].name, "tool") == 0)
        {
            tool = zen::is_truthy_full(infos[i].value);
            continue;
        }
        if (infos[i].name[0] == '_') continue;
        PropertyInfo existing;
        if (NodeClassDB::getPropertyInfo(pickedBase, infos[i].name, existing))
        {
            log_warn("script: %s: the variable '%s' hides a property of %s and is not exported",
                    name.c_str(), infos[i].name, pickedBase.c_str());
            continue;
        }
        ZenExport exported;
        exported.name = infos[i].name;
        const zen::Value defaultValue = infos[i].value;
        if (zen::is_bool(defaultValue))
        {
            exported.type = PropertyType::Bool;
            exported.defaultValue = PropertyValue::fromBool(defaultValue.as.boolean);
        }
        else if (zen::is_int(defaultValue))
        {
            exported.type = PropertyType::Int;
            exported.defaultValue = PropertyValue::fromInt(defaultValue.as.integer);
        }
        else if (zen::is_float(defaultValue))
        {
            exported.type = PropertyType::Float;
            exported.defaultValue = PropertyValue::fromFloat(defaultValue.as.number);
        }
        else
        {
            exported.type = PropertyType::String;
            exported.defaultValue =
                    PropertyValue::fromString(ct::String(zen::safe_string_chars(defaultValue),
                            static_cast<std::size_t>(zen::safe_string_len(defaultValue))));
        }
        exports.push_back(exported);
    }

    entry->klass = picked;
    entry->classRef = zen::Ref(vm, zen::val_obj(reinterpret_cast<zen::Obj*>(picked)));
    entry->baseClass = pickedBase;
    entry->tool = tool;
    for (int h = 0; h < kHookCount; ++h) entry->hookSlot[h] = slots[h];
    entry->initSlot = initSlot;
    entry->exports = exports;
    ++entry->version;

    if (reload)
    {
        for (std::size_t i = 0; i < instances.size(); ++i)
        {
            ZenNodeInstance* instance = instances[i];
            if (instance->entry != entry) continue;
            rebindInstance(instance->value(), entry->klass, true);
            instance->faulted = false;
            instance->node->notifyPropertyListChanged();
            if (instance->node->isReady()) instance->applyProcessFlags();
        }
    }
    return true;
}

ZenEntry* ZenScripting::Impl::loadEntry(const ct::String& path)
{
    ZenEntry* const* found = entryIndex.find(path);
    if (found)
    {
        ZenEntry* entry = *found;
        if (entry->klass) return entry;
        const ct::String resolved = FileSystem::getSingleton().resolve(path);
        if (!resolved.empty() && file_mod_time(resolved.c_str()) == entry->mtime &&
                file_size(resolved.c_str()) == entry->size)
            return nullptr;
        return compileEntry(entry, false) ? entry : nullptr;
    }

    ZenEntry* entry = new ZenEntry();
    entry->path = path;
    entries.push_back(entry);
    entryIndex.put(path, entry);
    const std::size_t slash = path.rfind('/');
    if (slash != ct::String::npos && slash > 0)
    {
        const ct::String directory = path.substr(0, slash);
        bool known = false;
        for (std::size_t i = 0; i < vmSearchPaths.size() && !known; ++i)
            known = vmSearchPaths[i] == directory;
        if (!known)
        {
            vmSearchPaths.push_back(directory);
            vm.add_search_path(directory.c_str());
        }
    }
    return compileEntry(entry, false) ? entry : nullptr;
}

int ZenScripting::Impl::reloadChanged()
{
    int reloaded = 0;
    const std::size_t count = entries.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        ZenEntry* entry = entries[i];
        if (entry->resolved.empty()) continue;
        const long long mtime = file_mod_time(entry->resolved.c_str());
        const long long size = file_size(entry->resolved.c_str());
        if (mtime < 0 || size < 0) continue;
        if (mtime == entry->mtime && size == entry->size) continue;
        if (compileEntry(entry, true)) ++reloaded;
    }
    return reloaded;
}

ZenScripting::ZenScripting()
        : mImpl(nullptr)
{
    sInstance = this;
    mImpl = new Impl(*this);
}

ZenScripting::~ZenScripting()
{
    delete mImpl;
    mImpl = nullptr;
    if (sInstance == this) sInstance = nullptr;
}

ZenScripting* ZenScripting::instance()
{
    return sInstance;
}

ZenScripting::Impl* ZenScripting::getImpl() const
{
    return mImpl;
}

void ZenScripting::attachScript(Node* node, const ct::String& path)
{
    Impl& impl = *mImpl;
    ZenNodeInstance* existing = static_cast<ZenNodeInstance*>(node->getScriptInstance());
    if (path.empty())
    {
        if (existing) existing->unbind();
        return;
    }

    ZenEntry* entry = impl.loadEntry(path);
    if (!entry)
    {
        if (existing) existing->unbind();
        return;
    }
    if (!node->isClass(entry->baseClass.c_str()))
    {
        impl.reportMessage("%s extends %s, which does not fit the node '%s' (%s)", path.c_str(),
                entry->baseClass.c_str(), node->getName().c_str(), node->getClassName());
        if (existing) existing->unbind();
        return;
    }

    ZenNodeInstance* instance = impl.instanceFor(node);
    instance->bind(entry, false);
    if (node->isReady()) instance->applyProcessFlags();
    if (instance->canRun()) instance->ensureInit();
}

void ZenScripting::update(double delta)
{
    Impl& impl = *mImpl;
    impl.pruneTweens();
    if (!impl.hotReload) return;
    impl.reloadElapsed += delta;
    if (impl.reloadElapsed < impl.reloadInterval) return;
    impl.reloadElapsed = 0.0;
    impl.reloadChanged();
}

void ZenScripting::setPrintHandler(PrintHandler handler, void* user)
{
    mImpl->printHandler = handler;
    mImpl->printUser = user;
}

int ZenScripting::preload(const ct::Vector<ct::String>& paths)
{
    int loaded = 0;
    for (std::size_t i = 0; i < paths.size(); ++i)
        if (mImpl->loadEntry(paths[i])) ++loaded;
    return loaded;
}

unsigned long long ZenScripting::getBindingsSignature() const
{
    return mImpl->bindingsSignature;
}

int ZenScripting::preloadList(const ct::String& listName)
{
    const ct::String text = FileSystem::getSingleton().readText(listName);
    if (text.empty()) return 0;
    ct::Json::Error parseError;
    const ct::Json document = ct::Json::parse(text, &parseError);
    const ct::Json* list = &document;
    if (!parseError && document.is_object())
    {
        if (const ct::Json* engine = document.find("engine"))
        {
            char expected[32];
            snprintf(expected, sizeof expected, "%016llx", mImpl->bindingsSignature);
            if (!engine->is_string() || !(engine->str() == expected))
            {
                log_error("script: the scripts in this pack were compiled for another engine "
                          "build; export the project again with this one");
                return 0;
            }
        }
        list = document.find("scripts");
    }
    if (parseError || !list || !list->is_array())
    {
        log_warn("script: %s is not a list of script paths", listName.c_str());
        return 0;
    }
    ct::Vector<ct::String> paths;
    for (std::size_t i = 0; i < list->size(); ++i)
        if (list->at(i).is_string()) paths.push_back(list->at(i).str());
    return preload(paths);
}

ZenScripting::ExportResult ZenScripting::exportBytecode(const ct::String& path,
        const ct::String& outputFile, bool stripDebug, ct::String& error)
{
    mImpl->probing = true;
    ZenEntry* entry = mImpl->loadEntry(path);
    mImpl->probing = false;
    if (!entry || !zen::is_func(entry->function.get()))
    {
        ZenEntry* const* tried = mImpl->entryIndex.find(path);
        if (tried && (*tried)->noNodeClass) return kNotNodeScript;
        error = getLastError().empty() ? ct::String("the script did not load") : getLastError();
        return kExportFailed;
    }
    char message[256] = "";
    if (!zen::dump_bytecode_file(&mImpl->vm, zen::as_func(entry->function.get()),
                outputFile.c_str(), stripDebug, message, sizeof message))
    {
        error = message[0] ? ct::String(message) : ct::String("the bytecode could not be written");
        return kExportFailed;
    }
    return kExported;
}

int ZenScripting::reloadChanged()
{
    return mImpl->reloadChanged();
}

void ZenScripting::setHotReload(bool enabled, double interval)
{
    mImpl->hotReload = enabled;
    mImpl->reloadInterval = interval;
    mImpl->reloadElapsed = 0.0;
}

bool ZenScripting::isHotReload() const
{
    return mImpl->hotReload;
}

int ZenScripting::getCompileCount() const
{
    return mImpl->compileCount;
}

int ZenScripting::getErrorCount() const
{
    return mImpl->errorCount;
}

const ct::String& ZenScripting::getLastError() const
{
    return mImpl->lastError;
}

void ZenScripting::clearErrors()
{
    mImpl->errorCount = 0;
    mImpl->lastError = ct::String();
}

bool ZenScripting::isFaulted(const Node* node) const
{
    const ScriptInstance* script = node ? node->getScriptInstance() : nullptr;
    return script && static_cast<const ZenNodeInstance*>(script)->faulted;
}

bool ZenScripting::isRunnable(const Node* node) const
{
    const ScriptInstance* script = node ? node->getScriptInstance() : nullptr;
    if (!script) return false;
    const ZenNodeInstance* instance = static_cast<const ZenNodeInstance*>(script);
    return instance->canRun() && !instance->faulted;
}

} // namespace prisma3d
