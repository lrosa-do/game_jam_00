#include "prisma3d/scene/SceneSerializer.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/scene/MissingNode.h"
#include "prisma3d/scene/ModelInstance.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/ScriptInstance.h"

#include "NodeFactory.h"
#include "platform.h"

#include <ct/json.hpp>

namespace prisma3d
{

namespace
{

ct::String indexPath(const ct::String& base, const char* field, std::size_t index)
{
    ct::String path = base;
    path += '.';
    path += field;
    path += '[';
    path.append_number(static_cast<unsigned long long>(index));
    path += ']';
    return path;
}

ct::String fieldPath(const ct::String& base, const ct::String& field)
{
    ct::String path = base;
    path += '.';
    path += field;
    return path;
}

bool isNodeKey(const ct::String& key)
{
    return key == "class" || key == "name" || key == "properties" || key == "data" ||
           key == "children" || key == "index" || key == "groups";
}

bool isInstanceKey(const ct::String& key)
{
    return key == "instance" || key == "name" || key == "properties" || key == "children" ||
           key == "index" || key == "editable" || key == "overrides" || key == "groups";
}

bool isOverrideKey(const ct::String& key)
{
    return key == "path" || key == "properties" || key == "children" || key == "groups";
}

bool isContentOf(const Node* node, const Node* instance)
{
    const Node* owner = node->getOwner();
    return owner && (owner == instance || instance->isAncestorOf(owner));
}

ct::Json writeProperties(const Node* node, const Node* reference, SceneResult& result,
        const ct::String& path)
{
    const ct::String className = node->getClassName();
    const bool compare = reference && className == reference->getClassName();

    ct::Json properties = ct::Json::object();
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList(className, list);
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        const PropertyInfo& info = list[i];
        if (info.isGroup() || !(info.usage & PropertyUsage::Storage)) continue;
        PropertyValue value;
        if (!NodeClassDB::getProperty(node, info.name, value)) continue;
        if (info.type == PropertyType::ResourcePath && value.asString().empty() &&
                NodeClassDB::isPropertyUnsaved(node, info.name))
        {
            result.addWarning(fieldPath(fieldPath(path, "properties"), info.name),
                    "the resource has no path; the property was not saved");
            continue;
        }
        PropertyValue baseValue;
        const bool known =
                compare ? NodeClassDB::getProperty(reference, info.name, baseValue)
                        : NodeClassDB::getDefaultPropertyValue(className, info.name, baseValue);
        if (known && value == baseValue) continue;
        properties[info.name] = scenejson::fromValue(value);
    }

    const ScriptInstance* script = node->getScriptInstance();
    if (script && script->hasScript())
    {
        const ScriptInstance* referenceScript = compare ? reference->getScriptInstance() : nullptr;
        ct::Vector<PropertyInfo> exported;
        script->getPropertyList(exported);
        for (std::size_t i = 0; i < exported.size(); ++i)
        {
            const PropertyInfo& info = exported[i];
            PropertyValue value;
            if (!script->getProperty(info.name, value)) continue;
            PropertyValue baseValue;
            const bool known = referenceScript && referenceScript->hasScript()
                                       ? referenceScript->getProperty(info.name, baseValue)
                                       : script->getDefaultProperty(info.name, baseValue);
            if (known && value == baseValue) continue;
            properties[info.name] = scenejson::fromValue(value);
        }
    }
    return properties;
}

ct::Json writeGroups(const Node* node, const Node* reference)
{
    ct::Json groups = ct::Json::array();
    ct::Vector<Node::GroupInfo> list;
    node->getGroups(list);
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        if (!list[i].persistent) continue;
        if (reference && reference->isInGroup(list[i].name)) continue;
        groups.push_back(ct::Json(list[i].name));
    }
    return groups;
}

struct OverrideEntry
{
    ct::String path;
    ct::Json properties = ct::Json::object();
    ct::Json groups = ct::Json::array();
    ct::Json children = ct::Json::array();
};

std::size_t overrideIndex(ct::Vector<OverrideEntry>& overrides, const ct::String& path)
{
    for (std::size_t i = 0; i < overrides.size(); ++i)
        if (overrides[i].path == path) return i;
    OverrideEntry entry;
    entry.path = path;
    overrides.push_back(entry);
    return overrides.size() - 1;
}

bool isInnerPath(const ct::String& path)
{
    if (path.empty() || path.front() == '/') return false;
    std::size_t pos = 0;
    while (pos <= path.size())
    {
        std::size_t end = path.find('/', pos);
        if (end == ct::String::npos) end = path.size();
        const ct::String part = path.substr(pos, end - pos);
        if (part.empty() || part == "." || part == "..") return false;
        pos = end + 1;
    }
    return true;
}

} // namespace

ct::Json SceneSerializer::writeChildren(const Node* node, SceneResult& result,
        const ct::String& path)
{
    ct::Json children = ct::Json::array();
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        const Node* child = node->getChild(i);
        if (child->isInternal()) continue;
        const std::size_t index = children.size();
        ct::Json entry = ct::Json::object();
        writeNode(child, entry, result, indexPath(path, "children", index), false);
        children.push_back(entry);
    }
    return children;
}

void SceneSerializer::writeMissing(const MissingNode* node, ct::Json& out, SceneResult& result,
        const ct::String& path)
{
    out = node->getEntry();
    out["name"] = ct::Json(node->getName());

    const ct::Json* original = out.find("children");
    if (original && !original->is_array()) return;

    ct::Json children = writeChildren(node, result, path);
    if (children.size() > 0) out["children"] = children;
    else
        out.erase("children");
}

void SceneSerializer::writeNode(const Node* node, ct::Json& out, SceneResult& result,
        const ct::String& path, bool plain)
{
    if (const MissingNode* missing = node->castTo<MissingNode>())
    {
        writeMissing(missing, out, result, path);
        return;
    }

    if (node->isInstance() && !plain)
    {
        writeInstance(node, out, result, path);
        return;
    }

    const ct::String className = node->getClassName();
    out["class"] = ct::Json(className);
    out["name"] = ct::Json(node->getName());

    ct::Json properties = writeProperties(node, nullptr, result, path);
    if (properties.size() > 0) out["properties"] = properties;

    ct::Json groups = writeGroups(node, nullptr);
    if (groups.size() > 0) out["groups"] = groups;

    ct::Json data = ct::Json::object();
    if (node->saveData(data, result, fieldPath(path, "data"))) out["data"] = data;

    ct::Json children = writeChildren(node, result, path);
    if (children.size() > 0) out["children"] = children;
}

void SceneSerializer::writeInstance(const Node* node, ct::Json& out, SceneResult& result,
        const ct::String& path)
{
    const ct::String file = node->getSceneFilePath();
    out["instance"] = ct::Json(file);
    out["name"] = ct::Json(node->getName());

    Prefab::BaseScope scope;
    const Node* base = scope.get(file);
    if (!base)
    {
        result.addWarning(fieldPath(path, "instance"),
                ct::String("the prefab '") + file +
                        "' could not be built; every property that differs from the class "
                        "default is saved");
    }

    ct::Json properties = writeProperties(node, base, result, path);
    if (properties.size() > 0) out["properties"] = properties;

    ct::Json groups = writeGroups(node, base);
    if (groups.size() > 0) out["groups"] = groups;

    const bool editable = node->mEditableInstance;
    if (editable) out["editable"] = ct::Json(true);

    ct::Vector<OverrideEntry> overrides;
    ct::Json children = ct::Json::array();

    struct Walker
    {
        const Node* instance;
        const Node* base;
        bool editable;
        SceneResult& result;
        const ct::String& path;
        ct::Vector<OverrideEntry>& overrides;
        ct::Json& children;

        void walk(const Node* current)
        {
            const bool atRoot = current == instance;
            for (std::size_t i = 0; i < current->getChildCount(); ++i)
            {
                const Node* child = current->getChild(i);
                if (child->isInternal()) continue;
                if (isContentOf(child, instance))
                {
                    if (editable && base)
                    {
                        const ct::String relative = instance->getPathTo(child);
                        const Node* counterpart =
                                relative.empty() ? nullptr : base->getNode(relative);
                        if (counterpart)
                        {
                            ct::Json changed = writeProperties(child, counterpart, result,
                                    fieldPath(path, "overrides"));
                            if (changed.size() > 0)
                                overrides[overrideIndex(overrides, relative)].properties = changed;
                            ct::Json added = writeGroups(child, counterpart);
                            if (added.size() > 0)
                                overrides[overrideIndex(overrides, relative)].groups = added;
                        }
                    }
                    walk(child);
                    continue;
                }

                ct::Json entry = ct::Json::object();
                if (atRoot)
                {
                    SceneSerializer::writeNode(child, entry, result,
                            indexPath(path, "children", children.size()), false);
                    entry["index"] = ct::Json(static_cast<long long>(child->getIndex()));
                    children.push_back(entry);
                    continue;
                }

                const ct::String relative = instance->getPathTo(current);
                if (!editable || relative.empty())
                {
                    result.addWarning(path,
                            ct::String("node '") + child->getName() + "' was added inside '" +
                                    current->getName() +
                                    "', which is part of the prefab and not editable; it was not "
                                    "saved");
                    continue;
                }
                const std::size_t target = overrideIndex(overrides, relative);
                SceneSerializer::writeNode(child, entry, result,
                        indexPath(indexPath(path, "overrides", target), "children",
                                overrides[target].children.size()),
                        false);
                entry["index"] = ct::Json(static_cast<long long>(child->getIndex()));
                overrides[target].children.push_back(entry);
            }
        }
    };

    Walker walker{ node, base, editable, result, path, overrides, children };
    walker.walk(node);

    if (children.size() > 0) out["children"] = children;
    if (overrides.size() > 0)
    {
        ct::Json list = ct::Json::array();
        for (std::size_t i = 0; i < overrides.size(); ++i)
        {
            ct::Json entry = ct::Json::object();
            entry["path"] = ct::Json(overrides[i].path);
            if (overrides[i].properties.size() > 0) entry["properties"] = overrides[i].properties;
            if (overrides[i].groups.size() > 0) entry["groups"] = overrides[i].groups;
            if (overrides[i].children.size() > 0) entry["children"] = overrides[i].children;
            list.push_back(entry);
        }
        out["overrides"] = list;
    }
}

void SceneSerializer::applyProperties(Node* node, const ct::Json& properties,
        const ct::String& path, ReadContext& context, SceneResult& result)
{
    const ct::String className = node->getClassName();
    const ct::String propertiesPath = fieldPath(path, "properties");
    if (!properties.is_object())
    {
        result.addError(propertiesPath, "must be an object");
        return;
    }

    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList(className, list);
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        const PropertyInfo& info = list[i];
        if (info.isGroup()) continue;
        const ct::Json* field = properties.find(info.name);
        if (!field) continue;
        const ct::String valuePath = fieldPath(propertiesPath, info.name);
        PropertyValue value;
        ct::String error;
        if (!scenejson::toValue(*field, info.type, value, error))
        {
            result.addError(valuePath, error);
            continue;
        }
        if (info.type == PropertyType::NodePath)
        {
            PendingPath entry;
            entry.node = node;
            entry.property = info.name;
            entry.value = value;
            entry.path = valuePath;
            context.pending.push_back(entry);
            continue;
        }
        sceneres::clearError();
        if (!NodeClassDB::setProperty(node, info.name, value))
            result.addError(valuePath, "the property cannot be written");
        else if (info.type == PropertyType::ResourcePath)
        {
            const ct::String issue = sceneres::takeError();
            if (!issue.empty()) result.addWarning(valuePath, issue);
        }
    }

    ScriptInstance* script = node->getScriptInstance();
    ct::Vector<PropertyInfo> exported;
    if (script && script->hasScript()) script->getPropertyList(exported);

    const ct::Json::Object& members = properties.members();
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        bool known = false;
        for (std::size_t j = 0; j < list.size() && !known; ++j)
            known = !list[j].isGroup() && list[j].name == members[i].key;
        if (known) continue;
        const PropertyInfo* scripted = nullptr;
        for (std::size_t j = 0; j < exported.size() && !scripted; ++j)
            if (exported[j].name == members[i].key) scripted = &exported[j];
        if (scripted)
        {
            const ct::String valuePath = fieldPath(propertiesPath, scripted->name);
            PropertyValue value;
            ct::String error;
            if (!scenejson::toValue(members[i].value, scripted->type, value, error))
                result.addError(valuePath, error);
            else if (!script->setProperty(scripted->name, value))
                result.addError(valuePath, "the property cannot be written");
            continue;
        }
        if (!known)
            result.addWarning(fieldPath(propertiesPath, members[i].key),
                    ct::String("unknown property for class '") + className + "', ignored");
    }
}

void SceneSerializer::applyGroups(Node* node, const ct::Json& json, const ct::String& path,
        SceneResult& result)
{
    const ct::Json* field = json.find("groups");
    if (!field) return;
    const ct::String groupsPath = fieldPath(path, "groups");
    if (!field->is_array())
    {
        result.addError(groupsPath, "must be an array");
        return;
    }
    for (std::size_t i = 0; i < field->size(); ++i)
    {
        const ct::Json& entry = field->at(i);
        if (!entry.is_string() || entry.str().empty())
        {
            result.addWarning(indexPath(path, "groups", i), "not a group name, ignored");
            continue;
        }
        node->addToGroup(entry.str(), true);
    }
}

void SceneSerializer::resolvePending(ReadContext& context, SceneResult& result)
{
    for (std::size_t i = 0; i < context.pending.size(); ++i)
    {
        PendingPath& entry = context.pending[i];
        if (!NodeClassDB::setProperty(entry.node, entry.property, entry.value))
            result.addError(entry.path, "the property cannot be written");
    }
    context.pending.clear();
}

void SceneSerializer::attachChild(Node* parent, Node* child, const ct::Json& json,
        const ct::String& path, SceneResult& result)
{
    const ct::String wanted = child->getName();
    parent->addChild(child, true);
    if (!wanted.empty() && child->getName() != wanted)
        result.addWarning(fieldPath(path, "name"),
                ct::String("the name '") + wanted +
                        "' is already used by a sibling; the node "
                        "was renamed '" +
                        child->getName() + "'");
    const ct::Json* index = json.find("index");
    if (!index) return;
    std::int64_t position = 0;
    if (!scenejson::toInt(*index, position) || position < 0)
    {
        result.addWarning(fieldPath(path, "index"), "not a non-negative whole number, ignored");
        return;
    }
    if (static_cast<std::size_t>(position) + 1 < parent->getChildCount())
        parent->moveChild(child, static_cast<std::size_t>(position));
}

void SceneSerializer::readChildren(Node* parent, const ct::Json& json, const ct::String& path,
        ReadContext& context, SceneResult& result)
{
    const ct::Json* childrenField = json.find("children");
    if (!childrenField) return;
    if (!childrenField->is_array())
    {
        result.addError(fieldPath(path, "children"), "must be an array");
        return;
    }
    for (std::size_t i = 0; i < childrenField->size(); ++i)
    {
        const ct::String childPath = indexPath(path, "children", i);
        Node* child = readNode(childrenField->at(i), childPath, context, result);
        if (child) attachChild(parent, child, childrenField->at(i), childPath, result);
    }
}

Node* SceneSerializer::readMissing(const ct::Json& json, const ct::String& originalClass,
        const ct::String& originalScene, const ct::String& path, ReadContext& context,
        SceneResult& result)
{
    MissingNode* node = NodeFactory::make<MissingNode>();
    node->setOriginalClass(originalClass);
    node->setOriginalScene(originalScene);

    ct::Json entry = json;
    entry.erase("index");
    node->setEntry(entry);

    const ct::Json* nameField = json.find("name");
    if (nameField && nameField->is_string()) node->setNameNoCheck(nameField->str());

    readChildren(node, json, path, context, result);
    return node;
}

Node* SceneSerializer::readNode(const ct::Json& json, const ct::String& path, ReadContext& context,
        SceneResult& result)
{
    if (!json.is_object())
    {
        result.addError(path, "expected an object");
        return nullptr;
    }

    if (json.find("instance")) return readInstance(json, path, context, result);

    const ct::Json* classField = json.find("class");
    if (!classField || !classField->is_string())
    {
        result.addError(fieldPath(path, "class"), "missing or not a text");
        return readMissing(json, ct::String(), ct::String(), path, context, result);
    }
    const ct::String className = classField->str();
    if (!NodeClassDB::classExists(className))
    {
        result.addError(fieldPath(path, "class"), ct::String("unknown class '") + className + "'");
        return readMissing(json, className, ct::String(), path, context, result);
    }
    if (!NodeClassDB::canInstantiate(className))
    {
        result.addError(fieldPath(path, "class"),
                ct::String("class '") + className + "' is abstract and cannot be created");
        return readMissing(json, className, ct::String(), path, context, result);
    }

    Node* node = NodeClassDB::instantiate(className);
    if (!node)
    {
        result.addError(fieldPath(path, "class"),
                ct::String("class '") + className + "' could not be created");
        return readMissing(json, className, ct::String(), path, context, result);
    }

    const ct::Json* nameField = json.find("name");
    if (nameField)
    {
        if (nameField->is_string()) node->setNameNoCheck(nameField->str());
        else
            result.addWarning(fieldPath(path, "name"), "not a text, ignored");
    }

    const ct::Json* propertiesField = json.find("properties");
    if (propertiesField) applyProperties(node, *propertiesField, path, context, result);

    applyGroups(node, json, path, result);

    const ct::Json* dataField = json.find("data");
    if (dataField)
    {
        const ct::String dataPath = fieldPath(path, "data");
        if (!dataField->is_object())
        {
            result.addError(dataPath, "must be an object");
        }
        else
        {
            const std::size_t before = result.errorCount();
            if (!node->loadData(*dataField, result, dataPath) && result.errorCount() == before)
                result.addError(dataPath, "the data was not accepted");
        }
    }

    const ct::Json::Object& members = json.members();
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        if (!isNodeKey(members[i].key))
            result.addWarning(fieldPath(path, members[i].key), "unknown field, ignored");
    }

    readChildren(node, json, path, context, result);

    return node;
}

Node* SceneSerializer::readInstance(const ct::Json& json, const ct::String& path,
        ReadContext& context, SceneResult& result)
{
    const ct::String instancePath = fieldPath(path, "instance");
    const ct::Json* instanceField = json.find("instance");
    if (!instanceField || !instanceField->is_string() || instanceField->str().empty())
    {
        result.addError(instancePath, "missing or not a text");
        return readMissing(json, ct::String(), ct::String(), path, context, result);
    }
    const ct::String file = FileSystem::simplifyPath(instanceField->str());
    if (file.empty())
    {
        result.addError(instancePath, "not a usable path");
        return readMissing(json, ct::String(), instanceField->str(), path, context, result);
    }

    for (std::size_t i = 0; i < context.prefabStack.size(); ++i)
    {
        if (context.prefabStack[i] != file) continue;
        ct::String message = "the prefab contains itself: ";
        for (std::size_t j = i; j < context.prefabStack.size(); ++j)
        {
            message += context.prefabStack[j];
            message += " -> ";
        }
        message += file;
        result.addError(instancePath, message);
        return readMissing(json, ct::String(), file, path, context, result);
    }

    Node* instance = nullptr;
    const std::size_t pendingBefore = context.pending.size();
    const ModelLoader* models = ModelLoader::singleton();
    if (models && models->importerFor(AssetLoader::modelFile(file)))
    {
        sceneres::clearError();
        const ModelId model = sceneres::loadModel(file);
        instance = model.valid() ? buildModel(model) : nullptr;
        if (!instance)
        {
            const ct::String reason = sceneres::takeError();
            result.addWarning(instancePath,
                    reason.empty() ? ct::String("the model could not be instantiated") : reason);
            return readMissing(json, ct::String(), file, path, context, result);
        }
    }
    else
    {
        ct::Rc<ct::Json> document = Prefab::acquire(file, result, instancePath, true);
        if (!document) return readMissing(json, ct::String(), file, path, context, result);
        const ct::Json* rootField = document->find("root");
        if (!rootField) return readMissing(json, ct::String(), file, path, context, result);

        ct::String contentPath = path;
        contentPath += '<';
        contentPath += file;
        contentPath += ">.root";
        context.prefabStack.push_back(file);
        instance = readNode(*rootField, contentPath, context, result);
        context.prefabStack.pop_back();
        if (!instance) return readMissing(json, ct::String(), file, path, context, result);
        if (instance->castTo<MissingNode>())
        {
            context.pending.resize(pendingBefore);
            NodeFactory::destroy(instance);
            return readMissing(json, ct::String(), file, path, context, result);
        }
    }

    instance->setSceneFilePath(file);
    Prefab::claimContent(instance);

    const ct::Json* nameField = json.find("name");
    if (nameField)
    {
        if (nameField->is_string()) instance->setNameNoCheck(nameField->str());
        else
            result.addWarning(fieldPath(path, "name"), "not a text, ignored");
    }

    const ct::Json* propertiesField = json.find("properties");
    if (propertiesField) applyProperties(instance, *propertiesField, path, context, result);

    applyGroups(instance, json, path, result);

    const ct::Json* editableField = json.find("editable");
    if (editableField)
    {
        bool editable = false;
        if (scenejson::toBool(*editableField, editable)) instance->mEditableInstance = editable;
        else
            result.addWarning(fieldPath(path, "editable"), "not a boolean, ignored");
    }

    const ct::Json::Object& members = json.members();
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        if (isInstanceKey(members[i].key)) continue;
        if (members[i].key == "class" || members[i].key == "data")
            result.addWarning(fieldPath(path, members[i].key),
                    "the class and the data come from the prefab, ignored");
        else
            result.addWarning(fieldPath(path, members[i].key), "unknown field, ignored");
    }

    const ct::Json* overridesField = json.find("overrides");
    if (overridesField)
    {
        const ct::String overridesPath = fieldPath(path, "overrides");
        if (!overridesField->is_array())
        {
            result.addError(overridesPath, "must be an array");
        }
        else
        {
            for (std::size_t i = 0; i < overridesField->size(); ++i)
            {
                const ct::Json& entry = overridesField->at(i);
                const ct::String entryPath = indexPath(path, "overrides", i);
                if (!entry.is_object())
                {
                    result.addError(entryPath, "expected an object");
                    continue;
                }
                const ct::Json* pathField = entry.find("path");
                if (!pathField || !pathField->is_string() || !isInnerPath(pathField->str()))
                {
                    result.addError(fieldPath(entryPath, "path"),
                            "missing or not a path to a node inside the instance");
                    continue;
                }
                Node* target = instance->getNode(pathField->str());
                if (!target || target == instance || !instance->isAncestorOf(target))
                {
                    result.addWarning(fieldPath(entryPath, "path"),
                            ct::String("there is no node '") + pathField->str() +
                                    "' in the prefab '" + file + "', ignored");
                    continue;
                }
                const ct::Json::Object& fields = entry.members();
                for (std::size_t j = 0; j < fields.size(); ++j)
                    if (!isOverrideKey(fields[j].key))
                        result.addWarning(fieldPath(entryPath, fields[j].key),
                                "unknown field, ignored");
                const ct::Json* overrideProperties = entry.find("properties");
                if (overrideProperties)
                    applyProperties(target, *overrideProperties, entryPath, context, result);
                applyGroups(target, entry, entryPath, result);
                readChildren(target, entry, entryPath, context, result);
            }
        }
    }

    readChildren(instance, json, path, context, result);
    return instance;
}

bool SceneResult::success() const
{
    return errorCount() == 0;
}

std::size_t SceneResult::errorCount() const
{
    std::size_t count = 0;
    for (std::size_t i = 0; i < diagnostics.size(); ++i)
        if (diagnostics[i].severity == SceneDiagnosticSeverity::Error) ++count;
    return count;
}

std::size_t SceneResult::warningCount() const
{
    return diagnostics.size() - errorCount();
}

void SceneResult::addWarning(const ct::String& path, const ct::String& message)
{
    SceneDiagnostic diagnostic;
    diagnostic.severity = SceneDiagnosticSeverity::Warning;
    diagnostic.path = path;
    diagnostic.message = message;
    diagnostics.push_back(diagnostic);
}

void SceneResult::addError(const ct::String& path, const ct::String& message)
{
    SceneDiagnostic diagnostic;
    diagnostic.severity = SceneDiagnosticSeverity::Error;
    diagnostic.path = path;
    diagnostic.message = message;
    diagnostics.push_back(diagnostic);
}

ct::String SceneResult::describe() const
{
    ct::String text;
    for (std::size_t i = 0; i < diagnostics.size(); ++i)
    {
        const SceneDiagnostic& diagnostic = diagnostics[i];
        text += diagnostic.severity == SceneDiagnosticSeverity::Error ? "error" : "warning";
        text += ' ';
        text += diagnostic.path.size() > 0 ? diagnostic.path : ct::String("(document)");
        text += ": ";
        text += diagnostic.message;
        text += '\n';
    }
    return text;
}

ct::String SceneSerializer::documentText(const ct::Json& root)
{
    ct::Json document = ct::Json::object();
    document["format"] = ct::Json(kFormatName);
    document["version"] = ct::Json(kFormatVersion);
    document["root"] = root;

    ct::String text = document.dump(2);
    text += '\n';
    return text;
}

bool SceneSerializer::save(const Node* root, ct::String& text, SceneResult* result)
{
    if (!root)
    {
        if (result) result->addError("", "no node to save");
        return false;
    }

    SceneResult local;
    SceneResult& report = result ? *result : local;
    ct::Json rootJson = ct::Json::object();
    {
        Prefab::BaseScope scope;
        writeNode(root, rootJson, report, "root", false);
    }

    text = documentText(rootJson);
    return true;
}

bool SceneSerializer::saveFile(const Node* root, const ct::String& path, SceneResult* result)
{
    ct::String text;
    if (!save(root, text, result)) return false;
    if (!FileSystem::writeText(path, text))
    {
        if (result) result->addError("", ct::String("could not write '") + path + "'");
        return false;
    }
    return true;
}

bool SceneSerializer::parseDocument(const ct::String& text, ct::Json& document, SceneResult& result)
{
    ct::Json::Error parseError;
    ct::Json parsed = ct::Json::parse(text, &parseError);
    if (parseError)
    {
        ct::String message = "not valid JSON: ";
        message += parseError.message;
        message += " (line ";
        message.append_number(static_cast<unsigned long long>(parseError.line));
        message += ", column ";
        message.append_number(static_cast<unsigned long long>(parseError.column));
        message += ')';
        result.addError("", message);
        return false;
    }
    if (!parsed.is_object())
    {
        result.addError("", "the document is not a JSON object");
        return false;
    }

    const ct::Json* format = parsed.find("format");
    if (!format || !format->is_string() || format->str() != kFormatName)
    {
        result.addError("format", ct::String("missing or not '") + kFormatName + "'");
        return false;
    }

    const ct::Json* version = parsed.find("version");
    std::int64_t versionNumber = 0;
    if (!version || !scenejson::toInt(*version, versionNumber) || versionNumber <= 0)
    {
        result.addError("version", "missing or not a positive whole number");
        return false;
    }
    if (versionNumber > static_cast<std::int64_t>(kFormatVersion))
    {
        ct::String message = "the file is version ";
        message.append_number(static_cast<long long>(versionNumber));
        message += ", newer than this build's ";
        message.append_number(static_cast<unsigned long long>(kFormatVersion));
        result.addError("version", message);
        return false;
    }

    const ct::Json::Object& members = parsed.members();
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        const ct::String& key = members[i].key;
        if (key != "format" && key != "version" && key != "root")
            result.addWarning(key, "unknown field, ignored");
    }

    if (!parsed.find("root"))
    {
        result.addError("root", "missing root node");
        return false;
    }

    document = parsed;
    return true;
}

Node* SceneSerializer::loadText(const ct::String& text, Node* parent, SceneResult& result,
        const ct::String& file)
{
    if (!parent)
    {
        result.addError("", "no parent node to load the scene under");
        return nullptr;
    }

    ct::Json document;
    if (!parseDocument(text, document, result)) return nullptr;

    ReadContext context;
    if (!file.empty()) context.prefabStack.push_back(file);
    Node* root = readNode(*document.find("root"), "root", context, result);

    resolvePending(context, result);

    if (root) parent->addChild(root, true);
    return root;
}

Node* SceneSerializer::load(const ct::String& text, Node* parent, SceneResult& result)
{
    return loadText(text, parent, result, ct::String());
}

Node* SceneSerializer::loadFile(const ct::String& path, Node* parent, SceneResult& result)
{
    const ct::String text = FileSystem::getSingleton().readText(path);
    if (text.empty())
    {
        result.addError("", ct::String("could not read '") + path + "'");
        return nullptr;
    }
    return loadText(text, parent, result, FileSystem::simplifyPath(path));
}

} // namespace prisma3d
