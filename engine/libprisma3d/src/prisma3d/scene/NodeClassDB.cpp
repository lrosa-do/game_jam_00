#include "prisma3d/scene/NodeClassDB.h"

#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/ScriptInstance.h"

#include "platform.h"

#include <ct/hashmap.hpp>

namespace prisma3d
{

struct NodeClassDB::Registry
{
    ct::Vector<ClassInfo> classes;
    ct::HashMap<ct::String, std::size_t> indices;
};

NodeClassDB::Registry& NodeClassDB::registry()
{
    static Registry instance;
    return instance;
}

void NodeClassDB::ensureSceneTypes()
{
    registerSceneTypes();
}

const NodeClassDB::ClassInfo* NodeClassDB::find(const ct::String& className)
{
    Registry& db = registry();
    const std::size_t* index = db.indices.find(className);
    return index ? &db.classes[*index] : nullptr;
}

bool NodeClassDB::addClass(const char* name, const char* parent, void* classPtr)
{
    Registry& db = registry();
    ct::String key(name);
    if (db.indices.contains(key)) return false;
    ClassInfo info;
    info.name = key;
    info.parent = parent;
    info.classPtr = classPtr;
    info.creation = nullptr;
    db.indices.put(key, db.classes.size());
    db.classes.push_back(info);
    return true;
}

void NodeClassDB::setCreation(const char* name, CreationFunc creation)
{
    Registry& db = registry();
    const std::size_t* index = db.indices.find(ct::String(name));
    if (!index)
    {
        log_error("node class: cannot register '%s'", name);
        return;
    }
    db.classes[*index].creation = creation;
}

void NodeClassDB::getClassList(ct::Vector<ct::String>& classes)
{
    ensureSceneTypes();
    const Registry& db = registry();
    for (std::size_t i = 0; i < db.classes.size(); ++i) classes.push_back(db.classes[i].name);
}

void NodeClassDB::getInheritersFromClass(const ct::String& className,
        ct::Vector<ct::String>& classes)
{
    ensureSceneTypes();
    const Registry& db = registry();
    for (std::size_t i = 0; i < db.classes.size(); ++i)
    {
        const ct::String& name = db.classes[i].name;
        if (name != className && isParentClass(name, className)) classes.push_back(name);
    }
}

void NodeClassDB::getDirectInheritersFromClass(const ct::String& className,
        ct::Vector<ct::String>& classes)
{
    ensureSceneTypes();
    const Registry& db = registry();
    for (std::size_t i = 0; i < db.classes.size(); ++i)
    {
        if (db.classes[i].parent == className) classes.push_back(db.classes[i].name);
    }
}

ct::String NodeClassDB::getParentClass(const ct::String& className)
{
    ensureSceneTypes();
    const ClassInfo* info = find(className);
    if (!info)
    {
        log_error("node class: cannot get class '%s'", className.c_str());
        return ct::String();
    }
    return info->parent;
}

bool NodeClassDB::classExists(const ct::String& className)
{
    ensureSceneTypes();
    return find(className) != nullptr;
}

bool NodeClassDB::isParentClass(const ct::String& className, const ct::String& inherits)
{
    ensureSceneTypes();
    const ClassInfo* info = find(className);
    while (info)
    {
        if (info->name == inherits) return true;
        info = find(info->parent);
    }
    return false;
}

bool NodeClassDB::canInstantiate(const ct::String& className)
{
    ensureSceneTypes();
    const ClassInfo* info = find(className);
    if (!info)
    {
        log_error("node class: cannot get class '%s'", className.c_str());
        return false;
    }
    return info->creation != nullptr;
}

bool NodeClassDB::isAbstract(const ct::String& className)
{
    ensureSceneTypes();
    const ClassInfo* info = find(className);
    if (!info)
    {
        log_error("node class: cannot get class '%s'", className.c_str());
        return false;
    }
    return info->creation == nullptr;
}

Node* NodeClassDB::instantiate(const ct::String& className)
{
    ensureSceneTypes();
    const ClassInfo* info = find(className);
    if (!info)
    {
        log_error("node class: cannot get class '%s'", className.c_str());
        return nullptr;
    }
    if (!info->creation)
    {
        log_error("node class: '%s' is abstract and cannot be instantiated", className.c_str());
        return nullptr;
    }
    return info->creation();
}

std::size_t PropertyInfo::enumCount() const
{
    if (hintString.size() == 0) return 0;
    std::size_t count = 1;
    for (std::size_t i = 0; i < hintString.size(); ++i)
    {
        if (hintString.data()[i] == ',') ++count;
    }
    return count;
}

ct::String PropertyInfo::enumName(std::size_t index) const
{
    std::size_t start = 0;
    std::size_t current = 0;
    for (std::size_t i = 0; i <= hintString.size(); ++i)
    {
        if (i == hintString.size() || hintString.data()[i] == ',')
        {
            if (current == index) return hintString.substr(start, i - start);
            ++current;
            start = i + 1;
        }
    }
    return ct::String();
}

void NodeClassDB::addPropertyGroup(const char* className, const char* name, const char* prefix,
        bool closed)
{
    Registry& db = registry();
    const std::size_t* index = db.indices.find(ct::String(className));
    if (!index)
    {
        log_error("node class: cannot add a property group to '%s'", className);
        return;
    }
    PropertyEntry entry;
    entry.info = PropertyInfo(PropertyType::Nil, name, PropertyHint::None, prefix,
            closed ? PropertyUsage::Group | PropertyUsage::GroupClosed : PropertyUsage::Group);
    db.classes[*index].properties.push_back(entry);
}

void NodeClassDB::addPropertySubgroup(const char* className, const char* name, const char* prefix)
{
    Registry& db = registry();
    const std::size_t* index = db.indices.find(ct::String(className));
    if (!index)
    {
        log_error("node class: cannot add a property subgroup to '%s'", className);
        return;
    }
    PropertyEntry entry;
    entry.info = PropertyInfo(PropertyType::Nil, name, PropertyHint::None, prefix,
            PropertyUsage::Subgroup);
    db.classes[*index].properties.push_back(entry);
}

void NodeClassDB::addProperty(const char* className, const PropertyInfo& info,
        const PropertySetter& setter, const PropertyGetter& getter, const PropertyUnsaved& unsaved)
{
    Registry& db = registry();
    const std::size_t* index = db.indices.find(ct::String(className));
    if (!index)
    {
        log_error("node class: cannot add property '%s' to '%s'", info.name.c_str(), className);
        return;
    }
    ClassInfo& type = db.classes[*index];
    if (type.propertyIndex.contains(info.name))
    {
        log_error("node class: property '%s' of '%s' is already registered", info.name.c_str(),
                className);
        return;
    }
    PropertyEntry entry;
    entry.info = info;
    entry.setter = setter;
    entry.getter = getter;
    entry.unsaved = unsaved;
    type.propertyIndex.put(info.name, type.properties.size());
    type.properties.push_back(entry);
}

void NodeClassDB::setPropertyDefaultValue(const char* className, const char* name,
        const PropertyValue& value)
{
    Registry& db = registry();
    const std::size_t* index = db.indices.find(ct::String(className));
    if (!index)
    {
        log_error("node class: cannot set a default value on '%s'", className);
        return;
    }
    db.classes[*index].defaults.put(ct::String(name), value);
}

void NodeClassDB::getPropertyList(const ct::String& className, ct::Vector<PropertyInfo>& list,
        bool noInheritance)
{
    ensureSceneTypes();
    ct::Vector<const ClassInfo*> chain;
    const ClassInfo* type = find(className);
    while (type)
    {
        chain.push_back(type);
        if (noInheritance) break;
        type = find(type->parent);
    }
    for (std::size_t i = chain.size(); i > 0; --i)
    {
        const ClassInfo* current = chain[i - 1];
        for (std::size_t j = 0; j < current->properties.size(); ++j)
            list.push_back(current->properties[j].info);
    }
}

void NodeClassDB::getInstancePropertyList(const Node* node, const ct::String& className,
        ct::Vector<PropertyInfo>& list, bool noInheritance)
{
    const std::size_t first = list.size();
    getPropertyList(className, list, noInheritance);
    if (!node) return;
    for (std::size_t i = first; i < list.size(); ++i)
    {
        if (list[i].isGroup()) continue;
        node->validateProperty(list[i]);
    }
    const ScriptInstance* script = node->getScriptInstance();
    const bool holdsScript = !noInheritance || className == "Node";
    if (script && script->hasScript() && holdsScript)
    {
        ct::Vector<PropertyInfo> exported;
        script->getPropertyList(exported);
        if (exported.size() > 0)
        {
            list.push_back(PropertyInfo(PropertyType::Nil, "Script Variables", PropertyHint::None,
                    "", PropertyUsage::Group));
            for (std::size_t i = 0; i < exported.size(); ++i) list.push_back(exported[i]);
        }
    }
}

bool NodeClassDB::getPropertyInfo(const ct::String& className, const ct::String& property,
        PropertyInfo& info, bool noInheritance)
{
    ensureSceneTypes();
    const ClassInfo* type = find(className);
    while (type)
    {
        const std::size_t* index = type->propertyIndex.find(property);
        if (index)
        {
            info = type->properties[*index].info;
            return true;
        }
        if (noInheritance) break;
        type = find(type->parent);
    }
    return false;
}

bool NodeClassDB::setProperty(Node* node, const ct::String& property, const PropertyValue& value)
{
    if (!node) return false;
    ensureSceneTypes();
    const ClassInfo* type = find(node->getClassName());
    while (type)
    {
        const std::size_t* index = type->propertyIndex.find(property);
        if (index)
        {
            const PropertyEntry& entry = type->properties[*index];
            if (!entry.setter) return false;
            PropertyValue converted;
            if (!value.convertTo(entry.info.type, converted)) return false;
            entry.setter(node, converted);
            return true;
        }
        type = find(type->parent);
    }
    if (ScriptInstance* script = node->getScriptInstance())
        return script->setProperty(property, value);
    return false;
}

bool NodeClassDB::getProperty(const Node* node, const ct::String& property, PropertyValue& value)
{
    if (!node) return false;
    ensureSceneTypes();
    const ClassInfo* type = find(node->getClassName());
    while (type)
    {
        const std::size_t* index = type->propertyIndex.find(property);
        if (index)
        {
            const PropertyEntry& entry = type->properties[*index];
            if (!entry.getter) return false;
            value = entry.getter(node);
            return true;
        }
        type = find(type->parent);
    }
    if (const ScriptInstance* script = node->getScriptInstance())
        return script->getProperty(property, value);
    return false;
}

bool NodeClassDB::isPropertyUnsaved(const Node* node, const ct::String& property)
{
    if (!node) return false;
    ensureSceneTypes();
    const ClassInfo* type = find(node->getClassName());
    while (type)
    {
        const std::size_t* index = type->propertyIndex.find(property);
        if (index)
        {
            const PropertyEntry& entry = type->properties[*index];
            return entry.unsaved && entry.unsaved(node);
        }
        type = find(type->parent);
    }
    return false;
}

bool NodeClassDB::getDefaultPropertyValue(const ct::String& className, const ct::String& property,
        PropertyValue& value)
{
    ensureSceneTypes();
    const ClassInfo* found = find(className);
    if (!found) return false;
    ClassInfo* type = const_cast<ClassInfo*>(found);

    if (!type->defaultsCached)
    {
        type->defaultsCached = true;
        if (type->creation)
        {
            Node* instance = type->creation();
            ct::Vector<PropertyInfo> list;
            getPropertyList(className, list);
            for (std::size_t i = 0; i < list.size(); ++i)
            {
                const PropertyInfo& info = list[i];
                if (!(info.usage & (PropertyUsage::Storage | PropertyUsage::Editor))) continue;
                if (type->defaults.contains(info.name)) continue;
                PropertyValue current;
                if (getProperty(instance, info.name, current))
                    type->defaults.put(info.name, current);
            }
            delete instance;
        }
    }

    const PropertyValue* result = type->defaults.find(property);
    if (!result) return false;
    value = *result;
    return true;
}

} // namespace prisma3d
