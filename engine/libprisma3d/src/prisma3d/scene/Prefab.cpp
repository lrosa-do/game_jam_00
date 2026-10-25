#include "prisma3d/scene/Prefab.h"

#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/render/AssetLoader.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/scene/MissingNode.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"

#include "NodeFactory.h"
#include "platform.h"

#include <ct/hashmap.hpp>

namespace prisma3d
{

namespace
{

struct BaseCache
{
    ct::Vector<ct::String> files;
    ct::Vector<Node*> nodes;
};

ct::HashMap<ct::String, ct::Rc<ct::Json>> sDocuments;
std::size_t sReads = 0;
BaseCache* sBases = nullptr;

const char* const kTransform[] = { "position", "rotation", "scale" };

ct::String joinPath(const ct::String& base, const ct::String& file, const ct::String& inner)
{
    ct::String path = base;
    path += '<';
    path += file;
    path += '>';
    if (!inner.empty())
    {
        path += '.';
        path += inner;
    }
    return path;
}

void copyDiagnostics(const SceneResult& from, SceneResult& to, const ct::String& base,
        const ct::String& file)
{
    for (std::size_t i = 0; i < from.diagnostics.size(); ++i)
    {
        const SceneDiagnostic& diagnostic = from.diagnostics[i];
        const ct::String path = joinPath(base, file, diagnostic.path);
        if (diagnostic.severity == SceneDiagnosticSeverity::Error)
            to.addError(path, diagnostic.message);
        else
            to.addWarning(path, diagnostic.message);
    }
}

void collectDependencies(const ct::Json& node, ct::Vector<ct::String>& out)
{
    if (!node.is_object()) return;
    const ct::Json* instance = node.find("instance");
    if (instance && instance->is_string()) out.push_back(FileSystem::simplifyPath(instance->str()));

    const ct::Json* children = node.find("children");
    if (children && children->is_array())
        for (std::size_t i = 0; i < children->size(); ++i)
            collectDependencies(children->at(i), out);

    const ct::Json* overrides = node.find("overrides");
    if (overrides && overrides->is_array())
    {
        for (std::size_t i = 0; i < overrides->size(); ++i)
        {
            const ct::Json& entry = overrides->at(i);
            if (!entry.is_object()) continue;
            const ct::Json* added = entry.find("children");
            if (!added || !added->is_array()) continue;
            for (std::size_t j = 0; j < added->size(); ++j) collectDependencies(added->at(j), out);
        }
    }
}

Node* topOf(Node* node)
{
    while (node->getParent()) node = node->getParent();
    return node;
}

void forceTransform(ct::Json& entry, const Node* node)
{
    for (const char* name: kTransform)
    {
        PropertyValue value;
        if (!NodeClassDB::getProperty(node, name, value) || value.type() != PropertyType::Vec3)
            continue;
        entry["properties"][name] = scenejson::fromValue(value);
    }
}

} // namespace

ct::Rc<ct::Json> Prefab::acquire(const ct::String& file, SceneResult& result,
        const ct::String& path, bool missingIsWarning)
{
    if (const ct::Rc<ct::Json>* cached = sDocuments.find(file)) return *cached;

    const ct::String text = FileSystem::getSingleton().readText(file);
    if (text.empty())
    {
        const ct::String message = ct::String("could not read the prefab '") + file + "'";
        if (missingIsWarning) result.addWarning(path, message);
        else
            result.addError(path, message);
        return ct::Rc<ct::Json>();
    }
    ++sReads;

    ct::Json document;
    SceneResult parsed;
    const bool valid = SceneSerializer::parseDocument(text, document, parsed);
    copyDiagnostics(parsed, result, path, file);
    if (!valid) return ct::Rc<ct::Json>();

    ct::Rc<ct::Json> shared = ct::make_rc<ct::Json>(document);
    sDocuments.put(file, shared);
    return shared;
}

bool Prefab::reaches(const ct::String& from, const ct::String& target,
        ct::Vector<ct::String>& visited, ct::Vector<ct::String>& chain)
{
    for (std::size_t i = 0; i < visited.size(); ++i)
        if (visited[i] == from) return false;
    visited.push_back(from);
    chain.push_back(from);

    SceneResult ignored;
    ct::Rc<ct::Json> document = acquire(from, ignored, ct::String(), true);
    if (document)
    {
        const ct::Json* root = document->find("root");
        ct::Vector<ct::String> dependencies;
        if (root) collectDependencies(*root, dependencies);
        for (std::size_t i = 0; i < dependencies.size(); ++i)
        {
            if (dependencies[i] == target)
            {
                chain.push_back(target);
                return true;
            }
            if (reaches(dependencies[i], target, visited, chain)) return true;
        }
    }

    chain.pop_back();
    return false;
}

bool Prefab::dependsOn(const ct::String& path, const ct::String& target,
        ct::Vector<ct::String>* chain)
{
    ct::Vector<ct::String> visited;
    ct::Vector<ct::String> found;
    const bool result = reaches(FileSystem::simplifyPath(path), FileSystem::simplifyPath(target),
            visited, found);
    if (chain) *chain = found;
    return result;
}

bool Prefab::refresh(const ct::String& file, SceneResult& result)
{
    const ct::String text = FileSystem::getSingleton().readText(file);
    if (text.empty())
    {
        result.addError(ct::String(), ct::String("could not read the prefab '") + file + "'");
        return false;
    }
    ++sReads;
    return store(file, text, result);
}

bool Prefab::store(const ct::String& file, const ct::String& text, SceneResult& result)
{
    ct::Json document;
    SceneResult parsed;
    const bool valid = SceneSerializer::parseDocument(text, document, parsed);
    copyDiagnostics(parsed, result, ct::String(), file);
    if (!valid) return false;
    sDocuments.put(file, ct::make_rc<ct::Json>(document));
    return true;
}

bool Prefab::writeFile(const ct::String& file, const ct::String& text, SceneResult& result)
{
    ct::String target = FileSystem::getSingleton().resolve(file);
    if (target.empty()) target = file;
    if (FileSystem::writeText(target, text)) return true;
    result.addError(ct::String(), ct::String("could not write '") + target + "'");
    return false;
}

bool Prefab::isCached(const ct::String& path)
{
    return sDocuments.contains(FileSystem::simplifyPath(path));
}

void Prefab::forget(const ct::String& path)
{
    sDocuments.erase(FileSystem::simplifyPath(path));
}

void Prefab::clearCache()
{
    sDocuments.clear();
}

std::size_t Prefab::fileReadCount()
{
    return sReads;
}

Prefab::BaseScope::BaseScope()
        : mOwner(sBases == nullptr)
{
    if (mOwner) sBases = new BaseCache;
}

Prefab::BaseScope::~BaseScope()
{
    if (!mOwner) return;
    for (std::size_t i = 0; i < sBases->nodes.size(); ++i) delete sBases->nodes[i];
    delete sBases;
    sBases = nullptr;
}

const Node* Prefab::BaseScope::get(const ct::String& file)
{
    for (std::size_t i = 0; i < sBases->files.size(); ++i)
        if (sBases->files[i] == file) return sBases->nodes[i];
    Node* node = buildBase(file);
    sBases->files.push_back(file);
    sBases->nodes.push_back(node);
    return node;
}

Node* Prefab::buildBase(const ct::String& file)
{
    ct::Json entry = ct::Json::object();
    entry["instance"] = ct::Json(file);
    SceneSerializer::ReadContext context;
    SceneResult ignored;
    Node* node = SceneSerializer::readInstance(entry, ct::String(), context, ignored);
    if (node && node->castTo<MissingNode>())
    {
        NodeFactory::destroy(node);
        return nullptr;
    }
    SceneSerializer::resolvePending(context, ignored);
    return node;
}

void Prefab::claimContent(Node* instance)
{
    struct Claim
    {
        static void run(Node* instance, Node* node)
        {
            for (std::size_t i = 0; i < node->getChildCount(); ++i)
            {
                Node* child = node->getChild(i);
                if (!child->getOwner()) child->setOwner(instance);
                run(instance, child);
            }
        }
    };
    Claim::run(instance, instance);
}

bool Prefab::checkCycle(const Node* root, const ct::String& file, bool skipRoot,
        SceneResult& result)
{
    struct Check
    {
        static bool run(const Node* root, const Node* node, const ct::String& file, bool skipRoot,
                SceneResult& result)
        {
            if (node->isInstance() && !(skipRoot && node == root))
            {
                ct::Vector<ct::String> chain;
                const ct::String& used = node->getSceneFilePath();
                bool cycle = used == file;
                if (cycle) chain.push_back(file);
                else
                    cycle = dependsOn(used, file, &chain);
                if (cycle)
                {
                    ct::String route = file;
                    for (std::size_t i = 0; i < chain.size(); ++i)
                    {
                        route += " -> ";
                        route += chain[i];
                    }
                    result.addError(root->getPathTo(node),
                            ct::String("the prefab would contain itself: ") + route);
                    return false;
                }
            }
            for (std::size_t i = 0; i < node->getChildCount(); ++i)
                if (!run(root, node->getChild(i), file, skipRoot, result)) return false;
            return true;
        }
    };
    return Check::run(root, root, file, skipRoot, result);
}

bool Prefab::saveFromNode(const Node* root, const ct::String& path, SceneResult& result,
        Node* reloadRoot)
{
    if (!root)
    {
        result.addError(ct::String(), "no node to save");
        return false;
    }
    const ct::String file = FileSystem::simplifyPath(path);
    if (file.empty())
    {
        result.addError(ct::String(), "no path to save the prefab to");
        return false;
    }
    if (!checkCycle(root, file, false, result)) return false;

    ct::Vector<Node*> candidates;
    ct::Vector<ct::Json> entries;
    ct::String text;
    {
        BaseScope scope;
        if (reloadRoot)
        {
            ct::Vector<ct::String> related;
            ct::Vector<ct::String> unrelated;
            collectCandidates(reloadRoot, file, candidates, related, unrelated);
            for (std::size_t i = 0; i < candidates.size(); ++i)
                entries.push_back(makeEntry(candidates[i], true, result));
        }

        ct::Json rootJson = ct::Json::object();
        SceneSerializer::writeNode(root, rootJson, result, "root", false);
        text = SceneSerializer::documentText(rootJson);
    }
    if (!writeFile(file, text, result)) return false;
    if (!store(file, text, result)) return false;

    for (std::size_t i = 0; i < candidates.size(); ++i)
        replaceInPlace(candidates[i], entries[i], result);
    return true;
}

Node* Prefab::instantiate(const ct::String& path, Node* parent, SceneResult& result)
{
    if (!parent)
    {
        result.addError(ct::String(), "no parent node to instantiate the prefab under");
        return nullptr;
    }
    const ct::String file = FileSystem::simplifyPath(path);
    if (file.empty())
    {
        result.addError(ct::String(), "no path to instantiate");
        return nullptr;
    }
    const ModelLoader* models = ModelLoader::singleton();
    const bool model = models && models->importerFor(AssetLoader::modelFile(file));
    if (!model && !acquire(file, result, "instance", false)) return nullptr;

    ct::Json entry = ct::Json::object();
    entry["instance"] = ct::Json(file);
    SceneSerializer::ReadContext context;
    Node* node = SceneSerializer::readInstance(entry, "instance", context, result);
    if (node && node->castTo<MissingNode>())
    {
        NodeFactory::destroy(node);
        return nullptr;
    }
    SceneSerializer::resolvePending(context, result);
    if (node) parent->addChild(node, true);
    return node;
}

void Prefab::findInstances(Node* searchRoot, const ct::String& path, ct::Vector<Node*>& out)
{
    if (!searchRoot) return;
    const ct::String file = FileSystem::simplifyPath(path);
    if (searchRoot->getSceneFilePath() == file && searchRoot->isInstance())
        out.push_back(searchRoot);
    for (std::size_t i = 0; i < searchRoot->getChildCount(); ++i)
        findInstances(searchRoot->getChild(i), file, out);
}

void Prefab::collectCandidates(Node* node, const ct::String& file, ct::Vector<Node*>& out,
        ct::Vector<ct::String>& related, ct::Vector<ct::String>& unrelated)
{
    if (node->isInstance())
    {
        const ct::String& used = node->getSceneFilePath();
        bool match = used == file;
        for (std::size_t i = 0; !match && i < related.size(); ++i) match = related[i] == used;
        bool known = match;
        for (std::size_t i = 0; !known && i < unrelated.size(); ++i) known = unrelated[i] == used;
        if (!known)
        {
            match = dependsOn(used, file);
            if (match) related.push_back(used);
            else
                unrelated.push_back(used);
        }
        if (match)
        {
            out.push_back(node);
            return;
        }
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collectCandidates(node->getChild(i), file, out, related, unrelated);
}

ct::Json Prefab::makeEntry(const Node* instance, bool keepTransform, SceneResult& result)
{
    ct::Json entry = ct::Json::object();
    SceneSerializer::writeInstance(instance, entry, result, instance->getName());
    if (keepTransform) forceTransform(entry, instance);
    return entry;
}

Node* Prefab::replaceInPlace(Node* node, const ct::Json& entry, SceneResult& result)
{
    Node* parent = node->getParent();
    if (!parent)
    {
        result.addError(node->getName(),
                "the instance has no parent node to be rebuilt under; it was not rebuilt");
        return nullptr;
    }

    SceneSerializer::ReadContext context;
    Node* fresh = SceneSerializer::readInstance(entry, node->getName(), context, result);
    if (fresh && fresh->castTo<MissingNode>())
    {
        NodeFactory::destroy(fresh);
        fresh = nullptr;
    }
    if (!fresh)
    {
        result.addError(node->getName(),
                "the instance could not be rebuilt and was kept as it was");
        return nullptr;
    }

    const std::size_t index = static_cast<std::size_t>(node->getIndex());
    Node* owner = node->getOwner();
    parent->removeChild(node);
    delete node;
    parent->addChild(fresh, true);
    parent->moveChild(fresh, index);
    if (owner) fresh->setOwner(owner);
    SceneSerializer::resolvePending(context, result);
    return fresh;
}

std::size_t Prefab::reloadInstances(const ct::String& path, Node* searchRoot, SceneResult& result)
{
    if (!searchRoot) return 0;
    const ct::String file = FileSystem::simplifyPath(path);

    ct::Vector<Node*> candidates;
    ct::Vector<ct::String> related;
    ct::Vector<ct::String> unrelated;
    collectCandidates(searchRoot, file, candidates, related, unrelated);

    ct::Vector<ct::Json> entries;
    {
        BaseScope scope;
        for (std::size_t i = 0; i < candidates.size(); ++i)
            entries.push_back(makeEntry(candidates[i], true, result));
    }

    if (!refresh(file, result)) return 0;

    std::size_t rebuilt = 0;
    for (std::size_t i = 0; i < candidates.size(); ++i)
        if (replaceInPlace(candidates[i], entries[i], result)) ++rebuilt;
    return rebuilt;
}

Node* Prefab::revert(Node* instance, SceneResult& result, bool keepTransform)
{
    if (!instance || !instance->isInstance())
    {
        result.addError(ct::String(), "the node is not a prefab instance");
        return nullptr;
    }
    ct::Json entry = ct::Json::object();
    entry["instance"] = ct::Json(instance->getSceneFilePath());
    entry["name"] = ct::Json(instance->getName());
    if (keepTransform) forceTransform(entry, instance);
    return replaceInPlace(instance, entry, result);
}

Node* Prefab::apply(Node* instance, SceneResult& result)
{
    if (!instance || !instance->isInstance())
    {
        result.addError(ct::String(), "the node is not a prefab instance");
        return nullptr;
    }
    if (!instance->getParent())
    {
        result.addError(instance->getName(), "the instance has no parent node to be rebuilt under");
        return nullptr;
    }
    const ct::String file = instance->getSceneFilePath();

    Node* base = buildBase(file);
    if (!base)
    {
        result.addError(file, "the prefab could not be built to know what it holds now");
        return nullptr;
    }

    if (!checkCycle(instance, file, true, result))
    {
        delete base;
        return nullptr;
    }

    Node* searchRoot = topOf(instance);
    const ct::String where = searchRoot->getPathTo(instance);

    ct::Vector<Node*> candidates;
    ct::Vector<ct::String> related;
    ct::Vector<ct::String> unrelated;
    collectCandidates(searchRoot, file, candidates, related, unrelated);

    ct::Vector<ct::Json> entries;
    ct::Json rootJson = ct::Json::object();
    {
        BaseScope scope;
        for (std::size_t i = 0; i < candidates.size(); ++i)
        {
            if (candidates[i] == instance)
            {
                ct::Json entry = ct::Json::object();
                entry["instance"] = ct::Json(file);
                entry["name"] = ct::Json(instance->getName());
                forceTransform(entry, instance);
                entries.push_back(entry);
            }
            else
                entries.push_back(makeEntry(candidates[i], true, result));
        }

        SceneSerializer::writeNode(instance, rootJson, result, "root", true);
    }
    rootJson["name"] = ct::Json(base->getName());
    ct::Json& properties = rootJson["properties"];
    if (!properties.is_object()) properties = ct::Json::object();
    const ct::String className = instance->getClassName();
    for (const char* name: kTransform)
    {
        PropertyValue value;
        PropertyValue defaultValue;
        if (!NodeClassDB::getProperty(base, name, value) ||
                !NodeClassDB::getDefaultPropertyValue(className, name, defaultValue))
            continue;
        if (value == defaultValue) properties.erase(name);
        else
            properties.set(name, scenejson::fromValue(value));
    }
    if (properties.size() == 0) rootJson.erase("properties");
    delete base;

    const ct::String text = SceneSerializer::documentText(rootJson);
    if (!writeFile(file, text, result)) return nullptr;
    if (!store(file, text, result)) return nullptr;

    for (std::size_t i = 0; i < candidates.size(); ++i)
        replaceInPlace(candidates[i], entries[i], result);

    if (where.empty()) return nullptr;
    return searchRoot->getNode(where);
}

} // namespace prisma3d
