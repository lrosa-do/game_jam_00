#pragma once

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/SceneSerializer.h"

#include <ct/json.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

class Prefab
{
public:
    static constexpr const char* kExtension = ".prefab.json";

    static bool saveFromNode(const Node* root, const ct::String& path, SceneResult& result,
            Node* reloadRoot = nullptr);
    static Node* instantiate(const ct::String& path, Node* parent, SceneResult& result);
    static Node* apply(Node* instance, SceneResult& result);
    static Node* revert(Node* instance, SceneResult& result, bool keepTransform = true);
    static std::size_t reloadInstances(const ct::String& path, Node* searchRoot,
            SceneResult& result);

    static void findInstances(Node* searchRoot, const ct::String& path, ct::Vector<Node*>& out);
    static bool dependsOn(const ct::String& path, const ct::String& target,
            ct::Vector<ct::String>* chain = nullptr);

    static bool isCached(const ct::String& path);
    static void forget(const ct::String& path);
    static void clearCache();
    static std::size_t fileReadCount();

private:
    friend class SceneSerializer;

    class BaseScope
    {
    public:
        BaseScope();
        ~BaseScope();
        BaseScope(const BaseScope&) = delete;
        BaseScope& operator=(const BaseScope&) = delete;

        const Node* get(const ct::String& file);

    private:
        bool mOwner;
    };

    static ct::Rc<ct::Json> acquire(const ct::String& file, SceneResult& result,
            const ct::String& path, bool missingIsWarning);
    static bool reaches(const ct::String& from, const ct::String& target,
            ct::Vector<ct::String>& visited, ct::Vector<ct::String>& chain);
    static bool refresh(const ct::String& file, SceneResult& result);
    static bool store(const ct::String& file, const ct::String& text, SceneResult& result);
    static bool writeFile(const ct::String& file, const ct::String& text, SceneResult& result);
    static Node* buildBase(const ct::String& file);
    static void claimContent(Node* instance);
    static bool checkCycle(const Node* root, const ct::String& file, bool skipRoot,
            SceneResult& result);
    static void collectCandidates(Node* node, const ct::String& file, ct::Vector<Node*>& out,
            ct::Vector<ct::String>& related, ct::Vector<ct::String>& unrelated);
    static ct::Json makeEntry(const Node* instance, bool keepTransform, SceneResult& result);
    static Node* replaceInPlace(Node* node, const ct::Json& entry, SceneResult& result);
};

} // namespace prisma3d
