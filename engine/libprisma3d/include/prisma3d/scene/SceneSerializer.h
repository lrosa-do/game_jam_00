#pragma once

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace ct
{
class Json;
}

namespace prisma3d
{

class MissingNode;

enum class SceneDiagnosticSeverity : std::uint8_t
{
    Warning,
    Error
};

struct SceneDiagnostic
{
    SceneDiagnosticSeverity severity = SceneDiagnosticSeverity::Warning;
    ct::String path;
    ct::String message;
};

struct SceneResult
{
    ct::Vector<SceneDiagnostic> diagnostics;

    bool success() const;
    std::size_t errorCount() const;
    std::size_t warningCount() const;
    void addWarning(const ct::String& path, const ct::String& message);
    void addError(const ct::String& path, const ct::String& message);
    ct::String describe() const;
};

class SceneSerializer
{
public:
    static constexpr const char* kFormatName = "prisma3d-scene";
    static constexpr std::uint32_t kFormatVersion = 1;

    static bool save(const Node* root, ct::String& text, SceneResult* result = nullptr);
    static bool saveFile(const Node* root, const ct::String& path, SceneResult* result = nullptr);
    static Node* load(const ct::String& text, Node* parent, SceneResult& result);
    static Node* loadFile(const ct::String& path, Node* parent, SceneResult& result);

private:
    friend class Prefab;

    struct PendingPath
    {
        Node* node = nullptr;
        ct::String property;
        PropertyValue value;
        ct::String path;
    };

    struct ReadContext
    {
        ct::Vector<PendingPath> pending;
        ct::Vector<ct::String> prefabStack;
    };

    static bool parseDocument(const ct::String& text, ct::Json& document, SceneResult& result);
    static Node* loadText(const ct::String& text, Node* parent, SceneResult& result,
            const ct::String& file);
    static Node* readNode(const ct::Json& json, const ct::String& path, ReadContext& context,
            SceneResult& result);
    static Node* readInstance(const ct::Json& json, const ct::String& path, ReadContext& context,
            SceneResult& result);
    static Node* readMissing(const ct::Json& json, const ct::String& originalClass,
            const ct::String& originalScene, const ct::String& path, ReadContext& context,
            SceneResult& result);
    static void readChildren(Node* parent, const ct::Json& json, const ct::String& path,
            ReadContext& context, SceneResult& result);
    static void attachChild(Node* parent, Node* child, const ct::Json& json, const ct::String& path,
            SceneResult& result);
    static void applyGroups(Node* node, const ct::Json& json, const ct::String& path,
            SceneResult& result);
    static void applyProperties(Node* node, const ct::Json& properties, const ct::String& path,
            ReadContext& context, SceneResult& result);
    static void resolvePending(ReadContext& context, SceneResult& result);
    static ct::String documentText(const ct::Json& root);
    static void writeNode(const Node* node, ct::Json& out, SceneResult& result,
            const ct::String& path, bool plain);
    static void writeInstance(const Node* node, ct::Json& out, SceneResult& result,
            const ct::String& path);
    static void writeMissing(const MissingNode* node, ct::Json& out, SceneResult& result,
            const ct::String& path);
    static ct::Json writeChildren(const Node* node, SceneResult& result, const ct::String& path);
};

} // namespace prisma3d
