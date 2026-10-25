#pragma once

#include "prisma3d/editor/McpServer.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/json.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class AssetsPanel;
class EditorDocument;
class EditorPlay;
class EditorSelection;
class EditorShell;
class EditorViewportPanel;
class GamePanel;
class HierarchyActions;
class MaterialPanel;
class Node;
class SceneTree;

struct McpContext
{
    SceneTree& tree;
    EditorDocument& document;
    EditorSelection& selection;
    HierarchyActions& actions;
    EditorViewportPanel& viewport;
    GamePanel& game;
    EditorPlay& play;
    MaterialPanel& materials;
    AssetsPanel& assets;
    EditorShell& shell;
};

class McpTools
{
public:
    static constexpr const char* kUndoPrefix = "MCP: ";
    static constexpr const char* kScriptFolder = "Scripts";
    static constexpr const char* kScriptExtension = ".py";
    static constexpr std::size_t kScriptLimit = 256 * 1024;
    static constexpr int kDefaultDepth = 12;
    static constexpr int kDefaultImageWidth = 1024;
    static constexpr int kDefaultConsoleLines = 60;

    explicit McpTools(const McpContext& context);
    McpTools(const McpTools&) = delete;
    McpTools& operator=(const McpTools&) = delete;

    void registerTools(McpServer& server);

    static ct::Json describeProperty(const PropertyInfo& info);
    static bool convertValue(const PropertyInfo& info, const ct::Json& json, PropertyValue& out,
            ct::String& error);
    static ct::Json valueToJson(const PropertyInfo& info, const PropertyValue& value);

private:
    using Method = void (McpTools::*)(const ct::Json&, McpToolResult&);

    struct Entry
    {
        PropertyInfo info;
        ct::String group;
    };

    struct Pending
    {
        PropertyInfo info;
        PropertyValue before;
        PropertyValue after;
    };

    void add(McpServer& server, const char* name, const char* description, const ct::Json& schema,
            McpPermission level, Method method);

    void getSceneTree(const ct::Json& arguments, McpToolResult& result);
    void getNode(const ct::Json& arguments, McpToolResult& result);
    void listClasses(const ct::Json& arguments, McpToolResult& result);
    void listAssets(const ct::Json& arguments, McpToolResult& result);
    void getSelection(const ct::Json& arguments, McpToolResult& result);
    void getProjectSettings(const ct::Json& arguments, McpToolResult& result);
    void getConsole(const ct::Json& arguments, McpToolResult& result);
    void screenshot(const ct::Json& arguments, McpToolResult& result);

    void createNode(const ct::Json& arguments, McpToolResult& result);
    void setProperties(const ct::Json& arguments, McpToolResult& result);
    void deleteNode(const ct::Json& arguments, McpToolResult& result);
    void renameNode(const ct::Json& arguments, McpToolResult& result);
    void reparentNode(const ct::Json& arguments, McpToolResult& result);
    void duplicateNode(const ct::Json& arguments, McpToolResult& result);
    void createPrimitive(const ct::Json& arguments, McpToolResult& result);
    void createMaterial(const ct::Json& arguments, McpToolResult& result);
    void assignMaterial(const ct::Json& arguments, McpToolResult& result);
    void importAsset(const ct::Json& arguments, McpToolResult& result);
    void newScene(const ct::Json& arguments, McpToolResult& result);
    void openScene(const ct::Json& arguments, McpToolResult& result);
    void saveScene(const ct::Json& arguments, McpToolResult& result);
    void selectNode(const ct::Json& arguments, McpToolResult& result);
    void createEnvironment(const ct::Json& arguments, McpToolResult& result);

    void writeScript(const ct::Json& arguments, McpToolResult& result);
    void attachScript(const ct::Json& arguments, McpToolResult& result);
    void play(const ct::Json& arguments, McpToolResult& result);
    void stop(const ct::Json& arguments, McpToolResult& result);

    Node* scene(McpToolResult& result);
    bool editable(McpToolResult& result);
    bool projectOpen(McpToolResult& result);
    Node* findNode(const ct::String& path);
    Node* requireNode(const ct::Json& arguments, const char* key, bool defaultRoot,
            McpToolResult& result);
    ct::String pathOf(const Node* node) const;
    ct::Json describeNode(const Node* node) const;
    ct::Json sceneInfo() const;
    ct::Json treeJson(const Node* node, int depth) const;
    void collectProperties(const Node* node, ct::Vector<Entry>& out) const;
    bool parseProperties(const Node* node, const ct::String& className, const ct::Json& properties,
            ct::Vector<Pending>& out, ct::String& error) const;
    void reportApplied(const Node* node, const ct::Vector<Pending>& pending,
            ct::Json& output) const;
    Node* makeNode(Node* parent, const ct::String& className, const ct::String& name,
            const ct::Json* properties, const char* action, ct::Json& output, ct::String& error);
    void select(Node* node);
    ct::String failureText(const ct::String& error, const char* fallback) const;
    bool normalizeProjectPath(const ct::String& path, ct::String& out, ct::String& error) const;
    ct::String scriptRoot() const;
    ct::String materialReference(const ct::String& path, ct::String& error) const;
    void finishAssets();

    McpContext mContext;
    const McpServer* mServer;
};

} // namespace prisma3d
