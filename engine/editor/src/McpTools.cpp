#include "prisma3d/editor/McpTools.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/AssetImport.h"
#include "prisma3d/editor/AssetTypes.h"
#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/CreateMenu.h"
#include "prisma3d/editor/CreateNodeDialog.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorLog.h"
#include "prisma3d/editor/EditorPlay.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/GamePanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/MaterialPanel.h"
#include "prisma3d/editor/MaterialProperties.h"
#include "prisma3d/editor/NodeNaming.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/MaterialLibrary.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/script/ZenScripting.h"

#include <igui/widgets/Widget.hpp>

#include <ct/sort.hpp>

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const double kDegrees = 57.29577951308232;
const double kRadians = 0.017453292519943295;
const char* const kScreenshotFile = "prisma3d_mcp_screenshot.png";
const char* const kPrimitiveKinds[] = { "box", "plane", "sphere", "cylinder", "cone", "capsule",
    "torus" };

class Schema
{
public:
    Schema& text(const char* name, const char* description, bool required = false)
    {
        return add(name, "string", description, required);
    }

    Schema& integer(const char* name, const char* description, bool required = false)
    {
        return add(name, "integer", description, required);
    }

    Schema& number(const char* name, const char* description, bool required = false)
    {
        return add(name, "number", description, required);
    }

    Schema& boolean(const char* name, const char* description, bool required = false)
    {
        return add(name, "boolean", description, required);
    }

    Schema& object(const char* name, const char* description, bool required = false)
    {
        return add(name, "object", description, required);
    }

    Schema& list(const char* name, const char* itemType, const char* description,
            bool required = false)
    {
        add(name, "array", description, required);
        ct::Json items = ct::Json::object();
        items["type"] = itemType;
        mProperties[name]["items"] = items;
        return *this;
    }

    Schema& choice(const char* name, const char* const* values, std::size_t count,
            const char* description, bool required = false)
    {
        add(name, "string", description, required);
        ct::Json names = ct::Json::array();
        for (std::size_t i = 0; i < count; ++i) names.push_back(ct::Json(values[i]));
        mProperties[name]["enum"] = names;
        return *this;
    }

    ct::Json build() const
    {
        ct::Json schema = ct::Json::object();
        schema["type"] = "object";
        schema["properties"] = mProperties;
        if (mRequired.size() > 0) schema["required"] = mRequired;
        schema["additionalProperties"] = false;
        return schema;
    }

private:
    Schema& add(const char* name, const char* type, const char* description, bool required)
    {
        ct::Json property = ct::Json::object();
        property["type"] = type;
        property["description"] = description;
        mProperties[name] = property;
        if (required) mRequired.push_back(ct::Json(name));
        return *this;
    }

    ct::Json mProperties = ct::Json::object();
    ct::Json mRequired = ct::Json::array();
};

bool stringArgument(const ct::Json& arguments, const char* key, ct::String& out)
{
    const ct::Json* field = arguments.find(key);
    if (!field || !field->is_string()) return false;
    out = field->str();
    return true;
}

bool boolArgument(const ct::Json& arguments, const char* key, bool fallback)
{
    const ct::Json* field = arguments.find(key);
    return field && field->is_bool() ? field->as_bool() : fallback;
}

long long intArgument(const ct::Json& arguments, const char* key, long long fallback,
        long long minimum, long long maximum)
{
    const ct::Json* field = arguments.find(key);
    long long value = fallback;
    if (field && field->is_number()) value = static_cast<long long>(field->as_double());
    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    return value;
}

ct::String lowered(const ct::String& text)
{
    ct::String result = text;
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        char& c = result.data()[i];
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + 32);
    }
    return result;
}

bool hexDigit(char c, int& value)
{
    if (c >= '0' && c <= '9') value = c - '0';
    else if (c >= 'a' && c <= 'f')
        value = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
        value = c - 'A' + 10;
    else
        return false;
    return true;
}

bool parseHexColor(const ct::String& text, Math::Vec4& out)
{
    if (text.empty() || text[0] != '#') return false;
    const std::size_t digits = text.size() - 1;
    if (digits != 3 && digits != 4 && digits != 6 && digits != 8) return false;
    float channel[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    const std::size_t width = digits <= 4 ? 1 : 2;
    for (std::size_t c = 0; c < digits / width; ++c)
    {
        int value = 0;
        for (std::size_t d = 0; d < width; ++d)
        {
            int nibble = 0;
            if (!hexDigit(text[1 + c * width + d], nibble)) return false;
            value = value * 16 + nibble;
            if (width == 1) value = value * 16 + nibble;
        }
        channel[c] = static_cast<float>(value) / 255.0f;
    }
    out = Math::Vec4(channel[0], channel[1], channel[2], channel[3]);
    return true;
}

bool projectRelative(const ct::String& path)
{
    if (path.starts_with("/") || path.starts_with("\\")) return false;
    if (path.size() >= 2 && path[1] == ':' &&
            ((path[0] >= 'a' && path[0] <= 'z') || (path[0] >= 'A' && path[0] <= 'Z')))
        return false;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= path.size(); ++i)
    {
        if (i < path.size() && path[i] != '/' && path[i] != '\\') continue;
        if (path.substr(start, i - start) == "..") return false;
        start = i + 1;
    }
    return true;
}

const char* hintName(PropertyHint hint)
{
    switch (hint)
    {
        case PropertyHint::Range:
            return "range";
        case PropertyHint::Enum:
            return "enum";
        case PropertyHint::Flags:
            return "flags";
        case PropertyHint::Layers3DRender:
            return "render_layers";
        case PropertyHint::Layers3DPhysics:
            return "physics_layers";
        case PropertyHint::File:
            return "file";
        case PropertyHint::ResourceType:
            return "resource";
        case PropertyHint::ColorNoAlpha:
            return "color_no_alpha";
        case PropertyHint::NodePathValidTypes:
            return "node_path";
        case PropertyHint::GroupEnable:
            return "group_enable";
        default:
            return "";
    }
}

bool closeEnough(const PropertyValue& a, const PropertyValue& b)
{
    if (a == b) return true;
    if (a.type() != b.type()) return false;
    const auto closeTo = [](double x, double y)
    {
        const double scale = fabs(x) > fabs(y) ? fabs(x) : fabs(y);
        return fabs(x - y) <= 1e-4 * (scale > 1.0 ? scale : 1.0);
    };
    switch (a.type())
    {
        case PropertyType::Float:
            return closeTo(a.asFloat(), b.asFloat());
        case PropertyType::Vec2:
            return closeTo(a.asVec2().x, b.asVec2().x) && closeTo(a.asVec2().y, b.asVec2().y);
        case PropertyType::Vec3:
            return closeTo(a.asVec3().x, b.asVec3().x) && closeTo(a.asVec3().y, b.asVec3().y) &&
                   closeTo(a.asVec3().z, b.asVec3().z);
        case PropertyType::Vec4:
            return closeTo(a.asVec4().x, b.asVec4().x) && closeTo(a.asVec4().y, b.asVec4().y) &&
                   closeTo(a.asVec4().z, b.asVec4().z) && closeTo(a.asVec4().w, b.asVec4().w);
        case PropertyType::Color:
            return closeTo(a.asColor().x, b.asColor().x) && closeTo(a.asColor().y, b.asColor().y) &&
                   closeTo(a.asColor().z, b.asColor().z) && closeTo(a.asColor().w, b.asColor().w);
        case PropertyType::Quat:
            return closeTo(a.asQuat().x, b.asQuat().x) && closeTo(a.asQuat().y, b.asQuat().y) &&
                   closeTo(a.asQuat().z, b.asQuat().z) && closeTo(a.asQuat().w, b.asQuat().w);
        default:
            return false;
    }
}

struct PrefixScope
{
    explicit PrefixScope(EditorUndoRedoManager& manager)
            : undo(manager),
              previous(manager.namePrefix())
    {
        undo.setNamePrefix(McpTools::kUndoPrefix);
    }

    ~PrefixScope()
    {
        undo.setNamePrefix(previous);
    }

    EditorUndoRedoManager& undo;
    ct::String previous;
};

struct ScaledImage
{
    ScaledImage() = default;
    ScaledImage(const ScaledImage&) = delete;
    ScaledImage& operator=(const ScaledImage&) = delete;

    ~ScaledImage()
    {
        delete pixmap;
    }

    Pixmap* pixmap = nullptr;
};

struct ListedItem
{
    ct::String name;
    bool directory = false;
    std::uint64_t size = 0;
};

bool collectListed(const char* path, PathType type, void* user)
{
    const ct::String name = AssetImport::nameOf(ct::String(path));
    if (name.empty() || name[0] == '.') return true;
    const bool directory = type == PATH_TYPE_DIRECTORY;
    if (!directory && type != PATH_TYPE_FILE) return true;
    ListedItem item;
    item.name = name;
    item.directory = directory;
    if (!directory)
    {
        PathInfo info;
        if (fs_get_path_info(path, &info) && info.size > 0)
            item.size = static_cast<std::uint64_t>(info.size);
    }
    static_cast<ct::Vector<ListedItem>*>(user)->push_back(item);
    return true;
}

ct::String levelName(int level)
{
    switch (level)
    {
        case LOGLEVEL_DEBUG:
            return "debug";
        case LOGLEVEL_WARN:
            return "warning";
        case LOGLEVEL_ERROR:
        case LOGLEVEL_OFF:
            return "error";
        default:
            return "info";
    }
}

} // namespace

McpTools::McpTools(const McpContext& context)
        : mContext(context),
          mServer(nullptr)
{
}

void McpTools::add(McpServer& server, const char* name, const char* description,
        const ct::Json& schema, McpPermission level, Method method)
{
    server.addTool(name, description, schema, level,
            [this, method, level](const ct::Json& arguments, McpToolResult& result)
            {
                if (level == McpPermission::Read)
                {
                    (this->*method)(arguments, result);
                    return;
                }
                PrefixScope scope(mContext.document.undoRedo());
                (this->*method)(arguments, result);
            });
}

void McpTools::registerTools(McpServer& server)
{
    mServer = &server;
    static const char* const views[] = { "scene", "game" };

    add(server, "get_scene_tree",
            "Returns the node tree of the scene being edited: name, class, path, script and "
            "children "
            "of every node, plus the scene state (file, unsaved changes, playing) and the current "
            "selection. Paths are relative to the scene root; '.' is the root.",
            Schema().text("path", "Subtree to return. Default '.', the whole scene.")
                    .integer("max_depth", "How many levels to return. Default 12.")
                    .build(),
            McpPermission::Read, &McpTools::getSceneTree);
    add(server, "get_node",
            "Returns one node with its editor-visible properties and their current values. Angles "
            "are in degrees, colours are [r, g, b, a] in 0..1, vectors are arrays, enums are "
            "integers with their names in 'enum' and the current name in 'label'.",
            Schema().text("path", "Node path relative to the scene root, '.' for the root.", true)
                    .build(),
            McpPermission::Read, &McpTools::getNode);
    add(server, "list_classes",
            "Lists the node classes that can be created. Pass 'class' to get one class with its "
            "parent chain and all of its editable properties (type, range, enum names).",
            Schema().text("class",
                            "Class name for the full property list, for example 'OmniLight3D'.")
                    .text("filter", "Only list classes whose name contains this text.")
                    .build(),
            McpPermission::Read, &McpTools::listClasses);
    add(server, "list_assets",
            "Lists the files and folders of the open project. Paths are relative to the project "
            "root, for example 'Assets/Materials/Orange.mat'.",
            Schema().text("folder", "Project-relative folder. Default is the asset root.").build(),
            McpPermission::Read, &McpTools::listAssets);
    add(server, "get_selection", "Returns the nodes selected in the editor.", Schema().build(),
            McpPermission::Read, &McpTools::getSelection);
    add(server, "get_project_settings",
            "Returns the open project (name, folders, scenes, main scene) and its settings. Only "
            "settings that differ from their default are listed unless include_defaults is true.",
            Schema().text("filter",
                            "Only settings whose name contains this text, for example 'display/'.")
                    .boolean("include_defaults",
                            "Also list settings that have their default value.")
                    .build(),
            McpPermission::Read, &McpTools::getProjectSettings);
    add(server, "get_console",
            "Returns the latest editor console lines (engine messages and script output, including "
            "script errors with file and line).",
            Schema().integer("lines", "How many lines, newest last. Default 60.").build(),
            McpPermission::Read, &McpTools::getConsole);
    add(server, "screenshot",
            "Returns a PNG image of the Scene view (editor camera, with grid and gizmos) or of the "
            "Game view (the scene camera). The view must be visible in the editor; if its tab is "
            "hidden the editor switches to it and the call has to be repeated.",
            Schema().choice("view", views, 2, "'scene' (default) or 'game'.")
                    .integer("max_width", "Scale down to this width in pixels. Default 1024.")
                    .build(),
            McpPermission::Read, &McpTools::screenshot);

    add(server, "create_node",
            "Creates a node of any creatable class (see list_classes) under a parent and selects "
            "it. Properties are applied with the same names and units as get_node returns. The "
            "whole call is one undo step.",
            Schema().text("class", "Node class, for example 'Node3D', 'OmniLight3D', 'Camera3D'.",
                            true)
                    .text("parent_path", "Parent node path. Default is the scene root.")
                    .text("name", "Node name. Default is derived from the class.")
                    .object("properties",
                            "Property values by name, for example {\"position\": [0, 1, 0]}.")
                    .build(),
            McpPermission::Edit, &McpTools::createNode);
    add(server, "set_properties",
            "Sets one or more properties on a node in a single undo step. Values use the same "
            "units as get_node: angles in degrees (the 'rotation' property is [x, y, z] degrees), "
            "colours as [r, g, b, a] or '#rrggbb', enums as integer or name, resources and "
            "scripts as project-relative paths. Returns the values read back.",
            Schema().text("path", "Node path relative to the scene root.", true)
                    .object("properties", "Property values by name.", true)
                    .build(),
            McpPermission::Edit, &McpTools::setProperties);
    add(server, "delete_node", "Deletes a node and its children. The scene root cannot be deleted.",
            Schema().text("path", "Node path relative to the scene root.", true).build(),
            McpPermission::Edit, &McpTools::deleteNode);
    add(server, "rename_node", "Renames a node. Invalid characters (. : @ / \" %) are refused.",
            Schema().text("path", "Node path relative to the scene root.", true)
                    .text("name", "The new name.", true)
                    .build(),
            McpPermission::Edit, &McpTools::renameNode);
    add(server, "reparent_node", "Moves a node under another parent.",
            Schema().text("path", "Node to move.", true)
                    .text("new_parent_path", "The new parent.", true)
                    .boolean("keep_global_transform", "Keep the world transform. Default true.")
                    .build(),
            McpPermission::Edit, &McpTools::reparentNode);
    add(server, "duplicate_node",
            "Duplicates a node with its children next to the original and selects the copy.",
            Schema().text("path", "Node path relative to the scene root.", true).build(),
            McpPermission::Edit, &McpTools::duplicateNode);
    add(server, "create_primitive",
            "Creates a MeshInstance3D with a primitive mesh, as the Create menu does. 'parameters' "
            "overrides the mesh parameters: box {size:[x,y,z]}, plane {width, depth, segments_x, "
            "segments_z, uv_tiles}, sphere {radius, rings, slices}, cylinder and cone {radius, "
            "height, slices}, capsule {radius, height, rings, slices}, torus {major_radius, "
            "minor_radius, major_segments, minor_segments}. A plane lies flat on XZ.",
            Schema().choice("kind", kPrimitiveKinds, 7, "Primitive kind.", true)
                    .object("parameters", "Mesh parameters, see the description.")
                    .text("parent_path", "Parent node path. Default is the scene root.")
                    .text("name", "Node name. Default is the kind.")
                    .list("position", "number", "[x, y, z] in the parent space.")
                    .list("rotation", "number", "[x, y, z] degrees.")
                    .list("scale", "number", "[x, y, z].")
                    .build(),
            McpPermission::Edit, &McpTools::createPrimitive);
    add(server, "create_material",
            "Creates Assets/Materials/<name>.mat and fills it. 'parameters' uses the material "
            "property names of the Material panel: albedo_color [r,g,b,a] or '#rrggbb', metallic, "
            "metallic_specular, roughness (0..1), emission_enabled, emission [r,g,b], "
            "emission_energy_multiplier, transparency (0 opaque, 1 alpha scissor, 2 alpha blend), "
            "cull_mode, shading_mode, normal_scale, clearcoat_enabled, clearcoat, and texture "
            "properties (albedo_texture, normal_texture, orm_texture, ...) as project-relative "
            "image paths. Use assign_material to put it on a mesh.",
            Schema().text("name", "Material name, also the file name.", true)
                    .object("parameters", "Material properties by name.")
                    .build(),
            McpPermission::Edit, &McpTools::createMaterial);
    add(server, "assign_material",
            "Assigns a material file to a surface of a MeshInstance3D, or to its override that "
            "replaces every surface.",
            Schema().text("path", "MeshInstance3D node path.", true)
                    .text("material",
                            "Project-relative .mat path, for example "
                            "'Assets/Materials/Orange.mat'.",
                            true)
                    .integer("surface", "Surface index. Default 0.")
                    .boolean("override", "Assign to the material override instead of a surface.")
                    .build(),
            McpPermission::Edit, &McpTools::assignMaterial);
    add(server, "import_asset",
            "Copies a model (.gltf, .glb, .obj, ...), texture or audio file from the machine "
            "running the editor into the project. It does not add anything to the scene.",
            Schema().text("source", "Absolute path of the file to import.", true)
                    .text("folder",
                            "Project-relative destination folder. Default is the asset root.")
                    .boolean("overwrite", "Replace a file that already exists. Default false.")
                    .build(),
            McpPermission::Edit, &McpTools::importAsset);
    add(server, "new_scene",
            "Starts an empty scene. Refused while the current scene has unsaved changes unless "
            "discard_changes is true.",
            Schema().boolean("discard_changes", "Throw away unsaved changes. Default false.")
                    .build(),
            McpPermission::Edit, &McpTools::newScene);
    add(server, "open_scene",
            "Opens a scene of the project. Refused while the current scene has unsaved changes "
            "unless discard_changes is true.",
            Schema().text("path",
                            "Project-relative scene file, for example 'Scenes/main.scene.json', or "
                            "a "
                            "scene name such as 'main'.",
                            true)
                    .boolean("discard_changes", "Throw away unsaved changes. Default false.")
                    .build(),
            McpPermission::Edit, &McpTools::openScene);
    add(server, "save_scene",
            "Saves the scene. Pass 'name' for a scene that has no file yet, or to save under a new "
            "name inside the Scenes folder of the project.",
            Schema().text("name", "Scene name for Save As, without extension.").build(),
            McpPermission::Edit, &McpTools::saveScene);
    add(server, "select_node", "Selects nodes in the editor. An empty list clears the selection.",
            Schema().text("path", "One node path.")
                    .list("paths", "string", "Several node paths.")
                    .build(),
            McpPermission::Edit, &McpTools::selectNode);
    add(server, "create_environment",
            "Creates the WorldEnvironment node of the scene (background, sky, ambient light, "
            "tonemap, glow, fog). Property names: background_mode (0 clear colour, 1 custom, 2 "
            "sky), "
            "background_color, ambient_light_source, ambient_light_color, tonemap_mode, "
            "glow_enabled, fog_enabled, fog_mode, fog_light_color, fog_density and the other "
            "fog_*, sky_* properties; get_node on the new node lists them all.",
            Schema().object("properties", "WorldEnvironment properties by name.")
                    .text("name", "Node name. Default 'WorldEnvironment'.")
                    .text("parent_path", "Parent node path. Default is the scene root.")
                    .build(),
            McpPermission::Edit, &McpTools::createEnvironment);

    add(server, "write_script",
            "Writes a script file under Assets/Scripts of the project (creating or replacing it). "
            "A script has one class that derives from the class of the node it is attached to, "
            "with optional hooks _ready(self), _process(self, delta), _physics_process(self, "
            "delta), _input(self, event). Class variables become node properties. Example:\n"
            "class Rotate(MeshInstance3D):\n    speed = 90.0\n\n    def _process(self, delta):\n"
            "        self.rotation_degrees.y += self.speed * delta\n"
            "Errors show in the console (get_console) when the game starts.",
            Schema().text("path", "File path such as 'Assets/Scripts/spin.py' or just 'spin.py'.",
                            true)
                    .text("source", "The full source text.", true)
                    .build(),
            McpPermission::Code, &McpTools::writeScript);
    add(server, "attach_script", "Attaches a script file of the project to a node.",
            Schema().text("node_path", "Node path relative to the scene root.", true)
                    .text("script_path", "Project-relative script path.", true)
                    .build(),
            McpPermission::Code, &McpTools::attachScript);
    add(server, "play",
            "Starts the game with the current scene in the editor (scripts run). While it runs the "
            "scene edit tools are refused. Returns any script errors.",
            Schema().build(), McpPermission::Code, &McpTools::play);
    add(server, "stop", "Stops the game and returns the scene to its edited state.",
            Schema().build(), McpPermission::Code, &McpTools::stop);
}

Node* McpTools::scene(McpToolResult& result)
{
    Node* root = mContext.document.scene();
    if (!root) result.fail("There is no scene. Use new_scene or open_scene first.");
    return root;
}

bool McpTools::editable(McpToolResult& result)
{
    if (!scene(result)) return false;
    if (mContext.document.playing() || mContext.play.playing())
    {
        result.fail("The game is running; the scene can't be edited. Call stop first.");
        return false;
    }
    return true;
}

bool McpTools::projectOpen(McpToolResult& result)
{
    if (mContext.document.project().isOpen()) return true;
    result.fail("There is no open project. Open or create one in the editor first.");
    return false;
}

Node* McpTools::findNode(const ct::String& requested)
{
    Node* root = mContext.document.scene();
    if (!root) return nullptr;
    ct::String path = requested.trimmed();
    if (path.empty() || path == "." || path == "/") return root;
    while (path.starts_with("./")) path = path.substr(2);
    Node* node = nullptr;
    if (path.starts_with("/"))
    {
        node = root->getNode(path);
        if (!node) path = path.substr(1);
    }
    if (!node) node = root->getNode(path);
    if (!node)
    {
        const ct::String prefix = root->getName() + "/";
        if (path == root->getName()) node = root;
        else if (path.starts_with(prefix))
            node = root->getNode(path.substr(prefix.size()));
    }
    if (!node || (node != root && !root->isAncestorOf(node))) return nullptr;
    return node;
}

Node* McpTools::requireNode(const ct::Json& arguments, const char* key, bool defaultRoot,
        McpToolResult& result)
{
    Node* root = scene(result);
    if (!root) return nullptr;
    ct::String path;
    if (!stringArgument(arguments, key, path))
    {
        if (defaultRoot) return root;
        result.fail(ct::String("Missing text argument '") + key + "'.");
        return nullptr;
    }
    Node* node = findNode(path);
    if (!node)
        result.fail(ct::String("No node at the path '") + path +
                    "'. Paths are relative to the scene root; use get_scene_tree to see them.");
    return node;
}

ct::String McpTools::pathOf(const Node* node) const
{
    const Node* root = mContext.document.scene();
    if (!root || node == root) return ".";
    return root->getPathTo(node);
}

ct::Json McpTools::describeNode(const Node* node) const
{
    ct::Json json = ct::Json::object();
    json["path"] = pathOf(node);
    json["name"] = node->getName();
    json["class"] = node->getClassName();
    return json;
}

ct::Json McpTools::sceneInfo() const
{
    const EditorDocument& document = mContext.document;
    ct::Json info = ct::Json::object();
    info["name"] = document.sceneName();
    info["file"] = document.scenePath();
    info["unsaved_changes"] = document.dirty();
    info["playing"] = document.playing();
    info["project_open"] = document.project().isOpen();
    return info;
}

ct::Json McpTools::treeJson(const Node* node, int depth) const
{
    ct::Json json = describeNode(node);
    if (!node->getScript().empty()) json["script"] = node->getScript();
    if (node->isInstance() && node != mContext.document.scene())
        json["instance_of"] = node->getSceneFilePath();
    if (const Node3D* spatial = node->castTo<Node3D>())
        if (!spatial->isVisible()) json["visible"] = false;
    if (mContext.selection.isSelected(node)) json["selected"] = true;

    ct::Json children = ct::Json::array();
    std::size_t hidden = 0;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        const Node* child = node->getChild(i);
        if (child->isInternal()) continue;
        if (depth <= 0)
        {
            ++hidden;
            continue;
        }
        children.push_back(treeJson(child, depth - 1));
    }
    if (children.size() > 0) json["children"] = children;
    if (hidden > 0) json["children_not_shown"] = static_cast<long long>(hidden);
    return json;
}

void McpTools::getSceneTree(const ct::Json& arguments, McpToolResult& result)
{
    Node* root = scene(result);
    if (!root) return;
    Node* start = root;
    ct::String path;
    if (stringArgument(arguments, "path", path))
    {
        start = findNode(path);
        if (!start)
        {
            result.fail(ct::String("No node at the path '") + path + "'.");
            return;
        }
    }
    const int depth = static_cast<int>(intArgument(arguments, "max_depth", kDefaultDepth, 0, 64));
    ct::Json output = ct::Json::object();
    output["scene"] = sceneInfo();
    output["tree"] = treeJson(start, depth);
    result.addText(output.dump());
}

void McpTools::collectProperties(const Node* node, ct::Vector<Entry>& out) const
{
    ct::Vector<ct::String> chain;
    for (ct::String name = node->getClassName(); !name.empty();
            name = NodeClassDB::getParentClass(name))
        chain.push_back(name);
    for (std::size_t c = chain.size(); c > 0; --c)
    {
        ct::Vector<PropertyInfo> list;
        NodeClassDB::getInstancePropertyList(node, chain[c - 1], list, true);
        ct::String group;
        for (std::size_t i = 0; i < list.size(); ++i)
        {
            const PropertyInfo& info = list[i];
            if (info.usage & PropertyUsage::Subgroup) continue;
            if (info.usage & PropertyUsage::Group)
            {
                group = info.name;
                continue;
            }
            if (!(info.usage & PropertyUsage::Editor)) continue;
            Entry entry;
            entry.info = info;
            entry.group = group;
            out.push_back(entry);
        }
    }
}

ct::Json McpTools::describeProperty(const PropertyInfo& info)
{
    ct::Json json = ct::Json::object();
    json["name"] = info.name;
    json["type"] = PropertyValue::typeName(info.type);
    if (info.hint != PropertyHint::None) json["hint"] = hintName(info.hint);
    if (info.hint == PropertyHint::Enum || info.hint == PropertyHint::Flags)
    {
        ct::Json names = ct::Json::array();
        for (std::size_t i = 0; i < info.enumCount(); ++i)
            names.push_back(ct::Json(info.enumName(i)));
        json[info.hint == PropertyHint::Enum ? "enum" : "flags"] = names;
    }
    else if (info.hint == PropertyHint::Range)
    {
        if (!info.rangeOrLess) json["min"] = info.rangeMin;
        if (!info.rangeOrGreater) json["max"] = info.rangeMax;
        if (info.rangeStep > 0.0) json["step"] = info.rangeStep;
    }
    else if (!info.hintString.empty())
        json["hint_string"] = info.hintString;
    if (info.radiansAsDegrees) json["unit"] = "degrees";
    return json;
}

ct::Json McpTools::valueToJson(const PropertyInfo& info, const PropertyValue& value)
{
    if (info.radiansAsDegrees)
    {
        if (value.type() == PropertyType::Float)
            return scenejson::fromValue(PropertyValue::fromFloat(value.asFloat() * kDegrees));
        if (value.type() == PropertyType::Vec3)
        {
            const Math::Vec3 radians = value.asVec3();
            return scenejson::fromValue(
                    PropertyValue::fromVec3(Math::Vec3(static_cast<float>(radians.x * kDegrees),
                            static_cast<float>(radians.y * kDegrees),
                            static_cast<float>(radians.z * kDegrees))));
        }
    }
    return scenejson::fromValue(value);
}

bool McpTools::convertValue(const PropertyInfo& info, const ct::Json& json, PropertyValue& out,
        ct::String& error)
{
    if (info.hint == PropertyHint::Enum && json.is_string())
    {
        const ct::String wanted = lowered(json.str().trimmed());
        for (std::size_t i = 0; i < info.enumCount(); ++i)
        {
            if (lowered(info.enumName(i).trimmed()) != wanted) continue;
            out = PropertyValue::fromInt(static_cast<long long>(i));
            return true;
        }
        error = ct::String("'") + json.str() + "' is not one of: " + info.hintString;
        return false;
    }
    if (info.type == PropertyType::Color)
    {
        Math::Vec4 color;
        if (json.is_string())
        {
            if (!parseHexColor(json.str(), color))
            {
                error = "expected [r, g, b, a] or a hex colour like '#ff8800'";
                return false;
            }
            out = PropertyValue::fromColor(color);
            return true;
        }
        Math::Vec3 rgb;
        if (json.is_array() && json.size() == 3 && scenejson::toVec3(json, rgb))
        {
            out = PropertyValue::fromColor(Math::Vec4(rgb.x, rgb.y, rgb.z, 1.0f));
            return true;
        }
    }
    if (!scenejson::toValue(json, info.type, out, error)) return false;
    if (info.type == PropertyType::ResourcePath && !projectRelative(out.asString()))
    {
        error = "resource paths must be relative to the project, without '..'";
        return false;
    }
    if (info.radiansAsDegrees)
    {
        if (out.type() == PropertyType::Float)
            out = PropertyValue::fromFloat(out.asFloat() * kRadians);
        else if (out.type() == PropertyType::Vec3)
        {
            const Math::Vec3 degrees = out.asVec3();
            out = PropertyValue::fromVec3(Math::Vec3(static_cast<float>(degrees.x * kRadians),
                    static_cast<float>(degrees.y * kRadians),
                    static_cast<float>(degrees.z * kRadians)));
        }
    }
    return true;
}

void McpTools::getNode(const ct::Json& arguments, McpToolResult& result)
{
    Node* node = requireNode(arguments, "path", false, result);
    if (!node) return;

    ct::Json output = describeNode(node);
    output["parent"] = node->getParent() && node != mContext.document.scene()
                               ? ct::Json(pathOf(node->getParent()))
                               : ct::Json();
    ct::Json children = ct::Json::array();
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (!node->getChild(i)->isInternal())
            children.push_back(ct::Json(node->getChild(i)->getName()));
    output["children"] = children;
    ct::Vector<Node::GroupInfo> groups;
    node->getGroups(groups);
    ct::Json groupNames = ct::Json::array();
    for (std::size_t i = 0; i < groups.size(); ++i)
        if (groups[i].persistent) groupNames.push_back(ct::Json(groups[i].name));
    if (groupNames.size() > 0) output["groups"] = groupNames;
    if (!node->getScript().empty()) output["script"] = node->getScript();
    if (node->isInstance() && node != mContext.document.scene())
        output["instance_of"] = node->getSceneFilePath();

    ct::Vector<Entry> entries;
    collectProperties(node, entries);
    ct::Json properties = ct::Json::array();
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        const PropertyInfo& info = entries[i].info;
        PropertyValue value;
        if (!NodeClassDB::getProperty(node, info.name, value)) continue;
        ct::Json property = describeProperty(info);
        if (!entries[i].group.empty()) property["group"] = entries[i].group;
        property["value"] = valueToJson(info, value);
        if (info.hint == PropertyHint::Enum)
            property["label"] = info.enumName(static_cast<std::size_t>(value.asInt()));
        properties.push_back(property);
    }
    output["properties"] = properties;
    result.addText(output.dump());
}

void McpTools::listClasses(const ct::Json& arguments, McpToolResult& result)
{
    ct::String className;
    if (stringArgument(arguments, "class", className))
    {
        if (!NodeClassDB::classExists(className))
        {
            result.fail(ct::String("There is no class '") + className + "'.");
            return;
        }
        ct::Json output = ct::Json::object();
        output["name"] = className;
        output["parent"] = NodeClassDB::getParentClass(className);
        output["creatable"] = !CreateNodeDialog::shouldHideType(className);
        ct::Json ancestors = ct::Json::array();
        for (ct::String name = NodeClassDB::getParentClass(className); !name.empty();
                name = NodeClassDB::getParentClass(name))
            ancestors.push_back(ct::Json(name));
        output["ancestors"] = ancestors;

        ct::Vector<ct::String> chain;
        for (ct::String name = className; !name.empty(); name = NodeClassDB::getParentClass(name))
            chain.push_back(name);
        ct::Json properties = ct::Json::array();
        for (std::size_t c = chain.size(); c > 0; --c)
        {
            ct::Vector<PropertyInfo> list;
            NodeClassDB::getPropertyList(chain[c - 1], list, true);
            ct::String group;
            for (std::size_t i = 0; i < list.size(); ++i)
            {
                const PropertyInfo& info = list[i];
                if (info.usage & PropertyUsage::Subgroup) continue;
                if (info.usage & PropertyUsage::Group)
                {
                    group = info.name;
                    continue;
                }
                if (!(info.usage & PropertyUsage::Editor)) continue;
                ct::Json property = describeProperty(info);
                if (!group.empty()) property["group"] = group;
                property["declared_in"] = chain[c - 1];
                PropertyValue fallback;
                if (NodeClassDB::getDefaultPropertyValue(className, info.name, fallback))
                    property["default"] = valueToJson(info, fallback);
                properties.push_back(property);
            }
        }
        output["properties"] = properties;
        result.addText(output.dump());
        return;
    }

    ct::String filter;
    stringArgument(arguments, "filter", filter);
    filter = lowered(filter);
    ct::Vector<ct::String> types;
    CreateNodeDialog::typeList(types);
    ct::Json list = ct::Json::array();
    for (std::size_t i = 0; i < types.size(); ++i)
    {
        if (!filter.empty() && !lowered(types[i]).contains(filter)) continue;
        ct::Json entry = ct::Json::object();
        entry["name"] = types[i];
        entry["parent"] = NodeClassDB::getParentClass(types[i]);
        list.push_back(entry);
    }
    ct::Json output = ct::Json::object();
    output["classes"] = list;
    result.addText(output.dump());
}

bool McpTools::normalizeProjectPath(const ct::String& path, ct::String& out,
        ct::String& error) const
{
    ct::String text = path.trimmed();
    for (std::size_t i = 0; i < text.size(); ++i)
        if (text.data()[i] == '\\') text.data()[i] = '/';
    while (text.ends_with("/")) text.pop_back();
    if (text.empty() || text == ".")
    {
        out = ct::String();
        return true;
    }
    return Project::normalizeRelativePath(text, out, error);
}

void McpTools::listAssets(const ct::Json& arguments, McpToolResult& result)
{
    if (!projectOpen(result)) return;
    const Project& project = mContext.document.project();
    ct::String folder = project.assetRoot();
    ct::String requested;
    ct::String error;
    if (stringArgument(arguments, "folder", requested) &&
            !normalizeProjectPath(requested, folder, error))
    {
        result.fail(ct::String("Bad folder: ") + error);
        return;
    }
    const ct::String absolute = folder.empty() ? project.root() : project.absolutePath(folder);
    if (!dir_exists(absolute.c_str()))
    {
        result.fail(ct::String("The folder '") + folder + "' does not exist.");
        return;
    }
    ct::Vector<ListedItem> items;
    fs_enumerate_directory(absolute.c_str(), false, &collectListed, &items);
    ct::sort(items.begin(), items.end(),
            [](const ListedItem& a, const ListedItem& b)
            {
                if (a.directory != b.directory) return a.directory;
                return a.name < b.name;
            });
    ct::Json list = ct::Json::array();
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        ct::Json entry = ct::Json::object();
        entry["name"] = items[i].name;
        entry["path"] = AssetImport::joinPath(folder, items[i].name);
        entry["kind"] = AssetTypes::label(AssetTypes::classify(items[i].name, items[i].directory));
        if (!items[i].directory) entry["size"] = static_cast<unsigned long long>(items[i].size);
        list.push_back(entry);
    }
    ct::Json output = ct::Json::object();
    output["folder"] = folder;
    output["entries"] = list;
    result.addText(output.dump());
}

void McpTools::getSelection(const ct::Json&, McpToolResult& result)
{
    ct::Json list = ct::Json::array();
    const ct::Vector<Node*>& selected = mContext.selection.selectedNodes();
    for (std::size_t i = 0; i < selected.size(); ++i) list.push_back(describeNode(selected[i]));
    ct::Json output = ct::Json::object();
    output["selection"] = list;
    result.addText(output.dump());
}

void McpTools::getProjectSettings(const ct::Json& arguments, McpToolResult& result)
{
    const Project& project = mContext.document.project();
    ct::Json output = ct::Json::object();
    output["scene"] = sceneInfo();
    if (!project.isOpen())
    {
        output["project"] = ct::Json();
        result.addText(output.dump());
        return;
    }
    ct::Json info = ct::Json::object();
    info["name"] = project.name();
    info["root"] = project.root();
    info["asset_root"] = project.assetRoot();
    info["main_scene"] = project.mainScene();
    ct::Json scenes = ct::Json::array();
    for (std::size_t i = 0; i < project.scenes().size(); ++i)
        scenes.push_back(ct::Json(project.scenes()[i]));
    info["scenes"] = scenes;
    output["project"] = info;

    ct::String filter;
    stringArgument(arguments, "filter", filter);
    const bool all = boolArgument(arguments, "include_defaults", false);
    ct::Json settings = ct::Json::array();
    for (std::size_t i = 0; i < Project::settingCount(); ++i)
    {
        const ProjectSettingInfo& setting = Project::settingInfo(i);
        if (!filter.empty() && !setting.name.contains(filter)) continue;
        if (!all && project.isDefault(setting.name)) continue;
        ct::Json entry = ct::Json::object();
        entry["name"] = setting.name;
        entry["type"] = PropertyValue::typeName(setting.type);
        entry["value"] = scenejson::fromValue(project.get(setting.name));
        entry["default"] = scenejson::fromValue(setting.defaultValue);
        settings.push_back(entry);
    }
    output["settings"] = settings;
    result.addText(output.dump());
}

void McpTools::getConsole(const ct::Json& arguments, McpToolResult& result)
{
    const std::size_t wanted =
            static_cast<std::size_t>(intArgument(arguments, "lines", kDefaultConsoleLines, 1, 500));
    EditorLog& log = EditorLog::instance();
    std::size_t cursor = log.total() > wanted ? log.total() - wanted : 0;
    ct::Vector<EditorLog::Entry> entries;
    log.read(cursor, entries);
    ct::Json list = ct::Json::array();
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        ct::Json line = ct::Json::object();
        line["level"] = levelName(entries[i].level);
        if (entries[i].script) line["script"] = true;
        line["text"] = entries[i].text;
        list.push_back(line);
    }
    ct::Json output = ct::Json::object();
    output["lines"] = list;
    result.addText(output.dump());
}

void McpTools::screenshot(const ct::Json& arguments, McpToolResult& result)
{
    ct::String view = "scene";
    stringArgument(arguments, "view", view);
    view = lowered(view);
    if (view != "scene" && view != "game")
    {
        result.fail("view must be 'scene' or 'game'.");
        return;
    }
    const bool game = view == "game";
    bool shown = true;
    for (const rt::Widget* widget = game ? mContext.game.view() : mContext.viewport.view(); widget;
            widget = widget->parent())
        shown = shown && widget->isVisible();
    if (!shown)
    {
        mContext.shell.showPanel(game ? EditorShell::kGamePanel : EditorShell::kScenePanel);
        result.fail(ct::String("The ") + (game ? "Game" : "Scene") +
                    " view was hidden, so it was not rendering. It is visible now; call screenshot "
                    "again in a moment.");
        return;
    }
    if (game && mContext.game.showingMessage())
    {
        result.fail(ct::String("The Game view shows: ") + mContext.game.message());
        return;
    }
    if (!(game ? mContext.game.rendering() : mContext.viewport.rendering()))
    {
        result.fail("The view has not rendered yet; call screenshot again in a moment.");
        return;
    }

    const TextureId texture = game ? mContext.game.texture() : mContext.viewport.texture();
    const int width = static_cast<int>(game ? mContext.game.width() : mContext.viewport.width());
    const int height = static_cast<int>(game ? mContext.game.height() : mContext.viewport.height());
    Device* device = Main::device();
    TextureStore* textures = Main::textures();
    if (width <= 0 || height <= 0 || !device || !device->driver() || !textures)
    {
        result.fail("The view has no image yet.");
        return;
    }
    prisma::RenderTarget source;
    source.texture = textures->gpu(texture);
    prisma::Rect area;
    area.x = 0;
    area.y = 0;
    area.width = static_cast<std::uint32_t>(width);
    area.height = static_cast<std::uint32_t>(height);
    ct::Vector<unsigned char> pixels;
    pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4);
    if (!device->driver()->readPixels(source, area, pixels.data()))
    {
        result.fail("Could not read the view back from the GPU.");
        return;
    }
    for (std::size_t i = 3; i < pixels.size(); i += 4) pixels[i] = 255;

    Pixmap image(width, height, 4, pixels.data());
    image.flip_vertical();
    const int maxWidth =
            static_cast<int>(intArgument(arguments, "max_width", kDefaultImageWidth, 64, 4096));
    ScaledImage scaled;
    Pixmap* output = &image;
    if (width > maxWidth)
    {
        const int scaledHeight = height * maxWidth / width > 0 ? height * maxWidth / width : 1;
        scaled.pixmap = image.resize(maxWidth, scaledHeight);
        if (scaled.pixmap) output = scaled.pixmap;
    }

    char folder[1024] = {};
    if (!fs_get_temp_path(folder, sizeof(folder)))
    {
        result.fail("There is no temporary folder to encode the image in.");
        return;
    }
    const ct::String file = ct::String(folder) + "/" + kScreenshotFile;
    const bool saved = output->save(file.c_str());
    std::size_t size = 0;
    std::uint8_t* bytes = saved ? file_read(file.c_str(), &size) : nullptr;
    fs_remove_path(file.c_str());
    if (!bytes)
    {
        result.fail("Could not encode the image.");
        return;
    }
    const ct::String encoded = scenejson::encodeBase64(bytes, size);
    fs_free(bytes);
    result.addImage(encoded, "image/png");
    ct::String note = view;
    note += " view ";
    note.append_number(output->width);
    note += 'x';
    note.append_number(output->height);
    result.addText(note);
}

void McpTools::select(Node* node)
{
    mContext.selection.clear();
    if (node) mContext.selection.add(node);
}

ct::String McpTools::failureText(const ct::String& error, const char* fallback) const
{
    return error.empty() ? ct::String(fallback) : error;
}

bool McpTools::parseProperties(const Node* node, const ct::String& className,
        const ct::Json& properties, ct::Vector<Pending>& out, ct::String& error) const
{
    if (!properties.is_object())
    {
        error = "'properties' must be an object of property values.";
        return false;
    }
    const ct::Json::Object& members = properties.members();
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        const ct::String& key = members[i].key;
        Pending pending;
        if (key == "script" && mServer && !mServer->allowCode())
        {
            error = ct::String("Attaching a script needs '") +
                    McpServer::permissionSwitch(McpPermission::Code) +
                    "' in the AI panel; it is turned off.";
            return false;
        }
        if (!NodeClassDB::getPropertyInfo(className, key, pending.info) || pending.info.isGroup())
        {
            error = ct::String("The class ") + className + " has no property '" + key +
                    "'. Use get_node or list_classes to see its properties.";
            return false;
        }
        ct::String reason;
        if (!convertValue(pending.info, members[i].value, pending.after, reason))
        {
            error = ct::String("Property '") + key + "': " + reason + ".";
            return false;
        }
        if (node && !NodeClassDB::getProperty(node, key, pending.before))
        {
            error = ct::String("The property '") + key + "' can't be read, so it can't be changed.";
            return false;
        }
        out.push_back(pending);
    }
    return true;
}

void McpTools::reportApplied(const Node* node, const ct::Vector<Pending>& pending,
        ct::Json& output) const
{
    ct::Json values = ct::Json::object();
    ct::Json warnings = ct::Json::array();
    for (std::size_t i = 0; i < pending.size(); ++i)
    {
        PropertyValue now;
        if (!NodeClassDB::getProperty(node, pending[i].info.name, now)) continue;
        values[pending[i].info.name] = valueToJson(pending[i].info, now);
        if (!closeEnough(now, pending[i].after))
            warnings.push_back(ct::Json(ct::String("'") + pending[i].info.name +
                                        "' reads back differently from the value sent; the node "
                                        "may have refused or adjusted it."));
    }
    output["properties"] = values;
    if (warnings.size() > 0) output["warnings"] = warnings;
}

Node* McpTools::makeNode(Node* parent, const ct::String& className, const ct::String& name,
        const ct::Json* properties, const char* action, ct::Json& output, ct::String& error)
{
    if (CreateNodeDialog::shouldHideType(className))
    {
        error = ct::String("Can't create a node of the class '") + className +
                "'. Use list_classes to see the classes that can be created.";
        return nullptr;
    }
    if (!name.empty() && Node::validateNodeName(name) != name)
    {
        error = HierarchyActions::kInvalidNameMessage;
        return nullptr;
    }
    ct::Vector<Pending> pending;
    if (properties && !parseProperties(nullptr, className, *properties, pending, error))
        return nullptr;

    const ct::Vector<Node*> selectionBefore = mContext.selection.selectedNodes();
    mContext.actions.takeEditable();
    EditorUndoRedoManager::Applying applying(&mContext.document.undoRedo());
    mContext.actions.allowChildren(parent);
    Node* node = parent->createChild(className.c_str(), true);
    if (!node)
    {
        mContext.actions.takeEditable();
        error = ct::String("Could not create a ") + className + ".";
        return nullptr;
    }
    if (!name.empty()) node->setName(name);
    for (std::size_t i = 0; i < pending.size(); ++i)
        NodeClassDB::setProperty(node, pending[i].info.name, pending[i].after);
    select(node);
    ct::Vector<Node*> created;
    created.push_back(node);
    mContext.actions.recordCreated(action, created, selectionBefore);
    reportApplied(node, pending, output);
    return node;
}

void McpTools::createNode(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    ct::String className;
    if (!stringArgument(arguments, "class", className))
    {
        result.fail("Missing text argument 'class'.");
        return;
    }
    Node* parent = requireNode(arguments, "parent_path", true, result);
    if (!parent) return;
    ct::String name;
    stringArgument(arguments, "name", name);
    const ct::Json* properties = arguments.find("properties");
    ct::Json output = ct::Json::object();
    ct::String error;
    Node* node = makeNode(parent, className, name, properties, "Create Node", output, error);
    if (!node)
    {
        result.fail(failureText(error, "The node could not be created."));
        return;
    }
    output["node"] = describeNode(node);
    result.addText(output.dump());
}

void McpTools::createEnvironment(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    Node* root = mContext.document.scene();
    ct::Vector<Node*> pending;
    pending.push_back(root);
    while (!pending.empty())
    {
        Node* node = pending.back();
        pending.pop_back();
        if (node->isClass("WorldEnvironment"))
        {
            result.fail(ct::String("The scene already has a WorldEnvironment at '") + pathOf(node) +
                        "'. Change it with set_properties.");
            return;
        }
        for (std::size_t i = 0; i < node->getChildCount(); ++i)
            pending.push_back(node->getChild(i));
    }
    Node* parent = requireNode(arguments, "parent_path", true, result);
    if (!parent) return;
    ct::String name = "WorldEnvironment";
    stringArgument(arguments, "name", name);
    const ct::Json* properties = arguments.find("properties");
    ct::Json output = ct::Json::object();
    ct::String error;
    Node* node = makeNode(parent, "WorldEnvironment", name, properties, "Create Environment",
            output, error);
    if (!node)
    {
        result.fail(failureText(error, "The environment could not be created."));
        return;
    }
    output["node"] = describeNode(node);
    result.addText(output.dump());
}

void McpTools::setProperties(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    Node* node = requireNode(arguments, "path", false, result);
    if (!node) return;
    const ct::Json* properties = arguments.find("properties");
    if (!properties || !properties->is_object() || properties->size() == 0)
    {
        result.fail("'properties' must be an object with at least one property.");
        return;
    }
    if (properties->find("name"))
    {
        result.fail("Use rename_node to change the name of a node.");
        return;
    }
    ct::Vector<Pending> pending;
    ct::String error;
    if (!parseProperties(node, node->getClassName(), *properties, pending, error))
    {
        result.fail(error);
        return;
    }

    EditorUndoRedoManager& undo = mContext.document.undoRedo();
    undo.createAction("Set Properties");
    for (std::size_t i = 0; i < pending.size(); ++i)
    {
        undo.addDoProperty(node, pending[i].info.name, pending[i].after);
        undo.addUndoProperty(node, pending[i].info.name, pending[i].before);
    }
    undo.commitAction();

    ct::Json output = ct::Json::object();
    output["node"] = describeNode(node);
    reportApplied(node, pending, output);
    result.addText(output.dump());
}

void McpTools::deleteNode(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    Node* node = requireNode(arguments, "path", false, result);
    if (!node) return;
    const ct::String path = pathOf(node);
    select(node);
    ct::String error;
    if (!mContext.actions.deleteSelection(error))
    {
        result.fail(failureText(error, "The node can't be deleted."));
        return;
    }
    ct::Json output = ct::Json::object();
    output["deleted"] = path;
    result.addText(output.dump());
}

void McpTools::renameNode(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    Node* node = requireNode(arguments, "path", false, result);
    if (!node) return;
    ct::String name;
    if (!stringArgument(arguments, "name", name))
    {
        result.fail("Missing text argument 'name'.");
        return;
    }
    ct::String error;
    if (!mContext.actions.rename(node, name, error))
    {
        result.fail(failureText(error, "The node can't be renamed."));
        return;
    }
    if (!error.empty())
    {
        result.fail(error);
        return;
    }
    ct::Json output = ct::Json::object();
    output["node"] = describeNode(node);
    result.addText(output.dump());
}

void McpTools::reparentNode(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    Node* node = requireNode(arguments, "path", false, result);
    if (!node) return;
    Node* parent = requireNode(arguments, "new_parent_path", false, result);
    if (!parent) return;
    select(node);
    ct::String error;
    if (!mContext.actions.reparentSelection(parent,
                boolArgument(arguments, "keep_global_transform", true), error))
    {
        result.fail(failureText(error,
                "Nothing was moved: the node is already the last child of that parent, or the "
                "move is not allowed."));
        return;
    }
    ct::Json output = ct::Json::object();
    output["node"] = describeNode(node);
    result.addText(output.dump());
}

void McpTools::duplicateNode(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    Node* node = requireNode(arguments, "path", false, result);
    if (!node) return;
    select(node);
    ct::String error;
    if (!mContext.actions.duplicateSelection(error))
    {
        result.fail(failureText(error, "The node can't be duplicated."));
        return;
    }
    const ct::Vector<Node*>& selected = mContext.selection.selectedNodes();
    ct::Json output = ct::Json::object();
    if (!selected.empty()) output["node"] = describeNode(selected[0]);
    result.addText(output.dump());
}

void McpTools::createPrimitive(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result)) return;
    ct::String kindName;
    stringArgument(arguments, "kind", kindName);
    kindName = lowered(kindName);
    int kind = -1;
    for (std::size_t i = 0; i < sizeof(kPrimitiveKinds) / sizeof(kPrimitiveKinds[0]); ++i)
        if (kindName == kPrimitiveKinds[i]) kind = static_cast<int>(i);
    if (kind < 0)
    {
        result.fail("kind must be one of: box, plane, sphere, cylinder, cone, capsule, torus.");
        return;
    }
    PrimitiveKind primitive = PrimitiveKind::Box;
    for (std::size_t i = 0; i < primitiveKindCount(); ++i)
        if (kindName == primitiveKindName(static_cast<PrimitiveKind>(i)))
            primitive = static_cast<PrimitiveKind>(i);

    PrimitiveRecipe recipe;
    primitiveDefaults(primitive, recipe);
    const ct::Json* parameters = arguments.find("parameters");
    if (parameters && !parameters->is_null())
    {
        if (!parameters->is_object())
        {
            result.fail("'parameters' must be an object.");
            return;
        }
        const ct::Json::Object& members = parameters->members();
        for (std::size_t m = 0; m < members.size(); ++m)
        {
            int value = 0;
            const PrimitiveParamDef* found = nullptr;
            for (std::size_t p = 0; p < primitiveParamCount(primitive) && !found; ++p)
            {
                const PrimitiveParamDef& def = primitiveParam(primitive, p);
                if (members[m].key == def.key) found = &def;
                else
                    value += def.type == PrimitiveParamType::Vec3 ? 3 : 1;
            }
            if (!found)
            {
                result.fail(ct::String("The ") + kindName + " has no parameter '" + members[m].key +
                            "'.");
                return;
            }
            const int count = found->type == PrimitiveParamType::Vec3 ? 3 : 1;
            float numbers[3] = {};
            bool ok = true;
            if (count == 3) ok = scenejson::toFloats(members[m].value, numbers, 3);
            else
                ok = scenejson::toFloat(members[m].value, numbers[0]);
            for (int c = 0; c < count && ok; ++c)
            {
                ok = numbers[c] >= found->minimum && numbers[c] <= found->maximum;
                if (ok && found->type == PrimitiveParamType::Count)
                    ok = numbers[c] == floorf(numbers[c]);
            }
            if (!ok)
            {
                result.fail(ct::String("Bad value for '") + members[m].key + "'.");
                return;
            }
            for (int c = 0; c < count; ++c) recipe.values[value + c] = numbers[c];
        }
    }
    ct::String error;
    PrimitiveRecipe checked;
    const ct::String meshPath = primitiveRecipePath(recipe);
    if (!parsePrimitiveRecipe(meshPath, checked, error))
    {
        result.fail(error);
        return;
    }

    Node* parent = requireNode(arguments, "parent_path", true, result);
    if (!parent) return;
    ct::String name;
    if (!stringArgument(arguments, "name", name))
    {
        name = kindName;
        for (int i = 0; i < CreateMenu::kItemCount; ++i)
        {
            const CreateMenu::Item item = static_cast<CreateMenu::Item>(i);
            if (CreateMenu::isPrimitive(item) && CreateMenu::primitiveKind(item) == primitive)
                name = CreateMenu::label(item);
        }
    }

    ct::Json properties = ct::Json::object();
    properties["mesh"] = meshPath;
    const char* const transform[] = { "position", "rotation", "scale" };
    for (const char* key: transform)
    {
        const ct::Json* value = arguments.find(key);
        if (value && !value->is_null()) properties[key] = *value;
    }
    ct::Json output = ct::Json::object();
    Node* node = makeNode(parent, "MeshInstance3D", name, &properties, "Create Primitive", output,
            error);
    if (!node)
    {
        result.fail(failureText(error, "The primitive could not be created."));
        return;
    }
    output["node"] = describeNode(node);
    output["mesh"] = meshPath;
    result.addText(output.dump());
}

void McpTools::finishAssets()
{
    mContext.assets.refresh();
}

ct::String McpTools::materialReference(const ct::String& path, ct::String& error) const
{
    if (path.contains("::")) return path;
    MaterialLibraryData data;
    const Project& project = mContext.document.project();
    if (!MaterialLibraryFormat::load(project.absolutePath(path), data, error)) return ct::String();
    if (data.materials.empty())
    {
        error = "The material file has no materials.";
        return ct::String();
    }
    return path + "::" + data.materials[0].info.name;
}

void McpTools::createMaterial(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result) || !projectOpen(result)) return;
    ct::String name;
    if (!stringArgument(arguments, "name", name))
    {
        result.fail("Missing text argument 'name'.");
        return;
    }
    name = name.trimmed();

    struct Write
    {
        const MaterialField* field = nullptr;
        PropertyValue value;
    };
    ct::Vector<Write> writes;
    const ct::Json* parameters = arguments.find("parameters");
    if (parameters && !parameters->is_null())
    {
        if (!parameters->is_object())
        {
            result.fail("'parameters' must be an object.");
            return;
        }
        const ct::Json::Object& members = parameters->members();
        for (std::size_t i = 0; i < members.size(); ++i)
        {
            Write write;
            write.field = MaterialProperties::find(members[i].key.c_str());
            if (!write.field)
            {
                result.fail(ct::String("Unknown material property '") + members[i].key + "'.");
                return;
            }
            ct::String reason;
            if (!convertValue(MaterialProperties::infoOf(*write.field), members[i].value,
                        write.value, reason))
            {
                result.fail(
                        ct::String("Material property '") + members[i].key + "': " + reason + ".");
                return;
            }
            writes.push_back(write);
        }
    }

    ct::String path;
    ct::String error;
    if (!mContext.materials.createLibrary(mContext.materials.materialsFolder(), name, path, error))
    {
        result.fail(failureText(error, "The material could not be created."));
        return;
    }
    sceneres::clearError();
    const MaterialId material = sceneres::loadMaterial(path);
    if (!material.valid())
    {
        result.fail(ct::String("The material file was written but could not be loaded: ") +
                    sceneres::takeError());
        return;
    }
    MaterialStore& store = Main::rendering()->materials();
    const MaterialInfo* info = store.get(material);
    MaterialFeatures features = MaterialFeatures::of(*info);
    ct::Json applied = ct::Json::array();
    ct::Json warnings = ct::Json::array();
    for (int pass = 0; pass < 2; ++pass)
    {
        for (std::size_t i = 0; i < writes.size(); ++i)
        {
            const bool isFeature = writes[i].field->feature != MaterialFeature::None;
            if (isFeature != (pass == 0)) continue;
            if (MaterialProperties::write(store, material, features, *writes[i].field,
                        writes[i].value))
                applied.push_back(ct::Json(writes[i].field->key));
            else
                warnings.push_back(ct::Json(ct::String("'") + writes[i].field->key +
                                            "' was not accepted (a missing texture, or a value the "
                                            "material refused)."));
        }
    }
    if (!mContext.materials.saveMaterial(material, error))
    {
        result.fail(failureText(error, "The material could not be saved."));
        return;
    }
    finishAssets();

    ct::Json output = ct::Json::object();
    const std::size_t separator = path.find("::");
    output["file"] = separator == ct::String::npos ? path : path.substr(0, separator);
    output["reference"] = path;
    output["applied"] = applied;
    if (warnings.size() > 0) output["warnings"] = warnings;
    result.addText(output.dump());
}

void McpTools::assignMaterial(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result) || !projectOpen(result)) return;
    Node* node = requireNode(arguments, "path", false, result);
    if (!node) return;
    MeshInstance3D* mesh = node->castTo<MeshInstance3D>();
    if (!mesh)
    {
        result.fail(ct::String("'") + pathOf(node) + "' is a " + node->getClassName() +
                    ", not a MeshInstance3D.");
        return;
    }
    ct::String material;
    if (!stringArgument(arguments, "material", material))
    {
        result.fail("Missing text argument 'material'.");
        return;
    }
    ct::String normalized;
    ct::String error;
    if (!normalizeProjectPath(material.contains("::") ? material.substr(0, material.find("::"))
                                                      : material,
                normalized, error) ||
            normalized.empty())
    {
        result.fail(ct::String("Bad material path: ") + failureText(error, "empty"));
        return;
    }
    const ct::String suffix =
            material.contains("::") ? material.substr(material.find("::")) : ct::String();
    const ct::String reference = materialReference(normalized + suffix, error);
    if (reference.empty())
    {
        result.fail(ct::String("Could not read the material file '") + normalized + "': " + error);
        return;
    }
    sceneres::clearError();
    const MaterialId id = sceneres::loadMaterial(reference);
    if (!id.valid())
    {
        result.fail(ct::String("Could not load the material: ") + sceneres::takeError());
        return;
    }
    int surface = MaterialPanel::kOverride;
    if (!boolArgument(arguments, "override", false))
    {
        surface = static_cast<int>(intArgument(arguments, "surface", 0, 0, 1 << 20));
        const MeshStore* meshes = MeshStore::singleton();
        const MeshInfo* meshInfo = meshes ? meshes->get(mesh->getMesh()) : nullptr;
        const int count = meshInfo ? static_cast<int>(meshInfo->surfaces.size()) : 0;
        if (surface >= count)
        {
            ct::String text = "The mesh has ";
            text.append_number(count);
            text += " surface(s); surface ";
            text.append_number(surface);
            text += " doesn't exist. Use override to cover every surface.";
            result.fail(text);
            return;
        }
    }
    ct::Vector<MeshInstance3D*> nodes;
    nodes.push_back(mesh);
    mContext.materials.recordAssign(nodes, surface, id);
    ct::Json output = ct::Json::object();
    output["node"] = describeNode(node);
    output["material"] = reference;
    output["slot"] = surface == MaterialPanel::kOverride ? ct::Json("override") : ct::Json(surface);
    result.addText(output.dump());
}

void McpTools::importAsset(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result) || !projectOpen(result)) return;
    ct::String source;
    if (!stringArgument(arguments, "source", source))
    {
        result.fail("Missing text argument 'source'.");
        return;
    }
    const Project& project = mContext.document.project();
    ct::String folder = project.assetRoot();
    ct::String requested;
    ct::String error;
    if (stringArgument(arguments, "folder", requested) &&
            !normalizeProjectPath(requested, folder, error))
    {
        result.fail(ct::String("Bad folder: ") + error);
        return;
    }
    const AssetKind kind = AssetTypes::classify(AssetImport::nameOf(source), false);
    if (kind != AssetKind::Model && kind != AssetKind::Texture && kind != AssetKind::Audio)
    {
        result.fail("Only model, texture and audio files can be imported.");
        return;
    }
    ct::Vector<ct::String> sources;
    sources.push_back(source);
    ImportPlan plan;
    AssetImport::plan(project, sources, folder, plan);
    if (plan.empty())
    {
        result.fail(plan.skipped.empty() ? ct::String("Nothing to import.") : plan.skipped[0]);
        return;
    }
    const bool overwrite = boolArgument(arguments, "overwrite", false);
    if (plan.clashCount() > 0 && !overwrite)
    {
        result.fail(plan.files[0].relative +
                    " already exists in the project. Pass overwrite=true to replace it.");
        return;
    }
    ct::Vector<ct::String> failed;
    const std::size_t copied = AssetImport::copy(plan, overwrite, failed);
    if (!failed.empty())
    {
        result.fail(ct::String("Could not copy ") + failed[0] + ".");
        return;
    }
    finishAssets();
    ct::Json files = ct::Json::array();
    for (std::size_t i = 0; i < plan.files.size(); ++i)
        files.push_back(ct::Json(plan.files[i].relative));
    ct::Json output = ct::Json::object();
    output["imported"] = files;
    output["copied"] = static_cast<unsigned long long>(copied);
    result.addText(output.dump());
}

void McpTools::newScene(const ct::Json& arguments, McpToolResult& result)
{
    if (mContext.document.playing())
    {
        result.fail("The game is running. Call stop first.");
        return;
    }
    if (mContext.document.dirty() && !boolArgument(arguments, "discard_changes", false))
    {
        result.fail("The current scene has unsaved changes. Call save_scene first, or pass "
                    "discard_changes=true to throw them away.");
        return;
    }
    if (!mContext.document.newScene())
    {
        result.fail(failureText(mContext.document.error(), "Could not start a new scene."));
        return;
    }
    ct::Json output = ct::Json::object();
    output["scene"] = sceneInfo();
    result.addText(output.dump());
}

void McpTools::openScene(const ct::Json& arguments, McpToolResult& result)
{
    if (mContext.document.playing())
    {
        result.fail("The game is running. Call stop first.");
        return;
    }
    ct::String path;
    if (!stringArgument(arguments, "path", path))
    {
        result.fail("Missing text argument 'path'.");
        return;
    }
    if (!projectOpen(result)) return;
    path = path.trimmed();
    if (!path.contains('/') && !path.ends_with(EditorDocument::kSceneSuffix))
        path = ct::String(Project::kScenesFolder) + "/" + EditorDocument::sceneFileName(path);
    ct::String normalized;
    ct::String reason;
    if (!Project::normalizeRelativePath(path, normalized, reason))
    {
        result.fail(ct::String("Bad scene path: ") + reason);
        return;
    }
    if (mContext.document.dirty() && !boolArgument(arguments, "discard_changes", false))
    {
        result.fail("The current scene has unsaved changes. Call save_scene first, or pass "
                    "discard_changes=true to throw them away.");
        return;
    }
    if (!mContext.document.openScene(normalized))
    {
        result.fail(failureText(mContext.document.error(), "Could not open the scene."));
        return;
    }
    ct::Json output = ct::Json::object();
    output["scene"] = sceneInfo();
    result.addText(output.dump());
}

void McpTools::saveScene(const ct::Json& arguments, McpToolResult& result)
{
    if (!scene(result)) return;
    if (mContext.document.playing())
    {
        result.fail("The game is running. Call stop first.");
        return;
    }
    ct::String name;
    const bool named = stringArgument(arguments, "name", name) && !name.trimmed().empty();
    if (!named && !mContext.document.hasScenePath())
    {
        result.fail("The scene has no file yet. Call save_scene with a 'name'.");
        return;
    }
    const bool saved =
            named ? mContext.document.saveSceneAs(name.trimmed()) : mContext.document.saveScene();
    if (!saved)
    {
        result.fail(failureText(mContext.document.error(), "The scene could not be saved."));
        return;
    }
    finishAssets();
    ct::Json output = ct::Json::object();
    output["scene"] = sceneInfo();
    result.addText(output.dump());
}

void McpTools::selectNode(const ct::Json& arguments, McpToolResult& result)
{
    if (!scene(result)) return;
    ct::Vector<Node*> nodes;
    ct::String path;
    if (stringArgument(arguments, "path", path))
    {
        Node* node = findNode(path);
        if (!node)
        {
            result.fail(ct::String("No node at the path '") + path + "'.");
            return;
        }
        nodes.push_back(node);
    }
    const ct::Json* paths = arguments.find("paths");
    if (paths && paths->is_array())
    {
        for (std::size_t i = 0; i < paths->size(); ++i)
        {
            Node* node = paths->at(i).is_string() ? findNode(paths->at(i).str()) : nullptr;
            if (!node)
            {
                result.fail("One of the paths does not name a node of the scene.");
                return;
            }
            nodes.push_back(node);
        }
    }
    mContext.selection.clear();
    for (std::size_t i = 0; i < nodes.size(); ++i) mContext.selection.add(nodes[i]);
    ct::Json list = ct::Json::array();
    for (std::size_t i = 0; i < nodes.size(); ++i) list.push_back(describeNode(nodes[i]));
    ct::Json output = ct::Json::object();
    output["selection"] = list;
    result.addText(output.dump());
}

ct::String McpTools::scriptRoot() const
{
    return AssetImport::joinPath(mContext.document.project().assetRoot(), kScriptFolder);
}

void McpTools::writeScript(const ct::Json& arguments, McpToolResult& result)
{
    if (!projectOpen(result)) return;
    ct::String path;
    ct::String source;
    if (!stringArgument(arguments, "path", path) || !stringArgument(arguments, "source", source))
    {
        result.fail("write_script needs the text arguments 'path' and 'source'.");
        return;
    }
    if (source.size() > kScriptLimit)
    {
        result.fail("The script is larger than 256 KiB.");
        return;
    }
    if (source.contains('\0'))
    {
        result.fail("The script contains a NUL character.");
        return;
    }
    const ct::String root = scriptRoot();
    ct::String normalized;
    ct::String error;
    if (!normalizeProjectPath(path.contains('/') || path.contains('\\')
                                      ? path
                                      : AssetImport::joinPath(root, path),
                normalized, error))
    {
        result.fail(ct::String("Bad script path: ") + error);
        return;
    }
    if (!normalized.starts_with(root + "/") || !normalized.ends_with(kScriptExtension) ||
            normalized.size() <= root.size() + 1 + 3)
    {
        result.fail(ct::String("Scripts live under ") + root + "/ and end in " + kScriptExtension +
                    ", for example " + root + "/spin.py.");
        return;
    }
    const Project& project = mContext.document.project();
    const ct::String absolute = project.absolutePath(normalized);
    const ct::String folder = AssetImport::parentOf(absolute);
    if (!folder.empty() && !fs_create_directory(folder.c_str()))
    {
        result.fail(ct::String("Could not create the folder ") + AssetImport::parentOf(normalized) +
                    ".");
        return;
    }
    const bool existed = file_exists(absolute.c_str());
    if (!file_write_text(absolute.c_str(), source.c_str()))
    {
        result.fail(ct::String("Could not write ") + normalized + ".");
        return;
    }
    finishAssets();
    ct::Json output = ct::Json::object();
    output["path"] = normalized;
    output["replaced"] = existed;
    output["bytes"] = static_cast<unsigned long long>(source.size());
    result.addText(output.dump());
}

void McpTools::attachScript(const ct::Json& arguments, McpToolResult& result)
{
    if (!editable(result) || !projectOpen(result)) return;
    Node* node = requireNode(arguments, "node_path", false, result);
    if (!node) return;
    ct::String script;
    if (!stringArgument(arguments, "script_path", script))
    {
        result.fail("Missing text argument 'script_path'.");
        return;
    }
    ct::String normalized;
    ct::String error;
    if (!normalizeProjectPath(script, normalized, error) || normalized.empty())
    {
        result.fail(ct::String("Bad script path: ") + failureText(error, "empty"));
        return;
    }
    if (!normalized.ends_with(kScriptExtension) ||
            !file_exists(mContext.document.project().absolutePath(normalized).c_str()))
    {
        result.fail(ct::String("There is no script file '") + normalized +
                    "' in the project. Create it with write_script first.");
        return;
    }
    PropertyValue before;
    if (!NodeClassDB::getProperty(node, "script", before))
    {
        result.fail("This node can't take a script.");
        return;
    }
    EditorUndoRedoManager& undo = mContext.document.undoRedo();
    undo.createAction("Attach Script");
    undo.addDoProperty(node, "script", PropertyValue::fromResourcePath(normalized));
    undo.addUndoProperty(node, "script", before);
    undo.commitAction();
    select(node);
    ct::Json output = ct::Json::object();
    output["node"] = describeNode(node);
    output["script"] = node->getScript();
    result.addText(output.dump());
}

void McpTools::play(const ct::Json&, McpToolResult& result)
{
    if (!scene(result)) return;
    ZenScripting* scripting = ZenScripting::instance();
    const int errorsBefore = scripting ? scripting->getErrorCount() : 0;
    if (!mContext.play.play())
    {
        result.fail(failureText(mContext.document.error(), "The game could not start."));
        return;
    }
    ct::Json output = ct::Json::object();
    output["playing"] = mContext.play.playing();
    if (scripting)
    {
        const int errors = scripting->getErrorCount() - errorsBefore;
        output["script_errors"] = errors > 0 ? errors : 0;
        if (errors > 0) output["last_script_error"] = scripting->getLastError();
    }
    result.addText(output.dump());
}

void McpTools::stop(const ct::Json&, McpToolResult& result)
{
    if (!mContext.play.playing())
    {
        result.fail("The game is not running.");
        return;
    }
    if (!mContext.play.stop())
    {
        result.fail(failureText(mContext.document.error(), "The game could not be stopped."));
        return;
    }
    ct::Json output = ct::Json::object();
    output["playing"] = mContext.play.playing();
    result.addText(output.dump());
}

} // namespace prisma3d
