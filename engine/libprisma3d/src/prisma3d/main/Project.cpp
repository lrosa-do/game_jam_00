#include "prisma3d/main/Project.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const double kInfinity = static_cast<double>(INFINITY);

const char* const kInputPrefix = "input/";

ct::String indexed(const ct::String& base, std::size_t index)
{
    ct::String text = base;
    text += '[';
    text.append_number(static_cast<unsigned long long>(index));
    text += ']';
    return text;
}

ct::String joined(const ct::String& base, const char* field)
{
    ct::String text = base;
    if (!text.empty()) text += '.';
    text += field;
    return text;
}

ct::String withReason(const ct::String& path, const ct::String& reason)
{
    if (path.empty()) return reason;
    ct::String text = path;
    text += ": ";
    text += reason;
    return text;
}

bool isAbsolutePath(const ct::String& path)
{
    if (path.empty()) return false;
    if (path[0] == '/' || path[0] == '\\') return true;
    return path.size() >= 2 && path[1] == ':';
}

ct::String slashes(const ct::String& path)
{
    ct::String result = path;
    char* data = result.data();
    for (std::size_t i = 0; i < result.size(); ++i)
        if (data[i] == '\\') data[i] = '/';
    return result;
}

ct::String trimmedTrailingSlashes(const ct::String& path)
{
    ct::String result = slashes(path);
    while (result.size() > 1 && result[result.size() - 1] == '/') result.pop_back();
    return result;
}

bool validProjectName(const ct::String& name)
{
    if (name.empty() || name == "." || name == "..") return false;
    for (std::size_t i = 0; i < name.size(); ++i)
    {
        const char c = name[i];
        if (c == '/' || c == '\\' || c == ':' || static_cast<unsigned char>(c) < 32) return false;
    }
    return true;
}

bool absolute(const ct::String& path, ct::String& out)
{
    char buffer[4096];
    if (!path_absolute(buffer, sizeof buffer, path.c_str())) return false;
    out = slashes(ct::String(buffer));
    return true;
}

bool isDirectory(const ct::String& path)
{
    PathInfo info;
    return fs_get_path_info(path.c_str(), &info) && info.type == PATH_TYPE_DIRECTORY;
}

const char* const kAtlasQuadrants[4] = {
    "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_0_subdiv",
    "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_1_subdiv",
    "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_2_subdiv",
    "rendering/lights_and_shadows/positional_shadow/atlas_quadrant_3_subdiv",
};

std::uint32_t atlasSubdivision(std::int64_t index)
{
    return index <= 0 ? 0u : 1u << (2 * (index - 1));
}

std::uint64_t bit(int value)
{
    return static_cast<std::uint64_t>(1) << value;
}

std::uint64_t maskOf(int first, int last)
{
    std::uint64_t mask = 0;
    for (int i = first; i <= last; ++i) mask |= bit(i);
    return mask;
}

class SettingTable
{
public:
    ct::Vector<ProjectSettingInfo> list;

    void add(const char* name, PropertyType type, const PropertyValue& value)
    {
        ProjectSettingInfo info;
        info.name = name;
        info.type = type;
        info.defaultValue = value;
        list.push_back(info);
    }

    void addBool(const char* name, bool value)
    {
        add(name, PropertyType::Bool, PropertyValue::fromBool(value));
    }

    void addInt(const char* name, std::int64_t value, double minimum, double maximum)
    {
        add(name, PropertyType::Int, PropertyValue::fromInt(value));
        list[list.size() - 1].limited = true;
        list[list.size() - 1].minimum = minimum;
        list[list.size() - 1].maximum = maximum;
    }

    void addEnum(const char* name, std::int64_t value, std::uint64_t allowedMask)
    {
        add(name, PropertyType::Int, PropertyValue::fromInt(value));
        list[list.size() - 1].allowedMask = allowedMask;
    }

    void addFloat(const char* name, double value, double minimum, double maximum)
    {
        add(name, PropertyType::Float, PropertyValue::fromFloat(value));
        list[list.size() - 1].limited = true;
        list[list.size() - 1].minimum = minimum;
        list[list.size() - 1].maximum = maximum;
    }
};

ct::Vector<ProjectSettingInfo> buildSettings()
{
    SettingTable table;
    table.addInt("application/run/max_fps", 0, 0.0, 1000.0);
    table.addInt("application/run/frame_delay_msec", 0, 0.0, kInfinity);
    table.addBool("application/run/low_processor_mode", false);
    table.addInt("application/run/low_processor_mode_sleep_usec", 6900, 0.0, kInfinity);

    table.addInt("display/window/size/viewport_width", 1152, 1.0, kInfinity);
    table.addInt("display/window/size/viewport_height", 648, 1.0, kInfinity);
    table.addEnum("display/window/size/mode", 0, maskOf(0, 4));
    table.addEnum("display/window/size/initial_position_type", 1,
            bit(1) | bit(3) | bit(4) | bit(5));
    table.addInt("display/window/size/initial_screen", 0, 0.0, kInfinity);
    table.addBool("display/window/size/resizable", true);
    table.addBool("display/window/size/borderless", false);
    table.addBool("display/window/size/always_on_top", false);
    table.addInt("display/window/size/window_width_override", 0, 0.0, kInfinity);
    table.addInt("display/window/size/window_height_override", 0, 0.0, kInfinity);
    table.addEnum("display/window/vsync/vsync_mode", 1, maskOf(0, 2));

    table.addInt("physics/common/physics_ticks_per_second", 60, 1.0, 1000.0);
    table.addInt("physics/common/max_physics_steps_per_frame", 8, 1.0, 100.0);
    table.addFloat("physics/common/physics_jitter_fix", 0.5, 0.0, kInfinity);
    table.addFloat("physics/3d/default_gravity", 9.8, -kInfinity, kInfinity);
    table.add("physics/3d/default_gravity_vector", PropertyType::Vec3,
            PropertyValue::fromVec3(Math::Vec3(0.0f, -1.0f, 0.0f)));

    table.addEnum("rendering/anti_aliasing/quality/msaa_3d", 0, maskOf(0, 3));
    table.addEnum("rendering/anti_aliasing/quality/screen_space_aa", 0, maskOf(0, 2));
    table.addFloat("rendering/anti_aliasing/quality/smaa_edge_detection_threshold", 0.05, 0.01,
            0.2);
    table.addBool("rendering/anti_aliasing/screen_space_roughness_limiter/enabled", true);
    table.addFloat("rendering/anti_aliasing/screen_space_roughness_limiter/amount", 0.25, 0.01,
            4.0);
    table.addFloat("rendering/anti_aliasing/screen_space_roughness_limiter/limit", 0.18, 0.01, 1.0);
    table.addFloat("rendering/scaling_3d/scale", 1.0, 0.25, 2.0);
    table.addInt("rendering/scaling_3d/width", 0, 0.0, 16384.0);
    table.addInt("rendering/scaling_3d/height", 0, 0.0, 16384.0);
    table.addEnum("rendering/scaling_3d/aspect", 0, maskOf(0, 3));
    table.addEnum("rendering/scaling_3d/mode", 0, maskOf(0, 1));
    table.addFloat("rendering/scaling_3d/fsr_sharpness", 0.2, 0.0, 2.0);
    table.addFloat("rendering/mesh_lod/lod_change/threshold_pixels", 1.0, 0.0, 1024.0);
    table.addInt("rendering/lights_and_shadows/directional_shadow/size", 4096, 256.0, 16384.0);
    table.addEnum("rendering/lights_and_shadows/directional_shadow/soft_shadow_filter_quality", 2,
            maskOf(0, 5));
    table.addInt("rendering/lights_and_shadows/positional_shadow/atlas_size", 4096, 256.0, 16384.0);
    table.addEnum("rendering/lights_and_shadows/positional_shadow/soft_shadow_filter_quality", 2,
            maskOf(0, 5));
    table.addEnum("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_0_subdiv", 2,
            maskOf(0, 6));
    table.addEnum("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_1_subdiv", 2,
            maskOf(0, 6));
    table.addEnum("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_2_subdiv", 3,
            maskOf(0, 6));
    table.addEnum("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_3_subdiv", 4,
            maskOf(0, 6));
    table.addEnum("rendering/environment/ssao/quality", 2, maskOf(0, 4));
    table.addBool("rendering/environment/ssao/half_size", true);
    table.addFloat("rendering/environment/ssao/adaptive_target", 0.5, 0.0, 1.0);
    table.addInt("rendering/environment/ssao/blur_passes", 2, 0.0, 6.0);
    table.addFloat("rendering/environment/ssao/fadeout_from", 50.0, 0.0, kInfinity);
    table.addFloat("rendering/environment/ssao/fadeout_to", 300.0, 64.0, kInfinity);
    table.addBool("rendering/environment/screen_space_reflection/half_size", true);
    table.addEnum("rendering/camera/depth_of_field/depth_of_field_bokeh_shape", 1, maskOf(0, 2));
    table.addEnum("rendering/camera/depth_of_field/depth_of_field_bokeh_quality", 1, maskOf(0, 3));
    table.addBool("rendering/camera/depth_of_field/depth_of_field_use_jitter", false);
    table.addBool("rendering/lightmapping/lightmap_gi/use_bicubic_filter", true);
    table.add("rendering/environment/defaults/default_clear_color", PropertyType::Color,
            PropertyValue::fromColor(Math::Vec4(0.3f, 0.3f, 0.3f, 1.0f)));
    table.add("export/pack_key", PropertyType::String, PropertyValue::fromString(ct::String()));
    table.addBool("export/keep_debug", false);
    table.add("export/exclude", PropertyType::String,
            PropertyValue::fromString(ct::String("*.blend;*.blend1;*.psd;*.xcf;*.kra;*.import")));
    return table.list;
}

const ct::Vector<ProjectSettingInfo>& settingTable()
{
    static const ct::Vector<ProjectSettingInfo> table = buildSettings();
    return table;
}

std::size_t settingIndex(const ct::String& name)
{
    const ct::Vector<ProjectSettingInfo>& table = settingTable();
    for (std::size_t i = 0; i < table.size(); ++i)
        if (table[i].name == name) return i;
    return table.size();
}

bool finite(const PropertyValue& value)
{
    switch (value.type())
    {
        case PropertyType::Float:
            return isfinite(value.asFloat()) != 0;
        case PropertyType::Vec3:
        {
            const Math::Vec3 v = value.asVec3();
            return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
        }
        case PropertyType::Color:
        {
            const Math::Vec4 v = value.asColor();
            return isfinite(v.x) && isfinite(v.y) && isfinite(v.z) && isfinite(v.w);
        }
        default:
            return true;
    }
}

ct::String numberText(double value)
{
    ct::String text;
    text.append_number(value, 6);
    return text;
}

bool validateSetting(const ProjectSettingInfo& info, const PropertyValue& value, ct::String& reason)
{
    if (!finite(value))
    {
        reason = "the value must be a finite number";
        return false;
    }
    if (info.allowedMask != 0)
    {
        const std::int64_t number = value.asInt();
        if (number < 0 || number >= 64 || (info.allowedMask & bit(static_cast<int>(number))) == 0)
        {
            reason = "";
            reason.append_number(static_cast<long long>(number));
            reason += " is not one of the supported values";
            return false;
        }
    }
    if (info.limited)
    {
        const double number = value.asFloat();
        if (!(number >= info.minimum && number <= info.maximum))
        {
            reason = numberText(number);
            reason += " is out of range (";
            reason += numberText(info.minimum);
            reason += " to ";
            reason += isinf(info.maximum) ? ct::String("no limit") : numberText(info.maximum);
            reason += ')';
            return false;
        }
    }
    return true;
}

bool readSceneList(const ct::Json& list, ct::Vector<ct::String>& scenes,
        ct::Vector<ct::String>& warnings, ct::String& error)
{
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        const ct::Json& entry = list.at(i);
        if (!entry.is_string())
        {
            error = withReason(indexed("scenes", i), "not a text");
            return false;
        }
        ct::String normalized;
        ct::String reason;
        if (!Project::normalizeRelativePath(entry.str(), normalized, reason))
        {
            error = withReason(indexed("scenes", i), ct::String("'") + entry.str() + "' " + reason);
            return false;
        }
        bool repeated = false;
        for (std::size_t j = 0; j < scenes.size(); ++j)
            if (scenes[j] == normalized) repeated = true;
        if (repeated)
        {
            warnings.push_back(withReason(indexed("scenes", i),
                    ct::String("'") + normalized + "' is listed more than once"));
            continue;
        }
        scenes.push_back(normalized);
    }
    return true;
}

void warnUnknownFields(const ct::Json& object, const char* const* known, std::size_t count,
        const ct::String& path, ct::Vector<ct::String>& warnings)
{
    const ct::Json::Object& members = object.members();
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        bool found = false;
        for (std::size_t j = 0; j < count; ++j)
            if (members[i].key == known[j]) found = true;
        if (!found)
            warnings.push_back(withReason(joined(path, members[i].key.c_str()), "unknown field"));
    }
}

int mouseButtonIndex(MouseButton button)
{
    switch (button)
    {
        case MouseButton::Left:
            return 1;
        case MouseButton::Right:
            return 2;
        case MouseButton::Middle:
            return 3;
        case MouseButton::X1:
            return 8;
        case MouseButton::X2:
            return 9;
        default:
            return 0;
    }
}

bool mouseButtonFromIndex(std::int64_t index, MouseButton& out)
{
    switch (index)
    {
        case 1:
            out = MouseButton::Left;
            return true;
        case 2:
            out = MouseButton::Right;
            return true;
        case 3:
            out = MouseButton::Middle;
            return true;
        case 8:
            out = MouseButton::X1;
            return true;
        case 9:
            out = MouseButton::X2;
            return true;
        default:
            return false;
    }
}

struct ModifierName
{
    const char* name;
    unsigned mask;
};

const ModifierName kModifierNames[] = {
    { "shift_pressed", kModShift },
    { "ctrl_pressed", kModControl },
    { "alt_pressed", kModAlt },
    { "meta_pressed", kModSuper },
};

unsigned eventModifiers(const InputEvent& event)
{
    return event.type == InputEventType::Key ? event.key.modifiers : event.mouseButton.modifiers;
}

bool validateInputEvent(const InputEvent& event, ct::String& reason)
{
    if (event.type == InputEventType::Key)
    {
        const int code = static_cast<int>(event.key.key);
        if (code < 1 || code >= static_cast<int>(Key::Max))
        {
            reason = "the key code is out of range";
            return false;
        }
        return true;
    }
    if (event.type == InputEventType::MouseButton)
    {
        if (mouseButtonIndex(event.mouseButton.button) == 0)
        {
            reason = "the mouse button is not one that can be stored";
            return false;
        }
        return true;
    }
    reason = "only key and mouse button events can be bound";
    return false;
}

bool validateInputAction(const ProjectInputAction& action, ct::String& reason)
{
    if (action.name.empty())
    {
        reason = "the action name is empty";
        return false;
    }
    if (!(action.deadzone >= 0.0f && action.deadzone <= 1.0f))
    {
        reason = "the dead zone must be between 0 and 1";
        return false;
    }
    for (std::size_t i = 0; i < action.events.size(); ++i)
    {
        ct::String eventReason;
        if (!validateInputEvent(action.events[i], eventReason))
        {
            reason = withReason(indexed("events", i), eventReason);
            return false;
        }
    }
    return true;
}

bool readInputEvent(const ct::Json& json, const ct::String& path, InputEvent& out,
        ct::Vector<ct::String>& warnings, ct::String& error)
{
    if (!json.is_object())
    {
        error = withReason(path, "not an object");
        return false;
    }
    const ct::Json* type = json.find("type");
    if (!type || !type->is_string())
    {
        error = withReason(joined(path, "type"), "missing or not a text");
        return false;
    }
    out = InputEvent();
    if (type->str() == "key")
    {
        static const char* const known[] = { "type", "keycode", "shift_pressed", "ctrl_pressed",
            "alt_pressed", "meta_pressed" };
        warnUnknownFields(json, known, sizeof known / sizeof known[0], path, warnings);
        std::int64_t code = 0;
        const ct::Json* keycode = json.find("keycode");
        if (!keycode || !scenejson::toInt(*keycode, code))
        {
            error = withReason(joined(path, "keycode"), "missing or not a whole number");
            return false;
        }
        if (code < 1 || code >= static_cast<std::int64_t>(Key::Max))
        {
            error = withReason(joined(path, "keycode"), "out of range");
            return false;
        }
        out.type = InputEventType::Key;
        out.key.key = static_cast<Key>(code);
        out.key.scancode = static_cast<int>(code);
    }
    else if (type->str() == "mouse_button")
    {
        static const char* const known[] = { "type", "button_index", "shift_pressed",
            "ctrl_pressed", "alt_pressed", "meta_pressed" };
        warnUnknownFields(json, known, sizeof known / sizeof known[0], path, warnings);
        std::int64_t index = 0;
        const ct::Json* button = json.find("button_index");
        if (!button || !scenejson::toInt(*button, index))
        {
            error = withReason(joined(path, "button_index"), "missing or not a whole number");
            return false;
        }
        MouseButton value = MouseButton::Left;
        if (!mouseButtonFromIndex(index, value))
        {
            error = withReason(joined(path, "button_index"),
                    "not a button that can be stored (1 left, 2 right, 3 middle, 8 and 9 extra)");
            return false;
        }
        out.type = InputEventType::MouseButton;
        out.mouseButton.button = value;
    }
    else
    {
        error = withReason(joined(path, "type"),
                ct::String("unknown event type '") + type->str() + "'");
        return false;
    }

    unsigned modifiers = 0;
    for (const ModifierName& modifier: kModifierNames)
    {
        const ct::Json* flag = json.find(modifier.name);
        if (!flag) continue;
        if (!flag->is_bool())
        {
            error = withReason(joined(path, modifier.name), "not true or false");
            return false;
        }
        if (flag->as_bool()) modifiers |= modifier.mask;
    }
    if (out.type == InputEventType::Key) out.key.modifiers = modifiers;
    else
        out.mouseButton.modifiers = modifiers;
    return true;
}

ct::Json writeInputEvent(const InputEvent& event)
{
    ct::Json json = ct::Json::object();
    if (event.type == InputEventType::Key)
    {
        json["type"] = ct::Json("key");
        json["keycode"] = ct::Json(static_cast<int>(event.key.key));
    }
    else
    {
        json["type"] = ct::Json("mouse_button");
        json["button_index"] = ct::Json(mouseButtonIndex(event.mouseButton.button));
    }
    const unsigned modifiers = eventModifiers(event);
    for (const ModifierName& modifier: kModifierNames)
        if (modifiers & modifier.mask) json[modifier.name] = ct::Json(true);
    return json;
}

} // namespace

Project::Project()
        : mOpen(false),
          mAssetRoot(kDefaultAssetRoot)
{
    const ct::Vector<ProjectSettingInfo>& table = settingTable();
    for (std::size_t i = 0; i < table.size(); ++i) mValues.push_back(table[i].defaultValue);
}

bool Project::fail(const ct::String& message)
{
    mError = message;
    return false;
}

bool Project::isOpen() const
{
    return mOpen;
}

const ct::String& Project::error() const
{
    return mError;
}

const ct::Vector<ct::String>& Project::warnings() const
{
    return mWarnings;
}

const ct::String& Project::name() const
{
    return mName;
}

bool Project::setName(const ct::String& name)
{
    if (name.empty()) return fail("the project name is empty");
    mName = name;
    return true;
}

const ct::String& Project::root() const
{
    return mRoot;
}

const ct::String& Project::manifestPath() const
{
    return mManifestPath;
}

const ct::String& Project::assetRoot() const
{
    return mAssetRoot;
}

bool Project::setAssetRoot(const ct::String& assetRoot)
{
    ct::String normalized;
    ct::String reason;
    if (!normalizeRelativePath(assetRoot, normalized, reason))
        return fail(withReason("assetRoot", ct::String("'") + assetRoot + "' " + reason));
    mAssetRoot = normalized;
    return true;
}

ct::String Project::assetPath() const
{
    return absolutePath(mAssetRoot);
}

ct::String Project::absolutePath(const ct::String& relative) const
{
    if (mRoot.empty()) return relative;
    return mRoot.ends_with("/") ? mRoot + relative : mRoot + "/" + relative;
}

bool Project::relativePath(const ct::String& path, ct::String& out) const
{
    if (!mOpen || path.empty()) return false;
    const ct::String text = slashes(path);
    ct::String reason;
    if (!isAbsolutePath(text)) return normalizeRelativePath(text, out, reason);

    ct::String full;
    if (!absolute(text, full)) return false;
    ct::String root;
    if (!absolute(mRoot, root)) return false;
    root = trimmedTrailingSlashes(root);
    if (root.size() <= 1)
    {
        if (!full.starts_with(root)) return false;
        return normalizeRelativePath(full.substr(root.size()), out, reason);
    }
    if (full.size() <= root.size() + 1 || !full.starts_with(root) || full[root.size()] != '/')
        return false;
    return normalizeRelativePath(full.substr(root.size() + 1), out, reason);
}

bool Project::normalizeRelativePath(const ct::String& path, ct::String& out, ct::String& reason)
{
    if (path.empty())
    {
        reason = "is empty";
        return false;
    }
    if (isAbsolutePath(path))
    {
        reason = "must be a path relative to the project folder";
        return false;
    }
    const ct::String text = slashes(path);
    ct::String result;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= text.size(); ++i)
    {
        if (i != text.size() && text[i] != '/') continue;
        const ct::String part = text.substr(start, i - start);
        start = i + 1;
        if (part.empty() || part == ".") continue;
        if (part == "..")
        {
            reason = "must not contain '..'";
            return false;
        }
        if (!result.empty()) result += '/';
        result += part;
    }
    if (result.empty())
    {
        reason = "is empty";
        return false;
    }
    out = result;
    return true;
}

const ct::Vector<ct::String>& Project::scenes() const
{
    return mScenes;
}

std::size_t Project::findScene(const ct::String& scene) const
{
    ct::String normalized;
    ct::String reason;
    if (!normalizeRelativePath(scene, normalized, reason)) return mScenes.size();
    for (std::size_t i = 0; i < mScenes.size(); ++i)
        if (mScenes[i] == normalized) return i;
    return mScenes.size();
}

bool Project::hasScene(const ct::String& scene) const
{
    return findScene(scene) < mScenes.size();
}

bool Project::addScene(const ct::String& scene)
{
    ct::String normalized;
    ct::String reason;
    if (!normalizeRelativePath(scene, normalized, reason))
        return fail(withReason("scenes", ct::String("'") + scene + "' " + reason));
    if (hasScene(normalized)) return fail(ct::String("'") + normalized + "' is already a scene");
    mScenes.push_back(normalized);
    return true;
}

bool Project::removeScene(const ct::String& scene)
{
    const std::size_t index = findScene(scene);
    if (index >= mScenes.size()) return fail(ct::String("'") + scene + "' is not a scene");
    if (mScenes[index] == mMainScene) mMainScene = "";
    mScenes.erase(mScenes.begin() + static_cast<std::ptrdiff_t>(index));
    return true;
}

bool Project::renameScene(const ct::String& from, const ct::String& to)
{
    const std::size_t index = findScene(from);
    if (index >= mScenes.size()) return fail(ct::String("'") + from + "' is not a scene");
    ct::String normalized;
    ct::String reason;
    if (!normalizeRelativePath(to, normalized, reason))
        return fail(withReason("scenes", ct::String("'") + to + "' " + reason));
    if (normalized != mScenes[index] && hasScene(normalized))
        return fail(ct::String("'") + normalized + "' is already a scene");
    if (mScenes[index] == mMainScene) mMainScene = normalized;
    mScenes[index] = normalized;
    return true;
}

const ct::String& Project::mainScene() const
{
    return mMainScene;
}

bool Project::setMainScene(const ct::String& scene)
{
    if (scene.empty())
    {
        mMainScene = "";
        return true;
    }
    const std::size_t index = findScene(scene);
    if (index >= mScenes.size())
        return fail(ct::String("the main scene '") + scene + "' is not listed in scenes");
    mMainScene = mScenes[index];
    return true;
}

const ct::Vector<ct::String>& Project::searchPaths() const
{
    return mSearchPaths;
}

bool Project::addSearchPath(const ct::String& path)
{
    const ct::String clean = trimmedTrailingSlashes(path);
    if (clean.empty()) return fail("the search path is empty");
    for (std::size_t i = 0; i < mSearchPaths.size(); ++i)
        if (mSearchPaths[i] == clean)
            return fail(ct::String("'") + clean + "' is already a search path");
    mSearchPaths.push_back(clean);
    return true;
}

bool Project::removeSearchPath(const ct::String& path)
{
    const ct::String clean = trimmedTrailingSlashes(path);
    for (std::size_t i = 0; i < mSearchPaths.size(); ++i)
    {
        if (mSearchPaths[i] != clean) continue;
        mSearchPaths.erase(mSearchPaths.begin() + static_cast<std::ptrdiff_t>(i));
        return true;
    }
    return fail(ct::String("'") + clean + "' is not a search path");
}

ct::Vector<ct::String> Project::resolvedSearchPaths() const
{
    ct::Vector<ct::String> result;
    for (std::size_t i = 0; i < mSearchPaths.size(); ++i)
    {
        const ct::String& path = mSearchPaths[i];
        ct::String resolved;
        if (isAbsolutePath(path) || mRoot.empty()) resolved = path;
        else
            resolved = mRoot + "/" + path;
        ct::String full;
        result.push_back(absolute(resolved, full) ? full : resolved);
    }
    return result;
}

std::size_t Project::settingCount()
{
    return settingTable().size();
}

const ProjectSettingInfo& Project::settingInfo(std::size_t index)
{
    return settingTable()[index];
}

const ProjectSettingInfo* Project::findSetting(const ct::String& name)
{
    const std::size_t index = settingIndex(name);
    return index < settingTable().size() ? &settingTable()[index] : nullptr;
}

PropertyValue Project::getDefault(const ct::String& name)
{
    const ProjectSettingInfo* info = findSetting(name);
    return info ? info->defaultValue : PropertyValue();
}

bool Project::has(const ct::String& name) const
{
    return settingIndex(name) < settingTable().size();
}

PropertyValue Project::get(const ct::String& name) const
{
    const std::size_t index = settingIndex(name);
    return index < mValues.size() ? mValues[index] : PropertyValue();
}

bool Project::set(const ct::String& name, const PropertyValue& value)
{
    const std::size_t index = settingIndex(name);
    if (index >= settingTable().size()) return fail(ct::String("unknown setting '") + name + "'");
    const ProjectSettingInfo& info = settingTable()[index];
    PropertyValue converted;
    if (!value.convertTo(info.type, converted))
        return fail(withReason(name, ct::String("expects ") + PropertyValue::typeName(info.type) +
                                             ", got " + PropertyValue::typeName(value.type())));
    if (info.type == PropertyType::Int && value.type() == PropertyType::Float &&
            value.asFloat() != floor(value.asFloat()))
        return fail(withReason(name, "expects a whole number"));
    ct::String reason;
    if (!validateSetting(info, converted, reason)) return fail(withReason(name, reason));
    mValues[index] = converted;
    return true;
}

bool Project::isDefault(const ct::String& name) const
{
    const std::size_t index = settingIndex(name);
    return index >= settingTable().size() || mValues[index] == settingTable()[index].defaultValue;
}

void Project::reset(const ct::String& name)
{
    const std::size_t index = settingIndex(name);
    if (index < settingTable().size()) mValues[index] = settingTable()[index].defaultValue;
}

std::int64_t Project::integer(const char* name) const
{
    return get(name).asInt();
}

double Project::number(const char* name) const
{
    return get(name).asFloat();
}

bool Project::flag(const char* name) const
{
    return get(name).asBool();
}

const ct::Vector<ProjectInputAction>& Project::inputActions() const
{
    return mInputActions;
}

const ProjectInputAction* Project::findInputAction(const ct::String& name) const
{
    for (std::size_t i = 0; i < mInputActions.size(); ++i)
        if (mInputActions[i].name == name) return &mInputActions[i];
    return nullptr;
}

bool Project::setInputAction(const ProjectInputAction& action)
{
    ct::String reason;
    if (!validateInputAction(action, reason))
        return fail(withReason(ct::String(kInputPrefix) + action.name, reason));
    for (std::size_t i = 0; i < mInputActions.size(); ++i)
    {
        if (mInputActions[i].name != action.name) continue;
        mInputActions[i] = action;
        return true;
    }
    mInputActions.push_back(action);
    return true;
}

bool Project::removeInputAction(const ct::String& name)
{
    for (std::size_t i = 0; i < mInputActions.size(); ++i)
    {
        if (mInputActions[i].name != name) continue;
        mInputActions.erase(mInputActions.begin() + static_cast<std::ptrdiff_t>(i));
        return true;
    }
    return fail(ct::String("unknown input action '") + name + "'");
}

ct::Json Project::writeInputAction(const ProjectInputAction& action) const
{
    ct::Json json = ct::Json::object();
    json["deadzone"] = scenejson::fromFloat(action.deadzone);
    ct::Json events = ct::Json::array();
    for (std::size_t i = 0; i < action.events.size(); ++i)
        events.push_back(writeInputEvent(action.events[i]));
    json["events"] = events;
    return json;
}

bool Project::readInputAction(const ct::Json& json, const ct::String& path, ProjectInputAction& out)
{
    if (!json.is_object()) return fail(withReason(path, "not an object"));
    static const char* const known[] = { "deadzone", "events" };
    warnUnknownFields(json, known, sizeof known / sizeof known[0], path, mWarnings);
    out.deadzone = InputMap::kDefaultDeadzone;
    out.events.clear();
    if (const ct::Json* deadzone = json.find("deadzone"))
    {
        if (!scenejson::toFloat(*deadzone, out.deadzone))
            return fail(withReason(joined(path, "deadzone"), "not a number"));
    }
    if (const ct::Json* events = json.find("events"))
    {
        if (!events->is_array()) return fail(withReason(joined(path, "events"), "not a list"));
        for (std::size_t i = 0; i < events->size(); ++i)
        {
            InputEvent event;
            ct::String error;
            if (!readInputEvent(events->at(i), indexed(joined(path, "events"), i), event, mWarnings,
                        error))
                return fail(error);
            out.events.push_back(event);
        }
    }
    ct::String reason;
    if (!validateInputAction(out, reason)) return fail(withReason(path, reason));
    return true;
}

bool Project::readSettings(const ct::Json& settings)
{
    const ct::Json::Object& members = settings.members();
    for (std::size_t i = 0; i < members.size(); ++i)
    {
        const ct::String& key = members[i].key;
        const ct::Json& value = members[i].value;
        const ct::String path = joined("settings", key.c_str());
        if (key.starts_with(kInputPrefix))
        {
            ProjectInputAction action;
            action.name = key.substr(strlen(kInputPrefix));
            if (action.name.empty()) return fail(withReason(path, "the action name is empty"));
            if (!readInputAction(value, path, action)) return false;
            if (findInputAction(action.name)) continue;
            mInputActions.push_back(action);
            continue;
        }
        const std::size_t index = settingIndex(key);
        if (index >= settingTable().size())
        {
            mWarnings.push_back(withReason(path, "unknown setting, kept as it is"));
            mExtraSettings[key] = value;
            continue;
        }
        const ProjectSettingInfo& info = settingTable()[index];
        PropertyValue parsed;
        ct::String reason;
        if (!scenejson::toValue(value, info.type, parsed, reason))
            return fail(withReason(path, reason));
        if (!validateSetting(info, parsed, reason)) return fail(withReason(path, reason));
        mValues[index] = parsed;
    }
    return true;
}

bool Project::readManifest(const ct::Json& document, const ct::String& manifestPath)
{
    if (!document.is_object()) return fail("the document is not a JSON object");

    const ct::Json* format = document.find("format");
    if (!format || !format->is_string() || format->str() != kFormatName)
        return fail(withReason("format", ct::String("missing or not '") + kFormatName + "'"));

    const ct::Json* version = document.find("version");
    std::int64_t versionNumber = 0;
    if (!version || !scenejson::toInt(*version, versionNumber) || versionNumber <= 0)
        return fail(withReason("version", "missing or not a positive whole number"));
    if (versionNumber > static_cast<std::int64_t>(kFormatVersion))
    {
        ct::String message = "the file is version ";
        message.append_number(static_cast<long long>(versionNumber));
        message += ", newer than this build's ";
        message.append_number(static_cast<unsigned long long>(kFormatVersion));
        return fail(withReason("version", message));
    }

    static const char* const known[] = { "format", "version", "name", "assetRoot", "scenes",
        "activeScene", "searchPaths", "settings" };
    warnUnknownFields(document, known, sizeof known / sizeof known[0], "", mWarnings);

    mManifestPath = manifestPath;
    const std::size_t slash = manifestPath.rfind('/');
    mRoot = slash == ct::String::npos ? ct::String()
            : slash == 0              ? ct::String("/")
                                      : manifestPath.substr(0, slash);

    if (const ct::Json* name = document.find("name"))
    {
        if (!name->is_string()) return fail(withReason("name", "not a text"));
        mName = name->str();
    }
    if (mName.empty())
    {
        const std::size_t previous = mRoot.rfind('/');
        mName = previous == ct::String::npos ? mRoot : mRoot.substr(previous + 1);
    }

    if (const ct::Json* assetRoot = document.find("assetRoot"))
    {
        if (!assetRoot->is_string()) return fail(withReason("assetRoot", "not a text"));
        ct::String reason;
        if (!normalizeRelativePath(assetRoot->str(), mAssetRoot, reason))
            return fail(
                    withReason("assetRoot", ct::String("'") + assetRoot->str() + "' " + reason));
    }

    if (const ct::Json* scenes = document.find("scenes"))
    {
        if (!scenes->is_array()) return fail(withReason("scenes", "not a list"));
        ct::String error;
        if (!readSceneList(*scenes, mScenes, mWarnings, error)) return fail(error);
    }

    if (const ct::Json* active = document.find("activeScene"))
    {
        if (!active->is_string()) return fail(withReason("activeScene", "not a text"));
        if (!active->str().empty())
        {
            ct::String normalized;
            ct::String reason;
            if (!normalizeRelativePath(active->str(), normalized, reason))
                return fail(
                        withReason("activeScene", ct::String("'") + active->str() + "' " + reason));
            if (!hasScene(normalized))
                return fail(withReason("activeScene",
                        ct::String("'") + normalized + "' is not listed in scenes"));
            mMainScene = normalized;
        }
    }

    if (const ct::Json* searchPaths = document.find("searchPaths"))
    {
        if (!searchPaths->is_array()) return fail(withReason("searchPaths", "not a list"));
        for (std::size_t i = 0; i < searchPaths->size(); ++i)
        {
            const ct::Json& entry = searchPaths->at(i);
            if (!entry.is_string())
                return fail(withReason(indexed("searchPaths", i), "not a text"));
            const ct::String clean = trimmedTrailingSlashes(entry.str());
            bool repeated = clean.empty();
            for (std::size_t j = 0; j < mSearchPaths.size(); ++j)
                if (mSearchPaths[j] == clean) repeated = true;
            if (repeated)
            {
                mWarnings.push_back(
                        withReason(indexed("searchPaths", i), "empty or repeated, ignored"));
                continue;
            }
            mSearchPaths.push_back(clean);
        }
    }

    if (const ct::Json* settings = document.find("settings"))
    {
        if (!settings->is_object()) return fail(withReason("settings", "not an object"));
        if (!readSettings(*settings)) return false;
    }

    mOpen = true;
    return true;
}

bool Project::open(const ct::String& manifestPath)
{
    if (manifestPath.empty()) return fail("the project path is empty");
    ct::String full;
    if (!absolute(manifestPath, full))
        return fail(ct::String("invalid path '") + manifestPath + "'");
    if (isDirectory(full)) full = full + "/" + kManifestName;
    return openManifest(full);
}

bool Project::openPacked(const ct::String& manifestName)
{
    if (manifestName.empty()) return fail("the project path is empty");
    return openManifest(manifestName);
}

bool Project::openManifest(const ct::String& full)
{
    const ct::String text = FileSystem::getSingleton().readText(full);
    if (text.empty()) return fail(ct::String("could not read '") + full + "'");

    ct::Json::Error parseError;
    const ct::Json document = ct::Json::parse(text, &parseError);
    if (parseError)
    {
        ct::String message = "not valid JSON: ";
        message += parseError.message;
        message += " (line ";
        message.append_number(static_cast<unsigned long long>(parseError.line));
        message += ", column ";
        message.append_number(static_cast<unsigned long long>(parseError.column));
        message += ')';
        return fail(message);
    }

    Project loaded;
    if (!loaded.readManifest(document, full)) return fail(loaded.mError);
    unregisterSearchPaths();
    *this = static_cast<Project&&>(loaded);
    mError = "";
    return true;
}

ct::String Project::buildManifest() const
{
    ct::Json document = ct::Json::object();
    document["format"] = ct::Json(kFormatName);
    document["version"] = ct::Json(kFormatVersion);
    document["name"] = ct::Json(mName);
    document["assetRoot"] = ct::Json(mAssetRoot);
    ct::Json scenes = ct::Json::array();
    for (std::size_t i = 0; i < mScenes.size(); ++i) scenes.push_back(ct::Json(mScenes[i]));
    document["scenes"] = scenes;
    document["activeScene"] = ct::Json(mMainScene);
    ct::Json searchPaths = ct::Json::array();
    for (std::size_t i = 0; i < mSearchPaths.size(); ++i)
        searchPaths.push_back(ct::Json(mSearchPaths[i]));
    document["searchPaths"] = searchPaths;

    ct::Json settings = ct::Json::object();
    const ct::Vector<ProjectSettingInfo>& table = settingTable();
    for (std::size_t i = 0; i < table.size(); ++i)
        if (mValues[i] != table[i].defaultValue)
            settings[table[i].name] = scenejson::fromValue(mValues[i]);
    for (std::size_t i = 0; i < mInputActions.size(); ++i)
        settings[ct::String(kInputPrefix) + mInputActions[i].name] =
                writeInputAction(mInputActions[i]);
    if (mExtraSettings.is_object())
    {
        const ct::Json::Object& extra = mExtraSettings.members();
        for (std::size_t i = 0; i < extra.size(); ++i) settings[extra[i].key] = extra[i].value;
    }
    document["settings"] = settings;

    ct::String text = document.dump(2);
    text += '\n';
    return text;
}

bool Project::save()
{
    if (!mOpen) return fail("no project is open");
    if (!FileSystem::writeText(mManifestPath, buildManifest()))
        return fail(ct::String("could not write '") + mManifestPath + "'");
    return true;
}

bool Project::create(const ct::String& parentDirectory, const ct::String& name)
{
    if (parentDirectory.empty()) return fail("the parent folder is empty");
    if (!validProjectName(name))
        return fail(ct::String("'") + name + "' is not a valid project name");
    ct::String parent;
    if (!absolute(parentDirectory, parent))
        return fail(ct::String("invalid folder '") + parentDirectory + "'");

    Project created;
    created.mName = name;
    created.mRoot = parent + "/" + name;
    created.mManifestPath = created.mRoot + "/" + kManifestName;
    if (file_exists(created.mManifestPath.c_str()))
        return fail(ct::String("a project already exists at '") + created.mRoot + "'");

    const char* const folders[] = { "", kScenesFolder, "Assets/Scripts", "Assets/Models",
        "Assets/Textures" };
    for (const char* folder: folders)
    {
        const ct::String path = folder[0] == '\0' ? created.mRoot : created.mRoot + "/" + folder;
        if (!fs_create_directory(path.c_str()))
            return fail(ct::String("could not create the folder '") + path + "'");
    }

    created.mOpen = true;
    if (!created.save()) return fail(created.mError);
    unregisterSearchPaths();
    *this = static_cast<Project&&>(created);
    mError = "";
    return true;
}

void Project::close()
{
    unregisterSearchPaths();
    Project empty;
    *this = static_cast<Project&&>(empty);
}

void Project::fillDeviceDesc(DeviceDesc& desc) const
{
    desc.title = mName.c_str();
    desc.width = static_cast<int>(integer("display/window/size/viewport_width"));
    desc.height = static_cast<int>(integer("display/window/size/viewport_height"));
    const std::int64_t widthOverride = integer("display/window/size/window_width_override");
    const std::int64_t heightOverride = integer("display/window/size/window_height_override");
    if (widthOverride > 0) desc.width = static_cast<int>(widthOverride);
    if (heightOverride > 0) desc.height = static_cast<int>(heightOverride);

    desc.mode = static_cast<WindowMode>(integer("display/window/size/mode"));
    desc.vsync = static_cast<VSyncMode>(integer("display/window/vsync/vsync_mode"));

    desc.flags = 0;
    if (!flag("display/window/size/resizable")) desc.flags |= kWindowFlagResizeDisabled;
    if (flag("display/window/size/borderless")) desc.flags |= kWindowFlagBorderless;
    if (flag("display/window/size/always_on_top")) desc.flags |= kWindowFlagAlwaysOnTop;

    switch (integer("display/window/size/initial_position_type"))
    {
        case 3:
            desc.screen = static_cast<int>(integer("display/window/size/initial_screen"));
            break;
        case 4:
            desc.screen = kScreenWithMouseFocus;
            break;
        case 5:
            desc.screen = kScreenWithKeyboardFocus;
            break;
        default:
            desc.screen = kScreenPrimary;
            break;
    }
}

void Project::applyToEngine(Engine& engine) const
{
    engine.setPhysicsTicksPerSecond(
            static_cast<int>(integer("physics/common/physics_ticks_per_second")));
    engine.setMaxPhysicsStepsPerFrame(
            static_cast<int>(integer("physics/common/max_physics_steps_per_frame")));
    engine.setPhysicsJitterFix(number("physics/common/physics_jitter_fix"));
    engine.setMaxFps(static_cast<int>(integer("application/run/max_fps")));
    engine.setFrameDelay(static_cast<std::uint32_t>(integer("application/run/frame_delay_msec")));
    engine.setLowProcessorUsageMode(flag("application/run/low_processor_mode"));
    engine.setLowProcessorUsageModeSleepUsec(
            static_cast<int>(integer("application/run/low_processor_mode_sleep_usec")));
}

void Project::applyToPhysics(PhysicsServer3D& physics) const
{
    const Math::Vec3 direction = get("physics/3d/default_gravity_vector").asVec3();
    const float magnitude = static_cast<float>(number("physics/3d/default_gravity"));
    physics.setGravity(
            Math::Vec3(direction.x * magnitude, direction.y * magnitude, direction.z * magnitude));
}

void Project::applyToInputMap(InputMap& map) const
{
    for (std::size_t i = 0; i < mInputActions.size(); ++i)
    {
        const ProjectInputAction& action = mInputActions[i];
        if (map.hasAction(action.name.c_str())) map.eraseAction(action.name.c_str());
        map.addAction(action.name.c_str(), action.deadzone);
        for (std::size_t j = 0; j < action.events.size(); ++j)
            map.actionAddEvent(action.name.c_str(), action.events[j]);
    }
}

void Project::applyToRendering(RenderingServer& server) const
{
    server.setDirectionalShadowSize(
            static_cast<std::uint32_t>(
                    integer("rendering/lights_and_shadows/directional_shadow/size")),
            server.directionalShadowSixteenBits());
    server.setDirectionalShadowQuality(static_cast<ShadowQuality>(
            integer("rendering/lights_and_shadows/directional_shadow/soft_shadow_filter_quality")));
    server.setPositionalShadowQuality(static_cast<ShadowQuality>(
            integer("rendering/lights_and_shadows/positional_shadow/soft_shadow_filter_quality")));
    server.setRoughnessLimiter(
            flag("rendering/anti_aliasing/screen_space_roughness_limiter/enabled"),
            static_cast<float>(
                    number("rendering/anti_aliasing/screen_space_roughness_limiter/amount")),
            static_cast<float>(
                    number("rendering/anti_aliasing/screen_space_roughness_limiter/limit")));
    server.setSsaoQuality(static_cast<SsaoQuality>(integer("rendering/environment/ssao/quality")),
            flag("rendering/environment/ssao/half_size"),
            static_cast<float>(number("rendering/environment/ssao/adaptive_target")),
            static_cast<std::uint32_t>(integer("rendering/environment/ssao/blur_passes")),
            static_cast<float>(number("rendering/environment/ssao/fadeout_from")),
            static_cast<float>(number("rendering/environment/ssao/fadeout_to")));
    server.setSsrHalfSize(flag("rendering/environment/screen_space_reflection/half_size"));
    server.setDepthOfFieldBokeh(
            static_cast<DofBokehShape>(
                    integer("rendering/camera/depth_of_field/depth_of_field_bokeh_shape")),
            static_cast<DofBlurQuality>(
                    integer("rendering/camera/depth_of_field/depth_of_field_bokeh_quality")),
            flag("rendering/camera/depth_of_field/depth_of_field_use_jitter"));
    server.setLightmapBicubicFilter(flag("rendering/lightmapping/lightmap_gi/use_bicubic_filter"));
    server.setSmaaEdgeDetectionThreshold(static_cast<float>(
            number("rendering/anti_aliasing/quality/smaa_edge_detection_threshold")));
}

void Project::applyToViewport(Viewport& viewport) const
{
    viewport.setMsaa3D(
            static_cast<ViewportMsaa>(integer("rendering/anti_aliasing/quality/msaa_3d")));
    viewport.setScaling3D(static_cast<float>(number("rendering/scaling_3d/scale")));
    viewport.setScaling3DSize(static_cast<std::uint32_t>(integer("rendering/scaling_3d/width")),
            static_cast<std::uint32_t>(integer("rendering/scaling_3d/height")));
    viewport.setScaling3DAspect(static_cast<int>(integer("rendering/scaling_3d/aspect")));
    viewport.setScaling3DMode(static_cast<int>(integer("rendering/scaling_3d/mode")));
    viewport.setFsrSharpness(static_cast<float>(number("rendering/scaling_3d/fsr_sharpness")));
    viewport.setScreenSpaceAA(
            static_cast<int>(integer("rendering/anti_aliasing/quality/screen_space_aa")));
    viewport.setMeshLodThreshold(
            static_cast<float>(number("rendering/mesh_lod/lod_change/threshold_pixels")));
    viewport.setPositionalShadowAtlasSize(
            static_cast<std::uint32_t>(
                    integer("rendering/lights_and_shadows/positional_shadow/atlas_size")),
            viewport.getPositionalShadowAtlasSixteenBits());
    for (std::uint32_t i = 0; i < 4; ++i)
        viewport.setPositionalShadowAtlasQuadrantSubdivision(i,
                atlasSubdivision(integer(kAtlasQuadrants[i])));
    viewport.setClearColor(get("rendering/environment/defaults/default_clear_color").asColor());
}

void Project::applyToViewport(RenderingServer& server, ViewportId viewport, bool clearColor) const
{
    server.viewportSetMsaa3D(viewport,
            static_cast<ViewportMsaa>(integer("rendering/anti_aliasing/quality/msaa_3d")));
    server.viewportSetScaling3D(viewport, static_cast<float>(number("rendering/scaling_3d/scale")));
    server.viewportSetScaling3DSize(viewport,
            static_cast<std::uint32_t>(integer("rendering/scaling_3d/width")),
            static_cast<std::uint32_t>(integer("rendering/scaling_3d/height")));
    server.viewportSetScaling3DAspect(viewport,
            static_cast<Scaling3DAspect>(integer("rendering/scaling_3d/aspect")));
    server.viewportSetScaling3DMode(viewport,
            static_cast<Scaling3DMode>(integer("rendering/scaling_3d/mode")));
    server.viewportSetFsrSharpness(viewport,
            static_cast<float>(number("rendering/scaling_3d/fsr_sharpness")));
    server.viewportSetFxaa(viewport,
            integer("rendering/anti_aliasing/quality/screen_space_aa") == 1);
    server.viewportSetSmaa(viewport,
            integer("rendering/anti_aliasing/quality/screen_space_aa") == 2);
    server.viewportSetMeshLodThreshold(viewport,
            static_cast<float>(number("rendering/mesh_lod/lod_change/threshold_pixels")));
    server.viewportSetPositionalShadowAtlasSize(viewport,
            static_cast<std::uint32_t>(
                    integer("rendering/lights_and_shadows/positional_shadow/atlas_size")),
            false);
    for (std::uint32_t i = 0; i < 4; ++i)
        server.viewportSetPositionalShadowAtlasQuadrantSubdivision(viewport, i,
                atlasSubdivision(integer(kAtlasQuadrants[i])));
    if (clearColor)
        server.viewportSetClearColor(viewport,
                get("rendering/environment/defaults/default_clear_color").asColor());
}

void Project::registerSearchPaths()
{
    if (!mOpen) return;
    FileSystem& files = FileSystem::getSingleton();
    ct::Vector<ct::String> wanted;
    wanted.push_back(mRoot);
    wanted.push_back(assetPath());
    const ct::Vector<ct::String> extra = resolvedSearchPaths();
    for (std::size_t i = 0; i < extra.size(); ++i) wanted.push_back(extra[i]);

    for (std::size_t i = 0; i < wanted.size(); ++i)
    {
        bool present = false;
        const ct::Vector<ct::String>& current = files.getSearchPaths();
        for (std::size_t j = 0; j < current.size(); ++j)
            if (current[j] == wanted[i]) present = true;
        if (present) continue;
        files.addSearchPath(wanted[i]);
        mRegisteredPaths.push_back(wanted[i]);
    }
}

void Project::unregisterSearchPaths()
{
    FileSystem& files = FileSystem::getSingleton();
    for (std::size_t i = 0; i < mRegisteredPaths.size(); ++i)
        files.removeSearchPath(mRegisteredPaths[i]);
    mRegisteredPaths.clear();
}

Node* Project::loadMainScene(Node* parent, SceneResult& result) const
{
    if (!mOpen)
    {
        result.addError("", "no project is open");
        return nullptr;
    }
    if (mMainScene.empty())
    {
        result.addError("activeScene", "the project has no main scene");
        return nullptr;
    }
    return SceneSerializer::loadFile(absolutePath(mMainScene), parent, result);
}

} // namespace prisma3d
