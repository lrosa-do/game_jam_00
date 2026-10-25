#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/InputMap.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTypes.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <ct/json.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += "/project_test";
    return root;
}

bool isFolder(const ct::String& path)
{
    PathInfo info;
    return fs_get_path_info(path.c_str(), &info) && info.type == PATH_TYPE_DIRECTORY;
}

bool isFile(const ct::String& path)
{
    PathInfo info;
    return fs_get_path_info(path.c_str(), &info) && info.type == PATH_TYPE_FILE;
}

ct::String readFile(const ct::String& path)
{
    return FileSystem::getSingleton().readText(path);
}

bool contains(const ct::String& text, const char* fragment)
{
    return strstr(text.c_str(), fragment) != nullptr;
}

bool refused(Project& project, const ct::String& manifest, const char* fragment)
{
    ct::String path = testRoot();
    path += "/refusals/projecto.json";
    CHECK(FileSystem::writeText(path, manifest));
    const bool opened = project.open(path);
    if (opened || !contains(project.error(), fragment))
    {
        printf("expected refusal containing '%s', got %s '%s'\n", fragment,
                opened ? "an opened project" : "the error", project.error().c_str());
        ++failures;
        return false;
    }
    return true;
}

InputEvent keyEvent(Key key, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = key;
    event.key.scancode = static_cast<int>(key);
    event.key.modifiers = modifiers;
    return event;
}

InputEvent mouseEvent(MouseButton button, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.button = button;
    event.mouseButton.modifiers = modifiers;
    return event;
}

bool sameEvent(const InputEvent& a, const InputEvent& b)
{
    if (a.type != b.type) return false;
    if (a.type == InputEventType::Key)
        return a.key.key == b.key.key && a.key.modifiers == b.key.modifiers;
    if (a.type == InputEventType::MouseButton)
        return a.mouseButton.button == b.mouseButton.button &&
               a.mouseButton.modifiers == b.mouseButton.modifiers;
    return false;
}

void testSettingTable()
{
    CHECK(Project::settingCount() >= 30);
    for (size_t i = 0; i < Project::settingCount(); ++i)
    {
        const ProjectSettingInfo& info = Project::settingInfo(i);
        CHECK(info.defaultValue.type() == info.type);
        for (size_t j = i + 1; j < Project::settingCount(); ++j)
            CHECK(info.name != Project::settingInfo(j).name);
        Project project;
        CHECK(project.has(info.name));
        CHECK(project.isDefault(info.name));
        CHECK(project.get(info.name) == info.defaultValue);
        CHECK(project.set(info.name, info.defaultValue));
    }

    Project project;
    CHECK(project.get("display/window/size/viewport_width").asInt() == 1152);
    CHECK(project.get("display/window/size/viewport_height").asInt() == 648);
    CHECK(project.get("display/window/size/mode").asInt() == 0);
    CHECK(project.get("display/window/size/resizable").asBool());
    CHECK(project.get("display/window/vsync/vsync_mode").asInt() == 1);
    CHECK(project.get("physics/common/physics_ticks_per_second").asInt() == 60);
    CHECK(project.get("physics/common/max_physics_steps_per_frame").asInt() == 8);
    CHECK(project.get("physics/common/physics_jitter_fix").asFloat() == 0.5);
    CHECK(project.get("physics/3d/default_gravity").asFloat() == 9.8);
    CHECK(project.get("physics/3d/default_gravity_vector").asVec3().y == -1.0f);
    CHECK(project.get("rendering/anti_aliasing/quality/msaa_3d").asInt() == 0);
    CHECK(project.get("rendering/scaling_3d/scale").asFloat() == 1.0);
    CHECK(project.get("rendering/lights_and_shadows/positional_shadow/atlas_size").asInt() == 4096);
    CHECK(project.get("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_2_subdiv")
                    .asInt() == 3);
    CHECK(project.get("application/run/max_fps").asInt() == 0);
    CHECK(project.get("application/run/low_processor_mode_sleep_usec").asInt() == 6900);

    CHECK(!project.has("display/window/size/nowhere"));
    CHECK(project.get("display/window/size/nowhere").isNil());
    CHECK(Project::findSetting("physics/3d/default_gravity") != nullptr);
    CHECK(Project::findSetting("physics/3d/nothing") == nullptr);
    CHECK(Project::getDefault("application/run/max_fps").asInt() == 0);

    CHECK(project.set("display/window/size/viewport_width", PropertyValue::fromInt(1920)));
    CHECK(!project.isDefault("display/window/size/viewport_width"));
    CHECK(project.get("display/window/size/viewport_width").asInt() == 1920);
    project.reset("display/window/size/viewport_width");
    CHECK(project.isDefault("display/window/size/viewport_width"));

    CHECK(project.set("display/window/size/viewport_width", PropertyValue::fromFloat(800.0)));
    CHECK(project.get("display/window/size/viewport_width").type() == PropertyType::Int);
    CHECK(!project.set("display/window/size/viewport_width", PropertyValue::fromFloat(800.5)));
    CHECK(contains(project.error(), "whole number"));
    CHECK(project.get("display/window/size/viewport_width").asInt() == 800);
    CHECK(!project.set("display/window/size/viewport_width", PropertyValue::fromInt(0)));
    CHECK(contains(project.error(), "out of range"));
    CHECK(!project.set("display/window/size/viewport_width", PropertyValue::fromString("wide")));
    CHECK(contains(project.error(), "expects Int, got String"));
    CHECK(!project.set("display/window/size/mode", PropertyValue::fromInt(9)));
    CHECK(contains(project.error(), "not one of the supported values"));
    CHECK(!project.set("display/window/size/initial_position_type", PropertyValue::fromInt(0)));
    CHECK(!project.set("display/window/size/initial_position_type", PropertyValue::fromInt(2)));
    CHECK(project.set("display/window/size/initial_position_type", PropertyValue::fromInt(3)));
    CHECK(project.set("display/window/vsync/vsync_mode", PropertyValue::fromInt(2)));
    CHECK(!project.set("display/window/vsync/vsync_mode", PropertyValue::fromInt(3)));
    CHECK(!project.set("physics/3d/default_gravity", PropertyValue::fromFloat(INFINITY)));
    CHECK(!project.set("physics/3d/default_gravity", PropertyValue::fromFloat(NAN)));
    CHECK(!project.set("physics/common/physics_ticks_per_second", PropertyValue::fromInt(0)));
    CHECK(!project.set("physics/common/physics_ticks_per_second", PropertyValue::fromInt(1001)));
    CHECK(!project.set("rendering/scaling_3d/scale", PropertyValue::fromFloat(0.1)));
    CHECK(project.set("rendering/scaling_3d/scale", PropertyValue::fromInt(2)));
    CHECK(project.get("rendering/scaling_3d/scale").type() == PropertyType::Float);
    CHECK(!project.set("physics/3d/default_gravity_vector", PropertyValue::fromFloat(1.0)));
    CHECK(!project.set("nothing/here", PropertyValue::fromInt(1)));
    CHECK(contains(project.error(), "unknown setting"));
}

void testCreate()
{
    removeTree(testRoot());
    ct::String parent = testRoot();
    parent += "/made";

    Project project;
    CHECK(!project.isOpen());
    CHECK(project.create(parent, "Alpha"));
    CHECK(project.error().empty());
    CHECK(project.isOpen());
    CHECK(project.name() == "Alpha");
    CHECK(project.root().ends_with("/made/Alpha"));
    CHECK(project.manifestPath() == project.root() + "/projecto.json");
    CHECK(project.assetRoot() == "Assets");
    CHECK(project.scenes().size() == 0);
    CHECK(project.mainScene().empty());
    CHECK(project.searchPaths().size() == 0);

    CHECK(isFolder(project.root()));
    CHECK(isFolder(project.root() + "/Scenes"));
    CHECK(isFolder(project.root() + "/Assets"));
    CHECK(isFolder(project.root() + "/Assets/Scripts"));
    CHECK(isFolder(project.root() + "/Assets/Models"));
    CHECK(isFolder(project.root() + "/Assets/Textures"));
    CHECK(isFile(project.root() + "/projecto.json"));
    CHECK(!isFile(project.root() + "/projecto.json.tmp"));

    const ct::Json manifest = ct::Json::parse(readFile(project.manifestPath()));
    CHECK(manifest.is_object());
    CHECK(manifest["format"].str() == "prisma3d-project");
    CHECK(manifest["version"].as_int() == 1);
    CHECK(manifest["name"].str() == "Alpha");
    CHECK(manifest["assetRoot"].str() == "Assets");
    CHECK(manifest["scenes"].is_array() && manifest["scenes"].size() == 0);
    CHECK(manifest["activeScene"].is_string() && manifest["activeScene"].str().empty());
    CHECK(manifest["searchPaths"].is_array() && manifest["searchPaths"].size() == 0);
    CHECK(manifest["settings"].is_object() && manifest["settings"].size() == 0);

    Project again;
    CHECK(!again.create(parent, "Alpha"));
    CHECK(contains(again.error(), "already exists"));
    CHECK(!again.isOpen());

    const char* const bad[] = { "", ".", "..", "a/b", "a\\b", "c:", "x\ny" };
    for (const char* name: bad)
    {
        Project invalid;
        CHECK(!invalid.create(parent, name));
        CHECK(contains(invalid.error(), "not a valid project name"));
        CHECK(!invalid.isOpen());
    }
    Project noParent;
    CHECK(!noParent.create("", "Beta"));

    Project blocked;
    ct::String file = parent + "/plain.txt";
    CHECK(FileSystem::writeText(file, "x"));
    CHECK(!blocked.create(file, "Gamma"));
    CHECK(!blocked.isOpen());
    CHECK(!blocked.error().empty());

    Project reopened;
    CHECK(reopened.open(project.manifestPath()));
    CHECK(reopened.name() == "Alpha");
    CHECK(reopened.root() == project.root());
    CHECK(reopened.warnings().size() == 0);

    Project fromFolder;
    CHECK(fromFolder.open(project.root()));
    CHECK(fromFolder.manifestPath() == project.manifestPath());
    ct::String folderSlash = project.root() + "/";
    Project fromSlash;
    CHECK(fromSlash.open(folderSlash));
    CHECK(fromSlash.root() == project.root());
}

void testSaveAndLoad()
{
    ct::String parent = testRoot();
    parent += "/saved";
    Project project;
    CHECK(project.create(parent, "Beta"));

    CHECK(project.set("display/window/size/viewport_width", PropertyValue::fromInt(1920)));
    CHECK(project.set("display/window/size/viewport_height", PropertyValue::fromInt(1080)));
    CHECK(project.set("display/window/size/mode", PropertyValue::fromInt(3)));
    CHECK(project.set("display/window/size/resizable", PropertyValue::fromBool(false)));
    CHECK(project.set("physics/common/physics_ticks_per_second", PropertyValue::fromInt(120)));
    CHECK(project.set("physics/common/physics_jitter_fix", PropertyValue::fromFloat(0.1)));
    CHECK(project.set("physics/3d/default_gravity", PropertyValue::fromFloat(3.7)));
    CHECK(project.set("physics/3d/default_gravity_vector",
            PropertyValue::fromVec3(Math::Vec3(0.0f, -0.5f, 0.25f))));
    CHECK(project.set("rendering/anti_aliasing/quality/msaa_3d", PropertyValue::fromInt(2)));
    CHECK(project.set("rendering/scaling_3d/scale", PropertyValue::fromFloat(0.75)));
    CHECK(project.set("rendering/environment/defaults/default_clear_color",
            PropertyValue::fromColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f))));
    CHECK(project.set("application/run/max_fps", PropertyValue::fromInt(144)));
    CHECK(project.set("display/window/size/borderless", PropertyValue::fromBool(false)));
    CHECK(project.isDefault("display/window/size/borderless"));

    CHECK(project.addScene("Scenes/main.scene.json"));
    CHECK(project.addScene("Scenes/level1.scene.json"));
    CHECK(project.setMainScene("Scenes/level1.scene.json"));
    CHECK(project.addSearchPath("shared/models"));
    CHECK(project.addSearchPath("/opt/assets/"));
    CHECK(project.setName("Beta game"));

    ProjectInputAction jump;
    jump.name = "jump";
    jump.deadzone = 0.5f;
    jump.events.push_back(keyEvent(Key::Space));
    jump.events.push_back(mouseEvent(MouseButton::Right, kModShift | kModControl));
    CHECK(project.setInputAction(jump));
    ProjectInputAction fire;
    fire.name = "fire";
    fire.events.push_back(mouseEvent(MouseButton::Left));
    fire.events.push_back(mouseEvent(MouseButton::X2));
    fire.events.push_back(keyEvent(Key::LeftControl, kModAlt | kModSuper));
    CHECK(project.setInputAction(fire));
    ProjectInputAction empty;
    empty.name = "nothing";
    CHECK(project.setInputAction(empty));

    CHECK(project.save());

    const ct::String text = readFile(project.manifestPath());
    const ct::Json manifest = ct::Json::parse(text);
    const ct::Json& settings = manifest["settings"];
    CHECK(settings.is_object());
    CHECK(settings.size() == 12 + 3);
    CHECK(settings.contains("display/window/size/viewport_width"));
    CHECK(settings.contains("physics/3d/default_gravity"));
    CHECK(settings.contains("rendering/environment/defaults/default_clear_color"));
    CHECK(!settings.contains("display/window/size/borderless"));
    CHECK(settings["display/window/size/viewport_height"].as_int() == 1080);
    CHECK(!settings.contains("physics/common/max_physics_steps_per_frame"));
    CHECK(!settings.contains("rendering/lights_and_shadows/directional_shadow/size"));
    CHECK(settings["physics/3d/default_gravity"].as_double() == 3.7);
    CHECK(settings["display/window/size/mode"].as_int() == 3);
    CHECK(settings["display/window/size/resizable"].as_bool() == false);
    CHECK(settings["physics/3d/default_gravity_vector"].is_array());
    CHECK(settings["input/jump"]["deadzone"].as_double() == 0.5);
    CHECK(settings["input/jump"]["events"].size() == 2);
    CHECK(settings["input/jump"]["events"][0]["type"].str() == "key");
    CHECK(settings["input/jump"]["events"][0]["keycode"].as_int() == 32);
    CHECK(settings["input/jump"]["events"][1]["type"].str() == "mouse_button");
    CHECK(settings["input/jump"]["events"][1]["button_index"].as_int() == 2);
    CHECK(settings["input/jump"]["events"][1]["shift_pressed"].as_bool());
    CHECK(settings["input/jump"]["events"][1]["ctrl_pressed"].as_bool());
    CHECK(!settings["input/jump"]["events"][1].contains("alt_pressed"));
    CHECK(manifest["scenes"].size() == 2);
    CHECK(manifest["activeScene"].str() == "Scenes/level1.scene.json");
    CHECK(manifest["searchPaths"][1].str() == "/opt/assets");
    CHECK(manifest["name"].str() == "Beta game");

    Project loaded;
    CHECK(loaded.open(project.manifestPath()));
    CHECK(loaded.warnings().size() == 0);
    CHECK(loaded.name() == "Beta game");
    CHECK(loaded.assetRoot() == "Assets");
    CHECK(loaded.scenes().size() == 2);
    CHECK(loaded.scenes()[0] == "Scenes/main.scene.json");
    CHECK(loaded.scenes()[1] == "Scenes/level1.scene.json");
    CHECK(loaded.mainScene() == "Scenes/level1.scene.json");
    CHECK(loaded.searchPaths().size() == 2);
    CHECK(loaded.searchPaths()[0] == "shared/models");
    CHECK(loaded.searchPaths()[1] == "/opt/assets");
    for (size_t i = 0; i < Project::settingCount(); ++i)
    {
        const ProjectSettingInfo& info = Project::settingInfo(i);
        if (!(loaded.get(info.name) == project.get(info.name)))
        {
            printf("setting '%s' changed on the round trip\n", info.name.c_str());
            ++failures;
        }
    }
    CHECK(loaded.get("physics/3d/default_gravity").asFloat() == 3.7);
    CHECK(loaded.get("physics/common/physics_jitter_fix").asFloat() == 0.1);
    CHECK(loaded.get("rendering/scaling_3d/scale").asFloat() == 0.75);
    CHECK(loaded.get("display/window/size/resizable").asBool() == false);
    CHECK(loaded.inputActions().size() == 3);
    const ProjectInputAction* loadedJump = loaded.findInputAction("jump");
    CHECK(loadedJump != nullptr);
    if (loadedJump)
    {
        CHECK(loadedJump->deadzone == 0.5f);
        CHECK(loadedJump->events.size() == 2);
        if (loadedJump->events.size() == 2)
        {
            CHECK(sameEvent(loadedJump->events[0], jump.events[0]));
            CHECK(sameEvent(loadedJump->events[1], jump.events[1]));
        }
    }
    const ProjectInputAction* loadedFire = loaded.findInputAction("fire");
    CHECK(loadedFire != nullptr && loadedFire->events.size() == 3);
    if (loadedFire && loadedFire->events.size() == 3)
    {
        CHECK(loadedFire->deadzone == InputMap::kDefaultDeadzone);
        for (size_t i = 0; i < 3; ++i) CHECK(sameEvent(loadedFire->events[i], fire.events[i]));
    }
    const ProjectInputAction* loadedEmpty = loaded.findInputAction("nothing");
    CHECK(loadedEmpty != nullptr && loadedEmpty->events.size() == 0);

    CHECK(loaded.save());
    CHECK(readFile(loaded.manifestPath()) == text);

    loaded.reset("physics/3d/default_gravity");
    loaded.reset("rendering/environment/defaults/default_clear_color");
    CHECK(loaded.removeInputAction("fire"));
    CHECK(!loaded.removeInputAction("fire"));
    CHECK(loaded.save());
    const ct::Json smaller = ct::Json::parse(readFile(loaded.manifestPath()));
    CHECK(!smaller["settings"].contains("physics/3d/default_gravity"));
    CHECK(!smaller["settings"].contains("input/fire"));
    CHECK(smaller["settings"].size() == 12 + 3 - 3);

    Project unsaved;
    CHECK(!unsaved.save());
    CHECK(contains(unsaved.error(), "no project is open"));

    loaded.close();
    CHECK(!loaded.isOpen());
    CHECK(loaded.name().empty());
    CHECK(loaded.scenes().size() == 0);
    CHECK(loaded.inputActions().size() == 0);
    CHECK(loaded.isDefault("physics/common/physics_ticks_per_second"));
}

void testScenesAndPaths()
{
    ct::String parent = testRoot();
    parent += "/scenes";
    Project project;
    CHECK(project.create(parent, "Gamma"));

    CHECK(project.addScene("Scenes\\a.scene.json"));
    CHECK(project.scenes()[0] == "Scenes/a.scene.json");
    CHECK(project.addScene("./Scenes//b/./c.scene.json"));
    CHECK(project.scenes()[1] == "Scenes/b/c.scene.json");
    CHECK(project.hasScene("Scenes/a.scene.json"));
    CHECK(project.hasScene("Scenes\\a.scene.json"));
    CHECK(!project.hasScene("Scenes/zzz.scene.json"));
    CHECK(!project.addScene("Scenes/a.scene.json"));
    CHECK(contains(project.error(), "already a scene"));
    CHECK(!project.addScene(""));
    CHECK(!project.addScene("../outside.scene.json"));
    CHECK(contains(project.error(), ".."));
    CHECK(!project.addScene("Scenes/../../outside.scene.json"));
    CHECK(!project.addScene("/etc/passwd"));
    CHECK(contains(project.error(), "relative"));
    CHECK(!project.addScene("C:/windows/x.json"));
    CHECK(!project.addScene("\\share\\x.json"));
    CHECK(!project.addScene("."));
    CHECK(project.scenes().size() == 2);

    CHECK(!project.setMainScene("Scenes/zzz.scene.json"));
    CHECK(contains(project.error(), "not listed in scenes"));
    CHECK(project.mainScene().empty());
    CHECK(project.setMainScene("Scenes\\a.scene.json"));
    CHECK(project.mainScene() == "Scenes/a.scene.json");

    CHECK(project.renameScene("Scenes/a.scene.json", "Scenes/start.scene.json"));
    CHECK(project.mainScene() == "Scenes/start.scene.json");
    CHECK(project.scenes()[0] == "Scenes/start.scene.json");
    CHECK(!project.renameScene("Scenes/nope.scene.json", "Scenes/x.scene.json"));
    CHECK(!project.renameScene("Scenes/start.scene.json", "Scenes/b/c.scene.json"));
    CHECK(!project.renameScene("Scenes/start.scene.json", "../x"));
    CHECK(project.renameScene("Scenes/start.scene.json", "Scenes/start.scene.json"));
    CHECK(project.mainScene() == "Scenes/start.scene.json");

    CHECK(project.removeScene("Scenes/b/c.scene.json"));
    CHECK(project.mainScene() == "Scenes/start.scene.json");
    CHECK(!project.removeScene("Scenes/b/c.scene.json"));
    CHECK(project.removeScene("Scenes/start.scene.json"));
    CHECK(project.mainScene().empty());
    CHECK(project.scenes().size() == 0);
    CHECK(project.addScene("Scenes/x.scene.json"));
    CHECK(project.setMainScene("Scenes/x.scene.json"));
    CHECK(project.setMainScene(""));
    CHECK(project.mainScene().empty());

    CHECK(project.setAssetRoot("Assets/Game"));
    CHECK(project.assetRoot() == "Assets/Game");
    CHECK(project.assetPath() == project.root() + "/Assets/Game");
    CHECK(!project.setAssetRoot(".."));
    CHECK(!project.setAssetRoot("/abs"));
    CHECK(!project.setAssetRoot(""));
    CHECK(project.assetRoot() == "Assets/Game");
    CHECK(project.setAssetRoot("Assets"));

    CHECK(!project.setName(""));
    CHECK(project.addSearchPath("extra/models/"));
    CHECK(project.searchPaths()[0] == "extra/models");
    CHECK(!project.addSearchPath("extra\\models"));
    CHECK(!project.addSearchPath(""));
    CHECK(project.addSearchPath("/opt/shared"));
    CHECK(project.addSearchPath("../beside"));
    const ct::Vector<ct::String> resolved = project.resolvedSearchPaths();
    CHECK(resolved.size() == 3);
    if (resolved.size() == 3)
    {
        CHECK(resolved[0] == project.root() + "/extra/models");
        CHECK(resolved[1] == "/opt/shared");
        CHECK(resolved[2].ends_with("/scenes/beside"));
    }
    ct::String relative;
    CHECK(project.relativePath(project.root() + "/Assets/Models/a.glb", relative));
    CHECK(relative == "Assets/Models/a.glb");
    CHECK(project.relativePath(project.root() + "/Assets/../Scenes//x.json", relative));
    CHECK(relative == "Scenes/x.json");
    CHECK(project.relativePath("Assets\\Textures\\t.png", relative));
    CHECK(relative == "Assets/Textures/t.png");
    CHECK(project.relativePath("./Assets/./t.png", relative));
    CHECK(relative == "Assets/t.png");
    CHECK(!project.relativePath("../outside.png", relative));
    CHECK(!project.relativePath("/etc/passwd", relative));
    CHECK(!project.relativePath(project.root(), relative));
    CHECK(!project.relativePath(project.root() + "/", relative));
    CHECK(!project.relativePath(project.root() + "/../beside/a.png", relative));
    CHECK(!project.relativePath(project.root() + "extra/a.png", relative));
    CHECK(!project.relativePath("", relative));
    Project closed;
    CHECK(!closed.relativePath("Assets/a.png", relative));

    CHECK(project.removeSearchPath("/opt/shared/"));
    CHECK(!project.removeSearchPath("/opt/shared"));
    CHECK(project.searchPaths().size() == 2);

    ct::String normalized;
    ct::String reason;
    CHECK(Project::normalizeRelativePath("a/./b//c/", normalized, reason) && normalized == "a/b/c");
    CHECK(!Project::normalizeRelativePath("a/b/..", normalized, reason));
    CHECK(!Project::normalizeRelativePath("..", normalized, reason));
}

void testFileSystemRegistration()
{
    ct::String parent = testRoot();
    parent += "/registered";
    Project project;
    CHECK(project.create(parent, "Delta"));
    CHECK(FileSystem::writeText(project.assetPath() + "/Textures/only_in_assets.txt", "a"));
    CHECK(FileSystem::writeText(project.root() + "/at_the_root.txt", "r"));
    CHECK(FileSystem::writeText(project.root() + "/extra/in_extra.txt", "e"));
    CHECK(project.addSearchPath("extra"));
    CHECK(project.save());

    FileSystem& files = FileSystem::getSingleton();
    CHECK(!files.exists("Textures/only_in_assets.txt"));
    CHECK(!files.exists("at_the_root.txt"));
    CHECK(!files.exists("in_extra.txt"));

    const size_t before = files.getSearchPaths().size();
    project.registerSearchPaths();
    CHECK(files.getSearchPaths().size() == before + 3);
    CHECK(files.exists("Textures/only_in_assets.txt"));
    CHECK(files.exists("at_the_root.txt"));
    CHECK(files.exists("in_extra.txt"));
    CHECK(files.exists("Assets/Textures/only_in_assets.txt"));

    project.registerSearchPaths();
    CHECK(files.getSearchPaths().size() == before + 3);

    project.unregisterSearchPaths();
    CHECK(files.getSearchPaths().size() == before);
    CHECK(!files.exists("Textures/only_in_assets.txt"));
    CHECK(!files.exists("in_extra.txt"));

    const ct::String root = project.root();
    files.addSearchPath(root);
    project.registerSearchPaths();
    CHECK(files.getSearchPaths().size() == before + 3);
    project.close();
    CHECK(files.getSearchPaths().size() == before + 1);
    CHECK(files.exists("at_the_root.txt"));
    files.removeSearchPath(root);
    CHECK(files.getSearchPaths().size() == before);
    CHECK(!files.exists("at_the_root.txt"));

    Project second;
    CHECK(second.open(parent + "/Delta"));
    second.registerSearchPaths();
    CHECK(files.exists("in_extra.txt"));
    CHECK(second.open(parent + "/Delta"));
    CHECK(!files.exists("in_extra.txt"));
    CHECK(files.getSearchPaths().size() == before);
}

void testRefusals()
{
    Project project;
    ct::String parent = testRoot();
    parent += "/good";
    CHECK(project.create(parent, "Keep"));
    CHECK(project.addScene("Scenes/s.scene.json"));
    CHECK(project.setMainScene("Scenes/s.scene.json"));
    CHECK(project.save());

    const char* const prefix = "{\"format\": \"prisma3d-project\", \"version\": 1, ";

    CHECK(refused(project, "{\"format\": \"radion-project\", \"version\": 1}", "format"));
    CHECK(refused(project, "{\"version\": 1}", "format"));
    CHECK(refused(project, "{\"format\": 3, \"version\": 1}", "format"));
    CHECK(refused(project, "{\"format\": \"prisma3d-project\"}", "version"));
    CHECK(refused(project, "{\"format\": \"prisma3d-project\", \"version\": 0}", "version"));
    CHECK(refused(project, "{\"format\": \"prisma3d-project\", \"version\": \"one\"}", "version"));
    CHECK(refused(project, "{\"format\": \"prisma3d-project\", \"version\": 2}", "newer"));
    CHECK(refused(project, "[1, 2]", "not a JSON object"));
    CHECK(refused(project, "{\"format\": \"prisma3d-project\", \"version\": 1,", "not valid JSON"));
    CHECK(refused(project, "not json at all", "line 1"));
    CHECK(refused(project, "{\"format\"\n: }", "line 2"));

    ct::String text = prefix;
    CHECK(refused(project, text + "\"scenes\": [\"a.json\"], \"activeScene\": \"b.json\"}",
            "'b.json' is not listed in scenes"));
    CHECK(refused(project, text + "\"scenes\": [], \"activeScene\": \"b.json\"}",
            "not listed in scenes"));
    CHECK(refused(project,
            text + "\"scenes\": [\"a.json\"], \"activeScene\": \"Scenes/../a.json\"}", "'..'"));
    CHECK(refused(project, text + "\"scenes\": [\"a.json\"], \"activeScene\": \"/a.json\"}",
            "relative"));
    CHECK(refused(project, text + "\"scenes\": [\"../a.json\"]}", "scenes[0]"));
    CHECK(refused(project, text + "\"scenes\": [\"ok.json\", \"x/../../a.json\"]}", "scenes[1]"));
    CHECK(refused(project, text + "\"scenes\": [\"/etc/a.json\"]}", "relative"));
    CHECK(refused(project, text + "\"scenes\": [\"C:\\\\a.json\"]}", "relative"));
    CHECK(refused(project, text + "\"scenes\": [\"\"]}", "scenes[0]"));
    CHECK(refused(project, text + "\"scenes\": [3]}", "scenes[0]: not a text"));
    CHECK(refused(project, text + "\"scenes\": \"a.json\"}", "scenes: not a list"));
    CHECK(refused(project, text + "\"activeScene\": 4}", "activeScene: not a text"));
    CHECK(refused(project, text + "\"assetRoot\": \"../assets\"}", "assetRoot"));
    CHECK(refused(project, text + "\"assetRoot\": \"/usr/assets\"}", "relative"));
    CHECK(refused(project, text + "\"assetRoot\": 5}", "assetRoot: not a text"));
    CHECK(refused(project, text + "\"name\": []}", "name: not a text"));
    CHECK(refused(project, text + "\"searchPaths\": [4]}", "searchPaths[0]"));
    CHECK(refused(project, text + "\"searchPaths\": \"x\"}", "searchPaths: not a list"));
    CHECK(refused(project, text + "\"settings\": []}", "settings: not an object"));
    CHECK(refused(project,
            text + "\"settings\": {\"display/window/size/viewport_width\": \"wide\"}}",
            "settings.display/window/size/viewport_width"));
    CHECK(refused(project, text + "\"settings\": {\"display/window/size/viewport_width\": 0}}",
            "out of range"));
    CHECK(refused(project, text + "\"settings\": {\"display/window/size/mode\": 7}}",
            "not one of the supported values"));
    CHECK(refused(project, text + "\"settings\": {\"physics/3d/default_gravity\": \"inf\"}}",
            "finite"));
    CHECK(refused(project, text + "\"settings\": {\"physics/3d/default_gravity_vector\": [1, 2]}}",
            "[x, y, z]"));
    CHECK(refused(project, text + "\"settings\": {\"input/\": {}}}", "action name is empty"));
    CHECK(refused(project, text + "\"settings\": {\"input/jump\": 3}}",
            "input/jump: not an object"));
    CHECK(refused(project,
            text + "\"settings\": {\"input/jump\": {\"events\": [{\"type\": \"joypad\"}]}}}",
            "unknown event type 'joypad'"));
    CHECK(refused(project,
            text + "\"settings\": {\"input/jump\": {\"events\": [{\"type\": \"key\"}]}}}",
            "events[0].keycode"));
    CHECK(refused(project,
            text + "\"settings\": {\"input/jump\": {\"events\": [{\"type\": \"key\", \"keycode\": "
                   "9999}]}}}",
            "out of range"));
    CHECK(refused(project,
            text + "\"settings\": {\"input/jump\": {\"events\": [{\"type\": \"mouse_button\", "
                   "\"button_index\": 5}]}}}",
            "button_index"));
    CHECK(refused(project,
            text + "\"settings\": {\"input/jump\": {\"events\": [{\"type\": \"key\", \"keycode\": "
                   "32, \"shift_pressed\": 1.5}]}}}",
            "shift_pressed"));
    CHECK(refused(project, text + "\"settings\": {\"input/jump\": {\"deadzone\": 2.0}}}",
            "dead zone"));
    CHECK(refused(project, text + "\"settings\": {\"input/jump\": {\"events\": 1}}}",
            "events: not a list"));

    CHECK(project.isOpen());
    CHECK(project.name() == "Keep");
    CHECK(project.mainScene() == "Scenes/s.scene.json");

    Project missing;
    CHECK(!missing.open(testRoot() + "/nowhere/projecto.json"));
    CHECK(contains(missing.error(), "could not read"));
    CHECK(!missing.isOpen());
    CHECK(!missing.open(""));
    CHECK(!missing.open(testRoot() + "/made"));
    CHECK(contains(missing.error(), "could not read"));
    ct::String emptyFile = testRoot() + "/refusals/empty.json";
    CHECK(FileSystem::writeText(emptyFile, ""));
    CHECK(!missing.open(emptyFile));
    CHECK(contains(missing.error(), "could not read"));

    Project lenient;
    ct::String path = testRoot();
    path += "/refusals/lenient/projecto.json";
    CHECK(FileSystem::writeText(path,
            text + "\"future\": 1, \"scenes\": [\"a.json\", \"./a.json\"], \"searchPaths\": [\"\", "
                   "\"x\", \"x/\"],"
                   " \"settings\": {\"display/window/size/mode\": 3, \"future/setting\": {\"k\": "
                   "[1, 2]}}}"));
    CHECK(lenient.open(path));
    CHECK(lenient.name() == "lenient");
    CHECK(lenient.assetRoot() == "Assets");
    CHECK(lenient.mainScene().empty());
    CHECK(lenient.scenes().size() == 1);
    CHECK(lenient.searchPaths().size() == 1);
    CHECK(lenient.get("display/window/size/mode").asInt() == 3);
    CHECK(lenient.warnings().size() == 5);
    bool unknownField = false;
    bool unknownSetting = false;
    for (size_t i = 0; i < lenient.warnings().size(); ++i)
    {
        if (contains(lenient.warnings()[i], "future: unknown field")) unknownField = true;
        if (contains(lenient.warnings()[i], "settings.future/setting: unknown setting"))
            unknownSetting = true;
    }
    CHECK(unknownField);
    CHECK(unknownSetting);
    CHECK(lenient.save());
    const ct::Json kept = ct::Json::parse(readFile(path));
    CHECK(kept["settings"]["future/setting"]["k"][1].as_int() == 2);
    CHECK(kept["settings"]["display/window/size/mode"].as_int() == 3);
    CHECK(!kept.contains("future"));
    CHECK(kept["name"].str() == "lenient");
}

void testDeviceDesc()
{
    Project project;
    DeviceDesc defaults;
    project.fillDeviceDesc(defaults);
    CHECK(defaults.width == 1152 && defaults.height == 648);
    CHECK(defaults.mode == prisma3d::WindowMode::Windowed);
    CHECK(defaults.vsync == VSyncMode::Enabled);
    CHECK(defaults.flags == 0);
    CHECK(defaults.screen == kScreenPrimary);
    CHECK(!defaults.headless);
    CHECK(defaults.driver == prisma::DriverType::Vulkan);

    ct::String parent = testRoot();
    parent += "/desc";
    CHECK(project.create(parent, "Window game"));
    CHECK(project.set("display/window/size/viewport_width", PropertyValue::fromInt(640)));
    CHECK(project.set("display/window/size/viewport_height", PropertyValue::fromInt(360)));
    DeviceDesc small;
    small.headless = true;
    small.driver = prisma::DriverType::Vulkan;
    project.fillDeviceDesc(small);
    CHECK(small.width == 640 && small.height == 360);
    CHECK(strcmp(small.title, "Window game") == 0);
    CHECK(small.headless);
    CHECK(small.driver == prisma::DriverType::Vulkan);

    CHECK(project.set("display/window/size/window_width_override", PropertyValue::fromInt(1600)));
    DeviceDesc partial;
    project.fillDeviceDesc(partial);
    CHECK(partial.width == 1600 && partial.height == 360);
    CHECK(project.set("display/window/size/window_height_override", PropertyValue::fromInt(900)));
    CHECK(project.set("display/window/size/mode", PropertyValue::fromInt(3)));
    CHECK(project.set("display/window/vsync/vsync_mode", PropertyValue::fromInt(0)));
    CHECK(project.set("display/window/size/resizable", PropertyValue::fromBool(false)));
    CHECK(project.set("display/window/size/borderless", PropertyValue::fromBool(true)));
    CHECK(project.set("display/window/size/always_on_top", PropertyValue::fromBool(true)));
    CHECK(project.set("display/window/size/initial_position_type", PropertyValue::fromInt(3)));
    CHECK(project.set("display/window/size/initial_screen", PropertyValue::fromInt(2)));
    DeviceDesc full;
    project.fillDeviceDesc(full);
    CHECK(full.width == 1600 && full.height == 900);
    CHECK(full.mode == prisma3d::WindowMode::Fullscreen);
    CHECK(full.vsync == VSyncMode::Disabled);
    CHECK(project.set("display/window/vsync/vsync_mode", PropertyValue::fromInt(2)));
    project.fillDeviceDesc(full);
    CHECK(full.vsync == VSyncMode::Mailbox);
    CHECK(full.flags ==
            (kWindowFlagResizeDisabled | kWindowFlagBorderless | kWindowFlagAlwaysOnTop));
    CHECK(full.screen == 2);

    CHECK(project.set("display/window/size/initial_position_type", PropertyValue::fromInt(4)));
    DeviceDesc mouse;
    project.fillDeviceDesc(mouse);
    CHECK(mouse.screen == kScreenWithMouseFocus);
    CHECK(project.set("display/window/size/initial_position_type", PropertyValue::fromInt(5)));
    DeviceDesc keyboard;
    project.fillDeviceDesc(keyboard);
    CHECK(keyboard.screen == kScreenWithKeyboardFocus);
    CHECK(project.set("display/window/size/mode", PropertyValue::fromInt(2)));
    DeviceDesc maximized;
    project.fillDeviceDesc(maximized);
    CHECK(maximized.mode == prisma3d::WindowMode::Maximized);
}

void testApply()
{
    Project project;
    ct::String parent = testRoot();
    parent += "/apply";
    CHECK(project.create(parent, "Apply"));

    Engine engine;
    CHECK(engine.getPhysicsTicksPerSecond() == 60);
    project.applyToEngine(engine);
    CHECK(engine.getPhysicsTicksPerSecond() == 60);
    CHECK(engine.getMaxPhysicsStepsPerFrame() == 8);
    CHECK(engine.getPhysicsJitterFix() == 0.5);
    CHECK(engine.getMaxFps() == 0);
    CHECK(engine.getFrameDelay() == 0);
    CHECK(!engine.isInLowProcessorUsageMode());
    CHECK(engine.getLowProcessorUsageModeSleepUsec() == 6900);

    CHECK(project.set("physics/common/physics_ticks_per_second", PropertyValue::fromInt(120)));
    CHECK(project.set("physics/common/max_physics_steps_per_frame", PropertyValue::fromInt(4)));
    CHECK(project.set("physics/common/physics_jitter_fix", PropertyValue::fromFloat(0.25)));
    CHECK(project.set("application/run/max_fps", PropertyValue::fromInt(144)));
    CHECK(project.set("application/run/frame_delay_msec", PropertyValue::fromInt(3)));
    CHECK(project.set("application/run/low_processor_mode", PropertyValue::fromBool(true)));
    CHECK(project.set("application/run/low_processor_mode_sleep_usec",
            PropertyValue::fromInt(1000)));
    project.applyToEngine(engine);
    CHECK(engine.getPhysicsTicksPerSecond() == 120);
    CHECK(engine.getMaxPhysicsStepsPerFrame() == 4);
    CHECK(engine.getPhysicsJitterFix() == 0.25);
    CHECK(engine.getMaxFps() == 144);
    CHECK(engine.getFrameDelay() == 3);
    CHECK(engine.isInLowProcessorUsageMode());
    CHECK(engine.getLowProcessorUsageModeSleepUsec() == 1000);

    PhysicsServer3D physics;
    physics.setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));
    Project plain;
    plain.applyToPhysics(physics);
    CHECK(physics.getGravity().x == 0.0f);
    CHECK(fabsf(physics.getGravity().y + 9.8f) < 1e-5f);
    CHECK(physics.getGravity().z == 0.0f);
    CHECK(project.set("physics/3d/default_gravity", PropertyValue::fromFloat(3.5)));
    project.applyToPhysics(physics);
    CHECK(fabsf(physics.getGravity().y + 3.5f) < 1e-5f);
    CHECK(project.set("physics/3d/default_gravity_vector",
            PropertyValue::fromVec3(Math::Vec3(1.0f, 0.0f, -1.0f))));
    project.applyToPhysics(physics);
    CHECK(fabsf(physics.getGravity().x - 3.5f) < 1e-5f);
    CHECK(physics.getGravity().y == 0.0f);
    CHECK(fabsf(physics.getGravity().z + 3.5f) < 1e-5f);

    InputMap map;
    map.addAction("jump", 0.9f);
    map.actionAddEvent("jump", keyEvent(Key::X));
    ProjectInputAction jump;
    jump.name = "jump";
    jump.deadzone = 0.4f;
    jump.events.push_back(keyEvent(Key::Space));
    jump.events.push_back(mouseEvent(MouseButton::Middle, kModShift));
    CHECK(project.setInputAction(jump));
    ProjectInputAction walk;
    walk.name = "walk";
    walk.events.push_back(keyEvent(Key::W));
    CHECK(project.setInputAction(walk));
    project.applyToInputMap(map);
    CHECK(map.hasAction("jump") && map.hasAction("walk"));
    CHECK(map.actionGetDeadzone("jump") == 0.4f);
    CHECK(map.actionGetDeadzone("walk") == InputMap::kDefaultDeadzone);
    const ct::Vector<InputEvent>* events = map.actionGetEvents("jump");
    CHECK(events != nullptr && events->size() == 2);
    if (events && events->size() == 2)
    {
        CHECK(sameEvent((*events)[0], jump.events[0]));
        CHECK(sameEvent((*events)[1], jump.events[1]));
    }
    CHECK(map.actionHasEvent("walk", keyEvent(Key::W)));
    CHECK(!map.actionHasEvent("jump", keyEvent(Key::X)));
    InputEvent pressed = keyEvent(Key::Space);
    pressed.key.pressed = true;
    CHECK(map.eventIsAction(pressed, "jump"));
    project.applyToInputMap(map);
    events = map.actionGetEvents("jump");
    CHECK(events != nullptr && events->size() == 2);

    SceneTree tree;
    tree.initialize();
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (window)
    {
        CHECK(project.set("rendering/anti_aliasing/quality/msaa_3d", PropertyValue::fromInt(2)));
        CHECK(project.set("rendering/scaling_3d/scale", PropertyValue::fromFloat(0.5)));
        CHECK(project.set("rendering/mesh_lod/lod_change/threshold_pixels",
                PropertyValue::fromFloat(2.5)));
        CHECK(project.set("rendering/lights_and_shadows/positional_shadow/atlas_size",
                PropertyValue::fromInt(2048)));
        CHECK(project.set("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_0_subdiv",
                PropertyValue::fromInt(1)));
        CHECK(project.set("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_1_subdiv",
                PropertyValue::fromInt(0)));
        CHECK(project.set("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_2_subdiv",
                PropertyValue::fromInt(5)));
        CHECK(project.set("rendering/lights_and_shadows/positional_shadow/atlas_quadrant_3_subdiv",
                PropertyValue::fromInt(6)));
        CHECK(project.set("rendering/environment/defaults/default_clear_color",
                PropertyValue::fromColor(Math::Vec4(0.25f, 0.5f, 0.75f, 1.0f))));
        project.applyToViewport(*window);
        CHECK(window->getMsaa3D() == ViewportMsaa::Msaa4x);
        CHECK(window->getScaling3D() == 0.5f);
        CHECK(window->getMeshLodThreshold() == 2.5f);
        CHECK(window->getPositionalShadowAtlasSize() == 2048);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(0) == 1);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(1) == 0);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(2) == 256);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(3) == 1024);
        CHECK(window->getClearColor().x == 0.25f && window->getClearColor().y == 0.5f &&
                window->getClearColor().z == 0.75f);
        Project defaults;
        defaults.applyToViewport(*window);
        CHECK(window->getMsaa3D() == ViewportMsaa::Disabled);
        CHECK(window->getScaling3D() == 1.0f);
        CHECK(window->getPositionalShadowAtlasSize() == 4096);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(0) == 4);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(1) == 4);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(2) == 16);
        CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(3) == 64);
    }
    tree.finalize();
}

void testMainScene()
{
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return;

    ct::String parent = testRoot();
    parent += "/run";
    Project project;
    CHECK(project.create(parent, "Run"));

    SceneResult closedResult;
    Project closed;
    CHECK(closed.loadMainScene(root, closedResult) == nullptr);
    CHECK(closedResult.errorCount() == 1);

    SceneResult noMain;
    CHECK(project.loadMainScene(root, noMain) == nullptr);
    CHECK(noMain.errorCount() == 1);
    CHECK(contains(noMain.describe(), "no main scene"));

    Node3D world;
    world.setName("world");
    Camera3D* camera = world.createChild<Camera3D>();
    camera->setName("eye");
    camera->setPerspective(48.0f, 0.3f, 900.0f);
    OmniLight3D* lamp = world.createChild<OmniLight3D>();
    lamp->setName("lamp");
    lamp->setEnergy(2.25f);
    lamp->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));

    CHECK(project.addScene("Scenes/main.scene.json"));
    CHECK(project.setMainScene("Scenes/main.scene.json"));
    SceneResult missing;
    CHECK(project.loadMainScene(root, missing) == nullptr);
    CHECK(contains(missing.describe(), "could not read"));

    SceneResult saved;
    CHECK(SceneSerializer::saveFile(&world, project.absolutePath("Scenes/main.scene.json"),
            &saved));
    CHECK(saved.success());
    CHECK(project.save());

    Project reopened;
    CHECK(reopened.open(project.manifestPath()));
    CHECK(reopened.mainScene() == "Scenes/main.scene.json");
    const size_t before = root->getChildCount();
    SceneResult loaded;
    Node* scene = reopened.loadMainScene(root, loaded);
    CHECK(loaded.diagnostics.size() == 0);
    CHECK(scene != nullptr);
    CHECK(root->getChildCount() == before + 1);
    if (scene)
    {
        CHECK(scene->getParent() == root);
        CHECK(scene->getName() == "world");
        CHECK(scene->castTo<Node3D>() != nullptr);
        CHECK(scene->getChildCount() == 2);
        if (scene->getChildCount() == 2)
        {
            Camera3D* loadedCamera = scene->getChild(0)->castTo<Camera3D>();
            CHECK(loadedCamera != nullptr && loadedCamera->getFov() == 48.0f);
            OmniLight3D* loadedLamp = scene->getChild(1)->castTo<OmniLight3D>();
            CHECK(loadedLamp != nullptr && loadedLamp->getEnergy() == 2.25f);
            CHECK(loadedLamp != nullptr && loadedLamp->getPosition().z == 3.0f);
        }
    }

    CHECK(FileSystem::writeText(reopened.absolutePath("Scenes/main.scene.json"), "{}"));
    SceneResult broken;
    CHECK(reopened.loadMainScene(root, broken) == nullptr);
    CHECK(broken.errorCount() >= 1);
    CHECK(root->getChildCount() == before + 1);
    tree.finalize();
}

} // namespace

int main()
{
    removeTree(testRoot());
    registerSceneTypes();

    testSettingTable();
    testCreate();
    testSaveAndLoad();
    testScenesAndPaths();
    testFileSystemRegistration();
    testRefusals();
    testDeviceDesc();
    testApply();
    testMainScene();

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("project ok\n");
    return 0;
}
