#include "prisma3d/editor/DefaultProject.h"

#include "prisma3d/editor/AssetImport.h"
#include "prisma3d/editor/CreateMenu.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/MaterialProperties.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/MaterialLibrary.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/PropertyValue.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

namespace prisma3d
{

namespace
{

const char* const kFloorName = "Floor";
const char* const kBoxName = "Box";
const char* const kSunName = "Sun";
const char* const kCameraName = "MainCamera";

ct::String materialFile(const char* name)
{
    return AssetImport::joinPath(DefaultProject::kMaterialFolder,
            ct::String(name) + MaterialProperties::kLibraryExtension);
}

bool writeMaterial(const Project& project, const char* name, const Math::Vec4& albedo,
        float roughness, ct::String& error)
{
    const ct::String library = project.absolutePath(materialFile(name));
    if (file_exists(library.c_str())) return true;
    MaterialLibraryData data;
    MaterialDef def;
    def.info.name = name;
    def.info.albedo = albedo;
    def.info.roughness = roughness;
    data.materials.push_back(def);
    return MaterialLibraryFormat::save(library, data, error);
}

MaterialId loadMaterial(const char* name, ct::String& error)
{
    sceneres::clearError();
    const MaterialId id = sceneres::loadMaterial(materialFile(name) + "::" + name);
    if (!id.valid()) error = sceneres::takeError();
    return id;
}

ct::String trimmedFolder(const ct::String& folder)
{
    ct::String result = folder;
    while (result.size() > 1 && (result.ends_with("/") || result.ends_with("\\")))
        result.pop_back();
    return result;
}

MeshInstance3D* addPrimitive(Node* parent, const char* name, const PrimitiveRecipe& recipe)
{
    MeshInstance3D* node = parent->createChild<MeshInstance3D>();
    if (!node) return nullptr;
    node->setName(name);
    NodeClassDB::setProperty(node, "mesh",
            PropertyValue::fromResourcePath(primitiveRecipePath(recipe)));
    return node;
}

bool writeIfMissing(const ct::String& absolute, const char* text, ct::String& error)
{
    if (file_exists(absolute.c_str())) return true;
    if (file_write_text(absolute.c_str(), text)) return true;
    error = ct::String("could not write '") + absolute + "'";
    return false;
}

struct ScriptTemplate
{
    const char* path;
    const char* text;
};

const ScriptTemplate kCameraScripts[] = {
    { "Assets/Scripts/free_camera.py", R"zen(import math

class FreeCamera(Camera3D):
    speed = 8.0
    fast_multiplier = 3.0
    sensitivity = 0.15
    _yaw = 0.0
    _pitch = 0.0

    def _ready(self):
        self.make_current()
        self._yaw = self.rotation_degrees.y
        self._pitch = self.rotation_degrees.x

    def _exit_tree(self):
        mouse_set_mode(MOUSE_MODE_VISIBLE)

    def _process(self, delta):
        if mouse_check_button_pressed(MOUSE_RIGHT):
            mouse_set_mode(MOUSE_MODE_CAPTURED)
        if mouse_check_button_released(MOUSE_RIGHT):
            mouse_set_mode(MOUSE_MODE_VISIBLE)
        if mouse_check_button(MOUSE_RIGHT):
            move = mouse_delta()
            self._yaw = self._yaw - move.x * self.sensitivity
            self._pitch = math.clamp(self._pitch - move.y * self.sensitivity, -89.0, 89.0)
            self.rotation_degrees = Vec3(self._pitch, self._yaw, 0.0)
        step = self.speed * delta
        if keyboard_check(KEY_LEFT_SHIFT):
            step = step * self.fast_multiplier
        if keyboard_check(KEY_W):
            self.move_forward(step)
        if keyboard_check(KEY_S):
            self.move_forward(-step)
        if keyboard_check(KEY_D):
            self.move_right(step)
        if keyboard_check(KEY_A):
            self.move_right(-step)
        if keyboard_check(KEY_E):
            self.move_up(step)
        if keyboard_check(KEY_Q):
            self.move_up(-step)
)zen" },
    { "Assets/Scripts/orbit_camera.py", R"zen(import math

class OrbitCamera(Camera3D):
    target_path = ""
    height = 1.0
    distance = 6.0
    min_distance = 1.5
    max_distance = 30.0
    zoom_step = 0.8
    sensitivity = 0.25
    smoothing = 12.0
    yaw = 30.0
    pitch = 20.0
    _shown_distance = 6.0

    def _ready(self):
        self.make_current()
        self._shown_distance = self.distance

    def _exit_tree(self):
        mouse_set_mode(MOUSE_MODE_VISIBLE)

    def _process(self, delta):
        orbiting = mouse_check_button(MOUSE_MIDDLE) or mouse_check_button(MOUSE_RIGHT)
        if orbiting and mouse_get_mode() != MOUSE_MODE_CAPTURED:
            mouse_set_mode(MOUSE_MODE_CAPTURED)
        if not orbiting and mouse_get_mode() == MOUSE_MODE_CAPTURED:
            mouse_set_mode(MOUSE_MODE_VISIBLE)
        if orbiting:
            move = mouse_delta()
            self.yaw = self.yaw - move.x * self.sensitivity
            self.pitch = math.clamp(self.pitch + move.y * self.sensitivity, -85.0, 85.0)
        wheel = mouse_wheel()
        if wheel != 0.0:
            self.distance = math.clamp(self.distance - wheel * self.zoom_step, self.min_distance, self.max_distance)
        pivot = Vec3(0.0, self.height, 0.0)
        if self.target_path != "":
            target = self.get_node(self.target_path)
            if target != None:
                pivot = target.global_position + Vec3(0.0, self.height, 0.0)
        blend = 1.0 - math.exp(-self.smoothing * delta)
        self._shown_distance = math.lerp(self._shown_distance, self.distance, blend)
        yaw = math.radians(self.yaw)
        pitch = math.radians(self.pitch)
        direction = Vec3(math.sin(yaw) * math.cos(pitch), math.sin(pitch), math.cos(yaw) * math.cos(pitch))
        self.global_position = pivot + direction * self._shown_distance
        self.look_at(pivot)
)zen" },
    { "Assets/Scripts/third_person_camera.py", R"zen(import math

class ThirdPersonCamera(Camera3D):
    target_path = ""
    height = 1.6
    distance = 5.0
    margin = 0.3
    sensitivity = 0.2
    min_pitch = -20.0
    max_pitch = 70.0
    spring = 10.0
    yaw = 0.0
    pitch = 20.0
    _arm_length = 5.0

    def _ready(self):
        self.make_current()
        self._arm_length = self.distance

    def _exit_tree(self):
        mouse_set_mode(MOUSE_MODE_VISIBLE)

    def _process(self, delta):
        if mouse_check_button_pressed(MOUSE_ANY):
            mouse_set_mode(MOUSE_MODE_CAPTURED)
        if keyboard_check_pressed(KEY_ESCAPE):
            mouse_set_mode(MOUSE_MODE_VISIBLE)
        if mouse_get_mode() == MOUSE_MODE_CAPTURED:
            move = mouse_delta()
            self.yaw = self.yaw - move.x * self.sensitivity
            self.pitch = math.clamp(self.pitch + move.y * self.sensitivity, self.min_pitch, self.max_pitch)
        if self.target_path == "":
            return
        target = self.get_node(self.target_path)
        if target == None:
            return
        pivot = target.global_position + Vec3(0.0, self.height, 0.0)
        yaw = math.radians(self.yaw)
        pitch = math.radians(self.pitch)
        direction = Vec3(math.sin(yaw) * math.cos(pitch), math.sin(pitch), math.cos(yaw) * math.cos(pitch))
        reach = pivot + direction * (self.distance + self.margin)
        if target.is_class("PhysicsBody3D"):
            hit = raycast(pivot, reach, None, target)
        else:
            hit = raycast(pivot, reach)
        wanted = self.distance
        if hit != None:
            wanted = math.clamp(hit.distance - self.margin, 0.2, self.distance)
        if wanted < self._arm_length:
            self._arm_length = wanted
        else:
            self._arm_length = math.lerp(self._arm_length, wanted, 1.0 - math.exp(-self.spring * delta))
        self.global_position = pivot + direction * self._arm_length
        self.look_at(pivot)
)zen" },
};

} // namespace

ct::String DefaultProject::parentFolder(const ct::String& appFolder)
{
    return trimmedFolder(appFolder) + "/" + kFolder;
}

ct::String DefaultProject::manifestPath(const ct::String& appFolder)
{
    return parentFolder(appFolder) + "/" + kName + "/" + Project::kManifestName;
}

bool DefaultProject::exists(const ct::String& appFolder)
{
    return file_exists(manifestPath(appFolder).c_str());
}

bool DefaultProject::wanted(const char* projectArgument, const ct::String& lastProject)
{
    if (projectArgument) return false;
    return lastProject.empty() || !file_exists(lastProject.c_str());
}

bool DefaultProject::open(EditorDocument& document, const ct::String& appFolder)
{
    if (!exists(appFolder)) return create(document, appFolder);
    return document.openProject(manifestPath(appFolder));
}

bool DefaultProject::create(EditorDocument& document, const ct::String& appFolder)
{
    if (!document.createProject(parentFolder(appFolder), kName)) return false;
    ct::String error;
    if (!writeAssets(document, error) || !buildScene(document, error) || !document.saveScene())
    {
        if (!error.empty()) log_error("editor: default project: %s", error.c_str());
        return false;
    }
    return true;
}

bool DefaultProject::writeAssets(EditorDocument& document, ct::String& error)
{
    const Project& project = document.project();
    if (!writeIfMissing(project.absolutePath(kScriptPath), kScriptText, error)) return false;
    for (const ScriptTemplate& script: kCameraScripts)
        if (!writeIfMissing(project.absolutePath(script.path), script.text, error)) return false;

    const ct::String folder = project.absolutePath(kMaterialFolder);
    if (!fs_create_directory(folder.c_str()))
    {
        error = ct::String("could not create '") + folder + "'";
        return false;
    }
    return writeMaterial(project, kMaterialName, Math::Vec4(0.95f, 0.45f, 0.10f, 1.0f), 0.55f,
                   error) &&
           writeMaterial(project, kFloorMaterialName, Math::Vec4(0.42f, 0.44f, 0.46f, 1.0f), 0.85f,
                   error);
}

bool DefaultProject::buildScene(EditorDocument& document, ct::String& error)
{
    Node* scene = document.scene();
    if (!scene)
    {
        error = "there is no scene";
        return false;
    }

    PrimitiveRecipe plane;
    primitiveDefaults(PrimitiveKind::Plane, plane);
    plane.values[0] = 20.0f;
    plane.values[1] = 20.0f;
    PrimitiveRecipe cube;
    primitiveDefaults(PrimitiveKind::Box, cube);
    cube.values[0] = 1.0f;
    cube.values[1] = 1.0f;
    cube.values[2] = 1.0f;

    MeshInstance3D* floor = addPrimitive(scene, kFloorName, plane);
    MeshInstance3D* box = addPrimitive(scene, kBoxName, cube);
    DirectionalLight3D* sun = scene->createChild<DirectionalLight3D>();
    Camera3D* camera = scene->createChild<Camera3D>();
    if (!floor || !box || !sun || !camera)
    {
        error = "could not create the nodes of the test scene";
        return false;
    }

    box->setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));
    const MaterialId orange = loadMaterial(kMaterialName, error);
    const MaterialId ground = loadMaterial(kFloorMaterialName, error);
    if (!orange.valid() || !ground.valid()) return false;
    floor->setSurfaceMaterial(0, ground);
    box->setSurfaceMaterial(0, orange);
    if (!NodeClassDB::setProperty(box, "script", PropertyValue::fromResourcePath(kScriptPath)))
    {
        error = "could not attach the script to the box";
        return false;
    }

    sun->setName(kSunName);
    sun->setShadow(true);
    sun->lookAtFromPosition(CreateMenu::sunFrom(), CreateMenu::sunTarget());

    camera->setName(kCameraName);
    camera->lookAtFromPosition(Math::Vec3(0.0f, 3.0f, 6.0f), Math::Vec3(0.0f, 0.5f, 0.0f));
    camera->setCurrent(true);

    document.markDirty();
    return true;
}

} // namespace prisma3d
