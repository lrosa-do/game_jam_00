#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/MeshPropertyEditor.h"
#include "prisma3d/editor/PropertyRow.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/WidgetApp.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const float kEpsilon = 1.0e-3f;

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_types_test_vulkan" : "/editor_types_test";
    return root;
}

struct Editor
{
    SceneTree& tree;
    WidgetHost& host;
    EditorSelection& selection;
    SceneTreePanel& hierarchy;
    InspectorPanel& inspector;
    EditorDocument& document;
};

void frame(Editor& editor)
{
    editor.tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    editor.selection.update();
    editor.selection.flush();
    editor.hierarchy.update();
    editor.inspector.update();
    editor.host.frame(1.0f / 60.0f, true);
    Main::rendering()->endFrame(true);
}

void frames(Editor& editor, int count)
{
    for (int i = 0; i < count; ++i) frame(editor);
}

InputEvent motion(int x, int y)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = 0;
    event.mouseMotion.relativeY = 0;
    return event;
}

InputEvent button(int x, int y, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = MouseButton::Left;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = 0;
    return event;
}

void click(Editor& editor, int x, int y)
{
    editor.host.handleEvent(motion(x, y));
    frame(editor);
    editor.host.handleEvent(button(x, y, true));
    frame(editor);
    editor.host.handleEvent(button(x, y, false));
    frame(editor);
    frame(editor);
}

int centerX(const rt::Widget* widget)
{
    const rt::Rect rect = widget->absoluteRect();
    return static_cast<int>(rect.x + rect.w * 0.5f);
}

int centerY(const rt::Widget* widget)
{
    const rt::Rect rect = widget->absoluteRect();
    return static_cast<int>(rect.y + rect.h * 0.5f);
}

void reveal(Editor& editor, const rt::Widget* widget)
{
    rt::ScrollView* scroll = editor.inspector.scrollView();
    if (!scroll || !widget) return;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const rt::Rect view = scroll->absoluteRect();
        const rt::Rect box = widget->absoluteRect();
        float delta = 0.0f;
        if (box.y < view.y + 4.0f) delta = box.y - view.y - 8.0f;
        else if (box.y + box.h > view.y + view.h - 4.0f)
            delta = box.y + box.h - (view.y + view.h) + 8.0f;
        if (delta == 0.0f) return;
        float target = scroll->scrollY() + delta;
        if (target < 0.0f) target = 0.0f;
        scroll->setScrollY(target);
        frames(editor, 2);
    }
}

void clickWidget(Editor& editor, const rt::Widget* widget)
{
    reveal(editor, widget);
    click(editor, centerX(widget), centerY(widget));
}

void clickAt(Editor& editor, const rt::Rect& rect)
{
    click(editor, static_cast<int>(rect.x + rect.w * 0.5f),
            static_cast<int>(rect.y + rect.h * 0.5f));
}

bool near(float a, float b, float epsilon = kEpsilon)
{
    return fabsf(a - b) <= epsilon;
}

PropertyValue property(const Node* node, const char* name)
{
    PropertyValue value;
    NodeClassDB::getProperty(node, name, value);
    return value;
}

ct::String meshPath(const Node* node)
{
    return property(node, "mesh").asString();
}

bool contains(const ct::String& text, const char* fragment)
{
    return strstr(text.c_str(), fragment) != nullptr;
}

ct::String readFile(const ct::String& path)
{
    return FileSystem::getSingleton().readText(path);
}

Node* childNamed(Node* parent, const char* name)
{
    return parent ? parent->findChild(name) : nullptr;
}

Node* makeNode(Editor& editor, const char* className, const char* name)
{
    Node* scene = editor.document.scene();
    Node* node = scene ? scene->createChild(className, true) : nullptr;
    CHECK(node != nullptr);
    if (node) node->setName(name);
    return node;
}

void select(Editor& editor, Node* node)
{
    editor.selection.clear();
    if (node) editor.selection.add(node);
    frames(editor, 3);
    CHECK(editor.inspector.inspected() == node);
}

void cleanScene(Editor& editor)
{
    CHECK(editor.document.saveScene());
    CHECK(!editor.document.dirty());
}

PropertyRow* rowOf(Editor& editor, const char* name)
{
    return editor.inspector.properties().rowNamed(name);
}

MeshId skinnedMesh()
{
    MeshStore* meshes = Main::meshes();
    const MeshId mesh = meshes->create("skinned");
    SurfaceData surface;
    surface.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    surface.positions.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
    surface.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    for (int i = 0; i < 3; ++i)
    {
        surface.joints.push_back(0u);
        surface.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
        surface.indices.push_back(static_cast<std::uint32_t>(i));
    }
    CHECK(meshes->addSurface(mesh, surface, "skinned"));
    return mesh;
}

void writeImage(const ct::String& path)
{
    Pixmap pixmap(4, 4, 4);
    pixmap.fill(128, 128, 128, 255);
    CHECK(pixmap.save(path.c_str()));
}

const char* const kTwoMeshes =
        "{\"asset\":{\"version\":\"2.0\"},"
        "\"buffers\":[{\"uri\":\"two.bin\",\"byteLength\":36}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36,\"target\":34962}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
        "\"min\":[0,0,0],\"max\":[1,1,0]}],"
        "\"meshes\":[{\"name\":\"Alpha\",\"primitives\":[{\"attributes\":{\"POSITION\":0}}]},"
        "{\"name\":\"Beta\",\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
        "\"nodes\":[{\"mesh\":0,\"name\":\"A\"},{\"mesh\":1,\"name\":\"B\"}],"
        "\"scenes\":[{\"nodes\":[0,1]}],\"scene\":0}";

const char* const kSolo = "v 0 0 0\n"
                          "v 1 0 0\n"
                          "v 0 1 0\n"
                          "f 1 2 3\n";

const char* const kMetal = "material \"steel\" { roughness = 0.3 metallic = 1 }\n"
                           "material \"gold\" { roughness = 0.2 metallic = 1 }\n";

const char* const kWood = "material \"oak\" { roughness = 0.9 }\n";

void writeAssets(const ct::String& root)
{
    CHECK(fs_create_directory((root + "/Assets/Models").c_str()));
    CHECK(fs_create_directory((root + "/Assets/Materials").c_str()));
    CHECK(fs_create_directory((root + "/Assets/Props").c_str()));
    CHECK(fs_create_directory((root + "/Assets/Textures").c_str()));
    writeImage(root + "/Assets/Textures/height.png");
    const float positions[9] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    CHECK(file_write((root + "/Assets/Models/two.bin").c_str(), positions, sizeof(positions)));
    CHECK(FileSystem::writeText(root + "/Assets/Models/two.gltf", kTwoMeshes));
    CHECK(FileSystem::writeText(root + "/Assets/Models/solo.obj", kSolo));
    CHECK(FileSystem::writeText(root + "/Assets/Materials/metal.mat", kMetal));
    CHECK(FileSystem::writeText(root + "/Assets/Props/wood.mat", kWood));
}

void testVisibility(Editor& editor)
{
    PropertyInspector& inspector = editor.inspector.properties();
    Node* camera = makeNode(editor, "Camera3D", "Cam");
    select(editor, camera);
    cleanScene(editor);
    CHECK(rowOf(editor, "fov") != nullptr && rowOf(editor, "size") == nullptr);
    CHECK(rowOf(editor, "projection") && rowOf(editor, "near") && rowOf(editor, "far") &&
            rowOf(editor, "cull_mask") && rowOf(editor, "current"));

    NodeClassDB::setProperty(camera, "fov", PropertyValue::fromFloat(70.0));
    frames(editor, 2);
    const unsigned int rebuilds = inspector.rebuildCount();
    PropertyRow* projection = rowOf(editor, "projection");
    CHECK(projection && projection->combo());
    if (!projection || !projection->combo()) return;
    reveal(editor, projection);
    projection->combo()->setSelectedIndex(1);
    frames(editor, 3);
    CHECK(camera->castTo<Camera3D>()->getProjection() == CameraProjection::Orthogonal);
    CHECK(rowOf(editor, "fov") == nullptr && rowOf(editor, "size") != nullptr);
    CHECK(rowOf(editor, "near") && rowOf(editor, "far"));
    CHECK(inspector.rebuildCount() == rebuilds + 1);
    CHECK(editor.document.dirty());

    cleanScene(editor);
    const ct::String path = editor.document.sceneAbsolutePath(editor.document.scenePath());
    CHECK(contains(readFile(path), "\"fov\""));
    NodeClassDB::setProperty(camera, "fov", PropertyValue::fromFloat(55.0));
    CHECK(editor.document.saveScene());
    CHECK(contains(readFile(path), "\"fov\""));
    {
        SceneResult loaded;
        Node* holder = editor.tree.getRoot()->createChild<Node3D>();
        holder->setName("holder");
        Node* reread = SceneSerializer::loadFile(path, holder, loaded);
        CHECK(reread != nullptr && loaded.success());
        Node* copy = childNamed(reread, "Cam");
        CHECK(copy != nullptr);
        if (copy)
        {
            CHECK(near(static_cast<float>(property(copy, "fov").asFloat()), 55.0f));
            CHECK(copy->castTo<Camera3D>()->getProjection() == CameraProjection::Orthogonal);
        }
        holder->queueFree();
        frames(editor, 2);
    }

    const unsigned int before = inspector.rebuildCount();
    camera->castTo<Camera3D>()->setPerspective(60.0f, 0.1f, 100.0f);
    frames(editor, 3);
    CHECK(rowOf(editor, "fov") != nullptr && rowOf(editor, "size") == nullptr);
    CHECK(inspector.rebuildCount() == before + 1);
    camera->castTo<Camera3D>()->setFov(80.0f);
    frames(editor, 3);
    CHECK(inspector.rebuildCount() == before + 1);
    CHECK(near(rowOf(editor, "fov")->field(0)->value(), 80.0f));

    Node* second = makeNode(editor, "Camera3D", "Cam2");
    second->castTo<Camera3D>()->setOrthogonal(5.0f, 0.1f, 100.0f);
    select(editor, second);
    CHECK(rowOf(editor, "size") != nullptr && rowOf(editor, "fov") == nullptr);
    select(editor, camera);
    CHECK(rowOf(editor, "fov") != nullptr && rowOf(editor, "size") == nullptr);
    second->queueFree();
    frames(editor, 2);
    cleanScene(editor);

    Node* sun = makeNode(editor, "DirectionalLight3D", "Sun");
    select(editor, sun);
    CHECK(rowOf(editor, "directional_shadow_mode") == nullptr &&
            rowOf(editor, "directional_shadow_split_1") == nullptr &&
            rowOf(editor, "shadow_bias") == nullptr && rowOf(editor, "shadow_enabled") == nullptr &&
            editor.inspector.properties().gate("shadow_enabled") != nullptr);
    sun->castTo<DirectionalLight3D>()->setShadow(true);
    frames(editor, 3);
    CHECK(rowOf(editor, "shadow_bias") != nullptr);
    CHECK(rowOf(editor, "directional_shadow_split_1") &&
            rowOf(editor, "directional_shadow_split_3"));
    PropertyRow* mode = rowOf(editor, "directional_shadow_mode");
    CHECK(mode && mode->combo());
    if (mode && mode->combo())
    {
        reveal(editor, mode);
        mode->combo()->setSelectedIndex(0);
        frames(editor, 3);
        CHECK(rowOf(editor, "directional_shadow_split_1") == nullptr &&
                rowOf(editor, "directional_shadow_blend_splits") == nullptr &&
                rowOf(editor, "directional_shadow_split_2") == nullptr &&
                rowOf(editor, "directional_shadow_split_3") == nullptr);
        CHECK(rowOf(editor, "directional_shadow_fade_start") != nullptr);
        rowOf(editor, "directional_shadow_mode")->combo()->setSelectedIndex(1);
        frames(editor, 3);
        CHECK(rowOf(editor, "directional_shadow_split_1") != nullptr &&
                rowOf(editor, "directional_shadow_blend_splits") != nullptr &&
                rowOf(editor, "directional_shadow_split_2") == nullptr);
    }
    sun->queueFree();
    frames(editor, 2);
    cleanScene(editor);
}

void testSkeletonVisibility(Editor& editor)
{
    PropertyInspector& inspector = editor.inspector.properties();
    Node* body = makeNode(editor, "MeshInstance3D", "Body");
    MeshInstance3D* instance = body->castTo<MeshInstance3D>();
    select(editor, body);
    CHECK(rowOf(editor, "skeleton") == nullptr);
    CHECK(rowOf(editor, "mesh") && rowOf(editor, "material_override") && rowOf(editor, "color"));

    NodeClassDB::setProperty(body, "skeleton", PropertyValue::fromNodePath("../Skel"));
    const MeshId plain = createBoxMesh(*Main::meshes());
    instance->setMesh(plain);
    frames(editor, 3);
    CHECK(rowOf(editor, "skeleton") == nullptr);

    const unsigned int rebuilds = inspector.rebuildCount();
    const MeshId skinned = skinnedMesh();
    instance->setMesh(skinned);
    frames(editor, 3);
    PropertyRow* skeleton = rowOf(editor, "skeleton");
    CHECK(skeleton != nullptr && skeleton->kind() == PropertyRow::Kind::NodePath);
    CHECK(skeleton && skeleton->text() && skeleton->text()->text() == "../Skel");
    CHECK(inspector.rebuildCount() == rebuilds + 1);

    instance->setMesh(plain);
    frames(editor, 3);
    CHECK(rowOf(editor, "skeleton") == nullptr);
    CHECK(inspector.rebuildCount() == rebuilds + 2);

    CHECK(editor.document.saveScene());
    const ct::String path = editor.document.sceneAbsolutePath(editor.document.scenePath());
    CHECK(contains(readFile(path), "\"skeleton\""));
    CHECK(contains(readFile(path), "../Skel"));

    instance->setMesh(skinned);
    select(editor, nullptr);
    select(editor, body);
    CHECK(rowOf(editor, "skeleton") != nullptr);
    instance->setMesh(MeshId());
    select(editor, nullptr);
    select(editor, body);
    CHECK(rowOf(editor, "skeleton") == nullptr);
    body->queueFree();
    frames(editor, 2);
    cleanScene(editor);
}

void testTransformReset(Editor& editor)
{
    InspectorPanel& panel = editor.inspector;
    Node3D* node = makeNode(editor, "Node3D", "Moved")->castTo<Node3D>();
    node->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    node->setRotationDegrees(Math::Vec3(10.0f, 20.0f, 30.0f));
    node->setScale(Math::Vec3(2.0f, 3.0f, 4.0f));
    select(editor, node);
    cleanScene(editor);

    CHECK(strcmp(InspectorPanel::resetTooltip(InspectorPanel::kPosition),
                  "Reset position to 0, 0, 0") == 0);
    CHECK(strcmp(InspectorPanel::resetTooltip(InspectorPanel::kRotation),
                  "Reset rotation to 0, 0, 0") == 0);
    CHECK(strcmp(InspectorPanel::resetTooltip(InspectorPanel::kScale), "Reset scale to 1, 1, 1") ==
            0);
    for (int row = 0; row < InspectorPanel::kRowCount; ++row)
    {
        RevertButton* reset = panel.resetButton(row);
        CHECK(reset != nullptr && reset->active());
        if (!reset) return;
        CHECK(reset->tooltip() == InspectorPanel::resetTooltip(row));
        CHECK(reset->absoluteRect().w > 8.0f && reset->absoluteRect().h > 8.0f);
        CHECK(reset->absoluteRect().x > panel.field(row, 2)->absoluteRect().x);
        CHECK(reset->absoluteRect().y < panel.field(row, 0)->absoluteRect().y);
    }
    CHECK(panel.resetButton(-1) == nullptr && panel.resetButton(3) == nullptr);

    unsigned int writes = panel.writeCount();
    clickWidget(editor, panel.resetButton(InspectorPanel::kPosition));
    CHECK(near(node->getPosition().x, 0.0f) && near(node->getPosition().y, 0.0f) &&
            near(node->getPosition().z, 0.0f));
    CHECK(near(node->getRotationDegrees().y, 20.0f) && near(node->getScale().z, 4.0f));
    CHECK(near(panel.field(0, 1)->value(), 0.0f) && near(panel.field(0, 2)->value(), 0.0f));
    CHECK(panel.writeCount() == writes + 1);
    CHECK(editor.document.dirty());
    cleanScene(editor);

    clickWidget(editor, panel.resetButton(InspectorPanel::kRotation));
    CHECK(near(node->getRotationDegrees().x, 0.0f) && near(node->getRotationDegrees().y, 0.0f) &&
            near(node->getRotationDegrees().z, 0.0f));
    CHECK(near(node->getScale().x, 2.0f));
    CHECK(near(panel.field(1, 2)->value(), 0.0f));
    CHECK(editor.document.dirty());
    cleanScene(editor);

    clickWidget(editor, panel.resetButton(InspectorPanel::kScale));
    CHECK(near(node->getScale().x, 1.0f) && near(node->getScale().y, 1.0f) &&
            near(node->getScale().z, 1.0f));
    CHECK(near(panel.field(2, 0)->value(), 1.0f) && near(panel.field(2, 1)->value(), 1.0f));
    CHECK(editor.document.dirty());
    cleanScene(editor);

    writes = panel.writeCount();
    node->setPosition(Math::Vec3(4.0f, 5.0f, 6.0f));
    frames(editor, 2);
    clickWidget(editor, panel.resetButton(InspectorPanel::kPosition));
    CHECK(near(node->getPosition().y, 0.0f) && panel.writeCount() == writes + 1);

    Node* plain = makeNode(editor, "Node", "Plain");
    select(editor, plain);
    CHECK(!panel.showsTransform());
    select(editor, node);
    node->queueFree();
    plain->queueFree();
    frames(editor, 2);
    cleanScene(editor);
}

struct KindExpectation
{
    PrimitiveKind kind;
    const char* path;
};

void checkMeshBuilt(Node* body, const ct::String& expected)
{
    MeshInstance3D* instance = body->castTo<MeshInstance3D>();
    CHECK(instance != nullptr);
    if (!instance) return;
    CHECK(sceneres::alive(instance->getMesh()));
    CHECK(sceneres::pathOf(instance->getMesh()) == expected);
    const MeshInfo* info = Main::meshes()->get(instance->getMesh());
    CHECK(info != nullptr && !info->surfaces.empty());
}

template<typename T>
int countWidgets(const rt::Widget* root)
{
    int count = rt::widget_cast<T>(root) ? 1 : 0;
    for (std::size_t i = 0; i < root->children().size(); ++i)
        count += countWidgets<T>(root->children()[i]);
    return count;
}

int findButtons(const rt::Widget* root)
{
    return countWidgets<rt::Button>(root);
}

int findCombos(const rt::Widget* root)
{
    return countWidgets<rt::ComboBox>(root);
}

int findNumberFields(const rt::Widget* root)
{
    return countWidgets<NumberField>(root);
}

MeshPropertyEditor* meshEditorOf(Editor& editor)
{
    PropertyRow* row = rowOf(editor, "mesh");
    CHECK(row != nullptr && row->kind() == PropertyRow::Kind::Mesh);
    return row ? row->meshEditor() : nullptr;
}

void testMeshSummaries(Editor&)
{
    const struct
    {
        const char* path;
        const char* text;
    } cases[] = {
        { "", "No mesh" },
        { "primitive:box?size=1,1,1", "Box 1 \xC3\x97 1 \xC3\x97 1" },
        { "primitive:box?size=2,0.5,3", "Box 2 \xC3\x97 0.5 \xC3\x97 3" },
        { "primitive:sphere?radius=0.5&rings=16&slices=24", "Sphere r 0.5, 16 rings, 24 slices" },
        { "primitive:cylinder?radius=0.5&height=1&slices=24", "Cylinder r 0.5, h 1, 24 slices" },
        { "primitive:cone?radius=2&height=3&slices=12", "Cone r 2, h 3, 12 slices" },
        { "primitive:capsule?radius=0.5&height=1&rings=8&slices=24",
            "Capsule r 0.5, h 1, 8 rings, 24 slices" },
        { "primitive:torus?major_radius=1&minor_radius=0.25&major_segments=24&minor_segments=12",
            "Torus R 1, r 0.25, 24 \xC3\x97 12 segments" },
        { "primitive:plane?width=2&depth=3&segments_x=4&segments_z=5&uv_tiles=1",
            "Plane 2 \xC3\x97 3, 4 \xC3\x97 5 segments" },
        { "Assets/Models/two.gltf::Beta", "Beta (two.gltf)" },
        { "Assets/Models/solo.obj", "Mesh (solo.obj)" },
    };
    for (const auto& item: cases) CHECK(MeshPropertyEditor::summaryOf(item.path) == item.text);
}

void testMeshModel(Editor& editor, Node* body)
{
    select(editor, body);
    MeshPropertyEditor* mesh = meshEditorOf(editor);
    if (!mesh) return;

    NodeClassDB::setProperty(body, "mesh",
            PropertyValue::fromResourcePath("Assets/Models/two.gltf::Beta"));
    select(editor, nullptr);
    select(editor, body);
    mesh = meshEditorOf(editor);
    CHECK(mesh->mode() == MeshPropertyEditor::Mode::Model);
    CHECK(mesh->summaryLabel()->text() == "Beta (two.gltf)");
    CHECK(mesh->summaryLabel()->isVisible());
    checkMeshBuilt(body, "Assets/Models/two.gltf::Beta");

    NodeClassDB::setProperty(body, "mesh",
            PropertyValue::fromResourcePath("Assets/Models/two.gltf::Alpha"));
    frames(editor, 3);
    CHECK(mesh->summaryLabel()->text() == "Alpha (two.gltf)");
    NodeClassDB::setProperty(body, "mesh",
            PropertyValue::fromResourcePath("Assets/Models/two.gltf::Beta"));
    cleanScene(editor);

    const ct::String scenePath = editor.document.sceneAbsolutePath(editor.document.scenePath());
    CHECK(contains(readFile(scenePath), "Assets/Models/two.gltf::Beta"));
    {
        SceneResult loaded;
        Node* holder = editor.tree.getRoot()->createChild<Node3D>();
        holder->setName("holder");
        Node* reread = SceneSerializer::loadFile(scenePath, holder, loaded);
        CHECK(reread != nullptr && loaded.success());
        Node* copy = childNamed(reread, "Body");
        CHECK(copy != nullptr && meshPath(copy) == "Assets/Models/two.gltf::Beta");
        holder->queueFree();
        frames(editor, 2);
    }

    CHECK(mesh->statsLabel() != nullptr);
    clickWidget(editor, rowOf(editor, "mesh")->revertButton());
    CHECK(meshPath(body).empty() && mesh->mode() == MeshPropertyEditor::Mode::None);
    cleanScene(editor);
}

void testMeshReadOnly(Editor& editor, Node* body)
{
    select(editor, body);
    MeshPropertyEditor* mesh = meshEditorOf(editor);
    if (!mesh) return;
    CHECK(mesh->mode() == MeshPropertyEditor::Mode::None);
    CHECK(mesh->summaryLabel()->text() == MeshPropertyEditor::kNoMeshText);
    CHECK(!mesh->statsLabel()->isVisible());
    rt::Collapsible* layers = editor.inspector.properties().section("Layers");
    CHECK(layers != nullptr && !layers->isExpanded());
    rt::Collapsible* geometry = editor.inspector.properties().section("Geometry");
    CHECK(geometry != nullptr && geometry->isExpanded());

    NodeClassDB::setProperty(body, "mesh",
            PropertyValue::fromResourcePath("primitive:box?size=1,1,1"));
    select(editor, nullptr);
    select(editor, body);
    mesh = meshEditorOf(editor);
    CHECK(mesh->mode() == MeshPropertyEditor::Mode::Primitive);
    CHECK(mesh->summaryLabel()->text() == "Box 1 \xC3\x97 1 \xC3\x97 1");
    CHECK(mesh->statsLabel()->isVisible());
    CHECK(mesh->statsLabel()->text() == "1 surface, 12 triangles");

    CHECK(rt::widget_cast<rt::Button>(mesh) == nullptr);
    // Only the hidden Save as .pmesh: a primitive is not saved and has no surface buttons.
    CHECK(findButtons(mesh) == 1 && !mesh->saveButton()->isVisible());
    CHECK(findCombos(mesh) == 0);
    CHECK(findNumberFields(mesh) == 0);

    NodeClassDB::setProperty(body, "mesh", PropertyValue::fromResourcePath(""));
    frames(editor, 3);
    mesh = meshEditorOf(editor);
    CHECK(mesh->mode() == MeshPropertyEditor::Mode::None && !mesh->statsLabel()->isVisible());

    PropertyInfo collision;
    CHECK(NodeClassDB::getPropertyInfo("Node3D", "collision_mesh", collision));
    CHECK(PropertyRow::kindOf(collision) == PropertyRow::Kind::Mesh);
    CHECK(PropertyRow::kindOf(PropertyInfo::resource("t", "Texture2D")) ==
            PropertyRow::Kind::ResourcePath);
    CHECK(PropertyRow::kindOf(PropertyInfo::resource("m", "Material")) ==
            PropertyRow::Kind::Material);
    CHECK(PropertyRow::kindOf(PropertyInfo::resource("m", "Mesh,Material")) ==
            PropertyRow::Kind::Mesh);
    CHECK(PropertyRow::kindOf(PropertyInfo::file("f", "*.png")) == PropertyRow::Kind::ResourcePath);
}

void testMaterialPicker(Editor& editor)
{
    PropertyInspector& inspector = editor.inspector.properties();
    Node* body = makeNode(editor, "MeshInstance3D", "Painted");
    select(editor, body);
    cleanScene(editor);

    PropertyRow* row = rowOf(editor, "material_override");
    CHECK(row != nullptr && row->kind() == PropertyRow::Kind::Material);
    if (!row) return;
    CHECK(row->text() == nullptr && row->materialLabel() != nullptr && row->materialButton());
    CHECK(row->materialLabel()->text() == "<empty>");
    CHECK(inspector.pickerMenu() == nullptr);

    clickWidget(editor, row->materialButton());
    rt::Menu* menu = inspector.pickerMenu();
    CHECK(menu != nullptr);
    if (!menu) return;
    CHECK(menu->actionCount() == 6);
    CHECK(menu->action(0)->text() == "Assets/Materials/metal.mat::steel");
    CHECK(menu->action(1)->text() == "Assets/Materials/metal.mat::gold");
    CHECK(menu->action(2)->isSeparator());
    CHECK(menu->action(3)->text() == "Assets/Props/wood.mat::oak");
    CHECK(menu->action(4)->isSeparator());
    CHECK(menu->action(5)->text() == "Clear");
    const int steel = 0;
    CHECK(rt::WidgetApp::instance().popup() == menu);

    menu->action(steel)->triggered.emit();
    frames(editor, 3);
    rt::WidgetApp::instance().closePopup();
    frames(editor, 2);
    const MeshInstance3D* instance = body->castTo<MeshInstance3D>();
    CHECK(property(body, "material_override").asString() == "Assets/Materials/metal.mat::steel");
    CHECK(Main::materials()->valid(instance->getMaterialOverride()));
    row = rowOf(editor, "material_override");
    CHECK(row->materialLabel()->text() == "steel");
    CHECK(row->materialLabel()->tooltip() == "Assets/Materials/metal.mat::steel");
    CHECK(row->revertible());
    CHECK(editor.document.dirty());
    cleanScene(editor);

    clickWidget(editor, row->materialButton());
    menu = inspector.pickerMenu();
    CHECK(menu != nullptr && menu->actionCount() == 6);
    for (int i = 0; menu && i < menu->actionCount(); ++i)
        if (menu->action(i)->text() == "Assets/Props/wood.mat::oak")
            menu->action(i)->triggered.emit();
    rt::WidgetApp::instance().closePopup();
    frames(editor, 3);
    CHECK(property(body, "material_override").asString() == "Assets/Props/wood.mat::oak");
    CHECK(rowOf(editor, "material_override")->materialLabel()->text() == "oak");

    clickWidget(editor, rowOf(editor, "material_override")->materialButton());
    menu = inspector.pickerMenu();
    for (int i = 0; menu && i < menu->actionCount(); ++i)
        if (menu->action(i)->text() == PropertyInspector::kClearMaterial)
            menu->action(i)->triggered.emit();
    rt::WidgetApp::instance().closePopup();
    frames(editor, 3);
    CHECK(property(body, "material_override").asString().empty());
    CHECK(!Main::materials()->valid(instance->getMaterialOverride()));
    CHECK(rowOf(editor, "material_override")->materialLabel()->text() == "<empty>");
    CHECK(!rowOf(editor, "material_override")->revertible());

    NodeClassDB::setProperty(body, "material_override",
            PropertyValue::fromResourcePath("Assets/Materials/metal.mat::gold"));
    frames(editor, 3);
    CHECK(rowOf(editor, "material_override")->materialLabel()->text() == "gold");
    clickWidget(editor, rowOf(editor, "material_override")->revertButton());
    CHECK(property(body, "material_override").asString().empty());
    CHECK(rowOf(editor, "material_override")->materialLabel()->text() == "<empty>");

    CHECK(PropertyRow::materialDisplayName("") == "<empty>");
    CHECK(PropertyRow::materialDisplayName("a/b.mat::name") == "name");
    CHECK(PropertyRow::materialDisplayName("plain") == "plain");

    Node* tree = makeNode(editor, "Tree3D", "Oak");
    select(editor, tree);
    CHECK(rowOf(editor, "bark_material") &&
            rowOf(editor, "bark_material")->kind() == PropertyRow::Kind::Material);
    CHECK(rowOf(editor, "leaf_material") &&
            rowOf(editor, "leaf_material")->kind() == PropertyRow::Kind::Material);
    tree->queueFree();
    body->queueFree();
    frames(editor, 3);
    cleanScene(editor);
}

void testNodePicker(Editor& editor)
{
    PropertyInspector& inspector = editor.inspector.properties();
    Node* a = makeNode(editor, "StaticBody3D", "A");
    Node* b = makeNode(editor, "RigidBody3D", "B");
    Node* other = makeNode(editor, "Node3D", "Other");
    Node* joint = makeNode(editor, "PinJoint3D", "Joint");
    Node* nested = joint->createChild("RigidBody3D", true);
    nested->setName("Nested");
    select(editor, joint);
    cleanScene(editor);

    PropertyRow* row = rowOf(editor, "node_a");
    CHECK(row != nullptr && row->kind() == PropertyRow::Kind::NodePath);
    if (!row) return;
    CHECK(row->text() != nullptr && row->nodeButton() != nullptr);
    CHECK(rowOf(editor, "enabled")->nodeButton() == nullptr);
    clickWidget(editor, row->nodeButton());
    rt::Menu* menu = inspector.pickerMenu();
    CHECK(menu != nullptr && rt::WidgetApp::instance().popup() == menu);
    if (!menu) return;
    CHECK(menu->actionCount() == 5);
    CHECK(menu->action(0)->text() == "../A" && menu->action(1)->text() == "../B" &&
            menu->action(2)->text() == "Nested");
    CHECK(menu->action(3)->isSeparator() && menu->action(4)->text() == "Clear");
    menu->action(1)->triggered.emit();
    rt::WidgetApp::instance().closePopup();
    frames(editor, 3);
    CHECK(property(joint, "node_a").type() == PropertyType::NodePath);
    CHECK(property(joint, "node_a").asString() == "../B");
    CHECK(rowOf(editor, "node_a")->text()->text() == "../B");
    CHECK(editor.document.dirty());
    CHECK(rowOf(editor, "node_b")->text()->text().empty());

    clickWidget(editor, rowOf(editor, "node_a")->nodeButton());
    menu = inspector.pickerMenu();
    menu->action(2)->triggered.emit();
    rt::WidgetApp::instance().closePopup();
    frames(editor, 3);
    CHECK(property(joint, "node_a").asString() == "Nested");
    clickWidget(editor, rowOf(editor, "node_a")->nodeButton());
    menu = inspector.pickerMenu();
    menu->action(menu->actionCount() - 1)->triggered.emit();
    rt::WidgetApp::instance().closePopup();
    frames(editor, 3);
    CHECK(property(joint, "node_a").asString().empty());
    CHECK(rowOf(editor, "node_a")->text()->text().empty());

    Node* mesh = makeNode(editor, "MeshInstance3D", "Floor");
    select(editor, mesh);
    CHECK(inspector.rowNamed("reflector") == nullptr);
    Node* mirror = makeNode(editor, "PlanarReflector3D", "Mirror");
    frames(editor, 3);
    select(editor, mesh);
    CHECK(inspector.rowNamed("reflector") != nullptr);
    CHECK(inspector.rowNamed("reflector")->label() == "Planar Reflector");
    clickWidget(editor, rowOf(editor, "reflector")->nodeButton());
    menu = inspector.pickerMenu();
    CHECK(menu != nullptr && menu->actionCount() == 3 && menu->action(0)->text() == "../Mirror");
    menu->action(0)->triggered.emit();
    rt::WidgetApp::instance().closePopup();
    frames(editor, 3);
    CHECK(property(mesh, "reflector").asString() == "../Mirror");

    mirror->queueFree();
    frames(editor, 3);
    CHECK(inspector.rowNamed("reflector") != nullptr);
    NodeClassDB::setProperty(mesh, "reflector", PropertyValue::fromNodePath(""));
    frames(editor, 3);
    CHECK(inspector.rowNamed("reflector") == nullptr);
    mesh->queueFree();
    other->queueFree();
    joint->queueFree();
    a->queueFree();
    b->queueFree();
    frames(editor, 3);
    cleanScene(editor);
}

void testEnvironmentGates(Editor& editor)
{
    Node* environment = makeNode(editor, "WorldEnvironment", "Environment");
    select(editor, environment);
    cleanScene(editor);
    const char* sections[5] = { "SSR", "SSAO", "Adjustments", "Depth of Field Far",
        "Depth of Field Near" };
    const char* flags[5] = { "ssr_enabled", "ssao_enabled", "adjustment_enabled",
        "dof_blur_far_enabled", "dof_blur_near_enabled" };
    const char* rows[5] = { "ssr_max_steps", "ssao_radius", "adjustment_brightness",
        "dof_blur_far_distance", "dof_blur_near_distance" };
    for (int i = 0; i < 5; ++i)
    {
        rt::Collapsible* section = editor.inspector.properties().section(sections[i]);
        CHECK(section != nullptr);
        if (!section) continue;
        CHECK(section->isCheckable());
        CHECK(!section->isChecked());
        CHECK(!property(environment, flags[i]).asBool());
        CHECK(rowOf(editor, flags[i]) == nullptr);
        CHECK(rowOf(editor, rows[i]) != nullptr);

        NodeClassDB::setProperty(environment, flags[i], PropertyValue::fromBool(true));
        frames(editor, 3);
        CHECK(section->isChecked());

        section->setExpanded(true);
        frames(editor, 3);
        reveal(editor, section);
        clickAt(editor, section->checkRect());
        CHECK(!property(environment, flags[i]).asBool());
        CHECK(!section->isChecked());
        clickAt(editor, section->checkRect());
        CHECK(property(environment, flags[i]).asBool());
        CHECK(section->isChecked());
    }
    cleanScene(editor);
}

void testCameraAttributesGates(Editor& editor)
{
    Node* camera = makeNode(editor, "Camera3D", "Eye");
    select(editor, camera);
    cleanScene(editor);
    PropertyInspector& properties = editor.inspector.properties();
    rt::Collapsible* override = properties.gate("attributes_override");
    CHECK(override != nullptr && override == properties.section("Attributes") &&
            override->isCheckable() && !override->isChecked());
    CHECK(properties.gate("attributes_dof_blur_far_enabled") == nullptr);
    CHECK(properties.gate("attributes_dof_blur_near_enabled") == nullptr);
    CHECK(properties.rowNamed("attributes_dof_blur_amount") == nullptr);

    NodeClassDB::setProperty(camera, "attributes_override", PropertyValue::fromBool(true));
    frames(editor, 3);
    override = properties.gate("attributes_override");
    CHECK(override != nullptr && override->isChecked());
    rt::Collapsible* far = properties.gate("attributes_dof_blur_far_enabled");
    rt::Collapsible* near = properties.gate("attributes_dof_blur_near_enabled");
    CHECK(far != nullptr && far == properties.section("Attributes DOF Far") && far->isCheckable() &&
            !far->isChecked());
    CHECK(near != nullptr && near == properties.section("Attributes DOF Near") &&
            near->isCheckable() && !near->isChecked());
    CHECK(properties.rowNamed("attributes_dof_blur_amount") != nullptr);
    CHECK(properties.rowNamed("attributes_dof_blur_far_distance") != nullptr);
    CHECK(properties.rowNamed("attributes_dof_blur_near_transition") != nullptr);

    NodeClassDB::setProperty(camera, "attributes_dof_blur_far_enabled",
            PropertyValue::fromBool(true));
    frames(editor, 3);
    far = properties.gate("attributes_dof_blur_far_enabled");
    CHECK(far != nullptr && far->isChecked());
    cleanScene(editor);
}

void testLayerGrid(Editor& editor)
{
    Node* camera = makeNode(editor, "Camera3D", "Eye");
    select(editor, camera);
    cleanScene(editor);
    PropertyRow* mask = rowOf(editor, "cull_mask");
    CHECK(mask && mask->kind() == PropertyRow::Kind::Layers && mask->layerGrid());
    if (!mask || !mask->layerGrid()) return;
    rt::Collapsible* section = editor.inspector.properties().section("Cull Mask");
    CHECK(section != nullptr && !section->isExpanded());
    if (section) section->setExpanded(true);
    frames(editor, 3);
    LayerGrid* grid = mask->layerGrid();
    reveal(editor, mask);
    CHECK(grid->count() == 20 && grid->groupSize() == 5);
    grid->setExpanded(true);
    frames(editor, 3);
    reveal(editor, mask);
    CHECK(grid->visibleCount() == 20);

    CHECK(LayerGrid::cellLabel(0) == "1" && LayerGrid::cellLabel(9) == "10" &&
            LayerGrid::cellLabel(19) == "20");

    const rt::Rect first = grid->cellRect(0);
    CHECK(first.w > 8.0f && near(first.w, first.h));
    const float step = first.w + 1.0f;
    const rt::Rect second = grid->cellRect(1);
    CHECK(near(second.x, first.x + step) && near(second.y, first.y));
    const rt::Rect fifth = grid->cellRect(4);
    CHECK(near(fifth.x, first.x + 4.0f * step) && near(fifth.y, first.y));
    const rt::Rect sixth = grid->cellRect(5);
    CHECK(near(sixth.x, first.x) && near(sixth.y, first.y + step));
    const rt::Rect tenth = grid->cellRect(9);
    CHECK(near(tenth.x, first.x + 4.0f * step) && near(tenth.y, first.y + step));
    const rt::Rect block = grid->cellRect(10);
    const bool sameLine = near(block.y, first.y);
    if (sameLine) CHECK(near(block.x, first.x + 5.0f * step + 3.0f));
    else
        CHECK(near(block.x, first.x) && near(block.y, first.y + 2.0f * step + 3.0f));
    CHECK(near(grid->cellRect(14).x - block.x, 4.0f * step));
    CHECK(near(grid->cellRect(15).y - block.y, step));
    CHECK(grid->cellRect(-1).w == 0.0f && grid->cellRect(20).w == 0.0f);

    const rt::Rect box = grid->absoluteRect();
    for (int i = 0; i < 20; ++i)
    {
        const rt::Rect cell = grid->cellRect(i);
        CHECK(cell.x >= box.x && cell.x + cell.w <= box.x + box.w + 0.5f);
        CHECK(cell.y >= box.y && cell.y + cell.h <= box.y + box.h + 0.5f);
        CHECK(grid->cellAt(cell.x + cell.w * 0.5f, cell.y + cell.h * 0.5f) == i);
        CHECK(grid->cellAt(cell.x + 0.5f, cell.y + 0.5f) == i);
        CHECK(grid->cellAt(cell.x + cell.w - 0.5f, cell.y + cell.h - 0.5f) == i);
        CHECK(grid->cellAt(cell.x + cell.w + 0.5f, cell.y + cell.h * 0.5f) == -1);
        CHECK(grid->cellAt(cell.x + cell.w * 0.5f, cell.y + cell.h + 0.5f) == -1);
    }
    CHECK(grid->cellAt(box.x - 3.0f, box.y + 3.0f) == -1);
    CHECK(grid->cellAt(box.x + 1.0f, box.y + 1.0f) == -1);
    CHECK(grid->cellAt(-100.0f, -100.0f) == -1);

    const rt::Rect centreRect = grid->cellRect(12);
    const float label = 14.0f;
    CHECK(near(LayerGrid::labelX(centreRect, label) + label * 0.5f,
            centreRect.x + centreRect.w * 0.5f));
    CHECK(LayerGrid::labelBaseline(centreRect) > centreRect.y + centreRect.h * 0.5f &&
            LayerGrid::labelBaseline(centreRect) < centreRect.y + centreRect.h);
    CHECK(near(LayerGrid::textAlpha(false), 0.5f) && LayerGrid::textAlpha(true) > 0.6f);
    CHECK(near(LayerGrid::cellAlpha(false, false), 0.2f) &&
            near(LayerGrid::cellAlpha(true, false), 0.6f) &&
            near(LayerGrid::cellAlpha(false, true), 0.35f));

    const rt::Rect target = grid->cellRect(7);
    editor.host.handleEvent(
            motion(static_cast<int>(target.x + 3.0f), static_cast<int>(target.y + 3.0f)));
    frame(editor);
    CHECK(grid->hovered() == 7 && !grid->expandHovered());
    CHECK(grid->tooltip() == "Layer 8\nBit 7, value 128");
    CHECK(LayerGrid::layerTooltip(0) == "Layer 1\nBit 0, value 1");
    const rt::Rect other = grid->cellRect(12);
    editor.host.handleEvent(
            motion(static_cast<int>(other.x + 3.0f), static_cast<int>(other.y + 3.0f)));
    frame(editor);
    CHECK(grid->hovered() == 12 && grid->tooltip() == "Layer 13\nBit 12, value 4096");
    editor.host.handleEvent(motion(5, 5));
    frame(editor);
    CHECK(grid->hovered() == -1);

    const std::int64_t before = property(camera, "cull_mask").asInt();
    clickAt(editor, grid->cellRect(3));
    CHECK(property(camera, "cull_mask").asInt() == (before ^ 8));
    CHECK(grid->isChecked(3) == (((before ^ 8) & 8) != 0));
    CHECK(editor.document.dirty());
    clickAt(editor, grid->cellRect(3));
    CHECK(property(camera, "cull_mask").asInt() == before);

    const rt::Rect a = grid->cellRect(1);
    const rt::Rect b = grid->cellRect(2);
    const rt::Rect c = grid->cellRect(3);
    const bool aOn = grid->isChecked(1);
    editor.host.handleEvent(motion(static_cast<int>(a.x + 4.0f), static_cast<int>(a.y + 4.0f)));
    frame(editor);
    editor.host.handleEvent(
            button(static_cast<int>(a.x + 4.0f), static_cast<int>(a.y + 4.0f), true));
    frame(editor);
    editor.host.handleEvent(motion(static_cast<int>(b.x + 4.0f), static_cast<int>(b.y + 4.0f)));
    frame(editor);
    editor.host.handleEvent(motion(static_cast<int>(c.x + 4.0f), static_cast<int>(c.y + 4.0f)));
    frame(editor);
    editor.host.handleEvent(
            button(static_cast<int>(c.x + 4.0f), static_cast<int>(c.y + 4.0f), false));
    frames(editor, 2);
    CHECK(grid->isChecked(1) == !aOn && grid->isChecked(2) == !aOn && grid->isChecked(3) == !aOn);
    const std::int64_t painted = property(camera, "cull_mask").asInt();
    CHECK(((painted >> 1) & 7) == (aOn ? 0 : 7));
    editor.host.handleEvent(
            motion(static_cast<int>(c.x + c.w + 6.0f), static_cast<int>(c.y + 4.0f)));
    frame(editor);
    CHECK(property(camera, "cull_mask").asInt() == painted);

    rt::MouseEvent soloMove;
    soloMove.x = grid->cellRect(6).x + 4.0f;
    soloMove.y = grid->cellRect(6).y + 4.0f;
    grid->onMouseMove(soloMove);
    rt::MouseEvent solo = soloMove;
    solo.ctrl = true;
    solo.button = 0;
    grid->onMousePress(solo);
    grid->onMouseRelease(solo);
    frames(editor, 2);
    CHECK(property(camera, "cull_mask").asInt() == (1 << 6));
    CHECK(grid->bits() == (1 << 6));
    grid->onMousePress(solo);
    grid->onMouseRelease(solo);
    frames(editor, 2);
    CHECK(property(camera, "cull_mask").asInt() ==
            (~static_cast<std::int64_t>(1 << 6) & 0xffffffffll));
    clickWidget(editor, mask->revertButton());
    CHECK(property(camera, "cull_mask").asInt() == before);

    Node* floor = makeNode(editor, "StaticBody3D", "Floor");
    select(editor, floor);
    PropertyRow* layer = rowOf(editor, "collision_layer");
    CHECK(layer && layer->layerGrid());
    if (layer && layer->layerGrid())
    {
        LayerGrid* physics = layer->layerGrid();
        reveal(editor, layer);
        CHECK(physics->count() == 32 && physics->groupSize() == 4);
        CHECK(physics->expandable() == (physics->visibleCount() < 32));
        if (physics->expandable())
        {
            CHECK(!physics->expanded() && physics->visibleCount() < 32);
            CHECK(physics->cellRect(31).w == 0.0f && physics->cellAt(-1.0f, -1.0f) == -1);
            const rt::Rect arrow = physics->expandRect();
            const rt::Rect gridBox = physics->absoluteRect();
            CHECK(arrow.w > 4.0f && arrow.x >= gridBox.x &&
                    arrow.x + arrow.w <= gridBox.x + gridBox.w);
            editor.host.handleEvent(motion(static_cast<int>(arrow.x + arrow.w * 0.5f),
                    static_cast<int>(arrow.y + arrow.h * 0.5f)));
            frame(editor);
            CHECK(physics->expandHovered() && physics->hovered() == -1);
            const std::int64_t layers = property(floor, "collision_layer").asInt();
            clickAt(editor, arrow);
            CHECK(physics->expanded() && physics->visibleCount() == 32);
            CHECK(property(floor, "collision_layer").asInt() == layers);
            reveal(editor, layer);
            CHECK(physics->expandable() && physics->expansionRows() > 0);
            const rt::Rect last = physics->cellRect(31);
            CHECK(last.w > 8.0f);
            clickAt(editor, last);
            CHECK(property(floor, "collision_layer").asInt() ==
                    (layers ^ (static_cast<std::int64_t>(1) << 31)));
            const rt::Rect close = physics->expandRect();
            clickAt(editor, close);
            CHECK(!physics->expanded() && physics->visibleCount() < 32);
        }
        else
        {
            clickAt(editor, physics->cellRect(31));
            CHECK((property(floor, "collision_layer").asInt() >> 31) != 0);
        }
    }
    floor->queueFree();
    camera->queueFree();
    frames(editor, 3);
    cleanScene(editor);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;
    useVulkan = vulkan;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    removeTree(testRoot());
    CHECK(fs_create_directory(testRoot().c_str()));
    CHECK(fs_create_directory((testRoot() + "/projects").c_str()));

    DeviceDesc desc;
    desc.title = "editor inspector types test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    tree.setAutoAcceptQuit(false);
    Node* scene = tree.getRoot() ? tree.getRoot()->createChild<Node3D>() : nullptr;
    CHECK(scene != nullptr);

    if (scene)
    {
        scene->setName("Scene");
        {
            WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
            CHECK(host.init());
            EditorShell shell(host);
            CHECK(shell.build());
            {
                EditorSelection selection(tree);
                SceneTreePanel hierarchy(tree, selection);
                InspectorPanel inspector(tree, selection);
                CHECK(hierarchy.attach(shell.panel(0)));
                CHECK(inspector.attach(shell.panel(2)));
                hierarchy.setEditedScene(scene);

                EditorSettings settings;
                Project project;
                EditorDocument document(tree, selection, hierarchy, project, settings);
                document.setSettingsPath(testRoot() + "/settings.json");
                inspector.setChangeListener(&document);
                inspector.setProject(&project);
                hierarchy.setChangeListener(&document);

                Editor editor = { tree, host, selection, hierarchy, inspector, document };
                frames(editor, 3);
                CHECK(document.createProject(testRoot() + "/projects", "Types"));
                frames(editor, 3);
                CHECK(project.isOpen() && document.scene() != nullptr);
                writeAssets(project.root());

                testVisibility(editor);
                testSkeletonVisibility(editor);
                testTransformReset(editor);

                Node* body = makeNode(editor, "MeshInstance3D", "Body");
                testMeshSummaries(editor);
                testMeshModel(editor, body);
                testMeshReadOnly(editor, body);
                body->queueFree();
                frames(editor, 2);
                cleanScene(editor);

                testMaterialPicker(editor);
                testNodePicker(editor);
                testLayerGrid(editor);
                testEnvironmentGates(editor);
                testCameraAttributesGates(editor);
            }
            host.shutdown();
        }
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);
    removeTree(testRoot());

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
