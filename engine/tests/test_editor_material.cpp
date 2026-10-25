#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/AssetImport.h"
#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/MaterialPanel.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/LoadedMaterialLibrary.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/FileDialog.hpp>
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
const float kRowHeight = 22.0f;

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_material_test_vulkan" : "/editor_material_test";
    return root;
}

InputEvent motion(int x, int y)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = 0;
    event.mouseMotion.relativeY = 0;
    event.mouseMotion.buttonMask = 0;
    return event;
}

InputEvent button(int x, int y, MouseButton which, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = which;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = 0;
    return event;
}

InputEvent keyEvent(Key key, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = key;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = modifiers;
    return event;
}

struct Rig
{
    SceneTree& tree;
    WidgetHost& host;
    EditorShell& shell;
    EditorSelection& selection;
    SceneTreePanel& hierarchy;
    InspectorPanel& inspector;
    EditorDocument& document;
    ProjectUi& ui;
    EditorViewportPanel& panel;
    HierarchyActions& actions;
    EditorMenus& menus;
    EditorStatusBar& status;
    AssetsPanel& assets;
    MaterialPanel& materials;
    int sceneCounter = 0;

    bool send(const InputEvent& event)
    {
        if (ui.handleEvent(event)) return true;
        if (actions.handleEvent(event)) return true;
        host.handleEvent(event);
        panel.handleEvent(event);
        return false;
    }

    void frame(float delta = 1.0f / 60.0f)
    {
        tree.process(delta);
        panel.update(delta);
        Main::rendering()->renderViewports();
        selection.update();
        selection.flush();
        hierarchy.update();
        inspector.update();
        ui.update();
        actions.update();
        menus.update();
        status.update(delta);
        assets.update(delta);
        materials.update();
        host.frame(delta, true);
        Main::rendering()->endFrame(true);
    }

    void frames(int count)
    {
        for (int i = 0; i < count; ++i) frame();
    }

    bool framePixel(int x, int y, unsigned char* pixel)
    {
        const float delta = 1.0f / 60.0f;
        tree.process(delta);
        panel.update(delta);
        Main::rendering()->renderViewports();
        selection.update();
        selection.flush();
        hierarchy.update();
        inspector.update();
        ui.update();
        actions.update();
        menus.update();
        status.update(delta);
        assets.update(delta);
        materials.update();
        host.frame(delta, true);
        prisma::Rect rect;
        rect.x = x;
        rect.y = y;
        rect.width = 1;
        rect.height = 1;
        const bool read = Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixel);
        Main::rendering()->endFrame(true);
        return read;
    }

    void click(int x, int y, MouseButton which = MouseButton::Left)
    {
        send(motion(x, y));
        frames(2);
        send(button(x, y, which, true));
        frames(2);
        send(button(x, y, which, false));
        frames(3);
    }

    void doubleClick(int x, int y)
    {
        click(x, y);
        click(x, y);
    }

    void drag(int fromX, int fromY, int toX, int toY)
    {
        send(motion(fromX, fromY));
        frames(2);
        send(button(fromX, fromY, MouseButton::Left, true));
        frames(2);
        for (int step = 1; step <= 6; ++step)
        {
            const int x = fromX + (toX - fromX) * step / 6;
            const int y = fromY + (toY - fromY) * step / 6;
            send(motion(x, y));
            frames(1);
        }
        frames(2);
        send(button(toX, toY, MouseButton::Left, false));
        frames(3);
    }
};

MaterialStore& store()
{
    return *Main::materials();
}

bool near(float a, float b, float tolerance = 1.0e-3f)
{
    return fabsf(a - b) <= tolerance;
}

ct::String absolute(Rig& rig, const ct::String& relative)
{
    return rig.document.project().absolutePath(relative);
}

bool exists(const ct::String& path)
{
    return file_exists(path.c_str()) || dir_exists(path.c_str());
}

ct::String readText(const ct::String& path)
{
    char* text = file_read_text(path.c_str());
    if (!text) return ct::String();
    const ct::String result(text);
    fs_free(text);
    return result;
}

bool writePng(const ct::String& path, int width, int height, unsigned char red)
{
    const ct::String folder = AssetImport::parentOf(path);
    if (!folder.empty()) fs_create_directory(folder.c_str());
    Pixmap pixmap(width, height, 4);
    pixmap.fill(red, 40, 80, 255);
    return pixmap.save(path.c_str());
}

void centerOf(const rt::Rect& rect, int& x, int& y)
{
    x = static_cast<int>(rect.x + rect.w * 0.5f);
    y = static_cast<int>(rect.y + rect.h * 0.5f);
}

void scrollTo(Rig& rig, const rt::Widget* widget)
{
    rt::ScrollView* scroll = rig.materials.scrollView();
    CHECK(scroll != nullptr && widget != nullptr);
    if (!scroll || !widget) return;
    for (int pass = 0; pass < 3; ++pass)
    {
        const rt::Rect view = scroll->absoluteRect();
        const rt::Rect target = widget->absoluteRect();
        const float delta = target.y - (view.y + view.h * 0.4f);
        if (fabsf(delta) < 4.0f) break;
        scroll->setScrollY(scroll->scrollY() + delta);
        rig.frames(2);
    }
}

void dragGrabber(Rig& rig, NumberField* field, float ratio)
{
    scrollTo(rig, field);
    int x = 0;
    int y = 0;
    centerOf(field->absoluteRect(), x, y);
    rig.send(motion(x, y));
    rig.frames(2);
    CHECK(field->grabberVisible());
    const rt::Rect box = field->absoluteRect();
    int startX = 0;
    int startY = 0;
    centerOf(field->grabberRect(), startX, startY);
    const int targetX = static_cast<int>(box.x + 6.0f + (box.w - 16.0f) * ratio + 2.0f);
    rig.send(motion(startX, startY));
    rig.frames(1);
    rig.send(button(startX, startY, MouseButton::Left, true));
    rig.frames(1);
    for (int i = 1; i <= 6; ++i)
    {
        rig.send(motion(startX + (targetX - startX) * i / 6, startY));
        rig.frames(1);
    }
    CHECK(field->isGrabbing());
    rig.send(button(targetX, startY, MouseButton::Left, false));
    rig.frames(2);
}

bool insideMaterialScroll(Rig& rig, const rt::Widget* widget)
{
    const rt::Widget* scroll = rig.materials.scrollView();
    for (const rt::Widget* w = widget; w; w = w->parent())
        if (w == scroll) return true;
    return false;
}

void clickWidget(Rig& rig, const rt::Widget* widget)
{
    CHECK(widget != nullptr);
    if (!widget) return;
    if (insideMaterialScroll(rig, widget)) scrollTo(rig, widget);
    int x = 0;
    int y = 0;
    centerOf(widget->absoluteRect(), x, y);
    rig.click(x, y);
}

void showMaterial(Rig& rig)
{
    rig.shell.dock()->showPanel("Material");
    rig.shell.dock()->layout();
    rig.frames(4);
}

void showAssets(Rig& rig)
{
    rig.shell.dock()->showPanel("Assets");
    rig.shell.dock()->layout();
    rig.frames(4);
}

rt::InputBox* floatBox()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (rt::InputBox* box = rt::widget_cast<rt::InputBox>(floats[i - 1])) return box;
    return nullptr;
}

rt::FileDialog* floatDialog()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (rt::FileDialog* dialog = rt::widget_cast<rt::FileDialog>(floats[i - 1])) return dialog;
    return nullptr;
}

void closeFloats(Rig& rig)
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return;
    for (int guard = 0; guard < 8; ++guard)
    {
        const ct::Vector<rt::FloatWindow*>& floats = app->floats();
        if (floats.empty()) break;
        app->removeFloat(floats[floats.size() - 1]);
        rig.frames(1);
    }
}

int depthOf(const Node* node, const Node* scene)
{
    int depth = 0;
    for (const Node* n = node; n && n != scene; n = n->getParent()) ++depth;
    return depth;
}

bool findRow(Rig& rig, Node* node, Node* wanted, int& row)
{
    if (node == wanted) return true;
    ++row;
    rt::TreeNode* item = rig.hierarchy.itemOf(node);
    if (!item || !item->isExpanded()) return false;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        if (node->getChild(i)->isInternal()) continue;
        if (findRow(rig, node->getChild(i), wanted, row)) return true;
    }
    return false;
}

int rowY(Rig& rig, Node* node)
{
    int row = 0;
    if (!findRow(rig, rig.hierarchy.editedScene(), node, row)) return -1;
    const rt::Rect area = rig.hierarchy.outliner()->absoluteRect();
    return static_cast<int>(area.y + static_cast<float>(row) * kRowHeight + kRowHeight * 0.5f);
}

int textX(Rig& rig, Node* node)
{
    const rt::Rect area = rig.hierarchy.outliner()->absoluteRect();
    return static_cast<int>(area.x) + 4 + 16 * depthOf(node, rig.hierarchy.editedScene()) + 40;
}

MeshInstance3D* addMesh(Node* parent, const char* name, MeshId mesh)
{
    MeshInstance3D* node = parent->createChild<MeshInstance3D>();
    node->setName(name);
    node->setMesh(mesh);
    return node;
}

Node* freshScene(Rig& rig)
{
    CHECK(rig.document.newScene());
    rig.frames(3);
    Node* scene = rig.document.scene();
    CHECK(scene != nullptr);
    return scene;
}

void select(Rig& rig, Node* node)
{
    rig.selection.clear();
    if (node) rig.selection.add(node);
    rig.frames(4);
}

void commit(Rig& rig)
{
    rig.frames(3);
    ct::String name = "scene";
    name.append_number(++rig.sceneCounter);
    CHECK(rig.document.saveSceneAs(name));
    rig.frames(3);
    CHECK(!rig.document.dirty());
}

MaterialId makeBuiltIn(const char* name, const char* model)
{
    const MaterialId id = store().create(name);
    store().setPath(id, model);
    store().setFloat(id, MaterialParam::Roughness, 0.3f);
    store().setColor(id, MaterialParam::Albedo, Math::Vec4(0.2f, 0.6f, 0.4f, 1.0f));
    return id;
}

bool sameValue(const PropertyValue& a, const PropertyValue& b)
{
    if (a.type() != b.type()) return false;
    switch (a.type())
    {
        case PropertyType::Bool:
            return a.asBool() == b.asBool();
        case PropertyType::Int:
            return a.asInt() == b.asInt();
        case PropertyType::Float:
            return near(static_cast<float>(a.asFloat()), static_cast<float>(b.asFloat()));
        case PropertyType::Vec2:
            return near(a.asVec2().x, b.asVec2().x) && near(a.asVec2().y, b.asVec2().y);
        case PropertyType::Vec3:
            return near(a.asVec3().x, b.asVec3().x) && near(a.asVec3().y, b.asVec3().y) &&
                   near(a.asVec3().z, b.asVec3().z);
        case PropertyType::Color:
            return near(a.asColor().x, b.asColor().x) && near(a.asColor().y, b.asColor().y) &&
                   near(a.asColor().z, b.asColor().z) && near(a.asColor().w, b.asColor().w);
        case PropertyType::ResourcePath:
            return a.asString() == b.asString();
        default:
            return false;
    }
}

int enumCount(const char* names)
{
    int count = 1;
    for (const char* p = names; *p; ++p)
        if (*p == ',') ++count;
    return count;
}

PropertyValue testValue(const MaterialField& field, const PropertyValue& current)
{
    switch (field.kind)
    {
        case MaterialFieldKind::Bool:
            return PropertyValue::fromBool(!current.asBool());
        case MaterialFieldKind::Float:
            if (field.ranged)
                return PropertyValue::fromSingle(
                        static_cast<float>(field.minimum + (field.maximum - field.minimum) * 0.25));
            return PropertyValue::fromSingle(3.5f);
        case MaterialFieldKind::Enum:
        {
            const int count = enumCount(field.names);
            const int next = (static_cast<int>(current.asInt()) + 1) % count;
            return PropertyValue::fromInt(next);
        }
        case MaterialFieldKind::Vec2:
            return PropertyValue::fromVec2(Math::Vec2(0.25f, 0.75f));
        case MaterialFieldKind::Vec3:
            return PropertyValue::fromVec3(Math::Vec3(1.5f, 2.5f, 3.5f));
        case MaterialFieldKind::Color:
            return PropertyValue::fromColor(
                    Math::Vec4(0.2f, 0.4f, 0.6f, field.noAlpha ? 1.0f : 0.5f));
        case MaterialFieldKind::Texture:
            return PropertyValue::fromResourcePath("Assets/Textures/a.png");
    }
    return PropertyValue();
}

InputEvent character(char c)
{
    InputEvent event;
    event.type = InputEventType::Char;
    event.character.codepoint = static_cast<std::uint32_t>(c);
    return event;
}

void typeText(Rig& rig, const char* text)
{
    for (const char* p = text; *p; ++p) rig.send(character(*p));
    rig.frames(1);
}

void testLayout(Rig& rig)
{
    CHECK(strcmp(EditorShell::panelName(EditorShell::kMaterialPanel), "Material") == 0);
    CHECK(rig.shell.panel(EditorShell::kMaterialPanel) != nullptr);
    CHECK(strstr(rig.shell.layout().c_str(), "Material") != nullptr);
    CHECK(rig.materials.attached());
    CHECK(rig.materials.mode() == MaterialPanel::Mode::None);
    CHECK(rig.materials.showsHint());
    CHECK(rig.materials.slotCount() == 0);
    CHECK(!rig.materials.edited().valid());

    const rt::Rect viewport = rig.shell.panel(1)->absoluteRect();
    const rt::Rect material = rig.shell.panel(EditorShell::kMaterialPanel)->absoluteRect();
    CHECK(material.x >= viewport.x + viewport.w - 1.0f && material.w > 100.0f);
    CHECK(rig.shell.panel(EditorShell::kMaterialPanel)->isVisible());
    CHECK(!rig.shell.panel(2)->isVisible());

    rig.shell.dock()->showPanel("Inspector");
    rig.frames(3);
    CHECK(rig.shell.panel(2)->isVisible() &&
            !rig.shell.panel(EditorShell::kMaterialPanel)->isVisible());
    rig.shell.dock()->showPanel("Material");
    rig.frames(3);
    CHECK(rig.shell.panel(EditorShell::kMaterialPanel)->isVisible() &&
            !rig.shell.panel(2)->isVisible());
    const rt::String saved = rig.shell.layout();
    CHECK(rig.shell.setLayout(
            "H200[{Hierarchy}|H780[V450[{Viewport}|{Console,Assets/1}]|{Inspector}]]"));
    rig.frames(2);
    rig.shell.dock()->showPanel("Material");
    rig.frames(2);
    const rt::Rect again = rig.shell.panel(EditorShell::kMaterialPanel)->absoluteRect();
    CHECK(near(again.x, material.x, 1.0f) && near(again.w, material.w, 1.0f));
    CHECK(strstr(rig.shell.layout().c_str(), "Material") != nullptr);
    CHECK(rig.shell.setLayout(saved));
    rig.frames(2);
}

void testPanelContent(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Box", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    Node3D* plain = scene->createChild<Node3D>();
    plain->setName("Plain");
    rig.frames(3);

    select(rig, plain);
    CHECK(rig.materials.mode() == MaterialPanel::Mode::None);
    CHECK(rig.materials.showsHint());

    select(rig, box);
    MaterialPanel& panel = rig.materials;
    CHECK(panel.mode() == MaterialPanel::Mode::Mesh);
    CHECK(panel.node() == box);
    CHECK(panel.slotCount() == 2);
    CHECK(panel.slotSurface(0) == 0 && panel.slotSurface(1) == MaterialPanel::kOverride);
    CHECK(panel.slotTitle(0) == "Surface 0" && panel.slotTitle(1) == "Override");
    CHECK(panel.slotSource(0) == MaterialProperties::kNoneText);
    CHECK(panel.slotSource(1) == MaterialProperties::kNoneText);
    CHECK(!panel.slotMaterial(0).valid());
    CHECK(panel.slotButton(0, MaterialPanel::kNew)->isEnabled());
    CHECK(!panel.slotButton(0, MaterialPanel::kEdit)->isEnabled());
    CHECK(!panel.slotButton(0, MaterialPanel::kClear)->isEnabled());
    CHECK(panel.showsHint());
    CHECK(!panel.edited().valid());

    const bool wasDirty = rig.document.dirty();
    const MaterialId builtIn = makeBuiltIn("skin", "Assets/Models/body.gltf");
    CHECK(Main::meshes()->setSurfaceMaterial(box->getMesh(), 0, builtIn));
    rig.frames(4);
    CHECK(panel.slotMaterial(0) == builtIn);
    CHECK(panel.slotName(0) == "skin");
    CHECK(panel.slotSource(0) == MaterialProperties::kBuiltInText);
    CHECK(panel.slotButton(0, MaterialPanel::kEdit)->isEnabled());
    CHECK(!panel.slotButton(0, MaterialPanel::kClear)->isEnabled());
    CHECK(panel.edited() == builtIn && panel.editedSlot() == 0);
    CHECK(!panel.editable());
    CHECK(!panel.showsHint());
    CHECK(panel.uniqueButton()->isVisible());
    CHECK(panel.nameLabel()->text() == "skin");
    CHECK(panel.sourceLabel()->text() == MaterialProperties::kBuiltInText);
    CHECK(!panel.saveButton()->isEnabled());
    CHECK(near(static_cast<float>(panel.fieldValue("roughness").asFloat()), 0.3f));
    CHECK(!panel.setField("roughness", PropertyValue::fromSingle(0.9f)));
    CHECK(near(store().get(builtIn)->roughness, 0.3f));
    CHECK(!panel.dirty());
    CHECK(rig.document.dirty() == wasDirty);

    CHECK(Main::meshes()->setSurfaceMaterial(box->getMesh(), 0, MaterialId()));
    select(rig, plain);
    rig.frames(3);
    CHECK(panel.mode() == MaterialPanel::Mode::None);
    CHECK(panel.slotCount() == 0);
}

void testNewMaterial(Rig& rig)
{
    showMaterial(rig);
    CHECK(rig.document.createProject(testRoot() + "/projects", "Materials"));
    rig.frames(4);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Box", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    commit(rig);
    select(rig, box);
    MaterialPanel& panel = rig.materials;
    CHECK(panel.materialsFolder() == "Assets/Materials");
    CHECK(panel.uniqueName("Assets/Materials", MaterialPanel::kDefaultName) == "NewMaterial");

    ct::String error;
    CHECK(!panel.newMaterial(MaterialPanel::kOverride, "bad/name", error) && !error.empty());
    CHECK(!exists(absolute(rig, "Assets/Materials/bad")));

    CHECK(panel.newMaterial(MaterialPanel::kOverride, "Painted", error));
    rig.frames(4);
    CHECK(exists(absolute(rig, "Assets/Materials/Painted.mat")));
    MaterialLibraryData data;
    CHECK(MaterialLibraryFormat::load(absolute(rig, "Assets/Materials/Painted.mat"), data, error));
    CHECK(data.materials.size() == 1 && data.materials[0].info.name == "Painted");
    const MaterialId painted = box->getMaterialOverride();
    CHECK(painted.valid());
    CHECK(sceneres::pathOf(painted) == "Assets/Materials/Painted.mat::Painted");
    CHECK(rig.document.dirty());
    rig.frames(3);
    PropertyRow* overrideRow = rig.inspector.properties().rowNamed("material_override");
    CHECK(overrideRow != nullptr);
    if (overrideRow)
        CHECK(overrideRow->value().asString() == "Assets/Materials/Painted.mat::Painted");
    CHECK(panel.edited() == painted && panel.editedSlot() == MaterialPanel::kOverride);
    CHECK(panel.editable());
    CHECK(!panel.showsHint());
    CHECK(panel.slotMaterial(panel.entryOf(MaterialPanel::kOverride)) == painted);
    CHECK(panel.slotSource(panel.entryOf(MaterialPanel::kOverride)) ==
            "Assets/Materials/Painted.mat::Painted");
    CHECK(panel.slotButton(panel.entryOf(MaterialPanel::kOverride), MaterialPanel::kClear)
                    ->isEnabled());
    CHECK(!panel.newMaterial(MaterialPanel::kOverride, "Painted", error));
    CHECK(error.contains("already exists"));
    CHECK(panel.uniqueName("Assets/Materials", "Painted") == "Painted2");

    commit(rig);
    CHECK(!rig.document.dirty());
    CHECK(panel.newMaterial(0, "Surfaced", error));
    CHECK(sceneres::pathOf(box->getSurfaceMaterial(0)) ==
            "Assets/Materials/Surfaced.mat::Surfaced");
    CHECK(rig.document.dirty());
    CHECK(panel.editedSlot() == 0);

    rig.document.markDirty();
    commit(rig);
    const ct::String sceneText = readText(absolute(rig, "Scenes/scene3.scene.json"));
    CHECK(sceneText.contains("Assets/Materials/Painted.mat::Painted"));
    CHECK(sceneText.contains("Assets/Materials/Surfaced.mat::Surfaced"));

    CHECK(panel.clearSlot(MaterialPanel::kOverride));
    rig.frames(4);
    CHECK(!box->getMaterialOverride().valid());
    CHECK(panel.editedSlot() == 0 && panel.edited() == box->getSurfaceMaterial(0));
    CHECK(!panel.clearSlot(MaterialPanel::kOverride));
    CHECK(panel.clearSlot(0));
    rig.frames(4);
    CHECK(!box->getSurfaceMaterial(0).valid());
    CHECK(!panel.edited().valid() && panel.showsHint());

    showMaterial(rig);
    const int entry = panel.entryOf(MaterialPanel::kOverride);
    CHECK(entry >= 0);
    clickWidget(rig, panel.slotButton(entry, MaterialPanel::kNew));
    rt::InputBox* box1 = floatBox();
    CHECK(box1 != nullptr);
    if (box1)
    {
        CHECK(box1->text() == "NewMaterial");
        box1->accepted.emit(ct::String("NewMaterial"));
        rig.frames(4);
        closeFloats(rig);
    }
    CHECK(exists(absolute(rig, "Assets/Materials/NewMaterial.mat")));
    CHECK(sceneres::pathOf(box->getMaterialOverride()) ==
            "Assets/Materials/NewMaterial.mat::NewMaterial");
    CHECK(panel.uniqueName("Assets/Materials", MaterialPanel::kDefaultName) == "NewMaterial2");

    clickWidget(rig, panel.slotButton(panel.entryOf(0), MaterialPanel::kNew));
    rt::InputBox* box2 = floatBox();
    CHECK(box2 != nullptr);
    if (box2)
    {
        CHECK(box2->text() == "NewMaterial2");
        closeFloats(rig);
    }

    const int editEntry = panel.entryOf(MaterialPanel::kOverride);
    CHECK(panel.editSlot(MaterialPanel::kOverride));
    CHECK(panel.editedSlot() == MaterialPanel::kOverride);
    clickWidget(rig, panel.slotButton(editEntry, MaterialPanel::kClear));
    CHECK(!box->getMaterialOverride().valid());
    rig.frames(4);
    CHECK(!panel.edited().valid());
}

void testAssetsNew(Rig& rig)
{
    showAssets(rig);
    CHECK(rig.assets.navigateTo("Assets"));
    rig.frames(3);
    rig.assets.showMenu(-1, 400.0f, 600.0f);
    rig.frames(2);
    rt::Menu* menu = rig.assets.contextMenu();
    rt::MenuAction* action = nullptr;
    for (int i = 0; i < menu->actionCount(); ++i)
        if (menu->action(i)->text() == "New Material...") action = menu->action(i);
    CHECK(action != nullptr);
    if (!action) return;
    CHECK(action->isEnabled());
    rt::WidgetApp::instance().closePopup();
    rig.frames(2);
    action->triggered.emit();
    rig.frames(3);
    rt::InputBox* box = floatBox();
    CHECK(box != nullptr);
    if (box)
    {
        CHECK(box->text() == "NewMaterial");
        box->accepted.emit(ct::String("InFolder"));
        rig.frames(4);
        closeFloats(rig);
    }
    CHECK(exists(absolute(rig, "Assets/InFolder.mat")));
    CHECK(rig.materials.mode() == MaterialPanel::Mode::Library);
    CHECK(rig.materials.libraryPath() == "Assets/InFolder.mat");
    CHECK(rig.materials.editable());
    CHECK(rig.assets.indexOf("InFolder.mat") >= 0);
    CHECK(rig.assets.selected() == rig.assets.indexOf("InFolder.mat"));
    showMaterial(rig);
}

void testEditing(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Edited", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    select(rig, box);
    MaterialPanel& panel = rig.materials;
    ct::String error;
    CHECK(writePng(absolute(rig, "Assets/Textures/a.png"), 8, 8, 200));
    CHECK(panel.newMaterial(MaterialPanel::kOverride, "Everything", error));
    rig.frames(4);
    const MaterialId id = panel.edited();
    CHECK(id.valid() && panel.editable());
    CHECK(!panel.isDirty(id));

    CHECK(rig.panel.gizmo() != nullptr);
    for (std::size_t i = 0; i < MaterialProperties::count(); ++i)
    {
        const MaterialField& field = MaterialProperties::field(i);
        const PropertyValue before = panel.fieldValue(field.key);
        const PropertyValue wanted = testValue(field, before);
        const bool written = panel.setField(field.key, wanted);
        if (!written) printf("field %s was refused\n", field.key);
        CHECK(written);
        const PropertyValue after = panel.fieldValue(field.key);
        if (!sameValue(after, wanted)) printf("field %s did not keep the value\n", field.key);
        CHECK(sameValue(after, wanted));
        CHECK(!sameValue(after, before) || field.kind == MaterialFieldKind::Float);
        panel.setField(field.key, before);
    }
    CHECK(panel.isDirty(id));
    CHECK(panel.dirty());

    const unsigned writes = panel.writeCount();
    CHECK(panel.setField("metallic", PropertyValue::fromSingle(0.7f)));
    CHECK(near(store().get(id)->metallic, 0.7f));
    float value = 0.0f;
    CHECK(store().getFloat(id, MaterialParam::Metallic, value) && near(value, 0.7f));
    CHECK(panel.writeCount() == writes + 1);
    CHECK(panel.setField("roughness", PropertyValue::fromSingle(0.25f)));
    CHECK(store().getFloat(id, MaterialParam::Roughness, value) && near(value, 0.25f));
    CHECK(panel.setField("metallic_specular", PropertyValue::fromSingle(0.8f)));
    CHECK(store().getFloat(id, MaterialParam::Specular, value) && near(value, 0.8f));
    CHECK(panel.setField("emission_energy_multiplier", PropertyValue::fromSingle(4.0f)));
    CHECK(store().getFloat(id, MaterialParam::EmissionEnergy, value) && near(value, 4.0f));
    CHECK(panel.setField("emission_energy_multiplier", PropertyValue::fromSingle(99.0f)));
    CHECK(store().getFloat(id, MaterialParam::EmissionEnergy, value) && near(value, 16.0f));
    CHECK(panel.setField("normal_scale", PropertyValue::fromSingle(-3.0f)));
    CHECK(store().getFloat(id, MaterialParam::NormalScale, value) && near(value, -3.0f));
    CHECK(panel.setField("clearcoat", PropertyValue::fromSingle(0.6f)));
    CHECK(store().getFloat(id, MaterialParam::Clearcoat, value) && near(value, 0.6f));
    CHECK(panel.setField("ao_light_affect", PropertyValue::fromSingle(0.4f)));
    CHECK(store().getFloat(id, MaterialParam::AoLightAffect, value) && near(value, 0.4f));
    CHECK(panel.setField("alpha_scissor_threshold", PropertyValue::fromSingle(0.35f)));
    CHECK(store().getFloat(id, MaterialParam::AlphaScissorThreshold, value) && near(value, 0.35f));
    CHECK(panel.setField("refraction_scale", PropertyValue::fromSingle(-0.5f)));
    CHECK(store().getFloat(id, MaterialParam::Refraction, value) && near(value, -0.5f));
    Math::Vec4 color;
    CHECK(panel.setField("albedo_color",
            PropertyValue::fromColor(Math::Vec4(0.9f, 0.1f, 0.2f, 0.8f))));
    CHECK(store().getColor(id, MaterialParam::Albedo, color));
    CHECK(near(color.x, 0.9f) && near(color.y, 0.1f) && near(color.z, 0.2f) && near(color.w, 0.8f));
    Math::Vec3 vector;
    CHECK(panel.setField("uv1_scale", PropertyValue::fromVec3(Math::Vec3(2, 3, 1))));
    CHECK(store().getVec3(id, MaterialParam::Uv1Scale, vector));
    CHECK(near(vector.x, 2.0f) && near(vector.y, 3.0f));
    CHECK(panel.setField("uv2_offset", PropertyValue::fromVec3(Math::Vec3(0.5f, 0.25f, 0))));
    CHECK(store().getVec3(id, MaterialParam::Uv2Offset, vector));
    CHECK(near(vector.x, 0.5f) && near(vector.y, 0.25f));
    CHECK(panel.setField("transparency", PropertyValue::fromInt(2)));
    CHECK(store().get(id)->transparency == MaterialTransparency::AlphaBlend);
    CHECK(panel.setField("cull_mode", PropertyValue::fromInt(2)));
    CHECK(store().get(id)->cull == MaterialCull::Disabled);
    CHECK(panel.setField("blend_mode", PropertyValue::fromInt(4)));
    CHECK(store().get(id)->blend == MaterialBlend::Screen);
    CHECK(panel.setField("shading_mode", PropertyValue::fromInt(1)));
    CHECK(store().get(id)->shading == MaterialShading::Unshaded);
    CHECK(panel.setField("shading_mode", PropertyValue::fromInt(0)));
    CHECK(panel.setField("depth_test", PropertyValue::fromBool(false)));
    CHECK(!store().get(id)->depthTest);
    CHECK(panel.setField("receive_shadows", PropertyValue::fromBool(false)));
    CHECK(!store().get(id)->receiveShadows);
    CHECK(panel.setField("detail_enabled", PropertyValue::fromBool(true)));
    CHECK(store().get(id)->detail);
    CHECK(panel.setField("detail_blend_mode", PropertyValue::fromInt(3)));
    CHECK(store().get(id)->detailBlend == MaterialBlend::Mul);
    CHECK(panel.setField("metallic_texture_channel", PropertyValue::fromInt(2)));
    CHECK(store().get(id)->metallicChannel == MaterialTextureChannel::Blue);
    CHECK(panel.setField("billboard_mode", PropertyValue::fromInt(1)));
    CHECK(store().get(id)->billboard == MaterialBillboard::FixedY);
    CHECK(panel.setField("grass", PropertyValue::fromBool(true)));
    CHECK(panel.setField("grass_draw_distance", PropertyValue::fromSingle(77.0f)));
    CHECK(store().get(id)->grass && near(store().get(id)->grassDrawDistance, 77.0f));
    CHECK(panel.setField("water", PropertyValue::fromBool(true)));
    CHECK(panel.setField("water_bump", PropertyValue::fromSingle(2.5f)));
    CHECK(store().get(id)->water && near(store().get(id)->waterData.bump, 2.5f));

    CHECK(panel.nameLabel()->text() == "Everything *");
    CHECK(panel.saveButton()->isEnabled());
    CHECK(rig.document.dirty());

    CHECK(panel.setField("metallic", PropertyValue::fromSingle(0.0f)));
    PropertyRow* metallic = panel.row("metallic");
    CHECK(metallic != nullptr && metallic->field(0) != nullptr && metallic->field(0)->ranged());
    if (metallic && metallic->field(0))
    {
        scrollTo(rig, metallic->field(0));
        clickWidget(rig, metallic->field(0));
        CHECK(metallic->field(0)->isEditing());
        typeText(rig, "0.55");
        rig.send(keyEvent(Key::Enter, true));
        rig.send(keyEvent(Key::Enter, false));
        rig.frames(2);
        CHECK(near(store().get(id)->metallic, 0.55f));
        CHECK(metallic->revertible());
        dragGrabber(rig, metallic->field(0), 0.65f);
        CHECK(fabsf(store().get(id)->metallic - 0.65f) < 0.05f);
        metallic->field(0)->setValue(7.0f);
        metallic->field(0)->valueChanged.emit(7.0f);
        rig.frames(2);
        CHECK(near(store().get(id)->metallic, 1.0f));
        CHECK(near(metallic->field(0)->value(), 1.0f));
        metallic->revertRequested.emit();
        rig.frames(2);
        CHECK(near(store().get(id)->metallic, 0.0f));
        CHECK(!metallic->revertible());
    }
    PropertyRow* cull = panel.row("cull_mode");
    CHECK(cull != nullptr && cull->combo() != nullptr);
    if (cull && cull->combo())
    {
        cull->combo()->setSelectedIndex(1);
        rig.frames(2);
        CHECK(store().get(id)->cull == MaterialCull::Front);
    }
    PropertyRow* depth = panel.row("depth_test");
    CHECK(depth != nullptr && depth->check() != nullptr);
    if (depth && depth->check())
    {
        depth->check()->toggled.emit(true);
        rig.frames(2);
        CHECK(store().get(id)->depthTest);
    }
    PropertyRow* albedo = panel.row("albedo_color");
    CHECK(albedo != nullptr && albedo->swatch() != nullptr);
    if (albedo && albedo->swatch())
    {
        albedo->swatch()->colorEdited.emit(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
        rig.frames(2);
        CHECK(near(store().get(id)->albedo.z, 0.3f));
    }
    PropertyRow* scale = panel.row("uv1_scale");
    CHECK(scale != nullptr && scale->field(1) != nullptr);
    if (scale && scale->field(1))
    {
        scale->field(1)->setValue(4.0f);
        scale->field(1)->valueChanged.emit(4.0f);
        rig.frames(2);
        CHECK(near(store().get(id)->uv1Scale.y, 4.0f));
    }
    CHECK(panel.row("albedo_texture") == nullptr && panel.textureRow("albedo_texture") != nullptr);
    CHECK(panel.row("nothing") == nullptr && panel.textureRow("nothing") == nullptr);
    CHECK(!panel.setField("nothing", PropertyValue::fromBool(true)));
}

void testConditional(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Cond", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    select(rig, box);
    MaterialPanel& panel = rig.materials;
    ct::String error;
    CHECK(panel.newMaterial(MaterialPanel::kOverride, "Conditional", error));
    rig.frames(4);
    const MaterialId id = panel.edited();
    CHECK(id.valid());

    CHECK(panel.groupShown("Transparency") && panel.fieldShown("transparency"));
    CHECK(panel.fieldShown("cull_mode") && panel.fieldShown("depth_test"));
    CHECK(!panel.fieldShown("alpha_scissor_threshold"));
    CHECK(!panel.fieldShown("alpha_antialiasing_mode"));
    CHECK(!panel.fieldShown("alpha_antialiasing_edge"));
    CHECK(panel.fieldShown("blend_mode"));
    CHECK(panel.setField("transparency", PropertyValue::fromInt(1)));
    CHECK(panel.fieldShown("alpha_scissor_threshold"));
    CHECK(panel.fieldShown("alpha_antialiasing_mode"));
    CHECK(!panel.fieldShown("alpha_antialiasing_edge"));
    CHECK(panel.setField("alpha_antialiasing_mode", PropertyValue::fromInt(1)));
    CHECK(panel.fieldShown("alpha_antialiasing_edge"));
    CHECK(!panel.fieldShown("blend_mode"));
    CHECK(panel.setField("transparency", PropertyValue::fromInt(2)));
    CHECK(!panel.fieldShown("alpha_scissor_threshold"));
    CHECK(!panel.fieldShown("alpha_antialiasing_mode"));
    CHECK(!panel.fieldShown("alpha_antialiasing_edge"));
    CHECK(panel.fieldShown("blend_mode"));
    CHECK(panel.setField("transparency", PropertyValue::fromInt(0)));
    CHECK(!panel.fieldShown("alpha_scissor_threshold"));

    CHECK(panel.groupShown("Emission") && panel.fieldShown("emission_enabled"));
    CHECK(!panel.fieldShown("emission") && !panel.fieldShown("emission_energy_multiplier"));
    CHECK(!panel.fieldShown("emission_operator") && !panel.fieldShown("emission_texture"));
    CHECK(panel.setField("emission_enabled", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("emission") && panel.fieldShown("emission_energy_multiplier"));
    CHECK(panel.fieldShown("emission_operator") && panel.fieldShown("emission_texture"));
    CHECK(store().get(id)->emission.x > 0.0f);
    CHECK(panel.setField("emission", PropertyValue::fromColor(Math::Vec4(0.0f, 0.5f, 0.0f, 1.0f))));
    CHECK(near(store().get(id)->emission.y, 0.5f));
    CHECK(panel.setField("emission_enabled", PropertyValue::fromBool(false)));
    CHECK(!panel.fieldShown("emission"));
    CHECK(store().get(id)->emission.x == 0.0f && store().get(id)->emission.y == 0.0f);
    CHECK(panel.setField("emission_enabled", PropertyValue::fromBool(true)));
    CHECK(near(store().get(id)->emission.y, 0.5f));

    CHECK(panel.fieldShown("normal_texture") && !panel.fieldShown("normal_scale"));
    CHECK(panel.setField("normal_texture",
            PropertyValue::fromResourcePath("Assets/Textures/a.png")));
    CHECK(panel.fieldShown("normal_scale"));
    CHECK(panel.setField("normal_texture", PropertyValue::fromResourcePath("")));
    CHECK(!panel.fieldShown("normal_scale"));

    CHECK(panel.fieldShown("clearcoat_enabled") && !panel.fieldShown("clearcoat"));
    CHECK(panel.setField("clearcoat_enabled", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("clearcoat") && panel.fieldShown("clearcoat_roughness"));
    CHECK(store().get(id)->clearcoat > 0.0f);
    CHECK(panel.setField("clearcoat_enabled", PropertyValue::fromBool(false)));
    CHECK(store().get(id)->clearcoat == 0.0f && !panel.fieldShown("clearcoat_roughness"));

    CHECK(panel.fieldShown("backlight_enabled") && !panel.fieldShown("backlight"));
    CHECK(panel.setField("backlight_enabled", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("backlight"));
    CHECK(store().get(id)->backlight.x > 0.0f);

    CHECK(panel.fieldShown("refraction_enabled") && !panel.fieldShown("refraction_scale"));
    CHECK(panel.setField("refraction_enabled", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("refraction_scale") && panel.fieldShown("refraction_texture"));
    CHECK(store().get(id)->refraction != 0.0f);

    CHECK(!panel.fieldShown("detail_mask") && !panel.fieldShown("detail_albedo"));
    CHECK(panel.setField("detail_enabled", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("detail_mask") && panel.fieldShown("detail_blend_mode"));
    CHECK(panel.fieldShown("detail_uv_layer") && panel.fieldShown("detail_normal"));

    CHECK(!panel.fieldShown("billboard_keep_scale"));
    CHECK(panel.setField("billboard_mode", PropertyValue::fromInt(1)));
    CHECK(panel.fieldShown("billboard_keep_scale"));

    CHECK(!panel.fieldShown("grass_draw_distance") && !panel.fieldShown("water_bump"));
    CHECK(panel.setField("grass", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("grass_draw_distance") && panel.fieldShown("grass_tint_a"));
    CHECK(!panel.fieldShown("leaf_wind_strength"));
    CHECK(panel.setField("leaf_wind", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("leaf_wind_strength"));
    CHECK(panel.setField("water", PropertyValue::fromBool(true)));
    CHECK(panel.fieldShown("water_bump") && panel.fieldShown("water_depth_fade"));

    CHECK(panel.fieldShown("metallic") && panel.fieldShown("metallic_texture"));
    CHECK(panel.fieldShown("roughness_texture") && panel.fieldShown("ao_texture"));
    CHECK(panel.setField("orm_texture", PropertyValue::fromResourcePath("Assets/Textures/a.png")));
    CHECK(!panel.fieldShown("metallic_texture") && !panel.fieldShown("metallic_texture_channel"));
    CHECK(!panel.fieldShown("roughness_texture") && !panel.fieldShown("ao_texture"));
    CHECK(panel.fieldShown("metallic") && panel.fieldShown("roughness"));
    CHECK(panel.fieldShown("ao_light_affect"));
    CHECK(panel.setField("orm_texture", PropertyValue::fromResourcePath("")));
    CHECK(panel.fieldShown("metallic_texture") && !panel.fieldShown("ao_light_affect"));

    const char* const litOnly[] = { "Metallic", "Roughness", "Emission", "Normal Map", "Clearcoat",
        "Ambient Occlusion", "Back Lighting" };
    for (const char* name: litOnly) CHECK(panel.groupShown(name));
    CHECK(panel.setField("shading_mode", PropertyValue::fromInt(1)));
    for (const char* name: litOnly)
    {
        if (panel.groupShown(name)) printf("group %s is still shown when unshaded\n", name);
        CHECK(!panel.groupShown(name));
    }
    CHECK(panel.groupShown("Albedo") && panel.groupShown("Transparency") &&
            panel.groupShown("UV1"));
    CHECK(panel.groupShown("Refraction") && panel.groupShown("Detail"));
    CHECK(panel.setField("shading_mode", PropertyValue::fromInt(0)));
    for (const char* name: litOnly) CHECK(panel.groupShown(name));
}

void testTextures(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Textured", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    select(rig, box);
    MaterialPanel& panel = rig.materials;
    ct::String error;
    CHECK(writePng(absolute(rig, "Assets/Textures/b.png"), 4, 4, 90));
    CHECK(panel.newMaterial(MaterialPanel::kOverride, "Textured", error));
    rig.frames(4);
    const MaterialId id = panel.edited();
    MaterialTextureRow* albedo = panel.textureRow("albedo_texture");
    CHECK(id.valid() && albedo != nullptr);
    if (!albedo) return;
    CHECK(albedo->path().empty() && albedo->nameLabel()->text() == MaterialTextureRow::kEmptyText);
    CHECK(!albedo->clearButton()->isEnabled());
    CHECK(!store().getTexture(id, MaterialTexture::Albedo).valid());

    panel.group("Albedo")->setExpanded(true);
    rig.frames(3);
    clickWidget(rig, albedo->browseButton());
    rt::FileDialog* dialog = floatDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        dialog->accepted.emit(absolute(rig, "Assets/Textures/a.png"));
        rig.frames(3);
        closeFloats(rig);
    }
    const TextureId assigned = store().getTexture(id, MaterialTexture::Albedo);
    CHECK(assigned.valid());
    CHECK(sceneres::pathOf(assigned) == "Assets/Textures/a.png");
    CHECK(albedo->path() == "Assets/Textures/a.png");
    CHECK(albedo->nameLabel()->text() == "a.png");
    CHECK(albedo->thumb()->texture() == assigned);
    CHECK(albedo->clearButton()->isEnabled());
    CHECK(panel.isDirty(id) && rig.document.dirty());

    clickWidget(rig, albedo->browseButton());
    rt::FileDialog* outside = floatDialog();
    CHECK(outside != nullptr);
    if (outside)
    {
        outside->accepted.emit(ct::String("/tmp/prisma3d_outside_the_project.png"));
        rig.frames(3);
        closeFloats(rig);
    }
    CHECK(store().getTexture(id, MaterialTexture::Albedo) == assigned);

    albedo->clearButton()->clicked.emit();
    rig.frames(2);
    CHECK(!store().getTexture(id, MaterialTexture::Albedo).valid());
    CHECK(albedo->path().empty() && !albedo->thumb()->texture().valid());

    rt::DragPayload payload;
    payload.type = AssetGrid::kDragType;
    payload.data = ct::String("Assets/Textures/b.png");
    CHECK(albedo->acceptsDrop(payload));
    albedo->onDropReceive(payload);
    rig.frames(2);
    CHECK(sceneres::pathOf(store().getTexture(id, MaterialTexture::Albedo)) ==
            "Assets/Textures/b.png");
    CHECK(albedo->path() == "Assets/Textures/b.png");

    rt::DragPayload notTexture;
    notTexture.type = AssetGrid::kDragType;
    notTexture.data = ct::String("Assets/Materials/Textured.mat");
    CHECK(!albedo->acceptsDrop(notTexture));
    rt::DragPayload foreign;
    foreign.type = "other";
    foreign.data = ct::String("Assets/Textures/b.png");
    CHECK(!albedo->acceptsDrop(foreign));

    showAssets(rig);
    CHECK(rig.assets.navigateTo("Assets/Textures"));
    rig.assets.setViewMode(AssetViewMode::List);
    rig.frames(4);
    const int entry = rig.assets.indexOf("a.png");
    CHECK(entry >= 0);
    rig.assets.grid()->ensureVisible(entry);
    rig.frames(2);
    showMaterial(rig);
    CHECK(rig.shell.panel(EditorShell::kMaterialPanel)->isVisible());
    CHECK(rig.shell.panel(4)->isVisible());
    scrollTo(rig, albedo);
    int fromX = 0;
    int fromY = 0;
    int toX = 0;
    int toY = 0;
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    centerOf(albedo->absoluteRect(), toX, toY);
    albedo->clearButton()->clicked.emit();
    rig.frames(2);
    CHECK(!store().getTexture(id, MaterialTexture::Albedo).valid());
    rig.drag(fromX, fromY, toX, toY);
    CHECK(sceneres::pathOf(store().getTexture(id, MaterialTexture::Albedo)) ==
            "Assets/Textures/a.png");
    CHECK(albedo->path() == "Assets/Textures/a.png");
    CHECK(panel.isDirty(id));

    CHECK(rig.assets.indexOf("b.png") >= 0);
    const int other = rig.assets.indexOf("b.png");
    rig.assets.grid()->ensureVisible(other);
    rig.frames(2);
    centerOf(rig.assets.grid()->itemRect(other), fromX, fromY);
    centerOf(panel.textureRow("normal_texture")->absoluteRect(), toX, toY);
    scrollTo(rig, panel.textureRow("normal_texture"));
    panel.group("Normal Map")->setExpanded(true);
    rig.frames(3);
    scrollTo(rig, panel.textureRow("normal_texture"));
    centerOf(panel.textureRow("normal_texture")->absoluteRect(), toX, toY);
    rig.drag(fromX, fromY, toX, toY);
    CHECK(sceneres::pathOf(store().getTexture(id, MaterialTexture::Normal)) ==
            "Assets/Textures/b.png");
    CHECK(sceneres::pathOf(store().getTexture(id, MaterialTexture::Albedo)) ==
            "Assets/Textures/a.png");

    CHECK(panel.setField("albedo_texture",
                  PropertyValue::fromResourcePath("Assets/Textures/missing.png")) == false);
    CHECK(sceneres::pathOf(store().getTexture(id, MaterialTexture::Albedo)) ==
            "Assets/Textures/a.png");
}

void testSave(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Saved", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    select(rig, box);
    MaterialPanel& panel = rig.materials;
    ct::String error;
    CHECK(panel.newMaterial(MaterialPanel::kOverride, "Persisted", error));
    rig.frames(4);
    const MaterialId id = panel.edited();
    CHECK(id.valid());
    CHECK(panel.setField("albedo_color",
            PropertyValue::fromColor(Math::Vec4(0.9f, 0.5f, 0.1f, 1.0f))));
    CHECK(panel.setField("metallic", PropertyValue::fromSingle(0.8f)));
    CHECK(panel.setField("roughness", PropertyValue::fromSingle(0.15f)));
    CHECK(panel.setField("transparency", PropertyValue::fromInt(1)));
    CHECK(panel.setField("alpha_scissor_threshold", PropertyValue::fromSingle(0.4f)));
    CHECK(panel.setField("cull_mode", PropertyValue::fromInt(2)));
    CHECK(panel.setField("emission_enabled", PropertyValue::fromBool(true)));
    CHECK(panel.setField("emission", PropertyValue::fromColor(Math::Vec4(0.1f, 0.7f, 0.3f, 1.0f))));
    CHECK(panel.setField("emission_energy_multiplier", PropertyValue::fromSingle(2.0f)));
    CHECK(panel.setField("uv1_scale", PropertyValue::fromVec3(Math::Vec3(3, 2, 1))));
    CHECK(panel.setField("albedo_texture",
            PropertyValue::fromResourcePath("Assets/Textures/a.png")));
    CHECK(panel.setField("normal_texture",
            PropertyValue::fromResourcePath("Assets/Textures/b.png")));
    CHECK(panel.setField("normal_scale", PropertyValue::fromSingle(1.5f)));
    CHECK(panel.setField("detail_enabled", PropertyValue::fromBool(true)));
    CHECK(panel.setField("detail_blend_mode", PropertyValue::fromInt(2)));
    CHECK(panel.setField("billboard_mode", PropertyValue::fromInt(1)));
    CHECK(panel.setField("receive_shadows", PropertyValue::fromBool(false)));
    CHECK(panel.setField("grass", PropertyValue::fromBool(true)));
    CHECK(panel.setField("grass_wind_amplitude", PropertyValue::fromSingle(0.5f)));
    CHECK(panel.setField("water", PropertyValue::fromBool(true)));
    CHECK(panel.setField("water_wave_scale", PropertyValue::fromSingle(11.0f)));
    CHECK(panel.dirty());

    const ct::String file = absolute(rig, "Assets/Materials/Persisted.mat");
    CHECK(!readText(file).contains("metallic"));
    CHECK(panel.saveButton()->isEnabled());
    clickWidget(rig, panel.saveButton());
    CHECK(!panel.dirty() && !panel.isDirty(id));
    CHECK(!panel.saveButton()->isEnabled());
    CHECK(panel.nameLabel()->text() == "Persisted");
    const ct::String text = readText(file);
    CHECK(text.contains("metallic"));
    CHECK(text.contains("../Textures/a.png"));
    CHECK(text.contains("../Textures/b.png"));

    MaterialInfo copy = *store().get(id);
    store().destroy(id);
    rig.frames(3);
    const MaterialId reloaded = sceneres::loadMaterial("Assets/Materials/Persisted.mat::Persisted");
    CHECK(reloaded.valid() && reloaded != id);
    const MaterialInfo* info = store().get(reloaded);
    CHECK(info != nullptr);
    if (info)
    {
        MaterialFeatures before = MaterialFeatures::of(copy);
        MaterialFeatures after = MaterialFeatures::of(*info);
        for (std::size_t i = 0; i < MaterialProperties::count(); ++i)
        {
            const MaterialField& field = MaterialProperties::field(i);
            const PropertyValue a = MaterialProperties::read(copy, before, field);
            const PropertyValue b = MaterialProperties::read(*info, after, field);
            if (!sameValue(a, b)) printf("field %s differs after the reload\n", field.key);
            CHECK(sameValue(a, b));
        }
        CHECK(info->detail && info->billboard == MaterialBillboard::FixedY);
        CHECK(sceneres::pathOf(info->textures[static_cast<int>(MaterialTexture::Albedo)]) ==
                "Assets/Textures/a.png");
    }

    ct::String loadError;
    MaterialLibraryData data;
    CHECK(MaterialLibraryFormat::load(file, data, loadError));
    CHECK(data.materials.size() == 1);

    box->setMaterialOverride(reloaded);
    select(rig, nullptr);
    select(rig, box);
    CHECK(panel.edited() == reloaded);
    CHECK(panel.setField("roughness", PropertyValue::fromSingle(0.55f)));
    CHECK(panel.dirty() && rig.document.dirty());
    CHECK(!rig.document.hasScenePath());
    commit(rig);
    CHECK(!panel.dirty());
    CHECK(readText(file).contains("0.55"));

    CHECK(panel.setField("roughness", PropertyValue::fromSingle(0.65f)));
    CHECK(readText(file).contains("0.55"));
    CHECK(rig.send(keyEvent(Key::S, true, kModControl)));
    rig.send(keyEvent(Key::S, false, kModControl));
    rig.frames(4);
    CHECK(!panel.dirty());
    CHECK(readText(file).contains("0.65"));
    CHECK(!rig.document.dirty());
}

void testSiblings(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Library", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    select(rig, box);
    ct::String error;
    MaterialLibraryData data;
    for (int i = 0; i < 3; ++i)
    {
        MaterialDef def;
        def.info.name = i == 0 ? "First" : (i == 1 ? "Second" : "Third");
        def.info.metallic = 0.1f * static_cast<float>(i + 1);
        data.materials.push_back(def);
    }
    const ct::String file = absolute(rig, "Assets/Multi.mat");
    CHECK(MaterialLibraryFormat::save(file, data, error));
    rig.frames(2);

    showAssets(rig);
    CHECK(rig.assets.navigateTo("Assets"));
    rig.assets.setViewMode(AssetViewMode::List);
    rig.frames(4);
    const int entry = rig.assets.indexOf("Multi.mat");
    CHECK(entry >= 0);
    rig.assets.grid()->ensureVisible(entry);
    rig.frames(2);
    int x = 0;
    int y = 0;
    centerOf(rig.assets.grid()->itemRect(entry), x, y);
    rig.doubleClick(x, y);
    MaterialPanel& panel = rig.materials;
    CHECK(panel.mode() == MaterialPanel::Mode::Library);
    CHECK(panel.libraryPath() == "Assets/Multi.mat");
    CHECK(panel.libraryCombo()->itemCount() == 3);
    CHECK(panel.libraryCombo()->selectedIndex() == 0);
    CHECK(panel.edited().valid() && store().get(panel.edited())->name == "First");
    CHECK(panel.slotCount() == 0);
    CHECK(rig.shell.panel(EditorShell::kMaterialPanel)->isVisible());
    rig.frames(3);
    CHECK(panel.libraryCombo()->isVisible());
    CHECK(near(static_cast<float>(panel.fieldValue("metallic").asFloat()), 0.1f));

    panel.libraryCombo()->setSelectedIndex(2);
    rig.frames(3);
    CHECK(store().get(panel.edited())->name == "Third");
    CHECK(near(static_cast<float>(panel.fieldValue("metallic").asFloat()), 0.3f));
    CHECK(panel.setField("metallic", PropertyValue::fromSingle(0.9f)));
    CHECK(panel.dirty());
    CHECK(panel.save(error));
    MaterialLibraryData after;
    CHECK(MaterialLibraryFormat::load(file, after, error));
    CHECK(after.materials.size() == 3);
    CHECK(after.materials[2].info.name == "Third" && near(after.materials[2].info.metallic, 0.9f));
    CHECK(after.materials[0].info.name == "First" && near(after.materials[0].info.metallic, 0.1f));
    CHECK(near(after.materials[1].info.metallic, 0.2f));

    select(rig, box);
    CHECK(panel.mode() == MaterialPanel::Mode::Mesh);
    CHECK(panel.setField("metallic", PropertyValue::fromSingle(0.5f)) == false);

    rig.assets.showMenu(entry, 300.0f, 500.0f);
    rig.frames(2);
    rt::Menu* menu = rig.assets.contextMenu();
    rt::MenuAction* edit = nullptr;
    for (int i = 0; i < menu->actionCount(); ++i)
        if (menu->action(i)->text() == "Edit") edit = menu->action(i);
    CHECK(edit != nullptr);
    if (edit)
    {
        rt::WidgetApp::instance().closePopup();
        rig.frames(2);
        edit->triggered.emit();
        rig.frames(3);
        CHECK(panel.mode() == MaterialPanel::Mode::Library);
        CHECK(panel.libraryPath() == "Assets/Multi.mat");
    }
    CHECK(rig.materials.openLibrary("Assets/Multi.mat::Second"));
    CHECK(store().get(panel.edited())->name == "Second");
    CHECK(panel.libraryCombo()->selectedIndex() == 1);
    CHECK(!rig.materials.openLibrary("Assets/Missing.mat"));
}

void testMakeUnique(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Unique", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    CHECK(writePng(absolute(rig, "Assets/Textures/a.png"), 8, 8, 200));
    const MaterialId builtIn = makeBuiltIn("Body Skin", "Assets/Models/body.gltf");
    const TextureId texture = sceneres::loadTexture("Assets/Textures/a.png", true);
    CHECK(texture.valid());
    store().setTexture(builtIn, MaterialTexture::Albedo, texture);
    store().setFloat(builtIn, MaterialParam::Metallic, 0.45f);
    CHECK(Main::meshes()->setSurfaceMaterial(box->getMesh(), 0, builtIn));
    select(rig, box);
    MaterialPanel& panel = rig.materials;
    CHECK(panel.edited() == builtIn && !panel.editable());
    CHECK(panel.uniqueButton()->isVisible());
    CHECK(panel.sourceLabel()->text() == MaterialProperties::kBuiltInText);

    clickWidget(rig, panel.uniqueButton());
    rig.frames(4);
    const MaterialId copy = panel.edited();
    CHECK(copy.valid() && copy != builtIn);
    CHECK(panel.editable());
    CHECK(!panel.uniqueButton()->isVisible());
    CHECK(box->getSurfaceMaterial(0) == copy);
    CHECK(rig.document.dirty());
    const MaterialInfo* info = store().get(copy);
    CHECK(info != nullptr && info->name == "Body Skin");
    if (info)
    {
        CHECK(info->path == "Assets/Materials/Body Skin.mat");
        CHECK(near(info->roughness, 0.3f) && near(info->metallic, 0.45f));
        CHECK(near(info->albedo.y, 0.6f));
        CHECK(sceneres::pathOf(info->textures[static_cast<int>(MaterialTexture::Albedo)]) ==
                "Assets/Textures/a.png");
    }
    CHECK(exists(absolute(rig, "Assets/Materials/Body Skin.mat")));
    CHECK(near(store().get(builtIn)->roughness, 0.3f));
    CHECK(store().get(builtIn)->path == "Assets/Models/body.gltf");
    CHECK(panel.setField("roughness", PropertyValue::fromSingle(0.9f)));
    CHECK(near(store().get(builtIn)->roughness, 0.3f));
    CHECK(near(store().get(copy)->roughness, 0.9f));
    CHECK(panel.slotMaterial(panel.entryOf(0)) == copy);
    ct::String error;
    CHECK(!panel.makeUnique(error) && !error.empty());

    CHECK(Main::meshes()->setSurfaceMaterial(box->getMesh(), 0, MaterialId()));
    box->setSurfaceMaterial(0, MaterialId());
    box->setMaterialOverride(builtIn);
    rig.frames(4);
    CHECK(panel.editSlot(MaterialPanel::kOverride));
    CHECK(!panel.editable());
    CHECK(panel.makeUnique(error));
    rig.frames(3);
    CHECK(box->getMaterialOverride() != builtIn && box->getMaterialOverride() == panel.edited());
    CHECK(sceneres::pathOf(box->getMaterialOverride()) ==
            "Assets/Materials/Body Skin2.mat::Body Skin2");
}

void testPreview(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* box =
            addMesh(scene, "Previewed", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    select(rig, box);
    MaterialPanel& panel = rig.materials;
    ct::String error;
    CHECK(panel.newMaterial(MaterialPanel::kOverride, "Shiny", error));
    CHECK(panel.setField("albedo_color",
            PropertyValue::fromColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f))));
    rig.frames(6);
    MaterialPreview& preview = panel.preview();
    CHECK(preview.material() == panel.edited());
    CHECK(preview.renderCount() > 0);
    CHECK(preview.texture().valid());

    unsigned char corner[4] = { 0, 0, 0, 0 };
    unsigned char middle[4] = { 0, 0, 0, 0 };
    CHECK(preview.readPixel(2, 2, corner));
    CHECK(preview.readPixel(128, 128, middle));
    CHECK(middle[0] > 60 && middle[0] > middle[2] + 20);
    CHECK(abs(static_cast<int>(corner[0]) - static_cast<int>(middle[0])) > 30);
    CHECK(corner[0] < 100 && corner[1] < 100 && corner[2] < 100);

    unsigned count = preview.renderCount();
    rig.frames(12);
    CHECK(preview.renderCount() == count);

    CHECK(panel.setField("albedo_color",
            PropertyValue::fromColor(Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f))));
    rig.frames(4);
    CHECK(preview.renderCount() > count);
    CHECK(preview.readPixel(128, 128, middle));
    CHECK(middle[2] > 60 && middle[2] > middle[0] + 20);

    count = preview.renderCount();
    clickWidget(rig, panel.shapeButton(MaterialPreview::Shape::Cube));
    rig.frames(4);
    CHECK(preview.shape() == MaterialPreview::Shape::Cube);
    CHECK(preview.renderCount() > count);
    CHECK(preview.readPixel(128, 128, middle));
    CHECK(middle[2] > 40);
    clickWidget(rig, panel.shapeButton(MaterialPreview::Shape::Sphere));
    rig.frames(4);
    CHECK(preview.shape() == MaterialPreview::Shape::Sphere);

    count = preview.renderCount();
    rig.shell.dock()->showPanel("Inspector");
    rig.frames(3);
    CHECK(panel.setField("roughness", PropertyValue::fromSingle(0.1f)));
    rig.frames(6);
    CHECK(preview.renderCount() == count);
    showMaterial(rig);
    CHECK(preview.renderCount() > count);

    const rt::Rect view = panel.previewView()->absoluteRect();
    CHECK(view.h > 100.0f && view.w >= view.h);
    CHECK(panel.setField("albedo_color",
            PropertyValue::fromColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f))));
    rig.frames(6);
    unsigned char shown[4] = { 0, 0, 0, 0 };
    int viewX = 0;
    int viewY = 0;
    centerOf(view, viewX, viewY);
    CHECK(rig.framePixel(viewX, viewY, shown));
    printf("shown %d %d %d\n", shown[0], shown[1], shown[2]);
    CHECK(shown[0] > 60 && shown[0] > shown[2] + 20);
}

void testDropOnScene(Rig& rig)
{
    showMaterial(rig);
    Node* scene = freshScene(rig);
    MeshInstance3D* target =
            addMesh(scene, "Target", createBoxMesh(*Main::meshes(), Math::Vec3(2, 2, 2)));
    Node3D* holder = scene->createChild<Node3D>();
    holder->setName("Holder");
    MeshInstance3D* inner =
            addMesh(holder, "Inner", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    inner->setPosition(Math::Vec3(30.0f, 0.0f, 0.0f));
    MeshInstance3D* row =
            addMesh(scene, "Row", createBoxMesh(*Main::meshes(), Math::Vec3(1, 1, 1)));
    row->setPosition(Math::Vec3(60.0f, 0.0f, 0.0f));
    commit(rig);
    select(rig, nullptr);

    ct::String error;
    ct::String path;
    CHECK(rig.materials.createLibrary("Assets/Drop", "Dropped", path, error));
    CHECK(path == "Assets/Drop/Dropped.mat::Dropped");
    rig.frames(2);

    rig.panel.camera().cursor().posX = 0.0;
    rig.panel.camera().cursor().posY = 0.0;
    rig.panel.camera().cursor().posZ = 0.0;
    rig.panel.camera().cursor().distance = 6.0f;
    rig.panel.camera().updateCamera();
    rig.frames(40);

    showAssets(rig);
    CHECK(rig.assets.navigateTo("Assets/Drop"));
    rig.assets.setViewMode(AssetViewMode::List);
    rig.frames(4);
    const int entry = rig.assets.indexOf("Dropped.mat");
    CHECK(entry >= 0);
    rig.assets.grid()->ensureVisible(entry);
    rig.frames(2);
    int fromX = 0;
    int fromY = 0;
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);

    const rt::Rect viewRect = rig.panel.view()->absoluteRect();
    const int centerX = static_cast<int>(viewRect.x + viewRect.w * 0.5f);
    const int centerY = static_cast<int>(viewRect.y + viewRect.h * 0.5f);
    CHECK(rig.panel.pickAt(static_cast<float>(centerX), static_cast<float>(centerY)) == target);

    CHECK(!target->getMaterialOverride().valid());
    rig.drag(fromX, fromY, centerX, centerY);
    CHECK(target->getMaterialOverride().valid());
    CHECK(sceneres::pathOf(target->getMaterialOverride()) == path);
    CHECK(rig.document.dirty());
    CHECK(!inner->getMaterialOverride().valid());

    commit(rig);
    target->setMaterialOverride(MaterialId());
    commit(rig);
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    rig.drag(fromX, fromY, static_cast<int>(viewRect.x + 4.0f),
            static_cast<int>(viewRect.y + viewRect.h - 4.0f));
    CHECK(!target->getMaterialOverride().valid());
    CHECK(!rig.document.dirty());

    rig.selection.clear();
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(row) != nullptr);
    const rt::Rect outliner = rig.hierarchy.outliner()->absoluteRect();
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    rig.drag(fromX, fromY, textX(rig, row), rowY(rig, row));
    CHECK(row->getMaterialOverride().valid());
    CHECK(sceneres::pathOf(row->getMaterialOverride()) == path);
    CHECK(rig.document.dirty());

    commit(rig);
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    rig.drag(fromX, fromY, textX(rig, holder), rowY(rig, holder));
    CHECK(inner->getMaterialOverride().valid());
    CHECK(rig.document.dirty());

    commit(rig);
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    rig.drag(fromX, fromY, static_cast<int>(outliner.x + 60.0f),
            static_cast<int>(outliner.y + outliner.h - 6.0f));
    CHECK(!rig.document.dirty());

    CHECK(rig.materials.dropMaterial("Assets/Drop/Dropped.mat", row) == 1);
    CHECK(rig.materials.dropMaterial("Assets/Drop/Dropped.mat::Dropped", target) == 1);
    CHECK(rig.materials.dropMaterial("Assets/Drop/Missing.mat", row) == 0);
    CHECK(rig.materials.dropMaterial("Assets/Drop/Dropped.mat", nullptr) == 0);

    select(rig, target);
    showMaterial(rig);
    CHECK(rig.materials.edited() == target->getMaterialOverride());
    commit(rig);
    rig.selection.clear();
    rig.frames(3);
    target->setMaterialOverride(MaterialId());
    rig.document.markDirty();
    commit(rig);
    select(rig, target);
    showMaterial(rig);
    CHECK(rig.materials.edited().valid() == false);
    CHECK(rig.materials.openLibrary(path));
    rig.frames(4);
    CHECK(rig.materials.edited().valid());
    int previewX = 0;
    int previewY = 0;
    const rt::Rect view = rig.materials.previewView()->absoluteRect();
    centerOf(view, previewX, previewY);
    rig.drag(previewX, previewY, centerX, centerY);
    CHECK(sceneres::pathOf(target->getMaterialOverride()) == path);

    row->setMaterialOverride(MaterialId());
    rig.frames(3);
    rig.drag(previewX, previewY, textX(rig, row), rowY(rig, row));
    CHECK(sceneres::pathOf(row->getMaterialOverride()) == path);
    rig.drag(previewX, previewY, textX(rig, holder), rowY(rig, holder));
    CHECK(inner->getMaterialOverride().valid());
}

} // namespace

int main(int argc, char** argv)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor material test";
    desc.driver = useVulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;

    removeTree(testRoot());
    CHECK(fs_create_directory(testRoot().c_str()));
    CHECK(fs_create_directory((testRoot() + "/projects").c_str()));

    SceneTree tree;
    CHECK(Main::start(&tree));
    tree.setAutoAcceptQuit(false);
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    RenderingServer& server = *Main::rendering();
    server.viewportSetActive(window->getViewportId(), false);

    Node3D* scene = tree.getRoot()->createChild<Node3D>();
    scene->setName("Scene");

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        EditorShell shell(host);
        CHECK(host.init());
        CHECK(shell.build());

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
        ProjectUi ui(document);
        CHECK(ui.attach(shell.toolBar()));

        EditorViewportPanel panel(server);
        panel.bindEditor(tree, selection);
        panel.setChangeListener(&document);
        EditorGizmo* gizmo = panel.gizmo();
        CHECK(gizmo != nullptr);
        if (gizmo)
            gizmo->setRootProvider([&document]() -> Node* { return document.scene(); });
        CHECK(panel.attach(shell.panel(1)));
        panel.setScenario(window->getScenario());

        HierarchyActions actions(tree, document, selection, hierarchy);
        CHECK(actions.attach());
        EditorMenus menus(shell, ui, panel, actions);
        CHECK(menus.attach());
        EditorStatusBar status(document, selection, server);
        CHECK(status.attach(shell.statusBar()));
        status.setGizmo(gizmo);

        MaterialPanel materials(server, tree, selection, document);
        CHECK(!materials.attached());
        CHECK(materials.attach(shell.panel(EditorShell::kMaterialPanel)));
        CHECK(materials.attached());
        CHECK(!materials.attach(shell.panel(EditorShell::kMaterialPanel)));
        materials.bindViewport(panel);

        AssetsPanel assets(document, selection, hierarchy, actions, ui);
        CHECK(assets.attach(shell.panel(4)));
        assets.bindViewport(panel);
        assets.bindMaterials(materials);
        materials.setRevealHandler([&shell] { shell.dock()->showPanel("Material"); });

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, panel,
            actions, menus, status, assets, materials };
        CHECK(shell.setLayout("H200[{Hierarchy}|H780[V450[{Viewport}|{Console,Assets/"
                              "1}]|{Inspector,Material/1}]]"));
        rig.frames(4);

        testLayout(rig);
        testPanelContent(rig);
        testNewMaterial(rig);
        testAssetsNew(rig);
        testEditing(rig);
        testConditional(rig);
        testTextures(rig);
        testSave(rig);
        testSiblings(rig);
        testMakeUnique(rig);
        testPreview(rig);
        testDropOnScene(rig);

        rig.frames(3);
        host.shutdown();
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
