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
#include "prisma3d/editor/EditorVersion.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/MeshPropertyEditor.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/PropertyNames.h"
#include "prisma3d/editor/PropertyRow.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include <ct/json.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

using namespace prisma3d;

#define SIGNATURE "prisma3d editor " PRISMA3D_EDITOR_VERSION " \xC2\xB7 djokersoft"
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
    root += useVulkan ? "/editor_project_test_vulkan" : "/editor_project_test";
    return root;
}

struct Editor
{
    SceneTree& tree;
    WidgetHost& host;
    EditorShell& shell;
    EditorSelection& selection;
    SceneTreePanel& hierarchy;
    InspectorPanel& inspector;
    EditorDocument& document;
    ProjectUi& ui;
};

void frame(Editor& editor)
{
    editor.tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    editor.ui.update();
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

InputEvent charEvent(char c)
{
    InputEvent event;
    event.type = InputEventType::Char;
    event.character.codepoint = static_cast<std::uint32_t>(c);
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

void pressKey(Editor& editor, Key key)
{
    editor.host.handleEvent(keyEvent(key, true));
    frame(editor);
    editor.host.handleEvent(keyEvent(key, false));
    frame(editor);
    frame(editor);
}

void typeText(Editor& editor, const char* text)
{
    for (const char* c = text; *c; ++c) editor.host.handleEvent(charEvent(*c));
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

void clickGate(Editor& editor, rt::Collapsible* gate)
{
    reveal(editor, gate);
    const rt::Rect box = gate->checkRect();
    click(editor, static_cast<int>(box.x + box.w * 0.5f), static_cast<int>(box.y + box.h * 0.5f));
}

void clickWidget(Editor& editor, const rt::Widget* widget)
{
    reveal(editor, widget);
    click(editor, centerX(widget), centerY(widget));
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

bool contains(const ct::String& text, const char* fragment)
{
    return strstr(text.c_str(), fragment) != nullptr;
}

bool isFile(const ct::String& path)
{
    PathInfo info;
    return fs_get_path_info(path.c_str(), &info) && info.type == PATH_TYPE_FILE;
}

bool isFolder(const ct::String& path)
{
    PathInfo info;
    return fs_get_path_info(path.c_str(), &info) && info.type == PATH_TYPE_DIRECTORY;
}

ct::String readFile(const ct::String& path)
{
    return FileSystem::getSingleton().readText(path);
}

template<typename T>
T* findWidget(rt::Widget* root, bool (*accept)(const T*, const void*), const void* argument)
{
    if (!root) return nullptr;
    if (T* typed = rt::widget_cast<T>(root))
        if (accept(typed, argument)) return typed;
    for (std::size_t i = 0; i < root->children().size(); ++i)
        if (T* found = findWidget<T>(root->children()[i], accept, argument)) return found;
    return nullptr;
}

bool buttonWithText(const rt::Button* candidate, const void* text)
{
    return candidate->text() == static_cast<const char*>(text);
}

rt::Button* findButton(rt::Widget* root, const char* text)
{
    return findWidget<rt::Button>(root, &buttonWithText, text);
}

rt::TextInput* findInput(rt::Widget* root)
{
    return findWidget<rt::TextInput>(
            root, [](const rt::TextInput*, const void*) { return true; }, nullptr);
}

std::size_t floatCount()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return app ? app->floats().size() : 0;
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

void clickDialogButton(Editor& editor, rt::Dialog* dialog, int index)
{
    CHECK(dialog != nullptr);
    if (!dialog) return;
    const rt::Rect rect = dialog->btnRect_(index);
    click(editor, static_cast<int>(rect.x + rect.w * 0.5f),
            static_cast<int>(rect.y + rect.h * 0.5f));
}

enum ConfirmButton
{
    kConfirmSave = 0,
    kConfirmDiscard = 1,
    kConfirmCancel = 2
};

void typeInto(Editor& editor, NumberField* field, const char* text)
{
    clickWidget(editor, field);
    CHECK(field->isEditing());
    typeText(editor, text);
    pressKey(editor, Key::Enter);
}

void typeInto(Editor& editor, rt::TextInput* input, const char* text, bool submit)
{
    clickWidget(editor, input);
    input->selectAll();
    typeText(editor, text);
    if (submit) pressKey(editor, Key::Enter);
}

ct::String projectManifest(const char* name)
{
    return testRoot() + "/projects/" + name + "/projecto.json";
}

int toolbarCenterX(Editor& editor, int index)
{
    const rt::Rect bar = editor.shell.toolBar()->absoluteRect();
    return static_cast<int>(bar.x + 2.0f + 14.0f + 30.0f * static_cast<float>(index));
}

void testProjectAndScenes(Editor& editor);
void testInspectorTypes(Editor& editor);
void testRows(Editor& editor);
void testUnsaved(Editor& editor);
void testRecents(Editor& editor);

} // namespace

namespace
{

void testProjectAndScenes(Editor& editor)
{
    EditorDocument& document = editor.document;
    Project& project = document.project();
    ProjectUi& ui = editor.ui;
    rt::ToolBar* bar = editor.shell.toolBar();

    CHECK(!project.isOpen());
    CHECK(!document.dirty());
    CHECK(document.scene() != nullptr && document.scene()->getName() == "Scene");
    CHECK(!document.hasScenePath());
    CHECK(document.sceneName() == "Untitled");
    CHECK(ui.windowTitle() == "Untitled - " SIGNATURE);
    for (int i = 0; i < ProjectUi::kButtonCount; ++i)
    {
        CHECK(ui.buttonId(static_cast<ProjectUi::Button>(i)) >= 0);
        CHECK(strlen(ProjectUi::buttonTooltip(static_cast<ProjectUi::Button>(i))) > 0);
    }
    CHECK(bar->isItemEnabled(ui.buttonId(ProjectUi::kNewProject)));
    CHECK(bar->isItemEnabled(ui.buttonId(ProjectUi::kOpenProject)));
    CHECK(!bar->isItemEnabled(ui.buttonId(ProjectUi::kSave)));
    CHECK(!bar->isItemEnabled(ui.buttonId(ProjectUi::kSaveAs)));
    CHECK(!bar->isItemEnabled(ui.buttonId(ProjectUi::kMainScene)));

    ui.save();
    CHECK(!project.isOpen());
    ui.saveAs();
    CHECK(ui.saveAsBox() == nullptr);

    const ct::String parent = testRoot() + "/projects";
    ui.newProject();
    frames(editor, 2);
    NewProjectDialog* dialog = ui.newProjectDialog();
    CHECK(dialog != nullptr);
    if (!dialog) return;
    CHECK(floatCount() == 1);
    ui.newProject();
    frames(editor, 2);
    CHECK(floatCount() == 1);

    dialog->parentInput()->setText(parent);
    dialog->nameInput()->setText("bad/name");

    clickWidget(editor, dialog->createButton());
    CHECK(ui.newProjectDialog() == dialog);
    CHECK(contains(dialog->error(), "not a valid project name"));
    CHECK(!project.isOpen());
    CHECK(!isFolder(parent + "/bad"));

    dialog->nameInput()->setText("");
    clickWidget(editor, dialog->createButton());
    CHECK(ui.newProjectDialog() == dialog);
    CHECK(contains(dialog->error(), "name"));

    dialog->parentInput()->setText("");
    clickWidget(editor, dialog->createButton());
    CHECK(ui.newProjectDialog() == dialog);
    CHECK(contains(dialog->error(), "folder"));

    clickWidget(editor, dialog->cancelButton());
    frames(editor, 2);
    CHECK(ui.newProjectDialog() == nullptr);
    CHECK(!project.isOpen());
    CHECK(document.scene() != nullptr && document.scene()->getName() == "Scene");

    ui.newProject();
    frames(editor, 2);
    dialog = ui.newProjectDialog();
    CHECK(dialog != nullptr);
    if (!dialog) return;
    dialog->parentInput()->setText("/nonexistent");
    clickWidget(editor, dialog->browseButton());
    rt::FileDialog* folders = ui.fileDialog();
    CHECK(folders != nullptr);
    if (folders)
    {
        CHECK(folders->mode() == rt::FileDialog::Mode::SelectFolder);
        folders->accepted.emit(parent);
        rt::WidgetApp::instance().removeFloat(folders);
        frames(editor, 2);
        CHECK(dialog->parentFolder() == parent);
        CHECK(ui.fileDialog() == nullptr);
    }
    dialog->parentInput()->setText(parent);
    dialog->nameInput()->setText("Demo");
    Node* firstScene = document.scene();
    const unsigned int serial = document.sceneSerial();
    clickWidget(editor, dialog->createButton());
    frames(editor, 4);
    CHECK(ui.newProjectDialog() == nullptr);

    const ct::String demo = parent + "/Demo";
    CHECK(project.isOpen() && project.name() == "Demo");
    CHECK(project.root() == demo);
    CHECK(isFolder(demo + "/Scenes"));
    CHECK(isFolder(demo + "/Assets/Scripts"));
    CHECK(isFolder(demo + "/Assets/Models"));
    CHECK(isFolder(demo + "/Assets/Textures"));
    CHECK(isFile(demo + "/projecto.json"));
    CHECK(isFile(demo + "/Scenes/main.scene.json"));
    CHECK(project.scenes().size() == 1 && project.scenes()[0] == "Scenes/main.scene.json");
    CHECK(project.mainScene() == "Scenes/main.scene.json");
    const ct::String manifest = readFile(demo + "/projecto.json");
    CHECK(contains(manifest, "\"activeScene\": \"Scenes/main.scene.json\""));
    CHECK(contains(manifest, "\"scenes\": [\n    \"Scenes/main.scene.json\"\n  ]"));

    CHECK(document.sceneSerial() == serial + 1);
    CHECK(document.scene() != nullptr && document.scene() != firstScene);
    CHECK(document.scene()->getName() == "Main");
    CHECK(strcmp(document.scene()->getClassName(), "Node3D") == 0);
    CHECK(document.scenePath() == "Scenes/main.scene.json");
    CHECK(document.sceneInProject() && document.isMainScene());
    CHECK(!document.dirty());
    CHECK(editor.hierarchy.editedScene() == document.scene());
    CHECK(editor.tree.getRoot()->getChildCount() == 1);
    CHECK(ui.windowTitle() == "Demo - main.scene.json - " SIGNATURE);
    CHECK(bar->isItemEnabled(ui.buttonId(ProjectUi::kSave)));
    CHECK(bar->isItemEnabled(ui.buttonId(ProjectUi::kSaveAs)));
    CHECK(!bar->isItemEnabled(ui.buttonId(ProjectUi::kMainScene)));
    CHECK(document.settings().recentProjectPaths.size() == 1);
    CHECK(document.settings().lastProjectPath == demo + "/projecto.json");
    CHECK(isFile(testRoot() + "/settings.json"));

    SceneResult loaded;
    Node* holder = editor.tree.getRoot()->createChild<Node3D>();
    holder->setName("holder");
    Node* reread = SceneSerializer::loadFile(demo + "/Scenes/main.scene.json", holder, loaded);
    CHECK(reread != nullptr && loaded.success());
    CHECK(reread && reread->getName() == "Main");
    holder->queueFree();
    frames(editor, 2);
    CHECK(!document.dirty());

    Node* lamp = makeNode(editor, "OmniLight3D", "Lamp");
    CHECK(document.dirty());
    ui.update();
    CHECK(ui.windowTitle() == "Demo - main.scene.json* - " SIGNATURE);
    const unsigned int titles = ui.titleSerial();
    frames(editor, 3);
    CHECK(ui.titleSerial() == titles);

    editor.shell.toolBar()->buttonClicked.emit(ui.buttonId(ProjectUi::kSave));
    frames(editor, 2);
    CHECK(!document.dirty());
    CHECK(ui.windowTitle() == "Demo - main.scene.json - " SIGNATURE);
    CHECK(contains(readFile(demo + "/Scenes/main.scene.json"), "Lamp"));
    CHECK(contains(readFile(demo + "/Scenes/main.scene.json"), "OmniLight3D"));

    lamp->setName("Lamp2");
    CHECK(document.dirty());
    lamp->setName("Lamp");
    CHECK(document.dirty());
    CHECK(document.saveScene());
    CHECK(!document.dirty());

    Node* extra = makeNode(editor, "Node3D", "Extra");
    CHECK(document.dirty());
    CHECK(document.saveScene());
    extra->queueFree();
    frames(editor, 2);
    CHECK(document.dirty());
    CHECK(document.saveScene());
    CHECK(!document.dirty());

    Node* second = makeNode(editor, "Node3D", "Second");
    Node* third = makeNode(editor, "Node3D", "Third");
    CHECK(document.saveScene());
    CHECK(!document.dirty());
    document.scene()->moveChild(third, 0);
    frames(editor, 2);
    CHECK(document.dirty());
    CHECK(document.saveScene());
    second->queueFree();
    third->queueFree();
    frames(editor, 2);
    CHECK(document.saveScene());

    editor.selection.clear();
    editor.selection.add(lamp);
    frames(editor, 3);
    CHECK(!document.dirty());
}

} // namespace

namespace
{

void writeImage(const ct::String& path, int red, int green, int blue)
{
    Pixmap pixmap(4, 4, 4);
    pixmap.fill(static_cast<std::uint8_t>(red), static_cast<std::uint8_t>(green),
            static_cast<std::uint8_t>(blue), 255);
    CHECK(pixmap.save(path.c_str()));
}

void cleanScene(Editor& editor)
{
    CHECK(editor.document.saveScene());
    CHECK(!editor.document.dirty());
}

void dragNumber(Editor& editor, NumberField* field, int dx, int steps)
{
    reveal(editor, field);
    const int x = centerX(field);
    const int y = centerY(field);
    editor.host.handleEvent(motion(x, y));
    frame(editor);
    editor.host.handleEvent(button(x, y, true));
    frame(editor);
    for (int i = 1; i <= steps; ++i)
    {
        editor.host.handleEvent(motion(x + dx * i / steps, y));
        frame(editor);
    }
    editor.host.handleEvent(button(x + dx, y, false));
    frame(editor);
    frame(editor);
}

void dragGrabber(Editor& editor, NumberField* field, float ratio)
{
    reveal(editor, field);
    const rt::Rect box = field->absoluteRect();
    editor.host.handleEvent(motion(centerX(field), centerY(field)));
    frames(editor, 2);
    CHECK(field->grabberVisible());
    const rt::Rect grabber = field->grabberRect();
    const int startX = static_cast<int>(grabber.x + grabber.w * 0.5f);
    const int startY = static_cast<int>(grabber.y + grabber.h * 0.5f);
    const float left = box.x + 6.0f;
    const float width = box.w - 12.0f - 4.0f;
    const int targetX = static_cast<int>(left + width * ratio + 2.0f);
    editor.host.handleEvent(motion(startX, startY));
    frame(editor);
    editor.host.handleEvent(button(startX, startY, true));
    frame(editor);
    for (int i = 1; i <= 6; ++i)
    {
        editor.host.handleEvent(motion(startX + (targetX - startX) * i / 6, startY));
        frame(editor);
    }
    CHECK(field->isGrabbing());
    editor.host.handleEvent(button(targetX, startY, false));
    frames(editor, 2);
}

void testStructure(Editor& editor, Node* lamp)
{
    PropertyInspector& inspector = editor.inspector.properties();
    NodeClassDB::setProperty(lamp, "shadow_enabled", PropertyValue::fromBool(true));
    select(editor, lamp);
    CHECK(inspector.node() == lamp);
    CHECK(inspector.rowCount() > 12);
    CHECK(inspector.sectionCount() >= 4);

    const char* hidden[] = { "position", "rotation", "scale", "quaternion", "name",
        "rotation_degrees", "global_position", "Transform", "visible", "process_mode" };
    for (const char* name: hidden) CHECK(inspector.rowNamed(name) == nullptr);
    CHECK(inspector.section("Transform") == nullptr);
    CHECK(inspector.section("Visibility") == nullptr);

    rt::Collapsible* collision = inspector.section("Collision");
    rt::Collapsible* light = inspector.section("Light");
    rt::Collapsible* shadow = inspector.section("Shadow");
    rt::Collapsible* omni = inspector.section("Omni");
    CHECK(collision && light && shadow && omni);
    if (!(collision && light && shadow && omni)) return;
    CHECK(light->title() == "Light");
    CHECK(collision->absoluteRect().y < light->absoluteRect().y);
    CHECK(light->absoluteRect().y < shadow->absoluteRect().y);
    CHECK(shadow->absoluteRect().y < omni->absoluteRect().y);

    PropertyRow* energy = inspector.rowNamed("light_energy");
    PropertyRow* size = inspector.rowNamed("light_size");
    PropertyRow* bias = inspector.rowNamed("shadow_normal_bias");
    PropertyRow* type = inspector.rowNamed("collision_type");
    CHECK(energy && size && bias && type == nullptr);
    if (!(energy && size && bias)) return;
    CHECK(inspector.gate("collision_enabled") == collision);
    CHECK(collision->isCheckable() && !collision->isChecked());
    CHECK(collision->children().empty());
    CHECK(energy->parent() == light && energy->label() == "Energy");
    rt::Collapsible* sizeSection = rt::widget_cast<rt::Collapsible>(size->parent());
    CHECK(sizeSection && sizeSection != light && sizeSection->title() == "Light");
    CHECK(size->label() == "Size");
    CHECK(bias->parent() == shadow && bias->label() == "Normal Bias");
    CHECK(inspector.rowNamed("omni_shadow_mode")->label() == "Shadow Mode");
    CHECK(inspector.rowNamed("omni_shadow_mode")->parent() == omni);
    CHECK(inspector.rowNamed("layers") == nullptr);
    CHECK(energy->kind() == PropertyRow::Kind::Float && energy->field(0)->ranged());
    CHECK(inspector.rowNamed("light_color")->kind() == PropertyRow::Kind::Color);
    CHECK(inspector.rowNamed("light_cull_mask")->kind() == PropertyRow::Kind::Layers);
    CHECK(inspector.rowNamed("collision_shape") == nullptr);
    CHECK(inspector.rowNamed("collision_radii") == nullptr);
    CHECK(inspector.rowNamed("collision_box_min") == nullptr);
    CHECK(inspector.rowNamed("collision_mesh") == nullptr);
    CHECK(inspector.rowNamed("shadow_enabled") == nullptr);
    CHECK(inspector.gate("shadow_enabled") == shadow);
    CHECK(shadow->isCheckable() && shadow->isChecked());

    for (std::size_t i = 0; i < inspector.rowCount(); ++i)
    {
        PropertyRow* row = inspector.row(i);
        CHECK(row != nullptr && row->kind() != PropertyRow::Kind::None);
        CHECK(row->value().type() == row->info().type);
        PropertyValue current;
        CHECK(NodeClassDB::getProperty(lamp, inspector.rowName(i), current));
        CHECK(!(current != row->value()));
    }

    const unsigned int rebuilds = inspector.rebuildCount();
    reveal(editor, shadow);
    const float expandedHeight = shadow->absoluteRect().h;
    click(editor, static_cast<int>(shadow->absoluteRect().x + 120.0f),
            static_cast<int>(shadow->absoluteRect().y + shadow->headerHeight() * 0.5f));
    CHECK(!shadow->isExpanded());
    CHECK(shadow->absoluteRect().h < expandedHeight);
    CHECK(shadow->absoluteRect().h <= shadow->headerHeight() + 1.0f);
    select(editor, nullptr);
    CHECK(inspector.rowCount() == 0 && inspector.node() == nullptr);
    select(editor, lamp);
    CHECK(inspector.rebuildCount() == rebuilds + 1);
    CHECK(inspector.section("Shadow") && !inspector.section("Shadow")->isExpanded());
    CHECK(inspector.section("Light") && inspector.section("Light")->isExpanded());
    inspector.section("Shadow")->setExpanded(true);
    frames(editor, 2);
    CHECK(inspector.section("Shadow")->absoluteRect().h >
            inspector.section("Shadow")->headerHeight());

    Node* other = makeNode(editor, "OmniLight3D", "Other");
    NodeClassDB::setProperty(other, "light_energy", PropertyValue::fromFloat(3.5));
    NodeClassDB::setProperty(other, "shadow_enabled", PropertyValue::fromBool(true));
    const unsigned int before = inspector.rebuildCount();
    select(editor, other);
    CHECK(inspector.rebuildCount() == before);
    CHECK(near(inspector.rowNamed("light_energy")->field(0)->value(), 3.5f));
    CHECK(inspector.rowNamed("light_energy")->revertible());
    other->queueFree();
    frames(editor, 3);
    CHECK(inspector.node() == nullptr && inspector.rowCount() == 0);
    CHECK(editor.inspector.showsNoSelection());
    select(editor, lamp);
}

void testScalars(Editor& editor, Node* lamp)
{
    PropertyInspector& inspector = editor.inspector.properties();
    EditorDocument& document = editor.document;
    NodeClassDB::setProperty(lamp, "shadow_enabled", PropertyValue::fromBool(false));
    select(editor, lamp);
    cleanScene(editor);

    CHECK(inspector.rowNamed("shadow_enabled") == nullptr);
    rt::Collapsible* shadowGate = inspector.gate("shadow_enabled");
    CHECK(shadowGate && shadowGate->isCheckable() && !shadowGate->isChecked());
    if (!shadowGate) return;
    CHECK(!property(lamp, "shadow_enabled").asBool());
    CHECK(inspector.rowNamed("shadow_bias") == nullptr &&
            inspector.rowNamed("shadow_normal_bias") == nullptr &&
            inspector.rowNamed("shadow_opacity") == nullptr &&
            inspector.rowNamed("shadow_blur") == nullptr);
    CHECK(inspector.rowNamed("light_energy") != nullptr);
    const unsigned int writes = inspector.writeCount();
    clickGate(editor, shadowGate);
    frames(editor, 3);
    shadowGate = inspector.gate("shadow_enabled");
    CHECK(property(lamp, "shadow_enabled").asBool());
    CHECK(shadowGate && shadowGate->isChecked());
    CHECK(inspector.rowNamed("shadow_bias") != nullptr &&
            inspector.rowNamed("shadow_normal_bias") != nullptr &&
            inspector.rowNamed("shadow_opacity") != nullptr &&
            inspector.rowNamed("shadow_blur") != nullptr);
    CHECK(inspector.writeCount() == writes + 1);
    CHECK(document.dirty());
    clickGate(editor, shadowGate);
    frames(editor, 3);
    shadowGate = inspector.gate("shadow_enabled");
    CHECK(!property(lamp, "shadow_enabled").asBool());
    CHECK(shadowGate && !shadowGate->isChecked());
    CHECK(inspector.rowNamed("shadow_bias") == nullptr);
    cleanScene(editor);
    clickGate(editor, shadowGate);
    frames(editor, 3);
    CHECK(property(lamp, "shadow_enabled").asBool());

    PropertyRow* energy = inspector.rowNamed("light_energy");
    CHECK(energy && energy->field(0) && energy->field(0)->ranged());
    const float defaultEnergy = static_cast<float>(property(lamp, "light_energy").asFloat());
    typeInto(editor, energy->field(0), "5");
    CHECK(near(static_cast<float>(property(lamp, "light_energy").asFloat()), 5.0f));
    CHECK(near(energy->field(0)->value(), 5.0f));
    CHECK(document.dirty());
    CHECK(energy->revertible() == (defaultEnergy != 5.0f));

    typeInto(editor, energy->field(0), "20");
    CHECK(near(static_cast<float>(property(lamp, "light_energy").asFloat()), 20.0f));
    CHECK(near(energy->field(0)->value(), 20.0f));
    typeInto(editor, energy->field(0), "-4");
    CHECK(near(static_cast<float>(property(lamp, "light_energy").asFloat()), 0.0f));
    CHECK(near(energy->field(0)->value(), 0.0f));

    dragGrabber(editor, energy->field(0), 0.5f);
    const float middle = static_cast<float>(property(lamp, "light_energy").asFloat());
    CHECK(middle > 4.0f && middle < 12.0f);
    CHECK(near(energy->field(0)->value(), middle, 0.01f));
    clickWidget(editor, energy->revertButton());
    CHECK(near(static_cast<float>(property(lamp, "light_energy").asFloat()), defaultEnergy));
    CHECK(!energy->revertible());

    PropertyRow* bias = inspector.rowNamed("shadow_bias");
    typeInto(editor, bias->field(0), "100");
    CHECK(near(static_cast<float>(property(lamp, "shadow_bias").asFloat()), 10.0f));
    CHECK(near(bias->field(0)->value(), 10.0f));
    typeInto(editor, bias->field(0), "-1");
    CHECK(near(static_cast<float>(property(lamp, "shadow_bias").asFloat()), 0.0f));
    typeInto(editor, bias->field(0), "0.12345");
    CHECK(near(static_cast<float>(property(lamp, "shadow_bias").asFloat()), 0.123f, 1.0e-5f));
    CHECK(near(bias->field(0)->value(), 0.123f, 1.0e-5f));
    CHECK(near(bias->field(0)->dragSensitivity(), 0.005f, 1.0e-6f));
    const unsigned int sameWrites = inspector.writeCount();
    typeInto(editor, bias->field(0), "0.1231");
    CHECK(inspector.writeCount() == sameWrites);
    CHECK(near(bias->field(0)->value(), 0.123f, 1.0e-5f));

    PropertyRow* blur = inspector.rowNamed("shadow_blur");
    typeInto(editor, blur->field(0), "2");
    const float start = static_cast<float>(property(lamp, "shadow_blur").asFloat());
    dragNumber(editor, blur->field(0), 40, 8);
    const float dragged = static_cast<float>(property(lamp, "shadow_blur").asFloat());
    CHECK(near(start, 2.0f));
    CHECK(near(dragged, 2.2f, 0.01f));
    CHECK(near(blur->field(0)->value(), dragged, 0.001f));

    PropertyRow* mode = inspector.rowNamed("omni_shadow_mode");
    CHECK(mode && mode->combo() && mode->choiceCount() == 2);
    CHECK(mode->combo()->itemCount() == 2 && mode->combo()->itemText(1) == "Cube");
    const std::int64_t before = property(lamp, "omni_shadow_mode").asInt();
    CHECK(mode->combo()->selectedIndex() == static_cast<int>(before));
    reveal(editor, mode->combo());
    click(editor, centerX(mode->combo()), centerY(mode->combo()));
    rt::Widget* popup = rt::WidgetApp::instance().popup();
    CHECK(popup != nullptr);
    if (popup)
    {
        const rt::Rect rect = popup->absoluteRect();
        const float row = rt::Theme::instance().inputHeight;
        const int target = before == 0 ? 1 : 0;
        click(editor, static_cast<int>(rect.x + rect.w * 0.5f),
                static_cast<int>(rect.y + row * (static_cast<float>(target) + 0.5f)));
        CHECK(property(lamp, "omni_shadow_mode").asInt() == target);
        CHECK(mode->combo()->selectedIndex() == target);
        PropertyValue modeDefault;
        CHECK(NodeClassDB::getDefaultPropertyValue("OmniLight3D", "omni_shadow_mode", modeDefault));
        CHECK(mode->revertible() == (target != modeDefault.asInt()));
    }

    CHECK(inspector.rowNamed("collision_radii") == nullptr);
    CHECK(inspector.rowNamed("collision_type") == nullptr);
    CHECK(inspector.rowNamed("collision_shape") == nullptr);
    rt::Collapsible* gate = inspector.gate("collision_enabled");
    CHECK(gate && gate->isCheckable() && !gate->isChecked());
    if (!gate) return;
    {
        reveal(editor, gate);
        const rt::Rect box = gate->checkRect();
        click(editor, static_cast<int>(box.x + box.w * 0.5f),
                static_cast<int>(box.y + box.h * 0.5f));
    }
    CHECK(property(lamp, "collision_type").asInt() == 1);
    frames(editor, 3);
    gate = inspector.gate("collision_enabled");
    CHECK(gate && gate->isChecked());
    CHECK(inspector.rowNamed("collision_type") != nullptr);
    CHECK(inspector.rowNamed("collision_type")->label() == "Group");
    CHECK(inspector.rowNamed("collision_shape") != nullptr);
    NodeClassDB::setProperty(lamp, "collision_shape", PropertyValue::fromInt(1));
    frames(editor, 3);
    CHECK(inspector.rowNamed("collision_box_min") == nullptr);
    PropertyRow* radii = inspector.rowNamed("collision_radii");
    CHECK(radii && radii->componentCount() == 3);
    if (!radii) return;
    const Math::Vec3 radiiBefore = property(lamp, "collision_radii").asVec3();
    typeInto(editor, radii->field(1), "2.5");
    Math::Vec3 radiiNow = property(lamp, "collision_radii").asVec3();
    CHECK(near(radiiNow.x, radiiBefore.x) && near(radiiNow.y, 2.5f) &&
            near(radiiNow.z, radiiBefore.z));
    typeInto(editor, radii->field(0), "-3");
    radiiNow = property(lamp, "collision_radii").asVec3();
    CHECK(near(radiiNow.x, 0.0f) && near(radiiNow.y, 2.5f));
    typeInto(editor, radii->field(2), "1500");
    radiiNow = property(lamp, "collision_radii").asVec3();
    CHECK(near(radiiNow.z, 1500.0f));
    CHECK(radii->revertible());
    clickWidget(editor, radii->revertButton());
    radiiNow = property(lamp, "collision_radii").asVec3();
    CHECK(near(radiiNow.x, radiiBefore.x) && near(radiiNow.y, radiiBefore.y) &&
            near(radiiNow.z, radiiBefore.z));
    CHECK(!radii->revertible());

    PropertyRow* kind = inspector.rowNamed("collision_type");
    typeInto(editor, kind->field(0), "2.6");
    CHECK(property(lamp, "collision_type").asInt() == 3);
    CHECK(near(kind->field(0)->value(), 3.0f));
    CHECK(near(kind->field(0)->dragSensitivity(), 0.5f));
    clickWidget(editor, kind->revertButton());
    CHECK(property(lamp, "collision_type").asInt() == 0);
    frames(editor, 3);
    gate = inspector.gate("collision_enabled");
    CHECK(gate && !gate->isChecked());
    CHECK(inspector.rowNamed("collision_shape") == nullptr);
    CHECK(inspector.rowNamed("collision_radii") == nullptr);

    NodeClassDB::setProperty(lamp, "collision_shape", PropertyValue::fromInt(0));
    frames(editor, 3);
    CHECK(inspector.rowNamed("collision_radii") == nullptr);

    cleanScene(editor);
}

void testColor(Editor& editor, Node* lamp)
{
    PropertyInspector& inspector = editor.inspector.properties();
    select(editor, lamp);
    cleanScene(editor);

    PropertyRow* color = inspector.rowNamed("light_color");
    CHECK(color && color->swatch());
    if (!color || !color->swatch()) return;
    ColorSwatch* swatch = color->swatch();
    CHECK(!swatch->alphaEnabled());
    const Math::Vec4 before = property(lamp, "light_color").asColor();
    CHECK(near(swatch->color().x, before.x) && near(swatch->color().y, before.y));

    clickWidget(editor, swatch);
    CHECK(swatch->isOpen());
    CHECK(swatch->picker() != nullptr);
    CHECK(color->isActive());
    if (swatch->picker())
    {
        swatch->picker()->colorChanged.emit(rt::Color(255, 0, 0, 255));
        frames(editor, 2);
        const Math::Vec4 red = property(lamp, "light_color").asColor();
        CHECK(near(red.x, 1.0f) && near(red.y, 0.0f) && near(red.z, 0.0f));
        CHECK(near(red.w, before.w));
        CHECK(near(swatch->color().x, 1.0f) && near(swatch->color().y, 0.0f));
        CHECK(editor.document.dirty());
        CHECK(color->revertible() == (before.x != 1.0f || before.y != 0.0f || before.z != 0.0f));

        rt::ColorPicker* picker = swatch->picker();
        picker->setColor(rt::Color(255, 0, 0, 255));
        const rt::Rect box = picker->absoluteRect();
        const float radius = fmaxf(20.0f, fminf(box.w - 30.0f, box.h - 44.0f) * 0.5f);
        const float barX = box.x + radius + 4.0f + radius + 10.0f + 9.0f;
        const float barTop = box.y + 4.0f;
        click(editor, static_cast<int>(barX), static_cast<int>(barTop + radius * 0.2f));
        const Math::Vec4 dimmer = property(lamp, "light_color").asColor();
        CHECK(dimmer.x > 0.6f && dimmer.x < 0.99f);
        CHECK(dimmer.y < 0.02f && dimmer.z < 0.02f);
        CHECK(near(dimmer.w, before.w));
    }
    clickWidget(editor, swatch);
    CHECK(!swatch->isOpen());
    CHECK(swatch->picker() == nullptr);
    CHECK(!color->isActive());

    clickWidget(editor, color->revertButton());
    const Math::Vec4 restored = property(lamp, "light_color").asColor();
    CHECK(near(restored.x, before.x) && near(restored.y, before.y) && near(restored.z, before.z));
    CHECK(!color->revertible());
    cleanScene(editor);
}

void testExternalChanges(Editor& editor, Node* lamp)
{
    PropertyInspector& inspector = editor.inspector.properties();
    NodeClassDB::setProperty(lamp, "shadow_enabled", PropertyValue::fromBool(true));
    select(editor, lamp);
    cleanScene(editor);

    PropertyRow* bias = inspector.rowNamed("shadow_bias");
    PropertyRow* energy = inspector.rowNamed("light_energy");
    clickWidget(editor, bias->field(0));
    CHECK(bias->field(0)->isEditing());
    typeText(editor, "0.5");
    NodeClassDB::setProperty(lamp, "light_energy", PropertyValue::fromFloat(7.0));
    NodeClassDB::setProperty(lamp, "shadow_enabled", PropertyValue::fromBool(true));
    frames(editor, 4);
    CHECK(near(energy->field(0)->value(), 7.0f));
    CHECK(inspector.gate("shadow_enabled")->isChecked());
    CHECK(energy->revertible());
    CHECK(bias->field(0)->isEditing());
    CHECK(bias->field(0)->editor()->text() == "0.5");
    CHECK(!editor.document.dirty());
    pressKey(editor, Key::Enter);
    CHECK(near(static_cast<float>(property(lamp, "shadow_bias").asFloat()), 0.5f));
    CHECK(editor.document.dirty());

    clickWidget(editor, bias->field(0));
    typeText(editor, "9");
    pressKey(editor, Key::Escape);
    CHECK(!bias->field(0)->isEditing());
    CHECK(near(static_cast<float>(property(lamp, "shadow_bias").asFloat()), 0.5f));

    NodeClassDB::setProperty(lamp, "light_energy", PropertyValue::fromFloat(1.0));
    NodeClassDB::setProperty(lamp, "shadow_enabled", PropertyValue::fromBool(false));
    frames(editor, 3);
    energy = inspector.rowNamed("light_energy");
    CHECK(energy && near(energy->field(0)->value(), 1.0f));
    CHECK(!inspector.gate("shadow_enabled")->isChecked());
    CHECK(inspector.rowNamed("shadow_bias") == nullptr);
    NodeClassDB::setProperty(lamp, "shadow_enabled", PropertyValue::fromBool(true));
    frames(editor, 3);
    bias = inspector.rowNamed("shadow_bias");
    CHECK(bias != nullptr);
    if (bias) clickWidget(editor, bias->revertButton());
    cleanScene(editor);
}

void clickCell(Editor& editor, LayerGrid* grid, int index)
{
    reveal(editor, grid);
    const rt::Rect cell = grid->cellRect(index);
    click(editor, static_cast<int>(cell.x + cell.w * 0.5f),
            static_cast<int>(cell.y + cell.h * 0.5f));
}

void testLayers(Editor& editor)
{
    PropertyInspector& inspector = editor.inspector.properties();
    Node* camera = makeNode(editor, "Camera3D", "Cam");
    select(editor, camera);
    cleanScene(editor);
    PropertyRow* mask = inspector.rowNamed("cull_mask");
    CHECK(mask && mask->kind() == PropertyRow::Kind::Layers);
    LayerGrid* grid = mask ? mask->layerGrid() : nullptr;
    CHECK(grid != nullptr);
    rt::Collapsible* maskSection = inspector.section("Cull Mask");
    CHECK(maskSection != nullptr && !maskSection->isExpanded() && mask &&
            mask->parent() == maskSection);
    if (maskSection)
    {
        reveal(editor, maskSection);
        const rt::Rect header = maskSection->absoluteRect();
        click(editor, static_cast<int>(header.x + header.w * 0.5f),
                static_cast<int>(header.y + 12.0f));
        CHECK(maskSection->isExpanded());
        frames(editor, 3);
    }
    if (grid)
    {
        reveal(editor, mask);
        CHECK(grid->count() == PropertyRow::kRenderLayers &&
                grid->groupSize() == PropertyRow::kRenderGroupSize);
        grid->setExpanded(true);
        frames(editor, 3);
        CHECK(grid->visibleCount() == grid->count());
        reveal(editor, mask);
        const std::int64_t before = property(camera, "cull_mask").asInt();
        CHECK(grid->bits() == before);
        CHECK(grid->isChecked(2) == ((before & 4) != 0));
        clickCell(editor, grid, 2);
        CHECK(property(camera, "cull_mask").asInt() == (before ^ 4));
        CHECK(grid->isChecked(2) == (((before ^ 4) & 4) != 0));
        CHECK(editor.document.dirty());
        clickCell(editor, grid, 19);
        CHECK(property(camera, "cull_mask").asInt() == (before ^ 4 ^ (1 << 19)));
        CHECK(grid->bits() == property(camera, "cull_mask").asInt());
        CHECK(grid->cellAt(-100.0f, -100.0f) == -1);
        clickWidget(editor, mask->revertButton());
        CHECK(property(camera, "cull_mask").asInt() == before);
        CHECK(grid->bits() == before);
    }

    Node* floor = makeNode(editor, "StaticBody3D", "Floor");
    select(editor, floor);
    PropertyRow* layer = inspector.rowNamed("collision_layer");
    CHECK(layer && layer->kind() == PropertyRow::Kind::Layers);
    LayerGrid* physics = layer ? layer->layerGrid() : nullptr;
    CHECK(physics != nullptr);
    if (physics)
    {
        CHECK(physics->count() == PropertyRow::kPhysicsLayers &&
                physics->groupSize() == PropertyRow::kPhysicsGroupSize);
        physics->setExpanded(true);
        frames(editor, 3);
        CHECK(physics->visibleCount() == physics->count());
        const std::int64_t before = property(floor, "collision_layer").asInt();
        clickCell(editor, physics, 31);
        CHECK(property(floor, "collision_layer").asInt() ==
                (before ^ (static_cast<std::int64_t>(1) << 31)));
        clickCell(editor, physics, 0);
        CHECK(property(floor, "collision_layer").asInt() ==
                (before ^ (static_cast<std::int64_t>(1) << 31) ^ 1));
    }
    cleanScene(editor);
}

void testTextAndResources(Editor& editor)
{
    PropertyInspector& inspector = editor.inspector.properties();
    EditorDocument& document = editor.document;
    const ct::String root = document.project().root();
    writeImage(root + "/Assets/Textures/particle.png", 255, 255, 255);
    writeImage(root + "/Assets/Textures/other.png", 20, 140, 220);

    Node* player = makeNode(editor, "AnimationPlayer", "Player");
    select(editor, player);
    cleanScene(editor);
    PropertyRow* autoplay = inspector.rowNamed("autoplay");
    CHECK(autoplay && autoplay->kind() == PropertyRow::Kind::String && autoplay->text());
    if (autoplay && autoplay->text())
    {
        typeInto(editor, autoplay->text(), "walk", false);
        CHECK(property(player, "autoplay").asString() == "walk");
        CHECK(document.dirty());
        CHECK(autoplay->isActive());
        NodeClassDB::setProperty(player, "autoplay", PropertyValue::fromString("walking"));
        frames(editor, 3);
        CHECK(autoplay->text()->text() == "walk");
        select(editor, nullptr);
        select(editor, player);
        autoplay = inspector.rowNamed("autoplay");
        CHECK(autoplay && autoplay->text() && autoplay->text()->text() == "walking");
        CHECK(autoplay && autoplay->revertible());
        if (!autoplay) return;
        clickWidget(editor, autoplay->revertButton());
        CHECK(property(player, "autoplay").asString().empty());
        CHECK(autoplay->text()->text().empty());
    }
    cleanScene(editor);

    Node* body = makeNode(editor, "MeshInstance3D", "Body");
    select(editor, body);
    CHECK(inspector.rowNamed("skeleton") == nullptr);
    Node* hair = makeNode(editor, "Hair3D", "Hair");
    select(editor, hair);
    cleanScene(editor);
    PropertyRow* skeleton = inspector.rowNamed("skeleton");
    CHECK(skeleton && skeleton->kind() == PropertyRow::Kind::NodePath && skeleton->text());
    if (skeleton && skeleton->text())
    {
        typeInto(editor, skeleton->text(), "../Skel", false);
        CHECK(property(hair, "skeleton").asString() == "..");
        pressKey(editor, Key::Enter);
        CHECK(property(hair, "skeleton").type() == PropertyType::NodePath);
        CHECK(property(hair, "skeleton").asString() == "../Skel");
        CHECK(document.dirty());
        CHECK(skeleton->revertible());
        clickWidget(editor, skeleton->revertButton());
        CHECK(property(hair, "skeleton").asString() == "..");
        CHECK(skeleton->text()->text() == "..");
    }
    cleanScene(editor);
    select(editor, body);

    PropertyRow* mesh = inspector.rowNamed("mesh");
    CHECK(mesh && mesh->kind() == PropertyRow::Kind::Mesh && mesh->meshEditor());
    if (mesh && mesh->meshEditor())
    {
        MeshPropertyEditor* editorWidget = mesh->meshEditor();
        reveal(editor, editorWidget);
        CHECK(editorWidget->mode() == MeshPropertyEditor::Mode::None);
        CHECK(editorWidget->summaryLabel()->text() == MeshPropertyEditor::kNoMeshText);
        NodeClassDB::setProperty(body, "mesh",
                PropertyValue::fromResourcePath("primitive:box?size=1,2,3"));
        frames(editor, 3);
        CHECK(editorWidget->mode() == MeshPropertyEditor::Mode::Primitive);
        CHECK(editorWidget->summaryLabel()->text() == "Box 1 \xC3\x97 2 \xC3\x97 3");
        CHECK(mesh->revertible());
        clickWidget(editor, mesh->revertButton());
        CHECK(property(body, "mesh").asString().empty());
        CHECK(editorWidget->mode() == MeshPropertyEditor::Mode::None);
        CHECK(!mesh->revertible());
    }
    cleanScene(editor);

    Node* smoke = makeNode(editor, "ParticleSystem3D", "Smoke");
    select(editor, smoke);
    cleanScene(editor);
    PropertyRow* sizeStart = inspector.rowNamed("particle_size_start");
    CHECK(sizeStart && sizeStart->kind() == PropertyRow::Kind::Vec2 &&
            sizeStart->componentCount() == 2);
    if (sizeStart && sizeStart->componentCount() == 2)
    {
        const Math::Vec2 before = property(smoke, "particle_size_start").asVec2();
        typeInto(editor, sizeStart->field(1), "3.25");
        const Math::Vec2 now = property(smoke, "particle_size_start").asVec2();
        CHECK(near(now.x, before.x) && near(now.y, 3.25f));
        CHECK(near(sizeStart->field(1)->value(), 3.25f));
        CHECK(strcmp(sizeStart->field(0)->tag().c_str(), "X") == 0 &&
                strcmp(sizeStart->field(1)->tag().c_str(), "Y") == 0);
    }

    PropertyRow* texture = inspector.rowNamed("texture");
    CHECK(texture && texture->kind() == PropertyRow::Kind::ResourcePath && texture->text() &&
            texture->browseButton());
    if (texture && texture->text() && texture->browseButton())
    {
        typeInto(editor, texture->text(), "Assets/Textures/particle.png", true);
        CHECK(property(smoke, "texture").asString() == "Assets/Textures/particle.png");

        const std::size_t floats = floatCount();
        clickWidget(editor, texture->browseButton());
        CHECK(floatCount() == floats + 1);
        rt::FileDialog* dialog = editor.ui.fileDialog();
        CHECK(dialog != nullptr);
        if (dialog)
        {
            CHECK(dialog->filter() == "*.png;*.jpg;*.jpeg;*.tga;*.bmp;*.dds;*.hdr");
            CHECK(dialog->mode() == rt::FileDialog::Mode::Open);
            dialog->accepted.emit(ct::String("/nonexistent/elsewhere/tex.png"));
            frames(editor, 2);
            CHECK(property(smoke, "texture").asString() == "Assets/Textures/particle.png");
            dialog->accepted.emit(root + "/Assets/Textures/other.png");
            frames(editor, 2);
            CHECK(property(smoke, "texture").asString() == "Assets/Textures/other.png");
            CHECK(texture->text()->text() == "Assets/Textures/other.png");
            CHECK(document.dirty());
            rt::WidgetApp::instance().removeFloat(dialog);
            frames(editor, 2);
            CHECK(floatCount() == floats);
        }

        clickWidget(editor, texture->browseButton());
        rt::FileDialog* second = editor.ui.fileDialog();
        select(editor, body);
        if (second)
        {
            second->accepted.emit(root + "/Assets/Textures/particle.png");
            frames(editor, 2);
            CHECK(property(smoke, "texture").asString() == "Assets/Textures/other.png");
            rt::WidgetApp::instance().removeFloat(second);
            frames(editor, 2);
        }
    }
    cleanScene(editor);
}

void testSoftBody(Editor& editor)
{
    PropertyInspector& inspector = editor.inspector.properties();
    Node* soft = makeNode(editor, "SoftBody3D", "Soft");
    if (!soft) return;
    select(editor, soft);
    cleanScene(editor);
    PropertyRow* steps = inspector.rowNamed("substeps");
    CHECK(steps && steps->kind() == PropertyRow::Kind::Int && steps->field(0)->ranged());
    if (!steps || !steps->field(0)->ranged()) return;
    CHECK(near(static_cast<float>(steps->field(0)->rangeMin()), 1.0f) &&
            near(static_cast<float>(steps->field(0)->rangeMax()), 64.0f));
    typeInto(editor, steps->field(0), "0");
    CHECK(property(soft, "substeps").asInt() == 1);
    CHECK(near(steps->field(0)->value(), 1.0f));
    typeInto(editor, steps->field(0), "100");
    CHECK(property(soft, "substeps").asInt() == 100);
    CHECK(near(steps->field(0)->ratio(), 1.0f));
    typeInto(editor, steps->field(0), "7.6");
    CHECK(property(soft, "substeps").asInt() == 8);
    CHECK(near(steps->field(0)->value(), 8.0f));
    dragNumber(editor, steps->field(0), 20, 4);
    CHECK(property(soft, "substeps").asInt() == 18);
    dragGrabber(editor, steps->field(0), 0.5f);
    const std::int64_t middle = property(soft, "substeps").asInt();
    CHECK(middle > 20 && middle < 45);
    clickWidget(editor, steps->revertButton());
    PropertyValue defaultSteps;
    CHECK(NodeClassDB::getDefaultPropertyValue("SoftBody3D", "substeps", defaultSteps));
    CHECK(property(soft, "substeps").asInt() == defaultSteps.asInt());
    cleanScene(editor);
}

void testInspectorTypes(Editor& editor)
{
    Node* lamp = childNamed(editor.document.scene(), "Lamp");
    CHECK(lamp != nullptr);
    if (!lamp) return;
    testStructure(editor, lamp);
    testScalars(editor, lamp);
    testColor(editor, lamp);
    testExternalChanges(editor, lamp);
    testLayers(editor);
    testTextAndResources(editor);
    testSoftBody(editor);
}

} // namespace

namespace
{

struct Capture
{
    PropertyValue last;
    int count = 0;
    int reverts = 0;
    int browses = 0;
};

PropertyRow* addProbe(Editor& editor, const PropertyInfo& info, const char* label, Capture& capture)
{
    rt::BoxLayout* root = editor.inspector.properties().root();
    PropertyRow* row = root->createChild<PropertyRow>(info, ct::String(label));
    row->edited.connect(
            [&capture](const PropertyValue& value)
            {
                capture.last = value;
                ++capture.count;
            });
    row->revertRequested.connect([&capture] { ++capture.reverts; });
    row->browseRequested.connect([&capture] { ++capture.browses; });
    rt::WidgetApp::instance().requestLayout();
    frames(editor, 3);
    return row;
}

void removeProbe(Editor& editor, PropertyRow* row)
{
    editor.inspector.properties().root()->removeChild(row);
    rt::WidgetApp::instance().requestLayout();
    frames(editor, 2);
}

void testNames()
{
    CHECK(capitalizePropertyName("shadow_normal_bias") == "Shadow Normal Bias");
    CHECK(capitalizePropertyName("energy") == "Energy");
    CHECK(capitalizePropertyName("size_of_the_box") == "Size of the Box");
    CHECK(capitalizePropertyName("aabb_min") == "AABB Min");
    CHECK(capitalizePropertyName("msaa_3d") == "MSAA 3D");
    CHECK(capitalizePropertyName("uv_tiles") == "UV Tiles");
    CHECK(capitalizePropertyName("fovY") == "Fov Y");
    CHECK(capitalizePropertyName("") == "");
    CHECK(capitalizePropertyName("__a__") == "A");
    CHECK(capitalizePropertyName("top_2d_view") == "Top 2D View");
    CHECK(capitalizePropertyName("the_end") == "The End");

    CHECK(PropertyRow::fileFilter(PropertyInfo::file("f", "*.png,jpg, .tga")) ==
            "*.png;*.jpg;*.tga");
    CHECK(PropertyRow::fileFilter(PropertyInfo::resource("r", "Material")) == "*.mat");
    CHECK(PropertyRow::fileFilter(PropertyInfo::resource("r", "Material,AudioStream")) ==
            "*.mat;*.wav;*.mp3;*.flac;*.ogg");
    CHECK(PropertyRow::fileFilter(PropertyInfo::resource("r", "Unknown")).empty());
    CHECK(PropertyRow::fileFilter(PropertyInfo(PropertyType::ResourcePath, "r")).empty());
}

void testRows(Editor& editor)
{
    testNames();
    select(editor, nullptr);
    CHECK(editor.inspector.properties().rowCount() == 0);

    {
        Capture capture;
        PropertyRow* flags = addProbe(editor,
                PropertyInfo::flags("mask", "Alpha,Beta,Gamma:8,None:0,Delta"), "Mask", capture);
        CHECK(flags->kind() == PropertyRow::Kind::Flags && flags->boxCount() == 4);
        if (flags->boxCount() == 4)
        {
            flags->setValue(PropertyValue::fromInt(10));
            CHECK(!flags->box(0)->isChecked() && flags->box(1)->isChecked() &&
                    flags->box(2)->isChecked() && !flags->box(3)->isChecked());
            CHECK(capture.count == 0);
            clickWidget(editor, flags->box(0));
            CHECK(capture.count == 1 && capture.last.asInt() == 11);
            clickWidget(editor, flags->box(1));
            CHECK(capture.count == 2 && capture.last.asInt() == 9);
            clickWidget(editor, flags->box(3));
            CHECK(capture.count == 3 && capture.last.asInt() == 25);
            CHECK(flags->box(3)->isChecked());
        }
        removeProbe(editor, flags);
    }

    {
        Capture capture;
        PropertyRow* quat = addProbe(editor, PropertyInfo(PropertyType::Quat, "q"), "Q", capture);
        CHECK(quat->kind() == PropertyRow::Kind::Quat && quat->componentCount() == 3);
        quat->setValue(PropertyValue::fromQuat(Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f)));
        CHECK(near(quat->field(0)->value(), 0.0f) && near(quat->field(1)->value(), 0.0f));
        typeInto(editor, quat->field(1), "90");
        CHECK(capture.count == 1 && capture.last.type() == PropertyType::Quat);
        const Math::Quaternion wanted =
                Math::Quaternion::FromEulerAngles(0.0f, 90.0f * 0.017453292519943295f, 0.0f);
        CHECK(fabsf(capture.last.asQuat().Dot(wanted)) > 0.9999f);
        const Math::Quaternion other =
                Math::Quaternion::FromEulerAngles(30.0f * 0.017453292519943295f, 0.0f, 0.0f);
        quat->setValue(PropertyValue::fromQuat(other));
        CHECK(near(quat->field(0)->value(), 30.0f, 0.01f));
        CHECK(near(quat->field(1)->value(), 0.0f, 0.01f));
        CHECK(capture.count == 1);
        typeInto(editor, quat->field(2), "10");
        CHECK(capture.count == 2);
        const Math::Quaternion both = Math::Quaternion::FromEulerAngles(
                30.0f * 0.017453292519943295f, 0.0f, 10.0f * 0.017453292519943295f);
        CHECK(fabsf(capture.last.asQuat().Dot(both)) > 0.9999f);
        removeProbe(editor, quat);
    }

    {
        Capture capture;
        PropertyRow* vec = addProbe(editor, PropertyInfo(PropertyType::Vec4, "v"), "V", capture);
        CHECK(vec->kind() == PropertyRow::Kind::Vec4 && vec->componentCount() == 4);
        vec->setValue(PropertyValue::fromVec4(Math::Vec4(1.0f, 2.0f, 3.0f, 4.0f)));
        CHECK(near(vec->field(3)->value(), 4.0f) && strcmp(vec->field(3)->tag().c_str(), "W") == 0);
        typeInto(editor, vec->field(3), "-2.5");
        const Math::Vec4 value = capture.last.asVec4();
        CHECK(capture.count == 1 && near(value.x, 1.0f) && near(value.y, 2.0f) &&
                near(value.z, 3.0f) && near(value.w, -2.5f));
        CHECK(!vec->revertible());
        clickWidget(editor, vec->revertButton());
        CHECK(capture.reverts == 0);
        vec->setRevertible(true);
        clickWidget(editor, vec->revertButton());
        CHECK(capture.reverts == 1);
        removeProbe(editor, vec);
    }

    {
        Capture capture;
        PropertyRow* choice =
                addProbe(editor, PropertyInfo::enumeration("e", "Low,Mid:5,High"), "E", capture);
        CHECK(choice->kind() == PropertyRow::Kind::Enum && choice->choiceCount() == 3);
        CHECK(choice->choiceValue(0) == 0 && choice->choiceValue(1) == 5 &&
                choice->choiceValue(2) == 6);
        choice->setValue(PropertyValue::fromInt(6));
        CHECK(choice->combo()->selectedIndex() == 2);
        choice->setValue(PropertyValue::fromInt(9));
        CHECK(choice->combo()->selectedIndex() == -1);
        CHECK(capture.count == 0);
        choice->combo()->setSelectedIndex(1);
        CHECK(capture.count == 1 && capture.last.asInt() == 5);
        removeProbe(editor, choice);
    }

    {
        Capture capture;
        PropertyInfo angle =
                PropertyInfo::range(PropertyType::Float, "angle", -360.0, 360.0, 0.1, true, true);
        angle.radiansAsDegrees = true;
        PropertyRow* row = addProbe(editor, angle, "Angle", capture);
        CHECK(row->kind() == PropertyRow::Kind::Float && row->field(0)->ranged());
        row->setValue(PropertyValue::fromFloat(1.5707963267948966));
        CHECK(near(row->field(0)->value(), 90.0f, 0.01f));
        typeInto(editor, row->field(0), "180");
        CHECK(capture.count == 1 &&
                near(static_cast<float>(capture.last.asFloat()), 3.14159265f, 1.0e-4f));
        typeInto(editor, row->field(0), "720");
        CHECK(near(row->field(0)->value(), 720.0f));
        removeProbe(editor, row);
    }

    {
        Capture capture;
        PropertyInfo count = PropertyInfo::range(PropertyType::Int, "count", 2.0, 10.0, 2.0);
        PropertyRow* row = addProbe(editor, count, "Count", capture);
        CHECK(row->constrained(PropertyValue::fromInt(5)).asInt() == 6);
        CHECK(row->constrained(PropertyValue::fromInt(0)).asInt() == 2);
        CHECK(row->constrained(PropertyValue::fromInt(11)).asInt() == 10);
        CHECK(row->constrained(PropertyValue::fromInt(100)).asInt() == 10);
        typeInto(editor, row->field(0), "7");
        CHECK(capture.last.asInt() == 8);
        typeInto(editor, row->field(0), "8");
        CHECK(capture.count == 1);
        removeProbe(editor, row);
    }

    {
        Capture capture;
        PropertyRow* color = addProbe(editor, PropertyInfo::color("c"), "C", capture);
        CHECK(color->kind() == PropertyRow::Kind::Color && color->swatch()->alphaEnabled());
        color->setValue(PropertyValue::fromColor(Math::Vec4(0.2f, 0.4f, 0.6f, 1.0f)));
        clickWidget(editor, color->swatch());
        CHECK(color->swatch()->isOpen() && color->swatch()->picker() != nullptr);
        if (color->swatch()->picker())
        {
            color->swatch()->picker()->colorChanged.emit(rt::Color(10, 20, 30, 128));
            CHECK(capture.count == 1 && capture.last.type() == PropertyType::Color);
            const Math::Vec4 value = capture.last.asColor();
            CHECK(near(value.x, 10.0f / 255.0f) && near(value.y, 20.0f / 255.0f) &&
                    near(value.z, 30.0f / 255.0f) && near(value.w, 128.0f / 255.0f));
        }
        rt::WidgetApp::instance().closePopup();
        frames(editor, 2);
        removeProbe(editor, color);
    }

    {
        Capture capture;
        PropertyRow* text = addProbe(editor, PropertyInfo(PropertyType::String, "s"), "S", capture);
        CHECK(text->kind() == PropertyRow::Kind::String && text->text() != nullptr);
        text->setValue(PropertyValue::fromString("hello"));
        CHECK(text->text()->text() == "hello" && capture.count == 0);
        typeInto(editor, text->text(), "bye", false);
        CHECK(capture.count >= 1 && capture.last.asString() == "bye");
        CHECK(capture.last.type() == PropertyType::String);
        removeProbe(editor, text);
    }

    {
        Capture capture;
        PropertyRow* path = addProbe(editor, PropertyInfo::file("f", "*.png"), "F", capture);
        CHECK(path->kind() == PropertyRow::Kind::ResourcePath && path->browseButton() != nullptr);
        clickWidget(editor, path->browseButton());
        CHECK(capture.browses == 1);
        typeInto(editor, path->text(), "a/b.png", false);
        CHECK(capture.count == 0);
        pressKey(editor, Key::Enter);
        CHECK(capture.count == 1 && capture.last.asString() == "a/b.png");
        CHECK(capture.last.type() == PropertyType::ResourcePath);
        removeProbe(editor, path);
    }

    {
        Capture capture;
        PropertyRow* flag = addProbe(editor, PropertyInfo(PropertyType::Bool, "b"), "B", capture);
        flag->setValue(PropertyValue::fromBool(true));
        CHECK(flag->check()->isChecked() && capture.count == 0);
        clickWidget(editor, flag->check());
        CHECK(capture.count == 1 && !capture.last.asBool());
        removeProbe(editor, flag);
    }
}

} // namespace

namespace
{

int quitCalls = 0;

void setEnergy(Editor& editor, Node* lamp, const char* text)
{
    select(editor, lamp);
    PropertyRow* energy = editor.inspector.properties().rowNamed("light_energy");
    CHECK(energy != nullptr);
    if (energy) typeInto(editor, energy->field(0), text);
}

float energyOf(Node* scene)
{
    Node* lamp = childNamed(scene, "Lamp");
    CHECK(lamp != nullptr);
    return lamp ? static_cast<float>(property(lamp, "light_energy").asFloat()) : -1.0f;
}

void saveAsThrough(Editor& editor, const char* name)
{
    rt::InputBox* box = editor.ui.saveAsBox();
    CHECK(box != nullptr);
    if (!box) return;
    box->setText(name);
    rt::Button* save = findButton(box->content(), "Save");
    CHECK(save != nullptr);
    if (save) click(editor, centerX(save), centerY(save));
}

void testUnsaved(Editor& editor)
{
    EditorDocument& document = editor.document;
    Project& project = document.project();
    ProjectUi& ui = editor.ui;
    const ct::String demo = project.root();
    Node* lamp = childNamed(document.scene(), "Lamp");
    CHECK(lamp != nullptr);
    if (!lamp) return;
    CHECK(!document.dirty());

    PropertyValue defaultEnergy;
    CHECK(NodeClassDB::getDefaultPropertyValue("OmniLight3D", "light_energy", defaultEnergy));
    const float initialEnergy = static_cast<float>(defaultEnergy.asFloat());
    CHECK(initialEnergy != 9.0f && initialEnergy != 4.0f && initialEnergy != 6.0f);
    ui.saveAs();
    frames(editor, 2);
    rt::InputBox* box = ui.saveAsBox();
    CHECK(box != nullptr);
    if (box)
    {
        const std::size_t open = floatCount();
        ui.saveAs();
        frames(editor, 2);
        CHECK(floatCount() == open && ui.saveAsBox() == box);
        CHECK(findInput(box->content()) != nullptr);
        CHECK(findInput(box->content())->text() == "NewScene");
        rt::Button* cancel = findButton(box->content(), "Cancel");
        CHECK(cancel != nullptr);
        if (cancel) click(editor, centerX(cancel), centerY(cancel));
        frames(editor, 2);
        CHECK(ui.saveAsBox() == nullptr);
        CHECK(document.scenePath() == "Scenes/main.scene.json");
    }
    ct::String reason;
    CHECK(document.validSceneName("main", reason));
    CHECK(!document.validSceneName("", reason));
    CHECK(!document.validSceneName("a/b", reason));
    CHECK(!document.validSceneName(".hidden", reason));
    CHECK(document.validSceneName("fine-name_2", reason));

    setEnergy(editor, lamp, "9");
    CHECK(document.dirty());
    ui.saveAs();
    frames(editor, 2);
    saveAsThrough(editor, "second");
    frames(editor, 2);
    CHECK(ui.saveAsBox() == nullptr);
    CHECK(isFile(demo + "/Scenes/second.scene.json"));
    CHECK(document.scenePath() == "Scenes/second.scene.json");
    CHECK(!document.dirty());
    CHECK(project.scenes().size() == 2 && project.hasScene("Scenes/second.scene.json"));
    CHECK(contains(readFile(demo + "/projecto.json"), "Scenes/second.scene.json"));
    CHECK(project.mainScene() == "Scenes/main.scene.json");
    CHECK(!document.isMainScene() && document.sceneInProject());
    CHECK(ui.windowTitle() == "Demo - second.scene.json - " SIGNATURE);
    CHECK(editor.shell.toolBar()->isItemEnabled(editor.ui.buttonId(ProjectUi::kMainScene)));
    CHECK(!document.validSceneName("main", reason) && contains(reason, "already exists"));
    CHECK(!document.saveSceneAs("main"));
    CHECK(document.scenePath() == "Scenes/second.scene.json");
    ui.saveAs();
    frames(editor, 2);
    saveAsThrough(editor, "main");
    frames(editor, 2);
    rt::InputBox* refused = ui.saveAsBox();
    CHECK(refused != nullptr);
    CHECK(project.scenes().size() == 2 && document.scenePath() == "Scenes/second.scene.json");
    if (refused)
    {
        rt::Button* cancel = findButton(refused->content(), "Cancel");
        if (cancel) click(editor, centerX(cancel), centerY(cancel));
        frames(editor, 2);
    }
    CHECK(ui.saveAsBox() == nullptr);

    ui.markMainScene();
    CHECK(project.mainScene() == "Scenes/second.scene.json" && document.isMainScene());
    CHECK(contains(readFile(demo + "/projecto.json"),
            "\"activeScene\": \"Scenes/second.scene.json\""));
    frames(editor, 2);
    CHECK(!editor.shell.toolBar()->isItemEnabled(editor.ui.buttonId(ProjectUi::kMainScene)));

    ui.openSceneNamed("Scenes/main.scene.json");
    frames(editor, 2);
    CHECK(!ui.confirming());
    CHECK(document.scenePath() == "Scenes/main.scene.json" && !document.isMainScene());
    CHECK(near(energyOf(document.scene()), initialEnergy));
    CHECK(editor.tree.getRoot()->getChildCount() == 1);
    CHECK(editor.hierarchy.editedScene() == document.scene());
    CHECK(editor.hierarchy.outliner() != nullptr);

    lamp = childNamed(document.scene(), "Lamp");
    setEnergy(editor, lamp, "4");
    CHECK(document.dirty());
    CHECK(ui.windowTitle() == "Demo - main.scene.json* - " SIGNATURE);
    Node* sceneBefore = document.scene();
    unsigned int serial = document.sceneSerial();

    ui.openSceneNamed("Scenes/second.scene.json");
    frames(editor, 2);
    CHECK(ui.confirming());
    rt::Dialog* dialog = ui.confirmDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->buttons_.size() == 3);
        CHECK(dialog->buttons_[kConfirmSave].label == "Save" &&
                dialog->buttons_[kConfirmDiscard].label == "Discard" &&
                dialog->buttons_[kConfirmCancel].label == "Cancel");
        CHECK(contains(dialog->message_, "main.scene.json"));
    }
    CHECK(document.scene() == sceneBefore && document.sceneSerial() == serial);
    clickDialogButton(editor, dialog, kConfirmCancel);
    frames(editor, 2);
    CHECK(!ui.confirming());
    CHECK(document.scene() == sceneBefore && document.sceneSerial() == serial);
    CHECK(document.scenePath() == "Scenes/main.scene.json" && document.dirty());
    CHECK(near(energyOf(document.scene()), 4.0f));
    CHECK(!contains(readFile(demo + "/Scenes/main.scene.json"), "\"light_energy\": 4"));

    ui.openSceneNamed("Scenes/second.scene.json");
    frames(editor, 2);
    CHECK(ui.confirming());
    dialog = ui.confirmDialog();
    ui.openSceneNamed("Scenes/second.scene.json");
    CHECK(ui.confirmDialog() == dialog);
    clickDialogButton(editor, dialog, kConfirmDiscard);
    frames(editor, 3);
    CHECK(!ui.confirming());
    CHECK(document.scenePath() == "Scenes/second.scene.json" && !document.dirty());
    CHECK(document.sceneSerial() == serial + 1 && document.scene() != sceneBefore);
    CHECK(near(energyOf(document.scene()), 9.0f));
    CHECK(editor.tree.getRoot()->getChildCount() == 1);
    CHECK(editor.hierarchy.editedScene() == document.scene());
    CHECK(editor.inspector.inspected() == nullptr);

    lamp = childNamed(document.scene(), "Lamp");
    setEnergy(editor, lamp, "6");
    CHECK(document.dirty());
    serial = document.sceneSerial();
    ui.openSceneNamed("Scenes/main.scene.json");
    frames(editor, 2);
    clickDialogButton(editor, ui.confirmDialog(), kConfirmSave);
    frames(editor, 3);
    CHECK(!ui.confirming());
    CHECK(document.scenePath() == "Scenes/main.scene.json" && !document.dirty());
    CHECK(document.sceneSerial() == serial + 1);
    CHECK(contains(readFile(demo + "/Scenes/second.scene.json"), "\"light_energy\": 6"));
    CHECK(document.openScene("Scenes/second.scene.json"));
    CHECK(near(energyOf(document.scene()), 6.0f));
    CHECK(document.openScene(demo + "/Scenes/main.scene.json"));
    CHECK(document.scenePath() == "Scenes/main.scene.json");

    ui.newScene();
    frames(editor, 2);
    CHECK(!ui.confirming() && !document.hasScenePath());
    CHECK(document.scene()->getName() == "Scene" && !document.dirty());
    CHECK(document.sceneName() == "Untitled");
    Node* orphan = makeNode(editor, "Node3D", "Orphan");
    CHECK(orphan != nullptr && document.dirty());
    CHECK(ui.windowTitle() == "Demo - Untitled* - " SIGNATURE);
    ui.openSceneNamed("Scenes/second.scene.json");
    frames(editor, 2);
    CHECK(ui.confirming());
    clickDialogButton(editor, ui.confirmDialog(), kConfirmSave);
    frames(editor, 2);
    CHECK(!ui.confirming());
    CHECK(ui.saveAsBox() != nullptr);
    CHECK(document.scenePath().empty() && document.dirty());
    {
        rt::InputBox* named = ui.saveAsBox();
        rt::Button* cancel = named ? findButton(named->content(), "Cancel") : nullptr;
        CHECK(cancel != nullptr);
        if (cancel) click(editor, centerX(cancel), centerY(cancel));
        frames(editor, 2);
    }
    CHECK(ui.saveAsBox() == nullptr && document.scenePath().empty() && document.dirty());
    CHECK(childNamed(document.scene(), "Orphan") != nullptr);

    ui.openSceneNamed("Scenes/second.scene.json");
    frames(editor, 2);
    clickDialogButton(editor, ui.confirmDialog(), kConfirmSave);
    frames(editor, 2);
    saveAsThrough(editor, "untitled1");
    frames(editor, 3);
    CHECK(ui.saveAsBox() == nullptr);
    CHECK(isFile(demo + "/Scenes/untitled1.scene.json"));
    CHECK(contains(readFile(demo + "/Scenes/untitled1.scene.json"), "Orphan"));
    CHECK(project.hasScene("Scenes/untitled1.scene.json"));
    CHECK(document.scenePath() == "Scenes/second.scene.json" && !document.dirty());

    ui.newScene();
    frames(editor, 2);
    makeNode(editor, "Node3D", "Temp");
    CHECK(document.dirty());
    ui.newScene();
    frames(editor, 2);
    clickDialogButton(editor, ui.confirmDialog(), kConfirmDiscard);
    frames(editor, 3);
    CHECK(!document.dirty() && document.scene()->getName() == "Scene");
    CHECK(childNamed(document.scene(), "Temp") == nullptr);
    CHECK(editor.tree.getRoot()->getChildCount() == 1);

    ui.openSceneFile();
    frames(editor, 2);
    rt::FileDialog* chooser = ui.fileDialog();
    CHECK(chooser != nullptr);
    if (chooser)
    {
        CHECK(chooser->filter() == "*.json");
        chooser->accepted.emit(demo + "/Scenes/main.scene.json");
        rt::WidgetApp::instance().removeFloat(chooser);
        frames(editor, 3);
        CHECK(document.scenePath() == "Scenes/main.scene.json");
    }

    const ct::String outside = testRoot() + "/outside.scene.json";
    CHECK(FileSystem::writeText(outside, readFile(demo + "/Scenes/main.scene.json")));
    const std::size_t listed = project.scenes().size();
    CHECK(document.openScene(outside));
    CHECK(document.scenePath() == outside && !document.sceneInProject());
    CHECK(project.scenes().size() == listed);
    CHECK(document.sceneName() == "outside.scene.json");
    CHECK(!document.setMainScene());
    makeNode(editor, "Node3D", "Outer");
    CHECK(document.dirty());
    CHECK(document.saveScene());
    CHECK(contains(readFile(outside), "Outer"));
    CHECK(!document.dirty());
    ui.save();
    CHECK(document.scenePath() == outside);
    CHECK(document.openScene("Scenes/main.scene.json"));

    Node* current = document.scene();
    const unsigned int kept = document.sceneSerial();
    CHECK(FileSystem::writeText(demo + "/Scenes/broken.scene.json", "{ not json"));
    CHECK(!document.openScene("Scenes/broken.scene.json"));
    CHECK(!document.error().empty());
    CHECK(document.scene() == current && document.sceneSerial() == kept);
    CHECK(current->getName() == "Main");
    CHECK(!document.openScene("Scenes/missing.scene.json"));
    CHECK(contains(document.error(), "does not exist"));
    CHECK(!document.openScene(""));
    CHECK(!project.hasScene("Scenes/broken.scene.json"));
    frames(editor, 3);
    CHECK(editor.tree.getRoot()->getChildCount() == 1);
    CHECK(!document.dirty());

    lamp = childNamed(document.scene(), "Lamp");
    setEnergy(editor, lamp, "3");
    CHECK(document.dirty());
    CHECK(ui.handleEvent(keyEvent(Key::S, true, kModControl)));
    frames(editor, 2);
    CHECK(!document.dirty());
    CHECK(contains(readFile(demo + "/Scenes/main.scene.json"), "\"light_energy\": 3"));
    CHECK(!ui.handleEvent(keyEvent(Key::S, true)));
    CHECK(!ui.handleEvent(keyEvent(Key::S, false, kModControl)));
    CHECK(!ui.handleEvent(keyEvent(Key::A, true, kModControl)));
    CHECK(ui.handleEvent(keyEvent(Key::S, true, kModControl | kModShift)));
    frames(editor, 2);
    CHECK(ui.saveAsBox() != nullptr);
    if (ui.saveAsBox())
    {
        rt::Button* cancel = findButton(ui.saveAsBox()->content(), "Cancel");
        if (cancel) click(editor, centerX(cancel), centerY(cancel));
        frames(editor, 2);
    }
    CHECK(ui.handleEvent(keyEvent(Key::N, true, kModControl)));
    frames(editor, 2);
    CHECK(!document.hasScenePath());
    CHECK(ui.handleEvent(keyEvent(Key::O, true, kModControl)));
    CHECK(ui.sceneMenu()->actionCount() >= 3);
    CHECK(document.openScene("Scenes/main.scene.json"));

    ui.rebuildSceneMenu();
    rt::Menu* menu = ui.sceneMenu();
    CHECK(menu->actionCount() == static_cast<int>(project.scenes().size()) + 2);
    CHECK(menu->action(0)->text() == "Scenes/main.scene.json");
    CHECK(menu->action(0)->isChecked());
    CHECK(contains(menu->action(1)->text(), "second.scene.json") &&
            contains(menu->action(1)->text(), "(main)"));
    CHECK(!menu->action(1)->isChecked());
    menu->action(1)->triggered.emit();
    frames(editor, 2);
    CHECK(document.scenePath() == "Scenes/second.scene.json");

    quitCalls = 0;
    ui.setQuitHandler([] { ++quitCalls; });
    ui.requestQuit();
    CHECK(quitCalls == 1);
    setEnergy(editor, childNamed(document.scene(), "Lamp"), "2");
    CHECK(document.dirty());
    ui.requestQuit();
    frames(editor, 2);
    CHECK(ui.confirming() && quitCalls == 1);
    clickDialogButton(editor, ui.confirmDialog(), kConfirmCancel);
    CHECK(quitCalls == 1 && document.dirty());
    ui.requestQuit();
    frames(editor, 2);
    clickDialogButton(editor, ui.confirmDialog(), kConfirmDiscard);
    CHECK(quitCalls == 2);
    ui.requestQuit();
    frames(editor, 2);
    clickDialogButton(editor, ui.confirmDialog(), kConfirmSave);
    CHECK(quitCalls == 3);
    CHECK(!document.dirty());
    CHECK(contains(readFile(demo + "/Scenes/second.scene.json"), "\"light_energy\": 2"));
    CHECK(document.openScene("Scenes/main.scene.json"));
}

} // namespace

namespace
{

void testRecents(Editor& editor)
{
    EditorDocument& document = editor.document;
    Project& project = document.project();
    ProjectUi& ui = editor.ui;
    EditorSettings& settings = document.settings();
    rt::ToolBar* bar = editor.shell.toolBar();
    const ct::String parent = testRoot() + "/projects";
    const ct::String demoManifest = projectManifest("Demo");
    const ct::String otherManifest = projectManifest("Other");
    CHECK(!document.dirty());
    CHECK(ProjectUi::defaultParentFolder(settings) == parent);

    CHECK(document.createProject(parent, "Other"));
    CHECK(project.isOpen() && project.name() == "Other");
    CHECK(settings.recentProjectPaths.size() == 2);
    CHECK(settings.recentProjectPaths[0] == otherManifest);
    CHECK(settings.recentProjectPaths[1] == demoManifest);
    CHECK(settings.lastProjectPath == otherManifest);
    CHECK(document.scenePath() == "Scenes/main.scene.json" && document.isMainScene());
    CHECK(!document.createProject(parent, "Other"));
    CHECK(contains(document.error(), "already exists"));
    CHECK(!document.createProject(parent, "bad/name"));
    CHECK(!document.createProject("", "Name"));
    CHECK(project.name() == "Other" && document.scenePath() == "Scenes/main.scene.json");
    frames(editor, 2);
    CHECK(editor.tree.getRoot()->getChildCount() == 1);

    EditorSettings reloaded;
    CHECK(reloaded.load(document.settingsPath()));
    CHECK(reloaded.recentProjectPaths.size() == 2 && reloaded.lastProjectPath == otherManifest);
    CHECK(reloaded.recentProjectPaths[0] == otherManifest);

    ui.update();
    CHECK(bar->isItemEnabled(ui.buttonId(ProjectUi::kRecentProjects)));
    ui.rebuildRecentMenu();
    rt::Menu* menu = ui.recentMenu();
    CHECK(menu != nullptr && menu->actionCount() == 6);
    if (menu && menu->actionCount() == 6)
    {
        CHECK(menu->action(0)->text() == "Other" && menu->action(1)->text() == "Demo");
        CHECK(menu->action(2)->isSeparator());
        CHECK(menu->action(3)->text() == "Clear recent projects");
        CHECK(menu->action(4)->isSeparator());
        CHECK(menu->action(5)->text() == "Close project");
        menu->action(1)->triggered.emit();
        frames(editor, 3);
        CHECK(project.name() == "Demo");
        CHECK(document.scenePath() == "Scenes/second.scene.json" && document.isMainScene());
        CHECK(!document.dirty());
        CHECK(settings.recentProjectPaths[0] == demoManifest);
        CHECK(settings.recentProjectPaths[1] == otherManifest);
        CHECK(settings.lastProjectPath == demoManifest);
        CHECK(near(energyOf(document.scene()), 2.0f));
        CHECK(editor.tree.getRoot()->getChildCount() == 1);
        CHECK(ui.windowTitle() == "Demo - second.scene.json - " SIGNATURE);
    }

    makeNode(editor, "Node3D", "Pending");
    CHECK(document.dirty());
    ui.openRecent(1);
    frames(editor, 2);
    CHECK(ui.confirming());
    clickDialogButton(editor, ui.confirmDialog(), kConfirmCancel);
    CHECK(project.name() == "Demo" && document.dirty());
    CHECK(childNamed(document.scene(), "Pending") != nullptr);
    ui.openRecent(1);
    frames(editor, 2);
    clickDialogButton(editor, ui.confirmDialog(), kConfirmDiscard);
    frames(editor, 3);
    CHECK(project.name() == "Other" && !document.dirty());
    CHECK(settings.recentProjectPaths[0] == otherManifest);
    ui.openRecent(99);
    CHECK(project.name() == "Other");

    ui.openProject();
    frames(editor, 2);
    rt::FileDialog* chooser = ui.fileDialog();
    CHECK(chooser != nullptr);
    if (chooser)
    {
        CHECK(chooser->filter() == "*.json");
        chooser->accepted.emit(demoManifest);
        rt::WidgetApp::instance().removeFloat(chooser);
        frames(editor, 3);
        CHECK(project.name() == "Demo" && document.isMainScene());
    }

    const ct::String brokenScene = project.root() + "/Scenes/second.scene.json";
    const ct::String goodScene = readFile(brokenScene);
    CHECK(FileSystem::writeText(brokenScene, "{ broken"));
    ui.openRecent(1);
    frames(editor, 3);
    CHECK(project.name() == "Other");
    ui.openRecent(1);
    frames(editor, 3);
    CHECK(project.name() == "Demo");
    CHECK(document.scenePath().empty() && document.scene()->getName() == "Scene");
    CHECK(!document.error().empty());
    CHECK(!document.dirty());
    CHECK(FileSystem::writeText(brokenScene, goodScene));
    CHECK(document.openScene("Scenes/second.scene.json"));

    settings.recentProjectPaths.push_back(testRoot() + "/gone/projecto.json");
    const std::size_t count = settings.recentProjectPaths.size();
    ui.openRecent(count - 1);
    frames(editor, 2);
    CHECK(project.name() == "Demo");
    CHECK(settings.recentProjectPaths.size() == count - 1);
    CHECK(!document.openProject(testRoot() + "/gone/projecto.json"));
    CHECK(!document.error().empty() && project.name() == "Demo");

    ui.closeProject();
    frames(editor, 3);
    CHECK(!project.isOpen());
    CHECK(settings.lastProjectPath.empty());
    CHECK(settings.recentProjectPaths.size() == 2);
    CHECK(document.scene()->getName() == "Scene" && !document.hasScenePath());
    CHECK(ui.windowTitle() == "Untitled - " SIGNATURE);
    CHECK(!bar->isItemEnabled(ui.buttonId(ProjectUi::kSave)));
    CHECK(editor.tree.getRoot()->getChildCount() == 1);
    ui.rebuildRecentMenu();
    CHECK(ui.recentMenu()->actionCount() == 4);

    ui.clearRecent();
    ui.update();
    CHECK(settings.recentProjectPaths.size() == 0);
    CHECK(!bar->isItemEnabled(ui.buttonId(ProjectUi::kRecentProjects)));
    EditorSettings cleared;
    CHECK(cleared.load(document.settingsPath()));
    CHECK(cleared.recentProjectPaths.size() == 0 && cleared.lastProjectPath.empty());

    CHECK(document.openProject(demoManifest));
    CHECK(project.name() == "Demo" && settings.recentProjectPaths.size() == 1);
    CHECK(document.openProject(project.root()));
    CHECK(project.name() == "Demo" && settings.recentProjectPaths.size() == 1);
    CHECK(document.scenePath() == "Scenes/second.scene.json");
    reloaded = EditorSettings();
    CHECK(reloaded.load(document.settingsPath()));
    CHECK(reloaded.lastProjectPath == demoManifest);

    const int newButton = toolbarCenterX(editor, 1);
    const rt::Rect bounds = bar->absoluteRect();
    click(editor, newButton, static_cast<int>(bounds.y + bounds.h * 0.5f));
    CHECK(ui.newProjectDialog() != nullptr || floatCount() > 0);
    if (NewProjectDialog* dialog = ui.newProjectDialog())
    {
        CHECK(dialog->parentFolder() == parent);
        clickWidget(editor, dialog->cancelButton());
        frames(editor, 2);
        CHECK(ui.newProjectDialog() == nullptr);
    }
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
    desc.title = "editor project test";
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

                ProjectUi ui(document);
                CHECK(ui.attach(shell.toolBar()));
                CHECK(!ui.attach(shell.toolBar()));

                Editor editor = { tree, host, shell, selection, hierarchy, inspector, document,
                    ui };
                frames(editor, 3);
                testProjectAndScenes(editor);
                testInspectorTypes(editor);
                testRows(editor);
                testUnsaved(editor);
                testRecents(editor);
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
