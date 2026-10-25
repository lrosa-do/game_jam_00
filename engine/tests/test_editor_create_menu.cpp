#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/CreateMenu.h"
#include "prisma3d/editor/CreateNodeDialog.h"
#include "prisma3d/editor/CreatePrimitiveDialog.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/PrimitiveParameterForm.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
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
    root += useVulkan ? "/editor_create_menu_test_vulkan" : "/editor_create_menu_test";
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

InputEvent key(Key which, bool pressed, unsigned modifiers = 0, bool echo = false)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = which;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = echo;
    event.key.modifiers = modifiers;
    return event;
}

InputEvent character(char c)
{
    InputEvent event;
    event.type = InputEventType::Char;
    event.character.codepoint = static_cast<std::uint32_t>(c);
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
        host.frame(delta, true);
        Main::rendering()->endFrame(true);
    }

    void frames(int count)
    {
        for (int i = 0; i < count; ++i) frame();
    }

    bool tap(Key which, unsigned modifiers = 0, bool echo = false)
    {
        const bool consumed = send(key(which, true, modifiers, echo));
        frames(2);
        send(key(which, false, modifiers));
        frames(2);
        return consumed;
    }

    void typeText(const char* text)
    {
        for (const char* c = text; *c; ++c) send(character(*c));
        frames(2);
    }

    void settle()
    {
        for (int i = 0; i < 5; ++i) frame(0.1f);
    }

    void click(int x, int y, MouseButton which = MouseButton::Left)
    {
        settle();
        send(motion(x, y));
        frames(2);
        send(button(x, y, which, true));
        frames(2);
        send(button(x, y, which, false));
        frames(3);
    }
};

Node3D* add(Node* parent, const char* name)
{
    Node3D* node = parent->createChild<Node3D>();
    node->setName(name);
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

void commit(Rig& rig)
{
    rig.frames(3);
    ct::String name = "scene";
    name.append_number(++rig.sceneCounter);
    const bool saved = rig.document.saveSceneAs(name);
    CHECK(saved);
    if (!saved)
        log_error("commit '%s': %s\n%s", name.c_str(), rig.document.error().c_str(),
                rig.document.lastResult().describe().c_str());
    rig.frames(3);
    CHECK(!rig.document.dirty());
}

void select(Rig& rig, Node* a, Node* b = nullptr, Node* c = nullptr)
{
    rig.selection.clear();
    if (a) rig.selection.add(a);
    if (b) rig.selection.add(b);
    if (c) rig.selection.add(c);
    rig.frames(3);
}

bool selectedIs(const Rig& rig, Node* a, Node* b = nullptr, Node* c = nullptr)
{
    const std::size_t expected = (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0);
    if (rig.selection.selectedNodes().size() != expected) return false;
    return (!a || rig.selection.isSelected(a)) && (!b || rig.selection.isSelected(b)) &&
           (!c || rig.selection.isSelected(c));
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
    return static_cast<int>(area.x) + 4 + 16 * depthOf(node, rig.hierarchy.editedScene()) + 24;
}

void clickMenuItem(Rig& rig, rt::Menu* menu, int item)
{
    const rt::Theme& theme = rt::Theme::instance();
    const rt::Rect rect = menu->absoluteRect();
    float y = rect.y + 4.0f;
    for (int i = 0; i < item; ++i)
        y += menu->action(i)->isSeparator() ? 9.0f : theme.menuItemHeight;
    const int px = static_cast<int>(rect.x + 24.0f);
    const int py = static_cast<int>(y + theme.menuItemHeight * 0.5f);
    rig.send(motion(px, py));
    rig.frames(2);
    rig.send(button(px, py, MouseButton::Left, true));
    rig.frames(2);
    rig.send(button(px, py, MouseButton::Left, false));
    rig.frames(3);
}

bool openBarMenu(Rig& rig, rt::Menu* menu)
{
    const rt::Rect barRect = rig.shell.menuBar()->absoluteRect();
    const int by = static_cast<int>(barRect.y + barRect.h * 0.5f);
    for (int x = static_cast<int>(barRect.x) + 4; x < 500; x += 4)
    {
        rig.send(motion(x, by));
        rig.frames(2);
        rig.send(button(x, by, MouseButton::Left, true));
        rig.frames(2);
        rig.send(button(x, by, MouseButton::Left, false));
        rig.frames(2);
        if (rt::WidgetApp::instance().popup() == menu) return true;
    }
    return false;
}

bool near(float a, float b)
{
    return fabsf(a - b) < 1e-4f;
}

bool nearVec(const Math::Vec3& a, const Math::Vec3& b)
{
    return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z);
}

Node* makeCrate(Rig& rig, Node* scene, const char* fileName, const char* instanceName)
{
    const ct::String path = testRoot() + "/" + fileName;
    Prefab::clearCache();
    {
        Node3D scratch;
        Node3D* root = scratch.createChild<Node3D>();
        root->setName("crate");
        add(root, "lid");
        add(root, "base");
        SceneResult saved;
        CHECK(Prefab::saveFromNode(root, path, saved));
    }
    SceneResult placed;
    Node* instance = Prefab::instantiate(path, scene, placed);
    CHECK(instance != nullptr && placed.success());
    if (instance) instance->setName(instanceName);
    rig.frames(2);
    return instance;
}

CreatePrimitiveDialog* primitiveDialog(Rig& rig)
{
    return rig.actions.createMenu().primitiveDialog();
}

int primitiveDialogCount()
{
    int count = 0;
    const ct::Vector<rt::FloatWindow*>& floats = rt::WidgetApp::instance().floats();
    for (std::size_t i = 0; i < floats.size(); ++i)
        if (rt::widget_cast<CreatePrimitiveDialog>(floats[i])) ++count;
    return count;
}

void closeDialogs(Rig& rig)
{
    while (CreatePrimitiveDialog* box = primitiveDialog(rig)) box->cancel();
    while (CreateNodeDialog* box = rig.actions.createDialog()) box->cancel();
    rig.frames(3);
}

void setFocus(Rig& rig, float x, float y, float z)
{
    EditorCamera::Cursor& cursor = rig.panel.camera().cursor();
    cursor.posX = static_cast<double>(x);
    cursor.posY = static_cast<double>(y);
    cursor.posZ = static_cast<double>(z);
}

ct::String describe(const Node* node)
{
    ct::String text = node->getName();
    text += ":";
    text += node->getClassName();
    ct::String inner;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        if (node->getChild(i)->isInternal()) continue;
        if (!inner.empty()) inner += ",";
        inner += describe(node->getChild(i));
    }
    if (!inner.empty()) text += "(" + inner + ")";
    return text;
}

ct::String childrenOf(const Node* node)
{
    ct::String text;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        if (node->getChild(i)->isInternal()) continue;
        if (!text.empty()) text += ",";
        text += describe(node->getChild(i));
    }
    return text;
}

void trigger(rt::Menu* menu, CreateMenu::Entry entry, int index)
{
    rt::MenuAction* action = menu->action(static_cast<int>(entry));
    if (CreateMenu::hasSubmenu(entry)) action = action->submenu()->action(index);
    action->triggered.emit();
}

void triggerItem(Rig& rig, CreateMenu::Item item)
{
    rt::Menu* menu = rig.menus.menu(EditorMenus::kCreate);
    for (int e = CreateMenu::kEntryEmpty; e < CreateMenu::kEntryCount; ++e)
    {
        const CreateMenu::Entry entry = static_cast<CreateMenu::Entry>(e);
        for (int i = 0; i < CreateMenu::entryItemCount(entry); ++i)
        {
            if (CreateMenu::entryItem(entry, i) != item) continue;
            trigger(menu, entry, i);
            rig.frames(3);
            return;
        }
    }
    CHECK(false);
}

PropertyValue property(const Node* node, const char* name)
{
    PropertyValue value;
    NodeClassDB::getProperty(node, name, value);
    return value;
}

ct::String recipePath(PrimitiveKind kind)
{
    PrimitiveRecipe recipe;
    primitiveDefaults(kind, recipe);
    return primitiveRecipePath(recipe);
}

const MeshInfo* meshInfoOf(const Node* node)
{
    const MeshInstance3D* instance = node ? node->castTo<MeshInstance3D>() : nullptr;
    return instance ? Main::meshes()->get(instance->getMesh()) : nullptr;
}

void checkMenuShape(rt::Menu* menu)
{
    CHECK(menu != nullptr);
    if (!menu) return;
    CHECK(menu->actionCount() == CreateMenu::kEntryCount);
    CHECK(menu->action(CreateMenu::kEntryAddNode)->text() == "Add Node...");
    CHECK(menu->action(CreateMenu::kEntryAddNode)->shortcut() == "Ctrl+A");
    CHECK(menu->action(CreateMenu::kEntryAddNode)->submenu() == nullptr);
    CHECK(menu->action(CreateMenu::kEntrySeparator)->isSeparator());

    const char* const topLabels[] = { "Empty", "3D Object", "Light", "Camera", "Audio", "Physics",
        "Effects", "Environment", "Vegetation" };
    const bool submenus[] = { false, true, true, false, true, true, true, true, true };
    const char* const children[][7] = {
        { "Empty" },
        { "Cube", "Sphere", "Capsule", "Cylinder", "Cone", "Plane", "Torus" },
        { "Directional Light", "Point Light", "Spot Light" },
        { "Camera" },
        { "Audio Source", "Audio Listener" },
        { "Static Body", "Rigid Body", "Character Body" },
        { "Particles", "Decal", "Reflection Probe" },
        { "World Environment", "Terrain...", "Water", "Ocean" },
        { "Tree", "Grass", "Forest" },
    };
    const int counts[] = { 1, 7, 3, 1, 2, 3, 3, 4, 3 };
    for (int i = 0; i < 9; ++i)
    {
        const int index = CreateMenu::kEntryEmpty + i;
        const rt::MenuAction* action = menu->action(index);
        CHECK(action->text() == topLabels[i]);
        CHECK(!action->isSeparator() && action->isEnabled());
        CHECK(CreateMenu::hasSubmenu(static_cast<CreateMenu::Entry>(index)) == submenus[i]);
        if (!submenus[i])
        {
            CHECK(action->submenu() == nullptr);
            continue;
        }
        const rt::Menu* submenu = action->submenu();
        CHECK(submenu != nullptr);
        if (!submenu) continue;
        CHECK(submenu->actionCount() == counts[i]);
        for (int c = 0; c < counts[i] && c < submenu->actionCount(); ++c)
        {
            CHECK(submenu->action(c)->text() == children[i][c]);
            CHECK(submenu->action(c)->isEnabled());
        }
    }
}

void testMenuStructure(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    commit(rig);

    CHECK(EditorMenus::kMenuCount == 7 && EditorMenus::kCreate == 2 && EditorMenus::kRun == 5 &&
            EditorMenus::kHelp == EditorMenus::kMenuCount - 1);
    CHECK(strcmp(EditorMenus::menuTitle(EditorMenus::kCreate), "Create") == 0);
    CHECK(strcmp(EditorMenus::menuTitle(EditorMenus::kEdit), "Edit") == 0);
    CHECK(strcmp(EditorMenus::menuTitle(EditorMenus::kView), "View") == 0);
    rt::Menu* bar = rig.menus.menu(EditorMenus::kCreate);
    CHECK(rig.shell.menuBar()->menu(1) == rig.menus.menu(EditorMenus::kEdit));
    CHECK(rig.shell.menuBar()->menu(2) == bar);
    CHECK(rig.shell.menuBar()->menu(3) == rig.menus.menu(EditorMenus::kView));
    checkMenuShape(bar);

    rt::Menu* context = rig.actions.contextMenu();
    CHECK(context != nullptr);
    if (context)
    {
        CHECK(context->actionCount() ==
                HierarchyActions::kItemCount + HierarchyActions::kContextOffset);
        CHECK(context->action(0)->text() == "Create");
        CHECK(context->action(1)->isSeparator());
        checkMenuShape(context->action(0)->submenu());
        CHECK(context->action(0)->submenu() != bar);
        CHECK(context->action(2)->text() == "Add Child Node...");
    }

    rt::ToolBar* toolBar = rig.hierarchy.toolBar();
    CHECK(toolBar != nullptr && rig.actions.addButtonId() >= 0);
    rt::Menu* dropdown = rig.actions.createMenu().dropdown();
    CHECK(dropdown != nullptr && dropdown != bar);
    checkMenuShape(dropdown);

    rt::Menu* edit = rig.menus.menu(EditorMenus::kEdit);
    CHECK(edit->actionCount() == EditorMenus::kEditItemCount);

    CHECK(CreateMenu::className(CreateMenu::kPointLight) == ct::String("OmniLight3D"));
    for (int i = 0; i < CreateMenu::kItemCount; ++i)
    {
        const CreateMenu::Item item = static_cast<CreateMenu::Item>(i);
        CHECK(NodeClassDB::classExists(ct::String(CreateMenu::className(item))));
        CHECK(NodeClassDB::canInstantiate(ct::String(CreateMenu::className(item))));
        CHECK(strlen(CreateMenu::label(item)) > 0);
    }
    for (int e = CreateMenu::kEntryEmpty; e < CreateMenu::kEntryCount; ++e)
    {
        const CreateMenu::Entry entry = static_cast<CreateMenu::Entry>(e);
        for (int i = 0; i < CreateMenu::entryItemCount(entry); ++i)
            CHECK(CreateMenu::entryItem(entry, i) != CreateMenu::kItemCount);
        CHECK(CreateMenu::entryItem(entry, CreateMenu::entryItemCount(entry)) ==
                CreateMenu::kItemCount);
    }

    rig.selection.clear();
    rig.frames(2);
    rig.actions.createMenu().refresh(bar);
    checkMenuShape(bar);
    (void) a;

    CHECK(rig.actions.createDialog() == nullptr);
    bar->action(CreateMenu::kEntryAddNode)->triggered.emit();
    rig.frames(3);
    CHECK(rig.actions.createDialog() != nullptr);
    closeDialogs(rig);
}

void testRealClicks(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    commit(rig);
    setFocus(rig, 0.0f, 0.0f, 0.0f);
    rt::Menu* bar = rig.menus.menu(EditorMenus::kCreate);

    select(rig, nullptr);
    CHECK(openBarMenu(rig, bar));
    clickMenuItem(rig, bar, CreateMenu::kEntryEmpty);
    CHECK(rt::WidgetApp::instance().popup() == nullptr);
    Node* empty = scene->findChild("Node3D");
    CHECK(empty != nullptr && selectedIs(rig, empty));
    CHECK(rig.document.dirty());
    commit(rig);

    CHECK(openBarMenu(rig, bar));
    clickMenuItem(rig, bar, CreateMenu::kEntryObject);
    rt::Menu* objects = bar->action(CreateMenu::kEntryObject)->submenu();
    CHECK(objects != nullptr && rt::WidgetApp::instance().popup() == bar);
    if (objects)
    {
        clickMenuItem(rig, objects, 1);
        CHECK(primitiveDialog(rig) != nullptr);
        CHECK(rt::WidgetApp::instance().popup() == nullptr);
        CHECK(empty->findChild("Sphere") == nullptr);
        rig.tap(Key::Enter);
        Node* sphere = empty ? empty->findChild("Sphere") : nullptr;
        CHECK(sphere != nullptr && selectedIs(rig, sphere));
        CHECK(primitiveDialog(rig) == nullptr);
    }
    commit(rig);

    select(rig, a);
    rig.click(textX(rig, a), rowY(rig, a), MouseButton::Right);
    rt::Menu* context = rig.actions.contextMenu();
    CHECK(rt::WidgetApp::instance().popup() == context);
    clickMenuItem(rig, context, 0);
    rt::Menu* nested = context->action(0)->submenu();
    CHECK(nested != nullptr && rt::WidgetApp::instance().popup() == context);
    if (nested)
    {
        clickMenuItem(rig, nested, CreateMenu::kEntryCamera);
        Node* camera = a->findChild("Camera3D");
        CHECK(camera != nullptr && selectedIs(rig, camera));
        CHECK(rt::WidgetApp::instance().popup() == nullptr);
    }
    commit(rig);

    select(rig, nullptr);
    rt::ToolBar* toolBar = rig.hierarchy.toolBar();
    CHECK(toolBar != nullptr);
    if (!toolBar) return;
    const rt::Rect barRect = toolBar->absoluteRect();
    rig.click(static_cast<int>(barRect.x + 2.0f + 14.0f),
            static_cast<int>(barRect.y + barRect.h * 0.5f));
    rt::Menu* dropdown = rig.actions.createMenu().dropdown();
    CHECK(dropdown != nullptr && rt::WidgetApp::instance().popup() == dropdown);
    if (dropdown)
    {
        const rt::Rect menuRect = dropdown->absoluteRect();
        CHECK(menuRect.y >= barRect.y + barRect.h - 1.0f);
        CHECK(menuRect.x >= barRect.x - 1.0f && menuRect.x <= barRect.x + barRect.w);
        clickMenuItem(rig, dropdown, CreateMenu::kEntryLight);
        rt::Menu* lights = dropdown->action(CreateMenu::kEntryLight)->submenu();
        CHECK(lights != nullptr && rt::WidgetApp::instance().popup() == dropdown);
        if (lights) clickMenuItem(rig, lights, 1);
        Node* light = scene->findChild("OmniLight3D");
        CHECK(light != nullptr && selectedIs(rig, light));
    }
    commit(rig);

    CHECK(rig.actions.createDialog() == nullptr);
    rig.tap(Key::A, kModControl);
    CHECK(rig.actions.createDialog() != nullptr);
    closeDialogs(rig);
}

struct ItemCase
{
    CreateMenu::Item item;
    const char* tree;
};

void testEveryItem(Rig& rig)
{
    const ItemCase cases[] = {
        { CreateMenu::kEmpty, "Node3D:Node3D" },
        { CreateMenu::kDirectionalLight, "DirectionalLight3D:DirectionalLight3D" },
        { CreateMenu::kPointLight, "OmniLight3D:OmniLight3D" },
        { CreateMenu::kSpotLight, "SpotLight3D:SpotLight3D" },
        { CreateMenu::kCamera, "Camera3D:Camera3D" },
        { CreateMenu::kAudioSource, "AudioStreamPlayer3D:AudioStreamPlayer3D" },
        { CreateMenu::kAudioListener, "AudioListener3D:AudioListener3D" },
        { CreateMenu::kStaticBody, "StaticBody3D:StaticBody3D(CollisionShape3D:CollisionShape3D)" },
        { CreateMenu::kRigidBody, "RigidBody3D:RigidBody3D(CollisionShape3D:CollisionShape3D)" },
        { CreateMenu::kCharacterBody,
            "CharacterBody3D:CharacterBody3D(CollisionShape3D:CollisionShape3D)" },
        { CreateMenu::kParticles, "ParticleSystem3D:ParticleSystem3D" },
        { CreateMenu::kDecal, "Decal3D:Decal3D" },
        { CreateMenu::kReflectionProbe, "ReflectionProbe3D:ReflectionProbe3D" },
        { CreateMenu::kWorldEnvironment, "WorldEnvironment:WorldEnvironment" },
    };
    const Math::Vec3 focus(2.5f, -1.0f, 4.0f);

    for (const ItemCase& entry: cases)
    {
        Node* scene = freshScene(rig);
        commit(rig);
        setFocus(rig, focus.x, focus.y, focus.z);
        select(rig, nullptr);
        triggerItem(rig, entry.item);
        CHECK(scene->getChildCount() == 1);
        Node* made = scene->getChildCount() > 0 ? scene->getChild(0) : nullptr;
        CHECK(made != nullptr);
        if (!made) continue;
        if (describe(made) != entry.tree)
            printf("item %s: %s\n", CreateMenu::label(entry.item), describe(made).c_str());
        CHECK(describe(made) == entry.tree);
        CHECK(selectedIs(rig, made));
        CHECK(rig.document.dirty());
        CHECK(made->getParent() == scene);
        CHECK(strcmp(made->getClassName(), CreateMenu::className(entry.item)) == 0);
        if (Node3D* spatial = made->castTo<Node3D>())
            CHECK(nearVec(spatial->getGlobalPosition(), focus));
        CHECK(rig.actions.createDialog() == nullptr && primitiveDialog(rig) == nullptr);

        select(rig, nullptr);
        triggerItem(rig, entry.item);
        CHECK(scene->getChildCount() == 2);
        Node* second = scene->getChildCount() > 1 ? scene->getChild(1) : nullptr;
        CHECK(second != nullptr && second != made && selectedIs(rig, second));
        if (second)
        {
            ct::String expected = made->getName() + "2";
            CHECK(second->getName() == expected);
        }
        commit(rig);
    }

    {
        Node* scene = freshScene(rig);
        setFocus(rig, 1.0f, 2.0f, 3.0f);
        select(rig, nullptr);
        triggerItem(rig, CreateMenu::kDirectionalLight);
        Node3D* light = scene->getChild(0)->castTo<Node3D>();
        CHECK(light != nullptr);
        Node3D* reference = scene->createChild<Node3D>();
        reference->setName("Reference");
        reference->lookAtFromPosition(CreateMenu::sunFrom(), CreateMenu::sunTarget());
        const Math::Vec3 wanted =
                reference->getGlobalTransform().TransformPoint(Math::Vec3(0.0f, 0.0f, -1.0f)) -
                reference->getGlobalPosition();
        if (light)
        {
            const Math::Vec3 got =
                    light->getGlobalTransform().TransformPoint(Math::Vec3(0.0f, 0.0f, -1.0f)) -
                    light->getGlobalPosition();
            CHECK(nearVec(got, wanted));
            CHECK(nearVec(light->getGlobalPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
            CHECK(!light->castTo<DirectionalLight3D>()->hasShadow());
        }
        CHECK(near(CreateMenu::sunFrom().x, 4.0f) && near(CreateMenu::sunFrom().y, 6.0f) &&
                near(CreateMenu::sunFrom().z, 3.0f));
        reference->queueFree();
        rig.frames(3);
    }

    {
        Node* scene = freshScene(rig);
        setFocus(rig, 0.0f, 0.0f, 0.0f);
        select(rig, nullptr);
        for (int e = CreateMenu::kEntryEmpty; e < CreateMenu::kEntryCount; ++e)
        {
            const CreateMenu::Entry entry = static_cast<CreateMenu::Entry>(e);
            for (int i = 0; i < CreateMenu::entryItemCount(entry); ++i)
            {
                const CreateMenu::Item item = CreateMenu::entryItem(entry, i);
                ct::String error;
                select(rig, nullptr);
                CHECK(rig.actions.createMenu().create(item, error));
                CHECK(error.empty());
            }
        }
        if (scene->getChildCount() != static_cast<std::size_t>(CreateMenu::kItemCount))
            printf("created %zu of %d: %s\n", scene->getChildCount(), CreateMenu::kItemCount,
                    childrenOf(scene).c_str());
        CHECK(scene->getChildCount() == static_cast<std::size_t>(CreateMenu::kItemCount));
        rig.frames(2);
        commit(rig);
    }
}

void testPlacement(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* parent = add(scene, "Parent");
    parent->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    parent->setRotationDegrees(Math::Vec3(0.0f, 90.0f, 0.0f));
    parent->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    Node3D* other = add(scene, "Other");
    commit(rig);
    setFocus(rig, 3.0f, 1.0f, -2.0f);

    select(rig, parent);
    triggerItem(rig, CreateMenu::kEmpty);
    Node3D* child =
            parent->findChild("Node3D") ? parent->findChild("Node3D")->castTo<Node3D>() : nullptr;
    CHECK(child != nullptr && selectedIs(rig, child));
    if (child) CHECK(nearVec(child->getGlobalPosition(), Math::Vec3(3.0f, 1.0f, -2.0f)));
    CHECK(scene->findChild("Node3D") == nullptr);

    select(rig, parent);
    triggerItem(rig, CreateMenu::kStaticBody);
    Node* body = parent->findChild("StaticBody3D");
    CHECK(body != nullptr && selectedIs(rig, body));
    if (Node3D* spatial = body ? body->castTo<Node3D>() : nullptr)
        CHECK(nearVec(spatial->getGlobalPosition(), Math::Vec3(3.0f, 1.0f, -2.0f)));

    select(rig, parent, other);
    triggerItem(rig, CreateMenu::kCamera);
    Node* first = parent->findChild("Camera3D");
    Node* second = other->findChild("Camera3D");
    CHECK(first != nullptr && second != nullptr && selectedIs(rig, first, second));
    if (first && second)
    {
        CHECK(nearVec(first->castTo<Node3D>()->getGlobalPosition(), Math::Vec3(3.0f, 1.0f, -2.0f)));
        CHECK(nearVec(second->castTo<Node3D>()->getGlobalPosition(),
                Math::Vec3(3.0f, 1.0f, -2.0f)));
    }

    select(rig, nullptr);
    rig.actions.setViewport(nullptr);
    triggerItem(rig, CreateMenu::kCamera);
    Node* atOrigin = scene->findChild("Camera3D");
    CHECK(atOrigin != nullptr);
    if (atOrigin) CHECK(nearVec(atOrigin->castTo<Node3D>()->getGlobalPosition(), Math::Vec3()));
    CHECK(nearVec(rig.actions.focusPoint(), Math::Vec3()));
    rig.actions.setViewport(&rig.panel);
    CHECK(nearVec(rig.actions.focusPoint(), Math::Vec3(3.0f, 1.0f, -2.0f)));
    rig.panel.camera().cursor().posX = 7.0;
    CHECK(nearVec(rig.actions.focusPoint(), Math::Vec3(7.0f, 1.0f, -2.0f)));

    Node* cratesScene = freshScene(rig);
    Node* crate = makeCrate(rig, cratesScene, "menu_crate.prefab.json", "crate");
    Node* lid = crate ? crate->findChild("lid") : nullptr;
    CHECK(lid != nullptr);
    if (lid)
    {
        commit(rig);
        select(rig, lid);
        CHECK(!cratesScene->isEditableInstance(crate));
        setFocus(rig, 1.0f, 1.0f, 1.0f);
        triggerItem(rig, CreateMenu::kPointLight);
        Node* light = lid->findChild("OmniLight3D");
        CHECK(light != nullptr && selectedIs(rig, light));
        CHECK(cratesScene->isEditableInstance(crate));
        CHECK(light && light->getOwner() == nullptr);
        CHECK(rig.document.dirty());
    }

    Node* root = freshScene(rig);
    Node* internal = add(root, "Internal");
    internal->setInternal(true);
    select(rig, internal);
    ct::String error;
    CHECK(!rig.actions.createMenu().create(CreateMenu::kEmpty, error));
    CHECK(root->getChildCount() == 1);
    (void) other;
}

void openKind(Rig& rig, CreateMenu::Item item, bool clear = true)
{
    if (clear) select(rig, nullptr);
    triggerItem(rig, item);
    CHECK(primitiveDialog(rig) != nullptr);
}

void typeNumber(Rig& rig, NumberField* field, const char* text)
{
    rig.click(static_cast<int>(field->absoluteRect().x + field->absoluteRect().w * 0.5f),
            static_cast<int>(field->absoluteRect().y + field->absoluteRect().h * 0.5f));
    CHECK(field->isEditing());
    rig.typeText(text);
    rig.tap(Key::Enter);
}

void testPrimitiveDialog(Rig& rig)
{
    const struct
    {
        CreateMenu::Item item;
        PrimitiveKind kind;
        const char* label;
    } kinds[] = {
        { CreateMenu::kCube, PrimitiveKind::Box, "Cube" },
        { CreateMenu::kSphere, PrimitiveKind::Sphere, "Sphere" },
        { CreateMenu::kCapsule, PrimitiveKind::Capsule, "Capsule" },
        { CreateMenu::kCylinder, PrimitiveKind::Cylinder, "Cylinder" },
        { CreateMenu::kCone, PrimitiveKind::Cone, "Cone" },
        { CreateMenu::kPlane, PrimitiveKind::Plane, "Plane" },
        { CreateMenu::kTorus, PrimitiveKind::Torus, "Torus" },
    };
    const Math::Vec3 focus(-4.0f, 0.5f, 6.0f);

    for (const auto& entry: kinds)
    {
        Node* scene = freshScene(rig);
        commit(rig);
        setFocus(rig, focus.x, focus.y, focus.z);
        select(rig, nullptr);
        openKind(rig, entry.item);
        CreatePrimitiveDialog* box = primitiveDialog(rig);
        if (!box) continue;
        CHECK(primitiveDialogCount() == 1);
        CHECK(box->title() == ct::String("Create ") + entry.label);
        CHECK(box->kind() == entry.kind && box->label() == entry.label);
        CHECK(box->nameText() == entry.label);
        CHECK(rt::WidgetApp::instance().focused() == box->nameInput());
        CHECK(scene->getChildCount() == 0);

        PrimitiveRecipe defaults;
        primitiveDefaults(entry.kind, defaults);
        PrimitiveParameterForm* form = box->parameters();
        CHECK(form->parameterCount() == static_cast<int>(primitiveParamCount(entry.kind)));
        CHECK(box->recipePath() == primitiveRecipePath(defaults));
        int valueIndex = 0;
        for (int p = 0; p < form->parameterCount(); ++p)
        {
            const PrimitiveParamDef& def = primitiveParam(entry.kind, static_cast<std::size_t>(p));
            CHECK(form->parameterKey(p) == def.key);
            const int components = def.type == PrimitiveParamType::Vec3 ? 3 : 1;
            CHECK(form->fieldCount(p) == components);
            for (int c = 0; c < components; ++c)
            {
                CHECK(form->field(p, c) != nullptr);
                if (!form->field(p, c)) continue;
                CHECK(near(form->field(p, c)->value(), def.fallback));
                CHECK(near(defaults.values[valueIndex + c], def.fallback));
                CHECK(form->field(p, c)->absoluteRect().w > 20.0f);
            }
            valueIndex += components;
        }
        CHECK(box->triangleCount() == primitiveTriangleCount(defaults));
        CHECK(box->summaryLabel()->text() ==
                CreatePrimitiveDialog::summaryText(primitiveTriangleCount(defaults)));
        CHECK(box->createButton()->isEnabled() && box->createButton()->absoluteRect().w > 20.0f);
        CHECK(box->cancelButton()->absoluteRect().w > 20.0f);
        CHECK(form->heightmapInput() == nullptr);

        const PrimitiveParamDef& first = primitiveParam(entry.kind, 0);
        if (first.type != PrimitiveParamType::Vec3)
        {
            typeNumber(rig, form->field(0, 0), "-3");
            CHECK(near(form->field(0, 0)->value(), first.minimum));
            typeNumber(rig, form->field(0, 0), "999999999");
            CHECK(near(form->field(0, 0)->value(), first.maximum));
            CHECK(form->parameterRevert(0)->active());
            rig.click(static_cast<int>(form->parameterRevert(0)->absoluteRect().x + 4.0f),
                    static_cast<int>(form->parameterRevert(0)->absoluteRect().y + 4.0f));
            CHECK(near(form->field(0, 0)->value(), first.fallback));
        }
        else
        {
            typeNumber(rig, form->field(0, 1), "-3");
            CHECK(near(form->field(0, 1)->value(), 0.0f));
        }
        if (entry.kind == PrimitiveKind::Sphere)
        {
            typeNumber(rig, form->field(1, 0), "1");
            CHECK(near(form->field(1, 0)->value(), 2.0f));
            typeNumber(rig, form->field(2, 0), "99999");
            CHECK(near(form->field(2, 0)->value(), 4096.0f));
            typeNumber(rig, form->field(1, 0), "7.6");
            CHECK(near(form->field(1, 0)->value(), 8.0f));
            CHECK(box->triangleCount() == primitiveTriangleCount(box->recipe()));
            CHECK(box->summaryLabel()->text() == CreatePrimitiveDialog::summaryText(8 * 4096 * 2));
        }

        rig.tap(Key::Escape);
        CHECK(primitiveDialog(rig) == nullptr);
        CHECK(scene->getChildCount() == 0);
        CHECK(!rig.document.dirty());
        CHECK(rig.actions.createMenu().lastRecipe(entry.kind).values[0] == defaults.values[0]);

        openKind(rig, entry.item);
        box = primitiveDialog(rig);
        CHECK(box && box->recipePath() == primitiveRecipePath(defaults));
        if (box) rig.tap(Key::Enter);
        CHECK(primitiveDialog(rig) == nullptr);
        CHECK(scene->getChildCount() == 1);
        Node* made = scene->getChildCount() > 0 ? scene->getChild(0) : nullptr;
        CHECK(made != nullptr);
        if (!made) continue;
        CHECK(describe(made) == ct::String(entry.label) + ":MeshInstance3D");
        CHECK(property(made, "mesh").asString() == primitiveRecipePath(defaults));
        CHECK(selectedIs(rig, made) && rig.document.dirty());
        CHECK(nearVec(made->castTo<Node3D>()->getGlobalPosition(), focus));
        CHECK(meshInfoOf(made) != nullptr);
        commit(rig);
    }

    {
        Node* scene = freshScene(rig);
        select(rig, nullptr);
        setFocus(rig, 0.0f, 0.0f, 0.0f);

        openKind(rig, CreateMenu::kSphere);
        CreatePrimitiveDialog* box = primitiveDialog(rig);
        CHECK(box != nullptr);
        if (!box) return;
        typeNumber(rig, box->parameters()->field(0, 0), "2");
        typeNumber(rig, box->parameters()->field(1, 0), "10");
        typeNumber(rig, box->parameters()->field(2, 0), "12");
        const ct::String chosen = "primitive:sphere?radius=2&rings=10&slices=12";
        CHECK(box->recipePath() == chosen);
        CHECK(box->summaryLabel()->text() == CreatePrimitiveDialog::summaryText(10 * 12 * 2));
        box->setNameText("Ball");
        rig.click(static_cast<int>(box->createButton()->absoluteRect().x + 6.0f),
                static_cast<int>(box->createButton()->absoluteRect().y + 6.0f));
        CHECK(primitiveDialog(rig) == nullptr);
        Node* ball = scene->findChild("Ball");
        CHECK(ball != nullptr && property(ball, "mesh").asString() == chosen);
        CHECK(rig.actions.createMenu().lastRecipe(PrimitiveKind::Sphere).values[0] == 2.0f);
        CHECK(rig.actions.createMenu().lastRecipe(PrimitiveKind::Box).values[0] == 1.0f);
        CHECK(near(meshInfoOf(ball)->aabb.Size().x, 4.0f) ||
                near(meshInfoOf(ball)->aabb.Size().x, 4.0f * 1.0f));

        openKind(rig, CreateMenu::kSphere);
        box = primitiveDialog(rig);
        CHECK(box != nullptr && box->recipePath() == chosen);
        CHECK(box && box->nameText() == "Sphere");
        if (box)
        {
            CHECK(near(box->parameters()->field(0, 0)->value(), 2.0f));
            CHECK(near(box->parameters()->field(1, 0)->value(), 10.0f));
            box->cancel();
            rig.frames(3);
        }

        openKind(rig, CreateMenu::kCube);
        box = primitiveDialog(rig);
        CHECK(box != nullptr && box->recipePath() == recipePath(PrimitiveKind::Box));
        openKind(rig, CreateMenu::kTorus);
        CHECK(primitiveDialogCount() == 1);
        box = primitiveDialog(rig);
        CHECK(box != nullptr && box->kind() == PrimitiveKind::Torus);
        if (box) box->cancel();
        rig.frames(3);

        openKind(rig, CreateMenu::kSphere);
        box = primitiveDialog(rig);
        if (box)
        {
            rig.click(static_cast<int>(box->nameInput()->absoluteRect().x + 10.0f),
                    static_cast<int>(box->nameInput()->absoluteRect().y + 6.0f));
            box->nameInput()->selectAll();
            rig.typeText("Sphere");
            rig.tap(Key::Enter);
        }
        CHECK(primitiveDialog(rig) == nullptr);
        CHECK(scene->findChild("Sphere") != nullptr);
        CHECK(property(scene->findChild("Sphere"), "mesh").asString() == chosen);

        openKind(rig, CreateMenu::kSphere);
        box = primitiveDialog(rig);
        if (box)
        {
            box->setNameText("Sphere");
            box->accept();
            rig.frames(3);
        }
        CHECK(scene->findChild("Sphere2") != nullptr);
        CHECK(selectedIs(rig, scene->findChild("Sphere2")));

        openKind(rig, CreateMenu::kCone);
        box = primitiveDialog(rig);
        if (box)
        {
            box->setNameText("a.b%c");
            box->accept();
            rig.frames(3);
        }
        CHECK(scene->findChild("a_b_c") != nullptr);
        openKind(rig, CreateMenu::kCone);
        box = primitiveDialog(rig);
        if (box)
        {
            box->setNameText("   ");
            box->accept();
            rig.frames(3);
        }
        CHECK(scene->findChild("Cone") != nullptr);

        openKind(rig, CreateMenu::kCylinder);
        box = primitiveDialog(rig);
        CHECK(box != nullptr);
        if (box)
        {
            box->closed.emit();
            rig.frames(3);
        }
        CHECK(primitiveDialog(rig) == nullptr && scene->findChild("Cylinder") == nullptr);

        select(rig, ball);
        openKind(rig, CreateMenu::kCylinder, false);
        box = primitiveDialog(rig);
        if (box)
        {
            box->accept();
            rig.frames(3);
        }
        Node* under = ball ? ball->findChild("Cylinder") : nullptr;
        CHECK(under != nullptr && selectedIs(rig, under));
        commit(rig);
    }
}

void testPhysicsColliders(Rig& rig)
{
    Node* scene = freshScene(rig);
    select(rig, nullptr);
    setFocus(rig, 0.0f, 5.0f, 0.0f);

    PrimitiveRecipe box;
    primitiveDefaults(PrimitiveKind::Box, box);
    const CreateMenu::Item boxBodies[] = { CreateMenu::kStaticBody, CreateMenu::kRigidBody };
    for (CreateMenu::Item item: boxBodies)
    {
        select(rig, nullptr);
        triggerItem(rig, item);
        Node* body = scene->getChild(scene->getChildCount() - 1);
        CHECK(body != nullptr && body->getChildCount() == 1);
        if (!body || body->getChildCount() != 1) continue;
        CollisionShape3D* shape = body->getChild(0)->castTo<CollisionShape3D>();
        CHECK(shape != nullptr);
        if (!shape) continue;
        CHECK(shape->getKind() == CollisionShapeKind::Box);
        CHECK(nearVec(shape->getBoxSize(),
                Math::Vec3(box.values[0], box.values[1], box.values[2])));
        CHECK(nearVec(shape->getGlobalPosition(), Math::Vec3(0.0f, 5.0f, 0.0f)));
        CHECK(nearVec(shape->getPosition(), Math::Vec3()));
    }

    select(rig, nullptr);
    triggerItem(rig, CreateMenu::kCharacterBody);
    Node* character = scene->getChild(scene->getChildCount() - 1);
    CHECK(character != nullptr && character->getChildCount() == 1);
    if (character && character->getChildCount() == 1)
    {
        PrimitiveRecipe capsule;
        primitiveDefaults(PrimitiveKind::Capsule, capsule);
        CollisionShape3D* shape = character->getChild(0)->castTo<CollisionShape3D>();
        CHECK(shape != nullptr);
        if (shape)
        {
            CHECK(shape->getKind() == CollisionShapeKind::Capsule);
            CHECK(near(shape->getCapsuleRadius(), capsule.values[0]));
            CHECK(near(shape->getCapsuleHeight(), capsule.values[1] + 2.0f * capsule.values[0]));
            CHECK(nearVec(shape->getPosition(), Math::Vec3()));
        }
    }
    commit(rig);
}

void testSaveAndReload(Rig& rig)
{
    Node* scene = freshScene(rig);
    select(rig, nullptr);
    setFocus(rig, 1.0f, 2.0f, 3.0f);
    triggerItem(rig, CreateMenu::kCube);
    CHECK(primitiveDialog(rig) != nullptr);
    if (CreatePrimitiveDialog* box = primitiveDialog(rig))
    {
        typeNumber(rig, box->parameters()->field(0, 0), "2");
        box->setNameText("Crate");
        box->accept();
        rig.frames(3);
    }
    select(rig, nullptr);
    triggerItem(rig, CreateMenu::kStaticBody);
    select(rig, nullptr);
    triggerItem(rig, CreateMenu::kCharacterBody);
    select(rig, nullptr);
    triggerItem(rig, CreateMenu::kDirectionalLight);
    select(rig, nullptr);
    triggerItem(rig, CreateMenu::kSpotLight);
    select(rig, nullptr);
    triggerItem(rig, CreateMenu::kWorldEnvironment);
    select(rig, nullptr);
    triggerItem(rig, CreateMenu::kCamera);
    CHECK(rig.document.dirty());
    const ct::String before = childrenOf(scene);

    CHECK(rig.document.saveSceneAs("created"));
    rig.frames(3);
    CHECK(!rig.document.dirty());
    const ct::String path = rig.document.scenePath();
    CHECK(rig.document.newScene());
    rig.frames(3);
    CHECK(rig.document.openScene(path));
    rig.frames(3);
    Node* reopened = rig.document.scene();
    CHECK(reopened != nullptr);
    if (!reopened) return;
    CHECK(!rig.document.dirty());
    CHECK(describe(reopened).find("Crate:MeshInstance3D") != ct::String::npos);
    CHECK(childrenOf(reopened) == before);

    Node* crate = reopened->findChild("Crate");
    CHECK(crate != nullptr);
    if (crate)
    {
        CHECK(property(crate, "mesh").asString() == "primitive:box?size=2,1,1");
        CHECK(meshInfoOf(crate) != nullptr && near(meshInfoOf(crate)->aabb.Size().x, 2.0f));
        CHECK(nearVec(crate->castTo<Node3D>()->getGlobalPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    }
    Node* character = reopened->findChild("CharacterBody3D");
    CHECK(character != nullptr && character->getChildCount() == 1);
    if (character && character->getChildCount() == 1)
    {
        CollisionShape3D* shape = character->getChild(0)->castTo<CollisionShape3D>();
        CHECK(shape != nullptr && shape->getKind() == CollisionShapeKind::Capsule);
        if (shape)
        {
            CHECK(near(shape->getCapsuleRadius(), 0.5f) && near(shape->getCapsuleHeight(), 2.0f));
        }
    }
    Node* statics = reopened->findChild("StaticBody3D");
    CHECK(statics != nullptr && statics->getChildCount() == 1);
    if (statics && statics->getChildCount() == 1)
    {
        CollisionShape3D* shape = statics->getChild(0)->castTo<CollisionShape3D>();
        CHECK(shape != nullptr && shape->getKind() == CollisionShapeKind::Box);
        if (shape) CHECK(nearVec(shape->getBoxSize(), Math::Vec3(1.0f, 1.0f, 1.0f)));
    }
    Node3D* sun = reopened->findChild("DirectionalLight3D")
                          ? reopened->findChild("DirectionalLight3D")->castTo<Node3D>()
                          : nullptr;
    CHECK(sun != nullptr);
    if (sun)
    {
        Node3D* reference = reopened->createChild<Node3D>();
        reference->setName("Reference");
        reference->lookAtFromPosition(CreateMenu::sunFrom(), CreateMenu::sunTarget());
        const Math::Vec3 wanted =
                reference->getGlobalTransform().TransformPoint(Math::Vec3(0.0f, 0.0f, -1.0f)) -
                reference->getGlobalPosition();
        const Math::Vec3 got =
                sun->getGlobalTransform().TransformPoint(Math::Vec3(0.0f, 0.0f, -1.0f)) -
                sun->getGlobalPosition();
        CHECK(nearVec(got, wanted));
        reference->queueFree();
        rig.frames(3);
    }
    CHECK(reopened->findChild("WorldEnvironment") != nullptr);
    CHECK(reopened->findChild("SpotLight3D") != nullptr);
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor create menu test";
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
        actions.setViewport(&panel);
        EditorMenus menus(shell, ui, panel, actions);
        CHECK(menus.attach());
        EditorStatusBar status(document, selection, server);
        CHECK(status.attach(shell.statusBar()));
        status.setGizmo(gizmo);

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, panel,
            actions, menus, status };
        rig.frames(4);

        CHECK(document.createProject(testRoot() + "/projects", "CreateMenu"));
        rig.frames(3);
        CHECK(!document.dirty());

        testMenuStructure(rig);
        testRealClicks(rig);
        testEveryItem(rig);
        testPlacement(rig);
        testPrimitiveDialog(rig);
        testPhysicsColliders(rig);
        testSaveAndReload(rig);

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
