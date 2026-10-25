#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
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
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Joint3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTreeListener.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/WidgetApp.hpp>

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

const char* const kRootText = "This operation can't be done on the tree root.";
const char* const kInvalidText =
        "Invalid node name, the following characters are not allowed:\n. : @ / \" %";

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_actions_test_vulkan" : "/editor_actions_test";
    return root;
}

struct Recorder : SceneTreeListener
{
    int added = 0;
    int removed = 0;
    int renamed = 0;

    void nodeAdded(Node*) override
    {
        ++added;
    }

    void nodeRemoved(Node*) override
    {
        ++removed;
    }

    void nodeRenamed(Node*) override
    {
        ++renamed;
    }
};

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

    void click(int x, int y, MouseButton which = MouseButton::Left)
    {
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

ct::String names(const Node* parent)
{
    ct::String text;
    for (std::size_t i = 0; i < parent->getChildCount(); ++i)
    {
        if (parent->getChild(i)->isInternal()) continue;
        if (!text.empty()) text += ",";
        text += parent->getChild(i)->getName();
    }
    return text;
}

bool sameProperties(const Node* a, const Node* b)
{
    if (strcmp(a->getClassName(), b->getClassName()) != 0) return false;
    ct::Vector<PropertyInfo> list;
    NodeClassDB::getPropertyList(a->getClassName(), list);
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        if (list[i].isGroup() || list[i].name == "name") continue;
        PropertyValue first;
        PropertyValue second;
        const bool okFirst = NodeClassDB::getProperty(a, list[i].name, first);
        const bool okSecond = NodeClassDB::getProperty(b, list[i].name, second);
        if (okFirst != okSecond || (okFirst && !(first == second)))
        {
            printf("property differs: %s\n", list[i].name.c_str());
            return false;
        }
    }
    return true;
}

bool selectedIs(const Rig& rig, Node* a, Node* b = nullptr, Node* c = nullptr)
{
    const std::size_t expected = (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0);
    if (rig.selection.selectedNodes().size() != expected) return false;
    return (!a || rig.selection.isSelected(a)) && (!b || rig.selection.isSelected(b)) &&
           (!c || rig.selection.isSelected(c));
}

rt::Dialog* popupDialog()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return app ? rt::widget_cast<rt::Dialog>(app->popup()) : nullptr;
}

void closePopup(Rig& rig)
{
    rt::WidgetApp::instance().closePopup();
    rig.frames(2);
}

void clickDialogButton(Rig& rig, rt::Dialog* dialog, int index)
{
    CHECK(dialog != nullptr);
    if (!dialog) return;
    const rt::Rect rect = dialog->btnRect_(index);
    rig.click(static_cast<int>(rect.x + rect.w * 0.5f), static_cast<int>(rect.y + rect.h * 0.5f));
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

void testRename(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* box = add(scene, "Box");
    Node3D* sphere = add(scene, "Sphere");
    Node3D* cube = add(scene, "Cube");
    Node3D* child = add(box, "Child");
    Node* hidden = box->createChild<Node3D>();
    hidden->setName("Hidden");
    hidden->setInternal(true);
    commit(rig);

    Recorder recorder;
    rig.tree.addListener(&recorder);
    ct::String error;

    CHECK(rig.actions.rename(box, "Box", error));
    CHECK(error.empty());
    CHECK(rig.actions.rename(box, " \t Box\n ", error));
    CHECK(error.empty());
    CHECK(rig.actions.rename(box, "  Box", error));
    CHECK(recorder.renamed == 0);
    rig.frames(3);
    CHECK(!rig.document.dirty());
    CHECK(box->getName() == "Box");

    CHECK(rig.actions.rename(box, "Crate", error));
    CHECK(error.empty());
    CHECK(box->getName() == "Crate");
    CHECK(recorder.renamed == 1);
    CHECK(rig.document.dirty());
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(box) && rig.hierarchy.itemOf(box)->text() == "Crate");
    commit(rig);

    CHECK(rig.actions.rename(box, "  Padded Name  ", error));
    CHECK(box->getName() == "Padded Name");
    CHECK(error.empty());
    CHECK(rig.actions.rename(box, "\t\n Tabbed\t", error));
    CHECK(box->getName() == "Tabbed");

    CHECK(rig.actions.rename(box, "a.b:c@d/e\"f%g", error));
    CHECK(error == kInvalidText);
    CHECK(box->getName() == "a_b_c_d_e_f_g");
    CHECK(rig.actions.rename(box, "  clean ", error));
    CHECK(error.empty());
    CHECK(box->getName() == "clean");
    CHECK(rig.actions.rename(box, " x.y ", error));
    CHECK(error == kInvalidText);
    CHECK(box->getName() == "x_y");

    CHECK(rig.actions.rename(box, "", error));
    CHECK(box->getName() == "Node3D");
    CHECK(error.empty());
    CHECK(rig.actions.rename(sphere, "   ", error));
    CHECK(sphere->getName() == "Node3D2");
    CHECK(error.empty());
    CHECK(rig.actions.rename(box, "Box", error));
    CHECK(rig.actions.rename(sphere, "Sphere", error));

    CHECK(rig.actions.rename(cube, "Sphere", error));
    CHECK(error.empty());
    CHECK(cube->getName() == "Sphere2");
    CHECK(sphere->getName() == "Sphere");
    CHECK(rig.actions.rename(cube, "Box", error));
    CHECK(cube->getName() == "Box2");
    CHECK(scene->findChild("Box2") == cube);
    CHECK(rig.actions.rename(cube, "Cube", error));
    CHECK(cube->getName() == "Cube");

    CHECK(!rig.actions.rename(hidden, "Shown", error));
    CHECK(hidden->getName() == "Hidden");
    CHECK(!rig.actions.rename(nullptr, "Shown", error));
    CHECK(!rig.actions.rename(rig.tree.getRoot(), "Shown", error));

    commit(rig);
    CHECK(rig.actions.rename(scene, "Level", error));
    CHECK(scene->getName() == "Level");
    CHECK(error.empty());
    rig.frames(3);
    CHECK(rig.document.dirty());
    CHECK(rig.hierarchy.itemOf(scene) && rig.hierarchy.itemOf(scene)->text() == "Level");
    commit(rig);
    CHECK(rig.actions.rename(scene, "Level", error));
    rig.frames(3);
    CHECK(!rig.document.dirty());

    CHECK(rig.actions.rename(child, "Leaf", error));
    CHECK(child->getName() == "Leaf");
    CHECK(rig.document.dirty());
    rig.tree.removeListener(&recorder);
}

void testRenameUi(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* box = add(scene, "Box");
    Node3D* sphere = add(scene, "Sphere");
    commit(rig);

    select(rig, box);
    CHECK(rig.actions.canRename());
    rig.actions.requestRename();
    rig.frames(3);
    rt::InputBox* boxWindow = rig.actions.renameBox();
    CHECK(boxWindow != nullptr);
    if (!boxWindow) return;
    CHECK(boxWindow->title() == "Rename Node");
    CHECK(boxWindow->text() == "Box");
    rig.actions.requestRename();
    rig.frames(2);
    CHECK(rig.actions.renameBox() == boxWindow);

    rt::WidgetApp& app = rt::WidgetApp::instance();
    CHECK(rt::widget_cast<rt::TextInput>(app.focused()) != nullptr);
    rig.typeText("Crate");
    CHECK(boxWindow->text() == "Crate");
    rig.tap(Key::Enter);
    CHECK(rig.actions.renameBox() == nullptr);
    CHECK(box->getName() == "Crate");
    CHECK(popupDialog() == nullptr);
    CHECK(rig.document.dirty());
    commit(rig);

    select(rig, sphere);
    rig.actions.requestRename();
    rig.frames(3);
    rig.typeText("bad.name@here");
    rig.tap(Key::Enter);
    CHECK(rig.actions.renameBox() == nullptr);
    CHECK(sphere->getName() == "bad_name_here");
    rt::Dialog* alert = popupDialog();
    CHECK(alert != nullptr);
    if (alert)
    {
        CHECK(alert->message_ == kInvalidText);
        closePopup(rig);
    }
    CHECK(popupDialog() == nullptr);

    select(rig, sphere);
    rig.actions.requestRename();
    rig.frames(3);
    CHECK(rig.actions.renameBox() != nullptr);
    rig.tap(Key::Escape);
    CHECK(rig.actions.renameBox() == nullptr);
    CHECK(sphere->getName() == "bad_name_here");

    select(rig, sphere);
    rig.actions.requestRename();
    rig.frames(3);
    CHECK(rig.actions.renameBox() != nullptr);
    sphere->queueFree();
    rig.frames(4);
    rt::InputBox* orphan = rig.actions.renameBox();
    CHECK(orphan != nullptr);
    if (orphan)
    {
        rig.typeText("Ghost");
        rig.tap(Key::Enter);
    }
    CHECK(rig.actions.renameBox() == nullptr);
    CHECK(scene->findChild("Ghost") == nullptr);
    CHECK(popupDialog() == nullptr);
}

void testDuplicate(Rig& rig)
{
    MeshStore& meshes = *Main::meshes();
    Node* scene = freshScene(rig);
    MeshInstance3D* box = scene->createChild<MeshInstance3D>();
    box->setName("Box");
    box->setMesh(createBoxMesh(meshes, Math::Vec3(1.0f, 2.0f, 3.0f)));
    box->setColor(Math::Vec4(0.25f, 0.5f, 0.75f, 1.0f));
    box->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    Node3D* second = add(scene, "B");
    Node3D* third = add(scene, "C");
    Node3D* fourth = add(scene, "D");
    Node3D* nested = add(second, "Nested");
    Node3D* deep = add(nested, "Deep");
    commit(rig);

    ct::String error;
    select(rig, box);
    CHECK(rig.actions.duplicateSelection(error));
    CHECK(error.empty());
    CHECK(names(scene) == "Box,Box2,B,C,D");
    Node* copy = scene->findChild("Box2");
    CHECK(copy != nullptr && copy->getIndex() == 1);
    CHECK(copy && copy != box && sameProperties(box, copy));
    CHECK(copy && static_cast<MeshInstance3D*>(copy)->getMesh() == box->getMesh());
    CHECK(selectedIs(rig, copy));
    CHECK(rig.document.dirty());
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(copy) != nullptr);
    CHECK(rig.inspector.inspected() == copy);

    CHECK(rig.actions.duplicateSelection(error));
    CHECK(names(scene) == "Box,Box2,Box3,B,C,D");
    select(rig, box);
    CHECK(rig.actions.duplicateSelection(error));
    CHECK(names(scene) == "Box,Box4,Box2,Box3,B,C,D");
    commit(rig);

    select(rig, fourth, second);
    CHECK(rig.actions.duplicateSelection(error));
    CHECK(error.empty());
    CHECK(names(scene) == "Box,Box4,Box2,Box3,B,C,D,B2,D2");
    Node* secondCopy = scene->findChild("B2");
    Node* fourthCopy = scene->findChild("D2");
    CHECK(secondCopy && fourthCopy);
    CHECK(selectedIs(rig, secondCopy, fourthCopy));
    CHECK(secondCopy && secondCopy->getChildCount() == 1);
    CHECK(secondCopy && secondCopy->getNode("Nested/Deep") != nullptr);
    CHECK(secondCopy && secondCopy->getNode("Nested/Deep") != deep);
    commit(rig);

    select(rig, third, second);
    CHECK(rig.actions.duplicateSelection(error));
    CHECK(names(scene) == "Box,Box4,Box2,Box3,B,C,B3,C2,D,B2,D2");
    CHECK(selectedIs(rig, scene->findChild("B3"), scene->findChild("C2")));
    commit(rig);

    select(rig, second, nested, deep);
    const std::size_t before = scene->getChildCount();
    CHECK(rig.actions.duplicateSelection(error));
    CHECK(scene->getChildCount() == before + 1);
    CHECK(second->getChildCount() == 1);
    Node* onlyTop = scene->findChild("B4");
    CHECK(onlyTop != nullptr && onlyTop->getChildCount() == 1);
    CHECK(selectedIs(rig, onlyTop));
    commit(rig);

    select(rig, nested, third);
    CHECK(rig.actions.duplicateSelection(error));
    CHECK(second->getChildCount() == 2);
    CHECK(names(second) == "Nested,Nested2");
    CHECK(scene->findChild("C3") != nullptr);
    CHECK(names(scene) == "Box,Box4,Box2,Box3,B,B4,C,C3,B3,C2,D,B2,D2");
    CHECK(selectedIs(rig, second->findChild("Nested2"), scene->findChild("C3")));
    commit(rig);

    select(rig, scene);
    const ct::String untouched = names(scene);
    CHECK(!rig.actions.duplicateSelection(error));
    CHECK(error == kRootText);
    CHECK(names(scene) == untouched);
    CHECK(selectedIs(rig, scene));
    select(rig, scene, third);
    CHECK(!rig.actions.duplicateSelection(error));
    CHECK(error == kRootText);
    CHECK(names(scene) == untouched);
    CHECK(!rig.document.dirty());

    select(rig, nullptr);
    CHECK(!rig.actions.duplicateSelection(error));
    CHECK(error.empty());
    CHECK(names(scene) == untouched);

    Node* hidden = third->createChild<Node3D>();
    hidden->setName("Hidden");
    hidden->setInternal(true);
    commit(rig);
    rig.selection.clear();
    rig.selection.add(hidden);
    CHECK(!rig.actions.duplicateSelection(error));
    CHECK(!rig.actions.canDuplicate());
    CHECK(names(scene) == untouched);
    CHECK(!rig.document.dirty());

    select(rig, third);
    CHECK(rig.actions.duplicateSelection(error));
    CHECK(rig.document.dirty());
    Node* thirdCopy = scene->findChild("C4");
    CHECK(thirdCopy != nullptr);
    CHECK(thirdCopy && thirdCopy->getChildCount() == 0);
}

void testDuplicatePaths(Rig& rig)
{
    Node* scene = freshScene(rig);
    Terrain3D* terrain = scene->createChild<Terrain3D>();
    terrain->setName("terrain");
    Node3D* anchor = add(scene, "Anchor");

    Node3D* branch = add(scene, "Branch");
    Terrain3D* inner = branch->createChild<Terrain3D>();
    inner->setName("inner");
    Forest3D* outer = branch->createChild<Forest3D>();
    outer->setName("outer");
    outer->setTerrain(terrain);
    Forest3D* local = branch->createChild<Forest3D>();
    local->setName("local");
    local->setTerrain(inner);
    Node3D* arm = add(branch, "Arm");
    PinJoint3D* pin = arm->createChild<PinJoint3D>();
    pin->setName("pin");
    pin->setNodeA("../../../Anchor");
    CHECK(outer->getTerrainPath() == "../../terrain");
    CHECK(local->getTerrainPath() == "../inner");
    CHECK(pin->getNode(pin->getNodeA()) == anchor);

    Forest3D* loose = scene->createChild<Forest3D>();
    loose->setName("loose");
    loose->setTerrain(terrain);
    commit(rig);

    ct::String error;
    select(rig, branch);
    CHECK(rig.actions.duplicateSelection(error));
    Node* copy = scene->findChild("Branch2");
    CHECK(copy != nullptr);
    if (copy)
    {
        Forest3D* copyOuter = static_cast<Forest3D*>(copy->findChild("outer"));
        Forest3D* copyLocal = static_cast<Forest3D*>(copy->findChild("local"));
        Node* copyInner = copy->findChild("inner");
        CHECK(copyOuter && copyOuter->getTerrain() == terrain);
        CHECK(copyOuter && copyOuter->getTerrainPath() == "../../terrain");
        CHECK(copyLocal && copyInner && copyLocal->getTerrain() == copyInner);
        CHECK(copyLocal && copyLocal->getTerrain() != inner);
        CHECK(copyLocal && copyLocal->getTerrainPath() == "../inner");
        PinJoint3D* copyPin = static_cast<PinJoint3D*>(copy->getNode("Arm/pin"));
        CHECK(copyPin != nullptr);
        CHECK(copyPin && copyPin->getNodeA() == "../../../Anchor");
        CHECK(copyPin && copyPin->getNode(copyPin->getNodeA()) == anchor);
    }
    commit(rig);

    select(rig, loose);
    CHECK(rig.actions.duplicateSelection(error));
    Forest3D* looseCopy = static_cast<Forest3D*>(scene->findChild("loose2"));
    CHECK(looseCopy != nullptr);
    CHECK(looseCopy && looseCopy->getTerrain() == terrain);
    CHECK(looseCopy && looseCopy->getTerrainPath() == "../terrain");
    commit(rig);

    select(rig, local);
    CHECK(rig.actions.duplicateSelection(error));
    Forest3D* localCopy = static_cast<Forest3D*>(branch->findChild("local2"));
    CHECK(localCopy != nullptr);
    CHECK(localCopy && localCopy->getTerrain() == inner);
    CHECK(localCopy && localCopy->getTerrainPath() == "../inner");
}

void testDuplicatePrefab(Rig& rig)
{
    const ct::String path = testRoot() + "/mast.prefab.json";
    Prefab::clearCache();
    {
        Node3D scratch;
        OmniLight3D* lamp = scratch.createChild<OmniLight3D>();
        lamp->setName("lamp");
        lamp->setEnergy(3.5f);
        Node3D* pole = lamp->createChild<Node3D>();
        pole->setName("pole");
        SceneResult saved;
        CHECK(Prefab::saveFromNode(lamp, path, saved));
        CHECK(saved.success());
    }

    Node* scene = freshScene(rig);
    SceneResult placed;
    Node* instance = Prefab::instantiate(path, scene, placed);
    CHECK(instance != nullptr && placed.success());
    if (!instance) return;
    instance->setName("mast");
    static_cast<OmniLight3D*>(instance)->setEnergy(9.0f);
    static_cast<Node3D*>(instance)->setPosition(Math::Vec3(4.0f, 0.0f, 1.0f));
    CHECK(instance->isInstance());
    commit(rig);

    ct::String error;
    select(rig, instance);
    CHECK(rig.actions.duplicateSelection(error));
    Node* copy = scene->findChild("mast2");
    CHECK(copy != nullptr);
    if (!copy) return;
    CHECK(copy->isInstance());
    CHECK(copy->getSceneFilePath() == path);
    CHECK(copy->getSceneFilePath() == instance->getSceneFilePath());
    CHECK(strcmp(copy->getClassName(), "OmniLight3D") == 0);
    CHECK(static_cast<OmniLight3D*>(copy)->getEnergy() == 9.0f);
    CHECK(static_cast<Node3D*>(copy)->getPosition().x == 4.0f);
    CHECK(copy->getChildCount() == instance->getChildCount());
    CHECK(copy->findChild("pole") != nullptr);
    CHECK(copy->findChild("pole") != instance->findChild("pole"));
    CHECK(rig.document.dirty());
}

void testForeign(Rig& rig)
{
    const ct::String path = testRoot() + "/crate.prefab.json";
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

    Node* scene = freshScene(rig);
    SceneResult placed;
    Node* instance = Prefab::instantiate(path, scene, placed);
    CHECK(instance != nullptr);
    if (!instance) return;
    Node* lid = instance->findChild("lid");
    Node* base = instance->findChild("base");
    CHECK(lid != nullptr && base != nullptr);
    if (!lid || !base) return;
    CHECK(lid->getOwner() == instance);
    commit(rig);

    ct::String error;
    CHECK(!rig.actions.rename(lid, "cap", error));
    CHECK(error == HierarchyActions::kForeignRefusal);
    CHECK(lid->getName() == "lid");

    select(rig, base);
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(error == HierarchyActions::kForeignRefusal);
    CHECK(base->getIndex() == 1);

    CHECK(!rig.actions.deleteSelection(error));
    CHECK(error == HierarchyActions::kForeignRefusal);
    rig.frames(2);
    CHECK(base->getParent() == instance);

    CHECK(rig.actions.duplicateSelection(error));
    CHECK(error.empty());
    CHECK(instance->getChildCount() == 3);

    select(rig, instance);
    CHECK(rig.actions.rename(instance, "box", error));
    CHECK(error.empty());
    CHECK(instance->getName() == "box");
}

void testDelete(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* parent = add(scene, "Parent");
    Node3D* childA = add(parent, "ChildA");
    add(parent, "ChildB");
    Node3D* leaf = add(scene, "Leaf");
    Node3D* other = add(scene, "Other");
    Node3D* lonely = add(scene, "Lonely");
    Node* hidden = lonely->createChild<Node3D>();
    hidden->setName("Hidden");
    hidden->setInternal(true);
    commit(rig);

    select(rig, parent);
    CHECK(rig.actions.deleteMessage() == "Delete node \"Parent\" and its children?");
    select(rig, leaf);
    CHECK(rig.actions.deleteMessage() == "Delete node \"Leaf\"?");
    select(rig, lonely);
    CHECK(rig.actions.deleteMessage() == "Delete node \"Lonely\"?");
    select(rig, leaf, other);
    CHECK(rig.actions.deleteMessage() == "Delete 2 nodes?");
    select(rig, parent, leaf);
    CHECK(rig.actions.deleteMessage() == "Delete 2 nodes and any children?");
    select(rig, parent, childA, leaf);
    CHECK(rig.actions.deleteMessage() == "Delete 2 nodes and any children?");
    select(rig, leaf, other, lonely);
    CHECK(rig.actions.deleteMessage() == "Delete 3 nodes?");

    const ct::String mastPath = testRoot() + "/pole.prefab.json";
    Prefab::clearCache();
    {
        Node3D scratch;
        Node3D* root = scratch.createChild<Node3D>();
        root->setName("pole");
        add(root, "top");
        SceneResult saved;
        CHECK(Prefab::saveFromNode(root, mastPath, saved));
    }
    SceneResult placed;
    Node* instance = Prefab::instantiate(mastPath, scene, placed);
    CHECK(instance != nullptr);
    if (instance)
    {
        instance->setName("pole1");
        CHECK(instance->getChildCount() > 0 && instance->isInstance());
        select(rig, instance);
        CHECK(rig.actions.deleteMessage() == "Delete node \"pole1\"?");
        commit(rig);
    }

    select(rig, parent);
    rig.actions.requestDelete(true);
    rig.frames(3);
    rt::Dialog* dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_ == "Delete node \"Parent\" and its children?");
        CHECK(dialog->buttons_.size() == 2);
        CHECK(dialog->buttons_[0].label == "Delete");
        CHECK(dialog->buttons_[1].label == "Cancel");
        clickDialogButton(rig, dialog, 1);
    }
    CHECK(popupDialog() == nullptr);
    rig.frames(3);
    CHECK(parent->getParent() == scene);
    CHECK(rig.selection.isSelected(parent));
    CHECK(!rig.document.dirty());

    rig.actions.requestDelete(true);
    rig.frames(3);
    CHECK(popupDialog() != nullptr);
    rig.actions.requestDelete(true);
    rig.frames(2);
    rig.tap(Key::Escape);
    CHECK(popupDialog() == nullptr);
    CHECK(parent->getParent() == scene);

    Recorder recorder;
    rig.tree.addListener(&recorder);
    select(rig, childA, parent);
    rig.actions.requestDelete(true);
    rig.frames(3);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog) clickDialogButton(rig, dialog, 0);
    CHECK(popupDialog() == nullptr);
    CHECK(rig.selection.selectedNodes().size() == 0);
    rig.frames(6);
    CHECK(recorder.removed == 3);
    CHECK(scene->findChild("Parent") == nullptr);
    CHECK(names(scene).find("Parent") == ct::String::npos);
    CHECK(rig.document.dirty());
    CHECK(rig.inspector.inspected() == nullptr);
    CHECK(rig.status.text(EditorStatusBar::kSelection) == "No selection");
    CHECK(rig.hierarchy.itemOf(scene) != nullptr);
    CHECK(rig.hierarchy.itemOf(leaf) != nullptr);
    rig.tree.removeListener(&recorder);
    commit(rig);

    select(rig, leaf);
    rig.actions.requestDelete(false);
    CHECK(popupDialog() == nullptr);
    CHECK(rig.selection.selectedNodes().size() == 0);
    rig.frames(6);
    CHECK(scene->findChild("Leaf") == nullptr);
    CHECK(rig.document.dirty());
    commit(rig);

    select(rig, scene);
    ct::String error;
    CHECK(!rig.actions.deleteSelection(error));
    CHECK(error == kRootText);
    CHECK(rig.selection.isSelected(scene));
    rig.actions.requestDelete(true);
    rig.frames(3);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_ == kRootText);
        closePopup(rig);
    }
    rig.actions.requestDelete(false);
    rig.frames(3);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog) closePopup(rig);
    select(rig, scene, other);
    CHECK(!rig.actions.deleteSelection(error));
    CHECK(error == kRootText);
    rig.frames(3);
    CHECK(other->getParent() == scene);
    CHECK(rig.selection.selectedNodes().size() == 2);
    CHECK(!rig.document.dirty());
    CHECK(!rig.actions.canDelete());

    rig.selection.clear();
    rig.selection.add(hidden);
    CHECK(!rig.actions.canDelete());
    CHECK(!rig.actions.deleteSelection(error));
    CHECK(error.empty());
    rig.frames(3);
    CHECK(hidden->getParent() == lonely);

    select(rig, other);
    rig.frames(2);
    CHECK(rig.inspector.inspected() == other);
    rig.actions.requestDelete(false);
    for (int i = 0; i < 8; ++i)
    {
        rig.frames(1);
        rig.inspector.update();
        rig.status.update(0.1f);
    }
    CHECK(rig.inspector.inspected() == nullptr);
    CHECK(scene->findChild("Other") == nullptr);
    CHECK(rig.panel.gizmo() != nullptr);
    std::size_t shown = 0;
    for (std::size_t i = 0; i < scene->getChildCount(); ++i)
    {
        if (scene->getChild(i)->isInternal()) continue;
        ++shown;
        CHECK(rig.hierarchy.itemOf(scene->getChild(i)) != nullptr);
    }
    CHECK(rig.hierarchy.itemOf(scene)->children().size() == shown);
}

void testMove(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    add(scene, "D");
    Node3D* e = add(scene, "E");
    Node3D* sub = add(a, "Sub");
    Node3D* subA = add(sub, "SubA");
    add(sub, "SubB");
    commit(rig);

    ct::String error;
    CHECK(names(scene) == "A,B,C,D,E");

    select(rig, c);
    CHECK(rig.actions.canMoveUp() && rig.actions.canMoveDown());
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(error.empty());
    CHECK(names(scene) == "A,C,B,D,E");
    CHECK(rig.document.dirty());
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(scene)->children()[1]->text() == "C");
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,B,C,D,E");
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,B,D,C,E");
    CHECK(selectedIs(rig, c));
    commit(rig);

    select(rig, a);
    CHECK(!rig.actions.canMoveUp());
    CHECK(rig.actions.canMoveDown());
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(error.empty());
    CHECK(names(scene) == "A,B,D,C,E");
    rig.frames(3);
    CHECK(!rig.document.dirty());
    select(rig, e);
    CHECK(rig.actions.canMoveUp());
    CHECK(!rig.actions.canMoveDown());
    CHECK(!rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,B,D,C,E");
    rig.frames(3);
    CHECK(!rig.document.dirty());

    select(rig, a, b, c);
    CHECK(!rig.actions.canMoveUp());
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(names(scene) == "A,B,D,C,E");
    select(rig, e, c);
    CHECK(!rig.actions.canMoveDown());
    CHECK(!rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,B,D,C,E");
    rig.frames(3);
    CHECK(!rig.document.dirty());

    select(rig, c, b);
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(names(scene) == "B,A,C,D,E");
    CHECK(rig.document.dirty());
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,B,D,C,E");
    CHECK(selectedIs(rig, b, c));
    commit(rig);

    select(rig, subA, c);
    CHECK(!rig.actions.canMoveUp());
    CHECK(rig.actions.canMoveDown());
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(sub) == "SubB,SubA");
    CHECK(names(scene) == "A,B,D,E,C");
    CHECK(selectedIs(rig, subA, c));
    commit(rig);
    CHECK(!rig.actions.canMoveDown());
    CHECK(!rig.actions.moveSelection(true, error));
    CHECK(rig.actions.canMoveUp());

    select(rig, scene);
    CHECK(!rig.actions.canMoveUp() && !rig.actions.canMoveDown());
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(error == kRootText);
    CHECK(!rig.actions.moveSelection(true, error));
    CHECK(error == kRootText);
    select(rig, scene, b);
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(error == kRootText);
    rig.actions.requestMove(false);
    rig.frames(3);
    rt::Dialog* alert = popupDialog();
    CHECK(alert != nullptr);
    if (alert)
    {
        CHECK(alert->message_ == kRootText);
        closePopup(rig);
    }
    CHECK(names(scene) == "A,B,D,E,C");
    CHECK(!rig.document.dirty());

    rig.selection.clear();
    rig.frames(3);
    CHECK(!rig.actions.canMoveUp() && !rig.actions.canMoveDown());
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(error.empty());
}

void testMoveOrder(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    Node3D* d = add(scene, "D");
    Node3D* e = add(scene, "E");
    Node3D* f = add(scene, "F");
    commit(rig);
    ct::String error;

    select(rig, c, b);
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(names(scene) == "B,C,A,D,E,F");
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,B,C,D,E,F");
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,D,B,C,E,F");
    commit(rig);

    select(rig, f, d);
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(names(scene) == "D,A,B,C,F,E");
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "A,D,B,C,E,F");
    commit(rig);

    select(rig, c, a);
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "D,A,B,E,C,F");
    commit(rig);

    select(rig, a, b, c);
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(scene) == "D,E,A,B,F,C");
    commit(rig);

    Node3D* group = add(scene, "G");
    Node3D* g1 = add(group, "G1");
    Node3D* g2 = add(group, "G2");
    Node3D* g3 = add(group, "G3");
    commit(rig);
    select(rig, g2, f);
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(names(group) == "G2,G1,G3");
    CHECK(names(scene) == "D,E,A,F,B,C,G");
    commit(rig);

    select(rig, g2, e);
    CHECK(rig.actions.canMoveDown());
    CHECK(!rig.actions.canMoveUp());
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(names(group) == "G2,G1,G3");
    CHECK(names(scene) == "D,E,A,F,B,C,G");
    select(rig, g3, g1);
    CHECK(!rig.actions.canMoveDown());
    CHECK(rig.actions.canMoveUp());
    rig.frames(3);
    CHECK(!rig.document.dirty());
}

void testMoveInternal(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* parent = add(scene, "Parent");
    Node3D* first = add(parent, "A");
    Node* hiddenOne = parent->createChild<Node3D>();
    hiddenOne->setName("I1");
    hiddenOne->setInternal(true);
    Node3D* second = add(parent, "B");
    Node* hiddenTwo = parent->createChild<Node3D>();
    hiddenTwo->setName("I2");
    hiddenTwo->setInternal(true);
    Node* hiddenThree = parent->createChild<Node3D>();
    hiddenThree->setName("I3");
    hiddenThree->setInternal(true);
    Node3D* third = add(parent, "C");
    Node* hiddenFour = parent->createChild<Node3D>();
    hiddenFour->setName("I4");
    hiddenFour->setInternal(true);
    commit(rig);
    ct::String error;

    CHECK(parent->getChildCount() == 7);
    CHECK(names(parent) == "A,B,C");

    select(rig, first);
    CHECK(!rig.actions.canMoveUp());
    CHECK(rig.actions.canMoveDown());
    select(rig, third);
    CHECK(rig.actions.canMoveUp());
    CHECK(!rig.actions.canMoveDown());

    select(rig, second);
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(names(parent) == "B,A,C");
    CHECK(second->getIndex() == 0);
    CHECK(parent->getChild(1) == first);
    CHECK(parent->getChild(2) == hiddenOne);
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(parent) == "A,B,C");
    CHECK(second->getIndex() == 1);
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(names(parent) == "B,A,C");
    CHECK(!rig.actions.moveSelection(false, error));
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(parent) == "A,C,B");
    CHECK(!rig.actions.moveSelection(true, error));
    CHECK(names(parent) == "A,C,B");
    CHECK(parent->getChildCount() == 7);

    select(rig, third);
    CHECK(rig.actions.moveSelection(false, error));
    CHECK(names(parent) == "C,A,B");
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(parent) == "A,B,C");
    CHECK(third->getIndex() > second->getIndex());

    select(rig, first, third);
    CHECK(!rig.actions.canMoveUp());
    CHECK(!rig.actions.canMoveDown());
    select(rig, first, second);
    CHECK(rig.actions.moveSelection(true, error));
    CHECK(names(parent) == "C,A,B");
    CHECK(rig.hierarchy.itemOf(parent)->children().size() == 3);
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(parent)->children()[0]->text() == "C");
    CHECK(rig.hierarchy.itemOf(parent)->children()[1]->text() == "A");
    CHECK(rig.hierarchy.itemOf(parent)->children()[2]->text() == "B");
}

void testMenus(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    Node3D* hidden = a->createChild<Node3D>();
    hidden->setName("Hidden");
    hidden->setInternal(true);
    commit(rig);

    rt::Menu* edit = rig.menus.menu(EditorMenus::kEdit);
    rt::Menu* context = rig.actions.contextMenu();
    CHECK(edit != nullptr && context != nullptr);
    if (!edit || !context) return;
    CHECK(strcmp(EditorMenus::menuTitle(EditorMenus::kEdit), "Edit") == 0);
    CHECK(rig.shell.menuBar()->menu(1) == edit);
    CHECK(edit->actionCount() == EditorMenus::kEditItemCount);
    CHECK(context->actionCount() ==
            HierarchyActions::kItemCount + HierarchyActions::kContextOffset);
    CHECK(rig.hierarchy.outliner()->contextMenu() == context);

    const char* const labels[] = { "Add Child Node...", "", "Duplicate", "", "Rename", "",
        "Move Up", "Move Down", "Reparent...", "Reparent to New Node...", "Save as Prefab...",
        "Create Static Collision", "", "Delete" };
    const char* const shortcuts[] = { "Ctrl+A", "", "Ctrl+D", "", "F2", "", "Ctrl+Up", "Ctrl+Down",
        "", "", "", "", "", "Del" };
    static_assert(sizeof(labels) / sizeof(labels[0]) == HierarchyActions::kItemCount,
            "one label per menu item");
    static_assert(sizeof(shortcuts) / sizeof(shortcuts[0]) == HierarchyActions::kItemCount,
            "one shortcut per menu item");
    const auto separator = [](int i)
    {
        return i == HierarchyActions::kItemSeparator1 || i == HierarchyActions::kItemSeparator2 ||
               i == HierarchyActions::kItemSeparator3 || i == HierarchyActions::kItemSeparator4;
    };
    for (int i = 0; i < HierarchyActions::kItemCount; ++i)
    {
        for (rt::Menu* menu: { edit, context })
        {
            const rt::MenuAction* item =
                    menu->action(i + (menu == context ? HierarchyActions::kContextOffset
                                                      : EditorMenus::kEditHeader));
            if (separator(i))
            {
                CHECK(item->isSeparator());
                continue;
            }
            CHECK(!item->isSeparator());
            CHECK(item->text() == labels[i]);
            CHECK(item->shortcut() == shortcuts[i]);
        }
    }

    const auto expect = [&](bool rename, bool duplicate, bool up, bool down, bool remove, int line)
    {
        rig.frames(2);
        const bool wanted[] = { true, true, duplicate, true, rename, true, up, down, duplicate,
            duplicate, rig.actions.canSaveAsPrefab(), rig.actions.canCreateStaticCollision(), true,
            remove };
        static_assert(sizeof(wanted) / sizeof(wanted[0]) == HierarchyActions::kItemCount,
                "one state per menu item");
        for (int i = 0; i < HierarchyActions::kItemCount; ++i)
        {
            if (separator(i)) continue;
            if (edit->action(i + EditorMenus::kEditHeader)->isEnabled() != wanted[i] ||
                    context->action(i + HierarchyActions::kContextOffset)->isEnabled() != wanted[i])
            {
                printf("FAIL menu state line %d item %d\n", line, i);
                ++failures;
            }
        }
        CHECK(rig.actions.canAddChild());
        CHECK(rig.actions.canRename() == rename);
        CHECK(rig.actions.canDuplicate() == duplicate);
        CHECK(rig.actions.canMoveUp() == up);
        CHECK(rig.actions.canMoveDown() == down);
        CHECK(rig.actions.canReparent() == duplicate);
        CHECK(rig.actions.canReparentToNew() == duplicate);
        CHECK(rig.actions.canDelete() == remove);
    };

    select(rig, nullptr);
    expect(false, false, false, false, false, __LINE__);
    select(rig, a);
    expect(true, true, false, true, true, __LINE__);
    select(rig, b);
    expect(true, true, true, true, true, __LINE__);
    select(rig, c);
    expect(true, true, true, false, true, __LINE__);
    select(rig, a, b);
    expect(false, true, false, true, true, __LINE__);
    select(rig, b, c);
    expect(false, true, true, false, true, __LINE__);
    select(rig, a, c);
    expect(false, true, false, false, true, __LINE__);
    select(rig, scene);
    expect(true, false, false, false, false, __LINE__);
    select(rig, scene, b);
    expect(false, false, false, false, false, __LINE__);
    rig.selection.clear();
    rig.selection.add(hidden);
    rig.frames(3);
    expect(false, false, false, false, false, __LINE__);

    select(rig, b);
    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditDuplicate);
    CHECK(names(scene) == "A,B,B2,C");
    CHECK(selectedIs(rig, scene->findChild("B2")));
    CHECK(rt::WidgetApp::instance().popup() == nullptr);
    commit(rig);

    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditMoveUp);
    CHECK(names(scene) == "A,B2,B,C");
    CHECK(rig.document.dirty());
    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditMoveDown);
    CHECK(names(scene) == "A,B,B2,C");
    commit(rig);

    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditRename);
    CHECK(rig.actions.renameBox() != nullptr);
    if (rig.actions.renameBox())
    {
        CHECK(rig.actions.renameBox()->text() == "B2");
        rig.typeText("Renamed");
        rig.tap(Key::Enter);
        CHECK(scene->findChild("Renamed") != nullptr);
    }
    commit(rig);

    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditDelete);
    rt::Dialog* dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_ == "Delete node \"Renamed\"?");
        clickDialogButton(rig, dialog, 0);
    }
    rig.frames(4);
    CHECK(names(scene) == "A,B,C");
    commit(rig);

    select(rig, b);
    rig.click(textX(rig, c), rowY(rig, c), MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == context);
    CHECK(selectedIs(rig, c));
    expect(true, true, true, false, true, __LINE__);
    clickMenuItem(rig, context,
            HierarchyActions::kItemDuplicate + HierarchyActions::kContextOffset);
    CHECK(names(scene) == "A,B,C,C2");
    CHECK(selectedIs(rig, scene->findChild("C2")));
    commit(rig);

    select(rig, a);
    rig.click(textX(rig, b), rowY(rig, b), MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == context);
    CHECK(selectedIs(rig, b));
    clickMenuItem(rig, context, HierarchyActions::kItemMoveUp + HierarchyActions::kContextOffset);
    CHECK(names(scene) == "B,A,C,C2");
    CHECK(selectedIs(rig, b));
    commit(rig);

    rig.click(textX(rig, a), rowY(rig, a), MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == context);
    CHECK(selectedIs(rig, a));
    clickMenuItem(rig, context, HierarchyActions::kItemRename + HierarchyActions::kContextOffset);
    CHECK(rig.actions.renameBox() != nullptr);
    rig.tap(Key::Escape);
    CHECK(rig.actions.renameBox() == nullptr);

    rig.click(textX(rig, scene), rowY(rig, scene), MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == context);
    CHECK(selectedIs(rig, scene));
    CHECK(context->action(HierarchyActions::kItemRename + HierarchyActions::kContextOffset)
                    ->isEnabled());
    CHECK(!context->action(HierarchyActions::kItemDuplicate + HierarchyActions::kContextOffset)
                    ->isEnabled());
    closePopup(rig);
}

void testShortcuts(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    commit(rig);

    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::TextInput* input = rig.shell.panel(3)->createChild<rt::TextInput>();
    rt::SpinBox* spin = rig.shell.panel(3)->createChild<rt::SpinBox>();
    NumberField* number = rig.shell.panel(3)->createChild<NumberField>("X");
    rt::Button* pressable = rig.shell.panel(3)->createChild<rt::Button>("Press");
    rig.frames(3);

    select(rig, b);
    app.setFocused(nullptr);
    CHECK(rig.tap(Key::F2));
    CHECK(rig.actions.renameBox() != nullptr);
    rig.tap(Key::Escape);
    CHECK(rig.actions.renameBox() == nullptr);

    CHECK(!rig.tap(Key::F2, 0, true));
    CHECK(rig.actions.renameBox() == nullptr);

    CHECK(rig.tap(Key::D, kModControl));
    CHECK(names(scene) == "A,B,B2,C");
    CHECK(selectedIs(rig, scene->findChild("B2")));
    CHECK(rig.tap(Key::D, kModControl, true));
    CHECK(names(scene) == "A,B,B2,B3,C");
    CHECK(rig.document.dirty());
    commit(rig);

    CHECK(!rig.tap(Key::D, 0));
    CHECK(!rig.tap(Key::D, kModControl | kModShift));
    CHECK(!rig.tap(Key::D, kModControl | kModAlt));
    CHECK(names(scene) == "A,B,B2,B3,C");

    select(rig, c);
    CHECK(rig.tap(Key::Up, kModControl));
    CHECK(names(scene) == "A,B,B2,C,B3");
    CHECK(rig.tap(Key::Up, kModControl, true));
    CHECK(names(scene) == "A,B,C,B2,B3");
    CHECK(rig.tap(Key::Down, kModControl));
    CHECK(names(scene) == "A,B,B2,C,B3");
    CHECK(rig.tap(Key::Down, kModControl, true));
    CHECK(names(scene) == "A,B,B2,B3,C");
    CHECK(rig.document.dirty());
    commit(rig);
    CHECK(rig.tap(Key::Down, kModControl));
    CHECK(names(scene) == "A,B,B2,B3,C");
    CHECK(!rig.document.dirty());
    CHECK(!rig.tap(Key::Up, 0));
    CHECK(!rig.tap(Key::Up, kModShift));

    select(rig, a);
    CHECK(rig.tap(Key::Delete, 0, true) == false);
    CHECK(popupDialog() == nullptr);
    CHECK(rig.tap(Key::Delete));
    rt::Dialog* dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_ == "Delete node \"A\"?");
        clickDialogButton(rig, dialog, 1);
    }
    CHECK(a->getParent() == scene);
    CHECK(rig.tap(Key::Delete));
    CHECK(popupDialog() != nullptr);
    CHECK(!rig.tap(Key::F2));
    CHECK(!rig.tap(Key::D, kModControl));
    CHECK(rig.actions.renameBox() == nullptr);
    CHECK(names(scene) == "A,B,B2,B3,C");
    rig.tap(Key::Escape);
    CHECK(popupDialog() == nullptr);

    CHECK(rig.tap(Key::Delete, kModShift));
    CHECK(popupDialog() == nullptr);
    rig.frames(6);
    CHECK(names(scene) == "B,B2,B3,C");
    CHECK(rig.selection.selectedNodes().size() == 0);
    CHECK(rig.document.dirty());
    commit(rig);

    CHECK(!rig.tap(Key::Delete));
    CHECK(!rig.tap(Key::F2));
    CHECK(!rig.tap(Key::D, kModControl));
    CHECK(!rig.tap(Key::Up, kModControl));

    select(rig, scene);
    CHECK(rig.tap(Key::Delete, kModShift));
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_ == kRootText);
        closePopup(rig);
    }
    CHECK(rig.tap(Key::D, kModControl));
    CHECK(popupDialog() != nullptr);
    closePopup(rig);
    CHECK(rig.tap(Key::Up, kModControl));
    CHECK(popupDialog() != nullptr);
    closePopup(rig);
    CHECK(rig.tap(Key::F2));
    CHECK(rig.actions.renameBox() != nullptr);
    rig.tap(Key::Escape);
    CHECK(names(scene) == "B,B2,B3,C");
    CHECK(!rig.document.dirty());

    select(rig, b);
    app.setFocused(input);
    CHECK(app.focused() == input);
    CHECK(!rig.tap(Key::F2));
    CHECK(!rig.tap(Key::D, kModControl));
    CHECK(!rig.tap(Key::Delete));
    CHECK(!rig.tap(Key::Delete, kModShift));
    CHECK(!rig.tap(Key::Up, kModControl));
    CHECK(!rig.tap(Key::Down, kModControl));
    CHECK(rig.actions.renameBox() == nullptr);
    CHECK(popupDialog() == nullptr);
    CHECK(names(scene) == "B,B2,B3,C");
    CHECK(!rig.document.dirty());

    app.setFocused(spin);
    CHECK(app.focused() == spin);
    CHECK(!rig.tap(Key::Delete, kModShift));
    CHECK(!rig.tap(Key::D, kModControl));
    CHECK(!rig.tap(Key::F2));
    CHECK(names(scene) == "B,B2,B3,C");

    number->beginEdit();
    CHECK(number->isEditing());
    CHECK(!rig.tap(Key::Delete, kModShift));
    CHECK(!rig.tap(Key::F2));
    CHECK(!rig.tap(Key::D, kModControl));
    CHECK(names(scene) == "B,B2,B3,C");
    number->cancelEdit();
    rig.frames(2);

    app.setFocused(pressable);
    CHECK(app.focused() == pressable);
    CHECK(!rig.tap(Key::F2));
    CHECK(rig.actions.renameBox() == nullptr);
    CHECK(rig.tap(Key::D, kModControl));
    CHECK(names(scene) == "B,B4,B2,B3,C");
    commit(rig);

    app.setFocused(rig.hierarchy.outliner());
    select(rig, c);
    CHECK(rig.tap(Key::F2));
    CHECK(rig.actions.renameBox() != nullptr);
    rig.tap(Key::Escape);
    app.setFocused(nullptr);

    app.setFocused(input);
    select(rig, c);
    CHECK(!rig.tap(Key::F2));
    CHECK(rig.actions.renameBox() == nullptr);
    app.setFocused(nullptr);
    rig.frames(2);
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor actions test";
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
        CHECK(!actions.attach());
        EditorMenus menus(shell, ui, panel, actions);
        CHECK(menus.attach());
        EditorStatusBar status(document, selection, server);
        CHECK(status.attach(shell.statusBar()));
        status.setGizmo(gizmo);

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, panel,
            actions, menus, status };
        rig.frames(4);

        CHECK(document.createProject(testRoot() + "/projects", "Actions"));
        rig.frames(3);
        CHECK(!document.dirty());

        testRename(rig);
        testRenameUi(rig);
        testDuplicate(rig);
        testDuplicatePaths(rig);
        testDuplicatePrefab(rig);
        testForeign(rig);
        testDelete(rig);
        testMove(rig);
        testMoveOrder(rig);
        testMoveInternal(rig);
        testMenus(rig);
        testShortcuts(rig);

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
