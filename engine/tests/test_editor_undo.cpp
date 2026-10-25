#include "Check.h"
#include "EditorIconCheck.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/CreateNodeDialog.h"
#include "prisma3d/editor/EditorCamera.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorGizmo.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorPlay.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/GamePanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/MaterialPanel.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/ReparentDialog.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/UndoRedo.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/DialogWidgets.hpp>
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
const float kPi = 3.14159265358979f;

bool useVulkan = false;
const char* gOnly = nullptr;

bool wanted(const char* name)
{
    return !gOnly || strcmp(gOnly, name) == 0;
}

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_undo_test_vulkan" : "/editor_undo_test";
    return root;
}

std::uint64_t gNow = 1000000;
std::uint64_t gStep = 5000;

std::uint64_t testClock()
{
    gNow += gStep;
    return gNow;
}

void expect(bool ok, const char* scenario, const char* what)
{
    if (ok) return;
    printf("FAIL [%s] %s\n", scenario, what);
    ++failures;
}

void expectSame(const ct::String& actual, const ct::String& wanted, const char* scenario,
        const char* what)
{
    if (actual == wanted) return;
    printf("FAIL [%s] %s\n", scenario, what);
    ++failures;
    std::size_t at = 0;
    while (at < actual.size() && at < wanted.size() && actual[at] == wanted[at]) ++at;
    const std::size_t from = at > 80 ? at - 80 : 0;
    printf("  actual: ...%s\n  wanted: ...%s\n", actual.substr(from, 200).c_str(),
            wanted.substr(from, 200).c_str());
}

InputEvent motion(int x, int y, unsigned mask = 0)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = 0;
    event.mouseMotion.relativeY = 0;
    event.mouseMotion.buttonMask = mask;
    return event;
}

InputEvent button(int x, int y, MouseButton which, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = which;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = modifiers;
    return event;
}

InputEvent key(Key which, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = which;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
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

const unsigned kLeftMask = 1u << static_cast<int>(MouseButton::Left);

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
    GamePanel& game;
    EditorPlay& play;
    int sceneCounter = 0;

    bool send(const InputEvent& event)
    {
        if (play.handleEvent(event)) return true;
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
        play.update();
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

    bool tap(Key which, unsigned modifiers = 0)
    {
        const bool consumed = send(key(which, true, modifiers));
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

    void drag(int fromX, int fromY, int toX, int toY)
    {
        send(motion(fromX, fromY));
        frames(2);
        send(button(fromX, fromY, MouseButton::Left, true));
        frames(2);
        for (int step = 1; step <= 6; ++step)
        {
            send(motion(fromX + (toX - fromX) * step / 6, fromY + (toY - fromY) * step / 6));
            frames(1);
        }
        frames(2);
        send(button(toX, toY, MouseButton::Left, false));
        frames(3);
    }

    EditorUndoRedoManager& history()
    {
        return document.undoRedo();
    }

    bool undoKey()
    {
        return tap(Key::Z, kModControl);
    }

    bool redoKey()
    {
        return tap(Key::Y, kModControl);
    }

    bool redoShiftKey()
    {
        return tap(Key::Z, kModControl | kModShift);
    }

    ct::String snap()
    {
        ct::String text;
        SceneResult result;
        if (!document.scene() || !SceneSerializer::save(document.scene(), text, &result))
            return ct::String("<no scene>");
        return text;
    }
};

Node3D* add(Node* parent, const char* name)
{
    Node3D* node = parent->createChild<Node3D>();
    node->setName(name);
    return node;
}

MeshInstance3D* addMesh(Node* parent, const char* name, const Math::Vec3& size)
{
    MeshInstance3D* node = parent->createChild<MeshInstance3D>();
    node->setName(name);
    node->setMesh(createBoxMesh(*Main::meshes(), size));
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

bool selectedIs(const Rig& rig, Node* a = nullptr, Node* b = nullptr, Node* c = nullptr)
{
    const std::size_t expected = (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0);
    if (rig.selection.selectedNodes().size() != expected) return false;
    return (!a || rig.selection.isSelected(a)) && (!b || rig.selection.isSelected(b)) &&
           (!c || rig.selection.isSelected(c));
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

void centerOf(const rt::Rect& rect, int& x, int& y)
{
    x = static_cast<int>(rect.x + rect.w * 0.5f);
    y = static_cast<int>(rect.y + rect.h * 0.5f);
}

rt::Dialog* popupDialog()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    return app ? rt::widget_cast<rt::Dialog>(app->popup()) : nullptr;
}

void clickDialogButton(Rig& rig, rt::Dialog* dialog, int index)
{
    CHECK(dialog != nullptr);
    if (!dialog) return;
    const rt::Rect rect = dialog->btnRect_(index);
    rig.click(static_cast<int>(rect.x + rect.w * 0.5f), static_cast<int>(rect.y + rect.h * 0.5f));
}

void reveal(Rig& rig, const rt::Widget* widget)
{
    rt::ScrollView* scroll = rig.inspector.scrollView();
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
        rig.frames(2);
    }
}

void typeInto(Rig& rig, NumberField* field, const char* text)
{
    reveal(rig, field);
    rig.click(centerX(field), centerY(field));
    CHECK(field->isEditing());
    rig.typeText(text);
    rig.tap(Key::Enter);
}

void dragField(Rig& rig, NumberField* field, int dx, int steps)
{
    const int x = centerX(field);
    const int y = centerY(field);
    rig.send(motion(x, y));
    rig.frames(1);
    rig.send(button(x, y, MouseButton::Left, true));
    rig.frames(1);
    for (int i = 1; i <= steps; ++i)
    {
        rig.send(motion(x + dx * i / steps, y));
        rig.frames(1);
    }
    rig.send(button(x + dx, y, MouseButton::Left, false));
    rig.frames(2);
}

void commitText(Rig& rig, CommitInput* input, const char* text)
{
    reveal(rig, input);
    rig.click(centerX(input), centerY(input));
    input->selectAll();
    if (text[0]) rig.typeText(text);
    else
        rig.tap(Key::Backspace);
    rig.tap(Key::Enter);
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

int eyeX(Rig& rig)
{
    const rt::Rect area = rig.hierarchy.outliner()->absoluteRect();
    return static_cast<int>(area.x + area.w) - 40 + 8;
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

ct::String absolute(Rig& rig, const ct::String& relative)
{
    return rig.document.project().absolutePath(relative);
}

typedef ct::Function<void(bool)> Extra;

void roundTrip(Rig& rig, const char* name, const ct::String& before, const ct::String& after,
        const Extra& extra)
{
    EditorUndoRedoManager& undo = rig.history();
    const std::size_t count = undo.undoCount();
    expect(undo.currentActionName() == name, name, "the action has the expected name");
    expect(rig.document.dirty(), name, "the action marks the scene unsaved");
    expect(rig.snap() == after, name, "the scene after the action is the expected one");
    expect(rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditUndo)->text() ==
                    ct::String("Undo ") + name,
            name, "the Edit menu names the action");
    if (extra) extra(true);
    for (int round = 0; round < 3; ++round)
    {
        expect(rig.undoKey(), name, "Ctrl+Z is consumed");
        expectSame(rig.snap(), before, name, "undo restores the scene exactly");
        expect(!rig.document.dirty(), name, "undoing back to the saved state clears Unsaved");
        expect(undo.redoActionName() == name, name, "the redo entry carries the action name");
        expect(undo.undoCount() == count - 1, name, "undo pops one entry");
        expect(rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditRedo)->text() ==
                        ct::String("Redo ") + name,
                name, "the Edit menu names the redo");
        if (extra) extra(false);
        if (round == 0) expect(rig.redoKey(), name, "Ctrl+Y is consumed");
        else if (round == 1)
            expect(rig.redoShiftKey(), name, "Ctrl+Shift+Z is consumed");
        else
            expect(rig.redoKey(), name, "Ctrl+Y is consumed again");
        expectSame(rig.snap(), after, name, "redo reapplies the action exactly");
        expect(rig.document.dirty(), name, "redo marks the scene unsaved again");
        expect(undo.undoCount() == count, name, "redo pushes the entry back");
        if (extra) extra(true);
    }
}

struct Order
{
    ct::Vector<int> log;

    ct::String text() const
    {
        ct::String result;
        for (std::size_t i = 0; i < log.size(); ++i)
        {
            if (i > 0) result += ",";
            result.append_number(log[i]);
        }
        return result;
    }
};

struct Witness
{
    ct::Vector<Node*> freed;

    bool has(const Node* node) const
    {
        for (std::size_t i = 0; i < freed.size(); ++i)
            if (freed[i] == node) return true;
        return false;
    }
};

Node3D* detached()
{
    Node3D holder;
    Node3D* node = holder.createChild<Node3D>();
    holder.removeChild(node);
    return node;
}

void testCoreOrder()
{
    UndoRedo history;
    history.setClock([] { return testClock(); });
    Order order;
    Order* log = &order;

    history.createAction("First");
    history.addDoMethod([log] { log->log.push_back(1); });
    history.addDoMethod([log] { log->log.push_back(2); });
    history.addUndoMethod([log] { log->log.push_back(10); });
    history.addUndoMethod([log] { log->log.push_back(20); });
    CHECK(history.getActionLevel() == 1);
    history.commitAction();
    CHECK(history.getActionLevel() == 0);
    CHECK(order.text() == "1,2");
    CHECK(history.hasUndo() && !history.hasRedo());
    CHECK(history.getCurrentActionName() == "First");
    CHECK(history.getHistoryCount() == 1 && history.getCurrentAction() == 0);

    CHECK(history.undo());
    CHECK(order.text() == "1,2,10,20");
    CHECK(!history.hasUndo() && history.hasRedo());
    CHECK(history.getNextActionName() == "First");
    CHECK(history.redo());
    CHECK(order.text() == "1,2,10,20,1,2");
    CHECK(!history.redo());

    order.log.clear();
    history.createAction("Backward", UndoRedo::kMergeDisable, true);
    history.addDoMethod([log] { log->log.push_back(1); });
    history.addUndoMethod([log] { log->log.push_back(10); });
    history.addUndoMethod([log] { log->log.push_back(20); });
    history.addUndoMethod([log] { log->log.push_back(30); });
    history.commitAction();
    CHECK(history.undo());
    CHECK(order.text() == "1,30,20,10");

    order.log.clear();
    history.createAction("Not executed");
    history.addDoMethod([log] { log->log.push_back(7); });
    history.addUndoMethod([log] { log->log.push_back(8); });
    history.commitAction(false);
    CHECK(order.text() == "");
    CHECK(history.getCurrentActionName() == "Not executed");
    CHECK(history.undo() && order.text() == "8");
    CHECK(history.redo() && order.text() == "8,7");

    order.log.clear();
    history.createAction("Nested");
    history.createAction("ignored name");
    history.addDoMethod([log] { log->log.push_back(1); });
    history.commitAction();
    CHECK(order.text() == "");
    CHECK(history.getActionLevel() == 1);
    CHECK(!history.undo());
    history.commitAction();
    CHECK(order.text() == "1");
    CHECK(history.getCurrentActionName() == "Nested");
    CHECK(history.getActionName(0) == "First" && history.getActionName(99) == "");

    bool committing = false;
    bool* probe = &committing;
    UndoRedo* self = &history;
    history.createAction("Committing");
    history.addDoMethod([probe, self] { *probe = self->isCommittingAction(); });
    CHECK(!history.isCommittingAction());
    history.commitAction();
    CHECK(committing && !history.isCommittingAction());

    ct::String notified;
    history.setCommitNotifyCallback([&notified](const ct::String& name) { notified = name; });
    history.createAction("Notify");
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(notified == "Notify");
}

void testCoreProperties()
{
    UndoRedo history;
    history.setClock([] { return testClock(); });
    Node3D* node = detached();
    CHECK(node != nullptr);
    if (!node) return;

    history.createAction("Move");
    history.addDoProperty(node, "position", PropertyValue::fromVec3(Math::Vec3(1, 2, 3)));
    history.addUndoProperty(node, "position", PropertyValue::fromVec3(node->getPosition()));
    history.commitAction();
    CHECK(node->getPosition() == Math::Vec3(1, 2, 3));
    CHECK(history.undo());
    CHECK(node->getPosition() == Math::Vec3(0, 0, 0));
    CHECK(history.redo());
    CHECK(node->getPosition() == Math::Vec3(1, 2, 3));
    node->queueFree();
}

void testCoreVersion()
{
    UndoRedo history;
    history.setClock([] { return testClock(); });
    const std::uint64_t start = history.getVersion();
    CHECK(start == 1);

    for (int i = 0; i < 3; ++i)
    {
        history.createAction("Step");
        history.addDoMethod([] {});
        history.commitAction();
    }
    CHECK(history.getVersion() == start + 3);
    CHECK(history.undo() && history.getVersion() == start + 2);
    CHECK(history.redo() && history.getVersion() == start + 3);

    int versions = 0;
    history.setVersionChangedCallback([&versions] { ++versions; });
    history.undo();
    history.redo();
    CHECK(versions == 2);

    const std::uint64_t before = history.getVersion();
    history.clearHistory();
    CHECK(history.getVersion() == before + 1);
    CHECK(!history.hasUndo() && !history.hasRedo() && history.getHistoryCount() == 0);
    history.clearHistory(false);
    CHECK(history.getVersion() == before + 1);
}

void testCoreMerge()
{
    UndoRedo history;
    Order order;
    Order* log = &order;
    std::uint64_t now = 100000;
    history.setClock([&now] { return now; });

    history.createAction("Edit", UndoRedo::kMergeEnds);
    history.addDoMethod([log] { log->log.push_back(1); });
    history.addUndoMethod([log] { log->log.push_back(-1); });
    history.commitAction();
    const std::uint64_t version = history.getVersion();

    now += 100;
    history.createAction("Edit", UndoRedo::kMergeEnds);
    CHECK(history.isMerging());
    history.addDoMethod([log] { log->log.push_back(2); });
    history.addUndoMethod([log] { log->log.push_back(-2); });
    history.commitAction();
    CHECK(!history.isMerging());
    CHECK(history.getHistoryCount() == 1);
    CHECK(history.getVersion() == version);
    CHECK(order.text() == "1,2");

    now += 100;
    history.createAction("Edit", UndoRedo::kMergeEnds);
    history.addDoMethod([log] { log->log.push_back(3); });
    history.commitAction();
    CHECK(history.getHistoryCount() == 1 && history.getVersion() == version);
    order.log.clear();
    CHECK(history.undo());
    CHECK(order.text() == "-1");
    CHECK(history.redo());
    CHECK(order.text() == "-1,3");

    now += 5000;
    history.createAction("Edit", UndoRedo::kMergeEnds);
    CHECK(!history.isMerging());
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(history.getHistoryCount() == 2);

    now += 10;
    history.createAction("Other", UndoRedo::kMergeEnds);
    CHECK(!history.isMerging());
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(history.getHistoryCount() == 3);

    now += 10;
    history.createAction("Other", UndoRedo::kMergeDisable);
    CHECK(!history.isMerging());
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(history.getHistoryCount() == 4);

    UndoRedo all;
    order.log.clear();
    all.setClock([&now] { return now; });
    all.createAction("Batch", UndoRedo::kMergeAll);
    all.addDoMethod([log] { log->log.push_back(1); });
    all.addUndoMethod([log] { log->log.push_back(-1); });
    all.commitAction();
    now += 50;
    all.createAction("Batch", UndoRedo::kMergeAll);
    all.addDoMethod([log] { log->log.push_back(2); });
    all.addUndoMethod([log] { log->log.push_back(-2); });
    all.commitAction();
    CHECK(all.getHistoryCount() == 1);
    CHECK(order.text() == "1,2");
    order.log.clear();
    CHECK(all.undo());
    CHECK(order.text() == "-1,-2");
    order.log.clear();
    CHECK(all.redo());
    CHECK(order.text() == "1,2");

    UndoRedo forced;
    forced.setClock([&now] { return now; });
    order.log.clear();
    forced.createAction("Force", UndoRedo::kMergeEnds);
    forced.addDoMethod([log] { log->log.push_back(1); });
    forced.addUndoMethod([log] { log->log.push_back(-1); });
    forced.commitAction();
    now += 20;
    forced.createAction("Force", UndoRedo::kMergeEnds);
    forced.startForceKeepInMergeEnds();
    forced.addDoMethod([log] { log->log.push_back(2); });
    forced.addUndoMethod([log] { log->log.push_back(-2); });
    forced.endForceKeepInMergeEnds();
    forced.addDoMethod([log] { log->log.push_back(3); });
    forced.addUndoMethod([log] { log->log.push_back(-3); });
    forced.commitAction();
    order.log.clear();
    CHECK(forced.undo());
    CHECK(order.text() == "-1,-2");
}

void testCoreHistoryLimit()
{
    UndoRedo history;
    history.setClock([] { return testClock(); });
    history.setMaxSteps(3);
    CHECK(history.getMaxSteps() == 3);
    for (int i = 0; i < 6; ++i)
    {
        ct::String name = "Action";
        name.append_number(i);
        history.createAction(name);
        history.addDoMethod([] {});
        history.commitAction();
    }
    CHECK(history.getHistoryCount() == 3);
    CHECK(history.getActionName(0) == "Action3");
    CHECK(history.getActionName(2) == "Action5");
    CHECK(history.getCurrentAction() == 2);
    CHECK(history.undo() && history.undo() && history.undo() && !history.undo());
}

void testCoreReferences()
{
    UndoRedo history;
    history.setClock([] { return testClock(); });
    Witness witness;
    Witness* seen = &witness;
    history.setDeleter([seen](Node* node) { seen->freed.push_back(node); });

    Node3D* added = detached();
    Node3D* removed = detached();
    Node3D* other = detached();
    CHECK(added && removed && other);
    if (!added || !removed || !other) return;

    history.createAction("Add");
    history.addDoReference(added);
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(witness.freed.empty());
    CHECK(history.undo());
    CHECK(witness.freed.empty());
    CHECK(history.redo());
    CHECK(history.undo());
    CHECK(witness.freed.empty());
    history.createAction("Next");
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(witness.has(added) && witness.freed.size() == 1);

    history.createAction("Remove");
    history.addUndoReference(removed);
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(history.undo());
    CHECK(history.redo());
    CHECK(!witness.has(removed));
    CHECK(history.undo());
    history.createAction("Other");
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(!witness.has(removed));

    history.createAction("Remove again");
    history.addUndoReference(removed);
    history.addDoMethod([] {});
    history.commitAction();
    history.setMaxSteps(2);
    history.createAction("Tail");
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(history.getHistoryCount() == 2);
    CHECK(!witness.has(removed));
    history.createAction("Tail2");
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(witness.has(removed) && witness.freed.size() == 2);

    history.createAction("Cleared");
    history.addUndoReference(other);
    history.addDoMethod([] {});
    history.commitAction();
    CHECK(!witness.has(other));
    history.clearHistory();
    CHECK(witness.has(other) && witness.freed.size() == 3);

    UndoRedo defaults;
    defaults.setClock([] { return testClock(); });
    Node3D* queued = detached();
    defaults.createAction("Add");
    defaults.addDoReference(queued);
    defaults.addDoMethod([] {});
    defaults.commitAction();
    CHECK(!queued->isQueuedForDeletion());
    CHECK(defaults.undo());
    defaults.createAction("Next");
    defaults.addDoMethod([] {});
    defaults.commitAction();
    CHECK(queued->isQueuedForDeletion());

    added->queueFree();
    removed->queueFree();
    other->queueFree();
}

void testManager()
{
    EditorUndoRedoManager manager;
    manager.history().setClock([] { return testClock(); });
    Node3D* node = detached();
    CHECK(node != nullptr);
    if (!node) return;

    CHECK(!manager.isHistoryUnsaved());
    CHECK(!manager.hasUndo() && !manager.hasRedo() && manager.currentActionName() == "");

    const auto edit = [&](const char* name, float x)
    {
        manager.createAction(name);
        manager.addDoProperty(node, "position", PropertyValue::fromVec3(Math::Vec3(x, 0, 0)));
        manager.addUndoProperty(node, "position", PropertyValue::fromVec3(node->getPosition()));
        manager.commitAction();
    };

    edit("One", 1.0f);
    CHECK(manager.isHistoryUnsaved());
    CHECK(manager.undo());
    CHECK(!manager.isHistoryUnsaved());
    CHECK(manager.redo());
    CHECK(manager.isHistoryUnsaved());

    manager.setHistoryAsSaved();
    CHECK(!manager.isHistoryUnsaved());
    edit("Two", 2.0f);
    CHECK(manager.isHistoryUnsaved());
    CHECK(manager.undo());
    CHECK(!manager.isHistoryUnsaved());
    CHECK(manager.undo());
    CHECK(manager.isHistoryUnsaved());
    CHECK(manager.redo());
    CHECK(!manager.isHistoryUnsaved());

    CHECK(manager.undo());
    edit("Three", 3.0f);
    CHECK(manager.isHistoryUnsaved());
    CHECK(manager.undo());
    CHECK(manager.isHistoryUnsaved());
    manager.setHistoryAsSaved();
    CHECK(!manager.isHistoryUnsaved());
    manager.setHistoryAsUnsaved();
    CHECK(manager.isHistoryUnsaved());

    manager.createAction("Silent", UndoRedo::kMergeDisable, false, false);
    manager.addDoMethod([] {});
    manager.addUndoMethod([] {});
    manager.setHistoryAsSaved();
    manager.commitAction();
    CHECK(!manager.isHistoryUnsaved());
    CHECK(manager.undo());
    CHECK(!manager.isHistoryUnsaved());

    CHECK(!manager.frozen());
    manager.setHistoryAsSaved();
    const std::uint64_t version = manager.version();
    const std::size_t count = manager.undoCount();
    manager.setFrozen(true);
    CHECK(manager.frozen() && !manager.hasUndo() && !manager.undo() && !manager.redo());
    edit("Frozen", 9.0f);
    CHECK(node->getPosition() == Math::Vec3(9, 0, 0));
    CHECK(manager.version() == version && manager.undoCount() == count);
    CHECK(!manager.isHistoryUnsaved());
    manager.setFrozen(false);
    CHECK(manager.undoCount() == count && manager.version() == version);

    bool busy = false;
    bool* probe = &busy;
    EditorUndoRedoManager* self = &manager;
    manager.createAction("Busy");
    manager.addDoMethod([probe, self] { *probe = self->busy() && self->isCommittingAction(); });
    manager.commitAction();
    CHECK(busy && !manager.busy());
    {
        EditorUndoRedoManager::Applying applying(&manager);
        CHECK(manager.busy());
    }
    CHECK(!manager.busy());

    node->queueFree();
}

void testRename(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* box = add(scene, "Box");
    add(scene, "Other");
    commit(rig);
    CHECK(!rig.history().hasUndo());
    CHECK(!rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditUndo)->isEnabled());
    CHECK(!rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditRedo)->isEnabled());

    select(rig, box);
    const ct::String before = rig.snap();
    rig.tap(Key::F2);
    rt::InputBox* boxWindow = rig.actions.renameBox();
    CHECK(boxWindow != nullptr);
    if (!boxWindow) return;
    rig.typeText("Crate");
    rig.tap(Key::Enter);
    CHECK(box->getName() == "Crate");
    CHECK(rig.history().currentActionName() == "Rename Node");
    const ct::String after = rig.snap();
    roundTrip(rig, "Rename Node", before, after, [box](bool done)
            { expect(box->getName() == (done ? "Crate" : "Box"), "rename", "name"); });
    commit(rig);
}

void testDuplicate(Rig& rig)
{
    Node* scene = freshScene(rig);
    add(scene, "A");
    Node3D* b = add(scene, "B");
    add(scene, "C");
    b->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    commit(rig);

    select(rig, b);
    const ct::String before = rig.snap();
    CHECK(rig.tap(Key::D, kModControl));
    Node* copy = scene->findChild("B2");
    CHECK(copy != nullptr);
    CHECK(names(scene) == "A,B,B2,C");
    CHECK(selectedIs(rig, copy));
    const ct::String after = rig.snap();
    roundTrip(rig, "Duplicate Node(s)", before, after,
            [&](bool done)
            {
                expect(done ? (scene->findChild("B2") == copy && selectedIs(rig, copy))
                            : (copy->getParent() == nullptr && !copy->isQueuedForDeletion() &&
                                      selectedIs(rig, b)),
                        "duplicate", "node identity and selection");
            });
    commit(rig);

    Node3D* a = static_cast<Node3D*>(scene->findChild("A"));
    Node* c = scene->findChild("C");
    select(rig, a, c);
    const ct::String beforeMany = rig.snap();
    CHECK(rig.tap(Key::D, kModControl));
    CHECK(names(scene) == "A,B,B2,C,A2,C2");
    const ct::String afterMany = rig.snap();
    roundTrip(rig, "Duplicate Node(s)", beforeMany, afterMany,
            [&](bool done)
            {
                expect(selectedIs(rig, done ? scene->findChild("A2") : a,
                               done ? scene->findChild("C2") : c),
                        "duplicate many", "selection");
            });
    commit(rig);
}

void testDelete(Rig& rig)
{
    Node* scene = freshScene(rig);
    add(scene, "First");
    Node3D* parent = add(scene, "Parent");
    Node3D* childA = add(parent, "ChildA");
    Node3D* childB = add(parent, "ChildB");
    Node3D* deep = add(childA, "Deep");
    deep->setOwner(scene);
    add(scene, "Last");
    const ct::String cratePath = testRoot() + "/pole_undo.prefab.json";
    Prefab::clearCache();
    {
        Node3D scratch;
        Node3D* root = scratch.createChild<Node3D>();
        root->setName("pole");
        add(root, "top");
        SceneResult saved;
        CHECK(Prefab::saveFromNode(root, cratePath, saved));
    }
    SceneResult placed;
    Node* instance = Prefab::instantiate(cratePath, scene, placed);
    CHECK(instance != nullptr);
    commit(rig);

    select(rig, parent);
    const ct::String before = rig.snap();
    CHECK(rig.tap(Key::Delete));
    rt::Dialog* dialog = popupDialog();
    CHECK(dialog != nullptr);
    clickDialogButton(rig, dialog, 0);
    CHECK(parent->getParent() == nullptr);
    CHECK(names(scene) == "First,Last,pole");
    const ct::String after = rig.snap();
    roundTrip(rig, "Delete Node(s)", before, after,
            [&](bool done)
            {
                expect(done ? (parent->getParent() == nullptr && parent->getChildCount() == 2 &&
                                      !parent->isQueuedForDeletion() &&
                                      rig.selection.selectedNodes().empty())
                            : (parent->getParent() == scene && parent->getIndex() == 1 &&
                                      parent->getChild(0) == childA &&
                                      parent->getChild(1) == childB && deep->getOwner() == scene &&
                                      selectedIs(rig, parent)),
                        "delete", "node kept alive, identity, index and selection");
            });
    CHECK(rig.undoKey());
    commit(rig);

    if (instance)
    {
        Node* top = instance->findChild("top");
        CHECK(top != nullptr && top->getOwner() == instance);
        select(rig, instance);
        const ct::String beforeInstance = rig.snap();
        rig.tap(Key::Delete, kModShift);
        CHECK(instance->getParent() == nullptr);
        roundTrip(rig, "Delete Node(s)", beforeInstance, rig.snap(),
                [&](bool done)
                {
                    expect(done || (top->getOwner() == instance && instance->getParent() == scene &&
                                           instance->isInstance()),
                            "delete instance", "the instance keeps its owner links");
                });
        commit(rig);
    }

    select(rig, childA, childB);
    const ct::String beforeTwo = rig.snap();
    rig.tap(Key::Delete, kModShift);
    CHECK(parent->getChildCount() == 0);
    roundTrip(rig, "Delete Node(s)", beforeTwo, rig.snap(),
            [&](bool done)
            {
                expect(done || (parent->getChild(0) == childA && parent->getChild(1) == childB),
                        "delete two", "children order");
            });
    commit(rig);
}

void testDeleteLifetime(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* doomed = add(scene, "Doomed");
    add(scene, "Keep");
    commit(rig);
    rig.history().history().setMaxSteps(3);

    select(rig, doomed);
    ct::String error;
    CHECK(rig.actions.deleteSelection(error));
    CHECK(doomed->getParent() == nullptr && !doomed->isQueuedForDeletion());
    rig.frames(3);
    CHECK(doomed->getParent() == nullptr && !doomed->isQueuedForDeletion());

    Node* keep = scene->findChild("Keep");
    for (int i = 0; i < 2; ++i)
    {
        select(rig, keep);
        CHECK(rig.actions.rename(keep, i == 0 ? "K1" : "K2", error));
        CHECK(!doomed->isQueuedForDeletion());
    }
    select(rig, keep);
    CHECK(rig.actions.rename(keep, "K3", error));
    CHECK(rig.history().history().getHistoryCount() == 3);
    CHECK(doomed->isQueuedForDeletion());
    rig.frames(2);
    rig.history().history().setMaxSteps(0);

    Node* added = nullptr;
    select(rig, keep);
    CHECK(rig.actions.createNode("Node3D", error));
    added = keep->findChild("Node3D");
    CHECK(added != nullptr);
    CHECK(rig.history().undo());
    CHECK(added->getParent() == nullptr && !added->isQueuedForDeletion());
    CHECK(rig.actions.rename(keep, "K4", error));
    CHECK(added->isQueuedForDeletion());
    rig.frames(2);
    commit(rig);
}

void testMove(Rig& rig)
{
    Node* scene = freshScene(rig);
    add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    add(scene, "D");
    commit(rig);

    select(rig, b);
    const ct::String before = rig.snap();
    CHECK(rig.tap(Key::Down, kModControl));
    CHECK(names(scene) == "A,C,B,D");
    roundTrip(rig, "Move Node(s)", before, rig.snap(), [&](bool done)
            { expect(names(scene) == (done ? "A,C,B,D" : "A,B,C,D"), "move", "order"); });
    commit(rig);

    select(rig, c, b);
    const ct::String beforeUp = rig.snap();
    CHECK(rig.tap(Key::Up, kModControl));
    CHECK(names(scene) == "C,B,A,D");
    roundTrip(rig, "Move Node(s)", beforeUp, rig.snap(),
            [&](bool done)
            {
                expect(names(scene) == (done ? "C,B,A,D" : "A,C,B,D") && selectedIs(rig, c, b),
                        "move many", "order and selection");
            });
    commit(rig);
}

void testReparent(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* target = add(scene, "T");
    Node3D* twin = add(target, "A");
    Node3D* other = add(scene, "O");
    a->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    a->setRotationDegrees(Math::Vec3(10.0f, 20.0f, 30.0f));
    target->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    target->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    (void) twin;
    (void) other;
    commit(rig);

    select(rig, a);
    const Math::Vec3 position = a->getPosition();
    const Math::Mat4 global = a->getGlobalTransform();
    const ct::String before = rig.snap();
    rig.actions.requestReparent();
    rig.frames(3);
    ReparentDialog* dialog = rig.actions.reparentDialog();
    CHECK(dialog != nullptr);
    if (!dialog) return;
    CHECK(dialog->pick(target));
    dialog->keepTransformBox()->setChecked(true);
    dialog->accept();
    rig.frames(3);
    CHECK(a->getParent() == target);
    CHECK(a->getName() != "A");
    const ct::String renamed = a->getName();
    const ct::String after = rig.snap();
    CHECK(fabsf(a->getGlobalTransform().GetTranslation().x - global.GetTranslation().x) < 1e-3f);
    roundTrip(rig, "Reparent Node", before, after,
            [&](bool done)
            {
                expect(done ? (a->getParent() == target && a->getName() == renamed &&
                                      selectedIs(rig, a))
                            : (a->getParent() == scene && a->getName() == "A" &&
                                      a->getIndex() == 0 && a->getPosition() == position &&
                                      selectedIs(rig, a)),
                        "reparent", "parent, name, index, transform and selection");
            });
    commit(rig);

    select(rig, twin, other);
    const ct::String beforeMany = rig.snap();
    rig.actions.requestReparent();
    rig.frames(3);
    dialog = rig.actions.reparentDialog();
    CHECK(dialog != nullptr);
    if (!dialog) return;
    CHECK(dialog->pick(scene));
    dialog->keepTransformBox()->setChecked(false);
    dialog->accept();
    rig.frames(3);
    CHECK(twin->getParent() == scene && other->getParent() == scene);
    roundTrip(rig, "Reparent Node", beforeMany, rig.snap(),
            [&](bool done)
            {
                expect(selectedIs(rig, twin, other),
                        done ? "reparent many after" : "reparent many before", "selection");
            });
    commit(rig);
}

void testReparentToNew(Rig& rig)
{
    Node* scene = freshScene(rig);
    add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    add(scene, "D");
    b->setPosition(Math::Vec3(4.0f, 5.0f, 6.0f));
    commit(rig);

    select(rig, b, c);
    const ct::String before = rig.snap();
    rig.actions.run(HierarchyActions::kReparentToNew);
    rig.frames(3);
    CreateNodeDialog* box = rig.actions.createDialog();
    CHECK(box != nullptr);
    if (!box) return;
    box->setSearchText("Node3D");
    box->confirm();
    rig.frames(3);
    Node* wrapper = scene->findChild("Node3D");
    CHECK(wrapper != nullptr);
    if (!wrapper) return;
    CHECK(b->getParent() == wrapper && c->getParent() == wrapper);
    const ct::String after = rig.snap();
    roundTrip(rig, "Reparent to New Node", before, after,
            [&](bool done)
            {
                expect(done ? (b->getParent() == wrapper && wrapper->getParent() == scene &&
                                      selectedIs(rig, b, c))
                            : (b->getParent() == scene && wrapper->getParent() == nullptr &&
                                      !wrapper->isQueuedForDeletion() && selectedIs(rig, b, c)),
                        "reparent to new", "wrapper lifetime, parents and selection");
            });
    commit(rig);
}

void testAddNode(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    add(scene, "B");
    commit(rig);

    select(rig, a);
    const ct::String before = rig.snap();
    CHECK(rig.tap(Key::A, kModControl));
    CreateNodeDialog* box = rig.actions.createDialog();
    CHECK(box != nullptr);
    if (!box) return;
    box->setSearchText("Camera3D");
    box->confirm();
    rig.frames(3);
    Node* camera = a->findChild("Camera3D");
    CHECK(camera != nullptr);
    CHECK(selectedIs(rig, camera));
    const ct::String after = rig.snap();
    roundTrip(rig, "Add Node", before, after,
            [&](bool done)
            {
                expect(done ? (a->findChild("Camera3D") == camera && selectedIs(rig, camera))
                            : (camera->getParent() == nullptr && !camera->isQueuedForDeletion() &&
                                      selectedIs(rig, a)),
                        "add node", "the node is the same object and the selection is restored");
            });
    commit(rig);

    select(rig, nullptr);
    const ct::String beforeRoot = rig.snap();
    CHECK(rig.tap(Key::A, kModControl));
    box = rig.actions.createDialog();
    CHECK(box != nullptr);
    if (box)
    {
        box->setSearchText("OmniLight3D");
        box->confirm();
        rig.frames(3);
    }
    Node* light = scene->findChild("OmniLight3D");
    CHECK(light != nullptr);
    roundTrip(rig, "Add Node", beforeRoot, rig.snap(),
            [&](bool done) {
                expect(done ? selectedIs(rig, light) : selectedIs(rig), "add under root",
                        "selection");
            });
    commit(rig);
}

void testCreateMenu(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    commit(rig);
    EditorCamera::Cursor& cursor = rig.panel.camera().cursor();
    cursor.posX = 2.0;
    cursor.posY = 1.0;
    cursor.posZ = -3.0;
    rig.frames(2);
    rt::Menu* bar = rig.menus.menu(EditorMenus::kCreate);

    select(rig, nullptr);
    const ct::String beforeEmpty = rig.snap();
    CHECK(openBarMenu(rig, bar));
    clickMenuItem(rig, bar, CreateMenu::kEntryEmpty);
    Node* empty = scene->findChild("Node3D");
    CHECK(empty != nullptr && selectedIs(rig, empty));
    roundTrip(rig, "Add Node", beforeEmpty, rig.snap(),
            [&](bool done)
            {
                expect(done ? empty->getParent() == scene : empty->getParent() == nullptr,
                        "create empty", "parent");
            });
    commit(rig);

    select(rig, a);
    const ct::String beforeSphere = rig.snap();
    CHECK(openBarMenu(rig, bar));
    clickMenuItem(rig, bar, CreateMenu::kEntryObject);
    rt::Menu* objects = bar->action(CreateMenu::kEntryObject)->submenu();
    CHECK(objects != nullptr);
    if (!objects) return;
    clickMenuItem(rig, objects, 1);
    rig.tap(Key::Enter);
    Node* sphere = a->findChild("Sphere");
    CHECK(sphere != nullptr && selectedIs(rig, sphere));
    const ct::String afterSphere = rig.snap();
    CHECK(afterSphere.find("Sphere") != ct::String::npos);
    roundTrip(rig, "Add Node", beforeSphere, afterSphere,
            [&](bool done)
            {
                expect(done ? (sphere->getParent() == a && selectedIs(rig, sphere))
                            : (sphere->getParent() == nullptr && selectedIs(rig, a)),
                        "create primitive", "parent and selection");
            });
    commit(rig);

    select(rig, nullptr);
    const ct::String beforeBody = rig.snap();
    rt::ToolBar* toolBar = rig.hierarchy.toolBar();
    CHECK(toolBar != nullptr);
    if (!toolBar) return;
    const rt::Rect barRect = toolBar->absoluteRect();
    rig.click(static_cast<int>(barRect.x + 2.0f + 14.0f),
            static_cast<int>(barRect.y + barRect.h * 0.5f));
    rt::Menu* dropdown = rig.actions.createMenu().dropdown();
    CHECK(dropdown != nullptr);
    if (!dropdown) return;
    clickMenuItem(rig, dropdown, CreateMenu::kEntryPhysics);
    rt::Menu* physics = dropdown->action(CreateMenu::kEntryPhysics)->submenu();
    CHECK(physics != nullptr);
    if (!physics) return;
    clickMenuItem(rig, physics, 0);
    Node* body = scene->findChild("StaticBody3D");
    CHECK(body != nullptr && body->getChildCount() == 1);
    roundTrip(rig, "Add Node", beforeBody, rig.snap(),
            [&](bool done)
            {
                expect(done ? body->getParent() == scene && body->getChildCount() == 1
                            : body->getParent() == nullptr && body->getChildCount() == 1,
                        "create body", "the body keeps its children");
            });
    commit(rig);
}

void showInspector(Rig& rig)
{
    rig.shell.dock()->showPanel("Inspector");
    rig.shell.dock()->layout();
    rig.frames(4);
}

void testInspectorFields(Rig& rig)
{
    showInspector(rig);
    Node* scene = freshScene(rig);
    Node3D* node = add(scene, "N");
    node->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    node->setRotationDegrees(Math::Vec3(10.0f, 20.0f, 30.0f));
    node->setScale(Math::Vec3(1.5f, 1.5f, 1.5f));
    commit(rig);
    select(rig, node);
    rig.frames(3);

    const Math::Vec3 originalRotation = node->getRotation();
    ct::String before = rig.snap();
    typeInto(rig, rig.inspector.field(InspectorPanel::kPosition, 0), "5");
    CHECK(node->getPosition() == Math::Vec3(5.0f, 2.0f, 3.0f));
    ct::String after = rig.snap();
    roundTrip(rig, "Set position", before, after,
            [&](bool done)
            {
                expect(node->getPosition() == (done ? Math::Vec3(5, 2, 3) : Math::Vec3(1, 2, 3)),
                        "set position", "position");
            });
    commit(rig);

    before = rig.snap();
    typeInto(rig, rig.inspector.field(InspectorPanel::kRotation, 1), "75");
    CHECK(fabsf(node->getRotationDegrees().y - 75.0f) < 1e-3f);
    after = rig.snap();
    roundTrip(rig, "Set rotation", before, after,
            [&](bool done)
            {
                if (!done)
                    expect(node->getRotation() == originalRotation, "set rotation",
                            "the rotation is restored bit for bit");
            });
    commit(rig);

    before = rig.snap();
    typeInto(rig, rig.inspector.field(InspectorPanel::kScale, 2), "3");
    after = rig.snap();
    roundTrip(rig, "Set scale", before, after,
            [&](bool done)
            {
                expect(node->getScale() == (done ? Math::Vec3(1.5f, 1.5f, 3.0f)
                                                 : Math::Vec3(1.5f, 1.5f, 1.5f)),
                        "set scale", "scale");
            });
    commit(rig);

    before = rig.snap();
    rig.click(centerX(rig.inspector.resetButton(InspectorPanel::kScale)),
            centerY(rig.inspector.resetButton(InspectorPanel::kScale)));
    CHECK(node->getScale() == Math::Vec3(1, 1, 1));
    after = rig.snap();
    roundTrip(rig, "Set scale", before, after,
            [&](bool done)
            {
                expect(node->getScale() ==
                                (done ? Math::Vec3(1, 1, 1) : Math::Vec3(1.5f, 1.5f, 3.0f)),
                        "reset scale", "scale");
            });
    commit(rig);

    gStep = 1;
    before = rig.snap();
    const std::size_t count = rig.history().undoCount();
    const Math::Vec3 startPosition = node->getPosition();
    dragField(rig, rig.inspector.field(InspectorPanel::kPosition, 0), 90, 12);
    gStep = 5000;
    const Math::Vec3 dragged = node->getPosition();
    CHECK(dragged.x != startPosition.x);
    CHECK(rig.history().undoCount() == count + 1);
    CHECK(rig.history().currentActionName() == "Set position");
    after = rig.snap();
    roundTrip(rig, "Set position", before, after,
            [&](bool done)
            {
                expect(node->getPosition() == (done ? dragged : startPosition), "drag position",
                        "a drag is one step and restores the start exactly");
            });
    commit(rig);

    gStep = 1;
    const std::size_t typed = rig.history().undoCount();
    typeInto(rig, rig.inspector.field(InspectorPanel::kPosition, 1), "7");
    typeInto(rig, rig.inspector.field(InspectorPanel::kPosition, 1), "8");
    gStep = 5000;
    CHECK(rig.history().undoCount() == typed + 1);
    CHECK(node->getPosition().y == 8.0f);
    CHECK(rig.undoKey());
    CHECK(node->getPosition().y == startPosition.y);
    CHECK(rig.redoKey());
    CHECK(node->getPosition().y == 8.0f);
    commit(rig);

    Node3D* second = add(scene, "S");
    second->setPosition(Math::Vec3(0, 0, 0));
    select(rig, second);
    rig.frames(3);
    gStep = 1;
    const std::size_t firstCount = rig.history().undoCount();
    typeInto(rig, rig.inspector.field(InspectorPanel::kPosition, 0), "2");
    select(rig, node);
    typeInto(rig, rig.inspector.field(InspectorPanel::kPosition, 0), "3");
    gStep = 5000;
    CHECK(rig.history().undoCount() == firstCount + 2);
    CHECK(rig.undoKey());
    CHECK(node->getPosition().x == dragged.x);
    CHECK(second->getPosition().x == 2.0f);
    CHECK(rig.undoKey());
    CHECK(second->getPosition().x == 0.0f);
    commit(rig);
}

void testInspectorProperty(Rig& rig)
{
    showInspector(rig);
    Node* scene = freshScene(rig);
    Camera3D* camera = scene->createChild<Camera3D>();
    camera->setName("Cam");
    commit(rig);
    select(rig, camera);
    rig.frames(3);

    PropertyRow* row = rig.inspector.properties().rowNamed("fov");
    CHECK(row != nullptr);
    if (!row) return;
    const float startFov = camera->getFov();
    const ct::String before = rig.snap();
    typeInto(rig, row->field(0), "42");
    CHECK(camera->getFov() == 42.0f);
    roundTrip(rig, "Set fov", before, rig.snap(), [&](bool done)
            { expect(camera->getFov() == (done ? 42.0f : startFov), "set fov", "fov"); });
    commit(rig);
}

void testHeader(Rig& rig)
{
    showInspector(rig);
    Node* scene = freshScene(rig);
    Node3D* node = add(scene, "Head");
    add(scene, "Other");
    commit(rig);
    select(rig, node);
    rig.frames(3);

    ct::String before = rig.snap();
    commitText(rig, rig.inspector.nameInput(), "Renamed");
    CHECK(node->getName() == "Renamed");
    roundTrip(rig, "Rename Node", before, rig.snap(), [&](bool done)
            { expect(node->getName() == (done ? "Renamed" : "Head"), "header name", "name"); });
    commit(rig);

    before = rig.snap();
    rig.click(centerX(rig.inspector.visibleBox()) - 20, centerY(rig.inspector.visibleBox()));
    CHECK(!node->isVisible());
    roundTrip(rig, "Set visible", before, rig.snap(),
            [&](bool done) { expect(node->isVisible() == !done, "header visible", "visible"); });
    commit(rig);

    before = rig.snap();
    rig.click(centerX(rig.inspector.activeBox()) - 20, centerY(rig.inspector.activeBox()));
    CHECK(node->getProcessMode() == Node::ProcessMode::Disabled);
    roundTrip(rig, "Set active", before, rig.snap(),
            [&](bool done)
            {
                expect((node->getProcessMode() == Node::ProcessMode::Disabled) == done,
                        "header active", "process mode");
            });
    commit(rig);

    before = rig.snap();
    commitText(rig, rig.inspector.tagInput(), "enemy, boss");
    CHECK(InspectorPanel::tagOf(node) == "enemy, boss");
    roundTrip(rig, "Set tag", before, rig.snap(),
            [&](bool done) {
                expect(InspectorPanel::tagOf(node) == (done ? "enemy, boss" : ""), "header tag",
                        "tag");
            });
    commit(rig);

    before = rig.snap();
    commitText(rig, rig.inspector.tagInput(), "boss");
    roundTrip(rig, "Set tag", before, rig.snap(),
            [&](bool done)
            {
                expect(InspectorPanel::tagOf(node) == (done ? "boss" : "enemy, boss"), "tag change",
                        "tag order is restored");
            });
    commit(rig);
}

void testEye(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    commit(rig);

    select(rig, a);
    ct::String before = rig.snap();
    rig.click(eyeX(rig), rowY(rig, a));
    CHECK(!a->isVisible());
    CHECK(rig.history().currentActionName() == "Toggle Visible");
    roundTrip(rig, "Toggle Visible", before, rig.snap(),
            [&](bool done)
            {
                expect(a->isVisible() == !done, "eye", "visible");
                rig.frames(2);
                expect(rig.hierarchy.outliner()->isNodeVisible(rig.hierarchy.itemOf(a)) == !done,
                        "eye", "the row icon follows undo and redo");
            });
    commit(rig);

    select(rig, b, c);
    before = rig.snap();
    rig.click(eyeX(rig), rowY(rig, b));
    CHECK(!c->isVisible() && !b->isVisible());
    CHECK(rig.history().currentActionName() == "Toggle Visibility of 2 Nodes");
    roundTrip(rig, "Toggle Visibility of 2 Nodes", before, rig.snap(), [&](bool done)
            { expect(c->isVisible() == !done && b->isVisible() == !done, "eye many", "visible"); });
    commit(rig);
}

struct Cam
{
    Math::Mat4 transform;
    EditorCamera::Projection projection;
    float width;
    float height;
    float originX;
    float originY;

    Math::Vec3 position() const
    {
        return transform.GetTranslation();
    }

    Math::Vec3 forward() const
    {
        return Math::Vec3(-transform.col2.x, -transform.col2.y, -transform.col2.z).Normalized();
    }

    Math::Vec2 project(const Math::Vec3& world) const
    {
        const Math::Vec3 local = transform.InverseAffine().TransformPoint(world);
        const float aspect = width / height;
        const float tanHalf = tanf(projection.fov * kPi / 360.0f);
        const float depth = -local.z;
        const float ndcX = local.x / (depth * tanHalf * aspect);
        const float ndcY = local.y / (depth * tanHalf);
        return Math::Vec2((ndcX * 0.5f + 0.5f) * width, (-ndcY * 0.5f + 0.5f) * height);
    }

    float gizmoScale(const Math::Vec3& world, float size = 80.0f) const
    {
        const float depth = forward().Dot(world - position());
        const float pixelsPerUnit = height / (2.0f * depth * tanf(projection.fov * kPi / 360.0f));
        const float factor = (height < 400.0f ? height : 400.0f) / 400.0f;
        return size / pixelsPerUnit * factor;
    }
};

Cam currentCam(const EditorViewportPanel& panel)
{
    Cam cam;
    cam.transform = panel.camera().interpToCameraTransform();
    cam.projection = panel.camera().projection();
    cam.width = static_cast<float>(panel.width());
    cam.height = static_cast<float>(panel.height());
    const rt::Rect rect = panel.view()->absoluteRect();
    cam.originX = rect.x;
    cam.originY = rect.y;
    return cam;
}

void focusCamera(Rig& rig, const Math::Vec3& target, float distance)
{
    EditorCamera::Cursor& cursor = rig.panel.camera().cursor();
    cursor.posX = target.x;
    cursor.posY = target.y;
    cursor.posZ = target.z;
    cursor.distance = distance;
    rig.frames(120);
}

void gizmoDrag(Rig& rig, int fromX, int fromY, int toX, int toY, unsigned modifiers = 0)
{
    rig.send(motion(fromX, fromY));
    rig.frames(1);
    rig.send(button(fromX, fromY, MouseButton::Left, true, modifiers));
    rig.frames(1);
    rig.send(motion((fromX + toX) / 2, (fromY + toY) / 2, kLeftMask));
    rig.frames(1);
    rig.send(motion(toX, toY, kLeftMask));
    rig.frames(1);
    rig.send(button(toX, toY, MouseButton::Left, false, modifiers));
    rig.frames(2);
}

void testGizmo(Rig& rig)
{
    Node* scene = freshScene(rig);
    MeshInstance3D* box = addMesh(scene, "Box", Math::Vec3(1, 1, 1));
    const Math::Vec3 home(0.0f, 0.0f, 0.0f);
    box->setPosition(home);
    commit(rig);
    rig.shell.dock()->showPanel("Viewport");
    rig.shell.dock()->layout();
    focusCamera(rig, home, 7.0f);
    select(rig, box);
    rig.frames(3);
    CHECK(rig.panel.gizmo()->visible());

    const Cam cam = currentCam(rig.panel);
    const float scale = cam.gizmoScale(home);
    const Math::Vec2 arrow = cam.project(home + Math::Vec3(1.0f, 0.0f, 0.0f) * (scale * 1.575f));
    const int fromX = static_cast<int>(cam.originX + arrow.x + 0.5f);
    const int fromY = static_cast<int>(cam.originY + arrow.y + 0.5f);

    const Math::Vec3 startPosition = box->getPosition();
    const Math::Vec3 startRotation = box->getRotation();
    const Math::Vec3 startScale = box->getScale();
    const ct::String before = rig.snap();
    const unsigned edits = rig.panel.gizmo()->editCount();
    gizmoDrag(rig, fromX, fromY, fromX + 60, fromY + 14);
    CHECK(rig.panel.gizmo()->editCount() == edits + 1);
    const Math::Vec3 moved = box->getPosition();
    CHECK(fabsf(moved.x - startPosition.x) > 0.2f);
    CHECK(rig.history().currentActionName() == "Translate");
    const ct::String after = rig.snap();
    roundTrip(rig, "Translate", before, after,
            [&](bool done)
            {
                expect(box->getPosition() == (done ? moved : startPosition) &&
                                box->getRotation() == startRotation &&
                                box->getScale() == startScale,
                        "gizmo translate", "one drag is one step and restores the start exactly");
                expect(selectedIs(rig, box), "gizmo translate", "the selection stays");
            });
    commit(rig);

    CHECK(rig.undoKey());
    CHECK(box->getPosition() == startPosition);
    commit(rig);

    {
        const std::size_t count = rig.history().undoCount();
        const ct::String beforeCancel = rig.snap();
        rig.send(motion(fromX, fromY));
        rig.frames(1);
        rig.send(button(fromX, fromY, MouseButton::Left, true));
        rig.frames(1);
        rig.send(motion(fromX + 40, fromY + 8, kLeftMask));
        rig.frames(1);
        CHECK(rig.panel.gizmo()->editing());
        CHECK(box->getPosition() != startPosition);
        rig.send(key(Key::Escape, true));
        rig.send(key(Key::Escape, false));
        rig.frames(2);
        rig.send(button(fromX + 40, fromY + 8, MouseButton::Left, false));
        rig.frames(2);
        CHECK(!rig.panel.gizmo()->editing());
        CHECK(rig.history().undoCount() == count);
        CHECK(box->getPosition() == startPosition);
        CHECK(rig.snap() == beforeCancel);
        CHECK(!rig.document.dirty());
        select(rig, box);
    }

    {
        const std::size_t count = rig.history().undoCount();
        rig.send(motion(fromX, fromY));
        rig.frames(1);
        rig.send(button(fromX, fromY, MouseButton::Left, true));
        rig.frames(1);
        rig.send(button(fromX, fromY, MouseButton::Left, false));
        rig.frames(2);
        CHECK(rig.history().undoCount() == count);
    }

    {
        rig.send(key(Key::E, true));
        rig.send(key(Key::E, false));
        rig.frame();
        CHECK(rig.panel.gizmo()->tool() == EditorGizmo::Tool::Rotate);
        Math::Vec3 toCamera = cam.position() - home;
        toCamera.y = 0.0f;
        toCamera = toCamera.Normalized();
        const float radius = scale * EditorGizmo::kCircleSize;
        const auto around = [&](float angle)
        {
            const float c = cosf(angle);
            const float n = sinf(angle);
            return home + Math::Vec3(toCamera.x * c + toCamera.z * n, 0.0f,
                                  -toCamera.x * n + toCamera.z * c) *
                                  radius;
        };
        const Math::Vec2 grab = cam.project(around(0.0f));
        const Math::Vec2 to = cam.project(around(0.45f));
        const int gx = static_cast<int>(cam.originX + grab.x + 0.5f);
        const int gy = static_cast<int>(cam.originY + grab.y + 0.5f);
        const int tx = static_cast<int>(cam.originX + to.x + 0.5f);
        const int ty = static_cast<int>(cam.originY + to.y + 0.5f);
        const ct::String beforeRotate = rig.snap();
        gizmoDrag(rig, gx, gy, tx, ty);
        const Math::Vec3 rotated = box->getRotation();
        CHECK(rig.history().currentActionName() == "Rotate");
        CHECK(rotated != startRotation);
        roundTrip(rig, "Rotate", beforeRotate, rig.snap(),
                [&](bool done)
                {
                    expect(box->getRotation() == (done ? rotated : startRotation) &&
                                    box->getPosition() == startPosition &&
                                    box->getScale() == startScale,
                            "gizmo rotate", "the rotation is restored exactly");
                });
        CHECK(rig.undoKey());
        commit(rig);
    }

    {
        rig.send(key(Key::R, true));
        rig.send(key(Key::R, false));
        rig.frame();
        CHECK(rig.panel.gizmo()->tool() == EditorGizmo::Tool::Scale);
        const Math::Vec2 handle =
                cam.project(home + Math::Vec3(1.0f, 0.0f, 0.0f) * (scale * 1.475f));
        const int gx = static_cast<int>(cam.originX + handle.x + 0.5f);
        const int gy = static_cast<int>(cam.originY + handle.y + 0.5f);
        const ct::String beforeScale = rig.snap();
        gizmoDrag(rig, gx, gy, gx + 55, gy + 9);
        const Math::Vec3 scaled = box->getScale();
        CHECK(rig.history().currentActionName() == "Scale");
        CHECK(scaled != startScale);
        roundTrip(rig, "Scale", beforeScale, rig.snap(),
                [&](bool done)
                {
                    expect(box->getScale() == (done ? scaled : startScale) &&
                                    box->getPosition() == startPosition &&
                                    box->getRotation() == startRotation,
                            "gizmo scale", "the scale is restored exactly");
                });
        CHECK(rig.undoKey());
        commit(rig);
    }

    rig.panel.gizmo()->setTool(EditorGizmo::Tool::Transform);
    rig.frames(2);
}

void showAssets(Rig& rig)
{
    rig.shell.dock()->showPanel("Assets");
    rig.shell.dock()->layout();
    rig.frames(4);
}

bool writeFile(const ct::String& path, const char* text)
{
    return file_write_text(path.c_str(), text);
}

void testMaterial(Rig& rig)
{
    Node* scene = freshScene(rig);
    MeshInstance3D* target = addMesh(scene, "Target", Math::Vec3(2, 2, 2));
    Node3D* holder = add(scene, "Holder");
    MeshInstance3D* inner = addMesh(holder, "Inner", Math::Vec3(1, 1, 1));
    inner->setPosition(Math::Vec3(30.0f, 0.0f, 0.0f));
    MeshInstance3D* row = addMesh(scene, "Row", Math::Vec3(1, 1, 1));
    row->setPosition(Math::Vec3(60.0f, 0.0f, 0.0f));
    commit(rig);
    select(rig, nullptr);

    ct::String error;
    ct::String path;
    CHECK(rig.materials.createLibrary("Assets/Undo", "Painted", path, error));
    rig.frames(2);
    rig.panel.camera().cursor().posX = 0.0;
    rig.panel.camera().cursor().posY = 0.0;
    rig.panel.camera().cursor().posZ = 0.0;
    rig.panel.camera().cursor().distance = 6.0f;
    rig.panel.camera().updateCamera();
    rig.frames(40);

    showAssets(rig);
    CHECK(rig.assets.navigateTo("Assets/Undo"));
    rig.assets.setViewMode(AssetViewMode::List);
    rig.frames(4);
    const int entry = rig.assets.indexOf("Painted.mat");
    CHECK(entry >= 0);
    rig.assets.grid()->ensureVisible(entry);
    rig.frames(2);
    int fromX = 0;
    int fromY = 0;
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    const rt::Rect view = rig.panel.view()->absoluteRect();
    const int viewX = static_cast<int>(view.x + view.w * 0.5f);
    const int viewY = static_cast<int>(view.y + view.h * 0.5f);
    CHECK(rig.panel.pickAt(static_cast<float>(viewX), static_cast<float>(viewY)) == target);

    const ct::String before = rig.snap();
    rig.drag(fromX, fromY, viewX, viewY);
    CHECK(target->getMaterialOverride().valid());
    CHECK(rig.history().currentActionName() == "Assign Material");
    const ct::String after = rig.snap();
    const MaterialId assigned = target->getMaterialOverride();
    roundTrip(rig, "Assign Material", before, after,
            [&](bool done)
            {
                expect(done ? target->getMaterialOverride() == assigned
                            : !target->getMaterialOverride().valid(),
                        "material drop", "the override is restored exactly");
            });
    commit(rig);

    select(rig, nullptr);
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    const ct::String beforeHolder = rig.snap();
    rig.drag(fromX, fromY, textX(rig, holder), rowY(rig, holder));
    CHECK(inner->getMaterialOverride().valid());
    roundTrip(rig, "Assign Material", beforeHolder, rig.snap(),
            [&](bool done)
            {
                expect(inner->getMaterialOverride().valid() == done, "material on children",
                        "the meshes under the row follow undo and redo");
            });
    commit(rig);

    select(rig, row);
    rig.shell.dock()->showPanel("Material");
    rig.shell.dock()->layout();
    rig.frames(4);
    const ct::String beforeSlot = rig.snap();
    CHECK(rig.materials.newMaterial(0, "SlotMat", error));
    rig.frames(3);
    CHECK(row->getSurfaceMaterial(0).valid());
    CHECK(rig.history().currentActionName() == "Assign Material");
    const ct::String afterSlot = rig.snap();
    roundTrip(rig, "Assign Material", beforeSlot, afterSlot,
            [&](bool done)
            {
                expect(row->getSurfaceMaterial(0).valid() == done, "material slot",
                        "the slot material follows undo and redo");
            });
    rig.frames(3);
    CHECK(rig.materials.clearSlot(0));
    CHECK(!row->getSurfaceMaterial(0).valid());
    CHECK(rig.history().currentActionName() == "Assign Material");
    CHECK(rig.undoKey());
    CHECK(row->getSurfaceMaterial(0).valid());
    CHECK(rig.redoKey());
    CHECK(!row->getSurfaceMaterial(0).valid());
    commit(rig);
}

void testScriptDrop(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    commit(rig);
    CHECK(fs_create_directory(absolute(rig, "Assets/Scripts").c_str()));
    CHECK(writeFile(absolute(rig, "Assets/Scripts/spin.py"),
            "class Spin(Node3D):\n    def _process(self, delta):\n        pass\n"));
    showAssets(rig);
    CHECK(rig.assets.navigateTo("Assets/Scripts"));
    rig.assets.setViewMode(AssetViewMode::List);
    rig.frames(4);
    const int entry = rig.assets.indexOf("spin.py");
    CHECK(entry >= 0);
    rig.assets.grid()->ensureVisible(entry);
    rig.frames(2);
    int fromX = 0;
    int fromY = 0;
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);

    select(rig, nullptr);
    const ct::String before = rig.snap();
    CHECK(a->getScript().empty());
    rig.drag(fromX, fromY, textX(rig, a), rowY(rig, a));
    CHECK(!a->getScript().empty());
    const ct::String attached = a->getScript();
    CHECK(rig.history().currentActionName() == "Attach Script");
    roundTrip(rig, "Attach Script", before, rig.snap(),
            [&](bool done)
            {
                expect(done ? a->getScript() == attached : a->getScript().empty(), "script drop",
                        "the script path");
            });
    commit(rig);

    const ct::String beforeApi = rig.snap();
    CHECK(rig.assets.attachScript("Assets/Scripts/spin.py", b));
    roundTrip(rig, "Attach Script", beforeApi, rig.snap(),
            [&](bool done)
            {
                expect(done ? !b->getScript().empty() : b->getScript().empty(), "script attach",
                        "the script path");
            });
    commit(rig);
}

void testInstantiate(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    add(scene, "B");
    commit(rig);

    CHECK(fs_create_directory(absolute(rig, "Assets/Prefabs").c_str()));
    const ct::String prefab = absolute(rig, "Assets/Prefabs/Crate.prefab.json");
    Prefab::clearCache();
    {
        Node3D scratch;
        Node3D* root = scratch.createChild<Node3D>();
        root->setName("crate");
        add(root, "lid");
        add(root, "base");
        SceneResult saved;
        CHECK(Prefab::saveFromNode(root, prefab, saved));
    }
    rig.panel.camera().cursor().posX = 0.0;
    rig.panel.camera().cursor().posY = 0.0;
    rig.panel.camera().cursor().posZ = 0.0;
    rig.panel.camera().cursor().distance = 6.0f;
    rig.panel.camera().updateCamera();
    rig.frames(40);

    showAssets(rig);
    CHECK(rig.assets.navigateTo("Assets/Prefabs"));
    rig.assets.setViewMode(AssetViewMode::List);
    rig.frames(4);
    const int entry = rig.assets.indexOf("Crate.prefab.json");
    CHECK(entry >= 0);
    rig.assets.grid()->ensureVisible(entry);
    rig.frames(2);
    int fromX = 0;
    int fromY = 0;
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    const rt::Rect view = rig.panel.view()->absoluteRect();
    const int viewX = static_cast<int>(view.x + view.w * 0.5f);
    const int viewY = static_cast<int>(view.y + view.h * 0.5f);

    select(rig, a);
    const ct::String before = rig.snap();
    rig.drag(fromX, fromY, viewX, viewY);
    Node* crate = scene->findChild("crate");
    CHECK(crate != nullptr);
    if (!crate) return;
    CHECK(selectedIs(rig, crate));
    Node* lid = crate->findChild("lid");
    CHECK(lid != nullptr && lid->getOwner() == crate);
    const Math::Vec3 where = static_cast<Node3D*>(crate)->getPosition();
    roundTrip(rig, "Instantiate Scene", before, rig.snap(),
            [&](bool done)
            {
                expect(done ? (scene->findChild("crate") == crate && selectedIs(rig, crate) &&
                                      lid->getOwner() == crate &&
                                      static_cast<Node3D*>(crate)->getPosition() == where)
                            : (crate->getParent() == nullptr && !crate->isQueuedForDeletion() &&
                                      selectedIs(rig, a)),
                        "instantiate drop", "identity, owners, position and selection");
            });
    commit(rig);

    select(rig, nullptr);
    centerOf(rig.assets.grid()->itemRect(entry), fromX, fromY);
    const ct::String beforeRow = rig.snap();
    rig.drag(fromX, fromY, textX(rig, a), rowY(rig, a));
    Node* second = a->findChild("crate");
    CHECK(second != nullptr);
    roundTrip(rig, "Instantiate Scene", beforeRow, rig.snap(),
            [&](bool done)
            {
                expect(done ? a->findChild("crate") == second : second->getParent() == nullptr,
                        "instantiate on row", "parent");
            });
    commit(rig);

    select(rig, nullptr);
    const ct::String beforeApi = rig.snap();
    Node* third = rig.assets.instantiateOn("Assets/Prefabs/Crate.prefab.json", nullptr);
    CHECK(third != nullptr);
    roundTrip(rig, "Instantiate Scene", beforeApi, rig.snap(), Extra());
    commit(rig);
}

void testSavedVersion(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    commit(rig);
    CHECK(!rig.document.dirty());
    ct::String error;

    CHECK(rig.actions.rename(a, "B", error));
    CHECK(rig.document.dirty());
    CHECK(rig.undoKey());
    CHECK(!rig.document.dirty());
    CHECK(rig.redoKey());
    CHECK(rig.document.dirty());
    CHECK(rig.document.saveSceneAs("saved_version"));
    CHECK(!rig.document.dirty());
    CHECK(rig.undoKey());
    CHECK(rig.document.dirty());
    CHECK(rig.redoKey());
    CHECK(!rig.document.dirty());

    CHECK(rig.undoKey());
    CHECK(rig.actions.rename(a, "C", error));
    CHECK(rig.document.dirty());
    CHECK(rig.undoKey());
    CHECK(rig.document.dirty());
    CHECK(rig.undoKey());
    CHECK(rig.document.dirty());
    CHECK(rig.redoKey());
    CHECK(rig.redoKey());
    CHECK(rig.document.dirty());
    CHECK(rig.document.saveScene());
    CHECK(!rig.document.dirty());

    a->setPosition(Math::Vec3(1, 1, 1));
    rig.document.sceneEdited();
    CHECK(rig.document.dirty());
    CHECK(rig.undoKey());
    CHECK(rig.document.dirty());
    CHECK(rig.document.saveScene());
    CHECK(!rig.document.dirty());
}

void testUi(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    commit(rig);
    showInspector(rig);

    rt::ToolBar* bar = rig.shell.toolBar();
    const int undoId = rig.ui.buttonId(ProjectUi::kUndo);
    const int redoId = rig.ui.buttonId(ProjectUi::kRedo);
    CHECK(undoId >= 0 && redoId >= 0 && undoId != redoId);
    CHECK(isAtlasIcon(ProjectUi::buttonIcon(ProjectUi::kUndo)));
    CHECK(isAtlasIcon(ProjectUi::buttonIcon(ProjectUi::kRedo)));
    CHECK(ProjectUi::buttonIcon(ProjectUi::kUndo) == EditorIcons::get("UndoRedo"));
    CHECK(ProjectUi::buttonIcon(ProjectUi::kRedo) == EditorIcons::get("Redo"));
    CHECK(strcmp(ProjectUi::buttonTooltip(ProjectUi::kUndo), "Undo (Ctrl+Z)") == 0);
    CHECK(strcmp(ProjectUi::buttonTooltip(ProjectUi::kRedo), "Redo (Ctrl+Y)") == 0);
    CHECK(!bar->isItemEnabled(undoId) && !bar->isItemEnabled(redoId));

    ct::String error;
    CHECK(rig.actions.rename(a, "B", error));
    rig.frames(3);
    CHECK(bar->isItemEnabled(undoId) && !bar->isItemEnabled(redoId));
    rt::Menu* edit = rig.menus.menu(EditorMenus::kEdit);
    CHECK(edit->action(EditorMenus::kEditUndo)->text() == "Undo Rename Node");
    CHECK(edit->action(EditorMenus::kEditUndo)->isEnabled());
    CHECK(edit->action(EditorMenus::kEditUndo)->shortcut() == "Ctrl+Z");
    CHECK(edit->action(EditorMenus::kEditRedo)->text() == "Redo");
    CHECK(!edit->action(EditorMenus::kEditRedo)->isEnabled());
    CHECK(edit->action(EditorMenus::kEditRedo)->shortcut() == "Ctrl+Y");
    CHECK(edit->action(EditorMenus::kEditSepHistory)->isSeparator());

    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditUndo);
    CHECK(a->getName() == "A");
    CHECK(bar->isItemEnabled(redoId) && !bar->isItemEnabled(undoId));
    CHECK(edit->action(EditorMenus::kEditRedo)->text() == "Redo Rename Node");
    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditRedo);
    CHECK(a->getName() == "B");

    bar->buttonClicked.emit(undoId);
    rig.frames(2);
    CHECK(a->getName() == "A");
    bar->buttonClicked.emit(redoId);
    rig.frames(2);
    CHECK(a->getName() == "B");

    select(rig, a);
    rig.frames(3);
    const std::size_t count = rig.history().undoCount();
    const ct::String state = rig.snap();
    rig.click(centerX(rig.inspector.nameInput()), centerY(rig.inspector.nameInput()));
    CHECK(WidgetHost::textFocused());
    CHECK(!rig.send(key(Key::Z, true, kModControl)));
    rig.send(key(Key::Z, false, kModControl));
    CHECK(!rig.send(key(Key::Y, true, kModControl)));
    rig.send(key(Key::Y, false, kModControl));
    CHECK(!rig.send(key(Key::Z, true, kModControl | kModShift)));
    rig.send(key(Key::Z, false, kModControl | kModShift));
    rig.frames(2);
    CHECK(rig.history().undoCount() == count && rig.snap() == state);
    rt::WidgetApp::instance().setFocused(nullptr);
    rig.frames(2);
    CHECK(!WidgetHost::textFocused());

    rig.actions.requestRename();
    rig.frames(3);
    CHECK(rig.actions.renameBox() != nullptr);
    CHECK(!rig.send(key(Key::Z, true, kModControl)));
    rig.send(key(Key::Z, false, kModControl));
    CHECK(rig.history().undoCount() == count);
    rig.tap(Key::Escape);
    CHECK(rig.actions.renameBox() == nullptr);
    rig.frames(2);
}

void testPlay(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    commit(rig);
    showInspector(rig);

    ct::String error;
    select(rig, b);
    CHECK(rig.actions.rename(a, "A1", error));
    CHECK(rig.actions.deleteSelection(error));
    CHECK(b->getParent() == nullptr);
    CHECK(rig.history().undoCount() == 2);
    const std::uint64_t version = rig.history().version();
    const ct::String names1 = rig.history().currentActionName();
    const ct::String snapshot = rig.snap();
    const bool dirty = rig.document.dirty();
    CHECK(dirty);

    CHECK(rig.play.play());
    CHECK(rig.document.playing());
    CHECK(rig.history().frozen());
    CHECK(rig.document.scene() != scene);
    CHECK(rig.document.dirty() == dirty);
    rig.undoKey();
    rig.redoKey();
    CHECK(rig.history().version() == version && rig.history().undoCount() == 2);
    CHECK(!rig.document.undo() && !rig.document.redo());
    CHECK(!rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditUndo)->isEnabled());
    CHECK(!rig.shell.toolBar()->isItemEnabled(rig.ui.buttonId(ProjectUi::kUndo)));

    Node* played = rig.document.scene()->findChild("A1");
    CHECK(played != nullptr && played != a);
    if (played)
    {
        select(rig, played);
        rig.frames(3);
        typeInto(rig, rig.inspector.field(InspectorPanel::kPosition, 0), "9");
        CHECK(static_cast<Node3D*>(played)->getPosition().x == 9.0f);
        CHECK(static_cast<Node3D*>(played)->getPosition().x == 9.0f);
        rig.actions.rename(played, "Played", error);
    }
    CHECK(rig.history().version() == version);
    CHECK(rig.history().undoCount() == 2);
    CHECK(rig.history().frozen());

    CHECK(rig.play.stop());
    CHECK(!rig.document.playing());
    CHECK(!rig.history().frozen());
    CHECK(rig.document.scene() == scene);
    CHECK(rig.history().version() == version);
    CHECK(rig.history().undoCount() == 2);
    CHECK(rig.history().currentActionName() == names1);
    CHECK(rig.document.dirty() == dirty);
    CHECK(rig.snap() == snapshot);
    CHECK(b->getParent() == nullptr && !b->isQueuedForDeletion());
    CHECK(a->getName() == "A1");

    CHECK(rig.undoKey());
    CHECK(b->getParent() == scene && b->getIndex() == 1);
    CHECK(rig.undoKey());
    CHECK(a->getName() == "A");
    CHECK(!rig.document.dirty());
    CHECK(rig.redoKey());
    CHECK(rig.redoKey());
    CHECK(rig.snap() == snapshot);
    commit(rig);
}

void testClearOnNew(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    commit(rig);
    select(rig, a);
    ct::String error;
    CHECK(rig.actions.deleteSelection(error));
    CHECK(rig.actions.rename(b, "B2", error));
    CHECK(rig.history().hasUndo());
    CHECK(rig.history().undoCount() == 2);
    CHECK(!a->isQueuedForDeletion());

    CHECK(rig.document.newScene());
    CHECK(a->isQueuedForDeletion());
    CHECK(!rig.history().hasUndo() && !rig.history().hasRedo());
    CHECK(rig.history().undoCount() == 0 && rig.history().redoCount() == 0);
    CHECK(!rig.document.dirty());
    rig.frames(3);
    const std::uint64_t cleared = rig.history().version();
    rig.undoKey();
    CHECK(rig.history().version() == cleared);
    CHECK(!rig.menus.action(EditorMenus::kEdit, EditorMenus::kEditUndo)->isEnabled());

    Node* scene2 = rig.document.scene();
    Node3D* c = add(scene2, "C");
    CHECK(rig.actions.rename(c, "D", error));
    CHECK(rig.history().hasUndo());
    CHECK(rig.document.saveSceneAs("reopen"));
    CHECK(rig.undoKey());
    CHECK(rig.document.openScene(rig.document.scenePath()));
    CHECK(!rig.history().hasUndo() && !rig.history().hasRedo());
    CHECK(!rig.document.dirty());
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;
    if (argc > 2) gOnly = argv[2];

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor undo test";
    desc.driver = useVulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;
    Engine::singleton()->setEditorHint(true);

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

    testCoreOrder();
    testCoreProperties();
    testCoreVersion();
    testCoreMerge();
    testCoreHistoryLimit();
    testCoreReferences();
    testManager();
    tree.flushDeleteQueue();

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
        document.undoRedo().history().setClock([] { return testClock(); });
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
        CHECK(panel.attach(shell.panel(EditorShell::kScenePanel)));
        panel.setScenario(window->getScenario());
        panel.connectPreDraw();

        HierarchyActions actions(tree, document, selection, hierarchy);
        CHECK(actions.attach());
        actions.setViewport(&panel);

        GamePanel game(server, document);
        CHECK(game.attach(shell.panel(EditorShell::kGamePanel)));
        game.setScenario(window->getScenario());
        game.connectPreDraw();
        EditorPlay play(tree, document);
        CHECK(play.attach(shell.runBar()));
        play.setViewHandler(
                [&shell](bool showGame) {
                    shell.showPanel(showGame ? EditorShell::kGamePanel : EditorShell::kScenePanel);
                });

        EditorMenus menus(shell, ui, panel, actions, &play);
        CHECK(menus.attach());
        EditorStatusBar status(document, selection, server);
        CHECK(status.attach(shell.statusBar()));
        status.setGizmo(gizmo);

        MaterialPanel materials(server, tree, selection, document);
        CHECK(materials.attach(shell.panel(EditorShell::kMaterialPanel)));
        materials.bindViewport(panel);
        AssetsPanel assets(document, selection, hierarchy, actions, ui);
        CHECK(assets.attach(shell.panel(4)));
        assets.bindViewport(panel);
        assets.bindMaterials(materials);
        materials.setRevealHandler([&shell] { shell.dock()->showPanel("Material"); });

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, panel,
            actions, menus, status, assets, materials, game, play };
        CHECK(shell.setLayout("H200[{Hierarchy}|H780[V450[{Scene,Game/0}|{Console,Assets/1}]|"
                              "{Inspector,Material/0}]]"));
        rig.frames(4);

        CHECK(document.createProject(testRoot() + "/projects", "Undo"));
        rig.frames(3);
        CHECK(!document.dirty());

        if (wanted("rename")) testRename(rig);
        if (wanted("duplicate")) testDuplicate(rig);
        if (wanted("delete")) testDelete(rig);
        if (wanted("deletelifetime")) testDeleteLifetime(rig);
        if (wanted("move")) testMove(rig);
        if (wanted("reparent")) testReparent(rig);
        if (wanted("reparenttonew")) testReparentToNew(rig);
        if (wanted("addnode")) testAddNode(rig);
        if (wanted("createmenu")) testCreateMenu(rig);
        if (wanted("inspectorfields")) testInspectorFields(rig);
        if (wanted("inspectorproperty")) testInspectorProperty(rig);
        if (wanted("header")) testHeader(rig);
        if (wanted("eye")) testEye(rig);
        if (wanted("gizmo")) testGizmo(rig);
        if (wanted("material")) testMaterial(rig);
        if (wanted("scriptdrop")) testScriptDrop(rig);
        if (wanted("instantiate")) testInstantiate(rig);
        if (wanted("savedversion")) testSavedVersion(rig);
        if (wanted("ui")) testUi(rig);
        if (wanted("play")) testPlay(rig);
        if (wanted("clearonnew")) testClearOnNew(rig);

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
