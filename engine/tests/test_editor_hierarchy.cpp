#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTreeListener.h"
#include "prisma3d/scene/Viewport.h"

#include <stdio.h>
#include <string.h>

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const float kRowHeight = 22.0f;

struct Recorder : SceneTreeListener
{
    int changed = 0;
    int added = 0;
    int removed = 0;
    int renamed = 0;
    Node* lastAdded = nullptr;
    Node* lastRemoved = nullptr;
    Node* lastRenamed = nullptr;

    void treeChanged() override
    {
        ++changed;
    }

    void nodeAdded(Node* node) override
    {
        ++added;
        lastAdded = node;
    }

    void nodeRemoved(Node* node) override
    {
        ++removed;
        lastRemoved = node;
    }

    void nodeRenamed(Node* node) override
    {
        ++renamed;
        lastRenamed = node;
    }

    void reset()
    {
        changed = added = removed = renamed = 0;
    }

    bool counts(int c, int a, int r, int n) const
    {
        return changed == c && added == a && removed == r && renamed == n;
    }
};

struct SelfRemover : SceneTreeListener
{
    SelfRemover(SceneTree& owner)
            : tree(owner)
    {
    }

    void nodeAdded(Node*) override
    {
        ++added;
        tree.removeListener(this);
    }

    SceneTree& tree;
    int added = 0;
};

struct VictimRemover : SceneTreeListener
{
    VictimRemover(SceneTree& owner, SceneTreeListener* other)
            : tree(owner),
              victim(other)
    {
    }

    void nodeAdded(Node*) override
    {
        ++added;
        tree.removeListener(victim);
    }

    SceneTree& tree;
    SceneTreeListener* victim;
    int added = 0;
};

struct LateAdder : SceneTreeListener
{
    LateAdder(SceneTree& owner, SceneTreeListener* other)
            : tree(owner),
              late(other)
    {
    }

    void nodeAdded(Node*) override
    {
        ++added;
        tree.addListener(late);
    }

    SceneTree& tree;
    SceneTreeListener* late;
    int added = 0;
};

struct SelectionCounter : EditorSelectionListener
{
    int count = 0;

    void selectionChanged() override
    {
        ++count;
    }
};

struct Editor
{
    SceneTree& tree;
    WidgetHost& host;
    EditorSelection& selection;
    SceneTreePanel& panel;
};

void frame(Editor& editor)
{
    editor.tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    editor.selection.update();
    editor.selection.flush();
    editor.panel.update();
    editor.host.frame(1.0f / 60.0f, true);
    Main::rendering()->endFrame(true);
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

int depthOf(const Node* node, const Node* scene)
{
    int depth = 0;
    for (const Node* n = node; n && n != scene; n = n->getParent()) ++depth;
    return depth;
}

bool findRow(Editor& editor, Node* node, Node* wanted, int& row)
{
    if (node == wanted) return true;
    ++row;
    rt::TreeNode* item = editor.panel.itemOf(node);
    if (!item || !item->isExpanded()) return false;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (findRow(editor, node->getChild(i), wanted, row)) return true;
    return false;
}

int rowOf(Editor& editor, Node* wanted)
{
    int row = 0;
    return findRow(editor, editor.panel.editedScene(), wanted, row) ? row : -1;
}

int rowY(Editor& editor, Node* node)
{
    const rt::Rect area = editor.panel.outliner()->absoluteRect();
    return static_cast<int>(
            area.y + static_cast<float>(rowOf(editor, node)) * kRowHeight + kRowHeight * 0.5f);
}

int textX(Editor& editor, Node* node)
{
    const rt::Rect area = editor.panel.outliner()->absoluteRect();
    return static_cast<int>(area.x) + 4 + 16 * depthOf(node, editor.panel.editedScene()) + 24;
}

int triangleX(Editor& editor, Node* node)
{
    const rt::Rect area = editor.panel.outliner()->absoluteRect();
    return static_cast<int>(area.x) + 4 + 16 * depthOf(node, editor.panel.editedScene()) + 4;
}

int eyeX(Editor& editor)
{
    const rt::Rect area = editor.panel.outliner()->absoluteRect();
    return static_cast<int>(area.x + area.w) - 40 + 8;
}

void clickRow(Editor& editor, Node* node)
{
    click(editor, textX(editor, node), rowY(editor, node));
}

void clickTriangle(Editor& editor, Node* node)
{
    click(editor, triangleX(editor, node), rowY(editor, node));
}

void clickEye(Editor& editor, Node* node)
{
    click(editor, eyeX(editor), rowY(editor, node));
}

ct::String expectedLabel(const Node* node)
{
    return node->getName().empty() ? ct::String(node->getClassName()) : node->getName();
}

bool matches(Editor& editor, Node* node, rt::TreeNode* item)
{
    if (!item || editor.panel.itemOf(node) != item || editor.panel.nodeOf(item) != node)
        return false;
    if (!(item->text() == expectedLabel(node))) return false;
    if (item->iconId() != editor.panel.icons().icon(node)) return false;
    std::size_t shown = 0;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        Node* child = node->getChild(i);
        if (child->isInternal())
        {
            if (editor.panel.itemOf(child) != nullptr) return false;
            continue;
        }
        if (shown >= item->children().size()) return false;
        if (!matches(editor, child, item->children()[shown])) return false;
        ++shown;
    }
    return shown == item->children().size();
}

std::size_t countNodes(const Node* node)
{
    std::size_t count = 1;
    for (std::size_t i = 0; i < node->getChildCount(); ++i) count += countNodes(node->getChild(i));
    return count;
}

void testListeners(SceneTree& tree)
{
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return;

    Recorder recorder;
    tree.addListener(&recorder);
    tree.addListener(&recorder);

    Node3D* a = root->createChild<Node3D>();
    CHECK(recorder.counts(1, 1, 0, 0));
    CHECK(recorder.lastAdded == a);

    Node3D* b = root->createChild<Node3D>();
    Node3D* leaf = a->createChild<Node3D>();
    recorder.reset();

    a->setName("renamed");
    CHECK(recorder.counts(1, 0, 0, 1));
    CHECK(recorder.lastRenamed == a);
    a->setName("renamed");
    CHECK(recorder.counts(2, 0, 0, 2));

    recorder.reset();
    Node* extra = root->createChild("Node3D");
    CHECK(extra != nullptr);
    CHECK(recorder.counts(1, 1, 0, 0));
    recorder.reset();
    root->moveChild(extra, 0);
    CHECK(recorder.counts(1, 0, 0, 0));
    root->moveChild(extra, 0);
    CHECK(recorder.counts(1, 0, 0, 0));
    root->moveChild(extra, 1000);
    CHECK(recorder.counts(1, 0, 0, 0));
    root->moveChild(extra, root->getChildCount());
    CHECK(recorder.counts(2, 0, 0, 0));

    recorder.reset();
    CHECK(leaf->reparent(b));
    CHECK(recorder.counts(2, 1, 1, 0));
    CHECK(recorder.lastRemoved == leaf && recorder.lastAdded == leaf);
    CHECK(!leaf->reparent(b));
    CHECK(!leaf->reparent(leaf));
    CHECK(recorder.counts(2, 1, 1, 0));

    Node3D* second = a->createChild<Node3D>();
    Node3D* third = a->createChild<Node3D>();
    CHECK(second != nullptr && third != nullptr);
    recorder.reset();
    CHECK(a->reparent(b));
    CHECK(recorder.counts(4, 3, 3, 0));

    recorder.reset();
    b->queueFree();
    CHECK(recorder.counts(0, 0, 0, 0));
    tree.process(1.0 / 60.0);
    CHECK(recorder.counts(5, 0, 5, 0));
    CHECK(recorder.lastRemoved == b);

    recorder.reset();
    extra->queueFree();
    tree.process(1.0 / 60.0);
    CHECK(recorder.counts(1, 0, 1, 0));

    recorder.reset();
    {
        SelfRemover remover(tree);
        Recorder last;
        tree.addListener(&remover);
        tree.addListener(&last);
        Node* one = root->createChild("Node3D");
        CHECK(recorder.added == 1 && remover.added == 1 && last.added == 1);
        Node* two = root->createChild("Node3D");
        CHECK(recorder.added == 2 && remover.added == 1 && last.added == 2);
        tree.removeListener(&last);
        one->queueFree();
        two->queueFree();
        tree.process(1.0 / 60.0);
        CHECK(last.removed == 0);
    }

    recorder.reset();
    {
        Recorder victim;
        VictimRemover killer(tree, &victim);
        tree.removeListener(&recorder);
        tree.addListener(&killer);
        tree.addListener(&victim);
        tree.addListener(&recorder);
        Node* one = root->createChild("Node3D");
        CHECK(killer.added == 1);
        CHECK(victim.added == 0);
        CHECK(recorder.added == 1);
        Node* two = root->createChild("Node3D");
        CHECK(killer.added == 2 && victim.added == 0 && recorder.added == 2);
        tree.removeListener(&killer);
        one->queueFree();
        two->queueFree();
        tree.process(1.0 / 60.0);
        CHECK(victim.removed == 0);
    }

    recorder.reset();
    {
        Recorder late;
        LateAdder adder(tree, &late);
        tree.addListener(&adder);
        Node* one = root->createChild("Node3D");
        CHECK(adder.added == 1 && late.added == 0);
        Node* two = root->createChild("Node3D");
        CHECK(adder.added == 2 && late.added == 1);
        tree.removeListener(&adder);
        one->queueFree();
        two->queueFree();
        tree.process(1.0 / 60.0);
        CHECK(late.removed == 2);
        tree.removeListener(&late);
    }

    tree.removeListener(&recorder);
    tree.removeListener(&recorder);
    recorder.reset();
    Node* silent = root->createChild("Node3D");
    CHECK(silent != nullptr && recorder.counts(0, 0, 0, 0));
    silent->queueFree();
    tree.process(1.0 / 60.0);
}

void testSelection(SceneTree& tree)
{
    Node* root = tree.getRoot();
    Node* parent = root->createChild<Node3D>();
    Node* child = parent->createChild<Node3D>();
    Node* other = root->createChild<Node3D>();

    {
        EditorSelection selection(tree);
        SelectionCounter counter;
        selection.addListener(&counter);
        selection.addListener(&counter);

        CHECK(!selection.add(nullptr));
        CHECK(selection.selectedNodes().empty());
        CHECK(!selection.isSelected(nullptr) && !selection.isSelected(parent));
        selection.update();
        selection.flush();
        CHECK(counter.count == 0);

        CHECK(selection.add(other));
        CHECK(selection.add(child));
        CHECK(selection.add(parent));
        CHECK(selection.add(child));
        CHECK(selection.selectedNodes().size() == 3);
        CHECK(selection.selectedNodes()[0] == other);
        CHECK(selection.selectedNodes()[1] == child);
        CHECK(selection.selectedNodes()[2] == parent);
        CHECK(selection.isSelected(child) && selection.isSelected(other));
        CHECK(selection.changePending());

        const ct::Vector<Node*> top = selection.topSelectedNodes();
        CHECK(top.size() == 2 && top[0] == other && top[1] == parent);

        selection.update();
        CHECK(counter.count == 0);
        selection.update();
        selection.flush();
        CHECK(counter.count == 1);
        selection.flush();
        selection.update();
        selection.flush();
        CHECK(counter.count == 1);
        CHECK(!selection.changePending());

        selection.remove(other);
        selection.remove(other);
        CHECK(!selection.isSelected(other) && selection.selectedNodes().size() == 2);
        selection.update(false);
        CHECK(counter.count == 2);
        selection.update(false);
        CHECK(counter.count == 2);

        selection.clear();
        CHECK(selection.selectedNodes().empty());
        selection.update();
        selection.clear();
        selection.update();
        selection.flush();
        CHECK(counter.count == 3);

        selection.add(child);
        selection.add(parent);
        selection.update();
        selection.flush();
        CHECK(counter.count == 4);
        child->queueFree();
        tree.process(1.0 / 60.0);
        CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == parent);
        CHECK(!selection.isSelected(child));
        selection.update();
        selection.update();
        selection.flush();
        CHECK(counter.count == 5);

        selection.removeListener(&counter);
        selection.add(other);
        selection.update(false);
        CHECK(counter.count == 5);
    }

    parent->queueFree();
    other->queueFree();
    tree.process(1.0 / 60.0);
}

void testPanel(Editor& editor)
{
    SceneTree& tree = editor.tree;
    EditorSelection& selection = editor.selection;
    SceneTreePanel& panel = editor.panel;
    SelectionCounter counter;
    selection.addListener(&counter);

    Node3D* scene = tree.getRoot()->createChild<Node3D>();
    scene->setName("Scene");
    Node* camera = scene->createChild<Camera3D>();
    camera->setName("Camera");
    Node* mesh = scene->createChild("MeshInstance3D");
    mesh->setName("Mesh");
    Node* part = mesh->createChild<Node3D>();
    part->setName("Part");
    Node* sub = mesh->createChild<Node3D>();
    sub->setName("Sub");
    Node* light = scene->createChild("OmniLight3D");
    light->setName("Light");
    Node* plain = scene->createChild("Node");
    plain->setName("Plain");
    Node3D* mesh3d = mesh->castTo<Node3D>();
    Node3D* camera3d = camera->castTo<Node3D>();
    CHECK(mesh3d && camera3d);
    if (!mesh3d || !camera3d) return;
    mesh3d->setVisible(false);

    CHECK(panel.rebuildCount() == 0 && panel.dirty());
    CHECK(panel.editedScene() == nullptr && panel.itemOf(scene) == nullptr);
    panel.setEditedScene(scene);
    CHECK(panel.editedScene() == scene);
    CHECK(panel.rebuildCount() == 0);
    frame(editor);
    frame(editor);
    CHECK(panel.rebuildCount() == 1 && !panel.dirty());

    rt::Outliner* outliner = panel.outliner();
    CHECK(outliner != nullptr);
    if (!outliner) return;
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    CHECK(panel.itemOf(scene)->parent() == nullptr);
    CHECK(panel.itemOf(part)->parent() == panel.itemOf(mesh));
    CHECK(panel.itemOf(scene)->children().size() == 4);
    CHECK(panel.itemOf(scene)->children()[0]->text() == "Camera");
    CHECK(panel.itemOf(scene)->children()[1]->text() == "Mesh");
    CHECK(panel.itemOf(scene)->children()[2]->text() == "Light");
    CHECK(panel.itemOf(scene)->children()[3]->text() == "Plain");
    CHECK(panel.itemOf(mesh)->children()[0]->text() == "Part");
    CHECK(panel.itemOf(mesh)->children()[1]->text() == "Sub");

    CHECK(panel.itemOf(scene)->iconId() == EditorIcons::get("Node3D"));
    CHECK(panel.itemOf(camera)->iconId() == EditorIcons::get("Camera3D"));
    CHECK(panel.itemOf(mesh)->iconId() == EditorIcons::get("MeshInstance3D"));
    CHECK(panel.itemOf(light)->iconId() == EditorIcons::get("OmniLight3D"));
    CHECK(panel.itemOf(plain)->iconId() == EditorIcons::get("Node"));
    CHECK(panel.itemOf(part)->iconId() == EditorIcons::get("Node3D"));
    CHECK(panel.itemOf(camera)->iconId() != rt::IconId::None);
    CHECK(outliner->selectedNode() == nullptr);
    // The tree opens with only the scene expanded; the checks below walk it all open.
    CHECK(panel.itemOf(scene)->isExpanded() && !panel.itemOf(mesh)->isExpanded());
    Node* const opened[] = { camera, mesh, light, plain, part, sub };
    for (Node* node: opened) panel.itemOf(node)->setExpanded(true);
    CHECK(outliner->isNodeVisible(panel.itemOf(camera)));
    CHECK(!outliner->isNodeVisible(panel.itemOf(mesh)));
    CHECK(outliner->isNodeVisible(panel.itemOf(plain)));
    CHECK(panel.nodeOf(nullptr) == nullptr);
    CHECK(panel.itemOf(tree.getRoot()) == nullptr);
    CHECK(rowOf(editor, sub) == 4 && rowOf(editor, plain) == 6);

    frame(editor);
    frame(editor);
    CHECK(panel.rebuildCount() == 1);

    Node* unnamed = plain->createChild("Node3D", true);
    CHECK(unnamed->getName() == "Node3D");
    Node* generated = plain->createChild("Node3D");
    CHECK(strncmp(generated->getName().c_str(), "@Node3D@", 8) == 0);
    frame(editor);
    CHECK(panel.itemOf(unnamed) && panel.itemOf(unnamed)->text() == "Node3D");
    CHECK(panel.itemOf(generated) && panel.itemOf(generated)->text() == generated->getName());
    unnamed->queueFree();
    generated->queueFree();
    frame(editor);
    mesh3d->setVisible(true);
    plain->setName("Plain");
    frame(editor);
    CHECK(outliner->isNodeVisible(panel.itemOf(mesh)));
    const unsigned int base = panel.rebuildCount();

    Node* added = scene->createChild("Node3D");
    added->setName("Added");
    CHECK(panel.rebuildCount() == base && panel.dirty());
    CHECK(panel.itemOf(added) == nullptr);
    CHECK(panel.itemOf(scene)->children().size() == 4);
    frame(editor);
    CHECK(panel.rebuildCount() == base + 1 && !panel.dirty());
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    CHECK(panel.itemOf(scene)->children().size() == 5);

    frame(editor);
    CHECK(panel.rebuildCount() == base + 1);

    plain->setName("Other");
    CHECK(panel.itemOf(plain)->text() == "Plain");
    frame(editor);
    CHECK(panel.rebuildCount() == base + 2);
    CHECK(panel.itemOf(plain)->text() == "Other");

    added->queueFree();
    CHECK(panel.itemOf(added) != nullptr);
    frame(editor);
    CHECK(panel.rebuildCount() == base + 3);
    CHECK(panel.itemOf(scene)->children().size() == 4);
    CHECK(matches(editor, scene, panel.itemOf(scene)));

    scene->moveChild(light, 0);
    CHECK(panel.rebuildCount() == base + 3);
    frame(editor);
    CHECK(panel.rebuildCount() == base + 4);
    CHECK(panel.itemOf(scene)->children()[0]->text() == "Light");
    CHECK(matches(editor, scene, panel.itemOf(scene)));

    scene->moveChild(light, 3);
    scene->moveChild(light, 0);
    frame(editor);
    CHECK(panel.rebuildCount() == base + 4);
    scene->moveChild(light, 0);
    scene->moveChild(light, 2);
    frame(editor);
    CHECK(panel.rebuildCount() == base + 5);
    CHECK(matches(editor, scene, panel.itemOf(scene)));

    CHECK(sub->reparent(light));
    frame(editor);
    CHECK(panel.rebuildCount() == base + 6);
    CHECK(panel.itemOf(sub)->parent() == panel.itemOf(light));
    CHECK(panel.itemOf(mesh)->children().size() == 1);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    CHECK(sub->reparent(mesh));
    frame(editor);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    const unsigned int before = panel.rebuildCount();

    Node* burst[12];
    for (int i = 0; i < 12; ++i)
    {
        burst[i] = scene->createChild("Node3D");
        burst[i]->setName("Burst");
    }
    for (int i = 0; i < 12; i += 3) burst[i]->queueFree();
    burst[1]->reparent(mesh);
    burst[2]->reparent(light);
    scene->moveChild(plain, 0);
    plain->setName("Moved");
    light->setName("Lit");
    scene->moveChild(light, 1);
    CHECK(panel.rebuildCount() == before);
    frame(editor);
    CHECK(panel.rebuildCount() == before + 1);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    CHECK(panel.itemOf(scene)->children()[0]->text() == "Moved");
    CHECK(panel.itemOf(light)->text() == "Lit");
    frame(editor);
    frame(editor);
    CHECK(panel.rebuildCount() == before + 1);
    plain->setName("Plain");
    light->setName("Light");
    for (int i = 0; i < 12; ++i)
        if (i % 3 != 0) burst[i]->queueFree();
    frame(editor);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    CHECK(panel.itemOf(scene)->children().size() == 4);
    scene->moveChild(plain, 3);
    frame(editor);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    CHECK(panel.itemOf(scene)->children()[3]->text() == "Plain");

    CHECK(counter.count == 0);
    CHECK(selection.selectedNodes().empty());

    clickRow(editor, mesh);
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == mesh);
    CHECK(counter.count == 1);
    CHECK(outliner->selectedNode() == panel.itemOf(mesh));
    frame(editor);
    frame(editor);
    CHECK(counter.count == 1);

    clickRow(editor, light);
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == light);
    CHECK(counter.count == 2);
    CHECK(outliner->selectedNode() == panel.itemOf(light));

    clickRow(editor, sub);
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == sub);
    CHECK(counter.count == 3);

    const unsigned int rebuilds = panel.rebuildCount();
    Node* sibling = scene->createChild("Node3D");
    sibling->setName("Sibling");
    frame(editor);
    CHECK(panel.rebuildCount() == rebuilds + 1);
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == sub);
    CHECK(outliner->selectedNode() == panel.itemOf(sub));
    CHECK(counter.count == 3);
    sibling->queueFree();
    frame(editor);
    CHECK(outliner->selectedNode() == panel.itemOf(sub));
    CHECK(counter.count == 3);

    selection.clear();
    selection.add(light);
    frame(editor);
    CHECK(counter.count == 4);
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == light);
    CHECK(outliner->selectedNode() == panel.itemOf(light));
    frame(editor);
    CHECK(counter.count == 4);

    selection.clear();
    frame(editor);
    CHECK(counter.count == 5);
    CHECK(outliner->selectedNode() == nullptr);
    CHECK(selection.selectedNodes().empty());

    clickTriangle(editor, mesh);
    CHECK(!panel.itemOf(mesh)->isExpanded());
    CHECK(panel.itemOf(scene)->isExpanded());
    CHECK(counter.count == 5);
    CHECK(rowOf(editor, plain) == 4);

    plain->setName("Renamed");
    frame(editor);
    CHECK(panel.itemOf(plain)->text() == "Renamed");
    CHECK(!panel.itemOf(mesh)->isExpanded());
    CHECK(panel.itemOf(scene)->isExpanded());
    CHECK(panel.itemOf(light)->isExpanded());
    CHECK(rowOf(editor, plain) == 4);

    clickRow(editor, camera);
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == camera);
    const int selectedBefore = counter.count;
    plain->setName("Plain");
    frame(editor);
    CHECK(!panel.itemOf(mesh)->isExpanded());
    CHECK(outliner->selectedNode() == panel.itemOf(camera));
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == camera);
    CHECK(counter.count == selectedBefore);

    selection.clear();
    selection.add(part);
    frame(editor);
    CHECK(panel.itemOf(mesh)->isExpanded());
    CHECK(outliner->selectedNode() == panel.itemOf(part));
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == part);

    clickTriangle(editor, mesh);
    CHECK(!panel.itemOf(mesh)->isExpanded());
    plain->setName("Again");
    frame(editor);
    CHECK(!panel.itemOf(mesh)->isExpanded());
    CHECK(outliner->selectedNode() == nullptr);
    CHECK(selection.isSelected(part));
    plain->setName("Plain");
    clickTriangle(editor, mesh);
    CHECK(panel.itemOf(mesh)->isExpanded());
    frame(editor);

    selection.clear();
    selection.add(camera);
    selection.add(mesh);
    selection.add(part);
    frame(editor);
    CHECK(outliner->selectedNode() == panel.itemOf(part));
    CHECK(selection.topSelectedNodes().size() == 2);
    selection.clear();
    frame(editor);

    CHECK(mesh3d->isVisible() && camera3d->isVisible());
    clickRow(editor, camera);
    const int eyeBase = counter.count;
    clickEye(editor, mesh);
    CHECK(!mesh3d->isVisible());
    CHECK(!outliner->isNodeVisible(panel.itemOf(mesh)));
    CHECK(camera3d->isVisible());
    CHECK(outliner->selectedNode() == panel.itemOf(camera));
    CHECK(counter.count == eyeBase);
    CHECK(selection.selectedNodes().size() == 1 && selection.selectedNodes()[0] == camera);
    clickEye(editor, mesh);
    CHECK(mesh3d->isVisible());
    CHECK(outliner->isNodeVisible(panel.itemOf(mesh)));

    clickEye(editor, plain);
    CHECK(outliner->isNodeVisible(panel.itemOf(plain)));

    mesh3d->setVisible(false);
    plain->setName("Hidden");
    frame(editor);
    CHECK(!outliner->isNodeVisible(panel.itemOf(mesh)));
    mesh3d->setVisible(true);
    plain->setName("Plain");
    frame(editor);
    CHECK(outliner->isNodeVisible(panel.itemOf(mesh)));

    selection.clear();
    selection.add(camera);
    selection.add(mesh);
    frame(editor);
    clickEye(editor, mesh);
    CHECK(!mesh3d->isVisible() && !camera3d->isVisible());
    CHECK(!outliner->isNodeVisible(panel.itemOf(camera)));
    CHECK(!outliner->isNodeVisible(panel.itemOf(mesh)));
    clickEye(editor, camera);
    CHECK(mesh3d->isVisible() && camera3d->isVisible());
    CHECK(outliner->isNodeVisible(panel.itemOf(camera)));
    CHECK(selection.selectedNodes().size() == 2);

    selection.clear();
    selection.add(mesh);
    frame(editor);
    CHECK(outliner->selectedNode() == panel.itemOf(mesh));
    const int deleteBase = counter.count;
    const std::size_t nodesBefore = countNodes(scene);
    mesh->queueFree();
    frame(editor);
    CHECK(selection.selectedNodes().empty());
    CHECK(counter.count == deleteBase + 1);
    CHECK(panel.itemOf(mesh) == nullptr && panel.itemOf(part) == nullptr);
    CHECK(outliner->selectedNode() == nullptr);
    CHECK(countNodes(scene) == nodesBefore - 3);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    frame(editor);
    CHECK(counter.count == deleteBase + 1);

    selection.add(light);
    frame(editor);
    CHECK(outliner->selectedNode() == panel.itemOf(light));
    panel.setEditedScene(nullptr);
    frame(editor);
    CHECK(panel.itemOf(scene) == nullptr && outliner->selectedNode() == nullptr);
    CHECK(panel.editedScene() == nullptr);
    panel.setEditedScene(scene);
    frame(editor);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    CHECK(outliner->selectedNode() == panel.itemOf(light));

    Node* hidden = scene->createChild<Node3D>();
    hidden->setName("Hidden");
    hidden->setInternal(true);
    Node* hiddenChild = hidden->createChild<Node3D>();
    hiddenChild->setName("HiddenChild");
    frame(editor);
    frame(editor);
    CHECK(panel.itemOf(hidden) == nullptr && panel.itemOf(hiddenChild) == nullptr);
    CHECK(matches(editor, scene, panel.itemOf(scene)));
    hidden->queueFree();
    frame(editor);
    frame(editor);
    CHECK(matches(editor, scene, panel.itemOf(scene)));

    scene->queueFree();
    frame(editor);
    CHECK(panel.editedScene() == nullptr);
    CHECK(selection.selectedNodes().empty());
    CHECK(panel.itemOf(light) == nullptr);
    frame(editor);
    CHECK(outliner->selectedNode() == nullptr);

    selection.removeListener(&counter);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor hierarchy test";
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

    testListeners(tree);
    testSelection(tree);

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        CHECK(host.init());
        EditorShell shell(host);
        CHECK(shell.build());
        {
            EditorSelection selection(tree);
            SceneTreePanel panel(tree, selection);
            CHECK(!panel.attached());
            CHECK(!panel.attach(nullptr));
            CHECK(panel.attach(shell.panel(0)));
            CHECK(panel.attached() && panel.outliner() != nullptr);
            CHECK(!panel.attach(shell.panel(0)));

            Editor editor = { tree, host, selection, panel };
            testPanel(editor);
        }
        host.shutdown();
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
