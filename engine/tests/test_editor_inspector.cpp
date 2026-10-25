#include "Check.h"
#include "EditorIconCheck.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/editor/ClassIcons.h"
#include "prisma3d/editor/EditorChange.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include <igui/widgets/WidgetApp.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 1280;
const int kHeight = 720;
const float kEpsilon = 1.0e-3f;

struct Editor
{
    SceneTree& tree;
    WidgetHost& host;
    EditorSelection& selection;
    InspectorPanel& panel;
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

InputEvent keyEvent(Key key, bool pressed)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = key;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = false;
    event.key.modifiers = 0;
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

void clickField(Editor& editor, NumberField* field)
{
    click(editor, centerX(field), centerY(field));
}

void typeInto(Editor& editor, NumberField* field, const char* text)
{
    clickField(editor, field);
    CHECK(field->isEditing());
    typeText(editor, text);
    pressKey(editor, Key::Enter);
}

void dragField(Editor& editor, NumberField* field, int dx, int steps)
{
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

bool near(float a, float b, float epsilon = kEpsilon)
{
    return fabsf(a - b) <= epsilon;
}

bool sameAngle(float a, float b, float epsilon = kEpsilon)
{
    const float radians = (a - b) * 0.017453292519943295f;
    return fabsf(sinf(radians)) <= epsilon && cosf(radians) > 0.0f;
}

bool vecNear(const Math::Vec3& a, const Math::Vec3& b, float epsilon = kEpsilon)
{
    return near(a.x, b.x, epsilon) && near(a.y, b.y, epsilon) && near(a.z, b.z, epsilon);
}

Math::Vec3 fieldVector(const InspectorPanel& panel, int row)
{
    return Math::Vec3(panel.field(row, 0)->value(), panel.field(row, 1)->value(),
            panel.field(row, 2)->value());
}

bool fieldsShow(const InspectorPanel& panel, int row, const Math::Vec3& expected,
        float epsilon = kEpsilon)
{
    return vecNear(fieldVector(panel, row), expected, epsilon);
}

bool allShow(const InspectorPanel& panel, Node3D* node)
{
    return fieldsShow(panel, InspectorPanel::kPosition, node->getPosition()) &&
           fieldsShow(panel, InspectorPanel::kRotation, node->getRotationDegrees()) &&
           fieldsShow(panel, InspectorPanel::kScale, node->getScale());
}

float quaternionGap(const Math::Quaternion& a, const Math::Quaternion& b)
{
    return 1.0f - fabsf(a.Dot(b));
}

void select(Editor& editor, Node* node)
{
    editor.selection.clear();
    if (node) editor.selection.add(node);
    frames(editor, 2);
}

Node3D* makeNode(Node* parent, const char* name, const Math::Vec3& position,
        const Math::Vec3& rotation, const Math::Vec3& scale)
{
    Node3D* node = parent->createChild<Node3D>();
    node->setName(name);
    node->setPosition(position);
    node->setRotationDegrees(rotation);
    node->setScale(scale);
    return node;
}

void testEmptyAndHeader(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    frames(editor, 2);
    CHECK(panel.inspected() == nullptr);
    CHECK(panel.showsNoSelection());
    CHECK(!panel.showsHeader() && !panel.showsTransform());
    CHECK(panel.writeCount() == 0);

    Node3D* cube = makeNode(scene, "Cube", Math::Vec3(1.5f, -2.0f, 3.0f),
            Math::Vec3(10.0f, 20.0f, 30.0f), Math::Vec3(1.0f, 2.0f, 3.0f));
    frames(editor, 2);
    CHECK(panel.inspected() == nullptr && panel.showsNoSelection());

    editor.selection.add(cube);
    CHECK(panel.inspected() == nullptr);
    frames(editor, 2);
    CHECK(panel.inspected() == cube);
    CHECK(!panel.showsNoSelection() && panel.showsHeader() && panel.showsTransform());
    CHECK(panel.headerName() == "Cube");
    CHECK(panel.headerClass() == "Node3D");
    CHECK(panel.headerIcon() == panel.icons().icon(cube));
    CHECK(isAtlasIcon(panel.headerIcon()));
    CHECK(panel.headerIcon() == ClassIcons().icon(ct::String("Node3D")));
    CHECK(panel.headerWidget()->absoluteRect().h > 0.0f);

    CHECK(near(panel.field(0, 0)->value(), 1.5f) && near(panel.field(0, 1)->value(), -2.0f) &&
            near(panel.field(0, 2)->value(), 3.0f));
    CHECK(near(panel.field(1, 0)->value(), 10.0f) && near(panel.field(1, 1)->value(), 20.0f) &&
            near(panel.field(1, 2)->value(), 30.0f));
    CHECK(near(panel.field(2, 0)->value(), 1.0f) && near(panel.field(2, 1)->value(), 2.0f) &&
            near(panel.field(2, 2)->value(), 3.0f));
    CHECK(panel.field(0, 0)->text() == "1.5" && panel.field(0, 1)->text() == "-2" &&
            panel.field(0, 2)->text() == "3");
    CHECK(panel.field(1, 0)->text() == "10" && panel.field(1, 1)->text() == "20" &&
            panel.field(1, 2)->text() == "30");
    CHECK(panel.field(2, 0)->text() == "1" && panel.field(2, 1)->text() == "2" &&
            panel.field(2, 2)->text() == "3");
    CHECK(strcmp(panel.field(0, 0)->tag().c_str(), "X") == 0 &&
            strcmp(panel.field(1, 1)->tag().c_str(), "Y") == 0 &&
            strcmp(panel.field(2, 2)->tag().c_str(), "Z") == 0);
    CHECK(allShow(panel, cube));
    CHECK(panel.writeCount() == 0);

    for (int row = 0; row < InspectorPanel::kRowCount; ++row)
    {
        for (int axis = 0; axis < InspectorPanel::kAxisCount; ++axis)
        {
            CHECK(panel.field(row, axis)->absoluteRect().w > 20.0f);
            CHECK(panel.field(row, axis)->dragSensitivity() ==
                    InspectorPanel::dragSensitivity(row));
        }
        CHECK(panel.field(row, 0)->absoluteRect().x < panel.field(row, 1)->absoluteRect().x);
        CHECK(panel.field(row, 1)->absoluteRect().x < panel.field(row, 2)->absoluteRect().x);
        if (row > 0)
            CHECK(panel.field(row - 1, 0)->absoluteRect().y <
                    panel.field(row, 0)->absoluteRect().y);
    }
    CHECK(panel.field(-1, 0) == nullptr && panel.field(0, 3) == nullptr &&
            panel.field(3, 0) == nullptr && panel.field(0, -1) == nullptr);

    frames(editor, 30);
    CHECK(panel.writeCount() == 0);
    CHECK(allShow(panel, cube));

    cube->setName("Renamed");
    frame(editor);
    CHECK(panel.headerName() == "Renamed");
    cube->setName("");
    frame(editor);
    CHECK(panel.headerName() == "Renamed");
    cube->setName("Cube");
    frame(editor);
    CHECK(panel.headerName() == "Cube");

    editor.selection.clear();
    frames(editor, 2);
    CHECK(panel.inspected() == nullptr && panel.showsNoSelection());
    CHECK(!panel.showsHeader() && !panel.showsTransform());
    cube->queueFree();
    frames(editor, 2);
}

void testNonSpatial(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    AnimationPlayer* player = scene->createChild<AnimationPlayer>();
    player->setName("Player");
    select(editor, player);
    CHECK(panel.inspected() == player);
    CHECK(panel.showsHeader() && !panel.showsTransform() && !panel.showsNoSelection());
    CHECK(panel.headerName() == "Player" && panel.headerClass() == "AnimationPlayer");
    CHECK(panel.headerIcon() == panel.icons().icon(player));
    CHECK(!panel.field(0, 0)->isVisible() || !panel.showsTransform());

    Node3D* cube = makeNode(scene, "Cube", Math::Vec3(4.0f, 5.0f, 6.0f), Math::Vec3(),
            Math::Vec3(1.0f, 1.0f, 1.0f));
    select(editor, cube);
    CHECK(panel.showsTransform() && panel.headerClass() == "Node3D");
    CHECK(near(panel.field(0, 0)->value(), 4.0f));
    select(editor, player);
    CHECK(!panel.showsTransform() && panel.headerClass() == "AnimationPlayer");
    select(editor, cube);
    CHECK(panel.showsTransform());
    CHECK(panel.writeCount() == 0);

    editor.selection.clear();
    player->queueFree();
    cube->queueFree();
    frames(editor, 2);
}

struct Edits : EditorChangeListener
{
    int count = 0;

    void sceneEdited() override
    {
        ++count;
    }
};

void commitText(Editor& editor, CommitInput* input, const char* text)
{
    click(editor, centerX(input), centerY(input));
    input->selectAll();
    if (text[0]) typeText(editor, text);
    else
        pressKey(editor, Key::Backspace);
    pressKey(editor, Key::Enter);
}

ct::String groupsOf(const Node* node)
{
    ct::Vector<Node::GroupInfo> groups;
    node->getGroups(groups);
    ct::String text;
    for (std::size_t i = 0; i < groups.size(); ++i)
    {
        if (!text.empty()) text += ",";
        text += groups[i].name;
        text += groups[i].persistent ? "!" : "?";
    }
    return text;
}

void testIdentity(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    Edits edits;
    panel.setChangeListener(&edits);

    Node3D* node =
            makeNode(scene, "Ident", Math::Vec3(), Math::Vec3(), Math::Vec3(1.0f, 1.0f, 1.0f));
    Node3D* sibling =
            makeNode(scene, "Other", Math::Vec3(), Math::Vec3(), Math::Vec3(1.0f, 1.0f, 1.0f));
    node->addToGroup("alpha", true);
    node->addToGroup("temp", false);
    select(editor, node);

    CHECK(panel.showsIdentity());
    CHECK(panel.nameInput()->text() == "Ident");
    CHECK(panel.visibleBox()->isVisible() && panel.visibleBox()->isChecked());
    CHECK(panel.activeBox()->isChecked());
    CHECK(panel.tagInput()->text() == "alpha" && InspectorPanel::tagOf(node) == "alpha");
    CHECK(panel.headerWidget()->absoluteRect().y < panel.nameInput()->absoluteRect().y);
    CHECK(panel.nameInput()->absoluteRect().y < panel.visibleBox()->absoluteRect().y);
    CHECK(panel.visibleBox()->absoluteRect().y < panel.tagInput()->absoluteRect().y);
    CHECK(panel.tagInput()->absoluteRect().y < panel.field(0, 0)->absoluteRect().y);
    CHECK(edits.count == 0);

    commitText(editor, panel.nameInput(), "Renamed");
    CHECK(node->getName() == "Renamed" && panel.nameInput()->text() == "Renamed");
    CHECK(panel.headerName() == "Renamed");
    CHECK(edits.count == 1);
    commitText(editor, panel.nameInput(), "Renamed");
    CHECK(edits.count == 1);

    commitText(editor, panel.nameInput(), "  Spaced  ");
    CHECK(node->getName() == "Spaced");
    commitText(editor, panel.nameInput(), "Other");
    CHECK(node->getName() == "Other2" && panel.nameInput()->text() == "Other2");
    commitText(editor, panel.nameInput(), "");
    CHECK(node->getName() == "Node3D" && panel.nameInput()->text() == "Node3D");
    commitText(editor, panel.nameInput(), "a.b");
    CHECK(node->getName() == "a_b");
    rt::WidgetApp::instance().closePopup();
    frames(editor, 3);
    commitText(editor, panel.nameInput(), "Ident");
    CHECK(node->getName() == "Ident");

    int before = edits.count;
    click(editor, centerX(panel.visibleBox()), centerY(panel.visibleBox()));
    CHECK(!node->isVisible() && !panel.visibleBox()->isChecked() && edits.count == before + 1);
    click(editor, centerX(panel.visibleBox()), centerY(panel.visibleBox()));
    CHECK(node->isVisible() && panel.visibleBox()->isChecked());
    node->setVisible(false);
    frames(editor, 3);
    CHECK(!panel.visibleBox()->isChecked());
    node->setVisible(true);
    frames(editor, 3);
    CHECK(panel.visibleBox()->isChecked());

    before = edits.count;
    click(editor, centerX(panel.activeBox()), centerY(panel.activeBox()));
    CHECK(node->getProcessMode() == Node::ProcessMode::Disabled && !panel.activeBox()->isChecked());
    CHECK(!node->isEnabled() && edits.count == before + 1);
    click(editor, centerX(panel.activeBox()), centerY(panel.activeBox()));
    CHECK(node->getProcessMode() == Node::ProcessMode::Inherit && panel.activeBox()->isChecked());
    CHECK(node->isEnabled());
    node->setProcessMode(Node::ProcessMode::Disabled);
    frames(editor, 3);
    CHECK(!panel.activeBox()->isChecked());
    node->setProcessMode(Node::ProcessMode::Inherit);
    frames(editor, 3);

    before = edits.count;
    commitText(editor, panel.tagInput(), "a, b ,a,,c");
    CHECK(groupsOf(node) == "temp?,a!,b!,c!");
    CHECK(panel.tagInput()->text() == "a, b, c" && edits.count == before + 1);
    CHECK(editor.tree.getNodeCountInGroup("b") == 1 &&
            editor.tree.getNodeCountInGroup("alpha") == 0);
    commitText(editor, panel.tagInput(), "a, b, c");
    CHECK(edits.count == before + 1);
    commitText(editor, panel.tagInput(), "");
    CHECK(groupsOf(node) == "temp?" && panel.tagInput()->text().empty());
    CHECK(!editor.tree.hasGroup("a"));

    select(editor, nullptr);
    CHECK(!panel.showsIdentity());
    AnimationPlayer* player = scene->createChild<AnimationPlayer>();
    player->setName("Player");
    select(editor, player);
    CHECK(panel.showsIdentity() && !panel.visibleBox()->isVisible());
    CHECK(panel.activeBox()->isVisible() && panel.nameInput()->text() == "Player");
    commitText(editor, panel.tagInput(), "npc");
    CHECK(player->isInGroup("npc") && player->getPersistentGroupCount() == 1);

    panel.setChangeListener(nullptr);
    editor.selection.clear();
    player->queueFree();
    node->queueFree();
    sibling->queueFree();
    frames(editor, 3);
}

void testTyping(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    Node3D* node = makeNode(scene, "Typed", Math::Vec3(1.0f, 2.0f, 3.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 1.0f, 1.0f));
    select(editor, node);
    const unsigned int base = panel.writeCount();

    NumberField* x = panel.field(0, 0);
    clickField(editor, x);
    CHECK(x->isEditing() && x->isActive());
    CHECK(x->editor()->isVisible());
    CHECK(x->editor()->text() == "1");
    typeText(editor, "2.5");
    CHECK(x->editor()->text() == "2.5");
    CHECK(node->getPosition().x == 1.0f);
    pressKey(editor, Key::Enter);
    CHECK(!x->isEditing() && !x->editor()->isVisible());
    CHECK(near(node->getPosition().x, 2.5f));
    CHECK(near(x->value(), 2.5f) && x->text() == "2.5");
    CHECK(panel.writeCount() == base + 1);
    CHECK(near(node->getPosition().y, 2.0f) && near(node->getPosition().z, 3.0f));

    typeInto(editor, panel.field(0, 1), "-7.25");
    CHECK(near(node->getPosition().y, -7.25f) && near(node->getPosition().x, 2.5f));
    CHECK(panel.field(0, 1)->text() == "-7.25");
    CHECK(panel.writeCount() == base + 2);

    typeInto(editor, panel.field(2, 2), "0.5");
    CHECK(near(node->getScale().z, 0.5f) && near(node->getScale().x, 1.0f));

    typeInto(editor, panel.field(1, 2), "30");
    CHECK(near(node->getRotationDegrees().z, 30.0f));

    clickField(editor, panel.field(0, 2));
    typeText(editor, "99");
    pressKey(editor, Key::Escape);
    CHECK(!panel.field(0, 2)->isEditing());
    CHECK(near(node->getPosition().z, 3.0f) && near(panel.field(0, 2)->value(), 3.0f));

    const unsigned int before = panel.writeCount();
    clickField(editor, panel.field(0, 2));
    typeText(editor, "-");
    pressKey(editor, Key::Enter);
    CHECK(!panel.field(0, 2)->isEditing());
    CHECK(near(node->getPosition().z, 3.0f));
    clickField(editor, panel.field(0, 2));
    pressKey(editor, Key::Enter);
    CHECK(!panel.field(0, 2)->isEditing() && near(node->getPosition().z, 3.0f));
    clickField(editor, panel.field(0, 2));
    typeText(editor, "1.2.3");
    pressKey(editor, Key::Enter);
    CHECK(near(node->getPosition().z, 3.0f));
    CHECK(panel.writeCount() == before);

    clickField(editor, panel.field(0, 2));
    typeText(editor, "8");
    clickField(editor, panel.field(1, 0));
    CHECK(!panel.field(0, 2)->isEditing() && panel.field(1, 0)->isEditing());
    CHECK(near(node->getPosition().z, 8.0f));
    pressKey(editor, Key::Escape);
    CHECK(!panel.field(1, 0)->isEditing());

    clickField(editor, panel.field(0, 0));
    pressKey(editor, Key::Enter);
    CHECK(near(node->getPosition().x, 2.5f));

    node->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    node->queueFree();
    editor.selection.clear();
    frames(editor, 2);
}

void testOutOfRange(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    Node3D* node = makeNode(scene, "Far", Math::Vec3(), Math::Vec3(), Math::Vec3(1.0f, 1.0f, 1.0f));
    select(editor, node);

    typeInto(editor, panel.field(0, 1), "250");
    CHECK(near(node->getPosition().y, 250.0f));
    CHECK(near(panel.field(0, 1)->value(), 250.0f) && panel.field(0, 1)->text() == "250");
    const unsigned int writes = panel.writeCount();
    frames(editor, 10);
    CHECK(near(panel.field(0, 1)->value(), 250.0f));
    CHECK(panel.writeCount() == writes);

    typeInto(editor, panel.field(0, 0), "-1234.5");
    CHECK(near(node->getPosition().x, -1234.5f) && near(panel.field(0, 0)->value(), -1234.5f));
    typeInto(editor, panel.field(0, 2), "1000000");
    CHECK(near(node->getPosition().z, 1000000.0f));

    typeInto(editor, panel.field(2, 0), "100");
    CHECK(near(node->getScale().x, 100.0f) && near(panel.field(2, 0)->value(), 100.0f));
    typeInto(editor, panel.field(2, 1), "-2");
    CHECK(near(node->getScale().y, -2.0f) && near(panel.field(2, 1)->value(), -2.0f));
    typeInto(editor, panel.field(2, 2), "0");
    CHECK(node->getScale().z == 0.0f && panel.field(2, 2)->value() == 0.0f);

    typeInto(editor, panel.field(1, 0), "720");
    CHECK(sameAngle(node->getRotationDegrees().x, 720.0f));

    node->queueFree();
    editor.selection.clear();
    frames(editor, 2);
}

void testDrag(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    Node3D* node = makeNode(scene, "Dragged", Math::Vec3(1.0f, 0.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 1.0f, 1.0f));
    select(editor, node);

    NumberField* x = panel.field(0, 0);
    dragField(editor, x, 40, 8);
    CHECK(!x->isEditing() && !x->isActive());
    CHECK(near(node->getPosition().x, 1.0f + 40.0f * InspectorPanel::dragSensitivity(0)));
    CHECK(near(x->value(), node->getPosition().x));

    dragField(editor, x, -100, 10);
    CHECK(near(node->getPosition().x, 3.0f - 100.0f * InspectorPanel::dragSensitivity(0)));
    dragField(editor, x, -400, 8);
    CHECK(near(node->getPosition().x, -22.0f));
    CHECK(near(x->value(), -22.0f));
    node->setPosition(Math::Vec3(-2.0f, 0.0f, 0.0f));
    frame(editor);

    NumberField* ry = panel.field(1, 1);
    dragField(editor, ry, 60, 6);
    CHECK(near(node->getRotationDegrees().y, 60.0f * InspectorPanel::dragSensitivity(1)));
    CHECK(near(ry->value(), 30.0f));

    NumberField* sz = panel.field(2, 2);
    dragField(editor, sz, 50, 5);
    CHECK(near(node->getScale().z, 1.0f + 50.0f * InspectorPanel::dragSensitivity(2)));

    const unsigned int writes = panel.writeCount();
    const float before = node->getPosition().y;
    NumberField* y = panel.field(0, 1);
    dragField(editor, y, 2, 2);
    CHECK(y->isEditing());
    CHECK(node->getPosition().y == before && panel.writeCount() == writes);
    pressKey(editor, Key::Escape);
    CHECK(!y->isEditing());

    const int px = centerX(x);
    const int py = centerY(x);
    editor.host.handleEvent(motion(px, py));
    frame(editor);
    editor.host.handleEvent(button(px, py, true));
    frame(editor);
    editor.host.handleEvent(motion(px + 20, py));
    frame(editor);
    CHECK(x->isDragging() && x->isActive());
    const float held = x->value();
    node->setPosition(Math::Vec3(-9.0f, node->getPosition().y, node->getPosition().z));
    node->setRotationDegrees(Math::Vec3(0.0f, 0.0f, 45.0f));
    frames(editor, 3);
    CHECK(x->value() == held);
    CHECK(near(panel.field(1, 2)->value(), 45.0f));
    editor.host.handleEvent(button(px + 20, py, false));
    frames(editor, 3);
    CHECK(!x->isActive());
    CHECK(near(x->value(), node->getPosition().x));

    node->queueFree();
    editor.selection.clear();
    frames(editor, 2);
}

void testCodeChanges(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    Node3D* node = makeNode(scene, "Moved", Math::Vec3(1.0f, 1.0f, 1.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 1.0f, 1.0f));
    select(editor, node);
    const unsigned int base = panel.writeCount();

    node->setPosition(Math::Vec3(7.0f, -8.0f, 9.0f));
    CHECK(near(panel.field(0, 0)->value(), 1.0f));
    frame(editor);
    CHECK(fieldsShow(panel, InspectorPanel::kPosition, Math::Vec3(7.0f, -8.0f, 9.0f)));
    CHECK(panel.field(0, 1)->text() == "-8");

    node->setRotationDegrees(Math::Vec3(15.0f, -25.0f, 35.0f));
    node->setScale(Math::Vec3(2.0f, 3.0f, 4.0f));
    frame(editor);
    CHECK(allShow(panel, node));
    CHECK(near(panel.field(1, 1)->value(), -25.0f));

    node->translate(Math::Vec3(1.0f, 0.0f, 0.0f));
    node->rotateY(0.1f);
    frame(editor);
    CHECK(allShow(panel, node));

    node->setTransform(Math::Mat4::Identity());
    frame(editor);
    CHECK(fieldsShow(panel, InspectorPanel::kPosition, Math::Vec3(0.0f, 0.0f, 0.0f)));
    CHECK(fieldsShow(panel, InspectorPanel::kScale, Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(allShow(panel, node));

    node->setPosition(Math::Vec3(5.0f, 5.0f, 5.0f));
    frame(editor);
    NumberField* held = panel.field(0, 0);
    clickField(editor, held);
    typeText(editor, "42");
    node->setPosition(Math::Vec3(6.0f, 6.0f, 6.0f));
    frames(editor, 3);
    CHECK(held->isEditing());
    CHECK(held->editor()->text() == "42");
    CHECK(near(held->value(), 5.0f));
    CHECK(near(panel.field(0, 1)->value(), 6.0f) && near(panel.field(0, 2)->value(), 6.0f));
    pressKey(editor, Key::Enter);
    CHECK(near(node->getPosition().x, 42.0f));
    CHECK(near(node->getPosition().y, 6.0f) && near(node->getPosition().z, 6.0f));

    clickField(editor, held);
    typeText(editor, "11");
    pressKey(editor, Key::Escape);
    node->setPosition(Math::Vec3(3.0f, 3.0f, 3.0f));
    frame(editor);
    CHECK(near(held->value(), 3.0f));
    CHECK(panel.writeCount() == base + 1);

    clickField(editor, panel.field(0, 0));
    typeText(editor, "77");
    Node3D* other = makeNode(scene, "Other", Math::Vec3(-4.0f, 0.0f, 0.0f), Math::Vec3(),
            Math::Vec3(1.0f, 1.0f, 1.0f));
    select(editor, other);
    CHECK(!panel.field(0, 0)->isEditing());
    CHECK(near(panel.field(0, 0)->value(), -4.0f));
    CHECK(near(node->getPosition().x, 3.0f));

    other->queueFree();
    node->queueFree();
    editor.selection.clear();
    frames(editor, 2);
}

void testSelectionSwitch(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    Node3D* a = makeNode(scene, "A", Math::Vec3(1.0f, 2.0f, 3.0f), Math::Vec3(10.0f, 0.0f, 0.0f),
            Math::Vec3(1.0f, 1.0f, 1.0f));
    Node3D* b = makeNode(scene, "B", Math::Vec3(-1.0f, -2.0f, -3.0f), Math::Vec3(0.0f, 45.0f, 0.0f),
            Math::Vec3(2.0f, 2.0f, 2.0f));
    Node* c = scene->createChild<Node>();
    c->setName("C");
    const unsigned int base = panel.writeCount();

    select(editor, a);
    CHECK(panel.headerName() == "A" && allShow(panel, a));
    select(editor, b);
    CHECK(panel.inspected() == b && panel.headerName() == "B" && allShow(panel, b));
    CHECK(near(panel.field(1, 1)->value(), 45.0f) && near(panel.field(0, 0)->value(), -1.0f));
    select(editor, c);
    CHECK(panel.headerName() == "C" && panel.headerClass() == "Node" && !panel.showsTransform());
    select(editor, a);
    CHECK(panel.inspected() == a && allShow(panel, a));
    CHECK(panel.writeCount() == base);

    editor.selection.clear();
    editor.selection.add(b);
    editor.selection.add(a);
    frames(editor, 2);
    CHECK(panel.inspected() == b && panel.headerName() == "B");
    editor.selection.remove(b);
    frames(editor, 2);
    CHECK(panel.inspected() == a);

    editor.selection.add(a);
    editor.selection.add(a);
    frames(editor, 3);
    CHECK(panel.inspected() == a && allShow(panel, a));

    clickField(editor, panel.field(0, 0));
    typeText(editor, "5");
    editor.selection.clear();
    editor.selection.add(a);
    frames(editor, 3);
    CHECK(panel.field(0, 0)->isEditing());
    CHECK(panel.field(0, 0)->editor()->text() == "5");
    pressKey(editor, Key::Escape);

    Node3D* child = a->createChild<Node3D>();
    child->setName("Child");
    child->setPosition(Math::Vec3(3.0f, 3.0f, 3.0f));
    select(editor, child);
    CHECK(panel.inspected() == child && panel.headerName() == "Child");
    CHECK(near(panel.field(0, 0)->value(), 3.0f));

    a->queueFree();
    editor.tree.process(1.0 / 60.0);
    CHECK(panel.inspected() == nullptr);
    frames(editor, 2);
    CHECK(panel.inspected() == nullptr && panel.showsNoSelection() && !panel.showsTransform());
    CHECK(editor.selection.selectedNodes().empty());

    select(editor, b);
    CHECK(panel.inspected() == b);
    b->queueFree();
    CHECK(panel.inspected() == b);
    editor.tree.process(1.0 / 60.0);
    CHECK(panel.inspected() == nullptr);
    frame(editor);
    CHECK(panel.showsNoSelection() && !panel.showsHeader());

    select(editor, c);
    c->queueFree();
    frames(editor, 3);
    CHECK(panel.inspected() == nullptr && panel.showsNoSelection());
}

void recordGimbal(Editor& editor, Node* scene)
{
    InspectorPanel& panel = editor.panel;
    Node3D* node =
            makeNode(scene, "Gimbal", Math::Vec3(), Math::Vec3(), Math::Vec3(1.0f, 1.0f, 1.0f));
    select(editor, node);

    const float angles[] = { 0.0f, 90.0f, -45.0f, 180.0f };
    const char* const labels[] = { "0", "90", "-45", "180" };
    const char* const axisNames[] = { "X", "Y", "Z" };
    int exact = 0;
    int changed = 0;
    for (int axis = 0; axis < 3; ++axis)
    {
        for (int i = 0; i < 4; ++i)
        {
            node->setRotationDegrees(Math::Vec3(0.0f, 0.0f, 0.0f));
            frame(editor);
            typeInto(editor, panel.field(InspectorPanel::kRotation, axis), labels[i]);
            Math::Vec3 typed(0.0f, 0.0f, 0.0f);
            if (axis == 0) typed.x = angles[i];
            else if (axis == 1)
                typed.y = angles[i];
            else
                typed.z = angles[i];
            const Math::Vec3 shown = fieldVector(panel, InspectorPanel::kRotation);
            const Math::Vec3 back = node->getRotationDegrees();
            printf("rotation %s=%s -> engine (%g, %g, %g)\n", axisNames[axis], labels[i],
                    static_cast<double>(back.x), static_cast<double>(back.y),
                    static_cast<double>(back.z));
            CHECK(vecNear(shown, back));
            const Math::Quaternion wanted =
                    Math::Quaternion::FromEulerAngles(typed.x * 0.017453292519943295f,
                            typed.y * 0.017453292519943295f, typed.z * 0.017453292519943295f);
            CHECK(quaternionGap(wanted, node->getQuaternion()) < 1.0e-6f);
            CHECK(quaternionGap(Math::Quaternion::FromEulerAngles(shown.x * 0.017453292519943295f,
                                        shown.y * 0.017453292519943295f,
                                        shown.z * 0.017453292519943295f),
                          wanted) < 1.0e-6f);
            CHECK(vecNear(shown, typed));
            CHECK(vecNear(back, typed));
            if (vecNear(shown, typed)) ++exact;
            else
                ++changed;
        }
    }
    CHECK(exact == 12 && changed == 0);
    printf("rotation round trip: %d of 12 identical, %d shown as another equal Euler triple\n",
            exact, changed);

    const float sweep[] = { 80.0f, 85.0f, 89.0f, 89.9f, 90.0f, 90.1f, 91.0f, 95.0f, 120.0f,
        179.0f };
    int jumps = 0;
    Math::Vec3 previous(0.0f, 0.0f, 0.0f);
    node->setRotationDegrees(Math::Vec3(0.0f, 0.0f, 0.0f));
    frame(editor);
    for (std::size_t i = 0; i < sizeof(sweep) / sizeof(sweep[0]); ++i)
    {
        node->setRotationDegrees(Math::Vec3(30.0f, 0.0f, 40.0f));
        frame(editor);
        char text[32];
        snprintf(text, sizeof(text), "%g", static_cast<double>(sweep[i]));
        typeInto(editor, panel.field(InspectorPanel::kRotation, 1), text);
        const Math::Vec3 shown = fieldVector(panel, InspectorPanel::kRotation);
        const Math::Quaternion wanted =
                Math::Quaternion::FromEulerAngles(30.0f * 0.017453292519943295f,
                        sweep[i] * 0.017453292519943295f, 40.0f * 0.017453292519943295f);
        CHECK(quaternionGap(wanted, node->getQuaternion()) < 1.0e-6f);
        CHECK(quaternionGap(Math::Quaternion::FromEulerAngles(shown.x * 0.017453292519943295f,
                                    shown.y * 0.017453292519943295f,
                                    shown.z * 0.017453292519943295f),
                      wanted) < 1.0e-5f);
        CHECK(vecNear(shown, Math::Vec3(30.0f, sweep[i], 40.0f)));
        printf("rotation (30, %g, 40) -> shown (%g, %g, %g)\n", static_cast<double>(sweep[i]),
                static_cast<double>(shown.x), static_cast<double>(shown.y),
                static_cast<double>(shown.z));
        if (i > 0 && (fabsf(shown.x - previous.x) > 90.0f || fabsf(shown.z - previous.z) > 90.0f))
            ++jumps;
        previous = shown;
    }
    printf("rotation sweep: %d jump(s) of more than 90 degrees in X or Z\n", jumps);
    CHECK(jumps == 0);

    node->queueFree();
    editor.selection.clear();
    frames(editor, 2);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor inspector test";
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
    Node* scene = tree.getRoot() ? tree.getRoot()->createChild<Node3D>() : nullptr;
    CHECK(scene != nullptr);

    if (scene)
    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        CHECK(host.init());
        EditorShell shell(host);
        CHECK(shell.build());
        {
            EditorSelection selection(tree);
            InspectorPanel panel(tree, selection);
            CHECK(!panel.attached());
            CHECK(!panel.attach(nullptr));
            CHECK(panel.attach(shell.panel(2)));
            CHECK(panel.attached());
            CHECK(!panel.attach(shell.panel(2)));

            Editor editor = { tree, host, selection, panel };
            testEmptyAndHeader(editor, scene);
            testNonSpatial(editor, scene);
            testIdentity(editor, scene);
            testTyping(editor, scene);
            testOutOfRange(editor, scene);
            testDrag(editor, scene);
            testCodeChanges(editor, scene);
            testSelectionSwitch(editor, scene);
            recordGimbal(editor, scene);
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
