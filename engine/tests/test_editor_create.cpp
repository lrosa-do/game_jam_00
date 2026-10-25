#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/CreateNodeDialog.h"
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
#include "prisma3d/editor/ReparentDialog.h"
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
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SceneTreeListener.h"
#include "prisma3d/scene/Terrain3D.h"
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

const char* const kRootText = "This operation can't be done on the tree root.";

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_create_test_vulkan" : "/editor_create_test";
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

bool inList(const ct::Vector<ct::String>& list, const char* value)
{
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i] == value) return true;
    return false;
}

int createDialogCount()
{
    int count = 0;
    const ct::Vector<rt::FloatWindow*>& floats = rt::WidgetApp::instance().floats();
    for (std::size_t i = 0; i < floats.size(); ++i)
        if (rt::widget_cast<CreateNodeDialog>(floats[i])) ++count;
    return count;
}

CreateNodeDialog* openAdd(Rig& rig)
{
    rig.actions.requestAddChild();
    rig.frames(3);
    return rig.actions.createDialog();
}

void closeCreateDialogs(Rig& rig)
{
    while (CreateNodeDialog* box = rig.actions.createDialog()) box->cancel();
    rig.frames(3);
}

void clickCentre(Rig& rig, const rt::Rect& rect)
{
    rig.click(static_cast<int>(rect.x + rect.w * 0.5f), static_cast<int>(rect.y + rect.h * 0.5f));
}

void clickRow(Rig& rig, ActivatableTree* tree, const rt::TreeNode* node)
{
    const ct::Vector<rt::TreeNode*> rows = tree->rows();
    int index = -1;
    for (std::size_t i = 0; i < rows.size(); ++i)
        if (rows[i] == node) index = static_cast<int>(i);
    CHECK(index >= 0);
    if (index < 0) return;
    const rt::Rect area = tree->absoluteRect();
    rig.click(static_cast<int>(area.x + 80.0f),
            static_cast<int>(area.y + static_cast<float>(index) * ActivatableTree::kRowHeight -
                             tree->scrollPosition() + ActivatableTree::kRowHeight * 0.5f));
}

void doubleClickRow(Rig& rig, ActivatableTree* tree, const rt::TreeNode* node)
{
    const ct::Vector<rt::TreeNode*> rows = tree->rows();
    int index = -1;
    for (std::size_t i = 0; i < rows.size(); ++i)
        if (rows[i] == node) index = static_cast<int>(i);
    CHECK(index >= 0);
    if (index < 0) return;
    const rt::Rect area = tree->absoluteRect();
    const int x = static_cast<int>(area.x + 80.0f);
    const int y =
            static_cast<int>(area.y + static_cast<float>(index) * ActivatableTree::kRowHeight -
                             tree->scrollPosition() + ActivatableTree::kRowHeight * 0.5f);
    rig.settle();
    rig.send(motion(x, y));
    rig.frames(2);
    for (int i = 0; i < 2; ++i)
    {
        rig.send(button(x, y, MouseButton::Left, true));
        rig.frame();
        rig.send(button(x, y, MouseButton::Left, false));
        rig.frame();
    }
    rig.frames(3);
}

ct::String settingsFile()
{
    return testRoot() + "/settings.json";
}

void resetCreateSettings(Rig& rig)
{
    rig.document.settings().createFavorites.clear();
    rig.document.settings().createRecent.clear();
}

CreateNodeDialog* standalone(Rig& rig, EditorSettings& settings, const char* preferred)
{
    return rt::WidgetApp::instance().addFloat<CreateNodeDialog>(settings, rig.hierarchy.icons(),
            ct::String(preferred), ct::String());
}

void testTypeList(Rig&)
{
    ct::Vector<ct::String> types;
    CreateNodeDialog::typeList(types);
    CHECK(types.size() > 40);

    for (std::size_t i = 0; i < types.size(); ++i)
    {
        if (i > 0) CHECK(types[i - 1] < types[i]);
        CHECK(NodeClassDB::classExists(types[i]));
        CHECK(NodeClassDB::canInstantiate(types[i]));
        CHECK(NodeClassDB::isParentClass(types[i], "Node"));
        CHECK(CreateNodeDialog::hiddenReason(types[i]) == nullptr);
        CHECK(!CreateNodeDialog::shouldHideType(types[i]));
    }

    const char* const present[] = { "Node", "Node3D", "MeshInstance3D", "Camera3D", "OmniLight3D",
        "SpotLight3D", "DirectionalLight3D", "Terrain3D", "AnimationPlayer", "RigidBody3D",
        "CollisionShape3D" };
    for (const char* name: present) CHECK(inList(types, name));

    const char* const absent[] = { "Light3D", "VisualInstance3D", "PhysicsBody3D", "Joint3D",
        "ParticleCollision3D", "MissingNode", "Window", "Viewport", "NoSuchClass" };
    for (const char* name: absent)
    {
        CHECK(!inList(types, name));
        CHECK(CreateNodeDialog::shouldHideType(name));
    }

    ct::Vector<ct::String> all;
    NodeClassDB::getClassList(all);
    for (std::size_t i = 0; i < all.size(); ++i)
    {
        const bool concrete = NodeClassDB::canInstantiate(all[i]);
        const bool listed = inList(types, all[i].c_str());
        const bool blocked = CreateNodeDialog::hiddenReason(all[i]) != nullptr;
        CHECK(listed == (concrete && !blocked));
    }

    const ct::Vector<ct::String> hidden = CreateNodeDialog::hiddenTypes();
    CHECK(hidden.size() == 2);
    CHECK(inList(hidden, "Window") && inList(hidden, "Viewport"));
    CHECK(CreateNodeDialog::hiddenReason("MissingNode") != nullptr);
    CHECK(CreateNodeDialog::hiddenReason("Window") != nullptr);
    CHECK(CreateNodeDialog::hiddenReason("Viewport") != nullptr);
    CHECK(CreateNodeDialog::hiddenReason("Camera3D") == nullptr);
}

void testScore(Rig& rig)
{
    EditorSettings settings;
    CreateNodeDialog* box = standalone(rig, settings, "Node3D");
    CHECK(box != nullptr);
    if (!box) return;
    rig.frames(2);

    CHECK(box->preferredType() == "Node3D");
    CHECK(box->isTypePreferred("Camera3D"));
    CHECK(box->isTypePreferred("Node3D"));
    CHECK(!box->isTypePreferred("AnimationPlayer"));
    CHECK(!box->isTypePreferred("Node"));

    CHECK(box->scoreType("Node3D", "") == 0.0f);
    CHECK(box->scoreType("Node3D", "zzz") == -1.0f);
    CHECK(box->scoreType("Node3D", "Nx") == -1.0f);
    CHECK(box->scoreType("Node3D", "Node3DD") == -1.0f);
    CHECK(box->scoreType("Node3D", "dn") == -1.0f);
    CHECK(box->scoreType("Node3D", "Node3D") == 1.0f);
    CHECK(box->scoreType("AnimationPlayer", "AnimationPlayer") == 1.0f);
    CHECK(box->scoreType("Node3D", "node3d") < 1.0f);

    CHECK(near(box->scoreType("Node3D", "Node"), 0.504f));
    CHECK(near(box->scoreType("Node3D", "node"), 0.504f));
    CHECK(near(box->scoreType("Node3D", "3D"), 0.144f));
    CHECK(near(box->scoreType("Node3D", "Nd3"), 0.1584f));
    CHECK(near(box->scoreType("AnimationPlayer", "Anim"), 0.22032f));
    CHECK(box->scoreType("Node3D", "Node") > box->scoreType("Node3D", "3D"));
    CHECK(box->scoreType("Node3D", "3D") < box->scoreType("Node3D", "Nd3"));

    settings.setCreateFavorite("Node3D", true);
    CHECK(near(box->scoreType("Node3D", "Node"), 0.63f));
    settings.rememberCreated("Node3D");
    CHECK(near(box->scoreType("Node3D", "Node"), 0.7f));
    CHECK(near(box->scoreType("AnimationPlayer", "Anim"), 0.22032f));
    settings.setCreateFavorite("AnimationPlayer", true);
    settings.rememberCreated("AnimationPlayer");
    CHECK(near(box->scoreType("AnimationPlayer", "Anim"), 0.306f));

    settings.createRecent.clear();
    settings.rememberCreated("Node3D");
    settings.rememberCreated("Camera3D");
    settings.rememberCreated("OmniLight3D");
    settings.rememberCreated("SpotLight3D");
    settings.rememberCreated("AnimationPlayer");
    CHECK(settings.createRecent.size() == 5);
    CHECK(near(box->scoreType("Node3D", "Node"), 0.63f));
    settings.rememberCreated("Node3D");
    CHECK(near(box->scoreType("Node3D", "Node"), 0.7f));

    settings.createFavorites.clear();
    settings.createRecent.clear();
    box->setSearchText("Light3D");
    CHECK(box->selectedType() == "OmniLight3D");
    CHECK(box->canCreate());
    settings.setCreateFavorite("SpotLight3D", true);
    box->setSearchText("Light3D");
    CHECK(box->selectedType() == "SpotLight3D");
    settings.setCreateFavorite("SpotLight3D", false);
    box->setSearchText("Light3D");
    CHECK(box->selectedType() == "OmniLight3D");
    settings.rememberCreated("SpotLight3D");
    box->setSearchText("Light3D");
    CHECK(box->selectedType() == "SpotLight3D");
    settings.createFavorites.clear();
    settings.createRecent.clear();

    box->setSearchText("Cam");
    CHECK(box->selectedType() == "Camera3D");
    CHECK(near(box->scoreType("Camera3D", "Cam"), 0.315f));
    box->setSearchText("Node3D");
    CHECK(box->selectedType() == "Node3D");
    box->setSearchText("camera3d");
    CHECK(box->selectedType() == "Camera3D");
    box->setSearchText("MshInst");
    CHECK(box->selectedType() == "MeshInstance3D");

    box->cancel();
    rig.frames(3);

    EditorSettings other;
    CreateNodeDialog* plain = standalone(rig, other, "");
    CHECK(plain != nullptr);
    if (plain)
    {
        CHECK(!plain->isTypePreferred("Node3D"));
        CHECK(near(plain->scoreType("Node3D", "Node"), 0.7f * 0.9f * 0.8f * 0.9f));
        plain->cancel();
    }
    rig.frames(3);
}

void testTree(Rig& rig)
{
    EditorSettings settings;
    CreateNodeDialog* box = standalone(rig, settings, "Node3D");
    CHECK(box != nullptr);
    if (!box) return;
    rig.frames(3);

    const rt::Rect frame = box->absoluteRect();
    for (const rt::Widget* part: { static_cast<const rt::Widget*>(box->optionsTree()),
             static_cast<const rt::Widget*>(box->favoritesTree()),
             static_cast<const rt::Widget*>(box->recentTree()),
             static_cast<const rt::Widget*>(box->searchInput()),
             static_cast<const rt::Widget*>(box->favoriteButton()),
             static_cast<const rt::Widget*>(box->createButton()),
             static_cast<const rt::Widget*>(box->cancelButton()) })
    {
        const rt::Rect area = part->absoluteRect();
        CHECK(area.w > 10.0f && area.h > 10.0f);
        CHECK(area.x >= frame.x && area.y >= frame.y);
        CHECK(area.x + area.w <= frame.x + frame.w + 0.5f);
        CHECK(area.y + area.h <= frame.y + frame.h + 0.5f);
    }
    FavoriteButton* star = box->favoriteButton();
    const rt::Rect starRect = star->absoluteRect();
    const rt::Rect searchRect = box->searchInput()->absoluteRect();
    CHECK(near(starRect.h, searchRect.h) && near(starRect.y, searchRect.y));
    CHECK(near(starRect.w, starRect.h));
    CHECK(starRect.x >= searchRect.x + searchRect.w - 0.5f &&
            starRect.x - (searchRect.x + searchRect.w) < 12.0f);
    {
        rt::Widget::Vec2f points[FavoriteButton::kStarVertices];
        star->starVertices(points);
        float minX = points[0].x;
        float maxX = points[0].x;
        float minY = points[0].y;
        float maxY = points[0].y;
        for (int i = 1; i < FavoriteButton::kStarVertices; ++i)
        {
            if (points[i].x < minX) minX = points[i].x;
            if (points[i].x > maxX) maxX = points[i].x;
            if (points[i].y < minY) minY = points[i].y;
            if (points[i].y > maxY) maxY = points[i].y;
        }
        CHECK(fabsf((minX + maxX) * 0.5f - (starRect.x + starRect.w * 0.5f)) < 0.5f);
        CHECK(fabsf((minY + maxY) * 0.5f - (starRect.y + starRect.h * 0.5f)) < 0.5f);
        CHECK(minX > starRect.x + 2.0f && maxX < starRect.x + starRect.w - 2.0f);
        CHECK(minY > starRect.y + 2.0f && maxY < starRect.y + starRect.h - 2.0f);
        CHECK(maxX - minX > starRect.w * 0.4f && maxY - minY > starRect.h * 0.4f);
        CHECK(star->starRadius() > 4.0f);
    }
    CHECK(star->tooltip() == CreateNodeDialog::kFavoriteTooltip);
    CHECK(star->isEnabled());
    CHECK(box->optionsTree()->rect().h > 150.0f);
    CHECK(box->optionsTree()->rect().w > 250.0f);
    CHECK(box->favoritesTree()->absoluteRect().x + box->favoritesTree()->rect().w <=
            box->optionsTree()->absoluteRect().x);
    CHECK(box->favoritesTree()->absoluteRect().y + box->favoritesTree()->rect().h <=
            box->recentTree()->absoluteRect().y + 0.5f);
    CHECK(box->createButton()->absoluteRect().x > box->cancelButton()->absoluteRect().x);

    CHECK(strcmp(CreateNodeDialog::kTitle, "Create New Node") == 0);
    CHECK(box->title() == "Create New Node");
    CHECK(box->searchInput()->placeholder() == "Search");
    CHECK(box->searchText().empty());

    rt::TreeNode* root = box->itemOf("Node");
    CHECK(root != nullptr);
    CHECK(box->optionsTree()->roots().size() == 1);
    CHECK(root && box->optionsTree()->roots()[0] == root);
    CHECK(root && root->isExpanded());
    CHECK(box->selectedType() == "Node");
    CHECK(box->canCreate());
    CHECK(box->createButton()->isEnabled());
    CHECK(box->statusLabel()->text().empty());

    ct::Vector<ct::String> types;
    CreateNodeDialog::typeList(types);
    for (std::size_t i = 0; i < types.size(); ++i) CHECK(box->hasType(types[i]));
    CHECK(!box->hasType("Window"));
    CHECK(!box->hasType("Viewport"));
    CHECK(!box->hasType("MissingNode"));

    rt::TreeNode* spatial = box->itemOf("Node3D");
    rt::TreeNode* visual = box->itemOf("VisualInstance3D");
    rt::TreeNode* mesh = box->itemOf("MeshInstance3D");
    CHECK(spatial && visual && mesh);
    if (spatial && visual && mesh && root)
    {
        CHECK(spatial->parent() == root);
        CHECK(visual->parent() == spatial);
        CHECK(mesh->parent() == visual);
        CHECK(!spatial->isExpanded());
        CHECK(!visual->isExpanded());
        CHECK(!mesh->isExpanded());
        CHECK(box->isInstantiable("Node3D"));
        CHECK(!box->isInstantiable("VisualInstance3D"));
        CHECK(box->isInstantiable("MeshInstance3D"));
    }

    const ct::Vector<rt::TreeNode*> collapsed = box->optionsTree()->rows();
    for (std::size_t i = 0; i < collapsed.size(); ++i)
        CHECK(collapsed[i] == root || collapsed[i]->parent() == root);

    if (root)
    {
        for (std::size_t i = 0; i < root->children().size(); ++i)
        {
            rt::TreeNode* child = root->children()[i];
            const bool instantiable = NodeClassDB::canInstantiate(child->text());
            CHECK(child->isExpanded() == !instantiable);
        }
    }

    box->setSearchText("Light");
    CHECK(box->itemOf("OmniLight3D") != nullptr);
    CHECK(box->itemOf("SpotLight3D") != nullptr);
    CHECK(box->itemOf("DirectionalLight3D") != nullptr);
    rt::TreeNode* abstractLight = box->itemOf("Light3D");
    CHECK(abstractLight != nullptr);
    CHECK(box->itemOf("OmniLight3D") && box->itemOf("OmniLight3D")->parent() == abstractLight);
    CHECK(abstractLight && abstractLight->parent() == box->itemOf("Node3D"));
    CHECK(box->itemOf("Node3D") && box->itemOf("Node3D")->parent() == box->itemOf("Node"));
    CHECK(box->itemOf("MeshInstance3D") == nullptr);
    CHECK(box->itemOf("Camera3D") == nullptr);
    CHECK(box->itemOf("Node")->isExpanded());
    CHECK(box->itemOf("Node3D")->isExpanded());
    CHECK(abstractLight && abstractLight->isExpanded());
    CHECK(!box->isInstantiable("Light3D"));

    CHECK(box->selectType("Light3D"));
    CHECK(box->selectedType() == "Light3D");
    CHECK(!box->canCreate());
    CHECK(!box->createButton()->isEnabled());
    CHECK(box->createButton()->tooltip() == CreateNodeDialog::kNotInstantiableTooltip);
    box->confirm();
    rig.frames(2);
    CHECK(rig.actions.createDialog() == box);
    CHECK(settings.createRecent.size() == 0);
    rig.tap(Key::Enter);
    CHECK(rig.actions.createDialog() == box);

    CHECK(box->selectType("OmniLight3D"));
    CHECK(box->canCreate());
    CHECK(box->createButton()->isEnabled());
    CHECK(box->createButton()->tooltip().empty());
    CHECK(!box->selectType("MeshInstance3D"));
    CHECK(box->selectedType() == "OmniLight3D");

    box->setSearchText("zzzz");
    CHECK(box->selectedType().empty());
    CHECK(!box->canCreate());
    CHECK(!box->createButton()->isEnabled());
    CHECK(box->noResultsText() == "No results for \"zzzz\".");
    CHECK(box->statusLabel()->text() == "No results for \"zzzz\".");
    CHECK(!box->favoriteButton()->isEnabled());
    CHECK(box->optionsTree()->selectedNode() == nullptr);
    CHECK(box->optionsTree()->roots().size() == 1);
    CHECK(box->optionsTree()->roots()[0]->children().empty());
    box->confirm();
    rig.tap(Key::Enter);
    CHECK(rig.actions.createDialog() == box);
    CHECK(settings.createRecent.size() == 0);

    box->setSearchText("");
    CHECK(box->selectedType() == "Node");
    CHECK(box->canCreate());
    CHECK(box->noResultsText().empty());
    CHECK(box->favoriteButton()->isEnabled());
    CHECK(!box->itemOf("Node3D")->isExpanded());

    box->cancel();
    rig.frames(3);
    CHECK(rig.actions.createDialog() == nullptr);
}

void testKeys(Rig& rig)
{
    resetCreateSettings(rig);
    freshScene(rig);
    CreateNodeDialog* box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;

    rt::WidgetApp& app = rt::WidgetApp::instance();
    CHECK(app.focused() == box->searchInput());
    ActivatableTree* tree = box->optionsTree();
    const ct::Vector<rt::TreeNode*> rows = tree->rows();
    CHECK(rows.size() >= 4);
    if (rows.size() < 4) return;
    CHECK(box->selectedType() == "Node");

    rig.tap(Key::Up);
    CHECK(box->selectedType() == "Node");
    rig.tap(Key::Down);
    CHECK(box->selectedType() == rows[1]->text());
    rig.tap(Key::Down);
    CHECK(box->selectedType() == rows[2]->text());
    rig.tap(Key::Up);
    CHECK(box->selectedType() == rows[1]->text());
    rig.tap(Key::PageDown);
    const std::size_t page = static_cast<std::size_t>(tree->pageRows());
    CHECK(page >= 1);
    std::size_t expected = 1 + page;
    if (expected >= rows.size()) expected = rows.size() - 1;
    CHECK(box->selectedType() == rows[expected]->text());
    for (int i = 0; i < 6; ++i) rig.tap(Key::PageDown);
    CHECK(box->selectedType() == rows[rows.size() - 1]->text());
    rig.tap(Key::Down);
    CHECK(box->selectedType() == rows[rows.size() - 1]->text());
    for (int i = 0; i < 6; ++i) rig.tap(Key::PageUp);
    CHECK(box->selectedType() == "Node");
    CHECK(app.focused() == box->searchInput());
    CHECK(box->searchText().empty());

    CHECK(box->selectType("Node3D"));
    rt::TreeNode* spatial = box->itemOf("Node3D");
    CHECK(spatial && !spatial->isExpanded());
    rig.tap(Key::Space);
    CHECK(spatial && spatial->isExpanded());
    rig.send(character(' '));
    rig.frames(2);
    CHECK(box->searchText().empty());
    rig.tap(Key::Space);
    CHECK(spatial && !spatial->isExpanded());
    rig.typeText("a b");
    CHECK(box->searchText() == "ab");

    box->setSearchText("Camera");
    CHECK(box->selectedType() == "Camera3D");
    rig.tap(Key::Up);
    CHECK(box->selectedType() == "Node3D");
    rig.tap(Key::Down);
    CHECK(box->selectedType() == "Camera3D");
    rig.tap(Key::Escape);
    CHECK(rig.actions.createDialog() == nullptr);
    rig.frames(2);
    CHECK(createDialogCount() == 0);
}

void testEditorSettingsFile(Rig&)
{
    const ct::String file = testRoot() + "/settings_lists.json";

    EditorSettings saved;
    for (int i = 0; i < 20; ++i)
    {
        ct::String name = "Type";
        name.append_number(i);
        saved.rememberCreated(name);
    }
    CHECK(saved.createRecent.size() == EditorSettings::kMaxRecentCreated);
    CHECK(saved.createRecent[0] == "Type19");
    CHECK(saved.createRecent[14] == "Type5");
    saved.rememberCreated("Type10");
    CHECK(saved.createRecent.size() == 15);
    CHECK(saved.createRecent[0] == "Type10");
    CHECK(saved.createRecent[1] == "Type19");
    CHECK(saved.createRecent[10] == "Type9");
    CHECK(saved.createRecent[14] == "Type5");
    saved.rememberCreated("");
    CHECK(saved.createRecent.size() == 15);

    saved.setCreateFavorite("Camera3D", true);
    saved.setCreateFavorite("OmniLight3D", true);
    saved.setCreateFavorite("Camera3D", true);
    saved.setCreateFavorite("", true);
    CHECK(saved.createFavorites.size() == 2);
    CHECK(saved.isCreateFavorite("Camera3D"));
    CHECK(!saved.isCreateFavorite("Node3D"));
    saved.setCreateFavorite("Node3D", false);
    CHECK(saved.createFavorites.size() == 2);
    saved.rememberProject("/projects/p/projecto.json");
    CHECK(saved.save(file));

    EditorSettings loaded;
    CHECK(loaded.load(file));
    CHECK(loaded.createRecent.size() == 15);
    for (std::size_t i = 0; i < 15; ++i) CHECK(loaded.createRecent[i] == saved.createRecent[i]);
    CHECK(loaded.createFavorites.size() == 2);
    CHECK(loaded.createFavorites[0] == "Camera3D");
    CHECK(loaded.createFavorites[1] == "OmniLight3D");
    CHECK(loaded.lastProjectPath == "/projects/p/projecto.json");
    loaded.setCreateFavorite("Camera3D", false);
    CHECK(loaded.createFavorites.size() == 1);
    CHECK(loaded.createFavorites[0] == "OmniLight3D");

    CHECK(FileSystem::writeText(file,
            "{\"lastProjectPath\": \"/old/projecto.json\", \"recentProjectPaths\": "
            "[\"/old/projecto.json\"]}"));
    EditorSettings old;
    CHECK(old.loadLegacyJson(file));
    CHECK(old.createFavorites.size() == 0);
    CHECK(old.createRecent.size() == 0);
    CHECK(old.lastProjectPath == "/old/projecto.json");
    CHECK(old.recentProjectPaths.size() == 1);

    EditorSettings kept;
    kept.rememberCreated("Node3D");
    kept.setCreateFavorite("Camera3D", true);
    CHECK(kept.loadLegacyJson(file));
    CHECK(kept.createRecent.size() == 1 && kept.createRecent[0] == "Node3D");
    CHECK(kept.createFavorites.size() == 1 && kept.createFavorites[0] == "Camera3D");
    CHECK(kept.save(file));
    EditorSettings again;
    CHECK(again.load(file));
    CHECK(again.createRecent.size() == 1);
    CHECK(again.createFavorites.size() == 1);

    ct::String many = "{\"createRecent\": [";
    for (int i = 0; i < 30; ++i)
    {
        if (i) many += ',';
        many += "\"R";
        many.append_number(i);
        many += '"';
    }
    many += ", 3, \"\", \"R1\"], \"createFavorites\": [\"A\", 4, \"A\", \"\", \"B\"]}";
    CHECK(FileSystem::writeText(file, many));
    EditorSettings tolerant;
    CHECK(tolerant.loadLegacyJson(file));
    CHECK(tolerant.createRecent.size() == EditorSettings::kMaxRecentCreated);
    CHECK(tolerant.createRecent[0] == "R0");
    CHECK(tolerant.createFavorites.size() == 2);
    CHECK(tolerant.createFavorites[0] == "A" && tolerant.createFavorites[1] == "B");

    CHECK(FileSystem::writeText(file, "{\"createRecent\": \"nope\", \"createFavorites\": 3}"));
    EditorSettings wrong;
    wrong.rememberCreated("Node3D");
    CHECK(wrong.loadLegacyJson(file));
    CHECK(wrong.createRecent.size() == 1);
    CHECK(wrong.createFavorites.size() == 0);
}

void testRecentFavorites(Rig& rig)
{
    resetCreateSettings(rig);
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    commit(rig);
    EditorSettings& settings = rig.document.settings();

    select(rig, a);
    CreateNodeDialog* box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->recentTree()->roots().size() == 0);
    CHECK(box->favoritesTree()->roots().size() == 0);
    CHECK(!box->favoriteButton()->isChecked());

    box->setSearchText("Camera3D");
    CHECK(box->selectedType() == "Camera3D");
    clickCentre(rig, box->favoriteButton()->absoluteRect());
    CHECK(settings.createFavorites.size() == 1 && settings.createFavorites[0] == "Camera3D");
    CHECK(box->favoriteButton()->isChecked());
    CHECK(box->favoritesTree()->roots().size() == 1);
    CHECK(box->favoritesTree()->roots()[0]->text() == "Camera3D");
    {
        EditorSettings fromFile;
        CHECK(fromFile.load(settingsFile()));
        CHECK(fromFile.createFavorites.size() == 1);
    }

    box->setSearchText("OmniLight3D");
    box->toggleFavorite();
    CHECK(settings.createFavorites.size() == 2 && settings.createFavorites[1] == "OmniLight3D");
    CHECK(box->favoritesTree()->roots().size() == 2);
    box->toggleFavorite();
    CHECK(settings.createFavorites.size() == 1);
    CHECK(!box->favoriteButton()->isChecked());
    box->setSearchText("OmniLight3D");
    CHECK(!box->favoriteButton()->isChecked());
    box->setSearchText("Camera3D");
    CHECK(box->favoriteButton()->isChecked());

    box->setSearchText("");
    CHECK(box->selectedType() == "Node");
    clickRow(rig, box->favoritesTree(), box->favoritesTree()->roots()[0]);
    CHECK(box->searchText() == "Camera3D");
    CHECK(box->selectedType() == "Camera3D");
    CHECK(box->recentTree()->selectedNode() == nullptr);

    box->setSearchText("Node3D");
    CHECK(box->selectedType() == "Node3D");
    clickRow(rig, box->favoritesTree(), box->favoritesTree()->roots()[0]);
    CHECK(box->searchText() == "Camera3D");
    CHECK(box->selectedType() == "Camera3D");

    box->setSearchText("Camera3D");
    rig.tap(Key::Enter);
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(settings.createRecent.size() == 1 && settings.createRecent[0] == "Camera3D");
    CHECK(a->findChild("Camera3D") != nullptr);
    {
        EditorSettings fromFile;
        CHECK(fromFile.load(settingsFile()));
        CHECK(fromFile.createRecent.size() == 1);
        CHECK(fromFile.createFavorites.size() == 1);
    }

    select(rig, a);
    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->searchText() == "Camera3D");
    CHECK(box->selectedType() == "Camera3D");
    CHECK(box->recentTree()->roots().size() == 1);
    box->setSearchText("Node3D");
    box->confirm();
    rig.frames(3);
    CHECK(settings.createRecent.size() == 2);
    CHECK(settings.createRecent[0] == "Node3D" && settings.createRecent[1] == "Camera3D");

    select(rig, a);
    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->searchText() == "Node3D");
    CHECK(box->recentTree()->roots().size() == 2);
    box->setSearchText("Camera3D");
    box->confirm();
    rig.frames(3);
    CHECK(settings.createRecent.size() == 2);
    CHECK(settings.createRecent[0] == "Camera3D" && settings.createRecent[1] == "Node3D");

    settings.createRecent.clear();
    settings.createRecent.push_back(ct::String("Camera3D"));
    settings.createRecent.push_back(ct::String("Node3D"));
    settings.createRecent.push_back(ct::String("OmniLight3D"));
    select(rig, a);
    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->recentTree()->roots().size() == 3);
    clickRow(rig, box->recentTree(), box->recentTree()->roots()[2]);
    CHECK(box->searchText() == "OmniLight3D");
    CHECK(box->selectedType() == "OmniLight3D");
    CHECK(box->favoritesTree()->selectedNode() == nullptr);
    clickRow(rig, box->recentTree(), box->recentTree()->roots()[1]);
    CHECK(box->searchText() == "Node3D");
    CHECK(box->selectedType() == "Node3D");
    box->recentTree()->activated.emit(box->recentTree()->roots()[0]);
    rig.frames(3);
    CHECK(rig.actions.createDialog() == nullptr);

    resetCreateSettings(rig);
    settings.createFavorites.push_back(ct::String("OmniLight3D"));
    select(rig, a);
    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    doubleClickRow(rig, box->favoritesTree(), box->favoritesTree()->roots()[0]);
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(a->findChild("OmniLight3D") != nullptr);
    CHECK(settings.createRecent.size() == 1 && settings.createRecent[0] == "OmniLight3D");

    select(rig, a);
    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    box->setSearchText("Decal");
    CHECK(box->selectedType() == "Decal3D");
    clickRow(rig, box->optionsTree(), box->itemOf("Node"));
    CHECK(box->selectedType() == "Node");
    doubleClickRow(rig, box->optionsTree(), box->itemOf("Decal3D"));
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(a->findChild("Decal3D") != nullptr);
    CHECK(settings.createRecent[0] == "Decal3D");

    settings.createRecent.clear();
    settings.createRecent.push_back(ct::String("Gone"));
    settings.createRecent.push_back(ct::String("Window"));
    settings.createRecent.push_back(ct::String("Node3D"));
    settings.createFavorites.clear();
    settings.createFavorites.push_back(ct::String("Missing"));
    settings.createFavorites.push_back(ct::String("Camera3D"));
    select(rig, a);
    box = openAdd(rig);
    CHECK(box != nullptr);
    if (box)
    {
        CHECK(box->recentTree()->roots().size() == 2);
        CHECK(box->favoritesTree()->roots().size() == 1);
        CHECK(settings.createFavorites.size() == 1);
        box->cancel();
    }
    rig.frames(3);
    resetCreateSettings(rig);
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

Node* reloaded(Rig& rig, Node* scene, SceneResult& saved, Node*& holder)
{
    ct::String text;
    saved = SceneResult();
    CHECK(SceneSerializer::save(scene, text, &saved));
    holder = rig.tree.getRoot()->createChild<Node3D>();
    holder->setName("holder");
    SceneResult loaded;
    Node* copy = SceneSerializer::load(text, holder, loaded);
    CHECK(copy != nullptr);
    CHECK(loaded.success());
    return copy;
}

void dropHolder(Rig& rig, Node* holder)
{
    if (holder) holder->queueFree();
    rig.frames(3);
}

void testCreateAction(Rig& rig)
{
    resetCreateSettings(rig);
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    commit(rig);
    ct::String error;

    CHECK(rig.actions.canAddChild());
    CHECK(rig.actions.can(HierarchyActions::kAddChild));

    select(rig, a);
    CHECK(rig.actions.createNode("Node3D", error));
    CHECK(error.empty());
    Node* first = a->findChild("Node3D");
    CHECK(first != nullptr);
    CHECK(first && strcmp(first->getClassName(), "Node3D") == 0);
    CHECK(selectedIs(rig, first));
    CHECK(rig.document.dirty());
    commit(rig);

    select(rig, a);
    CHECK(rig.actions.createNode("Node3D", error));
    CHECK(names(a) == "Node3D,Node3D2");
    CHECK(selectedIs(rig, a->findChild("Node3D2")));
    select(rig, a);
    CHECK(rig.actions.createNode("MeshInstance3D", error));
    CHECK(names(a) == "Node3D,Node3D2,MeshInstance3D");
    select(rig, a);
    CHECK(rig.actions.createNode("MeshInstance3D", error));
    CHECK(names(a) == "Node3D,Node3D2,MeshInstance3D,MeshInstance3D2");
    commit(rig);

    select(rig, b, c);
    CHECK(rig.actions.createNode("OmniLight3D", error));
    Node* lightB = b->findChild("OmniLight3D");
    Node* lightC = c->findChild("OmniLight3D");
    CHECK(lightB != nullptr && lightC != nullptr);
    CHECK(selectedIs(rig, lightB, lightC));
    CHECK(rig.selection.selectedNodes().size() == 2);
    CHECK(names(b) == "OmniLight3D" && names(c) == "OmniLight3D");
    commit(rig);

    select(rig, nullptr);
    CHECK(rig.actions.createNode("Camera3D", error));
    Node* camera = scene->findChild("Camera3D");
    CHECK(camera != nullptr);
    CHECK(selectedIs(rig, camera));
    CHECK(names(scene) == "A,B,C,Camera3D");

    select(rig, scene);
    CHECK(rig.actions.createNode("Camera3D", error));
    CHECK(names(scene) == "A,B,C,Camera3D,Camera3D2");
    CHECK(selectedIs(rig, scene->findChild("Camera3D2")));

    select(rig, scene, a);
    CHECK(rig.actions.createNode("Node3D", error));
    CHECK(scene->findChild("Node3D") != nullptr);
    CHECK(names(a) == "Node3D,Node3D2,MeshInstance3D,MeshInstance3D2,Node3D3");
    CHECK(rig.selection.selectedNodes().size() == 2);
    commit(rig);

    const std::size_t count = scene->getChildCount();
    select(rig, a);
    const char* const refused[] = { "Window", "Viewport", "MissingNode", "Light3D",
        "VisualInstance3D", "NoSuchClass", "" };
    for (const char* type: refused)
    {
        CHECK(!rig.actions.createNode(type, error));
        CHECK(!error.empty());
        CHECK(scene->getChildCount() == count);
        CHECK(selectedIs(rig, a));
    }
    CHECK(!rig.document.dirty());

    Node3D* hidden = a->createChild<Node3D>();
    hidden->setName("Hidden");
    hidden->setInternal(true);
    rig.selection.clear();
    rig.selection.add(hidden);
    rig.frames(3);
    CHECK(!rig.actions.createNode("Node3D", error));
    CHECK(hidden->getChildCount() == 0);
}

void testCreateDialogFlow(Rig& rig)
{
    resetCreateSettings(rig);
    Node* scene = freshScene(rig);
    Node3D* b = add(scene, "B");
    commit(rig);
    rt::WidgetApp& app = rt::WidgetApp::instance();

    CHECK(rig.actions.preferredType() == "Node3D");
    select(rig, b);
    CreateNodeDialog* box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->preferredType() == "Node3D");
    CHECK(app.focused() == box->searchInput());
    rig.typeText("OmniLight3D");
    CHECK(box->selectedType() == "OmniLight3D");
    rig.tap(Key::Enter);
    CHECK(rig.actions.createDialog() == nullptr);
    Node* light = b->findChild("OmniLight3D");
    CHECK(light != nullptr);
    CHECK(selectedIs(rig, light));
    CHECK(rig.document.settings().createRecent.size() == 1);
    CHECK(rig.document.dirty());
    commit(rig);

    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->searchText() == "OmniLight3D");
    CHECK(box->selectedType() == "OmniLight3D");
    box->setSearchText("Camera3D");
    CHECK(box->createButton()->isEnabled());
    clickCentre(rig, box->createButton()->absoluteRect());
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(light->findChild("Camera3D") != nullptr);
    CHECK(selectedIs(rig, light->findChild("Camera3D")));
    commit(rig);

    select(rig, b);
    const std::size_t before = b->getChildCount();
    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->searchText() == "Camera3D");
    clickCentre(rig, box->cancelButton()->absoluteRect());
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(b->getChildCount() == before);
    CHECK(rig.document.settings().createRecent[0] == "Camera3D");

    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    rig.tap(Key::Escape);
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(b->getChildCount() == before);

    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    box->closed.emit();
    rig.frames(2);
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(b->getChildCount() == before);

    box = openAdd(rig);
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(openAdd(rig) == box);
    CHECK(createDialogCount() == 1);
    box->cancel();
    rig.frames(2);

    {
        EditorSettings fromFile;
        CHECK(fromFile.load(settingsFile()));
        CHECK(fromFile.createRecent.size() == 2);
        CHECK(fromFile.createRecent[0] == "Camera3D");
        CHECK(fromFile.createRecent[1] == "OmniLight3D");
    }
    resetCreateSettings(rig);
}

void testCreateUnderPrefab(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node* crate = makeCrate(rig, scene, "crate_a.prefab.json", "crateA");
    if (!crate) return;
    Node* lid = crate->findChild("lid");
    CHECK(lid != nullptr);
    if (!lid) return;
    CHECK(lid->getOwner() == crate);
    commit(rig);
    ct::String error;

    Node* loose = lid->createChild<Node3D>();
    loose->setName("loose");
    {
        ct::String text;
        SceneResult saved;
        CHECK(SceneSerializer::save(scene, text, &saved));
        CHECK(saved.warningCount() > 0);
        CHECK(text.find("loose") == ct::String::npos);
    }
    loose->queueFree();
    rig.frames(3);
    CHECK(lid->findChild("loose") == nullptr);

    select(rig, lid);
    CHECK(!scene->isEditableInstance(crate));
    CHECK(rig.actions.createNode("Node3D", error));
    CHECK(error.empty());
    Node* made = lid->findChild("Node3D");
    CHECK(made != nullptr);
    CHECK(selectedIs(rig, made));
    CHECK(made && made->getOwner() == nullptr);
    CHECK(scene->isEditableInstance(crate));
    CHECK(rig.document.dirty());

    SceneResult saved;
    Node* holder = nullptr;
    Node* copy = reloaded(rig, scene, saved, holder);
    CHECK(saved.warningCount() == 0);
    if (copy)
    {
        CHECK(copy->getNode("crateA/lid/Node3D") != nullptr);
        CHECK(copy->getNode("crateA/lid/Node3D") &&
                strcmp(copy->getNode("crateA/lid/Node3D")->getClassName(), "Node3D") == 0);
        CHECK(copy->getNode("crateA/base") != nullptr);
    }
    dropHolder(rig, holder);
    commit(rig);

    select(rig, made);
    CHECK(rig.actions.createNode("OmniLight3D", error));
    CHECK(made->findChild("OmniLight3D") != nullptr);
    copy = reloaded(rig, scene, saved, holder);
    CHECK(saved.warningCount() == 0);
    CHECK(copy && copy->getNode("crateA/lid/Node3D/OmniLight3D") != nullptr);
    dropHolder(rig, holder);

    Node* plain = makeCrate(rig, scene, "crate_b.prefab.json", "crateB");
    if (!plain) return;
    select(rig, plain);
    CHECK(rig.actions.createNode("Camera3D", error));
    CHECK(plain->findChild("Camera3D") != nullptr);
    CHECK(!scene->isEditableInstance(plain));
    copy = reloaded(rig, scene, saved, holder);
    CHECK(saved.warningCount() == 0);
    CHECK(copy && copy->getNode("crateB/Camera3D") != nullptr);
    dropHolder(rig, holder);

    select(rig, crate->findChild("base"), plain->findChild("base"));
    CHECK(rig.actions.createNode("Node3D", error));
    CHECK(crate->findChild("base")->findChild("Node3D") != nullptr);
    CHECK(plain->findChild("base")->findChild("Node3D") != nullptr);
    CHECK(scene->isEditableInstance(plain));
    copy = reloaded(rig, scene, saved, holder);
    CHECK(saved.warningCount() == 0);
    CHECK(copy && copy->getNode("crateB/base/Node3D") != nullptr);
    CHECK(copy && copy->getNode("crateA/base/Node3D") != nullptr);
    dropHolder(rig, holder);
}

void testReparent(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    Node3D* a1 = add(a, "a1");
    Node3D* a2 = add(a, "a2");
    commit(rig);
    ct::String error;

    select(rig, c);
    CHECK(rig.actions.reparentSelection(a, true, error));
    CHECK(error.empty());
    CHECK(names(scene) == "A,B");
    CHECK(names(a) == "a1,a2,C");
    CHECK(c->getParent() == a);
    CHECK(selectedIs(rig, c));
    CHECK(rig.document.dirty());
    commit(rig);

    select(rig, c);
    CHECK(!rig.actions.reparentSelection(a, true, error));
    CHECK(error.empty());
    CHECK(names(a) == "a1,a2,C");
    CHECK(!rig.document.dirty());
    CHECK(selectedIs(rig, c));

    select(rig, a1);
    CHECK(rig.actions.reparentSelection(a, true, error));
    CHECK(names(a) == "a2,C,a1");
    CHECK(selectedIs(rig, a1));
    commit(rig);

    select(rig, c, a1);
    CHECK(!rig.actions.reparentSelection(a, true, error));
    CHECK(names(a) == "a2,C,a1");
    CHECK(!rig.document.dirty());

    select(rig, a2, a1);
    CHECK(rig.actions.reparentSelection(a, true, error));
    CHECK(names(a) == "C,a2,a1");
    CHECK(selectedIs(rig, a2, a1));
    commit(rig);

    select(rig, a);
    CHECK(!rig.actions.reparentSelection(a, true, error));
    CHECK(error.empty());
    CHECK(a->getParent() == scene);
    CHECK(!rig.document.dirty());

    CHECK(!rig.actions.reparentSelection(a2, true, error));
    CHECK(error == HierarchyActions::kSelectionChanged);
    CHECK(a->getParent() == scene);
    CHECK(names(a) == "C,a2,a1");
    CHECK(selectedIs(rig, a));
    CHECK(!rig.document.dirty());

    Node3D* deep = add(a2, "deep");
    rig.frames(2);
    CHECK(!rig.actions.reparentSelection(deep, true, error));
    CHECK(error == HierarchyActions::kSelectionChanged);
    select(rig, a, a2);
    CHECK(!rig.actions.reparentSelection(deep, false, error));
    CHECK(error == HierarchyActions::kSelectionChanged);
    CHECK(a->getParent() == scene && a2->getParent() == a);
    commit(rig);

    select(rig, a, a2);
    CHECK(rig.actions.reparentSelection(b, true, error));
    CHECK(error.empty());
    CHECK(a->getParent() == b && a2->getParent() == a);
    CHECK(names(scene) == "B");
    CHECK(selectedIs(rig, a, a2));
    commit(rig);

    Node3D* x = add(scene, "x");
    Node3D* y = add(scene, "y");
    Node3D* z = add(scene, "z");
    rig.frames(2);
    select(rig, z, x);
    CHECK(rig.actions.reparentSelection(b, true, error));
    CHECK(names(b) == "A,x,z");
    CHECK(selectedIs(rig, z, x));
    CHECK(y->getParent() == scene);
    commit(rig);

    CHECK(!rig.actions.reparentSelection(nullptr, true, error));
    select(rig, nullptr);
    CHECK(!rig.actions.reparentSelection(b, true, error));
    CHECK(error.empty());

    select(rig, y);
    Node3D outsider;
    CHECK(!rig.actions.reparentSelection(&outsider, true, error));
    CHECK(y->getParent() == scene);
}

void testReparentTransform(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* parent = add(scene, "P");
    parent->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    parent->setScale(Math::Vec3(2.0f, 2.0f, 2.0f));
    parent->setRotationDegrees(Math::Vec3(0.0f, 90.0f, 0.0f));
    Node3D* keep = add(scene, "keep");
    keep->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    keep->setRotationDegrees(Math::Vec3(0.0f, 30.0f, 0.0f));
    Node3D* drop = add(scene, "drop");
    drop->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    drop->setRotationDegrees(Math::Vec3(0.0f, 30.0f, 0.0f));
    Node* plain = scene->createChild<Node>();
    plain->setName("plain");
    Node3D* inPlain = add(plain, "inPlain");
    inPlain->setPosition(Math::Vec3(4.0f, 5.0f, 6.0f));
    commit(rig);
    ct::String error;

    const Math::Vec3 probe(1.0f, 0.0f, 0.0f);
    const Math::Vec3 keepPoint = keep->toGlobal(probe);
    const Math::Vec3 keepOrigin = keep->getGlobalPosition();
    const Math::Vec3 dropLocal = drop->getPosition();
    const Math::Vec3 dropPoint = drop->toGlobal(probe);

    select(rig, keep);
    CHECK(rig.actions.reparentSelection(parent, true, error));
    CHECK(keep->getParent() == parent);
    CHECK(nearVec(keep->getGlobalPosition(), keepOrigin));
    CHECK(nearVec(keep->toGlobal(probe), keepPoint));
    CHECK(!nearVec(keep->getPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));

    select(rig, drop);
    CHECK(rig.actions.reparentSelection(parent, false, error));
    CHECK(drop->getParent() == parent);
    CHECK(nearVec(drop->getPosition(), dropLocal));
    CHECK(!nearVec(drop->getGlobalPosition(), Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(!nearVec(drop->toGlobal(probe), dropPoint));

    const Math::Vec3 plainOrigin = inPlain->getGlobalPosition();
    select(rig, inPlain);
    CHECK(rig.actions.reparentSelection(parent, true, error));
    CHECK(nearVec(inPlain->getGlobalPosition(), plainOrigin));

    select(rig, keep);
    CHECK(rig.actions.reparentSelection(plain, true, error));
    CHECK(keep->getParent() == plain);
    CHECK(nearVec(keep->getGlobalPosition(), keepOrigin));
    CHECK(nearVec(keep->getPosition(), keepOrigin));
}

void testReparentNames(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* box1 = add(scene, "Box");
    Node3D* host = add(scene, "Host");
    add(host, "Box");
    Node3D* spare = add(scene, "Spare");
    Node3D* inner = add(spare, "Box");
    commit(rig);
    ct::String error;

    select(rig, box1);
    CHECK(rig.actions.reparentSelection(host, true, error));
    CHECK(box1->getName() == "Box2");
    CHECK(names(host) == "Box,Box2");
    CHECK(selectedIs(rig, box1));

    select(rig, inner);
    CHECK(rig.actions.reparentSelection(host, true, error));
    CHECK(inner->getName() == "Box3");
    CHECK(names(host) == "Box,Box2,Box3");

    select(rig, box1, inner);
    CHECK(rig.actions.reparentSelection(spare, true, error));
    CHECK(names(spare) == "Box2,Box3");
    CHECK(selectedIs(rig, box1, inner));

    Node3D* another = add(scene, "Box");
    rig.frames(2);
    select(rig, another);
    CHECK(rig.actions.reparentSelection(spare, true, error));
    CHECK(another->getName() == "Box");
    CHECK(names(spare) == "Box2,Box3,Box");

    ct::Vector<ct::String> before;
    Node3D* lone = add(scene, "lone");
    rig.frames(2);
    select(rig, lone);
    Node3D* twin = add(host, "lone");
    CHECK(twin->getName() == "lone");
    CHECK(rig.actions.reparentSelection(host, true, error));
    CHECK(lone->getName() == "lone2");
    CHECK(twin->getName() == "lone");
}

void testReparentPaths(Rig& rig)
{
    Node* scene = freshScene(rig);
    Terrain3D* terrain = scene->createChild<Terrain3D>();
    terrain->setName("terrain");
    Node3D* anchor = add(scene, "Anchor");
    Node3D* host = add(scene, "Host");
    Node3D* branch = add(scene, "Branch");
    Forest3D* forest = branch->createChild<Forest3D>();
    forest->setName("forest");
    forest->setTerrain(terrain);
    Node3D* arm = add(branch, "Arm");
    PinJoint3D* pin = arm->createChild<PinJoint3D>();
    pin->setName("pin");
    pin->setNodeA("../../../Anchor");
    CHECK(forest->getTerrainPath() == "../../terrain");
    CHECK(pin->getNode(pin->getNodeA()) == anchor);
    commit(rig);
    ct::String error;

    select(rig, branch);
    CHECK(rig.actions.reparentSelection(host, true, error));
    CHECK(forest->getTerrainPath() == "../../../terrain");
    CHECK(forest->getNode(forest->getTerrainPath()) == terrain);
    CHECK(forest->getTerrain() == terrain);
    CHECK(pin->getNodeA() == "../../../../Anchor");
    CHECK(pin->getNode(pin->getNodeA()) == anchor);

    select(rig, terrain);
    CHECK(rig.actions.reparentSelection(host, true, error));
    CHECK(forest->getTerrainPath() == "../../terrain");
    CHECK(forest->getNode(forest->getTerrainPath()) == terrain);

    select(rig, anchor);
    CHECK(rig.actions.reparentSelection(arm, true, error));
    CHECK(pin->getNodeA() == "../Anchor");
    CHECK(pin->getNode(pin->getNodeA()) == anchor);

    select(rig, pin);
    CHECK(rig.actions.reparentSelection(host, true, error));
    CHECK(pin->getNode(pin->getNodeA()) == anchor);
    CHECK(pin->getNodeA() == "../Branch/Arm/Anchor");

    select(rig, terrain);
    CHECK(rig.actions.reparentToNewNode("Node3D", error));
    CHECK(terrain->getParent() != host);
    CHECK(forest->getNode(forest->getTerrainPath()) == terrain);
    CHECK(pin->getNode(pin->getNodeA()) == anchor);

    select(rig, branch);
    CHECK(rig.actions.reparentSelection(scene, true, error));
    CHECK(forest->getNode(forest->getTerrainPath()) == terrain);
    CHECK(pin->getNode(pin->getNodeA()) == anchor);
    CHECK(forest->getTerrain() == terrain);
}

void testReparentLibrary(Rig&)
{
    Node3D root;
    Node3D* a = root.createChild<Node3D>();
    a->setName("A");
    Node3D* b = root.createChild<Node3D>();
    b->setName("B");
    Node3D* first = a->createChild<Node3D>();
    first->setName("same");
    Node3D* second = b->createChild<Node3D>();
    second->setName("same");

    CHECK(second->reparent(a));
    CHECK(second->getName() != "same");
    CHECK(second->getName().find("same") == ct::String::npos || second->getName()[0] == '@');
    CHECK(a->findChild(second->getName()) == second);

    Node3D* third = b->createChild<Node3D>();
    third->setName("same");
    CHECK(third->reparent(a, true));
    CHECK(third->getName() == "same2");
    CHECK(a->findChild("same2") == third);
    CHECK(!third->reparent(a, true));
    CHECK(!third->reparent(third, true));
    CHECK(!a->reparent(third, true));
    CHECK(!third->reparent(nullptr, true));
    CHECK(a->getChildCount() == 3);
}

void testReparentForeign(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* free = add(scene, "free");
    Node* crate = makeCrate(rig, scene, "crate_c.prefab.json", "crateC");
    if (!crate) return;
    Node* lid = crate->findChild("lid");
    Node* base = crate->findChild("base");
    CHECK(lid && base);
    if (!lid || !base) return;
    commit(rig);
    ct::String error;

    select(rig, lid);
    CHECK(!rig.actions.reparentSelection(free, true, error));
    CHECK(error == HierarchyActions::kForeignRefusal);
    CHECK(lid->getParent() == crate);
    select(rig, free, lid);
    CHECK(!rig.actions.reparentSelection(free, true, error));
    CHECK(error == HierarchyActions::kForeignRefusal);
    CHECK(!rig.actions.reparentToNewNode("Node3D", error));
    CHECK(error == HierarchyActions::kForeignRefusal);
    CHECK(names(scene) == "free,crateC");
    CHECK(!rig.document.dirty());

    select(rig, scene);
    CHECK(!rig.actions.reparentSelection(free, true, error));
    CHECK(error == HierarchyActions::kRootRefusal);
    CHECK(!rig.actions.reparentToNewNode("Node3D", error));
    CHECK(error == HierarchyActions::kRootRefusal);
    select(rig, scene, free);
    CHECK(!rig.actions.reparentSelection(base, true, error));
    CHECK(error == HierarchyActions::kRootRefusal);
    CHECK(!rig.document.dirty());

    select(rig, free);
    CHECK(rig.actions.reparentSelection(base, true, error));
    CHECK(error.empty());
    CHECK(free->getParent() == base);
    CHECK(scene->isEditableInstance(crate));
    CHECK(selectedIs(rig, free));

    SceneResult saved;
    Node* holder = nullptr;
    Node* copy = reloaded(rig, scene, saved, holder);
    CHECK(saved.warningCount() == 0);
    CHECK(copy && copy->getNode("crateC/base/free") != nullptr);
    dropHolder(rig, holder);
}

void testReparentDialog(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node3D* c = add(scene, "C");
    Node3D* a1 = add(a, "a1");
    Node3D* hidden = a->createChild<Node3D>();
    hidden->setName("Hidden");
    hidden->setInternal(true);
    b->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
    a->setPosition(Math::Vec3(1.0f, 1.0f, 1.0f));
    c->setPosition(Math::Vec3(0.0f, 0.0f, 2.0f));
    commit(rig);

    select(rig, nullptr);
    rig.actions.requestReparent();
    rig.frames(2);
    CHECK(rig.actions.reparentDialog() == nullptr);
    select(rig, scene);
    rig.actions.requestReparent();
    rig.frames(2);
    CHECK(rig.actions.reparentDialog() == nullptr);
    CHECK(popupDialog() != nullptr);
    if (popupDialog())
    {
        CHECK(popupDialog()->message_ == kRootText);
        closePopup(rig);
    }

    select(rig, a);
    rig.actions.requestReparent();
    rig.frames(3);
    ReparentDialog* box = rig.actions.reparentDialog();
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->title() == "Reparent Node");
    CHECK(strcmp(ReparentDialog::kPrompt, "Select new parent:") == 0);
    CHECK(box->keepTransformBox()->isChecked());
    CHECK(box->keepTransformBox()->text() == "Keep Global Transform");
    CHECK(box->reparentButton()->text() == "Reparent");
    CHECK(box->tree()->roots().size() == 1);
    CHECK(box->tree()->roots()[0]->text() == scene->getName());
    CHECK(box->itemOf(scene) == box->tree()->roots()[0]);
    CHECK(box->itemOf(a1) != nullptr);
    CHECK(box->itemOf(hidden) == nullptr);
    CHECK(box->picked() == nullptr);

    CHECK(!box->canPick(a));
    CHECK(!box->canPick(a1));
    CHECK(box->canPick(b));
    CHECK(box->canPick(c));
    CHECK(box->canPick(scene));
    CHECK(!box->canPick(nullptr));

    CHECK(!box->pick(a));
    CHECK(box->picked() == nullptr);
    CHECK(box->refusedCount() == 1);
    CHECK(!box->pick(a1));
    CHECK(box->picked() == nullptr);
    CHECK(box->refusedCount() == 2);
    clickRow(rig, box->tree(), box->itemOf(a));
    CHECK(box->picked() == nullptr);
    CHECK(box->tree()->selectedNode() == nullptr);
    CHECK(box->refusedCount() == 3);

    clickCentre(rig, box->reparentButton()->absoluteRect());
    CHECK(rig.actions.reparentDialog() == box);
    CHECK(a->getParent() == scene);

    clickRow(rig, box->tree(), box->itemOf(b));
    CHECK(box->picked() == b);
    CHECK(box->pick(c));
    CHECK(box->picked() == c);
    CHECK(!box->pick(a));
    CHECK(box->picked() == c);
    CHECK(box->tree()->selectedNode() == box->itemOf(c));
    CHECK(box->pick(b));

    const Math::Vec3 before = a->getGlobalPosition();
    clickCentre(rig, box->reparentButton()->absoluteRect());
    CHECK(rig.actions.reparentDialog() == nullptr);
    CHECK(a->getParent() == b);
    CHECK(nearVec(a->getGlobalPosition(), before));
    CHECK(nearVec(a->getPosition(), Math::Vec3(-4.0f, 1.0f, 1.0f)));
    CHECK(selectedIs(rig, a));
    CHECK(rig.document.dirty());
    commit(rig);

    select(rig, a);
    rig.actions.requestReparent();
    rig.frames(3);
    box = rig.actions.reparentDialog();
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->pick(scene));
    box->keepTransformBox()->setChecked(false);
    CHECK(!box->keepTransformBox()->isChecked());
    box->accept();
    rig.frames(3);
    CHECK(rig.actions.reparentDialog() == nullptr);
    CHECK(a->getParent() == scene);
    CHECK(nearVec(a->getPosition(), Math::Vec3(-4.0f, 1.0f, 1.0f)));
    commit(rig);

    select(rig, a, c);
    rig.actions.requestReparent();
    rig.frames(3);
    box = rig.actions.reparentDialog();
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(!box->canPick(a) && !box->canPick(c) && !box->canPick(a1));
    CHECK(box->canPick(b));
    rig.tap(Key::Escape);
    CHECK(rig.actions.reparentDialog() == nullptr);
    CHECK(a->getParent() == scene);

    rig.actions.requestReparent();
    rig.frames(3);
    box = rig.actions.reparentDialog();
    CHECK(box != nullptr);
    if (!box) return;
    box->tree()->activated.emit(box->itemOf(b));
    CHECK(rig.actions.reparentDialog() == box);
    CHECK(box->pick(b));
    rig.tap(Key::Enter);
    CHECK(rig.actions.reparentDialog() == nullptr);
    CHECK(a->getParent() == b && c->getParent() == b);
    CHECK(selectedIs(rig, a, c));
    commit(rig);

    select(rig, c);
    rig.actions.requestReparent();
    rig.frames(3);
    box = rig.actions.reparentDialog();
    CHECK(box != nullptr);
    if (!box) return;
    rig.actions.requestReparent();
    rig.frames(3);
    CHECK(rig.actions.reparentDialog() != nullptr);
    a1->queueFree();
    rig.frames(4);
    CHECK(rig.actions.reparentDialog() == nullptr);
    CHECK(c->getParent() == b);
}

void testReparentToNew(Rig& rig)
{
    ct::String error;

    {
        Node* scene = freshScene(rig);
        Node3D* a = add(scene, "A");
        Node3D* b = add(scene, "B");
        Node3D* c = add(scene, "C");
        b->setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));
        commit(rig);

        select(rig, b);
        CHECK(rig.actions.reparentToNewNode("Node3D", error));
        CHECK(error.empty());
        CHECK(names(scene) == "A,Node3D,C");
        Node* fresh = scene->findChild("Node3D");
        CHECK(fresh != nullptr);
        CHECK(fresh && b->getParent() == fresh);
        CHECK(fresh && fresh->getIndex() == 1);
        CHECK(selectedIs(rig, b));
        CHECK(nearVec(b->getGlobalPosition(), Math::Vec3(5.0f, 0.0f, 0.0f)));
        CHECK(rig.document.dirty());
        commit(rig);

        select(rig, a, c);
        CHECK(rig.actions.reparentToNewNode("Node3D", error));
        CHECK(names(scene) == "Node3D2,Node3D");
        Node* second = scene->findChild("Node3D2");
        CHECK(second != nullptr);
        CHECK(second && names(second) == "A,C");
        CHECK(selectedIs(rig, a, c));
        commit(rig);

        select(rig, nullptr);
        CHECK(!rig.actions.reparentToNewNode("Node3D", error));
        CHECK(error.empty());
        CHECK(names(scene) == "Node3D2,Node3D");
    }

    {
        Node* scene = freshScene(rig);
        Node3D* x = add(scene, "X");
        Node3D* y = add(scene, "Y");
        Node3D* z = add(scene, "Z");
        Node3D* p = add(x, "p");
        Node3D* q = add(y, "q");
        add(y, "r");
        commit(rig);

        select(rig, p, q);
        CHECK(rig.actions.reparentToNewNode("Node3D", error));
        CHECK(names(scene) == "X,Y,Z,Node3D");
        Node* fresh = scene->findChild("Node3D");
        CHECK(fresh != nullptr);
        CHECK(fresh && names(fresh) == "p,q");
        CHECK(p->getParent() == fresh && q->getParent() == fresh);
        CHECK(names(x).empty() && names(y) == "r");
        CHECK(selectedIs(rig, p, q));
        (void) z;
        commit(rig);
    }

    {
        Node* scene = freshScene(rig);
        Node3D* a = add(scene, "A");
        Node3D* b = add(scene, "B");
        Node3D* a1 = add(a, "a1");
        Node3D* a2 = add(a, "a2");
        add(scene, "C");
        commit(rig);

        select(rig, a1, b);
        CHECK(rig.actions.reparentToNewNode("OmniLight3D", error));
        CHECK(names(scene) == "A,OmniLight3D,C");
        Node* fresh = scene->findChild("OmniLight3D");
        CHECK(fresh != nullptr);
        CHECK(fresh && strcmp(fresh->getClassName(), "OmniLight3D") == 0);
        CHECK(fresh && names(fresh) == "a1,B");
        CHECK(names(a) == "a2");
        CHECK(selectedIs(rig, a1, b));
        commit(rig);

        select(rig, a, a2);
        CHECK(rig.actions.reparentToNewNode("Node3D", error));
        Node* wrapper = scene->findChild("Node3D");
        CHECK(wrapper != nullptr);
        CHECK(wrapper && names(wrapper) == "A");
        CHECK(names(a) == "a2");
        CHECK(names(scene) == "Node3D,OmniLight3D,C");
        CHECK(selectedIs(rig, a, a2));
        commit(rig);

        CHECK(!rig.actions.reparentToNewNode("Window", error));
        CHECK(!error.empty());
        CHECK(names(scene) == "Node3D,OmniLight3D,C");
        CHECK(!rig.document.dirty());
    }

    {
        Node* scene = freshScene(rig);
        Node3D* a = add(scene, "A");
        Node3D* b = add(scene, "B");
        commit(rig);

        select(rig, b);
        rig.actions.requestReparentToNew();
        rig.frames(3);
        CreateNodeDialog* box = rig.actions.createDialog();
        CHECK(box != nullptr);
        if (box)
        {
            box->setSearchText("Camera3D");
            rig.tap(Key::Enter);
            CHECK(rig.actions.createDialog() == nullptr);
            CHECK(names(scene) == "A,Camera3D");
            Node* camera = scene->findChild("Camera3D");
            CHECK(camera && names(camera) == "B");
            CHECK(selectedIs(rig, b));
        }
        (void) a;
        resetCreateSettings(rig);

        select(rig, scene);
        rig.actions.requestReparentToNew();
        rig.frames(3);
        CHECK(rig.actions.createDialog() == nullptr);
        CHECK(popupDialog() != nullptr);
        if (popupDialog())
        {
            CHECK(popupDialog()->message_ == kRootText);
            closePopup(rig);
        }
    }
}

void testMenusAndButton(Rig& rig)
{
    Node* scene = freshScene(rig);
    Node3D* a = add(scene, "A");
    Node3D* b = add(scene, "B");
    Node* crate = makeCrate(rig, scene, "crate_d.prefab.json", "crateD");
    commit(rig);

    rt::Menu* edit = rig.menus.menu(EditorMenus::kEdit);
    rt::Menu* context = rig.actions.contextMenu();
    CHECK(edit != nullptr && context != nullptr);
    if (!edit || !context) return;

    CHECK(strcmp(HierarchyActions::kAddTooltip, "Add/Create a New Node. (Ctrl+A)") == 0);
    CHECK(strcmp(HierarchyActions::label(HierarchyActions::kAddChild), "Add Child Node...") == 0);
    CHECK(strcmp(HierarchyActions::label(HierarchyActions::kReparent), "Reparent...") == 0);
    CHECK(strcmp(HierarchyActions::label(HierarchyActions::kReparentToNew),
                  "Reparent to New Node...") == 0);
    CHECK(strcmp(HierarchyActions::shortcut(HierarchyActions::kAddChild), "Ctrl+A") == 0);
    CHECK(EditorMenus::kEditAddChild == EditorMenus::kEditHeader);
    CHECK(EditorMenus::kEditDelete == EditorMenus::kEditItemCount - 1);
    CHECK(EditorMenus::kEditSep1 == EditorMenus::kEditHeader + 1 &&
            EditorMenus::kEditSep2 == EditorMenus::kEditHeader + 3 &&
            EditorMenus::kEditSep3 == EditorMenus::kEditHeader + 5 &&
            EditorMenus::kEditSep4 == EditorMenus::kEditHeader + 12);

    const int order[] = { EditorMenus::kEditAddChild, EditorMenus::kEditDuplicate,
        EditorMenus::kEditRename, EditorMenus::kEditMoveUp, EditorMenus::kEditMoveDown,
        EditorMenus::kEditReparent, EditorMenus::kEditReparentToNew, EditorMenus::kEditDelete };
    for (std::size_t i = 1; i < sizeof order / sizeof order[0]; ++i) CHECK(order[i - 1] < order[i]);

    select(rig, nullptr);
    rig.frames(2);
    for (rt::Menu* menu: { edit, context })
    {
        const int offset =
                menu == context ? HierarchyActions::kContextOffset : EditorMenus::kEditHeader;
        CHECK(menu->action(offset + HierarchyActions::kItemAddChild)->isEnabled());
        CHECK(!menu->action(offset + HierarchyActions::kItemReparent)->isEnabled());
        CHECK(!menu->action(offset + HierarchyActions::kItemReparentToNew)->isEnabled());
    }
    select(rig, scene);
    for (rt::Menu* menu: { edit, context })
    {
        const int offset =
                menu == context ? HierarchyActions::kContextOffset : EditorMenus::kEditHeader;
        CHECK(menu->action(offset + HierarchyActions::kItemAddChild)->isEnabled());
        CHECK(!menu->action(offset + HierarchyActions::kItemReparent)->isEnabled());
        CHECK(!menu->action(offset + HierarchyActions::kItemReparentToNew)->isEnabled());
    }
    select(rig, a, b);
    for (rt::Menu* menu: { edit, context })
    {
        const int offset =
                menu == context ? HierarchyActions::kContextOffset : EditorMenus::kEditHeader;
        CHECK(menu->action(offset + HierarchyActions::kItemReparent)->isEnabled());
        CHECK(menu->action(offset + HierarchyActions::kItemReparentToNew)->isEnabled());
    }
    select(rig, scene, a);
    CHECK(!edit->action(EditorMenus::kEditReparent)->isEnabled());
    CHECK(!rig.actions.canReparentToNew());

    select(rig, crate ? crate->findChild("lid") : nullptr);
    CHECK(edit->action(EditorMenus::kEditReparent)->isEnabled());
    rig.actions.requestReparent();
    rig.frames(2);
    CHECK(rig.actions.reparentDialog() == nullptr);
    CHECK(popupDialog() != nullptr);
    if (popupDialog())
    {
        CHECK(popupDialog()->message_ == HierarchyActions::kForeignRefusal);
        closePopup(rig);
    }
    rig.actions.requestReparentToNew();
    rig.frames(2);
    CHECK(rig.actions.createDialog() == nullptr);
    CHECK(popupDialog() != nullptr);
    closePopup(rig);

    select(rig, b);
    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditAddChild);
    CHECK(rig.actions.createDialog() != nullptr);
    CHECK(rt::WidgetApp::instance().popup() == nullptr);
    rig.tap(Key::Escape);
    CHECK(rig.actions.createDialog() == nullptr);

    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditReparent);
    CHECK(rig.actions.reparentDialog() != nullptr);
    CHECK(rig.actions.createDialog() == nullptr);
    if (rig.actions.reparentDialog()) rig.actions.reparentDialog()->cancel();
    rig.frames(2);
    CHECK(rig.actions.reparentDialog() == nullptr);

    CHECK(openBarMenu(rig, edit));
    clickMenuItem(rig, edit, EditorMenus::kEditReparentToNew);
    CreateNodeDialog* box = rig.actions.createDialog();
    CHECK(box != nullptr);
    if (box)
    {
        box->setSearchText("Node3D");
        box->confirm();
        rig.frames(3);
        Node* wrapper = scene->findChild("Node3D");
        CHECK(wrapper != nullptr);
        CHECK(wrapper && b->getParent() == wrapper);
        CHECK(selectedIs(rig, b));
    }
    commit(rig);

    select(rig, a);
    rig.click(textX(rig, a), rowY(rig, a), MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == context);
    clickMenuItem(rig, context, HierarchyActions::kItemAddChild + HierarchyActions::kContextOffset);
    CHECK(rig.actions.createDialog() != nullptr);
    closePopup(rig);
    if (rig.actions.createDialog()) rig.actions.createDialog()->cancel();
    rig.frames(2);

    rig.click(textX(rig, a), rowY(rig, a), MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == context);
    clickMenuItem(rig, context, HierarchyActions::kItemReparent + HierarchyActions::kContextOffset);
    CHECK(rig.actions.reparentDialog() != nullptr);
    if (rig.actions.reparentDialog()) rig.actions.reparentDialog()->cancel();
    rig.frames(2);

    rt::ToolBar* bar = rig.hierarchy.toolBar();
    CHECK(bar != nullptr);
    if (!bar) return;
    const int id = rig.actions.addButtonId();
    CHECK(id >= 0);
    CHECK(bar->isItemEnabled(id));
    CHECK(bar->isItemVisible(id));
    const rt::Rect barRect = bar->absoluteRect();
    CHECK(barRect.w > 0.0f && barRect.h > 0.0f);
    CHECK(barRect.y < rig.hierarchy.outliner()->absoluteRect().y);
    CHECK(rig.hierarchy.outliner()->absoluteRect().h > 100.0f);
    select(rig, nullptr);
    CHECK(createDialogCount() == 0);
    rig.click(static_cast<int>(barRect.x + 2.0f + 14.0f),
            static_cast<int>(barRect.y + barRect.h * 0.5f));
    rt::Menu* dropdown = rig.actions.createMenu().dropdown();
    CHECK(dropdown != nullptr);
    CHECK(rt::WidgetApp::instance().popup() == dropdown);
    CHECK(createDialogCount() == 0);
    if (dropdown)
    {
        CHECK(dropdown->absoluteRect().y >= barRect.y + barRect.h - 1.0f);
        clickMenuItem(rig, dropdown, CreateMenu::kEntryAddNode);
        CHECK(rig.actions.createDialog() != nullptr);
        if (rig.actions.createDialog()) rig.actions.createDialog()->cancel();
        rig.frames(2);
    }
    bar->buttonClicked.emit(id);
    rig.frames(2);
    CHECK(rt::WidgetApp::instance().popup() == dropdown);
    CHECK(createDialogCount() == 0);
    closePopup(rig);
}

void testAddShortcut(Rig& rig)
{
    Node* scene = freshScene(rig);
    add(scene, "A");
    commit(rig);
    rt::WidgetApp& app = rt::WidgetApp::instance();
    rt::TextInput* input = rig.shell.panel(3)->createChild<rt::TextInput>();
    rt::SpinBox* spin = rig.shell.panel(3)->createChild<rt::SpinBox>();
    NumberField* number = rig.shell.panel(3)->createChild<NumberField>("X");
    rt::Button* pressable = rig.shell.panel(3)->createChild<rt::Button>("Press");
    rig.frames(3);

    select(rig, nullptr);
    app.setFocused(nullptr);
    CHECK(rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 1);
    CreateNodeDialog* box = rig.actions.createDialog();
    CHECK(box != nullptr);
    if (box) CHECK(app.focused() == box->searchInput());

    CHECK(!rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 1);
    CHECK(rig.actions.createDialog() == box);

    app.setFocused(nullptr);
    CHECK(rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 1);
    CHECK(rig.actions.createDialog() == box);
    if (box) box->cancel();
    rig.frames(3);
    CHECK(createDialogCount() == 0);

    app.setFocused(nullptr);
    CHECK(!rig.tap(Key::A, kModControl, true));
    CHECK(createDialogCount() == 0);
    CHECK(!rig.tap(Key::A));
    CHECK(!rig.tap(Key::A, kModControl | kModShift));
    CHECK(!rig.tap(Key::A, kModControl | kModAlt));
    CHECK(createDialogCount() == 0);

    app.setFocused(input);
    CHECK(app.focused() == input);
    CHECK(!rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 0);
    app.setFocused(spin);
    CHECK(!rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 0);
    number->beginEdit();
    CHECK(number->isEditing());
    CHECK(!rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 0);
    number->cancelEdit();
    rig.frames(2);

    app.setFocused(pressable);
    CHECK(rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 1);
    closeCreateDialogs(rig);

    app.setFocused(rig.hierarchy.outliner());
    select(rig, scene->findChild("A"));
    CHECK(rig.tap(Key::A, kModControl));
    CHECK(createDialogCount() == 1);
    closeCreateDialogs(rig);
    app.setFocused(nullptr);
    rig.frames(2);
}

void testEveryType(Rig& rig)
{
    Node* scene = freshScene(rig);
    ct::Vector<ct::String> types;
    CreateNodeDialog::typeList(types);
    ct::String error;
    for (std::size_t i = 0; i < types.size(); ++i)
    {
        select(rig, nullptr);
        const bool made = rig.actions.createNode(types[i], error);
        CHECK(made);
        if (!made) printf("could not create %s: %s\n", types[i].c_str(), error.c_str());
    }
    rig.frames(6);
    CHECK(scene->getChildCount() == types.size());
    for (std::size_t i = 0; i < types.size(); ++i)
    {
        Node* node = scene->getChild(i);
        CHECK(node != nullptr);
        if (!node) continue;
        CHECK(types[i] == node->getClassName() || node->getName().find(types[i]) == 0);
    }
    rig.document.markDirty();
    select(rig, nullptr);
    for (std::size_t i = scene->getChildCount(); i > 0; --i) scene->getChild(i - 1)->queueFree();
    rig.frames(6);
    CHECK(scene->getChildCount() == 0);
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor create test";
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
        document.setSettingsPath(settingsFile());
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

        CHECK(document.createProject(testRoot() + "/projects", "Create"));
        rig.frames(3);
        CHECK(!document.dirty());

        testTypeList(rig);
        testScore(rig);
        testTree(rig);
        testKeys(rig);
        testEditorSettingsFile(rig);
        testRecentFavorites(rig);
        testCreateAction(rig);
        testCreateDialogFlow(rig);
        testCreateUnderPrefab(rig);
        testReparent(rig);
        testReparentTransform(rig);
        testReparentNames(rig);
        testReparentPaths(rig);
        testReparentLibrary(rig);
        testReparentForeign(rig);
        testReparentDialog(rig);
        testReparentToNew(rig);
        testMenusAndButton(rig);
        testAddShortcut(rig);
        testEveryType(rig);

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
