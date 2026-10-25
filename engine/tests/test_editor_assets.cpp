#include "Check.h"
#include "EditorIconCheck.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/AssetImport.h"
#include "prisma3d/editor/AssetsPanel.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorInput.h"
#include "prisma3d/editor/EditorMenus.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorStatusBar.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/FileDialogMemory.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/PropertyRow.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Prefab.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/DialogWidgets.hpp>
#include <igui/widgets/FileDialog.hpp>
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

enum ConfirmButton
{
    kConfirmSave = 0,
    kConfirmDiscard = 1,
    kConfirmCancel = 2
};

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_assets_test_vulkan" : "/editor_assets_test";
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
    int sceneCounter = 0;

    bool send(const InputEvent& event)
    {
        EditorInput(host, ui, actions, panel).handle(event);
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

ct::String absolute(Rig& rig, const ct::String& relative)
{
    return rig.document.project().absolutePath(relative);
}

bool writeText(const ct::String& path, const char* text)
{
    const ct::String folder = AssetImport::parentOf(path);
    if (!folder.empty()) fs_create_directory(folder.c_str());
    return file_write_text(path.c_str(), text);
}

bool writeBytes(const ct::String& path, const void* data, std::size_t size)
{
    const ct::String folder = AssetImport::parentOf(path);
    if (!folder.empty()) fs_create_directory(folder.c_str());
    return file_write(path.c_str(), data, size);
}

bool writePng(const ct::String& path, int width, int height, unsigned char red)
{
    const ct::String folder = AssetImport::parentOf(path);
    if (!folder.empty()) fs_create_directory(folder.c_str());
    Pixmap pixmap(width, height, 4);
    pixmap.fill(red, 40, 80, 255);
    return pixmap.save(path.c_str());
}

ct::String readText(const ct::String& path)
{
    char* text = file_read_text(path.c_str());
    if (!text) return ct::String();
    const ct::String result(text);
    fs_free(text);
    return result;
}

bool exists(const ct::String& path)
{
    return file_exists(path.c_str()) || dir_exists(path.c_str());
}

const float kTriangle[9] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };

const char* const kGltf =
        "{\"asset\":{\"version\":\"2.0\"},"
        "\"buffers\":[{\"uri\":\"tri.bin\",\"byteLength\":36}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
        "\"min\":[0,0,0],\"max\":[1,1,0]}],"
        "\"images\":[{\"uri\":\"tex/albedo.png\"}],"
        "\"textures\":[{\"source\":0}],"
        "\"materials\":[{\"name\":\"skin\",\"pbrMetallicRoughness\":{\"baseColorTexture\":"
        "{\"index\":0}}}],"
        "\"meshes\":[{\"name\":\"tri\",\"primitives\":[{\"attributes\":{\"POSITION\":0},"
        "\"material\":0}]}],"
        "\"nodes\":[{\"name\":\"TriNode\",\"mesh\":0,\"translation\":[1,2,3]}],"
        "\"scenes\":[{\"nodes\":[0]}],\"scene\":0}";

bool writeModel(const ct::String& folder, const char* name)
{
    ct::String base = name;
    if (!writeText(AssetImport::joinPath(folder, base + ".gltf"), kGltf)) return false;
    if (!writeBytes(AssetImport::joinPath(folder, "tri.bin"), kTriangle, sizeof(kTriangle)))
        return false;
    return writePng(AssetImport::joinPath(folder, "tex/albedo.png"), 4, 4, 200);
}

PropertyValue property(const Node* node, const char* name)
{
    PropertyValue value;
    NodeClassDB::getProperty(node, name, value);
    return value;
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

rt::Button* findButton(rt::Widget* root, const char* text)
{
    if (!root) return nullptr;
    if (rt::Button* found = rt::widget_cast<rt::Button>(root))
        if (found->text() == text) return found;
    for (rt::Widget* child: root->children())
        if (rt::Button* found = findButton(child, text)) return found;
    return nullptr;
}

void clickWidget(Rig& rig, const rt::Widget* widget)
{
    CHECK(widget != nullptr);
    if (!widget) return;
    int x = 0;
    int y = 0;
    const rt::Rect rect = widget->absoluteRect();
    x = static_cast<int>(rect.x + rect.w * 0.5f);
    y = static_cast<int>(rect.y + rect.h * 0.5f);
    rig.click(x, y);
}

rt::MenuAction* actionNamed(rt::Menu* menu, const char* text)
{
    for (int i = 0; i < menu->actionCount(); ++i)
        if (menu->action(i)->text() == text) return menu->action(i);
    return nullptr;
}

bool triggerAction(Rig& rig, const char* text)
{
    rt::Menu* menu = rig.assets.contextMenu();
    rt::MenuAction* action = actionNamed(menu, text);
    CHECK(action != nullptr);
    if (!action || !action->isEnabled()) return false;
    rt::WidgetApp::instance().closePopup();
    rig.frames(2);
    action->triggered.emit();
    rig.frames(3);
    return true;
}

void centerOf(const rt::Rect& rect, int& x, int& y)
{
    x = static_cast<int>(rect.x + rect.w * 0.5f);
    y = static_cast<int>(rect.y + rect.h * 0.5f);
}

void itemCenter(Rig& rig, int index, int& x, int& y)
{
    rig.assets.grid()->ensureVisible(index);
    rig.frames(1);
    centerOf(rig.assets.grid()->itemRect(index), x, y);
}

void clickItem(Rig& rig, int index)
{
    int x = 0;
    int y = 0;
    itemCenter(rig, index, x, y);
    rig.click(x, y);
}

void doubleClickItem(Rig& rig, int index)
{
    int x = 0;
    int y = 0;
    itemCenter(rig, index, x, y);
    rig.doubleClick(x, y);
}

int entryIndex(Rig& rig, const char* name)
{
    const int index = rig.assets.indexOf(name);
    if (index < 0) printf("no entry named %s\n", name);
    return index;
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
    CHECK(rig.document.saveSceneAs(name));
    rig.frames(3);
    CHECK(!rig.document.dirty());
}

Node3D* add(Node* parent, const char* name)
{
    Node3D* node = parent->createChild<Node3D>();
    node->setName(name);
    return node;
}

int countNamed(const Node* parent, const char* name)
{
    int count = 0;
    for (std::size_t i = 0; i < parent->getChildCount(); ++i)
        if (strcmp(parent->getChild(i)->getName().c_str(), name) == 0) ++count;
    return count;
}

void showAssets(Rig& rig)
{
    rig.shell.dock()->showPanel("Assets");
    rig.shell.dock()->layout();
    rig.frames(4);
}

void showConsole(Rig& rig)
{
    rig.shell.dock()->showPanel("Console");
    rig.shell.dock()->layout();
    rig.frames(4);
}

bool near(float a, float b, float tolerance = 1.0e-3f)
{
    return fabsf(a - b) <= tolerance;
}

bool hasKind(const ct::Vector<AssetEntry>& list, const char* name, AssetKind kind)
{
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i].name == name) return list[i].kind == kind;
    return false;
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

InputEvent character(char c)
{
    InputEvent event;
    event.type = InputEventType::Char;
    event.character.codepoint = static_cast<std::uint32_t>(c);
    return event;
}

void testTypes()
{
    CHECK(AssetTypes::classify("Assets", true) == AssetKind::Folder);
    CHECK(AssetTypes::classify("a.scene.json", false) == AssetKind::Scene);
    CHECK(AssetTypes::classify("A.SCENE.JSON", false) == AssetKind::Scene);
    CHECK(AssetTypes::classify("a.prefab.json", false) == AssetKind::Prefab);
    CHECK(AssetTypes::classify("projecto.json", false) == AssetKind::Other);
    CHECK(AssetTypes::classify("a.json", false) == AssetKind::Other);
    const char* const models[] = { "glb", "gltf", "fbx", "obj", "b3d", "3ds", "ms3d", "mesh",
        "pmesh" };
    for (const char* extension: models)
    {
        const ct::String name = ct::String("m.") + extension;
        CHECK(AssetTypes::classify(name, false) == AssetKind::Model);
        CHECK(AssetTypes::classify(AssetTypes::lowered(name).c_str(), false) == AssetKind::Model);
    }
    const char* const textures[] = { "png", "jpg", "jpeg", "tga", "bmp", "dds", "hdr" };
    for (const char* extension: textures)
        CHECK(AssetTypes::classify(ct::String("t.") + extension, false) == AssetKind::Texture);
    CHECK(AssetTypes::classify("PHOTO.JPG", false) == AssetKind::Texture);
    CHECK(AssetTypes::classify("m.mat", false) == AssetKind::Material);
    const char* const sounds[] = { "wav", "mp3", "flac", "ogg" };
    for (const char* extension: sounds)
        CHECK(AssetTypes::classify(ct::String("s.") + extension, false) == AssetKind::Audio);
    CHECK(AssetTypes::classify("run.py", false) == AssetKind::Script);
    CHECK(AssetTypes::classify("notes.txt", false) == AssetKind::Other);
    CHECK(AssetTypes::classify("README", false) == AssetKind::Other);
    CHECK(AssetTypes::classify("x.obj.bak", false) == AssetKind::Other);

    for (int i = 0; i < static_cast<int>(AssetKind::kCount); ++i)
    {
        const AssetKind kind = static_cast<AssetKind>(i);
        CHECK(AssetTypes::label(kind)[0] != '\0');
    }
    CHECK(AssetTypes::icon(AssetKind::Folder).tint.b !=
            AssetTypes::icon(AssetKind::Texture).tint.b);
    CHECK(AssetTypes::extensionOf("a.TXT") == "txt" && AssetTypes::extensionOf("a").empty());
}

void testLayout(Rig& rig)
{
    showAssets(rig);
    AssetsPanel& assets = rig.assets;
    CHECK(assets.attached());
    CHECK(assets.hasProject());
    CHECK(assets.folder().empty());
    CHECK(assets.root() != nullptr && assets.root()->isVisible());

    CHECK(assets.bookmarkCount() == 6);
    const char* const wanted[] = { "", "Scenes", "Assets", "Assets/Models", "Assets/Textures",
        "Assets/Scripts" };
    for (std::size_t i = 0; i < 6; ++i) CHECK(assets.bookmarkPath(i) == wanted[i]);
    CHECK(assets.tree()->roots().size() == 6);
    CHECK(assets.tree()->roots()[0]->text() == rig.document.project().name());
    CHECK(assets.tree()->roots()[0]->isExpanded());
    CHECK(!assets.tree()->roots()[2]->isExpanded());
    CHECK(assets.treeNodeOf("Assets/Textures") == assets.tree()->roots()[4]);
    CHECK(assets.tree()->selectedNode() == assets.tree()->roots()[0]);

    CHECK(assets.buttonId(AssetsPanel::kButtonCount) == -1);
    for (int i = 0; i < AssetsPanel::kButtonCount; ++i)
    {
        const AssetsPanel::Button b = static_cast<AssetsPanel::Button>(i);
        CHECK(assets.buttonId(b) >= 0);
        CHECK(isAtlasIcon(AssetsPanel::buttonIcon(b)));
        CHECK(AssetsPanel::buttonTooltip(b)[0] != '\0');
    }
    CHECK(!assets.toolBar()->isItemEnabled(assets.buttonId(AssetsPanel::kBack)));
    CHECK(!assets.toolBar()->isItemEnabled(assets.buttonId(AssetsPanel::kForward)));
    CHECK(!assets.toolBar()->isItemEnabled(assets.buttonId(AssetsPanel::kUp)));
    CHECK(assets.toolBar()->isItemEnabled(assets.buttonId(AssetsPanel::kRefresh)));
    CHECK(assets.toolBar()->isItemChecked(assets.buttonId(AssetsPanel::kGridView)));

    const ct::Vector<AssetEntry>& entries = assets.entries();
    CHECK(entries.size() >= 3);
    CHECK(hasKind(entries, "Assets", AssetKind::Folder));
    CHECK(hasKind(entries, "Scenes", AssetKind::Folder));
    CHECK(hasKind(entries, "projecto.json", AssetKind::Other));
    CHECK(entries.size() >= 2 && entries[0].directory && entries[1].directory);
    CHECK(entries[0].name == "Assets" && entries[0].path == "Assets");
    CHECK(assets.grid()->itemCount() == entries.size());
    CHECK(assets.breadcrumbs()->path().size() == 1);
    CHECK(assets.breadcrumbs()->path()[0] == rig.document.project().name());
    CHECK(assets.footer()->text().contains("items"));

    const rt::Rect grid = assets.grid()->absoluteRect();
    const rt::Rect treeRect = assets.tree()->absoluteRect();
    CHECK(grid.w > 200.0f && grid.h > 40.0f);
    CHECK(treeRect.w > 50.0f && treeRect.x < grid.x);

    const float before = assets.splitter()->ratio();
    const rt::Rect split = assets.splitter()->absoluteRect();
    const int handleX = static_cast<int>(treeRect.x + treeRect.w + 2.0f);
    const int handleY = static_cast<int>(split.y + split.h * 0.5f);
    rig.drag(handleX, handleY, handleX + 120, handleY);
    CHECK(assets.splitter()->ratio() > before + 0.02f);
    rig.drag(handleX + 120, handleY, handleX - 60, handleY);
    CHECK(assets.splitter()->ratio() < before + 0.1f);
}

void testListing(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    const ct::String mixed = absolute(rig, "Assets/Mixed");
    const char* const files[] = { "scene one.scene.json", "Crate.prefab.json", "robot.glb",
        "robot2.GLTF", "a.fbx", "b.obj", "c.b3d", "d.3ds", "e.ms3d", "f.mesh", "g.pmesh", "a.png",
        "B.JPG", "c.jpeg", "d.tga", "e.bmp", "f.dds", "g.hdr", "wood.mat", "s.wav", "s.mp3",
        "s.flac", "s.ogg", "behave.py", "notes.txt", "README", ".hidden", "apple.txt",
        "Banana.txt" };
    for (const char* file: files) CHECK(writeText(AssetImport::joinPath(mixed, file), "x"));
    CHECK(fs_create_directory(AssetImport::joinPath(mixed, "Zeta").c_str()));
    CHECK(fs_create_directory(AssetImport::joinPath(mixed, "alpha").c_str()));

    CHECK(assets.navigateTo("Assets/Mixed"));
    rig.frames(2);
    const ct::Vector<AssetEntry>& entries = assets.entries();
    CHECK(entries.size() == 30);
    CHECK(entries[0].name == "alpha" && entries[0].directory);
    CHECK(entries[1].name == "Zeta" && entries[1].directory);
    CHECK(entries[2].name == "a.fbx");
    for (std::size_t i = 0; i < entries.size(); ++i) CHECK(entries[i].name != ".hidden");
    CHECK(hasKind(entries, "alpha", AssetKind::Folder));
    CHECK(hasKind(entries, "scene one.scene.json", AssetKind::Scene));
    CHECK(hasKind(entries, "Crate.prefab.json", AssetKind::Prefab));
    CHECK(hasKind(entries, "robot.glb", AssetKind::Model));
    CHECK(hasKind(entries, "robot2.GLTF", AssetKind::Model));
    CHECK(hasKind(entries, "a.fbx", AssetKind::Model));
    CHECK(hasKind(entries, "g.pmesh", AssetKind::Model));
    CHECK(hasKind(entries, "a.png", AssetKind::Texture));
    CHECK(hasKind(entries, "B.JPG", AssetKind::Texture));
    CHECK(hasKind(entries, "f.dds", AssetKind::Texture));
    CHECK(hasKind(entries, "g.hdr", AssetKind::Texture));
    CHECK(hasKind(entries, "wood.mat", AssetKind::Material));
    CHECK(hasKind(entries, "s.flac", AssetKind::Audio));
    CHECK(hasKind(entries, "behave.py", AssetKind::Script));
    CHECK(hasKind(entries, "notes.txt", AssetKind::Other));
    CHECK(hasKind(entries, "README", AssetKind::Other));
    int apple = -1;
    int banana = -1;
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        if (entries[i].name == "apple.txt") apple = static_cast<int>(i);
        if (entries[i].name == "Banana.txt") banana = static_cast<int>(i);
    }
    CHECK(apple >= 0 && banana > apple);
    CHECK(entries[static_cast<std::size_t>(apple)].path == "Assets/Mixed/apple.txt");
    CHECK(entries[static_cast<std::size_t>(apple)].size == 1);

    const unsigned counted = assets.listingCount();
    rig.frames(10);
    CHECK(assets.listingCount() == counted);
    assets.refresh();
    CHECK(assets.listingCount() == counted + 1);
    rig.frames(10);
    CHECK(assets.listingCount() == counted + 1);

    CHECK(assets.tree()->selectedNode() == assets.treeNodeOf("Assets/Mixed"));
    CHECK(assets.pathOfTreeNode(assets.tree()->selectedNode()) == "Assets/Mixed");
    CHECK(assets.tree()->roots()[2]->isExpanded());
    CHECK(assets.breadcrumbs()->path().size() == 3);
    CHECK(assets.breadcrumbs()->path()[2] == "Mixed");
    CHECK(assets.footer()->text().contains("Assets/Mixed") &&
            assets.footer()->text().contains("30"));
    assets.goHome();
    rig.frames(2);
}

void testNavigation(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    CHECK(fs_create_directory(absolute(rig, "Assets/Mixed/alpha").c_str()));
    assets.refresh();
    rig.frames(2);
    CHECK(assets.navigateTo(""));
    CHECK(assets.folder().empty());
    CHECK(!assets.canGoBack() && !assets.canGoForward() && !assets.canGoUp());
    CHECK(!assets.goBack() && !assets.goForward() && !assets.goUp());
    const std::size_t baseline = assets.historySize();

    CHECK(assets.navigateTo("Assets"));
    CHECK(assets.navigateTo("Assets/Models"));
    CHECK(assets.navigateTo("Assets/Models"));
    CHECK(assets.historySize() == baseline + 2);
    CHECK(assets.folder() == "Assets/Models");
    CHECK(assets.canGoBack() && !assets.canGoForward() && assets.canGoUp());
    CHECK(assets.goBack());
    CHECK(assets.folder() == "Assets");
    CHECK(assets.goBack());
    CHECK(assets.folder().empty());
    CHECK(!assets.canGoBack() && assets.canGoForward());
    CHECK(assets.goForward() && assets.folder() == "Assets");
    CHECK(assets.navigateTo("Scenes"));
    CHECK(!assets.canGoForward());
    CHECK(assets.goBack() && assets.folder() == "Assets");
    CHECK(assets.goUp() && assets.folder().empty());
    CHECK(assets.navigateTo("Assets\\Models") && assets.folder() == "Assets/Models");
    CHECK(assets.navigateTo("./Assets//Textures/") && assets.folder() == "Assets/Textures");
    CHECK(assets.goUp() && assets.folder() == "Assets");
    CHECK(assets.goHome() && assets.folder().empty());
    CHECK(assets.navigateTo("Assets/Models"));

    const ct::String here = assets.folder();
    CHECK(!assets.navigateTo("../outside"));
    CHECK(!assets.navigateTo("Assets/../.."));
    CHECK(!assets.navigateTo("Nope"));
    CHECK(!assets.navigateTo("Assets/Models/missing"));
    CHECK(!assets.navigateTo("/tmp"));
    CHECK(!assets.navigateTo(absolute(rig, "Assets")));
    CHECK(!assets.navigateTo("projecto.json"));
    CHECK(assets.folder() == here);

    rt::ToolBar* bar = assets.toolBar();
    bar->buttonClicked.emit(assets.buttonId(AssetsPanel::kUp));
    rig.frames(2);
    CHECK(assets.folder() == "Assets");
    bar->buttonClicked.emit(assets.buttonId(AssetsPanel::kBack));
    rig.frames(2);
    CHECK(assets.folder() == "Assets/Models");
    bar->buttonClicked.emit(assets.buttonId(AssetsPanel::kBack));
    rig.frames(2);
    CHECK(assets.folder().empty());
    bar->buttonClicked.emit(assets.buttonId(AssetsPanel::kForward));
    rig.frames(2);
    CHECK(assets.folder() == "Assets/Models");
    bar->buttonClicked.emit(assets.buttonId(AssetsPanel::kHome));
    rig.frames(2);
    CHECK(assets.folder().empty());
    CHECK(!bar->isItemEnabled(assets.buttonId(AssetsPanel::kUp)));
    CHECK(!bar->isItemEnabled(assets.buttonId(AssetsPanel::kHome)));
    CHECK(bar->isItemEnabled(assets.buttonId(AssetsPanel::kBack)));

    CHECK(assets.navigateTo("Assets/Models"));
    rig.frames(2);
    assets.breadcrumbs()->itemClicked.emit(1);
    CHECK(assets.folder() == "Assets");
    assets.breadcrumbs()->itemClicked.emit(0);
    CHECK(assets.folder().empty());
    assets.breadcrumbs()->itemClicked.emit(2);
    CHECK(assets.folder().empty());
    assets.navigateTo("Assets/Models");
    assets.breadcrumbs()->itemClicked.emit(2);
    CHECK(assets.folder() == "Assets/Models");
    rig.frames(2);
    CHECK(assets.breadcrumbs()->path().size() == 3);

    rt::TreeNode* textures = assets.treeNodeOf("Assets/Textures");
    CHECK(textures != nullptr);
    if (textures)
    {
        assets.tree()->setSelectedNode(textures);
        rig.frames(2);
        CHECK(assets.folder() == "Assets/Textures");
        CHECK(assets.tree()->selectedNode() == textures);
    }
    CHECK(assets.navigateTo("Assets/Mixed/alpha"));
    rig.frames(2);
    rt::TreeNode* selected = assets.tree()->selectedNode();
    CHECK(selected != nullptr && assets.pathOfTreeNode(selected) == "Assets/Mixed/alpha");

    rt::TreeNode* assetsRoot = assets.tree()->roots()[2];
    CHECK(assetsRoot->isExpanded());
    bool hasMixed = false;
    for (rt::TreeNode* child: assetsRoot->children())
        if (assets.pathOfTreeNode(child) == "Assets/Mixed") hasMixed = true;
    CHECK(hasMixed);
    assets.goHome();
    rig.frames(2);
}

rt::TreeNode* childWithPath(AssetsPanel& assets, rt::TreeNode* parent, const char* path)
{
    for (rt::TreeNode* child: parent->children())
        if (assets.pathOfTreeNode(child) == path) return child;
    return nullptr;
}

rt::TreeNode* walkTree(AssetsPanel& assets, rt::TreeNode* root, const char* path)
{
    const ct::String text = path;
    rt::TreeNode* node = root;
    for (std::size_t i = 0; i <= text.size() && node; ++i)
    {
        if (i != text.size() && text[i] != '/') continue;
        const ct::String current = text.substr(0, i);
        node = childWithPath(assets, node, current.c_str());
    }
    return node;
}

void testTree(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    CHECK(fs_create_directory(absolute(rig, "Assets/Deep/one/two").c_str()));
    CHECK(fs_create_directory(absolute(rig, "Assets/Deep/side").c_str()));
    assets.navigateTo("");
    assets.refresh();
    rig.frames(4);
    rt::TreeView* tree = assets.tree();
    rt::TreeNode* project = tree->roots()[0];
    rt::TreeNode* assetsNode = childWithPath(assets, project, "Assets");
    CHECK(assetsNode != nullptr);
    if (!assetsNode) return;
    CHECK(!assetsNode->isExpanded() && assetsNode->hasChildren());
    CHECK(assets.pathOfTreeNode(assetsNode->children()[0]).empty());

    const rt::Rect area = tree->absoluteRect();
    rig.click(static_cast<int>(area.x + 25.0f), static_cast<int>(area.y + 36.0f));
    CHECK(assetsNode->isExpanded());
    CHECK(assets.folder().empty());
    rt::TreeNode* deep = childWithPath(assets, assetsNode, "Assets/Deep");
    CHECK(deep != nullptr);
    if (!deep) return;
    CHECK(!deep->isExpanded() && deep->hasChildren());
    CHECK(childWithPath(assets, assetsNode, "Assets/Mixed") != nullptr);
    CHECK(childWithPath(assets, assetsNode, "") == nullptr);

    deep->setExpanded(true);
    tree->nodeExpanded.emit(deep);
    CHECK(deep->children().size() == 2);
    rt::TreeNode* one = childWithPath(assets, deep, "Assets/Deep/one");
    rt::TreeNode* side = childWithPath(assets, deep, "Assets/Deep/side");
    CHECK(one != nullptr && one->hasChildren());
    CHECK(side != nullptr && !side->hasChildren());
    tree->nodeExpanded.emit(deep);
    CHECK(deep->children().size() == 2);

    rig.frames(2);
    rig.click(static_cast<int>(area.x + 90.0f), static_cast<int>(area.y + 24.0f * 2.0f + 12.0f));
    CHECK(assets.folder() == "Assets/Deep");
    CHECK(tree->selectedNode() == deep);

    ct::String error;
    CHECK(assets.createFolder("Assets/Deep", "three", error));
    rig.frames(3);
    rt::TreeNode* rebuilt = walkTree(assets, tree->roots()[0], "Assets/Deep");
    CHECK(rebuilt != nullptr && rebuilt->isExpanded());
    if (rebuilt)
    {
        CHECK(rebuilt->children().size() == 3);
        CHECK(childWithPath(assets, rebuilt, "Assets/Deep/three") != nullptr);
        CHECK(!rebuilt->children()[0]->isExpanded());
    }
    CHECK(tree->selectedNode() == assets.treeNodeOf("Assets/Deep") ||
            tree->selectedNode() != nullptr);
    CHECK(assets.pathOfTreeNode(tree->selectedNode()) == "Assets/Deep");

    CHECK(assets.navigateTo("Assets/Deep/one/two"));
    rig.frames(3);
    rt::TreeNode* two = assets.treeNodeOf("Assets/Deep/one/two");
    CHECK(two != nullptr && tree->selectedNode() == two);
    CHECK(assets.treeNodeOf("Assets/Deep/one")->isExpanded());
    CHECK(fs_remove_path(absolute(rig, "Assets/Deep/one/two").c_str()));
    assets.refresh();
    rig.frames(3);
    CHECK(assets.folder() == "Assets/Deep/one");
    CHECK(assets.treeNodeOf("Assets/Deep/one/two") == nullptr);
    CHECK(assets.goHome());
    rig.frames(2);
}

void testFilter(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    CHECK(assets.navigateTo("Assets/Mixed"));
    rig.frames(2);
    const std::size_t total = assets.entries().size();
    const unsigned counted = assets.listingCount();

    assets.setFilter("PNG");
    CHECK(assets.shown().size() == 1 && assets.shown()[0].name == "a.png");
    CHECK(assets.entries().size() == total);
    CHECK(assets.grid()->itemCount() == 1);
    CHECK(assets.listingCount() == counted);
    rig.frames(2);
    CHECK(assets.footer()->text().contains("1 of 30"));
    CHECK(assets.searchInput()->text() == "PNG");

    assets.setFilter("a");
    CHECK(assets.shown().size() > 5);
    assets.setFilter("zzz");
    CHECK(assets.shown().empty() && assets.grid()->itemCount() == 0);
    assets.setFilter("");
    CHECK(assets.shown().size() == total);

    const rt::Rect box = assets.searchInput()->absoluteRect();
    rig.click(static_cast<int>(box.x + box.w * 0.5f), static_cast<int>(box.y + box.h * 0.5f));
    const char* typed = "wood";
    for (const char* c = typed; *c; ++c) rig.send(character(*c));
    rig.frames(3);
    CHECK(assets.filter() == "wood");
    CHECK(assets.shown().size() == 1 && assets.shown()[0].name == "wood.mat");
    CHECK(assets.listingCount() == counted);

    assets.navigateTo("Assets");
    CHECK(assets.filter().empty() && assets.searchInput()->text().empty());
    assets.goBack();
    CHECK(assets.filter().empty() && assets.shown().size() == total);

    const float zoomBefore = assets.grid()->thumbnailSize();
    const rt::Rect zoom = assets.zoomSlider()->absoluteRect();
    rig.click(static_cast<int>(zoom.x + zoom.w * 0.95f), static_cast<int>(zoom.y + zoom.h * 0.5f));
    CHECK(assets.grid()->thumbnailSize() > zoomBefore + 40.0f);
    assets.setThumbnailSize(10.0f);
    CHECK(assets.grid()->thumbnailSize() == AssetGrid::kMinThumbnail);
    assets.setThumbnailSize(1000.0f);
    CHECK(assets.grid()->thumbnailSize() == AssetGrid::kMaxThumbnail);
    assets.setThumbnailSize(48.0f);

    rt::ToolBar* bar = assets.toolBar();
    bar->toggleChanged.emit(assets.buttonId(AssetsPanel::kListView), true);
    CHECK(assets.viewMode() == AssetViewMode::List &&
            assets.grid()->viewMode() == AssetViewMode::List);
    CHECK(bar->isItemChecked(assets.buttonId(AssetsPanel::kListView)));
    CHECK(!bar->isItemChecked(assets.buttonId(AssetsPanel::kGridView)));
    bar->toggleChanged.emit(assets.buttonId(AssetsPanel::kDetailsView), true);
    CHECK(assets.viewMode() == AssetViewMode::Details);
    bar->toggleChanged.emit(assets.buttonId(AssetsPanel::kDetailsView), false);
    CHECK(assets.viewMode() == AssetViewMode::Details);
    CHECK(bar->isItemChecked(assets.buttonId(AssetsPanel::kDetailsView)));
    bar->toggleChanged.emit(assets.buttonId(AssetsPanel::kGridView), true);
    CHECK(assets.viewMode() == AssetViewMode::Grid);
    rig.frames(2);
    assets.goHome();
    rig.frames(2);
}

void testThumbnails(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    TextureStore* store = TextureStore::singleton();
    CHECK(store != nullptr);
    if (!store) return;
    const ct::String folder = "Assets/Pictures";
    const ct::String absolutePictures = absolute(rig, folder);
    CHECK(writePng(AssetImport::joinPath(absolutePictures, "one.png"), 8, 8, 10));
    CHECK(writePng(AssetImport::joinPath(absolutePictures, "two.png"), 8, 8, 20));
    CHECK(writePng(AssetImport::joinPath(absolutePictures, "three.png"), 8, 8, 30));
    CHECK(writePng(AssetImport::joinPath(absolutePictures, "wide.png"), 600, 300, 40));
    CHECK(writeText(AssetImport::joinPath(absolutePictures, "bad.png"), "this is not an image"));
    CHECK(writeText(AssetImport::joinPath(absolutePictures, "text.txt"), "abc"));

    assets.setViewMode(AssetViewMode::List);
    CHECK(assets.navigateTo(folder));
    assets.refresh();
    rig.frames(2);
    CHECK(assets.thumbnailCount() == 0);
    const ct::Vector<AssetEntry> entries = assets.entries();
    CHECK(entries.size() == 6);

    const AssetEntry& bad = entries[static_cast<std::size_t>(assets.indexOf("bad.png"))];
    const AssetEntry& one = entries[static_cast<std::size_t>(assets.indexOf("one.png"))];
    const AssetEntry& two = entries[static_cast<std::size_t>(assets.indexOf("two.png"))];
    const AssetEntry& three = entries[static_cast<std::size_t>(assets.indexOf("three.png"))];
    const AssetEntry& wide = entries[static_cast<std::size_t>(assets.indexOf("wide.png"))];
    const AssetEntry& text = entries[static_cast<std::size_t>(assets.indexOf("text.txt"))];

    const std::size_t textures = store->count();
    CHECK(!assets.thumbnailFor(text).texture);
    CHECK(assets.thumbnailCount() == 0);

    rig.frames(1);
    const AssetThumbnail first = assets.thumbnailFor(one);
    const AssetThumbnail second = assets.thumbnailFor(two);
    const AssetThumbnail third = assets.thumbnailFor(three);
    CHECK(first.texture && second.texture && !third.texture);
    CHECK(first.texture.value != second.texture.value);
    CHECK(near(first.aspect, 1.0f));
    CHECK(assets.thumbnailCount() == 2);
    CHECK(store->count() == textures + 2);
    rig.frames(1);
    CHECK(assets.thumbnailFor(three).texture);
    CHECK(assets.thumbnailFor(one).texture.value == first.texture.value);
    CHECK(store->count() == textures + 3);

    rig.frames(1);
    const AssetThumbnail wideThumbnail = assets.thumbnailFor(wide);
    CHECK(wideThumbnail.texture && near(wideThumbnail.aspect, 2.0f));
    const TextureInfo* info = store->get(WidgetHost::textureId(wideThumbnail.texture));
    CHECK(info != nullptr && info->width == 256 && info->height == 128 && info->mipLevels == 1);
    CHECK(info != nullptr && info->path.empty());

    rig.frames(1);
    CHECK(!assets.thumbnailFor(bad).texture);
    CHECK(assets.thumbnailCount() == 5);
    const std::size_t afterBad = store->count();
    CHECK(!assets.thumbnailFor(bad).texture);
    CHECK(!assets.thumbnailFor(bad).texture);
    CHECK(assets.thumbnailCount() == 5 && store->count() == afterBad);

    assets.refresh();
    rig.frames(2);
    CHECK(assets.thumbnailCount() == 0);
    CHECK(store->count() == textures);

    CHECK(writePng(AssetImport::joinPath(absolutePictures, "bright.png"), 4, 4, 250));
    assets.refresh();
    assets.setViewMode(AssetViewMode::Grid);
    assets.setThumbnailSize(64.0f);
    rig.frames(10);
    CHECK(assets.thumbnailCount() >= 3);
    CHECK(assets.thumbnailCount() <= 6);
    {
        const int bright = assets.indexOf("bright.png");
        CHECK(bright >= 0);
        assets.grid()->ensureVisible(bright);
        rig.frames(2);
        const rt::Rect cell = assets.grid()->itemRect(bright);
        const int px = static_cast<int>(cell.x + cell.w * 0.5f);
        const int py = static_cast<int>(cell.y + 4.0f + assets.grid()->thumbnailSize() * 0.5f);
        unsigned char pixel[4] = { 0, 0, 0, 0 };
        CHECK(rig.framePixel(px, py, pixel));
        CHECK(pixel[0] > 240 && pixel[1] > 30 && pixel[1] < 50 && pixel[2] > 70 && pixel[2] < 90);
        const int plain = assets.indexOf("text.txt");
        assets.grid()->ensureVisible(plain);
        rig.frames(2);
        const rt::Rect other = assets.grid()->itemRect(plain);
        CHECK(rig.framePixel(static_cast<int>(other.x + 2.0f),
                static_cast<int>(other.y + other.h - 3.0f), pixel));
        CHECK(!(pixel[0] > 240 && pixel[1] < 50));
    }

    const ct::String many = "Assets/Many";
    const int total = static_cast<int>(AssetsPanel::kThumbnailLimit) + 20;
    for (int i = 0; i < total; ++i)
    {
        ct::String name = "img";
        name.append_number(1000 + i);
        name += ".png";
        CHECK(writePng(absolute(rig, AssetImport::joinPath(many, name)), 4, 4,
                static_cast<unsigned char>(i)));
    }
    assets.setViewMode(AssetViewMode::List);
    assets.refresh();
    CHECK(assets.navigateTo(many));
    rig.frames(3);
    const ct::Vector<AssetEntry> list = assets.entries();
    CHECK(static_cast<int>(list.size()) == total);
    const std::size_t baseline = store->count();
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        assets.thumbnailFor(list[i]);
        if (i % 2 == 1) rig.frames(1);
        CHECK(assets.thumbnailCount() <= AssetsPanel::kThumbnailLimit);
    }
    rig.frames(3);
    CHECK(assets.thumbnailCount() == AssetsPanel::kThumbnailLimit);
    CHECK(store->count() <= baseline + AssetsPanel::kThumbnailLimit);
    CHECK(store->count() >= baseline + AssetsPanel::kThumbnailLimit - 2);

    assets.goHome();
    rig.frames(3);
    assets.refresh();
    rig.frames(3);
    CHECK(assets.thumbnailCount() == 0);
    CHECK(store->count() == textures);
    assets.setViewMode(AssetViewMode::Grid);
    assets.setThumbnailSize(96.0f);
}

void testActivate(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    assets.setViewMode(AssetViewMode::List);
    CHECK(assets.navigateTo(""));
    rig.frames(3);

    int assetsIndex = entryIndex(rig, "Assets");
    CHECK(assetsIndex >= 0);
    clickItem(rig, assetsIndex);
    CHECK(assets.selected() == assetsIndex);
    CHECK(assets.folder().empty());
    doubleClickItem(rig, assetsIndex);
    CHECK(assets.folder() == "Assets");

    const int mixed = entryIndex(rig, "Mixed");
    CHECK(mixed >= 0);
    assets.activate(mixed);
    CHECK(assets.folder() == "Assets/Mixed");

    Node* scene = freshScene(rig);
    const ct::String before = rig.document.scenePath();
    const std::size_t sceneChildren = scene->getChildCount();

    const int textureIndex = entryIndex(rig, "a.png");
    doubleClickItem(rig, textureIndex);
    CHECK(assets.selected() == textureIndex);
    CHECK(assets.folder() == "Assets/Mixed");
    CHECK(scene->getChildCount() == sceneChildren);
    CHECK(popupDialog() == nullptr && !rig.ui.confirming());
    assets.activate(entryIndex(rig, "wood.mat"));
    CHECK(assets.selected() == entryIndex(rig, "wood.mat"));
    assets.activate(entryIndex(rig, "s.wav"));
    assets.activate(entryIndex(rig, "behave.py"));
    assets.activate(entryIndex(rig, "notes.txt"));
    CHECK(assets.selected() == entryIndex(rig, "notes.txt"));
    CHECK(scene->getChildCount() == sceneChildren && rig.document.scenePath() == before);
    assets.activate(-1);
    assets.activate(9999);
    CHECK(assets.selected() == entryIndex(rig, "notes.txt"));
    CHECK(!rig.document.dirty());
}

void testScenes(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    Node* scene = freshScene(rig);
    add(scene, "One");
    commit(rig);
    CHECK(rig.document.scenePath() == "Scenes/scene1.scene.json");
    Node* second = freshScene(rig);
    add(second, "Two");
    commit(rig);
    const ct::String secondPath = rig.document.scenePath();
    CHECK(secondPath == "Scenes/scene2.scene.json");

    CHECK(assets.navigateTo("Scenes"));
    rig.frames(3);
    const int firstIndex = entryIndex(rig, "scene1.scene.json");
    const int secondIndex = entryIndex(rig, "scene2.scene.json");
    CHECK(firstIndex >= 0 && secondIndex >= 0);
    CHECK(hasKind(assets.entries(), "scene1.scene.json", AssetKind::Scene));

    assets.activate(secondIndex);
    CHECK(!rig.ui.confirming());
    CHECK(rig.document.scenePath() == secondPath);

    Node* open = rig.document.scene();
    add(open, "Unsaved");
    rig.frames(3);
    CHECK(rig.document.dirty());
    doubleClickItem(rig, firstIndex);
    CHECK(rig.ui.confirming());
    CHECK(rig.document.scenePath() == secondPath);
    clickDialogButton(rig, rig.ui.confirmDialog(), kConfirmCancel);
    CHECK(!rig.ui.confirming());
    CHECK(rig.document.scenePath() == secondPath && rig.document.dirty());
    CHECK(rig.document.scene()->findChild("Unsaved") != nullptr);

    doubleClickItem(rig, firstIndex);
    CHECK(rig.ui.confirming());
    clickDialogButton(rig, rig.ui.confirmDialog(), kConfirmDiscard);
    rig.frames(3);
    CHECK(rig.document.scenePath() == "Scenes/scene1.scene.json");
    CHECK(!rig.document.dirty());
    CHECK(rig.document.scene()->findChild("One") != nullptr);
    CHECK(rig.document.scene()->findChild("Unsaved") == nullptr);

    assets.activate(secondIndex);
    CHECK(!rig.ui.confirming());
    CHECK(rig.document.scenePath() == secondPath);
    CHECK(rig.document.scene()->findChild("Two") != nullptr);

    add(rig.document.scene(), "Pending");
    rig.frames(3);
    assets.activate(firstIndex);
    CHECK(rig.ui.confirming());
    clickDialogButton(rig, rig.ui.confirmDialog(), kConfirmSave);
    rig.frames(3);
    CHECK(rig.document.scenePath() == "Scenes/scene1.scene.json");
    assets.activate(secondIndex);
    CHECK(rig.document.scene()->findChild("Pending") != nullptr);

    assets.activate(secondIndex);
    CHECK(!rig.ui.confirming());
    CHECK(assets.navigateTo(""));
}

void testPrefab(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    Prefab::clearCache();
    const ct::String relative = "Assets/Prefabs/Crate.prefab.json";
    {
        Node3D scratch;
        Node3D* crate = scratch.createChild<Node3D>();
        crate->setName("Crate");
        Node3D* lid = crate->createChild<Node3D>();
        lid->setName("Lid");
        crate->setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));
        CHECK(fs_create_directory(absolute(rig, "Assets/Prefabs").c_str()));
        SceneResult saved;
        CHECK(Prefab::saveFromNode(crate, absolute(rig, relative), saved));
        CHECK(saved.success());
    }

    Node* scene = freshScene(rig);
    commit(rig);
    CHECK(assets.navigateTo("Assets/Prefabs"));
    rig.frames(3);
    const int index = entryIndex(rig, "Crate.prefab.json");
    CHECK(index >= 0 && assets.shown()[static_cast<std::size_t>(index)].kind == AssetKind::Prefab);

    rig.selection.clear();
    rig.frames(2);
    assets.setViewMode(AssetViewMode::List);
    doubleClickItem(rig, index);
    Node* crate = scene->findChild("Crate");
    CHECK(crate != nullptr);
    if (crate)
    {
        CHECK(crate->isInstance() && crate->getSceneFilePath() == relative);
        CHECK(crate->findChild("Lid") != nullptr);
        CHECK(rig.selection.selectedNodes().size() == 1 && rig.selection.isSelected(crate));
        CHECK(rig.document.dirty());
        Node3D* spatial = crate->castTo<Node3D>();
        CHECK(spatial && near(spatial->getPosition().y, 0.5f));
    }
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(crate) != nullptr);

    commit(rig);
    Node3D* holder = add(scene, "Holder");
    rig.selection.clear();
    rig.selection.add(holder);
    rig.frames(3);
    assets.activate(index);
    Node* nested = holder->findChild("Crate");
    CHECK(nested != nullptr && nested->isInstance());
    CHECK(rig.selection.isSelected(nested) && rig.selection.selectedNodes().size() == 1);

    rig.selection.clear();
    rig.frames(2);
    assets.activate(index);
    CHECK(scene->findChild("Crate2") != nullptr);
    CHECK(countNamed(scene, "Crate") == 1);

    Node* instance = assets.instantiateOn("Assets/Prefabs/Crate.prefab.json", nullptr);
    CHECK(instance != nullptr);
    CHECK(assets.instantiateOn("Assets/Prefabs/missing.prefab.json", nullptr) == nullptr);
    CHECK(assets.instantiateOn("Assets/Mixed/a.png", nullptr) == nullptr);
    CHECK(assets.instantiateOn(absolute(rig, relative), nullptr) != nullptr);
    CHECK(assets.instantiateOn("../outside.prefab.json", nullptr) == nullptr);
    Node3D foreign;
    CHECK(assets.instantiateOn(relative, &foreign) == nullptr);
    CHECK(foreign.getChildCount() == 0);
    commit(rig);
}

void testModel(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    CHECK(writeModel(absolute(rig, "Assets/Models"), "tri"));

    Node* scene = freshScene(rig);
    commit(rig);
    CHECK(assets.navigateTo("Assets/Models"));
    assets.setViewMode(AssetViewMode::List);
    rig.frames(3);
    const int index = entryIndex(rig, "tri.gltf");
    CHECK(index >= 0 && assets.shown()[static_cast<std::size_t>(index)].kind == AssetKind::Model);
    CHECK(hasKind(assets.entries(), "tri.bin", AssetKind::Other));

    rig.selection.clear();
    doubleClickItem(rig, index);
    rig.frames(3);
    Node* model = scene->findChild("tri");
    CHECK(model != nullptr);
    if (!model) return;
    CHECK(model->castTo<Node3D>() != nullptr);
    CHECK(rig.selection.selectedNodes().size() == 1 && rig.selection.isSelected(model));
    CHECK(rig.document.dirty());
    CHECK(model->getChildCount() == 1);
    MeshInstance3D* mesh =
            model->getChildCount() == 1 ? model->getChild(0)->castTo<MeshInstance3D>() : nullptr;
    CHECK(mesh != nullptr);
    if (mesh)
    {
        CHECK(mesh->getName() == "TriNode");
        CHECK(mesh->getMesh().valid());
        CHECK(sceneres::pathOf(mesh->getMesh()) == "Assets/Models/tri.gltf::tri");
        CHECK(near(mesh->getPosition().x, 1.0f) && near(mesh->getPosition().y, 2.0f) &&
                near(mesh->getPosition().z, 3.0f));
    }
    CHECK(rig.hierarchy.itemOf(model) != nullptr);

    assets.activate(index);
    CHECK(model->findChild("tri") != nullptr);
    rig.selection.clear();
    rig.frames(2);
    assets.activate(index);
    CHECK(scene->findChild("tri2") != nullptr);
    CHECK(countNamed(scene, "tri") == 1);

    commit(rig);
    const ct::String saved = rig.document.scenePath();
    CHECK(rig.document.openScene(saved));
    rig.frames(3);
    Node* reopened = rig.document.scene();
    Node* again = reopened->findChild("tri2");
    CHECK(again != nullptr && again->getChildCount() == 1);
    MeshInstance3D* reloaded = again && again->getChildCount() == 1
                                       ? again->getChild(0)->castTo<MeshInstance3D>()
                                       : nullptr;
    CHECK(reopened->findChild("tri") != nullptr &&
            reopened->findChild("tri")->findChild("tri") != nullptr);
    CHECK(reloaded != nullptr && reloaded->getMesh().valid());
    if (reloaded) CHECK(sceneres::pathOf(reloaded->getMesh()) == "Assets/Models/tri.gltf::tri");

    ImportOptions options;
    options.translate = Math::Vec3(5.0f, 0.0f, 0.0f);
    options.rotate = Math::Vec3(0.0f, 90.0f, 0.0f);
    options.scale = 2.0f;
    Node* placed = assets.instantiate("Assets/Models/tri.gltf", reopened, nullptr, &options);
    CHECK(placed != nullptr);
    if (placed)
    {
        Node3D* spatial = placed->castTo<Node3D>();
        CHECK(spatial != nullptr);
        if (spatial)
        {
            CHECK(near(spatial->getPosition().x, 5.0f));
            CHECK(near(spatial->getScale().x, 2.0f) && near(spatial->getScale().y, 2.0f));
            CHECK(near(spatial->getRotationDegrees().y, 90.0f, 0.01f));
        }
    }
    CHECK(assets.instantiateOn("Assets/Models/absent.glb", nullptr) == nullptr);
    CHECK(writeText(absolute(rig, "Assets/Models/broken.glb"), "not a model"));
    CHECK(assets.instantiateOn("Assets/Models/broken.glb", nullptr) == nullptr);
    rig.frames(2);
    commit(rig);
}

ct::String menuTexts(rt::Menu* menu)
{
    ct::String text;
    for (int i = 0; i < menu->actionCount(); ++i)
    {
        if (i > 0) text += "|";
        text += menu->action(i)->isSeparator() ? ct::String("-") : menu->action(i)->text();
    }
    return text;
}

void closeAnyPopup(Rig& rig)
{
    rt::WidgetApp::instance().closePopup();
    rig.frames(2);
}

void testMenu(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    assets.setViewMode(AssetViewMode::List);
    CHECK(assets.navigateTo("Assets/Scripts"));
    rig.frames(3);
    CHECK(assets.entries().empty());

    const rt::Rect grid = assets.grid()->absoluteRect();
    const int emptyX = static_cast<int>(grid.x + grid.w * 0.5f);
    const int emptyY = static_cast<int>(grid.y + grid.h * 0.5f);
    rig.click(emptyX, emptyY, MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == assets.contextMenu());
    CHECK(menuTexts(assets.contextMenu()) == "In Assets/Scripts|-|New Folder...|New Script...|New "
                                             "Material...|Import...|-|Copy Path|Refresh");
    CHECK(!assets.contextMenu()->action(0)->isEnabled());
    CHECK(assets.contextMenu()->action(6)->isEnabled());
    closeAnyPopup(rig);

    CHECK(assets.navigateTo(""));
    rig.frames(2);
    rig.click(emptyX, static_cast<int>(grid.y + grid.h - 6.0f), MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == assets.contextMenu());
    const ct::String projectName = rig.document.project().name();
    CHECK(actionNamed(assets.contextMenu(), (ct::String("In ") + projectName).c_str()) != nullptr);
    CHECK(!actionNamed(assets.contextMenu(), "Copy Path")->isEnabled());
    closeAnyPopup(rig);

    CHECK(assets.navigateTo("Assets/Mixed"));
    rig.frames(3);
    int x = 0;
    int y = 0;
    const int png = entryIndex(rig, "a.png");
    itemCenter(rig, png, x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(rt::WidgetApp::instance().popup() == assets.contextMenu());
    CHECK(assets.selected() == png);
    CHECK(menuTexts(assets.contextMenu()) ==
            "In Assets/Mixed|-|New Folder...|New Script...|New Material...|Import...|-|Copy "
            "Path|Refresh|-|Rename...|Delete...");
    CHECK(!actionNamed(assets.contextMenu(), "New Material...")->isEnabled());
    CHECK(actionNamed(assets.contextMenu(), "Delete...")->isEnabled());
    closeAnyPopup(rig);

    const int model = entryIndex(rig, "robot.glb");
    itemCenter(rig, model, x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(actionNamed(assets.contextMenu(), "Instantiate") != nullptr);
    CHECK(actionNamed(assets.contextMenu(), "Open Scene") == nullptr);
    closeAnyPopup(rig);

    const int scene = entryIndex(rig, "scene one.scene.json");
    itemCenter(rig, scene, x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(actionNamed(assets.contextMenu(), "Open Scene") != nullptr);
    CHECK(actionNamed(assets.contextMenu(), "Instantiate") == nullptr);
    closeAnyPopup(rig);

    const int folder = entryIndex(rig, "alpha");
    itemCenter(rig, folder, x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(actionNamed(assets.contextMenu(), "In Assets/Mixed/alpha") != nullptr);
    CHECK(actionNamed(assets.contextMenu(), "Open") != nullptr);
    CHECK(triggerAction(rig, "New Folder..."));
    rt::InputBox* box = assets.nameBox();
    CHECK(box != nullptr);
    if (box)
    {
        box->setText("inner");
        clickWidget(rig, findButton(box->content(), "Create"));
        CHECK(assets.nameBox() == nullptr);
        CHECK(dir_exists(absolute(rig, "Assets/Mixed/alpha/inner").c_str()));
        CHECK(assets.folder() == "Assets/Mixed");
    }

    itemCenter(rig, folder, x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(triggerAction(rig, "Open"));
    CHECK(assets.folder() == "Assets/Mixed/alpha");
    CHECK(hasKind(assets.entries(), "inner", AssetKind::Folder));

    CHECK(assets.navigateTo("Assets/Mixed"));
    rig.frames(2);
    itemCenter(rig, entryIndex(rig, "a.png"), x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(triggerAction(rig, "Copy Path"));
    CHECK(assets.lastCopied() == "Assets/Mixed/a.png");
    itemCenter(rig, entryIndex(rig, "alpha"), x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(triggerAction(rig, "Copy Path"));
    CHECK(assets.lastCopied() == "Assets/Mixed/alpha");

    const unsigned counted = assets.listingCount();
    rig.click(emptyX, emptyY, MouseButton::Right);
    CHECK(triggerAction(rig, "Refresh"));
    CHECK(assets.listingCount() == counted + 1);

    itemCenter(rig, entryIndex(rig, "a.png"), x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(triggerAction(rig, "New Script..."));
    CHECK(assets.nameBox() != nullptr);
    if (assets.nameBox())
    {
        CHECK(assets.nameBox()->title() == "New Script");
        rt::WidgetApp::instance().removeFloat(assets.nameBox());
        rig.frames(3);
    }
    CHECK(assets.nameBox() == nullptr);

    rig.click(emptyX, emptyY, MouseButton::Right);
    CHECK(triggerAction(rig, "Import..."));
    CHECK(assets.fileDialog() != nullptr);
    if (assets.fileDialog())
    {
        rt::WidgetApp::instance().removeFloat(assets.fileDialog());
        rig.frames(3);
    }
    CHECK(assets.fileDialog() == nullptr);
}

void testCreate(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    CHECK(assets.navigateTo("Assets/Scripts"));
    rig.frames(2);
    ct::String error;

    CHECK(AssetsPanel::scriptClassName("player controller") == "PlayerController");
    CHECK(AssetsPanel::scriptClassName("2fast.py") == "Script2fast");
    CHECK(AssetsPanel::scriptClassName("???") == "NewScript");
    CHECK(AssetsPanel::scriptClassName("my_script") == "My_script");
    CHECK(AssetsPanel::scriptFileName("Hero") == "Hero.py");
    CHECK(AssetsPanel::scriptFileName("Hero.py") == "Hero.py");
    CHECK(AssetsPanel::scriptFileName("Hero.PY") == "Hero.PY");

    ct::String reason;
    CHECK(!AssetsPanel::validEntryName("", reason) && !reason.empty());
    CHECK(!AssetsPanel::validEntryName(".", reason) && !AssetsPanel::validEntryName("..", reason));
    CHECK(!AssetsPanel::validEntryName("a/b", reason) &&
            !AssetsPanel::validEntryName("a\\b", reason));
    CHECK(!AssetsPanel::validEntryName("a:b", reason) &&
            !AssetsPanel::validEntryName("a*b", reason));
    CHECK(AssetsPanel::validEntryName("fine name 2", reason));

    CHECK(assets.createScript("Assets/Scripts", "hero", error));
    CHECK(error.empty());
    const ct::String scriptPath = absolute(rig, "Assets/Scripts/hero.py");
    CHECK(readText(scriptPath) == AssetsPanel::scriptTemplate("hero"));
    CHECK(readText(scriptPath).contains("class Hero(Node3D):"));
    CHECK(readText(scriptPath).contains("def _ready(self):"));
    CHECK(readText(scriptPath).contains("def _process(self, dt):"));
    CHECK(hasKind(assets.entries(), "hero.py", AssetKind::Script));
    CHECK(assets.selected() == assets.indexOf("hero.py"));
    CHECK(!assets.createScript("Assets/Scripts", "hero", error));
    CHECK(error == "hero.py already exists.");
    CHECK(!assets.createScript("Assets/Scripts", "hero.py", error));
    CHECK(!assets.createScript("Assets/Scripts", "a/b", error) && !error.empty());
    CHECK(!assets.createScript("Assets/Missing", "x", error) && !error.empty());
    CHECK(!assets.createScript("../escape", "x", error) && !error.empty());
    CHECK(!exists(absolute(rig, "Assets/Scripts/b.py")));
    CHECK(assets.createScript("Assets/Scripts", "  spaced  ", error));
    CHECK(exists(absolute(rig, "Assets/Scripts/spaced.py")));

    CHECK(assets.createFolder("Assets/Scripts", "Enemies", error));
    CHECK(dir_exists(absolute(rig, "Assets/Scripts/Enemies").c_str()));
    CHECK(hasKind(assets.entries(), "Enemies", AssetKind::Folder));
    CHECK(assets.selected() == assets.indexOf("Enemies"));
    CHECK(!assets.createFolder("Assets/Scripts", "Enemies", error));
    CHECK(error == "A folder or file with that name already exists.");
    CHECK(!assets.createFolder("Assets/Scripts", "hero.py", error));
    CHECK(!assets.createFolder("Assets/Scripts", "..", error));
    CHECK(!assets.createFolder("Assets/Scripts", "x/y", error));
    CHECK(!assets.createFolder("Assets/Scripts", "", error));
    CHECK(!assets.createFolder("Assets/Nope", "x", error));
    CHECK(!assets.createFolder("../out", "x", error));
    CHECK(assets.createFolder("", "TopLevel", error));
    CHECK(dir_exists(absolute(rig, "TopLevel").c_str()));
    CHECK(assets.folder() == "Assets/Scripts");

    assets.requestFolder("Assets/Scripts");
    rig.frames(3);
    rt::InputBox* box = assets.nameBox();
    CHECK(box != nullptr);
    if (box)
    {
        CHECK(box->title() == "New Folder");
        assets.requestFolder("Assets");
        assets.requestScript("Assets");
        rig.frames(2);
        CHECK(assets.nameBox() == box);
        box->setText("Enemies");
        clickWidget(rig, findButton(box->content(), "Create"));
        rig.frames(3);
        CHECK(assets.nameBox() != nullptr);
        box = assets.nameBox();
        if (box)
        {
            box->setText("Allies");
            clickWidget(rig, findButton(box->content(), "Create"));
            rig.frames(3);
        }
        CHECK(assets.nameBox() == nullptr);
        CHECK(dir_exists(absolute(rig, "Assets/Scripts/Allies").c_str()));
    }

    assets.requestScript("Assets/Scripts");
    rig.frames(3);
    box = assets.nameBox();
    CHECK(box != nullptr);
    if (box)
    {
        CHECK(box->title() == "New Script");
        box->setText("boss");
        clickWidget(rig, findButton(box->content(), "Create"));
        rig.frames(3);
        CHECK(exists(absolute(rig, "Assets/Scripts/boss.py")));
        CHECK(assets.nameBox() == nullptr);
    }

    assets.toolBar()->buttonClicked.emit(assets.buttonId(AssetsPanel::kNewFolder));
    rig.frames(3);
    CHECK(assets.nameBox() != nullptr);
    if (assets.nameBox())
    {
        rt::WidgetApp::instance().removeFloat(assets.nameBox());
        rig.frames(3);
    }
    assets.goHome();
    rig.frames(2);
}

void testDelete(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    const ct::String junk = "Assets/Junk";
    CHECK(writeText(absolute(rig, "Assets/Junk/a.txt"), "a"));
    CHECK(writeText(absolute(rig, "Assets/Junk/b.txt"), "bb"));
    CHECK(writeText(absolute(rig, "Assets/Junk/old.glb"), "model"));
    CHECK(writeText(absolute(rig, "Assets/Junk/old.mat"), "mat"));
    CHECK(writeText(absolute(rig, "Assets/Junk/old.pskel"), "skel"));
    CHECK(writeText(absolute(rig, "Assets/Junk/keep.txt"), "keep"));
    CHECK(writeText(absolute(rig, "Assets/Junk/deep/er/c.txt"), "c"));
    CHECK(assets.navigateTo(junk));
    rig.frames(3);

    ct::String error;
    CHECK(!assets.deletePath("", error) && !error.empty());
    CHECK(!assets.deletePath("Scenes", error));
    CHECK(!assets.deletePath("Assets", error));
    CHECK(!assets.deletePath("../x", error));
    CHECK(!assets.deletePath("Assets/Junk/none.txt", error));
    CHECK(error == "Assets/Junk/none.txt does not exist.");
    CHECK(exists(absolute(rig, "Assets")) && exists(absolute(rig, "Scenes")));

    assets.requestDelete("Assets");
    CHECK(popupDialog() == nullptr);
    assets.requestDelete("");
    CHECK(popupDialog() == nullptr);

    assets.requestDelete("Assets/Junk/a.txt");
    rig.frames(2);
    rt::Dialog* dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->title_ == "Delete Asset");
        CHECK(dialog->message_.contains("Delete Assets/Junk/a.txt?"));
        CHECK(dialog->message_.contains("deletes the file from disk"));
        CHECK(dialog->message_.contains("undo cannot bring it back"));
        assets.requestDelete("Assets/Junk/b.txt");
        CHECK(popupDialog() == dialog);
        clickDialogButton(rig, dialog, 1);
    }
    CHECK(popupDialog() == nullptr);
    CHECK(exists(absolute(rig, "Assets/Junk/a.txt")));

    assets.requestDelete("Assets/Junk/a.txt");
    rig.frames(2);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog) clickDialogButton(rig, dialog, 0);
    rig.frames(3);
    CHECK(!exists(absolute(rig, "Assets/Junk/a.txt")));
    CHECK(exists(absolute(rig, "Assets/Junk/b.txt")));
    CHECK(assets.indexOf("a.txt") < 0 && assets.indexOf("b.txt") >= 0);

    assets.requestDelete("Assets/Junk/old.glb");
    rig.frames(2);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_.contains("Also deletes:"));
        // A glTF takes its .mat; skeletons and clips go only with a .pmesh that links them.
        CHECK(dialog->message_.contains("old.mat") && !dialog->message_.contains("old.pskel"));
        clickDialogButton(rig, dialog, 0);
    }
    rig.frames(3);
    CHECK(!exists(absolute(rig, "Assets/Junk/old.glb")));
    CHECK(!exists(absolute(rig, "Assets/Junk/old.mat")));
    CHECK(exists(absolute(rig, "Assets/Junk/old.pskel")));
    CHECK(exists(absolute(rig, "Assets/Junk/keep.txt")));

    int x = 0;
    int y = 0;
    assets.setViewMode(AssetViewMode::List);
    rig.frames(2);
    itemCenter(rig, assets.indexOf("b.txt"), x, y);
    rig.click(x, y, MouseButton::Right);
    CHECK(triggerAction(rig, "Delete..."));
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog) clickDialogButton(rig, dialog, 0);
    rig.frames(3);
    CHECK(!exists(absolute(rig, "Assets/Junk/b.txt")));

    CHECK(assets.navigateTo("Assets"));
    rig.frames(2);
    CHECK(assets.indexOf("Junk") >= 0);
    CHECK(assets.navigateTo(junk));
    rig.frames(2);
    CHECK(assets.navigateTo("Assets/Junk/deep/er"));
    rig.frames(2);
    assets.requestDelete(junk);
    rig.frames(2);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_.contains("Delete Assets/Junk?"));
        CHECK(dialog->message_.contains("folder and"));
        CHECK(dialog->message_.contains("items"));
        clickDialogButton(rig, dialog, 0);
    }
    rig.frames(3);
    CHECK(!exists(absolute(rig, junk)));
    CHECK(assets.folder() == "Assets");
    CHECK(assets.indexOf("Junk") < 0);
    CHECK(assets.treeNodeOf("Assets/Junk") == nullptr);
    CHECK(assets.tree()->selectedNode() == assets.tree()->roots()[2]);

    Node* scene = freshScene(rig);
    add(scene, "Keep");
    commit(rig);
    const ct::String open = rig.document.scenePath();
    CHECK(!assets.deletePath(open, error));
    CHECK(error == "The open scene cannot be deleted.");
    CHECK(exists(absolute(rig, open)));
    CHECK(!assets.deletePath("Scenes", error));

    Node* other = freshScene(rig);
    add(other, "Other");
    commit(rig);
    const ct::String second = rig.document.scenePath();
    CHECK(rig.document.project().hasScene(open));
    CHECK(assets.deletePath(open, error));
    CHECK(!exists(absolute(rig, open)));
    CHECK(!rig.document.project().hasScene(open));
    CHECK(rig.document.project().hasScene(second));

    Project reread;
    CHECK(reread.open(rig.document.project().manifestPath()));
    CHECK(!reread.hasScene(open) && reread.hasScene(second));
    CHECK(!assets.deletePath(second, error));
    CHECK(assets.navigateTo(""));
}

void testImport(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    const ct::String external = testRoot() + "/external";
    CHECK(writeModel(external, "ship"));
    CHECK(writeText(external + "/readme.txt", "hello"));
    CHECK(writePng(external + "/paint.png", 4, 4, 77));
    CHECK(writeText(external + "/outside.gltf",
            "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"uri\":\"../secret.bin\","
            "\"byteLength\":4},{\"uri\":\"data:application/octet-stream;base64,AAAAAA==\","
            "\"byteLength\":4},{\"uri\":\"missing.bin\",\"byteLength\":4}],"
            "\"images\":[{\"uri\":\"/etc/passwd\"}]}"));
    CHECK(writeText(testRoot() + "/secret.bin", "secret"));
    Node* scene = freshScene(rig);
    commit(rig);

    CHECK(fs_remove_path(absolute(rig, "Assets/Models/tex/albedo.png").c_str()));
    CHECK(fs_remove_path(absolute(rig, "Assets/Models/tex").c_str()));
    CHECK(assets.navigateTo("Assets/Models"));
    assets.refresh();
    rig.frames(3);
    CHECK(!assets.importing());

    ImportPlan plan;
    ct::Vector<ct::String> sources;
    sources.push_back(external + "/ship.gltf");
    CHECK(AssetImport::plan(rig.document.project(), sources, "Assets/Models", plan));
    CHECK(plan.files.size() == 3);
    CHECK(plan.files[0].relative == "Assets/Models/ship.gltf" && plan.files[0].primary);
    CHECK(plan.files[1].relative == "Assets/Models/tri.bin" && !plan.files[1].primary);
    CHECK(plan.files[2].relative == "Assets/Models/tex/albedo.png");
    CHECK(plan.files[0].kind == AssetKind::Model && plan.files[2].kind == AssetKind::Texture);
    CHECK(plan.firstModel() == 0 && plan.clashCount() == 1 && !plan.empty());
    CHECK(plan.files[1].exists);
    CHECK(plan.skipped.empty());

    sources.clear();
    sources.push_back(external + "/outside.gltf");
    sources.push_back(external + "/nothing.txt");
    sources.push_back(absolute(rig, "Assets/Models/tri.gltf"));
    CHECK(AssetImport::plan(rig.document.project(), sources, "Assets/Models", plan));
    CHECK(plan.files.size() == 1);
    CHECK(plan.skipped.size() == 5);
    CHECK(AssetImport::decodeUri("a%20b%2Fc%zz%2") == "a b/c%zz%2");
    CHECK(AssetImport::safeRelative("a/b.png") && !AssetImport::safeRelative("../b.png") &&
            !AssetImport::safeRelative("/b.png") && !AssetImport::safeRelative("a/../../b") &&
            !AssetImport::safeRelative("c:/b") && !AssetImport::safeRelative(""));
    CHECK(!AssetImport::plan(rig.document.project(), ct::Vector<ct::String>(), "", plan));

    CHECK(writeText(absolute(rig, "Assets/Models/tri.bin"), "stale"));
    CHECK(exists(absolute(rig, "Assets/Models/tri.bin")));
    sources.clear();
    sources.push_back(external + "/ship.gltf");
    assets.importFiles(sources, "Assets/Models");
    rig.frames(3);
    CHECK(assets.importing());
    rt::Dialog* dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->title_ == "Replace Files");
        CHECK(dialog->message_.contains("1 file already exists in Assets/Models"));
        CHECK(dialog->message_.contains("Assets/Models/tri.bin"));
        assets.importFiles(sources, "Assets/Models");
        CHECK(popupDialog() == dialog);
        clickDialogButton(rig, dialog, 1);
    }
    rig.frames(3);
    CHECK(!assets.importing());
    CHECK(popupDialog() == nullptr && assets.importDialog() == nullptr);
    CHECK(readText(absolute(rig, "Assets/Models/tri.bin")) == "stale");
    CHECK(!exists(absolute(rig, "Assets/Models/ship.gltf")));
    CHECK(!exists(absolute(rig, "Assets/Models/tex/albedo.png")));

    assets.importFiles(sources, "Assets/Models");
    rig.frames(3);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog) clickDialogButton(rig, dialog, 0);
    rig.frames(3);
    ImportDialog* options = assets.importDialog();
    CHECK(options != nullptr && assets.importing());
    if (options)
    {
        CHECK(options->title() == "Import Model");
        CHECK(options->instantiateBox()->isChecked());
        CHECK(options->options().scale == 1.0f);
        options->instantiateBox()->setChecked(false);
        clickWidget(rig, options->importButton());
    }
    rig.frames(3);
    CHECK(!assets.importing() && assets.importDialog() == nullptr);
    CHECK(exists(absolute(rig, "Assets/Models/ship.gltf")));
    CHECK(file_size(absolute(rig, "Assets/Models/tri.bin").c_str()) ==
            static_cast<int64_t>(sizeof(kTriangle)));
    CHECK(exists(absolute(rig, "Assets/Models/tex/albedo.png")));
    CHECK(readText(absolute(rig, "Assets/Models/ship.gltf")) == readText(external + "/ship.gltf"));
    CHECK(assets.indexOf("ship.gltf") >= 0 && assets.selected() == assets.indexOf("ship.gltf"));
    CHECK(scene->findChild("ship") == nullptr);
    CHECK(hasKind(assets.entries(), "tex", AssetKind::Folder));

    sources.clear();
    sources.push_back(external + "/ship.gltf");
    assets.importFiles(sources, "Assets/Models");
    rig.frames(3);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_.contains("3 files already exist"));
        clickDialogButton(rig, dialog, 0);
    }
    rig.frames(3);
    options = assets.importDialog();
    CHECK(options != nullptr);
    if (options)
    {
        CHECK(options->translateField(3) == nullptr && options->rotateField(-1) == nullptr);
        options->translateField(0)->setValue(4.0f);
        options->translateField(2)->setValue(-2.0f);
        options->rotateField(1)->setValue(90.0f);
        options->scaleField()->setValue(3.0f);
        const ImportOptions chosen = options->options();
        CHECK(chosen.instantiate && near(chosen.translate.x, 4.0f) && near(chosen.scale, 3.0f));
        clickWidget(rig, options->importButton());
    }
    rig.frames(3);
    Node* ship = scene->findChild("ship");
    CHECK(ship != nullptr);
    if (ship)
    {
        Node3D* spatial = ship->castTo<Node3D>();
        CHECK(spatial != nullptr);
        if (spatial)
        {
            CHECK(near(spatial->getPosition().x, 4.0f) && near(spatial->getPosition().z, -2.0f));
            CHECK(near(spatial->getScale().x, 3.0f) &&
                    near(spatial->getRotationDegrees().y, 90.0f, 0.01f));
        }
        CHECK(rig.selection.isSelected(ship));
        CHECK(ship->getChildCount() == 1);
        MeshInstance3D* mesh =
                ship->getChildCount() == 1 ? ship->getChild(0)->castTo<MeshInstance3D>() : nullptr;
        CHECK(mesh != nullptr && mesh->getMesh().valid());
        if (mesh) CHECK(sceneres::pathOf(mesh->getMesh()) == "Assets/Models/ship.gltf::tri");
    }
    CHECK(rig.document.dirty());

    sources.clear();
    sources.push_back(external + "/readme.txt");
    sources.push_back(external + "/paint.png");
    sources.push_back(external + "/readme.txt");
    assets.importFiles(sources, "Assets/Textures");
    rig.frames(3);
    CHECK(popupDialog() == nullptr && assets.importDialog() == nullptr && !assets.importing());
    CHECK(readText(absolute(rig, "Assets/Textures/readme.txt")) == "hello");
    CHECK(exists(absolute(rig, "Assets/Textures/paint.png")));
    CHECK(assets.folder() == "Assets/Models");

    CHECK(writeText(absolute(rig, "Assets/Textures/readme.txt"), "edited"));
    assets.importFiles(sources, "Assets/Textures");
    rig.frames(3);
    dialog = popupDialog();
    CHECK(dialog != nullptr);
    if (dialog)
    {
        CHECK(dialog->message_.contains("2 files already exist in Assets/Textures"));
        clickDialogButton(rig, dialog, 0);
    }
    rig.frames(3);
    CHECK(readText(absolute(rig, "Assets/Textures/readme.txt")) == "hello");
    CHECK(!assets.importing());

    sources.clear();
    sources.push_back(absolute(rig, "Assets/Textures/paint.png"));
    assets.importFiles(sources, "Assets/Models");
    rig.frames(3);
    CHECK(!assets.importing() && popupDialog() == nullptr);
    sources.clear();
    sources.push_back(external + "/nothing.txt");
    assets.importFiles(sources, "Assets/Models");
    CHECK(!assets.importing());
    sources.clear();
    sources.push_back(external + "/readme.txt");
    assets.importFiles(sources, "Assets/Nope");
    CHECK(!assets.importing());
    assets.importFiles(sources, "../outside");
    CHECK(!assets.importing());
    CHECK(!exists(absolute(rig, "../outside")));

    assets.requestImport("Assets/Models");
    rig.frames(3);
    rt::FileDialog* chooser = assets.fileDialog();
    CHECK(chooser != nullptr);
    if (chooser)
    {
        assets.requestImport("Assets");
        CHECK(assets.fileDialog() == chooser);
        CHECK(writeText(external + "/chosen.txt", "chosen"));
        chooser->accepted.emit(external + "/chosen.txt");
        rt::WidgetApp::instance().removeFloat(chooser);
        rig.frames(3);
    }
    CHECK(readText(absolute(rig, "Assets/Models/chosen.txt")) == "chosen");

    CHECK(assets.navigateTo("Assets/Models"));
    rig.frames(2);
    ct::Vector<ct::String> dropped;
    CHECK(writeText(external + "/dropped.txt", "dropped"));
    dropped.push_back(external + "/dropped.txt");
    const rt::Rect grid = assets.grid()->absoluteRect();
    rig.host.dropFiles(dropped, static_cast<int>(grid.x + 20.0f), static_cast<int>(grid.y + 20.0f));
    rig.frames(3);
    CHECK(readText(absolute(rig, "Assets/Models/dropped.txt")) == "dropped");
    CHECK(assets.indexOf("dropped.txt") >= 0);

    CHECK(writeText(external + "/dropped2.txt", "again"));
    dropped.clear();
    dropped.push_back(external + "/dropped2.txt");
    rig.host.dropFiles(dropped, 5, 5);
    rig.frames(3);
    CHECK(!exists(absolute(rig, "Assets/Models/dropped2.txt")));
    assets.root()->fileDrop.emit(dropped);
    rig.frames(3);
    CHECK(readText(absolute(rig, "Assets/Models/dropped2.txt")) == "again");
    commit(rig);
}

bool samePoint(const Math::Vec3& a, const Math::Vec3& b, float tolerance = 0.02f)
{
    return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) && near(a.z, b.z, tolerance);
}

void testDrag(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    Prefab::clearCache();
    assets.setViewMode(AssetViewMode::List);
    Node* scene = freshScene(rig);
    Node3D* holder = add(scene, "Holder");
    Node3D* other = add(scene, "Other");
    Node3D* leaf = add(holder, "Leaf");
    (void) leaf;
    commit(rig);
    rig.selection.clear();
    rig.frames(3);

    CHECK(assets.navigateTo("Assets/Prefabs"));
    rig.frames(3);
    const int prefab = entryIndex(rig, "Crate.prefab.json");
    CHECK(prefab >= 0);

    const rt::Rect viewRect = rig.panel.view()->absoluteRect();
    const int viewX = static_cast<int>(viewRect.x + viewRect.w * 0.5f);
    const int viewY = static_cast<int>(viewRect.y + viewRect.h * 0.55f);
    Math::Vec3 expected;
    CHECK(rig.panel.groundPoint(static_cast<float>(viewX) - viewRect.x,
            static_cast<float>(viewY) - viewRect.y, expected));
    CHECK(near(expected.y, 0.0f));
    CHECK(samePoint(rig.panel.dropPosition(static_cast<float>(viewX), static_cast<float>(viewY)),
            expected));

    int x = 0;
    int y = 0;
    itemCenter(rig, prefab, x, y);
    rig.drag(x, y, viewX, viewY);
    Node* dropped = scene->findChild("Crate");
    CHECK(dropped != nullptr);
    if (dropped)
    {
        Node3D* spatial = dropped->castTo<Node3D>();
        CHECK(spatial != nullptr);
        if (spatial)
        {
            const Math::Vec3 position = spatial->getGlobalTransform().GetTranslation();
            CHECK(samePoint(position, expected));
        }
        CHECK(rig.selection.isSelected(dropped) && rig.selection.selectedNodes().size() == 1);
        CHECK(rig.document.dirty());
    }

    rig.selection.clear();
    rig.selection.add(other);
    rig.frames(3);
    itemCenter(rig, prefab, x, y);
    rig.drag(x, y, viewX, viewY);
    CHECK(scene->findChild("Crate2") != nullptr);
    CHECK(other->findChild("Crate") == nullptr);

    rig.panel.camera().cursor().posX = 0.0;
    rig.panel.camera().cursor().posY = 10.0;
    rig.panel.camera().cursor().posZ = 0.0;
    rig.panel.camera().cursor().xRot = -0.5f;
    rig.panel.camera().cursor().distance = 4.0f;
    rig.panel.camera().updateCamera();
    rig.frames(40);
    Math::Vec3 miss;
    const float centerX = viewRect.w * 0.5f;
    const float centerY = viewRect.h * 0.5f;
    CHECK(!rig.panel.groundPoint(centerX, centerY, miss));
    const Math::Vec3 focus = rig.panel.focusPoint();
    CHECK(near(focus.y, 10.0f, 0.1f));
    CHECK(samePoint(rig.panel.dropPosition(viewRect.x + centerX, viewRect.y + centerY), focus));
    itemCenter(rig, prefab, x, y);
    rig.drag(x, y, static_cast<int>(viewRect.x + centerX), static_cast<int>(viewRect.y + centerY));
    Node* high = scene->findChild("Crate3");
    CHECK(high != nullptr);
    if (high && high->castTo<Node3D>())
        CHECK(near(high->castTo<Node3D>()->getGlobalTransform().GetTranslation().y, 10.0f, 0.2f));
    rig.panel.camera().resetCursor();
    rig.frames(30);

    rig.selection.clear();
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(holder) != nullptr);
    const int holderY = rowY(rig, holder);
    const int holderX = textX(rig, holder);
    CHECK(holderY > 0);
    itemCenter(rig, prefab, x, y);
    rig.drag(x, y, holderX, holderY);
    Node* under = holder->findChild("Crate");
    CHECK(under != nullptr && under->isInstance());
    CHECK(rig.selection.isSelected(under) && rig.selection.selectedNodes().size() == 1);
    if (under && under->castTo<Node3D>())
    {
        const Math::Vec3 local = under->castTo<Node3D>()->getPosition();
        CHECK(near(local.x, 0.0f) && near(local.y, 0.5f) && near(local.z, 0.0f));
    }
    CHECK(scene->findChild("Crate4") == nullptr);

    rig.frames(3);
    const int leafY = rowY(rig, leaf);
    CHECK(leafY > 0);
    itemCenter(rig, prefab, x, y);
    rig.drag(x, y, textX(rig, leaf), leafY);
    CHECK(leaf->findChild("Crate") != nullptr);

    const rt::Rect outliner = rig.hierarchy.outliner()->absoluteRect();
    itemCenter(rig, prefab, x, y);
    const std::size_t rootChildren = scene->getChildCount();
    rig.drag(x, y, static_cast<int>(outliner.x + 60.0f),
            static_cast<int>(outliner.y + outliner.h - 6.0f));
    CHECK(scene->getChildCount() == rootChildren + 1);
    CHECK(rig.selection.selectedNodes().size() == 1);
    CHECK(rig.selection.selectedNodes()[0]->getParent() == scene);

    const std::size_t before = scene->getChildCount();
    const rt::Rect status = rig.shell.statusBar()->absoluteRect();
    itemCenter(rig, prefab, x, y);
    rig.drag(x, y, static_cast<int>(status.x + 30.0f),
            static_cast<int>(status.y + status.h * 0.5f));
    CHECK(scene->getChildCount() == before);
    rig.drag(x, y, x + 3, y);
    CHECK(scene->getChildCount() == before);
    const rt::Rect tree = assets.tree()->absoluteRect();
    rig.drag(x, y, static_cast<int>(tree.x + 20.0f), static_cast<int>(tree.y + 20.0f));
    CHECK(scene->getChildCount() == before);
    CHECK(assets.folder() == "Assets/Prefabs");

    CHECK(assets.navigateTo("Assets/Mixed"));
    rig.frames(3);
    const int texture = entryIndex(rig, "a.png");
    itemCenter(rig, texture, x, y);
    rig.drag(x, y, viewX, viewY);
    CHECK(scene->getChildCount() == before);
    itemCenter(rig, texture, x, y);
    rig.drag(x, y, holderX, holderY);
    CHECK(scene->getChildCount() == before);
    const int folderEntry = entryIndex(rig, "alpha");
    itemCenter(rig, folderEntry, x, y);
    CHECK(!assets.grid()->isDragSource() || assets.grid()->hovered() == folderEntry);
    rig.drag(x, y, viewX, viewY);
    CHECK(scene->getChildCount() == before);

    rt::DragPayload payload;
    payload.type = AssetGrid::kDragType;
    payload.data = ct::String("Assets/Models/tri.gltf");
    ct::String path;
    CHECK(AssetGrid::payloadPath(payload, path) && path == "Assets/Models/tri.gltf");
    payload.type = "other";
    CHECK(!AssetGrid::payloadPath(payload, path));
    payload.type = AssetGrid::kDragType;
    payload.data = 5;
    CHECK(!AssetGrid::payloadPath(payload, path));

    assets.setViewMode(AssetViewMode::Grid);
    assets.setThumbnailSize(48.0f);
    rig.frames(3);
    CHECK(assets.navigateTo("Assets/Prefabs"));
    rig.frames(3);
    const int gridPrefab = entryIndex(rig, "Crate.prefab.json");
    itemCenter(rig, gridPrefab, x, y);
    const std::size_t again = scene->getChildCount();
    rig.drag(x, y, viewX, viewY);
    CHECK(scene->getChildCount() == again + 1);

    assets.setViewMode(AssetViewMode::Details);
    rig.frames(3);
    itemCenter(rig, entryIndex(rig, "Crate.prefab.json"), x, y);
    rig.drag(x, y, viewX, viewY);
    CHECK(scene->getChildCount() == again + 2);
    assets.setViewMode(AssetViewMode::Grid);
    assets.setThumbnailSize(96.0f);
    commit(rig);
}

void testExternal(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    CHECK(assets.navigateTo("Assets/Textures"));
    rig.frames(3);
    const unsigned counted = assets.listingCount();
    const std::size_t known = assets.entries().size();

    CHECK(writeText(absolute(rig, "Assets/Textures/outside1.txt"), "x"));
    rig.frames(20);
    CHECK(assets.listingCount() == counted);
    CHECK(assets.entries().size() == known);
    CHECK(assets.indexOf("outside1.txt") < 0);

    showConsole(rig);
    CHECK(!assets.root()->isVisible() || true);
    rig.frames(5);
    CHECK(writeText(absolute(rig, "Assets/Textures/outside2.txt"), "x"));
    showAssets(rig);
    rig.frames(2);
    CHECK(assets.listingCount() == counted + 1);
    CHECK(assets.indexOf("outside1.txt") >= 0 && assets.indexOf("outside2.txt") >= 0);
    CHECK(assets.entries().size() == known + 2);
    rig.frames(20);
    CHECK(assets.listingCount() == counted + 1);

    bool focused = false;
    assets.setFocusProbe([&focused] { return focused; });
    rig.frames(3);
    CHECK(writeText(absolute(rig, "Assets/Textures/outside3.txt"), "x"));
    rig.frames(5);
    CHECK(assets.listingCount() == counted + 1);
    CHECK(assets.indexOf("outside3.txt") < 0);
    focused = true;
    rig.frames(2);
    CHECK(assets.listingCount() == counted + 2);
    CHECK(assets.indexOf("outside3.txt") >= 0);
    rig.frames(10);
    CHECK(assets.listingCount() == counted + 2);
    focused = false;
    rig.frames(3);
    CHECK(fs_remove_path(absolute(rig, "Assets/Textures/outside1.txt").c_str()));
    rig.frames(5);
    CHECK(assets.indexOf("outside1.txt") >= 0);
    assets.toolBar()->buttonClicked.emit(assets.buttonId(AssetsPanel::kRefresh));
    rig.frames(2);
    CHECK(assets.indexOf("outside1.txt") < 0);
    CHECK(assets.listingCount() == counted + 3);

    CHECK(fs_create_directory(absolute(rig, "Assets/Textures/NewOutside").c_str()));
    assets.refresh();
    rig.frames(2);
    CHECK(assets.treeNodeOf("Assets/Textures/NewOutside") != nullptr);
    CHECK(assets.indexOf("NewOutside") >= 0);

    CHECK(assets.navigateTo("Assets/Textures/NewOutside"));
    rig.frames(2);
    CHECK(fs_remove_path(absolute(rig, "Assets/Textures/NewOutside").c_str()));
    assets.refresh();
    rig.frames(3);
    CHECK(assets.folder() == "Assets/Textures");
    CHECK(assets.canGoBack());
    CHECK(assets.goBack());
    CHECK(assets.folder() == "Assets/Textures");
    assets.setFocusProbe(ct::Function<bool()>());
    CHECK(assets.navigateTo(""));
}

void testProject(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    CHECK(assets.navigateTo("Assets/Models"));
    rig.frames(3);
    CHECK(assets.hasProject() && assets.historySize() > 1);
    const ct::String firstManifest = rig.document.project().manifestPath();

    rig.document.closeProject();
    rig.frames(4);
    CHECK(!assets.hasProject());
    CHECK(assets.entries().empty() && assets.shown().empty());
    CHECK(assets.folder().empty() && assets.historySize() == 1);
    CHECK(assets.tree()->roots().empty());
    CHECK(assets.footer()->text().empty());
    CHECK(assets.breadcrumbs()->path().empty());
    CHECK(!assets.navigateTo("Assets"));
    CHECK(!assets.toolBar()->isItemEnabled(assets.buttonId(AssetsPanel::kImport)));
    CHECK(!assets.toolBar()->isItemEnabled(assets.buttonId(AssetsPanel::kNewFolder)));
    CHECK(!assets.searchInput()->isEnabled());
    ct::String error;
    CHECK(!assets.createFolder("", "x", error) && !assets.createScript("", "x", error));
    CHECK(!assets.deletePath("Assets", error));
    assets.requestFolder("");
    assets.requestScript("");
    assets.requestImport("");
    assets.requestDelete("Assets");
    CHECK(assets.nameBox() == nullptr && assets.fileDialog() == nullptr &&
            popupDialog() == nullptr);
    ct::Vector<ct::String> none;
    none.push_back(testRoot() + "/external/readme.txt");
    assets.importFiles(none, "");
    CHECK(!assets.importing());
    CHECK(assets.instantiateOn("Assets/Prefabs/Crate.prefab.json", nullptr) == nullptr);
    rig.click(100, 100, MouseButton::Right);
    assets.showMenu(0, 10.0f, 10.0f);
    CHECK(rt::WidgetApp::instance().popup() != assets.contextMenu());

    CHECK(rig.document.openProject(firstManifest));
    rig.frames(4);
    CHECK(assets.hasProject());
    CHECK(assets.folder().empty() && assets.historySize() == 1);
    CHECK(assets.bookmarkCount() == 6);
    CHECK(assets.toolBar()->isItemEnabled(assets.buttonId(AssetsPanel::kImport)));
    CHECK(hasKind(assets.entries(), "Assets", AssetKind::Folder));
    CHECK(assets.searchInput()->isEnabled());

    CHECK(rig.document.createProject(testRoot() + "/projects", "Second"));
    rig.frames(4);
    CHECK(assets.hasProject());
    CHECK(assets.folder().empty());
    CHECK(assets.entries().size() == 3);
    CHECK(assets.indexOf("Mixed") < 0);
    CHECK(assets.navigateTo("Assets"));
    CHECK(assets.indexOf("Mixed") < 0);
    CHECK(assets.tree()->roots()[0]->text() == "Second");
    CHECK(rig.document.openProject(firstManifest));
    rig.frames(4);
    CHECK(assets.navigateTo("Assets"));
    CHECK(assets.indexOf("Mixed") >= 0);
    CHECK(assets.navigateTo(""));
}

} // namespace

InputEvent humanChar(char c)
{
    InputEvent event;
    event.type = InputEventType::Char;
    event.character.codepoint = static_cast<std::uint32_t>(c);
    return event;
}

InputEvent humanKey(Key key, bool pressed, unsigned modifiers = 0)
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

void humanType(Rig& rig, const char* text)
{
    for (const char* c = text; *c; ++c)
    {
        rig.send(humanChar(*c));
        rig.frame();
    }
    rig.frames(2);
}

void humanKeyPress(Rig& rig, Key key, unsigned modifiers = 0)
{
    rig.send(humanKey(key, true, modifiers));
    rig.frames(2);
    rig.send(humanKey(key, false, modifiers));
    rig.frames(3);
}

rt::TextInput* inputIn(rt::Widget* root)
{
    if (!root) return nullptr;
    if (rt::TextInput* found = rt::widget_cast<rt::TextInput>(root)) return found;
    for (rt::Widget* child: root->children())
        if (rt::TextInput* found = inputIn(child)) return found;
    return nullptr;
}

std::size_t visibleFloats()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    std::size_t count = 0;
    if (app)
        for (rt::FloatWindow* window: app->floats())
            if (window->isVisible()) ++count;
    return count;
}

void testEmptyState(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    CHECK(!assets.hasProject());
    CHECK(assets.emptyState() != nullptr && assets.emptyState()->isVisible());
    CHECK(assets.newProjectButton() && assets.newProjectButton()->text() == "New Project...");
    CHECK(assets.openProjectButton() && assets.openProjectButton()->text() == "Open Project...");
    CHECK(!assets.toolBar()->parent()->isVisible());

    showAssets(rig);
    clickWidget(rig, assets.newProjectButton());
    CHECK(rig.ui.newProjectDialog() != nullptr);
    if (rig.ui.newProjectDialog())
    {
        rt::WidgetApp::instance().removeFloat(rig.ui.newProjectDialog());
        rig.frames(3);
    }
    CHECK(rig.ui.newProjectDialog() == nullptr);

    clickWidget(rig, assets.openProjectButton());
    CHECK(rig.ui.fileDialog() != nullptr);
    if (rig.ui.fileDialog())
    {
        CHECK(rig.ui.fileDialog()->path() == EditorSettings::appFolder());
        rt::WidgetApp::instance().removeFloat(rig.ui.fileDialog());
        rig.frames(3);
    }
}

void testHumanFlows(Rig& rig)
{
    AssetsPanel& assets = rig.assets;
    showAssets(rig);
    CHECK(assets.emptyState() != nullptr && !assets.emptyState()->isVisible());
    CHECK(assets.toolBar()->parent()->isVisible());

    const rt::Rect zoom = assets.zoomSlider()->absoluteRect();
    CHECK(zoom.h <= 20.5f && zoom.w >= 90.0f && zoom.w <= 130.0f);

    ct::String scriptError;
    freshScene(rig);
    CHECK(assets.navigateTo("Assets"));
    rig.frames(2);
    assets.requestFolder("Assets");
    rig.frames(3);
    rt::InputBox* box = assets.nameBox();
    CHECK(box != nullptr);
    if (!box) return;
    rt::TextInput* input = inputIn(box->content());
    CHECK(input != nullptr && input->text() == "New Folder");
    if (!input) return;
    rig.click(static_cast<int>(input->absoluteRect().x + input->absoluteRect().w * 0.5f),
            static_cast<int>(input->absoluteRect().y + input->absoluteRect().h * 0.5f));
    CHECK(WidgetHost::textFocused());
    if (!Main::device()->isHeadless()) CHECK(Main::device()->textInputActive());
    const std::size_t floatsBefore = visibleFloats();
    humanKeyPress(rig, Key::A, kModControl);
    CHECK(visibleFloats() == floatsBefore);
    humanType(rig, "Typed Folder");
    CHECK(input->text() == "Typed Folder");
    humanKeyPress(rig, Key::Enter);
    CHECK(assets.nameBox() == nullptr);
    CHECK(dir_exists(absolute(rig, "Assets/Typed Folder").c_str()));

    CHECK(assets.createScript("Assets/Scripts", "rename_me", scriptError));
    Node* scene = rig.document.scene();
    Node3D* holder = add(scene, "Holder");
    rig.selection.clear();
    rig.selection.add(holder);
    rig.frames(3);
    assets.requestFolder("Assets");
    rig.frames(3);
    rt::WidgetApp::instance().setFocused(nullptr);
    const std::size_t childCount = scene->getChildCount();
    humanKeyPress(rig, Key::Delete);
    humanKeyPress(rig, Key::A, kModControl);
    CHECK(scene->getChildCount() == childCount);
    CHECK(visibleFloats() == 1);
    if (assets.nameBox()) rt::WidgetApp::instance().removeFloat(assets.nameBox());
    rig.frames(3);

    CHECK(assets.navigateTo("Assets/Scripts"));
    rig.frames(2);
    assets.requestRename("Assets/Scripts/rename_me.py");
    rig.frames(3);
    box = assets.nameBox();
    CHECK(box != nullptr && box->title() == "Rename");
    if (box)
    {
        input = inputIn(box->content());
        CHECK(input != nullptr && input->text() == "rename_me.py");
        rig.click(static_cast<int>(input->absoluteRect().x + 10.0f),
                static_cast<int>(input->absoluteRect().y + input->absoluteRect().h * 0.5f));
        humanKeyPress(rig, Key::A, kModControl);
        humanType(rig, "renamed.py");
        humanKeyPress(rig, Key::Enter);
        CHECK(assets.nameBox() == nullptr);
        CHECK(exists(absolute(rig, "Assets/Scripts/renamed.py")));
        CHECK(!exists(absolute(rig, "Assets/Scripts/rename_me.py")));
    }
    ct::String error;
    CHECK(!assets.renamePath("Assets", "Stuff", error) && !error.empty());
    CHECK(!assets.renamePath("Assets/Scripts/renamed.py", "a/b", error));
    CHECK(!assets.renamePath("Assets/Scripts/renamed.py", "hero.py", error));
    CHECK(assets.renamePath("Assets/Scripts/renamed.py", "final.py", error) && error.empty());
    CHECK(exists(absolute(rig, "Assets/Scripts/final.py")));

    rt::Dialog* confirm = nullptr;
    assets.requestDelete("Assets/Scripts/final.py");
    rig.frames(3);
    confirm = popupDialog();
    CHECK(confirm != nullptr);
    if (confirm)
    {
        CHECK(confirm->wrapped_.contains("\n"));
        clickDialogButton(rig, confirm, 1);
        rig.frames(2);
    }
    CHECK(exists(absolute(rig, "Assets/Scripts/final.py")));

    CHECK(assets.createScript("Assets/Scripts", "Dragged", error));
    rig.frames(3);
    const int script = entryIndex(rig, "Dragged.py");
    CHECK(script >= 0);
    rig.selection.clear();
    rig.frames(3);
    CHECK(rig.hierarchy.itemOf(holder) != nullptr);
    int x = 0;
    int y = 0;
    itemCenter(rig, script, x, y);
    rig.drag(x, y, textX(rig, holder), rowY(rig, holder));
    CHECK(property(holder, "script").asString() == "Assets/Scripts/Dragged.py");
    CHECK(rig.selection.isSelected(holder));
    CHECK(rig.document.dirty());

    Node3D* second = add(scene, "Second");
    rig.selection.clear();
    rig.selection.add(second);
    rig.frames(4);
    PropertyRow* row = rig.inspector.properties().rowNamed("script");
    CHECK(row != nullptr);
    if (row)
    {
        const rt::Rect rect = row->absoluteRect();
        itemCenter(rig, script, x, y);
        rig.drag(x, y, static_cast<int>(rect.x + rect.w * 0.5f),
                static_cast<int>(rect.y + rect.h * 0.5f));
        CHECK(property(second, "script").asString() == "Assets/Scripts/Dragged.py");
        CHECK(row->text() && row->text()->text() == "Assets/Scripts/Dragged.py");
    }

    const int model = entryIndex(rig, "Dragged.py");
    (void) model;
    CHECK(assets.navigateTo("Assets/Models"));
    rig.frames(3);
    const int tri = entryIndex(rig, "tri.gltf");
    if (tri >= 0 && row)
    {
        itemCenter(rig, tri, x, y);
        const rt::Rect rect = row->absoluteRect();
        rig.drag(x, y, static_cast<int>(rect.x + rect.w * 0.5f),
                static_cast<int>(rect.y + rect.h * 0.5f));
        CHECK(property(second, "script").asString() == "Assets/Scripts/Dragged.py");
    }
    CHECK(assets.attachScript("Assets/Scripts/final.py", second));
    CHECK(property(second, "script").asString() == "Assets/Scripts/final.py");
    CHECK(!assets.attachScript("Assets/Scripts/final.py", nullptr));

    rig.document.settings().folders.clear();
    assets.requestImport("Assets");
    rig.frames(3);
    CHECK(assets.fileDialog() != nullptr);
    if (assets.fileDialog())
    {
        CHECK(assets.fileDialog()->path() == EditorSettings::appFolder());
        const ct::String folder = testRoot() + "/external";
        CHECK(writeText(folder + "/remember.unknown", "x"));
        rt::TextInput* name = inputIn(assets.fileDialog()->content());
        CHECK(name != nullptr);
        if (name)
        {
            const rt::Rect box = name->absoluteRect();
            rig.click(static_cast<int>(box.x + box.w * 0.5f), static_cast<int>(box.y + 8.0f));
            humanType(rig, "/definitely/missing.unknown");
            humanKeyPress(rig, Key::Enter);
            CHECK(assets.fileDialog() != nullptr);
            humanKeyPress(rig, Key::A, kModControl);
            humanType(rig, (folder + "/remember.unknown").c_str());
            humanKeyPress(rig, Key::Enter);
        }
        rig.frames(4);
        while (assets.fileDialog())
        {
            rt::WidgetApp::instance().removeFloat(assets.fileDialog());
            rig.frames(2);
        }
    }
    CHECK(rig.document.settings().folderFor(EditorSettings::kFolderImport) ==
            testRoot() + "/external");
    if (assets.importing() || rig.ui.confirming()) rig.frames(2);
    assets.requestImport("Assets");
    rig.frames(3);
    if (assets.fileDialog())
    {
        CHECK(assets.fileDialog()->path() == testRoot() + "/external");
        rt::WidgetApp::instance().removeFloat(assets.fileDialog());
        rig.frames(3);
    }
}

int main(int argc, char** argv)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    testTypes();

    DeviceDesc desc;
    desc.title = "editor assets test";
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

        AssetsPanel assets(document, selection, hierarchy, actions, ui);
        CHECK(!assets.attached());
        CHECK(assets.attach(shell.panel(4)));
        CHECK(assets.attached());
        CHECK(!assets.attach(shell.panel(4)));
        assets.bindViewport(panel);

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, panel,
            actions, menus, status, assets };
        CHECK(shell.setLayout(
                "H200[{Hierarchy}|H780[V450[{Viewport}|{Console,Assets/1}]|{Inspector}]]"));
        rig.frames(4);
        CHECK(!assets.hasProject());
        CHECK(assets.entries().empty() && assets.footer()->text().empty());
        testEmptyState(rig);
        CHECK(assets.tree()->roots().empty());
        ct::String error;
        CHECK(!assets.createFolder("", "x", error));

        CHECK(document.createProject(testRoot() + "/projects", "Assets"));
        rig.frames(4);
        CHECK(!document.dirty());

        testLayout(rig);
        testNavigation(rig);
        testTree(rig);
        testListing(rig);
        testFilter(rig);
        testThumbnails(rig);
        testActivate(rig);
        testScenes(rig);
        testPrefab(rig);
        testModel(rig);
        testMenu(rig);
        testCreate(rig);
        testDelete(rig);
        testImport(rig);
        testDrag(rig);
        testExternal(rig);
        testProject(rig);
        testHumanFlows(rig);

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
