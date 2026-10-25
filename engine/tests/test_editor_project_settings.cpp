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
#include "prisma3d/editor/GamePanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/InspectorPanel.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/ProjectSettingsEditor.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/SceneTreePanel.h"
#include "prisma3d/editor/SectionedInspector.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/Theme.hpp>
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
const float kEpsilon = 1.0e-4f;
const char* const kDirectionalSize = "rendering/lights_and_shadows/directional_shadow/size";
const char* const kDirectionalQuality =
        "rendering/lights_and_shadows/directional_shadow/soft_shadow_filter_quality";
const char* const kMsaa = "rendering/anti_aliasing/quality/msaa_3d";
const char* const kScale = "rendering/scaling_3d/scale";
const char* const kViewportWidth = "display/window/size/viewport_width";

bool useVulkan = false;

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += useVulkan ? "/editor_project_settings_test_vulkan" : "/editor_project_settings_test";
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
    EditorViewportPanel& viewport;
    GamePanel& game;
    HierarchyActions& actions;
    EditorMenus& menus;
    ProjectSettingsEditor& settings;
    RenderingServer& server;
    int sceneCounter = 0;

    bool send(const InputEvent& event)
    {
        if (ui.handleEvent(event)) return true;
        if (actions.handleEvent(event)) return true;
        host.handleEvent(event);
        return false;
    }

    void frame(float delta = 1.0f / 60.0f)
    {
        tree.process(delta);
        viewport.update(delta);
        server.renderViewports();
        selection.update();
        selection.flush();
        hierarchy.update();
        inspector.update();
        ui.update();
        actions.update();
        menus.update();
        settings.update(delta);
        host.frame(delta, true);
        server.endFrame(true);
    }

    void frames(int count, float delta = 1.0f / 60.0f)
    {
        for (int i = 0; i < count; ++i) frame(delta);
    }

    void settle()
    {
        for (int i = 0; i < 4; ++i) frame(0.1f);
    }

    void tap(Key which, unsigned modifiers = 0)
    {
        send(key(which, true, modifiers));
        frames(2);
        send(key(which, false, modifiers));
        frames(2);
    }

    void typeText(const char* text)
    {
        for (const char* c = text; *c; ++c) send(character(*c));
        frames(2);
    }

    void click(int x, int y)
    {
        send(motion(x, y));
        frames(2);
        send(button(x, y, true));
        frames(2);
        send(button(x, y, false));
        frames(3);
    }
};

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

bool near(float a, float b)
{
    return fabsf(a - b) <= kEpsilon * (1.0f + fabsf(a) + fabsf(b));
}

ct::String manifest(const char* name)
{
    return testRoot() + "/projects/" + name + "/projecto.json";
}

ct::String readFile(const ct::String& path)
{
    return FileSystem::getSingleton().readText(path);
}

bool contains(const ct::String& text, const char* fragment)
{
    return strstr(text.c_str(), fragment) != nullptr;
}

SectionedInspector* sectioned(Rig& rig)
{
    ProjectSettingsWindow* window = rig.settings.window();
    return window ? window->inspector() : nullptr;
}

void reveal(Rig& rig, const rt::Widget* widget)
{
    SectionedInspector* inspector = sectioned(rig);
    if (!inspector || !widget) return;
    rt::ScrollView* scroll = inspector->scrollView();
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

void clickWidget(Rig& rig, const rt::Widget* widget)
{
    reveal(rig, widget);
    rig.click(centerX(widget), centerY(widget));
}

void typeInto(Rig& rig, NumberField* field, const char* text)
{
    clickWidget(rig, field);
    CHECK(field->isEditing());
    rig.typeText(text);
    rig.tap(Key::Enter);
}

void pickChoice(Rig& rig, PropertyRow* row, int index)
{
    clickWidget(rig, row->combo());
    rt::Widget* popup = rt::WidgetApp::instance().popup();
    CHECK(popup != nullptr);
    if (!popup) return;
    const rt::Rect rect = popup->absoluteRect();
    const float height = rt::Theme::instance().inputHeight;
    rig.click(static_cast<int>(rect.x + rect.w * 0.5f),
            static_cast<int>(rect.y + height * (static_cast<float>(index) + 0.5f)));
    CHECK(rt::WidgetApp::instance().popup() == nullptr);
}

bool openBarMenu(Rig& rig, rt::Menu* menu)
{
    const rt::Rect barRect = rig.shell.menuBar()->absoluteRect();
    const int by = static_cast<int>(barRect.y + barRect.h * 0.5f);
    for (int x = static_cast<int>(barRect.x) + 4; x < 500; x += 4)
    {
        rig.send(motion(x, by));
        rig.frames(2);
        rig.send(button(x, by, true));
        rig.frames(2);
        rig.send(button(x, by, false));
        rig.frames(2);
        if (rt::WidgetApp::instance().popup() == menu) return true;
    }
    return false;
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
    rig.send(button(px, py, true));
    rig.frames(2);
    rig.send(button(px, py, false));
    rig.frames(3);
}

void clickSection(Rig& rig, const char* path)
{
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector != nullptr);
    if (!inspector) return;
    rt::TreeNode* item = inspector->sectionItem(path);
    CHECK(item != nullptr);
    if (!item) return;
    const ct::Vector<rt::TreeNode*> rows = inspector->sections()->rows();
    int index = -1;
    for (std::size_t i = 0; i < rows.size(); ++i)
        if (rows[i] == item) index = static_cast<int>(i);
    CHECK(index >= 0);
    if (index < 0) return;
    const rt::Rect area = inspector->sections()->absoluteRect();
    rig.click(static_cast<int>(area.x) + 60,
            static_cast<int>(
                    area.y + (static_cast<float>(index) + 0.5f) * ActivatableTree::kRowHeight));
}

void openWindow(Rig& rig)
{
    rig.settings.open();
    rig.frames(4);
    CHECK(rig.settings.isOpen());
}

void closeWindow(Rig& rig)
{
    rig.settings.close();
    rig.frames(3);
    CHECK(!rig.settings.isOpen());
}

void pickSection(Rig& rig, const char* path)
{
    clickSection(rig, path);
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector && inspector->currentSection() == path);
    rig.frames(2);
}

void testMenuAndWindow(Rig& rig)
{
    rt::Menu* file = rig.menus.menu(EditorMenus::kFile);
    CHECK(file->actionCount() == EditorMenus::kFileItemCount);
    rt::MenuAction* entry = rig.menus.action(EditorMenus::kFile, EditorMenus::kFileSettings);
    CHECK(entry && entry->text() == "Project Settings...");
    CHECK(entry && entry->isEnabled());
    CHECK(!rig.settings.isOpen());

    CHECK(openBarMenu(rig, file));
    clickMenuItem(rig, file, EditorMenus::kFileSettings);
    CHECK(rig.settings.isOpen());
    ProjectSettingsWindow* window = rig.settings.window();
    CHECK(window != nullptr && window->title() == ProjectSettingsWindow::kTitle);
    if (!window) return;
    CHECK(window->searchBox()->placeholder() == ProjectSettingsWindow::kSearchPlaceholder);
    CHECK(!window->restartShown());
    const rt::Rect rect = window->absoluteRect();
    CHECK(rect.w > 400.0f && rect.h > 300.0f);
    CHECK(rect.x >= 0.0f && rect.y >= 0.0f && rect.x + rect.w <= static_cast<float>(kWidth) &&
            rect.y + rect.h <= static_cast<float>(kHeight));

    rig.settings.open();
    CHECK(rig.settings.window() == window);

    rig.tap(Key::Escape);
    CHECK(!rig.settings.isOpen());

    rig.settings.open();
    rig.frames(3);
    CHECK(rig.settings.isOpen());
    clickWidget(rig, rig.settings.window()->closeButton());
    CHECK(!rig.settings.isOpen());

    rig.settings.open();
    rig.frames(3);
    ProjectSettingsWindow* again = rig.settings.window();
    CHECK(again != nullptr);
    const rt::Rect box = again->absoluteRect();
    rig.click(static_cast<int>(box.x + box.w - 14.0f), static_cast<int>(box.y + 12.0f));
    CHECK(!rig.settings.isOpen());
}

void testSectionTree(Rig& rig)
{
    openWindow(rig);
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector != nullptr);
    if (!inspector) return;

    const char* const expected[] = { "application", "application/run", "display", "display/window",
        "physics", "physics/common", "physics/3d", "rendering", "rendering/anti_aliasing",
        "rendering/scaling_3d", "rendering/mesh_lod", "rendering/lights_and_shadows",
        "rendering/environment", "rendering/camera", "rendering/lightmapping", "export" };
    for (const char* path: expected) CHECK(inspector->sectionItem(path) != nullptr);
    CHECK(inspector->sectionPaths().size() == sizeof(expected) / sizeof(expected[0]));

    const char* const parents[] = { "application", "display", "physics", "rendering" };
    for (const char* path: parents)
    {
        CHECK(!inspector->sectionSelectable(path));
        CHECK(inspector->sectionItem(path)->hasChildren());
    }
    CHECK(inspector->sectionItem("rendering/lights_and_shadows")->parent() ==
            inspector->sectionItem("rendering"));
    CHECK(inspector->sectionItem("rendering/lights_and_shadows")->text() == "Lights and Shadows");
    CHECK(inspector->sectionItem("rendering/anti_aliasing")->text() == "Anti Aliasing");
    CHECK(inspector->sectionItem("rendering/scaling_3d")->text() == "Scaling 3D");
    CHECK(inspector->sectionItem("rendering/mesh_lod")->text() == "Mesh LOD");
    CHECK(inspector->sectionItem("rendering")->text() == "Rendering");
    for (std::size_t i = 4; i < sizeof(expected) / sizeof(expected[0]); ++i)
    {
        const ct::String path(expected[i]);
        if (path.find('/') != ct::String::npos) CHECK(inspector->sectionSelectable(path));
    }

    CHECK(inspector->currentSection() == "application/run");
    CHECK(inspector->rowNamed("application/run/max_fps") != nullptr);
    CHECK(inspector->rowNamed(kDirectionalSize) == nullptr);

    clickSection(rig, "rendering");
    CHECK(inspector->currentSection() == "application/run");

    pickSection(rig, "rendering/lights_and_shadows");
    CHECK(inspector->rowNamed("application/run/max_fps") == nullptr);
    PropertyRow* size = inspector->rowNamed(kDirectionalSize);
    CHECK(size != nullptr && size->label() == "Size");
    CHECK(size && size->info().name == kDirectionalSize);
    rt::Collapsible* directional = inspector->group("directional_shadow");
    rt::Collapsible* positional = inspector->group("positional_shadow");
    CHECK(directional && directional->title() == "Directional Shadow");
    CHECK(positional && positional->title() == "Positional Shadow");
    CHECK(size && size->parent() == directional);
    CHECK(inspector->rowNamed(kDirectionalQuality) != nullptr);
    CHECK(inspector->rowNamed("rendering/lights_and_shadows/positional_shadow/atlas_size") !=
            nullptr);
    CHECK(inspector->rowNamed("rendering/lights_and_shadows/positional_shadow/"
                              "atlas_quadrant_3_subdiv") != nullptr);
    CHECK(inspector->rowNamed(kScale) == nullptr);
    CHECK(inspector->rowCount() == 8);

    pickSection(rig, "rendering/anti_aliasing");
    CHECK(inspector->rowNamed(kMsaa) != nullptr);
    CHECK(inspector->groupCount() == 2);
    CHECK(inspector->group("quality") != nullptr);
    CHECK(inspector->group("screen_space_roughness_limiter") != nullptr);
    PropertyRow* msaa = inspector->rowNamed(kMsaa);
    CHECK(msaa && msaa->kind() == PropertyRow::Kind::Enum && msaa->combo()->itemCount() == 4);
    CHECK(msaa && msaa->label() == "MSAA 3D");
    PropertyRow* limiter =
            inspector->rowNamed("rendering/anti_aliasing/screen_space_roughness_limiter/enabled");
    CHECK(limiter && limiter->kind() == PropertyRow::Kind::Bool);

    pickSection(rig, "rendering/scaling_3d");
    PropertyRow* scale = inspector->rowNamed(kScale);
    CHECK(scale && scale->kind() == PropertyRow::Kind::Float && scale->field(0)->ranged());
    CHECK(scale && near(static_cast<float>(scale->field(0)->rangeMin()), 0.25f));
    CHECK(scale && near(static_cast<float>(scale->field(0)->rangeMax()), 2.0f));
    CHECK(scale && !scale->revertible());

    pickSection(rig, "display/window");
    PropertyRow* position = inspector->rowNamed("display/window/size/initial_position_type");
    CHECK(position && position->kind() == PropertyRow::Kind::Enum && position->choiceCount() == 4);
    CHECK(position && position->choiceValue(0) == 1 && position->choiceValue(1) == 3 &&
            position->choiceValue(3) == 5);
    PropertyRow* width = inspector->rowNamed(kViewportWidth);
    CHECK(width && width->kind() == PropertyRow::Kind::Int);
    CHECK(width && width->info().rangeOrGreater &&
            near(static_cast<float>(width->info().rangeMin), 1.0f));
    CHECK(inspector->rowNamed("display/window/vsync/vsync_mode")->choiceCount() == 3);
    CHECK(inspector->group("size") != nullptr && inspector->group("vsync") != nullptr);

    pickSection(rig, "physics/3d");
    PropertyRow* gravity = inspector->rowNamed("physics/3d/default_gravity");
    CHECK(gravity && gravity->info().rangeOrGreater && gravity->info().rangeOrLess);
    PropertyRow* direction = inspector->rowNamed("physics/3d/default_gravity_vector");
    CHECK(direction && direction->kind() == PropertyRow::Kind::Vec3);

    pickSection(rig, "rendering/environment");
    PropertyRow* clear = inspector->rowNamed(ProjectSettingsEditor::kClearColorSetting);
    CHECK(clear && clear->kind() == PropertyRow::Kind::Color);

    for (std::size_t i = 0; i < Project::settingCount(); ++i)
    {
        const ct::String& name = Project::settingInfo(i).name;
        const PropertyInfo info = ProjectSettingsEditor::describe(Project::settingInfo(i));
        CHECK(info.name == name && PropertyRow::kindOf(info) != PropertyRow::Kind::None);
    }
    CHECK(ProjectSettingsEditor::properties().size() == Project::settingCount());

    closeWindow(rig);
}

void testSearch(Rig& rig)
{
    openWindow(rig);
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector != nullptr);
    if (!inspector) return;
    const std::size_t all = inspector->sectionPaths().size();

    CHECK(SectionedInspector::pathMatches("rendering/lights_and_shadows/directional_shadow/size",
            "SHADOW"));
    CHECK(SectionedInspector::pathMatches("rendering/anti_aliasing/quality/msaa_3d", "msaa"));
    CHECK(SectionedInspector::pathMatches("rendering/scaling_3d/scale", "sclng"));
    CHECK(!SectionedInspector::pathMatches("rendering/scaling_3d/scale", "zzz"));

    clickWidget(rig, rig.settings.window()->searchBox());
    rig.typeText("shadow");
    CHECK(rig.settings.window()->searchBox()->text() == "shadow");
    CHECK(inspector->filter() == "shadow");
    CHECK(inspector->sectionPaths().size() == 2);
    CHECK(inspector->sectionItem("rendering") != nullptr &&
            inspector->sectionItem("rendering/lights_and_shadows") != nullptr);
    CHECK(inspector->sectionItem("rendering/scaling_3d") == nullptr &&
            inspector->sectionItem("display") == nullptr);
    CHECK(inspector->currentSection() == "rendering/lights_and_shadows");
    CHECK(inspector->rowCount() == 8);
    CHECK(inspector->rowNamed(kDirectionalSize) != nullptr);

    rig.settings.window()->searchBox()->selectAll();
    rig.typeText("msaa");
    CHECK(inspector->sectionPaths().size() == 2);
    CHECK(inspector->sectionItem("rendering/anti_aliasing") != nullptr);
    CHECK(inspector->currentSection() == "rendering/anti_aliasing");
    CHECK(inspector->rowCount() == 1 && inspector->rowNamed(kMsaa) != nullptr);

    rig.settings.window()->searchBox()->selectAll();
    rig.typeText("zzzz");
    CHECK(inspector->sectionPaths().empty() && inspector->rowCount() == 0);
    CHECK(inspector->currentSection().empty());

    rig.settings.window()->searchBox()->selectAll();
    rig.typeText("gravity");
    CHECK(inspector->currentSection() == "physics/3d");
    CHECK(inspector->rowCount() == 2);

    rig.settings.window()->searchBox()->selectAll();
    rig.typeText("size");
    CHECK(inspector->sectionItem("display/window") != nullptr &&
            inspector->sectionItem("rendering/lights_and_shadows") != nullptr);

    rig.settings.window()->searchBox()->selectAll();
    rig.tap(Key::Backspace);
    CHECK(rig.settings.window()->searchBox()->text().empty());
    CHECK(inspector->filter().empty());
    CHECK(inspector->sectionPaths().size() == all);
    CHECK(inspector->currentSection() == "display/window" ||
            inspector->currentSection() == "rendering/lights_and_shadows");
    closeWindow(rig);
}

void testEditAndRevert(Rig& rig)
{
    Project& project = rig.document.project();
    CHECK(project.isDefault(kDirectionalSize));
    openWindow(rig);
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector != nullptr);
    if (!inspector) return;
    pickSection(rig, "rendering/lights_and_shadows");

    PropertyRow* size = inspector->rowNamed(kDirectionalSize);
    CHECK(size && size->kind() == PropertyRow::Kind::Int && size->field(0)->ranged());
    if (!size) return;
    CHECK(near(size->field(0)->value(), 4096.0f) && !size->revertible());
    const unsigned int saves = rig.settings.saveCount();
    const unsigned int applies = rig.settings.applyCount();

    typeInto(rig, size->field(0), "2048");
    CHECK(project.get(kDirectionalSize).asInt() == 2048);
    CHECK(!project.isDefault(kDirectionalSize));
    CHECK(size->revertible() && near(size->field(0)->value(), 2048.0f));
    CHECK(rig.server.directionalShadowSize() == 2048);
    CHECK(rig.settings.applyCount() > applies);
    CHECK(rig.settings.savePending() && rig.settings.saveCount() == saves);
    CHECK(!contains(readFile(manifest("Settings")), "directional_shadow/size"));

    CHECK(rig.settings.flush());
    CHECK(!rig.settings.savePending() && rig.settings.saveCount() == saves + 1);
    CHECK(contains(readFile(manifest("Settings")), "\"rendering/lights_and_shadows/"
                                                   "directional_shadow/size\": 2048"));
    Project reloaded;
    CHECK(reloaded.open(manifest("Settings")));
    CHECK(reloaded.get(kDirectionalSize).asInt() == 2048);
    CHECK(reloaded.isDefault(kDirectionalQuality));

    PropertyRow* quality = inspector->rowNamed(kDirectionalQuality);
    CHECK(quality && quality->kind() == PropertyRow::Kind::Enum &&
            quality->combo()->itemCount() == 6);
    CHECK(quality && quality->combo()->itemText(0) == "Hard (Fastest)");
    if (!quality) return;
    CHECK(rig.server.directionalShadowQuality() == ShadowQuality::SoftLow);
    pickChoice(rig, quality, 0);
    CHECK(project.get(kDirectionalQuality).asInt() == 0);
    CHECK(rig.server.directionalShadowQuality() == ShadowQuality::Hard);
    CHECK(quality->revertible());
    pickChoice(rig, quality, 4);
    CHECK(rig.server.directionalShadowQuality() == ShadowQuality::SoftHigh);

    PropertyRow* positional = inspector->rowNamed(
            "rendering/lights_and_shadows/positional_shadow/soft_shadow_filter_quality");
    CHECK(positional != nullptr);
    if (positional)
    {
        pickChoice(rig, positional, 5);
        CHECK(rig.server.positionalShadowQuality() == ShadowQuality::SoftUltra);
        clickWidget(rig, positional->revertButton());
        CHECK(rig.server.positionalShadowQuality() == ShadowQuality::SoftLow);
        CHECK(!positional->revertible());
    }

    clickWidget(rig, size->revertButton());
    CHECK(project.get(kDirectionalSize).asInt() == 4096 && project.isDefault(kDirectionalSize));
    CHECK(near(size->field(0)->value(), 4096.0f) && !size->revertible());
    CHECK(rig.server.directionalShadowSize() == 4096);
    clickWidget(rig, quality->revertButton());
    CHECK(project.isDefault(kDirectionalQuality));
    CHECK(rig.server.directionalShadowQuality() == ShadowQuality::SoftLow);
    CHECK(quality->combo()->selectedIndex() == 2 && !quality->revertible());

    CHECK(rig.settings.savePending());
    rig.frames(10, 0.2f);
    CHECK(!rig.settings.savePending() && rig.settings.saveCount() >= saves + 2);
    CHECK(!contains(readFile(manifest("Settings")), "directional_shadow/size"));

    typeInto(rig, size->field(0), "1000000");
    CHECK(project.get(kDirectionalSize).asInt() == 16384);
    typeInto(rig, size->field(0), "10");
    CHECK(project.get(kDirectionalSize).asInt() == 256);
    clickWidget(rig, size->revertButton());
    CHECK(project.isDefault(kDirectionalSize));

    closeWindow(rig);
    CHECK(!rig.settings.savePending());
}

void testLiveApply(Rig& rig)
{
    Project& project = rig.document.project();
    const ViewportId sceneId = rig.viewport.viewportId();
    const ViewportId gameId = rig.game.viewportId();
    CHECK(sceneId.valid() && gameId.valid());
    CHECK(rig.server.viewportGetMsaa3D(sceneId) == ViewportMsaa::Disabled);
    CHECK(rig.server.viewportGetMsaa3D(gameId) == ViewportMsaa::Disabled);

    openWindow(rig);
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector != nullptr);
    if (!inspector) return;

    pickSection(rig, "rendering/anti_aliasing");
    PropertyRow* msaa = inspector->rowNamed(kMsaa);
    CHECK(msaa != nullptr);
    if (msaa)
    {
        pickChoice(rig, msaa, 2);
        CHECK(project.get(kMsaa).asInt() == 2);
        CHECK(rig.server.viewportGetMsaa3D(sceneId) == ViewportMsaa::Msaa4x);
        CHECK(rig.server.viewportGetMsaa3D(gameId) == ViewportMsaa::Msaa4x);
        clickWidget(rig, msaa->revertButton());
        CHECK(rig.server.viewportGetMsaa3D(sceneId) == ViewportMsaa::Disabled);
        CHECK(rig.server.viewportGetMsaa3D(gameId) == ViewportMsaa::Disabled);
    }
    PropertyRow* limiter =
            inspector->rowNamed("rendering/anti_aliasing/screen_space_roughness_limiter/amount");
    CHECK(limiter != nullptr);
    if (limiter)
    {
        typeInto(rig, limiter->field(0), "1.5");
        CHECK(near(rig.server.roughnessLimiterAmount(), 1.5f));
        clickWidget(rig, limiter->revertButton());
        CHECK(near(rig.server.roughnessLimiterAmount(), 0.25f));
    }

    pickSection(rig, "rendering/scaling_3d");
    PropertyRow* scale = inspector->rowNamed(kScale);
    CHECK(scale != nullptr);
    if (scale)
    {
        CHECK(near(rig.server.viewportGetScaling3D(sceneId), 1.0f));
        typeInto(rig, scale->field(0), "0.5");
        CHECK(near(static_cast<float>(project.get(kScale).asFloat()), 0.5f));
        CHECK(near(rig.server.viewportGetScaling3D(sceneId), 0.5f));
        CHECK(near(rig.server.viewportGetScaling3D(gameId), 0.5f));
        typeInto(rig, scale->field(0), "9");
        CHECK(near(rig.server.viewportGetScaling3D(sceneId), 2.0f));
        clickWidget(rig, scale->revertButton());
        CHECK(near(rig.server.viewportGetScaling3D(sceneId), 1.0f));
        CHECK(near(rig.server.viewportGetScaling3D(gameId), 1.0f));
    }

    pickSection(rig, "rendering/mesh_lod");
    PropertyRow* lod = inspector->rowNamed("rendering/mesh_lod/lod_change/threshold_pixels");
    CHECK(lod != nullptr);
    if (lod)
    {
        typeInto(rig, lod->field(0), "8");
        CHECK(near(rig.server.viewportGetMeshLodThreshold(sceneId), 8.0f));
        CHECK(near(rig.server.viewportGetMeshLodThreshold(gameId), 8.0f));
        clickWidget(rig, lod->revertButton());
        CHECK(near(rig.server.viewportGetMeshLodThreshold(sceneId), 1.0f));
    }

    CHECK(!rig.settings.window()->restartShown());
    closeWindow(rig);

    project.set(kScale, PropertyValue::fromFloat(0.75));
    project.set(kMsaa, PropertyValue::fromInt(3));
    rig.settings.applyLive();
    CHECK(near(rig.server.viewportGetScaling3D(gameId), 0.75f));
    CHECK(rig.server.viewportGetMsaa3D(sceneId) == ViewportMsaa::Msaa8x);
    project.reset(kScale);
    project.reset(kMsaa);
    rig.settings.applyLive();
    CHECK(near(rig.server.viewportGetScaling3D(sceneId), 1.0f));
    CHECK(rig.server.viewportGetMsaa3D(gameId) == ViewportMsaa::Disabled);
    rig.settings.flush();
}

void testRestartNotice(Rig& rig)
{
    Project& project = rig.document.project();
    openWindow(rig);
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector != nullptr);
    if (!inspector) return;
    CHECK(ProjectSettingsEditor::isLive(kMsaa) && !ProjectSettingsEditor::isLive(kViewportWidth));

    pickSection(rig, "display/window");
    CHECK(!rig.settings.window()->restartShown());
    PropertyRow* width = inspector->rowNamed(kViewportWidth);
    CHECK(width != nullptr);
    if (!width) return;
    typeInto(rig, width->field(0), "800");
    CHECK(project.get(kViewportWidth).asInt() == 800);
    CHECK(rig.settings.window()->restartShown());
    CHECK(rig.settings.window()->restartLabel()->text() == ProjectSettingsWindow::kRestartText);
    CHECK(rig.settings.window()->restartBar()->isVisible());
    rig.frames(3);

    clickWidget(rig, rig.settings.window()->restartCloseButton());
    CHECK(!rig.settings.window()->restartShown());

    PropertyRow* mode = inspector->rowNamed("display/window/size/mode");
    CHECK(mode != nullptr);
    if (mode)
    {
        pickChoice(rig, mode, 3);
        CHECK(project.get("display/window/size/mode").asInt() == 3);
        CHECK(rig.settings.window()->restartShown());
        clickWidget(rig, mode->revertButton());
    }
    clickWidget(rig, rig.settings.window()->restartCloseButton());
    pickSection(rig, "rendering/lights_and_shadows");
    PropertyRow* size = inspector->rowNamed(kDirectionalSize);
    if (size)
    {
        typeInto(rig, size->field(0), "512");
        CHECK(!rig.settings.window()->restartShown());
        clickWidget(rig, size->revertButton());
    }
    pickSection(rig, "display/window");
    clickWidget(rig, inspector->rowNamed(kViewportWidth)->revertButton());
    CHECK(project.isDefault(kViewportWidth));

    CHECK(rig.settings.window()->searchBox() != nullptr);
    closeWindow(rig);
}

void testSavedWithProject(Rig& rig)
{
    Project& project = rig.document.project();
    CHECK(rig.document.createProject(testRoot() + "/projects", "Other"));
    rig.frames(3);
    CHECK(rig.document.project().name() == "Other");

    CHECK(rig.document.openProject(manifest("Settings")));
    rig.frames(3);
    openWindow(rig);
    SectionedInspector* inspector = sectioned(rig);
    CHECK(inspector != nullptr);
    if (!inspector) return;
    pickSection(rig, "rendering/scaling_3d");
    PropertyRow* scale = inspector->rowNamed(kScale);
    CHECK(scale != nullptr);
    if (!scale) return;
    typeInto(rig, scale->field(0), "0.8");
    CHECK(rig.settings.savePending());
    CHECK(!contains(readFile(manifest("Settings")), "scaling_3d/scale"));

    CHECK(rig.document.openProject(manifest("Other")));
    rig.frames(3);
    CHECK(!rig.settings.savePending());
    CHECK(contains(readFile(manifest("Settings")), "\"rendering/scaling_3d/scale\": 0.8"));
    CHECK(project.name() == "Other" && project.isDefault(kScale));
    rig.frames(2);
    CHECK(near(rig.server.viewportGetScaling3D(rig.viewport.viewportId()), 1.0f));
    CHECK(near(static_cast<float>(inspector->rowNamed(kScale)->value().asFloat()), 1.0f));
    CHECK(!inspector->rowNamed(kScale)->revertible());

    CHECK(rig.document.openProject(manifest("Settings")));
    rig.frames(3);
    CHECK(near(rig.server.viewportGetScaling3D(rig.viewport.viewportId()), 0.8f));
    CHECK(near(rig.server.viewportGetScaling3D(rig.game.viewportId()), 0.8f));
    CHECK(near(static_cast<float>(inspector->rowNamed(kScale)->value().asFloat()), 0.8f));
    CHECK(inspector->rowNamed(kScale)->revertible());
    clickWidget(rig, inspector->rowNamed(kScale)->revertButton());
    closeWindow(rig);
    CHECK(near(rig.server.viewportGetScaling3D(rig.viewport.viewportId()), 1.0f));
    CHECK(!contains(readFile(manifest("Settings")), "scaling_3d/scale"));

    rig.document.closeProject();
    rig.frames(3);
    CHECK(!rig.settings.canOpen());
    CHECK(!rig.menus.action(EditorMenus::kFile, EditorMenus::kFileSettings)->isEnabled());
    CHECK(rig.document.openProject(manifest("Settings")));
    rig.frames(3);
    CHECK(rig.menus.action(EditorMenus::kFile, EditorMenus::kFileSettings)->isEnabled());
}

Node* firstOf(Node* node, const char* type)
{
    if (node->isClass(type)) return node;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (Node* found = firstOf(node->getChild(i), type)) return found;
    return nullptr;
}

std::size_t countOf(Node* node, const char* type)
{
    std::size_t count = node->isClass(type) ? 1 : 0;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        count += countOf(node->getChild(i), type);
    return count;
}

void select(Rig& rig, Node* node)
{
    rig.selection.clear();
    if (node) rig.selection.add(node);
    rig.frames(3);
}

void revealInspector(Rig& rig, const rt::Widget* widget)
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

void clickGate(Rig& rig, rt::Collapsible* gate)
{
    revealInspector(rig, gate);
    const rt::Rect box = gate->checkRect();
    rig.click(static_cast<int>(box.x + box.w * 0.5f), static_cast<int>(box.y + box.h * 0.5f));
    rig.frames(3);
}

void testEnvironmentMenu(Rig& rig)
{
    CHECK(rig.document.newScene());
    rig.frames(3);
    Node* scene = rig.document.scene();
    CHECK(scene != nullptr);
    if (!scene) return;
    Node3D* a = scene->createChild<Node3D>();
    a->setName("A");
    rig.frames(2);
    CHECK(rig.document.saveSceneAs("environment"));
    rig.frames(3);
    CHECK(!rig.document.dirty());
    CHECK(rig.actions.findEnvironment() == nullptr);
    select(rig, a);

    rt::Menu* sceneMenu = rig.menus.menu(EditorMenus::kScene);
    CHECK(sceneMenu->actionCount() == EditorMenus::kSceneItemCount);
    rt::MenuAction* entry = rig.menus.action(EditorMenus::kScene, EditorMenus::kSceneEnvironment);
    CHECK(entry && entry->text() == "Environment..." && entry->isEnabled());

    const std::size_t undoBefore = rig.document.undoRedo().undoCount();
    CHECK(openBarMenu(rig, sceneMenu));
    clickMenuItem(rig, sceneMenu, EditorMenus::kSceneEnvironment);
    CHECK(rt::WidgetApp::instance().popup() == nullptr);
    Node* environment = firstOf(scene, "WorldEnvironment");
    CHECK(environment != nullptr && countOf(scene, "WorldEnvironment") == 1);
    if (!environment) return;
    CHECK(environment->getParent() == scene);
    CHECK(rig.selection.selectedNodes().size() == 1 && rig.selection.isSelected(environment));
    CHECK(rig.actions.findEnvironment() == environment);
    CHECK(rig.document.dirty());
    CHECK(rig.document.undoRedo().undoCount() == undoBefore + 1);
    CHECK(rig.document.undoRedo().currentActionName() == HierarchyActions::kAddAction);
    CHECK(rig.inspector.properties().node() == environment);

    const WorldEnvironment* world = environment->castTo<WorldEnvironment>();
    const EnvironmentSettings defaults;
    CHECK(world && world->getEnvironment().glowEnabled == defaults.glowEnabled &&
            !world->getEnvironment().fogEnabled);
    CHECK(world && near(world->getEnvironment().glowIntensity, 0.3f) &&
            world->getEnvironment().glowBlendMode == GlowBlendMode::Screen);
    CHECK(world && near(world->getEnvironment().glowLevels[1], 0.8f) &&
            near(world->getEnvironment().glowLevels[3], 0.1f));
    CHECK(world && near(world->getEnvironment().fogDensity, 0.01f));

    rig.tap(Key::Z, kModControl);
    CHECK(firstOf(scene, "WorldEnvironment") == nullptr);
    CHECK(rig.selection.isSelected(a) && rig.selection.selectedNodes().size() == 1);
    CHECK(!rig.document.dirty());
    rig.tap(Key::Y, kModControl);
    environment = firstOf(scene, "WorldEnvironment");
    CHECK(environment != nullptr && countOf(scene, "WorldEnvironment") == 1);
    CHECK(rig.selection.selectedNodes().size() == 1 && rig.selection.isSelected(environment));

    select(rig, a);
    CHECK(rig.selection.isSelected(a));
    const std::size_t undoAfter = rig.document.undoRedo().undoCount();
    CHECK(openBarMenu(rig, sceneMenu));
    clickMenuItem(rig, sceneMenu, EditorMenus::kSceneEnvironment);
    CHECK(countOf(scene, "WorldEnvironment") == 1);
    CHECK(rig.selection.selectedNodes().size() == 1 &&
            rig.selection.isSelected(firstOf(scene, "WorldEnvironment")));
    CHECK(rig.document.undoRedo().undoCount() == undoAfter);

    rig.document.undo();
    rig.frames(2);
    CHECK(rig.document.saveSceneAs("environment"));
    rig.frames(2);
}

void testEnvironmentGroups(Rig& rig)
{
    CHECK(rig.document.newScene());
    rig.frames(3);
    Node* scene = rig.document.scene();
    CHECK(scene != nullptr);
    if (!scene) return;
    CHECK(rig.document.saveSceneAs("groups"));
    rig.frames(2);
    rig.actions.requestEnvironment();
    rig.frames(3);
    Node* environment = firstOf(scene, "WorldEnvironment");
    CHECK(environment != nullptr);
    if (!environment) return;
    PropertyInspector& properties = rig.inspector.properties();
    CHECK(properties.node() == environment);

    const char* const titles[] = { "Background", "Ambient Light", "Reflected Light", "Tonemap",
        "Glow", "Fog" };
    for (const char* title: titles) CHECK(properties.section(title) != nullptr);
    CHECK(properties.section("Sky") == nullptr);

    rt::Collapsible* glow = properties.gate("glow_enabled");
    rt::Collapsible* fog = properties.gate("fog_enabled");
    CHECK(glow && glow == properties.section("Glow") && glow->isCheckable() && !glow->isChecked());
    CHECK(fog && fog == properties.section("Fog") && fog->isCheckable() && !fog->isChecked());
    rt::Collapsible* ssao = properties.gate("ssao_enabled");
    rt::Collapsible* adjustments = properties.gate("adjustment_enabled");
    CHECK(ssao && ssao == properties.section("SSAO") && ssao->isCheckable() && !ssao->isChecked());
    CHECK(adjustments && adjustments == properties.section("Adjustments") &&
            adjustments->isCheckable() && !adjustments->isChecked());
    rt::Collapsible* ssr = properties.gate("ssr_enabled");
    CHECK(ssr && ssr == properties.section("SSR") && ssr->isCheckable() && !ssr->isChecked());
    rt::Collapsible* dofFar = properties.gate("dof_blur_far_enabled");
    rt::Collapsible* dofNear = properties.gate("dof_blur_near_enabled");
    CHECK(dofFar && dofFar == properties.section("Depth of Field Far") && dofFar->isCheckable() &&
            !dofFar->isChecked());
    CHECK(dofNear && dofNear == properties.section("Depth of Field Near") &&
            dofNear->isCheckable() && !dofNear->isChecked());
    CHECK(properties.section("Depth of Field") != nullptr &&
            properties.rowNamed("dof_blur_amount") != nullptr);
    CHECK(properties.gateCount() == 7);
    CHECK(properties.rowNamed("glow_enabled") == nullptr &&
            properties.rowNamed("fog_enabled") == nullptr);
    CHECK(properties.rowNamed("glow_intensity") == nullptr &&
            properties.rowNamed("fog_density") == nullptr);
    CHECK(properties.rowNamed("tonemap_mode") != nullptr &&
            properties.rowNamed("background_mode") != nullptr);
    if (!glow || !fog) return;

    const std::size_t before = rig.document.undoRedo().undoCount();
    clickGate(rig, glow);
    glow = properties.gate("glow_enabled");
    CHECK(glow && glow->isChecked());
    PropertyValue value;
    CHECK(NodeClassDB::getProperty(environment, "glow_enabled", value) && value.asBool());
    CHECK(properties.rowNamed("glow_intensity") != nullptr);
    CHECK(properties.rowNamed("glow_strength") != nullptr &&
            properties.rowNamed("glow_bloom") != nullptr);
    CHECK(properties.rowNamed("glow_blend_mode") != nullptr &&
            properties.rowNamed("glow_hdr_threshold") != nullptr);
    CHECK(properties.rowNamed("glow_levels/2") != nullptr);
    CHECK(properties.rowNamed("fog_density") == nullptr);
    CHECK(rig.document.undoRedo().undoCount() == before + 1);
    CHECK(environment->castTo<WorldEnvironment>()->getEnvironment().glowEnabled);

    clickGate(rig, properties.gate("fog_enabled"));
    fog = properties.gate("fog_enabled");
    CHECK(fog && fog->isChecked());
    CHECK(properties.rowNamed("fog_density") != nullptr &&
            properties.rowNamed("fog_mode") != nullptr);
    CHECK(properties.rowNamed("fog_light_color") != nullptr);
    CHECK(environment->castTo<WorldEnvironment>()->getEnvironment().fogEnabled);

    clickGate(rig, properties.gate("glow_enabled"));
    CHECK(!properties.gate("glow_enabled")->isChecked());
    CHECK(properties.rowNamed("glow_intensity") == nullptr &&
            properties.rowNamed("glow_strength") == nullptr);
    CHECK(properties.rowNamed("fog_density") != nullptr);
    CHECK(!environment->castTo<WorldEnvironment>()->getEnvironment().glowEnabled);

    rig.tap(Key::Z, kModControl);
    CHECK(properties.gate("glow_enabled")->isChecked());
    CHECK(properties.rowNamed("glow_intensity") != nullptr);
    rig.tap(Key::Y, kModControl);
    CHECK(!properties.gate("glow_enabled")->isChecked());
    clickGate(rig, properties.gate("fog_enabled"));
    CHECK(!properties.gate("fog_enabled")->isChecked() &&
            properties.rowNamed("fog_density") == nullptr);

    environment->castTo<WorldEnvironment>()->setEnvironment(
            [&]
            {
                EnvironmentSettings settings =
                        environment->castTo<WorldEnvironment>()->getEnvironment();
                settings.glowEnabled = true;
                return settings;
            }());
    rig.frames(3);
    CHECK(properties.gate("glow_enabled")->isChecked() &&
            properties.rowNamed("glow_intensity") != nullptr);
}

void testLightShadowGate(Rig& rig)
{
    CHECK(rig.document.newScene());
    rig.frames(3);
    Node* scene = rig.document.scene();
    CHECK(scene != nullptr);
    if (!scene) return;
    CHECK(rig.document.saveSceneAs("lights"));
    rig.frames(2);
    PropertyInspector& properties = rig.inspector.properties();

    Node* sun = scene->createChild("DirectionalLight3D", true);
    Node* lamp = scene->createChild("OmniLight3D", true);
    CHECK(sun != nullptr && lamp != nullptr);
    if (!sun || !lamp) return;
    sun->setName("Sun");
    lamp->setName("Lamp");
    rig.frames(2);

    select(rig, sun);
    CHECK(properties.node() == sun);
    rt::Collapsible* gate = properties.gate("shadow_enabled");
    CHECK(gate && gate == properties.section("Shadow") && gate->isCheckable() &&
            !gate->isChecked());
    CHECK(properties.rowNamed("shadow_enabled") == nullptr);
    CHECK(properties.rowNamed("shadow_bias") == nullptr &&
            properties.rowNamed("shadow_blur") == nullptr);
    CHECK(properties.section("Directional Shadow") == nullptr);
    CHECK(properties.rowNamed("directional_shadow_mode") == nullptr);
    CHECK(properties.rowNamed("directional_shadow_max_distance") == nullptr);
    CHECK(properties.rowNamed("light_energy") != nullptr);
    if (!gate) return;

    const std::size_t before = rig.document.undoRedo().undoCount();
    clickGate(rig, gate);
    gate = properties.gate("shadow_enabled");
    CHECK(gate && gate->isChecked());
    PropertyValue value;
    CHECK(NodeClassDB::getProperty(sun, "shadow_enabled", value) && value.asBool());
    CHECK(sun->castTo<DirectionalLight3D>()->hasShadow());
    CHECK(properties.rowNamed("shadow_bias") != nullptr &&
            properties.rowNamed("shadow_normal_bias") != nullptr);
    CHECK(properties.rowNamed("shadow_opacity") != nullptr &&
            properties.rowNamed("shadow_blur") != nullptr);
    CHECK(properties.section("Directional Shadow") != nullptr);
    CHECK(properties.rowNamed("directional_shadow_mode") != nullptr);
    CHECK(properties.rowNamed("directional_shadow_max_distance") != nullptr);
    CHECK(rig.document.undoRedo().undoCount() == before + 1);

    rig.tap(Key::Z, kModControl);
    CHECK(!properties.gate("shadow_enabled")->isChecked());
    CHECK(!sun->castTo<DirectionalLight3D>()->hasShadow());
    CHECK(properties.section("Directional Shadow") == nullptr);
    CHECK(properties.rowNamed("shadow_bias") == nullptr);
    rig.tap(Key::Y, kModControl);
    CHECK(properties.gate("shadow_enabled")->isChecked());
    CHECK(sun->castTo<DirectionalLight3D>()->hasShadow());
    CHECK(properties.section("Directional Shadow") != nullptr);
    CHECK(properties.rowNamed("directional_shadow_mode") != nullptr);

    clickGate(rig, properties.gate("shadow_enabled"));
    CHECK(!properties.gate("shadow_enabled")->isChecked());
    CHECK(!sun->castTo<DirectionalLight3D>()->hasShadow());
    CHECK(properties.rowNamed("shadow_bias") == nullptr);
    CHECK(properties.section("Directional Shadow") == nullptr);
    CHECK(properties.rowNamed("directional_shadow_mode") == nullptr);

    select(rig, lamp);
    gate = properties.gate("shadow_enabled");
    CHECK(gate && !gate->isChecked());
    CHECK(properties.rowNamed("omni_shadow_mode") == nullptr);
    CHECK(properties.section("Directional Shadow") == nullptr);
    clickGate(rig, gate);
    CHECK(lamp->castTo<Light3D>()->hasShadow());
    CHECK(properties.rowNamed("omni_shadow_mode") != nullptr);
    CHECK(properties.rowNamed("shadow_bias") != nullptr);
    CHECK(properties.section("Directional Shadow") == nullptr);
}

} // namespace

int main(int argc, char** argv)
{
    useVulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor project settings test";
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
        CHECK(hierarchy.attach(shell.panel(EditorShell::kHierarchyPanel)));
        CHECK(inspector.attach(shell.panel(EditorShell::kInspectorPanel)));
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

        EditorViewportPanel viewport(server);
        viewport.bindEditor(tree, selection);
        viewport.setChangeListener(&document);
        EditorGizmo* gizmo = viewport.gizmo();
        CHECK(gizmo != nullptr);
        if (gizmo)
            gizmo->setRootProvider([&document]() -> Node* { return document.scene(); });
        CHECK(viewport.attach(shell.panel(EditorShell::kScenePanel)));
        viewport.setScenario(window->getScenario());

        GamePanel game(server, document);
        CHECK(game.attach(shell.panel(EditorShell::kGamePanel)));
        game.setScenario(window->getScenario());

        HierarchyActions actions(tree, document, selection, hierarchy);
        CHECK(actions.attach());
        actions.setViewport(&viewport);
        ProjectSettingsEditor projectSettings(document);
        projectSettings.bindRendering(server, &viewport, &game);
        EditorMenus menus(shell, ui, viewport, actions);
        menus.setProjectSettings(&projectSettings);
        CHECK(menus.attach());

        Rig rig = { tree, host, shell, selection, hierarchy, inspector, document, ui, viewport,
            game, actions, menus, projectSettings, server };
        rig.frames(4);

        CHECK(!projectSettings.canOpen());
        projectSettings.open();
        CHECK(!projectSettings.isOpen());
        CHECK(!menus.action(EditorMenus::kFile, EditorMenus::kFileSettings)->isEnabled());

        CHECK(document.createProject(testRoot() + "/projects", "Settings"));
        rig.frames(3);
        CHECK(projectSettings.canOpen());
        CHECK(menus.action(EditorMenus::kFile, EditorMenus::kFileSettings)->isEnabled());

        testMenuAndWindow(rig);
        testSectionTree(rig);
        testSearch(rig);
        testEditAndRevert(rig);
        testLiveApply(rig);
        testRestartNotice(rig);
        testSavedWithProject(rig);
        testEnvironmentMenu(rig);
        testEnvironmentGroups(rig);
        testLightShadowGate(rig);

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
