#include "prisma3d/editor/EditorShell.h"

#include "prisma3d/editor/EditorIcons.h"

#include "platform.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Theme.hpp>

#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kPanelNames[EditorShell::kPanelCount] = { "Hierarchy", "Scene", "Inspector",
    "Console", "Assets", "Material", "Game", "AI", "Lightmap", "Script" };
const char* const kLegacyViewportName = "Viewport";
const char* const kStageName = "editor";

} // namespace

EditorShell::EditorShell(WidgetHost& host)
        : mHost(host),
          mDock(nullptr),
          mColumn(nullptr),
          mMenuBar(nullptr),
          mToolRow(nullptr),
          mToolBar(nullptr),
          mRunBar(nullptr),
          mToolFill(nullptr),
          mStatusBar(nullptr),
          mPanels{},
          mResetLayoutButton(-1)
{
}

EditorShell::~EditorShell()
{
}

bool EditorShell::build()
{
    if (mDock) return true;
    if (!mHost.ready()) return false;

    rt::Theme::instance() = rt::Theme::dark();
    rt::WidgetApp& app = mHost.widgets();
    rt::Widget* stage = app.addStage(kStageName);
    if (!stage || !app.setStage(kStageName)) return false;
    EditorIcons::loadDefault();

    mColumn = stage->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mColumn->setStretch(1);
    mColumn->setPadding(0.0f);
    mColumn->setSpacing(0.0f);

    mMenuBar = mColumn->createChild<rt::MenuBar>();
    mToolRow = mColumn->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    mToolRow->setPadding(0.0f);
    mToolRow->setSpacing(0.0f);
    mToolBar = mToolRow->createChild<rt::ToolBar>();
    mToolBar->setStretch(1.0f);
    mToolBar->setButtonSize(kToolBarButtonSize);
    mToolBar->setIconPadding(kToolBarIconPadding);
    mRunBar = mToolRow->createChild<rt::ToolBar>();
    mRunBar->setButtonSize(kToolBarButtonSize);
    mRunBar->setIconPadding(kToolBarIconPadding);
    mRunBar->setDraggable(false);
    mToolFill = mToolRow->createChild<rt::ToolBar>();
    mToolFill->setStretch(1.0f);
    mToolFill->setDraggable(false);
    mResetLayoutButton = mToolBar->addButton(resetLayoutIcon(), "Reset Layout");
    mToolBar->buttonClicked.connect(
            [this](int id)
            {
                if (id == mResetLayoutButton) resetLayout();
            });

    mDock = mColumn->createChild<rt::DockPanel>();
    mDock->setStretch(1);
    for (int i = 0; i < kPanelCount; ++i) mPanels[i] = mDock->addPanel<rt::Panel>(kPanelNames[i]);
    mDock->splitOff("Hierarchy", rt::DockSide::Left, 0.20f);
    mDock->splitOff("Inspector", rt::DockSide::Right, 0.22f);
    mDock->splitOff("Console", rt::DockSide::Bottom, 0.25f);
    mDock->moveTabToLeaf("Assets", "Console");
    mDock->moveTabToLeaf(kPanelNames[kAiPanel], "Console");
    mDock->showPanel("Console");
    mDock->moveTabToLeaf("Material", "Inspector");
    mDock->moveTabToLeaf(kPanelNames[kLightmapPanel], "Inspector");
    mDock->showPanel("Inspector");
    mDock->moveTabToLeaf(kPanelNames[kScriptPanel], kPanelNames[kScenePanel]);
    mDock->showPanel(kPanelNames[kScenePanel]);
    mStatusBar = mColumn->createChild<rt::StatusBar>();
    mDefaultLayout = mDock->saveLayout();
    return true;
}

bool EditorShell::built() const
{
    return mDock != nullptr;
}

const char* EditorShell::panelName(int index)
{
    return index >= 0 && index < kPanelCount ? kPanelNames[index] : "";
}

rt::Widget* EditorShell::panel(int index) const
{
    return index >= 0 && index < kPanelCount ? mPanels[index] : nullptr;
}

rt::DockPanel* EditorShell::dock() const
{
    return mDock;
}

rt::ToolBar* EditorShell::toolBar() const
{
    return mToolBar;
}

rt::ToolBar* EditorShell::runBar() const
{
    return mRunBar;
}

rt::IconId EditorShell::resetLayoutIcon()
{
    return EditorIcons::get("Reload");
}

rt::MenuBar* EditorShell::menuBar() const
{
    return mMenuBar;
}

rt::StatusBar* EditorShell::statusBar() const
{
    return mStatusBar;
}

rt::Widget* EditorShell::column() const
{
    return mColumn;
}

int EditorShell::resetLayoutButton() const
{
    return mResetLayoutButton;
}

rt::String EditorShell::layout() const
{
    return mDock ? mDock->saveLayout() : rt::String();
}

bool EditorShell::setLayout(const rt::String& text)
{
    if (!mDock || text.empty()) return false;
    rt::String adjusted = text;
    const std::size_t legacy = adjusted.find(kLegacyViewportName);
    if (legacy != rt::String::npos)
        adjusted = adjusted.substr(0, legacy) + kPanelNames[kScenePanel] +
                   adjusted.substr(legacy + strlen(kLegacyViewportName));
    if (!mDock->restoreLayout(adjusted)) return false;
    if (strstr(adjusted.c_str(), kPanelNames[kMaterialPanel]) == nullptr)
    {
        mDock->moveTabToLeaf(kPanelNames[kMaterialPanel], "Inspector");
        mDock->showPanel("Inspector");
    }
    if (strstr(adjusted.c_str(), kPanelNames[kGamePanel]) == nullptr)
    {
        mDock->moveTabToLeaf(kPanelNames[kGamePanel], kPanelNames[kScenePanel]);
        mDock->showPanel(kPanelNames[kScenePanel]);
    }
    if (strstr(adjusted.c_str(), kPanelNames[kAiPanel]) == nullptr)
        mDock->moveTabToLeaf(kPanelNames[kAiPanel], "Console");
    if (strstr(adjusted.c_str(), kPanelNames[kLightmapPanel]) == nullptr)
        mDock->moveTabToLeaf(kPanelNames[kLightmapPanel], "Inspector");
    if (strstr(adjusted.c_str(), kPanelNames[kScriptPanel]) == nullptr)
    {
        mDock->moveTabToLeaf(kPanelNames[kScriptPanel], kPanelNames[kScenePanel]);
        mDock->showPanel(kPanelNames[kScenePanel]);
    }
    mDock->layout();
    return true;
}

void EditorShell::showPanel(int index)
{
    if (!mDock || index < 0 || index >= kPanelCount) return;
    mDock->showPanel(kPanelNames[index]);
    mDock->layout();
}

void EditorShell::resetLayout()
{
    setLayout(mDefaultLayout);
}

bool EditorShell::loadLayout(const char* path)
{
    if (!mDock || !path) return false;
    char* text = file_read_text(path);
    if (!text) return false;
    const bool applied = setLayout(rt::String(text));
    free(text);
    return applied;
}

bool EditorShell::saveLayout(const char* path) const
{
    if (!mDock || !path) return false;
    const rt::String text = mDock->saveLayout();
    return file_write_text(path, text.c_str());
}

} // namespace prisma3d
