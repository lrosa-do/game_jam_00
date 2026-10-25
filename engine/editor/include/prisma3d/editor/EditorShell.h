#pragma once

#include "prisma3d/editor/WidgetHost.h"

#include <igui/widgets/DockPanel.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/MenuWidgets.hpp>
#include <igui/widgets/ToolBar.hpp>

namespace prisma3d
{

class EditorShell
{
public:
    static constexpr int kPanelCount = 10;
    static constexpr int kHierarchyPanel = 0;
    static constexpr int kScenePanel = 1;
    static constexpr int kInspectorPanel = 2;
    static constexpr int kConsolePanel = 3;
    static constexpr int kMaterialPanel = 5;
    static constexpr int kGamePanel = 6;
    static constexpr int kAiPanel = 7;
    static constexpr int kLightmapPanel = 8;
    static constexpr int kScriptPanel = 9;

    explicit EditorShell(WidgetHost& host);
    ~EditorShell();
    EditorShell(const EditorShell&) = delete;
    EditorShell& operator=(const EditorShell&) = delete;

    bool build();
    bool built() const;

    static const char* panelName(int index);
    ig::retained::Widget* panel(int index) const;
    ig::retained::DockPanel* dock() const;
    ig::retained::ToolBar* toolBar() const;
    ig::retained::ToolBar* runBar() const;
    ig::retained::MenuBar* menuBar() const;
    ig::retained::StatusBar* statusBar() const;
    ig::retained::Widget* column() const;
    static constexpr float kToolBarButtonSize = 28.0f;
    static constexpr float kToolBarIconPadding = 6.0f;
    int resetLayoutButton() const;
    static ig::retained::IconId resetLayoutIcon();

    ig::retained::String layout() const;
    bool setLayout(const ig::retained::String& text);
    void resetLayout();
    void showPanel(int index);
    bool loadLayout(const char* path);
    bool saveLayout(const char* path) const;

private:
    WidgetHost& mHost;
    ig::retained::DockPanel* mDock;
    ig::retained::BoxLayout* mColumn;
    ig::retained::MenuBar* mMenuBar;
    ig::retained::BoxLayout* mToolRow;
    ig::retained::ToolBar* mToolBar;
    ig::retained::ToolBar* mRunBar;
    ig::retained::ToolBar* mToolFill;
    ig::retained::StatusBar* mStatusBar;
    ig::retained::Widget* mPanels[kPanelCount];
    ig::retained::String mDefaultLayout;
    int mResetLayoutButton;
};

} // namespace prisma3d
