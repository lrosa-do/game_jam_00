#pragma once

#include "prisma3d/Input.h"

namespace prisma3d
{

class EditorViewportPanel;
class HierarchyActions;
class ProjectUi;
class WidgetHost;

class EditorInput
{
public:
    EditorInput(WidgetHost& host, ProjectUi& project, HierarchyActions& actions,
            EditorViewportPanel& viewport);

    void handle(const InputEvent& event);

private:
    WidgetHost& mHost;
    ProjectUi& mProject;
    HierarchyActions& mActions;
    EditorViewportPanel& mViewport;
};

} // namespace prisma3d
