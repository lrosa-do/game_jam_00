#include "prisma3d/editor/EditorInput.h"

#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/ProjectUi.h"
#include "prisma3d/editor/WidgetHost.h"

namespace prisma3d
{

EditorInput::EditorInput(WidgetHost& host, ProjectUi& project, HierarchyActions& actions,
        EditorViewportPanel& viewport)
        : mHost(host),
          mProject(project),
          mActions(actions),
          mViewport(viewport)
{
}

void EditorInput::handle(const InputEvent& event)
{
    if (mProject.handleEvent(event)) return;
    if (mActions.handleEvent(event)) return;
    mHost.handleEvent(event);
    mViewport.handleEvent(event);
}

} // namespace prisma3d
