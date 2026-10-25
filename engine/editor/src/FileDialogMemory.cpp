#include "prisma3d/editor/FileDialogMemory.h"

#include "prisma3d/main/Project.h"

#include "platform.h"

namespace prisma3d
{

namespace FileDialogMemory
{

namespace
{

ct::String leafOf(const ct::String& path)
{
    const std::size_t slash = path.rfind('/');
    if (slash == ct::String::npos || slash + 1 >= path.size()) return path;
    return path.substr(slash + 1);
}

} // namespace

void prepare(ig::retained::FileDialog& dialog, EditorSettings* settings, const char* purpose,
        const Project* project)
{
    const ct::String app = EditorSettings::appFolder();
    dialog.setPath(settings ? settings->folderFor(purpose) : app);
    dialog.addBookmark("Editor", app);
    if (project && project->isOpen())
    {
        PathInfo info;
        const ct::String assets = project->assetPath();
        if (fs_get_path_info(assets.c_str(), &info) && info.type == PATH_TYPE_DIRECTORY)
            dialog.addBookmark("Project assets", assets);
        const ct::String scenes = project->absolutePath(Project::kScenesFolder);
        if (fs_get_path_info(scenes.c_str(), &info) && info.type == PATH_TYPE_DIRECTORY)
            dialog.addBookmark("Project scenes", scenes);
    }
    if (!settings) return;
    for (std::size_t i = 0; i < settings->recentFolders.size(); ++i)
    {
        const ct::String& folder = settings->recentFolders[i];
        PathInfo info;
        if (!fs_get_path_info(folder.c_str(), &info) || info.type != PATH_TYPE_DIRECTORY) continue;
        dialog.addBookmark(ct::String("Recent: ") + leafOf(folder), folder);
    }
}

void remember(EditorSettings* settings, const char* purpose, const ct::String& chosen)
{
    if (!settings) return;
    settings->rememberFolderOfFile(purpose, chosen);
    settings->flush();
}

} // namespace FileDialogMemory

} // namespace prisma3d
