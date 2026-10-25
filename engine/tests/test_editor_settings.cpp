#include "Check.h"
#include "TestFiles.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/editor/EditorSettings.h"
#include "prisma3d/main/Project.h"

#include <ct/ini.hpp>
#include <ct/json.hpp>

#include <string.h>

using namespace prisma3d;

namespace
{

ct::String testRoot()
{
    ct::String root = PRISMA3D_TEST_DIR;
    root += "/editor_settings_test";
    return root;
}

void testRemember()
{
    EditorSettings settings;
    CHECK(settings.lastProjectPath.empty());
    CHECK(settings.recentProjectPaths.size() == 0);

    settings.rememberProject("");
    CHECK(settings.recentProjectPaths.size() == 0);

    settings.rememberProject("/projects/a/projecto.json");
    settings.rememberProject("/projects/b/projecto.json");
    settings.rememberProject("/projects/c/projecto.json");
    CHECK(settings.lastProjectPath == "/projects/c/projecto.json");
    CHECK(settings.recentProjectPaths.size() == 3);
    CHECK(settings.recentProjectPaths[0] == "/projects/c/projecto.json");
    CHECK(settings.recentProjectPaths[2] == "/projects/a/projecto.json");

    settings.rememberProject("/projects/a/../a/projecto.json");
    CHECK(settings.recentProjectPaths.size() == 3);
    CHECK(settings.recentProjectPaths[0] == "/projects/a/projecto.json");
    CHECK(settings.recentProjectPaths[1] == "/projects/c/projecto.json");
    CHECK(settings.lastProjectPath == "/projects/a/projecto.json");

    for (int i = 0; i < 20; ++i)
    {
        ct::String path = "/many/";
        path.append_number(i);
        path += "/projecto.json";
        settings.rememberProject(path);
    }
    CHECK(settings.recentProjectPaths.size() == EditorSettings::kMaxRecentProjects);
    CHECK(settings.recentProjectPaths[0] == "/many/19/projecto.json");
    CHECK(settings.recentProjectPaths[11] == "/many/8/projecto.json");

    settings.forgetProject("/many/19/projecto.json");
    CHECK(settings.recentProjectPaths.size() == 11);
    CHECK(settings.recentProjectPaths[0] == "/many/18/projecto.json");
    CHECK(settings.lastProjectPath.empty());
    settings.forgetProject("/nowhere/projecto.json");
    CHECK(settings.recentProjectPaths.size() == 11);

    settings.rememberProject("/projects/z/projecto.json");
    CHECK(!settings.lastProjectPath.empty());
    settings.clearLastProject();
    CHECK(settings.lastProjectPath.empty());
    CHECK(settings.recentProjectPaths.size() == 12);
}

void testFile()
{
    removeTree(testRoot());
    const ct::String file = testRoot() + "/nested/settings.ini";

    EditorSettings missing;
    CHECK(!missing.load(file));

    EditorSettings saved;
    saved.rememberProject("/projects/a/projecto.json");
    saved.rememberProject("/projects/b/projecto.json");
    CHECK(saved.save(file));

    EditorSettings loaded;
    CHECK(loaded.load(file));
    CHECK(loaded.lastProjectPath == "/projects/b/projecto.json");
    CHECK(loaded.recentProjectPaths.size() == 2);
    CHECK(loaded.recentProjectPaths[0] == "/projects/b/projecto.json");
    CHECK(loaded.recentProjectPaths[1] == "/projects/a/projecto.json");

    CHECK(FileSystem::writeText(file,
            "[project]\nlast=\nrecent.0=/a\nrecent.1=\nrecent.2=/a\nrecent.3=/b\n"));
    EditorSettings tolerant;
    CHECK(tolerant.load(file));
    CHECK(tolerant.lastProjectPath.empty());
    CHECK(tolerant.recentProjectPaths.size() == 2);

    ct::String many = "[project]\n";
    for (int i = 0; i < 30; ++i)
    {
        many += "recent.";
        many.append_number(i);
        many += "=/p";
        many.append_number(i);
        many += "\n";
    }
    CHECK(FileSystem::writeText(file, many));
    EditorSettings capped;
    CHECK(capped.load(file));
    CHECK(capped.recentProjectPaths.size() == EditorSettings::kMaxRecentProjects);
}

void testWithProject()
{
    Project project;
    CHECK(project.create(testRoot() + "/projects", "Remembered"));
    const ct::String file = testRoot() + "/with_project/settings.ini";

    EditorSettings first;
    first.rememberProject(project.manifestPath());
    CHECK(first.save(file));

    EditorSettings second;
    CHECK(second.load(file));
    CHECK(second.lastProjectPath == project.manifestPath());
    Project reopened;
    CHECK(reopened.open(second.lastProjectPath));
    CHECK(reopened.name() == "Remembered");

    Project gone;
    CHECK(!gone.open(testRoot() + "/projects/Missing/projecto.json"));
    second.forgetProject(testRoot() + "/projects/Missing/projecto.json");
    CHECK(second.lastProjectPath == project.manifestPath());
}

void testIni()
{
    removeTree(testRoot());
    const ct::String file = testRoot() + "/bin/prisma3d_editor.ini";

    EditorSettings saved;
    saved.setPath(file);
    saved.rememberProject("/projects/a/projecto.json");
    saved.rememberProject("/projects/b/projecto.json");
    saved.setCreateFavorite("Camera3D", true);
    saved.rememberCreated("OmniLight3D");
    saved.rememberFolder(EditorSettings::kFolderImport, testRoot());
    saved.rememberFolder(EditorSettings::kFolderTexture, testRoot() + "/bin");
    saved.windowWidth = 1500;
    saved.windowHeight = 820;
    saved.windowX = 30;
    saved.windowY = 40;
    saved.hasWindowPosition = true;
    saved.windowMaximized = true;
    saved.dockLayout = "H200[{Hierarchy}|{Viewport}]\nsecond \"line\"";
    saved.thumbnailSize = 120.0f;
    saved.assetViewMode = 2;
    saved.consoleWarn = false;
    saved.consoleAutoScroll = false;
    CHECK(saved.flush());
    CHECK(file_exists(file.c_str()));

    ct::Ini ini;
    CHECK(ct::Ini::load(ini, file.c_str()));
    CHECK(ini.get("project", "last") == "/projects/b/projecto.json");
    CHECK(ini.get("project", "recent.1") == "/projects/a/projecto.json");
    CHECK(ini.get_int("editor", "window_width") == 1500);
    CHECK(ini.get_bool("editor", "maximized"));
    CHECK(ini.get("folders", "import") == testRoot());
    CHECK(!ini.get_bool("console", "warn", true));

    EditorSettings loaded;
    CHECK(loaded.load(file));
    CHECK(loaded.lastProjectPath == "/projects/b/projecto.json");
    CHECK(loaded.recentProjectPaths.size() == 2 &&
            loaded.recentProjectPaths[1] == "/projects/a/projecto.json");
    CHECK(loaded.isCreateFavorite("Camera3D") && loaded.createRecent.size() == 1 &&
            loaded.createRecent[0] == "OmniLight3D");
    CHECK(loaded.windowWidth == 1500 && loaded.windowHeight == 820 && loaded.windowX == 30 &&
            loaded.windowY == 40 && loaded.hasWindowPosition && loaded.windowMaximized);
    CHECK(loaded.dockLayout == saved.dockLayout);
    CHECK(loaded.thumbnailSize == 120.0f && loaded.assetViewMode == 2);
    CHECK(!loaded.consoleWarn && loaded.consoleError && !loaded.consoleAutoScroll);
    CHECK(loaded.folderFor(EditorSettings::kFolderImport) == testRoot());
    CHECK(loaded.folderFor(EditorSettings::kFolderTexture) == testRoot() + "/bin");
    CHECK(loaded.recentFolders.size() == 2 && loaded.recentFolders[0] == testRoot() + "/bin");

    CHECK(loaded.folderFor(EditorSettings::kFolderFile) == EditorSettings::appFolder());
    loaded.rememberFolder(EditorSettings::kFolderFile, testRoot() + "/nowhere");
    CHECK(loaded.folderFor(EditorSettings::kFolderFile) == EditorSettings::appFolder());
    loaded.rememberFolderOfFile(EditorSettings::kFolderFile, file);
    CHECK(loaded.folderFor(EditorSettings::kFolderFile) == testRoot() + "/bin");

    CHECK(EditorSettings::defaultPath() ==
            EditorSettings::appFolder() + "/" + EditorSettings::kIniName);
    CHECK(EditorSettings::appFolder().size() > 1 && !EditorSettings::appFolder().ends_with("/"));
}

void testMigration()
{
    removeTree(testRoot());
    const ct::String legacy = testRoot() + "/old/prisma3d_editor_settings.json";
    const ct::String layout = testRoot() + "/old/prisma3d_editor_layout.txt";
    CHECK(FileSystem::writeText(legacy,
            "{\"lastProjectPath\": \"/p/last/projecto.json\", \"recentProjectPaths\": "
            "[\"/p/last/projecto.json\", \"/p/old/projecto.json\"], \"createFavorites\": "
            "[\"Camera3D\"], \"createRecent\": [\"Node3D\", \"OmniLight3D\"]}"));
    CHECK(FileSystem::writeText(layout, "H200[{Hierarchy}|{Viewport}]"));

    EditorSettings settings;
    CHECK(settings.loadLegacyJson(legacy));
    CHECK(settings.loadLegacyLayout(layout));
    CHECK(settings.lastProjectPath == "/p/last/projecto.json");
    CHECK(settings.recentProjectPaths.size() == 2);
    CHECK(settings.isCreateFavorite("Camera3D") && settings.createRecent.size() == 2);
    CHECK(settings.dockLayout == "H200[{Hierarchy}|{Viewport}]");

    const ct::String ini = testRoot() + "/new/prisma3d_editor.ini";
    CHECK(settings.save(ini));
    EditorSettings fresh;
    CHECK(fresh.load(ini));
    CHECK(fresh.lastProjectPath == settings.lastProjectPath);
    CHECK(fresh.recentProjectPaths.size() == 2 && fresh.createRecent.size() == 2);
    CHECK(fresh.dockLayout == settings.dockLayout);

    EditorSettings none;
    CHECK(!none.loadLegacyJson(testRoot() + "/old/missing.json"));
    CHECK(!none.load(testRoot() + "/new/missing.ini"));
}

} // namespace

int main()
{
    testRemember();
    testFile();
    testWithProject();
    testIni();
    testMigration();

    if (failures)
    {
        printf("%d failures\n", failures);
        return 1;
    }
    printf("editor settings ok\n");
    return 0;
}
