#pragma once

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

class EditorSettings
{
public:
    static constexpr std::size_t kMaxRecentProjects = 12;
    static constexpr std::size_t kMaxRecentCreated = 15;
    static constexpr std::size_t kMaxRecentFolders = 8;
    static constexpr const char* kIniName = "prisma3d_editor.ini";

    static constexpr const char* kFolderOpenProject = "open_project";
    static constexpr const char* kFolderNewProject = "new_project";
    static constexpr const char* kFolderScene = "scene";
    static constexpr const char* kFolderImport = "import";
    static constexpr const char* kFolderTexture = "texture";
    static constexpr const char* kFolderFile = "file";
    static constexpr const char* kFolderScript = "script";

    void setPath(const ct::String& path);
    const ct::String& path() const;
    bool flush() const;
    bool load(const ct::String& path);
    bool save(const ct::String& path) const;
    bool loadLegacyJson(const ct::String& path);
    bool loadLegacyLayout(const ct::String& path);

    static ct::String appFolder();
    static ct::String defaultPath();

    void rememberProject(const ct::String& manifestPath);
    void forgetProject(const ct::String& manifestPath);
    void clearLastProject();

    void rememberCreated(const ct::String& type);
    bool isCreateFavorite(const ct::String& type) const;
    void setCreateFavorite(const ct::String& type, bool favorite);

    ct::String folderFor(const char* purpose) const;
    void rememberFolder(const char* purpose, const ct::String& folder);
    void rememberFolderOfFile(const char* purpose, const ct::String& path);
    const ct::String* lastFolder(const char* purpose) const;

    ct::String lastProjectPath;
    ct::Vector<ct::String> recentProjectPaths;
    ct::Vector<ct::String> createFavorites;
    ct::Vector<ct::String> createRecent;

    struct FolderEntry
    {
        ct::String purpose;
        ct::String path;
    };
    ct::Vector<FolderEntry> folders;
    ct::Vector<ct::String> recentFolders;

    int windowWidth = 0;
    int windowHeight = 0;
    int windowX = 0;
    int windowY = 0;
    bool hasWindowPosition = false;
    bool windowMaximized = false;
    ct::String dockLayout;

    float thumbnailSize = 0.0f;
    int assetViewMode = -1;

    bool consoleTrace = true;
    bool consoleInfo = true;
    bool consoleWarn = true;
    bool consoleError = true;
    bool consoleAutoScroll = true;

    static constexpr int kDefaultMcpPort = 8787;
    int mcpPort = kDefaultMcpPort;
    bool mcpAllowEdits = false;
    bool mcpAllowCode = false;

private:
    ct::String mPath;
};

} // namespace prisma3d
