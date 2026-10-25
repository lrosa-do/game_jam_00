#include "prisma3d/editor/EditorSettings.h"

#include "prisma3d/core/FileSystem.h"

#include <ct/ini.hpp>
#include <ct/json.hpp>

#include "platform.h"


namespace prisma3d
{

namespace
{

ct::String normalized(const ct::String& path)
{
    char buffer[4096];
    if (!path_absolute(buffer, sizeof buffer, path.c_str())) return path;
    return ct::String(buffer);
}

std::size_t indexOf(const ct::Vector<ct::String>& list, const ct::String& path)
{
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i] == path) return i;
    return list.size();
}

void loadNames(const ct::Json* source, ct::Vector<ct::String>& list, std::size_t limit)
{
    if (!source || !source->is_array()) return;
    list.clear();
    for (std::size_t i = 0; i < source->size(); ++i)
    {
        const ct::Json& entry = source->at(i);
        if (!entry.is_string() || entry.str().empty()) continue;
        if (indexOf(list, entry.str()) < list.size()) continue;
        if (list.size() >= limit) break;
        list.push_back(entry.str());
    }
}

} // namespace

bool EditorSettings::loadLegacyJson(const ct::String& path)
{
    const ct::String text = FileSystem::getSingleton().readText(path);
    if (text.empty()) return false;

    ct::Json::Error error;
    const ct::Json root = ct::Json::parse(text, &error);
    if (error)
    {
        log_error("editor settings: '%s' is not valid JSON: %s", path.c_str(), error.message);
        return false;
    }
    if (!root.is_object()) return false;

    const ct::Json* last = root.find("lastProjectPath");
    if (last && last->is_string()) lastProjectPath = last->str();
    const ct::Json* recent = root.find("recentProjectPaths");
    if (recent && recent->is_array())
    {
        recentProjectPaths.clear();
        for (std::size_t i = 0; i < recent->size(); ++i)
        {
            const ct::Json& entry = recent->at(i);
            if (!entry.is_string() || entry.str().empty()) continue;
            if (indexOf(recentProjectPaths, entry.str()) < recentProjectPaths.size()) continue;
            if (recentProjectPaths.size() >= kMaxRecentProjects) break;
            recentProjectPaths.push_back(entry.str());
        }
    }
    loadNames(root.find("createFavorites"), createFavorites, static_cast<std::size_t>(-1));
    loadNames(root.find("createRecent"), createRecent, kMaxRecentCreated);
    return true;
}

namespace
{

ct::String indexed(const char* prefix, std::size_t index)
{
    ct::String key(prefix);
    key += '.';
    key.append_number(static_cast<unsigned long long>(index));
    return key;
}

void saveList(ct::Ini& ini, const char* section, const char* prefix,
        const ct::Vector<ct::String>& list)
{
    for (std::size_t i = 0; i < list.size(); ++i)
        ini.set(section, indexed(prefix, i).c_str(), list[i]);
}

void loadList(const ct::Ini& ini, const char* section, const char* prefix,
        ct::Vector<ct::String>& list, std::size_t limit)
{
    list.clear();
    for (std::size_t i = 0;; ++i)
    {
        const ct::String key = indexed(prefix, i);
        if (!ini.has(section, key.c_str())) break;
        const ct::String value = ini.get(section, key.c_str());
        if (value.empty() || indexOf(list, value) < list.size()) continue;
        if (list.size() >= limit) break;
        list.push_back(value);
    }
}

} // namespace

ct::String EditorSettings::appFolder()
{
    ct::String folder(dir_app());
    while (folder.size() > 1 && (folder.ends_with("/") || folder.ends_with("\\")))
        folder.pop_back();
    return folder;
}

ct::String EditorSettings::defaultPath()
{
    ct::String path = appFolder();
    path += '/';
    path += kIniName;
    return path;
}

void EditorSettings::setPath(const ct::String& path)
{
    mPath = path;
}

const ct::String& EditorSettings::path() const
{
    return mPath;
}

bool EditorSettings::flush() const
{
    return !mPath.empty() && save(mPath);
}

bool EditorSettings::load(const ct::String& path)
{
    ct::Ini ini;
    if (!ct::Ini::load(ini, path.c_str())) return false;

    lastProjectPath = ini.get("project", "last");
    loadList(ini, "project", "recent", recentProjectPaths, kMaxRecentProjects);
    loadList(ini, "create", "favorite", createFavorites, static_cast<std::size_t>(-1));
    loadList(ini, "create", "recent", createRecent, kMaxRecentCreated);

    folders.clear();
    if (const ct::Ini::Section* section = ini.section("folders"))
    {
        for (std::size_t i = 0; i < section->entries.size(); ++i)
        {
            const ct::Ini::Entry& entry = section->entries[i];
            if (entry.key.starts_with("recent.") || entry.value.empty()) continue;
            FolderEntry folder;
            folder.purpose = entry.key;
            folder.path = entry.value;
            folders.push_back(folder);
        }
    }
    loadList(ini, "folders", "recent", recentFolders, kMaxRecentFolders);

    windowWidth = static_cast<int>(ini.get_int("editor", "window_width", 0));
    windowHeight = static_cast<int>(ini.get_int("editor", "window_height", 0));
    hasWindowPosition = ini.has("editor", "window_x") && ini.has("editor", "window_y");
    windowX = static_cast<int>(ini.get_int("editor", "window_x", 0));
    windowY = static_cast<int>(ini.get_int("editor", "window_y", 0));
    windowMaximized = ini.get_bool("editor", "maximized", false);
    dockLayout = ini.get("editor", "layout");
    thumbnailSize = static_cast<float>(ini.get_double("assets", "thumbnail_size", 0.0));
    assetViewMode = static_cast<int>(ini.get_int("assets", "view_mode", -1));
    consoleTrace = ini.get_bool("console", "trace", true);
    consoleInfo = ini.get_bool("console", "info", true);
    consoleWarn = ini.get_bool("console", "warn", true);
    consoleError = ini.get_bool("console", "error", true);
    consoleAutoScroll = ini.get_bool("console", "auto_scroll", true);
    mcpPort = static_cast<int>(ini.get_int("mcp", "port", kDefaultMcpPort));
    if (mcpPort < 1024 || mcpPort > 65535) mcpPort = kDefaultMcpPort;
    mcpAllowEdits = ini.get_bool("mcp", "allow_edits", false);
    mcpAllowCode = ini.get_bool("mcp", "allow_code", false);
    return true;
}

bool EditorSettings::save(const ct::String& path) const
{
    ct::Ini ini;
    ini.set("project", "last", lastProjectPath);
    saveList(ini, "project", "recent", recentProjectPaths);
    saveList(ini, "create", "favorite", createFavorites);
    saveList(ini, "create", "recent", createRecent);
    for (std::size_t i = 0; i < folders.size(); ++i)
        ini.set("folders", folders[i].purpose.c_str(), folders[i].path);
    saveList(ini, "folders", "recent", recentFolders);
    ini.set("editor", "window_width", windowWidth);
    ini.set("editor", "window_height", windowHeight);
    if (hasWindowPosition)
    {
        ini.set("editor", "window_x", windowX);
        ini.set("editor", "window_y", windowY);
    }
    ini.set("editor", "maximized", windowMaximized);
    ini.set("editor", "layout", dockLayout);
    ini.set("assets", "thumbnail_size", static_cast<double>(thumbnailSize));
    ini.set("assets", "view_mode", assetViewMode);
    ini.set("console", "trace", consoleTrace);
    ini.set("console", "info", consoleInfo);
    ini.set("console", "warn", consoleWarn);
    ini.set("console", "error", consoleError);
    ini.set("console", "auto_scroll", consoleAutoScroll);
    ini.set("mcp", "port", mcpPort);
    ini.set("mcp", "allow_edits", mcpAllowEdits);
    ini.set("mcp", "allow_code", mcpAllowCode);

    const std::size_t slash = path.rfind('/');
    if (slash != ct::String::npos && slash > 0) fs_create_directory(path.substr(0, slash).c_str());
    return ini.save(path.c_str());
}

bool EditorSettings::loadLegacyLayout(const ct::String& path)
{
    const ct::String text = FileSystem::getSingleton().readText(path);
    if (text.empty()) return false;
    dockLayout = text;
    return true;
}

ct::String EditorSettings::folderFor(const char* purpose) const
{
    if (const ct::String* remembered = lastFolder(purpose))
    {
        PathInfo info;
        if (fs_get_path_info(remembered->c_str(), &info) && info.type == PATH_TYPE_DIRECTORY)
            return *remembered;
    }
    return appFolder();
}

const ct::String* EditorSettings::lastFolder(const char* purpose) const
{
    for (std::size_t i = 0; i < folders.size(); ++i)
        if (folders[i].purpose == purpose) return &folders[i].path;
    return nullptr;
}

void EditorSettings::rememberFolder(const char* purpose, const ct::String& folder)
{
    if (!purpose || folder.empty()) return;
    ct::String value = normalized(folder);
    while (value.size() > 1 && value.ends_with("/")) value.pop_back();
    bool found = false;
    for (std::size_t i = 0; i < folders.size() && !found; ++i)
    {
        if (folders[i].purpose != purpose) continue;
        folders[i].path = value;
        found = true;
    }
    if (!found)
    {
        FolderEntry entry;
        entry.purpose = purpose;
        entry.path = value;
        folders.push_back(entry);
    }
    const std::size_t existing = indexOf(recentFolders, value);
    if (existing < recentFolders.size())
        recentFolders.erase(recentFolders.begin() + static_cast<std::ptrdiff_t>(existing));
    recentFolders.insert(recentFolders.begin(), value);
    while (recentFolders.size() > kMaxRecentFolders) recentFolders.pop_back();
}

void EditorSettings::rememberFolderOfFile(const char* purpose, const ct::String& path)
{
    if (path.empty()) return;
    PathInfo info;
    if (fs_get_path_info(path.c_str(), &info) && info.type == PATH_TYPE_DIRECTORY)
    {
        rememberFolder(purpose, path);
        return;
    }
    const std::size_t slash = path.rfind('/');
    if (slash == ct::String::npos) return;
    rememberFolder(purpose, slash == 0 ? ct::String("/") : path.substr(0, slash));
}

void EditorSettings::rememberProject(const ct::String& manifestPath)
{
    if (manifestPath.empty()) return;
    const ct::String value = normalized(manifestPath);
    lastProjectPath = value;
    const std::size_t existing = indexOf(recentProjectPaths, value);
    if (existing < recentProjectPaths.size())
        recentProjectPaths.erase(
                recentProjectPaths.begin() + static_cast<std::ptrdiff_t>(existing));
    recentProjectPaths.insert(recentProjectPaths.begin(), value);
    while (recentProjectPaths.size() > kMaxRecentProjects) recentProjectPaths.pop_back();
}

void EditorSettings::forgetProject(const ct::String& manifestPath)
{
    const ct::String value = normalized(manifestPath);
    const std::size_t existing = indexOf(recentProjectPaths, value);
    if (existing < recentProjectPaths.size())
        recentProjectPaths.erase(
                recentProjectPaths.begin() + static_cast<std::ptrdiff_t>(existing));
    if (lastProjectPath == value) lastProjectPath = "";
}

void EditorSettings::clearLastProject()
{
    lastProjectPath = "";
}

void EditorSettings::rememberCreated(const ct::String& type)
{
    if (type.empty()) return;
    const std::size_t existing = indexOf(createRecent, type);
    if (existing < createRecent.size())
        createRecent.erase(createRecent.begin() + static_cast<std::ptrdiff_t>(existing));
    createRecent.insert(createRecent.begin(), type);
    while (createRecent.size() > kMaxRecentCreated) createRecent.pop_back();
}

bool EditorSettings::isCreateFavorite(const ct::String& type) const
{
    return indexOf(createFavorites, type) < createFavorites.size();
}

void EditorSettings::setCreateFavorite(const ct::String& type, bool favorite)
{
    if (type.empty()) return;
    const std::size_t existing = indexOf(createFavorites, type);
    if (favorite)
    {
        if (existing >= createFavorites.size()) createFavorites.push_back(type);
        return;
    }
    if (existing < createFavorites.size())
        createFavorites.erase(createFavorites.begin() + static_cast<std::ptrdiff_t>(existing));
}

} // namespace prisma3d
