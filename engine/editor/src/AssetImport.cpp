#include "prisma3d/editor/AssetImport.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Project.h"

#include <ct/json.hpp>

#include "platform.h"

namespace prisma3d
{

namespace
{

ct::String slashed(const ct::String& path)
{
    ct::String result = path;
    for (std::size_t i = 0; i < result.size(); ++i)
        if (result.data()[i] == '\\') result.data()[i] = '/';
    return result;
}

bool isFile(const ct::String& path)
{
    PathInfo info;
    return !path.empty() && fs_get_path_info(path.c_str(), &info) && info.type == PATH_TYPE_FILE;
}

ct::String absoluteOf(const ct::String& path)
{
    char buffer[4096];
    if (path_absolute(buffer, sizeof buffer, path.c_str())) return slashed(ct::String(buffer));
    return slashed(path);
}

int hexValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool alreadyPlanned(const ImportPlan& plan, const ct::String& destination)
{
    for (std::size_t i = 0; i < plan.files.size(); ++i)
        if (plan.files[i].destination == destination) return true;
    return false;
}

void collectUris(const ct::Json& document, const char* key, ct::Vector<ct::String>& out)
{
    const ct::Json* list = document.find(key);
    if (!list || !list->is_array()) return;
    for (std::size_t i = 0; i < list->size(); ++i)
    {
        const ct::Json* uri = (*list)[i].find("uri");
        if (uri && uri->is_string()) out.push_back(uri->str());
    }
}

} // namespace

std::size_t ImportPlan::clashCount() const
{
    std::size_t count = 0;
    for (std::size_t i = 0; i < files.size(); ++i)
        if (files[i].exists) ++count;
    return count;
}

int ImportPlan::firstModel() const
{
    for (std::size_t i = 0; i < files.size(); ++i)
        if (files[i].primary && files[i].kind == AssetKind::Model) return static_cast<int>(i);
    return -1;
}

bool ImportPlan::empty() const
{
    return files.empty();
}

ct::String AssetImport::parentOf(const ct::String& path)
{
    const ct::String text = slashed(path);
    const std::size_t slash = text.rfind('/');
    if (slash == ct::String::npos) return ct::String();
    if (slash == 0) return ct::String("/");
    return text.substr(0, slash);
}

ct::String AssetImport::nameOf(const ct::String& path)
{
    const ct::String text = slashed(path);
    const std::size_t slash = text.rfind('/');
    return slash == ct::String::npos ? text : text.substr(slash + 1);
}

ct::String AssetImport::stemOf(const ct::String& name)
{
    const std::size_t dot = name.rfind('.');
    if (dot == ct::String::npos || dot == 0) return name;
    return name.substr(0, dot);
}

ct::String AssetImport::joinPath(const ct::String& folder, const ct::String& name)
{
    if (folder.empty()) return name;
    if (name.empty()) return folder;
    return folder.ends_with("/") ? folder + name : folder + "/" + name;
}

ct::String AssetImport::decodeUri(const ct::String& uri)
{
    ct::String result;
    for (std::size_t i = 0; i < uri.size(); ++i)
    {
        if (uri[i] == '%' && i + 2 < uri.size() && hexValue(uri[i + 1]) >= 0 &&
                hexValue(uri[i + 2]) >= 0)
        {
            result.push_back(static_cast<char>(hexValue(uri[i + 1]) * 16 + hexValue(uri[i + 2])));
            i += 2;
        }
        else
            result.push_back(uri[i]);
    }
    return result;
}

bool AssetImport::safeRelative(const ct::String& path)
{
    const ct::String text = slashed(path);
    if (text.empty() || text[0] == '/' || text.contains(':')) return false;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= text.size(); ++i)
    {
        if (i != text.size() && text[i] != '/') continue;
        const ct::String part = text.substr(start, i - start);
        start = i + 1;
        if (part == "..") return false;
    }
    return true;
}

void AssetImport::gltfCompanions(const ct::String& gltfPath, ct::Vector<ct::String>& uris)
{
    const ct::String text = FileSystem::getSingleton().readText(gltfPath);
    if (text.empty()) return;
    ct::Json::Error error;
    const ct::Json document = ct::Json::parse(text, &error);
    if (error || !document.is_object()) return;
    collectUris(document, "buffers", uris);
    collectUris(document, "images", uris);
}

bool AssetImport::plan(const Project& project, const ct::Vector<ct::String>& sources,
        const ct::String& folder, ImportPlan& out)
{
    out = ImportPlan();
    if (!project.isOpen()) return false;

    for (std::size_t i = 0; i < sources.size(); ++i)
    {
        const ct::String source = absoluteOf(sources[i]);
        const ct::String name = nameOf(source);
        if (!isFile(source))
        {
            out.skipped.push_back(name + " is not a file");
            continue;
        }
        ct::String inside;
        if (project.relativePath(source, inside))
        {
            out.skipped.push_back(name + " is already in the project");
            continue;
        }

        ImportFile primary;
        primary.source = source;
        primary.relative = joinPath(folder, name);
        primary.destination = project.absolutePath(primary.relative);
        primary.kind = AssetTypes::classify(name, false);
        primary.exists =
                file_exists(primary.destination.c_str()) || dir_exists(primary.destination.c_str());
        primary.primary = true;
        if (alreadyPlanned(out, primary.destination)) continue;
        out.files.push_back(primary);

        if (AssetTypes::extensionOf(name) != "gltf") continue;
        ct::Vector<ct::String> uris;
        gltfCompanions(source, uris);
        const ct::String sourceFolder = parentOf(source);
        for (std::size_t u = 0; u < uris.size(); ++u)
        {
            const ct::String relativeUri = slashed(decodeUri(uris[u]));
            if (relativeUri.starts_with("data:")) continue;
            if (!safeRelative(relativeUri))
            {
                out.skipped.push_back(uris[u] + " is outside the model folder");
                continue;
            }
            const ct::String companionSource = joinPath(sourceFolder, relativeUri);
            if (!isFile(companionSource))
            {
                out.skipped.push_back(relativeUri + " (used by " + name + ") was not found");
                continue;
            }
            ImportFile companion;
            companion.source = companionSource;
            companion.relative = joinPath(folder, relativeUri);
            companion.destination = project.absolutePath(companion.relative);
            companion.kind = AssetTypes::classify(nameOf(relativeUri), false);
            companion.exists = file_exists(companion.destination.c_str());
            if (alreadyPlanned(out, companion.destination)) continue;
            out.files.push_back(companion);
        }
    }
    return !out.files.empty();
}

std::size_t AssetImport::copy(const ImportPlan& plan, bool overwrite,
        ct::Vector<ct::String>& failed)
{
    std::size_t copied = 0;
    for (std::size_t i = 0; i < plan.files.size(); ++i)
    {
        const ImportFile& file = plan.files[i];
        if (file.exists && !overwrite) continue;
        const ct::String folder = parentOf(file.destination);
        if (!folder.empty() && !fs_create_directory(folder.c_str()))
        {
            failed.push_back(file.relative);
            continue;
        }
        std::size_t size = 0;
        std::uint8_t* data = file_read(file.source.c_str(), &size);
        if (!data)
        {
            failed.push_back(file.relative);
            continue;
        }
        const bool written = file_write(file.destination.c_str(), data, size);
        fs_free(data);
        if (written) ++copied;
        else
            failed.push_back(file.relative);
    }
    return copied;
}

} // namespace prisma3d
