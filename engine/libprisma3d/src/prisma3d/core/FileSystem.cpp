#include "prisma3d/core/FileSystem.h"

#include "ZipArchive.h"

#include "prisma3d/core/FilePack.h"

#include "platform.h"

#include <string.h>

namespace prisma3d
{

FileSystem::FileSystem()
{
}

FileSystem::~FileSystem()
{
    unmountAll();
}

void FileSystem::unmountAll()
{
    for (Archive* archive: mArchives) delete archive;
    mArchives.clear();

    for (Archive* archive: mFallbacks) delete archive;
    mFallbacks.clear();
}

bool FileSystem::mountPack(const ct::String& packPath, const ct::String& key, std::uint64_t offset)
{
    const ct::String path = resolve(packPath);
    if (path.empty())
    {
        log_error("FileSystem: pack not found: %s", packPath.c_str());
        return false;
    }

    FilePack* pack = new FilePack();
    if (!pack->open(path, key, offset))
    {
        delete pack;
        return false;
    }

    mArchives.push_back(pack);
    return true;
}

bool FileSystem::mountFallbackPack(const std::uint8_t* data, std::size_t size,
        const ct::String& key)
{
    FilePack* pack = new FilePack();
    if (!pack->openFromMemory(data, size, key))
    {
        delete pack;
        return false;
    }

    mFallbacks.push_back(pack);
    return true;
}

bool FileSystem::mountArchive(const ct::String& zipPath)
{
    ct::Vector<std::uint8_t> data = readBinaryOnDisk(zipPath);
    if (data.empty())
    {
        log_error("FileSystem: could not read archive %s", zipPath.c_str());
        return false;
    }

    ZipArchive* archive = new ZipArchive();
    if (!archive->openFromMemory(static_cast<ct::Vector<std::uint8_t>&&>(data)))
    {
        log_error("FileSystem: failed to open archive %s", zipPath.c_str());
        delete archive;
        return false;
    }

    mArchives.push_back(archive);
    return true;
}

void FileSystem::addSearchPath(const ct::String& path)
{
    mSearchPaths.push_back(path);
}

void FileSystem::removeSearchPath(const ct::String& path)
{
    for (std::size_t i = 0; i < mSearchPaths.size();)
    {
        if (mSearchPaths[i] == path)
            mSearchPaths.erase(mSearchPaths.begin() + static_cast<std::ptrdiff_t>(i));
        else
            ++i;
    }
    ct::LockGuard lock(mResolveMutex);
    mResolved.clear();
}

const ct::Vector<ct::String>& FileSystem::getSearchPaths() const
{
    return mSearchPaths;
}

void FileSystem::setUserDirectory(const ct::String& path)
{
    mUserDirectory = path;
    ct::LockGuard lock(mResolveMutex);
    mResolved.clear();
}

const ct::String& FileSystem::getUserDirectory() const
{
    return mUserDirectory;
}

ct::String FileSystem::globalize(const ct::String& name) const
{
    const ct::String normalized = normalizeSeparators(name);
    if (!normalized.starts_with(kUserPrefix)) return normalized;
    if (mUserDirectory.empty()) return ct::String();
    return mUserDirectory + "/" + normalized.substr(strlen(kUserPrefix));
}

ct::String FileSystem::resolve(const ct::String& name) const
{
    {
        ct::LockGuard lock(mResolveMutex);
        if (const ct::String* cached = mResolved.find(name)) return *cached;
    }

    const ct::String normalized = normalizeSeparators(name);

    if (normalized.starts_with(kUserPrefix))
    {
        const ct::String path = globalize(normalized);
        if (path.empty() || !file_exists(path.c_str())) return ct::String();
        ct::LockGuard lock(mResolveMutex);
        mResolved[name] = path;
        return path;
    }

    if (file_exists(normalized.c_str()))
    {
        ct::LockGuard lock(mResolveMutex);
        mResolved[name] = normalized;
        return normalized;
    }

    for (const ct::String& directory: mSearchPaths)
    {
        const ct::String path = directory.empty() ? normalized : directory + "/" + normalized;
        if (file_exists(path.c_str()))
        {
            ct::LockGuard lock(mResolveMutex);
            mResolved[name] = path;
            return path;
        }
    }

    return ct::String();
}

const Archive* FileSystem::findInArchives(const ct::Vector<Archive*>& archives,
        const ct::String& name, ct::String& entry) const
{
    if (archives.empty()) return nullptr;
    for (const Archive* archive: archives)
    {
        if (archive->exists(name))
        {
            entry = name;
            return archive;
        }
    }
    for (const ct::String& directory: mSearchPaths)
    {
        if (directory.empty()) continue;
        const ct::String path = directory + "/" + name;
        for (const Archive* archive: archives)
        {
            if (archive->exists(path))
            {
                entry = path;
                return archive;
            }
        }
    }
    return nullptr;
}

bool FileSystem::exists(const ct::String& name) const
{
    ct::String entry;
    if (findInArchives(mArchives, name, entry)) return true;
    if (!resolve(name).empty()) return true;
    return findInArchives(mFallbacks, name, entry) != nullptr;
}

IoStream* FileSystem::open(const ct::String& name) const
{
    ct::String entry;
    if (const Archive* archive = findInArchives(mArchives, name, entry))
        return archive->open(entry);

    const ct::String path = resolve(name);
    if (!path.empty())
    {
        IoStream* stream = io_open_file(path.c_str(), "rb");
        if (!stream) log_error("FileSystem: failed to open %s", path.c_str());
        return stream;
    }

    if (const Archive* archive = findInArchives(mFallbacks, name, entry))
        return archive->open(entry);

    log_error("FileSystem: file not found: %s", name.c_str());
    return nullptr;
}

ct::Vector<std::uint8_t> FileSystem::readBinary(const ct::String& name) const
{
    ct::String entry;
    if (const Archive* archive = findInArchives(mArchives, name, entry))
        return archive->readBinary(entry);

    ct::Vector<std::uint8_t> onDisk = readBinaryOnDisk(name);
    if (!onDisk.empty()) return onDisk;

    if (const Archive* archive = findInArchives(mFallbacks, name, entry))
        return archive->readBinary(entry);
    return onDisk;
}

ct::Vector<std::uint8_t> FileSystem::readBinaryOnDisk(const ct::String& name) const
{
    ct::Vector<std::uint8_t> data;
    const ct::String path = resolve(name);
    if (path.empty()) return data;

    IoStream* stream = io_open_file(path.c_str(), "rb");
    if (!stream)
    {
        log_error("FileSystem: failed to open %s", path.c_str());
        return data;
    }

    const std::int64_t size = io_size(stream);
    if (size <= 0)
    {
        io_close(stream);
        log_error("FileSystem: invalid file size for %s", name.c_str());
        return data;
    }

    data.resize(static_cast<std::size_t>(size));
    const std::size_t bytesRead = io_read(stream, data.data(), data.size());
    io_close(stream);

    if (bytesRead != data.size())
    {
        log_error("FileSystem: short read on %s", name.c_str());
        data.clear();
    }

    return data;
}

ct::String FileSystem::readText(const ct::String& name) const
{
    const ct::Vector<std::uint8_t> data = readBinary(name);
    if (data.empty()) return ct::String();
    return ct::String(reinterpret_cast<const char*>(data.data()), data.size());
}

bool FileSystem::writeText(const ct::String& path, const ct::String& text)
{
    return writeBinary(path, text.data(), text.size());
}

bool FileSystem::writeBinary(const ct::String& name, const void* data, std::size_t size)
{
    const ct::String path = getSingleton().globalize(name);
    if (path.empty()) return false;
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') slash = i;
    if (slash != ct::String::npos && slash != 0)
    {
        const ct::String directory(path.c_str(), slash);
        if (!fs_create_directory(directory.c_str())) return false;
    }
    const ct::String temporary = path + ".tmp";
    fs_remove_path(temporary.c_str());
    if (!file_write(temporary.c_str(), data, size)) return false;
    if (!fs_rename_path(temporary.c_str(), path.c_str()))
    {
        fs_remove_path(temporary.c_str());
        return false;
    }
    return true;
}

ct::String FileSystem::normalizeSeparators(const ct::String& path)
{
    ct::String result(path);
    char* data = result.data();
    for (std::size_t i = 0; i < result.size(); ++i)
        if (data[i] == '\\') data[i] = '/';
    return result;
}

ct::String FileSystem::simplifyPath(const ct::String& path)
{
    const ct::String unified = normalizeSeparators(path);
    ct::Vector<ct::String> kept;
    ct::String current;
    for (std::size_t i = 0; i <= unified.size(); ++i)
    {
        if (i < unified.size() && unified[i] != '/')
        {
            current += unified[i];
            continue;
        }
        if (!current.empty() && current != ".")
        {
            if (current == ".." && !kept.empty() && kept.back() != "..") kept.pop_back();
            else
                kept.push_back(current);
        }
        current = ct::String();
    }
    ct::String result;
    if (!unified.empty() && unified[0] == '/') result += "/";
    for (std::size_t i = 0; i < kept.size(); ++i)
    {
        if (i > 0) result += "/";
        result += kept[i];
    }
    return result;
}

FileSystem& FileSystem::getSingleton()
{
    static FileSystem instance;
    return instance;
}

} // namespace prisma3d
