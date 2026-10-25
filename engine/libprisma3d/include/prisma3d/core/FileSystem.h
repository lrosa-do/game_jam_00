#pragma once

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/thread.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

struct IoStream;

namespace prisma3d
{

class Archive;

class FileSystem
{
public:
    static constexpr const char* kUserPrefix = "user://";

    FileSystem();
    ~FileSystem();
    FileSystem(const FileSystem&) = delete;
    FileSystem& operator=(const FileSystem&) = delete;

    void addSearchPath(const ct::String& path);
    void removeSearchPath(const ct::String& path);
    const ct::Vector<ct::String>& getSearchPaths() const;

    bool mountArchive(const ct::String& zipPath);
    bool mountPack(const ct::String& packPath, const ct::String& key, std::uint64_t offset = 0);
    bool mountFallbackPack(const std::uint8_t* data, std::size_t size, const ct::String& key);
    void unmountAll();

    void setUserDirectory(const ct::String& path);
    const ct::String& getUserDirectory() const;
    ct::String globalize(const ct::String& name) const;

    bool exists(const ct::String& name) const;
    ct::String resolve(const ct::String& name) const;

    IoStream* open(const ct::String& name) const;
    ct::Vector<std::uint8_t> readBinary(const ct::String& name) const;
    ct::String readText(const ct::String& name) const;

    static bool writeText(const ct::String& path, const ct::String& text);
    static bool writeBinary(const ct::String& path, const void* data, std::size_t size);
    static ct::String normalizeSeparators(const ct::String& path);
    static ct::String simplifyPath(const ct::String& path);

    static FileSystem& getSingleton();

private:
    ct::Vector<std::uint8_t> readBinaryOnDisk(const ct::String& name) const;
    const Archive* findInArchives(const ct::Vector<Archive*>& archives, const ct::String& name,
            ct::String& entry) const;

    ct::Vector<ct::String> mSearchPaths;
    ct::String mUserDirectory;
    ct::Vector<Archive*> mArchives;
    ct::Vector<Archive*> mFallbacks;
    mutable ct::Mutex mResolveMutex;
    mutable ct::HashMap<ct::String, ct::String> mResolved;
};

} // namespace prisma3d
