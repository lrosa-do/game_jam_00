#pragma once

#include "prisma3d/core/Archive.h"

#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/thread.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

struct IoStream;

namespace prisma3d
{

struct PackStamp
{
    std::uint64_t offset = 0;
    std::uint64_t size = 0;
    ct::String key;
};

class FilePack : public Archive
{
public:
    static constexpr std::uint32_t kVersion = 1;
    static constexpr std::uint32_t kStampVersion = 1;
    static constexpr std::size_t kStampSize = 32;
    static constexpr const char* kSidePackName = "data.rpak";
    static constexpr std::size_t kSaltSize = 16;
    static constexpr std::size_t kHeaderSize = 64;
    static constexpr std::size_t kTocEntrySize = 32;

    enum PackFlags : std::uint32_t
    {
        kPackEncrypted = 1u << 0
    };

    enum EntryFlags : std::uint16_t
    {
        kEntryDeflated = 1u << 0
    };

    FilePack();
    ~FilePack() override;

    FilePack(const FilePack&) = delete;
    FilePack& operator=(const FilePack&) = delete;

    bool open(const ct::String& path, const ct::String& key, std::uint64_t offset = 0);
    bool openFromMemory(const std::uint8_t* data, std::size_t size, const ct::String& key);
    void close();
    bool isOpen() const;

    static bool readStamp(const ct::String& executable, PackStamp& out);
    static bool writeStamp(const ct::String& executable, const ct::String& pack,
            const ct::String& key, const ct::String& output);
    static ct::String sidePackPath(const ct::String& executable);

    bool exists(const ct::String& name) const override;
    ct::Vector<std::uint8_t> readBinary(const ct::String& name) const override;
    IoStream* open(const ct::String& name) const override;

    std::uint32_t entryCount() const;
    const ct::String& entryName(std::uint32_t index) const;
    std::uint32_t entrySizeRaw(std::uint32_t index) const;
    std::uint32_t entrySizeStored(std::uint32_t index) const;

private:
    struct Entry
    {
        ct::String name;
        std::uint64_t dataOffset = 0;
        std::uint32_t sizeStored = 0;
        std::uint32_t sizeRaw = 0;
        std::uint32_t crc = 0;
        std::uint16_t flags = 0;
    };

    bool readDirectory(const char* label, const ct::String& key);
    bool readAt(std::uint64_t offset, void* destination, std::size_t size) const;
    bool readEntry(std::uint32_t index, ct::Vector<std::uint8_t>& raw) const;
    const std::uint32_t* findEntry(const ct::String& name) const;
    void dropCached() const;

    IoStream* mFile;
    std::uint64_t mBase;
    const std::uint8_t* mMemory;
    std::size_t mMemorySize;
    ct::Vector<Entry> mEntries;
    ct::HashMap<ct::String, std::uint32_t> mIndex;
    std::uint8_t mKey[32];
    bool mEncrypted;
    mutable ct::Mutex mReadMutex;
    // The last large entry opened stays decoded: a model read range by range opens its file once
    // per range, which would inflate the whole entry every time.
    mutable ct::Mutex mCacheMutex;
    mutable std::uint8_t* mCached;
    mutable std::uint32_t mCachedIndex;
};

class FilePackWriter
{
public:
    FilePackWriter();
    ~FilePackWriter();

    FilePackWriter(const FilePackWriter&) = delete;
    FilePackWriter& operator=(const FilePackWriter&) = delete;

    void setKey(const ct::String& key);
    void setCompressionLevel(int level);

    bool addFile(const ct::String& name, const ct::String& path);
    bool addData(const ct::String& name, const std::uint8_t* data, std::size_t size);

    bool write(const ct::String& path);

    std::uint32_t entryCount() const;
    std::uint64_t rawBytes() const;
    std::uint64_t storedBytes() const;

private:
    struct Pending
    {
        ct::String name;
        ct::Vector<std::uint8_t> stored;
        std::uint32_t sizeRaw = 0;
        std::uint32_t crc = 0;
        std::uint16_t flags = 0;
    };

    ct::Vector<Pending> mPending;
    ct::HashMap<ct::String, std::uint32_t> mIndex;
    ct::String mKey;
    int mLevel;
    std::uint64_t mRawBytes;
    std::uint64_t mStoredBytes;
};

} // namespace prisma3d
