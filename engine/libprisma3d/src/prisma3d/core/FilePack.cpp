#include "prisma3d/core/FilePack.h"

#include "prisma3d/core/ChaCha20.h"

#include "miniz.h"
#include "platform.h"

#include <atomic>
#include <new>

#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const char kMagic[4] = { 'R', 'P', 'A', 'K' };
const char kStampMagic[8] = { 'P', 'R', 'S', 'M', 'P', 'A', 'C', 'K' };

// Entries at least this big are kept decoded after they are opened.
const std::size_t kCacheMinBytes = 1u << 20;

// A decoded entry shared by the streams that read it and the cache: the reference count sits in
// front of the bytes, so the release callback of a stream finds it from the pointer it is given.
struct alignas(16) SharedHeader
{
    std::atomic<int> references;
};

std::uint8_t* allocateShared(std::size_t size)
{
    void* block = malloc(sizeof(SharedHeader) + (size > 0 ? size : 1));
    if (!block) return nullptr;
    SharedHeader* header = new (block) SharedHeader();
    header->references.store(1);
    return static_cast<std::uint8_t*>(block) + sizeof(SharedHeader);
}

SharedHeader* sharedHeader(void* bytes)
{
    return reinterpret_cast<SharedHeader*>(
            static_cast<std::uint8_t*>(bytes) - sizeof(SharedHeader));
}

void retainShared(std::uint8_t* bytes)
{
    sharedHeader(bytes)->references.fetch_add(1);
}

void releaseShared(void* bytes)
{
    SharedHeader* header = sharedHeader(bytes);
    if (header->references.fetch_sub(1) != 1) return;
    header->~SharedHeader();
    free(header);
}

class OptionalLock
{
public:
    OptionalLock(ct::Mutex& mutex, bool locked)
            : mMutex(locked ? &mutex : nullptr)
    {
        if (mMutex) mMutex->lock();
    }
    ~OptionalLock()
    {
        if (mMutex) mMutex->unlock();
    }
    OptionalLock(const OptionalLock&) = delete;
    OptionalLock& operator=(const OptionalLock&) = delete;

private:
    ct::Mutex* mMutex;
};

void directoryNonce(std::uint8_t nonce[ChaCha20::kNonceSize])
{
    memset(nonce, 0xff, ChaCha20::kNonceSize);
}

void entryNonce(std::uint8_t nonce[ChaCha20::kNonceSize], std::uint32_t index)
{
    memset(nonce, 0, ChaCha20::kNonceSize);
    nonce[0] = static_cast<std::uint8_t>(index);
    nonce[1] = static_cast<std::uint8_t>(index >> 8);
    nonce[2] = static_cast<std::uint8_t>(index >> 16);
    nonce[3] = static_cast<std::uint8_t>(index >> 24);
}

void cipher(const std::uint8_t key[ChaCha20::kKeySize],
        const std::uint8_t nonce[ChaCha20::kNonceSize], std::uint8_t* data, std::size_t size)
{
    ChaCha20 stream;
    stream.setKey(key);
    stream.setNonce(nonce, 0);
    stream.process(data, size);
}

std::uint32_t keyChecksum(const std::uint8_t key[ChaCha20::kKeySize])
{
    return static_cast<std::uint32_t>(mz_crc32(MZ_CRC32_INIT, key, ChaCha20::kKeySize));
}

ct::String normalize(const ct::String& name)
{
    ct::String out(name);
    char* data = out.data();
    for (std::size_t i = 0; i < out.size(); ++i)
        if (data[i] == '\\') data[i] = '/';
    return out;
}

const ct::String kEmptyName;

class Reader
{
public:
    explicit Reader(const std::uint8_t* data)
            : mData(data),
              mCursor(0)
    {
    }

    void readBytes(void* out, std::size_t count)
    {
        memcpy(out, mData + mCursor, count);
        mCursor += count;
    }

    std::uint16_t readU16()
    {
        std::uint16_t value = 0;
        value |= static_cast<std::uint16_t>(mData[mCursor]);
        value |= static_cast<std::uint16_t>(mData[mCursor + 1]) << 8;
        mCursor += 2;
        return value;
    }

    std::uint32_t readU32()
    {
        const std::uint32_t low = readU16();
        const std::uint32_t high = readU16();
        return low | (high << 16);
    }

    std::uint64_t readU64()
    {
        const std::uint64_t low = readU32();
        const std::uint64_t high = readU32();
        return low | (high << 32);
    }

private:
    const std::uint8_t* mData;
    std::size_t mCursor;
};

class Writer
{
public:
    explicit Writer(ct::Vector<std::uint8_t>& out)
            : mOut(out)
    {
    }

    void writeBytes(const void* data, std::size_t count)
    {
        const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);
        for (std::size_t i = 0; i < count; ++i) mOut.push_back(bytes[i]);
    }

    void writeU16(std::uint16_t value)
    {
        mOut.push_back(static_cast<std::uint8_t>(value));
        mOut.push_back(static_cast<std::uint8_t>(value >> 8));
    }

    void writeU32(std::uint32_t value)
    {
        writeU16(static_cast<std::uint16_t>(value));
        writeU16(static_cast<std::uint16_t>(value >> 16));
    }

    void writeU64(std::uint64_t value)
    {
        writeU32(static_cast<std::uint32_t>(value));
        writeU32(static_cast<std::uint32_t>(value >> 32));
    }

private:
    ct::Vector<std::uint8_t>& mOut;
};

void fillSalt(std::uint8_t salt[FilePack::kSaltSize])
{
    IoStream* source = io_open_file("/dev/urandom", "rb");
    if (source)
    {
        const bool ok = io_read(source, salt, FilePack::kSaltSize) == FilePack::kSaltSize;
        io_close(source);
        if (ok) return;
    }

    std::uint8_t seed[ChaCha20::kKeySize];
    memset(seed, 0, sizeof(seed));
    const std::uint64_t now = time_nanos();
    const std::uint64_t address = reinterpret_cast<std::uintptr_t>(salt);
    memcpy(seed, &now, sizeof(now));
    memcpy(seed + 8, &address, sizeof(address));
    std::uint8_t nonce[ChaCha20::kNonceSize];
    memset(nonce, 0x5a, sizeof(nonce));
    ChaCha20 stream;
    stream.setKey(seed);
    stream.setNonce(nonce, static_cast<std::uint32_t>(now >> 32));
    memset(salt, 0, FilePack::kSaltSize);
    stream.process(salt, FilePack::kSaltSize);
}

} // namespace

FilePack::FilePack()
        : mFile(nullptr),
          mBase(0),
          mMemory(nullptr),
          mMemorySize(0),
          mEncrypted(false),
          mCached(nullptr),
          mCachedIndex(0)
{
    memset(mKey, 0, sizeof(mKey));
}

FilePack::~FilePack()
{
    close();
}

void FilePack::dropCached() const
{
    ct::LockGuard lock(mCacheMutex);
    if (mCached) releaseShared(mCached);
    mCached = nullptr;
}

void FilePack::close()
{
    dropCached();
    if (mFile)
    {
        io_close(mFile);
        mFile = nullptr;
    }
    mBase = 0;
    mMemory = nullptr;
    mMemorySize = 0;
    mEntries.clear();
    mIndex.clear();
    memset(mKey, 0, sizeof(mKey));
    mEncrypted = false;
}

bool FilePack::isOpen() const
{
    return mFile != nullptr || mMemory != nullptr;
}

bool FilePack::readAt(std::uint64_t offset, void* destination, std::size_t size) const
{
    if (size == 0) return true;

    if (mMemory)
    {
        if (offset > mMemorySize || size > mMemorySize - offset) return false;
        memcpy(destination, mMemory + offset, size);
        return true;
    }

    if (!mFile) return false;
    ct::LockGuard lock(mReadMutex);
    if (io_seek(mFile, static_cast<std::int64_t>(mBase + offset), IO_SEEK_SET) < 0) return false;
    return io_read(mFile, destination, size) == size;
}

bool FilePack::open(const ct::String& path, const ct::String& key, std::uint64_t offset)
{
    close();

    mFile = io_open_file(path.c_str(), "rb");
    if (!mFile)
    {
        log_error("FilePack: could not open %s", path.c_str());
        return false;
    }
    mBase = offset;

    return readDirectory(path.c_str(), key);
}

bool FilePack::openFromMemory(const std::uint8_t* data, std::size_t size, const ct::String& key)
{
    close();

    if (!data || size == 0)
    {
        log_error("FilePack: no bytes to open as a pack");
        return false;
    }

    mMemory = data;
    mMemorySize = size;

    return readDirectory("<memory>", key);
}

bool FilePack::readDirectory(const char* label, const ct::String& key)
{
    std::uint8_t header[kHeaderSize];
    if (!readAt(0, header, kHeaderSize))
    {
        log_error("FilePack: %s is too short to hold a header", label);
        close();
        return false;
    }

    Reader view(header);
    char magic[4];
    view.readBytes(magic, 4);
    if (memcmp(magic, kMagic, 4) != 0)
    {
        log_error("FilePack: %s is not a pack (bad magic)", label);
        close();
        return false;
    }

    const std::uint32_t version = view.readU32();
    if (version != kVersion)
    {
        log_error("FilePack: %s is version %u, this build reads version %u", label, version,
                kVersion);
        close();
        return false;
    }

    const std::uint32_t flags = view.readU32();
    const std::uint32_t entryCount = view.readU32();
    const std::uint64_t tocOffset = view.readU64();
    const std::uint32_t tocSize = view.readU32();
    const std::uint32_t namesSize = view.readU32();
    std::uint8_t salt[kSaltSize];
    view.readBytes(salt, kSaltSize);
    const std::uint32_t keyCheck = view.readU32();

    mEncrypted = (flags & kPackEncrypted) != 0;
    if (mEncrypted)
    {
        if (key.empty())
        {
            log_error("FilePack: %s is encrypted and no key was given", label);
            close();
            return false;
        }
        ChaCha20::deriveKey(key, salt, mKey);
        if (keyChecksum(mKey) != keyCheck)
        {
            log_error("FilePack: wrong key for %s", label);
            close();
            return false;
        }
    }
    else if (!key.empty())
    {
        log_warn("FilePack: %s carries no encryption; the key given is unused", label);
    }

    if (static_cast<std::uint64_t>(tocSize) !=
            static_cast<std::uint64_t>(entryCount) * kTocEntrySize)
    {
        log_error("FilePack: %s declares %u entries but a %u byte table", label, entryCount,
                tocSize);
        close();
        return false;
    }

    const std::size_t directorySize = static_cast<std::size_t>(tocSize) + namesSize;
    ct::Vector<std::uint8_t> directory;
    directory.resize(directorySize);
    if (!readAt(tocOffset, directory.data(), directorySize))
    {
        log_error("FilePack: could not read the directory of %s", label);
        close();
        return false;
    }

    if (mEncrypted)
    {
        std::uint8_t nonce[ChaCha20::kNonceSize];
        directoryNonce(nonce);
        cipher(mKey, nonce, directory.data(), directorySize);
    }

    const char* names = reinterpret_cast<const char*>(directory.data()) + tocSize;
    Reader table(directory.data());

    mEntries.reserve(entryCount);
    for (std::uint32_t i = 0; i < entryCount; ++i)
    {
        const std::uint32_t nameOffset = table.readU32();
        const std::uint32_t nameLength = table.readU32();
        Entry entry;
        entry.dataOffset = table.readU64();
        entry.sizeStored = table.readU32();
        entry.sizeRaw = table.readU32();
        entry.crc = table.readU32();
        entry.flags = table.readU16();
        table.readU16();

        if (static_cast<std::size_t>(nameOffset) + nameLength > namesSize)
        {
            log_error("FilePack: %s has an entry whose name falls outside the name blob", label);
            close();
            return false;
        }
        entry.name = ct::String(names + nameOffset, nameLength);

        mIndex[entry.name] = static_cast<std::uint32_t>(mEntries.size());
        mEntries.push_back(static_cast<Entry&&>(entry));
    }

    return true;
}

const std::uint32_t* FilePack::findEntry(const ct::String& name) const
{
    if (!isOpen()) return nullptr;
    return mIndex.find(normalize(name));
}

bool FilePack::exists(const ct::String& name) const
{
    return findEntry(name) != nullptr;
}

bool FilePack::readEntry(std::uint32_t index, ct::Vector<std::uint8_t>& raw) const
{
    const Entry& entry = mEntries[index];

    ct::Vector<std::uint8_t> stored;
    stored.resize(entry.sizeStored);
    if (!readAt(entry.dataOffset, stored.data(), entry.sizeStored))
    {
        log_error("FilePack: could not read '%s'", entry.name.c_str());
        return false;
    }

    if (mEncrypted)
    {
        std::uint8_t nonce[ChaCha20::kNonceSize];
        entryNonce(nonce, index);
        cipher(mKey, nonce, stored.data(), entry.sizeStored);
    }

    if (entry.flags & kEntryDeflated)
    {
        raw.resize(entry.sizeRaw);
        mz_ulong destinationSize = entry.sizeRaw;
        const int status = mz_uncompress(raw.data(), &destinationSize, stored.data(),
                static_cast<mz_ulong>(entry.sizeStored));
        if (status != MZ_OK || destinationSize != entry.sizeRaw)
        {
            log_error("FilePack: '%s' failed to inflate (%s)", entry.name.c_str(),
                    mz_error(status));
            raw.clear();
            return false;
        }
    }
    else
    {
        raw = static_cast<ct::Vector<std::uint8_t>&&>(stored);
    }

    const std::uint32_t crc =
            static_cast<std::uint32_t>(mz_crc32(MZ_CRC32_INIT, raw.data(), raw.size()));
    if (crc != entry.crc)
    {
        log_error("FilePack: '%s' failed its checksum", entry.name.c_str());
        raw.clear();
        return false;
    }

    return true;
}

ct::Vector<std::uint8_t> FilePack::readBinary(const ct::String& name) const
{
    ct::Vector<std::uint8_t> raw;
    const std::uint32_t* index = findEntry(name);
    if (!index) return raw;
    readEntry(*index, raw);
    return raw;
}

IoStream* FilePack::open(const ct::String& name) const
{
    const std::uint32_t* index = findEntry(name);
    if (!index) return nullptr;
    const std::size_t size = mEntries[*index].sizeRaw;
    const bool cacheable = size >= kCacheMinBytes;

    // Large entries decode under the cache lock, so threads opening the same file wait for one
    // decode instead of each inflating it.
    OptionalLock lock(mCacheMutex, cacheable);
    if (cacheable && mCached && mCachedIndex == *index)
    {
        retainShared(mCached);
        IoStream* stream = io_open_memory_owned(mCached, size, releaseShared);
        if (!stream) releaseShared(mCached);
        return stream;
    }

    ct::Vector<std::uint8_t> raw;
    if (!readEntry(*index, raw)) return nullptr;
    std::uint8_t* bytes = allocateShared(raw.size());
    if (!bytes) return nullptr;
    if (raw.size() > 0) memcpy(bytes, raw.data(), raw.size());
    if (cacheable)
    {
        if (mCached) releaseShared(mCached);
        retainShared(bytes);
        mCached = bytes;
        mCachedIndex = *index;
    }

    IoStream* stream = io_open_memory_owned(bytes, raw.size(), releaseShared);
    if (!stream) releaseShared(bytes);
    return stream;
}

std::uint32_t FilePack::entryCount() const
{
    return static_cast<std::uint32_t>(mEntries.size());
}

const ct::String& FilePack::entryName(std::uint32_t index) const
{
    if (index >= mEntries.size()) return kEmptyName;
    return mEntries[index].name;
}

std::uint32_t FilePack::entrySizeRaw(std::uint32_t index) const
{
    if (index >= mEntries.size()) return 0;
    return mEntries[index].sizeRaw;
}

std::uint32_t FilePack::entrySizeStored(std::uint32_t index) const
{
    if (index >= mEntries.size()) return 0;
    return mEntries[index].sizeStored;
}

bool FilePack::readStamp(const ct::String& executable, PackStamp& out)
{
    IoStream* file = io_open_file(executable.c_str(), "rb");
    if (!file)
    {
        log_error("FilePack: could not open %s", executable.c_str());
        return false;
    }

    const std::int64_t fileSize = io_size(file);
    std::uint8_t trailer[kStampSize];
    if (fileSize < static_cast<std::int64_t>(kStampSize) ||
            io_seek(file, fileSize - static_cast<std::int64_t>(kStampSize), IO_SEEK_SET) < 0 ||
            io_read(file, trailer, kStampSize) != kStampSize)
    {
        io_close(file);
        return false;
    }

    Reader view(trailer);
    char magic[8];
    view.readBytes(magic, 8);
    if (memcmp(magic, kStampMagic, 8) != 0)
    {
        io_close(file);
        return false;
    }

    const std::uint32_t version = view.readU32();
    const std::uint32_t keySize = view.readU32();
    const std::uint64_t offset = view.readU64();
    const std::uint64_t size = view.readU64();
    if (version != kStampVersion)
    {
        log_error("FilePack: %s carries stamp version %u, this build reads version %u",
                executable.c_str(), version, kStampVersion);
        io_close(file);
        return false;
    }

    const std::uint64_t total = static_cast<std::uint64_t>(fileSize);
    if (keySize > total - kStampSize || offset > total || size > total - offset ||
            offset + size + keySize + kStampSize != total)
    {
        log_error("FilePack: the stamp on %s does not match its size", executable.c_str());
        io_close(file);
        return false;
    }

    ct::String key;
    if (keySize > 0)
    {
        ct::Vector<char> bytes;
        bytes.resize(keySize);
        if (io_seek(file, static_cast<std::int64_t>(offset + size), IO_SEEK_SET) < 0 ||
                io_read(file, bytes.data(), keySize) != keySize)
        {
            log_error("FilePack: could not read the stamped key of %s", executable.c_str());
            io_close(file);
            return false;
        }
        key = ct::String(bytes.data(), keySize);
    }
    io_close(file);

    out.offset = offset;
    out.size = size;
    out.key = key;
    return true;
}

ct::String FilePack::sidePackPath(const ct::String& executable)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < executable.size(); ++i)
        if (executable[i] == '/' || executable[i] == '\\') slash = i;
    if (slash == ct::String::npos) return ct::String(kSidePackName);
    return executable.substr(0, slash + 1) + kSidePackName;
}

namespace
{

bool copyBytes(IoStream* from, IoStream* to, std::uint64_t count)
{
    std::uint8_t chunk[65536];
    while (count > 0)
    {
        const std::size_t step =
                count < sizeof(chunk) ? static_cast<std::size_t>(count) : sizeof(chunk);
        if (io_read(from, chunk, step) != step) return false;
        if (io_write(to, chunk, step) != step) return false;
        count -= step;
    }
    return true;
}

} // namespace

bool FilePack::writeStamp(const ct::String& executable, const ct::String& pack,
        const ct::String& key, const ct::String& output)
{
    if (output == executable)
    {
        log_error("FilePack: the stamped copy must not overwrite %s", executable.c_str());
        return false;
    }

    PackStamp existing;
    if (readStamp(executable, existing))
    {
        log_error("FilePack: %s already carries a pack", executable.c_str());
        return false;
    }

    if (!pack.empty())
    {
        FilePack probe;
        if (!probe.open(pack, key)) return false;
    }

    IoStream* source = io_open_file(executable.c_str(), "rb");
    if (!source)
    {
        log_error("FilePack: could not open %s", executable.c_str());
        return false;
    }
    IoStream* data = pack.empty() ? nullptr : io_open_file(pack.c_str(), "rb");
    if (!pack.empty() && !data)
    {
        log_error("FilePack: could not open %s", pack.c_str());
        io_close(source);
        return false;
    }
    IoStream* target = io_open_file(output.c_str(), "wb");
    if (!target)
    {
        log_error("FilePack: could not create %s", output.c_str());
        if (data) io_close(data);
        io_close(source);
        return false;
    }

    const std::int64_t executableSize = io_size(source);
    const std::int64_t packSize = data ? io_size(data) : 0;
    bool ok = executableSize >= 0 && packSize >= 0;
    ok = ok && copyBytes(source, target, static_cast<std::uint64_t>(executableSize));
    if (data) ok = ok && copyBytes(data, target, static_cast<std::uint64_t>(packSize));
    if (ok && !key.empty()) ok = io_write(target, key.data(), key.size()) == key.size();

    if (ok)
    {
        ct::Vector<std::uint8_t> trailer;
        trailer.reserve(kStampSize);
        Writer writer(trailer);
        writer.writeBytes(kStampMagic, 8);
        writer.writeU32(kStampVersion);
        writer.writeU32(static_cast<std::uint32_t>(key.size()));
        writer.writeU64(static_cast<std::uint64_t>(executableSize));
        writer.writeU64(static_cast<std::uint64_t>(packSize));
        ok = trailer.size() == kStampSize &&
             io_write(target, trailer.data(), kStampSize) == kStampSize;
    }

    const bool closed = io_close(target);
    if (data) io_close(data);
    io_close(source);
    ok = ok && closed;
    if (!ok)
    {
        log_error("FilePack: writing %s failed part way through", output.c_str());
        return false;
    }
    if (!fs_make_executable(output.c_str()))
    {
        log_error("FilePack: %s", platform_get_error());
        return false;
    }
    return true;
}

FilePackWriter::FilePackWriter()
        : mLevel(MZ_BEST_COMPRESSION),
          mRawBytes(0),
          mStoredBytes(0)
{
}

FilePackWriter::~FilePackWriter()
{
}

void FilePackWriter::setKey(const ct::String& key)
{
    mKey = key;
}

void FilePackWriter::setCompressionLevel(int level)
{
    if (level < 0) level = 0;
    if (level > MZ_BEST_COMPRESSION) level = MZ_BEST_COMPRESSION;
    mLevel = level;
}

bool FilePackWriter::addFile(const ct::String& name, const ct::String& path)
{
    std::size_t size = 0;
    std::uint8_t* data = file_read(path.c_str(), &size);
    if (!data)
    {
        if (file_exists(path.c_str()) && file_size(path.c_str()) == 0)
            return addData(name, nullptr, 0);
        log_error("FilePackWriter: could not read %s", path.c_str());
        return false;
    }

    const bool added = addData(name, data, size);
    fs_free(data);
    return added;
}

bool FilePackWriter::addData(const ct::String& name, const std::uint8_t* data, std::size_t size)
{
    const ct::String key = normalize(name);
    if (key.empty())
    {
        log_error("FilePackWriter: an entry cannot have an empty name");
        return false;
    }
    if (mIndex.contains(key))
    {
        log_error("FilePackWriter: '%s' was already added", key.c_str());
        return false;
    }
    if (size > 0xffffffffull)
    {
        log_error("FilePackWriter: '%s' is larger than 4 GiB", key.c_str());
        return false;
    }

    Pending pending;
    pending.name = key;
    pending.sizeRaw = static_cast<std::uint32_t>(size);
    pending.crc = static_cast<std::uint32_t>(mz_crc32(MZ_CRC32_INIT, data, size));
    pending.flags = 0;

    bool deflated = false;
    if (mLevel > 0 && size > 0)
    {
        const mz_ulong bound = mz_compressBound(static_cast<mz_ulong>(size));
        ct::Vector<std::uint8_t> compressed;
        compressed.resize(static_cast<std::size_t>(bound));
        mz_ulong compressedSize = bound;
        const int status = mz_compress2(compressed.data(), &compressedSize, data,
                static_cast<mz_ulong>(size), mLevel);
        if (status == MZ_OK && compressedSize < size)
        {
            compressed.resize(static_cast<std::size_t>(compressedSize));
            pending.stored = static_cast<ct::Vector<std::uint8_t>&&>(compressed);
            pending.flags |= FilePack::kEntryDeflated;
            deflated = true;
        }
    }

    if (!deflated)
    {
        pending.stored.resize(size);
        if (size > 0) memcpy(pending.stored.data(), data, size);
    }

    mRawBytes += pending.sizeRaw;
    mStoredBytes += pending.stored.size();
    mIndex[key] = static_cast<std::uint32_t>(mPending.size());
    mPending.push_back(static_cast<Pending&&>(pending));
    return true;
}

bool FilePackWriter::write(const ct::String& path)
{
    std::uint8_t salt[FilePack::kSaltSize];
    fillSalt(salt);

    const bool encrypted = !mKey.empty();
    std::uint8_t key[ChaCha20::kKeySize];
    memset(key, 0, sizeof(key));
    if (encrypted) ChaCha20::deriveKey(mKey, salt, key);

    ct::String names;
    ct::Vector<std::uint8_t> toc;
    toc.reserve(mPending.size() * FilePack::kTocEntrySize);
    Writer tocWriter(toc);
    std::uint64_t offset = FilePack::kHeaderSize;
    for (std::size_t i = 0; i < mPending.size(); ++i)
    {
        const Pending& pending = mPending[i];
        tocWriter.writeU32(static_cast<std::uint32_t>(names.size()));
        tocWriter.writeU32(static_cast<std::uint32_t>(pending.name.size()));
        tocWriter.writeU64(offset);
        tocWriter.writeU32(static_cast<std::uint32_t>(pending.stored.size()));
        tocWriter.writeU32(pending.sizeRaw);
        tocWriter.writeU32(pending.crc);
        tocWriter.writeU16(pending.flags);
        tocWriter.writeU16(0);
        names += pending.name;
        offset += pending.stored.size();
    }

    IoStream* file = io_open_file(path.c_str(), "wb");
    if (!file)
    {
        log_error("FilePackWriter: could not create %s", path.c_str());
        return false;
    }

    ct::Vector<std::uint8_t> header;
    header.reserve(FilePack::kHeaderSize);
    Writer headerWriter(header);
    headerWriter.writeBytes(kMagic, 4);
    headerWriter.writeU32(FilePack::kVersion);
    headerWriter.writeU32(encrypted ? static_cast<std::uint32_t>(FilePack::kPackEncrypted) : 0u);
    headerWriter.writeU32(static_cast<std::uint32_t>(mPending.size()));
    headerWriter.writeU64(offset);
    headerWriter.writeU32(static_cast<std::uint32_t>(toc.size()));
    headerWriter.writeU32(static_cast<std::uint32_t>(names.size()));
    headerWriter.writeBytes(salt, FilePack::kSaltSize);
    headerWriter.writeU32(encrypted ? keyChecksum(key) : 0u);
    headerWriter.writeU32(0);
    headerWriter.writeU32(0);
    headerWriter.writeU32(0);

    bool ok = header.size() == FilePack::kHeaderSize &&
              io_write(file, header.data(), FilePack::kHeaderSize) == FilePack::kHeaderSize;

    for (std::size_t i = 0; ok && i < mPending.size(); ++i)
    {
        ct::Vector<std::uint8_t>& stored = mPending[i].stored;
        if (encrypted && stored.size() > 0)
        {
            std::uint8_t nonce[ChaCha20::kNonceSize];
            entryNonce(nonce, static_cast<std::uint32_t>(i));
            cipher(key, nonce, stored.data(), stored.size());
        }
        if (stored.size() > 0) ok = io_write(file, stored.data(), stored.size()) == stored.size();
    }

    if (ok)
    {
        const std::size_t directorySize = toc.size() + names.size();
        ct::Vector<std::uint8_t> directory;
        directory.resize(directorySize);
        if (toc.size() > 0) memcpy(directory.data(), toc.data(), toc.size());
        if (names.size() > 0) memcpy(directory.data() + toc.size(), names.data(), names.size());
        if (encrypted && directorySize > 0)
        {
            std::uint8_t nonce[ChaCha20::kNonceSize];
            directoryNonce(nonce);
            cipher(key, nonce, directory.data(), directorySize);
        }
        if (directorySize > 0)
            ok = io_write(file, directory.data(), directorySize) == directorySize;
    }

    const bool closed = io_close(file);
    ok = ok && closed;

    if (!ok) log_error("FilePackWriter: writing %s failed part way through", path.c_str());
    return ok;
}

std::uint32_t FilePackWriter::entryCount() const
{
    return static_cast<std::uint32_t>(mPending.size());
}

std::uint64_t FilePackWriter::rawBytes() const
{
    return mRawBytes;
}

std::uint64_t FilePackWriter::storedBytes() const
{
    return mStoredBytes;
}

} // namespace prisma3d
