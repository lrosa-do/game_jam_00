#include "Check.h"

#include "prisma3d/core/FilePack.h"
#include "prisma3d/core/FileSystem.h"

#include "miniz.h"
#include "platform.h"

#include <ct/thread.hpp>

#include <string.h>

using namespace prisma3d;

namespace
{

const int kEntryCount = 6;
const int kThreadCount = 8;
const int kIterations = 60;

const char* const kNames[kEntryCount] = { "a.txt", "models/b.txt", "c.txt", "d.txt", "large300.bin",
    "large1m.bin" };

const std::size_t kSizes[kEntryCount] = { 0, 0, 0, 0, 300 * 1024, 1024 * 1024 };

void fillEntry(ct::Vector<std::uint8_t>& bytes, int index, const char* tag)
{
    if (kSizes[index] == 0)
    {
        const ct::String text = ct::String(tag) + ":" + kNames[index];
        bytes.resize(text.size());
        memcpy(bytes.data(), text.c_str(), text.size());
        return;
    }
    const std::size_t size = kSizes[index];
    const std::uint32_t seed = static_cast<std::uint32_t>(tag[0]) * 2654435761u +
                               static_cast<std::uint32_t>(index) * 40503u;
    bytes.resize(size);
    std::uint32_t state = seed;
    for (std::size_t i = 0; i < size; ++i)
    {
        state = state * 1664525u + 1013904223u;
        const std::uint8_t noise = static_cast<std::uint8_t>(state >> 24);
        bytes[i] = (i / 64) % 16 == 0 ? noise : static_cast<std::uint8_t>((i * 7 + seed) & 0xFF);
    }
}

struct Contents
{
    ct::Vector<std::uint8_t> entries[kEntryCount];

    void build(const char* tag)
    {
        for (int i = 0; i < kEntryCount; ++i) fillEntry(entries[i], i, tag);
    }
};

bool writeZip(const ct::String& path, const Contents& contents)
{
    mz_zip_archive zip;
    memset(&zip, 0, sizeof(zip));
    if (!mz_zip_writer_init_heap(&zip, 0, 0)) return false;
    bool ok = true;
    for (int i = 0; i < kEntryCount; ++i)
        ok = ok && mz_zip_writer_add_mem(&zip, kNames[i], contents.entries[i].data(),
                           contents.entries[i].size(), MZ_BEST_COMPRESSION);
    void* data = nullptr;
    std::size_t size = 0;
    ok = ok && mz_zip_writer_finalize_heap_archive(&zip, &data, &size);
    ok = ok && file_write(path.c_str(), data, size);
    mz_free(data);
    mz_zip_writer_end(&zip);
    return ok;
}

bool writePack(const ct::String& path, const Contents& contents)
{
    FilePackWriter writer;
    writer.setKey("threads");
    bool ok = true;
    for (int i = 0; i < kEntryCount; ++i)
        ok = ok &&
             writer.addData(kNames[i], contents.entries[i].data(), contents.entries[i].size());
    return ok && writer.write(path);
}

bool sameBytes(const ct::Vector<std::uint8_t>& a, const ct::Vector<std::uint8_t>& b)
{
    return a.size() == b.size() && memcmp(a.data(), b.data(), a.size()) == 0;
}

void readerLoop(const FileSystem& files, const Contents& contents, int threadIndex,
        ct::Atomic<int>& mismatches)
{
    for (int iteration = 0; iteration < kIterations; ++iteration)
    {
        const int index = (threadIndex + iteration) % kEntryCount;
        const ct::Vector<std::uint8_t>& expected = contents.entries[index];

        if (!files.exists(kNames[index])) mismatches.fetch_add(1);

        const ct::Vector<std::uint8_t> read = files.readBinary(kNames[index]);
        if (!sameBytes(read, expected)) mismatches.fetch_add(1);

        IoStream* stream = files.open(kNames[index]);
        if (!stream)
        {
            mismatches.fetch_add(1);
            continue;
        }
        std::size_t size = 0;
        void* all = io_load(stream, &size, true);
        if (!all || size != expected.size() || memcmp(all, expected.data(), size) != 0)
            mismatches.fetch_add(1);
        if (all) fs_free(all);
    }
}

int runThreads(const FileSystem& files, const Contents& contents)
{
    ct::Atomic<int> mismatches(0);
    {
        ct::Vector<ct::Thread> threads;
        for (int t = 0; t < kThreadCount; ++t)
            threads.push_back(ct::Thread([&files, &contents, &mismatches, t]()
                    { readerLoop(files, contents, t, mismatches); }));
        for (std::size_t t = 0; t < threads.size(); ++t) threads[t].join();
    }
    return mismatches.load();
}

void checkAllExist(const FileSystem& files)
{
    for (int i = 0; i < kEntryCount; ++i) CHECK(files.exists(kNames[i]));
    CHECK(!files.exists("nowhere.txt"));
}

} // namespace

int main()
{
    const ct::String root = ct::String(PRISMA3D_TEST_DIR) + "/archive_threads";
    CHECK(fs_create_directory(root.c_str()));

    Contents zipContents;
    zipContents.build("zip");
    Contents packContents;
    packContents.build("pack");

    CHECK(writeZip(root + "/data.zip", zipContents));
    CHECK(writePack(root + "/data.rpak", packContents));

    {
        FileSystem files;
        files.addSearchPath(root);
        CHECK(files.mountArchive("data.zip"));
        checkAllExist(files);
        CHECK(runThreads(files, zipContents) == 0);
        checkAllExist(files);
        files.unmountAll();
    }

    {
        FileSystem files;
        files.addSearchPath(root);
        CHECK(files.mountPack("data.rpak", "threads"));
        checkAllExist(files);
        CHECK(runThreads(files, packContents) == 0);
        checkAllExist(files);
        files.unmountAll();
    }

    {
        FileSystem files;
        files.addSearchPath(root);
        CHECK(files.mountArchive("data.zip"));
        CHECK(files.mountPack("data.rpak", "threads"));
        for (int i = 0; i < kEntryCount; ++i)
            CHECK(sameBytes(files.readBinary(kNames[i]), zipContents.entries[i]));
        CHECK(runThreads(files, zipContents) == 0);
        files.unmountAll();
    }

    if (failures)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
