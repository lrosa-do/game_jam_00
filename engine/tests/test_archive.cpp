#include "Check.h"

#include "prisma3d/core/FileSystem.h"

#include "miniz.h"
#include "platform.h"

#include <string.h>

using prisma3d::FileSystem;

static const std::size_t kLargeSize = 200000;

static void fillLarge(ct::Vector<std::uint8_t>& bytes)
{
    bytes.resize(kLargeSize);
    for (std::size_t i = 0; i < kLargeSize; ++i)
        bytes[i] = static_cast<std::uint8_t>((i * 7 + i / 251) & 0xFF);
}

static bool writeZip(const ct::String& path, const ct::Vector<std::uint8_t>& large)
{
    mz_zip_archive zip;
    memset(&zip, 0, sizeof(zip));
    if (!mz_zip_writer_init_heap(&zip, 0, 0)) return false;
    bool ok = true;
    ok = ok && mz_zip_writer_add_mem(&zip, "shared.txt", "from zip", 8, MZ_NO_COMPRESSION);
    ok = ok && mz_zip_writer_add_mem(&zip, "models/wall.txt", "zip wall", 8, MZ_BEST_COMPRESSION);
    ok = ok &&
         mz_zip_writer_add_mem(&zip, "large.bin", large.data(), large.size(), MZ_BEST_COMPRESSION);
    ok = ok && mz_zip_writer_add_mem(&zip, "empty.bin", "", 0, MZ_NO_COMPRESSION);
    ok = ok && mz_zip_writer_add_mem(&zip, "folder/", nullptr, 0, MZ_NO_COMPRESSION);
    void* data = nullptr;
    std::size_t size = 0;
    ok = ok && mz_zip_writer_finalize_heap_archive(&zip, &data, &size);
    ok = ok && file_write(path.c_str(), data, size);
    mz_free(data);
    mz_zip_writer_end(&zip);
    return ok;
}

int main()
{
    const ct::String root = ct::String(PRISMA3D_TEST_DIR) + "/archive";
    CHECK(fs_create_directory(root.c_str()));
    CHECK(file_write((root + "/shared.txt").c_str(), "from disk", 9));
    CHECK(file_write((root + "/disk_only.txt").c_str(), "disk only", 9));
    CHECK(file_write((root + "/garbage.zip").c_str(), "this is not a zip file", 22));

    ct::Vector<std::uint8_t> large;
    fillLarge(large);
    CHECK(writeZip(root + "/data.zip", large));

    {
        FileSystem files;
        files.addSearchPath(root);

        CHECK(!files.mountArchive("missing.zip"));
        CHECK(!files.mountArchive("garbage.zip"));
        CHECK(files.readText("shared.txt") == "from disk");

        CHECK(files.mountArchive("data.zip"));

        CHECK(files.exists("shared.txt"));
        CHECK(files.readText("shared.txt") == "from zip");

        CHECK(files.exists("models/wall.txt"));
        CHECK(files.exists("models\\wall.txt"));
        CHECK(files.readText("models\\wall.txt") == "zip wall");
        CHECK(files.readText("MODELS/Wall.TXT") == "zip wall");

        const ct::Vector<std::uint8_t> read = files.readBinary("large.bin");
        CHECK(read.size() == kLargeSize);
        CHECK(read.size() == kLargeSize && memcmp(read.data(), large.data(), kLargeSize) == 0);

        CHECK(files.exists("empty.bin"));
        CHECK(files.readBinary("empty.bin").empty());
        CHECK(files.readBinary("folder/").empty());

        CHECK(files.exists("disk_only.txt"));
        CHECK(files.readText("disk_only.txt") == "disk only");

        CHECK(!files.exists("nowhere.txt"));
        CHECK(files.readBinary("nowhere.txt").empty());

        CHECK(files.resolve("models/wall.txt").empty());

        IoStream* stream = files.open("models\\wall.txt");
        CHECK(stream != nullptr);
        if (stream)
        {
            char text[9] = {};
            CHECK(io_size(stream) == 8);
            CHECK(io_read(stream, text, 8) == 8);
            CHECK(memcmp(text, "zip wall", 8) == 0);
            CHECK(io_close(stream));
        }

        stream = files.open("large.bin");
        CHECK(stream != nullptr);
        if (stream)
        {
            std::size_t size = 0;
            void* all = io_load(stream, &size, true);
            CHECK(all != nullptr && size == kLargeSize);
            CHECK(all && size == kLargeSize && memcmp(all, large.data(), kLargeSize) == 0);
            fs_free(all);
        }

        stream = files.open("empty.bin");
        CHECK(stream != nullptr);
        if (stream)
        {
            CHECK(io_size(stream) == 0);
            CHECK(io_close(stream));
        }

        CHECK(files.open("folder/") == nullptr);

        stream = files.open("shared.txt");
        CHECK(stream != nullptr);
        if (stream)
        {
            char text[9] = {};
            CHECK(io_read(stream, text, 8) == 8);
            CHECK(memcmp(text, "from zip", 8) == 0);
            CHECK(io_close(stream));
        }

        stream = files.open("disk_only.txt");
        CHECK(stream != nullptr);
        if (stream)
        {
            CHECK(io_size(stream) == 9);
            CHECK(io_close(stream));
        }

        files.unmountAll();
        CHECK(files.readText("shared.txt") == "from disk");
        CHECK(!files.exists("models/wall.txt"));
        CHECK(!files.exists("large.bin"));
    }

    if (failures)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
