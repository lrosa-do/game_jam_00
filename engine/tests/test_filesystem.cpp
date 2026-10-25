#include "Check.h"

#include "prisma3d/core/FileSystem.h"

#include "platform.h"

#include <string.h>

using prisma3d::FileSystem;

static bool writeText(const ct::String& path, const char* text)
{
    return file_write(path.c_str(), text, strlen(text));
}

int main()
{
    const ct::String root = ct::String(PRISMA3D_TEST_DIR) + "/filesystem";
    const ct::String first = root + "/first";
    const ct::String second = root + "/second";
    CHECK(fs_create_directory((first + "/models").c_str()));
    CHECK(fs_create_directory(second.c_str()));
    CHECK(writeText(first + "/shared.txt", "from first"));
    CHECK(writeText(second + "/shared.txt", "from second"));
    CHECK(writeText(second + "/only.txt", "only in second"));
    CHECK(writeText(first + "/models/wall.txt", "wall"));
    CHECK(file_write((first + "/empty.bin").c_str(), "", 0));

    {
        FileSystem files;
        CHECK(files.getSearchPaths().empty());
        CHECK(!files.exists("shared.txt"));
        CHECK(files.resolve("shared.txt").empty());
        CHECK(files.open("shared.txt") == nullptr);
        CHECK(files.readBinary("shared.txt").empty());
        CHECK(files.readText("shared.txt").empty());

        CHECK(files.exists(first + "/shared.txt"));
        CHECK(files.resolve(first + "/shared.txt") == first + "/shared.txt");
        CHECK(files.readText(first + "/shared.txt") == "from first");
    }

    {
        FileSystem files;
        files.addSearchPath(first);
        files.addSearchPath(second);
        CHECK(files.getSearchPaths().size() == 2);

        CHECK(files.exists("shared.txt"));
        CHECK(files.resolve("shared.txt") == first + "/shared.txt");
        CHECK(files.readText("shared.txt") == "from first");

        CHECK(files.resolve("only.txt") == second + "/only.txt");
        CHECK(files.readText("only.txt") == "only in second");

        CHECK(files.resolve("models\\wall.txt") == first + "/models/wall.txt");
        CHECK(files.readText("models\\wall.txt") == "wall");
        CHECK(files.readText("models/wall.txt") == "wall");

        CHECK(!files.exists("missing.txt"));
        CHECK(!files.exists("models"));
        CHECK(files.readBinary("empty.bin").empty());

        const ct::Vector<std::uint8_t> bytes = files.readBinary("only.txt");
        CHECK(bytes.size() == strlen("only in second"));
        CHECK(bytes.size() == 14 && memcmp(bytes.data(), "only in second", 14) == 0);

        IoStream* stream = files.open("only.txt");
        CHECK(stream != nullptr);
        if (stream)
        {
            char text[5] = {};
            CHECK(io_size(stream) == 14);
            CHECK(io_read(stream, text, 4) == 4);
            CHECK(memcmp(text, "only", 4) == 0);
            CHECK(io_close(stream));
        }

        files.removeSearchPath(first);
        CHECK(files.getSearchPaths().size() == 1);
        CHECK(files.resolve("shared.txt") == second + "/shared.txt");
        CHECK(files.readText("shared.txt") == "from second");
        CHECK(!files.exists("models/wall.txt"));

        files.removeSearchPath(second);
        CHECK(files.getSearchPaths().empty());
        CHECK(!files.exists("shared.txt"));
    }

    CHECK(FileSystem::normalizeSeparators("a\\b/c\\d.png") == "a/b/c/d.png");
    CHECK(&FileSystem::getSingleton() == &FileSystem::getSingleton());

    if (failures)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
