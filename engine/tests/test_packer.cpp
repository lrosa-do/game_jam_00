#include "Check.h"

#include "prisma3d/core/FilePack.h"

#include "platform.h"

#include <string.h>

using namespace prisma3d;

namespace
{

bool sameText(const ct::Vector<std::uint8_t>& bytes, const char* text)
{
    return bytes.size() == strlen(text) && memcmp(bytes.data(), text, bytes.size()) == 0;
}

} // namespace

int main()
{
    const ct::String root = ct::String(PRISMA3D_TEST_DIR) + "/packer";

    FilePack pack;
    CHECK(pack.open(root + "/tool.rpak", "tool key"));
    CHECK(pack.entryCount() == 5);
    CHECK(pack.exists("a.txt"));
    CHECK(pack.exists("sub/b.txt"));
    CHECK(pack.exists("sub/zeros.bin"));
    CHECK(pack.exists("extra/b.txt"));
    CHECK(pack.exists("extra/zeros.bin"));
    CHECK(sameText(pack.readBinary("a.txt"), "alpha file\n"));
    CHECK(sameText(pack.readBinary("extra/b.txt"), "nested b\n"));
    CHECK(pack.readBinary("sub/zeros.bin").size() == 3000);
    std::uint32_t zeros = 0;
    for (std::uint32_t i = 0; i < pack.entryCount(); ++i)
        if (pack.entryName(i) == "sub/zeros.bin") zeros = i;
    CHECK(pack.entrySizeRaw(zeros) == 3000 && pack.entrySizeStored(zeros) < 100);

    FilePack single;
    CHECK(single.open(root + "/single.rpak", ""));
    CHECK(single.entryCount() == 1);
    CHECK(single.exists("docs/a.txt"));
    CHECK(sameText(single.readBinary("docs/a.txt"), "alpha file\n"));

    std::size_t sourceSize = 0;
    char* source = file_read_text((root + "/tool_pack.cpp").c_str());
    CHECK(source != nullptr);
    if (source)
    {
        sourceSize = strlen(source);
        CHECK(sourceSize > 0);
        CHECK(strstr(source, "namespace prisma3d") != nullptr);
        CHECK(strstr(source, "kDefaultPackData[] = {") != nullptr);
        CHECK(strstr(source, "kDefaultPackKey = \"\\x74\\x6f\\x6f\\x6c\\x20\\x6b\\x65\\x79\"") !=
                nullptr);
        char expected[64];
        snprintf(expected, sizeof(expected), "kDefaultPackSize = %lluull;",
                static_cast<unsigned long long>(file_size((root + "/tool.rpak").c_str())));
        CHECK(strstr(source, expected) != nullptr);
        fs_free(source);
    }

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
