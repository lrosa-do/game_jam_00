#include "Check.h"

#include "prisma3d/core/ChaCha20.h"
#include "prisma3d/core/FilePack.h"
#include "prisma3d/core/FileSystem.h"

#include "platform.h"

#include <string.h>

using namespace prisma3d;

namespace
{

const std::size_t kLargeSize = 300000;

void fillLarge(ct::Vector<std::uint8_t>& bytes)
{
    bytes.resize(kLargeSize);
    for (std::size_t i = 0; i < kLargeSize; ++i)
        bytes[i] = static_cast<std::uint8_t>((i * 7 + i / 251) & 0xFF);
}

void fillNoise(ct::Vector<std::uint8_t>& bytes, std::size_t size)
{
    bytes.resize(size);
    std::uint32_t state = 0x12345678u;
    for (std::size_t i = 0; i < size; ++i)
    {
        state = state * 1664525u + 1013904223u;
        bytes[i] = static_cast<std::uint8_t>(state >> 24);
    }
}

bool sameBytes(const ct::Vector<std::uint8_t>& a, const std::uint8_t* b, std::size_t size)
{
    return a.size() == size && (size == 0 || memcmp(a.data(), b, size) == 0);
}

bool sameText(const ct::Vector<std::uint8_t>& a, const char* text)
{
    return sameBytes(a, reinterpret_cast<const std::uint8_t*>(text), strlen(text));
}

void test_chacha20_rfc8439()
{
    std::uint8_t key[ChaCha20::kKeySize];
    for (std::size_t i = 0; i < ChaCha20::kKeySize; ++i) key[i] = static_cast<std::uint8_t>(i);
    const std::uint8_t nonce[ChaCha20::kNonceSize] = { 0, 0, 0, 0, 0, 0, 0, 0x4a, 0, 0, 0, 0 };
    const char* text = "Ladies and Gentlemen of the class of '99: If I could offer you only one "
                       "tip for the future, sunscreen would be it.";
    const std::uint8_t expectedStart[16] = { 0x6e, 0x2e, 0x35, 0x9a, 0x25, 0x68, 0xf9, 0x80, 0x41,
        0xba, 0x07, 0x28, 0xdd, 0x0d, 0x69, 0x81 };
    const std::uint8_t expectedEnd[8] = { 0x8e, 0xed, 0xf2, 0x78, 0x5e, 0x42, 0x87, 0x4d };

    std::uint8_t data[114];
    memcpy(data, text, sizeof(data));
    ChaCha20 stream;
    stream.setKey(key);
    stream.setNonce(nonce, 1);
    stream.process(data, sizeof(data));
    CHECK(memcmp(data, expectedStart, sizeof(expectedStart)) == 0);
    CHECK(memcmp(data + 106, expectedEnd, sizeof(expectedEnd)) == 0);

    stream.setKey(key);
    stream.setNonce(nonce, 1);
    stream.process(data, sizeof(data));
    CHECK(memcmp(data, text, sizeof(data)) == 0);
}

void test_derive_key()
{
    std::uint8_t salt[ChaCha20::kSaltSize];
    for (std::size_t i = 0; i < ChaCha20::kSaltSize; ++i) salt[i] = static_cast<std::uint8_t>(i);
    std::uint8_t a[ChaCha20::kKeySize];
    std::uint8_t b[ChaCha20::kKeySize];
    std::uint8_t c[ChaCha20::kKeySize];
    ChaCha20::deriveKey("secret", salt, a);
    ChaCha20::deriveKey("secret", salt, b);
    CHECK(memcmp(a, b, ChaCha20::kKeySize) == 0);
    ChaCha20::deriveKey("secreu", salt, c);
    CHECK(memcmp(a, c, ChaCha20::kKeySize) != 0);
    salt[15] ^= 1;
    ChaCha20::deriveKey("secret", salt, c);
    CHECK(memcmp(a, c, ChaCha20::kKeySize) != 0);
}

void test_writer()
{
    FilePackWriter writer;
    CHECK(!writer.addData("", nullptr, 0));
    CHECK(writer.addData("a.txt", reinterpret_cast<const std::uint8_t*>("aaa"), 3));
    CHECK(!writer.addData("a.txt", reinterpret_cast<const std::uint8_t*>("bbb"), 3));
    CHECK(writer.addData("A.txt\\", nullptr, 0));
    CHECK(writer.entryCount() == 2);
    CHECK(!writer.addFile("missing.bin", "/nowhere/missing.bin"));
    CHECK(!writer.write("/nowhere/cannot.rpak"));
}

void checkPackContents(FilePack& pack, const ct::Vector<std::uint8_t>& large,
        const ct::Vector<std::uint8_t>& noise)
{
    CHECK(pack.isOpen());
    CHECK(pack.entryCount() == 5);
    CHECK(pack.entryName(0) == "shared.txt");
    CHECK(pack.entryName(1) == "models/wall.txt");
    CHECK(pack.entryName(2) == "large.bin");
    CHECK(pack.entryName(3) == "noise.bin");
    CHECK(pack.entryName(4) == "empty.bin");
    CHECK(pack.entryName(5).empty());
    CHECK(pack.entrySizeRaw(2) == kLargeSize);
    CHECK(pack.entrySizeStored(2) < kLargeSize);
    CHECK(pack.entrySizeStored(3) == noise.size());
    CHECK(pack.entrySizeRaw(4) == 0 && pack.entrySizeStored(4) == 0);
    CHECK(pack.entrySizeRaw(9) == 0 && pack.entrySizeStored(9) == 0);

    CHECK(pack.exists("shared.txt"));
    CHECK(pack.exists("models\\wall.txt"));
    CHECK(!pack.exists("MODELS/wall.txt"));
    CHECK(!pack.exists("nowhere.txt"));
    CHECK(sameText(pack.readBinary("shared.txt"), "from pack"));
    CHECK(sameText(pack.readBinary("models\\wall.txt"), "pack wall"));
    CHECK(sameBytes(pack.readBinary("large.bin"), large.data(), large.size()));
    CHECK(sameBytes(pack.readBinary("noise.bin"), noise.data(), noise.size()));
    CHECK(pack.exists("empty.bin"));
    CHECK(pack.readBinary("empty.bin").empty());
    CHECK(pack.readBinary("nowhere.txt").empty());

    IoStream* stream = pack.open("large.bin");
    CHECK(stream != nullptr);
    if (stream)
    {
        std::size_t size = 0;
        void* all = io_load(stream, &size, true);
        CHECK(all != nullptr && size == kLargeSize);
        CHECK(all && size == kLargeSize && memcmp(all, large.data(), kLargeSize) == 0);
        fs_free(all);
    }
    stream = pack.open("empty.bin");
    CHECK(stream != nullptr);
    if (stream)
    {
        CHECK(io_size(stream) == 0);
        CHECK(io_close(stream));
    }
    CHECK(pack.open("nowhere.txt") == nullptr);
}

bool writePack(const ct::String& path, const ct::String& key, int level,
        const ct::Vector<std::uint8_t>& large, const ct::Vector<std::uint8_t>& noise)
{
    FilePackWriter writer;
    writer.setKey(key);
    writer.setCompressionLevel(level);
    bool ok = true;
    ok = ok && writer.addData("shared.txt", reinterpret_cast<const std::uint8_t*>("from pack"), 9);
    ok = ok &&
         writer.addData("models\\wall.txt", reinterpret_cast<const std::uint8_t*>("pack wall"), 9);
    ok = ok && writer.addData("large.bin", large.data(), large.size());
    ok = ok && writer.addData("noise.bin", noise.data(), noise.size());
    ok = ok && writer.addData("empty.bin", nullptr, 0);
    ok = ok && writer.entryCount() == 5;
    ok = ok && writer.rawBytes() == 9 + 9 + large.size() + noise.size();
    ok = ok && writer.storedBytes() < writer.rawBytes();
    return ok && writer.write(path);
}

void test_pack_files(const ct::String& root)
{
    ct::Vector<std::uint8_t> large;
    fillLarge(large);
    ct::Vector<std::uint8_t> noise;
    fillNoise(noise, 4096);

    const ct::String plain = root + "/plain.rpak";
    const ct::String secret = root + "/secret.rpak";
    const ct::String stored = root + "/stored.rpak";
    CHECK(writePack(plain, "", 9, large, noise));
    CHECK(writePack(secret, "hunter2", 9, large, noise));

    {
        FilePackWriter writer;
        writer.setCompressionLevel(0);
        CHECK(writer.addData("large.bin", large.data(), large.size()));
        CHECK(writer.storedBytes() == large.size());
        CHECK(writer.write(stored));
    }

    {
        FilePack pack;
        CHECK(!pack.isOpen());
        CHECK(!pack.exists("shared.txt"));
        CHECK(pack.readBinary("shared.txt").empty());
        CHECK(pack.open(plain, ""));
        checkPackContents(pack, large, noise);
        pack.close();
        CHECK(!pack.isOpen());
        CHECK(pack.entryCount() == 0);

        CHECK(pack.open(plain, "unused key"));
        CHECK(pack.exists("shared.txt"));
    }

    {
        FilePack pack;
        CHECK(!pack.open(secret, ""));
        CHECK(!pack.isOpen());
        CHECK(!pack.open(secret, "hunter3"));
        CHECK(!pack.isOpen());
        CHECK(pack.open(secret, "hunter2"));
        checkPackContents(pack, large, noise);
    }

    {
        FilePack pack;
        CHECK(pack.open(stored, ""));
        CHECK(pack.entrySizeStored(0) == kLargeSize);
        CHECK(sameBytes(pack.readBinary("large.bin"), large.data(), large.size()));
    }

    {
        std::size_t size = 0;
        std::uint8_t* bytes = file_read(secret.c_str(), &size);
        CHECK(bytes != nullptr);
        if (bytes)
        {
            FilePack pack;
            CHECK(pack.openFromMemory(bytes, size, "hunter2"));
            checkPackContents(pack, large, noise);

            CHECK(!pack.openFromMemory(nullptr, 0, "hunter2"));
            CHECK(!pack.openFromMemory(bytes, FilePack::kHeaderSize - 1, "hunter2"));
            CHECK(!pack.openFromMemory(bytes, FilePack::kHeaderSize + 10, "hunter2"));

            ct::Vector<std::uint8_t> corrupt;
            corrupt.resize(size);
            memcpy(corrupt.data(), bytes, size);
            corrupt[FilePack::kHeaderSize + 1] ^= 0xff;
            CHECK(pack.openFromMemory(corrupt.data(), corrupt.size(), "hunter2"));
            CHECK(pack.exists("shared.txt"));
            CHECK(pack.readBinary("shared.txt").empty());
            CHECK(pack.open("shared.txt") == nullptr);
            CHECK(sameText(pack.readBinary("models/wall.txt"), "pack wall"));

            memcpy(corrupt.data(), bytes, size);
            corrupt[0] = 'X';
            CHECK(!pack.openFromMemory(corrupt.data(), corrupt.size(), "hunter2"));
            memcpy(corrupt.data(), bytes, size);
            corrupt[4] = 2;
            CHECK(!pack.openFromMemory(corrupt.data(), corrupt.size(), "hunter2"));

            memcpy(corrupt.data(), bytes, size);
            const std::uint32_t wrappingCount = 0x08000000u;
            const std::uint32_t wrappedTableSize = 0;
            const std::uint32_t shortNames = 8;
            memcpy(corrupt.data() + 12, &wrappingCount, 4);
            memcpy(corrupt.data() + 24, &wrappedTableSize, 4);
            memcpy(corrupt.data() + 28, &shortNames, 4);
            CHECK(!pack.openFromMemory(corrupt.data(), corrupt.size(), "hunter2"));
            fs_free(bytes);
        }
    }

    CHECK(file_write((root + "/garbage.rpak").c_str(), "this is not a pack at all!!!", 28));
    {
        FilePack pack;
        CHECK(!pack.open(root + "/garbage.rpak", ""));
        CHECK(!pack.open(root + "/missing.rpak", ""));
    }
}

void test_filesystem_mounts(const ct::String& root)
{
    ct::Vector<std::uint8_t> large;
    fillLarge(large);
    ct::Vector<std::uint8_t> noise;
    fillNoise(noise, 1024);

    CHECK(file_write((root + "/shared.txt").c_str(), "from disk", 9));
    CHECK(file_write((root + "/disk_only.txt").c_str(), "disk only", 9));

    {
        FilePackWriter writer;
        CHECK(writer.addData("shared.txt", reinterpret_cast<const std::uint8_t*>("fallback"), 8));
        CHECK(writer.addData("fallback_only.txt",
                reinterpret_cast<const std::uint8_t*>("only here"), 9));
        CHECK(writer.addData("disk_only.txt",
                reinterpret_cast<const std::uint8_t*>("fallback copy"), 13));
        CHECK(writer.write(root + "/fallback.rpak"));
    }

    std::size_t fallbackSize = 0;
    std::uint8_t* fallbackBytes = file_read((root + "/fallback.rpak").c_str(), &fallbackSize);
    CHECK(fallbackBytes != nullptr);
    if (!fallbackBytes) return;

    FileSystem files;
    files.addSearchPath(root);

    CHECK(!files.mountPack("missing.rpak", ""));
    CHECK(!files.mountPack("garbage.rpak", ""));
    CHECK(!files.mountPack("secret.rpak", "wrong"));
    CHECK(!files.mountFallbackPack(fallbackBytes, 10, ""));

    CHECK(files.mountFallbackPack(fallbackBytes, fallbackSize, ""));
    CHECK(files.readText("shared.txt") == "from disk");
    CHECK(files.readText("disk_only.txt") == "disk only");
    CHECK(files.exists("fallback_only.txt"));
    CHECK(files.readText("fallback_only.txt") == "only here");
    CHECK(files.resolve("fallback_only.txt").empty());
    IoStream* stream = files.open("fallback_only.txt");
    CHECK(stream != nullptr);
    if (stream)
    {
        char text[10] = {};
        CHECK(io_read(stream, text, 9) == 9);
        CHECK(memcmp(text, "only here", 9) == 0);
        CHECK(io_close(stream));
    }

    CHECK(files.mountPack("secret.rpak", "hunter2"));
    CHECK(files.readText("shared.txt") == "from pack");
    CHECK(files.readText("models/wall.txt") == "pack wall");
    CHECK(sameBytes(files.readBinary("large.bin"), large.data(), large.size()));
    CHECK(files.readText("disk_only.txt") == "disk only");
    CHECK(files.readText("fallback_only.txt") == "only here");
    CHECK(!files.exists("nowhere.txt"));
    CHECK(files.open("nowhere.txt") == nullptr);

    stream = files.open("shared.txt");
    CHECK(stream != nullptr);
    if (stream)
    {
        char text[10] = {};
        CHECK(io_read(stream, text, 9) == 9);
        CHECK(memcmp(text, "from pack", 9) == 0);
        CHECK(io_close(stream));
    }

    files.unmountAll();
    CHECK(files.readText("shared.txt") == "from disk");
    CHECK(!files.exists("fallback_only.txt"));
    CHECK(!files.exists("models/wall.txt"));
    fs_free(fallbackBytes);
}

} // namespace

// Large entries stay decoded after they are opened and are shared by the streams that read them:
// every stream keeps its bytes when another entry replaces the cached one or the pack closes.
void test_shared_entries(const ct::String& root)
{
    ct::Vector<std::uint8_t> first;
    ct::Vector<std::uint8_t> second;
    fillNoise(first, 3u << 20);
    fillNoise(second, 2u << 20);
    for (std::size_t i = 0; i < second.size(); ++i) second[i] ^= 0x5a;
    const ct::String path = root + "/shared.rpak";
    {
        FilePackWriter writer;
        writer.setCompressionLevel(1);
        CHECK(writer.addData("first.bin", first.data(), first.size()));
        CHECK(writer.addData("second.bin", second.data(), second.size()));
        CHECK(writer.addData("small.txt", reinterpret_cast<const std::uint8_t*>("small"), 5));
        CHECK(writer.write(path));
    }

    auto readAll = [](IoStream* stream, ct::Vector<std::uint8_t>& out)
    {
        out.clear();
        if (!stream) return;
        std::uint8_t chunk[65536];
        for (;;)
        {
            const std::size_t got = io_read(stream, chunk, sizeof(chunk));
            if (got == 0) break;
            for (std::size_t i = 0; i < got; ++i) out.push_back(chunk[i]);
        }
    };

    FilePack* pack = new FilePack();
    CHECK(pack->open(path, ""));
    IoStream* a = pack->open("first.bin");
    IoStream* b = pack->open("first.bin");
    IoStream* small = pack->open("small.txt");
    IoStream* c = pack->open("second.bin");
    CHECK(a && b && small && c);
    IoStream* d = pack->open("first.bin");
    delete pack;

    ct::Vector<std::uint8_t> bytes;
    IoStream* firsts[3] = { a, b, d };
    for (IoStream* stream: firsts)
    {
        readAll(stream, bytes);
        CHECK(sameBytes(bytes, first.data(), first.size()));
        if (stream) io_close(stream);
    }
    readAll(c, bytes);
    CHECK(sameBytes(bytes, second.data(), second.size()));
    readAll(small, bytes);
    CHECK(sameText(bytes, "small"));
    if (c) io_close(c);
    if (small) io_close(small);
    fs_remove_path(path.c_str());
}

int main()
{
    const ct::String root = ct::String(PRISMA3D_TEST_DIR) + "/pack";
    CHECK(fs_create_directory(root.c_str()));

    test_chacha20_rfc8439();
    test_derive_key();
    test_writer();
    test_pack_files(root);
    test_filesystem_mounts(root);
    test_shared_entries(root);

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
