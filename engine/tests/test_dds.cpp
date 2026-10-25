#include "Check.h"

#include "prisma3d/core/Pixmap.h"

#include "dds.h"
#include "platform.h"

#include <cstdint>
#include <string.h>

#include <ct/vector.hpp>

using namespace prisma3d;

namespace
{

typedef ct::Vector<unsigned char> Bytes;

std::uint32_t pack(unsigned r, unsigned g, unsigned b, unsigned a)
{
    return r | (g << 8) | (b << 16) | (a << 24);
}

Bytes make_dds(dds::DXGI_FORMAT format, unsigned width, unsigned height, const Bytes& payload,
        unsigned mips = 1, unsigned array_size = 1, bool cubemap = false, unsigned depth = 0)
{
    Bytes file;
    file.resize(sizeof(dds::Header) + payload.size());
    dds::write_header(file.data(), format, width, height, mips, array_size, cubemap, depth);
    if (payload.size() > 0)
        memcpy(file.data() + sizeof(dds::Header), payload.data(), payload.size());
    return file;
}

Bytes make_legacy_dxt1(unsigned width, unsigned height, const Bytes& payload)
{
    dds::Header header;
    memset(&header, 0, sizeof(header));
    dds::write_header(&header, dds::DXGI_FORMAT_BC1_UNORM, width, height);
    header.header.ddspf.dwFlags = dds::DDPF_FOURCC;
    header.header.ddspf.dwFourCC = dds::fourcc('D', 'X', 'T', '1');
    const std::size_t headerBytes = sizeof(dds::Header) - sizeof(dds::DDS_HEADER_DXT10);
    Bytes file;
    file.resize(headerBytes + payload.size());
    memcpy(file.data(), &header, headerBytes);
    memcpy(file.data() + headerBytes, payload.data(), payload.size());
    return file;
}

void put16(Bytes& b, std::uint16_t v)
{
    b.push_back(static_cast<unsigned char>(v & 0xFF));
    b.push_back(static_cast<unsigned char>(v >> 8));
}

void put32(Bytes& b, std::uint32_t v)
{
    put16(b, static_cast<std::uint16_t>(v & 0xFFFF));
    put16(b, static_cast<std::uint16_t>(v >> 16));
}

void colour_block(Bytes& b, std::uint16_t c0, std::uint16_t c1, const unsigned indices[16])
{
    put16(b, c0);
    put16(b, c1);
    std::uint32_t bits = 0;
    for (int i = 0; i < 16; ++i) bits |= indices[i] << (i * 2);
    put32(b, bits);
}

void solid_block(Bytes& b, std::uint16_t colour)
{
    const unsigned zero[16] = {};
    colour_block(b, colour, colour, zero);
}

void alpha_block(Bytes& b, unsigned a0, unsigned a1, const unsigned indices[16])
{
    b.push_back(static_cast<unsigned char>(a0));
    b.push_back(static_cast<unsigned char>(a1));
    std::uint64_t bits = 0;
    for (int i = 0; i < 16; ++i) bits |= static_cast<std::uint64_t>(indices[i]) << (i * 3);
    for (int i = 0; i < 6; ++i) b.push_back(static_cast<unsigned char>((bits >> (i * 8)) & 0xFF));
}

bool load(Pixmap& pixmap, const Bytes& file)
{
    return pixmap.load_from_memory(file.data(), static_cast<std::uint32_t>(file.size()));
}

Bytes bc1_palette_payload()
{
    unsigned indices[16];
    for (int i = 0; i < 16; ++i) indices[i] = static_cast<unsigned>((i + i / 4) % 4);
    Bytes payload;
    colour_block(payload, 0xF800, 0x07E0, indices);
    return payload;
}

void test_bc1_four_colour()
{
    const std::uint32_t palette[4] = { pack(255, 0, 0, 255), pack(0, 255, 0, 255),
        pack(170, 85, 0, 255), pack(85, 170, 0, 255) };
    const Bytes file = make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, bc1_palette_payload());
    Pixmap pixmap;
    CHECK(load(pixmap, file));
    CHECK(pixmap.is_valid());
    CHECK(pixmap.width == 4 && pixmap.height == 4 && pixmap.components == 4);
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x)
        {
            const int i = y * 4 + x;
            CHECK(pixmap.get_pixel(x, y) == palette[(i + i / 4) % 4]);
        }

    const Bytes srgb = make_dds(dds::DXGI_FORMAT_BC1_UNORM_SRGB, 4, 4, bc1_palette_payload());
    Pixmap other;
    CHECK(load(other, srgb));
    CHECK(other.components == 4);
    CHECK(other.get_pixel(0, 0) == pack(255, 0, 0, 255));
    CHECK(other.get_pixel(2, 0) == pack(170, 85, 0, 255));
    CHECK(other.get_pixel(3, 0) == pack(85, 170, 0, 255));
}

void test_bc1_punch_through()
{
    const unsigned indices[16] = { 0, 1, 2, 3, 3, 2, 1, 0, 2, 2, 3, 3, 0, 1, 1, 0 };
    const std::uint32_t palette[4] = { pack(0, 255, 0, 255), pack(255, 0, 0, 255),
        pack(127, 127, 0, 255), pack(0, 0, 0, 0) };
    Bytes payload;
    colour_block(payload, 0x07E0, 0xF800, indices);
    Pixmap pixmap;
    CHECK(load(pixmap, make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, payload)));
    CHECK(pixmap.components == 4);
    for (int i = 0; i < 16; ++i) CHECK(pixmap.get_pixel(i % 4, i / 4) == palette[indices[i]]);
}

void test_bc2()
{
    Bytes payload;
    for (int j = 0; j < 8; ++j)
        payload.push_back(static_cast<unsigned char>((2 * j) | ((2 * j + 1) << 4)));
    solid_block(payload, 0xFFFF);
    Pixmap pixmap;
    CHECK(load(pixmap, make_dds(dds::DXGI_FORMAT_BC2_UNORM, 4, 4, payload)));
    CHECK(pixmap.width == 4 && pixmap.height == 4 && pixmap.components == 4);
    for (int i = 0; i < 16; ++i)
        CHECK(pixmap.get_pixel(i % 4, i / 4) == pack(255, 255, 255, static_cast<unsigned>(i * 17)));

    Pixmap srgb;
    CHECK(load(srgb, make_dds(dds::DXGI_FORMAT_BC2_UNORM_SRGB, 4, 4, payload)));
    CHECK(srgb.get_pixel(3, 3) == pack(255, 255, 255, 255));
    CHECK(srgb.get_pixel(1, 0) == pack(255, 255, 255, 17));
}

void test_bc3()
{
    const unsigned mode8[8] = { 255, 0, 218, 182, 145, 109, 72, 36 };
    const unsigned mode6[8] = { 0, 255, 51, 102, 153, 204, 0, 255 };
    unsigned indicesA[16];
    unsigned indicesB[16];
    for (int i = 0; i < 16; ++i)
    {
        indicesA[i] = static_cast<unsigned>((i * 3) % 8);
        indicesB[i] = static_cast<unsigned>((i * 5 + 1) % 8);
    }
    Bytes payload;
    alpha_block(payload, 255, 0, indicesA);
    solid_block(payload, 0xF800);
    alpha_block(payload, 0, 255, indicesB);
    solid_block(payload, 0x07E0);
    Pixmap pixmap;
    CHECK(load(pixmap, make_dds(dds::DXGI_FORMAT_BC3_UNORM, 8, 4, payload)));
    CHECK(pixmap.width == 8 && pixmap.height == 4 && pixmap.components == 4);
    for (int i = 0; i < 16; ++i)
    {
        CHECK(pixmap.get_pixel(i % 4, i / 4) == pack(255, 0, 0, mode8[indicesA[i]]));
        CHECK(pixmap.get_pixel(4 + i % 4, i / 4) == pack(0, 255, 0, mode6[indicesB[i]]));
    }

    Pixmap srgb;
    CHECK(load(srgb, make_dds(dds::DXGI_FORMAT_BC3_UNORM_SRGB, 8, 4, payload)));
    CHECK(srgb.get_pixel(0, 0) == pack(255, 0, 0, 255));
}

void test_partial_blocks()
{
    Bytes payload;
    const unsigned zero[16] = {};
    colour_block(payload, 0xF800, 0x07E0, zero);
    Bytes second = bc1_palette_payload();
    for (std::size_t i = 0; i < second.size(); ++i) payload.push_back(second[i]);
    Pixmap pixmap;
    CHECK(load(pixmap, make_dds(dds::DXGI_FORMAT_BC1_UNORM, 5, 3, payload)));
    CHECK(pixmap.is_valid());
    CHECK(pixmap.width == 5 && pixmap.height == 3 && pixmap.components == 4);
    CHECK(pixmap.get_size() == 60);
    for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 4; ++x) CHECK(pixmap.get_pixel(x, y) == pack(255, 0, 0, 255));
    CHECK(pixmap.get_pixel(4, 0) == pack(255, 0, 0, 255));
    CHECK(pixmap.get_pixel(4, 1) == pack(0, 255, 0, 255));
    CHECK(pixmap.get_pixel(4, 2) == pack(170, 85, 0, 255));
}

void test_mip_chain()
{
    Bytes payload;
    solid_block(payload, 0xF800);
    solid_block(payload, 0x07E0);
    solid_block(payload, 0x001F);
    solid_block(payload, 0xFFFF);
    for (int i = 0; i < 8; ++i) payload.push_back(0xAB);
    Pixmap pixmap;
    CHECK(load(pixmap, make_dds(dds::DXGI_FORMAT_BC1_UNORM, 8, 8, payload, 2)));
    CHECK(pixmap.width == 8 && pixmap.height == 8 && pixmap.components == 4);
    CHECK(pixmap.get_pixel(0, 0) == pack(255, 0, 0, 255));
    CHECK(pixmap.get_pixel(3, 3) == pack(255, 0, 0, 255));
    CHECK(pixmap.get_pixel(4, 0) == pack(0, 255, 0, 255));
    CHECK(pixmap.get_pixel(7, 3) == pack(0, 255, 0, 255));
    CHECK(pixmap.get_pixel(0, 4) == pack(0, 0, 255, 255));
    CHECK(pixmap.get_pixel(3, 7) == pack(0, 0, 255, 255));
    CHECK(pixmap.get_pixel(4, 4) == pack(255, 255, 255, 255));
    CHECK(pixmap.get_pixel(7, 7) == pack(255, 255, 255, 255));
}

void test_uncompressed()
{
    const unsigned char rgba[16] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140,
        150, 160 };
    Bytes payload;
    for (int i = 0; i < 16; ++i) payload.push_back(rgba[i]);

    Pixmap straight;
    CHECK(load(straight, make_dds(dds::DXGI_FORMAT_R8G8B8A8_UNORM, 2, 2, payload)));
    CHECK(straight.width == 2 && straight.height == 2 && straight.components == 4);
    CHECK(straight.get_pixel(0, 0) == pack(10, 20, 30, 40));
    CHECK(straight.get_pixel(1, 0) == pack(50, 60, 70, 80));
    CHECK(straight.get_pixel(0, 1) == pack(90, 100, 110, 120));
    CHECK(straight.get_pixel(1, 1) == pack(130, 140, 150, 160));

    Pixmap srgb;
    CHECK(load(srgb, make_dds(dds::DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 2, 2, payload)));
    CHECK(srgb.components == 4);
    CHECK(srgb.get_pixel(1, 1) == pack(130, 140, 150, 160));

    Pixmap swapped;
    CHECK(load(swapped, make_dds(dds::DXGI_FORMAT_B8G8R8A8_UNORM, 2, 2, payload)));
    CHECK(swapped.components == 4);
    CHECK(swapped.get_pixel(0, 0) == pack(30, 20, 10, 40));
    CHECK(swapped.get_pixel(1, 0) == pack(70, 60, 50, 80));
    CHECK(swapped.get_pixel(0, 1) == pack(110, 100, 90, 120));
    CHECK(swapped.get_pixel(1, 1) == pack(150, 140, 130, 160));

    Pixmap swappedSrgb;
    CHECK(load(swappedSrgb, make_dds(dds::DXGI_FORMAT_B8G8R8A8_UNORM_SRGB, 2, 2, payload)));
    CHECK(swappedSrgb.get_pixel(0, 0) == pack(30, 20, 10, 40));

    Pixmap opaque;
    CHECK(load(opaque, make_dds(dds::DXGI_FORMAT_B8G8R8X8_UNORM, 2, 2, payload)));
    CHECK(opaque.components == 4);
    CHECK(opaque.get_pixel(0, 0) == pack(30, 20, 10, 255));
    CHECK(opaque.get_pixel(1, 0) == pack(70, 60, 50, 255));
    CHECK(opaque.get_pixel(0, 1) == pack(110, 100, 90, 255));
    CHECK(opaque.get_pixel(1, 1) == pack(150, 140, 130, 255));

    Bytes gray;
    const unsigned char values[6] = { 0, 50, 100, 150, 200, 255 };
    for (int i = 0; i < 6; ++i) gray.push_back(values[i]);
    Pixmap luminance;
    CHECK(load(luminance, make_dds(dds::DXGI_FORMAT_R8_UNORM, 3, 2, gray)));
    CHECK(luminance.is_valid());
    CHECK(luminance.width == 3 && luminance.height == 2 && luminance.components == 3);
    for (int i = 0; i < 6; ++i)
    {
        const unsigned char* texel = luminance.pixels + i * 3;
        CHECK(texel[0] == values[i] && texel[1] == values[i] && texel[2] == values[i]);
    }
}

void test_legacy_header()
{
    const Bytes file = make_legacy_dxt1(4, 4, bc1_palette_payload());
    CHECK(file.size() == sizeof(dds::Header) - sizeof(dds::DDS_HEADER_DXT10) + 8);
    Pixmap pixmap;
    CHECK(load(pixmap, file));
    CHECK(pixmap.width == 4 && pixmap.height == 4 && pixmap.components == 4);
    CHECK(pixmap.get_pixel(0, 0) == pack(255, 0, 0, 255));
    CHECK(pixmap.get_pixel(1, 0) == pack(0, 255, 0, 255));
    CHECK(pixmap.get_pixel(2, 0) == pack(170, 85, 0, 255));
    CHECK(pixmap.get_pixel(3, 0) == pack(85, 170, 0, 255));
}

void test_failures()
{
    Bytes block;
    solid_block(block, 0xF800);
    Bytes bc7;
    for (int i = 0; i < 16; ++i) bc7.push_back(0);
    Bytes cube;
    for (int i = 0; i < 6 * 8; ++i) cube.push_back(0);
    Bytes pair;
    for (int i = 0; i < 2 * 8; ++i) pair.push_back(0);
    Bytes volume;
    for (int i = 0; i < 2 * 8; ++i) volume.push_back(0);

    {
        Pixmap pixmap;
        CHECK(!load(pixmap, make_dds(dds::DXGI_FORMAT_BC7_UNORM, 4, 4, bc7)));
        CHECK(!pixmap.is_valid());
    }
    {
        Pixmap pixmap;
        CHECK(!load(pixmap, make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, cube, 1, 6, true)));
        CHECK(!pixmap.is_valid());
    }
    {
        Pixmap pixmap;
        CHECK(!load(pixmap, make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, pair, 1, 2)));
        CHECK(!pixmap.is_valid());
    }
    {
        Pixmap pixmap;
        CHECK(!load(pixmap, make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, volume, 1, 1, false, 2)));
        CHECK(!pixmap.is_valid());
    }
    {
        Bytes file = make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, block);
        Pixmap whole;
        CHECK(load(whole, file));
        file.resize(file.size() - 1);
        Pixmap pixmap;
        CHECK(!load(pixmap, file));
        CHECK(!pixmap.is_valid());
    }
    {
        Bytes empty;
        Pixmap pixmap;
        CHECK(!load(pixmap, make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, empty)));
        CHECK(!pixmap.is_valid());
    }
    {
        Bytes file = make_dds(dds::DXGI_FORMAT_R8G8B8A8_UNORM, 2, 2, Bytes());
        Pixmap pixmap;
        CHECK(!load(pixmap, file));
        CHECK(!pixmap.is_valid());
    }
    {
        Bytes legacyTruncated = make_legacy_dxt1(4, 4, Bytes());
        Pixmap pixmap;
        CHECK(!load(pixmap, legacyTruncated));
        CHECK(!pixmap.is_valid());
    }
}

void test_files()
{
    const char* directory = PRISMA3D_TEST_DIR "/dds";
    const char* path = PRISMA3D_TEST_DIR "/dds/sample.dds";
    CHECK(fs_create_directory(directory));
    const Bytes file = make_dds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, bc1_palette_payload());
    CHECK(file_write(path, file.data(), file.size()));

    Pixmap fromPath;
    CHECK(fromPath.load(path));
    CHECK(fromPath.width == 4 && fromPath.height == 4 && fromPath.components == 4);
    CHECK(fromPath.get_pixel(0, 0) == pack(255, 0, 0, 255));
    CHECK(fromPath.get_pixel(3, 0) == pack(85, 170, 0, 255));

    IoStream* stream = io_open_file(path, "rb");
    CHECK(stream != nullptr);
    if (stream)
    {
        Pixmap fromStream;
        CHECK(fromStream.load(stream));
        CHECK(fromStream.width == 4 && fromStream.height == 4 && fromStream.components == 4);
        CHECK(fromStream.get_pixel(1, 0) == pack(0, 255, 0, 255));
        CHECK(fromStream.get_pixel(2, 0) == pack(170, 85, 0, 255));
        io_close(stream);
    }
}

} // namespace

int main()
{
    test_bc1_four_colour();
    test_bc1_punch_through();
    test_bc2();
    test_bc3();
    test_partial_blocks();
    test_mip_chain();
    test_uncompressed();
    test_legacy_header();
    test_failures();
    test_files();
    if (failures == 0) printf("all checks passed\n");
    return failures ? 1 : 0;
}
