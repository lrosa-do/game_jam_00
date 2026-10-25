#include "Check.h"

#include "prisma3d/core/Color.h"
#include "prisma3d/core/Pixmap.h"

#include "platform.h"

#include <cstdint>
#include <cstring>

using namespace prisma3d;

namespace
{

std::uint32_t pack(unsigned r, unsigned g, unsigned b, unsigned a)
{
    return r | (g << 8) | (b << 16) | (a << 24);
}

void put(Pixmap& p, int x, int y, unsigned r, unsigned g, unsigned b, unsigned a)
{
    p.set_pixel(x, y, static_cast<std::uint8_t>(r), static_cast<std::uint8_t>(g),
            static_cast<std::uint8_t>(b), static_cast<std::uint8_t>(a));
}

std::uint32_t sample(int x, int y, int width)
{
    const unsigned i = static_cast<unsigned>(y * width + x + 1);
    return pack(i * 10, i * 10 + 1, i * 10 + 2, 200 + i);
}

void fill_sample(Pixmap& p)
{
    for (int y = 0; y < p.height; ++y)
        for (int x = 0; x < p.width; ++x) p.set_pixel(x, y, sample(x, y, p.width));
}

bool matches_sample(const Pixmap& p, int width, int height)
{
    if (!p.is_valid() || p.width != width || p.height != height || p.components != 4) return false;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (p.get_pixel(x, y) != sample(x, y, width)) return false;
    return true;
}

void test_basics()
{
    Pixmap a(4, 3, 4);
    CHECK(a.is_valid());
    CHECK(a.width == 4 && a.height == 3 && a.components == 4);
    CHECK(a.get_size() == 48);
    CHECK(a.has_alpha());

    Pixmap b(2, 2, 3);
    CHECK(b.get_size() == 12);
    CHECK(!b.has_alpha());

    Pixmap c(5, 2, 2);
    CHECK(c.get_size() == 20);
    CHECK(c.has_alpha());

    Pixmap d(3, 3, 1);
    CHECK(d.get_size() == 9);
    CHECK(!d.has_alpha());

    Pixmap empty;
    CHECK(!empty.is_valid());
    CHECK(empty.get_size() == 0);
}

void test_set_get()
{
    Pixmap p4(3, 3, 4);
    put(p4, 1, 2, 10, 20, 30, 40);
    CHECK(p4.get_pixel(1, 2) == pack(10, 20, 30, 40));
    Color c4 = p4.get_pixel_color(1, 2);
    CHECK(c4.r() == 10 && c4.g() == 20 && c4.b() == 30 && c4.a() == 40);
    CHECK(p4.get_pixel(0, 0) == 0);
    p4.set_pixel(0, 1, 0x80402010u);
    CHECK(p4.get_pixel(0, 1) == 0x80402010u);
    put(p4, 3, 0, 1, 2, 3, 4);
    put(p4, 0, 3, 1, 2, 3, 4);
    CHECK(p4.get_pixel(3, 0) == 0);
    CHECK(p4.get_pixel(0, 3) == 0);
    Color out = p4.get_pixel_color(9, 9);
    CHECK(out.value() == 0);

    Pixmap p3(3, 3, 3);
    put(p3, 2, 1, 10, 20, 30, 40);
    CHECK(p3.get_pixel(2, 1) == pack(10, 20, 30, 0));
    Color c3 = p3.get_pixel_color(2, 1);
    CHECK(c3.r() == 10 && c3.g() == 20 && c3.b() == 30 && c3.a() == 255);

    Pixmap p2(3, 3, 2);
    put(p2, 1, 1, 100, 150, 200, 77);
    CHECK(p2.get_pixel(1, 1) == pack(140, 77, 0, 0));
    Color c2 = p2.get_pixel_color(1, 1);
    CHECK(c2.r() == 140 && c2.g() == 140 && c2.b() == 140 && c2.a() == 77);

    Pixmap p1(3, 3, 1);
    put(p1, 0, 2, 100, 150, 200, 77);
    CHECK(p1.get_pixel(0, 2) == 140);
    Color c1 = p1.get_pixel_color(0, 2);
    CHECK(c1.r() == 140 && c1.g() == 140 && c1.b() == 140 && c1.a() == 255);
    put(p1, 1, 1, 255, 0, 0, 255);
    CHECK(p1.get_pixel(1, 1) == 76);
}

void test_fill_clear()
{
    Pixmap p(3, 2, 4);
    p.fill(1, 2, 3, 4);
    bool all = true;
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 3; ++x) all = all && p.get_pixel(x, y) == pack(1, 2, 3, 4);
    CHECK(all);

    p.fill(0xFF402010u);
    all = true;
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 3; ++x) all = all && p.get_pixel(x, y) == pack(0x10, 0x20, 0x40, 0xFF);
    CHECK(all);

    p.clear();
    all = true;
    for (int i = 0; i < p.get_size(); ++i) all = all && p.pixels[i] == 0;
    CHECK(all);

    Pixmap q(2, 2, 3);
    q.fill(9, 8, 7, 6);
    CHECK(q.get_pixel(1, 1) == pack(9, 8, 7, 0));
    q.clear();
    CHECK(q.get_pixel(1, 1) == 0);
}

void test_flip()
{
    Pixmap p(3, 2, 4);
    fill_sample(p);
    p.flip_vertical();
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 3; ++x) CHECK(p.get_pixel(x, y) == sample(x, 1 - y, 3));
    p.flip_vertical();
    CHECK(matches_sample(p, 3, 2));

    p.flip_horizontal();
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 3; ++x) CHECK(p.get_pixel(x, y) == sample(2 - x, y, 3));
    CHECK(p.get_pixel(0, 0) == pack(30, 31, 32, 203));
    CHECK(p.get_pixel(2, 1) == pack(40, 41, 42, 204));
    p.flip_horizontal();
    CHECK(matches_sample(p, 3, 2));

    Pixmap e;
    e.flip_vertical();
    e.flip_horizontal();
    CHECK(!e.is_valid());
}

void test_convert()
{
    Pixmap g(2, 1, 1);
    g.pixels[0] = 50;
    g.pixels[1] = 200;
    Pixmap* r = g.convert_to_rgba();
    CHECK(r != nullptr);
    if (r)
    {
        CHECK(r->components == 4 && r->width == 2 && r->height == 1);
        CHECK(r->get_pixel(0, 0) == pack(50, 50, 50, 255));
        CHECK(r->get_pixel(1, 0) == pack(200, 200, 200, 255));
        delete r;
    }

    Pixmap rgb(2, 1, 3);
    put(rgb, 0, 0, 1, 2, 3, 0);
    put(rgb, 1, 0, 250, 251, 252, 0);
    r = rgb.convert_to_rgba();
    CHECK(r != nullptr);
    if (r)
    {
        CHECK(r->components == 4);
        CHECK(r->get_pixel(0, 0) == pack(1, 2, 3, 255));
        CHECK(r->get_pixel(1, 0) == pack(250, 251, 252, 255));
        delete r;
    }

    Pixmap ga(1, 1, 2);
    ga.pixels[0] = 60;
    ga.pixels[1] = 90;
    r = ga.convert_to_rgba();
    CHECK(r != nullptr);
    if (r)
    {
        CHECK(r->get_pixel(0, 0) == pack(60, 60, 60, 90));
        delete r;
    }

    Pixmap rgba(3, 2, 4);
    fill_sample(rgba);
    r = rgba.convert_to_rgba();
    CHECK(r != nullptr);
    if (r)
    {
        CHECK(r != &rgba);
        CHECK(matches_sample(*r, 3, 2));
        delete r;
    }
}

void test_crop_resize()
{
    Pixmap p(4, 3, 4);
    fill_sample(p);

    Pixmap* c = p.crop(1, 1, 2, 2);
    CHECK(c != nullptr);
    if (c)
    {
        CHECK(c->width == 2 && c->height == 2 && c->components == 4);
        CHECK(c->get_pixel(0, 0) == pack(60, 61, 62, 206));
        CHECK(c->get_pixel(1, 0) == pack(70, 71, 72, 207));
        CHECK(c->get_pixel(0, 1) == pack(100, 101, 102, 210));
        CHECK(c->get_pixel(1, 1) == pack(110, 111, 112, 211));
        delete c;
    }

    c = p.crop(IntRect(3, 2, 5, 5));
    CHECK(c != nullptr);
    if (c)
    {
        CHECK(c->width == 1 && c->height == 1);
        CHECK(c->get_pixel(0, 0) == pack(120, 121, 122, 212));
        delete c;
    }

    c = p.crop(-1, -1, 3, 3);
    CHECK(c != nullptr);
    if (c)
    {
        CHECK(c->width == 2 && c->height == 2);
        CHECK(c->get_pixel(0, 0) == pack(10, 11, 12, 201));
        CHECK(c->get_pixel(1, 1) == pack(60, 61, 62, 206));
        delete c;
    }

    CHECK(p.crop(4, 0, 2, 2) == nullptr);
    CHECK(p.crop(0, 3, 2, 2) == nullptr);
    CHECK(p.crop(0, 0, 0, 2) == nullptr);

    Pixmap* x = p.crop_extended(IntRect(-1, -1, 3, 3), true);
    CHECK(x != nullptr);
    if (x)
    {
        CHECK(x->width == 3 && x->height == 3);
        CHECK(x->get_pixel(0, 0) == 0);
        CHECK(x->get_pixel(1, 0) == 0);
        CHECK(x->get_pixel(0, 1) == 0);
        CHECK(x->get_pixel(1, 1) == pack(10, 11, 12, 201));
        CHECK(x->get_pixel(2, 2) == pack(60, 61, 62, 206));
        delete x;
    }
    x = p.crop_extended(IntRect(-1, 0, 2, 1), false);
    CHECK(x != nullptr);
    if (x)
    {
        CHECK(x->get_pixel(0, 0) == pack(0, 0, 0, 255));
        CHECK(x->get_pixel(1, 0) == pack(10, 11, 12, 201));
        delete x;
    }

    Pixmap viaCtor(p, IntRect(2, 1, 2, 2));
    CHECK(viaCtor.width == 2 && viaCtor.height == 2 && viaCtor.components == 4);
    CHECK(viaCtor.get_pixel(0, 0) == pack(70, 71, 72, 207));
    CHECK(viaCtor.get_pixel(1, 1) == pack(120, 121, 122, 212));

    Pixmap s(2, 2, 4);
    fill_sample(s);
    Pixmap* up = s.resize(4, 4);
    CHECK(up != nullptr);
    if (up)
    {
        CHECK(up->width == 4 && up->height == 4 && up->components == 4);
        bool ok = true;
        for (int y = 0; y < 4; ++y)
            for (int x2 = 0; x2 < 4; ++x2)
                ok = ok && up->get_pixel(x2, y) == sample(x2 / 2, y / 2, 2);
        CHECK(ok);
        delete up;
    }

    Pixmap big(4, 4, 4);
    fill_sample(big);
    Pixmap* down = big.resize(2, 2);
    CHECK(down != nullptr);
    if (down)
    {
        CHECK(down->width == 2 && down->height == 2);
        CHECK(down->get_pixel(0, 0) == sample(0, 0, 4));
        CHECK(down->get_pixel(1, 0) == sample(2, 0, 4));
        CHECK(down->get_pixel(0, 1) == sample(0, 2, 4));
        CHECK(down->get_pixel(1, 1) == sample(2, 2, 4));
        delete down;
    }
}

void test_color()
{
    Color c(10, 20, 30, 40);
    CHECK(c.value() == 0x280A141Eu);
    CHECK(c.r() == 10 && c.g() == 20 && c.b() == 30 && c.a() == 40);
    CHECK(Color().value() == 0xFFFFFFFFu);
    CHECK(Color::lerp(Color(0, 0, 0, 0), Color(200, 100, 50, 255), 0.5f).value() ==
            Color(100, 50, 25, 127).value());
    CHECK(Color(0x00123456u).withAlpha(1.0f).value() == 0xFF123456u);
    CHECK(Color::fromHSV(0.0f, 1.0f, 1.0f).value() == Color(255, 0, 0, 255).value());
}

bool write_file_bytes(const char* path, const unsigned char* data, std::size_t n)
{
    return file_write(path, data, n);
}

void test_png_roundtrip()
{
    Pixmap src(5, 3, 4);
    fill_sample(src);
    CHECK(matches_sample(src, 5, 3));

    const char* path = PRISMA3D_TEST_DIR "/pixmap_roundtrip.png";
    CHECK(src.save(path));

    Pixmap a;
    CHECK(a.load(path));
    CHECK(matches_sample(a, 5, 3));

    IoStream* stream = io_open_file(path, "rb");
    CHECK(stream != nullptr);
    if (stream)
    {
        Pixmap b;
        CHECK(b.load(stream));
        CHECK(matches_sample(b, 5, 3));
        io_close(stream);
    }

    std::size_t size = 0;
    std::uint8_t* bytes = file_read(path, &size);
    CHECK(bytes != nullptr);
    CHECK(size > 8);
    if (bytes)
    {
        IoStream* mem = io_open_memory(bytes, size);
        CHECK(mem != nullptr);
        if (mem)
        {
            Pixmap c;
            CHECK(c.load(mem));
            CHECK(matches_sample(c, 5, 3));
            io_close(mem);
        }

        Pixmap d;
        CHECK(d.load_from_memory(bytes, static_cast<std::uint32_t>(size)));
        CHECK(matches_sample(d, 5, 3));
        fs_free(bytes);
    }

    Pixmap rgb(2, 2, 3);
    rgb.fill(11, 22, 33, 0);
    const char* rgbPath = PRISMA3D_TEST_DIR "/pixmap_roundtrip_rgb.png";
    CHECK(rgb.save(rgbPath));
    Pixmap rgbBack;
    CHECK(rgbBack.load(rgbPath));
    CHECK(rgbBack.components == 3 && rgbBack.width == 2 && rgbBack.height == 2);
    CHECK(rgbBack.get_pixel(1, 1) == pack(11, 22, 33, 0));
}

void test_load_failures()
{
    Pixmap p;
    CHECK(!p.load(PRISMA3D_TEST_DIR "/does_not_exist.png"));
    CHECK(!p.is_valid());
    CHECK(!p.load(static_cast<const char*>(nullptr)));
    CHECK(!p.load(static_cast<IoStream*>(nullptr)));

    unsigned char garbage[64];
    for (int i = 0; i < 64; ++i) garbage[i] = static_cast<unsigned char>(i * 37 + 11);
    CHECK(!p.load_from_memory(garbage, sizeof(garbage)));
    CHECK(!p.load_from_memory(nullptr, 10));
    CHECK(!p.load_from_memory(garbage, 0));
    CHECK(!p.is_valid());

    const char* junk = PRISMA3D_TEST_DIR "/pixmap_garbage.bin";
    CHECK(write_file_bytes(junk, garbage, sizeof(garbage)));
    CHECK(!p.load(junk));
    IoStream* mem = io_open_memory(garbage, sizeof(garbage));
    CHECK(mem != nullptr);
    if (mem)
    {
        CHECK(!p.load(mem));
        io_close(mem);
    }
    CHECK(!p.is_valid());

    Pixmap none;
    CHECK(!none.save(PRISMA3D_TEST_DIR "/never.png"));
}

} // namespace

static void test_fix_alpha_edges()
{
    {
        Pixmap image(7, 1, 4);
        for (std::uint32_t x = 0; x < 7; ++x) image.set_pixel(x, 0, 255, 255, 255, 0);
        image.set_pixel(0, 0, 200, 10, 20, 255);
        image.set_pixel(6, 0, 1, 2, 3, 19);
        image.fix_alpha_edges();
        CHECK(image.pixels[0] == 200 && image.pixels[1] == 10 && image.pixels[2] == 20 &&
                image.pixels[3] == 255);
        for (int x = 1; x <= 4; ++x)
        {
            const unsigned char* p = &image.pixels[x * 4];
            CHECK(p[0] == 200 && p[1] == 10 && p[2] == 20 && p[3] == 0);
        }
        CHECK(image.pixels[5 * 4] == 255 && image.pixels[5 * 4 + 1] == 255 &&
                image.pixels[5 * 4 + 3] == 0);
        CHECK(image.pixels[6 * 4] == 1 && image.pixels[6 * 4 + 1] == 2 &&
                image.pixels[6 * 4 + 3] == 19);
    }
    {
        Pixmap image(5, 1, 4);
        for (std::uint32_t x = 0; x < 5; ++x) image.set_pixel(x, 0, 9, 9, 9, 0);
        image.set_pixel(0, 0, 100, 0, 0, 20);
        image.set_pixel(4, 0, 0, 100, 0, 255);
        image.fix_alpha_edges();
        CHECK(image.pixels[1 * 4] == 100 && image.pixels[1 * 4 + 1] == 0);
        CHECK(image.pixels[2 * 4] == 100 && image.pixels[2 * 4 + 1] == 0);
        CHECK(image.pixels[3 * 4] == 0 && image.pixels[3 * 4 + 1] == 100);
    }
    {
        Pixmap image(3, 3, 4);
        for (std::uint32_t y = 0; y < 3; ++y)
            for (std::uint32_t x = 0; x < 3; ++x) image.set_pixel(x, y, 50, 60, 70, 0);
        image.fix_alpha_edges();
        CHECK(image.pixels[4 * 4] == 50 && image.pixels[4 * 4 + 1] == 60 &&
                image.pixels[4 * 4 + 2] == 70);
    }
    {
        Pixmap image(2, 1, 3);
        image.set_pixel(0, 0, 10, 20, 30, 255);
        image.set_pixel(1, 0, 40, 50, 60, 255);
        image.fix_alpha_edges();
        CHECK(image.pixels[3] == 40 && image.pixels[4] == 50 && image.pixels[5] == 60);
    }
}

static void test_fill_transparent_color()
{
    {
        Pixmap image(12, 1, 4);
        for (std::uint32_t x = 0; x < 12; ++x) image.set_pixel(x, 0, 255, 255, 255, 0);
        image.set_pixel(0, 0, 200, 10, 20, 255);
        image.fill_transparent_color();
        for (int x = 0; x < 12; ++x)
        {
            const unsigned char* p = &image.pixels[x * 4];
            CHECK(p[0] == 200 && p[1] == 10 && p[2] == 20);
            CHECK(p[3] == (x == 0 ? 255 : 0));
        }
    }
    {
        Pixmap image(9, 1, 4);
        for (std::uint32_t x = 0; x < 9; ++x) image.set_pixel(x, 0, 255, 255, 255, 0);
        image.set_pixel(0, 0, 100, 0, 0, 255);
        image.set_pixel(8, 0, 0, 100, 0, 20);
        image.fill_transparent_color();
        for (int x = 1; x <= 3; ++x)
            CHECK(image.pixels[x * 4] == 100 && image.pixels[x * 4 + 1] == 0);
        for (int x = 5; x <= 7; ++x)
            CHECK(image.pixels[x * 4] == 0 && image.pixels[x * 4 + 1] == 100);
        CHECK(image.pixels[8 * 4 + 3] == 20);
    }
    {
        Pixmap image(4, 4, 4);
        for (std::uint32_t y = 0; y < 4; ++y)
            for (std::uint32_t x = 0; x < 4; ++x) image.set_pixel(x, y, 255, 255, 255, 0);
        image.set_pixel(3, 3, 7, 8, 9, 255);
        image.fill_transparent_color();
        for (int i = 0; i < 16; ++i)
            CHECK(image.pixels[i * 4] == 7 && image.pixels[i * 4 + 1] == 8 &&
                    image.pixels[i * 4 + 2] == 9);
    }
    {
        Pixmap image(3, 1, 4);
        for (std::uint32_t x = 0; x < 3; ++x) image.set_pixel(x, 0, 50, 60, 70, 0);
        image.fill_transparent_color();
        CHECK(image.pixels[4] == 50 && image.pixels[5] == 60 && image.pixels[6] == 70);
    }
    {
        Pixmap image(2, 1, 3);
        image.set_pixel(0, 0, 10, 20, 30, 255);
        image.set_pixel(1, 0, 40, 50, 60, 255);
        image.fill_transparent_color();
        CHECK(image.pixels[3] == 40 && image.pixels[4] == 50 && image.pixels[5] == 60);
    }
}

int main()
{
    test_basics();
    test_set_get();
    test_fill_clear();
    test_flip();
    test_convert();
    test_crop_resize();
    test_color();
    test_png_roundtrip();
    test_load_failures();
    test_fix_alpha_edges();
    test_fill_transparent_color();
    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
