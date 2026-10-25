#include "prisma3d/core/Pixmap.h"

#include "prisma3d/core/Color.h"

#include <ct/vector.hpp>

#include "platform.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Bcn.h"
#include "dds.h"
#include "stb_image.h"
#include "stb_image_write.h"

namespace prisma3d
{

namespace
{

template<typename T>
inline T min_of(T a, T b)
{
    return b < a ? b : a;
}
template<typename T>
inline T max_of(T a, T b)
{
    return a < b ? b : a;
}

inline std::uint8_t clamp_to_byte(int value)
{
    return static_cast<std::uint8_t>(value < 0 ? 0 : (value > 255 ? 255 : value));
}

inline std::uint8_t mix_byte(std::uint8_t dst, std::uint8_t src, float alpha)
{
    const float ca = alpha < 0.0f ? 0.0f : (alpha > 1.0f ? 1.0f : alpha);
    const float ia = 1.0f - ca;
    return clamp_to_byte(
            static_cast<int>(lroundf(static_cast<float>(src) * ca + static_cast<float>(dst) * ia)));
}

Color blend_src_over(const Color& src, const Color& dst, float opacity)
{
    const float co = opacity < 0.0f ? 0.0f : (opacity > 1.0f ? 1.0f : opacity);
    if (co <= 0.0f) return dst;

    const std::uint32_t srcA = clamp_to_byte(
            static_cast<int>(lroundf((static_cast<float>(src.a()) / 255.0f) * co * 255.0f)));
    if (srcA == 0) return dst;

    const std::uint32_t srcR = src.r(), srcG = src.g(), srcB = src.b();
    std::uint32_t dstA = dst.a(), dstR = dst.r(), dstG = dst.g(), dstB = dst.b();

    dstA -= (dstA * srcA) / 255;
    const std::uint32_t outA = dstA + srcA;
    if (outA == 0) return Color(0, 0, 0, 0);

    return Color(static_cast<std::uint8_t>((dstR * dstA + srcR * srcA) / outA),
            static_cast<std::uint8_t>((dstG * dstA + srcG * srcA) / outA),
            static_cast<std::uint8_t>((dstB * dstA + srcB * srcA) / outA),
            static_cast<std::uint8_t>(outA));
}

bool is_dds(const unsigned char* buf, std::uint32_t sz)
{
    return sz >= 4 && buf[0] == 'D' && buf[1] == 'D' && buf[2] == 'S' && buf[3] == ' ';
}

bool assign_loaded_pixels(Pixmap& pm, const unsigned char* src, int w, int h, int src_comp);

bool load_dds(Pixmap& pm, const unsigned char* buf, std::uint32_t sz)
{
    const dds::Header header = dds::read_header(buf, sz);
    if (!header.is_valid())
    {
        log_error("DDS: invalid header");
        return false;
    }
    if (header.is_1d() || header.is_3d() || header.is_cubemap() || header.array_size() != 1)
    {
        log_error("DDS: only a 2D texture can be loaded into a Pixmap");
        return false;
    }
    const unsigned width = header.width();
    const unsigned height = header.height();
    if (width == 0 || height == 0 || width > 0x7fffffffu || height > 0x7fffffffu)
    {
        log_error("DDS: bad size %ux%u", width, height);
        return false;
    }
    const unsigned long long offset = header.mip_offset(0, 0);
    const unsigned long long bytes = header.mip_size(0);
    if (offset > sz || bytes > sz - offset)
    {
        log_error("DDS: file is shorter than its first mip level");
        return false;
    }
    const unsigned char* data = buf + offset;
    const std::size_t pixelCount = static_cast<std::size_t>(width) * height;

    BcnKind kind = BcnKind::BC1;
    bool compressed = true;
    switch (header.format())
    {
        case dds::DXGI_FORMAT_BC1_UNORM:
        case dds::DXGI_FORMAT_BC1_UNORM_SRGB:
            kind = BcnKind::BC1;
            break;
        case dds::DXGI_FORMAT_BC2_UNORM:
        case dds::DXGI_FORMAT_BC2_UNORM_SRGB:
            kind = BcnKind::BC2;
            break;
        case dds::DXGI_FORMAT_BC3_UNORM:
        case dds::DXGI_FORMAT_BC3_UNORM_SRGB:
            kind = BcnKind::BC3;
            break;
        default:
            compressed = false;
            break;
    }

    if (compressed)
    {
        ct::Vector<unsigned char> rgba;
        rgba.resize(pixelCount * 4);
        if (!decodeBcn(kind, data, static_cast<std::size_t>(bytes), width, height, rgba.data()))
        {
            log_error("DDS: block data is shorter than the image");
            return false;
        }
        return assign_loaded_pixels(pm, rgba.data(), static_cast<int>(width),
                static_cast<int>(height), 4);
    }

    switch (header.format())
    {
        case dds::DXGI_FORMAT_R8G8B8A8_UNORM:
        case dds::DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
            return assign_loaded_pixels(pm, data, static_cast<int>(width), static_cast<int>(height),
                    4);
        case dds::DXGI_FORMAT_R8_UNORM:
            return assign_loaded_pixels(pm, data, static_cast<int>(width), static_cast<int>(height),
                    1);
        case dds::DXGI_FORMAT_B8G8R8A8_UNORM:
        case dds::DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
        case dds::DXGI_FORMAT_B8G8R8X8_UNORM:
        {
            const bool opaque = header.format() == dds::DXGI_FORMAT_B8G8R8X8_UNORM;
            ct::Vector<unsigned char> rgba;
            rgba.resize(pixelCount * 4);
            for (std::size_t i = 0; i < pixelCount * 4; i += 4)
            {
                rgba[i] = data[i + 2];
                rgba[i + 1] = data[i + 1];
                rgba[i + 2] = data[i];
                rgba[i + 3] = opaque ? 255 : data[i + 3];
            }
            return assign_loaded_pixels(pm, rgba.data(), static_cast<int>(width),
                    static_cast<int>(height), 4);
        }
        default:
            log_error("DDS: format %d is not supported by Pixmap",
                    static_cast<int>(header.format()));
            return false;
    }
}

int select_components(int src_comp)
{
    return (src_comp == 2 || src_comp == 4) ? 4 : 3;
}

bool has_extension(const char* fn, const char* ext)
{
    if (!fn || !ext) return false;
    const char* dot = strrchr(fn, '.');
    if (!dot) return false;
    return strcmp(dot, ext) == 0;
}

bool assign_loaded_pixels(Pixmap& pm, const unsigned char* src, int w, int h, int src_comp)
{
    if (!src || w <= 0 || h <= 0 || src_comp < 1 || src_comp > 4) return false;

    const int nc = select_components(src_comp);
    const std::size_t total = static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4;
    unsigned char* np = static_cast<unsigned char*>(malloc(total));
    if (!np)
    {
        log_error("malloc failed");
        return false;
    }
    memset(np, 0, total);

    if (nc == src_comp)
    {
        memcpy(np, src,
                static_cast<std::size_t>(w) * static_cast<std::size_t>(h) *
                        static_cast<std::size_t>(nc));
    }
    else
    {
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
            {
                const std::size_t si = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                               static_cast<std::size_t>(x)) *
                                       static_cast<std::size_t>(src_comp);
                const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                               static_cast<std::size_t>(x)) *
                                       static_cast<std::size_t>(nc);
                const unsigned char r = src[si + 0];
                const unsigned char g = (src_comp >= 3) ? src[si + 1] : r;
                const unsigned char b = (src_comp >= 3) ? src[si + 2] : r;
                const unsigned char a = (src_comp == 2)   ? src[si + 1]
                                        : (src_comp == 4) ? src[si + 3]
                                                          : 255;
                if (nc == 3)
                {
                    np[di + 0] = r;
                    np[di + 1] = g;
                    np[di + 2] = b;
                }
                else
                {
                    np[di + 0] = r;
                    np[di + 1] = g;
                    np[di + 2] = b;
                    np[di + 3] = a;
                }
            }
    }

    if (pm.pixels) free(pm.pixels);
    pm.pixels = np;
    pm.width = w;
    pm.height = h;
    pm.components = nc;
    return true;
}

bool build_save_buf(const Pixmap& pm, ct::Vector<unsigned char>& scr, int& sc,
        const unsigned char*& sp)
{
    if (!pm.pixels || pm.width <= 0 || pm.height <= 0) return false;
    if (pm.components == 1 || pm.components == 3 || pm.components == 4)
    {
        sc = pm.components;
        sp = pm.pixels;
        return true;
    }
    sc = 4;
    scr.resize(static_cast<std::size_t>(pm.width) * static_cast<std::size_t>(pm.height) * 4);
    for (int y = 0; y < pm.height; y++)
        for (int x = 0; x < pm.width; x++)
        {
            const std::size_t di = static_cast<std::size_t>(y * pm.width + x) * 4;
            const Color c = pm.get_pixel_color(x, y);
            scr[di + 0] = c.r();
            scr[di + 1] = c.g();
            scr[di + 2] = c.b();
            scr[di + 3] = c.a();
        }
    sp = scr.data();
    return true;
}

bool save_tga(const Pixmap& pm, const char* fn)
{
    const bool gray = (pm.components == 1);
    const int bpp = gray ? 8 : ((pm.components == 4 || pm.components == 2) ? 32 : 24);
    unsigned char hdr[18] = {};
    hdr[2] = gray ? 3 : 2;
    hdr[12] = static_cast<unsigned char>(pm.width & 0xFF);
    hdr[13] = static_cast<unsigned char>((pm.width >> 8) & 0xFF);
    hdr[14] = static_cast<unsigned char>(pm.height & 0xFF);
    hdr[15] = static_cast<unsigned char>((pm.height >> 8) & 0xFF);
    hdr[16] = static_cast<unsigned char>(bpp);
    hdr[17] = static_cast<unsigned char>((bpp == 32 ? 8 : 0) | 0x20);

    const std::size_t row_bytes =
            static_cast<std::size_t>(pm.width) * static_cast<std::size_t>(bpp / 8);
    ct::Vector<unsigned char> data(sizeof(hdr) + row_bytes * static_cast<std::size_t>(pm.height));
    memcpy(data.data(), hdr, sizeof(hdr));

    ct::Vector<unsigned char> row(row_bytes);
    for (int y = 0; y < pm.height; y++)
    {
        if (gray)
            for (int x = 0; x < pm.width; x++) row[x] = pm.get_pixel_color(x, y).r();
        else if (bpp == 24)
            for (int x = 0; x < pm.width; x++)
            {
                const Color c = pm.get_pixel_color(x, y);
                const std::size_t o = static_cast<std::size_t>(x) * 3;
                row[o + 0] = c.b();
                row[o + 1] = c.g();
                row[o + 2] = c.r();
            }
        else
            for (int x = 0; x < pm.width; x++)
            {
                const Color c = pm.get_pixel_color(x, y);
                const std::size_t o = static_cast<std::size_t>(x) * 4;
                row[o + 0] = c.b();
                row[o + 1] = c.g();
                row[o + 2] = c.r();
                row[o + 3] = c.a();
            }
        memcpy(data.data() + sizeof(hdr) + static_cast<std::size_t>(y) * row_bytes, row.data(),
                row.size());
    }

    if (!file_write(fn, data.data(), data.size()))
    {
        log_error("TGA open failed: %s", fn ? fn : "");
        return false;
    }
    return true;
}

} // namespace

Pixmap::Pixmap()
        : pixels(nullptr),
          components(0),
          width(0),
          height(0)
{
}

Pixmap::~Pixmap()
{
    if (pixels) free(pixels);
}

Pixmap::Pixmap(const Pixmap& img, const IntRect& crop)
{
    width = crop.width;
    height = crop.height;
    components = img.components;
    pixels = static_cast<unsigned char*>(
            malloc(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4));
    memset(pixels, 0, static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4);
    for (int y = crop.y, off = 0; y < crop.y + crop.height; y++)
    {
        memcpy(pixels + off, img.pixels + (y * img.width + crop.x) * components,
                static_cast<std::size_t>(crop.width) * static_cast<std::size_t>(components));
        off += crop.width * components;
    }
}

Pixmap::Pixmap(int w, int h, int comp)
        : components(comp),
          width(w),
          height(h)
{
    pixels = static_cast<unsigned char*>(
            malloc(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4));
    memset(pixels, 0, static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4);
}

Pixmap::Pixmap(int w, int h, int comp, unsigned char* data)
        : components(comp),
          width(w),
          height(h)
{
    pixels = static_cast<unsigned char*>(
            malloc(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4));
    memset(pixels, 0, static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4);
    memcpy(pixels, data,
            static_cast<std::size_t>(w) * static_cast<std::size_t>(h) *
                    static_cast<std::size_t>(comp));
}

void Pixmap::set_pixel(std::uint32_t x, std::uint32_t y, std::uint8_t r, std::uint8_t g,
        std::uint8_t b, std::uint8_t a)
{
    if (x >= static_cast<std::uint32_t>(width) || y >= static_cast<std::uint32_t>(height)) return;

    if (components == 1)
    {
        float gray = static_cast<float>(r) * 0.299f + static_cast<float>(g) * 0.587f +
                     static_cast<float>(b) * 0.114f;
        pixels[y * width + x] = static_cast<unsigned char>(gray);
    }
    else if (components == 2)
    {
        float gray = static_cast<float>(r) * 0.299f + static_cast<float>(g) * 0.587f +
                     static_cast<float>(b) * 0.114f;
        pixels[(y * width + x) * 2] = static_cast<unsigned char>(gray);
        pixels[(y * width + x) * 2 + 1] = a;
    }
    else if (components == 3)
    {
        pixels[(y * width + x) * 3] = r;
        pixels[(y * width + x) * 3 + 1] = g;
        pixels[(y * width + x) * 3 + 2] = b;
    }
    else
    {
        pixels[(y * width + x) * 4] = r;
        pixels[(y * width + x) * 4 + 1] = g;
        pixels[(y * width + x) * 4 + 2] = b;
        pixels[(y * width + x) * 4 + 3] = a;
    }
}

void Pixmap::set_pixel(std::uint32_t x, std::uint32_t y, std::uint32_t rgba)
{
    set_pixel(x, y, static_cast<std::uint8_t>(rgba), static_cast<std::uint8_t>(rgba >> 8),
            static_cast<std::uint8_t>(rgba >> 16), static_cast<std::uint8_t>(rgba >> 24));
}

std::uint32_t Pixmap::get_pixel(std::uint32_t x, std::uint32_t y) const
{
    if (x >= static_cast<std::uint32_t>(width) || y >= static_cast<std::uint32_t>(height)) return 0;

    if (components == 1) return pixels[y * width + x];
    if (components == 2)
        return pixels[(y * width + x) * 2] |
               (static_cast<std::uint32_t>(pixels[(y * width + x) * 2 + 1]) << 8);
    if (components == 3)
        return pixels[(y * width + x) * 3] |
               (static_cast<std::uint32_t>(pixels[(y * width + x) * 3 + 1]) << 8) |
               (static_cast<std::uint32_t>(pixels[(y * width + x) * 3 + 2]) << 16);
    return pixels[(y * width + x) * 4] |
           (static_cast<std::uint32_t>(pixels[(y * width + x) * 4 + 1]) << 8) |
           (static_cast<std::uint32_t>(pixels[(y * width + x) * 4 + 2]) << 16) |
           (static_cast<std::uint32_t>(pixels[(y * width + x) * 4 + 3]) << 24);
}

Color Pixmap::get_pixel_color(std::uint32_t x, std::uint32_t y) const
{
    if (x >= static_cast<std::uint32_t>(width) || y >= static_cast<std::uint32_t>(height))
        return Color(0, 0, 0, 0);

    std::uint8_t r = 0, g = 0, b = 0, a = 255;
    if (components == 1) r = g = b = pixels[y * width + x];
    else if (components == 2)
    {
        r = g = b = pixels[(y * width + x) * 2];
        a = pixels[(y * width + x) * 2 + 1];
    }
    else if (components == 3)
    {
        r = pixels[(y * width + x) * 3];
        g = pixels[(y * width + x) * 3 + 1];
        b = pixels[(y * width + x) * 3 + 2];
    }
    else
    {
        r = pixels[(y * width + x) * 4];
        g = pixels[(y * width + x) * 4 + 1];
        b = pixels[(y * width + x) * 4 + 2];
        a = pixels[(y * width + x) * 4 + 3];
    }
    return Color(r, g, b, a);
}

void Pixmap::fill(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a)
{
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), r, g, b, a);
}

void Pixmap::fill(std::uint32_t rgba)
{
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), rgba);
}

void Pixmap::clear()
{
    if (pixels)
        memset(pixels, 0,
                static_cast<std::size_t>(width) * static_cast<std::size_t>(height) *
                        static_cast<std::size_t>(components));
}

bool Pixmap::load(const char* fn)
{
    std::size_t size = 0;
    std::uint8_t* data = file_read(fn, &size);
    if (!data || size == 0)
    {
        if (data) fs_free(data);
        log_error("Failed to load image: %s", fn ? fn : "");
        return false;
    }
    bool ok = load_from_memory(data, static_cast<std::uint32_t>(size));
    fs_free(data);
    if (ok) log_info("Load: %s (%d,%d) bpp:%d", fn, width, height, components);
    return ok;
}

bool Pixmap::load(IoStream* stream)
{
    std::size_t size = 0;
    void* data = stream ? io_load(stream, &size, false) : nullptr;
    if (!data || size == 0)
    {
        if (data) fs_free(data);
        log_error("Failed to load image: %s", "stream");
        return false;
    }
    bool ok = load_from_memory(static_cast<const unsigned char*>(data),
            static_cast<std::uint32_t>(size));
    fs_free(data);
    return ok;
}

bool Pixmap::load_from_memory(const unsigned char* buf, std::uint32_t sz)
{
    if (!buf || sz == 0)
    {
        log_error("Invalid buffer");
        return false;
    }
    if (is_dds(buf, sz)) return load_dds(*this, buf, sz);

    int w = 0, h = 0, sc = 0;
    unsigned char* ld = stbi_load_from_memory(buf, static_cast<int>(sz), &w, &h, &sc, 0);
    if (!ld)
    {
        log_error("Failed to load from memory: %s",
                stbi_failure_reason() ? stbi_failure_reason() : "");
        return false;
    }
    bool ok = assign_loaded_pixels(*this, ld, w, h, sc);
    stbi_image_free(ld);
    return ok;
}

bool Pixmap::is_16_bit_in_memory(const unsigned char* buffer, std::uint32_t bytesRead)
{
    return buffer && bytesRead > 0 &&
           stbi_is_16_bit_from_memory(buffer, static_cast<int>(bytesRead));
}

bool Pixmap::load_16_from_memory(const unsigned char* buffer, std::uint32_t bytesRead,
        ct::Vector<std::uint16_t>& out, int& width, int& height)
{
    out.clear();
    width = height = 0;
    if (!buffer || bytesRead == 0) return false;
    int w = 0, h = 0, channels = 0;
    stbi_us* loaded =
            stbi_load_16_from_memory(buffer, static_cast<int>(bytesRead), &w, &h, &channels, 1);
    if (!loaded)
    {
        log_error("Failed to load 16-bit image from memory: %s",
                stbi_failure_reason() ? stbi_failure_reason() : "");
        return false;
    }
    const std::size_t count = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    out.resize(count);
    for (std::size_t i = 0; i < count; ++i) out[i] = loaded[i];
    stbi_image_free(loaded);
    width = w;
    height = h;
    return true;
}

bool Pixmap::save(const char* fn)
{
    if (!pixels)
    {
        log_error("Null pixels, cannot save: %s", fn ? fn : "");
        return false;
    }
    if (has_extension(fn, ".tga")) return save_tga(*this, fn);

    ct::Vector<unsigned char> scr;
    int sc = 0;
    const unsigned char* sp = nullptr;
    if (!build_save_buf(*this, scr, sc, sp))
    {
        log_error("Build save buffer failed: %s", fn ? fn : "");
        return false;
    }

    bool ok = false;
    if (has_extension(fn, ".bmp"))
    {
        ok = stbi_write_bmp(fn, width, height, sc, sp) != 0;
        if (!ok) log_error("BMP save failed: %s", fn ? fn : "");
    }
    else if (has_extension(fn, ".png"))
    {
        ok = stbi_write_png(fn, width, height, sc, sp, width * sc) != 0;
        if (!ok) log_error("PNG save failed: %s", fn ? fn : "");
    }
    else
        log_error("Unsupported format: %s", fn ? fn : "");
    return ok;
}

void Pixmap::flip_vertical()
{
    if (!pixels) return;
    int rs = width * components;
    unsigned char* row = static_cast<unsigned char*>(malloc(static_cast<std::size_t>(rs)));
    for (int y = 0; y < height / 2; y++)
    {
        unsigned char* s = pixels + y * rs;
        unsigned char* d = pixels + (height - y - 1) * rs;
        memcpy(row, s, static_cast<std::size_t>(rs));
        memcpy(s, d, static_cast<std::size_t>(rs));
        memcpy(d, row, static_cast<std::size_t>(rs));
    }
    free(row);
}

void Pixmap::flip_horizontal()
{
    if (!pixels) return;
    int rs = width * components;
    unsigned char* tmp = static_cast<unsigned char*>(malloc(static_cast<std::size_t>(components)));
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width / 2; x++)
        {
            unsigned char* s = pixels + y * rs + x * components;
            unsigned char* d = pixels + y * rs + (width - x - 1) * components;
            memcpy(tmp, s, static_cast<std::size_t>(components));
            memcpy(s, d, static_cast<std::size_t>(components));
            memcpy(d, tmp, static_cast<std::size_t>(components));
        }
    free(tmp);
}

void Pixmap::tint(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>((c.r() * r) / 255),
                    static_cast<std::uint8_t>((c.g() * g) / 255),
                    static_cast<std::uint8_t>((c.b() * b) / 255), c.a());
        }
}

void Pixmap::fix_alpha_edges()
{
    if (!pixels || components != 4 || width <= 0 || height <= 0) return;

    const std::size_t bytes =
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4;
    ct::Vector<unsigned char> copy;
    copy.resize(bytes, 0);
    memcpy(copy.data(), pixels, bytes);
    const unsigned char* source = copy.data();

    const int maxRadius = 4;
    const int alphaThreshold = 20;
    const int maxDist = 0x7FFFFFFF;

    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            const unsigned char* read = &source[(i * width + j) * 4];
            unsigned char* write = &pixels[(i * width + j) * 4];
            if (read[3] >= alphaThreshold) continue;

            int closestDist = maxDist;
            unsigned char closestColor[3] = { 0, 0, 0 };

            const int fromX = j - maxRadius > 0 ? j - maxRadius : 0;
            const int toX = j + maxRadius < width - 1 ? j + maxRadius : width - 1;
            const int fromY = i - maxRadius > 0 ? i - maxRadius : 0;
            const int toY = i + maxRadius < height - 1 ? i + maxRadius : height - 1;

            for (int k = fromY; k <= toY; ++k)
            {
                for (int l = fromX; l <= toX; ++l)
                {
                    const int dy = i - k;
                    const int dx = j - l;
                    const int dist = dy * dy + dx * dx;
                    if (dist >= closestDist) continue;

                    const unsigned char* other = &source[(k * width + l) * 4];
                    if (other[3] < alphaThreshold) continue;

                    closestDist = dist;
                    closestColor[0] = other[0];
                    closestColor[1] = other[1];
                    closestColor[2] = other[2];
                }
            }

            if (closestDist != maxDist)
            {
                write[0] = closestColor[0];
                write[1] = closestColor[1];
                write[2] = closestColor[2];
            }
        }
    }
}

void Pixmap::fill_transparent_color()
{
    if (!pixels || components != 4 || width <= 0 || height <= 0) return;

    const int alphaThreshold = 20;
    const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    ct::Vector<std::uint8_t> filled;
    filled.resize(count, 0);
    ct::Vector<std::uint32_t> queue;
    queue.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        if (pixels[i * 4 + 3] < alphaThreshold) continue;
        filled[i] = 1;
        queue.push_back(static_cast<std::uint32_t>(i));
    }

    const int stepX[4] = { 1, -1, 0, 0 };
    const int stepY[4] = { 0, 0, 1, -1 };
    for (std::size_t head = 0; head < queue.size(); ++head)
    {
        const std::uint32_t index = queue[head];
        const int x = static_cast<int>(index % static_cast<std::uint32_t>(width));
        const int y = static_cast<int>(index / static_cast<std::uint32_t>(width));
        for (int n = 0; n < 4; ++n)
        {
            const int nx = x + stepX[n];
            const int ny = y + stepY[n];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            const std::size_t next =
                    static_cast<std::size_t>(ny) * static_cast<std::size_t>(width) +
                    static_cast<std::size_t>(nx);
            if (filled[next]) continue;
            filled[next] = 1;
            pixels[next * 4 + 0] = pixels[index * 4 + 0];
            pixels[next * 4 + 1] = pixels[index * 4 + 1];
            pixels[next * 4 + 2] = pixels[index * 4 + 2];
            queue.push_back(static_cast<std::uint32_t>(next));
        }
    }
}

Pixmap* Pixmap::convert_to_rgba() const
{
    if (components == 4) return new Pixmap(width, height, components, pixels);
    Pixmap* r = new Pixmap(width, height, 4);
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), c.r(), c.g(),
                    c.b(), c.a());
        }
    return r;
}

Pixmap* Pixmap::resize(int nw, int nh) const
{
    Pixmap* r = new Pixmap(nw, nh, components);
    for (int y = 0; y < nh; y++)
        for (int x = 0; x < nw; x++)
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    get_pixel((x * width) / nw, (y * height) / nh));
    return r;
}

void Pixmap::draw_line(int x1, int y1, int x2, int y2, const Color& color)
{
    int dx = abs(x2 - x1), dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1, sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    while (true)
    {
        set_pixel(static_cast<std::uint32_t>(x1), static_cast<std::uint32_t>(y1), color.r(),
                color.g(), color.b(), color.a());
        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

void Pixmap::draw_rect(int x, int y, int w, int h, const Color& color, bool filled)
{
    if (!pixels) return;
    if (filled)
    {
        for (int dy = 0; dy < h; dy++)
            for (int dx = 0; dx < w; dx++)
            {
                int px = x + dx, py = y + dy;
                if (px >= 0 && px < width && py >= 0 && py < height)
                    set_pixel(static_cast<std::uint32_t>(px), static_cast<std::uint32_t>(py),
                            color.r(), color.g(), color.b(), color.a());
            }
    }
    else
    {
        for (int dx = 0; dx < w; dx++)
        {
            if (x + dx >= 0 && x + dx < width && y >= 0 && y < height)
                set_pixel(x + dx, y, color.r(), color.g(), color.b(), color.a());
            if (x + dx >= 0 && x + dx < width && y + h - 1 >= 0 && y + h - 1 < height)
                set_pixel(x + dx, y + h - 1, color.r(), color.g(), color.b(), color.a());
        }
        for (int dy = 0; dy < h; dy++)
        {
            if (x >= 0 && x < width && y + dy >= 0 && y + dy < height)
                set_pixel(x, y + dy, color.r(), color.g(), color.b(), color.a());
            if (x + w - 1 >= 0 && x + w - 1 < width && y + dy >= 0 && y + dy < height)
                set_pixel(x + w - 1, y + dy, color.r(), color.g(), color.b(), color.a());
        }
    }
}

void Pixmap::draw_circle(int cx, int cy, int radius, const Color& color, bool filled)
{
    if (!pixels) return;
    if (filled)
    {
        int r2 = radius * radius;
        for (int y = -radius; y <= radius; y++)
            for (int x = -radius; x <= radius; x++)
                if (x * x + y * y <= r2)
                {
                    int px = cx + x, py = cy + y;
                    if (px >= 0 && px < width && py >= 0 && py < height)
                        set_pixel(static_cast<std::uint32_t>(px), static_cast<std::uint32_t>(py),
                                color.r(), color.g(), color.b(), color.a());
                }
    }
    else
    {
        int x = 0, y = radius, d = 3 - 2 * radius;
        while (x <= y)
        {
            auto pt = [&](int px, int py)
            {
                if (px >= 0 && px < width && py >= 0 && py < height)
                    set_pixel(static_cast<std::uint32_t>(px), static_cast<std::uint32_t>(py),
                            color.r(), color.g(), color.b(), color.a());
            };
            pt(cx + x, cy + y);
            pt(cx - x, cy + y);
            pt(cx + x, cy - y);
            pt(cx - x, cy - y);
            pt(cx + y, cy + x);
            pt(cx - y, cy + x);
            pt(cx + y, cy - x);
            pt(cx - y, cy - x);
            if (d < 0) d += 4 * x + 6;
            else
            {
                d += 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    }
}

void Pixmap::blend_pixel(std::uint32_t x, std::uint32_t y, const Color& color, float opacity,
        BlendMode mode)
{
    if (x >= static_cast<std::uint32_t>(width) || y >= static_cast<std::uint32_t>(height) ||
            !pixels)
        return;
    const float co = opacity < 0.0f ? 0.0f : (opacity > 1.0f ? 1.0f : opacity);
    if (co <= 0.0f) return;

    const Color dst = get_pixel_color(x, y);
    Color out = dst;

    switch (mode)
    {
        case BlendMode::copy:
            out = Color(color.r(), color.g(), color.b(), mix_byte(dst.a(), color.a(), co));
            break;
        case BlendMode::add:
            out = Color(clamp_to_byte(dst.r() + static_cast<int>(lroundf(
                                                        static_cast<float>(color.r()) * co))),
                    clamp_to_byte(dst.g() +
                                  static_cast<int>(lroundf(static_cast<float>(color.g()) * co))),
                    clamp_to_byte(dst.b() +
                                  static_cast<int>(lroundf(static_cast<float>(color.b()) * co))),
                    clamp_to_byte(dst.a() +
                                  static_cast<int>(lroundf(static_cast<float>(color.a()) * co))));
            break;
        case BlendMode::multiply:
            out = Color(
                    mix_byte(dst.r(),
                            clamp_to_byte(static_cast<int>(
                                    (static_cast<float>(dst.r()) * static_cast<float>(color.r())) /
                                    255.0f)),
                            co),
                    mix_byte(dst.g(),
                            clamp_to_byte(static_cast<int>(
                                    (static_cast<float>(dst.g()) * static_cast<float>(color.g())) /
                                    255.0f)),
                            co),
                    mix_byte(dst.b(),
                            clamp_to_byte(static_cast<int>(
                                    (static_cast<float>(dst.b()) * static_cast<float>(color.b())) /
                                    255.0f)),
                            co),
                    mix_byte(dst.a(), color.a(), co));
            break;
        default:
            out = blend_src_over(color, dst, co);
            break;
    }
    set_pixel(x, y, out.r(), out.g(), out.b(), out.a());
}

void Pixmap::draw_pixmap(const Pixmap& src, int x, int y)
{
    if (!pixels || !src.pixels) return;
    for (int sy = 0; sy < src.height; sy++)
        for (int sx = 0; sx < src.width; sx++)
        {
            int dx = x + sx, dy = y + sy;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height) continue;
            Color c = src.get_pixel_color(sx, sy);
            if (src.components == 4 || src.components == 2)
            {
                if (c.a() == 255)
                    set_pixel(static_cast<std::uint32_t>(dx), static_cast<std::uint32_t>(dy), c.r(),
                            c.g(), c.b(), c.a());
                else if (c.a() > 0)
                {
                    Color dd = get_pixel_color(dx, dy);
                    float al = c.a() / 255.0f;
                    set_pixel(static_cast<std::uint32_t>(dx), static_cast<std::uint32_t>(dy),
                            static_cast<std::uint8_t>(c.r() * al + dd.r() * (1.0f - al)),
                            static_cast<std::uint8_t>(c.g() * al + dd.g() * (1.0f - al)),
                            static_cast<std::uint8_t>(c.b() * al + dd.b() * (1.0f - al)), dd.a());
                }
            }
            else
                set_pixel(static_cast<std::uint32_t>(dx), static_cast<std::uint32_t>(dy), c.r(),
                        c.g(), c.b(), 255);
        }
}

void Pixmap::draw_pixmap(const Pixmap& src, int x, int y, const IntRect& sr)
{
    if (!pixels || !src.pixels) return;
    int sx = sr.x, sy = sr.y, sw = sr.width, sh = sr.height;
    if (sx < 0)
    {
        sw += sx;
        x -= sx;
        sx = 0;
    }
    if (sy < 0)
    {
        sh += sy;
        y -= sy;
        sy = 0;
    }
    if (sx + sw > src.width) sw = src.width - sx;
    if (sy + sh > src.height) sh = src.height - sy;

    for (int iy = 0; iy < sh; iy++)
        for (int ix = 0; ix < sw; ix++)
        {
            int dx = x + ix, dy = y + iy;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height) continue;
            Color c = src.get_pixel_color(sx + ix, sy + iy);
            if (src.components == 4 || src.components == 2)
            {
                if (c.a() == 255)
                    set_pixel(static_cast<std::uint32_t>(dx), static_cast<std::uint32_t>(dy), c.r(),
                            c.g(), c.b(), c.a());
                else if (c.a() > 0)
                {
                    Color dd = get_pixel_color(dx, dy);
                    float al = c.a() / 255.0f;
                    set_pixel(static_cast<std::uint32_t>(dx), static_cast<std::uint32_t>(dy),
                            static_cast<std::uint8_t>(c.r() * al + dd.r() * (1.0f - al)),
                            static_cast<std::uint8_t>(c.g() * al + dd.g() * (1.0f - al)),
                            static_cast<std::uint8_t>(c.b() * al + dd.b() * (1.0f - al)), dd.a());
                }
            }
            else
                set_pixel(static_cast<std::uint32_t>(dx), static_cast<std::uint32_t>(dy), c.r(),
                        c.g(), c.b(), 255);
        }
}

void Pixmap::draw_pixmap_blended(const Pixmap& src, int x, int y, float opacity, BlendMode mode)
{
    draw_pixmap_blended(src, x, y, IntRect(0, 0, src.width, src.height), opacity, mode);
}

void Pixmap::draw_pixmap_blended(const Pixmap& src, int x, int y, const IntRect& sr, float opacity,
        BlendMode mode)
{
    if (!pixels || !src.pixels) return;
    int sx = sr.x, sy = sr.y, sw = sr.width, sh = sr.height;
    if (sx < 0)
    {
        sw += sx;
        x -= sx;
        sx = 0;
    }
    if (sy < 0)
    {
        sh += sy;
        y -= sy;
        sy = 0;
    }
    if (sx + sw > src.width) sw = src.width - sx;
    if (sy + sh > src.height) sh = src.height - sy;

    for (int iy = 0; iy < sh; iy++)
        for (int ix = 0; ix < sw; ix++)
        {
            int dx = x + ix, dy = y + iy;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height) continue;
            blend_pixel(static_cast<std::uint32_t>(dx), static_cast<std::uint32_t>(dy),
                    src.get_pixel_color(sx + ix, sy + iy), opacity, mode);
        }
}

void Pixmap::copy_region(const Pixmap& src, const IntRect& sr, int dx, int dy)
{
    if (!src.pixels || !pixels) return;
    if (src.components != components) return;

    int sx = sr.x, sy = sr.y, sw = sr.width, sh = sr.height;
    if (sx < 0)
    {
        sw += sx;
        dx -= sx;
        sx = 0;
    }
    if (sy < 0)
    {
        sh += sy;
        dy -= sy;
        sy = 0;
    }
    if (sx + sw > src.width) sw = src.width - sx;
    if (sy + sh > src.height) sh = src.height - sy;

    for (int y = 0; y < sh; y++)
        for (int x = 0; x < sw; x++)
        {
            int tx = dx + x, ty = dy + y;
            if (tx >= 0 && tx < width && ty >= 0 && ty < height)
            {
                Color c = src.get_pixel_color(sx + x, sy + y);
                set_pixel(static_cast<std::uint32_t>(tx), static_cast<std::uint32_t>(ty), c.r(),
                        c.g(), c.b(), c.a());
            }
        }
}

void Pixmap::replace_color(const Color& from, const Color& to, float threshold)
{
    if (!pixels) return;
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            float dr = fabsf(static_cast<float>(c.r()) - static_cast<float>(from.r())) / 255.0f;
            float dg = fabsf(static_cast<float>(c.g()) - static_cast<float>(from.g())) / 255.0f;
            float db = fabsf(static_cast<float>(c.b()) - static_cast<float>(from.b())) / 255.0f;
            float da = fabsf(static_cast<float>(c.a()) - static_cast<float>(from.a())) / 255.0f;
            if ((dr + dg + db + da) / 4.0f <= threshold)
                set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), to.r(),
                        to.g(), to.b(), to.a());
        }
}

void Pixmap::set_color_key(const Color& key, float threshold)
{
    if (!pixels || (components != 4 && components != 2)) return;
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            Color c = get_pixel_color(x, y);
            float dr = fabsf(static_cast<float>(c.r()) - static_cast<float>(key.r())) / 255.0f;
            float dg = fabsf(static_cast<float>(c.g()) - static_cast<float>(key.g())) / 255.0f;
            float db = fabsf(static_cast<float>(c.b()) - static_cast<float>(key.b())) / 255.0f;
            if ((dr + dg + db) / 3.0f <= threshold)
                set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), c.r(),
                        c.g(), c.b(), 0);
        }
}

Pixmap* Pixmap::apply_blur(int radius) const
{
    if (!pixels || radius < 1) return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    int ks = radius * 2 + 1, ka = ks * ks;
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            int sr = 0, sg = 0, sb = 0, sa = 0;
            for (int ky = -radius; ky <= radius; ky++)
                for (int kx = -radius; kx <= radius; kx++)
                {
                    int px = max_of(0, min_of(width - 1, x + kx));
                    int py = max_of(0, min_of(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    sr += c.r();
                    sg += c.g();
                    sb += c.b();
                    sa += c.a();
                }
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>(sr / ka), static_cast<std::uint8_t>(sg / ka),
                    static_cast<std::uint8_t>(sb / ka), static_cast<std::uint8_t>(sa / ka));
        }
    return r;
}

Pixmap* Pixmap::apply_gaussian_blur(int) const
{
    if (!pixels) return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float k[3][3] = { { 1 / 16.0f, 2 / 16.0f, 1 / 16.0f }, { 2 / 16.0f, 4 / 16.0f, 2 / 16.0f },
        { 1 / 16.0f, 2 / 16.0f, 1 / 16.0f } };
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float sr = 0, sg = 0, sb = 0, sa = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = max_of(0, min_of(width - 1, x + kx)),
                        py = max_of(0, min_of(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float w = k[ky + 1][kx + 1];
                    sr += c.r() * w;
                    sg += c.g() * w;
                    sb += c.b() * w;
                    sa += c.a() * w;
                }
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>(sr), static_cast<std::uint8_t>(sg),
                    static_cast<std::uint8_t>(sb), static_cast<std::uint8_t>(sa));
        }
    return r;
}

Pixmap* Pixmap::apply_sharpen() const
{
    if (!pixels) return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float k[3][3] = { { 0, -1, 0 }, { -1, 5, -1 }, { 0, -1, 0 } };
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float sr = 0, sg = 0, sb = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = max_of(0, min_of(width - 1, x + kx)),
                        py = max_of(0, min_of(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float w = k[ky + 1][kx + 1];
                    sr += c.r() * w;
                    sg += c.g() * w;
                    sb += c.b() * w;
                }
            sr = max_of(0.0f, min_of(sr, 255.0f));
            sg = max_of(0.0f, min_of(sg, 255.0f));
            sb = max_of(0.0f, min_of(sb, 255.0f));
            Color o = get_pixel_color(x, y);
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>(sr), static_cast<std::uint8_t>(sg),
                    static_cast<std::uint8_t>(sb), o.a());
        }
    return r;
}

Pixmap* Pixmap::apply_edge_detection() const
{
    if (!pixels) return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float sx[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
    float sy[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float gx = 0, gy = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = max_of(0, min_of(width - 1, x + kx)),
                        py = max_of(0, min_of(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float g = c.r() * 0.299f + c.g() * 0.587f + c.b() * 0.114f;
                    gx += g * sx[ky + 1][kx + 1];
                    gy += g * sy[ky + 1][kx + 1];
                }
            float m = min_of(sqrtf(gx * gx + gy * gy), 255.0f);
            std::uint8_t e = static_cast<std::uint8_t>(m);
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), e, e, e,
                    255);
        }
    return r;
}

Pixmap* Pixmap::apply_emboss() const
{
    if (!pixels) return nullptr;
    Pixmap* r = new Pixmap(width, height, components);
    float k[3][3] = { { -2, -1, 0 }, { -1, 1, 1 }, { 0, 1, 2 } };
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float sr = 0, sg = 0, sb = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    int px = max_of(0, min_of(width - 1, x + kx)),
                        py = max_of(0, min_of(height - 1, y + ky));
                    Color c = get_pixel_color(px, py);
                    float w = k[ky + 1][kx + 1];
                    sr += c.r() * w;
                    sg += c.g() * w;
                    sb += c.b() * w;
                }
            sr = max_of(0.0f, min_of(sr + 128.0f, 255.0f));
            sg = max_of(0.0f, min_of(sg + 128.0f, 255.0f));
            sb = max_of(0.0f, min_of(sb + 128.0f, 255.0f));
            Color o = get_pixel_color(x, y);
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>(sr), static_cast<std::uint8_t>(sg),
                    static_cast<std::uint8_t>(sb), o.a());
        }
    return r;
}

Pixmap* Pixmap::generate_heightmap() const
{
    if (!pixels) return nullptr;
    Pixmap* r = new Pixmap(width, height, 1);
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            const Color c = get_pixel_color(x, y);
            const std::uint8_t gray = clamp_to_byte(
                    static_cast<int>(c.r() * 0.299f + c.g() * 0.587f + c.b() * 0.114f));
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), gray, gray,
                    gray, 255);
        }
    return r;
}

Pixmap* Pixmap::generate_normal_map(float strength) const
{
    if (!pixels) return nullptr;
    Pixmap* r = new Pixmap(width, height, 4);
    float sx[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
    float sy[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            float gx = 0.0f, gy = 0.0f;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                {
                    const int px = max_of(0, min_of(width - 1, x + kx));
                    const int py = max_of(0, min_of(height - 1, y + ky));
                    const Color c = get_pixel_color(px, py);
                    const float h = (c.r() * 0.299f + c.g() * 0.587f + c.b() * 0.114f) / 255.0f;
                    gx += h * sx[ky + 1][kx + 1];
                    gy += h * sy[ky + 1][kx + 1];
                }
            float nx = -gx * strength;
            float ny = -gy * strength;
            float nz = 1.0f;
            const float len = sqrtf(nx * nx + ny * ny + nz * nz);
            if (len > 0.00001f)
            {
                nx /= len;
                ny /= len;
                nz /= len;
            }
            const std::uint8_t nr = clamp_to_byte(static_cast<int>((nx * 0.5f + 0.5f) * 255.0f));
            const std::uint8_t ng = clamp_to_byte(static_cast<int>((ny * 0.5f + 0.5f) * 255.0f));
            const std::uint8_t nb = clamp_to_byte(static_cast<int>((nz * 0.5f + 0.5f) * 255.0f));
            r->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), nr, ng, nb,
                    255);
        }
    return r;
}

Pixmap* Pixmap::crop(const IntRect& rect) const
{
    if (!pixels) return nullptr;
    int x = rect.x, y = rect.y, w = rect.width, h = rect.height;
    if (x < 0)
    {
        w += x;
        x = 0;
    }
    if (y < 0)
    {
        h += y;
        y = 0;
    }
    if (x >= width || y >= height) return nullptr;
    if (x + w > width) w = width - x;
    if (y + h > height) h = height - y;
    if (w <= 0 || h <= 0) return nullptr;

    Pixmap* r = new Pixmap(w, h, components);
    if (!r->pixels)
    {
        delete r;
        return nullptr;
    }
    int bpr = w * components;
    for (int row = 0; row < h; row++)
        memcpy(r->pixels + row * bpr, pixels + ((y + row) * width + x) * components,
                static_cast<std::size_t>(bpr));
    return r;
}

Pixmap* Pixmap::crop(int x, int y, int w, int h) const
{
    IntRect r(x, y, w, h);
    return crop(r);
}

Pixmap* Pixmap::crop_extended(const IntRect& rect, bool fill_transparent) const
{
    if (!pixels) return nullptr;
    int x = rect.x, y = rect.y, w = rect.width, h = rect.height;
    if (w <= 0 || h <= 0) return nullptr;

    Pixmap* r = new Pixmap(w, h, components);
    if (!r->pixels)
    {
        delete r;
        return nullptr;
    }

    if (fill_transparent && (components == 4 || components == 2)) r->clear();
    else
        r->fill(static_cast<std::uint8_t>(0), static_cast<std::uint8_t>(0),
                static_cast<std::uint8_t>(0), static_cast<std::uint8_t>(255));

    int sx0 = max_of(0, x), sy0 = max_of(0, y);
    int sx1 = min_of(width, x + w), sy1 = min_of(height, y + h);
    if (sx0 >= sx1 || sy0 >= sy1) return r;

    int dox = sx0 - x, doy = sy0 - y;
    int cw = sx1 - sx0, ch = sy1 - sy0, bpr = cw * components;
    for (int row = 0; row < ch; row++)
        memcpy(r->pixels + ((doy + row) * w + dox) * components,
                pixels + ((sy0 + row) * width + sx0) * components, static_cast<std::size_t>(bpr));
    return r;
}

} // namespace prisma3d
