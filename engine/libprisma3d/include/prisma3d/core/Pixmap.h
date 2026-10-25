#pragma once

#include <ct/vector.hpp>

#include "prisma3d/core/Color.h"

#include <cstdint>

struct IoStream;

namespace prisma3d
{

struct IntRect
{
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;

    IntRect() = default;
    IntRect(int x_, int y_, int width_, int height_)
            : x(x_),
              y(y_),
              width(width_),
              height(height_)
    {
    }
};

class Pixmap
{
public:
    enum class BlendMode
    {
        copy,
        alpha,
        add,
        multiply
    };

    Pixmap();
    ~Pixmap();
    Pixmap(int w, int h, int components);
    Pixmap(int w, int h, int components, unsigned char* data);
    Pixmap(const Pixmap& image, const IntRect& crop);
    Pixmap(const Pixmap& other) = delete;
    Pixmap& operator=(const Pixmap& other) = delete;

    void set_pixel(std::uint32_t x, std::uint32_t y, std::uint8_t r, std::uint8_t g, std::uint8_t b,
            std::uint8_t a);
    void set_pixel(std::uint32_t x, std::uint32_t y, std::uint32_t rgba);
    std::uint32_t get_pixel(std::uint32_t x, std::uint32_t y) const;
    Color get_pixel_color(std::uint32_t x, std::uint32_t y) const;

    void fill(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a);
    void fill(std::uint32_t rgba);
    void clear();

    bool save(const char* file_name);
    bool load(const char* file_name);
    bool load(IoStream* stream);
    bool load_from_memory(const unsigned char* buffer, std::uint32_t bytesRead);
    static bool is_16_bit_in_memory(const unsigned char* buffer, std::uint32_t bytesRead);
    static bool load_16_from_memory(const unsigned char* buffer, std::uint32_t bytesRead,
            ct::Vector<std::uint16_t>& out, int& width, int& height);

    void flip_vertical();
    void flip_horizontal();
    void tint(std::uint8_t r, std::uint8_t g, std::uint8_t b);

    Pixmap* convert_to_rgba() const;
    void fix_alpha_edges();
    void fill_transparent_color();
    Pixmap* resize(int newWidth, int newHeight) const;
    Pixmap* crop(const IntRect& rect) const;
    Pixmap* crop(int x, int y, int w, int h) const;
    Pixmap* crop_extended(const IntRect& rect, bool fill_transparent = true) const;

    void draw_line(int x1, int y1, int x2, int y2, const Color& color);
    void draw_rect(int x, int y, int w, int h, const Color& color, bool fill = false);
    void draw_circle(int cx, int cy, int radius, const Color& color, bool fill = false);
    void draw_pixmap(const Pixmap& source, int x, int y);
    void draw_pixmap(const Pixmap& source, int x, int y, const IntRect& src_rect);
    void blend_pixel(std::uint32_t x, std::uint32_t y, const Color& color, float opacity = 1.0f,
            BlendMode mode = BlendMode::alpha);
    void draw_pixmap_blended(const Pixmap& source, int x, int y, float opacity = 1.0f,
            BlendMode mode = BlendMode::alpha);
    void draw_pixmap_blended(const Pixmap& source, int x, int y, const IntRect& src_rect,
            float opacity = 1.0f, BlendMode mode = BlendMode::alpha);

    void copy_region(const Pixmap& source, const IntRect& src_rect, int dst_x, int dst_y);

    void replace_color(const Color& from, const Color& to, float threshold = 0.0f);
    void set_color_key(const Color& key, float threshold = 0.0f);

    Pixmap* apply_blur(int radius) const;
    Pixmap* apply_gaussian_blur(int radius) const;
    Pixmap* apply_sharpen() const;
    Pixmap* apply_edge_detection() const;
    Pixmap* apply_emboss() const;

    Pixmap* generate_heightmap() const;
    Pixmap* generate_normal_map(float strength = 2.0f) const;

    bool is_valid() const
    {
        return pixels != nullptr;
    }
    int get_size() const
    {
        return width * height * components;
    }
    bool has_alpha() const
    {
        return components == 2 || components == 4;
    }

    unsigned char* pixels;
    int components;
    int width;
    int height;
};

} // namespace prisma3d
