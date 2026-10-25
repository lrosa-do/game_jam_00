#pragma once

#include <cstdint>

namespace prisma3d
{

class Color
{
public:
    static const std::uint32_t White;
    static const std::uint32_t Black;
    static const std::uint32_t Red;
    static const std::uint32_t Green;
    static const std::uint32_t Blue;
    static const std::uint32_t Yellow;
    static const std::uint32_t Cyan;
    static const std::uint32_t Magenta;
    static const std::uint32_t Orange;
    static const std::uint32_t Gray;
    static const std::uint32_t Transparent;

    Color()
            : mValue(0xFFFFFFFF)
    {
    }
    Color(std::uint32_t argb)
            : mValue(argb)
    {
    }
    Color(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 255);

    std::uint32_t value() const
    {
        return mValue;
    }
    void setValue(std::uint32_t v)
    {
        mValue = v;
    }

    std::uint8_t r() const
    {
        return (std::uint8_t)((mValue >> 16) & 0xFF);
    }
    std::uint8_t g() const
    {
        return (std::uint8_t)((mValue >> 8) & 0xFF);
    }
    std::uint8_t b() const
    {
        return (std::uint8_t)(mValue & 0xFF);
    }
    std::uint8_t a() const
    {
        return (std::uint8_t)((mValue >> 24) & 0xFF);
    }

    float red() const
    {
        return r() / 255.0f;
    }
    float green() const
    {
        return g() / 255.0f;
    }
    float blue() const
    {
        return b() / 255.0f;
    }
    float alpha() const
    {
        return a() / 255.0f;
    }

    void setRGB(std::uint8_t R, std::uint8_t G, std::uint8_t B);
    void setRGBA(std::uint8_t R, std::uint8_t G, std::uint8_t B, std::uint8_t A);
    Color withAlpha(float a) const;

    float getHue() const;
    float getSaturation() const;
    float getValue() const;
    float getLuminance() const;

    Color lerp(const Color& to, float t) const;
    Color multiply(const Color& other) const;

    std::uint32_t toARGB(float alpha) const;

    static Color fromRGB(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 255);
    static Color fromRGBFloat(float r, float g, float b, float a = 1.0f);
    static Color fromHSV(float h, float s, float v, float a = 1.0f);
    static Color lerp(Color from, Color to, float t);

    bool operator==(const Color& o) const
    {
        return mValue == o.mValue;
    }
    bool operator!=(const Color& o) const
    {
        return mValue != o.mValue;
    }
    operator std::uint32_t() const
    {
        return mValue;
    }

private:
    std::uint32_t mValue;
};

} // namespace prisma3d
