#include "prisma3d/core/Color.h"

#include <math.h>

namespace prisma3d
{

const std::uint32_t Color::White = 0xFFFFFFFF;
const std::uint32_t Color::Black = 0xFF000000;
const std::uint32_t Color::Red = 0xFFFF0000;
const std::uint32_t Color::Green = 0xFF00FF00;
const std::uint32_t Color::Blue = 0xFF0000FF;
const std::uint32_t Color::Yellow = 0xFFFFFF00;
const std::uint32_t Color::Cyan = 0xFF00FFFF;
const std::uint32_t Color::Magenta = 0xFFFF00FF;
const std::uint32_t Color::Orange = 0xFFFFA500;
const std::uint32_t Color::Gray = 0xFF808080;
const std::uint32_t Color::Transparent = 0x00000000;

namespace
{
inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
inline float fclamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
inline int imax(int a, int b)
{
    return a > b ? a : b;
}
inline int imin(int a, int b)
{
    return a < b ? a : b;
}
} // namespace

Color::Color(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a)
{
    mValue = (std::uint32_t(a) << 24) | (std::uint32_t(r) << 16) | (std::uint32_t(g) << 8) |
             std::uint32_t(b);
}

void Color::setRGB(std::uint8_t R, std::uint8_t G, std::uint8_t B)
{
    mValue = (mValue & 0xFF000000) | (std::uint32_t(R) << 16) | (std::uint32_t(G) << 8) |
             std::uint32_t(B);
}

void Color::setRGBA(std::uint8_t R, std::uint8_t G, std::uint8_t B, std::uint8_t A)
{
    mValue = (std::uint32_t(A) << 24) | (std::uint32_t(R) << 16) | (std::uint32_t(G) << 8) |
             std::uint32_t(B);
}

Color Color::withAlpha(float a) const
{
    std::uint32_t alpha;
    if (a <= 0.0f) alpha = 0;
    else if (a >= 1.0f)
        alpha = 0xFF;
    else
        alpha = std::uint32_t(0xFF * a);

    return Color((mValue & 0x00FFFFFF) | (alpha << 24));
}

float Color::getHue() const
{
    int h = r();
    int s = g();
    int v = b();

    int maxV = imax(h, imax(s, v));
    int minV = imin(h, imin(s, v));

    float hue = 0.0f;

    if (maxV == minV) hue = 0.0f;
    else if (maxV == h)
        hue = fmodf(60.0f * (s - v) / (maxV - minV) + 360.0f, 360.0f);
    else if (maxV == s)
        hue = 60.0f * (v - h) / (maxV - minV) + 120.0f;
    else if (maxV == v)
        hue = 60.0f * (h - s) / (maxV - minV) + 240.0f;

    return hue / 360.0f;
}

float Color::getSaturation() const
{
    int h = r();
    int s = g();
    int v = b();

    int maxV = imax(h, imax(s, v));
    if (maxV == 0) return 0.0f;

    int minV = imin(h, imin(s, v));
    return float(maxV - minV) / float(maxV);
}

float Color::getValue() const
{
    int h = r();
    int s = g();
    int v = b();
    return imax(h, imax(s, v)) / 255.0f;
}

float Color::getLuminance() const
{
    return 0.2126f * red() + 0.7152f * green() + 0.0722f * blue();
}

Color Color::lerp(const Color& to, float t) const
{
    return Color::lerp(*this, to, t);
}

Color Color::multiply(const Color& other) const
{
    return Color::fromRGBFloat(red() * other.red(), green() * other.green(), blue() * other.blue(),
            alpha());
}

std::uint32_t Color::toARGB(float alpha) const
{
    alpha = fclamp(alpha, 0.0f, 1.0f);
    return (std::uint32_t(0xFF * alpha) << 24) | (mValue & 0x00FFFFFF);
}

Color Color::fromRGB(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a)
{
    return Color(r, g, b, a);
}

Color Color::fromRGBFloat(float r, float g, float b, float a)
{
    auto intColor = [](float v) -> std::uint8_t
    { return (std::uint8_t) iclamp((int) (v * 256.0f), 0, 255); };
    return Color(intColor(r), intColor(g), intColor(b), intColor(a));
}

Color Color::fromHSV(float h, float s, float v, float a)
{
    h = fmodf(h * 360.0f, 360.0f);
    if (h < 0.0f) h += 360.0f;

    int hi = (int) (h / 60.0f) % 6;
    float f = h / 60.0f - floorf(h / 60.0f);
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);

    float rf = 0, gf = 0, bf = 0;
    switch (hi)
    {
        case 0:
            rf = v;
            gf = t;
            bf = p;
            break;
        case 1:
            rf = q;
            gf = v;
            bf = p;
            break;
        case 2:
            rf = p;
            gf = v;
            bf = t;
            break;
        case 3:
            rf = p;
            gf = q;
            bf = v;
            break;
        case 4:
            rf = t;
            gf = p;
            bf = v;
            break;
        case 5:
            rf = v;
            gf = p;
            bf = q;
            break;
    }

    std::uint8_t R = (std::uint8_t)(rf * 255.0f);
    std::uint8_t G = (std::uint8_t)(gf * 255.0f);
    std::uint8_t B = (std::uint8_t)(bf * 255.0f);
    std::uint8_t A = (std::uint8_t)(fclamp(a, 0.0f, 1.0f) * 255.0f);
    return Color(R, G, B, A);
}

Color Color::lerp(Color from, Color to, float t)
{
    if (t <= 0.0f) return from;
    if (t >= 1.0f) return to;

    int a = from.a();
    int r = from.r();
    int g = from.g();
    int b = from.b();

    int dA = (int) to.a() - a;
    int dR = (int) to.r() - r;
    int dG = (int) to.g() - g;
    int dB = (int) to.b() - b;

    a += (int) (dA * t);
    r += (int) (dR * t);
    g += (int) (dG * t);
    b += (int) (dB * t);

    return Color((std::uint8_t) r, (std::uint8_t) g, (std::uint8_t) b, (std::uint8_t) a);
}

} // namespace prisma3d
