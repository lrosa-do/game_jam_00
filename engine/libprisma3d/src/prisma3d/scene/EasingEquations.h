#pragma once

/*
 * Derived from Robert Penner's easing equations: http://robertpenner.com/easing/
 *
 * Copyright (c) 2001 Robert Penner
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <math.h>

namespace prisma3d
{

namespace easing
{

namespace math
{

const double kPi = 3.1415926535897932384626433833;

inline double sin(double x)
{
    return ::sin(x);
}

inline float sin(float x)
{
    return ::sinf(x);
}

inline double cos(double x)
{
    return ::cos(x);
}

inline float cos(float x)
{
    return ::cosf(x);
}

inline double sqrt(double x)
{
    return ::sqrt(x);
}

inline float sqrt(float x)
{
    return ::sqrtf(x);
}

inline double pow(double x, double y)
{
    return ::pow(x, y);
}

inline float pow(float x, float y)
{
    return ::powf(x, y);
}

} // namespace math

namespace Linear
{
inline float in(float t, float b, float c, float d)
{
    return c * t / d + b;
}
} // namespace Linear

namespace Sine
{
inline float in(float t, float b, float c, float d)
{
    return -c * math::cos(t / d * (math::kPi / 2)) + c + b;
}

inline float out(float t, float b, float c, float d)
{
    return c * math::sin(t / d * (math::kPi / 2)) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    return -c / 2 * (math::cos(math::kPi * t / d) - 1) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Sine

namespace Quint
{
inline float in(float t, float b, float c, float d)
{
    return c * math::pow(t / d, 5) + b;
}

inline float out(float t, float b, float c, float d)
{
    return c * (math::pow(t / d - 1, 5) + 1) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    t = t / d * 2;

    if (t < 1)
    {
        return c / 2 * math::pow(t, 5) + b;
    }
    return c / 2 * (math::pow(t - 2, 5) + 2) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Quint

namespace Quart
{
inline float in(float t, float b, float c, float d)
{
    return c * math::pow(t / d, 4) + b;
}

inline float out(float t, float b, float c, float d)
{
    return -c * (math::pow(t / d - 1, 4) - 1) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    t = t / d * 2;

    if (t < 1)
    {
        return c / 2 * math::pow(t, 4) + b;
    }
    return -c / 2 * (math::pow(t - 2, 4) - 2) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Quart

namespace Quad
{
inline float in(float t, float b, float c, float d)
{
    return c * math::pow(t / d, 2) + b;
}

inline float out(float t, float b, float c, float d)
{
    t /= d;
    return -c * t * (t - 2) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    t = t / d * 2;

    if (t < 1)
    {
        return c / 2 * math::pow(t, 2) + b;
    }
    return -c / 2 * ((t - 1) * (t - 3) - 1) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Quad

namespace Expo
{
inline float in(float t, float b, float c, float d)
{
    if (t == 0)
    {
        return b;
    }
    return c * math::pow(2, 10 * (t / d - 1)) + b - c * 0.001;
}

inline float out(float t, float b, float c, float d)
{
    if (t == d)
    {
        return b + c;
    }
    return c * 1.001 * (-math::pow(2, -10 * t / d) + 1) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    if (t == 0)
    {
        return b;
    }

    if (t == d)
    {
        return b + c;
    }

    t = t / d * 2;

    if (t < 1)
    {
        return c / 2 * math::pow(2, 10 * (t - 1)) + b - c * 0.0005;
    }
    return c / 2 * 1.0005 * (-math::pow(2, -10 * (t - 1)) + 2) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Expo

namespace Elastic
{
inline float in(float t, float b, float c, float d)
{
    if (t == 0)
    {
        return b;
    }

    t /= d;
    if (t == 1)
    {
        return b + c;
    }

    t -= 1;
    float p = d * 0.3f;
    float a = c * math::pow(2, 10 * t);
    float s = p / 4;

    return -(a * math::sin((t * d - s) * (2 * math::kPi) / p)) + b;
}

inline float out(float t, float b, float c, float d)
{
    if (t == 0)
    {
        return b;
    }

    t /= d;
    if (t == 1)
    {
        return b + c;
    }

    float p = d * 0.3f;
    float s = p / 4;

    return (c * math::pow(2, -10 * t) * math::sin((t * d - s) * (2 * math::kPi) / p) + c + b);
}

inline float inOut(float t, float b, float c, float d)
{
    if (t == 0)
    {
        return b;
    }

    if ((t /= d / 2) == 2)
    {
        return b + c;
    }

    float p = d * (0.3f * 1.5f);
    float a = c;
    float s = p / 4;

    if (t < 1)
    {
        t -= 1;
        a *= math::pow(2, 10 * t);
        return -0.5f * (a * math::sin((t * d - s) * (2 * math::kPi) / p)) + b;
    }

    t -= 1;
    a *= math::pow(2, -10 * t);
    return a * math::sin((t * d - s) * (2 * math::kPi) / p) * 0.5f + c + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Elastic

namespace Cubic
{
inline float in(float t, float b, float c, float d)
{
    t /= d;
    return c * t * t * t + b;
}

inline float out(float t, float b, float c, float d)
{
    t = t / d - 1;
    return c * (t * t * t + 1) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    t /= d / 2;
    if (t < 1)
    {
        return c / 2 * t * t * t + b;
    }

    t -= 2;
    return c / 2 * (t * t * t + 2) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Cubic

namespace Circ
{
inline float in(float t, float b, float c, float d)
{
    t /= d;
    return -c * (math::sqrt(1 - t * t) - 1) + b;
}

inline float out(float t, float b, float c, float d)
{
    t = t / d - 1;
    return c * math::sqrt(1 - t * t) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    t /= d / 2;
    if (t < 1)
    {
        return -c / 2 * (math::sqrt(1 - t * t) - 1) + b;
    }

    t -= 2;
    return c / 2 * (math::sqrt(1 - t * t) + 1) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Circ

namespace Bounce
{
inline float out(float t, float b, float c, float d)
{
    t /= d;

    if (t < (1 / 2.75f))
    {
        return c * (7.5625f * t * t) + b;
    }

    if (t < (2 / 2.75f))
    {
        t -= 1.5f / 2.75f;
        return c * (7.5625f * t * t + 0.75f) + b;
    }

    if (t < (2.5 / 2.75))
    {
        t -= 2.25f / 2.75f;
        return c * (7.5625f * t * t + 0.9375f) + b;
    }

    t -= 2.625f / 2.75f;
    return c * (7.5625f * t * t + 0.984375f) + b;
}

inline float in(float t, float b, float c, float d)
{
    return c - out(d - t, 0, c, d) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return in(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return out(t * 2 - d, b + h, h, d);
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Bounce

namespace Back
{
inline float in(float t, float b, float c, float d)
{
    float s = 1.70158f;
    t /= d;

    return c * t * t * ((s + 1) * t - s) + b;
}

inline float out(float t, float b, float c, float d)
{
    float s = 1.70158f;
    t = t / d - 1;

    return c * (t * t * ((s + 1) * t + s) + 1) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    float s = 1.70158f * 1.525f;
    t /= d / 2;

    if (t < 1)
    {
        return c / 2 * (t * t * ((s + 1) * t - s)) + b;
    }

    t -= 2;
    return c / 2 * (t * t * ((s + 1) * t + s) + 2) + b;
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Back

namespace Spring
{
inline float out(float t, float b, float c, float d)
{
    t /= d;
    float s = 1.0 - t;
    t = (math::sin(t * math::kPi * (0.2 + 2.5 * t * t * t)) * math::pow(s, (float) 2.2) + t) *
        (1.0 + (1.2 * s));
    return c * t + b;
}

inline float in(float t, float b, float c, float d)
{
    return c - out(d - t, 0, c, d) + b;
}

inline float inOut(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return in(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return out(t * 2 - d, b + h, h, d);
}

inline float outIn(float t, float b, float c, float d)
{
    if (t < d / 2)
    {
        return out(t * 2, b, c / 2, d);
    }
    float h = c / 2;
    return in(t * 2 - d, b + h, h, d);
}
} // namespace Spring

} // namespace easing

} // namespace prisma3d
