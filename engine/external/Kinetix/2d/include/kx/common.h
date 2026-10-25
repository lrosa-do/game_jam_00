#pragma once

#include <kinetix/core/Algorithm.h>

#include "math.h"
#include <math.h>
#include <stdint.h>

namespace kx
{

    struct Matrix2D
    {
        float a, b, c, d, tx, ty;

        Matrix2D() : a(1.0f), b(0.0f), c(0.0f), d(1.0f), tx(0.0f), ty(0.0f) {}

        Matrix2D(float a_, float b_, float c_, float d_, float tx_, float ty_)
            : a(a_), b(b_), c(c_), d(d_), tx(tx_), ty(ty_) {}

        vec2 Transform(float x, float y) const
        {
            return vec2(a * x + c * y + tx, b * x + d * y + ty);
        }

        vec2 Transform(const vec2 &p) const
        {
            return Transform(p.x, p.y);
        }
    };

    using Transform = Matrix2D;

    constexpr float kEpsilon = 1.1920929e-7f;
    constexpr float kPi = 3.14159265359f;
    constexpr float kLinearSlop = 0.5f;
    constexpr float kPolygonRadius = 2.0f * kLinearSlop;
    constexpr float kSpeculativeDistance = 4.0f * kLinearSlop;
    constexpr float kBaumgarte = 0.2f;
    constexpr float kVelocityThreshold = 50.0f;
    constexpr float kMaxLinearCorrection = 10.0f;
    constexpr float kMaxTranslation = 100.0f;
    constexpr float kAabbExtension = 10.0f;
    constexpr float kAabbMultiplier = 4.0f;
    constexpr float kAngularSlop = 2.0f / 180.0f * kPi;
    constexpr float kMaxAngularCorrection = 8.0f / 180.0f * kPi;
    constexpr float kSleepVelocity = 2.5f;
    constexpr float kSleepAngularVelocity = 0.05f;
    constexpr float kTimeToSleep = 0.5f;

    using kinetix::Clamp;
    using kinetix::Math::dot;

    // Cross 2D: escalar para dois vetores, vetor para vetor-escalar. O core so
    // tem o cross 3D, que nao se aplica.
    inline float Cross(const vec2 &a, const vec2 &b) { return a.x * b.y - a.y * b.x; }
    inline vec2 Cross(const vec2 &a, float s) { return vec2(s * a.y, -s * a.x); }
    inline vec2 Cross(float s, const vec2 &a) { return vec2(-s * a.y, s * a.x); }
    inline float Dot(const vec2 &a, const vec2 &b) { return kinetix::Math::dot(a, b); }
    inline float DistanceSquared(const vec2 &a, const vec2 &b)
    {
        vec2 c = a - b;
        return Dot(c, c);
    }
    inline float Distance(const vec2 &a, const vec2 &b) { return kinetix::Math::distance(a, b); }

    // Nao e kinetix::Math::normalize: esse divide pelo comprimento sem guarda, o
    // que da inf/NaN para o vetor zero. A fisica conta com o zero.
    inline vec2 Normalize(const vec2 &v)
    {
        float length = sqrtf(v.x * v.x + v.y * v.y);
        if (length < kEpsilon)
            return vec2(0.0f, 0.0f);
        float invLength = 1.0f / length;
        return vec2(v.x * invLength, v.y * invLength);
    }

    inline Transform MakeTransform(const vec2 &pos, float angleRad)
    {
        float c = cosf(angleRad);
        float s = sinf(angleRad);
        return Transform(c, s, -s, c, pos.x, pos.y);
    }

    inline vec2 Rotate(const Transform &xf, const vec2 &v)
    {
        return vec2(xf.a * v.x + xf.c * v.y, xf.b * v.x + xf.d * v.y);
    }

    inline vec2 InvRotate(const Transform &xf, const vec2 &v)
    {
        return vec2(xf.a * v.x + xf.b * v.y, xf.c * v.x + xf.d * v.y);
    }

    inline vec2 InvTransformPoint(const Transform &xf, const vec2 &p)
    {
        float px = p.x - xf.tx;
        float py = p.y - xf.ty;
        return vec2(xf.a * px + xf.b * py, xf.c * px + xf.d * py);
    }

    inline vec2 Min(const vec2 &a, const vec2 &b) { return kinetix::Math::min(a, b); }
    inline vec2 Max(const vec2 &a, const vec2 &b) { return kinetix::Math::max(a, b); }

}
