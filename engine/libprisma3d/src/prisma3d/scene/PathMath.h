#pragma once

#include "mathc.h"

#include <math.h>

namespace prisma3d
{

namespace pathmath
{

constexpr float kCmpEpsilon = 0.00001f;
constexpr double kUnitEpsilon = 0.001;
constexpr float kPi = 3.14159265358979323846f;

inline float lerp(float from, float to, float weight)
{
    return from + (to - from) * weight;
}

inline Math::Vec3 lerp(const Math::Vec3& from, const Math::Vec3& to, float weight)
{
    return Math::Vec3(lerp(from.x, to.x, weight), lerp(from.y, to.y, weight),
            lerp(from.z, to.z, weight));
}

inline float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

inline float signOf(float value)
{
    return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f);
}

inline float acosClamped(float value)
{
    return value < -1.0f ? kPi : (value > 1.0f ? 0.0f : acosf(value));
}

inline float fposmod(float x, float y)
{
    float value = fmodf(x, y);
    if ((value < 0.0f && y > 0.0f) || (value > 0.0f && y < 0.0f)) value += y;
    value += 0.0f;
    return value;
}

inline bool isEqualApprox(float left, float right)
{
    if (left == right) return true;
    float tolerance = kCmpEpsilon * fabsf(left);
    if (tolerance < kCmpEpsilon) tolerance = kCmpEpsilon;
    return fabsf(left - right) < tolerance;
}

inline bool isZeroApprox(float value)
{
    return fabsf(value) < kCmpEpsilon;
}

inline bool isEqualApprox(const Math::Vec3& a, const Math::Vec3& b)
{
    return isEqualApprox(a.x, b.x) && isEqualApprox(a.y, b.y) && isEqualApprox(a.z, b.z);
}

inline bool isFinite(const Math::Vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

inline Math::Vec3 normalized(const Math::Vec3& v)
{
    if (!isFinite(v)) return Math::Vec3(0.0f, 0.0f, 0.0f);
    float lengthSquared = v.x * v.x + v.y * v.y + v.z * v.z;
    if (lengthSquared == 0.0f) return Math::Vec3(0.0f, 0.0f, 0.0f);
    float length = sqrtf(lengthSquared);
    return Math::Vec3(v.x / length, v.y / length, v.z / length);
}

inline Math::Vec3 anyPerpendicular(const Math::Vec3& v)
{
    const Math::Vec3 axis = (fabsf(v.x) <= fabsf(v.y) && fabsf(v.x) <= fabsf(v.z))
                                    ? Math::Vec3(1.0f, 0.0f, 0.0f)
                                    : Math::Vec3(0.0f, 1.0f, 0.0f);
    return normalized(v.Cross(axis));
}

inline float bezierInterpolate(float start, float control1, float control2, float end, float t)
{
    float omt = (1.0f - t);
    float omt2 = omt * omt;
    float omt3 = omt2 * omt;
    float t2 = t * t;
    float t3 = t2 * t;
    return start * omt3 + control1 * omt2 * t * 3.0f + control2 * omt * t2 * 3.0f + end * t3;
}

inline float bezierDerivative(float start, float control1, float control2, float end, float t)
{
    float omt = (1.0f - t);
    float omt2 = omt * omt;
    float t2 = t * t;
    return (control1 - start) * 3.0f * omt2 + (control2 - control1) * 6.0f * omt * t +
           (end - control2) * 3.0f * t2;
}

inline float cubicInterpolate(float from, float to, float pre, float post, float weight)
{
    return 0.5f * ((from * 2.0f) + (-pre + to) * weight +
                          (2.0f * pre - 5.0f * from + 4.0f * to - post) * (weight * weight) +
                          (-pre + 3.0f * from - 3.0f * to + post) * (weight * weight * weight));
}

inline Math::Vec3 bezierInterpolate(const Math::Vec3& start, const Math::Vec3& control1,
        const Math::Vec3& control2, const Math::Vec3& end, float t)
{
    return Math::Vec3(bezierInterpolate(start.x, control1.x, control2.x, end.x, t),
            bezierInterpolate(start.y, control1.y, control2.y, end.y, t),
            bezierInterpolate(start.z, control1.z, control2.z, end.z, t));
}

inline Math::Vec3 bezierDerivative(const Math::Vec3& start, const Math::Vec3& control1,
        const Math::Vec3& control2, const Math::Vec3& end, float t)
{
    return Math::Vec3(bezierDerivative(start.x, control1.x, control2.x, end.x, t),
            bezierDerivative(start.y, control1.y, control2.y, end.y, t),
            bezierDerivative(start.z, control1.z, control2.z, end.z, t));
}

inline Math::Vec3 cubicInterpolate(const Math::Vec3& from, const Math::Vec3& to,
        const Math::Vec3& pre, const Math::Vec3& post, float weight)
{
    return Math::Vec3(cubicInterpolate(from.x, to.x, pre.x, post.x, weight),
            cubicInterpolate(from.y, to.y, pre.y, post.y, weight),
            cubicInterpolate(from.z, to.z, pre.z, post.z, weight));
}

inline Math::Mat3 fromRows(const Math::Vec3& row0, const Math::Vec3& row1, const Math::Vec3& row2)
{
    return Math::Mat3(Math::Vec3(row0.x, row1.x, row2.x), Math::Vec3(row0.y, row1.y, row2.y),
            Math::Vec3(row0.z, row1.z, row2.z));
}

inline Math::Vec3 rowOf(const Math::Mat3& m, int row)
{
    return Math::Vec3(m.col0[row], m.col1[row], m.col2[row]);
}

inline float element(const Math::Mat3& m, int row, int column)
{
    return m[column][row];
}

inline Math::Mat3 axisAngle(const Math::Vec3& axis, float angle)
{
    const Math::Vec3 axisSquared(axis.x * axis.x, axis.y * axis.y, axis.z * axis.z);
    const float cosine = cosf(angle);
    const float sine = sinf(angle);
    const float t = 1.0f - cosine;

    float r[3][3];
    r[0][0] = axisSquared.x + cosine * (1.0f - axisSquared.x);
    r[1][1] = axisSquared.y + cosine * (1.0f - axisSquared.y);
    r[2][2] = axisSquared.z + cosine * (1.0f - axisSquared.z);

    float xyzt = axis.x * axis.y * t;
    float zyxs = axis.z * sine;
    r[0][1] = xyzt - zyxs;
    r[1][0] = xyzt + zyxs;

    xyzt = axis.x * axis.z * t;
    zyxs = axis.y * sine;
    r[0][2] = xyzt + zyxs;
    r[2][0] = xyzt - zyxs;

    xyzt = axis.y * axis.z * t;
    zyxs = axis.x * sine;
    r[1][2] = xyzt - zyxs;
    r[2][1] = xyzt + zyxs;

    return fromRows(Math::Vec3(r[0][0], r[0][1], r[0][2]), Math::Vec3(r[1][0], r[1][1], r[1][2]),
            Math::Vec3(r[2][0], r[2][1], r[2][2]));
}

inline Math::Mat3 orthonormalized(const Math::Mat3& m)
{
    Math::Vec3 x = m.col0;
    Math::Vec3 y = m.col1;
    Math::Vec3 z = m.col2;

    x = normalized(x);
    y = (y - x * (x.Dot(y)));
    y = normalized(y);
    z = (z - x * (x.Dot(z)) - y * (y.Dot(z)));
    z = normalized(z);

    return Math::Mat3(x, y, z);
}

inline Math::Mat3 lookingAt(const Math::Vec3& target, const Math::Vec3& up)
{
    Math::Vec3 vz = normalized(target);
    vz = -vz;
    Math::Vec3 vx = up.Cross(vz);
    if (isZeroApprox(vx.x) && isZeroApprox(vx.y) && isZeroApprox(vx.z)) vx = anyPerpendicular(up);
    vx = normalized(vx);
    const Math::Vec3 vy = vz.Cross(vx);
    return Math::Mat3(vx, vy, vz);
}

inline Math::Mat3 lookingAt(const Math::Vec3& target)
{
    return lookingAt(target, Math::Vec3(0.0f, 1.0f, 0.0f));
}

inline void rotateToAlign(Math::Mat3& basis, const Math::Vec3& startDirection,
        const Math::Vec3& endDirection)
{
    const Math::Vec3 axis = normalized(startDirection.Cross(endDirection));
    if (axis.x * axis.x + axis.y * axis.y + axis.z * axis.z != 0.0f)
    {
        float dot = startDirection.Dot(endDirection);
        dot = clamp(dot, -1.0f, 1.0f);
        const float angle = acosClamped(dot);
        basis = axisAngle(axis, angle) * basis;
    }
}

inline Math::Quaternion quaternionOf(const Math::Mat3& m)
{
    const float m00 = element(m, 0, 0), m11 = element(m, 1, 1), m22 = element(m, 2, 2);
    const float trace = m00 + m11 + m22;
    float temp[4];

    if (trace > 0.0f)
    {
        float s = sqrtf(trace + 1.0f);
        temp[3] = (s * 0.5f);
        s = 0.5f / s;

        temp[0] = ((element(m, 2, 1) - element(m, 1, 2)) * s);
        temp[1] = ((element(m, 0, 2) - element(m, 2, 0)) * s);
        temp[2] = ((element(m, 1, 0) - element(m, 0, 1)) * s);
    }
    else
    {
        const int i = m00 < m11 ? (m11 < m22 ? 2 : 1) : (m00 < m22 ? 2 : 0);
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;

        float s = sqrtf(element(m, i, i) - element(m, j, j) - element(m, k, k) + 1.0f);
        temp[i] = s * 0.5f;
        s = 0.5f / s;

        temp[3] = (element(m, k, j) - element(m, j, k)) * s;
        temp[j] = (element(m, j, i) + element(m, i, j)) * s;
        temp[k] = (element(m, k, i) + element(m, i, k)) * s;
    }

    return Math::Quaternion(temp[0], temp[1], temp[2], temp[3]);
}

inline Math::Mat3 basisOf(const Math::Quaternion& q)
{
    const float d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const float s = 2.0f / d;
    const float xs = q.x * s, ys = q.y * s, zs = q.z * s;
    const float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
    const float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
    const float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
    return fromRows(Math::Vec3(1.0f - (yy + zz), xy - wz, xz + wy),
            Math::Vec3(xy + wz, 1.0f - (xx + zz), yz - wx),
            Math::Vec3(xz - wy, yz + wx, 1.0f - (xx + yy)));
}

inline Math::Quaternion slerp(const Math::Quaternion& from, const Math::Quaternion& to,
        float weight)
{
    float cosom = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
    Math::Quaternion to1 = to;

    if (cosom < 0.0f)
    {
        cosom = -cosom;
        to1 = Math::Quaternion(-to.x, -to.y, -to.z, -to.w);
    }

    float scale0, scale1;
    if ((1.0f - cosom) > kCmpEpsilon)
    {
        const float omega = acosClamped(cosom);
        const float sinom = sinf(omega);
        scale0 = sinf((1.0f - weight) * omega) / sinom;
        scale1 = sinf(weight * omega) / sinom;
    }
    else
    {
        scale0 = 1.0f - weight;
        scale1 = weight;
    }

    return Math::Quaternion(scale0 * from.x + scale1 * to1.x, scale0 * from.y + scale1 * to1.y,
            scale0 * from.z + scale1 * to1.z, scale0 * from.w + scale1 * to1.w);
}

inline Math::Mat3 slerp(const Math::Mat3& from, const Math::Mat3& to, float weight)
{
    const Math::Quaternion fromQuaternion = quaternionOf(from);
    const Math::Quaternion toQuaternion = quaternionOf(to);

    Math::Mat3 result = basisOf(slerp(fromQuaternion, toQuaternion, weight));
    const float scale0 = lerp(rowOf(from, 0).Length(), rowOf(to, 0).Length(), weight);
    const float scale1 = lerp(rowOf(from, 1).Length(), rowOf(to, 1).Length(), weight);
    const float scale2 = lerp(rowOf(from, 2).Length(), rowOf(to, 2).Length(), weight);
    return fromRows(rowOf(result, 0) * scale0, rowOf(result, 1) * scale1,
            rowOf(result, 2) * scale2);
}

inline float shortestArcAngle(const Math::Vec3& from, const Math::Vec3& to)
{
    const float almostOne = 0.99999975f;
    const Math::Vec3 n0 = normalized(from);
    const Math::Vec3 n1 = normalized(to);
    const float d = n0.Dot(n1);
    float w;
    if (fabsf(d) > almostOne)
    {
        if (d >= 0.0f) w = 1.0f;
        else
            w = 0.0f;
    }
    else
    {
        const float s = sqrtf((1.0f + d) * 2.0f);
        w = s * 0.5f;
    }
    return 2.0f * acosClamped(w);
}

inline Math::Vec3 scaleOf(const Math::Mat3& m)
{
    const float determinant = m.col0.Dot(m.col1.Cross(m.col2));
    const float sign = signOf(determinant);
    return Math::Vec3(sign * m.col0.Length(), sign * m.col1.Length(), sign * m.col2.Length());
}

inline void scaleLocal(Math::Mat3& m, const Math::Vec3& scale)
{
    m.col0 = m.col0 * scale.x;
    m.col1 = m.col1 * scale.y;
    m.col2 = m.col2 * scale.z;
}

inline Math::Vec3 eulerYXZ(const Math::Mat3& basis)
{
    Math::Mat3 m = orthonormalized(basis);
    if (m.col0.Dot(m.col1.Cross(m.col2)) < 0.0f)
    {
        m.col0 = -m.col0;
        m.col1 = -m.col1;
        m.col2 = -m.col2;
    }

    const float epsilon = 0.00000025f;
    Math::Vec3 euler(0.0f, 0.0f, 0.0f);
    const float m12 = element(m, 1, 2);

    if (m12 < (1.0f - epsilon))
    {
        if (m12 > -(1.0f - epsilon))
        {
            if (element(m, 1, 0) == 0.0f && element(m, 0, 1) == 0.0f && element(m, 0, 2) == 0.0f &&
                    element(m, 2, 0) == 0.0f && element(m, 0, 0) == 1.0f)
            {
                euler.x = atan2f(-m12, element(m, 1, 1));
                euler.y = 0.0f;
                euler.z = 0.0f;
            }
            else
            {
                euler.x = asinf(-m12);
                euler.y = atan2f(element(m, 0, 2), element(m, 2, 2));
                euler.z = atan2f(element(m, 1, 0), element(m, 1, 1));
            }
        }
        else
        {
            euler.x = kPi * 0.5f;
            euler.y = atan2f(element(m, 0, 1), element(m, 0, 0));
            euler.z = 0.0f;
        }
    }
    else
    {
        euler.x = -kPi * 0.5f;
        euler.y = -atan2f(element(m, 0, 1), element(m, 0, 0));
        euler.z = 0.0f;
    }
    return euler;
}

inline Math::Mat3 basisFromEulerYXZ(const Math::Vec3& euler)
{
    float c = cosf(euler.x);
    float s = sinf(euler.x);
    const Math::Mat3 xmat =
            fromRows(Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, c, -s), Math::Vec3(0.0f, s, c));

    c = cosf(euler.y);
    s = sinf(euler.y);
    const Math::Mat3 ymat =
            fromRows(Math::Vec3(c, 0.0f, s), Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec3(-s, 0.0f, c));

    c = cosf(euler.z);
    s = sinf(euler.z);
    const Math::Mat3 zmat =
            fromRows(Math::Vec3(c, -s, 0.0f), Math::Vec3(s, c, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));

    return ymat * xmat * zmat;
}

} // namespace pathmath

} // namespace prisma3d
