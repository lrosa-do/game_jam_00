#pragma once

#include "mathc.h"

#include <cmath>

namespace prisma3d
{

namespace modifiermath
{

constexpr float kEpsilon = 0.00001f;
constexpr double kTau = 6.283185307179586476925286766559;
constexpr double kPi = 3.1415926535897932384626433832795;

inline bool isZeroApprox(float value)
{
    return std::fabs(value) < kEpsilon;
}

inline bool isEqualApprox(float a, float b)
{
    if (a == b) return true;
    float tolerance = kEpsilon * std::fabs(a);
    if (tolerance < kEpsilon) tolerance = kEpsilon;
    return std::fabs(a - b) < tolerance;
}

inline bool isZeroApprox(const Math::Vec3& v)
{
    return isZeroApprox(v.x) && isZeroApprox(v.y) && isZeroApprox(v.z);
}

inline bool isEqualApprox(const Math::Vec3& a, const Math::Vec3& b)
{
    return isEqualApprox(a.x, b.x) && isEqualApprox(a.y, b.y) && isEqualApprox(a.z, b.z);
}

inline Math::Vec3 normalized(const Math::Vec3& v)
{
    const float lengthSquared = v.LengthSquared();
    if (lengthSquared == 0.0f) return Math::Vec3(0.0f, 0.0f, 0.0f);
    const float length = std::sqrt(lengthSquared);
    return Math::Vec3(v.x / length, v.y / length, v.z / length);
}

inline bool isNormalized(const Math::Quaternion& q)
{
    return std::fabs(q.LengthSquared() - 1.0f) < kEpsilon;
}

inline float angleTo(const Math::Vec3& from, const Math::Vec3& to)
{
    return std::atan2(from.Cross(to).Length(), from.Dot(to));
}

inline Math::Vec3 slide(const Math::Vec3& v, const Math::Vec3& normal)
{
    return v - normal * v.Dot(normal);
}

inline double fposmod(double value, double divisor)
{
    double result = std::fmod(value, divisor);
    if ((result < 0.0 && divisor > 0.0) || (result > 0.0 && divisor < 0.0)) result += divisor;
    return result;
}

inline Math::Quaternion quaternionFromAxisAngle(const Math::Vec3& axis, float angle)
{
    const float length = axis.Length();
    if (length == 0.0f) return Math::Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
    const float sinAngle = std::sin(angle * 0.5f);
    const float cosAngle = std::cos(angle * 0.5f);
    const float scale = sinAngle / length;
    return Math::Quaternion(axis.x * scale, axis.y * scale, axis.z * scale, cosAngle);
}

inline Math::Quaternion rotationQuaternion(const Math::Mat3& basis)
{
    Math::Vec3 x = basis.col0;
    Math::Vec3 y = basis.col1;
    Math::Vec3 z = basis.col2;
    x = normalized(x);
    y = y - x * x.Dot(y);
    y = normalized(y);
    z = z - x * x.Dot(z) - y * y.Dot(z);
    z = normalized(z);
    Math::Mat3 m(x, y, z);
    if (m.Determinant() < 0.0f) m = Math::Mat3(-m.col0, -m.col1, -m.col2);
    return Math::Quaternion::FromMat3(m);
}

inline Math::Quaternion rotationQuaternion(const Math::Mat4& transform)
{
    return rotationQuaternion(transform.UpperLeft3x3());
}

inline Math::Vec3 xformInverse(const Math::Mat3& basis, const Math::Vec3& v)
{
    return Math::Vec3(basis.col0.Dot(v), basis.col1.Dot(v), basis.col2.Dot(v));
}

inline Math::Vec3 xformInverse(const Math::Quaternion& q, const Math::Vec3& v)
{
    return q.Inverse() * v;
}

} // namespace modifiermath

} // namespace prisma3d
