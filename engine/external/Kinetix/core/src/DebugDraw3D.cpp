#include "kinetix/core/DebugDraw3D.h"
#include "kinetix/core/Algorithm.h"

#include <math.h>

namespace kinetix
{

namespace
{

void perpendicularBasis(const Math::vec3& direction, Math::vec3& right, Math::vec3& up)
{
    const Math::vec3 reference =
        fabsf(direction.y) > 0.99f ? Math::vec3(0.0f, 0.0f, 1.0f) : Math::vec3(0.0f, 1.0f, 0.0f);
    right = Math::normalize(Math::cross(direction, reference));
    up = Math::cross(right, direction);
}

Color lighten(Color color, f32 factor)
{
    const auto mix = [factor](u8 channel)
    {
        return static_cast<u8>(Math::mix(static_cast<f32>(channel), 255.0f, factor));
    };
    return Color(mix(color.r()), mix(color.g()), mix(color.b()), color.a());
}

} // namespace

DebugDraw3D::DebugDraw3D()
{
    mLines.reserve(1024);
    mTriangles.reserve(128);
}

DebugDraw3D& DebugDraw3D::getSingleton()
{
    static DebugDraw3D instance;
    return instance;
}

void DebugDraw3D::clear()
{
    mLines.clear();
    mTriangles.clear();
}

void DebugDraw3D::line(const Math::vec3& from, const Math::vec3& to, Color color, bool depthTest)
{
    mLines.push_back({from, to, color, depthTest});
}

void DebugDraw3D::box(const AABB& bounds, Color color)
{
    if (bounds.empty())
        return;

    const Math::vec3& a = bounds.min;
    const Math::vec3& b = bounds.max;
    const Math::vec3 corners[8] = {
        {a.x, a.y, a.z}, {b.x, a.y, a.z}, {b.x, a.y, b.z}, {a.x, a.y, b.z},
        {a.x, b.y, a.z}, {b.x, b.y, a.z}, {b.x, b.y, b.z}, {a.x, b.y, b.z},
    };
    constexpr u8 edges[24] = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6,
                              6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7};
    for (u8 i = 0; i < 24; i += 2)
        line(corners[edges[i]], corners[edges[i + 1]], color);
}

void DebugDraw3D::axis(const Math::mat4& transform, f32 size)
{
    const Math::vec3 origin = Math::vec3(transform[3]);
    line(origin, origin + Math::vec3(transform[0]) * size, Color::Red);
    line(origin, origin + Math::vec3(transform[1]) * size, Color::Green);
    line(origin, origin + Math::vec3(transform[2]) * size, Color::Blue);
}

void DebugDraw3D::grid(f32 y, u32 slices, f32 spacing, bool axes)
{
    if (slices == 0 || !(spacing > 0.0f))
        return;

    const f32 extent = static_cast<f32>(slices) * spacing;
    const Color minor(72, 76, 84);
    for (s32 i = -static_cast<s32>(slices); i <= static_cast<s32>(slices); ++i)
    {
        const f32 offset = static_cast<f32>(i) * spacing;
        const Color xColor = axes && i == 0 ? Color(Color::Red) : minor;
        const Color zColor = axes && i == 0 ? Color(Color::Blue) : minor;
        line(Math::vec3(-extent, y, offset), Math::vec3(extent, y, offset), xColor);
        line(Math::vec3(offset, y, -extent), Math::vec3(offset, y, extent), zColor);
    }
}

void DebugDraw3D::cursor3D(const Math::vec3& position, const Math::vec3& cameraRight,
                           const Math::vec3& cameraUp, f32 radius)
{
    constexpr u32 segments = 24;
    Math::vec3 previous = position + cameraRight * radius;
    for (u32 i = 1; i <= segments; ++i)
    {
        const f32 angle = static_cast<f32>(i) / static_cast<f32>(segments) * Math::two_pi<f32>();
        const Math::vec3 next =
            position + (cameraRight * Math::cos(angle) + cameraUp * Math::sin(angle)) * radius;
        line(previous, next, i % 2 == 0 ? Color(Color::White) : Color(255, 40, 40), false);
        previous = next;
    }

    const f32 crossSize = radius * 0.6f;
    line(position - cameraRight * crossSize, position + cameraRight * crossSize, Color(255, 40, 40),
         false);
    line(position - cameraUp * crossSize, position + cameraUp * crossSize, Color(255, 40, 40),
         false);
}

void DebugDraw3D::triangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c, Color color,
                           bool filled)
{
    if (filled)
        mTriangles.push_back({a, b, c, color});
    else
    {
        line(a, b, color);
        line(b, c, color);
        line(c, a, color);
    }
}

void DebugDraw3D::pickedTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                                 const Math::vec3& hit, f32 normalLength)
{
    triangle(a, b, c, Color(255, 190, 32, 72), true);
    triangle(a, b, c, Color::Yellow, false);

    const Math::vec3 cross = Math::cross(b - a, c - a);
    const f32 lengthSquared = Math::dot(cross, cross);
    if (lengthSquared > 0.0000001f && normalLength > 0.0f)
        line(hit, hit + cross * (normalLength / sqrtf(lengthSquared)), Color::Cyan);
}

void DebugDraw3D::circle(const Math::vec3& center, const Math::vec3& u, const Math::vec3& v,
                         f32 radius, u32 segments, Color color)
{
    segments = Max(segments, 3u);
    Math::vec3 previous = center + u * radius;
    for (u32 i = 1; i <= segments; ++i)
    {
        const f32 t = (static_cast<f32>(i) / static_cast<f32>(segments)) * Math::two_pi<f32>();
        const Math::vec3 point = center + (u * cosf(t) + v * sinf(t)) * radius;
        line(previous, point, color);
        previous = point;
    }
}

void DebugDraw3D::cross(const Math::vec3& position, f32 size, Color color)
{
    line(position - Math::vec3(size, 0.0f, 0.0f), position + Math::vec3(size, 0.0f, 0.0f), color);
    line(position - Math::vec3(0.0f, size, 0.0f), position + Math::vec3(0.0f, size, 0.0f), color);
    line(position - Math::vec3(0.0f, 0.0f, size), position + Math::vec3(0.0f, 0.0f, size), color);
}

void DebugDraw3D::arrowHead(const Math::vec3& from, const Math::vec3& to, f32 size, Color color)
{
    const Math::vec3 delta = to - from;
    if (Math::dot(delta, delta) < 1e-6f)
        return;
    const Math::vec3 forward = Math::normalize(delta);
    Math::vec3 side = Math::cross(Math::vec3(0.0f, 1.0f, 0.0f), forward);
    if (Math::dot(side, side) < 1e-6f)
        side = Math::cross(Math::vec3(1.0f, 0.0f, 0.0f), forward);
    side = Math::normalize(side);

    line(from, from + forward * size + side * (size / 3.0f), color);
    line(from, from + forward * size - side * (size / 3.0f), color);
}

void DebugDraw3D::arrow(const Math::vec3& from, const Math::vec3& to, f32 headFrom, f32 headTo,
                        Color color)
{
    line(from, to, color);
    if (headFrom > 0.001f)
        arrowHead(from, to, headFrom, color);
    if (headTo > 0.001f)
        arrowHead(to, from, headTo, color);
}

void DebugDraw3D::arc(const Math::vec3& from, const Math::vec3& to, f32 height, f32 headFrom,
                      f32 headTo, Color color)
{
    constexpr u32 kArcPoints = 8;
    constexpr f32 kPad = 0.05f;
    constexpr f32 kScale = (1.0f - kPad * 2.0f) / static_cast<f32>(kArcPoints);

    const Math::vec3 delta = to - from;
    const f32 rise = Math::length(delta) * height;

    const auto evaluate = [&](f32 u)
    {
        const f32 bow = 1.0f - (u * 2.0f - 1.0f) * (u * 2.0f - 1.0f);
        return from + delta * u + Math::vec3(0.0f, rise * bow, 0.0f);
    };

    Math::vec3 previous = evaluate(kPad);
    for (u32 i = 1; i <= kArcPoints; ++i)
    {
        const Math::vec3 point = evaluate(kPad + static_cast<f32>(i) * kScale);
        line(previous, point, color);
        previous = point;
    }

    if (headFrom > 0.001f)
        arrowHead(evaluate(kPad), evaluate(kPad + 0.05f), headFrom, color);
    if (headTo > 0.001f)
        arrowHead(evaluate(1.0f - kPad), evaluate(1.0f - kPad - 0.05f), headTo, color);
}

void DebugDraw3D::pointLightGizmo(const Math::vec3& position, f32 range, Color color, u32 segments)
{
    if (!(range > 0.0f))
        return;
    circle(position, Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f), range, segments,
           color);
    circle(position, Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f), range, segments,
           color);
    circle(position, Math::vec3(0.0f, 1.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f), range, segments,
           color);
}

void DebugDraw3D::spotLightGizmo(const Math::vec3& position, const Math::vec3& direction, f32 range,
                                 f32 innerAngleDegrees, f32 outerAngleDegrees, Color color,
                                 u32 segments)
{
    if (!(range > 0.0f) || !(Math::length(direction) > 0.0001f))
        return;
    const Math::vec3 forward = Math::normalize(direction);
    Math::vec3 right;
    Math::vec3 up;
    perpendicularBasis(forward, right, up);
    const Math::vec3 endCenter = position + forward * range;
    const u32 ribs = Max(4u, segments / 3);

    spotCone(position, endCenter, right, up, range, outerAngleDegrees, segments, ribs, color);
    spotCone(position, endCenter, right, up, range, innerAngleDegrees, segments, ribs,
             lighten(color, 0.6f));
}

void DebugDraw3D::spotCone(const Math::vec3& apex, const Math::vec3& endCenter,
                           const Math::vec3& right, const Math::vec3& up, f32 range, f32 angleDegrees,
                           u32 segments, u32 ribs, Color color)
{
    const f32 endRadius = range * tanf(Math::radians(Math::clamp(angleDegrees, 0.0f, 89.0f)));
    circle(endCenter, right, up, endRadius, segments, color);
    for (u32 i = 0; i < ribs; ++i)
    {
        const f32 t = (static_cast<f32>(i) / static_cast<f32>(ribs)) * Math::two_pi<f32>();
        const Math::vec3 point = endCenter + (right * cosf(t) + up * sinf(t)) * endRadius;
        line(apex, point, color);
    }
}

void DebugDraw3D::rectangleLightGizmo(const Math::vec3& position, const Math::vec3& direction,
                                      f32 width, f32 height, Color color, f32 normalLength)
{
    if (!(width > 0.0f) || !(height > 0.0f) || !(Math::length(direction) > 0.0001f))
        return;
    const Math::vec3 forward = Math::normalize(direction);
    Math::vec3 right;
    Math::vec3 up;
    perpendicularBasis(forward, right, up);
    const Math::vec3 halfRight = right * (width * 0.5f);
    const Math::vec3 halfUp = up * (height * 0.5f);
    const Math::vec3 corners[4] = {
        position - halfRight - halfUp,
        position + halfRight - halfUp,
        position + halfRight + halfUp,
        position - halfRight + halfUp,
    };
    for (u32 i = 0; i < 4; ++i)
        line(corners[i], corners[(i + 1) % 4], color);
    if (normalLength > 0.0f)
        line(position, position + forward * normalLength, color);
}

void DebugDraw3D::directionalLightGizmo(const Math::vec3& position, const Math::vec3& direction,
                                        Color color, f32 radius, f32 rayLength, u32 rayCount)
{
    if (!(Math::length(direction) > 0.0001f))
        return;
    const Math::vec3 forward = Math::normalize(direction);
    Math::vec3 right;
    Math::vec3 up;
    perpendicularBasis(forward, right, up);
    circle(position, right, up, radius, 24, color);

    rayCount = Max(rayCount, 1u);
    for (u32 i = 0; i < rayCount; ++i)
    {
        const f32 t = (static_cast<f32>(i) / static_cast<f32>(rayCount)) * Math::two_pi<f32>();
        const Math::vec3 start =
            position + (right * cosf(t) + up * sinf(t)) * (radius * 0.6f);
        line(start, start + forward * rayLength, color);
    }
    line(position, position + forward * rayLength, color);
}

const ct::Vector<DebugLine3D>& DebugDraw3D::lines() const
{
    return mLines;
}

const ct::Vector<DebugTriangle3D>& DebugDraw3D::triangles() const
{
    return mTriangles;
}

bool DebugDraw3D::empty() const
{
    return mLines.empty() && mTriangles.empty();
}

} // namespace kinetix
