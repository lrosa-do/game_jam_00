#pragma once

#include "Color.h"
#include "Math.h"

#include <ct/vector.hpp>

namespace kinetix
{

struct DebugLine3D
{
    Math::vec3 from;
    Math::vec3 to;
    Color color;
    bool depthTest = true;
};

struct DebugTriangle3D
{
    Math::vec3 a;
    Math::vec3 b;
    Math::vec3 c;
    Color color;
};

// Accumulates debug-draw commands as plain data, never touching a GPU. The renderer's owner
// reads lines()/triangles() each frame.
class DebugDraw3D
{
public:
    static DebugDraw3D& getSingleton();

    void clear();
    void line(const Math::vec3& from, const Math::vec3& to, Color color = Color::White,
              bool depthTest = true);
    void box(const AABB& bounds, Color color = Color::Green);
    void axis(const Math::mat4& transform, f32 size = 1.0f);
    void grid(f32 y, u32 slices, f32 spacing, bool axes = true);
    void triangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c, Color color,
                  bool filled = false);
    void pickedTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                        const Math::vec3& hit, f32 normalLength = 0.25f);
    void circle(const Math::vec3& center, const Math::vec3& u, const Math::vec3& v, f32 radius,
                u32 segments, Color color);

    void pointLightGizmo(const Math::vec3& position, f32 range, Color color, u32 segments = 24);
    void spotLightGizmo(const Math::vec3& position, const Math::vec3& direction, f32 range,
                        f32 innerAngleDegrees, f32 outerAngleDegrees, Color color,
                        u32 segments = 24);
    void rectangleLightGizmo(const Math::vec3& position, const Math::vec3& direction, f32 width,
                             f32 height, Color color, f32 normalLength = 0.5f);
    void directionalLightGizmo(const Math::vec3& position, const Math::vec3& direction, Color color,
                               f32 radius = 0.6f, f32 rayLength = 1.5f, u32 rayCount = 6);

    void cross(const Math::vec3& position, f32 size, Color color);
    void arrow(const Math::vec3& from, const Math::vec3& to, f32 headFrom, f32 headTo, Color color);
    void arc(const Math::vec3& from, const Math::vec3& to, f32 height, f32 headFrom, f32 headTo,
             Color color);
    void arrowHead(const Math::vec3& from, const Math::vec3& to, f32 size, Color color);
    void cursor3D(const Math::vec3& position, const Math::vec3& cameraRight,
                  const Math::vec3& cameraUp, f32 radius);

    const ct::Vector<DebugLine3D>& lines() const;
    const ct::Vector<DebugTriangle3D>& triangles() const;
    bool empty() const;

private:
    DebugDraw3D();

    void spotCone(const Math::vec3& apex, const Math::vec3& endCenter, const Math::vec3& right,
                  const Math::vec3& up, f32 range, f32 angleDegrees, u32 segments, u32 ribs,
                  Color color);

    ct::Vector<DebugLine3D> mLines;
    ct::Vector<DebugTriangle3D> mTriangles;
};

inline DebugDraw3D& DebugDraw()
{
    return DebugDraw3D::getSingleton();
}

} // namespace kinetix
