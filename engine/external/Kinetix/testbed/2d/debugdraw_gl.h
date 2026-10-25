#pragma once

// debugdraw_gl.h - kx::DebugDraw on top of GL.

#include <kx/debugdraw.h>

namespace testbed
{

class GLDebugDraw2D final : public kx::DebugDraw
{
public:
    void DrawCircleShape(const kx::Transform& xf, float radius, kx::Color color) override;
    void DrawPolygonShape(const kx::Transform& xf, const kx::vec2* verts, int count,
                          kx::Color color) override;
    void DrawSegment(const kx::vec2& a, const kx::vec2& b, kx::Color color) override;
    void DrawPoint(const kx::vec2& p, float size, kx::Color color) override;
    void DrawAABB(const kx::vec2& lower, const kx::vec2& upper, kx::Color color) override;
};

} // namespace testbed
