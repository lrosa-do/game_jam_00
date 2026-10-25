#include "debugdraw_gl.h"

#include <GL/gl.h>

#include <math.h>

namespace testbed
{

namespace
{
constexpr int kCircleSegments = 24;

void applyColor(kx::Color c)
{
    glColor4ub(c.r, c.g, c.b, c.a);
}
} // namespace

void GLDebugDraw2D::DrawCircleShape(const kx::Transform& xf, float radius, kx::Color color)
{
    const kx::vec2 center = xf.Transform(kx::vec2(0.0f, 0.0f));
    applyColor(color);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < kCircleSegments; ++i)
    {
        const float angle = 2.0f * 3.14159265359f * float(i) / float(kCircleSegments);
        glVertex2f(center.x + radius * cosf(angle), center.y + radius * sinf(angle));
    }
    glEnd();

    const kx::vec2 rim = xf.Transform(kx::vec2(radius, 0.0f));
    glBegin(GL_LINES);
    glVertex2f(center.x, center.y);
    glVertex2f(rim.x, rim.y);
    glEnd();
}

void GLDebugDraw2D::DrawPolygonShape(const kx::Transform& xf, const kx::vec2* verts, int count,
                                     kx::Color color)
{
    applyColor(color);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < count; ++i)
    {
        const kx::vec2 p = xf.Transform(verts[i]);
        glVertex2f(p.x, p.y);
    }
    glEnd();
}

void GLDebugDraw2D::DrawSegment(const kx::vec2& a, const kx::vec2& b, kx::Color color)
{
    applyColor(color);
    glBegin(GL_LINES);
    glVertex2f(a.x, a.y);
    glVertex2f(b.x, b.y);
    glEnd();
}

void GLDebugDraw2D::DrawPoint(const kx::vec2& p, float size, kx::Color color)
{
    applyColor(color);
    glPointSize(size);
    glBegin(GL_POINTS);
    glVertex2f(p.x, p.y);
    glEnd();
    glPointSize(1.0f);
}

void GLDebugDraw2D::DrawAABB(const kx::vec2& lower, const kx::vec2& upper, kx::Color color)
{
    applyColor(color);
    glBegin(GL_LINE_LOOP);
    glVertex2f(lower.x, lower.y);
    glVertex2f(upper.x, lower.y);
    glVertex2f(upper.x, upper.y);
    glVertex2f(lower.x, upper.y);
    glEnd();
}

} // namespace testbed
