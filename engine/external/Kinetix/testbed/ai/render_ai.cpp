#include "render_ai.h"

#include <GL/gl.h>

#include <math.h>

namespace testbed
{

namespace M = kinetix::Math;
using kinetix::usize;

void setupLightingAI()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);

    const GLfloat direction[4] = {0.3f, 1.0f, 0.5f, 0.0f};
    const GLfloat diffuse[4] = {0.8f, 0.8f, 0.78f, 1.0f};
    const GLfloat ambient[4] = {0.4f, 0.42f, 0.46f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, direction);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
}

void drawAgent(const M::vec3& position, const M::vec3& forward, const AgentStyle& style)
{
    M::vec3 heading = forward;
    heading.y = 0.0f;
    const float length = M::length(heading);
    heading = length > 1e-4f ? heading / length : M::vec3(0.0f, 0.0f, 1.0f);
    const M::vec3 side(-heading.z, 0.0f, heading.x);

    const float r = style.radius;
    const M::vec3 up(0.0f, 0.06f, 0.0f);
    const M::vec3 tip = position + heading * r + up;
    const M::vec3 backLeft = position - heading * (r * 0.75f) - side * (r * 0.6f) + up;
    const M::vec3 backRight = position - heading * (r * 0.75f) + side * (r * 0.6f) + up;

    glDisable(GL_LIGHTING);
    glColor3f(style.r, style.g, style.b);
    glBegin(GL_TRIANGLES);
    glVertex3f(tip.x, tip.y, tip.z);
    glVertex3f(backLeft.x, backLeft.y, backLeft.z);
    glVertex3f(backRight.x, backRight.y, backRight.z);
    glEnd();

    glColor3f(style.r * 0.6f, style.g * 0.6f, style.b * 0.6f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 16; ++i)
    {
        const float a = 2.0f * 3.14159265f * float(i) / 16.0f;
        glVertex3f(position.x + r * cosf(a), position.y + 0.05f, position.z + r * sinf(a));
    }
    glEnd();
}

void drawVelocity(const M::vec3& position, const M::vec3& velocity, float scale)
{
    M::vec3 flat = velocity;
    flat.y = 0.0f;
    const float speed = M::length(flat);
    if (speed < 0.05f)
        return;

    const M::vec3 tip = position + flat * scale;
    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 0.85f, 0.3f);
    glBegin(GL_LINES);
    glVertex3f(position.x, position.y + 0.12f, position.z);
    glVertex3f(tip.x, tip.y + 0.12f, tip.z);
    glEnd();
}

void drawPath(const ct::Vector<M::vec3>& corners, float r, float g, float b)
{
    if (corners.size() < 2)
        return;

    glDisable(GL_LIGHTING);
    glColor3f(r, g, b);
    glBegin(GL_LINE_STRIP);
    for (usize i = 0; i < corners.size(); ++i)
        glVertex3f(corners[i].x, corners[i].y + 0.15f, corners[i].z);
    glEnd();

    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for (usize i = 0; i < corners.size(); ++i)
        glVertex3f(corners[i].x, corners[i].y + 0.15f, corners[i].z);
    glEnd();
    glPointSize(1.0f);
}

void drawNavMeshSolid(const ct::Vector<M::vec3>& triangles)
{
    glEnable(GL_LIGHTING);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    glColor3f(0.26f, 0.34f, 0.30f);
    glBegin(GL_TRIANGLES);
    for (usize i = 0; i + 2 < triangles.size(); i += 3)
    {
        const M::vec3& a = triangles[i];
        const M::vec3& b = triangles[i + 1];
        const M::vec3& c = triangles[i + 2];
        M::vec3 normal = M::cross(b - a, c - a);
        const float length = M::length(normal);
        if (length > 1e-6f)
            normal = normal / length;
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
        glVertex3f(c.x, c.y, c.z);
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);

    glDisable(GL_LIGHTING);
    glColor3f(0.20f, 0.28f, 0.24f);
    glBegin(GL_LINES);
    for (usize i = 0; i + 2 < triangles.size(); i += 3)
        for (int e = 0; e < 3; ++e)
        {
            const M::vec3& a = triangles[i + usize(e)];
            const M::vec3& b = triangles[i + usize((e + 1) % 3)];
            glVertex3f(a.x, a.y, a.z);
            glVertex3f(b.x, b.y, b.z);
        }
    glEnd();
}

void drawGoalMarker(const M::vec3& position, float time)
{
    const float pulse = 0.7f + 0.3f * sinf(time * 3.0f);
    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 0.45f, 0.3f);
    for (int ring = 0; ring < 2; ++ring)
    {
        const float radius = (0.8f + float(ring) * 0.6f) * pulse;
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 24; ++i)
        {
            const float a = 2.0f * 3.14159265f * float(i) / 24.0f;
            glVertex3f(position.x + radius * cosf(a), position.y + 0.1f,
                       position.z + radius * sinf(a));
        }
        glEnd();
    }
    glBegin(GL_LINES);
    glVertex3f(position.x, position.y, position.z);
    glVertex3f(position.x, position.y + 2.0f, position.z);
    glEnd();
}

} // namespace testbed
