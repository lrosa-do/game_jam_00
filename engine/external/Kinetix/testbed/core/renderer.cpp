#include "renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>

#include <math.h>

namespace testbed
{

void beginFrame2D(const Camera2D& camera, int windowWidth, int windowHeight)
{
    glViewport(0, 0, windowWidth, windowHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    const float halfW = 0.5f * static_cast<float>(windowWidth) / camera.zoom;
    const float halfH = 0.5f * static_cast<float>(windowHeight) / camera.zoom;
    glOrtho(camera.centerX - halfW, camera.centerX + halfW, camera.centerY - halfH,
            camera.centerY + halfH, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
}

void beginFrame3D(const Camera3D& camera, int windowWidth, int windowHeight)
{
    glViewport(0, 0, windowWidth, windowHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const double aspect = windowHeight > 0 ? double(windowWidth) / double(windowHeight) : 1.0;
    gluPerspective(60.0, aspect, 0.1, 1000.0);

    const float cosPitch = cosf(camera.pitch);
    const kinetix::Math::vec3 eye(camera.target.x + camera.distance * cosPitch * sinf(camera.yaw),
                                  camera.target.y + camera.distance * sinf(camera.pitch),
                                  camera.target.z + camera.distance * cosPitch * cosf(camera.yaw));

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye.x, eye.y, eye.z, camera.target.x, camera.target.y, camera.target.z, 0.0, 1.0, 0.0);
    glEnable(GL_DEPTH_TEST);
}

void drawLine3D(const kinetix::Math::vec3& from, const kinetix::Math::vec3& to, float r, float g,
                float b)
{
    glColor3f(r, g, b);
    glBegin(GL_LINES);
    glVertex3f(from.x, from.y, from.z);
    glVertex3f(to.x, to.y, to.z);
    glEnd();
}

void drawGrid3D(float y, int slices, float spacing)
{
    const float extent = float(slices) * spacing * 0.5f;
    glColor3f(0.25f, 0.25f, 0.30f);
    glBegin(GL_LINES);
    for (int i = 0; i <= slices; ++i)
    {
        const float t = -extent + float(i) * spacing;
        glVertex3f(t, y, -extent);
        glVertex3f(t, y, extent);
        glVertex3f(-extent, y, t);
        glVertex3f(extent, y, t);
    }
    glEnd();
}

namespace
{

void readMatrices(double* modelview, double* projection, int* viewport)
{
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);
}

} // namespace

kinetix::Math::vec3 unprojectToPlane2D(int mouseX, int mouseY, int windowWidth, int windowHeight)
{
    (void)windowWidth;
    double modelview[16], projection[16];
    int viewport[4];
    readMatrices(modelview, projection, viewport);

    // Orthographic in the z=0 plane, so any depth unprojects to the same x/y.
    double x = 0.0, y = 0.0, z = 0.0;
    gluUnProject(double(mouseX), double(windowHeight - mouseY), 0.0, modelview, projection,
                 viewport, &x, &y, &z);
    return kinetix::Math::vec3(float(x), float(y), 0.0f);
}

void unprojectRay3D(int mouseX, int mouseY, int windowWidth, int windowHeight,
                    kinetix::Math::vec3& outOrigin, kinetix::Math::vec3& outDirection)
{
    (void)windowWidth;
    double modelview[16], projection[16];
    int viewport[4];
    readMatrices(modelview, projection, viewport);

    const double flippedY = double(windowHeight - mouseY);
    double nx = 0.0, ny = 0.0, nz = 0.0;
    double fx = 0.0, fy = 0.0, fz = 0.0;
    gluUnProject(double(mouseX), flippedY, 0.0, modelview, projection, viewport, &nx, &ny, &nz);
    gluUnProject(double(mouseX), flippedY, 1.0, modelview, projection, viewport, &fx, &fy, &fz);

    outOrigin = kinetix::Math::vec3(float(nx), float(ny), float(nz));
    kinetix::Math::vec3 direction(float(fx - nx), float(fy - ny), float(fz - nz));
    const float length = kinetix::Math::length(direction);
    outDirection = length > 1e-6f ? direction / length : kinetix::Math::vec3(0.0f, 0.0f, -1.0f);
}

} // namespace testbed
