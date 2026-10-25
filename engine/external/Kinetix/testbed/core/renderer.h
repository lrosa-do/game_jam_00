#pragma once

// renderer.h - the drawing surface the scenes share (immediate-mode GL).

#include <kinetix/core/Math.h>

namespace testbed
{

// Draws in world units; 2D scenes work in pixels (see kLinearSlop in kx/common.h).
struct Camera2D
{
    float centerX = 0.0f;
    float centerY = 0.0f;
    float zoom = 1.0f;
};

struct Camera3D
{
    kinetix::Math::vec3 target = kinetix::Math::vec3(0.0f, 0.0f, 0.0f);
    float distance = 30.0f;
    float yaw = 0.6f;   // radians
    float pitch = 0.5f;
    // A chase camera: yaw is where the scene wants the eye; the shell eases towards it and adds
    // the user's own orbit.
    bool chase = false;
};

void beginFrame2D(const Camera2D& camera, int windowWidth, int windowHeight);
void beginFrame3D(const Camera3D& camera, int windowWidth, int windowHeight);

void drawLine3D(const kinetix::Math::vec3& from, const kinetix::Math::vec3& to,
                float r, float g, float b);
void drawGrid3D(float y, int slices, float spacing);
void drawText(int x, int y, const char* text);

// Cursor to world; call after beginFrame2D/3D and before anything changes the matrices.
kinetix::Math::vec3 unprojectToPlane2D(int mouseX, int mouseY, int windowWidth, int windowHeight);
void unprojectRay3D(int mouseX, int mouseY, int windowWidth, int windowHeight,
                    kinetix::Math::vec3& outOrigin, kinetix::Math::vec3& outDirection);

} // namespace testbed
