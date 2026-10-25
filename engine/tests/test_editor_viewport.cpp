#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/editor/EditorCamera.h"
#include "prisma3d/editor/EditorShell.h"
#include "prisma3d/editor/EditorViewportPanel.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <igui/widgets/DockPanel.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;
namespace rt = ig::retained;

namespace
{

const int kWidth = 800;
const int kHeight = 600;
const float kPi = 3.14159265358979323846f;
const float kEps = 2e-5f;

typedef EditorCamera::Modifier Mod;

bool near(double a, double b, double eps = kEps)
{
    return fabs(a - b) <= eps;
}

bool nearVec(const Math::Vec3& a, const Math::Vec3& b, float eps = kEps)
{
    return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps);
}

struct Basis
{
    Math::Vec3 x;
    Math::Vec3 y;
    Math::Vec3 z;
};

Basis expectedBasis(float xRot, float yRot)
{
    const float a = -yRot;
    const float b = -xRot;
    const float sa = sinf(a);
    const float ca = cosf(a);
    const float sb = sinf(b);
    const float cb = cosf(b);
    Basis basis;
    basis.x = Math::Vec3(ca, 0.0f, -sa);
    basis.y = Math::Vec3(sa * sb, cb, ca * sb);
    basis.z = Math::Vec3(sa * cb, -sb, ca * cb);
    return basis;
}

Math::Vec3 column(const Math::Mat4& matrix, int index)
{
    return Math::Vec3(matrix[index]);
}

Math::Vec3 origin(const EditorCamera& camera)
{
    return column(camera.toCameraTransform(), 3);
}

Math::Vec3 cursorPosition(const EditorCamera::Cursor& cursor)
{
    return Math::Vec3(static_cast<float>(cursor.posX), static_cast<float>(cursor.posY),
            static_cast<float>(cursor.posZ));
}

Math::Vec3 eyePosition(const EditorCamera::Cursor& cursor)
{
    return Math::Vec3(static_cast<float>(cursor.eyePosX), static_cast<float>(cursor.eyePosY),
            static_cast<float>(cursor.eyePosZ));
}

InputEvent motion(int x, int y, int relativeX, int relativeY, unsigned mask)
{
    InputEvent event;
    event.type = InputEventType::MouseMotion;
    event.mouseMotion.x = x;
    event.mouseMotion.y = y;
    event.mouseMotion.relativeX = relativeX;
    event.mouseMotion.relativeY = relativeY;
    event.mouseMotion.buttonMask = mask;
    return event;
}

InputEvent button(int x, int y, MouseButton which, bool pressed, unsigned modifiers = 0)
{
    InputEvent event;
    event.type = InputEventType::MouseButton;
    event.mouseButton.x = x;
    event.mouseButton.y = y;
    event.mouseButton.button = which;
    event.mouseButton.pressed = pressed;
    event.mouseButton.modifiers = modifiers;
    return event;
}

InputEvent wheel(int x, int y, float deltaY)
{
    InputEvent event;
    event.type = InputEventType::MouseWheel;
    event.mouseWheel.x = x;
    event.mouseWheel.y = y;
    event.mouseWheel.deltaX = 0.0f;
    event.mouseWheel.deltaY = deltaY;
    return event;
}

InputEvent key(Key which, bool pressed, unsigned modifiers = 0, bool echo = false)
{
    InputEvent event;
    event.type = InputEventType::Key;
    event.key.key = which;
    event.key.scancode = 0;
    event.key.pressed = pressed;
    event.key.echo = echo;
    event.key.modifiers = modifiers;
    return event;
}

const unsigned kMiddle = 1u << static_cast<int>(MouseButton::Middle);
const unsigned kLeft = 1u << static_cast<int>(MouseButton::Left);
const unsigned kRight = 1u << static_cast<int>(MouseButton::Right);

void testSettings()
{
    EditorCamera::Settings s;
    CHECK(s.fov == 70.0f && s.zNear == 0.05f && s.zFar == 4000.0f);
    CHECK(s.orbitSensitivity == 0.25f && s.translationSensitivity == 1.0f);
    CHECK(s.orbitInertia == 0.0f && s.translationInertia == 0.05f && s.zoomInertia == 0.05f);
    CHECK(s.angleSnapThreshold == 10.0f);
    CHECK(s.freelookBaseSpeed == 5.0f && s.freelookSensitivity == 0.25f &&
            s.freelookInertia == 0.05f);
    CHECK(s.freelookScheme == EditorCamera::FreelookScheme::Default && !s.freelookInvertY &&
            !s.freelookSpeedZoomLink);
    CHECK(s.orbitButton == EditorCamera::NavigationButton::Middle &&
            s.panButton == EditorCamera::NavigationButton::Middle &&
            s.zoomButton == EditorCamera::NavigationButton::Middle);
    CHECK(s.orbitModifier1 == Mod::None && s.orbitModifier2 == Mod::None);
    CHECK(s.panModifier1 == Mod::Shift && s.panModifier2 == Mod::None);
    CHECK(s.zoomModifier1 == Mod::Control && s.zoomModifier2 == Mod::None);
    CHECK(s.orbitSnapModifier1 == Mod::Alt && s.orbitSnapModifier2 == Mod::None);
    CHECK(s.freelookForward == Key::W && s.freelookBackwards == Key::S &&
            s.freelookLeft == Key::A && s.freelookRight == Key::D && s.freelookUp == Key::E &&
            s.freelookDown == Key::Q);
    CHECK(s.freelookSpeedModifier == Mod::Shift && s.freelookSlowModifier == Mod::Alt);
    CHECK(s.zoomStyle == EditorCamera::ZoomStyle::Vertical);
    CHECK(s.emulateNumpad && !s.emulate3ButtonMouse && !s.invertX && !s.invertY);

    EditorCamera camera;
    const EditorCamera::Cursor& cursor = camera.cursor();
    CHECK(cursor.xRot == 0.5f && cursor.yRot == -0.5f && cursor.distance == 4.0f &&
            cursor.fovScale == 1.0f);
    CHECK(cursor.posX == 0.0 && cursor.posY == 0.0 && cursor.posZ == 0.0);
    CHECK(cursor.unsnappedXRot == 0.5f && cursor.unsnappedYRot == -0.5f);
    CHECK(camera.viewType() == EditorCamera::ViewType::User && !camera.isOrthogonal() &&
            camera.orthogonalMode() == EditorCamera::OrthogonalMode::Disabled &&
            camera.autoOrthogonalAllowed());
    CHECK(camera.freelookSpeed() == 5.0f && !camera.freelookEnabled());
    CHECK(camera.zNear() == 0.05f && camera.zFar() == 4000.0f && near(camera.fov(), 70.0));
    CHECK(strcmp(camera.viewTypeName().c_str(), "Perspective") == 0);
}

void testTransform()
{
    EditorCamera camera;
    const Basis basis = expectedBasis(0.5f, -0.5f);
    const Math::Mat4 transform = camera.toCameraTransform();
    CHECK(nearVec(column(transform, 0), basis.x));
    CHECK(nearVec(column(transform, 1), basis.y));
    CHECK(nearVec(column(transform, 2), basis.z));
    CHECK(nearVec(column(transform, 3), basis.z * 4.0f));
    CHECK(near(basis.z.x, sin(0.5) * cos(0.5)) && near(basis.z.y, sin(0.5)) &&
            near(basis.z.z, cos(0.5) * cos(0.5)));
    CHECK(basis.z.x > 0.0f && basis.z.y > 0.0f && basis.z.z > 0.0f);

    camera.cursor().posX = 3.0;
    camera.cursor().posY = -2.0;
    camera.cursor().posZ = 7.0;
    camera.cursor().distance = 10.0f;
    camera.cursor().xRot = -0.3f;
    camera.cursor().yRot = 1.1f;
    const Basis other = expectedBasis(-0.3f, 1.1f);
    const Math::Mat4 moved = camera.toCameraTransform();
    CHECK(nearVec(column(moved, 2), other.z));
    CHECK(nearVec(column(moved, 3), Math::Vec3(3.0f, -2.0f, 7.0f) + other.z * 10.0f));
    CHECK(nearVec(column(camera.interpToCameraTransform(), 3), basis.z * 4.0f));
    camera.updateCamera();
    CHECK(nearVec(column(camera.interpToCameraTransform(), 3), column(moved, 3)));
}

void testOrbit()
{
    EditorCamera camera;
    CHECK(!camera.inputEvent(motion(10, 10, 5, 5, 0)));
    CHECK(!camera.isNavigating());
    CHECK(camera.cursor().xRot == 0.5f);

    CHECK(camera.inputEvent(motion(10, 10, 20, -10, kMiddle)));
    CHECK(camera.isNavigating());
    const double radians = 0.25 * kPi / 180.0;
    CHECK(near(camera.cursor().yRot, -0.5 + 20 * radians, 1e-5));
    CHECK(near(camera.cursor().xRot, 0.5 - 10 * radians, 1e-5));
    CHECK(near(camera.cursor().unsnappedYRot, camera.cursor().yRot));
    CHECK(near(camera.cursor().unsnappedXRot, camera.cursor().xRot));
    CHECK(camera.viewType() == EditorCamera::ViewType::User);

    camera.inputEvent(motion(10, 10, 0, 100000, kMiddle));
    CHECK(camera.cursor().xRot == 1.57f);
    camera.inputEvent(motion(10, 10, 0, -100000, kMiddle));
    CHECK(camera.cursor().xRot == -1.57f);

    EditorCamera inverted;
    EditorCamera::Settings settings = inverted.settings();
    settings.invertX = true;
    settings.invertY = true;
    inverted.setSettings(settings);
    inverted.inputEvent(motion(0, 0, 8, 8, kMiddle));
    CHECK(near(inverted.cursor().yRot, -0.5 - 8 * radians, 1e-5));
    CHECK(near(inverted.cursor().xRot, 0.5 - 8 * radians, 1e-5));

    EditorCamera locked;
    locked.setLockRotation(true);
    locked.inputEvent(motion(0, 0, 30, 0, kMiddle));
    CHECK(locked.cursor().yRot == -0.5f && locked.cursor().xRot == 0.5f);
    CHECK(near(locked.cursor().posX, expectedBasis(0.5f, -0.5f).x.x * (-30.0 / 150.0), 1e-5));

    EditorCamera sensitive;
    EditorCamera::Settings fast = sensitive.settings();
    fast.orbitSensitivity = 1.0f;
    sensitive.setSettings(fast);
    sensitive.inputEvent(motion(0, 0, 10, 0, kMiddle));
    CHECK(near(sensitive.cursor().yRot, -0.5 + 10 * kPi / 180.0, 1e-5));
}

void testPan()
{
    EditorCamera camera;
    camera.inputEvent(key(Key::LeftShift, true, kModShift));
    CHECK(camera.modifiers() == kModShift);
    CHECK(camera.inputEvent(motion(0, 0, 30, 12, kMiddle)));
    const Basis basis = expectedBasis(0.5f, -0.5f);
    const Math::Vec3 expected = basis.x * (-30.0f / 150.0f) + basis.y * (12.0f / 150.0f);
    CHECK(nearVec(cursorPosition(camera.cursor()), expected));
    CHECK(camera.cursor().xRot == 0.5f && camera.cursor().yRot == -0.5f);

    camera.inputEvent(key(Key::LeftShift, false, 0));
    camera.cursor() = EditorCamera::Cursor();
    camera.cursor().distance = 8.0f;
    camera.inputEvent(key(Key::LeftShift, true, kModShift));
    camera.inputEvent(motion(0, 0, -15, 6, kMiddle));
    const Math::Vec3 scaled = (basis.x * (15.0f / 150.0f) + basis.y * (6.0f / 150.0f)) * 2.0f;
    CHECK(nearVec(cursorPosition(camera.cursor()), scaled));

    EditorCamera::Settings settings = camera.settings();
    settings.translationSensitivity = 3.0f;
    settings.invertX = true;
    settings.invertY = true;
    camera.setSettings(settings);
    camera.cursor() = EditorCamera::Cursor();
    camera.cursorPan(Math::Vec2(30.0f, 12.0f));
    const Math::Vec3 inverted =
            basis.x * (30.0f * 3.0f / 150.0f) - basis.y * (12.0f * 3.0f / 150.0f);
    CHECK(nearVec(cursorPosition(camera.cursor()), inverted));
}

void testZoom()
{
    EditorCamera camera;
    CHECK(camera.inputEvent(wheel(0, 0, 1.0f)));
    CHECK(near(camera.cursor().distance, 4.0 / 1.08, 1e-5));
    CHECK(camera.inputEvent(wheel(0, 0, -1.0f)));
    CHECK(near(camera.cursor().distance, 4.0, 1e-5));
    camera.inputEvent(wheel(0, 0, -2.0f));
    CHECK(near(camera.cursor().distance, 4.0 * 1.16, 1e-5));
    camera.inputEvent(wheel(0, 0, 2.0f));
    CHECK(near(camera.cursor().distance, 4.0, 1e-5));
    CHECK(!camera.inputEvent(wheel(0, 0, 0.0f)));

    camera.inputEvent(key(Key::LeftControl, true, kModControl));
    camera.inputEvent(motion(0, 0, 0, 16, kMiddle));
    CHECK(near(camera.cursor().distance, 4.0 * 1.2, 1e-5));
    camera.inputEvent(motion(0, 0, 0, -16, kMiddle));
    CHECK(near(camera.cursor().distance, 4.0, 1e-4));
    CHECK(camera.cursor().xRot == 0.5f && camera.cursor().yRot == -0.5f);

    EditorCamera::Settings settings = camera.settings();
    settings.zoomStyle = EditorCamera::ZoomStyle::Horizontal;
    camera.setSettings(settings);
    camera.cursor().distance = 4.0f;
    camera.inputEvent(motion(0, 0, 8, 0, kMiddle));
    CHECK(near(camera.cursor().distance, 4.0 * (1.0 - 8.0 / 80.0), 1e-5));
    camera.cursor().distance = 4.0f;
    camera.inputEvent(motion(0, 0, -8, 0, kMiddle));
    CHECK(near(camera.cursor().distance, 4.0 / (1.0 - 8.0 / 80.0), 1e-5));

    EditorCamera limits;
    CHECK(limits.zoomFailedAttempts() == 0);
    for (int i = 0; i < 80; ++i) limits.scaleCursorDistance(0.5f);
    CHECK(limits.cursor().distance == 0.2f);
    CHECK(limits.zoomFailedAttempts() > 0);
    limits.scaleCursorDistance(2.0f);
    CHECK(limits.cursor().distance == 0.4f && limits.zoomFailedAttempts() == 0);
    for (int i = 0; i < 80; ++i) limits.scaleCursorDistance(2.0f);
    CHECK(limits.cursor().distance == 1000.0f);
    CHECK(limits.zoomFailedAttempts() > 0);
}

void testInterpolation()
{
    EditorCamera camera;
    camera.updateCamera();
    CHECK(near(camera.interpolatedCursor().posX, 0.0));
    camera.cursor().posX = 10.0;
    camera.cursor().distance = 8.0f;
    camera.updateCamera(0.016f);
    const double weight = 0.016 / 0.05;
    CHECK(near(camera.interpolatedCursor().posX, 10.0 * weight, 1e-4));
    CHECK(near(camera.interpolatedCursor().distance, 4.0 + 4.0 * weight, 1e-4));
    double previous = camera.interpolatedCursor().posX;
    camera.updateCamera(0.016f);
    CHECK(near(camera.interpolatedCursor().posX, previous + (10.0 - previous) * weight, 1e-4));
    for (int i = 0; i < 40; ++i) camera.updateCamera(0.016f);
    CHECK(near(camera.interpolatedCursor().posX, 10.0, 1e-3));
    CHECK(near(camera.interpolatedCursor().distance, 8.0, 1e-3));
    CHECK(camera.interpolatedCursor().posX < 10.0);

    camera.cursor().posX = 20.0;
    camera.updateCamera(0.05f);
    CHECK(near(camera.interpolatedCursor().posX, 20.0));
    camera.cursor().posX = 30.0;
    camera.updateCamera(1.0f);
    CHECK(near(camera.interpolatedCursor().posX, 30.0));

    camera.cursor().posX = 0.0;
    camera.updateCamera();
    CHECK(near(camera.interpolatedCursor().posX, 0.0));

    CHECK(near(camera.interpToCameraTransform()[3].x, camera.toCameraTransform()[3].x));

    camera.cursor().xRot = 1.0f;
    camera.updateCamera(0.016f);
    CHECK(camera.interpolatedCursor().xRot == 1.0f);

    EditorCamera orbiting;
    EditorCamera::Settings settings = orbiting.settings();
    settings.orbitInertia = 0.5f;
    orbiting.setSettings(settings);
    orbiting.updateCamera();
    orbiting.cursor().yRot = 0.5f;
    orbiting.updateCamera(0.1f);
    const double step = -0.5 + (0.5 + 0.5) * (0.1 / 0.5);
    CHECK(near(orbiting.interpolatedCursor().yRot, step, 1e-5));
    orbiting.cursor().yRot = -0.35f;
    orbiting.updateCamera(0.1f);
    CHECK(orbiting.interpolatedCursor().yRot == -0.35f);
    orbiting.cursor().yRot = -0.1f;
    orbiting.updateCamera(0.1f);
    CHECK(near(orbiting.interpolatedCursor().yRot, -0.35 + 0.25 * 0.2, 1e-5));
}

void testFreelook()
{
    EditorCamera camera;
    camera.updateCamera();
    const Math::Vec3 start = origin(camera);
    const Basis basis = expectedBasis(0.5f, -0.5f);
    camera.setFreelookEnabled(true);
    CHECK(camera.freelookEnabled());
    CHECK(nearVec(eyePosition(camera.cursor()), start));
    CHECK(nearVec(eyePosition(camera.interpolatedCursor()), start));
    const Math::Vec3 forward = basis.z * -1.0f;

    camera.update(0.1f);
    CHECK(nearVec(origin(camera), start));

    camera.inputEvent(key(Key::W, true));
    camera.update(0.1f);
    CHECK(nearVec(cursorPosition(camera.cursor()), forward * 0.5f));
    CHECK(nearVec(eyePosition(camera.cursor()), start + forward * 0.5f));
    CHECK(nearVec(origin(camera), start + forward * 0.5f));
    CHECK(nearVec(column(camera.interpToCameraTransform(), 3), start + forward * 0.5f));

    camera.inputEvent(key(Key::W, false));
    camera.update(0.1f);
    CHECK(nearVec(origin(camera), start + forward * 0.5f));

    camera.inputEvent(key(Key::D, true));
    camera.inputEvent(key(Key::E, true));
    camera.update(0.2f);
    const Math::Vec3 afterStrafe = start + forward * 0.5f + (basis.x + basis.y) * 1.0f;
    CHECK(nearVec(origin(camera), afterStrafe));
    camera.inputEvent(key(Key::D, false));
    camera.inputEvent(key(Key::E, false));

    camera.inputEvent(key(Key::S, true));
    camera.inputEvent(key(Key::LeftShift, true, kModShift));
    const Math::Vec3 beforeFast = origin(camera);
    camera.update(0.1f);
    CHECK(nearVec(origin(camera), beforeFast - forward * (5.0f * 3.0f * 0.1f)));
    camera.inputEvent(key(Key::LeftShift, false, 0));
    camera.inputEvent(key(Key::LeftAlt, true, kModAlt));
    const Math::Vec3 beforeSlow = origin(camera);
    camera.update(0.3f);
    CHECK(nearVec(origin(camera), beforeSlow - forward * (5.0f * 0.333333f * 0.3f)));
    camera.inputEvent(key(Key::LeftAlt, false, 0));
    camera.inputEvent(key(Key::S, false));

    const Math::Vec3 beforeLook = origin(camera);
    CHECK(camera.inputEvent(motion(5, 5, 10, 20, 0)));
    CHECK(!camera.isNavigating());
    CHECK(near(camera.cursor().yRot, -0.5 + 10 * 0.25 * kPi / 180.0, 1e-5));
    CHECK(near(camera.cursor().xRot, 0.5 + 20 * 0.25 * kPi / 180.0, 1e-5));
    CHECK(nearVec(origin(camera), beforeLook, 1e-4f));
    CHECK(camera.viewType() == EditorCamera::ViewType::User);

    CHECK(camera.inputEvent(wheel(0, 0, 1.0f)));
    CHECK(near(camera.freelookSpeed(), 5.0 * 1.08, 1e-5));
    CHECK(near(camera.cursor().distance, 4.0, 1e-6));
    camera.inputEvent(wheel(0, 0, -1.0f));
    CHECK(near(camera.freelookSpeed(), 5.0, 1e-4));
    for (int i = 0; i < 120; ++i) camera.scaleFreelookSpeed(0.5f);
    CHECK(camera.freelookSpeed() == 0.2f);
    for (int i = 0; i < 120; ++i) camera.scaleFreelookSpeed(2.0f);
    CHECK(camera.freelookSpeed() == 1000.0f);

    camera.setFreelookEnabled(false);
    CHECK(!camera.freelookEnabled());
    camera.inputEvent(key(Key::W, true));
    const Math::Vec3 idle = origin(camera);
    camera.update(0.5f);
    CHECK(nearVec(origin(camera), idle));
    camera.inputEvent(key(Key::W, false));

    EditorCamera mouse;
    mouse.updateCamera();
    CHECK(mouse.inputEvent(button(0, 0, MouseButton::Right, true)));
    CHECK(mouse.freelookEnabled());
    mouse.inputEvent(button(0, 0, MouseButton::Right, false));
    CHECK(!mouse.freelookEnabled());
    mouse.inputEvent(button(0, 0, MouseButton::Right, true, kModShift));
    CHECK(!mouse.freelookEnabled());
    mouse.inputEvent(button(0, 0, MouseButton::Right, false));
    mouse.inputEvent(button(0, 0, MouseButton::Right, true));
    CHECK(mouse.freelookEnabled());
    mouse.inputEvent(key(Key::Escape, true));
    CHECK(!mouse.freelookEnabled());
    mouse.inputEvent(key(Key::F, true, kModShift));
    CHECK(mouse.freelookEnabled());
    mouse.inputEvent(key(Key::F, true, kModShift, true));
    CHECK(mouse.freelookEnabled());
    mouse.inputEvent(key(Key::F, true, kModShift));
    CHECK(!mouse.freelookEnabled());

    EditorCamera locked;
    EditorCamera::Settings settings = locked.settings();
    settings.freelookScheme = EditorCamera::FreelookScheme::FullyAxisLocked;
    locked.setSettings(settings);
    locked.updateCamera();
    locked.setFreelookEnabled(true);
    locked.inputEvent(key(Key::W, true));
    locked.inputEvent(key(Key::E, true));
    const Math::Vec3 lockedStart = origin(locked);
    locked.update(0.1f);
    const Math::Vec3 horizontal = Math::Vec3(basis.z.x, 0.0f, basis.z.z).Normalized() * -1.0f;
    const Math::Vec3 lockedMoved = origin(locked) - lockedStart;
    CHECK(near(lockedMoved.y, 0.5, 1e-4));
    CHECK(nearVec(Math::Vec3(lockedMoved.x, 0.0f, lockedMoved.z), horizontal * 0.5f, 1e-4f));

    EditorCamera partial;
    settings.freelookScheme = EditorCamera::FreelookScheme::PartiallyAxisLocked;
    partial.setSettings(settings);
    partial.updateCamera();
    partial.setFreelookEnabled(true);
    partial.inputEvent(key(Key::E, true));
    const Math::Vec3 partialStart = origin(partial);
    partial.update(0.1f);
    CHECK(nearVec(origin(partial) - partialStart, Math::Vec3(0.0f, 0.5f, 0.0f), 1e-4f));
}

void testOrthogonal()
{
    EditorCamera camera;
    EditorCamera::Projection perspective = camera.projection();
    CHECK(!perspective.orthogonal && near(perspective.fov, 70.0) && perspective.zNear == 0.05f &&
            perspective.zFar == 4000.0f);

    camera.setOrthogonal(true);
    CHECK(camera.isOrthogonal() &&
            camera.orthogonalMode() == EditorCamera::OrthogonalMode::Enabled);
    const EditorCamera::Projection ortho = camera.projection();
    CHECK(ortho.orthogonal);
    CHECK(near(ortho.size, 2.0 * 4.0 * tan(35.0 * kPi / 180.0), 1e-4));
    const Basis basis = expectedBasis(0.5f, -0.5f);
    CHECK(nearVec(origin(camera), basis.z * ((4000.0f - 0.05f) / 2.0f), 1e-2f));

    camera.cursor().distance = 8.0f;
    CHECK(near(camera.projection().size, 2.0 * 8.0 * tan(35.0 * kPi / 180.0), 1e-4));
    CHECK(nearVec(origin(camera), basis.z * ((4000.0f - 0.05f) / 2.0f), 1e-2f));

    camera.cursor().fovScale = 0.5f;
    CHECK(near(camera.fov(), 35.0, 1e-4));
    camera.cursor().fovScale = 2.5f;
    CHECK(near(camera.fov(), 175.0, 1e-4));
    EditorCamera::Settings settings = camera.settings();
    settings.fov = 100.0f;
    camera.setSettings(settings);
    CHECK(camera.fov() == 179.0f);
    settings.zNear = 0.0f;
    settings.zFar = 5000000.0f;
    camera.setSettings(settings);
    camera.updateCamera();
    CHECK(camera.zNear() == 0.01f && camera.zFar() == 1000000.0f);

    EditorCamera panning;
    panning.setOrthogonal(true);
    panning.updateCamera();
    CHECK(panning.inputEvent(motion(0, 0, 30, 12, kRight)));
    CHECK(panning.isNavigating());
    CHECK(nearVec(cursorPosition(panning.cursor()),
            basis.x * (-30.0f / 150.0f) + basis.y * (12.0f / 150.0f)));

    EditorCamera blocked;
    blocked.setOrthogonal(true);
    blocked.inputEvent(button(0, 0, MouseButton::Right, true));
    CHECK(!blocked.freelookEnabled());
    blocked.inputEvent(key(Key::F, true, kModShift));
    CHECK(!blocked.freelookEnabled());

    EditorCamera numpad;
    CHECK(numpad.inputEvent(key(Key::Kp5, true)));
    CHECK(numpad.isOrthogonal());
    CHECK(strcmp(numpad.viewTypeName().c_str(), "Orthogonal") == 0);
    CHECK(numpad.inputEvent(key(Key::Kp5, true)));
    CHECK(!numpad.isOrthogonal());
}

struct ViewCase
{
    EditorCamera::ViewType type;
    float xRot;
    float yRot;
    const char* name;
};

void testViews()
{
    const ViewCase cases[6] = {
        { EditorCamera::ViewType::Top, kPi / 2.0f, 0.0f, "Top Orthogonal [auto]" },
        { EditorCamera::ViewType::Bottom, -kPi / 2.0f, 0.0f, "Bottom Orthogonal [auto]" },
        { EditorCamera::ViewType::Left, 0.0f, kPi / 2.0f, "Left Orthogonal [auto]" },
        { EditorCamera::ViewType::Right, 0.0f, -kPi / 2.0f, "Right Orthogonal [auto]" },
        { EditorCamera::ViewType::Front, 0.0f, 0.0f, "Front Orthogonal [auto]" },
        { EditorCamera::ViewType::Rear, 0.0f, kPi, "Rear Orthogonal [auto]" },
    };
    for (const ViewCase& item: cases)
    {
        EditorCamera camera;
        camera.showView(item.type);
        CHECK(camera.viewType() == item.type);
        CHECK(near(camera.cursor().xRot, item.xRot, 1e-6) &&
                near(camera.cursor().yRot, item.yRot, 1e-6));
        CHECK(camera.cursor().unsnappedXRot == camera.cursor().xRot &&
                camera.cursor().unsnappedYRot == camera.cursor().yRot);
        CHECK(camera.orthogonalMode() == EditorCamera::OrthogonalMode::Auto &&
                camera.isOrthogonal());
        CHECK(strcmp(camera.viewTypeName().c_str(), item.name) == 0);
        CHECK(nearVec(column(camera.interpToCameraTransform(), 2),
                column(camera.toCameraTransform(), 2)));
    }

    {
        EditorCamera camera;
        camera.showView(EditorCamera::ViewType::Front);
        const Math::Mat4 transform = camera.toCameraTransform();
        CHECK(nearVec(column(transform, 0), Math::Vec3(1.0f, 0.0f, 0.0f)));
        CHECK(nearVec(column(transform, 1), Math::Vec3(0.0f, 1.0f, 0.0f)));
        CHECK(nearVec(column(transform, 2), Math::Vec3(0.0f, 0.0f, 1.0f)));
        camera.showView(EditorCamera::ViewType::Top);
        CHECK(nearVec(column(camera.toCameraTransform(), 2), Math::Vec3(0.0f, 1.0f, 0.0f)));
        camera.showView(EditorCamera::ViewType::Right);
        CHECK(nearVec(column(camera.toCameraTransform(), 2), Math::Vec3(1.0f, 0.0f, 0.0f)));
        camera.showView(EditorCamera::ViewType::Left);
        CHECK(nearVec(column(camera.toCameraTransform(), 2), Math::Vec3(-1.0f, 0.0f, 0.0f)));
        camera.showView(EditorCamera::ViewType::Rear);
        CHECK(nearVec(column(camera.toCameraTransform(), 2), Math::Vec3(0.0f, 0.0f, -1.0f)));
        camera.showView(EditorCamera::ViewType::Bottom);
        CHECK(nearVec(column(camera.toCameraTransform(), 2), Math::Vec3(0.0f, -1.0f, 0.0f)));
    }

    {
        EditorCamera camera;
        camera.showView(EditorCamera::ViewType::Top);
        CHECK(camera.isOrthogonal());
        camera.inputEvent(motion(0, 0, 3, 0, kMiddle));
        CHECK(camera.viewType() == EditorCamera::ViewType::User);
        CHECK(camera.orthogonalMode() == EditorCamera::OrthogonalMode::Disabled &&
                !camera.isOrthogonal());
        CHECK(strcmp(camera.viewTypeName().c_str(), "Perspective") == 0);
    }

    {
        EditorCamera camera;
        camera.setOrthogonal(true);
        camera.showView(EditorCamera::ViewType::Front);
        CHECK(camera.orthogonalMode() == EditorCamera::OrthogonalMode::Enabled);
        CHECK(strcmp(camera.viewTypeName().c_str(), "Front Orthogonal") == 0);
        camera.inputEvent(motion(0, 0, 3, 0, kMiddle));
        CHECK(camera.orthogonalMode() == EditorCamera::OrthogonalMode::Enabled);
        CHECK(camera.viewType() == EditorCamera::ViewType::User);
    }

    {
        EditorCamera camera;
        camera.setAutoOrthogonalAllowed(false);
        camera.showView(EditorCamera::ViewType::Top);
        CHECK(!camera.isOrthogonal());
        CHECK(strcmp(camera.viewTypeName().c_str(), "Top Perspective") == 0);
        camera.setAutoOrthogonalAllowed(true);
        camera.showView(EditorCamera::ViewType::Front);
        CHECK(camera.orthogonalMode() == EditorCamera::OrthogonalMode::Auto);
        camera.setAutoOrthogonalAllowed(false);
        CHECK(camera.orthogonalMode() == EditorCamera::OrthogonalMode::Enabled);
    }

    {
        EditorCamera camera;
        CHECK(camera.inputEvent(key(Key::Kp7, true)));
        CHECK(camera.viewType() == EditorCamera::ViewType::Top);
        CHECK(camera.inputEvent(key(Key::Kp7, true, kModAlt)));
        CHECK(camera.viewType() == EditorCamera::ViewType::Bottom);
        CHECK(camera.inputEvent(key(Key::Kp1, true)));
        CHECK(camera.viewType() == EditorCamera::ViewType::Front);
        CHECK(camera.inputEvent(key(Key::Kp1, true, kModAlt)));
        CHECK(camera.viewType() == EditorCamera::ViewType::Rear);
        CHECK(camera.inputEvent(key(Key::Kp3, true)));
        CHECK(camera.viewType() == EditorCamera::ViewType::Right);
        CHECK(camera.inputEvent(key(Key::Kp3, true, kModAlt)));
        CHECK(camera.viewType() == EditorCamera::ViewType::Left);
        CHECK(!camera.inputEvent(key(Key::Kp7, true, kModShift)));
        CHECK(camera.viewType() == EditorCamera::ViewType::Left);
        CHECK(!camera.inputEvent(key(Key::Kp7, false)));

        camera.inputEvent(key(Key::Seven, true));
        CHECK(camera.viewType() == EditorCamera::ViewType::Top);
        EditorCamera::Settings settings = camera.settings();
        settings.emulateNumpad = false;
        camera.setSettings(settings);
        camera.inputEvent(key(Key::One, true));
        CHECK(camera.viewType() == EditorCamera::ViewType::Top);
        CHECK(camera.emulateNumpadKey(Key::One) == Key::One);
        settings.emulateNumpad = true;
        camera.setSettings(settings);
        CHECK(camera.emulateNumpadKey(Key::Zero) == Key::Kp0);
        CHECK(camera.emulateNumpadKey(Key::Nine) == Key::Kp9);
        CHECK(camera.emulateNumpadKey(Key::A) == Key::A);

        EditorCamera steps;
        steps.inputEvent(key(Key::Kp4, true));
        CHECK(near(steps.cursor().yRot, -0.5 + kPi / 12.0, 1e-5));
        steps.inputEvent(key(Key::Kp6, true));
        CHECK(near(steps.cursor().yRot, -0.5, 1e-5));
        steps.inputEvent(key(Key::Kp8, true));
        CHECK(near(steps.cursor().xRot, 0.5 + kPi / 12.0, 1e-5));
        steps.inputEvent(key(Key::Kp2, true));
        CHECK(near(steps.cursor().xRot, 0.5, 1e-5));
        steps.inputEvent(key(Key::Kp9, true));
        CHECK(near(steps.cursor().yRot, -0.5 + kPi, 1e-5));
        for (int i = 0; i < 20; ++i) steps.inputEvent(key(Key::Kp8, true));
        CHECK(steps.cursor().xRot == 1.57f && steps.cursor().unsnappedXRot == 1.57f);
        steps.cursor().posX = 5.0;
        steps.inputEvent(key(Key::O, true));
        CHECK(steps.cursor().posX == 0.0);
    }
}

void testSnapAndCancel()
{
    EditorCamera snap;
    snap.cursor().xRot = 0.05f;
    snap.cursor().yRot = 0.05f;
    snap.cursor().unsnappedXRot = 0.05f;
    snap.cursor().unsnappedYRot = 0.05f;
    snap.inputEvent(key(Key::LeftAlt, true, kModAlt));
    snap.inputEvent(motion(0, 0, 2, 2, kMiddle));
    CHECK(snap.cursor().xRot == 0.0f && snap.cursor().yRot == 0.0f);
    CHECK(snap.cursor().unsnappedXRot > 0.05f && snap.cursor().unsnappedYRot > 0.05f);
    CHECK(snap.viewType() == EditorCamera::ViewType::Front);
    CHECK(snap.isOrthogonal());

    snap.cursor() = EditorCamera::Cursor();
    snap.inputEvent(key(Key::LeftAlt, true, kModAlt));
    snap.cursor().xRot = kPi / 2.0f - 0.05f;
    snap.cursor().unsnappedXRot = snap.cursor().xRot;
    snap.cursor().yRot = 0.04f;
    snap.cursor().unsnappedYRot = 0.04f;
    snap.inputEvent(motion(0, 0, 1, 1, kMiddle));
    CHECK(near(snap.cursor().xRot, kPi / 2.0, 1e-5) && snap.cursor().yRot == 0.0f);
    CHECK(snap.viewType() == EditorCamera::ViewType::Top);

    snap.cursor().unsnappedXRot = 0.0f;
    snap.cursor().xRot = 0.0f;
    snap.cursor().unsnappedYRot = kPi - 0.04f;
    snap.cursor().yRot = kPi - 0.04f;
    snap.inputEvent(motion(0, 0, 1, 0, kMiddle));
    CHECK(snap.viewType() == EditorCamera::ViewType::Rear);
    snap.cursor().unsnappedYRot = kPi / 2.0f - 0.04f;
    snap.inputEvent(motion(0, 0, 1, 0, kMiddle));
    CHECK(snap.viewType() == EditorCamera::ViewType::Left);
    snap.cursor().unsnappedYRot = -kPi / 2.0f + 0.04f;
    snap.inputEvent(motion(0, 0, 1, 0, kMiddle));
    CHECK(snap.viewType() == EditorCamera::ViewType::Right);
    snap.cursor().unsnappedYRot = kPi / 4.0f + 0.04f;
    snap.inputEvent(motion(0, 0, 1, 0, kMiddle));
    CHECK(snap.viewType() == EditorCamera::ViewType::User);
    CHECK(near(snap.cursor().yRot, kPi / 4.0, 1e-5));
    snap.inputEvent(key(Key::LeftAlt, false, 0));
    snap.cursor().unsnappedYRot = 0.04f;
    snap.inputEvent(motion(0, 0, 1, 0, kMiddle));
    CHECK(snap.cursor().yRot != 0.0f);

    EditorCamera cancel;
    const EditorCamera::Cursor before = cancel.cursor();
    cancel.inputEvent(motion(0, 0, 40, 40, kMiddle));
    cancel.inputEvent(motion(0, 0, 40, 40, kMiddle));
    CHECK(cancel.isNavigating() && cancel.cursor().yRot != before.yRot);
    CHECK(cancel.inputEvent(button(0, 0, MouseButton::Right, true)));
    CHECK(!cancel.isNavigating());
    CHECK(cancel.cursor().yRot == before.yRot && cancel.cursor().xRot == before.xRot);
    CHECK(!cancel.inputEvent(motion(0, 0, 40, 40, kMiddle)));
    CHECK(cancel.cursor().yRot == before.yRot);
    CHECK(!cancel.inputEvent(motion(0, 0, 0, 0, 0)));
    CHECK(cancel.inputEvent(motion(0, 0, 10, 0, kMiddle)));
    CHECK(cancel.cursor().yRot != before.yRot);
    cancel.inputEvent(motion(0, 0, 10, 0, kMiddle));
    CHECK(cancel.inputEvent(key(Key::Escape, true)));
    CHECK(!cancel.isNavigating() && cancel.cursor().yRot == before.yRot);
}

void testFovAndSchemes()
{
    EditorCamera camera;
    camera.inputEvent(key(Key::Equal, true));
    CHECK(near(camera.cursor().fovScale, 0.95, 1e-6));
    CHECK(near(camera.fov(), 70.0 * 0.95, 1e-4));
    camera.inputEvent(key(Key::Equal, false));
    camera.inputEvent(key(Key::Minus, true));
    CHECK(near(camera.cursor().fovScale, 1.0, 1e-6));
    camera.inputEvent(key(Key::Minus, true));
    CHECK(near(camera.cursor().fovScale, 1.05, 1e-6));
    camera.inputEvent(key(Key::Minus, false));
    for (int i = 0; i < 100; ++i)
    {
        camera.inputEvent(key(Key::Minus, true));
        camera.inputEvent(key(Key::Minus, false));
    }
    CHECK(camera.cursor().fovScale == 2.5f);
    for (int i = 0; i < 100; ++i)
    {
        camera.inputEvent(key(Key::Equal, true));
        camera.inputEvent(key(Key::Equal, false));
    }
    CHECK(near(camera.cursor().fovScale, 0.1, 1e-6));
    camera.inputEvent(key(Key::Zero, true));
    CHECK(camera.cursor().fovScale == 1.0f);
    camera.inputEvent(key(Key::Zero, false));

    EditorCamera tablet;
    tablet.applyNavigationScheme(EditorCamera::NavigationScheme::Tablet);
    CHECK(tablet.settings().emulate3ButtonMouse);
    tablet.inputEvent(key(Key::LeftAlt, true, kModAlt));
    CHECK(tablet.inputEvent(motion(0, 0, 10, 0, 0)));
    CHECK(near(tablet.cursor().yRot, -0.5 + 10 * 0.25 * kPi / 180.0, 1e-5));
    tablet.inputEvent(key(Key::LeftAlt, false, 0));
    CHECK(!tablet.inputEvent(motion(0, 0, 10, 0, 0)));
    tablet.inputEvent(key(Key::LeftShift, true, kModShift));
    const float yRot = tablet.cursor().yRot;
    CHECK(tablet.inputEvent(motion(0, 0, 30, 0, 0)));
    CHECK(tablet.cursor().yRot == yRot);
    CHECK(near(tablet.cursor().posX, expectedBasis(0.5f, yRot).x.x * (-30.0 / 150.0), 1e-4));
    tablet.inputEvent(key(Key::LeftShift, false, 0));
    tablet.inputEvent(key(Key::LeftControl, true, kModControl));
    CHECK(tablet.inputEvent(motion(0, 0, 0, 16, 0)));
    CHECK(near(tablet.cursor().distance, 4.8, 1e-5));
    tablet.inputEvent(key(Key::LeftControl, false, 0));
    const float distance = tablet.cursor().distance;
    tablet.inputEvent(motion(0, 0, 0, 16, kLeft));
    CHECK(tablet.cursor().distance == distance);

    EditorCamera modo;
    modo.applyNavigationScheme(EditorCamera::NavigationScheme::Modo);
    modo.inputEvent(key(Key::LeftAlt, true, kModAlt));
    modo.inputEvent(motion(0, 0, 10, 0, kLeft));
    CHECK(near(modo.cursor().yRot, -0.5 + 10 * 0.25 * kPi / 180.0, 1e-5));
    modo.inputEvent(motion(0, 0, 0, 0, 0));
    modo.inputEvent(key(Key::LeftControl, true, kModAlt | kModControl));
    modo.inputEvent(motion(0, 0, 0, 16, kLeft));
    CHECK(near(modo.cursor().distance, 4.8, 1e-5));

    EditorCamera restored;
    restored.applyNavigationScheme(EditorCamera::NavigationScheme::Maya);
    CHECK(restored.settings().orbitButton == EditorCamera::NavigationButton::Left);
    restored.applyNavigationScheme(EditorCamera::NavigationScheme::Custom);
    CHECK(restored.settings().orbitButton == EditorCamera::NavigationButton::Left);
    restored.applyNavigationScheme(EditorCamera::NavigationScheme::Default);
    const EditorCamera::Settings fresh;
    CHECK(restored.settings().orbitButton == fresh.orbitButton &&
            restored.settings().panButton == fresh.panButton &&
            restored.settings().zoomButton == fresh.zoomButton &&
            restored.settings().panModifier1 == fresh.panModifier1 &&
            restored.settings().zoomModifier1 == fresh.zoomModifier1 &&
            restored.settings().orbitSnapModifier1 == fresh.orbitSnapModifier1 &&
            restored.settings().emulate3ButtonMouse == fresh.emulate3ButtonMouse);

    EditorCamera maya;
    maya.applyNavigationScheme(EditorCamera::NavigationScheme::Maya);
    maya.inputEvent(key(Key::LeftAlt, true, kModAlt));
    maya.inputEvent(motion(0, 0, 10, 0, kLeft));
    CHECK(near(maya.cursor().yRot, -0.5 + 10 * 0.25 * kPi / 180.0, 1e-5));
    maya.inputEvent(motion(0, 0, 0, 0, 0));
    maya.inputEvent(motion(0, 0, 0, 16, kRight));
    CHECK(near(maya.cursor().distance, 4.8, 1e-5));
    maya.inputEvent(motion(0, 0, 0, 0, 0));
    maya.inputEvent(key(Key::LeftAlt, false, 0));
    const double panBefore = maya.cursor().posX;
    maya.inputEvent(motion(0, 0, 30, 0, kMiddle));
    CHECK(maya.cursor().posX != panBefore);
}

bool readTexel(TextureId texture, int x, int y, unsigned char* out)
{
    prisma::RenderTarget source;
    source.texture = Main::textures()->gpu(texture);
    prisma::Rect area;
    area.x = x;
    area.y = y;
    area.width = 1;
    area.height = 1;
    return Main::device()->driver()->readPixels(source, area, out);
}

bool readScreen(int x, int y, unsigned char* out)
{
    prisma::Rect area;
    area.x = x;
    area.y = y;
    area.width = 1;
    area.height = 1;
    return Main::device()->driver()->readPixels(prisma::RenderTarget(), area, out);
}

struct Scene
{
    Node* root = nullptr;
    MeshInstance3D* floor = nullptr;
    MeshInstance3D* box = nullptr;
    DirectionalLight3D* sun = nullptr;
};

Scene buildScene(Node* parent)
{
    Scene scene;
    MeshStore& meshes = *Main::meshes();
    scene.root = parent->createChild<Node3D>();
    scene.root->setName("Scene");
    scene.floor = scene.root->createChild<MeshInstance3D>();
    scene.floor->setName("Floor");
    scene.floor->setMesh(createPlaneMesh(meshes, 40.0f, 40.0f));
    scene.floor->setColor(Math::Vec4(0.5f, 0.5f, 0.5f, 1.0f));
    scene.floor->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
    scene.box = scene.root->createChild<MeshInstance3D>();
    scene.box->setName("Box");
    scene.box->setMesh(createBoxMesh(meshes, Math::Vec3(1.0f, 1.0f, 1.0f)));
    scene.box->setColor(Math::Vec4(1.0f, 0.1f, 0.1f, 1.0f));
    scene.sun = scene.root->createChild<DirectionalLight3D>();
    scene.sun->setName("Sun");
    scene.sun->setShadow(true);
    scene.sun->lookAtFromPosition(Math::Vec3(4.0f, 6.0f, 3.0f), Math::Vec3(0.0f, 0.0f, 0.0f));
    return scene;
}

struct Rig
{
    SceneTree& tree;
    WidgetHost& host;
    EditorViewportPanel& panel;

    void send(const InputEvent& event)
    {
        host.handleEvent(event);
        panel.handleEvent(event);
    }

    void frame(float delta = 1.0f / 60.0f)
    {
        tree.process(delta);
        panel.update(delta);
        Main::rendering()->renderViewports();
        host.frame(delta, true);
        Main::rendering()->endFrame(true);
    }
};

bool hasCamera(const Node* node)
{
    if (!node) return false;
    if (node->castTo<Camera3D>()) return true;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (hasCamera(node->getChild(i))) return true;
    return false;
}

} // namespace

int main(int argc, char** argv)
{
    testSettings();
    testTransform();
    testOrbit();
    testPan();
    testZoom();
    testInterpolation();
    testFreelook();
    testOrthogonal();
    testViews();
    testSnapAndCancel();
    testFovAndSchemes();

    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "editor viewport test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr && Main::gui() != nullptr);
    if (!Main::rendering() || !Main::gui()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    RenderingServer& server = *Main::rendering();
    server.viewportSetActive(window->getViewportId(), false);

    const Scene scene = buildScene(tree.getRoot());
    CHECK(scene.box != nullptr);
    const std::size_t nodesBefore = tree.getNodeCount();
    ViewportId freedViewport;

    {
        WidgetHost host(*Main::device(), *Main::gui(), *Main::textures());
        EditorShell shell(host);
        CHECK(host.init());
        CHECK(shell.build());

        EditorViewportPanel panel(server);
        Rig rig = { tree, host, panel };
        freedViewport = panel.viewportId();
        CHECK(!panel.attached());
        CHECK(panel.viewportId().valid() && panel.cameraId().valid());
        CHECK(server.viewportGetSrgbOutput(panel.viewportId()));
        CHECK(server.viewportGetUpdateMode(panel.viewportId()) == ViewportUpdateMode::Disabled);
        CHECK(!panel.rendering() && panel.width() == 0 && panel.height() == 0);
        CHECK(!panel.attach(nullptr));
        CHECK(panel.attach(shell.panel(1)));
        CHECK(panel.attached());
        CHECK(!panel.attach(shell.panel(1)));
        CHECK(panel.view() != nullptr && panel.view()->parent() == panel.column() &&
                panel.column()->parent() == shell.panel(1));
        CHECK(panel.orthogonalButton() >= 0);
        CHECK(panel.toolBar()->isItemVisible(panel.orthogonalButton()) &&
                !panel.toolBar()->isItemChecked(panel.orthogonalButton()));
        panel.setScenario(window->getScenario());
        CHECK(panel.scenario() == window->getScenario());
        panel.setClearColor(Math::Vec4(0.2f, 0.4f, 0.6f, 1.0f));
        CHECK(tree.getNodeCount() == nodesBefore);

        rig.frame();
        rig.frame();
        rig.frame();
        const rt::Rect rect = panel.view()->absoluteRect();
        const int cx = static_cast<int>(rect.x + rect.w * 0.5f);
        const int cy = static_cast<int>(rect.y + rect.h * 0.5f);
        CHECK(rect.w > 100.0f && rect.h > 100.0f);
        CHECK(panel.width() == static_cast<std::uint32_t>(rect.w + 0.5f));
        CHECK(panel.height() == static_cast<std::uint32_t>(rect.h + 0.5f));
        CHECK(panel.rendering());
        CHECK(server.viewportGetUpdateMode(panel.viewportId()) == ViewportUpdateMode::Always);
        CHECK(panel.texture().valid());
        const TextureInfo* info = Main::textures()->get(panel.texture());
        CHECK(info != nullptr && info->width == panel.width() && info->height == panel.height());
        CHECK(info != nullptr && !info->srgb);
        CHECK(WidgetHost::textureId(panel.shownTexture()) == panel.texture());
        CHECK(static_cast<bool>(panel.shownTexture()));
        CHECK(static_cast<bool>(WidgetHost::textureHandle(panel.texture())));
        CHECK(!static_cast<bool>(WidgetHost::textureHandle(TextureId())));
        CHECK(!WidgetHost::textureId(rt::TextureHandle()).valid());
        CHECK(host.textureCount() > 0);
        const unsigned int firstResizes = panel.resizeCount();
        CHECK(firstResizes == 1);

        const TextureId steady = panel.texture();
        const std::size_t texturesSteady = Main::textures()->count();
        for (int i = 0; i < 10; ++i) rig.frame();
        CHECK(panel.resizeCount() == firstResizes);
        CHECK(panel.texture() == steady);
        CHECK(Main::textures()->count() == texturesSteady);

        for (int i = 0; i < 60; ++i) rig.frame();
        const int w = static_cast<int>(panel.width());
        const int h = static_cast<int>(panel.height());
        unsigned char texel[4] = { 0, 0, 0, 0 };
        unsigned char pixel[4] = { 0, 0, 0, 0 };

        CHECK(readTexel(panel.texture(), w / 2, h / 2, texel));
        CHECK(texel[0] > texel[1] + 60 && texel[0] > texel[2] + 60);
        CHECK(readTexel(panel.texture(), 2, h - 3, texel));
        CHECK(abs(texel[0] - 51) <= 2 && abs(texel[1] - 102) <= 2 && abs(texel[2] - 153) <= 2);
        CHECK(readTexel(panel.texture(), w - 3, h - 3, texel));
        CHECK(abs(texel[0] - 51) <= 2 && abs(texel[1] - 102) <= 2 && abs(texel[2] - 153) <= 2);
        CHECK(readTexel(panel.texture(), w / 2, 20, texel));
        CHECK(abs(texel[0] - texel[1]) < 12 && texel[0] > 60 && texel[2] > texel[0]);

        rig.frame();
        {
            tree.process(1.0 / 60.0);
            panel.update(1.0f / 60.0f);
            server.renderViewports();
            host.frame(1.0f / 60.0f, true);
            CHECK(readScreen(cx, cy, pixel));
            CHECK(pixel[0] > pixel[1] + 60 && pixel[0] > pixel[2] + 60);
            CHECK(readTexel(panel.texture(), w / 2, h / 2, texel));
            CHECK(abs(pixel[0] - texel[0]) <= 3 && abs(pixel[1] - texel[1]) <= 3 &&
                    abs(pixel[2] - texel[2]) <= 3);
            CHECK(readScreen(static_cast<int>(rect.x + rect.w) - 4, static_cast<int>(rect.y) + 4,
                    pixel));
            CHECK(abs(pixel[0] - 51) <= 2 && abs(pixel[1] - 102) <= 2 && abs(pixel[2] - 153) <= 2);
            server.endFrame(true);
        }

        panel.camera().cursor().posY = -1.5;
        panel.camera().cursor().distance = 6.0f;
        for (int i = 0; i < 80; ++i) rig.frame();
        {
            const EditorCamera::Projection projection = panel.camera().projection();
            const Math::Mat4 view = panel.camera().interpToCameraTransform().InverseAffine();
            const Math::Vec3 local = view.TransformPoint(Math::Vec3(0.0f, 0.0f, 0.0f));
            const float depth = -local.z;
            const float tanHalf = tanf(projection.fov * kPi / 360.0f);
            const float aspect = static_cast<float>(w) / static_cast<float>(h);
            const float ndcX = local.x / (depth * tanHalf * aspect);
            const float ndcY = local.y / (depth * tanHalf);
            CHECK(ndcY > 0.2f);
            const int column = static_cast<int>((ndcX * 0.5f + 0.5f) * static_cast<float>(w));
            const int fromBottom = static_cast<int>((ndcY * 0.5f + 0.5f) * static_cast<float>(h));
            const int fromTop = h - 1 - fromBottom;

            tree.process(1.0 / 60.0);
            panel.update(1.0f / 60.0f);
            server.renderViewports();
            host.frame(1.0f / 60.0f, true);
            CHECK(readTexel(panel.texture(), column, fromBottom, texel));
            CHECK(texel[0] > texel[1] + 60 && texel[0] > texel[2] + 60);
            CHECK(readTexel(panel.texture(), column, fromTop, texel));
            CHECK(!(texel[0] > texel[1] + 60 && texel[0] > texel[2] + 60));
            const int screenX = static_cast<int>(rect.x) + column;
            CHECK(readScreen(screenX, static_cast<int>(rect.y) + fromTop, pixel));
            CHECK(pixel[0] > pixel[1] + 60 && pixel[0] > pixel[2] + 60);
            CHECK(readScreen(screenX, static_cast<int>(rect.y) + fromBottom, pixel));
            CHECK(!(pixel[0] > pixel[1] + 60 && pixel[0] > pixel[2] + 60));
            server.endFrame(true);
        }

        panel.camera().cursor().posY = 0.0;
        panel.camera().cursor().distance = 4.0f;
        for (int i = 0; i < 80; ++i) rig.frame();

        const float yaw = panel.camera().cursor().yRot;
        const float pitch = panel.camera().cursor().xRot;
        const float distance = panel.camera().cursor().distance;
        const double radians = 0.25 * kPi / 180.0;
        rig.send(motion(cx, cy, 0, 0, 0));
        CHECK(!panel.dragging());
        rig.send(button(cx, cy, MouseButton::Middle, true));
        CHECK(panel.dragging());
        rig.send(motion(cx + 12, cy + 4, 12, 4, kMiddle));
        CHECK(near(panel.camera().cursor().yRot, yaw + 12 * radians, 1e-5));
        CHECK(near(panel.camera().cursor().xRot, pitch + 4 * radians, 1e-5));
        CHECK(panel.camera().isNavigating());
        rig.send(motion(5, 5, 10, 0, kMiddle));
        CHECK(near(panel.camera().cursor().yRot, yaw + 22 * radians, 1e-5));
        rig.send(button(5, 5, MouseButton::Middle, false));
        CHECK(!panel.dragging());
        rig.send(motion(cx, cy, 0, 0, 0));
        CHECK(!panel.camera().isNavigating());

        const float yawAfter = panel.camera().cursor().yRot;
        const float pitchAfter = panel.camera().cursor().xRot;
        rig.send(motion(5, 5, 0, 0, 0));
        rig.send(button(5, 5, MouseButton::Middle, true));
        CHECK(!panel.dragging());
        rig.send(motion(5, 5, 30, 30, kMiddle));
        rig.send(motion(cx, cy, 30, 30, kMiddle));
        rig.send(motion(cx + 3, cy, 30, 30, kMiddle));
        rig.send(button(cx, cy, MouseButton::Middle, false));
        CHECK(panel.camera().cursor().yRot == yawAfter &&
                panel.camera().cursor().xRot == pitchAfter);
        CHECK(!panel.camera().isNavigating());
        rig.send(wheel(5, 5, 1.0f));
        CHECK(panel.camera().cursor().distance == distance);
        rig.send(wheel(cx, cy, 1.0f));
        CHECK(near(panel.camera().cursor().distance, distance / 1.08, 1e-5));
        rig.send(wheel(cx, cy, -1.0f));
        CHECK(near(panel.camera().cursor().distance, distance, 1e-4));

        rt::WidgetApp& app = host.widgets();
        app.setFocused(shell.panel(0));
        rig.send(key(Key::Kp7, true));
        CHECK(panel.camera().viewType() == EditorCamera::ViewType::User);
        rig.send(key(Key::Kp7, false));
        rig.send(button(cx, cy, MouseButton::Left, true));
        CHECK(app.focused() == nullptr);
        rig.send(button(cx, cy, MouseButton::Left, false));
        rig.send(key(Key::Kp7, true));
        CHECK(panel.camera().viewType() == EditorCamera::ViewType::Top);
        rig.send(key(Key::Kp7, false));
        rig.send(motion(5, 5, 0, 0, 0));
        rig.send(key(Key::Kp1, true));
        CHECK(panel.camera().viewType() == EditorCamera::ViewType::Top);
        rig.send(key(Key::Kp1, false));
        rig.send(motion(cx, cy, 0, 0, 0));
        rig.frame();
        CHECK(panel.camera().isOrthogonal());
        CHECK(panel.toolBar()->isItemChecked(panel.orthogonalButton()));
        CHECK(strcmp(static_cast<const char*>(panel.camera().viewTypeName().c_str()),
                      "Top Orthogonal [auto]") == 0);
        for (int i = 0; i < 80; ++i) rig.frame();
        {
            const EditorCamera::Projection projection = panel.camera().projection();
            CHECK(projection.orthogonal);
            const float sidePixels = static_cast<float>(h) / projection.size;
            tree.process(1.0 / 60.0);
            panel.update(1.0f / 60.0f);
            server.renderViewports();
            host.frame(1.0f / 60.0f, true);
            const int inside = static_cast<int>(sidePixels * 0.35f);
            const int outside = static_cast<int>(sidePixels * 0.8f);
            CHECK(inside > 8 && outside > inside + 8);
            CHECK(readTexel(panel.texture(), w / 2 + inside, h / 2, texel));
            CHECK(texel[0] > texel[1] + 40 && texel[0] > texel[2] + 40);
            CHECK(readTexel(panel.texture(), w / 2 - inside, h / 2 + inside, texel));
            CHECK(texel[0] > texel[1] + 40 && texel[0] > texel[2] + 40);
            CHECK(readTexel(panel.texture(), w / 2 + outside, h / 2 + outside, texel));
            CHECK(!(texel[0] > texel[1] + 40));
            CHECK(readTexel(panel.texture(), w / 2, h / 2 - outside, texel));
            CHECK(!(texel[0] > texel[1] + 40));
            server.endFrame(true);
        }

        {
            const rt::Rect bar = panel.toolBar()->absoluteRect();
            int toggleX = -1;
            for (int x = static_cast<int>(bar.x) + 400; x > static_cast<int>(bar.x); x -= 4)
            {
                const int y = static_cast<int>(bar.y + bar.h * 0.5f);
                rig.send(motion(x, y, 0, 0, 0));
                rig.frame();
                rig.send(button(x, y, MouseButton::Left, true));
                rig.frame();
                rig.send(button(x, y, MouseButton::Left, false));
                rig.frame();
                if (!panel.camera().isOrthogonal())
                {
                    toggleX = x;
                    break;
                }
            }
            CHECK(toggleX >= 0);
            CHECK(!panel.camera().isOrthogonal());
            CHECK(!panel.toolBar()->isItemChecked(panel.orthogonalButton()));
            CHECK(strcmp(static_cast<const char*>(panel.camera().viewTypeName().c_str()),
                          "Top Perspective") == 0);
            CHECK(panel.camera().viewType() == EditorCamera::ViewType::Top);
            if (toggleX >= 0)
            {
                const int y = static_cast<int>(bar.y + bar.h * 0.5f);
                rig.send(motion(toggleX, y, 0, 0, 0));
                rig.frame();
                rig.send(button(toggleX, y, MouseButton::Left, true));
                rig.frame();
                rig.send(button(toggleX, y, MouseButton::Left, false));
                rig.frame();
                CHECK(panel.camera().isOrthogonal());
                CHECK(panel.camera().orthogonalMode() == EditorCamera::OrthogonalMode::Enabled);
                CHECK(panel.toolBar()->isItemChecked(panel.orthogonalButton()));
            }
        }
        panel.camera().resetCursor();
        CHECK(!panel.camera().isOrthogonal());
        for (int i = 0; i < 10; ++i) rig.frame();
        CHECK(!panel.toolBar()->isItemChecked(panel.orthogonalButton()));

        {
            Device* device = Main::device();
            CHECK(device->mouseGetMode() == MouseMode::Visible);
            rig.send(motion(cx, cy, 0, 0, 0));
            rig.send(button(cx, cy, MouseButton::Right, true));
            CHECK(panel.camera().freelookEnabled());
            CHECK(device->mouseGetMode() == MouseMode::Captured);
            const Math::Vec3 before = origin(panel.camera());
            rig.send(key(Key::W, true));
            rig.frame(0.1f);
            rig.send(key(Key::W, false));
            const Math::Vec3 moved = origin(panel.camera()) - before;
            CHECK(moved.Length() > 0.3f && moved.Length() < 0.7f);
            const float yawLook = panel.camera().cursor().yRot;
            rig.send(motion(2, 2, 8, 0, kRight));
            CHECK(near(panel.camera().cursor().yRot, yawLook + 8 * radians, 1e-5));
            rig.send(button(2, 2, MouseButton::Right, false));
            CHECK(!panel.camera().freelookEnabled());
            CHECK(device->mouseGetMode() == MouseMode::Visible);
            rig.send(motion(cx, cy, 0, 0, 0));
            panel.camera().resetCursor();
            for (int i = 0; i < 80; ++i) rig.frame();
        }

        {
            const unsigned int resizes = panel.resizeCount();
            const int widthBefore = static_cast<int>(panel.width());
            shell.dock()->moveTabToLeaf("Hierarchy", "Console");
            shell.dock()->layout();
            rig.frame();
            rig.frame();
            const rt::Rect grown = panel.view()->absoluteRect();
            CHECK(grown.w > rect.w + 50.0f);
            CHECK(static_cast<int>(panel.width()) > widthBefore + 50);
            CHECK(panel.width() == static_cast<std::uint32_t>(grown.w + 0.5f));
            CHECK(panel.height() == static_cast<std::uint32_t>(grown.h + 0.5f));
            CHECK(panel.resizeCount() == resizes + 1);
            const TextureInfo* resized = Main::textures()->get(panel.texture());
            CHECK(resized != nullptr && resized->width == panel.width() &&
                    resized->height == panel.height());
            CHECK(WidgetHost::textureId(panel.shownTexture()) == panel.texture());
            for (int i = 0; i < 5; ++i) rig.frame();
            CHECK(panel.resizeCount() == resizes + 1);
            CHECK(readTexel(panel.texture(), static_cast<int>(panel.width()) / 2,
                    static_cast<int>(panel.height()) / 2, texel));
            CHECK(texel[0] > texel[1] + 60);

            shell.resetLayout();
            rig.frame();
            rig.frame();
            CHECK(panel.width() == static_cast<std::uint32_t>(rect.w + 0.5f));
            CHECK(panel.resizeCount() == resizes + 2);

            shell.dock()->moveTabToLeaf("Scene", "Console");
            shell.dock()->showPanel("Console");
            shell.dock()->layout();
            rig.frame();
            rig.frame();
            CHECK(!shell.panel(1)->isVisible());
            CHECK(!panel.rendering());
            CHECK(server.viewportGetUpdateMode(panel.viewportId()) == ViewportUpdateMode::Disabled);
            const float yawHidden = panel.camera().cursor().yRot;
            rig.send(motion(cx, cy, 0, 0, 0));
            rig.send(button(cx, cy, MouseButton::Middle, true));
            rig.send(motion(cx, cy, 30, 0, kMiddle));
            rig.send(button(cx, cy, MouseButton::Middle, false));
            CHECK(panel.camera().cursor().yRot == yawHidden);
            shell.dock()->showPanel("Scene");
            shell.dock()->layout();
            rig.frame();
            rig.frame();
            CHECK(shell.panel(1)->isVisible() && panel.rendering());
            CHECK(server.viewportGetUpdateMode(panel.viewportId()) == ViewportUpdateMode::Always);
            shell.resetLayout();
            rig.frame();
            rig.frame();
        }

        {
            panel.connectPreDraw();
            panel.connectPreDraw();
            shell.dock()->moveTabToLeaf("Hierarchy", "Console");
            shell.dock()->layout();
            const int stale = static_cast<int>(panel.width());
            server.renderViewports();
            server.endFrame(true);
            CHECK(static_cast<int>(panel.width()) > stale + 50);
            panel.disconnectPreDraw();
            panel.disconnectPreDraw();
            shell.resetLayout();
            const int wide = static_cast<int>(panel.width());
            server.renderViewports();
            server.endFrame(true);
            CHECK(static_cast<int>(panel.width()) == wide);
            rig.frame();
            CHECK(static_cast<int>(panel.width()) < wide - 50);
        }

        CHECK(tree.getNodeCount() == nodesBefore);
        CHECK(window->getCamera3D() == nullptr);
        CHECK(!hasCamera(tree.getRoot()));
        CHECK(server.viewportGetUpdateMode(freedViewport) == ViewportUpdateMode::Always);

        host.shutdown();
    }

    CHECK(server.viewportGetUpdateMode(freedViewport) == ViewportUpdateMode::Disabled);
    CHECK(tree.getNodeCount() == nodesBefore);

    tree.finalize();
    Main::cleanup();

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
