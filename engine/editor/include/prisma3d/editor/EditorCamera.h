#pragma once

#include "prisma3d/Input.h"

#include "mathc.h"

#include <ct/string.hpp>

namespace prisma3d
{

class EditorCamera
{
public:
    static constexpr float kDistanceDefault = 4.0f;
    static constexpr float kZoomFreelookMultiplier = 1.08f;
    static constexpr float kZoomFreelookMin = 0.01f;
    static constexpr float kZoomFreelookMax = 10000.0f;
    static constexpr float kCameraMinFovScale = 0.1f;
    static constexpr float kCameraMaxFovScale = 2.5f;
    static constexpr float kMinZ = 0.01f;
    static constexpr float kMaxZ = 1000000.0f;
    static constexpr float kMinFov = 0.01f;
    static constexpr float kMaxFov = 179.0f;

    enum class NavigationMode : int
    {
        None,
        Pan,
        Zoom,
        Orbit,
        Look
    };

    enum class NavigationScheme : int
    {
        Default,
        Maya,
        Modo,
        Custom,
        Tablet
    };

    enum class NavigationButton : int
    {
        Left,
        Middle,
        Right,
        Button4,
        Button5
    };

    enum class ZoomStyle : int
    {
        Vertical,
        Horizontal
    };

    enum class FreelookScheme : int
    {
        Default,
        PartiallyAxisLocked,
        FullyAxisLocked
    };

    enum class ViewType : int
    {
        User,
        Top,
        Bottom,
        Left,
        Right,
        Front,
        Rear
    };

    enum class OrthogonalMode : int
    {
        Disabled,
        Enabled,
        Auto
    };

    enum class Modifier : int
    {
        None,
        Shift,
        Control,
        Alt,
        Super
    };

    struct Cursor
    {
        double posX = 0.0;
        double posY = 0.0;
        double posZ = 0.0;
        float xRot = 0.5f;
        float yRot = -0.5f;
        float distance = 4.0f;
        float fovScale = 1.0f;
        float unsnappedXRot = 0.5f;
        float unsnappedYRot = -0.5f;
        double eyePosX = 0.0;
        double eyePosY = 0.0;
        double eyePosZ = 0.0;
    };

    struct Settings
    {
        float fov = 70.0f;
        float zNear = 0.05f;
        float zFar = 4000.0f;

        NavigationScheme navigationScheme = NavigationScheme::Default;
        NavigationButton orbitButton = NavigationButton::Middle;
        NavigationButton panButton = NavigationButton::Middle;
        NavigationButton zoomButton = NavigationButton::Middle;
        Modifier orbitModifier1 = Modifier::None;
        Modifier orbitModifier2 = Modifier::None;
        Modifier panModifier1 = Modifier::Shift;
        Modifier panModifier2 = Modifier::None;
        Modifier zoomModifier1 = Modifier::Control;
        Modifier zoomModifier2 = Modifier::None;
        Modifier orbitSnapModifier1 = Modifier::Alt;
        Modifier orbitSnapModifier2 = Modifier::None;
        bool emulate3ButtonMouse = false;
        bool emulateNumpad = true;
        bool invertX = false;
        bool invertY = false;

        float orbitSensitivity = 0.25f;
        float orbitInertia = 0.0f;
        float translationSensitivity = 1.0f;
        float translationInertia = 0.05f;
        float zoomInertia = 0.05f;
        ZoomStyle zoomStyle = ZoomStyle::Vertical;
        float angleSnapThreshold = 10.0f;

        FreelookScheme freelookScheme = FreelookScheme::Default;
        float freelookBaseSpeed = 5.0f;
        float freelookSensitivity = 0.25f;
        float freelookInertia = 0.05f;
        bool freelookSpeedZoomLink = false;
        bool freelookInvertY = false;
        Modifier freelookActivationModifier = Modifier::None;
        Key freelookForward = Key::W;
        Key freelookBackwards = Key::S;
        Key freelookLeft = Key::A;
        Key freelookRight = Key::D;
        Key freelookUp = Key::E;
        Key freelookDown = Key::Q;
        Modifier freelookSpeedModifier = Modifier::Shift;
        Modifier freelookSlowModifier = Modifier::Alt;
    };

    struct Projection
    {
        bool orthogonal = false;
        float fov = 70.0f;
        float size = 1.0f;
        float zNear = 0.05f;
        float zFar = 4000.0f;
    };

    EditorCamera();
    explicit EditorCamera(const Settings& settings);

    const Settings& settings() const;
    void setSettings(const Settings& settings);
    void applyNavigationScheme(NavigationScheme scheme);

    Cursor& cursor();
    const Cursor& cursor() const;
    const Cursor& interpolatedCursor() const;
    void resetCursor();

    bool inputEvent(const InputEvent& event);
    bool guiInput(const InputEvent& event);
    bool isNavigating() const;
    int zoomFailedAttempts() const;

    void setModifiers(unsigned modifiers);
    unsigned modifiers() const;
    void setKeyPressed(Key key, bool pressed);
    bool isKeyPressed(Key key) const;

    void cursorPan(const Math::Vec2& relative);
    void cursorLook(const Math::Vec2& relative);
    void cursorOrbit(const Math::Vec2& relative);
    void cursorZoom(const Math::Vec2& relative);

    void update(float delta);
    void updateCamera(float delta = 0.0f);
    void updateFreelook(float delta);
    void scaleFreelookSpeed(float scale);
    void scaleCursorDistance(float scale);
    void centerToOrigin();

    Math::Mat4 toCameraTransform() const;
    Math::Mat4 interpToCameraTransform() const;
    Projection projection() const;
    float fov() const;
    float zNear() const;
    float zFar() const;

    Key emulateNumpadKey(Key key) const;

    void setViewType(ViewType view);
    ViewType viewType() const;
    void showView(ViewType view);
    ct::String viewTypeName() const;

    void setFreelookEnabled(bool enabled);
    bool freelookEnabled() const;
    float freelookSpeed() const;

    void setOrthogonal(bool enabled);
    bool isOrthogonal() const;
    OrthogonalMode orthogonalMode() const;
    void forceAutoOrthogonal();
    void setAutoOrthogonalAllowed(bool allowed);
    bool autoOrthogonalAllowed() const;
    void setLockRotation(bool locked);
    bool lockRotation() const;
    bool navButtonWithModifiers(NavigationButton button) const;

private:
    struct ShortcutCheck
    {
        bool modPressed;
        bool notEmpty;
        int inputCount;
        NavigationButton mousePreference;
        NavigationMode result;
    };

    Math::Mat4 transformOf(const Cursor& cursor) const;
    Math::Mat3 basisOf(const Cursor& cursor) const;
    bool modifierPressed(Modifier modifier, bool trueIfEmpty) const;
    int shortcutCount(Modifier first, Modifier second) const;
    NavigationMode navModeFromShortcuts(NavigationButton button, const ShortcutCheck* checks,
            int count, bool notEmpty) const;
    int buildShortcutChecks(ShortcutCheck* out) const;
    void cancelNavigation();
    void syncLimits();
    void viewStateChanged();
    bool shortcutKey(const InputEvent& event, Key key, unsigned modifiers) const;
    bool viewportInput(const InputEvent& event);
    Modifier singleModifier(unsigned modifiers) const;
    Settings mSettings;
    Cursor mCursor;
    Cursor mCursorInterp;
    Cursor mPreviousCursor;
    ViewType mViewType;
    OrthogonalMode mOrthogonal;
    bool mAutoOrthogonalAllowed;
    bool mLockRotation;
    bool mNavigating;
    bool mNavigationCancelled;
    bool mFreelook;
    float mFreelookSpeed;
    float mFreelookBaseSpeed;
    int mZoomFailedAttempts;
    float mZNear;
    float mZFar;
    unsigned mModifiers;
    bool mKeys[static_cast<int>(Key::Max)];
};

} // namespace prisma3d
