#include "prisma3d/editor/EditorCamera.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const float kPi = Math::PI;
const double kCompareEpsilon = 0.00001;
const unsigned kShortcutModifiers = kModShift | kModControl | kModAlt | kModSuper;

float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

double clampd(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

float snapped(float value, float step)
{
    if (step != 0.0f) value = floorf(value / step + 0.5f) * step;
    return value;
}

bool equalApprox(float left, float right)
{
    if (left == right) return true;
    float tolerance = static_cast<float>(kCompareEpsilon) * fabsf(left);
    if (tolerance < static_cast<float>(kCompareEpsilon))
        tolerance = static_cast<float>(kCompareEpsilon);
    return fabsf(left - right) < tolerance;
}

bool zeroApprox(float value)
{
    return fabsf(value) < static_cast<float>(kCompareEpsilon);
}

float wrapf(float value, float low, float high)
{
    const float range = high - low;
    if (zeroApprox(range)) return low;
    float result = value - (range * floorf((value - low) / range));
    if (equalApprox(result, high)) return low;
    return result;
}

double lerpd(double from, double to, double weight)
{
    return from + (to - from) * weight;
}

float lerpf(float from, float to, float weight)
{
    return from + (to - from) * weight;
}

double yawOf(const Math::Mat3& basis)
{
    const double epsilon = 0.00000025;
    const double m12 = basis.col2.y;
    if (m12 < (1 - epsilon))
    {
        if (m12 > -(1 - epsilon))
        {
            if (basis.col0.y == 0 && basis.col1.x == 0 && basis.col2.x == 0 && basis.col0.z == 0 &&
                    basis.col0.x == 1)
                return 0.0;
            return atan2(static_cast<double>(basis.col2.x), static_cast<double>(basis.col2.z));
        }
        return atan2(static_cast<double>(basis.col1.x), static_cast<double>(basis.col0.x));
    }
    return -atan2(static_cast<double>(basis.col1.x), static_cast<double>(basis.col0.x));
}

} // namespace

EditorCamera::EditorCamera()
        : EditorCamera(Settings())
{
}

EditorCamera::EditorCamera(const Settings& settings)
        : mSettings(settings),
          mCursor(),
          mCursorInterp(),
          mPreviousCursor(),
          mViewType(ViewType::User),
          mOrthogonal(OrthogonalMode::Disabled),
          mAutoOrthogonalAllowed(true),
          mLockRotation(false),
          mNavigating(false),
          mNavigationCancelled(false),
          mFreelook(false),
          mFreelookSpeed(settings.freelookBaseSpeed),
          mFreelookBaseSpeed(settings.freelookBaseSpeed),
          mZoomFailedAttempts(0),
          mZNear(0.0f),
          mZFar(0.0f),
          mModifiers(0),
          mKeys{}
{
    syncLimits();
}

const EditorCamera::Settings& EditorCamera::settings() const
{
    return mSettings;
}

void EditorCamera::setSettings(const Settings& settings)
{
    mSettings = settings;
    mFreelookBaseSpeed = settings.freelookBaseSpeed;
    mFreelookSpeed = settings.freelookBaseSpeed;
    syncLimits();
}

void EditorCamera::applyNavigationScheme(NavigationScheme scheme)
{
    switch (scheme)
    {
        case NavigationScheme::Default:
            mSettings.orbitButton = NavigationButton::Middle;
            mSettings.panButton = NavigationButton::Middle;
            mSettings.zoomButton = NavigationButton::Middle;
            mSettings.emulate3ButtonMouse = false;
            mSettings.orbitModifier1 = Modifier::None;
            mSettings.orbitModifier2 = Modifier::None;
            mSettings.panModifier1 = Modifier::Shift;
            mSettings.panModifier2 = Modifier::None;
            mSettings.zoomModifier1 = Modifier::Control;
            mSettings.zoomModifier2 = Modifier::None;
            mSettings.orbitSnapModifier1 = Modifier::Alt;
            mSettings.orbitSnapModifier2 = Modifier::None;
            break;
        case NavigationScheme::Maya:
            mSettings.orbitButton = NavigationButton::Left;
            mSettings.panButton = NavigationButton::Middle;
            mSettings.zoomButton = NavigationButton::Right;
            mSettings.emulate3ButtonMouse = false;
            mSettings.orbitModifier1 = Modifier::Alt;
            mSettings.orbitModifier2 = Modifier::None;
            mSettings.panModifier1 = Modifier::None;
            mSettings.panModifier2 = Modifier::None;
            mSettings.zoomModifier1 = Modifier::Alt;
            mSettings.zoomModifier2 = Modifier::None;
            mSettings.orbitSnapModifier1 = Modifier::None;
            mSettings.orbitSnapModifier2 = Modifier::None;
            break;
        case NavigationScheme::Modo:
            mSettings.orbitButton = NavigationButton::Left;
            mSettings.panButton = NavigationButton::Left;
            mSettings.zoomButton = NavigationButton::Left;
            mSettings.emulate3ButtonMouse = false;
            mSettings.orbitModifier1 = Modifier::Alt;
            mSettings.orbitModifier2 = Modifier::None;
            mSettings.panModifier1 = Modifier::Shift;
            mSettings.panModifier2 = Modifier::Alt;
            mSettings.zoomModifier1 = Modifier::Alt;
            mSettings.zoomModifier2 = Modifier::Control;
            mSettings.orbitSnapModifier1 = Modifier::None;
            mSettings.orbitSnapModifier2 = Modifier::None;
            break;
        case NavigationScheme::Tablet:
            mSettings.orbitButton = NavigationButton::Middle;
            mSettings.panButton = NavigationButton::Middle;
            mSettings.zoomButton = NavigationButton::Middle;
            mSettings.emulate3ButtonMouse = true;
            mSettings.orbitModifier1 = Modifier::Alt;
            mSettings.orbitModifier2 = Modifier::None;
            mSettings.panModifier1 = Modifier::Shift;
            mSettings.panModifier2 = Modifier::None;
            mSettings.zoomModifier1 = Modifier::Control;
            mSettings.zoomModifier2 = Modifier::None;
            mSettings.orbitSnapModifier1 = Modifier::None;
            mSettings.orbitSnapModifier2 = Modifier::None;
            break;
        case NavigationScheme::Custom:
            break;
    }
}

EditorCamera::Cursor& EditorCamera::cursor()
{
    return mCursor;
}

const EditorCamera::Cursor& EditorCamera::cursor() const
{
    return mCursor;
}

const EditorCamera::Cursor& EditorCamera::interpolatedCursor() const
{
    return mCursorInterp;
}

void EditorCamera::resetCursor()
{
    mOrthogonal = OrthogonalMode::Disabled;
    setViewType(ViewType::User);
    mCursor = Cursor();
}

void EditorCamera::syncLimits()
{
    mZNear = clampf(mSettings.zNear, kMinZ, kMaxZ);
    mZFar = clampf(mSettings.zFar, kMinZ, kMaxZ);
}

Math::Mat3 EditorCamera::basisOf(const Cursor& cursor) const
{
    return Math::Mat3::RotationY(-cursor.yRot) * Math::Mat3::RotationX(-cursor.xRot);
}

Math::Mat4 EditorCamera::transformOf(const Cursor& cursor) const
{
    const Math::Mat3 basis = basisOf(cursor);
    const double offset = isOrthogonal() ? static_cast<double>((mZFar - mZNear) / 2.0f)
                                         : static_cast<double>(cursor.distance);
    const Math::Vec3 origin(static_cast<float>(cursor.posX + basis.col2.x * offset),
            static_cast<float>(cursor.posY + basis.col2.y * offset),
            static_cast<float>(cursor.posZ + basis.col2.z * offset));
    return Math::Mat4(basis, origin);
}

Math::Mat4 EditorCamera::toCameraTransform() const
{
    return transformOf(mCursor);
}

Math::Mat4 EditorCamera::interpToCameraTransform() const
{
    return transformOf(mCursorInterp);
}

float EditorCamera::fov() const
{
    return clampf(mSettings.fov * mCursor.fovScale, kMinFov, kMaxFov);
}

float EditorCamera::zNear() const
{
    return mZNear;
}

float EditorCamera::zFar() const
{
    return mZFar;
}

EditorCamera::Projection EditorCamera::projection() const
{
    Projection result;
    result.orthogonal = isOrthogonal();
    result.fov = fov();
    result.zNear = mZNear;
    result.zFar = mZFar;
    const float halfFov = result.fov * Math::DEG2RAD / 2.0f;
    result.size = 2.0f * mCursor.distance * tanf(halfFov);
    return result;
}

void EditorCamera::setModifiers(unsigned modifiers)
{
    mModifiers = modifiers;
}

unsigned EditorCamera::modifiers() const
{
    return mModifiers;
}

void EditorCamera::setKeyPressed(Key key, bool pressed)
{
    const int index = static_cast<int>(key);
    if (index > 0 && index < static_cast<int>(Key::Max)) mKeys[index] = pressed;
}

bool EditorCamera::isKeyPressed(Key key) const
{
    const int index = static_cast<int>(key);
    return index > 0 && index < static_cast<int>(Key::Max) && mKeys[index];
}

bool EditorCamera::modifierPressed(Modifier modifier, bool trueIfEmpty) const
{
    switch (modifier)
    {
        case Modifier::None:
            return trueIfEmpty;
        case Modifier::Shift:
            return (mModifiers & kModShift) != 0;
        case Modifier::Control:
            return (mModifiers & kModControl) != 0;
        case Modifier::Alt:
            return (mModifiers & kModAlt) != 0;
        case Modifier::Super:
            return (mModifiers & kModSuper) != 0;
    }
    return false;
}

int EditorCamera::shortcutCount(Modifier first, Modifier second) const
{
    return (first != Modifier::None ? 1 : 0) + (second != Modifier::None ? 1 : 0);
}

int EditorCamera::buildShortcutChecks(ShortcutCheck* out) const
{
    const Settings& s = mSettings;
    ShortcutCheck checks[3] = {
        { modifierPressed(s.orbitModifier1, true) && modifierPressed(s.orbitModifier2, true),
            s.orbitModifier1 != Modifier::None || s.orbitModifier2 != Modifier::None,
            shortcutCount(s.orbitModifier1, s.orbitModifier2), s.orbitButton,
            NavigationMode::Orbit },
        { modifierPressed(s.panModifier1, true) && modifierPressed(s.panModifier2, true),
            s.panModifier1 != Modifier::None || s.panModifier2 != Modifier::None,
            shortcutCount(s.panModifier1, s.panModifier2), s.panButton, NavigationMode::Pan },
        { modifierPressed(s.zoomModifier1, true) && modifierPressed(s.zoomModifier2, true),
            s.zoomModifier1 != Modifier::None || s.zoomModifier2 != Modifier::None,
            shortcutCount(s.zoomModifier1, s.zoomModifier2), s.zoomButton, NavigationMode::Zoom },
    };
    for (int i = 1; i < 3; ++i)
    {
        const ShortcutCheck item = checks[i];
        int j = i - 1;
        while (j >= 0 && checks[j].inputCount < item.inputCount)
        {
            checks[j + 1] = checks[j];
            --j;
        }
        checks[j + 1] = item;
    }
    for (int i = 0; i < 3; ++i) out[i] = checks[i];
    return 3;
}

EditorCamera::NavigationMode EditorCamera::navModeFromShortcuts(NavigationButton button,
        const ShortcutCheck* checks, int count, bool notEmpty) const
{
    if (notEmpty)
    {
        for (int i = 0; i < count; ++i)
            if (checks[i].modPressed && checks[i].notEmpty) return checks[i].result;
    }
    else
    {
        for (int i = 0; i < count; ++i)
            if (checks[i].mousePreference == button && checks[i].modPressed)
                return checks[i].result;
    }
    return NavigationMode::None;
}

bool EditorCamera::navButtonWithModifiers(NavigationButton button) const
{
    const Settings& s = mSettings;
    if (s.orbitButton == button && modifierPressed(s.orbitModifier1, true) &&
            modifierPressed(s.orbitModifier2, true))
        return true;
    if (s.panButton == button && modifierPressed(s.panModifier1, true) &&
            modifierPressed(s.panModifier2, true))
        return true;
    return s.zoomButton == button && modifierPressed(s.zoomModifier1, true) &&
           modifierPressed(s.zoomModifier2, true);
}

EditorCamera::Modifier EditorCamera::singleModifier(unsigned modifiers) const
{
    if (modifiers & kModShift) return Modifier::Shift;
    if (modifiers & kModAlt) return Modifier::Alt;
    if (modifiers & kModControl) return Modifier::Control;
    if (modifiers & kModSuper) return Modifier::Super;
    return Modifier::None;
}

bool EditorCamera::isNavigating() const
{
    return mNavigating;
}

int EditorCamera::zoomFailedAttempts() const
{
    return mZoomFailedAttempts;
}

void EditorCamera::cancelNavigation()
{
    mNavigating = false;
    mNavigationCancelled = true;
    mCursor = mPreviousCursor;
}

Key EditorCamera::emulateNumpadKey(Key key) const
{
    if (mSettings.emulateNumpad && key >= Key::Zero && key <= Key::Nine)
        return static_cast<Key>(
                static_cast<int>(key) - static_cast<int>(Key::Zero) + static_cast<int>(Key::Kp0));
    return key;
}

bool EditorCamera::guiInput(const InputEvent& event)
{
    if (event.type == InputEventType::MouseButton)
    {
        if (event.mouseButton.button == MouseButton::Right && event.mouseButton.pressed &&
                mNavigating)
        {
            cancelNavigation();
            return true;
        }
        return false;
    }

    if (event.type == InputEventType::MouseWheel)
    {
        const float amount = event.mouseWheel.deltaY;
        if (amount == 0.0f) return false;
        const float factor = amount < 0.0f ? -amount : amount;
        const float zoomFactor = 1.0f + (kZoomFreelookMultiplier - 1.0f) * factor;
        if (amount > 0.0f)
        {
            if (mFreelook) scaleFreelookSpeed(zoomFactor);
            else
                scaleCursorDistance(1.0f / zoomFactor);
        }
        else
        {
            if (mFreelook) scaleFreelookSpeed(1.0f / zoomFactor);
            else
                scaleCursorDistance(zoomFactor);
        }
        return true;
    }

    if (event.type == InputEventType::MouseMotion)
    {
        ShortcutCheck checks[3];
        int count = 0;
        if (!mFreelook) count = buildShortcutChecks(checks);

        const unsigned mask = event.mouseMotion.buttonMask;
        if (mask == 0) mNavigationCancelled = false;
        if (mNavigationCancelled) return false;
        if (!mNavigating) mPreviousCursor = mCursor;

        NavigationMode mode = NavigationMode::None;
        if (mask & mouseButtonToMask(MouseButton::Left))
        {
            const NavigationMode change =
                    navModeFromShortcuts(NavigationButton::Left, checks, count, false);
            if (change != NavigationMode::None) mode = change;
        }
        else if (mFreelook || (mask & mouseButtonToMask(MouseButton::Right)))
        {
            const NavigationMode change =
                    navModeFromShortcuts(NavigationButton::Right, checks, count, false);
            if ((mask & mouseButtonToMask(MouseButton::Right)) && change != NavigationMode::None)
                mode = change;
            else if (mFreelook)
                mode = NavigationMode::Look;
            else if (isOrthogonal())
                mode = NavigationMode::Pan;
        }
        else if (mask & mouseButtonToMask(MouseButton::Middle))
        {
            const NavigationMode change =
                    navModeFromShortcuts(NavigationButton::Middle, checks, count, false);
            if (change != NavigationMode::None) mode = change;
        }
        else if (mask & mouseButtonToMask(MouseButton::X1))
        {
            const NavigationMode change =
                    navModeFromShortcuts(NavigationButton::Button4, checks, count, false);
            if (change != NavigationMode::None) mode = change;
        }
        else if (mask & mouseButtonToMask(MouseButton::X2))
        {
            const NavigationMode change =
                    navModeFromShortcuts(NavigationButton::Button5, checks, count, false);
            if (change != NavigationMode::None) mode = change;
        }
        else if (mSettings.emulate3ButtonMouse)
        {
            const NavigationMode change =
                    navModeFromShortcuts(NavigationButton::Left, checks, count, true);
            if (change != NavigationMode::None) mode = change;
        }

        const Math::Vec2 relative(static_cast<float>(event.mouseMotion.relativeX),
                static_cast<float>(event.mouseMotion.relativeY));
        switch (mode)
        {
            case NavigationMode::Pan:
                cursorPan(relative);
                break;
            case NavigationMode::Zoom:
                cursorZoom(relative);
                break;
            case NavigationMode::Orbit:
                cursorOrbit(relative);
                break;
            case NavigationMode::Look:
                cursorLook(relative);
                break;
            case NavigationMode::None:
                mNavigating = false;
                return false;
        }
        if (!mFreelook) mNavigating = true;
        return true;
    }

    if (event.type != InputEventType::Key) return false;

    if (event.key.pressed && !event.key.echo && event.key.key == Key::Escape && mNavigating)
    {
        cancelNavigation();
        return true;
    }

    bool pressed = false;
    if (isKeyPressed(Key::Equal))
    {
        mCursor.fovScale = clampf(mCursor.fovScale - 0.05f, kCameraMinFovScale, kCameraMaxFovScale);
        pressed = true;
    }
    if (isKeyPressed(Key::Minus))
    {
        mCursor.fovScale = clampf(mCursor.fovScale + 0.05f, kCameraMinFovScale, kCameraMaxFovScale);
        pressed = true;
    }
    if (isKeyPressed(Key::Zero))
    {
        mCursor.fovScale = 1.0f;
        pressed = true;
    }
    return pressed;
}

bool EditorCamera::shortcutKey(const InputEvent& event, Key key, unsigned modifiers) const
{
    return event.type == InputEventType::Key && event.key.pressed && event.key.key == key &&
           (event.key.modifiers & kShortcutModifiers) == modifiers;
}

bool EditorCamera::viewportInput(const InputEvent& event)
{
    if (event.type == InputEventType::MouseButton)
    {
        if (event.mouseButton.button != MouseButton::Right) return false;
        if (event.mouseButton.pressed)
        {
            if (navButtonWithModifiers(NavigationButton::Right)) return false;
            const Modifier modifier = singleModifier(event.mouseButton.modifiers);
            if (!isOrthogonal() && modifier == mSettings.freelookActivationModifier)
            {
                setFreelookEnabled(true);
                return true;
            }
            return false;
        }
        setFreelookEnabled(false);
        return false;
    }

    if (event.type != InputEventType::Key || !event.key.pressed) return false;

    bool handled = false;
    if (mFreelook && event.key.key == Key::Escape)
    {
        setFreelookEnabled(false);
        return true;
    }

    const float step = kPi / 12.0f;
    if (shortcutKey(event, Key::Kp7, kModAlt))
    {
        showView(ViewType::Bottom);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp7, 0))
    {
        showView(ViewType::Top);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp1, kModAlt))
    {
        showView(ViewType::Rear);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp1, 0))
    {
        showView(ViewType::Front);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp3, kModAlt))
    {
        showView(ViewType::Left);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp3, 0))
    {
        showView(ViewType::Right);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp2, 0))
    {
        mCursor.xRot = clampf(mCursor.xRot - step, -1.57f, 1.57f);
        mCursor.unsnappedXRot = mCursor.xRot;
        setViewType(ViewType::User);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp8, 0))
    {
        mCursor.xRot = clampf(mCursor.xRot + step, -1.57f, 1.57f);
        mCursor.unsnappedXRot = mCursor.xRot;
        setViewType(ViewType::User);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp6, 0))
    {
        mCursor.yRot -= step;
        mCursor.unsnappedYRot = mCursor.yRot;
        setViewType(ViewType::User);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp4, 0))
    {
        mCursor.yRot += step;
        mCursor.unsnappedYRot = mCursor.yRot;
        setViewType(ViewType::User);
        handled = true;
    }
    if (shortcutKey(event, Key::Kp9, 0))
    {
        mCursor.yRot += kPi;
        mCursor.unsnappedYRot = mCursor.yRot;
        setViewType(ViewType::User);
        handled = true;
    }
    if (shortcutKey(event, Key::O, 0))
    {
        centerToOrigin();
        handled = true;
    }
    if (shortcutKey(event, Key::Kp5, 0))
    {
        setOrthogonal(!isOrthogonal());
        updateCamera();
        handled = true;
    }

    if (!isOrthogonal() && !event.key.echo && shortcutKey(event, Key::F, kModShift))
    {
        setFreelookEnabled(!mFreelook);
        handled = true;
    }
    return handled;
}

bool EditorCamera::inputEvent(const InputEvent& event)
{
    InputEvent translated = event;
    if (event.type == InputEventType::Key)
    {
        setKeyPressed(event.key.key, event.key.pressed);
        mModifiers = event.key.modifiers;
        if (event.key.pressed) translated.key.key = emulateNumpadKey(event.key.key);
    }
    else if (event.type == InputEventType::MouseButton)
    {
        mModifiers = event.mouseButton.modifiers;
    }

    const bool wasNavigating = mNavigating;
    const bool consumed = guiInput(translated);
    if (wasNavigating && !mNavigating) return consumed;
    const bool handled = viewportInput(translated);
    return consumed || handled;
}

void EditorCamera::cursorPan(const Math::Vec2& relative)
{
    float panSpeed = mSettings.translationSensitivity / 150.0f;
    if (mSettings.navigationScheme == NavigationScheme::Maya && (mModifiers & kModShift))
        panSpeed *= 10.0f;

    const Math::Mat3 basis = basisOf(mCursor);
    Math::Vec3 translation((mSettings.invertX ? -1.0f : 1.0f) * -relative.x * panSpeed,
            (mSettings.invertY ? -1.0f : 1.0f) * relative.y * panSpeed, 0.0f);
    translation *= mCursor.distance / kDistanceDefault;
    const Math::Vec3 moved = basis * translation;
    mCursor.posX += moved.x;
    mCursor.posY += moved.y;
    mCursor.posZ += moved.z;
}

void EditorCamera::cursorOrbit(const Math::Vec2& relative)
{
    if (mLockRotation)
    {
        cursorPan(relative);
        return;
    }

    const float radiansPerPixel = mSettings.orbitSensitivity * Math::DEG2RAD;

    mCursor.unsnappedXRot += relative.y * radiansPerPixel * (mSettings.invertY ? -1.0f : 1.0f);
    mCursor.unsnappedXRot = clampf(mCursor.unsnappedXRot, -1.57f, 1.57f);
    mCursor.unsnappedYRot += relative.x * radiansPerPixel * (mSettings.invertX ? -1.0f : 1.0f);

    mCursor.xRot = mCursor.unsnappedXRot;
    mCursor.yRot = mCursor.unsnappedYRot;

    ViewType newView = ViewType::User;

    const bool snapConfigured = mSettings.orbitSnapModifier1 != Modifier::None ||
                                mSettings.orbitSnapModifier2 != Modifier::None;
    if (snapConfigured && modifierPressed(mSettings.orbitSnapModifier1, true) &&
            modifierPressed(mSettings.orbitSnapModifier2, true))
    {
        const float snapAngle = 45.0f * Math::DEG2RAD;
        const float snapThreshold = mSettings.angleSnapThreshold * Math::DEG2RAD;

        const float xSnapped = snapped(mCursor.unsnappedXRot, snapAngle);
        const float ySnapped = snapped(mCursor.unsnappedYRot, snapAngle);

        const float xDistance = fabsf(mCursor.unsnappedXRot - xSnapped);
        const float yDistance = fabsf(mCursor.unsnappedYRot - ySnapped);

        if (xDistance < snapThreshold && yDistance < snapThreshold)
        {
            mCursor.xRot = xSnapped;
            mCursor.yRot = ySnapped;

            const float yWrapped = wrapf(ySnapped, -kPi, kPi);

            if (fabsf(xSnapped) < snapThreshold)
            {
                if (fabsf(yWrapped) < snapThreshold) newView = ViewType::Front;
                else if (fabsf(fabsf(yWrapped) - kPi) < snapThreshold)
                    newView = ViewType::Rear;
                else if (fabsf(yWrapped - kPi / 2.0f) < snapThreshold)
                    newView = ViewType::Left;
                else if (fabsf(yWrapped + kPi / 2.0f) < snapThreshold)
                    newView = ViewType::Right;
            }
            else if (fabsf(fabsf(xSnapped) - kPi / 2.0f) < snapThreshold)
            {
                if (fabsf(yWrapped) < snapThreshold ||
                        fabsf(fabsf(yWrapped) - kPi) < snapThreshold ||
                        fabsf(yWrapped - kPi / 2.0f) < snapThreshold ||
                        fabsf(yWrapped + kPi / 2.0f) < snapThreshold)
                    newView = xSnapped > 0.0f ? ViewType::Top : ViewType::Bottom;
            }
        }
    }

    setViewType(newView);
}

void EditorCamera::cursorLook(const Math::Vec2& relative)
{
    if (isOrthogonal())
    {
        cursorPan(relative);
        return;
    }

    const float degreesPerPixel =
            mSettings.freelookSensitivity * (mCursor.fovScale < 1.0f ? mCursor.fovScale : 1.0f);
    const float radiansPerPixel = degreesPerPixel * Math::DEG2RAD;

    const Math::Mat4 previous = toCameraTransform();

    if (mSettings.freelookInvertY) mCursor.xRot -= relative.y * radiansPerPixel;
    else
        mCursor.xRot += relative.y * radiansPerPixel;

    mCursor.xRot = clampf(mCursor.xRot, -1.57f, 1.57f);
    mCursor.unsnappedXRot = mCursor.xRot;

    mCursor.yRot += relative.x * radiansPerPixel;
    mCursor.unsnappedYRot = mCursor.yRot;

    const Math::Mat4 current = toCameraTransform();
    const Math::Vec3 position = current.GetTranslation();
    const Math::Vec3 previousPosition = previous.GetTranslation();
    const Math::Vec3 difference = previousPosition - position;
    mCursor.posX += difference.x;
    mCursor.posY += difference.y;
    mCursor.posZ += difference.z;

    setViewType(ViewType::User);
}

void EditorCamera::cursorZoom(const Math::Vec2& relative)
{
    float zoomSpeed = 1.0f / 80.0f;
    if (mSettings.navigationScheme == NavigationScheme::Maya && (mModifiers & kModShift))
        zoomSpeed *= 10.0f;

    if (mSettings.zoomStyle == ZoomStyle::Horizontal)
    {
        if (relative.x > 0.0f) scaleCursorDistance(1.0f - relative.x * zoomSpeed);
        else if (relative.x < 0.0f)
            scaleCursorDistance(1.0f / (1.0f + relative.x * zoomSpeed));
    }
    else
    {
        if (relative.y > 0.0f) scaleCursorDistance(1.0f + relative.y * zoomSpeed);
        else if (relative.y < 0.0f)
            scaleCursorDistance(1.0f / (1.0f - relative.y * zoomSpeed));
    }
}

void EditorCamera::update(float delta)
{
    updateFreelook(delta);
    updateCamera(delta);
}

void EditorCamera::updateCamera(float delta)
{
    const Cursor old = mCursorInterp;
    mCursorInterp = mCursor;

    if (delta > 0.0f)
    {
        if (mFreelook)
        {
            const double inertia = mSettings.freelookInertia;
            const double factor = inertia > 0.0 ? (1.0 / inertia) * delta : 1e30;
            const double weight = clampd(factor, 0.0, 1.0);
            mCursorInterp.eyePosX = lerpd(old.eyePosX, mCursor.eyePosX, weight);
            mCursorInterp.eyePosY = lerpd(old.eyePosY, mCursor.eyePosY, weight);
            mCursorInterp.eyePosZ = lerpd(old.eyePosZ, mCursor.eyePosZ, weight);
        }

        if (mSettings.orbitInertia > 0.0f)
        {
            const float weight = fminf(1.0f, delta * (1.0f / mSettings.orbitInertia));
            mCursorInterp.xRot = lerpf(old.xRot, mCursor.xRot, weight);
            mCursorInterp.yRot = lerpf(old.yRot, mCursor.yRot, weight);

            if (fabsf(mCursorInterp.xRot - mCursor.xRot) < 0.1f) mCursorInterp.xRot = mCursor.xRot;
            if (fabsf(mCursorInterp.yRot - mCursor.yRot) < 0.1f) mCursorInterp.yRot = mCursor.yRot;
        }

        if (mFreelook)
        {
            const Math::Vec3 forward = basisOf(mCursorInterp) * Math::Vec3(0.0f, 0.0f, -1.0f);
            mCursorInterp.posX = mCursorInterp.eyePosX + forward.x * mCursorInterp.distance;
            mCursorInterp.posY = mCursorInterp.eyePosY + forward.y * mCursorInterp.distance;
            mCursorInterp.posZ = mCursorInterp.eyePosZ + forward.z * mCursorInterp.distance;
        }
        else
        {
            if (mSettings.translationInertia > 0.0f)
            {
                const double weight = fmin(1.0, delta * (1.0 / mSettings.translationInertia));
                mCursorInterp.posX = lerpd(old.posX, mCursor.posX, weight);
                mCursorInterp.posY = lerpd(old.posY, mCursor.posY, weight);
                mCursorInterp.posZ = lerpd(old.posZ, mCursor.posZ, weight);
            }

            if (mSettings.zoomInertia > 0.0f)
            {
                const float weight = fminf(1.0f, delta * (1.0f / mSettings.zoomInertia));
                mCursorInterp.distance = lerpf(old.distance, mCursor.distance, weight);
            }
        }
    }

    syncLimits();
}

void EditorCamera::updateFreelook(float delta)
{
    if (!mFreelook) return;

    const Math::Mat3 basis = basisOf(mCursor);

    Math::Vec3 forward;
    if (mSettings.freelookScheme == FreelookScheme::FullyAxisLocked)
    {
        const float yaw = static_cast<float>(yawOf(basis));
        forward = Math::Mat3::RotationY(yaw) * Math::Vec3(0.0f, 0.0f, -1.0f);
    }
    else
    {
        forward = basis * Math::Vec3(0.0f, 0.0f, -1.0f);
    }

    const Math::Vec3 right = basis * Math::Vec3(1.0f, 0.0f, 0.0f);

    Math::Vec3 up;
    if (mSettings.freelookScheme == FreelookScheme::PartiallyAxisLocked ||
            mSettings.freelookScheme == FreelookScheme::FullyAxisLocked)
        up = Math::Vec3(0.0f, 1.0f, 0.0f);
    else
        up = basis * Math::Vec3(0.0f, 1.0f, 0.0f);

    Math::Vec3 direction(0.0f, 0.0f, 0.0f);
    if (isKeyPressed(mSettings.freelookLeft)) direction -= right;
    if (isKeyPressed(mSettings.freelookRight)) direction += right;
    if (isKeyPressed(mSettings.freelookForward)) direction += forward;
    if (isKeyPressed(mSettings.freelookBackwards)) direction -= forward;
    if (isKeyPressed(mSettings.freelookUp)) direction += up;
    if (isKeyPressed(mSettings.freelookDown)) direction -= up;

    float speed = mFreelookSpeed;
    if (modifierPressed(mSettings.freelookSpeedModifier, false)) speed *= 3.0f;
    if (modifierPressed(mSettings.freelookSlowModifier, false)) speed *= 0.333333f;

    const Math::Vec3 motion = direction * speed * delta;
    mCursor.posX += motion.x;
    mCursor.posY += motion.y;
    mCursor.posZ += motion.z;
    mCursor.eyePosX += motion.x;
    mCursor.eyePosY += motion.y;
    mCursor.eyePosZ += motion.z;
}

void EditorCamera::scaleFreelookSpeed(float scale)
{
    const float minSpeed = fmaxf(mZNear * 4.0f, kZoomFreelookMin);
    const float maxSpeed = fminf(mZFar / 4.0f, kZoomFreelookMax);
    if (minSpeed > maxSpeed) mFreelookSpeed = (minSpeed + maxSpeed) / 2.0f;
    else
        mFreelookSpeed = clampf(mFreelookSpeed * scale, minSpeed, maxSpeed);
}

void EditorCamera::scaleCursorDistance(float scale)
{
    const float minDistance = fmaxf(mZNear * 4.0f, kZoomFreelookMin);
    const float maxDistance = fminf(mZFar / 4.0f, kZoomFreelookMax);
    if (minDistance > maxDistance) mCursor.distance = (minDistance + maxDistance) / 2.0f;
    else
        mCursor.distance = clampf(mCursor.distance * scale, minDistance, maxDistance);

    if (mCursor.distance == maxDistance || mCursor.distance == minDistance) ++mZoomFailedAttempts;
    else
        mZoomFailedAttempts = 0;
}

void EditorCamera::centerToOrigin()
{
    mCursor.posX = 0.0;
    mCursor.posY = 0.0;
    mCursor.posZ = 0.0;
}

void EditorCamera::setViewType(ViewType view)
{
    const ViewType oldView = mViewType;
    const OrthogonalMode oldOrthogonal = mOrthogonal;

    mViewType = view;
    if (mViewType != ViewType::User)
    {
        if (mAutoOrthogonalAllowed && mOrthogonal != OrthogonalMode::Enabled)
            mOrthogonal = OrthogonalMode::Auto;
    }
    else if (mOrthogonal == OrthogonalMode::Auto)
    {
        mOrthogonal = OrthogonalMode::Disabled;
    }

    if (oldView != mViewType || oldOrthogonal != mOrthogonal) viewStateChanged();
}

void EditorCamera::viewStateChanged()
{
    if (mOrthogonal == OrthogonalMode::Auto)
    {
        setOrthogonal(true);
        updateCamera();
        forceAutoOrthogonal();
    }
    else if (mOrthogonal == OrthogonalMode::Disabled)
    {
        setOrthogonal(false);
        updateCamera();
    }
}

EditorCamera::ViewType EditorCamera::viewType() const
{
    return mViewType;
}

void EditorCamera::showView(ViewType view)
{
    switch (view)
    {
        case ViewType::Top:
            mCursor.yRot = 0.0f;
            mCursor.xRot = kPi / 2.0f;
            break;
        case ViewType::Bottom:
            mCursor.yRot = 0.0f;
            mCursor.xRot = -kPi / 2.0f;
            break;
        case ViewType::Left:
            mCursor.xRot = 0.0f;
            mCursor.yRot = kPi / 2.0f;
            break;
        case ViewType::Right:
            mCursor.xRot = 0.0f;
            mCursor.yRot = -kPi / 2.0f;
            break;
        case ViewType::Front:
            mCursor.xRot = 0.0f;
            mCursor.yRot = 0.0f;
            break;
        case ViewType::Rear:
            mCursor.xRot = 0.0f;
            mCursor.yRot = kPi;
            break;
        case ViewType::User:
            break;
    }
    mCursor.unsnappedXRot = mCursor.xRot;
    mCursor.unsnappedYRot = mCursor.yRot;
    setViewType(view);
}

ct::String EditorCamera::viewTypeName() const
{
    const bool orthogonal = isOrthogonal();
    ct::String name;
    switch (mViewType)
    {
        case ViewType::User:
            name = orthogonal ? "Orthogonal" : "Perspective";
            break;
        case ViewType::Top:
            name = orthogonal ? "Top Orthogonal" : "Top Perspective";
            break;
        case ViewType::Bottom:
            name = orthogonal ? "Bottom Orthogonal" : "Bottom Perspective";
            break;
        case ViewType::Left:
            name = orthogonal ? "Left Orthogonal" : "Left Perspective";
            break;
        case ViewType::Right:
            name = orthogonal ? "Right Orthogonal" : "Right Perspective";
            break;
        case ViewType::Front:
            name = orthogonal ? "Front Orthogonal" : "Front Perspective";
            break;
        case ViewType::Rear:
            name = orthogonal ? "Rear Orthogonal" : "Rear Perspective";
            break;
    }
    if (mOrthogonal == OrthogonalMode::Auto) name += " [auto]";
    return name;
}

void EditorCamera::setFreelookEnabled(bool enabled)
{
    if (mFreelook == enabled) return;

    mFreelook = enabled;
    mCursor = mCursorInterp;

    if (mFreelook)
    {
        const Math::Vec3 forward = basisOf(mCursor) * Math::Vec3(0.0f, 0.0f, -1.0f);
        mCursor.eyePosX = mCursor.posX - mCursor.distance * forward.x;
        mCursor.eyePosY = mCursor.posY - mCursor.distance * forward.y;
        mCursor.eyePosZ = mCursor.posZ - mCursor.distance * forward.z;
        mCursorInterp.eyePosX = mCursor.eyePosX;
        mCursorInterp.eyePosY = mCursor.eyePosY;
        mCursorInterp.eyePosZ = mCursor.eyePosZ;

        if (mSettings.freelookSpeedZoomLink) mFreelookSpeed = mFreelookBaseSpeed * mCursor.distance;
    }
}

bool EditorCamera::freelookEnabled() const
{
    return mFreelook;
}

float EditorCamera::freelookSpeed() const
{
    return mFreelookSpeed;
}

void EditorCamera::setOrthogonal(bool enabled)
{
    mOrthogonal = enabled ? OrthogonalMode::Enabled : OrthogonalMode::Disabled;
}

bool EditorCamera::isOrthogonal() const
{
    return mOrthogonal != OrthogonalMode::Disabled;
}

EditorCamera::OrthogonalMode EditorCamera::orthogonalMode() const
{
    return mOrthogonal;
}

void EditorCamera::forceAutoOrthogonal()
{
    if (mAutoOrthogonalAllowed) mOrthogonal = OrthogonalMode::Auto;
}

void EditorCamera::setAutoOrthogonalAllowed(bool allowed)
{
    mAutoOrthogonalAllowed = allowed;
    if (!allowed && mOrthogonal == OrthogonalMode::Auto) mOrthogonal = OrthogonalMode::Enabled;
}

bool EditorCamera::autoOrthogonalAllowed() const
{
    return mAutoOrthogonalAllowed;
}

void EditorCamera::setLockRotation(bool locked)
{
    mLockRotation = locked;
}

bool EditorCamera::lockRotation() const
{
    return mLockRotation;
}

} // namespace prisma3d
