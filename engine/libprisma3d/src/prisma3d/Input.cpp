#include "prisma3d/Input.h"

#include "prisma3d/Device.h"
#include "prisma3d/InputMap.h"

#include "platform.h"

#include <string.h>

namespace prisma3d
{

namespace
{

float clampUnit(float value)
{
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

float inverseLerp(float from, float to, float value)
{
    return (value - from) / (to - from);
}

float maxOf(float a, float b)
{
    return a > b ? a : b;
}

const std::uint64_t kNeverFrame = ~static_cast<std::uint64_t>(0);

} // namespace

Input* Input::sSingleton = nullptr;

Input::VelocityTrack::VelocityTrack()
{
    minRefFrame = 0.1f;
    maxRefFrame = 3.0f;
    reset();
}

void Input::VelocityTrack::update(const Math::Vec2& delta)
{
    const std::uint64_t tick = time_nanos() / 1000u;
    const std::uint32_t tdiff = static_cast<std::uint32_t>(tick - lastTick);
    const float deltaT = static_cast<float>(tdiff) / 1000000.0f;
    lastTick = tick;

    if (deltaT > maxRefFrame)
    {
        velocity = Math::Vec2();
        accum = delta;
        accumT = 0.0f;
        return;
    }

    accum += delta;
    accumT += deltaT;

    if (accumT < minRefFrame) return;

    velocity = accum / accumT;
    accum = Math::Vec2();
    accumT = 0.0f;
}

void Input::VelocityTrack::reset()
{
    lastTick = time_nanos() / 1000u;
    velocity = Math::Vec2();
    accum = Math::Vec2();
    accumT = 0.0f;
}

Input::ActionState::ActionState()
        : pressedProcessFrame(kNeverFrame),
          releasedProcessFrame(kNeverFrame),
          exact(true),
          apiPressed(false),
          apiStrength(0.0f)
{
    memset(pressed, 0, sizeof(pressed));
    memset(strength, 0, sizeof(strength));
    memset(rawStrength, 0, sizeof(rawStrength));
    cache.pressed = false;
    cache.strength = 0.0f;
    cache.rawStrength = 0.0f;
}

Input::Input()
        : mMap(new InputMap()),
          mProcessFrames(0),
          mMouseButtonMask(0),
          mDisableInput(false),
          mPollingEnabled(true),
          mApplicationFocused(true),
          mDispatch(nullptr),
          mDispatchUser(nullptr)
{
    memset(mKeysPressed, 0, sizeof(mKeysPressed));
    for (int i = 0; i < static_cast<int>(Key::Max); ++i)
        mKeyPressedFrame[i] = mKeyReleasedFrame[i] = kNeverFrame;
    for (int i = 0; i < static_cast<int>(MouseButton::Max); ++i)
        mMouseButtonPressedFrame[i] = mMouseButtonReleasedFrame[i] = kNeverFrame;
    sSingleton = this;
}

Input::~Input()
{
    if (sSingleton == this) sSingleton = nullptr;
    delete mMap;
}

Input* Input::singleton()
{
    return sSingleton;
}

InputMap& Input::map()
{
    return *mMap;
}

const InputMap& Input::map() const
{
    return *mMap;
}

void Input::beginFrame()
{
    ++mProcessFrames;
    mMouseDelta = Math::Vec2();
    mMouseWheel = Math::Vec2();
}

std::uint64_t Input::getProcessFrames() const
{
    return mProcessFrames;
}

bool Input::isAnythingPressed() const
{
    if (blocked()) return false;
    if (mMouseButtonMask != 0 || isAnyKeyPressed()) return true;
    for (const auto& entry: mActionStates)
        if (entry.value.cache.pressed) return true;
    return false;
}

bool Input::isAnyKeyPressed() const
{
    if (blocked()) return false;
    for (int i = 0; i < static_cast<int>(Key::Max); ++i)
        if (mKeysPressed[i]) return true;
    return false;
}

bool Input::isKeyPressed(Key key) const
{
    if (blocked()) return false;
    const int index = static_cast<int>(key);
    if (index <= 0 || index >= static_cast<int>(Key::Max)) return false;
    return mKeysPressed[index];
}

bool Input::isMouseButtonPressed(MouseButton button) const
{
    if (blocked()) return false;
    return (mMouseButtonMask & mouseButtonToMask(button)) != 0;
}

bool Input::isKeyJustPressed(Key key) const
{
    if (blocked()) return false;
    const int index = static_cast<int>(key);
    if (index <= 0 || index >= static_cast<int>(Key::Max)) return false;
    return mKeyPressedFrame[index] == mProcessFrames;
}

bool Input::isKeyJustReleased(Key key) const
{
    if (blocked()) return false;
    const int index = static_cast<int>(key);
    if (index <= 0 || index >= static_cast<int>(Key::Max)) return false;
    return mKeyReleasedFrame[index] == mProcessFrames;
}

bool Input::isMouseButtonJustPressed(MouseButton button) const
{
    if (blocked()) return false;
    const int index = static_cast<int>(button);
    if (index < 0 || index >= static_cast<int>(MouseButton::Max)) return false;
    return mMouseButtonPressedFrame[index] == mProcessFrames;
}

bool Input::isMouseButtonJustReleased(MouseButton button) const
{
    if (blocked()) return false;
    const int index = static_cast<int>(button);
    if (index < 0 || index >= static_cast<int>(MouseButton::Max)) return false;
    return mMouseButtonReleasedFrame[index] == mProcessFrames;
}

Math::Vec2 Input::getMouseDelta() const
{
    return blocked() ? Math::Vec2() : mMouseDelta;
}

Math::Vec2 Input::getMouseWheel() const
{
    return blocked() ? Math::Vec2() : mMouseWheel;
}

const Input::ActionState* Input::findAction(const char* action, bool exact) const
{
    if (!mMap->hasAction(action))
    {
        log_error("input: unknown action \"%s\"", action);
        return nullptr;
    }
    if (blocked()) return nullptr;
    const ActionState* state = mActionStates.find(ct::String(action));
    if (!state) return nullptr;
    if (exact && !state->exact) return nullptr;
    return state;
}

bool Input::isActionPressed(const char* action, bool exact) const
{
    const ActionState* state = findAction(action, exact);
    return state && state->cache.pressed;
}

bool Input::isActionJustPressed(const char* action, bool exact) const
{
    const ActionState* state = findAction(action, exact);
    return state && state->pressedProcessFrame == mProcessFrames;
}

bool Input::isActionJustReleased(const char* action, bool exact) const
{
    const ActionState* state = findAction(action, exact);
    return state && state->releasedProcessFrame == mProcessFrames;
}

float Input::getActionStrength(const char* action, bool exact) const
{
    const ActionState* state = findAction(action, exact);
    return state ? state->cache.strength : 0.0f;
}

float Input::getActionRawStrength(const char* action, bool exact) const
{
    const ActionState* state = findAction(action, exact);
    return state ? state->cache.rawStrength : 0.0f;
}

float Input::getAxis(const char* negativeAction, const char* positiveAction) const
{
    return getActionStrength(positiveAction) - getActionStrength(negativeAction);
}

Math::Vec2 Input::getVector(const char* negativeX, const char* positiveX, const char* negativeY,
        const char* positiveY, float deadzone) const
{
    Math::Vec2 vector(getActionRawStrength(positiveX) - getActionRawStrength(negativeX),
            getActionRawStrength(positiveY) - getActionRawStrength(negativeY));

    if (deadzone < 0.0f)
    {
        deadzone = 0.25f *
                   (mMap->actionGetDeadzone(positiveX) + mMap->actionGetDeadzone(negativeX) +
                           mMap->actionGetDeadzone(positiveY) + mMap->actionGetDeadzone(negativeY));
    }

    const float length = vector.Length();
    if (length <= deadzone) return Math::Vec2();
    if (length > 1.0f) return vector / length;
    return vector * (inverseLerp(deadzone, 1.0f, length) / length);
}

void Input::updateActionCache(const char* action, ActionState& state)
{
    state.cache.pressed = false;
    state.cache.strength = 0.0f;
    state.cache.rawStrength = 0.0f;

    const ct::Vector<InputEvent>* events = mMap->actionGetEvents(action);
    const int maxEvent = (events ? static_cast<int>(events->size()) : 0) + 1;
    for (int i = 0; i < maxEvent && i < kMaxEvent; ++i)
    {
        state.cache.pressed = state.cache.pressed || state.pressed[i];
        state.cache.strength = maxOf(state.cache.strength, state.strength[i]);
        state.cache.rawStrength = maxOf(state.cache.rawStrength, state.rawStrength[i]);
    }

    if (state.apiPressed)
    {
        state.cache.pressed = true;
        state.cache.strength = maxOf(state.cache.strength, state.apiStrength);
        state.cache.rawStrength = maxOf(state.cache.rawStrength, state.apiStrength);
    }
}

void Input::actionPress(const char* action, float strength)
{
    if (!mMap->hasAction(action))
    {
        log_error("input: unknown action \"%s\"", action);
        return;
    }
    ActionState& state = mActionStates[ct::String(action)];
    if (!state.cache.pressed) state.pressedProcessFrame = mProcessFrames;
    state.exact = true;
    state.apiPressed = true;
    state.apiStrength = clampUnit(strength);
    updateActionCache(action, state);
}

void Input::actionRelease(const char* action)
{
    if (!mMap->hasAction(action))
    {
        log_error("input: unknown action \"%s\"", action);
        return;
    }
    ActionState& state = mActionStates[ct::String(action)];
    state.cache.pressed = false;
    state.cache.strength = 0.0f;
    state.cache.rawStrength = 0.0f;
    state.releasedProcessFrame = mProcessFrames;
    memset(state.pressed, 0, sizeof(state.pressed));
    memset(state.strength, 0, sizeof(state.strength));
    memset(state.rawStrength, 0, sizeof(state.rawStrength));
    state.exact = true;
    state.apiPressed = false;
    state.apiStrength = 0.0f;
}

Math::Vec2 Input::getMousePosition() const
{
    return mMousePos;
}

Math::Vec2 Input::getLastMouseVelocity()
{
    mMouseVelocityTrack.update(Math::Vec2());
    return mMouseVelocityTrack.velocity;
}

unsigned Input::getMouseButtonMask() const
{
    if (blocked()) return 0;
    return mMouseButtonMask;
}

void Input::warpMouse(const Math::Vec2& position)
{
    if (Device* device = Device::singleton())
        device->warpMouse(static_cast<int>(position.x), static_cast<int>(position.y));
}

void Input::setMousePosition(const Math::Vec2& position)
{
    mMousePos = position;
}

void Input::parseActions(const InputEvent& event)
{
    for (const auto& entry: mMap->getActionMap())
    {
        const char* name = entry.key.c_str();
        bool isPressed = false;
        float strength = 0.0f;
        float rawStrength = 0.0f;
        int eventIndex = -1;
        if (!mMap->eventGetActionStatus(event, name, false, &isPressed, &strength, &rawStrength,
                    &eventIndex))
            continue;
        if (eventIndex < 0 || eventIndex >= kMaxEvent)
        {
            log_error("input: action \"%s\" cannot take more than %d events", name, kMaxEvent);
            continue;
        }

        ActionState& state = mActionStates[entry.key];
        state.pressed[eventIndex] = isPressed;
        state.strength[eventIndex] = strength;
        state.rawStrength[eventIndex] = rawStrength;

        if (!isPressed)
        {
            state.apiPressed = false;
            state.apiStrength = 0.0f;
        }
        state.exact = mMap->eventIsAction(event, name, true);

        const bool wasPressed = state.cache.pressed;
        updateActionCache(name, state);
        if (state.cache.pressed && !wasPressed) state.pressedProcessFrame = mProcessFrames;
        if (!state.cache.pressed && wasPressed) state.releasedProcessFrame = mProcessFrames;
    }
}

void Input::parseInputEvent(const InputEvent& event)
{
    switch (event.type)
    {
        case InputEventType::Key:
        {
            const int index = static_cast<int>(event.key.key);
            if (!event.key.echo && index > 0 && index < static_cast<int>(Key::Max))
            {
                if (event.key.pressed && !mKeysPressed[index])
                    mKeyPressedFrame[index] = mProcessFrames;
                if (!event.key.pressed && mKeysPressed[index])
                    mKeyReleasedFrame[index] = mProcessFrames;
                mKeysPressed[index] = event.key.pressed;
            }
            break;
        }
        case InputEventType::MouseButton:
        {
            const int index = static_cast<int>(event.mouseButton.button);
            const unsigned mask = mouseButtonToMask(event.mouseButton.button);
            if (index >= 0 && index < static_cast<int>(MouseButton::Max))
            {
                if (event.mouseButton.pressed && (mMouseButtonMask & mask) == 0)
                    mMouseButtonPressedFrame[index] = mProcessFrames;
                if (!event.mouseButton.pressed && (mMouseButtonMask & mask) != 0)
                    mMouseButtonReleasedFrame[index] = mProcessFrames;
            }
            if (event.mouseButton.pressed) mMouseButtonMask |= mask;
            else
                mMouseButtonMask &= ~mask;
            const Math::Vec2 position(static_cast<float>(event.mouseButton.x),
                    static_cast<float>(event.mouseButton.y));
            if (mMousePos != position) setMousePosition(position);
            break;
        }
        case InputEventType::MouseMotion:
        {
            const Math::Vec2 position(static_cast<float>(event.mouseMotion.x),
                    static_cast<float>(event.mouseMotion.y));
            if (mMousePos != position) setMousePosition(position);
            const Math::Vec2 relative(static_cast<float>(event.mouseMotion.relativeX),
                    static_cast<float>(event.mouseMotion.relativeY));
            mMouseVelocityTrack.update(relative);
            mMouseDelta = mMouseDelta + relative;
            break;
        }
        case InputEventType::MouseWheel:
            mMouseWheel =
                    mMouseWheel + Math::Vec2(event.mouseWheel.deltaX, event.mouseWheel.deltaY);
            break;
        default:
            break;
    }

    parseActions(event);

    if (mDispatch && !mDisableInput) mDispatch(event, mDispatchUser);
}

void Input::releasePressedEvents()
{
    if (mApplicationFocused) return;
    memset(mKeysPressed, 0, sizeof(mKeysPressed));
    mMouseButtonMask = 0;
    for (auto& entry: mActionStates)
        if (entry.value.cache.pressed) actionRelease(entry.key.c_str());
}

void Input::setApplicationFocused(bool focused)
{
    mApplicationFocused = focused;
    if (!focused) releasePressedEvents();
}

bool Input::isApplicationFocused() const
{
    return mApplicationFocused;
}

void Input::setEventDispatchFunction(EventDispatchFunction function, void* user)
{
    mDispatch = function;
    mDispatchUser = user;
}

void Input::setDisableInput(bool disable)
{
    mDisableInput = disable;
}

bool Input::isInputDisabled() const
{
    return mDisableInput;
}

void Input::setPollingEnabled(bool enabled)
{
    mPollingEnabled = enabled;
}

bool Input::isPollingEnabled() const
{
    return mPollingEnabled;
}

bool Input::blocked() const
{
    return mDisableInput || !mPollingEnabled;
}

} // namespace prisma3d
